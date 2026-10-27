#!/usr/bin/env python3
"""Export complete job 690 golden segment details for the public browser.

Run with DATABASE_URL set, or pass --env-file pointing to a local env file.
The export is a reviewed snapshot; segment times come from the detail rows.
"""

import argparse
import json
import os
from collections import defaultdict
from pathlib import Path
from urllib.parse import urlparse

import psycopg


JOB_ID = 690
EDITOR_KEY = "benchmark_reference_editor_v1"
MAX_GAP_SECONDS = 0.15
OUTPUT = Path(__file__).resolve().parents[1] / "blog/shotwell-wgo-bench/episodes/data.js"
VIDEO_BASE_URL = ("https://huggingface.co/datasets/Nano1337/wgo-bench-videos/resolve/"
                  "3f3d01542aad6ed66332001f077ff1b208700ae7/videos")


def database_url(env_file):
    if os.environ.get("DATABASE_URL"):
        return os.environ["DATABASE_URL"]
    if env_file:
        for line in Path(env_file).read_text(encoding="utf-8").splitlines():
            if line.startswith("DATABASE_URL="):
                return line.split("=", 1)[1].strip().strip("\"'")
    raise SystemExit("DATABASE_URL is required (or pass --env-file)")


def details_by_decision(items, decision):
    return [item["text"].strip() for item in items or []
            if isinstance(item, dict) and item.get("decision") == decision
            and isinstance(item.get("text"), str) and item["text"].strip()]


def has_full_coverage(rows, duration):
    cursor = 0.0
    for row in sorted(rows, key=lambda item: (item["start"], item["end"])):
        if row["start"] > cursor + MAX_GAP_SECONDS:
            return False
        cursor = max(cursor, row["end"])
    return cursor >= duration - MAX_GAP_SECONDS


def export(conn):
    with conn.cursor() as cur:
        cur.execute("SELECT jobs_config -> %s FROM jobs WHERE id = %s", (EDITOR_KEY, JOB_ID))
        editor = cur.fetchone()[0]
        if not isinstance(editor, dict):
            raise SystemExit("Job 690 has no benchmark reference editor data")
        cur.execute("""
            SELECT episode_id, start_boundary_id, end_boundary_id,
                   start_time, end_time, generation_status, items
            FROM job690_segment_details
            WHERE episode_id = ANY(%s)
            ORDER BY episode_id, start_time, end_time
        """, (list(editor),))
        detail_rows = defaultdict(list)
        for episode_id, start_id, end_id, start, end, status, items in cur:
            detail_rows[episode_id].append({
                "startBoundaryId": start_id,
                "endBoundaryId": end_id,
                "start": start,
                "end": end,
                "status": status,
                "required": details_by_decision(items, "required"),
                "optional": details_by_decision(items, "optional"),
            })
        cur.execute("""
            SELECT episode_id, original_id, notes, video_duration
            FROM episodes WHERE episode_id = ANY(%s)
        """, (list(editor),))
        media = {row[0]: row[1:] for row in cur}

    episodes = []
    for episode_id in sorted(editor, key=lambda value: int(value.rsplit("_", 1)[-1])):
        entry = editor[episode_id]
        rows = detail_rows[episode_id]
        duration = float(entry["duration"])
        if entry.get("review_status") != "pass" or not rows:
            continue
        if any(row["status"] != "done" or not row["required"]
               or row["end"] <= row["start"] for row in rows):
            continue
        if not has_full_coverage(rows, duration):
            continue
        if episode_id not in media:
            continue
        original_id, notes, _ = media[episode_id]
        benchmark = (notes or {}).get("benchmark") or {}
        video_url = (benchmark.get("metadata") or {}).get("source_video_url")
        if not video_url or urlparse(video_url).scheme != "https":
            continue
        timelines = entry.get("timelines") or []
        if not timelines:
            continue
        boundaries = [{
            "id": boundary["id"],
            "time": boundary["time"],
            "minimum": boundary.get("minimum", boundary["time"]),
            "maximum": boundary.get("maximum", boundary["time"]),
            "type": boundary.get("boundary_type", "major"),
        } for boundary in timelines[0]["boundaries"]]
        episodes.append({
            "id": episode_id,
            "sourceId": original_id,
            "task": benchmark.get("instruction") or "",
            "duration": duration,
            "videoUrl": f"{VIDEO_BASE_URL}/{original_id}.mp4",
            "originalVideoUrl": video_url,
            "reviewedAt": entry.get("reviewed_at"),
            "boundaries": boundaries,
            "segments": [{key: value for key, value in row.items() if key != "status"}
                         for row in rows],
        })
    return {
        "sourceJobId": JOB_ID,
        "sourceDataset": "macrodata/WGO-Bench",
        "license": "CC BY-NC-SA 4.0",
        "episodes": episodes,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env-file", help="Local env file containing DATABASE_URL")
    args = parser.parse_args()
    with psycopg.connect(database_url(args.env_file), connect_timeout=10) as conn:
        data = export(conn)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text("window.SHOTWELL_WGO_REFERENCE = " +
                      json.dumps(data, ensure_ascii=True, separators=(",", ":")) + ";\n",
                      encoding="utf-8")
    print(f"Exported {len(data['episodes'])} complete episodes to {OUTPUT}")


if __name__ == "__main__":
    main()
