import { Router, type NextFunction, type Request, type Response } from "express";

import { adminStore } from "../admin/store.js";
import { readSessionCookie, setSessionCookie } from "../auth/cookies.js";
import { authStore, type User } from "../auth/store.js";
import { config } from "../config.js";
import { notifyContactSubmitted, notifyUploadSubmitted } from "../email/notifications.js";
import { adminPage, appPage, messagePage, resultPage } from "../http/render.js";
import { createGcsUploadSessions, createResultReadStream, createUploadReadStream, getResultFileMetadata, resultObjectName, type NewUploadFileInput } from "../uploads/gcs.js";
import { parseUploadResultJson } from "../uploads/results.js";
import { createUploadId, uploadStore } from "../uploads/store.js";

export const appRouter = Router();
const maxMediaChunkBytes = 8 * 1024 * 1024;
type StreamableUploadFile = {
  originalName: string;
  objectName: string;
  contentType: string;
  sizeBytes: number;
};

function loginUrl() {
  const url = new URL("/login", config.authBaseUrl);
  url.searchParams.set("return_to", new URL("/auth/complete", config.appBaseUrl).toString());
  return url.toString();
}

async function getAppUser(req: Request, res: Response): Promise<User | null> {
  const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

  if (!session) {
    res.redirect(302, loginUrl());
    return null;
  }

  const user = await authStore.getUser(session.userId);
  if (!user) {
    res.redirect(302, loginUrl());
    return null;
  }

  return user;
}

async function getApiUser(req: Request, res: Response): Promise<User | null> {
  const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

  if (!session) {
    res.status(401).json({ error: "unauthorized" });
    return null;
  }

  const user = await authStore.getUser(session.userId);
  if (!user) {
    res.status(401).json({ error: "unauthorized" });
    return null;
  }

  return user;
}

function isShotwellAdmin(user: User) {
  const [, domain = ""] = user.email.trim().toLowerCase().split("@");
  return domain === "shotwell.ai";
}

async function getAccessibleUpload(user: User, uploadId: string) {
  const ownedUpload = await uploadStore.getUserUpload(user.id, uploadId);
  if (ownedUpload) {
    return ownedUpload;
  }

  return isShotwellAdmin(user) ? adminStore.getUpload(uploadId) : null;
}

async function getAdminUser(req: Request, res: Response): Promise<User | null> {
  const user = await getAppUser(req, res);
  if (!user) {
    return null;
  }

  if (!isShotwellAdmin(user)) {
    res
      .status(403)
      .type("html")
      .send(messagePage("Admin access required", "Sign in with a shotwell.ai email address to view this page.", "/", "Back to app"));
    return null;
  }

  return user;
}

function isUuid(value: string | undefined) {
  return Boolean(value && /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(value));
}

function contentTypeForUploadFile(file: StreamableUploadFile) {
  const contentType = file.contentType.trim().toLowerCase();
  const name = file.originalName.toLowerCase();
  if (name.endsWith(".mov") && (contentType === "video/quicktime" || contentType === "application/octet-stream")) {
    return "video/mp4";
  }

  if (contentType && contentType !== "application/octet-stream") {
    return contentType;
  }

  if (name.endsWith(".mp4")) return "video/mp4";
  if (name.endsWith(".mov")) return "video/mp4";
  if (name.endsWith(".webm")) return "video/webm";
  if (name.endsWith(".m4v")) return "video/mp4";
  if (name.endsWith(".mp3")) return "audio/mpeg";
  if (name.endsWith(".wav")) return "audio/wav";
  if (name.endsWith(".m4a")) return "audio/mp4";
  if (name.endsWith(".png")) return "image/png";
  if (name.endsWith(".jpg") || name.endsWith(".jpeg")) return "image/jpeg";
  if (name.endsWith(".gif")) return "image/gif";
  if (name.endsWith(".webp")) return "image/webp";
  return file.contentType || "application/octet-stream";
}

function isStreamableMedia(contentType: string) {
  return contentType.startsWith("video/") || contentType.startsWith("audio/");
}

function defaultMediaRange(size: number) {
  if (size <= 0) {
    return undefined;
  }

  return {
    start: 0,
    end: Math.min(size - 1, maxMediaChunkBytes - 1)
  };
}

function parseRangeHeader(header: string | undefined, size: number) {
  if (!header || size <= 0) {
    return undefined;
  }

  const match = /^bytes=(\d*)-(\d*)$/.exec(header);
  if (!match) {
    return null;
  }

  const [, rawStart, rawEnd] = match;
  let start: number;
  let end: number;

  if (!rawStart && rawEnd) {
    const suffixLength = Number(rawEnd);
    if (!Number.isInteger(suffixLength) || suffixLength <= 0) {
      return null;
    }

    start = Math.max(0, size - suffixLength);
    end = size - 1;
  } else {
    start = Number(rawStart);
    end = rawEnd ? Number(rawEnd) : Math.min(size - 1, start + maxMediaChunkBytes - 1);
  }

  if (!Number.isInteger(start) || !Number.isInteger(end) || start < 0 || end < start || start >= size) {
    return null;
  }

  return {
    start,
    end: Math.min(end, size - 1)
  };
}

function safeDownloadName(fileName: string) {
  const asciiName = (fileName.trim() || "shotwell-upload")
    .replace(/[\r\n"]/g, "")
    .replace(/[^\x20-\x7E]+/g, "_")
    .replace(/[\\"]/g, "_");

  return asciiName || "shotwell-upload";
}

function contentDispositionInline(fileName: string) {
  const asciiName = safeDownloadName(fileName);
  const encodedName = encodeURIComponent(fileName.trim() || "shotwell-upload")
    .replace(/['()]/g, (char) => `%${char.charCodeAt(0).toString(16).toUpperCase()}`)
    .replace(/\*/g, "%2A");

  return `inline; filename="${asciiName}"; filename*=UTF-8''${encodedName}`;
}

function contentDispositionAttachment(fileName: string) {
  const asciiName = safeDownloadName(fileName);
  const encodedName = encodeURIComponent(fileName.trim() || "shotwell-results")
    .replace(/['()]/g, (char) => `%${char.charCodeAt(0).toString(16).toUpperCase()}`)
    .replace(/\*/g, "%2A");

  return `attachment; filename="${asciiName}"; filename*=UTF-8''${encodedName}`;
}

function streamUploadFile(file: StreamableUploadFile, req: Request, res: Response, next: NextFunction) {
  const contentType = contentTypeForUploadFile(file);
  const size = Math.max(0, Math.trunc(file.sizeBytes));
  const parsedRange = parseRangeHeader(req.headers.range, size);

  res.setHeader("Accept-Ranges", "bytes");
  res.setHeader("Content-Type", contentType);
  res.setHeader("Content-Disposition", contentDispositionInline(file.originalName));

  if (parsedRange === null) {
    res.status(416);
    if (size > 0) {
      res.setHeader("Content-Range", `bytes */${size}`);
    }
    res.end();
    return;
  }

  const range = parsedRange ?? (isStreamableMedia(contentType) ? defaultMediaRange(size) : undefined);
  if (range) {
    res.status(206);
    res.setHeader("Content-Range", `bytes ${range.start}-${range.end}/${size}`);
    res.setHeader("Content-Length", String(range.end - range.start + 1));
  } else if (size > 0) {
    res.setHeader("Content-Length", String(size));
  }

  const stream = createUploadReadStream(file.objectName, range ?? undefined);
  stream.on("error", next);
  stream.pipe(res);
}

function readStreamText(stream: NodeJS.ReadableStream): Promise<string> {
  return new Promise((resolve, reject) => {
    const chunks: Buffer[] = [];
    stream.on("data", (chunk: Buffer | string) => {
      chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk));
    });
    stream.on("error", reject);
    stream.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
  });
}

async function readUploadResult(resultObjectName: string) {
  const text = await readStreamText(createResultReadStream(resultObjectName));
  return parseUploadResultJson(text);
}

function cleanFileName(value: unknown) {
  if (typeof value !== "string") {
    return "";
  }

  return value.trim().slice(0, 512);
}

function parseSourceUrl(value: unknown) {
  if (typeof value !== "string") {
    return undefined;
  }

  const sourceUrl = value.trim().slice(0, 2_048);
  if (!sourceUrl) {
    return undefined;
  }

  let parsedUrl: URL;
  try {
    parsedUrl = new URL(sourceUrl);
  } catch {
    throw new Error("Enter a valid URL.");
  }

  if (!["http:", "https:", "s3:", "gs:"].includes(parsedUrl.protocol)) {
    throw new Error("Enter a valid URL.");
  }

  return parsedUrl.toString();
}

function parseUploadRequest(body: unknown): { prompt: string; files: NewUploadFileInput[]; sourceUrl?: string } {
  if (!body || typeof body !== "object") {
    throw new Error("Invalid upload request.");
  }

  const input = body as { prompt?: unknown; files?: unknown; sourceUrl?: unknown };
  const filesInput = Array.isArray(input.files) ? input.files : [];
  const files = filesInput.map((file) => {
    const item = file && typeof file === "object"
      ? file as { name?: unknown; type?: unknown; size?: unknown }
      : {};
    const originalName = cleanFileName(item.name);

    if (!originalName) {
      throw new Error("Every file needs a name.");
    }

    return {
      originalName,
      contentType: typeof item.type === "string" && item.type.trim() ? item.type.trim().slice(0, 200) : undefined,
      sizeBytes: typeof item.size === "number" && Number.isFinite(item.size) ? item.size : undefined
    };
  });
  const sourceUrl = parseSourceUrl(input.sourceUrl);

  if (files.length === 0 && !sourceUrl) {
    throw new Error("Choose files or provide a link.");
  }

  if (files.length > 0 && sourceUrl) {
    throw new Error("Choose files or provide a link, not both.");
  }

  if (files.length > 100) {
    throw new Error("Upload at most 100 files at a time.");
  }

  return {
    prompt: typeof input.prompt === "string" ? input.prompt.trim().slice(0, 20_000) : "",
    files,
    sourceUrl
  };
}

function parsePromptInput(body: unknown) {
  if (!body || typeof body !== "object") {
    return "";
  }

  const input = body as { prompt?: unknown };
  return typeof input.prompt === "string" ? input.prompt.trim().slice(0, 20_000) : "";
}

function appScript() {
  return `"use strict";

function setUploadMessage(element, message, isError) {
  if (!element) return;
  element.hidden = false;
  element.textContent = message;
  element.classList.toggle("neutral", !isError);
}

function showUploadLimitDialog(dialog, status, contactUrl) {
  if (!dialog) return;
  const statusLabel = status === "completed" ? "complete" : "pending";
  const message = dialog.querySelector("[data-upload-limit-message]");
  const link = dialog.querySelector("[data-upload-limit-contact]");
  if (message) {
    message.textContent = "You already have one upload " + statusLabel + ". ";
  }
  if (link) {
    link.href = contactUrl || "https://shotwell.ai/contact/";
    link.textContent = "Contact us for more annotations!";
  }
  if (typeof dialog.showModal === "function") {
    dialog.showModal();
  } else {
    dialog.hidden = false;
  }
}

async function requestUploadSession(prompt, files, sourceUrl) {
  const response = await fetch("/api/uploads", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      prompt,
      sourceUrl,
      files: files.map((file) => ({
        name: file.name,
        type: file.type || "application/octet-stream",
        size: file.size
      }))
    })
  });

  if (!response.ok) {
    const payload = await response.json().catch(() => ({}));
    if (response.status === 409 && payload.code === "UPLOAD_LIMIT") {
      const error = new Error(payload.error || "You already have one upload.");
      error.name = "UploadLimitError";
      error.uploadStatus = payload.status;
      error.contactUrl = payload.contactUrl;
      throw error;
    }
    throw new Error(payload.error || "Could not start upload.");
  }

  return response.json();
}

async function uploadFileToGcs(session, file) {
  const response = await fetch(session.uploadUrl, {
    method: "PUT",
    headers: { "Content-Type": session.contentType || file.type || "application/octet-stream" },
    body: file
  });

  if (!response.ok) {
    throw new Error("Upload failed for " + file.name + ".");
  }
}

async function markUploadComplete(uploadId) {
  await fetch("/api/uploads/" + encodeURIComponent(uploadId) + "/uploaded", {
    method: "POST"
  });
}

document.addEventListener("DOMContentLoaded", () => {
  const form = document.querySelector("[data-upload-form]");
  if (!form) return;

  const fileInput = form.querySelector("[data-upload-files]");
  const sourceUrlInput = form.querySelector("[data-upload-source-url]");
  const promptInput = form.querySelector("[data-upload-prompt]");
  const submit = form.querySelector("[data-upload-submit]");
  const message = form.querySelector("[data-upload-message]");
  const uploadLimitDialog = document.querySelector("[data-upload-limit-dialog]");
  const uploadLimitClose = uploadLimitDialog?.querySelector("[data-upload-limit-close]");
  const tabs = Array.from(form.querySelectorAll("[data-upload-tab]"));
  const panels = Array.from(form.querySelectorAll("[data-upload-panel]"));

  function setUploadMode(mode) {
    form.dataset.uploadMode = mode;
    for (const tab of tabs) {
      const isActive = tab.dataset.uploadTab === mode;
      tab.setAttribute("aria-selected", String(isActive));
    }

    for (const panel of panels) {
      panel.hidden = panel.dataset.uploadPanel !== mode;
    }

    if (fileInput) {
      fileInput.disabled = mode !== "files";
      if (mode !== "files") {
        fileInput.value = "";
      }
    }

    if (sourceUrlInput) {
      sourceUrlInput.disabled = mode !== "url";
      if (mode !== "url") {
        sourceUrlInput.value = "";
      }
    }
  }

  for (const tab of tabs) {
    tab.addEventListener("click", () => {
      setUploadMode(tab.dataset.uploadTab || "files");
    });
  }

  setUploadMode("files");

  uploadLimitClose?.addEventListener("click", () => {
    if (typeof uploadLimitDialog.close === "function") {
      uploadLimitDialog.close();
    } else {
      uploadLimitDialog.hidden = true;
    }
  });

  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    const uploadMode = form.dataset.uploadMode || "files";
    const files = uploadMode === "files" ? Array.from(fileInput.files || []) : [];
    const sourceUrl = uploadMode === "url" ? (sourceUrlInput ? sourceUrlInput.value : "").trim() : "";

    if (uploadMode === "files" && files.length === 0) {
      setUploadMessage(message, "Choose at least one file to upload.", true);
      return;
    }

    if (uploadMode === "url" && !sourceUrl) {
      setUploadMessage(message, "Enter a URL to episodes.", true);
      return;
    }

    if (files.length > 0 && sourceUrl) {
      setUploadMessage(message, "Choose files or provide a link, not both.", true);
      return;
    }

    submit.disabled = true;
    setUploadMessage(message, "Preparing upload...", false);

    try {
      const payload = await requestUploadSession(promptInput.value || "", files, sourceUrl);
      for (let index = 0; index < files.length; index += 1) {
        setUploadMessage(message, "Uploading " + (index + 1) + " of " + files.length + "...", false);
        await uploadFileToGcs(payload.files[index], files[index]);
      }

      if (files.length > 0) {
        await markUploadComplete(payload.upload.id);
      }

      setUploadMessage(message, sourceUrl ? "Link received. Refreshing..." : "Upload received. Refreshing...", false);
      window.location.reload();
    } catch (error) {
      if (error instanceof Error && error.name === "UploadLimitError") {
        showUploadLimitDialog(uploadLimitDialog, error.uploadStatus, error.contactUrl);
        submit.disabled = false;
        return;
      }
      setUploadMessage(message, error instanceof Error ? error.message : "Upload failed.", true);
      submit.disabled = false;
    }
  });
});

document.addEventListener("DOMContentLoaded", () => {
  const viewer = document.querySelector("[data-result-viewer]");
  if (!viewer) return;

  const navItems = Array.from(viewer.querySelectorAll("[data-result-nav]"));
  const panels = Array.from(viewer.querySelectorAll("[data-result-panel]"));

  function notifyVisible(panel) {
    const notify = () => panel.dispatchEvent(new CustomEvent("result:visible", { bubbles: true }));
    window.requestAnimationFrame(() => {
      notify();
      window.requestAnimationFrame(notify);
    });
  }

  function showEpisode(id) {
    for (const nav of navItems) {
      const isActive = nav.dataset.resultNav === id;
      nav.setAttribute("aria-current", isActive ? "true" : "false");
    }

    for (const panel of panels) {
      const isActive = panel.dataset.resultPanel === id;
      panel.hidden = !isActive;
      if (!isActive) {
        for (const video of panel.querySelectorAll("video")) {
          video.pause();
        }
      } else {
        notifyVisible(panel);
      }
    }
  }

  for (const nav of navItems) {
    nav.addEventListener("click", () => {
      const id = nav.dataset.resultNav;
      if (!id) return;
      showEpisode(id);
      if (window.history && window.history.replaceState) {
        window.history.replaceState(null, "", "#" + id);
      }
    });
  }

  const requestedId = window.location.hash ? window.location.hash.slice(1) : "";
  const initial = panels.find((panel) => panel.dataset.resultPanel === requestedId) || panels[0];
  if (initial && initial.dataset.resultPanel) {
    showEpisode(initial.dataset.resultPanel);
  }
});

document.addEventListener("DOMContentLoaded", () => {
  const demos = Array.from(document.querySelectorAll("[data-result-demo]"));
  if (demos.length === 0) return;

  const prefersReducedMotion = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  function pad(value) {
    return value < 10 ? "0" + value : String(value);
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function formatTime(value) {
    const safe = Math.max(0, Number(value) || 0);
    const minutes = Math.floor(safe / 60);
    const seconds = safe - minutes * 60;
    if (Math.abs(seconds - Math.round(seconds)) < 0.05) {
      return minutes + ":" + pad(Math.round(seconds));
    }

    const fixed = seconds.toFixed(1);
    return minutes + ":" + (seconds < 10 ? "0" + fixed : fixed);
  }

  function actionState(action, time) {
    if (time < action.start) return "pending";
    if (time < action.end) return "active";
    return "done";
  }

  function parseActions(demo) {
    try {
      const parsed = JSON.parse(demo.dataset.actions || "[]");
      if (!Array.isArray(parsed)) return [];
      return parsed.map((action, index) => {
        const start = Math.max(0, Number(action.start) || 0);
        const end = Math.max(start + 0.001, Number(action.end) || start + 0.001);
        return {
          label: String(action.label || "Annotation " + (index + 1)),
          start,
          end,
          color: String(action.color || "#6E6E78")
        };
      });
    } catch (_error) {
      return [];
    }
  }

  function initResultDemo(demo) {
    const actions = parseActions(demo);
    const video = demo.querySelector("[data-result-video]");
    const track = demo.querySelector("[data-result-track]");
    const zoomTrack = demo.querySelector("[data-result-zoom-track]");
    const cards = demo.querySelector("[data-result-cards]");
    const tip = demo.querySelector("[data-result-tip]");
    const tipName = demo.querySelector("[data-result-tip-name]");
    const clock = demo.querySelector("[data-result-clock]");
    const playToggle = demo.querySelector("[data-result-play-toggle]");
    const isStatic = demo.dataset.static === "true" || !video;

    if (!track || !cards) return;

    const maxActionEnd = Math.max(1, ...actions.map((action) => action.end));
    let duration = maxActionEnd;
    const segmentEls = [];
    const rowEls = [];
    let playhead = null;
    let zoomPlayhead = null;
    let isScrubbing = false;
    let manuallyPaused = true;
    let frameRequest = 0;
    let windowStart = 0;
    let windowDuration = 1;

    function mediaDuration() {
      if (!video || Number.isNaN(video.duration) || video.duration <= 0) {
        return maxActionEnd;
      }

      return Math.max(maxActionEnd, video.duration);
    }

    function focusedWindowDuration() {
      if (duration <= 75) return duration;
      return Math.min(duration, actions.length > 80 ? 30 : 45);
    }

    function focusedWindowStart(time) {
      const size = focusedWindowDuration();
      return clamp(time - size / 2, 0, Math.max(0, duration - size));
    }

    function build() {
      track.innerHTML = "";
      if (zoomTrack) zoomTrack.innerHTML = "";
      cards.innerHTML = "";
      segmentEls.length = 0;
      rowEls.length = 0;

      actions.forEach((action, index) => {
        const segment = document.createElement("div");
        segment.className = "result-seg";
        segment.style.setProperty("--seg-color", action.color);
        const fill = document.createElement("div");
        fill.className = "result-seg-fill";
        fill.style.background = action.color;
        segment.appendChild(fill);
        track.appendChild(segment);

        let zoomSegment = null;
        let zoomFill = null;
        if (zoomTrack) {
          zoomSegment = document.createElement("div");
          zoomSegment.className = "result-seg";
          zoomSegment.style.setProperty("--seg-color", action.color);
          zoomFill = document.createElement("div");
          zoomFill.className = "result-seg-fill";
          zoomFill.style.background = action.color;
          zoomSegment.appendChild(zoomFill);
          zoomTrack.appendChild(zoomSegment);
        }

        segmentEls.push({ root: segment, fill, zoomRoot: zoomSegment, zoomFill });

        const row = document.createElement("li");
        row.className = "result-card-row";
        row.style.borderColor = "transparent";
        const name = document.createElement("b");
        name.textContent = action.label;
        const range = document.createElement("em");
        range.textContent = "[" + formatTime(action.start) + " - " + formatTime(action.end) + "]";
        row.append(name, range);
        row.addEventListener("click", () => seekTo(action.start + 0.01));
        cards.appendChild(row);
        rowEls.push(row);
      });

      playhead = document.createElement("div");
      playhead.className = "result-playhead";
      track.appendChild(playhead);

      if (zoomTrack) {
        zoomPlayhead = document.createElement("div");
        zoomPlayhead.className = "result-playhead";
        zoomTrack.appendChild(zoomPlayhead);
      }

      if (actions.length === 0) {
        const row = document.createElement("li");
        row.className = "result-card-row";
        row.textContent = "No annotations in this episode.";
        cards.appendChild(row);
        if (tip) tip.hidden = true;
      }
    }

    function layout() {
      duration = mediaDuration();
      track.setAttribute("aria-valuemax", duration.toFixed(2));
      if (zoomTrack) zoomTrack.setAttribute("aria-valuemax", duration.toFixed(2));

      actions.forEach((action, index) => {
        const startPct = action.start / duration * 100;
        const widthPct = Math.max(0.25, (action.end - action.start) / duration * 100);
        const segment = segmentEls[index];
        segment.root.style.left = startPct.toFixed(3) + "%";
        segment.root.style.width = widthPct.toFixed(3) + "%";
      });

      update(video ? video.currentTime || 0 : 0);
    }

    function activeIndexAt(time) {
      const active = actions.findIndex((action) => time >= action.start && time < action.end);
      if (active !== -1) return active;
      for (let index = actions.length - 1; index >= 0; index -= 1) {
        if (time >= actions[index].end) return index;
      }
      return actions.length > 0 ? 0 : -1;
    }

    function update(time) {
      const safeTime = clamp(time, 0, duration);
      const ratio = duration > 0 ? safeTime / duration : 0;
      if (playhead) playhead.style.setProperty("--pos", ratio.toFixed(4));
      windowDuration = focusedWindowDuration();
      windowStart = focusedWindowStart(safeTime);
      const windowEnd = windowStart + windowDuration;
      const windowRatio = windowDuration > 0 ? (safeTime - windowStart) / windowDuration : 0;
      if (zoomPlayhead) zoomPlayhead.style.setProperty("--pos", clamp(windowRatio, 0, 1).toFixed(4));
      track.setAttribute("aria-valuenow", safeTime.toFixed(2));
      track.setAttribute("aria-valuetext", formatTime(safeTime));
      if (zoomTrack) {
        zoomTrack.setAttribute("aria-valuenow", safeTime.toFixed(2));
        zoomTrack.setAttribute("aria-valuetext", formatTime(safeTime));
      }
      if (clock) clock.textContent = formatTime(safeTime);

      actions.forEach((action, index) => {
        const state = actionState(action, safeTime);
        const fill = state === "active"
          ? (safeTime - action.start) / (action.end - action.start)
          : state === "done" ? 1 : 0;
        segmentEls[index].root.dataset.state = state;
        segmentEls[index].fill.style.transform = "scaleX(" + clamp(fill, 0, 1).toFixed(3) + ")";
        rowEls[index].dataset.active = String(state === "active");
        rowEls[index].style.borderColor = state === "active" ? action.color : "";

        const zoomRoot = segmentEls[index].zoomRoot;
        const zoomFill = segmentEls[index].zoomFill;
        if (zoomRoot && zoomFill) {
          const visibleStart = Math.max(action.start, windowStart);
          const visibleEnd = Math.min(action.end, windowEnd);
          const isVisible = visibleEnd > visibleStart;
          zoomRoot.hidden = !isVisible;
          if (isVisible) {
            const startPct = (visibleStart - windowStart) / windowDuration * 100;
            const widthPct = Math.max(0.35, (visibleEnd - visibleStart) / windowDuration * 100);
            zoomRoot.style.left = startPct.toFixed(3) + "%";
            zoomRoot.style.width = widthPct.toFixed(3) + "%";
            zoomRoot.dataset.state = state;
            zoomFill.style.transform = "scaleX(" + clamp(fill, 0, 1).toFixed(3) + ")";
          }
        }
      });

      const activeIndex = activeIndexAt(safeTime);
      const active = activeIndex >= 0 ? actions[activeIndex] : null;
      if (tip && tipName && active) {
        const tipTrack = zoomTrack || track;
        const trackWidth = tipTrack.getBoundingClientRect().width;
        const centerTime = clamp((active.start + active.end) / 2, windowStart, windowEnd);
        const center = zoomTrack
          ? (centerTime - windowStart) / windowDuration * trackWidth
          : centerTime / duration * trackWidth;
        tip.hidden = false;
        tip.style.left = clamp(center, 70, Math.max(70, trackWidth - 70)) + "px";
        tip.style.setProperty("--result-tip-color", active.color);
        tipName.textContent = active.label;
      } else if (tip) {
        tip.hidden = true;
      }
    }

    function pointerTime(event, pointerTrack) {
      const rect = pointerTrack.getBoundingClientRect();
      const ratio = rect.width ? (event.clientX - rect.left) / rect.width : 0;
      if (pointerTrack === zoomTrack) {
        return windowStart + clamp(ratio, 0, 1) * windowDuration;
      }
      return clamp(ratio, 0, 1) * duration;
    }

    function seekTo(time) {
      const next = clamp(time, 0, duration);
      if (video) video.currentTime = next;
      update(next);
    }

    function startScrub(event) {
      if (event.button != null && event.button !== 0) return;
      event.preventDefault();
      isScrubbing = true;
      const targetTrack = event.currentTarget;
      targetTrack.classList.add("is-dragging");
      if (targetTrack.setPointerCapture && event.pointerId != null) targetTrack.setPointerCapture(event.pointerId);
      seekTo(pointerTime(event, targetTrack));
    }

    function moveScrub(event) {
      if (!isScrubbing) return;
      event.preventDefault();
      seekTo(pointerTime(event, event.currentTarget));
    }

    function endScrub(event) {
      if (!isScrubbing) return;
      isScrubbing = false;
      const targetTrack = event.currentTarget;
      targetTrack.classList.remove("is-dragging");
      if (targetTrack.releasePointerCapture && event.pointerId != null) {
        try { targetTrack.releasePointerCapture(event.pointerId); } catch (_error) {}
      }
    }

    function updatePlayToggle() {
      if (!playToggle) return;
      playToggle.textContent = video && !video.paused && !video.ended ? "Pause" : "Play";
    }

    function isVisibleDemo() {
      return !demo.closest("[hidden]");
    }

    function isKeyboardEditingTarget(target) {
      return target && target.closest && Boolean(target.closest("input, textarea, select, button, a, [contenteditable='true']"));
    }

    function startFrameLoop() {
      if (!video || frameRequest) return;
      const tick = () => {
        frameRequest = 0;
        if (!video || video.paused || video.ended || isScrubbing) return;
        update(video.currentTime || 0);
        startFrameLoop();
      };
      frameRequest = window.requestAnimationFrame(tick);
    }

    function stopFrameLoop() {
      if (!frameRequest) return;
      window.cancelAnimationFrame(frameRequest);
      frameRequest = 0;
    }

    function playVideo() {
      if (!video || prefersReducedMotion) return;
      manuallyPaused = false;
      video.muted = true;
      const promise = video.play();
      if (promise && promise.catch) promise.catch(() => {});
    }

    function pauseVideo() {
      if (!video) return;
      video.pause();
    }

    function togglePlayback() {
      if (!video) return;
      if (video.ended) video.currentTime = 0;
      if (video.paused) {
        playVideo();
      } else {
        manuallyPaused = true;
        pauseVideo();
      }
    }

    function keyboardSeek(delta) {
      const current = video ? video.currentTime || 0 : 0;
      seekTo(current + delta);
    }

    build();
    layout();

    track.addEventListener("pointerdown", startScrub);
    track.addEventListener("pointermove", moveScrub);
    track.addEventListener("pointerup", endScrub);
    track.addEventListener("pointercancel", endScrub);

    if (zoomTrack) {
      zoomTrack.addEventListener("pointerdown", startScrub);
      zoomTrack.addEventListener("pointermove", moveScrub);
      zoomTrack.addEventListener("pointerup", endScrub);
      zoomTrack.addEventListener("pointercancel", endScrub);
    }

    function handleTrackKeydown(event) {
      const current = video ? video.currentTime || 0 : 0;
      const step = event.shiftKey ? 5 : 1;
      let next = null;
      if (event.key === "ArrowLeft") next = current - step;
      if (event.key === "ArrowRight") next = current + step;
      if (event.key === "Home") next = 0;
      if (event.key === "End") next = duration;
      if (event.key === " ") {
        event.preventDefault();
        togglePlayback();
        return;
      }
      if (next == null) return;
      event.preventDefault();
      seekTo(next);
    }

    track.addEventListener("keydown", handleTrackKeydown);
    if (zoomTrack) zoomTrack.addEventListener("keydown", handleTrackKeydown);

    document.addEventListener("keydown", (event) => {
      if (event.defaultPrevented || !isVisibleDemo() || isKeyboardEditingTarget(event.target)) return;
      const step = event.shiftKey ? 5 : 1;
      if (event.key === " ") {
        event.preventDefault();
        togglePlayback();
      } else if (event.key === "ArrowLeft") {
        event.preventDefault();
        keyboardSeek(-step);
      } else if (event.key === "ArrowRight") {
        event.preventDefault();
        keyboardSeek(step);
      }
    });

    if (playToggle && !isStatic) {
      playToggle.addEventListener("click", togglePlayback);
    }

    if (video) {
      video.addEventListener("loadedmetadata", layout);
      video.addEventListener("durationchange", layout);
      video.addEventListener("timeupdate", () => update(video.currentTime || 0));
      video.addEventListener("play", () => {
        updatePlayToggle();
        startFrameLoop();
      });
      video.addEventListener("pause", () => {
        stopFrameLoop();
        updatePlayToggle();
        update(video.currentTime || 0);
      });
      video.addEventListener("ended", () => {
        stopFrameLoop();
        updatePlayToggle();
        update(video.currentTime || 0);
      });
      video.addEventListener("click", togglePlayback);

      if ("IntersectionObserver" in window && !prefersReducedMotion) {
        const observer = new IntersectionObserver((entries) => {
          const entry = entries[entries.length - 1];
          if (entry.isIntersecting && entry.intersectionRatio >= 0.45 && !manuallyPaused && !demo.closest("[hidden]")) {
            playVideo();
          } else if (!entry.isIntersecting || entry.intersectionRatio <= 0.1) {
            pauseVideo();
          }
        }, { threshold: [0, 0.1, 0.45, 1] });
        observer.observe(demo);
      }
    }

    demo.addEventListener("result:visible", () => {
      layout();
      if (video && !manuallyPaused) playVideo();
    });

    if ("ResizeObserver" in window) {
      const resizeObserver = new ResizeObserver(layout);
      resizeObserver.observe(track);
    } else {
      window.addEventListener("resize", layout);
    }

    updatePlayToggle();
  }

  demos.forEach(initResultDemo);
});
`;
}

appRouter.get("/", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const uploads = await uploadStore.listUserUploads(user.id);
    res.type("html").send(appPage(user, uploads));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/app.js", (_req, res) => {
  res.type("application/javascript").send(appScript());
});

// ── public contact endpoint ──
// The marketing site's "Talk to us" form posts here; submissions are
// forwarded to the founders through the admin-notification pipeline.

const contactAllowedOrigins = new Set(
  [
    config.publicSiteUrl.origin,
    "https://shotwell.ai",
    "https://www.shotwell.ai",
    ...(config.isProduction ? [] : ["http://localhost:8099", "http://127.0.0.1:8099"])
  ]
);

function setContactCors(req: Request, res: Response) {
  const origin = req.headers.origin;
  if (origin && contactAllowedOrigins.has(origin)) {
    res.setHeader("Access-Control-Allow-Origin", origin);
    res.setHeader("Vary", "Origin");
    res.setHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type");
    res.setHeader("Access-Control-Max-Age", "86400");
  }
}

const contactHits = new Map<string, number[]>();
const contactRateLimit = { windowMs: 60 * 60 * 1000, max: 5 };

function contactRateLimited(ip: string) {
  const now = Date.now();
  const hits = (contactHits.get(ip) ?? []).filter((t) => now - t < contactRateLimit.windowMs);
  if (hits.length >= contactRateLimit.max) {
    contactHits.set(ip, hits);
    return true;
  }
  hits.push(now);
  contactHits.set(ip, hits);
  return false;
}

function cleanContactField(value: unknown, maxLength: number) {
  return typeof value === "string" ? value.trim().slice(0, maxLength) : "";
}

appRouter.options("/api/contact", (req, res) => {
  setContactCors(req, res);
  res.sendStatus(204);
});

appRouter.post("/api/contact", async (req, res, next) => {
  try {
    setContactCors(req, res);

    const body = (req.body ?? {}) as Record<string, unknown>;

    // honeypot: real visitors never fill this
    if (cleanContactField(body.website, 100)) {
      res.json({ ok: true });
      return;
    }

    if (contactRateLimited(req.ip ?? "unknown")) {
      res.status(429).json({ error: "Too many requests. Email hello@shotwell.ai instead." });
      return;
    }

    const submission = {
      name: cleanContactField(body.name, 200),
      company: cleanContactField(body.company, 200),
      email: cleanContactField(body.email, 320),
      message: cleanContactField(body.message, 5_000)
    };

    if (!submission.name || !submission.company || !submission.message
      || !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(submission.email)) {
      res.status(400).json({ error: "Fill in your name, company, email, and message." });
      return;
    }

    await notifyContactSubmitted(submission);
    res.json({ ok: true });
  } catch (error) {
    next(error);
  }
});

appRouter.get("/admin", async (req, res, next) => {
  try {
    const user = await getAdminUser(req, res);
    if (!user) {
      return;
    }

    const dashboard = await adminStore.getDashboard();
    res.type("html").send(adminPage(user, dashboard));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/admin/users/:userId", async (req, res, next) => {
  try {
    const user = await getAdminUser(req, res);
    if (!user) {
      return;
    }

    if (!isUuid(req.params.userId)) {
      res
        .status(404)
        .type("html")
        .send(messagePage("User not found", "That account could not be found.", "/admin", "Back to admin"));
      return;
    }

    const dashboard = await adminStore.getDashboard({ userId: req.params.userId });
    if (!dashboard.selectedUser) {
      res
        .status(404)
        .type("html")
        .send(messagePage("User not found", "That account could not be found.", "/admin", "Back to admin"));
      return;
    }

    res.type("html").send(adminPage(user, dashboard));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/admin/uploads/:uploadId/files/:fileId", async (req, res, next) => {
  try {
    const user = await getAdminUser(req, res);
    if (!user) {
      return;
    }

    if (!isUuid(req.params.uploadId) || !isUuid(req.params.fileId)) {
      res.status(404).type("html").send(messagePage("File not found", "That upload file could not be found.", "/admin", "Back to admin"));
      return;
    }

    const file = await adminStore.getUploadFile(req.params.uploadId, req.params.fileId);
    if (!file) {
      res.status(404).type("html").send(messagePage("File not found", "That upload file could not be found.", "/admin", "Back to admin"));
      return;
    }

    streamUploadFile(file, req, res, next);
  } catch (error) {
    next(error);
  }
});

appRouter.get("/admin/uploads/:uploadId/result", async (req, res, next) => {
  try {
    const user = await getAdminUser(req, res);
    if (!user) {
      return;
    }

    res.redirect(308, `/uploads/${encodeURIComponent(req.params.uploadId)}/result`);
  } catch (error) {
    next(error);
  }
});

appRouter.get("/admin/uploads/:uploadId/results", async (req, res, next) => {
  try {
    const user = await getAdminUser(req, res);
    if (!user) {
      return;
    }

    res.redirect(308, `/uploads/${encodeURIComponent(req.params.uploadId)}/results`);
  } catch (error) {
    next(error);
  }
});

appRouter.post("/api/uploads", async (req, res, next) => {
  try {
    const user = await getApiUser(req, res);
    if (!user) {
      return;
    }

    const existingUploads = await uploadStore.listUserUploads(user.id);
    if (existingUploads.length > 0) {
      const status = existingUploads[0].status;
      res.status(409).json({
        code: "UPLOAD_LIMIT",
        status,
        error: `You already have one upload ${status === "completed" ? "complete" : "pending"}. Contact us for more annotations!`,
        contactUrl: new URL("/contact/", config.publicSiteUrl).toString()
      });
      return;
    }

    const input = parseUploadRequest(req.body);
    const uploadId = createUploadId();
    const origin = new URL(config.appBaseUrl).origin;
    const sessions = input.files.length > 0
      ? await createGcsUploadSessions({
        userId: user.id,
        uploadId,
        files: input.files,
        origin
      })
      : [];
    const uploadPrefix = [
      config.uploads.uploadPrefix,
      user.id,
      uploadId
    ].filter(Boolean).join("/");

    const upload = await uploadStore.createUpload({
      id: uploadId,
      userId: user.id,
      prompt: input.prompt,
      sourceUrl: input.sourceUrl,
      uploadPrefix,
      files: sessions.map((file) => ({
        id: file.id,
        originalName: file.originalName,
        objectName: file.objectName,
        contentType: file.contentType,
        sizeBytes: file.sizeBytes
      }))
    });

    if (upload.sourceUrl) {
      await notifyUploadSubmitted(user, upload);
    }

    res.json({
      upload: {
        id: upload.id,
        status: upload.status
      },
      files: sessions.map((file) => ({
        id: file.id,
        originalName: file.originalName,
        contentType: file.contentType,
        uploadUrl: file.uploadUrl
      }))
    });
  } catch (error) {
    if (error instanceof Error && [
      "Choose files or provide a link.",
      "Choose files or provide a link, not both.",
      "Enter a valid URL.",
      "Every file needs a name.",
      "Upload at most 100 files at a time."
    ].includes(error.message)) {
      res.status(400).json({ error: error.message });
      return;
    }

    next(error);
  }
});

appRouter.post("/api/uploads/:uploadId/uploaded", async (req, res, next) => {
  try {
    const user = await getApiUser(req, res);
    if (!user) {
      return;
    }

    const existingUpload = await uploadStore.getUserUpload(user.id, req.params.uploadId);
    const shouldNotify = Boolean(existingUpload && existingUpload.files.length > 0 && !existingUpload.filesUploadedAt);

    await uploadStore.markFilesUploaded(user.id, req.params.uploadId);

    if (shouldNotify) {
      const upload = await uploadStore.getUserUpload(user.id, req.params.uploadId);
      await notifyUploadSubmitted(user, upload ?? existingUpload!);
    }

    res.json({ ok: true });
  } catch (error) {
    next(error);
  }
});

appRouter.get("/uploads/:uploadId/files/:fileId", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const upload = await getAccessibleUpload(user, req.params.uploadId);
    const file = upload?.files.find((item) => item.id === req.params.fileId);
    if (!upload || !file) {
      res
        .status(404)
        .type("html")
        .send(messagePage("File not found", "That upload file could not be found.", "/", "Back to app"));
      return;
    }

    streamUploadFile(file, req, res, next);
  } catch (error) {
    next(error);
  }
});

appRouter.get("/uploads/:uploadId/result-preview", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const upload = await getAccessibleUpload(user, req.params.uploadId);
    if (!upload || upload.status !== "completed" || !upload.resultObjectName) {
      res.status(404).type("text").send("Preview not found.");
      return;
    }

    const objectName = resultObjectName(upload.id, "preview.mp4");
    let metadata;
    try {
      metadata = await getResultFileMetadata(objectName);
    } catch {
      res.status(404).type("text").send("Preview not found.");
      return;
    }

    const size = Math.max(0, Math.trunc(metadata.sizeBytes));
    const parsedRange = parseRangeHeader(req.headers.range, size);
    res.setHeader("Accept-Ranges", "bytes");
    res.setHeader("Content-Type", "video/mp4");
    res.setHeader("Content-Disposition", contentDispositionInline("preview.mp4"));

    if (parsedRange === null) {
      res.status(416).setHeader("Content-Range", `bytes */${size}`).end();
      return;
    }

    const range = parsedRange ?? defaultMediaRange(size);
    if (range) {
      res.status(206);
      res.setHeader("Content-Range", `bytes ${range.start}-${range.end}/${size}`);
      res.setHeader("Content-Length", String(range.end - range.start + 1));
    } else if (size > 0) {
      res.setHeader("Content-Length", String(size));
    }

    const stream = createResultReadStream(objectName, range ?? undefined);
    stream.on("error", next);
    stream.pipe(res);
  } catch (error) {
    next(error);
  }
});

appRouter.post("/uploads/:uploadId/prompt", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const updated = await uploadStore.updatePendingPrompt(
      user.id,
      req.params.uploadId,
      parsePromptInput(req.body)
    );

    if (!updated) {
      res
        .status(409)
        .type("html")
        .send(messagePage("Prompt locked", "Prompts can only be edited while an upload is pending.", "/", "Back to app"));
      return;
    }

    res.redirect(303, "/#previous-uploads");
  } catch (error) {
    next(error);
  }
});

appRouter.get("/uploads/:uploadId/result", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const upload = await getAccessibleUpload(user, req.params.uploadId);
    if (!upload || upload.status !== "completed" || !upload.resultObjectName) {
      res
        .status(404)
        .type("html")
        .send(messagePage("Results pending", "Results are not ready for this upload yet.", "/", "Back to app"));
      return;
    }

    let result;
    try {
      result = await readUploadResult(upload.resultObjectName);
    } catch (error) {
      console.error("Could not read result JSON.", error);
      res
        .status(500)
        .type("html")
        .send(messagePage("Result unavailable", "The result JSON could not be loaded.", "/", "Back to app"));
      return;
    }

    res.type("html").send(resultPage(user, upload, result));
  } catch (error) {
    next(error);
  }
});

appRouter.get("/uploads/:uploadId/results", async (req, res, next) => {
  try {
    const user = await getAppUser(req, res);
    if (!user) {
      return;
    }

    const upload = await getAccessibleUpload(user, req.params.uploadId);
    if (!upload || upload.status !== "completed" || !upload.resultObjectName) {
      res
        .status(404)
        .type("html")
        .send(messagePage("Results pending", "Results are not ready for this upload yet.", "/", "Back to app"));
      return;
    }

    const fileName = upload.resultFileName ?? "shotwell-results.json";
    res.setHeader("Content-Type", upload.resultContentType ?? "application/octet-stream");
    res.setHeader("Content-Disposition", contentDispositionAttachment(fileName));

    const stream = createResultReadStream(upload.resultObjectName);
    stream.on("error", next);
    stream.pipe(res);
  } catch (error) {
    next(error);
  }
});

appRouter.get("/auth/complete", async (req, res, next) => {
  try {
    const code = typeof req.query.code === "string" ? req.query.code : "";
    const handoff = await authStore.consumeHandoff(code);

    if (!handoff) {
      res
        .status(400)
        .type("html")
        .send(messagePage("Sign-in expired", "Start the sign-in flow again.", loginUrl(), "Sign in"));
      return;
    }

    const session = await authStore.createSession(handoff.userId, "app");
    setSessionCookie(res, "app", session.id);
    res.redirect(303, "/");
  } catch (error) {
    next(error);
  }
});

appRouter.get("/api/me", async (req, res, next) => {
  try {
    const session = await authStore.getSession(readSessionCookie(req, "app"), "app");

    if (!session) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    const user = await authStore.getUser(session.userId);
    if (!user) {
      res.status(401).json({ error: "unauthorized" });
      return;
    }

    res.json({
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        avatarUrl: user.avatarUrl
      }
    });
  } catch (error) {
    next(error);
  }
});
