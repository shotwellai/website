const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const context = { window: {} };
vm.runInNewContext(fs.readFileSync(path.join(__dirname, 'data.js'), 'utf8'), context);
const episodes = context.window.SHOTWELL_WGO_REFERENCE.episodes;
const segments = episodes.flatMap((episode) => episode.segments);
const requiredCount = Math.max(0, ...segments.map((segment) => segment.required.length));
const optionalCount = Math.max(0, ...segments.map((segment) => segment.optional.length));

const headers = [
  'episode_id', 'source_id', 'task', 'episode_duration_seconds', 'segment_number',
  'start_seconds', 'end_seconds',
  'start_boundary_id', 'start_boundary_type', 'start_tolerance_min_seconds', 'start_tolerance_max_seconds',
  'end_boundary_id', 'end_boundary_type', 'end_tolerance_min_seconds', 'end_tolerance_max_seconds',
  ...Array.from({ length: requiredCount }, (_, index) => `required_detail_${index + 1}`),
  ...Array.from({ length: optionalCount }, (_, index) => `optional_detail_${index + 1}`),
];

const rows = episodes.flatMap((episode) => {
  const boundaries = new Map(episode.boundaries.map((boundary) => [boundary.id, boundary]));
  return episode.segments.map((segment, index) => {
    const start = boundaries.get(segment.startBoundaryId);
    const end = boundaries.get(segment.endBoundaryId);
    return [
      episode.id, episode.sourceId, episode.task, episode.duration, index + 1,
      segment.start, segment.end,
      segment.startBoundaryId, start?.type, start?.minimum, start?.maximum,
      segment.endBoundaryId, end?.type, end?.minimum, end?.maximum,
      ...Array.from({ length: requiredCount }, (_, detail) => segment.required[detail]),
      ...Array.from({ length: optionalCount }, (_, detail) => segment.optional[detail]),
    ];
  });
});

const cell = (value) => {
  const text = String(value ?? '');
  const safe = typeof value === 'string' && /^\s*[=+\-@]/.test(text) ? `'${text}` : text;
  return `"${safe.replaceAll('"', '""')}"`;
};

const csv = [headers, ...rows].map((row) => row.map(cell).join(',')).join('\n') + '\n';
fs.writeFileSync(path.join(__dirname, 'shotwell-bench-reference-annotations.csv'), csv);
console.log(`Wrote ${rows.length} reference segments from ${episodes.length} episodes.`);
