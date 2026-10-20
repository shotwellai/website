/* A full-page circuit board under frosted plastic.
   The board is generated once, in page coordinates, then drawn into a fixed canvas
   tile by tile as the page scrolls. Light pulses run along the traces toward the
   cursor; hovering a target makes them rush there. On touch screens they wander. */

const COLORS = {
  green:    { base: '#1E5A44', trace: '#3E9E6C', traceHi: '#5BC08A', pad: '#E2BC5E', silk: 'rgba(255,255,255,.75)', chip: '#16161B', pin: '#BFC3C9', smd: '#55585E' },
  charcoal: { base: '#1B1A21', trace: '#55526A', traceHi: '#77738E', pad: '#D29A52', silk: 'rgba(214,210,226,.7)', chip: '#0E0D12', pin: '#9EA2AA', smd: '#5E5868' },
};
const PULSE = { yellow: ['#EFE2B4', 'rgba(201,168,84,'], pink: ['#EFE2B4', 'rgba(201,168,84,'] };   // muted; all pulses yellow
const TILE = 800, G = 16;

function rng(seed) { return () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296); }
const DIRS = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]].map(([x, y]) => { const l = Math.hypot(x, y); return [x / l, y / l]; });

/* ── generation ── */
function generate(W, H, seed) {
  const r = rng(seed), traces = [], chips = [], smds = [], vias = [], holes = [];
  const snap = (v) => Math.round(v / G) * G;
  // chips first, so bundles can leave from their pins
  for (let i = 0; i < W * H / 220000; i++) {
    const big = r() < 0.35, w = snap(big ? 96 + r() * 80 : 48 + r() * 64), h = snap(big ? w : 32 + r() * 32);
    chips.push({ x: snap(r() * (W - w)), y: snap(r() * (H - h)), w, h, quad: big, name: `U${i + 1}` });
  }
  const bundle = (sx, sy, d, lanes, sp) => {
    // a spine with 45° turns; lanes are offset copies of it
    const spine = [[sx, sy]];
    let x = sx, y = sy;
    for (let k = 0, n = 3 + (r() * 5 | 0); k < n; k++) {
      const len = (4 + (r() * 16 | 0)) * G;
      x += DIRS[d][0] * len; y += DIRS[d][1] * len;
      spine.push([x, y]);
      d = (d + (r() < 0.5 ? 1 : 7)) % 8;
      if (d % 2 === 1 && r() < 0.5) d = (d + (r() < 0.5 ? 1 : 7)) % 8;   // mostly come back to straight
    }
    for (let l = 0; l < lanes; l++) {
      const off = (l - (lanes - 1) / 2) * sp, pts = [];
      for (let i = 0; i < spine.length; i++) {
        const a = spine[Math.max(0, i - 1)], b = spine[i], c = spine[Math.min(spine.length - 1, i + 1)];
        const n1 = norm(a, b), n2 = norm(b, c);
        let nx = (n1[0] + n2[0]) || n2[0], ny = (n1[1] + n2[1]) || n2[1];
        const nl = Math.hypot(nx, ny) || 1; nx /= nl; ny /= nl;
        const m = 1 / Math.max(0.5, nx * n2[0] + ny * n2[1]);
        pts.push([b[0] + nx * off * m, b[1] + ny * off * m]);
      }
      traces.push(makeTrace(pts, r() < 0.12 ? 2.6 : 1.4));
    }
  };
  for (let i = 0; i < W * H / 60000; i++) {
    const lanes = 1 + (r() * (r() < 0.4 ? 6 : 2) | 0);
    if (chips.length && r() < 0.5) {                    // leave from a chip side
      const c = chips[r() * chips.length | 0], side = r() * 4 | 0;
      const d = [0, 2, 4, 6][side];
      const sx = side === 0 ? c.x + c.w : side === 2 ? c.x + c.w / 2 : side === 4 ? c.x : c.x + c.w / 2;
      const sy = side === 0 ? c.y + c.h / 2 : side === 2 ? c.y + c.h : side === 4 ? c.y + c.h / 2 : c.y;
      bundle(sx, sy, d, lanes, 7);
    } else bundle(snap(r() * W), snap(r() * H), (r() * 4 | 0) * 2, lanes, 7);
  }
  for (let i = 0; i < W * H / 14000; i++) smds.push({ x: r() * W, y: r() * H, v: r() < 0.5, k: r() < 0.6 ? 'R' : 'C', n: 1 + (r() * 99 | 0) });
  for (let i = 0; i < W * H / 9000; i++) vias.push([r() * W, r() * H]);
  for (let y = 300; y < H; y += 1100 + r() * 400) { holes.push([36 + r() * 30, y]); holes.push([W - 36 - r() * 30, y + 200 + r() * 300]); }
  return { W, H, traces, chips, smds, vias, holes };
}
function norm(a, b) { const dx = b[0] - a[0], dy = b[1] - a[1], l = Math.hypot(dx, dy) || 1; return [-dy / l, dx / l]; }
function makeTrace(pts, w) {
  const seg = [0];
  for (let i = 1; i < pts.length; i++) seg.push(seg[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
  let minY = Infinity, maxY = -Infinity, sx = 0, sy = 0;
  for (const [x, y] of pts) { minY = Math.min(minY, y); maxY = Math.max(maxY, y); sx += x; sy += y; }
  return { pts, seg, len: seg[seg.length - 1], w, minY, maxY, cx: sx / pts.length, cy: sy / pts.length };
}
function pointAt(t, s) {
  s = Math.max(0, Math.min(t.len, s));
  let i = 1; while (i < t.seg.length - 1 && t.seg[i] < s) i++;
  const a = t.pts[i - 1], b = t.pts[i], k = (s - t.seg[i - 1]) / ((t.seg[i] - t.seg[i - 1]) || 1);
  return [a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k];
}

/* ── drawing one tile of the static board ── */
function drawTile(b, ty, C) {
  const cv = document.createElement('canvas'); cv.width = b.W; cv.height = TILE;
  const g = cv.getContext('2d'); g.translate(0, -ty);
  g.fillStyle = C.base; g.fillRect(0, ty, b.W, TILE);
  const inTile = (y0, y1) => y1 >= ty - 40 && y0 <= ty + TILE + 40;
  g.lineCap = 'round'; g.lineJoin = 'round';
  for (const t of b.traces) {
    if (!inTile(t.minY, t.maxY)) continue;
    g.strokeStyle = t.w > 2 ? C.traceHi : C.trace; g.lineWidth = t.w;
    g.beginPath(); t.pts.forEach(([x, y], i) => i ? g.lineTo(x, y) : g.moveTo(x, y)); g.stroke();
    for (const [x, y] of [t.pts[0], t.pts[t.pts.length - 1]]) { g.fillStyle = C.pad; g.beginPath(); g.arc(x, y, 3, 0, 6.283); g.fill(); g.fillStyle = C.base; g.beginPath(); g.arc(x, y, 1.2, 0, 6.283); g.fill(); }
  }
  for (const [x, y] of b.vias) { if (!inTile(y, y)) continue; g.strokeStyle = C.pad; g.lineWidth = 1.4; g.beginPath(); g.arc(x, y, 2.6, 0, 6.283); g.stroke(); }
  for (const s of b.smds) {
    if (!inTile(s.y, s.y + 20)) continue;
    const w = s.v ? 6 : 14, h = s.v ? 14 : 6;
    g.fillStyle = C.pad; g.fillRect(s.x - 2, s.y - 2, s.v ? w + 4 : 4, s.v ? 4 : h + 4); g.fillRect(s.v ? s.x - 2 : s.x + w - 2, s.v ? s.y + h - 2 : s.y - 2, s.v ? w + 4 : 4, s.v ? 4 : h + 4);
    g.fillStyle = C.smd; g.fillRect(s.x, s.y, w, h);
    g.fillStyle = C.silk; g.font = '500 7px "IBM Plex Mono", monospace'; g.fillText(`${s.k}${s.n}`, s.x + w + 4, s.y + 7);
  }
  for (const c of b.chips) {
    if (!inTile(c.y - 10, c.y + c.h + 10)) continue;
    g.fillStyle = C.pin;
    for (let x = c.x + 6; x < c.x + c.w - 4; x += 7) { g.fillRect(x, c.y - 6, 3, 6); g.fillRect(x, c.y + c.h, 3, 6); }
    if (c.quad) for (let y = c.y + 6; y < c.y + c.h - 4; y += 7) { g.fillRect(c.x - 6, y, 6, 3); g.fillRect(c.x + c.w, y, 6, 3); }
    g.fillStyle = C.chip; g.fillRect(c.x, c.y, c.w, c.h);
    g.fillStyle = 'rgba(255,255,255,.18)'; g.beginPath(); g.arc(c.x + 8, c.y + 8, 2.5, 0, 6.283); g.fill();
    g.fillStyle = C.silk; g.font = '500 9px "IBM Plex Mono", monospace'; g.fillText(c.name, c.x, c.y - 10);
  }
  for (const [x, y] of b.holes) {
    if (!inTile(y - 20, y + 20)) continue;
    g.fillStyle = C.pad; g.beginPath(); g.arc(x, y, 14, 0, 6.283); g.fill();
    g.fillStyle = '#0B0B0E'; g.beginPath(); g.arc(x, y, 8, 0, 6.283); g.fill();
  }
  return cv;
}

/* ── the live board ── */
export function startBoard(canvas, opts = {}) {
  const ctx = canvas.getContext('2d'), lc = opts.lights || canvas, lx = lc.getContext('2d');
  const touch = matchMedia('(hover: none)').matches;
  const still = matchMedia('(prefers-reduced-motion: reduce)').matches;
  let board = null, tiles = new Map(), kind = opts.kind || 'green', pulses = [], flashes = [];
  let cursor = null, target = null, failNow = false, lastW = 0, drawnY = null;

  function build() {
    const W = innerWidth, H = document.documentElement.scrollHeight + 400;
    board = generate(W, H, 42); tiles = new Map(); lastW = W; drawnY = null;
    const dpr = Math.min(devicePixelRatio || 1, 2);
    for (const [c, x] of [[canvas, ctx], [lc, lx]]) { c.width = innerWidth * dpr; c.height = innerHeight * dpr; x.setTransform(dpr, 0, 0, dpr, 0, 0); }
    drawnY = null;
  }
  function invalidate() { drawnY = null; }
  function tile(i) { if (!tiles.has(i)) tiles.set(i, drawTile(board, i * TILE, COLORS[kind])); return tiles.get(i); }

  function spawn(sy) {
    const vis = board.traces.filter(t => t.maxY > sy - 200 && t.minY < sy + innerHeight + 200 && t.len > 80);
    if (!vis.length) return;
    const goal = target || cursor;
    for (let tries = 0; tries < 6; tries++) {
      const t = vis[Math.random() * vis.length | 0];
      let dir = Math.random() < 0.5 ? 1 : -1, p = 1;
      if (goal && !touch) {
        const a = t.pts[0], b = t.pts[t.pts.length - 1];
        const da = Math.hypot(a[0] - goal[0], a[1] - goal[1]), db = Math.hypot(b[0] - goal[0], b[1] - goal[1]);
        dir = db < da ? 1 : -1;                                        // run toward the end nearer the goal
        const d = Math.hypot(t.cx - goal[0], t.cy - goal[1]);
        p = Math.exp(-d / (target ? 380 : 520));
      }
      if (Math.random() > p) continue;
      let pink = false;
      if (failNow && opts.failEl) {                                  // pink only around the episode while it shows a failed step
        const r = opts.failEl.getBoundingClientRect(), fx = r.left + r.width / 2, fy = r.top + r.height / 2 + scrollY;
        pink = Math.hypot(t.cx - fx, t.cy - fy) < Math.max(r.width, r.height) * 0.7;
      }
      pulses.push({ t, s: dir > 0 ? 0 : t.len, dir, v: (target ? 2.2 : 1.1) + Math.random() * 0.6, pink });
      return;
    }
  }

  function frame() {
    if (innerWidth !== lastW) build();
    const sy = scrollY, W = innerWidth, H = innerHeight;
    // the still board is redrawn only when the view moves, so the frost above it is not re-blurred every frame
    if (drawnY !== sy) {
      ctx.clearRect(0, 0, W, H);
      for (let i = Math.floor(sy / TILE); i <= Math.floor((sy + H) / TILE); i++) ctx.drawImage(tile(i), 0, i * TILE - sy);
      drawnY = sy;
    }
    if (window.__boardPaused) { requestAnimationFrame(frame); return; }   // keep the last frame (used for screenshots)
    lx.clearRect(0, 0, W, H);
    if (!still) {
      const rate = touch ? 0.12 : target ? 0.6 : 0.3;   // calm: fewer, slower pulses
      for (let k = rate; k > 0; k--) if (Math.random() < k && pulses.length < 60) spawn(sy);
      lx.lineCap = 'round';
      pulses = pulses.filter(p => {
        p.s += p.dir * p.v;
        const done = p.s < 0 || p.s > p.t.len;
        const head = pointAt(p.t, p.s), tail = pointAt(p.t, p.s - p.dir * 46);
        if (head[1] > sy - 60 && head[1] < sy + H + 60) {
          const [core, glow] = PULSE[p.pink ? 'pink' : 'yellow'];
          // dimmer in the middle band, where the content sits
          const edge = Math.min(W * 0.46, 640), off = Math.abs(head[0] - W / 2) - edge;
          lx.globalAlpha = off <= 0 ? 0.3 : Math.min(1, 0.3 + off / 120 * 0.7);
          const grad = lx.createLinearGradient(tail[0], tail[1] - sy, head[0], head[1] - sy);
          grad.addColorStop(0, glow + '0)'); grad.addColorStop(1, glow + '.7)');
          lx.strokeStyle = grad;
          for (const [w, a] of [[9, 0.25], [4, 1]]) {          // a soft halo, then the line: glow without a CSS blur
            lx.globalAlpha *= a; lx.lineWidth = w; lx.beginPath(); lx.moveTo(tail[0], tail[1] - sy); lx.lineTo(head[0], head[1] - sy); lx.stroke(); lx.globalAlpha /= a;
          }
          lx.fillStyle = core; lx.beginPath(); lx.arc(head[0], head[1] - sy, 2.6, 0, 6.283); lx.fill();
          lx.globalAlpha = 1;
        }
        if (done) flashes.push({ x: head[0], y: head[1], a: 1, pink: p.pink });
        return !done;
      });
      flashes = flashes.filter(f => {
        f.a -= 0.02;
        const [core, glow] = PULSE[f.pink ? 'pink' : 'yellow'];
        lx.fillStyle = glow + (f.a * 0.6).toFixed(2) + ')'; lx.beginPath(); lx.arc(f.x, f.y - sy, 12 * (1.4 - f.a), 0, 6.283); lx.fill();
        return f.a > 0;
      });
    }
    requestAnimationFrame(frame);
  }

  build();
  addEventListener('resize', () => { const dpr = Math.min(devicePixelRatio || 1, 2); for (const [c, x] of [[canvas, ctx], [lc, lx]]) { c.width = innerWidth * dpr; c.height = innerHeight * dpr; x.setTransform(dpr, 0, 0, dpr, 0, 0); } drawnY = null; });
  addEventListener('load', build);
  if (!touch) addEventListener('pointermove', (e) => { cursor = [e.clientX, e.clientY + scrollY]; });
  // targets: buttons, links and sections pull the light when hovered
  for (const el of document.querySelectorAll(opts.targets || 'a.btn, .textlink, .card, .three > div, .data > div, .strip button, .field a, .episode .frame')) {
    el.addEventListener('pointerenter', () => { const r = el.getBoundingClientRect(); target = [r.left + r.width / 2, r.top + r.height / 2 + scrollY]; target.fail = !!el.closest('.episode'); });
    el.addEventListener('pointerleave', () => { target = null; });
  }
  requestAnimationFrame(frame);
  return {
    setKind(k) { kind = k; tiles = new Map(); drawnY = null; },
    setFail(v) { failNow = v; },
  };
}
