/* Generative art for the homepage mockups.
   mount(canvas, kind, opts) draws one study into a canvas and keeps it animated
   while it is on screen. Studies come from samples/generative.html. */

export const SEGMENTS = [
  { label: 'Pick up', start: 0, end: 1, fail: false },
  { label: 'Straighten', start: 1, end: 7, fail: true },
  { label: 'Fold right sleeve', start: 7, end: 10, fail: false },
  { label: 'Fold left sleeve', start: 10, end: 15, fail: true },
  { label: 'Fold first third', start: 15, end: 16, fail: false },
  { label: 'Fold second third', start: 16, end: 17, fail: false },
  { label: 'Stack', start: 17, end: 18, fail: false },
  { label: 'Home', start: 18, end: 19, fail: false },
];

const LIGHT = { ink: '34,34,42', ground: '#F5F2EB', acc: '#EFB702', fail: '175,50,97' };
const DARK = { ink: '245,242,235', ground: '#121216', acc: '#EFB702', fail: '229,90,130' };
const FROST_L = { ...LIGHT, fail: '34,34,42' }, FROST_D = { ...DARK, fail: '245,242,235' };   // E: failures in ink, no colour

function rng(seed) { return () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296); }
function wobble(r, n = 4) {
  const parts = Array.from({ length: n }, (_, i) => ({ f: (i + 1) * (0.6 + r()), p: r() * 6.283, a: 1 / (i + 1) }));
  return (t) => parts.reduce((s, q) => s + q.a * Math.sin(q.f * t * 6.283 + q.p), 0);
}

// One shared clip for every study that reads footage. A blob URL makes seeks work on any server.
let clip = null;
export function getClip() {
  if (clip) return clip;
  clip = document.createElement('video');
  Object.assign(clip, { muted: true, loop: true, playsInline: true });
  fetch('/fold-rubric.mp4').then(r => r.blob()).then(b => { clip.src = URL.createObjectURL(b); clip.play().catch(() => {}); });
  return clip;
}
export const currentSegment = (t) => SEGMENTS.find(g => t < g.end) || SEGMENTS[7];

const STUDIES = {
  /* The clip drawn with the letters of its current subtask. */
  labels: {
    init() { getClip(); return { last: 0, read: document.createElement('canvas') }; },
    draw(ctx, s, t, c) {
      const vid = getClip();
      if (t - s.data.last < 1 / 20 || vid.readyState < 2) return;
      s.data.last = t;
      const cw = Math.max(5, s.w / (s.opts.cols || 200)), ch = cw * 1.4, gw = Math.floor(s.w / cw), gh = Math.floor(s.h / ch);
      const read = s.data.read, rc = read.getContext('2d', { willReadFrequently: true });
      read.width = gw; read.height = gh;
      const iw = vid.videoWidth, ih = vid.videoHeight, sw = Math.min(iw, ih * s.w / s.h), sh = Math.min(ih, iw * s.h / s.w);
      rc.drawImage(vid, (iw - sw) / 2, (ih - sh) / 2, sw, sh, 0, 0, gw, gh);
      const px = rc.getImageData(0, 0, gw, gh).data;
      const seg = currentSegment(vid.currentTime);
      const word = seg.label.toUpperCase().replace(/ /g, '·') + '·';
      ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
      ctx.font = s.opts.font ? `${(cw * s.opts.fontScale).toFixed(1)}px ${s.opts.font}` : `500 ${(cw * 1.3).toFixed(1)}px "IBM Plex Mono", monospace`;
      for (let y = 0; y < gh; y++) for (let x = 0; x < gw; x++) {
        const i = (y * gw + x) * 4, l = (0.2126 * px[i] + 0.7152 * px[i + 1] + 0.0722 * px[i + 2]) / 255;
        const v = Math.min(1, Math.max(0, (l - 0.36) / 0.42));
        if (v < 0.08) continue;
        ctx.fillStyle = seg.fail && v > 0.75 ? `rgba(${c.fail},${v})` : `rgba(${c.ink},${0.15 + v * 0.85})`;
        ctx.fillText(word[(x + y * 3) % word.length], x * cw, y * ch + ch * 0.8);
      }
      s.opts.onFrame && s.opts.onFrame(vid, seg);
    }
  },

  /* Each row is one episode cut into the 8 real subtasks. */
  strata: {
    init(s, r) {
      return { eps: Array.from({ length: s.opts.rows || 40 }, () => {
        let acc = 0;
        const durs = SEGMENTS.map(g => (g.end - g.start) * (0.6 + r() * 0.8)), total = durs.reduce((a, b) => a + b, 0);
        return { cuts: durs.map(d => (acc += d) / total), fails: SEGMENTS.map(g => g.fail && r() < 0.16), drift: wobble(r, 2) };
      }) };
    },
    draw(ctx, s, t, c) {
      ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
      const p = s.opts.pad ?? 24, L = p, R = s.w - p, T = p, B = s.h - p, rows = s.data.eps.length, rh = (B - T) / rows;
      const tones = c === FROST_D ? ['#5A5A62', '#3D4350', '#55524B', '#6E6A60', '#47443F', '#5E5A52', '#3A3A40', '#77736A'] : c === GBC_PAL ? ['#4A4366', '#7A4FC0', '#9E97B4', '#BDB7CC', '#6E6A86', '#A9A2C0', '#4A4366', '#C9C4D6'] : c === GB_PAL ? ['#0F380F', '#306230', '#8BAC0F', '#306230', '#8BAC0F', '#306230', '#0F380F', '#8BAC0F'] : c === DARK ? ['#5A5A62', '#3D4350', '#55524B', '#6E6A60', '#47443F', '#5E5A52', '#3A3A40', '#77736A']
                               : ['#5A5A62', '#8C97A8', '#B9B4A6', '#D9D3C4', '#A7A39A', '#C9C3B5', '#7E7A73', '#E2DCCD'];
      const pts = SEGMENTS.map(() => []);
      s.data.eps.forEach((ep, i) => {
        const y = T + i * rh, sweep = Math.min(1, t * 0.35 - i * 0.012);
        if (sweep <= 0) return;
        let x0 = L;
        ep.cuts.forEach((cut, k) => {
          const cc = Math.min(1, cut + ep.drift(t * 0.02 + i * 0.01) * 0.004 * (k < 7));
          const x1 = L + (R - L) * Math.min(cc, sweep);
          if (x1 > x0) {
            ctx.fillStyle = ep.fails[k] ? `rgb(${c.fail})` : (k === 1 ? c.acc : tones[k]);
            ctx.globalAlpha = ep.fails[k] ? 0.9 : (k === 1 ? 0.85 : 0.75);
            ctx.fillRect(x0, y + rh * 0.18, x1 - x0 - 1, rh * 0.64);
            ctx.globalAlpha = 1;
          }
          pts[k].push([L + (R - L) * cc, y + rh / 2]);
          x0 = x1;
        });
      });
      ctx.strokeStyle = `rgba(${c.ink},.5)`; ctx.lineWidth = 0.7;
      pts.slice(0, 7).forEach(line => { ctx.beginPath(); line.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.stroke(); });
    }
  },

  /* Gripper speed for 40 episodes, stacked as ridge lines. */
  ridges: {
    init(s, r) {
      return { eps: Array.from({ length: s.opts.rows || 36 }, () => ({ jit: SEGMENTS.map(() => (r() - 0.5) * 0.6), amp: SEGMENTS.map(() => 0.4 + r() * 0.8), w: wobble(r, 5) })) };
    },
    speed(ep, u) {
      let v = 0.06 + 0.03 * ep.w(u * 3);
      SEGMENTS.forEach((g, k) => { const d = (u - (g.end + ep.jit[k]) / 19) / 0.022; v += ep.amp[k] * Math.exp(-d * d); });
      return v;
    },
    draw(ctx, s, t, c) {
      ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
      const n = s.data.eps.length, px = s.opts.padX ?? 0.1, L = s.w * px, R = s.w * (1 - px), T = s.h * 0.24, B = s.h * 0.94;
      const gap = (B - T) / n, N = 160, hero = Math.floor(n * 0.6);
      for (let i = 0; i < n; i++) {
        const ep = s.data.eps[i], y0 = T + i * gap;
        ctx.beginPath(); ctx.moveTo(L, y0);
        for (let j = 0; j <= N; j++) {
          const u = j / N;
          ctx.lineTo(L + (R - L) * u, y0 - this.speed(ep, (u + t * 0.015) % 1) * gap * 2.6 * Math.sin(Math.PI * u) ** 0.6);
        }
        ctx.lineTo(R, y0); ctx.closePath();
        ctx.fillStyle = c.ground; ctx.fill();
        ctx.strokeStyle = i === hero ? c.acc : `rgba(${c.ink},.9)`; ctx.lineWidth = i === hero ? 1.8 : 0.9; ctx.stroke();
      }
    }
  },

  /* 64 demonstrations of one carry path. */
  envelope: {
    init(s, r) { return { demos: Array.from({ length: 64 }, (_, i) => ({ wx: wobble(r), wy: wobble(r), amp: 0.5 + r(), fail: i < 6, speed: 0.85 + r() * 0.3 })) }; },
    base(u, s) { return [(0.12 + 0.76 * u) * s.w, (0.74 - 0.38 * Math.sin(Math.PI * u) ** 0.8) * s.h]; },
    draw(ctx, s, t, c) {
      ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
      const phase = (t % 7) / 5.5, N = 140, sc = Math.min(1, s.h / 400);
      for (const d of s.data.demos) {
        const end = Math.min(1, phase * d.speed), path = [];
        for (let i = 0; i <= N; i++) {
          const u = i / N, env = Math.sin(Math.PI * u);
          let [x, y] = this.base(u, s);
          x += d.wx(u) * 22 * sc * d.amp * env; y += d.wy(u) * 30 * sc * d.amp * env;
          if (d.fail && u > 0.55) y += (u - 0.55) ** 2 * s.h * 0.9;
          path.push([x, y]);
        }
        const stroke = (pts, style, lw) => { ctx.beginPath(); pts.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.strokeStyle = style; ctx.lineWidth = lw; ctx.stroke(); };
        stroke(path, d.fail ? `rgba(${c.fail},.25)` : `rgba(${c.ink},.08)`, 0.8);
        const k = Math.floor(N * end);
        if (k > 0) stroke(path.slice(Math.max(0, k - 28), k + 1), d.fail ? `rgba(${c.fail},.85)` : `rgba(${c.ink},.5)`, 1);
      }
      ctx.setLineDash([2, 5]); ctx.strokeStyle = c.acc; ctx.lineWidth = 1.5; ctx.beginPath();
      for (let i = 0; i <= N; i++) { const [x, y] = this.base(i / N, s); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
      ctx.stroke(); ctx.setLineDash([]);
      for (const u of [0, 1]) {
        const [x, y] = this.base(u, s);
        ctx.strokeStyle = `rgb(${c.ink})`; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(x, y, 12, 0, 6.283); ctx.moveTo(x - 8, y); ctx.lineTo(x + 8, y); ctx.moveTo(x, y - 8); ctx.lineTo(x, y + 8); ctx.stroke();
      }
    }
  },

  /* Success-rate contours with trial dots. */
  contours: {
    init(s, r) {
      return { hills: Array.from({ length: 5 }, () => ({ x: 0.2 + r() * 0.6, y: 0.2 + r() * 0.6, sx: 0.12 + r() * 0.15, sy: 0.12 + r() * 0.15, a: 0.5 + r() * 0.5, ph: r() * 6.28 })),
               trials: Array.from({ length: 110 }, () => ({ x: r(), y: r(), roll: r() })) };
    },
    f(h, x, y, t) {
      let v = 0;
      for (const q of h) { const dx = (x - q.x - 0.03 * Math.sin(t * 0.3 + q.ph)) / q.sx, dy = (y - q.y - 0.03 * Math.cos(t * 0.25 + q.ph)) / q.sy; v += q.a * Math.exp(-(dx * dx + dy * dy)); }
      return Math.min(1, v);
    },
    draw(ctx, s, t, c) {
      const h = s.data.hills;
      ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
      const pad = 18, W = s.w - 2 * pad, H = s.h - 2 * pad, gx = 70, gy = Math.max(10, Math.round(70 * H / W));
      const F = []; for (let j = 0; j <= gy; j++) { F.push([]); for (let i = 0; i <= gx; i++) F[j].push(this.f(h, i / gx, j / gy, t)); }
      const X = i => pad + i / gx * W, Y = j => pad + j / gy * H;
      for (let lv = 1; lv <= 9; lv++) {
        const L = lv / 10;
        ctx.strokeStyle = lv === 9 ? c.acc : `rgba(${c.ink},${0.18 + lv * 0.04})`; ctx.lineWidth = lv === 9 ? 2 : 0.8;
        ctx.beginPath();
        for (let j = 0; j < gy; j++) for (let i = 0; i < gx; i++) {
          const a = F[j][i], b = F[j][i + 1], cc = F[j + 1][i + 1], d = F[j + 1][i], e = [];
          const cut = (p, q, x1, y1, x2, y2) => { if ((p < L) !== (q < L)) { const k = (L - p) / (q - p); e.push([X(x1 + (x2 - x1) * k), Y(y1 + (y2 - y1) * k)]); } };
          cut(a, b, i, j, i + 1, j); cut(b, cc, i + 1, j, i + 1, j + 1); cut(d, cc, i, j + 1, i + 1, j + 1); cut(a, d, i, j, i, j + 1);
          for (let k = 0; k + 1 < e.length; k += 2) { ctx.moveTo(...e[k]); ctx.lineTo(...e[k + 1]); }
        }
        ctx.stroke();
      }
      for (const tr of s.data.trials) {
        const x = pad + tr.x * W, y = pad + tr.y * H;
        if (tr.roll < this.f(h, tr.x, tr.y, t)) { ctx.fillStyle = `rgb(${c.ink})`; ctx.fillRect(x - 1.5, y - 1.5, 3, 3); }
        else { ctx.strokeStyle = `rgb(${c.fail})`; ctx.lineWidth = 1.1; ctx.beginPath(); ctx.moveTo(x - 3, y - 3); ctx.lineTo(x + 3, y + 3); ctx.moveTo(x + 3, y - 3); ctx.lineTo(x - 3, y + 3); ctx.stroke(); }
      }
    }
  },

  /* Two end effectors leave light trails, like a long exposure. Dark only. */
  exposure: {
    init(s, r) {
      const grain = document.createElement('canvas'); grain.width = grain.height = 256;
      const g = grain.getContext('2d'), im = g.createImageData(256, 256);
      for (let i = 0; i < im.data.length; i += 4) { im.data[i] = im.data[i + 1] = im.data[i + 2] = r() * 255; im.data[i + 3] = 12; }
      g.putImageData(im, 0, 0);
      return { arms: [0, 1].map(() => ({ wx: wobble(r, 5), wy: wobble(r, 5) })), grain, fresh: true, last: null };
    },
    draw(ctx, s, t, c) {
      const d = s.data;
      if (d.fresh) { ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h); ctx.fillStyle = ctx.createPattern(d.grain, 'repeat'); ctx.fillRect(0, 0, s.w, s.h); d.fresh = false; }
      const dark = c === DARK;
      if (((t * 60) | 0) % 4 === 0) { ctx.fillStyle = dark ? 'rgba(18,18,22,.02)' : 'rgba(245,242,235,.03)'; ctx.fillRect(0, 0, s.w, s.h); }
      ctx.globalCompositeOperation = dark ? 'lighter' : 'source-over';
      const pos = d.arms.map((a, i) => {
        const side = i ? 1 : -1, u = t * 1.4;
        return [s.w / 2 + side * s.w * (0.17 + 0.08 * Math.sin(u * 1.3 + i)) + a.wx(u * 0.07) * s.w * 0.1,
                s.h * 0.45 + Math.sin(u * 2 + i * 1.7) * s.h * 0.2 + a.wy(u * 0.05) * s.h * 0.1];
      });
      if (d.last) pos.forEach((p, i) => {
        ctx.strokeStyle = i ? (dark ? 'rgba(239,183,2,.45)' : 'rgba(214,160,0,.8)') : (dark ? 'rgba(245,242,235,.35)' : 'rgba(34,34,42,.45)'); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(...d.last[i]); ctx.lineTo(...p); ctx.stroke();
      });
      d.last = pos;
      ctx.globalCompositeOperation = 'source-over';
    }
  },
};

/* Lines that gather into a shape, hold it, and let go. Shapes: the wordmark,
   the lantern mark, a gripper, and a shirt. The trails stay, like a long exposure. */
const SHAPES = ['word', 'mark', 'gripper', 'shirt'];
let markImg = null;
export function shapePoints(kind, w, h, n, r) {
  const sc = 0.5, cw = Math.round(w * sc), chh = Math.round(h * sc);
  const cv = document.createElement('canvas'); cv.width = cw; cv.height = chh;
  const g = cv.getContext('2d'); g.fillStyle = '#fff'; g.strokeStyle = '#fff';
  const wide = w > 900, cx = wide ? cw * 0.66 : cw / 2, cy = chh * (wide ? 0.5 : 0.5), S = wide ? Math.min(cw * 0.5, chh * 1.0) : Math.min(cw, chh * 1.6);
  if (kind === 'word') {
    g.font = `300 ${Math.round(S * 0.17)}px Fraunces, Georgia, serif`; g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText('Shotwell.', cx, cy);
  } else if (kind === 'mark' && markImg && markImg.complete) {
    const mh = S * 0.62, mw = mh * markImg.naturalWidth / markImg.naturalHeight;
    g.drawImage(markImg, cx - mw / 2, cy - mh / 2, mw, mh);
  } else if (kind === 'gripper') {
    const u = S / 10; g.lineWidth = u * 0.5; g.lineJoin = 'round'; g.lineCap = 'round';
    g.beginPath(); g.moveTo(cx, cy - 3.4 * u); g.lineTo(cx, cy - 1.2 * u); g.stroke();          // wrist
    g.fillRect(cx - 1.9 * u, cy - 1.4 * u, 3.8 * u, 0.9 * u);                                     // palm
    for (const sgn of [-1, 1]) {                                                                   // fingers
      g.beginPath(); g.moveTo(cx + sgn * 1.6 * u, cy - 0.6 * u); g.lineTo(cx + sgn * 1.6 * u, cy + 1.4 * u);
      g.lineTo(cx + sgn * 0.7 * u, cy + 2.4 * u); g.stroke();
    }
    g.beginPath(); g.arc(cx, cy + 1.3 * u, 0.75 * u, 0, 6.283); g.fill();                         // held object
  } else {
    const u = S / 11; g.beginPath();                                                               // t-shirt
    g.moveTo(cx - 1.3 * u, cy - 2.6 * u); g.quadraticCurveTo(cx, cy - 1.6 * u, cx + 1.3 * u, cy - 2.6 * u);
    g.lineTo(cx + 3.6 * u, cy - 1.6 * u); g.lineTo(cx + 2.9 * u, cy - 0.2 * u); g.lineTo(cx + 2.1 * u, cy - 0.6 * u);
    g.lineTo(cx + 2.1 * u, cy + 2.8 * u); g.lineTo(cx - 2.1 * u, cy + 2.8 * u); g.lineTo(cx - 2.1 * u, cy - 0.6 * u);
    g.lineTo(cx - 2.9 * u, cy - 0.2 * u); g.lineTo(cx - 3.6 * u, cy - 1.6 * u); g.closePath(); g.fill();
  }
  const a = g.getImageData(0, 0, cw, chh).data, edge = [], fill = [];
  const on = (x, y) => x >= 0 && y >= 0 && x < cw && y < chh && a[(y * cw + x) * 4 + 3] > 120;
  for (let y = 0; y < chh; y++) for (let x = 0; x < cw; x++) {
    if (!on(x, y)) continue;
    (on(x - 1, y) && on(x + 1, y) && on(x, y - 1) && on(x, y + 1) ? fill : edge).push([x / sc, y / sc]);
  }
  if (!edge.length) return null;
  return Array.from({ length: n }, (_, i) => {
    const src = i % 6 === 0 && fill.length ? fill : edge;
    const [x, y] = src[Math.floor(r() * src.length)];
    return [x, y];
  });
}

STUDIES.shapes = {
  init(s, r) {
    if (!markImg) { markImg = new Image(); markImg.src = '/brand/assets/SVG/shotwell-mark-white.svg'; }
    const n = s.w < 600 ? 900 : 1700;
    const ps = Array.from({ length: n }, () => ({ x: r() * s.w, y: r() * s.h, vx: 0, vy: 0, px: 0, py: 0 }));
    ps.forEach(p => { p.px = p.x; p.py = p.y; });
    return { ps, r, n, k: -1, targets: null, fresh: true };
  },
  draw(ctx, s, t, c) {
    const d = s.data, dark = c === DARK, CYC = 8;
    const k = Math.floor(t / CYC), ph = (t % CYC) / CYC;
    window.__shapes = { k, ph };
    if (k !== d.k) {                                   // a new shape each cycle
      const pts = shapePoints(SHAPES[k % SHAPES.length], s.w, s.h, d.n, d.r);
      if (pts) { d.targets = pts; d.k = k; }
    }
    if (d.fresh) { ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h); d.fresh = false; }
    // fade: slow while the shape holds, fast when it lets go
    const fade = ph > 0.78 ? 0.12 : 0.055;
    ctx.globalCompositeOperation = 'source-over';
    ctx.fillStyle = dark ? `rgba(18,18,22,${fade})` : `rgba(245,242,235,${fade})`; ctx.fillRect(0, 0, s.w, s.h);
    // pull: none, then rising, then hold, then release
    const pull = ph < 0.12 ? 0 : ph < 0.5 ? (ph - 0.12) / 0.38 : ph < 0.78 ? 1 : 0;
    ctx.globalCompositeOperation = dark ? 'lighter' : 'source-over';
    ctx.lineWidth = 0.8;
    const T = d.targets;
    for (let i = 0; i < d.ps.length; i++) {
      const p = d.ps[i];
      const a = Math.sin(p.x * 0.004 + t * 0.4) * Math.cos(p.y * 0.005 - t * 0.3) * 3.2;
      const free = 1 - pull;
      p.vx = p.vx * 0.9 + Math.cos(a) * 0.35 * free;
      p.vy = p.vy * 0.9 + Math.sin(a) * 0.35 * free;
      if (T && pull > 0) {
        const [tx, ty] = T[i % T.length];
        p.vx += (tx - p.x) * 0.012 * pull * pull; p.vy += (ty - p.y) * 0.012 * pull * pull;
      }
      if (pull === 1) { p.vx += (d.r() - 0.5) * 0.12; p.vy += (d.r() - 0.5) * 0.12; }
      if (ph > 0.78 && ph < 0.8) { p.vx += (d.r() - 0.5) * 6; p.vy += (d.r() - 0.5) * 6; }
      p.px = p.x; p.py = p.y; p.x += p.vx; p.y += p.vy;
      if (p.x < -40 || p.x > s.w + 40 || p.y < -40 || p.y > s.h + 40) { p.x = p.px = d.r() * s.w; p.y = p.py = d.r() * s.h; p.vx = p.vy = 0; continue; }
      const gold = i % 9 === 0;
      ctx.strokeStyle = dark ? (gold ? 'rgba(239,183,2,.5)' : 'rgba(245,242,235,.22)') : (gold ? 'rgba(214,160,0,.7)' : 'rgba(34,34,42,.25)');
      ctx.beginPath(); ctx.moveTo(p.px, p.py); ctx.lineTo(p.x, p.y); ctx.stroke();
      if (Math.abs(p.vx) + Math.abs(p.vy) < 0.8) {          // settled on the shape: draw a point
        ctx.fillStyle = ctx.strokeStyle; ctx.fillRect(p.x - 0.7, p.y - 0.7, 1.4, 1.4);
      }
    }
    ctx.globalCompositeOperation = 'source-over';
  }
};

/* ── Game Boy studies ── */
export const GB = ['#0F380F', '#306230', '#8BAC0F', '#9BBC0F'];   // darkest → lightest
const GB_PAL = { ink: '15,56,15', ground: '#9BBC0F', acc: '#306230', fail: '15,56,15' };
const BAYER = [0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5];
let lcdPlaying = true;
export function lcdTogglePlay() { const v = getClip(); lcdPlaying = !lcdPlaying; lcdPlaying ? v.play() : v.pause(); return lcdPlaying; }
export function lcdStep(delta) {
  const v = getClip(), i = SEGMENTS.indexOf(currentSegment(v.currentTime));
  const g = SEGMENTS[(i + delta + SEGMENTS.length) % SEGMENTS.length];
  v.currentTime = g.start + 0.01;
}

/* The clip on a 160×144 four-shade LCD, ordered-dithered, with a game text box. */
STUDIES.lcd = {
  init() {
    getClip();
    const small = document.createElement('canvas'); small.width = 160; small.height = 144;
    const read = document.createElement('canvas'); read.width = 160; read.height = 112;
    return { last: 0, small, read };
  },
  draw(ctx, s, t) {
    const vid = getClip();
    if (t - s.data.last < 1 / 15 || vid.readyState < 2) return;
    s.data.last = t;
    const { small, read } = s.data, sc = small.getContext('2d'), rc = read.getContext('2d', { willReadFrequently: true });
    const iw = vid.videoWidth, ih = vid.videoHeight, sw = Math.min(iw, ih * 160 / 112);
    rc.drawImage(vid, (iw - sw) / 2, 0, sw, ih, 0, 0, 160, 112);
    const px = rc.getImageData(0, 0, 160, 112).data, out = sc.createImageData(160, 144);
    const rgb = GB.map(h => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16)));
    for (let y = 0; y < 144; y++) for (let x = 0; x < 160; x++) {
      let lvl = 3;
      if (y < 112) {
        const i = (y * 160 + x) * 4, l = (0.2126 * px[i] + 0.7152 * px[i + 1] + 0.0722 * px[i + 2]) / 255;
        const v = Math.min(1, Math.max(0, (l - 0.08) / 0.8)) * 3 + (BAYER[(y & 3) * 4 + (x & 3)] / 16 - 0.5) * 0.9;
        lvl = Math.max(0, Math.min(3, Math.round(v)));
      }
      const o = (y * 160 + x) * 4, c = rgb[lvl];
      out.data[o] = c[0]; out.data[o + 1] = c[1]; out.data[o + 2] = c[2]; out.data[o + 3] = 255;
    }
    sc.putImageData(out, 0, 0);
    // text box: the current subtask and its verdict
    const seg = currentSegment(vid.currentTime), k = SEGMENTS.indexOf(seg);
    sc.fillStyle = GB[3]; sc.fillRect(2, 114, 156, 28);
    sc.strokeStyle = GB[0]; sc.lineWidth = 1; sc.strokeRect(2.5, 114.5, 155, 27); sc.strokeRect(4.5, 116.5, 151, 23);
    sc.fillStyle = GB[0]; sc.font = '8px "Press Start 2P", monospace'; sc.textBaseline = 'top';
    sc.fillText(seg.label.toUpperCase().slice(0, 17), 8, 120);
    sc.fillStyle = GB[1]; sc.fillText(`${k + 1}/8`, 8, 130);
    sc.fillStyle = seg.fail ? GB[0] : GB[1];
    if (seg.fail) { sc.fillRect(118, 129, 34, 10); sc.fillStyle = GB[3]; }
    sc.fillText(seg.fail ? 'FAIL' : ' OK', 120, 130);
    if (!lcdPlaying && (t * 2 | 0) % 2) { sc.fillStyle = GB[0]; sc.fillRect(146, 120, 2, 6); sc.fillRect(150, 120, 2, 6); }
    // upscale with hard pixels, then a faint pixel grid
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(small, 0, 0, s.w, s.h);
    const cell = s.w / 160;
    if (cell >= 2.5) {
      ctx.fillStyle = 'rgba(15,56,15,.10)';
      for (let x = 0; x <= 160; x++) ctx.fillRect(x * cell - 0.5, 0, 1, s.h);
      for (let y = 0; y <= 144; y++) ctx.fillRect(0, y * cell - 0.5, s.w, 1);
    }
    s.opts.onFrame && s.opts.onFrame(vid, seg);
  }
};

/* A circuit board: traces with 45° bends, vias, chips. Drawn once; it sits under clear plastic. */
STUDIES.pcb = {
  init(s, r) { return { r, done: false }; },
  draw(ctx, s) {
    if (s.data.done) return;
    s.data.done = true;
    const r = s.data.r, g = 14, cols = Math.ceil(s.w / g), rows = Math.ceil(s.h / g);
    ctx.clearRect(0, 0, s.w, s.h);
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    const chips = Array.from({ length: s.opts.chips ?? 3 }, () => {
      const w = (4 + (r() * 5 | 0)) * g, h = (3 + (r() * 3 | 0)) * g;
      return { x: Math.round(r() * (s.w - w) / g) * g, y: Math.round(r() * (s.h - h) / g) * g, w, h };
    });
    const dirs = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]];
    for (let n = 0; n < (s.opts.traces ?? 70); n++) {
      let x = (r() * cols | 0) * g, y = (r() * rows | 0) * g, d = (r() * 4 | 0) * 2;
      ctx.beginPath(); ctx.moveTo(x, y);
      const len = 6 + (r() * 24 | 0);
      for (let i = 0; i < len; i++) {
        if (r() < 0.22) d = (d + (r() < 0.5 ? 1 : 7)) % 8;
        x += dirs[d][0] * g; y += dirs[d][1] * g; ctx.lineTo(x, y);
      }
      ctx.strokeStyle = 'rgba(28,70,52,.42)'; ctx.lineWidth = r() < 0.2 ? 3 : 1.6; ctx.stroke();
      ctx.fillStyle = 'rgba(196,160,72,.85)'; ctx.beginPath(); ctx.arc(x, y, 2.6, 0, 6.283); ctx.fill();
      ctx.fillStyle = 'rgba(28,70,52,.6)'; ctx.beginPath(); ctx.arc(x, y, 1.1, 0, 6.283); ctx.fill();
    }
    chips.forEach((c, i) => {
      ctx.fillStyle = 'rgba(196,160,72,.8)';
      for (let x = c.x + 6; x < c.x + c.w - 4; x += 7) { ctx.fillRect(x, c.y - 5, 3, 5); ctx.fillRect(x, c.y + c.h, 3, 5); }
      ctx.fillStyle = 'rgba(22,22,30,.82)'; ctx.fillRect(c.x, c.y, c.w, c.h);
      ctx.fillStyle = 'rgba(255,255,255,.35)'; ctx.font = '500 9px "IBM Plex Mono", monospace';
      ctx.fillText(['SW-CPU', 'LBL-RAM', 'SEG 08', 'Q-CHK'][i % 4], c.x + 8, c.y + 16);
      ctx.beginPath(); ctx.arc(c.x + 8, c.y + c.h - 8, 2.5, 0, 6.283); ctx.fill();
    });
  }
};

/* ── Atomic purple: a colour LCD, and the parts inside the shell ── */
const GBC_PAL = { ink: '36,32,63', ground: '#DCDFCB', acc: '#7A4FC0', fail: '200,40,70' };

/* The clip on a 160×144 colour LCD: hard pixels, 5 bits per channel, a game text box. */
STUDIES.gbc = {
  init() {
    getClip();
    const small = document.createElement('canvas'); small.width = 160; small.height = 144;
    return { last: 0, small };
  },
  draw(ctx, s, t) {
    const vid = getClip();
    if (t - s.data.last < 1 / 20 || vid.readyState < 2) return;
    s.data.last = t;
    const sc = s.data.small.getContext('2d', { willReadFrequently: true });
    const iw = vid.videoWidth, ih = vid.videoHeight, sw = Math.min(iw, ih * 160 / 114);
    sc.imageSmoothingEnabled = true;
    sc.drawImage(vid, (iw - sw) / 2, 0, sw, ih, 0, 0, 160, 114);
    const im = sc.getImageData(0, 0, 160, 114), d = im.data;
    for (let i = 0; i < d.length; i += 4) for (let c = 0; c < 3; c++) {
      const v = Math.min(255, d[i + c] * 1.12 + 14);            // the screen is a little washed out
      d[i + c] = Math.round(v / 8) * 8;                          // 32 levels per channel
    }
    sc.putImageData(im, 0, 0);
    const seg = currentSegment(vid.currentTime), k = SEGMENTS.indexOf(seg);
    sc.fillStyle = '#F4F2EA'; sc.fillRect(0, 114, 160, 30);
    sc.strokeStyle = '#24203F'; sc.lineWidth = 1; sc.strokeRect(2.5, 116.5, 155, 25);
    sc.fillStyle = '#24203F'; sc.font = '8px "Press Start 2P", monospace'; sc.textBaseline = 'top';
    sc.fillText(seg.label.toUpperCase().slice(0, 17), 7, 120);
    sc.fillStyle = '#7A4FC0'; sc.fillText(`${k + 1}/8`, 7, 131);
    if (seg.fail) { sc.fillStyle = '#C82846'; sc.fillRect(116, 129, 38, 11); sc.fillStyle = '#fff'; sc.fillText('FAIL', 119, 131); }
    else { sc.fillStyle = '#2F8F5B'; sc.fillText('PASS', 119, 131); }
    if (!lcdPlaying && (t * 2 | 0) % 2) { sc.fillStyle = '#24203F'; sc.fillRect(148, 4, 3, 8); sc.fillRect(153, 4, 3, 8); }
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(s.data.small, 0, 0, s.w, s.h);
    const cell = s.w / 160;
    if (cell >= 2.5) {
      ctx.fillStyle = 'rgba(0,0,0,.07)';
      for (let x = 0; x <= 160; x++) ctx.fillRect(x * cell - 0.5, 0, 1, s.h);
      for (let y = 0; y <= 144; y++) ctx.fillRect(0, y * cell - 0.5, s.w, 1);
    }
    s.opts.onFrame && s.opts.onFrame(vid, seg);
  }
};

/* What you see through the shell: board patches, screw posts, molded rings and ribs. Drawn once. */
STUDIES.internals = {
  init(s, r) { return { r, done: false }; },
  draw(ctx, s) {
    if (s.data.done) return;
    s.data.done = true;
    const r = s.data.r, W = s.w, H = s.h;
    ctx.clearRect(0, 0, W, H);
    ctx.filter = 'blur(1.2px)';
    const deep = (a) => `rgba(74,40,138,${a})`;
    // long molded ribs
    for (let i = 0; i < H / 260; i++) {
      const x = r() * W * 0.8, y = r() * H, w = 120 + r() * 380, h = 10 + r() * 16;
      ctx.strokeStyle = deep(0.16); ctx.lineWidth = 2; ctx.beginPath(); ctx.roundRect(x, y, w, h, h / 2); ctx.stroke();
    }
    // board patches at the edges, as if the board shows at the sides of the shell
    ctx.filter = 'blur(.6px)';
    const nb = Math.max(2, Math.round(H / 1100));
    for (let n = 0; n < nb; n++) {
      const bw = 260 + r() * 220, bh = 200 + r() * 180, left = n % 2 === 0, peek = W < 700 ? 34 : Math.min(140, W * 0.09);
      const bx = left ? peek - bw : W - peek, by = (n + 0.25 + r() * 0.5) * H / nb - bh / 2;
      ctx.fillStyle = 'rgba(34,112,104,.42)'; ctx.beginPath(); ctx.roundRect(bx, by, bw, bh, 14); ctx.fill();
      ctx.strokeStyle = 'rgba(20,70,66,.5)'; ctx.lineWidth = 2; ctx.stroke();
      ctx.lineCap = 'round';
      for (let i = 0; i < 34; i++) {                       // traces with right-angle bends
        let x = bx + 12 + r() * (bw - 24), y = by + 12 + r() * (bh - 24);
        ctx.strokeStyle = r() < 0.7 ? 'rgba(206,176,92,.6)' : 'rgba(150,220,200,.45)'; ctx.lineWidth = r() < 0.2 ? 2.6 : 1.3;
        ctx.beginPath(); ctx.moveTo(x, y);
        for (let j = 0; j < 4; j++) {
          if (j % 2) x = Math.min(bx + bw - 10, Math.max(bx + 10, x + (r() - 0.5) * 140));
          else y = Math.min(by + bh - 10, Math.max(by + 10, y + (r() - 0.5) * 100));
          ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.fillStyle = 'rgba(214,184,100,.75)'; ctx.beginPath(); ctx.arc(x, y, 2.4, 0, 6.283); ctx.fill();
      }
      for (let i = 0; i < 4; i++) {                        // chips with legs
        const cw = 34 + r() * 46, ch = 22 + r() * 26, cx = bx + 16 + r() * (bw - cw - 32), cy = by + 16 + r() * (bh - ch - 32);
        ctx.fillStyle = 'rgba(214,184,100,.7)';
        for (let x = cx + 4; x < cx + cw - 3; x += 6) { ctx.fillRect(x, cy - 4, 2.5, 4); ctx.fillRect(x, cy + ch, 2.5, 4); }
        ctx.fillStyle = 'rgba(26,24,40,.72)'; ctx.fillRect(cx, cy, cw, ch);
      }
      for (let i = 0; i < 10; i++) {                       // small parts
        const x = bx + r() * bw, y = by + r() * bh;
        ctx.fillStyle = r() < 0.5 ? 'rgba(120,80,40,.6)' : 'rgba(220,220,230,.5)'; ctx.fillRect(x, y, 8, 4);
      }
    }
    ctx.filter = 'blur(.8px)';
    // molded screw posts: a deeper disc, a ring, a hole
    for (let i = 0; i < H / 260; i++) {
      const x = r() < 0.5 ? r() * W * 0.09 : W - r() * W * 0.09, y = r() * H, R = 14 + r() * 16;
      ctx.fillStyle = deep(0.2); ctx.beginPath(); ctx.arc(x, y, R, 0, 6.283); ctx.fill();
      ctx.strokeStyle = deep(0.32); ctx.lineWidth = 3; ctx.stroke();
      ctx.fillStyle = 'rgba(40,20,80,.35)'; ctx.beginPath(); ctx.arc(x, y, R * 0.3, 0, 6.283); ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,.25)'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, R - 2, 3.6, 5.2); ctx.stroke();
    }
    ctx.filter = 'none';
  }
};

/* ── Data sources, animated: VR teleop, leader/follower, UMI grippers, egocentric ── */
function design(ctx, s, c) {
  // a 240×160 design space, centred in the canvas
  ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
  const k = Math.min(s.w / 240, s.h / 160);
  ctx.save(); ctx.translate((s.w - 240 * k) / 2, (s.h - 160 * k) / 2); ctx.scale(k, k);
  ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.lineWidth = s.opts.fx === 'ascii' ? 5 : 2;   // thick strokes read better as characters
  return { ink: `rgb(${c.ink})`, dim: `rgba(${c.ink},.45)`, ground: c.ground, acc: c.acc };
}
function ik(bx, by, tx, ty, L1, L2, up = 1) {
  const dx = tx - bx, dy = ty - by, d = Math.max(Math.abs(L1 - L2) + 1, Math.min(L1 + L2 - 1, Math.hypot(dx, dy)));
  const a = Math.atan2(dy, dx), b = Math.acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
  const sh = a - b * up;
  const ex = bx + L1 * Math.cos(sh), ey = by + L1 * Math.sin(sh);
  const ea = Math.atan2(ty - ey, tx - ex);
  return { ex, ey, wx: ex + L2 * Math.cos(ea), wy: ey + L2 * Math.sin(ea), ea };
}
function drawArm(ctx, P, bx, by, arm, open, baseW = 52) {
  ctx.strokeStyle = P.ink;
  ctx.strokeRect(bx - baseW / 2, by, baseW, 7);
  ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(arm.ex, arm.ey); ctx.lineTo(arm.wx, arm.wy); ctx.stroke();
  // two fingers, opening across the forearm direction
  const n = arm.ea + Math.PI / 2, f = 4 + open * 6, L = 11;
  for (const sg of [-1, 1]) {
    const x0 = arm.wx + Math.cos(n) * f * sg, y0 = arm.wy + Math.sin(n) * f * sg;
    ctx.beginPath(); ctx.moveTo(arm.wx, arm.wy); ctx.lineTo(x0, y0); ctx.lineTo(x0 + Math.cos(arm.ea) * L, y0 + Math.sin(arm.ea) * L); ctx.stroke();
  }
  for (const [x, y, r] of [[bx, by, 5.5], [arm.ex, arm.ey, 5.5], [arm.wx, arm.wy, 4.5]]) {
    ctx.fillStyle = P.ground; ctx.beginPath(); ctx.arc(x, y, r, 0, 6.283); ctx.fill(); ctx.stroke();
  }
}
function signal(ctx, P, x0, x1, y, t) {
  ctx.strokeStyle = P.dim; ctx.setLineDash([3, 5]); ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = P.acc;
  for (let i = 0; i < 3; i++) { const u = ((t * 0.6 + i / 3) % 1); ctx.beginPath(); ctx.arc(x0 + (x1 - x0) * u, y, 2.6, 0, 6.283); ctx.fill(); }
}
function box(ctx, P, x, y, w = 13) { ctx.fillStyle = P.acc; ctx.strokeStyle = P.ink; ctx.fillRect(x - w / 2, y - w / 2, w, w); ctx.strokeRect(x - w / 2, y - w / 2, w, w); }

STUDIES.vr = {
  draw(ctx, s, t, c) {
    const P = design(ctx, s, c);
    const hx = 64 + Math.sin(t * 1.1) * 10, hy = 104 + Math.cos(t * 1.4) * 8;          // the operator's right hand
    const look = Math.sin(t * 0.7) * 3;
    ctx.strokeStyle = P.ink;
    ctx.beginPath(); ctx.moveTo(18, 140); ctx.quadraticCurveTo(46, 102, 76, 140); ctx.stroke();       // shoulders
    ctx.beginPath(); ctx.arc(46 + look * 0.3, 70, 16, 0, 6.283); ctx.stroke();                         // head
    ctx.fillStyle = P.ink; ctx.beginPath(); ctx.roundRect(30 + look, 62, 34, 14, 5); ctx.fill();       // headset
    ctx.beginPath(); ctx.moveTo(28 + look * 0.3, 66); ctx.quadraticCurveTo(46, 50, 64 + look * 0.3, 66); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(30, 116); ctx.lineTo(22, 112); ctx.moveTo(62, 116); ctx.lineTo(hx, hy); ctx.stroke();
    for (const [x, y] of [[22, 108], [hx, hy - 4]]) { ctx.strokeRect(x - 5, y, 10, 15); ctx.beginPath(); ctx.arc(x, y - 3, 6, 0, 6.283); ctx.stroke(); }
    signal(ctx, P, 92, 136, 92, t);
    const lag = t - 0.25, tx = 196 + Math.sin(lag * 1.1) * 18, ty = 86 + Math.cos(lag * 1.4) * 16;      // the follower copies the hand
    const grip = (Math.sin(t * 1.4) + 1) / 2;
    drawArm(ctx, P, 176, 134, ik(176, 134, tx, ty, 38, 36), grip);
    box(ctx, P, 222, 126);
    ctx.restore();
  }
};

STUDIES.leader = {
  init() { return { hist: [] }; },
  draw(ctx, s, t, c) {
    const P = design(ctx, s, c);
    const q = (u) => [52 + Math.sin(u * 1.2) * 12, 92 + Math.cos(u * 0.9) * 10];
    const [lx, ly] = q(t), [fx, fy] = q(t - 0.35);
    const g = (u) => (Math.sin(u * 1.6) + 1) / 2;
    const leader = ik(36, 134, lx, ly, 26, 24);
    drawArm(ctx, P, 36, 134, leader, 0, 36);
    // a handle with a trigger on the leader
    ctx.strokeStyle = P.ink; ctx.save(); ctx.translate(leader.wx, leader.wy); ctx.rotate(leader.ea);
    ctx.strokeRect(2, -4, 16, 8); ctx.beginPath(); ctx.moveTo(8, 4); ctx.lineTo(10 + g(t) * 3, 10); ctx.stroke(); ctx.restore();
    signal(ctx, P, 92, 132, 120, t);
    const follower = ik(178, 134, 120 + fx * 1.05, fy * 0.9, 42, 40);
    drawArm(ctx, P, 178, 134, follower, g(t - 0.35));
    box(ctx, P, 224, 126);
    ctx.restore();
  }
};

STUDIES.umi = {
  draw(ctx, s, t, c) {
    const P = design(ctx, s, c);
    const ph = (t * 0.45) % 1;                                                      // reach, close, lift, open
    const reach = ph < 0.25 ? ph / 0.25 : ph < 0.75 ? 1 : 1 - (ph - 0.75) / 0.25;
    const close = ph < 0.25 ? 0 : ph < 0.35 ? (ph - 0.25) / 0.1 : ph < 0.7 ? 1 : Math.max(0, 1 - (ph - 0.7) / 0.08);
    const lift = ph < 0.35 ? 0 : ph < 0.55 ? (ph - 0.35) / 0.2 : ph < 0.7 ? 1 : Math.max(0, 1 - (ph - 0.7) / 0.1);
    const y = 86 - lift * 26, gap = 46 - reach * 18;
    for (const sg of [-1, 1]) {
      ctx.save(); ctx.translate(120 + sg * gap, y); ctx.scale(-sg, 1);
      ctx.strokeStyle = P.ink;
      ctx.beginPath(); ctx.roundRect(4, -10, 44, 20, 5); ctx.stroke();                  // body
      ctx.save(); ctx.translate(30, 10); ctx.rotate(0.25); ctx.beginPath(); ctx.roundRect(-6, 0, 12, 34, 5); ctx.stroke(); ctx.restore();  // handle
      ctx.fillStyle = P.ink; ctx.beginPath(); ctx.roundRect(16, -22, 18, 11, 3); ctx.fill();           // camera
      ctx.fillStyle = P.acc; ctx.beginPath(); ctx.arc(25, -16.5, 3, 0, 6.283); ctx.fill();
      const o = 9 - close * 5;                                                           // fin-ray fingers
      ctx.beginPath(); ctx.moveTo(4, -o); ctx.lineTo(-14, -o + 3); ctx.lineTo(-10, -o + 8); ctx.closePath();
      ctx.moveTo(4, o); ctx.lineTo(-14, o - 3); ctx.lineTo(-10, o - 8); ctx.closePath(); ctx.stroke();
      ctx.restore();
    }
    box(ctx, P, 120, y, 14);
    ctx.strokeStyle = P.dim; ctx.beginPath(); ctx.moveTo(40, 132); ctx.lineTo(200, 132); ctx.stroke();
    ctx.restore();
  }
};

STUDIES.ego = {
  draw(ctx, s, t, c) {
    const P = design(ctx, s, c);
    const turn = Math.sin(t * 0.6) * 10;
    ctx.strokeStyle = P.ink;
    ctx.beginPath(); ctx.arc(120 + turn * 0.3, 34, 20, 0, 6.283); ctx.stroke();
    ctx.fillStyle = P.ink; ctx.beginPath(); ctx.roundRect(110 + turn * 0.5, 8, 20, 12, 3); ctx.fill();
    ctx.fillStyle = P.acc; ctx.beginPath(); ctx.arc(120 + turn * 0.5, 14, 3, 0, 6.283); ctx.fill();
    // what the camera sees
    const cx = 120 + turn * 2.4;
    ctx.strokeStyle = P.dim; ctx.setLineDash([3, 5]); ctx.beginPath();
    ctx.moveTo(114 + turn * 0.5, 22); ctx.lineTo(cx - 62, 146); ctx.moveTo(126 + turn * 0.5, 22); ctx.lineTo(cx + 62, 146); ctx.stroke(); ctx.setLineDash([]);
    // two hands pass an object back and forth
    const u = (Math.sin(t * 0.9) + 1) / 2, left = [86 + u * 10, 120 - Math.sin(u * Math.PI) * 10], right = [154 - (1 - u) * 10, 120 - Math.sin(u * Math.PI) * 10];
    const hand = (x, y, sg) => {
      ctx.strokeStyle = P.ink; ctx.fillStyle = P.ground; ctx.beginPath();
      ctx.moveTo(x - sg * 18, y + 24); ctx.lineTo(x - sg * 6, y + 6); ctx.quadraticCurveTo(x, y - 8, x + sg * 12, y - 2); ctx.lineTo(x + sg * 8, y + 8);
      ctx.quadraticCurveTo(x + sg * 4, y + 18, x - sg * 4, y + 26); ctx.stroke();
    };
    hand(...left, 1); hand(...right, -1);
    const ox = left[0] + 14 + (right[0] - 14 - left[0] - 14) * u;
    box(ctx, P, ox, Math.min(left[1], right[1]) - 4, 14);
    ctx.restore();
  }
};

/* Two studies in one canvas slot, cross-fading on a loop. */
export function loopPair(a, b, period = 6000) {
  let on = 0;
  setInterval(() => { on ^= 1; a.classList.toggle('off', on === 1); b.classList.toggle('off', on === 0); }, period);
  b.classList.add('off');
}

/* ── Post-processing: draw a study off screen, then show it dithered or as characters ── */
const FX_WORDS = { vr: 'TELEOP·', leader: 'LEADER·FOLLOWER·', umi: 'UMI·GRIPPER·', ego: 'EGOCENTRIC·' };
const hexRGB = (h) => [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16));
function fxFrame(kind, fx, src, ctx, s, c) {
  const W = src.width, H = src.height, d = src.getContext('2d', { willReadFrequently: true }).getImageData(0, 0, W, H).data;
  const g = hexRGB(c.ground), acc = hexRGB(c.acc), ink = c.ink.split(',').map(Number);
  // how far a pixel is from the ground (0..1), and whether it is the yellow accent
  const at = (x, y) => {
    const i = (y * W + x) * 4, r = d[i], gg = d[i + 1], b = d[i + 2];
    const dist = (Math.abs(r - g[0]) + Math.abs(gg - g[1]) + Math.abs(b - g[2])) / 3 / 255;
    const isAcc = Math.abs(r - acc[0]) + Math.abs(gg - acc[1]) + Math.abs(b - acc[2]) < 140;
    return [Math.min(1, dist * 1.8), isAcc];
  };
  ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
  if (fx === 'dither') {
    const px = 2, cw = Math.floor(W / px), ch = Math.floor(H / px);
    const small = s.data.fxSmall || (s.data.fxSmall = document.createElement('canvas'));
    small.width = cw; small.height = ch;
    const sc = small.getContext('2d'), out = sc.createImageData(cw, ch);
    for (let y = 0; y < ch; y++) for (let x = 0; x < cw; x++) {
      const [v, isAcc] = at(x * px, y * px), o = (y * cw + x) * 4;
      const on = v > (BAYER[(y & 3) * 4 + (x & 3)] + 0.5) / 16;
      const col = !on ? g : isAcc ? acc : ink;
      out.data[o] = col[0]; out.data[o + 1] = col[1]; out.data[o + 2] = col[2]; out.data[o + 3] = 255;
    }
    sc.putImageData(out, 0, 0);
    ctx.imageSmoothingEnabled = false; ctx.drawImage(small, 0, 0, s.w, s.h);
  } else {
    const word = FX_WORDS[kind] || 'SHOTWELL·', cell = Math.max(7, s.w / 32), rows = Math.floor(s.h / cell), cols = Math.floor(s.w / cell);
    ctx.font = `${Math.round(cell)}px "Pokemon GB", monospace`; ctx.textBaseline = 'top';
    let n = 0;
    for (let r = 0; r < rows; r++) for (let q = 0; q < cols; q++) {
      // coverage of the cell: sample a 3×3 grid inside it
      let sum = 0, accs = 0;
      for (let j = 0; j < 3; j++) for (let i = 0; i < 3; i++) {
        const [v, isAcc] = at(Math.min(W - 1, Math.floor((q + (i + 0.5) / 3) * cell * W / s.w)), Math.min(H - 1, Math.floor((r + (j + 0.5) / 3) * cell * H / s.h)));
        sum += v; if (isAcc && v > 0.2) accs++;
      }
      const v = sum / 9;
      if (v < 0.1) { n++; continue; }
      ctx.fillStyle = accs >= 3 ? c.acc : `rgba(${c.ink},${Math.min(1, 0.35 + v)})`;
      ctx.fillText(word[(n++) % word.length], q * cell, r * cell);
    }
  }
}

/* ── The data-source runner: pixel scenes on one ground line, like the offline dino game ── */
const SCENE_W = 72, SCENE_H = 44;   // pixel units per scene
function pix(sc) {
  const P = {
    dot(x, y) { sc.fillRect(Math.round(x), Math.round(y), 1, 1); },
    line(x0, y0, x1, y1, w = 1) {
      x0 = Math.round(x0); y0 = Math.round(y0); x1 = Math.round(x1); y1 = Math.round(y1);
      const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
      let e = dx + dy;
      for (;;) {
        sc.fillRect(x0, y0, w, w);
        if (x0 === x1 && y0 === y1) break;
        const e2 = 2 * e;
        if (e2 >= dy) { e += dy; x0 += sx; }
        if (e2 <= dx) { e += dx; y0 += sy; }
      }
    },
    dotted(x0, y0, x1, y1, gap = 3, phase = 0) {
      const n = Math.max(1, Math.round(Math.hypot(x1 - x0, y1 - y0) / gap));
      for (let i = 0; i <= n; i++) if ((i + phase) % 2 === 0) P.dot(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
    },
    rect(x, y, w, h) { sc.fillRect(Math.round(x), Math.round(y), Math.round(w), Math.round(h)); },
    frame(x, y, w, h) { P.rect(x, y, w, 1); P.rect(x, y + h - 1, w, 1); P.rect(x, y, 1, h); P.rect(x + w - 1, y, 1, h); },
    ring(cx, cy, r) { for (let a = 0; a < 6.283; a += 0.25) P.dot(cx + Math.cos(a) * r, cy + Math.sin(a) * r); },
    disc(cx, cy, r) { for (let y = -r; y <= r; y++) for (let x = -r; x <= r; x++) if (x * x + y * y <= r * r + r) P.dot(cx + x, cy + y); },
  };
  return P;
}
// a small pixel arm: base plate, two links, joints, two fingers
function pxArm(P, sc, ink, bx, by, tx, ty, L1, L2, open) {
  const a = ik(bx, by, tx, ty, L1, L2);
  sc.fillStyle = ink;
  P.rect(bx - 6, by, 13, 2);
  P.line(bx, by, a.ex, a.ey, 2); P.line(a.ex, a.ey, a.wx, a.wy, 2);
  P.rect(a.ex - 1, a.ey - 1, 3, 3); P.rect(bx - 1, by - 2, 3, 3);
  const n = a.ea + Math.PI / 2, f = 1.5 + open * 2.5;
  for (const sg of [-1, 1]) {
    const x0 = a.wx + Math.cos(n) * f * sg, y0 = a.wy + Math.sin(n) * f * sg;
    P.line(a.wx, a.wy, x0, y0); P.line(x0, y0, x0 + Math.cos(a.ea) * 4, y0 + Math.sin(a.ea) * 4);
  }
  return a;
}
// a standing person, facing right; returns the shoulder point
function pxPerson(P, sc, ink, x, gy, t, walk = false) {
  sc.fillStyle = ink;
  const hip = gy - 13, sh = gy - 24;
  const sw = walk ? Math.sin(t * 9) * 4 : 2;   // standing: feet a little apart
  P.line(x, hip, x + sw, gy - 1, 2); P.line(x, hip, x - sw, gy - 1, 2);            // legs
  P.line(x, hip, x, sh, 2);                                                         // body
  P.disc(x + 1, sh - 5, 3);                                                         // head
  return { x, y: sh + 1, head: [x + 1, sh - 5] };
}

// five-fingered hand sprites, palm toward the viewer, fingers down; thumb on the left
const HAND_OPEN = [
  '...###...',
  '..#####..',
  '..#####..',
  '#.#####..',
  '.##.#.#.#',
  '..#.#.#.#',
  '..#.#.#.#',
  '..#.#.#..',
  '....#....',
];
const HAND_CLOSED = [
  '...###...',
  '..#####..',
  '.######..',
  '.#######.',
  '..######.',
  '..#.#.#..',
];
// an arm that ends in a five-fingered hand; grip 0 = open, 1 = closed
function pxHandArm(P, sc, ink, bx, by, tx, ty, L1, L2, grip) {
  const a = ik(bx, by, tx, ty, L1, L2);
  sc.fillStyle = ink;
  P.rect(bx - 6, by, 13, 2); P.rect(bx - 1, by - 2, 3, 3);
  P.line(bx, by, a.ex, a.ey, 2); P.line(a.ex, a.ey, a.wx, a.wy, 2); P.rect(a.ex - 1, a.ey - 1, 3, 3);
  // the hand: two fixed sprites (open, closed), pointing down from the wrist; rotated pixels turn to noise at this size
  const sprite = grip > 0.5 ? HAND_CLOSED : HAND_OPEN, ox = Math.round(a.wx) - 4, oy = Math.round(a.wy) - 1;
  sprite.forEach((row, y) => [...row].forEach((ch, x) => { if (ch === '#') P.dot(ox + x, oy + y); }));
  return a;
}
export const SCENES = {
  vr(P, sc, c, t) {
    const gy = SCENE_H - 1, ink = `rgb(${c.ink})`;
    const me = pxPerson(P, sc, ink, 12, gy, t);
    sc.fillStyle = ink; P.rect(me.head[0] - 1, me.head[1] - 2, 6, 3);               // headset
    const hx = 21 + Math.sin(t * 1.3) * 3, hy = me.y + 6 + Math.cos(t * 1.7) * 3;
    P.line(me.x, me.y + 1, hx, hy); P.rect(hx, hy - 3, 2, 4);                       // arm + controller
    P.line(me.x, me.y + 1, me.x + 4, me.y + 9); P.rect(me.x + 4, me.y + 6, 2, 4);
    sc.fillStyle = c.acc; P.rect(hx, hy - 4, 2, 1); P.rect(me.x + 4, me.y + 5, 2, 1);
    sc.fillStyle = `rgba(${c.ink},.55)`; P.dotted(28, 20, 40, 20, 2, Math.floor(t * 6) % 2);
    const lag = t - 0.3;
    pxHandArm(P, sc, ink, 52, gy - 2, 58 + Math.sin(lag * 1.3) * 6, 22 + Math.cos(lag * 1.7) * 5, 13, 12, (Math.sin(t * 1.5) + 1) / 2);
    sc.fillStyle = c.acc; P.rect(63, gy - 6, 5, 5); sc.fillStyle = ink; P.frame(63, gy - 6, 5, 5);
  },
  leader(P, sc, c, t) {
    const gy = SCENE_H - 1, ink = `rgb(${c.ink})`, q = (u) => [Math.sin(u * 1.2), Math.cos(u * 0.9)];
    const [a1, b1] = q(t), [a2, b2] = q(t - 0.4);
    const L = pxArm(P, sc, ink, 10, gy - 2, 18 + a1 * 4, 24 + b1 * 4, 9, 8, 0);
    sc.fillStyle = ink; P.rect(L.wx, L.wy - 1, 4, 3);                               // handle
    sc.fillStyle = c.acc; P.dot(L.wx + 1, L.wy + 2);
    sc.fillStyle = `rgba(${c.ink},.55)`; P.dotted(24, 30, 36, 30, 2, Math.floor(t * 6) % 2);
    pxArm(P, sc, ink, 50, gy - 2, 58 + a2 * 8, 18 + b2 * 7, 15, 14, (Math.sin((t - 0.4) * 1.6) + 1) / 2);
    sc.fillStyle = c.acc; P.rect(64, gy - 6, 5, 5); sc.fillStyle = ink; P.frame(64, gy - 6, 5, 5);
  },
  umi(P, sc, c, t, st = {}) {
    const gy = SCENE_H - 1, ink = `rgb(${c.ink})`, ph = (t * 0.5) % 1;
    const reach = ph < 0.3 ? ph / 0.3 : ph < 0.75 ? 1 : 1 - (ph - 0.75) / 0.25;
    const closed = ph > 0.32 && ph < 0.72, lift = ph < 0.4 ? 0 : ph < 0.55 ? (ph - 0.4) / 0.15 : ph < 0.68 ? 1 : Math.max(0, 1 - (ph - 0.68) / 0.08);
    const me = pxPerson(P, sc, ink, 14, gy, t);
    // table
    sc.fillStyle = ink; P.rect(38, gy - 12, 28, 2); P.rect(40, gy - 10, 2, 10); P.rect(62, gy - 10, 2, 10);
    // the gripper held out in front: body, camera, two fin-ray fingers
    const gx = 22 + reach * 10, gyy = me.y + 7 - lift * 6;
    P.line(me.x, me.y + 1, gx, gyy + 2);
    P.rect(gx, gyy, 7, 4); P.rect(gx + 2, gyy + 4, 2, 4);                           // body + grip
    P.rect(gx + 1, gyy - 3, 4, 3); sc.fillStyle = c.acc; P.dot(gx + 3, gyy - 2);    // wrist camera
    sc.fillStyle = ink; const o = closed ? 1 : 3;
    P.line(gx + 7, gyy + 2 - o, gx + 11, gyy + 2 - o); P.line(gx + 7, gyy + 2 + o, gx + 11, gyy + 2 + o);
    // the box: on the table, or in the fingers while lifted
    if (!st.dropped) {
      const bx = closed ? gx + 9 : 52, by = closed ? gyy : gy - 17;
      sc.fillStyle = c.acc; P.rect(bx - 2, by, 5, 5); sc.fillStyle = ink; P.frame(bx - 2, by, 5, 5);
    }
  },
  robot(P, sc, c, t) {
    // a deployed mobile robot: wheels, a mast, a head with a camera, one arm; it moves boxes from a shelf to its tray
    const gy = SCENE_H - 1, ink = `rgb(${c.ink})`, roll = Math.round(Math.sin(t * 0.5) * 2);
    sc.fillStyle = ink;
    const bx = 6 + roll;
    P.rect(bx, gy - 9, 22, 6);                                                          // base
    for (const wx of [bx + 4, bx + 17]) { P.disc(wx, gy - 2, 2); }
    sc.fillStyle = c.ground; P.dot(bx + 4, gy - 2); P.dot(bx + 17, gy - 2); sc.fillStyle = ink;
    P.rect(bx + 2, gy - 12, 8, 3);                                                      // tray
    P.rect(bx + 15, gy - 30, 2, 21);                                                    // mast
    P.rect(bx + 11, gy - 37, 10, 7);                                                    // head
    sc.fillStyle = c.ground; P.rect(bx + 13, gy - 35, 6, 3);
    sc.fillStyle = c.acc; P.rect(bx + 16, gy - 34, 2, 1);                               // camera
    if ((t * 2 | 0) % 2) { P.dot(bx + 20, gy - 7); }                                    // status light
    sc.fillStyle = ink; P.rect(44, gy - 18, 22, 2); P.rect(46, gy - 16, 2, 16); P.rect(62, gy - 16, 2, 16);   // shelf
    // pick from the shelf, carry to the tray, release, go back
    const ph = (t * 0.35) % 1, pick = [54, gy - 22], drop = [bx + 6, gy - 16];
    const ease = (u) => u * u * (3 - 2 * u);
    let tgt, carrying;
    if (ph < 0.3) { const u = ease(ph / 0.3); tgt = [drop[0] + (pick[0] - drop[0]) * u, drop[1] + (pick[1] - drop[1]) * u - Math.sin(u * Math.PI) * 6]; carrying = false; }
    else if (ph < 0.4) { tgt = pick; carrying = ph > 0.35; }
    else if (ph < 0.7) { const u = ease((ph - 0.4) / 0.3); tgt = [pick[0] + (drop[0] - pick[0]) * u, pick[1] + (drop[1] - pick[1]) * u - Math.sin(u * Math.PI) * 6]; carrying = true; }
    else { tgt = drop; carrying = ph < 0.75; }
    const sh = [bx + 17, gy - 24], arm = ik(sh[0], sh[1], tgt[0], tgt[1] + 3, 13, 12, -1);
    P.line(sh[0], sh[1], arm.ex, arm.ey, 2); P.line(arm.ex, arm.ey, arm.wx, arm.wy, 2); P.rect(arm.ex - 1, arm.ey - 1, 3, 3);
    P.line(arm.wx - 2, arm.wy, arm.wx - 2, arm.wy + 3); P.line(arm.wx + 2, arm.wy, arm.wx + 2, arm.wy + 3);
    const boxAt = carrying ? [arm.wx - 2, arm.wy + 2] : ph >= 0.7 ? [drop[0] - 2, drop[1] + 1] : [pick[0] - 2, pick[1] - 1];
    sc.fillStyle = c.acc; P.rect(boxAt[0], boxAt[1], 5, 5); sc.fillStyle = ink; P.frame(boxAt[0], boxAt[1], 5, 5);
  },
  ego(P, sc, c, t) {
    const gy = SCENE_H - 1, ink = `rgb(${c.ink})`;
    const me = pxPerson(P, sc, ink, 16, gy, t);
    sc.fillStyle = ink; P.rect(me.head[0], me.head[1] - 5, 4, 2);                   // head camera
    sc.fillStyle = c.acc; P.dot(me.head[0] + 3, me.head[1] - 4);
    sc.fillStyle = ink; P.rect(30, gy - 14, 34, 2); P.rect(32, gy - 12, 2, 12); P.rect(60, gy - 12, 2, 12);   // counter
    // the hands move a box from one side of the counter to the other, and back
    const u = (Math.sin(t * 0.8) + 1) / 2, hx = 36 + u * 18, hy = gy - 20 - Math.sin(u * Math.PI) * 5;
    P.line(me.x, me.y + 1, hx, hy + 1); P.line(me.x, me.y + 2, hx - 2, hy + 3);
    sc.fillStyle = c.acc; P.rect(hx - 1, hy - 1, 5, 5); sc.fillStyle = ink; P.frame(hx - 1, hy - 1, 5, 5);
    sc.fillStyle = `rgba(${c.ink},.5)`;                                              // what the camera sees
    P.dotted(me.head[0] + 4, me.head[1] - 4, 34, gy - 16, 2, Math.floor(t * 4) % 2);
    P.dotted(me.head[0] + 4, me.head[1] - 4, 62, gy - 16, 2, Math.floor(t * 4) % 2);
  },
};
const SCENE_ORDER = [['vr', 'VR TELEOP'], ['leader', 'LEADER / FOLLOWER'], ['umi', 'UMI GRIPPERS'], ['ego', 'EGOCENTRIC'], ['robot', 'DEPLOYMENT']];
const NOTES = {
  vr: ['0:02 REACH', '0:04 GRASP', '0:06 PLACE'],
  leader: ['0:01 MIRROR', '0:03 GRASP', '0:05 LIFT'],
  umi: ['0:01 REACH', '0:03 GRASP', '0:05 LIFT'],
};
const hash = (n) => { const x = Math.sin(n * 12.9898 + 4.1) * 43758.5453; return x - Math.floor(x); };
const slips = (n, kind) => hash(n) < (kind === 'umi' ? 0.4 : 0.2);

/* The egocentric runner stays in place and walks; the world scrolls past.
   A dashed playhead annotates each scene that passes it. Dropped boxes roll toward the runner;
   it jumps them by itself, or on Space / tap. A counter keeps the episodes seen since load. */
const GAME_API = (typeof window !== 'undefined' && window.SHOTWELL_GAME_API) || 'https://label.shotwell.ai';
async function fetchBoard() {
  try { const r = await fetch(GAME_API + '/api/game/leaderboard'); if (r.ok) return (await r.json()).top || []; } catch (e) {}
  return null;
}
async function submitScore(initials, score, runMs) {
  const r = await fetch(GAME_API + '/api/game/score', { method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ initials, score, run_ms: runMs }) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || 'could not save');
  return j;
}
// the three-letter entry: a small form over the strip, so it works with a phone keyboard too
function initialsForm(s, onDone) {
  const host = s.canvas.parentElement; host.style.position = 'relative';
  const f = document.createElement('form'); f.className = 'hiscore';
  f.innerHTML = '<label>NEW HIGH SCORE · YOUR 3 LETTERS</label><input maxlength="3" autocomplete="off" autocapitalize="characters" spellcheck="false" aria-label="Three letters"><button>SAVE</button><output></output>';
  host.appendChild(f);
  const inp = f.querySelector('input'), out = f.querySelector('output'), btn = f.querySelector('button');
  inp.addEventListener('input', () => { inp.value = inp.value.toUpperCase().replace(/[^A-Z]/g, '').slice(0, 3); });
  f.addEventListener('submit', async (e) => {
    e.preventDefault();
    if (!/^[A-Z]{3}$/.test(inp.value)) { out.textContent = 'THREE LETTERS, A TO Z'; return; }
    btn.disabled = true; out.textContent = 'SAVING…';
    try { const res = await onDone(inp.value); f.remove(); return res; }
    catch (err) { out.textContent = String(err.message || err).toUpperCase(); btn.disabled = false; }
  });
  setTimeout(() => inp.focus({ preventScroll: true }), 50);
  return f;
}
let AC = null, muted = false;
function beep(kind) {
  if (muted) return;
  try { AC = AC || new (window.AudioContext || window.webkitAudioContext)(); } catch (e) { return; }
  const notes = { jump: [[520, 0.06], [780, 0.06]], double: [[780, 0.05], [1040, 0.06]], get: [[988, 0.05], [1319, 0.08]],
                  combo: [[1319, 0.05], [1568, 0.05], [2093, 0.09]], fail: [[300, 0.1], [180, 0.18]], flip: [[660, 0.08], [880, 0.08], [660, 0.08]] }[kind];
  let at = AC.currentTime;
  for (const [f, d] of notes) {
    const o = AC.createOscillator(), g = AC.createGain();
    o.type = 'square'; o.frequency.value = f; g.gain.setValueAtTime(0.045, at); g.gain.exponentialRampToValueAtTime(0.001, at + d);
    o.connect(g).connect(AC.destination); o.start(at); o.stop(at + d); at += d * 0.9;
  }
}
STUDIES.runner = {
  init(s, r) {
    const small = document.createElement('canvas'); small.width = SCENE_W; small.height = SCENE_H;
    const me = document.createElement('canvas'); me.width = 40; me.height = SCENE_H + 2;
    const prev = s.data || {};
    const d = { small, me, r, specks: Array.from({ length: 90 }, () => [r() * 2000, 1 + (r() * 6 | 0), 1 + (r() * 3 | 0)]),
      clouds: Array.from({ length: 4 }, (_, i) => [i * 520 + r() * 200, 8 + r() * 40]), boxes: [], dropped: new Set(), lastT: null,
      game: prev.game || false, hi: prev.hi || 0, logo: null, calm0: null, board: prev.board || null, form: prev.form || null };
    for (const tone of ['black', 'white']) { const im = new Image(); im.src = `/brand/assets/SVG/shotwell-mark-${tone}.svg`; d['mark_' + tone] = im; }
    s.canvas.__s = s;                                    // for tests
    if (!s.canvas.__keys) {
      s.canvas.__keys = true;
      const onLogo = (e) => {
        const cur = s.data, rc = s.canvas.getBoundingClientRect(), x = e.clientX - rc.left, y = e.clientY - rc.top, L = cur.logo;
        return L && x >= L[0] - 6 && x <= L[0] + L[2] + 6 && y >= L[1] - 6 && y <= L[1] + L[3] + 6;
      };
      const press = () => {
        const g = s.data.run; if (!s.data.game || !g) return;
        if (g.over) { if (s.data.form) return; if (performance.now() - g.overAt > 400) s.data.run = newRun(); return; }
        if (g.jumpY === 0) { g.vy = 420; g.holding = true; g.jumps = 1; beep('jump'); }
        else if (g.jumps < 2) { g.vy = 380; g.holding = true; g.jumps = 2; beep('double'); }     // double jump
      };
      const release = () => { const g = s.data.run; if (g) { g.holding = false; if (g.vy > 160) g.vy = 160; } };   // a tap is a hop; hold for height
      s.canvas.addEventListener('pointerdown', (e) => {
        const cur = s.data;
        if (onLogo(e)) { cur.game = !cur.game; cur.run = cur.game ? newRun() : null; cur.form?.remove(); cur.form = null; if (cur.game) fetchBoard().then(b => { if (b) cur.board = b; }); return; }
        press();
      });
      addEventListener('pointerup', release);
      s.canvas.addEventListener('pointermove', (e) => { s.canvas.style.cursor = onLogo(e) || s.data.game ? 'pointer' : ''; });
      const live = (e) => {
        if (!s.data.game || !['Space', 'ArrowDown', 'KeyM'].includes(e.code) || e.target.closest('input,textarea,button,a')) return false;
        const rc = s.canvas.getBoundingClientRect(); return !(rc.bottom < 0 || rc.top > innerHeight);
      };
      addEventListener('keydown', (e) => {
        if (!live(e)) return; e.preventDefault();
        const g = s.data.run;
        if (e.code === 'ArrowDown') { if (g) g.duck = true; }
        else if (e.code === 'KeyM') { if (!e.repeat) muted = !muted; }
        else if (!e.repeat) press();
      });
      addEventListener('keyup', (e) => {
        if (!live(e)) return;
        if (e.code === 'ArrowDown') { const g = s.data.run; if (g) g.duck = false; } else if (e.code === 'Space') release();
      });
    }
    if (d.game) d.run = prev.run || newRun();
    return d;
    function newRun() { return { t: 0, speed: 150, score: 0, jumpY: 0, vy: 0, jumps: 0, duck: false, holding: false, combo: 0, items: [], pops: [], parts: [], next: 0.8, over: false, overAt: 0, phase: 0, flipAt: -9 }; }
  },
  draw(ctx, s, t, c) {
    const d = s.data, dt = d.lastT == null ? 0 : Math.min(0.05, t - d.lastT); d.lastT = t;
    const g = d.game ? d.run : null;
    if (g) {
      const phase = Math.floor(g.score / 500) % 2;
      if (phase !== g.phase) { g.phase = phase; g.flipAt = t; beep('flip'); }
      if (phase) c = (c === DARK || c === FROST_D) ? FROST_L : FROST_D;
    }
    ctx.fillStyle = c.ground; ctx.fillRect(0, 0, s.w, s.h);
    const k = s.w < 600 ? 3 : 4, spacing = (SCENE_W + 30) * k;
    const speed = g ? (g.over ? 0 : g.speed) : 34;
    d.shift = (d.shift || 0) + speed * dt;
    const shift = d.shift, gy = Math.round(s.h - 40), ink = `rgb(${c.ink})`, half = SCENE_W * k / 2;
    const playX = Math.round(s.w * 0.5), runX = Math.round(s.w * (s.w < 600 ? 0.06 : 0.08));
    ctx.imageSmoothingEnabled = false;
    ctx.font = `8px "Pokemon GB", monospace`; ctx.textBaseline = 'top';
    const box = (x, y, w, fill, edge) => { const e = Math.max(1, k / 2); ctx.fillStyle = fill; ctx.fillRect(x, y, w, w); ctx.fillStyle = edge; ctx.fillRect(x, y, w, e); ctx.fillRect(x, y + w - e, w, e); ctx.fillRect(x, y, e, w); ctx.fillRect(x + w - e, y, e, w); };

    ctx.fillStyle = `rgba(${c.ink},.22)`;
    for (const [cx, cy] of d.clouds) {
      const x = ((cx - shift * 0.35) % (s.w + 200) + s.w + 200) % (s.w + 200) - 100;
      for (const [dx, dy, w] of [[2, 0, 6], [0, 1, 10], [1, 2, 12]]) ctx.fillRect(Math.round(x + dx * k), Math.round(cy + 18 + dy * k), w * k, k);
    }
    ctx.fillStyle = ink; ctx.fillRect(0, gy, s.w, Math.max(1, k / 2));
    ctx.fillStyle = `rgba(${c.ink},.5)`;
    for (const [sx, dy, w] of d.specks) { const x = ((sx - shift) % 2000 + 2000) % 2000; if (x < s.w) ctx.fillRect(Math.round(x), gy + 3 + (dy % 3) * 2, w * k / 2, Math.max(1, k / 2)); }
    if (!g) { ctx.fillStyle = `rgba(${c.ink},.3)`; for (let y = 34; y < gy; y += 8) ctx.fillRect(playX, y, Math.max(1, k / 2), 4); }

    // the logo: the way in and out of the game
    const mark = d[(c === DARK || c === FROST_D) ? 'mark_white' : 'mark_black'];
    if (mark.complete && mark.naturalWidth) {
      const mh = 26, mw = mh * mark.naturalWidth / mark.naturalHeight, mx = 16, my = 8;
      ctx.globalAlpha = g ? 1 : 0.7; ctx.drawImage(mark, mx, my, mw, mh); ctx.globalAlpha = 1;
      d.logo = [mx, my, mw, mh];
      if (g) {
        ctx.fillStyle = `rgba(${c.ink},.6)`; ctx.fillText(`SPACE: JUMP, AGAIN IN AIR · DOWN: DUCK · M: ${muted ? 'SOUND ON' : 'MUTE'}`, mx + mw + 10, my + 4);
        ctx.fillStyle = c.acc; ctx.fillRect(mx + mw + 10, my + 17, 7, 7); ctx.fillStyle = `rgba(${c.ink},.6)`; ctx.fillText('COLLECT', mx + mw + 21, my + 17);
        ctx.fillStyle = ink; ctx.fillRect(mx + mw + 90, my + 17, 7, 7); ctx.fillStyle = `rgba(${c.ink},.6)`; ctx.fillText('AVOID', mx + mw + 101, my + 17);
      }
    }

    // the score
    let score;
    if (g) { if (!g.over) g.score += speed * dt / 4; score = Math.floor(g.score); d.hi = Math.max(d.hi, score); }
    else { if (d.calm0 == null) d.calm0 = shift; score = Math.floor((shift - d.calm0) / 3); }
    ctx.textAlign = 'right'; ctx.fillStyle = `rgba(${c.ink},.75)`;
    ctx.fillText((d.hi ? `HI ${String(d.hi).padStart(5, '0')}   ` : '') + (g ? 'SCORE ' : 'EPISODES ') + String(score).padStart(5, '0'), s.w - 16, 12);
    ctx.textAlign = 'left';

    // the scenes: full strength when calm, dimmed behind the game
    const sc = d.small.getContext('2d'), P = pix(sc), first = Math.floor(shift / spacing);
    if (g) ctx.globalAlpha = 0.28;
    for (let j = -1; j <= Math.ceil(s.w / spacing) + 1; j++) {
      const n = first + j, idx = ((n % SCENE_ORDER.length) + SCENE_ORDER.length) % SCENE_ORDER.length, [kind, label] = SCENE_ORDER[idx], x = Math.round(n * spacing - shift);
      if (x > s.w || x + SCENE_W * k < 0) continue;
      const fail = slips(n, kind), centre = x + half;
      if (!g && fail && kind === 'umi' && centre < playX && !d.dropped.has(n)) { d.dropped.add(n); d.boxes.push({ wx: n * spacing + 46 * k, y: gy - 24 * k, vy: 0 }); }
      sc.clearRect(0, 0, SCENE_W, SCENE_H);
      SCENES[kind](P, sc, c, t + idx * 1.7, { dropped: d.dropped.has(n) });
      ctx.drawImage(d.small, x, gy - SCENE_H * k + k, SCENE_W * k, SCENE_H * k);
      if (g) continue;
      ctx.textAlign = 'center';
      const lw = ctx.measureText(label).width + 10;
      ctx.fillStyle = c.ground; ctx.fillRect(centre - lw / 2, gy + 18, lw, 14);
      ctx.fillStyle = `rgba(${c.ink},.85)`; ctx.fillText(label, centre, gy + 21);
      if (centre < playX + half && centre > playX - half * 1.4) {
        const word = fail ? 'FAIL' : 'PASS', bw = 52, bh = 18, bx = Math.round(centre - bw / 2), by = Math.round(gy - SCENE_H * k - bh - 2);
        ctx.fillStyle = fail ? ink : c.ground; ctx.fillRect(bx, by, bw, bh);
        ctx.fillStyle = ink; ctx.fillRect(bx, by, bw, 2); ctx.fillRect(bx, by + bh - 2, bw, 2); ctx.fillRect(bx, by, 2, bh); ctx.fillRect(bx + bw - 2, by, 2, bh);
        ctx.fillRect(Math.round(centre) - 1, by + bh, 2, 5);
        ctx.fillStyle = fail ? c.ground : ink; ctx.fillText(word, centre, by + 5);
      }
      ctx.textAlign = 'left';
    }
    ctx.globalAlpha = 1;

    if (!g) {                                                                           // calm: dropped UMI boxes fall and scroll away
      d.boxes = d.boxes.filter(b => { b.vy += 600 * dt; b.y = Math.min(gy - 5 * k, b.y + b.vy * dt); const sx = Math.round(b.wx - shift); box(sx, Math.round(b.y), 5 * k, c.acc, ink); return sx > -40; });
      return;
    }

    // ── the game ──
    const endRun = () => {
      const final = Math.floor(g.score), b = d.board || [];
      g.final = final;
      if (final > 0 && d.board && (b.length < 10 || final > b[b.length - 1].score) && !d.form) {
        d.form = initialsForm(s, async (letters) => {
          const res = await submitScore(letters, final, Math.round(g.t * 1000));
          d.board = res.top; g.rank = res.rank; d.form = null; return res;
        });
      }
    };
    if (!g.over) {
      g.t += dt; g.speed = g.t < 55 ? 150 + g.t * 6 : Math.min(760, 480 + (g.t - 55) * 4);   // after ~1 min it keeps speeding up, to 760
      g.next -= dt;
      if (g.next <= 0) {                                                                // spawn: a FAIL crate, or a run of yellow data
        const wx = shift + s.w + 30;
        const roll = d.r(), touch = matchMedia('(hover: none)').matches;
        const maxH = Math.min(4, 1 + Math.floor(g.t / 18));                              // stacks grow taller over time
        const height = () => 1 + Math.floor(Math.pow(d.r(), 0.8) * maxH);
        if (g.t > 8 && roll < Math.min(0.35, 0.18 + g.t * 0.003)) g.items.push({ type: 'drone', wx, high: !touch && d.r() < 0.55 });
        else if (roll < 0.66) {
          g.items.push({ type: 'crate', wx, h: height() });
          if (g.t > 20 && d.r() < Math.min(0.5, 0.2 + g.t * 0.005))                        // a second stack close behind
            g.items.push({ type: 'crate', wx: wx + (10 + d.r() * 8) * k, h: Math.max(1, height() - 1) });
        }
        else { const air = d.r() < 0.6; for (let i = 0; i < 3; i++) g.items.push({ type: 'gem', wx: wx + i * 9 * k, y: air ? gy - (16 + Math.sin(i / 2 * Math.PI) * 6) * k : gy - 6 * k }); }
        g.next = Math.max(0.38, 1.5 - g.t * 0.02) + d.r() * Math.max(0.25, 0.7 - g.t * 0.008);
      }
      if (g.vy !== 0 || g.jumpY > 0) {
        g.jumpY = Math.max(0, g.jumpY + g.vy * dt);
        g.vy -= (g.holding && g.vy > 0 ? 520 : 1100) * dt;                               // lighter gravity while held on the way up
        if (g.jumpY === 0) { g.vy = 0; g.jumps = 0; }
      }
    }
    // the runner's box: torso and legs
    const ducking = g.duck && g.jumpY === 0;
    const hb = { x0: runX + 8 * k, x1: runX + 17 * k, y0: gy - g.jumpY - (ducking ? 17 : 30) * k, y1: gy - g.jumpY };
    g.items = g.items.filter(it => {
      const sx = Math.round(it.wx - shift);
      if (it.type === 'crate') {
        const w = 6 * k, h = 6 * k * it.h, y = gy - h;
        for (let i = 0; i < it.h; i++) {
          const yy = y + i * 6 * k;
          box(sx, yy, w, ink, ink); ctx.fillStyle = c.ground;                            // a dark crate marked with an X
          for (let q = 1; q < 5; q++) { ctx.fillRect(sx + q * k, yy + q * k, k, k); ctx.fillRect(sx + (5 - q) * k, yy + q * k, k, k); }
        }
        if (!g.over && sx < hb.x1 && sx + w > hb.x0 && hb.y1 > y + 2) { g.over = true; g.overAt = performance.now(); g.combo = 0; beep('fail'); endRun(); }
      } else if (it.type === 'drone') {
        // a small quadcopter: body, two arms, spinning rotors
        const y = Math.round(it.high ? gy - 27 * k : gy - 9 * k) + Math.round(Math.sin(t * 5 + it.wx) * k);
        const spin = (t * 20 | 0) % 2;
        ctx.fillStyle = ink;
        ctx.fillRect(sx + 2 * k, y + k, 6 * k, 3 * k);
        ctx.fillRect(sx, y, 10 * k, k);
        ctx.fillRect(sx + (spin ? 0 : k), y - k, 3 * k, k / 2 + 1); ctx.fillRect(sx + (spin ? 7 : 6) * k, y - k, 3 * k, k / 2 + 1);
        ctx.fillStyle = c.ground; ctx.fillRect(sx + 4 * k, y + 2 * k, 2 * k, k);
        const w = 10 * k, h = 4 * k;
        if (!g.over && sx < hb.x1 && sx + w > hb.x0 && y < hb.y1 && y + h > hb.y0) { g.over = true; g.overAt = performance.now(); g.combo = 0; beep('fail'); endRun(); }
      } else if (!it.got) {
        const w = 4 * k, y = Math.round(it.y + Math.sin(t * 6 + it.wx) * 2);
        box(sx, y, w, c.acc, ink);
        if (!g.over && sx < hb.x1 && sx + w > hb.x0 && y < hb.y1 && y + w > hb.y0) {
          it.got = true; g.combo++;
          const mult = Math.min(4, 1 + Math.floor((g.combo - 1) / 3)), pts = 25 * mult;
          g.score += pts;
          g.pops.push({ x: sx, y, a: 1, txt: '+' + pts, big: mult > 1 && (g.combo - 1) % 3 === 0 ? 'x' + mult : '' });
          beep(mult > 1 && (g.combo - 1) % 3 === 0 ? 'combo' : 'get');
          for (let q = 0; q < 10; q++) g.parts.push({ x: sx + w / 2, y: y + w / 2, vx: (d.r() - 0.5) * 220, vy: -d.r() * 220, a: 1 });
        } else if (!it.missed && sx + w < hb.x0 - 4) { it.missed = true; g.combo = 0; }      // a missed box breaks the combo
      }
      return sx > -80;
    });
    // collect effects
    for (const p of g.parts) { p.x += p.vx * dt; p.y += p.vy * dt; p.vy += 500 * dt; p.a -= dt * 1.6; ctx.fillStyle = c.acc; if (p.a > 0) ctx.fillRect(Math.round(p.x), Math.round(p.y), k, k); }
    g.parts = g.parts.filter(p => p.a > 0);
    ctx.textAlign = 'center';
    for (const p of g.pops) {
      p.y -= 30 * dt; p.a -= dt * 1.2; if (p.a <= 0) continue;
      ctx.fillStyle = `rgba(${c.ink},${p.a.toFixed(2)})`; ctx.fillText(p.txt, p.x + 8, p.y - 12);
      if (p.big) { ctx.save(); ctx.font = '16px "Pokemon GB", monospace'; ctx.fillStyle = c.acc; ctx.fillText(p.big, p.x + 8, p.y - 34); ctx.restore(); }
    }
    // the combo meter
    if (g.combo > 0) {
      const mult = Math.min(4, 1 + Math.floor((g.combo - 1) / 3));
      ctx.textAlign = 'right'; ctx.fillStyle = mult > 1 ? c.acc : `rgba(${c.ink},.6)`;
      ctx.fillText(`COMBO ${g.combo}  x${mult}`, s.w - 16, 28); ctx.textAlign = 'center';
    }
    g.pops = g.pops.filter(p => p.a > 0);
    // the runner (hands free in the game: yellow only means data to collect)
    const mc = d.me.getContext('2d'), M = pix(mc);
    mc.clearRect(0, 0, d.me.width, d.me.height);
    SCENES.egoRunner(M, mc, c, g.over || g.jumpY > 0 ? 0 : t * (1.2 + g.speed / 200), false);
    const squash = ducking ? 0.6 : 1, dh = d.me.height * k * squash;
    ctx.drawImage(d.me, runX, gy + k - SCENE_H * k * squash - g.jumpY, d.me.width * k * (ducking ? 1.15 : 1), dh);
    if (g.over) {
      const msg = d.form ? 'FAIL' : 'FAIL · SPACE OR TAP TO RETRY', mw = ctx.measureText(msg).width + 24, mx = s.w / 2;
      ctx.fillStyle = ink; ctx.fillRect(mx - mw / 2, gy - 150, mw, 26); ctx.fillStyle = c.ground; ctx.fillText(msg, mx, gy - 141);
      if (d.board && d.board.length) {                                                  // the leaderboard, top 5
        const bx = s.w - 190, by = 52, rows = d.board.slice(0, 5);
        ctx.fillStyle = c.ground; ctx.fillRect(bx - 10, by - 8, 186, 22 + rows.length * 14);
        ctx.fillStyle = ink; ctx.fillRect(bx - 10, by - 8, 186, 2); ctx.fillRect(bx - 10, by + 12 + rows.length * 14, 186, 2);
        ctx.textAlign = 'left'; ctx.fillText('LEADERBOARD', bx, by);
        rows.forEach((r, i) => {
          ctx.fillStyle = g.rank === i + 1 ? c.acc : ink;
          ctx.fillText(`${i + 1}. ${r.initials}`, bx, by + 16 + i * 14);
          ctx.textAlign = 'right'; ctx.fillText(String(r.score).padStart(5, '0'), bx + 166, by + 16 + i * 14); ctx.textAlign = 'left';
        });
        ctx.textAlign = 'center';
      }
    }
    ctx.textAlign = 'left';
  }
};
// the walking person with a head camera, drawn in a small canvas of its own
SCENES.egoRunner = (P, sc, c, t, hold = true) => {
  const gy = SCENE_H - 1, ink = `rgb(${c.ink})`;
  const me = pxPerson(P, sc, ink, 12, gy, t, t !== 0);
  sc.fillStyle = ink; P.rect(me.head[0], me.head[1] - 5, 4, 2);
  sc.fillStyle = c.acc; P.dot(me.head[0] + 3, me.head[1] - 4);
  const bob = t ? Math.round(Math.abs(Math.sin(t * 9))) : 0;
  sc.fillStyle = ink;
  if (hold) {
    P.line(me.x, me.y + 1, me.x + 7, me.y + 7 + bob); P.line(me.x, me.y + 2, me.x + 6, me.y + 9 + bob);
    sc.fillStyle = c.acc; P.rect(me.x + 7, me.y + 5 + bob, 5, 4); sc.fillStyle = ink; P.frame(me.x + 7, me.y + 5 + bob, 5, 4);
  } else {                                                                            // arms swing with the stride
    const sw = t ? Math.sin(t * 9) * 4 : -3;
    P.line(me.x, me.y + 1, me.x + 3 + sw, me.y + 9); P.line(me.x, me.y + 1, me.x + 3 - sw, me.y + 9);
  }
  sc.fillStyle = `rgba(${c.ink},.5)`;
  P.dotted(me.head[0] + 4, me.head[1] - 4, me.x + 22, me.y + 4, 2, Math.floor((t || 0) * 6) % 2);
};

export function mount(canvas, kind, opts = {}) {
  const study = STUDIES[kind], ctx = canvas.getContext('2d');
  const dk = () => opts.dark || document.documentElement.dataset.theme === 'dark';
  const colors = () => opts.palette === 'frost' ? (dk() ? FROST_D : FROST_L) : opts.palette === 'gbc' ? GBC_PAL : opts.palette === 'gb' ? GB_PAL : (opts.dark || document.documentElement.dataset.theme === 'dark') ? DARK : LIGHT;
  const s = { w: 0, h: 0, dpr: 1, opts, data: null, t0: performance.now(), canvas };
  function size() {
    const r = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio || 1, 2);
    if (!r.width || !r.height) return;
    canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
    Object.assign(s, { w: r.width, h: r.height, dpr });
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    s.data = study.init ? study.init(s, rng(opts.seed || 7)) : {};
  }
  new ResizeObserver(size).observe(canvas);
  addEventListener('themechange', () => { s.data = null; size(); });
  let visible = false;
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; }).observe(canvas);
  const still = matchMedia('(prefers-reduced-motion: reduce)').matches;
  (function frame(now) {
    if (visible && s.w && s.data) {
      const t = still ? 30 : (now - s.t0) / 1000;
      if (!opts.fx) study.draw(ctx, s, t, colors());
      else if (t - (s.fxLast || 0) > 1 / 20) {                 // effects run at 20 fps
        s.fxLast = t;
        const off = s.off || (s.off = document.createElement('canvas'));
        if (off.width !== Math.round(s.w) || off.height !== Math.round(s.h)) { off.width = Math.round(s.w); off.height = Math.round(s.h); }
        const oc = off.getContext('2d'); oc.setTransform(1, 0, 0, 1, 0, 0);
        study.draw(oc, s, t, colors());
        fxFrame(kind, opts.fx, off, ctx, s, colors());
      }
    }
    requestAnimationFrame(frame);
  })(performance.now());
}

/* Wire a subtask strip (a row of spans) to the shared clip: highlight and seek. */
export function strip(el) {
  const vid = getClip();
  const cells = SEGMENTS.map(g => {
    const b = document.createElement('button');
    b.type = 'button'; b.style.flex = String(g.end - g.start); b.title = g.label;
    b.innerHTML = `<span>${g.label}</span><i></i>`;
    if (g.fail) b.classList.add('fail');
    b.addEventListener('click', () => { vid.currentTime = g.start + 0.01; vid.play().catch(() => {}); });
    el.appendChild(b); return b;
  });
  (function tick() {
    const seg = currentSegment(vid.currentTime);
    cells.forEach((b, j) => {
      const g = SEGMENTS[j], on = g === seg;
      b.classList.toggle('on', on);
      b.querySelector('i').style.width = on ? `${100 * (vid.currentTime - g.start) / (g.end - g.start)}%` : '0';
    });
    requestAnimationFrame(tick);
  })();
}
