/* Halftone renderer.
   Draws an image as a grid of dots whose radius tracks local luminance.
   A segmentation mask at low resolution is already a dot matrix — this is
   the same operation run on the source frame instead of the label.

   Source footage is typically dark (mean luma ~90/255), so a raw mapping
   produces mostly tiny dots and reads as sparse. `lo`/`hi`/`gamma` stretch
   the tonal range before the dot radius is computed. */

export function halftone(canvas, img, opts) {
  var o = Object.assign({
    step: 8,
    lo: 0.04,          // black point
    hi: 0.85,          // white point
    gamma: 0.62,       // <1 lifts midtones
    invert: true,      // true: bright pixels -> big dots (dark grounds)
    dot: '#F5F2EB',
    bg: null,
    maxScale: 0.60,
    progress: 1,
    focusX: 0.5,       // horizontal crop bias when the source is wider than the canvas
    keep: 1            // fraction of dots drawn — <1 dissolves the field like static
  }, opts || {});

  var w = canvas.width, h = canvas.height;
  if (!w || !h) return;

  var gw = Math.max(1, Math.floor(w / o.step));
  var gh = Math.max(1, Math.floor(h / o.step));
  var read = document.createElement('canvas');
  read.width = gw; read.height = gh;
  var rc = read.getContext('2d', { willReadFrequently: true });

  // Intrinsic size, not layout size: for a <video>, .width is the HTML
  // attribute (0 when unset) — using it silently draws nothing.
  var iw = img.videoWidth || img.naturalWidth || img.width;
  var ih = img.videoHeight || img.naturalHeight || img.height;
  if (!iw || !ih) return;

  // cover-fit the source into the grid so nothing is letterboxed
  var sr = iw / ih, dr = w / h, sx = 0, sy = 0, sw = iw, sh = ih;
  if (sr > dr) { sw = ih * dr; sx = (iw - sw) * o.focusX; }
  else { sh = iw / dr; sy = (ih - sh) / 2; }
  rc.drawImage(img, sx, sy, sw, sh, 0, 0, gw, gh);
  var data = rc.getImageData(0, 0, gw, gh).data;

  var ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, w, h);
  if (o.bg) { ctx.fillStyle = o.bg; ctx.fillRect(0, 0, w, h); }
  ctx.fillStyle = o.dot;

  var maxR = o.step * o.maxScale;
  var cutoff = o.progress * w;
  var span = Math.max(1e-6, o.hi - o.lo);

  for (var gy = 0; gy < gh; gy++) {
    for (var gx = 0; gx < gw; gx++) {
      var x = gx * o.step + o.step / 2;
      if (x > cutoff) continue;
      if (o.keep < 1) {
        // stable per-cell hash, so dots drop out in place instead of shimmering
        var rnd = Math.sin(gx * 127.1 + gy * 311.7) * 43758.5453;
        rnd -= Math.floor(rnd);
        if (rnd > o.keep) continue;
      }
      var i = (gy * gw + gx) * 4;
      var l = (0.2126 * data[i] + 0.7152 * data[i + 1] + 0.0722 * data[i + 2]) / 255;
      var v = (l - o.lo) / span;
      v = v < 0 ? 0 : v > 1 ? 1 : v;
      var t = Math.pow(o.invert ? v : 1 - v, o.gamma);
      var r = maxR * t;
      if (r < 0.3) continue;
      ctx.beginPath();
      ctx.arc(x, gy * o.step + o.step / 2, r, 0, 6.2832);
      ctx.fill();
    }
  }
}

/* Size a canvas to its parent at device pixel ratio. Returns CSS dimensions. */
export function sizeTo(canvas, ratio) {
  var box = canvas.parentElement.getBoundingClientRect();
  var w = box.width;
  var h = ratio ? w / ratio : box.height;
  var dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.style.width = w + 'px';
  canvas.style.height = h + 'px';
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  return { w: w, h: h, dpr: dpr };
}

/* Debounced resize helper. */
export function onResize(fn, ms) {
  var t;
  addEventListener('resize', function () {
    clearTimeout(t);
    t = setTimeout(fn, ms || 150);
  });
}
