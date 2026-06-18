/* ─────────────────────────────────────────────
   Hero pipeline graphic
   Raw clips stacked on the left are fed into the Shotwell hub one at a time.
   Each returns as a smaller annotated clip — segmented, action-labelled and
   graded pass/fail — that stays on screen and accumulates in a grid.
   ───────────────────────────────────────────── */
(function () {
  var root = document.getElementById('pipeline');
  if (!root) return;

  var VIDEO_SRC = 'fold-rubric.mp4';
  var DURATION = 19;

  var SEGMENTS = [
    { label: 'Pick up',           start: 0,  end: 1,  rules: [{ t: 'One arm pick up',   v: 'pass' }] },
    { label: 'Straighten',        start: 1,  end: 7,  rules: [{ t: 'No slipping',       v: 'fail' }, { t: 'Shirt orientation', v: 'pass' }] },
    { label: 'Fold right sleeve', start: 7,  end: 10, rules: [{ t: 'Right sleeve first', v: 'pass' }] },
    { label: 'Fold left sleeve',  start: 10, end: 15, rules: [{ t: 'Sleeve tucked in',  v: 'fail' }] },
    { label: 'Fold first third',  start: 15, end: 16, rules: [] },
    { label: 'Fold second third', start: 16, end: 17, rules: [] },
    { label: 'Stack',             start: 17, end: 18, rules: [{ t: 'One arm stack',     v: 'pass' }] },
    { label: 'Home',              start: 18, end: 19, rules: [] }
  ];

  function segStatus(seg) {
    if (!seg.rules.length) return 'pass';
    return seg.rules.some(function (r) { return r.v === 'fail'; }) ? 'fail' : 'pass';
  }

  // tally of every rule across the original clip
  var PASS = 0, FAIL = 0;
  SEGMENTS.forEach(function (s) { s.rules.forEach(function (r) { r.v === 'fail' ? FAIL++ : PASS++; }); });

  function rnd(a, b) { return a + Math.random() * (b - a); }
  function rndInt(a, b) { return Math.floor(rnd(a, b + 1)); }

  // a fresh, random segmentation + pass/fail grading for the new clips
  function makeRandomEpisode(src) {
    var n = rndInt(6, 9);
    var widths = [], total = 0;
    for (var i = 0; i < n; i++) { var w = rnd(0.6, 3); widths.push(w); total += w; }
    var segs = [], pass = 0, fail = 0;
    for (var j = 0; j < n; j++) {
      var status = Math.random() < 0.72 ? 'pass' : 'fail';
      status === 'fail' ? fail++ : pass++;
      segs.push({ flex: widths[j] / total, status: status });
    }
    if (fail === 0) { var idx = rndInt(0, n - 1); segs[idx].status = 'fail'; fail = 1; pass = n - 1; }
    return { src: src, name: 'episode_' + rndInt(1000, 9999), task: 'fold_clothing', segs: segs, pass: pass, fail: fail };
  }

  // episode 0 keeps the hand-authored original clip + grading; the four new
  // uploads each get a randomized segmentation, grading and name.
  var EPDATA = [{
    src: 'fold-rubric.mp4',
    name: 'episode_0431',
    task: 'fold_clothing',
    segs: SEGMENTS.map(function (s) { return { flex: (s.end - s.start), status: segStatus(s) }; }),
    pass: PASS, fail: FAIL
  }];
  ['episode-2.mp4', 'episode-3.mp4', 'episode-4.mp4', 'episode-5.mp4'].forEach(function (src) {
    EPDATA.push(makeRandomEpisode(src));
  });
  var COUNT = EPDATA.length;

  function fmt(t) {
    var m = Math.floor(t / 60), s = Math.round(t - m * 60);
    if (s === 60) { m += 1; s = 0; }
    return m + ':' + (s < 10 ? '0' : '') + s;
  }

  function curSeg(t) {
    for (var i = 0; i < SEGMENTS.length; i++) { if (t < SEGMENTS[i].end - 0.001) return SEGMENTS[i]; }
    return SEGMENTS[SEGMENTS.length - 1];
  }

  // ─── Shared clip as a blob URL (one fetch, every video reuses it) ───
  var videoURL = null, videoURLState = 'idle', videoCbs = [];
  function ensureVideoURL(cb) {
    if (videoURLState === 'ready') { if (cb) cb(); return; }
    if (cb) videoCbs.push(cb);
    if (videoURLState === 'loading') return;
    videoURLState = 'loading';
    fetch(VIDEO_SRC).then(function (r) { return r.blob(); })
      .then(function (b) { videoURL = URL.createObjectURL(b); })
      .catch(function () { videoURL = VIDEO_SRC; })
      .then(function () { videoURLState = 'ready'; videoCbs.splice(0).forEach(function (f) { f(); }); });
  }

  // ─── Build DOM ───────────────────────────────
  var wires = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  wires.setAttribute('class', 'pl-wires');

  var grid = document.createElement('div');
  grid.className = 'pl-grid';

  // Column 1 — incoming stack
  var inCol = document.createElement('div');
  inCol.className = 'pl-in';

  // each card carries its own clip; only the front one plays, the rest hold
  // on their own poster frame until it's their turn
  function makeCardVideo(src) {
    var v = document.createElement('video');
    v.muted = true; v.loop = true; v.playsInline = true;
    v.setAttribute('playsinline', ''); v.preload = 'auto';
    v.src = src;
    v.addEventListener('loadedmetadata', function () {
      var r = v.duration ? v.duration / DURATION : 1;
      if (isFinite(r) && r > 0) v.playbackRate = Math.min(16, Math.max(0.0625, r));
      try { v.currentTime = Math.min(0.6, (v.duration || 2) * 0.12); } catch (e) {}
    });
    v.addEventListener('loadeddata', function () { v.classList.add('is-ready'); });
    v.addEventListener('seeked', function () { v.classList.add('is-ready'); });
    return v;
  }

  var STACK = COUNT;
  var inCards = [];
  for (var i = 0; i < STACK; i++) {
    var card = document.createElement('div');
    card.className = 'vid-card';
    var stage = document.createElement('div');
    stage.className = 'vid-stage';
    var vid = makeCardVideo(EPDATA[i].src);
    stage.appendChild(vid);
    var bar = document.createElement('div');
    bar.className = 'vid-bar';
    bar.innerHTML =
      '<span class="vid-tri"></span>' +
      '<span class="vid-track"><i></i></span>' +
      '<span class="vid-len">' + fmt(DURATION) + '</span>';
    card.appendChild(stage);
    card.appendChild(bar);
    inCol.appendChild(card);
    inCards.push({ root: card, stage: stage, video: vid, fill: bar.querySelector('.vid-track i') });
  }

  var inLabel = document.createElement('div');
  inLabel.className = 'pl-in-label';
  inLabel.textContent = 'Your training data';
  inCol.appendChild(inLabel);

  // Column 2 — hub
  var hubCol = document.createElement('div');
  hubCol.className = 'pl-hub';
  var logo = document.createElement('div');
  logo.className = 'pl-logo';
  logo.innerHTML = '<img src="shotwell-logo.png" alt="Shotwell" />';
  var hubName = document.createElement('div');
  hubName.className = 'pl-hub-name';
  hubName.innerHTML = 'Shotwell<span>.</span>';
  hubCol.appendChild(logo);
  hubCol.appendChild(hubName);

  // Column 3 — annotated outputs
  var outCol = document.createElement('div');
  outCol.className = 'pl-out';

  var outCards = EPDATA.map(function (ep) {
    var card = document.createElement('div');
    card.className = 'out-card';
    card.setAttribute('data-shown', 'false');

    var head = document.createElement('div');
    head.className = 'out-head';
    head.innerHTML =
      '<span class="out-name">' + ep.name + ' · ' + ep.task + '</span>' +
      '<span class="out-tally"><span class="pass">' + ep.pass + '</span><span class="fail">' + ep.fail + '</span></span>';

    var barEl = document.createElement('div');
    barEl.className = 'out-bar';
    ep.segs.forEach(function (seg) {
      var sp = document.createElement('span');
      sp.className = 'out-seg';
      sp.setAttribute('data-status', seg.status);
      sp.style.flex = seg.flex.toFixed(4) + ' 0 0';
      barEl.appendChild(sp);
    });
    var play = document.createElement('span');
    play.className = 'out-play';
    play.style.left = '0%';
    barEl.appendChild(play);

    card.appendChild(head);
    card.appendChild(barEl);
    outCol.appendChild(card);

    return { root: card, play: play, start: 0 };
  });

  var outLabel = document.createElement('div');
  outLabel.className = 'pl-out-label';
  outLabel.textContent = 'Segmented with failures detected';
  outCol.insertBefore(outLabel, outCol.firstChild);

  grid.appendChild(inCol);
  grid.appendChild(hubCol);
  grid.appendChild(outCol);
  root.appendChild(wires);
  root.appendChild(grid);

  // ─── Scaler ──────────────────────────────────
  var frame = root.parentElement;
  var DESIGN_W = 1400;
  var SHIFT_X = 160;      // nudge whole graphic right so it sits more centered
  var curScale = 1;
  var FLY_DX = 280, FLY_DY = 0;

  function computeFly() {
    var inB = inCol.getBoundingClientRect();
    var lo = logo.getBoundingClientRect();
    FLY_DX = ((lo.left + lo.width / 2) - (inB.left + inB.width / 2)) / curScale;
    FLY_DY = ((lo.top + lo.height / 2) - (inB.top + inB.height / 2)) / curScale;
  }

  function fit() {
    var avail = frame.clientWidth;
    if (!avail) return;
    curScale = Math.min(1, avail / DESIGN_W);
    root.style.transform = 'translateX(-50%) scale(' + curScale + ') translateX(' + SHIFT_X + 'px)';
    frame.style.height = (root.offsetHeight * curScale) + 'px';
    computeFly();
    drawWires();
  }
  function ensureFit() { fit(); if (!frame.clientWidth) requestAnimationFrame(ensureFit); }

  // ─── Wires ───────────────────────────────────
  var SVGNS = 'http://www.w3.org/2000/svg';
  var inWire = document.createElementNS(SVGNS, 'path');
  inWire.setAttribute('class', 'pl-wire pl-wire--in');
  wires.appendChild(inWire);

  var outWires = outCards.map(function () {
    var p = document.createElementNS(SVGNS, 'path');
    p.setAttribute('class', 'pl-out-wire');
    wires.appendChild(p);
    return p;
  });

  function rel(el, atRight) {
    var r = el.getBoundingClientRect();
    var pr = root.getBoundingClientRect();
    return {
      x: (r.left + (atRight ? r.width : 0) - pr.left) / curScale,
      y: (r.top + r.height / 2 - pr.top) / curScale
    };
  }

  function drawWires() {
    var W = root.offsetWidth, H = root.offsetHeight;
    wires.setAttribute('viewBox', '0 0 ' + W + ' ' + H);
    wires.setAttribute('width', W);
    wires.setAttribute('height', H);

    var hl = rel(logo, false), hr = rel(logo, true);
    var inLeft = rel(inCol, false);
    // Start the wire well under the stacked clips (left of the stack centre) so
    // it always reaches beneath the videos and never falls short as a clip
    // peels off toward the hub.
    var frontRight = inLeft.x + inCol.offsetWidth / 2 - 90;
    inWire.setAttribute('d', 'M' + frontRight + ',' + hl.y + ' L' + hl.x + ',' + hl.y);
    // the stack never empties now, so the left connector is always present
    inWire.style.opacity = (order && order.length) ? '1' : '0';

    outCards.forEach(function (o, i) {
      var cp = rel(o.root, false);
      var mx = (hr.x + cp.x) / 2;
      outWires[i].setAttribute('d', 'M' + hr.x + ',' + hr.y + ' C' + mx + ',' + hr.y + ' ' + mx + ',' + cp.y + ' ' + cp.x + ',' + cp.y);
    });
  }

  // ─── Stack queue ─────────────────────────────
  var order = inCards.slice();

  function layoutStack() {
    order.forEach(function (ic, depth) {
      var dx = -depth * 26, dy = -depth * 20, sc = 1 - depth * 0.045;
      ic.root.style.zIndex = String(30 - depth);
      ic.root.style.opacity = depth === 0 ? '1' : String(Math.max(0.42, 0.86 - depth * 0.11).toFixed(2));
      ic.root.style.transform = 'translate(-50%,-50%) translate(' + dx + 'px,' + dy + 'px) scale(' + sc + ')';
      ic.root.classList.toggle('vid-card--ghost', depth !== 0);
    });
  }

  function playFront() {
    order.forEach(function (ic, depth) {
      if (!ic.video) return;
      if (depth === 0) { var p = ic.video.play(); if (p && p.catch) p.catch(function () {}); }
      else { try { ic.video.pause(); } catch (e) {} }
    });
  }

  // ─── Output playback (playheads) ────
  function tickOutputs() {
    var now = performance.now();
    outCards.forEach(function (o) {
      if (o.root.getAttribute('data-shown') !== 'true') return;
      var t = ((now - o.start) / 1000) % DURATION;
      o.play.style.left = (Math.min(1, t / DURATION) * 100) + '%';
    });
    var f = order[0];
    if (f && f.fill && f.video && f.video.duration) {
      f.fill.style.transform = 'scaleX(' + (f.video.currentTime / f.video.duration).toFixed(3) + ')';
    }
    // redraw connectors every frame so they track the cards as the
    // vertical layout shifts and connect the instant a box appears
    drawWires();
    requestAnimationFrame(tickOutputs);
  }

  // ─── Timeline ────────────────────────────────
  var timers = [];
  function after(ms, fn) { timers.push(setTimeout(fn, ms)); }
  function clearTimers() { timers.forEach(clearTimeout); timers = []; }
  function pulse() { logo.setAttribute('data-pulse', 'true'); after(1100, function () { logo.removeAttribute('data-pulse'); }); }

  var PLAY_MS = 1300;   // front clip plays before being fed in
  var HOLD_MS = 2200;   // pause on the full grid before resetting

  function hideOutputs() {
    outCards.forEach(function (o, i) {
      o.root.setAttribute('data-shown', 'false');
      outWires[i].removeAttribute('data-on');
    });
  }

  function revealOutput(k) {
    // a new pass restarts the right column
    if (k === 0) {
      outCards.forEach(function (o, i) {
        o.root.setAttribute('data-shown', 'false');
        outWires[i].removeAttribute('data-on');
        o.start = 0;
      });
    }
    var o = outCards[k];
    o.start = performance.now();
    o.root.setAttribute('data-shown', 'true');
    outWires[k].setAttribute('data-on', 'true');
    drawWires();
  }

  function processStep(idx) {
    // idx selects which annotated output is produced this round. The stack
    // never empties: the fed clip cycles to the back, so the left side runs
    // forever, and the right column restarts after every full pass of COUNT.
    after(PLAY_MS, function () {
      var gone = order[0];
      pulse();
      // feed the front clip into the hub
      if (gone) {
        gone.root.style.transition = 'transform 0.55s cubic-bezier(0.5,0,0.5,1), opacity 0.5s ease';
        gone.root.style.zIndex = '2';
        gone.root.style.transform = 'translate(-50%,-50%) translate(' + FLY_DX + 'px,' + FLY_DY + 'px) scale(0.12)';
        gone.root.style.opacity = '0';
      }
      // the annotated clip emerges from the hub
      after(280, function () { revealOutput(idx); });
      // once it's in, cycle that clip to the back and promote the next one
      after(600, function () {
        if (gone && gone.video) { try { gone.video.pause(); } catch (e) {} }
        order.push(order.shift());
        if (gone) {
          // re-enter from the left into the back of the stack
          gone.root.style.transition = 'none';
          gone.root.style.opacity = '0';
          gone.root.style.transform = 'translate(-50%,-50%) translate(-210px, 26px) scale(0.82)';
          void gone.root.offsetWidth;
          gone.root.style.transition = 'transform 0.55s cubic-bezier(0.22,1,0.36,1), opacity 0.45s ease';
        }
        layoutStack();
        playFront();
        after(360, function () { processStep((idx + 1) % COUNT); });
      });
    });
  }

  function refillStack(done) {
    clearTimers();
    hideOutputs();
    drawWires();
    order = inCards.slice();
    inCards.forEach(function (ic, i) {
      ic.root.style.transition = 'none';
      ic.root.style.opacity = '0';
      ic.root.style.transform = 'translate(-50%,-50%) translate(' + (-180 - i * 12) + 'px,' + (i * 7) + 'px) scale(0.85)';
    });
    void root.offsetWidth;
    inCards.forEach(function (ic) {
      ic.root.style.transition = 'transform 0.55s cubic-bezier(0.22,1,0.36,1), opacity 0.45s ease';
    });
    after(40, function () { layoutStack(); drawWires(); });
    after(560, function () { done && done(); });
  }

  function runCycle() {
    refillStack(function () {
      playFront();
      processStep(0);
    });
  }

  // ─── Kick off when in view ───────────────────
  var started = false;
  function start() {
    if (started) return;
    started = true;
    fit();
    requestAnimationFrame(tickOutputs);
    runCycle();
  }

  window.addEventListener('resize', fit);
  window.addEventListener('load', fit);
  if ('ResizeObserver' in window) new ResizeObserver(function () { fit(); }).observe(frame);
  ensureFit();
  setTimeout(fit, 300);
  setTimeout(fit, 900);
  setTimeout(fit, 1600);

  setTimeout(start, 350);
  if ('IntersectionObserver' in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) { if (e.isIntersecting) start(); });
    }, { threshold: 0.05 });
    io.observe(root);
  }
})();
