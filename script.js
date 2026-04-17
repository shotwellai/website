(function () {
  // ─── Hamburger menu toggle ───
  var toggle = document.getElementById('nav-toggle');
  var links = document.getElementById('nav-links');

  toggle.addEventListener('click', function () {
    toggle.classList.toggle('open');
    links.classList.toggle('open');
  });

  links.querySelectorAll('a').forEach(function (link) {
    link.addEventListener('click', function () {
      toggle.classList.remove('open');
      links.classList.remove('open');
    });
  });

  // ─── Nav scroll effect ───
  var nav = document.getElementById('nav');
  var scrolled = false;

  function checkScroll() {
    var shouldBeScrolled = window.scrollY > 40;
    if (shouldBeScrolled !== scrolled) {
      scrolled = shouldBeScrolled;
      nav.classList.toggle('scrolled', scrolled);
    }
  }

  window.addEventListener('scroll', checkScroll, { passive: true });
  checkScroll();

  // ─── Scroll-triggered reveals ───
  var revealEls = document.querySelectorAll('.reveal, .reveal-group');

  if ('IntersectionObserver' in window) {
    var observer = new IntersectionObserver(
      function (entries) {
        entries.forEach(function (entry) {
          if (entry.isIntersecting) {
            entry.target.classList.add('visible');
            observer.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.12, rootMargin: '0px 0px -40px 0px' }
    );

    revealEls.forEach(function (el) {
      observer.observe(el);
    });
  } else {
    revealEls.forEach(function (el) {
      el.classList.add('visible');
    });
  }

  // ─── Staggered hero reveals ───
  var heroReveals = document.querySelectorAll('.hero .reveal');
  heroReveals.forEach(function (el, i) {
    el.style.transitionDelay = (i * 0.12) + 's';
  });

  // Trigger hero reveals immediately after short delay
  setTimeout(function () {
    heroReveals.forEach(function (el) {
      el.classList.add('visible');
    });
  }, 100);

  // ─── Interactive Robotic Arms ───

  var canvas = document.getElementById('arms-canvas');
  var ctx = canvas.getContext('2d');
  var dpr = window.devicePixelRatio || 1;
  var W, H;

  var ARM_COLOR = '#C8793E';
  var BG_COLOR = '#EDE0D4';

  var SEG_SCALE = 1;
  var BASE_LENGTHS = [35, 58, 45];
  var STROKE_W = 2.2;
  var JOINT_R = 6;
  var JOINT_STROKE = 2;
  var SEG_WIDTH = 9;
  var GRIPPER_LEN = 14;
  var BASE_W = 28;
  var BASE_H = 9;
  var PEDESTAL_W = 36;
  var PEDESTAL_H = 5;

  var STIFFNESS = 0.055;
  var DAMPING = 0.83;

  var mouseInCanvas = false;
  var mouseX = 0;
  var mouseY = 0;
  var idleTime = 0;
  var activeBlend = 0;
  var ctaHovered = false;    // true while user hovers the CTA — suppresses cursor-follow

  var arms = [];

  // Arms are ground-mounted (pedestal on floor, arm extends up).
  // side: 'left' | 'right' — used only to give opposing idle phases.
  function createArm(baseX, baseY, side, phaseOffset) {
    return {
      baseX: baseX,
      baseY: baseY,
      side: side,
      phase: phaseOffset,
      joints: [
        { x: baseX, y: baseY },
        { x: baseX, y: baseY - BASE_LENGTHS[0] * SEG_SCALE },
        { x: baseX, y: baseY - (BASE_LENGTHS[0] + BASE_LENGTHS[1]) * SEG_SCALE },
        { x: baseX, y: baseY - (BASE_LENGTHS[0] + BASE_LENGTHS[1] + BASE_LENGTHS[2]) * SEG_SCALE }
      ],
      smoothTarget: { x: baseX, y: baseY - 140 * SEG_SCALE },
      velocity: { x: 0, y: 0 }
    };
  }

  // Two arms: one on each side of the hero text, mounted at the bottom of the hero section.
  function getArmMounts() {
    var heroEl = document.getElementById('hero');
    var rect = heroEl ? heroEl.getBoundingClientRect() : { top: 0, bottom: H, height: H };
    var baseY = rect.bottom - 10;

    // Position arms outside the text column, under the hero content.
    var innerEl = heroEl ? heroEl.querySelector('.hero-inner') : null;
    var GAP = 180; // px outside the text column
    var MIN_EDGE = 60;
    var leftX, rightX;
    if (innerEl) {
      var ir = innerEl.getBoundingClientRect();
      leftX = Math.max(MIN_EDGE, ir.left - GAP);
      rightX = Math.min(W - MIN_EDGE, ir.right + GAP);
    } else {
      leftX = W * 0.22;
      rightX = W * 0.78;
    }

    return [
      { x: leftX,  y: baseY, side: 'left' },
      { x: rightX, y: baseY, side: 'right' }
    ];
  }

  function initArms() {
    var mounts = getArmMounts();
    arms = [];
    mounts.forEach(function (m, i) {
      arms.push(createArm(m.x, m.y, m.side, i * 1.8));
    });
  }

  function updateArmBases() {
    var mounts = getArmMounts();
    for (var i = 0; i < arms.length && i < mounts.length; i++) {
      arms[i].baseX = mounts[i].x;
      arms[i].baseY = mounts[i].y;
    }
  }

  function resize() {
    var rect = canvas.parentElement.getBoundingClientRect();
    W = rect.width;
    H = rect.height;
    canvas.width = W * dpr;
    canvas.height = H * dpr;
    canvas.style.width = W + 'px';
    canvas.style.height = H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    SEG_SCALE = Math.min(W / 1280, H / 500) * 1.15;
    SEG_SCALE = Math.max(SEG_SCALE, 0.45);
    initArms();
  }

  // FABRIK IK solver
  function solveFABRIK(arm, targetX, targetY) {
    var lengths = BASE_LENGTHS.map(function (l) { return l * SEG_SCALE; });
    var totalLength = lengths[0] + lengths[1] + lengths[2];
    var joints = arm.joints;

    joints[0].x = arm.baseX;
    joints[0].y = arm.baseY;

    var dx = targetX - joints[0].x;
    var dy = targetY - joints[0].y;
    var dist = Math.sqrt(dx * dx + dy * dy);

    if (dist > totalLength * 0.95) {
      var s = (totalLength * 0.95) / dist;
      targetX = joints[0].x + dx * s;
      targetY = joints[0].y + dy * s;
    }

    for (var iter = 0; iter < 10; iter++) {
      joints[3].x = targetX;
      joints[3].y = targetY;
      for (var i = 2; i >= 0; i--) {
        var ddx = joints[i].x - joints[i + 1].x;
        var ddy = joints[i].y - joints[i + 1].y;
        var d = Math.sqrt(ddx * ddx + ddy * ddy) || 0.001;
        joints[i].x = joints[i + 1].x + ddx / d * lengths[i];
        joints[i].y = joints[i + 1].y + ddy / d * lengths[i];
      }
      joints[0].x = arm.baseX;
      joints[0].y = arm.baseY;
      for (var i = 1; i <= 3; i++) {
        var ddx = joints[i].x - joints[i - 1].x;
        var ddy = joints[i].y - joints[i - 1].y;
        var d = Math.sqrt(ddx * ddx + ddy * ddy) || 0.001;
        joints[i].x = joints[i - 1].x + ddx / d * lengths[i - 1];
        joints[i].y = joints[i - 1].y + ddy / d * lengths[i - 1];
      }
    }
  }

  function springToward(arm, targetX, targetY, stiffMul) {
    var k = STIFFNESS * (stiffMul || 1);
    var dx = targetX - arm.smoothTarget.x;
    var dy = targetY - arm.smoothTarget.y;
    arm.velocity.x += dx * k;
    arm.velocity.y += dy * k;
    arm.velocity.x *= DAMPING;
    arm.velocity.y *= DAMPING;
    arm.smoothTarget.x += arm.velocity.x;
    arm.smoothTarget.y += arm.velocity.y;
  }

  // Each arm has its own idle motion with phase offset (ground-mounted, reaches up).
  function getIdleTarget(arm, time) {
    var t = time + arm.phase;
    var range = 40 * SEG_SCALE;
    var homeY = arm.baseY - 110 * SEG_SCALE;
    var x = arm.baseX + Math.sin(t * 0.5) * range + Math.sin(t * 0.8 + 1.2) * range * 0.4;
    var y = homeY + Math.cos(t * 0.6 + 0.5) * 20 * SEG_SCALE + Math.sin(t * 0.35) * 10 * SEG_SCALE;
    return { x: x, y: y };
  }

  // ─── Drawing: industrial robotic arm style ───

  // Draw a thick outlined segment (rounded rect between two points)
  function drawThickSegment(x1, y1, x2, y2, width) {
    var dx = x2 - x1;
    var dy = y2 - y1;
    var len = Math.sqrt(dx * dx + dy * dy) || 1;
    var nx = dx / len;
    var ny = dy / len;
    var px = -ny;
    var py = nx;
    var hw = width / 2;

    ctx.beginPath();
    var ax = x1 + px * hw;
    var ay = y1 + py * hw;
    var bx = x2 + px * hw;
    var by = y2 + py * hw;
    var cx = x2 - px * hw;
    var cy = y2 - py * hw;
    var ddx = x1 - px * hw;
    var ddy = y1 - py * hw;

    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
    ctx.arc(x2, y2, hw, Math.atan2(py, px), Math.atan2(-py, -px), false);
    ctx.lineTo(ddx, ddy);
    ctx.arc(x1, y1, hw, Math.atan2(-py, -px), Math.atan2(py, px), false);
    ctx.closePath();

    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = STROKE_W * SEG_SCALE;
    ctx.stroke();
  }

  function drawJointCircle(x, y, radius) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = JOINT_STROKE * SEG_SCALE;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(x, y, radius * 0.3, 0, Math.PI * 2);
    ctx.fillStyle = ARM_COLOR;
    ctx.fill();
  }

  function drawGripper(arm) {
    var tip = arm.joints[3];
    var prev = arm.joints[2];
    var dx = tip.x - prev.x;
    var dy = tip.y - prev.y;
    var len = Math.sqrt(dx * dx + dy * dy) || 1;
    var nx = dx / len;
    var ny = dy / len;
    var px = -ny;
    var py = nx;

    var gLen = GRIPPER_LEN * SEG_SCALE;
    var spread = arm.gripperOpen;
    var fingerW = 3 * SEG_SCALE;

    var wristLen = 6 * SEG_SCALE;
    var wx = tip.x + nx * wristLen;
    var wy = tip.y + ny * wristLen;
    drawThickSegment(tip.x, tip.y, wx, wy, 8 * SEG_SCALE);

    var forkX = wx;
    var forkY = wy;

    var spreadAmt = spread * 10 * SEG_SCALE;
    var mid1x = forkX + nx * gLen * 0.5 + px * spreadAmt * 0.8;
    var mid1y = forkY + ny * gLen * 0.5 + py * spreadAmt * 0.8;
    var end1x = forkX + nx * gLen + px * spreadAmt;
    var end1y = forkY + ny * gLen + py * spreadAmt;
    var tip1x = end1x + px * (-spreadAmt * 0.4);
    var tip1y = end1y + py * (-spreadAmt * 0.4);

    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = fingerW;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    ctx.beginPath();
    ctx.moveTo(forkX, forkY);
    ctx.quadraticCurveTo(mid1x, mid1y, end1x, end1y);
    ctx.stroke();

    var mid2x = forkX + nx * gLen * 0.5 - px * spreadAmt * 0.8;
    var mid2y = forkY + ny * gLen * 0.5 - py * spreadAmt * 0.8;
    var end2x = forkX + nx * gLen - px * spreadAmt;
    var end2y = forkY + ny * gLen - py * spreadAmt;

    ctx.beginPath();
    ctx.moveTo(forkX, forkY);
    ctx.quadraticCurveTo(mid2x, mid2y, end2x, end2y);
    ctx.stroke();
  }

  function drawBase(arm) {
    var x = arm.baseX;
    var y = arm.baseY;
    var sw = SEG_SCALE;

    var pw = PEDESTAL_W * sw;
    var ph = PEDESTAL_H * sw;
    ctx.beginPath();
    ctx.rect(x - pw / 2, y - ph / 2, pw, ph);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = STROKE_W * sw;
    ctx.stroke();

    var bw = BASE_W * sw;
    var bh = BASE_H * sw;
    ctx.beginPath();
    ctx.rect(x - bw / 2, y - ph / 2 - bh, bw, bh);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = STROKE_W * sw;
    ctx.stroke();
  }

  function drawArm(arm) {
    var j = arm.joints;
    var sw = SEG_SCALE;
    var startY = j[0].y - (BASE_H + PEDESTAL_H / 2) * sw;

    drawBase(arm);

    drawThickSegment(j[0].x, startY, j[1].x, j[1].y, SEG_WIDTH * sw);
    drawThickSegment(j[1].x, j[1].y, j[2].x, j[2].y, (SEG_WIDTH - 2) * sw);
    drawThickSegment(j[2].x, j[2].y, j[3].x, j[3].y, (SEG_WIDTH - 4) * sw);

    drawJointCircle(j[0].x, startY, JOINT_R * sw);
    drawJointCircle(j[1].x, j[1].y, JOINT_R * sw);
    drawJointCircle(j[2].x, j[2].y, (JOINT_R - 1.5) * sw);

    var tip = j[3];
    var prev = j[2];
    var wdx = tip.x - prev.x;
    var wdy = tip.y - prev.y;
    var wlen = Math.sqrt(wdx * wdx + wdy * wdy) || 1;
    var wristLen = 6 * sw;
    var wx = tip.x + (wdx / wlen) * wristLen;
    var wy = tip.y + (wdy / wlen) * wristLen;
    drawThickSegment(tip.x, tip.y, wx, wy, 8 * sw);
  }

  function update() {
    idleTime += 1 / 60;

    var targetBlend = (mouseInCanvas && !ctaHovered) ? 1 : 0;
    activeBlend += (targetBlend - activeBlend) * 0.045;

    for (var i = 0; i < arms.length; i++) {
      var arm = arms[i];
      var idle = getIdleTarget(arm, idleTime);
      var tx, ty;

      if (activeBlend > 0.01) {
        tx = idle.x + (mouseX - idle.x) * activeBlend;
        ty = idle.y + (mouseY - idle.y) * activeBlend;
      } else {
        tx = idle.x;
        ty = idle.y;
      }

      springToward(arm, tx, ty, 1);
      solveFABRIK(arm, arm.smoothTarget.x, arm.smoothTarget.y);
    }
  }

  function draw() {
    ctx.clearRect(0, 0, W, H);
    for (var i = 0; i < arms.length; i++) {
      drawArm(arms[i]);
    }
  }

  function loop() {
    update();
    draw();
    requestAnimationFrame(loop);
  }

  window.addEventListener('mousemove', function (e) {
    var rect = canvas.getBoundingClientRect();
    mouseX = e.clientX - rect.left;
    mouseY = e.clientY - rect.top;
    mouseInCanvas = (
      e.clientX >= rect.left && e.clientX <= rect.right &&
      e.clientY >= rect.top && e.clientY <= rect.bottom
    );
  });

  window.addEventListener('mouseout', function (e) {
    if (!e.relatedTarget && !e.toElement) mouseInCanvas = false;
  });

  window.addEventListener('touchmove', function (e) {
    var rect = canvas.getBoundingClientRect();
    var touch = e.touches[0];
    mouseX = touch.clientX - rect.left;
    mouseY = touch.clientY - rect.top;
    mouseInCanvas = (
      touch.clientX >= rect.left && touch.clientX <= rect.right &&
      touch.clientY >= rect.top && touch.clientY <= rect.bottom
    );
  }, { passive: true });

  window.addEventListener('touchend', function () {
    mouseInCanvas = false;
  });

  window.addEventListener('resize', resize);
  window.addEventListener('scroll', updateArmBases, { passive: true });

  document.querySelectorAll('.btn-hero').forEach(function (btn) {
    btn.addEventListener('mouseenter', function () { ctaHovered = true; });
    btn.addEventListener('mouseleave', function () { ctaHovered = false; });
    btn.addEventListener('focus', function () { ctaHovered = true; });
    btn.addEventListener('blur', function () { ctaHovered = false; });
  });

  resize();
  loop();
})();

// ─── Temporal Segmentation animation ───
(function () {
  var videoEl = document.getElementById('seg-video');
  // 5 chunks, user-specified timings
  var CHUNKS = [
    { label: 'pick up sweater', start: 0.00, end: 1.00 },
    { label: 'spread sweater',  start: 1.00, end: 4.00 },
    { label: 'fold sleeves',    start: 4.00, end: 5.00 },
    { label: 'fold thirds',     start: 5.00, end: 6.50 },
    { label: 'stack sweater',   start: 6.50, end: 999 } // end patched to video duration
  ];
  var DURATION = 8; // patched on loadedmetadata

  var chunksEl = document.getElementById('seg-chunks');
  var trackEl = document.getElementById('seg-track');
  var countEl = document.getElementById('seg-count');
  var recEl = document.getElementById('seg-rec-time');
  var featureEl = document.getElementById('feature-segmentation');

  if (!chunksEl || !trackEl) return;

  // ─── Build scrubber bars (absolute-positioned so overlaps render correctly) ───
  var barEls = [];
  function assignRows() {
    var rows = [];
    var assignment = [];
    CHUNKS.forEach(function (c, i) {
      var placed = false;
      for (var r = 0; r < rows.length; r++) {
        if (rows[r].every(function (idx) { return CHUNKS[idx].end <= c.start; })) {
          rows[r].push(i);
          assignment[i] = r;
          placed = true;
          break;
        }
      }
      if (!placed) { rows.push([i]); assignment[i] = rows.length - 1; }
    });
    return { rows: rows, assignment: assignment };
  }
  var rowInfo = assignRows();
  trackEl.style.setProperty('--row-count', rowInfo.rows.length.toString());

  CHUNKS.forEach(function (c, i) {
    var bar = document.createElement('div');
    bar.className = 'seg-chunk-bar';
    bar.style.position = 'absolute';
    bar.style.left = '0%';
    bar.style.width = '100%';
    bar.style.top = (rowInfo.assignment[i] * (100 / rowInfo.rows.length)) + '%';
    bar.style.height = 'calc(' + (100 / rowInfo.rows.length) + '% - 3px)';
    var fill = document.createElement('div');
    fill.className = 'seg-chunk-bar-fill';
    bar.appendChild(fill);
    trackEl.appendChild(bar);
    barEls.push({ bar: bar, fill: fill });
  });
  function updateBarFlex() {
    CHUNKS.forEach(function (c, i) {
      var startPct = (c.start / DURATION) * 100;
      var endPct = (Math.min(c.end, DURATION) / DURATION) * 100;
      barEls[i].bar.style.left = startPct.toFixed(2) + '%';
      barEls[i].bar.style.width = (endPct - startPct).toFixed(2) + '%';
    });
  }
  var playhead = document.createElement('div');
  playhead.className = 'seg-playhead';
  trackEl.appendChild(playhead);

  // ─── Build chunk cards ───
  var chunkEls = [];
  CHUNKS.forEach(function (c) {
    var card = document.createElement('div');
    card.className = 'seg-chunk';
    card.setAttribute('data-state', 'pending');
    card.innerHTML =
      '<span class="seg-chunk-time"></span>' +
      '<span class="seg-chunk-label"><span class="typed"></span><span class="caret" style="display:none"></span></span>' +
      '<span class="seg-chunk-status"></span>';
    chunksEl.appendChild(card);
    chunkEls.push({
      root: card,
      time: card.querySelector('.seg-chunk-time'),
      typed: card.querySelector('.typed'),
      caret: card.querySelector('.caret')
    });
  });
  function updateTimeLabels() {
    CHUNKS.forEach(function (c, i) {
      chunkEls[i].time.textContent = fmtRange(c.start, Math.min(c.end, DURATION));
    });
  }

  function fmtRange(a, b) { return fmt(a) + ' – ' + fmt(b); }
  function fmt(t) {
    var m = Math.floor(t / 60);
    var s = t - m * 60;
    return pad(m) + ':' + s.toFixed(2).padStart(5, '0');
  }
  function pad(n) { return (n < 10 ? '0' : '') + n; }

  // ─── Chunk state rendering ───
  function updateChunks(currentTime) {
    CHUNKS.forEach(function (c, i) {
      var el = chunkEls[i];
      var bar = barEls[i];
      var state;
      if (currentTime < c.start) state = 'pending';
      else if (currentTime < c.end) state = 'active';
      else state = 'done';

      el.root.setAttribute('data-state', state);
      bar.bar.setAttribute('data-active', state === 'active' ? 'true' : 'false');
      bar.bar.setAttribute('data-done', state === 'done' ? 'true' : 'false');

      if (state === 'active') {
        var localT = (currentTime - c.start) / (c.end - c.start);
        bar.fill.style.setProperty('--fill', localT.toFixed(3));
      } else if (state === 'done') {
        bar.fill.style.setProperty('--fill', '1');
      } else {
        bar.fill.style.setProperty('--fill', '0');
      }

      if (state === 'pending') {
        el.typed.textContent = '';
        el.caret.style.display = 'none';
      } else {
        var typeStart = c.start + 0.1;
        var typeEnd = c.start + (c.end - c.start) * 0.7;
        var typeT = Math.max(0, Math.min(1, (currentTime - typeStart) / (typeEnd - typeStart)));
        var targetChars = Math.floor(typeT * c.label.length);
        el.typed.textContent = c.label.slice(0, targetChars);
        el.caret.style.display = (state === 'active' && targetChars < c.label.length) ? 'inline-block' : 'none';
        if (state === 'done') el.typed.textContent = c.label;
      }
    });

    var doneCount = CHUNKS.filter(function (c) { return currentTime >= c.end; }).length;
    countEl.textContent = doneCount + ' / ' + CHUNKS.length;
    playhead.style.setProperty('--pos', (currentTime / DURATION).toFixed(4));
    recEl.textContent = fmt(currentTime);
  }

  function updateEndTimecode() {
    var tc = document.querySelectorAll('.seg-timecode span');
    if (tc && tc[1]) tc[1].textContent = fmt(DURATION);
  }

  // ─── Video element playback ───
  var shouldBePlaying = false;

  function pollLoop() {
    if (videoEl && !isNaN(videoEl.duration) && videoEl.duration > 0) {
      var t = videoEl.currentTime;
      if (t > DURATION) t = DURATION;
      updateChunks(t);
    }
    requestAnimationFrame(pollLoop);
  }

  function onMeta() {
    if (!isNaN(videoEl.duration) && videoEl.duration > 0) {
      DURATION = videoEl.duration;
      CHUNKS[CHUNKS.length - 1].end = DURATION;
      updateBarFlex();
      updateTimeLabels();
      updateEndTimecode();
    }
  }
  videoEl.addEventListener('loadedmetadata', onMeta);
  videoEl.addEventListener('durationchange', onMeta);
  if (videoEl.readyState >= 1) onMeta();

  function playClip() {
    if (!videoEl) return;
    videoEl.muted = true;
    var p = videoEl.play();
    if (p && p.catch) p.catch(function () {});
  }
  function pauseClip() { if (videoEl) videoEl.pause(); }

  // ─── Intersection: only play when in view ───
  if ('IntersectionObserver' in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        shouldBePlaying = e.isIntersecting;
        if (shouldBePlaying) playClip(); else pauseClip();
      });
    }, { threshold: 0.25 });
    io.observe(featureEl);
  } else {
    shouldBePlaying = true;
    playClip();
  }

  requestAnimationFrame(pollLoop);

  // ─── Quality Filtering animation ───
  (function initQuality() {
    var grid = document.getElementById('quality-grid');
    var arrowsSvg = document.getElementById('quality-arrows');
    var passEl = document.getElementById('quality-pass-count');
    var failEl = document.getElementById('quality-fail-count');
    if (!grid || !arrowsSvg) return;

    var COLS = 16;
    var ROWS = 6;
    var TOTAL = COLS * ROWS;
    var dots = [];
    for (var i = 0; i < TOTAL; i++) {
      var d = document.createElement('div');
      d.className = 'q-dot';
      d.dataset.state = 'idle';
      grid.appendChild(d);
      dots.push(d);
    }

    var FAILURES = [
      { label: "Picked up two shirts" },
      { label: "Didn't straighten" },
      { label: "Asymmetric fold" }
    ];

    var totalCount = TOTAL;
    var failCount = 0;

    function updateLegend() {
      var passing = totalCount - failCount;
      passEl.textContent = passing + ' / ' + totalCount;
      failEl.textContent = failCount + ' / ' + totalCount;
    }
    updateLegend();

    function pickFailedIndices(n) {
      var available = [];
      for (var i = 0; i < dots.length; i++) {
        if (dots[i].dataset.state === 'idle') available.push(i);
      }
      var picks = [];
      while (picks.length < n && available.length) {
        var idx = available.splice(Math.floor(Math.random() * available.length), 1)[0];
        picks.push(idx);
      }
      return picks;
    }

    function clearArrows() {
      while (arrowsSvg.firstChild) arrowsSvg.removeChild(arrowsSvg.firstChild);
    }

    function drawArrow(targetEl, label, side) {
      var svgRect = arrowsSvg.getBoundingClientRect();
      var dotRect = targetEl.getBoundingClientRect();
      var tx = (dotRect.left + dotRect.width / 2) - svgRect.left;
      var ty = (dotRect.top + dotRect.height / 2) - svgRect.top;
      var dotR = dotRect.width / 2 + 4;

      var labelW = label.length * 6.4 + 16;
      var labelH = 22;
      var stageW = svgRect.width;
      var stageH = svgRect.height;
      var PAD = 6;

      var ax, ay, lx, ly, labelX, labelY, anchor;
      if (side === 'left') {
        ax = tx - dotR; ay = ty;
        lx = tx - 110; ly = ty - 28;
        labelX = lx - 6; labelY = ly + 4; anchor = 'end';
      } else if (side === 'right') {
        ax = tx + dotR; ay = ty;
        lx = tx + 110; ly = ty - 28;
        labelX = lx + 6; labelY = ly + 4; anchor = 'start';
      } else if (side === 'top') {
        ax = tx; ay = ty - dotR;
        lx = tx + 80; ly = ty - 70;
        labelX = lx + 6; labelY = ly + 4; anchor = 'start';
      } else {
        ax = tx; ay = ty + dotR;
        lx = tx - 80; ly = ty + 60;
        labelX = lx - 6; labelY = ly + 4; anchor = 'end';
      }

      var bgX = anchor === 'end' ? labelX - labelW + 6 : labelX - 6;
      if (bgX < PAD) {
        var shift = PAD - bgX;
        labelX += shift;
        lx += shift;
        bgX += shift;
      } else if (bgX + labelW > stageW - PAD) {
        var shift2 = (bgX + labelW) - (stageW - PAD);
        labelX -= shift2;
        lx -= shift2;
        bgX -= shift2;
      }
      var bgY = labelY - 14;
      if (bgY < PAD) {
        var shiftY = PAD - bgY;
        labelY += shiftY;
        ly += shiftY;
        bgY += shiftY;
      } else if (bgY + labelH > stageH - PAD) {
        var shiftY2 = (bgY + labelH) - (stageH - PAD);
        labelY -= shiftY2;
        ly -= shiftY2;
        bgY -= shiftY2;
      }

      var svgNS = 'http://www.w3.org/2000/svg';

      var path = document.createElementNS(svgNS, 'path');
      var c1x = (lx + ax) / 2;
      var c1y = ly;
      var c2x = (lx + ax) / 2;
      var c2y = ay;
      path.setAttribute('d', 'M' + lx + ',' + ly + ' C' + c1x + ',' + c1y + ' ' + c2x + ',' + c2y + ' ' + ax + ',' + ay);
      path.setAttribute('pathLength', '100');
      path.setAttribute('class', 'q-arrow-line');
      arrowsSvg.appendChild(path);

      var head = document.createElementNS(svgNS, 'polygon');
      var dx = ax - c2x, dy = ay - c2y;
      var len = Math.sqrt(dx*dx + dy*dy) || 1;
      var ux = dx / len, uy = dy / len;
      var px = -uy, py = ux;
      var hx1 = ax - ux * 7 + px * 4;
      var hy1 = ay - uy * 7 + py * 4;
      var hx2 = ax - ux * 7 - px * 4;
      var hy2 = ay - uy * 7 - py * 4;
      head.setAttribute('points', ax + ',' + ay + ' ' + hx1 + ',' + hy1 + ' ' + hx2 + ',' + hy2);
      head.setAttribute('class', 'q-arrow-head');
      arrowsSvg.appendChild(head);

      var bg = document.createElementNS(svgNS, 'rect');
      bg.setAttribute('x', bgX);
      bg.setAttribute('y', bgY);
      bg.setAttribute('width', labelW);
      bg.setAttribute('height', labelH);
      bg.setAttribute('rx', 3);
      bg.setAttribute('class', 'q-arrow-label-bg');
      arrowsSvg.appendChild(bg);

      var text = document.createElementNS(svgNS, 'text');
      text.setAttribute('x', labelX);
      text.setAttribute('y', labelY);
      text.setAttribute('text-anchor', anchor);
      text.setAttribute('class', 'q-arrow-label');
      text.textContent = label;
      arrowsSvg.appendChild(text);

      requestAnimationFrame(function () {
        path.setAttribute('data-show', 'true');
        head.setAttribute('data-show', 'true');
        bg.setAttribute('data-show', 'true');
        text.setAttribute('data-show', 'true');
      });
    }

    function chooseSide(dot) {
      var svgRect = arrowsSvg.getBoundingClientRect();
      var dotRect = dot.getBoundingClientRect();
      var cx = (dotRect.left + dotRect.width / 2) - svgRect.left;
      var relX = cx / svgRect.width;
      return relX < 0.5 ? 'right' : 'left';
    }

    function runCycle() {
      clearArrows();

      var picks = pickFailedIndices(3);
      var pickSet = {};
      picks.forEach(function (i) { pickSet[i] = true; });

      // Phase 1: flag dots
      picks.forEach(function (idx, i) {
        setTimeout(function () {
          dots[idx].dataset.state = 'flagged';
          failCount++;
          updateLegend();
        }, i * 450);
      });

      // Phase 1.5: turn remaining good dots green
      setTimeout(function () {
        for (var i = 0; i < dots.length; i++) {
          if (!pickSet[i] && dots[i].dataset.state === 'idle') {
            (function (el, delay) {
              setTimeout(function () { el.dataset.state = 'passed'; }, delay);
            })(dots[i], Math.random() * 700);
          }
        }
      }, 1700);

      // Phase 2: draw arrows + labels
      setTimeout(function () {
        picks.forEach(function (idx, i) {
          var side = chooseSide(dots[idx]);
          drawArrow(dots[idx], FAILURES[i % FAILURES.length].label, side);
        });
      }, 1500);

      // Phase 3: remove dots
      setTimeout(function () {
        picks.forEach(function (idx, i) {
          setTimeout(function () {
            dots[idx].dataset.state = 'removed';
            totalCount--;
            failCount--;
            updateLegend();
          }, i * 320);
        });
      }, 5400);

      // Phase 4: clear arrows
      setTimeout(function () {
        clearArrows();
      }, 7200);

      // Phase 5: restore everything to idle (hollow)
      setTimeout(function () {
        for (var i = 0; i < dots.length; i++) {
          dots[i].dataset.state = 'idle';
        }
        totalCount = TOTAL;
        failCount = 0;
        updateLegend();
      }, 8800);
    }

    var started = false;
    function maybeStart() {
      if (started) return;
      var rect = grid.getBoundingClientRect();
      if (rect.top < window.innerHeight * 0.85 && rect.bottom > 0) {
        started = true;
        setTimeout(runCycle, 800);
        setInterval(runCycle, 11500);
      }
    }
    window.addEventListener('scroll', maybeStart, { passive: true });
    maybeStart();
  })();

  // ─── Performance chart: training curves ───
  (function () {
    var svg = document.getElementById('perf-chart');
    if (!svg) return;

    var W = 820, H = 420;
    var PAD_L = 56, PAD_R = 28, PAD_T = 24, PAD_B = 46;
    var PW = W - PAD_L - PAD_R;
    var PH = H - PAD_T - PAD_B;

    var SVG_NS = 'http://www.w3.org/2000/svg';

    function xAt(t) { return PAD_L + t * PW; }
    function yAt(v) { return PAD_T + (1 - v) * PH; }

    function smoothNoise(seed) {
      var s = seed;
      return function () {
        s = (s * 9301 + 49297) % 233280;
        return s / 233280;
      };
    }

    function buildCurve(opts) {
      var pts = [];
      var N = 180;
      var rand = opts.rand;
      var lastNoise = 0;
      for (var i = 0; i <= N; i++) {
        var t = i / N;
        var v = opts.start + (opts.asymp - opts.start) * (1 - Math.exp(-opts.rate * t));
        var target = (rand() - 0.5) * opts.noise;
        lastNoise += (target - lastNoise) * 0.35;
        var noiseGate = Math.min(1, t * 3);
        v += lastNoise * noiseGate;
        v = Math.max(0, Math.min(0.99, v));
        pts.push({ t: t, v: v });
      }
      return pts;
    }

    function pathFrom(pts) {
      var d = '';
      for (var i = 0; i < pts.length; i++) {
        var p = pts[i];
        d += (i === 0 ? 'M' : 'L') + xAt(p.t).toFixed(2) + ',' + yAt(p.v).toFixed(2) + ' ';
      }
      return d.trim();
    }

    function el(name, attrs) {
      var e = document.createElementNS(SVG_NS, name);
      for (var k in attrs) e.setAttribute(k, attrs[k]);
      return e;
    }

    // ─── Grid & axes ───
    var gridG = el('g', { class: 'perf-grid' });
    var yTicks = [0, 0.25, 0.5, 0.75, 1.0];
    yTicks.forEach(function (v) {
      gridG.appendChild(el('line', {
        x1: PAD_L, x2: PAD_L + PW,
        y1: yAt(v), y2: yAt(v)
      }));
      var label = el('text', {
        x: PAD_L - 10, y: yAt(v) + 3,
        'text-anchor': 'end',
        class: 'perf-grid-label'
      });
      label.textContent = Math.round(v * 100) + '%';
      gridG.appendChild(label);
    });
    var xTicks = [0, 0.25, 0.5, 0.75, 1.0];
    xTicks.forEach(function (t) {
      var label = el('text', {
        x: xAt(t), y: PAD_T + PH + 20,
        'text-anchor': 'middle',
        class: 'perf-grid-label'
      });
      label.textContent = Math.round(t * 100) + 'k';
      gridG.appendChild(label);
    });
    var yAxLabel = el('text', {
      x: -(PAD_T + PH / 2), y: 16,
      transform: 'rotate(-90)',
      'text-anchor': 'middle',
      class: 'perf-axis-label'
    });
    yAxLabel.textContent = 'Success rate';
    gridG.appendChild(yAxLabel);
    var xAxLabel = el('text', {
      x: PAD_L + PW / 2, y: PAD_T + PH + 38,
      'text-anchor': 'middle',
      class: 'perf-axis-label'
    });
    xAxLabel.textContent = 'Training steps';
    gridG.appendChild(xAxLabel);
    svg.appendChild(gridG);

    // ─── Curves ───
    var rand1 = smoothNoise(73);
    var rand2 = smoothNoise(211);
    var baseline = buildCurve({ asymp: 0.82, rate: 3.2, start: 0.08, noise: 0.045, rand: rand1 });
    var shotwell = buildCurve({ asymp: 0.945, rate: 5.1, start: 0.08, noise: 0.028, rand: rand2 });

    var fillD = '';
    for (var i = 0; i < shotwell.length; i++) {
      var p = shotwell[i];
      fillD += (i === 0 ? 'M' : 'L') + xAt(p.t).toFixed(2) + ',' + yAt(p.v).toFixed(2) + ' ';
    }
    for (var j = baseline.length - 1; j >= 0; j--) {
      var q = baseline[j];
      fillD += 'L' + xAt(q.t).toFixed(2) + ',' + yAt(q.v).toFixed(2) + ' ';
    }
    fillD += 'Z';
    var fillPath = el('path', { d: fillD, class: 'perf-fill' });
    svg.appendChild(fillPath);

    var basePath = el('path', {
      d: pathFrom(baseline),
      class: 'perf-line perf-line--baseline'
    });
    svg.appendChild(basePath);

    var swPath = el('path', {
      d: pathFrom(shotwell),
      class: 'perf-line perf-line--shotwell'
    });
    svg.appendChild(swPath);

    // Clip the three path-based elements with a left-to-right rect so we can
    // animate them by expanding the rect's width over time.
    var clipId = 'perf-clip-' + Math.floor(Math.random() * 1e9);
    var defs = el('defs', {});
    var clip = el('clipPath', { id: clipId });
    var clipRect = el('rect', {
      x: PAD_L - 4,
      y: PAD_T - 4,
      width: 0,
      height: PH + 8
    });
    clip.appendChild(clipRect);
    defs.appendChild(clip);
    svg.appendChild(defs);
    basePath.setAttribute('clip-path', 'url(#' + clipId + ')');
    swPath.setAttribute('clip-path', 'url(#' + clipId + ')');
    fillPath.setAttribute('clip-path', 'url(#' + clipId + ')');
    basePath.style.strokeDasharray = 'none';
    swPath.style.strokeDasharray = 'none';
    fillPath.style.transition = 'none';
    fillPath.style.opacity = '0.08';

    var baseDot = el('circle', {
      cx: xAt(baseline[0].t), cy: yAt(baseline[0].v), r: 3.5,
      class: 'perf-end-dot perf-end-dot--baseline'
    });
    var swDot = el('circle', {
      cx: xAt(shotwell[0].t), cy: yAt(shotwell[0].v), r: 4.5,
      class: 'perf-end-dot'
    });
    baseDot.style.opacity = '1';
    baseDot.style.transition = 'none';
    swDot.style.opacity = '1';
    swDot.style.transition = 'none';
    svg.appendChild(baseDot);
    svg.appendChild(swDot);

    var annoLayer = document.getElementById('perf-annotations');

    function pctX(t) { return ((PAD_L + t * PW) / W) * 100; }
    function pctY(v) { return ((PAD_T + (1 - v) * PH) / H) * 100; }

    var bEnd = baseline[baseline.length - 1];
    var sEnd = shotwell[shotwell.length - 1];
    var gap = sEnd.v - bEnd.v;
    var gapPct = Math.round(gap * 1000) / 10;

    var annoShotwell = document.createElement('div');
    annoShotwell.className = 'perf-annotation perf-annotation--shotwell';
    annoShotwell.innerHTML = '<strong style="font-weight:500">' + Math.round(sEnd.v * 100) + '%</strong> · with Shotwell';
    annoShotwell.style.left = 'calc(' + pctX(sEnd.t) + '% + -70px)';
    annoShotwell.style.top = 'calc(' + pctY(sEnd.v) + '% + -24px)';
    annoShotwell.style.transform = 'translate(-50%, -50%)';
    annoShotwell.style.transition = 'opacity 0.35s ease';
    annoShotwell.style.opacity = '0';
    annoLayer.appendChild(annoShotwell);

    var annoBase = document.createElement('div');
    annoBase.className = 'perf-annotation perf-annotation--baseline';
    annoBase.innerHTML = Math.round(bEnd.v * 100) + '% · baseline';
    annoBase.style.left = 'calc(' + pctX(bEnd.t) + '% + -60px)';
    annoBase.style.top = 'calc(' + pctY(bEnd.v) + '% + 22px)';
    annoBase.style.transform = 'translate(-50%, -50%)';
    annoBase.style.transition = 'opacity 0.35s ease';
    annoBase.style.opacity = '0';
    annoLayer.appendChild(annoBase);

    var annoGap = document.createElement('div');
    annoGap.className = 'perf-annotation perf-annotation--gap';
    annoGap.innerHTML = '+' + gapPct.toFixed(1) + ' pp';
    annoGap.style.left = 'calc(' + pctX(0.995) + '% + -30px)';
    annoGap.style.top = 'calc(' + pctY((sEnd.v + bEnd.v) / 2) + '% + 0px)';
    annoGap.style.transform = 'translate(-50%, -50%)';
    annoGap.style.transition = 'opacity 0.35s ease';
    annoGap.style.opacity = '0';
    annoLayer.appendChild(annoGap);

    var valBase = document.getElementById('perf-value-baseline');
    var valShot = document.getElementById('perf-value-shotwell');
    function formatPct(n) { return n.toFixed(1) + '%'; }
    if (valBase) valBase.textContent = '0.0%';
    if (valShot) valShot.textContent = '0.0%';

    function pointAt(pts, target) {
      for (var i = pts.length - 1; i >= 0; i--) {
        if (pts[i].t <= target) {
          return pts[i];
        }
      }
      return pts[0];
    }

    var DRAW_MS = 7000;
    var HOLD_MS = 3000;
    var FADE_MS = 500;

    var running = false;
    var animRaf = null;

    function setProgress(p) {
      var clipW = PW * p + 4;
      clipRect.setAttribute('width', clipW);

      var bp = pointAt(baseline, p);
      var sp = pointAt(shotwell, p);
      baseDot.setAttribute('cx', xAt(bp.t));
      baseDot.setAttribute('cy', yAt(bp.v));
      swDot.setAttribute('cx', xAt(sp.t));
      swDot.setAttribute('cy', yAt(sp.v));

      if (valBase) valBase.textContent = formatPct(bp.v * 100);
      if (valShot) valShot.textContent = formatPct(sp.v * 100);

      var aOp = Math.max(0, Math.min(1, (p - 0.7) / 0.25));
      annoShotwell.style.opacity = aOp;
      annoBase.style.opacity = aOp;
      annoGap.style.opacity = aOp;
    }

    function resetProgress() {
      clipRect.setAttribute('width', 0);
      baseDot.setAttribute('cx', xAt(baseline[0].t));
      baseDot.setAttribute('cy', yAt(baseline[0].v));
      swDot.setAttribute('cx', xAt(shotwell[0].t));
      swDot.setAttribute('cy', yAt(shotwell[0].v));
      if (valBase) valBase.textContent = '0.0%';
      if (valShot) valShot.textContent = '0.0%';
      annoShotwell.style.opacity = 0;
      annoBase.style.opacity = 0;
      annoGap.style.opacity = 0;
    }

    function runOnce(onDone) {
      resetProgress();
      var startT = performance.now();
      function tick(now) {
        var elapsed = now - startT;
        if (elapsed < DRAW_MS) {
          var p = elapsed / DRAW_MS;
          var e = p < 1 ? 1 - Math.pow(1 - p, 1.6) : 1;
          setProgress(e);
          animRaf = requestAnimationFrame(tick);
        } else if (elapsed < DRAW_MS + HOLD_MS) {
          setProgress(1);
          animRaf = requestAnimationFrame(tick);
        } else if (elapsed < DRAW_MS + HOLD_MS + FADE_MS) {
          var fp = (elapsed - DRAW_MS - HOLD_MS) / FADE_MS;
          var op = 1 - fp;
          annoShotwell.style.opacity = op;
          annoBase.style.opacity = op;
          annoGap.style.opacity = op;
          animRaf = requestAnimationFrame(tick);
        } else {
          if (onDone) onDone();
        }
      }
      animRaf = requestAnimationFrame(tick);
    }

    function loopAnim() {
      runOnce(function () {
        setTimeout(loopAnim, 400);
      });
    }

    var stage = svg.closest('.reveal');
    function startAnim() {
      if (running) return;
      running = true;
      loopAnim();
    }
    if (stage) {
      if ('MutationObserver' in window) {
        var mo = new MutationObserver(function () {
          if (stage.classList.contains('visible')) {
            startAnim();
            mo.disconnect();
          }
        });
        mo.observe(stage, { attributes: true, attributeFilter: ['class'] });
      }
      if (stage.classList.contains('visible')) startAnim();
    }
  })();
})();
