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
  if (canvas) {
  var ctx = canvas.getContext('2d');
  var dpr = window.devicePixelRatio || 1;
  var W, H;

  var ARM_COLOR = '#6FA052';
  var BG_COLOR = '#DDE3CF';

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
    var GAP = 100; // px outside the text column
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
    // Rounded rectangle along the segment
    var ax = x1 + px * hw;
    var ay = y1 + py * hw;
    var bx = x2 + px * hw;
    var by = y2 + py * hw;
    var cx = x2 - px * hw;
    var cy = y2 - py * hw;
    var ddx = x1 - px * hw;
    var ddy = y1 - py * hw;

    // Draw with rounded ends
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

  // Draw a hollow joint circle
  function drawJointCircle(x, y, radius) {
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = JOINT_STROKE * SEG_SCALE;
    ctx.stroke();

    // Inner dot
    ctx.beginPath();
    ctx.arc(x, y, radius * 0.3, 0, Math.PI * 2);
    ctx.fillStyle = ARM_COLOR;
    ctx.fill();
  }

  // Draw the gripper/claw
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

    // Wrist connector (short thick piece)
    var wristLen = 6 * SEG_SCALE;
    var wx = tip.x + nx * wristLen;
    var wy = tip.y + ny * wristLen;
    drawThickSegment(tip.x, tip.y, wx, wy, 8 * SEG_SCALE);

    // Fork point
    var forkX = wx;
    var forkY = wy;

    // Two curved fingers
    var spreadAmt = spread * 10 * SEG_SCALE;
    // Finger 1 - upper prong with slight curve
    var mid1x = forkX + nx * gLen * 0.5 + px * spreadAmt * 0.8;
    var mid1y = forkY + ny * gLen * 0.5 + py * spreadAmt * 0.8;
    var end1x = forkX + nx * gLen + px * spreadAmt;
    var end1y = forkY + ny * gLen + py * spreadAmt;
    // Finger tip curves inward
    var tip1x = end1x + px * (-spreadAmt * 0.4);
    var tip1y = end1y + py * (-spreadAmt * 0.4);

    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = fingerW;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // Finger 1
    ctx.beginPath();
    ctx.moveTo(forkX, forkY);
    ctx.quadraticCurveTo(mid1x, mid1y, end1x, end1y);
    ctx.stroke();

    // Finger 2
    var mid2x = forkX + nx * gLen * 0.5 - px * spreadAmt * 0.8;
    var mid2y = forkY + ny * gLen * 0.5 - py * spreadAmt * 0.8;
    var end2x = forkX + nx * gLen - px * spreadAmt;
    var end2y = forkY + ny * gLen - py * spreadAmt;

    ctx.beginPath();
    ctx.moveTo(forkX, forkY);
    ctx.quadraticCurveTo(mid2x, mid2y, end2x, end2y);
    ctx.stroke();
  }

  // Draw rectangular base/pedestal — ground mounted (floor below arm).
  function drawBase(arm) {
    var x = arm.baseX;
    var y = arm.baseY;
    var sw = SEG_SCALE;

    // Pedestal (wider bottom)
    var pw = PEDESTAL_W * sw;
    var ph = PEDESTAL_H * sw;
    ctx.beginPath();
    ctx.rect(x - pw / 2, y - ph / 2, pw, ph);
    ctx.fillStyle = BG_COLOR;
    ctx.fill();
    ctx.strokeStyle = ARM_COLOR;
    ctx.lineWidth = STROKE_W * sw;
    ctx.stroke();

    // Upper base block
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
    // First segment visually starts above the base block (pedestal + block height)
    var startY = j[0].y - (BASE_H + PEDESTAL_H / 2) * sw;

    // Draw base pedestal first (ground mounted)
    drawBase(arm);

    // Draw segments (thick outlined rounded rects)
    drawThickSegment(j[0].x, startY, j[1].x, j[1].y, SEG_WIDTH * sw);
    drawThickSegment(j[1].x, j[1].y, j[2].x, j[2].y, (SEG_WIDTH - 2) * sw);
    drawThickSegment(j[2].x, j[2].y, j[3].x, j[3].y, (SEG_WIDTH - 4) * sw);

    // Draw joints on top of segments
    drawJointCircle(j[0].x, startY, JOINT_R * sw);
    drawJointCircle(j[1].x, j[1].y, JOINT_R * sw);
    drawJointCircle(j[2].x, j[2].y, (JOINT_R - 1.5) * sw);

    // Wrist cap
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

    // When the CTA is hovered, suppress cursor-follow so arms fall back to idle wandering
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

  // When the hero CTA is hovered, arms stop tracking the cursor and drift in idle directions
  document.querySelectorAll('.btn-hero').forEach(function (btn) {
    btn.addEventListener('mouseenter', function () { ctaHovered = true; });
    btn.addEventListener('mouseleave', function () { ctaHovered = false; });
    btn.addEventListener('focus', function () { ctaHovered = true; });
    btn.addEventListener('blur', function () { ctaHovered = false; });
  });

  resize();
  loop();
  } // end if (canvas)
})();

// ─── Temporal Segmentation animation ───
(function () {
  var videoEl = document.getElementById('seg-video');
  var trackEl = document.getElementById('seg-track');
  var actionsListEl = document.getElementById('seg-actions-list');
  var currentActionEl = document.getElementById('seg-current-action');
  var rubricEl = document.getElementById('seg-rubric');
  var countEl = document.getElementById('seg-count');
  var recEl = document.getElementById('seg-rec-time');
  var endEl = document.getElementById('seg-end-time');
  var featureEl = document.getElementById('feature-segmentation');
  var tweaksEl = document.getElementById('tweaks');

  if (!videoEl || !trackEl) return;

  // Detected actions (sequential), each a distinct color for strong contrast
  var ACTIONS = [
    { label: 'Pick up',           start: 0,  end: 1,  color: '#6FA052' },
    { label: 'Straighten',        start: 1,  end: 7,  color: '#4F9E84' },
    { label: 'Fold right sleeve', start: 7,  end: 10, color: '#BF4D34' },
    { label: 'Fold left sleeve',  start: 10, end: 15, color: '#D6A02E' },
    { label: 'Fold first third',  start: 15, end: 16, color: '#7BA63F' },
    { label: 'Fold second third', start: 16, end: 17, color: '#4E7CA8' },
    { label: 'Stack',             start: 17, end: 18, color: '#9D5690' },
    { label: 'Home',              start: 18, end: 19, color: '#7E7E8A' }
  ];

  // SOP rubric — verdict reveals when the playhead reaches the timestamp
  var RUBRIC = [
    { n: 1, title: 'One arm pick up',      desc: 'The shirt should be picked up from the pile and dropped into the center using one arm.',        at: 1,  verdict: 'pass' },
    { n: 2, title: 'No slipping',          desc: "The shirt shouldn't fall from a gripper while it's supposed to be held.",                       at: 4,  verdict: 'fail' },
    { n: 3, title: 'Shirt orientation',    desc: 'The shirt should be facing up with the right sleeve closer to the closer edge of the table.',   at: 5,  verdict: 'pass' },
    { n: 4, title: 'Right sleeve first',   desc: 'The right (closer) sleeve should be folded first.',                                             at: 9,  verdict: 'pass' },
    { n: 5, title: 'Sleeve tucked in',     desc: 'The sleeve is tucked in when the right or left sleeves are folded.',                            at: 14, verdict: 'fail' },
    { n: 6, title: 'One arm stack',        desc: 'The folded shirt should be stacked using only one arm.',                                        at: 18, verdict: 'pass' }
  ];

  var DURATION = 19; // patched from video metadata

  function fmt(t) {
    var m = Math.floor(t / 60);
    var s = t - m * 60;
    return pad(m) + ':' + s.toFixed(2).padStart(5, '0');
  }
  function pad(n) { return (n < 10 ? '0' : '') + n; }

  // ─── Scrubber segments ───
  var segEls = [];
  ACTIONS.forEach(function (a) {
    var seg = document.createElement('div');
    seg.className = 'seg-seg';
    seg.style.setProperty('--seg-color', a.color);
    var fill = document.createElement('div');
    fill.className = 'seg-seg-fill';
    fill.style.background = a.color;
    seg.appendChild(fill);
    trackEl.appendChild(seg);
    segEls.push({ root: seg, fill: fill });
  });
  var playhead = document.createElement('div');
  playhead.className = 'seg-playhead';
  trackEl.appendChild(playhead);

  function layoutSegments() {
    ACTIONS.forEach(function (a, i) {
      var startPct = (a.start / DURATION) * 100;
      var wPct = ((Math.min(a.end, DURATION) - a.start) / DURATION) * 100;
      segEls[i].root.style.left = startPct.toFixed(2) + '%';
      segEls[i].root.style.width = wPct.toFixed(2) + '%';
    });
  }

  // ─── Action chips (below scrubber) ───
  var chipEls = [];
  ACTIONS.forEach(function (a) {
    var chip = document.createElement('div');
    chip.className = 'seg-chip';
    chip.setAttribute('data-state', 'pending');
    chip.innerHTML =
      '<span class="seg-chip-dot" style="background:' + a.color + '"></span>' +
      '<span class="seg-chip-label">' + a.label + '</span>' +
      '<span class="seg-chip-time">' + fmt(a.start) + '</span>';
    actionsListEl.appendChild(chip);
    chipEls.push(chip);
  });

  // ─── Rubric items ───
  var ruleEls = [];
  RUBRIC.forEach(function (r) {
    var item = document.createElement('div');
    item.className = 'seg-rule';
    item.setAttribute('data-verdict', 'pending');
    item.innerHTML =
      '<span class="seg-rule-n">' + (r.n < 10 ? '0' + r.n : r.n) + '</span>' +
      '<div class="seg-rule-body">' +
        '<div class="seg-rule-title">' + r.title + '</div>' +
        '<div class="seg-rule-desc">' + r.desc + '</div>' +
      '</div>' +
      '<span class="seg-rule-verdict">' +
        '<span class="seg-rule-pending">' +
          '<span></span><span></span><span></span>' +
        '</span>' +
        '<span class="seg-rule-result"></span>' +
      '</span>';
    rubricEl.appendChild(item);
    ruleEls.push({ root: item, result: item.querySelector('.seg-rule-result') });
  });

  function findActiveAction(t) {
    for (var i = 0; i < ACTIONS.length; i++) {
      if (t >= ACTIONS[i].start && t < ACTIONS[i].end) return i;
    }
    return t >= DURATION ? ACTIONS.length - 1 : -1;
  }

  function update(t) {
    playhead.style.setProperty('--pos', (t / DURATION).toFixed(4));
    recEl.textContent = fmt(t);

    var active = findActiveAction(t);
    segEls.forEach(function (s, i) {
      var a = ACTIONS[i];
      var st = t < a.start ? 'pending' : (t < a.end ? 'active' : 'done');
      s.root.setAttribute('data-state', st);
      var f = st === 'active' ? (t - a.start) / (a.end - a.start) : (st === 'done' ? 1 : 0);
      s.fill.style.transform = 'scaleX(' + f.toFixed(3) + ')';
    });

    chipEls.forEach(function (chip, i) {
      var a = ACTIONS[i];
      chip.setAttribute('data-state', t < a.start ? 'pending' : (t < a.end ? 'active' : 'done'));
    });
    if (currentActionEl) currentActionEl.textContent = active >= 0 ? ACTIONS[active].label : '—';

    var evaluated = 0;
    RUBRIC.forEach(function (r, i) {
      var item = ruleEls[i];
      if (t >= r.at) {
        evaluated++;
        if (item.root.getAttribute('data-verdict') !== r.verdict) {
          item.root.setAttribute('data-verdict', r.verdict);
          item.result.textContent = r.verdict === 'pass' ? 'PASS' : 'FAIL';
        }
      } else if (item.root.getAttribute('data-verdict') !== 'pending') {
        item.root.setAttribute('data-verdict', 'pending');
        item.result.textContent = '';
      }
    });
    countEl.textContent = evaluated + ' / ' + RUBRIC.length;
  }

  function pollLoop() {
    if (videoEl && !isNaN(videoEl.duration) && videoEl.duration > 0) {
      var t = videoEl.currentTime;
      if (t > DURATION) t = DURATION;
      update(t);
    }
    requestAnimationFrame(pollLoop);
  }

  function onMeta() {
    if (!isNaN(videoEl.duration) && videoEl.duration > 0) {
      DURATION = videoEl.duration;
      if (endEl) endEl.textContent = fmt(DURATION);
      layoutSegments();
    }
  }
  videoEl.addEventListener('loadedmetadata', onMeta);
  videoEl.addEventListener('durationchange', onMeta);
  if (videoEl.readyState >= 1) onMeta();
  layoutSegments();

  function playClip() {
    if (!videoEl) return;
    videoEl.muted = true;
    var p = videoEl.play();
    if (p && p.catch) p.catch(function () {});
  }
  function pauseClip() { if (videoEl) videoEl.pause(); }

  if ('IntersectionObserver' in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (e) {
        if (e.isIntersecting) playClip(); else pauseClip();
      });
    }, { threshold: 0.25 });
    io.observe(featureEl);
  } else {
    playClip();
  }

  requestAnimationFrame(pollLoop);

  // ─── Tweaks wiring ───
  var TWEAK_DEFAULTS = /*EDITMODE-BEGIN*/{
    "variant": "default",
    "layout": "split"
  }/*EDITMODE-END*/;
  var tweakState = Object.assign({}, TWEAK_DEFAULTS);

  function applyTweaks() {
    if (tweakState.layout === 'filmstrip') {
      featureEl.setAttribute('data-variant', 'filmstrip');
      featureEl.classList.toggle('color-terminal', tweakState.variant === 'terminal');
      featureEl.classList.toggle('color-paper', tweakState.variant === 'paper');
    } else {
      featureEl.setAttribute('data-variant', tweakState.variant);
      featureEl.classList.remove('color-terminal', 'color-paper');
    }
    tweaksEl.querySelectorAll('.tweak-opts').forEach(function (group) {
      var key = group.getAttribute('data-tweak');
      group.querySelectorAll('.tweak-opt').forEach(function (btn) {
        btn.setAttribute('data-selected', btn.getAttribute('data-value') === tweakState[key] ? 'true' : 'false');
      });
    });
  }

  tweaksEl.querySelectorAll('.tweak-opt').forEach(function (btn) {
    btn.addEventListener('click', function () {
      var key = btn.parentElement.getAttribute('data-tweak');
      var val = btn.getAttribute('data-value');
      tweakState[key] = val;
      applyTweaks();
      try {
        window.parent.postMessage({ type: '__edit_mode_set_keys', edits: { [key]: val } }, '*');
      } catch (e) {}
    });
  });

  applyTweaks();

  window.addEventListener('message', function (e) {
    if (!e.data || typeof e.data !== 'object') return;
    if (e.data.type === '__activate_edit_mode') tweaksEl.classList.add('open');
    if (e.data.type === '__deactivate_edit_mode') tweaksEl.classList.remove('open');
  });
  try { window.parent.postMessage({ type: '__edit_mode_available' }, '*'); } catch (e) {}

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
      // Bias toward middle rows so arrows have room
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
      // Target coords in SVG-local space (SVG has no viewBox, so 1 user unit = 1 CSS px)
      var tx = (dotRect.left + dotRect.width / 2) - svgRect.left;
      var ty = (dotRect.top + dotRect.height / 2) - svgRect.top;
      var dotR = dotRect.width / 2 + 4;

      var labelW = label.length * 6.4 + 16;
      var labelH = 22;
      var stageW = svgRect.width;
      var stageH = svgRect.height;
      var PAD = 6;

      // anchor point on the dot edge
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

      // Clamp label position so its bg stays inside stage bounds
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

      // curved path from label area → dot edge
      var path = document.createElementNS(svgNS, 'path');
      var c1x = (lx + ax) / 2;
      var c1y = ly;
      var c2x = (lx + ax) / 2;
      var c2y = ay;
      path.setAttribute('d', 'M' + lx + ',' + ly + ' C' + c1x + ',' + c1y + ' ' + c2x + ',' + c2y + ' ' + ax + ',' + ay);
      path.setAttribute('pathLength', '100');
      path.setAttribute('class', 'q-arrow-line');
      arrowsSvg.appendChild(path);

      // arrow head
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

      // label background
      var bg = document.createElementNS(svgNS, 'rect');
      bg.setAttribute('x', bgX);
      bg.setAttribute('y', bgY);
      bg.setAttribute('width', labelW);
      bg.setAttribute('height', labelH);
      bg.setAttribute('rx', 3);
      bg.setAttribute('class', 'q-arrow-label-bg');
      arrowsSvg.appendChild(bg);

      // label text
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
      var cy = (dotRect.top + dotRect.height / 2) - svgRect.top;
      var relX = cx / svgRect.width;
      var relY = cy / svgRect.height;
      // Prefer left/right; pick whichever side has more room
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

    // Kick off when in view
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

    // y axis: success rate 0..1, shown 0..100
    // x axis: training steps 0..100k

    function xAt(t) { return PAD_L + t * PW; }        // t in 0..1
    function yAt(v) { return PAD_T + (1 - v) * PH; }  // v in 0..1

    // Build a baseline curve that plateaus ~0.82 with noise.
    // Shotwell curve climbs faster and plateaus ~0.94.
    function smoothNoise(seed) {
      // Deterministic pseudo-random based on seed so layout is reproducible
      var s = seed;
      return function () {
        s = (s * 9301 + 49297) % 233280;
        return s / 233280;
      };
    }

    function buildCurve(opts) {
      // opts: { asymp, rate, start, noise, rand }
      var pts = [];
      var N = 180;
      var rand = opts.rand;
      var lastNoise = 0;
      for (var i = 0; i <= N; i++) {
        var t = i / N;
        // Exponential approach to asymp
        var v = opts.start + (opts.asymp - opts.start) * (1 - Math.exp(-opts.rate * t));
        // Smooth noise (low-pass random walk)
        var target = (rand() - 0.5) * opts.noise;
        lastNoise += (target - lastNoise) * 0.35;
        // Attenuate noise near the start (curves start clean)
        var noiseGate = Math.min(1, t * 3);
        v += lastNoise * noiseGate;
        // Clamp
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
    // Horizontal grid lines at 0, 25, 50, 75, 100
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
    // Vertical tick marks
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
    // Axis labels
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

    // Gap fill between curves (shotwell above baseline)
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

    // Baseline line
    var basePath = el('path', {
      d: pathFrom(baseline),
      class: 'perf-line perf-line--baseline'
    });
    svg.appendChild(basePath);

    // Shotwell line
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
    // Start fully hidden
    basePath.style.strokeDasharray = 'none';
    swPath.style.strokeDasharray = 'none';
    fillPath.style.transition = 'none';
    fillPath.style.opacity = '0.08';

    // Moving "current step" dots ride along each curve as it draws
    var baseDot = el('circle', {
      cx: xAt(baseline[0].t), cy: yAt(baseline[0].v), r: 3.5,
      class: 'perf-end-dot perf-end-dot--baseline'
    });
    var swDot = el('circle', {
      cx: xAt(shotwell[0].t), cy: yAt(shotwell[0].v), r: 4.5,
      class: 'perf-end-dot'
    });
    // Force immediate visibility (no CSS transition delay)
    baseDot.style.opacity = '1';
    baseDot.style.transition = 'none';
    swDot.style.opacity = '1';
    swDot.style.transition = 'none';
    svg.appendChild(baseDot);
    svg.appendChild(swDot);

    // HTML annotations positioned over the chart (populated dynamically)
    var annoLayer = document.getElementById('perf-annotations');

    function pctX(t) { return ((PAD_L + t * PW) / W) * 100; }
    function pctY(v) { return ((PAD_T + (1 - v) * PH) / H) * 100; }

    var bEnd = baseline[baseline.length - 1];
    var sEnd = shotwell[shotwell.length - 1];
    var gap = sEnd.v - bEnd.v;
    var gapPct = Math.round(gap * 1000) / 10;

    // Build annotations upfront but hide them; they fade in near the end of the draw
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

    // Legend live values
    var valBase = document.getElementById('perf-value-baseline');
    var valShot = document.getElementById('perf-value-shotwell');
    function formatPct(n) { return n.toFixed(1) + '%'; }
    if (valBase) valBase.textContent = '0.0%';
    if (valShot) valShot.textContent = '0.0%';

    // Helper: find the last point in pts whose t <= target
    function pointAt(pts, target) {
      // Linear search (pts is short-ish)
      for (var i = pts.length - 1; i >= 0; i--) {
        if (pts[i].t <= target) {
          return pts[i];
        }
      }
      return pts[0];
    }

    // ─── Progressive training animation ───
    // Total "training" lasts ~7s of drawing + 3s hold + fade-out + loop
    var DRAW_MS = 7000;
    var HOLD_MS = 3000;
    var FADE_MS = 500;

    var running = false;
    var animRaf = null;

    function setProgress(p) {
      // p in [0, 1] — fraction of training steps completed
      var clipW = PW * p + 4; // +4 to match initial x offset padding
      clipRect.setAttribute('width', clipW);

      // Move end dots along their curves
      var bp = pointAt(baseline, p);
      var sp = pointAt(shotwell, p);
      baseDot.setAttribute('cx', xAt(bp.t));
      baseDot.setAttribute('cy', yAt(bp.v));
      swDot.setAttribute('cx', xAt(sp.t));
      swDot.setAttribute('cy', yAt(sp.v));

      // Update legend values
      if (valBase) valBase.textContent = formatPct(bp.v * 100);
      if (valShot) valShot.textContent = formatPct(sp.v * 100);

      // Fade annotations in as we approach the end
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
          // Ease slightly — slow start, natural finish
          var e = p < 1 ? 1 - Math.pow(1 - p, 1.6) : 1;
          setProgress(e);
          animRaf = requestAnimationFrame(tick);
        } else if (elapsed < DRAW_MS + HOLD_MS) {
          setProgress(1);
          animRaf = requestAnimationFrame(tick);
        } else if (elapsed < DRAW_MS + HOLD_MS + FADE_MS) {
          // Fade out annotations and end dots for a clean loop
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
        // Short pause before restart
        setTimeout(loopAnim, 400);
      });
    }

    // Hook into the existing reveal
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
