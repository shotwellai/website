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
  var BG_COLOR = '#F5EADD';
  var NUM_ARMS = 5;

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

  var STIFFNESS = 0.028;
  var DAMPING = 0.86;

  var mouseInCanvas = false;
  var mouseX = 0;
  var mouseY = 0;
  var idleTime = 0;
  var activeBlend = 0;

  var arms = [];

  function createArm(baseX, baseY, phaseOffset) {
    return {
      baseX: baseX,
      baseY: baseY,
      phase: phaseOffset,
      joints: [
        { x: baseX, y: baseY },
        { x: baseX, y: baseY - BASE_LENGTHS[0] * SEG_SCALE },
        { x: baseX, y: baseY - (BASE_LENGTHS[0] + BASE_LENGTHS[1]) * SEG_SCALE },
        { x: baseX, y: baseY - (BASE_LENGTHS[0] + BASE_LENGTHS[1] + BASE_LENGTHS[2]) * SEG_SCALE }
      ],
      smoothTarget: { x: baseX, y: baseY - 140 * SEG_SCALE },
      velocity: { x: 0, y: 0 },
      gripperOpen: 0.35
    };
  }

  function initArms() {
    var baseY = H - 10;
    arms = [];
    // 5 positions, skip the middle one (index 2)
    for (var i = 0; i < NUM_ARMS; i++) {
      if (i === 2) continue;
      var t = (i + 1) / (NUM_ARMS + 1);
      var phase = i * 1.35;
      arms.push(createArm(W * t, baseY, phase));
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

    if (targetY > arm.baseY - 25 * SEG_SCALE) {
      targetY = arm.baseY - 25 * SEG_SCALE;
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

  function springToward(arm, targetX, targetY) {
    var dx = targetX - arm.smoothTarget.x;
    var dy = targetY - arm.smoothTarget.y;
    arm.velocity.x += dx * STIFFNESS;
    arm.velocity.y += dy * STIFFNESS;
    arm.velocity.x *= DAMPING;
    arm.velocity.y *= DAMPING;
    arm.smoothTarget.x += arm.velocity.x;
    arm.smoothTarget.y += arm.velocity.y;
  }

  // Each arm has its own idle motion with phase offset
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

  // Draw rectangular base/pedestal
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

    // Draw base pedestal first
    drawBase(arm);

    // Draw segments (thick outlined rounded rects)
    drawThickSegment(j[0].x, j[0].y - (BASE_H + PEDESTAL_H / 2) * sw, j[1].x, j[1].y, SEG_WIDTH * sw);
    drawThickSegment(j[1].x, j[1].y, j[2].x, j[2].y, (SEG_WIDTH - 2) * sw);
    drawThickSegment(j[2].x, j[2].y, j[3].x, j[3].y, (SEG_WIDTH - 4) * sw);

    // Draw joints on top of segments
    drawJointCircle(j[0].x, j[0].y - (BASE_H + PEDESTAL_H / 2) * sw, JOINT_R * sw);
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

    var targetBlend = mouseInCanvas ? 1 : 0;
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

      springToward(arm, tx, ty);
      solveFABRIK(arm, arm.smoothTarget.x, arm.smoothTarget.y);

      var gripTarget = mouseInCanvas ? 0.6 : 0.35;
      arm.gripperOpen += (gripTarget - arm.gripperOpen) * 0.06;
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

  canvas.addEventListener('touchmove', function (e) {
    var rect = canvas.getBoundingClientRect();
    var touch = e.touches[0];
    mouseX = touch.clientX - rect.left;
    mouseY = touch.clientY - rect.top;
    mouseInCanvas = true;
    e.preventDefault();
  }, { passive: false });

  canvas.addEventListener('touchend', function () {
    mouseInCanvas = false;
  });

  window.addEventListener('resize', resize);
  resize();
  loop();
})();
