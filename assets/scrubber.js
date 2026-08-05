/* Review timeline bound to a video element.
   Click or drag anywhere on the track to seek. Segments carry the rubric
   output — pass regions are neutral, failures carry the accent.

   Segments are either legacy arrays [start, end, 'p'|'f'] or objects
   {a, b, fail, label, rules} for named subtasks. With opts.snap, a plain
   click (no drag) lands on the start of the clicked subtask. */

export function scrubber(track, video, segs, opts) {
  var o = Object.assign({ onSeek: null, snap: false, rich: false }, opts || {});

  var S = segs.map(function (s) {
    return Array.isArray(s)
      ? { a: s[0], b: s[1], fail: s[2] === 'f' }
      : { a: s.a, b: s.b, fail: !!s.fail, label: s.label, rules: s.rules || [],
          color: s.color };
  });

  track.innerHTML = '';
  var els = [], fills = [];
  S.forEach(function (s) {
    var d = document.createElement('div');
    d.className = 'seg ' + (s.fail ? 'f' : 'p');
    d.style.left = (s.a * 100) + '%';
    d.style.width = ((s.b - s.a) * 100 - 0.35) + '%';
    d.title = s.label || (s.fail ? 'flagged on review' : 'passed');
    if (s.color) d.style.setProperty('--segc', s.color);
    if (o.rich) {
      // played-portion fill, animated per segment like the live site's scrubber
      var fi = document.createElement('i');
      fi.className = 'fill';
      if (s.color) fi.style.background = s.color;
      d.appendChild(fi);
      fills.push(fi);
    }
    els.push(d);
    track.appendChild(d);
  });
  var head = document.createElement('div');
  head.className = 'play';
  track.appendChild(head);

  track.setAttribute('role', 'slider');
  track.setAttribute('tabindex', '0');
  track.setAttribute('aria-label', 'Episode timeline');
  track.setAttribute('aria-valuemin', '0');
  track.setAttribute('aria-valuemax', '100');

  function seekTo(clientX) {
    var b = track.getBoundingClientRect();
    var f = (clientX - b.left) / b.width;
    f = f < 0 ? 0 : f > 1 ? 1 : f;
    if (video.duration) video.currentTime = f * video.duration;
    if (o.onSeek) o.onSeek(f);
    return f;
  }

  var dragging = false, downX = 0;
  track.addEventListener('pointerdown', function (e) {
    dragging = true; downX = e.clientX;
    track.setPointerCapture(e.pointerId);
    seekTo(e.clientX);
    e.preventDefault();
  });
  track.addEventListener('pointermove', function (e) {
    if (dragging) seekTo(e.clientX);
  });
  ['pointerup', 'pointercancel'].forEach(function (ev) {
    track.addEventListener(ev, function (e) {
      if (dragging && o.snap && ev === 'pointerup' && Math.abs(e.clientX - downX) < 5) {
        // a click, not a drag — land on the start of the clicked subtask
        var b = track.getBoundingClientRect();
        var f = Math.min(1, Math.max(0, (e.clientX - b.left) / b.width));
        if (video.duration) video.currentTime = segAt(f).seg.a * video.duration;
      }
      dragging = false;
      if (track.hasPointerCapture && track.hasPointerCapture(e.pointerId)) {
        track.releasePointerCapture(e.pointerId);
      }
    });
  });

  track.addEventListener('keydown', function (e) {
    if (!video.duration) return;
    var step = e.shiftKey ? 5 : 1;
    if (e.key === 'ArrowRight') { video.currentTime = Math.min(video.duration, video.currentTime + step); e.preventDefault(); }
    else if (e.key === 'ArrowLeft') { video.currentTime = Math.max(0, video.currentTime - step); e.preventDefault(); }
    else if (e.key === 'Home') { video.currentTime = 0; e.preventDefault(); }
    else if (e.key === 'End') { video.currentTime = video.duration; e.preventDefault(); }
  });

  /* Which segment is the playhead inside? Lets the page name the current state. */
  function segAt(f) {
    for (var i = 0; i < S.length; i++) if (f >= S[i].a && f < S[i].b) return { seg: S[i], i: i };
    return { seg: S[S.length - 1], i: S.length - 1 };
  }

  return {
    segs: S,
    update: function () {
      if (!video.duration) return null;
      var f = video.currentTime / video.duration;
      head.style.left = (f * 100) + '%';
      track.setAttribute('aria-valuenow', Math.round(f * 100));
      var at = segAt(f);
      if (o.rich) {
        S.forEach(function (s, j) {
          var fr = f >= s.b ? 1 : f <= s.a ? 0 : (f - s.a) / (s.b - s.a);
          fills[j].style.transform = 'scaleX(' + fr + ')';
          els[j].dataset.active = String(j === at.i);
        });
      }
      return { f: f, seg: at.seg, i: at.i };
    }
  };
}

export function clock(t) {
  if (!isFinite(t)) return '0:00';
  return Math.floor(t / 60) + ':' + String(Math.floor(t % 60)).padStart(2, '0');
}
