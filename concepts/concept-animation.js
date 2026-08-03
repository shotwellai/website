(function () {
  var DEFAULT_ACTIONS = [
    { label: "Pick up", start: 0, end: 1, color: "#6E6E78", result: "pass" },
    { label: "Straighten", start: 1, end: 7, color: "#4A4A52", result: "pass" },
    { label: "Fold right sleeve", start: 7, end: 10, color: "#BF4D34", result: "pass" },
    { label: "Fold left sleeve", start: 10, end: 15, color: "#D6A02E", result: "fail" },
    { label: "Fold up bottom", start: 15, end: 16, color: "#8E8E96", result: "pass" },
    { label: "Fold down top", start: 16, end: 17, color: "#4E7CA8", result: "pass" },
    { label: "Stack", start: 17, end: 18, color: "#9D5690", result: "pass" },
    { label: "Home", start: 18, end: 19, color: "#7E7E8A", result: "pass" }
  ];

  function pad(n) {
    return (n < 10 ? "0" : "") + n;
  }

  function fmtShort(t) {
    var total = Math.max(0, Math.round(t));
    var m = Math.floor(total / 60);
    var s = total - m * 60;
    return m + ":" + pad(s);
  }

  function fmt(t) {
    var m = Math.floor(t / 60);
    var s = t - m * 60;
    return pad(m) + ":" + s.toFixed(2).padStart(5, "0");
  }

  function clamp(n, min, max) {
    return Math.max(min, Math.min(max, n));
  }

  function actionState(action, time) {
    return time < action.start ? "pending" : time < action.end ? "active" : "done";
  }

  function normalizeAction(action, index) {
    var palette = ["#6E6E78", "#4A4A52", "#BF4D34", "#D6A02E", "#8E8E96", "#4E7CA8", "#9D5690", "#7E7E8A"];
    return {
      label: action.label || "Action " + pad(index + 1),
      start: Number(action.start) || 0,
      end: Number(action.end) || Number(action.start) + 1 || 1,
      color: action.color || palette[index % palette.length],
      result: action.result === "fail" ? "fail" : "pass"
    };
  }

  function getActions(root) {
    var raw = root.getAttribute("data-actions");
    if (!raw) return DEFAULT_ACTIONS;
    try {
      var parsed = JSON.parse(raw);
      if (Array.isArray(parsed) && parsed.length) return parsed.map(normalizeAction);
    } catch (err) {}
    return DEFAULT_ACTIONS;
  }

  function initSegmentation(root) {
    var videoEl = root.querySelector(".seg-video-el");
    var trackEl = root.querySelector(".seg-scrubber-track");
    var labelLayerEl = root.querySelector(".seg-timeline-labels");
    var annotationListEl = root.querySelector(".seg-annotations-list");
    var playToggleEl = root.querySelector(".seg-play-toggle");
    if (!videoEl || !trackEl) return;

    var actions = getActions(root);
    var playbackRate = Number(root.getAttribute("data-playback-rate")) || 1;
    var duration = actions.reduce(function (max, action) { return Math.max(max, action.end); }, 0) || 19;
    var segEls = [];
    var labelEls = [];
    var annotationEls = [];

    videoEl.defaultPlaybackRate = playbackRate;
    videoEl.playbackRate = playbackRate;

    actions.forEach(function (action) {
      var seg = document.createElement("div");
      seg.className = "seg-seg";
      seg.style.setProperty("--seg-color", action.color);

      var fill = document.createElement("div");
      fill.className = "seg-seg-fill";
      fill.style.background = action.color;

      seg.appendChild(fill);
      trackEl.appendChild(seg);
      segEls.push({ root: seg, fill: fill });
    });

    var playhead = document.createElement("div");
    playhead.className = "seg-playhead";
    trackEl.appendChild(playhead);

    if (labelLayerEl) {
      actions.forEach(function (action, index) {
        var label = document.createElement("div");
        label.className = "seg-label";
        label.setAttribute("data-state", "pending");
        label.setAttribute("data-side", index % 2 === 0 ? "top" : "bottom");
        label.setAttribute("data-result", action.result);
        label.style.setProperty("--seg-color", action.color);
        label.innerHTML =
          '<span class="seg-label-text">' + action.label + "</span>" +
          '<span class="seg-label-status" aria-hidden="true"></span>';
        labelLayerEl.appendChild(label);
        labelEls.push(label);
      });
    }

    if (annotationListEl) {
      actions.forEach(function (action) {
        var row = document.createElement("li");
        row.className = "seg-annotation";
        row.setAttribute("data-result", action.result);
        row.setAttribute("data-state", "pending");
        row.style.setProperty("--seg-color", action.color);
        row.innerHTML =
          '<span class="seg-annotation-name">' + action.label + "</span>" +
          '<span class="seg-annotation-range">[' + fmtShort(action.start) + " - " + fmtShort(action.end) + "]</span>" +
          '<span class="seg-annotation-status">' + action.result.toUpperCase() + "</span>";
        annotationListEl.appendChild(row);
        annotationEls.push(row);
      });
    }

    function layoutSegments() {
      actions.forEach(function (action, index) {
        var startPct = action.start / duration * 100;
        var widthPct = (Math.min(action.end, duration) - action.start) / duration * 100;
        segEls[index].root.style.left = startPct.toFixed(2) + "%";
        segEls[index].root.style.width = widthPct.toFixed(2) + "%";
      });
      positionTimelineLabels();
    }

    function positionTimelineLabels() {
      if (!labelLayerEl || !labelEls.length) return;
      var trackWidth = trackEl.clientWidth;
      if (!trackWidth) return;

      var lanes = { top: [], bottom: [] };
      var gap = 4;
      var crowded = trackWidth < 520;

      function setLabelPosition(item) {
        item.label.setAttribute("data-side", item.side);
        item.label.style.setProperty("--label-left", item.left.toFixed(1) + "px");
        item.label.style.setProperty("--pointer-x", clamp(item.center - item.left, 8, Math.max(item.width - 8, 8)).toFixed(1) + "px");
        item.label.style.setProperty("--label-top", item.side === "top" ? "-62px" : "0px");
      }

      function packLane(items) {
        if (!items.length) return true;
        var totalWidth = 0;
        items.forEach(function (item, index) {
          totalWidth += item.width + (index === 0 ? 0 : gap);
        });
        if (totalWidth > trackWidth) return false;

        items.sort(function (a, b) { return a.center - b.center; });
        items.forEach(function (item) {
          item.left = clamp(item.center - item.width / 2, 0, Math.max(trackWidth - item.width, 0));
        });

        for (var i = 1; i < items.length; i++) {
          var minLeft = items[i - 1].left + items[i - 1].width + gap;
          if (items[i].left < minLeft) items[i].left = minLeft;
        }

        var last = items[items.length - 1];
        var overflow = last.left + last.width - trackWidth;
        if (overflow > 0) last.left -= overflow;

        for (var j = items.length - 2; j >= 0; j--) {
          var maxLeft = items[j + 1].left - gap - items[j].width;
          if (items[j].left > maxLeft) items[j].left = maxLeft;
        }

        for (var k = 0; k < items.length; k++) {
          if (items[k].left < 0 || items[k].left + items[k].width > trackWidth) return false;
        }

        items.forEach(setLabelPosition);
        return true;
      }

      labelEls.forEach(function (label, index) {
        var action = actions[index];
        var centerPx = (action.start + action.end) / 2 / duration * trackWidth;
        var width = label.offsetWidth || 80;
        var item = {
          label: label,
          side: index % 2 === 0 ? "top" : "bottom",
          center: centerPx,
          width: width,
          left: clamp(centerPx - width / 2, 0, Math.max(trackWidth - width, 0))
        };
        lanes[item.side].push(item);
        if (crowded) setLabelPosition(item);
      });

      if (!crowded) {
        crowded = !packLane(lanes.top) || !packLane(lanes.bottom);
        if (crowded) lanes.top.concat(lanes.bottom).forEach(setLabelPosition);
      }
      labelLayerEl.classList.toggle("is-condensed", crowded);
    }

    function update(time) {
      time = clamp(time, 0, duration);
      playhead.style.setProperty("--pos", (time / duration).toFixed(4));
      trackEl.setAttribute("aria-valuenow", time.toFixed(2));
      trackEl.setAttribute("aria-valuetext", fmt(time));

      segEls.forEach(function (seg, index) {
        var action = actions[index];
        var state = actionState(action, time);
        var fill = state === "active" ? (time - action.start) / (action.end - action.start) : state === "done" ? 1 : 0;
        seg.root.setAttribute("data-state", state);
        seg.fill.style.transform = "scaleX(" + fill.toFixed(3) + ")";
      });

      labelEls.forEach(function (label, index) {
        label.setAttribute("data-state", actionState(actions[index], time));
      });

      annotationEls.forEach(function (row, index) {
        row.setAttribute("data-state", actionState(actions[index], time));
      });
    }

    function onMeta() {
      if (!isNaN(videoEl.duration) && videoEl.duration > 0) {
        duration = videoEl.duration;
        trackEl.setAttribute("aria-valuemax", duration.toFixed(2));
        layoutSegments();
      }
    }

    function seekTo(time) {
      var maxTime = !isNaN(videoEl.duration) && videoEl.duration > 0 ? Math.min(duration, videoEl.duration) : duration;
      var next = clamp(time, 0, maxTime);
      videoEl.currentTime = next;
      update(next);
    }

    function pointerTime(event) {
      var rect = trackEl.getBoundingClientRect();
      var ratio = rect.width ? (event.clientX - rect.left) / rect.width : 0;
      return clamp(ratio, 0, 1) * duration;
    }

    var isScrubbing = false;
    function startScrub(event) {
      if (event.button != null && event.button !== 0) return;
      event.preventDefault();
      isScrubbing = true;
      trackEl.classList.add("is-dragging");
      if (trackEl.setPointerCapture && event.pointerId != null) trackEl.setPointerCapture(event.pointerId);
      seekTo(pointerTime(event));
    }

    function moveScrub(event) {
      if (!isScrubbing) return;
      event.preventDefault();
      seekTo(pointerTime(event));
    }

    function endScrub(event) {
      if (!isScrubbing) return;
      isScrubbing = false;
      trackEl.classList.remove("is-dragging");
      if (trackEl.releasePointerCapture && event.pointerId != null) {
        try { trackEl.releasePointerCapture(event.pointerId); } catch (err) {}
      }
    }

    function updatePlayToggle() {
      if (!playToggleEl) return;
      var playing = !videoEl.paused && !videoEl.ended;
      playToggleEl.dataset.state = playing ? "playing" : "paused";
      playToggleEl.setAttribute("aria-label", playing ? "Pause animation" : "Play animation");
      playToggleEl.setAttribute("aria-pressed", playing ? "true" : "false");
    }

    function playClip() {
      videoEl.muted = true;
      videoEl.playbackRate = playbackRate;
      var promise = videoEl.play();
      if (promise && promise.catch) promise.catch(function () {});
    }

    trackEl.addEventListener("pointerdown", startScrub);
    trackEl.addEventListener("pointermove", moveScrub);
    trackEl.addEventListener("pointerup", endScrub);
    trackEl.addEventListener("pointercancel", endScrub);
    trackEl.addEventListener("keydown", function (event) {
      var current = videoEl.currentTime || 0;
      var step = event.shiftKey ? 5 : 1;
      var next = null;
      if (event.key === "ArrowLeft") next = current - step;
      if (event.key === "ArrowRight") next = current + step;
      if (event.key === "Home") next = 0;
      if (event.key === "End") next = duration;
      if (next == null) return;
      event.preventDefault();
      seekTo(next);
    });

    if (playToggleEl) {
      playToggleEl.dataset.state = "paused";
      playToggleEl.addEventListener("click", function () {
        if (videoEl.paused) playClip();
        else videoEl.pause();
      });
    }

    videoEl.addEventListener("loadedmetadata", onMeta);
    videoEl.addEventListener("durationchange", onMeta);
    videoEl.addEventListener("play", updatePlayToggle);
    videoEl.addEventListener("pause", updatePlayToggle);
    videoEl.addEventListener("ended", updatePlayToggle);

    if (videoEl.readyState >= 1) onMeta();
    layoutSegments();
    update(0);

    if ("ResizeObserver" in window) {
      new ResizeObserver(positionTimelineLabels).observe(trackEl);
    } else {
      window.addEventListener("resize", positionTimelineLabels);
    }
    if (document.fonts && document.fonts.ready) {
      document.fonts.ready.then(positionTimelineLabels).catch(function () {});
    }

    function loop() {
      if (!isNaN(videoEl.duration) && videoEl.duration > 0) update(Math.min(videoEl.currentTime, duration));
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
    playClip();
  }

  document.querySelectorAll("[data-concept-seg]").forEach(initSegmentation);
})();
