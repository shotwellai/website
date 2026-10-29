(() => {
  const episodes = window.SHOTWELL_WGO_REFERENCE?.episodes || [];
  const byId = new Map(episodes.map((item) => [item.id, item]));
  const models = window.SHOTWELL_WGO_RESULTS?.models || {};
  const $ = (selector) => document.querySelector(selector);
  const video = $('#episode-video');
  const modelSelect = $('#model-select');
  const pauseAtMajorBoundaries = $('#pause-major-boundaries');
  const episodeList = $('#episode-list');
  const zoomTrack = $('#zoom-track');
  const fullTrack = $('#full-track');
  const zoomSeconds = 60;
  let episode = null;
  let selectedIndex = 0;
  let pendingSeek = null;
  let zoomStart = 0;
  let pausedBoundary = null;
  let boundaryPauseTimer = null;
  let selectedModel = '';

  const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));
  const make = (tag, className, value) => {
    const element = document.createElement(tag);
    if (className) element.className = className;
    if (value != null) element.textContent = value;
    return element;
  };
  const formatTime = (seconds, precise = true) => {
    const units = Math.round(seconds * (precise ? 100 : 1));
    const perMinute = precise ? 6000 : 60;
    const minutes = Math.floor(units / perMinute);
    const remainder = units % perMinute;
    return precise
      ? `${minutes}:${String(Math.floor(remainder / 100)).padStart(2, '0')}.${String(remainder % 100).padStart(2, '0')}`
      : `${minutes}:${String(remainder).padStart(2, '0')}`;
  };
  const rangeText = (start, end) => `${formatTime(start)}–${formatTime(end)}`;
  const percent = (time, start, duration) => `${clamp((time - start) / duration, 0, 1) * 100}%`;
  const zoomLength = () => Math.min(zoomSeconds, episode.duration);
  const modelEpisode = () => models[selectedModel]?.episodes?.[episode?.id];

  function boundaryDescription(boundary) {
    const type = boundary.type === 'minor' ? 'Optional boundary' : 'Required boundary';
    return `${type} at ${formatTime(boundary.time)}; acceptable ${rangeText(boundary.minimum, boundary.maximum)}`;
  }

  function renderBands(container, start, duration) {
    const end = start + duration;
    const fragment = document.createDocumentFragment();
    const descriptions = modelEpisode()?.descriptions;
    episode.segments.forEach((segment, index) => {
      const left = Math.max(segment.start, start);
      const right = Math.min(segment.end, end);
      if (right <= left) return;
      const band = make('span', 'timeline-segment-band');
      if (descriptions?.[index]) band.classList.add(descriptions[index].correct ? 'is-correct' : 'is-incorrect');
      band.style.left = percent(left, start, duration);
      band.style.width = `${(right - left) / duration * 100}%`;
      band.dataset.index = index;
      band.title = `${rangeText(segment.start, segment.end)}: ${segment.required[0]}`;
      fragment.append(band);
    });
    container.replaceChildren(fragment);
  }

  function renderBoundaries(container, start, duration) {
    const end = start + duration;
    const fragment = document.createDocumentFragment();
    episode.boundaries.forEach((boundary) => {
      if (boundary.time <= 0 || boundary.time >= episode.duration) return;
      const left = Math.max(boundary.minimum, start);
      const right = Math.min(boundary.maximum, end);
      const minor = boundary.type === 'minor';
      if (right > left) {
        const window = make('span', `boundary-window${minor ? ' minor' : ''}`);
        window.style.left = percent(left, start, duration);
        window.style.width = `${(right - left) / duration * 100}%`;
        fragment.append(window);
      }
      if (boundary.time < start || boundary.time > end) return;
      const point = make('button', `boundary-marker${minor ? ' minor' : ''}`);
      point.type = 'button';
      point.style.left = percent(boundary.time, start, duration);
      point.title = boundaryDescription(boundary);
      point.setAttribute('aria-label', boundaryDescription(boundary));
      point.addEventListener('click', (event) => {
        event.stopPropagation();
        seek(boundary.time);
      });
      fragment.append(point);
    });
    container.replaceChildren(fragment);
  }

  function renderModelBoundaries(container, start, duration) {
    const result = modelEpisode();
    const fragment = document.createDocumentFragment();
    if (result) {
      result.boundaries.forEach((boundary) => {
        if (boundary.time < start || boundary.time > start + duration) return;
        const marker = make('button', `model-boundary-marker${boundary.status === 'extra' ? ' extra' : ''}`);
        marker.type = 'button';
        marker.style.left = percent(boundary.time, start, duration);
        const status = boundary.status === 'hit' ? 'matched' : 'extra';
        const description = boundary.description ? ` ${boundary.description}` : '';
        marker.title = `${models[selectedModel].name} ${status} boundary at ${formatTime(boundary.time)}.${description}`;
        marker.setAttribute('aria-label', marker.title);
        marker.addEventListener('click', (event) => {
          event.stopPropagation();
          seek(boundary.time);
        });
        fragment.append(marker);
      });
    }
    container.replaceChildren(fragment);
  }

  function updateSelection() {
    const segment = episode.segments[selectedIndex];
    const setRange = (element, start, duration) => {
      const left = Math.max(segment.start, start);
      const right = Math.min(segment.end, start + duration);
      element.style.left = percent(left, start, duration);
      element.style.width = `${Math.max(0, right - left) / duration * 100}%`;
    };
    setRange($('#full-selected-range'), 0, episode.duration);
    setRange($('#zoom-selected-range'), zoomStart, zoomLength());
  }

  function renderDetail() {
    const segment = episode.segments[selectedIndex];
    const result = modelEpisode()?.descriptions[selectedIndex];
    $('#required-list').replaceChildren(...segment.required.map((detail, index) => {
      const item = make('li', result ? 'scored' : '');
      if (result) {
        const matched = result.matchedRequired.includes(index);
        item.classList.add(matched ? 'is-matched' : 'is-missed');
        item.setAttribute('aria-label', `${matched ? 'Matched' : 'Missed'}: ${detail}`);
        item.append(make('span', 'detail-status', matched ? 'Found' : 'Missed'));
      }
      item.append(document.createTextNode(detail));
      return item;
    }));
    $('#optional-list').replaceChildren(...segment.optional.map((detail) => make('li', '', detail)));
    $('#optional-empty').hidden = segment.optional.length > 0;
    const assessment = $('#model-assessment');
    assessment.hidden = !selectedModel;
    if (selectedModel) {
      $('#model-description-label').textContent = `${models[selectedModel].name} description`;
      const verdict = $('#model-verdict');
      verdict.className = `model-verdict ${result?.correct ? 'is-correct' : 'is-incorrect'}`;
      verdict.textContent = result ? (result.correct ? 'Correct' : 'Incorrect') : 'Not scored';
      $('#model-description').textContent = result?.text || 'No description returned for this segment.';
      const issues = $('#model-issues');
      issues.replaceChildren();
      if (result?.unsupported?.length) issues.append(make('p', '', `Unsupported: ${result.unsupported.join('; ')}`));
      if (result?.formatViolation) issues.append(make('p', '', `Format: ${result.formatViolation}`));
      issues.hidden = !issues.childElementCount;
    }
    updateSelection();
  }

  function selectAtTime(time) {
    let index = episode.segments.findIndex((segment, i) =>
      time >= segment.start && (time < segment.end || i === episode.segments.length - 1 && time <= segment.end));
    if (index < 0) {
      index = episode.segments.reduce((best, segment, i) => {
        const gap = Math.max(segment.start - time, time - segment.end, 0);
        const bestSegment = episode.segments[best];
        const bestGap = Math.max(bestSegment.start - time, time - bestSegment.end, 0);
        return gap < bestGap ? i : best;
      }, 0);
    }
    if (index !== selectedIndex) {
      selectedIndex = index;
      renderDetail();
    }
  }

  function renderZoom(time, force = false) {
    const length = zoomLength();
    const start = clamp(time - length / 2, 0, episode.duration - length);
    if (!force && Math.abs(start - zoomStart) < .05) return;
    zoomStart = start;
    renderBands($('#zoom-segments'), start, length);
    renderBoundaries($('#zoom-markers'), start, length);
    renderModelBoundaries($('#zoom-model-markers'), start, length);
    updateSelection();
    $('#zoom-label').textContent = `${formatTime(start, false)}–${formatTime(start + length, false)}`;
    const x0 = start / episode.duration * 1000;
    const x1 = (start + length) / episode.duration * 1000;
    $('#fan-area').setAttribute('points', `0,0 1000,0 ${x1},28 ${x0},28`);
    $('#full-window').style.left = percent(start, 0, episode.duration);
    $('#full-window').style.width = `${length / episode.duration * 100}%`;
  }

  function syncTime(time) {
    if (!episode) return;
    const detailTime = pausedBoundary !== null && video.paused && Math.abs(time - pausedBoundary) < .25
      ? Math.max(0, pausedBoundary - .001) : time;
    selectAtTime(detailTime);
    renderZoom(time);
    $('#zoom-playhead').style.left = percent(time, zoomStart, zoomLength());
    $('#full-playhead').style.left = percent(time, 0, episode.duration);
    [zoomTrack, fullTrack].forEach((track) => {
      track.setAttribute('aria-valuenow', String(time));
      track.setAttribute('aria-valuetext', `${formatTime(time)} of ${formatTime(episode.duration)}`);
    });
  }

  function seek(time) {
    const target = clamp(time, 0, episode.duration);
    clearBoundaryPauseTimer();
    pausedBoundary = null;
    if (video.readyState >= 1) video.currentTime = target;
    else pendingSeek = target;
    syncTime(target);
  }

  function clearBoundaryPauseTimer() {
    if (boundaryPauseTimer !== null) clearTimeout(boundaryPauseTimer);
    boundaryPauseTimer = null;
  }

  function scheduleBoundaryPause() {
    clearBoundaryPauseTimer();
    if (!episode || !pauseAtMajorBoundaries.checked || video.paused || video.seeking || video.readyState < 2 || video.playbackRate <= 0) return;
    const boundary = episode.boundaries.find((item) => item.type !== 'minor'
      && item.time > video.currentTime + .02 && item.time < episode.duration - .01);
    if (!boundary) return;
    const lead = Math.min(.25, video.playbackRate * .12);
    const delay = Math.max(0, (boundary.time - video.currentTime - lead) / video.playbackRate * 1000);
    boundaryPauseTimer = setTimeout(() => {
      boundaryPauseTimer = null;
      if (video.paused || video.seeking || !pauseAtMajorBoundaries.checked || !episode.boundaries.includes(boundary)) return;
      const remaining = boundary.time - video.currentTime;
      if (remaining > lead) {
        scheduleBoundaryPause();
        return;
      }
      video.pause();
      pausedBoundary = boundary.time;
      if (remaining > 0) video.currentTime = boundary.time;
      syncTime(video.currentTime);
    }, delay);
  }

  function renderEpisodeList() {
    const fragment = document.createDocumentFragment();
    episodes.forEach((item, index) => {
      const row = make('button', 'episode-row');
      row.type = 'button';
      row.dataset.episode = item.id;
      row.setAttribute('aria-label', `${item.sourceId}, ${item.task || 'reference episode'}, ${formatTime(item.duration, false)}`);
      const title = make('span', 'episode-row-title');
      title.append(make('span', '', `${index + 1}. ${item.sourceId}`),
        make('span', '', formatTime(item.duration, false)));
      row.append(title, make('span', 'episode-row-task', item.task || item.id));
      row.addEventListener('click', () => showEpisode(item.id));
      fragment.append(row);
    });
    episodeList.replaceChildren(fragment);
  }

  function applyModel() {
    const result = modelEpisode();
    [zoomTrack, fullTrack].forEach((track) => track.classList.toggle('has-model', Boolean(result)));
    $('#model-boundary-legend').hidden = !result;
    const summary = $('#model-boundary-summary');
    summary.hidden = !result;
    if (result) {
      const counts = result.boundarySummary;
      summary.textContent = `${counts.majorHits}/${counts.majorTotal} required matched · ${counts.extra} extra`;
    }
    renderBands($('#full-segments'), 0, episode.duration);
    renderModelBoundaries($('#full-model-markers'), 0, episode.duration);
    renderDetail();
    renderZoom(video.currentTime || 0, true);
    syncTime(video.currentTime || 0);
    const url = new URL(location.href);
    if (selectedModel) url.searchParams.set('model', selectedModel);
    else url.searchParams.delete('model');
    history.replaceState(null, '', url);
  }

  function showEpisode(id) {
    episode = byId.get(id) || episodes[0];
    if (!episode) return;
    clearBoundaryPauseTimer();
    video.pause();
    video.src = episode.videoUrl;
    video.load();
    pendingSeek = null;
    pausedBoundary = null;
    selectedIndex = 0;
    zoomStart = 0;
    $('#episode-id').textContent = episode.id;
    $('#episode-task').textContent = episode.task || episode.sourceId;
    $('#episode-duration').textContent = formatTime(episode.duration);
    episodeList.querySelectorAll('.episode-row').forEach((row) => {
      const active = row.dataset.episode === episode.id;
      row.classList.toggle('is-selected', active);
      if (active) row.setAttribute('aria-current', 'page');
      else row.removeAttribute('aria-current');
    });
    renderBands($('#full-segments'), 0, episode.duration);
    renderBoundaries($('#full-markers'), 0, episode.duration);
    const ruler = $('#time-ruler');
    ruler.replaceChildren();
    for (let step = 0; step <= 4; step += 1) {
      ruler.append(make('span', '', formatTime(episode.duration * step / 4, false)));
    }
    [zoomTrack, fullTrack].forEach((track) => track.setAttribute('aria-valuemax', String(episode.duration)));
    applyModel();
    const url = new URL(location.href);
    url.searchParams.set('episode', episode.id);
    history.replaceState(null, '', url);
  }

  function togglePlayback() {
    if (video.paused) video.play().catch(() => {});
    else video.pause();
  }

  function onTimeUpdate() {
    if (!episode) return;
    syncTime(video.currentTime);
  }

  $('#theme-toggle').addEventListener('click', () => {
    const theme = document.documentElement.dataset.theme === 'light' ? 'dark' : 'light';
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem('shotwell-sample-theme', theme); } catch (e) {}
    $('#theme-toggle').textContent = theme === 'light' ? '\u263e' : '\u263c';
  });
  $('#theme-toggle').textContent = document.documentElement.dataset.theme === 'light' ? '\u263e' : '\u263c';

  Object.entries(models).sort((a, b) => b[1].overallF1 - a[1].overallF1).forEach(([key, model]) => {
    const option = make('option', '', model.name);
    option.value = key;
    modelSelect.append(option);
  });
  modelSelect.disabled = !Object.keys(models).length;
  const requestedModel = new URLSearchParams(location.search).get('model');
  selectedModel = Object.hasOwn(models, requestedModel) ? requestedModel : '';
  modelSelect.value = selectedModel;
  modelSelect.addEventListener('change', () => {
    selectedModel = modelSelect.value;
    if (episode) applyModel();
  });

  if (!episodes.length) {
    episodeList.append(make('p', 'episode-empty', 'No complete reference episodes are available yet.'));
    $('.episode-stage').hidden = true;
    return;
  }
  renderEpisodeList();
  showEpisode(new URLSearchParams(location.search).get('episode'));
  [zoomTrack, fullTrack].forEach((track) => {
    track.addEventListener('click', (event) => {
      const rect = track.getBoundingClientRect();
      const fraction = clamp((event.clientX - rect.left) / rect.width, 0, 1);
      seek(track === zoomTrack ? zoomStart + fraction * zoomLength() : fraction * episode.duration);
    });
    track.addEventListener('keydown', (event) => {
      const delta = event.key === 'ArrowLeft' ? -1 : event.key === 'ArrowRight' ? 1 : 0;
      if (!delta && event.key !== 'Home' && event.key !== 'End') return;
      event.preventDefault();
      seek(event.key === 'Home' ? 0 : event.key === 'End' ? episode.duration : video.currentTime + delta);
    });
  });
  document.addEventListener('keydown', (event) => {
    if (event.code !== 'Space' || event.repeat || event.altKey || event.ctrlKey || event.metaKey) return;
    if (event.target.closest('button, input, select, textarea, [contenteditable="true"]')) return;
    event.preventDefault();
    togglePlayback();
  });
  video.addEventListener('loadedmetadata', () => {
    if (pendingSeek !== null) {
      video.currentTime = pendingSeek;
      pendingSeek = null;
    }
  });
  video.addEventListener('seeking', () => {
    clearBoundaryPauseTimer();
    if (pausedBoundary !== null && Math.abs(video.currentTime - pausedBoundary) > .02) pausedBoundary = null;
  });
  video.addEventListener('seeked', scheduleBoundaryPause);
  video.addEventListener('timeupdate', onTimeUpdate);
  video.addEventListener('play', () => {
    pausedBoundary = null;
    syncTime(video.currentTime);
  });
  video.addEventListener('playing', scheduleBoundaryPause);
  video.addEventListener('waiting', clearBoundaryPauseTimer);
  video.addEventListener('pause', clearBoundaryPauseTimer);
  video.addEventListener('ratechange', scheduleBoundaryPause);
  pauseAtMajorBoundaries.addEventListener('change', scheduleBoundaryPause);
})();
