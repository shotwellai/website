(() => {
  const data = window.SHOTWELL_WGO_REFERENCE;
  const episodes = data?.episodes || [];
  const byId = new Map(episodes.map((episode) => [episode.id, episode]));
  const $ = (selector) => document.querySelector(selector);
  const video = $('#episode-video');
  const episodeSelect = $('#episode-select');
  const search = $('#segment-search');
  const segmentList = $('#segment-list');
  const track = $('#boundary-track');
  const markers = $('#boundary-markers');
  const selectedRange = $('#selected-range');
  const playhead = $('#playhead');
  let episode = null;
  let selectedIndex = 0;
  let pendingSeek = null;

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
  const position = (time) => `${Math.max(0, Math.min(100, time / episode.duration * 100))}%`;
  const make = (tag, className, value) => {
    const element = document.createElement(tag);
    if (className) element.className = className;
    if (value != null) element.textContent = value;
    return element;
  };

  function updateTime(time = video.currentTime || 0) {
    if (!episode) return;
    playhead.style.left = position(time);
    $('#playback-time').textContent = `${formatTime(time)} / ${formatTime(episode.duration)}`;
  }

  function seek(time) {
    const target = Math.max(0, Math.min(episode.duration, time));
    if (video.readyState >= 1) video.currentTime = target;
    else pendingSeek = target;
    updateTime(target);
  }

  function boundaryDescription(boundary) {
    const type = boundary.type === 'minor' ? 'Optional split' : 'Major boundary';
    return `${type} at ${formatTime(boundary.time)}; acceptable ${rangeText(boundary.minimum, boundary.maximum)}`;
  }

  function renderBoundaries() {
    markers.replaceChildren();
    episode.boundaries.forEach((boundary) => {
      if (boundary.time <= 0 || boundary.time >= episode.duration) return;
      const span = make('span', `boundary-window ${boundary.type === 'minor' ? 'minor' : ''}`);
      span.style.left = position(boundary.minimum);
      span.style.width = `${Math.max(0, (boundary.maximum - boundary.minimum) / episode.duration * 100)}%`;
      markers.append(span);
      const point = make('button', `boundary-marker ${boundary.type === 'minor' ? 'minor' : ''}`);
      point.type = 'button';
      point.style.left = position(boundary.time);
      point.title = boundaryDescription(boundary);
      point.setAttribute('aria-label', boundaryDescription(boundary));
      point.addEventListener('click', (event) => {
        event.stopPropagation();
        seek(boundary.time);
        selectAtTime(boundary.time);
      });
      markers.append(point);
    });
    const ruler = $('#time-ruler');
    ruler.replaceChildren();
    for (let step = 0; step <= 4; step += 1) {
      ruler.append(make('span', '', formatTime(episode.duration * step / 4, false)));
    }
  }

  function segmentMatches(segment, query) {
    if (!query) return true;
    return [...segment.required, ...segment.optional, rangeText(segment.start, segment.end)]
      .join(' ').toLowerCase().includes(query);
  }

  function renderSegments() {
    const query = search.value.trim().toLowerCase();
    const fragment = document.createDocumentFragment();
    let count = 0;
    episode.segments.forEach((segment, index) => {
      if (!segmentMatches(segment, query)) return;
      count += 1;
      const row = make('button', `segment-row${index === selectedIndex ? ' is-selected' : ''}`);
      row.type = 'button';
      row.dataset.index = index;
      row.setAttribute('aria-label', `Segment ${index + 1}, ${rangeText(segment.start, segment.end)}. ${segment.required[0]}`);
      if (index === selectedIndex) row.setAttribute('aria-current', 'true');
      row.append(make('span', 'segment-number', String(index + 1).padStart(2, '0')));
      const body = make('span', 'segment-body');
      body.append(make('span', 'segment-text', segment.required[0]));
      const optional = segment.optional.length ? ` · ${segment.optional.length} optional` : '';
      body.append(make('span', 'segment-meta', `${rangeText(segment.start, segment.end)}${optional}`));
      row.append(body);
      row.addEventListener('click', () => selectSegment(index, true));
      fragment.append(row);
    });
    segmentList.replaceChildren(fragment);
    if (!count) segmentList.append(make('p', 'segment-empty', 'No matching segments.'));
    $('#segment-count').textContent = `${count} / ${episode.segments.length}`;
  }

  function boundaryWindow(id, recordedTime) {
    const boundary = episode.boundaries.find((item) => item.id === id);
    if (!boundary) return `Recorded ${formatTime(recordedTime)}`;
    const type = boundary.type === 'minor' ? 'optional split' : 'major';
    return `${rangeText(boundary.minimum, boundary.maximum)} · ${type}`;
  }

  function renderDetail() {
    const segment = episode.segments[selectedIndex];
    $('#detail-index').textContent = `SEGMENT ${selectedIndex + 1} / ${episode.segments.length}`;
    $('#detail-time').textContent = rangeText(segment.start, segment.end);
    $('#previous-segment').disabled = selectedIndex === 0;
    $('#next-segment').disabled = selectedIndex === episode.segments.length - 1;
    const windows = $('#boundary-windows');
    windows.replaceChildren();
    [['Start window', segment.startBoundaryId, segment.start],
      ['End window', segment.endBoundaryId, segment.end]].forEach(([label, id, time]) => {
      const item = make('span');
      item.append(make('strong', '', `${label}: `), document.createTextNode(boundaryWindow(id, time)));
      windows.append(item);
    });
    const required = $('#required-list');
    required.replaceChildren(...segment.required.map((detail) => make('li', '', detail)));
    const optional = $('#optional-list');
    optional.replaceChildren(...segment.optional.map((detail) => make('li', '', detail)));
    $('#optional-group').hidden = segment.optional.length === 0;
    selectedRange.style.left = position(segment.start);
    selectedRange.style.width = `${Math.max(0, (segment.end - segment.start) / episode.duration * 100)}%`;
    segmentList.querySelectorAll('.segment-row').forEach((row) => {
      const active = Number(row.dataset.index) === selectedIndex;
      row.classList.toggle('is-selected', active);
      if (active) row.setAttribute('aria-current', 'true');
      else row.removeAttribute('aria-current');
    });
  }

  function selectSegment(index, shouldSeek = false) {
    selectedIndex = Math.max(0, Math.min(episode.segments.length - 1, index));
    renderDetail();
    if (shouldSeek) seek(episode.segments[selectedIndex].start);
    const row = segmentList.querySelector(`.segment-row[data-index="${selectedIndex}"]`);
    if (row) row.scrollIntoView({ block: 'nearest' });
  }

  function selectAtTime(time) {
    const current = episode.segments[selectedIndex];
    if (current && time >= current.start && time < current.end) return;
    const index = episode.segments.findIndex((segment) => time >= segment.start && time < segment.end);
    if (index >= 0) selectSegment(index);
  }

  function showEpisode(id, initialIndex = 0) {
    episode = byId.get(id) || episodes[0];
    if (!episode) return;
    episodeSelect.value = episode.id;
    $('#episode-id').textContent = episode.id;
    $('#episode-task').textContent = episode.task || episode.sourceId;
    $('#episode-duration').textContent = formatTime(episode.duration);
    search.value = '';
    video.pause();
    video.src = episode.videoUrl;
    video.load();
    pendingSeek = null;
    selectedIndex = Math.max(0, Math.min(episode.segments.length - 1, initialIndex));
    renderBoundaries();
    renderSegments();
    renderDetail();
    updateTime(0);
    const url = new URL(location.href);
    url.searchParams.set('episode', episode.id);
    history.replaceState(null, '', url);
  }

  const themeButton = $('#theme-toggle');
  themeButton.addEventListener('click', () => {
    const theme = document.documentElement.dataset.theme === 'light' ? 'dark' : 'light';
    document.documentElement.dataset.theme = theme;
    try { localStorage.setItem('shotwell-sample-theme', theme); } catch (e) {}
    themeButton.textContent = theme === 'light' ? '\u263e' : '\u263c';
  });
  themeButton.textContent = document.documentElement.dataset.theme === 'light' ? '\u263e' : '\u263c';

  if (!episodes.length) {
    $('#collection-summary').textContent = 'No complete reference episodes are available yet.';
    return;
  }
  const totalSegments = episodes.reduce((sum, item) => sum + item.segments.length, 0);
  $('#collection-summary').textContent = `${episodes.length} reviewed episodes · ${totalSegments} acceptable segments`;
  episodes.forEach((item) => {
    const option = make('option', '', `${item.sourceId} · ${item.task || item.id}`);
    option.value = item.id;
    episodeSelect.append(option);
  });
  const selectedFromUrl = new URLSearchParams(location.search).get('episode');
  showEpisode(selectedFromUrl);
  episodeSelect.addEventListener('change', () => showEpisode(episodeSelect.value));
  search.addEventListener('input', renderSegments);
  $('#previous-segment').addEventListener('click', () => selectSegment(selectedIndex - 1, true));
  $('#next-segment').addEventListener('click', () => selectSegment(selectedIndex + 1, true));
  track.addEventListener('click', (event) => {
    const rect = track.getBoundingClientRect();
    const time = Math.max(0, Math.min(episode.duration, (event.clientX - rect.left) / rect.width * episode.duration));
    seek(time);
    selectAtTime(time);
  });
  video.addEventListener('loadedmetadata', () => {
    if (pendingSeek !== null) {
      video.currentTime = pendingSeek;
      pendingSeek = null;
    }
  });
  video.addEventListener('timeupdate', () => {
    updateTime();
    selectAtTime(video.currentTime);
  });
})();
