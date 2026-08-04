/* Theme toggle shared by the sample pages.
   Flipping the theme inverts the halftone polarity as well as the palette —
   on a light ground, dark pixels become the big dots. Pages listen for
   `themechange` and repaint their canvases. */

var KEY = 'shotwell-sample-theme';

export function current() {
  return document.documentElement.getAttribute('data-theme') || 'dark';
}

export function isLight() {
  return current() === 'light';
}

/* Dot colour and polarity for the active theme. */
export function dotStyle() {
  return isLight()
    ? { dot: '#22222A', invert: false }
    : { dot: '#F5F2EB', invert: true };
}

function apply(t) {
  document.documentElement.setAttribute('data-theme', t);
  var btn = document.getElementById('themeToggle');
  if (btn) btn.textContent = t === 'light' ? 'Dark' : 'Light';
  try { localStorage.setItem(KEY, t); } catch (e) {}
  dispatchEvent(new CustomEvent('themechange', { detail: t }));
}

export function init() {
  var saved;
  try { saved = localStorage.getItem(KEY); } catch (e) {}
  apply(saved || 'dark');
  var btn = document.getElementById('themeToggle');
  if (btn) btn.addEventListener('click', function () {
    apply(current() === 'light' ? 'dark' : 'light');
  });
}
