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
  if (btn) {
    btn.textContent = t === 'light' ? '\u263E' : '\u2600';
    btn.setAttribute('aria-label', t === 'light' ? 'Switch to dark mode' : 'Switch to light mode');
  }
  try { localStorage.setItem(KEY, t); } catch (e) {}
  dispatchEvent(new CustomEvent('themechange', { detail: t }));
}

/* long-press (600ms) on any [data-theme-longpress] element flips the theme
   and swallows the click so links don't navigate */
function wireLongPress(el) {
  var timer = null, fired = false;
  el.addEventListener('pointerdown', function () {
    fired = false;
    timer = setTimeout(function () {
      fired = true;
      apply(current() === 'light' ? 'dark' : 'light');
    }, 600);
  });
  ['pointerup', 'pointerleave', 'pointercancel'].forEach(function (ev) {
    el.addEventListener(ev, function () { clearTimeout(timer); });
  });
  el.addEventListener('click', function (e) {
    if (fired) { e.preventDefault(); fired = false; }
  });
  el.addEventListener('contextmenu', function (e) {
    if (fired) e.preventDefault();
  });
}

export function init() {
  var saved;
  try { saved = localStorage.getItem(KEY); } catch (e) {}
  apply(saved || 'dark');
  var btn = document.getElementById('themeToggle');
  if (btn) btn.addEventListener('click', function () {
    apply(current() === 'light' ? 'dark' : 'light');
  });
  document.querySelectorAll('[data-theme-longpress]').forEach(wireLongPress);
}
