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

function apply(t, persist) {
  document.documentElement.setAttribute('data-theme', t);
  document.querySelectorAll('#themeToggle, [data-theme-toggle]').forEach(function (btn) {
    btn.textContent = t === 'light' ? '\u263E' : '\u2600';
    btn.setAttribute('aria-label', t === 'light' ? 'Switch to dark mode' : 'Switch to light mode');
  });
  /* only an explicit choice pins the theme; otherwise keep following the OS */
  if (persist) { try { localStorage.setItem(KEY, t); } catch (e) {} }
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
      apply(current() === 'light' ? 'dark' : 'light', true);
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

function systemTheme() {
  try {
    return matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  } catch (e) { return 'dark'; }
}

function savedTheme() {
  try { return localStorage.getItem(KEY); } catch (e) { return null; }
}

export function init() {
  apply(savedTheme() || systemTheme());
  /* no explicit choice yet: follow the OS live */
  try {
    matchMedia('(prefers-color-scheme: light)').addEventListener('change', function (e) {
      if (!savedTheme()) apply(e.matches ? 'light' : 'dark');
    });
  } catch (e) {}
  document.querySelectorAll('#themeToggle, [data-theme-toggle]').forEach(function (btn) {
    btn.addEventListener('click', function () {
      apply(current() === 'light' ? 'dark' : 'light', true);
    });
  });
  document.querySelectorAll('[data-theme-longpress]').forEach(wireLongPress);
}
