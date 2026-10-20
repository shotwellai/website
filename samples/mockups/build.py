"""Build the three homepage mockups (a.html, b.html, c.html) from shared sections.

Run: python3 samples/mockups/build.py   then open http://localhost:8099/samples/mockups/
"""
from pathlib import Path

HERE = Path(__file__).parent

INTRO = "The best understanding of your robotics data. Labels, annotations, and quality checks for robotics labs, deployment companies, and data vendors."

CSS = """
:root{--ground:#F5F2EB;--surface:#EDE9DF;--ink:#22222A;--dim:rgba(34,34,42,.66);--faint:rgba(34,34,42,.42);
  --wire:rgba(34,34,42,.16);--acc:#EFB702;--fail:#AF3261;--pass:#4F8A6E;--r:0px;
  --display:'Fraunces',Georgia,serif;--body:'Outfit',system-ui,sans-serif;--mono:'IBM Plex Mono',ui-monospace,Menlo,monospace}
[data-theme="dark"]{--ground:#121216;--surface:#1B1B21;--ink:#F2EFE8;--dim:rgba(242,239,232,.66);--faint:rgba(242,239,232,.42);
  --wire:rgba(242,239,232,.14);--pass:#7FC4A0;--fail:#E2577F;color-scheme:dark}
*{margin:0;padding:0;box-sizing:border-box}
.logo-dark{display:none}
[data-theme="dark"] .logo-dark{display:block}[data-theme="dark"] .logo-light{display:none}
.theme{background:none;border:1px solid var(--wire);color:var(--dim);width:34px;height:34px;border-radius:50%;cursor:pointer;font-size:.95rem;line-height:1}
.theme:hover{color:var(--ink);border-color:var(--dim)}
html{scroll-behavior:smooth}
body{background:var(--ground);color:var(--ink);font-family:var(--body);line-height:1.6;-webkit-font-smoothing:antialiased}
a{color:inherit}
img,canvas{display:block}
.wrap{max-width:1200px;margin:0 auto;padding:0 40px}
.tag{font-family:var(--mono);font-size:.6875rem;letter-spacing:.18em;text-transform:uppercase;color:var(--dim)}
h1{font-family:var(--display);font-weight:300;letter-spacing:-.008em;line-height:1.04}
h2{font-family:var(--display);font-weight:300;font-size:clamp(2rem,3.6vw,2.9rem);line-height:1.08;letter-spacing:-.015em}
h3{font-family:var(--body);font-weight:500;font-size:1.0625rem;margin:14px 0 6px}
p{color:var(--dim)}
.btn{display:inline-flex;align-items:center;gap:8px;padding:12px 20px;font-size:.9375rem;font-weight:500;text-decoration:none;border:1px solid var(--ink);border-radius:var(--r)}
.btn.primary{background:var(--acc);border-color:var(--acc);color:#111}
.btn.ghost{background:transparent}
.btn:hover{filter:brightness(1.05)}
.textlink{align-self:center;color:var(--dim);text-underline-offset:5px;font-size:.9375rem;margin-left:8px}
.textlink:hover{color:var(--ink)}

/* announcement + nav */
.announce{display:flex;justify-content:center;align-items:center;gap:12px;background:#EFB702;color:#0C0C0E;font-size:.8125rem;font-weight:500;padding:9px 20px;text-decoration:none;
  white-space:nowrap;overflow:hidden}
.announce .a-tag{font-family:var(--mono);font-size:.5625rem;letter-spacing:.18em;text-transform:uppercase;background:#4A4A52;color:#F5F2EB;padding:3px 9px;flex:none}
.announce b{font-weight:600}
.announce u{text-underline-offset:3px}
.announce:hover{filter:brightness(1.05)}
/* top bar: copied from the live site */
.brandbar{position:sticky;top:0;z-index:40;display:flex;align-items:center;justify-content:space-between;gap:20px;padding:14px 40px;
  background:color-mix(in srgb,var(--ground) 90%,transparent);backdrop-filter:blur(10px)}
.brandbar img{height:24px;width:auto}
.navlinks{display:flex;gap:26px;align-items:center}
.navlinks a{font-size:.875rem;color:var(--dim);text-decoration:none}
.navlinks .tbtn{background:none;border:0;padding:4px 8px;cursor:pointer;color:var(--dim);font-size:1.05rem;line-height:1;width:auto;height:auto}
.navlinks .tbtn:hover{color:var(--ink)}
.navlinks a:not(.cta):hover{color:var(--ink)}
.navlinks .cta:hover{color:#0C0C0E;filter:brightness(1.06)}
.navlinks a.cta,.cta{background:var(--acc);color:#0C0C0E;padding:9px 17px;font-size:.8125rem;font-weight:500;text-decoration:none}
.navlinks .ghostcta{border:1px solid var(--wire-strong,rgba(34,34,42,.38));padding:8px 15px;color:var(--ink);font-weight:500}
[data-theme="dark"] .navlinks .ghostcta{border-color:rgba(242,239,232,.44)}
.navlinks .ghostcta:hover{border-color:var(--acc)}
.cta small{display:none;font-size:.75em;font-weight:400;opacity:.75}
@media (min-width:1100px){.cta small{display:inline}}

/* sections */
section{padding:110px 0}
.head{display:grid;grid-template-columns:1fr 1fr;gap:40px;align-items:end;margin-bottom:48px}
.head p{font-size:1.0625rem;max-width:46ch}
.deliver .head{grid-template-columns:1fr}
.deliver h2{max-width:none;white-space:nowrap}

/* trusted by */
.trusted{padding:110px 0 84px;text-align:center}
.trusted .t-lbl,.trusted .t-more{font-family:var(--mono);font-size:.875rem;letter-spacing:.26em;text-transform:uppercase;color:var(--dim)}
.trusted .t-lbl{margin-bottom:38px}
.trusted .t-more{margin-top:42px}
.logos{display:flex;align-items:center;justify-content:center;gap:64px;flex-wrap:wrap}
.logos img{height:30px;width:auto;mix-blend-mode:multiply;opacity:.92}
.logos img.tall{height:42px}.logos img.tile{height:38px}
.logos .trace{display:inline-flex;align-items:center;gap:8px}
.logos .trace img{height:30px}
.logos .trace span{font-family:'Manrope',var(--body);font-size:1.75rem;font-weight:600;letter-spacing:-.055em;line-height:1;color:var(--ink);opacity:.8}
[data-theme="dark"] .quote .who img{filter:invert(1) hue-rotate(180deg);mix-blend-mode:screen;opacity:.9}
[data-theme="dark"] .logos img{filter:invert(1) hue-rotate(180deg);mix-blend-mode:screen;opacity:.9}

/* annotated episode */
.episode .frame{position:relative;background:var(--ground);border:1px solid var(--wire);border-radius:var(--r);overflow:hidden}
.episode canvas{width:100%;aspect-ratio:1278/720}
.episode .chip{position:absolute;font-family:var(--mono);font-size:.625rem;letter-spacing:.14em;text-transform:uppercase;background:var(--ground);
  border:1px solid var(--wire);padding:4px 8px;color:var(--dim)}
.episode .chip.tl{top:14px;left:14px}.episode .chip.tr{top:14px;right:14px;color:var(--ink)}
.strip{display:flex;gap:2px;margin-top:10px}
.strip button{position:relative;height:34px;border:0;background:var(--surface);font-family:var(--mono);font-size:.5625rem;letter-spacing:.08em;
  text-transform:uppercase;color:var(--dim);text-align:left;padding:0 8px;overflow:hidden;white-space:nowrap;min-width:0;cursor:pointer}
.strip button.on{background:var(--ink);color:var(--ground)}
.strip button.fail.on{background:var(--fail)}
.strip button i{position:absolute;left:0;bottom:0;height:2px;background:var(--acc)}
.readout{display:grid;grid-template-columns:repeat(4,1fr);border:1px solid var(--wire);border-top:0;border-radius:0 0 var(--r) var(--r)}
.readout div{padding:14px 16px;border-right:1px solid var(--wire)}
.readout div:last-child{border-right:0}
.readout b{display:block;font-family:var(--mono);font-weight:500;font-size:.8125rem;margin-top:4px;color:var(--ink)}
.v-pass{color:var(--pass)!important}.v-fail{color:var(--fail)!important}

/* audiences */
.three{display:grid;grid-template-columns:repeat(3,1fr);border-top:1px solid var(--ink)}
.three > div{padding:26px 28px 8px 0}
.three > div + div{padding-left:28px;border-left:1px solid var(--wire)}
.three h3{font-family:var(--display);font-weight:400;font-size:1.6rem;margin:10px 0 10px}

/* deliverables */
.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:20px}
.card{background:var(--surface);border:1px solid var(--wire);border-radius:var(--r);overflow:hidden}
.card canvas{width:100%;aspect-ratio:3/2;background:var(--ground);border-bottom:1px solid var(--wire)}
.card .sample{aspect-ratio:3/2;background:var(--ground);border-bottom:1px solid var(--wire);padding:26px 28px;font-family:var(--mono);font-size:.8125rem;
  line-height:1.75;color:var(--ink);overflow:hidden}
.card .sample i{font-style:normal;color:var(--faint)}
.card .body{padding:18px 24px 24px}
.card .body p{font-size:.9375rem}

/* data */
.data{display:grid;grid-template-columns:repeat(3,1fr);gap:0;border:1px solid var(--wire);border-radius:var(--r)}
.data > div{padding:26px 26px 28px}
.data > div + div{border-left:1px solid var(--wire)}
.data ul{list-style:none;margin-top:12px}
.data li{font-family:var(--mono);font-size:.8125rem;padding:6px 0;border-top:1px solid var(--wire);color:var(--dim)}
.out{margin-top:20px;font-size:.9375rem}

/* data types: drawings in a row */
.runner{margin:0 -40px}
.runner canvas{display:block;width:100%;height:290px}
.carousel{overflow:hidden;margin:0 -40px;padding:6px 0;
  -webkit-mask:linear-gradient(90deg,transparent,#000 7%,#000 93%,transparent);mask:linear-gradient(90deg,transparent,#000 7%,#000 93%,transparent)}
.track{display:flex;gap:22px;width:max-content;animation:follow 36s linear infinite}
.carousel:hover .track{animation-play-state:paused}
@keyframes follow{to{transform:translateX(calc(-50% - 11px))}}
@media (prefers-reduced-motion:reduce){.track{animation:none;flex-wrap:wrap;width:auto}}
.kind{width:300px;flex:none}
.kind canvas{display:block;width:100%;aspect-ratio:3/2;border-bottom:1px solid var(--wire)}
.loop{position:relative;aspect-ratio:3/2;border-bottom:1px solid var(--wire)}
.loop canvas{position:absolute;inset:0;width:100%;height:100%;transition:opacity 1.2s;border:0!important}
.loop canvas.off{opacity:0}
.loop .chip.off{opacity:0}
.loop .chip{position:absolute;left:12px;bottom:10px;font-family:var(--mono);font-size:.625rem;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);
  background:var(--ground);border:1px solid var(--wire);padding:3px 7px;transition:opacity 1.2s}
.kind{background:var(--surface);border:1px solid var(--wire);border-radius:var(--r);overflow:hidden}
.kind svg{display:block;width:100%;height:auto;background:var(--ground);border-bottom:1px solid var(--wire);padding:10px 8px}
.kind .ln{fill:none;stroke:var(--ink);stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.kind .ink{fill:var(--ink)}.kind .acc{fill:var(--acc);stroke:var(--ink)}.kind .joint{fill:var(--ground)}
.kind .dash{stroke-dasharray:3 5;stroke:var(--dim)}
.kind figcaption{padding:16px 18px 20px}
.kind h3{margin:0 0 6px}
.kind p{font-size:.875rem}
/* quote */
.quote{text-align:center;padding-top:40px}
.quote blockquote{font-family:var(--display);font-weight:300;font-size:clamp(1.7rem,3.4vw,2.8rem);line-height:1.14;max-width:1020px;margin:0 auto;letter-spacing:-.015em}
.quote blockquote b{font-weight:400;color:var(--acc)}
.quote .who{display:flex;align-items:center;justify-content:center;gap:16px;margin-top:34px;font-size:.9375rem;color:var(--dim);flex-wrap:wrap}
.quote .who img{height:20px;mix-blend-mode:multiply;opacity:.92}
.quote .who a.case{font-family:var(--mono);font-size:.6875rem;letter-spacing:.12em;text-transform:uppercase;color:#0C0C0E;background:var(--acc);text-decoration:none;padding:9px 13px}

/* bottom bar: the live site's yellow field */
.field{background:var(--acc);color:#111;padding:88px 0 40px}
.field .wrap{display:flex;align-items:center;justify-content:space-between;gap:40px}
.field h2{font-size:clamp(1.9rem,4vw,3rem);max-width:24ch;color:#111}
.field h2 em{font-style:normal}
.field a{color:inherit;text-decoration:underline;text-underline-offset:7px;text-decoration-thickness:1px}
.field .f-mark{width:84px;height:auto;flex:none}
.field footer{margin-top:72px;width:100%;display:flex;gap:28px;flex-wrap:wrap;font-family:var(--mono);font-size:.6875rem;color:rgba(17,17,17,.62)}

@media (max-width:860px){
  .wrap{padding:0 20px}
  .field .f-mark{width:56px}
  .field footer{row-gap:6px;margin-top:48px}
  .quote .who img{height:15px}
  .brandbar{padding:12px 16px;flex-wrap:wrap}
  .navlinks{gap:14px}.navlinks a{font-size:.8125rem}.cta{padding:8px 12px}
  .announce{justify-content:flex-start}
  .announce .a-tag{display:none}
  .announce span{overflow:hidden;text-overflow:ellipsis}
  section{padding:72px 0}
  .head{grid-template-columns:1fr;gap:16px;margin-bottom:32px}
  .deliver h2{white-space:normal}
  .three,.cards,.data{grid-template-columns:1fr}
  .runner{margin:0 -20px}
  .runner canvas{height:230px}
  .three > div + div{padding-left:0;border-left:0;border-top:1px solid var(--wire)}
  .data > div + div{border-left:0;border-top:1px solid var(--wire)}
  .readout{grid-template-columns:1fr 1fr}
  .readout div:nth-child(2){border-right:0}
  .readout div:nth-child(-n+2){border-bottom:1px solid var(--wire)}
  .strip button span{display:none}
  .logos{gap:30px}
  .logos img{height:20px}.logos img.tall{height:28px}.logos img.tile{height:26px}
}
"""

HEAD = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shotwell — annotations for robotics data (mockup {name})</title>
<meta name="description" content="{intro}">
<link rel="icon" href="/favicon.svg">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Fraunces:opsz,wght@9..144,300;9..144,400&family=Manrope:wght@600&family=Outfit:wght@300;400;500&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
<script>document.documentElement.dataset.theme=new URLSearchParams(location.search).get("theme")||localStorage.getItem("mock-theme")||"light";</script>
<style>{css}{extra}</style></head><body class="{body}">
<a class="announce" href="/ultra-case-study/"><span class="a-tag">Case study</span><span>Ultra supports <b>6× more operators</b> per engineer with Shotwell. <u>Read how →</u></span></a>
"""

NAV = """<div class="brandbar">
  <a href="#"><img class="logo-light" src="/brand/assets/SVG/shotwell-logo-horizontal-black.svg" alt="Shotwell"><img class="logo-dark" src="/brand/assets/SVG/shotwell-logo-horizontal-white.svg" alt="Shotwell"></a>
  <div class="navlinks">
    <button class="theme tbtn" type="button" aria-label="Toggle theme">☾</button>
    <a href="/ultra-case-study/">Blog</a>
    <a href="https://demo.shotwell.ai/">Demo</a>
    <a class="ghostcta" href="#contact">Talk to us</a>
    <a class="cta" href="https://app.shotwell.ai/">Upload videos now <small>· 24 hr turnaround</small></a>
  </div>
</div>
"""

TRUSTED = """<div class="trusted"><div class="wrap">
  <div class="t-lbl">Trusted by</div>
  <div class="logos">
    <img src="/weave-logo.png" alt="Weave Robotics"><img src="/ultra-logo.svg" alt="Ultra">
    <img class="tall" src="/frontier-logo.svg" alt="Frontier Machines"><img class="tile" src="/parametric-logo.png" alt="Parametric">
  </div>
  <div class="t-more">And more</div>
</div></div>
"""

EPISODE = """<section class="episode" id="episode"><div class="wrap">
  <div class="head"><div><h2>Every step of every episode, named and checked</h2></div>
    <p>A real fold from our pipeline. The letters spell the subtask at each moment. Red marks a step that failed its rubric. Click a step to jump to it.</p></div>
  <div class="frame"><canvas id="ep-art" aria-label="The fold_clothing episode drawn with the letters of its current subtask"></canvas>
    <span class="chip tl">Episode 0431 · fold_clothing</span><span class="chip tr" id="ep-step">—</span></div>
  <div class="strip" id="ep-strip"></div>
  <div class="readout">
    <div><span class="tag">Subtask</span><b id="ro-step">—</b></div>
    <div><span class="tag">Time</span><b id="ro-time">—</b></div>
    <div><span class="tag">Rubric</span><b id="ro-rule">—</b></div>
    <div><span class="tag">Verdict</span><b id="ro-verdict">—</b></div>
  </div>
</div></section>
"""

QUOTE = """<section class="quote"><div class="wrap">
  <blockquote>“Shotwell annotations improved our training data visibility to allow every QA reviewer to review <b>6× more operators</b>.”</blockquote>
  <div class="who"><a class="case" href="/ultra-case-study/">Read case study →</a></div>
</div></section>
"""

REST = """<section class="deliver"><div class="wrap">
  <div class="head"><div><h2>Labels you can train on and query</h2></div></div>
  <div class="cards">
    <div class="card"><canvas data-art="strata" data-rows="22" aria-hidden="true"></canvas><div class="body"><h3>Subtask segmentation</h3><p>Sub-second boundaries for every step, checked against your SOP.</p></div></div>
    <div class="card"><div class="sample" aria-hidden="true"><i>[0:07 – 0:10]</i> Right gripper lifts the right sleeve and folds it across the centre line.<br><i>[0:10 – 0:15]</i> Left gripper folds the left sleeve; the cuff does not tuck in.<br><i>[0:15 – 0:16]</i> Both grippers fold the bottom third up.</div>
      <div class="body"><h3>Dense captions</h3><p>A plain description of what happens in each step, in the words your models use.</p></div></div>
    <div class="card"><div class="loop"><canvas data-art="envelope" aria-hidden="true"></canvas><canvas data-art="contours" aria-hidden="true"></canvas>
      <span class="chip a">Failed paths</span><span class="chip b">Out of distribution</span></div><div class="body"><h3>Failure detection and quality checks</h3><p>Rubric verdicts for every step. Find out-of-distribution data to understand what goes into your training.</p></div></div>
  </div>
</div></section>

<section class="takes"><div class="wrap">
  <div class="head"><div><h2>All forms of data</h2></div>
    <p>Egocentric, teleoperation, UMI, or deployment footage. Send us MP4, MCAP, or LeRobot, and get labels back as JSON or CSV within 24 hours.</p></div>
  <div class="runner"><canvas data-art="runner" aria-label="Four ways robot data is collected, passing along one line: VR teleoperation, leader and follower arms, UMI handheld grippers, and egocentric video."></canvas></div>
</div></section>



<section class="field" id="contact"><div class="wrap">
  <h2>Let's talk about your <em>data:</em> <a href="mailto:team@shotwell.ai">team@shotwell.ai</a></h2>
  <img class="f-mark" src="/brand/assets/SVG/shotwell-mark-black.svg" alt="Shotwell">
</div><div class="wrap"><footer><span>Shotwell — annotations for robotics data</span><span>team@shotwell.ai</span></footer></div></section>
"""

SCRIPT = """<script type="module">
import { mount, strip, getClip, currentSegment } from './art.js';
const RULES = {
  'Pick up': [['One arm pick up', 'pass']], 'Straighten': [['No slipping', 'fail'], ['Shirt orientation', 'pass']],
  'Fold right sleeve': [['Right sleeve first', 'pass']], 'Fold left sleeve': [['Sleeve tucked in', 'fail']],
  'Stack': [['One arm stack', 'pass']] };
const $ = (id) => document.getElementById(id);
const fmt = (s) => `0:${String(Math.floor(s)).padStart(2, '0')}`;
const gb = document.body.classList.contains('board');
if (gb) document.fonts.load('8px "Pokemon GB"');
mount($('ep-art'), 'labels', { palette: gb ? 'frost' : undefined, font: gb ? '"Pokemon GB", monospace' : undefined, fontScale: 1.0, onFrame(vid, seg) {
  $('ep-step').textContent = seg.label; $('ro-step').textContent = seg.label;
  $('ro-time').textContent = `${fmt(seg.start)} – ${fmt(seg.end)}`;
  const rules = RULES[seg.label] || [];
  $('ro-rule').textContent = rules.length ? rules.map(r => r[0]).join(' · ') : 'No rule for this step';
  const v = $('ro-verdict'); v.textContent = seg.fail ? 'FAIL' : 'PASS'; v.className = seg.fail ? 'v-fail' : 'v-pass';
  dispatchEvent(new CustomEvent('segment', { detail: seg }));
}});
strip($('ep-strip'));
const onBoard = document.body.classList.contains('board');
const FX = { envelope: 'dither', contours: 'dither' };
document.querySelectorAll('canvas[data-art]').forEach((c, i) => mount(c, c.dataset.art, { rows: +c.dataset.rows || undefined, seed: 11 + i,
  palette: onBoard ? 'frost' : undefined, fx: onBoard ? FX[c.dataset.art] : undefined }));
import { loopPair } from './art.js';
document.querySelectorAll('.loop').forEach(l => {
  const [a, b] = l.querySelectorAll('canvas'), [ca, cb] = l.querySelectorAll('.chip');
  loopPair(a, b); loopPair(ca, cb);
});
/* hero: size the headline to the full content width, so it lines up with the video's edges */
const anchor = document.querySelector('.hero .anchor'), heroGrid = document.querySelector('.hero .grid'), heroH1 = document.querySelector('.hero h1');
const alignHero = () => {
  if (!heroH1 || !heroGrid) return;
  heroH1.style.fontSize = '';
  if (innerWidth > 860) {
    const want = heroGrid.getBoundingClientRect().width, r = document.createRange(); r.selectNodeContents(heroH1);
    for (let i = 0; i < 2; i++) {                                  // two passes: glyph side bearings do not scale exactly
      const have = r.getBoundingClientRect().width, size = parseFloat(getComputedStyle(heroH1).fontSize);
      heroH1.style.fontSize = (size * want / have).toFixed(2) + 'px';
    }
  }
  heroGrid.style.setProperty('--col', '1.3fr');
};
alignHero(); addEventListener('resize', alignHero); document.fonts.ready.then(alignHero);
const btn = document.querySelector('.theme');
const paint = () => { btn.textContent = document.documentElement.dataset.theme === 'dark' ? '☀' : '☾'; };
paint();
btn.addEventListener('click', () => {
  const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
  document.documentElement.dataset.theme = next; localStorage.setItem('mock-theme', next);
  paint(); dispatchEvent(new Event('themechange'));
});
</script>
</body></html>"""

# ── A · Labels: a quiet type-led hero; the labelled episode is the picture ──
A_CSS = """
.hero{padding:96px 0 40px}
.hero h1{font-size:clamp(2.4rem,7.4vw,6.4rem);white-space:nowrap}
/* the headline spans the content width; text starts at the left edge, buttons end at the right edge */
.hero .grid{display:grid;grid-template-columns:var(--col, 1.4fr) 1fr;column-gap:0;align-items:center;margin-top:36px}
.hero .grid p{max-width:52ch;padding-right:40px}
.hero p{font-size:1.125rem;line-height:1.6;color:var(--ink);opacity:.78}
.hero .ctas{display:flex;gap:18px;flex-wrap:wrap;align-items:center;justify-content:flex-end}
.hero + .episode{padding-top:40px}
.hero + .episode .head{display:none}
@media (max-width:860px){.hero{padding:56px 0 8px}.hero h1{white-space:normal}.hero .grid{grid-template-columns:1fr;gap:22px;margin-top:20px}.hero .ctas{justify-content:flex-start}}
"""
A_HERO = f"""<header class="hero"><div class="wrap">
  <h1>Annotations for <span class="anchor">robotics</span></h1>
  <div class="grid"><p>{INTRO}</p><div class="ctas"><a class="btn primary" href="https://app.shotwell.ai/">Upload videos · 24 h</a><a class="textlink" href="#contact">or talk to us</a></div></div>
</div></header>
"""

# ── B · Panels: an xDOF-style grid of rounded panels ──
B_CSS = """
:root{--r:12px}
body.panels{background:#EFEBE2}
.bento{max-width:1280px;margin:0 auto;padding:20px 20px 0;display:grid;grid-template-columns:1.6fr 1fr;grid-template-rows:auto auto;gap:14px}
.panel{background:var(--ground);border:1px solid var(--wire);border-radius:var(--r);position:relative;overflow:hidden}
.panel.art{grid-row:span 2;min-height:560px}
.panel.art canvas{position:absolute;inset:0;width:100%;height:100%}
.panel.art .chip{position:absolute;top:16px;left:16px;font-family:var(--mono);font-size:.625rem;letter-spacing:.14em;text-transform:uppercase;
  background:var(--ground);border:1px solid var(--wire);border-radius:6px;padding:5px 9px;color:var(--dim)}
.panel.art .legend{position:absolute;left:16px;right:16px;bottom:14px;display:flex;justify-content:space-between;font-family:var(--mono);font-size:.625rem;
  letter-spacing:.12em;text-transform:uppercase;color:var(--dim)}
.panel.text{padding:34px 32px 30px;display:flex;flex-direction:column;justify-content:space-between;gap:24px}
.panel.text h1{font-size:clamp(2.4rem,4.2vw,3.6rem)}
.panel.text p{font-size:1rem;line-height:1.6;color:var(--ink);opacity:.78;margin-top:16px}
.panel.text .ctas{display:flex;gap:10px;flex-wrap:wrap}
.minis{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.mini{padding:20px 22px;background:var(--ground);border:1px solid var(--wire);border-radius:var(--r)}
.mini .big{font-family:var(--display);font-weight:300;font-size:2.6rem;line-height:1;margin:4px 0 6px}
.mini p{font-size:.875rem}
.panels section .wrap,.panels .trusted .wrap{max-width:1280px;padding:0 20px}
.panels .trusted{border:0;padding:44px 0}
.panels .three{border-top:0;gap:14px}
.panels .three > div{background:var(--ground);border:1px solid var(--wire)!important;border-radius:var(--r);padding:24px!important}
.panels .data{gap:14px;border:0}
.panels .data > div{background:var(--ground);border:1px solid var(--wire)!important;border-radius:var(--r)}
.panels .card{background:var(--ground)}
.panels .quote{border:0}
.panels .quote .wrap{background:var(--ground);border:1px solid var(--wire);border-radius:var(--r);padding:56px 48px}
.panels .strip button:first-child{border-radius:var(--r) 0 0 0}
@media (max-width:860px){.bento{grid-template-columns:1fr}.panel.art{min-height:360px;grid-row:auto}.panels .quote .wrap{padding:32px 22px}}
"""
B_HERO = f"""<div class="bento">
  <div class="panel art"><canvas data-art="strata" data-rows="44" aria-hidden="true"></canvas>
    <span class="chip">46 episodes · 8 subtasks · fold_clothing</span>
    <div class="legend"><span>Pick up</span><span>Straighten</span><span>Folds</span><span>Stack · Home</span></div></div>
  <div class="panel text"><div><h1>Know what is in every episode</h1><p>{INTRO}</p></div>
    <div class="ctas"><a class="btn primary" href="https://app.shotwell.ai/">Upload videos</a><a class="btn ghost" href="#contact">Talk to us</a></div></div>
  <div class="minis">
    <div class="mini"><div class="big">24 h</div><p>From upload to reviewed labels.</p></div>
    <div class="mini"><div class="big">6×</div><p>More operators per QA reviewer.</p></div>
  </div>
</div>
"""

# ── C · Dark plate: a Pantheon-style full-screen art hero, then the light page ──
C_CSS = """
.plate{position:relative;height:100svh;min-height:560px;background:var(--ground);color:var(--ink);overflow:hidden}
.plate canvas{position:absolute;left:0;right:0;top:0;width:100%;height:78%}
/*old*/.plate .brandbar{position:absolute;left:0;right:0;top:0;background:transparent;border:0;backdrop-filter:none;z-index:3}
.plate::after{content:'';position:absolute;left:0;right:0;bottom:0;height:62%;background:linear-gradient(to top,var(--ground) 55%,transparent);pointer-events:none}
.plate .bottom{position:absolute;left:0;right:0;bottom:0;padding:0 0 44px;z-index:2}
.plate .bottom .wrap{display:flex;justify-content:space-between;align-items:end;gap:40px}
.plate .text{max-width:640px;position:relative}
.plate h1{font-size:clamp(2.6rem,5.4vw,4.8rem)}
.plate p{margin-top:16px;font-size:1.0625rem}
.plate .ctas{display:flex;gap:12px;margin-top:24px;flex-wrap:wrap}
.plate .latest{display:flex;align-items:center;gap:12px;font-size:.875rem;color:var(--dim);text-decoration:none;white-space:nowrap}
.plate .latest span{font-family:var(--mono);font-size:.625rem;letter-spacing:.14em;border:1px solid var(--wire);padding:3px 7px}
.plate .latest:hover{color:var(--ink)}
@media (max-width:860px){.plate{height:auto;min-height:0;padding-top:64px}.plate canvas{position:relative;height:46vh}
  .plate .bottom{position:relative;padding:0 0 36px}.plate::after{height:30%}.plate .bottom .wrap{flex-direction:column;align-items:start;gap:20px}.plate .text{padding:12px 0 0}}
"""
C_HERO = f"""<header class="plate"><canvas data-art="shapes" aria-hidden="true"></canvas>
  {NAV}
  <div class="bottom"><div class="wrap">
    <div class="text"><h1>Annotations for robotics data</h1><p>{INTRO}</p>
      <div class="ctas"><a class="btn primary" href="https://app.shotwell.ai/">Upload videos · 24 h</a><a class="textlink" href="#contact">or talk to us</a></div></div>
    <a class="latest" href="/ultra-case-study/"><span>LATEST</span>How Ultra reviews 6× more operators →</a>
  </div></div>
</header>
"""


# ── D · Atomic purple: the whole page is see-through plastic; content sits in screens ──
D_CSS = """
body.atomic{--plA:#C7B4E6;--plB:#AA92D6;--plC:#8F74C2;--navy:#24203F;--rubber:#4A4760;--bezel:#2F2C3D;--emboss:rgba(255,255,255,.55);
  --ground:transparent;--ink:var(--navy);--dim:rgba(36,32,63,.78);--faint:rgba(36,32,63,.5);--wire:rgba(60,36,110,.22);--r:16px;
  position:relative;color:var(--navy);
  background:radial-gradient(1400px 900px at 20% 0%,var(--plA),transparent 70%),radial-gradient(1200px 1000px at 90% 60%,var(--plB),transparent 70%),var(--plC)}
body.atomic[data-plastic="teal"]{--plA:#A9DCDA;--plB:#7FC3C4;--plC:#5AA8AE}
body.atomic[data-plastic="berry"]{--plA:#F0A9C6;--plB:#DE7FA8;--plC:#C45E8C}
body.atomic[data-plastic="clear"]{--plA:#E8EAEE;--plB:#CDD2DA;--plC:#B4BAC6}
/* the parts inside, then gloss and a milky grain on top of them */
#inside{position:absolute;left:0;top:0;width:100%;pointer-events:none;z-index:0}
body.atomic::after{content:'';position:fixed;inset:0;pointer-events:none;z-index:1;
  background:linear-gradient(112deg,rgba(255,255,255,.22) 0 12%,transparent 26%,transparent 62%,rgba(255,255,255,.12) 72%,transparent 84%),
  url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='160' height='160'><filter id='n'><feTurbulence baseFrequency='.9' numOctaves='2'/><feColorMatrix values='0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 .07 0'/></filter><rect width='100%' height='100%' filter='url(%23n)'/></svg>")}
body.atomic > *:not(#inside){position:relative;z-index:2}
body.atomic h1,body.atomic h2,body.atomic h3{font-family:var(--body);font-weight:700;letter-spacing:-.025em;color:var(--navy)}
body.atomic p{color:var(--dim)}
body.atomic .theme{display:none}
.px{font-family:'Press Start 2P',monospace}
.molded{color:var(--emboss);text-shadow:0 -1px 0 rgba(60,30,110,.35),0 1px 0 rgba(255,255,255,.35)}
.colour span:nth-child(5n+1){color:#6B3FA0}.colour span:nth-child(5n+2){color:#E0383E}.colour span:nth-child(5n+3){color:#3E9E4A}
.colour span:nth-child(5n+4){color:#3B6FD6}.colour span:nth-child(5n+5){color:#E8B020}

/* nav: printed on the shell */
body.atomic nav{background:transparent;border:0;backdrop-filter:none;position:relative}
body.atomic nav .wrap{height:76px}
.word{font-family:var(--body);font-style:italic;font-weight:800;font-size:1.5rem;letter-spacing:-.03em;color:var(--navy);text-decoration:none}
.word small{font-style:normal;font-weight:700;font-size:.62rem;letter-spacing:.16em;margin-left:8px;vertical-align:4px}
/* rubber */
body.atomic .btn{border:0;border-radius:999px;font-weight:600;background:var(--rubber);color:#F1EEF8;
  box-shadow:inset 0 -3px 0 rgba(0,0,0,.3),inset 0 1px 0 rgba(255,255,255,.18),0 3px 8px rgba(40,20,80,.25)}
body.atomic .btn:active{transform:translateY(1px);box-shadow:inset 0 -1px 0 rgba(0,0,0,.3)}
body.atomic .btn.primary{background:#3A2E68}
body.atomic .textlink{color:var(--navy)}

/* screens: a dark bezel with the light glass inside */
.bezel{background:var(--bezel);border-radius:14px 14px 42px 14px;padding:14px 18px 18px;
  box-shadow:inset 0 2px 0 rgba(255,255,255,.08),inset 0 -2px 6px rgba(0,0,0,.4),0 10px 24px rgba(40,20,80,.25)}
.bezel .rule{display:flex;align-items:center;gap:10px;margin:0 0 10px;font-size:.5rem;color:#B9B4CC;letter-spacing:.08em;font-style:italic;
  font-family:var(--body);font-weight:600;text-transform:uppercase}
.bezel .rule i{flex:1;height:2px;background:linear-gradient(#B03A84 0 50%,#3E4FA8 50%)}
.glass{position:relative;background:#DCDFCB;border-radius:2px;overflow:hidden;box-shadow:inset 0 0 0 2px rgba(0,0,0,.25)}
.glass canvas{display:block;width:100%;image-rendering:pixelated}

/* hero */
.ahero{padding:56px 0 30px}
.ahero .grid{display:grid;grid-template-columns:1fr 1.05fr;gap:60px;align-items:center}
.ahero h1{font-size:clamp(2.8rem,5.6vw,5rem);line-height:.98}
.ahero p{margin-top:22px;font-size:1.0625rem;max-width:44ch}
.ahero .ctas{display:flex;gap:14px;margin-top:28px;align-items:center;flex-wrap:wrap}
.unit{position:relative}
.unit .power{position:absolute;left:-58px;top:46%;display:flex;flex-direction:column;align-items:center;gap:6px;font-size:.45rem;color:var(--navy)}
.unit .power b{width:9px;height:9px;border-radius:50%;background:#FF2E4C;box-shadow:0 0 10px #FF2E4C}
.unit .glass canvas{aspect-ratio:10/9}
.unit .name{margin:16px 0 0;text-align:center}
.unit .name .word{font-size:1.15rem}
.controls{display:flex;justify-content:space-between;align-items:center;margin:22px 20px 0}
.pad{position:relative;width:96px;height:96px}
.pad i{position:absolute;background:var(--rubber);border-radius:6px;box-shadow:inset 0 -3px 0 rgba(0,0,0,.35),0 3px 6px rgba(40,20,80,.25)}
.pad i.h{left:0;top:32px;width:96px;height:32px}.pad i.v{left:32px;top:0;width:32px;height:96px}
.pad button{position:absolute;top:32px;width:32px;height:32px;background:transparent;border:0;cursor:pointer;z-index:2}
.pad button.l{left:0}.pad button.r{right:0}
.pad button::after{content:'';position:absolute;inset:11px;border:5px solid transparent}
.pad button.l::after{border-right-color:rgba(255,255,255,.35);left:2px}.pad button.r::after{border-left-color:rgba(255,255,255,.35);right:2px}
.ab2{display:flex;gap:18px;align-items:flex-end}
.ab2 label{display:flex;flex-direction:column;align-items:center;gap:8px;font-size:.45rem;color:var(--navy)}
.ab2 button{width:54px;height:54px;border-radius:50%;border:0;cursor:pointer;background:var(--rubber);color:#CFCBE0;font:700 .9rem var(--body);
  box-shadow:inset 0 -4px 0 rgba(0,0,0,.35),0 4px 8px rgba(40,20,80,.3)}
.ab2 label:first-child{margin-bottom:-18px}
.ab2 button:active{transform:translateY(2px)}

/* sections printed on the shell */
body.atomic section{padding:96px 0}
body.atomic .trusted{border:0;padding:40px 0 20px}
body.atomic .trusted .tag{color:var(--emboss);text-shadow:0 -1px 0 rgba(60,30,110,.35)}
body.atomic .logos img{filter:brightness(0) invert(1) drop-shadow(0 -1px 0 rgba(60,30,110,.45));opacity:.6;mix-blend-mode:normal}
body.atomic .three{border-top:2px solid var(--wire)}
body.atomic .three > div + div{border-left:2px solid var(--wire)}
body.atomic .cards{gap:28px}
body.atomic .card{background:transparent;border:0;overflow:visible}
body.atomic .card .shot{background:var(--bezel);border-radius:14px 14px 34px 14px;padding:12px 12px 16px;box-shadow:inset 0 -2px 6px rgba(0,0,0,.4),0 10px 24px rgba(40,20,80,.22)}
body.atomic .card canvas,body.atomic .card .sample{border:0;background:#DCDFCB;border-radius:2px}
body.atomic .card .sample{font-family:'Press Start 2P',monospace;font-size:.55rem;line-height:2.1;color:var(--navy);padding:22px}
body.atomic .card .sample i{color:#7A4FC0}
body.atomic .card .body{padding:16px 6px 0}
body.atomic .data{border:0;gap:0}
body.atomic .data > div + div{border-left:2px solid var(--wire)}
body.atomic .data li{border-top:1px solid var(--wire);color:var(--dim)}
body.atomic .quote blockquote{font-family:var(--body);font-weight:700;letter-spacing:-.025em;color:var(--navy)}
body.atomic .quote blockquote b{font-weight:800}
body.atomic .quote .who img{filter:brightness(0) invert(1) drop-shadow(0 -1px 0 rgba(60,30,110,.45));opacity:.65;mix-blend-mode:normal}
/* contact: printed on the shell, with the speaker */
body.atomic .field{background:transparent;color:var(--navy)}
body.atomic .field h2{color:var(--navy)}
body.atomic .field footer{color:var(--dim)}
body.atomic .field .f-mark{display:none}
.speaker{width:150px;height:150px;border-radius:50%;flex:none;position:relative;
  background:radial-gradient(circle,rgba(40,20,80,.55) 0 3.2px,transparent 3.6px) 0 0/15px 15px;
  -webkit-mask:radial-gradient(circle,#000 68%,transparent 70%);mask:radial-gradient(circle,#000 68%,transparent 70%)}
.plastics{display:flex;gap:8px;align-items:center}
.plastics button{width:16px;height:16px;border-radius:50%;border:1.5px solid rgba(255,255,255,.8);cursor:pointer;padding:0}
.plastics button[aria-pressed="true"]{outline:2px solid var(--navy);outline-offset:2px}
@media (max-width:860px){
  .ahero .grid{grid-template-columns:1fr;gap:36px}
  .unit .power{display:none}
  body.atomic .three > div + div,body.atomic .data > div + div{border-left:0;border-top:2px solid var(--wire)}
  .speaker{width:90px;height:90px}
  .plastics{display:none}
}
"""

D_NAV = """<nav><div class="wrap">
  <a class="word" href="#">Shotwell.<small class="colour"><span>D</span><span>A</span><span>T</span><span>A</span></small></a>
  <div class="links"><a href="/ultra-case-study/">Blog</a><a href="https://demo.shotwell.ai/">Demo</a>
    <span class="plastics" role="group" aria-label="Plastic colour">
      <button data-p="purple" style="background:#9F86CF" aria-label="Atomic purple" aria-pressed="true"></button>
      <button data-p="teal" style="background:#7FC3C4" aria-label="Teal" aria-pressed="false"></button>
      <button data-p="berry" style="background:#DE7FA8" aria-label="Berry" aria-pressed="false"></button>
      <button data-p="clear" style="background:#D5D9E0" aria-label="Clear" aria-pressed="false"></button></span>
    <a class="btn primary" href="https://app.shotwell.ai/">Upload videos</a></div>
</div></nav>
"""

D_HERO = f"""<header class="ahero"><div class="wrap grid">
  <div><h1>Annotations for robotics <span class="colour"><span>d</span><span>a</span><span>t</span><span>a</span></span></h1>
    <p>{INTRO}</p>
    <div class="ctas"><a class="btn primary" href="https://app.shotwell.ai/">Upload videos · 24 h</a><a class="textlink" href="#contact">or talk to us</a></div></div>
  <div class="unit">
    <div class="power px"><b></b>POWER</div>
    <div class="bezel"><div class="rule"><i></i>Dot matrix with subtask labels<i></i></div>
      <div class="glass"><canvas id="lcd" aria-label="The fold_clothing episode on a colour dot-matrix screen, with its current subtask"></canvas></div></div>
    <div class="name"><span class="word">Shotwell <span class="colour"><span>C</span><span>O</span><span>L</span><span>O</span><span>R</span></span></span></div>
    <div class="controls">
      <div class="pad"><i class="h"></i><i class="v"></i><button class="l" id="pad-l" aria-label="Previous subtask"></button><button class="r" id="pad-r" aria-label="Next subtask"></button></div>
      <div class="ab2"><label class="px"><button id="btn-b" aria-label="Replay this subtask">B</button>REPLAY</label><label class="px"><button id="btn-a" aria-label="Play or pause">A</button>PLAY</label></div>
    </div>
  </div>
</div></header>
"""

D_SCRIPT = """<script type="module">
import { mount, lcdTogglePlay, lcdStep } from './art.js';
const $ = (id) => document.getElementById(id);
// the inside of the shell spans the whole page
const inside = document.createElement('canvas'); inside.id = 'inside'; inside.setAttribute('aria-hidden', 'true');
document.body.prepend(inside);
const fit = () => { inside.style.height = document.body.scrollHeight + 'px'; };
fit(); addEventListener('load', fit);
mount(inside, 'internals', { seed: 5 });
mount($('lcd'), 'gbc');
document.querySelectorAll('canvas[data-art]').forEach((c, i) => mount(c, c.dataset.art, { rows: +c.dataset.rows || undefined, seed: 11 + i, palette: 'gbc' }));
$('btn-a').onclick = () => lcdTogglePlay(); $('btn-b').onclick = () => lcdStep(0);
$('pad-l').onclick = () => lcdStep(-1); $('pad-r').onclick = () => lcdStep(1);
addEventListener('keydown', (e) => {
  if (e.key === 'ArrowLeft') lcdStep(-1); else if (e.key === 'ArrowRight') lcdStep(1);
  else if (e.key.toLowerCase() === 'a') lcdTogglePlay(); else if (e.key.toLowerCase() === 'b') lcdStep(0);
});
document.querySelectorAll('.plastics button').forEach(b => b.onclick = () => {
  document.body.dataset.plastic = b.dataset.p;
  document.querySelectorAll('.plastics button').forEach(x => x.setAttribute('aria-pressed', String(x === b)));
});
</script>
</body></html>"""

# ── E · Circuit board under frosted Atomic Purple ──
E_CSS = A_CSS + """
html:has(body.board){overflow-x:clip}
body.board{overflow-x:clip;background:#1B1A21;--frost:245,242,235;--ground:rgba(var(--frost),.5);--surface:rgba(255,255,255,.22);--wire:rgba(34,34,42,.18)}
html[data-theme="dark"] body.board{--frost:18,18,22;--ground:rgba(var(--frost),.5);--surface:rgba(255,255,255,.06);--wire:rgba(242,239,232,.16)}
#board{position:fixed;inset:0;width:100vw;height:100vh;z-index:0;pointer-events:none}
#lights{position:fixed;inset:0;width:100vw;height:100vh;z-index:1;pointer-events:none;opacity:.5}
.frost{position:fixed;inset:0;z-index:1;pointer-events:none;background:rgba(var(--frost),.985);backdrop-filter:blur(5px) saturate(.6);-webkit-backdrop-filter:blur(5px) saturate(.6)}
/* the centre, where the content sits, is thicker plastic: more opaque, more blur, soft edges */
.center{position:fixed;inset:0;z-index:1;pointer-events:none;background:rgba(var(--frost),.6);
  --edge:min(42vw,600px);
  -webkit-mask:linear-gradient(90deg,transparent calc(50% - var(--edge) - 120px),#000 calc(50% - var(--edge)),#000 calc(50% + var(--edge)),transparent calc(50% + var(--edge) + 120px));
  mask:linear-gradient(90deg,transparent calc(50% - var(--edge) - 120px),#000 calc(50% - var(--edge)),#000 calc(50% + var(--edge)),transparent calc(50% + var(--edge) + 120px))}
html[data-theme="dark"] .frost{background:rgba(var(--frost),.9)}
html[data-theme="dark"] .center{background:rgba(var(--frost),.45)}
.sheen{position:fixed;inset:0;z-index:1;pointer-events:none;
  background:linear-gradient(116deg,rgba(255,255,255,.14) 0 10%,transparent 24%,transparent 62%,rgba(255,255,255,.07) 72%,transparent 84%),
  url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='160' height='160'><filter id='n'><feTurbulence baseFrequency='.9' numOctaves='2'/><feColorMatrix values='0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 .06 0'/></filter><rect width='100%' height='100%' filter='url(%23n)'/></svg>")}
body.board > *:not(#board):not(#lights):not(.frost):not(.center):not(.sheen){position:relative;z-index:2}

/* thicker plastic behind text: a soft cloud of extra frost, no edge */
.thick{position:relative;z-index:0}
.thick::before{content:'';position:absolute;inset:-30px -36px;z-index:-1;pointer-events:none;background:rgba(var(--frost),.5);
  backdrop-filter:blur(6px);-webkit-backdrop-filter:blur(6px);
  -webkit-mask:radial-gradient(closest-side,#000 62%,transparent);mask:radial-gradient(closest-side,#000 62%,transparent)}
/* windows: clear glass where the content sits */
body.board .episode .frame{background:#F5F2EB}
html[data-theme="dark"] body.board .episode .frame{background:#121216}
body.board .card canvas,body.board .card .sample{background:#F5F2EB}
html[data-theme="dark"] body.board .card canvas,html[data-theme="dark"] body.board .card .sample{background:#121216}
body.board .episode .frame,body.board .card{box-shadow:0 0 0 1px rgba(255,255,255,.35),0 18px 40px rgba(20,10,40,.25)}
body.board .card{background:rgba(var(--frost),.35);}
body.board .readout,body.board .strip button{background:rgba(var(--frost),.35)}
body.board .data{background:rgba(var(--frost),.3);}
/* the yellow bar as yellow plastic */
body.board .field{background:#EFB702}
/* everything on the video in the Game Boy font, at its native 8 px (or 16 px) so the pixels stay sharp */
body.board .episode .chip,body.board .strip button,body.board .readout .tag,body.board .readout b{font-family:'Pokemon GB',monospace;letter-spacing:0;-webkit-font-smoothing:none}
body.board .episode .chip{font-size:8px;padding:6px 8px;text-transform:uppercase}
body.board .strip button{font-size:8px;text-transform:uppercase}
body.board .readout .tag{font-size:8px;text-transform:uppercase;color:var(--dim)}
body.board .readout b{font-size:8px;line-height:16px;margin-top:8px;font-weight:400}
/* grain: on the page and on every card and window, so no surface is flat */
body.board .sheen{opacity:1}
body.board .sheen::after{content:'';position:absolute;inset:0;opacity:.3;mix-blend-mode:multiply;
  background:url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='200' height='200'><filter id='g'><feTurbulence type='fractalNoise' baseFrequency='.95' numOctaves='3' stitchTiles='stitch'/><feColorMatrix type='saturate' values='0'/><feComponentTransfer><feFuncA type='table' tableValues='0 .22'/></feComponentTransfer></filter><rect width='100%' height='100%' filter='url(%23g)'/></svg>")}
html[data-theme="dark"] body.board .sheen::after{mix-blend-mode:screen;opacity:.35}
body.board .card,body.board .kind,body.board .episode .frame{position:relative}
body.board .card::after,body.board .kind::after,body.board .episode .frame::after{content:'';position:absolute;inset:0;pointer-events:none;opacity:.6;mix-blend-mode:multiply;
  background:url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='160' height='160'><filter id='g'><feTurbulence type='fractalNoise' baseFrequency='1.1' numOctaves='2' stitchTiles='stitch'/><feColorMatrix type='saturate' values='0'/><feComponentTransfer><feFuncA type='table' tableValues='0 .18'/></feComponentTransfer></filter><rect width='100%' height='100%' filter='url(%23g)'/></svg>")}
html[data-theme="dark"] body.board .card::after,html[data-theme="dark"] body.board .kind::after,html[data-theme="dark"] body.board .episode .frame::after{mix-blend-mode:screen;opacity:.35}
/* the subtask timeline: one faint Game Boy tint per step; the current step stronger, outlined */
/* the subtask timeline: the Game Boy's olive greens; the current step in DMG green */
body.board .strip button{--seg:#C9D09A;background:var(--seg);color:#1F2A10}
body.board .strip button:nth-child(even){--seg:#B6C080}
body.board .strip button.on,body.board .strip button.fail.on{background:#8BAC0F;color:#0F380F;box-shadow:inset 0 0 0 2px #0F380F}
body.board .strip button.fail.on::before{content:'✕ '}
body.board .strip button i{background:#0F380F}
html[data-theme="dark"] body.board .strip button{--seg:rgba(139,172,15,.14);color:#C9D09A}
html[data-theme="dark"] body.board .strip button:nth-child(even){--seg:rgba(139,172,15,.24)}
html[data-theme="dark"] body.board .strip button.on,html[data-theme="dark"] body.board .strip button.fail.on{background:#8BAC0F;color:#0F380F}
body.board{--fail:#22222A}
html[data-theme="dark"] body.board{--fail:#F2EFE8}
.switch{background:none;border:1px solid var(--wire);color:var(--dim);height:34px;padding:0 12px;border-radius:17px;cursor:pointer;font:500 .75rem var(--mono);letter-spacing:.06em}
.switch:hover{color:var(--ink)}
@media (max-width:860px){.switch{display:none}}
"""

E_SCRIPT = SCRIPT.replace("</script>\n</body></html>", """
import { startBoard } from './board.js';
const cv = document.createElement('canvas'); cv.id = 'board'; cv.setAttribute('aria-hidden', 'true');
const frost = document.createElement('div'); frost.className = 'frost';
const center = document.createElement('div'); center.className = 'center';
const sheen = document.createElement('div'); sheen.className = 'sheen';
const lights = document.createElement('canvas'); lights.id = 'lights'; lights.setAttribute('aria-hidden', 'true');
document.body.prepend(cv, frost, center, lights, sheen);
const board = startBoard(cv, { lights, kind: 'charcoal', failEl: document.querySelector('.episode .frame') });
addEventListener('segment', (e) => board.setFail(e.detail.fail));

</script>
</body></html>""")

PAGES = {
    "a": ("A · Labels", A_CSS, "", NAV + A_HERO + EPISODE + TRUSTED + QUOTE + REST),
    "c": ("C · Plate", C_CSS, "", C_HERO + TRUSTED + QUOTE + EPISODE + REST),
    "e": ("E · Board", E_CSS, "board", NAV + A_HERO + EPISODE + TRUSTED + QUOTE + REST),
}

INDEX = """<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Homepage mockups</title><style>body{font:16px/1.6 system-ui;margin:48px;max-width:640px}li{margin:8px 0}</style></head><body>
<h1>Homepage mockups</h1><ul>
<li><a href="a.html">A · Labels</a> — quiet type hero; the labelled episode is the picture</li>
<li><a href="d.html">D · Atomic purple</a> — the page is see-through plastic; content sits in screens</li>
<li><a href="e.html">E · Board</a> — circuit board under frosted Atomic Purple</li>
<li><a href="c.html">C · Plate</a> — Pantheon-style full-screen art hero</li>
</ul></body></html>"""

import time
VERSION = str(int(time.time()))   # new module URLs on every build, so browsers never run stale art code
for slug, (name, extra, body, content) in PAGES.items():
    html = HEAD.format(name=name, intro=INTRO, css=CSS, extra=extra, body=body) + content + (D_SCRIPT if slug == "d" else E_SCRIPT if slug == "e" else SCRIPT)
    if slug == "e":
        # the Pokémon Red/Blue font: fan recreation "Pokemon GB" (Jackster), freeware per fonts/pokemon-gb-LICENSE.txt
        html = html.replace('<style>', "<style>@font-face{font-family:'Pokemon GB';src:url('fonts/pokemon-gb.ttf') format('truetype');font-display:block}", 1)
    if slug == "d":
        html = html.replace('<link href="https://fonts.googleapis.com/css2?family=Fraunces', '<link href="https://fonts.googleapis.com/css2?family=Press+Start+2P&family=Outfit:wght@300;400;500;600;700;800&family=Fraunces')
        html = html.replace('<a class="announce"', '<a class="announce" style="display:none"')
        for kind in ("strata", "envelope", "contours"):
            html = html.replace(f'<canvas data-art="{kind}"', f'<div class="shot"><canvas data-art="{kind}"', 1)
        html = html.replace('aria-hidden="true"></canvas><div class="body">', 'aria-hidden="true"></canvas></div><div class="body">')
        html = html.replace('<div class="sample" aria-hidden="true">', '<div class="shot"><div class="sample" aria-hidden="true">').replace('fold the bottom third up.</div>', 'fold the bottom third up.</div></div>')
        html = html.replace('<img class="f-mark"', '<div class="speaker" aria-hidden="true"></div><img class="f-mark"')
    html = html.replace("from './art.js'", f"from './art.js?v={VERSION}'").replace("from './board.js'", f"from './board.js?v={VERSION}'")
    (HERE / f"{slug}.html").write_text(html)
(HERE / "index.html").write_text(INDEX)
print("built", ", ".join(f"{s}.html" for s in PAGES), "+ index.html")
