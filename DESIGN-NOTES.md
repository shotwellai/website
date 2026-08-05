# Design & brand work — session notes

Handoff document. Covers logo vectorisation, the brand page, the logo kit, social
banners, and the `samples/` design-language explorations. Written so a fresh session
can pick up without the conversation history.

Last updated: 3 Aug 2026.

---

## 1. Current state at a glance

| Thing | Where | Status |
|---|---|---|
| Traced logo SVGs | `brand/assets/`, repo root | **On `main`** (PR #4, merged) |
| Brand page | `/brand/index.html` | **On `main`**, live at shotwell.ai/brand — **not linked from anywhere** |
| Logo kit ZIP | `brand/shotwell-logo-kit.zip` + `~/Downloads/Shotwell-Logo-Kit.zip` | Done, sendable |
| Social banners | `~/Downloads/Shotwell-Banners/` | Done, 6 files |
| Figma Make file | `~/Downloads/Logo Design File/` | Rebuilt around final logo |
| Design-language samples | `samples/` | **Untracked**, local only |
| Root logo SVGs | `favicon.svg`, `shotwell-lockup.svg`, `shotwell-mark*.svg` | **Uncommitted** |

Local branch is `brand-page` (already merged). `main` has moved well ahead — Ultra case
study, video animations, a rewritten `index.html`. **The `samples/` pages were built
against an older `index.html`; re-check assumptions before porting anything.**

---

## 2. The logo

### What it is

A hand-drawn walking lantern. Two drawings exist:

- **Thin / primary** — fine strokes, teardrop flame. `shotwell-mark.svg`, viewBox `74 109 378 592`.
- **Bold / alternate** — heavy strokes, blob flame, pointed cap. `shotwell-mark-bold.svg`, viewBox `56 55 392 639`.

The thin one is **primary**. The bold one is the alternate for small sizes, low
resolution, print with ink spread, and embroidery. **The bold mark is required for the
favicon** — the thin one's legs are under 1% of mark height and disappear below ~48px.

### How they were traced

Source: `episode` stills the user supplied, plus `shotwell-logo.png`. Tooling installed
via Homebrew and pip: `potrace 1.16`, `cairosvg`, `Pillow`, `numpy`.

Colour separation first — the drawings are near-black ink `rgb(28,27,27)` plus a burnt
orange flame `rgb(201,110,45)` — then each mask traced separately so the flame is its own
path and can be recoloured.

```
potrace -s -o out.svg -k 0.45 --turdsize 4 --alphamax 1.0 --opttolerance 0.6 in.pgm
```

**Two things that matter if this is ever redone:**

- Feed potrace a **native-resolution grayscale PGM**, not a supersampled 1-bit mask.
  Supersampling a bitmask re-traces interpolation artefacts — it produced 67KB of
  over-detailed curves versus 8.5KB at native res, with no fidelity gain.
- Both masks must be traced at identical dimensions so the emitted `<g transform>`
  matches and the flame registers inside the ink without manual alignment.

Verified by rasterising the result and comparing to the source: **thin mark IoU 0.975
ink / 0.985 flame; bold mark IoU 0.985.**

### Wordmark

**Fraunces, weight 400, capital S, black period.** Set as `Shotwell.`

Chosen over Cormorant Garamond and the previous Instrument Serif. Cormorant was the more
elegant pairing but its hairlines break down at nav size.

### Lockup construction

- Mark height = **1.7 × wordmark cap height**
- Mark and wordmark **vertically centred** on their ink (not baseline-aligned — that was
  an earlier iteration the user rejected)
- Gap = **0.34 × wordmark font-size**
- Ink `#2A2A32`; flame `#C96E2D` or ink for fully mono

Fraunces' true cap ratio is **0.7128**, measured from the font file. The `0.73` in the
Figma Make file is an approximation.

---

## 3. Logo kit

`brand/shotwell-logo-kit.zip` (968KB, 36 files). Also unzipped at
`~/Downloads/Shotwell-Logo-Kit/`.

Contents: horizontal lockup and mark-only, each as SVG in three variants (standard with
ember flame / all-black / all-white) plus transparent PNGs — lockups at 400/800/1600w,
marks at 256/512/1024. Bold alternate in its own `alternate-bold/` folder. `README.md`
written for the recipient, `PREVIEW.png` for a quick look.

### The wordmark is outlined — and this was the hard part

A third party without Fraunces would otherwise get Times New Roman. Outlining required
`fontTools`, which had two traps:

**Fraunces defaults are `wght 900, opsz 9`** — not the 400/64 the design uses. Naive
glyph extraction produces a heavy, wrong-optical-size wordmark. Must instantiate:

```python
inst = instantiateVariableFont(TTFont(src), {"wght": 400, "opsz": 64}, inplace=False)
```

**Kerning lives in GPOS and does not apply to raw outlines.** Two pairs in "Shotwell."
are kerned: `we` at **−26** units, `ll` at **+41** (upem 2000). These must be walked out
of the GPOS PairPos subtables and applied manually to the advance positions.

**Install note:** `pip install fonttools` fails in this environment — `files.pythonhosted.org`
does not resolve, though `pypi.org` does. Workaround: download the source tarball from
GitHub and run with `PYTHONPATH=/path/to/fonttools-*/Lib`.

---

## 4. Brand page — `/brand`

Merged to `main` in PR #4. Live at `shotwell.ai/brand`.

Two sections only: **The logo** and **The mark**. Each has a preview with a
white/dark/tinted background switcher that swaps to the correct variant and names the
file to use — that's the page's one idea, since background context is the entire reason
multiple variants exist. Below each, download chips for every SVG and PNG.

**Written self-contained — it does not link `styles.css`.** That stylesheet is ~2,100
lines carrying nav, hero, and tweaks-panel rules; importing it risked specificity
collisions on section spacing. The page redeclares the same tokens instead.

An earlier version had Instrument Serif display type, plus Colour / Spacing / Don'ts /
Typeface sections. The user cut all of it — **the page is deliberately minimal, Outfit
only. Don't re-add sections without asking.**

### Also in that PR

`.gitignore` gained `db-backups/`, `*.sql`, `*.sql.gz`. The repo is **public** and
`db-backups/` holds production SQL dumps that were untracked but unignored. Nothing from
it was ever committed — history is clean.

---

## 5. Social banners

`~/Downloads/Shotwell-Banners/` — six PNGs, light and dark:

| File | Size | For |
|---|---|---|
| `shotwell-x-header-*` | 1500 × 500 | X / Twitter header |
| `shotwell-linkedin-profile-*` | 1584 × 396 | LinkedIn personal background |
| `shotwell-linkedin-company-*` | 1128 × 191 | LinkedIn company cover |

Logo is centred at 28–34% of banner width. **Centred deliberately** — both platforms
overlap the bottom-left with the avatar and crop the sides on mobile; a left-aligned
logo gets partly covered. Verified against the avatar zones.

Asked about HDR (the Instagram "extra bright whites" effect): **it does not apply.**
Profile chrome is a separate pipeline from feed media on every platform and is flattened
to SDR. LinkedIn has no HDR support at all.

---

## 6. Figma Make file — `~/Downloads/Logo Design File/`

React + Vite + shadcn export from Figma Make. `npm i && npm run dev`.

`src/app/App.tsx` was rebuilt around the final logo. Key structure:

- `MARK_RATIO = 1.7`, `LOCKUP_FS = 64`, `FRAUNCES_CAP = 0.73` at the top — the resize dials
- `ShotwellMark` (thin) and `ShotwellMarkBoldMono` (bold) both take `width` **or** `height`,
  plus `ink` and `flame` colour props
- `ShotwellLockup` composes mark + live Fraunces text; props `size`, `ink`, `bold`, `flame`
- `Wordmark` takes `family`, `caps`, `size`, `weight`, `color`, `dot`

Sections: The Logo → Backgrounds → The Mark → Alternate (bold) → Scale → App icon →
Construction → Wordmark spec → Palette → Original sketches.

The neon gradient (`#00FF87 → #00BFFF → #8B5CF6`) was **removed entirely**, including
from `theme.css` where `--accent` was a raw `#00BFFF` painting blue onto shadcn
components. Reverted to stone `#E0D8C4`.

Alignment uses `align-items: baseline` / `items-center` rather than computed offsets. A
replaced element's baseline is its bottom margin edge, so this works without magic
numbers. Fraunces happens to have ink-centre within 0.005em of line-box centre at
`line-height: 1` (asc 98 / desc 26) — **that's a property of Fraunces, not of the CSS.**
Another face may need a `translateY` correction.

---

## 7. Design language — `samples/`

Untracked, local only. Serve with `python3 -m http.server 8099` from the repo root, then
open `http://localhost:8099/samples/`.

Built from a vision board the user supplied (two versions; the second added a yellow
halftone sphere, an ink starburst, a Bedrock Energy press card, and a magenta blob).

### The thesis

The board's references are almost all **tracked skeletons, keypoints joined by hairlines,
registration marks, mesh trajectories, scan logs** — which is what annotated robot data
looks like. So the design language is **annotation as aesthetic**, and **halftone is the
house image treatment** (a segmentation mask at low resolution *is* a dot matrix).

### Files

```
samples/
  index.html      contact sheet + rationale
  demo.html       full homepage demo, palette switcher
  hero-a.html     hero band, live halftone video, scrubber   ← user prefers this
  hero-b.html     hero as a measured "plate" with registration chrome
  hero-c.html     editorial split, frame one side, argument the other
  press.html      announcement template                       ← user likes this
  fields.html     stacked saturated colour fields
  palettes.html   five colour directions compared
  assets/
    halftone.js   the renderer — works on <img> or <video>
    theme.js      light/dark toggle, inverts halftone polarity
    scrubber.js   click/drag/keyboard video timeline
    tokens.css    design tokens
    frame-*.jpg   stills extracted from the episode videos
```

### Halftone parameters — do not lose these

The source footage averages **~90/255 luma** — genuinely dark. A raw brightness-to-dot-size
mapping produces mostly tiny dots and reads as noise. This was the single biggest bug and
took a contact sheet to diagnose:

```js
halftone(canvas, imgOrVideo, {
  step: 7 * dpr,   // 6–7 for stills, 7 for video
  lo: 0.04,        // black point
  hi: 0.85,        // white point
  gamma: 0.62,     // lifts midtones — this is the fix
  invert: true     // dark grounds: bright pixels → big dots. Flip for light grounds.
})
```

Video works because `drawImage` accepts a video element. Repaint is capped at 24fps —
the dot grid is the cost, not the decode. Dot count is independent of DPR because `step`
scales with it.

**Structural rule: no type ever sits directly on the dot field.** Floating serif over
halftone is unreadable regardless of tone mapping. All three heroes put type on a solid
panel or a separate column.

### Colour — measured, twice, and I was wrong twice

**Ember `#C96E2D` is dead.** The user rejected orange as "very AI driven," and they were
right on two counts:

1. *Aesthetically* — cream ground + high-contrast serif + terracotta accent is a
   documented AI-design default.
2. *Functionally* — clustering 18 frames across five episodes: **95% of saturated
   footage pixels sit at hue 26–30°** (the wooden table). An orange accent competes with
   the imagery rather than marking it. The footage's *neutrals* are cool (h170–200), so
   a cold palette lets the wood be the warmth.

Then the user asked whether the four replacement palettes came from the board. **They
didn't** — they came from the footage and from CV convention. Clustering the board
itself (170,908 chromatic pixels, page-white and near-black excluded):

| Hue band | Share |
|---|---|
| Yellow (`#EFB702` alone is 32%) | **44%** |
| Blue | 14% |
| Cyan | 14% |
| Magenta | 11% |
| Green | 9% |
| **Orange** | **3.6%** |

**Orange was never on the board.** It came from the logo flame and was carried across
without checking.

### The five directions in `palettes.html`

| Name | Accent | Source |
|---|---|---|
| **Board** | gold `#EFB702` + blue `#3D82E8` | the vision board — the only board-derived one |
| Segmentation | cyan `#00C8DC` + magenta `#E5006E` | COCO / Cityscapes mask convention |
| Blueprint | ultramarine `#4B6BFF` | technical drawing |
| Instrument | green `#5BE39B` | footage neutrals + gripper LED |
| Darkroom | red `#D8342B` | photographic, warm without terracotta |

Caveat on gold: at h46 it's only ~18° from the wood. It separates on saturation and
brightness (`s0.99 v0.94` vs the wood's `s0.5 v0.4`), not hue — narrower than blue's escape.

`demo.html` has a **palette switcher** cycling Board → Segmentation → Blueprint across a
full page, which is where the accent has to carry the CTA, eyebrow rules, pass ticks, and
saturated field simultaneously.

Reworked 2026-08-03 to use only content from today's live site: real headline/subtitle,
nav = Case study / Talk to us / Upload videos now (24 hr), Ultra case-study section (the
10→60 operators line + the `dataset.query` card, linking `/ultra-case-study/`), trusted-by,
and the yellow field is the contact block. The hero scrubber uses the **real 8-subtask
segmentation of fold-rubric.mp4 from `hero-pipeline.js`** (labels, timings, rubric rules);
clicking a segment snaps to that subtask's start, and a live annotation plate (top right)
shows the current step name in **dot-matrix type** plus its ✓/✕ rubric rules. Poster-first
hero: paints a halftone still instantly, video takes over when frames arrive.

**Halftone-on-video bug (fixed 2026-08-03):** `halftone()` originally read `img.width`,
which is the HTML *attribute* (0) on a `<video>` element — so the video path silently drew
nothing and the hero went black in every browser once playback started. It now reads
`videoWidth/naturalWidth`. If a canvas treatment ever shows a black band again, check
intrinsic-vs-attribute size first.

**Blog (added 2026-08-03):** `blog.html` (index) and `blog-ultra.html` (the Ultra case
study restyled) follow the **press.html card language** — surface card with accent tab,
mono kicker over a rule, halftone object/band, mono meta row, date+serif rows for the
list. Content is the real `/ultra-case-study/` text (X%/Y% placeholders omitted).
Demo nav links Blog; "Trusted by" sits above the case-study section per user request.

### Type

Fraunces (display, matches the logo), Outfit (body), **IBM Plex Mono** (labels). The live
site's `styles.css` defines `--font-mono: 'Outfit'` — a sans pretending to be mono. The
board's entire label system depends on real monospace.

---

## 8. Open questions

1. **Which palette.** User is leaning **Board (yellow `#EFB702`)** — said "i like the
   yellow" reviewing `demo.html` (2026-08-03). Not final; the switcher remains for
   comparison.
2. **Which hero.** User prefers A. B and C still use the static frame and the old clip —
   if live video is right, it should apply to whichever layout wins.
3. **Does the logo flame follow the site accent, or stay its own colour?** Legitimately
   could differ, but should be a decision rather than drift.
4. **`/brand` is not linked from anywhere.** Live but unreachable without the URL. There's
   no footer on the site to put it in.
5. **The homepage still uses `shotwell-logo.png`** — the old flameless raster, 2 refs in
   `index.html`. Swapping to the traced SVG shifts nav layout (the new mark is
   tight-cropped where the PNG had built-in whitespace) and needs a CSS adjustment.
6. **`samples/` is untracked** and root-level `favicon.svg` / `shotwell-lockup.svg` /
   `shotwell-mark*.svg` are uncommitted.

---

## 9. Rebuilding things

```bash
# serve the site + samples
python3 -m http.server 8099        # → localhost:8099/samples/

# Figma Make file
cd ~/Downloads/"Logo Design File" && npm run dev

# extract a frame
ffmpeg -ss 2 -i episode-4.mp4 -frames:v 1 -vf scale=1400:-2 -q:v 3 out.jpg

# render an SVG (needs the dyld path on this machine)
DYLD_LIBRARY_PATH=/opt/homebrew/lib python3 -c "import cairosvg; ..."
```

Videos in the repo: `episode-2..5.mp4` (1916×1074), `fold-rubric.mp4` (1278×720, 19.1s —
this is the one the live hero uses, and the one the samples use).

**`episode-4` halftones best** of the five — white robot arms give strong bright dots and
real light/dark structure. `episode-2` is the muddiest. The samples use `fold-rubric`
because the user asked to keep the original hero clip.
