# M02 — Contextual camera and manual zoom

Date: 2026-09-28
Status: COMPLETE

## What changed

### Camera (new)

- `include/lander/camera.hpp` (header-only, rendering-free): a `Camera` with a
  world-space centre `(x, y)` and a `zoom` multiplier on a base
  pixels-per-meter scale. It has two modes:
  - `AUTO` (default): eases the zoom between an `overview_zoom` (0.40) at
    high altitude and a `landing_zoom` (1.40) near the surface. The choice is
    driven by the followed target's altitude above `terrain.height_at(x)`
    (never absolute world y) and uses **hysteresis** — it switches to landing
    below `alt_to_landing` and back to overview above `alt_to_overview` — so
    hovering in the gap between the two thresholds cannot make it flicker.
  - `MANUAL`: the zoom is set by the player with the mouse wheel (up = in,
    down = out), applied multiplicatively by `wheel_step` (0.12) and clamped
    to `[zoom_min, zoom_max]` = [0.20, 4.0]. The camera still follows the
    target's position. With no wheel input the zoom is stable.
- `M` toggles the mode. The game starts in AUTO. Entering MANUAL keeps the
  current zoom (no visible jump); returning to AUTO leaves it and the AUTO
  target eases it toward the contextual scale. Mode switching never touches
  the simulation.
- All AUTO zoom changes and all centre movement ease with
  `1 - exp(-rate * dt)` smoothing (`zoom_rate` 3.0, `follow_rate` 4.0), so
  there is no instantaneous visual snap.
- `scale()` is the **single** world-to-screen factor (base_scale 14.0 * zoom)
  consumed by every renderer, so camera motion/zoom can only rigidly move and
  scale the fixed world-space terrain — it cannot change its shape.

### Framing (deliberate design change from the M01 follow-up)

- The lander is held at a fixed fraction from the top of the screen
  (`lander_top_fraction` 0.30). The GUI bakes a **zoom-proportional** vertical
  bias into the camera target: `target_y = lander_y - (0.5 - f) *
  (windowHeight / scale)`. Because the bias scales with the current scale, the
  downward reach of the frame scales with the zoom: the ground stays in view
  at every scale and the lander is never pushed off the top when zoomed in.
- The earlier plan of a *fixed* pixel offset (36 px below centre at all zooms)
  was rejected: a probe showed that with a fixed offset the terrain surface
  fell below the bottom edge of the (shrinking) window in every tested seed,
  so the "landing" view did not actually show the nearby terrain.

### Threshold tuning (documented deviation from the spec's suggested values)

- The spec suggested `overview -> landing` at altitude < ~25 m and
  `landing -> overview` at > ~35 m, and explicitly allows adjusting these if
  the resulting view is clearly better. They were set to `alt_to_landing` =
  18 m and `alt_to_overview` = 25 m instead.
- Reason: with `landing_zoom` 1.40 the landing view's downward reach is
  `(1 - f) * windowHeight = 0.7 * (720 / (14 * 1.4))` = ~25.7 m. The
  *landing* state stays active up to `alt_to_overview`, so that threshold must
  be at or below the 25.7 m reach or the ground drops out of the frame while
  the player climbs back up through the dead band. Keeping the spec's 35 m
  would require a landing zoom of ~1.0 (no real magnification), which defeats
  "magnifies the landing area". 18/25 gives a 7 m hysteresis gap and keeps the
  ground in the landing view at all times.
- Resulting field of view: overview (0.40) shows ~228 m horizontally (enough
  to survey the >=100 m-spaced landing sites); landing (1.40) shows ~65 m
  horizontally / ~37 m vertically (enough detail for final corrections).

### GUI

- `src/gui.cpp`: the local `Camera` and hard-coded `kScale` were removed. All
  renderers (`to_screen`, `draw_space`, `draw_terrain`, `draw_lander`,
  `draw_debris`, `draw_hud`) now take `const lander::Camera&` and use
  `cam.scale()` / `cam.x()` / `cam.y()`.
- Input: `SDL_EVENT_MOUSE_WHEEL` accumulates vertical notches into
  `pending_wheel` (consumed once per frame); `SDL_SCANCODE_M` sets
  `pending_cam_toggle`. `cam.update(...)` is called every frame — even while
  paused — so the mode toggle and wheel stay responsive (with the lander
  frozen the position target is constant, so there is no drift).
- HUD: a top-right indicator shows `CAM AUTO` (dim) or `CAM MANUAL x.xX`
  (green, with the current zoom). The bottom control help and `--help` now
  mention `M CAMERA` / `WHEEL ZOOM`. The existing M01 terrain sampling
  (fixed world-space 1 m lattice), HUD altitude, landing/crash, and restart
  behaviour are unchanged.

### Tests

- `tests/test_camera.cpp` (new, registered as `lander_camera_tests` in
  `CMakeLists.txt`): focused, rendering-free checks for
  - AUTO is the initial mode at a neutral zoom (scale == base_scale);
  - AUTO selects the landing scale below the lower threshold and the overview
    scale above the upper threshold;
  - hysteresis preserves the current state inside the dead band and switches
    only when a threshold is crossed (no chatter);
  - the zoom eases monotonically and converges to its target (no hard snap);
  - entering MANUAL preserves the current zoom; the wheel changes the zoom
    only in MANUAL (in, then out);
  - manual zoom clamps at the min and max;
  - returning to AUTO resumes automatic behaviour and eases back to the
    contextual scale;
  - a long mixed input sequence leaves the simulation state and the terrain
    heights byte-for-byte unchanged.
- All M00/M01 `lander_tests` pass unchanged.

## Verification

Automated:

    cmake --build build
    ctest --test-dir build --output-on-failure   # 2/2 passed
    git diff --check                              # clean

Headless GUI runs (SDL dummy video/audio drivers), all exit code 0. Final
simulation states match the M01 baseline exactly, confirming the camera is
presentation-only and the fixed-timestep physics is untouched:

    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ./build/lander_gui \
        --seed 31337 --frames 200 --fps 60
    # final: x=0.000 y=11.112 vx=0.000 vy=-5.359 ... ticks=397 state=flying
    # (identical to the M01 200-frame result)

    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ./build/lander_gui \
        --seed 1234 --frames 600 --fps 60
    # final: ... y=-3.279 ... state=crashed
    # (y matches the M01 crash height = terrain.height_at(0))

    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ./build/lander_gui --help
    # usage now lists "M camera mode (Auto/Manual), mouse wheel zoom (Manual)"

Numeric terrain-rigidity checks (world-space, per the text-only constraint):

- Zoom invariance: for two camera states differing only in zoom, 51 shared
  world samples map by a pure rigid similarity with solved a_x = a_y = 5.0
  (uniform scale) and max residual 1.278e-10 px — no wobble, swim, or stretch.
- Translation invariance (M01 invariant re-confirmed): for two camera
  positions, 67 shared samples are a constant on-screen shift of -420 px with
  max deviation 0.000e+00 px.

Numeric AUTO-framing checks (driving the real `Camera` with scripted altitude
profiles and measuring the resulting frame window):

- Descent 34 m -> 3 m and ascent 3 m -> 80 m (exercising the 90 m overview
  reach): the terrain surface and the lander remain inside the frame at every
  step for all tested seeds (worst off-screen gap 0.00 m).
- Oscillating altitude through the dead band: exactly 2 mode switches per
  cycle (6 in 12 s at 0.25 Hz) — clean hysteresis, no rapid flicker.

Numeric pixel assertions on the regenerated screenshots (color counts only;
the coding model did not inspect the images):

- `overview_47519.ppm` (deepest surface, wide view): 384575 space, 444285
  regolith, 10878 worn-band, 3441 surface-line, 576 pad-highlight, 40
  lander-body pixels.
- `landing_777.ppm` (low start, landing scale): 551728 space, 275850 regolith,
  10813 worn-band, 3275 surface-line, 685 pad-green, 966 pad-highlight, 424
  lander-body pixels.
- `approach_232.ppm` (multi-pad, mid-fall): 364807 space, 464393 regolith,
  10725 worn-band, 3292 surface-line, 574 pad-highlight, 42 lander-body
  pixels.

In every shot the terrain now renders (the flawed first-draft overview shots
counted zero regolith pixels because the fixed-offset framing pushed the
ground off-screen); the corrected zoom-proportional framing shows terrain in
both the overview and the landing views.

Human visual/play verification (requested):

- Run `./build/lander_gui`. AUTO: climb high for a wide overview (survey
  several landing sites), descend and confirm the view smoothly magnifies
  near the surface, hover around the ~18–25 m transition and confirm no
  flicker, then climb again and confirm it smoothly returns to overview.
- MANUAL: press M (confirm `CAM MANUAL`), use the mouse wheel to zoom in and
  out, stop scrolling and confirm the zoom holds, fly while zoomed and confirm
  the camera keeps following, press M again and confirm it eases back to the
  automatic scale.
- Regression: confirm the terrain does not wobble/swim/stretch while the
  camera translates or zooms, and that landing/crash behaviour is unchanged.
- The three artifacts in `records/m02-screenshots/` are for human inspection.

## Problems encountered

1. Fixed-offset framing lost the ground: the initial plan placed the lander a
   constant 36 px below screen centre at every zoom. A geometry probe showed
   that, once the window shrinks at landing zoom, the terrain surface falls
   below the bottom edge for every seed, so the "landing" view did not show
   the nearby terrain. Fixed by switching to a zoom-proportional framing bias
   (see Framing).
2. Dead-band threshold vs. landing reach: keeping the spec's 35 m
   landing->overview threshold with a 1.4x landing zoom would leave the ground
   off-screen while climbing through the upper dead band. Fixed by lowering
   the thresholds to 18/25 m to match the landing view's 25.7 m reach (see
   Threshold tuning). The unit-test hysteresis altitudes were updated to the
   new thresholds.
3. 24-frame headless tick boundary: the M02 24-frame run reports 45 fixed
   ticks where the M01 "24-frame" figure corresponded to 44. This is a GUI
   real-time frame-pacing boundary, not a physics change; the 200-frame and
   600-frame runs match M01 exactly and the simulation sources are
   byte-identical.

## Follow-ups (out of scope for M02)

- Curved-moon stage (per AGENTS.md): circular moon, radial gravity, orbit-scale
  values, and a camera/local frame usable in flight. Not started.
- The PPM artifacts in `records/m02-screenshots/` are kept on disk as
  human-inspection artifacts (git-ignored via `/records/*-screenshots/`) and
  are regenerable with the commands above; they are not committed, keeping the
  milestone commit code-and-docs only.
