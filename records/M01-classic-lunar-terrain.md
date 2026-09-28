# M01 — Classic lunar terrain

Date: 2026-09-28
Status: COMPLETE

## What changed

### Terrain model (new)

- `include/lander/terrain.hpp` / `src/terrain.cpp`: a `Terrain` is a
  single-valued, deterministic 1-D height function of x, with a list of
  flat landing sites (`Pad` moved here from `sim.hpp`; it now also carries
  the site's flat surface height `y`).
- Surface: three octaves of 1-D value noise with linear interpolation
  (linear, not smooth, so the surface stays visibly jagged):
  - broad undulation: ±9 m, 120 m period
  - medium hills: ±4 m, 28 m period
  - fine detail: ±1.6 m, 6.5 m period
  - total amplitude ≤ ~14.6 m, so the spawn point (y = 20 m) is always
    above ground (covered by a test).
- Landing sites: up to 3, each 12 m wide (half-width 6 m, wider than the
  lander), centers drawn deterministically from [-150, 150] with a minimum
  center spacing of 100 m (sites can never overlap). Each site is flattened
  to the base-surface height at its center, so it reads as a deliberate
  plateau carved into the terrain. Some seeds place only 2 sites (rejection
  attempts exhausted after 200 tries each); 2 always satisfies "several".
- Randomness comes from a SplitMix64 PRNG (the same integer-only family
  `src/sim.cpp` already uses), seeded with the game seed, so the whole
  world (terrain + sites) is reproducible per seed.

### Simulation

- `include/lander/sim.hpp` / `src/sim.cpp`: `Simulation` now owns a
  `Terrain`; `reset(seed)` regenerates it. The old single-pad, flat-floor
  model is gone.
- Ground contact uses `terrain.height_at(lander.x)` instead of y = 0: the
  lander is clamped to the actual surface and the contact is classified as
  landing (on a designated site, within the existing velocity and angle
  limits) or crash (contact off-site, or limits exceeded). The existing
  fixed-timestep integrator, thrust, fuel, scoring, and terminal-state
  rules are unchanged.

### GUI

- `src/gui.cpp` renders the real generated terrain: it samples
  `height_at` every meter across the view and fills a regolith body with a
  worn band and a lighter surface line, 10 m distance ticks, and each
  landing site as a bright slab with an edge highlight and a faint vertical
  guide line so sites are findable from altitude.
- HUD altitude is now relative to the terrain under the lander; crash
  debris is placed on the terrain height. Lander, star field, controls,
  pause, restart, and new-seed behavior are unchanged.
- Fixed a latent bug in `write_ppm` (the screenshot helper from M00): it
  assumed 3 bytes per pixel while reading rows with the surface pitch, so
  every screenshot was pixel-phase shifted and leaked the unused 0xFF byte
  of the 4-byte format into the data. It now converts to 24-bit RGB and
  copies pixel by pixel.

### Tests

`tests/test_sim.cpp` (single binary, `check()` assertions):

- Updated for per-pad `y`: the rest-state test checks the lander sits at
  the pad's surface height; the off-site crash test starts just above the
  terrain at a point verified to be outside every site.
- New coverage: same seed → identical heights and site layout; different
  seed → different surface; every generated site is flat and at least as
  wide as required, with several present; height queries are
  deterministic and the surface is visibly uneven (range > 5 m); the spawn
  point is always above the generated terrain.
- All M00 physics tests (thrust, rotation, fuel, safe landing, impact
  velocity crash, angle limits, modulo-2π angle equivalence, terminal
  state stability, reset determinism) pass unchanged.

## Verification

Automated:

    cmake --build build
    ctest --test-dir build --output-on-failure   # 1/1 passed
    git diff --check

Headless GUI runs (SDL dummy video/audio drivers), exit code 0 for both:

    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ./build/lander_gui \
        --seed 31337 --frames 200 --fps 60 \
        --screenshot records/m01-screenshots/seed31337-descent.ppm
    # final: ... state=flying seed=31337

    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ./build/lander_gui \
        --seed 1234 --frames 600 --fps 60 \
        --screenshot records/m01-screenshots/seed1234-crash.ppm
    # final: y=-3.279 ... state=crashed seed=1234
    # (y matches the generated terrain height at x=0, not y=0)

Numeric pixel assertions on the screenshots (color counts only; per the
text-only constraint, the coding model did not inspect the images):

- seed31337-descent.ppm (1280×720): 772791 space, 56878 regolith, 9611
  worn-band, 3308 surface-line, 340 pad-green, 720 pad-highlight
  (edge + guide), 240 lander-body pixels.
- seed1234-crash.ppm: whole frame covered by the terminal-state dim
  overlay, as expected after a crash.
- Note: seed 1234's sites (x ≈ [-95,-83] and [63,75]) are outside the
  ±45.7 m start viewport, so the start frame shows terrain without a
  visible slab; seed 31337 (site at x ≈ [12.7,24.7]) shows the slab
  entering view during descent.

Human visual/play verification (to be confirmed by a human, then noted
here): open the three artifacts in `records/m01-screenshots/`
(`seed31337-start.ppm`, `seed31337-descent.ppm`, `seed1234-crash.ppm`)
and/or run `./build/lander_gui` and check the Human verification list in
the milestone spec.

## Problems encountered

1. Dangling reference in the modulo-2π angle test: it held a `const Pad&`
   into `pads()` and then called `reset()`, which now replaces the whole
   `Terrain` (reallocating the pad vector) and invalidated the reference.
   Fixed by copying the `Pad` by value before the reset.
2. `write_ppm` pixel-phase bug (latent since M00): discovered because
   known on-screen colors counted zero pixels in screenshots. See What
   changed / GUI.

## Follow-ups (out of scope for M01)

- Curved-moon stage (per AGENTS.md): circular moon, radial gravity,
  orbit-scale values. Not started.
- The PPM artifacts in `records/m01-screenshots/` are kept on disk as
  human-inspection artifacts and are regenerable with the commands above;
  they are not committed to keep the milestone commit code-and-docs only.
