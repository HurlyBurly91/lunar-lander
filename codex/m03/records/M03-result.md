# M03 — Diagnose and Repair Regressions — Result

## Implementation Summary

All six reported regressions were diagnosed in `src/sim.cpp` and repaired at
their root causes:

1. Physics depending on render/update frequency: `advance()` executed one
   physics step per call. Restored fixed-timestep accumulation: `advance()`
   adds `real_dt` to the accumulator and runs as many fixed 1/120 s steps as
   fit, stopping the loop when the state becomes terminal.
2. Rotational motion preserved across restarts: `reset()` copied the previous
   `state_.omega` into the fresh state. `reset()` now value-initializes the
   full `State` (and clears the accumulator).
3. Low-altitude contacts passing through the surface for one frame: ground
   contact was only checked *before* integration, so a step crossing the
   surface ended with `y < 0`. A post-integration contact check now clamps
   `y` to 0 and resolves the contact (land/crash) within the same step. The
   pre-integration check is kept for states already touching the surface at
   step start.
4. Fuel going slightly negative: fuel was never clamped after burns. Fuel is
   now clamped to 0 after each fixed step.
5. Angle representation breaking landing classification: the angle was never
   normalized and the landing check compared raw `|angle|` against the safe
   limit, so an angle equivalent to a small upright deviation modulo 2*pi
   (for example `2*pi - 0.05`) was classified as a crash. Angles are now
   normalized into the canonical `[-pi, pi]` representation after each fixed
   step, and `resolve_ground_contact()` compares the angular distance from
   upright (normalized absolute value). No landing threshold was changed.
6. Same seed producing different pads: `reset()` mixed `std::random_device`
   entropy into the terrain RNG (`seed ^ rd()`). The pad RNG is now seeded
   solely by the game seed, restoring per-seed determinism of terrain and of
   whole games.

## Files Changed

- `src/sim.cpp` — all six repairs (accumulator loop in `advance()`; clean
  state and seed-only RNG in `reset()`; post-integration contact check, fuel
  clamp, and angle normalization in `step_fixed()`; normalized-angle
  comparison in `resolve_ground_contact()`).
- `tests/test_sim.cpp` — existing tests kept unchanged; regression tests
  added: pad determinism across independent instances for several seeds;
  frame-rate independence (identical state and tick counts for 1/120, 1/60,
  1/30, and mixed `advance()` chunking over two simulated seconds); same-step
  settling and landing for a crossing pad contact; same-step crash for a
  crossing off-pad contact; crash on unsafe (high-impact) pad contact;
  landing-classification equivalence for angles differing by multiples of
  2*pi (both safe and unsafe classes); state freezing after landing;
  full-session determinism under a time-varying control sequence; fuel
  clamping under simultaneous main-thrust and rotation burns.

## Tests Executed

- `cmake -S . -B build`
- `cmake --build build -j` (clean build, `-Wall -Wextra -Wpedantic`, no
  warnings)
- `ctest --test-dir build --output-on-failure` — `lander_tests` passed,
  including the two pre-existing tests and all new regression tests.
- `git diff --check` — clean.
- Direct `lander_cli` runs: two runs produced identical output
  (`x=0 y=19.1833 vx=0 vy=-1.62 fuel=100`), matching one second of pure
  free-fall.
- Throwaway cross-process probe (built in `/tmp`, outside the repo) printing
  pads plus the final state after a 300-step session with seed 424242: two
  separate process runs produced byte-identical output, confirming
  process-level determinism.

## Failures Encountered

None. The build was clean on the first pass and all tests passed on the
first run after the fixes.

## Dead Ends or Reverted Approaches

None. Each regression had a single localized root cause in `src/sim.cpp`;
no alternative approaches were attempted or reverted.

## Remaining Known Defects

None known. Pre-existing behaviors intentionally left untouched (out of M03
scope):

- `advance()` with a NaN `real_dt` pollutes the accumulator without running a
  step until the next reset (pre-existing guard behavior, unchanged).
- This repository snapshot predates the M02 extensions (rotational inertia
  config, three pads, input recording/replay); M03's scope is regression
  repair, so those features were not added.
