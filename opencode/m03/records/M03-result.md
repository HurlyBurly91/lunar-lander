# M03 Result — Diagnose and Repair Regressions

## Implementation summary

Six regressions were identified in `src/sim.cpp` (each was annotated in the
code) and repaired at their root causes:

1. **Frame-rate-dependent physics** — `advance()` executed exactly one physics
   step per call. Restored the fixed-timestep accumulator: `advance()` adds
   `real_dt` to the accumulator and runs `step_fixed()` once per consumed
   `fixed_dt`, stopping early if the lander lands or crashes mid-loop.
2. **Reset preserving angular velocity** — `reset()` saved `state_.omega`
   before clearing state and restored it. `reset()` now assigns a fresh
   `State{}` only, clearing all dynamic state.
3. **Low-altitude tunneling** — `step_fixed()` resolved ground contact *before*
   integrating and returned early, so a crossing step stayed below the surface
   until the next call. Contact resolution now happens *after* integration:
   the step that reaches `y <= 0` clamps `y` to 0 and resolves the contact in
   the same step.
4. **Negative fuel** — fuel consumption was never clamped. Fuel is now clamped
   to 0 after per-step consumption.
5. **Angle-boundary misclassification** — the angle was never normalized and
   the safety check used the raw angle. A `normalize_angle()` helper maps any
   angle to (-pi, pi]; the representation is normalized each fixed step, and
   the landing safety check uses the angular distance from upright. The
   landing thresholds themselves are unchanged (not weakened).
6. **Non-deterministic pads** — `reset()` seeded the RNG with
   `seed ^ random_device()`, so identical seeds produced different pads. The
   seed is now the sole source of terrain randomness.

## Files changed

- `src/sim.cpp` — all six fixes above.
- `tests/test_sim.cpp` — kept the two pre-existing tests unchanged and added
  regression tests:
  - frame-rate independence (same elapsed time via 1 call vs 64 calls produces
    the same tick count and state),
  - pad determinism for identical seeds (five seeds, incl. edge values),
  - near-upright lander at the 2pi +/- 0.01 boundary must land,
  - sideways lander (pi/2) must still crash (criteria not weakened),
  - fast low-altitude crossing is resolved in the same step and never leaves
    the lander below the surface,
  - state is frozen after landing until reset.
- `STATUS.md` — state set to COMPLETE.

## Tests executed

- `cmake -S . -B build`
- `cmake --build build -j` (clean rebuild; no warnings with
  -Wall -Wextra -Wpedantic)
- `ctest --test-dir build --output-on-failure` — 1/1 passed
- `git diff --check` — clean
- `./build/lander_cli` run twice, output byte-identical (CLI determinism)
- Discrimination check: temporarily restored the committed (regressed)
  `src/sim.cpp` and ran the suite; all six regression categories failed
  (14 failed assertions), including the two pre-existing tests. Restored the
  fix and the full suite passed.

## Failures encountered

- Initial baseline: the two pre-existing tests failed ("reset must clear
  angular velocity", "fuel must not become negative") — expected, they
  correspond to regressions 2 and 4. No unexpected failures; no test was
  modified or weakened.

## Dead ends or reverted approaches

- None. Each regression had a single clear root cause visible in the code; no
  alternative approaches were needed.

## Remaining known defects

- None known. All six reported symptoms are fixed and covered by tests.
  Note: as in the committed codebase, this milestone does not add M02
  features (multiple pads, rotational inertia, input recording); those are
  separate milestones and were intentionally not touched.
