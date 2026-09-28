# M01 — Implement the Lander

## Goal

Complete the existing skeletal Lunar Lander simulation.

Do not replace the project wholesale. Implement the supplied interfaces and
extend them only when technically justified.

## Required behavior

Implement:

- deterministic reset from a seed
- deterministic landing-pad generation
- fixed 120 Hz physics
- frame-rate-independent `advance(real_dt, input)`
- gravity
- directional main thrust
- angular control
- finite fuel consumption
- fuel clamping at zero
- angle normalization
- pad collision
- safe landing
- unsafe landing/crash
- ground crash outside pads
- score on successful landing
- restart/reset behavior

The simulation must support test-state injection through the existing
`set_state()` interface.

## Acceptance criteria

- Project builds cleanly.
- Existing tests pass.
- Two simulations reset with the same seed generate the same pads.
- Equivalent elapsed time split across different `advance()` intervals produces
  equivalent physics.
- Fuel never becomes negative.
- Reset completely restores dynamic state.
- Safe pad contact lands.
- Excessive impact velocity crashes.
- Ground contact outside a pad crashes.
- Angles equivalent modulo 2π are treated equivalently for landing.
- `git diff --check` passes.

## Closeout

Write `records/M01-result.md`.
