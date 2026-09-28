# M03 — Diagnose and Repair Regressions

## Goal

The repository contains a previously working Lunar Lander implementation with
multiple regressions.

Repair the implementation. Do not simply special-case the visible tests.

## Reported symptoms

Users have reported all of the following:

1. Physics changes depending on render/update frequency.
2. Restart sometimes preserves rotational motion from the previous attempt.
3. Very low-altitude contacts sometimes pass through the surface for one frame.
4. Fuel can become slightly negative.
5. A nearly upright lander can be classified incorrectly when its angle crosses
   the ±π / 2π representation boundary.
6. Starting two games with the same seed does not always produce identical
   landing pads.

Not every regression has an existing visible test.

Determine the actual causes.

## Acceptance criteria

- All six reported regressions are fixed.
- Existing tests pass.
- Add regression tests where useful.
- Do not weaken landing criteria.
- Do not remove fixed-timestep behavior.
- Determinism is restored.
- `git diff --check` passes.

## Closeout

Write `records/M03-result.md`.
