# Status

## Current milestone

M05 — Binary moon and first contract loop

State: AWAITING HUMAN VERIFICATION

Phase: HUMAN VERIFICATION (M05-R1)

All automated verification for M05-R1 is complete (build, 4/4 test suite,
git diff --check, headless smoke runs, and the new contract-loop test all
pass; see TASKS.md "Verification evidence"). The milestone is now awaiting
human confirmation of the runtime items H01..H07; it is not complete and no
completion record has been written until the user confirms each one.

M05 turns the M04 flight simulation into the first complete game loop by
extending the world to a compact two-body system: a 1/9-scale companion
derived from the canonical universe scaling law, a fixed 600 m analytic
circular barycentric binary, simultaneous two-body gravity, body-relative
collision/landing/takeoff on moving bodies, a system-scale camera view, and
a minimal repeating primary <-> companion contract loop.

Specification:

milestones/M05-binary-moon-contract-loop.md

Execution ledger:

TASKS.md

## Prior milestone

M04 — Circular moon and orbital physics

State: COMPLETE

Record:

records/M04-circular-moon-orbital-physics.md

M04 shipped a closed circular moon with radial inverse-square gravity, wrapped
circular terrain and landing sites, local-frame landing checks, O circularize /
F refuel, a corrected moon fill, and a usable local-frame camera. The M04-R1
presentation follow-up (exact player camera anchor, fixed-step render
interpolation, high-resolution timing, fixed screen-space starfield) and the
M04-R2 continuous presentation-time flame animation are complete. All automated
checks pass and all human verification items (M04-R1-H01/H02/H03, M04-R2-H01)
were confirmed on 2026-09-28.

Specification:

milestones/M04-circular-moon-orbital-physics.md

## Prior milestone

M03 — Throttle and camera polish

State: COMPLETE
