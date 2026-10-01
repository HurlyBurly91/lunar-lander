# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: AWAITING HUMAN VERIFICATION

Phase: M06-R1 automated work is complete and the milestone is awaiting
explicit human confirmation of `M06-R1-H01` through `M06-R1-H09`. `M06` builds
on the completed `M05` binary-moon contract loop by adding a compact KSP-style
flight computer: a deterministic zero-thrust ballistic predictor, one maneuver
node, SAS-style attitude holds, a finite-burn node executor, and three
planners (circularize, transfer to the other moon, and match destination-pad
velocity). No `M07` ECS work is part of this milestone.

Specification:

milestones/M06-flight-computer-and-maneuver-planning.md

Execution ledger:

TASKS.md

## Prior milestone

M05 — Binary moon and first contract loop

State: COMPLETE

Record:

records/M05-binary-moon-contract-loop.md

M05 shipped the compact two-body moon system, simultaneous two-body gravity,
body-relative landing/takeoff on moving tidally-locked bodies, the system-scale
camera, and the first repeating primary <-> companion contract loop.

Specification:

milestones/M05-binary-moon-contract-loop.md

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
