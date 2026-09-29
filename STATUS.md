# Status

## Current milestone

M05 — Binary moon and first contract loop

State: AWAITING HUMAN VERIFICATION

Phase: M05-R2 (smooth local camera + SYSTEM navigation polish; automated work
complete)

M05-R1 automated verification was complete, but human verification (2026-09-28)
found three presentation failures: the local camera snaps/teleports when the
reference body changes; the SYSTEM view gives no closing/opening cue once the
destination leaves the viewport; and the SYSTEM-scale ship is a white square.
These are addressed by follow-up request group M05-R2 (see TASKS.md). All
M05-R2 automated checks now pass; the implicated M05-R1 H-items (H03/H04/H05)
and the M05-R2 H-items remain open and must be re-verified by the user. M05 is
not complete and no completion record has been written.

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
