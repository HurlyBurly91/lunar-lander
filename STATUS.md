# Status

## Current milestone

M05 — Binary moon and first contract loop

State: AWAITING HUMAN VERIFICATION

Phase: M05-R3 (camera readability, explicit HUD, reaction-wheel damping,
navigation/gravity overlay, explicit CW/CCW circularize, triple-tap guards for
`N` / `O` / `Shift+O`, compact crash-dialog geometry, tidal locking for both
moons)

M05-R1 and M05-R2 automated work is complete, but their human-verification
items remain open. Human verification of the M05-R2 build produced the
M05-R3 follow-up request group (see TASKS.md): LOCAL AUTO needs a projected
lander readability floor, SYSTEM zoom needs smooth wide-to-close wheel control
with a readable lander, the HUD needs explicit reference-frame readouts,
reaction-wheel angular damping is requested, a navigation/gravity vector
overlay is requested, and the circularize control must use explicit
`O` / `Shift+O` CW/CCW semantics. M05-R3 was then extended with reusable
triple-tap guards for dangerous/debug controls, a compact crash-dialog
geometry fix, and tidal locking for both moons (M05-R3-09): both bodies spin
with the binary line of centres (zero at t = 0, ~216.94 s period), terrain
and pads rotate rigidly with their body, landed attachment and takeoff
inherit the full surface-point velocity, landing is evaluated against the
rotating surface point, and the contract destination pad is a moving target.

All automated work for M05-R3 is complete (2026-09-29): clean build, all
five test suites pass (including the new tidal-locking tests), whitespace
check clean, headless GUI smoke OK. The milestone now awaits user
confirmation of every still-open human-verification item (M05-R1, M05-R2,
and M05-R3 H01..H15). M05 remains open and no completion record has been
written.

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
