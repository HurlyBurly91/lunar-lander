# Status

## Current milestone

M05 — Binary moon and first contract loop

State: AWAITING HUMAN VERIFICATION

Phase: M05-R3-23..30 automated corrective round complete; awaiting human
re-test of H17 and H22 on the next corrected build. H18, H19, H20, and H21
are confirmed PASS. The round implemented geometry-driven body render
coverage (full body at wide zoom, viewport-safe local patch only when its
closure is provably outside the viewport), removed the visible radial
tick/seam artifacts, made PRIMARY/COMPANION treatment identical, and added
new headless regression tests. See TASKS.md.

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

All automated work for M05-R3-01..09 was complete (2026-09-29): clean build,
all five test suites pass (including the new tidal-locking tests), whitespace
check clean, headless GUI smoke OK.

Human verification of that build produced a further set of findings,
persisted as M05-R3-10..16 (2026-09-29): `R` retry must use the same
triple-tap protection as the other guarded controls; LOCAL manual zoom must
reach a scale range similar to SYSTEM while keeping the reference body
"down"; the starfield must become an inertial background that rotates with
the final presentation camera angle (replacing the fixed screen-space
backdrop); the SYSTEM camera must never auto-pan toward the destination
(ship always exactly centred); zoom displays need unambiguous adaptive
formatting (e.g. `0.04X`, `1.00X`); and two one-shot debug initializers are
requested, `B x3` (body-synchronous orbit around the source body, far side
from the target) and `T x3` (ballistic inter-body transfer computed with the
real two-body gravity), both triple-tap guarded, neither an autopilot.

Automated work for M05-R3-10..16 is complete (2026-09-29): clean build,
all five test suites pass (including the new guarded-key, wide-zoom,
inertial-starfield, SYSTEM no-pan, sync-orbit, and transfer tests),
whitespace check clean, headless GUI smoke OK. See TASKS.md for per-item
evidence, including the transfer solver design and the companion-source
sync-orbit drift caveat.

Human verification of that build (2026-09-29) confirmed H19 (SYSTEM
centre-on-ship) and H20 (`B x3` sync-orbit), but exposed new findings,
persisted as the corrective round M05-R3-17..22: H17 FAIL (the wide LOCAL zoom
exposes a visible render/draw failure), H18 PARTIAL (the LOCAL inertial star
rotation is correct, but the SYSTEM-fixed starfield is also correct and must
not regress), and H21 FAIL (`T x3` freezes the interactive loop while solving
and teleports the craft to a canonical departure position). The round fixes the
wide-LOCAL-zoom rendering failure (M05-R3-17), fixes black terrain/body seams
(M05-R3-18), makes `T x3` non-blocking (M05-R3-19), reworks `T x3` to change
only velocity with no position teleport (M05-R3-20), adds a starfield
no-regression check (M05-R3-21), and handles the b49a476 human-verification
 state (M05-R3-22). Automated work for this round is complete (2026-09-29);
awaiting human re-test on the corrected build.

Human re-test of that corrected build (57a9b8a, 2026-09-29) confirmed H18
(starfield), H21 (`T x3`), and the still-closed H19/H20, but H17 (wide LOCAL
reference-body rendering) and H22 (radial/vertical seam artifacts) remain
FAIL. The failure is reference-body-specific and matches the remaining
camera-mode/reference-body-dependent `full_body` coverage and the
`scale > 0.35` radial tick path in `src/gui.cpp`. This created the
M05-R3-23..30 follow-up corrective round, whose automated work is now
complete (2026-09-29).

All still-open human-verification items (M05-R1, M05-R2, M05-R3
H01..H16, H17, and H22) remain open until explicitly confirmed by the user
(H18/H19/H20/H21 are closed with human evidence). M05 remains open and no
completion record has been written.

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
