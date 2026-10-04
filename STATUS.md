# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: AWAITING HUMAN VERIFICATION

Phase: `M06-R12` (M06 HARDENING — prediction reference-frame architecture).
Implementation + automated verification are **complete**; only the human visual
pass (`M06-R12-H01` / `M06-R12-H02`) remains open (alongside the still-open
`M06-R11-H01` and `M06-R7-H01`). This bounded pass re-frames the
already-computed INERTIAL prediction for display / analysis and changes NO
physics / propagation / gravity / collision / transfer / landing / guidance /
binary-ephemeris behaviour:
- a pure frame transform `transform_to_frame` (`include/lander/pred_frame.hpp` /
  `src/pred_frame.cpp`) gives three inertial display frames — WORLD (the existing
  barycentric frame, unchanged), PRIMARY (body-0 centred), COMPANION (body-1
  centred) — via position/velocity ephemeris subtraction only (no rotation, no
  body-fixed frame, no SOI, no gravity switch);
- a pure AUTO orbit-reference classifier (`classify_auto`) reads the inertial
  samples and, per sample + per body, computes a two-body epsilon / dominance /
  winding / radial-ratio / angular-consistency window and commits a
  Primary / World / Companion reference segment with hysteresis (3 samples AND
  0.5 s to enter and to release; body->body changes always route through World);
- `TrajectoryPrediction` now carries parallel timed samples
  (`position_world`, `velocity_world`, `time`) alongside the existing decimated
  `pre` / `post` positions (additive; no consumer changed);
- the GUI gains F5 = AUTO / F6 = PRIMARY / F7 = COMPANION / F8 = WORLD display
  frame keys (F2 / F3 / F4 policy keys unchanged), draws AUTO segments as
  separate polylines with a transition marker at each boundary (no cross-frame
  connector), and shows a persistent `FRAME` legend row plus a debug-panel
  `PRED FRAME` / `REF SEG` / per-body `EPS`/`DOM`/`WIND`/`RATIO` / `PHYSICS =
  WORLD / INERTIAL` readout.
The classifier's two fixture GATES pass with real physics: a bounded companion
orbit coast classifies AUTO -> COMPANION (98.7%, final COMPANION) and a bounded
primary orbit coast classifies AUTO -> PRIMARY (99.8%, final PRIMARY). This is
the next pass of the bounded post-M06 hardening (R11 = Pass 1 diagnostics); it
added NO physics fix, NO frozen-COAST validator, NO autoland / transfer /
collision change, and NO M07 work. **PRED-01..08, SIM-COLL-01, and the TFD-1/
TFD-2 transfer defects all remain OPEN** (see `docs/m06-predictor-physics-issues.md`);
M06 is NOT closed / accepted. The underlying harness is the M06-R8
subsystem-isolation debug tool (`--debug-subsystem <name>`, 11 modes) —
implemented and evidenced in `TASKS.md` (M06-R12, then M06-R11, M06-R10, M06-R9,
M06-R8).

Phase detail (M06-R8 harness): a bounded subsystem-isolation debug
harness. `./build/lander_gui --debug-subsystem <name>` launches the game
showing exactly ONE M06 subsystem at a time (11 modes: manual, predictor,
attitude, node-edit, node-executor, transfer-cold, transfer-warm,
autoland-primary, autoland-companion, autoland-cross, ui), with unrelated
flight-computer / debug readouts hidden and a common minimum flight readout
always shown. This is a DEBUG pass, NOT a feature or hardening pass: it must
not fix subsystem defects, change canonical physics / `fixed_dt`, weaken
tests, start M07, or commit. A debug scenario is a startup-only fixture (fixed
per-mode seed, reproducible); once begun the subsystem runs through the normal
simulation / control paths (no runtime teleportation or hidden forces). With
no selector, normal gameplay is byte-for-byte unchanged (verified: headless
final-state line is byte-identical for absent vs `none`). The `transfer-cold`
/ `transfer-warm` modes are meant to EXPOSE the deferred TFD-1/TFD-2 defects,
not fix them. M06-R8 (all R8-01..R8-16 and V01..V06) is implemented and
evidenced in `TASKS.md`. The M06-R9 follow-up — keeping the predicted-trajectory
overlay (the projected path: COAST / PLAN arc + PE / AP / closest-approach
markers, the powered LIVE projection, and the kind legend) visible in
`--debug-subsystem predictor`, the mode's primary graphical observable — is
implemented and evidenced there; it reuses the existing normal-gameplay drawing
calls (no new rendering) and leaves every other mode and normal gameplay
unchanged. The harness is available for the user to inspect one subsystem at a
time. `M06-R7-H01` (the consolidated M06 playtest) stays OPEN as the acceptance
gate; this tool supports that playtest one subsystem at a time. M06-R6 / M06-R7
remain code-complete (see `TASKS.md`).

Test status (re-verified for M06-R12): full build clean (only the two
pre-existing narrowing warnings at `src/gui.cpp:1477,1495`); ctest is **10/12**
— the only 2 failing targets are the documented KNOWN POST-M06
transfer-subsystem defects (TFD-1 `lander_tests` "primary-source transfer found
a plausible arc", TFD-2 `lander_transfer_warm_tests` "0->1 solved at 0/4"), both
in the untouched `src/ballistic.cpp` solver, unchanged from the baseline and not
part of this pass. **No new failures.** The new `lander_pred_frame_tests`
target (the 12th) passes: all 15 R12 tests (transform identity / moving-body
removal / co-rotating stability; companion + primary classifier GATES; via-World
P->W->C and C->W->P transitions; hysteresis prev-dependence; incomplete-horizon
gate; no-mutation; segment/sample alignment; `predict_trajectory`->`classify_auto`
end-to-end) pass. All M06-core targets (landing, flight-computer, predictor,
binary, GUI smoke) remain green. See "M06 known post-M06 transfer-subsystem
defects" in `TASKS.md`.

Open human-verification: `M06-R12-H01` (static frame + legend + debug-panel
readout visual pass) and `M06-R12-H02` (AUTO frame visual pass — segmented
arc, transition markers, no cross-frame connector) are OPEN and gate this pass.
`M06-R11-H01` (the Pass 1 diagnostics visual pass) and `M06-R7-H01` (the
consolidated M06 playtest) also remain OPEN. The M06 **predictor subsystem is
NOT accepted** (findings PRED-01..08 + SIM-COLL-01 remain OPEN in the ledger;
the reference-frame pass only re-frames the display / analysis and does not
close those issues). `M06-R7-H01` remains the overall M06 acceptance gate; the
`manual` mode is a provisional pass (harness only — SIM-COLL-01 supersedes any
collision/contact acceptance). The two transfer modes remain subject to the
known TFD-1/TFD-2.
M06-R11 (Pass 1 diagnostics) is committed and pushed to `origin`
(2026-10-04, commit `758223d`). M06-R12 (this prediction reference-frame
architecture) is now committed and pushed to `origin` at the human-verification
blocker so the game can be shown / reviewed; M06 as a whole is NOT closed /
accepted and the remaining human items (`M06-R12-H01`/`H02`, `M06-R11-H01`,
`M06-R7-H01`) stay open. `M07` is not active.

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
