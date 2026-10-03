# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: ACTIVE

Phase: `M06-R10` (M06 DEBUG PHASE: predictor human inspection — record / defer /
stop). The user ran the `--debug-subsystem predictor` mode and found issues, so
the M06 **predictor subsystem is NOT accepted**; the nine findings (PRED-01..08,
SIM-COLL-01) are recorded in a durable ledger (`docs/m06-predictor-physics-
issues.md`) with stable IDs, preserved uncertainty, and the specific tests
required to close each. Diagnosis and any fix are **deferred to a bounded
post-M06 predictor/physics hardening pass** (not M07, not an M06 scope
expansion). This was a strictly record-only pass: no defect was fixed, no
parameter / physics / collision / predictor / rendering change, no new test or
scenario, no commit; the automated baseline is preserved at **9/11** (only the
known TFD-1/TFD-2 fail). The `manual` mode is a provisional pass (harness
functionality only; SIM-COLL-01 supersedes any collision/contact acceptance).
The remaining debug subsystems may be inspected independently; the two transfer
modes remain subject to the known TFD-1/TFD-2. The underlying harness is the
M06-R8 subsystem-isolation debug tool (`--debug-subsystem <name>`, 11 modes;
M06-R9 kept the predicted-trajectory overlay visible in predictor mode) —
implemented and evidenced in `TASKS.md` (M06-R10, then M06-R9, then M06-R8).

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

Test status: full build clean; ctest is **9/11**. The 2 failing targets are
documented KNOWN POST-M06 transfer-subsystem defects (TFD-1 `lander_tests`,
TFD-2 `lander_transfer_warm_tests`) DEFERRED to a post-M06 hardening ledger per
the R7 MVP policy — unchanged from before this phase, not marked passing, not
weakened, not part of the MVP playtest. The new `lander_debug_subsystem_tests`
target (selector, seeds, common readout, fixture signatures, determinism,
landing getters) passes. All M06-core targets (landing, flight-computer,
predictor, binary, GUI smoke) are green. Re-verified in the M06-R10 record-only
baseline: full build clean; `lander_debug_subsystem_tests`,
`lander_predictor_tests`, and `lander_landing_tests` all pass (landing at full
~168 s); ctest is 9/11 with **no new failures**. See "M06 known post-M06
transfer-subsystem defects" in `TASKS.md`.

Open human-verification: `M06-R7-H01` (the consolidated M06 playtest) remains
the single acceptance gate and stays OPEN. The M06 **predictor subsystem is NOT
accepted** (record-only findings PRED-01..08 + SIM-COLL-01, deferred to a
bounded post-M06 predictor/physics hardening pass); the `manual` mode is a
provisional pass (harness only — SIM-COLL-01 supersedes any collision/contact
acceptance). The remaining debug subsystems may be inspected independently of
the predictor deferral (the two transfer modes remain subject to the known
TFD-1/TFD-2). This debug phase does NOT commit; M06 is committed and pushed
only after human acceptance. `M07` is not active.

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
