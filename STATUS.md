# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: AWAITING HUMAN VERIFICATION

Phase: M06-R13 (THREE-BODY HIERARCHICAL SYSTEM / OUTER MOONLET) is COMPLETE and
committed/pushed (2026-10-05; H01/H02/H03 passed). M06 is now awaiting the
still-open human visual passes M06-R12-H01/H02, M06-R11-H01, M06-R7-H01 (no
implementation in flight). R13 was the first M06 hardening pass that
intentionally changes the canonical gravity / ephemeris model (R8-R12 kept the
physics frozen; R13 SUPERSEDED the frozen-physics scope of those passes,
notably M06-R12-P09, per the explicit user request — the supersession is
preserved in `TASKS.md`). The two-body system becomes a fixed three-body
hierarchical (Jacobi) system:
- body 2 = OUTER MOONLET, same scale as the companion (`R2 = R1 = R0/9`,
  `mu2 = mu1 = mu0/81`), distinct salted terrain seed (a new pure function of
  the primary seed);
- the inner pair (0,1) keeps EXACTLY the old M05 relative two-body motion
  about its barycentre `B01` (separation 600 m; same omega / period as M05);
- the outer pair (`B01`, body 2) orbits the total barycentre (at the origin)
  on a fixed 1200 m circle at `omega_outer = sqrt((mu0+mu1+mu2)/1200^3)`
  starting at pi/2 (`T_outer ~= 610.13 s`); the total barycentre stays at the
  origin for all t;
- all three bodies' gravity fields are always active on the spacecraft in the
  one global inertial frame (no SOI / patched conics / gravity switch /
  stabilization); tidal locking is per-body (bodies 0/1 at `omega_inner`,
  body 2 at `omega_outer`);
- the contract loop stays 0 <-> 1 (body 2 is never a contract destination);
  legacy transfer / sync routes from body 2 fail safely (no new three-body
  transfer algorithm; TFD-1 / TFD-2 stay open);
- the R12 display-frame layer is preserved and extended: F9 = MOONLET fixed
  frame, a three-body AUTO classifier (all R12 thresholds / hysteresis
  unchanged; body->body transitions always route through World), three-body
  debug-panel readouts, and a new `--debug-predictor-body 0|1|2`
  predictor-fixture selector.
23 automated tests + 3 human gates (`M06-R13-H01` normal play; `M06-R13-H02`
predictor + body-2 fixture; `M06-R13-H03` predictor + body-1 regression). The
10-outer-period zero-thrust moonlet-orbit stability gate is a STOP-and-report
gate (no retuning without a new user decision). **M06-R13-09 (USER decision
2026-10-05):** the R12 primary-fixture invariance is SUPERSEDED — the
2-body-calibrated R12 primary gate became a non-gating LEGACY DIAGNOSTIC, the
companion case is observational (not forced), and the PRIMARY gate is
re-baselined to a clean current-3-body-world orbit run through the authoritative
 Simulation; the classifier SEMANTICS are
 unchanged. The PRIMARY gate observation window is a BOUNDED ONE-REVOLUTION
 classifier-correctness gate (one complete body-0-relative revolution; if it
 cannot complete before the first physical loss, STOP-and-report, do NOT retune)
 — the earlier "10 local periods" wording was SUPERSEDED by the same 2026-10-05
 user decision (`M06-R13-V23` / `M06-R13-09-04`). **PRED-01..08, SIM-COLL-01, and
 the TFD-1/TFD-2 transfer defects all
remain OPEN** (see `docs/m06-predictor-physics-issues.md`); M06 is NOT closed /
accepted; M07 is not started. The underlying harness is the M06-R8
subsystem-isolation debug tool (`--debug-subsystem <name>`, 11 modes); the
M06-R12 reference-frame architecture (F5-F8, AUTO classifier, segmented
rendering, `PHYSICS = WORLD / INERTIAL`) is unchanged in behaviour and
generalized to three bodies in this pass. The canonical rules are extended in
`docs/physics-model-gravity.md`; the full atomic requirements live in
`TASKS.md` (## M06-R13).

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

Test status (current for M06-R13): ALL NON-HUMAN VERIFICATION COMPLETE.
`lander_pred_frame_tests` (all pass): V19 `gate_moonlet` PASSES (~99.8% MOONLET,
final MOONLET); V20 3-body via-WORLD PASSES (P->M = WPWM, M->P = WMWP, no direct
P<->M adjacency; P<->C still WPWC/WCWP); the R12 `gate_primary` /
`primary_fixture_robust_to_prev` (2-body-calibrated) regress under the 3-body
world (root cause: the 3-body tidal/differential perturbation lowers the
primary's dominance — classifier constants UNCHANGED) and are now a NON-GATING
legacy diagnostic, while the new `gate_primary_current` (V23) is the PRIMARY
gate and PASSES as a bounded ONE-REVOLUTION classifier-correctness window
(final=PRIMARY, 100% mature PRIMARY, zero chatter/comp/moonlet, ~1 rev winding,
bounded radius, one revolution completed before any physical loss);
`report_companion` is observational. `lander_predictor_tests` (all pass) now
include V18 3-body predictor parity (window parity bit-identical; 3-body cold
rebuild == authoritative Simulation; a 2-body-only reference diverges 0.308 m,
proving the body-2 field is live). `lander_tests` (all pass) includes V21
10-outer-period stability (seed=555, ~6100 s, body-2 distance stays 49-58 m, no
crash/landing, deterministic). `lander_transfer_warm_tests` and
`lander_binary_tests` pass. Full regression (V22): `ctest` = 11/12 (sole failure
= the pre-existing V14-C body-2 cross-body landing red, NOT introduced by R13;
the earlier TFD-1/TFD-2 hard failures no longer surface — the post-M06 transfer
tests now document/expect the deferred behavior and pass); headless GUI smoke
(incl. `--debug-predictor-body 0/1/2`) exits 0. The only source change this
session was the purely additive `tests/test_predictor.cpp` (V18).

Human-verification status: the M06-R13 closeout gates `M06-R13-H01` (normal
play: three bodies render, contract loop undisturbed, F9 works),
`M06-R13-H02` (predictor + body-2 fixture: AUTO MOONLET, F9, body-2 readouts),
and `M06-R13-H03` (predictor + body-1 fixture unchanged from R12 — regression
check) all PASSed (2026-10-05). The still-open gates from earlier passes
remain: `M06-R12-H01` (static frame + legend + debug-panel visual pass),
`M06-R12-H02` (AUTO frame visual pass — segmented arc, transition markers, no
cross-frame connector), `M06-R11-H01` (the Pass 1 diagnostics visual pass),
and `M06-R7-H01` (the consolidated M06 playtest, the overall M06 acceptance
gate; the `manual` mode is a provisional pass — SIM-COLL-01 supersedes any
collision/contact acceptance; the two transfer modes remain subject to the
known TFD-1/TFD-2). The M06 **predictor subsystem is NOT accepted** (PRED-01..
08 + SIM-COLL-01 remain OPEN in the ledger).
M06-R11 (Pass 1 diagnostics) is committed and pushed to `origin`
(2026-10-04, commit `758223d`). M06-R12 (the prediction reference-frame
architecture) is committed and pushed to `origin` (2026-10-04, commit
`e968d1c`) at its human-verification blocker so the game could be shown /
 reviewed. M06-R13 (this three-body hierarchical pass) is COMPLETE: all 23
 automated tests + the M06-R13-09 supersession are implemented and passing
 (no new ctest failures), and the H01/H02/H03 human gates PASSed (2026-10-05);
 it is now committed and pushed to `origin`. A non-blocking deferred camera/UI
 polish note is recorded (the SYSTEM-view auto-fit zooms somewhat too far out
 now that body 2 expands the system bounds; not fixed in R13). M06 as a whole
 is NOT closed / accepted; the human items above (R12-H01/H02, R11-H01,
 R7-H01) stay open, and the deferred PRED-01..08 / SIM-COLL-01 / TFD-1 /
 TFD-2 remain. `M07` is not active.

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
