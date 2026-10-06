# Status

## Current milestone

M06 — Flight computer and maneuver planning

State: AWAITING HUMAN VERIFICATION

Phase: M06-R18 (node-executor observability / presentation prep) — AWAITING
HUMAN VERIFICATION at M06-R18-H01 (implementation + automated verification
complete 2026-10-06; STOPPED UNCOMMITTED until human acceptance). Work
shipped in this phase: (1) the "magic force" observation is CONFIRMED from
the code as a PRESENTATION defect — the physics is genuine engine (the
authoritative sim consumes the executor's applied `Input.main_throttle`:
real acceleration + real fuel spend), while the drawn plume was sourced from
the player's manual throttle knob (no plume while the executor burns); the
same latent defect exists for the transfer midcourse and the landing
autopilot; (2) the rendered thrust source is now the actual applied
main-throttle input — one narrow presentation source of truth
(`actual_thrust` in gui.cpp fed from the exact `step_input` beside
`panel_ctx.last_step_input`; pure gate
`presentation_thrust_level(state, applied)` in sim.hpp/sim.cpp; correct for
manual, node executor, transfer midcourse, and landing autopilot; no
controller change; flame never inferred from acceleration); (3) a minimal
node-executor scene visualization at the drawn ship (fixed ACT thrust-axis
ray via `thrust_hat(angle)`, fixed VGO ray from the executor's
`dv_remaining()` omitted when ~zero, compact state label; numeric panel
kept) — new delimited region "node-executor debug display geometry" in
debug_subsystem.hpp + `draw_node_executor_debug_axes` in gui.cpp; (4) the
node-executor debug fixture starts PAUSED with a "PAUSED FOR NODE EXECUTOR
[P] RUN" banner, executor remaining armed (no default-node-time / delta-v /
engine-acceleration / executor-timing / physical-state change). Strict scope
held: NO executor redesign or retune (bang-bang, alignment thresholds,
ignition, burn time, VGO accounting, final partial throttle, node planning,
maneuver basis, transfer midcourse, landing autopilot, physics, prediction,
camera, M07 all untouched). Automated results 2026-10-06: build clean;
`lander_flight_computer_tests` PASS (new
test_node_executor_presentation, checks A-E incl. real-simulation physicality:
per-step dv == main_accel x applied x dt, fuel 1000 -> 952, VGO 4 -> 2 m/s,
final-partial-throttle 0.29-0.31, no residual post-COMPLETE/ABORT);
`lander_debug_subsystem_tests` PASS (new test_node_executor_overlay: camera
rotation only, fixed length, zero/eps/non-finite VGO omission, purity); full
`ctest` 11/12 (sole failure = pre-existing, unrelated V14-C cross-body
soft-land in lander_landing_tests, NOT fixed, reported separately); headless
smokes: `--debug-subsystem node-executor --frames 300` -> ticks=0, fuel
1000.00, rc=0 (paused fixture); `--seed 1 --frames 120` -> ticks=237,
state=landed, rc=0 (baseline unchanged); paused-scene screenshot artifact at
/tmp/opencode/r18_node_executor_paused.ppm for the human run. GATE: human
re-run `./build/lander_gui --debug-subsystem node-executor`, press P, verify
the 8 checks in M06-R18-H01 (ALIGN/WAIT no flame, BURN flame along actual
axis with THR/fuel/VGO moving, final partial step, COMPLETE, ABORT,
off-axis safety, physicality). No commit until human acceptance.
Historical phase record below (node-edit cell M06-R17 + R16 + R15; all
COMPLETE, committed as c2114f3 on 2026-10-06): M06-R17 (R16 correction:
node-edit RUNNING jitter / stair-step — presentation cadence) — COMPLETE
(2026-10-06): diagnosis confirmed, fix implemented, automated verification
complete, and the human gate M06-R17-H01 was a PASS (all R15/R16 checks plus
RUNNING stability; no node-edit defect remains); the node-edit cell
(R15 + R16 + R17) is committed. The M06-R16 human
re-run (2026-10-06) was a PARTIAL PASS: everything passed (paused fixture +
banner, marker on the PRE/POST junction through all edits, de-collided
labels, H/J/K/L/C/DEL, F5-F9, pause/resume) EXCEPT one defect — while the
simulation is RUNNING, the node/trajectory visualization visibly jitters /
stair-steps (stable once paused). The 12 Hz cache-rebuild hypothesis was
CONFIRMED by an instrumented run (forced 4 Hz rebuild cadence headless: in
43/43 rebuild epochs the marker was frozen between rebuilds, snapped back
onto the ship on each, jump ~= v_ship x interval, node_time_effective ==
rebuild epoch throughout the overdue window). The fix (presentation-only):
a pure `node_event_state` flight-computer helper computes the node-event
geometry (position, effective epoch, NodeBasis, dv_world) from the CURRENT
ship state at render cadence (zero propagation when EFFECTIVE NOW; bounded
propagation for a future node); the node-edit marker / arrows / labels
consume it every frame so the marker stays glued to the drawn ship while
RUNNING; the PRE/POST long arcs remain 12 Hz cached with a new explicit
`PRED AGE <ms>` staleness readout. No predictor physics, no 12 Hz budget
change, no normal-gameplay or other-debug-mode change (TASKS.md P01-P05).
Automated verification: new overdue + future node-event regressions pass;
`ctest` = 11/12 (sole failure = pre-existing V14-C body-2 cross-body
landing); headless smokes exit 0 (node-edit paused ticks=0; normal seed 1
ticks=237 unchanged). Human re-run (M06-R17-H01, 2026-10-06, USER): PASS —
the RUNNING jitter is fixed acceptably and all R15/R16 checks hold; no
node-edit defect remains. The user's experimental ENTER press crossed into
the node-executor subsystem (the next cell, M06-R18); its missing-plume
observation is that cell's presentation-defect hypothesis. The original
defect history: M06-R15's node-edit visualization FAILED its first human gate
(H01):
the NODE marker / PGR / RAD / DV graphics drift off the PRE/POST arc once the
node is overdue, and the NODE/PGR/RAD/DV labels collide. Root cause: the
predictor clamps the node to the current prediction epoch internally
(`t_node = max(t0, snap(node->time, dt))`) but R15 rendered the node graphics
from the raw, stale `maneuver_node->time` (a future node's raw time == its
effective epoch, so Delete+C "fixes" it until the node lapses). R16 is a
display/presentation-only correction: (1) add a single explicit effective epoch
`node_time_effective = max(t0, snap(node->time, fixed_dt))` to
`TrajectoryPrediction` and make EVERY node graphic consume it (marker transform,
AUTO-frame lookup, PGR/RAD/DV geometry); the raw scheduled `node->time` is never
silently overwritten and is shown as `SCHED T±<s>` with an `EFFECTIVE NOW` tag
once overdue; (2) a deterministic overdue-node regression (marker == PRE/POST
junction in PRIMARY/COMPANION/MOONLET at the effective epoch; the old
raw-epoch transform drifts; no sim mutation; covers fresh/overdue/Delete+C/
F5-F9); (3) the `--debug-subsystem node-edit` fixture starts PAUSED by default
with a "PAUSED FOR NODE EDIT  [P] RUN" banner so a human can inspect it without
the event epoch moving (P toggles pause; H/J/K/L/C/Delete rebuild the prediction
immediately while paused with no physics tick; normal play + other debug modes
unchanged; no hidden orbit stabilization); (4) deterministic, non-colliding
placement of the four labels (NODE below-right of the marker; PGR/RAD/DV at
their arrow tips; DV offset from PGR when nearly parallel); (5) explicit
`pre.back() == post.front() == node_position` at the effective epoch (overdue =>
at the current ship position). Scope guard: no change to maneuver-node planning
math, the node basis, the node executor, the attitude controller, gravity /
ephemerides, the prediction integrator, the R12/R13 frame transforms, the
control bindings, or normal gameplay. R16 implementation and automated
verification are COMPLETE (2026-10-05; D01-D07 / V01-V06 in `TASKS.md`): the
effective epoch now drives every node graphic, the overdue-node regression
passes, the node-edit fixture starts paused, and the labels are de-collided.
The M06-R16-H01 human re-run (2026-10-06) was a PARTIAL PASS (only the RUNNING
jitter remained); the phase is now M06-R17, which fixes that one defect; DO NOT
commit until M06-R15-H01 re-PASSes via the M06-R17-H01 re-run. M06-R14
(attitude debug visualization) is the most recently COMPLETED pass
(2026-10-05; M06-R14-H01 human-accepted; committed and pushed to origin).
M06 is still awaiting the other open human passes M06-R12-H01/H02,
M06-R11-H01, M06-R7-H01 (unchanged by R14/R15). The previously completed pass,
M06-R13 (THREE-BODY HIERARCHICAL SYSTEM / OUTER MOONLET), is COMPLETE and
committed/pushed (2026-10-05; H01/H02/H03 passed). R13 was the first M06
hardening pass that intentionally changes the canonical gravity / ephemeris
model (R8-R12 kept the physics frozen; R13 SUPERSEDED the frozen-physics scope
of those passes, notably M06-R12-P09, per the explicit user request — the
supersession is preserved in `TASKS.md`). The two-body system becomes a fixed three-body
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

Test status (current for M06-R18, which kept all R17/R16/R15 work and added
its regressions; the R13 automated baseline below is unchanged): M06-R18 —
full build clean (2026-10-06); new `tests/test_flight_computer.cpp`
`test_node_executor_presentation` passes (check A: manual=0 / applied=1 /
plume=1; check B: WAIT and ALIGN no-thrust, no plume; check C: real
`Simulation` run of the fixture, 60 wait + 60 burn steps — per-step dv vs the
identical zero-input reference run matches `main_accel x applied x dt` within
1e-3, fuel 1000 -> 952, VGO 4 -> 2 m/s at the node; check D: 4.11 m/s node ->
124 burn steps, final partial throttle 0.29-0.31 with plume == applied every
step; check E: zero applied + plume post-COMPLETE and post-ABORT, plus
crashed/landed/empty-fuel suppression gates); new
`tests/test_debug_subsystem.cpp` `test_node_executor_overlay` passes (camera
rotation only, fixed screen length, zero/below-eps/non-finite VGO omitted,
pure function); `ctest` = 11/12 (sole failure = the pre-existing, unrelated
V14-C cross-body soft-land in `lander_landing_tests`, NOT introduced by R18);
headless smokes: `--debug-subsystem node-executor --frames 300` exits 0 with
`ticks=0` and fuel 1000.00 (fixture starts PAUSED), `--seed 1 --frames 120`
exits 0 with `ticks=237` / `state=landed` (normal baseline unchanged);
paused-scene screenshot artifact written to
/tmp/opencode/r18_node_executor_paused.ppm for human inspection. M06-R17 —
full build clean (2026-10-06); new `tests/test_debug_subsystem.cpp`
regressions
`test_node_event_overdue_matches_predictor` / `test_node_event_future_matches_
predictor` pass (node-event state from the current ship state equals the ship
state exactly when overdue and equals `predict_trajectory` for the same
epoch, overdue and future cases); `ctest` = 11/12 (sole failure = the
pre-existing V14-C body-2 cross-body landing, NOT introduced by R17);
headless smokes exit 0 (node-edit paused `ticks=0`; normal seed 1 `ticks=237`
identical to baseline). M06-R16 — full build clean;
`lander_flight_computer_tests` (new `test_node_time_effective_overdue`:
effective-epoch == t0 when overdue / == snap(node.time) when future, per-frame
F5-F9 junction match, stale raw-epoch transform drifts, node.time never
overwritten, no sim mutation), `lander_debug_subsystem_tests` (new
`test_node_edit_label_placement`: distinct/deterministic NODE/PGR/RAD/DV label
positions, DV side-flip when near-parallel), and `lander_predictor_tests` all
pass; `ctest` = 11/12 (sole failure = the pre-existing V14-C body-2 cross-body
landing, NOT introduced by R16); headless smokes: `--debug-subsystem node-edit`
exits 0 with `ticks=0` (fixture correctly starts PAUSED), normal gameplay exits
0 with ticks advancing, and `attitude` / `node-executor` / `predictor` smokes
all exit 0 still ticking (other modes unchanged). M06-R14 — `ctest` = 11/12
(sole failure = the pre-existing V14-C body-2 cross-body landing, NOT
introduced by R14); `lander_debug_subsystem_tests` (incl. the new
`test_attitude_debug_axes`) pass; headless `--debug-subsystem attitude` /
`predictor` / normal smokes all exit 0.
M06-R13 baseline (all non-human verification complete):
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

Human-verification status: the CURRENT gate is `M06-R18-H01` (AWAITING HUMAN
VERIFICATION — do not self-complete; work uncommitted until accepted): human
re-run `./build/lander_gui --debug-subsystem node-executor` (fixture starts
PAUSED with the banner), press P, then verify: (1) ALIGN — ACT axis
physically rotates toward the VGO, throttle 0, no flame; (2) WAIT — alignment
held, throttle 0, no flame; (3) BURN — begins only when aligned and at/after
ignition, visible flame along the actual craft thrust axis, panel THR > 0,
fuel decreases, VGO decreases; (4) FINAL STEP — partial throttle visible /
reported, no obvious overshoot; (5) COMPLETE — VGO ~0, throttle 0, flame
gone, no latent force; (6) ABORT — fresh run, Shift+Enter or X during
ALIGN/WAIT/BURN -> ABORTED, throttle immediately and permanently zero, no
latent force or flame; (7) OFF-AXIS SAFETY — no burn while substantially
misaligned merely because the node time has passed; (8) PHYSICALITY —
acceleration matches the actual thrust / fuel spend, no velocity snap /
"magic force" look. Earlier gates: the M06-R13 closeout gates `M06-R13-H01` (normal
play: three bodies render, contract loop undisturbed, F9 works),
`M06-R13-H02` (predictor + body-2 fixture: AUTO MOONLET, F9, body-2 readouts),
and `M06-R13-H03` (predictor + body-1 fixture unchanged from R12 — regression
check) all PASSed (2026-10-05). The M06-R14 attitude-visualization gate
`M06-R14-H01` also PASSed (2026-10-05, USER). The M06-R15 node-edit
visualization gate `M06-R15-H01` FAILED its first human verification (node
graphics drift when overdue; label collisions) and was corrected by M06-R16
(implementation + automated verification COMPLETE 2026-10-05); the M06-R16-H01
re-run (2026-10-06) was a PARTIAL PASS — all R15/R16 checks passed EXCEPT one
newly observed defect: while the simulation is RUNNING, the node/trajectory
visualization visibly jitters / stair-steps (stable once paused). The remaining
defect was corrected by M06-R17 (presentation cadence: node-event geometry
at render cadence via the new `node_event_state` flight-computer helper,
12 Hz long arcs kept with an explicit `PRED AGE` staleness readout); the
M06-R17-H01 re-run (2026-10-06, USER) was a PASS — all R15/R16 checks plus
RUNNING stability, no node-edit defect remaining — and the node-edit cell
(R15 + R16 + R17) is now committed. The
still-open gates from earlier passes
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
  it is now committed and pushed to `origin`. M06-R14 (the attitude debug
  visualization) is also COMPLETE: code + headless regression implemented,
  M06-R14-H01 human-accepted (2026-10-05), and it is committed and pushed to
  `origin`. A non-blocking deferred camera/UI
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
