# Tasks

```yaml
Milestone: M06
State: ACTIVE
Phase: HUMAN_VERIFICATION
Active-Request: M06-R19
```

This is the canonical live execution ledger. It intentionally contains only
resume-critical current state. The exact 2026-10-06 pre-migration ledger is
preserved verbatim in `records/M06-pre-experience-TASKS.snapshot.md` from
source commit `508344fbfafb81f74d92964111d60bb9de954147`; use that snapshot
only for provenance or legacy-ID detail, not as active execution state.

## Task-state vocabulary

```text
[ ] OPEN
[~] IN_PROGRESS
[?] BLOCKED
[H] AWAITING_HUMAN
[x] VERIFIED
[-] SUPERSEDED
```

## Current decision boundary

M06 is active. The node-executor hardening cell (M06-R18 + M06-R19) is now
COMPLETE and committed at this checkpoint: both human gates PASS on 2026-10-08
(M06-R18-H01 re-run through M06-R19-H01, and M06-R19-H01 directly). The
magnitude-gated continuous alignment-safety fix in the canonical VGO region of
`src/autopilot.cpp` stops the executor spin (no off-axis sustained thrust; ACT
converges onto VGO; on material mid-burn separation the burn is cut to zero
throttle and the executor re-enters ALIGN, re-burns on re-align, and never
returns to WAIT after ignition); the longer PGR +4.0 / RAD +2.0 fixture
(|dv| ~4.47 m/s, ~1.12 s nominal burn) is observable with the on-panel TEST
NODE / NOM BURN / ARMED BY FIXTURE readout; the sustained burn shows coherent
fuel and VGO decrease; COMPLETE is reached with no latent thrust; a fresh
run with an explicit abort is clean and permanent. The bounded R19 re-entry
measurably relaxed the R5-V08 closed-loop approach ratio to 0.631 (HEAD
ballistic) / 0.626 (warm-start seed); per the user's decision the derived 0.60
proxy in `tests/test_transfer_warm.cpp` is SUPERSEDED by a 0.65 ceiling
(smallest bound containing the measured safe cases), while the USER-level
R5-V08 convergence requirement is NOT superseded and the R19 gate stays
mandatory (complementary guard = the node-executor whole-run invariant in
`tests/test_flight_computer.cpp`). Full `ctest` = 11/12 (sole red:
pre-existing V14-C cross-body soft-land, failing as a timeout; not a
regression, not fixed here).

The next subsystem cell is TRANSFER-COLD. Its 2026-10-08 human observation is
BLOCKED on presentation / debug observability only (NOT the solver): the panel
reports COLD PRIMARY -> COMPANION RESULT SOLVED with miss / TOF / departure /
arrival-relative-speed / TERRAIN validated / propagation count / wall time, but
the scene is not meaningfully inspectable — the COMPANION is not clearly
visible and the accepted transfer arc is not drawn for a human to inspect. The
COLD solver's automated verification remains valid; the transfer-cold cell is
NOT human-accepted on the basis of "the panel says SOLVED." This observation is
captured as a new follow-up request (M06-R20, transfer-cold visual
observability — PRESENTATION ONLY, COLD solver untouched) in the next
transition. Do NOT advance to transfer-warm until M06-R20-H01 passes. M06 is
NOT closed (inherited gates R11-H01 / R12-H01 / R12-H02 / R7-H01 remain).

2026-10-08 human follow-up on M06-R19-H01 (attempted, NOT judged): the
node-executor debug fixture's 0.5 m/s prograde-only node burns only ~0.125 s at
the 4.0 m/s^2 main accel, too short to visually judge the continuous
alignment-safety behaviour ("doesn't really run long enough; what maneuver is
it supposed to be doing?"). This is a fixture observability defect, NOT an
executor-behaviour change and NOT a pass/fail. Fix (M06-R19-D06): make the
debug-only node-executor fixture an observable mixed PGR+RAD node (dv_prograde
+4.0, dv_radial +2.0; |dv| 4.472 m/s -> ~1.12 s nominal burn) and surface the
intended maneuver on the panel; node-edit keeps its 0.5 m/s prograde node;
normal node defaults / gameplay / accel / thresholds / executor / physics are
all unchanged. M06-R19-H01 stays AWAITING_HUMAN and is re-run after the
fixture change; M06-R18-H01 stays unresolved (re-run through M06-R19-H01).
Do not self-complete or commit until the human accepts M06-R19-H01.

The baseline durable state is sufficient to resume with experience retrieval
disabled. The experience store is empty at migration; do not fabricate precedent.

## M06-R18 — node-executor observability / presentation prep

Source: USER (2026-10-06)
State: COMPLETE

- [x] M06-R18-01 Verify the "magic force" observation.
  Conclusion: CONFIRMED presentation defect, not hidden physics. The authoritative
  simulation consumes the executor's real `Input.main_throttle`; the old plume
  renderer used the manual throttle knob, so executor/midcourse/autoland thrust
  could accelerate and burn fuel with no visible plume.
  Evidence: source-path inspection recorded in the pre-migration ledger; current
  implementation routes the exact applied throttle into presentation.

- [x] M06-R18-02 Render engine plume from the actual applied main-engine input.
  Files: `src/gui.cpp`, `include/lander/sim.hpp`, `src/sim.cpp`.
  Evidence: `test_node_executor_presentation` A-E passed 2026-10-06; manual,
  node-executor, transfer-midcourse, and landing-autopilot paths share the same
  applied-input presentation source; COMPLETE/ABORT/land/crash/empty-fuel leave
  no latent plume.

- [x] M06-R18-03 Add minimal node-executor scene visualization.
  Requirement: ACT ray from actual thrust axis; VGO ray from exact
  `node_executor.dv_remaining()`; compact executor-state label; fixed-screen
  geometry; numeric panel retained; no separate guidance recomputation.
  Files: `include/lander/debug_subsystem.hpp`, `src/gui.cpp`.
  Evidence: `test_node_executor_overlay` passed 2026-10-06.

- [x] M06-R18-04 Start `--debug-subsystem node-executor` paused with
  `PAUSED FOR NODE EXECUTOR [P] RUN`, leaving the existing executor armed and
  changing no node timing, delta-v, engine, controller, physics, or normal-play
  behavior.
  Evidence: headless fixture smoke produced `ticks=0`, fuel unchanged, rc=0.

### M06-R18 preservation constraints

- M06-R18-P01 Preserve the canonical node executor: no retune/redesign of
  bang-bang attitude, alignment thresholds, ignition, burn time, VGO accounting,
  final partial throttle, node planning/basis, transfer midcourse, landing
  autopilot, physics, prediction, camera, or M07 scope unless human evidence
  creates a new persisted follow-up request.
- M06-R18-P02 Preserve the existing numeric executor panel (STATE / NODE / VGO /
  THR / FUEL / RESULT).
- M06-R18-P03 Preserve canonical source/document bindings in `src/autopilot.cpp`
  and applicable flight-guidance docs; R18 reads executor state only for display.

### M06-R18 automated verification

- [x] M06-R18-V01 Applied executor throttle drives physical input and plume even
  when the manual throttle variable is zero.
- [x] M06-R18-V02 ALIGN/WAIT own control with zero main throttle and no plume.
- [x] M06-R18-V03 BURN has applied throttle > 0, fuel decreases, VGO decreases,
  and per-step delta-v matches `main_accel * throttle * dt` against the zero-input
  reference run.
- [x] M06-R18-V04 Final partial step uses fractional throttle (~0.30 in the
  4.11 m/s regression) and presentation matches the applied fraction.
- [x] M06-R18-V05 COMPLETE and ABORT leave zero subsequent thrust/plume;
  landed/crashed/empty-fuel suppression remains.
- [x] M06-R18-V06 ACT/VGO debug geometry is fixed-screen, camera-rotation-only,
  read-only, and omits invalid/near-zero VGO.
- [x] M06-R18-V07 Build and focused suites pass; full `ctest` is 11/12.
  Sole red: pre-existing `lander_landing_tests` V14-C cross-body soft-land;
  R18 did not modify or mask it. Node-executor paused smoke and normal-game
  smoke both exit 0.

### M06-R18 human verification

- [x] M06-R18-H01 Node-executor human gate.
  Run: `./build/lander_gui --debug-subsystem node-executor`, then press `P`.
  RESULT 2026-10-06 (human): FAIL. Once engaged, the executor spins
  uncontrollably: `STATE BURN / THR 1.00` while the ACT thrust-axis ray and
  VGO ray are visibly separated. (Root-caused and resolved by M06-R19.)
  RESULT 2026-10-08 (human, re-run through M06-R19-H01): PASS — no
  uncontrolled spinning; ACT converges onto VGO; no plume/THR while
  materially misaligned; a sustained visible burn with coherent fuel and VGO
  decrease; on material mid-burn separation THR drops to 0 and the ship
  re-aligns before thrust resumes (no off-axis burn); final partial burn
  normal; COMPLETE reached with no latent thrust; fresh run P-then-X gives a
  clean permanent ABORT.
  Verify:
  1. ALIGN: ACT rotates physically toward VGO; throttle 0; no flame.
  2. WAIT: alignment held; throttle 0; no flame.
  3. BURN: starts only when aligned and at/after ignition; visible plume follows
     the actual thrust axis; panel THR > 0; fuel and VGO decrease.
  4. FINAL STEP: fractional throttle is reported/visible where observable; no
     obvious overshoot.
  5. COMPLETE: VGO ~0; throttle/plume/latent force all stop.
  6. ABORT: fresh run, Shift+Enter or X during ALIGN/WAIT/BURN -> ABORTED;
     throttle immediately and permanently zero; no latent plume/force.
  7. OFF-AXIS SAFETY: passing node time never forces a substantially misaligned
     burn.
  8. PHYSICALITY: acceleration coincides with engine thrust/fuel spend; no
     direct velocity snap or "magic force" appearance.

### M06-R18 derived implementation tasks

- [x] M06-R18-D01 Actual-applied-throttle presentation source.
- [x] M06-R18-D02 Pure node-executor overlay geometry.
- [x] M06-R18-D03 Node-executor scene overlay.
- [x] M06-R18-D04 Paused node-executor fixture/banner.
- [x] M06-R18-D05 Supplemental presentation/physicality regressions.
- [x] M06-R18-D06 Build, focused verification, ctest, smoke, durable-state gate.

## M06-R19 — node-executor burn-direction stability / continuous alignment safety

Source: USER (2026-10-06), follow-up to M06-R18-H01 (FAIL: executor spins
uncontrollably while burning — `STATE BURN / THR 1.00` with ACT and VGO rays
separated). M06-R18-H01 stays unresolved and is re-run through M06-R19-H01.

- [x] M06-R19-01 Diagnose the spin quantitatively first: deterministic fixture
  reproducing the GUI node-executor scenario with per-step records (executor
  state, VGO vector + direction, angle/omega, alignment, throttle, delivered
  impulse). Decisive evidence: a step with `BURN && !aligned &&
  main_throttle > 0`, and the VGO direction rotating during the burn. Raw
  output goes to a generated artifact; the ledger keeps only the causal
  conclusion and decisive measurements. One bounded investigative loop: no
  new request ID per trial.
  CONCLUSION (2026-10-06, headless, deterministic): REPRODUCED and root-caused.
  The exact GUI fixture (seed 1005, `dv_prograde = 0.5`, node at 5.0 s,
  ignition 4.9375 s) burns for 15,000 steps instead of 15. As the tracked VGO
  magnitude approaches zero near the end of the burn, the one-step-lagged
  nose delivers impulses that are a large fraction of the shrinking vector;
  the VGO direction rotates several rad/step, and at step 606 (t = 5.050 s)
  the vector crosses the origin (0.0336 -> 0.0039 m/s) so `normalize(VGO)`
  flips about 4.8 rad. `make_input` holds `burning` unconditionally once
  `state_ == Burn` and `after_step` has no BURN->safe transition, so the
  engine keeps firing (max THR 1.00) at the flipping target: the error pins
  near +-pi (max 178.7 deg), omega grows unbounded (max 2.31 rad/s, ~38 full
  turns), the residual regrows to 2.06 m/s, and the whole fuel tank burns
  (125 s) ending INCOMPLETE. Decisive evidence: 14,987 steps with
  `BURN && !aligned && main_throttle > 0` (first at step 606, 13 steps after
  ignition). Raw artifact: `/tmp/opencode/r19_diag.tsv` (per-step trace).
  Files touched: temporary `tests/diag_node_executor.cpp` (+ CMake target;
  removed after the fix is verified) — no behavior change.

- [x] M06-R19-02 Narrow fix: continuous alignment safety during BURN. When the
  executor becomes materially misaligned while BURN -> `main_throttle = 0`
  (physical engine off), continue normal bang-bang attitude correction toward
  the current VGO, and resume physical thrust only when alignment is safe
  again. VGO must not be discarded or rewritten while thrust is off; it
  decreases only by actually delivered impulse. Choose the smallest coherent
  state-machine change consistent with the canonical semantics (zero-throttle
  alignment hold inside BURN, or temporary re-entry to ALIGN: after nominal
  ignition ALIGN until safe / BURN while safe / ALIGN again when safety is
  lost / BURN resumes only when safe; no return to WAIT after nominal
  ignition). Do NOT fix it by snapping angle/omega, rotating VGO
  artificially, projecting delivered delta-v onto the desired VGO, ignoring
  off-axis thrust, directly modifying velocity, loosening the alignment
  threshold until the test passes, or changing physics.
  DONE (2026-10-06): magnitude-gated re-entry in `src/autopilot.cpp` — while
  the tracked VGO is within `kSmallVgoSteps` (=1.5 full-thrust steps) of zero
  and the ship is materially misaligned (outside the 0.05 rad / 0.1 rad/s
  band) the burn is cut and the executor re-enters ALIGN; it re-burns when
  re-aligned and never returns to WAIT after ignition. The re-entry is only
  armed in the small-vector regime (above it the VGO direction is stable and
  the bang-bang tracks while thrusting). Bang-bang, VGO accounting, ignition
  timing, and the final partial throttle are untouched; no state mutation. The
  0.001667 delivery floor, the 0.25 replan tolerance, and the alignment band
  are all preserved unchanged.

- [x] M06-R19-03 Regression test with the exact human failure: arm a
  maneuver, enter BURN while aligned, construct a physically reachable
  attitude-rate / VGO geometry that loses alignment mid-burn; verify
  `aligned == false` and `main_throttle == 0`, attitude still steering toward
  the current VGO, `main_throttle` may resume once realigned, VGO decreases
  only on thrust-delivered steps, no runaway VGO-direction / attitude chase,
  COMPLETE remains reachable, final partial throttle still works. Invariant
  over the whole run: `main_throttle > 0 => aligned(state)` (within a
  deterministic tolerance).
  DONE (2026-10-06): `test_node_executor_alignment_safety` in
  `tests/test_flight_computer.cpp` — scenario A (zero-gravity minimal geometry
  armed at the observed ignition state) and scenario B (the exact GUI
  node-executor fixture, seed 1005, through the authoritative Simulation).
  Both assert the magnitude-gated whole-run invariant `main_throttle > 0 =>
  aligned`, COMPLETE instead of burn-out, bounded thrust steps, bounded
  |omega|/sweep, fuel not drained, and no return to WAIT after ignition. This
  test is the complementary guard that keeps the R19 gate mandatory behind the
  R5-V08 0.65 ceiling (see M06-R19-05). Passes post-fix; the failure mode it
  targets (off-axis thrust while misaligned in the small-VGO regime) fails on
  pre-fix code.

- [x] M06-R19-04 Human observability: keep the R18 ACT/VGO rays and numeric
  panel; add one compact diagnostic only if needed (`ERR` in degrees /
  `OMEGA` in rad/s, node-executor mode only, no clutter). A human must be able
  to see ACT converging onto VGO, thrust OFF during material separation, and
  thrust resuming only when aligned.
  DONE (2026-10-06): compact ERR (deg) / OMEGA (rad/s) readout added to the
  node-executor numeric panel in `src/gui.cpp` (node-executor mode only); the
  R18 ACT/VGO rays, panel, and plume source are preserved. Together they let a
  human see ACT converging onto VGO, THR dropping to 0 during material
  separation, and thrust resuming only when aligned.

- [x] M06-R19-05 R5-V08 derived-proxy supersession (USER decision, 2026-10-06).
  The R19 continuous alignment-safety re-entry is a bounded, magnitude-gated
  zero-throttle interval in the closed-loop transfer endgame; it causally
  relaxes the R5-V08 approach ratio from the pre-R19 0.60 derived proxy to
  0.631 (HEAD ballistic) / 0.626 (warm-start seed). This is the cost of the
  higher-authority R19 safety rule, not a transfer-solver regression, and not
  a permission for general transfer degradation. Action: the derived 0.60
  executable proxy in `tests/test_transfer_warm.cpp` (the `min_target_dist <
  0.6 * start_target_dist` check) is SUPERSEDED by a 0.65 regression ceiling —
  the smallest bound containing the measured safe cases, ~0.019 margin over
  the worst (0.631) — with an in-code comment recording this supersession and
  its precondition. The USER-level R5-V08 requirement (the physical executor
  remains consistent with the planned transfer and converges toward the target
  using ordinary thrust) is NOT superseded. The 0.65 ceiling is valid only
  while the R19 gate stays mandatory; the complementary guard is the
  node-executor whole-run invariant in `tests/test_flight_computer.cpp`
  (M06-R19-03), which must keep failing on any implementation that regains
  transfer margin by re-emitting off-axis thrust. Historical 0.60 evidence and
  the causal measurements 0.626 / 0.631 are preserved here. No new request
  group: this is a resolution inside the existing M06-R19 loop.
  Measured: `lander_transfer_warm_tests` R5-V08 ratio 0.63 (min_dist 456.8 /
  start_dist 724.1, 0 retarget-induced crash, 0 land) — PASSES under 0.65.

### M06-R19 preservation constraints

- M06-R19-P01 Preserve the canonical node executor per
  `docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md`:
  VGO reduced only by actually delivered thrust impulse; desired direction =
  `normalize(VGO)`; ordinary physical attitude/thrust only; no direct state
  mutation; no substantially off-axis forced burn; final partial throttle;
  O(1) HOT; clean abort/fuel/crash/landing. Do NOT replace bang-bang with
  PID/MPC or any other controller.
- M06-R19-P02 Do not weaken existing tests; do not loosen the alignment
  threshold as the fix mechanism; do not touch physics, camera, M07 scope, or
  other cells. Do not advance to transfer-cold until this executor cell
  passes.
- M06-R19-P03 Preserve R18 work (applied-throttle plume source, ACT/VGO rays,
  paused fixture, numeric panel) and the canonical source/document bindings in
  `src/autopilot.cpp`. If the fix changes a documented algorithm, update the
  canonical document in the same work.

### M06-R19 automated verification

- [x] M06-R19-V01 Bounded diagnosis: deterministic per-step instrumentation of
  the GUI node-executor fixture reproduces the failure; decisive measurements
  recorded; raw log kept as an artifact; temporary diagnostics removed
  afterwards unless retained as bounded debug output.
  DONE: reproduced the spin (seed 1005, dv 0.5): 14,987 of 15,000 steps
  `BURN && !aligned && main_throttle > 0` after the VGO direction flipped at
  step 606; raw per-step trace captured, then the temporary
  `tests/diag_node_executor.cpp` / `tests/diag_transfer_r19.cpp` + their CMake
  targets removed after the fix was verified.
- [x] M06-R19-V02 The regression test (M06-R19-03) fails on pre-fix code and
  passes post-fix.
  DONE: `test_node_executor_alignment_safety` fails on the pre-fix executor
  (unbounded off-axis burn) and passes post-fix (both scenarios A and B).
- [x] M06-R19-V03 Whole-run invariant `main_throttle > 0 => aligned(state)`
  holds (within deterministic tolerance).
  DONE: holds over the entire post-fix run in both regression scenarios
  (magnitude-gated: asserted only in the small-VGO flip-danger regime).
- [x] M06-R19-V04 Suites: focused node-executor tests,
  `lander_flight_computer_tests`, `lander_debug_subsystem_tests`,
  `lander_predictor_tests`, full `ctest` at the 11/12 baseline (sole red
  remains the unrelated V14-C; do not fix it here). Headless smokes:
  node-executor paused `ticks=0`; normal seed 1 `ticks=237`.
  DONE: `ctest -j 8 --timeout 90` = 11/12; the sole red is `lander_landing_tests`
  (pre-existing V14-C cross-body soft-land, failing as a timeout — it hangs on
  HEAD too; not a regression, not fixed here). R5-V08 is GREEN under the
   superseded 0.65 ceiling (M06-R19-05). Node-executor focused suites pass.
- [x] M06-R19-V05 Fixture-contract regression (M06-R19-D06): a
  `lander_debug_subsystem_tests` test asserts the node-executor fixture node
  has frame == PRIMARY (0), time ~= t0 + 5 s, dv_prograde == +4.0,
  dv_radial == +2.0, is armed in the one-shot executor, and has nominal burn ==
  hypot(4,2)/main_accel ~= 1.118 s; the node-edit fixture is unchanged
  (dv_prograde 0.5, dv_radial 0.0); normal (None) gameplay has no node. The
  on-panel TEST NODE / NOM BURN values are derived from the fixture node +
  config (no duplicated magic constants). "Starts PAUSED" remains a GUI-level
  guarantee covered by the headless node-executor paused smoke (ticks=0). R19
  safety regressions (`lander_flight_computer_tests`) stay unchanged and pass.
  DONE: `test_node_executor_fixture_contract` added to
  `tests/test_debug_subsystem.cpp` (Snapshot extended with node_frame /
  node_time / node_dv_prograde / node_dv_radial / node_burn_time; `run()`
  populates them from the reference-bound fixture node and the armed
  executor's `burn_time()`). It passes: node-executor frame 0, time 5.0,
  dv 4.0/2.0, armed, `burn_time()` = hypot(4,2)/4.0 = 1.118 s; node-edit 0.5/0.0
  and not armed (cleared `burn_time()` 0.0); None has no node / no armed
  executor. `lander_debug_subsystem_tests`, `lander_flight_computer_tests`,
  `lander_transfer_warm_tests`, `lander_predictor_tests` all pass; full ctest
  11/12 (sole red = pre-existing V14-C `lander_landing_tests` timeout).

### M06-R19 human verification

- [x] M06-R19-H01 Node-executor re-gate (covers M06-R18-H01).
  Run: `./build/lander_gui --debug-subsystem node-executor` (the fixture is now
  an observable mixed PGR+RAD node, pre-armed, and starts paused).
  1. Do NOT press Enter. 2. Inspect the panel's `TEST NODE` line: PGR +4.00 /
     RAD +2.00 / DV ~4.47 m/s and `NOM BURN ~1.12 s`; confirm `EXECUTOR ARMED
     BY FIXTURE` and the `[P] RUN / [X] ABORT` legend. 3. Press `P` once.
  4. Accept only if: no uncontrolled spinning; ACT converges onto VGO; no
     plume/THR while materially misaligned; a sustained visible burn with fuel
     and VGO decreasing coherently; if ACT/VGO separate materially mid-burn,
     THR drops to 0 and the ship re-aligns before thrust resumes (no off-axis
     burn); final partial burn normal; COMPLETE reached without latent thrust.
  5. Fresh run: press `P` then `X` during execution -> clean ABORT, throttle
     immediately and permanently zero, no latent plume/force.
  RESULT 2026-10-08 (human): PASS — all acceptance items confirmed (list
  above): no spinning; ACT converges onto VGO; no off-axis sustained thrust;
  sustained burn with coherent VGO/fuel decrease; the R19 zero-throttle
  re-entry to ALIGN observed on material mid-burn separation (thrust resumes
  only when re-aligned); COMPLETE reached with no latent thrust; fresh-run
  P-then-X clean permanent ABORT. M06-R18-H01 (re-run here) also PASS.

### M06-R19 derived implementation tasks

- [x] M06-R19-D01 Diagnostic harness (temporary; per-step logger over the
  node-executor GUI fixture). Removed after verification (see M06-R19-V01).
- [x] M06-R19-D02 Continuous alignment-gating fix in the canonical VGO region
  of `src/autopilot.cpp` + canonical document update. Magnitude-gated
  re-entry to ALIGN; the `docs/...node-execution.md` "Continuous alignment
  safety" section made explicit (magnitude-gated, never in the large zone,
  continuation re-arm noted).
- [x] M06-R19-D03 Exact-failure regression test + whole-run invariant
  (`tests/test_flight_computer.cpp`); header notes it is the guard behind the
  R5-V08 0.65 ceiling.
- [x] M06-R19-D04 Compact ERR/OMEGA readout in node-executor mode
  (`src/gui.cpp`).
- [x] M06-R19-D05 Run verification (focused + ctest + smokes) and the R5-V08
  0.60 -> 0.65 supersession (`tests/test_transfer_warm.cpp`); stop uncommitted
  at M06-R19-H01.
- [x] M06-R19-D06 Fixture observability (human follow-up, 2026-10-08): make the
  debug-only node-executor fixture an observable mixed PGR+RAD node —
  `node.dv_prograde = 4.0`, `node.dv_radial = 2.0` (|dv| = 4.472 m/s -> ~1.12 s
  nominal full-throttle burn; node epoch stays ~t0+5 s, frame stays PRIMARY) —
  and surface the intended maneuver on the node-executor panel (`TEST NODE
  PGR/RAD/DV` + `NOM BURN`, `EXECUTOR ARMED BY FIXTURE`, `[P] RUN / [X] ABORT`;
  the legend no longer implies `[Return]`). Node-edit keeps its 0.5 m/s
  prograde node. DEBUG FIXTURE ONLY: no change to normal node defaults,
  gameplay, main/rotate accel, executor thresholds, the R19 safety gate, the
  node-execution algorithm, VGO accounting, physics, planner, transfer,
  landing, camera, or the paused-start behaviour. Do not re-tune R19 for this.
  DONE: (1) `src/debug_subsystem.cpp` — the shared NodeEdit/NodeExecutor dv line
  is split: the NodeExecutor case sets `node.dv_prograde = 4.0; node.dv_radial
  = 2.0`, the NodeEdit case keeps `node.dv_prograde = 0.5` (radial 0.0); node
  epoch (`default_node`, t0+5 s) and frame (PRIMARY) unchanged, executor still
  armed by the fixture at t0. (2) `src/gui.cpp` node-executor panel — added
  `TEST NODE <frame> PGR %+.2f RAD %+.2f DV %5.2f M/S`, `NOM BURN %5.2f S
  (full throttle)` (= |dv|/config().main_accel), and `EXECUTOR ARMED BY
  FIXTURE - [P] RUN  [X] ABORT`, all derived from the fixture node + config
  (no magic display constants) and guarded by `if (maneuver_node)`; the
  `[Return] exec ...` legend now reads `[P] RUN  [X] ABORT   (executor is
  pre-armed; no Return)`. The `PAUSED FOR NODE EXECUTOR [P] RUN` banner and the
  normal-gameplay help text are untouched.

## Inherited unresolved human gates

These remain unresolved and must not be inferred complete from later automated
work. Preserve them while M06 remains active (current follow-up cell:
TRANSFER-COLD / M06-R20).

- [H] M06-R11-H01 Trustworthy-diagnostics visual pass: confirm corrected
  labels/units/body-relative readouts in the debug harness.
- [H] M06-R12-H01 Static prediction-frame visual pass: fixed frames, legend,
  debug-panel frame/readout coherence.
- [H] M06-R12-H02 AUTO prediction-frame visual pass: segmented reference-frame
  path, transition markers, and no cross-frame connector.
- [H] M06-R7-H01 Consolidated M06 MVP playtest. This remains the overall
  milestone human-acceptance gate after subsystem cells are ready.

## Known unresolved / deferred issues

- `M06-R6-V14` remains incomplete because the V14-C cross-body soft-land case
  is the sole current `ctest` failure (11/12 baseline). Do not hide or weaken it;
  decide/fix it in the appropriate landing cell before M06 closeout.
- `PRED-01..PRED-08`, `SIM-COLL-01`, `TFD-1`, and `TFD-2` remain open in
  `docs/m06-predictor-physics-issues.md`. They are not silently closed by R18.
- Deferred camera/UI polish: SYSTEM-view auto-fit can zoom too far out after
  the outer moonlet expanded system bounds. This is non-blocking and was not
  fixed in R13.

## Recent accepted checkpoints relevant to resume

- M06-R13 three-body hierarchical system: human-verified and committed
  (`07f7817`).
- M06-R14 attitude debug visualization/cell: human-verified and committed
  (`f5251df`).
- M06-R15/R16/R17 node-edit visualization/corrections: human-verified and
  committed together (`c2114f3`).
- M06-R18 implementation committed pre-migration (`508344f`); its human gate
  M06-R18-H01 FAILED 2026-10-06 and is re-run through M06-R19-H01.
- M06-R18-H01 + M06-R19-H01 node-executor human gates: PASS 2026-10-08. The
  node-executor hardening cell (R18+R19) is COMPLETE and committed at the
  current checkpoint. Next subsystem cell: TRANSFER-COLD (-> M06-R20).
- 2026-10-06 durable-state migration to the experience-augmented architecture:
  committed (`f2dc61d`); exact pre-migration ledgers preserved under
  `records/M06-pre-experience-*.snapshot.md`.

## Relevant canonical/domain documents

Read selectively according to the work being changed:

- `docs/physics-model-gravity.md`
- `docs/flight-guidance-computational-rate-tiers.md`
- `docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md`
- `docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md`
- `docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md`
- `docs/m06-predictor-physics-issues.md`
- `docs/m06-subsystem-debug-harness.md`

Do not load every document by default.
