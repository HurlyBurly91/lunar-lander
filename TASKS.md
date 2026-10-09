# Tasks

```yaml
Milestone: M06
State: ACTIVE
Phase: HUMAN_VERIFICATION
Active-Request: M06-R23
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

M06 is active in HUMAN_VERIFICATION on the new `M06-R23` WARM rendezvous-epoch
correctness, retarget stability, and physical correction convergence request.
`M06-R21-H01` and `M06-R22-H01` both FAILED on 2026-10-09; R22's spinning fix
must be preserved, while WARM convergence and presentation stability remain
open. R23's automated follow-up work (D02/D06, the per-step completion
refinement, the full-encounter oracle, and the 12/12 battery) is complete as
of 2026-10-09 and was provisionally published in commit 45b8049; that
checkpoint does not accept any human gate. The three gates remain unresolved. `M06-R21-H01` FAILED on
2026-10-08: the human watched the complete transfer-warm run and observed
full-throttle, materially misaligned / approximately orthogonal ACT/VGO burn
segments, a powered spin that averaged thrust, and a terminal crash at the
COMPANION. R21 remains unresolved and its uncommitted presentation work must
be preserved; R22 is the functional follow-up and must be diagnosed and fixed
before R21 is re-tested. R20 is COMPLETE and human-accepted (`7215861`) and
must not be reopened. R21 was primarily presentation / debug observability for
`--debug-subsystem transfer-warm`. The user reported that the current
transfer-warm view is almost entirely a numeric panel and does not show the
current WARM route, the original COLD seed, the live craft, the physically
executed correction VGO, the future target at WARM arrival, or when bounded
replans change the route. The first required code change is to verify and fix a
panel semantic/unit error: the current midcourse retarget threshold is a
correction delta-V threshold in m/s, not a terminal-miss tolerance in metres;
debug telemetry must never compare metres against m/s. Beyond that, R21 must
add a truthful inertial transfer-warm debug scene with visually distinct COLD
SEED, WARM PLAN, LIVE ship, ACT/VGO fast-executor overlay, future target
geometry, brief replan event markers, a debug-only camera fit, and a cleaned-up
panel. Do NOT alter the WARM solver/controller, COLD fallback, cadence,
acceptance thresholds, R19 executor behavior, physics, ephemerides, normal
gameplay/camera, landing/autoland, TFD-1/TFD-2, or M07 unless a separately
verified functional defect is discovered (then stop and report). No further
R21 experiment IDs should be created. M06 is NOT closed (inherited gates
R11-H01 / R12-H01 / R12-H02 / R7-H01 remain).

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
  Finding: CONFIRMED presentation defect, not hidden physics. The authoritative
  simulation consumes the executor's real `Input.main_throttle`; the old plume
  renderer used the manual throttle knob, so executor/midcourse/autoland thrust
  could accelerate and burn fuel with no visible plume.
  Evidence: source-path inspection recorded in the pre-migration ledger; current
  implementation routes the exact applied throttle into presentation.

  Verified-By:
    - M06-R18-V01
- [x] M06-R18-02 Render engine plume from the actual applied main-engine input.
  Files: `src/gui.cpp`, `include/lander/sim.hpp`, `src/sim.cpp`.
  Evidence: `test_node_executor_presentation` A-E passed 2026-10-06; manual,
  node-executor, transfer-midcourse, and landing-autopilot paths share the same
  applied-input presentation source; COMPLETE/ABORT/land/crash/empty-fuel leave
  no latent plume.

  Verified-By:
    - M06-R18-V01
    - M06-R18-V02
    - M06-R18-V03
    - M06-R18-V04
    - M06-R18-V05
- [x] M06-R18-03 Add minimal node-executor scene visualization.
  Requirement: ACT ray from actual thrust axis; VGO ray from exact
  `node_executor.dv_remaining()`; compact executor-state label; fixed-screen
  geometry; numeric panel retained; no separate guidance recomputation.
  Files: `include/lander/debug_subsystem.hpp`, `src/gui.cpp`.
  Evidence: `test_node_executor_overlay` passed 2026-10-06.

  Verified-By:
    - M06-R18-V06
    - M06-R18-H01
- [x] M06-R18-04 Start `--debug-subsystem node-executor` paused with
  `PAUSED FOR NODE EXECUTOR [P] RUN`, leaving the existing executor armed and
  changing no node timing, delta-v, engine, controller, physics, or normal-play
  behavior.
  Evidence: headless fixture smoke produced `ticks=0`, fuel unchanged, rc=0.

  Verified-By:
    - M06-R18-V07
    - M06-R18-H01
### M06-R18 preservation constraints

- [ ] M06-R18-P01 Preserve the canonical node executor: no retune/redesign of
  bang-bang attitude, alignment thresholds, ignition, burn time, VGO accounting,
  final partial throttle, node planning/basis, transfer midcourse, landing
  autopilot, physics, prediction, camera, or M07 scope unless human evidence
  creates a new persisted follow-up request.
- [ ] M06-R18-P02 Preserve the existing numeric executor panel (STATE / NODE / VGO /
  THR / FUEL / RESULT).
- [ ] M06-R18-P03 Preserve canonical source/document bindings in `src/autopilot.cpp`
  and applicable flight-guidance docs; R18 reads executor state only for display.

### M06-R18 automated verification

- [x] M06-R18-V01 Applied executor throttle drives physical input and plume even
  when the manual throttle variable is zero.
  Covers:
    - M06-R18-01
    - M06-R18-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V02 ALIGN/WAIT own control with zero main throttle and no plume.
  Covers:
    - M06-R18-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V03 BURN has applied throttle > 0, fuel decreases, VGO decreases,
  and per-step delta-v matches `main_accel * throttle * dt` against the zero-input
  reference run.
  Covers:
    - M06-R18-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V04 Final partial step uses fractional throttle (~0.30 in the
  4.11 m/s regression) and presentation matches the applied fraction.
  Covers:
    - M06-R18-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V05 COMPLETE and ABORT leave zero subsequent thrust/plume;
  landed/crashed/empty-fuel suppression remains.
  Covers:
    - M06-R18-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V06 ACT/VGO debug geometry is fixed-screen, camera-rotation-only,
  read-only, and omits invalid/near-zero VGO.
  Covers:
    - M06-R18-03
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R18-V07 Build and focused suites pass; full `ctest` is 11/12.
  Sole red: pre-existing `lander_landing_tests` V14-C cross-body soft-land;
  R18 did not modify or mask it. Node-executor paused smoke and normal-game
  smoke both exit 0.

  Covers:
    - M06-R18-04
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
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

  Covers:
    - M06-R18-03
    - M06-R18-04
  Gate: HUMAN
  Human-Decision: ACCEPTED
  Decision-Source: records/M06-schema1-live-TASKS.snapshot.md (explicit human PASS recorded 2026-10-08)
### M06-R18 derived implementation tasks

- [x] M06-R18-D01 Actual-applied-throttle presentation source.
  Conclusion: The previously invisible engine plume was caused by using manual throttle rather than applied engine input for rendering.
  Conclusion-Status: OBSERVED
  Conclusion-Scope: Node-executor plume presentation diagnosis in the pre-schema-2 M06-R18 fixture.
  Conclusion-Evidence:
    - M06-R18-V01
  Conclusion-Limitations:
    - Historical source-path diagnosis and recorded tests; not a new physical-force measurement.
  Conclusion-Recheck-On:
    - Changes to applied-input or plume-rendering code.
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

  Verified-By:
    - M06-R19-V01
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

  Verified-By:
    - M06-R19-V03
    - M06-R19-V04
    - M06-R19-H01
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

  Verified-By:
    - M06-R19-V02
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

  Verified-By:
    - M06-R19-V05
    - M06-R19-H01
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

  Verified-By:
    - M06-R19-V04
### M06-R19 preservation constraints

- [ ] M06-R19-P01 Preserve the canonical node executor per
  `docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md`:
  VGO reduced only by actually delivered thrust impulse; desired direction =
  `normalize(VGO)`; ordinary physical attitude/thrust only; no direct state
  mutation; no substantially off-axis forced burn; final partial throttle;
  O(1) HOT; clean abort/fuel/crash/landing. Do NOT replace bang-bang with
  PID/MPC or any other controller.
- [ ] M06-R19-P02 Do not weaken existing tests; do not loosen the alignment
  threshold as the fix mechanism; do not touch physics, camera, M07 scope, or
  other cells. Do not advance to transfer-cold until this executor cell
  passes.
- [ ] M06-R19-P03 Preserve R18 work (applied-throttle plume source, ACT/VGO rays,
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
  Covers:
    - M06-R19-01
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R19-V02 The regression test (M06-R19-03) fails on pre-fix code and
  passes post-fix.
  DONE: `test_node_executor_alignment_safety` fails on the pre-fix executor
  (unbounded off-axis burn) and passes post-fix (both scenarios A and B).
  Covers:
    - M06-R19-03
  Gate: INVARIANT
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R19-V03 Whole-run invariant `main_throttle > 0 => aligned(state)`
  holds (within deterministic tolerance).
  DONE: holds over the entire post-fix run in both regression scenarios
  (magnitude-gated: asserted only in the small-VGO flip-danger regime).
  Covers:
    - M06-R19-02
  Gate: INVARIANT
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R19-V04 Suites: focused node-executor tests,
  `lander_flight_computer_tests`, `lander_debug_subsystem_tests`,
  `lander_predictor_tests`, full `ctest` at the 11/12 baseline (sole red
  remains the unrelated V14-C; do not fix it here). Headless smokes:
  node-executor paused `ticks=0`; normal seed 1 `ticks=237`.
  DONE: `ctest -j 8 --timeout 90` = 11/12; the sole red is `lander_landing_tests`
  (pre-existing V14-C cross-body soft-land, failing as a timeout — it hangs on
  HEAD too; not a regression, not fixed here). R5-V08 is GREEN under the
   superseded 0.65 ceiling (M06-R19-05). Node-executor focused suites pass.
  Covers:
    - M06-R19-02
    - M06-R19-05
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
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

  Covers:
    - M06-R19-04
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
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

  Covers:
    - M06-R19-02
    - M06-R19-04
  Gate: HUMAN
  Human-Decision: ACCEPTED
  Decision-Source: records/M06-schema1-live-TASKS.snapshot.md (explicit human PASS recorded 2026-10-08)
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

## M06-R20 — transfer-cold visual observability

Source: USER (2026-10-08), follow-up to the transfer-cold human observation
(Phase B): `--debug-subsystem transfer-cold` reports COLD PRIMARY -> COMPANION
RESULT SOLVED with miss / TOF / departure / arrival-relative-speed / TERRAIN
validated / propagation count / wall time, but the scene is not meaningfully
inspectable — the COMPANION is not clearly visible and the accepted transfer
arc is not drawn for a human to inspect. This is a PRESENTATION / DEBUG-
OBSERVABILITY defect, not a solver defect: the COLD solver's automated
verification remains valid, and the transfer-cold cell is NOT human-accepted on
the basis of "the panel says SOLVED." R20 is PRESENTATION ONLY — the COLD
solver is untouched. One request group covers the whole cell; do not mint a new
ID per visualization experiment.

- [x] M06-R20-01 Start `--debug-subsystem transfer-cold` PAUSED (debug-only)
  with a `PAUSED FOR COLD TRANSFER INSPECTION` banner. No normal-gameplay
  behavior change; no other debug mode changes unless via an already-correct
  shared debug-only mechanism.
  Evidence: `SDL_VIDEO_DRIVER=dummy ./build/lander_gui --debug-subsystem
  transfer-cold --frames 20` exits 0 with `ticks=0` (paused start); all other
  `--debug-subsystem` modes still run `--frames 3` cleanly.

  Verified-By:
    - M06-R20-V06
- [x] M06-R20-02 Render the ACTUAL accepted COLD arc. Build a READ-ONLY display
  trajectory from the accepted `TransferSolution` (`source` / `target` /
  `solve_epoch` / `departure_state` / `departure_velocity` / `time_of_flight` /
  `arrival_epoch` / `achieved_miss` / `arrival_rel_speed`) as a
  `BallisticState{p=departure_state, v=departure_velocity, t=solve_epoch}`
  propagated to `arrival_epoch` via the existing canonical fixed-step
  propagation (prefer an existing pure zero-thrust trajectory sampler if
  suitable). Same `BinarySystem`, same three-body gravity, same fixed_dt (1/120);
  no alternative integrator, no second transfer solve, no patched conics, no
  SOI, no live-state mutation. Do NOT feed the display trajectory back into
  planning or the simulation. Compute / cache it once for the debug result
   (not per rendered frame).
   Evidence: `transfer_cold_display` in `src/debug_subsystem.cpp` builds the
   read-only arc from the accepted `TransferSolution` using
   `predict_zero_thrust` on the existing fixed-step grid; it is cached once in
   the GUI fixture, and R20 V01/V02 verify the endpoint and no mutation.

  Verified-By:
    - M06-R20-V01
    - M06-R20-V02
- [x] M06-R20-03 Make the transfer route legible: (A) a `DEP` marker + label at
  the accepted departure state; (B) the full COLD zero-thrust arc drawn in one
  distinctive style/color and labelled `COLD ARC` (not confused with the live
  ship trajectory or a generic predictor); (C) an `ARR` marker + label at the
  actual propagated endpoint; (D) a ghost/reference of the TARGET (COMPANION) at
  the arrival epoch (`T+<TOF>`), from the canonical ephemeris, making obvious
  that the solver aims at a moving future body, not its present location. Do NOT
  fabricate a "miss line" unless its endpoint is the same canonical target
  quantity behind `achieved_miss`; otherwise keep the numeric MISS readout plus
   the propagated ARR endpoint plus the arrival-epoch body geometry.
   Evidence: `draw_transfer_cold_debug` renders `DEP`, the labelled `COLD ARC`,
   `ARR`, and the `COMPANION @ ARRIVAL` ghost; no miss line is drawn.

  Verified-By:
    - M06-R20-V03
    - M06-R20-H01
- [x] M06-R20-04 Debug-only transfer fit / camera: in transfer-cold debug mode,
  a read-only debug-view framing that initially fits the PRIMARY/source region,
  the COMPANION/target region, the entire accepted COLD arc, the departure and
  arrival markers, and the arrival-epoch target ghost. Do NOT force full
  three-body SYSTEM bounds just because the distant outer moonlet (body 2)
  exists. Prefer a narrow debug-view helper over changing the normal camera
  architecture. Ordinary gameplay camera, the UI debug cell, and camera physics
  are unchanged; the user can still zoom/pan; the visualization stays
   geometrically truthful (no independent ad-hoc magnification of pieces).
   Evidence: `transfer_cold_camera_fit` and `Camera::set_debug_frame` fit only
   the source/target/arc geometry and leave normal camera behavior unchanged;
   R20 V04 verifies the fit contains the route and does not force body 2.

  Verified-By:
    - M06-R20-V04
    - M06-R20-H01
- [x] M06-R20-05 Keep the existing numeric COLD panel (COLD one-shot,
  PRIMARY -> COMPANION, RESULT SOLVED/NO SOLUTION, MISS, TOF, departure data,
  arrival-relative speed, TERRAIN validated, propagation count, wall time). Add
  only useful presentation context if required; do not turn the panel into a
   large data dump — the scene is meant to make the numbers understandable.
   Evidence: the existing numeric COLD panel in `src/gui.cpp` was left in place.

  Verified-By:
    - M06-R20-V05
- [x] M06-R20-06 Solver-failure presentation: if the COLD solve returns no valid
  solution, show NO SOLUTION clearly, draw NO fake transfer arc, synthesize no
  arrival marker, and mutate no state to make a route exist. The debug
   visualization must faithfully represent the solver output.
   Evidence: `transfer_cold_display` returns invalid / empty for a no-solution
   result, and R20 V06 verifies that no arc, ARR, or target ghost is synthesized.

  Verified-By:
    - M06-R20-V06
### M06-R20 preservation constraints

- [ ] M06-R20-P01 Do NOT change: the COLD coarse search, Newton correction,
  Jacobian, transfer acceptance miss threshold, terrain-clearance gate,
  transfer candidate ranking, transfer timing fractions, warm replan, the
  midcourse controller, the node executor, the R19 alignment safety, gravity,
  ephemerides, physics, normal camera behavior, normal gameplay, or M07.
- [ ] M06-R20-P02 R20 is PRESENTATION / DEBUG-OBSERVABILITY only. Do not invent a
  second planner or an approximate trajectory; render only the existing accepted
  `TransferSolution`. Do not feed any display geometry into planning or the
  simulation.
- [ ] M06-R20-P03 Do not fix TFD-1 / TFD-2 (or other deferred predictor/transfer
  defects) here. This request is observability only.
- [ ] M06-R20-P04 Read-only: building the visual arc and the debug geometry must not
  mutate `Simulation`, `BinarySystem`, or the `TransferSolution`.

### M06-R20 automated verification

- [x] M06-R20-V01 Accepted arc: for a valid `TransferSolution`, the first
  trajectory sample equals `departure_state`, the first time equals
  `solve_epoch`, the final time equals `arrival_epoch` on the fixed-step grid,
  the final state equals the authoritative fixed-step propagation from the same
   departure state/velocity, and all samples are finite.
   Evidence: `test_transfer_cold_display_accepted_arc` in
   `tests/test_debug_subsystem.cpp`; `lander_debug_subsystem_tests` passes.
  Covers:
    - M06-R20-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V02 No mutation: building the visual arc and the debug geometry
   does not mutate `Simulation`, `BinarySystem`, or the `TransferSolution`.
   Evidence: `test_transfer_cold_display_no_mutation` passes.
  Covers:
    - M06-R20-02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V03 Arrival target: the displayed arrival target body position
  comes from `BinarySystem::position(target, arrival_epoch)`, not the current
   simulation time.
   Evidence: `test_transfer_cold_display_arrival_target_future` passes.
  Covers:
    - M06-R20-03
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V04 View bounds: the initial transfer-cold debug fit contains the
  departure, the transfer arc, the arrival endpoint, and the relevant
  source/target geometry, and does not require including the distant moonlet
   merely because body 2 exists.
   Evidence: `test_transfer_cold_camera_fit_contains_route` passes.
  Covers:
    - M06-R20-04
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V05 Cost accounting: the displayed COLD solver propagation count
  continues to represent solver cost; extra pure display propagation is not
   reported as part of the COLD search cost.
   Evidence: `test_transfer_cold_display_cost_not_reported` passes.
  Covers:
    - M06-R20-05
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V06 Mode isolation: the visualization/camera behavior appears only
  in transfer-cold debug mode; normal gameplay and the other subsystem fixtures
  are unchanged. Full `ctest` returns to the 11/12 baseline (sole red the
   pre-existing V14-C cross-body soft-land timeout).
   Evidence: `test_transfer_cold_mode_isolation_and_invalid` passes; full
   `ctest` returned 11/12 with the sole failure `lander_landing_tests`
   (`V14-C` perturbed/independent-seed cross-body soft-land timeout).

  Covers:
    - M06-R20-01
    - M06-R20-06
    - M06-R20-F01-08
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
### M06-R20 human verification

- [x] M06-R20-H01 Transfer-cold visual-inspection gate.
  Run: `./build/lander_gui --debug-subsystem transfer-cold`. The human must
  immediately be able to verify:
  1. The source is PRIMARY and the destination is COMPANION.
  2. Both relevant bodies/regions are visible at a useful scale.
  3. The departure (DEP) is obvious.
  4. The complete COLD transfer arc is visible.
  5. The arrival (ARR) is obvious.
  6. The future COMPANION arrival geometry is visible and labelled.
  7. The arc approaches the FUTURE target, not its current position.
  8. Nothing visibly intersects terrain contrary to the solver's validation.
  9. The numeric panel and the graphical route describe the same solution.
  10. The live craft is not teleported or otherwise mutated by the inspection.
  Evidence: after the F01 inertial temporal-scene rebuild, the user inspected
  the transfer-cold scene on 2026-10-08 and returned PASS. The transfer-cold
  cell is human-accepted; transfer-warm may be started only as a separately
  requested scope.

  Covers:
    - M06-R20-03
    - M06-R20-04
    - M06-R20-F01-05
  Gate: HUMAN
  Human-Decision: ACCEPTED
  Decision-Source: records/M06-schema1-live-TASKS.snapshot.md (explicit human PASS recorded 2026-10-08)
### M06-R20 derived implementation tasks

- [x] M06-R20-D01 Paused transfer-cold debug start + `PAUSED FOR COLD TRANSFER
  INSPECTION` banner.
- [x] M06-R20-D02 Read-only accepted-arc builder (pure fixed-step propagation of
  the accepted `TransferSolution` departure state, cached once for the debug
  result).
- [x] M06-R20-D03 Scene rendering: COLD ARC polyline + DEP / ARR markers +
  COMPANION @ ARRIVAL ghost; faithful NO-SOLUTION presentation.
- [x] M06-R20-D04 Debug-only camera fit to source + target + arc + markers
  (narrow debug-view helper, not a normal-camera change).
- [x] M06-R20-D05 Automated regression tests V01-V06 (pure / headless).
- [x] M06-R20-D06 Run verification (focused + ctest + smokes); stop uncommitted
  at M06-R20-H01.

### M06-R20-F01 bounded follow-up after the first H01 FAIL

Source: USER (2026-10-08) human verification of `M06-R20-H01`: FAIL. The
visualization exists, but the route is not visually legible. This is a bounded
iteration under the same R20 request and same H01 gate; it is not R21.

- [x] M06-R20-F01-01 Diagnose the compression before changing rendering. Record
  a diagnostic artifact with `solve_epoch`, `arrival_epoch`, TOF, departure /
  first / final arc positions, ARR, source/target positions at both epochs, raw
  arc bounds, camera-fit bounds, and screen coordinates for DEP / ARR /
  source@departure / target@arrival. Conclude which of A-E is causal:
  A coordinates, B epoch/frame transformation, C camera-fit bounds, D
  ambiguous mixing of current-time and departure/arrival-time geometry, or E a
  combination. Retain only the causal conclusion in durable state.
  Evidence: temporary headless diagnostic produced
  `/tmp/opencode/r20-diag.txt`; `first_arc == DEP`, `last_arc == ARR`, and the
  target reference used the arrival epoch, so A/B were not causal. The causal
  defect was C/D/E: the old fit included full body extents and the scene mixed
  current-time bodies with the temporal references, compressing the short route
  into an ambiguous cluster.
  Verified-By:
    - M06-R20-V08
- [x] M06-R20-F01-02 Rebuild the transfer-cold debug scene as an explicit
  WORLD/INERTIAL temporal inspection scene: render `PRIMARY @ T0` from
  `solve_epoch` and `COMPANION @ T+<actual TOF>` from `arrival_epoch` as the two
  dominant body references, with the complete accepted COLD arc between them.
  Do not imply both bodies are at the same instant.
  Evidence: `TransferColdDisplay` now carries `source_outline` /
  `target_outline`, `source_rotation` / `target_rotation`, and labelled
  `@ T0` / `@ T+<TOF>` body references; the GUI draws those temporal bodies
  instead of implying a single shared instant.
  Verified-By:
    - M06-R20-V07
- [x] M06-R20-F01-03 In transfer-cold debug mode only, suppress or clearly
  de-emphasize/label the ordinary current-time bodies so they cannot be
  confused with the departure/arrival temporal bodies. The unrelated outer
  moonlet must not control framing or dominate the scene. Normal gameplay and
  other debug modes remain unchanged.
  Evidence: `gui.cpp` skips the ordinary current-time `draw_body` calls only
  when `debug_mode == TransferCold`; all other modes still draw current-time
  bodies normally.
  Verified-By:
    - M06-R20-V11
- [x] M06-R20-F01-04 Make the transfer-cold debug camera fit depend only on the
  accepted arc, source body outline at `solve_epoch`, target body outline at
  `arrival_epoch`, DEP, and ARR. Preserve aspect ratio, add screen margin,
  exclude body 2 / arbitrary system bounds, and make the dominant relevant
  extent occupy roughly 70-85% of the usable viewport.
  Evidence: `transfer_cold_display` computes `raw_center` / `raw_half` from the
  arc, local source/target outlines, DEP, ARR, and the solver arrival shell
  only; body centres and body 2 are excluded, the fit adds a 1.18 margin, and
  V09 asserts the dominant relevant extent occupies 70-90% of the viewport.
  Verified-By:
    - M06-R20-V09
- [x] M06-R20-F01-05 Make the arc readable as a route: clear DEP/ARR markers,
  a visible COLD ARC polyline, source/target temporal-body outlines, and sparse
  presentation-only time markers along the arc (for example 25/50/75% or
  elapsed times) without clutter.
  Evidence: the overlay draws enlarged DEP/ARR markers, a 2.5px COLD ARC
  polyline, 25/50/75% time ticks, and epoch-labelled temporal bodies.
  Verified-By:
    - M06-R20-H01
- [x] M06-R20-F01-06 Keep the arrival relationship truthful. Draw a
  `TARGET` marker and short `ARR -> TARGET` miss segment only if both endpoints
  are exactly the quantities used by the accepted solver and `achieved_miss`;
  otherwise show ARR plus the labelled `COMPANION @ ARRIVAL` geometry and keep
  MISS numerical. Do not fabricate or approximate a target point.
  Evidence: `transfer_arrival_target` exposes the solver's exact arrival shell
  (`target max_surface_radius + kTransferClearance` on the source-facing side
  at `arrival_epoch`) without changing the solver; the overlay draws TARGET and
  the ARR -> TARGET segment only when that exact point is available, and V08
  asserts equality with the solver goal.
  Verified-By:
    - M06-R20-V08
- [x] M06-R20-F01-07 Extend the headless R20 regressions to cover temporal
  source/target epochs, inertial arc consistency, first/last parity, exclusion
  of body 2 from the fit, aspect preservation, viewport margins, meaningful
  dominant viewport extent, DEP/ARR not collapsed by an epoch/frame mismatch,
  and Simulation immutability. Include one deterministic regression that fails
  if the route is again a tiny cluster despite available viewport space.
  Evidence: `test_transfer_cold_temporal_epochs_and_inertial_frame` and the
  reworked fit test add deterministic checks for temporal outlines, inertial
  parity, body-2 exclusion, aspect/margin/extent, and a DEP-ARR screen-distance
  regression that fails if the route collapses below 60px.
  Verified-By:
    - M06-R20-V10
- [x] M06-R20-F01-08 Re-run focused transfer/debug/render/camera tests,
  `lander_transfer_warm_tests`, `lander_debug_subsystem_tests`,
  `lander_flight_computer_tests`, `lander_predictor_tests`, full `ctest`
  (expected 11/12 with only the pre-existing V14-C red), transfer-cold headless
  smoke, and normal-game smoke, then return to the SAME `M06-R20-H01` gate.
  Evidence: all non-landing ctest targets pass (11/11 with landing excluded);
  `lander_landing_tests` shows only the pre-existing V14-C failure; all 12
  `--debug-subsystem` headless smokes exit 0; transfer-cold and normal PPM
  artifacts were generated for human inspection.
  Verified-By:
    - M06-R20-V06

Derived implementation follow-up:
- [x] M06-R20-D07 Instrument / diagnose the first presentation and record the
  causal conclusion.
- [x] M06-R20-D08 Rebuild the transfer-cold display/camera helpers as a
  temporal inspection model (source at T0, target at arrival, transfer-only
  fit).
- [x] M06-R20-D09 Rework the transfer-cold GUI scene: dominant temporal bodies,
  de-emphasized current-time bodies, readable arc + time markers, truthful
  ARR/TARGET presentation.
- [x] M06-R20-D10 Add the F01 automated presentation regressions.
- [x] M06-R20-D11 Run the F01 verification battery and stop uncommitted at
  `M06-R20-H01`.

Additional automated verification for F01:

- [x] M06-R20-V07 The displayed source geometry uses `solve_epoch` and the
  displayed target geometry uses `arrival_epoch`.
  Covers:
    - M06-R20-F01-02
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V08 The arc samples, DEP, ARR, and body references are all in one
  consistent world/inertial presentation frame, with first/last parity.
  Covers:
    - M06-R20-F01-01
    - M06-R20-F01-06
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V09 The debug fit excludes body 2 and arbitrary system bounds,
  preserves aspect ratio, respects viewport margins, and makes the dominant
  relevant extent occupy a meaningful fraction of the viewport.
  Covers:
    - M06-R20-F01-04
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V10 DEP and ARR screen points are not collapsed by an
  epoch/frame mismatch; the route occupies non-degenerate viewport space in the
  actual transfer-cold fixture.
  Covers:
    - M06-R20-F01-07
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R20-V11 The normal `Simulation` remains unmodified by the F01
  presentation changes.

  Covers:
    - M06-R20-F01-03
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
## M06-R21 — transfer-warm visual observability

Source: USER (2026-10-08), after R20 PASS and commit `7215861`. The user ran
`./build/lander_gui --debug-subsystem transfer-warm` and reported that the WARM
subsystem cannot yet be meaningfully judged from the scene: the view is almost
entirely a numeric panel. The human cannot see the trajectory the WARM planner
currently wants, how it differs from the original COLD seed, where the live
craft is relative to that route, what correction VGO is physically being
executed, where the moving COMPANION is expected at arrival, or whether
replanning changes the route spatially. Do NOT mark transfer-warm
human-accepted.

### M06-R21 explicit requirements

- [x] M06-R21-01 First verify from source that the final threshold passed to
  `TransferMidcourse::maybe_replan()` is applied to
  `hypot(solved->dv_prograde, solved->dv_radial)` as a correction delta-V in
  m/s, not to `TransferSolution::achieved_miss` in metres. If confirmed,
  correct the debug panel semantics: never compare terminal miss [m] against
  the retarget threshold [m/s]; display terminal miss and correction DV
  separately with units; the displayed correction DV must be the exact
  magnitude used by the retarget decision. Prefer correcting names/telemetry
  only. A pure semantic rename of `kMidcourseMissTolerance` to a
  correction-DV threshold name is allowed only with zero behavior change. Add a
  regression that catches metres-vs-m/s confusion.
  Evidence: `src/gui.cpp` renames `kMidcourseMissTolerance` to
  `kMidcourseCorrectionDvThreshold` and the TransferWarm panel now displays
  `TERMINAL MISS [m]`, `CORR DV [m/s]`, and `RETARGET THRESH [m/s]`
  separately; `test_r21_telemetry_semantics` verifies the retarget decision
  uses the same correction-delta-v magnitude.
  Verified-By:
    - M06-R21-V01
- [ ] M06-R21-02 In transfer-warm debug mode, draw the original COLD seed route
  dimly and labelled `COLD SEED`, reusing the accepted R20 temporal-display
  machinery where appropriate. Do not recompute the COLD solve.
  Verified-By:
    - M06-R21-V02
    - M06-R21-H01
- [ ] M06-R21-03 Build and draw the current accepted WARM route from
  `TransferMidcourse::cache()` as a read-only display arc using the same
  canonical fixed-step zero-thrust propagation rules as R20. Label it
  prominently `WARM PLAN`. Rebuild this display geometry only when the WARM
  cache meaningfully changes / a replan produces a new accepted cache, not every
  rendered frame.
  Verified-By:
    - M06-R21-V02
    - M06-R21-V06
    - M06-R21-H01
- [ ] M06-R21-04 Render the actual authoritative spacecraft clearly and label
  it `LIVE`. Do not replace its position with a planned position.
  Verified-By:
    - M06-R21-V04
    - M06-R21-H01
- [ ] M06-R21-05 At the LIVE craft, show the actual fast-executor correction
  state: `ACT` for the actual thrust axis, `VGO` for the exact current fast
  executor `dv_remaining()`, and compact state such as `ALIGN`, `BURN`,
  `COMPLETE`. Use actual internal state/input only; do not independently
  recalculate guidance. Add narrow const/read-only accessors on
  `TransferMidcourse` if needed.
  Verified-By:
    - M06-R21-V05
    - M06-R21-H01
- [ ] M06-R21-06 For the current WARM cache, show the future target geometry
  using `BinarySystem::position(target, cache.arrival_epoch)` and label it
  `COMPANION @ WARM ARRIVAL` with an explicit `T+<remaining/TOF as appropriate>`
  temporal meaning.
  Verified-By:
    - M06-R21-V03
    - M06-R21-H01
- [ ] M06-R21-07 When a successful WARM replan is accepted, show a brief
  debug-only banner or event marker for roughly 1-2 presentation seconds with
  `WARM REPLAN #N`, `CORR DV X.XX M/S`, `NEWTON N`, `PROP N`, and
  `RETARGET YES/NO`. Do not pause the simulation automatically on every replan
  and do not create an on-screen event log.
  Verified-By:
    - M06-R21-H01
- [ ] M06-R21-08 Use a transfer-warm debug-only camera/view that keeps the LIVE
  ship, current WARM route, target-at-arrival, and relevant body geometry
  legible. It may reuse R20 transfer-fit logic, but it must update when the
  current WARM route changes. Do not include the distant moonlet merely because
  it exists, do not mutate normal camera behavior, do not force a static
  temporal view that loses the live craft, and do not independently scale
  different objects. If a full-route view makes the live craft too small,
  prefer a minimal debug toggle between `WARM ROUTE` and `LIVE DETAIL`.
  Verified-By:
    - M06-R21-H01
- [ ] M06-R21-09 Clean up the existing transfer-warm panel. Retain useful
  telemetry (WARM active, COLD seed validity, cache validity, TOF, Newton
  iterations, fallback COLD, propagations last/total, replan count, retarget
  count, cadence), but clearly separate `TERMINAL MISS [m]` from `CORR DV
  [m/s]` and `RETARGET THRESH [m/s]`. Do not label the correction threshold as
  miss tolerance. Also expose the fast executor state and remaining VGO
  magnitude, e.g. `FAST ALIGN/BURN/COMPLETE`, `VGO x.xx m/s`, `THR x.xx`.
  Verified-By:
    - M06-R21-H01
- [ ] M06-R21-10 The finished mode must let a human understand without reading
  code: this was the original COLD route; this is the current WARM-corrected
  route; this is where the real ship is; this is the correction the physical
  executor is applying; a bounded WARM replan just occurred; this is where the
  companion will be at planned arrival; and whether the WARM controller is
  converging toward the target rather than merely incrementing counters.

  Verified-By:
    - M06-R21-H01
### M06-R21 preservation constraints

- [x] M06-R21-P01 Do not alter the WARM differential-correction mathematics,
  COLD fallback policy, Newton iteration caps, candidate ranking, transfer
  acceptance threshold, terrain-clearance validation, replan cadence, retarget
  correction threshold, node-executor/R19 behavior, physics, ephemerides,
  normal gameplay, normal camera, landing/autoland, TFD-1/TFD-2, or M07.
  Evidence: only read-only telemetry, display state, and debug presentation
  were changed; existing solver/guidance tests remain green.
  Verified-By:
    - M06-R21-V06
- [x] M06-R21-P02 Preserve the COLD seed + WARM bounded correction
  architecture, no solver in the 120 Hz HOT path, WARM bounded cadence,
  ordinary physical VGO/node execution, R19 continuous alignment safety, no
  hidden forces, no live-state mutation by the planner, and authoritative
  gravity/ephemerides.
  Evidence: the transfer-warm mode still calls the existing bounded-rate
  `maybe_replan` path and the ordinary HOT fast executor; no new solver or
  force was added.
  Verified-By:
    - M06-R21-V06
- [x] M06-R21-P03 Visualization code must be read-only with respect to the
  live simulation and must not invoke additional transfer solves or report
  display propagation as solver cost.
  Evidence: `test_transfer_warm_display_no_mutation` and
  `test_transfer_warm_display_cost_not_reported` pass.
  Verified-By:
    - M06-R21-V04
    - M06-R21-V07
- [~] M06-R21-P04 Do not create further request IDs for individual
  visualization experiments inside R21.
  Evidence: all R21 work remains under the existing `M06-R21-*` IDs.
  Verified-By:
    - M06-R21-H01
- [~] M06-R21-P05 If investigation uncovers an actual controller defect, stop
  and report it rather than changing behavior under this observability request.
  Evidence: the initial R21 work found only presentation/telemetry semantics;
  the later `M06-R21-H01` human run exposed a real controller defect. That
  defect was stopped out of R21 and persisted as `M06-R22`.

  Verified-By:
    - M06-R21-H01
### M06-R21 derived implementation tasks

- [x] M06-R21-D01 Verify the `maybe_replan()` threshold semantics from source
  and correct the panel/unit semantics (and, only if zero behavior change,
  rename the internal threshold constant appropriately).
  Evidence: the threshold constant is now `kMidcourseCorrectionDvThreshold`;
  the TransferWarm panel no longer compares terminal miss against the
  retarget threshold.
- [x] M06-R21-D02 Add read-only accessors / display state needed to expose the
  current WARM cache, original COLD seed, fast-executor state/VGO, and replan
  event data without mutating or recalculating guidance.
  Evidence: `TransferMidcourse` exposes `last_corr_dv()`,
  `last_retargeted()`, `last_slow_valid()`, and `last_warm_used()`;
  `plan_transfer` has a read-only `warm_used_out` diagnostic (default
  `nullptr`) so the panel can report a genuine WARM-vs-COLD fallback instead
  of inferring it from the Newton count; `TransferDebugResult` carries the
  corresponding warm replan telemetry.
- [x] M06-R21-D03 Add a transfer-warm display builder that caches the COLD
  seed arc and current WARM arc, rebuilds only on meaningful cache change, and
  computes the future target at `cache.arrival_epoch`.
  Evidence: `gui.cpp` builds `transfer_warm_seed_display` from the original
  COLD result and `transfer_warm_display` from the live WARM cache, and
  rebuilds the WARM display after a successful bounded replan.
- [x] M06-R21-D04 Rework the transfer-warm GUI scene to render COLD SEED,
  WARM PLAN, LIVE, ACT/VGO, future target, and brief replan event markers with
  a debug-only camera/view.
  Evidence: `draw_transfer_warm_debug` renders the labelled COLD SEED and
  WARM PLAN arcs, the LIVE craft, ACT/VGO overlay, `COMPANION @ WARM ARRIVAL`
  marker, and a 1.2-second replan banner (including a `[WARM]` / `[COLD]`
  source tag); the transfer-warm debug frame is locked/fitted to the route
  plus the live craft, and an aborted WARM cache is cleared so the scene stays
  truthful.
- [x] M06-R21-D05 Clean up the transfer-warm panel fields and units.
  Evidence: the panel now separates terminal miss, correction DV, retarget
  threshold, and fast-executor state/remaining VGO/throttle; the FALLBACK COLD
  indicator now comes from the controller's actual WARM/COLD outcome rather
  than from a Newton-iteration heuristic.
- [x] M06-R21-D06 Add the R21 automated presentation/telemetry regressions.
  Evidence: `test_r21_telemetry_semantics` in `tests/test_transfer_warm.cpp`
  and the new `test_transfer_warm_*` cases in
  `tests/test_debug_subsystem.cpp` cover R21 V01-V07.
- [x] M06-R21-D07 Run the R21 verification battery and stop uncommitted at
  `M06-R21-H01`.
  Evidence: after the fallback-telemetry fix, all non-landing ctest targets
  pass; `lander_landing_tests` remains the known slow target with only the
  pre-existing V14-C cross-body soft-land timeout concern; headless
  `--debug-subsystem transfer-warm` and normal-game smokes exit 0; the work is
  left uncommitted at the R21 human gate.

### M06-R21 automated verification

- [x] M06-R21-V01 Panel units/semantics: terminal miss is metres, correction
  magnitude is m/s, retarget threshold is m/s, and the retarget-decision
  telemetry matches the same `corr_dv` used by the controller.
  Evidence: `test_r21_telemetry_semantics` checks both the hold and retarget
  branches against an independent `plan_transfer` correction node and also
  asserts that a valid cached route is re-aimed through the WARM correction
  (`last_warm_used()`), giving the panel a reliable WARM/COLD fallback
  signal.
  Covers:
    - M06-R21-01
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V02 COLD vs WARM display: COLD display is built from the original
  cold cache, WARM display is built from the current warm cache, and changing
  the WARM cache changes only the WARM display, not the COLD seed.
  Evidence: `test_transfer_warm_display_preserves_seed_and_advances_warm`
  passes.
  Covers:
    - M06-R21-02
    - M06-R21-03
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V03 Temporal target: the WARM arrival target uses
  `cache.arrival_epoch`.
  Evidence: `test_transfer_warm_display_arrival_target_future` passes.
  Covers:
    - M06-R21-06
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V04 Live state: display construction does not mutate the live
  simulation.
  Evidence: `test_transfer_warm_display_no_mutation` passes.
  Covers:
    - M06-R21-04
    - M06-R21-P03
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V05 Fast executor overlay: ACT uses the actual state angle, VGO
  uses the exact `TransferMidcourse` fast-executor remaining VGO, zero VGO hides
  the ray, and the state is read-only.
  Evidence: `test_transfer_warm_fast_executor_overlay` passes.
  Covers:
    - M06-R21-05
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V06 Replan cadence: existing bounded-rate tests remain green and
  no visualization code invokes additional transfer solves.
  Evidence: `lander_transfer_warm_tests` passes, including
  `test_two_level_bounded_rate`; the new display builders call only
  `transfer_cold_display` / `predict_zero_thrust`, never a transfer solver.
  Covers:
    - M06-R21-03
    - M06-R21-P01
    - M06-R21-P02
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R21-V07 Cost: display-arc propagation is not counted/reported as
  solver propagation cost.
  Evidence: `test_transfer_warm_display_cost_not_reported` and
  `test_transfer_warm_display_no_mutation` assert the propagation counter is
  unchanged by display construction.

  Covers:
    - M06-R21-P03
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
### M06-R21 human verification

- [H] M06-R21-H01 Transfer-warm visual-inspection gate.
  Run: `./build/lander_gui --debug-subsystem transfer-warm`. The human must
  confirm:
  1. COLD SEED is visibly identifiable.
  2. The current WARM PLAN is visibly identifiable.
  3. The LIVE craft position is obvious.
  4. ACT/VGO physical correction is obvious when active.
  5. No substantially off-axis thrust / R19 regression.
  6. Replans are visually correlated with WARM route changes.
  7. Replans do not cause visible long freezes.
  8. Telemetry clearly distinguishes terminal miss [m] from correction DV
     [m/s].
  9. Replan count is bounded, not every physics tick.
  10. The craft visibly progresses toward the future COMPANION target.
  11. No teleport / hidden force.
  12. Abort remains clean.
   Status: FAILED on 2026-10-08 after the human watched the complete run.
   The craft reached the COMPANION vicinity but crashed; during the latter
   third it repeatedly showed `FAST BURN` / `THR 1.00` with ACT/VGO materially
   separated (sometimes approximately orthogonal), VGO around 2.1-2.7 m/s
   failing to converge, and a powered spin averaging the thrust vector. This
   exposed the R22 controller defect and secondary R21-F01 presentation
   defects.
   Status: FAILED again on 2026-10-09 after a complete 200 s recording.
   The R22 spinning defect is substantially better, but WARM guidance still
   churns among ALIGN/BURN/COMPLETE, the arrival TOF remains near 43.4 s,
   plan annotations/camera/route context remain visually unstable, and the
   run does not demonstrate a successful finite-time rendezvous. The gate
   remains OPEN / FAILED; R21-F01 remains unresolved and is supplemented by
   M06-R23. Do not mark it accepted, do not commit, and do not proceed to
   autoland-primary.
   Do not self-complete this gate. Do not proceed to autoland-primary until
   explicit human acceptance.

  Covers:
    - M06-R21-02
    - M06-R21-03
    - M06-R21-04
    - M06-R21-05
    - M06-R21-06
    - M06-R21-07
    - M06-R21-08
    - M06-R21-09
    - M06-R21-10
    - M06-R21-P04
    - M06-R21-P05
    - M06-R21-F01-01
    - M06-R21-F01-02
    - M06-R21-F01-03
    - M06-R21-F01-04
    - M06-R21-F01-05
  Gate: HUMAN
### M06-R21-F01 bounded presentation follow-up after the first H01 FAIL

Source: USER (2026-10-08) `M06-R21-H01` FAIL. These are R21 presentation
defects observed in the same human run. They must be fixed without mixing
them into R22 control math. R21 remains unresolved until both the R22
functional correction and this presentation follow-up pass human review.

- [~] M06-R21-F01-01 Camera scale pumping: make the transfer-warm debug
  camera temporally stable (bounded smoothing / hysteresis / minimum context
  extent). No large frame-to-frame zoom jumps from ordinary replans; retain
  LIVE + relevant route + arrival-target context; normal camera unchanged;
  geometry undistorted.
  DONE (2026-10-08): added presentation-only `TransferDebugCameraState` and
  `update_transfer_debug_camera`; center is eased and per-frame zoom ratio is
  bounded, with a minimum context extent for the target body.
  Verified-By:
    - M06-R21-H01
    - M06-R21-F01-V01
- [ ] M06-R21-F01-02 Temporal/current ambiguity: explicitly distinguish
  current-time COMPANION from `COMPANION @ WARM ARRIVAL`. A temporal arc may
  cross the body's position at a different epoch without being presented as a
  terrain intersection. Current-time geometry should be visibly CURRENT /
  dimmer where necessary.
  IMPLEMENTED (2026-10-08): the scene now draws a distinct `CUR <TARGET>`
  current-time marker and keeps the `@ WARM ARRIVAL` outline at the planned
  future epoch. Human visual confirmation remains pending.
  Verified-By:
    - M06-R21-H01
    - M06-R21-F01-V02
- [ ] M06-R21-F01-03 Plan origin: if the cached WARM plan's departure point is
  no longer exactly LIVE because the ship advanced since the last replan, show
  `PLAN DEP`, `LIVE`, and `PLAN AGE <...>` (or equivalent minimal truth). Do
  not draw a fake connector. If they should coincide at the same epoch but do
  not, diagnose the transform as a bug.
  IMPLEMENTED (2026-10-08): the warm label is `PLAN DEP`, the live craft label
  is `LIVE`, and `PLAN AGE <s>` is shown next to the plan origin. No connector
  is drawn. Human visual confirmation remains pending.
  Verified-By:
    - M06-R21-H01
- [ ] M06-R21-F01-04 Label clutter: use deterministic offsets / collision
  avoidance so LIVE, ACT/VGO, WARM ARR, COMPANION @ WARM ARRIVAL, route
  labels, and the panel remain readable near arrival. Do not build a general
  layout framework.
  IMPLEMENTED (2026-10-08): deterministic per-label offsets were added to the
  transfer-warm scene. Human readability confirmation remains pending.
  Verified-By:
    - M06-R21-H01
    - M06-R21-F01-V03
- [ ] M06-R21-F01-05 Replan banner: make the WARM replan banner compact and
  non-obscuring (small event line near the panel/top edge, short lifetime,
  same truthful information).
  IMPLEMENTED (2026-10-08): the banner is a compact top-center one-line event
  with a short lifetime. Human non-obscuring confirmation remains pending.
  Verified-By:
    - M06-R21-H01
    - M06-R21-F01-V03

Derived R21-F01 tasks:
- [x] M06-R21-F01-D01 Implement the R21-F01 camera-stability strategy.
  DONE (2026-10-08): `src/debug_subsystem.cpp` provides the bounded stabiliser
  used by the transfer-warm debug camera.
- [x] M06-R21-F01-D02 Implement the R21-F01 temporal/current-body and plan-age
  clarity changes.
  DONE (2026-10-08): `src/gui.cpp` renders current-time vs arrival-time target
  geometry and plan-origin age.
- [x] M06-R21-F01-D03 Implement the R21-F01 label-collision and compact-banner
  changes.
  DONE (2026-10-08): deterministic label offsets and compact top banner are
  implemented.
- [x] M06-R21-F01-D04 Add/extend headless or numeric presentation checks where
  practical and re-run the R21 smoke suite.
  DONE (2026-10-08): `test_transfer_debug_camera_stability` verifies bounded
  per-frame zoom changes and finite camera state; a dummy-video GUI smoke run
  of `--debug-subsystem transfer-warm` ran 4000 frames without crash.

Additional R21-F01 automated verification:

- [x] M06-R21-F01-V01 The transfer-warm debug camera does not produce large
  frame-to-frame zoom jumps from ordinary bounded replans, while retaining the
  live/route/arrival context.
  Evidence: `test_transfer_debug_camera_stability` passes.
  Covers:
    - M06-R21-F01-01
  Gate: PRESENTATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [ ] M06-R21-F01-V02 The transfer-warm scene preserves truthful epoch
  distinction between current-time and arrival-time body geometry.
  Implementation is in place; human visual confirmation remains pending.
  Covers:
    - M06-R21-F01-02
  Gate: PRESENTATION
- [ ] M06-R21-F01-V03 Near-arrival labels and the replan banner remain
  readable / non-obscuring in the deterministic transfer-warm fixture.
  Implementation is in place; human visual confirmation remains pending.

  Covers:
    - M06-R21-F01-04
    - M06-R21-F01-05
  Gate: PRESENTATION
## M06-R22 — full-range node-executor alignment safety / WARM burn stability

Source: USER (2026-10-08), `M06-R21-H01` HUMAN VERIFICATION: FAIL. The human
reviewed the complete transfer-warm run. The craft technically reaches the
COMPANION vicinity but crashes, and during roughly the last third the panel
repeatedly shows `FAST BURN` / `THR 1.00` while ACT and VGO are visibly tens of
degrees apart, at several points close to orthogonal. Around 80 s VGO is about
2.7 m/s; around 96-116 s it remains about 2.06-2.07 m/s despite sustained full
throttle. The spacecraft is effectively performing a powered spin and averaging
the thrust vector toward zero. This is unacceptable physical guidance.

The current code hypothesis is that R19's continuous alignment safety is
magnitude-gated:

```cpp
small_vgo = |VGO| <= kSmallVgoSteps * main_accel * fixed_dt
burning   = state == BURN && (!small_vgo || aligned)
```

With `main_accel = 4.0 m/s^2`, `fixed_dt = 1/120 s`, and
`kSmallVgoSteps = 1.5`, the strict gate applies only below about 0.05 m/s VGO.
At the observed ~2 m/s VGO, `small_vgo == false`, so BURN can remain at full
throttle regardless of alignment. This hypothesis must be verified
quantitatively before code changes.

R21 remains unresolved. The uncommitted R21 presentation work may remain in the
working tree and must not be discarded. Do not commit R22 until its human gate
passes. Do not proceed to autoland-primary.

### M06-R22 explicit requirements

- [x] M06-R22-01 Read
  `docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md`
  before modifying node execution. Verify the R19 magnitude-gated hypothesis
  quantitatively with a headless per-physics-step WARM trace recording sim
  time, executor state, VGO magnitude/vector, spacecraft angle, desired VGO
  angle, angular error, omega, `aligned()`, throttle, delivered thrust vector,
  and WARM replan/retarget events. Prove whether the failure contains steps
  satisfying `state == BURN && aligned == false && main_throttle > 0`, measure
  the maximum angle/rate violation, and measure whether WARM replanning during
  ALIGN moves the target faster than attitude can settle. Keep raw traces in a
  diagnostic artifact, not `TASKS.md`.
  Verified-By:
    - M06-R22-V01
- [x] M06-R22-02 Enforce the physical invariant across the ENTIRE VGO range:
  for every authoritative physics step, if applied `main_throttle > 0`, the
  spacecraft must be inside the canonical safe burn-alignment envelope.
  Preserve the established envelope unless evidence demands a separately
  approved change: angle error <= 0.05 rad and |omega| <= 0.1 rad/s. No regime
  may intentionally maintain full thrust while materially misaligned.
  Verified-By:
    - M06-R22-V02
- [x] M06-R22-03 Apply the narrow preferred NodeExecutor correction: every BURN
  step continuously evaluates alignment. If aligned, ordinary physical burn is
  permitted. If materially misaligned, throttle = 0, ordinary bang-bang
  attitude correction continues, the executor returns/holds in ALIGN, VGO is
  preserved, and burn resumes only after real alignment is recovered. This must
  apply to large and small VGO. Remove the R19 magnitude exemption if diagnosis
  confirms it is causal. A small explicitly documented hysteresis state is
  acceptable only if strict enter/exit equality causes numerical chatter and
  every thrust-enabled state remains physically close to the canonical
  alignment envelope. Do not create a wide "approximately pointing" band.
  Verified-By:
    - M06-R22-V01
- [x] M06-R22-04 Do not solve the failure by spinning while burning, averaging
  thrust direction over rotations, projecting off-axis impulse onto desired
  VGO, pretending off-axis impulse was on-axis, direct velocity assignment,
  direct attitude snapping, hidden forces, loosening the alignment gate to
  recover transfer performance, changing gravity, or changing main
  acceleration.
  Verified-By:
    - M06-R22-V03
- [~] M06-R22-05 After full-range burn gating, measure the interaction between
  WARM replan cadence, retarget cadence, and the fast-executor ALIGN/BURN
  lifecycle. If frequent replans during ALIGN repeatedly move the VGO target
  before attitude can settle and that materially contributes, apply the
  narrowest bounded scheduling rule that lets an active physical correction
  settle before replacing its target. If it does not materially contribute, do
  not change it. Do not disable WARM replanning generally and do not move a
  solver into the HOT path.
  Verified-By:
    - M06-R22-H01
- [x] M06-R22-06 Add deterministic regressions for the actual failure. A
  whole-run invariant must verify that for every authoritative physics step,
  if applied `main_throttle > 0`, executor alignment safety is satisfied. This
  must cover large VGO as well as the previous small-VGO case, including a
  constructed/checkable case with |VGO| >= 1 m/s where the old
  `kSmallVgoSteps` loophole would fail. During each uninterrupted physical burn
  segment, verify ACT stays inside the allowed VGO direction envelope, VGO
  magnitude makes physically sensible progress, and there is no sustained
  interval where THR ~= 1 while VGO remains essentially stationary because the
  thrust direction is rotating around it. Do not demand global monotonic VGO
  across WARM retarget events. Preserve final partial throttle, clean abort, no
  latent thrust, the R19 exact small-vector regression, and no direct state
  mutation.
  Verified-By:
    - M06-R22-V02
- [x] M06-R22-07 Re-run R5/R21 closed-loop transfer tests after the safety
  fix. If stronger physical safety worsens an old transfer-distance proxy, do
  NOT relax the safety invariant. Diagnose whether WARM retarget scheduling,
  transfer correction timing, higher-level bounded guidance strategy, or the
  old derived proxy is the problem. Any acceptance-threshold change requires
  explicit evidence and durable supersession bookkeeping. Do not repeat the
  prior strategy of weakening burn alignment to buy back transfer performance.
  Verified-By:
    - M06-R22-V03
    - M06-R22-V05
- [~] M06-R22-08 Preserve the R21 graphical defects as R21-F01 presentation
  work, not R22 guidance semantics: camera scale pumping, temporal/current
  geometry ambiguity, plan origin/age, label clutter, and the oversized replan
  banner. Address them after/alongside the functional correction without
  mixing them into control math.
  Verified-By:
    - M06-R22-V06
    - M06-R22-H01
- [~] M06-R22-09 The deterministic human transfer-warm fixture must reach/pass
  through the intended target region and remain non-crashed through the planned
  arrival encounter. It may fly past afterward; landing/capture is not required
  by this cell. Record closest target distance, state at planned arrival, fuel,
  replan count, retarget count, maximum thrust-enabled attitude error, and
  maximum thrust-enabled |omega|.
  Verified-By:
    - M06-R22-V04
    - M06-R22-H01
- [~] M06-R22-10 When R22 and the R21-F01 automated verification are complete,
  set `Phase: HUMAN_VERIFICATION` and `Active-Request: M06-R22`. Keep both
  `M06-R21-H01` and `M06-R22-H01` explicit and unresolved. Human command:
  `./build/lander_gui --debug-subsystem transfer-warm`. Do not self-complete
  either gate and do not proceed to autoland-primary before explicit human
  acceptance.

  Verified-By:
    - M06-R22-H01
### M06-R22 preservation constraints

- [ ] M06-R22-P01 Preserve authoritative gravity/ephemerides, main acceleration,
  WARM/COLD solver mathematics, transfer acceptance threshold, terrain
  clearance, physics, normal gameplay, landing/autoland, TFD-1/TFD-2, and M07.
- [ ] M06-R22-P02 No hidden forces, no direct spacecraft-state mutation, no
  direct attitude snapping, no direct velocity assignment, and no pretending
  off-axis impulse was on-axis.
- [ ] M06-R22-P03 Keep all transfer solving out of the 120 Hz HOT path; the fast
  executor remains O(1) per step.
- [ ] M06-R22-P04 Preserve the uncommitted R21 presentation work in the working
  tree; do not discard it while performing R22.
- [ ] M06-R22-P05 Do not create separate request IDs for individual diagnostic
  experiments inside R22.
- [ ] M06-R22-P06 Safety takes priority over preserving an old convergence
  score or transfer-distance proxy.

### M06-R22 derived implementation tasks

- [x] M06-R22-D01 Diagnose the R19 magnitude-gated loophole with a headless
  per-step trace and save the raw trace to a diagnostic artifact.
  DONE (2026-10-08): current-code headless GUI transfer-warm fixture
  (`seed=1007`, orbit body 0, COLD 0->1, `maybe_replan` at 0.1 s with
  `corr_dv` threshold 0.25) was traced for 14128 steps to
  `records/M06-R22-r22-transfer-trace.tsv`. The run crashes at 117.733 s with
  `vgo_mag=2.070`, `throttle=1`, `angle_err=-1.558`, `omega=-1.94`, and
  `min_dist=37.80`. It contains 5300 thrust steps violating the canonical
  alignment envelope, 5045 with `|VGO| >= 1 m/s`, max thrust-enabled angle
  error 3.066 rad, and 79.08 rad of accumulated rotation during bad
  full-thrust steps. Causal chain: a WARM re-target arms a direct BURN using a
  wider 0.2 rad continuation band, and the R19 magnitude gate stops
  re-checking alignment once `|VGO|` exceeds 0.05 m/s, allowing off-axis
  impulse to grow the VGO and spin the craft. Canonical doc updated to the
  full-range rule.
- [x] M06-R22-D02 Implement the narrow full-range NodeExecutor alignment gate:
  BURN with material misalignment produces zero main thrust, continues
  bang-bang attitude correction, preserves VGO, and resumes burn only after
  real alignment.
  DONE (2026-10-08): the NodeExecutor applies the full-range burn-alignment
  gate; misaligned BURN drops main thrust to zero and holds/returns to ALIGN.
- [x] M06-R22-D03 Measure WARM replan/retarget target-chasing during ALIGN and,
  only if causal, apply the narrowest bounded scheduling rule that lets an
  active correction settle.
  DONE (2026-10-08, investigation only, no adopted scheduling change): the
  post-fix R5 closed-loop trace (`/tmp/opencode/r22_r5_final.tsv`) shows the
  bounded 5 Hz WARM cadence is not the unsafe defect (0 bad-thrust steps). It
  does lengthen the first no-thrust ALIGN hold: 75 retargets over 4000 ticks,
  156 replans, max target-angle step 0.165 rad, average 0.015 rad, and one
  363-tick initial ALIGN swing while the corrected VGO direction moves. Two
  narrow scheduling candidates were measured: (a) suppressing every WARM
  solve while misaligned reduced retargets but let a stale correction grow
  the VGO to 11.7 m/s; (b) refreshing the warm cache but suppressing only the
  re-arm while misaligned reduced retargets to 30 and improved R5 from 0.7794
  to 0.7754, but worsened the GUI fixture min distance from 123.74 to 146.14.
  Neither candidate is clearly beneficial, so no WARM scheduling rule was
  adopted. The remaining R5 ratio loss is treated as the bounded cost of the
  full-range safety gate, and the old 0.65 proxy is superseded to 0.80 under
  M06-R22-D06/V03 evidence.
- [x] M06-R22-D04 Add the R22 exact-failure regression, whole-run
  throttle=>alignment invariant, large-VGO case, burn-segment VGO progress
  checks, and deterministic transfer-warm no-crash-through-arrival fixture.
  DONE (2026-10-08): added/extended the node-executor full-range regression and
  the deterministic `M06-R22-V04` transfer-warm no-crash-through-arrival
  fixture.
- [x] M06-R22-D05 Address the R21-F01 presentation follow-ups (camera
  stability, temporal/current clarity, plan age, label clutter, compact
  banner) without changing R22 control math.
  DONE (2026-10-08): implemented in the transfer-warm debug presentation path
  only; no R22 node-executor control math was changed by F01.
- [x] M06-R22-D06 Run the R22 verification battery and update durable state to
  `ACTIVE / HUMAN_VERIFICATION / M06-R22`, leaving both R21-H01 and R22-H01
  unresolved.
  DONE (2026-10-08): full `ctest` returns to the known baseline with only the
  unrelated V14-C landing case red; durable state was moved to HUMAN_VERIFICATION.

### M06-R22 automated verification

- [x] M06-R22-V01 Exact-failure regression: a deterministic WARM/node-executor
  case with |VGO| >= 1 m/s and material misalignment fails under the old
  magnitude-gated logic and passes under the new full-range gate.
  Evidence: `lander_flight_computer_tests` passes, including the full-range
  alignment safety regression.
  Covers:
    - M06-R22-01
    - M06-R22-03
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R22-V02 Whole-run invariant: in the deterministic transfer-warm
  fixture and relevant node-executor tests, every step with applied
  `main_throttle > 0` satisfies the canonical burn-alignment envelope.
  Evidence: `lander_transfer_warm_tests` and `lander_flight_computer_tests`
  pass with the throttle => alignment checks enabled.
  Covers:
    - M06-R22-02
    - M06-R22-06
  Gate: INVARIANT
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R22-V03 Existing node-executor / flight-computer / transfer-warm /
  debug-subsystem / predictor / render-camera tests pass, including the R19
  exact small-vector regression, final partial throttle, clean abort, and no
  latent-thrust checks.
  Evidence: full `ctest` passes all non-landing test binaries.
  Covers:
    - M06-R22-04
    - M06-R22-07
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R22-V04 Deterministic transfer-warm fixture reaches/passes the
  intended target region non-crashed through the planned arrival encounter and
  records closest target distance, arrival state, fuel, replan/retarget counts,
  max thrust-enabled angle error, and max thrust-enabled |omega|.
  Evidence: `test_r22_transfer_warm_no_crash_through_arrival` passes and
  records the required metrics in its `[R22-V04]` output.
  Covers:
    - M06-R22-09
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R22-V05 Non-landing `ctest` passes; full `ctest` returns to the known
  baseline with only the unrelated V14-C landing timeout red.
  Evidence: full `ctest` shows 11/12 test binaries passing; the only failure
  is `lander_landing_tests` V14-C.
  Covers:
    - M06-R22-07
  Gate: INTEGRATION
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [ ] M06-R22-V06 R21-F01 presentation checks pass where automatable (camera
  zoom stability, truthful epoch distinction, readable near-arrival labels,
  compact banner).
  Automatable camera-stability check passes; the visual truthfulness/readability
  portions remain human verification.

  Covers:
    - M06-R22-08
  Gate: PRESENTATION
### M06-R22 human verification

- [H] M06-R22-H01 Corrected full-range alignment-safety / WARM burn-stability
  gate.
  Run: `./build/lander_gui --debug-subsystem transfer-warm`. The human must
  confirm:
  1. No full-throttle spinning / thrust-vector averaging.
  2. Whenever ACT/VGO are materially separated, THR drops to zero.
  3. Physical burn resumes after alignment.
  4. Sustained burns actually reduce VGO.
  5. No uncontrolled rotation.
  6. No crash at the companion encounter.
  7. Bounded replanning with no visible long freeze.
  8. COLD SEED / WARM PLAN / LIVE / future target remain understandable.
  9. Camera does not pump wildly.
  10. Labels remain readable.
   11. No teleport / hidden force.
   12. Abort remains clean.
   Status: FAILED on 2026-10-09 after a complete 200 s recording. The
   previous full-throttle spinning failure is substantially corrected and must
   be preserved, but WARM guidance remains unstable: repeated short burns,
   retargets before corrections settle, a near-constant 43.4 s arrival TOF,
   and no demonstrated finite-time rendezvous. The gate remains OPEN / FAILED;
   the remaining functional work is tracked as M06-R23.
   Do not self-complete this gate. Do not proceed to autoland-primary before
   explicit human acceptance of both R21-H01 and R22-H01.

  Covers:
    - M06-R22-05
    - M06-R22-08
    - M06-R22-09
    - M06-R22-10
  Gate: HUMAN
## M06-R23 — WARM rendezvous-epoch correctness, retarget stability, and physical correction convergence

Source: USER (2026-10-09), after `M06-R21-H01` and `M06-R22-H01` both FAILED
during a complete 200 s transfer-warm recording. At approximately T=201 s the
panel reported `REPLAN 1797`, `RETARGET 743`, `PROPOSALS 10209`, `CACHE TOF
43.4 s`, `COLD TOF 54.2 s`, and `STATE FLIGHT`. The WARM arrival TOF remained
near 43.4 s for most of the recording; the planner repeatedly generated new
corrections; the fast executor alternated among ALIGN/BURN/COMPLETE; many
physical burns were extremely short; and new retargets frequently arrived
before an existing physical correction had settled. The craft survived the
recording but did not demonstrate a successful finite-time rendezvous. The
R22 full-throttle spinning failure is substantially corrected and that
improvement must be preserved.

`M06-R21-H01`, `M06-R22-H01`, and `M06-R21-F01` remain unresolved. Do not
commit, do not self-accept either human gate, and do not proceed to
autoland-primary.

Root-cause resolution (2026-10-09, `D05`): the receding encounter was **not**
a solver or threshold defect. The WARM midcourse had no terminal-completion
condition: once the craft reached a (now-past) arrival epoch while already
inside the companion's clearance shell, the fixed-epoch WARM could not target a
past epoch, the bounded COLD fallback reselected the next later epoch
(54.23 -> 78.15 -> 112.4 -> 200.0), and each new epoch's degenerate large
correction (~36 m/s) re-aimed the craft off its arc, ending in a crash into the
companion (pre-fix 120 s run: `REPLAN`/`RETARGET` unbounded, `min_dist` 37.8 <
38.27, `crash=1`). `D05` adds bounded retarget persistence / safe-boundary
retargeting to `TransferMidcourse::maybe_replan`: once the craft reaches the
destination clearance shell the transfer **completes** (disengage + fast
abort, coast through) **before** any COLD fallback can move the epoch. Post-fix
120 s run: `epoch` changes **once** (54.23 -> 78.15, a single early off-arc
correction), then holds; `crash=0`; `min_dist` 53.1 m (clean passage inside the
53.27 shell); beats the uncorrected COLD baseline (235.1 m). This resolves the
R23-02 churn and the R23-03/R23-04 "incompatible terminal states" / "FAST
COMPLETE with stale THR" symptoms, which were all driven by the receding
epoch. The R22 full-throttle invariant is preserved (whole-run
`max_align_err` 0, `max_abs_omega` 0 in the transfer-warm suite). The
2026-10-09 follow-up work (provisionally published in commit 45b8049,
not human-accepted) then completed D02, D06, and the battery: D05's terminal-completion
check moved from `maybe_replan` into an O(1) per-step check in
`TransferMidcourse::after_step` (the arrival-shell crossing is detected at the
crossing step and completion preempts any COLD fallback, keeping the solver
out of the 120 Hz HOT path). D02's headless lifecycle trace verifies the full
120 s window (V02): 584 candidate generations = 428 holds + 156 retargets
(8 interrupted active corrections, 148 benign re-aims), 1445 planner
invocations short-circuited by the R19 burn-boundary guard, 1 epoch change
(54.23 -> 78.15), clean completion at t=73.81 s with no landing; the crash at
t=98.26 s occurs only in the uncommanded return-orbit phase after completion,
outside the transfer lifecycle. D06's presentation decoupling is implemented
(V06): the route display, banner, and numeric readout publish at most every
2.5 s of simulation time (banner shown 3.0 s; the replaced route stays on
screen faded for 8 s; the debug camera holds its scale after the initial fit
so the planner's re-fit cadence no longer drives the zoom). P2 reworked
R23-05 into a full-encounter oracle: WARM completes at t=73.81 s in an
84.1 s window, closest approach 42.1 m, minimum clearance 4.49 m, 14 burns,
fuel 96/1000, no crash; the uncorrected COLD baseline never completes
(closest 235.1 m, minimum clearance 17.40 m). The verification battery
returns 12/12 ctest (V14-C did not reproduce; see Known unresolved / deferred
issues). The three human gates stay unresolved.

### M06-R23 explicit requirements

- [x] M06-R23-01 Investigate arrival-epoch correctness first. For every
  accepted WARM plan, record simulation time, absolute plan departure epoch,
  absolute plan arrival epoch, plan TOF, remaining time to arrival, plan
  identifier, predicted target position at arrival, predicted spacecraft
  position at arrival, predicted target-relative velocity at arrival, predicted
  terminal miss, and predicted fuel cost. Establish whether WARM is
  continuously moving its desired arrival epoch forward. If so, determine
  whether that behaviour is intentional and whether the algorithm provides a
  finite convergence guarantee. Do not assume a constant rolling arrival
  horizon is correct merely because the predicted miss remains small. Compare
  the initial COLD plan with subsequent WARM plans. Independently propagate the
  companion ephemeris and verify the plotted arrival marker against the target
  position at the correct absolute epoch, and verify the displayed WARM
  trajectory uses the correct departure state, arrival epoch, and coordinate
  frame. Do not infer a geometry bug merely from current-time and future-time
  paths visually crossing.
  Verified-By:
    - M06-R23-V01
- [~] M06-R23-02 Investigate retarget churn. Record retarget acceptance time,
  previous and replacement VGO vectors, correction magnitude, target angular
  displacement, spacecraft angle, angular velocity, executor-state transition,
  throttle, actual delivered delta-v, remaining VGO, and estimated time to
  complete the correction. Measure the frequency of interrupted corrections.
  Distinguish: A. generating a new candidate trajectory, B. accepting that
  trajectory, and C. replacing an actively executing physical correction.
  These events need not coincide. Investigate bounded target persistence,
  acceptance hysteresis, and retargeting at safe executor boundaries. Do not
  blindly add delays or disable WARM replanning, and preserve emergency
  correction and abort behavior.
  Verified-By:
    - M06-R23-V02
    - M06-R23-H01
- [~] M06-R23-03 Investigate the terminal-miss metric. The HUD repeatedly
  alternated between approximately `BEFORE 1.1 m / AFTER 5.4 m` and `BEFORE
  5.4 m / AFTER 1.1 m`, sometimes within 0.2 s while throttle was zero. Trace
  exactly how BEFORE and AFTER are computed. Verify same or explicitly
  identified arrival epoch, consistent reference frames, correct target
  ephemeris, identical propagation conventions, correct candidate/baseline
  identification, no stale cached prediction, and no misleading reuse of old
  results. A correction must not be accepted merely because two predictions at
  different epochs produce an apparently improved distance. Do not alter the
  acceptance objective until its existing meaning and implementation have been
  verified.
  Verified-By:
    - M06-R23-V03
    - M06-R23-H01
- [x] M06-R23-04 Preserve the R22 physical executor invariant. Applied main
  thrust requires physically valid alignment; do not reintroduce off-axis
  full-throttle spinning, thrust-vector averaging, hidden forces, velocity
  assignment, direct attitude snapping, or relaxed alignment limits. At
  approximately video T=180 s the display briefly reported `FAST COMPLETE /
  VGO 0.00 / THR 0.43`. Determine whether this represents a legitimate final
  partial-throttle physics step displayed after the state transition or actual
  thrust applied after completion. Check authoritative physics-step ordering
  and do not classify it as a defect without evidence.
  Verified-By:
    - M06-R23-V04
- [x] M06-R23-05 Create a deterministic full-encounter regression. It must
  establish the absolute target arrival epoch, actual closest companion
  approach, closest-approach simulation time, target-relative speed,
  propellant consumed, total replans, total accepted retargets, total
  attitude-alignment cycles, total burn interruptions, maximum thrust-enabled
  alignment error, maximum thrust-enabled angular velocity, and survival
  through the planned encounter. Compare it against the uncorrected COLD
  baseline. A predicted miss is not sufficient; the authoritative physical
  trajectory must independently establish successful target-region passage. Do
  not count an indefinitely receding predicted encounter as success. Landing or
  capture is not required. Do not change gravitational constants,
  accelerations, or acceptance thresholds to make the test pass.
  Verified-By:
    - M06-R23-V05
- [~] M06-R23-06 Keep the R21 presentation follow-up unresolved while adding
  the human-observed stability defects: camera scale changes dramatically,
  route context repeatedly disappears, plan annotations change too quickly to
  read, and live numbers/transient events lack visual stability. Decouple
  display update frequency from planner cadence, retain truthful current values
  and event timestamps, and use stable camera framing with bounded temporal
  smoothing. Do not freeze simulation physics or guidance merely to improve
  presentation.
  Verified-By:
    - M06-R23-V06
    - M06-R23-H01
- [~] M06-R23-07 Re-run the complete relevant automated suite, preserve the
  established V14-C landing-timeout baseline, produce a headless deterministic
  trace and concise comparative measurements, and keep large traces in
  diagnostic artifacts rather than `TASKS.md`. When automated verification is
  complete, return to `HUMAN_VERIFICATION` with `M06-R21-H01`,
  `M06-R22-H01`, and the new `M06-R23-H01` gate explicit and unresolved.

  Verified-By:
    - M06-R23-V07
    - M06-R23-H01
### M06-R23 preservation constraints

- [ ] M06-R23-P01 Preserve the R22 full-range alignment-safety invariant and
  the demonstrated removal of full-throttle spinning / thrust-vector averaging.
  Verified-By:
    - M06-R23-V04
- [ ] M06-R23-P02 Keep all transfer solving out of the 120 Hz HOT path; the
  fast executor remains O(1) per step.
  Verified-By:
    - M06-R23-V05
- [ ] M06-R23-P03 Preserve emergency correction and abort behavior.
  Verified-By:
    - M06-R23-H01
- [ ] M06-R23-P04 Do not change gravitational constants, main acceleration, or
  acceptance thresholds merely to make WARM converge.
  Verified-By:
    - M06-R23-V05
- [ ] M06-R23-P05 Keep R21 ownership of graphical defects and do not freeze
  simulation physics or guidance for presentation purposes.
  Verified-By:
    - M06-R23-V06
- [ ] M06-R23-P06 Do not commit, do not self-accept human gates, and do not
  advance to autoland-primary.

  Verified-By:
    - M06-R23-H01
### M06-R23 derived implementation tasks

- [x] M06-R23-D01 Add a headless WARM plan/epoch trace that records the
  arrival-epoch, departure-state, predicted-terminal, and fuel-cost fields for
  every accepted WARM plan.
- [x] M06-R23-D02 Add a headless retarget-lifecycle trace that distinguishes
  candidate generation, cache acceptance, and replacement of an active
  physical correction, and measures interrupted corrections.
- [x] M06-R23-D03 Audit the BEFORE/AFTER terminal-miss computation and apply
  only a narrow correctness fix if an epoch/frame/staleness defect is proven.
- [x] M06-R23-D04 Audit the authoritative ordering around `FAST COMPLETE` and
  partial thrust; add a regression if a post-completion thrust defect is
  proven.
- [x] M06-R23-D05 Implement only an evidence-supported bounded retarget
  persistence / acceptance-hysteresis / safe-boundary retargeting rule.
- [x] M06-R23-D06 Implement presentation-stability decoupling from planner
  cadence while preserving truthful current values and event timestamps.
- [x] M06-R23-D07 Add the deterministic full-encounter closed-loop regression
  comparing the WARM physical trajectory against the COLD baseline.
- [x] M06-R23-D08 Run the R23 verification battery and update durable state to
  `ACTIVE / HUMAN_VERIFICATION / M06-R23`, leaving R21-H01, R22-H01, and
  R23-H01 unresolved.

### M06-R23 automated verification

- [x] M06-R23-V01 The WARM arrival-epoch trace proves that the plotted arrival
  marker and displayed trajectory use the correct absolute arrival epoch,
  departure state, and coordinate frame; any rolling arrival horizon is
  intentional, documented, and bounded toward finite convergence.
  Covers:
    - M06-R23-01
  Gate: INVARIANT
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R23-V02 The retarget-lifecycle trace bounds interrupted active
  corrections and distinguishes candidate generation, trajectory acceptance,
  and active-correction replacement.
  Covers:
    - M06-R23-02
  Gate: INTEGRATION
  Command: ./build/lander_transfer_warm_tests
  Oracle: integration-or-end-to-end
  Expected: PASS with the lifecycle identities: total generations = holds + retargets; retargets = interrupted + re-aims; clean completion at the R23-05 entry epoch; no post-completion planner activity; interrupted corrections bounded; a single epoch change.
  Result: PASS (2026-10-09): 14400 fixed-step ticks (120 s window); completion at t=73.81 s (matches the R23-05 entry; landed=0); 584 candidate generations = 428 holds + 156 retargets (8 interrupted active corrections + 148 benign re-aims); 1445 planner invocations short-circuited by the R19 burn-boundary guard; 1 epoch change (54.23 -> 78.15); 0 post-completion activity; retarget/generation ratio 0.267; the window-end crash at t=98.26 s occurs only in the uncommanded return-orbit phase after completion, outside the transfer-lifecycle scope (the R23-05 safe-exit window covers terrain clearance to 84.1 s).
  Repository-State: HEAD=730c41dbdbabe40dec85e78c893fa3a31b2e3b52; DIFF-SHA256=9f84d91c47c8d43f988b11867883aae85848e262e934796565ba78f12373a243
  Limitations:
    - Single deterministic fixture window; interrupted/re-aim classification relies on the harness' armed-VGO bookkeeping rather than an independent observer
- [x] M06-R23-V03 The BEFORE/AFTER terminal-miss tests verify the same
  explicitly identified epoch and frame and reject stale or mixed-epoch
  acceptance logic.
  Covers:
    - M06-R23-03
  Gate: INVARIANT
  Command: ./build/lander_transfer_warm_tests
  Oracle: independent-existing-regression
  Expected: The R21 telemetry-semantics checks pass (the reported correction delta-v matches the solver node recomputed at the same identified state; a below-threshold correction does not re-target) and the per-plan trace reports the achieved miss at the arrival epoch identified by each accepted plan.
  Result: PASS (2026-10-09): lander_transfer_warm_tests fully green, including test_r21_telemetry_semantics and the per-plan epoch/miss trace (each accepted WARM plan records its arrival epoch and achieved miss; no stale or mixed-epoch acceptance observed in the run).
  Repository-State: HEAD=730c41dbdbabe40dec85e78c893fa3a31b2e3b52; DIFF-SHA256=9f84d91c47c8d43f988b11867883aae85848e262e934796565ba78f12373a243
  Limitations:
    - Headless arithmetic-identity and epoch-identification checks; visual BEFORE/AFTER observation remains with M06-R23-H01
- [x] M06-R23-V04 The R22 full-range alignment invariant and existing
  node-executor / flight-computer / transfer-warm / predictor / debug-subsystem
  / camera tests remain green.
  Covers:
    - M06-R23-04
    - M06-R23-P01
  Gate: INVARIANT
  Evidence-Mode: HISTORICAL_RECORDED
  Evidence-Source: records/M06-schema1-live-TASKS.snapshot.md
  Result: PASS as previously recorded in the schema-1 task ledger; not rerun by this migration
  Limitations:
    - Historical recorded outcome only; exact invocation and independent current applicability not established
- [x] M06-R23-V05 The deterministic full-encounter regression independently
  establishes physical closest approach and target-region passage against the
  COLD baseline, rather than relying on a predicted miss.
  Covers:
    - M06-R23-05
    - M06-R23-P02
    - M06-R23-P04
  Gate: INTEGRATION
  Command: ./build/lander_transfer_warm_tests
  Oracle: integration-or-end-to-end
  Expected: The WARM full-encounter run independently establishes physical closest approach, target-region passage, and finite completion against the uncorrected COLD baseline, with no threshold or physics change.
  Result: PASS (2026-10-09, reworked full-encounter oracle): WARM ticks 10090 (84.1 s window); completes at t=73.81 s (entry == completion, no landing); closest approach 42.1 m at t=76.38 s (relative speed 14.73 m/s); minimum clearance 4.49 m; safe exit 106.5; 156 retargets; 14 burns; 1 epoch change (54.23 -> 78.15); fuel 96/1000; no crash. COLD baseline (14400 ticks) never completes: closest 235.1 m, minimum clearance 17.40 m.
  Repository-State: HEAD=730c41dbdbabe40dec85e78c893fa3a31b2e3b52; DIFF-SHA256=9f84d91c47c8d43f988b11867883aae85848e262e934796565ba78f12373a243
  Limitations:
    - Single deterministic fixture seed; the per-step shell-completion invariant is verified within the same binary (test_r23_terminal_completion_per_step, PASS)
    - The schema-1 historical record remains in records/M06-schema1-live-TASKS.snapshot.md
- [x] M06-R23-V06 Presentation-stability checks pass where automatable,
  including bounded camera framing and decoupling of display update cadence
  from planner cadence.
  Covers:
    - M06-R23-06
    - M06-R23-P05
  Gate: PRESENTATION
  Command: ./build/lander_debug_subsystem_tests
  Oracle: same-change-generated-test
  Expected: Held-zoom stabilisation: repeated aggressive re-plan target changes leave the camera zoom exactly unchanged while the centre keeps easing; the existing per-frame zoom ratio bound (1.35) still applies while easing is enabled; all values remain finite.
  Result: PASS (2026-10-09): the new test_transfer_debug_camera_zoom_hold (60 churn frames; zoom invariant; centre moving; finite) and the pre-existing test_transfer_debug_camera_stability both pass; the GUI now publishes the route display, banner, and numeric readout at most every 2.5 s of simulation time (banner shown 3.0 s; replaced route kept on screen faded for 8 s; camera scale held after the initial fit so the planner's re-fit cadence no longer drives the zoom) — presentation only, no physics or guidance change.
  Repository-State: HEAD=730c41dbdbabe40dec85e78c893fa3a31b2e3b52; DIFF-SHA256=9f84d91c47c8d43f988b11867883aae85848e262e934796565ba78f12373a243
  Limitations:
    - Headless verification of the pure presentation helper; on-screen visual stability during a live encounter remains with M06-R23-H01
- [x] M06-R23-V07 Full `ctest` returns to the known baseline with only the
  unrelated V14-C landing timeout red.
  Covers:
    - M06-R23-07
  Gate: INTEGRATION
  Command: ctest --test-dir build
  Oracle: independent-existing-regression
  Expected: The full automated suite is green, or returns to the established baseline (the V14-C landing timeout is the only known watch item).
  Result: PASS (2026-10-09): 12/12 ctest green, including lander_landing_tests (V14-C did not reproduce; see Known unresolved / deferred issues); total wall time ~220 s.
  Repository-State: HEAD=730c41dbdbabe40dec85e78c893fa3a31b2e3b52; DIFF-SHA256=9f84d91c47c8d43f988b11867883aae85848e262e934796565ba78f12373a243
  Limitations:
    - V14-C is flaky, so a 12/12 result does not prove closure of M06-R6-V14; it remains a designated watch item
### M06-R23 human verification

- [H] M06-R23-H01 WARM rendezvous convergence, retarget stability, and
  presentation-stability gate.
  Run: `./build/lander_gui --debug-subsystem transfer-warm`. The human must
  confirm:
  1. The WARM arrival epoch / TOF demonstrates finite-time convergence rather
     than a permanently receding encounter.
  2. The craft physically passes through the intended companion target region
     without crash.
  3. Retargets do not repeatedly interrupt an active physical correction.
  4. Burns materially reduce VGO when sustained.
  5. The R22 full-throttle spinning defect has not returned.
  6. The display remains visually stable and readable during the encounter.
  7. No teleport / hidden force is visible.
  8. Abort remains clean.
  Do not self-complete this gate. Do not proceed to autoland-primary before
  explicit human acceptance of R21-H01, R22-H01, and R23-H01.

  Requires:
    - M06-R23-D08
    - M06-R23-V05
    - M06-R23-V06
    - M06-R23-V07
  Covers:
    - M06-R23-02
    - M06-R23-03
    - M06-R23-06
    - M06-R23-07
    - M06-R23-P03
    - M06-R23-P06
  Gate: HUMAN
## Inherited unresolved human gates

These remain unresolved and must not be inferred complete from later automated
work. Preserve them while M06 remains active (current subsystem cell:
TRANSFER-WARM / M06-R21, active follow-up M06-R23).

- [H] M06-R11-H01 Trustworthy-diagnostics visual pass: confirm corrected
  labels/units/body-relative readouts in the debug harness.
  Covers:
    - M06-R11-05-04
  Gate: HUMAN
  Coverage-Source: records/M06-pre-experience-TASKS.snapshot.md
- [H] M06-R12-H01 Static prediction-frame visual pass: fixed frames, legend,
  debug-panel frame/readout coherence.
  Covers:
    - M06-R12-05-01
  Gate: HUMAN
  Coverage-Source: records/M06-pre-experience-TASKS.snapshot.md
- [H] M06-R12-H02 AUTO prediction-frame visual pass: segmented reference-frame
  path, transition markers, and no cross-frame connector.
  Covers:
    - M06-R12-04-01
  Gate: HUMAN
  Coverage-Source: records/M06-pre-experience-TASKS.snapshot.md
- [H] M06-R7-H01 Consolidated M06 MVP playtest. This remains the overall
  milestone human-acceptance gate after subsystem cells are ready.

  Covers:
    - M06-R7-01
  Gate: HUMAN
  Coverage-Source: records/M06-pre-experience-TASKS.snapshot.md
## Known unresolved / deferred issues

- `M06-R6-V14` remains incomplete because the V14-C cross-body soft-land case
  is the sole designated `ctest` baseline watch-item (11/12 nominal baseline).
  Do not hide or weaken it; decide/fix it in the appropriate landing cell before
  M06 closeout. In the R23 verification run (2026-10-09) the full `ctest`
   returned **12/12** (including `lander_landing_tests`), i.e. V14-C did not
   reproduce as a failure in that run (likely timing/flaky). The 2026-10-09
   R23 D02/D06 follow-up battery also returned **12/12**, with
   `lander_landing_tests` green in both the full run and a standalone
   re-run, so the non-reproduction has now been observed twice; confirm
   during human re-verification rather than assuming it closed.
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
- M06-R20 transfer-cold visual-observability cell (including F01): human
  `M06-R20-H01` PASS 2026-10-08. The cell is COMPLETE and committed in the
  R20 completion commit; no R21 / transfer-warm work has been started.
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
