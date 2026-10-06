# Tasks

```yaml
Milestone: M06
State: ACTIVE
Phase: HUMAN_VERIFICATION
Active-Request: M06-R18
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

M06 is active in HUMAN_VERIFICATION. M06-R18 implementation and automated
verification are complete; the next action is the explicit human node-executor
visual/physicality gate. Do not start another implementation group unless that
gate produces feedback requiring follow-up or the user explicitly changes scope.

The baseline durable state is sufficient to resume with experience retrieval
disabled. The experience store is empty at migration; do not fabricate precedent.

## M06-R18 — node-executor observability / presentation prep

Source: USER (2026-10-06)
State: AWAITING_HUMAN

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

- [H] M06-R18-H01 Node-executor human gate.
  Run: `./build/lander_gui --debug-subsystem node-executor`, then press `P`.
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

## Inherited unresolved human gates

These remain unresolved and must not be inferred complete from later automated
work. Preserve them while R18 is in human verification.

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
- Pre-migration R18 implementation/verification snapshot: `508344f`.

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
