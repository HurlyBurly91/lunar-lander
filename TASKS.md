# Tasks

Milestone: M06 — Flight computer and maneuver planning
State: AWAITING HUMAN VERIFICATION
Active request group: M06-R1
Milestone specification: milestones/M06-flight-computer-and-maneuver-planning.md

`TASKS.md` is the live execution ledger for the active milestone only.

## M06 bootstrap

- [x] M06-GATE-01 Confirm M05 is complete before starting M06
  Source: AGENTS.md / repository state
  Evidence:
  - `STATUS.md` previously showed M05 COMPLETE
  - `TASKS.md` previously showed M05 COMPLETE
  - `records/M05-binary-moon-contract-loop.md` exists
  - `HEAD` / `origin/main` at M05 closeout commit `6f1e30ee04c1e77b68b1094554dddb0762aaa4df`
- [x] M06-GATE-02 Persist M06 durable state
  Source: USER continuation request
  Files:
  - `PROJECT.md`
  - `STATUS.md`
  - `milestones/M06-flight-computer-and-maneuver-planning.md`
  - `TASKS.md`
  Evidence:
  - M06 milestone file created
  - `PROJECT.md` marks M05 COMPLETE and M06 ACTIVE
  - `STATUS.md` current milestone is M06 ACTIVE
  - this ledger replaced the completed M05 ledger

## M06-R1 — flight computer and maneuver planning

Source: USER
State: AWAITING HUMAN VERIFICATION

This request group adds a compact KSP-style flight computer to the M05 binary
game. Full stable semantics live in the M06 milestone specification.

### User requirements

- [x] M06-R1-01 Shared authoritative zero-thrust predictor
  Source: USER
  Requirement:
  - Add a deterministic trajectory predictor using the same semi-implicit /
    symplectic Euler fixed-step rule as live flight.
  - Use `Config::fixed_dt = 1.0 / 120.0`.
  - Use the real analytic future binary ephemeris.
  - Keep both gravitational fields active at all times.
  - Do not use RK4 as the authoritative displayed predictor.
  - Do not mutate simulation state during prediction.
  - Refactor live-flight stepping and prediction to share a common propagation
    helper so they cannot silently diverge.
  Files:
  - `include/lander/ballistic.hpp`
  - `src/ballistic.cpp`
  - `include/lander/sim.hpp`
  - `src/sim.cpp`
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  Evidence:
  - `step_ballistic` and `propagate_ballistic` implement the shared
    semi-implicit Euler order at `Config::fixed_dt = 1.0 / 120.0`.
  - `Simulation::step_once` and `predict_trajectory` use the same two-body
    analytic ephemeris and shared stepping path.
  - `tests/test_flight_computer.cpp::test_predictor_parity` passed in
    `lander_flight_computer_tests`.

- [x] M06-R1-02 Single maneuver node model
  Source: USER
  Requirement:
  - Support exactly one active maneuver node.
  - Store fixed-step-aligned node time, node frame body, signed prograde Δv,
    and signed radial Δv.
  - Reconstruct the node's world-frame Δv from the node basis.
  - Pre-node prediction uses zero thrust up to the node.
  - At the node, position is unchanged and velocity is changed by the node's
    ideal world Δv.
  - Post-node prediction continues zero thrust from the modified state.
  - The default prediction horizon is approximately two binary periods.
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  - `src/gui.cpp`
  Evidence:
  - `lander::ManeuverNode` stores only `time`, `frame_body`, `dv_prograde`,
    and `dv_radial`.
  - `predict_trajectory` produces separate `pre` and `post` branches, applies
    the reconstructed `dv_world` at the node, and uses a two-binary-period
    horizon in the GUI.
  - `tests/test_flight_computer.cpp::test_node_impulse_prediction` and
    `test_snap_time_and_default_node` passed.

- [x] M06-R1-03 Node basis and degenerate frames
  Source: USER
  Requirement:
  - Compute node prograde/radial from the predicted pre-burn state relative to
    the node's frame body.
  - Use:
    - `r = p_pre - body_pos`
    - `v = v_pre - body_vel`
    - `prograde = normalize(v)` when valid
    - radial-out as the component of `r_hat` orthogonal to prograde, signed so
      it points outward
  - Handle zero velocity, near-radial velocity, and near-zero radius cases with
    deterministic orthonormal fallbacks.
  - No NaNs or wall-clock/random dependence.
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  Evidence:
  - `compute_node_basis` implements the required body-relative basis and finite
    orthonormal fallbacks.
  - `tests/test_flight_computer.cpp::test_node_basis` passed, including
    zero-velocity, fully degenerate, and near-radial cases.

- [x] M06-R1-04 Predicted trajectory readouts
  Source: USER
  Requirement:
  - Compute predicted terrain impact using future body positions and tidal
    rotation.
  - Compute closest approach to the moving destination pad using the actual
    moving surface point.
  - Compute predicted PE/AP as numeric local extrema of body-relative distance
    `rho(t)` over the post-node branch.
  - PE/AP are predictions, not invariant conic elements.
  - Report no-solution / `--` states when no suitable extremum exists.
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  - `src/gui.cpp`
  Evidence:
  - `TrajectoryPrediction` reports `impact`, `closest`, `peri`, and `apo`.
  - `draw_flight_computer` renders `CP`, `PE`, and `AP` with `--` when absent.
  - `tests/test_flight_computer.cpp::test_trajectory_prediction` passed.

- [x] M06-R1-05 SAS-style attitude modes
  Source: USER
  Requirement:
  - Add attitude modes: OFF, PROGRADE, RETROGRADE, RADIAL OUT, RADIAL IN,
    TARGET, ANTI-TARGET, MANEUVER.
  - PROGRADE/RETROGRADE/RADIAL use the current reference-body-relative state.
  - TARGET/ANTI-TARGET use the current contract destination pad.
  - MANEUVER uses the active executor's remaining Δv direction.
  - Undefined target vectors fail safely to OFF-like behavior with visible UI
    feedback.
  Files:
  - `include/lander/autopilot.hpp`
  - `src/autopilot.cpp`
  - `src/gui.cpp`
  Evidence:
  - `AttitudeMode` and `attitude_target_direction` implement the eight modes.
  - GUI keys `1` through `8` select the modes; attitude selection is ignored
    while an executor is active.
  - Invalid directions produce `pc_message = "ATT INVALID"` in the flight
    computer panel.
  - `tests/test_flight_computer.cpp::test_attitude_controller` passed.

- [x] M06-R1-06 Attitude controller
  Source: USER
  Requirement:
  - Implement a bounded double-integrator attitude controller using the existing
    rotation input path.
  - Use `config.rotate_accel` as the only angular acceleration magnitude.
  - Use shortest-angle wrap.
  - Use a deterministic bang-bang law with braking before overshoot.
  - Use small angular/rate deadbands.
  - The controller must emit ordinary `Input` only; it must not directly mutate
    `state.angle` or `state.omega`.
  - Manual rotation input overrides the controller for that step.
  Files:
  - `include/lander/autopilot.hpp`
  - `src/autopilot.cpp`
  - `src/gui.cpp`
  Evidence:
  - `attitude_input` emits only `rotate_left` / `rotate_right`; it does not set
    throttle or reaction wheels.
  - `tests/test_flight_computer.cpp::test_attitude_controller` covers aligned,
    shortest-path, wraparound, braking, undefined-target, zero-vector, and
    manual-override behavior.

- [x] M06-R1-07 Finite-burn node executor
  Source: USER
  Requirement:
  - Add a node executor state machine: IDLE, WAIT, ALIGN, BURN, COMPLETE,
    ABORTED, INCOMPLETE.
  - For a node with world Δv `dv_total`:
    - `burn_time = |dv_total| / config.main_accel`
    - `t_ignite = node.time - 0.5 * burn_time`
  - If arming occurs after `t_ignite`, the executor may continue aligning and
    must show a late state.
  - During BURN, command ordinary rotation and main throttle only.
  - Consume ordinary fuel.
  - Track thrust-produced remaining Δv, not total spacecraft velocity error.
  - Final step uses partial throttle clamped to `[0, 1]`.
  - Abort, fuel exhaustion, crash, landing, and reset must leave no latent
    autopilot output.
  - No teleportation, direct state mutation, hidden force, or orbit
    stabilization.
  Files:
  - `include/lander/autopilot.hpp`
  - `src/autopilot.cpp`
  - `include/lander/sim.hpp`
  - `src/sim.cpp`
  - `src/gui.cpp`
  Evidence:
  - `NodeExecutor` implements the required state machine and emits ordinary
    `Input` only.
  - GUI `Return` arms the executor, `Shift+Return` and `X` abort it, manual
    main-throttle input aborts it, and terminal executor states clear the
    attitude mode.
  - `tests/test_flight_computer.cpp::test_node_executor` covers burn duration,
    ignition time, wait/align, abort, fuel exhaustion, crash/landing, zero-Δv
    completion, and no latent output.

- [x] M06-R1-08 CIRCULARIZE planner
  Source: USER
  Requirement:
  - Add a CIRCULARIZE planner that creates or edits the single node.
  - Use osculating circularization around the node's frame body at the node
    time.
  - Use `v_circ = sqrt(mu_frame / rho)`.
  - Choose tangent direction from the sign of body-relative angular momentum.
  - Set the node's prograde/radial components from the resulting world Δv.
  - Do not auto-execute the node.
  - Do not mutate live state.
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  - `src/gui.cpp`
  Evidence:
  - `plan_circularize` returns an optional node and does not mutate live
    state.
  - GUI `U` invokes the planner and leaves the node editable without
    auto-executing.
  - `tests/test_flight_computer.cpp::test_planners` verifies that the planned
    node removes the body-relative radial velocity component.

- [x] M06-R1-09 TRANSFER TO OTHER MOON planner
  Source: USER
  Requirement:
  - Add a TRANSFER TO OTHER MOON planner that creates or edits the single node.
  - Refactor the existing M05 transfer solver into a pure reusable solver.
  - Solve from the predicted pre-burn state at the node time.
  - If no node exists, create one at a sensible default node time.
  - The node Δv is the solved departure velocity minus the predicted ship
    velocity at the node.
  - If no valid transfer exists, do not corrupt the node.
  - Preserve existing `T x3` one-shot behavior by calling the same refactored
    solver.
  Files:
  - `include/lander/ballistic.hpp`
  - `src/ballistic.cpp`
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  - `include/lander/sim.hpp`
  - `src/sim.cpp`
  - `src/gui.cpp`
  Evidence:
  - `solve_transfer_velocity` is a pure shared solver.
  - `Simulation::transfer()` calls the same solver.
  - `plan_transfer` solves from the predicted pre-burn state and returns
    `std::nullopt` without corrupting the existing node when no solution
    exists.
  - GUI `I` invokes the planner.
  - `tests/test_flight_computer.cpp::test_planners` verifies planner / `T x3`
    agreement and position preservation.

- [x] M06-R1-10 MATCH TARGET VELOCITY planner
  Source: USER
  Requirement:
  - Add a MATCH TARGET VELOCITY planner that creates or edits the single node.
  - Target the actual moving destination pad surface point.
  - If no node exists, place it near the predicted closest approach to that
    moving pad.
  - Compute target velocity from body-center velocity plus tidal surface-point
    rotation.
  - Set node Δv to `target_velocity - predicted_ship_velocity_at_node`.
  - Do not auto-land or auto-execute.
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  - `src/gui.cpp`
  Evidence:
  - `plan_match_target` targets the moving destination pad surface point and
    returns an editable node.
  - GUI `Y` invokes the planner.
  - `tests/test_flight_computer.cpp::test_planners` verifies that the ideal
    post-node velocity matches the moving pad velocity at the node time.

- [x] M06-R1-11 Flight-computer GUI integration
  Source: USER
  Requirement:
  - Show node state, countdown, Δv components, burn time, executor state,
    attitude mode, closest approach, and predicted PE/AP.
  - Render the pre-node and post-node predicted trajectory with distinguishable
    colors when the navigation overlay is visible.
  - Show node, predicted impact, and closest-approach markers.
  - Add documented keyboard controls for node creation/editing, planners,
    attitude modes, execute, and abort.
  - Preserve all existing M05 controls.
  - Reset node/executor/attitude state on crash, landing, reset, and new
    contract.
  Files:
  - `src/gui.cpp`
  - `include/lander/flight_computer.hpp`
  - `include/lander/autopilot.hpp`
  Evidence:
  - `draw_flight_computer` renders node, countdown, `DPGR`, `DRAD`, `TOT`,
    `BURN`, `EXEC`, `ATT`, `IGN`, `CP`, `PE`, `AP`, `IMPACT`, and messages.
  - `draw_trajectory` renders pre-node and post-node branches, node marker,
    impact marker, closest-approach marker, and PE/AP markers when the
    navigation overlay is active.
  - `print_usage` and the bottom legend document the M06 controls.
  - `reset_flight_computer` is called from `start_mission`, crash/landing, and
    new-contract detection.
  - Headless `lander_gui --frames 500 --seed 1 --orbit-demo` completed with
    `state=flying` and no crash.

### Preservation constraints

- [x] M06-R1-P01 Preserve M05 canonical physics
  Source: USER / AGENTS.md / docs/physics-model-gravity.md
  Constraint:
  - Keep both gravitational fields active.
  - Keep analytic binary ephemeris and tidal locking.
  - Keep body-relative collision/landing/takeoff rules.
  - Do not introduce SOI switching, patched conics, hidden capture, or hidden
    orbit stabilization.
  Evidence:
  - The shared ballistic path uses `BinarySystem::gravity` with both bodies
    active and the analytic ephemeris.
  - No SOI, patched-conic, or capture logic was added.
  - Existing `lander_tests` and `lander_binary_tests` passed.

- [x] M06-R1-P02 Preserve deterministic fixed-step simulation
  Source: USER / AGENTS.md
  Constraint:
  - Keep `Config::fixed_dt = 1.0 / 120.0`.
  - Keep pause/reset/seed determinism.
  - Do not use render time, random values, or wall-clock time in physics,
    prediction, planning, attitude, or executor logic.
  Evidence:
  - The GUI now steps through `Simulation::set_accumulator` and
    `Simulation::step_once` at the existing `fixed_dt`.
  - Prediction, planning, attitude, and executor logic use only simulation
    time, state, config, and binary ephemeris.
  - `tests/test_flight_computer.cpp::test_determinism` passed.

- [x] M06-R1-P03 Autopilot must use ordinary input only
  Source: USER
  Constraint:
  - Autopilot and executor may produce `Input` only.
  - They must not directly mutate `state.angle`, `state.omega`, position,
    velocity, fuel, contract state, or reference-body state.
  Evidence:
  - `attitude_input` and `NodeExecutor::make_input` return `Input` values.
  - `NodeExecutor::after_step` only updates executor bookkeeping; it does not
    mutate `State`.
  - `tests/test_flight_computer.cpp::test_attitude_controller` and
    `test_node_executor` verify ordinary-Input behavior and no latent output.

- [x] M06-R1-P04 No M07/ECS scope creep
  Source: USER / AGENTS.md
  Constraint:
  - Do not introduce ECS, modular spacecraft, threads, multiple nodes, time
    warp, 3D, or a generic planning framework.
  Evidence:
  - M06 added small `ballistic`, `flight_computer`, and `autopilot` modules
    only.
  - No ECS, threads, multiple nodes, time warp, or 3D systems were introduced.

- [x] M06-R1-P05 Preserve existing M05 controls and debug one-shots
  Source: USER / AGENTS.md
  Constraint:
  - Existing M05 keys and guarded triple-tap behavior remain available.
  - `O x3`, `Shift+O x3`, `B x3`, `T x3`, `F`, `R`, `N`, `M`, `V`, `P`, `G`,
    `E`, `Shift+E`, and mouse wheel behavior must not regress unless explicitly
    changed by a later user requirement.
  Evidence:
  - The M06 GUI edits only added new scancode cases and helpers; existing M05
    scancode handlers remain in `src/gui.cpp`.
  - `lander_guarded_actions_tests` passed.
  - The usage text and bottom legend still list the M05 controls.

- [x] M06-R1-P06 Use SDL3 only
  Source: AGENTS.md
  Constraint:
  - GUI work uses SDL3 directly.
  - Keyboard state uses SDL scancodes with `SDL_GetKeyboardState()`.
  Evidence:
  - The new GUI handlers use `SDL_SCANCODE_*` values and the existing
    `SDL_GetKeyboardState` polling lambda.
  - The build uses the pinned SDL3 submodule.

### Automated verification

- [x] M06-R1-V01 Predictor parity tests
  Requirement:
  - Test that a zero-input live `Simulation` clone and the pure predictor match
    over a meaningful horizon.
  - Test that both bodies' gravity affect prediction.
  Evidence:
  - `tests/test_flight_computer.cpp::test_predictor_parity` compares a
    zero-input `Simulation` stepped with `step_once` against
    `step_ballistic` and verifies that changing `mu` changes the path.
  - Executed in `lander_flight_computer_tests`; target passed.

- [x] M06-R1-V02 Shared propagator / transfer refactor tests
  Requirement:
  - Existing M05 deterministic tests continue to pass.
  - `T x3` still changes velocity only, with no position teleport.
  - Live simulation and predictor use the shared propagation helper.
  Evidence:
  - Full CTest suite passed after the refactor.
  - `tests/test_flight_computer.cpp::test_planners` verifies that
    `Simulation::transfer()` and `plan_transfer` agree and that `T x3`
    leaves position unchanged.

- [x] M06-R1-V03 Node basis tests
  Requirement:
  - Test normalization, orthogonality, outward radial sign, angular-momentum
    tangent choice, and degenerate fallbacks.
  Evidence:
  - `tests/test_flight_computer.cpp::test_node_basis` passed.

- [x] M06-R1-V04 Node impulse branch tests
  Requirement:
  - Test pre-node branch unchanged until node time.
  - Test position continuity at node.
  - Test velocity changes by reconstructed `dv_world`.
  - Test post-node branch differs deterministically.
  Evidence:
  - `tests/test_flight_computer.cpp::test_node_impulse_prediction` passed.

- [x] M06-R1-V05 Trajectory / closest approach / PE/AP tests
  Requirement:
  - Test terrain impact using future body position and tidal rotation.
  - Test closest approach to moving destination pad.
  - Test PE/AP as local extrema of `rho(t)` and no-solution behavior.
  Evidence:
  - `tests/test_flight_computer.cpp::test_trajectory_prediction` passed.

- [x] M06-R1-V06 Planner tests
  Requirement:
  - Test CIRCULARIZE, TRANSFER TO OTHER MOON, and MATCH TARGET VELOCITY.
  - Test no-solution transfer does not corrupt node state.
  - Test match-target uses moving pad velocity.
  Evidence:
  - `tests/test_flight_computer.cpp::test_planners` passed.

- [x] M06-R1-V07 Attitude controller tests
  Requirement:
  - Test shortest-path wrap, convergence, braking, deadbands, undefined-target
    fail-safe, and `Input`-only behavior.
  Evidence:
  - `tests/test_flight_computer.cpp::test_attitude_controller` passed.

- [x] M06-R1-V08 Node executor tests
  Requirement:
  - Test burn time, ignition time, finite fuel consumption, partial final
    throttle, abort, fuel exhaustion, crash/landing inactivation, and no latent
    output.
  Evidence:
  - `tests/test_flight_computer.cpp::test_node_executor` passed.

- [x] M06-R1-V09 Determinism and regression gates
  Requirement:
  - Test repeated identical input/node sequences produce identical results.
  - Run:
    - `cmake -S . -B build`
    - `cmake --build build -j`
    - `ctest --test-dir build --output-on-failure`
    - `git diff --check`
    - headless `lander_gui` smoke
  Evidence:
  - `tests/test_flight_computer.cpp::test_determinism` passed.
  - Ran from `/home/jordan/lunar-lander`:
    - `cmake -S /home/jordan/lunar-lander -B /home/jordan/lunar-lander/build`
      succeeded.
    - `cmake --build /home/jordan/lunar-lander/build -j` succeeded.
    - `ctest --test-dir /home/jordan/lunar-lander/build --output-on-failure`
      reported `100% tests passed, 0 tests failed out of 7`.
    - `git -C /home/jordan/lunar-lander diff --check` produced no output.
    - `SDL_VIDEODRIVER=dummy /home/jordan/lunar-lander/build/lander_gui
      --frames 500 --seed 1 --orbit-demo` printed:
      `final: x=211.575 y=290.938 vx=17.816 vy=-13.469 angle=0.000
      fuel=1000.00 ticks=1271 state=flying score=0 contracts=0 ref=0
      seed=1`

### Human verification

- [ ] M06-R1-H01 Predicted zero-thrust path matches actual path
  Source: USER
  Criterion:
  - When the player does not thrust after observing the prediction, the actual
    flight path matches the displayed zero-thrust prediction.
  Evidence: pending user confirmation

- [ ] M06-R1-H02 Single node editing is clear
  Source: USER
  Criterion:
  - One node can be created, moved, and edited without confusing UI state.
  - Prograde/radial editing produces intuitively different predicted paths.
  Evidence: pending user confirmation

- [ ] M06-R1-H03 Attitude holds are intuitive
  Source: USER
  Criterion:
  - Attitude modes point the craft in the expected directions.
  - Rotation is smooth enough to use and always takes the shortest path.
  Evidence: pending user confirmation

- [ ] M06-R1-H04 Node execution is physical
  Source: USER
  Criterion:
  - EXECUTE NODE physically rotates, waits, and burns.
  - There is no teleport, direct velocity snap, or hidden stabilization.
  - A well-executed burn reasonably matches the predicted ideal node.
  Evidence: pending user confirmation

- [ ] M06-R1-H05 CIRCULARIZE is useful
  Source: USER
  Criterion:
  - CIRCULARIZE creates a sensible node.
  - Executing it produces a useful local orbit around the selected body.
  Evidence: pending user confirmation

- [ ] M06-R1-H06 Transfer planner is useful and physical
  Source: USER
  Criterion:
  - TRANSFER TO OTHER MOON creates an editable future node whose predicted path
    approaches the moving other moon.
  - Execution uses real thrust and gravity, not a shortcut.
  Evidence: pending user confirmation

- [ ] M06-R1-H07 Match-target planner reduces target-relative velocity
  Source: USER
  Criterion:
  - MATCH TARGET VELOCITY creates a sensible node near or at closest approach.
  - Executing it substantially reduces target-relative velocity.
  Evidence: pending user confirmation

- [ ] M06-R1-H08 Prediction readouts behave sensibly
  Source: USER
  Criterion:
  - Predicted PE/AP and closest approach behave sensibly in normal orbits,
    transfers, and degenerate cases.
  Evidence: pending user confirmation

- [ ] M06-R1-H09 No M05 regression
  Source: USER
  Criterion:
  - A full M05 contract flight remains playable with no regression in camera,
    tidal locking, HUD, landing, contract transition, or one-shot debug
    controls.
  Evidence: pending user confirmation

### Derived implementation tasks

- [x] M06-R1-D01 Create M06 milestone, status, project, and task state
  Source: derived
  Files:
  - `PROJECT.md`
  - `STATUS.md`
  - `milestones/M06-flight-computer-and-maneuver-planning.md`
  - `TASKS.md`
  Evidence:
  - M06 milestone created
  - `PROJECT.md` updated
  - `STATUS.md` updated
  - this `TASKS.md` ledger created
- [x] M06-R1-D02 Inspect implementation surface
  Source: derived
  Files:
  - `include/lander/sim.hpp`
  - `src/sim.cpp`
  - `include/lander/binary.hpp`
  - `src/gui.cpp`
  - `CMakeLists.txt`
  - relevant tests
  Evidence:
  - Inspected the M05 simulation, binary system, GUI, CMake targets, and
    existing test surface before implementing M06.
- [x] M06-R1-D03 Refactor shared ballistic propagation and transfer solver
  Source: derived
  Depends: M06-R1-D02
  Files:
  - `include/lander/ballistic.hpp`
  - `src/ballistic.cpp`
  - `include/lander/sim.hpp`
  - `src/sim.cpp`
  Evidence:
  - Added `step_ballistic`, `propagate_ballistic`, `predict_zero_thrust`, and
    `solve_transfer_velocity`.
  - `Simulation::transfer()` now calls the shared pure solver.
  - Added `Simulation::step_once` for GUI fixed-step integration.
- [x] M06-R1-D04 Implement flight computer prediction and node model
  Source: derived
  Depends: M06-R1-D03
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  Evidence:
  - Implemented `ManeuverNode`, `NodeBasis`, `snap_time`, `default_node`,
    `compute_node_basis`, `node_world_dv`, `TrajectoryPrediction`, and
    `predict_trajectory`.
- [x] M06-R1-D05 Implement planners
  Source: derived
  Depends: M06-R1-D04
  Files:
  - `include/lander/flight_computer.hpp`
  - `src/flight_computer.cpp`
  Evidence:
  - Implemented `plan_circularize`, `plan_transfer`, and `plan_match_target`.
- [x] M06-R1-D06 Implement attitude controller and node executor
  Source: derived
  Depends: M06-R1-D04
  Files:
  - `include/lander/autopilot.hpp`
  - `src/autopilot.cpp`
  Evidence:
  - Implemented `AttitudeMode`, `attitude_target_direction`,
    `attitude_input`, `ExecutorState`, and `NodeExecutor`.
- [x] M06-R1-D07 Integrate GUI controls, trajectory rendering, and HUD
  Source: derived
  Depends: M06-R1-D04, M06-R1-D06
  Files:
  - `src/gui.cpp`
  Evidence:
  - Added M06 state, `reset_flight_computer`, destination-pad helper, key
    handlers, fixed-step accumulator loop, per-frame prediction, trajectory
    rendering, flight-computer panel, and legend text.
- [x] M06-R1-D08 Add/extend automated tests and CMake targets
  Source: derived
  Depends: M06-R1-D03, M06-R1-D04, M06-R1-D05, M06-R1-D06
  Files:
  - `tests/test_flight_computer.cpp`
  - `CMakeLists.txt`
  Evidence:
  - Added `tests/test_flight_computer.cpp` and registered
    `lander_flight_computer_tests` in `CMakeLists.txt`.
  - The target passed in the full CTest run.
- [x] M06-R1-D09 Run M06 automated closeout gates
  Source: derived
  Depends: M06-R1-D08
  Evidence:
  - See `M06-R1-V09` for configure, build, CTest, whitespace, and headless
    GUI smoke evidence.
- [x] M06-R1-D10 Update STATUS/TASKS to AWAITING HUMAN VERIFICATION and push
  Source: derived
  Depends: M06-R1-D09
  Evidence:
  - `STATUS.md` and `TASKS.md` set to `AWAITING HUMAN VERIFICATION`.
  - Commit and push are the remaining gate actions for this ledger update.
