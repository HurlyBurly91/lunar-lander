# M06 — Flight computer and maneuver planning

## Goal

Turn the completed M05 binary-moon game into a much more playable orbital game
by adding a compact KSP-style flight computer.

M06 adds:

- a deterministic zero-thrust ballistic trajectory predictor
- one maneuver node with a prograde/radial editing frame
- a visible pre-node / post-node predicted trajectory
- SAS-style attitude holds: OFF, PROGRADE, RETROGRADE, RADIAL OUT, RADIAL IN,
  TARGET, ANTI-TARGET, MANEUVER
- a finite-burn node executor that plans, aligns, waits, and burns using
  ordinary rotation, thrust, fuel, and gravity
- three planner actions:
  - CIRCULARIZE
  - TRANSFER TO OTHER MOON
  - MATCH TARGET VELOCITY

The flight computer must make the existing M05 contract loop practical without
turning the game into a hidden autopilot simulator.

The player still flies a real spacecraft.

The planner proposes a maneuver. The player can inspect, edit, execute, or
ignore it. Execution uses the same physical flight model as manual flight.

---

## Non-goals and boundaries

M06 is not M07.

Do not introduce:

- ECS
- modular spacecraft
- a second physics engine
- patched conics
- sphere-of-influence switching
- nearest-body-only gravity
- hidden capture forces
- hidden orbit stabilization
- teleportation
- direct velocity assignment by normal player-facing autopilot
- direct position assignment by normal player-facing autopilot
- time warp
- multiple maneuver nodes
- 3D flight
- background threads or concurrency
- a generic maneuver-planning framework

The authoritative simulation remains the existing M05 `Simulation`.

The flight computer is a presentation and planning layer around that
simulation.

It may predict.

It may propose.

It may emit ordinary `Input`.

It must not secretly change the physical rules.

---

## Canonical physics constraint

M06 must preserve the M05 canonical physics model:

- both moons exert gravity on the spacecraft at all times
- the binary ephemeris is analytic and deterministic
- both bodies are tidally locked to the binary line of centres
- terrain and pads move with their bodies
- surface contact and landing use body-relative, rotating-surface velocity
- world coordinates and velocity remain in the shared inertial frame
- the reference body is a presentation/gameplay frame, not a gravity switch

Do not disable, weaken, or patch either gravitational field to make planner
output look cleaner.

The other body's gravity may make a "circular" orbit non-perfect. That is
correct.

---

## Authoritative prediction

The trajectory predictor must use the same ballistic integration rule as live
flight.

Live flight uses fixed-step semi-implicit / symplectic Euler:

    a = binary.gravity(ship_position, sim_time)
    v = v + a * dt
    x = x + v * dt
    t = t + dt

with:

    dt = Config::fixed_dt = 1.0 / 120.0

The M06 predictor must use the same integration order and the same two-body
gravity function.

It must not use RK4 as the authoritative displayed predictor.

It must not approximate the binary ephemeris with a frozen body position.

It must evaluate both body positions and velocities at the prediction time
using the existing analytic `BinarySystem` model.

It must not mutate:

- `Simulation`
- `State`
- fuel
- contract state
- reference body
- binary phase
- terrain
- camera
- UI state

Prediction is a pure function of:

- current `State` snapshot
- current simulation time
- `Config`
- `BinarySystem`
- optional maneuver node
- prediction horizon

The existing live-flight stepping code and the predictor should share a
refactored propagation helper so they cannot silently diverge.

That refactor must preserve the existing M05 deterministic behavior.

The existing `T x3` one-shot transfer initializer must continue to work after
the transfer solver is refactored.

---

## Prediction horizon and sampling

The default prediction horizon is:

    horizon = 2 * binary.period()

With the M05 constants, that is approximately 433.9 s.

The predictor integrates internally at the authoritative fixed-step rate:

    120 Hz

Presentation samples may be decimated.

The total number of stored trajectory points should remain bounded, preferably
roughly 512 to 1024 points per trajectory branch.

The exact decimation may be implemented by selecting every Nth integrated
sample, where N is chosen to keep the stored vector near the desired size.

Do not change the integration step to make rendering cheaper.

Render-time interpolation is acceptable for drawing, but the predicted data
must come from the fixed-step prediction.

---

## Single maneuver node

M06 supports exactly one active maneuver node.

A maneuver node is a planned instantaneous velocity change applied to the
predicted spacecraft state at a future time.

The node stores:

- `time`: predicted simulation time of the node, snapped to a fixed-step
  boundary
- `frame_body`: the body used to define the node's prograde/radial basis
- `dv_prograde`: signed Δv component along the node prograde axis
- `dv_radial`: signed Δv component along the node radial-out axis

The node does not store a hidden third normal-axis component.

The world-frame Δv vector is reconstructed from the node basis:

    dv_world = dv_prograde * prograde_hat + dv_radial * radial_out_hat

The node is an ideal planning primitive.

The pre-node branch ends at the node time.

At the node, position is unchanged and velocity is changed by `dv_world`.

The post-node branch begins from that modified state.

This is the same conceptual model used by KSP-style maneuver nodes.

The executor later approximates this ideal impulse with a finite thrust burn.

---

## Node basis

The node basis is computed from the predicted pre-burn state relative to the
node's `frame_body`.

Let:

    t_pre = node.time
    p_pre = predicted ship position at t_pre with no node applied
    v_pre = predicted ship velocity at t_pre with no node applied
    b_pos = frame_body position at t_pre
    b_vel = frame_body velocity at t_pre
    r = p_pre - b_pos
    v = v_pre - b_vel

Define:

    h = cross2(r, v) = r.x * v.y - r.y * v.x

The nominal basis is:

    r_hat = normalize(r)
    prograde_hat = normalize(v)
    radial_candidate = r_hat - dot(r_hat, prograde_hat) * prograde_hat
    radial_out_hat = normalize(radial_candidate)

If:

    dot(radial_out_hat, r_hat) < 0

then flip `radial_out_hat`.

The resulting `prograde_hat` and `radial_out_hat` must be orthonormal.

Degenerate cases must be handled deterministically.

If the velocity vector is too small to define prograde, use a tangent vector
derived from `r`:

    if h >= 0:
        tangent_hat = normalize(-r.y, r.x)
    else:
        tangent_hat = normalize(r.y, -r.x)

If `r` is also too small to define a radial direction, fall back to a
deterministic world-frame orthonormal basis without producing NaNs.

The same degenerate-frame logic should be reused for current-state attitude
holds so that the UI and node basis behave consistently.

No basis vector may depend on wall-clock time, random values, or floating-point
uninitialized state.

---

## Predicted trajectory

The predicted trajectory consists of two branches:

1. pre-node branch:
   - starts from the current actual state
   - integrates zero-thrust, zero-spin physics to the node time
   - if there is no node, this is the only branch

2. post-node branch:
   - starts from the pre-node endpoint
   - applies the node's ideal `dv_world`
   - integrates zero-thrust, zero-spin physics to the prediction horizon

The post-node branch is what lets the player see the effect of the planned
maneuver.

The predictor must also compute:

- terrain impact time/body, if the predicted path enters a body's terrain
  before the horizon
- closest approach to the moving destination pad
- predicted periapsis / apoapsis values, `PE` and `AP`, as numeric local
  extrema of the body-relative distance curve

Terrain impact must use the future position and tidal rotation of the relevant
body, not a frozen body.

Closest approach to the destination pad must use the actual moving surface
point:

    target_position(t) =
        body_position(t)
        + rotate(pad_local_position, body_rotation(t))

The closest-approach result should report:

- distance
- time
- target position at that time

The predicted `PE` and `AP` are not classical invariant orbital elements.

They are local extrema of:

    rho(t) = |ship_position(t) - frame_body_position(t)|

over the post-node predicted branch.

A reasonable deterministic implementation is:

- sample `rho(t)` at the fixed prediction grid
- find local minima for `PE` and local maxima for `AP`
- reject noise below a small significance threshold
- report the earliest suitable extremum after the node, or `--` if none exists
  before the horizon end

Do not compute full conic orbital elements and do not present them as exact
physical orbits.

They are predictions in the real M05 two-body/tidal environment.

---

## Attitude modes

M06 adds SAS-like attitude hold modes.

Available modes:

- OFF
- PROGRADE
- RETROGRADE
- RADIAL OUT
- RADIAL IN
- TARGET
- ANTI-TARGET
- MANEUVER

OFF produces no autopilot rotation.

PROGRADE points along the current body-relative velocity.

RETROGRADE points opposite current body-relative velocity.

RADIAL OUT points away from the current reference body centre.

RADIAL IN points toward the current reference body centre.

TARGET points toward the current contract destination pad.

ANTI-TARGET points away from the current contract destination pad.

MANEUVER points along the active node executor's remaining Δv vector while a
burn is being set up or executed.

If a mode's target vector is undefined or degenerate, the autopilot must fail
safely to OFF-like behavior and the UI should show a visible reason or a
clear `--` state.

The current reference body is used for PROGRADE / RETRO / RADIAL modes.

The node's `frame_body` is used for node editing and node basis display, but
current-state attitude holds should follow the active reference body to match
the existing HUD.

---

## Attitude controller

The attitude controller is a bounded double integrator for the existing
spacecraft pitch/rotation state.

The live equations are:

    theta_dot = omega
    omega_dot = u

where:

    u ∈ {-config.rotate_accel, 0, +config.rotate_accel}

The controller must use the same rotation inputs as manual flight.

It must not directly write `state.angle` or `state.omega`.

For a desired world direction `d`, the desired spacecraft angle is:

    theta_desired = atan2(-d.x, d.y)

because the existing thrust direction is:

    thrust_hat(theta) = {-sin(theta), cos(theta)}

The controller uses the shortest angular error:

    error = wrap_pi(theta_desired - state.angle)

A simple deterministic bang-bang law is sufficient:

- if the rate is large enough that braking is needed before reaching the
  target angle, brake against the current rate
- otherwise accelerate toward the target angle
- use small angular and rate deadbands to avoid jitter

A suitable braking test is:

    if sign(omega) == sign(error)
       and omega * omega / (2 * rotate_accel) >= |error|:
       brake

Manual rotation input overrides the autopilot for that step.

The autopilot does not set main throttle except through the node executor.

---

## Node executor

The node executor turns an ideal node into a finite burn using ordinary
thrust.

It is a small state machine.

States:

- IDLE
- WAIT
- ALIGN
- BURN
- COMPLETE
- ABORTED
- INCOMPLETE

Arming an executor from a node computes:

    dv_total = node dv_world
    burn_time = |dv_total| / config.main_accel
    t_ignite = node.time - 0.5 * burn_time

If:

    now < t_ignite

the executor starts in WAIT.

If:

    now >= t_ignite

the executor starts in ALIGN and is marked late.

During WAIT and ALIGN, the executor commands attitude toward the current
remaining Δv direction but does not command main thrust.

During BURN, the executor commands:

- rotation toward the remaining Δv direction
- main throttle

The burn uses normal fuel consumption.

The ship can crash, miss, run out of fuel, or land while executing.

The executor must not cheat.

It must not apply hidden force, modify velocity directly, or teleport.

The remaining Δv is the thrust-produced Δv still needed, not the total
spacecraft velocity error.

Each fixed step during burn should update it approximately as:

    dv_remaining -= thrust_hat(state.angle)
                     * config.main_accel
                     * throttle
                     * fixed_dt

The final step should use partial throttle:

    throttle_needed =
        |dv_remaining| / (config.main_accel * fixed_dt)

clamped to `[0, 1]`.

If fuel runs out before completion, the executor becomes INCOMPLETE.

If the user aborts, the executor becomes ABORTED.

If the ship crashes, lands, or the game resets, the executor must become
inactive and leave no latent autopilot output behind.

A completed or aborted executor must not continue commanding rotation or
thrust.

---

## Planners

M06 provides exactly three planners.

Each planner creates or edits the single maneuver node.

None of the three planners automatically executes the node.

None of the three planners directly mutates live `State`.

### CIRCULARIZE

CIRCULARIZE creates a node that makes the spacecraft approximately circular
around the node's frame body at the node time.

This is an osculating local circularization, not a hidden stabilized orbit.

From the predicted pre-burn state relative to the frame body:

    r = p_pre - b_pos
    v = v_pre - b_vel
    rho = |r|
    v_circ = sqrt(mu_frame / rho)

The tangent direction is chosen from the sign of angular momentum:

    h = cross2(r, v)

    if h >= 0:
        tangent_hat = normalize(-r.y, r.x)
    else:
        tangent_hat = normalize(r.y, -r.x)

The desired body-relative circular velocity is:

    v_circ_vec = v_circ * tangent_hat

The desired world velocity is:

    v_desired = b_vel + v_circ_vec

The node's world Δv is:

    dv_world = v_desired - v_pre

The planner stores that as prograde/radial components in the node basis.

The other body's gravity remains active after execution.

The resulting orbit may perturb.

That is acceptable.

### TRANSFER TO OTHER MOON

TRANSFER TO OTHER MOON reuses the existing M05 transfer solver.

The M05 `Simulation::transfer()` behavior must be refactored into a pure
solver that can be called from the planner without mutating live state.

The planner solves from the predicted pre-burn state at the node time.

If no node exists yet, it creates one at a sensible default node time, such as
current time + 5 s snapped to a fixed-step boundary.

The solver's output is a desired departure velocity at the node time that
reaches the moving destination body/shell under the real M05 two-body
gravity.

The node's Δv is:

    dv_world = solved_departure_velocity - predicted_ship_velocity_at_node

The existing `T x3` debug initializer may keep its current one-shot behavior,
but it should call the same refactored solver so the planner and the debug
initializer do not diverge.

If the solver finds no valid transfer, the planner must not corrupt the node.

It should leave the existing node unchanged or report no solution.

### MATCH TARGET VELOCITY

MATCH TARGET VELOCITY creates a node whose Δv matches the spacecraft's
world velocity to the moving destination pad's world velocity.

This is a velocity-matching planner, not a landing autopilot.

If no node exists, the planner should place the node near the predicted
closest approach to the moving destination pad.

If a node exists, it uses the existing node time.

The target velocity is the velocity of the actual moving surface point:

    v_target =
        body_center_velocity
        + omega_spin cross r_local_world

The node's Δv is:

    dv_world = v_target - predicted_ship_velocity_at_node

The planner stores that as prograde/radial components in the node basis.

Executing this node should make the spacecraft arrive near the destination
pad with small target-relative velocity, but the player still must land
deliberately.

---

## Code organization

M06 should add small dedicated modules rather than dumping all logic into
`src/gui.cpp` or `Simulation`.

Expected new or refactored boundaries:

- `include/lander/ballistic.hpp` / `src/ballistic.cpp`
  - shared fixed-step propagation
  - pure transfer solver
  - trajectory sampling helpers

- `include/lander/flight_computer.hpp` / `src/flight_computer.cpp`
  - maneuver node type
  - node basis
  - predicted trajectory
  - closest approach
  - predicted PE/AP
  - planners

- `include/lander/autopilot.hpp` / `src/autopilot.cpp`
  - attitude modes
  - attitude controller
  - node executor state machine

`Simulation` remains the authoritative physical state owner.

`FlightComputer` is a pure prediction/planning layer.

`Autopilot` emits ordinary `Input`.

`src/gui.cpp` handles SDL input mapping, rendering, HUD, and legend text.

This boundary is intended to make the later M07 ECS decomposition easier, but
M06 itself must not become an ECS implementation.

---

## GUI and controls

The GUI must expose the flight computer clearly.

Required visible states:

- node exists / no node
- node countdown
- node prograde/radial/total Δv
- predicted burn time
- executor state
- attitude mode
- predicted closest approach to the destination pad
- predicted PE/AP
- no-solution / invalid-frame indicators when relevant

The predicted trajectory should be rendered when the navigation overlay is
visible.

Use distinguishable colors:

- pre-node baseline: neutral grey/white
- post-node path: green or blue
- node marker: clear diamond or cross
- predicted terrain impact: red marker
- closest approach to destination: amber marker

The existing M05 controls remain intact.

New M06 controls should use currently unused keys and be documented in the
in-game legend.

A suitable control layout is:

- `C`: create or re-place the node 5 seconds ahead, snapped to a fixed step
- `Shift+C`: create or re-place the node using the other body as frame body
- `Delete`: remove the node
- `H`: node time -1 s
- `J`: node time +1 s
- `K`: node prograde Δv +0.1 m/s
- `Shift+K`: node prograde Δv -0.1 m/s
- `L`: node radial Δv +0.1 m/s
- `Shift+L`: node radial Δv -0.1 m/s
- `U`: plan CIRCULARIZE into the node
- `I`: plan TRANSFER TO OTHER MOON into the node
- `Y`: plan MATCH TARGET VELOCITY into the node
- `1`: attitude OFF
- `2`: attitude PROGRADE
- `3`: attitude RETROGRADE
- `4`: attitude RADIAL OUT
- `5`: attitude RADIAL IN
- `6`: attitude TARGET
- `7`: attitude ANTI-TARGET
- `8`: attitude MANEUVER
- `Return`: EXECUTE NODE
- `Shift+Return`: ABORT executor
- `X`: engine cutoff; also aborts an active executor

While an executor is active, manual main-throttle input should abort the
executor.

Manual rotation input may override attitude for that step, but does not by
itself cancel the executor unless the chosen control semantics require it.

After crash, landing, reset, or new contract, the node, executor, and
attitude state must reset cleanly.

---

## Determinism

M06 must remain deterministic under the existing fixed-step model.

For the same:

- seed
- initial state
- input sequence
- autopilot state sequence
- node editing sequence

the flight computer must produce the same prediction, planner output,
attitude decisions, and executor behavior.

No random values may affect prediction, planning, attitude, or execution.

No wall-clock time may enter the physics or planning logic.

Render time may only affect presentation interpolation.

---

## Automated verification

Automated tests must cover at least:

1. Predictor parity:
   - a zero-input `Simulation` clone and the pure predictor produce matching
     positions/velocities over the prediction horizon within deterministic
     tolerance
   - changing either body's `mu` changes the predicted path

2. Shared integrator refactor:
   - existing M05 deterministic tests still pass
   - `T x3` one-shot transfer still changes velocity but not position
   - live simulation and predictor use the same shared stepping helper

3. Node basis:
   - prograde/radial are normalized and orthogonal in normal cases
   - radial-out sign points outward relative to the frame body
   - degenerate zero-velocity and near-radial cases produce finite
     deterministic orthonormal frames

4. Node impulse:
   - with a nonzero node, the pre-node branch is unchanged from the no-node
     prediction until the node time
   - position is continuous across the node
   - velocity changes by the reconstructed `dv_world`
   - the post-node branch differs deterministically after the node

5. Trajectory prediction:
   - terrain impact uses future body position and tidal rotation
   - closest approach to the moving destination pad uses the moving surface
     point
   - predicted PE/AP are numeric local extrema of `rho(t)` and report `--`
     or equivalent when no suitable extremum exists

6. Planners:
   - CIRCULARIZE produces a node whose ideal post-node state has reduced
     radial velocity error relative to a local circular state
   - TRANSFER TO OTHER MOON reuses the same solver as `T x3` and produces no
     node corruption when no solution exists
   - MATCH TARGET VELOCITY produces a node whose ideal post-node world
     velocity matches the moving destination pad velocity at the node time

7. Attitude controller:
   - wraps to the shortest angular path
   - converges from initial angle/rate combinations
   - brakes before overshooting in high-rate cases
   - emits only `Input`, never direct `State` mutation
   - fails safely on undefined target vectors

8. Node executor:
   - finite burn duration equals `|dv| / main_accel`
   - ignition time is `node_time - 0.5 * burn_time`
   - burn consumes fuel and applies thrust through ordinary `Input`
   - final step uses partial throttle
   - abort, fuel exhaustion, crash, and completion leave no latent output
   - no direct position/velocity mutation occurs during execution

9. Determinism:
   - repeated runs with the same scripted inputs and node actions produce the
     same final state and executor state

10. Regression:
    - full M05 test suite passes
    - build, CTest, whitespace check, and headless GUI smoke pass

---

## Human verification

Human verification items for M06:

- M06-R1-H01:
  - the displayed zero-thrust prediction matches the actual path when the
    player does not thrust after the prediction is shown

- M06-R1-H02:
  - one node can be created, moved, and edited without confusing UI state
  - prograde/radial editing produces intuitively different predicted paths

- M06-R1-H03:
  - attitude hold modes point the craft in the expected directions
  - rotation is smooth enough to use and always takes the shortest path

- M06-R1-H04:
  - EXECUTE NODE physically rotates, waits, and burns
  - there is no teleport, direct velocity snap, or hidden stabilization
  - the result reasonably matches the predicted ideal node when executed well

- M06-R1-H05:
  - CIRCULARIZE creates a sensible node
  - executing it produces a useful local orbit around the selected body

- M06-R1-H06:
  - TRANSFER TO OTHER MOON creates an editable future node whose predicted
    path approaches the moving other moon
  - execution uses real thrust and gravity, not a shortcut

- M06-R1-H07:
  - MATCH TARGET VELOCITY creates a sensible node near or at closest approach
  - executing it substantially reduces target-relative velocity

- M06-R1-H08:
  - predicted PE/AP and closest-approach readouts behave sensibly in normal
    orbits, transfers, and degenerate cases

- M06-R1-H09:
  - a full M05 contract flight remains playable with no M05 regression in
    camera, tidal locking, HUD, landing, contract transition, or one-shot
    debug controls

Do not mark M06 complete until these human verification items are explicitly
confirmed.

---

## Acceptance

M06 is ready for human verification when:

- the build passes
- the full test suite passes
- whitespace check is clean
- headless GUI smoke passes
- the flight computer is visible and usable in the SDL game
- the player can plan, edit, execute, and abort a node
- all three planners work through the same single-node model
- the M05 contract loop remains playable
- no M07/ECS work has started
- `STATUS.md` and `TASKS.md` are set to `AWAITING HUMAN VERIFICATION`
- the work is committed and pushed according to repository policy

M06 becomes COMPLETE only after all required human verification items pass
and the milestone is closed out in `records/`.

Do not begin M07 until M06 is COMPLETE.
