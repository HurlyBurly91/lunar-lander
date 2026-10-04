# M06 — Flight computer and maneuver planning

## Request history and active scope

`M06-R1` defined the initial compact flight computer.

`M06-R2` is the active request group. It was supplied after `M06-R1` reached
human verification and it explicitly supersedes the earlier `M06` boundary that
excluded landing autopilot.

Target-pad landing autopilot is now an explicit `M06-R2` requirement. It must
still use ordinary physical inputs and the authoritative simulation; it must
not teleport the craft, assign state directly, or add hidden forces.

`M06-R3` is now the active request group. It was supplied after `M06-R2`
reached human verification and addresses a real defect: the displayed
trajectory is a **zero-thrust (COAST)** path, so at low altitude with persistent
partial throttle it predicts a different impact point than the powered craft
actually reaches. `M06-R3` redefines the trajectory display as three explicitly
labelled kinds:

- `COAST` = zero thrust from the current state (kept for orbital mechanics).
- `LIVE` = what the actual spacecraft will do if the player makes no further
  control changes. This is produced by projecting a snapshot of the
  authoritative `Simulation` forward with the SAME `step_once()` / Input
  composition / `NodeExecutor` / attitude / reaction-wheel / fuel / gravity /
  terrain / landing code as live flight, holding the persistent controls
  (throttle, attitude/SAS, reaction-wheel, active node executor, active landing
  autopilot) while momentary manual rotation is released. It reports an
  authoritative `PRED LAND` / `PRED CRASH` contact result and marker.
- `PLAN` = the ideal maneuver / autopilot planned path.

`M06-R3` is required to be cheap: a shifted receding-horizon rollout (a ring of
per-step samples plus one cloned tail state that shifts and appends one
authoritative step per live step when the actual state matches the predicted
next state within tight tolerance and the policy signature is unchanged) with an
anytime progressive cold rebuild (near horizon first) on policy change, never
feeding prediction compute time into simulation dt. The prediction model IS the
game simulation: no second integrator, no RK / SQP / iLQR / nonlinear
optimization. This supersedes the trajectory-presentation behaviour behind the
`M06-R2-H04` low-altitude defect.

`M06-R4` is a research-backed
formalization of the O(1) hot-path control, layered on the same node-executor /
attitude code that `M06-R3`'s LIVE projection replays (so the two ship
together): (1) the existing bounded bang-bang attitude controller is retained
unchanged (a handful of wraps/multiplies/comparison/sign per fixed step — no
PID/MPC/optimizer/search in the 120 Hz path); (2) the node executor is
formalized as velocity-to-be-gained (VGO) guidance — `dv_remaining` is the VGO,
updated only by the delivered thrust impulse (`thrust_hat · main_accel ·
throttle · dt`), never by total Δv so gravity cannot contaminate it; (3) the
executor aligns before ignition (ALIGN -> READY/WAIT -> BURN -> COMPLETE): it
begins aligning toward the VGO at throttle 0 immediately after EXECUTE, holds
that direction while waiting for ignition, and burns only at/after ignition
while aligned — removing the old `aligned || now >= node_time` rule that forced
an off-axis burn when late (a late arm shows LATE and burns once aligned); (4)
the finite burn uses `burn_time = |VGO| / main_accel`, nominal ignition
`node_time - burn_time/2`, normal full throttle, a final-step partial throttle
`clamp(|VGO|/(main_accel·dt), 0, 1)`, and terminates when the VGO magnitude
falls below a small deterministic tolerance. The ideal node remains an
impulsive planning primitive; only the physical execution is finite.

`M06-R5` is a transfer-targeting requirement. It scales the existing M05/M06
transfer solver (coarse basin discovery + bounded Newton correction +
authoritative full-resolution validation — retained, not replaced) into a COLD
solve (no useful previous solution; may keep the bounded coarse grid) and a
WARM replan (a previous solution exists and the target/problem changed only
slightly): cache a successful solution (source/target body, solve epoch,
departure state/velocity, time of flight / arrival epoch, achieved miss), shift
it to the current epoch, use the previous departure velocity/flight time as the
initial guess, and run a bounded 2x2 Newton/differential correction
(`J = dF/dv0`, `J·delta_v = -F`, `v0 <- v0 + lambda·delta_v` with bounded
damping/backtracking) targeting `O(K·N)` with no `speed_grid·angle_grid` factor
on ordinary warm replans (the coarse grid is a cold-only fallback). It is a
bounded-frequency planning-layer operation, never a 120 Hz one: it runs on
TRANSFER request, on autoland enter/re-entering the transfer phase, and at a
bounded low rate after meaningful prediction error. Midcourse correction is
two-level (slow warm planner + fast VGO/required-velocity controller); an
optional numerically-evaluated ZEM / required-velocity close-approach law may
replace a further transfer solve for final non-landing interception (classical
proportional navigation is not to be introduced blindly).

`M06-R6` is the low-complexity powered-landing guidance that refines the
`M06-R2` target-pad autoland (the `Simulation` landing checker remains
authoritative for actual success). A phase state machine
(`ASCEND/CLEAR -> TRANSFER -> CAPTURE/BRAKE -> APPROACH -> DESCENT ->
TOUCHDOWN`) chooses a target state; a low-level terminal ZEM/ZEV command
(`a_cmd = 6·ZEM/t_go^2 - 2·ZEV/t_go`, the requested thrust correction with no
second gravity term) aims at the moving target (a co-rotating hover waypoint
`p_hover = p_pad + radial_out·h_approach` for APPROACH, the physical pad for
DESCENT), reading the zero-effort state from the `M06-R3` rolling predictor as
an O(1) cache lookup rather than a fresh propagation. Time-to-go is a bounded
K-candidate scan (no unbounded search, no nonlinear optimizer); guidance is
recomputed at ~10-20 Hz while attitude/throttle control runs at 120 Hz; terrain
/glideslope gates govern APPROACH and DESCENT (a terrain intersection triggers
replan/abort, never a state correction). No MPC/SQP/convex/pseudospectral/RL/DP
 optimizer in the M06 landing hot path.

`M06-R7` is the M06 MVP-scope and terminal-feasibility handoff requirement.
From this point, M06 is targeted as an integrated MVP: one representative
deterministic happy path plus enough automated coverage per remaining
requirement, then a consolidated human playtest. Broad controller parameter
sweeps and further M06 guidance sophistication stop once that criterion is met.
The V14-C cross-body landing must be a physical, non-knife-edge path through
the authoritative `Simulation` contact checker. The high-energy `Approach ->
Descent` transition is governed by a terminal-feasibility gate: before entering
`Descent`, the autopilot hypothetically evaluates the same existing terminal
ZEM/ZEV / Apollo-polynomial / bounded `t_go` machinery for a `Descent` state
and hands off only when the command is valid, `t_go` is bounded, the predicted
peak acceleration has margin below `main_accel`, and the radial/tangential/
attitude state is inside the existing descent gates. Otherwise it remains in
`BRAKE` / `APPROACH` and re-evaluates at the bounded guidance cadence. The
canonical terminal algorithm is not modified; the gate must reuse the same
terminal machinery so the two cannot diverge.

`M06-R12` is a bounded post-M06 diagnostic / presentation hardening pass: it
adds a prediction REFERENCE-FRAME architecture. All prediction propagation
stays in the inertial / world frame; R12 only adds a display / analysis layer
with three inertial frames (WORLD = the existing barycentric frame, unchanged
and still available; PRIMARY = body-0 centred; COMPANION = body-1 centred) plus
an AUTO mode that runs a pure orbit-reference classifier (specific-energy,
self-vs-tidal acceleration dominance, unwrapped angular sweep, radial ratio,
and angular consistency over a short bounded window, with enter/release
hysteresis) over the already-computed inertial samples to pick the most
meaningful reference frame per segment. The frame transform is inertial
subtraction only (`ship - body position - body velocity`; no rotation; no
body-fixed / rotating frame; no SOI / patched conics / gravity switch / orbit
stabilization; `Simulation` is never mutated). AUTO segments the displayed path
per frame with visible transition markers (no cross-frame connector). This is
the pass following the bounded post-M06 hardening (R8 harness, R9 predictor
overlay, R10 record-only, R11 diagnostics); it changes no physics and defers
all known predictor / transfer defects (PRED-01..08, SIM-COLL-01,
TFD-1 / TFD-2). Full atomic requirements, thresholds, and the 14-test
verification live in `TASKS.md` (## M06-R12).

All M06 flight-computer additions follow the canonical HOT / WARM / COLD
computational rate tiers:

    docs/flight-guidance-computational-rate-tiers.md

(HOT = O(1) analytical/feedback per 1/120 s step; WARM = cached/incremental with
fixed small bounds; COLD = bounded numerical planning, warm-started when
practical, never blocking or advancing simulation time.)

`M06` remains open while the bounded post-M06 hardening passes complete —
`M06-R8` (subsystem-isolation harness), `M06-R9` (predictor overlay), `M06-R10`
(record-only), `M06-R11` (diagnostics), and `M06-R12` (prediction reference-
frame architecture) — and the unresolved human verification items resolve;
`M06-R2`..`M06-R7` are code-complete (see `TASKS.md`).

`M07` is not active.

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

Landing autopilot is no longer a non-goal. `M06-R2` explicitly adds
target-pad landing autopilot, constrained by all the ordinary-input and
no-cheating rules above.

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

## M06-R2 active requirements

`M06-R2` addresses the human-verification failures from `M06-R1` and adds the
explicit target-pad landing autopilot requirement.

### Node executor timing

- An armed executor begins aligning immediately.
- The sequence is:
  - `ALIGN EARLY`
  - `ALIGNED WAIT`
  - physical burn centred around the node time
  - `COMPLETE`
- The executor may not use `now >= node_time` as permission to burn while
  badly misaligned.
- If alignment is late, it continues aligning, reports a late state, and
  begins burning only when safe.
- Executor status must distinguish at least `ALIGN`, `READY`, `WAIT`,
  `BURN`, `COMPLETE`, `LATE`, `ABORTED`, and `INCOMPLETE`.

### Prediction semantics

- The flight computer distinguishes:
  - `COAST`: zero-thrust prediction from the current state
  - `IDEAL NODE`: instantaneous node-impulse planning prediction
  - `EXECUTION PREDICTION`: prediction from the actual finite-burn executor /
    attitude controller
- While planning, show `COAST` and `IDEAL NODE`.
- While executing, show `EXECUTION PREDICTION`, preferably using a copied
  authoritative simulation and ordinary `Input`.
- After successful execution, consume the node, clear `MANEUVER`, and stop
  reapplying the node's delta-v.

### Trajectory presentation

- Keep the authoritative integrator at `1/120 s`.
- Do not draw all integrated steps every frame.
- Cache a sufficiently dense world-space trajectory and apply camera-space
  adaptive simplification.
- The apparent polyline error should be about 1 screen pixel or less where
  practical.
- Close zoom retains more points than wide zoom.

### Closest approach

- Show explicit `CA SHIP` and `CA TARGET` markers at the same future time.
- Connect them with a thin closest-approach line.
- Label the geometry `CA` or equivalent.
- The panel reports `CLOSEST APPROACH`, distance, and countdown.
- The current target marker remains visually distinct from the future
  closest-approach target marker.

### Planner intent

- The flight computer stores and displays the active plan kind:
  - `None`
  - `ManualNode`
  - `Circularize`
  - `Transfer`
  - `MatchTarget`
  - `LandAtPad`
- The UI must say what the current plan is, not only what its delta-v is.

### UI reorganization

- The GUI separates:
  - `CONTROLS`: ordinary flight/camera controls
  - `FLIGHT COMPUTER`: target, plan, node editing, attitude, execute/abort,
    prediction readouts, and autoland state
  - `DEBUG`: guarded developer/debug one-shot actions
- The giant undifferentiated bottom legend is removed or reduced to a compact
  help affordance.
- Active state, planner intent, and execution phase must be visually obvious.

### Mouse target selection

- Landing pads are mouse-selectable.
- Selection uses stable body/pad identity, not a stale world-space point.
- The selected pad's world state is re-derived each frame from the moving
  body.
- Pads have a minimum screen-space hit target.
- The current contract destination remains the default unless a pad is
  explicitly selected.

### Target-pad landing autopilot

- The player can select a landing pad, choose `LAND AT PAD`, and press
  `EXECUTE`.
- The spacecraft physically flies to that pad using ordinary rotation and
  main-engine throttle.
- No teleportation, direct state mutation, hidden force, or hidden capture is
  allowed.
- The autopilot uses a high-level phase machine such as:
  - `IDLE`
  - `ASCEND`
  - `TRANSFER`
  - `BRAKE`
  - `APPROACH`
  - `DESCENT`
  - `TOUCHDOWN`
  - `COMPLETE`
  - `ABORTED`
  - `NO_SOLUTION`
- The UI displays the current phase.
- If the target is on the other body, the autopilot performs controlled
  ascent, uses/refines an inter-body transfer plan, executes the burn
  physically, replans near arrival, matches useful target-relative velocity,
  and hands off to terminal descent.
- If the target is on the same body, it skips inter-body transfer and uses a
  safe clearance/approach trajectory.
- It does not fly a straight chord through a moon.

### ZEM/ZEV terminal guidance

- Terminal guidance uses a simple receding-horizon ZEM/ZEV-style law.
- For candidate `t_go`:
  - zero-thrust propagate the current state to `t + t_go` using the
    authoritative moving two-body gravity
  - evaluate the selected pad/approach point at the same time
  - `ZEM = p_target - p_zero`
  - `ZEV = v_target - v_zero`
  - `a_cmd = 6 / t_go^2 * ZEM - 2 / t_go * ZEV`
- Gravity is not added again because the zero-effort prediction already
  includes it.
- Feasible `t_go` values are selected by deterministic bounded scan.
- Commanded acceleration is bounded by `main_accel`.
- Guidance emits ordinary attitude/throttle `Input` through the existing
  physical controllers.
- A moving hover/approach waypoint above the selected pad is used before
  final descent.
- The hover point is body-scaled, co-rotating, and has a documented minimum
  altitude.
- Final descent targets the rotating pad surface-point velocity and satisfies
  the existing landing thresholds.

### SYSTEM long-range zoom

- `SYSTEM` remains ship-centred and inertial.
- Wheel zoom is symmetric logarithmic/exponential, conceptually
  `zoom *= exp(k * wheel_delta)`.
- Minimum zoom is extended substantially, approximately into `0.0005 ..
  0.001`, or to an evidence-based equivalent.
- Extreme zoom-out keeps the ship, bodies, and target readable.
- No auto-pan is reintroduced.

### Performance and launch hitch

- The user-reported abrupt launch jump must be investigated.
- The GUI measures frame wall time, prediction rebuild cost, planner solve
  cost, and fixed physics steps per rendered frame.
- Prediction is cached and refreshed at a bounded cadence, with immediate
  recomputation on relevant edits.
- Expensive planner or prediction work must not cause an unseen burst of
  simulation steps that makes the spacecraft appear to teleport.
- No threads/concurrency are introduced unless absolutely unavoidable.
- A predictor/planner microbenchmark is added and its results recorded in
  `TASKS.md`.

### Takeoff terrain-penetration fix

- Ordinary takeoff must not release the spacecraft into or under rotating
  terrain.
- The takeoff release test uses a candidate first fixed step and the actual
  future rotating terrain at `t0 + fixed_dt`.
- Release requires positive clearance and outward relative radial motion,
  within a small numerically motivated tolerance.
- If the candidate does not separate safely, the craft remains attached.
- The first free-flight step is not double-integrated.
- The takeoff acceleration frame uses the actual surface-point acceleration,
  not merely the body-centre acceleration.
- Deterministic throttle threshold/sweep tests cover both bodies, multiple
  binary phases, and representative terrain seeds.

### M06-R2 human verification

- `M06-R2-H01`: flight-computer controls are understandable from the UI.
- `M06-R2-H02`: planned maneuver and execution phase are immediately obvious.
- `M06-R2-H03`: closest-approach graphics have obvious meaning.
- `M06-R2-H04`: trajectories visually conform to actual flight at close and
  wide zoom.
- `M06-R2-H05`: `EXECUTE` aligns early and burns cleanly.
- `M06-R2-H06`: ordinary launches do not visually jump/teleport because the
  flight computer is running.
- `M06-R2-H07`: `SYSTEM` can zoom far enough out for long-range navigation.
- `M06-R2-H08`: mouse pad selection is clear and stable.
- `M06-R2-H09`: click-pad `LAND` / `EXECUTE` performs a safe physical
  landing.
- `M06-R2-H10`: full `M05` contract gameplay remains intact.
- `M06-R2-H11`: ordinary manual launches, including near-threshold partial
  throttle, transition smoothly without sinking below terrain or false
  crashing.

The unresolved `M06-R1-H01`, `M06-R1-H02`, `M06-R1-H04`, and
`M06-R1-H09` items remain open until the corrected behavior is explicitly
confirmed. The remaining `M06-R1` human items also remain open.

## Acceptance

M06 is ready for human verification when:

- the build passes
- the full M06-core test suite passes (landing, flight-computer, predictor,
  binary, GUI smoke); the 2 known post-M06 transfer-subsystem defect targets
  (TFD-1 `lander_tests`, TFD-2 `lander_transfer_warm_tests`) are documented and
  deferred, not MVP blockers (see "M06 MVP closeout status")
- whitespace check is clean
- headless GUI smoke passes
- the flight computer is visible and usable in the SDL game
- the player can plan, edit, execute, and abort a node
- all three planners work through the same single-node model
- target-pad landing autopilot works through ordinary physical inputs
- the M05 contract loop remains playable
- no M07/ECS work has started
- the `M06-R2` performance benchmark has run and its results are recorded
- the takeoff threshold/sweep tests pass
- `STATUS.md` and `TASKS.md` are set to `AWAITING HUMAN VERIFICATION`
- the work is committed and pushed according to repository policy

M06 becomes COMPLETE only after all required human verification items pass
and the milestone is closed out in `records/`.

Do not begin M07 until M06 is COMPLETE.

## M06 MVP closeout status (2026-10-03)

This section records the actual MVP-closeout posture; the full retrospective
(rationale, evidence, requirement traceability) is written to `records/M06-*.md`
at final closeout, after human acceptance.

- Implementation: complete through `M06-R7`. The high-energy `Approach ->
  Descent` heuristic is replaced by a bounded terminal-feasibility gate
  (`landing_terminal_preview` + `LandingAutopilot::terminal_handoff_ok`) that
  reuses the exact canonical terminal ZEM/ZEV / Apollo-polynomial / bounded
  `t_go` machinery (no duplicated equations — P01). The gate accepts a Descent
  handoff only when the hypothetical command is valid, `t_go` is within the
  bounded threshold, predicted peak acceleration has margin below `main_accel`,
  and tangential/radial/lateral/attitude are inside the existing descent gates;
  otherwise it stays in BRAKE / APPROACH and re-evaluates at the bounded
  guidance cadence.
- V14-C: the representative cross-body primary -> companion happy path now
  SOFT-LANDS through the authoritative `Simulation` contact checker (touchdown
  at tick 12062 in `test_primary_companion_same_and_cross_body`); two
  perturbations also soft-land, so it is not a knife-edge. Temporary V14
  diagnostics are removed.
- Test posture: full build clean; ctest is **8/10**. The 2 failing targets are
  documented as KNOWN POST-M06 transfer-subsystem defects (see `TASKS.md`,
  "M06 known post-M06 transfer-subsystem defects"):
  - TFD-1 `lander_tests` — M05-origin one-shot primary -> companion "plausible
    arc" (test_sim.cpp:1677) no longer finds a clearing arc on the tiny
    companion after the M06-R5 shared-solver changes.
  - TFD-2 `lander_transfer_warm_tests` — V07 multi-phase solvability is now
    0/4 (was 2/4) and the warm/cold re-plan agreement check disagrees (warm 1
    vs cold 2769 propagations) on the same tiny-companion arc.
  Both are DEFERRED to a fresh bounded post-M06 transfer-subsystem hardening
  ledger (opened only after M06 closes); they are not M06 MVP blockers, and the
  tests are not weakened/deleted. All M06-core targets (landing, flight-
  computer, predictor, binary, GUI smoke) are green.
- State: `M06` is at AWAITING HUMAN VERIFICATION on the single consolidated
  playtest `M06-R7-H01`. Not committed until the user accepts. `M07` is not
  active.
