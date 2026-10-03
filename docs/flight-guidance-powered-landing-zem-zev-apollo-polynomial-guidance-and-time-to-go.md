# Flight Guidance — Powered Landing with ZEM/ZEV, Apollo-Style Polynomial Guidance, and Bounded Time-to-Go

This document is the canonical cross-milestone reference for powered landing
guidance.

It governs:

- selected-pad approach
- moving hover-waypoint guidance
- braking and capture
- terminal powered descent
- time-to-go selection
- terrain-safe phase transitions

It does not redefine gravity, terrain motion, tidal locking, or landing
collision rules.

Those remain canonical in:

    docs/physics-model-gravity.md

The HOT / WARM / COLD rate budget this document instantiates is defined in:

    docs/flight-guidance-computational-rate-tiers.md

The powered-landing controller must fly the same physical spacecraft used by
manual flight.

No guidance law may teleport, directly assign position/velocity, suppress
gravity, or bypass the ordinary landing checker.

## Computational goal

Powered descent runs in real time.

The intended architecture is:

    rolling authoritative prediction
                ↓
      low-rate landing guidance
                ↓
       desired acceleration
                ↓
      O(1) attitude/throttle
                ↓
            Simulation

The guidance algorithm should be analytical or fixed-small-bound work.

A long-horizon optimizer is not required.

## Historical basis — Apollo lunar landing guidance

A primary historical reference is:

George W. Cherry,
"A Derivation of the Improved Lunar Landing Guidance Equations",
LUMINARY Memo #63,
MIT Instrumentation Laboratory,
27 January 1969.

Cherry derives improved lunar landing guidance equations and discusses the
trade between computational simplicity and dynamical properties.

The Apollo material is important here as engineering precedent:

    powered descent can be guided by repeatedly evaluating a compact
    analytical terminal-guidance law

rather than solving a large general optimization problem every control cycle.

Lunar Lander is not a literal reproduction of the Apollo Guidance Computer or
Apollo powered-descent software.

The project uses the broader principle:

    current state
        +
    desired terminal state
        +
    time-to-go
        ->
    analytical acceleration command

## Research basis — ZEM/ZEV

A directly relevant family is Zero-Effort-Miss / Zero-Effort-Velocity
feedback guidance.

Representative terrain-aware references include:

Liuyu Zhou and Yuanqing Xia,
"Improved ZEM/ZEV feedback guidance for Mars powered descent phase",
Advances in Space Research,
Volume 54, Issue 11, 2014, pages 2446-2455.

DOI:

    10.1016/j.asr.2014.08.011

and:

Yao Zhang, Yanning Guo, Guangfu Ma, Tianyi Zeng,
"Collision avoidance ZEM/ZEV optimal feedback guidance for powered descent
phase of landing on Mars",
Advances in Space Research,
Volume 59, Issue 6, 2017, pages 1514-1525.

DOI:

    10.1016/j.asr.2016.12.040

These papers are useful because they make an important limitation explicit:

    classical unconstrained ZEM/ZEV does not automatically guarantee a
    terrain-safe path

Therefore Lunar Lander combines the analytical terminal guidance with:

- phase logic
- moving hover waypoints
- explicit terrain prediction
- approach/descent corridors
- the authoritative collision/landing system

The project does not copy the more elaborate optimizers from those papers.

## Zero-effort state

At current time:

    t

choose candidate remaining flight time:

    t_go > 0

Then:

    t_f =
        t + t_go

Define the zero-effort future state as the state reached at t_f if no further
main-engine thrust is applied while the canonical gravitational environment
continues normally.

Let:

    p_zero(t_go)
    v_zero(t_go)

be that predicted zero-thrust position and velocity.

The prediction must include:

- both gravitational fields
- future binary motion
- the authoritative simulation timestep/model
- any other canonical physical effect that belongs in zero-effort propagation

M06-R3 maintains a rolling authoritative trajectory specifically so that this
future state can be obtained cheaply.

Do not perform a fresh full long-horizon integration inside every landing
guidance update.

## Moving terminal target

A landing pad is attached to a rotating/moving body.

At:

    t_f = t + t_go

obtain:

    p_target(t_f)
    v_target(t_f)

from the actual moving surface point.

For terminal touchdown:

    p_target =
        future pad position

    v_target =
        future pad surface-point velocity

Do not target:

    current pad world position

as though it were stationary.

## Zero-Effort-Miss

Define:

    ZEM =
        p_target - p_zero

This is the predicted position error at terminal time if no additional thrust
correction is applied.

## Zero-Effort-Velocity

Define:

    ZEV =
        v_target - v_zero

This is the predicted terminal velocity error under zero additional thrust.

## Canonical initial ZEM/ZEV guidance form

The initial M06 powered-landing guidance law is:

    a_cmd =
        6 * ZEM / t_go^2
        -
        2 * ZEV / t_go

This produces a requested corrective thrust acceleration.

The zero-effort state already contains the gravity-induced future evolution.

Therefore:

    DO NOT ADD GRAVITY TO a_cmd AGAIN

Doing so would double-count the gravitational dynamics.

## Converting acceleration command to physical controls

Let:

    a_req =
        |a_cmd|

Then:

    desired_thrust_direction =
        normalize(a_cmd)

and nominal throttle:

    throttle =
        clamp(
            a_req / main_accel,
            0,
            1)

This is a guidance request.

The craft must still physically rotate using the canonical attitude
controller.

If angular error is unsafe for thrust:

    align first

Do not directly apply the commanded acceleration vector to spacecraft state.

The actual acceleration remains whatever Simulation produces from:

    canonical gravity
        +
    physically oriented main-engine thrust

## Time-to-go

Classical powered-descent guidance depends strongly on:

    t_go

M06 should not introduce an unbounded time-of-flight optimizer.

The current estimate is a bias, not a schedule.  The initial estimate is one
local free-fall time from the current pad-relative altitude plus `t_go_min`,
clamped to the configured window:

    t_go_estimate =
        clamp(
            sqrt(2 * altitude / g) + t_go_min,
            t_go_min,
            t_go_max)

The initial canonical method is a tiny deterministic candidate scan.

Let candidate count:

    K

be a small fixed constant.

Evaluate a small deterministic set of K candidates across the configured
`[t_go_min, t_go_max]` window.  The implementation uses a fixed log-spaced
grid over that window; the current time-to-go estimate biases the candidate
cost, but it does not shrink the scan to a neighbourhood of the estimate.

    t_go[0]
    t_go[1]
    ...
    t_go[K-1]

For each candidate:

    obtain p_zero, v_zero from rolling prediction
    obtain future moving target state
    compute ZEM
    compute ZEV
    compute a_cmd
    reject non-finite candidate
    reject candidate exceeding physical thrust feasibility
    reject candidate violating terrain/phase safety
    compute a small deterministic cost

Choose the best valid candidate.

Because K is fixed:

    candidate selection is O(K)

and effectively O(1) with respect to prediction-horizon length.

Do not let K scale with the 120 Hz trajectory sample count.

## Apollo-style polynomial refinement

Apollo-style polynomial/time-to-go guidance is the documented refinement to use
when the constant-acceleration ZEM/ZEV command plus the bounded candidate scan
proves inadequate (for example when the constant command cannot hold a
hovering-like radial thrust for the terminal phase).

It is still a bounded analytical refinement: no general nonlinear optimizer is
introduced.

For the same ZEM/ZEV errors and a candidate `t_go`, the Apollo-style total
acceleration profile over the remaining flight is:

    a(t) =
        a0 + a1 * t

with:

    a0 =
        6 * ZEM / t_go^2
        -
        2 * ZEV / t_go

    a1 =
        6 * ZEV / t_go^2
        -
        12 * ZEM / t_go^3

The guidance update emits a held command over its guidance interval `dt`:

    a_held =
        a0 + a1 * (dt / 2)

The held command is the interval average of the linear Apollo profile. The
throttle is derived from `|a_held|`, and the thrust direction is
`normalize(a_held)`.

The bounded t_go scan remains active. For each candidate:

- reject non-finite candidates
- reject candidates whose planned peak acceleration exceeds `main_accel`;
  the peak is `max(|a0|, |a0 + a1 * t_go|)`
- in DESCENT, reject candidates whose initial command `a0` has an inward
  radial component relative to the target body (a powered soft descent should
  not command the engine to thrust toward the surface)

This refinement intentionally changes the constant-acceleration behavior when
a polynomial profile is physically better; tests must demonstrate the intended
landing behavior rather than equivalence to the older constant command.

## Guidance-rate separation

The physics loop remains:

    120 Hz

Landing guidance need not recompute the entire guidance solution every physics
tick.

A reasonable initial guidance update rate is:

    approximately 10-20 Hz

subject to measurement and playtesting.

Between guidance updates:

    retain the latest desired acceleration / thrust direction

while:

    attitude control
    throttle execution
    physical Simulation

continue at the authoritative 120 Hz rate.

This keeps the high-rate hot path O(1).

## Canonical landing phase structure

Powered landing uses a small explicit state machine.

The canonical conceptual sequence is:

    ASCEND / CLEAR
        ↓
    TRANSFER
        ↓
    CAPTURE / BRAKE
        ↓
    APPROACH
        ↓
    DESCENT
        ↓
    TOUCHDOWN

Not every maneuver must exercise every phase.

Same-body short approaches may skip TRANSFER.

Cross-body landings normally require it.

## ASCEND / CLEAR

A craft leaving a pad or low terrain must first obtain safe geometric
clearance.

Do not immediately target a straight world-space line to a distant pad if that
line passes through terrain or through the body.

The clearance maneuver uses ordinary physical thrust.

## TRANSFER

Long-range/inter-body targeting is provided by the canonical transfer planner:

    docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md

Powered landing does not invent a second competing inter-moon transfer
algorithm.

## CAPTURE / BRAKE

As the vehicle approaches the target body/region:

- reduce target-relative velocity
- establish a physically reachable approach state
- avoid diving through terrain
- replan when meaningful miss develops

This phase may use the same ZEM/ZEV/required-velocity concepts with an
appropriate intermediate target state.

## APPROACH

Do not initially target the physical pad surface from arbitrary geometry.

Construct a co-moving hover/approach waypoint above the selected pad:

    p_hover =
        p_pad
        +
        radial_out * h_approach

The target velocity is the velocity of that corresponding moving point.

The waypoint moves with:

- body centre translation
- body rotation
- pad identity

The approach phase attempts to reach a controlled state near this waypoint.

## DESCENT

Enter DESCENT only when conditions are within deterministic tolerances such as:

    lateral position error
    body-relative altitude
    lateral/tangential relative speed
    vertical/radial relative speed
    attitude feasibility

During DESCENT the target transitions toward the actual pad state.

Touchdown targeting must converge toward:

    pad position
    pad surface velocity

not zero global velocity.

## Terrain and no-subsurface safety

Bare ZEM/ZEV is not the terrain planner.

Terrain safety is provided by the combined architecture:

    phase selection
    hover waypoint
    authoritative future trajectory
    body-relative altitude
    terrain intersection checks
    descent corridor
    existing Simulation collision rules

If the predicted controlled path intersects terrain before intended touchdown:

    do not hide the problem

Instead:

    remain/revert to APPROACH
    replan
    ascend
    or abort

as appropriate.

Never add:

    hidden repulsion force
    terrain clipping exception
    position correction
    velocity correction

to make autoland appear successful.

## Fuel and acceleration feasibility

For each guidance command:

    |a_cmd| <= main_accel

must be checked.

Commands outside physical acceleration capacity indicate that the selected
terminal time/state is infeasible.

The solution is to:

    choose another bounded t_go candidate
    change phase/waypoint
    replan
    or report no solution

not to apply impossible acceleration.

Fuel remains governed by the normal physical Simulation.

## Computational complexity invariant

Excluding the already-maintained rolling prediction:

per landing-guidance update:

    moving target evaluation         O(1)
    zero-effort cache lookup         O(1)
    ZEM                              O(1)
    ZEV                              O(1)
    acceleration command             O(1)
    fixed K time-to-go scan          O(1) for fixed K
    small phase/terrain checks       O(1)

per 120 Hz physical control step:

    attitude controller              O(1)
    throttle application             O(1)

M06 powered landing must not introduce:

    nonlinear MPC
    SQP
    convex optimization
    pseudospectral optimization
    dynamic programming
    reinforcement-learning control
    fresh long-horizon integration every tick

A later milestone may intentionally adopt a more sophisticated method, but that
requires an explicit change to this canonical document.

## Relationship to rolling prediction

The rolling prediction architecture is not merely a UI optimization.

It is an enabling numerical primitive for landing guidance.

The guidance layer should obtain zero-effort future states from the maintained
prediction cache rather than repeatedly recreating them.

This makes:

    ZEM/ZEV evaluation

small enough for real-time repeated use.

If the prediction implementation changes during later milestones, landing
guidance must retain an equivalent cheap authoritative zero-effort-state query.

## Implementation traceability

When powered landing is implemented, the actual guidance-law region must use:

    // BEGIN CANONICAL ALGORITHM: powered landing ZEM-ZEV guidance
    // Reference:
    // docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md

and end with:

    // END CANONICAL ALGORITHM: powered landing ZEM-ZEV guidance

If time-to-go selection is separate, mark it explicitly:

    // BEGIN CANONICAL ALGORITHM: powered landing bounded time-to-go selection
    // Reference:
    // docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md

    ...

    // END CANONICAL ALGORITHM: powered landing bounded time-to-go selection

If approach/terminal target-state construction becomes a substantial separate
algorithmic region, it may receive its own BEGIN/END pair referencing this same
document.

These markers must survive M07 ECS decomposition and later code movement.

## Verification expectations

Deterministic tests should eventually cover:

- zero-effort prediction parity
- pure position-error ZEM response
- pure velocity-error ZEV response
- finite command near minimum allowed t_go
- rejection of physically infeasible acceleration
- moving target position
- moving target velocity
- co-rotating hover waypoint
- PRIMARY landing
- COMPANION landing
- same-body approach
- cross-body approach
- no-subsurface trajectory requirement
- descent-phase entry conditions
- touchdown relative velocity
- real Simulation landing acceptance
- clean abort
- no direct state mutation
- guidance runtime independent of long prediction-horizon length
