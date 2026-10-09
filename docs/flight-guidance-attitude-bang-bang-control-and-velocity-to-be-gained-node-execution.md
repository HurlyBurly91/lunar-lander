# Flight Guidance — Bang-Bang Attitude Control and Velocity-to-Be-Gained Node Execution

This document is the canonical cross-milestone reference for the mathematical
control law used by Lunar Lander for:

- spacecraft attitude hold
- maneuver-vector pointing
- finite-burn maneuver-node execution

Architectural refactors may move or decompose the implementation, but they must
preserve the algorithm described here unless an explicit requirement changes it.

This document does not redefine the physical universe. Gravity, body motion,
body-relative state, and ephemerides remain governed by:

    docs/physics-model-gravity.md

The HOT / WARM / COLD rate budget this document instantiates is defined in:

    docs/flight-guidance-computational-rate-tiers.md

## Design goal

The real-time flight-control hot path must remain extremely small.

At the authoritative 120 Hz physics rate:

    attitude control:
        O(1)

    node execution:
        O(1)

The hot path must not contain work proportional to the prediction horizon,
transfer duration, number of trajectory samples, or any general optimization
problem.

The intended structure is:

    planner
        produces required delta-v
            ↓
    velocity-to-be-gained guidance
            ↓
    bang-bang attitude controller
            ↓
    ordinary physical thrust
            ↓
    Simulation

Planning and control are deliberately separate.

## Research basis

### Bang-bang control of bounded double-integrator dynamics

A useful canonical model for the game's single rotational degree of freedom is:

    theta_dot = omega

    omega_dot = u

with bounded angular acceleration:

    u ∈ {-alpha_max, 0, +alpha_max}

The time-optimal control of an ideal bounded double integrator has bang-bang
structure.

Relevant control literature includes:

Edoardo Serpelloni, Manfredi Maggiore, Christopher Damaren,
"Bang-bang hybrid stabilization of perturbed double-integrators",
Automatica, Volume 69, 2016, pages 315-323.

DOI:

    10.1016/j.automatica.2016.02.028

The Lunar Lander controller is deliberately much simpler than the complete
robust hybrid controller in that paper.

The project adaptation uses the elementary stopping-distance relation for
constant angular deceleration.

If:

    |omega| = current angular speed

and:

    alpha_max = available angular acceleration magnitude

then the angle required to brake to zero angular velocity is:

    theta_stop =
        omega^2 / (2 * alpha_max)

Let:

    e =
        wrap_pi(theta_desired - theta)

be the shortest signed angular error.

The basic switching rule is:

    if rotating toward the target
    and
    theta_stop >= |e|

        brake against current omega

    else

        accelerate toward the target

Small deterministic angle and angular-rate deadbands prevent unnecessary
switching near the target.

This is the canonical mathematical structure.

Exact deadband constants may be tuned from playtesting without changing the
algorithm.

## Spacecraft-angle convention

The game uses:

    thrust_hat(theta) =
        (-sin(theta), cos(theta))

Therefore for desired world-frame thrust direction:

    d = (dx, dy)

the desired spacecraft angle is:

    theta_desired =
        atan2(-dx, dy)

All attitude modes ultimately reduce to supplying a desired world-frame
direction to this controller.

Examples include:

    PROGRADE
    RETROGRADE
    RADIAL OUT
    RADIAL IN
    TARGET
    ANTI-TARGET
    MANEUVER

The source of the direction may change.

The underlying bang-bang attitude controller should not.

## Manual control priority

Manual rotation input has priority for the current physics step.

A manual override must not directly mutate:

    state.angle
    state.omega

It remains ordinary physical rotational input.

When the manual command is released, an enabled attitude mode may resume
control normally.

## Velocity-to-be-gained guidance

A historical precedent for this architecture is:

Frederick H. Martin and Richard H. Battin,
"Computer-Controlled Steering of the Apollo Spacecraft",
Journal of Spacecraft and Rockets,
Volume 5, Number 4, 1968.

The Apollo discussion defines an instantaneous velocity-to-be-gained vector as
the difference between required velocity and present velocity and describes
steering thrust so that this vector is driven toward zero.

Lunar Lander uses the same broad guidance idea for finite execution of an
ideal maneuver node, adapted to the game's simpler engine and deterministic
simulation.

For the node executor define:

    VGO =
        dv_remaining

This is the remaining velocity change that must be produced by the engine.

It is NOT the total difference between spacecraft inertial velocity before and
after a physics step.

Gravity also changes spacecraft velocity.

Therefore VGO accounting subtracts only physically delivered thrust impulse.

For one fixed step:

    dv_thrust =
        thrust_hat
        * main_accel
        * throttle
        * fixed_dt

and:

    VGO_next =
        VGO_current - dv_thrust

Do not subtract gravitational acceleration from VGO.

Do not infer delivered engine delta-v from total spacecraft velocity change.

## Desired burn direction

While VGO is nonzero:

    desired_direction =
        normalize(VGO)

The ordinary attitude controller points the spacecraft thrust vector toward
that direction.

The vector's direction can rotate as each delivered impulse is applied along
the previous step's nose, which lags the vector; the effect is most acute
near the end of a burn, but an off-axis burn is unsafe at any magnitude.
This is not an additional steering law; it is handled by the continuous
alignment safety in the burn-completion rules below, which keeps the engine
off whenever the direction is materially misaligned, so the remaining vector
only ever shrinks.

No direct state mutation is permitted.

In particular, the executor must not directly assign:

    position
    velocity
    angle
    angular velocity

## Finite-burn timing

An ideal maneuver node is an instantaneous planning primitive.

Its physical execution is a finite engine burn.

For initial required delta-v magnitude:

    dv = |VGO_initial|

and constant full-throttle acceleration:

    a = main_accel

the nominal burn duration is:

    burn_time =
        dv / a

The nominal ignition time for a burn centred on the ideal maneuver-node epoch
is:

    ignite_time =
        node_time - burn_time / 2

This is an initial timing estimate.

Physical alignment safety takes priority over blindly meeting the ideal
impulse time.

## Canonical executor state progression

The semantic sequence is:

    ALIGN
        ↓
    READY / WAIT
        ↓
    BURN
        ↓
    COMPLETE

Implementation enum names may differ slightly, but behavior must preserve this
structure.

Immediately after execution is armed:

    begin aligning toward VGO

If alignment completes before nominal ignition:

    hold alignment
    throttle = 0
    wait for ignition

At ignition:

    begin physical burn only if sufficiently aligned

If the craft is late:

    expose LATE status
    continue aligning
    begin burn when alignment is safe

Do not use node time itself as permission to perform a substantially off-axis
burn.

In particular, later implementations must not reintroduce a rule equivalent to:

    aligned || now >= node_time

as an unconditional burn gate.

## Burn completion

During the burn, continue updating:

    VGO

from thrust-delivered impulse.

Normally:

    throttle = 1

until the remaining delta-v fits inside one fixed physics step.

For the final partial step:

    step_dv =
        main_accel * fixed_dt

    throttle =
        clamp(
            |VGO| / step_dv,
            0,
            1)

This prevents a large one-step overshoot.

Complete when VGO falls below a small deterministic tolerance.

Fuel exhaustion produces an incomplete/failure state.

Crash, landing, reset, explicit abort, or other terminating conditions must
remove all latent executor output.

## Continuous alignment safety

Alignment is checked at every physics step, not only at burn entry.

An off-axis impulse is hazardous at every VGO magnitude. When the nose lags
the remaining vector, the delivered impulse can rotate the vector, and the
next step points at the rotated vector; with enough initial misalignment or
angular rate this becomes a self-reinforcing spin in which the tracked VGO
can grow instead of shrink. A re-target can also hand the executor a new
VGO direction while the craft is still rotating. The safety is therefore
continuous and full-range:

- While the remaining VGO is nonzero and the craft is inside the alignment
  band, the executor burns (subject to ignition timing and the final partial
  throttle).
- While the craft is materially misaligned, the executor stops the physical
  thrust and returns to the alignment state, regardless of VGO magnitude:

    BURN  --(materially misaligned)-->  ALIGN
    ALIGN --(aligned, ignition already reached)-->  BURN

Rules:

- The return to ALIGN cuts only the engine command; the attitude controller
  keeps steering toward the current VGO with the unchanged bang-bang law.
- A misaligned step never delivers a thrust impulse, so the remaining VGO
  can only shrink on the aligned burn steps that resume.
- The misalignment band and angular-rate band are deterministic constants
  that may be tuned; they do not relax the entry alignment gate.
- After ignition, the re-entry from ALIGN goes straight back to BURN; the
  executor never returns to WAIT once ignition has been reached.
- The bang-bang attitude law, the VGO accounting, the ignition timing, and
  the final partial throttle are unchanged by this rule.
- This rule is not a permission to burn off-axis; gating the burn on
  alignment checked only once, at entry, is forbidden for the same reason
  as the node-time gate above.

When a higher-rate guidance layer re-arms the node (a re-target) and the
craft is already inside the same strict alignment band used for burn
continuation, with bounded angular rate, the executor may enter BURN
directly, skipping the ALIGN swing. This is an arming convenience only; it
must not use a wider band than the continuous alignment safety, and the
full-range re-entry above applies equally to a continuing burn.

## Complexity invariant

At each 120 Hz physics step the controller may perform operations such as:

    angle wrapping
    vector normalization
    dot products
    magnitude calculation
    stopping-angle calculation
    scalar comparisons
    final throttle clamp

This is O(1).

The attitude/controller hot path must not call:

    long-horizon trajectory prediction
    transfer search
    differential correction
    general root finding
    generic optimization
    MPC
    dynamic programming

Those belong to planning or lower-rate guidance layers.

## Numerical and architectural preservation rules

Later refactors may:

- move these functions into ECS systems
- split control from UI
- change storage representation
- remove temporary M06 classes
- improve caching
- tune deadbands
- add tests
- add instrumentation

They must not silently replace the canonical algorithm with:

- PID-only attitude steering
- arbitrary proportional angle control
- MPC
- direct orientation snapping
- direct velocity assignment
- hidden steering impulses

without an explicit requirement and an update to this document.

## Implementation traceability

Every implementation region containing this canonical algorithm must identify
this document directly in source code.

Bang-bang attitude control regions must use:

    // BEGIN CANONICAL ALGORITHM: bang-bang attitude control
    // Reference:
    // docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md

and end with:

    // END CANONICAL ALGORITHM: bang-bang attitude control

Velocity-to-be-gained node-execution regions must use:

    // BEGIN CANONICAL ALGORITHM: velocity-to-be-gained node execution
    // Reference:
    // docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md

and end with:

    // END CANONICAL ALGORITHM: velocity-to-be-gained node execution

If later refactoring moves the algorithm, move the markers with it.

Do not leave markers around code that no longer implements the documented
algorithm.

## Verification expectations

Regression coverage should preserve at least:

- shortest-angle wrapping
- acceleration toward target
- braking near the stopping boundary
- angular-rate damping near target
- manual override
- no direct state mutation
- early alignment before burn
- no substantially off-axis forced burn
- continuous alignment safety during the burn (thrust only while aligned)
- thrust-only VGO accounting
- final partial throttle
- clean abort
- clean fuel exhaustion
- clean crash/landing/reset behavior

An architectural transition is not complete until equivalent tests still pass.
