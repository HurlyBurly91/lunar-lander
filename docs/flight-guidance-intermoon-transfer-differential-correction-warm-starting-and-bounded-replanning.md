# Flight Guidance — Inter-Moon Transfer Differential Correction, Warm Starting, and Bounded Replanning

This document is the canonical cross-milestone reference for numerical
inter-moon transfer targeting.

It governs:

- transfer departure-velocity targeting
- cold transfer searches
- differential correction
- warm-started replanning
- bounded numerical work
- final authoritative trajectory validation

It does not redefine gravity.

The physical model remains canonical in:

    docs/physics-model-gravity.md

The HOT / WARM / COLD rate budget this document instantiates is defined in:

    docs/flight-guidance-computational-rate-tiers.md

In particular:

- both moons exert gravity continuously
- there is no sphere-of-influence switch
- there are no patched conics
- body motion uses the canonical binary ephemeris
- world metres and seconds remain physical simulation units

## Design goal

The transfer planner may be numerically heavier than the 120 Hz control loop,
but it must remain:

    deterministic
    bounded
    physically honest
    warm-startable
    separate from the hot control path

The normal architecture is:

    cold or warm numerical targeting
                ↓
         desired departure state
                ↓
             node / VGO
                ↓
        O(1) physical executor

Transfer solving is an event-driven or low-rate planning operation.

It is not a per-physics-step operation.

## Research basis — differential correction in a multi-body problem

A directly relevant research precedent is:

Meibo Lv, Minghu Tan, Daming Zhou,
"Design of two-impulse Earth-Moon transfers using differential correction approach",
Aerospace Science and Technology,
Volume 60, 2017, pages 183-192.

DOI:

    10.1016/j.ast.2016.11.008

That work considers Earth-Moon transfers in the circular restricted
three-body problem and uses an initial estimate followed by Newton-Raphson
differential correction to obtain an accurate initial state.

Lunar Lander is not claiming to simulate the physical Earth-Moon CR3BP.

The reusable idea is:

    nonlinear multi-body propagation
        +
    terminal error
        +
    local sensitivity to initial conditions
        +
    bounded Newton-style correction

The game's canonical binary gravity and terrain rules remain its own model.

## Research basis — warm starting / real-time iteration

A second relevant principle comes from:

Moritz Diehl, Hans Georg Bock, Johannes P. Schlöder,
"A Real-Time Iteration Scheme for Nonlinear Optimization in Optimal Feedback Control",
SIAM Journal on Control and Optimization,
Volume 43, Issue 5, pages 1714-1736.

DOI:

    10.1137/S0363012902400713

The project does NOT implement that paper's full nonlinear MPC machinery.

The relevant principle is temporal coherence:

    a nearby problem should reuse and refine a nearby previous solution

rather than repeatedly cold-solving the entire search space.

That principle is canonical for transfer replanning.

## Lunar Lander transfer formulation

Let:

    x0 =
        spacecraft position at departure epoch

    v0 =
        candidate departure velocity

    t0 =
        departure epoch

    T =
        chosen flight duration

Let the authoritative ballistic propagator under the canonical binary field
produce:

    x(t0 + T ; x0, v0)

Let:

    x_target(t0 + T)

be the desired moving arrival point or arrival shell position.

Define terminal position error:

    F(v0) =
        x(t0 + T ; x0, v0)
        -
        x_target(t0 + T)

For planar targeting:

    F : R^2 -> R^2

We seek:

    F(v0) ≈ 0

subject to:

- bounded departure speed
- actual future body motion
- terrain clearance
- deterministic flight-time bounds
- authoritative final validation

## Differential correction

Approximate the 2x2 sensitivity matrix:

    J =
        dF / dv0

using deterministic finite differences around the candidate departure
velocity.

For perturbation epsilon:

    J[:,0] ≈
        (F(v0 + (epsilon,0)) - F(v0 - (epsilon,0)))
        /
        (2 * epsilon)

    J[:,1] ≈
        (F(v0 + (0,epsilon)) - F(v0 - (0,epsilon)))
        /
        (2 * epsilon)

Solve:

    J * delta_v =
        -F(v0)

Then propose:

    v_trial =
        v0 + lambda * delta_v

where:

    0 < lambda <= 1

Use bounded deterministic damping/backtracking if the full Newton step does not
reduce miss.

Use a small fixed maximum correction count.

Never iterate indefinitely waiting for arbitrary convergence.

Singular or badly conditioned Jacobians must fail deterministically and fall
back to an allowed alternative.

## Cold solve

A cold solve is permitted when there is no useful nearby previous solution.

The existing M05/M06 approach of bounded basin discovery is appropriate.

A cold solve may use:

    small fixed set of candidate flight durations
        ×
    bounded speed samples
        ×
    bounded direction samples

to locate promising basins.

Only a small fixed number of best basins should enter the more expensive
differential-correction stage.

The cold search is a fallback.

It is not the normal replanning path after a valid solution already exists.

All loop bounds must remain explicit and deterministic.

## Warm solve

After a successful transfer solve, retain enough information to warm-start a
nearby future solve.

Useful cached solution information includes:

    source body
    target body
    solve epoch
    departure position
    departure velocity
    flight duration
    arrival epoch
    terminal miss
    accepted trajectory family / basin identity if useful

When:

- source and target are unchanged
- only a small amount of simulation time has elapsed
- current state remains near the expected state
- the previous solution remains physically relevant

start from the previous solution.

A warm replan targets the cached absolute arrival epoch.

As the solve epoch advances, the remaining flight duration and its period
fraction shrink. The solver does not roll a fixed TOF horizon forward, because
the target body is moving and a rolling horizon keeps moving the arrival shell
and can leave a persistent one-step correction.

The previous departure velocity is the warm-start seed. It sits in the same
velocity basin as the current solution, so the bounded correction converges
even when the craft is off the nominal arc (for example mid injection burn).
Seeding instead from the raw current state velocity starts the correction in
the wrong basin when the craft is far from the arc and drives repeated
cold-fallback re-targets, so it is not used.

If the fixed-epoch correction fails, or if the remaining horizon falls below
the minimum solve horizon, a bounded cold fallback may select a new later
feasible arrival epoch.

That fallback is bounded by a terminal-completion rule (bounded retarget
persistence / safe-boundary retargeting). Once the guided craft reaches the
destination body's clearance shell, the transfer objective -- a finite passage
through the target region -- is achieved. The midcourse completes at that
boundary: it disengages the slow planner and the fast correction so the craft
coasts through, and it does not run another cold fallback that would drag the
arrival epoch forward to a later one. Without this rule the cold fallback
reselects a later epoch at every reached epoch, the arrival recedes
indefinitely, and the repeated re-aims (the last a degenerate large impulse)
drive the craft into the companion.

The rule is a HOT evaluation on every 1/120 s fixed physics step, not part of
the bounded-rate WARM replan: it compares the post-step ship state against the
companion's closed-form position at the post-step time and against a
per-destination cached clearance shell (the terrain's maximum-surface-radius
scan is O(samples) and is run at most once per arming cycle, never in the
120 Hz loop). Per-step evaluation means neither a burn-state gate nor a
not-yet-due replan interval can preempt completion, and because the completion
commands no thrust it cannot violate the full-range alignment-safety
invariant.

Ordinary warm replanning should be:

    previous solution
        ↓
    epoch/state adjustment
        ↓
    bounded differential correction
        ↓
    authoritative validation

Do not repeat the full speed-angle coarse search merely because the craft
advanced one frame or one guidance update.

## Complexity target

For a warm solution let:

    N =
        number of integration steps in one candidate propagation

    K =
        small fixed differential-correction iteration bound

The intended complexity is approximately:

    O(K * N)

with K fixed and small.

The important invariant is removal of the large multiplicative cold-search
grid from ordinary replanning.

Cold solving may be more expensive:

    O(F * S * A * Ncold)
        +
    bounded correction

where F, S, and A are explicitly bounded candidate counts.

Cold solving remains event-driven.

Neither cold nor warm transfer targeting belongs in the 120 Hz control path.

## Separation between planning and correction

The numerical planner answers:

    what departure / transfer state should we aim for?

The real-time executor answers:

    what physical thrust should be produced now?

Small physical execution errors should normally be handled by cheap
velocity-to-be-gained guidance or another bounded local correction law.

Do not invoke a complete cold transfer solve for every tiny trajectory error.

If predicted terminal miss grows beyond a meaningful threshold:

    request a warm replan

If warm correction fails:

    optionally perform a bounded cold fallback

If no acceptable trajectory exists:

    report NO SOLUTION

Never teleport or directly assign the required departure velocity.

## Arrival geometry

The target is moving.

Target evaluation must use the future canonical body ephemeris.

When targeting a body surface shell or landing approach location, its future
position must respect the canonical body rotation and surface geometry.

Do not freeze the target at its current location.

Do not solve a nominal two-body Hohmann transfer and then hide the discrepancy.

## Terrain validation

A numerically good terminal miss is not sufficient.

Before accepting a solution, validate its full trajectory against the actual
future terrain geometry at authoritative simulation resolution where required.

A transfer that numerically reaches the arrival target but intersects either
body is invalid.

Terrain-clearance validation is a hard physical gate.

It is not merely a cost penalty.

## Authoritative final pass

Coarser propagation may be used to discover or refine a basin.

Before a transfer solution becomes player-facing or executable, validate it
using the authoritative fixed simulation step:

    fixed_dt = 1 / 120 s

and the same canonical gravity function used by live flight.

The final accepted solution must satisfy:

    finite state
    speed bounds
    terminal miss tolerance
    terrain clearance
    deterministic reproducibility

## Algorithms explicitly not canonical for M06 transfer targeting

Do not silently replace this solver with:

    patched conics
    sphere-of-influence switching
    nearest-body gravity
    pure Hohmann approximation
    exhaustive global search every replan
    stochastic optimization
    genetic algorithms
    generic nonlinear MPC
    direct state mutation

A later explicit requirement may replace the algorithm, but the replacement
must update this canonical document in the same work.

## Implementation traceability

Every implementation region containing transfer targeting mathematics must
reference this file.

Use:

    // BEGIN CANONICAL ALGORITHM: inter-moon transfer differential correction
    // Reference:
    // docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md

and:

    // END CANONICAL ALGORITHM: inter-moon transfer differential correction

If useful, individual regions may be marked more specifically:

    // BEGIN CANONICAL ALGORITHM: inter-moon transfer cold basin discovery
    ...
    // END CANONICAL ALGORITHM: inter-moon transfer cold basin discovery

    // BEGIN CANONICAL ALGORITHM: inter-moon transfer differential correction
    ...
    // END CANONICAL ALGORITHM: inter-moon transfer differential correction

    // BEGIN CANONICAL ALGORITHM: inter-moon transfer authoritative validation
    ...
    // END CANONICAL ALGORITHM: inter-moon transfer authoritative validation

All such regions reference this document.

When M07 or later milestones move these algorithms into new systems, the
markers move with the implementation.

## Verification expectations

Regression tests should cover:

- PRIMARY -> COMPANION
- COMPANION -> PRIMARY
- multiple binary phases
- deterministic repeatability
- successful differential correction from a good initial estimate
- deterministic failure on singular/unusable correction
- terrain rejection
- final authoritative-resolution validation
- warm solve from a nearby previous solution
- warm solve requiring materially fewer trajectory evaluations than cold solve
- bounded iteration counts
- no mutation of live spacecraft state
- no transfer solve in the 120 Hz hot control path

Performance instrumentation should count trajectory propagations rather than
only wall-clock time so algorithmic regressions remain visible across machines.
