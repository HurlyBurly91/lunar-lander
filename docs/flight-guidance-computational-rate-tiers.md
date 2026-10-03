# Flight Guidance — Computational Rate Tiers (HOT / WARM / COLD)

This document is the canonical cross-milestone rule for the computational rate
budget of every flight-computer feature in Lunar Lander.

It governs how much work any flight-computer addition may do, and at what rate.

It does not redefine gravity, body motion, ephemerides, or landing collision
rules. Those remain canonical in:

    docs/physics-model-gravity.md

It is the rate-budget umbrella for the specific canonical guidance documents:

    docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
    docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md
    docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md

Each of those documents is a concrete instantiation of the tiers below.

## The three tiers

    HOT — every 1/120 s:
        analytical / feedback computation only
        target O(1)

    WARM — guidance/replanning:
        incremental cached computation
        fixed small iteration/candidate bounds
        target O(1) or O(K) with compile-time-bounded K

    COLD — explicit planning event:
        numerical propagation / shooting permitted
        bounded iterations
        warm-start previous solution whenever available
        never block or advance physical simulation time because computation took
        time

## General principles

- Prefer established aerospace guidance laws whose online form is analytical or
  incremental over generic optimizers.
- Any proposed algorithm whose per-physics-tick work is proportional to the
  entire prediction horizon requires explicit justification and benchmark
  evidence before it is accepted.
- The authoritative simulation timestep (`fixed_dt = 1 / 120 s`) must never be
  stretched, blocked, or advanced because a HOT / WARM / COLD computation took
  time.

## Tier-to-feature mapping

- HOT: bang-bang attitude control, velocity-to-be-gained node execution,
  throttle application, the rolling predictor's per-step shift/append (one
  authoritative step), and manual input.
- WARM: landing guidance update (ZEM/ZEV, bounded t_go scan, phase/terrain
  checks), warm transfer replan (bounded differential correction).
- COLD: cold transfer solve (bounded basin discovery + bounded correction),
  explicit transfer/landing planning events, one-time prediction cold rebuild.

## Rate separation

- The physical control loop runs at 120 Hz.
- WARM guidance may run at a lower rate (e.g. 10-20 Hz) and holds its latest
  result between updates while HOT control continues at 120 Hz.
- COLD planning is event-driven (explicit request, phase entry, or a bounded
  low-rate replan after a meaningful prediction error) and is never a
  per-physics-step operation.

## Implementation traceability

Source regions may annotate their tier so the rate budget stays visible:

    // FLIGHT-COMPUTER TIER: HOT | WARM | COLD
    // Reference: docs/flight-guidance-computational-rate-tiers.md

A region marked HOT must not call into WARM/COLD work. A region marked WARM
must not do horizon-proportional-per-tick work. A region marked COLD must be
event-driven, bounded, warm-started when a previous solution exists, and must
not block or advance simulation time.

## Verification expectations

- HOT work stays O(1) per 1/120 s step (instrumented).
- WARM work stays O(1) / O(K) with bounded K, independent of horizon length.
- COLD work is bounded and warm-started when a previous solution exists, is
  measured (propagation counts / iteration counts), and has no effect on
  simulation dt or blocking.
