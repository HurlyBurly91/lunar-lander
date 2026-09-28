# M02 — Extend an Existing Lander

## Goal

Extend the existing working implementation without regressing its current
behavior.

This milestone deliberately requires changes spanning state, physics,
simulation control, tests, and scoring.

## 1. Rotational inertia

Replace the effective fixed angular-acceleration model with torque divided by
moment of inertia.

Add these public `Config` fields:

    double rotation_torque{1.6};
    double moment_of_inertia{1.0};

For one rotational thruster:

    angular_acceleration = rotation_torque / moment_of_inertia

A moment of inertia twice as large must produce approximately half the angular
acceleration for otherwise identical conditions.

## 2. Multiple landing pads

Generate exactly three deterministic, non-overlapping pads from the simulation
seed.

Each pad has a multiplier:

    1
    2
    3

Pad generation must be reproducible.

A successful landing scores:

    100 * pad.multiplier

## 3. Input recording and replay

Add:

    struct InputFrame {
        double real_dt;
        Input input;
    };

Add these public `Simulation` methods:

    void start_recording();
    std::vector<InputFrame> stop_recording();
    void replay(const std::vector<InputFrame>& frames);

Requirements:

- recording begins empty
- every subsequent `advance()` call is recorded
- `stop_recording()` disables recording and returns the sequence
- `replay()` applies the supplied frames to the current simulation state
- replay itself must not recursively append frames to a recording
- reset + replay of a recorded deterministic session must reproduce its final
  state

## Acceptance criteria

- All pre-existing tests continue to pass.
- Add useful visible tests for the new behavior.
- Rotational inertia affects angular acceleration correctly.
- Exactly three deterministic pads exist.
- Pads do not overlap.
- Pad multipliers are 1, 2, and 3.
- Landing score follows the pad multiplier.
- Recorded input can reproduce final state after reset.
- `git diff --check` passes.

## Closeout

Write `records/M02-result.md`.
