# Lunar Lander Harness Benchmark

## Purpose

This is a small C++20 Lunar Lander simulation used to compare autonomous coding
harnesses while holding the language model and inference server constant.

Correctness and engineering process matter more than visual appearance.

## Constraints

- C++20
- CMake
- No third-party libraries
- No network access
- Simulation code must remain independent from rendering/UI code
- Tests must run headlessly
- Floating-point comparisons should use reasonable tolerances
- Deterministic behavior must remain deterministic

## Coordinate convention

- +x = right
- +y = up
- angle 0 = upright
- positive angle = counter-clockwise
- main thrust acts along the lander's local upward axis

## Simulation model

The simulation uses a fixed physics timestep of 1/120 second.

`advance(real_dt, input)` receives elapsed wall/render time. It must accumulate
elapsed time and execute zero or more fixed simulation steps.

Changing the rate at which `advance()` is called must not materially change the
state produced for the same elapsed simulated time and controls.

## Basic dynamics

Gravity acts downward.

Main thrust contributes acceleration according to lander orientation.

Rotational controls modify angular velocity.

Fuel is finite and may never become negative.

A lander contacting a pad is considered safely landed only when all configured
limits are satisfied:

- horizontal speed
- vertical speed
- angular deviation from upright

Ground contact outside a landing pad is a crash.

Unsafe pad contact is a crash.

Once landed or crashed, subsequent simulation advances do not change the
physical state until reset.

## Determinism

Given:

- identical configuration
- identical seed
- identical initial state
- identical sequence of inputs and elapsed times

the resulting simulation state and terrain must be reproducible.
