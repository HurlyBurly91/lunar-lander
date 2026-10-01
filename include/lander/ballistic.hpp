#pragma once

#include "lander/binary.hpp"

#include <vector>

namespace lander {

// One zero-thrust state on the fixed-step prediction grid. `t` is the
// authoritative simulation time at which `(p, v)` is valid.
struct BallisticState {
    Vec2 p{};
    Vec2 v{};
    double t{};
};

// The number of fixed steps of size `dt` from time `t0` to time `t1`,
// snapped to the fixed-step grid. Negative intervals are clamped to zero.
int ballistic_steps(double t0, double t1, double dt);

// One semi-implicit Euler step through the full two-body inverse-square
// field: `a = gravity(p, t)`, then `v += a dt`, `p += v dt`, `t += dt`.
// This is the same scheme and field ordering as
// `Simulation::integrate_flight` with the main engine and rotation
// disabled, so a zero-input prediction matches the live simulation exactly.
BallisticState step_ballistic(const BinarySystem& bin,
                              const BallisticState& state, double dt);

// Propagate `state` forward by exactly `steps` fixed steps and return the
// endpoint. Pure: neither `state` nor `bin` is modified.
BallisticState propagate_ballistic(const BinarySystem& bin,
                                   const BallisticState& state, int steps,
                                   double dt);

// A decimated zero-thrust trajectory from `state.t` to `end_t` on the
// shared fixed-step grid. The first and last states are always included;
// intermediate states are sampled at a fixed stride chosen to return at most
// about `target_samples` points per branch. Pure.
std::vector<BallisticState> predict_zero_thrust(const BinarySystem& bin,
                                                const BallisticState& start,
                                                double end_t, double dt,
                                                int target_samples = 1024);

// The M05 inter-body transfer solver, extracted as a pure function so the
// one-shot `Simulation::transfer()` developer initializer and the M06
// `TRANSFER TO OTHER MOON` planner can share exactly the same deterministic
// search. It returns the initial inertial velocity that sends a zero-thrust
// arc from `(x0, t0)` to the target body's arrival shell, with full-arc
// terrain clearance, or fails without side effects.
bool solve_transfer_velocity(const BinarySystem& bin, double dt,
                             const Vec2& x0, int source, int target, double t0,
                             Vec2& v_out);

}  // namespace lander
