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

// M06-R5: a cached inter-body transfer solution — the warm-start record. A
// successful (cold) solve stores the source/target bodies, the solve epoch
// and departure state/velocity, the time of flight / arrival epoch, and the
// achieved terminal miss. A later WARM replan targets the cached absolute
// arrival epoch from the current epoch: the remaining flight duration (and
// its period fraction) shrinks as time advances, while the previous
// departure velocity seeds the bounded differential correction instead of
// re-running the coarse grid.
struct TransferSolution {
    bool valid{false};
    int source{-1};
    int target{-1};
    double solve_epoch{0.0};     // t0 the solution was solved from
    Vec2 departure_state{};      // x0
    Vec2 departure_velocity{};   // solved initial velocity at the node
    double time_of_flight{0.0};  // arrival_epoch - solve_epoch
    double arrival_epoch{0.0};   // t1
    double achieved_miss{0.0};   // terminal miss at the authoritative step
    double fraction{0.0};        // flight time as a fraction of the period
    int newton_iterations{0};    // differential-correction iterations used (warm)
    // M06-R6: predicted target-relative arrival speed (the arc's endpoint
    // velocity relative to the target body's inertial velocity at the arrival
    // epoch). Populated on a successful solve; the lower-energy handoff ranks
    // bounded flight-time candidates by this value among those that pass the
    // hard gates, and the terminal de-orbit capture reads it to decide when to
    // bleed off excess approach energy before the ordinary braking profile.
    double arrival_rel_speed{0.0};
};

// The M05 inter-body transfer solver, extracted as a pure function so the
// one-shot `Simulation::transfer()` developer initializer and the M06
// `TRANSFER TO OTHER MOON` planner can share exactly the same deterministic
// search. It returns the initial inertial velocity that sends a zero-thrust
// arc from `(x0, t0)` to the target body's arrival shell, with full-arc
// terrain clearance, or fails without side effects.
//
// M06-R5: the optional `out_solution` (defaulted, so existing call sites are
// unaffected) lets a caller capture the accepted solution's flight time /
// arrival epoch / achieved miss as a `TransferSolution` cache entry for a
// later warm replan. When null, the solver's behavior is exactly as before.
bool solve_transfer_velocity(const BinarySystem& bin, double dt,
                              const Vec2& x0, int source, int target, double t0,
                              Vec2& v_out, TransferSolution* out_solution =
                                  nullptr);

// M06-R20-F01: read-only canonical arrival-shell target for a transfer epoch.
// Computes exactly the goal point used by the accepted COLD / WARM terminal
// miss (the target surface clearance shell on the side facing the source at
// the arrival epoch), without solving, propagating, or mutating any state.
// Returns false for invalid body indices or coincident centres.
bool transfer_arrival_target(const BinarySystem& bin, int source, int target,
                             double arrival_epoch, Vec2& goal_out);

// M06-R5-02: bounded 2x2 Newton / differential correction on the terminal
// error F(v0) = propagate(x0, v0, t0, steps, dt) - goal. A central-difference
// Jacobian J = dF/dv0, the linear solve J·delta_v = -F, and the damped update
// v0 <- v0 + lambda·delta_v with backtracking; a small fixed iteration maximum
// (no arbitrary convergence) and deterministic singular-J handling (a
// near-zero determinant stops with `singular_jacobian` set). Pure.
struct NewtonCorrectionResult {
    bool converged{false};
    bool singular_jacobian{false};
    int iterations{0};
    double final_miss{0.0};
    Vec2 v0{};
};
NewtonCorrectionResult differential_correction(
    const BinarySystem& bin, double dt, const Vec2& x0, double t0, int steps,
    const Vec2& goal, Vec2 v0, int max_iters, double accept_miss,
    double newton_step);

// M06-R5-01 / M06-R23: a warm-started transfer replan. It targets the
// previously accepted solution's absolute arrival epoch from the current
// epoch, seeds the departure velocity from the previous solution, runs a
// bounded differential correction (M06-R5-02), and validates authoritatively
// (a full 1/120 propagation plus the same terrain-clearance gate the cold
// solver uses). The remaining flight duration (and its period fraction)
// shrinks as the solve epoch advances; if the remaining horizon falls below
// the minimum solve horizon, the result is invalid and the caller's bounded
// cold fallback selects a later feasible epoch. It never repeats the coarse
// speed/direction grid (M06-R5-03), so an ordinary warm replan is O(K·N) with
// a small fixed K.
// Pure; returns a valid solution or a valid=false result that the caller
// turns into a cold-solve fallback.
TransferSolution solve_transfer_warm(const BinarySystem& bin, double dt,
                                      const Vec2& x0, int source, int target,
                                      double t0, const TransferSolution& prev);

// M06-R5 telemetry: zero-thrust-propagation counters for the transfer solvers.
// `ballistic_reset_propagation_count` zeroes the running count; a subsequent
// cold or warm solve accumulates into it, and `ballistic_propagation_count`
// returns the number of full trajectory propagations the last solve performed.
// Used to verify the warm solver's O(K·N) cost bound (M06-R5-02/V02) and to
// record the cold-vs-warm benchmark (M06-R5-V09). It has no effect on the
// solver's result.
void ballistic_reset_propagation_count();
int ballistic_propagation_count();

}  // namespace lander
