// M06-R5 transfer-targeting tests: the warm-started differential-correction
// replan (M06-R5-01/02/03), the bounded-failure fallback (M06-R5-02/V03), the
// no-live-state-mutation invariant (M06-R5-P05/V04), the bidirectional and
// multi-phase solve coverage (M06-R5-04/V06/V07), and the cold-vs-warm
// benchmark record (M06-R5-V09).
//
// Run headlessly; no SDL dependency. The cold solver (`solve_transfer_velocity`)
// and the warm solver (`solve_transfer_warm`) are both pure, so every test
// constructs a canonical `BinarySystem`, runs the solver(s), and asserts on the
// returned values and the propagation-count telemetry hook.
#include "lander/autopilot.hpp"
#include "lander/ballistic.hpp"
#include "lander/binary.hpp"
#include "lander/flight_computer.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>

namespace {

using lander::BallisticState;
using lander::BinarySystem;
using lander::Config;
using lander::Input;
using lander::ManeuverNode;
using lander::NewtonCorrectionResult;
using lander::NodeBasis;
using lander::State;
using lander::TransferMidcourse;
using lander::TransferSolution;
using lander::Vec2;
using lander::Simulation;
using lander::ballistic_propagation_count;
using lander::ballistic_reset_propagation_count;
using lander::companion_seed;
using lander::compute_node_basis;
using lander::default_node;
using lander::differential_correction;
using lander::kPi;
using lander::plan_transfer;
using lander::propagate_ballistic;
using lander::solve_transfer_velocity;
using lander::solve_transfer_warm;
using lander::snap_time;

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) { return std::abs(a - b) <= eps; }

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

void check_close_vec(lander::Vec2 a, lander::Vec2 b, double eps,
                     const char* message) {
    if (std::hypot(a.x - b.x, a.y - b.y) > eps) {
        ++failures;
        std::printf("FAIL: %s ((%.12g, %.12g) != (%.12g, %.12g), eps %.3g)\n",
                    message, a.x, a.y, b.x, b.y, eps);
    }
}

bool finite(const Vec2& v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

const double kDt = 1.0 / 120.0;

// A departure position near the surface of body `body` at epoch `t0`, at the
// given altitude above the terrain at the given surface arc.
Vec2 surface_departure(const BinarySystem& bin, int body, double t0, double alt,
                       double arc) {
    const lander::Body& b = bin.body(body);
    const double th = b.terrain.angle_at_arc(arc);
    const double rr = b.terrain.surface_radius_at_arc(arc) + alt;
    const Vec2 pos = bin.position(body, t0);
    return {pos.x + rr * std::cos(th), pos.y + rr * std::sin(th)};
}

// Search a small grid of departure positions near the source body for one that
// the COLD solver accepts, and capture the resulting TransferSolution. Returns
// true on success. Used to obtain a deterministic, geometry-robust warm-start
// record for the other tests (the exact solvable departure depends on the
// canonical terrain, so we find it rather than hard-coding a fragile one).
bool find_cold(const BinarySystem& bin, int source, int target, double t0,
               Vec2& x0_out, TransferSolution& sol_out,
               int max_searches = 35) {
    static const double kAlts[] = {60, 90, 120, 150, 200, 250, 300};
    static const double kArcs[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    int tried = 0;
    for (double alt : kAlts) {
        for (double arc : kArcs) {
            if (tried >= max_searches) {
                return false;
            }
            ++tried;
            const Vec2 cand = surface_departure(bin, source, t0, alt, arc);
            TransferSolution sol{};
            Vec2 v{};
            if (solve_transfer_velocity(bin, kDt, cand, source, target, t0, v,
                                        &sol)) {
                x0_out = cand;
                sol_out = sol;
                return true;
            }
        }
    }
    return false;
}

// ---- M06-R5-V01: a warm replan converges to the same basin as the cold solve.
void test_warm_matches_cold() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    check(find_cold(bin, 0, 1, t0, x0, cold), "a cold 0->1 transfer exists");
    if (!cold.valid) {
        return;
    }

    // Advance the craft one fixed step along the solved arc to model "the craft
    // has moved; replan from here."
    const BallisticState s0{x0, cold.departure_velocity, t0};
    const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
    const double t1 = t0 + kDt;

    const TransferSolution warm =
        solve_transfer_warm(bin, kDt, s1.p, 0, 1, t1, cold);
    check(warm.valid, "the warm replan is valid");
    if (!warm.valid) {
        return;
    }
    // Same accepted basin: the same flight-time fraction and an arrival epoch
    // / miss / departure velocity within a small tolerance of the cold solve.
    check_close(warm.fraction, cold.fraction, 1e-9,
                "warm uses the same flight-time basin (fraction)");
    check_close(warm.arrival_epoch, cold.arrival_epoch, 2.0,
                "warm arrival epoch within 2 s of cold");
    check_close(warm.achieved_miss, cold.achieved_miss, 10.0,
                "warm terminal miss within 10 m of cold");
    check_close_vec(warm.departure_velocity, cold.departure_velocity, 3.0,
                    "warm departure velocity within 3 m/s of cold");
    check_close(warm.achieved_miss, cold.achieved_miss, 12.0,
                "warm stays in the cold solver's acceptance bound");
}

// ---- M06-R5-V02: a warm solve uses far fewer full propagations than cold.
void test_warm_fewer_propagations() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    check(find_cold(bin, 0, 1, t0, x0, cold), "a cold 0->1 transfer exists");
    if (!cold.valid) {
        return;
    }
    const BallisticState s0{x0, cold.departure_velocity, t0};
    const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
    const double t1 = t0 + kDt;

    // Count full trajectory propagations for each path. The propagation counter
    // resets between the two solves so each measurement is independent.
    lander::ballistic_reset_propagation_count();
    Vec2 cv{};
    const bool cold_ok =
        solve_transfer_velocity(bin, kDt, x0, 0, 1, t0, cv);
    const int cold_prop = lander::ballistic_propagation_count();
    check(cold_ok && cold_prop > 0, "cold solve ran and was counted");

    lander::ballistic_reset_propagation_count();
    const TransferSolution warm =
        solve_transfer_warm(bin, kDt, s1.p, 0, 1, t1, cold);
    const int warm_prop = lander::ballistic_propagation_count();
    check(warm.valid && warm_prop > 0, "warm solve ran and was counted");

    std::printf("  [R5-V02] cold propagations=%d  warm propagations=%d  "
                "ratio=%.1fx\n",
                cold_prop, warm_prop,
                cold_prop > 0 ? double(cold_prop) / double(warm_prop) : 0.0);
    // The warm path is O(K·N) with a small fixed K; it must use a small
    // fraction of the cold path's coarse-grid workload.
    check(warm_prop > 0 && warm_prop * 5 < cold_prop,
          "warm uses <1/5 of the cold solve's propagations");
}

// ---- M06-R5-V03: a bounded failure never exceeds the iteration bound, is
// deterministic, and the warm replan's failure falls back to a cold solve.
void test_bounded_failure() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));

    // (a) differential_correction on a far goal with a one-step budget and a
    // tight gate stops at the iteration bound with a finite velocity, and is
    // deterministic. From a cold (0,0) seed a single Newton step reaches ~4.6 m
    // here; that is well short of the 0.1 m gate, so the correction must respect
    // the one-step bound and stop (not converged) rather than diverge. (Two or
    // more steps polish the same goal to <1e-3 m, so the bound is what makes
    // this a bounded failure, not an unreachable goal.)
    {
        const double t0 = 0.0;
        const Vec2 x0 = surface_departure(bin, 0, t0, 120.0, 0.0);
        const int steps = 500;
        const Vec2 goal{1000.0, 1000.0};  // far goal, polished by >=2 Newton steps
        const Vec2 seed{0.0, 0.0};
        const int max_iters = 1;
        const double gate = 0.1;
        const NewtonCorrectionResult a =
            differential_correction(bin, kDt, x0, t0, steps, goal, seed,
                                    max_iters, gate, 0.25);
        const NewtonCorrectionResult b =
            differential_correction(bin, kDt, x0, t0, steps, goal, seed,
                                    max_iters, gate, 0.25);
        check(a.iterations <= max_iters, "bounded: did not exceed max iterations");
        check(finite(a.v0), "bounded: corrected velocity is finite");
        check(!a.converged,
              "bounded: a one-step budget against a tight gate does not converge");
        check(a.v0.x == b.v0.x && a.v0.y == b.v0.y &&
                  a.iterations == b.iterations,
              "bounded: the correction is deterministic");
    }

    // (b) the warm replan fails deterministically when the cached record does
    // not match the requested route, and the caller's cold fallback still
    // solves it.
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    check(find_cold(bin, 0, 1, t0, x0, cold), "a cold 0->1 transfer exists");
    if (cold.valid) {
        TransferSolution mismatched = cold;
        mismatched.source = 1;  // corrupt the record so warm must reject it
        mismatched.target = 0;
        const TransferSolution bad_warm =
            solve_transfer_warm(bin, kDt, x0, 0, 1, t0, mismatched);
        check(!bad_warm.valid,
              "warm with a mismatched cached record fails (no hang, no throw)");
        Vec2 fb{};
        const bool fallback_ok =
            solve_transfer_velocity(bin, kDt, x0, 0, 1, t0, fb);
        check(fallback_ok, "cold fallback still solves the 0->1 transfer");
        check_close_vec(fb, cold.departure_velocity, 1e-6,
                        "cold fallback reproduces the cold solve (deterministic)");
    }
}

// ---- M06-R13: independent free-flight differential-correction regression.
// A zero-mass BinarySystem makes propagate an exact free flight: with zero
// acceleration the integrator keeps velocity constant, so
//     p_final = p0 + v0*T
// exactly (no gravity, no curvature). The terminal error
//     F(v0) = p0 + v0*T - goal
// is therefore exactly linear in v0 with Jacobian J = T*I. A deliberately
// wrong seed must be driven to the known exact targeting velocity
// v_target = (goal - p0)/T in a single Newton step.
//
// This is an independent oracle for the finite-difference convention: a
// reversed derivative sign makes the damped update step the wrong way (the
// trial miss grows, every backtrack is rejected, and the correction stalls on
// the seed), and a transposed Jacobian is exposed by the 48-phase real-field
// sweep (whose gravity Jacobian is not symmetric). Neither defect can be
// hidden by retuning kAcceptMiss, because here the exact answer is known.
void test_free_flight_linear_correction() {
    const BinarySystem bin{};  // every body massless -> zero field everywhere
    const double t0 = 0.0;
    const int steps = 600;  // T = 5.0 s at the 1/120 s fixed step
    const double T = steps * kDt;
    const Vec2 x0{10.0, 20.0};
    const Vec2 v_target{3.0, -4.0};
    const Vec2 goal{x0.x + v_target.x * T, x0.y + v_target.y * T};
    const Vec2 v_seed{v_target.x + 1.5, v_target.y + 0.75};  // deliberately wrong

    const NewtonCorrectionResult r =
        differential_correction(bin, kDt, x0, t0, steps, goal, v_seed, 8,
                                1.0e-9, 0.25);
    check(r.converged, "free flight: a wrong seed converges to the target");
    check(!r.singular_jacobian, "free flight: the T*I Jacobian is not singular");
    check(r.iterations <= 2,
          "free flight: an exactly-linear F converges in one Newton step");
    check(r.final_miss < 1.0e-9, "free flight: terminal miss reaches ~zero");
    check_close_vec(r.v0, v_target, 1.0e-9,
                    "free flight: corrected velocity equals the exact target");

    // Independent of the correction path: propagating the corrected velocity
    // as a free flight must land exactly on the goal (p0 + v0*T).
    const BallisticState end =
        propagate_ballistic(bin, BallisticState{x0, r.v0, t0}, steps, kDt);
    check_close_vec(end.p, goal, 1.0e-9,
                    "free flight: p0 + v0*T lands on the goal exactly");
}

// ---- M06-R5-V04: solving and warm-replanning never mutate live state.
void test_no_mutation() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    check(find_cold(bin, 0, 1, t0, x0, cold), "a cold 0->1 transfer exists");
    if (!cold.valid) {
        return;
    }

    // Snapshot the inputs and the binary's body positions before the replan.
    const Vec2 x0_before = x0;
    const Vec2 p0_before = bin.position(0, t0);
    const Vec2 p1_before = bin.position(1, t0);
    const Vec2 v0_before = cold.departure_velocity;

    const BallisticState s0{x0, cold.departure_velocity, t0};
    const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
    solve_transfer_warm(bin, kDt, s1.p, 0, 1, t0 + kDt, cold);

    check(x0_before.x == x0.x && x0_before.y == x0.y,
          "departure position unchanged by the solver");
    check_close_vec(bin.position(0, t0), p0_before, 1e-12,
                    "primary position unchanged after solve/replan");
    check_close_vec(bin.position(1, t0), p1_before, 1e-12,
                    "companion position unchanged after solve/replan");
    check(v0_before.x == cold.departure_velocity.x &&
              v0_before.y == cold.departure_velocity.y,
          "the cached record's departure velocity is unchanged");
}

// ---- M06-R5-V06: both directions solve (cold and warm).
void test_bidirectional() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double t0 = 0.0;

    // 0 -> 1 (PRIMARY -> COMPANION)
    {
        Vec2 x0{};
        TransferSolution cold{};
        check(find_cold(bin, 0, 1, t0, x0, cold), "cold PRIMARY->COMPANION");
        if (cold.valid) {
            const BallisticState s0{x0, cold.departure_velocity, t0};
            const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
            const TransferSolution warm =
                solve_transfer_warm(bin, kDt, s1.p, 0, 1, t0 + kDt, cold);
            check(warm.valid, "warm PRIMARY->COMPANION");
        }
    }
    // 1 -> 0 (COMPANION -> PRIMARY)
    {
        Vec2 x0{};
        TransferSolution cold{};
        check(find_cold(bin, 1, 0, t0, x0, cold), "cold COMPANION->PRIMARY");
        if (cold.valid) {
            const BallisticState s0{x0, cold.departure_velocity, t0};
            const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
            const TransferSolution warm =
                solve_transfer_warm(bin, kDt, s1.p, 1, 0, t0 + kDt, cold);
            check(warm.valid, "warm COMPANION->PRIMARY");
        }
    }
}

// ---- M06-R5-V07: the transfer solves across several binary phases.
void test_multi_phase() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double period = bin.period();
    int solved = 0;
    int warm_checked = 0;
    // The transfer must solve across the binary's orbit, not only at one
    // epoch. Each phase searches a bounded number of departures (so a phase
    // with no clearing window is abandoned quickly rather than exhausting the
    // full grid); a warm replan is additionally confirmed at the first
    // phase that solves.
    for (double frac : {0.0, 0.25, 0.5, 0.75}) {
        const double t0 = frac * period;
        Vec2 x0{};
        TransferSolution cold{};
        if (find_cold(bin, 0, 1, t0, x0, cold, /*max_searches=*/8)) {
            ++solved;
            if (warm_checked == 0) {
                const BallisticState s0{x0, cold.departure_velocity, t0};
                const BallisticState s1 = propagate_ballistic(bin, s0, 1, kDt);
                const TransferSolution warm =
                    solve_transfer_warm(bin, kDt, s1.p, 0, 1, t0 + kDt, cold);
                check(warm.valid, "warm replan succeeds at a sampled phase");
                ++warm_checked;
            }
        }
    }
    check(solved >= 2, "the 0->1 transfer solves across multiple phases");
    std::printf("  [R5-V07] 0->1 solved at %d/4 sampled phases "
                "(warm confirmed at %d)\n",
                solved, warm_checked);
}

// ---- M06-R5-D04: the planner's warm path. A cold plan seeds the cache; a
// re-plan of a slightly perturbed state with the same cache must take the warm
// (bounded differential-correction) path and cost far fewer zero-thrust
// propagations than an equivalent cold search. `find_cold` supplies a known
// solvable departure so the test is robust to the chosen phase.
void test_plan_transfer_warm_path() {
    Config cfg{};
    const auto bin =
        BinarySystem::canonical(cfg.mu, 503ULL, companion_seed(503ULL));
    const double dt = cfg.fixed_dt;

    // Find a known-solvable 0->1 departure (position + velocity at t0=0).
    Vec2 x0{};
    TransferSolution seed{};
    if (!find_cold(bin, 0, 1, 0.0, x0, seed)) {
        return;  // no transfer at this phase; the warm-path claim is N/A.
    }

    // A flying State at t0 with the solvable departure position.
    State s0{};
    s0.x = x0.x;
    s0.y = x0.y;
    s0.vx = seed.departure_velocity.x;
    s0.vy = seed.departure_velocity.y;
    s0.fuel = 1000.0;
    s0.landed = false;
    s0.crashed = false;

    // Node at "now" so the pre-burn propagation is zero and the departure
    // position fed to the solver is exactly x0.
    ManeuverNode node = default_node(0.0, 0, dt);
    node.time = snap_time(0.0, dt);

    // First plan: empty cache -> the cold search fills the cache.
    TransferSolution cache{};
    std::optional<ManeuverNode> n1 =
        plan_transfer(bin, cfg, s0, 0.0, 0, node, &cache);
    check(n1.has_value(), "cold plan_transfer found a transfer");
    check(cache.valid && cache.source == 0 && cache.target == 1,
          "cold plan seeded the warm cache for the 0->1 route");
    if (!n1.has_value()) {
        return;
    }

    // Small drift: nudge the departure position and re-plan at the same node.
    State s1 = s0;
    s1.x += 0.1;
    s1.y += 0.1;

    // Equivalent COLD re-plan (forced by an empty cache) as the baseline.
    TransferSolution cold_cache{};
    ballistic_reset_propagation_count();
    std::optional<ManeuverNode> ncold =
        plan_transfer(bin, cfg, s1, 0.0, 0, node, &cold_cache);
    const int cold_prop = ballistic_propagation_count();

    // WARM re-plan from the same state with the seeded cache.
    ballistic_reset_propagation_count();
    std::optional<ManeuverNode> nwarm =
        plan_transfer(bin, cfg, s1, 0.0, 0, node, &cache);
    const int warm_prop = ballistic_propagation_count();

    check(nwarm.has_value(), "warm plan_transfer found a transfer");
    check(ncold.has_value() == nwarm.has_value(),
          "cold and warm re-plans agree on solvability");
    if (ncold && nwarm) {
        check(std::abs(ncold->time - nwarm->time) < 1e-9,
              "re-plan keeps the node time");
        check(std::abs(ncold->dv_prograde - nwarm->dv_prograde) < 3.0 &&
              std::abs(ncold->dv_radial - nwarm->dv_radial) < 3.0,
              "warm re-plan stays near the cold delta-v");
    }
    check(warm_prop < cold_prop, "warm re-plan costs fewer propagations");
    std::printf("  plan_transfer re-plan: warm %d vs cold %d propagations\n",
                warm_prop, cold_prop);
}

// ---- M06-R5-V09: record the cold-vs-warm benchmark.
void test_benchmark() {
    const auto bin =
        BinarySystem::canonical(lander::Config{}.mu, 503ULL,
                                companion_seed(503ULL));
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    if (!find_cold(bin, 0, 1, t0, x0, cold)) {
        std::printf("  [R5-V09] no cold transfer; skipping benchmark\n");
        return;
    }
    const BallisticState s0{x0, cold.departure_velocity, t0};
    // Shift by one fixed step: the normal per-planning-cycle replan cadence.
    // For the well-conditioned warm-start seed this lands within the acceptance
    // bound with zero Newton iterations (a genuinely cheap replan); the bounded
    // Newton loop itself is exercised separately in test_bounded_failure.
    const int kShiftSteps = 1;
    const BallisticState s1 = propagate_ballistic(bin, s0, kShiftSteps, kDt);
    const double t_shift = t0 + kShiftSteps * kDt;

    auto t_c0 = std::chrono::steady_clock::now();
    Vec2 cv{};
    lander::ballistic_reset_propagation_count();
    const bool cold_ok =
        solve_transfer_velocity(bin, kDt, x0, 0, 1, t0, cv);
    auto t_c1 = std::chrono::steady_clock::now();
    const int cold_prop = lander::ballistic_propagation_count();
    const double cold_ms =
        std::chrono::duration<double, std::milli>(t_c1 - t_c0).count();

    auto t_w0 = std::chrono::steady_clock::now();
    TransferSolution warm{};
    lander::ballistic_reset_propagation_count();
    warm = solve_transfer_warm(bin, kDt, s1.p, 0, 1, t_shift, cold);
    auto t_w1 = std::chrono::steady_clock::now();
    const int warm_prop = lander::ballistic_propagation_count();
    const double warm_ms =
        std::chrono::duration<double, std::milli>(t_w1 - t_w0).count();

    std::printf(
        "  [R5-V09] COLD  : %8.3f ms  propagations=%-6d newton_iters=-  "
        "miss=%6.2f  clearance=%s\n",
        cold_ms, cold_prop, cold.achieved_miss, cold_ok ? "CLEAR" : "FAIL");
    std::printf(
        "  [R5-V09] WARM  : %8.3f ms  propagations=%-6d newton_iters=%-2d "
        "miss=%6.2f  clearance=%s\n",
        warm_ms, warm_prop, warm.newton_iterations, warm.achieved_miss,
        warm.valid ? "CLEAR" : "FAIL");

    check(cold_ok && warm.valid, "benchmark: both solves succeed");
    check(warm_prop > 0 && warm_prop < cold_prop,
          "benchmark: warm uses fewer propagations than cold");
    check(warm.newton_iterations >= 0 && warm.newton_iterations <= 8,
          "benchmark: warm Newton iterations within the bounded maximum");
    // The wall-time relationship is recorded, not hard-asserted (timing is
    // noisy); the propagation-count ratio is the deterministic cost guarantee.
    check(warm_ms > 0.0, "benchmark: warm solve has a positive measured time");
}

// ---- M06-R5-V05 / V08: the D05 two-level midcourse controller, driven
// headlessly. A `TransferMidcourse` is armed to hold the arc to a known
// solvable transfer (zero-delta-v node at "now": the craft is already on the
// arc, so the distinguishing behavior under test is the bounded-rate WARM
// re-aim plus the O(1) HOT VGO corrections). The controller's ordinary `Input`
// is fed to a real `Simulation` step each tick, so this exercises the actual
// physics integration, not a solver in isolation.
struct MidcourseRun {
    int ticks{0};
    int slow_plans{0};
    int retargets{0};
    bool crashed{false};
    bool landed{false};
    double start_target_dist{0.0};
    double min_target_dist{1.0e30};
};

MidcourseRun run_midcourse(const BinarySystem& seed_bin, const Config& cfg,
                           const State& start, const TransferSolution& seed,
                           int ticks, double replan_interval,
                           double miss_tolerance) {
    MidcourseRun r{};
    r.ticks = ticks;

    Simulation sim;
    sim.reset(503ULL);
    sim.set_state(start);
    const BinarySystem& bin = sim.binary();

    r.start_target_dist =
        std::hypot(start.x - bin.position(seed.target, 0.0).x,
                   start.y - bin.position(seed.target, 0.0).y);
    r.min_target_dist = r.start_target_dist;

    TransferMidcourse mc;
    const double now0 = sim.sim_time();
    const ManeuverNode arm_node{};  // zero-delta-v node at "now" (on the arc)
    const NodeBasis arm_basis = compute_node_basis(
        bin, now0, seed.source, {start.x, start.y}, {start.vx, start.vy});
    mc.arm(arm_node, seed, seed.source >= 0 ? seed.source : 0, arm_basis,
           now0, cfg);

    for (int i = 0; i < ticks; ++i) {
        const State before = sim.state();
        const double now = sim.sim_time();
        // WARM (bounded rate): the only path that may reach a solver.
        mc.maybe_replan(bin, cfg, before, now, replan_interval, miss_tolerance);
        // HOT (O(1), every fixed step): the fast VGO; never triggers a solver.
        const Input input =
            mc.make_input(before, now, cfg, /*manual_left=*/false,
                          /*manual_right=*/false);
        sim.step_once(input);
        mc.after_step(before, sim.state(), input, sim.sim_time(), cfg);

        const State& after = sim.state();
        const Vec2 tpos = bin.position(seed.target, sim.sim_time());
        const double d = std::hypot(after.x - tpos.x, after.y - tpos.y);
        if (d < r.min_target_dist) {
            r.min_target_dist = d;
        }
        if (!mc.active()) {
            break;  // landed or crashed: the transfer is over.
        }
    }

    r.crashed = sim.state().crashed;
    r.landed = sim.state().landed;
    r.slow_plans = mc.slow_plans();
    r.retargets = mc.retargets();
    (void)seed_bin;
    return r;
}

// ---- M06-R5-V05: the slow (WARM) planner runs at a bounded rate, never on
// every 1/120 s fixed step. Over a 1.0 s window at a 10 Hz cadence the number
// of slow solves is far below the 120 fixed steps, and at least one runs.
void test_two_level_bounded_rate() {
    Config cfg{};
    const auto seed_bin =
        BinarySystem::canonical(cfg.mu, 503ULL, companion_seed(503ULL));
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    if (!find_cold(seed_bin, 0, 1, t0, x0, cold)) {
        std::printf("  [R5-V05] no cold 0->1 transfer; skipping\n");
        return;
    }
    // The craft is on the solved arc (departure velocity already applied).
    State start{};
    start.x = x0.x;
    start.y = x0.y;
    start.vx = cold.departure_velocity.x;
    start.vy = cold.departure_velocity.y;
    start.fuel = 1000.0;

    const int ticks = 120;  // 1.0 s of fixed steps
    const double replan_interval = 0.1;  // 10 Hz WARM cadence
    const MidcourseRun r =
        run_midcourse(seed_bin, cfg, start, cold, ticks, replan_interval, 0.25);

    std::printf("  [R5-V05] ticks=%d  slow_plans=%d  retargets=%d\n", r.ticks,
                r.slow_plans, r.retargets);
    check(r.slow_plans > 0, "bounded rate: the slow planner ran at least once");
    check(r.slow_plans < r.ticks,
          "bounded rate: the slow planner ran fewer times than the fixed steps");
    // A 10 Hz cadence over 1.0 s allows at most ~10 replans (plus a small
    // boundary margin); keep a soft cap well below the 120 steps.
    const int expected_max =
        (int)std::llround(ticks * cfg.fixed_dt / replan_interval) + 2;
    check(r.slow_plans <= expected_max,
          "bounded rate: the slow planner respects the cadence bound");
}

// ---- M06-R5-V08: closed-loop consistency. Feeding the controller's ordinary
// inputs to the real physics step drives a perturbed (imperfect-departure)
// craft back toward the target body's arrival region without crashing, using
// only bounded-rate re-aims and O(1) corrections.
void test_two_level_closed_loop() {
    Config cfg{};
    const auto seed_bin =
        BinarySystem::canonical(cfg.mu, 503ULL, companion_seed(503ULL));
    const double dt = cfg.fixed_dt;
    const double t0 = 0.0;
    Vec2 x0{};
    TransferSolution cold{};
    if (!find_cold(seed_bin, 0, 1, t0, x0, cold)) {
        std::printf("  [R5-V08] no cold 0->1 transfer; skipping\n");
        return;
    }
    // Imperfect departure: the craft departs 1% faster than the solved arc.
    // It is slightly off the arc and must be corrected back onto a trajectory
    // that reaches the target region, via a series of small bounded corrections.
    State start{};
    start.x = x0.x;
    start.y = x0.y;
    start.vx = cold.departure_velocity.x * 1.01;
    start.vy = cold.departure_velocity.y * 1.01;
    start.fuel = 1000.0;

    // Run through the transfer's time of flight (the min target distance is
    // reached near arrival), capped so the test stays fast.
    int ticks = (int)std::llround(cold.time_of_flight / dt);
    if (ticks < 240) {
        ticks = 240;
    }
    if (ticks > 4000) {
        ticks = 4000;
    }
    const double replan_interval = 0.2;  // 5 Hz WARM cadence
    const MidcourseRun r =
        run_midcourse(seed_bin, cfg, start, cold, ticks, replan_interval, 0.25);

    std::printf("  [R5-V08] ticks=%d  slow_plans=%d  retargets=%d  "
                "start_dist=%.1f  min_dist=%.1f  ratio=%.2f  crashed=%d  "
                "landed=%d\n",
                r.ticks, r.slow_plans, r.retargets, r.start_target_dist,
                r.min_target_dist,
                r.start_target_dist > 0.0 ? r.min_target_dist /
                                                r.start_target_dist
                                          : 0.0,
                r.crashed ? 1 : 0, r.landed ? 1 : 0);
    check(r.slow_plans > 0, "closed loop: the slow planner ran");
    check(r.slow_plans < r.ticks,
          "closed loop: the slow planner ran fewer times than the fixed steps");
    check(r.retargets > 0,
          "closed loop: a miss beyond tolerance re-targeted the fast VGO");
    check(!r.crashed, "closed loop: the craft did not crash");
    check(r.start_target_dist > 0.0, "closed loop: initial separation is set");
    // Regression ceiling on the closed-loop approach ratio.
    //
    // The original 0.60 proxy predates M06-R19 and assumed the node executor
    // could burn continuously through the small-VGO endgame. M06-R19's
    // continuous alignment safety (higher authority than this proxy) is
    // magnitude-gated: when the residual VGO is within a few physical steps of
    // zero and the craft is materially misaligned, the engine is cut and the
    // executor re-enters ALIGN until re-aligned. That bounded zero-throttle
    // re-entry interval causally costs a small, irreducible amount of closing
    // progress, and is NOT a permission to degrade the transfer generally.
    //
    // Measured under the R19 gate, with the pre-R19 ballistic (HEAD): the
    // approach ratio is 0.631 (warm-start seed variant: 0.626). The 0.65
    // ceiling is the smallest bound containing those measured safe cases,
    // leaving only ~0.019 margin over the worst case. It encodes exactly the
    // bounded safety cost and nothing more; a regression back toward 1.0 (or a
    // failure to re-target / crash) still fails this check.
    //
    // This bound is only valid in the presence of the R19 continuous
    // alignment-safety gate, which the node-executor regression test
    // (test_flight_computer.cpp, M06-R19-03 / whole-run invariant
    // `main_throttle > 0 => aligned`) keeps mandatory. Reintroducing
    // off-axis thrust to buy back transfer margin is a defect, not an
    // improvement, and that guard must keep failing on it.
    check(r.min_target_dist < 0.65 * r.start_target_dist,
          "closed loop: the craft converged toward the target body");
}

}  // namespace

int main() {
    test_warm_matches_cold();
    test_warm_fewer_propagations();
    test_bounded_failure();
    test_free_flight_linear_correction();
    test_no_mutation();
    test_bidirectional();
    test_multi_phase();
    test_plan_transfer_warm_path();
    test_benchmark();
    test_two_level_bounded_rate();
    test_two_level_closed_loop();

    if (failures == 0) {
        std::printf("All lander_transfer_warm_tests passed\n");
        return 0;
    }
    std::printf("%d transfer-warm test(s) failed\n", failures);
    return 1;
}
