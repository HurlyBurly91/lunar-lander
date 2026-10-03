// M06-R2-11 / M06-R2-D06: predictor and planner microbenchmark.
//
// Measures the two expensive per-frame / on-demand flight-computer calls so
// their cost is visible and tracked rather than a mystery:
//   * predict_trajectory: the on-screen zero-thrust arc. Reported for the
//     cheap near-surface case (early terrain impact, short integration) and the
//     expensive sustained-orbit case (full ~2-binary-period horizon, no early
//     impact).
//   * plan_transfer: the on-demand transfer planner (a root-finding solve).
//
// This is a measurement tool, not a pass/fail test; it prints a summary line
// that can be pasted into the task ledger. It is deterministic (fixed seed) and
// uses only the public lander:: API.

#include "lander/flight_computer.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

using namespace lander;
using Clock = std::chrono::steady_clock;

static double ms_of(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        b - a)
        .count();
}

static void print_stats(const char* label, const std::vector<double>& ms) {
    if (ms.empty()) {
        std::printf("  %s: (no samples)\n", label);
        return;
    }
    std::vector<double> s = ms;
    std::sort(s.begin(), s.end());
    const double mn = s.front();
    const double mx = s.back();
    const double p50 = s[s.size() / 2];
    const double p95 = s[static_cast<size_t>(s.size() * 0.95)];
    double sum = 0.0;
    for (double v : s) sum += v;
    std::printf(
        "  %s: n=%zu min=%.3fms p50=%.3fms avg=%.3fms p95=%.3fms max=%.3fms\n",
        label, s.size(), mn, p50, sum / s.size(), p95, mx);
}

// Place the ship on a high, stable circular orbit around the reference body so
// the zero-thrust prediction arc clears terrain for the full horizon (the
// expensive case). Returns true if the ship is airborne.
static bool to_high_orbit(Simulation& sim) {
    const auto& bin = sim.binary();
    const Vec2 bp = bin.position(0, 0.0);
    const Vec2 bv = bin.velocity(0, 0.0);
    const double rlen = std::hypot(bp.x, bp.y);
    const double rl = (rlen < 1e-9) ? 1.0 : rlen;
    const Vec2 ru{bp.x / rl, bp.y / rl};
    State s = sim.state();
    s.x = bp.x + 500.0 * ru.x;
    s.y = bp.y + 500.0 * ru.y;
    s.vx = bv.x;
    s.vy = bv.y;
    s.landed = false;
    s.crashed = false;
    s.landed_body = 0;
    sim.set_state(s);
    sim.circularize();
    const double fixed_dt = sim.config().fixed_dt;
    for (int f = 0; f < 30 && !sim.state().crashed; ++f) {
        sim.set_accumulator(std::min(sim.accumulator() + (1.0 / 60.0), 10.0));
        while (sim.accumulator() >= fixed_dt) {
            sim.set_accumulator(sim.accumulator() - fixed_dt);
            Input in{};
            (void)sim.step_once(in);
            if (sim.state().crashed) break;
        }
    }
    return !sim.state().crashed && !sim.state().landed;
}

// Launch from the pad with a fixed throttle using the M05-parity drain loop.
static bool to_launch(Simulation& sim, double throttle) {
    const double fixed_dt = sim.config().fixed_dt;
    for (int f = 0; f < 240 && sim.state().landed && !sim.state().crashed;
         ++f) {
        sim.set_accumulator(std::min(sim.accumulator() + (1.0 / 60.0), 10.0));
        while (sim.accumulator() >= fixed_dt) {
            sim.set_accumulator(sim.accumulator() - fixed_dt);
            Input in{};
            in.main_throttle = throttle;
            (void)sim.step_once(in);
            if (sim.state().crashed) break;
        }
    }
    return !sim.state().crashed && !sim.state().landed;
}

int main(int argc, char** argv) {
    std::uint64_t seed = 1;
    int reps = 200;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--seed" && i + 1 < argc) seed = std::strtoull(argv[++i], 0, 10);
        else if (a == "--reps" && i + 1 < argc) reps = std::atoi(argv[++i]);
    }

    std::printf(
        "flight-computer microbenchmark  seed=%llu  reps=%d\n",
        (unsigned long long)seed, reps);

    // --- predict_trajectory: near-surface (cheap, early impact) ---
    {
        Simulation sim;
        sim.reset(seed);
        if (!to_launch(sim, 0.46)) {
            std::printf("  predict(near): could not launch\n");
        } else {
            const double horizon = 2.0 * sim.binary().period();
            std::vector<double> ms;
            for (int i = 0; i < reps; ++i) {
                std::optional<ManeuverNode> no_node;
                const auto t0 = Clock::now();
                (void)predict_trajectory(sim.binary(), sim.config(), sim.state(),
                                         sim.sim_time(), sim.reference_body(),
                                         sim.contract().destination_body,
                                         no_node, horizon, 512);
                ms.push_back(ms_of(t0, Clock::now()));
            }
            print_stats("predict (near-surface, early impact)", ms);
        }
    }

    // --- predict_trajectory: high orbit (expensive, full horizon) ---
    {
        Simulation sim;
        sim.reset(seed);
        if (!to_high_orbit(sim)) {
            std::printf("  predict(orbit): could not reach orbit\n");
        } else {
            const double horizon = 2.0 * sim.binary().period();
            const int steps = static_cast<int>(std::llround(horizon / sim.config().fixed_dt));
            std::vector<double> ms;
            for (int i = 0; i < reps; ++i) {
                std::optional<ManeuverNode> no_node;
                const auto t0 = Clock::now();
                (void)predict_trajectory(sim.binary(), sim.config(), sim.state(),
                                         sim.sim_time(), sim.reference_body(),
                                         sim.contract().destination_body,
                                         no_node, horizon, 512);
                ms.push_back(ms_of(t0, Clock::now()));
            }
            std::printf("  (orbit horizon=%.2fs ~ %d fixed steps, 512 samples)\n",
                        horizon, steps);
            print_stats("predict (sustained orbit, full horizon)", ms);
        }
    }

    // --- plan_transfer: on-demand transfer planner solve ---
    {
        Simulation sim;
        sim.reset(seed);
        bool ok = to_high_orbit(sim);
        if (!ok) ok = to_launch(sim, 0.6);
        if (!ok) {
            std::printf("  plan_transfer: no in-flight state\n");
        } else {
            std::vector<double> ms;
            int solved = 0;
            for (int i = 0; i < reps; ++i) {
                std::optional<ManeuverNode> existing = default_node(
                    sim.sim_time(), sim.reference_body(), sim.config().fixed_dt);
                const auto t0 = Clock::now();
                auto planned = plan_transfer(sim.binary(), sim.config(),
                                             sim.state(), sim.sim_time(),
                                             sim.reference_body(), existing);
                const double el = ms_of(t0, Clock::now());
                ms.push_back(el);
                if (planned.has_value()) ++solved;
            }
            std::printf("  (solutions found: %d/%d)\n", solved, reps);
            print_stats("plan_transfer (on-demand planner solve)", ms);
        }
    }
    return 0;
}
