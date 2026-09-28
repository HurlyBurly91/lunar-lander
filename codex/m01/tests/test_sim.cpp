#include "lander/sim.hpp"

#include <cmath>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool close(double a, double b, double eps = 1e-8) {
    return std::abs(a - b) <= eps;
}

// Runs the simulation with no input until it lands or crashes.
// Returns true if a terminal state was reached within the step budget.
bool run_to_terminal(lander::Simulation& sim, int max_steps = 24 * 120) {
    for (int i = 0; i < max_steps; ++i) {
        sim.advance(1.0 / 120.0, {});
        if (sim.state().landed || sim.state().crashed) {
            return true;
        }
    }
    return false;
}

// State that hovers just above the middle of the pad with a slow, safe
// descent profile; `angle` may be overridden per test case.
lander::State safe_state_above(const lander::Pad& pad, double angle) {
    lander::State s{};
    s.x = (pad.x_min + pad.x_max) / 2.0;
    s.y = 0.3;
    s.vx = 0.2;
    s.vy = -1.0;
    s.angle = angle;
    s.fuel = 50.0;
    return s;
}

}

int main() {
    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(1234);
        b.reset(1234);

        check(!a.pads().empty(),
              "reset should generate at least one landing pad");

        check(a.pads() == b.pads(),
              "same seed should generate identical pads");
    }

    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(5);
        b.reset(5);

        for (int i = 0; i < 60; ++i) {
            a.advance(1.0 / 60.0, {});
        }

        for (int i = 0; i < 30; ++i) {
            b.advance(1.0 / 30.0, {});
        }

        check(close(a.state().y, b.state().y, 1e-7),
              "physics must be independent of advance frequency");

        check(close(a.state().vy, b.state().vy, 1e-7),
              "velocity must be independent of advance frequency");
    }

    {
        lander::Simulation sim;
        sim.reset(99);

        for (int i = 0; i < 5000; ++i) {
            sim.advance(1.0 / 120.0, {.main_thrust = true});
        }

        check(sim.state().fuel >= 0.0,
              "fuel may never become negative");
    }

    {
        // Reset completely restores dynamic state and regenerates pads.
        lander::Simulation sim;
        sim.reset(7);
        const lander::State fresh = sim.state();
        const auto fresh_pads = sim.pads();

        sim.advance(0.5, {.main_thrust = true, .rotate_left = true});
        lander::State injected = sim.state();
        injected.fuel = 12.5;
        injected.score = 42;
        sim.set_state(injected);

        sim.reset(7);

        const lander::State& restored = sim.state();
        check(restored == fresh, "reset must restore the full dynamic state");
        check(sim.pads() == fresh_pads,
              "reset with the same seed must regenerate identical pads");

        lander::Simulation other;
        other.reset(8);
        check(sim.pads() != other.pads(),
              "different seeds should generate different pads");
    }

    {
        // Safe pad contact lands and scores by pad multiplier.
        lander::Simulation sim;
        sim.reset(2024);
        const lander::Pad& pad = sim.pads().front();
        sim.set_state(safe_state_above(pad, 0.05));

        check(run_to_terminal(sim), "safe descent must reach the ground");
        check(sim.state().landed, "safe pad contact must land");
        check(!sim.state().crashed, "safe pad contact must not crash");
        check(sim.state().score == 100 * pad.multiplier,
              "landing score must follow the pad multiplier");
        check(sim.state().y == 0.0, "landed lander must rest on the surface");
        check(sim.state().vx == 0.0 && sim.state().vy == 0.0 &&
                  sim.state().omega == 0.0,
              "landed lander must be at rest");
    }

    {
        // Excessive impact velocity crashes.
        lander::Simulation sim;
        sim.reset(2024);
        const lander::Pad& pad = sim.pads().front();
        lander::State s = safe_state_above(pad, 0.0);
        s.vy = -10.0;
        sim.set_state(s);

        check(run_to_terminal(sim), "fast descent must reach the ground");
        check(sim.state().crashed, "excessive impact velocity must crash");
        check(!sim.state().landed, "crashed lander must not count as landed");
        check(sim.state().score == 0, "a crash must not score");
    }

    {
        // Ground contact outside every pad crashes, even with a gentle
        // descent profile.
        lander::Simulation sim;
        sim.reset(2024);
        const lander::Pad& pad = sim.pads().front();
        lander::State s = safe_state_above(pad, 0.0);
        s.x = pad.x_max + 50.0;
        sim.set_state(s);

        check(run_to_terminal(sim), "descent must reach the ground");
        check(sim.state().crashed,
              "ground contact outside a pad must crash");
        check(!sim.state().landed, "landing requires pad contact");
    }

    {
        // Angles equivalent modulo 2*pi are treated equivalently for
        // landing.
        lander::Simulation sim;
        sim.reset(2024);
        const lander::Pad& pad = sim.pads().front();

        for (double angle : {2.0 * kPi + 0.05, -2.0 * kPi - 0.05,
                             4.0 * kPi, 2.0 * kPi}) {
            sim.reset(2024);
            sim.set_state(safe_state_above(pad, angle));
            check(run_to_terminal(sim), "descent must reach the ground");
            check(sim.state().landed,
                  "equivalent angle (modulo 2*pi) must land like the base");
        }

        // An angle near pi is far from upright and must crash.
        sim.reset(2024);
        sim.set_state(safe_state_above(pad, kPi + 0.05));
        check(run_to_terminal(sim), "descent must reach the ground");
        check(sim.state().crashed,
              "an angle near pi must not be treated as upright");
    }

    {
        // Once terminal, further advances must not change the state.
        lander::Simulation sim;
        sim.reset(2024);
        lander::State s = safe_state_above(sim.pads().front(), 0.0);
        s.vy = -10.0;
        sim.set_state(s);
        check(run_to_terminal(sim), "fast descent must reach the ground");
        check(sim.state().crashed, "fast descent must crash");

        const lander::State before = sim.state();
        const auto before_pads = sim.pads();
        for (int i = 0; i < 120; ++i) {
            sim.advance(1.0 / 120.0,
                        {.main_thrust = true, .rotate_left = true,
                         .rotate_right = true});
        }
        check(sim.state() == before,
              "terminal state must not change on further advances");
        check(sim.pads() == before_pads, "pads must not change after crash");
    }

    return failures == 0 ? 0 : 1;
}
