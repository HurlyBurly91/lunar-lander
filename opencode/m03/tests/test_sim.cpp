#include "lander/sim.hpp"

#include <cmath>
#include <iostream>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool state_close(const lander::State& a, const lander::State& b, double tol) {
    return std::abs(a.x - b.x) <= tol &&
           std::abs(a.y - b.y) <= tol &&
           std::abs(a.vx - b.vx) <= tol &&
           std::abs(a.vy - b.vy) <= tol &&
           std::abs(a.angle - b.angle) <= tol &&
           std::abs(a.omega - b.omega) <= tol &&
           std::abs(a.fuel - b.fuel) <= tol &&
           a.ticks == b.ticks &&
           a.landed == b.landed &&
           a.crashed == b.crashed;
}

double pad_center(const lander::Simulation& sim) {
    const auto& pad = sim.pads().front();
    return (pad.x_min + pad.x_max) / 2.0;
}

// Places the lander just above its pad, gently descending, upright, and
// steps until it lands.
void land_gently(lander::Simulation& sim) {
    lander::State s{};
    s.x = pad_center(sim);
    s.y = 0.001;
    s.vy = -0.5;
    s.angle = 0.05;
    sim.set_state(s);

    for (int i = 0; i < 12 && !sim.state().landed && !sim.state().crashed;
         ++i) {
        sim.advance(1.0 / 120.0, {});
    }
}

}

int main() {
    {
        lander::Simulation sim;
        sim.reset(100);

        auto s = sim.state();
        s.omega = 3.5;
        sim.set_state(s);

        sim.reset(100);

        check(std::abs(sim.state().omega) < 1e-12,
              "reset must clear angular velocity");
    }

    {
        lander::Simulation sim;
        sim.reset(100);

        auto s = sim.state();
        s.fuel = 0.001;
        sim.set_state(s);

        for (int i = 0; i < 10; ++i) {
            sim.advance(1.0 / 120.0, {.main_thrust = true});
        }

        check(sim.state().fuel >= 0.0,
              "fuel must not become negative");
    }

    {
        // Regression: physics depended on the rate advance() was called.
        // The same elapsed time split differently must execute the same
        // number of fixed steps and produce the same state.
        lander::Simulation a;
        lander::Simulation b;
        a.reset(7);
        b.reset(7);

        const lander::Input thrust{.main_thrust = true};
        const double total = 0.537;

        a.advance(total, thrust);

        const double chunk = total / 64.0;
        for (int i = 0; i < 64; ++i) {
            b.advance(chunk, thrust);
        }

        check(a.state().ticks == b.state().ticks,
              "fixed-step count must not depend on advance() call pattern");
        check(state_close(a.state(), b.state(), 1e-9),
              "equivalent elapsed time must produce equivalent physics");
    }

    {
        // Regression: the seed was no longer the sole source of terrain
        // randomness, so identical seeds gave different pads.
        const std::uint64_t seeds[] = {
            1, 2, 42, 1234567, 18446744073709551615ULL};

        for (std::uint64_t seed : seeds) {
            lander::Simulation a;
            lander::Simulation b;
            a.reset(seed);
            b.reset(seed);

            check(a.pads() == b.pads(),
                  "same seed must produce identical pads");
        }
    }

    {
        // Regression: a nearly upright lander was misclassified when its
        // angle crossed the 2*pi representation boundary.
        lander::Simulation sim;

        for (double angle : {kTwoPi - 0.01, kTwoPi + 0.01}) {
            sim.reset(100);

            lander::State s{};
            s.x = pad_center(sim);
            s.y = 0.001;
            s.vy = -0.1;
            s.angle = angle;
            sim.set_state(s);

            sim.advance(1.0 / 120.0, {});
            sim.advance(1.0 / 120.0, {});

            check(sim.state().landed,
                  "near-upright lander at the 2*pi boundary must land");
        }
    }

    {
        // Landing criteria must not have been weakened by the angle fix:
        // a genuinely sideways lander still crashes.
        lander::Simulation sim;
        sim.reset(100);

        lander::State s{};
        s.x = pad_center(sim);
        s.y = 0.001;
        s.vy = -0.1;
        s.angle = kPi / 2.0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});
        sim.advance(1.0 / 120.0, {});

        check(sim.state().crashed,
              "a sideways lander must still crash");
    }

    {
        // Regression: very low-altitude contacts were resolved before
        // integration, leaving the lander below the surface for a frame.
        lander::Simulation sim;
        sim.reset(100);

        lander::State s{};
        s.x = pad_center(sim);
        s.y = 0.0001;
        s.vy = -50.0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().y >= 0.0,
              "lander must not remain below the surface after a step");
        check(sim.state().landed || sim.state().crashed,
              "ground contact must be resolved in the step that reaches it");
    }

    {
        // Once landed, further advances must not change physical state.
        lander::Simulation sim;
        sim.reset(100);

        land_gently(sim);
        check(sim.state().landed, "gentle descent must land on the pad");

        const lander::State before = sim.state();
        sim.advance(0.1, {});
        sim.advance(0.25, {.main_thrust = true});

        check(state_close(sim.state(), before, 0.0),
              "state must not change after landing");
    }

    return failures == 0 ? 0 : 1;
}
