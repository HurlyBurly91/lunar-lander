#include "lander/sim.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool near_eq(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool same_state(const lander::State& a, const lander::State& b) {
    return near_eq(a.x, b.x) && near_eq(a.y, b.y) &&
           near_eq(a.vx, b.vx) && near_eq(a.vy, b.vy) &&
           near_eq(a.angle, b.angle) && near_eq(a.omega, b.omega) &&
           near_eq(a.fuel, b.fuel) && a.landed == b.landed &&
           a.crashed == b.crashed && a.score == b.score &&
           a.ticks == b.ticks;
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
        // Same seed must produce identical landing pads, every time, in
        // independent simulation instances (no hidden randomness source).
        for (std::uint64_t seed = 1; seed <= 8; ++seed) {
            lander::Simulation a;
            lander::Simulation b;
            a.reset(seed);
            b.reset(seed);
            check(a.pads() == b.pads(),
                  "same seed must produce identical landing pads");
        }
    }

    {
        // Frame-rate independence: the same elapsed simulated time must
        // produce the same state regardless of how advance() is chunked.
        const std::vector<double> chunk_120(240, 1.0 / 120.0);
        const std::vector<double> chunk_60(120, 1.0 / 60.0);
        const std::vector<double> chunk_30(60, 1.0 / 30.0);

        std::vector<double> mixed;
        mixed.insert(mixed.end(), 120, 1.0 / 120.0);
        mixed.insert(mixed.end(), 60, 1.0 / 60.0);

        const auto run = [](const std::vector<double>& chunks) {
            lander::Simulation sim;
            sim.reset(7);
            for (const double dt : chunks) {
                sim.advance(dt, {});
            }
            return sim.state();
        };

        const lander::State reference = run(chunk_120);
        check(same_state(reference, run(chunk_60)),
              "1/60 chunking must match 1/120 chunking");
        check(same_state(reference, run(chunk_30)),
              "1/30 chunking must match 1/120 chunking");
        check(same_state(reference, run(mixed)),
              "mixed chunking must match 1/120 chunking");
        check(reference.ticks == 240,
              "two simulated seconds must be 240 fixed steps");
    }

    {
        // A step that crosses the surface at low altitude must settle onto
        // the surface and resolve contact within the same step.
        lander::Simulation sim;
        sim.reset(99);
        const lander::Pad pad = sim.pads().front();

        lander::State s = sim.state();
        s.x = 0.5 * (pad.x_min + pad.x_max);
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -0.5;
        s.angle = 0.0;
        s.omega = 0.0;
        s.fuel = 50.0;
        s.ticks = 0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().landed,
              "safe low-altitude pad contact must land");
        check(sim.state().crashed == false,
              "safe low-altitude pad contact must not crash");
        check(sim.state().y == 0.0,
              "contact must settle exactly at the surface");
        check(sim.state().score == 100,
              "landing score must follow the pad multiplier");
    }

    {
        // A crossing contact outside a pad is a crash within the same step.
        lander::Simulation sim;
        sim.reset(99);
        const lander::Pad pad = sim.pads().front();

        lander::State s = sim.state();
        s.x = pad.x_max + 5.0;
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -0.5;
        s.angle = 0.0;
        s.omega = 0.0;
        s.fuel = 50.0;
        s.ticks = 0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().crashed,
              "ground contact outside a pad must crash");
        check(sim.state().y == 0.0,
              "crash must settle exactly at the surface");
    }

    {
        // An unsafe pad contact (excessive impact speed) crashes.
        lander::Simulation sim;
        sim.reset(99);
        const lander::Pad pad = sim.pads().front();

        lander::State s = sim.state();
        s.x = 0.5 * (pad.x_min + pad.x_max);
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -5.0;
        s.angle = 0.0;
        s.omega = 0.0;
        s.fuel = 50.0;
        s.ticks = 0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().crashed,
              "excessive impact speed must crash");
        check(sim.state().landed == false,
              "excessive impact speed must not land");
    }

    {
        // Angles equivalent modulo 2*pi must be treated equivalently when
        // classifying contact, including across the +/-pi / 2*pi
        // representation boundary.
        lander::Simulation sim;
        sim.reset(99);
        const lander::Pad pad = sim.pads().front();

        const auto try_angle = [&](double angle, bool expect_landed) {
            sim.reset(99);
            lander::State s = sim.state();
            s.x = 0.5 * (pad.x_min + pad.x_max);
            s.y = 0.0;
            s.vx = 0.0;
            s.vy = 0.0;
            s.angle = angle;
            s.omega = 0.0;
            s.fuel = 50.0;
            s.ticks = 0;
            sim.set_state(s);
            sim.advance(1.0 / 120.0, {});
            check(sim.state().landed == expect_landed,
                  "angle classification must be invariant modulo 2*pi");
        };

        constexpr double two_pi = 6.283185307179586;

        try_angle(-0.05, true);
        try_angle(0.05, true);
        try_angle(two_pi - 0.05, true);
        try_angle(two_pi + 0.05, true);
        try_angle(-two_pi - 0.05, true);
        try_angle(-two_pi + 0.05, true);
        try_angle(2.0 * two_pi - 0.05, true);

        try_angle(0.5, false);
        try_angle(two_pi + 0.5, false);
        try_angle(-two_pi - 0.5, false);
        try_angle(3.141592653589793, false);
    }

    {
        // Once landed or crashed, further advances must not change state.
        lander::Simulation sim;
        sim.reset(99);
        const lander::Pad pad = sim.pads().front();

        lander::State s = sim.state();
        s.x = 0.5 * (pad.x_min + pad.x_max);
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -0.5;
        s.angle = 0.0;
        s.omega = 0.0;
        s.fuel = 50.0;
        s.ticks = 0;
        sim.set_state(s);
        sim.advance(1.0 / 120.0, {});

        const lander::State frozen = sim.state();
        sim.advance(0.25, {.main_thrust = true, .rotate_left = true});
        sim.advance(0.5, {});
        check(same_state(frozen, sim.state()),
              "state must be frozen after landing");
    }

    {
        // Full-session determinism: the same seed and the same per-step
        // control sequence must reproduce the identical final state.
        const auto drive = [](lander::Simulation& sim) {
            for (int i = 0; i < 600; ++i) {
                lander::Input input;
                input.main_thrust = (i % 37) < 2;
                input.rotate_left = (i % 53) == 0;
                input.rotate_right = (i % 71) == 0;
                sim.advance(1.0 / 120.0, input);
                if (sim.state().landed || sim.state().crashed) {
                    break;
                }
            }
        };

        lander::Simulation a;
        lander::Simulation b;
        a.reset(555);
        b.reset(555);
        drive(a);
        drive(b);
        check(same_state(a.state(), b.state()),
              "same seed and same inputs must reproduce the same state");
        check(a.pads() == b.pads(),
              "same seed must reproduce the same pads in a session");
    }

    {
        // Fuel stays clamped at zero even with simultaneous burns.
        lander::Simulation sim;
        sim.reset(100);

        auto s = sim.state();
        s.fuel = 0.001;
        sim.set_state(s);

        for (int i = 0; i < 10; ++i) {
            sim.advance(1.0 / 120.0, {.main_thrust = true, .rotate_left = true});
        }

        check(sim.state().fuel >= 0.0,
              "fuel must never become negative");
        check(sim.state().fuel == 0.0,
              "exhausted fuel must settle exactly at zero");
    }

    return failures == 0 ? 0 : 1;
}
