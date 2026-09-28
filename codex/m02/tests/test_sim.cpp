#include "lander/sim.hpp"

#include <cmath>
#include <cstdint>
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

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool state_close(const lander::State& a, const lander::State& b,
                 double eps = 1e-9) {
    return close(a.x, b.x, eps) && close(a.y, b.y, eps) &&
           close(a.vx, b.vx, eps) && close(a.vy, b.vy, eps) &&
           close(a.angle, b.angle, eps) && close(a.omega, b.omega, eps) &&
           close(a.fuel, b.fuel, eps) &&
           a.landed == b.landed && a.crashed == b.crashed &&
           a.score == b.score && a.ticks == b.ticks;
}

}

int main() {
    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(12345);
        b.reset(12345);

        check(a.pads() == b.pads(),
              "same seed should produce same pads");
    }

    {
        lander::Simulation sim;
        sim.reset(1);

        const double initial_fuel = sim.state().fuel;

        for (int i = 0; i < 120; ++i) {
            sim.advance(1.0 / 120.0, {.main_thrust = true});
        }

        check(sim.state().fuel < initial_fuel,
              "thrust should consume fuel");

        check(sim.state().fuel >= 0.0,
              "fuel must not become negative");
    }

    {
        lander::Simulation sim;
        sim.reset(9);

        auto s = sim.state();
        s.omega = 4.0;
        sim.set_state(s);

        sim.reset(9);

        check(close(sim.state().omega, 0.0),
              "reset should clear angular velocity");
    }

    // M02: rotational inertia.

    {
        lander::Simulation sim;
        sim.reset(3);

        sim.advance(1.0 / 120.0, {.rotate_left = true});

        check(close(sim.state().omega,
                    (1.6 / 1.0) * (1.0 / 120.0),
                    1e-12),
              "one rotation step should apply torque / inertia * dt");
    }

    {
        lander::Config config;
        config.moment_of_inertia = 2.0;

        lander::Simulation light(lander::Config{});
        lander::Simulation heavy(config);

        light.reset(11);
        heavy.reset(11);

        for (int i = 0; i < 240; ++i) {
            const lander::Input spin{.rotate_left = true};
            light.advance(1.0 / 120.0, spin);
            heavy.advance(1.0 / 120.0, spin);
        }

        check(close(heavy.state().omega, light.state().omega / 2.0, 1e-9),
              "double moment of inertia should halve angular acceleration");
    }

    // M02: multiple landing pads.

    {
        const std::uint64_t seeds[] = {7, 42, 999, 424242, 123456789};

        for (const std::uint64_t seed : seeds) {
            lander::Simulation sim;
            sim.reset(seed);

            const auto& pads = sim.pads();

            check(pads.size() == 3,
                  "simulation should generate exactly three pads");
            check(pads[0].multiplier == 1 && pads[1].multiplier == 2 &&
                      pads[2].multiplier == 3,
                  "pad multipliers should be 1, 2, and 3");

            for (std::size_t i = 0; i < pads.size(); ++i) {
                check(pads[i].x_min < pads[i].x_max,
                      "pad should have positive width");
                for (std::size_t j = i + 1; j < pads.size(); ++j) {
                    const bool disjoint =
                        pads[i].x_max <= pads[j].x_min ||
                        pads[j].x_max <= pads[i].x_min;
                    check(disjoint, "pads must not overlap");
                }
            }
        }
    }

    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(2024);
        b.reset(2024);

        check(a.pads() == b.pads(),
              "same seed should produce the same three pads");

        b.reset(2025);

        check(a.pads() != b.pads(),
              "different seeds should produce different pads");
    }

    {
        // Landing on each pad scores 100 * its multiplier.
        for (int i = 0; i < 3; ++i) {
            lander::Simulation sim;
            sim.reset(31337);

            const auto& pad = sim.pads()[i];

            lander::State drop{};
            drop.x = 0.5 * (pad.x_min + pad.x_max);
            drop.y = 0.5;
            sim.set_state(drop);

            for (int k = 0; k < 240; ++k) {
                sim.advance(1.0 / 120.0, {});
                if (sim.state().landed || sim.state().crashed) {
                    break;
                }
            }

            check(sim.state().landed, "drop onto a pad should land safely");
            check(sim.state().score == 100 * pad.multiplier,
                  "landing score should follow pad multiplier");
        }
    }

    {
        lander::Simulation sim;
        sim.reset(31337);

        lander::State drop{};
        drop.x = sim.pads().front().x_min - 1.0;
        drop.y = 0.5;
        sim.set_state(drop);

        for (int k = 0; k < 240; ++k) {
            sim.advance(1.0 / 120.0, {});
            if (sim.state().landed || sim.state().crashed) {
                break;
            }
        }

        check(sim.state().crashed,
              "ground contact outside a pad should crash");
        check(sim.state().score == 0, "crash should not score");
    }

    // M02: input recording and replay.

    {
        lander::Simulation sim;
        sim.reset(77);

        sim.start_recording();
        check(sim.stop_recording().empty(),
              "recording should start empty");

        sim.advance(1.0 / 120.0, {});
        check(sim.stop_recording().empty(),
              "advance without recording should not be recorded");
    }

    {
        lander::Simulation sim;
        sim.reset(5);

        sim.start_recording();
        const lander::Input thrust{.main_thrust = true, .rotate_right = true};
        sim.advance(1.0 / 120.0, thrust);
        sim.advance(0.0, {});
        sim.advance(0.02, {.rotate_left = true});

        const auto frames = sim.stop_recording();

        check(frames.size() == 3, "every advance should be recorded");
        check(close(frames[0].real_dt, 1.0 / 120.0) &&
                  frames[0].input == thrust,
              "recorded frame should capture dt and input");
        check(close(frames[1].real_dt, 0.0) &&
                  frames[1].input == lander::Input{},
              "zero dt advance should still be recorded");
        check(close(frames[2].real_dt, 0.02) &&
                  frames[2].input == lander::Input{.rotate_left = true},
              "recorded frame should capture dt and input");
    }

    {
        lander::Simulation sim;
        sim.reset(13);

        sim.start_recording();
        sim.advance(1.0 / 120.0, {});
        const auto first = sim.stop_recording();
        check(first.size() == 1, "stop_recording should return the sequence");

        sim.advance(1.0 / 120.0, {});

        sim.start_recording();
        sim.advance(1.0 / 120.0, {.main_thrust = true});
        const auto second = sim.stop_recording();

        check(second.size() == 1, "a new recording should start empty");
        check(second[0].input.main_thrust,
              "a new recording should capture only later advances");
    }

    {
        // Reset + replay of a recorded session reproduces its final state.
        lander::Simulation sim;
        sim.reset(987654321);
        sim.start_recording();

        for (int i = 0; i < 600; ++i) {
            lander::Input frame{};
            frame.main_thrust = (i % 150) < 50;
            frame.rotate_left = (i % 75) < 12;
            frame.rotate_right = (i % 75) >= 63;
            sim.advance(1.0 / 120.0, frame);
        }

        const lander::State original = sim.state();
        const auto frames = sim.stop_recording();
        check(frames.size() == 600,
              "all session advances should be recorded");

        sim.reset(987654321);
        sim.replay(frames);

        check(state_close(sim.state(), original),
              "reset + replay should reproduce the final state");
    }

    {
        // Replay applies frames to the current state without resetting it.
        lander::Simulation sim;
        sim.reset(7);

        lander::State shifted = sim.state();
        shifted.x = 5.0;
        sim.set_state(shifted);

        sim.replay({{1.0 / 120.0, lander::Input{}}});

        check(sim.state().ticks == 1, "replay should advance the simulation");
        check(close(sim.state().x, 5.0),
              "replay should not reset the current state");
    }

    {
        // Replay must not append to an active recording.
        lander::Simulation sim;
        sim.reset(4242);

        sim.start_recording();
        sim.advance(1.0 / 120.0, {.rotate_left = true});
        sim.advance(0.0, {.main_thrust = true});
        const auto frames = sim.stop_recording();
        check(frames.size() == 2,
              "recording should capture both advances");

        lander::Simulation replaying_sim;
        replaying_sim.reset(4242);
        replaying_sim.start_recording();
        replaying_sim.replay(frames);
        const auto captured = replaying_sim.stop_recording();
        check(captured.empty(),
              "replay must not append to an active recording");

        lander::Simulation plain_sim;
        plain_sim.reset(4242);
        plain_sim.replay(frames);

        check(state_close(replaying_sim.state(), plain_sim.state()),
              "recording during replay should not change the state");
    }

    return failures == 0 ? 0 : 1;
}
