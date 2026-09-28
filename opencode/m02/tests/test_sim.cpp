#include "lander/sim.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

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

void check_state_equal(const lander::State& a, const lander::State& b) {
    check(a.ticks == b.ticks, "replay reproduces ticks");
    check(a.landed == b.landed, "replay reproduces landed");
    check(a.crashed == b.crashed, "replay reproduces crashed");
    check(a.score == b.score, "replay reproduces score");
    check(close(a.x, b.x), "replay reproduces x");
    check(close(a.y, b.y), "replay reproduces y");
    check(close(a.vx, b.vx), "replay reproduces vx");
    check(close(a.vy, b.vy), "replay reproduces vy");
    check(close(a.angle, b.angle), "replay reproduces angle");
    check(close(a.omega, b.omega), "replay reproduces omega");
    check(close(a.fuel, b.fuel), "replay reproduces fuel");
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

    {
        // M02: angular acceleration = rotation_torque / moment_of_inertia
        lander::Simulation sim;
        sim.reset(1);

        for (int i = 0; i < 120; ++i) {
            sim.advance(1.0 / 120.0, {.rotate_left = true});
        }

        check(close(sim.state().omega, 1.6, 1e-9),
              "left rotation integrates torque/inertia angular acceleration");

        lander::Simulation right;
        right.reset(1);

        for (int i = 0; i < 120; ++i) {
            right.advance(1.0 / 120.0, {.rotate_right = true});
        }

        check(close(right.state().omega, -1.6, 1e-9),
              "right rotation is negative");

        lander::Simulation both;
        both.reset(1);

        for (int i = 0; i < 120; ++i) {
            both.advance(1.0 / 120.0, {.rotate_left = true, .rotate_right = true});
        }

        check(close(both.state().omega, 0.0, 1e-12),
              "opposing rotations cancel");
    }

    {
        // M02: doubling the moment of inertia halves angular acceleration
        lander::Config default_cfg{};
        lander::Config heavy_cfg{};
        heavy_cfg.moment_of_inertia = 2.0 * default_cfg.moment_of_inertia;

        lander::Simulation light(default_cfg);
        lander::Simulation heavy(heavy_cfg);

        light.reset(3);
        heavy.reset(3);

        const lander::Input rotate{.rotate_left = true};

        for (int i = 0; i < 240; ++i) {
            light.advance(1.0 / 120.0, rotate);
            heavy.advance(1.0 / 120.0, rotate);
        }

        check(close(light.state().omega, 2.0 * heavy.state().omega, 1e-9),
              "double inertia produces half angular acceleration");

        check(close(light.state().omega, 3.2, 1e-9),
              "2s of default rotation reaches 3.2 rad/s");

        check(close(heavy.state().omega, 1.6, 1e-9),
              "2s of doubled-inertia rotation reaches 1.6 rad/s");
    }

    {
        // M02: exactly three pads, multipliers 1/2/3, non-overlapping,
        // deterministic across a range of seeds
        for (std::uint64_t seed = 0; seed < 32; ++seed) {
            lander::Simulation sim;
            sim.reset(seed);

            const auto& pads = sim.pads();

            check(pads.size() == 3, "exactly three pads");
            if (pads.size() != 3) {
                continue;
            }

            check(pads[0].multiplier == 1 &&
                  pads[1].multiplier == 2 &&
                  pads[2].multiplier == 3,
                  "pad multipliers are 1, 2, 3 in ascending x order");

            for (int i = 0; i + 1 < 3; ++i) {
                check(pads[i].x_max < pads[i + 1].x_min,
                      "pads do not overlap");
            }

            check(pads[0].x_min >= -8.0 - 1e-12 &&
                  pads[2].x_max <= 8.0 + 1e-12,
                  "pads stay within the world bounds");

            for (const auto& pad : pads) {
                check(pad.x_max > pad.x_min, "pad has positive width");
            }
        }

        lander::Simulation a;
        lander::Simulation b;

        a.reset(1);
        b.reset(2);

        check(a.pads() != b.pads(),
              "different seeds should produce different pads");
    }

    {
        // M02: a safe touchdown on each pad scores 100 * multiplier
        for (int multiplier : {1, 2, 3}) {
            lander::Simulation sim;
            sim.reset(4242);

            const lander::Pad* target = nullptr;
            for (const auto& pad : sim.pads()) {
                if (pad.multiplier == multiplier) {
                    target = &pad;
                }
            }

            check(target != nullptr, "each multiplier has a pad");
            if (!target) {
                continue;
            }

            auto s = sim.state();
            s.x = 0.5 * (target->x_min + target->x_max);
            s.y = 0.05;
            s.vx = 0.0;
            s.vy = -1.0;
            s.angle = 0.0;
            s.omega = 0.0;
            sim.set_state(s);

            for (int i = 0; i < 60 && !sim.state().landed && !sim.state().crashed;
                 ++i) {
                sim.advance(1.0 / 120.0, {});
            }

            check(sim.state().landed, "safe touchdown on pad lands");
            check(sim.state().score == 100 * multiplier,
                  "landing score follows the pad multiplier");
        }
    }

    {
        // M02: ground contact outside every pad still crashes
        lander::Simulation sim;
        sim.reset(4242);

        const auto& pads = sim.pads();
        const double gap_x = 0.5 * (pads[0].x_max + pads[1].x_min);

        auto s = sim.state();
        s.x = gap_x;
        s.y = 0.05;
        s.vx = 0.0;
        s.vy = -1.0;
        sim.set_state(s);

        for (int i = 0; i < 60 && !sim.state().crashed; ++i) {
            sim.advance(1.0 / 120.0, {});
        }

        check(sim.state().crashed, "ground contact outside pads crashes");
        check(!sim.state().landed, "crash is not a landing");
        check(sim.state().score == 0, "crash scores nothing");
    }

    {
        // M02: unsafe contact on a pad is still a crash
        lander::Simulation sim;
        sim.reset(4242);

        const auto& pad3 = sim.pads()[2];

        auto s = sim.state();
        s.x = 0.5 * (pad3.x_min + pad3.x_max);
        s.y = 0.05;
        s.vx = 0.0;
        s.vy = -5.0;
        sim.set_state(s);

        for (int i = 0; i < 60 && !sim.state().crashed; ++i) {
            sim.advance(1.0 / 120.0, {});
        }

        check(sim.state().crashed, "unsafe pad contact crashes");
        check(!sim.state().landed, "unsafe pad contact does not land");
    }

    {
        // M02: recording begins empty and captures every advance() call
        lander::Simulation fresh;
        fresh.start_recording();
        auto nothing = fresh.stop_recording();
        check(nothing.empty(), "recording begins empty");

        lander::Simulation idle;
        idle.reset(11);
        idle.advance(1.0 / 120.0, {});
        auto not_recording = idle.stop_recording();
        check(not_recording.empty(),
              "nothing is recorded before start_recording");

        lander::Simulation sim;
        sim.reset(11);
        sim.start_recording();

        const int n = 25;
        for (int i = 0; i < n; ++i) {
            lander::Input input;
            if (i % 3 == 0) {
                input.main_thrust = true;
            }
            if (i % 3 == 1) {
                input.rotate_left = true;
            }
            const double dt = (i == 10) ? 0.0 : 1.0 / 120.0;
            sim.advance(dt, input);
        }

        auto frames = sim.stop_recording();
        check(frames.size() == static_cast<std::size_t>(n),
              "every advance call is recorded");

        bool pattern_ok = frames.size() == static_cast<std::size_t>(n);
        for (int i = 0; pattern_ok && i < n; ++i) {
            lander::Input expected;
            if (i % 3 == 0) {
                expected.main_thrust = true;
            }
            if (i % 3 == 1) {
                expected.rotate_left = true;
            }
            const double dt = (i == 10) ? 0.0 : 1.0 / 120.0;
            pattern_ok = frames[i] == lander::InputFrame{dt, expected};
        }
        check(pattern_ok, "recorded frames match the advance calls");

        sim.advance(1.0 / 120.0, {});
        auto after_stop = sim.stop_recording();
        check(after_stop.empty(),
              "recording stays disabled after stop_recording");
    }

    {
        // M02: reset + replay of a recorded session reproduces the state
        lander::Simulation sim;
        sim.reset(987);
        sim.start_recording();

        for (int i = 0; i < 300; ++i) {
            lander::Input input;
            if (i % 4 < 2) {
                input.main_thrust = true;
            }
            if (i % 7 == 0) {
                input.rotate_left = true;
            }
            if (i % 11 == 0) {
                input.rotate_right = true;
            }
            sim.advance(1.0 / 120.0, input);
        }

        auto frames = sim.stop_recording();
        const lander::State recorded_final = sim.state();

        sim.reset(987);
        sim.replay(frames);

        check_state_equal(sim.state(), recorded_final);
    }

    {
        // M02: replaying a landing session reproduces the landing and score
        lander::Simulation sim;
        sim.reset(555);

        const lander::Pad* target = nullptr;
        for (const auto& pad : sim.pads()) {
            if (pad.multiplier == 3) {
                target = &pad;
            }
        }

        check(target != nullptr, "pad with multiplier 3 exists");
        if (!target) {
            return failures == 0 ? 0 : 1;
        }

        lander::State pre;
        pre.x = 0.5 * (target->x_min + target->x_max);
        pre.y = 0.05;
        pre.vx = 0.0;
        pre.vy = -1.0;

        sim.set_state(pre);
        sim.start_recording();

        for (int i = 0; i < 40; ++i) {
            sim.advance(1.0 / 120.0, {});
        }

        auto frames = sim.stop_recording();

        check(sim.state().landed, "landing session lands");
        check(sim.state().score == 300, "landing on the x3 pad scores 300");

        const lander::State final_state = sim.state();

        sim.reset(555);
        sim.set_state(pre);
        sim.replay(frames);

        check_state_equal(sim.state(), final_state);
    }

    {
        // M02: replay never appends to an active recording
        lander::Simulation sim;
        sim.reset(13);

        sim.start_recording();
        sim.advance(1.0 / 120.0, {.main_thrust = true});
        sim.advance(1.0 / 120.0, {.rotate_right = true});
        auto first_pass = sim.stop_recording();
        check(first_pass.size() == 2, "two frames recorded on first pass");

        sim.start_recording();
        sim.replay(first_pass);
        auto second_pass = sim.stop_recording();
        check(second_pass.empty(),
              "replay does not append frames to an active recording");
    }

    return failures == 0 ? 0 : 1;
}
