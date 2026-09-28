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
    s.y = pad.y + 0.3;
    s.vx = 0.2;
    s.vy = -1.0;
    s.angle = angle;
    s.fuel = 50.0;
    return s;
}

// True when x is not inside any landing site.
bool outside_all_pads(const lander::Terrain& terrain, double x) {
    for (const lander::Pad& pad : terrain.pads()) {
        if (x >= pad.x_min && x <= pad.x_max) {
            return false;
        }
    }
    return true;
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
            sim.advance(1.0 / 120.0, {.main_throttle = 1.0});
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

        sim.advance(0.5, {.main_throttle = 1.0, .rotate_left = true});
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
        check(close(sim.state().y, pad.y, 1e-9),
              "landed lander must rest on the pad surface");
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
        // Ground contact on jagged (non-pad) terrain crashes, even with a
        // gentle descent profile. The lander starts just above the surface
        // at a point that is not part of any landing site.
        lander::Simulation sim;
        sim.reset(2024);
        const lander::Pad& pad = sim.pads().front();

        double x = pad.x_max + 50.0;
        while (!outside_all_pads(sim.terrain(), x)) {
            x += 1.0;
        }
        lander::State s{};
        s.x = x;
        s.y = sim.terrain().height_at(x) + 0.3;
        s.vx = 0.2;
        s.vy = -1.0;
        s.fuel = 50.0;
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
        // Copy the pad by value: reset() below regenerates the terrain,
        // which would invalidate a reference into it.
        const lander::Pad pad = sim.pads().front();

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
                        {.main_throttle = 1.0, .rotate_left = true,
                         .rotate_right = true});
        }
        check(sim.state() == before,
              "terminal state must not change on further advances");
        check(sim.pads() == before_pads, "pads must not change after crash");
    }

    {
        // Terrain: the same seed must produce the identical surface.
        lander::Simulation a;
        lander::Simulation b;
        a.reset(1234);
        b.reset(1234);

        check(a.terrain().pads() == b.terrain().pads(),
              "same seed must produce identical landing sites");
        bool same_heights = true;
        for (double x = -200.0; x <= 200.0; x += 0.5) {
            if (a.terrain().height_at(x) != b.terrain().height_at(x)) {
                same_heights = false;
                break;
            }
        }
        check(same_heights, "same seed must produce identical terrain heights");
    }

    {
        // Terrain: different seeds must produce different surfaces.
        lander::Simulation a;
        lander::Simulation b;
        a.reset(7);
        b.reset(8);

        bool different = a.terrain().pads() != b.terrain().pads();
        if (!different) {
            for (double x = -200.0; x <= 200.0; x += 0.5) {
                if (a.terrain().height_at(x) != b.terrain().height_at(x)) {
                    different = true;
                    break;
                }
            }
        }
        check(different, "different seeds should produce different terrain");
    }

    {
        // Terrain: every generated landing site is a flat, landable
        // section of the surface, and there are several of them.
        lander::Simulation sim;
        sim.reset(1234);
        const auto& pads = sim.terrain().pads();

        check(pads.size() >= 2,
              "terrain should contain several landing sites");
        for (const lander::Pad& pad : pads) {
            check(pad.x_max - pad.x_min >= 6.0,
                  "a landing site must be wide enough for the lander");
            bool flat = true;
            for (double x = pad.x_min; x <= pad.x_max + 1e-9; x += 0.25) {
                if (sim.terrain().height_at(x) != pad.y) {
                    flat = false;
                    break;
                }
            }
            check(flat, "every generated landing site must be flat");
        }
    }

    {
        // Terrain: height queries are deterministic and the surface is
        // visibly uneven (not a flat plane).
        lander::Simulation a;
        lander::Simulation b;
        a.reset(2024);
        b.reset(2024);

        double min_h = 1e30;
        double max_h = -1e30;
        bool deterministic = true;
        for (double x = -150.0; x <= 150.0; x += 0.5) {
            const double ha = a.terrain().height_at(x);
            if (ha != b.terrain().height_at(x)) {
                deterministic = false;
            }
            min_h = std::min(min_h, ha);
            max_h = std::max(max_h, ha);
        }
        check(deterministic, "terrain height queries must be deterministic");
        check(max_h - min_h > 5.0,
              "terrain must be visibly uneven, not a flat plane");
    }

    {
        // The spawn point must always start above the generated terrain.
        for (std::uint64_t seed : {0ULL, 5ULL, 7ULL, 99ULL, 1234ULL,
                                   2024ULL}) {
            lander::Simulation sim;
            sim.reset(seed);
            check(sim.terrain().height_at(0.0) < 20.0,
                  "spawn point must start above the terrain");
        }
    }

    {
        // Throttle 0 produces no main-engine acceleration or main-engine fuel
        // burn: only gravity acts, and the fuel gauge is untouched.
        const lander::Config config{};
        lander::Simulation sim(config);
        sim.reset(1);
        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        sim.set_state(s);
        const lander::State before = sim.state();

        sim.advance(config.fixed_dt, {.main_throttle = 0.0});
        const lander::State& after = sim.state();

        check(close(after.vy, before.vy - config.gravity * config.fixed_dt,
                    1e-9),
              "throttle 0 applies no main-engine acceleration");
        check(close(after.vx, before.vx, 1e-9),
              "throttle 0 leaves horizontal velocity unchanged");
        check(close(after.fuel, before.fuel, 1e-9),
              "throttle 0 burns no main-engine fuel");
        check(close(after.omega, before.omega, 1e-9),
              "throttle 0 does not spin the lander");
    }

    {
        // Throttle 1 preserves the existing full-thrust behaviour.
        const lander::Config config{};
        lander::Simulation sim(config);
        sim.reset(1);
        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        sim.set_state(s);
        const lander::State before = sim.state();

        sim.advance(config.fixed_dt, {.main_throttle = 1.0});
        const lander::State& after = sim.state();

        check(
            close(after.vy,
                  before.vy +
                      (config.main_accel - config.gravity) * config.fixed_dt,
                  1e-9),
            "throttle 1 preserves the full main-engine acceleration");
        check(close(after.fuel, before.fuel - config.main_fuel_burn *
                                              config.fixed_dt,
                    1e-9),
              "throttle 1 burns the full main-engine fuel rate");
    }

    {
        // Throttle 0.5 produces approximately half the main-engine
        // acceleration and burns approximately half the main-engine fuel.
        const lander::Config config{};
        lander::Simulation sim(config);
        sim.reset(1);
        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        sim.set_state(s);
        const lander::State before = sim.state();

        sim.advance(config.fixed_dt, {.main_throttle = 0.5});
        const lander::State& after = sim.state();

        check(
            close(after.vy,
                  before.vy +
                      (0.5 * config.main_accel - config.gravity) *
                          config.fixed_dt,
                  1e-9),
            "throttle 0.5 applies half the main-engine acceleration");
        check(close(after.fuel, before.fuel - 0.5 * config.main_fuel_burn *
                                              config.fixed_dt,
                    1e-9),
              "throttle 0.5 burns half the main-engine fuel");
    }

    {
        // Out-of-range throttle values are clamped to [0, 1].
        const lander::Config config{};

        lander::Simulation hi(config);
        hi.reset(1);
        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        hi.set_state(s);
        hi.advance(config.fixed_dt, {.main_throttle = 2.0});
        check(close(hi.state().vy,
                    (-config.gravity + config.main_accel) * config.fixed_dt,
                    1e-9),
              "throttle > 1 is clamped to full thrust");
        check(close(hi.state().fuel, 50.0 - config.main_fuel_burn *
                                             config.fixed_dt,
                    1e-9),
              "throttle > 1 burns at most the full fuel rate");

        lander::Simulation lo(config);
        lo.reset(1);
        lo.set_state(s);
        lo.advance(config.fixed_dt, {.main_throttle = -1.0});
        check(close(lo.state().vy, -config.gravity * config.fixed_dt, 1e-9),
              "throttle < 0 is clamped to engine off");
        check(close(lo.state().fuel, 50.0, 1e-9),
              "throttle < 0 burns no main-engine fuel");
    }

    {
        // Rotation behaviour is independent of the main-engine throttle.
        const lander::Config config{};

        lander::Simulation off(config);
        off.reset(1);
        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        off.set_state(s);
        off.advance(config.fixed_dt, {.main_throttle = 0.0,
                                     .rotate_left = true});

        lander::Simulation full(config);
        full.reset(1);
        full.set_state(s);
        full.advance(config.fixed_dt, {.main_throttle = 1.0,
                                      .rotate_left = true});

        check(close(off.state().omega, -config.rotate_accel * config.fixed_dt,
                    1e-9),
              "rotation acceleration is unchanged at throttle 0");
        check(close(full.state().omega, off.state().omega, 1e-9),
              "rotation is independent of the main-engine throttle");
        check(close(full.state().angle, off.state().angle, 1e-9),
              "angle is independent of the main-engine throttle");
    }

    {
        // Fixed-timestep determinism is preserved with a non-zero throttle.
        const lander::Config config{};
        lander::Simulation a(config);
        lander::Simulation b(config);
        a.reset(42);
        b.reset(42);

        lander::State s{};
        s.x = 0.0;
        s.y = 1000.0;
        s.fuel = 50.0;
        a.set_state(s);
        b.set_state(s);

        const lander::Input input{.main_throttle = 0.3};
        for (int i = 0; i < 60; ++i) {
            a.advance(1.0 / 60.0, input);
        }
        for (int i = 0; i < 30; ++i) {
            b.advance(1.0 / 30.0, input);
        }

        check(close(a.state().x, b.state().x, 1e-7) &&
                  close(a.state().y, b.state().y, 1e-7) &&
                  close(a.state().vx, b.state().vx, 1e-7) &&
                  close(a.state().vy, b.state().vy, 1e-7) &&
                  close(a.state().fuel, b.state().fuel, 1e-7) &&
                  close(a.state().angle, b.state().angle, 1e-7),
              "throttle behaviour must be independent of advance frequency");
    }

    return failures == 0 ? 0 : 1;
}
