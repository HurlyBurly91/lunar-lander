#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void check(bool value, const char* name) {
    if (!value) {
        ++failures;
        std::cerr << "HIDDEN FAIL: " << name << '\n';
    }
}

bool close(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

bool same_state(const lander::State& a, const lander::State& b) {
    return
        close(a.x, b.x) &&
        close(a.y, b.y) &&
        close(a.vx, b.vx) &&
        close(a.vy, b.vy) &&
        close(a.angle, b.angle) &&
        close(a.omega, b.omega) &&
        close(a.fuel, b.fuel) &&
        a.landed == b.landed &&
        a.crashed == b.crashed &&
        a.score == b.score &&
        a.ticks == b.ticks;
}

}

int main() {
    {
        lander::Simulation sim;
        sim.reset(123);

        check(sim.pads().size() == 3,
              "exactly three pads");

        std::vector<int> multipliers;

        for (const auto& p : sim.pads())
            multipliers.push_back(p.multiplier);

        std::sort(multipliers.begin(), multipliers.end());

        check(multipliers == std::vector<int>({1, 2, 3}),
              "pad multipliers");

        auto pads = sim.pads();
        std::sort(pads.begin(), pads.end(),
                  [](const auto& a, const auto& b) {
                      return a.x_min < b.x_min;
                  });

        check(pads[0].x_max < pads[1].x_min &&
              pads[1].x_max < pads[2].x_min,
              "pads do not overlap");
    }

    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(987654);
        b.reset(987654);

        check(a.pads() == b.pads(),
              "three-pad generation deterministic");
    }

    {
        lander::Config light_cfg;
        light_cfg.rotation_torque = 2.0;
        light_cfg.moment_of_inertia = 1.0;

        lander::Config heavy_cfg = light_cfg;
        heavy_cfg.moment_of_inertia = 2.0;

        lander::Simulation light(light_cfg);
        lander::Simulation heavy(heavy_cfg);

        light.reset(1);
        heavy.reset(1);

        light.advance(light_cfg.fixed_dt, {.rotate_left = true});
        heavy.advance(heavy_cfg.fixed_dt, {.rotate_left = true});

        check(close(light.state().omega, 2.0 * heavy.state().omega, 1e-5),
              "torque divided by moment of inertia");
    }

    {
        lander::Simulation source;
        source.reset(222);

        source.start_recording();

        for (int i = 0; i < 30; ++i)
            source.advance(1.0 / 60.0,
                           {.main_thrust = i > 10,
                            .rotate_left = i < 5});

        const auto expected = source.state();
        const auto frames = source.stop_recording();

        check(frames.size() == 30,
              "one recording frame per advance call");

        lander::Simulation replayed;
        replayed.reset(222);
        replayed.replay(frames);

        check(same_state(expected, replayed.state()),
              "record/replay determinism");
    }

    {
        lander::Simulation sim;
        sim.reset(500);

        for (const auto& pad : sim.pads()) {
            sim.reset(500);

            const auto& pads = sim.pads();
            const auto it = std::find_if(
                pads.begin(),
                pads.end(),
                [&](const auto& p) {
                    return p.multiplier == pad.multiplier;
                });

            auto s = sim.state();
            s.x = (it->x_min + it->x_max) / 2.0;
            s.y = 0.001;
            s.vx = 0.0;
            s.vy = -0.1;
            s.angle = 0.0;
            sim.set_state(s);

            sim.advance(1.0 / 120.0, {});

            check(sim.state().landed,
                  "safe pad landing");

            check(sim.state().score == 100 * it->multiplier,
                  "pad score multiplier");
        }
    }

    return failures == 0 ? 0 : 1;
}
