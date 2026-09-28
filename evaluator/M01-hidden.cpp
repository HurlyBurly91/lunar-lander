#include "lander/sim.hpp"

#include <cmath>
#include <iostream>

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

}

int main() {
    {
        lander::Simulation a;
        lander::Simulation b;
        a.reset(918273);
        b.reset(918273);

        check(!a.pads().empty(), "pad generation");
        check(a.pads() == b.pads(), "seed determinism");
    }

    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(77);
        b.reset(77);

        for (int i = 0; i < 120; ++i)
            a.advance(1.0 / 120.0, {});

        for (int i = 0; i < 40; ++i)
            b.advance(1.0 / 40.0, {});

        check(close(a.state().y, b.state().y),
              "fixed timestep position");

        check(close(a.state().vy, b.state().vy),
              "fixed timestep velocity");
    }

    {
        lander::Simulation sim;
        sim.reset(12);

        auto s = sim.state();
        s.fuel = 0.0001;
        sim.set_state(s);

        for (int i = 0; i < 100; ++i)
            sim.advance(1.0 / 120.0, {.main_thrust = true});

        check(sim.state().fuel >= 0.0, "fuel clamp");
    }

    {
        lander::Simulation sim;
        sim.reset(55);

        const auto pad = sim.pads().front();

        auto s = sim.state();
        s.x = (pad.x_min + pad.x_max) / 2.0;
        s.y = 0.001;
        s.vx = 0.1;
        s.vy = -0.1;
        s.angle = 0.0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().landed && !sim.state().crashed,
              "safe landing");
    }

    {
        lander::Simulation sim;
        sim.reset(55);

        const auto pad = sim.pads().front();

        auto s = sim.state();
        s.x = (pad.x_min + pad.x_max) / 2.0;
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -8.0;
        s.angle = 0.0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().crashed && !sim.state().landed,
              "unsafe impact");
    }

    {
        lander::Simulation sim;
        sim.reset(55);

        const auto pad = sim.pads().front();

        auto s = sim.state();
        s.x = (pad.x_min + pad.x_max) / 2.0;
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -0.1;
        s.angle = 2.0 * 3.14159265358979323846 - 0.01;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().landed,
              "angle equivalence modulo 2pi");
    }

    return failures == 0 ? 0 : 1;
}
