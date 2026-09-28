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

        a.reset(31337);
        b.reset(31337);

        check(a.pads() == b.pads(),
              "deterministic terrain");
    }

    {
        lander::Simulation a;
        lander::Simulation b;

        a.reset(44);
        b.reset(44);

        for (int i = 0; i < 60; ++i)
            a.advance(1.0 / 60.0, {});

        for (int i = 0; i < 30; ++i)
            b.advance(1.0 / 30.0, {});

        check(close(a.state().y, b.state().y),
              "frame-rate-independent position");

        check(close(a.state().vy, b.state().vy),
              "frame-rate-independent velocity");
    }

    {
        lander::Simulation sim;
        sim.reset(11);

        auto s = sim.state();
        s.omega = 5.0;
        sim.set_state(s);

        sim.reset(11);

        check(close(sim.state().omega, 0.0),
              "restart clears angular velocity");
    }

    {
        lander::Simulation sim;
        sim.reset(19);

        const auto pad = sim.pads().front();

        auto s = sim.state();
        s.x = (pad.x_min + pad.x_max) / 2.0;
        s.y = 0.001;
        s.vx = 0.0;
        s.vy = -1.0;
        s.angle = 0.0;
        sim.set_state(s);

        sim.advance(1.0 / 120.0, {});

        check(sim.state().landed || sim.state().crashed,
              "ground crossing resolved in same simulation step");

        check(sim.state().y >= 0.0,
              "ground penetration prevented");
    }

    {
        lander::Simulation sim;
        sim.reset(7);

        auto s = sim.state();
        s.fuel = 0.00001;
        sim.set_state(s);

        for (int i = 0; i < 10; ++i)
            sim.advance(1.0 / 120.0, {.main_thrust = true});

        check(sim.state().fuel >= 0.0,
              "fuel clamp");
    }

    {
        lander::Simulation sim;
        sim.reset(101);

        const auto pad = sim.pads().front();

        auto s = sim.state();
        s.x = (pad.x_min + pad.x_max) / 2.0;
        s.y = 0.0001;
        s.vx = 0.0;
        s.vy = -0.1;
        s.angle = 2.0 * 3.14159265358979323846 - 0.01;

        sim.set_state(s);
        sim.advance(1.0 / 120.0, {});

        check(sim.state().landed,
              "angle wraparound at landing");
    }

    return failures == 0 ? 0 : 1;
}
