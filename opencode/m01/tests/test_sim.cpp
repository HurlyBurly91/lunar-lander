#include "lander/sim.hpp"

#include <cmath>
#include <iostream>

namespace {

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

    return failures == 0 ? 0 : 1;
}
