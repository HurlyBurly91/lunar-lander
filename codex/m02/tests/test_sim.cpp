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

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
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

    return failures == 0 ? 0 : 1;
}
