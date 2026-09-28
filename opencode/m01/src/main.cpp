#include "lander/sim.hpp"

#include <iostream>

int main() {
    lander::Simulation sim;
    sim.reset(42);

    for (int i = 0; i < 120; ++i) {
        sim.advance(1.0 / 120.0, {});
    }

    const auto& s = sim.state();

    std::cout
        << "x=" << s.x
        << " y=" << s.y
        << " vx=" << s.vx
        << " vy=" << s.vy
        << " fuel=" << s.fuel
        << '\n';
}
