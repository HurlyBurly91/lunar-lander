#include "lander/sim.hpp"

namespace lander {

Simulation::Simulation(Config config)
    : config_(config) {
    reset(0);
}

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;
    state_ = State{};

    // TODO: deterministic terrain/pad generation.
    pads_.clear();
}

void Simulation::advance(double, Input) {
    // TODO
}

const State& Simulation::state() const noexcept {
    return state_;
}

const std::vector<Pad>& Simulation::pads() const noexcept {
    return pads_;
}

void Simulation::set_state(const State& state) {
    state_ = state;
}

void Simulation::step_fixed(Input) {
    // TODO
}

void Simulation::resolve_ground_contact() {
    // TODO
}

} // namespace lander
