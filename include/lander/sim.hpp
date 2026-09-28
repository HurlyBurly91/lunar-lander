#pragma once

#include "lander/terrain.hpp"

#include <cstdint>
#include <vector>

namespace lander {

struct Input {
    // Normalised main-engine throttle: 0.0 = engine off, 1.0 = full thrust.
    // It scales both the main-engine acceleration and the main-engine fuel
    // burn linearly (see Simulation::step_fixed). Values outside [0, 1] are
    // clamped before use, so callers cannot change the physics.
    double main_throttle{};
    bool rotate_left{};
    bool rotate_right{};

    bool operator==(const Input&) const = default;
};

struct Config {
    double gravity{1.62};
    double main_accel{5.0};
    double rotate_accel{1.6};
    double main_fuel_burn{8.0};
    double rotation_fuel_burn{1.5};

    double fixed_dt{1.0 / 120.0};

    double safe_vertical_speed{2.0};
    double safe_horizontal_speed{1.0};
    double safe_angle_rad{0.15};
};

struct State {
    double x{};
    double y{20.0};
    double vx{};
    double vy{};
    double angle{};
    double omega{};
    double fuel{100.0};

    bool landed{};
    bool crashed{};
    int score{};
    std::uint64_t ticks{};

    bool operator==(const State&) const = default;
};

class Simulation {
public:
    explicit Simulation(Config config = {});

    void reset(std::uint64_t seed);
    void advance(double real_dt, Input input);

    const State& state() const noexcept;
    const Terrain& terrain() const noexcept;
    const std::vector<Pad>& pads() const noexcept;

    void set_state(const State& state);

private:
    void step_fixed(Input input);
    void resolve_ground_contact();

    Config config_;
    State state_;
    Terrain terrain_{};
    double accumulator_{};
    std::uint64_t seed_{};
};

} // namespace lander
