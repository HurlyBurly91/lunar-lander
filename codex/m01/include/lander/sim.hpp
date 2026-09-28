#pragma once

#include <cstdint>
#include <vector>

namespace lander {

struct Input {
    bool main_thrust{};
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

struct Pad {
    double x_min{};
    double x_max{};
    int multiplier{1};

    bool operator==(const Pad&) const = default;
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
    const std::vector<Pad>& pads() const noexcept;

    void set_state(const State& state);

private:
    void step_fixed(Input input);
    void resolve_ground_contact();

    Config config_;
    State state_;
    std::vector<Pad> pads_;
    double accumulator_{};
    std::uint64_t seed_{};
};

} // namespace lander
