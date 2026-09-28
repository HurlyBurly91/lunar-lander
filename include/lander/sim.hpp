#pragma once

#include "lander/terrain.hpp"

#include <cmath>
#include <cstdint>

namespace lander {

struct Config {
    double fixed_dt{1.0 / 120.0};
    double mu{178976.334};
    double main_accel{4.0};
    double rotate_accel{1.2};
    double fuel{100.0};
    double fuel_burn{8.0};
    double safe_vertical_speed{2.0};
    double safe_horizontal_speed{1.0};
    double safe_angle_rad{0.15};
};

struct State {
    double x{};
    double y{};
    double vx{};
    double vy{};
    double angle{};
    double omega{};
    double fuel{};
    bool landed{false};
    bool crashed{false};
    int score{0};
    int ticks{0};
    bool operator==(const State&) const = default;
};

struct Input {
    bool rotate_left{false};
    bool rotate_right{false};
    double main_throttle{0.0};
};

struct LocalVelocity {
    double radial{};
    double tangential{};
};

bool operator==(const Input& lhs, const Input& rhs);
bool operator==(const Config& lhs, const Config& rhs);

double radial_distance(const State& state);
double local_up_angle(const State& state);
double local_attitude_angle(const State& state);
LocalVelocity local_velocity(const State& state);
double surface_radius_at(const Terrain& terrain, const State& state);
double altitude_at(const Terrain& terrain, const State& state);

class Simulation {
public:
    Simulation() = default;
    explicit Simulation(const Config& config);

    void reset(std::uint64_t seed);
    void advance(double elapsed, const Input& input);
    const State& state() const noexcept { return state_; }
    const Config& config() const noexcept { return config_; }
    const Terrain& terrain() const noexcept { return terrain_; }
    std::uint64_t seed() const noexcept { return seed_; }
    double accumulator() const noexcept { return accumulator_; }

    void set_state(const State& state);
    void set_config(const Config& config) { config_ = config; }

private:
    void step_fixed(const Input& input);
    void resolve_ground_contact();

    Config config_;
    State state_;
    Terrain terrain_{0};
    std::uint64_t seed_{0};
    double accumulator_{0.0};
};

}  // namespace lander
