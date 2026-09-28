#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lander {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// Wraps an angle into the canonical [-pi, pi] representation so that angles
// equivalent modulo 2*pi compare identically.
double normalize_angle(double angle) {
    double wrapped = std::fmod(angle + kPi, kTwoPi);
    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }
    return wrapped - kPi;
}

} // namespace

Simulation::Simulation(Config config)
    : config_(config) {
    reset(0);
}

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;

    // A reset restores the full initial state, including angular velocity.
    state_ = State{};

    // The seed is the sole source of terrain randomness, so two games with
    // the same seed always generate identical landing pads.
    std::mt19937_64 rng(seed);

    std::uniform_real_distribution<double> center_dist(-6.0, 6.0);
    const double center = center_dist(rng);

    pads_.clear();
    pads_.push_back(Pad{
        .x_min = center - 1.5,
        .x_max = center + 1.5,
        .multiplier = 1
    });
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

void Simulation::advance(double real_dt, Input input) {
    if (real_dt <= 0.0 || state_.landed || state_.crashed) {
        return;
    }

    // Fixed-timestep integration: accumulate elapsed time and run as many
    // fixed physics steps as fit, so the result depends on elapsed simulated
    // time, not on how advance() is chunked.
    accumulator_ += real_dt;
    const double dt = config_.fixed_dt;

    while (accumulator_ >= dt) {
        accumulator_ -= dt;
        step_fixed(input);
        if (state_.landed || state_.crashed) {
            break;
        }
    }
}

void Simulation::step_fixed(Input input) {
    const double dt = config_.fixed_dt;

    // The lander is already touching the surface (for example an injected
    // state): settle it and resolve contact immediately.
    if (state_.y <= 0.0) {
        state_.y = 0.0;
        resolve_ground_contact();
        return;
    }

    double rotational_direction = 0.0;

    if (input.rotate_left) {
        rotational_direction += 1.0;
    }

    if (input.rotate_right) {
        rotational_direction -= 1.0;
    }

    if (rotational_direction != 0.0 && state_.fuel > 0.0) {
        state_.omega += rotational_direction * config_.rotate_accel * dt;
        state_.fuel -= config_.rotation_fuel_burn * dt;
    }

    double ax = 0.0;
    double ay = -config_.gravity;

    if (input.main_thrust && state_.fuel > 0.0) {
        ax += -std::sin(state_.angle) * config_.main_accel;
        ay +=  std::cos(state_.angle) * config_.main_accel;
        state_.fuel -= config_.main_fuel_burn * dt;
    }

    // Fuel is finite and may never become negative.
    state_.fuel = std::max(0.0, state_.fuel);

    state_.vx += ax * dt;
    state_.vy += ay * dt;

    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;

    // Keep the angle in the canonical [-pi, pi] representation.
    state_.angle = normalize_angle(state_.angle + state_.omega * dt);

    // A step that crosses the surface must settle into it in the same step,
    // so the lander never rests below the surface between calls.
    if (state_.y <= 0.0) {
        state_.y = 0.0;
        resolve_ground_contact();
    }

    ++state_.ticks;
}

void Simulation::resolve_ground_contact() {
    const Pad* contacted = nullptr;

    for (const auto& pad : pads_) {
        if (state_.x >= pad.x_min && state_.x <= pad.x_max) {
            contacted = &pad;
            break;
        }
    }

    if (!contacted) {
        state_.crashed = true;
        return;
    }

    // Landing compares the angular distance from upright, so angles that
    // differ by a multiple of 2*pi are treated equivalently.
    state_.angle = normalize_angle(state_.angle);
    const double angular_deviation = std::abs(state_.angle);

    const bool safe =
        std::abs(state_.vx) <= config_.safe_horizontal_speed &&
        std::abs(state_.vy) <= config_.safe_vertical_speed &&
        angular_deviation <= config_.safe_angle_rad;

    if (safe) {
        state_.landed = true;
        state_.score = 100 * contacted->multiplier;
    } else {
        state_.crashed = true;
    }
}

} // namespace lander
