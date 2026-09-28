#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lander {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPadHalfWidth = 1.5;

double normalized_angle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
}

// The landing zone is split into three disjoint slots. Each slot contributes
// one pad whose center is jittered by the seed within the slot, inset by the
// pad half-width. Pads are therefore deterministic, seeded, and
// non-overlapping by construction, with multipliers 1, 2, 3 in slot order.
std::vector<Pad> generate_pads(std::uint64_t seed) {
    std::mt19937_64 rng(seed);

    const double slot_bounds[3][2] = {
        {-6.0, -2.0},
        {-2.0, 2.0},
        {2.0, 6.0},
    };
    const int multipliers[3] = {1, 2, 3};

    std::vector<Pad> pads;
    pads.reserve(3);

    for (int i = 0; i < 3; ++i) {
        std::uniform_real_distribution<double> center_dist(
            slot_bounds[i][0] + kPadHalfWidth,
            slot_bounds[i][1] - kPadHalfWidth);

        const double center = center_dist(rng);

        pads.push_back(Pad{
            .x_min = center - kPadHalfWidth,
            .x_max = center + kPadHalfWidth,
            .multiplier = multipliers[i]
        });
    }

    return pads;
}

}

Simulation::Simulation(Config config)
    : config_(config) {
    reset(0);
}

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;
    state_ = State{};
    pads_ = generate_pads(seed);
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

void Simulation::start_recording() {
    recorded_frames_.clear();
    recording_ = true;
}

std::vector<InputFrame> Simulation::stop_recording() {
    recording_ = false;
    auto frames = std::move(recorded_frames_);
    recorded_frames_.clear();
    return frames;
}

void Simulation::replay(const std::vector<InputFrame>& frames) {
    replaying_ = true;
    for (const auto& frame : frames) {
        advance(frame.real_dt, frame.input);
    }
    replaying_ = false;
}

void Simulation::advance(double real_dt, Input input) {
    if (recording_ && !replaying_) {
        recorded_frames_.push_back(InputFrame{real_dt, input});
    }

    if (real_dt <= 0.0 || state_.landed || state_.crashed) {
        return;
    }

    accumulator_ += std::min(real_dt, 0.25);

    while (accumulator_ + 1e-12 >= config_.fixed_dt) {
        step_fixed(input);
        accumulator_ -= config_.fixed_dt;

        if (state_.landed || state_.crashed) {
            break;
        }
    }
}

void Simulation::step_fixed(Input input) {
    const double dt = config_.fixed_dt;

    double rotational_direction = 0.0;

    if (input.rotate_left) {
        rotational_direction += 1.0;
    }

    if (input.rotate_right) {
        rotational_direction -= 1.0;
    }

    if (rotational_direction != 0.0 && state_.fuel > 0.0) {
        const double angular_acceleration =
            config_.rotation_torque / config_.moment_of_inertia;
        state_.omega += rotational_direction * angular_acceleration * dt;
        state_.fuel -= config_.rotation_fuel_burn * dt;
    }

    double ax = 0.0;
    double ay = -config_.gravity;

    if (input.main_thrust && state_.fuel > 0.0) {
        ax += -std::sin(state_.angle) * config_.main_accel;
        ay +=  std::cos(state_.angle) * config_.main_accel;
        state_.fuel -= config_.main_fuel_burn * dt;
    }

    state_.fuel = std::max(0.0, state_.fuel);

    state_.vx += ax * dt;
    state_.vy += ay * dt;

    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;

    state_.angle = normalized_angle(state_.angle + state_.omega * dt);

    ++state_.ticks;

    if (state_.y <= 0.0) {
        state_.y = 0.0;
        resolve_ground_contact();
    }
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

    const double angle_error = std::abs(normalized_angle(state_.angle));

    const bool safe =
        std::abs(state_.vx) <= config_.safe_horizontal_speed &&
        std::abs(state_.vy) <= config_.safe_vertical_speed &&
        angle_error <= config_.safe_angle_rad;

    if (safe) {
        state_.landed = true;
        state_.score = 100 * contacted->multiplier;
    } else {
        state_.crashed = true;
    }
}

} // namespace lander
