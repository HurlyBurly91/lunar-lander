#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lander {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalized_angle(double angle) {
    return std::remainder(angle, 2.0 * kPi);
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

    constexpr int kPadCount = 3;
    constexpr double kPadHalfWidth = 1.5;
    constexpr double kMinPadGap = 0.5;
    constexpr double kWorldMin = -8.0;
    constexpr double kWorldMax = 8.0;
    constexpr double kFreeSpan = (kWorldMax - kWorldMin) -
        kPadCount * (2.0 * kPadHalfWidth) -
        (kPadCount - 1) * kMinPadGap;

    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> cut_dist(0.0, kFreeSpan);

    double cuts[kPadCount - 1];
    for (double& cut : cuts) {
        cut = cut_dist(rng);
    }
    std::sort(cuts, cuts + (kPadCount - 1));

    double gaps[kPadCount];
    gaps[0] = cuts[0];
    for (int i = 1; i < kPadCount - 1; ++i) {
        gaps[i] = cuts[i] - cuts[i - 1];
    }
    gaps[kPadCount - 1] = kFreeSpan - cuts[kPadCount - 2];

    pads_.clear();
    double left = kWorldMin + gaps[0];
    for (int i = 0; i < kPadCount; ++i) {
        const double center = left + kPadHalfWidth;
        pads_.push_back(Pad{
            .x_min = left,
            .x_max = center + kPadHalfWidth,
            .multiplier = i + 1
        });
        if (i + 1 < kPadCount) {
            left = center + kPadHalfWidth + kMinPadGap + gaps[i + 1];
        }
    }
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
    frames_.clear();
    recording_ = true;
}

std::vector<InputFrame> Simulation::stop_recording() {
    recording_ = false;
    auto recorded = std::move(frames_);
    frames_.clear();
    return recorded;
}

void Simulation::replay(const std::vector<InputFrame>& frames) {
    const bool was_recording = recording_;
    recording_ = false;
    for (const auto& frame : frames) {
        advance(frame.real_dt, frame.input);
    }
    recording_ = was_recording;
}

void Simulation::advance(double real_dt, Input input) {
    if (recording_) {
        frames_.push_back(InputFrame{real_dt, input});
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
