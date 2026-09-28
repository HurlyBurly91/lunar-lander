#include "lander/sim.hpp"

#include <cmath>
#include <cstdint>

namespace lander {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// A single advance() call runs at most this many fixed steps. A much larger
// real-time gap (e.g. the process was suspended) would otherwise stall the
// application for a long time; excess time beyond this cap is discarded.
constexpr int kMaxStepsPerAdvance = 600;

// Score awarded for a safe landing, scaled by the pad multiplier.
constexpr int kScoreBase = 100;

// Maps any angle to the equivalent angle in (-pi, pi]. Angles that are
// equivalent modulo 2*pi therefore collapse to a single canonical value,
// which keeps orientation-based checks (e.g. landing uprightness) stable
// across the +/-pi representation boundary.
double normalize_angle(double angle) {
    double normalized = std::fmod(angle + kPi, kTwoPi);
    if (normalized < 0.0) {
        normalized += kTwoPi;
    }
    return normalized - kPi;
}

} // namespace

Simulation::Simulation(Config config)
    : config_(config) {
    reset(0);
}

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;
    state_ = State{};

    // The terrain (and the landing sites carved into it) is regenerated
    // from the seed, so the same seed always replays the same world.
    terrain_ = Terrain(seed_);
}

void Simulation::advance(double real_dt, Input input) {
    if (state_.landed || state_.crashed) {
        return;
    }
    if (real_dt <= 0.0 || !std::isfinite(real_dt)) {
        return;
    }

    accumulator_ += real_dt;
    const double max_accumulator =
        static_cast<double>(kMaxStepsPerAdvance) * config_.fixed_dt;
    if (accumulator_ > max_accumulator) {
        accumulator_ = max_accumulator;
    }

    // Fixed-timestep integration: the number of physics steps depends only
    // on the total elapsed simulated time, not on how real_dt was split
    // across advance() calls.
    while (accumulator_ >= config_.fixed_dt) {
        accumulator_ -= config_.fixed_dt;
        step_fixed(input);
        if (state_.landed || state_.crashed) {
            break;
        }
    }
}

const State& Simulation::state() const noexcept {
    return state_;
}

const Terrain& Simulation::terrain() const noexcept {
    return terrain_;
}

const std::vector<Pad>& Simulation::pads() const noexcept {
    return terrain_.pads();
}

void Simulation::set_state(const State& state) {
    state_ = state;
    state_.angle = normalize_angle(state_.angle);
}

void Simulation::step_fixed(Input input) {
    if (state_.landed || state_.crashed) {
        return;
    }

    const double dt = config_.fixed_dt;

    // Fuel: both the main engine and the rotation thrusters burn fuel.
    // Fuel is consumed in steps and clamped at zero.
    const bool main_active = input.main_thrust && state_.fuel > 0.0;
    const bool rotate_active =
        (input.rotate_left || input.rotate_right) && state_.fuel > 0.0;

    if (main_active) {
        state_.fuel -= config_.main_fuel_burn * dt;
        if (state_.fuel < 0.0) {
            state_.fuel = 0.0;
        }
    }
    if (rotate_active) {
        state_.fuel -= config_.rotation_fuel_burn * dt;
        if (state_.fuel < 0.0) {
            state_.fuel = 0.0;
        }
    }

    // Accelerations: gravity pulls down; main thrust acts along the
    // lander's local upward axis, which for angle 0 = upright and positive
    // angle = counter-clockwise is (-sin(angle), cos(angle)).
    double ax = 0.0;
    double ay = -config_.gravity;
    if (main_active) {
        ax += -config_.main_accel * std::sin(state_.angle);
        ay += config_.main_accel * std::cos(state_.angle);
    }

    double angular_accel = 0.0;
    if (input.rotate_left && rotate_active) {
        angular_accel -= config_.rotate_accel;
    }
    if (input.rotate_right && rotate_active) {
        angular_accel += config_.rotate_accel;
    }

    // Semi-implicit Euler: update velocities first, then advance positions.
    state_.omega += angular_accel * dt;
    state_.vx += ax * dt;
    state_.vy += ay * dt;
    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;
    state_.angle = normalize_angle(state_.angle + state_.omega * dt);

    ++state_.ticks;

    resolve_ground_contact();
}

void Simulation::resolve_ground_contact() {
    if (state_.landed || state_.crashed) {
        return;
    }

    // The lander is at (or below) the terrain surface at its horizontal
    // position. Clamp it onto the surface so it can never tunnel through
    // it, then classify the contact.
    const double ground = terrain_.height_at(state_.x);
    if (state_.y > ground) {
        return;
    }
    state_.y = ground;

    const Pad* pad = nullptr;
    for (const Pad& candidate : terrain_.pads()) {
        if (state_.x >= candidate.x_min && state_.x <= candidate.x_max) {
            pad = &candidate;
            break;
        }
    }

    if (pad == nullptr) {
        // Ground contact outside any landing pad is a crash.
        state_.crashed = true;
        return;
    }

    const double impact_speed = -state_.vy;
    const bool safe =
        std::abs(state_.vx) <= config_.safe_horizontal_speed &&
        impact_speed <= config_.safe_vertical_speed &&
        std::abs(normalize_angle(state_.angle)) <= config_.safe_angle_rad;

    if (!safe) {
        // Pad contact that violates any configured limit is a crash.
        state_.crashed = true;
        return;
    }

    // Safe landing: freeze the lander in a resting state on the pad.
    state_.landed = true;
    state_.score = kScoreBase * pad->multiplier;
    state_.vx = 0.0;
    state_.vy = 0.0;
    state_.omega = 0.0;
    state_.angle = 0.0;
}

} // namespace lander
