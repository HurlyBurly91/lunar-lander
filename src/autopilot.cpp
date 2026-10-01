#include "lander/autopilot.hpp"

#include <algorithm>
#include <cmath>

namespace lander {

namespace {

double wrap_pi(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

double vec_length(const Vec2& v) { return std::hypot(v.x, v.y); }

Vec2 normalize_safe(const Vec2& v) {
    const double r = vec_length(v);
    if (r < 1.0e-12) {
        return {};
    }
    return {v.x / r, v.y / r};
}

int attitude_command(const State& state, const Config& config,
                     const std::optional<Vec2>& direction) {
    if (!direction || vec_length(*direction) < 1.0e-9) {
        return 0;
    }
    // thrust_hat(theta) = {-sin(theta), cos(theta)}, so the nose angle that
    // points along `d` is atan2(-d.x, d.y).
    const double desired = std::atan2(-(*direction).x, (*direction).y);
    const double error = wrap_pi(desired - state.angle);
    constexpr double kAngleDeadband = 0.01;
    constexpr double kRateDeadband = 0.02;

    if (std::abs(error) <= kAngleDeadband &&
        std::abs(state.omega) <= kRateDeadband) {
        return 0;
    }
    if (state.omega != 0.0) {
        if (std::abs(error) <= kAngleDeadband) {
            return -std::copysign(1.0, state.omega);
        }
        if (state.omega * error > 0.0 && config.rotate_accel > 0.0) {
            const double brake_distance =
                state.omega * state.omega / (2.0 * config.rotate_accel);
            if (brake_distance >= std::abs(error)) {
                return -std::copysign(1.0, state.omega);
            }
        }
    }
    return error > 0.0 ? 1 : -1;
}

}  // namespace

const char* attitude_mode_name(AttitudeMode mode) {
    switch (mode) {
        case AttitudeMode::Off: return "OFF";
        case AttitudeMode::Prograde: return "PROGRADE";
        case AttitudeMode::Retrograde: return "RETROGRADE";
        case AttitudeMode::RadialOut: return "RAD OUT";
        case AttitudeMode::RadialIn: return "RAD IN";
        case AttitudeMode::Target: return "TARGET";
        case AttitudeMode::AntiTarget: return "ANTI-TARGET";
        case AttitudeMode::Maneuver: return "MANEUVER";
    }
    return "OFF";
}

std::optional<Vec2> attitude_target_direction(
    AttitudeMode mode, const NodeBasis& basis, const Vec2& ship_position,
    const Vec2& target_position, const Vec2& maneuver_dv) {
    switch (mode) {
        case AttitudeMode::Off:
            return std::nullopt;
        case AttitudeMode::Prograde:
            return vec_length(basis.prograde) > 1.0e-9
                       ? std::optional<Vec2>(basis.prograde)
                       : std::nullopt;
        case AttitudeMode::Retrograde:
            return vec_length(basis.prograde) > 1.0e-9
                       ? std::optional<Vec2>(basis.prograde * -1.0)
                       : std::nullopt;
        case AttitudeMode::RadialOut:
            return vec_length(basis.radial_out) > 1.0e-9
                       ? std::optional<Vec2>(basis.radial_out)
                       : std::nullopt;
        case AttitudeMode::RadialIn:
            return vec_length(basis.radial_out) > 1.0e-9
                       ? std::optional<Vec2>(basis.radial_out * -1.0)
                       : std::nullopt;
        case AttitudeMode::Target: {
            const Vec2 d = target_position - ship_position;
            const Vec2 n = normalize_safe(d);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
        case AttitudeMode::AntiTarget: {
            const Vec2 d = ship_position - target_position;
            const Vec2 n = normalize_safe(d);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
        case AttitudeMode::Maneuver: {
            const Vec2 n = normalize_safe(maneuver_dv);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
    }
    return std::nullopt;
}

Input attitude_input(const State& state, const Config& config,
                     const std::optional<Vec2>& target_direction,
                     bool manual_left, bool manual_right) {
    Input input{};
    if (manual_left) {
        input.rotate_left = true;
    }
    if (manual_right) {
        input.rotate_right = true;
    }
    if (manual_left || manual_right) {
        return input;
    }
    const int command = attitude_command(state, config, target_direction);
    if (command > 0) {
        input.rotate_right = true;
    } else if (command < 0) {
        input.rotate_left = true;
    }
    return input;
}

const char* executor_state_name(ExecutorState state) {
    switch (state) {
        case ExecutorState::Idle: return "IDLE";
        case ExecutorState::Wait: return "WAIT";
        case ExecutorState::Align: return "ALIGN";
        case ExecutorState::Burn: return "BURN";
        case ExecutorState::Complete: return "COMPLETE";
        case ExecutorState::Aborted: return "ABORTED";
        case ExecutorState::Incomplete: return "INCOMPLETE";
    }
    return "IDLE";
}

void NodeExecutor::clear() {
    *this = NodeExecutor{};
}

void NodeExecutor::arm(const ManeuverNode& node, const NodeBasis& basis,
                       double now, const Config& config) {
    clear();
    frame_body_ = node.frame_body;
    node_time_ = node.time;
    dv_total_ = node_world_dv(node, basis);
    dv_remaining_ = dv_total_;

    if (config.main_accel <= 0.0 ||
        vec_length(dv_total_) <= 1.0e-9) {
        state_ = ExecutorState::Complete;
        return;
    }

    burn_time_ = vec_length(dv_total_) / config.main_accel;
    ignite_time_ = node_time_ - 0.5 * burn_time_;
    if (now < ignite_time_) {
        state_ = ExecutorState::Wait;
    } else {
        state_ = ExecutorState::Align;
        late_ = now >= node_time_;
    }
}

void NodeExecutor::abort() {
    if (active()) {
        state_ = ExecutorState::Aborted;
    }
}

bool NodeExecutor::aligned(const State& state) const {
    const Vec2 direction = normalize_safe(dv_remaining_);
    if (vec_length(direction) < 1.0e-9) {
        return true;
    }
    const double desired = std::atan2(-direction.x, direction.y);
    const double error = std::abs(wrap_pi(desired - state.angle));
    return error <= 0.05 && std::abs(state.omega) <= 0.1;
}

Input NodeExecutor::make_input(const State& state, double now,
                               const Config& config, bool manual_left,
                               bool manual_right) const {
    Input input{};
    if (!active()) {
        return input;
    }

    // The Align -> Burn transition happens on the first step at or after
    // ignition (or late, at the node), so the input for that step must
    // already contain the thrust command; otherwise the finite burn would
    // slip one fixed step.
    const bool burning =
        state_ == ExecutorState::Burn ||
        (state_ == ExecutorState::Align && now >= ignite_time_ &&
         (aligned(state) || now >= node_time_));

    if (state_ == ExecutorState::Align || burning) {
        const Vec2 direction = normalize_safe(dv_remaining_);
        input = attitude_input(state, config,
                               vec_length(direction) > 1.0e-9
                                   ? std::optional<Vec2>(direction)
                                   : std::nullopt,
                               manual_left, manual_right);
    }

    if (burning) {
        const double step_dv = config.main_accel * config.fixed_dt;
        if (step_dv > 0.0) {
            const double magnitude = vec_length(dv_remaining_);
            input.main_throttle = std::clamp(magnitude / step_dv, 0.0, 1.0);
        }
    }

    return input;
}

void NodeExecutor::after_step(const State& before, const State& after,
                              const Input& input, double now,
                              const Config& config) {
    if (!active()) {
        return;
    }

    if (after.crashed) {
        state_ = ExecutorState::Aborted;
        return;
    }

    if (state_ == ExecutorState::Wait && now >= ignite_time_) {
        state_ = ExecutorState::Align;
    }

    if (state_ == ExecutorState::Align &&
        (aligned(after) || now >= node_time_)) {
        state_ = ExecutorState::Burn;
    }

    if (state_ == ExecutorState::Burn) {
        const double throttle = std::clamp(input.main_throttle, 0.0, 1.0);
        if (before.fuel > 0.0 && throttle > 0.0) {
            const Vec2 thrust_hat{-std::sin(before.angle),
                                  std::cos(before.angle)};
            const double dv_step =
                config.main_accel * throttle * config.fixed_dt;
            dv_remaining_ = dv_remaining_ - thrust_hat * dv_step;
        }

        // The next `make_input` scales the final partial step, so the burn
        // is complete only once the tracked impulse has actually been spent.
        if (vec_length(dv_remaining_) <= 1.0e-9) {
            state_ = ExecutorState::Complete;
            dv_remaining_ = {};
            return;
        }
        if (after.fuel <= 0.0) {
            state_ = ExecutorState::Incomplete;
            return;
        }
    }

    if (after.landed && active()) {
        state_ = ExecutorState::Aborted;
    }
}

}  // namespace lander
