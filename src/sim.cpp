#include "lander/sim.hpp"

#include <algorithm>

namespace lander {

namespace {

double normalize_angle(double angle) {
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle;
}

double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

}  // namespace

bool operator==(const Input& lhs, const Input& rhs) {
    return lhs.rotate_left == rhs.rotate_left &&
           lhs.rotate_right == rhs.rotate_right &&
           lhs.main_throttle == rhs.main_throttle;
}

bool operator==(const Config& lhs, const Config& rhs) {
    return lhs.fixed_dt == rhs.fixed_dt &&
           lhs.mu == rhs.mu &&
           lhs.main_accel == rhs.main_accel &&
           lhs.rotate_accel == rhs.rotate_accel &&
           lhs.fuel == rhs.fuel &&
           lhs.fuel_burn == rhs.fuel_burn &&
           lhs.safe_vertical_speed == rhs.safe_vertical_speed &&
           lhs.safe_horizontal_speed == rhs.safe_horizontal_speed &&
           lhs.safe_angle_rad == rhs.safe_angle_rad;
}

double radial_distance(const State& state) {
    return std::hypot(state.x, state.y);
}

double local_up_angle(const State& state) {
    double rho = radial_distance(state);
    if (rho < 1.0e-9) {
        return 0.0;
    }
    return std::atan2(state.y, state.x) - 0.5 * kPi;
}

double local_attitude_angle(const State& state) {
    double diff = state.angle - local_up_angle(state);
    return std::atan2(std::sin(diff), std::cos(diff));
}

LocalVelocity local_velocity(const State& state) {
    LocalVelocity out{};
    double rho = radial_distance(state);
    if (rho < 1.0e-9) {
        return out;
    }
    double theta = std::atan2(state.y, state.x);
    double up_x = std::cos(theta);
    double up_y = std::sin(theta);
    double right_x = std::sin(theta);
    double right_y = -std::cos(theta);
    out.radial = state.vx * up_x + state.vy * up_y;
    out.tangential = state.vx * right_x + state.vy * right_y;
    return out;
}

double surface_radius_at(const Terrain& terrain, const State& state) {
    return terrain.surface_radius_at_angle(std::atan2(state.y, state.x));
}

double altitude_at(const Terrain& terrain, const State& state) {
    return radial_distance(state) - surface_radius_at(terrain, state);
}

State interpolated_state(const State& previous, const State& current,
                         double alpha, bool snap_to_current) {
    if (snap_to_current || alpha <= 0.0) {
        return current;
    }
    if (alpha >= 1.0) {
        return current;
    }

    State out = current;

    // Interpolate the moon-centred polar coordinates instead of taking a
    // straight chord between two inertial positions. Both are presentation
    // only, but the polar form keeps orbital angle progression monotonic at
    // render cadences that do not divide the 120 Hz physics cadence evenly.
    const double previous_radius = radial_distance(previous);
    const double current_radius = radial_distance(current);
    if (previous_radius < 1.0e-9 || current_radius < 1.0e-9) {
        out.x = previous.x + (current.x - previous.x) * alpha;
        out.y = previous.y + (current.y - previous.y) * alpha;
    } else {
        const double previous_theta = std::atan2(previous.y, previous.x);
        const double current_theta = std::atan2(current.y, current.x);
        const double theta_delta = std::atan2(
            std::sin(current_theta - previous_theta),
            std::cos(current_theta - previous_theta));
        const double theta = previous_theta + theta_delta * alpha;
        const double radius =
            previous_radius + (current_radius - previous_radius) * alpha;
        out.x = std::cos(theta) * radius;
        out.y = std::sin(theta) * radius;
    }

    const double angle_delta =
        std::atan2(std::sin(current.angle - previous.angle),
                   std::cos(current.angle - previous.angle));
    out.angle = normalize_angle(previous.angle + angle_delta * alpha);
    return out;
}

double flame_flick(double t) {
    // t is the continuous presentation clock in seconds. Two incommensurate
    // sine terms give a smooth, lively flicker. The frequencies stay well
    // below the 60 Hz sampling rate (the earlier tick-derived 24.8 Hz would
    // alias at 60 Hz) so the flame reads as a smooth animation, not a
    // staircase.
    const double a = kTwoPi * 7.0 * t;
    const double b = kTwoPi * 11.0 * t + 1.0;
    return 0.5 + 0.5 * std::sin(a) + 0.3 * std::sin(b);
}

double flame_length(double thrust_level, double t) {
    // Preserves the original magnitude mapping (full throttle -> original
    // flame extent, low throttle -> short puff) and the ~[0.43, 1.87] length
    // range, but drives the variation from continuous time instead of the
    // integer tick counter.
    const double full_len = 0.7 + 0.9 * flame_flick(t);
    return thrust_level * full_len;
}

Simulation::Simulation(const Config& config) : config_(config) {}

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;
    state_ = State{};
    terrain_ = Terrain(seed_);

    double r = terrain_.surface_radius_at_arc(0.0) + 20.0;
    state_.x = 0.0;
    state_.y = r;
    state_.angle = 0.0;
    state_.fuel = config_.fuel;
    previous_ = state_;
}

void Simulation::set_state(const State& state) {
    State next = state;
    next.angle = normalize_angle(next.angle);
    next.fuel = std::max(0.0, next.fuel);
    state_ = next;
    previous_ = next;
}

void Simulation::circularize() {
    if (state_.landed || state_.crashed) {
        return;
    }
    const double r = radial_distance(state_);
    if (r < 1.0e-9) {
        return;
    }
    const double theta = std::atan2(state_.y, state_.x);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);
    const LocalVelocity lv = local_velocity(state_);
    const double speed = std::sqrt(config_.mu / r);
    const double direction =
        std::abs(lv.tangential) > 1.0e-6
            ? (lv.tangential < 0.0 ? -1.0 : 1.0)
            : 1.0;
    state_.vx = right_x * speed * direction;
    state_.vy = right_y * speed * direction;
}

void Simulation::refuel() {
    if (state_.landed || state_.crashed) {
        return;
    }
    state_.fuel = config_.fuel;
}

void Simulation::advance(double elapsed, const Input& input) {
    if (state_.landed || state_.crashed) {
        return;
    }

    double dt = config_.fixed_dt;
    if (dt <= 0.0) {
        return;
    }

    accumulator_ = std::min(accumulator_ + elapsed, 10.0);
    while (accumulator_ >= dt) {
        accumulator_ -= dt;
        step_fixed(input);
        if (state_.landed || state_.crashed) {
            break;
        }
    }
}

void Simulation::step_fixed(const Input& input) {
    previous_ = state_;
    const double dt = config_.fixed_dt;
    double throttle = clamp01(input.main_throttle);
    bool main_active = throttle > 0.0;

    double ax = 0.0;
    double ay = 0.0;

    double rho = radial_distance(state_);
    if (rho > 1.0e-9) {
        double inv_r3 = 1.0 / (rho * rho * rho);
        ax = -config_.mu * state_.x * inv_r3;
        ay = -config_.mu * state_.y * inv_r3;
    }

    if (main_active) {
        double tx = -std::sin(state_.angle);
        double ty = std::cos(state_.angle);
        ax += config_.main_accel * throttle * tx;
        ay += config_.main_accel * throttle * ty;
    }

    state_.omega +=
        (input.rotate_left ? -config_.rotate_accel : 0.0) * dt +
        (input.rotate_right ? config_.rotate_accel : 0.0) * dt;

    state_.vx += ax * dt;
    state_.vy += ay * dt;
    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;
    state_.angle = normalize_angle(state_.angle + state_.omega * dt);

    if (main_active) {
        state_.fuel = std::max(0.0, state_.fuel - config_.fuel_burn * dt);
    }

    state_.ticks += 1;
    resolve_ground_contact();
}

void Simulation::resolve_ground_contact() {
    if (state_.landed || state_.crashed) {
        return;
    }

    double rho = radial_distance(state_);
    if (rho < 1.0e-9) {
        state_.crashed = true;
        state_.vx = 0.0;
        state_.vy = 0.0;
        state_.omega = 0.0;
        return;
    }

    double theta = std::atan2(state_.y, state_.x);
    double arc = Terrain::arc_at_angle(theta);
    double surface = terrain_.surface_radius_at_arc(arc);
    if (rho > surface) {
        return;
    }

    state_.x = std::cos(theta) * surface;
    state_.y = std::sin(theta) * surface;

    const Pad* pad = terrain_.pad_at_arc(arc);
    LocalVelocity lv = local_velocity(state_);
    double up_angle = theta - 0.5 * kPi;
    bool safe = false;
    if (pad) {
        bool radial_ok = std::abs(lv.radial) <= config_.safe_vertical_speed;
        bool tangential_ok = std::abs(lv.tangential) <= config_.safe_horizontal_speed;
        bool angle_ok =
            std::abs(std::atan2(std::sin(state_.angle - up_angle),
                                std::cos(state_.angle - up_angle))) <=
            config_.safe_angle_rad;
        safe = radial_ok && tangential_ok && angle_ok;
    }

    if (!safe) {
        state_.crashed = true;
        state_.vx = 0.0;
        state_.vy = 0.0;
        state_.omega = 0.0;
        return;
    }

    state_.landed = true;
    state_.score = 100 * pad->multiplier;
    state_.vx = 0.0;
    state_.vy = 0.0;
    state_.omega = 0.0;
    state_.angle = normalize_angle(up_angle);
}

}  // namespace lander
