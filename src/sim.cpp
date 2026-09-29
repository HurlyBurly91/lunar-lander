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

double radial_distance(const State& state, const Vec2& bpos) {
    return std::hypot(state.x - bpos.x, state.y - bpos.y);
}

double local_up_angle(const State& state, const Vec2& bpos) {
    const double rx = state.x - bpos.x;
    const double ry = state.y - bpos.y;
    if (std::hypot(rx, ry) < 1.0e-9) {
        return 0.0;
    }
    return std::atan2(ry, rx) - 0.5 * kPi;
}

double local_attitude_angle(const State& state, const Vec2& bpos) {
    const double diff = state.angle - local_up_angle(state, bpos);
    return std::atan2(std::sin(diff), std::cos(diff));
}

LocalVelocity local_velocity(const State& state, const Vec2& bpos,
                             const Vec2& bvel) {
    LocalVelocity out{};
    const double rx = state.x - bpos.x;
    const double ry = state.y - bpos.y;
    if (std::hypot(rx, ry) < 1.0e-9) {
        return out;
    }
    const double theta = std::atan2(ry, rx);
    const double up_x = std::cos(theta);
    const double up_y = std::sin(theta);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);
    out.radial = (state.vx - bvel.x) * up_x + (state.vy - bvel.y) * up_y;
    out.tangential =
        (state.vx - bvel.x) * right_x + (state.vy - bvel.y) * right_y;
    return out;
}

double surface_radius_at(const Terrain& terrain, const State& state,
                         const Vec2& bpos) {
    const double theta = std::atan2(state.y - bpos.y, state.x - bpos.x);
    return terrain.surface_radius_at_arc(terrain.arc_at_angle(theta));
}

double altitude_at(const Terrain& terrain, const State& state,
                    const Vec2& bpos) {
    return radial_distance(state, bpos) -
           surface_radius_at(terrain, state, bpos);
}

double local_angular_velocity(const State& state, const Vec2& bpos,
                               const Vec2& bvel) {
    const LocalVelocity lv = local_velocity(state, bpos, bvel);
    const double r = radial_distance(state, bpos);
    if (r < 1.0e-9) {
        return 0.0;
    }
    return lv.tangential / r;
}

State attached_state(const BinarySystem& system, int body_index,
                     double landed_arc, double t) {
    const Body& body = system.body(body_index);
    const Vec2 pos = system.position(body_index, t);
    const Vec2 vel = system.velocity(body_index, t);
    const double theta = body.terrain.angle_at_arc(landed_arc);
    const double r = body.terrain.surface_radius_at_arc(landed_arc);
    State out{};
    out.x = pos.x + std::cos(theta) * r;
    out.y = pos.y + std::sin(theta) * r;
    out.vx = vel.x;
    out.vy = vel.y;
    out.angle = normalize_angle(theta - 0.5 * kPi);
    out.fuel = 0.0;
    out.landed = true;
    out.landed_body = body_index;
    out.landed_arc = landed_arc;
    return out;
}

State interpolated_state(const State& previous, const State& current,
                         double alpha, bool snap_to_current) {
    if (snap_to_current || alpha <= 0.0 || alpha >= 1.0) {
        return current;
    }

    State out = current;

    // Presentation-only linear blend of the global (inertial) coordinates.
    // A straight chord between consecutive authoritative positions is used
    // instead of the M04 moon-centred polar form because the reference frame
    // now moves: with two bodies the barycentric origin is no longer a good
    // polar anchor, and per-step arcs are tiny compared with the body
    // radii, so the chord error is negligible.
    out.x = previous.x + (current.x - previous.x) * alpha;
    out.y = previous.y + (current.y - previous.y) * alpha;

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

Simulation::Simulation() { reset(0); }

Simulation::Simulation(const Config& config) : config_(config) { reset(0); }

void Simulation::reset(std::uint64_t seed) {
    seed_ = seed;
    accumulator_ = 0.0;
    sim_time_ = 0.0;
    contracts_completed_ = 0;
    reference_body_ = 0;
    binary_ = BinarySystem::canonical(config_.mu, seed_,
                                      companion_seed(seed_));
    contract_ = Contract{};
    contract_.origin_body = 0;
    contract_.destination_body = 1;
    contract_.reward = 100 * binary_.body(1).terrain.pads().front().multiplier;
    last_completed_.reset();

    state_ = State{};
    state_.landed = true;
    state_.landed_body = 0;
    state_.landed_arc = 0.0;
    state_.fuel = config_.fuel;
    attach_to_body();
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
    const Body& body = binary_.body(reference_body_);
    const Vec2 bpos = binary_.position(reference_body_, sim_time_);
    const double r = radial_distance(state_, bpos);
    if (r < 1.0e-9) {
        return;
    }
    const double theta = std::atan2(state_.y - bpos.y, state_.x - bpos.x);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);
    const Vec2 bvel = binary_.velocity(reference_body_, sim_time_);
    const LocalVelocity lv = local_velocity(state_, bpos, bvel);
    const double speed = std::sqrt(body.mu / r);
    const double direction =
        std::abs(lv.tangential) > 1.0e-6
            ? (lv.tangential < 0.0 ? -1.0 : 1.0)
            : 1.0;
    // Body-relative circular velocity plus the body's own ephemeris
    // velocity: a one-time state change, with no continuing stabilization.
    state_.vx = bvel.x + right_x * speed * direction;
    state_.vy = bvel.y + right_y * speed * direction;
}

void Simulation::refuel() {
    if (state_.crashed) {
        return;
    }
    state_.fuel = config_.fuel;
}

void Simulation::advance(double elapsed, const Input& input) {
    if (state_.crashed) {
        return;
    }

    const double dt = config_.fixed_dt;
    if (dt <= 0.0) {
        return;
    }

    accumulator_ = std::min(accumulator_ + elapsed, 10.0);
    while (accumulator_ >= dt) {
        accumulator_ -= dt;
        step_fixed(input);
        if (state_.crashed) {
            break;
        }
    }
}

void Simulation::step_fixed(const Input& input) {
    previous_ = state_;
    const double dt = config_.fixed_dt;
    const double t0 = sim_time_;
    sim_time_ += dt;
    state_.ticks += 1;

    if (state_.landed) {
        if (try_takeoff(input, t0)) {
            integrate_flight(input, t0);
            return;
        }
        // Still on the ground: re-attach to the moving body and hold the
        // ship on its local surface point. System time keeps advancing.
        attach_to_body();
        update_reference_body();
        return;
    }

    integrate_flight(input, t0);
}

void Simulation::integrate_flight(const Input& input, double t0) {
    const double dt = config_.fixed_dt;
    const double throttle = clamp01(input.main_throttle);
    const bool main_active = throttle > 0.0;

    // Both bodies' fields are always active; the ephemeris is sampled at
    // the start of this fixed step, matching the state being integrated.
    Vec2 a = binary_.gravity(Vec2{state_.x, state_.y}, t0);

    if (main_active) {
        const double tx = -std::sin(state_.angle);
        const double ty = std::cos(state_.angle);
        a.x += config_.main_accel * throttle * tx;
        a.y += config_.main_accel * throttle * ty;
    }

    state_.omega +=
        (input.rotate_left ? -config_.rotate_accel : 0.0) * dt +
        (input.rotate_right ? config_.rotate_accel : 0.0) * dt;

    state_.vx += a.x * dt;
    state_.vy += a.y * dt;
    state_.x += state_.vx * dt;
    state_.y += state_.vy * dt;
    state_.angle = normalize_angle(state_.angle + state_.omega * dt);

    if (main_active) {
        state_.fuel = std::max(0.0, state_.fuel - config_.fuel_burn * dt);
    }

    resolve_ground_contact();
    update_reference_body();
}

void Simulation::attach_to_body() {
    const int i = state_.landed_body;
    const Body& body = binary_.body(i);
    const Vec2 pos = binary_.position(i, sim_time_);
    const Vec2 vel = binary_.velocity(i, sim_time_);
    const double theta = body.terrain.angle_at_arc(state_.landed_arc);
    const double r = body.terrain.surface_radius_at_arc(state_.landed_arc);
    state_.x = pos.x + std::cos(theta) * r;
    state_.y = pos.y + std::sin(theta) * r;
    state_.vx = vel.x;
    state_.vy = vel.y;
    state_.omega = 0.0;
    state_.angle = normalize_angle(theta - 0.5 * kPi);
}

bool Simulation::try_takeoff(const Input& input, double t0) {
    const double throttle = clamp01(input.main_throttle);
    if (throttle <= 0.0 || state_.fuel <= 0.0) {
        return false;
    }

    const int i = state_.landed_body;
    const Body& body = binary_.body(i);
    const Vec2 ship{state_.x, state_.y};
    const double theta = body.terrain.angle_at_arc(state_.landed_arc);
    const double up_x = std::cos(theta);
    const double up_y = std::sin(theta);

    // Effective downward acceleration in the body's (non-inertial) frame:
    // the total gravitational field at the ship minus the body's own
    // barycentric acceleration, projected on the local vertical. A
    // commanded thrust that does not exceed it cannot lift the ship, so the
    // ground keeps holding: no jitter, no fuel use, no attitude change.
    const Vec2 g = binary_.gravity(ship, t0);
    const Vec2 bacc = binary_.acceleration(i, t0);
    const double g_down =
        -((g.x - bacc.x) * up_x + (g.y - bacc.y) * up_y);
    if (config_.main_accel * throttle <= g_down) {
        return false;
    }

    state_.landed = false;
    // Release at this step's start: position stays on the current pad, and
    // the ship inherits the body's current global translational velocity.
    const Vec2 bvel = binary_.velocity(i, t0);
    state_.vx = bvel.x;
    state_.vy = bvel.y;
    state_.omega = 0.0;
    return true;
}

void Simulation::resolve_ground_contact() {
    if (state_.landed || state_.crashed) {
        return;
    }

    for (int i = 0; i < 2; ++i) {
        const Body& body = binary_.body(i);
        const Vec2 bpos = binary_.position(i, sim_time_);
        const double rx = state_.x - bpos.x;
        const double ry = state_.y - bpos.y;
        const double rho = std::hypot(rx, ry);
        if (rho < 1.0e-9) {
            state_.crashed = true;
            state_.crash_body = i;
            state_.vx = 0.0;
            state_.vy = 0.0;
            state_.omega = 0.0;
            return;
        }

        const double theta = std::atan2(ry, rx);
        const double arc = body.terrain.arc_at_angle(theta);
        const double surface = body.terrain.surface_radius_at_arc(arc);
        if (rho > surface) {
            continue;
        }

        // In contact with body i: evaluate everything body-relative.
        const Vec2 bvel = binary_.velocity(i, sim_time_);
        const LocalVelocity lv = local_velocity(state_, bpos, bvel);
        const double up_angle = theta - 0.5 * kPi;
        const Pad* pad = body.terrain.pad_at_arc(arc);
        bool safe = false;
        if (pad) {
            const bool radial_ok =
                std::abs(lv.radial) <= config_.safe_vertical_speed;
            const bool tangential_ok =
                std::abs(lv.tangential) <= config_.safe_horizontal_speed;
            const bool angle_ok =
                std::abs(std::atan2(std::sin(state_.angle - up_angle),
                                    std::cos(state_.angle - up_angle))) <=
                config_.safe_angle_rad;
            safe = radial_ok && tangential_ok && angle_ok;
        }

        if (!safe) {
            state_.crashed = true;
            state_.crash_body = i;
            state_.x = bpos.x + std::cos(theta) * surface;
            state_.y = bpos.y + std::sin(theta) * surface;
            state_.vx = 0.0;
            state_.vy = 0.0;
            state_.omega = 0.0;
            return;
        }

        state_.landed = true;
        state_.landed_body = i;
        state_.landed_arc = arc;
        state_.score += 100 * pad->multiplier;
        if (i == contract_.destination_body && pad->center_arc == 0.0) {
            last_completed_ = contract_;
            last_completed_->completed = true;
            state_.score += contract_.reward;
            contracts_completed_ += 1;
            contract_.origin_body = i;
            contract_.destination_body = 1 - i;
            contract_.reward =
                100 * binary_.body(contract_.destination_body)
                          .terrain.pads().front().multiplier;
        }
        state_.x = bpos.x + std::cos(theta) * surface;
        state_.y = bpos.y + std::sin(theta) * surface;
        state_.vx = bvel.x;
        state_.vy = bvel.y;
        state_.omega = 0.0;
        state_.angle = normalize_angle(up_angle);
        return;
    }
}

void Simulation::update_reference_body() {
    if (state_.landed) {
        reference_body_ = state_.landed_body;
        return;
    }

    double d[2];
    for (int i = 0; i < 2; ++i) {
        const Vec2 bpos = binary_.position(i, sim_time_);
        d[i] = std::hypot(state_.x - bpos.x, state_.y - bpos.y);
    }
    // Deterministic hysteresis: only switch to the other body once it is
    // clearly (20% or more) closer, preventing flicker near the crossover.
    const int other = 1 - reference_body_;
    if (d[other] < 0.8 * d[reference_body_]) {
        reference_body_ = other;
    }
}

}  // namespace lander
