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
           lhs.main_throttle == rhs.main_throttle &&
           lhs.reaction_wheels == rhs.reaction_wheels;
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
    lhs.safe_angle_rad == rhs.safe_angle_rad &&
    lhs.reaction_wheel_accel == rhs.reaction_wheel_accel &&
    lhs.reaction_wheel_taper == rhs.reaction_wheel_taper;
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
                          const Vec2& bpos, double body_rotation) {
    const double theta = std::atan2(state.y - bpos.y, state.x - bpos.x);
    return terrain.surface_radius_at_arc(
        terrain.arc_at_angle(theta - body_rotation));
}

double altitude_at(const Terrain& terrain, const State& state,
                    const Vec2& bpos, double body_rotation) {
    return radial_distance(state, bpos) -
           surface_radius_at(terrain, state, bpos, body_rotation);
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

double target_range_rate(const Vec2& ship_pos, const Vec2& ship_vel,
                          const Vec2& target_pos, const Vec2& target_vel) {
    const double dx = ship_pos.x - target_pos.x;
    const double dy = ship_pos.y - target_pos.y;
    const double r = std::hypot(dx, dy);
    if (r < 1.0e-9) {
        return 0.0;
    }
    return ((ship_vel.x - target_vel.x) * dx +
            (ship_vel.y - target_vel.y) * dy) /
           r;
}

double orbital_rate(const State& state, const Vec2& bpos,
                     const Vec2& bvel) {
    return local_angular_velocity(state, bpos, bvel);
}

const char* range_rate_label(double range_rate, double tolerance) {
    if (range_rate <= -tolerance) {
        return "CLOSE";
    }
    if (range_rate >= tolerance) {
        return "OPEN";
    }
    return "HOLD";
}

NavCues navigation_cues(const State& state, const BinarySystem& system,
                          double t, int destination_body) {
    NavCues out{};
    const Vec2 ship_pos{state.x, state.y};
    const Vec2 ship_vel{state.vx, state.vy};
    const Body& destination = system.body(destination_body);
    const double pad_arc = destination.terrain.pads().front().center_arc;
    // The base pad is a point fixed on the rotating surface (M05-R3 tidal
    // locking): both its position and its velocity track the body's spin,
    // so the destination is a genuinely moving target.
    const BinarySystem::SurfacePoint dest_pad = system.surface_point(
        destination_body, destination.terrain.angle_at_arc(pad_arc),
        destination.terrain.surface_radius_at_arc(pad_arc), t);
    out.target_position = dest_pad.position;
    out.target_direction = out.target_position - ship_pos;
    out.target_distance =
        std::hypot(out.target_direction.x, out.target_direction.y);
    if (out.target_distance > 1.0e-9) {
        out.target_direction =
            out.target_direction * (1.0 / out.target_distance);
    } else {
        out.target_direction = {};
    }

    out.relative_velocity = ship_vel - dest_pad.velocity;
    out.relative_speed =
        std::hypot(out.relative_velocity.x, out.relative_velocity.y);

    out.gravity_primary = system.gravity_from(0, ship_pos, t);
    out.gravity_companion = system.gravity_from(1, ship_pos, t);
    out.net_gravity = out.gravity_primary + out.gravity_companion;
    out.g_primary = std::hypot(out.gravity_primary.x, out.gravity_primary.y);
    out.g_companion =
        std::hypot(out.gravity_companion.x, out.gravity_companion.y);
    out.g_net = std::hypot(out.net_gravity.x, out.net_gravity.y);
    return out;
}

namespace {

double gravitational_influence(double mu, double distance) {
    if (distance <= 1.0e-9) {
        return 1.0e300;
    }
    return mu / (distance * distance);
}

}  // namespace

int reference_body_for(double mu0, double mu1, double distance0,
                       double distance1, int current, double margin) {
    const bool current_is_primary = current != 1;
    const double i0 = gravitational_influence(mu0, distance0);
    const double i1 = gravitational_influence(mu1, distance1);
    if (current_is_primary) {
        return i1 > margin * i0 ? 1 : 0;
    }
    return i0 > margin * i1 ? 0 : 1;
}

State attached_state(const BinarySystem& system, int body_index,
                     double landed_arc, double t) {
    const Body& body = system.body(body_index);
    // The ship sits on a point fixed to the rotating surface: the surface
    // point's full position and velocity (translation plus spin).
    const BinarySystem::SurfacePoint sp = system.surface_point(
        body_index, body.terrain.angle_at_arc(landed_arc),
        body.terrain.surface_radius_at_arc(landed_arc), t);
    const double world_angle =
        body.terrain.angle_at_arc(landed_arc) + system.body_rotation(t);
    State out{};
    out.x = sp.position.x;
    out.y = sp.position.y;
    out.vx = sp.velocity.x;
    out.vy = sp.velocity.y;
    out.angle = normalize_angle(world_angle - 0.5 * kPi);
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

void Simulation::circularize(bool counter_clockwise) {
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
    const double speed = std::sqrt(body.mu / r);
    // The local "right" vector points clockwise around the body; reversing
    // it gives the counter-clockwise circular direction.
    const double direction = counter_clockwise ? -1.0 : 1.0;
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

    if (input.reaction_wheels) {
        // Finite angular damping opposite the current spin. It tapers to
        // zero as `omega` approaches zero and never teleports, translates,
        // or consumes fuel.
        const double taper = std::clamp(
            std::abs(state_.omega) / config_.reaction_wheel_taper, 0.0, 1.0);
        const double damping_accel = config_.reaction_wheel_accel * taper;
        state_.omega -= std::copysign(damping_accel * dt, state_.omega);
    }

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
    // The ship rides the rotating surface: full surface-point velocity
    // (translation plus spin) and nose along the local radial.
    const BinarySystem::SurfacePoint sp = binary_.surface_point(
        i, body.terrain.angle_at_arc(state_.landed_arc),
        body.terrain.surface_radius_at_arc(state_.landed_arc), sim_time_);
    const double world_angle =
        body.terrain.angle_at_arc(state_.landed_arc) +
        binary_.body_rotation(sim_time_);
    state_.x = sp.position.x;
    state_.y = sp.position.y;
    state_.vx = sp.velocity.x;
    state_.vy = sp.velocity.y;
    state_.omega = 0.0;
    state_.angle = normalize_angle(world_angle - 0.5 * kPi);
}

bool Simulation::try_takeoff(const Input& input, double t0) {
    const double throttle = clamp01(input.main_throttle);
    if (throttle <= 0.0 || state_.fuel <= 0.0) {
        return false;
    }

    const int i = state_.landed_body;
    const Body& body = binary_.body(i);
    const Vec2 ship{state_.x, state_.y};
    // The local vertical follows the rotating surface point (M05-R3).
    const double world_angle =
        body.terrain.angle_at_arc(state_.landed_arc) +
        binary_.body_rotation(t0);
    const double up_x = std::cos(world_angle);
    const double up_y = std::sin(world_angle);

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
    // the ship inherits the full velocity of the surface point it left
    // (translational plus the spin of that point).
    const double r =
        body.terrain.surface_radius_at_arc(state_.landed_arc);
    const BinarySystem::SurfacePoint sp =
        binary_.surface_point(i,
                              body.terrain.angle_at_arc(state_.landed_arc),
                              r, t0);
    state_.vx = sp.velocity.x;
    state_.vy = sp.velocity.y;
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
        // The terrain lives in body-local coordinates: subtract the body's
        // tidal-lock spin to get the arc under the ship.
        const double arc = body.terrain.arc_at_angle(theta -
                                                      binary_.body_rotation(
                                                          sim_time_));
        const double surface = body.terrain.surface_radius_at_arc(arc);
        if (rho > surface) {
            continue;
        }

        // In contact with body i: evaluate everything body-relative. The
        // surface point the ship touched is rotating (M05-R3), so the
        // relative velocity is against that point's full inertial velocity:
        // the body's translational velocity plus the spin velocity of the
        // contact offset.
        const Vec2 bvel = binary_.velocity(i, sim_time_);
        const double ox = std::cos(theta) * surface;
        const double oy = std::sin(theta) * surface;
        const Vec2 sp_vel{bvel.x - binary_.omega() * oy,
                          bvel.y + binary_.omega() * ox};
        const LocalVelocity lv = local_velocity(state_, bpos, sp_vel);
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
        // Landed: the ship is placed on the rotating surface point and
        // inherits its full velocity (translation plus spin).
        state_.x = bpos.x + std::cos(theta) * surface;
        state_.y = bpos.y + std::sin(theta) * surface;
        state_.vx = sp_vel.x;
        state_.vy = sp_vel.y;
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

    const Vec2 p0 = binary_.position(0, sim_time_);
    const Vec2 p1 = binary_.position(1, sim_time_);
    const double d0 = std::hypot(state_.x - p0.x, state_.y - p0.y);
    const double d1 = std::hypot(state_.x - p1.x, state_.y - p1.y);
    reference_body_ = reference_body_for(binary_.body(0).mu,
                                         binary_.body(1).mu, d0, d1,
                                         reference_body_);
}

}  // namespace lander
