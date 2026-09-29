#include "lander/sim.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

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

// M05-R3-16: the transfer solver's clearance shell above a surface.
constexpr double kTransferClearance = 15.0;

// M05-R3-16: how far outside the actual terrain the transfer arc must
// stay at each step, relative to the collision test the simulation itself
// runs. The game re-checks terrain one fixed step after the solver's
// sample, and the bodies rotate in between, so the margin keeps the
// solver's guarantee valid against the game's test.
constexpr double kTransferTerrainMargin = 1.0;

// M05-R3-16: propagate (p, v) forward by exactly `steps` fixed steps of
// size `dt` with the same semi-implicit Euler scheme and the same
// two-body inverse-square field as Simulation::integrate_flight (gravity
// sampled at each step's start time), without thrust, spin, fuel, or
// collision. Pure: the inputs are unchanged.
Vec2 propagate(const BinarySystem& bin, const Vec2& p0, const Vec2& v0,
               double t0, int steps, double dt) {
    Vec2 p = p0;
    Vec2 v = v0;
    double t = t0;
    for (int i = 0; i < steps; ++i) {
        const Vec2 a = bin.gravity(p, t);
        v = v + a * dt;
        p = p + v * dt;
        t += dt;
    }
    return p;
}

// M05-R3-16: whether the unthrust arc from (p0, v0) stays outside both
// bodies' actual terrain at every fixed step. This mirrors the collision
// test the simulation applies each step (tidal-lock rotation included),
// with a margin. Pure: no state is mutated.
bool transfer_arc_clear(const BinarySystem& bin, const Vec2& p0,
                        const Vec2& v0, double t0, int steps, double dt,
                         int source) {
    Vec2 p = p0;
    Vec2 v = v0;
    double t = t0;
    for (int i = 0; i < steps; ++i) {
        const Vec2 a = bin.gravity(p, t);
        v = v + a * dt;
        p = p + v * dt;
        t += dt;
        for (int b = 0; b < 2; ++b) {
            const Vec2 bp = bin.position(b, t);
            const double rx = p.x - bp.x;
            const double ry = p.y - bp.y;
            const double rho = std::hypot(rx, ry);
            if (rho < 1.0e-9) {
                return false;
            }
            const double theta = std::atan2(ry, rx);
            const Terrain& ter = bin.body(b).terrain;
            const double surface =
                ter.surface_radius_at_arc(
                    ter.arc_at_angle(theta - bin.body_rotation(t)));
            if (b == source) {
                // The craft departs from this body's exact surface point (no
                // clearance shell, M05-R3-20). The game's contact test crashes
                // on any below-surface reading, so the departure must be
                // outward enough to stay above the local terrain at every step
                // from the first one on.
                if (rho < surface) {
                    if (getenv("LL_TRANSFER_DEBUG")) {
                        std::fprintf(stderr, "  arc-clear REJECT src b=%d i=%d rho=%.3f surf=%.3f\n", b, i, rho, surface);
                    }
                    return false;
                }
            } else {
                // The other body: keep the conservative clearance margin so
                // the approach never grazes its terrain.
                if (rho - surface < kTransferTerrainMargin) {
                    if (getenv("LL_TRANSFER_DEBUG")) {
                        std::fprintf(stderr, "  arc-clear REJECT oth b=%d i=%d rho=%.3f surf=%.3f margin=%.3f\n", b, i, rho, surface, rho - surface);
                    }
                    return false;
                }
            }
        }
    }
    return true;
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

void Simulation::sync_orbit() {
    if (state_.crashed) {
        return;
    }
    const int source = state_.landed ? state_.landed_body : reference_body_;
    const int other = 1 - source;
    const double t = sim_time_;
    const Vec2 spos = binary_.position(source, t);
    const Vec2 svel = binary_.velocity(source, t);
    const Vec2 opos = binary_.position(other, t);
    const double away = std::hypot(spos.x - opos.x, spos.y - opos.y);
    if (away < 1.0e-9) {
        return;
    }
    // Body-synchronous circular orbit: the ship's offset from the source
    // rotates with the binary's angular velocity, so the ship co-rotates
    // with the binary and holds a fixed direction in the rotating frame.
    // Its relative radial velocity is exactly zero. The initial attitude
    // points radially out, away from the source (the far side of the
    // binary). Like circularize, this is one instantaneous state change;
    // no continuing stabilization or autopilot.
    const double omega = binary_.omega();
    const double r = std::cbrt(binary_.body(source).mu / (omega * omega));
    const Vec2 dir{(spos.x - opos.x) / away, (spos.y - opos.y) / away};
    State next{};
    next.x = spos.x + dir.x * r;
    next.y = spos.y + dir.y * r;
    next.vx = svel.x - omega * (next.y - spos.y);
    next.vy = svel.y + omega * (next.x - spos.x);
    next.fuel = state_.fuel;
    next.angle = normalize_angle(std::atan2(dir.y, dir.x));
    next.omega = 0.0;
    next.ticks = state_.ticks;
    next.score = state_.score;
    next.landed = false;
    next.landed_body = -1;
    set_state(next);
}

bool Simulation::transfer() {
    if (state_.crashed) {
        return false;
    }
    const int source = state_.landed ? state_.landed_body : reference_body_;
    const int target = 1 - source;
    const double t0 = sim_time_;
    const double dt = config_.fixed_dt;
    const Body& tgt = binary_.body(target);

    // M05-R3-20: the craft departs from its CURRENT world position, which is
    // left bit-identical by this initializer. For a landed craft that is the
    // exact surface point (which already carries the body's translational +
    // rotational surface velocity); for a flying craft it is the current
    // flight position. Only the velocity is replaced (no teleport).
    const Vec2 x0{state_.x, state_.y};

    // M05-R3-19: staged deterministic solver with fixed work bounds,
    // independent of player input and of the wall clock. The binary is
    // analytic, so a coarse integration step locates the most promising
    // terminal-miss basin, a medium step polishes each basin (polar scan +
    // damped Newton), and a final pass at the authoritative fixed step
    // re-polishes the best candidates and validates the miss, the speed
    // bound, and terrain clearance before any state is changed.
    constexpr double kFlightFractions[] = {0.15, 0.20, 0.25, 0.30, 0.40,
                                           0.50};
    constexpr double kCoarseDt = 0.5;
    // Medium integration step for the refine stage: coarse enough to keep each
    // propagation cheap, fine enough that the converged velocity lands within
    // ~a metre of the authoritative minimum, which the final full-step Newton
    // then polishes. Using the full 1/120 step here would make the refine (the
    // most propagations) dominate the solve time.
    constexpr double kMediumDt = 0.1;
    constexpr int kCoarseSpeeds = 13;     // 10..34 m/s by 2
    constexpr double kCoarseSpeedMin = 10.0;
    constexpr double kCoarseSpeedStep = 2.0;
    constexpr int kCoarseDirs = 24;       // 15-degree steps
    constexpr int kTopBasins = 3;
    constexpr double kCoarseAccept = 300.0;
    constexpr int kRefineSpeedRadius = 2;  // +/-2 m/s by 1
    constexpr int kRefineAngRadius = 3;    // +/-15 degrees by 5
    constexpr int kNewtonMax = 5;
    constexpr int kFinalNewtonMax = 4;
    constexpr double kNewtonStep = 0.25;
    constexpr double kMaxSpeed = 60.0;
    // Arrival-shell proximity tolerance (m). Re-tuned upward from 5.0 because
    // the velocity-only arc now originates at the surface (0 clearance) rather
    // than the old 15 m departure shell, which shifts the outward basin's
    // minimum to ~5.8 m. The hard safety gate remains transfer_arc_clear's
    // 1 m terrain margin over every step, so a looser proximity is safe: any
    // arc that clips terrain is rejected there regardless of this bound.
    constexpr double kAcceptMiss = 8.0;

    struct Candidate {
        double miss{};
        double speed{};
        double fraction{};
        Vec2 v0{};
    };
    Candidate best[2]{};
    for (int k = 0; k < 2; ++k) {
        best[k].miss = 1.0e30;
    }
    auto consider = [&](const Candidate& c) {
        for (int k = 0; k < 2; ++k) {
            if (c.miss < best[k].miss ||
                (c.miss == best[k].miss && c.speed < best[k].speed) ||
                (c.miss == best[k].miss && c.speed == best[k].speed &&
                 c.fraction < best[k].fraction)) {
                for (int j = 1; j > k; --j) {
                    best[j] = best[j - 1];
                }
                best[k] = c;
                return;
            }
        }
    };
    // Arrival shell for a candidate flight time: a small clearance shell
    // above the target's surface on the side the departure is coming from,
    // so the final approach stays outside the terrain.
    auto goal_at = [&](double t1, Vec2& goal_out) {
        const Vec2 s1 = binary_.position(source, t1);
        const Vec2 g1 = binary_.position(target, t1);
        const double dg = std::hypot(s1.x - g1.x, s1.y - g1.y);
        if (dg < 1.0e-9) {
            return false;
        }
        const Vec2 approach{(s1.x - g1.x) / dg, (s1.y - g1.y) / dg};
        const double r_arr =
            tgt.terrain.max_surface_radius() + kTransferClearance;
        goal_out = {g1.x + approach.x * r_arr, g1.y + approach.y * r_arr};
        return true;
    };
    // M05-R3-20: penalty (per metre) for a candidate's FIRST-step dip back
    // into the departure body's terrain, added to the Newton's terminal
    // objective. The game's contact test crashes on any below-surface reading,
    // so a velocity that reaches the target but initially falls into the body
    // is a crash; this steers the polish toward an outward departure. It is a
    // cheap single-step objective hint only; transfer_arc_clear is the
    // authoritative full-arc clearance gate (it re-checks every step).
    constexpr double kDivePenalty = 100.0;
    auto first_step_dive = [&](const Vec2& v) {
        const Vec2 a0 = binary_.gravity(x0, t0);
        const Vec2 v1 = v + a0 * dt;
        const Vec2 p1 = x0 + v1 * dt;
        const double t1s = t0 + dt;
        const Vec2 sp = binary_.position(source, t1s);
        const double rx = p1.x - sp.x;
        const double ry = p1.y - sp.y;
        const double rho1 = std::hypot(rx, ry);
        if (rho1 < 1.0e-9) {
            return 1.0e30;
        }
        const double theta = std::atan2(ry, rx);
        const Terrain& ter = binary_.body(source).terrain;
        const double surface = ter.surface_radius_at_arc(
            ter.arc_at_angle(theta - binary_.body_rotation(t1s)));
        return std::max(0.0, surface - rho1);
    };

    for (double fraction : kFlightFractions) {
        const int steps =
            static_cast<int>(std::lround(fraction * binary_.period() / dt));
        if (steps < 10) {
            continue;
        }
        const double t1 = t0 + steps * dt;
        Vec2 x_goal{};
        if (!goal_at(t1, x_goal)) {
            continue;
        }
        // Medium-step count for the refine stage (see kMediumDt).
        const int msteps =
            std::max(8, static_cast<int>(std::lround((t1 - t0) / kMediumDt)));

        // Coarse stage: a coarse integration step makes each propagation
        // cheap; the terminal miss is used only to locate the basins.
        const int csteps =
            std::max(8, static_cast<int>(std::lround((t1 - t0) / kCoarseDt)));
        auto miss_c = [&](const Vec2& v) {
            const Vec2 f = propagate(binary_, x0, v, t0, csteps, kCoarseDt);
            const double m = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
            return std::isfinite(m) ? m : 1.0e30;
        };
        struct Coarse {
            double miss{};
            double speed{};
            double ang{};
        };
        Coarse top[kTopBasins]{};
        for (int k = 0; k < kTopBasins; ++k) {
            top[k].miss = 1.0e30;
        }
        for (int si = 0; si < kCoarseSpeeds; ++si) {
            const double speed = kCoarseSpeedMin + kCoarseSpeedStep * si;
            for (int di = 0; di < kCoarseDirs; ++di) {
                const double ang = kTwoPi * di / kCoarseDirs;
                const double m =
                    miss_c(Vec2{speed * std::cos(ang), speed * std::sin(ang)});
                for (int k = 0; k < kTopBasins; ++k) {
                    if (m < top[k].miss) {
                        for (int j = kTopBasins - 1; j > k; --j) {
                            top[j] = top[j - 1];
                        }
                        top[k] = {m, speed, ang};
                        break;
                    }
                }
            }
        }
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "[T] frac=%.2f coarse_top=%.3f\n", fraction,
                         top[0].miss);
        }
        if (top[0].miss > kCoarseAccept) {
            continue;  // no basin at this flight time
        }

        // Refine stage: polish each promising basin (polar scan + damped
        // Newton) at the authoritative fixed step, so the converged
        // velocity is already valid against the simulation's own
        // integration. Only the top basins per flight time are refined,
        // which bounds the work.
        for (int k = 0; k < kTopBasins; ++k) {
            if (top[k].miss > 1000.0) {
                continue;  // no refine window can reach such a candidate
            }
            double m = top[k].miss;
            Vec2 v0{top[k].speed * std::cos(top[k].ang),
                    top[k].speed * std::sin(top[k].ang)};
            auto miss_m = [&](const Vec2& v) {
                const Vec2 f =
                    propagate(binary_, x0, v, t0, msteps, kMediumDt);
                double mm = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
                mm += kDivePenalty * first_step_dive(v);
                return std::isfinite(mm) ? mm : 1.0e30;
            };
            for (int si = -kRefineSpeedRadius; si <= kRefineSpeedRadius;
                 ++si) {
                const double speed = top[k].speed + 1.0 * si;
                if (speed < 1.0e-9) {
                    continue;
                }
                for (int ai = -kRefineAngRadius; ai <= kRefineAngRadius;
                     ++ai) {
                    const double ang = top[k].ang + kTwoPi * ai / 36.0;
                    const Vec2 v{speed * std::cos(ang), speed * std::sin(ang)};
                    const double mm = miss_m(v);
                    if (mm < m) {
                        m = mm;
                        v0 = v;
                    }
                }
            }
            // Damped Newton polish on the terminal miss: central-difference
            // 2x2 Jacobian, with the step halved whenever it fails to
            // improve the miss.
            for (int iter = 0; iter < kNewtonMax && m >= kAcceptMiss;
                  ++iter) {
                const Vec2 fp0 =
                    propagate(binary_, x0, v0 - Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fm0 =
                    propagate(binary_, x0, v0 + Vec2{kNewtonStep, 0}, t0,
                              msteps, kMediumDt);
                const Vec2 fp1 =
                    propagate(binary_, x0, v0 - Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const Vec2 fm1 =
                    propagate(binary_, x0, v0 + Vec2{0, kNewtonStep}, t0,
                              msteps, kMediumDt);
                const double j00 = (fp0.x - fm0.x) / (2.0 * kNewtonStep);
                const double j01 = (fp0.y - fm0.y) / (2.0 * kNewtonStep);
                const double j10 = (fp1.x - fm1.x) / (2.0 * kNewtonStep);
                const double j11 = (fp1.y - fm1.y) / (2.0 * kNewtonStep);
                const double det = j00 * j11 - j01 * j10;
                if (std::abs(det) < 1.0e-9) {
                    break;  // singular or degenerate Jacobian
                }
                const Vec2 f = propagate(binary_, x0, v0, t0, msteps, kMediumDt);
                const double fx = f.x - x_goal.x;
                const double fy = f.y - x_goal.y;
                double dvx = (-fx * j11 + fy * j01) / det;
                double dvy = (-fy * j00 + fx * j10) / det;
                bool improved = false;
                for (int damp = 0; damp < 6; ++damp) {
                    const Vec2 trial{v0.x + dvx, v0.y + dvy};
                    const double mt = miss_m(trial);
                    if (mt < m) {
                        v0 = trial;
                        m = mt;
                        improved = true;
                        break;
                    }
                    dvx *= 0.5;
                    dvy *= 0.5;
                }
                if (!improved) {
                    break;
                }
            }
            const double speed = std::hypot(v0.x, v0.y);
            if (getenv("LL_TRANSFER_DEBUG")) {
                const Vec2 fdbg = propagate(binary_, x0, v0, t0, steps, dt);
                const double tterm =
                    std::hypot(fdbg.x - x_goal.x, fdbg.y - x_goal.y);
                const Vec2 spc = binary_.position(source, t0);
                double ux = x0.x - spc.x, uy = x0.y - spc.y;
                const double ul = std::hypot(ux, uy);
                ux /= ul;
                uy /= ul;
                std::fprintf(stderr,
                             "  [refine] frac=%.2f speed=%.3f term=%.3f "
                             "dive=%.4f radial=%.3f m_pen=%.3f\n",
                             fraction, speed, tterm, first_step_dive(v0),
                             v0.x * ux + v0.y * uy, m);
            }
            if (m >= kAcceptMiss || speed < 1.0e-9 || speed > kMaxSpeed) {
                continue;  // not plausible: reject, never teleport
            }
            consider(Candidate{m, speed, fraction, v0});
        }
    }

    if (best[0].miss >= 1.0e30) {
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr,
                         "[T] src=%d tgt=%d done have_solution=0 (no basin)\n",
                         source, target);
        }
        return false;  // no plausible solution: the state is untouched
    }

    // Final pass at the authoritative fixed step: re-polish the best
    // candidate(s) and accept the first whose terminal miss, speed bound,
    // and terrain clearance all hold. This is the source of truth, so a
    // coarse/medium basin that does not survive it is discarded.
    Vec2 chosen_v0{};
    bool have_solution = false;
    for (int k = 0; k < 2 && !have_solution; ++k) {
        const Candidate& c = best[k];
        if (c.miss >= 1.0e30) {
            break;
        }
        const int steps =
            static_cast<int>(std::lround(c.fraction * binary_.period() / dt));
        const double t1 = t0 + steps * dt;
        Vec2 x_goal{};
        if (!goal_at(t1, x_goal)) {
            continue;
        }
        auto miss_f = [&](const Vec2& v) {
            const Vec2 f = propagate(binary_, x0, v, t0, steps, dt);
            double mm = std::hypot(f.x - x_goal.x, f.y - x_goal.y);
            mm += kDivePenalty * first_step_dive(v);
            return std::isfinite(mm) ? mm : 1.0e30;
        };
        // Cheap terminal-only miss, used to pre-filter escape-search candidates
        // before paying for a full terrain clearance check.
        auto term_f = [&](const Vec2& v) {
            const Vec2 f = propagate(binary_, x0, v, t0, steps, dt);
            return std::hypot(f.x - x_goal.x, f.y - x_goal.y);
        };
        Vec2 v0 = c.v0;
        double m = miss_f(v0);
        for (int iter = 0; iter < kFinalNewtonMax && m >= kAcceptMiss;
             ++iter) {
            const Vec2 fp0 =
                propagate(binary_, x0, v0 - Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fm0 =
                propagate(binary_, x0, v0 + Vec2{kNewtonStep, 0}, t0, steps, dt);
            const Vec2 fp1 =
                propagate(binary_, x0, v0 - Vec2{0, kNewtonStep}, t0, steps, dt);
            const Vec2 fm1 =
                propagate(binary_, x0, v0 + Vec2{0, kNewtonStep}, t0, steps, dt);
            const double j00 = (fp0.x - fm0.x) / (2.0 * kNewtonStep);
            const double j01 = (fp0.y - fm0.y) / (2.0 * kNewtonStep);
            const double j10 = (fp1.x - fm1.x) / (2.0 * kNewtonStep);
            const double j11 = (fp1.y - fm1.y) / (2.0 * kNewtonStep);
            const double det = j00 * j11 - j01 * j10;
            if (std::abs(det) < 1.0e-9) {
                break;  // singular or degenerate Jacobian
            }
            const Vec2 f = propagate(binary_, x0, v0, t0, steps, dt);
            const double fx = f.x - x_goal.x;
            const double fy = f.y - x_goal.y;
            double dvx = (-fx * j11 + fy * j01) / det;
            double dvy = (-fy * j00 + fx * j10) / det;
            bool improved = false;
            for (int damp = 0; damp < 6; ++damp) {
                const Vec2 trial{v0.x + dvx, v0.y + dvy};
                const double mt = miss_f(trial);
                if (mt < m) {
                    v0 = trial;
                    m = mt;
                    improved = true;
                    break;
                }
                dvx *= 0.5;
                dvy *= 0.5;
            }
            if (!improved) {
                break;
            }
        }
        const double speed = std::hypot(v0.x, v0.y);
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "[T] final frac=%.2f m=%.3f speed=%.3f\n",
                         c.fraction, m, speed);
        }
        if (m >= kAcceptMiss || speed < 1.0e-9 || speed > kMaxSpeed) {
            if (getenv("LL_TRANSFER_DEBUG")) {
                std::fprintf(stderr, "  reject: miss/speed\n");
            }
            continue;
        }
        // The arc must stay outside both bodies' actual terrain at every
        // flight step, by the same test the simulation applies (tidal
        // rotation included). This forces a landed craft to depart outward
        // enough to clear its own surface, and keeps the final approach from
        // grazing the target's terrain.
        if (transfer_arc_clear(binary_, x0, v0, t0, steps, dt, source)) {
            chosen_v0 = v0;
            have_solution = true;
            continue;
        }
        if (getenv("LL_TRANSFER_DEBUG")) {
            std::fprintf(stderr, "  arc not clear: escaping\n");
        }
        // The polished terminal-miss arc grazes terrain (typically the target's
        // surface on the final approach). Search a small neighbourhood for the
        // nearest velocity that both clears the terrain and still reaches the
        // arrival shell, so a shallow graze does not sink the whole transfer.
        const double a0 = std::atan2(v0.y, v0.x);
        bool escaped = false;
        for (int pass = 0; pass < 2 && !escaped; ++pass) {
            const double sf = (pass == 0 ? 0.03 : 0.07);  // speed span
            const double da = (pass == 0 ? 1.0 : 3.0) * kTwoPi / 360.0;  // deg
            for (int is = -1; is <= 1 && !escaped; ++is) {
                for (int ia = -1; ia <= 1; ++ia) {
                    const double s = speed * (1.0 + is * sf);
                    if (s < 1.0e-9 || s > kMaxSpeed) {
                        continue;
                    }
                    const double ang = a0 + ia * da;
                    const Vec2 v2{s * std::cos(ang), s * std::sin(ang)};
                    if (term_f(v2) >= kAcceptMiss) {
                        continue;  // no longer reaches the arrival shell
                    }
                    if (!transfer_arc_clear(binary_, x0, v2, t0, steps, dt,
                                            source)) {
                        continue;  // still grazes terrain
                    }
                    chosen_v0 = v2;
                    have_solution = true;
                    escaped = true;
                }
            }
        }
    }
    if (getenv("LL_TRANSFER_DEBUG")) {
        std::fprintf(stderr, "[T] src=%d tgt=%d done have_solution=%d\n",
                     source, target, (int)have_solution);
    }

    if (!have_solution) {
        return false;  // no plausible solution: the state is untouched
    }

    // One-shot activation: the craft's POSITION is left exactly as it was;
    // only its velocity is replaced by the solved ballistic velocity. It is
    // unlanded and non-crashed, with the nose along the initial velocity.
    // Fuel, score, and the phase clock are preserved; there is no autopilot
    // after the velocity change.
    State next{};
    next.x = state_.x;
    next.y = state_.y;
    next.vx = chosen_v0.x;
    next.vy = chosen_v0.y;
    next.fuel = state_.fuel;
    next.angle = normalize_angle(std::atan2(chosen_v0.y, chosen_v0.x));
    next.omega = 0.0;
    next.ticks = state_.ticks;
    next.score = state_.score;
    next.landed = false;
    next.landed_body = -1;
    set_state(next);
    return true;
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
