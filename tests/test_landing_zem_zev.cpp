// M06-R6: low-complexity powered-landing guidance. Verified headlessly:
//   V01  the R6 zero-effort source (an O(1) lookup into the maintained M06-R3
//        Coast/zero-thrust rolling predictor) agrees with a cold zero-thrust
//        propagation from the same state.
//   V02  ZEM/ZEV is zero when the zero-effort state equals the terminal state.
//   V03  the acceleration command points along a pure position error.
//   V04  the acceleration command points against a pure velocity error.
//   V05  the command is finite as t_go approaches the configured minimum.
//   V06  the bounded t_go scan rejects infeasible (over-thrust) candidates.
//   V07  the hover waypoint (and pad) is a moving/rotating surface point.
// V02-V07 are pure-math (no simulation stepping, no predictor); V01 uses the
// R3 predictor but does not step the live simulation. No SDL anywhere.
#include "lander/ballistic.hpp"
#include "lander/binary.hpp"
#include "lander/landing.hpp"
#include "lander/predictor.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {

using lander::Config;
using lander::LandingConfig;
using lander::LandingPhase;
using lander::LandingTargetState;
using lander::Vec2;

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

void check_close(double a, double b, double eps, const char* message) {
    if (std::abs(a - b) > eps) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b, eps);
    }
}

void check_close_vec(lander::Vec2 a, lander::Vec2 b, double eps,
                     const char* message) {
    if (std::hypot(a.x - b.x, a.y - b.y) > eps) {
        ++failures;
        std::printf("FAIL: %s ((%.9g,%.9g) != (%.9g,%.9g), eps %.3g)\n", message,
                    a.x, a.y, b.x, b.y, eps);
    }
}

void check_finite(const lander::Vec2& v, const char* message) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
        ++failures;
        std::printf("FAIL: %s (non-finite (%.9g,%.9g))\n", message, v.x, v.y);
    }
}

// A flying simulation used only for its analytic binary ephemeris (body
// positions / velocities / rotation) and terrain; the tests never step it.
lander::Simulation make_test_sim(std::uint64_t seed) {
    lander::Simulation sim(Config{});
    sim.reset(seed);
    return sim;
}

double pad_angle(const lander::BinarySystem& bin, int body) {
    return bin.body(body).terrain.angle_at_arc(0.0);
}
double pad_radius(const lander::BinarySystem& bin, int body) {
    return bin.body(body).terrain.surface_radius_at_arc(0.0);
}

// A stable near-circular orbit around the primary, high enough to coast through
// the test horizon without touching terrain (no live stepping of this sim).
lander::Simulation make_flying_sim(std::uint64_t seed) {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(seed);
    const lander::BinarySystem& bin = sim.binary();
    const double r = sim.terrain().max_surface_radius() + 900.0;
    const lander::Vec2 p0 = bin.position(0, 0.0);
    const lander::Vec2 v0 = bin.velocity(0, 0.0);
    const double speed = std::sqrt(cfg.mu / r);
    lander::State s{};
    s.x = p0.x;
    s.y = p0.y + r;
    s.vx = v0.x + speed;
    s.vy = v0.y;
    s.angle = 0.0;
    s.fuel = 1000.0;
    sim.set_state(s);
    return sim;
}

// The R6 zero-effort source: an O(1) index into the maintained Coast predictor
// ring. samples_[0] is the origin; each later sample is one fixed step ahead.
// When the requested terminal time lies beyond the predictor's cached horizon
// or in the terminal (crashed/landed) tail, continue the zero-thrust ballistic
// extrapolation from the last non-terminal cached sample using the shared
// pure ballistic propagator. This keeps the within-horizon lookup O(1) while
// still giving the guidance a well-defined mathematical coast for long,
// hover-like t_go candidates whose physical coast would have impacted.
lander::BallisticState zero_from_predictor(
    const lander::BinarySystem& bin,
    const lander::RecedingHorizonPredictor& pred, double t_go, double dt) {
    const auto& s = pred.samples();
    int req = (int)std::lround(t_go / dt);
    if (req < 0) req = 0;
    const int idx = std::min(req, (int)s.size() - 1);
    const auto& sample = s[idx];
    if (req < (int)s.size() && !sample.state.crashed &&
        !sample.state.landed) {
        return {{sample.state.x, sample.state.y},
                {sample.state.vx, sample.state.vy}, sample.time};
    }
    int last = idx;
    while (last > 0 && (s[last].state.crashed || s[last].state.landed)) {
        --last;
    }
    const lander::BallisticState start{{s[last].state.x, s[last].state.y},
                                       {s[last].state.vx, s[last].state.vy},
                                       s[last].time};
    const int extra = req - last;
    return propagate_ballistic(bin, start, extra, dt);
}

}  // namespace

// V01: the maintained Coast (zero-thrust) rolling predictor supplies the
// zero-effort state; an O(1) lookup into it must agree with an independent
// cold zero-thrust propagation from the same state (R6-03).
void test_cached_zero_effort_matches_cold() {
    lander::Simulation seed = make_flying_sim(7);
    const Config& cfg = seed.config();
    const double dt = cfg.fixed_dt;
    const lander::BinarySystem& bin = seed.binary();
    const lander::State& s0 = seed.state();

    lander::RecedingHorizonPredictor pred;
    pred.configure(720, 1e-9);  // 6 s horizon, well past every t_go tested
    lander::FlightPolicy coast{};
    coast.kind = lander::PredictionKind::Coast;
    lander::NodeExecutor executor{};
    // One call builds the whole horizon (large budget) so the ring is fully
    // populated; no live simulation step is performed.
    pred.cold_rebuild(seed, coast, executor, 1000000);

    for (double t_go : {1.0, 2.0, 3.5, 5.0}) {
        const lander::BallisticState cached =
            zero_from_predictor(bin, pred, t_go, dt);
        lander::BallisticState origin{{s0.x, s0.y}, {s0.vx, s0.vy},
                                      seed.sim_time()};
        const int steps = (int)std::lround(t_go / dt);
        const lander::BallisticState cold =
            lander::propagate_ballistic(bin, origin, steps, dt);
        check_close_vec(cached.p, cold.p, 1e-6, "V01 cached position == cold");
        check_close_vec(cached.v, cold.v, 1e-6, "V01 cached velocity == cold");
        check_close(cached.t, cold.t, 1e-9, "V01 cached time == cold");
    }
}

// V02: zero error => zero command.
void test_zem_zev_zero_at_terminal() {
    const Vec2 cmd =
        lander::landing_acceleration_command({0.0, 0.0}, {0.0, 0.0}, 5.0);
    check_close_vec(cmd, {0.0, 0.0}, 1e-12, "V02 zero-error command is zero");
}

// V03: pure position error => command along the error (toward the target).
void test_command_direction_pure_position() {
    const Vec2 cmd =
        lander::landing_acceleration_command({100.0, 0.0}, {0.0, 0.0}, 10.0);
    check_close(cmd.x, 6.0, 1e-9, "V03 magnitude 6*ZEM/t^2");
    check_close(cmd.y, 0.0, 1e-9, "V03 y is zero");
    check(cmd.x > 0.0, "V03 command points along the position error");
}

// V04: pure velocity error => command against the error (braking).
void test_command_direction_pure_velocity() {
    const Vec2 cmd =
        lander::landing_acceleration_command({0.0, 0.0}, {10.0, 0.0}, 10.0);
    check_close(cmd.x, -2.0, 1e-9, "V04 magnitude -2*ZEV/t");
    check_close(cmd.y, 0.0, 1e-9, "V04 y is zero");
    check(cmd.x < 0.0, "V04 command points against the velocity error");
}

// V05: the command stays finite across the configured t_go range (including the
// minimum), for representative errors.
void test_command_finite_near_min_t_go() {
    LandingConfig landing{};
    const Vec2 zem{50.0, 30.0};
    const Vec2 zev{10.0, -5.0};
    for (double t = landing.t_go_min; t <= landing.t_go_max + 1e-9; t += 1.0) {
        const Vec2 cmd = lander::landing_acceleration_command(zem, zev, t);
        check_finite(cmd, "V05 finite across the t_go range");
    }
}

// V06: the bounded scan rejects over-thrust candidates and returns no t_go when
// none is feasible, but selects one for a moderate error.
void test_infeasible_t_go_rejected() {
    const lander::Simulation sim = make_test_sim(11);
    const lander::BinarySystem& bin = sim.binary();
    const Config& cfg = sim.config();
    LandingConfig landing{};

    // Far-off zero-effort at every candidate t_go => a_req exceeds main_accel
    // for all of them => no feasible t_go.
    lander::ZeroEffortQuery too_far = [&](double t_go) {
        const auto sp =
            bin.surface_point(0, pad_angle(bin, 0), pad_radius(bin, 0), t_go);
        lander::BallisticState z{};
        z.p = {sp.position.x + 100000.0, sp.position.y};
        z.v = {0.0, 0.0};
        z.t = t_go;
        return z;
    };
    const auto none = lander::landing_select_time_to_go(
        bin, cfg, landing, 0.0, landing.t_go_min, 0, LandingPhase::Descent,
        too_far);
    check(!none.has_value(), "V06 all-infeasible scan returns no t_go");

    // Moderate velocity error: near t_go_min the required acceleration exceeds
    // main_accel, but longer candidates are feasible and their initial command
    // is outward, so the scan must select one.
    lander::ZeroEffortQuery moderate = [&](double t_go) {
        const auto sp =
            bin.surface_point(0, pad_angle(bin, 0), pad_radius(bin, 0), t_go);
        lander::BallisticState z{};
        z.p = {sp.position.x, sp.position.y};
        z.v = {sp.velocity.x + 10.0, sp.velocity.y};
        z.t = t_go;
        return z;
    };
    const auto some = lander::landing_select_time_to_go(
        bin, cfg, landing, 0.0, landing.t_go_min, 0, LandingPhase::Descent,
        moderate);
    check(some.has_value(), "V06 a moderate error selects a feasible t_go");
    if (some) {
        check(*some >= landing.t_go_min - 1e-9 && *some <= landing.t_go_max + 1e-9,
              "V06 selected t_go is within the configured bounds");
    }
}

// V07: the APPROACH hover waypoint and the DESCENT pad are moving/rotating
// surface points: they match BinarySystem::surface_point at each future time
// and actually move as the body spins.
void test_hover_waypoint_moves_with_body() {
    const lander::Simulation sim = make_test_sim(11);
    const lander::BinarySystem& bin = sim.binary();
    LandingConfig landing{};
    const double pa = pad_angle(bin, 0);
    const double pr = pad_radius(bin, 0);
    const double hr = pr + landing.hover_height;

    for (double t : {0.0, 10.0, 37.0, 90.0, 200.0}) {
        const LandingTargetState hover =
            lander::landing_target_state(bin, 0, LandingPhase::Approach, landing,
                                         0.0, t);
        const auto hs = bin.surface_point(0, pa, hr, t);
        check_close_vec(hover.position, hs.position, 1e-9,
                        "V07 hover position matches surface_point");
        check_close_vec(hover.velocity, hs.velocity, 1e-9,
                        "V07 hover velocity matches surface_point");

        const LandingTargetState pad =
            lander::landing_target_state(bin, 0, LandingPhase::Descent, landing,
                                         0.0, t);
        const auto ps = bin.surface_point(0, pa, pr, t);
        check_close_vec(pad.position, ps.position, 1e-9,
                        "V07 pad position matches surface_point");
        check_close_vec(pad.velocity, ps.velocity, 1e-9,
                        "V07 pad velocity matches surface_point");
    }

    // The waypoint must actually move over time (tidal-lock spin rotates the
    // pad), not stay pinned to a fixed world point.
    const LandingTargetState a =
        lander::landing_target_state(bin, 0, LandingPhase::Approach, landing, 0.0,
                                     0.0);
    const LandingTargetState b =
        lander::landing_target_state(bin, 0, LandingPhase::Approach, landing, 0.0,
                                     200.0);
    const double moved = std::hypot(a.position.x - b.position.x,
                                    a.position.y - b.position.y);
    check(moved > 1e-3, "V07 hover waypoint moves as the body rotates");
}

// The composed guidance update returns a valid command whose throttle is the
// required acceleration scaled by the available thrust and clamped to [0,1].
void test_guidance_command_sane() {
    const lander::Simulation sim = make_test_sim(11);
    const lander::BinarySystem& bin = sim.binary();
    const Config& cfg = sim.config();
    LandingConfig landing{};

    lander::ZeroEffortQuery moderate = [&](double t_go) {
        const auto sp =
            bin.surface_point(0, pad_angle(bin, 0), pad_radius(bin, 0), t_go);
        lander::BallisticState z{};
        z.p = {sp.position.x + 8.0, sp.position.y};
        z.v = {sp.velocity.x, sp.velocity.y};
        z.t = t_go;
        return z;
    };
    const lander::LandingCommand cmd = lander::landing_guidance_command(
        bin, cfg, landing, 0.0, landing.t_go_min, 0, LandingPhase::Approach,
        moderate);
    check(cmd.valid, "sanity guidance command is valid");
    check(cmd.throttle >= 0.0 && cmd.throttle <= 1.0,
          "sanity throttle is clamped to [0,1]");
    const double a_req = std::hypot(cmd.acceleration.x, cmd.acceleration.y);
    check_close(cmd.throttle, a_req / cfg.main_accel, 1e-9,
                "sanity throttle equals a_req/main_accel");
    (void)cmd.t_go;
}

// Place a ship directly above the base pad of `body` at `altitude`, co-rotating
// with the surface point plus an optional radial velocity (negative = falling).
// The nose starts pointing radially out (away from the surface).
lander::State state_above_pad(const lander::Simulation& sim, int body,
                              double altitude, double radial_v) {
    const lander::BinarySystem& bin = sim.binary();
    const lander::Body& b = bin.body(body);
    const double t = sim.sim_time();
    const lander::Vec2 pos = bin.position(body, t);
    const lander::Vec2 vel = bin.velocity(body, t);
    const double theta = b.terrain.angle_at_arc(0.0);
    const double r = b.terrain.surface_radius_at_arc(0.0) + altitude;
    const lander::Vec2 up{std::cos(theta), std::sin(theta)};
    const double ox = r * up.x, oy = r * up.y;
    const double spin_x = -bin.omega() * oy, spin_y = bin.omega() * ox;
    lander::State s{};
    s.x = pos.x + ox;
    s.y = pos.y + oy;
    s.vx = vel.x + spin_x + radial_v * up.x;
    s.vy = vel.y + spin_y + radial_v * up.y;
    s.angle = theta - 0.5 * lander::kPi;  // nose radially out
    s.fuel = 1000.0;
    return s;
}

// Headless autoland run: drives the landing autopilot against the authoritative
// Simulation at the fixed step rate, maintaining a R3 Coast predictor as the
// zero-effort source, until the ship lands, crashes, or the tick budget elapses.
struct LandingRunResult {
    bool landed{false};
    bool crashed{false};
    int ticks{0};
    int landed_body{-1};
    double landed_arc{0.0};       // raw terrain arc (m, possibly near 2*pi*R).
    double landed_arc_norm{0.0};  // normalized to [0, circumference).
    bool on_target_pad{false};
    double final_altitude{0.0};  // m above the target surface point (radial).
    double final_radial_v{0.0};  // m/s, + is outward.
    double final_tang_v{0.0};    // m/s, body-relative tangential.
    // Seated (post-landing) contact frame: measured against the *actual*
    // terrain point under the ship (its real arc, tidal-spin-corrected) -- the
    // frame the authoritative Simulation contact check uses, unlike the fixed
    // arc-0 reference used by the [V11] verbose trace. Note that on landing the
    // sim snaps the ship onto that surface point and hands it the point's full
    // velocity, so radial/tang/angle read as exactly 0 here (co-moving with the
    // surface). They confirm the ship is seated on the pad, not the impact
    // margins; those were already enforced by the sim to declare `landed`
    // (radial <= safe_vertical, tangential <= safe_horizontal, angle
    // <= safe_angle) rather than `crashed`.
    double contact_altitude{0.0};  // m above the terrain point beneath the ship.
    double contact_radial_v{0.0};  // m/s, co-moving snap (0 by construction).
    double contact_tang_v{0.0};    // m/s, co-moving snap (0 by construction).
    double contact_angle_err{0.0}; // rad, co-moving snap (0 by construction).
    lander::LandingPhase phase{lander::LandingPhase::Approach};
    // V08: the lowest radial clearance (m) of any pre-landing flying state above
    // the target body's local terrain surface. `clipped` is set when the ship's
    // centre dipped more than 1 m below that surface (a terrain clip / chord
    // through the moon) before the final contact.
    bool clipped{false};
    double min_clearance{1e300};
    // V10: the last pre-landing (flying) state's inertial velocity and the
    // target pad's *moving* surface-point inertial velocity at that same instant
    // (the co-moving pair the guidance converges toward), plus the ship's
    // body-relative radial/tangential velocity at that instant.
    double ship_vel_x{0.0}, ship_vel_y{0.0};
    double pad_vel_x{0.0}, pad_vel_y{0.0};
    double pre_contact_radial_v{0.0};
    double pre_contact_tang_v{0.0};
};

 LandingRunResult run_autoland(std::uint64_t seed, int target_body, int start_body,
                               double start_altitude, double start_radial_v,
                               const lander::LandingConfig& lcfg, int max_ticks,
                               bool verbose = false) {
      lander::Config cfg{};
      lander::LandingConfig lc = lcfg;
      lander::Simulation sim(cfg);
     sim.reset(seed);
     sim.set_state(state_above_pad(sim, start_body, start_altitude,
                                   start_radial_v));

     // R3 Coast (zero-thrust) predictor, maintained at the live rate, is the
     // zero-effort source the guidance queries in O(1) (R6-03, R6-09).
     const int horizon =
         (int)std::lround(lc.t_go_max / cfg.fixed_dt) + 50;
     lander::RecedingHorizonPredictor coast;
     coast.configure(horizon, 1e-9);
     lander::FlightPolicy coast_policy{};
     coast_policy.kind = lander::PredictionKind::Coast;
     lander::NodeExecutor coast_exec{};
     coast.cold_rebuild(sim, coast_policy, coast_exec, horizon);

     lander::LandingAutopilot ap;
     ap.arm(target_body, lc,
            [&](double t_go) {
                return zero_from_predictor(sim.binary(), coast, t_go,
                                           cfg.fixed_dt);
            });

    auto rel = [&](const lander::Simulation& s) {
        const lander::BinarySystem& bin = s.binary();
        const lander::Body& b = bin.body(target_body);
        const double t = s.sim_time();
        const lander::Vec2 pos = bin.position(target_body, t);
        const lander::Vec2 vel = bin.velocity(target_body, t);
        const double th = b.terrain.angle_at_arc(0.0);
        const double rp = b.terrain.surface_radius_at_arc(0.0);
        const lander::Vec2 up{std::cos(th), std::sin(th)};
        const lander::Vec2 rt{std::sin(th), -std::cos(th)};
        const double ox = rp * up.x, oy = rp * up.y;
        const lander::Vec2 sp{-bin.omega() * oy, bin.omega() * ox};
        const lander::Vec2 rp2{s.state().x - (pos.x + ox),
                               s.state().y - (pos.y + oy)};
        const lander::Vec2 rv{s.state().vx - (vel.x + sp.x),
                              s.state().vy - (vel.y + sp.y)};
        struct Q { double alt, rad, tan; };
        return Q{rp2.x * up.x + rp2.y * up.y, rv.x * up.x + rv.y * up.y,
                 rv.x * rt.x + rv.y * rt.y};
    };

    const double pad_a = pad_angle(sim.binary(), target_body);
    const double pad_r = pad_radius(sim.binary(), target_body);
    LandingRunResult res{};
    for (int i = 0; i < max_ticks; ++i) {
        const lander::State before = sim.state();
        // V08: radial clearance of this flying state above the target body's
        // local terrain surface; the ship's centre must never dip below it.
        {
            const lander::Body& tb = sim.binary().body(target_body);
            const double tt = sim.sim_time();
            const lander::Vec2 bpos = sim.binary().position(target_body, tt);
            const double dist = std::hypot(before.x - bpos.x, before.y - bpos.y);
            const double theta = std::atan2(before.y - bpos.y, before.x - bpos.x);
            const double arc = tb.terrain.arc_at_angle(
                theta - sim.binary().body_rotation(tt));
            const double clearance = dist - tb.terrain.surface_radius_at_arc(arc);
            if (clearance < res.min_clearance) res.min_clearance = clearance;
            if (clearance < -1.0) res.clipped = true;
        }
        // V10: remember the last flying state's inertial velocity and the moving
        // target pad's inertial velocity at that instant (overwritten each step;
        // the surviving value is the pre-contact pair once the run lands).
        {
            res.ship_vel_x = before.vx;
            res.ship_vel_y = before.vy;
            const lander::Vec2 padv =
                sim.binary().surface_point(target_body, pad_a, pad_r,
                                           sim.sim_time())
                    .velocity;
            res.pad_vel_x = padv.x;
            res.pad_vel_y = padv.y;
            const lander::Vec2 relv{before.vx - padv.x, before.vy - padv.y};
            const lander::Vec2 up{std::cos(pad_a), std::sin(pad_a)};
            const lander::Vec2 rt{std::sin(pad_a), -std::cos(pad_a)};
            res.pre_contact_radial_v = relv.x * up.x + relv.y * up.y;
            res.pre_contact_tang_v = relv.x * rt.x + relv.y * rt.y;
        }
        const lander::Input in =
            ap.make_input(sim.state(), cfg, sim.sim_time());
        sim.step_once(in);
        res.ticks = i + 1;
        if (verbose && (i % 60 == 0)) {
            const auto q = rel(sim);
            std::printf("  [V11] t=%6.2f phase=%-9s alt=%7.2f rv=%7.3f "
                        "tv=%7.3f thr=%5.2f\n",
                        sim.sim_time(), lander::landing_phase_name(ap.phase()),
                        q.alt, q.rad, q.tan, in.main_throttle);
        }
        if (!sim.state().crashed && !sim.state().landed) {
            coast.advance(sim, sim.state(), coast_policy, coast_exec, horizon);
        }
        ap.after_step(before, sim.state(), sim.binary(), cfg, sim.sim_time());
        if (sim.state().landed) {
            res.landed = true;
            break;
        }
        if (sim.state().crashed) {
            res.crashed = true;
            break;
        }
    }
    res.phase = ap.phase();
    res.landed_body = sim.state().landed ? sim.state().landed_body : -1;
    res.landed_arc = sim.state().landed ? sim.state().landed_arc : 0.0;
    res.on_target_pad = res.landed_body == target_body &&
                        sim.binary()
                            .body(target_body)
                            .terrain.pad_at_arc(res.landed_arc) != nullptr;
    const auto q = rel(sim);
    res.final_altitude = q.alt;
    res.final_radial_v = q.rad;
    res.final_tang_v = q.tan;
    // True contact-frame margins, measured against the *actual* terrain point
    // beneath the ship (its real arc, tidal-rotation-corrected, velocity vs the
    // rotating contact point) -- the exact frame the authoritative Simulation
    // contact check uses to decide landed-vs-crashed.
    {
        const lander::Body& b = sim.binary().body(target_body);
        const double t = sim.sim_time();
        const lander::Vec2 bpos = sim.binary().position(target_body, t);
        const lander::Vec2 bvel = sim.binary().velocity(target_body, t);
        const double theta =
            std::atan2(sim.state().y - bpos.y, sim.state().x - bpos.x);
        const double arc =
            b.terrain.arc_at_angle(theta - sim.binary().body_rotation(t));
        const double surface = b.terrain.surface_radius_at_arc(arc);
        const double ox = std::cos(theta) * surface;
        const double oy = std::sin(theta) * surface;
        const lander::Vec2 sp_vel{bvel.x - sim.binary().omega() * oy,
                                  bvel.y + sim.binary().omega() * ox};
        const lander::LocalVelocity lv =
            lander::local_velocity(sim.state(), bpos, sp_vel);
        res.contact_altitude =
            lander::altitude_at(b.terrain, sim.state(), bpos,
                                sim.binary().body_rotation(t));
        res.contact_radial_v = lv.radial;
        res.contact_tang_v = lv.tangential;
        res.contact_angle_err = lander::local_attitude_angle(sim.state(), bpos);
        if (res.landed) {
            res.landed_arc_norm = b.terrain.normalize_arc(res.landed_arc);
        }
    }
    return res;
}

// Place a ship on the hover ring (pad radius + hover height), offset by
// `tangential` metres along the surface from the pad, co-rotating with the
// local surface point, with the nose initially off by `nose_error` radians.
// The range to the co-rotating hover waypoint is ~`tangential` (arc length), so
// a small offset lands inside the approach corridor and a large one outside.
lander::State state_near_hover(const lander::Simulation& sim, int body,
                               double hover_height, double tangential,
                               double nose_error) {
    const lander::BinarySystem& bin = sim.binary();
    const lander::Body& b = bin.body(body);
    const double t = sim.sim_time();
    const double pad_angle = b.terrain.angle_at_arc(0.0);
    const double rp = b.terrain.surface_radius_at_arc(0.0);
    const double rh = rp + hover_height;
    const double local_angle = pad_angle + tangential / rh;
    const auto sp = bin.surface_point(body, local_angle, rh, t);
    lander::State s{};
    s.x = sp.position.x;
    s.y = sp.position.y;
    s.vx = sp.velocity.x;
    s.vy = sp.velocity.y;
    s.angle = local_angle - 0.5 * lander::kPi + nose_error;  // nose out + error
    s.fuel = 1000.0;
    return s;
}

// Drives the autoland from an arbitrary start state and records how the phase
// machine uses the approach corridor: whether DESCENT is ever entered, at what
// range, and the maximum range observed while in DESCENT. This directly checks
// the R6-07 invariant that descent begins only inside the approach corridor.
struct CorridorProbe {
    bool entered_descent{false};
    double max_range_while_descent{0.0};
    double range_at_first_descent{0.0};
    int first_descent_tick{-1};
    lander::LandingPhase final_phase{lander::LandingPhase::Approach};
    bool landed{false};
    bool crashed{false};
};

CorridorProbe probe_descent_corridor(std::uint64_t seed, int target_body,
                                     const lander::State& start,
                                     const lander::LandingConfig& lcfg,
                                     int max_ticks) {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(seed);
    sim.set_state(start);

    const int horizon =
        (int)std::lround(lcfg.t_go_max / cfg.fixed_dt) + 50;
    lander::RecedingHorizonPredictor coast;
    coast.configure(horizon, 1e-9);
    lander::FlightPolicy coast_policy{};
    coast_policy.kind = lander::PredictionKind::Coast;
    lander::NodeExecutor coast_exec{};
    coast.cold_rebuild(sim, coast_policy, coast_exec, horizon);

    lander::LandingAutopilot ap;
    ap.arm(target_body, lcfg,
           [&](double t_go) {
               return zero_from_predictor(sim.binary(), coast, t_go,
                                          cfg.fixed_dt);
           });

    CorridorProbe p{};
    for (int i = 0; i < max_ticks; ++i) {
        const lander::State before = sim.state();
        const lander::Input in =
            ap.make_input(sim.state(), cfg, sim.sim_time());
        sim.step_once(in);
        if (!sim.state().crashed && !sim.state().landed) {
            coast.advance(sim, sim.state(), coast_policy, coast_exec, horizon);
        }
        ap.after_step(before, sim.state(), sim.binary(), cfg, sim.sim_time());
        if (sim.state().landed) {
            p.landed = true;
            p.final_phase = ap.phase();
            break;
        }
        if (sim.state().crashed) {
            p.crashed = true;
            p.final_phase = ap.phase();
            break;
        }
        const auto hover = lander::landing_target_state(
            sim.binary(), target_body, lander::LandingPhase::Approach, lcfg,
            sim.sim_time(), 0.0);
        const double range =
            std::hypot(hover.position.x - sim.state().x,
                       hover.position.y - sim.state().y);
        if (ap.phase() == lander::LandingPhase::Descent) {
            if (!p.entered_descent) {
                p.entered_descent = true;
                p.first_descent_tick = i;
                p.range_at_first_descent = range;
            }
            if (range > p.max_range_while_descent) p.max_range_while_descent = range;
        }
    }
    if (!p.landed && !p.crashed) p.final_phase = ap.phase();
    return p;
}

// V11: end-to-end target-pad autoland on the same body. The ship starts above
// the pad; the autopilot must bring it to a soft landing as declared by the
// authoritative Simulation landing checker (no direct state mutation).
void test_end_to_end_same_body_landing() {
    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 45.0;
    lc.hover_height = 30.0;
    lc.approach_radius = 40.0;
    lc.approach_speed = 8.0;
    // Start at the hover waypoint with a small initial downward drift to seed
    // the descent.
    const LandingRunResult r = run_autoland(7, 0, 0, 30.0, -1.0, lc, 8000, true);
    check(r.landed && r.on_target_pad,
          "V11 autoland reaches a soft landing on the target pad");
    check(!r.crashed, "V11 autoland does not crash");
    if (r.landed) {
        std::printf(
            "  [V11 seated] body=%d arc_norm=%.4f m ticks=%d contact_alt=%+.3f "
            "radial=%+.3f tang=%+.3f angle=%.3f rad (co-moving snap; impact "
            "margins were enforced by the sim to declare landed)\n",
            r.landed_body, r.landed_arc_norm, r.ticks, r.contact_altitude,
            r.contact_radial_v, r.contact_tang_v, r.contact_angle_err);
    }
    if (!r.landed || !r.on_target_pad || r.crashed) {
        std::printf(
            "  [V11 diag] landed=%d on_target_pad=%d crashed=%d ticks=%d "
            "phase=%s body=%d arc=%.3f alt=%.3f rv=%.3f tv=%.3f "
            "contact_alt=%.3f contact_rad=%.3f contact_tan=%.3f "
            "contact_ang=%.3f\n",
            (int)r.landed, (int)r.on_target_pad, (int)r.crashed, r.ticks,
            lander::landing_phase_name(r.phase), r.landed_body, r.landed_arc,
            r.final_altitude, r.final_radial_v, r.final_tang_v,
            r.contact_altitude, r.contact_radial_v, r.contact_tang_v,
            r.contact_angle_err);
    }
}

// V12: the autopilot's public API never mutates the spacecraft state; it only
// emits ordinary Input (const access throughout).
void test_no_state_mutation() {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(7);
    sim.set_state(state_above_pad(sim, 0, 60.0, 0.0));

    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 20.0;
    lander::LandingAutopilot ap;
    ap.arm(0, lc, [](double) { return lander::BallisticState{}; });

    const lander::State before = sim.state();
    const lander::Input in = ap.make_input(sim.state(), cfg, sim.sim_time());
    ap.after_step(before, sim.state(), sim.binary(), cfg, sim.sim_time());
    (void)in;
    check(before == sim.state(),
          "V12 autopilot API does not mutate the spacecraft state");
}

// V13: aborting a target-pad autoland leaves no throttle or attitude output.
void test_abort_no_output() {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(7);
    sim.set_state(state_above_pad(sim, 0, 60.0, 0.0));

    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 20.0;
    lander::LandingAutopilot ap;
    ap.arm(0, lc, [](double) { return lander::BallisticState{}; });
    ap.after_step(sim.state(), sim.state(), sim.binary(), cfg, sim.sim_time());
    ap.abort();
    const lander::Input in = ap.make_input(sim.state(), cfg, sim.sim_time());
    check(in.main_throttle == 0.0, "V13 aborted autoland: zero throttle");
    check(!in.rotate_left && !in.rotate_right,
          "V13 aborted autoland: no rotation output");
}

// V08: the approach/descent trajectory stays outside the target body's terrain
// for the whole run -- the ship's centre never dips below the local surface
// (no chord through the moon) -- until the final contact snaps it onto the pad.
void test_approach_stays_outside_terrain() {
    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 45.0;
    lc.hover_height = 30.0;
    lc.approach_radius = 40.0;
    lc.approach_speed = 8.0;
    const LandingRunResult r = run_autoland(7, 0, 0, 30.0, -1.0, lc, 8000);
    check(r.landed, "V08 autoland lands (the run is complete)");
    check(!r.clipped,
          "V08 the approach/descent trajectory never clips the target terrain");
    check(r.min_clearance > -1.0,
          "V08 the minimum radial terrain clearance stays above -1 m");
    std::printf("  [V08] min_clearance=%.4f m clipped=%d (landed=%d)\n",
                r.min_clearance, (int)r.clipped, (int)r.landed);
}

// V09: descent begins only inside the approach corridor. From a start well
// outside the corridor the phase must never be DESCENT while the range to the
// hover waypoint exceeds the corridor; from a start inside the corridor the
// phase does enter DESCENT, and only within the corridor.
void test_descent_only_inside_corridor() {
    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 45.0;
    lc.hover_height = 30.0;
    lc.approach_radius = 40.0;
    lc.approach_speed = 8.0;
    const double tol = 1.0;  // m, slack on the corridor boundary

    lander::Simulation sim(lander::Config{});
    sim.reset(7);

    // Outside the corridor: 150 m along the surface from the pad.
    const lander::State out_start =
        state_near_hover(sim, 0, lc.hover_height, 150.0, 0.4);
    const CorridorProbe out = probe_descent_corridor(7, 0, out_start, lc, 8000);
    check(out.max_range_while_descent <= lc.approach_radius + tol,
          "V09 outside-corridor start never drives DESCENT outside the corridor");

    // Inside the corridor: 15 m along the surface from the pad.
    const lander::State in_start =
        state_near_hover(sim, 0, lc.hover_height, 15.0, 0.4);
    const CorridorProbe in = probe_descent_corridor(7, 0, in_start, lc, 8000);
    check(in.entered_descent, "V09 inside-corridor start does enter DESCENT");
    check(in.range_at_first_descent <= lc.approach_radius + tol,
          "V09 descent begins only inside the approach corridor");

    std::printf("  [V09] OUT(150m): entered_descent=%d max_range_in_descent=%.2f m "
                "final=%s | IN(15m): entered_descent=%d first_range=%.2f m "
                "first_tick=%d final=%s\n",
                (int)out.entered_descent, out.max_range_while_descent,
                lander::landing_phase_name(out.final_phase),
                (int)in.entered_descent, in.range_at_first_descent,
                in.first_descent_tick, lander::landing_phase_name(in.final_phase));
}

// V10: the commanded touchdown velocity is the pad's *moving* surface-point
// velocity. The DESCENT target state matches BinarySystem::surface_point at the
// terminal time and moves as the body spins; and an end-to-end autoland's
// pre-contact inertial velocity matches that moving velocity within the
// soft-landing margins (a co-moving touchdown).
void test_touchdown_matches_moving_surface() {
    lander::LandingConfig lc{};
    const lander::Simulation sim = make_test_sim(11);
    const lander::BinarySystem& bin = sim.binary();
    const double pa = pad_angle(bin, 0);
    const double pr = pad_radius(bin, 0);

    lander::Vec2 prev{};
    bool first = true;
    bool moved = false;
    for (double t : {0.0, 10.0, 37.0, 90.0, 200.0}) {
        const lander::LandingTargetState target = lander::landing_target_state(
            bin, 0, lander::LandingPhase::Descent, lc, 0.0, t);
        const auto sp = bin.surface_point(0, pa, pr, t);
        check_close_vec(target.velocity, sp.velocity, 1e-9,
                        "V10 descent target velocity == moving surface-point velocity");
        check_close_vec(target.position, sp.position, 1e-9,
                        "V10 descent target position == moving surface-point position");
        if (first) {
            prev = target.velocity;
            first = false;
        } else if (std::hypot(target.velocity.x - prev.x,
                              target.velocity.y - prev.y) > 1e-6) {
            moved = true;
        }
    }
    check(moved,
          "V10 the commanded touchdown velocity actually moves (co-rotating pad)");

    const LandingRunResult r = run_autoland(7, 0, 0, 30.0, -1.0, lc, 8000);
    check(r.landed, "V10 autoland lands");
    if (r.landed) {
        const double dv = std::hypot(r.ship_vel_x - r.pad_vel_x,
                                     r.ship_vel_y - r.pad_vel_y);
        check(dv <= 3.0,
              "V10 pre-contact inertial velocity matches the moving pad velocity");
        check(std::fabs(r.pre_contact_radial_v) <= 2.5,
              "V10 pre-contact radial speed is within the safe vertical band");
        check(std::fabs(r.pre_contact_tang_v) <= 1.5,
              "V10 pre-contact tangential speed is within the safe horizontal band");
        std::printf("  [V10] pre-contact: dv=%.3f m/s (pad moving) radial=%.3f "
                    "tang=%.3f m/s (co-moving margins)\n",
                    dv, r.pre_contact_radial_v, r.pre_contact_tang_v);
    }
}

// V14: the target-pad autoland works for the primary and the companion, and for
// same-body and cross-body targets (a soft landing on the target pad, no crash).
void test_primary_companion_same_and_cross_body() {
    lander::LandingConfig lc{};
    lc.t_go_min = 2.0;
    lc.t_go_max = 45.0;
    lc.hover_height = 30.0;
    lc.approach_radius = 40.0;
    lc.approach_speed = 8.0;
    // Start the high-energy velocity kill from the deorbit entry altitude so
    // the gross cross-body tangential error is bled off gradually instead of
    // being reversed in one saturated deorbit burn.
    lc.brake_final_alt = 180.0;

    // A: same-body on the primary (body 0).
    const LandingRunResult a = run_autoland(7, 0, 0, 30.0, -1.0, lc, 8000);
    check(a.landed && a.on_target_pad && !a.crashed,
          "V14-A primary same-body: soft-lands on the target pad");
    // B: same-body on the companion (body 1).
    const LandingRunResult b = run_autoland(7, 1, 1, 30.0, -1.0, lc, 8000);
    check(b.landed && b.on_target_pad && !b.crashed,
          "V14-B companion same-body: soft-lands on the target pad");
    // C: cross-body: start above the primary's pad, target the companion's pad.
    const LandingRunResult c = run_autoland(7, 1, 0, 30.0, -1.0, lc, 14400);
    check(c.landed && c.on_target_pad && !c.crashed,
          "V14-C cross-body (primary -> companion): soft-lands on the target pad");

    std::printf(
        "  [V14] A(primary same): landed=%d on_pad=%d crash=%d ticks=%d "
        "minclear=%.2f m\n",
        (int)a.landed, (int)a.on_target_pad, (int)a.crashed, a.ticks,
        a.min_clearance);
    std::printf(
        "  [V14] B(companion same): landed=%d on_pad=%d crash=%d ticks=%d "
        "minclear=%.2f m\n",
        (int)b.landed, (int)b.on_target_pad, (int)b.crashed, b.ticks,
        b.min_clearance);
    std::printf(
        "  [V14] C(cross primary->companion): landed=%d on_pad=%d crash=%d "
        "ticks=%d phase=%s minclear=%.2f m\n",
        (int)c.landed, (int)c.on_target_pad, (int)c.crashed, c.ticks,
        lander::landing_phase_name(c.phase), c.min_clearance);

    // M06-R7: the cross-body landing is not knife-edge. A small perturbation
    // of the passing start velocity and an independent body/terrain seed both
    // still soft-land, so the low-energy handoff envelope carries margin.
    const LandingRunResult c_pert =
        run_autoland(7, 1, 0, 30.0, -0.8, lc, 14400);
    check(c_pert.landed && c_pert.on_target_pad && !c_pert.crashed,
          "V14-C perturbed start velocity: still soft-lands on the pad");
    const LandingRunResult c_seed =
        run_autoland(8, 1, 0, 30.0, -1.0, lc, 14400);
    check(c_seed.landed && c_seed.on_target_pad && !c_seed.crashed,
          "V14-C independent body/terrain seed: still soft-lands on the pad");
}

// V15: the guidance call is bounded in the predictor's cached horizon. The
// zero-effort source is an O(1) maintained lookup, so one guidance update issues
// a fixed, small number of zero-effort queries regardless of how long the Coast
// ring is -- the guidance-call work is therefore independent of horizon length.
void test_guidance_call_bounded_in_horizon() {
    const lander::Simulation sim = make_flying_sim(7);
    const Config& cfg = sim.config();
    const double dt = cfg.fixed_dt;
    const lander::BinarySystem& bin = sim.binary();
    lander::LandingConfig landing{};
    const int K = landing.t_go_candidates;

    auto count_queries = [&](int horizon) {
        lander::RecedingHorizonPredictor p;
        p.configure(horizon, 1e-9);
        lander::FlightPolicy coast{};
        coast.kind = lander::PredictionKind::Coast;
        lander::NodeExecutor ex{};
        p.cold_rebuild(sim, coast, ex, horizon);
        int n = 0;
        lander::ZeroEffortQuery q = [&](double t_go) {
            ++n;
            return zero_from_predictor(bin, p, t_go, dt);
        };
        lander::landing_guidance_command(bin, cfg, landing, sim.sim_time(), 10.0, 0,
                                         lander::LandingPhase::Approach, q);
        return n;
    };
    for (const int h : { (int)std::lround(4.0 / dt),
                         (int)std::lround(12.0 / dt),
                         (int)std::lround(landing.t_go_max / dt) + 50 }) {
        const int n = count_queries(h);
        check(n >= K && n <= K + 1,
              "V15 the guidance call's zero-effort queries stay bounded (K..K+1) "
              "regardless of the predictor horizon");
    }

    // Benchmark: per-call cost for a short ring vs a full-window ring. The work
    // is horizon-independent (a fixed K lookups), so the two should be close.
    auto time_per_call_us = [&](int horizon, int n) {
        lander::RecedingHorizonPredictor p;
        p.configure(horizon, 1e-9);
        lander::FlightPolicy coast{};
        coast.kind = lander::PredictionKind::Coast;
        lander::NodeExecutor ex{};
        p.cold_rebuild(sim, coast, ex, horizon);
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i) {
            lander::ZeroEffortQuery q = [&](double t_go) {
                return zero_from_predictor(bin, p, t_go, dt);
            };
            lander::landing_guidance_command(bin, cfg, landing, sim.sim_time(),
                                             10.0, 0,
                                             lander::LandingPhase::Approach, q);
        }
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(t1 - t0).count() / n * 1e6;
    };
    const double s_us = time_per_call_us((int)std::lround(12.0 / dt), 200);
    const double l_us = time_per_call_us((int)std::lround(landing.t_go_max / dt) + 50,
                                         200);
    std::printf(
        "  [V15] guidance-call cost: 12s-ring=%.3f us/call full-ring=%.3f us/call "
        "(horizon-independent; fixed K=%d zero-effort lookups)\n",
        s_us, l_us, K);
    check(l_us < 4.0 * s_us + 1e-6,
          "V15 the guidance-call cost stays bounded as the horizon grows");
}

int main() {
    test_cached_zero_effort_matches_cold();
    test_zem_zev_zero_at_terminal();
    test_command_direction_pure_position();
    test_command_direction_pure_velocity();
    test_command_finite_near_min_t_go();
    test_infeasible_t_go_rejected();
    test_hover_waypoint_moves_with_body();
    test_guidance_command_sane();
    test_end_to_end_same_body_landing();
    test_no_state_mutation();
    test_abort_no_output();
    test_approach_stays_outside_terrain();
    test_descent_only_inside_corridor();
    test_touchdown_matches_moving_surface();
    test_primary_companion_same_and_cross_body();
    test_guidance_call_bounded_in_horizon();

    if (failures == 0) {
        std::printf("ALL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d FAILURES\n", failures);
    return 1;
}
