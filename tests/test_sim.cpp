// M05 simulation tests: the two-body binary system, body-relative navigation,
// landing / crash / takeoff rules, the contract loop, circularize, and the
// presentation-only helpers (interpolation, flame animation).
#include "lander/guarded_actions.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) { return std::abs(a - b) <= eps; }

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

double norm_angle(double a) {
    a = std::fmod(a, 2.0 * lander::kPi);
    if (a < 0.0) {
        a += 2.0 * lander::kPi;
    }
    return a;
}

// A flying (unlanded, uncrashed) ship at a chosen world point.
lander::State state_at(double x, double y, double vx, double vy, double angle,
                       double omega) {
    lander::State s{};
    s.x = x;
    s.y = y;
    s.vx = vx;
    s.vy = vy;
    s.angle = angle;
    s.omega = omega;
    s.fuel = 1000.0;
    s.landed = false;
    s.crashed = false;
    return s;
}

// A flying ship placed relative to body i at ephemeris time 0: `altitude`
// metres above the local surface at body-local `arc`, with radial /
// tangential velocity given relative to the (tidally locked, spinning)
// surface point at that location, and an attitude offset from the local
// vertical.
lander::State state_relative(const lander::BinarySystem& bin, int i, double arc,
                             double altitude, double radial_v, double tang_v,
                             double angle_offset) {
    const lander::Body& body = bin.body(i);
    const lander::Vec2 pos = bin.position(i, 0.0);
    const lander::Vec2 vel = bin.velocity(i, 0.0);
    const double theta = body.terrain.angle_at_arc(arc);
    const double r = body.terrain.surface_radius_at_arc(arc) + altitude;
    const double up_x = std::cos(theta);
    const double up_y = std::sin(theta);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);
    // M05-R3 tidal locking: a point fixed at radius r carries the spin
    // velocity omega x offset, so a ship at rest with respect to the pad
    // must carry it too.
    const double ox = r * up_x;
    const double oy = r * up_y;
    const double spin_x = -bin.omega() * oy;
    const double spin_y = bin.omega() * ox;
    lander::State s{};
    s.x = pos.x + r * up_x;
    s.y = pos.y + r * up_y;
    s.vx = vel.x + spin_x + radial_v * up_x + tang_v * right_x;
    s.vy = vel.y + spin_y + radial_v * up_y + tang_v * right_y;
    s.angle = theta - 0.5 * lander::kPi + angle_offset;
    s.fuel = 1000.0;
    s.landed = false;
    s.crashed = false;
    return s;
}

// Advance with no input until the ship lands or crashes (or the bound is hit).
int run_to_contact(lander::Simulation& sim) {
    const double dt = sim.config().fixed_dt;
    for (int i = 0; i < 24 * 120; ++i) {
        if (sim.state().landed || sim.state().crashed) {
            return i;
        }
        sim.advance(dt, {});
    }
    return -1;
}

// M05-R1-V01: the intrinsic gravity / orbit parameters of both bodies follow
// the canonical 1/9 scaling law.
void test_reference_values() {
    const lander::Config cfg{};
    const double mu0 = cfg.mu;
    const double R0 = lander::kReferenceRadius;
    const double s = 1.0 / 9.0;
    const double R1 = R0 * s;
    const double mu1 = mu0 * s * s;

    check_close(mu0 / (R0 * R0), 1.62, 0.005, "primary surface gravity ~ 1.62");
    check_close(mu1 / (R1 * R1), 1.62, 0.005, "companion surface gravity ~ 1.62");
    check_close(mu1 / (R1 * R1), mu0 / (R0 * R0), 1e-9,
                "companion gravity equals the primary's (1/9 scaling)");
    check_close(std::sqrt(mu0 / R0), 23.2048, 0.001, "primary circular speed");
    check_close(2 * lander::kPi * std::sqrt((R0 * R0 * R0) / mu0), 90.0, 0.01,
                "primary circular period ~ 90 s");
    check_close(std::sqrt(mu1 / R1), std::sqrt(mu0 / R0) / 3.0, 1e-6,
                "companion circular speed ~ 1/3 primary");
    check_close(2 * lander::kPi * std::sqrt((R1 * R1 * R1) / mu1),
                2 * lander::kPi * std::sqrt((R0 * R0 * R0) / mu0) / 3.0, 1e-6,
                "companion circular period ~ 1/3 primary");
}

// M05-R1-V05 / P03: terrain stays deterministic per seed, wraps, and the
// companion is a 1/9 copy of the primary's generator.
void test_terrain() {
    lander::Terrain a1(7), a2(7), b(8);
    check(a1.reference_radius() == b.reference_radius(), "same reference radius");
    check(a1.circumference() == b.circumference(), "primary circumference equal");
    check(
        a1.surface_radius_at_arc(123.0) == a2.surface_radius_at_arc(123.0),
        "deterministic surface radius for a seed");
    check(a1.surface_radius_at_arc(123.0) != b.surface_radius_at_arc(123.0),
          "different seed differs");

    const double C = a1.circumference();
    check_close(a1.surface_radius_at_arc(100.0),
                a1.surface_radius_at_arc(100.0 - C), 1e-12,
                "surface radius is periodic in arc");
    check_close(a1.arc_at_angle(a1.angle_at_arc(500.0)), 500.0, 1e-9,
                "angle -> arc -> angle round trip");

    check(a1.pads().size() == 3, "three pads");
    check_close(a1.pads().front().center_arc, 0.0, 1e-12, "front pad at arc 0");
    for (const auto& pad : a1.pads()) {
        check(a1.pad_at_arc(pad.center_arc) == &pad,
              "pad_at_arc returns the pad at its own centre");
    }
    check(a1.pad_at_arc(0.5 * C) == nullptr, "mid-arc is not a pad");

    double mn = 1e300, mx = -1e300;
    for (int i = 0; i < 2048; ++i) {
        const double r = a1.surface_radius_at_arc(i * C / 2048.0);
        mn = std::min(mn, r);
        mx = std::max(mx, r);
    }
    check(mx - mn > 5.0, "primary relief is substantial (> 5 m)");
    check_close(a1.max_surface_radius(), mx, 1.0,
                "max_surface_radius ~ sampled maximum");

    const double r1 = lander::kReferenceRadius / 9.0;
    lander::Terrain comp(999, r1, 1.0 / 9.0);
    check_close(comp.circumference(), C / 9.0, 1e-9,
                "companion circumference is 1/9");
    double cmn = 1e300, cmx = -1e300;
    for (int i = 0; i < 2048; ++i) {
        const double r =
            comp.surface_radius_at_arc(i * comp.circumference() / 2048.0);
        cmn = std::min(cmn, r);
        cmx = std::max(cmx, r);
    }
    check(cmx - cmn < (mx - mn) / 2.0, "companion relief much smaller");
    check(cmx - cmn > 0.5, "companion relief still visible (> 0.5 m)");
}

// M05-R1-V05 / V06 / V09 (init): reset produces a landed ship at the primary
// base with a fresh contract toward the companion, and is deterministic.
void test_reset_state() {
    lander::Simulation simA, simB;
    simA.reset(42);
    simB.reset(42);
    const lander::State& s = simA.state();
    const lander::BinarySystem& bin = simA.binary();
    check(s.landed, "reset lands the ship");
    check(!s.crashed, "reset is not crashed");
    check(s.landed_body == 0, "reset lands on the primary");
    check_close(s.landed_arc, 0.0, 1e-12, "reset lands at the base pad (arc 0)");
    check_close(s.fuel, simA.config().fuel, 1e-12, "reset fills the tank");
    check(simA.reference_body() == 0, "reset reference body is the primary");
    check_close(simA.sim_time(), 0.0, 1e-12, "reset sim_time is 0");
    check_close(simA.presentation_time(), -simA.config().fixed_dt, 1e-9,
                "presentation_time at reset");
    check(simA.contracts_completed() == 0, "no contracts completed yet");

    const lander::State ref = lander::attached_state(bin, 0, 0.0, 0.0);
    check_close(s.x, ref.x, 1e-9, "reset x matches the attached state");
    check_close(s.y, ref.y, 1e-9, "reset y matches the attached state");
    check_close(s.vx, ref.vx, 1e-9, "reset vx matches the attached state");
    check_close(s.vy, ref.vy, 1e-9, "reset vy matches the attached state");
    check_close(norm_angle(s.angle), norm_angle(ref.angle), 1e-9,
                "reset angle matches the attached state");

    check(simA.contract().origin_body == 0, "contract origin is the primary");
    check(simA.contract().destination_body == 1,
          "contract destination is the companion");
    const int front_mult =
        simA.binary().body(1).terrain.pads().front().multiplier;
    check(simA.contract().reward == 100 * front_mult,
          "contract reward = 100 * destination front-pad multiplier");

    check(simA.state() == simB.state(), "identical seeds -> identical state");
    check_close(simA.sim_time(), simB.sim_time(), 1e-12, "identical sim_time");
}

// M05-R1-V04: the body-relative navigation helpers (distance, altitude, local
// up / attitude / velocity / angular rate) are correct on both bodies.
void test_local_frame() {
    lander::Config cfg{};
    auto bin = lander::BinarySystem::canonical(cfg.mu, 5,
                                               lander::companion_seed(5));
    const std::array<double, 4> fracs = {0.0, 0.2, 0.5, 0.8};
    for (int i = 0; i < 2; ++i) {
        const lander::Body& body = bin.body(i);
        const double C = body.terrain.circumference();
        for (double fr : fracs) {
            const double arc = fr * C;
            const double theta = body.terrain.angle_at_arc(arc);
            const double r = body.terrain.surface_radius_at_arc(arc) + 10.0;
            const lander::Vec2 pos = bin.position(i, 0.0);
            const lander::Vec2 vel = bin.velocity(i, 0.0);
            const double up_x = std::cos(theta);
            const double up_y = std::sin(theta);
            const double right_x = std::sin(theta);
            const double right_y = -std::cos(theta);

            lander::State s{};
            s.x = pos.x + r * up_x;
            s.y = pos.y + r * up_y;
            s.vx = vel.x + 3.0 * up_x + 4.0 * right_x;
            s.vy = vel.y + 3.0 * up_y + 4.0 * right_y;
            s.angle = norm_angle(theta - 0.5 * lander::kPi);
            s.fuel = 100.0;

            check_close(lander::radial_distance(s, pos), r, 1e-9,
                        "radial_distance");
            check_close(lander::altitude_at(body.terrain, s, pos), 10.0, 1e-6,
                        "altitude_at");
            const double exp_up =
                std::atan2(std::sin(theta), std::cos(theta)) -
                0.5 * lander::kPi;
            check_close(lander::local_up_angle(s, pos), exp_up, 1e-9,
                        "local_up_angle");
            const lander::LocalVelocity lv = lander::local_velocity(s, pos, vel);
            check_close(lv.radial, 3.0, 1e-9, "local radial velocity");
            check_close(lv.tangential, 4.0, 1e-9, "local tangential velocity");
            check_close(lander::local_angular_velocity(s, pos, vel), 4.0 / r,
                        1e-9, "local_angular_velocity = tangential / r");
            check_close(lander::local_attitude_angle(s, pos), 0.0, 1e-9,
                        "aligned attitude -> 0");

            lander::State st = s;
            st.angle = norm_angle(theta - 0.5 * lander::kPi + 0.2);
            check_close(lander::local_attitude_angle(st, pos), 0.2, 1e-9,
                        "tilted attitude -> 0.2");
        }
    }
}

// M05-R1-V02 / V07: one fixed step integrates the summed two-body field,
// applies thrust along the nose, rotates, and burns fuel; semi-implicit Euler.
void test_one_step_physics() {
    lander::Config cfg{};
    auto bin = lander::BinarySystem::canonical(cfg.mu, 6,
                                               lander::companion_seed(6));
    const double dt = cfg.fixed_dt;
    const lander::Vec2 p0 = bin.position(0, 0.0);
    // A point 400 m to the left of the primary, clear of both bodies.
    const double px = p0.x - 400.0;
    const double py = p0.y;

    {
        lander::Simulation sim;
        sim.set_state(state_at(px, py, 0.0, 0.0, 0.0, 0.0));
        sim.advance(dt, {});
        const lander::Vec2 a = bin.gravity({px, py}, 0.0);
        const double vx = a.x * dt;
        const double vy = a.y * dt;
        const lander::State& r = sim.state();
        check_close(r.vx, vx, 1e-9, "unpowered one step: vx = gravity*dt");
        check_close(r.vy, vy, 1e-9, "unpowered one step: vy = gravity*dt");
        check_close(r.x, px + vx * dt, 1e-9, "unpowered one step: x");
        check_close(r.y, py + vy * dt, 1e-9, "unpowered one step: y");
        check_close(r.angle, 0.0, 1e-9, "unpowered one step: attitude");
        check_close(r.fuel, 1000.0, 1e-12, "unpowered one step: no fuel burn");
        check(!r.landed && !r.crashed, "unpowered one step: still in flight");

        // The total field must differ from a primary-only field (companion
        // is active, never switched off).
        const double rx = px - p0.x;
        const double ry = py - p0.y;
        const double rr = std::hypot(rx, ry);
        const double inv = 1.0 / (rr * rr * rr);
        const lander::Vec2 aprim{-bin.body(0).mu * rx * inv,
                                 -bin.body(0).mu * ry * inv};
        check(std::hypot(a.x - aprim.x, a.y - aprim.y) > 1e-3,
              "two-body field differs from primary-only");
    }

    {
        lander::Simulation sim;
        sim.set_state(state_at(px, py, 0.0, 0.0, 0.0, 0.0));
        lander::Input in{};
        in.rotate_right = true;
        in.main_throttle = 1.0;
        sim.advance(dt, in);
        const lander::Vec2 g = bin.gravity({px, py}, 0.0);
        // Nose at angle 0 -> thrust (0, +main_accel).
        const double ax = g.x;
        const double ay = g.y + cfg.main_accel;
        const double omega1 = cfg.rotate_accel * dt;
        const double vx = ax * dt;
        const double vy = ay * dt;
        const lander::State& r = sim.state();
        check_close(r.omega, omega1, 1e-12, "rotation sets omega");
        check_close(r.vx, vx, 1e-9, "thrust+gravity one step: vx");
        check_close(r.vy, vy, 1e-9, "thrust+gravity one step: vy");
        check_close(r.x, px + vx * dt, 1e-9, "thrust+gravity one step: x");
        check_close(r.y, py + vy * dt, 1e-9, "thrust+gravity one step: y");
        check_close(norm_angle(r.angle), norm_angle(omega1 * dt), 1e-9,
                    "angle follows the new omega");
        check_close(r.fuel, 1000.0 - cfg.fuel_burn * dt, 1e-9, "fuel burns");
    }
}

// M05-R1-V20: the fixed-step integrator is independent of the frame cadence.
void test_fixed_step_determinism() {
    lander::Config cfg{};
    const std::uint64_t seed = 11;
    auto bin = lander::BinarySystem::canonical(cfg.mu, seed,
                                               lander::companion_seed(seed));
    const double dt = cfg.fixed_dt;
    const lander::Vec2 p0 = bin.position(0, 0.0);
    const lander::State base =
        state_at(p0.x - 400.0, p0.y, 0.0, 0.0, 0.3, 0.0);

    lander::Simulation simA, simB;
    simA.reset(seed);
    simB.reset(seed);
    simA.set_state(base);
    simB.set_state(base);
    for (int i = 0; i < 120; ++i) {
        simA.advance(dt, {});
    }
    for (int i = 0; i < 60; ++i) {
        simB.advance(2.0 * dt, {});
    }

    check(simA.state() == simB.state(),
          "frame cadence does not change the resulting state");
    check(simA.state().ticks == 120, "1*dt cadence ran 120 fixed steps");
    check(simB.state().ticks == 120, "2*dt cadence ran 120 fixed steps");
    check_close(simA.sim_time(), 1.0, 1e-9, "1*dt cadence reaches 1 s");
    check_close(simB.sim_time(), 1.0, 1e-9, "2*dt cadence reaches 1 s");
}

// M05-R1-V05: a safe body-relative drop lands on the pad (and only there) on
// either body, co-moving with the body.
void test_landing_rules() {
    lander::Config cfg{};
    const std::uint64_t seed = 21;
    lander::Simulation probe;
    probe.reset(seed);

    for (int body = 0; body < 2; ++body) {
        const auto& pads = probe.terrain(body).pads();
        for (const auto& pad : pads) {
            lander::Simulation sim;
            sim.reset(seed);
            lander::State s = state_relative(sim.binary(), body, pad.center_arc,
                                             0.3, -1.0, 0.2, 0.05);
            sim.set_state(s);
            const int steps = run_to_contact(sim);
            check(steps >= 0, "reached a terminal ground state");
            const lander::State& r = sim.state();
            check(r.landed && !r.crashed, "safe drop lands and does not crash");
            check(r.landed_body == body, "landed on the intended body");
            const lander::Vec2 bpos = sim.binary().position(body, sim.sim_time());
            const double surface =
                sim.terrain(body).surface_radius_at_arc(r.landed_arc);
            check_close(lander::radial_distance(r, bpos), surface, 1e-6,
                        "rests on the pad surface");
            check(sim.terrain(body).pad_at_arc(r.landed_arc) != nullptr,
                  "landed on a pad");
            // At rest relative to the pad means at rest relative to the
            // rotating surface point the ship landed on (M05-R3).
            const lander::Vec2 sp_vel = sim.binary().surface_point(
                body, sim.terrain(body).angle_at_arc(r.landed_arc), surface,
                sim.sim_time()).velocity;
            const lander::LocalVelocity lv =
                lander::local_velocity(r, bpos, sp_vel);
            check(std::abs(lv.radial) <= cfg.safe_vertical_speed + 1e-6,
                  "radial speed is within the safe band at rest");
            check(std::abs(lv.tangential) <= cfg.safe_horizontal_speed + 1e-6,
                  "tangential speed is within the safe band at rest");
        }
    }
}

// M05-R1-V05: unsafe contact (non-pad arc, excessive speed, or misaligned
// attitude) crashes the ship on the body it touched.
void test_crash_rules() {
    const std::uint64_t seed = 22;
    lander::Simulation probe;
    probe.reset(seed);

    // (a) Non-pad arc: even gentle speeds crash.
    {
        const int body = 0;
        const double arc = 0.5 * probe.terrain(body).circumference();
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s = state_relative(sim.binary(), body, arc, 0.3, -1.0, 0.2,
                                         0.05);
        sim.set_state(s);
        run_to_contact(sim);
        const lander::State& r = sim.state();
        check(r.crashed && !r.landed, "non-pad arc crashes");
        check(r.crash_body == body, "crash body is the primary");
    }
    // (b) Excessive radial speed on a pad.
    {
        const int body = 1;
        const auto& pad = probe.terrain(body).pads().front();
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s =
            state_relative(sim.binary(), body, pad.center_arc, 0.3, -10.0, 0.0, 0.0);
        sim.set_state(s);
        run_to_contact(sim);
        const lander::State& r = sim.state();
        check(r.crashed, "excessive radial speed crashes");
        check(r.crash_body == body, "crash body is the companion");
    }
    // (c) Excessive tangential speed on a pad.
    {
        const int body = 0;
        const auto& pad = probe.terrain(body).pads().front();
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s =
            state_relative(sim.binary(), body, pad.center_arc, 0.3, -1.0, 2.5, 0.0);
        sim.set_state(s);
        run_to_contact(sim);
        check(sim.state().crashed, "excessive tangential speed crashes");
    }
    // (d) Misaligned attitude (0.2 rad) on a pad.
    {
        const int body = 1;
        const auto& pad = probe.terrain(body).pads().front();
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s =
            state_relative(sim.binary(), body, pad.center_arc, 0.3, -1.0, 0.0, 0.2);
        sim.set_state(s);
        run_to_contact(sim);
        check(sim.state().crashed, "misaligned attitude (0.2 rad) crashes");
    }
    // (e) Attitude aligned to the world vertical instead of the local vertical.
    {
        const int body = 0;
        const auto& pad = probe.terrain(body).pads()[1];
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s =
            state_relative(sim.binary(), body, pad.center_arc, 0.3, -1.0, 0.0, 0.0);
        s.angle = 0.0;
        sim.set_state(s);
        run_to_contact(sim);
        check(sim.state().crashed, "world-vertical attitude crashes");
    }
}

// M05-R1-V06: a landed ship rides its moving body (no jitter); a thrust below
// surface gravity cannot lift it; full throttle takes off, inheriting the body
// velocity. Verified for both bodies.
void test_landed_attachment_and_takeoff() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 31;

    // (A) Landed ship tracks its body over time.
    {
        lander::Simulation sim;
        sim.reset(seed);
        for (int i = 0; i < 120; ++i) {
            sim.advance(dt, {});
        }
        const lander::State& r = sim.state();
        check(r.landed && !r.crashed, "still landed after 1 s of no input");
        check(r.landed_body == 0, "still on the primary");
        const lander::State ref =
            lander::attached_state(sim.binary(), 0, 0.0, sim.sim_time());
        check_close(r.x, ref.x, 1e-6, "x tracks the body");
        check_close(r.y, ref.y, 1e-6, "y tracks the body");
        check_close(r.vx, ref.vx, 1e-6, "vx tracks the body");
        check_close(r.vy, ref.vy, 1e-6, "vy tracks the body");
        const lander::Vec2 start = sim.binary().position(0, 0.0);
        check(std::hypot(r.x - start.x, r.y - start.y) > 0.05,
              "the body moved over time");
    }

    // (B) Small throttle (below surface gravity) stays landed: no jitter, no
    // fuel use, no attitude change.
    {
        lander::Simulation sim;
        sim.reset(seed);
        lander::Input in{};
        in.main_throttle = 0.01;
        for (int i = 0; i < 60; ++i) {
            sim.advance(dt, in);
        }
        const lander::State& r = sim.state();
        check(r.landed && !r.crashed, "small throttle stays landed");
        check_close(r.fuel, cfg.fuel, 1e-9, "small throttle burns no fuel");
        const lander::State ref =
            lander::attached_state(sim.binary(), 0, 0.0, sim.sim_time());
        check_close(r.x, ref.x, 1e-6, "no jitter: x stays on the pad");
        check_close(r.y, ref.y, 1e-6, "no jitter: y stays on the pad");
    }

    // (C) Full throttle takes off from the primary.
    {
        lander::Simulation sim;
        sim.reset(seed);
        lander::Input in{};
        in.main_throttle = 1.0;
        sim.advance(dt, in);
        check(!sim.state().landed, "full throttle takes off on the first step");
        check(!sim.state().crashed, "takeoff is not a crash");
        for (int i = 0; i < 119; ++i) {
            sim.advance(dt, in);
        }
        const lander::State& r = sim.state();
        const lander::Vec2 bpos = sim.binary().position(0, sim.sim_time());
        check(lander::altitude_at(sim.terrain(0), r, bpos) > 0.0,
              "the ship has risen above the pad");
        check(!r.landed && !r.crashed, "in flight after takeoff");
        check(r.fuel < cfg.fuel, "takeoff burned fuel");
    }

    // (D) Takeoff from the companion uses the same generic path.
    {
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s = lander::attached_state(sim.binary(), 1, 0.0, 0.0);
        s.fuel = cfg.fuel;
        sim.set_state(s);
        lander::Input in{};
        in.main_throttle = 1.0;
        sim.advance(dt, in);
        check(!sim.state().landed, "companion takeoff lifts off");
        for (int i = 0; i < 119; ++i) {
            sim.advance(dt, in);
        }
        const lander::State& r = sim.state();
        const lander::Vec2 bpos = sim.binary().position(1, sim.sim_time());
        check(lander::altitude_at(sim.terrain(1), r, bpos) > 0.0,
              "companion ship has risen");
    }
}

// M05-R1-V09 / V14: a crashed state is terminal; the ship and the clock freeze.
void test_terminal_state_is_frozen() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 41;

    lander::Simulation sim;
    sim.reset(seed);
    const double arc = 0.5 * sim.terrain(0).circumference();
    lander::State s =
        state_relative(sim.binary(), 0, arc, 0.3, -30.0, 0.0, 0.0);
    sim.set_state(s);
    run_to_contact(sim);
    check(sim.state().crashed, "ship crashed");
    const lander::State before = sim.state();
    const double t_before = sim.sim_time();
    for (int i = 0; i < 100; ++i) {
        sim.advance(dt, {});
    }
    check(sim.state() == before, "crashed state is frozen");
    check_close(sim.sim_time(), t_before, 1e-12, "sim time freezes after crash");
}

// M05-R1-V14: set_state normalises the angle into [0, 2pi) and clamps fuel.
void test_set_state_normalizes() {
    lander::Simulation sim;
    sim.reset(1);

    {
        lander::State s{};
        s.x = 100;
        s.y = 0;
        s.angle = 10.0;
        s.fuel = 500;
        sim.set_state(s);
        check_close(sim.state().angle, 10.0 - 2.0 * lander::kPi, 1e-9,
                    "angle wrapped into [0, 2pi)");
        check_close(sim.state().fuel, 500.0, 1e-12, "positive fuel preserved");
    }
    {
        lander::State s{};
        s.x = 100;
        s.y = 0;
        s.angle = -1.0;
        sim.set_state(s);
        check_close(sim.state().angle, -1.0 + 2.0 * lander::kPi, 1e-9,
                    "negative angle wrapped");
    }
    {
        lander::State s{};
        s.x = 100;
        s.y = 0;
        s.fuel = -50.0;
        sim.set_state(s);
        check_close(sim.state().fuel, 0.0, 1e-12, "negative fuel clamped to 0");
    }
}

// M05-R1-V07: circularize sets a body-relative circular velocity about the
// current reference body (no position / attitude / fuel change).
void test_circularize_state() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 51;

    // Primary: reference body is the primary immediately after reset.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const double r0 = sim.terrain(0).max_surface_radius() + 20.0;
        const double alt = r0 - sim.terrain(0).surface_radius_at_arc(0.0);
        sim.set_state(
            state_relative(sim.binary(), 0, 0.0, alt, 0.0, 0.0, 0.0));
        sim.circularize();
        const lander::Vec2 bpos = sim.binary().position(0, 0.0);
        const lander::Vec2 bvel = sim.binary().velocity(0, 0.0);
        const double r = lander::radial_distance(sim.state(), bpos);
        const double theta =
            std::atan2(sim.state().y - bpos.y, sim.state().x - bpos.x);
        const double speed = std::sqrt(sim.binary().body(0).mu / r);
        check_close(r, r0, 1e-6, "circularize does not move the ship");
        const lander::Vec2 vexp{bvel.x + std::sin(theta) * speed,
                                bvel.y + (-std::cos(theta)) * speed};
        check_close(sim.state().vx, vexp.x, 1e-6, "circularize vx (primary)");
        check_close(sim.state().vy, vexp.y, 1e-6, "circularize vy (primary)");
        check_close(sim.state().angle, 0.0, 1e-9,
                    "circularize does not change attitude");
        check_close(sim.state().fuel, 1000.0, 1e-9,
                    "circularize does not burn fuel");
    }

    // Companion: a step near body 1 flips the reference to the companion.
    {
        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(
            state_relative(sim.binary(), 1, 0.0, 8.0, 0.0, 0.0, 0.0));
        sim.advance(dt, {});
        check(sim.reference_body() == 1, "reference flips to the companion");
        sim.circularize();
        const lander::Vec2 bpos = sim.binary().position(1, sim.sim_time());
        const lander::Vec2 bvel = sim.binary().velocity(1, sim.sim_time());
        const double r = lander::radial_distance(sim.state(), bpos);
        const double theta =
            std::atan2(sim.state().y - bpos.y, sim.state().x - bpos.x);
        const double speed = std::sqrt(sim.binary().body(1).mu / r);
        const lander::Vec2 vexp{bvel.x + std::sin(theta) * speed,
                                bvel.y + (-std::cos(theta)) * speed};
        check_close(sim.state().vx, vexp.x, 1e-6, "circularize vx (companion)");
        check_close(sim.state().vy, vexp.y, 1e-6, "circularize vy (companion)");
    }

    // Circularise is a no-op while landed or crashed.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const lander::State before = sim.state();
        sim.circularize();
        check(sim.state() == before, "circularize is a no-op while landed");
    }
}

// M05-R1-V07: after circularizing, the ship remains in a usable orbit about
// the primary (bounded radius, no crash, no escape).
void test_orbit_is_usable() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 61;
    lander::Simulation sim;
    sim.reset(seed);

    const double r0 = sim.terrain(0).max_surface_radius() + 25.0;
    const double alt = r0 - sim.terrain(0).surface_radius_at_arc(0.0);
    sim.set_state(state_relative(sim.binary(), 0, 0.0, alt, 0.0, 0.0, 0.0));
    sim.circularize();

    const double local_period =
        2.0 * lander::kPi * std::sqrt((r0 * r0 * r0) / cfg.mu);
    const int steps = (int)std::lround(1.5 * local_period / dt);
    double rmin = 1e300, rmax = -1e300;
    for (int i = 0; i < steps && !sim.state().crashed; ++i) {
        sim.advance(dt, {});
        const lander::Vec2 p = sim.binary().position(0, sim.sim_time());
        const double r =
            std::hypot(sim.state().x - p.x, sim.state().y - p.y);
        rmin = std::min(rmin, r);
        rmax = std::max(rmax, r);
    }

    check(!sim.state().crashed, "orbit does not crash");
    check(rmin > sim.terrain(0).max_surface_radius(), "orbit stays above terrain");
    check(rmin > 0.7 * r0, "orbit does not decay toward the surface");
    check(rmax < 1.5 * r0, "orbit does not escape the primary");
}

// M05-R1-P02: refuel refills the tank in flight and on the ground, and never
// after a crash.
void test_refuel_only_changes_fuel() {
    lander::Config cfg{};
    const std::uint64_t seed = 71;
    const double dt = cfg.fixed_dt;

    // In flight.
    {
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s = state_at(0.0, 400.0, 10.0, 0.0, 0.0, 0.0);
        s.fuel = 200.0;  // a partially drained tank
        sim.set_state(s);
        for (int i = 0; i < 120; ++i) {
            sim.advance(dt, {});
        }
        check(!sim.state().landed && !sim.state().crashed, "still in flight");
        const double fuel_before = sim.state().fuel;
        sim.refuel();
        check_close(sim.state().fuel, cfg.fuel, 1e-9, "refuel refills the tank");
        check(sim.state().fuel > fuel_before, "refuel increases the tank");
    }
    // On the ground.
    {
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s = sim.state();  // the landed state produced by reset
        s.fuel = 5.0;
        sim.set_state(s);
        sim.refuel();
        check_close(sim.state().fuel, cfg.fuel, 1e-9, "refuel works on the ground");
        check(sim.state().landed, "still landed after refuel");
    }
    // Never after a crash.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const double arc = 0.5 * sim.terrain(0).circumference();
        lander::State s = state_relative(sim.binary(), 0, arc, 0.3, -30.0, 0.0, 0.0);
        sim.set_state(s);
        run_to_contact(sim);
        check(sim.state().crashed, "crashed for the refuel test");
        lander::State cs = sim.state();
        cs.fuel = 1.0;
        sim.set_state(cs);
        const double fuel_before = sim.state().fuel;
        sim.refuel();
        check_close(sim.state().fuel, fuel_before, 1e-12,
                    "refuel is blocked after a crash");
    }
}

// M05-R1-P01: the flame is driven by the continuous presentation clock, not an
// integer tick count.
void test_flame_animation_continuous() {
    for (double t = 0.0; t < 2.0; t += 0.03) {
        check_close(lander::flame_length(0.0, t), 0.0, 1e-12,
                    "no thrust -> no flame");
    }

    double mn = 1e300, mx = -1e300;
    double prev_len = -1.0;
    bool saw_change = false;
    for (double t = 0.0; t < 2.0; t += 0.001) {
        const double len = lander::flame_length(1.0, t);
        mn = std::min(mn, len);
        mx = std::max(mx, len);
        if (prev_len >= 0.0 && std::abs(len - prev_len) > 1e-12) {
            saw_change = true;
        }
        prev_len = len;
    }
    check(mn >= 0.42 && mx <= 1.88,
          "full-thrust flame stays in the documented ~[0.43, 1.87] range");
    check(mx - mn > 0.8, "flame has substantial variation");
    check(saw_change, "flame length varies continuously with time");
    check(std::abs(lander::flame_length(1.0, 0.3000) -
                   lander::flame_length(1.0, 0.3001)) < 1e-2,
          "flame varies smoothly between close times");
}

// M05-R1-V21: render interpolation blends positions linearly, attitude by
// shortest arc, never touches fuel, and snaps at the ends.
void test_interpolated_state() {
    lander::State prev{};
    prev.x = 0;
    prev.y = 0;
    prev.angle = 0.0;
    prev.fuel = 500;
    prev.vx = 1;
    prev.vy = 2;
    lander::State cur{};
    cur.x = 10;
    cur.y = 0;
    cur.angle = 1.0;
    cur.fuel = 400;
    cur.vx = 3;
    cur.vy = 4;

    const lander::State mid = lander::interpolated_state(prev, cur, 0.5, false);
    check_close(mid.x, 5.0, 1e-12, "x lerps linearly");
    check_close(mid.y, 0.0, 1e-12, "y lerps linearly");
    check_close(mid.angle, 0.5, 1e-9, "angle lerps (shortest arc)");
    check_close(mid.fuel, cur.fuel, 1e-12, "fuel is not interpolated");
    check_close(mid.vx, cur.vx, 1e-12, "vx is not interpolated");

    check(lander::interpolated_state(prev, cur, 0.0, false) == cur,
          "alpha 0 -> current");
    check(lander::interpolated_state(prev, cur, 1.0, false) == cur,
          "alpha 1 -> current");
    check(lander::interpolated_state(prev, cur, 0.3, true) == cur,
          "snap_to_current -> current");

    lander::State ap{};
    ap.angle = 350.0 * lander::kPi / 180.0;
    lander::State ac{};
    ac.angle = 10.0 * lander::kPi / 180.0;
    const lander::State am = lander::interpolated_state(ap, ac, 0.5, false);
    check_close(am.angle, 0.0, 1e-6, "angle takes the short way around");
}

// M05-R1-V09: place the ship just above a chosen body's pad and let it fall
// until it lands or crashes; used to drive the contract state machine.
int drop_on(lander::Simulation& sim, int body, int pad_index) {
    const auto& pads = sim.terrain(body).pads();
    lander::State s =
        state_relative(sim.binary(), body, pads[pad_index].center_arc, 0.3, -1.0,
                       0.2, 0.05);
    sim.set_state(s);
    return run_to_contact(sim);
}

// M05-R2-V01: reference-body selection follows local gravitational influence
// with deterministic hysteresis, still honors the landed body, and does not
// flicker during a stable primary orbit.
void test_reference_body_influence() {
    // Pure selection rule.
    check(lander::reference_body_for(81.0, 1.0, 9.0, 1.0, 0) == 0,
          "equal influence keeps the primary");
    check(lander::reference_body_for(81.0, 1.0, 9.0, 1.0, 1) == 1,
          "equal influence keeps the companion");
    check(lander::reference_body_for(81.0, 1.0, 9.0, 2.0, 0) == 0,
          "primary is kept below the switching margin");
    check(lander::reference_body_for(81.0, 1.0, 10.0, 1.0, 1) == 1,
          "companion is kept below the switching margin");
    check(lander::reference_body_for(81.0, 1.0, 10.0, 1.0, 0) == 1,
          "companion wins past the switching margin");
    check(lander::reference_body_for(81.0, 1.0, 1.0, 10.0, 1) == 0,
          "primary wins past the switching margin");
    check(lander::reference_body_for(81.0, 1.0, 0.0, 1.0, 0) == 0,
          "zero primary distance dominates");
    check(lander::reference_body_for(81.0, 1.0, 1.0, 0.0, 1) == 1,
          "zero companion distance dominates");
    check(lander::reference_body_for(81.0, 1.0, 0.0, 0.0, 0) == 0,
          "both zero keeps the primary");
    check(lander::reference_body_for(81.0, 1.0, 0.0, 0.0, 1) == 1,
          "both zero keeps the companion");

    const lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 71;

    // Near the companion, one step selects the companion.
    {
        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(state_relative(sim.binary(), 1, 0.0, 8.0, 0.0, 0.0, 0.0));
        sim.advance(dt, {});
        check(sim.reference_body() == 1, "near the companion, the reference is the companion");
    }

    // Far from the companion, the primary reference is stable for a second.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const lander::Vec2 p0 = sim.binary().position(0, 0.0);
        sim.set_state(state_at(p0.x - 400.0, p0.y, 0.0, 0.0, 0.0, 0.0));
        for (int i = 0; i < 120; ++i) {
            sim.advance(dt, {});
        }
        check(sim.reference_body() == 0,
              "a ship far from the companion keeps the primary reference");
    }

    // A stable circular orbit around the primary does not flip.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const double r0 = sim.terrain(0).max_surface_radius() + 25.0;
        const double alt = r0 - sim.terrain(0).surface_radius_at_arc(0.0);
        sim.set_state(state_relative(sim.binary(), 0, 0.0, alt, 0.0, 0.0, 0.0));
        sim.circularize();
        const double local_period =
            2.0 * lander::kPi * std::sqrt((r0 * r0 * r0) / cfg.mu);
        const int steps = (int)std::lround(1.5 * local_period / dt);
        bool flipped = false;
        for (int i = 0; i < steps && !sim.state().crashed; ++i) {
            sim.advance(dt, {});
            flipped = flipped || sim.reference_body() != 0;
        }
        check(!sim.state().crashed, "reference-orbit probe does not crash");
        check(!flipped, "a stable primary orbit keeps the primary reference");
    }

    // The landed body always forces the reference.
    {
        lander::Simulation sim;
        sim.reset(seed);
        drop_on(sim, 1, 0);
        check(sim.state().landed, "companion landing probe lands");
        check(sim.reference_body() == 1, "landing on the companion forces its reference");
    }
    {
        lander::Simulation sim;
        sim.reset(seed);
        drop_on(sim, 0, 0);
        check(sim.state().landed, "primary landing probe lands");
        check(sim.reference_body() == 0, "landing on the primary forces its reference");
    }
}

// M05-R1-V09: the delivery contract loop. A safe landing on the destination
// base pad completes the active contract exactly once (pad bonus plus the
// contract reward, after which the destination reverses); a safe landing on a
// non-base pad does not complete it.
void test_contract_loop() {
    const std::uint64_t seed = 23;

    // (a) Complete exactly once on the destination (companion) base pad.
    {
        lander::Simulation sim;
        sim.reset(seed);
        check(sim.contract().origin_body == 0, "initial contract from the primary");
        check(sim.contract().destination_body == 1, "initial contract to the companion");
        const auto& cpads = sim.terrain(1).pads();
        const int cf = cpads.front().multiplier;
        const int reward0 = sim.contract().reward;
        check(reward0 == 100 * cf, "initial reward is the companion base value");
        const int score0 = sim.state().score;  // 0

        const int steps = drop_on(sim, 1, 0);  // the destination base pad
        check(steps >= 0, "reached contact on the destination base");
        const lander::State& r = sim.state();
        check(r.landed && !r.crashed, "safe landing on the destination base");
        check(r.landed_body == 1, "landed on the companion");
        check(sim.contracts_completed() == 1, "base landing completes one contract");
        check(sim.last_completed().has_value(), "a completion record is kept");
        check(sim.last_completed()->completed, "completion record is marked complete");
        check(sim.last_completed()->origin_body == 0 &&
                  sim.last_completed()->destination_body == 1,
              "completion record is the primary -> companion contract");
        check(sim.state().score == score0 + 100 * cf + reward0,
              "score gains the pad bonus and the contract reward");
        // The next contract reverses to the primary base.
        check(sim.contract().origin_body == 1, "next contract from the companion");
        check(sim.contract().destination_body == 0, "next contract to the primary");
        check(
            sim.contract().reward ==
            100 * sim.binary().body(0).terrain.pads().front().multiplier,
            "next reward is the primary base value");
        // Still landed: no second completion, the score stays fixed.
        const int score_after = sim.state().score;
        const int cc_after = sim.contracts_completed();
        for (int i = 0; i < 10; ++i) {
            sim.advance(sim.config().fixed_dt, {});
        }
        check(sim.state().score == score_after, "reward paid exactly once");
        check(sim.contracts_completed() == cc_after, "contract count stable");
    }

    // (b) A safe landing on a non-base pad does not complete the contract.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const auto& cpads = sim.terrain(1).pads();
        check(cpads.size() > 1, "the companion has a non-base pad");
        check(cpads[1].center_arc != 0.0, "the chosen pad is not the base");
        const int pad_mult = cpads[1].multiplier;
        drop_on(sim, 1, 1);  // a non-base pad on the destination body
        const lander::State& r = sim.state();
        check(r.landed && !r.crashed, "safe landing on a non-base pad");
        check(r.landed_body == 1, "landed on the destination body");
        check(sim.contracts_completed() == 0, "non-base landing does not complete");
        check(!sim.last_completed().has_value(), "no completion record");
        check(sim.contract().destination_body == 1, "contract unchanged");
        check(sim.state().score == 100 * pad_mult,
              "only the pad bonus is scored, no contract reward");
    }

    // (c) The destination alternates body to body across consecutive landings.
    {
        lander::Simulation sim;
        sim.reset(seed);  // primary -> companion
        drop_on(sim, 1, 0);  // complete leg 1 on the companion base
        check(sim.contracts_completed() == 1, "leg 1 completes");
        check(sim.contract().destination_body == 0,
              "after leg 1 the contract targets the primary");
        drop_on(sim, 0, 0);  // complete leg 2 on the primary base
        check(sim.contracts_completed() == 2, "leg 2 completes");
        check(sim.contract().destination_body == 1,
              "the contract alternates back to the companion");
        check(sim.contract().origin_body == 0, "origin back to the primary");
    }
}

void test_target_range_rate() {
    check_close(
        lander::target_range_rate({10.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0},
                                  {0.0, 0.0}),
        -1.0, 1.0e-12, "approaching along the range axis is closing");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {1.0, 0.0}, {0.0, 0.0},
                                  {0.0, 0.0}),
        1.0, 1.0e-12, "receding along the range axis is opening");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {0.0, 0.0}, {0.0, 0.0},
                                  {0.0, 0.0}),
        0.0, 1.0e-12, "no relative motion gives zero range rate");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {-1.0, 0.0}, {0.0, 0.0},
                                  {-1.0, 0.0}),
        0.0, 1.0e-12, "target matching the ship's velocity gives zero");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {0.0, -5.0}, {0.0, 0.0},
                                  {0.0, 0.0}),
        0.0, 1.0e-12, "pure cross-range motion gives zero range rate");
    check_close(
        lander::target_range_rate({0.0, 0.0}, {5.0, 5.0}, {0.0, 0.0},
                                  {0.0, 0.0}),
        0.0, 1.0e-12, "the zero-range guard returns zero");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {-0.3, 0.4}, {0.0, 0.0},
                                  {0.0, 0.0}),
        -0.3, 1.0e-12, "mixed motion projects onto the range axis");
    check_close(
        lander::target_range_rate({10.0, 0.0}, {0.3, 0.4}, {0.0, 0.0},
                                  {0.0, 0.0}),
        0.3, 1.0e-12, "outward mixed motion projects positively");
}

// M05-R3-04: the reaction-wheel control applies finite angular damping
// opposite the current spin. It tapers to zero, never zeroes the state
// directly, and never changes translation, fuel, or the un-damped control
// trajectory.
void test_reaction_wheel_damping() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 101;

    auto run = [&](bool active, double omega0) {
        lander::Simulation sim;
        sim.reset(seed);
        lander::State s = state_at(0.0, 1000.0, 5.0, -2.0, 0.3, omega0);
        sim.set_state(s);
        lander::Input in{};
        in.reaction_wheels = active;
        sim.advance(dt, in);
        return sim.state();
    };

    for (double omega0 : {0.5, 0.1, -0.4, 0.0}) {
        const lander::State with = run(true, omega0);
        const lander::State without = run(false, omega0);

        check_close(with.x, without.x, 1.0e-12,
                    "reaction wheels do not change the x trajectory");
        check_close(with.y, without.y, 1.0e-12,
                    "reaction wheels do not change the y trajectory");
        check_close(with.vx, without.vx, 1.0e-12,
                    "reaction wheels do not change the x velocity");
        check_close(with.vy, without.vy, 1.0e-12,
                    "reaction wheels do not change the y velocity");
        check_close(with.fuel, without.fuel, 1.0e-12,
                    "reaction wheels do not burn fuel");
        check_close(without.omega, omega0, 1.0e-12,
                    "without reaction wheels the spin is unchanged");

        const double taper =
            std::clamp(std::abs(omega0) / cfg.reaction_wheel_taper, 0.0, 1.0);
        const double expected =
            omega0 - std::copysign(cfg.reaction_wheel_accel * taper * dt,
                                   omega0);
        check_close(with.omega, expected, 1.0e-12,
                    "reaction-wheel damping follows the tapered formula");
        if (omega0 > 0.0) {
            check(with.omega < without.omega,
                  "positive spin is damped toward zero");
        } else if (omega0 < 0.0) {
            check(with.omega > without.omega,
                  "negative spin is damped toward zero");
        }
    }
}

// M05-R3-V05 / V17: navigation_cues gives the contract-destination pad
// direction, velocity relative to the moving pad (the pad is a point fixed
// on the rotating surface), and the true per-body / net gravity.
void test_navigation_cues() {
    lander::Config cfg{};
    const auto bin =
        lander::BinarySystem::canonical(cfg.mu, 301ULL,
                                        lander::companion_seed(301ULL));
    const double t = 3.7;
    const lander::Body& destination = bin.body(1);
    const lander::Pad& pad = destination.terrain.pads().front();
    // The destination pad is fixed on the rotating surface: its world
    // position and velocity come from the rotating-surface point.
    const lander::BinarySystem::SurfacePoint dest_pad = bin.surface_point(
        1, destination.terrain.angle_at_arc(pad.center_arc),
        destination.terrain.surface_radius_at_arc(pad.center_arc), t);
    const double theta =
        destination.terrain.angle_at_arc(pad.center_arc) +
        bin.body_rotation(t);
    const lander::Vec2 up{std::cos(theta), std::sin(theta)};
    const lander::Vec2 right{std::sin(theta), -std::cos(theta)};
    const lander::Vec2 target = dest_pad.position;

    lander::State s{};
    s.x = target.x + 100.0 * up.x;
    s.y = target.y + 100.0 * up.y;
    s.vx = dest_pad.velocity.x + 2.0 * up.x + 3.0 * right.x;
    s.vy = dest_pad.velocity.y + 2.0 * up.y + 3.0 * right.y;
    s.fuel = 1000.0;

    const lander::NavCues cues =
        lander::navigation_cues(s, bin, t, 1);

    check_close(cues.target_position.x, target.x, 1.0e-9,
                "nav target is the destination base pad x");
    check_close(cues.target_position.y, target.y, 1.0e-9,
                "nav target is the destination base pad y");
    check_close(cues.target_distance, 100.0, 1.0e-9, "nav target distance");
    check_close(cues.target_direction.x, -up.x, 1.0e-9,
                "nav target direction points from the ship toward the pad x");
    check_close(cues.target_direction.y, -up.y, 1.0e-9,
                "nav target direction points from the ship toward the pad y");
    check_close(cues.relative_velocity.x, 2.0 * up.x + 3.0 * right.x, 1.0e-9,
                "relative velocity is ship velocity minus pad velocity x");
    check_close(cues.relative_velocity.y, 2.0 * up.y + 3.0 * right.y, 1.0e-9,
                "relative velocity is ship velocity minus pad velocity y");
    check_close(cues.relative_speed, std::hypot(2.0, 3.0), 1.0e-9,
                "relative speed magnitude");

    const lander::Vec2 pos{s.x, s.y};
    check_close(cues.gravity_primary.x,
                bin.gravity_from(0, pos, t).x, 1.0e-12,
                "nav primary gravity is the primary inverse-square field x");
    check_close(cues.gravity_primary.y,
                bin.gravity_from(0, pos, t).y, 1.0e-12,
                "nav primary gravity is the primary inverse-square field y");
    check_close(cues.gravity_companion.x,
                bin.gravity_from(1, pos, t).x, 1.0e-12,
                "nav companion gravity is the companion inverse-square x");
    check_close(cues.gravity_companion.y,
                bin.gravity_from(1, pos, t).y, 1.0e-12,
                "nav companion gravity is the companion inverse-square y");
    const lander::Vec2 total = bin.gravity(pos, t);
    check_close(cues.net_gravity.x, total.x, 1.0e-12,
                "nav net gravity equals the total field x");
    check_close(cues.net_gravity.y, total.y, 1.0e-12,
                "nav net gravity equals the total field y");
    check_close(cues.g_primary,
                std::hypot(cues.gravity_primary.x, cues.gravity_primary.y),
                1.0e-12, "primary gravity magnitude");
    check_close(cues.g_companion,
                std::hypot(cues.gravity_companion.x,
                           cues.gravity_companion.y),
                1.0e-12, "companion gravity magnitude");
    check_close(cues.g_net, std::hypot(cues.net_gravity.x, cues.net_gravity.y),
                1.0e-12, "net gravity magnitude");

    lander::State at_target{};
    at_target.x = target.x;
    at_target.y = target.y;
    at_target.vx = dest_pad.velocity.x;
    at_target.vy = dest_pad.velocity.y;
    at_target.fuel = 1.0;
    const lander::NavCues zero = lander::navigation_cues(at_target, bin, t, 1);
    check_close(zero.target_distance, 0.0, 1.0e-12,
                "zero range gives zero target distance");
    check_close(zero.target_direction.x, 0.0, 1.0e-12,
                "zero range gives a zero target direction x");
    check_close(zero.target_direction.y, 0.0, 1.0e-12,
                "zero range gives a zero target direction y");
    check_close(zero.relative_speed, 0.0, 1.0e-12,
                "co-moving with the pad gives zero relative speed");
}

// M05-R3-V03: the explicit HUD helpers produce the readout values and the
// CLOSE / OPEN / HOLD labels.
void test_hud_helper_readouts() {
    lander::State s{};
    s.omega = 1.0;
    check_close(lander::spin_deg_per_s(s), 180.0 / lander::kPi, 1.0e-12,
                "SPIN converts rad/s to deg/s");
    s.omega = -2.0;
    check_close(lander::spin_deg_per_s(s), -360.0 / lander::kPi, 1.0e-12,
                "SPIN preserves the spin direction");

    lander::State orb{};
    orb.x = 10.0;
    orb.y = 0.0;
    orb.vx = 0.0;
    orb.vy = 4.0;
    check_close(lander::orbital_rate(orb, {0.0, 0.0}, {0.0, 0.0}), -0.4,
                1.0e-12, "ORB is body-relative tangential velocity / radius");
    check_close(
        lander::orbital_rate(orb, {0.0, 0.0}, {0.0, 0.0}),
        lander::local_angular_velocity(orb, {0.0, 0.0}, {0.0, 0.0}), 1.0e-12,
        "ORB matches the local angular-rate definition");

    check(std::strcmp(lander::range_rate_label(-0.1), "CLOSE") == 0,
          "a negative range rate labels CLOSE");
    check(std::strcmp(lander::range_rate_label(0.1), "OPEN") == 0,
          "a positive range rate labels OPEN");
    check(std::strcmp(lander::range_rate_label(0.0), "HOLD") == 0,
          "zero range rate labels HOLD");
    check(std::strcmp(lander::range_rate_label(-0.05), "CLOSE") == 0,
          "the negative tolerance boundary labels CLOSE");
    check(std::strcmp(lander::range_rate_label(0.05), "OPEN") == 0,
          "the positive tolerance boundary labels OPEN");
    check(std::strcmp(lander::range_rate_label(0.049), "HOLD") == 0,
          "inside the tolerance labels HOLD");
}

// M05-R3-V06: O and Shift+O call the same one-time circularize with explicit
// clockwise / counter-clockwise tangential directions.
void test_circularize_directions() {
    lander::Config cfg{};
    const std::uint64_t seed = 81;

    lander::Simulation sim;
    sim.reset(seed);
    const double r0 = sim.terrain(0).max_surface_radius() + 20.0;
    const double alt = r0 - sim.terrain(0).surface_radius_at_arc(0.0);
    sim.set_state(state_relative(sim.binary(), 0, 0.0, alt, 0.0, 0.0, 0.0));

    const lander::Vec2 bpos = sim.binary().position(0, 0.0);
    const lander::Vec2 bvel = sim.binary().velocity(0, 0.0);
    const double r = lander::radial_distance(sim.state(), bpos);
    const double theta =
        std::atan2(sim.state().y - bpos.y, sim.state().x - bpos.x);
    const double speed = std::sqrt(sim.binary().body(0).mu / r);
    const lander::Vec2 right{std::sin(theta), -std::cos(theta)};

    sim.circularize(false);
    check_close(sim.state().vx, bvel.x + right.x * speed, 1.0e-6,
                "explicit clockwise circularize uses the local right vector");
    check_close(sim.state().vy, bvel.y + right.y * speed, 1.0e-6,
                "explicit clockwise circularize uses the local right vector y");
    check_close(sim.state().x, bpos.x + std::cos(theta) * r, 1.0e-6,
                "clockwise circularize does not move the ship x");
    check_close(sim.state().y, bpos.y + std::sin(theta) * r, 1.0e-6,
                "clockwise circularize does not move the ship y");
    check_close(sim.state().fuel, 1000.0, 1.0e-9,
                "clockwise circularize does not burn fuel");

    sim.set_state(state_relative(sim.binary(), 0, 0.0, alt, 0.0, 0.0, 0.0));
    sim.circularize(true);
    check_close(sim.state().vx, bvel.x - right.x * speed, 1.0e-6,
                "explicit counter-clockwise circularize reverses tangential x");
    check_close(sim.state().vy, bvel.y - right.y * speed, 1.0e-6,
                "explicit counter-clockwise circularize reverses tangential y");
    check_close(sim.state().angle, 0.0, 1.0e-9,
                "counter-clockwise circularize does not change attitude");

    lander::Simulation landed;
    landed.reset(seed);
    const lander::State before = landed.state();
    landed.circularize(false);
    landed.circularize(true);
    check(landed.state() == before,
          "both explicit circularize directions are no-ops while landed");
}

// M05-R3-15: sync_orbit places the ship in a body-synchronous circular orbit
// on the far side of the source body (the landed body when landed, otherwise
// the current reference body): the exact position/velocity of the
// synchronous-radius formula, co-rotating with the binary, zero spin, nose
// radially out, fuel / score / ticks / phase preserved, no-op while crashed,
// deterministic per seed.
void test_sync_orbit_state() {
    lander::Config cfg{};
    const std::uint64_t seed = 501;

    // (a) Landed on the primary: the source is the landed body.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const int source = 0;
        const int other = 1;
        const double t = 0.0;
        const lander::Vec2 spos = sim.binary().position(source, t);
        const lander::Vec2 svel = sim.binary().velocity(source, t);
        const lander::Vec2 opos = sim.binary().position(other, t);
        const double away = std::hypot(spos.x - opos.x, spos.y - opos.y);
        const lander::Vec2 dir{(spos.x - opos.x) / away,
                               (spos.y - opos.y) / away};
        const double omega = sim.binary().omega();
        const double r =
            std::cbrt(sim.binary().body(source).mu / (omega * omega));
        check(r > 500.0 && r < 700.0,
              "the primary synchronous radius is in the expected band");

        sim.sync_orbit();
        const lander::State& s = sim.state();
        check(!s.landed && !s.crashed, "sync_orbit places the ship in flight");
        check(s.landed_body == -1, "the synced ship is not attached");
        check_close(s.x, spos.x + dir.x * r, 1e-9, "sync orbit position x");
        check_close(s.y, spos.y + dir.y * r, 1e-9, "sync orbit position y");
        check_close(s.vx, svel.x - omega * (s.y - spos.y), 1e-9,
                    "sync orbit co-rotates with the binary vx");
        check_close(s.vy, svel.y + omega * (s.x - spos.x), 1e-9,
                    "sync orbit co-rotates with the binary vy");
        check_close(norm_angle(s.angle), norm_angle(std::atan2(dir.y, dir.x)),
                    1e-9, "the nose points radially out, away from the other body");
        check_close(s.omega, 0.0, 1e-12, "the synced spin is zero");
        check_close(s.fuel, cfg.fuel, 1e-9, "sync_orbit preserves the fuel");
        check(s.ticks == 0 && s.score == 0,
              "sync_orbit preserves ticks and score");
        const double toward =
            (s.x - spos.x) * (opos.x - spos.x) +
            (s.y - spos.y) * (opos.y - spos.y);
        check(toward < 0.0,
              "the synced ship starts on the far side of the binary");
        const lander::LocalVelocity lv = lander::local_velocity(s, spos, svel);
        check_close(lv.radial, 0.0, 1e-9,
                    "the synced orbit has zero relative radial speed");
        check_close(lv.tangential, -omega * r, 1e-9,
                    "the synced orbit carries the binary angular velocity");
    }

    // (b) Landed on the companion: the source is the companion.
    {
        lander::Simulation sim;
        sim.reset(seed);
        check(drop_on(sim, 1, 0) >= 0, "the companion probe lands");
        check(sim.state().landed, "the companion probe is landed");
        const int source = 1;
        const int other = 0;
        const double t = sim.sim_time();
        const lander::Vec2 spos = sim.binary().position(source, t);
        const lander::Vec2 svel = sim.binary().velocity(source, t);
        const lander::Vec2 opos = sim.binary().position(other, t);
        const double away = std::hypot(spos.x - opos.x, spos.y - opos.y);
        const lander::Vec2 dir{(spos.x - opos.x) / away,
                               (spos.y - opos.y) / away};
        const double omega = sim.binary().omega();
        const double r =
            std::cbrt(sim.binary().body(source).mu / (omega * omega));
        check(r > 100.0 && r < 200.0,
              "the companion synchronous radius is in the expected band");

        sim.sync_orbit();
        const lander::State& s = sim.state();
        check(!s.landed && !s.crashed,
              "companion sync_orbit places the ship in flight");
        check_close(s.x, spos.x + dir.x * r, 1e-9,
                    "companion sync orbit position x");
        check_close(s.y, spos.y + dir.y * r, 1e-9,
                    "companion sync orbit position y");
        check_close(s.vx, svel.x - omega * (s.y - spos.y), 1e-9,
                    "companion sync orbit co-rotates vx");
        check_close(s.vy, svel.y + omega * (s.x - spos.x), 1e-9,
                    "companion sync orbit co-rotates vy");
        const lander::LocalVelocity lv = lander::local_velocity(s, spos, svel);
        check_close(lv.radial, 0.0, 1e-9,
                    "the companion synced orbit has zero relative radial speed");
        check_close(lv.tangential, -omega * r, 1e-9,
                    "the companion synced orbit carries the binary angular velocity");
    }

    // (c) Flying near the companion: the reference body is the source.
    {
        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(state_relative(sim.binary(), 1, 0.0, 8.0, 0.0, 0.0, 0.0));
        sim.advance(cfg.fixed_dt, {});
        check(sim.reference_body() == 1, "the reference is the companion");
        sim.sync_orbit();
        const int source = 1;
        const int other = 0;
        const double t = sim.sim_time();
        const lander::Vec2 spos = sim.binary().position(source, t);
        const lander::Vec2 svel = sim.binary().velocity(source, t);
        const lander::Vec2 opos = sim.binary().position(other, t);
        const double away = std::hypot(spos.x - opos.x, spos.y - opos.y);
        const lander::Vec2 dir{(spos.x - opos.x) / away,
                               (spos.y - opos.y) / away};
        const double omega = sim.binary().omega();
        const double r =
            std::cbrt(sim.binary().body(source).mu / (omega * omega));
        const lander::State& s = sim.state();
        check(!s.landed && !s.crashed,
              "in-flight sync_orbit places the ship in flight");
        check_close(s.x, spos.x + dir.x * r, 1e-9,
                    "reference-source sync orbit position x");
        check_close(s.y, spos.y + dir.y * r, 1e-9,
                    "reference-source sync orbit position y");
        check_close(s.vx, svel.x - omega * (s.y - spos.y), 1e-9,
                    "reference-source sync orbit co-rotates vx");
        check_close(s.vy, svel.y + omega * (s.x - spos.x), 1e-9,
                    "reference-source sync orbit co-rotates vy");
    }

    // (d) No-op while crashed.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const double arc = 0.5 * sim.terrain(0).circumference();
        sim.set_state(
            state_relative(sim.binary(), 0, arc, 0.3, -30.0, 0.0, 0.0));
        run_to_contact(sim);
        check(sim.state().crashed, "crashed for the sync-orbit no-op test");
        const lander::State before = sim.state();
        sim.sync_orbit();
        check(sim.state() == before, "sync_orbit is a no-op while crashed");
    }

    // (e) Deterministic for a fixed seed.
    {
        auto run = [&]() {
            lander::Simulation sim;
            sim.reset(seed);
            sim.sync_orbit();
            return sim.state();
        };
        check(run() == run(), "sync_orbit is deterministic for a fixed seed");
    }
}

// M05-R3-15: the primary-sourced synced orbit stays visually stable over two
// binary periods (no crash, no decay, radius stays near the synchronous
// radius). The companion-sourced orbit drifts outward over the same span
// (gravity there is weaker than the co-rotation requirement) but still does
// not crash.
void test_sync_orbit_stability() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 502;

    {
        lander::Simulation sim;
        sim.reset(seed);
        sim.sync_orbit();  // landed on the primary at reset: source 0
        const double omega = sim.binary().omega();
        const double r0 =
            std::cbrt(sim.binary().body(0).mu / (omega * omega));
        check_close(lander::radial_distance(
                        sim.state(), sim.binary().position(0, sim.sim_time())),
                    r0, 1e-9, "the synced orbit starts at the synchronous radius");
        const int steps =
            static_cast<int>(std::lround(2.0 * sim.binary().period() / dt));
        double rmin = 1e300, rmax = -1e300;
        for (int i = 0; i < steps && !sim.state().crashed; ++i) {
            sim.advance(dt, {});
            const double r = lander::radial_distance(
                sim.state(), sim.binary().position(0, sim.sim_time()));
            rmin = std::min(rmin, r);
            rmax = std::max(rmax, r);
        }
        check(!sim.state().crashed, "the primary synced orbit does not crash");
        check(rmin > 0.7 * r0, "the primary synced orbit does not decay");
        check(rmax < 1.3 * r0,
              "the primary synced orbit stays near the synchronous radius");
    }

    {
        lander::Simulation sim;
        sim.reset(seed);
        check(drop_on(sim, 1, 0) >= 0, "the companion stability probe lands");
        sim.sync_orbit();  // landed on the companion: source 1
        const int steps =
            static_cast<int>(std::lround(2.0 * sim.binary().period() / dt));
        for (int i = 0; i < steps && !sim.state().crashed; ++i) {
            sim.advance(dt, {});
        }
        check(!sim.state().crashed,
              "the companion synced orbit does not crash despite the drift");
    }
}

// M05-R3-20: velocity-only transfer. T x3 changes ONLY the ship's velocity;
// its position stays bit-identical. The departure point is the current world
// position (the exact surface point when landed, the current flight point
// when flying). SOURCE = landed body (if landed) else the reference body;
// TARGET = the other body. A solution replaces the velocity with a real
// two-body ballistic arc that reaches the target's clearance shell and stays
// outside terrain; a no-solution leaves the state bit-identical (no teleport).
// Fuel / score / ticks / phase preserved; deterministic per seed.
void test_transfer() {
    lander::Config cfg{};
    const double dt = cfg.fixed_dt;
    const std::uint64_t seed = 503;

    // Mirrors the solver's propagator: the same semi-implicit Euler with the
    // two-body field sampled at each step's start; no thrust / spin / fuel /
    // collision.
    auto propagate = [&](const lander::BinarySystem& bin, double x, double y,
                         double vx, double vy, double t, int steps) {
        for (int i = 0; i < steps; ++i) {
            const lander::Vec2 a = bin.gravity({x, y}, t);
            vx += a.x * dt;
            vy += a.y * dt;
            x += vx * dt;
            y += vy * dt;
            t += dt;
        }
        return lander::Vec2{x, y};
    };

    // Re-search the solver's candidate flight times for the placed state and
    // return the candidate whose terminal point is closest to the target's
    // arrival shell: the clearance shell above the target's surface on the
    // approach side. The solver accepts a miss up to 8 m at a point on that
    // shell, so a match within that bound confirms a real ballistic arc.
    auto best_fraction = [&](const lander::Simulation& sim, int source,
                             int target) {
        const double t0 = sim.sim_time();
        const lander::State& s = sim.state();
        double best = -1.0;
        double best_err = 1e30;
        for (double fraction : {0.15, 0.20, 0.25, 0.30, 0.40, 0.50}) {
            const int steps =
                static_cast<int>(std::lround(fraction *
                                             sim.binary().period() / dt));
            if (steps < 10) {
                continue;
            }
            const double t1 = t0 + steps * dt;
            const lander::Vec2 g1 = sim.binary().position(target, t1);
            const lander::Vec2 s1 = sim.binary().position(source, t1);
            const double dg = std::hypot(s1.x - g1.x, s1.y - g1.y);
            if (dg < 1.0e-9) {
                continue;
            }
            const lander::Vec2 approach{(s1.x - g1.x) / dg,
                                        (s1.y - g1.y) / dg};
            const double r_arr =
                sim.binary().body(target).terrain.max_surface_radius() + 15.0;
            const lander::Vec2 goal{g1.x + approach.x * r_arr,
                                    g1.y + approach.y * r_arr};
            const lander::Vec2 f =
                propagate(sim.binary(), s.x, s.y, s.vx, s.vy, t0, steps);
            const double err = std::hypot(f.x - goal.x, f.y - goal.y);
            if (err < best_err) {
                best_err = err;
                best = fraction;
            }
        }
        return (best < 0.0 || best_err > 8.0) ? -1.0 : best;
    };

    // Verify the velocity-only invariants for a SOLVED transfer: position
    // bit-identical to `before`, velocity replaced, unlanded / non-attached,
    // fuel / score / ticks / phase preserved, spin zero, nose along the
    // velocity, speed bounded, and the arc reaches the target's clearance
    // shell on the approach side and stays outside the target's worst-case
    // surface at every half-second sample.
    auto check_solved = [&](const lander::Simulation& sim, int source,
                            const lander::State& before, double t_before) {
        const int target = 1 - source;
        const lander::State& s = sim.state();
        check(!s.landed && !s.crashed, "the placed ship is in flight");
        check(s.landed_body == -1, "the placed ship is not attached");
        check(s.x == before.x && s.y == before.y,
              "the transfer leaves the ship's position bit-identical");
        check(s.vx != before.vx || s.vy != before.vy,
              "the transfer replaced the ship's velocity");
        check_close(s.fuel, before.fuel, 1e-12, "the transfer preserves the fuel");
        check(s.ticks == before.ticks, "the transfer preserves the tick count");
        check(s.score == before.score, "the transfer preserves the score");
        check_close(sim.sim_time(), t_before, 1e-12,
                    "the transfer preserves the phase clock");
        check_close(s.omega, 0.0, 1e-12, "the placed spin is zero");
        const double speed = std::hypot(s.vx, s.vy);
        check(speed > 1e-9 && speed <= 60.0,
              "the launch speed is nonzero and within the plausibility bound");
        check_close(s.angle, norm_angle(std::atan2(s.vy, s.vx)), 1e-9,
                    "the nose points along the launch velocity");

        // The arc really reaches the target's clearance shell under gravity
        // alone, arrives on the approach side, and stays outside the target's
        // worst-case surface at every half-second sample (source clearance is
        // the solver's own arc-clear's job; the craft legitimately begins on
        // the source surface, so the source is not part of this check).
        const double t0 = sim.sim_time();
        const double fraction = best_fraction(sim, source, target);
        check(fraction > 0.0,
              "the placed arc reaches the target clearance shell");
        if (fraction > 0.0) {
            const int steps =
                static_cast<int>(std::lround(fraction *
                                             sim.binary().period() / dt));
            const double t1 = t0 + steps * dt;
            const lander::Vec2 g1 = sim.binary().position(target, t1);
            const lander::Vec2 s1 = sim.binary().position(source, t1);
            const lander::Vec2 f =
                propagate(sim.binary(), s.x, s.y, s.vx, s.vy, t0, steps);
            check((f.x - g1.x) * (s1.x - g1.x) + (f.y - g1.y) * (s1.y - g1.y) >
                      0.0,
                  "the arrival is on the approach side of the target");
            double px = s.x, py = s.y, vx = s.vx, vy = s.vy, t = t0;
            bool clear = true;
            for (int i = 0; i < steps; ++i) {
                if (i % 60 == 0) {
                    const lander::Vec2 bp = sim.binary().position(target, t);
                    if (std::hypot(px - bp.x, py - bp.y) <
                            sim.binary().body(target).terrain.max_surface_radius()) {
                        clear = false;
                    }
                }
                const lander::Vec2 a = sim.binary().gravity({px, py}, t);
                vx += a.x * dt;
                vy += a.y * dt;
                px += vx * dt;
                py += vy * dt;
                t += dt;
            }
            check(clear,
                  "the approach stays outside the target's worst-case surface");
        }
    };

    // A no-solution leaves the state bit-identical (no teleport, M05-R3-20).
    auto check_nosolution = [&](const lander::Simulation& sim,
                                const lander::State& before) {
        check(sim.state() == before,
              "the no-solution transfer leaves the state untouched");
    };

    // (1) Landed primary -> companion: the guaranteed-positive case. Must
    //     produce a real arc and stay fast enough not to stall the game loop.
    {
        lander::Simulation sim;
        sim.reset(seed);
        check(sim.state().landed && sim.state().landed_body == 0,
              "the probe is landed on the primary");
        const lander::State before = sim.state();
        const double t_before = sim.sim_time();
        const auto clk0 = std::chrono::steady_clock::now();
        check(sim.transfer(), "the primary-source transfer found a plausible arc");
        const auto clk1 = std::chrono::steady_clock::now();
        const long ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(clk1 - clk0)
                .count();
        check(ms < 200,
              "the primary-source transfer solve stays under the loop stall bound");
        if (sim.state().landed_body == -1 && !sim.state().landed) {
            check_solved(sim, 0, before, t_before);
        }
    }

    // (2) Landed companion -> primary: a real arc when one exists, otherwise a
    //     no-solution that leaves the state bit-identical.
    {
        lander::Simulation sim;
        sim.reset(seed);
        check(drop_on(sim, 1, 0) >= 0,
              "the companion transfer probe lands first");
        check(sim.state().landed && sim.state().landed_body == 1,
              "the probe is landed on the companion");
        const lander::State before = sim.state();
        const double t_before = sim.sim_time();
        if (sim.transfer()) {
            check_solved(sim, 1, before, t_before);
        } else {
            check_nosolution(sim, before);
        }
    }

    // (3) Flying reference-source: the position is left bit-identical and
    //     either the velocity is replaced (a real arc) or the state is
    //     untouched (no-solution).
    {
        lander::Simulation sim;
        sim.reset(seed);
        sim.set_state(
            state_relative(sim.binary(), 0, 0.0, 100.0, 0.0, 0.0, 0.0));
        check(sim.reference_body() == 0, "the reference is the primary");
        check(!sim.state().landed, "the probe is in flight");
        const lander::State before = sim.state();
        const double t_before = sim.sim_time();
        if (sim.transfer()) {
            const lander::State& s = sim.state();
            check(s.x == before.x && s.y == before.y,
                  "the flying transfer leaves the position bit-identical");
            check(s.vx != before.vx || s.vy != before.vy,
                  "the flying transfer replaced the velocity");
            check_close(s.fuel, before.fuel, 1e-9,
                        "the flying transfer preserves the fuel");
            check(!s.landed && !s.crashed, "the flying-source ship is in flight");
            check_close(sim.sim_time(), t_before, 1e-12,
                        "the flying transfer preserves the phase clock");
        } else {
            check_nosolution(sim, before);
        }
    }

    // No-op while crashed: reports no solution and leaves the state
    // untouched.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const double arc = 0.5 * sim.terrain(0).circumference();
        sim.set_state(
            state_relative(sim.binary(), 0, arc, 0.3, -30.0, 0.0, 0.0));
        run_to_contact(sim);
        check(sim.state().crashed, "crashed for the transfer no-op test");
        const lander::State before = sim.state();
        check(!sim.transfer(), "transfer() reports no solution while crashed");
        check(sim.state() == before,
              "transfer() leaves the crashed state untouched");
    }

    // Deterministic for a fixed seed, in both directions.
    {
        auto run = [&](int source) {
            lander::Simulation sim;
            sim.reset(seed);
            if (source == 1) {
                drop_on(sim, 1, 0);
            }
            sim.transfer();
            return sim.state();
        };
        check(run(0) == run(0), "the primary-source transfer is deterministic");
        check(run(1) == run(1), "the companion-source transfer is deterministic");
    }
}

}  // namespace

// M05-R3-V14: a landed ship stays attached to a rotating surface point.
void test_landed_attachment_rotating() {
    lander::Config cfg{};
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 411ULL, lander::companion_seed(411ULL));

    for (int i = 0; i < 2; ++i) {
        const lander::Body& body = bin.body(i);
        const double c = body.terrain.circumference();
        for (double arc : {0.0, 0.25 * c, 0.6 * c, 0.95 * c}) {
            const double local = body.terrain.angle_at_arc(arc);
            const double radius =
                body.terrain.surface_radius_at_arc(arc);
            for (double t : {0.0, 3.0, 24.0, bin.period() * 0.5,
                             bin.period()}) {
                const lander::State s =
                    lander::attached_state(bin, i, arc, t);
                const lander::Vec2 cpos = bin.position(i, t);
                const double world = local + bin.body_rotation(t);
                check_close(s.x, cpos.x + std::cos(world) * radius, 1.0e-9,
                            "attached position rides the rotating surface x");
                check_close(s.y, cpos.y + std::sin(world) * radius, 1.0e-9,
                            "attached position rides the rotating surface y");
                const lander::Vec2 sp_vel =
                    bin.surface_point(i, local, radius, t).velocity;
                check_close(s.vx, sp_vel.x, 1.0e-9,
                            "attached velocity is the surface-point velocity x");
                check_close(s.vy, sp_vel.y, 1.0e-9,
                            "attached velocity is the surface-point velocity y");
                check_close(lander::radial_distance(s, cpos), radius, 1.0e-9,
                            "attached ship stays at the surface radius");
                check_close(s.landed_arc, arc, 1.0e-12,
                            "the landed arc is unchanged");
                // The nose points along the local radial (body-local up).
                const double diff = std::atan2(
                    std::sin(s.angle - (world - 0.5 * lander::kPi)),
                    std::cos(s.angle - (world - 0.5 * lander::kPi)));
                check_close(diff, 0.0, 1.0e-9,
                            "the nose points along the local radial");
            }
        }
    }
}

// M05-R3-V15: a takeoff releases the ship with the full surface-point
// velocity (centre plus spin), not just the centre velocity.
void test_takeoff_inherits_surface_velocity() {
    lander::Config cfg{};
    const std::uint64_t seed = 412;
    const double dt = cfg.fixed_dt;

    lander::Simulation sim;
    sim.reset(seed);
    const lander::State on_ground = sim.state();
    const int body = on_ground.landed_body;
    const lander::Body& b = sim.binary().body(body);
    const double local = b.terrain.angle_at_arc(on_ground.landed_arc);
    const double radius =
        b.terrain.surface_radius_at_arc(on_ground.landed_arc);
    const lander::Vec2 sp_vel =
        sim.binary().surface_point(body, local, radius, 0.0).velocity;
    const lander::Vec2 cvel = sim.binary().velocity(body, 0.0);
    check(std::hypot(sp_vel.x - cvel.x, sp_vel.y - cvel.y) > 1.0,
          "the surface-point velocity is not the centre velocity alone");

    // A manually released ship (same on-pad position, surface-point
    // velocity) and the thrusting lander must produce identical states:
    // the takeoff inherits the release velocity exactly.
    lander::Simulation manual;
    manual.reset(seed);
    lander::State m = manual.state();
    // Mirror the release exactly: try_takeoff clears `landed` but keeps
    // `landed_body` / `landed_arc` as they were on the pad.
    m.landed = false;
    m.crashed = false;
    m.vx = sp_vel.x;
    m.vy = sp_vel.y;
    m.omega = 0.0;
    manual.set_state(m);

    lander::Input input{};
    input.main_throttle = 1.0;
    for (int i = 0; i < 40; ++i) {
        sim.advance(dt, input);
        manual.advance(dt, input);
    }
    check(!sim.state().crashed, "the takeoff flight does not crash");
    check(!sim.state().landed, "full thrust keeps the ship off the pad");
    check(sim.state() == manual.state(),
          "takeoff trajectory matches a manual release at the "
          "surface-point velocity");
    check(sim.state().x != on_ground.x || sim.state().y != on_ground.y,
          "the ship has actually left the pad");
}

// M05-R3-V16: landing and crash are evaluated against the rotating surface
// point, not the body centre.
void test_landing_vs_rotating_surface() {
    lander::Config cfg{};
    const std::uint64_t seed = 413;

    // (a) A ship that matches the surface point within the safe bands lands.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const int body = 1;  // the companion
        const lander::Body& b = sim.binary().body(body);
        const double pad_arc = b.terrain.pads().front().center_arc;
        sim.set_state(
            state_relative(sim.binary(), body, pad_arc, 0.5, -0.5, 0.0, 0.0));
        check(run_to_contact(sim) >= 0, "a surface-matched ship reaches the pad");
        const lander::State& r = sim.state();
        check(r.landed && !r.crashed, "a surface-matched ship lands safely");
        check(r.landed_body == body, "the ship lands on the intended body");
        check(b.terrain.pad_at_arc(r.landed_arc) != nullptr,
              "the contact arc is on the pad");
        // The post-landing state carries the surface-point velocity.
        const double local = b.terrain.angle_at_arc(r.landed_arc);
        const double radius = b.terrain.surface_radius_at_arc(r.landed_arc);
        const lander::Vec2 sp_vel = sim.binary().surface_point(
            body, local, radius, sim.sim_time()).velocity;
        check_close(r.vx, sp_vel.x, 1.0e-9,
                    "post-landing velocity is the surface-point velocity x");
        check_close(r.vy, sp_vel.y, 1.0e-9,
                    "post-landing velocity is the surface-point velocity y");
    }

    // (b) A ship that matches the body centre but not the spin: its
    // surface-relative tangential speed is the full spin speed (~9.6 m/s
    // on the primary), far outside the safe band, so it crashes even
    // though its radial descent is gentle.
    {
        lander::Simulation sim;
        sim.reset(seed);
        const int body = 0;  // the primary
        const lander::Body& b = sim.binary().body(body);
        const double pad_arc = b.terrain.pads().front().center_arc;
        const lander::Vec2 pos = sim.binary().position(body, 0.0);
        const lander::Vec2 vel = sim.binary().velocity(body, 0.0);
        const double theta = b.terrain.angle_at_arc(pad_arc);
        const double rad = b.terrain.surface_radius_at_arc(pad_arc) + 0.5;
        const double up_x = std::cos(theta);
        const double up_y = std::sin(theta);
        lander::State s = state_at(pos.x + rad * up_x, pos.y + rad * up_y,
                                   vel.x - 0.5 * up_x, vel.y - 0.5 * up_y,
                                   theta - 0.5 * lander::kPi, 0.0);
        sim.set_state(s);
        check(run_to_contact(sim) >= 0, "a centre-matched ship reaches the pad");
        const lander::State& r = sim.state();
        check(r.crashed && !r.landed,
              "a centre-matched ship crashes on the spinning pad");
        check(r.crash_body == body, "the crash is on the intended body");
    }
}

// M05-R3-V17: the contract destination is the moving pad: its target
// position and velocity track the rotating surface point over time.
void test_destination_pad_moving_target() {
    lander::Config cfg{};
    const auto bin = lander::BinarySystem::canonical(
        cfg.mu, 414ULL, lander::companion_seed(414ULL));

    const int dest = 1;
    const lander::Body& body = bin.body(dest);
    const double pad_arc = body.terrain.pads().front().center_arc;
    const double local = body.terrain.angle_at_arc(pad_arc);
    const double radius = body.terrain.surface_radius_at_arc(pad_arc);

    lander::State s{};
    s.x = 0.0;
    s.y = 0.0;
    s.fuel = 1.0;

    const double t1 = 2.5;
    const double t2 = 40.0;
    const lander::NavCues c1 = lander::navigation_cues(s, bin, t1, dest);
    const lander::NavCues c2 = lander::navigation_cues(s, bin, t2, dest);

    // The target position and velocity equal the analytic rotating-surface
    // point of the pad at each time.
    const auto p1 = bin.surface_point(dest, local, radius, t1);
    const auto p2 = bin.surface_point(dest, local, radius, t2);
    check_close(c1.target_position.x, p1.position.x, 1.0e-9,
                "the target is the pad position at t1 x");
    check_close(c1.target_position.y, p1.position.y, 1.0e-9,
                "the target is the pad position at t1 y");
    check_close(c1.target_direction.x,
                (p1.position.x - s.x) / c1.target_distance, 1.0e-9,
                "the target direction points at the moving pad x");
    check_close(c2.target_position.x, p2.position.x, 1.0e-9,
                "the target is the pad position at t2 x");
    check_close(c2.target_position.y, p2.position.y, 1.0e-9,
                "the target is the pad position at t2 y");

    // Equivalently: body centre plus the spin-rotated local offset.
    const double world = local + bin.body_rotation(t1);
    check_close(c1.target_position.x,
                bin.position(dest, t1).x + std::cos(world) * radius, 1.0e-9,
                "the target is the centre plus rotated offset x");
    check_close(c1.target_position.y,
                bin.position(dest, t1).y + std::sin(world) * radius, 1.0e-9,
                "the target is the centre plus rotated offset y");

    // The pad is a genuinely moving target: its world position changes
    // between the two sample times, and the relative velocity uses the
    // pad's spin velocity.
    check(std::hypot(p2.position.x - p1.position.x,
                     p2.position.y - p1.position.y) > 1.0,
          "the pad's world position changes with time");
    const lander::Vec2 rel =
        lander::Vec2{0.0, 0.0} - p1.velocity;  // a stationary ship at the origin
    check_close(c1.relative_velocity.x, rel.x, 1.0e-9,
                "the relative velocity subtracts the pad's velocity x");
    check_close(c1.relative_velocity.y, rel.y, 1.0e-9,
                "the relative velocity subtracts the pad's velocity y");
}

// M05-R3-V18: determinism with rotating bodies: identical seeds and input
// sequences produce identical trajectories and terminal states (including
// landings on rotating surfaces and takeoffs), and a reset restores the
// same state and rotation phase.
void test_rotating_determinism() {
    lander::Config cfg{};
    const std::uint64_t seed = 415;
    const double dt = cfg.fixed_dt;

    auto run_once = [&]() {
        lander::Simulation sim;
        sim.reset(seed);
        // Fly in and land on the companion's rotating pad.
        const int body = 1;
        const lander::Body& b = sim.binary().body(body);
        const double pad_arc = b.terrain.pads().front().center_arc;
        sim.set_state(state_relative(sim.binary(), body, pad_arc, 0.3, -1.0,
                                     0.2, 0.05));
        for (int i = 0; i < 200 && !sim.state().landed &&
                            !sim.state().crashed;
             ++i) {
            sim.advance(dt, {});
        }
        check(sim.state().landed,
              "the determinism probe lands on the rotating companion");
        // Take off again with full thrust.
        lander::Input up{};
        up.main_throttle = 1.0;
        for (int i = 0; i < 100; ++i) {
            sim.advance(dt, up);
        }
        return sim;
    };

    lander::Simulation a = run_once();
    lander::Simulation b = run_once();
    check(a.state() == b.state(),
          "two same-seed runs produce identical states");
    check(a.binary().body_rotation(a.sim_time()) ==
              b.binary().body_rotation(b.sim_time()),
          "both runs share the same rotation phase");
    check(a.sim_time() == b.sim_time(), "both runs share the same time");

    a.reset(seed);
    b.reset(seed);
    check(a.state() == b.state(),
          "a reset restores the same state and rotation phase");
    check(a.binary().body_rotation(a.sim_time()) == 0.0,
          "the reset rotation phase is zero");
}

// M05-R4-03 / M05-R5: the reaction-wheel control is a discrete GUI toggle
// plus an optional transient `Shift+E` hold, composed into the simulation
// input. The test covers the pure rules (discrete presses, no autorepeat,
// Shift-qualified presses, hold arming, manual-rotation priority, inactive
// context, and reset) so the GUI can rely on one small helper rather than
// duplicating the composition.
void test_reaction_wheel_toggle() {
    lander::ReactionWheelToggle rw;
    check(!rw.enabled(), "the reaction-wheel toggle starts OFF");
    check(!rw.input(false), "an OFF reaction-wheel toggle produces no input");

    check(!rw.press(false, true),
          "a Shift+E press does not turn the stored reaction-wheel toggle ON");
    check(!rw.enabled(), "a Shift+E press leaves the stored toggle OFF");
    check(rw.input(false, true, true),
          "a Shift+E hold arms reaction-wheel input while the toggle is OFF");
    check(!rw.input(true, true, true),
          "manual rotation suppresses the Shift+E hold input for that step");
    check(!rw.enabled(),
          "a Shift+E hold does not switch the stored reaction-wheel toggle ON");
    check(!rw.input(false, true, false),
          "releasing the Shift+E hold with the toggle OFF produces no input");

    rw.press();
    check(rw.enabled(), "a non-repeat E press turns the reaction wheels ON");
    check(rw.input(false),
          "an ON reaction-wheel toggle enables damping without rotation");

    check(!rw.input(true),
          "manual rotation suppresses the damping input while the toggle is ON");
    check(rw.enabled(),
          "manual rotation does not switch the stored reaction-wheel toggle OFF");
    check(rw.input(false),
          "releasing manual rotation resumes damping automatically");

    check(rw.input(false, true, true),
          "the stored ON toggle and a Shift+E hold both arm the same input");
    check(rw.enabled(),
          "a Shift+E hold while the toggle is ON does not toggle it OFF");

    rw.press();
    check(!rw.enabled(), "a second non-repeat E press turns the wheels OFF");
    check(!rw.input(false),
          "an OFF reaction-wheel toggle produces no damping input again");

    check(!rw.press(true, true),
          "a Shift+E autorepeat does not toggle the reaction wheels");
    check(!rw.enabled(), "a Shift+E autorepeat leaves the stored toggle OFF");

    rw.press();
    check(rw.input(false, true),
          "an active ON reaction-wheel toggle produces damping input");
    check(!rw.input(false, false),
          "an inactive context disables the reaction-wheel input without "
          "changing the stored toggle");
    check(!rw.input(false, false, true),
          "an inactive context disables a Shift+E hold without changing the "
          "stored toggle");
    check(rw.enabled(),
          "an inactive context does not clear the stored reaction-wheel "
          "toggle");

    rw.press();  // OFF again
    rw.press(true);
    check(!rw.enabled(), "key autorepeat does not toggle the reaction wheels");

    rw.press();  // ON
    rw.press(true);
    check(rw.enabled(), "key autorepeat while ON does not toggle the wheels");

    rw.reset();
    check(!rw.enabled(), "resetting the reaction-wheel toggle turns it OFF");
    check(!rw.input(false),
          "a reset reaction-wheel toggle produces no damping input");
    check(rw.input(false, true, true),
          "after reset, a Shift+E hold arms input only transiently");
    check(!rw.enabled(),
          "the transient Shift+E hold does not persist in the stored toggle");
    check(!rw.input(false, true, false),
          "releasing the hold after reset produces no damping input");
}

int main() {
    test_reference_values();
    test_terrain();
    test_reset_state();
    test_contract_loop();
    test_local_frame();
    test_one_step_physics();
    test_fixed_step_determinism();
    test_landing_rules();
    test_crash_rules();
    test_landed_attachment_and_takeoff();
    test_terminal_state_is_frozen();
    test_set_state_normalizes();
    test_circularize_state();
    test_circularize_directions();
    test_sync_orbit_state();
    test_sync_orbit_stability();
    test_transfer();
    test_orbit_is_usable();
    test_reference_body_influence();
    test_target_range_rate();
    test_reaction_wheel_damping();
    test_reaction_wheel_toggle();
    test_navigation_cues();
    test_hud_helper_readouts();
    test_landed_attachment_rotating();
    test_takeoff_inherits_surface_velocity();
    test_landing_vs_rotating_surface();
    test_destination_pad_moving_target();
    test_rotating_determinism();
    test_refuel_only_changes_fuel();
    test_flame_animation_continuous();
    test_interpolated_state();

    if (failures == 0) {
        std::puts("All lander_tests passed");
        return 0;
    }
    std::printf("%d lander_tests failed\n", failures);
    return 1;
}
