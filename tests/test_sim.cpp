// M05 simulation tests: the two-body binary system, body-relative navigation,
// landing / crash / takeoff rules, the contract loop, circularize, and the
// presentation-only helpers (interpolation, flame animation).
#include "lander/sim.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

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
// metres above the local surface at body-local `arc`, with optional
// body-relative radial / tangential velocity and an attitude offset from the
// local vertical.
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
    lander::State s{};
    s.x = pos.x + r * up_x;
    s.y = pos.y + r * up_y;
    s.vx = vel.x + radial_v * up_x + tang_v * right_x;
    s.vy = vel.y + radial_v * up_y + tang_v * right_y;
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
            const lander::Vec2 bvel =
                sim.binary().velocity(body, sim.sim_time());
            const lander::LocalVelocity lv = lander::local_velocity(r, bpos, bvel);
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

}  // namespace

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
    test_orbit_is_usable();
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
