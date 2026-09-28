#include "lander/sim.hpp"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) {
    return std::abs(a - b) <= eps;
}

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

double normalize_angle(double angle) {
    angle = std::fmod(angle, 2.0 * lander::kPi);
    if (angle < 0.0) {
        angle += 2.0 * lander::kPi;
    }
    return angle;
}

double normalized_difference(double a, double b) {
    double d = a - b;
    d = std::fmod(d + lander::kPi, 2.0 * lander::kPi);
    if (d < 0.0) {
        d += 2.0 * lander::kPi;
    }
    return d - lander::kPi;
}

lander::State state_at_arc(const lander::Terrain& terrain, double arc,
                           double altitude, double radial_velocity,
                           double tangential_velocity,
                           double local_angle_offset) {
    const double theta = lander::Terrain::angle_at_arc(arc);
    const double r = terrain.surface_radius_at_arc(arc) + altitude;
    const double up_x = std::cos(theta);
    const double up_y = std::sin(theta);
    const double right_x = std::sin(theta);
    const double right_y = -std::cos(theta);

    lander::State s{};
    s.x = r * up_x;
    s.y = r * up_y;
    s.vx = radial_velocity * up_x + tangential_velocity * right_x;
    s.vy = radial_velocity * up_y + tangential_velocity * right_y;
    s.angle = (theta - 0.5 * lander::kPi) + local_angle_offset;
    s.fuel = 100.0;
    return s;
}

double find_non_pad_arc(const lander::Terrain& terrain) {
    const double C = lander::Terrain::circumference();
    for (double arc = 0.0; arc < C; arc += 1.0) {
        if (terrain.pad_at_arc(arc) == nullptr) {
            return arc;
        }
    }
    return 0.0;
}

bool run_to_terminal(lander::Simulation& sim, int max_steps = 24 * 120) {
    for (int i = 0; i < max_steps; ++i) {
        sim.advance(sim.config().fixed_dt, {});
        if (sim.state().landed || sim.state().crashed) {
            return true;
        }
    }
    return sim.state().landed || sim.state().crashed;
}

void test_reference_values() {
    lander::Config config{};
    const double R = lander::Terrain::reference_radius();
    const double v = std::sqrt(config.mu / R);
    const double T = 2.0 * lander::kPi * std::sqrt((R * R * R) / config.mu);
    check_close(v, 23.205, 0.01, "reference circular speed");
    check_close(T, 90.0, 0.01, "reference circular period");
}

void test_terrain_determinism_and_wrap() {
    for (std::uint64_t seed : {1ULL, 2ULL, 3ULL, 1234ULL}) {
        lander::Terrain a(seed);
        lander::Terrain b(seed);
        lander::Terrain c(seed + 1);

        check(!a.pads().empty(), "terrain must have pads");
        check(a.pads() == b.pads(), "same seed must produce same pads");
        check(a.pads() != c.pads(), "different seeds should differ");
        check(a.pad_at_arc(0.0) != nullptr, "spawn arc must be a pad");

        const double C = lander::Terrain::circumference();
        double min_r = 1e30;
        double max_r = -1e30;
        for (int i = 0; i < 2048; ++i) {
            const double arc = i * C / 2048.0;
            const double ra = a.surface_radius_at_arc(arc);
            const double rb = b.surface_radius_at_arc(arc);
            if (ra != rb) {
                check(false, "same seed must produce same surface radius");
                break;
            }
            min_r = std::min(min_r, ra);
            max_r = std::max(max_r, ra);
        }
        check(max_r - min_r > 5.0, "terrain should be visibly uneven");

        check_close(a.base_radius_at_arc(0.0),
                    a.base_radius_at_arc(C), 1e-12,
                    "base radius must be periodic");
        check_close(a.base_radius_at_arc(-1e-6),
                    a.base_radius_at_arc(C - 1e-6), 1e-4,
                    "base radius must wrap smoothly");
        check_close(a.surface_radius_at_arc(0.001),
                    a.surface_radius_at_arc(C - 0.001), 1e-9,
                    "surface radius must wrap smoothly at spawn pad");

        for (const lander::Pad& pad : a.pads()) {
            check(pad.half_width >= 3.0, "pad should be at least 6 m wide");
            for (double off = -pad.half_width; off <= pad.half_width + 1e-9;
                 off += 0.25) {
                const double r =
                    a.surface_radius_at_arc(pad.center_arc + off);
                check_close(r, pad.radius, 1e-12, "pad surface should be flat");
            }
        }

        lander::Simulation sim;
        sim.reset(seed);
        const double alt = lander::altitude_at(sim.terrain(), sim.state());
        check(alt > 0.0 && alt < 50.0, "spawn should start above terrain");
        check_close(alt, 20.0, 1e-9, "spawn should start 20 m above terrain");
    }
}

void test_radial_gravity() {
    lander::Simulation sim;
    sim.reset(7);
    const lander::Config& config = sim.config();
    const double r = sim.terrain().max_surface_radius() + 100.0;

    auto accel_magnitude = [&](double radius, double theta) {
        lander::State s{};
        s.x = std::cos(theta) * radius;
        s.y = std::sin(theta) * radius;
        s.fuel = 100.0;
        sim.set_state(s);
        sim.advance(config.fixed_dt, {});
        return std::hypot(sim.state().vx, sim.state().vy) / config.fixed_dt;
    };

    const double g1 = accel_magnitude(r, 0.0);
    const double g2 = accel_magnitude(2.0 * r, 0.0);
    check_close(g1 / (4.0 * g2), 1.0, 1e-9, "gravity should scale 1/r^2");

    for (double theta : {0.0, 0.3, 1.1, 2.0, 3.0, 4.2, 5.5}) {
        lander::State s{};
        s.x = std::cos(theta) * r;
        s.y = std::sin(theta) * r;
        s.fuel = 100.0;
        sim.set_state(s);
        sim.advance(config.fixed_dt, {});
        const double dvx = sim.state().vx / config.fixed_dt;
        const double dvy = sim.state().vy / config.fixed_dt;
        const double rx = std::cos(theta);
        const double ry = std::sin(theta);
        const double dot = (-dvx * rx - dvy * ry) /
                           std::hypot(dvx, dvy);
        check(dot > 0.999, "gravity should point toward moon centre");
    }
}

void test_one_step_physics() {
    lander::Simulation sim;
    sim.reset(11);
    const lander::Config& config = sim.config();
    const double r = sim.terrain().max_surface_radius() + 100.0;
    const double g = config.mu / (r * r);

    {
        lander::State s{};
        s.x = r;
        s.y = 0.0;
        s.fuel = 100.0;
        sim.set_state(s);
        sim.advance(config.fixed_dt, {});
        check_close(sim.state().vx, -g * config.fixed_dt, 1e-9,
                    "no thrust: radial gravity on +x");
        check_close(sim.state().vy, 0.0, 1e-9, "no thrust: no tangential drift");
        check_close(sim.state().fuel, 100.0, 1e-9,
                    "no thrust: no fuel burn");
    }

    {
        lander::State s{};
        s.x = 0.0;
        s.y = r;
        s.fuel = 100.0;
        sim.set_state(s);
        lander::Input input{};
        input.main_throttle = 1.0;
        sim.advance(config.fixed_dt, input);
        check_close(sim.state().vy,
                    (config.main_accel - g) * config.fixed_dt, 1e-9,
                    "full throttle: thrust minus gravity");
        check_close(sim.state().fuel,
                    100.0 - config.fuel_burn * config.fixed_dt, 1e-9,
                    "full throttle: fuel burn");
    }

    {
        lander::State s{};
        s.x = 0.0;
        s.y = r;
        s.fuel = 100.0;
        sim.set_state(s);
        lander::Input input{};
        input.main_throttle = 0.5;
        sim.advance(config.fixed_dt, input);
        check_close(sim.state().vy,
                    (0.5 * config.main_accel - g) * config.fixed_dt, 1e-9,
                    "half throttle: thrust minus gravity");
    }

    {
        lander::State s{};
        s.x = 0.0;
        s.y = r;
        s.fuel = 100.0;
        sim.set_state(s);
        lander::Input input{};
        input.main_throttle = 2.0;
        sim.advance(config.fixed_dt, input);
        const double expected =
            (config.main_accel - g) * config.fixed_dt;
        check_close(sim.state().vy, expected, 1e-9,
                    "throttle above 1 must clamp to full");
    }

    {
        lander::State s{};
        s.x = 0.0;
        s.y = r;
        s.fuel = 100.0;
        sim.set_state(s);
        lander::Input input{};
        input.main_throttle = -1.0;
        sim.advance(config.fixed_dt, input);
        check_close(sim.state().vy, -g * config.fixed_dt, 1e-9,
                    "negative throttle must clamp to zero");
    }

    {
        lander::State s{};
        s.x = r;
        s.y = 0.0;
        s.fuel = 50.0;
        sim.set_state(s);
        lander::Input input{};
        input.rotate_left = true;
        sim.advance(config.fixed_dt, input);
        check_close(sim.state().omega, -config.rotate_accel * config.fixed_dt,
                    1e-9, "rotate left changes angular velocity");
        check_close(normalize_angle(sim.state().angle),
                    2.0 * lander::kPi -
                        config.rotate_accel * config.fixed_dt *
                            config.fixed_dt,
                    1e-9, "rotate left changes angle");
        check_close(sim.state().fuel, 50.0, 1e-9,
                    "rotation alone should not burn fuel");
    }
}

void test_fixed_step_determinism() {
    lander::Simulation seed_sim;
    seed_sim.reset(987);
    const double r = seed_sim.terrain().max_surface_radius() + 30.0;
    const double v = std::sqrt(seed_sim.config().mu / r);

    auto run = [&](double step, double elapsed, double throttle) {
        lander::Simulation sim;
        sim.reset(987);
        lander::State s{};
        s.x = 0.0;
        s.y = r;
        s.vx = v;
        s.fuel = 100.0;
        sim.set_state(s);
        lander::Input input{};
        input.main_throttle = throttle;
        sim.advance(elapsed, input);
        return sim.state();
    };

    const auto a = run(1.0 / 60.0, 1.0, 0.3);
    const auto b = run(1.0 / 30.0, 1.0, 0.3);
    check(a == b, "fixed timestep must be independent of caller step size");
}

void test_local_frame_helpers() {
    lander::Terrain terrain(42);
    const double C = lander::Terrain::circumference();

    for (double arc : {0.0, 0.25 * C, 0.5 * C, 0.75 * C}) {
        const double theta = lander::Terrain::angle_at_arc(arc);
        const lander::State s = state_at_arc(terrain, arc, 10.0, 3.0, 4.0, 0.0);

        check_close(lander::radial_distance(s),
                    terrain.surface_radius_at_arc(arc) + 10.0, 1e-9,
                    "radial distance helper");
        check_close(lander::altitude_at(terrain, s), 10.0, 1e-9,
                    "altitude helper");
        check_close(normalized_difference(lander::local_up_angle(s),
                                          theta - 0.5 * lander::kPi),
                    0.0, 1e-9,
                    "local up angle helper");

        const auto lv = lander::local_velocity(s);
        check_close(lv.radial, 3.0, 1e-9, "radial velocity helper");
        check_close(lv.tangential, 4.0, 1e-9, "tangential velocity helper");

        const lander::State aligned =
            state_at_arc(terrain, arc, 0.0, 0.0, 0.0, 0.0);
        check_close(lander::local_attitude_angle(aligned), 0.0, 1e-9,
                    "aligned local attitude is zero");

        const lander::State tilted =
            state_at_arc(terrain, arc, 0.0, 0.0, 0.0, 0.2);
        check_close(lander::local_attitude_angle(tilted), 0.2, 1e-9,
                    "tilted local attitude is preserved");
    }
}

void test_landing_rules() {
    for (std::uint64_t seed : {1ULL, 2ULL, 3ULL}) {
        lander::Simulation probe;
        probe.reset(seed);
        const lander::Terrain& terrain = probe.terrain();

        for (const lander::Pad& pad : terrain.pads()) {
            lander::Simulation sim;
            sim.reset(seed);
            const lander::State s =
                state_at_arc(terrain, pad.center_arc, 0.3, -1.0, 0.2, 0.05);
            sim.set_state(s);
            check(run_to_terminal(sim), "safe landing must terminate");
            check(sim.state().landed, "safe landing should land");
            check(!sim.state().crashed, "safe landing should not crash");
            check_close(sim.state().score, 100 * pad.multiplier, 1e-9,
                        "landing score follows pad multiplier");
            check_close(lander::radial_distance(sim.state()), pad.radius,
                        1e-6, "landed lander should rest on pad radius");
            check_close(sim.state().vx, 0.0, 1e-12, "landed velocity freezes");
            check_close(sim.state().vy, 0.0, 1e-12, "landed velocity freezes");
            check_close(sim.state().omega, 0.0, 1e-12,
                        "landed angular velocity freezes");
            check_close(lander::local_attitude_angle(sim.state()), 0.0, 1e-9,
                        "landed attitude aligns with local up");
        }

        {
            lander::Simulation sim;
            sim.reset(seed);
            const lander::Pad& pad = terrain.pads().front();
            lander::State s =
                state_at_arc(terrain, pad.center_arc, 0.3, -1.0, 0.2, 0.05);
            s.angle += 2.0 * lander::kPi;
            sim.set_state(s);
            check(run_to_terminal(sim), "mod-2pi attitude should land");
            check(sim.state().landed, "mod-2pi attitude should land");
        }
    }
}

void test_crash_rules() {
    for (std::uint64_t seed : {1ULL, 2ULL}) {
        lander::Simulation probe;
        probe.reset(seed);
        const lander::Terrain& terrain = probe.terrain();

        lander::Simulation non_pad;
        non_pad.reset(seed);
        const double arc = find_non_pad_arc(terrain);
        non_pad.set_state(
            state_at_arc(terrain, arc, 0.3, -1.0, 0.2, 0.05));
        check(run_to_terminal(non_pad), "non-pad contact should terminate");
        check(non_pad.state().crashed, "non-pad contact should crash");

        lander::Simulation radial;
        radial.reset(seed);
        const lander::Pad& pad = terrain.pads().front();
        radial.set_state(
            state_at_arc(terrain, pad.center_arc, 0.3, -10.0, 0.2, 0.05));
        check(run_to_terminal(radial), "fast radial contact should terminate");
        check(radial.state().crashed, "fast radial contact should crash");

        lander::Simulation tangential;
        tangential.reset(seed);
        tangential.set_state(
            state_at_arc(terrain, pad.center_arc, 0.3, -1.0, 2.0, 0.05));
        check(run_to_terminal(tangential),
              "fast tangential contact should terminate");
        check(tangential.state().crashed,
              "fast tangential contact should crash");

        lander::Simulation attitude;
        attitude.reset(seed);
        attitude.set_state(
            state_at_arc(terrain, pad.center_arc, 0.3, -1.0, 0.2, 0.2));
        check(run_to_terminal(attitude),
              "misaligned attitude should terminate");
        check(attitude.state().crashed,
              "misaligned attitude should crash");

        if (terrain.pads().size() >= 2) {
            lander::Simulation global_up;
            global_up.reset(seed);
            lander::State s = state_at_arc(
                terrain, terrain.pads()[1].center_arc, 0.3, -1.0, 0.2, 0.0);
            s.angle = 0.0;
            global_up.set_state(s);
            check(run_to_terminal(global_up),
                  "global-up attitude should terminate");
            check(global_up.state().crashed,
                  "global-up attitude away from local up should crash");
        }

        lander::Simulation below;
        below.reset(seed);
        below.set_state(
            state_at_arc(terrain, arc, -0.1, 0.0, 0.0, 0.0));
        below.advance(below.config().fixed_dt, {});
        check(below.state().crashed, "starting below terrain should crash");
    }
}

void test_terminal_state_is_frozen() {
    lander::Simulation sim;
    sim.reset(5);
    const lander::Terrain& terrain = sim.terrain();
    const lander::Pad& pad = terrain.pads().front();

    sim.set_state(state_at_arc(terrain, pad.center_arc, 0.3, -1.0, 0.2, 0.05));
    check(run_to_terminal(sim), "terminal setup");
    const lander::State landed = sim.state();
    lander::Input input{};
    input.main_throttle = 1.0;
    input.rotate_left = true;
    sim.advance(1.0, input);
    check(sim.state() == landed, "landed state must not advance");

    const double arc = find_non_pad_arc(terrain);
    sim.set_state(state_at_arc(terrain, arc, 0.3, -10.0, 0.0, 0.0));
    check(run_to_terminal(sim), "crash setup");
    const lander::State crashed = sim.state();
    sim.advance(1.0, input);
    check(sim.state() == crashed, "crashed state must not advance");
}

void test_set_state_normalizes() {
    lander::Simulation sim;
    lander::State s{};
    s.angle = 2.0 * lander::kPi + 0.2;
    sim.set_state(s);
    check_close(sim.state().angle, 0.2, 1e-12, "positive angle wraps");

    s.angle = -0.1;
    sim.set_state(s);
    check_close(sim.state().angle, 2.0 * lander::kPi - 0.1, 1e-12,
                "negative angle wraps");
}

void test_orbit_stays_bounded() {
    for (bool clockwise : {true, false}) {
        lander::Simulation sim;
        sim.reset(1234);
        const lander::Config& config = sim.config();
        const double r0 = sim.terrain().max_surface_radius() + 20.0;
        const double v = std::sqrt(config.mu / r0);

        lander::State s{};
        s.x = 0.0;
        s.y = r0;
        s.vx = clockwise ? v : -v;
        s.vy = 0.0;
        s.fuel = 100.0;
        sim.set_state(s);

        const double T =
            2.0 * lander::kPi * std::sqrt((r0 * r0 * r0) / config.mu);
        const int steps =
            static_cast<int>(std::ceil(3.0 * T / config.fixed_dt));
        double min_r = 1e30;
        double max_r = -1e30;
        double previous_theta = lander::kPi / 2;
        double total_theta = 0.0;

        for (int i = 0; i < steps; ++i) {
            sim.advance(config.fixed_dt, {});
            const lander::State& st = sim.state();
            if (st.landed || st.crashed) {
                break;
            }
            const double r = lander::radial_distance(st);
            min_r = std::min(min_r, r);
            max_r = std::max(max_r, r);
            const double theta = std::atan2(st.y, st.x);
            double d = theta - previous_theta;
            if (d > lander::kPi) {
                d -= 2.0 * lander::kPi;
            } else if (d < -lander::kPi) {
                d += 2.0 * lander::kPi;
            }
            total_theta += d;
            previous_theta = theta;
        }

        check(!sim.state().landed && !sim.state().crashed,
              "terrain-clearing orbit should not land or crash");
        check(min_r > r0 * 0.98 && max_r < r0 * 1.02,
              "circular orbit radius should stay bounded");
        const double expected =
            (clockwise ? -3.0 * 2.0 * lander::kPi
                       : 3.0 * 2.0 * lander::kPi);
        check_close(total_theta, expected, 0.05 * std::abs(expected),
                    "orbit should complete about three revolutions");
    }
}

}  // namespace

int main() {
    test_reference_values();
    test_terrain_determinism_and_wrap();
    test_radial_gravity();
    test_one_step_physics();
    test_fixed_step_determinism();
    test_local_frame_helpers();
    test_landing_rules();
    test_crash_rules();
    test_terminal_state_is_frozen();
    test_set_state_normalizes();
    test_orbit_stays_bounded();

    if (failures == 0) {
        std::puts("All lander_tests passed");
        return 0;
    }
    std::printf("%d lander_tests failed\n", failures);
    return 1;
}
