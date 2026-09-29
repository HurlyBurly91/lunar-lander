#include "lander/sim.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

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

double norm_angle(double a) {
    a = std::fmod(a, 2.0 * lander::kPi);
    if (a < 0.0) {
        a += 2.0 * lander::kPi;
    }
    return a;
}

// M05-R1-V01: the canonical universe law and the 1/9 companion scaling.
void test_canonical_laws() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double R0 = lander::kReferenceRadius;
    const double g0 = mu0 / (R0 * R0);
    const double v0 = std::sqrt(mu0 / R0);
    const double T0 = 2.0 * lander::kPi * std::sqrt((R0 * R0 * R0) / mu0);

    // mu0 = g0*R0^2 (g0 rounded to 1.62), v0 = sqrt(mu0/R0), T0 ~= 90 s.
    check_close(g0, 1.62, 0.005, "primary intrinsic surface gravity ~ 1.62");
    check_close(v0, 23.2048, 0.001, "primary nominal circular speed");
    check_close(T0, 90.0, 0.01, "primary nominal circular period ~ 90 s");

    const auto bin =
        lander::BinarySystem::canonical(mu0, 1ULL, lander::companion_seed(1ULL));

    const lander::Body& p = bin.body(0);
    const lander::Body& c = bin.body(1);
    check_close(p.reference_radius, R0, 1e-9, "primary radius is the reference");
    check_close(p.mu, mu0, 1e-9, "primary mu is the reference");
    check(p.side == -1, "primary sits on the negative side");
    check(c.side == +1, "companion sits on the positive side");

    // Companion is exactly 1/9 the radius, mu/81, with matching surface
    // gravity (so the same 1.62 m/s^2 surface world at 1/9 the scale).
    const double s = 1.0 / 9.0;
    check_close(c.reference_radius, R0 * s, 1e-9,
                "companion radius is R0/9");
    check_close(c.mu, mu0 * s * s, 1e-6, "companion mu is mu0/81");

    const double g1 = c.mu / (c.reference_radius * c.reference_radius);
    check_close(g1, g0, 1e-6, "companion surface gravity matches the primary");

    const double v1 = std::sqrt(c.mu / c.reference_radius);
    check_close(v1, v0 / 3.0, 1e-6, "companion nominal circular speed ~ v0/3");

    const double T1 = 2.0 * lander::kPi *
                      std::sqrt((c.reference_radius * c.reference_radius *
                                 c.reference_radius) /
                                c.mu);
    check_close(T1, T0 / 3.0, 1e-6,
                "companion nominal circular period ~ 30 s");

    // Binary-system derived quantities.
    check_close(bin.separation(), 600.0, 1e-9, "fixed centre-to-centre 600 m");
    check_close(bin.mu_system(), mu0 * (1.0 + s * s), 1e-6,
                "mu_system is mu0 + mu1");
    const double omega =
        std::sqrt(bin.mu_system() / (600.0 * 600.0 * 600.0));
    check_close(bin.omega(), omega, 1e-12, "omega = sqrt(mu_system/D^3)");
    check_close(bin.period(), 2.0 * lander::kPi / omega, 1e-6,
                "binary period = 2*pi/omega");
    check_close(bin.period(), 216.94244, 0.01, "binary period ~ 216.94 s");

    check_close(p.barycentric_radius, 600.0 / 82.0, 1e-6,
                "primary barycentric radius D/82");
    check_close(c.barycentric_radius, 600.0 * 81.0 / 82.0, 1e-6,
                "companion barycentric radius 81*D/82");
    check_close(p.barycentric_radius * bin.omega(), 0.21192, 1e-3,
                "primary barycentric speed");
    check_close(c.barycentric_radius * bin.omega(), 17.16555, 1e-2,
                "companion barycentric speed");
}

// M05-R1-V02: the analytic ephemeris is a fixed separation about a fixed
// barycentre, with analytic velocities that match position derivatives.
void test_ephemeris() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double mu1 = mu0 / 81.0;
    const auto bin =
        lander::BinarySystem::canonical(mu0, 1ULL, lander::companion_seed(1ULL));

    const std::vector<double> ts = {
        0.0, 1.0, 10.0, 50.0, bin.period() * 0.25, bin.period() * 0.5,
        bin.period() * 0.75, bin.period()};

    for (double t : ts) {
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        const double sep = std::hypot(p1.x - p0.x, p1.y - p0.y);
        check_close(sep, 600.0, 1e-6, "separation stays 600 m");

        // The barycentre stays at the origin for all time.
        const double bx = mu0 * p0.x + mu1 * p1.x;
        const double by = mu0 * p0.y + mu1 * p1.y;
        check(std::hypot(bx, by) < 1e-6, "barycentre stays at the origin");
    }

    // One full binary period returns both bodies to their start.
    for (int i = 0; i < 2; ++i) {
        const lander::Vec2 p0 = bin.position(i, 0.0);
        const lander::Vec2 pT = bin.position(i, bin.period());
        check_close(std::hypot(pT.x - p0.x, pT.y - p0.y), 0.0, 1e-6,
                    "one period returns body to its start");
        const lander::Vec2 v0 = bin.velocity(i, 0.0);
        const lander::Vec2 vT = bin.velocity(i, bin.period());
        check_close(std::hypot(vT.x - v0.x, vT.y - v0.y), 0.0, 1e-6,
                    "one period returns body velocity");
    }

    // Analytic velocity agrees with the central-difference position derivative.
    for (int i = 0; i < 2; ++i) {
        const double t = 3.0;
        const double h = 1e-4;
        const lander::Vec2 pa = bin.position(i, t - h);
        const lander::Vec2 pb = bin.position(i, t + h);
        const lander::Vec2 fd{(pb.x - pa.x) / (2.0 * h),
                              (pb.y - pa.y) / (2.0 * h)};
        const lander::Vec2 ve = bin.velocity(i, t);
        check_close(std::hypot(fd.x - ve.x, fd.y - ve.y), 0.0, 1e-4,
                    "analytic velocity matches position derivative");
    }

    // The prescribed barycentric acceleration equals the other body's
    // inverse-square field on the circular orbit.
    for (int i = 0; i < 2; ++i) {
        const int other = 1 - i;
        const double t = 7.0;
        const lander::Vec2 acc = bin.acceleration(i, t);
        const lander::Vec2 pi = bin.position(i, t);
        const lander::Vec2 po = bin.position(other, t);
        const double rx = po.x - pi.x;
        const double ry = po.y - pi.y;
        const double r = std::hypot(rx, ry);
        const double inv = 1.0 / (r * r * r);
        const lander::Vec2 field{bin.body(other).mu * rx * inv,
                                 bin.body(other).mu * ry * inv};
        check_close(std::hypot(acc.x - field.x, acc.y - field.y), 0.0, 1e-6,
                    "barycentric accel equals the other body's field");
    }
}

// M05-R1-V03: the spacecraft field is the exact superposition of both bodies'
// inverse-square fields; neither is ever switched off.
void test_gravity_superposition() {
    const lander::Config config{};
    const auto bin =
        lander::BinarySystem::canonical(config.mu, 2ULL,
                                        lander::companion_seed(2ULL));

    for (double t : {0.0, 20.0, 80.0}) {
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        const std::vector<lander::Vec2> pts = {
            {0.5 * (p0.x + p1.x), 0.5 * (p0.y + p1.y)},
            {p0.x + 100.0, p0.y},
            {p1.x + 40.0, p1.y},
            {-800.0, 200.0},
        };
        for (const lander::Vec2& p : pts) {
            const lander::Vec2 g = bin.gravity(p, t);
            double ax = 0.0;
            double ay = 0.0;
            for (int i = 0; i < 2; ++i) {
                const lander::Vec2 q = bin.position(i, t);
                const double rx = p.x - q.x;
                const double ry = p.y - q.y;
                const double r = std::hypot(rx, ry);
                if (r < 1.0e-9) {
                    continue;
                }
                const double inv = 1.0 / (r * r * r);
                ax -= bin.body(i).mu * rx * inv;
                ay -= bin.body(i).mu * ry * inv;
            }
            check_close(g.x, ax, 1e-9, "combined field x equals sum of both");
            check_close(g.y, ay, 1e-9, "combined field y equals sum of both");
        }
    }

    // At the midpoint the companion's contribution must not be dropped, even
    // though the primary's field is much stronger: the total differs from a
    // primary-only field by the companion's field.
    const lander::Vec2 p0 = bin.position(0, 0.0);
    const lander::Vec2 p1 = bin.position(1, 0.0);
    const lander::Vec2 mid{0.5 * (p0.x + p1.x), 0.5 * (p0.y + p1.y)};
    const lander::Vec2 total = bin.gravity(mid, 0.0);
    double px = 0.0;
    double py = 0.0;
    {
        const double rx = mid.x - p0.x;
        const double ry = mid.y - p0.y;
        const double r = std::hypot(rx, ry);
        const double inv = 1.0 / (r * r * r);
        px = -bin.body(0).mu * rx * inv;
        py = -bin.body(0).mu * ry * inv;
    }
    check(std::hypot(total.x - px, total.y - py) > 1.0e-3,
          "companion field is present at the midpoint (not switched off)");
}

// M05-R3-V05: the per-body field exposed for the navigation overlay is the
// same inverse-square contribution that the total field sums.
void test_gravity_from_matches_total() {
    const lander::Config config{};
    const auto bin =
        lander::BinarySystem::canonical(config.mu, 20ULL,
                                        lander::companion_seed(20ULL));

    for (double t : {0.0, 13.0, 77.0}) {
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        const std::vector<lander::Vec2> pts = {
            {0.5 * (p0.x + p1.x), 0.5 * (p0.y + p1.y)},
            {p0.x + 120.0, p0.y - 30.0},
            {p1.x - 50.0, p1.y + 20.0},
            {-900.0, -250.0},
        };
        for (const lander::Vec2& p : pts) {
            const lander::Vec2 g0 = bin.gravity_from(0, p, t);
            const lander::Vec2 g1 = bin.gravity_from(1, p, t);
            const lander::Vec2 total = bin.gravity(p, t);
            check_close(g0.x + g1.x, total.x, 1.0e-12,
                        "gravity() is the sum of gravity_from(0, ...)");
            check_close(g0.y + g1.y, total.y, 1.0e-12,
                        "gravity() is the sum of gravity_from(1, ...)");

            for (int i = 0; i < 2; ++i) {
                const lander::Vec2 q = bin.position(i, t);
                const double rx = p.x - q.x;
                const double ry = p.y - q.y;
                const double r = std::hypot(rx, ry);
                const lander::Vec2 expected{
                    -bin.body(i).mu * rx / (r * r * r),
                    -bin.body(i).mu * ry / (r * r * r)};
                const lander::Vec2 actual = bin.gravity_from(i, p, t);
                check_close(actual.x, expected.x, 1.0e-9,
                            "gravity_from x is inverse-square");
                check_close(actual.y, expected.y, 1.0e-9,
                            "gravity_from y is inverse-square");
            }
        }
    }

    const lander::Vec2 q0 = bin.position(0, 0.0);
    check_close(bin.gravity_from(0, q0, 0.0).x, 0.0, 1.0e-12,
                "zero guard x at the exact body centre");
    check_close(bin.gravity_from(0, q0, 0.0).y, 0.0, 1.0e-12,
                "zero guard y at the exact body centre");
}

// M05-R1-V04: relative kinematics subtracts the body's position and velocity.
void test_relative_kinematics() {
    const lander::Config config{};
    const auto bin =
        lander::BinarySystem::canonical(config.mu, 3ULL,
                                        lander::companion_seed(3ULL));

    // A ship co-moving with the primary's rotating surface point (attached
    // to its pad) has zero local velocity relative to that point and sits at
    // the surface radius (M05-R3).
    {
        const double t = 5.0;
        const lander::State s = lander::attached_state(bin, 0, 0.0, t);
        const lander::Vec2 bpos = bin.position(0, t);
        const double local = bin.body(0).terrain.angle_at_arc(0.0);
        const double surface = bin.body(0).terrain.surface_radius_at_arc(0.0);
        const lander::Vec2 sp_vel =
            bin.surface_point(0, local, surface, t).velocity;
        const lander::LocalVelocity lv = lander::local_velocity(s, bpos, sp_vel);
        check_close(lv.radial, 0.0, 1e-9, "co-moving ship has zero local radial");
        check_close(lv.tangential, 0.0, 1e-9,
                    "co-moving ship has zero local tangential");
        check_close(lander::radial_distance(s, bpos), surface, 1e-9,
                    "co-moving ship sits at the surface radius");
    }

    // A globally stationary ship beside the moving companion (at t = 0 the
    // companion sits at +x with purely +y velocity) has a large local
    // tangential velocity but zero local radial velocity.
    {
        const lander::Vec2 cpos = bin.position(1, 0.0);
        lander::State s{};
        s.x = cpos.x + 30.0;
        s.y = cpos.y;
        s.vx = 0.0;
        s.vy = 0.0;
        const lander::Vec2 bvel = bin.velocity(1, 0.0);
        const lander::LocalVelocity lv = lander::local_velocity(s, cpos, bvel);
        check_close(lv.radial, 0.0, 1e-9,
                    "stationary ship has zero local radial");
        check_close(lv.tangential,
                    bin.body(1).barycentric_radius * bin.omega(), 1e-9,
                    "stationary ship has the companion's tangential speed");
        check_close(lander::radial_distance(s, cpos), 30.0, 1e-9,
                    "stationary ship is 30 m from the companion");
    }
}

// M05-R3-V11: the rotation law and the tidal-lock invariant.
void test_body_rotation_law() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 401ULL, lander::companion_seed(401ULL));

    check_close(bin.body_rotation(0.0), 0.0, 1.0e-12,
                "rotation is zero at t = 0");
    // The spin is the pure time function theta(t) - theta0.
    for (double t : {0.0, 1.0, 17.3, 60.0, 100.0, bin.period() * 0.5,
                     bin.period()}) {
        check_close(bin.body_rotation(t), bin.theta(t) - 0.0, 1.0e-9,
                    "body_rotation(t) = theta(t) - theta0");
    }
    // The spin rate equals the binary orbital rate; the period is ~216.94 s.
    const double h = 1.0e-4;
    const double t = 21.0;
    const double fd = (bin.body_rotation(t + h) - bin.body_rotation(t - h)) /
                     (2.0 * h);
    check_close(fd, bin.omega(), 1.0e-6,
                "spin rate equals the binary orbital rate");
    check_close(bin.period(), 216.94, 0.1, "binary period is ~216.94 s");
    // One full binary period returns the rotation phase to zero.
    check_close(std::atan2(std::sin(bin.body_rotation(bin.period())),
                           std::cos(bin.body_rotation(bin.period()))),
                0.0, 1.0e-9, "one period returns the rotation phase");

    // Tidal-lock invariant: for every body the companion sits at a fixed
    // body-local direction for all time (the same face points at it).
    for (int i = 0; i < 2; ++i) {
        const int other = 1 - i;
        const lander::Vec2 d0 =
            bin.position(other, 0.0) - bin.position(i, 0.0);
        const double ref = std::atan2(d0.y, d0.x);
        for (double tt : {5.0, 33.0, 90.0, 180.0, bin.period() * 0.75,
                          bin.period()}) {
            const lander::Vec2 d = bin.position(other, tt) - bin.position(i, tt);
            const double world = std::atan2(d.y, d.x);
            check_close(
                std::atan2(std::sin(world - bin.body_rotation(tt) - ref),
                           std::cos(world - bin.body_rotation(tt) - ref)),
                0.0, 1.0e-9,
                "the companion keeps a fixed local direction (tidal lock)");
        }
    }
}

// M05-R3-V12: the local<->world angle transform and its inverse.
void test_local_world_angle_transform() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 402ULL, lander::companion_seed(402ULL));

    for (int i = 0; i < 2; ++i) {
        const lander::Body& body = bin.body(i);
        const double c = body.terrain.circumference();
        for (double t : {0.0, 3.7, 40.0, bin.period() * 0.625}) {
            const double rot = bin.body_rotation(t);
            for (double frac : {0.0, 0.125, 0.37, 0.5, 0.83, 0.999}) {
                const double arc = frac * c;
                const double local = body.terrain.angle_at_arc(arc);
                const double world = local + rot;
                // The inverse recovers the local angle from a world angle.
                const double back = body.terrain.angle_at_arc(
                    body.terrain.arc_at_angle(world - rot));
                check_close(std::atan2(std::sin(back - local),
                                       std::cos(back - local)),
                            0.0, 1.0e-6,
                            "the inverse recovers the local angle");
                // A world-angle terrain lookup matches the local lookup.
                check_close(
                    body.terrain.surface_radius_at_arc(
                        body.terrain.arc_at_angle(world - rot)),
                    body.terrain.surface_radius_at_arc(arc), 1.0e-9,
                    "world-angle lookup matches the local lookup");
            }
        }
    }
}

// M05-R3-V13: surface-point position and velocity, including the
// finite-difference derivative of the position.
void test_surface_point_velocity() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 403ULL, lander::companion_seed(403ULL));

    for (int i = 0; i < 2; ++i) {
        const lander::Body& body = bin.body(i);
        const double c = body.terrain.circumference();
        for (double t : {0.0, 2.2, 25.0, bin.period() * 0.8}) {
            const lander::Vec2 cpos = bin.position(i, t);
            const lander::Vec2 cvel = bin.velocity(i, t);
            for (double frac : {0.0, 0.25, 0.6, 1.0}) {
                const double arc = frac * c;
                const double local = body.terrain.angle_at_arc(arc);
                const double radius =
                    body.terrain.surface_radius_at_arc(arc);
                const auto sp = bin.surface_point(i, local, radius, t);
                const double world = local + bin.body_rotation(t);
                check_close(sp.position.x, cpos.x + std::cos(world) * radius,
                            1.0e-9, "surface point position x");
                check_close(sp.position.y, cpos.y + std::sin(world) * radius,
                            1.0e-9, "surface point position y");
                const double ox = std::cos(world) * radius;
                const double oy = std::sin(world) * radius;
                check_close(sp.velocity.x, cvel.x - bin.omega() * oy, 1.0e-9,
                            "surface point velocity x is centre plus spin");
                check_close(sp.velocity.y, cvel.y + bin.omega() * ox, 1.0e-9,
                            "surface point velocity y is centre plus spin");
                const double h = 1.0e-4;
                const auto a = bin.surface_point(i, local, radius, t - h);
                const auto b = bin.surface_point(i, local, radius, t + h);
                const double fdx = (b.position.x - a.position.x) / (2.0 * h);
                const double fdy = (b.position.y - a.position.y) / (2.0 * h);
                check_close(std::hypot(fdx - sp.velocity.x,
                                       fdy - sp.velocity.y),
                            0.0, 1.0e-4,
                            "surface point velocity matches the position "
                            "derivative");
            }
        }
        // A zero offset (the body centre) carries exactly the centre velocity.
        const lander::Vec2 cvel9 = bin.velocity(i, 9.0);
        const auto spc = bin.surface_point(i, 0.0, 0.0, 9.0);
        check(spc.velocity == cvel9,
              "a zero offset gives exactly the centre velocity");
    }
}

// M05-R3-V19: the ephemeris, gravity, and period are unchanged by the
// rotation (closed-form values match the pre-rotation model).
void test_ephemeris_gravity_unchanged() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 404ULL, lander::companion_seed(404ULL));

    check_close(bin.period(), 216.94, 0.1, "binary period is ~216.94 s");

    // Positions and velocities match the prescribed closed form
    // (fixed circular radii, theta(t) = theta0 + omega t) at every sample.
    for (int i = 0; i < 2; ++i) {
        const lander::Body& b = bin.body(i);
        for (double t : {0.0, 4.0, 77.0, bin.period() * 0.5, bin.period()}) {
            const double th = bin.theta(t);
            const lander::Vec2 p = bin.position(i, t);
            check_close(p.x, b.side * b.barycentric_radius * std::cos(th),
                        1.0e-9, "position x matches the closed form");
            check_close(p.y, b.side * b.barycentric_radius * std::sin(th),
                        1.0e-9, "position y matches the closed form");
            const lander::Vec2 v = bin.velocity(i, t);
            check_close(v.x, -b.side * b.barycentric_radius * bin.omega() *
                                 std::sin(th),
                        1.0e-9, "velocity x matches the closed form");
            check_close(v.y, b.side * b.barycentric_radius * bin.omega() *
                                 std::cos(th),
                        1.0e-9, "velocity y matches the closed form");
        }
    }

    // The gravitational field at fixed world points is the pure inverse-
    // square sum from the closed-form centre positions: the rotation adds
    // no force to free flight.
    const lander::Vec2 probe{120.0, -60.0};
    for (double t : {0.0, 12.0, 95.0, bin.period() * 0.6}) {
        const lander::Vec2 g = bin.gravity(probe, t);
        lander::Vec2 sum{};
        for (int i = 0; i < 2; ++i) {
            const lander::Body& b = bin.body(i);
            const double th = bin.theta(t);
            const double rx =
                probe.x - b.side * b.barycentric_radius * std::cos(th);
            const double ry =
                probe.y - b.side * b.barycentric_radius * std::sin(th);
            const double r3 = std::pow(std::hypot(rx, ry), 3.0);
            sum = sum + lander::Vec2{-b.mu * rx / r3, -b.mu * ry / r3};
        }
        check_close(g.x, sum.x, 1.0e-9,
                    "gravity x is the pure inverse-square sum");
        check_close(g.y, sum.y, 1.0e-9,
                    "gravity y is the pure inverse-square sum");
    }
}

}  // namespace

int main() {
    test_canonical_laws();
    test_ephemeris();
    test_gravity_superposition();
    test_gravity_from_matches_total();
    test_relative_kinematics();
    test_body_rotation_law();
    test_local_world_angle_transform();
    test_surface_point_velocity();
    test_ephemeris_gravity_unchanged();

    if (failures == 0) {
        std::puts("All lander_binary_tests passed");
        return 0;
    }
    std::printf("%d lander_binary_tests failed\n", failures);
    return 1;
}
