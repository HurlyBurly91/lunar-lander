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

// M05-R1-V04: relative kinematics subtracts the body's position and velocity.
void test_relative_kinematics() {
    const lander::Config config{};
    const auto bin =
        lander::BinarySystem::canonical(config.mu, 3ULL,
                                        lander::companion_seed(3ULL));

    // A ship co-moving with the primary (attached to its surface) has zero
    // local velocity and sits at the surface radius.
    {
        const double t = 5.0;
        const lander::State s = lander::attached_state(bin, 0, 0.0, t);
        const lander::Vec2 bpos = bin.position(0, t);
        const lander::Vec2 bvel = bin.velocity(0, t);
        const lander::LocalVelocity lv = lander::local_velocity(s, bpos, bvel);
        check_close(lv.radial, 0.0, 1e-9, "co-moving ship has zero local radial");
        check_close(lv.tangential, 0.0, 1e-9,
                    "co-moving ship has zero local tangential");
        check_close(lander::radial_distance(s, bpos),
                    bin.body(0).terrain.surface_radius_at_arc(0.0), 1e-9,
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

}  // namespace

int main() {
    test_canonical_laws();
    test_ephemeris();
    test_gravity_superposition();
    test_relative_kinematics();

    if (failures == 0) {
        std::puts("All lander_binary_tests passed");
        return 0;
    }
    std::printf("%d lander_binary_tests failed\n", failures);
    return 1;
}
