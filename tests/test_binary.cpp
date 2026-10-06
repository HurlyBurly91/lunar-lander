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

// M06-R13-V01: the canonical three-body construction and the per-body
// scale / seed laws.
void test_three_body_construction() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double R0 = lander::kReferenceRadius;
    const double s = 1.0 / 9.0;
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(mu0, 7ULL,
                                        lander::companion_seed(7ULL));

    check(lander::BinarySystem::kBodyCount == 3,
          "the system has exactly three bodies");
    check(bin.bodies().size() == 3, "bodies() exposes three entries");

    const lander::Body& p = bin.body(0);
    const lander::Body& c = bin.body(1);
    const lander::Body& m = bin.body(2);

    check_close(p.reference_radius, R0, 1e-9, "primary radius is the reference");
    check_close(p.mu, mu0, 1e-9, "primary mu is the reference");

    // M05 companion, unchanged: R0/9 radius, mu/81, matching surface
    // gravity, v0/3 circular speed, T0/3 period.
    const double g0 = mu0 / (R0 * R0);
    const double v0 = std::sqrt(mu0 / R0);
    const double T0 = 2.0 * lander::kPi * std::sqrt((R0 * R0 * R0) / mu0);
    check_close(g0, 1.62, 0.005, "primary intrinsic surface gravity ~ 1.62");
    check_close(v0, 23.2048, 0.001, "primary nominal circular speed");
    check_close(T0, 90.0, 0.01, "primary nominal circular period ~ 90 s");
    check_close(c.reference_radius, R0 * s, 1e-9, "companion radius is R0/9");
    check_close(c.mu, mu0 * s * s, 1e-6, "companion mu is mu0/81");
    check_close(c.mu / (c.reference_radius * c.reference_radius), g0, 1e-6,
                "companion surface gravity matches the primary");
    check_close(std::sqrt(c.mu / c.reference_radius), v0 / 3.0, 1e-6,
                "companion nominal circular speed ~ v0/3");
    check_close(
        2.0 * lander::kPi *
            std::sqrt((c.reference_radius * c.reference_radius *
                       c.reference_radius) /
                     c.mu),
        T0 / 3.0, 1e-6, "companion nominal circular period ~ 30 s");

    // M06-R13 outer moonlet: the companion's scale, a distinct salted
    // seed, matching surface gravity.
    check_close(m.reference_radius, R0 * s, 1e-9, "moonlet radius is R0/9");
    check_close(m.mu, mu0 * s * s, 1e-6, "moonlet mu is mu0/81");
    check_close(m.mu / (m.reference_radius * m.reference_radius), g0, 1e-6,
                "moonlet surface gravity matches the primary");
    check(m.terrain.seed() == lander::moonlet_seed(7ULL),
          "moonlet terrain seed is the salted primary seed");
    check(m.terrain.seed() != p.terrain.seed(),
          "moonlet terrain seed differs from the primary's");
    check(m.terrain.seed() != c.terrain.seed(),
          "moonlet terrain seed differs from the companion's");
    check(c.terrain.seed() == lander::companion_seed(7ULL),
          "companion terrain seed is unchanged");

    // All three terrains share the companion scale; the reference radius
    // is stored per terrain.
    check_close(m.terrain.reference_radius(), R0 * s, 1e-9,
                "moonlet terrain reference radius is R0/9");
}

// M06-R13-V02: the total barycentre (all three masses) stays at the
// origin for all time, including across full outer periods.
void test_total_barycentre() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double mu1 = mu0 / 81.0;
    const double mu2 = mu1;
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(mu0, 11ULL,
                                        lander::companion_seed(11ULL));

    const double T = bin.period_outer();
    for (double t : {0.0, T * 0.25, T * 0.5, T, 3.0 * T, 10.0 * T}) {
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        const lander::Vec2 p2 = bin.position(2, t);
        const double bx = mu0 * p0.x + mu1 * p1.x + mu2 * p2.x;
        const double by = mu0 * p0.y + mu1 * p1.y + mu2 * p2.y;
        check_close(std::hypot(bx, by), 0.0, 1e-5,
                    "total barycentre stays at the origin");
    }
}

// M06-R13-V03: the inner separation stays exactly 600 m for all time,
// while the inner pair itself drifts with the outer orbit.
void test_inner_separation() {
    const lander::Config config{};
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(config.mu, 12ULL,
                                        lander::companion_seed(12ULL));

    const double T = bin.period_outer();
    for (int k = 0; k < 20; ++k) {
        const double t = k * 0.05 * T;
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        check_close(std::hypot(p1.x - p0.x, p1.y - p0.y), 600.0, 1e-6,
                    "separation stays 600 m");
    }
    // The inner pair is no longer pinned at the origin: the barycentre
    // itself orbits (start vs midpoint of an outer period differ by
    // a_inner_outer * sqrt(2) > 1 m).
    const lander::Vec2 b0 = bin.barycentre_inner(0.0);
    const lander::Vec2 bm = bin.barycentre_inner(0.5 * T);
    check(std::hypot(bm.x - b0.x, bm.y - b0.y) > 1.0,
          "the inner barycentre moves across an outer half-period");
}

// M06-R13-V04: the inner-pair relative configuration (position offset and
// relative velocity about B01) is exactly the old M05 two-body solution,
// computed independently from mu0 alone.
void test_inner_pair_preservation() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double mu1 = mu0 / 81.0;
    const double mu_inner = mu0 + mu1;
    const double omega = std::sqrt(mu_inner / (600.0 * 600.0 * 600.0));
    const double a0 = 600.0 * mu1 / mu_inner;
    const double a1 = 600.0 * mu0 / mu_inner;
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(mu0, 13ULL,
                                        lander::companion_seed(13ULL));

    const double T = bin.period_outer();
    for (double t : {0.0, 7.0, 41.0, T * 0.37, T, 2.0 * T}) {
        const double th = omega * t;
        const lander::Vec2 b = bin.barycentre_inner(t);
        const lander::Vec2 p0 = bin.position(0, t);
        const lander::Vec2 p1 = bin.position(1, t);
        const lander::Vec2 r0{p0.x - b.x, p0.y - b.y};
        const lander::Vec2 r1{p1.x - b.x, p1.y - b.y};
        check_close(r0.x, -a0 * std::cos(th), 1e-9,
                    "primary offset about B01 matches the M05 closed form x");
        check_close(r0.y, -a0 * std::sin(th), 1e-9,
                    "primary offset about B01 matches the M05 closed form y");
        check_close(r1.x, a1 * std::cos(th), 1e-9,
                    "companion offset about B01 matches the M05 closed form x");
        check_close(r1.y, a1 * std::sin(th), 1e-9,
                    "companion offset about B01 matches the M05 closed form y");

        // The old M05 absolute positions recovered: old P_i = r_i.
        const lander::Vec2 v0 = bin.velocity(0, t);
        const lander::Vec2 v1 = bin.velocity(1, t);
        const lander::Vec2 dv{v1.x - v0.x, v1.y - v0.y};
        check_close(dv.x, -600.0 * omega * std::sin(th), 1e-9,
                    "relative velocity x matches the M05 closed form");
        check_close(dv.y, 600.0 * omega * std::cos(th), 1e-9,
                    "relative velocity y matches the M05 closed form");
    }
}

// M06-R13-V05: analytic velocity / acceleration agree with central
// differences of the position law for all three bodies.
void test_analytic_derivatives() {
    const lander::Config config{};
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(config.mu, 14ULL,
                                        lander::companion_seed(14ULL));

    const double h = 1.0e-3;
    for (double t : {3.0, 50.0}) {
        for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
            const lander::Vec2 pa = bin.position(i, t - h);
            const lander::Vec2 pb = bin.position(i, t + h);
            const lander::Vec2 fd{(pb.x - pa.x) / (2.0 * h),
                                  (pb.y - pa.y) / (2.0 * h)};
            const lander::Vec2 ve = bin.velocity(i, t);
            check_close(std::hypot(fd.x - ve.x, fd.y - ve.y), 0.0, 1e-6,
                        "analytic velocity matches the position derivative");

            const lander::Vec2 va = bin.velocity(i, t - h);
            const lander::Vec2 vb = bin.velocity(i, t + h);
            const lander::Vec2 fdv{(vb.x - va.x) / (2.0 * h),
                                   (vb.y - va.y) / (2.0 * h)};
            const lander::Vec2 ae = bin.acceleration(i, t);
            check_close(std::hypot(fdv.x - ae.x, fdv.y - ae.y), 0.0, 1e-6,
                        "analytic acceleration matches the velocity "
                        "derivative");
        }
    }
}

// M06-R13-V06: the outer-orbit closed form: body 2 and the inner
// barycentre on fixed circles about the origin, phase pi/2 at t = 0.
void test_outer_orbit_closed_form() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double mu1 = mu0 / 81.0;
    const double mu_inner = mu0 + mu1;
    const double mu_total = mu_inner + mu1;
    const double omega = std::sqrt(mu_total / (1200.0 * 1200.0 * 1200.0));
    const double a2 = 1200.0 * mu_inner / mu_total;
    const double aio = 1200.0 * mu1 / mu_total;
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(mu0, 15ULL,
                                        lander::companion_seed(15ULL));

    check_close(bin.omega_outer(), omega, 1e-15,
                "omega_outer = sqrt(mu_total / D_OUTER^3)");
    check_close(bin.a2_outer(), a2, 1e-9, "a2_outer = D_OUTER * mu_inner / "
                                          "mu_total");
    check_close(bin.a_inner_outer(), aio, 1e-9, "a_inner_outer = "
                                                "D_OUTER * mu2 / mu_total");

    for (double t : {0.0, 17.0, 120.0, bin.period_outer() * 0.75,
                     bin.period_outer()}) {
        const double th = 0.5 * lander::kPi + omega * t;
        const lander::Vec2 p2 = bin.position(2, t);
        const lander::Vec2 b = bin.barycentre_inner(t);

        check_close(std::hypot(p2.x, p2.y), a2, 1e-9,
                    "body 2 sits on its fixed circle");
        check_close(norm_angle(std::atan2(p2.y, p2.x) - th), 0.0, 1e-9,
                    "body 2 phase starts at pi/2");
        check_close(b.x, -aio * std::cos(th), 1e-9,
                    "B01 matches its closed form x");
        check_close(b.y, -aio * std::sin(th), 1e-9,
                    "B01 matches its closed form y");
        check_close(std::hypot(b.x, b.y), aio, 1e-9,
                    "B01 sits on its fixed circle");
        check_close(std::hypot(p2.x - b.x, p2.y - b.y), 1200.0, 1e-6,
                    "outer separation stays 1200 m");
    }
}

// M06-R13-V07: one inner period returns the relative configuration; one
// outer period returns body 2 and B01 to their start.
void test_one_period_return() {
    const lander::Config config{};
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(config.mu, 16ULL,
                                        lander::companion_seed(16ULL));

    for (double t : {17.0, 500.0}) {
        const lander::Vec2 d0{bin.position(1, t).x - bin.position(0, t).x,
                              bin.position(1, t).y - bin.position(0, t).y};
        const lander::Vec2 d1{
            bin.position(1, t + bin.period_inner()).x -
                bin.position(0, t + bin.period_inner()).x,
            bin.position(1, t + bin.period_inner()).y -
                bin.position(0, t + bin.period_inner()).y};
        check_close(std::hypot(d1.x - d0.x, d1.y - d0.y), 0.0, 1e-9,
                    "one inner period returns the relative position");
        const lander::Vec2 dv0{bin.velocity(1, t).x - bin.velocity(0, t).x,
                               bin.velocity(1, t).y - bin.velocity(0, t).y};
        const lander::Vec2 dv1{
            bin.velocity(1, t + bin.period_inner()).x -
                bin.velocity(0, t + bin.period_inner()).x,
            bin.velocity(1, t + bin.period_inner()).y -
                bin.velocity(0, t + bin.period_inner()).y};
        check_close(std::hypot(dv1.x - dv0.x, dv1.y - dv0.y), 0.0, 1e-9,
                    "one inner period returns the relative velocity");
    }

    for (double t : {33.0, 800.0}) {
        const lander::Vec2 p0 = bin.position(2, t);
        const lander::Vec2 pT = bin.position(2, t + bin.period_outer());
        check_close(std::hypot(pT.x - p0.x, pT.y - p0.y), 0.0, 1e-9,
                    "one outer period returns body 2 to its start");
        const lander::Vec2 v0 = bin.velocity(2, t);
        const lander::Vec2 vT = bin.velocity(2, t + bin.period_outer());
        check_close(std::hypot(vT.x - v0.x, vT.y - v0.y), 0.0, 1e-9,
                    "one outer period returns body 2 velocity");
        const lander::Vec2 b0 = bin.barycentre_inner(t);
        const lander::Vec2 bT = bin.barycentre_inner(t + bin.period_outer());
        check_close(std::hypot(bT.x - b0.x, bT.y - b0.y), 0.0, 1e-9,
                    "one outer period returns B01 to its start");
    }
}

// M06-R13-V08: per-body tidal locking: bodies 0/1 spin at the inner rate,
// body 2 at the outer rate; each body keeps a fixed face toward its orbit
// partner; the unindexed accessor keeps the M05 body-0 form.
void test_body_rotation_law() {
    const lander::Config config{};
    const lander::BinarySystem bin = lander::BinarySystem::canonical(
        config.mu, 17ULL, lander::companion_seed(17ULL));

    check_close(bin.body_rotation(0.0), 0.0, 1.0e-12,
                "rotation is zero at t = 0");
    for (double t : {0.0, 1.0, 17.3, 60.0, 100.0, bin.period() * 0.5,
                     bin.period(), 400.0}) {
        check_close(bin.body_rotation(0, t), bin.omega_inner() * t, 1.0e-9,
                    "body 0 spins at the inner rate");
        check_close(bin.body_rotation(1, t), bin.omega_inner() * t, 1.0e-9,
                    "body 1 spins at the inner rate");
        check_close(bin.body_rotation(2, t), bin.omega_outer() * t, 1.0e-9,
                    "body 2 spins at the outer rate");
        check_close(bin.body_rotation(t), bin.omega_inner() * t, 1.0e-9,
                    "the unindexed accessor keeps the M05 body-0 form");
    }
    check_close(bin.body_spin_rate(0), bin.omega_inner(), 1e-15,
                "spin rate of body 0");
    check_close(bin.body_spin_rate(1), bin.omega_inner(), 1e-15,
                "spin rate of body 1");
    check_close(bin.body_spin_rate(2), bin.omega_outer(), 1e-15,
                "spin rate of body 2");

    // Finite-difference spin rates.
    const double h = 1.0e-4;
    for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
        const double t = 21.0;
        const double fd = (bin.body_rotation(i, t + h) -
                           bin.body_rotation(i, t - h)) /
                          (2.0 * h);
        check_close(fd, bin.body_spin_rate(i), 1.0e-6,
                    "finite-difference spin rate matches");
    }

    // Tidal-lock invariant, inner pair: each body sees its partner at a
    // fixed body-local direction for all time.
    for (int i = 0; i < 2; ++i) {
        const int other = 1 - i;
        const lander::Vec2 d0 =
            bin.position(other, 0.0) - bin.position(i, 0.0);
        const double ref = std::atan2(d0.y, d0.x);
        for (double tt : {5.0, 33.0, 90.0, 180.0, bin.period() * 0.75,
                          bin.period(), 300.0}) {
            const lander::Vec2 d =
                bin.position(other, tt) - bin.position(i, tt);
            const double world = std::atan2(d.y, d.x);
            check_close(
                std::atan2(std::sin(world - bin.body_rotation(i, tt) - ref),
                           std::cos(world - bin.body_rotation(i, tt) - ref)),
                0.0, 1.0e-9,
                "the partner keeps a fixed local direction (tidal lock)");
        }
    }

    // Tidal-lock invariant, moonlet: body 2 sees the inner barycentre at a
    // fixed body-local direction for all time.
    {
        const lander::Vec2 d0 =
            bin.barycentre_inner(0.0) - bin.position(2, 0.0);
        const double ref = std::atan2(d0.y, d0.x);
        const double T = bin.period_outer();
        for (double tt : {0.0, 20.0, 100.0, T * 0.25, T * 0.5, T, 3.0 * T}) {
            const lander::Vec2 d =
                bin.barycentre_inner(tt) - bin.position(2, tt);
            const double world = std::atan2(d.y, d.x);
            check_close(
                std::atan2(std::sin(world - bin.body_rotation(2, tt) - ref),
                           std::cos(world - bin.body_rotation(2, tt) - ref)),
                0.0, 1.0e-9,
                "the inner barycentre keeps a fixed moonlet-local "
                "direction (tidal lock)");
        }
    }
}

// M06-R13-V09: three-body spacecraft gravity: the total field is the sum
// of all three inverse-square fields, none ever switched off; the old
// two-body formula differs by exactly the body-2 term.
void test_three_body_gravity() {
    const lander::Config config{};
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(config.mu, 20ULL,
                                        lander::companion_seed(20ULL));
    const double mu2 = config.mu / 81.0;

    for (double t : {0.0, 50.0, 500.0}) {
        const lander::Vec2 p2 = bin.position(2, t);
        const std::vector<lander::Vec2> pts = {
            {0.0, 0.0},
            {0.5 * (bin.position(0, t).x + bin.position(1, t).x),
             0.5 * (bin.position(0, t).y + bin.position(1, t).y)},
            {p2.x + 100.0, p2.y},
            {-2000.0, 300.0},
        };
        for (const lander::Vec2& p : pts) {
            const lander::Vec2 g = bin.gravity(p, t);
            double ax = 0.0;
            double ay = 0.0;
            for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
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
            check_close(g.x, ax, 1e-9,
                        "combined field x equals the three-body sum");
            check_close(g.y, ay, 1e-9,
                        "combined field y equals the three-body sum");
        }
    }

    // Near the moonlet: the old two-body formula differs from the new
    // total by exactly the body-2 term, and that term is nonzero with the
    // exact inverse-square magnitude and direction.
    {
        const double t = 25.0;
        const lander::Vec2 p2 = bin.position(2, t);
        const lander::Vec2 p{p2.x + 100.0, p2.y};
        const lander::Vec2 total = bin.gravity(p, t);
        const lander::Vec2 two =
            bin.gravity_from(0, p, t) + bin.gravity_from(1, p, t);
        const lander::Vec2 g2 = bin.gravity_from(2, p, t);
        check_close(total.x - two.x, g2.x, 1e-12,
                    "new total minus old two-body field is the body-2 term "
                    "x");
        check_close(total.y - two.y, g2.y, 1e-12,
                    "new total minus old two-body field is the body-2 term "
                    "y");

        const double rx = p.x - p2.x;
        const double ry = p.y - p2.y;
        const double r = std::hypot(rx, ry);
        const lander::Vec2 expected{-mu2 * rx / (r * r * r),
                                    -mu2 * ry / (r * r * r)};
        check_close(g2.x, expected.x, 1e-9,
                    "body-2 field is inverse-square (x)");
        check_close(g2.y, expected.y, 1e-9,
                    "body-2 field is inverse-square (y)");
        check_close(std::hypot(g2.x, g2.y), mu2 / (r * r), 1e-12,
                    "body-2 field magnitude is mu2/r^2");
        check(std::hypot(g2.x, g2.y) > 1e-6,
              "body-2 field is nonzero near the moonlet (not switched "
              "off)");
    }

    // The field at fixed world points is the pure inverse-square sum from
    // the closed-form centre positions: the rotation adds no force.
    const lander::Vec2 probe{120.0, -60.0};
    for (double t : {0.0, 12.0, 95.0}) {
        const lander::Vec2 g = bin.gravity(probe, t);
        lander::Vec2 sum{};
        for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
            const lander::Body& b = bin.body(i);
            const lander::Vec2 q = bin.position(i, t);
            const double rx = probe.x - q.x;
            const double ry = probe.y - q.y;
            const double r3 = std::pow(std::hypot(rx, ry), 3.0);
            sum = sum + lander::Vec2{-b.mu * rx / r3, -b.mu * ry / r3};
        }
        check_close(g.x, sum.x, 1.0e-9,
                    "gravity x is the pure inverse-square sum");
        check_close(g.y, sum.y, 1.0e-9,
                    "gravity y is the pure inverse-square sum");
    }
}

// M06-R13-V10: the legacy M05 accessors keep their meanings and values;
// the new hierarchical accessors match the model.
void test_legacy_accessors() {
    const lander::Config config{};
    const double mu0 = config.mu;
    const double mu1 = mu0 / 81.0;
    const double mu_inner = mu0 + mu1;
    const double mu_total = mu_inner + mu1;
    const lander::BinarySystem bin =
        lander::BinarySystem::canonical(mu0, 21ULL,
                                        lander::companion_seed(21ULL));

    // Unchanged M05 values.
    check_close(bin.separation(), 600.0, 1e-9, "separation() is D01 = 600");
    check_close(bin.mu_system(), mu_inner, 1e-9,
                "mu_system() is the inner-pair mu (M05 value)");
    const double omega = std::sqrt(mu_inner / (600.0 * 600.0 * 600.0));
    check_close(bin.omega(), omega, 1e-12, "omega() is the M05 rate");
    check_close(bin.period(), 2.0 * lander::kPi / omega, 1e-6,
                "period() is 2*pi/omega (M05 value)");
    check_close(bin.period(), 216.94244, 0.01,
                "binary period is still ~ 216.94 s");
    check_close(bin.d_inner(), 600.0, 1e-9, "d_inner() is D01");
    check_close(bin.omega_inner(), bin.omega(), 1e-15,
                "omega_inner() equals the legacy omega()");
    check_close(bin.period_inner(), bin.period(), 1e-9,
                "period_inner() equals the legacy period()");
    for (double t : {0.0, 3.7, 99.0}) {
        check_close(bin.theta(t), bin.theta_inner(t), 1e-12,
                    "theta() equals theta_inner()");
    }

    // New outer-hierarchy values.
    check_close(bin.d_outer(), 1200.0, 1e-9, "d_outer() is D_OUTER = 1200");
    check_close(bin.mu_inner(), mu_inner, 1e-9, "mu_inner() is mu0 + mu1");
    check_close(bin.mu_outer_system(), mu_total, 1e-9,
                "mu_outer_system() is mu0 + mu1 + mu2");
    const double omega_o =
        std::sqrt(mu_total / (1200.0 * 1200.0 * 1200.0));
    check_close(bin.omega_outer(), omega_o, 1e-15,
                "omega_outer() = sqrt(mu_total / 1200^3)");
    check_close(bin.period_outer(), 2.0 * lander::kPi / omega_o, 1e-6,
                "period_outer() = 2*pi/omega_outer");
    check_close(bin.period_outer(), 610.13, 0.5,
                "outer period is ~ 610.13 s");
    check_close(bin.a0_inner(), 600.0 * mu1 / mu_inner, 1e-9,
                "a0_inner() is the M05 primary barycentric radius");
    check_close(bin.a1_inner(), 600.0 * mu0 / mu_inner, 1e-9,
                "a1_inner() is the M05 companion barycentric radius");
    check_close(bin.a_inner_outer(), 1200.0 * mu1 / mu_total, 1e-9,
                "a_inner_outer() is the B01 circle radius");
    check_close(bin.a2_outer(), 1200.0 * mu_inner / mu_total, 1e-9,
                "a2_outer() is the body-2 circle radius");

    // B01 at t = 0: phase pi/2 -> (0, -a_inner_outer).
    const lander::Vec2 b0 = bin.barycentre_inner(0.0);
    check_close(b0.x, 0.0, 1e-12, "B01(0) x is zero");
    check_close(b0.y, -1200.0 * mu1 / mu_total, 1e-9,
                "B01(0) y is -a_inner_outer (phase pi/2)");
}

// M05-R1-V04 (preserved): relative kinematics subtract the body's
// position and velocity; generalized to the three-body ephemeris.
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

    // A globally stationary ship beside the moving companion has a local
    // velocity exactly the negative of the companion's centre velocity
    // decomposed: radial = -bvel.x (the outer-orbit drift of the centre at
    // t = 0) and tangential = -bvel.y * -1 = the inner orbital speed.
    {
        const lander::Vec2 cpos = bin.position(1, 0.0);
        lander::State s{};
        s.x = cpos.x + 30.0;
        s.y = cpos.y;
        s.vx = 0.0;
        s.vy = 0.0;
        const lander::Vec2 bvel = bin.velocity(1, 0.0);
        const lander::LocalVelocity lv = lander::local_velocity(s, cpos, bvel);
        const double mu0 = config.mu;
        const double mu1 = mu0 / 81.0;
        const double mu_inner = mu0 + mu1;
        const double mu_total = mu_inner + mu1;
        const double omega_i = std::sqrt(mu_inner / (600.0 * 600.0 * 600.0));
        const double omega_o =
            std::sqrt(mu_total / (1200.0 * 1200.0 * 1200.0));
        const double a1 = 600.0 * mu0 / mu_inner;
        const double aio = 1200.0 * mu1 / mu_total;
        check_close(lv.radial, -aio * omega_o, 1e-9,
                    "stationary ship sees the centre's outer-orbit drift "
                    "radially");
        check_close(lv.tangential, a1 * omega_i, 1e-9,
                    "stationary ship sees the companion's inner orbital "
                    "speed tangentially");
        check_close(lander::radial_distance(s, cpos), 30.0, 1e-9,
                    "stationary ship is 30 m from the companion");
    }
}

// M05-R3-V12 (preserved): the local<->world angle transform and its
// inverse, per body with the per-body spin.
void test_local_world_angle_transform() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 402ULL, lander::companion_seed(402ULL));

    for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
        const lander::Body& body = bin.body(i);
        const double c = body.terrain.circumference();
        for (double t : {0.0, 3.7, 40.0, bin.period() * 0.625}) {
            const double rot = bin.body_rotation(i, t);
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

// M05-R3-V13 (preserved): surface-point position and velocity, including
// the finite-difference derivative of the position, per body.
void test_surface_point_velocity() {
    const lander::Config config{};
    const auto bin = lander::BinarySystem::canonical(
        config.mu, 403ULL, lander::companion_seed(403ULL));

    for (int i = 0; i < lander::BinarySystem::kBodyCount; ++i) {
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
                const double world = local + bin.body_rotation(i, t);
                check_close(sp.position.x, cpos.x + std::cos(world) * radius,
                            1.0e-9, "surface point position x");
                check_close(sp.position.y, cpos.y + std::sin(world) * radius,
                            1.0e-9, "surface point position y");
                const double ox = std::cos(world) * radius;
                const double oy = std::sin(world) * radius;
                const double w = bin.body_spin_rate(i);
                check_close(sp.velocity.x, cvel.x - w * oy, 1.0e-9,
                            "surface point velocity x is centre plus spin");
                check_close(sp.velocity.y, cvel.y + w * ox, 1.0e-9,
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

}  // namespace

int main() {
    test_three_body_construction();
    test_total_barycentre();
    test_inner_separation();
    test_inner_pair_preservation();
    test_analytic_derivatives();
    test_outer_orbit_closed_form();
    test_one_period_return();
    test_body_rotation_law();
    test_three_body_gravity();
    test_legacy_accessors();
    test_relative_kinematics();
    test_local_world_angle_transform();
    test_surface_point_velocity();

    if (failures == 0) {
        std::puts("All lander_binary_tests passed");
        return 0;
    }
    std::printf("%d lander_binary_tests failed\n", failures);
    return 1;
}
