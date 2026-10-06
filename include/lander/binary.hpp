#pragma once

#include "lander/terrain.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace lander {

struct Vec2 {
    double x{};
    double y{};

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
};

// One moon of the three-body system. `terrain` is the body's surface in
// body-local coordinates; a body-local angle becomes a world angle by
// adding that body's tidal-lock spin (BinarySystem::body_rotation(index,
// t)). `mu` is the gravitational parameter.
struct Body {
    double reference_radius{kReferenceRadius};
    double mu{0.0};
    Terrain terrain{};
};

// Derive the companion's terrain seed from the game seed. A salted
// splitmix64-style finalizer keeps the companion layout independent of
// the primary's while remaining a pure function of the game seed, so a
// reset with the same seed reproduces the terrain exactly.
inline std::uint64_t companion_seed(std::uint64_t seed) {
    std::uint64_t x = seed ^ 0x5851F42D0C0A75E2ULL;
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// M06-R13: derive the outer moonlet's terrain seed from the primary game
// seed with a different salt, so all three terrains are distinct pure
// functions of the game seed and a reset with the same seed reproduces
// all three exactly.
inline std::uint64_t moonlet_seed(std::uint64_t seed) {
    std::uint64_t x = seed ^ 0x6A5A2E7E4B7D9C35ULL;
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

// Closed-form hierarchical (Jacobi) ephemeris for the three-moon system:
// the inner pair (body 0 = primary, body 1 = companion) keeps the M05
// relative two-body motion about the inner barycentre B01 (fixed
// separation D01, angular rate omega_inner); the outer pair (B01 and
// body 2 = outer moonlet) orbits the total barycentre on a fixed circle
// D_OUTER at omega_outer, starting at pi/2. The total barycentre stays
// at the origin for all time. This is a prescribed analytic orbit (no
// N-body integration of the bodies).
//
// Inner pair:
//     mu_inner      = mu0 + mu1
//     omega_inner   = sqrt(mu_inner / D01^3)
//     theta_inner   = omega_inner * t
//     P0 = B01 - a0_inner * (cos, sin)(theta_inner)
//     P1 = B01 + a1_inner * (cos, sin)(theta_inner)
//     a0_inner = D01 * mu1 / mu_inner,  a1_inner = D01 * mu0 / mu_inner
//
// Outer pair:
//     mu_outer_system = mu_inner + mu2
//     omega_outer     = sqrt(mu_outer_system / D_OUTER^3)
//     theta_outer     = pi/2 + omega_outer * t
//     P2  = +a2_outer * (cos, sin)(theta_outer)
//     B01 = -a_inner_outer * (cos, sin)(theta_outer)
//     a2_outer = D_OUTER * mu_inner / mu_outer_system
//     a_inner_outer = D_OUTER * mu2 / mu_outer_system
//
// M05-R3 tidal locking, generalized per body: bodies 0 and 1 spin
// prograde with the inner orbit (omega_inner), body 2 with the outer
// orbit (omega_outer), so the same face of each body always points
// along its own orbital motion. A point fixed on a surface then carries
// the body's translational velocity plus the spin velocity of its
// offset (see surface_point).
class BinarySystem {
public:
    static constexpr int kBodyCount = 3;

    BinarySystem() = default;

    // The M06-R13 canonical three-body system: the primary is the M04
    // reference moon (mu0, R0); the companion is 1/9 the radius with
    // mu/81 and 1/9 relief scale (M05), so its intrinsic surface gravity
    // matches the primary's; the outer moonlet has the companion's scale
    // with a terrain derived from a distinct salt of the primary seed.
    static BinarySystem canonical(double mu0, std::uint64_t primary_seed,
                                  std::uint64_t companion_seed) {
        constexpr double kCompanionScale = 1.0 / 9.0;
        const double r1 = kReferenceRadius * kCompanionScale;
        const double mu1 = mu0 * kCompanionScale * kCompanionScale;
        const double mu_inner = mu0 + mu1;
        const double mu_total = mu_inner + mu1;
        constexpr double kDInner = 600.0;
        constexpr double kDOuter = 1200.0;

        BinarySystem system;
        system.d_inner_ = kDInner;
        system.d_outer_ = kDOuter;
        system.mu_inner_ = mu_inner;
        system.mu_outer_system_ = mu_total;
        system.omega_inner_ =
            std::sqrt(mu_inner / (kDInner * kDInner * kDInner));
        system.omega_outer_ =
            std::sqrt(mu_total / (kDOuter * kDOuter * kDOuter));
        system.a0_inner_ = kDInner * mu1 / mu_inner;
        system.a1_inner_ = kDInner * mu0 / mu_inner;
        system.a_inner_outer_ = kDOuter * mu1 / mu_total;
        system.a2_outer_ = kDOuter * mu_inner / mu_total;

        Body primary;
        primary.reference_radius = kReferenceRadius;
        primary.mu = mu0;
        primary.terrain = Terrain(primary_seed);
        system.bodies_[0] = primary;

        Body companion;
        companion.reference_radius = r1;
        companion.mu = mu1;
        companion.terrain = Terrain(companion_seed, r1, kCompanionScale);
        system.bodies_[1] = companion;

        Body moonlet;
        moonlet.reference_radius = r1;
        moonlet.mu = mu1;
        moonlet.terrain = Terrain(moonlet_seed(primary_seed), r1,
                                  kCompanionScale);
        system.bodies_[2] = moonlet;
        return system;
    }

    // M05 legacy accessors: the inner pair's circular-orbit parameters.
    // `mu_system` / `omega` / `period` / `theta` / `separation` keep their
    // M05 meanings (the relative 0/1 motion is the unchanged M05
    // two-body solution).
    double separation() const noexcept { return d_inner_; }
    double mu_system() const noexcept { return mu_inner_; }
    double omega() const noexcept { return omega_inner_; }
    double period() const noexcept { return kTwoPi / omega_inner_; }
    double theta(double t) const noexcept { return theta_inner(t); }

    // M06-R13 hierarchical parameters.
    double d_inner() const noexcept { return d_inner_; }
    double d_outer() const noexcept { return d_outer_; }
    double mu_inner() const noexcept { return mu_inner_; }
    double mu_outer_system() const noexcept { return mu_outer_system_; }
    double omega_inner() const noexcept { return omega_inner_; }
    double omega_outer() const noexcept { return omega_outer_; }
    double period_inner() const noexcept { return kTwoPi / omega_inner_; }
    double period_outer() const noexcept { return kTwoPi / omega_outer_; }
    double a0_inner() const noexcept { return a0_inner_; }
    double a1_inner() const noexcept { return a1_inner_; }
    double a_inner_outer() const noexcept { return a_inner_outer_; }
    double a2_outer() const noexcept { return a2_outer_; }

    double theta_inner(double t) const noexcept {
        return omega_inner_ * t;
    }
    double theta_outer(double t) const noexcept {
        return 0.5 * kPi + omega_outer_ * t;
    }

    // The inner-pair barycentre (origin at t's outer phase).
    Vec2 barycentre_inner(double t) const noexcept {
        const double th = theta_outer(t);
        return {-a_inner_outer_ * std::cos(th), -a_inner_outer_ * std::sin(th)};
    }

    // M05-R3 tidal locking, generalized per body: bodies 0 and 1 spin
    // at the inner rate, body 2 at the outer rate, so a body-fixed
    // surface point keeps a fixed face along its own orbital motion for
    // all time. The spin is the pure time function (omega * t). The
    // unindexed overload is the body-0 / M05 convenience form.
    double body_rotation(int index, double t) const noexcept {
        return (index == 2) ? omega_outer_ * t : omega_inner_ * t;
    }
    double body_spin_rate(int index) const noexcept {
        return (index == 2) ? omega_outer_ : omega_inner_;
    }
    double body_rotation(double t) const noexcept {
        return theta_inner(t);
    }

    struct SurfacePoint {
        Vec2 position{};
        Vec2 velocity{};
    };

    // Position and inertial velocity of a point fixed on body `index` at
    // body-local angle `local_angle` and radius `radius`, at ephemeris
    // time `t`: the body centre plus the spin-rotated offset, and the
    // body's translational velocity plus the spin velocity of that
    // offset (body_spin_rate(index) x offset).
    SurfacePoint surface_point(int index, double local_angle, double radius,
                               double t) const {
        const Vec2 pos = position(index, t);
        const Vec2 vel = velocity(index, t);
        const double rot = body_rotation(index, t);
        const double w = body_spin_rate(index);
        const double ox = std::cos(local_angle + rot) * radius;
        const double oy = std::sin(local_angle + rot) * radius;
        return {
            {pos.x + ox, pos.y + oy},
            {vel.x - w * oy, vel.y + w * ox},
        };
    }

    const Body& body(int index) const { return bodies_[index]; }
    const std::array<Body, kBodyCount>& bodies() const { return bodies_; }

    Vec2 position(int index, double t) const {
        if (index < 2) {
            const double th = theta_inner(t);
            const Vec2 b = barycentre_inner(t);
            const double a = (index == 0) ? a0_inner_ : a1_inner_;
            const double s = (index == 0) ? -1.0 : 1.0;
            return {b.x + s * a * std::cos(th),
                    b.y + s * a * std::sin(th)};
        }
        const double th = theta_outer(t);
        return {a2_outer_ * std::cos(th), a2_outer_ * std::sin(th)};
    }

    Vec2 velocity(int index, double t) const {
        if (index < 2) {
            const double th = theta_inner(t);
            const double th_o = theta_outer(t);
            const double a = (index == 0) ? a0_inner_ : a1_inner_;
            const double s = (index == 0) ? -1.0 : 1.0;
            return {a_inner_outer_ * omega_outer_ * std::sin(th_o) -
                        s * a * omega_inner_ * std::sin(th),
                    -a_inner_outer_ * omega_outer_ * std::cos(th_o) +
                        s * a * omega_inner_ * std::cos(th)};
        }
        const double th = theta_outer(t);
        return {-a2_outer_ * omega_outer_ * std::sin(th),
                a2_outer_ * omega_outer_ * std::cos(th)};
    }

    // The body's prescribed barycentric orbital acceleration: the outer
    // circular term of its centre (all bodies translate with B01 or orbit
    // the origin) plus, for bodies 0/1, the inner circular term of its
    // offset about B01. For a body on a circle through a point r this is
    // -omega^2 * r for each component.
    Vec2 acceleration(int index, double t) const {
        if (index < 2) {
            const Vec2 b = barycentre_inner(t);
            const Vec2 p = position(index, t);
            const double wo2 = omega_outer_ * omega_outer_;
            const double wi2 = omega_inner_ * omega_inner_;
            return {-wo2 * b.x - wi2 * (p.x - b.x),
                    -wo2 * b.y - wi2 * (p.y - b.y)};
        }
        const Vec2 p = position(index, t);
        const double w2 = omega_outer_ * omega_outer_;
        return {-w2 * p.x, -w2 * p.y};
    }

    // Vector sum of all three bodies' inverse-square gravitational
    // fields at p. All fields are always active; there is no SOI
    // switching or stabilization of any kind.
    Vec2 gravity(const Vec2& p, double t) const {
        Vec2 acc{};
        for (int i = 0; i < kBodyCount; ++i) {
            acc = acc + gravity_from(i, p, t);
        }
        return acc;
    }

    // The inverse-square gravitational field produced by one body at p.
    Vec2 gravity_from(int index, const Vec2& p, double t) const {
        const Body& b = bodies_[index];
        const Vec2 pos = position(index, t);
        const double rx = p.x - pos.x;
        const double ry = p.y - pos.y;
        const double r = std::hypot(rx, ry);
        if (r < 1.0e-9) {
            return {};
        }
        const double inv_r3 = 1.0 / (r * r * r);
        return {-b.mu * rx * inv_r3, -b.mu * ry * inv_r3};
    }

private:
    double d_inner_{600.0};
    double d_outer_{1200.0};
    double mu_inner_{0.0};
    double mu_outer_system_{0.0};
    double omega_inner_{0.0};
    double omega_outer_{0.0};
    double a0_inner_{0.0};
    double a1_inner_{0.0};
    double a_inner_outer_{0.0};
    double a2_outer_{0.0};
    std::array<Body, kBodyCount> bodies_;
};

}  // namespace lander
