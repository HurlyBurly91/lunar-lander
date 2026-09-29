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

// One moon of the binary system. `terrain` is the body's surface in
// body-local coordinates (the bodies do not spin, so body-local angles are
// world angles), `mu` is the gravitational parameter, and
// `side * barycentric_radius` places the body on its side of the fixed
// barycentre (-1 = primary, +1 = companion).
struct Body {
    double reference_radius{kReferenceRadius};
    double mu{0.0};
    double barycentric_radius{0.0};
    int side{-1};
    Terrain terrain{};
};

// Closed-form ephemeris for two moons orbiting a fixed barycentre at the
// origin: theta(t) = theta0 + omega * t with each body on a fixed circular
// radius on opposite sides. This is a prescribed analytic orbit (no N-body
// integration) that obeys ordinary two-body mechanics:
//
//     mu_system = mu0 + mu1
//     omega     = sqrt(mu_system / D^3)
//     a_i       = D * mu_other / mu_system
//
// so the barycentre stays at the origin for all time. The bodies translate
// but do not spin (no axial rotation in M05), so a point fixed on a surface
// carries the body's translational velocity.
class BinarySystem {
public:
    BinarySystem() = default;

    // The M05 canonical binary: the primary is the M04 reference moon
    // (mu0, R0); the companion is 1/9 the radius with mu/81 and 1/9 relief
    // scale, so its intrinsic surface gravity matches the primary's.
    static BinarySystem canonical(double mu0, std::uint64_t primary_seed,
                                  std::uint64_t companion_seed) {
        constexpr double kCompanionScale = 1.0 / 9.0;
        const double r1 = kReferenceRadius * kCompanionScale;
        const double mu1 = mu0 * kCompanionScale * kCompanionScale;
        const double separation = 600.0;
        const double mu_total = mu0 + mu1;
        const double omega =
            std::sqrt(mu_total / (separation * separation * separation));

        BinarySystem system;
        system.separation_ = separation;
        system.mu_system_ = mu_total;
        system.omega_ = omega;
        system.theta0_ = 0.0;

        Body primary;
        primary.reference_radius = kReferenceRadius;
        primary.mu = mu0;
        primary.barycentric_radius = mu1 * separation / mu_total;
        primary.side = -1;
        primary.terrain = Terrain(primary_seed);
        system.bodies_[0] = primary;

        Body companion;
        companion.reference_radius = r1;
        companion.mu = mu1;
        companion.barycentric_radius = mu0 * separation / mu_total;
        companion.side = +1;
        companion.terrain = Terrain(companion_seed, r1, kCompanionScale);
        system.bodies_[1] = companion;
        return system;
    }

    double separation() const noexcept { return separation_; }
    double mu_system() const noexcept { return mu_system_; }
    double omega() const noexcept { return omega_; }
    double period() const noexcept { return kTwoPi / omega_; }

    double theta(double t) const noexcept { return theta0_ + omega_ * t; }

    const Body& body(int index) const { return bodies_[index]; }
    const std::array<Body, 2>& bodies() const { return bodies_; }

    Vec2 position(int index, double t) const {
        const Body& b = bodies_[index];
        const double th = theta(t);
        return {b.side * b.barycentric_radius * std::cos(th),
                b.side * b.barycentric_radius * std::sin(th)};
    }

    Vec2 velocity(int index, double t) const {
        const Body& b = bodies_[index];
        const double th = theta(t);
        return {-b.side * b.barycentric_radius * omega_ * std::sin(th),
                b.side * b.barycentric_radius * omega_ * std::cos(th)};
    }

    // The body's prescribed barycentric orbital acceleration
    // (-omega^2 * position), equal to the other body's inverse-square field
    // on the circular orbit. Used to convert inertial accelerations into a
    // body's non-inertial frame (for example the takeoff check).
    Vec2 acceleration(int index, double t) const {
        const Vec2 p = position(index, t);
        const double w2 = omega_ * omega_;
        return {-w2 * p.x, -w2 * p.y};
    }

    // Vector sum of both bodies' inverse-square gravitational fields at p.
    // Both fields are always active; there is no SOI switching or
    // stabilization of any kind.
    Vec2 gravity(const Vec2& p, double t) const {
        Vec2 acc{};
        for (int i = 0; i < 2; ++i) {
            const Body& b = bodies_[i];
            const Vec2 pos = position(i, t);
            const double rx = p.x - pos.x;
            const double ry = p.y - pos.y;
            const double r = std::hypot(rx, ry);
            if (r < 1.0e-9) {
                continue;
            }
            const double inv_r3 = 1.0 / (r * r * r);
            acc.x -= b.mu * rx * inv_r3;
            acc.y -= b.mu * ry * inv_r3;
        }
        return acc;
    }

private:
    double separation_{600.0};
    double mu_system_{0.0};
    double omega_{0.0};
    double theta0_{0.0};
    std::array<Body, 2> bodies_;
};

// Derive the companion's terrain seed from the game seed. A salted
// splitmix64-style finalizer keeps the companion layout independent of the
// primary's while remaining a pure function of the game seed, so a reset
// with the same seed reproduces both terrains exactly.
inline std::uint64_t companion_seed(std::uint64_t seed) {
    std::uint64_t x = seed ^ 0x5851F42D0C0A75E2ULL;
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

}  // namespace lander
