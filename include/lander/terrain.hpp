#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace lander {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kReferenceRadius = 332.384;
constexpr double kReferenceCircumference = kTwoPi * kReferenceRadius;
constexpr double kSpawnAngle = 0.5 * kPi;

struct Pad {
    double center_arc{};
    double half_width{6.0};
    double radius{};
    int multiplier{1};
    bool operator==(const Pad&) const = default;
};

// A closed circular terrain body. `reference_radius` sets the body's size and
// `height_scale` scales the natural terrain relief relative to that radius.
// The default constructor/arguments reproduce the M04 reference moon exactly:
// for a given seed, every generated value (pads, base heights, surface
// radii) is bit-identical to the M04 Terrain.
class Terrain {
public:
    Terrain() = default;
    Terrain(std::uint64_t seed, double reference_radius = kReferenceRadius,
            double height_scale = 1.0);

    static constexpr double spawn_angle() noexcept { return kSpawnAngle; }

    double reference_radius() const noexcept { return reference_radius_; }
    double circumference() const noexcept { return circumference_; }

    double normalize_arc(double arc) const {
        double n = std::fmod(arc, circumference_);
        if (n < 0.0) {
            n += circumference_;
        }
        return n;
    }

    double arc_at_angle(double theta) const {
        double d = std::fmod(theta - kSpawnAngle + kPi, kTwoPi);
        if (d < 0.0) {
            d += kTwoPi;
        }
        d -= kPi;
        double arc = d * reference_radius_;
        if (arc < 0.0) {
            arc += circumference_;
        }
        return arc;
    }

    double angle_at_arc(double arc) const {
        return kSpawnAngle + arc / reference_radius_;
    }

    double surface_radius_at_arc(double arc) const;
    double surface_radius_at_angle(double theta) const;
    double base_radius_at_arc(double arc) const;
    const Pad* pad_at_arc(double arc) const;
    double max_surface_radius() const;

    const std::vector<Pad>& pads() const noexcept { return pads_; }
    std::uint64_t seed() const noexcept { return seed_; }

private:
    double base_height_at_arc(double arc) const;

    std::uint64_t seed_{0};
    double reference_radius_{kReferenceRadius};
    double height_scale_{1.0};
    double circumference_{kReferenceCircumference};
    std::vector<Pad> pads_;
};

}  // namespace lander
