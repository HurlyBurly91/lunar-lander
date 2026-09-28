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

class Terrain {
public:
    Terrain() = default;
    explicit Terrain(std::uint64_t seed);

    static constexpr double reference_radius() noexcept { return kReferenceRadius; }
    static constexpr double circumference() noexcept { return kReferenceCircumference; }
    static constexpr double spawn_angle() noexcept { return kSpawnAngle; }

    static double normalize_arc(double arc) {
        double n = std::fmod(arc, kReferenceCircumference);
        if (n < 0.0) {
            n += kReferenceCircumference;
        }
        return n;
    }

    static double arc_at_angle(double theta) {
        double d = std::fmod(theta - kSpawnAngle + kPi, kTwoPi);
        if (d < 0.0) {
            d += kTwoPi;
        }
        d -= kPi;
        double arc = d * kReferenceRadius;
        if (arc < 0.0) {
            arc += kReferenceCircumference;
        }
        return arc;
    }

    static double angle_at_arc(double arc) {
        return kSpawnAngle + arc / kReferenceRadius;
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
    std::vector<Pad> pads_;
};

}  // namespace lander
