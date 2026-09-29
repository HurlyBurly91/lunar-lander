#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace lander {

struct Star {
    double x{};
    double y{};
    int size{1};
    int bright{255};

    bool operator==(const Star&) const = default;
    bool operator!=(const Star&) const = default;
};

struct ScreenPoint {
    double x{};
    double y{};

    bool operator==(const ScreenPoint&) const = default;
};

// splitmix64 step: advances `state` in place and returns the next output
// word. Shared by the starfield generator and the GUI's deterministic crash
// debris so both stay pure functions of their seeds.
inline std::uint64_t splitmix64_next(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// Deterministic starfield generator (M05-R2-01, inertial upgrade
// M05-R3-12). `make_stars` fills a full-viewport disk of radius
// 0.5 * hypot(width, height) centred on the viewport, so the base positions
// cover the viewport at any presentation rotation; positions are the stars'
// screen positions at camera angle 0. The generator is pure: same seed and
// viewport produce bit-identical stars, different seeds produce different
// fields, and nothing in it is rendering-dependent.
inline std::vector<Star> make_stars(std::uint64_t seed, double width,
                                    double height) {
    constexpr int kStarCount = 280;
    std::vector<Star> stars;
    stars.reserve(kStarCount);

    std::uint64_t state = seed * 0x9E3779B97F4A7C15ULL ^ 0xA5A4801A5A480193ULL;
    auto uniform = [&state]() {
        return static_cast<double>(splitmix64_next(state) >> 11) *
               0x1.0p-53;
    };

    const double cx = 0.5 * width;
    const double cy = 0.5 * height;
    const double radius = 0.5 * std::hypot(width, height);
    for (int i = 0; i < kStarCount; ++i) {
        const double u1 = uniform();
        const double u2 = uniform();
        const double u3 = uniform();
        const double r = radius * std::sqrt(u1);
        const double theta = 2.0 * 3.14159265358979323846 * u2;
        Star star;
        star.x = cx + r * std::cos(theta);
        star.y = cy + r * std::sin(theta);
        star.size = u3 < 0.15 ? 2 : 1;
        star.bright = 90 + static_cast<int>(u3 * 140.0);
        stars.push_back(star);
    }
    return stars;
}

// M05-R3-12: presentation position of a star. The starfield is an inertial
// backdrop: it carries the scene's rotation (the final presentation camera
// angle, identical in LOCAL and SYSTEM) but no translation, so stars never
// parallax with the world and never depend on camera mode, reference body,
// phase, or pan. The base position is the star's screen position at angle
// 0, so the transform is the same rotation the rendered world undergoes,
// applied about the viewport centre: with c = cos(a), s = sin(a),
//   sx = w/2 + (c*dx - s*dy)
//   sy = h/2 + (s*dx + c*dy)
// where (dx, dy) = (star.x - w/2, star.y - h/2). This is an involution-free
// rotation: a 2*pi angle returns every star exactly to its generated
// position, and changing the angle by d rotates the pattern by d about the
// centre, exactly like the world.
inline ScreenPoint star_screen_pos(const Star& star, double width,
                                   double height, double angle) {
    const double dx = star.x - 0.5 * width;
    const double dy = star.y - 0.5 * height;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {width * 0.5 + (c * dx - s * dy),
            height * 0.5 + (s * dx + c * dy)};
}

}  // namespace lander
