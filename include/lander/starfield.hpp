#pragma once

// A small, deterministic, cosmetic starfield for the Lunar Lander GUI.
//
// The stars are treated as infinitely distant: a star's screen position is
// fixed in viewport/celestial background space and is independent of camera
// translation and zoom. The generation stream is deliberately separate from
// the simulation's RNG, so it never influences physics.
//
// This header is intentionally rendering-free (no SDL) so the star behaviour
// can be tested headlessly.

#include <cmath>
#include <cstdint>
#include <vector>

namespace lander {

struct Star {
    double x{};  // fixed screen-space position, pixels from the left
    double y{};  // fixed screen-space position, pixels from the top
    int size{};
    std::uint8_t bright{};

    bool operator==(const Star&) const = default;
};

struct ScreenPoint {
    double x{};
    double y{};
};

// SplitMix64 step, shared by the starfield and any other cosmetic,
// seed-derived decoration in the GUI.
inline std::uint64_t splitmix64_next(std::uint64_t& state) {
    state += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

// Generate a fixed starfield for a viewport of the given size. The result is
// deterministic for a given seed and viewport.
inline std::vector<Star> make_stars(std::uint64_t seed, double width,
                                    double height) {
    std::uint64_t state =
        seed * 0x9E3779B97F4A7C15ULL ^ 0xA5A4801A5A480193ULL;
    std::vector<Star> stars;
    stars.reserve(140);
    for (int i = 0; i < 140; ++i) {
        const double u1 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u2 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        const double u3 =
            static_cast<double>(splitmix64_next(state) >> 11) * 0x1.0p-53;
        Star s;
        s.x = u1 * width;
        s.y = u2 * height;
        s.size = u3 < 0.15 ? 2 : 1;
        s.bright = static_cast<std::uint8_t>(90 + static_cast<int>(u3 * 140));
        stars.push_back(s);
    }
    return stars;
}

// Screen position of a star. The camera translation and zoom arguments are
// intentionally ignored: infinitely-distant stars do not move with camera
// translation or zoom. The camera angle rotates the fixed star pattern around
// the viewport center so the local horizon stays aligned with the camera.
inline ScreenPoint star_screen_pos(const Star& star,
                                   double center_x,
                                   double center_y,
                                   double camera_angle,
                                   double camera_x = 0.0,
                                   double camera_y = 0.0,
                                   double scale = 1.0) {
    (void)camera_x;
    (void)camera_y;
    (void)scale;
    double dx = star.x - center_x;
    double dy = star.y - center_y;
    double c = std::cos(camera_angle);
    double s = std::sin(camera_angle);
    return {center_x + dx * c - dy * s,
            center_y + dx * s + dy * c};
}

}  // namespace lander
