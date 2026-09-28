#include "lander/starfield.hpp"
#include "lander/terrain.hpp"

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

}  // namespace

int main() {
    const double width = 1280.0;
    const double height = 720.0;
    const double cx = width / 2.0;
    const double cy = height / 2.0;

    const auto a = lander::make_stars(1234, width, height);
    const auto b = lander::make_stars(1234, width, height);
    const auto c = lander::make_stars(4321, width, height);

    check(a.size() == 140, "starfield should have 140 stars");
    check(a == b, "same seed should generate the same starfield");
    check(a != c, "different seeds should generate different starfields");

    for (const lander::Star& star : a) {
        check(star.x >= 0.0 && star.x <= width, "star x within viewport");
        check(star.y >= 0.0 && star.y <= height, "star y within viewport");
        check(star.size == 1 || star.size == 2, "star size is 1 or 2");

        const lander::ScreenPoint base =
            lander::star_screen_pos(star, cx, cy, 0.0);
        check_close(base.x, star.x, 1e-12, "zero angle leaves x unchanged");
        check_close(base.y, star.y, 1e-12, "zero angle leaves y unchanged");

        const lander::ScreenPoint translated =
            lander::star_screen_pos(star, cx, cy, 0.0, 100.0, 50.0, 2.0);
        check_close(translated.x, star.x, 1e-12,
                    "translation/zoom ignored for x");
        check_close(translated.y, star.y, 1e-12,
                    "translation/zoom ignored for y");

        const double dx = star.x - cx;
        const double dy = star.y - cy;
        for (double angle : {0.1, -0.4, 1.0, 2.2, -2.9}) {
            const lander::ScreenPoint p =
                lander::star_screen_pos(star, cx, cy, angle);
            const double expected_x =
                cx + dx * std::cos(angle) - dy * std::sin(angle);
            const double expected_y =
                cy + dx * std::sin(angle) + dy * std::cos(angle);
            check_close(p.x, expected_x, 1e-9, "star rotates with camera");
            check_close(p.y, expected_y, 1e-9, "star rotates with camera");
        }
    }

    lander::Star probe;
    probe.x = cx + 100.0;
    probe.y = cy;
    const lander::ScreenPoint quarter =
        lander::star_screen_pos(probe, cx, cy, 0.5 * lander::kPi);
    check_close(quarter.x, cx, 1e-9, "quarter rotation x");
    check_close(quarter.y, cy + 100.0, 1e-9, "quarter rotation y");

    const lander::ScreenPoint half =
        lander::star_screen_pos(probe, cx, cy, lander::kPi);
    check_close(half.x, cx - 100.0, 1e-9, "half rotation x");
    check_close(half.y, cy, 1e-9, "half rotation y");

    if (failures == 0) {
        std::puts("All lander_starfield_tests passed");
        return 0;
    }
    std::printf("%d lander_starfield_tests failed\n", failures);
    return 1;
}
