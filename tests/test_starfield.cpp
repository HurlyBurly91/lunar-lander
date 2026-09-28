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
        check_close(base.x, star.x, 1e-12, "screen x is the generated x");
        check_close(base.y, star.y, 1e-12, "screen y is the generated y");

        for (double camera_x : {0.0, 100.0, -250.0}) {
            for (double camera_y : {0.0, 50.0, 900.0}) {
                for (double scale : {0.2, 1.0, 4.0}) {
                    for (double angle : {0.0, 0.1, -0.4, 1.0, 2.2, -2.9,
                                         lander::kPi}) {
                        const lander::ScreenPoint p =
                            lander::star_screen_pos(star, cx, cy, angle,
                                                    camera_x, camera_y,
                                                    scale);
                        check_close(p.x, star.x, 1e-12,
                                    "camera x/y/angle/scale leaves x fixed");
                        check_close(p.y, star.y, 1e-12,
                                    "camera x/y/angle/scale leaves y fixed");
                    }
                }
            }
        }
    }

    if (failures == 0) {
        std::puts("All lander_starfield_tests passed");
        return 0;
    }
    std::printf("%d lander_starfield_tests failed\n", failures);
    return 1;
}
