#include "lander/starfield.hpp"

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

void check_close(double a, double b, double eps, const char* message) {
    if (std::abs(a - b) > eps) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

double norm(double x, double y) {
    return std::hypot(x, y);
}

// Rotate a point about the viewport centre by `delta` (screen-space,
// y-down). This is the same rotation the rendered world undergoes: the
// gui world transform maps world offset (dx, dy) to screen offset
// (c*dx + s*dy, dx*s - dy*c), which is a pure rotation by the camera
// angle about the centre.
void rotate_about_centre(double w, double h, double delta, double* x,
                         double* y) {
    const double dx = *x - 0.5 * w;
    const double dy = *y - 0.5 * h;
    const double c = std::cos(delta);
    const double s = std::sin(delta);
    *x = 0.5 * w + (c * dx - s * dy);
    *y = 0.5 * h + (s * dx + c * dy);
}

void test_generation() {
    const double width = 1280.0;
    const double height = 720.0;
    const double cx = 0.5 * width;
    const double cy = 0.5 * height;
    const double radius = 0.5 * std::hypot(width, height);

    const auto a = lander::make_stars(1234, width, height);
    const auto b = lander::make_stars(1234, width, height);
    const auto c = lander::make_stars(4321, width, height);

    check(a.size() == 280, "starfield should have 280 stars");
    check(a == b, "same seed should generate the same starfield");
    check(a != c, "different seeds should generate different starfields");

    // The base positions fill a disk of radius 0.5 * hypot(w, h) centred on
    // the viewport, so the backdrop covers the whole viewport at any
    // presentation rotation.
    for (const lander::Star& star : a) {
        check(norm(star.x - cx, star.y - cy) <= radius + 1.0e-9,
              "star lies inside the coverage disk");
        check(star.size == 1 || star.size == 2, "star size is 1 or 2");
        check(star.bright >= 90 && star.bright <= 229,
              "star brightness is in range");
    }
    // Every viewport corner is inside the coverage disk, so a rotation can
    // never expose an empty corner.
    for (const double px : {0.0, width}) {
        for (const double py : {0.0, height}) {
            check(norm(px - cx, py - cy) <= radius + 1.0e-9,
                  "viewport corner covered by the disk");
        }
    }
}

void test_angle_zero_canonical() {
    const double width = 1280.0;
    const double height = 720.0;
    const auto stars = lander::make_stars(1234, width, height);
    for (const lander::Star& star : stars) {
        const lander::ScreenPoint p =
            lander::star_screen_pos(star, width, height, 0.0);
        check_close(p.x, star.x, 1.0e-12, "angle 0 x is the generated x");
        check_close(p.y, star.y, 1.0e-12, "angle 0 y is the generated y");
    }
}

void test_full_revolution_identity() {
    const double width = 1280.0;
    const double height = 720.0;
    const auto stars = lander::make_stars(1234, width, height);
    for (const lander::Star& star : stars) {
        const lander::ScreenPoint p =
            lander::star_screen_pos(star, width, height, 2.0 * M_PI);
        check_close(p.x, star.x, 1.0e-9, "2*pi rotation returns x");
        check_close(p.y, star.y, 1.0e-9, "2*pi rotation returns y");
    }
}

void test_rotation_matches_scene() {
    // M05-R3-12: changing the camera angle by d must rotate the starfield
    // by exactly d about the viewport centre, the same rigid rotation the
    // rendered world undergoes. Radius from the centre is invariant.
    const double width = 1280.0;
    const double height = 720.0;
    const double cx = 0.5 * width;
    const double cy = 0.5 * height;
    const auto stars = lander::make_stars(1234, width, height);

    const double angles[] = {0.0, 0.1, -0.4, 1.0, 2.2, -2.9, M_PI};
    const double deltas[] = {0.05, -0.3, 0.7, 1.5};
    for (const lander::Star& star : stars) {
        for (double a : angles) {
            const lander::ScreenPoint pa =
                lander::star_screen_pos(star, width, height, a);
            check_close(norm(pa.x - cx, pa.y - cy),
                        norm(star.x - cx, star.y - cy), 1.0e-6,
                        "rotation preserves the star's radius");
            for (double d : deltas) {
                const lander::ScreenPoint pd =
                    lander::star_screen_pos(star, width, height, a + d);
                double ex = pa.x;
                double ey = pa.y;
                rotate_about_centre(width, height, d, &ex, &ey);
                check_close(pd.x, ex, 1.0e-6,
                            "angle change rotates the pattern like the "
                            "scene (x)");
                check_close(pd.y, ey, 1.0e-6,
                            "angle change rotates the pattern like the "
                            "scene (y)");
            }
        }
    }
}

void test_inertial_not_world_attached() {
    // M05-R3-12: the star position depends only on the star's base
    // position, the viewport, and the camera angle. There is no camera
    // centre, reference body, phase, pan, or zoom term in the transform,
    // so the backdrop cannot parallax-translate with the world. This is
    // asserted by the API shape (no such arguments exist) and by the
    // invariance checks below: the same (star, viewport, angle) triple
    // always yields the same point, independently of any world state.
    const double width = 1280.0;
    const double height = 720.0;
    const auto stars = lander::make_stars(77, width, height);
    const lander::Star& star = stars[0];
    const double angles[] = {-3.0, -1.0, 0.0, 1.0, 3.0, 6.28};
    for (double a : angles) {
        const lander::ScreenPoint p1 =
            lander::star_screen_pos(star, width, height, a);
        const lander::ScreenPoint p2 =
            lander::star_screen_pos(star, width, height, a);
        check(p1 == p2, "same (star, viewport, angle) is deterministic");
    }
}

}  // namespace

int main() {
    test_generation();
    test_angle_zero_canonical();
    test_full_revolution_identity();
    test_rotation_matches_scene();
    test_inertial_not_world_attached();

    if (failures == 0) {
        std::puts("All lander_starfield_tests passed");
        return 0;
    }
    std::printf("%d lander_starfield_tests failed\n", failures);
    return 1;
}
