// Headless tests for the deterministic, camera-independent starfield (M03).
// The stars must be fixed in screen/celestial background space: camera
// translation and zoom must not move them, and the layout must be stable for
// a given seed.

#include "lander/starfield.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool close(double a, double b, double eps = 1e-12) {
    return std::abs(a - b) <= eps;
}

}  // namespace

int main() {
    const double width = 1280.0;
    const double height = 720.0;

    // Generation is deterministic for a given seed and viewport.
    {
        const std::vector<lander::Star> a =
            lander::make_stars(1234, width, height);
        const std::vector<lander::Star> b =
            lander::make_stars(1234, width, height);

        check(!a.empty(), "starfield should contain stars");
        check(a.size() == b.size(),
              "the same seed must generate the same number of stars");
        bool same = a.size() == b.size();
        for (size_t i = 0; same && i < a.size(); ++i) {
            same = close(a[i].x, b[i].x) && close(a[i].y, b[i].y) &&
                   a[i].size == b[i].size && a[i].bright == b[i].bright;
        }
        check(same, "the same seed must generate identical stars");
    }

    // Different seeds should produce a different layout.
    {
        const std::vector<lander::Star> a =
            lander::make_stars(1234, width, height);
        const std::vector<lander::Star> b =
            lander::make_stars(4321, width, height);

        bool different = a.size() != b.size();
        for (size_t i = 0; !different && i < a.size(); ++i) {
            different = !close(a[i].x, b[i].x) || !close(a[i].y, b[i].y);
        }
        check(different, "different seeds should generate different stars");
    }

    // Stars are generated inside the viewport.
    {
        const std::vector<lander::Star> stars =
            lander::make_stars(7, width, height);
        bool in_bounds = true;
        for (const lander::Star& s : stars) {
            in_bounds = in_bounds && s.x >= 0.0 && s.x < width && s.y >= 0.0 &&
                        s.y < height && s.size > 0;
        }
        check(in_bounds, "stars must be generated inside the viewport");
    }

    // A star's screen position is independent of camera x/y and scale.
    {
        const std::vector<lander::Star> stars =
            lander::make_stars(99, width, height);
        check(!stars.empty(), "need at least one star for the camera test");

        for (const lander::Star& star : stars) {
            const lander::ScreenPoint base =
                lander::star_screen_pos(star, 0.0, 0.0, 1.0);
            const lander::ScreenPoint translated =
                lander::star_screen_pos(star, 350.0, -120.0, 1.0);
            const lander::ScreenPoint zoomed_in =
                lander::star_screen_pos(star, 0.0, 0.0, 4.0);
            const lander::ScreenPoint zoomed_out =
                lander::star_screen_pos(star, -80.0, 60.0, 0.2);

            check(close(base.x, translated.x) && close(base.y, translated.y),
                  "camera translation must not move a star");
            check(close(base.x, zoomed_in.x) && close(base.y, zoomed_in.y),
                  "zooming in must not move a star");
            check(close(base.x, zoomed_out.x) && close(base.y, zoomed_out.y),
                  "zooming out must not move a star");
        }
    }

    if (failures == 0) {
        std::cout << "starfield tests passed\n";
        return 0;
    }
    std::cerr << failures << " starfield test(s) failed\n";
    return 1;
}
