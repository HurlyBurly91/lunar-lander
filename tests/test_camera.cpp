// Focused, rendering-free tests for the lander::Camera presentation logic
// (M02: contextual camera and manual zoom). The camera is pure logic: it is
// driven by a target position, an altitude, and a little input, and it must
// never touch the simulation or the terrain. These checks verify the AUTO
// contextual zoom, the hysteresis, the smooth transitions, the MANUAL wheel
// zoom with clamping, the mode toggle, and the invariants that camera
// operations leave the simulation and terrain untouched.

#include "lander/camera.hpp"
#include "lander/sim.hpp"

#include <algorithm>
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

bool close(double a, double b, double eps) {
    return std::abs(a - b) <= eps;
}

// Advance a camera for `frames` fixed steps at 60 Hz with a constant
// altitude and no wheel input. The followed target defaults to the spawn
// point, a fixed world location (not the moving camera centre).
void settle(lander::Camera& cam, double altitude, int frames,
            double target_x = 0.0, double target_y = 20.0) {
    for (int i = 0; i < frames; ++i) {
        cam.update(1.0 / 60.0, target_x, target_y, altitude, 0, false);
    }
}

// A handful of world-x samples used to prove the terrain is untouched.
std::vector<double> sample_heights(const lander::Terrain& terrain) {
    std::vector<double> out;
    for (double x = -200.0; x <= 200.0; x += 5.0) {
        out.push_back(terrain.height_at(x));
    }
    for (const lander::Pad& pad : terrain.pads()) {
        out.push_back(pad.x_min);
        out.push_back(pad.x_max);
        out.push_back(pad.y);
    }
    return out;
}

bool same_heights(const std::vector<double>& a,
                  const std::vector<double>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// The same world-to-screen transform the GUI uses. The lander is expected to
// sit at a fixed anchor: horizontally centred, and lander_top_fraction down
// from the top of the window.
struct ScreenPos {
    double x{};
    double y{};
};

ScreenPos to_screen(double world_x, double world_y,
                    const lander::Camera& cam) {
    const lander::CameraParams& p = cam.params();
    return {p.window_width / 2.0 + (world_x - cam.x()) * cam.scale(),
            p.window_height / 2.0 - (world_y - cam.y()) * cam.scale()};
}

double anchor_y(const lander::CameraParams& p) {
    return p.lander_top_fraction * p.window_height;
}

double anchor_x(const lander::CameraParams& p) {
    return p.window_width / 2.0;
}

}  // namespace

int main() {
    const lander::CameraParams params;

    // -- AUTO is the initial mode and starts at a neutral zoom. -----------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        check(cam.mode() == lander::CameraMode::kAuto, "initial mode is AUTO");
        check(close(cam.zoom(), 1.0, 1e-9), "initial zoom is 1.0");
        check(close(cam.scale(), params.base_scale, 1e-9),
              "initial scale equals the base scale");
    }

    // -- AUTO selects the landing scale below the lower threshold. --------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 240);  // well below alt_to_landing (18)
        check(cam.auto_wants_landing(), "AUTO picks landing below 18 m");
        check(close(cam.zoom(), params.landing_zoom, 0.01),
              "AUTO converges to the landing scale when low");
    }

    // -- AUTO selects the overview scale above the upper threshold. -------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 60.0, 240);  // well above alt_to_overview (25)
        check(!cam.auto_wants_landing(), "AUTO picks overview above 25 m");
        check(close(cam.zoom(), params.overview_zoom, 0.01),
              "AUTO converges to the overview scale when high");
    }

    // -- Hysteresis: the dead band between the thresholds keeps the state.
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 240);  // well below alt_to_landing (18)
        check(cam.auto_wants_landing(), "start settled in landing");

        settle(cam, 20.0, 5);  // in the dead band [18, 25]
        check(cam.auto_wants_landing(),
              "dead band keeps the landing state (no flicker to overview)");

        settle(cam, 30.0, 2);  // above the upper threshold (25)
        check(!cam.auto_wants_landing(),
              "crossing above 25 m switches to overview");

        settle(cam, 20.0, 5);  // back in the dead band [18, 25]
        check(!cam.auto_wants_landing(),
              "dead band keeps the overview state (no flicker to landing)");

        settle(cam, 10.0, 2);  // below the lower threshold (18)
        check(cam.auto_wants_landing(),
              "crossing below 18 m switches back to landing");
    }

    // -- The zoom eases smoothly toward its target, never a hard snap. ----
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);  // zoom starts at 1.0
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, false);  // target landing
        const double after_one = cam.zoom();
        check(after_one > 1.0 && after_one < params.landing_zoom,
              "after one step the zoom is between the old and target values");

        double previous = after_one;
        bool monotonic = true;
        for (int i = 0; i < 60; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, false);
            if (cam.zoom() < previous) {
                monotonic = false;
            }
            previous = cam.zoom();
        }
        check(monotonic, "the zoom approaches its target monotonically");
        check(close(cam.zoom(), params.landing_zoom, 0.05),
              "the zoom converges to the target over time");
    }

    // -- Toggling to MANUAL preserves the current zoom (no jump). ----------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 240);  // converge to the landing scale
        const double before = cam.zoom();
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // press M
        check(cam.mode() == lander::CameraMode::kManual,
              "pressing M switches to MANUAL");
        check(close(cam.zoom(), before, 1e-9),
              "entering MANUAL keeps the current zoom");
    }

    // -- The mouse wheel changes the zoom only in MANUAL. ------------------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 240);  // converged landing (~1.4)
        const double before = cam.zoom();
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +3, false);  // wheel up in AUTO
        check(close(cam.zoom(), before, 0.01),
              "in AUTO the mouse wheel does not change the zoom");

        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        const double m0 = cam.zoom();
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);  // wheel up
        const double after_up = cam.zoom();
        check(after_up > m0, "in MANUAL the wheel up zooms in");
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, -1, false);  // wheel down
        const double after_down = cam.zoom();
        check(after_down < after_up, "in MANUAL the wheel down zooms out");
    }

    // -- Manual zoom respects the minimum and maximum clamps. --------------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        for (int i = 0; i < 200; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);
        }
        check(close(cam.zoom(), params.zoom_max, 1e-9),
              "manual zoom-in is clamped at the maximum");
        for (int i = 0; i < 400; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, -1, false);
        }
        check(close(cam.zoom(), params.zoom_min, 1e-9),
              "manual zoom-out is clamped at the minimum");
    }

    // -- Toggling back to AUTO resumes automatic zoom behaviour. -----------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        for (int i = 0; i < 30; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);  // zoom in
        }
        const double manual_zoom = cam.zoom();
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> AUTO
        check(cam.mode() == lander::CameraMode::kAuto,
              "pressing M again returns to AUTO");
        settle(cam, 60.0, 240);  // high altitude -> overview target
        check(close(cam.zoom(), params.overview_zoom, 0.01),
              "AUTO resumes and eases back to the contextual scale");
        check(cam.zoom() < manual_zoom,
              "returning to AUTO moved the zoom back toward the auto target");
    }

    // -- Camera operations never modify the simulation or the terrain. -----
    {
        lander::Simulation sim;
        sim.reset(1234);
        const lander::State before_state = sim.state();
        const std::vector<double> before_heights =
            sample_heights(sim.terrain());

        lander::Camera cam(params);
        cam.snap(before_state.x, before_state.y);
        // Drive the camera through a mix of inputs over many frames.
        for (int i = 0; i < 300; ++i) {
            const double alt = 5.0 + 60.0 * std::sin(i * 0.05);
            cam.update(1.0 / 60.0, before_state.x, before_state.y, alt,
                       (i % 7 == 0) ? (i % 2 ? 1 : -1) : 0,
                       i % 37 == 0);
        }

        check(sim.state() == before_state,
              "camera updates leave the simulation state unchanged");
        check(same_heights(sample_heights(sim.terrain()), before_heights),
              "camera updates leave the terrain unchanged");
    }

    // -- Rapid manual zoom keeps the followed target at its screen anchor. --
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        settle(cam, 10.0, 120);  // let the focus converge on the target

        double max_dx = 0.0;
        double max_dy = 0.0;
        for (int i = 0; i < 300; ++i) {
            // Aggressively alternate large wheel steps, including bursts of
            // several notches consumed by a single update.
            const int wheel = (i % 2 == 0) ? (i % 4 == 0 ? 5 : -5)
                                           : (i % 3 == 0 ? 3 : -3);
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, wheel, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dx = std::max(max_dx, std::abs(p.x - anchor_x(params)));
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(max_dx < 1.0,
              "rapid manual zoom keeps the lander horizontally anchored");
        check(max_dy < 1.0,
              "rapid manual zoom keeps the lander at its vertical anchor");

        settle(cam, 10.0, 120);  // no wheel: the anchor must not drift
        const ScreenPos settled = to_screen(0.0, 20.0, cam);
        check(std::abs(settled.x - anchor_x(params)) < 1.0 &&
                  std::abs(settled.y - anchor_y(params)) < 1.0,
              "repeated zoom-in/zoom-out does not accumulate framing error");
    }

    // -- AUTO zoom transitions also keep the lander anchored. ---------------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 60.0, 240);  // settled at the overview scale

        // Drop below the landing threshold and let AUTO ease to the landing
        // scale. The zoom changes continuously, but the followed target must
        // stay at its screen anchor throughout.
        double max_dy = 0.0;
        for (int i = 0; i < 240; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(close(cam.zoom(), params.landing_zoom, 0.01),
              "AUTO transitions to the landing scale when low");
        check(max_dy < 1.0,
              "AUTO zoom transitions do not lose the lander's framing");
    }

    // -- A moving target is still followed (normal smoothing is retained). --
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 120);  // converge on the spawn point

        // The target moves away horizontally; the camera must keep closing
        // the gap rather than being frozen by the zoom-anchor fix. A first-
        // order follow has a small steady-state lag proportional to the
        // target's speed, so the assertion allows for that.
        double target_x = 0.0;
        for (int i = 0; i < 120; ++i) {
            target_x += 0.05;  // 3 m/s over 2 s
            cam.update(1.0 / 60.0, target_x, 20.0, 10.0, 0, false);
        }
        check(cam.x() > 0.0,
              "the camera continues to follow a moving target");
        check(std::abs(cam.x() - target_x) < 2.0,
              "follow smoothing keeps the camera near a moving target");
        const ScreenPos p = to_screen(target_x, 20.0, cam);
        check(std::abs(p.y - anchor_y(params)) < 1.0,
              "following a moving target keeps the vertical anchor");
    }

    // -- A single wheel step changes the scale but not the lander's screen y.
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        settle(cam, 10.0, 120);

        const ScreenPos before = to_screen(0.0, 20.0, cam);
        check(std::abs(before.y - anchor_y(params)) < 1e-6,
              "before a single zoom step the lander is at its vertical anchor");

        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);
        const ScreenPos after_up = to_screen(0.0, 20.0, cam);
        check(std::abs(after_up.y - before.y) < 1e-6,
              "a single wheel-up step leaves the lander's screen y unchanged");
        check(std::abs(after_up.y - anchor_y(params)) < 1e-6,
              "after wheel-up the lander remains at the configured fraction");

        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, -1, false);
        const ScreenPos after_down = to_screen(0.0, 20.0, cam);
        check(std::abs(after_down.y - before.y) < 1e-6,
              "a single wheel-down step leaves the lander's screen y unchanged");
        check(std::abs(after_down.y - anchor_y(params)) < 1e-6,
              "after wheel-down the lander remains at the configured fraction");
    }

    // -- A burst of wheel-up events to the 4.0X clamp keeps the anchor. -----
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        settle(cam, 10.0, 120);

        double max_dy = 0.0;
        for (int i = 0; i < 200; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(close(cam.zoom(), params.zoom_max, 1e-9),
              "repeated wheel-up reaches the 4.0X clamp");
        check(max_dy < 1e-6,
              "the wheel-up burst to 4.0X keeps the vertical anchor exactly");

        for (int i = 0; i < 60; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, +1, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(max_dy < 1e-6,
              "holding at 4.0X does not slowly recover from a displacement");
    }

    // -- A burst of wheel-down events to the 0.2X clamp keeps the anchor. ---
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        settle(cam, 10.0, 120);

        double max_dy = 0.0;
        for (int i = 0; i < 400; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, -1, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(close(cam.zoom(), params.zoom_min, 1e-9),
              "repeated wheel-down reaches the 0.2X clamp");
        check(max_dy < 1e-6,
              "the wheel-down burst to 0.2X keeps the vertical anchor exactly");

        for (int i = 0; i < 60; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, -1, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(max_dy < 1e-6,
              "holding at 0.2X does not slowly recover from a displacement");
    }

    // -- Alternating rapid zoom in/out does not accumulate framing error. ---
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, true);  // -> MANUAL
        settle(cam, 10.0, 120);

        double max_dy = 0.0;
        for (int i = 0; i < 1000; ++i) {
            const int wheel = (i % 2 == 0) ? +5 : -5;
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, wheel, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(max_dy < 1e-6,
              "alternating rapid zoom in/out does not accumulate vertical error");

        settle(cam, 10.0, 120);
        const ScreenPos settled = to_screen(0.0, 20.0, cam);
        check(std::abs(settled.y - anchor_y(params)) < 1e-6,
              "after rapid zooming, the settled vertical anchor is exact");
    }

    // -- AUTO zoom interpolation keeps the vertical anchor in both directions.
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 60.0, 240);  // settled at the overview scale

        double max_dy = 0.0;
        for (int i = 0; i < 240; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 10.0, 0, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(close(cam.zoom(), params.landing_zoom, 0.01),
              "AUTO eases to the landing scale when low");
        check(max_dy < 1e-6,
              "AUTO overview -> landing interpolation keeps the anchor");

        for (int i = 0; i < 240; ++i) {
            cam.update(1.0 / 60.0, 0.0, 20.0, 60.0, 0, false);
            const ScreenPos p = to_screen(0.0, 20.0, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(close(cam.zoom(), params.overview_zoom, 0.01),
              "AUTO eases back to the overview scale when high");
        check(max_dy < 1e-6,
              "AUTO landing -> overview interpolation keeps the anchor");
    }

    // -- A vertically moving target keeps the exact screen-y anchor. --------
    {
        lander::Camera cam(params);
        cam.snap(0.0, 20.0);
        settle(cam, 10.0, 120);

        double target_y = 20.0;
        double max_dy = 0.0;
        for (int i = 0; i < 120; ++i) {
            target_y -= 0.05;  // 3 m/s descent over 2 s
            cam.update(1.0 / 60.0, 0.0, target_y, 10.0, 0, false);
            const ScreenPos p = to_screen(0.0, target_y, cam);
            max_dy = std::max(max_dy, std::abs(p.y - anchor_y(params)));
        }
        check(max_dy < 1e-6,
              "a moving target's screen y stays at the configured anchor");
    }

    if (failures == 0) {
        std::cout << "camera tests passed\n";
        return 0;
    }
    std::cerr << failures << " camera test(s) failed\n";
    return 1;
}
