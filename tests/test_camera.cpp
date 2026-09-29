#include "lander/camera.hpp"
#include "lander/sim.hpp"

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

struct ScreenPos {
    double x{};
    double y{};
};

ScreenPos to_screen(double world_x, double world_y,
                    const lander::Camera& cam) {
    const lander::CameraParams& p = cam.params();
    const double dx = world_x - cam.center_x();
    const double dy = world_y - cam.center_y();
    const double c = std::cos(cam.angle());
    const double s = std::sin(cam.angle());
    const double local_x = dx * c + dy * s;
    const double local_y = -dx * s + dy * c;
    return {p.window_width / 2.0 + local_x * cam.scale(),
            p.window_height / 2.0 - local_y * cam.scale()};
}

std::vector<double> sample_terrain(const lander::Terrain& terrain) {
    std::vector<double> out;
    const double C = terrain.circumference();
    for (int i = 0; i < 256; ++i) {
        out.push_back(terrain.surface_radius_at_arc(i * C / 256.0));
    }
    for (const lander::Pad& pad : terrain.pads()) {
        out.push_back(pad.center_arc);
        out.push_back(pad.half_width);
        out.push_back(pad.radius);
        out.push_back(static_cast<double>(pad.multiplier));
    }
    return out;
}

void drive_auto(lander::Camera& cam, double altitude, int frames = 360) {
    for (int i = 0; i < frames; ++i) {
        cam.update(1.0 / 60.0, 0.0, lander::kReferenceRadius + 20.0, altitude,
                   0, false);
    }
}

void check_anchor(lander::Camera& cam, double target_x, double target_y) {
    const ScreenPos p = to_screen(target_x, target_y, cam);
    check_close(p.x, cam.params().window_width / 2.0, 1e-7,
                "target should stay horizontally centred at convergence");
    check_close(p.y, cam.params().window_height * 0.30, 1e-7,
                "target should stay at the vertical framing anchor");
}

void test_initial_state() {
    lander::Camera cam;
    check(cam.mode() == lander::CameraMode::kAuto, "default mode is AUTO");
    check_close(cam.zoom(), 1.0, 1e-12, "default zoom is 1");
    check_close(cam.scale(), 14.0, 1e-12, "default scale is base scale");
    check_close(cam.angle(), 0.0, 1e-12, "default angle is north");
}

void test_snap_and_rotation() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius;

    cam.snap(0.0, R + 20.0);
    check(cam.mode() == lander::CameraMode::kAuto, "snap restarts AUTO");
    check_close(cam.zoom(), 1.0, 1e-12, "snap uses neutral zoom");
    check_close(cam.angle(), 0.0, 1e-12, "north pole camera angle is 0");
    const double offset =
        (0.5 - 0.3) * 720.0 / cam.scale();
    check_close(cam.center_x(), 0.0, 1e-12, "north snap x centre");
    check_close(cam.center_y(), R + 20.0 - offset, 1e-12,
                "north snap y centre");
    check_anchor(cam, 0.0, R + 20.0);

    cam.snap(R + 20.0, 0.0);
    check_close(cam.angle(), -0.5 * lander::kPi, 1e-12,
                "east camera angle points local up right");
    check_anchor(cam, R + 20.0, 0.0);

    const ScreenPos target = to_screen(R + 20.0, 0.0, cam);
    const ScreenPos up = to_screen(R + 21.0, 0.0, cam);
    check_close(up.x, target.x, 1e-9, "local up should move screen up");
    check_close(up.y, target.y - cam.scale(), 1e-9,
                "local up should move screen up by one metre");

    const ScreenPos right = to_screen(R + 20.0, -1.0, cam);
    check_close(right.x, target.x + cam.scale(), 1e-9,
                "local right should move screen right");
    check_close(right.y, target.y, 1e-9,
                "local right should not move screen vertically");

    cam.snap(0.0, -(R + 20.0));
    check_close(std::fabs(cam.angle()), lander::kPi, 1e-12,
                "south camera angle is +/- pi");
    check_anchor(cam, 0.0, -(R + 20.0));
}

void test_auto_hysteresis_and_zoom() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius;
    cam.snap(0.0, R + 20.0);

    drive_auto(cam, 30.0);
    check(!cam.auto_wants_landing(), "high altitude selects overview");
    check_close(cam.zoom(), lander::CameraParams{}.overview_zoom, 1e-3,
                "high altitude eases to overview zoom");

    drive_auto(cam, 22.0);
    check(!cam.auto_wants_landing(), "dead band holds overview");

    drive_auto(cam, 10.0);
    check(cam.auto_wants_landing(), "low altitude selects landing");
    check_close(cam.zoom(), lander::CameraParams{}.landing_zoom, 1e-3,
                "low altitude eases to landing zoom");

    drive_auto(cam, 22.0);
    check(cam.auto_wants_landing(), "dead band holds landing");

    drive_auto(cam, 30.0);
    check(!cam.auto_wants_landing(), "above overview threshold releases");
    check_anchor(cam, 0.0, R + 20.0);
}

void test_manual_mode_and_wheel() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius;
    cam.snap(0.0, R + 20.0);

    cam.update(0.0, 0.0, R + 20.0, 0.0, 5, false);
    check_close(cam.zoom(), 1.0, 1e-12, "AUTO ignores wheel");

    cam.update(0.0, 0.0, R + 20.0, 0.0, 0, true);
    check(cam.mode() == lander::CameraMode::kManual, "M toggles MANUAL");

    cam.update(0.0, 0.0, R + 20.0, 0.0, 1, false);
    check_close(cam.zoom(), 1.12, 1e-12, "wheel in multiplies zoom in");

    cam.update(0.0, 0.0, R + 20.0, 0.0, -1, false);
    check_close(cam.zoom(), 1.12 * (1.0 - lander::CameraParams{}.wheel_step),
                1e-12, "wheel out multiplies zoom out");

    cam.snap(0.0, R + 20.0);
    cam.update(0.0, 0.0, R + 20.0, 0.0, 0, true);
    cam.update(0.0, 0.0, R + 20.0, 0.0, -1, false);
    check_close(cam.zoom(), 1.0 - lander::CameraParams{}.wheel_step, 1e-12,
                "wheel out from neutral zooms out");

    for (int i = 0; i < 100; ++i) {
        cam.update(0.0, 0.0, R + 20.0, 0.0, 1, false);
    }
    check_close(cam.zoom(), 4.0, 1e-12, "manual zoom clamps at max");

    for (int i = 0; i < 100; ++i) {
        cam.update(0.0, 0.0, R + 20.0, 0.0, -1, false);
    }
    check_close(cam.zoom(), 0.2, 1e-12, "manual zoom clamps at min");

    cam.update(0.0, 0.0, R + 20.0, 0.0, 0, true);
    check(cam.mode() == lander::CameraMode::kAuto, "M toggles back AUTO");
}

void test_follow_and_anchor() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius;
    cam.snap(0.0, R + 20.0);

    double theta = 0.5 * lander::kPi;
    double x = 0.0;
    double y = R + 10.0;
    for (int i = 0; i < 120; ++i) {
        theta += 0.0001;
        x = std::cos(theta) * (R + 10.0);
        y = std::sin(theta) * (R + 10.0);
        cam.update(1.0 / 60.0, x, y, 10.0, 0, false);
    }

    const double expected_angle = theta - 0.5 * lander::kPi;
    double diff = cam.angle() - expected_angle;
    diff = std::fmod(diff + lander::kPi, 2.0 * lander::kPi);
    if (diff < 0.0) {
        diff += 2.0 * lander::kPi;
    }
    diff -= lander::kPi;
    check_close(diff, 0.0, 0.05, "camera angle should follow target");

    const ScreenPos p = to_screen(x, y, cam);
    check_close(p.y, cam.params().window_height * 0.30, 1e-7,
                "moving target keeps vertical anchor");
    check(std::abs(p.x - cam.params().window_width / 2.0) < 20.0,
          "moving target stays near horizontal centre while following");
}

void test_full_revolution_anchor_and_zoom() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    const double R = lander::kReferenceRadius + 10.0;
    const int steps = 1200;

    cam.snap(0.0, R);
    for (int i = 1; i <= steps; ++i) {
        const double theta = 0.5 * lander::kPi + i * lander::kTwoPi / steps;
        const double x = std::cos(theta) * R;
        const double y = std::sin(theta) * R;
        cam.update(1.0 / 60.0, x, y, 10.0, 0, false);
        const ScreenPos pos = to_screen(x, y, cam);
        check_close(pos.x, p.window_width / 2.0, 1e-6,
                    "full revolution keeps exact horizontal anchor");
        check_close(pos.y, p.window_height * p.lander_top_fraction, 1e-6,
                    "full revolution keeps exact vertical anchor");
    }

    cam.snap(0.0, R);
    cam.update(0.0, 0.0, R, 10.0, 0, true);
    for (int i = 0; i < 200 && cam.zoom() < p.zoom_max - 1e-9; ++i) {
        cam.update(0.0, 0.0, R, 10.0, 1, false);
        const ScreenPos pos = to_screen(0.0, R, cam);
        check_close(pos.x, p.window_width / 2.0, 1e-6,
                    "manual zoom-in sweep preserves horizontal anchor");
        check_close(pos.y, p.window_height * p.lander_top_fraction, 1e-6,
                    "manual zoom-in sweep preserves vertical anchor");
    }
    check_close(cam.zoom(), p.zoom_max, 1e-9,
                "manual zoom sweeps up to the 4.0X maximum");
    for (int i = 0; i < 400 && cam.zoom() > p.zoom_min + 1e-9; ++i) {
        cam.update(0.0, 0.0, R, 10.0, -1, false);
        const ScreenPos pos = to_screen(0.0, R, cam);
        check_close(pos.x, p.window_width / 2.0, 1e-6,
                    "manual zoom-out sweep preserves horizontal anchor");
        check_close(pos.y, p.window_height * p.lander_top_fraction, 1e-6,
                    "manual zoom-out sweep preserves vertical anchor");
    }
    check_close(cam.zoom(), p.zoom_min, 1e-9,
                "manual zoom sweeps down to the 0.2X minimum");

    const double auto_y = lander::kReferenceRadius + 20.0;
    cam.snap(0.0, auto_y);
    drive_auto(cam, 30.0);
    check_anchor(cam, 0.0, auto_y);
    drive_auto(cam, 5.0);
    check_anchor(cam, 0.0, auto_y);
}

void test_zero_dt_angle_change() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius;
    cam.snap(0.0, R + 20.0);
    cam.update(0.0, R + 20.0, 0.0, 10.0, 0, false);
    check_close(cam.angle(), -0.5 * lander::kPi, 1e-12,
                "zero-dt update can change camera angle");
}

void test_camera_does_not_modify_simulation_or_terrain() {
    lander::Simulation sim;
    sim.reset(42);
    const lander::State before_state = sim.state();
    const std::vector<double> before_terrain =
        sample_terrain(sim.terrain());

    lander::Camera cam;
    cam.snap(before_state.x, before_state.y);
    for (int i = 0; i < 300; ++i) {
        const double alt = 5.0 + 20.0 * (i % 7);
        cam.update(1.0 / 60.0, before_state.x, before_state.y, alt,
                   i % 3 == 0 ? 1 : -1, i % 17 == 0);
    }

    check(sim.state() == before_state, "camera must not modify simulation");
    check(sample_terrain(sim.terrain()) == before_terrain,
          "camera must not modify terrain");
}

// M05-R1: the inertial SYSTEM view frames the whole binary with the ship at
// the exact viewport centre, holds angle 0, and clamps wheel zoom to the
// (narrow) system range.
void test_system_mode_basic() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    cam.snap(0.0, lander::kReferenceRadius + 20.0);  // start in AUTO

    cam.set_system(true);
    check(cam.system_view(), "set_system(true) enters SYSTEM view");
    check(cam.mode() == lander::CameraMode::kSystem, "mode is SYSTEM");
    check_close(cam.zoom(), p.system_zoom, 1e-12, "SYSTEM starts at system_zoom");
    check_close(cam.angle(), 0.0, 1e-12, "SYSTEM view is unrotated (angle 0)");

    // The ship is the target; it sits at the exact viewport centre.
    const double tx = 123.0, ty = -77.0;
    cam.update(1.0 / 60.0, tx, ty, 50.0, 0, false);
    check_close(cam.angle(), 0.0, 1e-12, "angle stays 0 in SYSTEM update");
    check_close(cam.center_x(), tx, 1e-12, "SYSTEM centres on the ship x");
    check_close(cam.center_y(), ty, 1e-12, "SYSTEM centres on the ship y");
    const ScreenPos ship = to_screen(tx, ty, cam);
    check_close(ship.x, p.window_width / 2.0, 1e-7,
                "ship sits at exact horizontal centre");
    check_close(ship.y, p.window_height / 2.0, 1e-7,
                "ship sits at exact vertical centre");

    // A point 100 m to the right of the ship maps 100 * scale px to the right.
    const ScreenPos right = to_screen(tx + 100.0, ty, cam);
    check_close(right.x, p.window_width / 2.0 + 100.0 * cam.scale(), 1e-7,
                "world x offset maps to the right");
    check_close(right.y, p.window_height / 2.0, 1e-7,
                "world x offset does not move vertically");
}

void test_system_mode_zoom_clamp() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    cam.snap(0.0, lander::kReferenceRadius + 20.0);
    cam.set_system(true);

    for (int i = 0; i < 200; ++i) {
        cam.update(0.0, 0.0, 0.0, 0.0, 1, false);
    }
    check_close(cam.zoom(), p.system_zoom_max, 1e-12,
                "SYSTEM wheel-in clamps at system_zoom_max");
    check_close(cam.angle(), 0.0, 1e-12, "zoom-in leaves angle 0");

    for (int i = 0; i < 400; ++i) {
        cam.update(0.0, 0.0, 0.0, 0.0, -1, false);
    }
    check_close(cam.zoom(), p.system_zoom_min, 1e-12,
                "SYSTEM wheel-out clamps at system_zoom_min");
    check_close(cam.angle(), 0.0, 1e-12, "zoom-out leaves angle 0");
}

// Entering the SYSTEM view must remember the local mode / zoom / want-landing
// state and restore it exactly when the system view is left.
void test_system_mode_saves_restores_local_state() {
    lander::Camera cam;
    cam.snap(0.0, lander::kReferenceRadius + 20.0);  // AUTO, zoom 1, wants landing
    drive_auto(cam, 30.0);  // release to overview: wants_landing false, zoom ~0.4
    const auto mode_before = cam.mode();
    const double zoom_before = cam.zoom();
    const bool landing_before = cam.auto_wants_landing();
    check(!landing_before, "premise: overview (no landing) before entering");

    cam.set_system(true);
    // Mess around in SYSTEM view: zoom and centering change, but the saved
    // local state must be untouched.
    for (int i = 0; i < 5; ++i) {
        cam.update(1.0 / 60.0, 50.0, -30.0, 999.0, 1, false);
    }

    cam.set_system(false);
    check(!cam.system_view(), "set_system(false) leaves SYSTEM view");
    check(cam.mode() == mode_before, "local mode restored");
    check_close(cam.zoom(), zoom_before, 1e-12, "local zoom restored exactly");
    check(cam.auto_wants_landing() == landing_before,
          "want-landing restored exactly");
}

// Re-entering the SYSTEM view must not re-save (so leaving restores the
// original local state), and snap() in SYSTEM view must keep the inertial
// system view rather than switching back to AUTO.
void test_system_mode_no_resave_and_snap_keeps_system() {
    lander::Camera cam;
    cam.snap(0.0, lander::kReferenceRadius + 20.0);  // AUTO, zoom 1
    cam.set_system(true);
    cam.set_system(true);  // re-entering is a no-op: must not re-save zoom
    cam.set_system(false);
    check_close(cam.zoom(), 1.0, 1e-12,
                "re-entering SYSTEM does not re-save the local zoom");

    cam.set_system(true);
    cam.snap(45.0, -12.0);
    check(cam.system_view(), "snap() keeps SYSTEM view");
    check_close(cam.angle(), 0.0, 1e-12, "snap() in SYSTEM leaves angle 0");
    check_close(cam.center_x(), 45.0, 1e-12, "snap() in SYSTEM recentres x");
    check_close(cam.center_y(), -12.0, 1e-12, "snap() in SYSTEM recentres y");
}

}  // namespace

int main() {
    test_initial_state();
    test_snap_and_rotation();
    test_auto_hysteresis_and_zoom();
    test_manual_mode_and_wheel();
    test_follow_and_anchor();
    test_full_revolution_anchor_and_zoom();
    test_zero_dt_angle_change();
    test_camera_does_not_modify_simulation_or_terrain();
    test_system_mode_basic();
    test_system_mode_zoom_clamp();
    test_system_mode_saves_restores_local_state();
    test_system_mode_no_resave_and_snap_keeps_system();

    if (failures == 0) {
        std::puts("All lander_camera_tests passed");
        return 0;
    }
    std::printf("%d lander_camera_tests failed\n", failures);
    return 1;
}
