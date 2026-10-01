#include "lander/camera.hpp"
#include "lander/sim.hpp"

#include <array>
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
    check_close(cam.zoom(), lander::CameraParams{}.zoom_min, 1e-12,
                "manual zoom clamps at the wide minimum");

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
                "manual zoom sweeps down to the wide minimum");

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

void test_auto_zoom_readability() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    const double readable_zoom = lander::camera_readability_zoom(p);
    cam.snap(0.0, lander::kReferenceRadius + 20.0);

    drive_auto(cam, 30.0);
    check(cam.zoom() >= readable_zoom - 1.0e-9,
          "AUTO overview zoom respects the projected-size readability floor");
    check(cam.scale() * lander::kLanderMajorMetres >=
              lander::kMinReadableLanderPx - 1.0e-9,
          "overview projected lander stays readable");
    check(lander::lander_uses_full_model(cam.scale()),
          "overview scale renders the full lander");

    drive_auto(cam, 5.0);
    check(cam.zoom() >= readable_zoom - 1.0e-9,
          "AUTO landing zoom respects the projected-size readability floor");
    check(cam.scale() * lander::kLanderMajorMetres >=
              lander::kMinReadableLanderPx - 1.0e-9,
          "landing projected lander stays readable");
    check(lander::lander_uses_full_model(cam.scale()),
          "landing scale renders the full lander");
    check(cam.scale() <= p.base_scale * p.zoom_max + 1.0e-12,
          "local AUTO zoom remains bounded");
}

void test_ship_representation_threshold() {
    const lander::CameraParams p{};
    const double readable_scale =
        p.base_scale * lander::camera_readability_zoom(p);

    check(lander::lander_uses_full_model(readable_scale),
          "the readability threshold uses the full model");
    check(!lander::lander_uses_full_model(readable_scale * (1.0 - 1.0e-6)),
          "just below the readability threshold uses the marker");
    check(lander::lander_uses_full_model(p.base_scale * p.zoom_max),
          "local maximum zoom uses the full model");
    check(!lander::lander_uses_full_model(p.base_scale * p.zoom_min),
          "local minimum zoom uses the marker");

    // The marker and the full lander use the same attitude transform at the
    // threshold: the marker nose direction equals the full model's local
    // +y nose transformed by the ship attitude and camera angle.
    const std::array<double, 4> ship_angles = {-0.3, 0.0, 0.7, 2.2};
    const std::array<double, 4> camera_angles = {0.0, 0.4, -1.1, 3.0};
    for (double ship_angle : ship_angles) {
        for (double camera_angle : camera_angles) {
            const auto marker =
                lander::marker_triangle(0.0, 0.0, ship_angle, camera_angle,
                                        10.0);
            const lander::Vec2 marker_dir{marker.nose.x, marker.nose.y};
            const double mlen = std::hypot(marker_dir.x, marker_dir.y);

            const double nx = -std::sin(ship_angle);
            const double ny = std::cos(ship_angle);
            const double c = std::cos(camera_angle);
            const double s = std::sin(camera_angle);
            const double local_x = nx * c + ny * s;
            const double local_y = -nx * s + ny * c;
            const lander::Vec2 full_dir{local_x, -local_y};
            const double flen = std::hypot(full_dir.x, full_dir.y);

            check(mlen > 0.0, "marker nose is nonzero");
            check_close(marker_dir.x / mlen, full_dir.x / flen, 1.0e-12,
                        "marker nose matches the full model's attitude x");
            check_close(marker_dir.y / mlen, full_dir.y / flen, 1.0e-12,
                        "marker nose matches the full model's attitude y");
        }
    }

    check_close(lander::cue_arrow_length(0.0, lander::kVelocityCueScale,
                                         lander::kCueMaxLengthPx),
                0.0, 1.0e-12, "zero magnitude gives a zero-length cue");
    check_close(lander::cue_arrow_length(1.0, lander::kVelocityCueScale,
                                         lander::kCueMaxLengthPx),
                lander::kVelocityCueScale, 1.0e-12,
                "small cues use the direct px/unit scale");
    check_close(lander::cue_arrow_length(100.0, lander::kGravityCueScale,
                                         lander::kCueMaxLengthPx),
                lander::kCueMaxLengthPx, 1.0e-12, "large cues clamp to max");
}

void test_system_smooth_zoom_and_no_local_switch() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    const double dt = 1.0 / 60.0;
    cam.snap(0.0, lander::kReferenceRadius + 20.0);
    cam.set_system(true);

    check_close(cam.zoom(), p.system_zoom, 1.0e-12,
                "SYSTEM starts at the default target zoom");
    check_close(cam.system_target_zoom(), p.system_zoom, 1.0e-12,
                "SYSTEM starts at the default target zoom");

    cam.update(dt, 0.0, 0.0, 0.0, 1, false);
    check(cam.mode() == lander::CameraMode::kSystem,
          "wheeling in stays in SYSTEM view");
    check_close(cam.angle(), 0.0, 1.0e-12, "SYSTEM wheel-in keeps angle 0");
    check(cam.system_target_zoom() > cam.zoom() + 1.0e-9,
          "a positive-dt wheel-in changes the target before the rendered zoom");
    check(cam.zoom() > p.system_zoom + 1.0e-9,
          "the rendered SYSTEM zoom eases toward the new target");

    for (int i = 0; i < 600; ++i) {
        cam.update(dt, 0.0, 0.0, 0.0, 1, false);
    }
    check_close(cam.system_target_zoom(), p.system_zoom_max, 1.0e-9,
                "SYSTEM target zoom clamps at the wide-to-close maximum");
    check_close(cam.zoom(), p.system_zoom_max, 1.0e-6,
                "the rendered SYSTEM zoom reaches the maximum smoothly");
    check(cam.mode() == lander::CameraMode::kSystem,
          "a readable SYSTEM zoom does not switch to LOCAL");
    check_close(cam.angle(), 0.0, 1.0e-12,
                "a readable SYSTEM zoom stays inertial");

    for (int i = 0; i < 900; ++i) {
        cam.update(dt, 0.0, 0.0, 0.0, -1, false);
    }
    check_close(cam.system_target_zoom(), p.system_zoom_min, 1.0e-9,
                "SYSTEM target zoom clamps at the wide minimum");
    check_close(cam.zoom(), p.system_zoom_min, 1.0e-6,
                "the rendered SYSTEM zoom reaches the minimum smoothly");
    check(cam.mode() == lander::CameraMode::kSystem,
          "a far SYSTEM zoom does not switch to LOCAL");
}

void test_angle_transition_shortest_path() {
    lander::Camera cam;
    cam.snap(0.0, 0.0, 3.0);
    check_close(cam.angle(), 3.0, 1.0e-12, "snap sets the starting angle");

    cam.update(1.0 / 60.0, 0.0, 0.0, 10.0, 0, false, 3.1);
    check_close(cam.angle(), 3.1, 1.0e-12,
                "a small angular change snaps instead of transitioning");

    cam.snap(0.0, 0.0, 3.0);
    cam.update(1.0 / 60.0, 0.0, 0.0, 10.0, 0, false, -2.5);
    check(cam.angle() > 3.0,
          "a large angular change moves along the shortest arc");
    for (int i = 0; i < 300; ++i) {
        cam.update(1.0 / 60.0, 0.0, 0.0, 10.0, 0, false, -2.5);
        double relative = std::fmod(cam.angle() - 3.0 + lander::kTwoPi,
                                    lander::kTwoPi);
        if (relative < 0.0) {
            relative += lander::kTwoPi;
        }
        check(relative <= 0.7832 + 1.0e-6,
              "the transition stays inside the minimal arc");
    }
    check_close(cam.angle(), -2.5, 1.0e-12,
                "the transition settles exactly on the target angle");
}

void test_angle_transition_preserves_simulation() {
    lander::Simulation sim;
    sim.reset(42);
    const lander::State before = sim.state();
    const int ref_before = sim.reference_body();

    lander::Camera cam;
    const lander::Vec2 ref_pos =
        sim.binary().position(ref_before, sim.sim_time());
    cam.snap(before.x, before.y,
             lander::local_up_angle(before, ref_pos));
    for (int i = 0; i < 300; ++i) {
        cam.update(1.0 / 60.0, before.x, before.y, 10.0, 0, false,
                   before.angle + 1.0);
    }

    check(sim.state() == before, "camera transition does not modify the ship");
    check(sim.reference_body() == ref_before,
          "camera transition does not change reference-body selection");
}

// M05-R3-13: the SYSTEM camera never auto-pans or auto-zooms. The focus is
// always exactly the ship and the zoom is only ever changed by the wheel
// (or the entry default), regardless of where the destination body is. A
// destination that is out of frame is reported by the offscreen indicator
// and the HUD, never by moving the camera.
void test_system_destination_never_moves_camera() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    cam.snap(0.0, lander::kReferenceRadius + 20.0);
    cam.set_system(true);

    // Close destination: the ship stays exactly centred and the zoom stays
    // at the entry default.
    cam.set_system_destination(100.0, 0.0);
    cam.update(0.0, 0.0, 0.0, 0.0, 0, false);
    check_close(cam.zoom(), p.system_zoom, 1.0e-12,
                "a close destination does not change the SYSTEM zoom");
    check_close(cam.center_x(), 0.0, 1.0e-12,
                "a close destination keeps the ship exactly centred x");
    check_close(cam.center_y(), 0.0, 1.0e-12,
                "a close destination keeps the ship exactly centred y");
    const ScreenPos ship = to_screen(0.0, 0.0, cam);
    check_close(ship.x, p.window_width / 2.0, 1.0e-7,
                "the ship sits at the exact viewport centre (x)");
    check_close(ship.y, p.window_height / 2.0, 1.0e-7,
                "the ship sits at the exact viewport centre (y)");

    // Far destination: the same invariants hold.
    cam.set_system_destination(100000.0, 0.0);
    cam.update(0.0, 0.0, 0.0, 0.0, 0, false);
    check_close(cam.zoom(), p.system_zoom, 1.0e-12,
                "a far destination does not change the SYSTEM zoom");
    check_close(cam.center_x(), 0.0, 1.0e-12,
                "a far destination keeps the ship exactly centred x");
    check_close(cam.center_y(), 0.0, 1.0e-12,
                "a far destination keeps the ship exactly centred y");

    // The wheel is the only zoom control, and its result is preserved when
    // the destination changes.
    cam.set_system_destination(100.0, 0.0);
    cam.update(0.0, 0.0, 0.0, 0.0, -1, false);
    const double manual_zoom = cam.zoom();
    check(manual_zoom != p.system_zoom, "wheel input changes the zoom");
    cam.set_system_destination(50000.0, -300.0);
    cam.update(0.0, 0.0, 0.0, 0.0, 0, false);
    check_close(cam.zoom(), manual_zoom, 1.0e-12,
                "changing the destination preserves the wheel zoom");
    check_close(cam.center_x(), 0.0, 1.0e-12,
                "the focus never moves to a ship/destination midpoint");
    check(cam.has_system_destination(),
          "the destination is still tracked for the offscreen indicator");
}

void test_offscreen_indicator() {
    lander::Camera cam;
    const lander::CameraParams& p = cam.params();
    cam.snap(0.0, lander::kReferenceRadius + 20.0);
    cam.set_system(true);
    cam.update(1.0 / 60.0, 0.0, 0.0, 0.0, 0, false);

    const auto on = lander::offscreen_target_indicator(100.0, 0.0, cam);
    check(on.on_screen, "a visible destination is reported on-screen");
    check_close(on.x, p.window_width / 2.0 + 100.0 * cam.scale(), 1.0e-9,
                "on-screen x is true world scale");
    check_close(on.y, p.window_height / 2.0, 1.0e-9,
                "on-screen y is centred");

    const auto right = lander::offscreen_target_indicator(5000.0, 0.0, cam);
    check(!right.on_screen, "a far-right destination is off-screen");
    check_close(right.dir_x, 1.0, 1.0e-12, "right indicator points right");
    check_close(right.dir_y, 0.0, 1.0e-12, "right indicator has no y");
    check_close(right.x, p.window_width, 1.0e-9,
                "right indicator anchors to the right edge");
    check_close(right.y, p.window_height / 2.0, 1.0e-9,
                "right indicator keeps the centre height");

    const auto up = lander::offscreen_target_indicator(0.0, 5000.0, cam);
    check(!up.on_screen, "a far-north destination is off-screen");
    check_close(up.dir_x, 0.0, 1.0e-12, "up indicator has no x");
    check_close(up.dir_y, -1.0, 1.0e-12, "up indicator points screen-up");
    check_close(up.x, p.window_width / 2.0, 1.0e-9,
                "up indicator keeps the centre width");
    check_close(up.y, 0.0, 1.0e-9, "up indicator anchors to the top edge");
}

void test_marker_triangle_orientation() {
    const auto up = lander::marker_triangle(100.0, 100.0, 0.0, 0.0, 10.0);
    check_close(up.nose.x, 100.0, 1.0e-12, "ship up points screen-up");
    check_close(up.nose.y, 90.0, 1.0e-12, "ship-up nose is above the centre");
    check(up.left.y > up.nose.y && up.right.y > up.nose.y,
          "up-facing marker back vertices are behind the nose");
    check(up.left.x > 100.0 && up.right.x < 100.0,
          "up-facing marker back vertices spread sideways");

    const auto right =
        lander::marker_triangle(100.0, 100.0, 0.0, 0.5 * lander::kPi, 10.0);
    check_close(right.nose.x, 110.0, 1.0e-12,
                "camera rotation carries the nose to screen-right");
    check_close(right.nose.y, 100.0, 1.0e-12,
                "camera-right nose has no vertical offset");

    const auto left =
        lander::marker_triangle(100.0, 100.0, 0.5 * lander::kPi, 0.0, 10.0);
    check_close(left.nose.x, 90.0, 1.0e-12,
                "ship-left attitude points screen-left");
    check_close(left.nose.y, 100.0, 1.0e-12,
                "screen-left nose has no vertical offset");

    const auto down =
        lander::marker_triangle(100.0, 100.0, lander::kPi, 0.0, 10.0);
    check_close(down.nose.x, 100.0, 1.0e-12,
                "ship-down attitude has no horizontal offset");
    check_close(down.nose.y, 110.0, 1.0e-12,
                "ship-down nose points screen-down");
}

// M05-R3-V10: the crash dialog is a compact, content-sized, screen-space
// modal whose geometry is independent of body, terrain, camera mode, and
// SYSTEM zoom.
void test_crash_modal_geometry() {
    // The existing crash dialog uses these three content lines. The character
    // counts are the same for a PRIMARY crash and a COMPANION crash, so the
    // modal geometry must be identical.
    const int title_chars = 7;    // "CRASHED"
    const int title_scale = 6;
    const int score_chars = 12;   // "SCORE 123456"
    const int score_scale = 3;
    const int hint_chars = 26;    // "R RETRY   N x3 NEW SEED"
    const int hint_scale = 2;

    const auto primary = lander::crash_modal_rect(
        title_chars, title_scale, score_chars, score_scale, hint_chars,
        hint_scale, 1280, 720);
    const auto companion = lander::crash_modal_rect(
        title_chars, title_scale, score_chars, score_scale, hint_chars,
        hint_scale, 1280, 720);
    check(primary.x == companion.x && primary.y == companion.y &&
              primary.w == companion.w && primary.h == companion.h,
          "primary and companion crash modals have identical geometry");

    check(primary.w > 0 && primary.h > 0, "the crash modal is bounded");
    check(primary.w <= 1280 - 48 && primary.h <= 720 - 48,
          "the crash modal stays inside the viewport margins");
    check(primary.x >= 24 && primary.y >= 24,
          "the crash modal respects the viewport margin");
    check(primary.x + primary.w <= 1280 - 24 &&
              primary.y + primary.h <= 720 - 24,
          "the crash modal does not extend past the viewport");
    check(primary.h < 240,
          "the crash modal is compact rather than a tall column");

    const auto wider = lander::crash_modal_rect(
        title_chars, title_scale, score_chars + 8, score_scale, hint_chars,
        hint_scale, 1280, 720);
    check(wider.w > primary.w,
          "the crash modal width grows with longer score content");
    check(wider.h == primary.h,
          "the crash modal height is content-sized by line count/scale");

    const auto small_viewport = lander::crash_modal_rect(
        title_chars, title_scale, score_chars, score_scale, hint_chars,
        hint_scale, 320, 240);
    check(small_viewport.w <= 320 - 48 && small_viewport.h <= 240 - 48,
          "the crash modal clamps to a small viewport");
}

// M05-R4-01: M from a local view toggles AUTO/MANUAL; M from SYSTEM enters
// MANUAL directly, restores the saved local zoom, and eases the angle to the
// target reference-body direction without changing simulation state.
void test_m_local_toggles_auto_manual() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius + 20.0;
    cam.snap(0.0, R);
    check(cam.mode() == lander::CameraMode::kAuto,
          "M local test starts in AUTO");

    cam.update(0.0, 0.0, R, 20.0, 0, true);
    check(cam.mode() == lander::CameraMode::kManual,
          "M from LOCAL AUTO enters MANUAL");

    cam.update(0.0, 0.0, R, 20.0, 0, true);
    check(cam.mode() == lander::CameraMode::kAuto,
          "M from LOCAL MANUAL returns to AUTO");
}

void test_m_from_system_enters_manual() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius + 20.0;
    cam.snap(0.0, R);
    check(cam.mode() == lander::CameraMode::kAuto,
          "M-from-system test starts in AUTO");

    cam.set_system(true);
    cam.update(1.0 / 60.0, 0.0, R, 20.0, 0, false);
    check(cam.system_view(), "premise: camera is in SYSTEM view");

    cam.enter_manual();
    check(!cam.system_view(), "M from SYSTEM leaves SYSTEM view");
    check(cam.mode() == lander::CameraMode::kManual,
          "M from SYSTEM enters LOCAL MANUAL, not the saved AUTO mode");
    check_close(cam.zoom(), 1.0, 1e-12,
                "M from SYSTEM restores the saved local zoom");

    // The SYSTEM camera holds angle 0. Leaving into manual should not snap
    // to an arbitrary local angle; the next update eases toward the
    // reference-body radial-down direction supplied by the caller.
    const double tx = R;
    const double ty = 0.0;
    const double target_angle = -0.5 * lander::kPi;
    const double angle_before = cam.angle();
    check_close(angle_before, 0.0, 1e-12,
                "premise: SYSTEM left the camera angle at zero");
    cam.update(1.0 / 60.0, tx, ty, 20.0, 0, false, target_angle);
    check(!cam.system_view(), "the next update remains in the local view");
    check(cam.mode() == lander::CameraMode::kManual,
          "the next update remains MANUAL");
    check(cam.angle() < angle_before && cam.angle() > target_angle,
          "the angle starts easing toward the target without snapping");

    for (int i = 0; i < 240; ++i) {
        cam.update(1.0 / 60.0, tx, ty, 20.0, 0, false, target_angle);
    }
    check_close(cam.angle(), target_angle, 1e-6,
                "the manual camera converges to the radial-down angle");
    check_anchor(cam, tx, ty);
}

void test_m_from_system_keeps_v_restore_semantics() {
    lander::Camera cam;
    const double R = lander::kReferenceRadius + 20.0;
    cam.snap(0.0, R);
    cam.update(0.0, 0.0, R, 20.0, 0, true);  // MANUAL
    check(cam.mode() == lander::CameraMode::kManual,
          "premise: local mode before SYSTEM is MANUAL");

    cam.set_system(true);
    cam.update(1.0 / 60.0, 0.0, R, 20.0, 0, false);
    cam.set_system(false);
    check(cam.mode() == lander::CameraMode::kManual,
          "V restore still returns to the saved MANUAL mode");

    // If the player chooses MANUAL from SYSTEM instead, that explicit choice
    // becomes the saved local mode for the next V round trip.
    cam.set_system(true);
    cam.update(1.0 / 60.0, 0.0, R, 20.0, 0, false);
    cam.enter_manual();
    check(cam.mode() == lander::CameraMode::kManual,
          "M from SYSTEM chooses MANUAL");
    cam.set_system(true);
    cam.update(1.0 / 60.0, 0.0, R, 20.0, 0, false);
    cam.set_system(false);
    check(cam.mode() == lander::CameraMode::kManual,
          "a later V restore preserves the explicitly chosen MANUAL mode");
}

void test_m_from_system_does_not_modify_simulation() {
    lander::Simulation sim;
    sim.reset(7);
    const lander::State before = sim.state();
    const std::vector<double> before_terrain = sample_terrain(sim.terrain());

    lander::Camera cam;
    cam.snap(before.x, before.y);
    cam.set_system(true);
    cam.update(1.0 / 60.0, before.x, before.y, 20.0, 0, false);
    cam.enter_manual();
    for (int i = 0; i < 120; ++i) {
        cam.update(1.0 / 60.0, before.x, before.y, 20.0, 0, false);
    }

    check(sim.state() == before, "M from SYSTEM does not modify simulation");
    check(sample_terrain(sim.terrain()) == before_terrain,
          "M from SYSTEM does not modify terrain");
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
    test_auto_zoom_readability();
    test_ship_representation_threshold();
    test_angle_transition_shortest_path();
    test_angle_transition_preserves_simulation();
    test_system_destination_never_moves_camera();
    test_system_smooth_zoom_and_no_local_switch();
    test_offscreen_indicator();
    test_marker_triangle_orientation();
    test_m_local_toggles_auto_manual();
    test_m_from_system_enters_manual();
    test_m_from_system_keeps_v_restore_semantics();
    test_m_from_system_does_not_modify_simulation();
    test_crash_modal_geometry();

    if (failures == 0) {
        std::puts("All lander_camera_tests passed");
        return 0;
    }
    std::printf("%d lander_camera_tests failed\n", failures);
    return 1;
}
