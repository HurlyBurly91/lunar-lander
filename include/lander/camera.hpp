#pragma once

// A presentation-only camera for the Lunar Lander GUI.
//
// The camera has a world-space centre (x, y) and a zoom (a multiplier on a
// base pixels-per-meter scale). It offers two modes:
//
//   * AUTO   - the default. Chooses an overview scale at high altitude and a
//              landing scale near the surface, based on the followed target's
//              altitude above local terrain, with hysteresis to avoid flicker
//              and smoothing so the scale eases between the two.
//   * MANUAL - the player sets the zoom with the mouse wheel; the camera
//              still follows the target's position.
//
// The caller decides where the camera should look (target_x / target_y) each
// update; the camera eases smoothly toward that point. In the GUI the target
// is the lander, with a zoom-proportional vertical bias so the lander sits at
// a fixed fraction from the top of the screen and the terrain below it stays
// in view at every zoom level (see gui.cpp).
//
// This class is pure presentation logic: it is driven by the target's
// position, altitude, and a small amount of input, and it never writes to the
// simulation, the terrain, or any world geometry. The scale() it exposes is
// the single world-to-screen factor used by every renderer, so camera motion
// and zoom can only rigidly move and scale the fixed world-space terrain,
// never change its shape.

#include <cmath>

namespace lander {

enum class CameraMode {
    kAuto,
    kManual,
};

// Tunable presentation parameters. These are gameplay values, not physics
// constants, and may be adjusted if a different view is clearly better.
struct CameraParams {
    double base_scale = 14.0;            // pixels per meter at zoom 1.0
    double overview_zoom = 0.40;         // AUTO scale while high altitude
    double landing_zoom = 1.40;          // AUTO scale while near the surface
    // The landing view's downward reach is 0.7 * windowHeight = 36 / landing
    // zoom metres (~25.7 m at 1.4x). alt_to_overview stays at or below that
    // reach so the ground never drops out of the frame while the landing
    // state is active (e.g. while the player climbs back up through the dead
    // band); alt_to_landing sits below it for a stable hysteresis gap.
    double alt_to_landing = 18.0;        // metres; below this AUTO -> landing
    double alt_to_overview = 25.0;       // metres; above this AUTO -> overview
    double zoom_min = 0.20;              // MANUAL zoom lower clamp
    double zoom_max = 4.0;               // MANUAL zoom upper clamp
    double follow_rate = 4.0;            // 1/s, position smoothing
    double zoom_rate = 3.0;              // 1/s, AUTO zoom smoothing
    double wheel_step = 0.12;            // fraction of zoom change per notch
    double lander_top_fraction = 0.30;   // GUI framing: lander at this fraction
                                         // down from the top of the screen
};

class Camera {
public:
    Camera() = default;
    explicit Camera(const CameraParams& params) : params_(params) {}

    // Place the camera exactly on (x, y) with a neutral zoom and restart in
    // AUTO. Used when the game is started or reset; the caller supplies the
    // already-framed target point.
    void snap(double x, double y) {
        mode_ = CameraMode::kAuto;
        wants_landing_ = true;
        zoom_ = 1.0;
        pos_x_ = x;
        pos_y_ = y;
    }

    CameraMode mode() const { return mode_; }
    double zoom() const { return zoom_; }
    double x() const { return pos_x_; }
    double y() const { return pos_y_; }

    // Which scale AUTO is currently easing toward. Exposed for tests.
    bool auto_wants_landing() const { return wants_landing_; }

    // Tunable parameters, exposed read-only so the GUI can share the same
    // values (e.g. the lander framing fraction) without duplicating them.
    const CameraParams& params() const { return params_; }

    // Effective pixels-per-meter at the current zoom. This is the single
    // world-to-screen scale used by all renderers.
    double scale() const { return params_.base_scale * zoom_; }

    // Advance the camera by dt seconds.
    //   target_x / target_y: the world point the camera should ease toward.
    //   altitude: the followed target's height above local terrain, metres.
    //   wheel_delta: mouse-wheel notches accumulated since the last update
    //                (positive = up = zoom in, negative = down = zoom out);
    //                applied only in MANUAL mode.
    //   toggle_mode: true if the camera-mode key was pressed this frame.
    // Pure with respect to the target: it reads the supplied values only and
    // never modifies the simulation or the terrain.
    void update(double dt, double target_x, double target_y, double altitude,
                int wheel_delta, bool toggle_mode) {
        if (toggle_mode) {
            mode_ = (mode_ == CameraMode::kAuto) ? CameraMode::kManual
                                                : CameraMode::kAuto;
            // Entering MANUAL keeps the current zoom (no visible jump);
            // returning to AUTO leaves it too and the auto target eases it.
        }

        if (mode_ == CameraMode::kManual) {
            if (wheel_delta > 0) {
                zoom_ *= 1.0 + params_.wheel_step;  // wheel up: zoom in
            } else if (wheel_delta < 0) {
                zoom_ *= 1.0 - params_.wheel_step;  // wheel down: zoom out
            }
            zoom_ = clamp(zoom_, params_.zoom_min, params_.zoom_max);
        } else {
            // AUTO: pick overview vs landing with hysteresis so hovering in
            // the dead band between the two thresholds does not flicker.
            if (!wants_landing_) {
                if (altitude < params_.alt_to_landing) {
                    wants_landing_ = true;
                }
            } else if (altitude > params_.alt_to_overview) {
                wants_landing_ = false;
            }
            if (dt > 0.0) {
                const double target_zoom =
                    wants_landing_ ? params_.landing_zoom
                                   : params_.overview_zoom;
                const double a = 1.0 - std::exp(-params_.zoom_rate * dt);
                zoom_ += (target_zoom - zoom_) * a;
            }
        }

        // Ease the centre toward the caller-supplied point. The vertical
        // framing bias (so the ground stays in view) is already baked into
        // target_y by the caller and scales with the current zoom.
        if (dt > 0.0) {
            const double a = 1.0 - std::exp(-params_.follow_rate * dt);
            pos_x_ += (target_x - pos_x_) * a;
            pos_y_ += (target_y - pos_y_) * a;
        }
    }

private:
    static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    CameraParams params_{};
    CameraMode mode_ = CameraMode::kAuto;
    bool wants_landing_ = true;
    double zoom_ = 1.0;
    double pos_x_ = 0.0;
    double pos_y_ = 20.0;
};

}  // namespace lander
