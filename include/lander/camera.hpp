#pragma once

// A presentation-only camera for the Lunar Lander GUI.
//
// The camera has a smoothed world-space *horizontal focus* (eased toward the
// caller's target x each update) and an exact *vertical framing anchor*
// derived from the latest target y, plus a *zoom* (a multiplier on a base
// pixels-per-meter scale). From those it derives its world-space centre, so
// the whole view can be expressed as a single rigid transform (centre +
// scale) applied to the fixed world-space terrain.
//
// It offers two modes:
//
//   * AUTO   - the default. Chooses an overview scale at high altitude and a
//              landing scale near the surface, based on the followed target's
//              altitude above local terrain, with hysteresis to avoid flicker
//              and smoothing so the scale eases between the two.
//   * MANUAL - the player sets the zoom with the mouse wheel; the camera
//              still follows the target's position.
//
// Framing: the camera places the followed target's world y at a fixed screen
// anchor - horizontally centred and a fixed fraction (lander_top_fraction)
// down from the top of the window - and it knows the window size to do so.
// That anchor is what makes the view useful: the terrain below the lander
// stays in view at every zoom level, and the lander is never pushed off the
// top when zoomed in.
//
// The horizontal focus eases toward the target (retained position-follow
// smoothing), but the vertical centre is recomputed directly from the latest
// target y and the *current* scale on every update. A zoom change therefore
// re-centres the view immediately in the same frame, so the lander's screen
// y is invariant at every scale and repeated rapid wheel events cannot
// accumulate framing error.
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
    double lander_top_fraction = 0.30;   // lander at this fraction down from
                                          // the top of the screen
    double window_width = 1280.0;        // viewport size used to place the
    double window_height = 720.0;        // framing anchor (match the window)
};

class Camera {
public:
    Camera() = default;
    explicit Camera(const CameraParams& params) : params_(params) {}

    // Place the horizontal focus and vertical anchor exactly on (x, y) with a
    // neutral zoom and restart in AUTO. Used when the game is started or
    // reset; the caller supplies the raw world point of the followed target.
    // The derived centre then frames that point at the fixed screen anchor
    // for the current (neutral) scale.
    void snap(double x, double y) {
        mode_ = CameraMode::kAuto;
        wants_landing_ = true;
        zoom_ = 1.0;
        focus_x_ = x;
        target_y_ = y;
    }

    CameraMode mode() const { return mode_; }
    double zoom() const { return zoom_; }
    // World-space centre of the view. The x is the smoothed horizontal focus
    // (retained follow behaviour); the y is the latest target y shifted down
    // by a framing offset that scales with the *current* zoom, so the lander
    // stays at its fixed screen anchor at every scale.
    double x() const { return focus_x_; }
    double y() const { return target_y_ - framing_offset_y(); }

    // Which scale AUTO is currently easing toward. Exposed for tests.
    bool auto_wants_landing() const { return wants_landing_; }

    // Tunable parameters, exposed read-only so the GUI can share the same
    // values (e.g. the viewport size) without duplicating them.
    const CameraParams& params() const { return params_; }

    // Effective pixels-per-meter at the current zoom. This is the single
    // world-to-screen scale used by all renderers.
    double scale() const { return params_.base_scale * zoom_; }

    // Advance the camera by dt seconds.
    //   target_x / target_y: the world point (the followed lander) that the
    //                       focus should ease toward.
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

        // The vertical anchor always uses the latest target y (never a
        // smoothed copy), while the horizontal focus eases toward the target
        // x. Both depend only on the target and dt - never on the zoom - and
        // y() recomputes its framing offset from the current scale on every
        // access, so a zoom change re-centres the view immediately in the
        // same frame.
        target_y_ = target_y;
        if (dt > 0.0) {
            const double a = 1.0 - std::exp(-params_.follow_rate * dt);
            focus_x_ += (target_x - focus_x_) * a;
        }
    }

private:
    // The downward (world) offset from the target y that places the target at
    // the fixed screen anchor. It grows as the zoom decreases, so the view
    // keeps a constant *world* reach below the lander and the terrain never
    // leaves the frame at any scale.
    double framing_offset_y() const {
        return (0.5 - params_.lander_top_fraction) *
               (params_.window_height / scale());
    }

    static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    CameraParams params_{};
    CameraMode mode_ = CameraMode::kAuto;
    bool wants_landing_ = true;
    double zoom_ = 1.0;
    double focus_x_ = 0.0;
    double target_y_ = 20.0;
};

}  // namespace lander
