#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lander {

enum class CameraMode {
    kAuto,
    kManual,
    kSystem,
};

struct CameraParams {
    double base_scale = 14.0;
    double overview_zoom = 0.46;
    double landing_zoom = 1.40;
    double alt_to_landing = 18.0;
    double alt_to_overview = 25.0;
    double zoom_min = 0.20;
    double zoom_max = 4.0;
    double follow_rate = 4.0;
    double zoom_rate = 3.0;
    double wheel_step = 0.12;
    double lander_top_fraction = 0.30;
    double window_width = 1280.0;
    double window_height = 720.0;
    // SYSTEM view (M05): an inertial, unrotated view of the whole binary.
    // The default 0.04 gives 0.56 px/m at base_scale 14, enough to see the
    // 600 m separation inside a 1280 px viewport. M05-R3 allows zooming up
    // to 2.0 (28 px/m) so the ship itself can become readable without
    // leaving the inertial system view.
    double system_zoom = 0.04;
    double system_zoom_min = 0.01;
    double system_zoom_max = 2.0;
    double system_viewport_margin = 80.0;
    double angle_transition_time = 0.35;
    double angle_jump_threshold = 0.4;
};

inline constexpr double kHalfPi = 0.5 * 3.14159265358979323846;
inline constexpr double kLanderMajorMetres = 2.5;
inline constexpr double kMinReadableLanderPx = 16.0;
inline constexpr double kTargetCueLengthPx = 24.0;
inline constexpr double kVelocityCueScale = 2.5;
inline constexpr double kGravityCueScale = 18.0;
inline constexpr double kCueMaxLengthPx = 96.0;

inline bool lander_uses_full_model(double scale) {
    return scale * kLanderMajorMetres >= kMinReadableLanderPx - 1.0e-12;
}

inline double camera_readability_zoom(const CameraParams& p) {
    return kMinReadableLanderPx / (p.base_scale * kLanderMajorMetres);
}

inline double cue_arrow_length(double magnitude, double px_per_unit,
                               double max_px) {
    return std::clamp(magnitude * px_per_unit, 0.0, max_px);
}

inline double system_frame_zoom(double distance, const CameraParams& p) {
    if (distance < 1.0e-6) {
        return p.system_zoom_max;
    }
    const double available =
        0.5 * std::min(p.window_width, p.window_height) -
        p.system_viewport_margin;
    if (available <= 0.0) {
        return p.system_zoom_max;
    }
    const double zoom = 2.0 * available / (distance * p.base_scale);
    return std::clamp(zoom, p.system_zoom_min, p.system_zoom_max);
}

// Sentinel meaning "derive the camera angle from the target position" (the
// M04 behavior). NaN is a convenient default argument.
inline constexpr double kNoTargetAngle =
    std::numeric_limits<double>::quiet_NaN();

class Camera {
public:
    Camera() = default;
    explicit Camera(const CameraParams& params)
        : params_(params), system_target_zoom_(params.system_zoom) {}

    // Presentation-only system view: inertial (unrotated) framing of the
    // whole binary with the ship at the viewport centre. Entering it saves
    // the current local mode/zoom/want-landing state so it is restored
    // exactly when the system view is left.
    void set_system(bool enable) {
        if (enable) {
            if (mode_ == CameraMode::kSystem) {
                return;
            }
            saved_mode_ = mode_;
            saved_zoom_ = zoom_;
            saved_wants_landing_ = wants_landing_;
            mode_ = CameraMode::kSystem;
            zoom_ = params_.system_zoom;
            system_target_zoom_ = params_.system_zoom;
            system_zoom_manual_ = false;
            angle_transition_active_ = false;
            angle_ = 0.0;
            angle_initialized_ = true;
        } else {
            if (mode_ != CameraMode::kSystem) {
                return;
            }
            mode_ = saved_mode_;
            zoom_ = saved_zoom_;
            wants_landing_ = saved_wants_landing_;
            system_target_zoom_ = params_.system_zoom;
            system_zoom_manual_ = false;
            angle_transition_active_ = false;
        }
    }
    bool system_view() const { return mode_ == CameraMode::kSystem; }

    void snap(double x, double y, double target_angle = kNoTargetAngle) {
        target_x_ = x;
        target_y_ = y;
        if (mode_ == CameraMode::kSystem) {
            angle_ = 0.0;
            angle_initialized_ = true;
            angle_transition_active_ = false;
            system_zoom_manual_ = false;
            system_target_zoom_ = params_.system_zoom;
            zoom_ = params_.system_zoom;
            focus_x_ = target_x_;
            focus_y_ = target_y_;
            return;
        }
        mode_ = CameraMode::kAuto;
        wants_landing_ = true;
        zoom_ = 1.0;
        system_zoom_manual_ = false;
        set_angle(target_angle, x, y);

        const double offset = framing_offset();
        const double c = std::cos(angle_);
        const double s = std::sin(angle_);
        const double up_x = -s;
        const double up_y = c;
        focus_x_ = target_x_ - up_x * offset;
        focus_y_ = target_y_ - up_y * offset;
    }

    CameraMode mode() const { return mode_; }
    double zoom() const { return zoom_; }
    double scale() const { return params_.base_scale * zoom_; }
    double angle() const { return angle_; }
    double center_x() const { return focus_x_; }
    double center_y() const { return focus_y_; }
    bool auto_wants_landing() const { return wants_landing_; }
    const CameraParams& params() const { return params_; }

    void set_system_destination(double x, double y) {
        has_system_destination_ = true;
        system_destination_x_ = x;
        system_destination_y_ = y;
    }
    void clear_system_destination() { has_system_destination_ = false; }
    bool has_system_destination() const { return has_system_destination_; }
    double system_destination_x() const { return system_destination_x_; }
    double system_destination_y() const { return system_destination_y_; }
    bool system_zoom_manual() const { return system_zoom_manual_; }
    double system_target_zoom() const { return system_target_zoom_; }

    void update(double dt, double target_x, double target_y, double altitude,
                int wheel_delta, bool toggle_mode,
                double target_angle = kNoTargetAngle) {
        target_x_ = target_x;
        target_y_ = target_y;

        if (mode_ == CameraMode::kSystem) {
            // Inertial framing: fixed angle, ship at the exact viewport
            // centre (no local anchor offset), wheel changing the target
            // zoom multiplicatively, rendered zoom easing toward that target.
            // Local mode/zoom/want-landing state is untouched.
            if (wheel_delta > 0) {
                system_target_zoom_ *= 1.0 + params_.wheel_step;
                system_zoom_manual_ = true;
            } else if (wheel_delta < 0) {
                system_target_zoom_ *= 1.0 - params_.wheel_step;
                system_zoom_manual_ = true;
            }
            system_target_zoom_ = clamp(
                system_target_zoom_, params_.system_zoom_min,
                params_.system_zoom_max);

            if (has_system_destination_ && !system_zoom_manual_) {
                const double dx = system_destination_x_ - target_x_;
                const double dy = system_destination_y_ - target_y_;
                const double distance = std::hypot(dx, dy);
                const double available =
                    0.5 * std::min(params_.window_width, params_.window_height) -
                    params_.system_viewport_margin;
                if (available > 0.0 &&
                    distance * params_.base_scale * params_.system_zoom_min <=
                        2.0 * available + 1.0e-9) {
                    system_target_zoom_ =
                        system_frame_zoom(distance, params_);
                } else {
                    system_target_zoom_ = params_.system_zoom;
                }
            }

            if (dt <= 0.0) {
                zoom_ = system_target_zoom_;
            } else {
                const double a = 1.0 - std::exp(-params_.zoom_rate * dt);
                zoom_ += (system_target_zoom_ - zoom_) * a;
            }
            zoom_ = clamp(zoom_, params_.system_zoom_min,
                          params_.system_zoom_max);
            angle_ = 0.0;
            angle_initialized_ = true;
            angle_transition_active_ = false;
            focus_x_ = target_x_;
            focus_y_ = target_y_;
            if (has_system_destination_) {
                const double dx = system_destination_x_ - target_x_;
                const double dy = system_destination_y_ - target_y_;
                const double distance = std::hypot(dx, dy);
                const double available =
                    0.5 * std::min(params_.window_width, params_.window_height) -
                    params_.system_viewport_margin;
                const auto fits = [&](double z) {
                    return distance * params_.base_scale * z <=
                           2.0 * available + 1.0e-9;
                };
                if (available > 0.0 && fits(zoom_)) {
                    focus_x_ = 0.5 * (target_x_ + system_destination_x_);
                    focus_y_ = 0.5 * (target_y_ + system_destination_y_);
                }
            }
            return;
        }

        if (toggle_mode) {
            mode_ = (mode_ == CameraMode::kAuto) ? CameraMode::kManual
                                                 : CameraMode::kAuto;
        }

        if (mode_ == CameraMode::kManual) {
            if (wheel_delta > 0) {
                zoom_ *= 1.0 + params_.wheel_step;
            } else if (wheel_delta < 0) {
                zoom_ *= 1.0 - params_.wheel_step;
            }
            zoom_ = clamp(zoom_, params_.zoom_min, params_.zoom_max);
        } else {
            if (!wants_landing_) {
                if (altitude < params_.alt_to_landing) {
                    wants_landing_ = true;
                }
            } else if (altitude > params_.alt_to_overview) {
                wants_landing_ = false;
            }
            if (dt > 0.0) {
                double target_zoom =
                    wants_landing_ ? params_.landing_zoom
                                   : params_.overview_zoom;
                target_zoom = std::max(target_zoom,
                                       camera_readability_zoom(params_));
                const double a = 1.0 - std::exp(-params_.zoom_rate * dt);
                zoom_ += (target_zoom - zoom_) * a;
            }
        }

        update_angle(dt, target_angle, target_x, target_y);

        // The player-follow anchor is exact: the target is placed at the
        // configured screen position without follow lag. The local-frame
        // basis itself rotates with the target, so smoothing the focus in that
        // rotating frame would accumulate horizontal drift during an orbit.
        const double c = std::cos(angle_);
        const double s = std::sin(angle_);
        const double up_x = -s;
        const double up_y = c;
        const double offset = framing_offset();
        focus_x_ = target_x_ - up_x * offset;
        focus_y_ = target_y_ - up_y * offset;
    }

private:
    void set_angle(double target_angle, double x, double y) {
        angle_transition_active_ = false;
        angle_initialized_ = true;
        if (!std::isnan(target_angle)) {
            angle_ = target_angle;
            return;
        }
        const double rho = std::hypot(x, y);
        if (rho > 1.0e-9) {
            angle_ = std::atan2(y, x) - kHalfPi;
        }
    }

    double desired_angle(double target_angle, double x, double y) const {
        if (!std::isnan(target_angle)) {
            return target_angle;
        }
        const double rho = std::hypot(x, y);
        if (rho > 1.0e-9) {
            return std::atan2(y, x) - kHalfPi;
        }
        return angle_;
    }

    static double shortest_arc(double from, double to) {
        return std::atan2(std::sin(to - from), std::cos(to - from));
    }

    void update_angle(double dt, double target_angle, double x, double y) {
        const double desired = desired_angle(target_angle, x, y);
        if (dt <= 0.0) {
            angle_ = desired;
            angle_initialized_ = true;
            angle_transition_active_ = false;
            return;
        }
        if (!angle_initialized_) {
            angle_ = desired;
            angle_initialized_ = true;
            angle_transition_active_ = false;
            return;
        }

        const double delta = shortest_arc(angle_, desired);
        if (std::abs(delta) <= params_.angle_jump_threshold) {
            angle_ = desired;
            angle_transition_active_ = false;
            return;
        }

        if (!angle_transition_active_) {
            angle_transition_active_ = true;
            angle_transition_from_ = angle_;
            angle_transition_to_ = desired;
            angle_transition_total_ =
                std::max(params_.angle_transition_time, 1.0e-6);
            angle_transition_remaining_ =
                std::max(angle_transition_total_ - dt, 0.0);
        } else if (std::abs(
                       shortest_arc(angle_transition_to_, desired)) > 0.05) {
            angle_transition_from_ = angle_;
            angle_transition_to_ = desired;
            angle_transition_remaining_ =
                std::max(angle_transition_total_ - dt, 0.0);
        } else {
            angle_transition_remaining_ =
                std::max(angle_transition_remaining_ - dt, 0.0);
        }

        const double arc =
            shortest_arc(angle_transition_from_, angle_transition_to_);
        const double p = 1.0 -
                         angle_transition_remaining_ / angle_transition_total_;
        const double t = clamp(p, 0.0, 1.0);
        const double eased = t * t * (3.0 - 2.0 * t);
        if (angle_transition_remaining_ <= 0.0) {
            angle_ = angle_transition_to_;
            angle_transition_active_ = false;
        } else {
            angle_ = angle_transition_from_ + arc * eased;
        }
    }

    double framing_offset() const {
        return (0.5 - params_.lander_top_fraction) *
               (params_.window_height / scale());
    }

    static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    CameraParams params_{};
    CameraMode mode_ = CameraMode::kAuto;
    CameraMode saved_mode_ = CameraMode::kAuto;
    double saved_zoom_ = 1.0;
    bool saved_wants_landing_ = true;
    bool wants_landing_ = true;
    double zoom_ = 1.0;
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double focus_x_ = 0.0;
    double focus_y_ = 0.0;
    double angle_ = 0.0;
    bool angle_initialized_ = false;
    bool angle_transition_active_ = false;
    double angle_transition_from_ = 0.0;
    double angle_transition_to_ = 0.0;
    double angle_transition_remaining_ = 0.0;
    double angle_transition_total_ = 0.0;
    bool system_zoom_manual_ = false;
    double system_target_zoom_ = 0.04;
    bool has_system_destination_ = false;
    double system_destination_x_ = 0.0;
    double system_destination_y_ = 0.0;
};

struct MarkerPoint {
    double x{};
    double y{};
};

struct MarkerTriangle {
    MarkerPoint nose{};
    MarkerPoint left{};
    MarkerPoint right{};
};

inline MarkerTriangle marker_triangle(double cx, double cy, double ship_angle,
                                      double camera_angle, double size) {
    const double nx = -std::sin(ship_angle);
    const double ny = std::cos(ship_angle);
    const double c = std::cos(camera_angle);
    const double s = std::sin(camera_angle);
    double dx = nx * c + ny * s;
    double dy = nx * s - ny * c;
    double len = std::hypot(dx, dy);
    if (len < 1.0e-9) {
        dx = 0.0;
        dy = -1.0;
        len = 1.0;
    } else {
        dx /= len;
        dy /= len;
    }
    const MarkerPoint nose{cx + dx * size, cy + dy * size};
    const double px = -dy;
    const double py = dx;
    const double bx = cx - dx * 0.6 * size;
    const double by = cy - dy * 0.6 * size;
    return {nose,
            {bx + px * 0.5 * size, by + py * 0.5 * size},
            {bx - px * 0.5 * size, by - py * 0.5 * size}};
}

struct ScreenRect {
    int x{};
    int y{};
    int w{};
    int h{};
};

// M05-R3-08: content-sized, screen-space crash-dialog geometry. The result
// depends only on the modal's text content and the viewport, never on terrain
// clipping, reference body, camera mode, or SYSTEM zoom.
inline ScreenRect crash_modal_rect(int title_chars, int title_scale,
                                   int score_chars, int score_scale,
                                   int hint_chars, int hint_scale,
                                   int viewport_width, int viewport_height) {
    auto line_width = [](int chars, int scale) {
        return std::max(0, chars * 6 * scale - scale);
    };
    constexpr int kPaddingX = 28;
    constexpr int kPaddingY = 24;
    constexpr int kGap = 16;

    const int content_width = std::max(
        std::max(line_width(title_chars, title_scale),
                 line_width(score_chars, score_scale)),
        line_width(hint_chars, hint_scale));
    const int max_width = std::max(80, viewport_width - 48);
    const int max_height = std::max(60, viewport_height - 48);
    const int width = std::clamp(content_width + 2 * kPaddingX, 80,
                                 max_width);
    const int height = std::clamp(
        2 * kPaddingY + 7 * title_scale + kGap + 7 * score_scale + kGap +
            7 * hint_scale,
        60, max_height);

    const int x = std::clamp((viewport_width - width) / 2, 24,
                             std::max(24, viewport_width - width - 24));
    const int y = std::clamp((viewport_height - height) / 2, 24,
                             std::max(24, viewport_height - height - 24));
    return {x, y, width, height};
}

struct OffscreenIndicator {
    bool on_screen{};
    double x{};
    double y{};
    double dir_x{};
    double dir_y{};
};

inline OffscreenIndicator offscreen_target_indicator(double target_x,
                                                     double target_y,
                                                     const Camera& cam) {
    const CameraParams& p = cam.params();
    const double dx = target_x - cam.center_x();
    const double dy = target_y - cam.center_y();
    const double c = std::cos(cam.angle());
    const double s = std::sin(cam.angle());
    const double local_x = dx * c + dy * s;
    const double local_y = -dx * s + dy * c;
    const double sx = p.window_width / 2.0 + local_x * cam.scale();
    const double sy = p.window_height / 2.0 - local_y * cam.scale();

    OffscreenIndicator out;
    if (sx >= 0.0 && sx <= p.window_width && sy >= 0.0 &&
        sy <= p.window_height) {
        out.on_screen = true;
        out.x = sx;
        out.y = sy;
        return out;
    }

    const double cx = p.window_width / 2.0;
    const double cy = p.window_height / 2.0;
    double vx = sx - cx;
    double vy = sy - cy;
    const double vlen = std::hypot(vx, vy);
    if (vlen < 1.0e-9) {
        out.on_screen = false;
        return out;
    }
    vx /= vlen;
    vy /= vlen;
    const double tx =
        std::abs(vx) > 1.0e-12 ? 0.5 * p.window_width / std::abs(vx)
                               : 1.0e30;
    const double ty =
        std::abs(vy) > 1.0e-12 ? 0.5 * p.window_height / std::abs(vy)
                               : 1.0e30;
    const double t = std::min(tx, ty);
    out.on_screen = false;
    out.x = cx + vx * t;
    out.y = cy + vy * t;
    out.dir_x = vx;
    out.dir_y = vy;
    return out;
}

}  // namespace lander
