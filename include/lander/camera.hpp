#pragma once

#include <cmath>
#include <cstdint>

namespace lander {

enum class CameraMode {
    kAuto,
    kManual,
};

struct CameraParams {
    double base_scale = 14.0;
    double overview_zoom = 0.40;
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
};

class Camera {
public:
    Camera() = default;
    explicit Camera(const CameraParams& params) : params_(params) {}

    void snap(double x, double y) {
        mode_ = CameraMode::kAuto;
        wants_landing_ = true;
        zoom_ = 1.0;
        target_x_ = x;
        target_y_ = y;
        update_angle(x, y);

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

    void update(double dt, double target_x, double target_y, double altitude,
                int wheel_delta, bool toggle_mode) {
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
                const double target_zoom =
                    wants_landing_ ? params_.landing_zoom
                                   : params_.overview_zoom;
                const double a = 1.0 - std::exp(-params_.zoom_rate * dt);
                zoom_ += (target_zoom - zoom_) * a;
            }
        }

        target_x_ = target_x;
        target_y_ = target_y;
        update_angle(target_x, target_y);

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
    void update_angle(double x, double y) {
        const double rho = std::hypot(x, y);
        if (rho > 1.0e-9) {
            angle_ = std::atan2(y, x) - 0.5 * 3.14159265358979323846;
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
    bool wants_landing_ = true;
    double zoom_ = 1.0;
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double focus_x_ = 0.0;
    double focus_y_ = 0.0;
    double angle_ = 0.0;
};

}  // namespace lander
