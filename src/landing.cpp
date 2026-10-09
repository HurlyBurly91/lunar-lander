// M06-R6: low-complexity powered-landing guidance — ZEM/ZEV from a maintained
// zero-effort reference, a bounded K-candidate time-to-go scan, and a landing
// phase state machine that picks the moving target state. The command is a
// physical thrust correction (no second gravity term, no direct state
// mutation); the ordinary Simulation landing checker stays authoritative.
#include "lander/landing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace lander {

namespace {

double vec_len(const Vec2& v) { return std::hypot(v.x, v.y); }

// The selected pad is the body's base pad (body-local arc 0). Its body-local
// angle and surface radius track the tidal-lock spin, so the target point is
// moving at the terminal time.
double pad_local_angle(const Body& body) {
    return body.terrain.angle_at_arc(0.0);
}

double pad_surface_radius(const Body& body) {
    return body.terrain.surface_radius_at_arc(0.0);
}

// Target-body-centred radial/tangential frame for the high-energy VGO
// capture/deorbit laws (V14 C). `r_hat` points from the target body centre to
// the ship; `t_hat` is the 90-degree counter-clockwise perpendicular (the
// local tangential direction). `v_r`/`v_t` decompose the ship's velocity
// relative to the target body in that frame. This is the "no waypoint chasing"
// reference the user-specified VGO law drives against.
struct RTFrame {
    Vec2 r_hat{};
    Vec2 t_hat{};
    double r{0.0};
    double v_r{0.0};
    double v_t{0.0};
};

RTFrame target_rt_frame(const State& s, const BinarySystem& bin, int body,
                        double now) {
    RTFrame f;
    const Vec2 c = bin.position(body, now);
    f.r = std::hypot(s.x - c.x, s.y - c.y);
    if (f.r > 1e-9) {
        f.r_hat = {(s.x - c.x) / f.r, (s.y - c.y) / f.r};
    } else {
        f.r_hat = {1.0, 0.0};
    }
    f.t_hat = {-f.r_hat.y, f.r_hat.x};  // 90 deg counter-clockwise
    const Vec2 cv = bin.velocity(body, now);
    const Vec2 v{s.vx - cv.x, s.vy - cv.y};
    f.v_r = v.x * f.r_hat.x + v.y * f.r_hat.y;
    f.v_t = v.x * f.t_hat.x + v.y * f.t_hat.y;
    return f;
}

// VGO acceleration: cancel the net two-body field (-g) and drive the
// target-relative velocity to (v_r_des, v_t_des) with a first-order lag `tau`:
//     a_cmd = -g + ((v_r_des - v_r) * r_hat + (v_t_des - v_t) * t_hat) / tau
// The caller chooses the desired radial/tangential targets; direction of
// motion is preserved by the caller (sign of v_t_des).
Vec2 vgo_accel(const State& s, const BinarySystem& bin, int body, double now,
               double v_r_des, double v_t_des, double tau) {
    const RTFrame f = target_rt_frame(s, bin, body, now);
    const Vec2 g = bin.gravity({s.x, s.y}, now);
    const double t = 1.0 / std::max(1e-9, tau);
    const double dr = v_r_des - f.v_r;
    const double dt = v_t_des - f.v_t;
    return {-g.x + t * (dr * f.r_hat.x + dt * f.t_hat.x),
            -g.y + t * (dr * f.r_hat.y + dt * f.t_hat.y)};
}

}  // namespace

const char* landing_phase_name(LandingPhase phase) {
    switch (phase) {
        case LandingPhase::Ascend:
            return "ASCEND";
        case LandingPhase::Transfer:
            return "TRANSFER";
        case LandingPhase::Capture:
            return "CAPTURE";
        case LandingPhase::Deorbit:
            return "DE-ORBIT";
        case LandingPhase::Brake:
            return "BRAKE";
        case LandingPhase::Approach:
            return "APPROACH";
        case LandingPhase::Descent:
            return "DESCENT";
        case LandingPhase::Touchdown:
            return "TOUCHDOWN";
    }
    return "TOUCHDOWN";
}

// BEGIN CANONICAL ALGORITHM: powered landing target-state construction
// Reference:
// docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md
LandingTargetState landing_target_state(const BinarySystem& bin, int target_body,
                                        LandingPhase phase,
                                        const LandingConfig& config, double now,
                                        double t_go) {
    const Body& body = bin.body(target_body);
    const double t_f = now + t_go;
    const double pad_angle = pad_local_angle(body);
    const double pad_radius = pad_surface_radius(body);
    if (phase == LandingPhase::Descent) {
        // Terminal touchdown: the physical pad's moving surface point.
        const auto sp =
            bin.surface_point(target_body, pad_angle, pad_radius, t_f);
        return {sp.position, sp.velocity};
    }
    // APPROACH and the earlier phases target a fixed offset radially out from
    // the pad: the co-rotating hover waypoint, with that moving point's
    // velocity, so the vehicle hovers over the pad without touching it.
    const double hover_radius = pad_radius + config.hover_height;
    const auto sp =
        bin.surface_point(target_body, pad_angle, hover_radius, t_f);
    return {sp.position, sp.velocity};
}
// END CANONICAL ALGORITHM: powered landing target-state construction

// BEGIN CANONICAL ALGORITHM: powered landing ZEM-ZEV guidance
// Reference:
// docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md
Vec2 landing_acceleration_command(const Vec2& zem, const Vec2& zev, double t_go) {
    if (t_go <= 1e-9) {
        return {};
    }
    const double inv = 1.0 / t_go;
    return {6.0 * zem.x * inv * inv - 2.0 * zev.x * inv,
            6.0 * zem.y * inv * inv - 2.0 * zev.y * inv};
}

// Apollo-style refinement of the same ZEM/ZEV errors: a linear total
// acceleration profile a(t) = a0 + a1*t over the candidate terminal time. The
// held guidance command is the interval average of that profile over the
// guidance interval.
struct AccelProfile {
    Vec2 a0{};
    Vec2 a1{};
};

AccelProfile landing_acceleration_profile(const Vec2& zem, const Vec2& zev,
                                          double t_go) {
    if (t_go <= 1e-9) {
        return {};
    }
    const double inv = 1.0 / t_go;
    const double inv2 = inv * inv;
    const double inv3 = inv2 * inv;
    return {{6.0 * zem.x * inv2 - 2.0 * zev.x * inv,
             6.0 * zem.y * inv2 - 2.0 * zev.y * inv},
            {6.0 * zev.x * inv2 - 12.0 * zem.x * inv3,
             6.0 * zev.y * inv2 - 12.0 * zem.y * inv3}};
}

Vec2 landing_polynomial_command(const Vec2& zem, const Vec2& zev, double t_go,
                                double dt) {
    const AccelProfile p = landing_acceleration_profile(zem, zev, t_go);
    const double half_dt = 0.5 * dt;
    return {p.a0.x + p.a1.x * half_dt, p.a0.y + p.a1.y * half_dt};
}
// END CANONICAL ALGORITHM: powered landing ZEM-ZEV guidance

// ZEM (position error) and ZEV (velocity error) evaluated in the inertial
// world frame between the future zero-effort state and the future moving
// target state. The Apollo-style polynomial command is an inertial-frame
// total acceleration profile; evaluating it in a rotating body frame would
// mix in fictitious Coriolis/centrifusal terms that the closed-form ZEM/ZEV
// derivation does not include.
struct ZemZev {
    Vec2 zem{};
    Vec2 zev{};
    Vec2 zero_p{};
    Vec2 target_p{};
};

ZemZev compute_zem_zev(const BinarySystem& bin, int target_body,
                       LandingPhase phase, const LandingConfig& config,
                       double now, double t_go, const BallisticState& zero) {
    const double t_f = now + t_go;
    const LandingTargetState target =
        landing_target_state(bin, target_body, phase, config, now, t_go);
    ZemZev out;
    out.zem = {target.position.x - zero.p.x, target.position.y - zero.p.y};
    out.zev = {target.velocity.x - zero.v.x, target.velocity.y - zero.v.y};
    out.zero_p = zero.p;
    out.target_p = target.position;
    (void)t_f;
    return out;
}

// BEGIN CANONICAL ALGORITHM: powered landing bounded time-to-go selection
// Reference:
// docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md
std::optional<double> landing_select_time_to_go(
    const BinarySystem& bin, const Config& config, const LandingConfig& landing,
    double now, double t_go_estimate, int target_body, LandingPhase phase,
    const ZeroEffortQuery& zero_effort) {
    if (!zero_effort) {
        return std::nullopt;
    }
    double best_tgo = 0.0;
    double best_cost = 0.0;
    bool found = false;

    // The reference outward direction at the current epoch.  For DESCENT this
    // is the local surface normal; for APPROACH and earlier phases it is the
    // hover-waypoint normal.  It is used only to bias the bounded scan toward
    // commands that the existing attitude controller can actually execute.
    Vec2 ref_outward{};
    {
        const Vec2 c0 = bin.position(target_body, now);
        const LandingTargetState t0 =
            landing_target_state(bin, target_body, phase, landing, now, 0.0);
        const Vec2 o{t0.position.x - c0.x, t0.position.y - c0.y};
        const double ol = vec_len(o);
        if (ol > 1e-9) {
            ref_outward = {o.x / ol, o.y / ol};
        }
    }

    // Deterministic bounded scan: a fixed-size log-spaced grid across the
    // configured t_go window.  The current estimate biases the cost, not the
    // geometry of the scan, so the scan remains O(1) and always covers both
    // short (aggressive) and long (hover-like) terminal times.
    const int K = landing.t_go_candidates > 1 ? landing.t_go_candidates : 2;
    const double log_ratio =
        std::log(landing.t_go_max / landing.t_go_min);
    for (int k = 0; k < K; ++k) {
        const double frac = (K == 1) ? 0.0 : (double)k / (double)(K - 1);
        double cand =
            landing.t_go_min * std::exp(frac * log_ratio);
        cand = std::clamp(cand, landing.t_go_min, landing.t_go_max);

        const BallisticState zero = zero_effort(cand);

        // The zero-effort source is a nearest-neighbour sample from the
        // receding-horizon predictor, so its time can differ from `now + cand`
        // by up to one simulation step.  A larger gap means the ballistic
        // prediction has already terminated (crash / landing) and the
        // terminal state is stale; such candidates are skipped.
        const double gap = (now + cand) - zero.t;
        const double tol = 2.0 * config.fixed_dt;
        if (gap > 1.0) {
            continue;
        }

        const ZemZev zz =
            compute_zem_zev(bin, target_body, phase, landing, now, cand, zero);
        const AccelProfile profile =
            landing_acceleration_profile(zz.zem, zz.zev, cand);
        const Vec2 a_term{profile.a0.x + profile.a1.x * cand,
                          profile.a0.y + profile.a1.y * cand};
        const double peak_len =
            std::max(vec_len(profile.a0), vec_len(a_term));
        if (!std::isfinite(peak_len) || peak_len > config.main_accel) {
            continue;
        }
        if (phase == LandingPhase::Descent &&
            vec_len(ref_outward) > 1e-9) {
            const double initial_radial =
                profile.a0.x * ref_outward.x + profile.a0.y * ref_outward.y;
            if (initial_radial < 0.0) {
                continue;
            }
        }
        const Vec2 held =
            landing_polynomial_command(zz.zem, zz.zev, cand, landing.guidance_interval);
        const double held_len = vec_len(held);
        if (!std::isfinite(held_len)) {
            continue;
        }

        // Deterministic cost:
        //   * prefer candidates close to the current t_go estimate;
        //   * penalise stale zero-effort samples.
        // Candidates whose Apollo profile peak exceeds `main_accel` are
        // rejected above, so no throttle-saturation term is needed here.
        const double est_err =
            std::fabs(cand - t_go_estimate) / landing.t_go_max;
        const double cost =
            1.0 * est_err + 0.1 * std::max(0.0, gap - tol);
        if (!std::isfinite(cost)) {
            continue;
        }
        if (!found || cost < best_cost) {
            found = true;
            best_cost = cost;
            best_tgo = cand;
        }
    }
    return found ? std::optional<double>(best_tgo) : std::nullopt;
}
// END CANONICAL ALGORITHM: powered landing bounded time-to-go selection

LandingCommand landing_guidance_command(const BinarySystem& bin,
                                        const Config& config,
                                        const LandingConfig& landing, double now,
                                        double t_go_estimate, int target_body,
                                        LandingPhase phase,
                                        const ZeroEffortQuery& zero_effort) {
    LandingCommand cmd{};
    const std::optional<double> tgo = landing_select_time_to_go(
        bin, config, landing, now, t_go_estimate, target_body, phase, zero_effort);
    if (!tgo) {
        return cmd;
    }
    const double cand = *tgo;
    const BallisticState zero = zero_effort(cand);
    const ZemZev zz =
        compute_zem_zev(bin, target_body, phase, landing, now, cand, zero);
    cmd.t_go = cand;
    // The Apollo-style command is already an inertial/world-frame total
    // acceleration; drive the engine directly along it.
    cmd.acceleration =
        landing_polynomial_command(zz.zem, zz.zev, cand, landing.guidance_interval);
    cmd.throttle =
        std::clamp(vec_len(cmd.acceleration) / config.main_accel, 0.0, 1.0);
    cmd.valid = true;
    return cmd;
}

// M06-R7: pure feasibility preview for a hypothetical Descent handoff. This is
// intentionally outside the canonical marked regions. It reuses the canonical
// bounded t_go selector, Apollo profile, and polynomial command machinery, then
// reports the bounded metrics used by the high-energy terminal handoff gate.
LandingTerminalPreview landing_terminal_preview(
    const BinarySystem& bin, const Config& config, const LandingConfig& landing,
    double now, double t_go_estimate, int target_body,
    const ZeroEffortQuery& zero_effort) {
    LandingTerminalPreview p{};
    if (!zero_effort) {
        return p;
    }

    const std::optional<double> tgo = landing_select_time_to_go(
        bin, config, landing, now, t_go_estimate, target_body,
        LandingPhase::Descent, zero_effort);
    if (!tgo) {
        return p;
    }
    p.t_go = *tgo;

    const BallisticState zero = zero_effort(*tgo);
    const ZemZev zz =
        compute_zem_zev(bin, target_body, LandingPhase::Descent, landing, now,
                        *tgo, zero);
    const AccelProfile profile =
        landing_acceleration_profile(zz.zem, zz.zev, *tgo);
    const Vec2 a_term{profile.a0.x + profile.a1.x * *tgo,
                      profile.a0.y + profile.a1.y * *tgo};
    p.peak_accel = std::max(vec_len(profile.a0), vec_len(a_term));
    p.acceleration =
        landing_polynomial_command(zz.zem, zz.zev, *tgo, landing.guidance_interval);

    Vec2 c0 = bin.position(target_body, now);
    const LandingTargetState t0 =
        landing_target_state(bin, target_body, LandingPhase::Descent, landing,
                             now, 0.0);
    const Vec2 o{t0.position.x - c0.x, t0.position.y - c0.y};
    const double ol = vec_len(o);
    if (ol > 1e-9) {
        p.initial_radial =
            (profile.a0.x * o.x + profile.a0.y * o.y) / ol;
    }

    p.command_valid =
        std::isfinite(p.t_go) && std::isfinite(p.peak_accel) &&
        std::isfinite(p.acceleration.x) && std::isfinite(p.acceleration.y) &&
        std::isfinite(p.initial_radial) &&
        vec_len(p.acceleration) <= config.main_accel + 1e-9;

    p.feasible =
        p.command_valid &&
        p.t_go <= landing.terminal_handoff_t_go_max + 1e-9 &&
        p.peak_accel <=
            landing.terminal_handoff_peak_factor * config.main_accel;
    return p;
}

// LandingAutopilot ----------------------------------------------------------------

namespace {

double wrap_pi(double a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a < 0.0) {
        a += kTwoPi;
    }
    return a - kPi;
}

// Nose angle whose thrust_hat points along `d` (thrust_hat(t) = {-sin t, cos t}).
double nose_angle_for(const Vec2& d) {
    return std::atan2(-d.x, d.y);
}

// Wrapped body-frame angle error of the ship relative to the selected pad
// (+ means the ship is ahead of the pad in the co-rotation direction). Used to
// fold the along-track offset into the high-energy tangential *velocity* target
// rather than adding a separate position acceleration, which would fight the
// velocity PD when the craft is offset.
double pad_angle_error(const State& s, const BinarySystem& bin, int body,
                       double now) {
    const Vec2 c = bin.position(body, now);
    const double world = std::atan2(s.y - c.y, s.x - c.x);
    const double ship_body = world - bin.body_rotation(body, now);
    return wrap_pi(ship_body - pad_local_angle(bin.body(body)));
}

}  // namespace

void LandingAutopilot::arm(int target_body, const LandingConfig& config,
                           const ZeroEffortQuery& zero_effort) {
    armed_ = true;
    terminal_ = false;
    target_body_ = target_body;
    config_ = config;
    zero_effort_ = zero_effort;
    phase_ = LandingPhase::Approach;
    t_go_estimate_ = 0.5 * (config_.t_go_min + config_.t_go_max);
    last_guidance_time_ = -1e300;
    command_ = {};
    status_ = {};
    status_.armed = true;
    status_.phase = phase_;
    // V14: reset the routing / cross-body transfer state.
    source_body_ = -1;
    routed_ = false;
    target_disturbed_ = false;
    transfer_mc_ = {};
    transfer_sol_ = {};
    transfer_launched_ = false;
    last_launch_try_ = -1e300;
    high_energy_ = false;
    brake_handoff_ready_time_ = -1e300;
    terminal_preview_ = {};
    last_terminal_preview_time_ = -1e300;
}

void LandingAutopilot::abort() {
    if (!armed_) {
        return;
    }
    armed_ = false;
    terminal_ = false;
    command_ = {};
    status_ = {};
    transfer_mc_.abort();
    terminal_preview_ = {};
    last_terminal_preview_time_ = -1e300;
}

Input LandingAutopilot::make_input(const State& state, const Config& config,
                                    double now) const {
    if (!armed_ || terminal_) {
        return {};
    }
    // V14: the inter-moon transfer is driven by the two-level midcourse's HOT
    // VGO (a per-step, state-dependent Input that cannot be held), so it is the
    // one phase that bypasses the held-command path.
    if (phase_ == LandingPhase::Transfer) {
        return transfer_mc_.make_input(state, now, config, false, false);
    }
    Input input{};
    if (!command_.valid) {
        return input;
    }
    const double mag = vec_len(command_.acceleration);
    if (mag < 1e-9) {
        return input;  // No thrust requested: stay completely quiet.
    }
    const Vec2 dir{command_.acceleration.x / mag, command_.acceleration.y / mag};
    // Route the commanded direction through the ordinary bang-bang attitude
    // controller so the craft physically rotates to apply the thrust (R6-04).
    // This is shared by the ZEM/ZEV terminal flow (Approach/Descent) and the
    // V14 capture/ascend gravity-feedforward commands.
    input = attitude_input(state, config, std::optional<Vec2>(dir), false, false);
    // Apply main throttle only while the nose is within the thrust-alignment
    // limit of the commanded direction; otherwise hold attitude and coast
    // (no significant thrust while angular error is unsafe, R6-04).
    const double err = wrap_pi(nose_angle_for(dir) - state.angle);
    if (std::abs(err) <= config_.angle_thrust_limit) {
        input.main_throttle = command_.throttle;
    } else {
        input.main_throttle = 0.0;
    }
    return input;
}

void LandingAutopilot::update_t_go_estimate(const State& state,
                                            const BinarySystem& bin,
                                            const Config& config, double now) {
    (void)config;
    // Estimate the terminal time-to-go from the pad-relative radial state.
    // The estimate is a bias for the bounded t_go scan, not a hard schedule:
    // one local free-fall time plus t_go_min, clamped to the admissible
    // window. A short estimate yields a dive-and-brake profile whose initial
    // command points radially inward (a 180-degree nose reversal); a longer
    // estimate yields an outward, gravity-compensating command that the
    // existing attitude controller can execute without a full reversal, so
    // both phases bias toward the longer end of the window.
    const Body& body = bin.body(target_body_);
    const Vec2 body_pos = bin.position(target_body_, now);
    const auto pad = bin.surface_point(
        target_body_, pad_local_angle(body), pad_surface_radius(body), now);
    const Vec2 up_world{pad.position.x - body_pos.x, pad.position.y - body_pos.y};
    const double up_len = vec_len(up_world);
    Vec2 up{1.0, 0.0};
    if (up_len > 1e-9) {
        up = {up_world.x / up_len, up_world.y / up_len};
    }
    const Vec2 rel_p{state.x - pad.position.x, state.y - pad.position.y};
    const double altitude = rel_p.x * up.x + rel_p.y * up.y;
    const double dist = std::hypot(state.x - body_pos.x, state.y - body_pos.y);
    const double g = body.mu / (dist * dist);
    // Pad-relative radial velocity in the co-rotating pad frame: positive is
    // climbing away from the pad, negative is descending toward it.
    const Vec2 rel_v{state.vx - pad.velocity.x, state.vy - pad.velocity.y};
    const double vr = rel_v.x * up.x + rel_v.y * up.y;
    // The estimate is a bias for the bounded t_go scan, not a hard schedule.
    // Use the no-thrust time-to-pad evaluated with the *current* radial
    // velocity rather than the from-rest free-fall time sqrt(2h/g): a craft
    // already descending (vr < 0) reaches the pad sooner, so centreing the
    // scan on that actual descent keeps the selected t_go short enough to
    // commit instead of stalling into a long hover -- the from-rest bias made
    // the low-gravity companion hover-lock. Adding t_go_min leaves a braking
    // margin, and the clamp keeps it inside the admissible window.
    const double disc = vr * vr + 2.0 * g * std::max(0.0, altitude);
    const double t_ballistic =
        (g > 1e-9 && disc > 0.0) ? (vr + std::sqrt(disc)) / g : 0.0;
    // A high-energy hand-off can arrive with a larger waypoint-relative state
    // error than a clean local approach. Biasing the estimate toward the short
    // end of the window makes the bounded scan prefer the more aggressive
    // candidates that actually commit to the descent instead of stalling into
    // a long hover.
    const double ballistic_bias = high_energy_ ? 0.2 : 1.0;
    const double raw_estimate =
        ballistic_bias * t_ballistic + config_.t_go_min;
    t_go_estimate_ =
        std::clamp(raw_estimate, config_.t_go_min, config_.t_go_max);
}

void LandingAutopilot::guidance_update(const State& state, const BinarySystem& bin,
                                        const Config& config, double now) {
    update_t_go_estimate(state, bin, config, now);
    command_ = landing_guidance_command(bin, config, config_, now, t_go_estimate_,
                                        target_body_, phase_, zero_effort_);
}

void LandingAutopilot::terminal_preview_update(const State& state,
                                               const BinarySystem& bin,
                                               const Config& config, double now) {
    if (!zero_effort_) {
        return;
    }
    update_t_go_estimate(state, bin, config, now);
    terminal_preview_ = landing_terminal_preview(
        bin, config, config_, now, t_go_estimate_, target_body_, zero_effort_);
    last_terminal_preview_time_ = now;
}

bool LandingAutopilot::terminal_handoff_ok(const State& state,
                                           const BinarySystem& bin, double now) const {
    if (!high_energy_) {
        return true;
    }
    const bool preview_ok =
        terminal_preview_.command_valid && terminal_preview_.feasible;
    if (!preview_ok) {
        return false;
    }

    const Body& tb = bin.body(target_body_);
    const Vec2 bpos = bin.position(target_body_, now);
    const double r = std::hypot(state.x - bpos.x, state.y - bpos.y);
    if (r <= 1e-9) {
        return false;
    }
    const Vec2 up{(state.x - bpos.x) / r, (state.y - bpos.y) / r};
    const Vec2 rt{-up.y, up.x};
    const auto sp =
        bin.surface_point(target_body_, pad_local_angle(tb),
                          pad_surface_radius(tb), now);
    const Vec2 relv{state.vx - sp.velocity.x, state.vy - sp.velocity.y};
    const double vr = relv.x * up.x + relv.y * up.y;
    const double vt = relv.x * rt.x + relv.y * rt.y;
    const auto wp = bin.surface_point(
        target_body_, pad_local_angle(tb),
        pad_surface_radius(tb) + config_.hover_height, now);
    const Vec2 ep{wp.position.x - state.x, wp.position.y - state.y};
    const double lat = ep.x * rt.x + ep.y * rt.y;
    const double alt = r - pad_surface_radius(tb);

    const bool alt_ok = alt <= config_.brake_handoff_alt;
    const bool radial_ok =
        vr <= -config_.terminal_handoff_radial_min &&
        vr >= -config_.terminal_handoff_radial_max;
    const bool tang_ok =
        std::abs(vt) <= config_.terminal_handoff_tangential;
    const bool lat_ok = std::abs(lat) <= config_.terminal_handoff_lateral;
    // The preview command's nose alignment is intentionally NOT part of this
    // gate: the phase transition itself already requires the *held* shaping /
    // approach command to be within the thrust-alignment limit, and the
    // Descent command is only throttled after the ordinary bang-bang
    // controller aligns the nose to it. Requiring the hypothetical preview
    // direction to be pre-aligned would keep a high-energy arrival in the
    // shaping phase even when the state and bounded terminal profile are ready.
    return alt_ok && radial_ok && tang_ok && lat_ok;
}

void LandingAutopilot::update_phase(const State& before, const State& after,
                                    const BinarySystem& bin, const Config& config,
                                    double now) {
    (void)before;
    (void)config;
    if (phase_ == LandingPhase::Approach) {
        const auto hover = landing_target_state(
            bin, target_body_, LandingPhase::Approach, config_, now, 0.0);
        const double range =
            std::hypot(hover.position.x - after.x, hover.position.y - after.y);
        const double v_rel =
            std::hypot(after.vx - hover.velocity.x, after.vy - hover.velocity.y);
        // DESCENT is entered only when the held approach command is physically
        // aligned with the current nose; otherwise the craft keeps approaching
        // and rotating until the attitude controller brings it within the
        // thrust-alignment limit (R6-07, attitude feasibility).
        bool attitude_ok = false;
        const double mag = vec_len(command_.acceleration);
        if (command_.valid && mag > 1e-9) {
            const Vec2 dir{command_.acceleration.x / mag,
                           command_.acceleration.y / mag};
            const double err = wrap_pi(nose_angle_for(dir) - after.angle);
            attitude_ok = std::abs(err) <= config_.angle_thrust_limit;
        }
        // For a high-energy arrival the terminal Descent target is the pad
        // itself, and its short t_go candidates are rejected when the initial
        // Apollo acceleration points inward. Keep the craft in Approach until
        // the pad-relative radial velocity has been slowed enough that those
        // short candidates can point outward and commit the descent.
        bool high_energy_radial_ok = true;
        if (high_energy_ && config_.high_energy_radial_max < 0.0) {
            const Body& tb = bin.body(target_body_);
            const Vec2 bpos = bin.position(target_body_, now);
            const double r = std::hypot(after.x - bpos.x, after.y - bpos.y);
            const Vec2 up{r > 1e-9 ? (after.x - bpos.x) / r : 1.0,
                          r > 1e-9 ? (after.y - bpos.y) / r : 0.0};
            const auto sp = bin.surface_point(
                target_body_, pad_local_angle(tb), pad_surface_radius(tb), now);
            const Vec2 relv{after.vx - sp.velocity.x, after.vy - sp.velocity.y};
            const double vr = relv.x * up.x + relv.y * up.y;
            high_energy_radial_ok = vr >= config_.high_energy_radial_max;
        }
        // M06-R7: a high-energy front end may enter the terminal Descent law
        // only when the low-rate hypothetical Descent preview and the current
        // pad-relative state are both inside the bounded descent corridor.
        // Low-energy arrivals keep the existing Approach -> Descent rule.
        const bool terminal_ok = terminal_handoff_ok(after, bin, now);
        if (range <= config_.approach_radius && v_rel <= config_.approach_speed &&
            attitude_ok && high_energy_radial_ok && terminal_ok) {
            phase_ = LandingPhase::Descent;
        }
    }
}

// V14 (M06-R6-08/09/10): route to the correct phase for the source/target
// geometry. arm() has no live state, so the routing runs on the first step.
// Same-body targets pick CAPTURE when the target body is disturbed by the
// other body's gravity (a pure ZEM/ZEV command cannot hold a hover against it)
// and the existing APPROACH flow when it is not. Cross-body starts in ASCEND.
void LandingAutopilot::route(const State& before, const BinarySystem& bin,
                             double now) {
    // M06-R13: the source is the nearest of the three bodies (two-body
    // behaviour is unchanged: near the primary / companion region the nearest
    // is 0 / 1; only near the moonlet does it become 2).
    double best_d = 1e30;
    int src = 0;
    for (int b = 0; b < 3; ++b) {
        const Vec2 cb = bin.position(b, now);
        const double d = std::hypot(before.x - cb.x, before.y - cb.y);
        if (d < best_d) {
            best_d = d;
            src = b;
        }
    }
    source_body_ = src;

    const Body& target = bin.body(target_body_);
    const Vec2 pad = bin.surface_point(
        target_body_, pad_local_angle(target), pad_surface_radius(target), now)
                        .position;
    const double g_target = vec_len(bin.gravity_from(target_body_, pad, now));
    // M06-R13: the disturbing field at the pad is the vector sum of every
    // body other than the target. With two bodies this is exactly the old
    // single other-body term, so R12 behaviour is preserved.
    Vec2 g_other_sum{};
    for (int b = 0; b < 3; ++b) {
        if (b == target_body_) {
            continue;
        }
        const Vec2 g = bin.gravity_from(b, pad, now);
        g_other_sum.x += g.x;
        g_other_sum.y += g.y;
    }
    const double g_other = vec_len(g_other_sum);
    const double ratio = g_target > 1e-9 ? g_other / g_target : 0.0;
    target_disturbed_ = ratio > config_.disturbance_ratio;

    if (source_body_ == target_body_) {
        // Same-body start: never a cross-body high-energy arrival, so the VGO
        // deorbit/brake flow does not apply. A disturbed local start begins at
        // the capture entry; an undisturbed one at the pad.
        high_energy_ = false;
        phase_ = target_disturbed_ ? LandingPhase::Capture
                                   : LandingPhase::Approach;
    } else {
        phase_ = LandingPhase::Ascend;
    }
    // Latch so routing runs exactly once (on the first step): re-running it
    // every tick would reset the phase machine and undo the Approach->Descent
    // transition the terminal guidance depends on.
    routed_ = true;
}

// V14: a physical gravity-feedforward command that cancels the full two-body
// field at the ship and drives a bounded-rate velocity/position correction
// toward a co-rotating waypoint. `climb` selects the launch waypoint (ascend)
// over the source pad versus the pad itself with a slow descent (capture).
// There is no second gravity term in the ZEM/ZEV sense: the single -g here is
// the cancellation that makes a hover/descent possible against the net field.
LandingCommand LandingAutopilot::capture_command(const State& state,
                                                 const BinarySystem& bin,
                                                 const Config& config, double now,
                                                 bool climb) {
    LandingCommand cmd{};
    const int frame = (climb && source_body_ >= 0) ? source_body_
                                                   : target_body_;
    const Body& body = bin.body(frame);
    const double pad_angle = pad_local_angle(body);
    const double pad_radius = pad_surface_radius(body);
    const Vec2 center = bin.position(frame, now);

    Vec2 up{0.0, 1.0};
    const Vec2 to_ship{state.x - center.x, state.y - center.y};
    const double ts = vec_len(to_ship);
    if (ts > 1e-9) {
        up = {to_ship.x / ts, to_ship.y / ts};
    }

    // HIGH-ENERGY capture (V14 C): a grossly hyperbolic cross-body arrival
    // (|v_rel| above high_energy_factor * local circular at capture entry).
    // Remove the gross energy with a target-body-centred VGO that cancels the
    // net two-body field and drives the target-relative velocity to a bounded
    // local-circular state: v_r -> 0 stops the dive, v_t -> v_circ settles into
    // a bound orbit while preserving the direction of motion. No pad or
    // waypoint chasing, and no ZEM/ZEV yet -- that is the deorbit/brake/
    // terminal flow that after_step runs next.
    if (!climb && high_energy_) {
        const double v_circ = std::sqrt(body.mu / std::max(1e-9, ts));
        const RTFrame f = target_rt_frame(state, bin, frame, now);
        const double v_t_des = (f.v_t >= 0.0) ? v_circ : -v_circ;
        const Vec2 a = vgo_accel(state, bin, frame, now, 0.0, v_t_des,
                                 config_.capture_time);
    cmd.acceleration = a;
    cmd.t_go = 0.0;
    cmd.throttle = std::clamp(vec_len(a) / config.main_accel, 0.0, 1.0);
    cmd.valid = true;
    return cmd;
}

    // ASCEND / LOW-ENERGY CAPTURE: a gentle gravity-feedforward PD to a
    // co-rotating waypoint that cancels the net two-body field. Ascend hovers
    // at the launch waypoint above the source pad; a local capture converges
    // onto the pad with a bounded descent rate that ramps to a hover at the
    // surface, so the ship is slowed well before the last few metres (the
    // existing V14-A/B behaviour, preserved).
    const double wp_height =
        climb ? config_.transfer_launch_altitude : 0.0;
    const auto wp =
        bin.surface_point(frame, pad_angle, pad_radius + wp_height, now);
    double rate = 0.0;
    if (!climb) {
        const double scale =
            std::clamp((ts - pad_radius) / std::max(1e-9, config_.hover_height),
                       0.0, 1.0);
        rate = config_.capture_descent_speed * scale;
    }
    const Vec2 target_vel{wp.velocity.x - rate * up.x,
                          wp.velocity.y - rate * up.y};
    const Vec2 g = bin.gravity({state.x, state.y}, now);
    const double k_d = 1.0 / std::max(1e-9, config_.capture_vel_time);
    const double k_p = 1.0 / std::max(1e-9, config_.capture_pos_time);
    const Vec2 err_v{target_vel.x - state.vx, target_vel.y - state.vy};
    const Vec2 err_p{wp.position.x - state.x, wp.position.y - state.y};
    const Vec2 a_cmd{-g.x + k_d * err_v.x + k_p * err_p.x,
                     -g.y + k_d * err_v.y + k_p * err_p.y};
    cmd.acceleration = a_cmd;
    cmd.t_go = 0.0;
    cmd.throttle = std::clamp(vec_len(a_cmd) / config.main_accel, 0.0, 1.0);
    cmd.valid = true;
    return cmd;
}

// V14: deorbit command (M06-R6 high-energy). A bounded VGO in the target-body
// radial frame. The tangential base is the target body's own co-rotation speed
// at the craft radius (omega * r): matching that angular rate holds the pad
// angle instead of letting the craft drift behind it, which a local-circular
// target does while r is above the circular/co-rotation crossover. A damped,
// bounded velocity offset closes the residual along-track angle error. The
// radial target is a constant inward descent.
LandingCommand LandingAutopilot::deorbit_command(const State& state,
                                                const BinarySystem& bin,
                                                const Config& config, double now) {
    LandingCommand cmd{};
    const RTFrame f = target_rt_frame(state, bin, target_body_, now);
    const double v_rot_local = bin.body_spin_rate(target_body_) * f.r;
    const double dphi = pad_angle_error(state, bin, target_body_, now);
    const double align = -config_.deorbit_align_gain * f.r * dphi;
    const double damp = -config_.deorbit_align_damp * (f.v_t - v_rot_local);
    const double offset =
        std::clamp(align + damp, -config_.deorbit_align_max,
                   config_.deorbit_align_max);
    const double v_t_des = v_rot_local + offset;
    const Vec2 a = vgo_accel(state, bin, target_body_, now,
                             -config_.deorbit_descent, v_t_des,
                             config_.deorbit_time);
    cmd.acceleration = a;
    cmd.t_go = 0.0;
    cmd.throttle = std::clamp(vec_len(a) / config.main_accel, 0.0, 1.0);
    cmd.valid = true;
    return cmd;
}

// V14: brake command (M06-R6 high-energy). A tighter VGO frame match to the
// co-rotating hover waypoint. The waypoint's own velocity (projected into the
// craft's radial frame) already carries the small radial/tangential component
// that steers a laterally-offset craft back onto the pad, so the alignment PD
// only needs to settle the residual angular rate. The radial target is a
// bounded descent that fades to a small non-zero rate at the hover altitude,
// leaving the terminal ZEM/ZEV law a genuine descent to commit.
LandingCommand LandingAutopilot::brake_command(const State& state,
                                                const BinarySystem& bin,
                                                const Config& config, double now) {
    LandingCommand cmd{};
    const Body& body = bin.body(target_body_);
    const double pad_radius = pad_surface_radius(body);
    const Vec2 center = bin.position(target_body_, now);
    const double ts = std::hypot(state.x - center.x, state.y - center.y);
    const double alt = ts - pad_radius;
    const auto wp = bin.surface_point(
        target_body_, pad_local_angle(body),
        pad_radius + config_.hover_height, now);
    const auto pad_sp = bin.surface_point(
        target_body_, pad_local_angle(body), pad_radius, now);
    const RTFrame f = target_rt_frame(state, bin, target_body_, now);
    const Vec2 cv = bin.velocity(target_body_, now);
    const Vec2 wpv{wp.velocity.x - cv.x, wp.velocity.y - cv.y};
    const Vec2 padv{pad_sp.velocity.x - cv.x, pad_sp.velocity.y - cv.y};
    const double v_r_base = wpv.x * f.r_hat.x + wpv.y * f.r_hat.y;
    const double wpv_t = wpv.x * f.t_hat.x + wpv.y * f.t_hat.y;
    const double padv_t = padv.x * f.t_hat.x + padv.y * f.t_hat.y;
    const double vrel = std::hypot(state.vx - cv.x, state.vy - cv.y);
    // Once the craft is low and slow, the angular PD that keeps it over the
    // hover waypoint forces a co-rotating tangential speed that the terminal
    // Descent law then has to undo. Switch to a pure pad-relative velocity kill
    // so the hand-off state has a small pad-relative tangential speed; the
    // terminal law can correct the remaining along-track offset.
    const bool kill =
        alt <= config_.brake_final_alt && vrel <= config_.brake_final_speed;
    double v_t_des;
    double v_r_des;
    double dphi = 0.0;
    double offset = 0.0;
    double descent_rate = 0.0;
    if (kill) {
        // The tangential setpoint is the co-rotating speed at the craft's own
        // radius (omega * altitude above the pad surface), which is the
        // pad-relative speed needed to stay over the pad.  Above the hover
        // altitude this keeps the craft in the target body's rotating frame
        // instead of forcing it down to the low hover speed; below the hover
        // altitude it fades to the small hand-off value so the terminal law
        // receives a manageable pad-relative tangential error.
        const double omega = bin.body_spin_rate(target_body_);
        const double alt_pos = std::max(0.0, alt);
        double target_rel;
        if (alt_pos >= config_.brake_kill_ramp_alt) {
            target_rel = omega * alt_pos;
        } else {
            const double span = std::max(
                1e-9, config_.brake_kill_ramp_alt - config_.brake_handoff_alt);
            const double frac = std::clamp(
                (config_.brake_kill_ramp_alt - alt_pos) / span, 0.0, 1.0);
            target_rel = (1.0 - frac) * omega * config_.brake_kill_ramp_alt +
                         frac * config_.brake_kill_tangential;
        }
        // A bounded angle-rate PD cancels the residual pad-angle error without
        // the saturated de-orbit burn that a full position PD produces from a
        // large cross-body arrival offset.  The position term schedules the
        // relative angular rate that would zero the current angle error in
        // `1 / brake_kill_align_gain` seconds; the rate term damps the actual
        // relative angular rate toward that schedule.
        dphi = pad_angle_error(state, bin, target_body_, now);
        const double T_align =
            1.0 / std::max(1e-9, config_.brake_kill_align_gain);
        const double actual_rel_rate =
            f.v_t / std::max(1e-9, f.r) - omega;
        const double desired_rel_rate = -dphi / T_align;
        offset = std::clamp(
            f.r * (desired_rel_rate - actual_rel_rate),
            -config_.brake_kill_align_max, config_.brake_kill_align_max);
        v_t_des = padv_t + target_rel + offset;
        double descent;
        if (alt_pos >= config_.brake_kill_ramp_alt) {
            descent = config_.brake_kill_descent;
        } else {
            const double span = std::max(
                1e-9, config_.brake_kill_ramp_alt - config_.brake_handoff_alt);
            const double frac = std::clamp(
                (config_.brake_kill_ramp_alt - alt_pos) / span, 0.0, 1.0);
            descent = (1.0 - frac) * config_.brake_kill_descent +
                      frac * config_.brake_kill_descent_min;
        }
        v_r_des = -descent;
    } else {
        const double v_t_base = (1.0 - config_.brake_tangential_blend) * padv_t +
                                config_.brake_tangential_blend * wpv_t;
        dphi = pad_angle_error(state, bin, target_body_, now);
        const double align = -config_.brake_align_gain * f.r * dphi;
        const double damp = -config_.brake_align_damp * (f.v_t - v_t_base);
        offset = std::clamp(align + damp, -config_.brake_align_max,
                            config_.brake_align_max);
        v_t_des = v_t_base + offset;
        const double fade_span = std::max(1e-9, 0.5 * config_.hover_height);
        const double descent_scale =
            std::clamp((alt - config_.hover_height) / fade_span, 0.0, 1.0);
        descent_rate = config_.brake_min_descent +
            (config_.brake_descent - config_.brake_min_descent) * descent_scale;
        v_r_des = v_r_base - descent_rate;
    }
    const Vec2 a = vgo_accel(state, bin, target_body_, now, v_r_des, v_t_des,
                             config_.brake_time);
    cmd.acceleration = a;
    cmd.t_go = 0.0;
    cmd.throttle = std::clamp(vec_len(a) / config.main_accel, 0.0, 1.0);
    cmd.valid = true;
    return cmd;
}

// V14 (M06-R6-10): climb to the launch waypoint over the source body, then run
// the COLD transfer solve at a bounded rate and arm the two-level midcourse.
void LandingAutopilot::update_ascend(const State& before, const State& after,
                                     const BinarySystem& bin, const Config& config,
                                     double now) {
    (void)before;
    command_ = capture_command(after, bin, config, now, /*climb=*/true);

    const Vec2 center = bin.position(source_body_, now);
    const Vec2 to_ship{after.x - center.x, after.y - center.y};
    const double ts = vec_len(to_ship);
    const Vec2 up{ts > 1e-9 ? to_ship.x / ts : 0.0,
                  ts > 1e-9 ? to_ship.y / ts : 0.0};
    const double alt = to_ship.x * up.x + to_ship.y * up.y;

    if (alt >= config_.transfer_launch_altitude &&
        (last_launch_try_ < -1e299 ||
         now - last_launch_try_ >= config_.transfer_launch_interval)) {
        last_launch_try_ = now;
        ManeuverNode node{};
        node.time = now;
        node.frame_body = source_body_;
        const auto planned =
            plan_transfer(bin, config, after, now, source_body_, node,
                          &transfer_sol_);
        if (planned) {
            const NodeBasis basis = compute_node_basis(
                bin, now, source_body_, {after.x, after.y},
                {after.vx, after.vy});
            transfer_mc_.arm(*planned, transfer_sol_, source_body_, basis, now,
                             config);
            transfer_launched_ = true;
            phase_ = LandingPhase::Transfer;
        }
    }
}

// V14 (M06-R6-10): drive the two-level transfer midcourse (HOT VGO every step,
// WARM bounded-rate replan), then hand off to capture once the arc brings the
// ship inside the target's capture shell.
void LandingAutopilot::update_transfer(const State& before, const State& after,
                                       const BinarySystem& bin, const Config& config,
                                       double now) {
    const double now_pre = now - config.fixed_dt;
    // Reconstruct the Input the const make_input emitted this tick (same state
    // and pre-step time), advance the fast executor, then run the bounded-rate
    // WARM replan (one step of lag versus the GUI's pre-step replan).
    const Input applied =
        transfer_mc_.make_input(before, now_pre, config, false, false);
    transfer_mc_.after_step(before, after, bin, applied, now, config);
    transfer_mc_.maybe_replan(bin, config, before, now_pre,
                              config_.transfer_replan_interval,
                              config_.transfer_miss_tolerance);

    // Auto capture shell: a high-energy cross-body arrival flies by with a
    // pericenter well above the target's reference radius, so the shell must be
    // wide enough that the inbound leg intersects it before closest approach.
    // 6x the reference radius covers the V14 primary->companion transfer
    // (pericenter ~5.2x the companion reference radius) while staying local.
    const double cap_radius =
        config_.transfer_capture_radius > 0.0
            ? config_.transfer_capture_radius
            : 6.0 * std::max(
                  1.0, bin.body(target_body_).reference_radius);
    const Vec2 tc = bin.position(target_body_, now);
    const double d_target = std::hypot(after.x - tc.x, after.y - tc.y);
    if (d_target <= cap_radius) {
        transfer_mc_.abort();
        transfer_launched_ = false;
        if (target_disturbed_) {
            // Classify the arrival against the local circular speed: a grossly
            // hyperbolic speed is a HIGH-ENERGY arrival (runs the VGO
            // deorbit/brake flow), otherwise a low-energy arrival lands directly
            // from capture (the existing V14-A/B behaviour, preserved).
            const Body& tb = bin.body(target_body_);
            const double v_circ = std::sqrt(tb.mu / std::max(1e-9, d_target));
            const Vec2 bvel = bin.velocity(target_body_, now);
            const double v_rel =
                std::hypot(after.vx - bvel.x, after.vy - bvel.y);
            high_energy_ = v_rel > config_.high_energy_factor * v_circ;
            phase_ = LandingPhase::Capture;
            command_ = capture_command(after, bin, config, now, /*climb=*/false);
        } else {
            high_energy_ = false;
            phase_ = LandingPhase::Approach;
            command_ = {};
        }
    }
}

void LandingAutopilot::after_step(const State& before, const State& after,
                                   const BinarySystem& bin, const Config& config,
                                   double now) {
    if (!armed_) {
        return;
    }
    if (after.landed || after.crashed) {
        terminal_ = true;
        phase_ = LandingPhase::Touchdown;
        command_ = {};
        transfer_mc_.abort();
        status_.phase = phase_;
        status_.output_active = false;
        status_.terminal = true;
        status_.t_go = 0.0;
        status_.command_accel = 0.0;
        return;
    }
    if (!routed_) {
        route(before, bin, now);
    }
    if (phase_ == LandingPhase::Ascend) {
        update_ascend(before, after, bin, config, now);
    } else if (phase_ == LandingPhase::Transfer) {
        update_transfer(before, after, bin, config, now);
    } else if (phase_ == LandingPhase::Capture) {
        command_ = capture_command(after, bin, config, now, /*climb=*/false);
        // HIGH-ENERGY: once the capture VGO has bled off the gross flyby energy
        // (target-relative speed down near the local circular speed), begin the
        // controlled deorbit. A low-energy local arrival never meets this gate,
        // so it keeps landing directly from Capture (the existing V14-B flow).
        if (high_energy_) {
            const Body& tb = bin.body(target_body_);
            const Vec2 bpos = bin.position(target_body_, now);
            const double r = std::hypot(after.x - bpos.x, after.y - bpos.y);
            const double v_circ = std::sqrt(tb.mu / std::max(1e-9, r));
            const Vec2 bvel = bin.velocity(target_body_, now);
            const double v_rel = std::hypot(after.vx - bvel.x, after.vy - bvel.y);
            if (v_rel < config_.deorbit_entry_factor * v_circ) {
                phase_ = LandingPhase::Deorbit;
                command_ = {};
            }
        }
    } else if (phase_ == LandingPhase::Deorbit) {
        // Use the same bounded local velocity-kill as the brake phase: the
        // high-energy arrival only needs its target-body-relative velocity
        // bled off and a pad-aligned descent established, not a full orbital
        // de-orbit burn.
        command_ = brake_command(after, bin, config, now);
        const Body& tb = bin.body(target_body_);
        const Vec2 bpos = bin.position(target_body_, now);
        const double r = std::hypot(after.x - bpos.x, after.y - bpos.y);
        const double alt = r - pad_surface_radius(tb);
        // The deorbit runs straight down to the fixed brake band; it does not
        // bounce back to capture. (The deorbit VGO drives the tangential speed
        // toward the local circular regime and the radial speed to a bounded
        // inward descent, so the combined target-relative speed legitimately
        // stays near ~v_circ during the descent; an exit gate on that total
        // would thrash.) Once low enough, hand off to the pre-approach frame
        // match.
        if (alt <= config_.brake_alt_in) {
            phase_ = LandingPhase::Brake;
            command_ = {};
        }
    } else if (phase_ == LandingPhase::Brake) {
        command_ = brake_command(after, bin, config, now);
        const Body& tb = bin.body(target_body_);
        const Vec2 bpos = bin.position(target_body_, now);
        const double r = std::hypot(after.x - bpos.x, after.y - bpos.y);
        const double alt = r - pad_surface_radius(tb);
        // Hand-off state: the pad-relative radial/tangential split, not the total
        // speed. The terminal ZEM/ZEV law commits cleanly from a moderate, mostly
        // radial descent over the pad; a large tangential error (even at a modest
        // total speed) makes the short t_go candidates over-thrust and drives the
        // bounded scan into a long hover-like candidate.
        const auto sp = bin.surface_point(
            target_body_, pad_local_angle(tb), pad_surface_radius(tb), now);
        const Vec2 up{r > 1e-9 ? (after.x - bpos.x) / r : 1.0,
                      r > 1e-9 ? (after.y - bpos.y) / r : 0.0};
        const Vec2 rt{-up.y, up.x};
        const Vec2 relv{after.vx - sp.velocity.x, after.vy - sp.velocity.y};
        const double vr2 = relv.x * up.x + relv.y * up.y;
        const double vt2 = relv.x * rt.x + relv.y * rt.y;
        const auto wp = bin.surface_point(
            target_body_, pad_local_angle(tb),
            pad_surface_radius(tb) + config_.hover_height, now);
        const Vec2 ep{wp.position.x - after.x, wp.position.y - after.y};
        const double ep_t = ep.x * rt.x + ep.y * rt.y;
        const bool radial_ok = vr2 <= -config_.brake_handoff_radial_min &&
                               vr2 >= -config_.brake_handoff_radial_max;
        // The terminal Descent target is the pad itself, so the hand-off state
        // must have a small pad-relative tangential speed. The brake's final
        // velocity kill drives exactly this component to zero.
        const double vt_ref = 0.0;
        const bool tang_ok =
            std::abs(vt2 - vt_ref) <= config_.brake_handoff_tangential;
        const bool lat_ok = std::abs(ep_t) <= config_.brake_handoff_lateral;
        const bool alt_ok = alt <= config_.brake_handoff_alt;
        // Hysteresis: if the craft climbs back out of the brake band, resume the
        // deorbit. Otherwise, once it is low enough for the terminal law's
        // time-to-go to stay short, the radial descent is in the hand-off window,
        // and the tangential error is small, hand off to the existing ZEM/ZEV
        // terminal law (Approach -> Descent -> Touchdown). The state gates above
        // are sufficient; the terminal law re-aligns through the ordinary
        // attitude controller, so no extra nose-alignment gate is applied here.
        if (alt > config_.brake_alt_out) {
            phase_ = LandingPhase::Deorbit;
            command_ = {};
        } else {
            // M06-R7: while the high-energy front end is still in Brake, keep a
            // low-rate hypothetical Descent preview current. The Brake ->
            // Approach transition itself is not a Descent handoff, so it is
            // gated only by the existing brake-shaped state corridor; the
            // terminal-feasibility gate is applied when Approach tries to enter
            // Descent.
            if (high_energy_ &&
                (last_terminal_preview_time_ < -1e299 ||
                 now - last_terminal_preview_time_ >= config_.guidance_interval)) {
                terminal_preview_update(after, bin, config, now);
            }
            const bool gates_ok =
                alt_ok && radial_ok && tang_ok && lat_ok;
            if (gates_ok) {
                if (brake_handoff_ready_time_ < -1e299) {
                    brake_handoff_ready_time_ = now;
                }
                const bool dwell_ok =
                    config_.brake_handoff_dwell <= 0.0 ||
                    now - brake_handoff_ready_time_ >=
                        config_.brake_handoff_dwell;
                if (dwell_ok) {
                    phase_ = LandingPhase::Approach;
                    command_ = {};
                    brake_handoff_ready_time_ = -1e300;
                }
            } else {
                brake_handoff_ready_time_ = -1e300;
            }
        }
    } else {
        // M06-R7: a high-energy front end that has reached the Approach
        // altitude keeps the existing low-rate hypothetical Descent preview
        // current and continues the bounded brake-shaped velocity kill until
        // the terminal handoff gate is satisfied.
        bool high_energy_shaping = false;
        if (high_energy_ && phase_ == LandingPhase::Approach) {
            if (last_terminal_preview_time_ < -1e299 ||
                now - last_terminal_preview_time_ >=
                    config_.guidance_interval) {
                terminal_preview_update(after, bin, config, now);
            }
            if (!terminal_handoff_ok(after, bin, now)) {
                command_ = brake_command(after, bin, config, now);
                high_energy_shaping = true;
            } else {
                // M06-R7 MVP: the bled-down high-energy front end has reached
                // the low-energy handoff envelope. Hand it to the existing
                // gentle capture-PD terminal (the V14-B flow) instead of the
                // canonical ZEM/ZEV Descent. Forcing the ZEM/ZEV terminal onto
                // the tiny companion after a high-energy arrival is deferred to
                // post-M06 subsystem hardening.
                high_energy_ = false;
                phase_ = LandingPhase::Capture;
                command_ = capture_command(after, bin, config, now,
                                           /*climb=*/false);
            }
        }
        if (!high_energy_shaping && phase_ != LandingPhase::Capture) {
            update_phase(before, after, bin, config, now);
            if (last_guidance_time_ < -1e299 ||
                now - last_guidance_time_ >= config_.guidance_interval) {
                last_guidance_time_ = now;
                guidance_update(after, bin, config, now);
            }
        }
    }

    if (phase_ == LandingPhase::Transfer) {
        status_.phase = phase_;
        status_.t_go = 0.0;
        status_.command_accel = 0.0;
        status_.output_active = transfer_mc_.active();
    } else {
        const double mag = vec_len(command_.acceleration);
        status_.phase = phase_;
        status_.t_go = command_.valid ? command_.t_go : 0.0;
        status_.command_accel = command_.valid ? mag : 0.0;
        status_.output_active = command_.valid && mag > 1e-9;
    }
}

}  // namespace lander
