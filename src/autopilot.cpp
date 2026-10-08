#include "lander/autopilot.hpp"

#include <algorithm>
#include <cmath>

namespace lander {

namespace {

double wrap_pi(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

double vec_length(const Vec2& v) { return std::hypot(v.x, v.y); }

Vec2 normalize_safe(const Vec2& v) {
    const double r = vec_length(v);
    if (r < 1.0e-12) {
        return {};
    }
    return {v.x / r, v.y / r};
}

// Continuous alignment safety (M06-R19): a burn is only held while the nose
// is within kAlignAngleBand of the current VGO direction and the spin stays
// below kAlignOmegaBand; otherwise the executor drops back to ALIGN.
constexpr double kAlignAngleBand = 0.05;
constexpr double kAlignOmegaBand = 0.1;
// The tracked VGO is in the "small vector" (flip-danger) regime when its
// magnitude is at most this many full-thrust steps. A single lagged,
// off-axis step is then a large fraction of the vector and can rotate or
// flip its direction by O(1) rad inside one step, beyond what the bang-bang
// can track (M06-R19: a step delivered against a ~1.5-step vector flipped
// it 1.57 rad and the burn chased the flip for the rest of the tank). Above
// it, the VGO direction is stable enough that the ordinary bang-bang
// tracking oscillation is safe while thrusting, so the continuous
// re-check uses the strict alignment band only in the small-vector regime.
constexpr double kSmallVgoSteps = 1.5;
constexpr double kContinuationAngleBand = 0.2;

// FLIGHT-COMPUTER TIER: HOT
// BEGIN CANONICAL ALGORITHM: bang-bang attitude control
// Reference:
// docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
int attitude_command(const State& state, const Config& config,
                     const std::optional<Vec2>& direction) {
    if (!direction || vec_length(*direction) < 1.0e-9) {
        return 0;
    }
    // thrust_hat(theta) = {-sin(theta), cos(theta)}, so the nose angle that
    // points along `d` is atan2(-d.x, d.y).
    const double desired = std::atan2(-(*direction).x, (*direction).y);
    const double error = wrap_pi(desired - state.angle);
    constexpr double kAngleDeadband = 0.01;
    constexpr double kRateDeadband = 0.02;

    if (std::abs(error) <= kAngleDeadband &&
        std::abs(state.omega) <= kRateDeadband) {
        return 0;
    }
    if (state.omega != 0.0) {
        if (std::abs(error) <= kAngleDeadband) {
            return -std::copysign(1.0, state.omega);
        }
        if (state.omega * error > 0.0 && config.rotate_accel > 0.0) {
            const double brake_distance =
                state.omega * state.omega / (2.0 * config.rotate_accel);
            if (brake_distance >= std::abs(error)) {
                return -std::copysign(1.0, state.omega);
            }
        }
    }
    return error > 0.0 ? 1 : -1;
}
// END CANONICAL ALGORITHM: bang-bang attitude control

}  // namespace

const char* attitude_mode_name(AttitudeMode mode) {
    switch (mode) {
        case AttitudeMode::Off: return "OFF";
        case AttitudeMode::Prograde: return "PROGRADE";
        case AttitudeMode::Retrograde: return "RETROGRADE";
        case AttitudeMode::RadialOut: return "RAD OUT";
        case AttitudeMode::RadialIn: return "RAD IN";
        case AttitudeMode::Target: return "TARGET";
        case AttitudeMode::AntiTarget: return "ANTI-TARGET";
        case AttitudeMode::Maneuver: return "MANEUVER";
    }
    return "OFF";
}

std::optional<Vec2> attitude_target_direction(
    AttitudeMode mode, const NodeBasis& basis, const Vec2& ship_position,
    const Vec2& target_position, const Vec2& maneuver_dv) {
    switch (mode) {
        case AttitudeMode::Off:
            return std::nullopt;
        case AttitudeMode::Prograde:
            return vec_length(basis.prograde) > 1.0e-9
                       ? std::optional<Vec2>(basis.prograde)
                       : std::nullopt;
        case AttitudeMode::Retrograde:
            return vec_length(basis.prograde) > 1.0e-9
                       ? std::optional<Vec2>(basis.prograde * -1.0)
                       : std::nullopt;
        case AttitudeMode::RadialOut:
            return vec_length(basis.radial_out) > 1.0e-9
                       ? std::optional<Vec2>(basis.radial_out)
                       : std::nullopt;
        case AttitudeMode::RadialIn:
            return vec_length(basis.radial_out) > 1.0e-9
                       ? std::optional<Vec2>(basis.radial_out * -1.0)
                       : std::nullopt;
        case AttitudeMode::Target: {
            const Vec2 d = target_position - ship_position;
            const Vec2 n = normalize_safe(d);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
        case AttitudeMode::AntiTarget: {
            const Vec2 d = ship_position - target_position;
            const Vec2 n = normalize_safe(d);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
        case AttitudeMode::Maneuver: {
            const Vec2 n = normalize_safe(maneuver_dv);
            return vec_length(n) > 1.0e-9 ? std::optional<Vec2>(n)
                                          : std::nullopt;
        }
    }
    return std::nullopt;
}

Input attitude_input(const State& state, const Config& config,
                     const std::optional<Vec2>& target_direction,
                     bool manual_left, bool manual_right) {
    Input input{};
    if (manual_left) {
        input.rotate_left = true;
    }
    if (manual_right) {
        input.rotate_right = true;
    }
    if (manual_left || manual_right) {
        return input;
    }
    const int command = attitude_command(state, config, target_direction);
    if (command > 0) {
        input.rotate_right = true;
    } else if (command < 0) {
        input.rotate_left = true;
    }
    return input;
}

const char* executor_state_name(ExecutorState state) {
    switch (state) {
        case ExecutorState::Idle: return "IDLE";
        case ExecutorState::Wait: return "WAIT";
        case ExecutorState::Align: return "ALIGN";
        case ExecutorState::Burn: return "BURN";
        case ExecutorState::Complete: return "COMPLETE";
        case ExecutorState::Aborted: return "ABORTED";
        case ExecutorState::Incomplete: return "INCOMPLETE";
    }
    return "IDLE";
}

// FLIGHT-COMPUTER TIER: HOT
// BEGIN CANONICAL ALGORITHM: velocity-to-be-gained node execution
// Reference:
// docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
void NodeExecutor::clear() {
    *this = NodeExecutor{};
}

void NodeExecutor::arm(const ManeuverNode& node, const NodeBasis& basis,
                       double now, const Config& config, bool direct_burn) {
    clear();
    frame_body_ = node.frame_body;
    node_time_ = node.time;
    dv_total_ = node_world_dv(node, basis);
    dv_remaining_ = dv_total_;

    if (config.main_accel <= 0.0 ||
        vec_length(dv_total_) <= 1.0e-9) {
        state_ = ExecutorState::Complete;
        return;
    }

    burn_time_ = vec_length(dv_total_) / config.main_accel;
    ignite_time_ = node_time_ - 0.5 * burn_time_;
    // Immediately after arming the executor begins aligning toward the VGO
    // with throttle 0, regardless of ignition time (M06-R4-03). Once aligned
    // it holds the VGO direction in WAIT until ignition; it never burns
    // before ignition. A late arm (now >= node_time) is exposed as LATE.
    // A continuation re-arm (midcourse re-target with the nose already near
    // the new VGO direction) resumes the correction burn immediately,
    // skipping the ALIGN swing. Only late nodes (time <= now) may direct-
    // burn, so the pre-ignition ALIGN/WAIT semantics (M06-R4-03) are
    // preserved for ordinary arms.
    if (direct_burn && now >= node_time_) {
        state_ = ExecutorState::Burn;
    } else {
        state_ = ExecutorState::Align;
    }
    late_ = now >= node_time_;
}

void NodeExecutor::abort() {
    if (active()) {
        state_ = ExecutorState::Aborted;
    }
}

bool NodeExecutor::aligned(const State& state) const {
    const Vec2 direction = normalize_safe(dv_remaining_);
    if (vec_length(direction) < 1.0e-9) {
        return true;
    }
    const double desired = std::atan2(-direction.x, direction.y);
    const double error = std::abs(wrap_pi(desired - state.angle));
    return error <= kAlignAngleBand && std::abs(state.omega) <= kAlignOmegaBand;
}

Input NodeExecutor::make_input(const State& state, double now,
                               const Config& config, bool manual_left,
                               bool manual_right) const {
    Input input{};
    if (!active()) {
        return input;
    }

    // ALIGN and the pre-ignition WAIT hold always steer toward the VGO with
    // throttle 0. The physical burn starts only at/after ignition and only
    // while aligned: we never force an off-axis burn just because the node
    // time passed. The first burning step must already carry the thrust
    // command, otherwise the finite burn would slip one fixed step.
    //
    // Continuous alignment safety is magnitude-gated (M06-R19): while the
    // burn holds, thrust requires the strict alignment band only in the
    // small-vector flip-danger regime (|VGO| <= kSmallVgoSteps full steps,
    // where one lagged off-axis step can flip the VGO direction); above it,
    // the bang-bang tracks the (slowly rotating) VGO direction while
    // thrusting, so the burn continues without a per-step re-check. In both
    // regimes a step that fails the required check delivers no impulse and
    // after_step re-enters ALIGN.
    const bool is_aligned = aligned(state);
    const double step_dv = config.main_accel * config.fixed_dt;
    const bool small_vgo =
        step_dv > 0.0 &&
        vec_length(dv_remaining_) <= kSmallVgoSteps * step_dv;
    const bool burning =
        (state_ == ExecutorState::Burn &&
         (!small_vgo || is_aligned)) ||
        (state_ == ExecutorState::Align && now >= ignite_time_ &&
         is_aligned) ||
        (state_ == ExecutorState::Wait && now >= ignite_time_ && is_aligned);

    if (state_ == ExecutorState::Align || state_ == ExecutorState::Wait ||
        state_ == ExecutorState::Burn) {
        const Vec2 direction = normalize_safe(dv_remaining_);
        input = attitude_input(state, config,
                               vec_length(direction) > 1.0e-9
                                   ? std::optional<Vec2>(direction)
                                   : std::nullopt,
                               manual_left, manual_right);
    }

    if (burning) {
        const double step_dv = config.main_accel * config.fixed_dt;
        if (step_dv > 0.0) {
            const double magnitude = vec_length(dv_remaining_);
            input.main_throttle = std::clamp(magnitude / step_dv, 0.0, 1.0);
        }
    }

    return input;
}

void NodeExecutor::after_step(const State& before, const State& after,
                              const Input& input, double now,
                              const Config& config) {
    if (!active()) {
        return;
    }

    if (after.crashed) {
        state_ = ExecutorState::Aborted;
        return;
    }

    if (state_ == ExecutorState::Align) {
        // Once aligned, hold the VGO direction: WAIT before ignition, BURN
        // at/after ignition. A still-misaligned ship stays in ALIGN.
        if (aligned(after)) {
            state_ = (now < ignite_time_) ? ExecutorState::Wait
                                          : ExecutorState::Burn;
        }
    } else if (state_ == ExecutorState::Wait) {
        // Aligned and holding before ignition; re-align if disturbed and
        // ignite once the ignition time is reached while still aligned.
        if (!aligned(after)) {
            state_ = ExecutorState::Align;
        } else if (now >= ignite_time_) {
            state_ = ExecutorState::Burn;
        }
    }

    if (state_ == ExecutorState::Burn) {
        const double throttle = std::clamp(input.main_throttle, 0.0, 1.0);
        if (before.fuel > 0.0 && throttle > 0.0) {
            const Vec2 thrust_hat{-std::sin(before.angle),
                                  std::cos(before.angle)};
            const double dv_step =
                config.main_accel * throttle * config.fixed_dt;
            dv_remaining_ = dv_remaining_ - thrust_hat * dv_step;
        }

        // The next `make_input` scales the final partial step, so the burn
        // is complete only once the tracked impulse has actually been spent.
        // The floor is one alignment-band width of a single thrust step
        // (main_accel * fixed_dt * kAlignAngleBand): a partial step of that
        // size or smaller can leave at most that much off-axis remainder
        // (band * magnitude), which no later aligned step can deliver, and
        // chasing it would only rotate the sub-floor residual out of the
        // band again and again (each re-alignment is a full attitude
        // rotation with the engine off). At the floor the remainder is
        // below the guidance's own angular resolution and is declared
        // delivered (M06-R19 follow-up).
        const double delivery_floor =
            config.main_accel * config.fixed_dt * kAlignAngleBand;
        if (vec_length(dv_remaining_) <=
            std::max(delivery_floor, 1.0e-9)) {
            state_ = ExecutorState::Complete;
            dv_remaining_ = {};
            return;
        }
        if (after.fuel <= 0.0) {
            state_ = ExecutorState::Incomplete;
            return;
        }
        // Continuous alignment safety (M06-R19), magnitude-gated: in the
        // small-vector flip-danger regime (|VGO| <= kSmallVgoSteps full
        // steps), a lagged off-axis impulse can rotate or flip the VGO
        // direction by O(1) rad inside a step, beyond what the bang-bang
        // tracks, so alignment is re-checked every step and a misaligned
        // state stops the physical thrust and returns to ALIGN (the Align
        // transition re-enters BURN once aligned; never back to WAIT after
        // ignition). Above the regime the VGO direction is stable, the
        // bang-bang tracks it while thrusting, and the burn holds without
        // the per-step re-check.
        const double gate_step_dv = config.main_accel * config.fixed_dt;
        if (gate_step_dv > 0.0 &&
            vec_length(dv_remaining_) <= kSmallVgoSteps * gate_step_dv &&
            !aligned(after)) {
            state_ = ExecutorState::Align;
        }
    }

    if (after.landed && active()) {
        state_ = ExecutorState::Aborted;
    }
}
// END CANONICAL ALGORITHM: velocity-to-be-gained node execution

// M06-R5-05 / D05: two-level inter-moon transfer midcourse controller.
//
// The fast NodeExecutor (HOT, every 1/120 s) is the per-step authority for
// attitude and the finite correction burns; the slow planner is plan_transfer,
// invoked at a bounded rate by maybe_replan (WARM). Only the slow path may
// reach into a solver; the fast path is a thin O(1) delegation, so the 120 Hz
// loop never blocks on a propagation.
// FLIGHT-COMPUTER TIER: WARM
// BEGIN CANONICAL ALGORITHM: bounded-rate warm transfer midcourse
// Reference:
// docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md
// docs/flight-guidance-computational-rate-tiers.md
void TransferMidcourse::arm(const ManeuverNode& node,
                            const TransferSolution& solution,
                            int reference_body, const NodeBasis& basis,
                            double now, const Config& config) {
    fast_.arm(node, basis, now, config);
    cache_ = solution;
    node_ = node;
    reference_body_ =
        reference_body < 0 ? 0 : (reference_body > 1 ? 1 : reference_body);
    engaged_ = true;
    last_replan_ = now;
    slow_plans_ = 0;
    retargets_ = 0;
}

void TransferMidcourse::abort() {
    engaged_ = false;
    fast_.abort();
}

// FLIGHT-COMPUTER TIER: HOT
Input TransferMidcourse::make_input(const State& state, double now,
                                    const Config& config, bool manual_left,
                                    bool manual_right) const {
    return fast_.make_input(state, now, config, manual_left, manual_right);
}

void TransferMidcourse::after_step(const State& before, const State& after,
                                   const Input& input, double now,
                                   const Config& config) {
    fast_.after_step(before, after, input, now, config);
    // The arc is done once the ship lands or crashes; stop re-aiming.
    if (after.landed || after.crashed) {
        engaged_ = false;
    }
}

bool TransferMidcourse::maybe_replan(const BinarySystem& bin,
                                     const Config& config, const State& state,
                                     double now, double replan_interval,
                                     double miss_tolerance) {
    if (!engaged_) {
        return false;
    }
    // Never interrupt an in-progress correction BURN (M06-R19: a burn may
    // re-enter ALIGN as a zero-throttle alignment hold, so it is tracked
    // here, but a re-aim must not replace a partially delivered correction);
    // a re-aim during an ALIGN swing only refreshes the swing target and is
    // allowed, as in the original bounded-rate design.
    if (fast_.state() == ExecutorState::Burn) {
        return false;
    }
    // Bounded-rate gate: at most one slow solve per replan_interval, never on
    // every 1/120 s fixed step (M06-R5-05 / V05).
    if (now - last_replan_ < replan_interval) {
        return false;
    }
    last_replan_ = now;

    // WARM slow planner: re-aim a midcourse correction from the current time.
    // plan_transfer is warm-first (bounded differential correction reusing the
    // cache) and falls back to the coarse cold search only on warm failure.
    // The source frame is held stable for the whole transfer (the cache's
    // route), so the reference-body switch near the target does not flip the
    // transfer direction.
    const int source = cache_.source >= 0 ? cache_.source : reference_body_;
    ManeuverNode corr{};
    corr.time = now;
    corr.frame_body = source;
    const auto solved =
        plan_transfer(bin, config, state, now, source, corr, &cache_);
    ++slow_plans_;
    if (!solved) {
        return true;  // no solution this cycle; hold the current node
    }

    // A miss beyond tolerance: the re-aim's required correction exceeds the
    // tolerance, so re-target the fast VGO with the corrected node. Otherwise
    // the ship is on the planned arc and the fast controller simply holds it.
    const double corr_dv = std::hypot(solved->dv_prograde, solved->dv_radial);
    if (corr_dv <= miss_tolerance) {
        return true;
    }

    const NodeBasis basis = compute_node_basis(
        bin, now, solved->frame_body, {state.x, state.y},
        {state.vx, state.vy});
    // Burn-continuation re-arm: a re-target that keeps the nose within the
    // continuation band of the new VGO direction (with bounded rate) resumes
    // the correction burn immediately instead of paying a full ALIGN
    // re-swing; otherwise the ordinary ALIGN re-swing applies. The re-target
    // always arms a late node (time = now), so a direct burn is legal.
    const Vec2 new_world_dv = node_world_dv(*solved, basis);
    const double desired =
        vec_length(new_world_dv) > 1.0e-9
            ? std::atan2(-new_world_dv.x, new_world_dv.y)
            : state.angle;
    const double err_to_new = std::abs(wrap_pi(desired - state.angle));
    const bool direct =
        err_to_new <= kContinuationAngleBand &&
        std::abs(state.omega) <= kAlignOmegaBand;
    fast_.arm(*solved, basis, now, config, direct);
    node_ = *solved;
    ++retargets_;
    return true;
}
// END CANONICAL ALGORITHM: bounded-rate warm transfer midcourse

}  // namespace lander
