#pragma once

#include "lander/flight_computer.hpp"
#include "lander/sim.hpp"

#include <optional>

namespace lander {

// SAS-like attitude hold modes. `Maneuver` holds the direction of the active
// executor's remaining delta-v.
enum class AttitudeMode {
    Off,
    Prograde,
    Retrograde,
    RadialOut,
    RadialIn,
    Target,
    AntiTarget,
    Maneuver,
};

const char* attitude_mode_name(AttitudeMode mode);

// Resolve an attitude mode to a desired world-frame thrust direction, if the
// mode is defined for the given frame. Returns `std::nullopt` for `Off` and
// for any degenerate / zero target vector so the caller fails safely.
std::optional<Vec2> attitude_target_direction(
    AttitudeMode mode, const NodeBasis& basis, const Vec2& ship_position,
    const Vec2& target_position, const Vec2& maneuver_dv);

// Bang-bang attitude controller for the existing finite angular-acceleration
// model. Returns an `Input` containing only rotation commands (thrust and
// reaction wheels stay clear), so the result can be merged with any manual
// input. Manual rotation, when present, overrides the hold for this call.
Input attitude_input(const State& state, const Config& config,
                     const std::optional<Vec2>& target_direction,
                     bool manual_left, bool manual_right);

// One-shot node executor state. The executor emits ordinary `Input` only; it
// never mutates spacecraft state directly.
enum class ExecutorState {
    Idle,
    Wait,
    Align,
    Burn,
    Complete,
    Aborted,
    Incomplete,
};

const char* executor_state_name(ExecutorState state);

class NodeExecutor {
public:
    void clear();
    void arm(const ManeuverNode& node, const NodeBasis& basis, double now,
             const Config& config);
    void abort();

    bool active() const noexcept {
        return state_ == ExecutorState::Wait ||
               state_ == ExecutorState::Align ||
               state_ == ExecutorState::Burn;
    }

    // Ordinary input for the next fixed step. Manual rotation overrides the
    // attitude hold for this step but does not cancel the executor; manual
    // main-throttle handling is the GUI's responsibility (it aborts the
    // executor).
    Input make_input(const State& state, double now, const Config& config,
                     bool manual_left, bool manual_right) const;

    // Advance the executor after one authoritative fixed step.
    void after_step(const State& before, const State& after,
                    const Input& input, double now, const Config& config);

    ExecutorState state() const noexcept { return state_; }
    const Vec2& dv_remaining() const noexcept { return dv_remaining_; }
    const Vec2& dv_total() const noexcept { return dv_total_; }
    double burn_time() const noexcept { return burn_time_; }
    double ignite_time() const noexcept { return ignite_time_; }
    double node_time() const noexcept { return node_time_; }
    int frame_body() const noexcept { return frame_body_; }
    bool late() const noexcept { return late_; }

private:
    bool aligned(const State& state) const;

    ExecutorState state_{ExecutorState::Idle};
    int frame_body_{0};
    double node_time_{0.0};
    double ignite_time_{0.0};
    double burn_time_{0.0};
    bool late_{false};
    Vec2 dv_total_{};
    Vec2 dv_remaining_{};
};

}  // namespace lander
