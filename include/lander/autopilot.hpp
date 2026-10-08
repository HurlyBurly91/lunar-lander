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
             const Config& config, bool direct_burn = false);
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

// M06-R5-05 / D05: two-level inter-moon transfer midcourse controller.
//
// Composes the fast velocity-to-be-gained NodeExecutor (HOT: every 1/120 s,
// O(1) - it holds the ship on the planned arc and delivers each small
// correction) with a slower, bounded-rate, warm-started replan (WARM). The
// slow planner is plan_transfer itself: at most once per `replan_interval` it
// re-aims the arc from the ship's current state (warm differential correction,
// cold-fallback only if that fails) and, when the required correction exceeds
// `miss_tolerance` (a miss beyond tolerance), re-targets the fast executor
// with the corrected node. COLD never runs on every fixed step; the fast
// executor keeps running at 120 Hz while the slow planner holds between
// updates.
class TransferMidcourse {
public:
    // Arm the two-level controller to hold the arc to `node` (the planned
    // transfer departure). `solution` seeds the warm-start cache; `basis` is
    // the node basis computed from the predicted pre-burn state;
    // `reference_body` is the navigation frame the transfer was planned in.
    // The fast executor is armed immediately with `node`.
    void arm(const ManeuverNode& node, const TransferSolution& solution,
             int reference_body, const NodeBasis& basis, double now,
             const Config& config);

    // The two-level controller is engaged in a transfer. True from `arm` until
    // the ship lands / crashes / the controller is aborted; the fast executor
    // may be idle (coasting) between corrections while this stays true.
    bool active() const noexcept { return engaged_; }

    // HOT (O(1), every 1/120 s): ordinary input for the fast VGO. Delegates to
    // the NodeExecutor; it never triggers any transfer solver. Emits no thrust
    // or rotation while the ship is coasting between corrections.
    Input make_input(const State& state, double now, const Config& config,
                     bool manual_left, bool manual_right) const;

    // Advance the fast executor after one authoritative fixed step; disengage
    // when the ship lands or crashes.
    void after_step(const State& before, const State& after,
                    const Input& input, double now, const Config& config);

    // WARM (bounded rate): re-aim the arc from the current state via
    // plan_transfer (warm-first / cold-fallback) at most once per
    // `replan_interval`; never on every fixed step. When the re-aim reports a
    // required correction larger than `miss_tolerance` (a miss beyond
    // tolerance), the fast executor re-targets with the corrected node.
    // Returns true when a slow solve ran on this call (callers use this to
    // count bounded-rate replans, M06-R5-V05).
    bool maybe_replan(const BinarySystem& bin, const Config& config,
                      const State& state, double now,
                      double replan_interval = 0.1,
                      double miss_tolerance = 0.25);

    // Abort the whole controller (fast executor and engaged state).
    void abort();

    // Telemetry for the rate-tier verification (M06-R5-V05 / V08).
    int slow_plans() const noexcept { return slow_plans_; }
    int retargets() const noexcept { return retargets_; }
    const TransferSolution& cache() const noexcept { return cache_; }
    const std::optional<ManeuverNode>& node() const noexcept { return node_; }
    const NodeExecutor& fast() const noexcept { return fast_; }

private:
    NodeExecutor fast_{};
    TransferSolution cache_{};
    std::optional<ManeuverNode> node_{};
    int reference_body_{0};
    bool engaged_{false};
    double last_replan_{0.0};
    int slow_plans_{0};
    int retargets_{0};
};

}  // namespace lander
