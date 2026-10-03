#pragma once

// M06-R3: live trajectory projection and shifted receding-horizon prediction.
//
// This module defines the prediction-policy plumbing and the rolling
// predictor that projects the authoritative Simulation forward with "no
// further control changes". The rollout reuses the exact live-flight step
// code (compose_step_input + Simulation::step_once + NodeExecutor), so the
// projected path and its terrain contact are the authoritative cloned-sim
// result, never a second approximate integrator (M06-R3-P01/P14).

#include "lander/autopilot.hpp"
#include "lander/sim.hpp"

#include <deque>

namespace lander {

// The three explicitly-labelled prediction concepts (M06-R3-01). COAST is the
// zero-thrust ballistic path (kept for orbital mechanics), LIVE is what the
// actual spacecraft will do if the player makes no further control changes,
// and PLAN is the ideal node / autopilot planned path.
enum class PredictionKind {
    Coast,
    Live,
    Plan,
};

// The persistent control state that defines a LIVE projection: everything the
// player has set that survives beyond the current instant. Momentary manual
// rotation keys are deliberately NOT part of this policy: the LIVE projection
// assumes they are released after the current instant (M06-R3-02).
struct FlightPolicy {
    PredictionKind kind{PredictionKind::Live};
    double throttle{0.0};
    AttitudeMode attitude_mode{AttitudeMode::Off};
    bool rw_enabled{false};
    bool rw_hold{false};
    std::optional<ManeuverNode> node;
    int reference_body{0};
    int target_body{1};
};

// A deterministic identity for the prediction policy (M06-R3-10). If any
// future-control assumption changes, the cached horizon must be rebuilt from
// the current authoritative state; it is never reused under a different
// policy.
struct PolicySignature {
    PredictionKind kind{PredictionKind::Live};
    double throttle{0.0};
    AttitudeMode attitude_mode{AttitudeMode::Off};
    bool rw_enabled{false};
    bool rw_hold{false};
    bool node_present{false};
    double node_time{0.0};
    int node_frame_body{0};
    double node_prograde{0.0};
    double node_radial{0.0};
    int reference_body{0};
    int target_body{1};
    ExecutorState executor_state{ExecutorState::Idle};
    double executor_dv_x{0.0};
    double executor_dv_y{0.0};
    bool operator==(const PolicySignature&) const = default;
};

PolicySignature make_policy_signature(const FlightPolicy& policy,
                                      const NodeExecutor& executor);

// One delivery of the per-step simulation input, shared by the live flight
// loop and the predictor so the projected flight is bit-for-bit the same code
// path as real flight (M06-R3-03). `sim` supplies the before-state, ephemeris
// time, reference body and destination pad; `manual_left`/`manual_right` are
// the momentary rotation keys (false for a "no further control changes"
// projection).
Input compose_step_input(Simulation& sim, const FlightPolicy& policy,
                         const NodeExecutor& executor, bool manual_left,
                         bool manual_right);

// A lightweight per-step prediction sample: the authoritative fixed-step state
// at `time` (simulation seconds) / `tick`. The full Simulation is not stored
// per point, only at the moving tail (M06-R3-12).
struct PredictorSample {
    State state{};
    double time{0.0};
    int tick{0};
};

// The predicted contact outcome from the cloned rollout (M06-R3-04): the
// authoritative PRED LAND / PRED CRASH result, with the contact body, time and
// world position.
struct PredictedContact {
    bool valid{false};
    bool landed{false};
    int body{-1};
    int crash_body{-1};
    double time{0.0};
    Vec2 position{};
};

// The shifted receding-horizon predictor (M06-R3-08..12). It holds ONE cloned
// Simulation (the rollout tail) and a ring of per-step samples. Under an
// unchanged policy whose predicted next state matches the actual state it
// shifts by one fixed step (O(1) new sim work); on a policy change or a cache
// mismatch it cold-rebuilds from the actual state, progressively, under a
// per-frame budget (M06-R3-11). It never mutates the live simulation.
class RecedingHorizonPredictor {
public:
    void configure(int horizon_steps, double tolerance = 1.0e-9) {
        horizon_steps_ = horizon_steps > 1 ? horizon_steps : 1;
        tolerance_ = tolerance;
    }

    // Cold-rebuild the cached horizon from the current authoritative state of
    // `seed_sim` (a live simulation; only its state/config/binary/contract are
    // copied into the internal tail). Extends up to `budget_steps` this call
    // (progressive rebuild). Resets the contact.
    void cold_rebuild(const Simulation& seed_sim, const FlightPolicy& policy,
                      const NodeExecutor& seed_executor, int budget_steps);

    // One authoritative live step has completed. `actual` is the new live
    // state; `policy` / `seed_executor` describe the current persistent
    // controls. Shifts + appends when consistent, else cold-rebuilds.
    void advance(const Simulation& seed_sim, const State& actual,
                 const FlightPolicy& policy, const NodeExecutor& seed_executor,
                 int budget_steps);

    const std::deque<PredictorSample>& samples() const { return samples_; }
    const PredictedContact& contact() const { return contact_; }
    bool primed() const { return primed_; }
    bool terminal() const { return terminal_; }

    // Diagnostic counters (M06-R3-15).
    bool cache_hit_last() const { return cache_hit_last_; }
    int cold_rebuilds() const { return cold_rebuilds_; }
    int shifts() const { return shifts_; }
    int invalidations() const { return invalidations_; }
    int steps_last_frame() const { return steps_last_frame_; }
    int horizon_steps() const { return horizon_steps_; }
    int total_steps_executed() const { return total_steps_executed_; }

    void reset() {
        samples_.clear();
        contact_ = {};
        primed_ = false;
        terminal_ = false;
        cache_hit_last_ = false;
        cold_rebuilds_ = 0;
        shifts_ = 0;
        invalidations_ = 0;
        steps_last_frame_ = 0;
        total_steps_executed_ = 0;
    }

private:
    void extend(int max_steps, const FlightPolicy& policy);
    PredictorSample sample_from(Simulation& sim) const;
    bool matches(const State& a, const State& b) const;

    Simulation tail_{};
    NodeExecutor executor_{};
    FlightPolicy policy_{};
    std::deque<PredictorSample> samples_;
    PolicySignature signature_{};
    PredictedContact contact_{};
    double tolerance_{1.0e-9};
    int horizon_steps_{1200};
    bool primed_{false};
    bool terminal_{false};
    bool cache_hit_last_{false};
    int cold_rebuilds_{0};
    int shifts_{0};
    int invalidations_{0};
    int steps_last_frame_{0};
    int total_steps_executed_{0};
};

}  // namespace lander
