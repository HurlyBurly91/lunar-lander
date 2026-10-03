// M06-R3: live trajectory projection and shifted receding-horizon prediction.
// See include/lander/predictor.hpp for the contract.

#include "lander/predictor.hpp"

#include <algorithm>
#include <cmath>

namespace lander {

namespace {

// The destination base pad is a point fixed on the destination body's rotating
// surface; both position and velocity track the tidal-lock spin. Mirrors the
// GUI's destination-pad helper so the predictor and the live flight agree on
// the Target / AntiTarget attitude reference.
Vec2 destination_pad_position(const Simulation& sim, double t) {
    const int dest = sim.contract().destination_body;
    const Terrain& terrain = sim.binary().body(dest).terrain;
    return sim.binary().surface_point(dest, terrain.angle_at_arc(0.0),
                                      terrain.surface_radius_at_arc(0.0),
                                      t).position;
}

}  // namespace

PolicySignature make_policy_signature(const FlightPolicy& policy,
                                      const NodeExecutor& executor) {
    PolicySignature sig{};
    sig.kind = policy.kind;
    sig.throttle = policy.throttle;
    sig.attitude_mode = policy.attitude_mode;
    sig.rw_enabled = policy.rw_enabled;
    sig.rw_hold = policy.rw_hold;
    sig.reference_body = policy.reference_body;
    sig.target_body = policy.target_body;
    if (policy.node) {
        sig.node_present = true;
        sig.node_time = policy.node->time;
        sig.node_frame_body = policy.node->frame_body;
        sig.node_prograde = policy.node->dv_prograde;
        sig.node_radial = policy.node->dv_radial;
    }
    sig.executor_state = executor.state();
    sig.executor_dv_x = executor.dv_remaining().x;
    sig.executor_dv_y = executor.dv_remaining().y;
    return sig;
}

Input compose_step_input(Simulation& sim, const FlightPolicy& policy,
                         const NodeExecutor& executor, bool manual_left,
                         bool manual_right) {
    const State& before = sim.state();
    const double now = sim.sim_time();
    const Config& config = sim.config();

    Input step_input{};
    if (executor.active()) {
        step_input = executor.make_input(before, now, config, manual_left,
                                         manual_right);
    } else {
        const BinarySystem& bin = sim.binary();
        const int ref = policy.reference_body;
        const Vec2 pos{before.x, before.y};
        const Vec2 vel{before.vx, before.vy};
        const NodeBasis basis =
            compute_node_basis(bin, now, ref, pos, vel);
        Vec2 maneuver_dv{};
        if (policy.attitude_mode == AttitudeMode::Maneuver && policy.node) {
            maneuver_dv = node_world_dv(*policy.node, basis);
        }
        const Vec2 target = destination_pad_position(sim, now);
        const auto direction = attitude_target_direction(
            policy.attitude_mode, basis, pos, target, maneuver_dv);
        step_input = attitude_input(before, config, direction, manual_left,
                                    manual_right);
        step_input.main_throttle = policy.throttle;
    }

    // Reaction-wheel damping composes with the manual-rotation priority rule
    // and is suppressed when the craft is not active (crashed). Mirrors the
    // live-loop ReactionWheelToggle::input composition.
    const bool manual_rotation = step_input.rotate_left || step_input.rotate_right;
    step_input.reaction_wheels =
        !before.crashed && (policy.rw_enabled || policy.rw_hold) &&
        !manual_rotation;

    return step_input;
}

PredictorSample
RecedingHorizonPredictor::sample_from(Simulation& sim) const {
    PredictorSample s{};
    s.state = sim.state();
    s.time = sim.sim_time();
    s.tick = sim.state().ticks;
    return s;
}

bool RecedingHorizonPredictor::matches(const State& a, const State& b) const {
    const bool terminal_same =
        (a.landed == b.landed) && (a.crashed == b.crashed) &&
        (a.landed_body == b.landed_body) && (a.crash_body == b.crash_body);
    if (!terminal_same) {
        return false;
    }
    if (a.landed || a.crashed) {
        // Once a terminal event has occurred the exact match is enough.
        return true;
    }
    const double t = tolerance_;
    return std::abs(a.x - b.x) <= t && std::abs(a.y - b.y) <= t &&
           std::abs(a.vx - b.vx) <= t && std::abs(a.vy - b.vy) <= t &&
           std::abs(a.angle - b.angle) <= t && std::abs(a.omega - b.omega) <= t &&
           std::abs(a.fuel - b.fuel) <= t;
}

void RecedingHorizonPredictor::extend(int max_steps, const FlightPolicy& policy) {
    for (int i = 0; i < max_steps; ++i) {
        if (terminal_) {
            break;
        }
        // A horizon of `horizon_steps_` future steps means the sample window
        // holds the origin plus `horizon_steps_` projected states (horizon+1
        // total). Stop once that many are cached.
        if (static_cast<int>(samples_.size()) > horizon_steps_) {
            break;
        }
        const State before = tail_.state();
        const Input input =
            compose_step_input(tail_, policy, executor_, false, false);
        tail_.step_once(input);
        executor_.after_step(before, tail_.state(), input, tail_.sim_time(),
                             tail_.config());
        steps_last_frame_++;
        total_steps_executed_++;

        samples_.push_back(sample_from(tail_));
        if (tail_.state().crashed) {
            contact_ = PredictedContact{true, false, -1, tail_.state().crash_body,
                                        tail_.sim_time(),
                                        {tail_.state().x, tail_.state().y}};
            terminal_ = true;
            break;
        }
        if (tail_.state().landed) {
            contact_ = PredictedContact{true, true, tail_.state().landed_body,
                                        -1, tail_.sim_time(),
                                        {tail_.state().x, tail_.state().y}};
            terminal_ = true;
            break;
        }
    }
}

void RecedingHorizonPredictor::cold_rebuild(const Simulation& seed_sim,
                                            const FlightPolicy& policy,
                                            const NodeExecutor& seed_executor,
                                            int budget_steps) {
    steps_last_frame_ = 0;
    samples_.clear();
    contact_ = {};
    terminal_ = false;
    policy_ = policy;
    // Deep-copy the authoritative simulation into the rollout tail and copy
    // the node executor so the projected flight replays the exact same step
    // code and controller state as real flight. The live simulation is never
    // touched (M06-R3-P02).
    tail_ = seed_sim;
    executor_ = seed_executor;
    samples_.push_back(sample_from(tail_));
    signature_ = make_policy_signature(policy, seed_executor);
    // A direct cold rebuild already establishes a horizon, so the predictor is
    // primed: the next advance shifts/reuses rather than rebuilding again.
    primed_ = true;
    extend(budget_steps, policy);
    cold_rebuilds_++;
}

void RecedingHorizonPredictor::advance(const Simulation& seed_sim,
                                       const State& actual,
                                       const FlightPolicy& policy,
                                       const NodeExecutor& seed_executor,
                                       int budget_steps) {
    steps_last_frame_ = 0;
    if (!primed_) {
        primed_ = true;
        cache_hit_last_ = false;
        cold_rebuild(seed_sim, policy, seed_executor, budget_steps);
        return;
    }

    const PolicySignature sig = make_policy_signature(policy, seed_executor);
    if (sig != signature_) {
        // A future-control assumption changed: invalidate and rebuild from the
        // current authoritative state (M06-R3-10). Never reuse an old horizon.
        invalidations_++;
        cache_hit_last_ = false;
        cold_rebuild(seed_sim, policy, seed_executor, budget_steps);
        return;
    }

    // Shift: the predicted next state must agree with the actual one, or the
    // cache is stale (a disturbance the policy did not capture) and we rebuild
    // (M06-R3-09).
    if (samples_.size() >= 2 && !terminal_ &&
        matches(actual, samples_[1].state)) {
        cache_hit_last_ = true;
        shifts_++;
        samples_.pop_front();
        // Rebuild the tail to the horizon: the ring holds the origin plus
        // `horizon_steps_` projected states (horizon+1 total), so grow back up
        // to that total. In steady state exactly one step is appended; during a
        // progressive fill we grow toward the horizon by the per-frame budget
        // (M06-R3-11).
        const int needed =
            (horizon_steps_ + 1) - static_cast<int>(samples_.size());
        const int grow =
            std::max(1, needed > 0 ? std::min(needed, budget_steps) : 1);
        extend(grow, policy);
        return;
    }

    invalidations_++;
    cache_hit_last_ = false;
    cold_rebuild(seed_sim, policy, seed_executor, budget_steps);
}

}  // namespace lander
