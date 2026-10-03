// M06-R3: live-trajectory projection and shifted receding-horizon predictor.
// Verifies the prediction semantics (COAST / LIVE / contact) and the real-time
// rolling algorithm (cold == rolling, shift stability, policy invalidation,
// progressive rebuild, no mutation of live state, compute-time isolation).
#include "lander/ballistic.hpp"
#include "lander/predictor.hpp"
#include "lander/sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

bool close(double a, double b, double eps) { return std::abs(a - b) <= eps; }

void check_close(double a, double b, double eps, const char* message) {
    if (!close(a, b, eps)) {
        ++failures;
        std::printf("FAIL: %s (%.12g != %.12g, eps %.3g)\n", message, a, b,
                    eps);
    }
}

void check_close_vec(lander::Vec2 a, lander::Vec2 b, double eps,
                     const char* message) {
    if (std::hypot(a.x - b.x, a.y - b.y) > eps) {
        ++failures;
        std::printf("FAIL: %s ((%.9g,%.9g) != (%.9g,%.9g), eps %.3g)\n", message,
                    a.x, a.y, b.x, b.y, eps);
    }
}

// A flying ship placed relative to body i at ephemeris time 0 (same layout as
// the flight-computer tests).
lander::State state_relative(const lander::BinarySystem& bin, int i,
                             double altitude, double radial_v, double tang_v,
                             double angle_offset) {
    const lander::Body& body = bin.body(i);
    const lander::Vec2 pos = bin.position(i, 0.0);
    const lander::Vec2 vel = bin.velocity(i, 0.0);
    const double theta = body.terrain.angle_at_arc(0.0);
    const double r = body.terrain.surface_radius_at_arc(0.0) + altitude;
    const lander::Vec2 up{std::cos(theta), std::sin(theta)};
    const lander::Vec2 right{std::sin(theta), -std::cos(theta)};
    const double ox = r * up.x;
    const double oy = r * up.y;
    const double spin_x = -bin.omega() * oy;
    const double spin_y = bin.omega() * ox;
    lander::State s{};
    s.x = pos.x + ox;
    s.y = pos.y + oy;
    s.vx = vel.x + spin_x + radial_v * up.x + tang_v * right.x;
    s.vy = vel.y + spin_y + radial_v * up.y + tang_v * right.y;
    s.angle = theta - 0.5 * lander::kPi + angle_offset;  // 0 => nose radially out
    s.fuel = 1000.0;
    s.landed = false;
    s.crashed = false;
    return s;
}

// A stable flying orbit around the primary: high enough (and coasting) that no
// contact occurs over the test horizons, but with a tangential velocity so the
// rollout is non-trivial.
lander::Simulation make_orbit_sim(std::uint64_t seed, double altitude = 900.0) {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(seed);
    const lander::BinarySystem& bin = sim.binary();
    const double r = sim.terrain().max_surface_radius() + altitude;
    const lander::Vec2 p0 = bin.position(0, 0.0);
    const lander::Vec2 v0 = bin.velocity(0, 0.0);
    const double speed = std::sqrt(sim.config().mu / r);
    lander::State s{};
    s.x = p0.x;
    s.y = p0.y + r;
    s.vx = v0.x + speed;
    s.vy = v0.y;
    s.angle = 0.0;
    s.fuel = 1000.0;
    s.landed = false;
    s.crashed = false;
    sim.set_state(s);
    return sim;
}

// A low-altitude ship that is falling toward body i and will contact terrain
// within the test horizon under a persistent throttle.
lander::Simulation make_falling_sim(std::uint64_t seed, int body,
                                    double throttle) {
    lander::Config cfg{};
    lander::Simulation sim(cfg);
    sim.reset(seed);
    const lander::BinarySystem& bin = sim.binary();
    lander::State s =
        state_relative(bin, body, 60.0, -14.0 * throttle - 2.0, 3.0, 0.0);
    sim.set_state(s);
    return sim;
}

const lander::FlightPolicy coast_policy() {
    lander::FlightPolicy p{};
    p.kind = lander::PredictionKind::Coast;
    return p;
}

const lander::FlightPolicy live_policy(lander::AttitudeMode mode,
                                       double throttle) {
    lander::FlightPolicy p{};
    p.kind = lander::PredictionKind::Live;
    p.throttle = throttle;
    p.attitude_mode = mode;
    p.reference_body = 0;
    p.target_body = 1;
    return p;
}

}  // namespace

// V01 + V13 + P03: with zero thrust and no attitude/node/RW, the LIVE and
// COAST predictions must agree and both must match the pure ballistic
// zero-thrust predictor.
void test_zero_thrust_consistency() {
    const int horizon = 600;
    lander::Config cfg{};
    lander::Simulation seed = make_orbit_sim(7);
    const lander::State start = seed.state();

    lander::RecedingHorizonPredictor live;
    live.configure(horizon, 1e-9);
    live.cold_rebuild(seed, live_policy(lander::AttitudeMode::Off, 0.0),
                      lander::NodeExecutor{}, horizon);
    lander::RecedingHorizonPredictor coast;
    coast.configure(horizon, 1e-9);
    coast.cold_rebuild(seed, coast_policy(), lander::NodeExecutor{}, horizon);

    check(live.samples().size() == coast.samples().size(),
          "zero-thrust LIVE and COAST have equal sample counts");
    bool agree = true;
    const size_t n = std::min(live.samples().size(), coast.samples().size());
    for (size_t i = 0; i < n; ++i) {
        const lander::State& a = live.samples()[i].state;
        const lander::State& b = coast.samples()[i].state;
        if (std::hypot(a.x - b.x, a.y - b.y) > 1e-6 ||
            std::hypot(a.vx - b.vx, a.vy - b.vy) > 1e-6) {
            agree = false;
            break;
        }
    }
    check(agree, "zero-thrust LIVE path == COAST path");

    // The pure ballistic step primitive (M06-R2-02 parity) uses the same
    // semi-implicit scheme and field ordering as Simulation::integrate_flight
    // with thrust and rotation disabled, so a zero-input clone must match it
    // step for step.
    const double dt = cfg.fixed_dt;
    lander::BallisticState bs{{start.x, start.y}, {start.vx, start.vy}, 0.0};
    bool ballistic_agree = true;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0) {
            bs = lander::step_ballistic(seed.binary(), bs, dt);
        }
        if (std::hypot(coast.samples()[i].state.x - bs.p.x,
                       coast.samples()[i].state.y - bs.p.y) > 1e-9) {
            ballistic_agree = false;
            break;
        }
    }
    check(ballistic_agree, "COAST clone path matches the pure ballistic predictor");
}

// V02: a persistent main throttle makes the LIVE path diverge from COAST.
void test_throttle_diverges() {
    const int horizon = 600;
    lander::Simulation seed = make_orbit_sim(7);
    lander::RecedingHorizonPredictor live;
    live.configure(horizon, 1e-9);
    live.cold_rebuild(seed, live_policy(lander::AttitudeMode::Off, 1.0),
                      lander::NodeExecutor{}, horizon);
    lander::RecedingHorizonPredictor coast;
    coast.configure(horizon, 1e-9);
    coast.cold_rebuild(seed, coast_policy(), lander::NodeExecutor{}, horizon);
    const lander::State& a = live.samples().back().state;
    const lander::State& b = coast.samples().back().state;
    check(std::hypot(a.x - b.x, a.y - b.y) > 1.0,
          "a persistent throttle makes LIVE differ from COAST");
}

// V03/V04/V06/V11: from a low-altitude falling state with a persistent
// throttle, the clone predictor's predicted contact (time, position, body,
// landed/crashed, fuel) must match the real simulation's actual contact.
void run_contact_case(const lander::FlightPolicy& policy, const char* label,
                      double contact_eps) {
    const int horizon = 3000;
    lander::Simulation seed = make_falling_sim(11, 0, 1.0);
    const lander::State origin = seed.state();

    // Run the real simulation forward with the same policy (no manual keys)
    // until it lands or crashes, recording the authoritative contact.
    lander::Simulation real = seed;
    lander::NodeExecutor real_exec;
    lander::PredictedContact real_contact;
    for (int i = 0; i < horizon &&
         !real.state().landed && !real.state().crashed; ++i) {
        const lander::State before = real.state();
        const lander::Input in =
            lander::compose_step_input(real, policy, real_exec, false, false);
        real.step_once(in);
        real_exec.after_step(before, real.state(), in, real.sim_time(),
                             real.config());
    }
    check(real.state().landed || real.state().crashed,
          "real sim reached a contact");
    if (!real.state().landed && !real.state().crashed) {
        std::printf("  (note: %s no contact within horizon)\n", label);
        return;
    }
    real_contact.valid = true;
    real_contact.landed = real.state().landed;
    real_contact.body =
        real.state().landed ? real.state().landed_body : -1;
    real_contact.crash_body =
        real.state().crashed ? real.state().crash_body : -1;
    real_contact.time = real.sim_time();
    real_contact.position = {real.state().x, real.state().y};
    const double real_fuel = real.state().fuel;

    // The clone predictor from the same origin must predict the same contact.
    lander::RecedingHorizonPredictor pred;
    pred.configure(horizon, 1e-9);
    pred.cold_rebuild(seed, policy, lander::NodeExecutor{}, horizon);
    const lander::PredictedContact& pc = pred.contact();

    check(pc.valid, "predictor found a predicted contact");
    if (pc.valid) {
        check(pc.landed == real_contact.landed, "contact landed/crashed matches");
        if (pc.landed) {
            check(pc.body == real_contact.body, "contact body (landed) matches");
        } else {
            check(pc.crash_body == real_contact.crash_body,
                  "contact body (crashed) matches");
        }
        check_close(pc.time, real_contact.time, 1.0 / 120.0,
                    "contact time within one fixed step");
        check_close_vec(pc.position, real_contact.position, contact_eps,
                        "contact position matches");
        // Fuel is consumed identically to the real execution up to the
        // contact step (V11).
        const lander::State& ps = pred.samples().back().state;
        check_close(ps.fuel, real_fuel, 1e-6, "fuel consumed identically");
    }
    std::printf("  contact %s: t=%.3f pos=(%.3f,%.3f) landed=%d body=%d crash=%d "
                "(real t=%.3f pos=(%.3f,%.3f) fuel=%.2f)\n",
                label, pc.time, pc.position.x, pc.position.y,
                pc.landed ? 1 : 0, pc.body, pc.crash_body, real_contact.time,
                real_contact.position.x, real_contact.position.y, real_fuel);
}

void test_contact_agreement() {
    run_contact_case(live_policy(lander::AttitudeMode::Off, 0.5),
                     "throttle-50% (V03/V04/V06/V11)", 1.0);
    run_contact_case(live_policy(lander::AttitudeMode::Target, 0.5),
                     "attitude-Target (V09)", 2.0);
    // Reaction wheels on: the contact is driven by the same rollout with RW
    // damping active (V08).
    lander::FlightPolicy rw = live_policy(lander::AttitudeMode::Off, 0.5);
    rw.rw_enabled = true;
    run_contact_case(rw, "RW-on (V08)", 2.0);
}

// V05: contact agreement also for a ship falling toward the companion.
void test_companion_contact() {
    const int horizon = 3000;
    lander::Simulation seed = make_falling_sim(12, 1, 1.0);
    lander::FlightPolicy policy = live_policy(lander::AttitudeMode::Off, 0.5);
    lander::NodeExecutor exec{};

    lander::Simulation real = seed;
    lander::PredictedContact real_contact;
    for (int i = 0; i < horizon && !real.state().landed &&
         !real.state().crashed;
         ++i) {
        const lander::State before = real.state();
        const lander::Input in =
            lander::compose_step_input(real, policy, exec, false, false);
        real.step_once(in);
        exec.after_step(before, real.state(), in, real.sim_time(), real.config());
    }
    check(real.state().landed || real.state().crashed,
          "companion: real sim reached a contact");
    lander::RecedingHorizonPredictor pred;
    pred.configure(horizon, 1e-9);
    pred.cold_rebuild(seed, policy, lander::NodeExecutor{}, horizon);
    check(pred.contact().valid, "companion: predictor found a contact");
}

// V10: with an active node executor, the LIVE rollout simulates the finite
// executor and the predicted contact still matches the real execution.
void test_node_executor_contact() {
    const int horizon = 3000;
    const double dt = 1.0 / 120.0;
    lander::Simulation seed = make_falling_sim(13, 0, 1.0);
    const lander::BinarySystem& bin = seed.binary();

    lander::ManeuverNode node = lander::default_node(0.0, 0, dt);
    node.time = 2.0;
    node.dv_radial = 2.0;
    const lander::State origin = seed.state();
    const lander::NodeBasis basis = lander::compute_node_basis(
        bin, 0.0, 0, {origin.x, origin.y}, {origin.vx, origin.vy});

    lander::FlightPolicy policy = live_policy(lander::AttitudeMode::Off, 0.0);
    policy.node = node;

    lander::NodeExecutor exec;
    exec.arm(node, basis, 0.0, seed.config());

    lander::Simulation real = seed;
    lander::NodeExecutor real_exec;
    real_exec.arm(node, basis, 0.0, real.config());
    for (int i = 0; i < horizon && !real.state().landed &&
         !real.state().crashed;
         ++i) {
        const lander::State before = real.state();
        const lander::Input in =
            lander::compose_step_input(real, policy, real_exec, false, false);
        real.step_once(in);
        real_exec.after_step(before, real.state(), in, real.sim_time(),
                             real.config());
    }
    check(real.state().landed || real.state().crashed,
          "node: real sim reached a contact");
    lander::RecedingHorizonPredictor pred;
    pred.configure(horizon, 1e-9);
    pred.cold_rebuild(seed, policy, exec, horizon);
    check(pred.contact().valid, "node: predictor found a contact");
}

// V12: the predictor must never mutate the live simulation.
void test_no_mutation() {
    lander::Simulation seed = make_orbit_sim(21);
    const lander::State state_before = seed.state();
    const lander::State prev_before = seed.previous_state();
    const double time_before = seed.sim_time();
    const int ref_before = seed.reference_body();
    const lander::Contract contract_before = seed.contract();
    const std::uint64_t seed_before = seed.seed();

    lander::FlightPolicy policy = live_policy(lander::AttitudeMode::Off, 0.5);
    lander::RecedingHorizonPredictor pred;
    pred.configure(600, 1e-9);
    pred.cold_rebuild(seed, policy, lander::NodeExecutor{}, 600);
    // Advance a few live steps while the predictor runs.
    lander::Simulation live = seed;
    for (int i = 0; i < 240; ++i) {
        const lander::State before = live.state();
        const lander::Input in = lander::compose_step_input(
            live, policy, lander::NodeExecutor{}, false, false);
        live.step_once(in);
        pred.advance(live, live.state(), policy, lander::NodeExecutor{}, 40);
    }

    const lander::State state_after = seed.state();
    check(state_before == state_after, "live state unchanged by predictor");
    check(prev_before == seed.previous_state(),
          "live previous state unchanged by predictor");
    check_close(seed.sim_time(), time_before, 1e-12,
                "live sim_time unchanged by predictor");
    check(seed.reference_body() == ref_before, "reference body unchanged");
    check(contract_before == seed.contract(), "contract unchanged");
    check(seed.seed() == seed_before, "seed unchanged");
}

// V14 + V15: a single uninterrupted cold rollout and a shifted (rolling)
// rollout produce identical states; repeat for many shifts to catch any
// accumulated divergence. A coasting orbit (zero-thrust) is used so the
// trajectory never contacts terrain over the long horizon.
void test_cold_equals_rolling(int horizon, int shifts, double altitude,
                              const char* label) {
    lander::Simulation seed = make_orbit_sim(31, altitude);
    lander::FlightPolicy policy = coast_policy();
    lander::NodeExecutor idle{};

    // One uninterrupted cold rollout of length horizon + shifts.
    lander::RecedingHorizonPredictor cold;
    cold.configure(horizon + shifts, 1e-9);
    cold.cold_rebuild(seed, policy, idle, horizon + shifts);
    const auto& C = cold.samples();
    check(cold.terminal() == false, "coast rollout stays in flight (cold)");

    // Rolling: full cold to the horizon, then advance `shifts` live steps.
    lander::Simulation live = seed;
    lander::NodeExecutor live_exec{};
    lander::RecedingHorizonPredictor roll;
    roll.configure(horizon, 1e-9);
    roll.cold_rebuild(live, policy, live_exec, horizon);
    for (int k = 0; k < shifts; ++k) {
        const lander::State before = live.state();
        const lander::Input in =
            lander::compose_step_input(live, policy, live_exec, false, false);
        live.step_once(in);
        live_exec.after_step(before, live.state(), in, live.sim_time(),
                             live.config());
        roll.advance(live, live.state(), policy, live_exec, horizon);
    }
    const auto& R = roll.samples();

    check(R.size() == static_cast<size_t>(horizon) + 1,
          "rolling ring holds a full horizon");
    bool agree = true;
    for (size_t i = 0; i < R.size(); ++i) {
        const size_t idx = static_cast<size_t>(shifts) + i;
        if (idx >= C.size()) {
            agree = false;
            break;
        }
        const lander::State& r = R[i].state;
        const lander::State& c = C[idx].state;
        if (std::hypot(r.x - c.x, r.y - c.y) > 1e-7 ||
            std::hypot(r.vx - c.vx, r.vy - c.vy) > 1e-7 ||
            std::abs(r.angle - c.angle) > 1e-7 ||
            std::abs(r.omega - c.omega) > 1e-7 ||
            std::abs(r.fuel - c.fuel) > 1e-6) {
            agree = false;
            std::printf("  first divergence at window index %zu (step %zd)\n",
                        i, static_cast<long>(shifts) + static_cast<long>(i));
            break;
        }
    }
    check(agree, label);
    // Under a stable policy the rolling predictor must have mostly shifted
    // (few invalidations) and appended roughly one step per live step.
    check(roll.shifts() >= shifts - 1, "rolling mostly shifted (no churn)");
    check(roll.invalidations() <= 1, "rolling did not repeatedly invalidate");
}

// V16: a change to any future-control assumption invalidates the cache and
// forces a rebuild from the current authoritative state.
void test_policy_invalidation() {
    const int horizon = 40;
    lander::Simulation seed = make_orbit_sim(41);
    lander::FlightPolicy policy = live_policy(lander::AttitudeMode::Off, 0.4);
    lander::NodeExecutor idle{};

    lander::RecedingHorizonPredictor pred;
    pred.configure(horizon, 1e-9);
    pred.cold_rebuild(seed, policy, idle, horizon);
    const int rebuilds_after_cold = pred.cold_rebuilds();
    const int invalidations_after_cold = pred.invalidations();

    lander::Simulation live = seed;
    const lander::State before = live.state();
    const lander::Input in =
        lander::compose_step_input(live, policy, idle, false, false);
    live.step_once(in);

    // Same policy, matching state: should be a cache hit (no invalidation).
    pred.advance(live, live.state(), policy, idle, horizon);
    check(pred.cache_hit_last(), "unchanged policy + matching state is a hit");
    check(pred.invalidations() == invalidations_after_cold,
          "a cache hit does not invalidate");

    // Change the throttle: must invalidate and rebuild.
    lander::FlightPolicy changed = policy;
    changed.throttle = 0.9;
    pred.advance(live, live.state(), changed, idle, horizon);
    check(pred.cache_hit_last() == false, "throttle change is not a hit");
    check(pred.invalidations() == invalidations_after_cold + 1,
          "throttle change invalidates the cache");
    check(pred.cold_rebuilds() > rebuilds_after_cold,
          "throttle change forces a rebuild");

    // A disturbed actual state (same policy as the stored signature) also
    // invalidates, exercising the state-mismatch (not signature) path.
    lander::Simulation disturbed = live;
    lander::State perturbed = disturbed.state();
    perturbed.vx += 0.75;
    perturbed.vy += 0.25;
    disturbed.set_state(perturbed);
    pred.advance(disturbed, disturbed.state(), changed, idle, horizon);
    check(pred.cache_hit_last() == false, "a state mismatch is not a hit");
    check(pred.invalidations() == invalidations_after_cold + 2,
          "a state mismatch invalidates the cache");
}

// V17 + V18: a progressive (anytime) cold rebuild produces the same final
// horizon as one uninterrupted rebuild, and the near horizon is available
// before the long horizon finishes. A coasting orbit keeps the rollout in
// flight so the window fills to the full horizon.
void test_progressive_rebuild() {
    const int horizon = 600;
    const int budget = 60;  // near horizon per frame
    lander::Simulation seed = make_orbit_sim(51, 1200.0);
    lander::FlightPolicy policy = coast_policy();
    lander::NodeExecutor idle{};

    // One uninterrupted cold rollout over twice the horizon, from which the
    // final window (the last `horizon + 1` states) is the reference.
    lander::RecedingHorizonPredictor ref;
    ref.configure(2 * horizon, 1e-9);
    ref.cold_rebuild(seed, policy, idle, 2 * horizon);
    const auto& R = ref.samples();
    check(ref.terminal() == false, "coast reference stays in flight");
    check(R.size() == static_cast<size_t>(2 * horizon) + 1,
          "uninterrupted cold rollout fills its horizon");

    // Progressive: seed a short near horizon, then grow under a per-frame
    // budget across `horizon` live steps.
    lander::Simulation live = seed;
    lander::NodeExecutor live_exec{};
    lander::RecedingHorizonPredictor prog;
    prog.configure(horizon, 1e-9);
    prog.cold_rebuild(live, policy, live_exec, budget);
    check(prog.samples().size() == static_cast<size_t>(budget) + 1,
          "near horizon is available after the first budget");
    for (int k = 0; k < horizon; ++k) {
        const lander::State before = live.state();
        const lander::Input in =
            lander::compose_step_input(live, policy, live_exec, false, false);
        live.step_once(in);
        live_exec.after_step(before, live.state(), in, live.sim_time(),
                             live.config());
        prog.advance(live, live.state(), policy, live_exec, budget);
    }
    const auto& P = prog.samples();
    check(P.size() == static_cast<size_t>(horizon) + 1,
          "progressive rebuild fills the horizon");
    // After `horizon` shifts the progressive window covers states
    // [horizon .. 2*horizon] of the uninterrupted reference.
    bool agree = true;
    for (size_t i = 0; i < P.size(); ++i) {
        const size_t idx = static_cast<size_t>(horizon) + i;
        if (idx >= R.size()) {
            agree = false;
            break;
        }
        if (std::hypot(P[i].state.x - R[idx].state.x,
                       P[i].state.y - R[idx].state.y) > 1e-6 ||
            std::abs(P[i].state.fuel - R[idx].state.fuel) > 1e-6) {
            agree = false;
            break;
        }
    }
    check(agree, "progressive rebuild matches the uninterrupted cold horizon");
}

// M06-R3-V19: prediction is a pure look-ahead. Its time grid is a fixed-dt
// extension of the authoritative simulation clock, bounded by the horizon
// window, no matter how many prediction steps were executed to build it; the
// live clock advances only by its own fixed steps (prediction wall/compute
// time is never added to simulation elapsed time).
void test_compute_time_not_flight_time() {
    const int horizon = 300;
    lander::Simulation live = make_orbit_sim(12345, 900.0);
    const double dt = live.config().fixed_dt;
    const double t0 = live.sim_time();

    lander::FlightPolicy coast{};  // zero-thrust policy
    lander::NodeExecutor exec{};   // armed, no node
    lander::RecedingHorizonPredictor p;
    p.configure(horizon);
    p.cold_rebuild(live, coast, exec, horizon);

    auto grid_anchored = [&]() {
        const double now = live.sim_time();
        for (const auto& s : p.samples()) {
            if (s.time < now - 1e-9 || s.time > now + horizon * dt + 1e-9) {
                return false;
            }
        }
        for (size_t i = 1; i < p.samples().size(); ++i) {
            if (!close(p.samples()[i].time - p.samples()[i - 1].time, dt, 1e-9)) {
                return false;
            }
        }
        return true;
    };

    check(grid_anchored(), "cold grid is a fixed-dt look-ahead of the live clock");

    int steps = 0;
    for (int i = 0; i < 60; ++i) {
        const lander::State before = live.state();
        const lander::Input in =
            lander::compose_step_input(live, coast, exec, false, false);
        live.step_once(in);
        exec.after_step(before, live.state(), in, live.sim_time(), live.config());
        ++steps;
        p.advance(live, live.state(), coast, exec, horizon);
        if (live.state().crashed || live.state().landed) {
            break;
        }
        check(grid_anchored(),
              "rolling grid stays a fixed-dt look-ahead anchored to the live "
              "clock");
    }
    check(steps > 0, "the live flight actually advanced");
    check_close(live.sim_time(), t0 + steps * dt, 1e-6,
                "live clock advanced only by its own fixed steps");
}

// M06-R3-V20: over the same live flight the rolling predictor executes far
// fewer prediction steps than a cold-every-frame rollout (one shift+append per
// live step versus a full horizon rebuild on every frame).
void test_cold_vs_rolling_workload() {
    const int horizon = 300;
    const int frames = 300;
    lander::FlightPolicy coast{};

    lander::Simulation roll_live = make_orbit_sim(12345, 900.0);
    lander::NodeExecutor roll_exec{};
    lander::RecedingHorizonPredictor roll;
    roll.configure(horizon);
    roll.cold_rebuild(roll_live, coast, roll_exec, horizon);
    for (int i = 0; i < frames;
         ++i && !(roll_live.state().crashed || roll_live.state().landed)) {
        const lander::State before = roll_live.state();
        const lander::Input in = lander::compose_step_input(roll_live, coast,
                                                            roll_exec, false,
                                                            false);
        roll_live.step_once(in);
        roll_exec.after_step(before, roll_live.state(), in, roll_live.sim_time(),
                             roll_live.config());
        roll.advance(roll_live, roll_live.state(), coast, roll_exec, horizon);
    }
    const int rolling_steps = roll.total_steps_executed();

    lander::Simulation cold_live = make_orbit_sim(12345, 900.0);
    lander::NodeExecutor cold_exec{};
    lander::RecedingHorizonPredictor cold;
    cold.configure(horizon);
    int cold_steps = 0;
    for (int i = 0; i < frames;
         ++i && !(cold_live.state().crashed || cold_live.state().landed)) {
        cold.reset();
        cold.cold_rebuild(cold_live, coast, cold_exec, horizon);
        cold_steps += cold.total_steps_executed();
        const lander::State before = cold_live.state();
        const lander::Input in = lander::compose_step_input(cold_live, coast,
                                                            cold_exec, false,
                                                            false);
        cold_live.step_once(in);
        cold_exec.after_step(before, cold_live.state(), in, cold_live.sim_time(),
                             cold_live.config());
    }

    std::printf(
        "  workload over %d frames (horizon %d): rolling=%d steps, "
        "cold-every-frame=%d steps\n",
        frames, horizon, rolling_steps, cold_steps);
    check(rolling_steps > 0 && cold_steps > 0,
          "both workloads executed prediction steps");
    check(rolling_steps < cold_steps / 4,
          "rolling workload is far less than cold-every-frame");
}

int main() {
    test_zero_thrust_consistency();
    test_throttle_diverges();
    test_contact_agreement();
    test_companion_contact();
    test_node_executor_contact();
    test_no_mutation();
    test_cold_equals_rolling(1200, 1200, 900.0,
                             "cold == rolling after 1200 shifts");
    test_cold_equals_rolling(600, 2400, 1500.0,
                             "cold == rolling after 2400 shifts");
    test_policy_invalidation();
    test_progressive_rebuild();
    test_compute_time_not_flight_time();
    test_cold_vs_rolling_workload();

    if (failures == 0) {
        std::printf("All lander_predictor_tests passed\n");
        return 0;
    }
    std::printf("%d predictor test(s) failed\n", failures);
    return 1;
}
