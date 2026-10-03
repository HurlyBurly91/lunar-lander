// M06-R8 subsystem-isolation debug harness -- unit tests.
//
// These tests exercise the new lander::debug_subsystem module (the
// --debug-subsystem selector, the deterministic per-mode scenario seeds, the
// common minimum readout, and the startup-only scenario fixtures) headlessly
// against the `lander` library. They are the automated evidence for:
//   * R8-02  selector parses / round-trips / rejects unknown names
//   * R8-04  deterministic per-mode seeds (unique, nonzero, stable)
//   * R8-03  the common minimum readout is computed for a live simulation
//   * R8-05  each fixture arms exactly the one subsystem under test and leaves
//            the others in their cleared state
//   * R8-06  repeated setup for the same mode is deterministic
//
// No SDL dependency; everything runs from the library.
#include "lander/debug_subsystem.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <set>
#include <string>

namespace {

using lander::AttitudeMode;
using lander::BallisticState;
using lander::DebugSubsystem;
using lander::DebugSubsystems;
using lander::LandingAutopilot;
using lander::LandingConfig;
using lander::ManeuverNode;
using lander::NodeExecutor;
using lander::RecedingHorizonPredictor;
using lander::Simulation;
using lander::State;
using lander::TransferDebugResult;
using lander::TransferMidcourse;
using lander::ZeroEffortQuery;
using lander::debug_scenario_seed;
using lander::debug_subsystem_description;
using lander::debug_subsystem_name;
using lander::keeps_prediction_overlay;
using lander::make_common_readout;
using lander::parse_debug_subsystem;
using lander::setup_debug_scenario;

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", message);
    }
}

void check_close(double a, double b, double eps, const char* message) {
    if (std::abs(a - b) > eps) {
        ++failures;
        std::printf("FAIL: %s (%.9g != %.9g)\n", message, a, b);
    }
}

// A full, comparable record of a freshly-set-up scenario: the authoritative
// state plus every subsystem's engaged flag and the transfer cold-solve
// telemetry. Two runs of the same mode must produce identical snapshots.
struct Snapshot {
    double sim_time = 0.0;
    State st{};
    double altitude = 0.0;
    double tangential = 0.0;
    bool node_present = false;
    bool node_active = false;
    bool transfer_active = false;
    bool landing_armed = false;
    AttitudeMode attitude = AttitudeMode::Off;
    bool transfer_computed = false;
    bool transfer_cold_valid = false;
    int cold_propagations = 0;
};

// Build a fresh simulation and the full subsystem bundle, run the requested
// fixture, and capture the resulting snapshot.
Snapshot run(DebugSubsystem mode) {
    Simulation sim;
    RecedingHorizonPredictor live;
    RecedingHorizonPredictor coast;
    NodeExecutor exec;
    TransferMidcourse mc;
    LandingAutopilot ap;
    AttitudeMode att = AttitudeMode::Off;
    TransferDebugResult tdbg{};
    LandingConfig lcfg{};
    ZeroEffortQuery zero_effort = [](double) { return BallisticState{}; };
    std::optional<ManeuverNode> node;

    DebugSubsystems subs{sim, exec, mc, ap, live, coast, lcfg, node, att,
                         &tdbg, zero_effort};
    setup_debug_scenario(mode, subs);

    Snapshot s;
    s.sim_time = sim.sim_time();
    s.st = sim.state();
    const auto readout = make_common_readout(sim);
    s.altitude = readout.altitude;
    s.tangential = readout.tangential_velocity;
    s.node_present = node.has_value();
    s.node_active = exec.active();
    s.transfer_active = mc.active();
    s.landing_armed = ap.armed();
    s.attitude = att;
    s.transfer_computed = tdbg.computed;
    s.transfer_cold_valid = tdbg.cold.valid;
    s.cold_propagations = tdbg.cold_propagations;
    return s;
}

// R8-02: the CLI selector parses, round-trips through name(), and rejects
// unknown / empty names (the latter maps to None, i.e. normal gameplay).
void test_selector() {
    const char* names[] = {"manual", "predictor", "attitude", "node-edit",
                           "node-executor", "transfer-cold", "transfer-warm",
                           "autoland-primary", "autoland-companion",
                           "autoland-cross", "ui"};
    for (const char* n : names) {
        const auto m = parse_debug_subsystem(n);
        check(m.has_value(), "valid selector parsed");
        check(m.has_value() && std::string(debug_subsystem_name(*m)) == n,
              "name round-trips");
        check(m.has_value() &&
                  !std::string(debug_subsystem_description(*m)).empty(),
              "description non-empty");
    }
    const auto none = parse_debug_subsystem("none");
    check(none.has_value() && *none == DebugSubsystem::None,
          "'none' maps to the None mode");
    check(!parse_debug_subsystem("").has_value(), "empty maps to nullopt");
    check(!parse_debug_subsystem("definitely-not-a-mode").has_value(),
          "unknown name maps to nullopt");
}

// R8-04: per-mode seeds are nonzero, unique across the eleven modes, and None
// maps to 0 (so an absent selector never seeds a debug scenario).
void test_seeds() {
    check(debug_scenario_seed(DebugSubsystem::None) == 0, "None seed is 0");
    const DebugSubsystem all[] = {DebugSubsystem::Manual, DebugSubsystem::Predictor,
                                  DebugSubsystem::Attitude, DebugSubsystem::NodeEdit,
                                  DebugSubsystem::NodeExecutor, DebugSubsystem::TransferCold,
                                  DebugSubsystem::TransferWarm,
                                  DebugSubsystem::AutolandPrimary,
                                  DebugSubsystem::AutolandCompanion,
                                  DebugSubsystem::AutolandCross,
                                  DebugSubsystem::Ui};
    std::set<std::uint64_t> seen;
    for (const auto m : all) {
        const std::uint64_t seed = debug_scenario_seed(m);
        check(seed != 0, "non-None seed is nonzero");
        check(seen.insert(seed).second, "seeds are unique per mode");
    }
    check(seen.size() == 11, "eleven distinct non-None seeds");
}

// R8-03: the common minimum readout is computed correctly for a fresh
// (landed-on-pad) simulation: sim time, reference / target body, flight state,
// and an on-surface altitude.
void test_common_readout_landed() {
    Simulation sim;
    sim.reset(12345);
    const auto c = make_common_readout(sim);
    check_close(c.sim_time, 0.0, 1e-9, "sim_time is 0 after reset");
    check(c.reference_body == 0, "reference body is the primary");
    check(c.target_body == 1, "contract target is the companion");
    check(c.landed, "fresh sim is landed");
    check(!c.crashed, "fresh sim is not crashed");
    check(c.altitude >= -1.0 && c.altitude <= 1.0, "on-surface altitude ~ 0");
    check(std::isfinite(c.x) && std::isfinite(c.y), "finite position");
}

// R8-05 + R8-06: each fixture places a deterministic scenario and arms exactly
// the one subsystem under test, leaving the rest in their cleared state.
void test_fixture_signatures() {
    // Manual / Ui / None: freshly reset, landed on the primary pad, nothing
    // else engaged.
    for (const auto m : {DebugSubsystem::Manual, DebugSubsystem::Ui,
                         DebugSubsystem::None}) {
        const auto s = run(m);
        check(s.st.landed, "landed on the pad");
        check(!s.node_present && !s.node_active, "no node / executor");
        check(!s.transfer_active, "no transfer");
        check(!s.landing_armed, "no landing");
        check(s.attitude == AttitudeMode::Off, "attitude off");
    }

    // Predictor: a clean orbit, no autonomous controls engaged (the live
    // predictor is driven by the main loop, not arming).
    {
        const auto s = run(DebugSubsystem::Predictor);
        check(!s.st.landed, "in flight");
        check(s.altitude > 1.0, "orbit altitude above the surface");
        check(!s.node_present && !s.node_active, "no node / executor");
        check(!s.transfer_active, "no transfer");
        check(!s.landing_armed, "no landing");
        check(s.attitude == AttitudeMode::Off, "attitude off");
    }

    // Attitude: a clean orbit with a deliberate prograde attitude demand.
    {
        const auto s = run(DebugSubsystem::Attitude);
        check(!s.st.landed, "in flight");
        check(s.attitude == AttitudeMode::Prograde, "prograde attitude demand");
        check(!s.node_present && !s.node_active, "no node / executor");
        check(!s.transfer_active, "no transfer");
        check(!s.landing_armed, "no landing");
    }

    // NodeEdit: a planned node exists, but the one-shot executor is NOT armed.
    {
        const auto s = run(DebugSubsystem::NodeEdit);
        check(!s.st.landed, "in flight");
        check(s.node_present, "a node is planned");
        check(!s.node_active, "executor is NOT armed in node-edit mode");
        check(!s.transfer_active, "no transfer");
        check(!s.landing_armed, "no landing");
        check(s.attitude == AttitudeMode::Off, "attitude off");
    }

    // NodeExecutor: the planned node AND the one-shot executor are armed, with
    // the maneuver attitude demand active.
    {
        const auto s = run(DebugSubsystem::NodeExecutor);
        check(!s.st.landed, "in flight");
        check(s.node_present, "a node is planned");
        check(s.node_active, "executor IS armed in node-executor mode");
        check(!s.transfer_active, "no transfer");
        check(!s.landing_armed, "no landing");
        check(s.attitude == AttitudeMode::Maneuver, "maneuver attitude active");
    }

    // TransferCold: a one-shot COLD solve runs and is recorded; the midcourse
    // controller is NOT armed (warm only) and landing is not armed.
    {
        const auto s = run(DebugSubsystem::TransferCold);
        check(!s.st.landed, "in flight");
        check(s.transfer_computed, "cold solve was run");
        check(s.cold_propagations >= 1, "cold solve used the propagation hook");
        check(!s.transfer_active, "cold mode does not arm the midcourse");
        check(!s.landing_armed, "no landing");
    }

    // TransferWarm: the cold solve seeds the cache and the two-level
    // midcourse is engaged (iff the cold seed solved).
    {
        const auto s = run(DebugSubsystem::TransferWarm);
        check(!s.st.landed, "in flight");
        check(s.transfer_computed, "cold seed solve was run");
        check(s.transfer_active == s.transfer_cold_valid,
              "midcourse engaged iff the cold seed solved");
        check(!s.landing_armed, "no landing");
    }

    // Autoland (primary / companion / cross): the landing autopilot is armed;
    // the one-shot executor and the transfer midcourse are not.
    for (const auto m : {DebugSubsystem::AutolandPrimary,
                         DebugSubsystem::AutolandCompanion,
                         DebugSubsystem::AutolandCross}) {
        const auto s = run(m);
        check(s.landing_armed, "landing autopilot is armed");
        check(!s.node_active, "one-shot executor not armed");
        check(!s.transfer_active, "transfer midcourse not armed");
    }
}

// R8-06: repeated setup for the same mode is fully deterministic (state and
// every arming flag match), covering the modes that arm a subsystem or place
// the ship in orbit.
void test_determinism() {
    const DebugSubsystem modes[] = {
        DebugSubsystem::Predictor, DebugSubsystem::Attitude,
        DebugSubsystem::NodeEdit,  DebugSubsystem::NodeExecutor,
        DebugSubsystem::TransferCold, DebugSubsystem::TransferWarm,
        DebugSubsystem::AutolandPrimary, DebugSubsystem::AutolandCompanion,
        DebugSubsystem::AutolandCross};
    for (const auto m : modes) {
        const auto a = run(m);
        const auto b = run(m);
        check_close(a.st.x, b.st.x, 1e-9, "det: x");
        check_close(a.st.y, b.st.y, 1e-9, "det: y");
        check_close(a.st.vx, b.st.vx, 1e-9, "det: vx");
        check_close(a.st.vy, b.st.vy, 1e-9, "det: vy");
        check_close(a.st.angle, b.st.angle, 1e-12, "det: angle");
        check(a.st.landed == b.st.landed, "det: landed");
        check(a.node_present == b.node_present, "det: node_present");
        check(a.node_active == b.node_active, "det: node_active");
        check(a.transfer_active == b.transfer_active, "det: transfer_active");
        check(a.landing_armed == b.landing_armed, "det: landing_armed");
        check(a.attitude == b.attitude, "det: attitude");
        check(a.transfer_cold_valid == b.transfer_cold_valid,
               "det: cold_valid");
    }
}

// M06-R8 (expansion): the small read-only getters on the landing autopilot
// expose exactly the armed target, active laws, held command, and terminal
// preview that the expanded panel reads; they change no state.
void test_landing_debug_getters() {
    const ZeroEffortQuery zero = [](double) {
        return BallisticState{};
    };

    LandingAutopilot ap;
    check(!ap.armed(), "not armed initially");
    check(ap.target_body() == 0, "default target body");
    check(ap.source_body() < 0, "source unresolved before arming");
    check(!ap.high_energy() && !ap.target_disturbed(), "laws default off");
    check(!ap.command().valid, "held command invalid before any update");
    check(!ap.terminal_preview().feasible,
          "terminal preview infeasible before any update");

    ap.arm(1, LandingConfig{}, zero);
    check(ap.armed(), "armed");
    check(ap.target_body() == 1, "target_body() after arming");
    ap.abort();
    check(!ap.armed(), "aborted");
}

// M06-R8 (follow-up): the harness's single source of truth for the scene
// overlay. Only the predictor isolation keeps the predicted-trajectory overlay
// visible by default; every other mode (and normal gameplay / None) is
// unchanged -- none of them get the always-on predictor overlay.
void test_predictor_keeps_prediction_overlay() {
    check(keeps_prediction_overlay(DebugSubsystem::Predictor),
          "predictor keeps the prediction overlay");
    const DebugSubsystem others[] = {
        DebugSubsystem::None, DebugSubsystem::Manual, DebugSubsystem::Attitude,
        DebugSubsystem::NodeEdit, DebugSubsystem::NodeExecutor,
        DebugSubsystem::TransferCold, DebugSubsystem::TransferWarm,
        DebugSubsystem::AutolandPrimary, DebugSubsystem::AutolandCompanion,
        DebugSubsystem::AutolandCross, DebugSubsystem::Ui};
    for (const auto m : others) {
        check(!keeps_prediction_overlay(m), "other mode does not force it");
    }
}

}  // namespace

int main() {
    test_selector();
    test_seeds();
    test_common_readout_landed();
    test_fixture_signatures();
    test_determinism();
    test_landing_debug_getters();
    test_predictor_keeps_prediction_overlay();

    if (failures == 0) {
        std::printf("All lander_debug_subsystem_tests passed\n");
        return 0;
    }
    std::printf("%d debug-subsystem test(s) failed\n", failures);
    return 1;
}
