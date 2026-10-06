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
#include "lander/debug_font.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <set>
#include <string>

namespace {

using lander::AttitudeDebugAxes;
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
using lander::Vec2;
using lander::altitude_at;
using lander::attitude_debug_axes;
using lander::thrust_hat;
using lander::ZeroEffortQuery;
using lander::debug_scenario_seed;
using lander::debug_subsystem_description;
using lander::debug_subsystem_name;
using lander::keeps_prediction_overlay;
using lander::make_common_readout;
using lander::parse_debug_subsystem;
using lander::setup_debug_scenario;

// M06-R15: the pure node-edit geometry helpers under test.
using lander::NodeBasis;
using lander::NodeEditArrows;

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

void check_close_vec(const Vec2& a, const Vec2& b, double eps,
                     const char* message) {
    if (std::hypot(a.x - b.x, a.y - b.y) > eps) {
        ++failures;
        std::printf("FAIL: %s ((%.9g, %.9g) != (%.9g, %.9g))\n", message,
                    a.x, a.y, b.x, b.y);
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

// M06-R11 (FIX 1): the embedded 5x7 debug font's character coverage.
// glyph() in gui.cpp delegates to lander::debug_font, so proving that every
// character the diagnostic panels print is "visible" proves those strings
// actually render (lowercase and square brackets no longer silently fall
// through to blank space). Runs headless: no SDL, no GUI build.
void test_debug_font_coverage() {
    // normalize(): lowercase -> uppercase; everything else passes through.
    check(lander::debug_font::normalize('a') == 'A', "normalize a->A");
    check(lander::debug_font::normalize('z') == 'Z', "normalize z->Z");
    check(lander::debug_font::normalize('A') == 'A', "normalize A->A");
    check(lander::debug_font::normalize('5') == '5', "normalize digit");
    check(lander::debug_font::normalize('-') == '-', "normalize punct");

    // visible(): every supported character (including lowercase via
    // normalize) renders; every clearly-unsupported one does not (the old
    // silent-blank failure mode).
    const char supported[] = {' ', 'A', 'Z', '0', '9', '-', '+', '.', ':',
                              '/', '=', '%', '!', '(', ')', ',', '[', ']',
                              'a', 'z'};
    for (const char c : supported) {
        check(lander::debug_font::visible(c), "visible: supported char");
    }
    const char unsupported[] = {'#', '{', '}', '@', '?', '<', '>', '\\',
                                '|', '~', '^', '"', '\'', '`'};
    for (const char c : unsupported) {
        check(!lander::debug_font::visible(c), "not visible: unsupported");
    }

    // The exact label strings the debug panels print (M06-R11 FIX 3) must
    // consist entirely of renderable characters, or a human reading the
    // panel would see missing glyphs.
    const char* labels[] = {
        "frames: W=inertial(world) R=ref-surface B=body-centre(rot)",
        "B VR/VT (body-centre frame)",
        "B V-REL (vs target body centre)",
        "R-ALT", "B V-REL", "MIN R", "MAX R", "CLR-PT",
        "(ref body, W)", "(ref-pt clearance, both bodies)", "(no forecast)",
        "ETA", "(not primed)", "primed", "avail", "SAMPLES", "HORIZON",
        "CP %s T%+.0fs", "ROLLING MODE", "PRED", "LAND", "CRASH"};
    for (const char* s : labels) {
        for (const char* p = s; *p; ++p) {
            check(lander::debug_font::visible(*p), "label char is renderable");
        }
    }
}

// M06-R11 (FIX 2): the common readout's altitude is measured against the
// SELECTED reference body's own terrain and tidal rotation, matches an
// independent altitude_at call, and stays signed (a sub-surface reading is
// negative, never clamped to zero or blanked).
void test_body_relative_altitude() {
    // In flight near the primary (unstepped): the reference body stays 0, so
    // the readout's altitude must equal an independent altitude_at against
    // body 0's terrain + rotation at t=0, and be positive (orbit above the
    // surface).
    {
        Simulation sim;
        sim.reset(777);
        const auto& bin = sim.binary();
        const lander::Body& b = bin.body(0);
        const double r = b.terrain.max_surface_radius() + 20.0;
        const Vec2 p0 = bin.position(0, 0.0);
        const Vec2 v0 = bin.velocity(0, 0.0);
        const double speed = std::sqrt(b.mu / r);
        State st{};
        st.x = p0.x;
        st.y = p0.y + r;
        st.vx = v0.x + speed;
        st.vy = v0.y;
        st.fuel = sim.config().fuel;
        sim.set_state(st);

        const auto c = make_common_readout(sim);
        check(c.reference_body == 0, "readout reference body is the primary");
        check(c.reference_label == "PRIMARY", "reference_label is PRIMARY");
        const double independent =
            altitude_at(b.terrain, sim.state(), p0, bin.body_rotation(0.0));
        check_close(c.altitude, independent, 1e-9, "altitude == independent");
        check(c.altitude >= 20.0 - 1e-6, "orbit clears the surface by ~20 m");
        check(std::isfinite(c.radial_velocity) &&
                  std::isfinite(c.tangential_velocity),
              "finite body-centre velocities");
    }

    // Deep inside the primary body: the altitude must be NEGATIVE (signed),
    // proving the sub-surface case is not clamped to zero or blanked.
    {
        Simulation sim;
        sim.reset(888);
        const auto& bin = sim.binary();
        const Vec2 p0 = bin.position(0, 0.0);
        State st{};
        st.x = p0.x;
        st.y = p0.y + 1.0;  // 1 m from the centre: deep inside the terrain
        sim.set_state(st);

        const auto c = make_common_readout(sim);
        const double independent =
            altitude_at(bin.body(0).terrain, sim.state(), p0,
                        bin.body_rotation(0.0));
        check_close(c.altitude, independent, 1e-9, "sub-surface altitude ==");
        check(c.altitude < 0.0, "sub-surface altitude is negative (signed)");
        check(c.reference_body == 0, "primary reference when not stepped");
    }
}

// A minimal owner of the live subsystems so a fixture's effect can be
// inspected against the simulation's binary (kept alive for each test).
struct Harness {
    Simulation sim{};
    RecedingHorizonPredictor live{};
    RecedingHorizonPredictor coast{};
    NodeExecutor exec{};
    TransferMidcourse mc{};
    LandingAutopilot ap{};
    AttitudeMode att = AttitudeMode::Off;
    TransferDebugResult tdbg{};
    LandingConfig lcfg{};
    ZeroEffortQuery ze = [](double) { return BallisticState{}; };
    std::optional<ManeuverNode> node;

    void setup(DebugSubsystem m) {
        DebugSubsystems s{sim, exec, mc, ap, live, coast, lcfg, node, att,
                          &tdbg, ze};
        setup_debug_scenario(m, s);
    }
};

// M06-R11 (FIX 3): place_in_orbit uses the SELECTED body's own terrain and mu
// (not the primary's) for the orbit radius and circular speed, preserving the
// +20 m clearance and the due-north / tangential direction convention.
void test_orbit_fixture_primary() {
    Harness h;
    h.setup(DebugSubsystem::Predictor);  // place_in_orbit(sim, 0)
    const auto& bin = h.sim.binary();
    const lander::Body& b = bin.body(0);
    const double r = b.terrain.max_surface_radius() + 20.0;
    const Vec2 p0 = bin.position(0, 0.0);
    const Vec2 v0 = bin.velocity(0, 0.0);
    const double speed = std::sqrt(b.mu / r);
    const State st = h.sim.state();
    check_close(st.x, p0.x, 1e-9, "primary orbit x = body x");
    check_close(st.y, p0.y + r, 1e-9, "primary orbit y = body y + r");
    check_close(st.vx, v0.x + speed, 1e-9, "primary orbit vx = v0.x + circ");
    check_close(st.vy, v0.y, 1e-9, "primary orbit vy = v0.y");
    const double alt = altitude_at(b.terrain, st, p0, bin.body_rotation(0.0));
    check(alt > 0.0, "primary orbit clears its own surface");
}

// M06-R11 (FIX 3): the companion-orbit fixture uses the companion's own
// (much smaller) terrain radius and mu, not the primary's -- the old code
// produced a too-large radius and too-slow circular speed here.
void test_orbit_fixture_companion() {
    Harness h;
    h.setup(DebugSubsystem::AutolandCompanion);  // place_in_orbit(sim, 1)
    const auto& bin = h.sim.binary();
    const lander::Body& b = bin.body(1);
    const double r = b.terrain.max_surface_radius() + 20.0;
    const Vec2 p0 = bin.position(1, 0.0);
    const Vec2 v0 = bin.velocity(1, 0.0);
    const double speed = std::sqrt(b.mu / r);
    const State st = h.sim.state();
    check_close(st.x, p0.x, 1e-9, "companion orbit x = body x");
    check_close(st.y, p0.y + r, 1e-9, "companion orbit y = body y + r");
    check_close(st.vx, v0.x + speed, 1e-9, "companion orbit vx = v0.x + circ");
    check_close(st.vy, v0.y, 1e-9, "companion orbit vy = v0.y");
    const double alt = altitude_at(b.terrain, st, p0, bin.body_rotation(0.0));
    check(alt > 0.0, "companion orbit clears its own surface");
    check(r < 100.0, "companion orbit radius is on the companion's scale");
}

// M06-R14: the pure attitude-visualization geometry. The ACT axis must equal the
// canonical thrust_hat convention (angle 0 -> +y, positive -> CCW); the TGT axis
// is the normalized supplied direction; has_target reflects whether a usable
// target exists (nullopt or a zero-length vector both mean "no ray").
void test_attitude_debug_axes() {
    // thrust_hat: the canonical nose / thrust axis.
    check_close(thrust_hat(0.0).x, 0.0, 1e-12, "thrust_hat(0).x == 0");
    check_close(thrust_hat(0.0).y, 1.0, 1e-12, "thrust_hat(0).y == 1");
    for (const double a : {0.3, 1.1, -0.7, 2.5, 3.0}) {
        check_close(thrust_hat(a).x, -std::sin(a), 1e-12, "thrust_hat x");
        check_close(thrust_hat(a).y, std::cos(a), 1e-12, "thrust_hat y");
    }

    // A nonzero, non-unit target is normalized into a unit target_dir.
    {
        const double a = 0.5;
        const Vec2 target{3.0, -4.0};  // length 5
        const AttitudeDebugAxes ax = attitude_debug_axes(a, target);
        check(ax.has_target, "nonzero target -> has_target");
        check_close(ax.actual_dir.x, -std::sin(a), 1e-12, "actual x");
        check_close(ax.actual_dir.y, std::cos(a), 1e-12, "actual y");
        check_close(ax.target_dir.x, 0.6, 1e-12, "target x normalized (3/5)");
        check_close(ax.target_dir.y, -0.8, 1e-12, "target y normalized (-4/5)");
        const double tl = std::hypot(ax.target_dir.x, ax.target_dir.y);
        check_close(tl, 1.0, 1e-12, "target_dir is unit length");
    }

    // nullopt target -> no TGT ray, zero target_dir.
    {
        const AttitudeDebugAxes ax = attitude_debug_axes(0.2, std::nullopt);
        check(!ax.has_target, "nullopt -> !has_target");
        check_close(ax.target_dir.x, 0.0, 1e-12, "nullopt -> target x 0");
        check_close(ax.target_dir.y, 0.0, 1e-12, "nullopt -> target y 0");
        check_close(ax.actual_dir.y, std::cos(0.2), 1e-12, "nullopt actual y");
    }

    // A zero-length target is treated as "no target".
    {
        const AttitudeDebugAxes ax =
            attitude_debug_axes(0.9, Vec2{0.0, 0.0});
        check(!ax.has_target, "zero target -> !has_target");
    }
}

// M06-R15: the pure, headless node-edit geometry helpers that the on-scene
// overlay and its fixed-screen arrows are built from. These are the automated
// evidence that (V01) the arrows are fixed-length and camera-rotated, (V02) the
// PGR / RAD / DV arrows come straight from the exact prediction basis / dv_world
// (never recomputed), and (V03) the frame shift anchors a world point to the
// node epoch and is the identity for the world frame. (V06) they are pure.
void test_node_edit_debug_geometry() {
    const double kLen = 40.0;

    // V01a: cam_rotate_dir is a pure camera rotation R2D(cam_angle).
    {
        const double a = 0.6;
        const Vec2 e0{1.0, 0.0};
        const Vec2 r = lander::cam_rotate_dir(e0, a);
        check_close(r.x, std::cos(a), 1e-12, "rotated (1,0).x == cos(a)");
        check_close(r.y, std::sin(a), 1e-12, "rotated (1,0).y == sin(a)");
        const Vec2 r0 = lander::cam_rotate_dir(e0, 0.0);
        check_close(r0.x, 1.0, 1e-12, "identity cam leaves (1,0).x");
        check_close(r0.y, 0.0, 1e-12, "identity cam leaves (1,0).y");
    }

    // V01b: screen_arrow_tip returns a fixed-length, camera-rotated tip and
    // normalises a non-unit input direction.
    {
        const Vec2 anchor{100.0, 200.0};
        const double a = 0.7;
        const Vec2 tip = lander::screen_arrow_tip(anchor, Vec2{1.0, 0.0}, a,
                                                  kLen);
        check_close(std::hypot(tip.x - anchor.x, tip.y - anchor.y), kLen, 1e-9,
                    "screen_arrow_tip fixed length");
        check_close(tip.x - anchor.x, kLen * std::cos(a), 1e-9, "tip dir.x");
        check_close(tip.y - anchor.y, kLen * std::sin(a), 1e-9, "tip dir.y");

        const Vec2 tip2 =
            lander::screen_arrow_tip(anchor, Vec2{3.0, 4.0}, 0.0, kLen);
        check_close(std::hypot(tip2.x - anchor.x, tip2.y - anchor.y), kLen,
                    1e-9, "non-unit dir normalised to fixed length");
        check_close(tip2.x - anchor.x, kLen * 0.6, 1e-9, "norm dir.x (3/5)");
        // Screen y is flipped (to_screen_point): world (0,1) at cam 0 -> screen
        // (0,-1). So a world dir (0.6,0.8) maps to screen (0.6,-0.8).
        check_close(tip2.y - anchor.y, -kLen * 0.8, 1e-9,
                    "norm dir.y (4/5) [screen y flipped]");
    }

    // V01c: identity camera -> tip == anchor + length * dir, unchanged.
    {
        const Vec2 anchor{0.0, 0.0};
        const Vec2 tip =
            lander::screen_arrow_tip(anchor, Vec2{0.0, 1.0}, 0.0, 30.0);
        check_close(tip.x, 0.0, 1e-12, "identity up tip.x == 0");
        check_close(tip.y, -30.0, 1e-12, "identity up tip.y == -30 (y flip)");
    }

    // V02a: the PGR / RAD arrows match the supplied NodeBasis exactly.
    {
        const Vec2 anchor{50.0, 50.0};
        const NodeBasis basis{Vec2{1.0, 0.0}, Vec2{0.0, 1.0}};
        const NodeEditArrows ar =
            lander::node_edit_arrows(anchor, basis, Vec2{0.0, 0.0}, 0.0, kLen);
        check_close(
            std::hypot(ar.pgr_tip.x - anchor.x, ar.pgr_tip.y - anchor.y), kLen,
            1e-9, "PGR length == fixed");
        check_close(ar.pgr_tip.x - anchor.x, kLen, 1e-9, "PGR dir == prograde");
        check_close(ar.pgr_tip.y - anchor.y, 0.0, 1e-9, "PGR dir y == 0");
        check_close(ar.rad_tip.x - anchor.x, 0.0, 1e-9, "RAD dir x == 0");
        // radial_out (0,1) -> screen (0,-1) at cam 0 (screen y flipped).
        check_close(ar.rad_tip.y - anchor.y, -kLen, 1e-9,
                    "RAD dir == radial [screen y flipped]");
    }

    // V02b: the DV arrow is the exact dv_world (normalised direction), with
    // the reported magnitude == |dv_world|.
    {
        const Vec2 anchor{0.0, 0.0};
        const NodeBasis basis{Vec2{1.0, 0.0}, Vec2{0.0, 1.0}};
        const Vec2 dv{3.0, 4.0};  // |dv| = 5, unit (0.6, 0.8)
        const NodeEditArrows ar =
            lander::node_edit_arrows(anchor, basis, dv, 0.0, kLen);
        check(ar.dv_present, "nonzero dv -> dv_present");
        check_close(ar.dv_length_mps, 5.0, 1e-12, "dv magnitude == 5");
        check_close(ar.dv_tip.x, kLen * 0.6, 1e-9, "DV dir == norm dv.x");
        check_close(ar.dv_tip.y, -kLen * 0.8, 1e-9,
                    "DV dir == norm dv.y [screen y flipped]");
    }

    // V02c: a zero dv_world -> no DV arrow, but the magnitude is reported as 0
    // and the PGR / RAD arrows are still drawn.
    {
        const Vec2 anchor{0.0, 0.0};
        const NodeBasis basis{Vec2{1.0, 0.0}, Vec2{0.0, 1.0}};
        const NodeEditArrows ar =
            lander::node_edit_arrows(anchor, basis, Vec2{0.0, 0.0}, 0.0, kLen);
        check(!ar.dv_present, "zero dv -> !dv_present");
        check_close(ar.dv_length_mps, 0.0, 1e-12, "zero dv -> magnitude 0");
        check_close(std::hypot(ar.pgr_tip.x, ar.pgr_tip.y), kLen, 1e-9,
                    "PGR still drawn when dv == 0");
        check_close(std::hypot(ar.rad_tip.x, ar.rad_tip.y), kLen, 1e-9,
                    "RAD still drawn when dv == 0");
    }

    // V03: frame_shift_point anchors a world point to the given frame at the
    // node epoch, and is the identity for the world frame / a stationary anchor.
    {
        const Vec2 wp{100.0, 50.0};
        const Vec2 w =
            lander::frame_shift_point(wp, -1, Vec2{999, 999}, Vec2{0, 0});
        check(w == wp, "world frame shift is identity");

        const Vec2 body_at_t{10.0, 20.0};
        const Vec2 body_at_tnow{30.0, 40.0};
        const Vec2 b =
            lander::frame_shift_point(wp, 0, body_at_t, body_at_tnow);
        check_close(b.x, 100.0 - 10.0 + 30.0, 1e-12, "body shift x");
        check_close(b.y, 50.0 - 20.0 + 40.0, 1e-12, "body shift y");

        const Vec2 same{5.0, 5.0};
        check(lander::frame_shift_point(wp, 1, same, same) == wp,
              "stationary body anchor -> identity");
    }

    // V06: the helpers are pure -- they take const refs and cannot mutate.
    {
        const NodeBasis basis{Vec2{1.0, 0.0}, Vec2{0.0, 1.0}};
        const Vec2 dv{3.0, 4.0};
        const Vec2 anchor{7.0, 8.0};
        (void)lander::node_edit_arrows(anchor, basis, dv, 0.4, kLen);
        (void)lander::screen_arrow_tip(anchor, dv, 0.4, kLen);
        (void)lander::cam_rotate_dir(dv, 0.4);
        (void)lander::frame_shift_point(anchor, 0, dv, basis.prograde);
        (void)lander::node_edit_label_pos(anchor, dv, 2.0, 3.0);
        check(basis.prograde.x == 1.0 && basis.radial_out.y == 1.0,
              "NodeBasis unmutated");
        check(dv.x == 3.0 && dv.y == 4.0, "dv_world unmutated");
        check(anchor.x == 7.0 && anchor.y == 8.0, "anchor unmutated");
    }
}

// M06-R16 (V04): deterministic, non-colliding placement of the node-edit
// arrow labels. Each label sits `along` pixels beyond its arrow tip, further
// out along the arrow, offset perpendicular by `side`. For two nearly
// parallel arrows (the R15 defect: a mostly-prograde DV drawn on top of PGR)
// the same-side placements coincide, while the opposite-side placement moves
// the DV label clear of the PGR label. Perpendicular arrows stay distinct
// under the same-side convention. The helper is pure and deterministic and
// degrades to the tip for a zero-length arrow.
void test_node_edit_label_placement() {
    const Vec2 anchor{0.0, 0.0};

    // PGR and DV both point screen-right (+x at camera 0), fixed 40 px.
    const Vec2 tip_a{40.0, 0.0};
    const Vec2 tip_b{40.0, 0.0};
    const Vec2 a_side = lander::node_edit_label_pos(anchor, tip_a, 6.0, 7.0);
    const Vec2 b_side = lander::node_edit_label_pos(anchor, tip_b, 6.0, 7.0);
    check(a_side == b_side,
          "parallel arrows: same-side labels collide (the R15 defect)");
    // The label sits 6 px beyond the tip, further along the arrow, +7 px to
    // the left of the arrow travel (screen +y for a +x arrow).
    check_close(a_side.x, 46.0, 1e-9, "label extends along the arrow");
    check_close(a_side.y, 7.0, 1e-9, "label offset perpendicular to the arrow");

    // Flipping the side puts the DV label on the other side of its tip:
    // 14 px away from the PGR label, individually readable.
    const Vec2 b_other = lander::node_edit_label_pos(anchor, tip_b, 6.0, -7.0);
    check_close(std::hypot(b_other.x - a_side.x, b_other.y - a_side.y), 14.0,
                1e-9, "opposite-side label is 2*side from the same-side one");
    check_close(b_other.x, 46.0, 1e-9, "flipped label still extends along");
    check_close(b_other.y, -7.0, 1e-9, "flipped label on the other side");
    const Vec2 b_again = lander::node_edit_label_pos(anchor, tip_b, 6.0, -7.0);
    check(b_again == b_other, "label placement is deterministic");

    // RAD points screen-down (0,-1 at camera 0): its same-side label stays
    // well clear of the PGR label because the arrows are perpendicular.
    const Vec2 rad_tip{0.0, -40.0};
    const Vec2 rad_label = lander::node_edit_label_pos(anchor, rad_tip, 6.0,
                                                       7.0);
    check(std::hypot(rad_label.x - a_side.x, rad_label.y - a_side.y) > 10.0,
          "perpendicular arrow label stays clear of the PGR label");

    // A degenerate zero-length arrow falls back to the tip.
    const Vec2 degenerate =
        lander::node_edit_label_pos(anchor, anchor, 6.0, 7.0);
    check(degenerate == anchor, "zero-length arrow -> label at the tip");
}

// M06-R18 (V06): the pure, headless node-executor overlay geometry the
// `--debug-subsystem node-executor` scene visualization is built from:
// fixed on-screen ray length (pixels, camera-zoom independent),
// camera-rotated directions (reusing cam_rotate_dir / screen_arrow_tip),
// the ACT ray from thrust_hat(ship angle), the VGO ray from the executor's
// remaining delta-v (present only while its magnitude exceeds eps), and
// purity (same inputs -> same output).
void test_node_executor_overlay() {
    const Vec2 anchor{500.0, 300.0};
    const double kLen = 46.0;

    // Camera 0, ship angle 0: the thrust axis is world +y, which the
    // screen-y inversion maps to screen -y, so the ACT tip sits exactly
    // kLen pixels above the anchor.
    {
        const auto o = lander::node_executor_overlay(
            anchor, 0.0, Vec2{0.0, 3.0}, 0.0, kLen);
        check_close_vec(o.act_tip, Vec2{anchor.x, anchor.y - kLen}, 1e-12,
                        "camera 0: ACT tip is kLen along screen -y");
    }

    // Ship nose along world +x (angle -pi/2) with a +x VGO: at camera 0 both
    // rays point screen +x; a 90-degree camera rotation carries both to
    // screen +y (camera rotation only, never the map scale).
    {
        const auto o = lander::node_executor_overlay(anchor, -0.5 * M_PI,
                                                     Vec2{4.0, 0.0}, 0.0, kLen);
        check_close_vec(o.act_tip, Vec2{anchor.x + kLen, anchor.y}, 1e-12,
                        "ACT follows the thrust axis (world +x) at camera 0");
        check(o.vgo_present, "a non-negligible VGO is present");
        check_close(o.vgo_mps, 4.0, 1e-12, "the VGO magnitude is reported");
        check_close_vec(o.vgo_tip, o.act_tip, 1e-12,
                        "the VGO ray points along the remaining delta-v");

        const auto rc = lander::node_executor_overlay(anchor, -0.5 * M_PI,
                                                      Vec2{4.0, 0.0},
                                                      0.5 * M_PI, kLen);
        check_close_vec(rc.act_tip, Vec2{anchor.x, anchor.y + kLen}, 1e-12,
                        "camera rotation carries the ACT ray");
        check_close_vec(rc.vgo_tip, rc.act_tip, 1e-12,
                        "camera rotation carries the VGO ray");
    }

    // Fixed length: the same VGO direction at a different magnitude gives
    // the same tip (the ray encodes direction only).
    {
        const auto big = lander::node_executor_overlay(
            anchor, 0.0, Vec2{0.0, 3.0}, 0.0, kLen);
        const auto small = lander::node_executor_overlay(
            anchor, 0.0, Vec2{0.0, 0.4}, 0.0, kLen);
        check_close_vec(small.vgo_tip, big.vgo_tip, 1e-12,
                        "fixed length regardless of the VGO magnitude");
        check(small.vgo_present, "a 0.4 m/s VGO is still present");
    }

    // Zero or below-eps VGO: no ray, the tip falls back to the anchor, and
    // the magnitude is still reported.
    {
        const auto o = lander::node_executor_overlay(
            anchor, 0.3, Vec2{0.0, 0.0}, 0.0, kLen);
        check(!o.vgo_present, "zero VGO -> no VGO ray");
        check_close_vec(o.vgo_tip, anchor, 1e-12, "zero VGO -> tip == anchor");
        check_close(o.vgo_mps, 0.0, 1e-12, "zero VGO magnitude reported");
        const auto tiny = lander::node_executor_overlay(
            anchor, 0.3, Vec2{2.0e-4, 0.0}, 0.0, kLen);
        check(!tiny.vgo_present, "a VGO below eps -> no ray");
    }

    // Non-finite VGO: no ray, anchor fallback, and no NaN anywhere.
    {
        const auto o = lander::node_executor_overlay(
            anchor, 0.3, Vec2{std::nan(""), 0.0}, 0.0, kLen);
        check(!o.vgo_present, "non-finite VGO -> no ray");
        check_close_vec(o.vgo_tip, anchor, 1e-12,
                        "non-finite VGO -> tip == anchor");
        check(std::isfinite(o.act_tip.x) && std::isfinite(o.act_tip.y),
              "the ACT tip stays finite");
    }

    // Purity: identical inputs give identical geometry.
    {
        const auto a = lander::node_executor_overlay(
            anchor, 1.2, Vec2{2.0, -3.0}, 0.7, kLen);
        const auto b = lander::node_executor_overlay(
            anchor, 1.2, Vec2{2.0, -3.0}, 0.7, kLen);
        check(a.act_tip == b.act_tip && a.vgo_tip == b.vgo_tip &&
                  a.vgo_present == b.vgo_present && a.vgo_mps == b.vgo_mps,
              "same inputs -> same output");
    }
}

// M06-R17 (V02): the node-edit event state resolves the node's effective
// epoch from the CURRENT ship state with the exact rule predict_trajectory
// uses (max(t_now, snap(node.time, fixed_dt))), and agrees with the
// predictor's node anchor computed from the same state. These are the
// semantics the node-edit overlay now consumes at render cadence.
void test_node_event_overdue_matches_predictor() {
    Harness h;
    h.setup(DebugSubsystem::NodeEdit);
    const auto& bin = h.sim.binary();
    const auto& cfg = h.sim.config();
    const double t_now = h.sim.sim_time();
    const State ship = h.sim.state();
    check(h.node.has_value(), "node-edit fixture has a node");

    // Scheduled epoch in the past: the effective epoch must clamp to t_now
    // and the event state must be the ship's current state, unpropagated.
    const lander::ManeuverNode overdue = [&] {
        lander::ManeuverNode n = *h.node;
        n.time = lander::snap_time(t_now, cfg.fixed_dt) - 2.0 * cfg.fixed_dt;
        return n;
    }();
    const lander::NodeEventState ev =
        lander::node_event_state(bin, cfg, ship, t_now, overdue);

    check_close(ev.time, t_now, 1e-12, "overdue node clamps to t_now");
    check_close(ev.state.p.x, ship.x, 1e-9, "overdue event x == ship x");
    check_close(ev.state.p.y, ship.y, 1e-9, "overdue event y == ship y");
    check_close(ev.state.v.x, ship.vx, 1e-9, "overdue event vx == ship vx");
    check_close(ev.state.v.y, ship.vy, 1e-9, "overdue event vy == ship vy");

    // The same rule predict_trajectory applies from the same state.
    const lander::TrajectoryPrediction pred = lander::predict_trajectory(
        bin, cfg, ship, t_now, h.sim.reference_body(),
        h.sim.contract().destination_body, overdue, 2.0 * bin.period(), 512);
    check_close(pred.node_time_effective, t_now, 1e-12,
                "predictor clamps the same node to t_now");
    check_close(pred.node_position.x, ev.state.p.x, 1e-9,
                "event position == predictor node position (x)");
    check_close(pred.node_position.y, ev.state.p.y, 1e-9,
                "event position == predictor node position (y)");
    check(ev.basis_valid == pred.basis_valid, "basis validity agrees");
    check_close(ev.basis.prograde.x, pred.basis.prograde.x, 1e-9,
                "prograde.x agrees");
    check_close(ev.basis.prograde.y, pred.basis.prograde.y, 1e-9,
                "prograde.y agrees");
    check_close(ev.basis.radial_out.x, pred.basis.radial_out.x, 1e-9,
                "radial.x agrees");
    check_close(ev.basis.radial_out.y, pred.basis.radial_out.y, 1e-9,
                "radial.y agrees");
    check_close(ev.dv_world.x, pred.dv_world.x, 1e-9, "dv_world.x agrees");
    check_close(ev.dv_world.y, pred.dv_world.y, 1e-9, "dv_world.y agrees");
    check_close(ev.total_dv, pred.total_dv, 1e-9, "total_dv agrees");
    // The fixture node carries a 0.5 m/s prograde delta-v.
    check(ev.total_dv > 0.4 && ev.total_dv < 0.6,
          "overdue event reports the node delta-v magnitude");
}

void test_node_event_future_matches_predictor() {
    Harness h;
    h.setup(DebugSubsystem::NodeEdit);
    const auto& bin = h.sim.binary();
    const auto& cfg = h.sim.config();
    const double t_now = h.sim.sim_time();
    const State ship = h.sim.state();
    check(h.node.has_value(), "node-edit fixture has a node");
    const lander::ManeuverNode future = *h.node;  // t0 + 5 s, ahead of now
    check(future.time > t_now, "fixture node is scheduled ahead of now");

    const lander::NodeEventState ev =
        lander::node_event_state(bin, cfg, ship, t_now, future);

    check_close(ev.time, lander::snap_time(future.time, cfg.fixed_dt), 1e-12,
                "future node keeps its snapped scheduled epoch");
    check(ev.time > t_now, "future event epoch is ahead of now");

    const lander::TrajectoryPrediction pred = lander::predict_trajectory(
        bin, cfg, ship, t_now, h.sim.reference_body(),
        h.sim.contract().destination_body, future, 2.0 * bin.period(), 512);
    check_close(pred.node_time_effective, ev.time, 1e-12,
                "event epoch == predictor node epoch");
    check_close(pred.node_position.x, ev.state.p.x, 1e-9,
                "event position == predictor node position (x)");
    check_close(pred.node_position.y, ev.state.p.y, 1e-9,
                "event position == predictor node position (y)");
    check(ev.basis_valid == pred.basis_valid, "basis validity agrees");
    check_close(ev.basis.prograde.x, pred.basis.prograde.x, 1e-9,
                "prograde.x agrees");
    check_close(ev.basis.prograde.y, pred.basis.prograde.y, 1e-9,
                "prograde.y agrees");
    check_close(ev.basis.radial_out.x, pred.basis.radial_out.x, 1e-9,
                "radial.x agrees");
    check_close(ev.basis.radial_out.y, pred.basis.radial_out.y, 1e-9,
                "radial.y agrees");
    check_close(ev.dv_world.x, pred.dv_world.x, 1e-9, "dv_world.x agrees");
    check_close(ev.dv_world.y, pred.dv_world.y, 1e-9, "dv_world.y agrees");
    check_close(ev.total_dv, pred.total_dv, 1e-9, "total_dv agrees");
    // The future event state must differ from the ship's current state
    // (it was propagated, not copied).
    const double moved =
        std::hypot(ev.state.p.x - ship.x, ev.state.p.y - ship.y);
    check(moved > 1.0, "future event state was propagated away from now");
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
    test_debug_font_coverage();
    test_body_relative_altitude();
    test_orbit_fixture_primary();
    test_orbit_fixture_companion();
    test_attitude_debug_axes();
    test_node_edit_debug_geometry();
    test_node_edit_label_placement();
    test_node_executor_overlay();
    test_node_event_overdue_matches_predictor();
    test_node_event_future_matches_predictor();

    if (failures == 0) {
        std::printf("All lander_debug_subsystem_tests passed\n");
        return 0;
    }
    std::printf("%d debug-subsystem test(s) failed\n", failures);
    return 1;
}
