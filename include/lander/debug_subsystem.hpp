#pragma once

// M06-R8: subsystem-isolation debug harness.
//
// This module backs the `--debug-subsystem <name>` developer option in the
// graphical game. It lets a developer bring up the game with exactly one of
// the 11 M06 subsystems isolated: a focused panel replaces the normal
// flight-computer panel, the unrelated planning / nav / prediction / banner
// surfaces are hidden, and a deterministic startup-only fixture places the ship
// in a starting condition that exercises the selected subsystem without hidden
// forces. It is a read-only diagnostic harness: it must never change normal
// gameplay, canonical physics, or fixed_dt, and it must never fix subsystem
// defects (it only surfaces them).

#include "lander/autopilot.hpp"     // NodeExecutor, TransferMidcourse, AttitudeMode
#include "lander/ballistic.hpp"     // BallisticState, transfer solves
#include "lander/flight_computer.hpp"  // ManeuverNode, NodeBasis, transfer planning
#include "lander/landing.hpp"       // LandingAutopilot, LandingConfig, ZeroEffortQuery
#include "lander/predictor.hpp"     // RecedingHorizonPredictor, FlightPolicy
#include "lander/sim.hpp"           // Simulation, State, Config

#include <cstdint>
#include <optional>
#include <string>

namespace lander {

// The 11 subsystem-isolation debug modes. `None` (the default, and the value
// for an absent or "none" selector) runs ordinary gameplay with no debug
// surface at all; every other value isolates exactly one M06 subsystem for
// inspection.
enum class DebugSubsystem {
    None,
    Manual,
    Predictor,
    Attitude,
    NodeEdit,
    NodeExecutor,
    TransferCold,
    TransferWarm,
    AutolandPrimary,
    AutolandCompanion,
    AutolandCross,
    Ui,
};

// Parse a `--debug-subsystem` selector. Returns the matching mode for a valid
// name (including "none" -> None) and std::nullopt for an unrecognized name
// (the caller reports the error and exits non-zero before the window opens).
std::optional<DebugSubsystem> parse_debug_subsystem(const std::string& name);

// Stable display name for a mode (e.g. "predictor", "autoland-cross").
const char* debug_subsystem_name(DebugSubsystem mode);

// One-line human description of what a mode isolates (used in the panel
// header and the usage / help text).
const char* debug_subsystem_description(DebugSubsystem mode);

// True when this isolation mode keeps the scene's prediction / trajectory
// overlay drawn (the mode's primary graphical observable: the COAST / LIVE /
// PLAN projected arcs plus their PE / AP / closest-approach markers and the
// kind legend). Only the predictor isolation does; every other mode is
// unchanged (normal gameplay and `ui` gate the overlay on the G toggle,
// node-edit keeps its own G-gated pre/post arc, the rest draw none). This is
// the single source of truth the GUI render loop consults, so the harness
// decision is unit-testable headlessly.
bool keeps_prediction_overlay(DebugSubsystem mode);

// The common minimum readout shared by every mode's panel (M06-R8-02). Every
// field is derived from the live simulation only; the mode's subsystem name is
// rendered separately by the panel.
struct DebugCommonReadout {
    double sim_time{};
    double x{};
    double y{};
    int reference_body{};
    int target_body{};
    std::string reference_label;  // "PRIMARY" / "COMPANION" / "BODY<n>" / "NONE"
    std::string target_label;
    bool landed{false};
    bool crashed{false};
    double altitude{};         // signed m, above the reference body's terrain
    double radial_velocity{};  // m/s, + away from the reference body centre
    double tangential_velocity{};  // m/s, body-centre frame
    double relative_speed{};   // m/s, vs the contract-destination body centre
};
DebugCommonReadout make_common_readout(const Simulation& sim);

// Make a readout's reference / target body identity explicit instead of a bare
// index: "PRIMARY" for 0, "COMPANION" for 1, "BODY<n>" otherwise, "NONE" for an
// unset (-1) slot. Header-only so the headless tests can assert on it directly.
inline std::string debug_body_label(int index) {
    if (index == 0) return "PRIMARY";
    if (index == 1) return "COMPANION";
    if (index == 2) return "MOONLET";  // M06-R13 outer moonlet
    if (index < 0) return "NONE";
    return "BODY" + std::to_string(index);
}

// A fixed, per-mode scenario seed so each mode's startup fixture is fully
// reproducible and independent of `--seed` (M06-R8-04 / D04). `None` returns 0
// (the caller uses the normal seed for None).
std::uint64_t debug_scenario_seed(DebugSubsystem mode);

// M06-R13: parse a `--debug-predictor-body 0|1|2` selector for the predictor
// isolation's startup fixture. Returns the body index (0 primary, 1 companion,
// 2 outer moonlet) and std::nullopt for any value outside [0, 2].
std::optional<int> parse_debug_predictor_body(const std::string& value);

// Observation of the (one-shot) COLD inter-moon transfer solve used by the
// transfer-cold and transfer-warm debug modes. The COLD solve is the seed for
// the WARM midcourse in transfer-warm; the live WARM solve is read directly
// from the midcourse's cache by the panel.
struct TransferDebugResult {
    bool computed{false};    // a COLD solve was attempted in this startup fixture
    bool warm{false};        // true for transfer-warm, false for transfer-cold
    int source{};
    int target{};
    TransferSolution cold{};  // the COLD solve (the seed for the WARM midcourse)
    int cold_propagations{};  // zero-thrust propagations the COLD solve used
    double cold_solve_ms{};   // wall time of the one-shot COLD solve
    // Live WARM re-plan observations, filled by the debug panel's caller in
    // transfer-warm mode only (the last bounded-rate re-plan that ran). All
    // zero / false until a re-plan has happened. Diagnostic state only: it
    // never feeds back into the midcourse.
    double warm_miss_before{};
    double warm_miss_after{};
    int warm_propagations_last{};
    int warm_propagations_total{};
    bool warm_fallback_last{false};  // last re-plan used the COLD fallback
};

// Bundle of the live subsystems (owned by gui.cpp's main) that a debug scenario
// fixture may reset, arm, or solve. All members are references into the caller.
struct DebugSubsystems {
    Simulation& sim;
    NodeExecutor& node_executor;
    TransferMidcourse& transfer_mc;
    LandingAutopilot& landing_ap;
    RecedingHorizonPredictor& live_predictor;
    RecedingHorizonPredictor& landing_coast;
    const LandingConfig& landing_cfg;
    std::optional<ManeuverNode>& maneuver_node;
    AttitudeMode& attitude_mode;
    TransferDebugResult* transfer_debug;  // non-null for transfer modes
    const ZeroEffortQuery& zero_effort;   // landing zero-effort source
};

// Startup-only scenario fixture (M06-R8-04 / D04). For `mode == None` this is a
// no-op. Otherwise it re-seeds the simulation with the per-mode seed, places
// the ship into that mode's deterministic in-flight or landed starting
// condition, and arms exactly the one subsystem under test (or, for
// transfer-cold / transfer-warm, runs the one-shot transfer solve and stores
// the result in `transfer_debug`). It never applies hidden forces: from here
// on the armed subsystem runs through the normal simulation / control paths,
// which is what lets a developer watch a subsystem defect (TFD-1 / TFD-2) or
// healthy behavior emerge.
//
// M06-R13: `predictor_body` selects which body the predictor isolation's
// startup fixture orbits (0 primary [default], 1 companion, 2 outer moonlet).
// It is consulted only in `Predictor` mode; every other mode ignores it.
void setup_debug_scenario(DebugSubsystem mode, DebugSubsystems& s,
                          int predictor_body = 0);

}  // namespace lander
