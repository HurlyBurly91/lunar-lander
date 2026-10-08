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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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

// ---------------------------------------------------------------------------
// M06-R14 -- attitude debug visualization (pure geometry, headless-testable).
//
// These helpers describe the compact attitude overlay drawn ONLY in the
// `--debug-subsystem attitude` isolation. They are pure: they take the ship's
// current attitude angle and the already-resolved target direction and return
// the two screen-independent unit direction vectors to draw. No simulation,
// camera, or SDL is involved, so the ray math can be regression-tested
// headlessly. They deliberately reuse the canonical attitude convention and the
// same target direction the flight computer / panel use -- they never compute
// an attitude target of their own.
//
// Reference: docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-
// be-gained-node-execution.md (thrust_hat / nose convention).
struct AttitudeDebugAxes {
    Vec2 actual_dir{};        // unit: ACTUAL thrust axis = thrust_hat(angle)
    Vec2 target_dir{};        // unit: TARGET direction (zero when no target)
    bool has_target = false;  // false => no target ray should be drawn
};

// The canonical thrust / nose axis for an attitude angle: angle 0 points along
// +y and positive angles rotate counter-clockwise, matching the drawn ship
// nose, the system-view marker triangle, and the autopilot's thrust
// application. thrust_hat(a) == (-sin a, cos a).
inline Vec2 thrust_hat(double angle) {
    return Vec2{-std::sin(angle), std::cos(angle)};
}

// Build the two unit direction vectors for the attitude overlay from the
// ship's attitude angle and the already-resolved (optional) target direction.
// The target is normalized here; a missing (nullopt) or zero-length target
// yields has_target == false with a zero target_dir (draw no TGT ray).
inline AttitudeDebugAxes attitude_debug_axes(double actual_angle,
                                             const std::optional<Vec2>& target) {
    AttitudeDebugAxes axes;
    axes.actual_dir = thrust_hat(actual_angle);
    if (target.has_value()) {
        const double len = std::hypot(target->x, target->y);
        if (len > 1.0e-12) {
            axes.target_dir = Vec2{target->x / len, target->y / len};
            axes.has_target = true;
        }
    }
    return axes;
}

// BEGIN CANONICAL ALGORITHM: node-edit debug display geometry
// Reference: docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-
// be-gained-node-execution.md (node / VGO / basis semantics that the overlay
// visualizes); docs/physics-model-gravity.md (physics stays WORLD/INERTIAL).
//
// The following are READ-ONLY, presentation-only geometry helpers used ONLY by
// the `--debug-subsystem node-edit` isolation. They take plain values (no
// Simulation / ManeuverNode / TrajectoryPrediction), so they never mutate game
// state and are fully headless-testable. The arrow directions are expressed in
// the camera-rotated DISPLAY space (the same rotation `to_screen_point` applies
// to world vectors), so the rendered arrows keep a FIXED on-screen length
// regardless of camera zoom, and their orientation matches how the surrounding
// PRE/POST trajectory arc is drawn in the selected frame.

// The camera-rotated (screen-space) unit direction for a world/display unit
// vector `d` at camera roll `camera_angle`. Mirrors the rotation inside
// `to_screen_point`: local_x = dx*c + dy*s; local_y = -dx*s + dy*c; and the
// screen y-axis is inverted, so the on-screen direction is
// (dx*c + dy*s, dx*s - dy*c). Pure; no mutation.
inline Vec2 cam_rotate_dir(const Vec2& d, double camera_angle) {
    const double c = std::cos(camera_angle);
    const double s = std::sin(camera_angle);
    return Vec2{d.x * c + d.y * s, d.x * s - d.y * c};
}

// The tip (in screen pixel coordinates) of a fixed-length arrow drawn from
// `anchor` along the camera-rotated direction `d`. `length_px` is in pixels
// (camera-zoom independent); a non-finite direction yields `anchor`. Pure.
inline Vec2 screen_arrow_tip(const Vec2& anchor, const Vec2& d,
                             double camera_angle, double length_px) {
    const double norm = std::hypot(d.x, d.y);
    if (!(norm > 0.0) || !std::isfinite(norm)) return anchor;
    const Vec2 dir = Vec2{d.x / norm, d.y / norm};
    const Vec2 rot = cam_rotate_dir(dir, camera_angle);
    return Vec2{anchor.x + rot.x * length_px, anchor.y + rot.y * length_px};
}

// The fixed-screen arrow endpoints for the node-edit overlay, all measured from
// the same on-screen anchor. PGR/RAD come from the supplied NodeBasis (used
// verbatim, never recomputed); the DV ray comes from the supplied dv_world and
// is only present when its magnitude exceeds `eps` (the magnitude is always
// reported via dv_length_mps). All tips are a fixed `length_px` from `anchor`.
struct NodeEditArrows {
    Vec2 pgr_tip{};
    Vec2 rad_tip{};
    Vec2 dv_tip{};
    bool dv_present = false;
    double dv_length_mps = 0.0;
};

inline NodeEditArrows node_edit_arrows(const Vec2& anchor, const NodeBasis& basis,
                                       const Vec2& dv_world, double camera_angle,
                                       double length_px, double eps = 1.0e-6) {
    NodeEditArrows a;
    a.pgr_tip = screen_arrow_tip(anchor, basis.prograde, camera_angle, length_px);
    a.rad_tip = screen_arrow_tip(anchor, basis.radial_out, camera_angle, length_px);
    const double dv_mag = std::hypot(dv_world.x, dv_world.y);
    a.dv_length_mps = dv_mag;
    if (dv_mag > eps) {
        a.dv_present = true;
        a.dv_tip = screen_arrow_tip(anchor, dv_world, camera_angle, length_px);
    } else {
        a.dv_present = false;
        a.dv_tip = anchor;
    }
    return a;
}

// The display-frame translation of a WORLD position `wp` for the given frame:
// identity for the world frame (frame_body < 0), otherwise
// wp - body_at_t + body_at_tnow (the same shift `pred_frame` / draw_trajectory
// apply). `body_at_t` is the frame body's position at the feature's epoch and
// `body_at_tnow` at the render time; both are supplied so this stays pure.
inline Vec2 frame_shift_point(const Vec2& wp, int frame_body,
                               const Vec2& body_at_t, const Vec2& body_at_tnow) {
    if (frame_body < 0) return wp;
    return wp - body_at_t + body_at_tnow;
}

// M06-R16: deterministic screen-space position for a node-edit arrow label.
// The label sits `along` pixels beyond the arrow tip, further out along the
// arrow, then `side` pixels perpendicular to it (positive = left of the arrow
// travel, negative = the other side). Two nearly-parallel arrows (e.g. PGR and
// a mostly-prograde DV) therefore keep their labels on opposite sides and stay
// individually readable. Pure: same inputs -> same output, so the placement is
// headless-testable. A degenerate (zero-length) arrow returns the tip.
inline Vec2 node_edit_label_pos(const Vec2& anchor, const Vec2& tip,
                                double along, double side) {
    const double dx = tip.x - anchor.x;
    const double dy = tip.y - anchor.y;
    const double len = std::hypot(dx, dy);
    if (!(len > 1.0e-9)) return tip;
    const double ux = dx / len;
    const double uy = dy / len;
    return Vec2{tip.x + ux * along - uy * side, tip.y + uy * along + ux * side};
}

// END CANONICAL ALGORITHM: node-edit debug display geometry

// BEGIN CANONICAL ALGORITHM: node-executor debug display geometry
// Reference: docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-
// be-gained-node-execution.md (VGO / thrust-axis semantics the overlay
// visualizes); docs/physics-model-gravity.md (the VGO stays WORLD/INERTIAL).
//
// M06-R18: READ-ONLY, presentation-only geometry for the
// `--debug-subsystem node-executor` isolation. Same display conventions as
// the node-edit overlay: fixed on-screen length (pixels, camera-zoom
// independent) and camera-rotated directions (reusing `cam_rotate_dir` /
// `screen_arrow_tip`), anchored at the drawn ship. The ACT ray is the ship's
// canonical thrust axis `thrust_hat(angle)`; the VGO ray is the executor's
// remaining delta-v `dv_remaining()` (world frame) and is present only while
// its magnitude exceeds `eps` (the magnitude is always reported via
// vgo_mps). Pure: no simulation, camera, or SDL; same inputs -> same output.

struct NodeExecutorOverlay {
    Vec2 act_tip{};      // tip of the fixed-length ACT (thrust-axis) ray
    Vec2 vgo_tip{};      // tip of the fixed-length VGO ray (== anchor when absent)
    bool vgo_present = false;
    double vgo_mps = 0.0;  // remaining VGO magnitude, always reported
};

inline NodeExecutorOverlay node_executor_overlay(const Vec2& anchor,
                                                 double ship_angle,
                                                 const Vec2& dv_remaining,
                                                 double camera_angle,
                                                 double length_px,
                                                 double eps = 1.0e-3) {
    NodeExecutorOverlay o;
    o.act_tip = screen_arrow_tip(anchor, thrust_hat(ship_angle),
                                 camera_angle, length_px);
    o.vgo_mps = std::hypot(dv_remaining.x, dv_remaining.y);
    if (o.vgo_mps > eps) {
        o.vgo_present = true;
        o.vgo_tip = screen_arrow_tip(anchor, dv_remaining,
                                     camera_angle, length_px);
    } else {
        o.vgo_present = false;
        o.vgo_tip = anchor;
    }
    return o;
}

// END CANONICAL ALGORITHM: node-executor debug display geometry

// BEGIN CANONICAL ALGORITHM: inter-moon transfer cold display geometry
// Reference: docs/flight-guidance-intermoon-transfer-differential-correction-
// warm-starting-and-bounded-replanning.md (the accepted COLD solution being
// displayed); docs/physics-model-gravity.md (the arc stays WORLD/INERTIAL).
//
// M06-R20 / M06-R20-F01: READ-ONLY, presentation-only helpers for the
// `--debug-subsystem transfer-cold` isolation. They turn an already-accepted
// `TransferSolution` into an explicit inertial temporal inspection scene: a
// frozen zero-thrust display arc, the source body's local surface at the solve
// epoch, the target body's local surface at the arrival epoch, the solver's
// arrival-shell target when available, and an aspect-preserving debug camera
// fit. They never re-solve, never mutate the solution / binary / simulation,
// and are not used by normal gameplay.

struct TransferColdDisplay {
    bool valid = false;
    int source = -1;
    int target = -1;
    std::vector<BallisticState> arc;
    Vec2 dep{};
    Vec2 arr{};
    Vec2 target_at_arrival{};
    Vec2 source_at_solve{};
    bool target_point_valid = false;
    Vec2 target_point{};
    std::vector<Vec2> source_outline{};
    std::vector<Vec2> target_outline{};
    double source_rotation = 0.0;
    double target_rotation = 0.0;
    Vec2 raw_center{};
    Vec2 raw_half{};
    Vec2 fit_center{};
    Vec2 fit_half{};
    double solve_epoch = 0.0;
    double arrival_epoch = 0.0;
};

struct TransferCameraFit {
    Vec2 center{};
    double angle = 0.0;
    double zoom = 0.0;
};

TransferColdDisplay transfer_cold_display(const BinarySystem& bin,
                                          const TransferSolution& sol,
                                          double fixed_dt, int samples);

TransferCameraFit transfer_cold_camera_fit(
    const Vec2& center, const Vec2& half, double window_width,
    double window_height, double base_scale, double min_zoom,
    double max_zoom);

// END CANONICAL ALGORITHM: inter-moon transfer cold display geometry

}  // namespace lander
