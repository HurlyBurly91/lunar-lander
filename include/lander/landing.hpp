#pragma once

// M06-R6: low-complexity powered-landing guidance. The high-level phase state
// machine chooses a target state; the low-level terminal law is always the same
// classical ZEM/ZEV acceleration command aimed at that moving target. The
// command is a physical thrust correction (the zero-effort reference already
// includes the real gravity evolution, so no gravity is added a second time) and
// never mutates state directly. The zero-effort state is injected as a query so
// this module stays decoupled from the receding-horizon predictor (which
// includes this module's parent, autopilot); in production the query is the R3
// predictor's O(1) maintained lookup, never a fresh long-horizon integration
// inside the guidance call (M06-R3 / R6-03, R6-09).
//
// See the canonical document for the full contract; the implemented law is
// marked with canonical algorithm regions in src/landing.cpp.

#include "lander/autopilot.hpp"
#include "lander/ballistic.hpp"
#include "lander/binary.hpp"
#include "lander/sim.hpp"

#include <functional>
#include <optional>

namespace lander {

// High-level phase of the powered-landing sequence. The phase picks the target
// state the terminal guidance converges toward; the low-level ZEM/ZEV law is
// shared across phases (R6-01).
enum class LandingPhase {
    Ascend,     // climb out of the launch/crash region toward free flight.
    Transfer,   // long-range translation toward the destination body.
    Capture,    // shed gross hyperbolic/flyby energy (high-energy VGO) or the
                // gentle gravity-feedforward capture (low-energy PD). No pad
                // chasing: the target is the target body centre.
    Deorbit,    // cut the tangential velocity toward the target body's
                // rotating-frame regime and establish a bounded inward radial
                // descent, shaping the trajectory for terminal guidance.
    Brake,      // small O(1) pre-approach frame match: reduce the residual
                // pad-relative velocity and establish a safe attitude, then
                // hand off to the existing ZEM/ZEV terminal law.
    Approach,   // align over the pad under the co-rotating hover waypoint.
    Descent,    // final powered descent; terminal target is the moving pad.
    Touchdown,  // terminal contact as declared by the authoritative Simulation.
};

const char* landing_phase_name(LandingPhase phase);

// Landing-guidance parameters. The guidance recomputes at a lower rate than the
// 120 Hz control loop; t_go is scanned over a bounded, fixed-size candidate set
// around the current estimate (R6-05).
struct LandingConfig {
    double guidance_interval{0.075};  // s (13.3 Hz guidance recomputation)
    double t_go_min{2.0};             // s, nearest admissible terminal time
    double t_go_max{45.0};            // s, farthest admissible terminal time
    int t_go_candidates{20};          // fixed K candidates across the window
    double t_go_spread{0.35};         // geometric growth per candidate step
    double hover_height{30.0};         // m, hover waypoint offset above the pad
    double angle_thrust_limit{0.15};  // rad, no significant thrust while off
    double approach_radius{40.0};     // m, enter DESCENT within this of the hover
    double approach_speed{6.0};       // m/s, and at or below this relative speed
    // V14 (M06-R6-08/09): the gravity-feedforward CAPTURE law used when the
    // target body is "disturbed" (the other body's gravity is a significant
    // fraction of the target's own gravity at the pad). A pure ZEM/ZEV command
    // has no gravity term and cannot hold a hover against that disturbance, so
    // capture cancels the full two-body field and drives a bounded-rate
    // velocity/position correction toward the co-rotating pad. `disturbance_ratio`
    // is the threshold: other/target gravity at the target pad above it => capture.
    double capture_descent_speed{1.0};  // m/s, max target descent rate at the top
    double capture_vel_time{2.0};       // s  velocity-tracking time constant (k_d = 1/t)
    double capture_pos_time{12.0};      // s  position-tracking time constant (k_p = 1/t)
    // V14 (M06-R6-D14): the high-energy capture/de-orbit/brake controller for a
    // grossly hyperbolic (cross-body) arrival, all in the target-body-centred
    // r/t frame (no pad chasing, no position/rate hover terms). It is structurally
    // three phases, every transition hysteresis-gated, and hands off to the
    // existing APPROACH -> DESCENT ZEM/ZEV law:
    //   CAPTURE  : a VGO retrograde brake that removes the gross flyby energy.
    //              It targets zero radial velocity (no dive) and the bounded
    //              local circular tangential speed, preserving the existing
    //              direction of motion, so the pericenter stays clear of the
    //              surface while the flyby energy is shed.
    //   DE-ORBIT : a VGO that cuts the tangential velocity toward the target
    //              body's rotating-frame regime (co-rotating tangential, capped
    //              at local circular so the orbit stays bound) and drives a
    //              bounded inward radial descent, shaping the path for the
    //              terminal guidance without asking ZEM/ZEV to hit the pad yet.
    //   BRAKE    : a small O(1) frame match (gravity feedforward + velocity
    //              tracking toward the co-rotating hover waypoint) that reduces
    //              the residual pad-relative velocity and establishes a safe
    //              attitude, then hands off once slow, low, and aligned.
    double high_energy_factor{1.0};     // high-energy if target-relative speed
                                          //   > factor * local circular at capture entry
    double capture_time{1.5};           // s  VGO time constant, high-energy capture
    double deorbit_time{2.0};           // s  VGO time constant, de-orbit
    double deorbit_descent{2.5};        // m/s  bounded inward radial descent rate
    double deorbit_align_gain{0.5};     // 1/s  de-orbit along-track angle-error gain
    double deorbit_align_damp{2.0};     // -    de-orbit angular-rate damping
    double deorbit_align_max{10.0};     // m/s  bound on the de-orbit alignment velocity
                                           //   offset (prevents the VGO from commanding a
                                           //   sideways thrust the attitude controller
                                           //   cannot follow)
    double deorbit_entry_factor{1.3};   // hysteresis: capture -> de-orbit when
                                           //   v_rel < factor * local circular
    double deorbit_exit_factor{1.6};    // (reserved) de-orbit exit gate width
    double brake_alt_in{110.0};         // m  de-orbit -> brake when altitude below this
    double brake_alt_out{130.0};        // m  hysteresis: brake -> de-orbit when above
    double brake_hover_speed{5.0};      // m/s  (reserved) total pad-relative speed gate
    double brake_time{0.4};             // s  VGO time constant, brake frame match
    double brake_align_gain{0.5};       // 1/s  brake along-track angle-error gain
    double brake_align_damp{2.0};       // -    brake angular-rate damping
    double brake_align_max{5.0};        // m/s  bound on the brake alignment velocity
                                           //   offset
    double brake_descent{3.0};          // m/s  seeded downward rate while well above the
                                           //   hover waypoint; fades to brake_min_descent
    double brake_min_descent{2.5};      // m/s  residual downward rate kept at the hover
                                           //   altitude so the terminal law receives a
                                           //   genuine descent instead of a hover
    double brake_tangential_blend{1.0}; // -    blend between pad-surface (0) and hover-
                                           //   waypoint (1) tangential base used by the brake
    double brake_final_alt{180.0};       // m  below this the high-energy velocity kill is
                                            //   active (the deorbit phase uses the same kill)
    double brake_final_speed{8.0};       // m/s  target-body speed gate for the velocity kill
    double brake_kill_ramp_alt{40.0};    // m  altitude where the kill's tangential setpoint
                                            //   starts fading from co-rotation toward the
                                            //   small hand-off value
    double brake_kill_descent{3.0};      // m/s  radial descent rate at the top of the
                                            //   kill's descent ramp
    double brake_kill_descent_min{2.0}; // m/s  radial descent rate at the hand-off
                                            //   altitude
    double brake_kill_tangential{0.3};   // m/s  pad-relative tangential setpoint at hand-off
    double brake_kill_align_gain{0.2};   // 1/s  angle-rate feedback gain (1 / alignment time)
                                            //   for the kill's bounded pad-angle correction
    double brake_kill_align_max{10.0};   // m/s  clamp on the kill's pad-angle correction
    double brake_handoff_alt{30.0};       // m  hand off to the terminal law no higher than this
    double brake_handoff_tangential{0.5};  // m/s  max pad-relative tangential speed at hand-off
    double brake_handoff_radial_min{1.0};  // m/s  min downward radial speed at hand-off
    double brake_handoff_radial_max{4.0};  // m/s  max downward radial speed at hand-off
    double brake_handoff_lateral{16.0};   // m  max along-track offset from the hover waypoint
    // High-energy Approach -> Descent gate: the pad-relative radial velocity
    // must not be faster than this (negative) value before the terminal Descent
    // phase is entered. 0 disables the gate (the default low-energy behaviour).
    double high_energy_radial_max{0.0};
    // High-energy Brake hand-off dwell: once the hand-off gates are satisfied,
    // remain in Brake for this many seconds before transitioning to Approach,
    // so the alignment can settle. 0 disables the dwell (default).
    double brake_handoff_dwell{0.0};
    // M06-R7 terminal-feasibility handoff gate for the high-energy front end.
    // The craft may enter the terminal flow only when a hypothetical Descent
    // guidance update from the current state selects a bounded terminal time,
    // the Apollo profile peak is below a conservative fraction of the main
    // acceleration, and the current pad-relative state is inside the descent
    // corridor. The values default to the existing high-energy hand-off band.
    double terminal_handoff_t_go_max{20.0};   // s  max admissible selected t_go
    double terminal_handoff_peak_factor{0.85}; // -  max peak accel as a fraction
                                                 //   of main_accel
    double terminal_handoff_tangential{0.5};   // m/s  max pad-relative tangential
    double terminal_handoff_radial_min{1.0};   // m/s  min downward pad-relative
    double terminal_handoff_radial_max{2.5};   // m/s  max downward pad-relative
    double terminal_handoff_lateral{16.0};     // m  max along-track offset
    double disturbance_ratio{0.2};      // other/target gravity ratio at the target pad
    // V14 (M06-R6-10): cross-body launch. Climb to this altitude above the
    // source body in ASCEND before running the COLD transfer solve, then ride
    // the arc (TRANSFER) to the target's capture shell and hand to CAPTURE.
    double transfer_launch_altitude{90.0};  // m above the source surface
    double transfer_climb_speed{3.0};       // m/s climb rate in ASCEND
    double transfer_capture_radius{0.0};    // m; 0 = auto (2.5x target max surface radius)
    double transfer_replan_interval{0.1};   // s  WARM bounded-rate replan period
    double transfer_miss_tolerance{0.25};   // m  re-target when replan miss exceeds this
    double transfer_launch_interval{0.25};  // s  max cadence for COLD launch solves
};

// The target state the terminal guidance converges toward at future time
// now + t_go: the moving pad (DESCENT) or the co-rotating hover waypoint
// (APPROACH and earlier phases), position and velocity from
// BinarySystem::surface_point at the future time (R6-02).
struct LandingTargetState {
    Vec2 position{};
    Vec2 velocity{};
};

LandingTargetState landing_target_state(const BinarySystem& bin, int target_body,
                                        LandingPhase phase,
                                        const LandingConfig& config, double now,
                                        double t_go);

// The classical fixed-terminal ZEM/ZEV command: the requested acceleration
// correction that drives the (position, velocity) error to zero at the terminal
// time. It is a thrust correction only; no gravity term is added (R6-04, P02).
Vec2 landing_acceleration_command(const Vec2& zem, const Vec2& zev, double t_go);

// A zero-effort query: given a candidate terminal time-to-go, the zero-thrust
// state reached at now + t_go if no further main-engine thrust is applied
// (R6-03). In production this is the O(1) maintained rolling-predictor lookup;
// the guidance math is independent of how the source is produced.
using ZeroEffortQuery = std::function<BallisticState(double t_go)>;

// Bounded K-candidate time-to-go selection: a fixed small set around the
// current estimate; reject non-finite or over-thrust candidates; choose the
// lowest deterministic cost (R6-05, P03). Returns the selected t_go, or nullopt
// when no candidate is feasible.
std::optional<double> landing_select_time_to_go(
    const BinarySystem& bin, const Config& config, const LandingConfig& landing,
    double now, double t_go_estimate, int target_body, LandingPhase phase,
    const ZeroEffortQuery& zero_effort);

// The concrete guidance output for one update: the selected t_go, the requested
// acceleration command, and the throttle needed to supply it. valid is false
// when no candidate was feasible.
struct LandingCommand {
    double t_go{};
    Vec2 acceleration{};
    double throttle{};
    bool valid{false};
};

// M06-R7: pure feasibility preview for a hypothetical Descent handoff. It
// reuses the canonical terminal selector/profile/command machinery without
// modifying those regions and returns the bounded metrics used by the
// high-energy terminal handoff gate.
struct LandingTerminalPreview {
    bool command_valid{false};
    bool feasible{false};
    double t_go{0.0};
    double peak_accel{0.0};
    Vec2 acceleration{};
    double initial_radial{0.0};
};

LandingTerminalPreview landing_terminal_preview(
    const BinarySystem& bin, const Config& config, const LandingConfig& landing,
    double now, double t_go_estimate, int target_body,
    const ZeroEffortQuery& zero_effort);

// One guidance update: scan the bounded t_go set, compute the ZEM/ZEV command at
// the selected t_go, and derive the throttle (R6-04, R6-05). This is the O(1)
// work of the guidance call; the zero-effort query is expected to be an O(1)
// maintained lookup (R6-09).
LandingCommand landing_guidance_command(const BinarySystem& bin,
                                        const Config& config,
                                        const LandingConfig& landing, double now,
                                        double t_go_estimate, int target_body,
                                        LandingPhase phase,
                                        const ZeroEffortQuery& zero_effort);

// The high-level target-pad landing autopilot (R6). It holds the current phase
// and the most recent guidance command. `make_input` runs at the 120 Hz control
// rate and re-emits the held command: it routes the commanded direction through
// the ordinary bang-bang attitude controller and applies main throttle only
// while the nose is within the thrust-alignment limit (R6-04). `after_step`
// recomputes the command at the lower guidance rate and advances the phase
// machine (R6-01, R6-07, R6-08). It emits only ordinary Input; it never mutates
// spacecraft state directly (R6-04, P01). In production the zero-effort query is
// the R3 maintained Coast-predictor lookup (R6-03, R6-09), not a fresh rollout.
class LandingAutopilot {
public:
    struct Status {
        LandingPhase phase{LandingPhase::Approach};
        bool armed{false};
        bool output_active{false};  // currently emitting any throttle/rotation.
        double t_go{0.0};
        double command_accel{0.0};
        bool terminal{false};
    };

    // Arm an autoland targeting `target_body`. The zero-effort query must be a
    // maintained source (the R3 Coast predictor lookup); it is invoked only from
    // the guidance update, never per control tick.
    void arm(int target_body, const LandingConfig& config,
             const ZeroEffortQuery& zero_effort);
    void set_config(const LandingConfig& config) { config_ = config; }
    void abort();

    bool armed() const { return armed_; }
    LandingPhase phase() const { return phase_; }
    const Status& status() const { return status_; }
    // Telemetry / diagnostics for the two-level transfer drive (V14 cross-body).
    const lander::TransferMidcourse& mc() const noexcept { return transfer_mc_; }

    // M06-R8: small read-only getters so the subsystem-isolation debug panel
    // can show the armed target, the active capture path, and the held
    // command / terminal preview. Pure accessors: no behaviour change.
    int target_body() const noexcept { return target_body_; }
    int source_body() const noexcept { return source_body_; }
    bool high_energy() const noexcept { return high_energy_; }
    bool target_disturbed() const noexcept { return target_disturbed_; }
    const LandingCommand& command() const noexcept { return command_; }
    const LandingTerminalPreview& terminal_preview() const noexcept {
        return terminal_preview_;
    }

    // 120 Hz control rate: the ordinary Input for this tick (held command,
    // bang-bang attitude, aligned-gated throttle).
    Input make_input(const State& state, const Config& config, double now) const;

    // 120 Hz, after each live step: recompute the command at the guidance rate
    // and advance the phase machine.
    void after_step(const State& before, const State& after,
                    const BinarySystem& bin, const Config& config, double now);

private:
    void guidance_update(const State& state, const BinarySystem& bin,
                         const Config& config, double now);
    void update_phase(const State& before, const State& after,
                      const BinarySystem& bin, const Config& config, double now);
    void update_t_go_estimate(const State& state, const BinarySystem& bin,
                              const Config& config, double now);

    // V14: route to the correct phase for the source/target geometry on the
    // first step (arm() has no live state). Same-body disturbed targets enter
    // CAPTURE; clean targets keep the existing APPROACH flow; cross-body starts
    // in ASCEND.
    void route(const State& before, const BinarySystem& bin, double now);
    // V14: capture/ascend command. `climb` selects the launch waypoint over the
    // source body (ascend); otherwise a high-energy arrival uses the
    // target-body-centred VGO (no pad chasing) and a slow local arrival uses a
    // gentle gravity-feedforward PD to the pad.
    LandingCommand capture_command(const State& state, const BinarySystem& bin,
                                   const Config& config, double now, bool climb);
    // V14: deorbit command -- a bounded VGO descent that matches the target
    // body's co-rotation (so the pad angle is held) and closes the residual
    // along-track angle error with a damped, bounded velocity offset.
    LandingCommand deorbit_command(const State& state, const BinarySystem& bin,
                                    const Config& config, double now);
    // V14: brake command -- a tighter VGO frame match to the co-rotating hover
    // waypoint, with a bounded descent rate, before handing off to the ZEM/ZEV
    // terminal law.
    LandingCommand brake_command(const State& state, const BinarySystem& bin,
                                  const Config& config, double now);
    void update_ascend(const State& before, const State& after,
                        const BinarySystem& bin, const Config& config, double now);
    void update_transfer(const State& before, const State& after,
                          const BinarySystem& bin, const Config& config, double now);

    // M06-R7: low-rate hypothetical Descent preview for the high-energy
    // terminal handoff gate.
    void terminal_preview_update(const State& state, const BinarySystem& bin,
                                 const Config& config, double now);
    bool terminal_handoff_ok(const State& state, const BinarySystem& bin,
                             double now) const;

    bool armed_{false};
    bool terminal_{false};
    int target_body_{0};
    int source_body_{-1};
    bool routed_{false};
    bool target_disturbed_{false};
    LandingConfig config_{};
    ZeroEffortQuery zero_effort_{};
    LandingPhase phase_{LandingPhase::Approach};
    double t_go_estimate_{10.0};
    double last_guidance_time_{-1e300};
    LandingCommand command_{};
    Status status_{};
    // V14 cross-body: the two-level transfer midcourse and its warm cache.
    lander::TransferMidcourse transfer_mc_{};
    lander::TransferSolution transfer_sol_{};
    bool transfer_launched_{false};
    double last_launch_try_{-1e300};
    // V14 (M06-R6-D14): a grossly hyperbolic (cross-body) arrival runs the
    // high-energy CAPTURE -> DE-ORBIT -> BRAKE flow; a slow same-body capture
    // keeps the gentle gravity-feedforward law and lands in place.
    bool high_energy_{false};
    double brake_handoff_ready_time_{-1e300};
    LandingTerminalPreview terminal_preview_{};
    double last_terminal_preview_time_{-1e300};
};

}  // namespace lander
