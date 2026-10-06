#pragma once

#include "lander/binary.hpp"
#include "lander/terrain.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace lander {

struct Config {
    double fixed_dt{1.0 / 120.0};
    // Gravitational parameter of the primary (M04 reference moon). The
    // companion is derived from this value by the canonical scaling law.
    double mu{178976.334};
    double main_accel{4.0};
    double rotate_accel{1.2};
    double fuel{1000.0};
    double fuel_burn{8.0};
    double safe_vertical_speed{2.0};
    double safe_horizontal_speed{1.0};
    double safe_angle_rad{0.15};
    // M05-R3: finite angular damping for the manual reaction-wheel control.
    // The full acceleration is applied only when `|omega|` reaches the taper
    // rate; it tapers linearly toward zero and never zeroes omega directly.
    double reaction_wheel_accel{0.8};
    double reaction_wheel_taper{0.25};
};

struct State {
    double x{};
    double y{};
    double vx{};
    double vy{};
    double angle{};
    double omega{};
    double fuel{};
    bool landed{false};
    bool crashed{false};
    // While landed: the body the ship is attached to (-1 when not landed)
    // and the body-local surface arc of the attachment point.
    int landed_body{-1};
    double landed_arc{0.0};
    // The body whose surface the ship hit (-1 when not crashed).
    int crash_body{-1};
    int score{0};
    int ticks{0};
    bool operator==(const State&) const = default;
};

// One delivery assignment in the repeating contract loop. The active
// contract is never completed in place: when the spacecraft safely lands on
// the destination base pad, the reward is paid and the next contract (from
// that base to the other body) replaces it. `completed` therefore marks a
// contract record that has finished; the Simulation additionally keeps the
// most recently completed record for presentation.
struct Contract {
    int origin_body{0};
    int destination_body{1};
    int reward{0};
    bool completed{false};
    bool operator==(const Contract&) const = default;
};

struct Input {
    bool rotate_left{false};
    bool rotate_right{false};
    double main_throttle{0.0};
    // M05-R3: manual reaction-wheel angular-rate damping (polled key).
    bool reaction_wheels{false};
};

struct LocalVelocity {
    double radial{};
    double tangential{};
};

// Body-relative navigation helpers. All quantities are relative to the
// given body's centre (bpos) and the velocity passed as bvel; ground and
// landing checks pass the rotating surface point's velocity so the relative
// components include the tidal-lock spin (M05-R3).
double radial_distance(const State& state, const Vec2& bpos);
double local_up_angle(const State& state, const Vec2& bpos);
double local_attitude_angle(const State& state, const Vec2& bpos);
LocalVelocity local_velocity(const State& state, const Vec2& bpos,
                              const Vec2& bvel);
// The terrain arc (and surface radius) below `state`: the world angle from
// the body centre converted to a body-local angle by subtracting the
// body's tidal-lock spin `body_rotation` (0.0 for a non-spinning body).
double surface_radius_at(const Terrain& terrain, const State& state,
                          const Vec2& bpos, double body_rotation = 0.0);
double altitude_at(const Terrain& terrain, const State& state,
                    const Vec2& bpos, double body_rotation = 0.0);
// Presentation: the ship's local angular rate (rad/s) about the given body's
// centre: the body-relative tangential velocity divided by the body-relative
// radial distance. Zero at the exact body centre. The sign encodes direction.
// The HUD and its tests both call this.
double local_angular_velocity(const State& state, const Vec2& bpos,
                               const Vec2& bvel);

// M05-R2: signed scalar range rate between the ship and a target body.
// Negative means closing, positive means opening; zero inside the guard
// radius.
double target_range_rate(const Vec2& ship_pos, const Vec2& ship_vel,
                          const Vec2& target_pos, const Vec2& target_vel);

// M05-R3: the ship's local angular rate (rad/s) about a body's centre,
// presented in degrees per second for the HUD.
inline double spin_deg_per_s(const State& state) {
    return state.omega * (180.0 / kPi);
}

// M05-R3: relative tangential velocity divided by relative radial distance
// (the "ORB" readout). Same definition as local_angular_velocity, exposed
// with an explicit HUD name.
double orbital_rate(const State& state, const Vec2& bpos,
                     const Vec2& bvel);

// M05-R3: directional label for a signed range rate. Negative range rates
// are closing, positive range rates are opening.
const char* range_rate_label(double range_rate, double tolerance = 0.05);

// M05-R3: pure navigation/gravity cues for the compact screen-space overlay.
// The target is the contract destination base pad: a point fixed on the
// destination body's rotating surface, so both its position and its velocity
// track the tidal-lock spin; the relative velocity is the ship's minus that
// surface point's velocity. All gravity is the true two-body inverse-square
// field (no faking).
struct NavCues {
    Vec2 target_position{};
    Vec2 target_direction{};
    double target_distance{};
    Vec2 relative_velocity{};
    double relative_speed{};
    Vec2 gravity_primary{};
    Vec2 gravity_companion{};
    Vec2 net_gravity{};
    double g_primary{};
    double g_companion{};
    double g_net{};
};
NavCues navigation_cues(const State& state, const BinarySystem& system,
                         double t, int destination_body);

// M05-R2: deterministic reference-body selection by local gravitational
// influence, `mu / distance^2`, with hysteresis. The current body is kept
// unless the other body's influence exceeds `margin *` the current influence;
// at an exact crossover the current body is kept.
int reference_body_for(double mu0, double mu1, double distance0,
                        double distance1, int current, double margin = 1.2);

// M06-R13: three-body generalization of reference_body_for: candidates are
// the primary, the companion, and the outer moonlet; the current body is
// kept unless some other body's influence exceeds `margin *` the current
// influence; at an exact crossover the current body is kept.
int reference_body_for3(double mu0, double mu1, double mu2, double d0,
                        double d1, double d2, int current,
                        double margin = 1.2);

// Presentation only: the state of a ship attached to `body_index` at the
// body-local surface arc `landed_arc`, at ephemeris time `t`. The position
// is the body's surface point at that arc (tidal-lock spin included), the
// velocity is that surface point's full inertial velocity (translational
// plus spin), and the nose points along the local vertical (radial out).
State attached_state(const BinarySystem& system, int body_index,
                     double landed_arc, double t);

// Presentation-only interpolation between two authoritative fixed-step
// states: a linear (Cartesian) blend of global positions and a
// shortest-arc blend of the attitude angle. It never modifies either input
// and is not used by physics, collision, fuel use, or scoring.
State interpolated_state(const State& previous, const State& current,
                         double alpha, bool snap_to_current);

// Presentation-only flame animation, driven by a continuous presentation
// clock in seconds (not the integer simulation tick counter). These helpers
// never affect physics, fuel, thrust, collision, or authoritative
// determinism. flame_flick is the smooth per-time variation term;
// flame_length maps it to a flame length scaled by the throttle level.
double flame_flick(double t);
double flame_length(double thrust_level, double t);

// M06-R18: presentation-only source of truth for the drawn plume. The
// rendered thrust level is the ACTUAL main-engine input the authoritative
// simulation last received (`applied_main_throttle`, the `Input.main_throttle`
// composed for the last `Simulation::step_once`), clamped to [0, 1] and
// suppressed to 0 for a crashed / landed ship or an empty tank. It is the
// same value for every thrust source (manual, node executor, transfer
// midcourse, landing autopilot) because every one of them composes its burn
// through that one input field. Never used by physics, fuel, collision, or
// scoring; headless-testable pure mapping.
double presentation_thrust_level(const State& state,
                                 double applied_main_throttle);

bool operator==(const Input& lhs, const Input& rhs);
bool operator==(const Config& lhs, const Config& rhs);

class Simulation {
public:
    // A default-constructed simulation starts a fresh game (seed 0):
    // landed at the primary base at the start of the binary ephemeris.
    Simulation();
    explicit Simulation(const Config& config);

    // Starts a fresh game from `seed`: both terrains and the binary phase
    // are pure functions of the seed, the ship is landed at the primary
    // base, and the first contract targets the companion base.
    void reset(std::uint64_t seed);
    void advance(double elapsed, const Input& input);
    // Run exactly one authoritative fixed step with `input`. The GUI uses
    // this so a per-step autopilot can re-compose the input from the state
    // produced by the previous step. Returns false when the step crashed.
    bool step_once(const Input& input);
    // Directly set the fixed-step accumulator (clamped to `[0, 10]`) for a
    // GUI-owned fixed-step loop.
    void set_accumulator(double value) {
        accumulator_ = value < 0.0 ? 0.0 : (value > 10.0 ? 10.0 : value);
    }
    const State& state() const noexcept { return state_; }
    const State& previous_state() const noexcept { return previous_; }
    const Config& config() const noexcept { return config_; }
    const BinarySystem& binary() const noexcept { return binary_; }
    const Terrain& terrain() const noexcept { return binary_.body(0).terrain; }
    const Terrain& terrain(int index) const noexcept {
        return binary_.body(index).terrain;
    }
    std::uint64_t seed() const noexcept { return seed_; }
    double accumulator() const noexcept { return accumulator_; }
    // Authoritative fixed-step simulation time in seconds. The binary
    // ephemeris is driven by this clock only; pausing pauses it.
    double sim_time() const noexcept { return sim_time_; }
    // The presentation time between the previous and current authoritative
    // states that render interpolation represents.
    double presentation_time() const noexcept {
        return sim_time_ - config_.fixed_dt + accumulator_;
    }
    // Presentation reference body (local HUD, camera, circularize). It never
    // affects which gravitational fields are active.
    int reference_body() const noexcept { return reference_body_; }
    const Contract& contract() const noexcept { return contract_; }
    const std::optional<Contract>& last_completed() const noexcept {
        return last_completed_;
    }
    int contracts_completed() const noexcept { return contracts_completed_; }

    void set_state(const State& state);
    void set_config(const Config& config) { config_ = config; }

    // One-time developer control: set the velocity for a circular orbit
    // around the current reference body (body-relative circular speed plus
    // the body's own ephemeris velocity). Clockwise is the default;
    // `counter_clockwise` reverses the tangential direction. No continuing
    // stabilization.
    void circularize(bool counter_clockwise = false);
    // M05-R3-15: one-shot developer initializer: places the ship in a
    // body-synchronous circular orbit around the source body (the landed
    // body when the ship is landed, otherwise the current reference body)
    // on the side opposite the other body, with the binary's angular
    // velocity, so the ship co-rotates with the binary. It is a single
    // instantaneous state change; no continuing stabilization or autopilot
    // after it. No-op while crashed.
    void sync_orbit();
    // M05-R3-16: one-shot developer initializer: deterministically solves a
    // ballistic arc from the source body (the landed body when the ship is
    // landed, otherwise the current reference body) to the other body using
    // the real two-body gravity field and the bodies' future ephemeris, and
    // places the ship at the arc's start: a small clearance shell above the
    // source, unlanded, nose along the initial velocity. Returns false and
    // leaves the state unchanged when no plausible solution exists for this
    // binary phase. No-op (false) while crashed.
    bool transfer();
    // Refill the fuel tank; allowed in flight and on the ground, never
    // after a crash.
    void refuel();

private:
    void step_fixed(const Input& input);
    void integrate_flight(const Input& input, double t0);
    void attach_to_body();
    bool try_takeoff(const Input& input, double t0);
    void resolve_ground_contact();
    void update_reference_body();

    Config config_;
    State state_;
    State previous_{};
    BinarySystem binary_;
    Contract contract_;
    std::optional<Contract> last_completed_;
    int contracts_completed_{0};
    int reference_body_{0};
    std::uint64_t seed_{0};
    double sim_time_{0.0};
    double accumulator_{0.0};
};

}  // namespace lander
