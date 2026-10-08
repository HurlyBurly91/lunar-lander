// M06-R8: subsystem-isolation debug harness (implementation).
//
// See include/lander/debug_subsystem.hpp. This is a read-only diagnostic
// harness: it only builds deterministic startup-only fixtures and parses the
// selector. It never changes normal gameplay, canonical physics, or fixed_dt,
// and it never fixes subsystem defects (it only surfaces them).

#include "lander/debug_subsystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace lander {

namespace {

// Place the ship in a terrain-clearing circular orbit around `body` at t = 0,
// reusing the proven --orbit-demo placement: the ship is due north of the body
// centre, with tangential velocity equal to the circular-orbit speed plus the
// body's own barycentric velocity. Pure with respect to the binary (it only
// sets the ship state).
//
// M06-R11: the orbit radius and speed now come from the SELECTED body's own
// terrain and gravitational parameter (b.terrain, b.mu), not from the primary
// terrain and cfg.mu (the primary's mu). The previous code produced a wrong
// (too large) radius and wrong (too slow) circular speed for a companion
// orbit, because the companion is ~1/9 the primary's radius with ~1/81 its mu.
// The direction convention (due north, tangential = circular speed + body
// barycentric velocity) and the +20 m clearance are preserved.
void place_in_orbit(Simulation& sim, int body) {
    const Config& cfg = sim.config();
    const BinarySystem& bin = sim.binary();
    const Body& b = bin.body(body);
    const double r = b.terrain.max_surface_radius() + 20.0;
    const Vec2 p0 = bin.position(body, 0.0);
    const Vec2 v0 = bin.velocity(body, 0.0);
    const double speed = std::sqrt(b.mu / r);
    State orbit{};
    orbit.x = p0.x;
    orbit.y = p0.y + r;
    orbit.vx = v0.x + speed;
    orbit.vy = v0.y;
    orbit.angle = 0.0;
    orbit.fuel = cfg.fuel;
    sim.set_state(orbit);
}

}  // namespace

std::optional<DebugSubsystem> parse_debug_subsystem(const std::string& name) {
    if (name == "none") {
        return DebugSubsystem::None;
    }
    if (name == "manual") {
        return DebugSubsystem::Manual;
    }
    if (name == "predictor") {
        return DebugSubsystem::Predictor;
    }
    if (name == "attitude") {
        return DebugSubsystem::Attitude;
    }
    if (name == "node-edit") {
        return DebugSubsystem::NodeEdit;
    }
    if (name == "node-executor") {
        return DebugSubsystem::NodeExecutor;
    }
    if (name == "transfer-cold") {
        return DebugSubsystem::TransferCold;
    }
    if (name == "transfer-warm") {
        return DebugSubsystem::TransferWarm;
    }
    if (name == "autoland-primary") {
        return DebugSubsystem::AutolandPrimary;
    }
    if (name == "autoland-companion") {
        return DebugSubsystem::AutolandCompanion;
    }
    if (name == "autoland-cross") {
        return DebugSubsystem::AutolandCross;
    }
    if (name == "ui") {
        return DebugSubsystem::Ui;
    }
    return std::nullopt;
}

std::optional<int> parse_debug_predictor_body(const std::string& value) {
    if (value == "0") {
        return 0;  // primary
    }
    if (value == "1") {
        return 1;  // companion
    }
    if (value == "2") {
        return 2;  // outer moonlet
    }
    return std::nullopt;
}

const char* debug_subsystem_name(DebugSubsystem mode) {
    switch (mode) {
        case DebugSubsystem::None:
            return "none";
        case DebugSubsystem::Manual:
            return "manual";
        case DebugSubsystem::Predictor:
            return "predictor";
        case DebugSubsystem::Attitude:
            return "attitude";
        case DebugSubsystem::NodeEdit:
            return "node-edit";
        case DebugSubsystem::NodeExecutor:
            return "node-executor";
        case DebugSubsystem::TransferCold:
            return "transfer-cold";
        case DebugSubsystem::TransferWarm:
            return "transfer-warm";
        case DebugSubsystem::AutolandPrimary:
            return "autoland-primary";
        case DebugSubsystem::AutolandCompanion:
            return "autoland-companion";
        case DebugSubsystem::AutolandCross:
            return "autoland-cross";
        case DebugSubsystem::Ui:
            return "ui";
    }
    return "none";
}

const char* debug_subsystem_description(DebugSubsystem mode) {
    switch (mode) {
        case DebugSubsystem::None:
            return "normal gameplay (no debug surface)";
        case DebugSubsystem::Manual:
            return "manual thrust / rotation control on a landed ship";
        case DebugSubsystem::Predictor:
            return "receding-horizon live ballistic trajectory prediction";
        case DebugSubsystem::Attitude:
            return "proportional-derivative attitude controller (VGO)";
        case DebugSubsystem::NodeEdit:
            return "single maneuver-node planner (no burn executed)";
        case DebugSubsystem::NodeExecutor:
            return "one-shot fixed-step node executor / VGO";
        case DebugSubsystem::TransferCold:
            return "one-shot COLD inter-moon transfer solve (coarse grid)";
        case DebugSubsystem::TransferWarm:
            return "WARM two-level transfer midcourse (bounded differential "
                  "correction)";
        case DebugSubsystem::AutolandPrimary:
            return "target-pad powered landing, primary body (source==target)";
        case DebugSubsystem::AutolandCompanion:
            return "target-pad powered landing, companion body (source==target)";
        case DebugSubsystem::AutolandCross:
            return "target-pad powered landing, cross-body (source!=target)";
        case DebugSubsystem::Ui:
            return "camera / contract / HUD / banner / control UI";
    }
    return "normal gameplay (no debug surface)";
}

bool keeps_prediction_overlay(DebugSubsystem mode) {
    // Only the predictor isolation keeps its predicted-trajectory overlay
    // visible by default. Every other mode (including None / normal gameplay
    // and Ui, which use the G toggle, and NodeEdit, which keeps its own
    // G-gated pre/post arc) is unchanged, so this never touches normal
    // gameplay rendering.
    return mode == DebugSubsystem::Predictor;
}

DebugCommonReadout make_common_readout(const Simulation& sim) {
    DebugCommonReadout r;
    const State& st = sim.state();
    const double t = sim.sim_time();
    r.sim_time = t;
    r.x = st.x;
    r.y = st.y;
    r.reference_body = sim.reference_body();
    r.target_body = sim.contract().destination_body;
    r.reference_label = debug_body_label(r.reference_body);
    r.target_label = debug_body_label(r.target_body);
    r.landed = st.landed;
    r.crashed = st.crashed;

    const BinarySystem& bin = sim.binary();
    const Vec2 ref_pos = bin.position(r.reference_body, t);
    const Vec2 ref_vel = bin.velocity(r.reference_body, t);
    // M06-R11: altitude is measured against the SELECTED reference body's own
    // terrain, using that body's tidal rotation at time t -- not the primary
    // terrain and not local_up_angle (a per-point world up that is not the
    // tidal-frame rotation altitude_at expects). The value stays signed so a
    // sub-surface reading is not silently clamped to zero or blanked.
    const Terrain& ref_terrain = bin.body(r.reference_body).terrain;
    const double rot = bin.body_rotation(r.reference_body, t);
    r.altitude = altitude_at(ref_terrain, st, ref_pos, rot);
    const LocalVelocity lv = local_velocity(st, ref_pos, ref_vel);
    r.radial_velocity = lv.radial;
    r.tangential_velocity = lv.tangential;

    // Relative speed of the ship with respect to the contract-destination
    // body's centre (inertial) velocity -- body-centre, not the pad.
    const Vec2 dest_vel = bin.velocity(r.target_body, t);
    const double rvx = st.vx - dest_vel.x;
    const double rvy = st.vy - dest_vel.y;
    r.relative_speed = std::hypot(rvx, rvy);
    return r;
}

std::uint64_t debug_scenario_seed(DebugSubsystem mode) {
    switch (mode) {
        case DebugSubsystem::None:
            return 0;
        case DebugSubsystem::Manual:
            return 1001;
        case DebugSubsystem::Predictor:
            return 1002;
        case DebugSubsystem::Attitude:
            return 1003;
        case DebugSubsystem::NodeEdit:
            return 1004;
        case DebugSubsystem::NodeExecutor:
            return 1005;
        case DebugSubsystem::TransferCold:
            return 1006;
        case DebugSubsystem::TransferWarm:
            return 1007;
        case DebugSubsystem::AutolandPrimary:
            return 1008;
        case DebugSubsystem::AutolandCompanion:
            return 1009;
        case DebugSubsystem::AutolandCross:
            return 1010;
        case DebugSubsystem::Ui:
            return 1011;
    }
    return 0;
}

void setup_debug_scenario(DebugSubsystem mode, DebugSubsystems& s,
                          int predictor_body) {
    if (mode == DebugSubsystem::None) {
        return;
    }

    // Deterministic per-mode seed, then clear every subsystem so exactly the
    // one under test can be armed (preservation M06-R8-06).
    s.sim.reset(debug_scenario_seed(mode));
    s.node_executor.clear();
    s.transfer_mc.abort();
    s.landing_ap.abort();
    s.maneuver_node.reset();
    s.attitude_mode = AttitudeMode::Off;

    const Config& cfg = s.sim.config();

    switch (mode) {
        case DebugSubsystem::Manual:
        case DebugSubsystem::Ui:
            // Leave the freshly reset ship landed on the primary at its pad
            // (reset() attaches it there). Nothing else is armed.
            break;

        case DebugSubsystem::Predictor:
            // A clean orbit around the selected body (M06-R13: default primary,
            // --debug-predictor-body 1 / 2 for companion / outer moonlet) so
            // the live predictor (driven in the main loop) has a moving target
            // to keep receding-horizon predictions of.
            place_in_orbit(s.sim,
                           predictor_body < 0 ? 0
                                              : (predictor_body > 2
                                                     ? 2
                                                     : predictor_body));
            break;

        case DebugSubsystem::Attitude:
            // A clean orbit with a deliberate attitude demand so the PD
            // attitude controller has a nonzero error to drive.
            place_in_orbit(s.sim, 0);
            s.attitude_mode = AttitudeMode::Prograde;
            break;

        case DebugSubsystem::NodeEdit:
        case DebugSubsystem::NodeExecutor: {
            place_in_orbit(s.sim, 0);
            const double t0 = s.sim.sim_time();
            ManeuverNode node = default_node(t0, 0, cfg.fixed_dt);
            if (mode == DebugSubsystem::NodeExecutor) {
                // M06-R19 (fixture observability): a deliberately OBSERVABLE
                // mixed PGR+RAD node. The prior 0.5 m/s prograde-only node was
                // a ~0.125 s full-throttle burn at the 4.0 m/s^2 main accel,
                // too short to visually judge the continuous alignment-safety
                // behaviour. |dv| = hypot(4,2) = 4.472 m/s -> a ~1.12 s
                // full-throttle burn: long, visibly non-pure-prograde, and
                // bounded. DEBUG FIXTURE ONLY: node-edit keeps its own 0.5 m/s
                // prograde node (below); normal node defaults, gameplay,
                // main/rotate accel, thresholds, the executor, and physics are
                // all unchanged.
                node.dv_prograde = 4.0;
                node.dv_radial = 2.0;
            } else {
                // NodeEdit: a small visible prograde burn (unchanged).
                node.dv_prograde = 0.5;
            }
            s.maneuver_node = node;
            if (mode == DebugSubsystem::NodeExecutor) {
                // Arm the one-shot executor on the node, mirroring the
                // Return-key execution path.
                const State& st = s.sim.state();
                const double t = std::max(t0, snap_time(node.time, cfg.fixed_dt));
                const BallisticState initial{{st.x, st.y}, {st.vx, st.vy}, t0};
                const int steps = ballistic_steps(t0, t, cfg.fixed_dt);
                const BallisticState pre = propagate_ballistic(
                    s.sim.binary(), initial, steps, cfg.fixed_dt);
                const NodeBasis basis =
                    compute_node_basis(s.sim.binary(), t, node.frame_body,
                                       pre.p, pre.v);
                s.node_executor.arm(node, basis, t0, cfg);
                s.attitude_mode = AttitudeMode::Maneuver;
            }
            break;
        }

        case DebugSubsystem::TransferCold:
        case DebugSubsystem::TransferWarm: {
            place_in_orbit(s.sim, 0);
            const double t0 = s.sim.sim_time();
            const State& st = s.sim.state();
            const int source = 0;
            const int target = 1;
            Vec2 v_out{};
            TransferSolution cold{};
            // Measure the COLD solve's zero-thrust-propagation cost (it must be
            // a bounded coarse grid, never a per-step solve) and its wall time.
            ballistic_reset_propagation_count();
            const auto solve_clock = std::chrono::steady_clock::now();
            const bool solved =
                solve_transfer_velocity(s.sim.binary(), cfg.fixed_dt,
                                        {st.x, st.y}, source, target, t0,
                                        v_out, &cold);
            const double solve_ms =
                std::chrono::duration_cast<std::chrono::duration<double,
                                                                 std::milli>>(
                    std::chrono::steady_clock::now() - solve_clock)
                    .count();
            if (s.transfer_debug) {
                s.transfer_debug->computed = true;
                s.transfer_debug->warm = (mode == DebugSubsystem::TransferWarm);
                s.transfer_debug->source = source;
                s.transfer_debug->target = target;
                s.transfer_debug->cold = cold;
                s.transfer_debug->cold_propagations =
                    ballistic_propagation_count();
                s.transfer_debug->cold_solve_ms = solve_ms;
            }
            if (mode == DebugSubsystem::TransferWarm && solved) {
                // Seed a valid warm cache from the cold solution and arm the
                // two-level midcourse; the main loop's maybe_replan then runs
                // the bounded-rate WARM differential correction each interval
                // (never per fixed step).
                const NodeBasis basis =
                    compute_node_basis(s.sim.binary(), t0, source, {st.x, st.y},
                                       {st.vx, st.vy});
                ManeuverNode arm_node{};
                arm_node.time = t0;
                arm_node.frame_body = source;
                s.transfer_mc.arm(arm_node, cold, source, basis, t0, cfg);
            }
            break;
        }

        case DebugSubsystem::AutolandPrimary:
        case DebugSubsystem::AutolandCompanion:
        case DebugSubsystem::AutolandCross: {
            const int target =
                (mode == DebugSubsystem::AutolandPrimary) ? 0 : 1;
            // The ship orbits the landing target (primary / companion); for the
            // cross-body mode it orbits the primary while the target is the
            // companion, exercising the source!=target route.
            const int orbit_body =
                (mode == DebugSubsystem::AutolandCross) ? 0 : target;
            place_in_orbit(s.sim, orbit_body);
            // Configure + cold-rebuild the maintained zero-thrust Coast
            // predictor that backs the landing autopilot's O(1) zero-effort
            // source, mirroring the 9-key arm path. The caller's `zero_effort`
            // lambda closes over this predictor and the gui-local lookup.
            const int horizon =
                (int)std::lround(s.landing_cfg.t_go_max / cfg.fixed_dt) + 50;
            s.landing_coast.configure(horizon, 1e-9);
            FlightPolicy coast_policy{};
            coast_policy.kind = PredictionKind::Coast;
            NodeExecutor coast_exec{};
            s.landing_coast.cold_rebuild(s.sim, coast_policy, coast_exec,
                                         horizon);
            s.landing_ap.arm(target, s.landing_cfg, s.zero_effort);
            break;
        }

        case DebugSubsystem::None:
        default:
            break;
    }
}

// BEGIN CANONICAL ALGORITHM: inter-moon transfer cold display geometry
// Reference: docs/flight-guidance-intermoon-transfer-differential-correction-
// warm-starting-and-bounded-replanning.md (the accepted COLD solution being
// displayed); docs/physics-model-gravity.md (the arc stays WORLD/INERTIAL).

TransferColdDisplay transfer_cold_display(const BinarySystem& bin,
                                          const TransferSolution& sol,
                                          double fixed_dt, int samples) {
    TransferColdDisplay d;
    if (!sol.valid || fixed_dt <= 0.0 || samples < 1) {
        return d;
    }
    if (sol.source < 0 || sol.source >= BinarySystem::kBodyCount ||
        sol.target < 0 || sol.target >= BinarySystem::kBodyCount) {
        return d;
    }
    if (sol.arrival_epoch + 1.0e-12 < sol.solve_epoch) {
        return d;
    }

    d.valid = true;
    d.source = sol.source;
    d.target = sol.target;
    d.solve_epoch = sol.solve_epoch;
    d.arrival_epoch = sol.arrival_epoch;
    d.dep = sol.departure_state;

    const BallisticState start{sol.departure_state, sol.departure_velocity,
                               sol.solve_epoch};
    d.arc = predict_zero_thrust(bin, start, sol.arrival_epoch, fixed_dt,
                                samples);
    if (d.arc.empty()) {
        d.valid = false;
        return d;
    }
    for (const auto& st : d.arc) {
        if (!std::isfinite(st.p.x) || !std::isfinite(st.p.y) ||
            !std::isfinite(st.v.x) || !std::isfinite(st.v.y) ||
            !std::isfinite(st.t)) {
            d.valid = false;
            return d;
        }
    }

    d.arr = d.arc.back().p;
    d.target_at_arrival = bin.position(sol.target, sol.arrival_epoch);
    d.source_at_solve = bin.position(sol.source, sol.solve_epoch);

    double minx = d.dep.x, maxx = d.dep.x;
    double miny = d.dep.y, maxy = d.dep.y;
    auto include = [&](const Vec2& p) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    };
    for (const auto& st : d.arc) {
        include(st.p);
    }
    include(d.arr);
    include(d.target_at_arrival);
    include(d.source_at_solve);

    auto include_body = [&](int index, double t) {
        const Vec2 c = bin.position(index, t);
        const double r = bin.body(index).terrain.max_surface_radius();
        include({c.x - r, c.y - r});
        include({c.x + r, c.y - r});
        include({c.x - r, c.y + r});
        include({c.x + r, c.y + r});
    };
    include_body(sol.source, sol.solve_epoch);
    include_body(sol.target, sol.arrival_epoch);

    d.fit_center = {0.5 * (minx + maxx), 0.5 * (miny + maxy)};
    const double margin = 1.18;
    d.fit_half = {std::max(0.5 * (maxx - minx) * margin, 30.0),
                  std::max(0.5 * (maxy - miny) * margin, 30.0)};
    return d;
}

TransferCameraFit transfer_cold_camera_fit(
    const Vec2& center, const Vec2& half, double window_width,
    double window_height, double base_scale, double min_zoom,
    double max_zoom) {
    TransferCameraFit f;
    f.center = center;
    f.angle = 0.0;
    const double floor_zoom = std::max(min_zoom, 1.0e-3);
    const double ceiling = std::max(max_zoom, floor_zoom);
    if (half.x <= 0.0 || half.y <= 0.0 || window_width <= 0.0 ||
        window_height <= 0.0 || base_scale <= 0.0) {
        f.zoom = floor_zoom;
        return f;
    }
    const double zx = window_width / (2.0 * half.x) / base_scale;
    const double zy = window_height / (2.0 * half.y) / base_scale;
    f.zoom = std::clamp(std::min(zx, zy), floor_zoom, ceiling);
    return f;
}

// END CANONICAL ALGORITHM: inter-moon transfer cold display geometry

}  // namespace lander
