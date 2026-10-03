// M06-R8: subsystem-isolation debug harness (implementation).
//
// See include/lander/debug_subsystem.hpp. This is a read-only diagnostic
// harness: it only builds deterministic startup-only fixtures and parses the
// selector. It never changes normal gameplay, canonical physics, or fixed_dt,
// and it never fixes subsystem defects (it only surfaces them).

#include "lander/debug_subsystem.hpp"

#include <chrono>
#include <cmath>

namespace lander {

namespace {

// Place the ship in a terrain-clearing circular orbit around `body` at t = 0,
// reusing the proven --orbit-demo placement: the ship is due north of the body
// centre, with tangential velocity equal to the circular-orbit speed plus the
// body's own barycentric velocity. Pure with respect to the binary (it only
// sets the ship state).
void place_in_orbit(Simulation& sim, int body) {
    const Config& cfg = sim.config();
    const BinarySystem& bin = sim.binary();
    const Terrain& terrain = sim.terrain();
    const double r = terrain.max_surface_radius() + 20.0;
    const Vec2 p0 = bin.position(body, 0.0);
    const Vec2 v0 = bin.velocity(body, 0.0);
    const double speed = std::sqrt(cfg.mu / r);
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
    r.sim_time = sim.sim_time();
    r.x = st.x;
    r.y = st.y;
    r.reference_body = sim.reference_body();
    r.target_body = sim.contract().destination_body;
    r.landed = st.landed;
    r.crashed = st.crashed;

    const BinarySystem& bin = sim.binary();
    const Terrain& terrain = sim.terrain();
    const Vec2 ref_pos = bin.position(r.reference_body, sim.sim_time());
    const Vec2 ref_vel = bin.velocity(r.reference_body, sim.sim_time());
    const double rot = local_up_angle(st, ref_pos);
    r.altitude = altitude_at(terrain, st, ref_pos, rot);
    const LocalVelocity lv = local_velocity(st, ref_pos, ref_vel);
    r.radial_velocity = lv.radial;
    r.tangential_velocity = lv.tangential;

    // Relative speed of the ship with respect to the contract-destination
    // body's inertial velocity.
    const Vec2 dest_vel = bin.velocity(r.target_body, sim.sim_time());
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

void setup_debug_scenario(DebugSubsystem mode, DebugSubsystems& s) {
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
            // A clean orbit so the live predictor (driven in the main loop)
            // has a moving target to keep receding-horizon predictions of.
            place_in_orbit(s.sim, 0);
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
            node.dv_prograde = 0.5;  // a small visible prograde burn
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

}  // namespace lander
