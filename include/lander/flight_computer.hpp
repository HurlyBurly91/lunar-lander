#pragma once

#include "lander/ballistic.hpp"
#include "lander/binary.hpp"
#include "lander/pred_frame.hpp"
#include "lander/sim.hpp"

#include <optional>
#include <vector>

namespace lander {

// The single M06 maneuver node. The node is an ideal planning primitive:
// its world-frame delta-v is reconstructed from a body-relative basis at the
// predicted pre-burn state, and the node impulse itself changes velocity but
// not position.
struct ManeuverNode {
    double time{0.0};
    int frame_body{0};
    double dv_prograde{0.0};
    double dv_radial{0.0};
};

// The orthonormal body-relative basis used to store and display a node's
// delta-v. `prograde` is the body-relative velocity direction (with the
// documented degenerate fallbacks), and `radial_out` is the outward-facing
// direction obtained by removing the prograde component from the body-relative
// radius vector.
struct NodeBasis {
    Vec2 prograde{};
    Vec2 radial_out{};
};

// Snap a prediction/planning time to the shared fixed-step grid.
double snap_time(double t, double dt);

// A sensible default node: 5 s ahead of `t0`, snapped to a fixed step, framed
// on `frame_body`, with zero delta-v.
ManeuverNode default_node(double t0, int frame_body, double dt);

// Compute the node basis from the predicted pre-burn state. Pure and
// deterministic, including the degenerate zero-velocity / zero-radius cases.
NodeBasis compute_node_basis(const BinarySystem& bin, double t,
                             int frame_body, const Vec2& p_pre,
                             const Vec2& v_pre);

// Reconstruct the node's world-frame delta-v from its stored components.
Vec2 node_world_dv(const ManeuverNode& node, const NodeBasis& basis);

// The first predicted entry into a body's terrain (future body position and
// tidal rotation included), if any before the horizon.
struct TerrainImpact {
    bool valid{false};
    int body{-1};
    double time{0.0};
    Vec2 position{};
};

// The closest predicted approach to the destination base pad, where the pad
// is a point fixed on the destination body's rotating surface.
struct ClosestApproach {
    bool valid{false};
    double distance{0.0};
    double time{0.0};
    Vec2 position{};
    Vec2 target_position{};
};

// A predicted local extremum of the body-relative distance curve `rho(t)` over
// the post-node branch. These are numeric prediction readouts (PE / AP), not
// classical invariant orbital elements.
struct PeriAp {
    bool valid{false};
    double distance{0.0};
    double time{0.0};
    Vec2 position{};
};

struct TrajectoryPrediction {
    // Decimated presentation samples. The first sample of `post` is the node
    // position with the ideal impulse already applied.
    std::vector<Vec2> pre;
    std::vector<Vec2> post;

    // M06-R12: full chronological world-frame samples (pre then post), each
    // carrying position, velocity, and simulation time. Parallel to / aligned
    // with the decimated `pre`/`post` above (one timed sample per decimated
    // sample). This is the display-capable substrate for the reference-frame
    // transform and the AUTO orbit-reference classifier; the existing Vec2
    // `pre`/`post` are left unchanged for existing consumers.
    std::vector<TimedTrajectorySample> timed;

    Vec2 node_position{};
    TerrainImpact impact{};
    ClosestApproach closest{};
    PeriAp peri{};
    PeriAp apo{};

    NodeBasis basis{};
    bool basis_valid{false};
    Vec2 dv_world{};
    double total_dv{0.0};
    // M06-R16: the single effective node-event epoch, in simulation seconds.
    // = max(t0, snap_time(node->time, fixed_dt)) when a node exists, 0.0 when
    // there is none. This is the exact epoch predict_trajectory anchors the
    // pre-burn endpoint, node_position, NodeBasis, dv_world, and the PRE/POST
    // boundary at. Every node graphic (the marker transform, the AUTO-frame
    // lookup, and the PGR/RAD/DV geometry) MUST consume this one value --
    // never the raw, stale ManeuverNode::time -- so an overdue node
    // (node->time < t0) stays anchored to the current prediction epoch
    // instead of drifting.
    double node_time_effective{0.0};
};

// Pure zero-thrust prediction from `start` at `t0`. With a node, the pre-node
// branch runs to the node time, the ideal impulse is applied, and the post-node
// branch runs to `t0 + horizon_seconds`. Without a node, only the pre-node
// branch is produced (to the horizon). The integrator is the shared
// semi-implicit Euler fixed-step scheme, so a zero-input live simulation and
// this predictor agree step for step.
TrajectoryPrediction predict_trajectory(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body, int destination_body,
    const std::optional<ManeuverNode>& node, double horizon_seconds,
    int target_samples = 1024);

// M06-R17: the node's EVENT state resolved at a chosen "now" (`t_now`) rather
// than at a prediction cache's rebuild epoch. It resolves the node's
// effective epoch exactly like predict_trajectory does (
// max(t_now, snap_time(node.time, fixed_dt)) ) and returns the zero-thrust
// ship state at that epoch propagated from the CURRENT ship state, the
// NodeBasis at the epoch, and the node's world delta-v reconstructed in that
// basis. When the node's scheduled epoch is already in the past, the
// effective epoch is t_now itself and no propagation is performed (the event
// is the ship's current state); otherwise a single bounded propagation runs
// from the current state to the snapped scheduled epoch. This is the cheap,
// render-cadence twin of the predictor's node anchor, for the `node-edit`
// overlay: the long-arc prediction cache is rebuilt at a bounded cadence, so
// its node anchor is the ship's state at the last rebuild epoch, while this
// helper keeps the on-scene node graphics tracking the live ship. Pure: it
// never mutates the simulation, the node, or any cache.
struct NodeEventState {
    // The "now" this event was resolved from (echoed for callers).
    double now{0.0};
    // The effective event epoch: now when the node is overdue, otherwise the
    // node's scheduled time snapped to the fixed-step grid.
    double time{0.0};
    // The zero-thrust ship state at `time`, propagated from the current
    // state (identical to it when the node is overdue).
    BallisticState state{};
    NodeBasis basis{};
    bool basis_valid{false};
    Vec2 dv_world{};
    double total_dv{0.0};
};

NodeEventState node_event_state(const BinarySystem& bin, const Config& cfg,
                                const State& ship, double t_now,
                                const ManeuverNode& node);

// Osculating local circularization around the node's frame body. Creates or
// edits the single node; never mutates live state.
std::optional<ManeuverNode> plan_circularize(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body,
    const std::optional<ManeuverNode>& existing);

// Plans an inter-body transfer node from the predicted pre-burn state.
// M06-R5 warm-first flow: when `cache` holds a valid transfer solution for the
// same route, a bounded differential correction re-aims that cached solution at
// the current node time; on any warm failure (or an empty / mismatched `cache`)
// it falls back to the full coarse COLD search. A successful solve (warm or
// cold) updates `*cache` for the next call. `cache` is optional, so call sites
// that pass nothing keep the pure cold behaviour. Returns the planned node on
// success, or `std::nullopt` when no valid transfer exists (the caller then
// leaves any existing node unchanged).
std::optional<ManeuverNode> plan_transfer(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body,
    const std::optional<ManeuverNode>& existing,
    TransferSolution* cache = nullptr);

// Velocity-match the moving destination pad. If no node exists yet, the node
// is placed near the predicted closest approach to that pad.
std::optional<ManeuverNode> plan_match_target(
    const BinarySystem& bin, const Config& config, const State& start,
    double t0, int reference_body, int destination_body,
    const std::optional<ManeuverNode>& existing);

}  // namespace lander
