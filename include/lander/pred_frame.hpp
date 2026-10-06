#pragma once

// M06-R12: prediction reference-frame display and the AUTO orbit-reference
// classifier.
//
// This is a pure, SDL-free analysis / display layer over the shared
// zero-thrust prediction. It never changes physics: every sample is propagated
// in WORLD / inertial coordinates (the canonical three-body field, all three
// bodies always active, no SOI), and the only transformation applied here is an
// inertial frame shift (position -= body position, velocity -= body velocity)
// with NO rotation and no body-fixed / rotating frame. The display modes
// (WORLD / PRIMARY / COMPANION / MOONLET / AUTO) therefore differ only by the
// analysis and transform applied to the same world-space samples
// (M06-R12-P01, extended to three bodies by M06-R13).
//
// AUTO is a bounded orbit-reference classifier: over a short analysis window
// it decides whether the ship is bound to the primary, bound to the companion,
// bound to the outer moonlet, or in a world (inter-orbit) transfer, using a
// small hysteresis so the label is stable. It is driven entirely by the stored
// samples and the ephemeris and never touches the simulation, the samples, or
// any physics.

#include "lander/binary.hpp"

#include <vector>

namespace lander {

// One inertial prediction sample: the world position, world velocity, and the
// authoritative simulation time at which it is valid. This is the shared data
// substrate that every display frame reads; the physics (propagation) is
// always in world coordinates, so the display modes differ only by the
// transform / analysis applied here (M06-R12-D02).
struct TimedTrajectorySample {
    Vec2 position_world{};
    Vec2 velocity_world{};
    double time{0.0};
};

// The display reference frames (M06-R12-01, extended to three bodies by
// M06-R13). WORLD is the inertial barycentric frame (the current rosette
// view); PRIMARY / COMPANION / MOONLET are the three bodies' moving centres;
// AUTO is the label selected by the classifier. All are inertial (no rotating
// / body-fixed frame).
enum class PredFrame {
    World,
    Primary,
    Companion,
    Moonlet,
    Auto,
};

// Stable short display name for a display frame.
const char* pred_frame_name(PredFrame frame);

// The orbit-reference segment a sample is labelled with (M06-R12-03, extended
// to three bodies by M06-R13): bound to the primary, bound to the companion,
// bound to the outer moonlet, or a world (inter-orbit) transfer.
enum class RefSegment {
    World,
    Primary,
    Companion,
    Moonlet,
};

const char* ref_segment_name(RefSegment s);

// The body index a segment refers to (-1 for the world/transfer segment).
int ref_segment_body(RefSegment s);

// A sample expressed in a display frame: the inertial-shifted position and
// velocity in that frame (World returns the sample unchanged). `time` is
// frame-independent (the simulation clock).
struct FrameSample {
    Vec2 position{};
    Vec2 velocity{};
    double time{0.0};
};

// Pure inertial frame transform (M06-R12-D01): subtract the display body's
// ephemeris position / velocity at the sample time. No rotation; the body
// keeps its own prescribed motion, so a stable orbit straightens into a nearly
// fixed circle in its own frame while a transfer keeps its world shape.
FrameSample transform_to_frame(const TimedTrajectorySample& s,
                               const BinarySystem& bin, PredFrame frame);

// Named classifier constants, held in one place (M06-R12-P09) so the
// orbit-reference classifier never scatters magic numbers.
struct ClassifierParams {
    // E1: analysis window W = clamp(window_scale * T_local, min_window,
    // max_window) — a few seconds, never a full orbit.
    double window_scale{0.25};
    double min_window{2.0};
    double max_window{8.0};
    // E1: do not start a new capture if fewer than this fraction of W is
    // available at the sample (incomplete-horizon tail).
    double min_available_frac{0.75};
    // E3 raw-candidate thresholds (per body, over the window).
    double dominance_min{1.25};      // a_self / a_tidal
    double delta_theta_min{0.349};   // 20 degrees of body-relative winding
    double angular_consistency_min{0.75};  // |delta| / sum|increments|
    double radial_ratio_max{4.0};    // rho_max / rho_min in the window
    // E5 hysteresis: a transition needs this many consecutive qualifying
    // samples AND this much elapsed time.
    int confirm_count{3};
    double confirm_seconds{0.5};
};

// Per-body classifier diagnostics at one sample (for the debug panel).
struct BodyMetrics {
    double epsilon{0.0};          // specific orbital energy vs this body
    double dominance{0.0};        // a_self / a_tidal
    double delta_theta{0.0};      // unwrapped body-relative angle swept (rad)
    double angular_consistency{0.0};  // |delta| / sum|increments| (1 = clean)
    double radial_ratio{0.0};     // rho_max / rho_min in the window
};

// The per-sample diagnostics and the raw (pre-hysteresis) E4 winner.
struct SampleMetrics {
    BodyMetrics body[3]{};
    RefSegment raw{RefSegment::World};
    bool window_ok{true};  // at least one body has >= min_available_frac of W
};

// Result of the AUTO classifier: one post-hysteresis segment and one
// diagnostics record per input sample.
struct AutoClassifyResult {
    std::vector<RefSegment> segments;
    std::vector<SampleMetrics> metrics;
};

// The AUTO orbit-reference classifier (M06-R12-03, E1-E5). Pure with respect
// to its inputs (it only reads `samples` and `bin`). `prev_state` is the
// committed segment carried in from before the first sample (World by default)
// and `out` receives the per-sample segments and diagnostics. It never
// touches the simulation, the samples, or any physics.
void classify_auto(const std::vector<TimedTrajectorySample>& samples,
                   const BinarySystem& bin, const ClassifierParams& params,
                   RefSegment prev_state, AutoClassifyResult& out);

}  // namespace lander
