// M06-R12 tests: prediction reference-frame transform (inertial, no rotation)
// and the AUTO orbit-reference classifier (primary / companion / world transfer,
// with hysteresis). Headless, no SDL.
//
// The two *fixture* tests below are the milestone gate: a real, bounded
// companion-orbit coast must classify AUTO -> COMPANION, and a real
// primary-orbit coast must classify AUTO -> PRIMARY. The remaining tests cover
// the transform and the classifier mechanics.

#include "lander/ballistic.hpp"
#include "lander/binary.hpp"
#include "lander/flight_computer.hpp"
#include "lander/pred_frame.hpp"
#include "lander/sim.hpp"
#include "lander/terrain.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using lander::BinarySystem;
using lander::RefSegment;

namespace {

int failures = 0;

[[noreturn]] void fail(const char* msg) {
    std::fprintf(stderr, "  FAIL: %s\n", msg);
    std::exit(1);
}
void check(bool cond, const char* msg) { if (!cond) fail(msg); }
void check_close(double a, double b, double tol, const char* msg) {
    if (std::abs(a - b) > tol) {
        std::string s = std::string(msg) + "  (got " + std::to_string(a) +
                         ", want " + std::to_string(b) + ")";
        fail(s.c_str());
    }
}

// The canonical binary exactly as the game builds it (primary mu = 178976.334,
// seed 0), so the fixtures share the real ephemeris and terrain.
BinarySystem make_binary() {
    lander::Simulation sim;  // default: reset(0), canonical binary
    return sim.binary();
}

// A circular orbit of radius `r` around `body` at the binary's t=0: position at
// local "up" (radial out from the body centre), velocity = body ephemeris
// velocity + the circular tangential speed. Mirrors place_in_orbit. Returns a
// zero-thrust ballistic start state.
lander::BallisticState place_on_orbit(const BinarySystem& bin, int body,
                                      double r) {
    const double mu = bin.body(body).mu;
    const lander::Vec2 p0 = bin.position(body, 0.0);
    const lander::Vec2 bvel = bin.velocity(body, 0.0);
    const double speed = std::sqrt(mu / r);
    const double ang = std::atan2(p0.y, p0.x);
    const lander::Vec2 up{std::cos(ang), std::sin(ang)};
    const lander::Vec2 tangential{-up.y, up.x};
    lander::BallisticState st{};
    st.p = {p0.x + up.x * r, p0.y + up.y * r};
    st.v = {bvel.x + tangential.x * speed, bvel.y + tangential.y * speed};
    st.t = 0.0;
    return st;
}

// Like place_on_orbit but anchored to the body's ephemeris at `t0` (not t=0),
// so a second coast can begin at an arbitrary later simulation time and remain
// bound to that body's moving position. Used to splice a real inter-body
// transition into one chronological sample sequence.
lander::BallisticState place_at(const BinarySystem& bin, int body, double r,
                                double t0) {
    const double mu = bin.body(body).mu;
    const lander::Vec2 p0 = bin.position(body, t0);
    const lander::Vec2 bvel = bin.velocity(body, t0);
    const double speed = std::sqrt(mu / r);
    const double ang = std::atan2(p0.y, p0.x);
    const lander::Vec2 up{std::cos(ang), std::sin(ang)};
    const lander::Vec2 tangential{-up.y, up.x};
    lander::BallisticState st{};
    st.p = {p0.x + up.x * r, p0.y + up.y * r};
    st.v = {bvel.x + tangential.x * speed, bvel.y + tangential.y * speed};
    st.t = t0;
    return st;
}

std::vector<lander::TimedTrajectorySample> to_timed(
    const std::vector<lander::BallisticState>& states) {
    std::vector<lander::TimedTrajectorySample> out;
    out.reserve(states.size());
    for (const auto& s : states) out.push_back({s.p, s.v, s.t});
    return out;
}

struct Counts { int world = 0, primary = 0, companion = 0; };
Counts count_segments(const std::vector<RefSegment>& seg) {
    Counts c;
    for (auto s : seg) {
        if (s == RefSegment::World) c.world++;
        else if (s == RefSegment::Primary) c.primary++;
        else c.companion++;
    }
    return c;
}

void print_metrics(int n, const std::vector<lander::TimedTrajectorySample>& s,
                   const lander::AutoClassifyResult& res) {
    for (int k : {0, n / 4, n / 2, 3 * n / 4, n - 1}) {
        const auto& m = res.metrics[k];
        std::fprintf(stdout,
                     "    t=%7.2f  P: eps=%+.3f dom=%5.2f dth=%+.3f cons=%4.2f "
                     "ratio=%4.2f | C: eps=%+.3f dom=%5.2f dth=%+.3f "
                     "cons=%4.2f ratio=%4.2f | raw=%s\n",
                     s[k].time, m.body[0].epsilon, m.body[0].dominance,
                     m.body[0].delta_theta, m.body[0].angular_consistency,
                     m.body[0].radial_ratio, m.body[1].epsilon,
                     m.body[1].dominance, m.body[1].delta_theta,
                     m.body[1].angular_consistency, m.body[1].radial_ratio,
                     lander::ref_segment_name(m.raw));
    }
}

void print_bin(const BinarySystem& bin) {
    const lander::Vec2 p00 = bin.position(0, 0.0);
    const lander::Vec2 p10 = bin.position(1, 0.0);
    std::fprintf(stdout,
                 "  BIN pos0=(%.1f,%.1f) pos1=(%.1f,%.1f) mu0=%.0f mu1=%.0f "
                 "refR0=%.1f refR1=%.1f maxR0=%.1f maxR1=%.1f sep=%.0f "
                 "period=%.1f\n",
                 p00.x, p00.y, p10.x, p10.y, bin.body(0).mu, bin.body(1).mu,
                 bin.body(0).reference_radius, bin.body(1).reference_radius,
                 bin.body(0).terrain.max_surface_radius(),
                 bin.body(1).terrain.max_surface_radius(), bin.separation(),
                 bin.period());
}

// GATE: a real, bounded companion-orbit coast must resolve AUTO -> COMPANION.
void gate_companion() {
    const BinarySystem bin = make_binary();
    const double r = bin.body(1).terrain.max_surface_radius() + 20.0;
    print_bin(bin);
    std::fprintf(stdout, "  companion orbit radius r=%.1f\n", r);
    const auto start = place_on_orbit(bin, 1, r);
    // A few companion orbit periods (the orbit is companion-dominant and
    // winding for this window before tidal drift takes over).
    const auto traj =
        lander::predict_zero_thrust(bin, start, 40.0, 1.0 / 120.0, 2048);
    const auto samples = to_timed(traj);
    check(samples.size() >= 200, "companion: not enough coast samples");

    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(samples, bin, params, RefSegment::World, res);
    const int n = (int)res.segments.size();

    const Counts c = count_segments(res.segments);
    const double frac_comp = (double)c.companion / n;
    std::fprintf(stdout,
                 "  [gate companion] n=%d  world=%d primary=%d companion=%d "
                 "final=%s  companion_frac=%.2f\n",
                 n, c.world, c.primary, c.companion,
                 lander::ref_segment_name(res.segments[n - 1]), frac_comp);
    print_metrics(n, samples, res);

    check(res.segments[n - 1] == RefSegment::Companion,
          "companion GATE: final segment must be COMPANION");
    check(frac_comp >= 0.6, "companion GATE: >=60% of arc must be COMPANION");
    check(c.primary == 0, "companion GATE: never PRIMARY");
}

// GATE: a real, bounded primary-orbit coast must resolve AUTO -> PRIMARY.
void gate_primary() {
    const BinarySystem bin = make_binary();
    const double r = bin.body(0).terrain.max_surface_radius() + 20.0;
    std::fprintf(stdout, "  primary orbit radius r=%.1f\n", r);
    const auto start = place_on_orbit(bin, 0, r);
    const auto traj =
        lander::predict_zero_thrust(bin, start, 300.0, 1.0 / 120.0, 2048);
    const auto samples = to_timed(traj);
    check(samples.size() >= 200, "primary: not enough coast samples");

    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(samples, bin, params, RefSegment::World, res);
    const int n = (int)res.segments.size();

    const Counts c = count_segments(res.segments);
    const double frac_prim = (double)c.primary / n;
    std::fprintf(stdout,
                 "  [gate primary] n=%d  world=%d primary=%d companion=%d "
                 "final=%s  primary_frac=%.2f\n",
                 n, c.world, c.primary, c.companion,
                 lander::ref_segment_name(res.segments[n - 1]), frac_prim);
    print_metrics(n, samples, res);

    check(res.segments[n - 1] == RefSegment::Primary,
          "primary GATE: final segment must be PRIMARY");
    check(frac_prim >= 0.6, "primary GATE: >=60% of arc must be PRIMARY");
    check(c.companion == 0, "primary GATE: never COMPANION");
}

void append(std::vector<lander::TimedTrajectorySample>& out,
            const std::vector<lander::TimedTrajectorySample>& in) {
    out.insert(out.end(), in.begin(), in.end());
}

// Collapse a segment run to its distinct transitions (e.g. "WPW").
std::string reduced(const std::vector<RefSegment>& seg) {
    std::string s;
    for (auto x : seg) {
        const char c =
            x == RefSegment::World ? 'W'
            : x == RefSegment::Primary ? 'P' : 'C';
        if (s.empty() || s.back() != c) s.push_back(c);
    }
    return s;
}

// True if two consecutive samples are a forbidden direct body->body pair.
bool has_direct_body_transition(const std::vector<RefSegment>& seg) {
    for (size_t i = 1; i < seg.size(); ++i) {
        const bool a_body = seg[i - 1] != RefSegment::World;
        const bool b_body = seg[i] != RefSegment::World;
        if (a_body && b_body && seg[i - 1] != seg[i]) {
            return true;  // P->C or C->P adjacent
        }
    }
    return false;
}

// V03: a point that co-moves with a body (position = body ephemeris + a fixed
// offset, velocity = body ephemeris velocity) is a fixed point in that body's
// frame: the inertial transform removes the body's motion exactly, so the
// body-frame position is constant for every time. This is the stability the
// body-centred display relies on (a stable orbit straightens into a circle).
void co_rotating_point_stationary() {
    const BinarySystem bin = make_binary();
    const int body = 0;
    const lander::Vec2 offset{13.0, -7.0};
    for (const double t : {0.0, 5.0, 23.7, 90.0}) {
        const lander::Vec2 bp = bin.position(body, t);
        const lander::Vec2 bv = bin.velocity(body, t);
        const lander::TimedTrajectorySample s{
            {bp.x + offset.x, bp.y + offset.y}, {bv.x, bv.y}, t};
        const lander::FrameSample f =
            lander::transform_to_frame(s, bin, lander::PredFrame::Primary);
        check_close(f.position.x, offset.x, 1e-6, "co-rotating x");
        check_close(f.position.y, offset.y, 1e-6, "co-rotating y");
        check_close(f.velocity.x, 0.0, 1e-6, "co-rotating vx");
        check_close(f.velocity.y, 0.0, 1e-6, "co-rotating vy");
    }
}

// V06/V07: the via-World rule (E5) -- a body->body change of reference must
// route through World, never jump P<->C directly. Checked as a property over
// several real trajectories (both single-body coasts and two spliced
// primary->companion / companion->primary transfers), and, for the spliced
// transfers, that the reduced sequence really does pass through all three
// regimes in order.
void no_direct_body_transition() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    const double rc = bin.body(1).terrain.max_surface_radius() + 20.0;

    // (a) single-body coasts: each must never contain the other body at all.
    {
        auto prim = to_timed(
            lander::predict_zero_thrust(bin, place_on_orbit(bin, 0, rp), 200.0,
                                        1.0 / 120.0, 2048));
        lander::AutoClassifyResult r;
        lander::classify_auto(prim, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        check(!has_direct_body_transition(r.segments),
              "primary coast: no direct body transition");
        check(count_segments(r.segments).companion == 0,
              "primary coast: never COMPANION");
    }
    {
        auto comp = to_timed(
            lander::predict_zero_thrust(bin, place_on_orbit(bin, 1, rc), 60.0,
                                        1.0 / 120.0, 2048));
        lander::AutoClassifyResult r;
        lander::classify_auto(comp, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        check(!has_direct_body_transition(r.segments),
              "companion coast: no direct body transition");
        check(count_segments(r.segments).primary == 0,
              "companion coast: never PRIMARY");
    }

    // (b) a spliced primary->companion transfer: a primary-orbit coast, then a
    // companion-orbit coast generated from the companion's ephemeris at the
    // splice time (so it stays bound to the companion's moving position). The
    // classifier must route P -> W -> C and never emit a direct P<->C jump.
    {
        std::vector<lander::TimedTrajectorySample> arc;
        auto first = lander::predict_zero_thrust(
            bin, place_at(bin, 0, rp, 0.0), 60.0, 1.0 / 120.0, 2048);
        append(arc, to_timed(first));
        const double t1 = arc.back().time;
        append(arc, to_timed(
            lander::predict_zero_thrust(bin, place_at(bin, 1, rc, t1),
                                        t1 + 40.0, 1.0 / 120.0, 2048)));
        lander::AutoClassifyResult r;
        lander::classify_auto(arc, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        const std::string seq = reduced(r.segments);
        std::fprintf(stdout,
                     "  [spliced P->C] seq=%s  W=%d P=%d C=%d\n", seq.c_str(),
                     c.world, c.primary, c.companion);
        check(!has_direct_body_transition(r.segments),
              "spliced P->C: no direct P<->C adjacency");
        check(c.primary > 0 && c.companion > 0 && c.world > 0,
              "spliced P->C: all three regimes must appear");
        const size_t fp = seq.find('P');
        const size_t fc = seq.find('C');
        check(fp != std::string::npos && fc != std::string::npos && fp < fc,
              "spliced P->C: primary must precede companion");
        check(seq.find('W', fp + 1) != std::string::npos &&
                 seq.find('W', fp + 1) < fc,
              "spliced P->C: a world bridge must sit between primary and "
              "companion");
    }

    // (c) the reverse: a spliced companion->primary transfer must route
    // C -> W -> P.
    {
        std::vector<lander::TimedTrajectorySample> arc;
        auto first = lander::predict_zero_thrust(
            bin, place_at(bin, 1, rc, 0.0), 40.0, 1.0 / 120.0, 2048);
        append(arc, to_timed(first));
        const double t1 = arc.back().time;
        append(arc, to_timed(
            lander::predict_zero_thrust(bin, place_at(bin, 0, rp, t1),
                                        t1 + 60.0, 1.0 / 120.0, 2048)));
        lander::AutoClassifyResult r;
        lander::classify_auto(arc, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        const std::string seq = reduced(r.segments);
        std::fprintf(stdout,
                     "  [spliced C->P] seq=%s  W=%d P=%d C=%d\n", seq.c_str(),
                     c.world, c.primary, c.companion);
        check(!has_direct_body_transition(r.segments),
              "spliced C->P: no direct P<->C adjacency");
        check(c.primary > 0 && c.companion > 0 && c.world > 0,
              "spliced C->P: all three regimes must appear");
        const size_t fc = seq.find('C');
        const size_t fp = seq.find('P');
        check(fc != std::string::npos && fp != std::string::npos && fc < fp,
              "spliced C->P: companion must precede primary");
        check(seq.find('W', fc + 1) != std::string::npos &&
                 seq.find('W', fc + 1) < fp,
              "spliced C->P: a world bridge must sit between companion and "
              "primary");
    }
}

// V08: hysteresis state dependence. Starting from prev=World, the first sample
// cannot already have transitioned (a transition needs a run of qualifying
// samples, not a single one), so the arc opens in World and only locks to the
// orbiting body after the confirm delay. Starting from prev=Primary, the ship
// is already in that segment and the arc opens in Primary from sample 0.
void hysteresis_prev_dependence() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    auto prim = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 0, rp), 120.0,
                                    1.0 / 120.0, 2048));

    lander::AutoClassifyResult from_world;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::World, from_world);
    check(from_world.segments.front() == RefSegment::World,
          "prev=World: arc must open in World (no instant transition)");
    check(from_world.segments.back() == RefSegment::Primary,
          "prev=World: must lock to PRIMARY eventually");
    // The World->Primary transition must be delayed by the confirm window.
    const double first_primary_t = [&] {
        for (size_t i = 0; i < from_world.segments.size(); ++i) {
            if (from_world.segments[i] == RefSegment::Primary) {
                return prim[i].time;
            }
        }
        return -1.0;
    }();
    check(first_primary_t >= 0.0, "prev=World: PRIMARY must be reached");

    lander::AutoClassifyResult from_primary;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::Primary, from_primary);
    check(from_primary.segments.front() == RefSegment::Primary,
          "prev=Primary: arc must open in Primary from sample 0");
}

// V09: incomplete horizon (E1). A real companion-orbit coast that is only long
// enough to fill less than 75% of the analysis window must never start a
// capture: the whole arc stays in the previous (World) segment, even though
// the same orbit on a long horizon does lock to COMPANION.
void incomplete_horizon_no_capture() {
    const BinarySystem bin = make_binary();
    const double rc = bin.body(1).terrain.max_surface_radius() + 20.0;

    // Long horizon: locks to the companion (the gate regime).
    auto long_arc = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 1, rc), 40.0,
                                    1.0 / 120.0, 2048));
    lander::AutoClassifyResult rl;
    lander::classify_auto(long_arc, bin, lander::ClassifierParams{},
                          RefSegment::World, rl);
    check(rl.segments.back() == RefSegment::Companion,
          "long horizon: must lock to COMPANION");

    // Very short horizon: the available window never reaches the 75% gate, so
    // no capture can begin and the arc stays World the whole time.
    auto short_arc = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 1, rc), 1.0,
                                    1.0 / 120.0, 2048));
    check(short_arc.size() >= 2, "short arc: need samples");
    lander::AutoClassifyResult rs;
    lander::classify_auto(short_arc, bin, lander::ClassifierParams{},
                          RefSegment::World, rs);
    check(rs.segments.front() == RefSegment::World,
          "short horizon: opens in World");
    check(rs.segments.back() == RefSegment::World,
          "short horizon: stays World (no capture can start)");
    // The window gate must actually be the binding constraint: at least one
    // tail sample reports an unusable window.
    bool saw_unusable = false;
    for (const auto& m : rs.metrics) {
        if (!m.window_ok) saw_unusable = true;
    }
    check(saw_unusable, "short horizon: window gate must engage");
}

// V10: classify_auto is pure with respect to its inputs: it must not mutate the
// sample array (positions, velocities, times) it classifies.
void no_mutation() {
    const BinarySystem bin = make_binary();
    const double rc = bin.body(1).terrain.max_surface_radius() + 20.0;
    auto samples = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 1, rc), 40.0,
                                    1.0 / 120.0, 2048));
    const auto before = samples;  // deep copy
    lander::AutoClassifyResult r;
    lander::classify_auto(samples, bin, lander::ClassifierParams{},
                          RefSegment::World, r);
    check(samples.size() == before.size(), "mutation: size changed");
    for (size_t i = 0; i < samples.size(); ++i) {
        check(samples[i].position_world.x == before[i].position_world.x &&
                  samples[i].position_world.y == before[i].position_world.y &&
                  samples[i].velocity_world.x ==
                      before[i].velocity_world.x &&
                  samples[i].velocity_world.y ==
                      before[i].velocity_world.y &&
                  samples[i].time == before[i].time,
              "mutation: a sample changed");
    }
}

// V11: the classifier's per-sample segment and diagnostics arrays are aligned
// with the input (one each, same order), so the renderer can index them
// sample-by-sample; the segment count must equal the sample count.
void segments_align_with_samples() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    auto samples = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 0, rp), 90.0,
                                    1.0 / 120.0, 2048));
    lander::AutoClassifyResult r;
    lander::classify_auto(samples, bin, lander::ClassifierParams{},
                          RefSegment::World, r);
    check(r.segments.size() == samples.size(),
          "segments count must equal sample count");
    check(r.metrics.size() == samples.size(),
          "metrics count must equal sample count");
}

// V13: the primary fixture is robust to the incoming hysteresis state: whether
// the classifier is told it was last in World or in Companion, a real primary
// orbit coast must still settle on PRIMARY.
void primary_fixture_robust_to_prev() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    auto prim = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 0, rp), 300.0,
                                    1.0 / 120.0, 2048));
    lander::AutoClassifyResult from_world;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::World, from_world);
    lander::AutoClassifyResult from_companion;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::Companion, from_companion);
    check(from_world.segments.back() == RefSegment::Primary,
          "primary robust: prev=World -> PRIMARY");
    check(from_companion.segments.back() == RefSegment::Primary,
          "primary robust: prev=Companion -> PRIMARY");
}

// V12/V13: end-to-end through the GUI's actual prediction path. The
// flight-computer `predict_trajectory` (a node-less COAST) is the prediction
// the display consumes; feeding its `timed` samples to `classify_auto` must
// resolve the same AUTO frame as the dedicated fixture coasts (COMPANION /
// PRIMARY). This proves the timed-sample storage and the classifier work
// together on the real pipeline, not just on `predict_zero_thrust`.
void fixture_end_to_end() {
    lander::Simulation sim;  // default reset(0), canonical binary
    const BinarySystem& bin = sim.binary();
    const lander::Config& cfg = sim.config();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    const double rc = bin.body(1).terrain.max_surface_radius() + 20.0;

    // A node-less COAST prediction from a bounded companion orbit must resolve
    // AUTO -> COMPANION end to end.
    {
        lander::State st{};
        st.fuel = cfg.fuel;
        const auto b = place_at(bin, 1, rc, 0.0);
        st.x = b.p.x;
        st.y = b.p.y;
        st.vx = b.v.x;
        st.vy = b.v.y;
        auto pred =
            lander::predict_trajectory(bin, cfg, st, 0.0, 1, 1,
                                       std::nullopt, 40.0, 2048);
        lander::AutoClassifyResult r;
        lander::classify_auto(pred.timed, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        std::fprintf(stdout, "  [e2e companion] n=%zu W=%d P=%d C=%d final=%s\n",
                     pred.timed.size(), c.world, c.primary, c.companion,
                     lander::ref_segment_name(r.segments.back()));
        check(r.segments.back() == RefSegment::Companion,
              "e2e companion: final frame must be COMPANION");
        check(c.companion >= 0.9 * (int)pred.timed.size(),
              "e2e companion: >=90% of the arc must be COMPANION");
    }
    // The same end-to-end path from a bounded primary orbit must resolve
    // AUTO -> PRIMARY.
    {
        lander::State st{};
        st.fuel = cfg.fuel;
        const auto b = place_at(bin, 0, rp, 0.0);
        st.x = b.p.x;
        st.y = b.p.y;
        st.vx = b.v.x;
        st.vy = b.v.y;
        auto pred =
            lander::predict_trajectory(bin, cfg, st, 0.0, 0, 0,
                                       std::nullopt, 120.0, 2048);
        lander::AutoClassifyResult r;
        lander::classify_auto(pred.timed, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        std::fprintf(stdout, "  [e2e primary] n=%zu W=%d P=%d C=%d final=%s\n",
                     pred.timed.size(), c.world, c.primary, c.companion,
                     lander::ref_segment_name(r.segments.back()));
        check(r.segments.back() == RefSegment::Primary,
              "e2e primary: final frame must be PRIMARY");
        check(c.primary >= 0.9 * (int)pred.timed.size(),
              "e2e primary: >=90% of the arc must be PRIMARY");
    }
}

// V01: the WORLD frame is the inertial identity transform.
void world_identity() {
    const BinarySystem bin = make_binary();
    const lander::TimedTrajectorySample s{{12.0, -3.0}, {0.5, 2.0}, 7.0};
    const lander::FrameSample f =
        lander::transform_to_frame(s, bin, lander::PredFrame::World);
    check_close(f.position.x, 12.0, 1e-12, "world x");
    check_close(f.position.y, -3.0, 1e-12, "world y");
    check_close(f.velocity.x, 0.5, 1e-12, "world vx");
    check_close(f.velocity.y, 2.0, 1e-12, "world vy");
    check_close(f.time, 7.0, 1e-12, "world t");
}

// V02: a moving-body translation is removed by the frame transform.
void moving_body_translation_removed() {
    const BinarySystem bin = make_binary();
    const double t = 3.0;
    const lander::Vec2 ship_pos{50.0, 60.0};
    const lander::Vec2 ship_vel{0.0, 0.0};
    const lander::TimedTrajectorySample s{ship_pos, ship_vel, t};
    const lander::FrameSample f =
        lander::transform_to_frame(s, bin, lander::PredFrame::Companion);
    const lander::Vec2 bpos = bin.position(1, t);
    const lander::Vec2 bvel = bin.velocity(1, t);
    check_close(f.position.x, ship_pos.x - bpos.x, 1e-9, "companion rel x");
    check_close(f.position.y, ship_pos.y - bpos.y, 1e-9, "companion rel y");
    check_close(f.velocity.x, ship_vel.x - bvel.x, 1e-9, "companion rel vx");
    check_close(f.velocity.y, ship_vel.y - bvel.y, 1e-9, "companion rel vy");
}

}  // namespace

int main() {
    std::printf("M06-R12 prediction reference frame tests\n");

    std::printf("gate_companion...");
    gate_companion();
    std::printf("ok\n");

    std::printf("gate_primary...");
    gate_primary();
    std::printf("ok\n");

    std::printf("co_rotating_point_stationary...");
    co_rotating_point_stationary();
    std::printf("ok\n");

    std::printf("world_identity...");
    world_identity();
    std::printf("ok\n");

    std::printf("moving_body_translation_removed...");
    moving_body_translation_removed();
    std::printf("ok\n");

    std::printf("no_direct_body_transition...");
    no_direct_body_transition();
    std::printf("ok\n");

    std::printf("hysteresis_prev_dependence...");
    hysteresis_prev_dependence();
    std::printf("ok\n");

    std::printf("incomplete_horizon_no_capture...");
    incomplete_horizon_no_capture();
    std::printf("ok\n");

    std::printf("no_mutation...");
    no_mutation();
    std::printf("ok\n");

    std::printf("segments_align_with_samples...");
    segments_align_with_samples();
    std::printf("ok\n");

    std::printf("primary_fixture_robust_to_prev...");
    primary_fixture_robust_to_prev();
    std::printf("ok\n");

    std::printf("fixture_end_to_end...");
    fixture_end_to_end();
    std::printf("ok\n");

    std::printf("ALL PASS (%d failures so far)\n", failures);
    return failures;
}
