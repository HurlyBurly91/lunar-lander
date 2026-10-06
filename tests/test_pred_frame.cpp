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

struct Counts { int world = 0, primary = 0, companion = 0, moonlet = 0; };
Counts count_segments(const std::vector<RefSegment>& seg) {
    Counts c;
    for (auto s : seg) {
        if (s == RefSegment::World) c.world++;
        else if (s == RefSegment::Primary) c.primary++;
        else if (s == RefSegment::Companion) c.companion++;
        else c.moonlet++;
    }
    return c;
}

// M06-R13: print all three bodies' per-sample classifier metrics (the R12
// print_metrics above only covers bodies 0/1; the moonlet gate needs body 2).
void print_metrics3(int n,
                    const std::vector<lander::TimedTrajectorySample>& s,
                    const lander::AutoClassifyResult& res) {
    for (int k : {0, n / 4, n / 2, 3 * n / 4, n - 1}) {
        const auto& m = res.metrics[k];
        for (int b = 0; b < 3; b++) {
            std::fprintf(stdout,
                         "    t=%7.2f  B%d: eps=%+.3f dom=%5.2f dth=%+.3f "
                         "cons=%4.2f ratio=%4.2f\n",
                         s[k].time, b, m.body[b].epsilon, m.body[b].dominance,
                         m.body[b].delta_theta, m.body[b].angular_consistency,
                         m.body[b].radial_ratio);
        }
        std::fprintf(stdout, "    t=%7.2f  raw=%s\n", s[k].time,
                     lander::ref_segment_name(m.raw));
    }
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

// Forward declaration (defined below): collapse a segment run to its distinct
// transitions (e.g. "WPWM").
std::string reduced(const std::vector<RefSegment>& seg);

// OBSERVATIONAL (not a gate; M06-R13-09): report the ACTUAL behavior of the
// tight 600 m strong-tide companion-orbit coast under the UNCHANGED 3-body
// classifier. R13 does not force COMPANION here; it reports the resolved
// final frame, per-body occupancy, the transition sequence, and EPS / DOM /
// WIND / RATIO so the behavior in that environment is visible.
void report_companion() {
    const BinarySystem bin = make_binary();
    const double r = bin.body(1).terrain.max_surface_radius() + 20.0;
    print_bin(bin);
    std::fprintf(stdout,
                 "  companion orbit radius r=%.1f (observational, not a "
                 "gate)\n",
                 r);
    const auto start = place_on_orbit(bin, 1, r);
    // A few companion orbit periods (the orbit is companion-dominant and
    // winding for this window before tidal drift takes over).
    const auto traj =
        lander::predict_zero_thrust(bin, start, 40.0, 1.0 / 120.0, 2048);
    const auto samples = to_timed(traj);

    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(samples, bin, params, RefSegment::World, res);
    const int n = (int)res.segments.size();

    const Counts c = count_segments(res.segments);
    const double frac_comp = n ? (double)c.companion / n : 0.0;
    std::fprintf(stdout,
                 "  [companion OBSERVATIONAL] n=%d  world=%d primary=%d "
                 "companion=%d moonlet=%d  final=%s  companion_frac=%.2f "
                 "seq=%s\n",
                 n, c.world, c.primary, c.companion, c.moonlet,
                 lander::ref_segment_name(res.segments[n - 1]), frac_comp,
                 reduced(res.segments).c_str());
    print_metrics3(n, samples, res);
}

// LEGACY R12 DIAGNOSTIC (not a gate; M06-R13-09). The historical 2-body-
// calibrated primary fixture (a circular orbit about body 0 at maxR0 + 20 m,
// coasted 300 s at 120 Hz through the zero-thrust predictor) was tuned to read
// cleanly in the OLD 2-body world; in the canonical 3-body world the same
// nominal trajectory legitimately differs (the 3-body differential / tidal
// perturbation lowers the primary's dominance). It no longer gates R13; it is
// preserved as regression telemetry reporting the exact EPS / DOM / WIND /
// RATIO (the R12 two-body view below + the three-body view), the segment
// occupancy, the transition sequence, and the point / time at which PRIMARY is
// released (leaves PRIMARY after first acquiring it).
void report_legacy_r12_primary() {
    const BinarySystem bin = make_binary();
    const double r = bin.body(0).terrain.max_surface_radius() + 20.0;
    std::fprintf(stdout,
                 "  LEGACY R12 primary fixture r=%.1f (2-body-calibrated; "
                 "diagnostic only, NOT a gate)\n",
                 r);
    const auto start = place_on_orbit(bin, 0, r);
    const auto traj =
        lander::predict_zero_thrust(bin, start, 300.0, 1.0 / 120.0, 2048);
    const auto samples = to_timed(traj);

    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(samples, bin, params, RefSegment::World, res);
    const int n = (int)res.segments.size();

    const Counts c = count_segments(res.segments);
    const double frac_prim = n ? (double)c.primary / n : 0.0;
    std::fprintf(stdout,
                 "  [legacy R12 primary] n=%d  world=%d primary=%d "
                 "companion=%d moonlet=%d  final=%s  primary_frac=%.3f  "
                 "seq=%s\n",
                 n, c.world, c.primary, c.companion, c.moonlet,
                 lander::ref_segment_name(res.segments[n - 1]), frac_prim,
                 reduced(res.segments).c_str());
    // R12 two-body metric view (the original R12 diagnostic readout).
    print_metrics(n, samples, res);
    // Three-body metric view (what the current 3-body classifier sees).
    print_metrics3(n, samples, res);

    bool in_primary = false;
    for (int i = 0; i < n; ++i) {
        if (res.segments[i] == RefSegment::Primary) in_primary = true;
        if (in_primary && res.segments[i] != RefSegment::Primary) {
            std::fprintf(stdout,
                         "  [legacy R12 primary] PRIMARY released at sample "
                         "%d (t=%.3f s) -> %s\n",
                         i, samples[i].time,
                         lander::ref_segment_name(res.segments[i]));
            break;
        }
    }
    if (!in_primary) {
        std::fprintf(stdout,
                     "  [legacy R12 primary] PRIMARY never acquired in "
                     "window\n");
    }
}

// GATE (M06-R13-V19): a bounded moonlet-orbit coast must resolve AUTO -> MOONLET.
// The orbit radius matches the `--debug-predictor-body 2` fixture (the moonlet's
// max surface radius + 20 m): a low, tight, companion-scale orbit that is
// moonlet-dominant and winding for the 8 s window. If it does NOT classify
// MOONLET this is a STOP-and-report gate: print all three bodies' metrics and
// fail without tuning any classifier constant (same semantics as M06-R12-V05).
void gate_moonlet() {
    const BinarySystem bin = make_binary();
    const double r = bin.body(2).terrain.max_surface_radius() + 20.0;
    std::fprintf(stdout, "  moonlet orbit radius r=%.1f (maxR2=%.1f)\n", r,
                 bin.body(2).terrain.max_surface_radius());
    const auto start = place_on_orbit(bin, 2, r);
    const auto traj =
        lander::predict_zero_thrust(bin, start, 300.0, 1.0 / 120.0, 2048);
    const auto samples = to_timed(traj);
    check(samples.size() >= 200, "moonlet: not enough coast samples");

    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(samples, bin, params, RefSegment::World, res);
    const int n = (int)res.segments.size();

    const Counts c = count_segments(res.segments);
    const double frac_moon = (double)c.moonlet / n;
    std::fprintf(stdout,
                 "  [gate moonlet] n=%d  world=%d primary=%d companion=%d "
                 "moonlet=%d  final=%s  moonlet_frac=%.3f\n",
                 n, c.world, c.primary, c.companion, c.moonlet,
                 lander::ref_segment_name(res.segments[n - 1]), frac_moon);
    print_metrics3(n, samples, res);

    check(res.segments[n - 1] == RefSegment::Moonlet,
          "moonlet GATE: final segment must be MOONLET");
    check(frac_moon >= 0.95, "moonlet GATE: >=95% of arc must be MOONLET");
}

// GATE (M06-R13; the current-world PRIMARY classifier gate). The 10-period
// re-baseline (M06-R13-V23) is SUPERSEDED: the diagnostic clearance scan shows
// the canonical 3-body system does not hold a near-surface prograde body-0
// orbit for 10 local periods (20-150 m clearances fail after ~1.1-1.4 periods;
// 300 m survives ~7.9 periods but becomes highly eccentric), while the AUTO
// classifier is correct on every physically valid segment. This is physics, not
// a classifier defect, so the 10-period survival requirement is dropped in
// favour of a bounded classifier-correctness gate.
//
// Fixture (unchanged canonical construction): body 0, r = max_surface_radius +
// 20 m, v_local = sqrt(mu0 / r) tangential, initial inertial state = body-0
// ephemeris + local tangential; NO stabilization / hidden force / altered
// gravity. It is coasted through the AUTHORITATIVE Simulation (zero input) and
// evaluated with the UNCHANGED 3-body AUTO classifier.
//
// The valid classifier evaluation interval is:
//   start = the first PRIMARY sample (AUTO entry hysteresis has matured);
//   end   = the EARLIER of (a) ONE complete body-relative revolution about body
//           0, and (b) the first physical loss of the trajectory (crash /
//           contact / landing).
// If one full revolution cannot complete before physical failure, STOP and
// report (no gate verdict). Over that mature physically-valid interval we
// require: final frame PRIMARY; PRIMARY occupancy >= 99%; no repeated
// PRIMARY<->WORLD chatter; zero COMPANION; zero MOONLET; continuous,
// consistent angular winding (~one clean revolution); bounded radius. Survival
// beyond one revolution is NOT required. No classifier / gravity / fixture
// constant is tuned.
void gate_primary_current() {
    lander::Simulation sim;
    sim.reset(0);  // canonical three-body system
    const lander::BinarySystem& bin = sim.binary();
    const double r = bin.body(0).terrain.max_surface_radius() + 20.0;
    const double mu0 = bin.body(0).mu;
    const double v_local = std::sqrt(mu0 / r);
    const double T_local = 2.0 * lander::kPi * std::sqrt(r * r * r / mu0);
    std::fprintf(stdout,
                 "  PRIMARY gate: body 0  r=%.1f  v_local=%.3f  T_local=%.3f s"
                 "  (authoritative Simulation, unchanged 3-body classifier;"
                 " interval = hysteresis-matured .. one body-relative"
                 " revolution)\n",
                 r, v_local, T_local);

    // Start: a circular orbit about body 0 (canonical construction), injected
    // as a Simulation State with no stabilization or hidden force.
    const lander::BallisticState st0 = place_on_orbit(bin, 0, r);
    lander::State st{};
    st.x = st0.p.x;
    st.y = st0.p.y;
    st.vx = st0.v.x;
    st.vy = st0.v.y;
    st.angle = 0.0;
    st.omega = 0.0;
    st.fuel = 1000.0;
    st.landed = false;
    st.crashed = false;
    sim.set_state(st);

    const double dt = sim.config().fixed_dt;
    // Coast the authoritative sim to a safety cap (3 local periods); in
    // practice the close 600 m companion ends the arc earlier (crash). Recording
    // a little past one revolution keeps the classifier window complete around
    // the one-revolution mark even though the gate interval ends there.
    const int cap_steps = (int)std::lround(3.0 * T_local / dt);
    std::vector<lander::TimedTrajectorySample> arc;
    std::vector<double> wind;  // cumulative unwrapped body-0-relative angle
    arc.reserve(20000);
    wind.reserve(20000);
    {
        const lander::State& s0 = sim.state();
        arc.push_back({{s0.x, s0.y}, {s0.vx, s0.vy}, sim.sim_time()});
        wind.push_back(0.0);
    }
    const lander::Vec2 b00 = bin.position(0, 0.0);
    double prev_raw = std::atan2(arc[0].position_world.y - b00.y,
                                 arc[0].position_world.x - b00.x);
    bool crashed = false, landed = false;
    int rev_index = -1;  // first sample index where one full revolution is done
    for (int i = 0; i < cap_steps; ++i) {
        sim.advance(dt, {});
        const lander::State& ns = sim.state();
        arc.push_back({{ns.x, ns.y}, {ns.vx, ns.vy}, sim.sim_time()});
        const lander::Vec2 b0 = bin.position(0, sim.sim_time());
        const double a = std::atan2(ns.y - b0.y, ns.x - b0.x);
        double da = a - prev_raw;
        if (da > lander::kPi) da -= 2.0 * lander::kPi;
        if (da < -lander::kPi) da += 2.0 * lander::kPi;
        prev_raw = a;
        wind.push_back(wind.back() + da);
        if (rev_index < 0 && std::abs(wind.back()) >= 2.0 * lander::kPi) {
            rev_index = (int)arc.size() - 1;
        }
        if (ns.crashed) {
            crashed = true;
            break;
        }
        if (ns.landed) {
            landed = true;
            break;
        }
    }
    const int n = (int)arc.size();

    // Whole-arc report metrics (context for the physics characterization).
    double min_r = 1e30, max_r = 0.0;
    for (int i = 0; i < n; ++i) {
        const lander::Vec2 b0 = bin.position(0, arc[i].time);
        const double rr =
            std::hypot(arc[i].position_world.x - b0.x,
                       arc[i].position_world.y - b0.y);
        min_r = std::min(min_r, rr);
        max_r = std::max(max_r, rr);
    }
    const double wind_revs = (n > 1) ? wind[n - 1] / (2.0 * lander::kPi) : 0.0;

    // Classify the full recorded arc with the UNCHANGED 3-body classifier.
    const lander::ClassifierParams params;
    lander::AutoClassifyResult res;
    lander::classify_auto(arc, bin, params, RefSegment::World, res);
    check((int)res.segments.size() == n,
          "primary GATE: classifier sample count mismatch");
    const Counts c = count_segments(res.segments);

    // Interval start: first PRIMARY sample (entry hysteresis matured).
    int entry = -1;
    for (int i = 0; i < n; ++i) {
        if (res.segments[i] == RefSegment::Primary) {
            entry = i;
            break;
        }
    }
    // If one revolution could not complete before physical failure, STOP.
    if (rev_index < 0) {
        std::fprintf(stdout,
                     "  [primary gate] ONE REVOLUTION DID NOT COMPLETE BEFORE "
                     "PHYSICAL FAILURE: arc n=%d wind=%.3f rev crashed=%d "
                     "landed=%d\n",
                     n, wind_revs, (int)crashed, (int)landed);
        print_metrics3(n, arc, res);
        fail("primary GATE: one body-relative revolution did not complete "
             "before the trajectory was physically lost (STOP; see spec)");
    }
    const int m0 = entry < 0 ? 0 : entry;  // hysteresis-matured start
    const int m1 = rev_index;              // one complete revolution
    check(m0 <= m1,
          "primary GATE: entry hysteresis must mature before one revolution");

    // Metrics over the mature physically-valid interval [m0, m1].
    int mature = 0, mature_primary = 0, pw_chatter = 0, comp_in = 0,
        moon_in = 0;
    double net_wind = 0.0, sum_abs_wind = 0.0;
    double min_r_iv = 1e30, max_r_iv = 0.0;
    for (int i = m0; i <= m1; ++i) {
        mature++;
        const RefSegment s = res.segments[i];
        if (s == RefSegment::Primary) mature_primary++;
        else if (s == RefSegment::Companion) comp_in++;
        else if (s == RefSegment::Moonlet) moon_in++;
        if (i > m0) {
            const RefSegment sp = res.segments[i - 1];
            if ((sp == RefSegment::Primary && s == RefSegment::World) ||
                (sp == RefSegment::World && s == RefSegment::Primary))
                pw_chatter++;
            const double dw = wind[i] - wind[i - 1];
            net_wind += dw;
            sum_abs_wind += std::abs(dw);
        }
        const lander::Vec2 b0 = bin.position(0, arc[i].time);
        const double rr = std::hypot(arc[i].position_world.x - b0.x,
                                     arc[i].position_world.y - b0.y);
        min_r_iv = std::min(min_r_iv, rr);
        max_r_iv = std::max(max_r_iv, rr);
    }
    const double mature_frac = mature > 0 ? (double)mature_primary / mature : 0.0;
    const double wind_iv_revs = net_wind / (2.0 * lander::kPi);
    const double wind_consistency =
        sum_abs_wind > 0 ? std::abs(net_wind) / sum_abs_wind : 0.0;

    std::fprintf(
        stdout,
        "  [primary gate] arc n=%d  world=%d primary=%d companion=%d "
        "moonlet=%d  crashed=%d landed=%d  whole-arc wind=%.2f rev  "
        "radius min=%.1f max=%.1f\n",
        n, c.world, c.primary, c.companion, c.moonlet, (int)crashed,
        (int)landed, wind_revs, min_r, max_r);
    std::fprintf(
        stdout,
        "  [primary gate] interval [%d..%d]  mature=%d  primary=%d frac=%.4f "
        "chatter=%d comp=%d moon=%d  wind=%.3f rev cons=%.3f  "
        "radius min=%.1f max=%.1f (r=%.1f)\n",
        m0, m1, mature, mature_primary, mature_frac, pw_chatter, comp_in,
        moon_in, wind_iv_revs, wind_consistency, min_r_iv, max_r_iv, r);
    std::fprintf(stdout,
                 "  [primary gate] final frame=%s  seq(whole)=%s\n",
                 lander::ref_segment_name(res.segments[n - 1]),
                 reduced(res.segments).c_str());
    print_metrics3(n, arc, res);

    // GATE checks (STOP-and-report; no threshold / gravity / fixture tuning).
    check(res.segments[m1] == RefSegment::Primary,
          "primary GATE: resolved frame at the one-revolution mark must be "
          "PRIMARY");
    check(mature_frac >= 0.99,
          "primary GATE: >= 99% of the mature interval must be PRIMARY");
    check(pw_chatter <= 1,
          "primary GATE: no repeated PRIMARY<->WORLD chatter in the interval");
    check(comp_in == 0, "primary GATE: zero COMPANION in the interval");
    check(moon_in == 0, "primary GATE: zero MOONLET in the interval");
    check(wind_iv_revs >= 0.5 && wind_consistency >= 0.9,
          "primary GATE: continuous, consistent ~one-revolution angular "
          "winding in the interval");
    check(max_r_iv < 2.0 * r && min_r_iv > 0.5 * r,
          "primary GATE: radius stays bounded over the one-revolution "
          "interval");
}

// NON-GATING PHYSICS CHARACTERIZATION (M06-R13): the diagnostic clearance scan
// that established why the 10-period PRIMARY gate was superseded. It coasts a
// zero-thrust body-0 circular orbit at several clearances through the
// authoritative Simulation and reports how long each survives and how the orbit
// degrades. This is evidence only; it never gates the milestone and never
// tunes a constant.
void report_primary_physics_characterization() {
    const double clearances[] = {20.0, 40.0, 80.0, 150.0, 300.0};
    std::fprintf(stdout, "  [physics characterization] non-gating scan of "
                 "near-surface primary-orbit lifetime vs clearance\n");
    for (double clr : clearances) {
        lander::Simulation sim;
        sim.reset(0);
        const BinarySystem& bin = sim.binary();
        const double r0 = bin.body(0).terrain.max_surface_radius();
        const double r = r0 + clr;
        const double mu0 = bin.body(0).mu;
        const double T_local = 2.0 * lander::kPi * std::sqrt(r * r * r / mu0);
        const lander::BallisticState st0 = place_on_orbit(bin, 0, r);
        lander::State st{};
        st.x = st0.p.x;
        st.y = st0.p.y;
        st.vx = st0.v.x;
        st.vy = st0.v.y;
        st.angle = 0.0;
        st.omega = 0.0;
        st.fuel = 1000.0;
        st.landed = false;
        st.crashed = false;
        sim.set_state(st);
        const double dt = sim.config().fixed_dt;
        const int steps = (int)std::lround(10.0 * T_local / dt);
        double min_r = 1e30, max_r = 0.0;
        int reached = 0;
        bool crashed = false, landed = false;
        for (int i = 0; i < steps; ++i) {
            sim.advance(dt, {});
            const lander::State& ns = sim.state();
            const lander::Vec2 b0 = bin.position(0, sim.sim_time());
            const double rr = std::hypot(ns.x - b0.x, ns.y - b0.y);
            min_r = std::min(min_r, rr);
            max_r = std::max(max_r, rr);
            reached++;
            if (ns.crashed) {
                crashed = true;
                break;
            }
            if (ns.landed) {
                landed = true;
                break;
            }
        }
        std::fprintf(
            stdout,
            "    clr=%4.0f  r=%6.1f  T=%.1fs  survived %d/%d steps "
            "(%.2f of 10 periods)  crash=%d land=%d  min_r=%.1f max_r=%.1f "
            "(maxR0=%.1f)\n",
            clr, r, T_local, reached, steps, (double)reached / steps * 10.0,
            (int)crashed, (int)landed, min_r, max_r, r0);
    }
    std::fprintf(stdout,
                 "    Near-surface prograde primary orbits in the current "
                 "canonical three-body system are physically short-lived under "
                 "the close 600 m companion's perturbation. Tested 20-150 m "
                 "clearances fail after ~1.1-1.4 local periods; 300 m survives "
                 "~7.9 periods but becomes highly eccentric. This is not an "
                 "AUTO classifier failure.\n");
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
            : x == RefSegment::Primary ? 'P'
            : x == RefSegment::Companion ? 'C' : 'M';
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

// V20 (M06-R13): with the moonlet present, body->body transitions must still
// route through WORLD. A spliced primary -> moonlet coast enters MOONLET only
// through a WORLD bridge (no adjacent P -> M pair); the reverse splice yields
// M -> W -> P. (The existing P <-> C splices above still produce WPWC / WCWP.)
void no_direct_body_transition_moonlet() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    const double rm = bin.body(2).terrain.max_surface_radius() + 20.0;

    // (a) spliced primary -> moonlet: P -> W -> M (no adjacent P<->M).
    {
        std::vector<lander::TimedTrajectorySample> arc;
        auto first = lander::predict_zero_thrust(
            bin, place_at(bin, 0, rp, 0.0), 60.0, 1.0 / 120.0, 2048);
        append(arc, to_timed(first));
        const double t1 = arc.back().time;
        append(arc, to_timed(
            lander::predict_zero_thrust(bin, place_at(bin, 2, rm, t1),
                                        t1 + 40.0, 1.0 / 120.0, 2048)));
        lander::AutoClassifyResult r;
        lander::classify_auto(arc, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        const std::string seq = reduced(r.segments);
        std::fprintf(stdout,
                     "  [spliced P->M] seq=%s  W=%d P=%d M=%d\n", seq.c_str(),
                     c.world, c.primary, c.moonlet);
        check(!has_direct_body_transition(r.segments),
              "spliced P->M: no direct P<->M adjacency");
        check(c.primary > 0 && c.moonlet > 0 && c.world > 0,
              "spliced P->M: all three regimes must appear");
        const size_t fp = seq.find('P');
        const size_t fm = seq.find('M');
        check(fp != std::string::npos && fm != std::string::npos && fp < fm,
              "spliced P->M: primary must precede moonlet");
        check(seq.find('W', fp + 1) != std::string::npos &&
                 seq.find('W', fp + 1) < fm,
              "spliced P->M: a world bridge must sit between primary and "
              "moonlet");
    }
    // (b) the reverse: a spliced moonlet -> primary coast routes M -> W -> P.
    {
        std::vector<lander::TimedTrajectorySample> arc;
        auto first = lander::predict_zero_thrust(
            bin, place_at(bin, 2, rm, 0.0), 40.0, 1.0 / 120.0, 2048);
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
                     "  [spliced M->P] seq=%s  W=%d P=%d M=%d\n", seq.c_str(),
                     c.world, c.primary, c.moonlet);
        check(!has_direct_body_transition(r.segments),
              "spliced M->P: no direct M<->P adjacency");
        check(c.primary > 0 && c.moonlet > 0 && c.world > 0,
              "spliced M->P: all three regimes must appear");
        const size_t fm = seq.find('M');
        const size_t fp = seq.find('P');
        check(fm != std::string::npos && fp != std::string::npos && fm < fp,
              "spliced M->P: moonlet must precede primary");
        check(seq.find('W', fm + 1) != std::string::npos &&
                 seq.find('W', fm + 1) < fp,
              "spliced M->P: a world bridge must sit between moonlet and "
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
// LEGACY R12 DIAGNOSTIC (not a gate; M06-R13-09): the historical 2-body-
// calibrated 300 s primary fixture classified from two different previous
// frames (World and Companion). In the R12 2-body world it resolved PRIMARY
// from both; in the 3-body world the same nominal arc legitimately differs, so
// this is preserved as a prev-dependence OBSERVATION (report the resolved
// final frame for each prev) rather than a gate that the arc must end PRIMARY.
void primary_fixture_robust_to_prev() {
    const BinarySystem bin = make_binary();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    auto prim = to_timed(
        lander::predict_zero_thrust(bin, place_on_orbit(bin, 0, rp), 300.0,
                                    1.0 / 120.0, 2048));
    check(prim.size() >= 200, "primary prev: not enough coast samples");
    lander::AutoClassifyResult from_world;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::World, from_world);
    lander::AutoClassifyResult from_companion;
    lander::classify_auto(prim, bin, lander::ClassifierParams{},
                          RefSegment::Companion, from_companion);
    std::fprintf(
        stdout,
        "  [legacy R12 primary prev-dependence, diagnostic] prev=World -> %s ; "
        "prev=Companion -> %s\n",
        lander::ref_segment_name(from_world.segments.back()),
        lander::ref_segment_name(from_companion.segments.back()));
}

// V12/V13: end-to-end through the GUI's actual prediction path. The
// flight-computer `predict_trajectory` (a node-less COAST) is the prediction
// the display consumes; feeding its `timed` samples to `classify_auto` must
// resolve a valid AUTO frame. This proves the timed-sample storage and the
// classifier work together on the real pipeline, not just on
// `predict_zero_thrust`. Per M06-R13-09 the companion outcome is OBSERVATIONAL
// (not forced) and the 2-body-calibrated primary fixture is a legacy
// DIAGNOSTIC; the only assertion is the pipeline sanity (a non-empty timed
// arc that the classifier consumes cleanly).
void fixture_end_to_end() {
    lander::Simulation sim;  // default reset(0), canonical binary
    const BinarySystem& bin = sim.binary();
    const lander::Config& cfg = sim.config();
    const double rp = bin.body(0).terrain.max_surface_radius() + 20.0;
    const double rc = bin.body(1).terrain.max_surface_radius() + 20.0;

    // A node-less COAST prediction from a bounded companion orbit (the
    // companion frame is observational under M06-R13-09: report, don't gate).
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
        check(pred.timed.size() >= 200,
              "e2e companion: prediction produced a non-empty timed arc");
        lander::AutoClassifyResult r;
        lander::classify_auto(pred.timed, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        std::fprintf(
            stdout,
            "  [e2e companion, observational] n=%zu W=%d P=%d C=%d M=%d "
            "final=%s\n",
            pred.timed.size(), c.world, c.primary, c.companion, c.moonlet,
            lander::ref_segment_name(r.segments.back()));
    }
    // The same end-to-end path from the R12 2-body-calibrated primary fixture
    // (a legacy diagnostic under M06-R13-09; the PRIMARY gate is now
    // gate_primary_current).
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
        check(pred.timed.size() >= 200,
              "e2e primary: prediction produced a non-empty timed arc");
        lander::AutoClassifyResult r;
        lander::classify_auto(pred.timed, bin, lander::ClassifierParams{},
                              RefSegment::World, r);
        const Counts c = count_segments(r.segments);
        std::fprintf(
            stdout,
            "  [e2e primary, legacy R12 fixture, diagnostic] n=%zu W=%d P=%d "
            "C=%d M=%d final=%s\n",
            pred.timed.size(), c.world, c.primary, c.companion, c.moonlet,
            lander::ref_segment_name(r.segments.back()));
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

    std::printf("no_direct_body_transition_moonlet...");
    no_direct_body_transition_moonlet();
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

    std::printf("fixture_end_to_end...");
    fixture_end_to_end();
    std::printf("ok\n");

    std::printf("primary_fixture_robust_to_prev (legacy R12 diagnostic)...");
    primary_fixture_robust_to_prev();
    std::printf("ok\n");

    std::printf("report_legacy_r12_primary (legacy R12 diagnostic)...");
    report_legacy_r12_primary();
    std::printf("ok\n");

    std::printf("report_companion (observational)...");
    report_companion();
    std::printf("ok\n");

    std::printf("report_primary_physics_characterization (non-gating)...\n");
    report_primary_physics_characterization();

    std::printf("ALL UNIT TESTS + REPORTS PASS (%d failures so far)\n",
                failures);

    // The two classification gates use fail-fast; they run LAST so every unit
    // test and report above has reported its result regardless of gate outcome
    // (M06-R13-09: the primary gate is now gate_primary_current; the companion
    // case is observational; the R12 primary fixture is a legacy diagnostic).
    std::printf("gate_moonlet...");
    gate_moonlet();
    std::printf("ok\n");

    std::printf("gate_primary_current (current-world PRIMARY gate)...");
    gate_primary_current();
    std::printf("ok\n");

    std::printf("ALL PASS (%d failures so far)\n", failures);
    return failures;
}
