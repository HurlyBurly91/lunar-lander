# M06 Predictor / Physics Issue Ledger

Status: **DEFERRED** — recorded during the M06-R8 debug-harness human
inspection (2026-10-03). These are **not** defects to be fixed inside M06.
Diagnosis and any fix belong to a **bounded post-M06 predictor/physics
hardening pass** (scoped separately; *not* M07, and *not* an expansion of M06's
accepted scope). The M06 **predictor subsystem is NOT accepted** on this basis.

**M06-R11 Pass 1 (2026-10-03) — diagnostics only.** A first bounded pass of the
hardening effort corrected the *diagnostic readouts* so the debug panels tell
the truth (see the "M06-R11 — Pass 1 outcome" section at the end): the debug
font now renders every diagnostic character, the common readout's altitude is
body-relative and signed, the PE/AP / clearance / contact / horizon / frame
readouts are relabelled with explicit bodies + units, and the debug orbit
fixture uses the selected body's own terrain/mu. **No issue in this file was
fixed by Pass 1**: every PRED / SIM-COLL issue below remains **OPEN**. Two
symptoms were *partially* improved (PRED-01's marker now carries an in-scene
label; PRED-06's predictor readout is relabelled MIN R / MAX R) but their
underlying quantitative verification is still deferred to a later pass.

This is a *record of symptoms + code-grounded observations + candidate causes*.
Where a root cause is not proven it is explicitly marked **UNPROVEN** and must
be resolved by the quantitative test specified for that issue. Do **not** read a
candidate cause here as a confirmed diagnosis. No defect in this file was fixed
during M06.

## Issue index

| ID | One-line summary | Class | Status |
|----|------------------|-------|--------|
| PRED-01 | The yellow/amber square in the scene is the closest-approach-to-destination-pad marker; its meaning is not explained in the legend | rendering / UI | OPEN (identity determined; needs an explicit label + rendered/numeric position-agreement check) |
| SIM-COLL-01 | The authoritative craft visually penetrates the terrain before CRASHED is declared | simulation / contact | OPEN (candidate causes listed; needs a quantitative contact test) |
| PRED-02 | The predicted crash/contact marker and the terminal trajectory point can sit visibly inside the terrain | prediction / contact | OPEN (UNPROVEN: fixed-step localisation vs surface/height mismatch vs state error) |
| PRED-03 | Predicted trajectories look strongly primary-centric during close companion approaches | gravity model | OPEN (visual symptom; code shows a canonical 2-body sum, so the "missing gravity" hypothesis is contradicted — needs quantification) |
| PRED-04 | Projected trajectories can cross the companion's disk without a companion-contact termination | collision model | OPEN (needs an explicit both-body collision test) |
| PRED-05 | No quantitative frozen-COAST truth comparison of the prediction vs the authoritative sim exists | verification | OPEN (test to be written in the hardening pass) |
| PRED-06 | PE/AP reference-body handling around the companion is unverified | prediction / readout | OPEN (needs a reference-selection test) |
| PRED-07 | Closest-approach (CP) prediction/marker accuracy is unverified against a real zero-input coast | prediction / readout | OPEN (needs a frozen-coast comparison) |
| PRED-08 | No deterministic companion-specific predictor scenarios exist for human verification | test fixtures | OPEN (scenarios to be added in the hardening pass) |

## Positive findings preserved (do not re-verify from scratch in the pass)

Confirmed working during the M06 debug inspection; keep as a green baseline:

- [x] Predictor trajectory overlay is visible in `--debug-subsystem predictor` (M06-R9).
- [x] F2 / F3 / F4 switch COAST / LIVE / PLAN.
- [x] Cache / policy / signature instrumentation is visible in the panel.
- [x] SHIFT / rebuild activity is observable.
- [x] Normal gameplay is unchanged by the debug harness.
- [x] Predictor debug mode starts and runs to completion (headless exit 0).
- [x] Primary-centred predicted trajectories render and evolve over time.
- [x] A primary predicted crash/contact can be produced.
- [x] The subsystem-isolation harness itself is functioning (all 12 selectors).

## Manual debug mode

- [~] **MANUAL — provisional pass.** The `manual` debug-harness mode is
  **provisionally PASS** for its *debug-harness functionality* (the pilot is the
  "subsystem"; controls, readout, deterministic fixture, and isolation all work).
  This provisional pass does **NOT** extend to general collision/contact
  behaviour: **SIM-COLL-01 supersedes any assumption that collision/contact is
  accepted.** Manual-flight UI/input isolation can pass while collision handling
  remains an open physics defect.

## Code-grounded observations (inspection only; no behaviour changed)

Recorded so the hardening pass starts from a known baseline. These are
*observations of the current code*, not diagnoses.

### Collision / contact detection (authoritative sim)
- `Simulation::resolve_ground_contact()` (`src/sim.cpp:564`) runs **once per
  fixed step**, after that step's integration.
- It loops **both bodies** (`for i in 0..2`) and, per body, compares the ship's
  **reference point** `(state.x, state.y)` to the body's terrain:
  - `rho = |ship - body_centre|`; if `rho < 1e-9` → crash (fell to centre).
  - `surface = terrain.surface_radius_at_arc(arc)` at the ship's arc; if
    `rho > surface` → not in contact (continue); otherwise contact.
  - On contact: a safe pad landing (radial / tangential / angle all within
    limits) → land; otherwise **crash**.
- On crash the code **snaps the ship back to the terrain surface**
  (`state.x/y = body_centre + surface * (cosθ, sinθ)`) before freezing.
- **Collision uses the ship's single reference point only**, not the rendered
  hull geometry (legs, body), which extends below the reference point
  (`draw_lander`).
- **There is no sub-step (within-fixed-step) contact localisation**: the crash
  is detected at the step boundary and the ship is snapped to the surface at
  that step's (rotating, tidally-locked) arc.

### Predicted contact (receding-horizon predictor)
- `RecedingHorizonPredictor::extend()` (`src/predictor.cpp:118`) clones the
  authoritative sim into a tail and steps it under the current policy; it reads
  `tail_.state().crashed` / `.landed` — i.e. it reuses the **same** contact
  logic above. The recorded `contact_.position` is the tail's state position at
  the crash/land step (the snapped position).

### Gravity model
- `BinarySystem::gravity(p, t)` (`include/lander/binary.hpp:157`) returns the
  **vector sum of both bodies'** inverse-square fields; both are always active,
  **no SOI switching / no nearest-only / no stabilization**.
- Both the live sim and the ballistic / predictor integrators
  (`step_ballistic`, `src/ballistic.cpp:121`) call this same `bin.gravity`, so
  the prediction uses the **same moving two-body gravity model** as the
  authoritative sim. (Canonical reference: `docs/physics-model-gravity.md`,
  "Spacecraft gravity".)

### Prediction readouts and their references
- **CP / closest approach** (`TrajectoryPrediction::closest`,
  `src/flight_computer.cpp:214-219`): minimum distance over the pre-node
  (COAST) branch **to the destination base pad** (`destination_pad`), a moving
  point fixed on the destination body's rotating surface. Fields: `distance`,
  `time`, `position` (ship at closest), `target_position` (the pad).
- **PE / AP** (`src/flight_computer.cpp:277-280`): local extrema of `rho(t)`
  where `rho = |ship - reference_body|` — i.e. **relative to the reference
  body only** (a single-body quantity; the struct comment states these are
  numeric readouts, not invariant orbital elements). The reference body is
  chosen by the nearest-body heuristic `reference_body_for` (`src/sim.cpp:659`),
  which can switch between the two bodies.
- **CP and PE/AP therefore use different reference bodies** (destination pad vs
  reference body).
- **AMBER / YELLOW square** (`src/gui.cpp:1253-1258`): drawn at
  `closest.target_position` (the destination pad's position at the closest-
  approach time), colour `(255,196,64)`, 6×6 px — this is the **PRED-01**
  marker. Other scene markers: red `X` = predicted impact/crash (flight-computer
  `impact` at `src/gui.cpp:1243`, and the live predictor `contact` at
  `src/gui.cpp:1299`), white 4×4 = PE, light-blue 4×4 = AP, purple `+` = node,
  magenta/green `X` = live predicted contact (PRED CRASH / PRED LAND).
- **Scene legend** (`src/gui.cpp:1321`, `draw_prediction_legend`) documents only
  the three arc **colours** (COAST / LIVE / PLAN); it does **not** document the
  CP / PE / AP / impact / node markers — the source of the PRED-01 ambiguity.

---

## PRED-01 — Yellow square marker is ambiguous

**Observed**
- Predictor mode displays a yellow square near / around the secondary body or
  the predicted path; its meaning is not obvious from the legend or the debug
  panel.

**What the rendering code shows (no behaviour change)**
- The yellow/amber square is `TrajectoryPrediction::closest`, drawn at
  `closest.target_position` = the **destination base pad's** position at the
  predicted closest-approach time (`src/gui.cpp:1253-1258`, colour
  `(255,196,64)`, 6×6 px). It is the "CP" (closest approach) marker. It is part
  of `draw_trajectory` (the flight-computer COAST/PLAN prediction), not the
  live cyan predictor. It is valid whenever a zero-thrust ballistic branch is
  computed (initialised at t0 and updated per sample), so it is usually
  present.
- It is **not** listed in `draw_prediction_legend` (which only documents the
  COAST / LIVE / PLAN arc colours), so on-screen it has no caption.

**Requirement for later investigation**
- Give the marker an explicit UI label / legend entry (e.g. "CP" at the
  destination pad).
- Verify the rendered position agrees with the numeric `closest` state
  (`distance`, `time`, `target_position`), including when the destination pad
  is on the secondary body (the "near the secondary body" case).

**Uncertainty / status**: identity is determined from the code above; the
*reason it is confusing* (destination-pad reference, no legend entry) and the
exact pad/body in the observed run are to be confirmed in the pass. **OPEN.**

---

## SIM-COLL-01 — Authoritative spacecraft penetrates terrain before crash resolves

**Observed**
- During real simulation the rendered craft can travel for a visible interval
  with a substantial portion below the terrain surface; CRASHED is declared
  only later. This was visually obvious, not sub-pixel.

**Class**: simulation / contact (not merely a predictor-rendering defect).

**Candidate causes (UNPROVEN — the pass must determine which applies)**
1. **Collision geometry uses only the craft's reference point / centre** — the
   code confirms `resolve_ground_contact` tests only `state.x/y`, not the
   rendered hull (legs / body) which extends below the reference point.
2. **Mismatch between rendered craft dimensions and collision geometry** — the
   rendered model has extent the collision ignores (related to #1).
3. **Fixed-step contact detection / localisation** — contact is checked once
   per fixed step with no sub-step localisation; at high speed the reference
   point can cross (and pass below) the surface between steps before detection.
4. **Terrain / body-frame timing** — the arc / surface used at detection is
   from the *current* step of the rotating, tidally-locked body; the crossing
   that actually happened was against the previous step's surface.
5. Another cause.

**Requirement for later investigation**
- A quantitative contact test: for a controlled fast descent, record the
  per-step reference-point radius vs the per-arc surface, the first step with
  `rho <= surface`, the penetration depth at that step, and the interval from
  first *hull* contact (rendered) to CRASHED. Determine which of (1)-(5)
  accounts for the observed penetration, then scope the fix in the hardening
  pass.

**Uncertainty / status**: cause not established. **OPEN.**

---

## PRED-02 — Predicted crash/contact position can appear underground

**Observed**
- The predictor's red crash/contact marker can sit visibly inside the terrain
  rather than at the surface; in at least one case the predicted trajectory
  extends to a point materially below the visible terrain boundary.

**Code-grounded context**
- The recorded crash position is the tail-sim state position at the crash step,
  which the sim snaps to `surface_radius_at_arc(arc)`. The *preceding*
  trajectory samples are the raw integrated positions (a polyline), with **no
  sub-step clipping at the surface crossing**.
- The collision surface is `terrain.surface_radius_at_arc(arc)`; the rendered
  terrain is drawn from the same terrain model, but the exact surface height
  used for the snap vs the rendered mesh at that arc is not asserted equal here.

**Candidate causes (UNPROVEN)**
1. **Fixed-step contact localisation** — the last trajectory samples are below
   the surface because detection happens at the next step boundary (the
   polyline is not clipped at the true crossing).
2. **Collision-surface vs rendered-surface mismatch** — the snap target
   (`surface_radius_at_arc`) differs from the rendered terrain height at that
   arc, so the marker lands below the visible surface.
3. **Prediction-state error** — the cloned tail diverges from the live sim
   (ephemeris / terrain state), so the crash is localised at a different point.

**Requirement for later investigation**
- Quantitative: at a predicted impact, compare (a) the marker position, (b) the
  last pre-impact trajectory sample, (c) the terrain surface height at that arc,
  and (d) the live sim's own impact point. Decide whether the residual
  below-surface offset is a localisation artefact (#1) or a surface/height
  mismatch (#2 / #3). **Do not** change rendering to conceal the symptom.

**Uncertainty / status**: root cause not established. **OPEN.**

---

## PRED-03 — Other-body gravitational prediction appears incorrect or incomplete

**Observed**
- During close encounters with the companion, predicted trajectories often stay
  strongly primary-centric; repeated visual tests near the companion did not
  convincingly show a transition to a companion-bound / flyby trajectory. The
  effect is clearest during close approaches to the companion.

**Canonical requirement (`docs/physics-model-gravity.md`, "Spacecraft gravity")**
- Both gravitational fields act at all times; no SOI switching; no
  nearest-body-only gravity; the prediction uses the same moving two-body
  gravity model as the authoritative sim.

**Code-grounded context (important)**
- `BinarySystem::gravity` (`include/lander/binary.hpp:157`) *does* sum both
  bodies' inverse-square fields, always active, no SOI; and both the live sim
  and the ballistic / predictor integrators use this same function. So the
  specific hypothesis "the predictor drops the companion's gravity" is
  **contradicted by the code** — recorded here so the pass does not re-chase it.
- The companion's `mu` is `mu0/81` (≈ 2209 m^3/s^2, R1 ≈ 36.9 m), so its field
  is a *weak perturbation* until the ship is well within the companion's Hill
  sphere; a visually "close" approach on screen may still be far enough out
  that the primary legitimately dominates.

**Requirement for later investigation (quantitative, not visual)**
- Run the **frozen-COAST** comparison (PRED-05) and the companion scenarios
  (PRED-08). For a set of companion-approach cases, record the per-body
  acceleration magnitudes `|a_primary|` vs `|a_companion|` along the trajectory
  (the sim already computes these, `src/sim.cpp:163-169`) plus the trajectory
  curvature / which body the ship is bound to at each sample. Establish whether
  the observed primary-centricity is (a) physically correct (companion field
  genuinely weaker at that range), (b) a presentation / frame effect (the arc
  is drawn in the primary reference frame), or (c) a real predictor-vs-live
  divergence at equal state.

**Uncertainty / status**: verified *visual* symptom; root cause **UNPROVEN**.
**OPEN.**

---

## PRED-04 — Predicted collision/contact with the companion appears suspect

**Observed**
- During close encounters, projected trajectories can visually pass through /
  across the companion's rendered disk without an obvious companion-contact
  termination; the primary terrain-collision prediction is visibly exercised,
  but the equivalent companion-collision behaviour is not demonstrated.

**Code-grounded context**
- The authoritative contact check (and thus the predictor's cloned contact)
  **does** iterate both bodies (`resolve_ground_contact`, `src/sim.cpp:569`). A
  companion contact is therefore *in the model* — it triggers when the ship's
  reference point satisfies `rho <= terrain.surface_radius_at_arc(arc)` for the
  companion.

**Candidate causes (UNPROVEN)**
1. **Rendered-disk vs collision-surface mismatch** — the companion's on-screen
   disk radius may exceed the terrain `surface_radius_at_arc` at the arc the
   ship crosses, so a visually-"through the disk" pass is actually above the
   collision surface (grazing a terrain valley / low-relief arc).
2. **Reference-point-only geometry** (see SIM-COLL-01) — the hull may visually
   overlap the disk while the reference point is still outside `surface`.
3. **Per-step sampling** — a graze shorter than one fixed step is missed.

**Requirement for later investigation**
- An explicit **both-body** collision test: construct cases that (a) impact the
  companion, (b) graze it just above the surface, (c) pass over a terrain valley
  of the companion; verify the predictor / authoritative contact fires for the
  impact case and *does not* for the non-contact cases, and compare the
  rendered disk radius to `surface_radius_at_arc` for the companion. **OPEN.**

---

## PRED-05 — No quantitative frozen-COAST truth comparison exists

**Observed**
- The predictor panel exposes useful cache / policy / debug state, but there is
  no numerical verification that a prediction made at `t0` matches the
  subsequent authoritative simulation.

**Required later test (to be written in the hardening pass — NOT now)**
- At `t0`: snapshot the authoritative state; **freeze the COAST prediction**
  (zero main thrust, zero active RW / control command, no node execution); do
  **not** replace or re-refresh the reference prediction.
- Compare the authoritative state against **fixed-step-aligned samples** of the
  frozen prediction at several future times. Record at minimum:
  - relative sample time
  - predicted position / actual position / **position error (m)**
  - predicted velocity / actual velocity / **velocity error (m/s)**
  - **max** and **RMS** position error
  - **max** and **RMS** velocity error
  - a **justified PASS / FAIL tolerance**
- The reference prediction MUST remain frozen (do **not** compare against a
  continuously-refreshed rolling trajectory).

**Uncertainty / status**: the test does not exist yet. **OPEN** (deferred).

---

## PRED-06 — PE/AP reference handling around the companion is unverified

**Observed**
- PE / AP readouts exist and look plausible around the primary; their
  reference-body selection and meaning in a companion-bound orbit have not been
  established.

**Code-grounded context**
- PE / AP are local extrema of `rho(t) = |ship - reference_body|`
  (`src/flight_computer.cpp:277-280`) — i.e. **relative to the reference body
  only** (a single-body quantity; the struct comment states these are numeric
  readouts, not invariant elements). The reference body is chosen by the
  nearest-body heuristic `reference_body_for` (`src/sim.cpp:659`), which can
  switch.

**Requirement for later investigation**
- Determine (a) which body PE / AP is referenced to in a companion-bound orbit,
  (b) whether / when that reference switches and how the readout behaves across
  the switch, and (c) whether the values remain meaningful when the ship is
  bound to the companion rather than the reference body. **OPEN** (deferred).

---

## PRED-07 — Closest-approach (CP) prediction/marker is unverified

**Observed**
- Closest-approach graphics / readouts exist, but their exact semantics and
  accuracy have not been compared against a real zero-input coast.

**Code-grounded context**
- CP = the minimum, over the pre-node (COAST) branch, of the ship's distance to
  the **destination base pad** (a moving point on the destination body's
  surface), recording `time`, `position` (ship) and `target_position` (pad).

**Requirement for later investigation**
- Freeze COAST (per PRED-05); record the predicted closest-approach time,
  distance, and target state / position; let the authoritative sim coast with
  unchanged controls; measure the **actual** minimum separation / time; compare
  quantitatively (and confirm the on-screen marker agrees, closing PRED-01's
  position-agreement item). **OPEN** (deferred).

---

## PRED-08 — Companion-specific predictor scenarios are missing from human verification

**Observed**
- There are no deterministic scenarios for the companion cases, so the human
  pass cannot separate gravity / collision / rendering errors.

**Required later scenarios (to be added in the hardening pass — NOT now)**
- close companion flyby
- bound companion orbit
- direct companion impact
- primary impact control case (the known-good baseline for comparison)

Each should be a **deterministic, startup-only** fixture (per the harness
rules) and should be used to isolate gravity-model errors (PRED-03) from
collision-model errors (PRED-04, SIM-COLL-01) from rendering / marker errors
(PRED-01, PRED-02, PRED-07).

**Uncertainty / status**: the fixtures do not exist yet. **OPEN** (deferred).

---

## Scope guard for the hardening pass

When this is picked up:
- It is a **bounded** predictor / physics hardening pass — **not** M07, and not
  an expansion of M06's accepted scope.
- Any fix must respect the canonical gravity model
  (`docs/physics-model-gravity.md`) and the no-weaken-tests rule.
- The frozen-COAST validator (PRED-05) and the companion scenarios (PRED-08)
  are the entry points; the remaining issues (PRED-01 / 02 / 03 / 04 / 06 / 07,
  SIM-COLL-01) should be resolved *using* those, not by ad-hoc visual checks.

---

## M06-R11 — Pass 1 outcome (diagnostics-only; all issues remain OPEN)

Pass 1 made the debug **readouts** trustworthy. It deliberately changed **no**
predictor / controller physics, so **none** of the issues above is closed.

### Confirmed and corrected in Pass 1 (readouts / debug fixture only)

- **Debug font actually renders.** `src/gui.cpp` `glyph()` (now delegating to
  the headless helpers in `include/lander/debug_font.hpp`,
  `debug_font::normalize` / `debug_font::visible`) maps lowercase ASCII to
  uppercase and draws `[` / `]`; the supported set is space, `A-Z`, `0-9`,
  `- + . : / = % ! ( ) , [ ]`. Diagnostic labels that were silently blanking
  (lowercase, brackets) now render.
- **Body-relative, signed altitude.** `make_common_readout`
  (`src/debug_subsystem.cpp`) computes the common `R-ALT` from the **selected
  reference body's** terrain (`bin.body(ref).terrain`) and its tidal rotation
  (`bin.body_rotation`) via `altitude_at`, no longer the primary terrain +
  `local_up_angle`. The value is **signed** (negative below the surface); zero
  means invalid, not a clamped altitude.
- **Reference identity made explicit.** The common readout now shows
  `REF <PRIMARY|COMPANION>(#n)` and `TGT <...>(#n)` so the altitude / V-REL are
  never attributed to an unnamed body. The production gravitational-reference
  selection (`reference_body_for`) was **not** altered; where it keeps the
  reference on the primary for a small-body near orbit, the readout says so
  rather than implying the orbit body.
- **Meaning / units relabelled:** manual `SURF VR/VT` → body-centre-relative
  (not surface-relative); common `V-REL` → target **body-centre** relative (not
  pad-relative); predictor `PE/AP` (min/max body-centre radius) → `MIN R` /
  `MAX R` (this is *not* the flight-computer / node-edit apsis, which is a
  genuine local-extremum and stays `PE/AP`); clearance → `CLR-PT`
  reference-**point** clearance computed with `altitude_at` per sample for both
  bodies; contact `T+` → `ETA` (`contact.time − sim_time`) with the absolute
  `t=` shown separately; horizon → steps **+ seconds** and the available
  forecast duration when partly filled; rolling-predictor readouts are visually
  distinct from long COAST/PLAN; every world-frame position/velocity/distance is
  labelled `W` (world/inertial); unavailable values show an explicit state.
- **Closest-approach marker labelled.** The amber CP square in the scene
  (`draw_trajectory`, `src/gui.cpp`) is now labelled with its **destination
  body** and ETA (`CP <body> T<+eta>s`). This addresses PRED-01's *label* part
  only; the quantitative rendered-vs-numeric position-agreement check and the
  destination-pad reference explanation remain OPEN in PRED-01 / PRED-07.
- **Debug orbit fixture uses the selected body.** `place_in_orbit(body)`
  (`src/debug_subsystem.cpp`) now uses the selected body's
  `terrain`/`max_surface_radius`, `mu`, `position(body,0)` and
  `velocity(body,0)` (previously the primary terrain + `cfg.mu`). Direction
  convention and the +20 m clearance are preserved.

### Debug autoland scenario vs the passing V14 fixtures (delta, left unchanged)

- The live debug autoland scenario (autoland-companion / primary / cross) is a
  **near-surface (+20 m above the selected body's terrain), live-stepped,
  full-`LandingAutopilot`-armed** fixture that runs through the normal
  simulation / control loop.
- The passing V14 test fixtures
  (`tests/test_landing_zem_zev.cpp`, `make_flying_sim`) are **high (~+900 m,
  primary-only), analytic, never-stepped** orbits with a *constant* gravity
  field used to validate the terminal-velocity guidance *algorithm* in
  isolation, not a live landing campaign.
- The two therefore differ in altitude, body count, stepping, and arm state;
  the debug scenario is a developer **initializer** (also the B×3 /
  `sync_orbit` developer command), **not** a validated stationary
  companion-orbit fixture, and was **left unchanged** by Pass 1.

### What must remain OPEN after Pass 1

- **PRED-01** — marker now labelled, but the rendered-position-vs-numeric and
  destination-pad-reference verification is still owed. **OPEN.**
- **SIM-COLL-01** — no contact-detection / sub-step-localisation change was
  made (out of Pass 1 scope). **OPEN.**
- **PRED-02** — predicted-contact below-surface localisation / surface-mismatch
  unchanged. **OPEN.**
- **PRED-03** — gravity-model primary-centricity unchanged (the code already
  sums both fields; quantification still owed). **OPEN.**
- **PRED-04** — both-body companion-collision behaviour unchanged. **OPEN.**
- **PRED-05** — frozen-COAST quantitative truth comparison does not exist yet.
  **OPEN.**
- **PRED-06** — predictor `MIN R` / `MAX R` now labelled with the reference
  body, but reference-selection / switch behaviour in a companion-bound orbit
  is still unverified. **OPEN.**
- **PRED-07** — CP marker now shows body + ETA, but a frozen-coast CP accuracy
  comparison is still owed. **OPEN.**
- **PRED-08** — no deterministic companion-specific scenarios added yet.
  **OPEN.**

The frozen-COAST validator (PRED-05) and the companion scenarios (PRED-08)
remain the entry points for Pass 2 and beyond.
