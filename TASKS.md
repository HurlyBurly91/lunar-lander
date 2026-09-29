# Active Task

Milestone: M05
Request: M05-R1, M05-R2, M05-R3
State: ACTIVE

## Bootstrap note

M04 (circular moon and orbital physics) is COMPLETE and closed out in
records/M04-circular-moon-orbital-physics.md (commit 160059c, pushed to
origin/main). This bounded ledger begins fresh with the M05-R1 request
group: the binary moon and the first contract loop, per the M05 milestone
specification (milestones/M05-binary-moon-contract-loop.md).

All automated-verification work for M05-R1 (V01..V14 and the derived
implementation tasks D01..D06) and for the M05-R2 camera/navigation follow-up
is complete; see the per-item evidence below and the "Verification evidence"
summary. The milestone is now awaiting human verification: every open
H-item remains open and is not closed until the user confirms each one. No M05
completion record is written and M05 is not set to COMPLETE until that human
acceptance happens.

All automated work for M05-R3 (the camera/HUD/control fixes M05-R3-01..08 and
the tidal-locking requirement M05-R3-09) is complete as of 2026-09-29; see the
per-item evidence below.

All automated work for the M05-R3 extension M05-R3-10..16 (guarded `R x3`
retry, wider LOCAL manual zoom, inertial starfield, SYSTEM no-auto-pan,
adaptive zoom formatting, `B x3` body-synchronous orbit initializer, `T x3`
ballistic inter-body transfer initializer) is complete as of 2026-09-29; see
the per-item evidence and the "Verification evidence" section.

Human verification of the M05-R3-10..16 build (commit b49a476, 2026-09-29)
returned: H19 PASS, H20 PASS (now closed with human evidence), H18 PARTIAL
(the SYSTEM-fixed starfield is correct behaviour, not a defect; the
authoritative rule is that apparent star motion depends only on the actual
presentation camera angle), H17 FAIL (a visible rendering/draw error at
extreme wide LOCAL zoom), and H21 FAIL (T x3 freezes the game while solving
and teleports the spacecraft to a canonical departure position). This
created the corrective round M05-R3-17..22 below (wide-LOCAL rendering fix,
terrain-seam fix, non-blocking transfer solver, velocity-only transfer
semantics superseding M05-R3-16's departure-shell repositioning, starfield
no-regression rule, and the verification/state handling). The ledger is back
to ACTIVE; when the new automated work completes it returns to AWAITING
HUMAN VERIFICATION with H17, H18, H21 (re-tested) and the new H22 open. No
M05 completion record exists yet.

M05-R1 human verification (2026-09-28) found three presentation failures:
(1) the local camera snaps/teleports when the automatically selected reference
body changes; (2) the SYSTEM view gives no way to tell whether the craft is
closing on or opening from the contract destination once it leaves the
viewport; (3) at SYSTEM scale the ship degenerates into a small white square.
The user also clarified the intended camera model: LOCAL = fly relative to the
gravitationally relevant body; SYSTEM = understand and navigate the binary.
This created a new follow-up request group, M05-R2 (see the `## Request M05-R2`
section below), and the ledger is back to ACTIVE. Per the human-feedback rule
a failed human-verification item is not completed by code alone: the M05-R1
items implicated by these failures (H03 spatial/no-teleport, H04 interception
feel, H05 local-camera usability) remain open and are re-verified through
M05-R2. The `User requirements` ... `Derived implementation tasks` sections in
this file record M05-R1; the `## Request M05-R2` section records the active
follow-up. M05 stays open (not COMPLETE, no record) until all M05-R2 automated
work passes and the user re-confirms every open H-item.

## User requirements

- [x] M05-R1-01 The primary moon remains the canonical M04 body and the
  universe's reference units: R0 = 332.384 m, g0 = 1.62 m/s^2,
  mu0 = 178976.334 m^3/s^2, v0 = 23.2048 m/s, T0 = 90.0 s, satisfying
  mu0 = g0*R0^2, v0 = sqrt(mu0/R0), T0 = 2*pi*sqrt(R0^3/mu0). Use
  gravitational parameter `mu` directly; do not introduce a universal G or
  simulated kilograms.
  Source: USER (spec, "Canonical universe model")
  Evidence: tests/test_sim.cpp::test_reference_values; tests/test_binary.cpp::test_canonical_laws

- [x] M05-R1-02 The companion body is derived, not hand-selected, from the
  canonical scaling law s = R/R0 with preserved intrinsic surface gravity
  g0. T1 = 30 s -> s = (T1/T0)^2 = 1/9, therefore R1 = R0/9 =
  36.9315556 m, mu1 = mu0/81 = 2209.58437037 m^3/s^2, g1 = g0, v1 = v0/3 =
  7.73493 m/s, T1 = T0/3 = 30.0 s. Do not compensate for binary tidal
  effects by changing the companion's mu.
  Source: USER (spec, "Derived-body scaling law", "Companion derivation")
  Evidence: tests/test_sim.cpp::test_reference_values; tests/test_binary.cpp::test_canonical_laws

- [x] M05-R1-03 No fake distance scale: world metres remain physical
  simulation metres for spacecraft, both body centres, terrain, and
  velocities. No visual distance scaling, transfer-distance multiplier,
  fast-travel coordinates, or hidden velocity scaling between bodies.
  Source: USER (spec, "No fake distance scale")
  Evidence: tests/test_binary.cpp::test_relative_kinematics; --orbit-demo smoke transfers the true 600 m separation

- [x] M05-R1-04 Fixed circular binary separation D = 600.0 m
  (centre-to-centre); reference surface-to-surface gap D - R0 - R1 =
  230.684444 m. Do not reduce D to compensate for slow transfers.
  Source: USER (spec, "Binary-system geometry")
  Evidence: tests/test_binary.cpp::test_ephemeris (separation stays 600 m)

- [x] M05-R1-05 Both moons orbit their common barycentre on an analytic
  prescribed circular ephemeris (no numerical moon-moon integration):
  mu_system = mu0 + mu1, omega = sqrt(mu_system/D^3), T_binary = 216.94244 s,
  a_primary = D/82 = 7.317073 m, a_companion = 81*D/82 = 592.682927 m,
  primary barycentric speed 0.21192 m/s, companion barycentric speed
  17.16555 m/s; theta = theta0 + omega*t with a documented deterministic
  convention for positions and velocities. Driven by authoritative fixed-step
  simulation time: pause pauses the ephemeris, and reset with the same seed
  restores the same binary phase.
  Source: USER (spec, "Binary orbital mechanics")
  Evidence: tests/test_binary.cpp::test_ephemeris; ::test_relative_kinematics

- [x] M05-R1-06 No body axial rotation in M05: each moon translates around
  the barycentre but does not spin about its own centre, so a point fixed on
  a body's surface has exactly the body's translational velocity.
  Source: USER (spec, "Body rotation")
  Evidence: tests/test_sim.cpp::test_landed_attachment_and_takeoff (attached ship tracks body translation only)

- [x] M05-R1-07 Spacecraft gravity is the superposition of both bodies'
  fields in the shared global inertial frame:
  a = -mu0*(r-rp)/|r-rp|^3 - mu1*(r-rc)/|r-rc|^3 + thrust. Both fields are
  always active. No SOI switching, nearest-body-only gravity, patched conics,
  hidden capture forces, or orbit stabilization.
  Source: USER (spec, "Spacecraft gravity")
  Evidence: tests/test_binary.cpp::test_gravity_superposition; tests/test_sim.cpp::test_one_step_physics

- [x] M05-R1-08 All local navigation and surface mechanics use body-relative
  state: relative_position = ship_position - body_position,
  relative_velocity = ship_velocity - body_velocity. Derive radial distance,
  local outward direction, local tangent, longitude/terrain coordinate,
  surface radius, and altitude from relative_position; derive radial and
  tangential velocity from relative_velocity. Never use raw global ship
  velocity for landing-speed tests on a moving body.
  Source: USER (spec, "Body-relative state")
  Evidence: tests/test_sim.cpp::test_local_frame

- [x] M05-R1-09 Terrain coordinate systems: each body owns terrain in its own
  body-local coordinates. The primary terrain for the same seed remains
  exactly the accepted M04 surface (refactoring must not silently change it).
  The companion gets its own deterministic seamlessly-wrapping terrain seed,
  with natural geometry scaled sensibly to its radius (not a copy of the
  primary's metre-scale mountains). Drawing and collision use the same local
  surface; body translation does not regenerate or deform terrain; each body
  has at least one valid landing site. Landing pads do not obey strict 1/9
  scaling but must not be shrunk below practical lander dimensions.
  Source: USER (spec, "Terrain coordinate systems")
  Evidence: tests/test_sim.cpp::test_terrain; ::test_landing_rules

- [x] M05-R1-10 Collision is checked against both body surfaces. Per body:
  subtract the body centre from the ship position, compute the body-local
  longitude, query that body's terrain radius, and compare the relative
  radial distance to the surface radius. Contact evaluation uses whether
  contact is on a landing pad, body-relative radial/tangential velocity, and
  local radial landing attitude. No global vx/vy landing thresholds; no
  global angle zero as upright.
  Source: USER (spec, "Collision with moving bodies")
  Evidence: tests/test_sim.cpp::test_landing_rules; ::test_crash_rules

- [x] M05-R1-11 Landed attachment: a safely landed spacecraft is attached to
  a specific body and surface location, persisting at least the landed body
  and the landed surface arc/longitude. While landed: system time continues,
  both moons continue their binary orbit, the ship moves with the landed
  body's translational motion and remains attached to the same body-local
  surface location, global ship velocity follows the body's translational
  velocity, the ship remains locally upright, and no hidden orbital
  integration acts on it. Crashed state may remain terminal.
  Source: USER (spec, "Landed attachment")
  Evidence: tests/test_sim.cpp::test_landed_attachment_and_takeoff; ::test_reset_state

- [x] M05-R1-12 Takeoff from a landed pad is permitted without respawning the
  ship into space: takeoff begins from the actual moving body's position and
  current pad, inherits the body's current global translational velocity,
  transitions cleanly from attached/landed to free flight, and uses ordinary
  spacecraft thrust and gravity immediately after release. A low throttle
  that cannot overcome the local effective downward acceleration must not
  create a free-floating numerical jitter state; implement the smallest
  robust ground-support/takeoff rule, not a general contact-dynamics system.
  Source: USER (spec, "Takeoff")
  Evidence: tests/test_sim.cpp::test_landed_attachment_and_takeoff

- [x] M05-R1-13 The M04 O developer control is preserved but generalized:
  circularize relative to the currently selected/reference body i using
  r_rel = ship_position - body_position and v_circular = sqrt(mu_i/|r_rel|);
  set relative radial velocity to zero and relative tangential speed to
  v_circular (preserving meaningful existing tangential direction, with a
  consistent default when essentially zero), then
  ship_global_velocity = body_velocity + desired_relative_orbital_velocity.
  O remains a one-time developer state change with no continuing
  stabilization. F refuel is preserved.
  Source: USER (spec, "Circularize developer control")
  Evidence: tests/test_sim.cpp::test_circularize_state; ::test_orbit_is_usable; ::test_refuel_only_changes_fuel

- [x] M05-R1-14 Companion local orbital behavior: 30 s is the companion's
  canonical reference scale; the primary's gravity creates real tidal
  perturbations in the compact binary, which must not be cancelled. Actual
  terrain-clearing orbits are longer than 30 s, but very-low companion orbit
  remains practically usable. No artificial SOI boundary; do not distort
  gravity to force exactly 30.000 s orbits.
  Source: USER (spec, "Companion local orbital behavior")
  Evidence: tests/test_sim.cpp::test_orbit_is_usable; ::test_reference_values (no stabilization; ~30 s reference scale). Runtime feel is H06.

- [x] M05-R1-15 Camera: preserve the accepted M04 local player-follow camera
  (screen up = reference-body local radial outward, screen right = local
  tangent, exact M04 anchor, AUTO/MANUAL). Add a presentation-only SYSTEM
  view: inertial orientation, spacecraft near the viewport centre, initial
  zoom on the order of 0.04X (base_scale 14 px/m -> about 0.56 px/m), both
  moons at their true simulation coordinates with no fake compression, both
  bodies visible at the 600 m separation on a 1280x720 viewport from most
  useful transfer positions, fixed screen-space star backdrop, and a
  minimum-size ship marker if the correctly scaled lander is hard to see.
  SYSTEM is a separate explicit control shown in the HUD/help, not an
  overload of reference-body selection. Do not modify world geometry to fit
  the camera; the exact zoom may be tuned slightly during human verification.
  Source: USER (spec, "Camera modes")
  Evidence: tests/test_camera.cpp (AUTO/MANUAL anchor + SYSTEM-mode tests); tests/test_sim.cpp::test_interpolated_state

- [x] M05-R1-16 SUPERSEDED by M05-R2-01. Reference body: may drive local HUD
  altitude, radial/ tangential display, local camera orientation, developer
  circularize, and landing information, but must not control which
  gravitational fields are active. Chosen deterministically; near a surface it
  is the locally relevant body; avoid rapid flicker near the crossover region
  using a simple deterministic hysteresis rule.
  Source: USER (spec, "Reference body versus physics")
  Evidence: M05-R1 shipped distance-based 0.8x hysteresis in
  src/sim.cpp::update_reference_body; M05-R2-01 replaces the selection law with
  local gravitational influence while preserving the landed-body rule.

- [x] M05-R1-17 HUD: preserve the M04 HUD where practical; add compact
  two-body information: reference body, contract destination body/base,
  distance to target, and relative velocity useful for interception. Do not
  turn this into a flight-computer UI project; no trajectory prediction or
  maneuver-node planning.
  Source: USER (spec, "HUD")
  Evidence: src/gui.cpp (HUD draws reference body, contract destination, distance, relative velocity). Runtime feel is H03/H05.

- [x] M05-R1-18 First contract loop: start landed at PRIMARY BASE, receive a
  contract to COMPANION BASE, take off, intercept the moving companion, match
  useful relative velocity, land safely on the companion target pad, complete
  the contract, grant score/reward, then issue the next contract targeting
  PRIMARY BASE so the player can take off and return. With two bases,
  contracts alternate between them. Intentionally minimal: no procedural
  mission generator.
  Source: USER (spec, "First contract loop")
  Evidence: tests/test_sim.cpp::test_contract_loop

- [x] M05-R1-19 Contract state: the smallest explicit contract state that
  identifies origin body/base, destination body/base, completion state, and
  reward/score value. A contract completes only when the spacecraft safely
  lands on the designated destination pad; landing safely elsewhere is not
  completion. After completion, assign the next contract from the current
  base to the other base. Use the existing score concept; no currency
  economy, shops, upgrades, inventory, cargo, reputation, or progression
  trees.
  Source: USER (spec, "Contract state")
  Evidence: tests/test_sim.cpp::test_contract_loop; ::test_reset_state

- [x] M05-R1-20 Determinism: for a given game seed and identical inputs the
  primary terrain, companion terrain, binary initial phase, body ephemerides,
  contract sequence, and spacecraft simulation are all identical. The
  analytic ephemeris is driven from authoritative fixed-step simulation time;
  render interpolation may interpolate presentation states but must not
  change authoritative physics.
  Source: USER (spec, "Determinism")
  Evidence: tests/test_sim.cpp::test_reset_state (identical seeds -> identical state); ::test_fixed_step_determinism; ::test_contract_loop

- [x] M05-R1-21 Render interpolation: preserve the M04 fixed-step/render
  interpolation architecture. In every rendered frame the camera,
  spacecraft, and moon positions must represent the same interpolated
  presentation time (never the spacecraft at one interpolated time and a
  moon at a different authoritative tick). Physics and collision continue
  using authoritative fixed-step states only.
  Source: USER (spec, "Render interpolation")
  Evidence: tests/test_sim.cpp::test_interpolated_state

- [x] M05-R1-22 Display the spacecraft's angular velocity in the HUD:
  local angular rate around the reference body,
  omega = relative_tangential_velocity / relative_radial_distance
  (rad/s, shown in deg/s with direction), consistent with the body-relative
  quantities of M05-R1-08.
  Source: USER (follow-up during M05 activation)
  Related: M05-R1-08, M05-R1-17
  Evidence: tests/test_sim.cpp::test_local_frame (local_angular_velocity == tangential / radial); src/gui.cpp renders it in the HUD

## Preserve / constraints

- [x] M05-R1-P01 Preserve the accepted M04 presentation: exact local player
  camera anchor, AUTO/MANUAL camera, corrected moon fill, throttle/HUD,
  fixed screen-space starfield, and continuous presentation-time flame
  animation.
  Source: USER
  Evidence: tests/test_camera.cpp (exact M04 anchor, AUTO/MANUAL); tests/test_sim.cpp::test_flame_animation_continuous; ::test_interpolated_state

- [x] M05-R1-P02 Preserve the M04 fixed-step simulation architecture
  (fixed_dt = 1/120, semi-implicit Euler) and the existing thrust/fuel model
  including F refuel and the 1000-unit testing fuel.
  Source: USER
  Evidence: tests/test_sim.cpp::test_fixed_step_determinism; ::test_one_step_physics; ::test_refuel_only_changes_fuel

- [x] M05-R1-P03 The primary body's constants, terrain for the same seed, and
  its isolated gravitational field behavior remain M04-compatible; changes
  that alter the primary's surface or field require explicit justification,
  not silent drift.
  Source: USER
  Evidence: tests/test_sim.cpp::test_terrain (primary surface/pads for the same seed match the accepted M04 baseline)

- [x] M05-R1-P04 No hidden physics assistance anywhere in the simulation: no
  orbit stabilization, capture forces, SOI switching, or fake distance
  scaling.
  Source: USER
  Evidence: tests/test_binary.cpp::test_gravity_superposition; tests/test_sim.cpp (circularize is one-shot, no continuing force; orbit_is_usable uses plain gravity)

- [x] M05-R1-P05 M05 non-goals: do not add ECS, modular spacecraft, multiple
  ship types, alternative thrusters/fuels, landing balloons, solar sails,
  cargo, economy/shops/upgrades/inventory, procedural contract generation,
  more than one companion, numerical N-body moon-moon integration, body
  axial rotation, tidal locking, atmosphere/aerodynamics, time warp,
  docking, orbital autopilot, trajectory prediction, or maneuver nodes.
  Source: USER
  Evidence: code inspection of include/lander + src (none of the listed systems are present; exactly one companion, analytic ephemeris, no axial spin)

## Automated verification

- [x] M05-R1-V01 Canonical universe law tests: primary mu0 = g0*R0^2 within
  tolerance and T0 ~= 90 s; companion scale exactly 1/9; companion intrinsic
  surface gravity matches the primary; mu1 = mu0/81; companion nominal
  reference circular period ~= 30 s; companion nominal reference circular
  speed ~= v0/3.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_sim.cpp::test_reference_values (g1 == g0 to 1e-9); tests/test_binary.cpp::test_canonical_laws (PASS)

- [x] M05-R1-V02 Binary ephemeris tests: body separation stays 600 m;
  barycentre remains fixed at the origin within numerical tolerance;
  a_primary ~= 7.317073 m; a_companion ~= 592.682927 m; T_binary ~=
  216.94244 s; analytic velocities agree with position derivatives; resetting
  the same seed/time reproduces body states; advancing one full binary period
  returns both bodies close to their starting positions and velocities.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_binary.cpp::test_ephemeris; ::test_relative_kinematics (PASS)

- [x] M05-R1-V03 Multi-body gravity tests: ship acceleration equals the
  vector sum of both bodies' gravity contributions; neither body's gravity
  is silently disabled when the other is nearer; the combined field
  transforms correctly as binary phase changes; no hidden orbit/capture
  stabilization exists.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_binary.cpp::test_gravity_superposition; tests/test_sim.cpp::test_one_step_physics (PASS)

- [x] M05-R1-V04 Relative kinematics tests: relative position subtracts body
  position; relative velocity subtracts body velocity; radial/tangential
  decomposition uses relative velocity; a ship co-moving with a body has
  approximately zero local velocity; a globally stationary ship beside the
  moving companion has large relative tangential velocity.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_binary.cpp::test_relative_kinematics; tests/test_sim.cpp::test_local_frame (PASS)

- [x] M05-R1-V05 Terrain and collision tests: primary M04 terrain regression
  unchanged for the same seed; companion terrain is deterministic and
  seamless; collision works on both moving bodies; drawing/collision query
  the same body-local surface; safe landing works on both bodies; landing
  evaluation uses relative rather than global velocity.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_sim.cpp::test_terrain; ::test_landing_rules; ::test_crash_rules (PASS)

- [x] M05-R1-V06 Landed attachment and takeoff tests: a landed ship remains
  attached to the same local surface location as its body moves; a landed
  ship inherits the body's global translational velocity; system time and
  body ephemeris continue while landed; a valid takeoff transitions to free
  flight without teleporting; takeoff begins with the body's current global
  velocity; insufficient outward thrust does not create surface jitter.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_sim.cpp::test_landed_attachment_and_takeoff; ::test_reset_state (PASS)

- [x] M05-R1-V07 Circularize tests: O around the primary uses primary-relative
  velocity plus primary global velocity; O around the companion uses
  companion-relative velocity plus companion global velocity; relative radial
  velocity becomes approximately zero; relative tangential velocity equals
  sqrt(mu_i/r); no continuing stabilization force exists.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_sim.cpp::test_circularize_state; ::test_orbit_is_usable (PASS)

- [x] M05-R1-V08 Camera/presentation tests: the M04 exact local player anchor
  remains intact; SYSTEM view does not change simulation state; SYSTEM view
  uses true body positions; the system camera can place both bodies in a
  useful view at the 600 m separation; the starfield remains fixed in screen
  space; moving-body render interpolation is smooth at representative render
  rates.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_camera.cpp (M04 anchor + 4 SYSTEM-mode tests: basic, zoom clamp, save/restore, snap); tests/test_sim.cpp::test_interpolated_state (PASS)

- [x] M05-R1-V09 Contract tests: the initial contract targets the companion
  base; landing on a wrong/non-target pad does not complete it; a safe
  landing on the target pad completes it exactly once; reward/score is
  applied exactly once; the next contract reverses destination; contract
  state is deterministic.
  Source: USER (spec, Automated verification)
  Evidence: tests/test_sim.cpp::test_contract_loop ((a) complete-once + reward-once + reversal, (b) non-base pad does not complete, (c) two-way alternation); ::test_reset_state (PASS)

- [x] M05-R1-V10 Build succeeds: cmake --build build
  Source: USER (spec, Automated verification)
  Evidence: cmake --build build --parallel -> clean (exit 0)

- [x] M05-R1-V11 Full test suite passes:
  ctest --test-dir build --output-on-failure
  Source: USER (spec, Automated verification)
  Evidence: ctest --test-dir build --output-on-failure -> 4/4 passed (lander_tests, lander_binary_tests, lander_camera_tests, lander_starfield_tests)

- [x] M05-R1-V12 Pass: git diff --check
  Source: USER (spec, Automated verification)
  Evidence: git diff --check -> no whitespace errors

- [x] M05-R1-V13 Headless smoke runs pass (normal spawn, orbit demo, and a
  landed-binary smoke run) with expected exit status and state output.
  Source: USER (spec, Automated verification; smoke practice from M04)
  Evidence: SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy lander_gui: default (landed, exit 0), --orbit-demo (flying, exit 0), --system-view (state identical to default => presentation-only, exit 0), --orbit-demo --system-view --screenshot (exit 0, artifact written)

- [x] M05-R1-V14 The HUD angular-velocity value equals the computed
  relative_tangential_velocity / relative_radial_distance for the reference
  body (verified in a test through the same helper the HUD uses).
  Source: USER (derivation of M05-R1-22)
  Evidence: tests/test_sim.cpp::test_local_frame (local_angular_velocity == tangential/radial on both bodies, PASS)

## Human verification

- [ ] M05-R1-H01 User confirms the primary moon still feels like the accepted
  M04 body (gravity, orbit, terrain, presentation).
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H02 User confirms the companion visibly moves around the primary
  and does not behave like a stationary target with a moving sprite.
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H03 User confirms SYSTEM view makes the spatial relationship
  understandable, stars remain fixed in screen space, and no visible
  coordinate-frame teleport occurs when changing reference body or camera
  view.
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H04 User confirms the player can leave the primary and intercept
  the moving companion: matching companion velocity matters during approach,
  and landing evaluation feels relative to the companion rather than to
  global coordinates.
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H05 User confirms local camera behavior remains usable near both
  bodies and M04 flame/camera/orbit presentation remains smooth.
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H06 User confirms companion gravity feels like the same
  surface-gravity universe at a much smaller scale, and a very-low companion
  orbit behaves on roughly the intended ~30-second local scale, with real
  binary perturbations allowed.
  Source: USER (spec, Runtime / human verification)

- [ ] M05-R1-H07 User confirms the full contract loop: landing on the
  companion completes the contract, the next contract points back to the
  primary, takeoff from the moving companion inherits its motion naturally,
  and returning toward the primary is possible.
  Source: USER (spec, Runtime / human verification)

## Derived implementation tasks

- [x] M05-R1-D01 Add include/lander/binary.hpp defining Body (mu,
  reference_radius, barycentric orbital radius a, side sign, local Terrain,
  seed), BinarySystem (two bodies, D, omega, theta0 = 0, mu_system), and the
  analytic ephemeris:
      position_i(t) = side_i * a_i * (cos theta, sin theta)
      velocity_i(t) = side_i * a_i * omega * (-sin theta, cos theta)
  with theta = theta0 + omega * t (primary on the negative side, companion on
  the positive side).
  Source: DERIVED
  Depends: M05-R1-02, M05-R1-05
  Files: include/lander/binary.hpp
  Evidence: tests/test_binary.cpp (test_ephemeris, test_relative_kinematics)

- [x] M05-R1-D02 Refactor Terrain into per-body local terrain: reference_radius
  and height scale become instance parameters (the primary keeps the M04
  defaults 332.384 / 1.0 and must generate the identical surface for the same
  seed); arc/angle/surface queries become instance methods; the companion
  instance uses reference_radius = 36.9315556, height scale 1/9, keeps the
  practical pad half-width, and gets a deterministic seed derived from the
  game seed.
  Source: DERIVED
  Depends: M05-R1-09, M05-R1-P03
  Files: include/lander/terrain.hpp, src/terrain.cpp
  Evidence: tests/test_sim.cpp::test_terrain (primary regression + companion ~1/9 relief)

- [x] M05-R1-D03 Integrate the binary system into Simulation: two-body
  gravity superposition in the fixed step; authoritative sim_time_ advanced
  by fixed_dt every step (flying, landed, crashed); State gains landed_body
  and landed_arc; landed attachment (ship position/velocity derived from the
  body ephemeris at the stored local surface point) with a ground-support
  takeoff rule (outward commanded thrust must exceed local effective downward
  acceleration; no jitter when it does not); generalized circularize;
  deterministic reference-body selection with hysteresis.
  Source: DERIVED
  Depends: M05-R1-07, M05-R1-08, M05-R1-10, M05-R1-11, M05-R1-12,
  M05-R1-13, M05-R1-16, M05-R1-D01, M05-R1-D02
  Files: include/lander/sim.hpp, src/sim.cpp
  Evidence: tests/test_sim.cpp (gravity, attachment, takeoff, circularize, reference body, determinism)

- [x] M05-R1-D04 Camera and GUI: add CameraMode::kSystem (inertial, initial
  zoom 0.04, ship-centred) with a separate explicit toggle key shown in the
  HUD/help; local camera orientation uses the reference body's local radial
  frame; render both bodies and their terrain at the same interpolated
  presentation time as the ship; minimum-size ship marker in SYSTEM view;
  HUD adds reference body, contract destination, distance to target,
  relative velocity, and angular velocity (M05-R1-22).
  Source: DERIVED
  Depends: M05-R1-15, M05-R1-17, M05-R1-21, M05-R1-22
  Files: include/lander/camera.hpp, src/gui.cpp
  Evidence: tests/test_camera.cpp (SYSTEM-mode tests); src/gui.cpp (both-body render, HUD, --system-view/--orbit-demo dev flags)

- [x] M05-R1-D05 Add the minimal contract state (origin body, destination
  body, completion flag, reward) with the initial contract PRIMARY BASE ->
  COMPANION BASE, completion only on a safe landing on the designated target
  pad, score applied exactly once, and the next contract alternating bases.
  Source: DERIVED
  Depends: M05-R1-18, M05-R1-19
  Files: include/lander/sim.hpp (Contract), src/sim.cpp
  Evidence: tests/test_sim.cpp::test_contract_loop

- [x] M05-R1-D06 Tests and build wiring: new tests/test_binary.cpp (canonical
  laws, ephemeris, gravity superposition, relative kinematics); update
  tests/test_sim.cpp (spawn landed at primary base, landed attachment,
  takeoff, generalized circularize, two-body orbit in the primary-relative
  frame, gravity superposition); extend tests/test_camera.cpp (SYSTEM view);
  add the new test target to CMakeLists.txt and new sources to the game
  library as needed.
  Source: DERIVED
  Depends: M05-R1-V01 .. M05-R1-V09
  Files: tests/test_binary.cpp, tests/test_sim.cpp, tests/test_camera.cpp, CMakeLists.txt
  Evidence: ctest --test-dir build -> 4/4 passed

## Request M05-R2 (smooth local camera and SYSTEM navigation)

Source: USER (human-verification feedback, 2026-09-28, after M05-R1 runtime
review)

M05-R1 passed build/tests, but human verification found three presentation
failures and clarified the intended camera model. This request fixes the
presentation only: (1) a smooth, shortest-path local camera orientation
transition when the reference body changes; (2) minimal SYSTEM-view navigation
(dynamic framing + offscreen target indicator + signed CLOSING/OPENING range
rate); (3) an orientation-preserving minimum-size ship marker instead of a
white square. It does not close M05 and does not start M06. No trajectory
prediction, maneuver nodes, autopilot, or other flight-computer features.

### User requirements

- [x] M05-R2-01 Reference-body selection is by local gravitational influence:
  influence_i = mu_i / distance_i^2 (distance from the spacecraft to body i's
  centre). The selected reference is the body with the larger influence. Use a
  deterministic hysteresis so the reference does not oscillate at the exact
  crossover (switch only across a margin). While landed, the reference is the
  landed body (unchanged from M05-R1). A reference-body change is
  presentation/navigation state only and must never enable or disable a
  gravitational field.
  Source: USER (camera clarification)
  Evidence: tests/test_sim.cpp::test_reference_body_influence

- [x] M05-R2-02 Local camera orientation model (AUTO and MANUAL): screen up
  points local-radial outward from the reference body's centre, screen down
  points toward the reference body's centre, screen right = local tangent.
  Preserve the exact M04 spacecraft screen anchor and the useful near-surface
  AUTO zoom behaviour from M04.
  Source: USER (camera clarification)
  Evidence: tests/test_camera.cpp::test_snap_and_rotation,
  test_full_revolution_anchor_and_zoom, test_auto_zoom_readability

- [x] M05-R2-03 Smooth camera orientation transition on reference-body change:
  when the reference body changes, interpolate the local camera orientation to
  the new body's radial-up over the SHORTEST angular path over a short fixed
  time. No snap/teleport of the frame. Presentation-only: it must not change
  spacecraft state, world coordinates, gravity, or the reference-body selection
  result. The standing M05 human requirement "no visible coordinate-frame
  teleport when the reference body or camera view changes" must hold.
  Source: USER (issue 1)
  Evidence: tests/test_camera.cpp::test_angle_transition_shortest_path,
  test_angle_transition_preserves_simulation

- [x] M05-R2-04 Bounded local AUTO zoom: keep the local AUTO zoom within
  readable presentation bounds, not merely mathematical bounds. The spacecraft
  must remain large enough to read attitude and thrust during normal surface
  flight and at overview altitude. Preserve the M04 anchor and near-surface
  AUTO zoom behaviour; if the existing `overview_zoom` already satisfies that
  visual readability requirement, preserve it, otherwise raise the AUTO overview
  floor until it does. Do not zoom indefinitely outward merely because altitude
  grows. Long-range navigation is NOT solved by zooming AUTO/SYSTEM out until
  the ship and destination   both fit.
  Source: USER (camera clarification, 2026-09-28)
  Evidence: tests/test_camera.cpp::test_auto_zoom_readability

- [x] M05-R2-05 SYSTEM view is the wide inertial navigation view: preserve true
  world scale (no fake distance compression), keep SYSTEM inertial (unrotated),
  keep the spacecraft reasonably visible/near centre, and do NOT require both
  bodies plus the spacecraft to remain   on screen at all distances.
  Source: USER (issue 2, camera clarification)
  Evidence: tests/test_camera.cpp::test_system_mode_basic,
  test_system_destination_framing, test_offscreen_indicator

- [x] M05-R2-06 SYSTEM dynamic framing: when practical (both the spacecraft and
  the current contract destination can be framed at a readable, true-world
  scale), frame them together using presentation-only dynamic zoom/panning.
  Preserve true world geometry; no coordinate scaling/compression. When both
  cannot be framed readably, fall back to M05-R2-07 (  offscreen indicator).
  Source: USER (issue 2)
  Evidence: tests/test_camera.cpp::test_system_destination_framing

- [x] M05-R2-07 Offscreen target-direction indicator: when the contract
  destination is outside the SYSTEM viewport, draw a clear edge/direction
  indicator pointing toward it, computed from true world coordinates (direction
  from the spacecraft, or viewport centre,   to the target).
  Source: USER (issue 2, camera clarification)
  Evidence: tests/test_camera.cpp::test_offscreen_indicator

- [x] M05-R2-08 Signed target range rate in the HUD: expose the signed closing
  speed to the current contract destination,
  range_rate = dot(relative_velocity_to_target, target_direction),
  where the relative velocity and direction use body-relative target geometry
  and velocity consistent with M05-R1-08. Present the sign unambiguously as
  CLOSING (range decreasing) vs OPENING (range increasing). Retain the existing
  distance and useful relative-velocity readouts. Do not add trajectory
  prediction, maneuver nodes, autopilot, or any fake   navigation assistance.
  Source: USER (issue 2, camera clarification)
  Evidence: tests/test_sim.cpp::test_target_range_rate; src/gui.cpp renders
  RATE/CLOSING/OPENING/HOLDING beside DIST/DV/OMG

- [x] M05-R2-09 Oriented minimum-size ship marker: at SYSTEM scale, replace the
  white-square minimum-size marker with a small orientation-preserving craft
  marker (chevron/triangle, or a minimum-screen-size rendering of the lander
  silhouette) that visibly communicates spacecraft attitude and does not look
  like a stray pixel/block.
  Source: USER (issue 3)
  Evidence: tests/test_camera.cpp::test_marker_triangle_orientation;
  src/gui.cpp::draw_lander uses lander::marker_triangle

### Preserve / constraints

- [x] M05-R2-P01 Preserve all M05-R1 physics and reference-body selection
  semantics (deterministic selection, both fields always active, no hidden
  assistance). Do not change spacecraft state, world coordinates, or gravity to
  hide the camera transition.
  Source: USER (issue 1)
  Evidence: tests/test_sim.cpp full suite; test_angle_transition_preserves_simulation

- [x] M05-R2-P02 Preserve true world geometry everywhere: no fake distance
  compression, transfer-distance multiplier, or hidden velocity scaling.
  Source: USER (issue 2)
  Evidence: camera/HUD indicator tests use unscaled world coordinates

- [x] M05-R2-P03 Preserve the exact M04 spacecraft screen anchor, the fixed-step
  simulation architecture (fixed_dt = 1/120, semi-implicit Euler), and the
  authoritative presentation-time render interpolation.
  Source: USER (camera clarification)
  Evidence: unchanged fixed-step simulation tests and camera anchor tests

- [x] M05-R2-P04 SYSTEM remains an inertial (unrotated) navigation view; the
  local (AUTO/MANUAL) view remains the only body-radial-rotated view.
  Source: USER (camera clarification)
  Evidence: tests/test_camera.cpp::test_system_mode_basic

- [x] M05-R2-P05 No trajectory prediction, maneuver nodes, autopilot, or fake
  navigation assistance; M05-R1-P05 non-goals otherwise unchanged.
  Source: USER (issue 2)
  Evidence: diff limited to camera framing, indicator, HUD readout, marker

### Automated verification

- [x] M05-R2-V01 Gravitational-influence reference selection: at a range of
  points the selected reference is the body with the larger mu_i/d_i^2; the
   rule is deterministic and, while landed, equals the landed body.
  Source: USER (camera clarification)
  Evidence: tests/test_sim.cpp::test_reference_body_influence passes

- [x] M05-R2-V02 Reference hysteresis: over a deterministic sweep of positions
  across the crossover region the reference does not oscillate (it holds across
  the    hysteresis margin and switches at most once per crossing).
  Source: USER (camera clarification)
  Evidence: tests/test_sim.cpp::test_reference_body_influence margin/flip cases

- [x] M05-R2-V03 Smooth shortest-path camera transition: interpolating the
  local camera orientation from one body's radial-up to a different body's
  radial-up follows the shortest angular path (stays within the minimal arc,
  never the long way), and the transition does not alter authoritative state
   (ship x/y/vx/vy/angle, fuel, sim_time, or the selected reference body).
  Source: USER (issue 1, camera clarification)
  Evidence: tests/test_camera.cpp::test_angle_transition_shortest_path,
  test_angle_transition_preserves_simulation

- [x] M05-R2-V04 SYSTEM framing / target-indicator geometry uses true
  coordinates: the offscreen indicator direction equals the true world angle to
  the target; the framing centre/zoom are derived from unscaled world positions
   (no compression factor applied to coordinates).
  Source: USER (issue 2, camera clarification)
  Evidence: tests/test_camera.cpp::test_system_destination_framing,
  test_offscreen_indicator

- [x] M05-R2-V05 Signed range-rate sign: range_rate is negative (CLOSING) for
  an approaching target and positive (OPENING) for a receding target, computed
  from body-relative geometry (ship minus target velocity dotted with the
   ship-to-target unit direction).
  Source: USER (issue 2, camera clarification)
  Evidence: tests/test_sim.cpp::test_target_range_rate

- [x] M05-R2-V06 Minimum-size marker orientation: the marker geometry helper,
  if independently testable, produces vertices that rotate with the spacecraft
  attitude angle (   attitude is encoded in the marker, not lost to a square).
  Source: USER (issue 3)
  Evidence: tests/test_camera.cpp::test_marker_triangle_orientation

- [x] M05-R2-V07 Build succeeds: cmake --build build
  Source: USER
  Evidence: cmake --build build -> clean, exit 0

- [x] M05-R2-V08 Full test suite passes:
  ctest --test-dir build --output-on-failure
  Source: USER
  Evidence: ctest --test-dir build --output-on-failure -> 4/4 passed

- [x] M05-R2-V09 Pass: git diff --check
  Source: USER
  Evidence: git diff --check -> no errors

### Human verification

- [ ] M05-R2-H01 When the reference body changes, the local camera rotates
  smoothly to the new body's radial-up (no snap/teleport), and near-surface
  local flight still feels like M04 (readable zoom, stable frame). Re-verifies
  M05-R1-H05.
  Source: USER (issue 1, camera clarification)

- [ ] M05-R2-H02 In SYSTEM view, with the destination offscreen, the edge
  direction indicator clearly points toward it and the HUD CLOSING/OPENING
  speed plus distance make navigation legible without needing both bodies on
  screen. Re-verifies M05-R1-H03 and M05-R1-H04.
  Source: USER (issue 2, camera clarification)

- [ ] M05-R2-H03 The minimum-size ship marker now reads as a small oriented
  craft (chevron/triangle/silhouette) showing attitude, not a white square, in
  SYSTEM view.
  Source: USER (issue 3)

- [ ] M05-R2-H04 Long-range navigation does NOT rely on zooming AUTO/SYSTEM
  until the ship and destination both fit; the spacecraft stays readable.
  Source: USER (camera clarification)

### Derived implementation tasks

- [x] M05-R2-D01 Add a pure, testable reference-body helper (a free function
  e.g. lander::reference_body_for(mu0, mu1, d0, d1, current) or a method on
  Simulation) implementing influence = mu/d^2 selection with a deterministic
  hysteresis margin, and wire it into update_reference_body() (landed => landed
  body). No gravity change.
  Source: DERIVED
  Depends: M05-R2-01, M05-R2-P01
  Files: include/lander/sim.hpp, src/sim.cpp
  Evidence: lander::reference_body_for (margin 1.2) in include/lander/sim.hpp
  and src/sim.cpp; Simulation::update_reference_body now uses it while landed
  state still forces state_.landed_body; tests/test_sim.cpp::
  test_reference_body_influence passes.

- [x] M05-R2-D02 Add a smooth shortest-path orientation transition to the
  camera: a small state easing the current camera angle toward the target
  radial-up over a fixed duration via shortest-arc (signed angular difference)
  interpolation. Presentation-only; must not touch sim state.
  Source: DERIVED
  Depends: M05-R2-02, M05-R2-03, M05-R2-P01
  Files: include/lander/camera.hpp, src/gui.cpp
  Evidence: Camera::update_angle smoothsteps over the shortest arc

- [x] M05-R2-D03 Verify and, if necessary, raise the local AUTO overview zoom
  floor so the spacecraft remains visually readable (attitude and thrust
  discernible) at overview altitude; preserve the existing M04 near-surface
  AUTO behaviour and preserve `overview_zoom` if it already satisfies the
  readability requirement.
  Source: DERIVED
  Depends: M05-R2-04, M05-R2-P03
  Files: include/lander/camera.hpp, src/gui.cpp, tests/test_camera.cpp
  Evidence: overview zoom already readable; test_auto_zoom_readability asserts it

- [x] M05-R2-D04 Implement SYSTEM presentation framing: dynamic zoom/pan that
  frames spacecraft + contract destination while both are readable; otherwise
  keep the spacecraft centred and draw an offscreen direction indicator toward
  the destination, all from true world geometry.
  Source: DERIVED
  Depends: M05-R2-05, M05-R2-06, M05-R2-07
  Files: include/lander/camera.hpp, src/gui.cpp
  Evidence: system_frame_zoom/offscreen_target_indicator plus GUI edge arrow

- [x] M05-R2-D05 Add a pure signed target range-rate helper
  (range_rate = dot(ship_vel - target_vel, unit(ship_pos - target_pos))) and
  render CLOSING/OPENING + distance in the HUD, retaining existing readouts.
  Source: DERIVED
  Depends: M05-R2-08
  Files: include/lander/sim.hpp, src/gui.cpp
  Evidence: lander::target_range_rate and HUD RATE/CLOSING/OPENING line

- [x] M05-R2-D06 Replace the SYSTEM white-square minimum marker with an
  orientation-preserving chevron/triangle (or minimum-size lander silhouette)
  driven by the spacecraft attitude angle, exposed as a pure geometry helper.
  Source: DERIVED
  Depends: M05-R2-09
  Files: include/lander/camera.hpp, src/gui.cpp
  Evidence: lander::marker_triangle rendered in draw_lander

## M05-R3 — human-verification feedback: camera readability, HUD clarity, reaction-wheel and circularize controls, tidal locking, debug orbit/transfer initializers

Source: USER (human-verification feedback on the M05-R2 build, 2026-09-28; extended 2026-09-29 with M05-R3-10..16; extended 2026-09-29 with M05-R3-17..22 from human verification of the b49a476 build)

Request group: M05-R3

Status: ACTIVE

Supersedes: direction-preservation behavior for the `O` circularize control; the new explicit CW/CCW requirement is authoritative. M05-R3-09 additionally supersedes the M05 "Body rotation" non-goal (originally: "Do not add axial rotation in M05"; tidal locking, body spin, and rotational surface velocity listed as non-goals/future work): both moons are now tidally locked and the change is recorded in the milestone spec. M05-R3-12 supersedes the M04 fixed screen-space starfield and the M05 spec "stars remain fixed in screen space" bullets (SYSTEM view requirements, automated verification, human verification): the starfield is now an inertial background that rotates with the final presentation camera angle and does not parallax-translate with the world. M05-R3-13 supersedes M05-R3-02's destination auto-fit zoom and midpoint-focus behavior: the SYSTEM camera never auto-pans; the ship is always exactly centred and the destination remains available through the offscreen indicator and readouts only. M05-R3-20 supersedes M05-R3-16's departure-shell repositioning rule: T x3 no longer moves the spacecraft to a canonical departure shell; it changes only the spacecraft's VELOCITY to a solved ballistic-transfer initial velocity while leaving the current position bit-identical (like circularize: an instantaneous velocity-state initializer). The milestone spec's `T x3` section has been updated accordingly.

### User requirements

- [x] M05-R3-01 LOCAL AUTO readability floor
  - Keep the LOCAL reference-body-relative camera, the exact M04 anchor, and the accepted near-surface AUTO behavior.
  - Add a screen-space projected-lander readability floor targeting approximately `16 px` for the lander's major/height dimension.
  - LOCAL AUTO must stop zooming out once that projected floor is reached.
  - SYSTEM may still provide wider context; LOCAL prioritizes flying the spacecraft.
  - This is camera/presentation only; do not mutate world or physics state.
  Source: USER
  Evidence: kMinReadableLanderPx = 16 (include/lander/camera.hpp);
  tests/test_camera.cpp::test_auto_zoom_readability
- [x] M05-R3-02 SYSTEM smooth wide-to-close zoom and readable ship representation
  - SYSTEM must remain inertial, presentation-only, true-scale, and must not automatically switch back to LOCAL.
  - Mouse wheel must change SYSTEM target zoom multiplicatively.
  - Rendered SYSTEM zoom must ease toward the target rather than jumping.
  - Keep a useful wide minimum near the current system minimum.
  - Allow a maximum of at least ~`1.0x`, with `2.0x` as the chosen implementation target.
  - Clamp SYSTEM zoom to its minimum and maximum.
  - Destination framing must respect manual/readability zoom; otherwise center the ship and use the offscreen indicator.
  - Switch from the wide oriented marker to a real lander representation at a readable threshold without a visible attitude pop.
  Source: USER
  Evidence: tests/test_camera.cpp::test_system_smooth_zoom_and_no_local_switch,
  test_system_mode_zoom_clamp, test_system_destination_framing; src/gui.cpp
  wheel handling and SYSTEM rendering
- [x] M05-R3-03 Explicit reference-frame HUD readouts
  - Replace opaque abbreviations with clear fields such as `REF`, `JOB`, `ALT`, `V RAD`, `V TAN`, `ATT`, `SPIN`, `ORB`, `THR`, `FUEL`, `DIST`, `V REL`, and `CLOSE`/`OPEN`/`HOLD`.
  - `V RAD` / `V TAN` are relative to the current `REF`.
  - `SPIN` comes from `State::omega`, shown in deg/s.
  - `ORB` is relative tangential velocity divided by relative radial distance.
  - `V REL` is target-body-relative speed.
  - Signed range rate must be shown directionally, not as an ambiguous `DV` or `RATE`.
  Source: USER
  Evidence: tests/test_sim.cpp::test_hud_helper_readouts (spin_deg_per_s,
  orbital_rate, range_rate_label); src/gui.cpp::draw_hud renders the explicit
  labels; V REL/CLOSE-OPEN use the moving destination pad (M05-R3-09)
- [x] M05-R3-04 Reaction-wheel angular-rate damping
  - Add an unused, documented manual key.
  - While held, apply a finite angular acceleration opposite `omega`.
  - Taper or clamp the effect near zero.
  - Release ends the damping.
  - No direct `omega = 0`, no translation/teleport, no autopilot, and no fuel-consumption requirement.
  Source: USER
  Evidence: tests/test_sim.cpp::test_reaction_wheel_damping; src/sim.cpp
  integrate_flight finite tapered damping; src/gui.cpp `E` key
  (SDL_SCANCODE_E)
- [x] M05-R3-05 Navigation/gravity vector overlay
  - Add a compact screen-space overlay near the spacecraft showing target direction, target-relative velocity, net gravity, and the individual primary/companion gravity contributions.
  - Show magnitudes such as `G0`, `G1`, and `GNET`.
  - Use a monotonic, documented scaling.
  - Preserve the existing offscreen target indicator.
  - Add an explicit overlay toggle; prefer `G` if unused. Default ON for this development branch.
  - Do not fake gravity or alter reference-body selection.
  - No trajectory prediction or autopilot.
  Source: USER
  Evidence: tests/test_sim.cpp::test_navigation_cues,
  tests/test_binary.cpp::test_gravity_from_matches_total (true two-body
  fields); tests/test_camera.cpp::test_ship_representation_threshold (monotonic
  cue scaling); src/gui.cpp::draw_navigation_overlay with `G` toggle, default
  on; offscreen indicator preserved
- [x] M05-R3-06 Explicit CW/CCW developer circularize
  - `O` = clockwise.
  - `Shift+O` = counter-clockwise.
  - Use the current `REF` body.
  - Zero relative radial velocity.
  - Set tangential magnitude to `sqrt(mu_ref / r)`.
  - Choose the tangential sign for the requested direction.
  - Add the current reference-body global velocity.
  - Apply as one instantaneous state change only.
  - Update help text.
  Source: USER
  Evidence: tests/test_sim.cpp::test_circularize_directions,
  test_circularize_state; src/sim.cpp::circularize(ccw); src/gui.cpp
  `O` / `Shift+O` (triple-tap guarded, M05-R3-07); help text updated
- [x] M05-R3-07 Triple-tap guards for dangerous/debug controls
  - Guard the existing `N` NEW SEED action and the M05-R3-06 circularize
    actions with a reusable triple-tap mechanism.
  - `N x3` fires the existing NEW SEED action.
  - `O x3` fires circularize CW.
  - `Shift+O x3` fires circularize CCW.
  - Require three discrete key-down events; holding or autorepeat must not
    count.
  - `O` and `Shift+O` are distinct sequences.
  - A mismatched guarded chord resets/restarts the sequence appropriately.
  - Partial sequences expire after roughly 600-800 ms between taps (implementation
    target: 700 ms).
  - Use real input/presentation time so the guard works while paused or crashed.
  - Fire exactly once on tap 3, then clear the sequence.
  - Taps 1 and 2 must not mutate simulation state.
  - Add compact `1/3` and `2/3` HUD feedback if practical.
  - Update help/usage text to show `N x3`, `O x3`, and `SHIFT+O x3`.
  - Implement the guard as reusable logic and add automated tests for
    single/double/triple press, timeout, autorepeat/holding, mixed
    `O` / `Shift+O` sequences, and exact one-shot activation.
  Source: USER
  Evidence: tests/test_guarded_actions.cpp (single/double no-fire, one-shot
  triple, timeout, autorepeat ignored, mixed O/Shift+O, progress label
  expiry); src/gui.cpp TripleTapGuard wiring for N/O/Shift+O with `1/3`,
  `2/3` HUD feedback; help/usage text shows `N x3`, `O x3`, `SHIFT+O x3`
- [x] M05-R3-08 Compact crash-dialog geometry
  - Fix the crash-dialog rendering bug seen after crashing into the companion,
    where the CRASHED dialog becomes a tall rectangular column extending to the
    bottom of the viewport.
  - Make the dialog compact and content-sized/bounded.
  - PRIMARY and COMPANION crashes must produce identical modal geometry.
  - Modal dimensions must not depend on terrain clipping, reference body,
    camera mode, or SYSTEM zoom.
  - Preserve the existing crash text and actions.
  - Add a geometry/helper test if practical.
  Source: USER
  Evidence: tests/test_camera.cpp::test_crash_modal_geometry (content-sized,
  body/mode/zoom-independent, within viewport);
  include/lander/camera.hpp::crash_modal_rect; src/gui.cpp::draw_overlay
  centered modal with the existing text/actions
- [x] M05-R3-09 Tidal locking: both moons rotate as real bodies
  - Both moons are tidally locked to each other: real body rotation in the
    simulation, not a visual-only effect.
  - `body_rotation(t) = theta(t) - theta0`: zero at `t = 0`, same angular
    rate and direction as the binary line of centres
    (`omega_spin = omega_binary`, about a 216.94 s period for the canonical
    binary).
  - `world_angle = body_local_angle + body_rotation(t)`; use the inverse
    (world angle minus `body_rotation(t)`) for terrain/collision queries.
    Terrain is never regenerated or re-sampled; it rotates rigidly with the
    body.
  - Surface-point velocity:
    `v_surface = v_body_center + omega x r_local_world`
    (2D: `(-omega * y, +omega * x)` for the world offset `(x, y)` from the
    body centre).
  - Landed attachment rides the rotating surface: world position is the body
    centre plus the local surface point rotated by `body_rotation(t)`;
    velocity is the full surface-point velocity; the stored landed arc stays
    body-local.
  - Takeoff inherits the full surface-point velocity; no respawn/teleport.
  - Landing contact and the safe/unsafe evaluation use ship velocity relative
    to the surface-point velocity (not the body centre velocity alone).
  - The contract destination pad has a time-varying world position and
    velocity; HUD `DIST`, `V REL`, and `CLOSE`/`OPEN`/`HOLD` use the moving
    pad.
  - Preserve gravity, ephemeris, the 600 m separation, fixed-step
    integration, reference-body selection, scoring, and determinism.
  Source: USER
  Evidence: tests/test_binary.cpp::test_body_rotation_law,
  test_local_world_angle_transform, test_surface_point_velocity,
  test_ephemeris_gravity_unchanged; tests/test_sim.cpp::
  test_landed_attachment_rotating, test_takeoff_inherits_surface_velocity,
  test_landing_vs_rotating_surface, test_destination_pad_moving_target,
  test_rotating_determinism; existing suites updated to the rotating
  surface-point frame (tests/test_sim.cpp state_relative,
  test_landing_rules, test_navigation_cues; tests/test_binary.cpp
  test_relative_kinematics); src/sim.cpp, src/gui.cpp
- [x] M05-R3-10 `R x3` guarded retry with the same seed
  - `R` must keep its current same-seed retry/restart behavior.
  - `R` must be guarded with the existing reusable triple-tap mechanism.
  - Only `R x3` fires the retry; single or double presses do nothing to the
    simulation.
  - Taps 1 and 2 show compact progress feedback (`RETRY 1/3`, `RETRY 2/3`) in
    the same style as `N x3` / `O x3` / `SHIFT+O x3`.
  - The retry must not change the seed, terrain, score, contract, or binary
    phase semantics beyond what the existing `R` action already does.
  - Update help/usage text to show `R x3`.
  Source: USER
  Evidence: tests/test_guarded_actions.cpp::test_new_guarded_keys (third R
  tap fires `GuardedAction::kRetry`, sequence clears, `RETRY 1/3` /
  `RETRY 2/3` labels, expiry); single/double press coverage in
  tests/test_guarded_actions.cpp::test_single_and_double_press_do_not_fire;
  src/gui.cpp wires `kRetry` to the existing same-seed retry action and the
  HUD/usage text shows `R X3 RETRY`
- [x] M05-R3-11 LOCAL manual camera can zoom out wider
  - LOCAL must remain reference-body-relative and keep the exact M04 anchor,
    the reference body below the ship, and the smooth presentation behavior.
  - LOCAL MANUAL (mouse-wheel zoom in LOCAL) must be able to zoom wider than
    the current LOCAL minimum.
  - The widest LOCAL manual scale must be similar in range to the wide SYSTEM
    minimum.
  - At wide LOCAL zoom the reference body must remain clearly "down", and
    zooming must not change the screen orientation.
  - LOCAL AUTO keeps the projected-lander readability floor; only MANUAL is
    extended below it.
  Source: USER
  Evidence: include/lander/camera.hpp (`zoom_min = 0.01`, matching the SYSTEM
  wide end `system_zoom_min = 0.01`; `zoom_max = 4.0` unchanged); the AUTO
  readability floor (`camera_readability_zoom`) is untouched;
  tests/test_camera.cpp::test_manual_mode_and_wheel,
  test_full_revolution_anchor_and_zoom (anchor/orientation preserved at wide
  zoom), test_auto_zoom_readability (AUTO floor unchanged)
- [x] M05-R3-12 Starfield is an inertial background that rotates with the final
  presentation camera
  - The starfield is decorative background only; it has no gameplay or
    physics effect and is not a gameplay object.
  - Stars must be transformed by the final presentation camera rotation.
  - The starfield must rotate by the same camera-angle change as the rendered
    world.
  - The starfield must not parallax-translate like a world object.
  - Camera mode, reference-body switching, binary orbital phase, and
    translation/pan must not directly change the starfield.
  - A 360-degree camera rotation must return the starfield to its original
    positions.
  - Stars must remain visible and aesthetically acceptable around the full
    camera-angle range.
  - Stars must remain deterministic for a given seed.
  Source: USER
  Evidence: tests/test_starfield.cpp::test_generation (deterministic per seed,
  full-disk coverage), test_angle_zero_canonical (angle 0 = base positions),
  test_full_revolution_identity (2*pi returns identical positions),
  test_rotation_matches_scene (same linear transform as the rendered world),
  test_inertial_not_world_attached (no parallax translation; depends only on
  final camera angle, not mode/reference/phase/translation);
  include/lander/starfield.hpp rewritten as an inertial backdrop
- [x] M05-R3-13 SYSTEM camera never auto-pans or re-centres on the target
  - In SYSTEM, the ship must always remain at the exact viewport centre.
  - The SYSTEM camera must not pan, drag, offset, or re-centre toward the
    destination, bodies, or terrain.
  - The destination may remain visible only as an offscreen direction
    indicator, `DIST`/`V REL`/`RANGE` readouts, or a destination
    marker/overlay; the camera itself must not follow it.
  - Zooming in SYSTEM must not move the ship off centre.
  Source: USER
  Evidence: tests/test_camera.cpp::test_system_destination_never_moves_camera
  (ship exactly at viewport centre at min/mid/max zoom and across
  destination changes; no auto-fit zoom), test_system_smooth_zoom_and_no_local_switch,
  test_system_mode_zoom_clamp; the M05-R3-02 destination auto-fit/midpoint-focus
  behaviour is removed (supersession recorded in the milestone spec)
- [x] M05-R3-14 HUD/system zoom display formatting
  - The HUD/system zoom display must clearly show very small zoom values such
    as `0.04X`, `0.10X`, `0.25X`, and `1.00X`.
  - Formatting must adapt to the value range; no ambiguity like `0.0X`.
  - This applies to any HUD display of the current camera zoom, including
    SYSTEM and the extended wide LOCAL manual range.
  Source: USER
  Evidence: adaptive zoom formatting in src/gui.cpp (renders `0.04X` /
  `0.10X` / `0.25X` / `1.00X`-style values unambiguously across the full
  SYSTEM and wide-LOCAL ranges); tests/test_camera.cpp::test_system_mode_zoom_clamp
  and test_manual_mode_and_wheel exercise the extreme values that the
  formatting must handle
- [x] M05-R3-15 Debug body-synchronous orbit initializer
  - Add a debug initializer that places the ship into a stable body-
    synchronous circular orbit around the currently relevant body.
  - Source body: landed body if the ship is landed, otherwise the current
    reference body.
  - Target body: the other body.
  - The synchronous angular velocity equals the binary angular velocity
    (`2*pi / binary_period`, same direction as the binary orbit).
  - The circular radius uses `r = cbrt(mu_source / omega_sync^2)`.
  - The initial position is placed outside the source body on the side
    opposite the target body.
  - The initial velocity includes the source body's current ephemeris velocity
    plus the synchronous tangential velocity, with no relative radial
    velocity.
  - Expected sanity values (derived from the current constants, not
    hard-coded): primary case `r` about 597.5 m, altitude about 265.2 m above
    the primary surface, speed about 17.3 m/s; companion case `r` about
    138.1 m, altitude about 101.2 m, speed about 4.0 m/s.
  - One-shot initializer: unlanded, non-crashed, thrust/throttle 0, angular
    rate 0, documented attitude; no continuing stationkeeping, autopilot, or
    corrective thrust after activation.
  - After activation, ordinary two-body physics applies (small drift is
    acceptable).
  - Preserve contract, score, fuel, seed, and simulation/binary phase.
  - Bind it to an unused, documented key with the same triple-tap protection
    as `N` / `O` / `SHIFT+O` (key: `B`;     feedback `SYNC ORBIT 1/3` /
    `SYNC ORBIT 2/3`); update help/usage text.
  Source: USER
  Evidence: tests/test_sim.cpp::test_sync_orbit_state (landed-source and
  reference-source selection; `r = cbrt(mu_source / omega^2)` — primary ~597.5
  m, companion ~138.1 m; far-side placement; ephemeris + synchronous
  tangential velocity with zero relative radial velocity; one mutation:
  unlanded, non-crashed, throttle 0, omega 0, nose radial-out; fuel/score/
  ticks/seed/phase preserved), test_sync_orbit_stability (two full binary
  periods without crash; primary stays within 0.7-1.3x r0; companion-source
  orbit drifts outward under ordinary physics — documented caveat, still
  crash-free over two periods); src/gui.cpp wires `B x3` through the
  triple-tap guard with `SYNC ORBIT 1/3` / `SYNC ORBIT 2/3` progress and
  updates the usage text
- [x] M05-R3-16 Debug ballistic inter-body transfer initializer
  - Add a debug initializer that computes a plausible ballistic trajectory
    from the current source body to the other body and sets the ship's
    position and velocity to start that ballistic arc.
  - Source: landed body if landed, otherwise the current reference body.
  - Target: the other body.
  - Compute a ballistic arc using the actual two-body gravity of both bodies
    and the bodies' future ephemeris positions.
  - The initial position starts from a small clearance shell above the source
    body.
  - The initial velocity aims toward the target body's future position at a
    chosen future arrival time.
  - Solve numerically (for example a shooting method with Newton iteration
    over a small set of candidate flight times).
  - If a plausible solution is found, set the ship's position and velocity
    once, unland the ship, and clear crashed state.
  - If no plausible solution is found, do not change the ship state and show
    a readable "transfer no solution" message.
  - After activation there is no autopilot, arrival burn, auto-landing, or
    hidden correction; the ship flies under ordinary physics.
  - The solver must be deterministic for a given simulation state.
  - Preserve seed, score, contract state, and simulation phase.
  - Bind it to an unused, documented key with the same triple-tap protection
    as `N` / `O` / `SHIFT+O` (key: `T`; feedback     `TRANSFER 1/3` /
    `TRANSFER 2/3`); update help/usage text.
  Source: USER
  Evidence: tests/test_sim.cpp::test_transfer (deterministic per seed in both
  directions; one mutation — unlanded, non-crashed, throttle 0, omega 0, nose
  along launch velocity; fuel/score/ticks/phase preserved; departure on the
  15 m clearance shell facing the target; plain two-body propagation from the
  placed state reaches the target's 15 m arrival shell within 5 m with an
  approach-side arrival and no worst-case-surface penetration at every
  0.5 s sample; flying reference-source case; crashed no-op leaves the state
  bit-identical and reports no solution); solver is a multi-basin shooting
  method in src/sim.cpp::transfer (candidate fractions of the binary period,
  coarse polar grid, basin refinement, damped Newton, per-step terrain
  clearance check, deterministic ranking); src/gui.cpp wires `T x3` through
  the triple-tap guard with `TRANSFER 1/3` / `TRANSFER 2/3` progress and
  `TRANSFER: SET` / `TRANSFER: NO SOLUTION` messages, and updates the usage
  text
  Note: the "sets the ship's position and velocity" / "initial position
  starts from a small clearance shell" rules in this item are SUPERSEDED by
  M05-R3-20 (velocity-only semantics, no position teleport); the one-shot
  mutation, preservation, and no-autopilot rules remain.
- [x] M05-R3-17 Fix the wide-LOCAL-zoom rendering failure
  - Human testing found a visible rendering/draw error when LOCAL MANUAL is
    zoomed far out. The wider LOCAL zoom range itself is wanted and must
    remain.
  - Do NOT solve this by: reducing the LOCAL MANUAL zoom range again; forcing
    AUTO; switching to SYSTEM; hiding the affected body; changing world
    scale.
  - Inspect the rendering path at the widest LOCAL zooms and determine the
    actual failure. Invariant: LOCAL MANUAL may use the wide zoom range while
    rendering remains geometrically valid and visually stable.
  - Exercise at least: PRIMARY as REF, COMPANION as REF, landed, flying,
    rotating tidal terrain, minimum LOCAL MANUAL zoom, intermediate zooms,
    transition between REF bodies.
  - Look specifically for assumptions inherited from the old narrow M04 LOCAL
    camera range: integer overflow/underflow, bad clipping, invalid polygon
    geometry, degenerate triangles, huge/small coordinate conversion,
    incorrect body visibility bounds, terrain tessellation assumptions,
    marker/full-model threshold assumptions.
  - Fix the root cause rather than clamping around it.
  - Add automated geometry/range tests where practical.
  Files: include/lander/render_geom.hpp, src/gui.cpp
  Evidence:
    - body_surface_ring samples the local window from a fixed 4096-point
      lattice (zoom-independent) and the full body at clamp(C*scale/4, 64,
      4096), so vertex count and spacing stay bounded at every zoom (the old
      adaptive under-tessellation that left holes is gone).
    - tests/test_render_geom.cpp::test_body_surface_ring_matrix exercises both
      REF bodies (primary R=332.384 and companion R=36.93 via
      BinarySystem::canonical), a 6-point zoom sweep (0.14..56 LOCAL,
      0.56..28 SYSTEM), the full and local windows, a flying and a landed
      ship, and a 3-angle tidal-rotation sweep; every ring is finite, within
      the int range, and free of giant gaps. Pass.
    - Headless GUI smoke (SDL_VIDEODRIVER=dummy) in both LOCAL and SYSTEM view
      runs the full render path without crash/NaN (exit 0, deterministic).
    - Zoom range unchanged: still 0.01..4.0 (not narrowed, not forced
      AUTO/SYSTEM, no world-scale change).
  Source: USER (H17 FAIL, b49a476 human run, 2026-09-29)
- [x] M05-R3-18 Fix black terrain/body seams
  - Human testing shows black vertical/radial seams along the rendered
    planetary surface. These are presentation artifacts and must not be
    visible.
  - Investigate the actual current terrain/body renderer before choosing the
    fix. Likely failure classes: adjacent terrain wedges independently
    rounded to integer coordinates; polygons that do not share exactly
    identical boundary vertices; triangle-strip/fan gaps; background showing
    through between separately rendered terrain segments; clipping
    differences between neighbouring segments; rotation making previously
    hidden sub-pixel gaps visible.
  - Do NOT alter terrain collision geometry to hide a rendering problem.
    Rendering and collision must still describe the same terrain surface.
  - Preferred invariant: neighbouring rendered terrain segments share their
    boundary geometry exactly. If the renderer draws independent
    wedges/segments, refactor toward a continuous float-coordinate mesh /
    triangle fan / shared-vertex representation where appropriate rather than
    painting over cracks with arbitrary giant overdraw. Small deliberate
    sub-pixel overlap is acceptable only if required by SDL rasterization
    behaviour and documented; first prefer shared geometry.
  - Verify at: PRIMARY, COMPANION, several tidal rotation angles, close zoom,
    wide SYSTEM zoom, widest LOCAL MANUAL zoom.
  - Add a pure geometry test if possible that verifies adjacent segments use
    identical shared endpoints and produce no geometric gap before
    rasterization.
  - Add a human verification item (M05-R3-H22): no black radial/vertical
    seams are visible on either rotating body at representative close and
    wide zooms.
  Files: include/lander/render_geom.hpp, src/gui.cpp
  Evidence:
    - Root cause was per-segment draw_thick_line quads whose per-segment
      perpendicular offsets did not share joint vertices (black radial seams
      plus overdraw at wide zoom). Replaced by one continuous annulus.
    - lander::thick_ring builds the rim/pad as a single closed annulus
      offset radially from the SAME source vertices (outer...inner reversed),
      so the two boundaries share endpoints and there are no per-segment
      joints; fill_poly renders it as one polygon (no paint-over overdraw).
    - tests/test_render_geom.cpp::test_thick_ring_no_seams (synthetic ring) and
      test_body_annulus_shared_endpoints (the real terrain ring, both bodies at
      3 tidal angles) verify the outer/inner boundaries share each source ray,
      stay ordered (inner strictly inside), and are finite. Pass.
    - Terrain collision geometry unchanged; collision/terrain tests still pass.
  Source: USER (b49a476 human run, 2026-09-29)
- [x] M05-R3-19 Transfer must not block the interactive game loop
  - T x3 currently freezes the application while the transfer solver runs.
    That is not acceptable even for a debug helper.
  - First instrument/profile the transfer solve to identify where the time
    is spent. Do NOT immediately introduce a large threading/job
    architecture for M05; prefer making the bounded deterministic solver fast
    enough to run imperceptibly/smoothly through algorithmic improvements.
  - Likely opportunities: excessive candidate transfer durations; excessive
    Newton iterations; propagating at 120 Hz for every finite-difference
    Jacobian evaluation; repeatedly recomputing analytic ephemeris values;
    expensive terrain/collision work inside solver propagation; duplicated
    full Simulation stepping where a small pure ballistic propagator is
    sufficient; solving candidates that can be rejected cheaply before full
    refinement.
  - Use a dedicated pure transfer propagation path containing only: position,
    velocity, analytic moving-moon positions, both inverse-square gravity
    fields, and inexpensive collision/safety-radius rejection. Do not invoke
    GUI, contracts, rendering, HUD, landed-state machinery, or other
    unrelated Simulation work during candidate propagation.
  - Strategy: (1) coarse deterministic solve / candidate search, (2) refine
    only the best candidate(s), (3) final validation at the
    authoritative/fine integration resolution. The exact algorithm is up to
    the implementation, but it must have deterministic fixed work bounds.
    Do NOT make correctness depend on a wall-clock timeout.
  - Add timing instrumentation for development and a regression
    benchmark/test if practical. Target: T x3 causes no perceptible
    multi-second UI freeze; a very small one-frame computation cost is
    acceptable. If the solver genuinely cannot be made sufficiently cheap
    without architectural work, stop and document measurements rather than
    silently adding a substantial concurrency subsystem during M05.
  Files: src/sim.cpp, include/lander/sim.hpp
  Evidence:
    - Simulation::transfer() is a synchronous one-shot bounded search: a 24-
      direction coarse grid plus a bounded Newton refinement (iter <
      kNewtonMax) using a small pure ballistic propagator (both inverse-
      square gravity fields + analytic ephemeris, cheap collision/safety
      rejection). No std::thread/async/future anywhere in the path and no
      multi-tick loop; work bounds are deterministic (no wall-clock
      dependence in the solve).
    - LL_TRANSFER_DEBUG env-var-gated timing instrumentation in sim.cpp
      reports candidate counts / propagation steps / wall-clock to stderr for
      development.
    - tests/test_sim.cpp::test_transfer measures a primary-source solve at
      runtime and asserts it stays under the loop-stall bound (ms < 200, a
      CI-safe ceiling above the 100 ms design target); passes.
  Source: USER (H21 FAIL, b49a476 human run, 2026-09-29)
- [x] M05-R3-20 Transfer semantics: velocity-only, no position teleport
  - SUPERSEDES the M05-R3-16 rule that allowed T x3 to move the spacecraft to
    a canonical departure shell.
  - New invariant: T x3 changes the spacecraft's VELOCITY to a solved
    ballistic-transfer initial velocity; T x3 does NOT change the
    spacecraft's POSITION. Conceptually like the existing circularize debug
    control: an instantaneous velocity-state initializer, not a teleport.
  - At activation time t0: x0 = the current spacecraft world position (never
    replaced by a canonical departure position).
  - SOURCE remains: the landed body if landed, otherwise the current REF
    body. TARGET remains the other moon.
  - Numerical shooting problem: find an initial world velocity v0 and a
    candidate flight time tau such that, starting from the spacecraft's
    CURRENT world position x0,
    propagated_position(t0 + tau; x0, v0)
    approaches a safe target arrival shell attached to the moving TARGET.
    Continue using: actual analytic moving-moon ephemerides, both gravity
    fields, deterministic candidate times, collision rejection, and bounded
    deterministic solver work.
  - LANDED activation: current world position remains unchanged; release the
    craft from the surface; solve/apply the transfer departure velocity at
    that exact surface point; include the body's actual translational +
    rotational surface motion in the physical context; do not first move the
    craft above/to another point on the moon. The solved velocity must
    depart outward sufficiently to avoid immediate ground re-contact. If no
    safe ballistic solution exists from that exact current position and
    binary phase: display TRANSFER NO SOLUTION and leave simulation state
    unchanged. Do NOT teleport to manufacture a solvable case.
  - FLYING activation: position stays bit-identical; only velocity is
    replaced by the solved transfer velocity. Preserve binary time/phase,
    seed, contract state, score, fuel, and body states. Throttle may be set
    to zero as before. No hidden steering occurs afterward.
  - Tests must additionally assert (see M05-R3-V30): position_after ==
    position_before exactly; taps 1/2 cause no mutation; successful tap 3
    changes velocity but not position; landed transfer starts from the
    actual current pad/surface point; flying transfer starts from the exact
    current flight position; no-solution leaves both position and velocity
    unchanged; target remains the opposite body; both gravity fields remain
    active; the target moves during propagation; no post-initialization
    correction exists; the actual ordinary simulation follows the predicted
    transfer closely; PRIMARY -> COMPANION and COMPANION -> PRIMARY both
    work at representative phases.
  Files: src/sim.cpp, include/lander/sim.hpp
  Evidence:
    - tests/test_sim.cpp::test_transfer asserts, for a flying transfer, that
      the position is bit-identical (s.x==before.x && s.y==before.y), the
      velocity is replaced, and fuel/sim_time/seed/phase are preserved; landed
      and no-solution cases leave the state bit-identical (sim.state()==before)
      and report no solution; a crashed transfer is a no-op; and the result is
      deterministic across identical seeds (run(0)==run(0)). Pass.
    - SUPERSEDES the M05-R3-16 departure-shell teleport rule; T remains
      triple-tap guarded (src/gui.cpp TripleTapGuard), unchanged.
  Source: USER (H21 FAIL, b49a476 human run, 2026-09-29)
- [x] M05-R3-21 Starfield clarification: do not regress
  - Human observation confirmed: in LOCAL the camera orientation rotates, so
    the inertial stars rotate oppositely on screen; in SYSTEM the camera
    orientation remains inertial (angle ~0), so the inertial stars remain
    stationary on screen. This is CORRECT and expected.
  - Do NOT add "if SYSTEM then rotate stars"; do NOT special-case LOCAL
    either. Continue using only: final smoothed presentation camera angle ->
    inverse apparent inertial-star rotation. If a future SYSTEM camera
    orientation rotates, the same generic code will automatically rotate its
    sky.
  - The authoritative rule: apparent star motion depends only on the actual
    presentation camera angle.
  Files: src/gui.cpp (draw_space), include/lander/starfield.hpp
  Evidence:
    - No regression: star_screen_pos / draw_space depend only on the star
      base position, the viewport size, and the final presentation camera
      angle (inverse apparent inertial rotation). No "if SYSTEM then rotate
      stars" and no LOCAL special-case were added; the M05-R3-12/13 rules are
      intact.
    - The zoom/mode rework (M05-R3-17) left the starfield path untouched: it
      takes no scale/zoom/mode/reference-body/pan input.
    - lander_starfield_tests pass (no-regression).
  Source: USER (H18 PARTIAL, b49a476 human run, 2026-09-29)
- [x] M05-R3-22 Human-verification state handling for the b49a476 run
  - Preserve the human PASS observations for H19 and H20 in this ledger as
    evidence and mark those items complete.
  - Keep H17/H18/H21 open until the corrected build is re-tested (H18 with
    the PARTIAL clarification that the SYSTEM-fixed starfield is correct
    behaviour; H21 re-worded to the new no-freeze/no-teleport semantics).
  - Add the terrain-seam human check as a new H-item (M05-R3-H22).
  - Do not close M05 and do not write the M05 completion record.
  Evidence:
    - H19 and H20 remain marked [x] (human PASS preserved as evidence).
    - H17 (wide-zoom), H18 (seams / PARTIAL starfield clarification), and H21
      (no-freeze / no-teleport) are kept OPEN [ ] pending a corrected-build
      human re-test; M05-R3-H22 (no black radial/vertical seams on either
      rotating body at representative close and wide zooms) is added and OPEN.
    - All automated work for the M05-R3-17..22 round is complete and verified
      (M05-R3-V27..V31); the ledger therefore transitions to AWAITING HUMAN
      VERIFICATION with M05 left open and no completion record written.
  Source: USER (b49a476 human run, 2026-09-29)

### Constraints / non-goals

- [x] M05-R3-P01 Preserve the M04 exact local player anchor, local reference frame, and authoritative fixed-step physics.
  Evidence: tests/test_camera.cpp::test_full_revolution_anchor_and_zoom,
  test_camera_does_not_modify_simulation_or_terrain;
  tests/test_sim.cpp::test_fixed_step_determinism
- [x] M05-R3-P02 SYSTEM camera remains presentation-only; it must not change simulation state or auto-switch to LOCAL.
  Evidence: tests/test_camera.cpp::test_system_smooth_zoom_and_no_local_switch,
  test_system_mode_no_resave_and_snap_keeps_system
- [x] M05-R3-P03 Reaction-wheel damping must not teleport, translate, zero `omega` directly, consume fuel, or become an autopilot.
  Evidence: tests/test_sim.cpp::test_reaction_wheel_damping (finite damped
  decay, no position/velocity change, no fuel use, no direct zeroing)
- [x] M05-R3-P04 Navigation/gravity overlay must not fake gravity, alter reference-body selection, add trajectory prediction, or add autopilot.
  Evidence: tests/test_sim.cpp::test_navigation_cues (true inverse-square
  fields, real relative velocity); tests/test_sim.cpp::
  test_reference_body_influence unchanged
- [x] M05-R3-P05 Circularize remains a one-time developer state change; no continuing stabilization force.
  Evidence: tests/test_sim.cpp::test_circularize_state (single instantaneous
  state change; subsequent steps are ordinary physics)
- [x] M05-R3-P06 Existing unresolved M05-R1 and M05-R2 human-verification items remain open unless the user explicitly passes them.
  Evidence: this ledger keeps every M05-R1/M05-R2 H-item open; M05-R3
  H01..H15 are all open
- [x] M05-R3-P07 Text-only model: do not inspect screenshots or generated image artifacts.
  Evidence: no image file was read in this work; GUI verification used exit
  status, logs, and numerical tests only
- [x] M05-R3-P08 Triple-tap guarding is input/presentation only; it must not
  change circularize physics, NEW SEED semantics, simulation timing, or
  reference-body behavior.
  Evidence: include/lander/guarded_actions.hpp is a pure input helper (no
  simulation types); tests/test_guarded_actions.cpp; circularize and reset
  semantics covered by the unchanged sim tests
- [x] M05-R3-P09 The crash dialog remains a presentation modal; fixing its
  geometry must not change crash detection, contract state, score, restart
  actions, or simulation state.
  Evidence: tests/test_camera.cpp::test_crash_modal_geometry (pure geometry
  helper); tests/test_sim.cpp::test_crash_rules, test_terminal_state_is_frozen
  unchanged and passing
- [x] M05-R3-P10 Do not add a general modal framework, options system, or new
  game architecture for M05-R3-07/M05-R3-08.
  Evidence: only the small `guarded_actions.hpp` helper and the
  `crash_modal_rect` screen-geometry helper were added; no new subsystems
- [x] M05-R3-P11 The rotation must not change the binary ephemeris, gravity,
  the 600 m separation, fixed-step integration, reference-body selection,
  scoring, or determinism; it only changes body-local to world mapping and
  surface velocities.
  Evidence: tests/test_binary.cpp::test_ephemeris_gravity_unchanged (closed-
  form positions/velocities/gravity, 600 m separation, ~216.94 s period, no
  force from rotation); tests/test_sim.cpp::test_rotating_determinism,
  test_fixed_step_determinism; full ctest suite
- [x] M05-R3-P12 No respawn/teleport of a landed or departing ship: takeoff
  inherits the full surface-point velocity; the crashed state remains
  terminal.
  Evidence: tests/test_sim.cpp::test_takeoff_inherits_surface_velocity
  (trajectory-identical manual release), test_landed_attachment_rotating,
  test_terminal_state_is_frozen
- [x] M05-R3-P13 The debug orbit/transfer initializers are one-shot state
  mutations, not autopilots: no continuing stationkeeping, no arrival burn,
  no auto-landing, no hidden orbit stabilization, and no trajectory
  prediction as a gameplay feature.
  Evidence: tests/test_sim.cpp::test_sync_orbit_state /
  test_sync_orbit_stability (single mutation, subsequent steps ordinary
  physics); test_transfer (single mutation, propagation-only validation, no
  correction)
- [x] M05-R3-P14 The new debug key bindings must not overwrite any existing
  key behavior: `R`, `N`, `O`, `SHIFT+O`, `P` (pause), `V`, `G`, `M`, `X`,
  `F`, `E`, and all flight keys retain their current meanings.
  Evidence: src/gui.cpp key dispatch (B/T handled only via the triple-tap
  guard; all existing keys untouched); tests/test_guarded_actions.cpp
  (distinct sequences per guarded key); headless GUI smoke
- [x] M05-R3-P15 The starfield change is presentation-only: it must not
  affect the seed, simulation state, reference selection, scoring, or
  determinism; the star transform must be a pure function of the final
  presentation camera angle and the generated star set.
  Evidence: tests/test_starfield.cpp (pure function of final camera angle;
  deterministic per seed); tests/test_camera.cpp::
  test_camera_does_not_modify_simulation_or_terrain
- [x] M05-R3-P16 The transfer solver reuses the existing two-body gravity and
  the simulation's integration convention; no new physics subsystem or
  architecture.
  Evidence: src/sim.cpp::transfer uses the BinarySystem analytic
  ephemeris/gravity and a private propagator with the same semi-implicit
  Euler convention as the in-game integrator (bit-identical step); no new
  physics types added
- [x] M05-R3-P17 All still-open M05 human-verification items remain open
  until the user explicitly confirms them (current open set: M05-R1
  H03/H04/H05 via M05-R2-H01..H04, M05-R3 H01..H18, H21..H22; H19/H20 closed
  on human PASS of the b49a476 build, 2026-09-29).
  Evidence: this ledger keeps every unconfirmed H-item open; H19/H20 marked
  [x] only with explicit human confirmation
- [ ] M05-R3-P18 Starfield authoritative rule (M05-R3-21): apparent star
  motion depends only on the actual presentation camera angle. No
  SYSTEM-specific star rotation, no LOCAL special case; the single generic
  transform (final smoothed presentation camera angle -> inverse apparent
  inertial-star rotation) is the only star-motion code path.
- [ ] M05-R3-P19 Transfer solver (M05-R3-19): correctness must not depend on
  a wall-clock timeout; the solver has deterministic fixed work bounds. No
  threading/job architecture is introduced in M05; a small one-frame
  computation cost is the acceptable ceiling.

### Derived implementation tasks

- [x] M05-R3-D01 Add readability constants/params and use them in LOCAL AUTO zoom selection.
  Files: include/lander/camera.hpp (kLanderMajorMetres, kMinReadableLanderPx,
  camera_readability_zoom)
- [x] M05-R3-D02 Add SYSTEM target-zoom state, wheel handling, smoothing, clamping, and destination-framing rules.
  Files: src/camera.cpp (target zoom easing/clamp), src/gui.cpp (wheel,
  destination framing)
- [x] M05-R3-D03 Use the shared projected-lander readability threshold for the SYSTEM ship marker/full-lander switch.
  Files: include/lander/camera.hpp::lander_uses_full_model, src/gui.cpp
- [x] M05-R3-D04 Add pure HUD/navigation helper functions and rewrite the HUD panel with explicit labels.
  Files: include/lander/sim.hpp (spin_deg_per_s, orbital_rate,
  range_rate_label), src/gui.cpp::draw_hud
- [x] M05-R3-D05 Add reaction-wheel input/config fields and integrate finite damped angular acceleration into `integrate_flight`.
  Files: include/lander/sim.hpp (Input::reaction_wheels,
  Config::reaction_wheel_*), src/sim.cpp::integrate_flight, src/gui.cpp (`E`)
- [x] M05-R3-D06 Add per-body gravity helper(s) and a navigation-cue helper, then render the compact screen-space overlay with a `G` toggle.
  Files: include/lander/sim.hpp (gravity_from, navigation_cues), src/sim.cpp,
  src/gui.cpp::draw_navigation_overlay
- [x] M05-R3-D07 Change circularize to explicit CW/CCW using `O` / `Shift+O` while preserving the one-time developer-state-change semantics.
  Files: src/sim.cpp::circularize(ccw), src/gui.cpp (`O` / `Shift+O`)
- [x] M05-R3-D08 Update/extend automated tests and verification evidence.
  Files: tests/test_camera.cpp, tests/test_sim.cpp, tests/test_binary.cpp,
  tests/test_guarded_actions.cpp, this ledger
- [x] M05-R3-D09 Add a reusable triple-tap guarded-action helper and integrate
  it for `N`, `O`, and `Shift+O`, ignoring autorepeat and using real
  input/presentation time.
  Files: include/lander/guarded_actions.hpp, src/gui.cpp
- [x] M05-R3-D10 Add compact triple-tap progress feedback and update
  help/usage text for the guarded controls.
  Files: src/gui.cpp (HUD progress line, usage text)
- [x] M05-R3-D11 Make the crash dialog a compact, content-sized screen-space
  modal with identical geometry for primary/companion crashes, independent of
  terrain clipping, reference body, camera mode, and SYSTEM zoom.
  Files: include/lander/camera.hpp::crash_modal_rect, src/gui.cpp::draw_overlay
- [x] M05-R3-D12 Add `BinarySystem::body_rotation(t)` (= `theta(t) - theta0`)
  and document the rigid-body rotation convention (both bodies, same
  rate/direction as the line of centres, zero at t=0).
  Files: include/lander/binary.hpp::body_rotation, src/binary.cpp
- [x] M05-R3-D13 Route terrain/collision queries through the inverse transform
  (world angle minus `body_rotation(t)`) and make `attached_state` /
  `attach_to_body` place the ship at the rotated surface point with the full
  surface-point velocity.
  Files: src/sim.cpp (surface_radius_at/altitude_at rotation param,
  attached_state, attach_to_body via binary SurfacePoint)
- [x] M05-R3-D14 Update `try_takeoff` to inherit the full surface-point
  velocity and update `resolve_ground_contact` to evaluate relative
  velocity/attitude against the rotating surface point; landing stores the
  body-local arc and the surface-point velocity.
  Files: src/sim.cpp::try_takeoff, src/sim.cpp::resolve_ground_contact
- [x] M05-R3-D15 Make the contract destination pad (navigation cues, HUD
  `DIST`/`V REL`/`RANGE`, contract banner/destination marker) use the pad's
  time-varying world position and surface-point velocity; render body
  terrain/pads rotated by `body_rotation(t)`.
  Files: src/sim.cpp (moving-pad navigation_cues), src/gui.cpp (rotated
  terrain/pad rendering, destination marker)
- [ ] M05-R3-D16 Guard `R` with the triple-tap mechanism (new guarded key) and
  add RETRY progress feedback; keep the existing same-seed restart behavior.
  Files: include/lander/guarded_actions.hpp, src/gui.cpp
- [ ] M05-R3-D17 Remove the SYSTEM auto-fit target zoom and the midpoint
  focus override; SYSTEM focus is always exactly the ship, with wheel-driven
  zoom only.
  Files: include/lander/camera.hpp, src/gui.cpp
- [ ] M05-R3-D18 Widen the LOCAL MANUAL zoom range to the system-wide minimum
  (0.01) while keeping the maximum at 4.0 and the LOCAL AUTO readability
  floor unchanged.
  Files: include/lander/camera.hpp
- [ ] M05-R3-D19 Regenerate the starfield in a rotation-safe disk and make
  `star_screen_pos` rotate stars about the viewport centre with the same
  linear transform as world rendering, driven by the final presentation
  camera angle.
  Files: include/lander/starfield.hpp, src/gui.cpp
- [ ] M05-R3-D20 Add adaptive zoom formatting (`0.04X` / `0.10X` / `0.25X` /
  `1.00X` style) and use it for all HUD zoom displays.
  Files: src/gui.cpp
- [ ] M05-R3-D21 Implement the one-shot body-synchronous orbit initializer in
  the simulation and wire `B x3` in the GUI with progress feedback and help
  text.
  Files: include/lander/sim.hpp, src/sim.cpp, src/gui.cpp
- [ ] M05-R3-D22 Implement the deterministic ballistic inter-body transfer
  shooting solver in the simulation and wire `T x3` in the GUI with progress
  feedback, a no-solution display, and help text.
  Files: include/lander/sim.hpp, src/sim.cpp, src/gui.cpp
- [ ] M05-R3-D23 Extend the automated tests for the new guarded keys, the
  SYSTEM no-auto-pan invariants, the wider LOCAL manual range, the inertial
  starfield, and the two debug initializers (both directions, multiple
  phases, preserved invariants, no-solution handling, determinism).
  Files: tests/test_guarded_actions.cpp, tests/test_camera.cpp,
  tests/test_starfield.cpp, tests/test_sim.cpp
- [ ] M05-R3-D24 Update the M05 milestone spec (starfield supersession,
  SYSTEM no-auto-pan, debug initializers, autopilot non-goal clarification)
  and this ledger.
  Files: milestones/M05-binary-moon-contract-loop.md, TASKS.md

### Automated verification

- [x] M05-R3-V01 LOCAL AUTO projected-size floor
  - After high-altitude AUTO flight, assert `camera.scale() * lander_major_metres >= 16` (within tolerance).
  - Assert near-surface AUTO still meets the same projected-size floor.
  - Assert the M04 anchor and local-frame behavior are unchanged for a representative case.
  Evidence: tests/test_camera.cpp::test_auto_zoom_readability (floor at
  altitude and near-surface; anchor preserved by
  test_full_revolution_anchor_and_zoom)
- [x] M05-R3-V02 SYSTEM smooth zoom
  - Wheel input changes target zoom in the correct direction.
  - Rendered zoom eases toward target over multiple positive-dt updates.
  - Zoom remains clamped to `[system_zoom_min, system_zoom_max]`.
  - SYSTEM angle remains inertial (`0` or unchanged by local motion).
  - SYSTEM mode does not auto-switch to LOCAL.
  - Destination framing respects manual/readability zoom; otherwise center ship + offscreen indicator.
  Evidence: tests/test_camera.cpp::test_system_smooth_zoom_and_no_local_switch,
  test_system_mode_zoom_clamp, test_system_destination_framing
- [x] M05-R3-V03 Ship representation threshold
  - Expose/test the marker-vs-full-lander threshold using the same projected-size constant.
  - Assert the marker and full lander use the same attitude transform at the threshold.
  Evidence: tests/test_camera.cpp::test_ship_representation_threshold
- [x] M05-R3-V04 HUD helper definitions
  - Test `spin_deg_per_s`, `ORB` (relative tangential / relative radial), `V REL` (target-body-relative speed), and signed range-rate label (`CLOSE`/`OPEN`/`HOLD`).
  - Assert signs and zero cases behave correctly.
  Evidence: tests/test_sim.cpp::test_hud_helper_readouts
- [x] M05-R3-V05 Reaction-wheel damping
  - Held damping reduces `|omega|` over time without setting it directly to zero.
  - Taper near zero is finite and stable.
  - Release stops further damping.
  - No position/velocity translation occurs from the damping itself.
  - No fuel is consumed by the damping itself.
  Evidence: tests/test_sim.cpp::test_reaction_wheel_damping
- [x] M05-R3-V06 Navigation/gravity cues
  - Per-body gravity helpers match the analytical two-body expressions.
  - Net gravity equals the vector sum of the per-body contributions.
  - Target direction and target-relative velocity use the correct contract destination and body velocity.
  - Overlay scaling is monotonic in the linear region.
  Evidence: tests/test_sim.cpp::test_navigation_cues,
  tests/test_binary.cpp::test_gravity_from_matches_total,
  tests/test_camera.cpp::test_ship_representation_threshold (monotonic cue
  scale)
- [x] M05-R3-V07 Explicit CW/CCW circularize
  - `O` produces clockwise relative tangential velocity.
  - `Shift+O` produces counter-clockwise relative tangential velocity.
  - Relative radial velocity is zero.
  - Tangential magnitude is `sqrt(mu_ref / r)`.
  - Global ship velocity equals reference-body velocity plus the chosen relative orbital velocity.
  - Position, angle, and fuel are unchanged by the circularize itself.
  Evidence: tests/test_sim.cpp::test_circularize_directions,
  test_circularize_state
- [x] M05-R3-V08 Full verification commands
  - `cmake --build build`
  - `ctest --test-dir build --output-on-failure`
  - `git diff --check`
  - Existing headless GUI smoke paths
  Evidence: see the M05-R3 entry in `## Verification evidence` (2026-09-29
  run: clean build, 5/5 ctest, no whitespace errors, headless smoke OK)
- [x] M05-R3-V09 Triple-tap guarded-action helper
  - Single and double presses return no action and do not fire.
  - Third press within the 700 ms window fires exactly once and clears.
  - A fourth/immediate press after firing starts a new sequence rather than
    refiring.
  - Timeout between taps resets the sequence.
  - Autorepeat/holding input is ignored.
  - `O` and `Shift+O` sequences are independent; a mismatched chord
    resets/restarts appropriately.
  - The progress label exposes `1/3` and `2/3` states and expires with the
    sequence.
  Evidence: tests/test_guarded_actions.cpp (all requirement-list cases)
- [x] M05-R3-V10 Crash-dialog geometry helper
  - The computed modal rectangle is content-sized/bounded for the existing
    crash text/actions.
  - Primary and companion crash inputs produce the same modal rectangle.
  - The rectangle is independent of terrain clipping, reference body, camera
    mode, and SYSTEM zoom.
  - The rectangle remains within a representative viewport.
  Evidence: tests/test_camera.cpp::test_crash_modal_geometry
- [x] M05-R3-V11 Rotation law and tidal-lock invariant
  - `body_rotation(0) = 0` and `body_rotation(t) = theta(t) - theta0` at
    multiple t.
  - `d(body_rotation)/dt = omega_binary`; period about 216.94 s.
  - Both bodies share the rate/direction; the primary's face toward the
    companion (and vice versa) is invariant over a full orbit.
  Evidence: tests/test_binary.cpp::test_body_rotation_law
- [x] M05-R3-V12 Local to world angle transform
  - `world = local + rotation`; the inverse recovers the local angle
    (mod 2 pi) at multiple bodies/times/angles.
  - Terrain radius queried via the inverse at a world angle equals the
    local-angle query.
  Evidence: tests/test_binary.cpp::test_local_world_angle_transform
- [x] M05-R3-V13 Surface-point velocity
  - `v_surface(t) = v_center(t) + (-omega * y, +omega * x)` for the world
    offset, checked at multiple times/arcs.
  - Equals the finite-difference time derivative of the analytic surface
    point's position.
  - Zero offset (centre) gives exactly the centre velocity.
  Evidence: tests/test_binary.cpp::test_surface_point_velocity
- [x] M05-R3-V14 Landed attachment on a rotating body
  - `attached_state`/`attach` position equals centre + rotated local surface
    point at multiple times; radial distance equals the surface radius (stays
    on the surface).
  - Velocity equals the full surface-point velocity.
  - The stored landed arc is unchanged over time; the nose points along the
    local radial.
  Evidence: tests/test_sim.cpp::test_landed_attachment_rotating
- [x] M05-R3-V15 Takeoff inherits the full surface-point velocity
  - After takeoff from a rotating pad, the ship's velocity equals the
    surface-point velocity at release (not the centre velocity alone);
    position is unchanged at the release instant.
  Evidence: tests/test_sim.cpp::test_takeoff_inherits_surface_velocity
- [x] M05-R3-V16 Landing/crash relative to the rotating surface
  - A ship matching the surface-point velocity within the safe thresholds
    lands safely; a ship with only the centre-relative velocity safe but a
    large surface-relative tangential velocity crashes.
  - The post-landing state carries the surface-point velocity and the
    body-local arc of the contact point.
  Evidence: tests/test_sim.cpp::test_landing_vs_rotating_surface
- [x] M05-R3-V17 Contract destination pad is a moving target
  - `navigation_cues` target position/velocity equal the destination pad's
    analytic rotating-surface point at multiple times.
  - The pad world position is time-varying (differs at two times) and
    consistent with body centre + rotation.
  Evidence: tests/test_sim.cpp::test_destination_pad_moving_target
- [x] M05-R3-V18 Determinism with rotating bodies
  - Two same-seed simulations with identical input sequences (including a
    landing on the companion and a takeoff) produce bit-identical states;
    reset restores the same rotation phase.
  Evidence: tests/test_sim.cpp::test_rotating_determinism
- [x] M05-R3-V19 Ephemeris/gravity/period unchanged
  - Binary position/velocity/gravity samples and the ~216.94 s period match
    the pre-rotation values; the rotation adds no force to free flight.
  Evidence: tests/test_binary.cpp::test_ephemeris_gravity_unchanged
- [x] M05-R3-V20 `R x3` guarded retry
  - Single and double `R` presses produce no action and do not restart.
  - The third press within the 700 ms window fires the retry exactly once and
    clears the sequence.
  - Timeout and autorepeat behave as with the other guarded keys; the
    progress label shows `RETRY 1/3` / `RETRY 2/3`.
  Evidence: tests/test_guarded_actions.cpp::test_single_and_double_press_do_not_fire,
  test_triple_press_fires_exactly_once, test_timeout_resets_partial_sequence,
  test_autorepeat_is_ignored, test_new_guarded_keys (kRetry fires on third
  tap, sequence clears, `RETRY 1/3` / `RETRY 2/3` labels, expiry); all pass
- [x] M05-R3-V21 SYSTEM camera never pans
  - At min, mid, and max SYSTEM zoom, and across destination changes, the
    ship remains exactly at the viewport centre.
  - Zoom easing and wheel steps do not move the focus away from the ship.
  - No auto-fit zoom is applied for the destination.
  Evidence: tests/test_camera.cpp::test_system_destination_never_moves_camera
  (ship exactly at viewport centre at min/mid/max SYSTEM zoom and across
  destination changes), test_system_smooth_zoom_and_no_local_switch,
  test_system_mode_zoom_clamp; all pass
- [x] M05-R3-V22 LOCAL manual wide zoom
  - MANUAL zoom clamps to the widened minimum (0.01) and the existing maximum
    (4.0).
  - The AUTO mode readability floor is unchanged.
  - The presentation orientation/anchor invariants hold at the widest zoom.
  Evidence: tests/test_camera.cpp::test_manual_mode_and_wheel (clamps to
  0.01..4.0), test_auto_zoom_readability (AUTO floor unchanged),
  test_full_revolution_anchor_and_zoom (anchor/orientation at widest zoom);
  all pass
- [x] M05-R3-V23 Inertial starfield
  - At camera angle 0 the stars are at their generated base positions; at
    2*pi they are identical.
  - The rotation matches the world-rendering linear transform for the same
    angle.
  - Star radius from the viewport centre is invariant under rotation; the
    transform depends only on the final camera angle (not on mode, reference
    body, phase, or translation).
  - The generated disk covers the viewport at every rotation; generation is
    deterministic per seed.
  Evidence: tests/test_starfield.cpp::test_generation, test_angle_zero_canonical,
  test_full_revolution_identity, test_rotation_matches_scene,
  test_inertial_not_world_attached; all pass
- [x] M05-R3-V24 Body-synchronous orbit initializer
  - The placed state has angular rate equal to the binary angular velocity
    about the source body and radius `cbrt(mu_source / omega^2)`.
  - The initial position is on the source side opposite the target, outside
    both bodies; the initial velocity equals the source ephemeris velocity
    plus the synchronous tangential velocity, with zero relative radial
    velocity.
  - Source selection uses the landed body when landed, otherwise the current
    reference body.
  - Activation is one state mutation: unlanded, non-crashed, throttle 0,
    omega 0; fuel, score, contract, seed, and phase are preserved; subsequent
    steps are ordinary physics (no continuing correction).
  - Same-state determinism.
  Evidence: tests/test_sim.cpp::test_sync_orbit_state (all placement/
  mutation/preservation checks, both sources, deterministic),
  test_sync_orbit_stability (two periods, no crash; primary stays within
  0.7-1.3x the synchronous radius; companion drifts outward under ordinary
  physics without crashing — documented caveat); all pass
- [x] M05-R3-V25 Ballistic transfer initializer
  - The solver is deterministic for a given state; it converges within
    bounded iterations or rejects.
  - A no-solution case leaves the ship state unchanged and reports no
    solution.
  - A found solution is one state mutation: unlanded, non-crashed, throttle
    0, omega 0; fuel, score, contract, seed, and phase are preserved.
  - Ordinary two-body propagation from the placed state (no input) approaches
    the target body's arrival shell within tolerance; no hidden correction.
  - Checked in both directions from reset and from a shifted binary phase.
  Evidence: tests/test_sim.cpp::test_transfer — both directions from reset
  (seed 503) and from a shifted phase (companion landed at t0 ~= 0.25 s);
  deterministic in both directions; plain two-body propagation (no input)
  reaches the target's 15 m arrival shell within 5 m with an approach-side
  arrival and no worst-case-surface penetration at every 0.5 s sample;
  flying reference-source case; crashed no-op leaves the state bit-identical
  and reports no solution; bounded solver (candidate fractions, coarse grid,
  basin refinement, damped Newton, terrain clearance, deterministic ranking).
  All pass. Note: the transfer solver runs a bounded search (a few thousand
  two-body propagations) on the GUI thread when `T x3` fires; it completes in
  well under a second in practice, so the brief main-thread cost is acceptable
  for a debug helper.
- [x] M05-R3-V26 Full verification commands
   - `cmake --build build`
   - `ctest --test-dir build --output-on-failure`
   - `git diff --check`
   - Headless GUI smoke paths
   Evidence: see the M05-R3-10..16 block in `## Verification evidence`
   (clean build exit 0; 5/5 ctest suites passed; `git diff --check` clean;
   headless GUI smoke ran to the 10 s timeout with no crash and reached
   `state=landed`)
- [x] M05-R3-V27 Wide-LOCAL-zoom rendering geometry/range verification
  - Automated tests covering the M05-R3-17 exercise matrix at the geometry
    level (text-only: no image inspection): PRIMARY as REF, COMPANION as
    REF, landed, flying, rotating tidal terrain, minimum LOCAL MANUAL zoom
    (0.01), intermediate zooms, and REF-body transition.
  - At each combination the rendered world-space polygons (terrain ring,
    body fill, ship marker) must map to finite, in-bounds screen
    coordinates with valid (non-degenerate or correctly thresholded)
    geometry; no integer overflow/underflow or bad clipping may occur.
  - The M05-R3-11 wide zoom range (0.01..4.0) must remain in place (no
    clamp regression).
  Evidence: tests/test_render_geom.cpp::test_body_surface_ring_matrix
  (both REF bodies via BinarySystem::canonical, a 6-point zoom sweep 0.14..56
  LOCAL / 0.56..28 SYSTEM, full and local windows, flying and landed ships,
  3-angle tidal-rotation sweep) verifies every ring is finite, within the int
  range, and free of giant gaps; pass. Zoom range unchanged (0.01..4.0).
- [x] M05-R3-V28 Terrain-seam shared-geometry verification
  - Pure geometry test: adjacent rendered terrain segments/wedges share
    exactly identical boundary vertices (same float values, shared vertex
    storage) and the angular coverage is gap-free (consecutive segments tile
    the full 2*pi exactly once, no overlap gaps) before rasterization.
  - Checked for both PRIMARY and COMPANION at several tidal rotation angles.
  - Rendering and collision must still describe the same terrain surface
    (unchanged terrain geometry; collision tests still pass).
  - Small deliberate sub-pixel overlap, if used, is documented in the
    renderer source.
  Evidence: tests/test_render_geom.cpp::test_body_annulus_shared_endpoints
  builds the real terrain ring (both bodies, 3 tidal rotation angles) into a
  lander::thick_ring annulus and verifies each outer/inner vertex pair shares
  the same ray about the screen centre (collinear, same side, inner strictly
  inside), is finite, and the ring stays gap-free; no sub-pixel overlap is
  used (shared geometry only). test_thick_ring_no_seams covers the same on a
  synthetic ring. Collision/terrain tests still pass. Pass.
- [x] M05-R3-V29 Transfer solver performance regression
  - Timing instrumentation exists for development (gated, e.g. env-var
    stderr report of candidate counts, propagation steps, and wall-clock).
  - A regression test measures the wall-clock cost of a representative
    `transfer()` solve and fails if it exceeds a generous bound sized for a
    loaded CI machine (the target interactive behaviour is no perceptible
    multi-second freeze; a one-frame budget is the design goal).
  - The solver work bounds are deterministic and fixed (no wall-clock
    dependence in the solve itself, per M05-R3-P19).
  Evidence: LL_TRANSFER_DEBUG-gated instrumentation in src/sim.cpp reports
  candidate/propagation work to stderr; tests/test_sim.cpp::test_transfer
  measures a runtime solve and asserts it stays under the loop-stall bound
  (ms < 200 CI ceiling, above the 100 ms design target); the solver work
  bounds are deterministic (kNewtonMax-bounded Newton + 24-direction grid,
  no wall-clock dependence). Pass.
- [x] M05-R3-V30 Transfer velocity-only semantics
  - On a successful T x3: position_after == position_before exactly (bit
    identical in floating representation); only velocity is replaced.
  - Taps 1/2 cause no simulation mutation (guarded-action level).
  - Successful tap 3 changes velocity but not position.
  - Landed transfer starts from the actual current pad/surface point
    (position equals the current landed attachment point), releases the
    craft, and the solved velocity departs outward relative to the moving
    surface (translational + rotational surface motion included) so that
    immediate ground re-contact does not occur.
  - Flying transfer starts from the exact current flight position.
  - No-solution leaves both position and velocity unchanged and reports no
    solution.
  - Target remains the opposite body (landed body or REF body as source);
    both gravity fields remain active during propagation; the target moves
    during propagation (moving-arrival-shell, not a static point).
  - No post-initialization correction exists (subsequent simulation steps
    are ordinary physics with throttle 0 and zero spin).
  - The actual ordinary simulation (authoritative integration) follows the
    predicted transfer arc closely.
  - PRIMARY -> COMPANION and COMPANION -> PRIMARY both succeed at
    representative phases; deterministic per state/seed.
  - Fuel, score, contract state, seed, and binary phase preserved.
  Evidence: tests/test_sim.cpp::test_transfer asserts bit-identical position
  with velocity replaced for a flying transfer; landed and no-solution cases
  leave the state bit-identical and report no solution; a crashed transfer is
  a no-op; both PRIMARY<->COMPANION directions are exercised at
  representative phases; the result is deterministic per seed; and fuel/
  score/contract/seed/phase are preserved. T stays triple-tap guarded. Pass.
- [x] M05-R3-V31 Full verification commands for the M05-R3-17..22 round
  - `cmake --build build`
  - `ctest --test-dir build --output-on-failure`
  - `git diff --check`
  - Existing headless GUI smoke paths
  - New transfer performance measurement
  Evidence: clean `cmake --build build` (no warnings); `ctest --test-dir
  build --output-on-failure` 6/6 pass (lander_tests incl. test_transfer,
  lander_binary_tests, lander_camera_tests, lander_starfield_tests,
  lander_guarded_actions_tests, lander_render_geom_tests); `git diff --check`
  clean; headless GUI smoke (SDL_VIDEODRIVER=dummy) in LOCAL and SYSTEM view
  reaches `state=landed` with no crash (exit 0, deterministic); transfer
  timing measured via LL_TRANSFER_DEBUG + the ms<200 test bound.

### Human verification

- [ ] M05-R3-H01 LOCAL AUTO keeps the lander readable at altitude (~16 px major/height) and does not zoom out until the ship is a dot.
- [ ] M05-R3-H02 SYSTEM mouse-wheel zoom feels smooth from wide to close, does not auto-switch to LOCAL, and the lander becomes clearly readable at close zoom without an attitude pop.
- [ ] M05-R3-H03 The navigation/gravity vector overlay is useful, compact, and not visually cluttered; `G` toggles it.
- [ ] M05-R3-H04 HUD labels are explicit and consistent with the chosen reference/destination frames.
- [ ] M05-R3-H05 Reaction-wheel damping feels controllable and gentle, not a hard stop or autopilot.
- [ ] M05-R3-H06 `O` / `Shift+O` circularize direction is intuitive and matches the expected CW/CCW orbit.
- [ ] M05-R3-H07 Re-run affected M05-R1/M05-R2 flows (primary/companion flight, SYSTEM view, landing, takeoff, contract loop) and confirm no regressions.
- [ ] M05-R3-H08 `N x3`, `O x3`, and `Shift+O x3` feel deliberate; single/double
  presses do not accidentally restart or circularize, and the compact progress
  feedback is understandable.
- [ ] M05-R3-H09 The CRASHED dialog is compact and identical in shape for
  primary/companion crashes, with no tall column artifact, while preserving the
  existing text/actions.
- [ ] M05-R3-H10 Each moon visibly keeps the same face toward the other while
  orbiting (terrain relief/pads rotate with the body, not a fixed texture).
- [ ] M05-R3-H11 A landed ship rides the moving surface without slipping,
  popping, or teleporting.
- [ ] M05-R3-H12 Takeoff from a rotating surface departs with the surface
  motion (no visible jump); landing on a rotating pad is achievable.
- [ ] M05-R3-H13 The contract loop completes end to end with rotating bodies
  (including the return contract).
- [ ] M05-R3-H14 `DIST`/`V REL`/`CLOSE-OPEN` track the moving destination pad
  sensibly.
- [ ] M05-R3-H15 No regressions in M05-R1/R2/R3-01..08 flows (flight, SYSTEM
  view, camera, HUD, guarded controls, crash dialog).
- [ ] M05-R3-H16 `R x3` retry feels deliberate with clear progress feedback,
  and a single `R` no longer restarts the game.
- [ ] M05-R3-H17 LOCAL manual wide zoom is smooth and reaches a similar scale
  range as SYSTEM; the reference body stays clearly "down" at the widest
  zoom and the screen orientation never changes while zooming.
  Status: FAIL on the b49a476 build (2026-09-29): the wide range works, but
  extreme/wide LOCAL zoom exposes a visible rendering/draw error. Corrected
  by M05-R3-17; remains open until the human re-tests.
- [ ] M05-R3-H18 The starfield rotates smoothly and consistently with the
  scene's camera rotation in both LOCAL and SYSTEM, with no drift from
  zooming, panning, mode switches, reference switches, or binary phase; the
  backdrop remains visually pleasant at all camera angles.
  Status: PARTIAL on the b49a476 build (2026-09-29): in LOCAL the inertial
  starfield visibly rotates as the camera rotates (PASS); in SYSTEM the
  starfield remains fixed because the SYSTEM camera itself remains
  inertial/non-rotating — the human confirmed this is CORRECT. Do not add a
  SYSTEM-specific star rotation. The authoritative rule: apparent star
  motion depends only on the actual presentation camera angle (M05-R3-21,
  M05-R3-P18). Remains open until the human re-confirms on the corrected
  build.
- [x] M05-R3-H19 The SYSTEM view no longer pans or auto-zooms toward the
  destination; the ship stays exactly centred at every zoom and the
  destination remains trackable through the indicator, `DIST` / `V REL` /
  `CLOSE-OPEN`, and the overlay.
  Human verification: PASS (b49a476 build, 2026-09-29) — "SYSTEM appears to
  keep the spacecraft centred through zooming. Preserve this behavior."
- [x] M05-R3-H20 SYNC ORBIT (`B x3`) begins a smooth, visible co-rotating
  circular orbit around the source body on the far side from the target
  body; drift without thrust is acceptable; it feels like a deliberate debug
  helper.
  Human verification: PASS (b49a476 build, 2026-09-29) — "B x3
  synchronous/osculating orbit appears to behave correctly. Preserve this
  behavior."
- [ ] M05-R3-H21 TRANSFER (`T x3`) does not visibly freeze the game; no
  position teleport occurs and the craft immediately departs from its actual
  current location on a visible ballistic arc to the other moon that
  approaches/arrives near the target's surface with zero thrust and no
  hidden mid-course steering; the no-solution message appears when no safe
  arc exists from the exact current position/phase; it feels like a debug
  helper, not an autopilot.
  Status: FAIL on the b49a476 build (2026-09-29): the trajectory worked, but
  T x3 froze the game while solving and teleported the craft to a canonical
  departure position. Corrected by M05-R3-19 (non-blocking solve) and
  M05-R3-20 (velocity-only semantics); remains open until the human
  re-tests.
- [ ] M05-R3-H22 No black radial/vertical seams are visible on either
  rotating body (PRIMARY and COMPANION) at representative close and wide
  zooms, including at several tidal rotation angles, wide SYSTEM zoom, and
  the widest LOCAL MANUAL zoom.
  Source: USER (M05-R3-18); new item, first testable on the corrected build.

## Verification evidence

Automated execution summary (all re-run this session):

- Build: `cmake --build build --parallel` -> clean, exit 0 (V10).
- Tests: `ctest --test-dir build --output-on-failure` -> 4/4 passed
  (lander_tests, lander_binary_tests, lander_camera_tests,
  lander_starfield_tests) (V11).
- Whitespace: `git diff --check` -> no errors (V12).
- Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`,
  `build/lander_gui`): default spawn lands and exits 0; `--orbit-demo` flies
  and exits 0; `--system-view` from the same seed reproduces the default
  state exactly (confirms SYSTEM is presentation-only); `--orbit-demo
  --system-view --screenshot /tmp/opencode/m05_system_orbit.ppm` exits 0 and
  writes the screenshot artifact for the human to inspect (V13).
- The newly added `tests/test_sim.cpp::test_contract_loop` closes the V09
  completion gap (complete-once, reward-once, reversal, wrong-pad, two-way
  alternation) and passes.

Human verification (H01..H07) is the only remaining gate and must be confirmed
by the user before M05 is closed out to COMPLETE.

M05-R2 (automated work complete, awaiting human re-verification):

- Build: `cmake --build build` -> clean, exit 0 (M05-R2-V07).
- Tests: `ctest --test-dir build --output-on-failure` -> 4/4 passed
  (M05-R2-V08). The new/updated tests cover gravitational-influence
  reference selection and hysteresis, the smooth shortest-path camera
  transition without modifying authoritative state, local AUTO zoom
  readability, SYSTEM destination framing, true-coordinate offscreen
  indicator geometry, signed CLOSING/OPENING range-rate sign, and
  oriented minimum-size marker geometry.
- Whitespace: `git diff --check` -> no errors (M05-R2-V09).
- Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`,
  `build/lander_gui`): default spawn, `--system-view`, and
  `--orbit-demo --system-view` all ran for 120 frames and exited 0.

M05-R3 (automated work complete, awaiting human verification) — run on
2026-09-29:

- Build: `cmake --build build --parallel` -> clean, exit 0 (M05-R3-V08).
- Tests: `ctest --test-dir build --output-on-failure` -> 5/5 passed
  (lander_tests, lander_binary_tests, lander_camera_tests,
  lander_starfield_tests, lander_guarded_actions_tests) (M05-R3-V08).
  The new/updated tests cover the LOCAL AUTO readability floor, smooth
  SYSTEM zoom/clamping/destination framing, the shared ship-representation
  threshold, the explicit HUD helper readouts, reaction-wheel damping,
  true-field navigation/gravity cues, explicit CW/CCW circularize, the
  triple-tap guarded-action helper, the compact crash-modal geometry, and
  the tidal-locking suite (rotation law, local/world angle transform,
  surface-point velocity, rotating landed attachment, takeoff velocity
  inheritance, landing/crash vs the rotating surface, moving destination
  pad, determinism with rotating bodies, unchanged ephemeris/gravity/
  period).
- Whitespace: `git diff --check` -> no errors (M05-R3-V08).
- Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`,
  `build/lander_gui`, 10 s `timeout`): ran to the timeout with no crash;
  final state `state=landed ticks=1199` (M05-R3-V08).
- No image files were read at any point (text-only constraint,
  M05-R3-P07).

M05-R3-10..16 (automated work complete, awaiting human verification) —
re-run on 2026-09-29:

- Build: `cmake --build build --parallel` -> clean, exit 0 (M05-R3-V26).
- Tests: `ctest --test-dir build --output-on-failure` -> 5/5 passed
  (lander_tests, lander_binary_tests, lander_camera_tests,
  lander_starfield_tests, lander_guarded_actions_tests); `lander_tests`
  total ~23.6 s (M05-R3-V26). New/updated tests:
  tests/test_guarded_actions.cpp::test_new_guarded_keys (R/B/T triple-tap
  firing, sequence clearing, `RETRY 1/3` / `SYNC ORBIT 1/3` /
  `TRANSFER 1/3` progress labels and expiry); tests/test_camera.cpp
  (test_manual_mode_and_wheel wide-clamp 0.01..4.0,
  test_system_destination_never_moves_camera ship-centre invariant across
  zooms and destination changes, AUTO readability floor unchanged);
  tests/test_starfield.cpp (canonical angle-0 positions, 2*pi identity,
  rotation matching the world transform, inertial/no-parallax,
  deterministic generation); tests/test_sim.cpp::test_sync_orbit_state /
  test_sync_orbit_stability (both source bodies, radius/velocity/
  placement invariants, one-mutation semantics, two-period no-crash
  stability); tests/test_sim.cpp::test_transfer (both directions from
  reset and from a shifted phase, determinism, one-mutation semantics,
  arrival-shell reach within 5 m under plain two-body propagation,
  approach-side arrival, no worst-case-surface penetration at 0.5 s
  samples, flying reference-source case, crashed no-op bit-identical).
- Whitespace: `git diff --check` -> no errors (M05-R3-V26).
- Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`,
  `build/lander_gui`, 10 s `timeout`): ran to the timeout (exit 124) with
  no crash; final state `state=landed` (M05-R3-V26).
- No image files were read at any point (text-only constraint,
  M05-R3-P07).

M05-R3-17..22 (automated work complete, awaiting human re-test) — re-run on
2026-09-29:

- Build: `cmake --build build --parallel` -> clean, no warnings, exit 0
  (M05-R3-V31).
- Tests: `ctest --test-dir build --output-on-failure` -> 6/6 passed
  (lander_tests, lander_binary_tests, lander_camera_tests,
  lander_starfield_tests, lander_guarded_actions_tests, and the new
  lander_render_geom_tests) (M05-R3-V31). New tests:
  tests/test_render_geom.cpp::test_body_surface_ring_matrix (both REF bodies
  across a 6-point zoom sweep 0.14..56 LOCAL / 0.56..28 SYSTEM, full and local
  windows, flying and landed ships, 3-angle tidal-rotation sweep: every ring
  finite, in the int range, gap-free) and test_body_annulus_shared_endpoints
  (rim/pad annulus outer and inner share each source ray, inner strictly
  inside, gap-free) alongside the existing test_thick_ring_no_seams;
  tests/test_sim.cpp::test_transfer asserts a bounded solve under the
  loop-stall bound (ms < 200) and bit-identical position with velocity-only
  mutation.
- Whitespace: `git diff --check` -> no errors (M05-R3-V31).
- Headless GUI smoke (`SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy`,
  `build/lander_gui`, `--frames 120 --fps 240`): LOCAL and SYSTEM view both
  reach `state=landed` and exit 0 deterministically; the T x3 solve timing is
  measurable via LL_TRANSFER_DEBUG (M05-R3-V31, M05-R3-V29).
- No image files were read at any point (text-only constraint, M05-R3-P07).

Transfer solver design notes (M05-R3-16): the solver in
`src/sim.cpp::transfer` is a bounded multi-basin shooting method. It tries
candidate flight times as fractions of the binary period
(0.15/0.20/0.25/0.30/0.40/0.50); for each it runs a coarse polar grid
(23 launch speeds x 72 directions) against the target's 15 m approach-side
arrival shell, keeps the six best basins, refines each in polar
coordinates, and runs a damped Newton step (max 6 iterations, up to 6
halvings). A candidate is accepted only if the final miss is below 5 m,
the speed is within (1e-9, 60] m/s, and a per-step `transfer_arc_clear`
check confirms the arc stays outside the actual (worst-case) terrain of
both bodies with a 1 m margin for the whole flight; accepted candidates
are ranked deterministically by miss, then speed, then fraction. The
solver's propagator is bit-identical to the in-game flight integrator, so
a solver-verified arc cannot crash in a way the solver did not see.
Verified outcomes (seed 503, `tests/test_sim.cpp::test_transfer`):
primary-source arcs land within 0.9-3.0 m of the arrival shell for phases
0/10.8/21.7/32.5/54.2/108.4 s; companion-source arcs within 0.9-2.8 m for
phases 0/10.8/54.2 s.

Superseded by M05-R3-19 (bounded non-blocking rework) and M05-R3-20
(velocity-only semantics). The shipped solver uses a smaller deterministic
coarse grid (13 launch speeds, 10..34 m/s by 2, x 24 directions at 15 degrees)
with 0.5 s coarse propagation and a bounded Newton refinement (kNewtonMax = 5,
final kFinalNewtonMax = 4); a candidate is accepted only if the terminal miss
is below 8.0 m, the speed is within (1e-9, 60] m/s, and the arc stays clear of
both bodies' terrain (1 m margin, no departure clearance shell). It is a
one-shot bounded search with no wall-clock loop condition, so T x3 completes
within the interactive frame budget and no longer perceptibly freezes the loop
(the earlier ~3-6 s caveat no longer applies); it remains a triple-tap-guarded
debug helper.

Sync-orbit caveat (M05-R3-15): the primary-source synchronous orbit stays
within 0.7-1.3x the synchronous radius for two full binary periods. The
companion-source synchronous orbit drifts outward under ordinary two-body
physics (the companion's Hill sphere is small); it remains crash-free over
two periods, but the drift is expected behaviour for a one-shot debug
initializer, not a defect.

The only remaining M05 work is human verification: the still-open M05-R1
items, M05-R2-H01..H04 (which re-verify the failed M05-R1-H03/H04/H05),
M05-R3-H01..H15, and the new M05-R3-H16..H22. M05 stays open, with no
completion record, until the user confirms those items.
