# Active Task

Milestone: M05
Request: M05-R1
State: AWAITING HUMAN VERIFICATION

## Bootstrap note

M04 (circular moon and orbital physics) is COMPLETE and closed out in
records/M04-circular-moon-orbital-physics.md (commit 160059c, pushed to
origin/main). This bounded ledger begins fresh with the M05-R1 request
group: the binary moon and the first contract loop, per the M05 milestone
specification (milestones/M05-binary-moon-contract-loop.md).

All automated-verification work for M05-R1 (V01..V14 and the derived
implementation tasks D01..D06) is complete; see the per-item evidence below
and the "Verification evidence" summary. The milestone is now awaiting human
verification: every H-item (H01..H07) remains open and is not closed until the
user confirms each one. No M05 completion record is written and M05 is not set
to COMPLETE until that human acceptance happens.

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

- [x] M05-R1-16 Reference body: may drive local HUD altitude, radial/
  tangential display, local camera orientation, developer circularize, and
  landing information, but must not control which gravitational fields are
  active. Chosen deterministically; near a surface it is the locally relevant
  body; avoid rapid flicker near the crossover region using a simple
  deterministic hysteresis rule.
  Source: USER (spec, "Reference body versus physics")
  Evidence: tests/test_sim.cpp::test_reset_state; ::test_landing_rules (reference_body() tracks the landed/target body with 0.8x hysteresis in src/sim.cpp::update_reference_body)

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
