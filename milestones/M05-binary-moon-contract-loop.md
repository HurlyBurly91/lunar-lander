# M05 — Binary moon and first contract loop

## Goal

Turn the M04 flight simulation into the first complete game loop while extending
the world from one moon to a compact two-body system.

M05 adds:

- a smaller companion moon moving in a real circular binary orbit with the
  primary moon
- gravitational influence from both bodies on the spacecraft
- body-relative collision, navigation, orbit, landing, and takeoff
- a system-scale camera view
- a minimal repeating contract loop between a base on the primary and a base on
  the companion

The main M04 moon is the canonical calibration body for the universe.

Do not choose the companion's gravity constants independently.

The companion and future bodies must be derived from the canonical rules below.

This milestone must remain a playable direct extension of M04.

Do not introduce ECS yet.

---

## Canonical universe model

The M04 primary moon defines the reference physical units of this fictional
universe.

Canonical primary values:

    reference radius R0:             332.384 m
    intrinsic surface gravity g0:    1.62 m/s^2
    gravitational parameter mu0:     178976.334 m^3/s^2
    nominal surface circular speed:  23.2048 m/s
    nominal surface circular period: 90.0 s

These values obey ordinary Newtonian two-body relationships:

    mu0 = g0 * R0^2

    v0 = sqrt(mu0 / R0)

    T0 = 2*pi*sqrt(R0^3 / mu0)

Equivalently:

    g0 = 4*pi^2*R0 / T0^2

The existing main moon is therefore not merely one arbitrary gameplay object.
It establishes the gravitational scale used to derive other bodies.

Use gravitational parameter `mu` directly in simulation. There is no need to
introduce an explicit universal G or simulated kilograms in M05.

---

## Derived-body scaling law

For the family of small solid bodies used by this game, define a dimensionless
body-radius scale:

    s = R / R0

Bodies in this family preserve the canonical intrinsic surface gravity:

    g_surface = g0

Therefore:

    R  = s * R0

    mu = s^2 * mu0

    v_surface_circular = sqrt(s) * v0

    T_surface_circular = sqrt(s) * T0

This is an explicit rule of the game universe.

It means radius and gravitational parameter are not independently tuned.

It also means that if interpreted using ordinary mass/density language:

    mass parameter scales as R^2
    mean density would scale approximately as 1/R

That unusual density scaling is acceptable for this fictional compressed
universe. The simulation's canonical quantity is `mu`, not material density.

"Surface gravity" in this rule means the body's own intrinsic gravitational
acceleration:

    mu / R^2

It does not mean that tidal acceleration from another nearby body magically
vanishes.

In a binary system, the total effective acceleration near a surface may differ
slightly because other gravitational fields are physically present.

Do not compensate for that by changing the body's `mu`.

---

## Companion derivation

The companion must have a nominal reference low circular-orbit period of:

    T1 = 30 s

The primary reference period is:

    T0 = 90 s

From:

    T1 / T0 = sqrt(s)

derive:

    s = (T1 / T0)^2
      = (30 / 90)^2
      = 1/9

Therefore the companion constants are derived, not hand-selected:

    radius R1:
        R0 / 9
        = 36.9315556 m

    intrinsic surface gravity:
        g1 = g0
        ~= 1.62 m/s^2

    gravitational parameter:
        mu1 = mu0 / 81
        ~= 2209.58437037 m^3/s^2

    nominal reference circular speed:
        v1 = v0 / 3
        ~= 7.73493 m/s

    nominal reference circular period:
        T1 = T0 / 3
        ~= 30.0 s

As with the primary, actual terrain lies above the reference radius.

A practical terrain-clearing orbit around the companion will therefore have a
period somewhat greater than 30 seconds.

Do not distort gravity merely to force every usable orbit to exactly 30.000 s.

---

## No fake distance scale

World metres remain physical simulation metres.

Do not create:

- a different visual distance scale
- a transfer-distance multiplier
- patched "fast travel" coordinates
- hidden velocity scaling between bodies

The system is compressed by putting the bodies genuinely close together in the
same inertial coordinate system.

The spacecraft, both body centres, terrain, velocities, and gravity all use
the same metre/second units.

---

## Binary-system geometry

Use a fixed circular binary separation for M05:

    D = 600.0 m

This is centre-to-centre separation.

Reference surface-to-surface gap:

    gap = D - R0 - R1
        ~= 230.684444 m

This is deliberately compressed for gameplay.

It should make a transfer between moons take tens of seconds rather than many
minutes while still requiring interception and velocity matching.

Do not reduce D merely because a poorly planned intercept takes longer.

Transfer time is supposed to depend on:

- launch phase
- ship velocity
- companion orbital velocity
- thrust
- trajectory

---

## Binary orbital mechanics

The two moons orbit their common barycentre.

Do not hold the primary perfectly fixed while pretending only the companion
moves.

For M05 the body-body orbit may be an analytic prescribed circular ephemeris.
Do not numerically integrate mutual moon-moon gravity.

The prescribed orbit must nevertheless obey ordinary two-body mechanics.

Total gravitational parameter:

    mu_system = mu0 + mu1

Angular rate:

    omega = sqrt(mu_system / D^3)

Binary period:

    T_binary = 2*pi/omega
             = 2*pi*sqrt(D^3 / mu_system)
             ~= 216.94244 s

Because:

    mu1 / mu0 = 1/81

the barycentric orbital radii are:

    a_primary =
        D * mu1 / (mu0 + mu1)
        = D / 82
        ~= 7.317073 m

    a_companion =
        D * mu0 / (mu0 + mu1)
        = 81*D / 82
        ~= 592.682927 m

Corresponding circular speeds are approximately:

    primary barycentric speed:
        0.21192 m/s

    companion barycentric speed:
        17.16555 m/s

At simulation time t:

    theta = theta0 + omega*t

A valid deterministic convention is:

    primary_position =
        -a_primary * (cos(theta), sin(theta))

    companion_position =
        +a_companion * (cos(theta), sin(theta))

and:

    primary_velocity =
        +a_primary*omega * (sin(theta), -cos(theta))

    companion_velocity =
        +a_companion*omega * (-sin(theta), cos(theta))

Equivalent sign/orientation conventions are acceptable if documented and
internally consistent.

Use authoritative fixed-step simulation time for the ephemeris.

Do not use render time to drive body positions.

Pause must pause the physical ephemeris.

Reset with the same seed must restore the same binary phase.

---

## Body rotation

Do not add axial rotation in M05.

Each moon translates around the barycentre but does not spin about its own
centre.

Therefore a point fixed on a body's surface has the body's translational
velocity in M05.

Tidal locking, body spin, rotational surface velocity, and day/night cycles are
future work.

---

## Spacecraft gravity

The spacecraft exists in the same global inertial frame as both bodies.

Its gravitational acceleration is the superposition of both gravitational
fields:

    a =
        -mu0 * (r_ship - r_primary)
             / |r_ship - r_primary|^3

        -mu1 * (r_ship - r_companion)
             / |r_ship - r_companion|^3

plus spacecraft thrust.

Both gravitational fields are always active.

Do not implement:

- sphere-of-influence physics switching
- nearest-body-only gravity
- patched conics
- hidden capture forces
- orbit stabilization

"Current body", "reference body", and "target body" may exist for presentation
or gameplay logic, but not as switches that disable physical gravity.

---

## Body-relative state

All local navigation and surface mechanics must be computed relative to the
relevant moving body.

For body i:

    relative_position =
        ship_position - body_position

    relative_velocity =
        ship_velocity - body_velocity

Then derive from `relative_position`:

- radial distance
- local outward direction
- local tangent
- longitude / terrain coordinate
- surface radius
- altitude

and from `relative_velocity`:

- radial velocity
- tangential velocity

This is mandatory.

Never use raw global ship velocity for landing-speed tests on a moving body.

Example:

If the companion is moving at approximately 17 m/s globally and the spacecraft
beside it is also moving at approximately 17 m/s in the same direction, the
local landing velocity can be near zero.

If the spacecraft is nearly stationary globally while the companion sweeps
into it at approximately 17 m/s, that is a high-relative-speed collision.

---

## Terrain coordinate systems

Each body owns terrain in its own body-local coordinates.

The primary terrain must preserve the accepted M04 terrain for the same primary
seed.

Refactoring terrain to support multiple body centres must not silently change
the primary's generated surface.

The companion gets its own deterministic wrapped terrain seed.

Natural terrain geometry should be scaled sensibly relative to body radius
rather than copying metre-scale mountains from the primary unchanged.

At minimum:

- terrain is deterministic
- terrain wraps seamlessly
- drawing and collision use the same local surface
- body translation does not regenerate terrain
- terrain does not deform as the body or camera moves
- each body has at least one valid landing site

Artificial landing-pad dimensions do not have to obey strict 1/9 planetary
scaling.

Pads must remain large enough for the existing lander to use.

Do not shrink a landing pad below practical spacecraft dimensions merely to
satisfy geometric similarity.

---

## Collision with moving bodies

Collision must be checked against both body surfaces.

For each body:

1. subtract the body's centre from ship position
2. compute body-local longitude
3. query that body's terrain radius
4. compare relative radial distance to surface radius

On contact, evaluate:

- whether contact is on a landing pad
- body-relative radial velocity
- body-relative tangential velocity
- local radial landing attitude

Do not use global `vx/vy` landing thresholds.

Do not use global angle zero as upright.

---

## Landed attachment

M04 treats landing as a terminal frozen simulation state.

M05 must change that because landing on a moving body cannot freeze the universe.

A safely landed spacecraft is attached to a specific body and landing location.

Persist enough state to identify at least:

- landed body
- landed surface arc / longitude or equivalent local surface coordinate

While landed:

- system time continues
- both moons continue their binary orbit
- the ship moves with the landed body's translational motion
- the ship remains attached to the same body-local surface location
- global ship velocity follows the body's translational velocity
- the ship remains locally upright
- no hidden orbital integration acts on the attached ship

Crashed state may remain terminal for M05.

---

## Takeoff

The game loop must permit departure from a landed pad.

Do not respawn the ship into space merely because a new contract begins.

While landed, determine whether commanded outward thrust is sufficient to leave
the surface.

At minimum, takeoff must:

- begin from the actual moving body's position
- inherit the body's current global translational velocity
- begin from the current pad
- transition cleanly from attached/landed to free flight
- use ordinary spacecraft thrust and gravity immediately after release

A low throttle that cannot overcome local effective downward acceleration should
not create a free-floating numerical jitter state.

Implement the smallest robust ground-support/takeoff rule needed for this
milestone.

Do not create a general wheel/contact dynamics system.

---

## Circularize developer control

Preserve the M04 `O` developer control, but generalize it to moving bodies.

`O` must circularize relative to the currently selected/reference body.

For body i:

    r_rel = ship_position - body_position

    v_body = body_velocity

    v_circular = sqrt(mu_i / |r_rel|)

Set:

- relative radial velocity to zero
- relative tangential speed to `v_circular`
- preserve meaningful existing relative tangential direction
- default consistently when relative tangential velocity is essentially zero

Then:

    ship_global_velocity =
        body_velocity + desired_relative_orbital_velocity

Do not forget the body's global velocity.

`O` remains a one-time developer state change.

No continuing stabilization is permitted.

Preserve `F` refuel.

---

## Companion local orbital behavior

The companion's canonical isolated/reference low-orbit period is approximately
30 seconds because it is a 1/9-scale body under the universe scaling law.

Once embedded in the compact binary, the primary's gravity creates real tidal
perturbations.

Do not cancel those perturbations.

Therefore:

- the 30-second value is the companion's canonical reference scale
- an actual terrain-clearing orbit will be longer
- an actual orbit in the binary may be perturbed
- low companion orbit should nevertheless be practically usable

At D = 600 m the companion's approximate Hill scale is large enough relative to
its ~36.93 m reference radius to permit useful very-low local orbital behavior.

Do not add an artificial sphere-of-influence boundary.

---

## Camera modes

Preserve the accepted M04 local player-follow camera.

Near a body, local camera behavior remains:

    screen up ~= that body's local radial outward
    screen right ~= local tangent

The exact player anchor from M04 remains required in local flight modes.

Add a system-scale view for inter-body navigation.

### SYSTEM view

SYSTEM view is a presentation tool only.

Requirements:

- inertial orientation rather than local radial-up rotation
- spacecraft near the centre of the viewport
- sufficiently wide scale to understand the ship's location relative to both
  moons
- both moon positions use their true simulation coordinates
- no fake compression in rendering
- stars remain the accepted fixed screen-space backdrop
- include a minimum-size ship marker if the correctly scaled lander would be
  difficult to see

Use an initial system zoom on the order of:

    0.04X

with the existing `base_scale = 14 px/m`, giving approximately:

    0.56 px/m

At D = 600 m this is enough to show the compact binary geometry on a
1280x720 viewport from most useful transfer positions.

The exact SYSTEM zoom may be tuned slightly during human verification.

Do not modify world geometry to make it fit the camera.

Preserve the existing local AUTO/MANUAL camera behavior.

Use a separate explicit SYSTEM-view control rather than overloading physical
reference-body selection with camera behavior.

The exact key may be chosen to fit the current control layout and must be shown
in the HUD/help.

---

## Reference body versus physics

A `reference body` may be used for:

- local HUD altitude
- local radial/tangential velocity display
- local camera orientation
- developer circularize
- landing information

It must not control which gravitational fields are active.

Choose the reference body deterministically.

Near a surface, it should correspond to the locally relevant body.

During an inter-body transfer, SYSTEM view should make dependence on an
arbitrary reference body unimportant.

Avoid rapid reference-body flicker near the crossover region; use a simple
deterministic hysteresis rule if necessary.

---

## HUD

Preserve the M04 HUD where practical.

Add compact information needed for two-body flight.

At minimum show:

- reference body
- contract destination body/base
- distance to target
- relative velocity useful for interception

The exact presentation may use compact labels.

Do not turn M05 into a flight-computer UI project.

Do not add trajectory prediction or maneuver-node planning yet.

---

## First contract loop

M05 must produce the first repeating gameplay loop.

Initial loop:

    start landed at PRIMARY BASE
        ->
    receive contract to COMPANION BASE
        ->
    take off
        ->
    intercept the moving companion
        ->
    match useful relative velocity
        ->
    land safely on the companion target pad
        ->
    contract completes
        ->
    score/reward is granted
        ->
    next contract targets PRIMARY BASE
        ->
    player can take off and return

With only two bases, contracts may simply alternate between them.

This is intentionally minimal.

The purpose is to prove:

- launch
- inter-body navigation
- moving-target interception
- body-relative landing
- reward
- next job

Do not add a procedural mission generator.

---

## Contract state

Add the smallest explicit contract state needed.

A contract should identify at least:

- origin body/base
- destination body/base
- completion state
- reward/score value

A contract completes only when the spacecraft safely lands on the designated
destination pad.

Landing safely somewhere else is not contract completion.

After completion, assign the next contract from the current base to the other
base.

Use the existing score concept for M05.

Do not add:

- currency economy
- shops
- upgrades
- inventory
- cargo simulation
- reputation
- progression trees

Those belong to later milestones.

---

## Determinism

For a given game seed and identical inputs:

- primary terrain is identical
- companion terrain is identical
- binary initial phase is identical
- body ephemerides are identical
- contract sequence is identical
- spacecraft simulation is identical

The analytic binary ephemeris must be driven from authoritative fixed-step
simulation time.

Render interpolation may interpolate presentation states, but must not change
authoritative physics.

---

## Render interpolation

Preserve the M04 fixed-step/render interpolation architecture.

The moving moons now also require coherent presentation.

Camera, spacecraft, and moon positions shown in a frame must represent the same
interpolated presentation time.

Do not render the spacecraft at one interpolated time and the companion at a
different authoritative tick.

Physics and collision continue using authoritative fixed-step states only.

---

## Automated verification

Add tests covering at least:

### Canonical universe laws

- primary constants still satisfy approximately:

      mu0 = g0 * R0^2

- primary nominal period remains approximately 90 s
- companion scale is exactly 1/9
- companion intrinsic surface gravity matches the primary
- companion `mu` is `mu0 / 81`
- companion nominal reference circular period is approximately 30 s
- companion nominal reference circular speed is approximately `v0 / 3`

### Binary ephemeris

- body separation remains 600 m
- barycentre remains fixed at the origin within numerical tolerance
- primary barycentric radius is approximately 7.317073 m
- companion barycentric radius is approximately 592.682927 m
- binary period is approximately 216.94244 s
- analytic velocities agree with position derivatives
- resetting the same seed/time reproduces body states
- advancing one full binary period returns both bodies close to their starting
  positions and velocities

### Multi-body gravity

- ship acceleration equals the vector sum of both bodies' gravity
- neither body gravity is silently disabled when the other is nearer
- acceleration transforms correctly as binary phase changes
- no hidden orbit/capture stabilization exists

### Relative kinematics

- relative position subtracts body position
- relative velocity subtracts body velocity
- radial/tangential decomposition uses relative velocity
- a ship co-moving with a body has approximately zero local velocity
- a globally stationary ship beside the moving companion has large relative
  tangential velocity

### Terrain and collision

- primary M04 terrain regression remains unchanged for the same seed
- companion terrain is deterministic and seamless
- collision works on both moving bodies
- drawing/collision query the same body-local surface
- safe landing works on both bodies
- landing evaluation uses relative rather than global velocity

### Landed attachment and takeoff

- landed ship remains attached to the same local surface location as its body
  moves
- landed ship inherits body global translational velocity
- system time/body ephemeris continues while landed
- valid takeoff transitions to free flight without teleporting
- takeoff begins with the body's current global velocity
- insufficient outward thrust does not create surface jitter

### Circularize

- `O` around the primary uses primary-relative velocity plus primary global
  velocity
- `O` around the companion uses companion-relative velocity plus companion
  global velocity
- relative radial velocity becomes approximately zero
- relative tangential velocity equals `sqrt(mu_i/r)`
- no continuing stabilization force exists

### Camera/presentation

- M04 exact local player anchor remains intact
- SYSTEM view does not change simulation state
- SYSTEM view uses true body positions
- system camera can place both bodies in a useful view at the initial 600 m
  separation
- starfield remains fixed in screen space
- moving-body render interpolation is smooth at representative render rates

### Contracts

- initial contract targets companion base
- landing on wrong/non-target pad does not complete contract
- safe landing on target completes it exactly once
- reward/score is applied exactly once
- next contract reverses destination
- contract state is deterministic

Run:

    cmake --build build
    ctest --test-dir build --output-on-failure
    git diff --check

---

## Runtime / human verification

Human verification must include:

- primary moon still feels like the accepted M04 body
- companion visibly moves around the primary
- SYSTEM view makes the spatial relationship understandable
- the player can leave the primary and intercept the moving companion
- the companion does not behave like a stationary target with a moving sprite
- matching companion velocity matters during approach
- landing evaluation feels relative to the companion rather than to global
  coordinates
- local camera behavior remains usable near both bodies
- companion gravity feels like the same surface-gravity universe at a much
  smaller scale
- a very-low companion orbit behaves on roughly the intended ~30-second local
  scale, with real binary perturbations allowed
- landing on companion completes the contract
- the next contract points back to the primary
- takeoff from the moving companion inherits its motion naturally
- returning toward the primary is possible
- no visible coordinate-frame teleport occurs when changing reference body or
  camera view
- stars remain fixed in screen space
- M04 flame/camera/orbit presentation remains smooth

The coding model is text-only and must not inspect screenshots.

---

## Non-goals

Do not implement in M05:

- ECS conversion
- modular spacecraft
- multiple ship types
- alternative thrusters
- alternative fuels
- landing balloons
- solar sails
- cargo simulation
- economy
- shop/upgrades
- inventory
- procedural contract generation
- more than the primary and one companion
- numerical N-body integration of moon-moon motion
- body axial rotation
- tidal locking
- atmosphere
- aerodynamics
- time warp
- docking
- orbital autopilot
- trajectory prediction
- maneuver nodes
- patched conics
- sphere-of-influence physics switching
- hidden orbit stabilization
- fake distance compression

Those are later milestones.

---

## Acceptance criteria

M05 is complete when:

- the primary remains the canonical M04 body
- companion physical constants are derived from the canonical scaling law
- companion has a nominal ~30-second reference low-orbit scale
- both moons move in the defined 600 m circular barycentric binary
- the ship feels gravity from both bodies simultaneously
- local motion and landing use body-relative position and velocity
- the player can land on and take off from either moving body
- SYSTEM view makes inter-body position understandable
- a contract from primary to companion can be completed
- a subsequent return contract is generated
- primary terrain remains compatible with M04
- automated tests pass
- human inter-body flight/landing verification passes

---

## Closeout

After acceptance:

1. update `STATUS.md`
2. write:

       records/M05-binary-moon-contract-loop.md

3. preserve:
   - canonical universe equations
   - final body constants
   - final binary separation
   - measured binary period/speeds
   - relative-coordinate conventions
   - companion-orbit measurements
   - contract-loop behavior
   - automated verification evidence
   - human verification results
4. archive the bounded M05 TASKS state into the record according to AGENTS.md
5. commit with a message beginning:

       M05: binary moon contract loop

6. allow the post-commit hook to push main to origin
7. verify origin/main
8. do not begin M06 in the same task
