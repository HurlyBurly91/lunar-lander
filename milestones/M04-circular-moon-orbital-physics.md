# M04 — Circular moon and orbital physics

## Goal

Replace the flat-world geometry with a small circular moon and real 2-D radial
gravity so sustained orbital flight becomes possible.

This is the first curved-moon milestone.

The game should retain the existing Lunar Lander controls, terrain feel,
throttle, camera modes, and HUD where practical while changing the underlying
world from a flat plane to a closed circular body.

## Design target

Use a deliberately tiny fictional moon for fast mobile-game traversal.

Reference values:

    surface gravity:              1.62 m/s^2
    reference moon radius:       332.384 m
    reference circumference:     2088.43 m
    gravitational parameter mu:  178976.334 m^3/s^2
    reference circular speed:    23.205 m/s
    reference circular period:   90 s

The 90-second period is the nominal near-surface orbital scale.

Because real terrain rises above the reference radius, a safely terrain-clearing
orbit will naturally have a somewhat longer period. Do not distort the physics
merely to force every possible low orbit to exactly 90.000 seconds.

The purpose is a world where:

- ordinary landing flights take seconds
- fast suborbital transfers can cross large portions of the moon quickly
- a complete low circular orbit is on the order of 90 seconds
- orbital mechanics become useful gameplay rather than a long simulation wait

## Coordinate system

Move the simulation to global 2-D Cartesian coordinates.

The moon centre is:

    (0, 0)

The lander state position is a point in this global inertial frame:

    position = (x, y)

Velocity is also global/inertial:

    velocity = (vx, vy)

At the initial spawn, place the lander near the "north pole" so existing visual
orientation remains intuitive:

    local outward ~= +Y

The exact spawn longitude is not important as long as reset is deterministic
and starts safely above a landing region.

## Gravity

Replace constant downward gravity with inverse-square radial gravity.

For position vector:

    r = (x, y)
    distance = |r|

gravity acceleration is:

    a = -mu * r / |r|^3

with:

    mu ~= 178976.334 m^3/s^2

At the reference moon radius this must produce approximately:

    1.62 m/s^2

Gravity must point toward the moon centre everywhere.

Do not use a constant-magnitude radial approximation.

## Lander attitude and thrust

Keep lander attitude in the global inertial frame.

The existing convention should remain usable:

    angle 0 -> thrust axis points +Y

Main-engine thrust acts along the lander's body thrust axis exactly as before,
but now in global coordinates.

Do not automatically rotate the physical lander merely because it moves around
the moon.

A free-flying lander's inertial attitude should remain inertial unless the
player commands rotation.

Rotation controls and throttle behavior from M03 remain unchanged.

## Local vertical and landing attitude

"Upright" is no longer global angle 0 except near the starting longitude.

For landing checks, compute the local outward radial direction at the contact
point.

A safe landing requires the lander's thrust/up axis to align with the local
surface outward direction within the existing safe-angle tolerance.

Do not test landing attitude against global angle zero.

## Circular terrain

Wrap the deterministic M01 terrain around the moon.

Represent the surface as:

    surface_radius(theta) =
        reference_radius + terrain_height(theta)

or an equivalent arc-length representation.

Use wrapped arc distance around the reference circumference:

    s in [0, circumference)

Terrain must be:

- deterministic from seed
- continuous around the entire moon
- seamless at the wrap boundary
- visibly uneven
- the same geometry used by rendering and collision
- fixed in moon/world coordinates

There must be no discontinuity where longitude wraps from the end of the
circumference back to zero.

Do not regenerate terrain based on camera position.

## Landing sites

Preserve multiple deterministic landing sites.

Landing sites become short surface arcs around the moon.

They must:

- be deterministic from seed
- be clearly visible
- be approximately flat/constant-radius landing regions
- participate in the real collision surface
- support the existing landing/crash rules
- wrap safely near the longitude seam

Landing-site width should continue to be expressed in metres of surface arc,
not arbitrary angular units.

## Collision

At lander position:

1. compute radial distance
2. compute longitude / wrapped terrain coordinate
3. query terrain surface radius there
4. detect contact when lander radial distance reaches the surface

Collision and drawing must query the same terrain representation.

On contact:

- safe pad + safe velocity + safe local attitude -> landed
- otherwise -> crashed

Landing velocity checks should use local components:

    radial velocity
    tangential velocity

rather than raw global vx/vy.

Preserve the existing gameplay thresholds initially unless curved-world testing
shows a concrete reason to tune them.

## Camera

Adapt the existing M02/M03 camera to the curved moon.

Near the lander, the screen should behave like a useful local frame:

    screen up    ~= local radial outward
    screen right ~= local tangent direction

This allows landing gameplay to remain visually intuitive anywhere around the
moon.

The camera may rotate as the lander travels around the body.

Preserve:

- AUTO overview / landing zoom
- accepted overview zoom 0.40
- accepted landing zoom 1.40
- 18/25 m AUTO hysteresis
- MANUAL mouse-wheel zoom
- M camera-mode toggle
- zoom anchoring / lander framing fixes from M03

AUTO altitude is now:

    radial_distance - local_surface_radius

not global y.

Camera transformations must not modify terrain geometry.

## Starfield

Stars remain effectively infinitely distant.

They must still have:

- no translational parallax from local lander/camera movement
- no zoom parallax

If the camera rotates to maintain local radial-up orientation, the starfield
should respond only to camera rotation, as an inertial distant background would.

Do not reintroduce the old camera-position parallax behavior.

## HUD

Preserve existing HUD information.

Altitude must remain local terrain-relative altitude.

Velocity display should become useful in the curved frame.

Prefer showing local components:

    VEL TANGENTIAL  RADIAL

or similarly compact labels rather than raw global vx/vy.

Do not redesign the entire HUD in this milestone.

Throttle and fuel readouts remain unchanged.

## Orbital behavior

The implementation must support genuine unpowered circular or near-circular
orbits.

At the reference radius, the analytic circular-orbit speed is approximately:

    23.205 m/s

and the analytic period is approximately:

    90 s

For a safe orbit above terrain, calculate the correct circular velocity from:

    v = sqrt(mu / r)

and expected period from:

    T = 2*pi*sqrt(r^3 / mu)

Do not fake orbital motion, constrain the lander to a circle, or add hidden
stabilization.

A circular orbit should emerge from the same gravity/integration system used
for all other flight.

## Numerical integration

Retain the existing fixed timestep unless testing demonstrates it is inadequate.

The current 120 Hz simulation is expected to be sufficient for this scale.

Add orbital regression tests that detect unacceptable long-term drift.

Do not switch integrators merely for architectural elegance.

If the existing semi-implicit Euler method proves measurably inadequate over a
few orbits, make the smallest justified numerical change and document it.

## Automated verification

Add tests covering at least:

- surface gravity magnitude is approximately 1.62 m/s^2 at reference radius
- gravity points exactly toward the moon centre at several positions
- gravity follows inverse-square scaling
- terrain wraps seamlessly at the circumference boundary
- same seed produces identical circular terrain
- landing sites remain deterministic
- collision uses radial surface height
- safe landing succeeds at more than one moon longitude
- landing attitude is evaluated against local radial outward, not global up
- local radial/tangential velocity decomposition is correct
- nominal circular-orbit speed calculation matches approximately 23.205 m/s
- nominal reference period is approximately 90 s
- a terrain-clearing circular orbit remains bounded over at least several
  complete revolutions
- orbit direction works clockwise and counter-clockwise
- existing throttle behavior remains correct
- existing camera tests remain correct where applicable

Run:

    cmake --build build
    ctest --test-dir build --output-on-failure
    git diff --check

## Runtime / human verification

Launch:

    ./build/lander_gui

Verify manually:

- the world visibly curves
- the player can travel far enough horizontally to see the terrain bend away
- continuing in one direction eventually returns to the original region
- there is no visible terrain seam
- landing works at different longitudes around the moon
- local "up" remains visually sensible around the moon
- camera zoom and framing remain stable
- terrain does not wobble while the camera rotates/translates/zooms
- starfield does not translate with the lander
- throttle still feels correct
- a sufficiently fast tangential trajectory can enter sustained orbit
- orbiting the body feels roughly on the intended ~90-second gameplay scale
- deorbiting and landing uses the same physics rather than a mode switch

The coding model is text-only and must not inspect screenshots itself.

## Helpful test support

It is acceptable to add a developer-only command-line option that initializes
the lander in a calculated circular orbit for verification, for example:

    ./build/lander_gui --orbit-demo

if this materially simplifies orbital testing.

Such a mode must:

- use the real simulation
- calculate orbital velocity from mu and radius
- not change normal gameplay behavior
- remain clearly developer/test functionality

Do not add a gameplay "orbit autopilot" in M04.

## Non-goals

Do not implement:

- contracts or missions
- economy
- upgrades
- cargo
- multiple spacecraft
- touch/mobile controls
- Game Center
- achievements
- ads or monetization
- atmospheric drag
- aerodynamics
- planet rotation
- N-body gravity
- other moons/planets
- orbital autopilot
- docking
- time warp

Those are later product milestones.

## Acceptance criteria

M04 is complete when:

- the moon is a closed circular world
- terrain wraps seamlessly
- gravity is radial and inverse-square
- landing works using local radial/tangential physics
- the camera remains usable around the full moon
- the starfield behaves as an inertial distant background
- real free-flight circular orbits are possible
- the nominal orbital scale is about 90 seconds
- automated orbital regression tests pass
- existing throttle/camera/terrain functionality is preserved
- human curved-world and orbit verification passes

## Closeout

After acceptance:

1. update STATUS.md
2. write records/M04-circular-moon-orbital-physics.md
3. record constants, coordinate conventions, test results, numerical drift,
   camera behavior, and human verification
4. commit with a message beginning:

       M04: circular moon orbital physics

5. push main to origin

Do not begin M05 in the same task.
