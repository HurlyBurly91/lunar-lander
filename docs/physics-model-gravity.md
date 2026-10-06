# Physics Model — Gravity

This document is the canonical cross-milestone reference for the gravity and
orbital-physics rules established by M04/M05.

Milestones may introduce new bodies or configurations, but these rules remain
authoritative unless an explicit requirement changes them.

## Canonical universe model

The M04 primary moon defines the reference physical units of this fictional
universe.

Canonical primary values:

    reference radius R0:             332.384 m
    intrinsic surface gravity g0:    1.62 m/s^2
    gravitational parameter mu0:     178976.334 m^3/s^2
    nominal surface circular speed:  23.2048 m/s
    nominal surface circular period: 90.0 s

These obey:

    mu0 = g0 * R0^2

    v0 = sqrt(mu0 / R0)

    T0 = 2*pi*sqrt(R0^3 / mu0)

Equivalently:

    g0 = 4*pi^2*R0 / T0^2

The primary establishes the gravitational scale used to derive other bodies.

Use gravitational parameter `mu` directly in simulation. There is no need to
introduce an explicit universal G or simulated kilograms.

## Derived-body scaling law

For the family of small solid bodies used by this game:

    s = R / R0

Bodies preserve the canonical intrinsic surface gravity:

    g_surface = g0

Therefore:

    R = s * R0

    mu = s^2 * mu0

    v_surface_circular = sqrt(s) * v0

    T_surface_circular = sqrt(s) * T0

Radius and gravitational parameter are not independently tuned.

If interpreted using ordinary mass/density language:

    mass parameter scales as R^2
    mean density scales approximately as 1/R

The simulation's canonical quantity is `mu`, not material density.

"Surface gravity" means the body's own intrinsic acceleration:

    mu / R^2

Nearby gravitational fields remain physically present. Do not compensate for
them by changing the body's `mu`.

## M05 companion derivation

The M05 companion has nominal reference low circular-orbit period:

    T1 = 30 s

with:

    T0 = 90 s

From:

    T1 / T0 = sqrt(s)

derive:

    s = (30 / 90)^2
      = 1/9

Therefore:

    R1 = R0 / 9
       = 36.9315556 m

    g1 = g0
       ~= 1.62 m/s^2

    mu1 = mu0 / 81
        ~= 2209.58437037 m^3/s^2

    v1 = v0 / 3
       ~= 7.73493 m/s

    T1 = T0 / 3
       ~= 30.0 s

Actual terrain lies above the reference radius, so a terrain-clearing orbit
may have a period greater than 30 seconds.

Do not distort gravity to force every usable orbit to exactly 30 seconds.

## Physical distance scale

World metres remain physical simulation metres.

Do not create:

- a different visual distance scale
- a transfer-distance multiplier
- patched fast-travel coordinates
- hidden velocity scaling between bodies

Bodies, spacecraft, terrain, velocities, distances, and gravity share the same
physical coordinate system and metre/second units.

## M05 binary geometry

The M05 binary uses centre-to-centre separation:

    D = 600.0 m

Reference surface gap:

    gap = D - R0 - R1
        ~= 230.684444 m

The compression is physical: the bodies genuinely occupy these positions in
the same inertial coordinate system.

## Circular binary mechanics

The two moons orbit their common barycentre.

Do not hold the primary fixed while pretending only the companion moves.

The body-body orbit may use an analytic prescribed circular ephemeris rather
than numerically integrating mutual moon-moon gravity, but it must obey
ordinary two-body mechanics.

Total gravitational parameter:

    mu_system = mu0 + mu1

Angular rate:

    omega = sqrt(mu_system / D^3)

Binary period:

    T_binary = 2*pi/omega
             = 2*pi*sqrt(D^3 / mu_system)
             ~= 216.94244 s

For the M05 mass-parameter ratio:

    mu1 / mu0 = 1/81

the barycentric radii are:

    a_primary =
        D * mu1 / (mu0 + mu1)
        = D / 82
        ~= 7.317073 m

    a_companion =
        D * mu0 / (mu0 + mu1)
        = 81*D / 82
        ~= 592.682927 m

Corresponding circular speeds are approximately:

    primary:   0.21192 m/s
    companion: 17.16555 m/s

At simulation time `t`:

    theta = theta0 + omega*t

A valid deterministic convention is:

    primary_position =
        -a_primary * (cos(theta), sin(theta))

    companion_position =
        +a_companion * (cos(theta), sin(theta))

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

## Tidal locking

Both M05 moons are tidally locked to each other.

Each spins about its own centre with the same angular rate and direction as the
binary line of centres:

    body_rotation(t) = theta(t) - theta0

The rotation is zero at `t = 0`.

For a terrain feature:

    world_angle =
        body_local_angle + body_rotation(t)

Terrain/collision queries recover body-local angle by subtracting
`body_rotation(t)`.

A point fixed on the surface has velocity:

    v_surface =
        v_body_center + omega_spin x r_local_world

In 2D:

    omega x (x, y) =
        (-omega*y, +omega*x)

A landed spacecraft rides the rotating surface.

Takeoff inherits the full surface-point velocity.

Landing contact and landing safety use spacecraft velocity relative to the
surface-point velocity.

The destination pad is likewise a moving surface point.

Tidal rotation does not change gravity, binary separation, binary ephemeris,
or reference-body selection.

## Spacecraft gravity

The spacecraft exists in the same global inertial frame as both bodies.

Its gravitational acceleration is:

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

"Current body", "reference body", and "target body" may be used for
presentation or gameplay logic, but never to disable physical gravity.

## Body-relative state

All local navigation and surface mechanics are relative to the relevant moving
body.

For body `i`:

    relative_position =
        ship_position - body_position

    relative_velocity =
        ship_velocity - body_velocity

Derive from relative position:

- radial distance
- local outward direction
- local tangent
- longitude / terrain coordinate
- surface radius
- altitude

Derive from relative velocity:

- radial velocity
- tangential velocity

Never use raw global spacecraft velocity for landing-speed tests on a moving
body.

For rotating-surface contact, use velocity relative to the actual surface
point, as defined above.

## Local circular-orbit quantities

The usual local relation remains:

    v_circular = sqrt(mu / r)

In the binary system this describes a local/instantaneous body-relative
circular condition.

The other body's gravitational field remains active and may perturb the orbit.

Do not add hidden stabilization to preserve an ideal circular orbit.

## M06-R13 three-body hierarchical system (canonical extension)

M06-R13 extends the M05 binary into a fixed three-body hierarchical (Jacobi)
system by adding body 2 = OUTER MOONLET. For the three-body system this
section supersedes the two-body statements in "M05 binary geometry",
"Circular binary mechanics", "Tidal locking", and "Spacecraft gravity"; the
M04 primary constants and the derived-body scaling law are unchanged.

Bodies:

    body 0 = PRIMARY:        R0 = 332.384 m,  mu0 = 178976.334 m^3/s^2
    body 1 = COMPANION:      R1 = R0/9,       mu1 = mu0/81        (M05)
    body 2 = OUTER MOONLET:  R2 = R1,         mu2 = mu1          (M06-R13)
                             (same scale as the companion)

Each terrain is a distinct pure function of the primary seed (the primary
seed, the M05 companion seed, and a new moonlet seed derived by a separate
salt).

Hierarchy — all analytic, no N-body integration of the bodies.

Inner pair (bodies 0 and 1): the unchanged M05 relative motion about the
inner-pair barycentre `B01`:

    mu_inner        = mu0 + mu1
    omega_inner     = sqrt(mu_inner / D01^3)
    theta_inner(t)  = omega_inner * t          (theta = 0 at t = 0)
    D01             = 600.0 m                  (unchanged)
    a0_inner        = D01 * mu1 / mu_inner     ~= 7.317 m
    a1_inner        = D01 * mu0 / mu_inner     ~= 592.683 m

    P0 = B01 - a0_inner * (cos(theta_inner), sin(theta_inner))
    P1 = B01 + a1_inner * (cos(theta_inner), sin(theta_inner))

`omega_inner` equals the M05 binary angular rate, so the relative motion of
(0,1) is bit-for-bit the M05 two-body solution (T_inner ~= 216.94244 s).

Outer pair (inner-pair barycentre `B01` and body 2): a fixed circle about the
total barycentre, which stays at the origin:

    mu_outer_system = mu_inner + mu2
    omega_outer     = sqrt(mu_outer_system / D_OUTER^3)
    theta_outer(t)  = pi/2 + omega_outer * t   (starts at pi/2)
    D_OUTER         = 1200.0 m
    a_inner_outer   = D_OUTER * mu2 / mu_outer_system      ~= 14.40 m
    a2_outer        = D_OUTER * mu_inner / mu_outer_system ~= 1185.60 m

    P2  = +a2_outer * (cos(theta_outer), sin(theta_outer))
    B01 = -a_inner_outer * (cos(theta_outer), sin(theta_outer))

Invariants, for all t:

    |P2 - B01| = D_OUTER
    mu0*P0 + mu1*P1 + mu2*P2 = 0        (total barycentre at the origin)

    T_outer = 2*pi/omega_outer ~= 610.13 s

Derivatives of a circular component `r = A*(cos(theta), sin(theta))`:

    v = A * omega * (-sin(theta), cos(theta))
    a = -omega^2 * r

Bodies 0 and 1 each carry both the inner and the outer component (their
centre translates with `B01` and orbits about it); body 2 carries only the
outer component.

Tidal locking is per-body:

    body_rotation(0, t) = body_rotation(1, t) = omega_inner * t
    body_rotation(2, t) = omega_outer * t

The "Tidal locking" surface-point velocity, contact, and landing rules apply
per body with that body's own spin rate.

Spacecraft gravity — all three fields always active, in the one global
inertial frame:

    a = sum over i = 0..2 of
          -mu_i * (r_ship - P_i) / |r_ship - P_i|^3
      + thrust

Do not implement sphere-of-influence switching, nearest-body-only gravity,
patched conics, hidden capture forces, or orbit stabilization (as before).
"Reference body" / "target body" stay presentation or gameplay logic; they
never disable a gravitational field.

The reference body generalizes to three candidates with the same rule as
M05: dominance by `mu_i / distance^2` with the same 1.2 hysteresis margin.

Contract / transfer scope: the repeating contract loop remains exactly
0 <-> 1; body 2 is never a contract origin or destination. One-shot developer
routes from a body-2 source (transfer / sync orbit) fail safely rather than
inventing a three-body transfer algorithm.
