# M01 — Classic lunar terrain

## Goal

Replace the current flat ground plane with deterministic jagged lunar terrain
containing a small number of clearly defined flat landing sites.

At completion the existing game should look and play like a basic Lunar Lander
rather than a lander over an infinite flat floor.

## Starting state

M00 is a working SDL3 game with:

- lander physics
- thrust and rotation controls
- fuel
- HUD
- landing/crash handling
- restart/new seed
- one flat ground plane
- one landing pad

Preserve the working simulation and controls.

## Required behavior

### Terrain

Generate a one-dimensional lunar surface represented by connected terrain
segments or vertices.

The terrain must:

- be visibly jagged
- contain hills, valleys, and slopes
- be deterministic for a given seed
- span enough horizontal distance for meaningful flight
- contain several deliberately flat landing sites

Do not use a single flat baseline with decorative mountains behind it.
The rendered terrain must be the actual collision surface.

### Landing sites

Generate a small number of flat sections suitable for landing.

Landing sites must:

- be visually distinguishable
- be part of the terrain itself
- have enough width for the lander
- be deterministic for a given seed

Landing on non-flat terrain must not count as a successful landing.

### Collision

Ground contact must use the generated terrain height at the lander's
horizontal position rather than y = 0.

A landing succeeds only when:

- the lander contacts a designated landing site
- impact velocities satisfy the existing safety limits
- orientation satisfies the existing safety limit

Otherwise contact is a crash.

### Rendering

Render the actual generated terrain as the visible lunar surface.

Keep the current lander, HUD, star field, and controls unless a small change is
required by the new terrain.

## Constraints

- SDL3 only.
- Preserve the current fixed-timestep simulation.
- Preserve deterministic seeded behavior.
- Keep terrain implementation straightforward.
- Do not introduce an ECS or terrain framework.
- Do not implement a circular moon.
- Do not implement radial gravity.
- Do not implement orbital mechanics.
- Do not redesign unrelated systems.
- Do not attempt to visually inspect image files with the coding model; it is
  text-only.

## Non-goals

This milestone does NOT include:

- curved planetary geometry
- orbital flight
- 10-minute orbit tuning
- procedural craters
- terrain destruction
- camera zoom redesign
- difficulty modes
- economy or missions
- sound
- elaborate scoring changes

## Automated verification

Add tests covering at least:

- identical seed produces identical terrain
- different seeds produce different terrain
- every generated landing site is flat
- terrain height queries are deterministic
- safe contact on a landing site succeeds
- contact with sloped/non-pad terrain crashes
- existing physics tests continue to pass

Run:

    cmake --build build
    ctest --test-dir build --output-on-failure
    git diff --check

## Human verification

Launch:

    ./build/lander_gui

Confirm manually:

- terrain is visibly uneven
- several flat landing sites are visible/findable
- flying horizontally reveals additional terrain
- the lander collides with the terrain actually being drawn
- landing on a pad works
- hitting a slope crashes
- existing controls still feel correct

The coding model must not attempt to inspect screenshots itself.

## Acceptance criteria

M01 is complete only when:

- generated terrain replaces the flat plane
- collision follows that terrain
- landing sites are actual flat terrain sections
- seeded generation is deterministic
- automated tests pass
- the GUI launches successfully
- human visual/play verification is recorded

## Closeout

After acceptance:

1. update STATUS.md
2. write records/M01-classic-lunar-terrain.md
3. record implementation, tests, problems encountered, and verification
4. commit with a message beginning:

       M01: classic lunar terrain

Do not begin another milestone in the same task.
