# M02 — Contextual camera and manual zoom

## Goal

Add a camera system that provides both:

1. an automatic Lunar-Lander-style contextual view that shows more terrain
   during navigation and magnifies the landing area near the surface, and
2. a manual camera mode whose zoom is controlled with the mouse wheel.

This milestone changes camera presentation only.

Do not change terrain generation, collision, lander physics, gravity, or
orbital behavior.

## Starting state

M01 provides:

- deterministic jagged lunar terrain
- actual terrain collision
- several flat landing sites
- working SDL3 controls
- HUD
- fixed-timestep simulation
- a camera that follows the lander
- fixed world-space terrain rendering with no camera-induced terrain wobble

Preserve all of this behavior.

## Camera modes

The game has two camera modes:

### AUTO

Default mode.

Automatically choose an appropriate view based primarily on the lander's
altitude above the local terrain.

At higher altitude:

- zoom out
- show substantially more horizontal terrain
- make it possible to survey multiple landing sites
- keep the lander approximately centered

Near the surface:

- zoom in to a landing-scale view
- show the lander and nearby terrain with enough detail for final corrections

Use altitude above:

    terrain.height_at(lander.x)

not absolute world y.

The automatic camera must use hysteresis so it cannot rapidly switch modes
when hovering near a threshold.

The transition between overview and landing scales should be smooth rather
than an instantaneous visual snap.

Keep this simple. A two-state AUTO camera with interpolation is sufficient.

Suggested initial behavior:

    overview -> landing view:
        altitude < ~25 m

    landing -> overview:
        altitude > ~35 m

These values are gameplay tuning parameters, not physics constants, and may be
adjusted during implementation if the resulting view is clearly better.

### MANUAL

Manual mode disables automatic zoom selection.

The camera still follows the lander's world position, but its scale is chosen
by the player.

Mouse wheel:

    wheel up   -> zoom in
    wheel down -> zoom out

Manual zoom must:

- be continuous or use sufficiently small increments
- be clamped to a useful minimum and maximum
- remain stable when no wheel input occurs
- never affect simulation state

## Mode toggle

Add a keyboard control to toggle between AUTO and MANUAL camera modes.

Use:

    M

Behavior:

- game starts in AUTO
- pressing M switches to MANUAL
- pressing M again switches back to AUTO
- when entering MANUAL, begin from the camera's current zoom so there is no
  visible jump
- when returning to AUTO, smoothly converge toward the automatic target zoom

Do not reset the lander or alter simulation state when switching modes.

## HUD

Add a small camera indicator to the existing HUD.

Examples:

    CAM AUTO
    CAM MANUAL

In MANUAL mode also show the current zoom or scale in a compact form, for
example:

    CAM MANUAL 1.4X

Do not redesign the HUD.

Update the controls help to mention:

    M CAMERA MODE
    WHEEL ZOOM

## Camera transform

There must be one consistent world-to-screen camera transform used by:

- lander
- terrain
- landing pads
- guide markers
- crash debris
- other world-space geometry

Camera movement or zoom must never modify terrain geometry.

The M01 fix is an invariant:

- terrain sampling remains anchored in fixed world coordinates
- moving or zooming the camera must not make the terrain change shape
- rendering and collision continue using the same terrain model

Keep world coordinates and camera calculations in floating point until SDL
requires raster coordinates.

## Scope

This milestone may modify:

- camera state and update behavior
- world-to-screen transformation
- input handling for camera mode / mouse wheel
- HUD camera status
- tests specifically needed for camera behavior

Avoid unrelated refactoring.

## Non-goals

Do NOT implement:

- circular moon geometry
- radial gravity
- orbital mechanics
- 10-minute orbit tuning
- terrain-generation changes
- new landing physics
- mission systems
- economy
- sound
- minimap
- free-camera panning
- mouse-drag camera movement

Those belong to later work if requested.

## Automated verification

Add focused tests where practical for camera logic separated from rendering.

Verify at least:

- AUTO is the initial camera mode
- AUTO selects landing-scale zoom below the lower threshold
- AUTO selects overview-scale zoom above the upper threshold
- hysteresis preserves the existing state between thresholds
- camera zoom transitions smoothly toward its target
- toggling to MANUAL preserves the current zoom
- mouse-wheel input changes zoom only in MANUAL
- manual zoom respects min/max clamps
- toggling back to AUTO resumes automatic zoom behavior
- camera operations never modify simulation or terrain state
- all existing M00/M01 tests remain passing

Run:

    cmake --build build
    ctest --test-dir build --output-on-failure
    git diff --check

## Runtime verification

Launch:

    ./build/lander_gui

Human verification:

### AUTO

- climb high enough to obtain a useful wide terrain overview
- multiple terrain features / landing sites can be surveyed
- descend and verify the view smoothly magnifies near the surface
- hover around the transition region and verify the camera does not flicker
  rapidly between scales
- climb again and verify it smoothly returns to overview

### MANUAL

- press M and confirm AUTO -> MANUAL
- use the mouse wheel to zoom in and out
- stop scrolling and verify zoom remains fixed
- fly while manually zoomed and verify the camera continues following
- press M and confirm MANUAL -> AUTO
- confirm the camera smoothly returns to the appropriate automatic scale

### Regression

- terrain must not wobble, swim, stretch, or change contour while translating
  or zooming
- landing/crash behavior remains unchanged

The coding model is text-only and must not attempt to visually inspect
screenshots. It may generate artifacts or numerical rendering checks for human
inspection.

## Acceptance criteria

M02 is complete when:

- AUTO contextual zoom works
- MANUAL mouse-wheel zoom works
- M toggles reliably between modes
- AUTO uses local terrain-relative altitude
- hysteresis prevents threshold chatter
- transitions are smooth
- HUD shows camera mode
- terrain remains visually rigid in world space
- existing tests pass
- human play verification passes

## Closeout

After acceptance:

1. update STATUS.md
2. write records/M02-contextual-camera.md
3. document implementation, tuning values, tests, and human verification
4. commit with a message beginning:

       M02: contextual camera

5. push main to origin

Do not begin another milestone in the same task.
