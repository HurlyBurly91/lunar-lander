# M03 — Throttle and camera polish

## Goal

Polish the current Lunar Lander controls and camera behavior before beginning
the curved-moon work.

This milestone addresses three issues found during human M02 verification:

1. the main engine is binary on/off and needs a controllable throttle,
2. rapid manual zoom can cause the camera to lose its framing of the lander,
3. the starfield moves incorrectly with camera translation and zoom.

Also use the new throttle to perform a meaningful human hysteresis check around
the existing AUTO camera thresholds.

Do not begin curved-moon or orbital work.

## Starting state

M02 provides:

- AUTO contextual camera
- overview zoom 0.40
- landing zoom 1.40
- AUTO hysteresis:
  - enter landing view below 18 m
  - return to overview above 25 m
- MANUAL camera mode
- mouse-wheel zoom
- terrain fixed rigidly in world space
- unified camera transform
- deterministic terrain and collision
- working SDL3 game

Human verification found:

- overview zoom 0.40 feels appropriate
- the transition around 25 m feels natural
- terrain remains visually stable under rapid zoom
- manual rapid wheel scrolling can cause the lander to drift away from its
  intended framing
- the starfield motion looks wrong because it responds strongly to camera
  translation
- camera hysteresis is difficult to test manually with an on/off main engine

Treat the existing AUTO values as accepted defaults.

## Part 1 — Main-engine throttle

Replace the binary main-engine command with a continuous throttle.

Represent main-engine throttle as a normalized value:

    0.0 = engine off
    1.0 = full thrust

The simulation must scale both:

- main-engine acceleration
- main-engine fuel consumption

linearly with throttle.

For throttle value `t`:

    acceleration = t * configured main acceleration
    fuel burn    = t * configured main fuel burn

Clamp throttle to [0, 1].

Rotation controls and rotation fuel behavior remain unchanged.

### Player controls

Use persistent throttle control:

    Up / W      increase throttle
    Down / S    decrease throttle
    X           immediate throttle cutoff

Throttle should change continuously while increase/decrease is held rather than
jumping through a tiny number of fixed presets.

Choose a reasonable ramp rate that allows both quick response and fine control.
Keep the rate as an explicit tunable presentation/control constant.

The throttle setting persists when the player releases the key.

Reset/retry/new-seed should restore throttle to zero.

Do not couple throttle state to camera mode.

### HUD

Add throttle indication to the existing HUD.

For example:

    THR  37%

Keep the existing fuel display.

The engine flame should reflect throttle magnitude rather than merely being
present/absent.

At very low nonzero throttle it may be short; at full throttle it should be at
its current/full visual extent.

### Simulation tests

Update/add tests covering at least:

- throttle 0 produces no main-engine acceleration or main-engine fuel burn
- throttle 1 preserves the existing full-thrust behavior
- throttle 0.5 produces approximately half the main-engine acceleration
- throttle 0.5 burns approximately half the main-engine fuel
- throttle is clamped to [0, 1]
- rotation behavior remains unchanged
- deterministic fixed-timestep behavior remains intact

Do not weaken existing tests merely to accommodate the new input model.

## Part 2 — Manual zoom must preserve lander framing

Human testing found that rapid mouse-wheel zoom can cause the camera to lose
track of the lander even though terrain rendering remains geometrically stable.

Fix the camera behavior so aggressive manual zoom does not make the lander
wander substantially away from its intended screen anchor.

The lander should remain approximately at the configured framing position
while zoom changes rapidly.

### Required behavior

When camera scale changes:

- zoom must be anchored to the followed lander/target
- changing zoom must not create a large transient screen-space displacement of
  the lander
- rapid repeated wheel events must not accumulate framing error
- normal camera-follow smoothing may remain for world translation
- terrain must remain rigid and unchanged in world space

A useful implementation model is:

- preserve the lander's current/intended screen-space anchor while changing
  scale
- immediately compensate camera centre for the scale change
- then continue normal follow smoothing

Do not solve this by disabling follow smoothing globally.

AUTO mode must retain the same principle: zoom changes should not visibly lose
the lander.

### Existing AUTO tuning

Preserve unless a genuine regression requires otherwise:

    overview zoom: 0.40
    landing zoom:  1.40
    AUTO -> landing: altitude < 18 m
    landing -> AUTO overview: altitude > 25 m

Do not retune these simply because this milestone touches Camera.

## Part 3 — Starfield behavior

The current starfield incorrectly applies camera parallax:

    star.x - cam.x * scale * 0.25
    star.y - cam.y * scale * 0.25

This makes stars move substantially during a few hundred metres of lunar
translation and also couples star position to zoom.

For the current game, treat the stars as effectively infinitely distant.

### Required behavior

The starfield must:

- remain deterministic for a given seed
- remain fixed in screen/celestial background space during camera translation
- not shift merely because camera zoom changes
- not wrap in response to lander/camera movement
- remain visually behind all world geometry

Camera x/y translation and camera scale must not affect individual star screen
positions.

Keep the implementation simple. Do not add a 3-D skybox, celestial mechanics,
or a new star-coordinate system.

Future curved-moon work may revisit sky orientation if required. That is not
part of M03.

## Part 4 — Hysteresis human verification

The new throttle should make it possible to hover or climb/descend slowly near
the AUTO camera transition band.

Human verification should explicitly test:

- approach 18 m slowly from above
- verify AUTO transitions toward landing view
- remain between roughly 18 and 25 m
- verify AUTO does not repeatedly switch states
- slowly climb past 25 m
- verify AUTO returns toward overview
- descend again and verify the behavior is repeatable

The coding model must not claim this human visual/control check has passed
without user confirmation.

## Scope constraints

Preserve:

- M01 terrain generation
- terrain collision
- fixed world-space terrain sampling
- M02 unified camera transform
- AUTO overview zoom 0.40
- AUTO landing zoom 1.40
- current 18/25 m hysteresis thresholds
- MANUAL mode and wheel zoom
- SDL3
- deterministic simulation behavior

Do not implement:

- circular moon
- radial gravity
- orbital mechanics
- planet rotation
- terrain-generation redesign
- missions
- economy
- sound
- minimap
- free camera
- new rendering framework

## Automated verification

Run:

    cmake --build build
    ctest --test-dir build --output-on-failure
    git diff --check

Add focused tests where practical for:

- throttle physics
- throttle fuel scaling
- throttle limits
- zoom-anchor/framing behavior
- star positions being independent of camera x/y and scale

Existing M00/M01/M02 tests must continue to pass.

## Runtime verification

Launch:

    ./build/lander_gui

Verify:

### Throttle

- throttle can be smoothly increased and decreased
- throttle persists when keys are released
- low throttle gives visibly weaker thrust
- X cuts throttle immediately
- reset starts at zero throttle
- fine descent/hover control is substantially easier than with binary thrust

### Camera

- AUTO behavior still feels as it did in M02
- overview 0.40 remains useful
- transition around 25 m still feels natural
- rapid MANUAL wheel scrolling does not lose the lander
- repeated zoom-in/zoom-out keeps the lander near its intended screen anchor
- terrain does not wobble or deform

### Starfield

- climb several hundred metres
- descend again
- move horizontally
- zoom rapidly in MANUAL mode
- stars remain visually fixed rather than sliding with camera translation or
  zoom

### Hysteresis

Perform the slow 18–25 m test described above.

The coding model is text-only and must not inspect screenshots itself.

## Acceptance criteria

M03 is complete when:

- main engine uses a continuous persistent throttle
- acceleration and fuel burn scale correctly with throttle
- HUD displays throttle
- rapid manual zoom preserves lander framing
- AUTO behavior remains stable
- starfield is independent of camera translation and zoom
- all automated tests pass
- GUI launches successfully
- human verification confirms throttle feel, zoom framing, starfield behavior,
  and AUTO hysteresis

## Closeout

After acceptance:

1. update STATUS.md
2. write records/M03-throttle-camera-polish.md
3. document implementation, constants, tests, and unresolved issues
4. commit with a message beginning:

       M03: throttle and camera polish

5. push main to origin

Do not begin M04 or curved-moon work in the same task.
