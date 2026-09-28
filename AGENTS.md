# Lunar Lander

Build this incrementally. Do not invent a large architecture, milestone system,
economy, mining system, mission framework, ECS, ADR process, or other future
complexity unless explicitly requested.

## Immediate goal

Start from the existing Lunar Lander code in this repository and make it a
playable graphical game.

### Stage 1 — playable flat Lunar Lander

Get the existing simulation working through a simple GUI.

Required:
- SDL window
- render the lander
- render flat ground and landing pad(s)
- keyboard controls
- visible thrust / orientation
- basic HUD
- landing detection
- crash detection
- restart

Preserve the existing useful simulation behavior and tests where practical.

The priority is getting a playable game on screen. Do not refactor working code
just to make the architecture prettier.

## Stage 2 — curved moon

Only after Stage 1 is working well:

- replace the flat world with a circular moon
- use radial gravity toward the moon centre
- render visibly curved terrain
- make the camera/local frame usable near the surface and in flight
- keep the same basic Lunar Lander gameplay

Use a deliberately small fictional moon so orbital flight is practical.

Target near-surface values:

- surface gravity: about 1.62 m/s^2
- low circular orbit period: about 600 s (10 minutes)
- moon radius: about 14.77 km
- circumference: about 92.8 km
- near-surface circular orbit speed: about 155 m/s

These values can be tuned slightly for gameplay.

## Stage 3 and later

Do not implement anything beyond the curved playable moon unless explicitly
asked.

Possible later ideas such as terrain variety, missions, cargo, mining,
resources, economy, etc. are only ideas for after the core game works.

## Working style

- Make small, testable changes.
- Run/build the game frequently.
- Prefer straightforward code over frameworks.
- Do not create elaborate milestone documents.
- Do not stop for review after every small implementation step.
- If something already works, preserve it unless changing it is necessary for
  the current stage.
- Keep the project focused on producing a playable game first.

## SDL requirement

Use SDL3 directly for the GUI. Do not use SDL2 or sdl2-compat.

Stage 1 should be migrated to SDL3 before further GUI work.

For keyboard state, use SDL scancodes with SDL_GetKeyboardState(), not SDLK
keycodes as array indices.

Keep the migration narrow: update the existing GUI and CMake integration to
SDL3 without redesigning the simulation or starting Stage 2.

## Text-only model limitation

The coding model used for this repository is text-only.

Never attempt to send an image to the model through any tool or prompt.

Specifically:
- Do not use Read or any equivalent tool on PNG, JPG, JPEG, WebP, PPM, BMP, GIF,
  screenshots, or other image files.
- Do not convert an image to another image format in order to inspect it.
- Do not attach image data, base64 image data, or image tool results to the model.
- Do not attempt visual screenshot inspection.
- An image-bearing tool result can make the current OpenCode session unusable.

Screenshots may be generated only as artifacts for the human user to inspect.
After generating one, do not read it back into the model.

Verify graphical behavior using:
- process exit status
- logs/stdout/stderr
- simulation state
- automated tests
- scripted keyboard/input behavior
- numerical assertions

If visual inspection is required, report the screenshot path to the user and stop
there rather than attempting to inspect the image yourself.


## Git and repository workflow

The active repository is the Lunar Lander game on branch `main`.
Its GitHub remote is `origin`.

### Active project paths

Normal Lunar Lander milestone work may modify:

- AGENTS.md
- PROJECT.md
- STATUS.md
- RUN_PROMPT.txt
- CMakeLists.txt
- .gitignore
- .gitmodules
- include/
- src/
- tests/
- milestones/
- records/
- third_party/

The following are historical coding-agent benchmark material, not the active
game project:

- codex/
- opencode/
- evaluator/
- _common/
- benchmark-runs/

Do not inspect, modify, stage, clean, restore, or commit those historical paths
during normal Lunar Lander milestone work unless explicitly asked to work on
the benchmark.

Pre-existing modifications in historical benchmark paths are not part of the
current milestone and must be left alone.

### Staging and commits

Never use:

    git add .
    git add -A

Stage only explicit active-project paths belonging to the current milestone.

Before committing, inspect:

    git status --short
    git diff --cached --stat
    git diff --cached --check

Do not require the entire worktree to be clean, because historical benchmark
paths may contain unrelated changes.

Each milestone should end in its own commit after its build, tests, and required
human verification pass.

### SDL3 dependency

SDL3 is a pinned Git submodule at:

    third_party/SDL

It uses the upstream repository:

    https://github.com/libsdl-org/SDL.git

The currently pinned SDL release is 3.4.16.

A fresh checkout must initialize dependencies with:

    git submodule update --init --recursive

Do not replace SDL with a machine-local installation under ~/opt or /usr/local.
Do not update the SDL revision during unrelated milestones.

### Push policy

After successfully committing a completed milestone or a required follow-up fix,
push `main` to `origin`:

    git push origin main

Do not leave completed milestone commits only on the local machine.

If a normal push fails, report the exact error. Do not force-push, rewrite
history, change remotes, or discard work in order to make a push succeed.

Do not begin the next milestone until the current milestone commit and any
required follow-up fixes have been pushed.
