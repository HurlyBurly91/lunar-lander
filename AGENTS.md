# Lunar Lander

Framework-Policy: .durable-state/framework/AGENTS.md

Before substantive work, read and obey the versioned framework policy above,
then apply the Lunar Lander-specific rules in this file. This root file is
project-owned. Generic durable-state policy belongs in
`.durable-state/framework/` and is updated with `durable-state update`.

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
- nominal near-surface circular orbit period: about 90 s
- reference moon radius: about 332.4 m
- circumference: about 2.09 km
- reference circular orbit speed: about 23.2 m/s

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
- Keep milestone documents scoped to the active vertical slice.
- Do not create speculative future architecture or implementation plans.
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
- TASKS.md
- CMakeLists.txt
- .githooks/
- .gitignore
- .gitmodules
- include/
- src/
- tests/
- milestones/
- records/
- experiences/
- docs/
- third_party/

`.durable-state/MANIFEST` and `.durable-state/framework/` are framework-owned.
Normal milestone work may read them but must not edit, stage selectively around,
or regenerate them. Update them only through the central durable-state updater.

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

## Automatic push

This repository uses a tracked post-commit hook under:

    .githooks/post-commit

and the local repository must have:

    git config core.hooksPath .githooks

Every commit made on `main` is automatically pushed to `origin/main`.

Agents should therefore:

- commit completed milestone work normally
- allow the post-commit hook to push automatically
- verify that the push succeeded
- if the hook reports a push failure, retry with:

      git push origin main

Do not ask the user to manually push a successfully completed milestone.
Do not force-push or rewrite history.

## Durable-state specialization

This repository uses the **experience-augmented durable-state architecture**.
The ordinary durable-state control plane must remain complete when experience
retrieval is disabled.

The 2026-10-06 migration preserved the exact pre-migration live ledgers as
immutable provenance snapshots under `records/`; those snapshots are not
current execution truth. `STATUS.md` and `TASKS.md` remain the active control
plane.

Experience retrieval is enabled only when `STATUS.md` explicitly says:

    Experience-Retrieval: ENABLED

Otherwise skip retrieval and usefulness telemetry while preserving all baseline
state, evidence, verification, and human gates.

### Lunar Lander canonical references

For gravity, body scaling, orbital mechanics, ephemerides, trajectory physics,
and body-relative motion, the canonical reference is:

    docs/physics-model-gravity.md

For flight-computer computational-rate policy (HOT / WARM / COLD), attitude /
VGO execution, inter-moon transfer correction, and powered-landing guidance,
the canonical references are:

    docs/flight-guidance-computational-rate-tiers.md
    docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
    docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md
    docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md

M06 predictor/debug findings and the isolation harness have additional current
domain references under `docs/m06-*.md`; load them only when that domain is
being changed or investigated.

If a requirement changes a canonical rule, update the applicable document in
the same work and reconcile accepted canonical changes before milestone
closeout.
