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

## Durable task ledger

`TASKS.md` is the canonical execution state for the current task.

The model/UI todo list is NOT authoritative. It is temporary and may disappear
after context compaction, `/new`, model changes, crashes, or resumed sessions.

### Prompt ingestion rule

Whenever the user provides a multi-part implementation request, follow-up
request, bug list, acceptance checklist, or other substantive set of
requirements:

1. Before changing implementation code, parse the request into `TASKS.md`.
2. Give every independently verifiable requirement a stable ID.
3. Split compound requirements into atomic checklist items where useful.
4. Preserve explicit constraints, non-goals, constants, and "do not" clauses.
5. Preserve human-verification requirements separately from automated checks.
6. Only after `TASKS.md` reflects the request may implementation begin.

The UI todo list may mirror `TASKS.md`, but it must never contain requirements
that are absent from `TASKS.md`.

### Required TASKS.md structure

Use this general structure:

    # Active Task

    Milestone: Mxx
    Request: Mxx-Rn
    State: ACTIVE

    ## User requirements

    - [ ] Mxx-Rn-01 ...
    - [ ] Mxx-Rn-02 ...

    ## Preserve / constraints

    - [ ] Mxx-Rn-P01 ...

    ## Automated verification

    - [ ] Mxx-Rn-V01 ...

    ## Human verification

    - [ ] Mxx-Rn-H01 ...

    ## Derived implementation tasks

    - [ ] Mxx-Rn-D01 ...

    ## Evidence / notes

Requirements originating from the user must not be silently replaced by
derived implementation tasks.

### Stable task IDs

Task IDs are durable.

Do not renumber existing task IDs when a later prompt adds more work.

A later request should receive a new request number, for example:

    M04-R1
    M04-R2
    M04-R3

and new requirements:

    M04-R3-01
    M04-R3-02
    ...

If a later user instruction changes an earlier requirement, retain the old
entry but mark it explicitly:

    - [x] M04-R2-03 SUPERSEDED by M04-R3-02

Do not silently edit history so that the earlier requirement disappears.

### Checklist semantics

Use:

    [ ] pending
    [~] in progress
    [x] verified complete
    [!] blocked

Do not mark an item `[x]` merely because code intended to implement it exists.

A completed item should have evidence when practical, for example:

    - [x] M04-R3-02 Exact camera anchoring
      Evidence: tests/test_camera.cpp; lander_camera_tests passes

Human-verification items may only be marked complete after the user explicitly
confirms them.

### Parsing requirements

When converting a user prompt into the ledger:

- do not omit requirements because they appear repetitive
- do not collapse materially different requirements into one vague bullet
- preserve numerical constants exactly
- preserve requested keys, controls, filenames, APIs, and formulas
- preserve explicit prohibitions
- distinguish USER requirements from DERIVED implementation work
- if a requirement is ambiguous, record the ambiguity instead of silently
  selecting a different interpretation

The durable ledger should contain enough information that another agent can
continue the task without access to the original chat prompt.

### During implementation

Update `TASKS.md` as work progresses.

Before moving to another major part of the request, update the status of the
current items.

When new defects or required substeps are discovered, append them under
"Derived implementation tasks" with new stable IDs.

Do not delete unfinished items merely because the implementation approach
changed.

### Session startup / resume

Before continuing an existing task, read:

    AGENTS.md
    PROJECT.md
    STATUS.md
    TASKS.md
    active milestone specification

If `TASKS.md` has `State: ACTIVE`, continue from its open and blocked items.

Do not reconstruct current work solely from conversational memory or the UI
todo list.

### Completion gate

Before claiming a task is complete:

1. inspect every item in `TASKS.md`
2. verify no required USER item remains `[ ]`, `[~]`, or `[!]`
3. verify all automated-verification items have evidence
4. verify required human items have actually been confirmed by the user

If human verification remains, stop and request it. Do not close the milestone.

### Milestone closeout

When the milestone is accepted:

- transfer relevant implementation decisions and verification evidence into the
  milestone record under `records/`
- update STATUS.md
- set TASKS.md State to COMPLETE
- preserve the completed checklist rather than deleting it
- commit and push according to repository Git policy
