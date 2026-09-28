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

## Durable task state

The repository uses several deliberately separate durable-state layers.

They are not interchangeable.

### State architecture

    AGENTS.md
        durable procedural memory
        how work is done

    PROJECT.md
        durable product intent
        where the project is going

    STATUS.md
        minimal current pointer
        which milestone and phase are active

    milestones/Mxx-*.md
        stable task semantics / milestone contract
        what success means

    TASKS.md
        bounded high-fidelity execution state
        what must happen now

    records/Mxx-*.md
        compressed retrospective provenance
        what actually happened and why

    RUN_PROMPT.txt
        reconstruction procedure
        how a fresh agent restores the current state

`TASKS.md` is the canonical live execution state.

The UI/model todo list is temporary and non-authoritative.

### Bounded working-state rule

`TASKS.md` is durable working memory, not a permanent append-only event log.

It contains only the active milestone's current execution state and the
requirement/evidence history needed to finish that milestone correctly.

At milestone closeout:

1. preserve the final requirement IDs and relevant supersession relationships
   in the milestone record
2. preserve requirement -> implementation links where useful
3. preserve requirement -> test/evidence links
4. preserve human-verification results
5. write the resulting retrospective state to `records/Mxx-*.md`
6. mark the current TASKS ledger complete
7. reset `TASKS.md` when the next milestone begins

Do not carry all atomic tasks from completed milestones into the next
milestone's working context.

Git history preserves lower-level intermediate mutations when needed.

Historical records are durable memory but are NOT loaded automatically on every
session. Read a prior record only when needed to understand a dependency,
rationale, regression, or previous design decision.

### Authority and precedence

For current work, interpret state in this order:

1. latest explicit user instruction
2. `TASKS.md` representation of that instruction
3. active milestone specification
4. `PROJECT.md`

`AGENTS.md` supplies operating invariants and workflow rules across all of
those layers.

If a new user instruction changes an earlier active requirement, preserve the
relationship explicitly rather than silently rewriting history, for example:

    - [x] M04-R1-07 SUPERSEDED by M04-R2-03

If a user instruction changes stable milestone semantics rather than merely
implementation details, update the active milestone specification as well.

### STATUS.md states

Use a small explicit state vocabulary:

    NOT STARTED
    ACTIVE
    AWAITING HUMAN VERIFICATION
    BLOCKED
    COMPLETE

`STATUS.md` should remain a small pointer, not a second task ledger.

It should identify:

- active milestone
- state
- optional current phase
- milestone specification
- `TASKS.md` as the live execution ledger

Do not copy the full checklist into STATUS.md.

### Prompt ingestion rule

Whenever the user provides a substantive multi-part implementation request,
follow-up request, bug list, changed requirement, acceptance checklist, or
other execution-relevant instruction:

1. determine whether it changes the active milestone contract or only the
   current execution state
2. before changing implementation code, persist it to `TASKS.md`
3. allocate a stable request ID if it represents a new request group
4. give independently verifiable requirements stable IDs
5. split materially distinct requirements into atomic entries
6. preserve constants, formulas, controls, filenames, APIs, and explicit
   prohibitions
7. preserve automated and human verification separately
8. only then begin implementation

The UI todo list may mirror `TASKS.md`, but must not contain required work that
is absent from `TASKS.md`.

### Stable IDs

Within an active milestone, request groups use:

    M04-R1
    M04-R2
    M04-R3

Atomic entries use:

    M04-R2-01      user requirement
    M04-R2-P01     preserve / constraint
    M04-R2-V01     automated verification
    M04-R2-H01     human verification
    M04-R2-D01     derived implementation task

Do not renumber IDs after they have been assigned.

Requirements originating from the user must not be replaced by derived
implementation tasks.

### Checklist semantics

Use:

    [ ] pending
    [~] in progress
    [x] verified complete
    [!] blocked

Do not mark an item `[x]` merely because code intended to satisfy it exists.

Completion requires evidence appropriate to the requirement.

Example:

    - [x] M04-R2-03 Exact camera anchoring
      Source: USER
      Files: include/lander/camera.hpp
      Evidence: tests/test_camera.cpp::full_revolution_anchor

Useful metadata may include:

    Source:
    Files:
    Evidence:
    Depends:
    Supersedes:
    Notes:

Do not add metadata mechanically when it provides no value.

Human-verification items may only be marked complete after explicit user
confirmation.

### Requirement traceability

When practical, preserve the chain:

    user observation / requirement
        ->
    stable requirement ID
        ->
    implementation artifact
        ->
    executable test or other evidence
        ->
    verification result

A test's existence alone is weaker evidence than a successful execution.
Record actual verification results when practical.

### During implementation

Update `TASKS.md` as execution proceeds.

Before moving to another major requirement group, persist the current state.

If implementation reveals a necessary subtask not explicitly requested, append
it under `Derived implementation tasks` with a stable D-ID.

Do not delete an unfinished requirement merely because the implementation
approach changed.

Do not use the conversation transcript or model todo list as the sole durable
record of unfinished work.

### Session startup and resume

A fresh or resumed agent should reconstruct current state semantically:

1. read `AGENTS.md` for operating invariants
2. read `STATUS.md` to identify the active milestone and phase
3. read the active milestone specification for stable requirements and
   acceptance criteria
4. read `TASKS.md` for current unresolved execution state, dependencies,
   relevant completed verification, and pending human verification
5. read `PROJECT.md` for product-level direction and constraints

Do not automatically load historical milestone records.

Retrieve a historical record only when needed to understand a dependency,
rationale, regression, or prior design decision.

If the current user message changes requirements, persist that change to
`TASKS.md` before implementation.

### Completion gate

Before claiming the current request or milestone is complete:

1. inspect every required item in `TASKS.md`
2. verify no applicable USER requirement remains `[ ]`, `[~]`, or `[!]`
3. verify automated-verification items have execution evidence
4. verify preservation constraints still hold
5. verify required human items have actually been confirmed by the user

If required human verification remains:

- set STATUS to `AWAITING HUMAN VERIFICATION`
- keep the relevant human items open
- stop and request human verification
- do not close the milestone

### Milestone closeout

After human acceptance:

1. write/update `records/Mxx-*.md`
2. preserve the final requirement IDs relevant to what shipped
3. preserve important supersession relationships
4. preserve implementation decisions and rationale
5. preserve automated execution evidence
6. preserve human-verification results
7. update `STATUS.md` to `COMPLETE`
8. set the active `TASKS.md` ledger to `COMPLETE`
9. commit and push according to repository Git policy

When the next milestone begins, replace `TASKS.md` with a fresh bounded ledger
for that milestone rather than carrying forward the previous milestone's
atomic execution history.

### State transitions after human feedback

If the milestone is in `AWAITING HUMAN VERIFICATION` and human feedback creates
new required implementation or automated-verification work:

1. persist the feedback as a new request group in `TASKS.md`
2. set `TASKS.md` State to `ACTIVE`
3. set `STATUS.md` State to `ACTIVE`
4. update `STATUS.md` Phase to identify the new active request group
5. perform the new implementation and automated verification work
6. when all automated work for that request is complete, return both
   `TASKS.md` and `STATUS.md` to `AWAITING HUMAN VERIFICATION`
7. keep all unresolved human-verification items open

Do not leave `STATUS.md` as `AWAITING HUMAN VERIFICATION` while implementation
or automated-verification work is actively in progress.

A human-verification failure does not complete the previous H-item. Keep it open
until the user explicitly confirms the corrected behavior.

If the new request supersedes an earlier active requirement, preserve that
relationship explicitly in `TASKS.md` rather than deleting or silently rewriting
the earlier requirement.
