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
- experiences/
- docs/
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

## Durable-state architecture

This repository uses the **experience-augmented durable-state architecture**.
The ordinary durable-state system remains the known-good control plane; the
experience layer is experimental, optional, advisory, and removable without
changing project execution semantics.

The 2026-10-06 migration preserved the exact pre-migration live ledgers as
immutable provenance snapshots under `records/`; those snapshots are not
active execution truth. `STATUS.md` and `TASKS.md` after this migration are
the current control plane.

## Durable-state components

```text
AGENTS.md
    operating policy

PROJECT.md
    product direction and roadmap

docs/
    canonical cross-milestone domain rules

STATUS.md
    minimal current milestone/state pointer

TASKS.md
    canonical live execution ledger

milestones/
    stable milestone contracts

records/
    permanent milestone closeout evidence

experiences/
    selective reusable precedent + diagnostic retrieval telemetry

RUN_PROMPT.txt
    deterministic reconstruction procedure
```

Do not create competing state files such as `TODO.md`, `PROGRESS.md`, `CONTEXT.md`, `MEMORY.md`, `DECISIONS.md`, `PLAN.md`, or `LESSONS.md` unless an explicit architectural change replaces this model.

`experiences/` is not a general notes directory.

## Authority

Use this precedence:

```text
latest explicit user instruction
        ↓
TASKS.md representation of that instruction
        ↓
active milestone specification / repository policy
        ↓
docs/ canonical domain truth
        ↓
current execution evidence
        ↓
relevant precedent
        ↓
general model intuition
```

Experience records are advisory.

They may never override explicit user intent, current milestone requirements, repository invariants, or current evidence.

A new or changed implementation requirement must be written to `TASKS.md` with a stable ID **before implementation code is changed for it**.

If durable files disagree, repair durable state before substantive implementation.

## Canonical domain rules

Stable cross-milestone system/domain rules may live under `docs/`; `docs/` is an active project path.

Canonical documents are loaded on demand. Do not load all of `docs/` automatically.

When current work touches a domain with a known canonical document, read that document before modifying the domain.

Source code may identify its applicable canonical document directly using a delimited implementation region:

```text
// BEGIN CANONICAL ALGORITHM: <descriptive name>
// Reference: docs/<document>.md

... implementation ...

// END CANONICAL ALGORITHM: <descriptive name>
```

Use the host language's comment syntax when `//` is not valid, but preserve the `BEGIN CANONICAL ALGORITHM`, `Reference:`, and `END CANONICAL ALGORITHM` labels.

When inspecting, modifying, moving, or refactoring code inside such a marked region:

1. read the document named by `Reference:` before changing the marked code;
2. preserve the documented algorithm and invariants unless the current requirement explicitly changes them;
3. move the BEGIN / Reference / END markers with the implementation if the code is relocated or decomposed;
4. if an explicit requirement changes the canonical algorithm, update the referenced document in the same work;
5. do not load unrelated canonical documents merely because other documents exist under `docs/`;
6. do not remove or weaken the reference merely because the surrounding architecture changes.

Milestone contracts and `TASKS.md` may also name relevant canonical documents. When a milestone depends on canonical domain state, it may classify documents as `READ`, `MAY MODIFY`, or `MUST PRESERVE`.

This mechanism keeps canonical knowledge available without placing every domain document into normal session context.

Authority for current work is:

```text
latest explicit user instruction
        ↓
TASKS.md representation
        ↓
active milestone specification
        ↓
applicable docs/ rules
        ↓
PROJECT.md
```

If a requirement changes a canonical rule, update the applicable document in the same work. Reconcile all accepted canonical-domain changes before milestone closeout.

### Lunar Lander canonical references

For gravity, body scaling, orbital mechanics, ephemerides, trajectory physics,
and body-relative motion, the canonical reference is:

    docs/physics-model-gravity.md

For flight-computer computational-rate policy (HOT / WARM / COLD), attitude
/ VGO execution, inter-moon transfer correction, and powered-landing guidance,
the canonical references are:

    docs/flight-guidance-computational-rate-tiers.md
    docs/flight-guidance-attitude-bang-bang-control-and-velocity-to-be-gained-node-execution.md
    docs/flight-guidance-intermoon-transfer-differential-correction-warm-starting-and-bounded-replanning.md
    docs/flight-guidance-powered-landing-zem-zev-apollo-polynomial-guidance-and-time-to-go.md

M06 predictor/debug findings and the isolation harness have additional current
domain references under `docs/m06-*.md`; load them only when that domain is
being changed or investigated.

## Milestone state and phase

Allowed milestone states:

```text
NOT_STARTED
ACTIVE
BLOCKED
COMPLETE
```

Allowed active phases:

```text
IMPLEMENTATION
AUTOMATED_VERIFICATION
HUMAN_VERIFICATION
FOLLOW_UP
```

State and phase are separate.

`STATUS.md` must reflect current execution truth.

Human verification is an active phase, not a terminal state.

Use:

```yaml
State: ACTIVE
Phase: HUMAN_VERIFICATION
```

not a top-level state such as `AWAITING HUMAN VERIFICATION`.

## Task states

Use:

```text
[ ] OPEN
[~] IN_PROGRESS
[?] BLOCKED
[H] AWAITING_HUMAN
[x] VERIFIED
[-] SUPERSEDED
```

`AWAITING_HUMAN` is task-level status.

`HUMAN_VERIFICATION` is milestone phase.

`ACTIVE` is milestone state.

Do not conflate them.

## Stable IDs

Every materially new or changed user request affecting the current milestone receives a request group:

```text
M01-R1
M01-R2
...
```

Recommended requirement namespaces:

```text
M01-R1-01    explicit user requirement
M01-R1-P01   preservation constraint
M01-R1-D01   derived implementation task
M01-R1-V01   automated verification
M01-R1-H01   human verification
```

IDs never silently change meaning.

If a later requirement replaces an earlier one, preserve the earlier ID and mark it `SUPERSEDED` with `Superseded-By:`.

## Human verification and follow-up

Never fabricate human acceptance.

Passing automated tests does not satisfy a requirement that explicitly needs human judgment.

If human verification reveals a defect or requested change:

```text
ACTIVE / HUMAN_VERIFICATION
        ↓
persist new request group
        ↓
ACTIVE / FOLLOW_UP
        ↓
ACTIVE / IMPLEMENTATION
        ↓
ACTIVE / AUTOMATED_VERIFICATION
        ↓
ACTIVE / HUMAN_VERIFICATION
```

Preserve all unresolved prior human-verification tasks.

Do not keep the milestone in `HUMAN_VERIFICATION` while implementation or automated verification is actively occurring.

A human-observed defect normally creates a new request group instead of silently mutating the completed request that exposed it.

Repeated observations or trials inside one already-persisted investigative request are different. If the user requirement, acceptance criteria, preservation constraints, investigative question, and execution phase remain unchanged, keep those trials under the existing request group and normally under one existing derived task.

Do not allocate a new request ID or `Dxx` task merely because another capture, measurement, parameter value, or candidate trial was produced.

## Baseline execution rules

For each substantive request:

1. read current durable state;
1a. read relevant `docs/` selectively for the affected domain;
2. persist new/changed requirements to `TASKS.md`;
3. synchronize `STATUS.md` and the `TASKS.md` header;
4. identify the decision/work context;
5. optionally retrieve relevant precedent when the decision is materially non-trivial;
6. implement the highest-priority non-blocked work;
7. transition to `AUTOMATED_VERIFICATION` when implementation is ready;
8. run relevant automated verification;
9. attach concise evidence to the corresponding tasks;
10. move subjective/interactive checks to `AWAITING_HUMAN`;
11. enter `ACTIVE / HUMAN_VERIFICATION` only when no implementation or automated-verification work remains;
12. stop for explicit human judgment where required.

The baseline sequence must still work correctly with experience retrieval disabled.

## Investigative transaction granularity

Durable state preserves expensive-to-reconstruct decisions; it does not require every observation to become a repository transaction.

Treat repeated work as one bounded investigative loop when it serves one persisted request/task, asks the same diagnostic question, and does not change the user-facing contract, preservation constraints, milestone scope, active request, or execution phase.

Inside that loop:

- reuse the existing request group and derived task;
- do not change `STATUS.md` for each measurement or trial;
- do not append `records/` prose for each failed capture;
- do not retrieve experiences for routine trial-to-trial iteration;
- do not require a Git checkpoint for each trial;
- put exhaustive measurements in generated JSON/TSV/CSV/log/capture artifacts;
- persist only the decisive measurements, artifact reference when useful, conclusion, uncertainty, and next direction needed to resume safely.

`STATUS.md` changes only when milestone state, execution phase, or active request changes.

Checkpoint durable state when losing the current decision would be materially expensive to reconstruct. Typical boundaries are:

- a new or changed user requirement;
- a causal conclusion that changes the next investigative direction;
- acceptance or rejection of a strategy, architecture, or topology;
- a blocker or human-verification gate;
- a long interruption or handoff;
- a risky operation where the immediately preceding decision must survive failure.

Another frame, sample, parameter trial, expected failed capture, or longer context is not by itself a checkpoint boundary.

## Experience model

Experience memory stores small, evaluated precedents:

```text
context
    ↓
decision
    ↓
action
    ↓
observed outcome
    ↓
lesson
    ↓
future applicability
```

Do not implement this as a giant literal decision tree.

Use structured cases retrieved by relevance.

The conceptual loop is:

```text
RETRIEVE
REUSE / ADAPT
REVISE / EVALUATE
RETAIN SELECTIVELY
```

## When to retrieve precedent

Do not retrieve experience for every trivial edit.

Retrieval is appropriate for materially non-trivial decisions such as:

- ambiguous requirement interpretation;
- human-verification failure;
- repeated test failure;
- regression;
- architecture choice;
- workflow/state-transition anomaly;
- non-obvious debugging strategy;
- conflicting automated and human evidence;
- substantial refactor strategy;
- choosing among multiple plausible fixes;
- a situation explicitly similar to a prior failure.

If no useful precedent is found, continue normally. Missing precedent is never a blocker.

For a bounded investigative loop, retrieve at loop entry only when precedent may materially change the strategy. Once the strategy is established, skip retrieval for routine inner-loop trials. Retrieve again only when a strategic boundary is crossed, such as a changed failure class, conflicting evidence, repeated stall, material topology/architecture change, or new human feedback.

Do not create retrieval telemetry for a trial in which no retrieval occurred.

Retrieve only a small relevant set, normally a few cases rather than the whole store.

## Precedent adaptation

Do not blindly replay an older action.

For every materially used precedent compare:

```text
similarities
differences
assumptions
applicability
```

Prefer transferring the lesson rather than copying implementation details that do not fit the current context.

## Selective retention

Do not store every event.

Retain an experience only when:

1. something materially informative happened;
2. the outcome has enough evidence to evaluate;
3. there is plausible future reuse value.

Good candidates include:

- non-obvious successful decisions;
- plausible mistakes likely to recur;
- human-verification failures that revealed test gaps;
- unexpected subsystem interactions;
- requirement interpretations proven wrong;
- workflow choices that materially saved or wasted effort;
- architectural decisions whose consequences became observable;
- repeated failure patterns;
- useful debugging strategies;
- cases where an internal metric diverged from human-visible quality.

Routine compilation, typo fixes, ordinary passing tests, and individual trials inside a still-open investigative loop are not experiences.

## Experience confidence and status

Experience confidence may be:

```text
low
medium
high
```

Experience lifecycle status may be:

```text
active
stale
superseded
```

Preserve stale/superseded precedent for provenance but exclude superseded cases from ordinary retrieval and down-rank stale cases.

Do not build elaborate automatic expiry or scoring until observed failures demonstrate a need.

## Retrieval instrumentation

Record enough diagnostic telemetry to answer:

```text
experience retrieved
        ↓
was it actually used?
        ↓
helpful / neutral / misleading
        ↓
did the resulting decision succeed?
```

`experiences/retrievals.jsonl` is diagnostic telemetry, not authoritative project state.

Distinguish:

- retrieved precedent;
- precedent actually used;
- usefulness once an outcome is observable;
- resulting decision outcome.

Do not promote an experience merely because it was retrieved frequently.

## From experience to policy

An experience is a case.

A heuristic is a generalized lesson supported by one unusually strong case or repeated cases.

A durable `AGENTS.md` rule requires stronger evidence and generality.

Conceptually:

```text
episode
  ↓ repeated/strong evidence
heuristic
  ↓ general, stable, well-supported
AGENTS.md policy
```

Do not allow one anecdote to modify durable operating policy automatically.

## Experimental control and rollback

The experience layer must remain separable from baseline execution.

It must not be required to:

- reconstruct current state;
- know what remains to be done;
- identify current requirements;
- run tests;
- gate human verification;
- complete milestones;
- close out records.

Use this tuning rule:

```text
no observed problem
    → leave it alone

specific observed failure
    → tune that mechanism only

systematically worse behavior
    → disable experience retrieval
    → continue with baseline durable state
```

Do not pre-emptively tune retrieval, retention, ranking, staleness, or promotion for hypothetical failures.

## Milestone closeout

A milestone is `COMPLETE` only when all required automated and human verification has passed.

Then:

1. confirm milestone acceptance criteria;
1a. reconcile accepted canonical domain-state changes into `docs/`;
2. write `records/Mxx-*.md`;
3. preserve final IDs, supersession links, important decisions, evidence, human results, and known limitations;
4. evaluate whether any completed outcome warrants a reusable experience;
5. set `STATUS.md` to `COMPLETE`;
6. update `PROJECT.md` at roadmap granularity when appropriate;
7. commit/push according to repository policy;
8. initialize the next milestone;
9. reset `TASKS.md` to bounded live state.

Do not write a final closeout record or closeout commit while required human verification remains unresolved.

## Historical retrieval

Do not load all records or all experiences by default.

Read records when provenance is needed.

Retrieve experiences when a non-trivial current decision may benefit from reusable precedent.

## Bootstrap rule

If this repository is still in the template bootstrap state:

```yaml
Milestone: M01
State: NOT_STARTED
```

use the current explicit user request to populate `PROJECT.md`, define the first milestone contract, create the first request group in `TASKS.md`, and transition to `ACTIVE / IMPLEMENTATION` before modifying implementation code.

Experience files start empty. Do not fabricate precedent during bootstrap.
