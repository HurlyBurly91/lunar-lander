# Versioned Durable-State Framework Payload

This directory is the replaceable, versioned payload for the
`experience-augmented` durable-state framework.

When installed in a project it lives at:

```text
.durable-state/framework/
```

Files below that directory are **framework-owned**. Project agents must not edit
them directly. Project-specific policy belongs in the project root `AGENTS.md`;
project-specific resume additions belong in the root `RUN_PROMPT.txt`; live
state remains in `PROJECT.md`, `STATUS.md`, `TASKS.md`, `milestones/`, `docs/`,
`records/`, `sources/`, `data/`, and `experiences/`.

## Release identity

`RELEASE` contains two independent versions:

```text
FRAMEWORK_VERSION
    version of the replaceable policy/prompt/schema payload

SCHEMA_VERSION
    compatibility version for project-owned durable-state structure
```

A framework-version change with the same schema may be installed mechanically.
A schema-version change requires an ordered migration and must not be applied by
blind replacement.

Projects record the installed release in:

```text
.durable-state/MANIFEST
```

The manifest also records the upstream source commit and a deterministic payload
hash. The updater refuses to overwrite locally modified framework files.

## Schema 2 executable semantics

Schema 2 preserves the bounded state model and adds mechanically supported
semantics:

```text
typed decision-shaping conclusions
explicit bidirectional requirement-to-evidence coverage
deterministic state/evidence validation
human-gate readiness and prerequisite closure
```

Framework 1.1.1 added historical follow-up IDs and provenance-bound legacy
evidence for migrations. Framework 1.1.3 added established-state enforcement
for `HUMAN_VERIFICATION`, including selected-gate disambiguation, active-request
alignment, transitive prerequisite checks, and rejection of unresolved pre-gate
machine work. Framework 1.1.4 excluded Python-generated `__pycache__/` and
`.pyc` files from payload checksums and installed snapshots, preventing
validator execution from invalidating its own immutable source-payload hash.
Framework 1.1.5 adds asynchronous external-verification contracts so an agent
can record an exact provider run and target, checkpoint, yield, and later resume
without occupying a long-running polling call.

Relevant framework-owned contracts:

```text
AGENTS.md
SCHEMAS.md
ASYNC_EXTERNAL_VERIFICATION.md
HUMAN_GATE_READINESS.md
RUN_PROMPT.txt
```

`validator.py` is part of the versioned payload. It checks mechanically decidable
state and evidence relationships without executing application commands,
operating a scheduler, or replacing human judgment.

## Project integration

A managed project root contains these markers:

```text
AGENTS.md:
    Framework-Policy: .durable-state/framework/AGENTS.md

RUN_PROMPT.txt:
    Framework-Resume: .durable-state/framework/RUN_PROMPT.txt
```

The root files remain project-owned. They load this generic framework and then
add repository-specific constraints. Project state is never regenerated from
the starter template during an update.

## Commands

From a checkout of `durable-state-machine`:

```bash
./bin/durable-state status /path/to/project
./bin/durable-state install /path/to/project
./bin/durable-state update /path/to/project
./bin/durable-state migrate --dry-run /path/to/project
./bin/durable-state validate /path/to/project
./bin/durable-state validate --strict /path/to/project
./bin/durable-state validate --json /path/to/project
./bin/durable-state fingerprint /path/to/project
./bin/durable-state scan /path/containing/repos
```

`install`, `update`, and `migrate` manage only `.durable-state/framework/` and
`.durable-state/MANIFEST`. They do not rewrite project-owned state. `migrate`
will finalize only the reviewed schema-1 to schema-2 migration and only after the
current validator accepts the project-owned files in strict mode.

Update the central checkout first:

```bash
git -C /path/to/durable-state-machine pull --ff-only
```

Then inspect status and apply compatible framework updates. Follow the reviewed
migration guide when `status` reports `MIGRATION_REQUIRED`.
