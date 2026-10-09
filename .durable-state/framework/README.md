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
./bin/durable-state scan /path/containing/repos
```

`install` and `update` only manage `.durable-state/framework/` and
`.durable-state/MANIFST`. They do not rewrite project-owned state.

Update the central checkout first:

```bash
git -C /path/to/durable-state-machine pull --ff-only
```

Then inspect status and apply compatible framework updates. Schema migrations
are deliberately refused until a migration exists and is reviewed.
