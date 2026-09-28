# AGENTS.md

This repository is an autonomous coding benchmark.

## Required read order

Before doing any implementation work:

1. Read `PROJECT.md`.
2. Read `STATUS.md`.
3. Read the authoritative milestone named by `STATUS.md`.
4. Inspect the existing source and tests.
5. Execute the milestone until its acceptance criteria are satisfied.

## Operating rules

- Work autonomously.
- Do not ask for implementation guidance unless there is a genuine external blocker.
- Do not inspect parent directories, sibling benchmark repositories, or anything
  outside this Git repository.
- Do not use the network.
- Do not rewrite the milestone to make it easier.
- Do not remove or weaken tests.
- Preserve existing behavior unless the milestone explicitly changes it.
- Prefer understanding existing architecture before adding parallel mechanisms.
- Build and test after meaningful changes.
- Diagnose failures rather than blindly retrying edits.
- Do not declare success solely because the program compiles.
- Continue until all visible acceptance criteria are satisfied or a genuine
  blocker has been demonstrated.

## Required verification

Before completing a milestone, run:

    cmake -S . -B build
    cmake --build build -j
    ctest --test-dir build --output-on-failure
    git diff --check

## Closeout

When finished:

1. Update `STATUS.md`.
2. Write `records/<milestone>-result.md`.
3. Record:
   - implementation summary
   - files changed
   - tests executed
   - failures encountered
   - dead ends or reverted approaches
   - remaining known defects
4. Do not modify previous benchmark records.
