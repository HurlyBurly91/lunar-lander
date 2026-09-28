#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "usage: $0 <codex|opencode> <m01|m02|m03>"
    exit 2
fi

HARNESS="$1"
TASK="$2"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPO="$ROOT/$HARNESS/$TASK"

case "$TASK" in
    m01) HIDDEN="$ROOT/evaluator/M01-hidden.cpp" ;;
    m02) HIDDEN="$ROOT/evaluator/M02-hidden.cpp" ;;
    m03) HIDDEN="$ROOT/evaluator/M03-hidden.cpp" ;;
    *) echo "unknown task: $TASK"; exit 2 ;;
esac

echo "== visible build/tests =="
cmake -S "$REPO" -B "$REPO/build"
cmake --build "$REPO/build" -j
ctest --test-dir "$REPO/build" --output-on-failure

echo
echo "== hidden evaluator =="

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

g++ \
    -std=c++20 \
    -O2 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -I"$REPO/include" \
    "$REPO/src/sim.cpp" \
    "$HIDDEN" \
    -o "$TMP/hidden-eval"

"$TMP/hidden-eval"

echo
echo "PASS: $HARNESS $TASK"
