#!/usr/bin/env python3
"""Schema-2 validator entry point with human-gate readiness enforcement.

The original schema-2 validator is retained in ``validator_core.py``. This
entry point extends it with the established-state invariant discovered during
M18-R13 live use: HUMAN_VERIFICATION is legal only when the selected human gate
is actually the next executable gate and all pre-gate work is complete.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

# The managed framework directory is hash-protected. Importing validator_core
# must not generate files inside the installed payload.
sys.dont_write_bytecode = True
from typing import Sequence

CORE_PATH = Path(__file__).with_name("validator_core.py")
SPEC = importlib.util.spec_from_file_location("durable_state_validator_core", CORE_PATH)
if SPEC is None or SPEC.loader is None:  # pragma: no cover - installation corruption
    raise RuntimeError(f"cannot load durable-state validator core: {CORE_PATH}")
_core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = _core
SPEC.loader.exec_module(_core)

SUPPORTED_SCHEMA_VERSION = _core.SUPPORTED_SCHEMA_VERSION
FieldValue = _core.FieldValue
Task = _core.Task
Diagnostic = _core.Diagnostic
ParsedLedger = _core.ParsedLedger

# Keep the existing validator module grammar API available to clients/tests.
MILESTONE_RE = _core.MILESTONE_RE
REQUEST_RE = _core.REQUEST_RE
TASK_ID_RE = _core.TASK_ID_RE
TASK_REF_RE = _core.TASK_REF_RE

# Accept the same letter-suffixed milestone grammar as validator_core.
_REQUEST_GROUP_RE = re.compile(rf"^({_core.REQUEST_PATTERN})(?=-|$)")
_TERMINAL_MARKERS = {"x", "-"}


class Validator(_core.Validator):
    """Core schema-2 validator plus human-gate readiness checks."""

    def validate(self) -> list[Diagnostic]:
        self._validate_manifest_schema()

        status_path = self.root / "STATUS.md"
        tasks_path = self.root / "TASKS.md"
        status_header = self._parse_yaml_header(status_path)
        ledger = self._parse_tasks(tasks_path)

        if status_header is None or ledger is None:
            return self._sorted_diagnostics()

        self._validate_headers(status_path, status_header, ledger)
        task_by_id = self._validate_task_identity(ledger)
        self._validate_references(ledger, task_by_id)
        self._validate_evidence_coverage(ledger, task_by_id)
        self._validate_verification_tasks(ledger, task_by_id)
        self._validate_conclusions(ledger, task_by_id)
        self._validate_supersession(ledger, task_by_id)
        self._validate_file_bindings(ledger)
        self._validate_completion(ledger)
        self._validate_human_gate_readiness(
            status_path,
            status_header,
            ledger,
            task_by_id,
        )

        return self._sorted_diagnostics()

    @staticmethod
    def _request_group(task_id: str) -> str | None:
        match = _REQUEST_GROUP_RE.match(task_id)
        return match.group(1) if match else None

    def _dependency_refs(self, task: Task) -> list[tuple[str, int]]:
        return self._refs([*task.values("Requires"), *task.values("Blocked-By")])

    def _task_depends_on(
        self,
        task: Task,
        target_id: str,
        task_by_id: dict[str, Task],
        seen: set[str] | None = None,
    ) -> bool:
        visited = set() if seen is None else set(seen)
        if task.task_id in visited:
            return False
        visited.add(task.task_id)
        for ref, _ in self._dependency_refs(task):
            if ref == target_id:
                return True
            dependency = task_by_id.get(ref)
            if dependency and self._task_depends_on(
                dependency,
                target_id,
                task_by_id,
                visited,
            ):
                return True
        return False

    def _selected_human_gate(
        self,
        status_path: Path,
        status: dict[str, FieldValue],
        ledger: ParsedLedger,
        task_by_id: dict[str, Task],
    ) -> Task | None:
        pending = [
            task for task in ledger.tasks if task.kind == "H" and task.marker == "H"
        ]
        next_gate = status.get("Next-Gate")

        if next_gate is None:
            if len(pending) == 1:
                return pending[0]
            if not pending:
                self.error(
                    "E131",
                    status_path,
                    status.get("Phase", FieldValue("", 1)).line,
                    "HUMAN_VERIFICATION requires a pending [H] human-verification task",
                )
            else:
                self.error(
                    "E131",
                    status_path,
                    status.get("Phase", FieldValue("", 1)).line,
                    "HUMAN_VERIFICATION has multiple pending human gates; add STATUS.md Next-Gate",
                )
            return None

        if self._placeholder(next_gate.value):
            self.error(
                "E132",
                status_path,
                next_gate.line,
                "Next-Gate is unresolved",
            )
            return None

        gate = task_by_id.get(next_gate.value)
        if gate is None:
            self.error(
                "E132",
                status_path,
                next_gate.line,
                f"Next-Gate {next_gate.value} does not reference a live task",
            )
            return None
        if gate.kind != "H" or gate.marker != "H":
            self.error(
                "E132",
                status_path,
                next_gate.line,
                f"Next-Gate {next_gate.value} must reference a pending [H] task",
            )
            return None
        return gate

    def _validate_gate_prerequisites(
        self,
        ledger: ParsedLedger,
        gate: Task,
        task_by_id: dict[str, Task],
    ) -> None:
        visited: set[str] = set()
        visiting: set[str] = set()
        reported_unresolved: set[str] = set()
        reported_cycles: set[tuple[str, str]] = set()

        def visit(task: Task) -> None:
            if task.task_id in visited:
                return
            visiting.add(task.task_id)
            for ref, line in self._dependency_refs(task):
                dependency = task_by_id.get(ref)
                if dependency is None:
                    continue  # Core reference validation reports the missing ID.
                if ref in visiting:
                    edge = (task.task_id, ref)
                    if edge not in reported_cycles:
                        reported_cycles.add(edge)
                        self.error(
                            "E136",
                            ledger.path,
                            line,
                            f"selected human gate prerequisite cycle: {task.task_id} -> {ref}",
                        )
                    continue
                if dependency.marker not in _TERMINAL_MARKERS and ref not in reported_unresolved:
                    reported_unresolved.add(ref)
                    self.error(
                        "E134",
                        ledger.path,
                        line,
                        f"selected human gate {gate.task_id} has unresolved prerequisite {ref}",
                    )
                visit(dependency)
            visiting.remove(task.task_id)
            visited.add(task.task_id)

        visit(gate)

    def _validate_human_gate_readiness(
        self,
        status_path: Path,
        status: dict[str, FieldValue],
        ledger: ParsedLedger,
        task_by_id: dict[str, Task],
    ) -> None:
        phase = status.get("Phase")
        if phase is None or phase.value != "HUMAN_VERIFICATION":
            return

        gate = self._selected_human_gate(status_path, status, ledger, task_by_id)
        if gate is None:
            return

        active_request = status.get("Active-Request")
        gate_request = self._request_group(gate.task_id)
        if (
            active_request is not None
            and gate_request is not None
            and active_request.value != gate_request
        ):
            self.error(
                "E133",
                status_path,
                active_request.line,
                f"HUMAN_VERIFICATION Active-Request {active_request.value} must match selected gate request {gate_request}",
            )

        self._validate_gate_prerequisites(ledger, gate, task_by_id)

        if active_request is None:
            return
        for task in ledger.tasks:
            if self._request_group(task.task_id) != active_request.value:
                continue
            if task.kind not in {"D", "V"} or task.marker in _TERMINAL_MARKERS:
                continue
            if self._task_depends_on(task, gate.task_id, task_by_id):
                continue  # Explicitly downstream work may remain pending.
            self.error(
                "E135",
                ledger.path,
                task.line,
                f"HUMAN_VERIFICATION has unresolved pre-gate work {task.task_id}",
            )


# The retained CLI implementation resolves Validator from its module globals.
_core.Validator = Validator
build_parser = _core.build_parser


def main(argv: Sequence[str] | None = None) -> int:
    return _core.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
