"""Append ICROS /factory/diagnostics records to a text log file."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional, Sequence, TextIO

from diagnostic_msgs.msg import DiagnosticArray

from waypoint_manager.leg_loader import diagnostic_level_to_int, parse_vertex_from_name

LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR"}


def format_status_values(values: Sequence[Any]) -> str:
    """Serialize DiagnosticStatus.values (key-value pairs) for text logs."""
    parts: list[str] = []
    for entry in values:
        key = str(getattr(entry, "key", ""))
        value = str(getattr(entry, "value", ""))
        if key or value:
            parts.append(f"{key}={value}")
    return ";".join(parts)


class DiagnosticsTxtLogger:
    """Write one text line per DiagnosticStatus entry."""

    def __init__(self, path: Path) -> None:
        self._path = path.expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._handle: Optional[TextIO] = None
        self._open()

    @property
    def path(self) -> Path:
        return self._path

    def _open(self) -> None:
        self._handle = self._path.open("a", encoding="utf-8")

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None

    def log_array(self, msg: DiagnosticArray, timestamp: str) -> None:
        if self._handle is None:
            return

        for status in msg.status:
            level = diagnostic_level_to_int(getattr(status, "level", 0))
            name = str(getattr(status, "name", ""))
            vertex = parse_vertex_from_name(name)
            level_name = LEVEL_NAMES.get(level, "UNKNOWN")
            hardware_id = str(getattr(status, "hardware_id", ""))
            message = str(getattr(status, "message", ""))
            values = format_status_values(getattr(status, "values", []))
            vertex_str = "" if vertex is None else str(vertex)

            line = (
                f"[{timestamp}] device={name} level={level}({level_name}) "
                f"vertex={vertex_str} hardware_id={hardware_id} message={message} "
                f"values={values}"
            )
            self._handle.write(line + "\n")

        self._handle.flush()
