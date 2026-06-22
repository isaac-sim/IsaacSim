# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Lightweight scoped wall-clock timing.

This mirrors the small C++ scoped-timer pattern used elsewhere: each named scope
adds one sample to a process-local table, and callers can print the aggregate
table when a run finishes.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from types import TracebackType
from typing import TextIO


@dataclass
class ScopedTimerEntry:
    """Aggregate timing data for one timer name."""

    num_events: int = 0
    total_time_ns: int = 0

    @property
    def average_time_ms(self) -> float:
        """Average elapsed wall time in milliseconds."""
        if self.num_events == 0:
            return 0.0
        return self.total_time_ns / self.num_events / 1_000_000.0

    @property
    def total_time_s(self) -> float:
        """Total elapsed wall time in seconds."""
        return self.total_time_ns / 1_000_000_000.0


class ScopedTimer:
    """Context-manager timer that aggregates samples by name.

    Args:
        name: The timer name that samples are aggregated under.

    Example:

    .. code-block:: python

        with ScopedTimer("render_keyframes"):
            render_keyframes(...)

        ScopedTimer.print_table()
    """

    _lock = threading.Lock()
    _table: dict[str, ScopedTimerEntry] = {}
    _enabled: bool = True

    @classmethod
    def set_enabled(cls, enabled: bool) -> None:
        """Enable or disable timing globally; when disabled, scoped timers are no-ops.

        Args:
            enabled: Whether timing is enabled.
        """
        cls._enabled = enabled

    def __init__(self, name: str) -> None:
        self._name = name
        self._start_ns: int | None = None
        self._elapsed_ns: int | None = None

    def __enter__(self) -> "ScopedTimer":
        """Start the timer, or do nothing when timing is disabled.

        Returns:
            This timer instance.
        """
        if not self._enabled:
            self._start_ns = None
            self._elapsed_ns = None
            return self
        self._start_ns = time.perf_counter_ns()
        self._elapsed_ns = None
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Stop the timer and record the elapsed sample.

        Args:
            exc_type: Exception type if the block raised, else None.
            exc: Exception instance if the block raised, else None.
            traceback: Traceback if the block raised, else None.
        """
        if self._start_ns is None:
            return
        elapsed_ns = time.perf_counter_ns() - self._start_ns
        self._elapsed_ns = elapsed_ns
        with self._lock:
            entry = self._table.setdefault(self._name, ScopedTimerEntry())
            entry.num_events += 1
            entry.total_time_ns += elapsed_ns
        self._start_ns = None

    @property
    def elapsed_time_ms(self) -> float | None:
        """Elapsed time for this scoped timer instance, in milliseconds."""
        if self._elapsed_ns is None:
            return None
        return self._elapsed_ns / 1_000_000.0

    @classmethod
    def reset_all(cls) -> None:
        """Clear all recorded timing data."""
        with cls._lock:
            cls._table.clear()

    @classmethod
    def get_values_for_timer(cls, name: str) -> tuple[int, float]:
        """Return the recorded values for a timer name.

        Args:
            name: The timer name to look up.

        Returns:
            The ``(num_events, average_time_ms)`` for the timer.
        """
        with cls._lock:
            entry = cls._table.get(name, ScopedTimerEntry())
            return entry.num_events, entry.average_time_ms

    @classmethod
    def rows(cls) -> list[tuple[str, int, float, float]]:
        """Return the aggregate timer rows.

        Returns:
            One ``(name, count, average_ms, total_s)`` tuple per timer, sorted by name.
        """
        with cls._lock:
            return [
                (name, entry.num_events, entry.average_time_ms, entry.total_time_s)
                for name, entry in sorted(cls._table.items())
            ]

    @classmethod
    def format_table(cls) -> str:
        """Format the aggregate timer table.

        Returns:
            The formatted table text, or an empty string when no timers ran.
        """
        rows = cls.rows()
        if not rows:
            return ""

        name_width = max(len(name) for name, _, _, _ in rows)
        lines = [
            "==========================================================================",
            "Profile Timers:",
        ]
        for name, num_events, average_ms, total_s in rows:
            lines.append(
                f"{name:>{name_width}}, average time used: {average_ms:10.3f} ms, "
                f"{num_events:10d} event(s), total time used: {total_s:10.3f} secs"
            )
        lines.append("==========================================================================")
        return "\n".join(lines)

    @classmethod
    def print_table(cls, stream: TextIO | None = None) -> None:
        """Log the aggregate timer table if any timers ran.

        Logs at info level by default, so the profiling output is not flagged as an error (Kit routes
        Python ``stderr`` to its error channel). Pass ``stream`` to write to that stream instead.

        Args:
            stream: Output stream to write to; when None, the table is logged at info level.
        """
        table = cls.format_table()
        if not table:
            return
        if stream is not None:
            print(table, file=stream)
            return
        try:
            import carb

            carb.log_info(table)
        except ImportError:
            print(table)
