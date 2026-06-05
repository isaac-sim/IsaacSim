# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility tools for rate control, cycle timing, and diagnostics."""

from __future__ import annotations

import sys
import time
from collections import OrderedDict
from typing import Optional


def write(s: str) -> None:
    """A convenient utility method for writing a string to sys.stdout with a buffer flush but no.

    newline.

    Args:
        s: The string to write.
    """
    sys.stdout.write(s)
    sys.stdout.flush()


class SteadyRate:
    """Maintains the steady cycle rate provided on initialization by adaptively sleeping an amount.

    of time to make up the remaining cycle time after work is done.

    Usage:

    rate = SteadyRate(rate_hz=30.)
    while True:
      do.work()  # Do any work.
      rate.sleep()  # Sleep for the remaining cycle time.

    Args:
        rate_hz: The rate in hz to run at.
    """

    def __init__(self, rate_hz: float) -> None:
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.last_sleep_end = time.time()

    def sleep(self) -> None:
        """Sleep for the remaining cycle time to maintain the target rate."""
        work_elapse = time.time() - self.last_sleep_end
        sleep_time = self.dt - work_elapse
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        self.last_sleep_end = time.time()


class CycleTimer:
    """Track time between ticks to measure the cycle rate.

    Currently implemented very simply to take the average across all time. Prints a message every
    print_dt seconds.

    Args:
        print_dt: The desired time delta between prints.
    """

    def __init__(self, print_dt: Optional[float] = 1.0) -> None:
        self.print_dt = print_dt

        self.start_time = None
        self.next_print_time = None
        self.num_ticks = None

    @property
    def elapse_time(self) -> float:
        """Accessor for the current elapsed time."""
        return time.time() - self.start_time

    def tick(self) -> None:
        """Tick the cycle timer. Prints the measured rate in hz every print_dt seconds."""
        curr_time = time.time()

        if self.start_time is None:
            self.start_time = curr_time
            self.num_ticks = 0
            self.next_print_time = curr_time + self.print_dt
            return

        self.num_ticks += 1
        if curr_time >= self.next_print_time:
            elapse = curr_time - self.start_time
            dt = elapse / self.num_ticks
            print("measured rate_hz:", (1.0 / dt))

            self.next_print_time += self.print_dt


class Profiler(object):
    """A profiling utility for capturing the average percentage of a cycle given sections of code.

    take.

    Basic usage: (see cortex_main.py for an example)

        profiler = Profiler(name="cortex_loop_runner", alpha=0.99, skip_cycles=100)

        while simulation_app.is_running():
            profiler.start_cycle()

            profiler.start_capture("task1")
            ... perform task 1 ...
            profiler.end_capture("task1")

            profiler.start_capture("task2")
            ... perform task 2 ...
            profiler.end_capture("task2")

            profiler.end_cycle()
            profiler.print_report(max_rate_hz=rate_hz)

    Args:
        name:
            The name of this profile report. Used in the printout. This parameter can be used to
            distinguish profiler reports when are multiple are running simultaneoulsy. E.g. if each
            of many extensions is reporting it's own profile.
        alpha:
            The alpha blending parameter of the exponential weighted average. Blending is performed
            as running_val = alpha * running_val + (1.-alpha) * new_val.
        skip_cycles:
            The number of cycles to skip up front. E.g. if we know the first k cycles are
            artificially slow, we can use this parameter to skip those cycles.
        print_rate_hz:
            How frequently to print. Printing once per loop can be unreadable. This parameter can be
            used to throttle the prints so they're easier to parse visually.
    """

    def __init__(
        self,
        name: Optional[str] = "report",
        alpha: Optional[float] = 0.9999,
        skip_cycles: Optional[int] = 10,
        print_rate_hz: Optional[float] = 1.0,
    ) -> None:
        self.name = name
        self.alpha = alpha
        self.cycle_num = 0
        self.skip_cycles = skip_cycles
        self.print_rate_hz = print_rate_hz
        self.print_dt = 1.0 / print_rate_hz
        self.last_print_time = None

        self.capture_tags = OrderedDict()
        self.capture_start_times = {}
        self.capture_avg_durations = {}

    @property
    def is_active(self) -> bool:
        """Returns true if the profiler is past the skip cycle set. The profiler won't capture and.

        print anything until is_active is true.
        """
        return self.cycle_num > self.skip_cycles

    def start_cycle(self) -> None:
        """Start the current cycle capture. This method should be called at the beginning of the.

        cycle before any captures.
        """
        self.cycle_num += 1
        self.start_capture("cycle")

    def start_capture(self, tag: str) -> None:
        """Start a named capture. This method should be called after self.start_cycle(), and later.

        self.end_capture(tag) should be called to end the capture anytime before self.end_cycle() is
        called.

        Args:
            tag: The string tag to give this capture. Used to reference the capture in a call to
                end_capture() and used as the capture name in the printed report.
        """
        self.capture_tags[tag] = None
        self.capture_start_times[tag] = time.time()

    def end_capture(self, tag: str) -> None:
        """End the named capture. The tag provided should be tag corresponding to a given open.

        capture. This method should be called after self.start_capture(tag) and before
        self.end_cycle().

        Args:
            tag: The string tag assigned to the capture on start_capture(tag).
        """
        if not self.is_active:
            return

        duration = time.time() - self.capture_start_times[tag]
        if tag in self.capture_avg_durations:
            prev_avg = self.capture_avg_durations[tag]
            self.capture_avg_durations[tag] = self.alpha * prev_avg + (1.0 - self.alpha) * duration
        else:
            self.capture_avg_durations[tag] = duration

    def end_cycle(self) -> None:
        """End the current cycle. No more captures should be performed after this call until.

        self.start_cycle() is again called.
        """
        self.end_capture("cycle")

    def has_avg(self, tag: str) -> bool:
        """Query whether an average capture duration is available for the specified tag.

        Args:
            tag: The string tag given to the capture on start_capture(tag).

        Returns:
            True if there is an active average capture duration available for the given tag.
        """
        return tag in self.capture_avg_durations

    def get_avg(self, tag: str) -> float:
        """Returns the average capture duration for the specified tag. This method does not check.

        whether the average duration exists. Use self.has_avg(tag) to see whether it's safe to call
        this method.

        Args:
            tag: The string tag of the requested capture.

        Returns:
            The average capture duration for the specified tag.
        """
        return self.capture_avg_durations[tag]

    def get_avg_cycle(self) -> float:
        """Get the average cycle duration.

        Returns:
            The cycle average.
        """
        return self.capture_avg_durations["cycle"]

    def print_report(self, max_rate_hz: Optional[float] = None) -> None:
        """Prints a report of the average captures.

        The max_rate_hz parameter can be used to set a cap for the reported cycle rate (hz). E.g. if
        the profiler is capturing only a portion of the overall computation (user code, for
        instance) the measured hz will be high. If the loop runner is running at a realtime rate of
        60hz, this max_rate_hz cap can be used to report slowdowns if necessary, but the cap if it's
        running fast.

        Example:

            ======= <cortex_loop_runner> =======
            avg cycle time: 0.0073777115377921774
            rate hz - w/o cap: 135.54338562540977 ; cap: 60.0
            breakdown:
             - 1) cycle: 0.007378, frac: 100.000000%
             - 2) behavior: 0.000005, frac: 0.070350%
             - 3) world_and_task_step: 0.000009, frac: 0.117199%
             - 4) sim_step: 0.001285, frac: 17.416409%
             - 5) render: 0.003948, frac: 53.512893%

        Args:
            max_rate_hz: The maximum rate in hz to print the report. Throttles prints faster than
                that rate.
        """
        curr_time = time.time()
        if self.last_print_time is None:
            self.last_print_time = curr_time
            return
        elif (curr_time - self.last_print_time) < self.print_dt:
            return

        if not self.is_active:
            print("<profile suppressed during skip period>")
            return

        print(f"======= <{self.name}> =======")
        cycle = self.get_avg_cycle()
        print("avg cycle time:", cycle)

        avg_hz = 1.0 / cycle
        if max_rate_hz is not None and avg_hz > max_rate_hz:
            print("rate hz - w/o cap:", avg_hz, "; cap:", max_rate_hz)
        else:
            print("avg hz:", avg_hz)

        print("breakdown:")
        for i, tag in enumerate(self.capture_tags):
            if self.has_avg(tag):
                avg = self.get_avg(tag)
                print(f" - {i + 1}) {tag}: {avg:f}, frac: {100.0 * avg / cycle:f}%")

        self.last_print_time = curr_time
