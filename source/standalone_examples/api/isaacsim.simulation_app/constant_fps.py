# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate maintaining a constant frame rate in simulation."""

import argparse
import time

from isaacsim import SimulationApp

# Simple example showing how to fix frame rate to (roughly) a constant value (with respect to wall-clock)
# Note frame rate cannot be set artificially higher than what the sim will run at on the current hardware,
# but it can be kept artificially lower for (eg.) synchronization with an external service.

DESIRED_FRAME_RATE = 10.0  # frames per second
frame_period_s = 1.0 / DESIRED_FRAME_RATE

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": True})

import carb
import omni

# Callback to measure app update time as precisely as possible
last_frametime_timestamp_ns = 0.0
app_update_time_s = 0.0


def update_event_callback(event: carb.events.IEvent) -> None:
    """Record the wall-clock time of each app update event."""
    global last_frametime_timestamp_ns, app_update_time_s
    timestamp_ns = time.perf_counter_ns()
    app_update_time_s = round((timestamp_ns - last_frametime_timestamp_ns) / 1e9, 9)
    last_frametime_timestamp_ns = timestamp_ns


update_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
    on_event=update_event_callback,
    observer_name="constant_fps.update_event_callback",
)

frame_count = 0
while simulation_app.is_running():
    # Measure duration of single app update
    simulation_app.update()
    # Sleep for the duration of the fixed frame
    sleep_duration_s = frame_period_s - app_update_time_s
    if sleep_duration_s <= 0.0:
        carb.log_warn(f"simulation_app.update() took {app_update_time_s} s >= fixed period {frame_period_s} s.")
    else:
        time.sleep(sleep_duration_s)
    instantaneous_fps = 1.0 / max(frame_period_s, app_update_time_s)
    carb.log_warn(f"FPS is {instantaneous_fps}")
    frame_count += 1
    if args.test and frame_count >= 10:
        break

update_sub = None

simulation_app.close()
