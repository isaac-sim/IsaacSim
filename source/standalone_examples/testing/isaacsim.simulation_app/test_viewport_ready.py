# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test to verify that viewport rendering callbacks are triggered on the first frame."""

import argparse
import sys
import time

parser = argparse.ArgumentParser(description="Script to test viewport ready")
parser.add_argument(
    "--silent",
    action="store_true",
    default=False,
    help="Silent mode (default: False)",
)
_ARGS, _ = parser.parse_known_args()
from isaacsim import SimulationApp

# Create simulation app
kit = SimulationApp({"headless": False})


import carb.eventdispatcher
import omni
import omni.usd

# Track if callback was called
callback_called = False


def data_acquisition_callback(event):
    """Callback function triggered on rendering events."""
    global callback_called
    print(f"Received render event: {event.event_name}")
    callback_called = True


# Subscribe to NEW_FRAME rendering events
usd_context = omni.usd.get_context()
data_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=usd_context.stage_rendering_event_name(omni.usd.StageRenderingEventType.NEW_FRAME, True),
    on_event=data_acquisition_callback,
    observer_name="test_viewport_ready.acquisition_callback",
    order=0,
)

try:
    # Simulate one frame to trigger rendering callback
    kit.update()

    # Check if callback was called on first frame
    if not callback_called:
        if not _ARGS.silent:
            print("[error] Rendering callback was not called on the first frame")

        # Wait for viewport to be ready before exiting to prevent app hang
        max_wait_frames = 100
        for i in range(max_wait_frames):
            if not _ARGS.silent:
                print(f"Waiting for viewport to be ready before exiting... frame {i}")
            kit.update()
            time.sleep(0.02)
            if callback_called:
                break

        if not _ARGS.silent:
            sys.exit(1)

    if not _ARGS.silent:
        print("SUCCESS: Rendering callback was called on the first frame")

finally:
    # Cleanup
    data_callback = None
    kit.close()
