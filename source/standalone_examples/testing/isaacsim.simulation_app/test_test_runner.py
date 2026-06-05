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

"""Test to verify that the Test Runner window can load and populate tests without errors."""

import argparse
import asyncio
import os
import sys

# Parse arguments before SimulationApp is created
parser = argparse.ArgumentParser(description="Test Runner validation script")
parser.add_argument(
    "--experience",
    type=str,
    default="isaacsim.exp.full.kit",
    help="Experience file to use (default: isaacsim.exp.full.kit)",
)
_ARGS, _ = parser.parse_known_args()

from isaacsim import SimulationApp

# Create simulation app
# Disable ROS2 bridge since this test doesn't need it and it can crash if ROS environment isn't set up
experience_name = _ARGS.experience
experience_path = experience_name
if not os.path.isabs(experience_name):
    experience_path = f'{os.environ["EXP_PATH"]}/{experience_name}'
kit = SimulationApp(
    {
        "headless": True,
    },
    experience=experience_path,
)

import omni.kit.app
import omni.kit.test
import omni.ui as ui

# Track any async task exceptions that occur
captured_exceptions = []

# Track test discovery state
test_discovery_complete = False
discovered_test_count = 0


def custom_exception_handler(loop, context):
    """Custom exception handler to capture asyncio task exceptions."""
    exception = context.get("exception")
    message = context.get("message", "")
    if exception:
        captured_exceptions.append((message, exception))
        print(f"[ERROR] Asyncio task exception: {message} - {type(exception).__name__}: {exception}", file=sys.stderr)
    else:
        captured_exceptions.append((message, None))
        print(f"[ERROR] Asyncio error: {message}", file=sys.stderr)


# Set custom exception handler
loop = asyncio.get_event_loop()
original_handler = loop.get_exception_handler()
loop.set_exception_handler(custom_exception_handler)

# Enable required extensions
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.test.utils", True)
for _ in range(3):
    kit.update()

tests_ext_name = "omni.kit.window.tests"
print(f"[INFO] Enabling {tests_ext_name}")
manager.set_extension_enabled(tests_ext_name, True)
for _ in range(10):
    kit.update()
    if manager.is_extension_enabled(tests_ext_name):
        break
else:
    print(f"[ERROR] Timed out enabling '{tests_ext_name}'.")
    kit.close()
    sys.exit(2)

# Allow frames for extension to fully initialize
for _ in range(120):
    kit.update()


def on_tests_discovered(canceled=False):
    """Callback when test discovery completes."""
    global test_discovery_complete, discovered_test_count
    if canceled:
        print("[WARNING] Test discovery was canceled")
    else:
        discovered_test_count = len(populator.tests)
    test_discovery_complete = True


# Start test discovery
print("[INFO] Starting test discovery...")
populator = omni.kit.test.TestPopulateAll()
populator.get_tests(on_tests_discovered)

# Wait for test discovery to complete with timeout
MAX_FRAMES = 500
for frame in range(MAX_FRAMES):
    kit.update()
    if test_discovery_complete:
        break
    if frame > 0 and frame % 100 == 0:
        print(f"[INFO] Still discovering tests... (frame {frame})")
else:
    print(f"[WARNING] Test discovery did not complete within {MAX_FRAMES} frames")
    print("[WARNING] This may indicate a test module with blocking code at import time")

# Show the Test Runner window to verify it can display
ui.Workspace.show_window("Test Runner", True)
for _ in range(10):
    kit.update()

# Restore the original exception handler
loop.set_exception_handler(original_handler)

# Print results
print(f"[INFO] Total tests discovered: {discovered_test_count}")

# Check minimum test count
MIN_EXPECTED_TESTS = 1000
if discovered_test_count < MIN_EXPECTED_TESTS:
    print("\n" + "=" * 80, file=sys.stderr)
    print(
        f"TEST FAILED: Expected at least {MIN_EXPECTED_TESTS} tests, but only found {discovered_test_count}",
        file=sys.stderr,
    )
    print("=" * 80 + "\n", file=sys.stderr)
    kit.close()
    sys.exit(1)

# Check if any exceptions were captured
if captured_exceptions:
    print("\n" + "=" * 80, file=sys.stderr)
    print("TEST FAILED: Asyncio task exceptions occurred", file=sys.stderr)
    print("=" * 80, file=sys.stderr)
    for i, (msg, exc) in enumerate(captured_exceptions, 1):
        print(f"\nException {i}: {msg}", file=sys.stderr)
        if exc:
            print(f"  {type(exc).__name__}: {exc}", file=sys.stderr)
    print("=" * 80 + "\n", file=sys.stderr)
    kit.close()
    sys.exit(1)

print("TEST PASSED: No asyncio task exceptions occurred")
kit.close()
