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

"""Verifies SimulationApp configuration for deferred stage creation, extra Kit arguments, asset-root settings, and clean shutdown without an unsaved-stage prompt."""

import os
import sys
from typing import Any

from isaacsim import SimulationApp

# Create single SimulationApp instance with combined configuration
simulation_app = SimulationApp(
    {"create_new_stage": False, "extra_args": ["--/app/extra/arg=1", "--/app/some/other/arg=2"]}
)

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app


def test_createstage_config(kit: Any) -> None:
    """Test app startup without creating new stage.

    Args:
        kit: The SimulationApp instance to test.
    """
    print("\n[TEST 1] Testing create_new_stage configuration...")

    for i in range(100):
        kit.update()

    omni.kit.app.get_app().print_and_log("Config: No empty stage was created")

    if omni.usd.get_context().get_stage() is not None:
        print("[fatal] Stage is not None", flush=True)
        sys.exit(1)
    stage_utils.create_new_stage()

    if omni.usd.get_context().get_stage() is None:
        print("[fatal] Stage is None", flush=True)
        sys.exit(1)

    for i in range(100):
        kit.update()

    stage_utils.close_stage()

    print("[TEST 1] PASSED - create_new_stage configuration works correctly")


def test_extra_args(kit: Any) -> None:
    """Test passing extra arguments to SimulationApp.

    Args:
        kit: The SimulationApp instance to test.
    """
    print("\n[TEST 2] Testing extra_args configuration...")

    kit.update()

    server_check = carb.settings.get_settings().get_as_string("/persistent/isaac/asset_root/default")

    env_asset_root = os.environ.get("ISAACSIM_ASSET_ROOT")
    expected_asset_root = env_asset_root if env_asset_root is not None else "omniverse://ov-test-this-is-working"
    if server_check != expected_asset_root:
        print(f"[fatal] isaac nucleus default setting not {expected_asset_root}, instead: {server_check}", flush=True)
        sys.exit(1)

    arg_1 = carb.settings.get_settings().get_as_int("/app/extra/arg")
    arg_2 = carb.settings.get_settings().get_as_int("/app/some/other/arg")

    if arg_1 != 1:
        print(f"[fatal] /app/extra/arg was not 1 and was {arg_1} instead", flush=True)
        sys.exit(1)

    if arg_2 != 2:
        print(f"[fatal] /app/some/other/arg was not 2 and was {arg_2} instead", flush=True)
        sys.exit(1)

    print("[TEST 2] PASSED - extra_args configuration works correctly")


def test_unsaved_on_exit(kit: Any) -> None:
    """Test that app exits cleanly without prompting for unsaved changes.

    Args:
        kit: The SimulationApp instance to test.
    """
    print("\n[TEST 3] Testing unsaved changes on exit...")

    # Create a new stage for this test
    stage_utils.create_new_stage()

    from isaacsim.core.experimental.objects import GroundPlane

    GroundPlane("/World/ground_plane")

    frame_idx = 0
    while kit.is_running():
        kit.update()
        # we should exit this loop before we hit frame 200 unless we are stuck on an exit screen
        if frame_idx >= 200:
            print("[fatal] App is stuck on exit screen (frame >= 200)")
            sys.exit(1)
        # try exiting, it should exit unless a save file dialog shows up.
        if frame_idx == 100:
            omni.kit.app.get_app().post_quit()
        frame_idx += 1

    print("[TEST 3] PASSED - App exits cleanly without save dialog")


# Run all configuration tests
print("=" * 60)
print("Running SimulationApp Configuration Tests")
print("=" * 60)

try:
    test_createstage_config(simulation_app)
    test_extra_args(simulation_app)
    test_unsaved_on_exit(simulation_app)

    print("\n" + "=" * 60)
    print("ALL TESTS PASSED")
    print("=" * 60)

except Exception as e:
    print(f"\n[fatal] Test suite failed with exception: {e}")
    import traceback

    traceback.print_exc()
    sys.exit(1)
finally:
    simulation_app.close()  # Cleanup application
