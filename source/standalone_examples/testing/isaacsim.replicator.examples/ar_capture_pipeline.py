# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test AR capture pipeline with replicator orchestrator stepping."""

import argparse
import sys

import numpy as np

# Tolerance for position comparisons (in meters)
POSITION_TOLERANCE = 0.006

parser = argparse.ArgumentParser()
parser.add_argument(
    "--gpu_dynamics", action="store_true", default=False, help="Simulation context with GPU dynamics (device='cuda')"
)
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import SimulationContext
from isaacsim.core.api.objects import DynamicCuboid

EXPECTED_LOCATIONS = {
    "initial": (0, 0, 3),
    "reset": (0, 0, 2.9972751140594482),
    "stop": (0, 0, 3),
    "frame_0": (0, 0, 3),
    "frame_1": (0, 0, 2.942775249481201),
    "frame_2": (0, 0, 2.7874503135681152),
    "frame_3": (0, 0, 2.534025192260742),
}

# Create the environment
omni.usd.get_context().new_stage()
dynamic_cuboid = DynamicCuboid(prim_path="/World/Cube", name="cube", position=(0, 0, 3))
dynamic_cuboid_prim = dynamic_cuboid.prim
timeline = omni.timeline.get_timeline_interface()

use_gpu_dynamics = args.gpu_dynamics
if use_gpu_dynamics:
    print(f"Creating simulation context with GPU dynamics")
    simulation_context = SimulationContext(stage_units_in_meters=1.0, set_defaults=False, device="cuda")
else:
    print(f"Creating simulation context with default (CPU) dynamics")
    simulation_context = SimulationContext(stage_units_in_meters=1.0, set_defaults=False)


# AR settings
extension_manager = omni.kit.app.get_app().get_extension_manager()
extension_manager.set_extension_enabled_immediate("omni.physx.fabric", False)
carb.settings.get_settings().set_bool("/physics/updateToUsd", True)
carb.settings.get_settings().set_bool("/physics/updateParticlesToUsd", True)
carb.settings.get_settings().set_bool("/physics/updateVelocitiesToUsd", True)
carb.settings.get_settings().set_bool("/physics/updateForceSensorsToUsd", True)
carb.settings.get_settings().set_bool("/physics/outputVelocitiesLocalSpace", True)
carb.settings.get_settings().set_bool("/physics/suppressReadback", False)


# Initial world state
location = dynamic_cuboid_prim.GetAttribute("xformOp:translate").Get()
print(f"Initial state:")
print(f"  timeline time: {timeline.get_current_time():.4f}, is_playing: {timeline.is_playing()}, location: {location}")
passed = np.allclose(location, EXPECTED_LOCATIONS["initial"], atol=POSITION_TOLERANCE)
diff = tuple(float(x) for x in (np.array(location) - np.array(EXPECTED_LOCATIONS["initial"])))
if passed:
    print(f"[PASS] Initial location is as expected: {location}, diff: {diff}")
else:
    print(f"[FAIL] Initial location is not as expected: {location} vs {EXPECTED_LOCATIONS['initial']}, diff: {diff}")
if not passed:
    sys.exit(1)

# AR pre-capture simulation_context.reset()
simulation_context.reset()
location = dynamic_cuboid_prim.GetAttribute("xformOp:translate").Get()
print(f"After simulation_context.reset():")
print(f"  timeline time: {timeline.get_current_time():.4f}, is_playing: {timeline.is_playing()}, location: {location}")
passed = np.allclose(location, EXPECTED_LOCATIONS["reset"], atol=POSITION_TOLERANCE)
diff = tuple(float(x) for x in (np.array(location) - np.array(EXPECTED_LOCATIONS["reset"])))
if passed:
    print(f"[PASS] Reset location is as expected: {location}, diff: {diff}")
else:
    print(f"[FAIL] Reset location is not as expected: {location} vs {EXPECTED_LOCATIONS['reset']}, diff: {diff}")
if not passed:
    sys.exit(1)

# AR pre-capture simulation_context.stop()
simulation_context.stop()
location = dynamic_cuboid_prim.GetAttribute("xformOp:translate").Get()
print(f"After simulation_context.stop():")
print(f"  timeline time: {timeline.get_current_time():.4f}, is_playing: {timeline.is_playing()}, location: {location}")
passed = np.allclose(location, EXPECTED_LOCATIONS["stop"], atol=POSITION_TOLERANCE)
diff = tuple(float(x) for x in (np.array(location) - np.array(EXPECTED_LOCATIONS["stop"])))
if passed:
    print(f"[PASS] Stop location is as expected: {location}, diff: {diff}")
else:
    print(f"[FAIL] Stop location is not as expected: {location} vs {EXPECTED_LOCATIONS['stop']}, diff: {diff}")
if not passed:
    sys.exit(1)

# Capture frame 0
print(f"Frame 0:")
carb.settings.get_settings().set_bool("/app/player/playSimulations", False)
timeline.play()
timeline.commit_silently()
simulation_app.update()
rep.orchestrator.step()

location = dynamic_cuboid_prim.GetAttribute("xformOp:translate").Get()
print(f"After rep.orchestrator.step():")
print(f"  timeline time: {timeline.get_current_time():.4f}, is_playing: {timeline.is_playing()}, location: {location}")
passed = np.allclose(location, EXPECTED_LOCATIONS["frame_0"], atol=POSITION_TOLERANCE)
diff = tuple(float(x) for x in (np.array(location) - np.array(EXPECTED_LOCATIONS["frame_0"])))
if passed:
    print(f"[PASS] Frame 0 location is as expected: {location}, diff: {diff}")
else:
    print(f"[FAIL] Frame 0 location is not as expected: {location} vs {EXPECTED_LOCATIONS['frame_0']}, diff: {diff}")
if not passed:
    sys.exit(1)


# Capture frame 1-n
num_captures = 3
capture_interval = 5
for i in range(num_captures):
    print(f"Frame {i+1}/{num_captures}:")

    # Make sure the timeline is playing (not needed if we use rep.orchestrator.step(pause_timeline=False))
    if not timeline.is_playing():
        timeline.play()
        timeline.commit_silently()

        # Advance the simulation with the capture_interval
        carb.settings.get_settings().set_bool("/app/player/playSimulations", True)
        for _ in range(capture_interval):
            simulation_context.step(render=False)
        simulation_app.update()
        carb.settings.get_settings().set_bool("/app/player/playSimulations", False)

        # Capture the frame
        rep.orchestrator.step()
        location = dynamic_cuboid_prim.GetAttribute("xformOp:translate").Get()
        print(f"After rep.orchestrator.step():")
        print(
            f"  timeline time: {timeline.get_current_time():.4f}, is_playing: {timeline.is_playing()}, location: {location}"
        )
        passed = np.allclose(location, EXPECTED_LOCATIONS[f"frame_{i+1}"], atol=POSITION_TOLERANCE)
        diff = tuple(float(x) for x in (np.array(location) - np.array(EXPECTED_LOCATIONS[f"frame_{i+1}"])))
        if passed:
            print(f"[PASS] Frame {i+1} location is as expected: {location}, diff: {diff}")
        else:
            print(
                f"[FAIL] Frame {i+1} location is not as expected: {location} vs {EXPECTED_LOCATIONS[f'frame_{i+1}']}, diff: {diff}"
            )
        if not passed:
            sys.exit(1)

simulation_app.close()
