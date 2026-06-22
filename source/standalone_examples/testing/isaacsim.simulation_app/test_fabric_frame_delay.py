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

"""Verifies that Fabric world matrices for rigid and articulated prims are synchronized after render across tensor, USD, USDRT, and Fabric backends."""

import os
from typing import Any

from isaacsim import SimulationApp

# Launch Isaac Sim headless with a zero-delay experience so Fabric updates are immediate.
simulation_app = SimulationApp(
    {"headless": True}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.zero_delay.kit'
)

import sys

import carb
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"


def _get_fabric_world_position(prim_path: str) -> np.ndarray:
    # Fabric attributes are exposed through the Fabric backend, not USD.
    with backend_utils.use_backend("fabric"):
        world_matrix = prim_utils.get_prim_attribute_value(prim_path, "omni:fabric:worldMatrix")
    return np.array(world_matrix.ExtractTranslation())


def _assert_fabric_position(prim_path: str, expected: np.ndarray, message: str) -> None:
    position = _get_fabric_world_position(prim_path)
    if not np.isclose(position, expected, atol=0.01).all():
        print(f"[fatal] {message}", position, expected)
        sys.exit(1)


def _run_backend_test(device: str, backend: str, timeline: Any) -> None:
    print(f"Running test with device: {device}, backend: {backend}")
    timeline.stop()
    # Create a fresh stage and configure simulation settings for each backend.
    stage_utils.create_new_stage()
    SimulationManager.set_device(device)
    SimulationManager.set_physics_dt(0.01)
    SimulationManager.set_backend(backend)
    # Spawn a cube and a Franka articulation to validate Fabric data for both rigid and kinematic prims.
    stage_utils.define_prim("/World/Origin1", "Xform")
    cube = Cube(
        "/World/Origin1/cube",
        sizes=1.0,
        positions=[-3.0, 0.0, 0.1],
        scales=[1.0, 2.0, 0.2],
    )
    # Apply collision and rigid body APIs to the cube so PhysX writes into Fabric.
    GeomPrim("/World/Origin1/cube", apply_collision_apis=True)
    RigidPrim("/World/Origin1/cube")
    # Load the Franka robot asset and prepare prim handles.
    stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Franka")
    articulated_system = Articulation("/World/Franka")
    RigidPrim("/World/Franka/panda_link1")
    timeline.play()

    # Ensure PhysX has produced Fabric data at least once before querying.
    SimulationManager.step(steps=1, update_fabric=True)
    # Move the articulation root to validate kinematic updates.
    articulated_system.set_world_poses(positions=[[-10, -10, 0]])
    # Teleport the cube so we can verify Fabric's world matrix sync pre- and post-render.
    position, _ = cube.get_world_poses()
    position = position.numpy()[0]
    position[0] += 3
    cube.set_world_poses(positions=position.reshape(1, 3))

    # A render tick should synchronize PhysX and kinematic updates into Fabric.
    RenderingManager.render()
    _assert_fabric_position(
        "/World/Franka/panda_link1", np.array([-10.0, -10.0, 0.33]), "Kinematic Tree is not updated in fabric"
    )
    # After render, Fabric should now reflect the teleported cube position.
    _assert_fabric_position("/World/Origin1/cube", np.array([0.0, 0.0, 0.1]), "PhysX is not synced with Fabric CPU")


timeline = omni.timeline.get_timeline_interface()
# Run the same validation for GPU/Tensor and CPU backends.
for device, backend in [["cuda:0", "tensor"], ["cpu", "usd"], ["cpu", "usdrt"], ["cpu", "fabric"]]:
    _run_backend_test(device, backend, timeline)

print(f"[PASS] Fabric frame delay test passed")
simulation_app.close()
