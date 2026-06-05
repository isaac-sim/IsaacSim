# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test articulation root orientation, determinism, and tensor API handles."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
import sys

import numpy as np
import omni.physics.core
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, is_stage_loading
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path


def setup_clean_stage() -> None:
    """Create a fresh stage for each test body."""
    World.clear_instance()
    create_new_stage()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()


def test_articulation_root():
    """Test articulation root orientation bug fix."""
    print("Running test_articulation_root...")
    setup_clean_stage()

    asset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/orientation_bug.usd")
    my_world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(usd_path=asset_path, prim_path="/World")
    articulated = Articulation("/World/microwave")
    my_world.scene.add(articulated)
    my_world.reset()

    for i in range(3):
        my_world.step(render=True)

    if not (np.isclose(articulated.get_world_poses()[1], [-0.50, -0.49, 0.49, 0.50], atol=1e-02)).all():
        print(
            f"[fatal] Articulation is not using the correct default state due to a mismatch in the ArticulationRoot representation"
        )
        my_world.clear_instance()
        return False

    my_world.clear_instance()
    print("[PASS] test_articulation_root")
    return True


def test_articulation_determinism():
    """Test Franka articulation convergence determinism."""
    print("Running test_articulation_determinism...")

    assets_root_path = get_assets_root_path()

    def test_franka_slow_convergence():
        setup_clean_stage()
        robot_prim_path = "/panda"
        add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd", prim_path=robot_prim_path
        )

        # Start Simulation and wait
        my_world = World(stage_units_in_meters=1.0)
        my_world.reset()

        robot = Robot(robot_prim_path)
        robot.initialize()
        robot.get_articulation_controller().set_gains(1e4 * np.ones(9), 1e3 * np.ones(9))
        robot.set_solver_position_iteration_count(64)
        robot.set_solver_velocity_iteration_count(64)
        robot.post_reset()

        my_world.step(render=True)

        timeout = 200

        action = ArticulationAction(
            joint_positions=np.array(
                [
                    -0.40236897393760085,
                    -0.44815597748391767,
                    -0.16028112816211953,
                    -2.4554393933564986,
                    -0.34608791253975374,
                    2.9291361940824485,
                    0.4814803907662416,
                    None,
                    None,
                ]
            )
        )

        robot.get_articulation_controller().apply_action(action)
        for i in range(timeout):
            my_world.step()
            current_positions = robot.get_joint_positions()
            target_positions = action.joint_positions
            # Create a mask for non-None entries
            mask = np.array([pos is not None for pos in target_positions])
            # Compute diff only for non-None entries
            diff = current_positions[mask] - np.array([pos for pos in target_positions if pos is not None])
            if np.linalg.norm(diff) < 0.01:
                my_world.clear_instance()
                return i

        my_world.clear_instance()
        return timeout

    frames_to_converge = np.empty(5)
    for i in range(5):
        num_frames = test_franka_slow_convergence()
        frames_to_converge[i] = num_frames

    # Takes the same number of frames to converge every time
    print(f"Over 5 trials, the Franka converged to target in {frames_to_converge} frames.")
    if np.unique(frames_to_converge).shape[0] != 1:
        print(f"[fatal] Non-deterministic test converged in varying number of frames: {frames_to_converge}")
        return False

    # On the develop branch, this test always takes 26 frames to converge
    if frames_to_converge[0] != 27:
        print(f"[fatal] Didn't converge in the right number of frames (expected 27, got {frames_to_converge[0]})")
        return False

    print("[PASS] test_articulation_determinism")
    return True


def test_tensor_api_handles():
    """Test tensor API handles with physics callbacks."""
    print("Running test_tensor_api_handles...")
    setup_clean_stage()

    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
    assets_root_path = get_assets_root_path()
    asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
    articulated_system_1 = my_world.scene.add(Robot(prim_path="/World/Franka", name="my_franka_1"))

    velocities = []

    def step_callback_1(step_size, context):
        b = articulated_system_1.get_joint_velocities()
        velocities.append(b)

    sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=False, order=0, on_update=step_callback_1
    )

    my_world.reset()
    try:
        for i in range(5):
            my_world.step(render=False)
    except Exception as e:
        print(f"[fatal] Exception during tensor API handle test: {e}")
        my_world.clear_instance()
        return False
    finally:
        sub = None

    if len(velocities) == 0:
        print("[fatal] No velocities captured")
        my_world.clear_instance()
        return False

    if velocities[-1] is None:
        print("[fatal] Final joint velocities returned None")
        my_world.clear_instance()
        return False

    my_world.clear_instance()
    print("[PASS] test_tensor_api_handles")
    return True


# Run all tests
all_passed = True

if not test_articulation_root():
    all_passed = False

if not test_articulation_determinism():
    all_passed = False

if not test_tensor_api_handles():
    all_passed = False

simulation_app.close()

if not all_passed:
    sys.exit(1)
