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

"""Interactive example demonstrating robotic bin stacking using a UR10 robot with autonomous bin handling on a conveyor system."""

import random

import isaacsim.cortex.framework.math_util as math_util
import numpy as np
import omni
from isaacsim.core.api.objects.capsule import VisualCapsule
from isaacsim.core.api.objects.sphere import VisualSphere
from isaacsim.core.api.tasks.base_task import BaseTask
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.behaviors.ur10 import bin_stacking_behavior as behavior
from isaacsim.cortex.examples.cortex.cortex_base import CortexBase
from isaacsim.cortex.framework.cortex_rigid_prim import CortexRigidPrim
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path
from isaacsim.cortex.framework.robot import CortexUr10


class Ur10Assets:
    """Container for asset file paths used in the UR10 bin stacking demonstration.

    This class provides centralized access to USD file paths for all assets required in the UR10 bin stacking
    scenario, including the robot workspace, bins, environment backgrounds, and interactive objects. All paths
    are resolved relative to the Isaac Sim assets root directory.

    The class initializes paths for:
    - UR10 robot table setup with suction gripper
    - Small KLT bins for stacking operations
    - Warehouse environment background
    - Rubik's cube props for manipulation tasks
    """

    def __init__(self) -> None:
        self.assets_root_path = get_assets_root_path()

        self.ur10_table_usd = (
            self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_short_suction.usd"
        )
        self.small_klt_usd = self.assets_root_path + "/Isaac/Props/KLT_Bin/small_KLT.usd"
        self.background_usd = self.assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        self.rubiks_cube_usd = self.assets_root_path + "/Isaac/Props/Rubiks_Cube/rubiks_cube.usd"


def random_bin_spawn_transform() -> tuple[np.ndarray, np.ndarray]:
    """Generate a random spawn transform for a bin on the conveyor.

    Creates a randomized position and orientation for spawning bins. The position has random x-coordinate
    within the conveyor width, with a 50% chance of flipping the bin upside down for varied orientations.

    Returns:
        A tuple containing (position, quaternion) where position is the spawn coordinates and quaternion
        represents the orientation values.
    """
    x = random.uniform(-0.15, 0.15)
    y = 1.5
    z = -0.15
    position = np.array([x, y, z])

    z = random.random() * 0.02 - 0.01
    w = random.random() * 0.02 - 0.01
    norm = np.sqrt(z**2 + w**2)
    quat = math_util.Quaternion([w / norm, 0, 0, z / norm])
    if random.random() > 0.5:
        print("<flip>")
        # flip the bin so it's upside down
        quat = quat * math_util.Quaternion([0, 0, 1, 0])
    else:
        print("<no flip>")

    return position, quat.vals


class BinStackingTask(BaseTask):
    """A robotic task for bin stacking automation using a UR10 robot.

    This task manages the dynamic spawning and manipulation of bins on a conveyor system. It continuously
    spawns bins with random orientations and positions, monitors their movement through the conveyor system,
    and coordinates with the robot's behavior system for stacking operations. The task handles bin lifecycle
    management including creation, tracking, and cleanup of bin objects in the simulation environment.

    The task integrates with Isaac Sim's Cortex framework to provide autonomous bin handling capabilities.
    Bins are spawned with random orientations (including potential upside-down configurations) and initial
    velocities to simulate realistic conveyor belt behavior. The system tracks bins as they move through
    different zones and triggers new bin spawning when previous bins have been processed.

    Args:
        env_path: Path to the environment in the USD stage where bins will be spawned.
        assets: Asset manager containing USD file paths for bin models and other required assets.
    """

    def __init__(self, env_path: str, assets: object) -> None:
        super().__init__("bin_stacking")
        self.assets = assets

        self.env_path = env_path
        self.bins = []
        self.stashed_bins = []
        self.on_conveyor = None

    def _spawn_bin(self, rigid_bin: CortexRigidPrim) -> None:
        """Spawns a bin at a random position and orientation on the conveyor.

        Sets the bin's world pose with random position and orientation, applies initial velocity,
        and makes it visible in the scene.

        Args:
            rigid_bin: The rigid bin prim to spawn on the conveyor.
        """
        x, q = random_bin_spawn_transform()
        rigid_bin.set_world_pose(position=x, orientation=q)
        rigid_bin.set_linear_velocity(np.array([0, -0.30, 0]))
        rigid_bin.set_visibility(True)

    def post_reset(self) -> None:
        """Resets the task state after a reset event.

        Removes all existing bins from the scene, clears the bins list, and resets the conveyor state.
        """
        if len(self.bins) > 0:
            for rigid_bin in self.bins:
                self.scene.remove_object(rigid_bin.name)
            self.bins.clear()

        self.on_conveyor = None

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Spawn a new randomly oriented bin if the previous bin has been placed.

        Args:
            time_step_index: The current simulation time step index.
            simulation_time: The current simulation time in seconds.
        """
        spawn_new = False
        if self.on_conveyor is None:
            spawn_new = True
        else:
            (x, y, z), _ = self.on_conveyor.get_world_pose()
            is_on_conveyor = y > 0.0 and x > -0.4 and x < 0.4
            if not is_on_conveyor:
                spawn_new = True

        if spawn_new:
            name = f"bin_{len(self.bins)}"
            prim_path = self.env_path + f"/bins/{name}"
            add_reference_to_stage(usd_path=self.assets.small_klt_usd, prim_path=prim_path)
            self.on_conveyor = self.scene.add(CortexRigidPrim(name=name, prim_path=prim_path))

            self._spawn_bin(self.on_conveyor)
            self.bins.append(self.on_conveyor)

    def world_cleanup(self) -> None:
        """Cleans up all task-related objects and resets internal state.

        Clears the bins list, stashed bins list, and resets the conveyor reference.
        """
        self.bins = []
        self.stashed_bins = []
        self.on_conveyor = None
        return


class BinStacking(CortexBase):
    """Interactive example demonstrating robotic bin stacking using a UR10 robot.

    This class sets up a complete bin stacking simulation where a UR10 robot autonomously picks up bins
    from a conveyor belt and stacks them. The simulation includes:

    - A UR10 robot with suction gripper capabilities
    - Dynamic bin spawning on a moving conveyor belt with random orientations
    - Intelligent behavior system for bin detection, pickup, and stacking
    - Collision avoidance with registered obstacles
    - Real-time decision monitoring and diagnostics

    The robot uses Cortex behavior trees to autonomously:

    1. Detect bins arriving on the conveyor belt
    2. Plan collision-free paths to approach bins
    3. Handle bins in various orientations (including upside-down)
    4. Pick up bins using suction gripper
    5. Navigate to stacking location while avoiding obstacles
    6. Stack bins in an organized manner

    The simulation continuously spawns new bins with random positions and orientations, creating an
    ongoing stacking task. Each bin may be oriented normally or flipped upside-down, requiring the
    robot to adapt its approach strategy.

    Obstacles are registered around the workspace including navigation barriers, flip station spheres,
    and dome constraints to ensure safe robot operation.

    Args:
        monitor_fn: Optional callback function for receiving behavior diagnostics and decision stack
            updates during simulation execution.
    """

    def __init__(self, monitor_fn: callable = None) -> None:
        super().__init__()
        self._monitor_fn = monitor_fn
        self.robot = None

    def setup_scene(self) -> None:
        """Sets up the bin stacking scene with the UR10 robot, obstacles, and environment.

        Creates the robot workspace including the UR10 table, background environment, and collision obstacles
        for navigation and manipulation planning.
        """
        world = self.get_world()
        env_path = "/World/Ur10Table"
        ur10_assets = Ur10Assets()
        add_reference_to_stage(usd_path=ur10_assets.ur10_table_usd, prim_path=env_path)
        add_reference_to_stage(usd_path=ur10_assets.background_usd, prim_path="/World/Background")
        background_prim = SingleXFormPrim(
            "/World/Background", position=[10.00, 2.00, -1.18180], orientation=[0.7071, 0, 0, 0.7071]
        )
        self.robot = world.add_robot(CortexUr10(name="robot", prim_path=f"{env_path}/ur10"))

        obs = world.scene.add(
            VisualSphere(
                "/World/Ur10Table/Obstacles/FlipStationSphere",
                name="flip_station_sphere",
                position=np.array([0.73, 0.76, -0.13]),
                radius=0.2,
                visible=False,
            )
        )
        self.robot.register_obstacle(obs)
        obs = world.scene.add(
            VisualSphere(
                "/World/Ur10Table/Obstacles/NavigationDome",
                name="navigation_dome_obs",
                position=[-0.031, -0.018, -1.086],
                radius=1.1,
                visible=False,
            )
        )
        self.robot.register_obstacle(obs)

        az = np.array([1.0, 0.0, -0.3])
        ax = np.array([0.0, 1.0, 0.0])
        ay = np.cross(az, ax)
        R = math_util.pack_R(ax, ay, az)
        quat = math_util.matrix_to_quat(R)
        obs = world.scene.add(
            VisualCapsule(
                "/World/Ur10Table/Obstacles/NavigationBarrier",
                name="navigation_barrier_obs",
                position=[0.471, 0.276, -0.463 - 0.1],
                orientation=quat,
                radius=0.5,
                height=0.9,
                visible=False,
            )
        )
        self.robot.register_obstacle(obs)

        obs = world.scene.add(
            VisualCapsule(
                "/World/Ur10Table/Obstacles/NavigationFlipStation",
                name="navigation_flip_station_obs",
                position=np.array([0.766, 0.755, -0.5]),
                radius=0.5,
                height=0.5,
                visible=False,
            )
        )
        self.robot.register_obstacle(obs)

    async def setup_post_load(self) -> None:
        """Configures the bin stacking task and decision network after scene loading.

        Initializes the BinStackingTask, sets up the scene components, and creates the behavior decision
        network for autonomous bin manipulation operations.
        """
        world = self.get_world()
        env_path = "/World/Ur10Table"
        ur10_assets = Ur10Assets()
        if not self.robot:
            self.robot = world._robots["robot"]
            world._current_tasks.clear()
            world._behaviors.clear()
            world._logical_state_monitors.clear()
        self.task = BinStackingTask(env_path, ur10_assets)
        print(world.scene)
        self.task.set_up_scene(world.scene)
        world.add_task(self.task)
        self.decider_network = behavior.make_decider_network(self.robot, self._on_monitor_update)
        world.add_decider_network(self.decider_network)
        return

    def _on_monitor_update(self, diagnostics: object) -> None:
        """Handles updates from the decision network monitoring system.

        Formats the decision stack information and forwards diagnostics data to the registered monitor
        function for display or logging purposes.

        Args:
            diagnostics: Diagnostic information from the decision network monitoring system.
        """
        decision_stack = ""
        if self.decider_network._decider_state.stack:
            decision_stack = "\n".join(
                [
                    "{0}{1}".format("  " * i, element)
                    for i, element in enumerate(str(i) for i in self.decider_network._decider_state.stack)
                ]
            )

        if self._monitor_fn:
            self._monitor_fn(diagnostics, decision_stack)

    def _on_physics_step(self, step_size: float) -> None:
        """Executes a single physics simulation step.

        Advances the world simulation by one step without rendering to maintain physics calculations
        for the bin stacking scenario.

        Args:
            step_size: The time step size for the physics simulation.
        """
        world = self.get_world()
        world.step(False, False)
        return

    async def on_event_async(self) -> None:
        """Handles asynchronous event processing for the bin stacking scenario.

        Resets the Cortex system, registers the physics step callback, and starts the simulation
        playback for autonomous bin manipulation operations.
        """
        world = self.get_world()
        await omni.kit.app.get_app().next_update_async()
        world.reset_cortex()
        world.add_physics_callback("sim_step", self._on_physics_step)
        await world.play_async()
        return

    async def setup_pre_reset(self) -> None:
        """Prepares the scene for reset by cleaning up physics callbacks.

        Removes the physics step callback to ensure proper cleanup before the simulation reset.
        """
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        return

    def world_cleanup(self) -> None:
        """Performs cleanup operations for the bin stacking world.

        Clears any remaining world state and prepares for scene teardown or reinitialization.
        """
        return
