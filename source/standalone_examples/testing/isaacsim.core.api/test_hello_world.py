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

"""Test hello world task with cloned sphere and cuboid environments."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
from abc import abstractmethod

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.cloner import GridCloner
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.prims import RigidPrim

torch = import_module("torch")

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


class HelloWorld(BaseTask):
    """Base task that clones objects across multiple environments."""

    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """Initialize the HelloWorld task with environment configuration."""
        BaseTask.__init__(self, name=name, offset=offset)

        self._num_envs = num_envs
        self._env_spacing = env_spacing

        self._cloner = GridCloner(self._env_spacing)

        return

    def set_up_scene(self, scene) -> None:
        """Set up the simulation scene with cloned objects.

        Args:
            scene: The scene to populate with objects and ground plane.
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        task_object = self.set_object()
        prim_paths = self._cloner.generate_paths("/World/object", self._num_envs)
        self._cloner.clone(
            source_prim_path=task_object.prim_path,
            prim_paths=prim_paths,
            position_offsets=np.array([[0, 0, 1.0]] * self._num_envs),
        )
        self._object = RigidPrim(prim_paths_expr=f"/World/object_[0-{self._num_envs-1}]", name="object_view")
        scene.add(self._object)

        return

    @abstractmethod
    def set_object(self):
        """Create and return the object to clone across environments."""
        raise NotImplementedError

    def get_observations(self) -> dict:
        """Get current observations from the simulation.

        Returns:
            Dictionary containing object positions and velocities.
        """
        object_positions, _ = self._object.get_world_poses()
        object_velocities = self._object.get_velocities()

        observations = {self._object.name: {"positions": object_positions, "velocities": object_velocities}}
        return observations

    def calculate_metrics(self) -> None:
        """Calculate task metrics for all environments."""
        return torch.zeros(self._num_envs, device=self._device)

    def is_done(self) -> None:
        """Check if the task is completed for each environment."""
        return torch.zeros(self._num_envs, device=self._device)


class HelloWorldSphere(HelloWorld):
    """HelloWorld task using dynamic sphere objects."""

    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """Initialize the HelloWorldSphere task with sphere objects."""
        super().__init__(name=name, num_envs=num_envs, env_spacing=env_spacing, offset=offset)

    def set_object(self):
        """Create and return a dynamic sphere object."""
        radius = 0.1
        density = 1000.0

        return DynamicSphere(prim_path="/World/object_0", name="object_0", radius=radius, mass=None, density=density)


class HelloWorldCuboid(HelloWorld):
    """HelloWorld task using dynamic cuboid objects."""

    def __init__(self, name, num_envs, env_spacing, offset=None) -> None:
        """Initialize the HelloWorldCuboid task with cuboid objects."""
        super().__init__(name=name, num_envs=num_envs, env_spacing=env_spacing, offset=offset)

    def set_object(self):
        """Create and return a dynamic cuboid object."""
        size = np.array([0.2, 0.2, 0.2])
        density = 1000.0

        return DynamicCuboid(
            prim_path="/World/object_0", name="object_0", size=1.0, scale=size, mass=None, density=density
        )


num_envs = 10
env_spacing = 1
physicsscene_path = "/physicsScene"

my_world = World(stage_units_in_meters=1.0, physics_prim_path=physicsscene_path, backend="torch", device="cuda:0")
my_task = HelloWorldSphere(name="hello_world", num_envs=num_envs, env_spacing=env_spacing)
my_world.add_task(my_task)
my_world.reset()

reset_needed = False
while simulation_app.is_running():
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        # deal with sim re-initialization after restarting sim
        if reset_needed:
            # initialize simulation views
            my_world.reset(soft=True)
            reset_needed = False
        observations = my_world.get_observations()

    my_world.step(render=True)
    if args.test is True:
        break

simulation_app.close()
