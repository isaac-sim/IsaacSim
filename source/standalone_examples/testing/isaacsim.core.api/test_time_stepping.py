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

"""Verifies SimulationContext physics and rendering step counts, callback dt values, and render-only behavior across several timestep configurations. Also confirms render calls update PhysX state before pose queries."""

import math
import sys
import unittest
from typing import Any

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
if not math.isclose(simulation_context.get_physics_dt(), 1.0 / 60.0):
    print(f"[fatal] Physics dt mismatch: expected {1.0 / 60.0}, got {simulation_context.get_physics_dt()}")
    sys.exit(1)
if not math.isclose(simulation_context.get_rendering_dt(), 1.0 / 60.0):
    print(f"[fatal] Rendering dt mismatch: expected {1.0 / 60.0}, got {simulation_context.get_rendering_dt()}")
    sys.exit(1)
simulation_context.clear_instance()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
if not math.isclose(simulation_context.get_physics_dt(), 1.0 / 60.0):
    print(f"[fatal] Physics dt mismatch: expected {1.0 / 60.0}, got {simulation_context.get_physics_dt()}")
    sys.exit(1)
if not math.isclose(simulation_context.get_rendering_dt(), 1.0 / 60.0):
    print(f"[fatal] Rendering dt mismatch: expected {1.0 / 60.0}, got {simulation_context.get_rendering_dt()}")
    sys.exit(1)
add_reference_to_stage(asset_path, "/Franka")
# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()


class TimeStepTester(unittest.TestCase):
    """Track and verify physics and rendering step counts and dt values."""

    def __init__(self) -> None:
        self.physics_steps = 0
        self.render_steps = 0
        self.physics_dt = 1.0 / 60.0
        self.render_dt = 1.0 / 60.0

    def step_callback(self, step_size: float) -> None:
        """Record a physics step with the given step size."""
        print("simulate with step: ", step_size)
        self.physics_steps = self.physics_steps + 1
        self.physics_dt = step_size

    def render_callback(self, event: Any) -> None:
        """Record a render step with the dt from the event payload."""
        print("update app with step: ", event.payload["dt"])
        self.render_steps = self.render_steps + 1
        self.render_dt = event.payload["dt"]

    def check_steps(self, physics_steps: int, render_steps: int) -> None:
        """Assert that recorded step counts match expected values."""
        if physics_steps != self.physics_steps:
            self.assertAlmostEqual(physics_steps, self.physics_steps)
        if render_steps != self.render_steps:
            self.assertAlmostEqual(render_steps, self.render_steps)

    def check_dt(self, physics_dt: float, render_dt: float) -> None:
        """Assert that recorded dt values match expected values."""
        if physics_dt != self.physics_dt:
            self.assertAlmostEqual(physics_dt, self.physics_dt)
        if render_dt != self.render_dt:
            self.assertAlmostEqual(render_dt, self.render_dt)

    def reset_values(self) -> None:
        """Reset all tracked step counts and dt values to defaults."""
        self.physics_steps = 0
        self.render_steps = 0
        self.physics_dt = 1.0 / 60.0
        self.render_dt = 1.0 / 60.0


tester = TimeStepTester()
simulation_context.add_physics_callback("physics_callback", tester.step_callback)
simulation_context.add_render_callback("render_callback", tester.render_callback)
simulation_context.stop()
simulation_context.play()
tester.reset_values()

print("step physics once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=False)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 0)
tester.reset_values()

print("step physics & rendering once with a step size of 1/60 second, these are the default settings")
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 1)
tester.reset_values()
print("step physics & rendering once with a step size of 1/60 second")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(1, 1)
tester.reset_values()
print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(10, 1)
tester.reset_values()

print("step physics once at 600Hz without rendering")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
simulation_context.step(render=False)
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(1, 0)
tester.reset_values()

print("step physics 10 steps at a 1/600s per step and rendering at 1.0/60s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 600.0, rendering_dt=1.0 / 60.0)
for step in range(10):
    simulation_context.step(render=False)
simulation_context.render()
tester.check_dt(1.0 / 600.0, 1.0 / 60.0)
tester.check_steps(10, 1)
tester.reset_values()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
simulation_context.render()
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(0, 1)
tester.reset_values()

print("render a frame, moving editor timeline forward by 1.0/60s, physics does not simulate")
simulation_context.set_simulation_dt(physics_dt=0.0, rendering_dt=1.0 / 60)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 60.0)
tester.check_steps(0, 1)
tester.reset_values()

print("step physics once 1/60s per step and rendering 10 times at 1.0/600s")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
for step in range(10):
    simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 1.0 / 600.0)
tester.check_steps(1, 10)
tester.reset_values()

print("step physics once 1/60s per step and rendering once at 1.0/600s by explicitly calling step and render")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 600.0)
simulation_context.step(render=False)
simulation_context.render()
tester.check_dt(1.0 / 60.0, 1.0 / 600.0)
tester.check_steps(1, 1)
tester.reset_values()

render_hz = 20
print("step physics once 1/60s per step and rendering once at 1.0/20s by explicitly calling step and render")
simulation_context.set_simulation_dt(physics_dt=1.0 / (render_hz * 5), rendering_dt=1.0 / render_hz)
simulation_context.step(render=True)
tester.check_dt(1.0 / (render_hz * 5), 1.0 / render_hz)
tester.check_steps(5, 1)
tester.reset_values()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=False)
simulation_context.render()

tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(1, 1)
tester.reset_values()

print("step physics once 1/60s per step, rendering a frame does not move editor timeline forward")
simulation_context.set_simulation_dt(physics_dt=1.0 / 60.0, rendering_dt=0.0)
simulation_context.step(render=True)
tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(1, 1)
tester.reset_values()

print("render a new frame with simulation stopped, editor timeline does not move forward")
simulation_context.stop()  # stop calls render, so clear tester after this
tester.reset_values()

simulation_context.render()
tester.check_dt(1.0 / 60.0, 0)
tester.check_steps(0, 1)
tester.reset_values()

print("Test physics update on render call")
# Test from test_rendering.py - verify physics state updates during render
simulation_context.clear_instance()

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.prims import XFormPrim

torch = import_module("torch")

my_world = World(stage_units_in_meters=1.0, device="cuda:0", backend="torch")
cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_1",
        position=torch.tensor([0, 0, 1.0]),
        scale=torch.tensor([0.6, 0.5, 0.2]),
        size=1.0,
        color=torch.tensor([255, 0, 0]),
    )
)
xfrom_cube = XFormPrim("/new_cube_2")
my_world.scene.add_default_ground_plane()
my_world.reset()
for i in range(500):
    my_world.step(render=False)
my_world.render()
if not (xfrom_cube.get_world_poses(usd=False)[0][:, -1].item() < 10e-02):
    print(f"[fatal] PhysX status is not updated in the rendering call")
    simulation_app.close()
    sys.exit(1)

print("cleanup and exit")
my_world.clear_instance()
simulation_app.close()
