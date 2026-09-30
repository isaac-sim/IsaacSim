# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Visualize the falling-cube simulation with the OVGL debug viewport."""

from __future__ import annotations

import os
import time

import isaacsim.physics_engines.ovphysx  # noqa: F401
from isaacsim.foundation.objects import Cube, DistantLight, PhysicsScene, Stage
from isaacsim.foundation.prims import ColliderBody, GroundPlane, RigidBody
from isaacsim.ovgl_viewport.debug import Camera, Viewport, author_viewport
from isaacsim.physics.entities import RigidBodyEntity
from isaacsim.physics.manager import PhysicsManager

_PHYSICS_DT = 1.0 / 60.0
_SIMULATION_STEPS = 120
_MAX_STEPS_PER_FRAME = 15
_MAX_FRAME_TIME = _MAX_STEPS_PER_FRAME * _PHYSICS_DT


def _author_stage() -> str:
    """Author the simulated and rendered scene as neutral USDA text.

    Returns:
        USDA representation of the authored root layer.
    """
    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")

        GroundPlane("/World/Ground", sizes=10.0)
        Cube("/World/Cube", sizes=1.0).set_local_poses(translations=[0.0, 0.0, 2.0])

        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
        ColliderBody("/World/Cube")
        RigidBody("/World/Cube")

        light = DistantLight("/World/KeyLight")
        light.set_intensities([[3000.0]])
        light.set_colors([[1.0, 0.92, 0.78]])

        author_viewport(stage)
        return stage.export_stage_to_string()
    finally:
        stage.close_stage()


def main() -> None:
    """Simulate and render the falling cube in an interactive viewport."""
    os.environ.setdefault("OVGL_SS", "1")

    stage = None
    physics_manager = None
    try:
        stage = Stage("ovstage").import_stage_from_string(_author_stage())
        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        physics_manager.setup(dt=_PHYSICS_DT)
        if not physics_manager.initialize(stage.get_stage_ptr(), stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        simulated_cube = RigidBodyEntity("ovphysx", "/World/Cube")

        with Viewport(
            stage,
            title="Falling Cube Simulation",
            camera=Camera(target=(0.0, 0.0, 0.75), distance=7.0),
        ) as viewport:
            if not viewport.poll_events():
                raise RuntimeError("OVGL viewport closed before its initial render.")
            frame = viewport.render()

            print("Dropping the cube now.")
            positions, _ = simulated_cube.get_world_poses()
            initial_height = float(positions.numpy()[0, 2])
            final_height = initial_height
            simulation_step = 0
            last_frame_time = time.monotonic()
            accumulated_simulation_time = 0.0

            while viewport.poll_events():
                current_time = time.monotonic()
                accumulated_simulation_time += min(current_time - last_frame_time, _MAX_FRAME_TIME)
                last_frame_time = current_time
                step_count = min(
                    int(accumulated_simulation_time / _PHYSICS_DT),
                    _MAX_STEPS_PER_FRAME,
                    _SIMULATION_STEPS - simulation_step,
                )

                if step_count:
                    completed_steps = physics_manager.step(steps=step_count)
                    simulation_step += completed_steps
                    accumulated_simulation_time -= completed_steps * _PHYSICS_DT
                    if not physics_manager.publish_transforms_to_stage():
                        raise RuntimeError("Physics transform publication failed.")
                    positions, _ = simulated_cube.get_world_poses()
                    final_height = float(positions.numpy()[0, 2])

                frame = viewport.render()
                if simulation_step == _SIMULATION_STEPS:
                    break

            print(f"Cube height: {initial_height:.3f} -> {final_height:.3f}.")
            print(
                "Controls: drag with the left mouse button to look, hold WASD to move, hold Q/E for down/up, use the "
                "wheel to dolly, press R to reset, and press Escape to quit."
            )
            while viewport.poll_events():
                frame = viewport.render()
            print(f"Viewport rendered {frame.frame_number} frame(s).")
    finally:
        try:
            if physics_manager is not None and physics_manager.is_initialized():
                physics_manager.invalidate()
        finally:
            if stage is not None:
                stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
