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

"""Simulate a cube falling onto a ground plane."""

from __future__ import annotations

import isaacsim.physics_engines.ovphysx  # noqa: F401
from isaacsim.foundation.objects import Cube, PhysicsScene, Stage
from isaacsim.foundation.prims import ColliderBody, GroundPlane, RigidBody
from isaacsim.physics.entities import RigidBodyEntity
from isaacsim.physics.manager import PhysicsManager

_SIMULATION_STEPS = 120


def _author_stage() -> str:
    """Author the scene with Foundation and return its USDA text.

    Returns:
        Flattened USDA representation.
    """
    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")

        GroundPlane("/World/Ground", sizes=10.0)
        Cube("/World/Cube", sizes=1.0).set_local_poses(translations=[0.0, 0.0, 2.0])

        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
        ColliderBody("/World/Cube")
        RigidBody("/World/Cube")
        return stage.export_stage_to_string()
    finally:
        stage.close_stage()


def main() -> tuple[float, float]:
    """Author and simulate a falling cube.

    Returns:
        Cube heights before and after the simulation.
    """
    stage = None
    physics_manager = None

    try:
        stage = Stage("ovstage").import_stage_from_string(_author_stage(), make_default=False)
        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")

        time_step = 1.0 / 60.0
        physics_manager.setup(dt=time_step)
        if not physics_manager.initialize(stage.get_stage_ptr(), stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        cube = RigidBodyEntity("ovphysx", "/World/Cube")

        positions, _ = cube.get_world_poses()
        initial_height = float(positions.numpy()[0, 2])
        physics_manager.step(steps=_SIMULATION_STEPS)
        positions, _ = cube.get_world_poses()
        final_height = float(positions.numpy()[0, 2])

        print(f"Cube height: {initial_height:.3f} -> {final_height:.3f}.")
        return initial_height, final_height
    finally:
        if physics_manager is not None and physics_manager.is_initialized():
            physics_manager.invalidate()
        if stage is not None:
            stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
