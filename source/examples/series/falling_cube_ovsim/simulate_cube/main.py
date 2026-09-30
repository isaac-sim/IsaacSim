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
from isaacsim.ovsim.api import make_client


def main() -> tuple[float, float]:
    """Author and simulate a falling cube.

    Returns:
        Cube heights before and after the simulation.
    """
    client = make_client("in-process")
    simulation_initialized = False
    stage_open = False

    try:
        if not client.control.authoring.create_stage():
            raise RuntimeError("Stage creation failed.")
        stage_open = True

        cube = "/World/Cube"
        client.control.authoring.define_prim(cube, "Cube")
        client.data.write(cube, "size", 1.0)
        client.data.write(cube, "position", [0.0, 0.0, 2.0])
        client.control.authoring.define_prim(cube, "ColliderBody")
        client.control.authoring.define_prim(cube, "RigidBody")

        ground_plane = "/World/GroundPlane"
        client.control.authoring.define_prim(ground_plane, "GroundPlane")

        physics_scene = "/World/PhysicsScene"
        client.control.authoring.define_prim(physics_scene, "PhysicsScene")
        client.data.write(physics_scene, "physics:gravityDirection", [0.0, 0.0, -1.0])
        client.data.write(physics_scene, "physics:gravityMagnitude", 9.81)

        client.control.simulation.set_parameter("physics", "physics-engine", "ovphysx")
        client.control.simulation.initialize()
        simulation_initialized = True

        initial_height = float(client.data.read(cube, "position").numpy()[0, 2])
        for _ in range(10):
            client.control.simulation.step()
        final_height = float(client.data.read(cube, "position").numpy()[0, 2])

        print(f"Cube height: {initial_height:.3f} -> {final_height:.3f}.")
        return initial_height, final_height
    finally:
        if simulation_initialized:
            client.control.simulation.invalidate()
        if stage_open:
            client.control.authoring.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
