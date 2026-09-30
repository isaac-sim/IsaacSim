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

"""Author a falling cube locally and simulate it through a remote OV SIM server."""

from __future__ import annotations

import argparse

from isaacsim.ovsim.api import make_client


def main(endpoint: str | None = None) -> tuple[float, float]:
    """Author locally, transfer stage data, and simulate the cube remotely.

    Args:
        endpoint: Insecure gRPC endpoint. When omitted, parse ``--endpoint`` from the command line.

    Returns:
        Cube heights before and after the remote simulation.
    """
    if endpoint is None:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--endpoint", required=True, help="Insecure gRPC endpoint, for example 127.0.0.1:50051")
        endpoint = parser.parse_args().endpoint

    local_client = make_client("in-process")
    remote_client = make_client("grpc", {"endpoint": endpoint})
    local_stage_open = False
    remote_stage_open = False

    try:
        if not local_client.control.authoring.create_stage():
            raise RuntimeError("Stage creation failed.")
        local_stage_open = True

        cube = "/World/Cube"
        local_client.control.authoring.define_prim(cube, "Cube")
        local_client.data.write(cube, "size", 1.0)
        local_client.data.write(cube, "position", [0.0, 0.0, 2.0])
        local_client.control.authoring.define_prim(cube, "ColliderBody")
        local_client.control.authoring.define_prim(cube, "RigidBody")

        physics_scene = "/World/PhysicsScene"
        local_client.control.authoring.define_prim(physics_scene, "PhysicsScene")
        local_client.data.write(physics_scene, "physics:gravityDirection", [0.0, 0.0, -1.0])
        local_client.data.write(physics_scene, "physics:gravityMagnitude", 9.81)

        stage_content = local_client.control.authoring.export_stage_to_string()
        if not remote_client.control.authoring.import_stage_from_string(stage_content):
            raise RuntimeError("Remote simulation creation failed.")
        remote_stage_open = True
        remote_client.control.simulation.initialize()

        initial_height = float(remote_client.data.read(cube, "position").numpy()[0, 2])
        for _ in range(10):
            remote_client.control.simulation.step()
        final_height = float(remote_client.data.read(cube, "position").numpy()[0, 2])

        print(f"Remote cube height: {initial_height:.3f} -> {final_height:.3f}.")
        return initial_height, final_height
    finally:
        if remote_stage_open:
            remote_client.control.authoring.close_stage()
        if local_stage_open:
            local_client.control.authoring.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
