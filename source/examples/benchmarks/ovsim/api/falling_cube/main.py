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

"""Benchmark a completed OV Sim physics step and cube position readback."""

from __future__ import annotations

import argparse

import isaacsim.physics_engines.ovphysx  # noqa: F401
import numpy as np
from isaacsim.foundation.objects import PhysxScene
from isaacsim.ovsim.api import make_client
from isaacsim.performance.benchmarking import benchmark


class FallingCube:
    """One falling cube, with rendering disabled."""

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu") -> None:
        self.time_step = time_step
        self.physics_device = physics_device
        self.local = make_client("in-process")
        self.client = self.local
        self.initialized = False

    def setup(self) -> None:
        """Create the scene and initialize physics and controls."""
        cube = "/World/Cube"
        self.local.control.authoring.create_stage()
        self.local.control.authoring.define_prim(cube, "Cube")
        self.local.data.write(cube, "size", 1.0)
        self.local.data.write(cube, "position", [0.0, 0.0, 2.0])
        self.local.control.authoring.define_prim(cube, "ColliderBody")
        self.local.control.authoring.define_prim(cube, "RigidBody")
        self.local.control.authoring.define_prim("/World/GroundPlane", "GroundPlane")
        scene = "/World/PhysicsScene"
        self.local.control.authoring.define_prim(scene, "PhysicsScene")
        self.local.data.write(scene, "physics:gravityDirection", [0.0, 0.0, -1.0])
        self.local.data.write(scene, "physics:gravityMagnitude", 9.81)
        physics_scene = PhysxScene(scene)
        physics_scene.set_enabled_gpu_dynamics(np.array([self.physics_device == "gpu"]))
        physics_scene.set_broadphase_types("GPU" if self.physics_device == "gpu" else "MBP")
        self.client.control.simulation.set_parameter("physics", "physics-engine", "ovphysx")
        self.client.control.simulation.set_parameter("physics", "time-step", self.time_step)
        self.client.control.simulation.initialize()
        self.initialized = True
        self.initial_height = float(self.client.data.read("/World/Cube", "position").numpy()[0, 2])
        self.height = self.initial_height

    def run(self) -> None:
        """Complete one measured simulation frame."""
        self.client.control.simulation.step()
        self.height = float(self.client.data.read("/World/Cube", "position").numpy()[0, 2])

    def validate(self) -> bool:
        """Check that simulation advanced with finite state."""
        return (
            0.0 < self.height < self.initial_height
            and self.client.control.simulation.get_parameter("physics", "time-step") == self.time_step
        )

    def teardown(self) -> None:
        """Release the simulation resources."""
        if self.initialized:
            self.client.control.simulation.invalidate()
        self.local.control.authoring.close_stage()


def main() -> None:
    """Run the workload and reject invalid results."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", help="Benchmark JSON output path")
    parser.add_argument("--physics-device", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--warmup-frames", type=int, default=100)
    parser.add_argument("--measured-frames", type=int, default=1000)
    args = parser.parse_args()
    workload = benchmark.workflow(
        "benchmarks.ovsim.api.falling_cube",
        metrics=("simulation",),
        resources=("cpu", "gpu"),
        simulated_time="time_step",
        warmup_frames=args.warmup_frames,
        measured_frames=args.measured_frames,
        completion="fetched",
    )(FallingCube)
    result = benchmark.run(
        workload,
        parameters={"time_step": args.time_step, "physics_device": args.physics_device},
        output=args.output,
    )
    if not result["status"]["valid"]:
        raise RuntimeError(result["status"]["errors"])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
