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

"""Benchmark articulation state reads with completed physics and state readback."""

from __future__ import annotations

import argparse
import math
import os

import isaacsim.physics_engines.ovphysx  # noqa: F401
import numpy as np
import warp as wp
from isaacsim.foundation.objects import PhysicsScene, PhysxScene, Stage
from isaacsim.performance.benchmarking import benchmark
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from pxr import Sdf

DEFAULT_ASSET = (
    os.environ.get("ISAACSIM_ASSET_ROOT")
    or "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
).rstrip("/\\") + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"


class Workload:
    """Run one physics frame and read public entity state.

    Args:
        time_step: Simulated seconds per frame.
        physics_device: CPU or GPU dynamics.
        asset: SimpleArticulation USD asset URL.
    """

    def __init__(self, time_step: float = 0.005, physics_device: str = "cpu", asset: str = DEFAULT_ASSET) -> None:
        if not math.isfinite(time_step) or time_step <= 0:
            raise ValueError("time_step must be finite and positive")
        self.time_step = time_step
        self.physics_device = physics_device
        self.stage = None
        self.manager = None
        self.frames = 0
        self.asset = asset

    def setup(self) -> None:
        """Create the scene and initialize the requested dynamics backend."""
        authoring = Stage("openusd").create_stage()
        try:
            PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
            scene = PhysxScene("/World/PhysicsScene")
            scene.set_enabled_gpu_dynamics(self.physics_device == "gpu")
            scene.set_broadphase_types("GPU" if self.physics_device == "gpu" else "MBP")
            stage_text = authoring.export_stage_to_string()
        finally:
            authoring.close_stage()
        layer = Sdf.Layer.CreateAnonymous("joint_state.usda")
        layer.ImportFromString(stage_text)
        robot = Sdf.CreatePrimInLayer(layer, "/Articulation")
        robot.specifier = Sdf.SpecifierDef
        robot.referenceList.prependedItems = [Sdf.Reference(self.asset, Sdf.Path("/Articulation"))]
        self.stage = Stage("ovstage").import_stage_from_string(layer.ExportToString(), make_default=False)
        self.manager = PhysicsManager.get_instance()
        if not self.manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        self.manager.setup(dt=self.time_step)
        if not self.manager.initialize(self.stage.get_stage_ptr(), self.stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        self.body = ArticulationEntity("ovphysx", "/Articulation")
        count = len(self.body.dof_names)
        targets = np.zeros((1, count), dtype=np.float32)
        for name in ("RevoluteJoint", "PrismaticJoint"):
            targets[0, self.body.dof_names.index(name)] = 0.5
        self.body.set_dof_gains(
            stiffnesses=wp.array(np.zeros((1, count), dtype=np.float32), device="cpu"),
            dampings=wp.array(np.full((1, count), 1e6, dtype=np.float32), device="cpu"),
        )
        self.body.set_dof_velocity_targets(wp.array(targets, device="cpu"))
        self.initial = self.body.get_dof_positions().numpy().copy()

    def run(self) -> None:
        """Finish one physics step and read its state on the host."""
        self.manager.step()
        self.state = (
            self.body.get_dof_positions().numpy(),
            self.body.get_dof_velocities().numpy(),
            self.body.get_dof_projected_joint_forces().numpy(),
        )
        self.frames += 1

    def validate(self) -> bool:
        """Check finite state, actual motion, and completed simulation time."""
        return bool(
            self.frames > 0
            and all(np.isfinite(value).all() for value in self.state)
            and np.linalg.norm(self.state[0] - self.initial) > 1e-7
            and self.manager.get_simulated_physics_steps() == self.frames
        )

    def teardown(self) -> None:
        """Close the physics engine and owned stage."""
        try:
            if self.manager is not None and self.manager.is_initialized():
                self.manager.invalidate()
        finally:
            if self.stage is not None:
                self.stage.close_stage()


def main() -> None:
    """Run the workload and fail when its result is invalid."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--physics-device", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument("--time-step", type=float, default=0.005)
    parser.add_argument("--warmup-frames", type=int, default=100)
    parser.add_argument("--measured-frames", type=int, default=1000)
    args = parser.parse_args()
    parameters = {"time_step": args.time_step, "physics_device": args.physics_device}
    workflow = benchmark.workflow(
        "benchmarks.physics.entities.joint_state",
        metrics=("simulation",),
        simulated_time="time_step",
        warmup_frames=args.warmup_frames,
        measured_frames=args.measured_frames,
        completion="fetched",
        resources=("cpu", "gpu"),
    )(Workload)
    result = benchmark.run(workflow, parameters=parameters, output=args.output)
    if not result["status"]["valid"]:
        raise RuntimeError(result["status"]["errors"])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
