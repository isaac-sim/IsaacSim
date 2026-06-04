# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Benchmark physics raycast sensor performance in Isaac Sim."""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--num-sensors", type=int, default=1, help="Number of sensors")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend, defaults",
)

args, unknown = parser.parse_known_args()

n_sensor = args.num_sensors
n_frames = args.num_frames

if n_sensor < 1:
    raise SystemExit("--num-sensors must be >= 1")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import math

import numpy as np
import omni.kit.app
import omni.timeline
from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat

enable_extension("isaacsim.benchmark.services")
enable_extension("isaacsim.sensors.experimental.physics")
from isaacsim.benchmark.services import BaseIsaacBenchmark
from isaacsim.sensors.experimental.physics import Raycast

# Lidar geometry parameters (equivalent to legacy PhysX lidar params:
# horizontal_fov=360, vertical_fov=30, horizontal_resolution=0.4, vertical_resolution=4).
HORIZONTAL_FOV = 360.0
VERTICAL_FOV = 30.0
HORIZONTAL_RESOLUTION = 0.4
VERTICAL_RESOLUTION = 4.0


def _build_lidar_rays() -> tuple[np.ndarray, np.ndarray]:
    """Build a solid-state lidar ray pattern as Nx3 origins/directions arrays."""
    h_count = int(HORIZONTAL_FOV / HORIZONTAL_RESOLUTION)
    v_count = int(VERTICAL_FOV / VERTICAL_RESOLUTION) + 1
    origins = np.zeros((h_count * v_count, 3), dtype=np.float32)
    directions = np.zeros((h_count * v_count, 3), dtype=np.float32)
    for vi in range(v_count):
        v_angle = math.radians(-VERTICAL_FOV / 2 + VERTICAL_FOV * vi / max(v_count - 1, 1))
        for hi in range(h_count):
            # Azimuth divides by h_count (not h_count - 1) so the 360° sweep wraps
            # without duplicating the endpoint; elevation uses (v_count - 1) since
            # it spans an open arc.
            h_angle = math.radians(-HORIZONTAL_FOV / 2 + HORIZONTAL_FOV * hi / h_count)
            idx = vi * h_count + hi
            directions[idx] = (
                math.cos(v_angle) * math.cos(h_angle),
                math.cos(v_angle) * math.sin(h_angle),
                math.sin(v_angle),
            )
    return origins, directions


# ----------------------------------------------------------------------
# Create benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_physx_lidar",
    workflow_metadata={"metadata": [{"name": "num_lidars", "data": n_sensor}]},
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
PhysicsContext(physics_dt=1.0 / 60.0)

ray_origins, ray_directions = _build_lidar_rays()

# Sensor translation is identical for every sensor (set for full_warehouse.usd).
sensor_translation = np.array([[-8.0, 13.0, 2.0]], dtype=np.float32)

for i in range(n_sensor):
    lidar_path = f"/World/PhysxLidar_{i}"
    sensor_orientation = euler_angles_to_quat([90.0, 0.0, 90.0 + (i * 360.0 / n_sensor)], degrees=True).reshape(1, 4)
    Raycast.create(
        lidar_path,
        translations=sensor_translation,
        orientations=sensor_orientation,
        min_range=0.4,
        max_range=100.0,
        ray_origins=ray_origins,
        ray_directions=ray_directions,
    )

    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.set_phase("benchmark", warmup_frames=15)

timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()

simulation_app.close()
