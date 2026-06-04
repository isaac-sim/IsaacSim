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

"""Benchmark script for RTX Lidar with ROS2 PointCloud2 metadata publishing.

This benchmark creates multiple RTX Lidar sensors, each with their own ROS2 OmniGraph
that publishes PointCloud2 messages with configurable metadata fields.
"""

import argparse

# Available metadata options that can be enabled
METADATA_OPTIONS = [
    "Intensity",
    "Timestamp",
    "EmitterId",
    "ChannelId",
    "MaterialId",
    "TickId",
    "HitNormal",
    "Velocity",
    "ObjectId",
    "EchoId",
    "TickState",
]

parser = argparse.ArgumentParser(description="Benchmark RTX Lidar with ROS2 PointCloud2 metadata")
parser.add_argument("--num-sensors", type=int, default=1, help="Number of sensors")
parser.add_argument("--num-gpus", type=int, default=None, help="Number of GPUs on machine")
parser.add_argument(
    "--lidar-type",
    type=str,
    default="Rotary",
    choices=["Rotary", "Solid_State"],
    help="Type of lidar to create",
)
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument(
    "--backend-type",
    default="OmniPerfKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile", "OmniPerfKPIFile"],
    help="Benchmarking backend type",
)
parser.add_argument(
    "--metadata",
    type=str,
    nargs="+",
    default=["Intensity", "ObjectId"],
    choices=METADATA_OPTIONS,
    help=f"Metadata fields to include in PointCloud2. Choices: {METADATA_OPTIONS}",
)
parser.add_argument("--gpu-frametime", action="store_true", help="Enable GPU frametime measurement")
parser.add_argument("--non-headless", action="store_false", dest="headless", help="Run with GUI")

args, unknown = parser.parse_known_args()

n_sensor = args.num_sensors
n_gpu = args.num_gpus
n_frames = args.num_frames
lidar_type = args.lidar_type
metadata_fields = args.metadata
gpu_frametime = args.gpu_frametime
headless = args.headless

from isaacsim import SimulationApp

extra_args = []
if "ObjectId" in metadata_fields:
    extra_args.append("--/rtx-transient/stableIds/enabled=true")
if "HitNormal" in metadata_fields:
    extra_args.append("--/app/sensors/nv/lidar/publishNormals=true")
simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu, "extra_args": extra_args})

import carb
import numpy as np
import omni
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

enable_extension("isaacsim.benchmark.services")
enable_extension("isaacsim.ros2.bridge")
omni.kit.app.get_app().update()

# Initialize ROS2
import rclpy
from isaacsim.benchmark.services import DEFAULT_RECORDERS, BaseIsaacBenchmark

rclpy.init()

# Create the benchmark
recorders = DEFAULT_RECORDERS + ["gpu_frametime"] if gpu_frametime else DEFAULT_RECORDERS
benchmark = BaseIsaacBenchmark(
    benchmark_name="benchmark_rtx_lidar_ros2_metadata",
    workflow_metadata={
        "metadata": [
            {"name": "num_sensors", "data": n_sensor},
            {"name": "lidar_type", "data": lidar_type},
            {"name": "metadata_fields", "data": ",".join(metadata_fields)},
            {"name": "num_gpus", "data": carb.settings.get_settings().get("/renderer/multiGpu/currentGpuCount")},
        ]
    },
    backend_type=args.backend_type,
    recorders=recorders,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

scene_path = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
print("Stage loaded")

timeline = omni.timeline.get_timeline_interface()

# Set this to true so that we always publish regardless of subscribers
carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/publish_without_verification", True)

# Determine the minimum aux_output_level needed for the requested metadata.
# Levels form a hierarchy: NONE < BASIC < EXTRA < FULL.  Each metadata field
# maps to the minimum GMO AuxType that includes it (see LidarAuxiliaryData in
# the GenericModelOutput docs).
_METADATA_AUX_LEVEL = {
    "Intensity": "NONE",
    "Timestamp": "NONE",
    "EmitterId": "BASIC",
    "ChannelId": "BASIC",
    "TickId": "BASIC",
    "EchoId": "BASIC",
    "TickState": "BASIC",
    "MaterialId": "EXTRA",
    "ObjectId": "EXTRA",
    "HitNormal": "FULL",
    "Velocity": "FULL",
}
_AUX_LEVEL_ORDER = ["NONE", "BASIC", "EXTRA", "FULL"]
aux_output_level = _AUX_LEVEL_ORDER[max(_AUX_LEVEL_ORDER.index(_METADATA_AUX_LEVEL[f]) for f in metadata_fields)]
print(f"Aux output level for metadata {metadata_fields}: {aux_output_level}")

lidars = []
lidar_sensors = []
graphs = []

for i in range(n_sensor):
    lidar_path = f"/World/RtxLidar_{lidar_type}_{i}"
    graph_path = f"/World/ROS2_Lidar_Graph_{i}"

    sensor_translation = np.array([-8 + i * 2.0, 13, 2.0])

    lidar_config = f"Example_{lidar_type}"
    print(f"Creating Lidar {i}: Config={lidar_config}, Path={lidar_path}")

    lidar = Lidar.create(
        lidar_path,
        config=lidar_config,
        translations=sensor_translation,
        aux_output_level=aux_output_level,
    )
    lidars.append(lidar)

    print(lidar.paths[0])

    lidar_sensor = LidarSensor(lidar)
    lidar_sensors.append(lidar_sensor)

    render_product_path = str(lidar_sensor.render_product.GetPath())

    keys = og.Controller.Keys

    tick_node = f"{graph_path}/on_playback_tick"
    context_node = f"{graph_path}/ros2_context"
    sim_frame_node = f"{graph_path}/simulation_frame"
    pcl_config_node = f"{graph_path}/point_cloud_config"
    lidar_helper_node = f"{graph_path}/lidar_helper"

    config_set_values = [(f"{pcl_config_node}.inputs:output{field}", True) for field in metadata_fields]

    helper_set_values = [
        (f"{lidar_helper_node}.inputs:topicName", f"/lidar_{i}/point_cloud"),
        (f"{lidar_helper_node}.inputs:type", "point_cloud"),
        (f"{lidar_helper_node}.inputs:frameId", f"lidar_{i}"),
        (f"{lidar_helper_node}.inputs:renderProductPath", render_product_path),
    ]

    if "ObjectId" in metadata_fields:
        helper_set_values.append((f"{lidar_helper_node}.inputs:enableObjectIdMap", True))

    graph_handle, _, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                (tick_node, "omni.graph.action.OnPlaybackTick"),
                (context_node, "isaacsim.ros2.bridge.ROS2Context"),
                (sim_frame_node, "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                (pcl_config_node, "isaacsim.ros2.bridge.ROS2RtxLidarPointCloudConfig"),
                (lidar_helper_node, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: config_set_values + helper_set_values,
            keys.CONNECT: [
                (f"{tick_node}.outputs:tick", f"{sim_frame_node}.inputs:execIn"),
                (f"{sim_frame_node}.outputs:step", f"{lidar_helper_node}.inputs:execIn"),
                (f"{context_node}.outputs:context", f"{lidar_helper_node}.inputs:context"),
                (f"{pcl_config_node}.outputs:selectedMetadata", f"{lidar_helper_node}.inputs:selectedMetadata"),
            ],
        },
    )
    graphs.append(graph_handle)

print(f"Created {n_sensor} sensors with metadata: {metadata_fields}")

benchmark.store_measurements()

# Perform benchmark
benchmark.set_phase("benchmark", warmup_frames=15)
timeline.play()

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()

# Cleanup
rclpy.shutdown()
simulation_app.close()
