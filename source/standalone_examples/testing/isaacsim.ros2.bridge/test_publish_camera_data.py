# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test publishing camera data through ROS2 bridge with timestamp validation."""

import argparse
import sys

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--test-steps", type=int, default=1000)  # Run for a few frames
args, _ = parser.parse_known_args()

import carb
from isaacsim import SimulationApp

BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

CONFIG = {"renderer": "RealTimePathTracing", "headless": True}

# Example ROS2 bridge sample demonstrating the manual loading of stages and manual publishing of images
simulation_app = SimulationApp(CONFIG)
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import nucleus

# Enable ROS2 bridge extension
app_utils.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

stage_utils.set_stage_units(meters_per_unit=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the environment
stage_utils.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

from collections import defaultdict
from threading import Event, Thread

##### Timestamp checker (duplicates and backwards time) and ROS2 subscriber Node
import rclpy
from rclpy.node import Node


class TimestampChecker(Node):
    """Check ROS2 topic timestamps for duplicates and backwards time."""

    def __init__(self):
        super().__init__("timestamp_checker")
        self.topic_timestamps = defaultdict(set)  # topic_name -> set of timestamps
        self.topic_last_timestamp = defaultdict(lambda: None)  # topic_name -> last timestamp
        self.event = Event()
        self.error_detected = False  # Flag to track if any timestamp errors are detected

        self.subscribed_types = {}

    def subscribe_dynamic(self, topic_name, msg_type_str):
        """Subscribe to a topic dynamically by resolving the message type string."""
        if topic_name in self.subscribed_types:
            return
        msg_type = self._import_message_type(msg_type_str)
        if msg_type is None:
            print(f"Could not import type {msg_type_str} for topic {topic_name}")
            return
        self.create_subscription(
            msg_type, topic_name, lambda msg, topic=topic_name: self.check_timestamp(msg, topic), qos_profile=10
        )
        self.subscribed_types[topic_name] = msg_type

    def check_timestamp(self, msg, topic_name):
        """Validate that message timestamps are unique and monotonically increasing."""
        timestamp = getattr(msg, "header", None)
        if timestamp:
            time_val = (timestamp.stamp.sec, timestamp.stamp.nanosec)
            # print(f"Timestamp: {time_val}")
            # Check for duplicate timestamps
            if time_val in self.topic_timestamps[topic_name]:
                print(f"[error] Duplicate timestamp {time_val} detected on topic {topic_name}")
                self.error_detected = True
                return

            # Check for backwards timestamps
            last_timestamp = self.topic_last_timestamp[topic_name]
            if last_timestamp is not None:
                last_sec, last_nanosec = last_timestamp
                current_sec, current_nanosec = time_val

                # Convert to total nanoseconds for easier comparison
                last_total_ns = last_sec * 1_000_000_000 + last_nanosec
                current_total_ns = current_sec * 1_000_000_000 + current_nanosec

                if current_total_ns < last_total_ns:
                    print(
                        f"[error] Backwards timestamp detected on topic {topic_name}: "
                        f"current {time_val} < previous {last_timestamp}"
                    )
                    self.error_detected = True
                    return

            # Update tracking data structures
            self.topic_timestamps[topic_name].add(time_val)
            self.topic_last_timestamp[topic_name] = time_val

    def _import_message_type(self, msg_type_str):
        try:
            parts = msg_type_str.split("/")
            pkg = parts[0]
            submod = parts[1]
            msg = parts[2]
            import importlib

            modname = f"{pkg}.{submod}"
            mod = importlib.import_module(modname)
            return getattr(mod, msg)
        except Exception:
            return None

    def stop(self):
        """Signal the checker to stop spinning."""
        self.event.set()


def run_checker(checker):
    """Spin the timestamp checker node until it is stopped."""
    while rclpy.ok() and not checker.event.is_set():
        rclpy.spin_once(checker, timeout_sec=0.1)


##### END OF TIMESTAMP CHECKER CLASS

###### Camera helper functions for setting up publishers. ########


# Paste functions from the tutorial here
def publish_camera_tf(camera: Camera):
    """Publish TF transforms for a camera prim using an OmniGraph action graph."""
    camera_prim = camera.prim_path

    if not omni.usd.get_context().get_stage().GetPrimAtPath(camera_prim).IsValid():
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        # Generate the camera_frame_id. OmniActionGraph will use the last part of
        # the full camera prim path as the frame name, so we will extract it here
        # and use it for the pointcloud frame_id.
        camera_frame_id = camera_prim.split("/")[-1]

        # Generate an action graph associated with camera TF publishing.
        ros_camera_graph_path = "/CameraTFActionGraph"

        # If a camera graph is not found, create a new one.
        if not omni.usd.get_context().get_stage().GetPrimAtPath(ros_camera_graph_path).IsValid():
            ros_camera_graph, _, _, _ = og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ],
                },
            )

        # Generate 2 nodes associated with each camera: TF from world to ROS camera convention, and world frame.
        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ComputeTF_" + camera_frame_id, "isaacsim.core.nodes.IsaacComputeTransformTree"),
                    ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_" + camera_frame_id + "_world", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_" + camera_frame_id + ".inputs:topicName", "/tf"),
                    # Note if topic_name is changed to something else besides "/tf",
                    # it will not be captured by the ROS tf broadcaster.
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:topicName", "/tf"),
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:childFrameId", camera_frame_id + "_world"),
                    # Static transform from ROS camera convention to world (+Z up, +X forward) convention:
                    ("PublishRawTF_" + camera_frame_id + "_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (
                        ros_camera_graph_path + "/OnTick.outputs:tick",
                        "ComputeTF_" + camera_frame_id + ".inputs:execIn",
                    ),
                    (
                        "ComputeTF_" + camera_frame_id + ".outputs:execOut",
                        "PublishTF_" + camera_frame_id + ".inputs:execIn",
                    ),
                    (
                        "ComputeTF_" + camera_frame_id + ".outputs:parentFrames",
                        "PublishTF_" + camera_frame_id + ".inputs:parentFrames",
                    ),
                    (
                        "ComputeTF_" + camera_frame_id + ".outputs:childFrames",
                        "PublishTF_" + camera_frame_id + ".inputs:childFrames",
                    ),
                    (
                        "ComputeTF_" + camera_frame_id + ".outputs:translations",
                        "PublishTF_" + camera_frame_id + ".inputs:translations",
                    ),
                    (
                        "ComputeTF_" + camera_frame_id + ".outputs:orientations",
                        "PublishTF_" + camera_frame_id + ".inputs:orientations",
                    ),
                    (
                        ros_camera_graph_path + "/OnTick.outputs:tick",
                        "PublishRawTF_" + camera_frame_id + "_world.inputs:execIn",
                    ),
                    (
                        ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                        "PublishTF_" + camera_frame_id + ".inputs:timeStamp",
                    ),
                    (
                        ros_camera_graph_path + "/IsaacClock.outputs:simulationTime",
                        "PublishRawTF_" + camera_frame_id + "_world.inputs:timeStamp",
                    ),
                ],
            },
        )
    except Exception as e:
        print(e)

    # Set target prims on the compute node for USD pose. All other frames are static.
    set_target_prims(
        primPath=ros_camera_graph_path + "/ComputeTF_" + camera_frame_id,
        inputName="inputs:targetPrims",
        targetPrimPaths=[camera_prim],
    )
    return


def publish_camera_info(camera: Camera, freq):
    """Publish camera info messages at the specified frequency."""
    from isaacsim.ros2.core import read_camera_info

    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_camera_info"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info, _ = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    return


def publish_pointcloud_from_depth(camera: Camera, freq):
    """Publish point cloud data generated from depth images at the specified frequency."""
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_pointcloud"  # Set topic name to the camera's name
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    # Note, this pointcloud publisher will convert the Depth image to a pointcloud using the Camera intrinsics.
    # This pointcloud generation method does not support semantic labeled objects.
    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)

    writer = rep.writers.get(rv + "ROS2PublishPointCloud")
    writer.initialize(frameId=frame_id, nodeNamespace=node_namespace, queueSize=queue_size, topicName=topic_name)
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


def publish_depth(camera: Camera, freq):
    """Publish depth image data at the specified frequency."""
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_depth"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace=node_namespace, queueSize=queue_size, topicName=topic_name)
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


def publish_rgb(camera: Camera, freq):
    """Publish RGB image data at the specified frequency."""
    # The following code will link the camera's render product and publish the data to the specified topic name.
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_rgb"
    queue_size = 1
    node_namespace = ""
    frame_id = camera.prim_path.split("/")[-1]  # This matches what the TF tree is publishing.

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(frameId=frame_id, nodeNamespace=node_namespace, queueSize=queue_size, topicName=topic_name)
    writer.attach([render_product])

    # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    return


###################################################################

# Create a Camera prim. Note that the Camera class takes the position and orientation in the world axes convention.
camera = Camera(
    prim_path="/World/floating_camera",
    position=np.array([-3.11, -1.87, 1.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=transform_utils.euler_angles_to_quaternion(np.array([0, 0, 0]), degrees=True).numpy(),
)
camera.initialize()

simulation_app.update()
camera.initialize()

############### Calling Camera publishing functions ###############

# Call the publishers.
# Make sure you pasted in the helper functions above, and uncomment out the following lines before running.

approx_freq = 30
publish_camera_tf(camera)
publish_camera_info(camera, approx_freq)
publish_rgb(camera, approx_freq)
publish_depth(camera, approx_freq)
publish_pointcloud_from_depth(camera, approx_freq)

####################################################################

# Call the subscribers.
rclpy.init()
checker = TimestampChecker()

topic_type_pairs = [
    ("/camera_pointcloud", "sensor_msgs/msg/PointCloud2"),
    ("/camera_depth", "sensor_msgs/msg/Image"),
]
for topic, type_str in topic_type_pairs:
    checker.subscribe_dynamic(topic, type_str)

checker_thread = Thread(target=run_checker, args=(checker,))
checker_thread.start()

####################################################################

# Initialize physics and start simulation
app_utils.play()
simulation_app.update()

for _ in range(args.test_steps):
    simulation_app.update()
    if checker.error_detected:
        print("[error] Exiting simulation loop due to timestamp error")
        break

# Clean up
checker.stop()
rclpy.shutdown()
checker_thread.join()
checker.destroy_node()
app_utils.stop()
simulation_app.close()
