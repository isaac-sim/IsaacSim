# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import asyncio
import os
import subprocess
import sys
import weakref

import carb
import omni
import omni.ext
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import register_node_writer_with_telemetry

BRIDGE_NAME = "isaacsim.ros2.bridge"
BRIDGE_PREFIX = "ROS2"


class ROS2BridgeExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ros2bridge = None
        self._module = None
        self._rclpy_instance = None

        # WAR for incorrect extension trying to be started
        ext_manager = omni.kit.app.get_app().get_extension_manager()

        self._extension_path = ext_manager.get_extension_path(ext_id)

        backup_ros_distro = carb.settings.get_settings().get_as_string("/exts/isaacsim.ros2.bridge/ros_distro")
        ros_distro = os.environ.get("ROS_DISTRO")
        if ros_distro is None:
            carb.log_warn("ROS_DISTRO env var not found, Please source ROS before enabling this extension")
            omni.kit.app.get_app().print_and_log(f"Using backup internal ROS2 {backup_ros_distro} distro")
            ros_distro = backup_ros_distro
            os.environ["ROS_DISTRO"] = ros_distro
            # os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

        if ros_distro not in ["humble", "jazzy"]:
            carb.log_error(f"ROS_DISTRO of {ros_distro} is currently not supported")
            ext_manager.set_extension_enabled("isaacsim.ros2.bridge", False)
            return

        if sys.platform == "win32":
            if os.environ.get("PATH"):
                os.environ["PATH"] = os.environ.get("PATH") + ";" + self._extension_path + "/bin"
                # WAR: sys.path on windows is missing PYTHONPATH variables, causing rclpy to not be found
                if os.environ.get("PYTHONPATH") is not None:
                    sys.path.extend(os.environ.get("PYTHONPATH").split(";"))
            else:
                os.environ["PATH"] = self._extension_path + "/bin"

        carb.get_framework().load_plugins(
            loaded_file_wildcards=["isaacsim.ros2.bridge.plugin"],
            search_paths=[os.path.abspath(os.path.join(self._extension_path, "bin"))],
        )
        from isaacsim.ros2.bridge import _ros2_bridge

        self._module = _ros2_bridge

        if self.check_status(os.environ["ROS_DISTRO"]) is False:
            if sys.platform == "linux":
                omni.kit.app.get_app().print_and_log(
                    f"To use the internal libraries included with the extension please set the following environment variables to use with FastDDS (default) or CycloneDDS before starting Isaac Sim:\n\n"
                    f"FastDDS (default):\n"
                    f"export ROS_DISTRO={ros_distro}\n"
                    f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
                    f"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{self._extension_path}/{ros_distro}/lib\n\n"
                    f"OR\n\n"
                    f"CycloneDDS:\n"
                    f"export ROS_DISTRO={ros_distro}\n"
                    f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
                    f"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{self._extension_path}/{ros_distro}/lib\n\n"
                )
            else:
                omni.kit.app.get_app().print_and_log(
                    f"To use the internal libraries included with the extension, please set the environment variables using one of the following methods before starting Isaac Sim:\n\n"
                    f"Command Prompt (CMD):\n"
                    f"set ROS_DISTRO={ros_distro}\n"
                    f"set RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
                    f"set PATH=%PATH%;{self._extension_path}/{ros_distro}/lib\n\n"
                    f"PowerShell:\n"
                    f'$env:ROS_DISTRO = "{ros_distro}"\n'
                    f'$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"\n'
                    f'$env:PATH = "$env:PATH;{self._extension_path}/{ros_distro}/lib"\n\n'
                )
            carb.log_error(f"ROS2 Bridge startup failed")
            ext_manager.set_extension_enabled("isaacsim.ros2.bridge", False)
        else:
            self._ros2bridge = self._module.acquire_ros2_bridge_interface()
            if self._ros2bridge.get_startup_status() is False:
                carb.log_error(f"ROS2 Bridge startup failed")
                return
            self.register_nodes()

            # manually load the rclpy extension, only if we know ROS 2 is working
            if f"{self._extension_path}" not in sys.path:
                sys.path.append(f"{self._extension_path}")

            if ros_distro == "humble":
                from humble.rclpy import Extension as rclpy_ext

                self._rclpy_instance = rclpy_ext()
            if ros_distro == "jazzy":
                from jazzy.rclpy import Extension as rclpy_ext

                self._rclpy_instance = rclpy_ext()

            if self._rclpy_instance is not None:
                self._rclpy_instance.on_startup("isaacsim.ros2.bridge")

    def on_shutdown(self):
        async def safe_shutdown(module, bridge):
            omni.timeline.get_timeline_interface().stop()
            await omni.kit.app.get_app().next_update_async()
            if bridge is not None:
                module.release_ros2_bridge_interface(bridge)

        asyncio.ensure_future(safe_shutdown(self._module, self._ros2bridge))
        self.unregister_nodes()
        if self._rclpy_instance is not None:
            self._rclpy_instance.on_shutdown()

    def check_status(self, distro):
        # Run an external process that checks if ROS2 can be loaded
        # If ROS2 cannot be loaded a memory leak occurs, running in a separate process prevent this
        path = os.path.abspath(self._extension_path + "/bin")
        ros_lib_path = os.path.join(os.path.abspath(f"{self._extension_path}/{distro}/lib"), "")

        command = [f'./isaacsim.ros2.bridge.check "{ros_lib_path}"']
        if sys.platform == "win32":

            command = [f"isaacsim.ros2.bridge.check.exe", f"{ros_lib_path}"]

        try:
            output = subprocess.check_output(command, shell=True, cwd=f"{path}")
            return True
        except subprocess.CalledProcessError as grepexc:
            print(grepexc.output.decode("utf-8"))
            return False

    def register_nodes(self):

        # For Simulation and System time. Removed first S char in keys to account for both upper and lower cases.
        TIME_TYPES = [("imulationTime", ""), ("ystemTime", "SystemTime")]

        for time_type in TIME_TYPES:
            ##### Publish RGB
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishImage",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    f"{rv}IsaacConvertRGBAToRGB",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            ##### Publish Depth
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.DistanceToImagePlane.name
            )
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishImage",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    f"{rv}IsaacPassthroughImagePtr",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32FC1",
                category=BRIDGE_NAME,
            )

            # publish depth pcl
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishPointCloud",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=[
                    f"{rv}IsaacConvertDepthToPointCloud",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # instance
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishInstanceSegmentation",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    "instance_segmentation",
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("InstanceSegmentation")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32SC1",
                category=BRIDGE_NAME,
            )

            # Semantic
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishSemanticSegmentation",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    "semantic_segmentation",
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("SemanticSegmentation")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32SC1",
                category=BRIDGE_NAME,
            )

            # Bbox2d tight
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox2DTight",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox2D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_2d_tight", attributes_mapping={"input:semanticTypes": ["class"]}
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox2DTight")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # bbox2d Loose
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox2DLoose",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox2D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_2d_loose",
                        attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox2DLoose")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # bbox3d Loose
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox3D",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox3D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_3d",
                        attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox3D")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # camera info
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishCameraInfo",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishCameraInfo",
                annotators=[
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # outputs that we can publish labels for
            label_names = {
                "instance_segmentation": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "InstanceSegmentation"
                ),
                "semantic_segmentation": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "SemanticSegmentation"
                ),
                "bounding_box_2d_tight": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "BoundingBox2DTight"
                ),
                "bounding_box_2d_loose": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "BoundingBox2DLoose"
                ),
                "bounding_box_3d": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox3D"),
            }
            for annotator, annotator_name in label_names.items():
                register_node_writer_with_telemetry(
                    name=f"{annotator_name}{BRIDGE_PREFIX}{time_type[1]}PublishSemanticLabels",
                    node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishSemanticLabels",
                    annotators=[
                        annotator,
                        f"{annotator_name}IsaacSimulationGate",
                        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                            f"IsaacReadS{time_type[0]}",
                            attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"},
                        ),
                    ],
                    category=BRIDGE_NAME,
                )

            # RTX lidar PCL publisher
            register_node_writer_with_telemetry(
                name=f"RtxLidar{BRIDGE_PREFIX}{time_type[1]}PublishPointCloud",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=[
                    "IsaacExtractRTXSensorPointCloudNoAccumulator",
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            register_node_writer_with_telemetry(
                name=f"RtxLidar{BRIDGE_PREFIX}{time_type[1]}PublishPointCloudBuffer",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=[
                    "IsaacCreateRTXLidarScanBuffer",
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # RTX Radar PCL publisher
            register_node_writer_with_telemetry(
                name=f"RtxRadar{BRIDGE_PREFIX}{time_type[1]}PublishPointCloud",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=[
                    "IsaacExtractRTXSensorPointCloudNoAccumulator",
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # RTX lidar LaserScan publisher
            register_node_writer_with_telemetry(
                name=f"RtxLidar{BRIDGE_PREFIX}{time_type[1]}PublishLaserScan",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishLaserScan",
                annotators=[
                    "IsaacComputeRTXLidarFlatScan",
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

    def unregister_nodes(self):
        for writer in rep.WriterRegistry.get_writers(category=BRIDGE_NAME):
            rep.writers.unregister_writer(writer)
