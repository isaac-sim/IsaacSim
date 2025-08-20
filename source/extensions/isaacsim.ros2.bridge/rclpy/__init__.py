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

import os
import sys

import carb
import omni.ext
import omni.kit


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        ros_distro = os.environ.get("ROS_DISTRO")
        if ros_distro in ["humble", "jazzy"] and os.path.join(f"{ros_distro}", "rclpy") in os.path.join(
            os.path.dirname(__file__)
        ):
            omni.kit.app.get_app().print_and_log("Attempting to load system rclpy")
            ament_prefix = os.environ.get("AMENT_PREFIX_PATH")
            if ament_prefix is not None and os.environ.get("OLD_PYTHONPATH") is not None:
                for python_path in os.environ.get("OLD_PYTHONPATH").split(":"):
                    for ament_path in ament_prefix.split(":"):
                        if python_path.startswith(os.path.abspath(ament_path) + os.sep):
                            sys.path.append(os.path.join(python_path))
                            break
            try:
                import rclpy

                rclpy.init()
                rclpy.shutdown()
                omni.kit.app.get_app().print_and_log("rclpy loaded")
            except Exception as e:
                omni.kit.app.get_app().print_and_log(f"Could not import system rclpy: {e}")
                omni.kit.app.get_app().print_and_log(f"Attempting to load internal rclpy for ROS Distro: {ros_distro}")
                sys.path.append(os.path.join(os.path.dirname(__file__)))
                ext_manager = omni.kit.app.get_app().get_extension_manager()
                self._extension_path = ext_manager.get_extension_path(ext_id)
                if sys.platform == "win32":
                    if os.environ.get("PATH"):
                        os.environ["PATH"] = os.environ.get("PATH") + ";" + self._extension_path + f"/{ros_distro}/lib"
                    else:
                        os.environ["PATH"] = self._extension_path + f"/{ros_distro}/lib"
                        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
                try:
                    import rclpy

                    rclpy.init()
                    rclpy.shutdown()
                    omni.kit.app.get_app().print_and_log("rclpy loaded")
                except Exception as e:
                    omni.kit.app.get_app().print_and_log(f"Could not import internal rclpy: {e}")
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
                try:
                    import rclpy

                    rclpy.init()
                    rclpy.shutdown()
                except Exception as e:
                    carb.log_warn("Could not import rclpy")
            return

    def on_shutdown(self):
        rclpy_path = os.path.join(os.path.dirname(__file__))
        if rclpy_path in sys.path:
            sys.path.remove(rclpy_path)
