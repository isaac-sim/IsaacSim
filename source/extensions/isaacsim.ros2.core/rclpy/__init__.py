# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from typing import Any

import carb
import omni.ext
import omni.kit
from isaacsim.ros2.core.impl.ros2_common import (
    print_environment_setup_instructions,
    restore_ros2_python_paths,
    setup_ros2_environment,
)


class Extension(omni.ext.IExt):
    """Extension to load and configure rclpy for ROS 2."""

    def on_startup(self, ext_id: Any) -> None:
        """Initialize the extension.

        Args:
            ext_id: Extension identifier provided by Kit.
        """
        ros_distro = os.environ.get("ROS_DISTRO")
        if ros_distro is not None and os.path.join(f"{ros_distro}", "rclpy") in os.path.join(os.path.dirname(__file__)):
            omni.kit.app.get_app().print_and_log("Attempting to load system rclpy")
            restore_ros2_python_paths()
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

                setup_ros2_environment(self._extension_path, ros_distro)

                try:
                    import rclpy

                    rclpy.init()
                    rclpy.shutdown()
                    omni.kit.app.get_app().print_and_log("rclpy loaded")
                except Exception as e:
                    omni.kit.app.get_app().print_and_log(f"Could not import internal rclpy: {e}")
                    print_environment_setup_instructions(self._extension_path, ros_distro)
                try:
                    import rclpy

                    rclpy.init()
                    rclpy.shutdown()
                except Exception as e:
                    carb.log_warn("Could not import rclpy")
            return

    def on_shutdown(self) -> None:
        """Clean up resources when the extension shuts down."""
        rclpy_path = os.path.join(os.path.dirname(__file__))
        if rclpy_path in sys.path:
            sys.path.remove(rclpy_path)
