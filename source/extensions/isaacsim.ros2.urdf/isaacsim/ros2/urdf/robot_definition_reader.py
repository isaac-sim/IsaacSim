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


"""ROS 2 robot description reader helpers."""

import re
import threading
import typing

import carb


def package_path_to_system_path(package_name: str, relative_path: str = "") -> str:
    """Resolve a ROS 2 package share directory.

    Args:
        package_name: ROS 2 package name to resolve.
        relative_path: Optional relative path within the package.

    Returns:
        Absolute path to the package share directory.
    """
    from ament_index_python.packages import get_package_share_directory

    package_share_path = get_package_share_directory(package_name)
    if relative_path:
        return f"{package_share_path}/{relative_path}"
    return package_share_path


def replace_package_urls_with_paths(input_string: str) -> tuple[str, bool]:
    """Replace ROS package URLs with filesystem paths.

    Args:
        input_string: URDF string containing package URLs.

    Returns:
        A tuple of (updated_urdf_string, package_found) where updated_urdf_string contains resolved paths and
        package_found indicates whether any package URL was successfully resolved.
    """
    pattern = r"package://([^/]+)"
    matches = re.findall(pattern, input_string)
    package_found = False

    for package_name in matches:
        try:
            package_path = package_path_to_system_path(package_name)
        except Exception as e:
            carb.log_error(f"Could not resolve package '{package_name}'")
            continue
        package_url = "package://" + package_name
        input_string = input_string.replace(package_url, package_path)
        package_found = True

    return input_string, package_found


def singleton(class_: type) -> typing.Callable:
    """Return a singleton decorator for the provided class.

    Args:
        class_: Class to wrap as a singleton.

    Returns:
        Callable that returns a singleton instance of the class.
    """
    instances: dict[type, typing.Any] = {}

    def getinstance(*args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


Singleton = singleton


@singleton
class RobotDefinitionReader:
    """Fetch robot descriptions from ROS 2 nodes.

    This is a singleton class that queries ROS 2 nodes for robot_description parameters and resolves package URLs
    to filesystem paths.
    """

    def __init__(self) -> None:
        self.node_name = None
        self.node = None
        self.future = None
        self._request_generation = 0
        self._request_thread: threading.Thread | None = None
        self.description_received_fn: typing.Callable[[str, bool], None] | None = None
        self.status_fn: typing.Callable[[str, int], None] | None = None
        self.urdf_doc = ""
        self.urdf_abs = ""
        self.package_found = False

    def __del__(self) -> None:
        """Shutdown the ROS 2 node and context."""
        import rclpy

        if self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        rclpy.try_shutdown()

    def on_description_received(self, _: str) -> None:
        """Invoke the description callback."""
        if self.description_received_fn:
            self.description_received_fn(self.urdf_abs, self.package_found)

    def service_call(self, node: typing.Any, node_name: str, request_generation: int) -> None:
        """Query the robot description parameter from a node.

        Args:
            node: ROS 2 node to query.
            node_name: ROS 2 node name to query.
            request_generation: Request generation this worker belongs to.

        Example:

        .. code-block:: python

            >>> reader = RobotDefinitionReader()  # doctest: +SKIP
            >>> reader.service_call(None, "robot_state_publisher", 1)  # doctest: +SKIP
        """
        import rclpy
        from rcl_interfaces.srv import GetParameters

        try:
            client = node.create_client(GetParameters, f"/{node_name}/get_parameters")
            while rclpy.ok() and request_generation == self._request_generation:
                if client.wait_for_service(timeout_sec=0.1):
                    break
            else:
                return

            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            self.future = future

            while rclpy.ok() and request_generation == self._request_generation:
                if future.cancelled():
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
                if future.done():
                    break

            if future.done() and request_generation == self._request_generation:
                try:
                    response = future.result()
                    if response.values:
                        for param in response.values:
                            self.urdf_doc = param.string_value
                            self.urdf_abs, self.package_found = replace_package_urls_with_paths(self.urdf_doc)
                            self.on_description_received(self.urdf_abs)
                except Exception as e:
                    carb.log_error(f"Service call failed {e!r}")
                    if self.status_fn:
                        self.status_fn("ROS node error", 0xFF0000FF)
        except rclpy._rclpy_pybind11.RCLError:
            carb.log_warn(f"ROS 2 context shut down while querying node '{node_name}'")

        try:
            node.destroy_node()
        except rclpy._rclpy_pybind11.RCLError:
            pass
        if request_generation == self._request_generation:
            rclpy.try_shutdown()

    def _cancel_request(self) -> bool:
        self._request_generation += 1
        if self.future:
            self.future.cancel()
            self.future = None
        if self._request_thread is threading.current_thread():
            self._request_thread = None
            return True
        if self._request_thread and self._request_thread.is_alive():
            self._request_thread.join(timeout=1.0)
            if self._request_thread.is_alive():
                carb.log_warn("Timed out waiting for the previous ROS 2 robot description request to stop")
                return False
        self._request_thread = None
        return True

    def cancel(self) -> None:
        """Cancel any active robot description request."""
        import rclpy

        self._cancel_request()
        self.node_name = None
        rclpy.try_shutdown()

    def start_get_robot_description(self, node_name: str) -> None:
        """Start fetching the robot description.

        Args:
            node_name: ROS 2 node name to query.
        """
        import rclpy

        if not self._cancel_request():
            return
        if not node_name:
            self.node_name = None
            return

        request_generation = self._request_generation
        self.node_name = node_name
        try:
            rclpy.init()
        except RuntimeError:
            pass
        node = rclpy.create_node("service_client")

        thread = threading.Thread(target=self.service_call, args=(node, node_name, request_generation), daemon=True)
        self._request_thread = thread
        thread.start()
