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

"""ROS 2 transform listener interface for querying and managing TF data in Isaac Sim."""

import threading

import carb
import yaml


def acquire_transform_listener_interface() -> TFListener:
    """Acquires a TF listener interface for ROS 2 transform monitoring.

    Returns:
        A new TFListener instance for querying ROS 2 transforms.
    """
    interface = TFListener()
    return interface


def release_transform_listener_interface(interface: TFListener) -> None:
    """Releases a TF listener interface and cleans up resources.

    Args:
        interface: The TFListener instance to release.
    """
    interface.finalize()


class TFListener:
    """A ROS 2 transform listener for querying and managing TF (Transform) data.

    This class provides an interface to listen to ROS 2 TF broadcasts and query transform relationships between
    frames. It initializes a ROS 2 node with a transform listener, manages the lifecycle of ROS 2 resources,
    and provides methods to retrieve transform data, frame relationships, and handle listener state.

    The listener runs in a separate thread using a multi-threaded executor to handle incoming transform
    messages without blocking the main thread. It supports querying individual transforms between frames,
    retrieving all available transforms relative to a root frame, and accessing frame hierarchy information.

    Args:
        node_name: Name of the ROS 2 node to create for the transform listener.
    """

    def __init__(self, node_name: str = "ros2_tf_listener") -> None:
        self._node_name = node_name

        self._node = None
        self._listener = None

    def initialize(self, distro: str) -> bool:
        """Initializes the ROS2 TF listener node and starts the executor thread.

        Args:
            distro: ROS distribution identifier.

        Returns:
            True if initialization was successful.
        """
        import rclpy
        import tf2_ros

        rclpy.init()
        self._node = rclpy.node.Node(self._node_name)

        # tf2 implementation
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._time = rclpy.time.Time
        self._lookup_transform = self._tf_buffer.lookup_transform
        self._all_frames_as_yaml = self._tf_buffer.all_frames_as_yaml

        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin).start()
        return True

    def finalize(self) -> None:
        """Shuts down the TF listener, executor, and ROS2 node."""
        import rclpy

        if self._listener:
            self._listener.unregister()
            self._listener = None
        if self._executor:
            self._executor.shutdown()
            self._executor = None
        if self._node:
            self._node.destroy_node()
            self._node = None

        try:
            rclpy.shutdown()
        except RuntimeError as e:
            carb.log_info(f"rclpy.shutdown: {e}")

    def get_transforms(self, root_frame: str) -> tuple[set, dict, list]:
        """Retrieves all available transforms relative to the root frame.

        Args:
            root_frame: The reference frame to compute transforms from.

        Returns:
            A tuple containing (frames, transforms, relations) where frames is a set of frame names,
            transforms is a dictionary mapping frame names to (translation, rotation) tuples, and relations
            is a list of (child, parent) frame pairs.
        """
        frames = set()
        relations = []
        transforms = {}

        if self._listener:
            frames_info = yaml.load(self._all_frames_as_yaml(), Loader=yaml.SafeLoader)
            if type(frames_info) is dict:
                for frame, info in frames_info.items():
                    frames.add(frame)
                    frames.add(info["parent"])
                    relations.append((frame, info["parent"]))
                    try:
                        transform = self._lookup_transform(root_frame, frame, self._time())
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        transform = (
                            [translation.x, translation.y, translation.z],
                            [rotation.x, rotation.y, rotation.z, rotation.w],
                        )
                        transforms[frame] = transform
                    except Exception:
                        pass

        return frames, transforms, relations

    def get_transform(self, target_frame: str, source_frame: str) -> tuple[tuple, str]:
        """Retrieves the transform between two frames.

        Args:
            target_frame: The target frame name.
            source_frame: The source frame name.

        Returns:
            A tuple of (transform, error) where transform is a tuple of (translation, rotation) if successful,
            or an empty tuple if failed. The error is an empty string on success or the exception type name on
            failure.
        """
        try:
            transform = self._lookup_transform(target_frame, source_frame, self._time())
        except Exception as e:
            return (), type(e).__name__
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transform = ([translation.x, translation.y, translation.z], [rotation.x, rotation.y, rotation.z, rotation.w])
        return transform, ""

    def reset(self) -> None:
        """Clears the TF buffer to remove stale transform data."""
        # remove "TF_OLD_DATA ignoring data from the past" warning
        if self._listener:
            carb.log_info("Reset TF listener (ROS2)")
            self._tf_buffer.clear()

    def is_ready(self) -> bool:
        """Whether the TF listener has been initialized and is ready to use.

        Returns:
            True if the listener is initialized.
        """
        return self._listener is not None
