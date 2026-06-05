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

"""ROS transform listener implementation for querying and managing coordinate frame transformations."""

import time

import carb
import yaml


def acquire_transform_listener_interface(use_tf2: bool = True) -> TFListener:
    """Acquire a transform listener interface for ROS TF.

    Args:
        use_tf2: Whether to use TF2 implementation instead of the legacy TF implementation.

    Returns:
        A TFListener interface instance.
    """
    interface = TFListener(use_tf2=use_tf2)
    return interface


def release_transform_listener_interface(interface: TFListener) -> None:
    """Release the transform listener interface.

    Args:
        interface: The TFListener interface instance to release.
    """
    interface.finalize()


class TFListener:
    """A ROS transform listener that subscribes to transform data and provides query functionality.

    This class creates a ROS node that listens to transform broadcasts and allows querying of coordinate frame
    transformations. It supports both the modern tf2 implementation and the legacy tf implementation for ROS 1.

    The listener must be initialized with a ROS distribution before use by calling the ``initialize()`` method.
    Once initialized, it can query transformations between frames, retrieve all available frames, and reset the
    internal transform buffer.

    Args:
        node_name: Name of the ROS node to create.
        use_tf2: Whether to use the tf2 implementation. If False, uses the legacy tf implementation.
    """

    def __init__(self, node_name: str = "ros_tf_listener", use_tf2: bool = True) -> None:
        self._node_name = node_name
        self._use_tf2 = use_tf2

        self._listener = None

    def initialize(self, distro: str) -> bool:
        """Initialize the ROS transform listener.

        Sets up the ROS node and configures either a tf2 or tf listener based on the instance configuration.
        Checks if the ROS master is running before initialization.

        Args:
            distro: ROS distribution identifier.

        Returns:
            True if initialization was successful, False if ROS master is not running.
        """
        import rosgraph
        import rospy

        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except Exception:
            carb.log_warn("ROS master is not running")
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            time.sleep(0.1)
            carb.log_info(f"ROS node started ({self._node_name})")
        except rospy.ROSException as e:
            carb.log_error(f"ROS node ({self._node_name}): {e}")

        # tf2 implementation
        if self._use_tf2:
            import rospy
            import tf2_ros

            self._tf_buffer = tf2_ros.Buffer()
            self._listener = tf2_ros.TransformListener(self._tf_buffer)
            # internal methods
            self._time = rospy.Time
            self._lookup_transform = self._tf_buffer.lookup_transform
            self._all_frames_as_yaml = self._tf_buffer.all_frames_as_yaml
        # tf implementation
        else:
            import tf

            self._listener = tf.TransformListener()
            # internal methods
            self._time = self._listener.getLatestCommonTime
            self._lookup_transform = self._listener.lookupTransform
            self._all_frames_as_yaml = self._listener._buffer.all_frames_as_yaml
        return True

    def finalize(self) -> None:
        """Clean up the transform listener.

        Unregisters the tf2 listener if using tf2 implementation and releases listener resources.
        """
        if self._listener:
            if self._use_tf2:
                self._listener.unregister()
            self._listener = None

        # shutdown ROS node
        # rospy.signal_shutdown("isaacsim.ros2.tf_viewer")

    def get_transforms(self, root_frame: str) -> tuple[set, dict, list]:
        """Retrieve all transform frames and their relationships relative to a root frame.

        Args:
            root_frame: The reference frame to compute transforms relative to.

        Returns:
            A tuple containing (frames, transforms, relations) where frames is a set of all frame names,
            transforms is a dictionary mapping frame names to their transform data as (translation, rotation) tuples,
            and relations is a list of (child, parent) frame pairs.
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
                        transform = self._lookup_transform(
                            root_frame, frame, self._time() if self._use_tf2 else self._time(root_frame, frame)
                        )
                        if self._use_tf2:
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

    def get_transform(self, target_frame: str, source_frame: str) -> tuple[tuple | dict, str]:
        """Look up the transform between two frames.

        Args:
            target_frame: The target coordinate frame.
            source_frame: The source coordinate frame.

        Returns:
            A tuple of (transform, error_message) where transform is either a tuple of
            (translation, rotation) lists on success or an empty tuple on failure, and error_message
            is the exception type name if an error occurred or an empty string on success.
        """
        try:
            transform = self._lookup_transform(
                target_frame, source_frame, self._time() if self._use_tf2 else self._time(target_frame, source_frame)
            )
        except Exception as e:
            return (), type(e).__name__
        if self._use_tf2:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transform = (
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
        return transform, ""

    def reset(self) -> None:
        """Clear the transform buffer.

        Removes cached transform data to eliminate "TF_OLD_DATA ignoring data from the past" warnings.
        """
        # remove "TF_OLD_DATA ignoring data from the past" warning
        if self._listener:
            carb.log_info("Reset TF listener (ROS)")
            self._tf_buffer.clear() if self._use_tf2 else self._listener._buffer.clear()

    def is_ready(self) -> bool:
        """Check if the transform listener is initialized and ready.

        Returns:
            True if the listener has been initialized, False otherwise.
        """
        return self._listener is not None
