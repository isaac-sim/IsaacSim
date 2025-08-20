# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import re

import carb
import isaacsim.core.utils.prims as prim_utils
from geometry_msgs.msg import Accel, Point, Pose, Quaternion, Twist, Vector3
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from simulation_interfaces.msg import EntityState, Result
from std_msgs.msg import Header


def get_filtered_entities(usdrt_stage, filter_pattern=None):
    """Get filtered entities based on regex pattern.

    This function retrieves all prim paths from the usdrt stage for efficient traversing and optionally filters them using a regular expression pattern.

    Args:
        usdrt_stage: The usdrt stage to traverse for entities (for performance).
        filter_pattern: Regex pattern to filter entities. If None, all entities are returned.

    Returns:
        Tuple containing filtered entities list and error message if any.
        The first element is a list of prim path strings, the second is an error message string.

    Raises:
        Exception: If regex compilation fails.
    """
    # Check if usdrt stage is available
    if not usdrt_stage:
        return [], "usdrt Stage not available for traversing"

    # Get all prim paths from the usdrt stage
    all_prim_paths = [prim.GetPrimPath().pathString for prim in list(usdrt_stage.Traverse())[1:]]

    # Filter entities based on pattern if provided
    if filter_pattern:
        try:
            pattern = re.compile(filter_pattern)
            filtered_paths = [path for path in all_prim_paths if pattern.search(path)]
        except re.error as e:
            return [], f"Invalid regex pattern: {e}"
    else:
        filtered_paths = all_prim_paths

    return filtered_paths, ""


async def get_entity_state(entity_path):
    """Get state for a single entity.

    This function retrieves the complete state information for a specified entity,
    including pose, velocity, and acceleration data. For rigid body entities,
    physics-based velocity information is included when available.

    Args:
        entity_path: Path to the entity as a string.

    Returns:
        Tuple containing entity state, error message, and status code.
        The first element is an EntityState object or None if error occurred,
        the second is an error message string, and the third is a status code integer.

    Raises:
        Exception: If entity state retrieval fails due to invalid paths or physics errors.
    """
    # Check if entity exists
    if not prim_utils.is_prim_path_valid(entity_path):
        return None, f"Entity '{entity_path}' does not exist", Result.RESULT_NOT_FOUND
    # Get the prim
    prim = prim_utils.get_prim_at_path(entity_path)

    # Instance Proxy prims are currently not supported
    if prim.IsInstanceProxy():
        return (
            None,
            f"Entity '{entity_path}' is of InstanceProxy type. Cannot retrieve state.",
            Result.RESULT_FEATURE_UNSUPPORTED,
        )

    # Get the frame_id - use isaac:nameOverride if available and not empty, otherwise use prim name
    if prim.HasAttribute("isaac:nameOverride"):
        override_value = prim.GetAttribute("isaac:nameOverride").Get()
        if override_value and override_value.strip():
            frame_id = override_value
        else:
            frame_id = prim.GetName()
    else:
        # Extract just the name part from the full path
        frame_id = prim.GetName()

    # Initialize the entity state
    entity_state = EntityState()
    entity_state.header = Header(frame_id=frame_id, stamp=Header().stamp)

    # Check for PhysicsRigidBodyAPI
    applied_apis = prim.GetAppliedSchemas()
    has_rigid_body = "PhysicsRigidBodyAPI" in applied_apis

    # Set velocities and accelerations to zero by default
    entity_state.twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    entity_state.acceleration = Accel(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

    if has_rigid_body:
        # For rigid body prims, use experimental RigidPrim
        try:

            # Create a RigidPrim for the rigid body prim
            rigid_prim = RigidPrim(paths=entity_path, reset_xform_op_properties=False)

            # Get world poses (position and orientation)
            positions, orientations = rigid_prim.get_world_poses()

            # Convert wp.array to numpy for indexing
            positions = positions.numpy().squeeze()
            orientations = orientations.numpy().squeeze()

            # Set the pose in the response
            entity_state.pose = Pose(
                position=Point(x=float(positions[0]), y=float(positions[1]), z=float(positions[2])),
                orientation=Quaternion(
                    w=float(orientations[0]),
                    x=float(orientations[1]),
                    y=float(orientations[2]),
                    z=float(orientations[3]),
                ),
            )

            # Get velocities if physics tensor entity is valid
            if rigid_prim.is_physics_tensor_entity_valid():
                velocities = rigid_prim.get_velocities()

                # Convert wp.array to numpy for indexing
                linear_vel = velocities[0].numpy().squeeze()  # linear velocities
                angular_vel = velocities[1].numpy().squeeze()  # angular velocities

                # Set linear velocity
                entity_state.twist.linear = Vector3(
                    x=float(linear_vel[0]), y=float(linear_vel[1]), z=float(linear_vel[2])
                )

                # Set angular velocity
                entity_state.twist.angular = Vector3(
                    x=float(angular_vel[0]), y=float(angular_vel[1]), z=float(angular_vel[2])
                )
            else:
                carb.log_warn(f"Physics tensor entity not valid for rigid body '{entity_path}', velocities set to zero")

        except Exception as rigid_error:
            carb.log_warn(f"Error getting state for rigid body '{entity_path}': {rigid_error}")
            # Fallback to identity pose and zero velocities
            entity_state.pose = Pose(
                position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
    else:
        # For non-rigid body prims, use experimental XformPrim
        try:

            # Create an XformPrim for the entity
            xform_prim = XformPrim(paths=entity_path, reset_xform_op_properties=False)

            # Get world poses (position and orientation)
            positions, orientations = xform_prim.get_world_poses()

            # Convert wp.array to numpy for indexing
            positions = positions.numpy().squeeze()
            orientations = orientations.numpy().squeeze()

            # Set the pose in the response
            entity_state.pose = Pose(
                position=Point(x=float(positions[0]), y=float(positions[1]), z=float(positions[2])),
                orientation=Quaternion(
                    w=float(orientations[0]),  # w first in experimental prims
                    x=float(orientations[1]),
                    y=float(orientations[2]),
                    z=float(orientations[3]),
                ),
            )

        except Exception as xform_error:
            carb.log_warn(f"Error getting pose for '{entity_path}': {xform_error}")
            # Fallback to identity pose
            entity_state.pose = Pose(
                position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )

    return entity_state, "", Result.RESULT_OK


def create_empty_entity_state():
    """Create an empty entity state with default values.

    This function creates a new EntityState object with all fields initialized
    to their default values, including zero position, identity orientation,
    and zero velocities and accelerations.

    Returns:
        Empty entity state object with default values.
        An EntityState object with all fields set to safe default values.
    """
    from geometry_msgs.msg import Accel, Point, Pose, Quaternion, Twist, Vector3
    from std_msgs.msg import Header

    empty_state = EntityState()
    empty_state.header = Header(frame_id="", stamp=Header().stamp)
    empty_state.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    empty_state.twist = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    empty_state.acceleration = Accel(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

    return empty_state
