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

"""Shared ROS 2 node test utilities.

Provides simulation stepping, USD transform edits, asset setup, QoS helpers, and
object-detection geometry used by camera, sensor, transform, and publisher tests.
"""

import math
from typing import Any

import numpy as np
import omni
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.sensors.experimental.physics import Raycast
from pxr import Sdf, UsdPhysics


async def simulate_async(seconds: Any, steps_per_sec: Any = 60, callback: Any = None) -> None:
    """Run simulation for a given duration asynchronously.

    Args:
        seconds: Duration to simulate.
        steps_per_sec: Number of simulation steps per second.
        callback: Optional callback to run after each update.
    """
    for _ in range(int(seconds * steps_per_sec)):
        await omni.kit.app.get_app().next_update_async()
        if callback:
            callback()


def set_translate(prim: Any, new_loc: Any) -> None:
    """Set the translation of a USD prim.

    Args:
        prim: USD prim to translate.
        new_loc: New translation value.
    """
    from pxr import Gf, UsdGeom

    properties = prim.GetPropertyNames()
    if "xformOp:translate" in properties:
        translate_attr = prim.GetAttribute("xformOp:translate")

        translate_attr.Set(new_loc)
    elif "xformOp:translation" in properties:
        translation_attr = prim.GetAttribute("xformOp:translate")
        translation_attr.Set(new_loc)
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        matrix.SetTranslateOnly(new_loc)
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(Gf.Matrix4d().SetTranslate(new_loc))


def set_rotate(prim: Any, rot_mat: Any) -> None:
    """Set the rotation of a USD prim.

    Args:
        prim: USD prim to rotate.
        rot_mat: Rotation matrix to apply.
    """
    from pxr import Gf, UsdGeom

    properties = prim.GetPropertyNames()
    if "xformOp:rotate" in properties:
        rotate_attr = prim.GetAttribute("xformOp:rotate")
        rotate_attr.Set(rot_mat)
    elif "xformOp:transform" in properties:
        transform_attr = prim.GetAttribute("xformOp:transform")
        matrix = prim.GetAttribute("xformOp:transform").Get()
        matrix.SetRotateOnly(rot_mat.ExtractRotation())
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(Gf.Matrix4d().SetRotate(rot_mat))


async def add_cube(path: Any, size: Any, offset: Any) -> Any:
    """Add a physics-enabled cube to the stage.

    Args:
        path: USD path for the cube.
        size: Cube size value.
        offset: Translation offset for the cube.

    Returns:
        Created cube geometry.
    """
    from pxr import UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)

    cubeGeom.CreateSizeAttr(size)
    cubeGeom.AddTranslateOp().Set(offset)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
    rigid_api.CreateRigidBodyEnabledAttr(True)
    UsdPhysics.CollisionAPI.Apply(cubePrim)

    return cubeGeom


def create_raycast_lidar_sensor(
    path: str = "/World/Lidar",
    parent: str | None = None,
    h_fov: float = 360.0,
    h_resolution: float = 0.4,
    v_fov: float = 0.0,
    v_count: int = 1,
    min_range: float = 0.4,
    max_range: float = 100.0,
    translations: list | None = None,
) -> str:
    """Create a physics raycast sensor configured as a horizontal lidar.

    Args:
        path: Sensor prim path or child name.
        parent: Parent prim path.
        h_fov: Horizontal field of view.
        h_resolution: Horizontal angular resolution.
        v_fov: Vertical field of view.
        v_count: Number of vertical scan lines.
        min_range: Minimum raycast range.
        max_range: Maximum raycast range.
        translations: Sensor translation values.

    Returns:
        Full prim path of the created sensor.
    """
    if translations is None:
        translations = [[0.0, 0.0, 0.0]]
    if parent is None:
        parent = str(Sdf.Path(path).GetParentPath())
        path = Sdf.Path(path).name
    h_count = int(h_fov / h_resolution)
    origins = []
    directions = []
    for vi in range(v_count):
        if v_count > 1:
            v_angle = math.radians(-v_fov / 2 + v_fov * vi / (v_count - 1))
        else:
            v_angle = 0.0
        for hi in range(h_count):
            h_angle = math.radians(-h_fov / 2 + h_fov * hi / h_count)
            dx = math.cos(v_angle) * math.cos(h_angle)
            dy = math.cos(v_angle) * math.sin(h_angle)
            dz = math.sin(v_angle)
            origins.append([0.0, 0.0, 0.0])
            directions.append([dx, dy, dz])

    raycast = Raycast.create(
        f"{parent}/{path}",
        min_range=min_range,
        max_range=max_range,
        ray_origins=origins,
        ray_directions=directions,
        translations=translations,
    )
    return raycast.paths[0]


async def add_carter(assets_root_path: Any, prim_path: Any = "/Carter") -> Any:
    """Add a Carter robot to the stage.

    Args:
        assets_root_path: Isaac assets root path.
        prim_path: Prim path for the Carter robot.

    Returns:
        Prim path for the Carter robot.
    """
    from pxr import Gf, PhysicsSchemaTools

    stage_utils.add_reference_to_stage(
        assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd", prim_path
    )
    stage = omni.usd.get_context().get_stage()
    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))
    await omni.kit.app.get_app().next_update_async()
    return prim_path


async def add_carter_ros(assets_root_path: Any, prim_path: Any = "/Carter") -> Any:
    """Add a Carter robot with ROS 2 graphs to the stage.

    Args:
        assets_root_path: Isaac assets root path.
        prim_path: Prim path for the Carter robot.

    Returns:
        Prim path for the Carter robot.
    """
    from pxr import Gf, PhysicsSchemaTools

    stage_utils.add_reference_to_stage(assets_root_path + "/Isaac/Samples/ROS2/Robots/Carter_ROS.usd", prim_path)
    await omni.kit.app.get_app().next_update_async()
    # Disabling cameras by default
    import omni.graph.core as og

    ros_cameras_graph_path = prim_path + "/ROS_Cameras"

    prims_to_disable = [
        ros_cameras_graph_path + "/isaac_create_render_product_left.inputs:enabled",
        ros_cameras_graph_path + "/isaac_create_render_product_right.inputs:enabled",
        ros_cameras_graph_path + "/ros2_camera_helper.inputs:enabled",
        ros_cameras_graph_path + "/ros2_camera_helper_01.inputs:enabled",
        ros_cameras_graph_path + "/ros2_camera_helper_03.inputs:enabled",
        ros_cameras_graph_path + "/ros2_camera_helper_04.inputs:enabled",
        ros_cameras_graph_path + "/ros2_camera_info_helper.inputs:enabled",
    ]
    for prim_to_disable in prims_to_disable:
        og.Controller.set(og.Controller.attribute(prim_to_disable), False)

    stage = omni.usd.get_context().get_stage()

    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))
    await omni.kit.app.get_app().next_update_async()
    return prim_path


async def add_nova_carter_ros(assets_root_path: Any) -> None:
    """Add a Nova Carter robot with ROS 2 graphs to the stage.

    Args:
        assets_root_path: Isaac assets root path.
    """
    result, error = await stage_utils.open_stage_async(
        assets_root_path + "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"
    )
    await omni.kit.app.get_app().next_update_async()


async def add_franka(assets_root_path: Any) -> None:
    """Add a Franka robot to the stage.

    Args:
        assets_root_path: Isaac assets root path.
    """
    result, error = await stage_utils.open_stage_async(
        assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
    )


def get_qos_profile(depth: int = 1, history: str = "keep_last") -> Any:
    """Create a ROS 2 QoS profile with the given parameters.

    Args:
        depth: QoS queue depth.
        history: QoS history policy name.

    Returns:
        ROS 2 QoS profile.
    """
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    history_policy = QoSHistoryPolicy.SYSTEM_DEFAULT if history == "system_default" else QoSHistoryPolicy.KEEP_LAST
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=history_policy, depth=depth)


def swap_joint_bodies(joint_path: Any) -> None:
    """Swap body0 and body1 relationships and local transforms on a USD joint.

    Args:
        joint_path: USD path of the joint prim.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.Joint(prim)

    body0_targets = joint.GetBody0Rel().GetTargets()
    body1_targets = joint.GetBody1Rel().GetTargets()
    joint.GetBody0Rel().SetTargets(body1_targets)
    joint.GetBody1Rel().SetTargets(body0_targets)

    pos0_attr = prim.GetAttribute("physics:localPos0")
    pos1_attr = prim.GetAttribute("physics:localPos1")
    rot0_attr = prim.GetAttribute("physics:localRot0")
    rot1_attr = prim.GetAttribute("physics:localRot1")

    pos0 = pos0_attr.Get() if pos0_attr else None
    pos1 = pos1_attr.Get() if pos1_attr else None
    rot0 = rot0_attr.Get() if rot0_attr else None
    rot1 = rot1_attr.Get() if rot1_attr else None

    if pos0 is not None and pos1 is not None:
        pos0_attr.Set(pos1)
        pos1_attr.Set(pos0)
    if rot0 is not None and rot1 is not None:
        rot0_attr.Set(rot1)
        rot1_attr.Set(rot0)


SIMPLE_ARTICULATION_3J_REVERSED_JOINTS = [
    "/Articulation/Arm/CenterRevoluteJoint",
    "/Articulation/DistalPivot/DistalRevoluteJoint",
]


def fix_reversed_joints(joint_paths: Any) -> None:
    """Swap body0/body1 on a list of joints that have reversed parent/child ordering.

    Args:
        joint_paths: List of USD paths to joints that need body0/body1 swapped.
    """
    for path in joint_paths:
        swap_joint_bodies(path)


def set_joint_drive_parameters(
    joint_path: Any, joint_type: Any, drive_type: Any, target_value: Any, stiffness: Any = None, damping: Any = None
) -> Any:
    """Set drive parameters for a joint on the stage.

    Args:
        joint_path: USD path of the joint prim.
        joint_type: Joint drive name.
        drive_type: Drive target mode.
        target_value: Target value to set.
        stiffness: Optional drive stiffness.
        damping: Optional drive damping.

    Returns:
        False if the drive does not exist, otherwise None.
    """
    stage = omni.usd.get_context().get_stage()
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joint_path), joint_type)

    if not drive:
        # if no drive exists, return false
        return False

    if drive_type == "position":
        if not drive.GetTargetPositionAttr():
            drive.CreateTargetPositionAttr(target_value)
        else:
            drive.GetTargetPositionAttr().Set(target_value)
    elif drive_type == "velocity":
        if not drive.GetTargetVelocityAttr():
            drive.CreateTargetVelocityAttr(target_value)
        else:
            drive.GetTargetVelocityAttr().Set(target_value)

    if stiffness is not None:
        if not drive.GetStiffnessAttr():
            drive.CreateStiffnessAttr(stiffness)
        else:
            drive.GetStiffnessAttr().Set(stiffness)

    if damping is not None:
        if not drive.GetDampingAttr():
            drive.CreateDampingAttr(damping)
        else:
            drive.GetDampingAttr().Set(damping)


def _create_cube_with_material(
    index: int, position: np.ndarray, scale: np.ndarray, color: np.ndarray, material_props: tuple, enable_material: bool
) -> dict:
    """Create a cube with optional material.

    Args:
        index: Cube index used to build the prim path.
        position: Cube position.
        scale: Cube scale.
        color: Cube display color.
        material_props: Nonvisual material properties.
        enable_material: Whether to apply nonvisual material.

    Returns:
        Cube metadata keyed by prim path, or an empty dictionary.
    """
    cube_path = f"/World/cube_{index}"
    cube = Cube(
        cube_path,
        sizes=1.0,
        positions=position,
        scales=scale,
    )

    cube_info = {}
    if enable_material:
        nv_material = NonVisualMaterial(
            f"{cube_path}/nv_material",
            bases=material_props[0],
            coatings=material_props[1],
            attributes=material_props[2],
        )
        cube.apply_visual_materials(nv_material)
        cube_info = {"material_id": NonVisualMaterial.encode_material_ids(nv_material).numpy().item()}

    return {cube_path: cube_info} if cube_info else {}


def create_sarcophagus(enable_nonvisual_material: bool = True) -> Any:
    """Create a nested cube structure for testing object detection.

    Args:
        enable_nonvisual_material: Whether to apply nonvisual materials.

    Returns:
        Cube metadata for the generated structure.
    """
    # Autogenerate sarcophagus
    dims = [(10, 5, 7), (15, 9, 11), (20, 13, 15), (25, 17, 19)]
    cube_configs = [
        # (position_formula, scale, color, material_props)
        (
            lambda l, h1, h, signs: np.multiply(signs, [l + 0.5, l / 2, h1 - h / 2]),
            lambda l, h: [1, l, h],
            np.array([1, 0, 0]),
            ("aluminum", "paint", "emissive"),
        ),
        (
            lambda l, h1, h, signs: np.multiply(signs, [l / 2, l + 0.5, h1 - h / 2]),
            lambda l, h: [l, 1, h],
            np.array([0, 1, 0]),
            ("steel", "clearcoat", "emissive"),
        ),
        (
            lambda l, h1, h, signs: np.multiply(signs, [l / 2, l / 2, h1 + 0.5]),
            lambda l, h: [l, l, 1],
            np.array([0, 0, 1]),
            ("concrete", "clearcoat", "emissive"),
        ),
        (
            lambda l, h1, h, signs: np.multiply(signs, [l / 2, l / 2, -h2 - 0.5]),
            lambda l, h: [l, l, 1],
            np.array([1, 1, 0]),
            ("concrete", "paint", "emissive"),
        ),
    ]

    i = 0
    cube_info = {}
    for l, h1, h2 in dims:
        h = h1 + h2
        x_sign = -1 if 0 < i < 3 else 1
        y_sign = -1 if i > 1 else 1
        signs = np.array([x_sign, y_sign, 1])

        for j, (pos_fn, scale_fn, color, mat_props) in enumerate(cube_configs[:3]):
            position = pos_fn(l, h1, h, signs)
            scale = np.array(scale_fn(l, h))
            cube_info.update(
                _create_cube_with_material(i * 4 + j, position, scale, color, mat_props, enable_nonvisual_material)
            )

        # Bottom cube needs h2 parameter
        position = np.multiply(signs, [l / 2, l / 2, -h2 - 0.5])
        scale = np.array([l, l, 1])
        cube_info.update(
            _create_cube_with_material(
                i * 4 + 3, position, scale, cube_configs[3][2], cube_configs[3][3], enable_nonvisual_material
            )
        )

        i += 1
    return cube_info
