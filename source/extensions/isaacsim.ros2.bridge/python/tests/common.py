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
import functools
import gc
import time
import types
import unittest

import carb
import numpy as np
import omni
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import PhysxSchema, UsdGeom, UsdPhysics


class ROS2TestCase(omni.kit.test.AsyncTestCase):
    """Base test class that automatically times all test methods.

    This class extends omni.kit.test.AsyncTestCase to automatically print
    the execution time of each test method. All test classes should inherit
    from this instead of omni.kit.test.AsyncTestCase directly.
    """

    async def setUp(self):
        """Set up test timing before each test method."""
        self._test_start_time = time.time()
        self._timeline = omni.timeline.get_timeline_interface()
        import rclpy

        if not rclpy.ok():
            rclpy.init()

    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        """Print test execution time after each test method."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        import rclpy

        if rclpy.ok():
            rclpy.shutdown()

        test_duration = time.time() - self._test_start_time
        test_name = self._testMethodName
        print(f"\n[TEST TIMING] {test_name}: {test_duration:.3f} seconds")
        gc.collect()


def set_translate(prim, new_loc):
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


def set_rotate(prim, rot_mat):
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


async def add_cube(path, size, offset):
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


async def add_carter():
    from pxr import Gf, PhysicsSchemaTools

    assets_root_path = await get_assets_root_path_async()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/NVIDIA/Carter/carter_v1_physx_lidar.usd")
    stage = omni.usd.get_context().get_stage()

    PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.25), Gf.Vec3f(0.5))


async def add_carter_ros():
    from pxr import Gf, PhysicsSchemaTools

    assets_root_path = await get_assets_root_path_async()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Samples/ROS2/Robots/Carter_ROS.usd")

    # Disabling cameras by default
    import omni.graph.core as og

    ros_cameras_graph_path = "/Carter/ROS_Cameras"

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


async def add_nova_carter_ros():
    assets_root_path = await get_assets_root_path_async()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")


async def add_franka():
    assets_root_path = await get_assets_root_path_async()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return
    (result, error) = await open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")


def get_qos_profile():
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def fields_to_dtype(fields, point_step):
    """Convert a list of PointFields to a numpy record datatype."""
    DUMMY_FIELD_PREFIX = "__"

    from sensor_msgs.msg import PointField

    # mappings between PointField types and numpy types
    type_mappings = [
        (PointField.INT8, np.dtype("int8")),
        (PointField.UINT8, np.dtype("uint8")),
        (PointField.INT16, np.dtype("int16")),
        (PointField.UINT16, np.dtype("uint16")),
        (PointField.INT32, np.dtype("int32")),
        (PointField.UINT32, np.dtype("uint32")),
        (PointField.FLOAT32, np.dtype("float32")),
        (PointField.FLOAT64, np.dtype("float64")),
    ]
    pftype_to_nptype = dict(type_mappings)
    nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

    # sizes (in bytes) of PointField types
    pftype_sizes = {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }

    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(("%s%d" % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def set_joint_drive_parameters(joint_path, joint_type, drive_type, target_value, stiffness=None, damping=None):
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
