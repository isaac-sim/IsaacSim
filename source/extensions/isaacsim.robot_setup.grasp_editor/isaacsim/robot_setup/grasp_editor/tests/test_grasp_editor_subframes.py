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

"""Test suite for validating grasp subframe positioning and pose computations in robotic grasping scenarios."""

import asyncio

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.robot_setup.grasp_editor import import_grasps_from_file
from isaacsim.robot_setup.grasp_editor.util import move_rb_subframe_to_position
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdLux, UsdPhysics


class TestGraspSubframes(omni.kit.test.AsyncTestCase):
    """Test suite for validating grasp subframe positioning and pose computations in robotic grasping scenarios.

    This test class validates the functionality of the grasp editor system by testing pose calculations between
    gripper and rigid body subframes. It specifically tests the ability to derive gripper poses from rigid body
    positions and vice versa, ensuring that grasp specifications can accurately compute transformations between
    the two coordinate frames.

    The test setup creates a scene with a Robotiq 2F-140 gripper and a soup can object, each with defined
    subframes. Ground truth poses are established based on authored grasp data, and the tests verify that the
    grasp specification system can accurately reproduce these poses through coordinate transformations.

    Key test scenarios include:
    - Computing gripper poses from known rigid body poses using grasp specifications
    - Computing rigid body poses from known gripper poses using grasp specifications
    - Moving rigid body subframes to desired positions and orientations
    - Validating pose accuracy against recorded ground truth data from the Properties Panel
    """

    async def setUp(self) -> None:
        """Set up test environment with gripper, soup can, and grasp specifications.

        Creates a new stage with a Robotiq gripper, tomato soup can with subframe, ground plane,
        and lighting. Loads grasp data from YAML file for testing grasp pose calculations.
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_setup.grasp_editor")
        extension_path = ext_manager.get_extension_path(ext_id)
        self._grasp_file = extension_path + "/data/robotiq_soup_grasp.yaml"

        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()
        await self._create_light()

        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[-0.6, -0.45, 0.3], target=[0, 0.3, 0])

        asset_root_path = await get_assets_root_path_async()

        gripper_usd_path = asset_root_path + "/Isaac/Robots/Robotiq/2F-140/Robotiq_2F_140_config.usd"
        self._gripper_path = "/Robotiq_2F_140"
        stage_utils.add_reference_to_stage(gripper_usd_path, self._gripper_path)
        self._gripper_xform = XformPrim(self._gripper_path, reset_xform_op_properties=True)
        self._gripper_xform.set_world_poses(np.array([-0.24, 0.02, 0.11]))
        self._gripper_subframe = "/Robotiq_2F_140/robotiq_base_link"

        fixed_joint_path = self._gripper_path + "/FixedJoint"

        stage = stage_utils.get_current_stage()
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
        fixed_joint.GetBody1Rel().SetTargets([self._gripper_subframe])

        self._rb_path = "/soup_can"
        stage_utils.add_reference_to_stage(
            asset_root_path + "/Isaac/Props/YCB/Axis_Aligned/005_tomato_soup_can.usd", self._rb_path
        )

        self._rb_xform = XformPrim(self._rb_path, reset_xform_op_properties=True)
        self._rb_xform.set_world_poses(
            np.array([-0.06, 0.0, 0.14])[np.newaxis, :], np.array([0.707, -0.707, 0.0, 0.0])[np.newaxis, :]
        )

        stage_utils.define_prim("/soup_can/subframe")
        self._rb_subframe = XformPrim(
            "/soup_can/subframe",
            translations=np.array([0.2, 0.1, -0.05]),
            orientations=np.array([0.5, 0.2, 0.6, -1]),
            reset_xform_op_properties=True,
        )

        await self._create_light()

        GroundPlane("/ground")

        self._ground_truth_rb_translations = [
            np.array([0.14, -0.05, 0.04]),
            np.array([-0.02809064, 0.09239036, -0.06723371]),
            np.array([-0.37823358, -0.09652396, -0.05137331]),
        ]

        self._ground_truth_rb_quats = [
            np.array([-0.45706322, 0.25004356, 0.20701967, 0.82807867]),
            np.array([0.76656104, -0.42007253, 0.06654073, -0.48113986]),
            np.array([-0.32704174, 0.84878149, -0.11111393, 0.40033409]),
        ]

        self._ground_truth_gripper_translation = np.array([-0.24, 0.02, 0.11])
        self._ground_truth_gripper_orientation = np.array([0.0, 7.07106781e-01, 0.0, 7.07106781e-01])

        self._grasp_spec = import_grasps_from_file(self._grasp_file)
        await app_utils.update_app_async()

    def _assert_almost_equal(self, a: object, b: object, msg: str = "", tol: float = 1e-6) -> None:
        """Assert that two arrays are almost equal within tolerance.

        Overrides the default method to support array comparisons by converting inputs to NumPy
        arrays and checking element-wise differences.

        Args:
            a: First array or value to compare.
            b: Second array or value to compare.
            msg: Optional error message.
            tol: Tolerance for comparison.
        """
        # overriding method because it doesn't support iterables
        a = np.array(a)
        b = np.array(b)
        self.assertFalse(np.any(abs(a[a != np.array(None)] - b[b != np.array(None)]) > tol), msg)

    async def _create_light(self) -> None:
        """Create a sphere light in the scene for proper illumination.

        Adds a UsdLux SphereLight with specified radius and intensity positioned above the scene.
        """
        sphereLight = UsdLux.SphereLight.Define(stage_utils.get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        XformPrim(str(sphereLight.GetPath().pathString), reset_xform_op_properties=True).set_world_poses([6.5, 0, 12])

    async def tearDown(self) -> None:
        """Clean up test environment after test completion.

        Waits for any pending asset loading operations to complete before updating the stage.
        """
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await app_utils.update_app_async()

    # Each ground truth pose for the representative subframes was captured when creating the imported
    # grasp file.  The information in the file should be enough to exactly recover the ground truth pose
    def _compare_rigid_body_pose_to_ground_truth(
        self, grasp_index: int, translation: object, orientation: object
    ) -> None:
        """Compare rigid body pose against recorded ground truth values.

        Validates that the computed pose matches the expected ground truth pose for the
        specified grasp index within tolerance.

        Args:
            grasp_index: Index of the grasp to validate.
            translation: Computed translation to compare.
            orientation: Computed orientation quaternion to compare.
        """
        self._assert_almost_equal(self._ground_truth_rb_translations[grasp_index], translation)
        self._assert_almost_equal(self._ground_truth_rb_quats[grasp_index], orientation, tol=1e-3)

    def _compare_gripper_pose_to_ground_truth(self, translation: object, orientation: object) -> None:
        """Compare gripper pose against recorded ground truth values.

        Validates that the computed gripper pose matches the expected ground truth pose
        within tolerance.

        Args:
            translation: Computed gripper translation to compare.
            orientation: Computed gripper orientation quaternion to compare.
        """
        self._assert_almost_equal(self._ground_truth_gripper_translation, translation)
        self._assert_almost_equal(self._ground_truth_gripper_orientation, orientation, tol=1e-3)

    # Carry out test described in
    async def test_derived_poses(self) -> None:
        """Ground truths were recorded from the Properties Panel while using the Grasp Editor UI.

        to author the grasps in `robotiq_soup_grasp.yaml`.  These ground truths represent the
        state of the gripper and soup for each authored grasp (The gripper was left in one place).

        This test starts validates that give the ground truth location of the soup can subframe, the
        GraspSpec class is able to derive the ground truth position of the gripper subframe.  Likewise,
        given the ground truth position of the gripper subframe, the GraspSpec class can derive the
        ground truth position of the soup can subframe.
        """
        for i, (rb_trans, rb_quat) in enumerate(zip(self._ground_truth_rb_translations, self._ground_truth_rb_quats)):
            t, q = self._grasp_spec.compute_gripper_pose_from_rigid_body_pose(
                self._grasp_spec.get_grasp_names()[i], rb_trans, rb_quat
            )
            self._compare_gripper_pose_to_ground_truth(t, q)

        for i in range(len(self._ground_truth_rb_translations)):
            t, q = self._grasp_spec.compute_rigid_body_pose_from_gripper_pose(
                self._grasp_spec.get_grasp_names()[i],
                self._ground_truth_gripper_translation,
                self._ground_truth_gripper_orientation,
            )
            self._compare_rigid_body_pose_to_ground_truth(i, t, q)

    async def test_move_rb_subframe_to_position(self) -> None:
        """Test moving rigid body subframe to desired position and orientation.

        Validates that the move_rb_subframe_to_position utility function correctly positions
        the soup can subframe at the specified world pose.
        """
        # The move_rb_base_to_position
        desired_trans = np.array([0.123, -2.4, 0.6])
        desired_orient = np.array([0.965, 0.0, 0.0, -0.259])  # -30 degree about Z

        move_rb_subframe_to_position(self._rb_xform, self._rb_subframe.paths[0], desired_trans, desired_orient)

        t, q = self._rb_subframe.get_world_poses()
        t, q = t[0].numpy(), q[0].numpy()

        self._assert_almost_equal(t, desired_trans)
        # Error is expected to accumulate from rotation conversions
        self._assert_almost_equal(q, desired_orient, tol=1e-3)
