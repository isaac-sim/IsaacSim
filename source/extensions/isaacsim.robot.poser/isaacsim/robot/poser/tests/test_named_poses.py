# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for Robot Poser named-pose CRUD, import/export, and validation."""

import json
import logging
import os
import tempfile

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.robot.poser.robot_poser import (
    NAMED_POSES_SCOPE,
    PoseResult,
    _build_cold_start_seeds,
    _sanitize_name,
    delete_named_pose,
    export_poses,
    get_named_pose,
    import_poses,
    list_named_poses,
    store_named_pose,
    validate_robot_schema,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema
from usd.schema.isaac.robot_schema import utils as robot_utils
from usd.schema.isaac.robot_schema.math import Joint, Transform


def _create_test_robot(stage: Usd.Stage) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim]:
    """Create a two-link robot with a revolute joint and RobotAPI.

    Shared fixture used by multiple test classes.

    Args:
        stage: USD stage to define prims on.

    Returns:
        Tuple of (robot_prim, link1_prim, joint_prim).
    """
    robot_xform = UsdGeom.Xform.Define(stage, "/World/Robot")
    UsdPhysics.RigidBodyAPI.Apply(robot_xform.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

    link1 = UsdGeom.Xform.Define(stage, "/World/Robot/Link1")
    UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())

    joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/joint1")
    joint.CreateBody0Rel().SetTargets([robot_xform.GetPrim().GetPath()])
    joint.CreateBody1Rel().SetTargets([link1.GetPrim().GetPath()])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    robot_schema.ApplyRobotAPI(robot_xform.GetPrim())
    return robot_xform.GetPrim(), link1.GetPrim(), joint.GetPrim()


class TestNamedPoseCRUD(omni.kit.test.AsyncTestCase):
    """Async tests for named-pose create, read, update, and delete."""

    async def setUp(self) -> None:
        """Create a fresh USD stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot(self) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim]:
        """Create the test robot and return (robot_prim, link_prim, joint_prim).

        Returns:
            Tuple of (robot_prim, link_prim, joint_prim).
        """
        return _create_test_robot(self._stage)

    def _make_pose_result(self, start: str = "/World/Robot", end: str = "/World/Robot/Link1") -> PoseResult:
        """Return a valid PoseResult with one revolute joint value (in radians).

        Args:
            start: Path of the start link.
            end: Path of the end link.

        Returns:
            A PoseResult with one joint at 45 degrees.
        """
        return PoseResult(
            success=True,
            joints={"/World/Robot/joint1": float(np.radians(45))},
            joint_fixed={"/World/Robot/joint1": False},
            start_link=start,
            end_link=end,
            target_position=[1.0, 0.0, 0.0],
            target_orientation=[1.0, 0.0, 0.0, 0.0],
        )

    # -- validate_robot_schema ---------------------------------------------

    async def test_validate_robot_schema_true(self) -> None:
        """Verify that validate_robot_schema returns True for a prim with RobotAPI."""
        robot_prim, _, _ = self._create_robot()
        self.assertTrue(validate_robot_schema(robot_prim))

    async def test_validate_robot_schema_false(self) -> None:
        """Verify that validate_robot_schema returns False for a prim without RobotAPI."""
        prim = UsdGeom.Xform.Define(self._stage, "/World/Plain").GetPrim()
        self.assertFalse(validate_robot_schema(prim))

    # -- _sanitize_name ----------------------------------------------------

    async def test_sanitize_name(self) -> None:
        """Verify that _sanitize_name replaces spaces and disallowed chars with underscores."""
        self.assertEqual(_sanitize_name("Home Pose"), "Home_Pose")
        self.assertEqual(_sanitize_name("a/b.c"), "a_b_c")

    async def test_sanitize_name_handles_colon(self) -> None:
        """Verify that colon is replaced (was previously passed through, crashing USD)."""
        self.assertEqual(_sanitize_name("home:v2"), "home_v2")
        self.assertEqual(_sanitize_name("ns:pose:1"), "ns_pose_1")

    async def test_sanitize_name_handles_empty_and_invalid_leading(self) -> None:
        """Verify that empty and non-identifier-start inputs become valid USD identifiers."""
        self.assertTrue(Sdf.Path.IsValidIdentifier(_sanitize_name("")))
        self.assertTrue(Sdf.Path.IsValidIdentifier(_sanitize_name("1pose")))
        self.assertTrue(Sdf.Path.IsValidIdentifier(_sanitize_name("home:v2")))
        self.assertTrue(Sdf.Path.IsValidIdentifier(_sanitize_name(":::")))

    async def test_sanitize_name_handles_non_ascii(self) -> None:
        """Verify that non-ASCII letters/digits are rejected by USD and replaced."""
        # str.isalnum() accepts Unicode; Sdf.Path.IsValidIdentifier does not.
        # Tf.MakeValidIdentifier replaces non-ASCII with underscores.
        for raw in ("café", "位姿", "²pose", "naïve_pose", "pose-α"):
            sanitized = _sanitize_name(raw)
            self.assertTrue(
                Sdf.Path.IsValidIdentifier(sanitized),
                f"_sanitize_name({raw!r}) returned invalid identifier {sanitized!r}",
            )

    async def test_store_named_pose_with_colon_and_empty(self) -> None:
        """Verify that previously crash-inducing inputs now store successfully."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        self.assertTrue(store_named_pose(self._stage, robot_prim, "home:v2", result))
        self.assertTrue(store_named_pose(self._stage, robot_prim, "", result))

        names = list_named_poses(self._stage, robot_prim)
        self.assertIn(_sanitize_name("home:v2"), names)
        self.assertIn(_sanitize_name(""), names)

    async def test_store_named_pose_warns_on_sanitized_collision(self) -> None:
        """Verify that overwriting a sanitized name with a different raw input emits a warning."""
        import logging

        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        self.assertTrue(store_named_pose(self._stage, robot_prim, "home:v2", result))
        with self.assertLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING) as cm:
            self.assertTrue(store_named_pose(self._stage, robot_prim, "home/v2", result))
        self.assertTrue(any("sanitizes to" in msg for msg in cm.output))

    # -- store_named_pose --------------------------------------------------

    async def test_store_named_pose_creates_prim(self) -> None:
        """Verify that store_named_pose creates an IsaacNamedPose prim under Named_Poses scope."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        ok = store_named_pose(self._stage, robot_prim, "Home", result)
        self.assertTrue(ok)

        scope = self._stage.GetPrimAtPath(robot_prim.GetPath().AppendChild(NAMED_POSES_SCOPE))
        self.assertTrue(scope.IsValid())

        pose_prim = self._stage.GetPrimAtPath(scope.GetPath().AppendChild("Home"))
        self.assertTrue(pose_prim.IsValid())
        self.assertEqual(pose_prim.GetTypeName(), robot_schema.Classes.NAMED_POSE.value)

    async def test_store_named_pose_sets_attributes(self) -> None:
        """Verify that store_named_pose writes joint values and valid to the pose prim."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        store_named_pose(self._stage, robot_prim, "Home", result)

        pose_path = robot_prim.GetPath().AppendChild(NAMED_POSES_SCOPE).AppendChild("Home")
        pose_prim = self._stage.GetPrimAtPath(pose_path)

        self.assertTrue(robot_utils.GetNamedPoseValid(pose_prim))
        values = robot_utils.GetNamedPoseJointValues(pose_prim)
        self.assertIsNotNone(values)
        self.assertAlmostEqual(float(values[0]), 45.0, places=4)

    async def test_store_named_pose_sets_relationships(self) -> None:
        """Verify that store_named_pose sets start_link, end_link, and joints relationships."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        store_named_pose(self._stage, robot_prim, "Home", result)

        pose_path = robot_prim.GetPath().AppendChild(NAMED_POSES_SCOPE).AppendChild("Home")
        pose_prim = self._stage.GetPrimAtPath(pose_path)

        self.assertEqual(
            robot_utils.GetNamedPoseStartLink(pose_prim),
            Sdf.Path("/World/Robot"),
        )
        self.assertEqual(
            robot_utils.GetNamedPoseEndLink(pose_prim),
            Sdf.Path("/World/Robot/Link1"),
        )
        joints = robot_utils.GetNamedPoseJoints(pose_prim)
        self.assertEqual(len(joints), 1)

    async def test_store_named_pose_registers_in_robot_relationship(self) -> None:
        """Verify that store_named_pose adds the pose to the robot's namedPoses relationship."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        store_named_pose(self._stage, robot_prim, "Home", result)

        poses = robot_utils.GetAllNamedPoses(self._stage, robot_prim)
        self.assertEqual(len(poses), 1)

    async def test_store_named_pose_rejects_failed_result(self) -> None:
        """Verify that store_named_pose returns False when PoseResult.success is False."""
        robot_prim, _, _ = self._create_robot()
        result = PoseResult(success=False)
        ok = store_named_pose(self._stage, robot_prim, "Bad", result)
        self.assertFalse(ok)

    async def test_store_named_pose_xform_ops(self) -> None:
        """Verify that stored pose has translate and orient xform ops matching target."""
        robot_prim, _, _ = self._create_robot()
        result = self._make_pose_result()
        store_named_pose(self._stage, robot_prim, "Home", result)

        pose_path = robot_prim.GetPath().AppendChild(NAMED_POSES_SCOPE).AppendChild("Home")
        pose_prim = self._stage.GetPrimAtPath(pose_path)
        xformable = UsdGeom.Xformable(pose_prim)

        has_translate = False
        has_orient = False
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = op.Get()
                self.assertAlmostEqual(float(v[0]), 1.0)
                has_translate = True
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                has_orient = True
        self.assertTrue(has_translate)
        self.assertTrue(has_orient)

    # -- get_named_pose ----------------------------------------------------

    async def test_get_named_pose_roundtrip(self) -> None:
        """Verify that get_named_pose returns stored joint values and links after store_named_pose."""
        robot_prim, _, _ = self._create_robot()
        original = self._make_pose_result()
        store_named_pose(self._stage, robot_prim, "Home", original)

        retrieved = get_named_pose(self._stage, robot_prim, "Home")
        self.assertIsNotNone(retrieved)
        self.assertTrue(retrieved.success)
        self.assertEqual(retrieved.start_link, "/World/Robot")
        self.assertEqual(retrieved.end_link, "/World/Robot/Link1")
        self.assertIn("/World/Robot/joint1", retrieved.joints)
        self.assertAlmostEqual(
            retrieved.joints["/World/Robot/joint1"],
            original.joints["/World/Robot/joint1"],
            places=4,
        )

    async def test_get_named_pose_nonexistent(self) -> None:
        """Verify that get_named_pose returns None when the pose name does not exist."""
        robot_prim, _, _ = self._create_robot()
        self.assertIsNone(get_named_pose(self._stage, robot_prim, "DoesNotExist"))

    async def test_get_named_pose_no_scope(self) -> None:
        """Verify that get_named_pose returns None when Named_Poses scope does not exist."""
        robot_prim, _, _ = self._create_robot()
        self.assertIsNone(get_named_pose(self._stage, robot_prim, "Any"))

    # -- list_named_poses --------------------------------------------------

    async def test_list_named_poses(self) -> None:
        """Verify that list_named_poses returns names of all stored poses for the robot."""
        robot_prim, _, _ = self._create_robot()
        store_named_pose(self._stage, robot_prim, "A", self._make_pose_result())
        store_named_pose(self._stage, robot_prim, "B", self._make_pose_result())

        names = list_named_poses(self._stage, robot_prim)
        self.assertIn("A", names)
        self.assertIn("B", names)
        self.assertEqual(len(names), 2)

    async def test_list_named_poses_empty(self) -> None:
        """Verify that list_named_poses returns empty list when no poses are stored."""
        robot_prim, _, _ = self._create_robot()
        self.assertEqual(list_named_poses(self._stage, robot_prim), [])

    # -- delete_named_pose -------------------------------------------------

    async def test_delete_named_pose(self) -> None:
        """Verify that delete_named_pose removes the pose prim and from robot relationship."""
        robot_prim, _, _ = self._create_robot()
        store_named_pose(self._stage, robot_prim, "Del", self._make_pose_result())

        self.assertTrue(delete_named_pose(self._stage, robot_prim, "Del"))
        self.assertEqual(list_named_poses(self._stage, robot_prim), [])
        self.assertIsNone(get_named_pose(self._stage, robot_prim, "Del"))

    async def test_delete_named_pose_nonexistent(self) -> None:
        """Verify that delete_named_pose returns False when the pose name does not exist."""
        robot_prim, _, _ = self._create_robot()
        self.assertFalse(delete_named_pose(self._stage, robot_prim, "Ghost"))

    async def test_delete_named_pose_removes_from_relationship(self) -> None:
        """Verify that delete_named_pose removes only the target from namedPoses relationship."""
        robot_prim, _, _ = self._create_robot()
        store_named_pose(self._stage, robot_prim, "A", self._make_pose_result())
        store_named_pose(self._stage, robot_prim, "B", self._make_pose_result())

        delete_named_pose(self._stage, robot_prim, "A")
        remaining = list_named_poses(self._stage, robot_prim)
        self.assertEqual(remaining, ["B"])


class TestNamedPoseImportExport(omni.kit.test.AsyncTestCase):
    """Tests for export_poses / import_poses JSON round-trip."""

    async def setUp(self) -> None:
        """Create a fresh USD stage before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def _create_robot(self) -> Usd.Prim:
        """Create the test robot and return the robot prim.

        Returns:
            The robot root prim.
        """
        robot_prim, _, _ = _create_test_robot(self._stage)
        return robot_prim

    async def test_export_import_roundtrip(self) -> None:
        """Verify that export to JSON and import restores named poses on another robot."""
        robot_prim = self._create_robot()
        pose = PoseResult(
            success=True,
            joints={"/World/Robot/joint1": float(np.radians(30))},
            joint_fixed={"/World/Robot/joint1": True},
            start_link="/World/Robot",
            end_link="/World/Robot/Link1",
            target_position=[0.5, 0.0, 0.0],
            target_orientation=[1.0, 0.0, 0.0, 0.0],
        )
        store_named_pose(self._stage, robot_prim, "Export_Test", pose)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as fh:
            path = fh.name

        try:
            ok = export_poses(self._stage, robot_prim, path)
            self.assertTrue(ok)

            with open(path) as fh:
                data = json.load(fh)
            self.assertIn("_meta", data)
            self.assertEqual(data["_meta"]["units"], "radians")
            self.assertIn("note", data["_meta"])
            self.assertIn("Export_Test", data["poses"])
            self.assertTrue(data["poses"]["Export_Test"]["valid"])

            # Import into a fresh robot on the same stage
            robot2 = UsdGeom.Xform.Define(self._stage, "/World/Robot2")
            UsdPhysics.RigidBodyAPI.Apply(robot2.GetPrim())
            UsdPhysics.ArticulationRootAPI.Apply(robot2.GetPrim())
            link2 = UsdGeom.Xform.Define(self._stage, "/World/Robot2/Link1")
            UsdPhysics.RigidBodyAPI.Apply(link2.GetPrim())
            joint2 = UsdPhysics.RevoluteJoint.Define(self._stage, "/World/Robot2/joint1")
            joint2.CreateBody0Rel().SetTargets([robot2.GetPrim().GetPath()])
            joint2.CreateBody1Rel().SetTargets([link2.GetPrim().GetPath()])
            joint2.CreateAxisAttr("X")
            joint2.CreateLocalPos0Attr().Set(Gf.Vec3f(1.0, 0.0, 0.0))
            joint2.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint2.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint2.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            robot_schema.ApplyRobotAPI(robot2.GetPrim())

            count = import_poses(self._stage, robot2.GetPrim(), path)
            self.assertEqual(count, 1)

            names = list_named_poses(self._stage, robot2.GetPrim())
            self.assertIn("Export_Test", names)
        finally:
            os.unlink(path)

    async def test_export_empty_poses(self) -> None:
        """Verify that export_poses writes empty dict when robot has no named poses."""
        robot_prim = self._create_robot()
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as fh:
            path = fh.name
        try:
            ok = export_poses(self._stage, robot_prim, path)
            self.assertTrue(ok)
            with open(path) as fh:
                data = json.load(fh)
            self.assertIn("_meta", data)
            self.assertIn("note", data["_meta"])
            self.assertEqual(data["poses"], {})
        finally:
            os.unlink(path)

    async def test_import_multiple_poses(self) -> None:
        """Verify that import_poses loads multiple poses from a JSON file."""
        robot_prim = self._create_robot()
        data = {
            "_meta": {
                "units": "radians",
                "note": "Revolute joint values are stored in radians. USD natively uses degrees.",
            },
            "poses": {
                "Pose_1": {
                    "joints": {"/World/Robot/joint1": 0.5},
                    "joint_fixed": {"/World/Robot/joint1": False},
                    "start_link": "/World/Robot",
                    "end_link": "/World/Robot/Link1",
                    "target_position": [1.0, 0.0, 0.0],
                    "target_orientation": [1.0, 0.0, 0.0, 0.0],
                    "valid": True,
                },
                "Pose_2": {
                    "joints": {"/World/Robot/joint1": 1.0},
                    "joint_fixed": {"/World/Robot/joint1": True},
                    "start_link": "/World/Robot",
                    "end_link": "/World/Robot/Link1",
                    "target_position": [0.5, 0.5, 0.0],
                    "target_orientation": [1.0, 0.0, 0.0, 0.0],
                    "valid": True,
                },
            },
        }
        with tempfile.NamedTemporaryFile(suffix=".json", mode="w", delete=False) as fh:
            json.dump(data, fh)
            path = fh.name
        try:
            count = import_poses(self._stage, robot_prim, path)
            self.assertEqual(count, 2)
            names = list_named_poses(self._stage, robot_prim)
            self.assertEqual(sorted(names), ["Pose_1", "Pose_2"])
        finally:
            os.unlink(path)


class TestColdStartSeedLadder(omni.kit.test.AsyncTestCase):
    """Unit tests for the multi-seed cold-start ladder used by ``RobotPoser.solve_ik``.

    These tests do not require a USD stage or simulation — they exercise
    the pure-Python seed-generation helper directly with synthetic Joint
    objects.  Their purpose is to lock in the priority order, joint-limit
    handling, and determinism guarantees that ``solve_ik``'s cold-start
    path relies on to escape the Franka 7-DOF zero-config local minimum.
    """

    @staticmethod
    def _make_joint(prim_path: str, lower: float, upper: float) -> Joint:
        """Build a minimal Joint with the requested limits.

        The screw axis, home pose, and joint type are irrelevant for the
        seed-ladder helper — only ``lower``/``upper`` are read.

        Args:
            prim_path: Joint prim path to store on the joint.
            lower: Lower joint limit.
            upper: Upper joint limit.

        Returns:
            Minimal joint with the requested limits.
        """
        identity = Transform(t=[0.0, 0.0, 0.0], q=[1.0, 0.0, 0.0, 0.0])
        return Joint(
            w=np.array([0.0, 0.0, 1.0], dtype=float),
            v=np.array([0.0, 0.0, 0.0], dtype=float),
            home=identity,
            prim_path=prim_path,
            lower=float(lower),
            upper=float(upper),
        )

    async def test_empty_chain_returns_single_zero_array(self) -> None:
        """Verify that an empty joint list yields a single empty seed (no crash)."""
        seeds = _build_cold_start_seeds([])
        self.assertEqual(len(seeds), 1)
        self.assertEqual(seeds[0].shape, (0,))

    async def test_first_seed_is_joint_limit_midpoint(self) -> None:
        """Verify the highest-priority candidate is the joint-limit midpoint."""
        joints = [
            self._make_joint("/r/j0", -2.0, 4.0),
            self._make_joint("/r/j1", 0.0, 1.0),
            self._make_joint("/r/j2", -1.5, 1.5),
        ]
        seeds = _build_cold_start_seeds(joints, n_random=2)
        np.testing.assert_allclose(seeds[0], [1.0, 0.5, 0.0])

    async def test_zero_configuration_is_final_fallback(self) -> None:
        """Verify the legacy zero-configuration seed is the final candidate."""
        joints = [self._make_joint(f"/r/j{i}", -1.0, 1.0) for i in range(4)]
        seeds = _build_cold_start_seeds(joints, n_random=3)
        np.testing.assert_array_equal(seeds[-1], np.zeros(4))

    async def test_random_seeds_respect_finite_joint_limits(self) -> None:
        """Verify random restarts stay inside authored joint limits."""
        lo = [-1.5, -3.0, 0.5, -0.25]
        hi = [1.5, 3.0, 2.0, 0.25]
        joints = [self._make_joint(f"/r/j{i}", lo[i], hi[i]) for i in range(4)]
        seeds = _build_cold_start_seeds(joints, n_random=5)
        # Skip seed[0] (midpoint) and seed[-1] (zeros); inspect random restarts.
        for q in seeds[1:-1]:
            self.assertTrue(np.all(q >= np.array(lo)))
            self.assertTrue(np.all(q <= np.array(hi)))

    async def test_unbounded_joint_midpoint_is_zero(self) -> None:
        """Verify joints without finite limits use 0.0 as the midpoint."""
        joints = [
            self._make_joint("/r/j0", -np.inf, np.inf),
            self._make_joint("/r/j1", -2.0, 2.0),
        ]
        seeds = _build_cold_start_seeds(joints, n_random=0)
        np.testing.assert_allclose(seeds[0], [0.0, 0.0])

    async def test_seed_ladder_is_deterministic(self) -> None:
        """Verify two invocations produce the same ladder (reproducibility)."""
        joints = [self._make_joint(f"/r/j{i}", -np.pi, np.pi) for i in range(7)]
        seeds_a = _build_cold_start_seeds(joints, n_random=3)
        seeds_b = _build_cold_start_seeds(joints, n_random=3)
        self.assertEqual(len(seeds_a), len(seeds_b))
        for a, b in zip(seeds_a, seeds_b):
            np.testing.assert_array_equal(a, b)

    async def test_ladder_length_is_one_plus_random_plus_one(self) -> None:
        """Verify ladder length: midpoint + n_random restarts + zeros."""
        joints = [self._make_joint(f"/r/j{i}", -1.0, 1.0) for i in range(3)]
        for n_random in (0, 1, 5):
            seeds = _build_cold_start_seeds(joints, n_random=n_random)
            self.assertEqual(len(seeds), 1 + n_random + 1)


class TestColdStartSolveIK(omni.kit.test.AsyncTestCase):
    """Behavioral tests for ``RobotPoser.solve_ik`` seeding strategy.

    Uses the existing 1-DOF revolute fixture.  The fixture's joint has no
    authored limits, so the midpoint candidate equals the legacy zeros and
    backward compatibility is preserved.  These tests cover:

    * explicit ``seed=`` short-circuits the ladder,
    * a cached ``_last_solution`` runs first and skips the ladder when it
      converges,
    * a cached ``_last_solution`` that does NOT converge falls back to the
      cold-start ladder (regression for the stale-cache trap),
    * cold-start failure on an unreachable target emits a single warning
      while still returning the lowest-error attempt,
    * :meth:`RobotPoser.apply_pose` refuses to apply a failed
      :class:`PoseResult`.
    """

    async def setUp(self) -> None:
        """Create a fresh USD stage and 1-DOF revolute robot fixture before each test."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self._robot, self._link, self._joint = _create_test_robot(self._stage)

    async def test_cold_start_unreachable_target_warns_and_returns_best_attempt(self) -> None:
        """Cold-start IK failure logs a warning and returns the lowest-error attempt."""
        from isaacsim.robot.poser import RobotPoser, Transform

        poser = RobotPoser(self._stage, self._robot, self._robot, self._link)
        # Place the target far outside the chain's reachable workspace.  The
        # 1-DOF fixture cannot translate the end link in z, so this is
        # guaranteed to fail for every seed in the ladder.
        target = Transform(t=[1.0, 0.0, 5.0], q=[1.0, 0.0, 0.0, 0.0])

        with self.assertLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING) as cm:
            result = poser.solve_ik(target)

        self.assertFalse(result.success)
        # Joints are populated with the best (lowest-error) attempt.
        self.assertEqual(set(result.joints.keys()), {"/World/Robot/joint1"})
        self.assertTrue(any("cold-start IK failed" in msg for msg in cm.output))

    async def test_explicit_seed_skips_cold_start_ladder(self) -> None:
        """Caller-supplied seed disables the cold-start ladder (no cold-start warning)."""
        from isaacsim.robot.poser import RobotPoser, Transform

        poser = RobotPoser(self._stage, self._robot, self._robot, self._link)
        target = Transform(t=[1.0, 0.0, 0.0], q=[1.0, 0.0, 0.0, 0.0])

        with self.assertNoLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING):
            _ = poser.solve_ik(target, seed=[0.0])

    async def test_cached_last_solution_skips_cold_start_ladder(self) -> None:
        """A cached _last_solution that converges short-circuits the ladder."""
        from isaacsim.robot.poser import RobotPoser, Transform

        poser = RobotPoser(self._stage, self._robot, self._robot, self._link)
        # Pre-cache a solution so the cached-seed path is exercised.
        poser.set_seed([0.5])
        target = Transform(t=[1.0, 0.0, 0.0], q=[1.0, 0.0, 0.0, 0.0])

        with self.assertNoLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING):
            _ = poser.solve_ik(target)

    async def test_stale_cached_solution_falls_back_to_ladder(self) -> None:
        """Regression: a non-converging cached ``_last_solution`` must trigger the cold-start ladder.

        Previously, a stale cache from a far-away previous target would seed
        the solver once and short-circuit the ladder. If that single attempt
        landed in the wrong convergence basin, ``success=False`` was returned
        even when the ladder could have converged. The fallback path runs the
        full ladder when the cached attempt does not converge.
        """
        from isaacsim.robot.poser import RobotPoser, Transform

        poser = RobotPoser(self._stage, self._robot, self._robot, self._link)
        # Force a cache that cannot converge to the target. For the 1-DOF
        # fixture the cache is itself probably good enough, so target an
        # unreachable z to ensure the cached attempt fails. The point under
        # test is the WARNING surface: when cold-start fallback runs we get
        # the cold-start warning; without fallback we would get nothing.
        poser.set_seed([1.234])
        target = Transform(t=[1.0, 0.0, 7.0], q=[1.0, 0.0, 0.0, 0.0])

        with self.assertLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING) as cm:
            result = poser.solve_ik(target)

        self.assertFalse(result.success)
        self.assertTrue(
            any("cold-start IK failed" in msg and "stale-cache fallback" in msg for msg in cm.output),
            f"Expected stale-cache fallback warning, got: {cm.output}",
        )

    async def test_apply_pose_refuses_failed_pose_result(self) -> None:
        """Regression: ``apply_pose(PoseResult)`` with ``success=False`` must be a no-op."""
        from isaacsim.robot.poser import PoseResult, RobotPoser

        poser = RobotPoser(self._stage, self._robot, self._robot, self._link)
        failed = PoseResult(
            success=False,
            joints={"/World/Robot/joint1": 1.234},
            joint_fixed={"/World/Robot/joint1": False},
            start_link="/World/Robot",
            end_link="/World/Robot/link1",
        )

        with self.assertLogs("isaacsim.robot.poser.robot_poser", level=logging.WARNING) as cm:
            poser.apply_pose(failed)
        self.assertTrue(
            any("refusing to apply a failed PoseResult" in msg for msg in cm.output),
            f"Expected refusal warning, got: {cm.output}",
        )
