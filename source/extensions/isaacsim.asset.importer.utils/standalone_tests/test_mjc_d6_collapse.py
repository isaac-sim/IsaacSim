# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Regression tests for collapsing over-constrained MJCF joint groups into PhysX D6 joints.

Covers isaac-sim/IsaacLab#6854: when several single-axis joints connect the
same body pair (e.g. a 3-DOF humanoid hip authored as three hinges), the
collapse must preserve every source DOF on a distinct D6 axis, keep per-axis
limits and drive parameters on the correct axis, lock every unused D6 axis,
and record which source MJCF joint feeds each used axis.

Runs with plain ``pxr`` (no Omni modules) like test_smoke.py. The fixtures
mirror what mujoco-usd-converter produces: joints are exported x-aligned
(``physics:axis = X``) with the MJCF axis direction encoded in
``localRot0``/``localRot1``.
"""

from __future__ import annotations

import unittest

from pxr import Gf, Sdf, Usd, UsdPhysics

from isaacsim.asset.importer.utils.impl.mjc_to_physx_conversion_utils import (
    combine_overconstrained_joints_to_d6,
)

_ROT_AXES = ("rotX", "rotY", "rotZ")
_TRANS_AXES = ("transX", "transY", "transZ")


def _x_aligned_quat(axis_dir: tuple[float, float, float]) -> Gf.Quatf:
    """Shortest-arc rotation taking local +X to the given MJCF axis direction."""
    rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*axis_dir))
    return Gf.Quatf(rotation.GetQuat())


def _make_revolute_joint(
    stage: Usd.Stage,
    name: str,
    body0: Usd.Prim,
    body1: Usd.Prim,
    axis_dir: tuple[float, float, float],
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
    lower: float | None = None,
    upper: float | None = None,
    stiffness: float | None = None,
    damping: float | None = None,
    max_force: float | None = None,
) -> UsdPhysics.RevoluteJoint:
    """Author a revolute joint the way mujoco-usd-converter does."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(f"/Robot/Joints/{name}"))
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateAxisAttr().Set("X")

    quat = _x_aligned_quat(axis_dir)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos))
    joint.CreateLocalRot0Attr().Set(quat)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*pos))
    joint.CreateLocalRot1Attr().Set(quat)

    if lower is not None and upper is not None:
        joint.CreateLowerLimitAttr().Set(lower)
        joint.CreateUpperLimitAttr().Set(upper)

    if stiffness is not None or damping is not None or max_force is not None:
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        if stiffness is not None:
            drive.CreateStiffnessAttr().Set(stiffness)
        if damping is not None:
            drive.CreateDampingAttr().Set(damping)
        if max_force is not None:
            drive.CreateMaxForceAttr().Set(max_force)
    return joint


def _build_three_hinge_stage() -> Usd.Stage:
    """A humanoid-style hip: three orthogonal hinges between torso and thigh."""
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Robot")
    torso = stage.DefinePrim("/Robot/torso")
    thigh = stage.DefinePrim("/Robot/thigh")
    _make_revolute_joint(
        stage, "hip_x", torso, thigh, (1, 0, 0),
        lower=-30.0, upper=45.0, stiffness=100.0, damping=10.0, max_force=1000.0,
    )
    _make_revolute_joint(
        stage, "hip_y", torso, thigh, (0, 1, 0),
        lower=-20.0, upper=60.0, stiffness=200.0, damping=20.0, max_force=2000.0,
    )
    _make_revolute_joint(
        stage, "hip_z", torso, thigh, (0, 0, 1),
        lower=-90.0, upper=90.0, stiffness=300.0, damping=30.0, max_force=3000.0,
    )
    return stage


def _build_two_hinge_stage() -> Usd.Stage:
    """A 2-DOF joint group (e.g. a wrist): two orthogonal hinges."""
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Robot")
    upper = stage.DefinePrim("/Robot/upper_arm")
    forearm = stage.DefinePrim("/Robot/forearm")
    _make_revolute_joint(
        stage, "wrist_x", upper, forearm, (1, 0, 0),
        lower=-45.0, upper=45.0, stiffness=50.0, damping=5.0, max_force=500.0,
    )
    _make_revolute_joint(
        stage, "wrist_y", upper, forearm, (0, 1, 0),
        lower=-70.0, upper=10.0, stiffness=80.0, damping=8.0, max_force=800.0,
    )
    return stage


def _active_joints(stage: Usd.Stage) -> list[Usd.Prim]:
    return [p for p in stage.TraverseAll() if p.IsA(UsdPhysics.Joint) and p.IsActive()]


def _limit(prim: Usd.Prim, axis: str) -> tuple[float | None, float | None] | None:
    if not prim.HasAPI(UsdPhysics.LimitAPI, axis):
        return None
    api = UsdPhysics.LimitAPI(prim, axis)
    low = api.GetLowAttr()
    high = api.GetHighAttr()
    return (
        low.Get() if low and low.HasAuthoredValue() else None,
        high.Get() if high and high.HasAuthoredValue() else None,
    )


def _drive(prim: Usd.Prim, axis: str) -> dict:
    if not prim.HasAPI(UsdPhysics.DriveAPI, axis):
        return {}
    api = UsdPhysics.DriveAPI(prim, axis)
    out = {}
    for key, getter_name in (
        ("stiffness", "GetStiffnessAttr"),
        ("damping", "GetDampingAttr"),
        ("max_force", "GetMaxForceAttr"),
    ):
        attr = getattr(api, getter_name)()
        if attr and attr.HasAuthoredValue():
            out[key] = attr.Get()
    return out


def _axis_source_name(prim: Usd.Prim, axis: str) -> str | None:
    attr = prim.GetAttribute(f"mjcf:{axis}:name")
    if attr and attr.IsValid() and attr.HasAuthoredValue():
        return str(attr.Get())
    return None


class TestThreeHingeCollapse(unittest.TestCase):
    """Collapsing a 3-hinge group must preserve all three DOFs."""

    def setUp(self) -> None:
        self.stage = _build_three_hinge_stage()
        combine_overconstrained_joints_to_d6(self.stage)
        joints = _active_joints(self.stage)
        self.assertEqual(len(joints), 1, f"expected a single D6 joint, got {joints}")
        self.d6 = joints[0]

    def test_all_source_dofs_survive(self) -> None:
        used = [
            axis
            for axis in _ROT_AXES
            if (limit := _limit(self.d6, axis)) is not None and limit[0] is not None and limit[0] <= limit[1]
        ]
        self.assertEqual(
            len(used), 3, f"expected 3 usable rotation axes, got {used}; duplicate-axis joints were dropped"
        )

    def test_per_axis_limits_preserved(self) -> None:
        authored = {_axis_source_name(self.d6, axis): _limit(self.d6, axis) for axis in _ROT_AXES}
        self.assertEqual(authored["hip_x"], (-30.0, 45.0))
        self.assertEqual(authored["hip_y"], (-20.0, 60.0))
        self.assertEqual(authored["hip_z"], (-90.0, 90.0))

    def test_per_axis_drives_preserved(self) -> None:
        expected = {
            "hip_x": {"stiffness": 100.0, "damping": 10.0, "max_force": 1000.0},
            "hip_y": {"stiffness": 200.0, "damping": 20.0, "max_force": 2000.0},
            "hip_z": {"stiffness": 300.0, "damping": 30.0, "max_force": 3000.0},
        }
        for axis in _ROT_AXES:
            name = _axis_source_name(self.d6, axis)
            self.assertIn(name, expected, f"axis {axis} missing source-joint binding")
            self.assertEqual(_drive(self.d6, axis), expected[name], f"axis {axis} ({name})")

    def test_unused_axes_locked(self) -> None:
        for axis in _TRANS_AXES:
            limit = _limit(self.d6, axis)
            self.assertIsNotNone(limit, f"{axis} has no LimitAPI at all (axis is FREE)")
            self.assertGreater(limit[0], limit[1], f"{axis} not locked: {limit}")

    def test_frame_matches_source_axes(self) -> None:
        rotation = Gf.Rotation(UsdPhysics.Joint(self.d6).GetLocalRot1Attr().Get())
        expected = {"hip_x": Gf.Vec3d(1, 0, 0), "hip_y": Gf.Vec3d(0, 1, 0), "hip_z": Gf.Vec3d(0, 0, 1)}
        d6_dirs = {"rotX": Gf.Vec3d(1, 0, 0), "rotY": Gf.Vec3d(0, 1, 0), "rotZ": Gf.Vec3d(0, 0, 1)}
        for axis in _ROT_AXES:
            name = _axis_source_name(self.d6, axis)
            physical = rotation.TransformDir(d6_dirs[axis])
            self.assertGreater(
                physical * expected[name], 0.999,
                f"{axis} bound to {name} but points along {physical}, expected {expected[name]}",
            )

    def test_non_primary_joints_deactivated(self) -> None:
        inactive = [p for p in self.stage.TraverseAll() if p.IsA(UsdPhysics.Joint) and not p.IsActive()]
        self.assertEqual(len(inactive), 2)

    def test_rerun_is_idempotent(self) -> None:
        self.assertEqual(combine_overconstrained_joints_to_d6(self.stage), 0)
        self.assertEqual(len(_active_joints(self.stage)), 1)


class TestTwoHingeCollapse(unittest.TestCase):
    """Collapsing a 2-hinge group must lock the remaining rotation axis."""

    def setUp(self) -> None:
        self.stage = _build_two_hinge_stage()
        combine_overconstrained_joints_to_d6(self.stage)
        self.d6 = _active_joints(self.stage)[0]

    def test_third_rotation_axis_locked(self) -> None:
        locked = [
            axis
            for axis in _ROT_AXES
            if (limit := _limit(self.d6, axis)) is not None and limit[0] is not None and limit[0] > limit[1]
        ]
        used = [axis for axis in _ROT_AXES if axis not in locked]
        self.assertEqual(len(used), 2, f"expected 2 used rotation axes, got {used}")
        self.assertEqual(len(locked), 1, f"expected the unused rotation axis locked, got {locked}")
        for axis in _TRANS_AXES:
            limit = _limit(self.d6, axis)
            self.assertIsNotNone(limit)
            self.assertGreater(limit[0], limit[1], f"{axis} not locked: {limit}")

    def test_axis_binding_metadata(self) -> None:
        bound = sorted(name for axis in _ROT_AXES if (name := _axis_source_name(self.d6, axis)))
        self.assertEqual(bound, ["wrist_x", "wrist_y"])


class TestEdgeCases(unittest.TestCase):
    """Degenerate and inconsistent source groups."""

    def test_left_handed_axis_triple_flips_limits(self) -> None:
        stage = _build_three_hinge_stage()
        stage.RemovePrim("/Robot/Joints/hip_z")
        _make_revolute_joint(
            stage, "hip_z", stage.GetPrimAtPath("/Robot/torso"), stage.GetPrimAtPath("/Robot/thigh"),
            (0, 0, -1), lower=-90.0, upper=90.0, stiffness=300.0, damping=30.0, max_force=3000.0,
        )
        combine_overconstrained_joints_to_d6(stage)
        d6 = _active_joints(stage)[0]

        bound_axis = next(axis for axis in _ROT_AXES if _axis_source_name(d6, axis) == "hip_z")
        self.assertEqual(_limit(d6, bound_axis), (-90.0, 90.0))

        # The D6 frame must remain right-handed.
        rotation = Gf.Rotation(UsdPhysics.Joint(d6).GetLocalRot1Attr().Get())
        x = rotation.TransformDir(Gf.Vec3d(1, 0, 0))
        y = rotation.TransformDir(Gf.Vec3d(0, 1, 0))
        z = rotation.TransformDir(Gf.Vec3d(0, 0, 1))
        self.assertGreater(Gf.Cross(x, y) * z, 0.999)

        # The bound axis must point along the source joint's physical axis (0, 0, -1).
        d6_dirs = {"rotX": Gf.Vec3d(1, 0, 0), "rotY": Gf.Vec3d(0, 1, 0), "rotZ": Gf.Vec3d(0, 0, 1)}
        physical = rotation.TransformDir(d6_dirs[bound_axis])
        self.assertGreater(abs(physical * Gf.Vec3d(0, 0, -1)), 0.999)

    def test_true_duplicate_axis_dropped_but_others_survive(self) -> None:
        stage = _build_three_hinge_stage()
        _make_revolute_joint(
            stage, "hip_x_dup", stage.GetPrimAtPath("/Robot/torso"), stage.GetPrimAtPath("/Robot/thigh"),
            (1, 0, 0), lower=-10.0, upper=10.0, stiffness=1.0, damping=1.0,
        )
        combine_overconstrained_joints_to_d6(stage)
        d6 = _active_joints(stage)[0]

        bound = sorted(name for axis in _ROT_AXES if (name := _axis_source_name(d6, axis)))
        self.assertEqual(bound, ["hip_x", "hip_y", "hip_z"])
        self.assertFalse(stage.GetPrimAtPath("/Robot/Joints/hip_x_dup").IsActive())

    def test_unlimited_hinge_axis_stays_free(self) -> None:
        stage = _build_three_hinge_stage()
        stage.RemovePrim("/Robot/Joints/hip_z")
        _make_revolute_joint(
            stage, "hip_z", stage.GetPrimAtPath("/Robot/torso"), stage.GetPrimAtPath("/Robot/thigh"), (0, 0, 1)
        )
        combine_overconstrained_joints_to_d6(stage)
        d6 = _active_joints(stage)[0]

        bound_axis = next(axis for axis in _ROT_AXES if _axis_source_name(d6, axis) == "hip_z")
        limit = _limit(d6, bound_axis)
        self.assertTrue(limit is None or limit == (None, None), f"unlimited hinge got limits {limit}")
        for axis in _TRANS_AXES:
            locked = _limit(d6, axis)
            self.assertIsNotNone(locked)
            self.assertGreater(locked[0], locked[1], f"{axis} not locked: {locked}")

    def test_mismatched_joint_positions_warn(self) -> None:
        stage = _build_three_hinge_stage()
        stage.RemovePrim("/Robot/Joints/hip_z")
        _make_revolute_joint(
            stage, "hip_z", stage.GetPrimAtPath("/Robot/torso"), stage.GetPrimAtPath("/Robot/thigh"),
            (0, 0, 1), pos=(0.05, 0.0, 0.0), lower=-90.0, upper=90.0,
        )
        with self.assertLogs("isaacsim.asset.importer.utils.impl.mjc_to_physx_conversion_utils", level="WARNING") as logs:
            combine_overconstrained_joints_to_d6(stage)
        self.assertTrue(
            any("different" in message and "position" in message for message in logs.output),
            f"no joint-frame-mismatch warning in {logs.output}",
        )


if __name__ == "__main__":
    unittest.main()
