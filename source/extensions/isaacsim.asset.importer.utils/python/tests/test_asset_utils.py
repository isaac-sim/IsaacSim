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

"""Test asset utils functionality."""

import math

import omni.kit.test
import omni.usd
from isaacsim.asset.importer.utils.impl import asset_utils
from pxr import Sdf, Usd, UsdPhysics


def _make_stage() -> Usd.Stage:
    stage = Usd.Stage.CreateInMemory()
    root = stage.DefinePrim("/robot", "Xform")
    stage.SetDefaultPrim(root)
    return stage


def _add_rigid_link(stage: Usd.Stage, path: str) -> Usd.Prim:
    prim = stage.DefinePrim(path, "Xform")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    return prim


def _add_revolute_joint(stage: Usd.Stage, path: str, parent_path: str, child_path: str) -> Usd.Prim:
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    return joint.GetPrim()


def _add_prismatic_joint(stage: Usd.Stage, path: str, parent_path: str, child_path: str) -> Usd.Prim:
    joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    return joint.GetPrim()


def _add_fixed_joint(
    stage: Usd.Stage,
    path: str,
    parent_path: str | None,
    child_path: str,
) -> Usd.Prim:
    fj = UsdPhysics.FixedJoint.Define(stage, path)
    if parent_path is not None:
        fj.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    return fj.GetPrim()


def _make_robot() -> Usd.Stage:
    stage = _make_stage()
    _add_rigid_link(stage, "/robot/base")
    _add_rigid_link(stage, "/robot/link1")
    _add_rigid_link(stage, "/robot/link2")
    _add_revolute_joint(stage, "/robot/shoulder_joint", "/robot/base", "/robot/link1")
    _add_prismatic_joint(stage, "/robot/slider_joint", "/robot/link1", "/robot/link2")
    return stage


class TestAssetUtils(omni.kit.test.AsyncTestCase):
    """Test helpers in :mod:`isaacsim.asset.importer.utils.impl.asset_utils`.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...
    """

    async def setUp(self) -> None:
        """Prepare a new USD stage for each test.

        Example:

        .. code-block:: python

            >>> import omni.usd
            >>> omni.usd.get_context()
            <...>
        """
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # -- _apply_to_joints ----------------------------------------------------

    async def test_apply_to_joints_scalar_applies_to_all(self) -> None:
        """Scalar spec should be dispatched to every joint."""
        joints = {
            "shoulder_revolute": ("prim_a", True, "angular"),
            "elbow_revolute": ("prim_b", True, "angular"),
            "finger_prismatic": ("prim_c", False, "linear"),
        }
        calls = []

        def fn(prim, inst, value):
            calls.append((prim, inst, value))

        asset_utils._apply_to_joints(joints, 42, fn)
        self.assertEqual(len(calls), 3)
        for _, _, v in calls:
            self.assertEqual(v, 42)

    async def test_apply_to_joints_dict_pattern_matches_subset(self) -> None:
        """Dict spec with literal patterns should only match named joints."""
        joints = {
            "shoulder_revolute": ("prim_a", True, "angular"),
            "elbow_revolute": ("prim_b", True, "angular"),
            "finger_prismatic": ("prim_c", False, "linear"),
        }
        calls = []

        def fn(prim, inst, value):
            calls.append((prim, value))

        asset_utils._apply_to_joints(joints, {"shoulder": 10, "finger": 20}, fn)
        values = {prim: val for prim, val in calls}
        self.assertEqual(values["prim_a"], 10)
        self.assertEqual(values["prim_c"], 20)
        self.assertNotIn("prim_b", values)

    async def test_apply_to_joints_regex_pattern(self) -> None:
        """Regex pattern should match multiple joints."""
        joints = {
            "shoulder_revolute": ("prim_a", True, "angular"),
            "elbow_revolute": ("prim_b", True, "angular"),
            "finger_prismatic": ("prim_c", False, "linear"),
        }
        calls = []

        def fn(prim, inst, value):
            calls.append(prim)

        asset_utils._apply_to_joints(joints, {".*revolute": 1}, fn)
        self.assertEqual(len(calls), 2)
        self.assertIn("prim_a", calls)
        self.assertIn("prim_b", calls)

    async def test_apply_to_joints_unmatched_pattern_raises(self) -> None:
        """A pattern matching no joints should raise ValueError."""
        joints = {
            "shoulder_revolute": ("prim_a", True, "angular"),
        }
        with self.assertRaises(ValueError):
            asset_utils._apply_to_joints(joints, {"no_such_joint": 1}, lambda *a: None)

    async def test_apply_to_joints_pass_is_revolute(self) -> None:
        """is_revolute kwarg should be forwarded when requested."""
        joints = {
            "shoulder_revolute": ("prim_a", True, "angular"),
            "finger_prismatic": ("prim_c", False, "linear"),
        }
        kw_received = []

        def fn(prim, inst, value, *, is_revolute=False):
            kw_received.append((prim, is_revolute))

        asset_utils._apply_to_joints(joints, 0, fn, pass_is_revolute=True)
        revolute_flags = {p: r for p, r in kw_received}
        self.assertTrue(revolute_flags["prim_a"])
        self.assertFalse(revolute_flags["prim_c"])

    # -- _collect_joints -----------------------------------------------------

    async def test_collect_joints_revolute_and_prismatic(self) -> None:
        """Revolute and prismatic joints should be collected with correct metadata."""
        stage = _make_robot()
        joints = asset_utils._collect_joints(stage)

        self.assertIn("shoulder_joint", joints)
        self.assertIn("slider_joint", joints)
        _, is_rev, inst = joints["shoulder_joint"]
        self.assertTrue(is_rev)
        self.assertEqual(inst, "angular")
        _, is_rev2, inst2 = joints["slider_joint"]
        self.assertFalse(is_rev2)
        self.assertEqual(inst2, "linear")

    async def test_collect_joints_ignores_fixed(self) -> None:
        """Fixed joints should not appear in the collected set."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/child")
        _add_fixed_joint(stage, "/robot/fj", "/robot/base", "/robot/child")

        joints = asset_utils._collect_joints(stage)
        self.assertEqual(len(joints), 0)

    async def test_collect_joints_empty_stage(self) -> None:
        """An empty stage should produce an empty joints dict."""
        stage = _make_stage()
        self.assertEqual(asset_utils._collect_joints(stage), {})

    # -- apply_link_density --------------------------------------------------

    async def test_density_set_on_massless_links(self) -> None:
        """Density should be set on links with MassAPI but no explicit mass."""
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/link")
        UsdPhysics.MassAPI.Apply(link)

        asset_utils.apply_link_density(stage, 1000.0)

        density_attr = UsdPhysics.MassAPI(link).GetDensityAttr()
        self.assertAlmostEqual(density_attr.Get(), 1000.0)

    async def test_density_skips_links_with_mass(self) -> None:
        """Links that already have a positive mass should not get density overwritten."""
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/link")
        mass_api = UsdPhysics.MassAPI.Apply(link)
        mass_api.CreateMassAttr().Set(5.0)

        asset_utils.apply_link_density(stage, 1000.0)

        density_attr = mass_api.GetDensityAttr()
        has_value = density_attr and density_attr.HasValue()
        if has_value:
            self.assertNotAlmostEqual(density_attr.Get(), 1000.0)

    async def test_density_applies_mass_api_when_missing(self) -> None:
        """Rigid body links without MassAPI should have it applied and the
        density set in a single pass.
        """
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/link")
        # No explicit MassAPI applied here.
        self.assertFalse(link.HasAPI(UsdPhysics.MassAPI))

        asset_utils.apply_link_density(stage, 500.0)

        self.assertTrue(link.HasAPI(UsdPhysics.MassAPI))
        density_attr = UsdPhysics.MassAPI(link).GetDensityAttr()
        self.assertTrue(density_attr.IsValid())
        self.assertAlmostEqual(density_attr.Get(), 500.0)

    async def test_density_skips_non_rigid_body_prims(self) -> None:
        """Prims without RigidBodyAPI should never get MassAPI added."""
        stage = _make_stage()
        prim = stage.DefinePrim("/robot/decoration", "Xform")
        # No RigidBodyAPI -> apply_link_density must not touch it.
        asset_utils.apply_link_density(stage, 500.0)
        self.assertFalse(prim.HasAPI(UsdPhysics.MassAPI))

    async def test_density_multiple_links(self) -> None:
        """Only massless links should receive density when multiple links exist."""
        stage = _make_stage()
        link_a = _add_rigid_link(stage, "/robot/a")
        link_b = _add_rigid_link(stage, "/robot/b")
        UsdPhysics.MassAPI.Apply(link_a)
        mass_api_b = UsdPhysics.MassAPI.Apply(link_b)
        mass_api_b.CreateMassAttr().Set(2.0)

        asset_utils.apply_link_density(stage, 800.0)

        self.assertAlmostEqual(UsdPhysics.MassAPI(link_a).GetDensityAttr().Get(), 800.0)
        density_b = UsdPhysics.MassAPI(link_b).GetDensityAttr()
        if density_b and density_b.HasValue():
            self.assertNotAlmostEqual(density_b.Get(), 800.0)

    # -- _detect_fixed_base --------------------------------------------------

    async def test_detect_fixed_base_no_joints(self) -> None:
        """No joints should mean not fixed-base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        self.assertFalse(asset_utils._detect_fixed_base(stage, root, []))

    async def test_detect_fixed_base_world_anchor(self) -> None:
        """A FixedJoint with body0 unset (world) should be detected as fixed base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        fj_prim = _add_fixed_joint(stage, "/robot/fj", None, "/robot/base")
        self.assertTrue(asset_utils._detect_fixed_base(stage, root, [fj_prim]))

    async def test_detect_fixed_base_non_rigid_body(self) -> None:
        """A FixedJoint to a non-rigid-body Xform should be detected as fixed base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        fj_prim = _add_fixed_joint(stage, "/robot/fj", "/robot", "/robot/base")
        self.assertTrue(asset_utils._detect_fixed_base(stage, root, [fj_prim]))

    async def test_detect_fixed_base_two_rigid_bodies(self) -> None:
        """A FixedJoint between two rigid bodies is NOT a fixed base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/other")
        fj_prim = _add_fixed_joint(stage, "/robot/fj", "/robot/other", "/robot/base")
        self.assertFalse(asset_utils._detect_fixed_base(stage, root, [fj_prim]))

    async def test_detect_fixed_base_revolute_only(self) -> None:
        """A revolute joint should not count as a fixed base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/link1")
        rj = _add_revolute_joint(stage, "/robot/rj", "/robot/base", "/robot/link1")
        self.assertFalse(asset_utils._detect_fixed_base(stage, root, [rj]))

    async def test_detect_fixed_base_body0_is_root(self) -> None:
        """Root link on body0 side with no body1 should be detected as fixed base."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        fj = UsdPhysics.FixedJoint.Define(stage, "/robot/fj")
        fj.CreateBody0Rel().SetTargets([root.GetPath()])
        self.assertTrue(asset_utils._detect_fixed_base(stage, root, [fj.GetPrim()]))

    # -- apply_fix_base ------------------------------------------------------

    async def test_fix_base_creates_joint(self) -> None:
        """A FixedJoint should be created when no existing fixed base is found."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")

        asset_utils.apply_fix_base(stage)

        fix_joint_prim = stage.GetPrimAtPath("/robot/fix_base_joint")
        self.assertTrue(fix_joint_prim.IsValid())
        self.assertTrue(fix_joint_prim.IsA(UsdPhysics.FixedJoint))

    async def test_fix_base_skips_when_already_fixed(self) -> None:
        """No new joint should be created when the robot is already fixed."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_fixed_joint(stage, "/robot/world_fj", None, "/robot/base")

        asset_utils.apply_fix_base(stage)

        fix_joint_prim = stage.GetPrimAtPath("/robot/fix_base_joint")
        self.assertFalse(fix_joint_prim.IsValid())

    async def test_fix_base_no_default_prim(self) -> None:
        """apply_fix_base should not crash when no default prim exists."""
        stage = Usd.Stage.CreateInMemory()
        asset_utils.apply_fix_base(stage)

    async def test_fix_base_no_rigid_body(self) -> None:
        """apply_fix_base should not crash when no rigid body exists."""
        stage = _make_stage()
        asset_utils.apply_fix_base(stage)

    # -- apply_floating_base -------------------------------------------------

    async def test_floating_base_removes_world_anchor(self) -> None:
        """A FixedJoint with body0 unset (world) anchored to root must be removed."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_fixed_joint(stage, "/robot/world_fj", None, "/robot/base")

        asset_utils.apply_floating_base(stage)

        self.assertFalse(stage.GetPrimAtPath("/robot/world_fj").IsValid())

    async def test_floating_base_removes_anchor_to_non_rigid(self) -> None:
        """A FixedJoint anchoring root to a non-rigid Xform must be removed."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_fixed_joint(stage, "/robot/scene_fj", "/robot", "/robot/base")

        asset_utils.apply_floating_base(stage)

        self.assertFalse(stage.GetPrimAtPath("/robot/scene_fj").IsValid())

    async def test_floating_base_removes_anchor_with_root_on_body0(self) -> None:
        """Root on body0 side with no body1 must also be removed."""
        stage = _make_stage()
        root = _add_rigid_link(stage, "/robot/base")
        fj = UsdPhysics.FixedJoint.Define(stage, "/robot/world_fj")
        fj.CreateBody0Rel().SetTargets([root.GetPath()])

        asset_utils.apply_floating_base(stage)

        self.assertFalse(stage.GetPrimAtPath("/robot/world_fj").IsValid())

    async def test_floating_base_preserves_internal_fixed_joints(self) -> None:
        """A FixedJoint between two rigid bodies must NOT be removed."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/sensor_mount")
        _add_fixed_joint(stage, "/robot/mount_fj", "/robot/base", "/robot/sensor_mount")

        asset_utils.apply_floating_base(stage)

        self.assertTrue(stage.GetPrimAtPath("/robot/mount_fj").IsValid())

    async def test_floating_base_preserves_other_joint_types(self) -> None:
        """Non-FixedJoint joints must be untouched even when anchored to root."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/link1")
        _add_revolute_joint(stage, "/robot/rj", "/robot/base", "/robot/link1")

        asset_utils.apply_floating_base(stage)

        self.assertTrue(stage.GetPrimAtPath("/robot/rj").IsValid())

    async def test_floating_base_no_rigid_body(self) -> None:
        """apply_floating_base should not crash when no rigid body exists."""
        stage = _make_stage()
        asset_utils.apply_floating_base(stage)

    async def test_floating_base_no_anchor_present(self) -> None:
        """When there's no world-anchoring fixed joint, the stage stays untouched."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/link1")
        _add_revolute_joint(stage, "/robot/rj", "/robot/base", "/robot/link1")

        before = {p.GetPath().pathString for p in stage.Traverse()}
        asset_utils.apply_floating_base(stage)
        after = {p.GetPath().pathString for p in stage.Traverse()}
        self.assertEqual(before, after)

    async def test_fix_base_targets_articulation_root(self) -> None:
        """The created FixedJoint should target the articulation root link."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")
        _add_rigid_link(stage, "/robot/link1")
        _add_revolute_joint(stage, "/robot/rj", "/robot/base", "/robot/link1")

        asset_utils.apply_fix_base(stage)

        fj = UsdPhysics.FixedJoint(stage.GetPrimAtPath("/robot/fix_base_joint"))
        targets = fj.GetBody1Rel().GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(str(targets[0]), "/robot/base")

    async def test_fix_base_picks_root_not_first_traversed(self) -> None:
        """A leaf link earlier in traversal order must NOT be chosen as root."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/a_leaf")
        _add_rigid_link(stage, "/robot/base")
        _add_revolute_joint(stage, "/robot/rj", "/robot/base", "/robot/a_leaf")

        asset_utils.apply_fix_base(stage)

        fj = UsdPhysics.FixedJoint(stage.GetPrimAtPath("/robot/fix_base_joint"))
        targets = fj.GetBody1Rel().GetTargets()
        self.assertEqual(str(targets[0]), "/robot/base")

    async def test_fix_base_relocates_articulation_root(self) -> None:
        """apply_fix_base should also relocate ArticulationRootAPI off the rigid body."""
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/base")
        UsdPhysics.ArticulationRootAPI.Apply(link)

        asset_utils.apply_fix_base(stage)

        parent = stage.GetPrimAtPath("/robot")
        self.assertTrue(parent.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link.HasAPI(UsdPhysics.ArticulationRootAPI))

    # -- fix_articulation_root_for_fixed_base --------------------------------

    async def test_art_root_moved_to_parent(self) -> None:
        """ArticulationRootAPI should be relocated from the rigid body to its parent."""
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/base")
        UsdPhysics.ArticulationRootAPI.Apply(link)

        count = asset_utils.fix_articulation_root_for_fixed_base(stage)

        self.assertEqual(count, 1)
        parent = stage.GetPrimAtPath("/robot")
        self.assertTrue(parent.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link.HasAPI(UsdPhysics.ArticulationRootAPI))

    async def test_art_root_returns_zero_when_none(self) -> None:
        """Return 0 when no articulation root exists."""
        stage = _make_stage()
        _add_rigid_link(stage, "/robot/base")

        count = asset_utils.fix_articulation_root_for_fixed_base(stage)
        self.assertEqual(count, 0)

    async def test_art_root_not_on_rigid_body_ignored(self) -> None:
        """ArticulationRootAPI on a non-rigid-body prim should not be relocated."""
        stage = _make_stage()
        parent = stage.GetPrimAtPath("/robot")
        UsdPhysics.ArticulationRootAPI.Apply(parent)

        count = asset_utils.fix_articulation_root_for_fixed_base(stage)
        self.assertEqual(count, 0)
        self.assertTrue(parent.HasAPI(UsdPhysics.ArticulationRootAPI))

    async def test_art_root_parent_already_has_api(self) -> None:
        """Duplicate on rigid body should be removed when parent already has the API."""
        stage = _make_stage()
        link = _add_rigid_link(stage, "/robot/base")
        UsdPhysics.ArticulationRootAPI.Apply(link)
        parent = stage.GetPrimAtPath("/robot")
        UsdPhysics.ArticulationRootAPI.Apply(parent)

        count = asset_utils.fix_articulation_root_for_fixed_base(stage)
        self.assertEqual(count, 1)
        self.assertTrue(parent.HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(link.HasAPI(UsdPhysics.ArticulationRootAPI))

    async def test_art_root_multiple(self) -> None:
        """Multiple articulation roots on rigid bodies should all be relocated."""
        stage = _make_stage()
        link_a = _add_rigid_link(stage, "/robot/a")
        link_b = _add_rigid_link(stage, "/robot/b")
        UsdPhysics.ArticulationRootAPI.Apply(link_a)
        UsdPhysics.ArticulationRootAPI.Apply(link_b)

        count = asset_utils.fix_articulation_root_for_fixed_base(stage)
        self.assertEqual(count, 2)
        parent = stage.GetPrimAtPath("/robot")
        self.assertTrue(parent.HasAPI(UsdPhysics.ArticulationRootAPI))

    # -- apply_joint_drives --------------------------------------------------

    async def test_drive_type_scalar(self) -> None:
        """Setting drive_type as a scalar should apply to all joints."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, drive_type="force")

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                self.assertEqual(drive.GetTypeAttr().Get(), "force")
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "linear")
                self.assertEqual(drive.GetTypeAttr().Get(), "force")

    async def test_drive_type_per_joint(self) -> None:
        """Setting drive_type as a dict should apply per-pattern values."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, drive_type={"shoulder": "acceleration", "slider": "force"})

        for prim in stage.Traverse():
            if prim.GetName() == "shoulder_joint":
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                self.assertEqual(drive.GetTypeAttr().Get(), "acceleration")
            elif prim.GetName() == "slider_joint":
                drive = UsdPhysics.DriveAPI.Get(prim, "linear")
                self.assertEqual(drive.GetTypeAttr().Get(), "force")

    async def test_stiffness_revolute_rad_to_deg(self) -> None:
        """Revolute stiffness (Nm/rad) should be converted to USD (Nm/deg)."""
        stage = _make_robot()
        stiffness_rad = 100.0
        asset_utils.apply_joint_drives(stage, stiffness=stiffness_rad)

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                expected = stiffness_rad * math.pi / 180.0
                self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), expected, places=6)

    async def test_stiffness_prismatic_no_conversion(self) -> None:
        """Prismatic stiffness should not undergo unit conversion."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, stiffness=200.0)

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.PrismaticJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "linear")
                self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), 200.0, places=6)

    async def test_damping_revolute_rad_to_deg(self) -> None:
        """Revolute damping (Nm*s/rad) should be converted to USD (Nm*s/deg)."""
        stage = _make_robot()
        damping_rad = 50.0
        asset_utils.apply_joint_drives(stage, damping=damping_rad)

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                expected = damping_rad * math.pi / 180.0
                self.assertAlmostEqual(drive.GetDampingAttr().Get(), expected, places=6)

    async def test_damping_prismatic_no_conversion(self) -> None:
        """Prismatic damping should not undergo unit conversion."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, damping=25.0)

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.PrismaticJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "linear")
                self.assertAlmostEqual(drive.GetDampingAttr().Get(), 25.0, places=6)

    async def test_target_type_none_zeros_gains(self) -> None:
        """target_type='none' should zero out both stiffness and damping."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, stiffness=100.0, damping=50.0)
        asset_utils.apply_joint_drives(stage, target_type="none")

        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), 0.0)
                self.assertAlmostEqual(drive.GetDampingAttr().Get(), 0.0)

    async def test_drives_no_joints_is_noop(self) -> None:
        """apply_joint_drives should not crash when the stage has no joints."""
        stage = _make_stage()
        asset_utils.apply_joint_drives(stage, drive_type="force", stiffness=100.0)

    async def test_drives_all_none_is_noop(self) -> None:
        """Calling apply_joint_drives with all-None params should not modify the stage."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage)

    async def test_stiffness_regex_pattern(self) -> None:
        """Per-pattern stiffness should apply correct values with unit conversion."""
        stage = _make_robot()
        asset_utils.apply_joint_drives(stage, stiffness={"shoulder": 300.0, "slider": 500.0})

        for prim in stage.Traverse():
            if prim.GetName() == "shoulder_joint":
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                expected = 300.0 * math.pi / 180.0
                self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), expected, places=6)
            elif prim.GetName() == "slider_joint":
                drive = UsdPhysics.DriveAPI.Get(prim, "linear")
                self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), 500.0, places=6)

    # -- apply_mjc_actuator_gains --------------------------------------------

    async def test_mjc_gains_set(self) -> None:
        """Gain and bias attributes should be written to MjcActuator prims."""
        stage = _make_stage()
        stage.DefinePrim("/robot/actuator_0", "MjcActuator")
        gain_prm = [100.0] + [0.0] * 9
        bias_prm = [0.0, -100.0, -10.0] + [0.0] * 7

        count = asset_utils.apply_mjc_actuator_gains(stage, "fixed", "affine", gain_prm, bias_prm)

        self.assertEqual(count, 1)
        prim = stage.GetPrimAtPath("/robot/actuator_0")
        self.assertEqual(prim.GetAttribute("mjc:gainType").Get(), "fixed")
        self.assertEqual(prim.GetAttribute("mjc:biasType").Get(), "affine")
        self.assertEqual(list(prim.GetAttribute("mjc:gainPrm").Get()), gain_prm)
        self.assertEqual(list(prim.GetAttribute("mjc:biasPrm").Get()), bias_prm)

    async def test_mjc_gains_multiple_actuators(self) -> None:
        """All MjcActuator prims should be updated."""
        stage = _make_stage()
        for i in range(3):
            stage.DefinePrim(f"/robot/actuator_{i}", "MjcActuator")

        count = asset_utils.apply_mjc_actuator_gains(stage, "fixed", "none", [50.0] + [0.0] * 9, [0.0] * 10)
        self.assertEqual(count, 3)

    async def test_mjc_gains_no_actuators(self) -> None:
        """Return 0 when no MjcActuator prims exist."""
        stage = _make_stage()
        count = asset_utils.apply_mjc_actuator_gains(stage, "fixed", "affine", [0.0] * 10, [0.0] * 10)
        self.assertEqual(count, 0)

    async def test_mjc_gains_overwrite(self) -> None:
        """Calling apply_mjc_actuator_gains again should overwrite previous values."""
        stage = _make_stage()
        stage.DefinePrim("/robot/actuator_0", "MjcActuator")
        asset_utils.apply_mjc_actuator_gains(stage, "fixed", "affine", [1.0] + [0.0] * 9, [0.0] * 10)

        new_gain = [999.0] + [0.0] * 9
        asset_utils.apply_mjc_actuator_gains(stage, "muscle", "none", new_gain, [0.0] * 10)

        prim = stage.GetPrimAtPath("/robot/actuator_0")
        self.assertEqual(prim.GetAttribute("mjc:gainType").Get(), "muscle")
        self.assertEqual(list(prim.GetAttribute("mjc:gainPrm").Get()), new_gain)

    async def test_mjc_gains_ignores_non_actuators(self) -> None:
        """Non-MjcActuator prims should not be modified."""
        stage = _make_stage()
        stage.DefinePrim("/robot/not_an_actuator", "Xform")
        stage.DefinePrim("/robot/actuator", "MjcActuator")

        count = asset_utils.apply_mjc_actuator_gains(stage, "fixed", "affine", [0.0] * 10, [0.0] * 10)
        self.assertEqual(count, 1)
