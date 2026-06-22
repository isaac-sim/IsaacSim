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

"""Tests for physics validation rules.

Regression coverage for ``RigidBodyHasMassAPI``:
- unauthored ``physics:principalAxes`` must not be flagged as "not normalized".
- missing ``MassAPI`` / missing individual attrs must not crash the checker.

Regression coverage for ``NonAdjacentCollisionMeshesDoNotClash``:
- pairs outside the ``defaultPrim`` subtree must be skipped.
- adjacency lookup must walk to the nearest ``RigidBodyAPI`` ancestor.

Regression coverage for ``HasArticulationRoot``:
- rigid-body-only stages with no joints must not be flagged for missing
  ``ArticulationRootAPI``.
"""

from __future__ import annotations

import omni.kit.test
from isaacsim.asset.validation.physics_rules import (
    HasArticulationRoot,
    NonAdjacentCollisionMeshesDoNotClash,
    RigidBodyHasMassAPI,
    _find_rigid_body_ancestor,
)
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics


class _ErrorCapturingRigidBodyChecker(RigidBodyHasMassAPI):
    """Subclass that captures ``_AddError`` / ``_AddInfo`` / ``_AddWarning`` emissions."""

    def __init__(self) -> None:
        super().__init__()
        self.errors: list[str] = []
        self.infos: list[str] = []
        self.warnings: list[str] = []

    # BaseRuleChecker emission hooks — accept arbitrary kwargs for compatibility.
    def _AddError(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.errors.append(message)

    def _AddInfo(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.infos.append(message)

    def _AddWarning(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.warnings.append(message)


def _make_rigid_body_prim(stage: Usd.Stage, path: str = "/Robot/link") -> Usd.Prim:
    """Define an Xform prim at ``path`` with ``RigidBodyAPI`` applied.

    Args:
        stage: Stage where the prim is defined.
        path: Path where the rigid body prim is defined.

    Returns:
        The prim with ``RigidBodyAPI`` applied.
    """
    # Ensure parent /Robot exists as a defined prim so the path resolves.
    stage.DefinePrim("/Robot", "Xform")
    prim = stage.DefinePrim(path, "Xform")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    return prim


def _apply_mass_api_with(
    prim: Usd.Prim,
    *,
    mass: float | None = 0.1,
    diagonal_inertia: Gf.Vec3f | None = Gf.Vec3f(0.01, 0.01, 0.01),
    principal_axes: Gf.Quatf | None = None,
) -> None:
    """Apply ``MassAPI`` to ``prim`` and optionally author each attribute.

    Pass ``None`` for any field to skip authoring it. Useful for precondition tests.

    Args:
        prim: Prim to receive ``MassAPI``.
        mass: Mass value to author, or None to skip authoring it.
        diagonal_inertia: Diagonal inertia value to author, or None to skip authoring it.
        principal_axes: Principal axes value to author, or None to skip authoring it.
    """
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    if mass is not None:
        mass_api.CreateMassAttr(mass)
    if diagonal_inertia is not None:
        mass_api.CreateDiagonalInertiaAttr(diagonal_inertia)
    if principal_axes is not None:
        mass_api.CreatePrincipalAxesAttr(principal_axes)


class TestRigidBodyHasMassAPI(omni.kit.test.AsyncTestCase):
    """Regression tests for ``RigidBodyHasMassAPI`` authorship and principalAxes handling."""

    async def test_rigid_body_unauthored_principal_axes(self) -> None:
        """Unauthored principalAxes must NOT trigger the 'not normalized' error."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, principal_axes=None)  # NO authored principalAxes
        checker = _ErrorCapturingRigidBodyChecker()
        checker.check_rigid_body_prim(prim)
        self.assertEqual(checker.errors, [], f"Expected zero errors, got {checker.errors}")
        self.assertEqual(checker.infos, [], f"Expected zero infos, got {checker.infos}")

    async def test_rigid_body_identity_principal_axes(self) -> None:
        """Normalized (identity) principalAxes is accepted."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, principal_axes=Gf.Quatf(1, 0, 0, 0))
        checker = _ErrorCapturingRigidBodyChecker()
        checker.check_rigid_body_prim(prim)
        self.assertEqual(checker.errors, [])

    async def test_rigid_body_rotated_normalized_principal_axes(self) -> None:
        """Rotated-but-normalized principalAxes (length=1.0) is accepted."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, principal_axes=Gf.Quatf(0.5, 0.5, 0.5, 0.5))
        checker = _ErrorCapturingRigidBodyChecker()
        checker.check_rigid_body_prim(prim)
        self.assertEqual(checker.errors, [])

    async def test_rigid_body_unnormalized_principal_axes(self) -> None:
        """Authored-but-unnormalized principalAxes MUST still be flagged (regression)."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, principal_axes=Gf.Quatf(2, 0, 0, 0))
        checker = _ErrorCapturingRigidBodyChecker()
        checker.check_rigid_body_prim(prim)
        self.assertEqual(len(checker.errors), 1, f"Expected 1 error, got {checker.errors}")
        self.assertIn("principal axes is not normalized", checker.errors[0])

    async def test_rigid_body_missing_mass_api_no_crash(self) -> None:
        """RigidBodyAPI without MassAPI — 1 error, no crash."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)  # NO MassAPI applied
        checker = _ErrorCapturingRigidBodyChecker()
        try:
            checker.check_rigid_body_prim(prim)
        except Exception as exc:  # noqa: BLE001
            self.fail(f"check_rigid_body_prim raised {type(exc).__name__}: {exc}")
        self.assertEqual(len(checker.errors), 1)
        self.assertIn("no mass api", checker.errors[0])

    async def test_rigid_body_missing_mass_attr_no_crash(self) -> None:
        """MassAPI applied but physics:mass not authored — graceful, no crash."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, mass=None)  # MassAPI applied, physics:mass NOT authored
        checker = _ErrorCapturingRigidBodyChecker()
        try:
            checker.check_rigid_body_prim(prim)
        except Exception as exc:  # noqa: BLE001
            self.fail(f"check_rigid_body_prim raised {type(exc).__name__}: {exc}")
        error_messages = " | ".join(checker.errors)
        self.assertIn("no mass attr", error_messages)
        # Value-based checks must NOT fire when a precondition is missing.
        self.assertNotIn("principal axes is not normalized", error_messages)

    async def test_rigid_body_missing_diagonal_inertia_no_crash(self) -> None:
        """Missing physics:diagonalInertia — graceful, no crash."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, diagonal_inertia=None)
        checker = _ErrorCapturingRigidBodyChecker()
        try:
            checker.check_rigid_body_prim(prim)
        except Exception as exc:  # noqa: BLE001
            self.fail(f"check_rigid_body_prim raised {type(exc).__name__}: {exc}")
        self.assertIn("no diagonal inertia attr", " | ".join(checker.errors))

    async def test_rigid_body_missing_principal_axes_attr_no_crash(self) -> None:
        """Missing physics:principalAxes attribute entirely — graceful, no crash."""
        stage = Usd.Stage.CreateInMemory()
        prim = _make_rigid_body_prim(stage)
        _apply_mass_api_with(prim, principal_axes=None)
        # Strip principalAxes attribute entirely if MassAPI auto-created it.
        if prim.HasAttribute("physics:principalAxes"):
            prim.RemoveProperty("physics:principalAxes")
        checker = _ErrorCapturingRigidBodyChecker()
        try:
            checker.check_rigid_body_prim(prim)
        except Exception as exc:  # noqa: BLE001
            self.fail(f"check_rigid_body_prim raised {type(exc).__name__}: {exc}")
        # Only reached when the attribute was actually removed (skip if MassAPI
        # does not auto-create it, which is the common case).
        if not prim.HasAttribute("physics:principalAxes"):
            self.assertIn("no principal axes attr", " | ".join(checker.errors))


# ---------------------------------------------------------------------------
# NonAdjacentCollisionMeshesDoNotClash tests
# ---------------------------------------------------------------------------


class _NonAdjacentChecker(NonAdjacentCollisionMeshesDoNotClash):
    """Subclass that captures ``_AddError`` emissions and lets tests inject pair state.

    Bypasses ``CheckStage`` entirely — tests exercise the pair-filtering loop by
    populating ``collisions_pairs`` and ``adjacent_mesh_matrix`` directly and
    invoking the inner loop (factored into ``_check_pairs``).
    """

    def __init__(self) -> None:
        super().__init__()
        self.errors: list[str] = []

    def _AddError(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.errors.append(message)


class TestNonAdjacentCollisionMeshesDoNotClash(omni.kit.test.AsyncTestCase):
    """Regression tests for the pair-filter loop.

    Covers defaultPrim scoping and rigid-body-ancestor adjacency lookup.
    """

    def _build_robot_with_ground_plane(self) -> Usd.Stage:
        """Build a stage whose defaultPrim is /Robot, with a sibling /GroundPlane.

        /Robot/link0 and /Robot/link1 both carry RigidBodyAPI and are connected
        by a joint under /Robot. /GroundPlane carries RigidBodyAPI and is OUTSIDE
        the defaultPrim subtree.

        Returns:
            The stage containing the robot and sibling ground plane.
        """
        stage = Usd.Stage.CreateInMemory()
        robot = stage.DefinePrim("/Robot", "Xform")
        stage.SetDefaultPrim(robot)
        link0 = stage.DefinePrim("/Robot/link0", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link0)
        UsdPhysics.CollisionAPI.Apply(link0)
        link1 = stage.DefinePrim("/Robot/link1", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link1)
        UsdPhysics.CollisionAPI.Apply(link1)
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        ground = stage.DefinePrim("/GroundPlane", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(ground)
        UsdPhysics.CollisionAPI.Apply(ground)
        return stage

    def _build_robot_with_nested_colliders(self) -> Usd.Stage:
        """Build a stage where collider meshes are NESTED under the rigid-body links.

        /Robot/link0 and /Robot/link1 each have RigidBodyAPI. Collider meshes live at
        /Robot/link0/collisions/mesh_0 and /Robot/link1/collisions/mesh_1 (depth ≥ 2).
        A joint connects link0 and link1.

        Returns:
            The stage containing nested colliders under joint-connected links.
        """
        stage = Usd.Stage.CreateInMemory()
        robot = stage.DefinePrim("/Robot", "Xform")
        stage.SetDefaultPrim(robot)
        link0 = stage.DefinePrim("/Robot/link0", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link0)
        mesh0 = stage.DefinePrim("/Robot/link0/collisions/mesh_0", "Mesh")
        UsdPhysics.CollisionAPI.Apply(mesh0)
        link1 = stage.DefinePrim("/Robot/link1", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link1)
        mesh1 = stage.DefinePrim("/Robot/link1/collisions/mesh_1", "Mesh")
        UsdPhysics.CollisionAPI.Apply(mesh1)
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        return stage

    async def test_skips_ground_plane_outside_default_prim(self) -> None:
        """A collider outside defaultPrim must not be flagged as 'not adjacent'."""
        stage = self._build_robot_with_ground_plane()
        checker = _NonAdjacentChecker()
        checker.adjacent_mesh_matrix = {}
        # Ground plane collider vs. robot link collider — would be flagged before the fix.
        checker.collisions_pairs = {("/GroundPlane", "/Robot/link0")}
        checker._check_pairs(stage)  # type: ignore[attr-defined]
        self.assertEqual(checker.errors, [], f"Expected zero errors, got {checker.errors}")

    async def test_nested_colliders_treated_as_adjacent(self) -> None:
        """Collider pairs nested under joint-connected rigid bodies are adjacent."""
        stage = self._build_robot_with_nested_colliders()
        checker = _NonAdjacentChecker()
        # Match the dict-build pattern in compute_adjacent_mesh_dict — keys on rigid-body paths.
        checker.adjacent_mesh_matrix = {
            Sdf.Path("/Robot/link0"): [Sdf.Path("/Robot/link1")],
            Sdf.Path("/Robot/link1"): [Sdf.Path("/Robot/link0")],
        }
        checker.collisions_pairs = {
            ("/Robot/link0/collisions/mesh_0", "/Robot/link1/collisions/mesh_1"),
        }
        checker._check_pairs(stage)  # type: ignore[attr-defined]
        self.assertEqual(
            checker.errors,
            [],
            f"Expected nested colliders to resolve to joint-connected rigid bodies; got {checker.errors}",
        )

    async def test_negative_nested_colliders_not_adjacent(self) -> None:
        """Nested colliders under rigid bodies NOT connected by a joint are flagged."""
        stage = self._build_robot_with_nested_colliders()
        checker = _NonAdjacentChecker()
        # Empty adjacency dict — no joint known for this test.
        checker.adjacent_mesh_matrix = {}
        checker.collisions_pairs = {
            ("/Robot/link0/collisions/mesh_0", "/Robot/link1/collisions/mesh_1"),
        }
        checker._check_pairs(stage)  # type: ignore[attr-defined]
        self.assertEqual(len(checker.errors), 1)
        self.assertIn("not adjacent", checker.errors[0])


class TestFindRigidBodyAncestor(omni.kit.test.AsyncTestCase):
    """Unit tests for the ``_find_rigid_body_ancestor`` helper."""

    async def test_direct_rigid_body(self) -> None:
        """A direct rigid body prim resolves to its own path."""
        stage = Usd.Stage.CreateInMemory()
        link = stage.DefinePrim("/Robot/link", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link)
        result = _find_rigid_body_ancestor(link)
        self.assertEqual(result, Sdf.Path("/Robot/link"))

    async def test_nested_collider_finds_ancestor(self) -> None:
        """A nested collider resolves to its rigid body ancestor."""
        stage = Usd.Stage.CreateInMemory()
        link = stage.DefinePrim("/Robot/link", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(link)
        mesh = stage.DefinePrim("/Robot/link/collisions/mesh", "Mesh")
        result = _find_rigid_body_ancestor(mesh)
        self.assertEqual(result, Sdf.Path("/Robot/link"))

    async def test_no_rigid_body_returns_empty(self) -> None:
        """A prim without a rigid body ancestor resolves to an empty path."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/GroundPlane", "Xform")
        # No RigidBodyAPI applied anywhere up the chain.
        result = _find_rigid_body_ancestor(prim)
        self.assertTrue(result.isEmpty, f"Expected empty path, got {result}")


# ---------------------------------------------------------------------------
# HasArticulationRoot joint-presence gate tests
# ---------------------------------------------------------------------------


class _HasArticulationRootChecker(HasArticulationRoot):
    """Subclass that captures ``_AddError`` emissions."""

    def __init__(self) -> None:
        super().__init__()
        self.errors: list[str] = []

    def _AddError(self, message: str, **kwargs: object) -> None:  # type: ignore[override]  # noqa: N802
        self.errors.append(message)


class TestHasArticulationRoot(omni.kit.test.AsyncTestCase):
    """Regression tests for the joint-presence gate."""

    async def test_rigid_body_only_stage_is_not_flagged(self) -> None:
        """A stage with rigid bodies but no joints is NOT flagged."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Vehicle/chassis", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        checker = _HasArticulationRootChecker()
        checker.CheckStage(stage)
        self.assertEqual(checker.errors, [], f"Expected zero errors, got {checker.errors}")

    async def test_joints_present_without_articulation_root_flagged(self) -> None:
        """Regression: joints present but no ArticulationRootAPI — original catch preserved."""
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/Robot/link0", "Xform")
        stage.DefinePrim("/Robot/link1", "Xform")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        checker = _HasArticulationRootChecker()
        checker.CheckStage(stage)
        self.assertEqual(len(checker.errors), 1)
        self.assertIn("Articulation Root API is not set", checker.errors[0])

    async def test_joints_present_with_articulation_root_not_flagged(self) -> None:
        """Regression: joints + ArticulationRootAPI — no error."""
        stage = Usd.Stage.CreateInMemory()
        robot = stage.DefinePrim("/Robot", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        stage.DefinePrim("/Robot/link0", "Xform")
        stage.DefinePrim("/Robot/link1", "Xform")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joints/j0")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/link0")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/link1")])
        checker = _HasArticulationRootChecker()
        checker.CheckStage(stage)
        self.assertEqual(checker.errors, [])
