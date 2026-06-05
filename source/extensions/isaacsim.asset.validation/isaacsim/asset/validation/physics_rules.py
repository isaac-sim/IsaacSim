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

"""Validation rules for physics simulation configuration."""

from __future__ import annotations

from typing import Any

import carb
import usdrt
from omni.asset_validator.core import registerRule
from omni.physics.core import ContactEventType, get_physics_simulation_interface
from omni.physx.bindings._physx import SETTING_UPDATE_TO_USD
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

from .util import DedupBaseRuleChecker

# from omni.physx.scripts.physicsUtils import get_initial_collider_pairs # ideally, import Ales's code here, blocked atm


@registerRule("IsaacSim.PhysicsRules")
class RigidBodyHasMassAPI(DedupBaseRuleChecker):
    """Validates that rigid bodies have properly configured mass properties.

    This rule checks that prims with the RigidBodyAPI also have the MassAPI applied
    with properly defined mass, diagonal inertia, and principal axes attributes.
    """

    def check_rigid_body_prim(self, prim: Usd.Prim) -> None:
        """Check if a rigid body prim has proper mass configuration.

        Validation uses layered precondition guards: missing ``MassAPI`` triggers
        an early return so downstream ``.Get()`` calls cannot raise
        ``'NoneType' object is not subscriptable``; missing per-attribute authored
        values are collected up front and reported all at once; value-based checks
        (triangle inequality, principalAxes normalization) run only after every
        authorship precondition has passed.

        ``physics:principalAxes`` is special-cased: the engine treats an unauthored
        attribute as identity, so the validator only enforces normalization when the
        attribute has an authored value with non-trivial length.

        Args:
            prim: The rigid body prim to validate.
        """
        path = prim.GetPath()

        # MassAPI must be applied before any value-based check.
        if not prim.HasAPI(UsdPhysics.MassAPI):
            self._AddError(message=f"Rigid body {path} has rigid body api but no mass api", at=prim)
            return

        # MassAPI predeclares ``physics:mass`` / ``physics:diagonalInertia`` /
        # ``physics:principalAxes`` as schema attributes, so ``prim.HasAttribute(...)``
        # returns True even when the attribute has no authored opinion. ``Get()``
        # would then return ``None`` and raise ``'NoneType' object is not
        # subscriptable``. Check ``HasAuthoredValue()`` to catch the authorship gap.
        mass_attr = prim.GetAttribute("physics:mass")
        inertia_attr = prim.GetAttribute("physics:diagonalInertia")
        pa_attr = prim.GetAttribute("physics:principalAxes")

        missing_any = False
        if not (mass_attr and mass_attr.HasAuthoredValue()):
            self._AddError(message=f"Rigid body {path} has mass api but no mass attr", at=prim)
            missing_any = True
        if not (inertia_attr and inertia_attr.HasAuthoredValue()):
            self._AddError(message=f"Rigid body {path} has mass api but no diagonal inertia attr", at=prim)
            missing_any = True
        # principalAxes authored-vs-unauthored handling lives in the value-based
        # check below. Here we only flag the case where the schema attribute itself
        # has been removed (manual RemoveProperty on a MassAPI-applied prim).
        if not prim.HasAttribute("physics:principalAxes"):
            self._AddError(message=f"Rigid body {path} has mass api but no principal axes attr", at=prim)
            missing_any = True
        if missing_any:
            return

        # Value-based checks (only reached when every precondition passes).
        mass_value = mass_attr.Get()
        if mass_value == 0:
            self._AddInfo(message=f"Rigid body {path} has mass of 0", at=prim)

        diagonal_inertia = inertia_attr.Get()
        if diagonal_inertia == Gf.Vec3f(0, 0, 0):
            self._AddInfo(message=f"Rigid body {path} has diagonal inertia of [0, 0, 0]", at=prim)
        else:
            # check triangle inequality: I1 + I2 >= I3, I1 + I3 >= I2, I2 + I3 >= I1
            i1, i2, i3 = diagonal_inertia[0], diagonal_inertia[1], diagonal_inertia[2]
            if i1 + i2 < i3:
                self._AddError(
                    message=f"Rigid body {path} violates inertia triangle inequality: I1 + I2 ({i1 + i2}) < I3 ({i3})",
                    at=prim,
                )
            if i1 + i3 < i2:
                self._AddError(
                    message=f"Rigid body {path} violates inertia triangle inequality: I1 + I3 ({i1 + i3}) < I2 ({i2})",
                    at=prim,
                )
            if i2 + i3 < i1:
                self._AddError(
                    message=f"Rigid body {path} violates inertia triangle inequality: I2 + I3 ({i2 + i3}) < I1 ({i1})",
                    at=prim,
                )

        # principalAxes normalization: enforce ONLY when the value is authored AND
        # its length is meaningfully non-zero. Unauthored principalAxes means the
        # engine treats it as identity, so the validator must not flag the engine
        # default as "not normalized". The length > 1e-4 floor defends against
        # all-zero quaternions that would otherwise report a false normalization error.
        if pa_attr.HasAuthoredValue():
            length = pa_attr.Get().GetLength()
            if length > 1e-4 and abs(length - 1.0) > 1e-4:
                self._AddError(
                    message=f"Rigid body {path}'s principal axes is not normalized, but: {length}",
                    at=prim,
                )

    def CheckStage(self, stage: Usd.Stage) -> None:  # noqa: N802
        """Check all rigid bodies in the stage for proper mass configuration.

        Args:
            stage: The USD stage to validate.
        """
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.check_rigid_body_prim(prim)


@registerRule("IsaacSim.PhysicsRules")
class RigidBodyHasCollider(DedupBaseRuleChecker):
    """Validates that enabled rigid bodies have collision geometry.

    This rule checks that prims with an enabled RigidBodyAPI also have the CollisionAPI
    applied, which is required for collision detection in physics simulation.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if an enabled rigid body has collision geometry.

        Args:
            prim: The USD prim to validate.
        """
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        if not rigid_body_api:
            return
        else:
            # check if the rigid body api is enabled
            if not rigid_body_api.GetRigidBodyEnabledAttr().Get():
                return
            for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    return
            self._AddError(message=f"Rigid body {prim.GetPath()} has rigid body api but no collision api", at=prim)


def _find_rigid_body_ancestor(prim: Usd.Prim) -> Sdf.Path:
    """Walk up to the nearest ancestor (inclusive) carrying UsdPhysics.RigidBodyAPI.

    Returns ``Sdf.Path.emptyPath`` if no such ancestor exists. Used by
    :class:`NonAdjacentCollisionMeshesDoNotClash` to key adjacency-dict lookups
    on the rigid body that owns each collider, rather than the collider's
    direct parent — which fails whenever collision meshes live nested under a
    link (e.g. ``/Robot/link0/collisions/mesh_0``).

    Args:
        prim: The prim (typically a collider) to search upward from.

    Returns:
        Path of the nearest ancestor (or ``prim`` itself) with
        ``UsdPhysics.RigidBodyAPI`` applied, or ``Sdf.Path.emptyPath`` if none.
    """
    current = prim
    while current and current.IsValid() and not current.IsPseudoRoot():
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            return current.GetPath()
        current = current.GetParent()
    return Sdf.Path.emptyPath


def compute_adjacent_mesh_dict(stage: Usd.Stage) -> dict:
    """Compute a dictionary mapping body paths to lists of adjacent body paths.

    Args:
        stage: The USD stage to analyze.

    Returns:
        A dictionary mapping body paths to lists of adjacent body paths.
    """
    # Traverse through the joints, log every pair of connected bodies
    defaultPrim = stage.GetDefaultPrim()
    if not defaultPrim or not defaultPrim.IsValid():
        return {}

    adjacent_mesh_matrix = {}

    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxJointAPI):
            joint = UsdPhysics.Joint(prim)
            body0_targets = joint.GetBody0Rel().GetTargets()
            if not body0_targets:
                continue
            body0 = body0_targets[0]
            body1_targets = joint.GetBody1Rel().GetTargets()
            if not body1_targets:
                continue
            body1 = body1_targets[0]

            # body0 and body1 are adjacent, log into joint dict
            if body0 not in adjacent_mesh_matrix:
                adjacent_mesh_matrix[body0] = []
            if body1 not in adjacent_mesh_matrix:
                adjacent_mesh_matrix[body1] = []
            adjacent_mesh_matrix[body0].append(body1)
            adjacent_mesh_matrix[body1].append(body0)

    return adjacent_mesh_matrix


# Copied from Ales's code
def get_initial_collider_pairs(stage: Usd.Stage) -> set[tuple[str, str]]:
    """Get all collider pairs that are in contact in the physics simulation.

    This function performs a single physics simulation step and collects all collider pairs
    that are in contact. It temporarily modifies physics settings to ensure accurate contact
    detection and restores them after completion.

    The function:
    1. Creates a temporary session layer for contact reporting
    2. Enables contact reporting for all rigid bodies
    3. Runs a single physics simulation step
    4. Collects all collider pairs that are in contact
    5. Restores original physics settings

    Args:
        stage: The USD stage containing the physics scene to analyze.

    Returns:
        A set of tuples, where each tuple contains the paths of two colliders
            that are in contact. The paths in each tuple are sorted alphabetically
            to ensure consistent ordering regardless of which collider initiated
            the contact.

    Note:
        This function temporarily modifies physics settings and runs a simulation step.
        The original settings are restored after the function completes.
    """

    def on_contact_event(contact_headers: Any, contact_data: Any, friction_anchors: Any) -> None:
        for contact_header in contact_headers:
            if contact_header.type == ContactEventType.CONTACT_FOUND:
                collider0 = str(PhysicsSchemaTools.intToSdfPath(contact_header.collider0))
                collider1 = str(PhysicsSchemaTools.intToSdfPath(contact_header.collider1))
                # Store as a tuple, ensuring consistent ordering
                pair = tuple(sorted([collider0, collider1]))
                unique_collider_pairs.add(pair)

    unique_collider_pairs = set()  # Use a set to store unique collider pairs
    session_sub_layer = Sdf.Layer.CreateAnonymous()
    stage.GetSessionLayer().subLayerPaths.append(session_sub_layer.identifier)
    old_layer = stage.GetEditTarget().GetLayer()
    stage.SetEditTarget(Usd.EditTarget(session_sub_layer))

    # Added this to avoid stage not in cache error
    stageCache = UsdUtils.StageCache.Get()
    stageCache.Insert(stage)  # Register the stage

    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    usdrtStage = usdrt.Usd.Stage.Attach(stage_id)
    prim_paths = usdrtStage.GetPrimsWithAppliedAPIName("PhysicsRigidBodyAPI")

    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(str(prim_path))
        if prim:
            contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contact_report_api.CreateThresholdAttr().Set(0)

    settings = carb.settings.get_settings()
    write_usd = settings.get_as_bool(SETTING_UPDATE_TO_USD)
    write_fabric = settings.get_as_bool("/physics/fabricEnabled")

    settings.set(SETTING_UPDATE_TO_USD, False)
    settings.set("/physics/fabricEnabled", False)

    initial_attach = False
    if get_physics_simulation_interface().get_attached_stage() != stage_id:
        get_physics_simulation_interface().initialize(stage_id)
        initial_attach = True

    contact_report_sub = get_physics_simulation_interface().subscribe_physics_contact_report_events(on_contact_event)

    get_physics_simulation_interface().simulate(1.0 / 60.0, 0.0)

    if contact_report_sub:
        contact_report_sub = None

    if initial_attach:
        get_physics_simulation_interface().close()

    settings.set(SETTING_UPDATE_TO_USD, write_usd)
    settings.set("/physics/fabricEnabled", write_fabric)

    stage.SetEditTarget(old_layer)

    stage.GetSessionLayer().subLayerPaths.remove(session_sub_layer.identifier)
    session_sub_layer = None

    return unique_collider_pairs


@registerRule("IsaacSim.PhysicsRules")
class NonAdjacentCollisionMeshesDoNotClash(DedupBaseRuleChecker):
    """Validates that non-adjacent collision meshes don't intersect.

    This rule checks that collision meshes that aren't connected by joints don't
    intersect each other, which can cause unstable physics simulation.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:  # noqa: N802
        """Check for intersecting non-adjacent collision meshes.

        Populates ``self.adjacent_mesh_matrix`` and ``self.collisions_pairs``
        from the stage, then delegates to :meth:`_check_pairs`. The
        ``_check_pairs`` helper is public-to-the-class so tests can inject
        pre-computed pair/adjacency state without running a live PhysX step.

        Args:
            stage: The USD stage to validate.
        """
        self.adjacent_mesh_matrix = compute_adjacent_mesh_dict(stage)  # keyed on rigid-body paths
        self.collisions_pairs = get_initial_collider_pairs(stage)  # tuples of collider Sdf paths
        self._check_pairs(stage)

    def _check_pairs(self, stage: Usd.Stage) -> None:
        """Run the inner pair-filter loop.

        Factored out of :meth:`CheckStage` so tests can inject collision pairs
        without running a live PhysX simulation step.

        Two filters:

        - Pairs outside the ``stage.GetDefaultPrim()`` subtree are skipped.
          Ground planes and other environment scaffolding shipped alongside
          the robot in the source USD are not the robot's non-adjacency
          problem to report.
        - Adjacency is looked up on the rigid body that *owns* each collider
          (via :func:`_find_rigid_body_ancestor`), not on the collider's
          immediate parent. Collision meshes nested deeper than one level
          under a link (e.g. ``/link/collisions/mesh_N``) would otherwise
          never match the adjacency dict's rigid-body keys.

        The adjacency dict (:func:`compute_adjacent_mesh_dict`) already keys
        on rigid-body paths, so lookup keys align with dict keys here.

        Args:
            stage: The USD stage being validated. Used for path->prim lookups.
        """
        default_prim = stage.GetDefaultPrim()
        if not default_prim or not default_prim.IsValid():
            return
        default_prim_path = default_prim.GetPath()

        for collision_pair in self.collisions_pairs:
            body0_prim = stage.GetPrimAtPath(collision_pair[0])
            body1_prim = stage.GetPrimAtPath(collision_pair[1])
            if not body0_prim or not body1_prim:
                continue

            # Both prims must live under defaultPrim.
            if not (
                body0_prim.GetPath().HasPrefix(default_prim_path) and body1_prim.GetPath().HasPrefix(default_prim_path)
            ):
                continue

            # Walk to the rigid-body ancestor for adjacency lookup.
            body0_rb = _find_rigid_body_ancestor(body0_prim)
            body1_rb = _find_rigid_body_ancestor(body1_prim)
            if body0_rb.isEmpty or body1_rb.isEmpty:
                # Orphan collider without a rigid-body ancestor. Not this validator's job.
                continue

            if body1_rb in self.adjacent_mesh_matrix.get(body0_rb, []):
                continue

            self._AddError(
                message=(f"Colliding meshes {body0_prim.GetPath()} and " f"{body1_prim.GetPath()} are not adjacent"),
                at=body0_prim,
            )


@registerRule("IsaacSim.PhysicsRules")
class InvisibleCollisionMeshHasPurposeGuide(DedupBaseRuleChecker):
    """Validates that invisible collision meshes have purpose set to 'guide'.

    This rule checks that collision meshes with visibility set to 'invisible'
    have their purpose set to 'guide', following USD best practices.
    """

    def CheckPrim(self, prim: Usd.Prim) -> None:  # noqa: N802
        """Check if invisible collision meshes have proper purpose setting.

        Args:
            prim: The USD prim to validate.
        """
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            return
        prim_imageable = UsdGeom.Imageable(prim)
        prim_visibility = prim_imageable.ComputeVisibility()

        match prim_visibility:
            case UsdGeom.Tokens.inherited:
                return
            case UsdGeom.Tokens.invisible:
                prim_purpose = prim_imageable.ComputePurpose()
                if prim_purpose != UsdGeom.Tokens.guide:
                    self._AddWarning(
                        message=f"Invisible collision mesh {prim.GetPath()} purpose: [{prim_purpose}], not [guide]",
                        at=prim,
                    )
                return
            case _:
                return


@registerRule("IsaacSim.PhysicsRules")
class HasArticulationRoot(DedupBaseRuleChecker):
    """Validates that at least one prim in the stage has the ArticulationRootAPI.

    This rule checks that the USD stage contains at least one prim with the
    UsdPhysics.ArticulationRootAPI applied. The ArticulationRootAPI is required for
    proper articulation simulation in physics.
    """

    def CheckStage(self, stage: Usd.Stage) -> None:  # noqa: N802
        """Check if the stage has at least one articulation root.

        Skipped on rigid-body-only stages (vehicles, non-articulated props)
        which are not expected to carry ``ArticulationRootAPI``. Only fires
        when the stage has joints but no articulation root.

        Args:
            stage: The USD stage to validate.
        """
        if not any(prim.IsA(UsdPhysics.Joint) for prim in stage.Traverse()):
            return
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                return
        self._AddError(
            message="Articulation Root API is not set on any prim in the stage",
            at=stage,
        )
