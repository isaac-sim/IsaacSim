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

"""Convert an assembled PhysX USD asset to MuJoCo/Newton representations.

Entry point is :func:`convert_physx_asset_to_mjc`; lower-level passes are exposed
for standalone use. Revolute/prismatic joints get ``MjcJointAPI`` + ``MjcActuator``,
mimic joints become ``NewtonMimicAPI``, and articulation roots gain
``NewtonArticulationRootAPI``. Multi-DOF joints (generic D6 and ``SphericalJoint``)
are left untouched and warned about, since expanding them would change the
articulation structure. Pass a ``delta_stage`` (see :func:`make_delta_stage`) to
keep all authored opinions in a separate overlay layer.
"""

from __future__ import annotations

import logging

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .physx_types import PhysxAttr, PhysxMimicAttr, PhysxMimicRel, PhysxSchema
from .urdf_to_mjc_physx_conversion_utils import convert_physx_to_mjc, create_mjc_actuator_from_physics

_logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _is_d6_joint(prim: Usd.Prim) -> bool:
    """Return ``True`` if *prim* is a generic D6 (``UsdPhysics.Joint``) and not a typed subclass.

    Args:
        prim: USD prim to process.

    Returns:
        The resulting value.
    """
    return (
        prim.IsA(UsdPhysics.Joint)
        and not prim.IsA(UsdPhysics.RevoluteJoint)
        and not prim.IsA(UsdPhysics.PrismaticJoint)
        and not prim.IsA(UsdPhysics.SphericalJoint)
        and not prim.IsA(UsdPhysics.DistanceJoint)
        and not prim.IsA(UsdPhysics.FixedJoint)
    )


# ---------------------------------------------------------------------------
# Public conversion passes
# ---------------------------------------------------------------------------


def warn_unconverted_multi_dof_joints(stage: Usd.Stage) -> int:
    """Warn about D6/spherical joints that are skipped (converting them would alter the articulation structure).

    Args:
        stage: USD stage to inspect.

    Returns:
        Number of multi-DOF joints found and skipped.
    """
    skipped = 0
    for prim in stage.Traverse():
        if _is_d6_joint(prim):
            kind = "D6"
        elif prim.IsA(UsdPhysics.SphericalJoint):
            kind = "spherical"
        else:
            continue
        _logger.warning(
            f"{kind} joint {prim.GetPath()} is not converted to MuJoCo/Newton; "
            "expanding it would change the articulation structure"
        )
        skipped += 1
    return skipped


def convert_physx_mimic_to_newton(stage: Usd.Stage) -> int:
    """Convert ``PhysxMimicJointAPI`` to ``NewtonMimicAPI`` (referenceJoint/gearing/offset → mimicJoint/coef0/coef1).

    The PhysX attrs are left in place. Only the first axis instance per prim is
    mapped (a warning is emitted for extras).

    Args:
        stage: USD stage to update (authoring goes to the current edit target).

    Returns:
        Number of prims converted.
    """
    converted = 0
    for prim in stage.Traverse():
        applied = prim.GetAppliedSchemas()
        mimic_prefix = f"{PhysxSchema.MIMIC_JOINT_API.value}:"
        mimic_instances = [s.split(":")[1] for s in applied if s.startswith(mimic_prefix)]
        if not mimic_instances:
            continue

        if not prim.HasAPI("NewtonMimicAPI"):
            prim.ApplyAPI("NewtonMimicAPI")

        if len(mimic_instances) > 1:
            _logger.warning(
                f"{prim.GetPath()} has {len(mimic_instances)} PhysxMimicJointAPI instances "
                f"{mimic_instances}; only '{mimic_instances[0]}' will be mapped to NewtonMimicAPI"
            )

        axis = mimic_instances[0]

        ref_rel = prim.GetRelationship(PhysxMimicRel.REFERENCE_JOINT.format(axis))
        if ref_rel and ref_rel.IsValid():
            targets = ref_rel.GetTargets()
            if targets:
                prim.CreateRelationship("newton:mimicJoint").SetTargets(list(targets))

        gearing_attr = prim.GetAttribute(PhysxMimicAttr.GEARING.format(axis))
        if gearing_attr and gearing_attr.IsValid() and gearing_attr.HasAuthoredValue():
            prim.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(gearing_attr.Get())

        offset_attr = prim.GetAttribute(PhysxMimicAttr.OFFSET.format(axis))
        if offset_attr and offset_attr.IsValid() and offset_attr.HasAuthoredValue():
            prim.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(offset_attr.Get())

        converted += 1
        _logger.info(f"Converted PhysxMimicJointAPI on {prim.GetPath()} → NewtonMimicAPI")

    return converted


def convert_physx_articulation_to_newton(stage: Usd.Stage) -> int:
    """Add ``NewtonArticulationRootAPI`` to prims that carry ``PhysxArticulationAPI``.

    Maps ``physxArticulation:enabledSelfCollisions`` →
    ``newton:selfCollisionEnabled``.  The PhysX API is left in place.

    Args:
        stage: USD stage to update (authoring goes to the current edit target).

    Returns:
        Number of articulation root prims updated.
    """
    updated = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(PhysxSchema.ARTICULATION_API):
            continue

        if not prim.HasAPI("NewtonArticulationRootAPI"):
            prim.ApplyAPI("NewtonArticulationRootAPI")

        self_coll_attr = prim.GetAttribute(PhysxAttr.ARTICULATION_SELF_COLLISION.name)
        if self_coll_attr and self_coll_attr.IsValid() and self_coll_attr.HasAuthoredValue():
            newton_attr = prim.GetAttribute("newton:selfCollisionEnabled")
            if not newton_attr or not newton_attr.IsValid():
                newton_attr = prim.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool)
            newton_attr.Set(self_coll_attr.Get())

        updated += 1
        _logger.info(f"Added NewtonArticulationRootAPI to {prim.GetPath()}")

    return updated


# ---------------------------------------------------------------------------
# Stage factory helper
# ---------------------------------------------------------------------------


def make_delta_stage(source_stage: Usd.Stage) -> Usd.Stage:
    """Create an in-memory overlay stage that sublayers *source_stage*.

    The returned stage's edit target is set to its own root layer so all
    authoring goes there, leaving the source layer untouched.

    Args:
        source_stage: The PhysX asset stage to overlay.

    Returns:
        New in-memory ``Usd.Stage`` with ``source_stage``'s root layer as a sublayer.
    """
    delta_stage = Usd.Stage.CreateInMemory()
    delta_stage.GetRootLayer().subLayerPaths.append(source_stage.GetRootLayer().identifier)
    delta_stage.SetEditTarget(delta_stage.GetEditTargetForLocalLayer(delta_stage.GetRootLayer()))

    source_default = source_stage.GetDefaultPrim()
    if source_default:
        delta_stage.SetDefaultPrim(delta_stage.GetPrimAtPath(source_default.GetPath()))

    return delta_stage


# ---------------------------------------------------------------------------
# Top-level interface
# ---------------------------------------------------------------------------


def convert_physx_asset_to_mjc(
    source_stage: Usd.Stage,
    *,
    delta_stage: Usd.Stage | None = None,
) -> None:
    """Convert a PhysX USD asset to MuJoCo/Newton in a single pass.

    Passes: (1) warn on multi-DOF D6/spherical joints, (2) ``MjcJointAPI`` +
    ``MjcActuator`` for revolute/prismatic joints, (3) mimic → ``NewtonMimicAPI``,
    (4) articulation roots → ``NewtonArticulationRootAPI``.

    Args:
        source_stage: Fully assembled PhysX USD stage to read from.
        delta_stage: Optional overlay that must already sublayer ``source_stage``
            (see :func:`make_delta_stage`). All authoring goes to its root layer,
            leaving ``source_stage`` untouched. When ``None``, edits land on
            ``source_stage``.

    Raises:
        ValueError: If ``delta_stage`` does not overlay ``source_stage``.
    """
    if delta_stage is not None:
        # delta_stage must overlay source_stage or we'd edit the wrong layer.
        source_root = source_stage.GetRootLayer()
        if source_root not in delta_stage.GetLayerStack(includeSessionLayers=False):
            raise ValueError(
                f"delta_stage does not overlay source_stage: source root layer "
                f"'{source_root.identifier}' is not in delta_stage's layer stack. "
                f"Use make_delta_stage(source_stage) to create a valid overlay."
            )
        work_stage = delta_stage
        work_stage.SetEditTarget(work_stage.GetEditTargetForLocalLayer(work_stage.GetRootLayer()))
    else:
        work_stage = source_stage

    # Ensure the MjcActuator scope exists.
    default_prim = work_stage.GetDefaultPrim()
    if default_prim:
        scope_path = default_prim.GetPath().AppendChild("Physics")
    else:
        scope_path = Sdf.Path("/Physics")
    if not work_stage.GetPrimAtPath(scope_path).IsValid():
        UsdGeom.Scope.Define(work_stage, scope_path)

    # Pass 1: warn about unconverted multi-DOF joints (D6 / spherical).
    skipped = warn_unconverted_multi_dof_joints(work_stage)
    if skipped:
        _logger.info(f"Skipped {skipped} multi-DOF joint(s); only single-axis joints are converted")

    # Pass 2 & 3: MjcJointAPI + actuators for single-axis joints.
    # Apply the API only if missing, but always (re)author mjc:* attributes so a
    # partially-converted joint is completed. Actuator naming/idempotency and
    # leaf-name collisions across the hierarchy are handled inside
    # create_mjc_actuator_from_physics.
    for prim in work_stage.Traverse():
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        if "MjcJointAPI" not in prim.GetAppliedSchemas():
            prim.ApplyAPI("MjcJointAPI")
        convert_physx_to_mjc(prim, author_reference=False)
        create_mjc_actuator_from_physics(prim, work_stage, scope_path)

    # Pass 3: mimic joints.
    mimic_count = convert_physx_mimic_to_newton(work_stage)
    _logger.info(f"Converted {mimic_count} PhysxMimicJointAPI instance(s) → NewtonMimicAPI")

    # Pass 4: articulation roots.
    art_count = convert_physx_articulation_to_newton(work_stage)
    _logger.info(f"Converted {art_count} PhysxArticulationAPI prim(s) → NewtonArticulationRootAPI")
