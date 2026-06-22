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

"""Verifies standalone importability and public API surface for isaacsim.asset.importer.utils without Omni modules. Covers PhysX helper reexports, submodule access, schema value formatting, mimic attribute naming, and authoring helper attributes on a USD stage."""

from __future__ import annotations

import sys
import unittest


class TestSmoke(unittest.TestCase):
    """Import and namespace validation."""

    def test_import_physx_types(self) -> None:
        """Verify physx_types enums are importable."""
        from isaacsim.asset.importer.utils.impl.physx_types import (
            PhysxAttr,
            PhysxMimicAttr,
            PhysxMimicRel,
            PhysxSchema,
        )

        self.assertTrue(len(PhysxAttr) > 0)
        self.assertTrue(len(PhysxSchema) > 0)
        self.assertTrue(len(PhysxMimicAttr) > 0)
        self.assertTrue(len(PhysxMimicRel) > 0)

    def test_no_omni_modules(self) -> None:
        """No omni.* modules should be loaded after import."""
        from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr  # noqa: F401

        omni_mods = [m for m in sys.modules if m.startswith("omni.")]
        self.assertEqual(omni_mods, [], f"Unexpected omni modules: {omni_mods}")

    def test_top_level_function_reexports(self) -> None:
        """Public helpers documented in ``python_api.md`` must be importable from the package root."""
        # If the package's ``__init__.py`` forgets to re-export these names, this import block raises ``ImportError``.
        from isaacsim.asset.importer.utils import (  # noqa: F401
            MESH_APPROXIMATION_MAP,
            PHYSICS_AXIS_MAP,
            ROBOT_TYPE_TOKENS,
            USD_GEOMETRY_TYPES,
            PhysxAttr,
            PhysxMimicAttr,
            PhysxMimicRel,
            PhysxSchema,
            add_joint_schemas,
            add_rigid_body_schemas,
            apply_fix_base,
            apply_joint_drives,
            apply_link_density,
            apply_mjc_actuator_gains,
            clean_mesh_operation,
            collision_from_visuals,
            create_robot_schema,
            delete_scope,
            enable_self_collision,
            fix_articulation_root_for_fixed_base,
            generate_mesh_uv_normals_operation,
            get_stage_id,
            merge_mesh,
            merge_meshes_operation,
            open_stage,
            parse_robot_name,
            remove_custom_scopes,
            resolve_unique_path,
            run_asset_transformer_profile,
            save_stage,
        )

    def test_submodules_still_accessible(self) -> None:
        """Importing the submodules through the package root must still work."""
        from isaacsim.asset.importer.utils import (  # noqa: F401
            asset_utils,
            importer_utils,
            merge_mesh_utils,
            physx_types,
            stage_utils,
        )

        # Spot-check that the submodule actually re-exports a real attribute.
        self.assertTrue(callable(importer_utils.collision_from_visuals))


class TestFunctional(unittest.TestCase):
    """PhysX type enum correctness tests."""

    def test_physx_attr_names(self) -> None:
        """Verify PhysxAttr enum values have correct attribute name format."""
        from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr

        for attr in PhysxAttr:
            self.assertIn(":", attr.name, f"{attr} missing colon in name")
            self.assertIsNotNone(attr.type, f"{attr} has None type")

    def test_physx_schema_values(self) -> None:
        """Verify PhysxSchema enum values are correct API names."""
        from isaacsim.asset.importer.utils.impl.physx_types import PhysxSchema

        self.assertEqual(PhysxSchema.JOINT_API.value, "PhysxJointAPI")
        self.assertEqual(PhysxSchema.ARTICULATION_API.value, "PhysxArticulationAPI")
        self.assertEqual(PhysxSchema.MIMIC_JOINT_API.value, "PhysxMimicJointAPI")

    def test_physx_mimic_format(self) -> None:
        """Verify mimic attr/rel format methods produce correct strings."""
        from isaacsim.asset.importer.utils.impl.physx_types import PhysxMimicAttr, PhysxMimicRel

        self.assertEqual(PhysxMimicAttr.GEARING.format("rotZ"), "physxMimicJoint:rotZ:gearing")
        self.assertEqual(PhysxMimicAttr.OFFSET.format("rotX"), "physxMimicJoint:rotX:offset")
        self.assertEqual(PhysxMimicRel.REFERENCE_JOINT.format("rotZ"), "physxMimicJoint:rotZ:referenceJoint")

    def test_create_physx_attrs_on_stage(self) -> None:
        """Verify PhysxAttr enums can create real USD attributes."""
        from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/joint")
        prim = joint.GetPrim()

        prim.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(0.1)
        prim.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.5)
        prim.CreateAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name, PhysxAttr.JOINT_MAX_VELOCITY.type).Set(180.0)

        self.assertAlmostEqual(prim.GetAttribute(PhysxAttr.JOINT_ARMATURE.name).Get(), 0.1)
        self.assertAlmostEqual(prim.GetAttribute(PhysxAttr.JOINT_FRICTION.name).Get(), 0.5)
        self.assertAlmostEqual(prim.GetAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name).Get(), 180.0)


if __name__ == "__main__":
    unittest.main()
