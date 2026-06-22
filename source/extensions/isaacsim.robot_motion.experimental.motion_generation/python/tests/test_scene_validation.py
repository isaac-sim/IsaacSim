# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify scene validation for ancestor scaling before world binding.

The tests ensure scaled ancestors are rejected in flat and nested hierarchies,
including uniform and negative scale, while local scale on tracked prims,
identity-scaled parents, and parentless prims remain valid.
"""

import omni.kit.test
from isaacsim.core.experimental.objects import Cube, Sphere
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.robot_motion.experimental.motion_generation.impl.utils import scene_validation
from omni.kit.app import get_app


class TestSceneValidation(omni.kit.test.AsyncTestCase):
    """Test class for scene validation of ancestor transforms and scaling."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await create_new_stage_async()
        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        await get_app().next_update_async()

    async def test_scene_validation_detects_scaled_parent(self) -> None:
        """Test that scene validation detects when a parent has non-identity scaling."""
        # Create a parent Xform with scaling
        parent_xform = Sphere(
            paths="/World/ParentXform",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[2.0, 3.0, 4.0],  # Non-identity scaling
        )

        # Create a child sphere under the scaled parent
        child_sphere = Sphere(
            paths="/World/ParentXform/ChildSphere",
            radii=0.5,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/ParentXform/ChildSphere"])

        # Should detect the parent as invalid
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/ParentXform", invalid_ancestors)

    async def test_scene_validation_detects_both_parent_and_local_scaling(self) -> None:
        """Test detection when both parent AND child have scaling."""
        # Parent with scaling
        parent_xform = Cube(
            paths="/World/Parent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[2.0, 2.0, 2.0],
        )

        # Child with its own scaling
        child_cube = Cube(
            paths="/World/Parent/Cube",
            sizes=0.1,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[3.0, 3.0, 3.0],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/Parent/Cube"])

        # Should detect the parent as invalid
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/Parent", invalid_ancestors)

    async def test_scene_validation_accepts_identity_parent_scaling(self) -> None:
        """Test that scene validation accepts a prim when parent has identity scaling."""
        # Create parent with identity scaling
        parent_xform = Sphere(
            paths="/World/IdentityParent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],  # Identity scaling
        )

        # Create child sphere with its own scaling
        child_sphere = Sphere(
            paths="/World/IdentityParent/Sphere",
            radii=0.5,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 2.0, 3.0],  # Local scaling is allowed
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/IdentityParent/Sphere"])

        # Should NOT detect any invalid ancestors
        self.assertEqual(len(invalid_ancestors), 0)

    async def test_scene_validation_accepts_prim_without_parent(self) -> None:
        """Test that scene validation accepts a prim at the root level (no parent Xform)."""
        # Create sphere directly under /World
        sphere = Sphere(
            paths="/World/RootSphere",
            radii=0.5,
            positions=[1.0, 2.0, 3.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.5, 1.5, 1.5],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/RootSphere"])

        # Should NOT detect any invalid ancestors
        self.assertEqual(len(invalid_ancestors), 0)

    async def test_scene_validation_detects_nested_hierarchy_with_scaling(self) -> None:
        """Test detection with multiple levels of hierarchy where scaling.

        occurs at an intermediate level (not direct parent).
        """
        # Create a 3-level hierarchy:
        # Root -> Parent (scaled) -> Child (identity) -> Grandchild (our tracked prim)

        # Root with identity
        root_xform = Sphere(
            paths="/World/Root",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        # Parent with scaling
        parent_xform = Sphere(
            paths="/World/Root/Parent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 2.0, 3.0],
        )

        # Child with identity
        child_xform = Sphere(
            paths="/World/Root/Parent/Child",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        # Grandchild sphere (tracked prim)
        grandchild_sphere = Sphere(
            paths="/World/Root/Parent/Child/Sphere",
            radii=0.3,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/Root/Parent/Child/Sphere"])

        # Should detect the grandparent as invalid
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/Root/Parent", invalid_ancestors)

    async def test_scene_validation_detects_uniform_parent_scaling(self) -> None:
        """Test that even uniform parent scaling is detected.

        ANY parent scaling (uniform or non-uniform) is invalid.
        """
        # Parent with uniform scaling
        parent_xform = Cube(
            paths="/World/UniformParent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[2.0, 2.0, 2.0],  # Uniform scaling
        )

        # Child with uniform scaling
        child_cube = Cube(
            paths="/World/UniformParent/Cube",
            sizes=0.2,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[3.0, 3.0, 3.0],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/UniformParent/Cube"])

        # Should detect the parent as invalid (even though scaling is uniform)
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/UniformParent", invalid_ancestors)

    async def test_scene_validation_detects_negative_scale_in_parent(self) -> None:
        """Test that negative scaling in parent (mirrored/left-handed coordinate system).

        is detected as invalid.
        """
        # Parent with negative scaling
        parent_xform = Sphere(
            paths="/World/MirroredParent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[-1.0, 1.0, 1.0],  # Negative X scale
        )

        # Child sphere
        child_sphere = Sphere(
            paths="/World/MirroredParent/Sphere",
            radii=0.5,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        await get_app().next_update_async()

        # Check for invalid ancestors
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=["/World/MirroredParent/Sphere"])

        # Should detect the parent as invalid
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/MirroredParent", invalid_ancestors)

    async def test_scene_validation_with_multiple_prims(self) -> None:
        """Test validation with multiple prims at once, some valid and some with invalid ancestors."""
        # Create first hierarchy with valid ancestors
        valid_parent = Sphere(
            paths="/World/ValidParent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],  # Identity scaling
        )
        valid_child = Sphere(
            paths="/World/ValidParent/Sphere",
            radii=0.5,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        # Create second hierarchy with invalid ancestors
        invalid_parent = Cube(
            paths="/World/InvalidParent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[2.0, 2.0, 2.0],  # Non-identity scaling
        )
        invalid_child = Cube(
            paths="/World/InvalidParent/Cube",
            sizes=0.2,
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        await get_app().next_update_async()

        # Check both prims at once
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(
            prim_paths=["/World/ValidParent/Sphere", "/World/InvalidParent/Cube"]
        )

        # Should only detect the invalid parent
        self.assertGreater(len(invalid_ancestors), 0)
        self.assertIn("/World/InvalidParent", invalid_ancestors)
        self.assertNotIn("/World/ValidParent", invalid_ancestors)

    async def test_scene_validation_caches_checked_ancestors(self) -> None:
        """Test that validation efficiently caches checked ancestors when validating.

        multiple prims that share ancestors.
        """
        # Create parent with scaling
        parent = Sphere(
            paths="/World/Parent",
            positions=[0.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[2.0, 2.0, 2.0],
        )

        # Create multiple children under the same parent
        child1 = Sphere(
            paths="/World/Parent/Child1",
            radii=0.5,
            positions=[1.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        child2 = Cube(
            paths="/World/Parent/Child2",
            sizes=0.3,
            positions=[-1.0, 0.0, 0.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[1.0, 1.0, 1.0],
        )

        await get_app().next_update_async()

        # Validate both children (they share the same invalid parent)
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(
            prim_paths=["/World/Parent/Child1", "/World/Parent/Child2"]
        )

        # Should detect the parent as invalid
        # The parent should only appear once even though we checked two children
        self.assertTrue(len(invalid_ancestors) == 1)
        self.assertIn("/World/Parent", invalid_ancestors)
