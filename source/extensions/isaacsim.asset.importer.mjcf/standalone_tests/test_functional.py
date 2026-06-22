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

"""Verifies standalone MJCF imports for pendulum and cartpole models. Covers USD creation, imported body and joint structure, revolute and prismatic joint typing, and geometry presence."""

from __future__ import annotations

import os
import tempfile
import unittest

from pxr import Usd, UsdGeom, UsdPhysics

# Minimal two-body MJCF with a hinge joint and geometry.
_PENDULUM_MJCF = """\
<mujoco model="pendulum">
  <compiler angle="radian"/>

  <default>
    <joint damping="0.5"/>
    <geom density="1000" rgba="0.8 0.3 0.1 1"/>
  </default>

  <worldbody>
    <body name="base" pos="0 0 1.0">
      <geom name="base_geom" type="sphere" size="0.1"/>
      <joint name="swing" type="hinge" axis="0 1 0" range="-1.57 1.57"/>

      <body name="arm" pos="0 0 -0.5">
        <geom name="arm_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.04"/>

        <body name="tip" pos="0 0 -0.4">
          <geom name="tip_geom" type="sphere" size="0.06"/>
          <joint name="tip_hinge" type="hinge" axis="0 1 0" range="-0.5 0.5"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="swing_motor" joint="swing" gear="100"/>
    <motor name="tip_motor" joint="tip_hinge" gear="50"/>
  </actuator>
</mujoco>
"""

# Cart-pole: prismatic slide + revolute hinge.
_CARTPOLE_MJCF = """\
<mujoco model="cartpole">
  <compiler angle="degree"/>

  <worldbody>
    <body name="cart" pos="0 0 0.5">
      <geom name="cart_geom" type="box" size="0.2 0.1 0.05" rgba="0.3 0.5 0.8 1"/>
      <joint name="slide" type="slide" axis="1 0 0" range="-2 2"/>

      <body name="pole" pos="0 0 0.05">
        <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.6" size="0.02"/>
        <joint name="hinge" type="hinge" axis="0 1 0" range="-180 180"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="slide_motor" joint="slide" gear="10"/>
  </actuator>
</mujoco>
"""


@unittest.skipUnless(
    os.environ.get("ISAACSIM_TEST_MJCF_IMPORT"),
    "Skipped: mujoco-usd-converter has a numpy array comparison bug "
    "(set_schema_attribute ValueError). Set ISAACSIM_TEST_MJCF_IMPORT=1 to run.",
)
class TestMJCFImportPendulum(unittest.TestCase):
    """Functional tests: import pendulum MJCF → validate USD stage."""

    def setUp(self) -> None:
        """Create a temp directory and write the test MJCF."""
        self._tmpdir = tempfile.mkdtemp(prefix="mjcf_test_")
        self._mjcf_path = os.path.join(self._tmpdir, "pendulum.xml")
        with open(self._mjcf_path, "w") as f:
            f.write(_PENDULUM_MJCF)

    def tearDown(self) -> None:
        """Clean up temp directory."""
        import shutil

        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _import(self) -> Usd.Stage:
        """Run the MJCF importer and return the resulting USD stage.

        Returns:
            The USD stage opened from the generated import output.
        """
        from isaacsim.asset.importer.mjcf import MJCFImporter
        from isaacsim.asset.importer.mjcf.impl.config import MJCFImporterConfig

        config = MJCFImporterConfig(
            mjcf_path=self._mjcf_path,
            usd_path=self._tmpdir,
        )
        importer = MJCFImporter(config=config)
        usd_path = importer.import_mjcf()
        self.assertTrue(os.path.isfile(usd_path), f"USD file not created: {usd_path}")

        stage = Usd.Stage.Open(usd_path)
        self.assertIsNotNone(stage, "Failed to open generated USD stage")
        return stage

    def test_import_produces_usd(self) -> None:
        """Verify import_mjcf produces a valid USD file."""
        stage = self._import()
        self.assertTrue(stage.GetPseudoRoot().GetChildren())

    def test_bodies_exist(self) -> None:
        """Verify all MJCF bodies become USD prims."""
        stage = self._import()
        expected_bodies = {"base", "arm", "tip"}

        found_bodies = set()
        for prim in stage.TraverseAll():
            if prim.GetName() in expected_bodies:
                found_bodies.add(prim.GetName())

        self.assertEqual(found_bodies, expected_bodies, f"Missing bodies: {expected_bodies - found_bodies}")

    def test_joints_exist(self) -> None:
        """Verify all MJCF joints become USD joint prims."""
        stage = self._import()
        expected_joints = {"swing", "tip_hinge", "PhysicsFixedJoint"}

        found_joints = set()
        for prim in stage.TraverseAll():
            if prim.IsA(UsdPhysics.Joint):
                found_joints.add(prim.GetName())

        self.assertEqual(
            found_joints,
            expected_joints,
            f"Missing joints: {expected_joints - found_joints}; extra joints: {found_joints - expected_joints}",
        )

    def test_hinge_is_revolute(self) -> None:
        """Verify MJCF hinge joints map to USD RevoluteJoint."""
        stage = self._import()

        for prim in stage.TraverseAll():
            if prim.GetName() == "swing" and prim.IsA(UsdPhysics.Joint):
                self.assertTrue(
                    prim.IsA(UsdPhysics.RevoluteJoint),
                    f"swing should be RevoluteJoint, got {prim.GetTypeName()}",
                )
                return
        self.fail("swing joint prim not found")

    def test_geometry_exists(self) -> None:
        """Verify visual geometry prims exist in the stage."""
        stage = self._import()

        geom_count = 0
        for prim in stage.TraverseAll():
            if prim.IsA(UsdGeom.Gprim):
                geom_count += 1

        self.assertGreater(geom_count, 0, "No geometry prims found in stage")


@unittest.skipUnless(
    os.environ.get("ISAACSIM_TEST_MJCF_IMPORT"),
    "Skipped: mujoco-usd-converter has a numpy array comparison bug "
    "(set_schema_attribute ValueError). Set ISAACSIM_TEST_MJCF_IMPORT=1 to run.",
)
class TestMJCFImportCartpole(unittest.TestCase):
    """Functional tests: import cart-pole MJCF → validate USD stage."""

    def setUp(self) -> None:
        """Create a temp directory and write the cart-pole MJCF."""
        self._tmpdir = tempfile.mkdtemp(prefix="mjcf_test_")
        self._mjcf_path = os.path.join(self._tmpdir, "cartpole.xml")
        with open(self._mjcf_path, "w") as f:
            f.write(_CARTPOLE_MJCF)

    def tearDown(self) -> None:
        """Clean up temp directory."""
        import shutil

        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _import(self) -> Usd.Stage:
        """Run the MJCF importer and return the resulting USD stage.

        Returns:
            The USD stage opened from the generated import output.
        """
        from isaacsim.asset.importer.mjcf import MJCFImporter
        from isaacsim.asset.importer.mjcf.impl.config import MJCFImporterConfig

        config = MJCFImporterConfig(
            mjcf_path=self._mjcf_path,
            usd_path=self._tmpdir,
        )
        importer = MJCFImporter(config=config)
        usd_path = importer.import_mjcf()
        self.assertTrue(os.path.isfile(usd_path), f"USD file not created: {usd_path}")

        stage = Usd.Stage.Open(usd_path)
        self.assertIsNotNone(stage, "Failed to open generated USD stage")
        return stage

    def test_import_produces_usd(self) -> None:
        """Verify import_mjcf produces a valid USD file."""
        stage = self._import()
        self.assertTrue(stage.GetPseudoRoot().GetChildren())

    def test_slide_is_prismatic(self) -> None:
        """Verify MJCF slide joints map to USD PrismaticJoint."""
        stage = self._import()

        for prim in stage.TraverseAll():
            if prim.GetName() == "slide" and prim.IsA(UsdPhysics.Joint):
                self.assertTrue(
                    prim.IsA(UsdPhysics.PrismaticJoint),
                    f"slide should be PrismaticJoint, got {prim.GetTypeName()}",
                )
                return
        self.fail("slide joint prim not found")

    def test_hinge_is_revolute(self) -> None:
        """Verify MJCF hinge joints map to USD RevoluteJoint."""
        stage = self._import()

        for prim in stage.TraverseAll():
            if prim.GetName() == "hinge" and prim.IsA(UsdPhysics.Joint):
                self.assertTrue(
                    prim.IsA(UsdPhysics.RevoluteJoint),
                    f"hinge should be RevoluteJoint, got {prim.GetTypeName()}",
                )
                return
        self.fail("hinge joint prim not found")

    def test_both_joints_present(self) -> None:
        """Verify both slide and hinge joints exist."""
        stage = self._import()
        expected = {"slide", "hinge"}

        found = set()
        for prim in stage.TraverseAll():
            if prim.IsA(UsdPhysics.Joint):
                found.add(prim.GetName())

        self.assertEqual(found, expected, f"Missing joints: {expected - found}")


if __name__ == "__main__":
    unittest.main()
