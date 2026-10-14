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

"""Regression for Newton mimic relationships in packaged MJCF physics variants."""

import gc
import os
import tempfile

import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from omni.physics.core import get_physics_simulation_interface
from pxr import Usd

_MJCF = """<mujoco model="mimic_profile">
  <worldbody><body name="base"><geom type="sphere" size="0.05" mass="1"/>
    <body name="left"><joint name="finger_joint1" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="0.1"/></body>
    <body name="right"><joint name="finger_joint2" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="0.1"/></body>
  </body></worldbody>
  <equality><joint joint1="finger_joint1" joint2="finger_joint2" polycoef="0.01 2 0 0 0"/></equality>
</mujoco>"""


class TestMjcfMimicProfile(omni.kit.test.AsyncTestCase):
    """Verify the default imported package preserves a complete mimic in both variants."""

    async def test_mimic_relation_available_in_physx_and_mujoco(self) -> None:
        """A mimic API in the common physics layer must carry its target there too."""
        with tempfile.TemporaryDirectory(prefix="mimic_profile_") as work_dir:
            mjcf_path = os.path.join(work_dir, "mimic.xml")
            with open(mjcf_path, "w", encoding="utf-8") as stream:
                stream.write(_MJCF)
            config = MJCFImporterConfig(mjcf_path=mjcf_path, usd_path=work_dir, run_asset_transformer=True)
            output_path = MJCFImporter(config).import_mjcf()
            stage = Usd.Stage.Open(output_path)
            self.assertIsNotNone(stage)
            root = stage.GetDefaultPrim()
            variants = root.GetVariantSet("Physics")
            expected_target = "/mimic_profile/Geometry/base/right/finger_joint2"
            joint_path = "/mimic_profile/Geometry/base/left/finger_joint1"

            self.assertTrue({"physx", "mujoco"}.issubset(set(variants.GetVariantNames())))
            for variant in ("physx", "mujoco"):
                with self.subTest(variant=variant):
                    self.assertTrue(variants.SetVariantSelection(variant))
                    joint = stage.GetPrimAtPath(joint_path)
                    self.assertTrue(joint.HasAPI("NewtonMimicAPI"))
                    relationship = joint.GetRelationship("newton:mimicJoint")
                    self.assertEqual([str(path) for path in relationship.GetTargets()], [expected_target])
                    self.assertTrue(stage.GetPrimAtPath(expected_target).IsValid())
                    self.assertAlmostEqual(joint.GetAttribute("newton:mimicCoef0").Get(), 0.01)
                    self.assertAlmostEqual(joint.GetAttribute("newton:mimicCoef1").Get(), 2.0)

            # PhysX validates NewtonMimicAPI on Play in the reopened Kit stage.
            self.assertTrue(variants.SetVariantSelection("physx"))
            stage.GetRootLayer().Save()
            del relationship, joint, variants, root, stage
            gc.collect()

            stage = joint = subscription = None
            timeline = omni.timeline.get_timeline_interface()
            try:
                opened, stage = stage_utils.open_stage(output_path)
                self.assertTrue(opened)
                self.assertEqual(omni.usd.get_context().get_stage().GetRootLayer().realPath, output_path)
                self.assertTrue(SimulationManager.switch_physics_engine("physx"))
                joint = stage.GetPrimAtPath(joint_path)
                self.assertEqual(
                    [str(path) for path in joint.GetRelationship("newton:mimicJoint").GetTargets()],
                    [expected_target],
                )
                self.assertTrue(stage.GetPrimAtPath(expected_target).IsValid())
                steps = [0]
                subscription = get_physics_simulation_interface().subscribe_physics_on_step_events(
                    pre_step=False,
                    order=0,
                    on_update=lambda dt, context: steps.__setitem__(0, steps[0] + 1),
                )
                timeline.play()
                for _ in range(30):
                    await omni.kit.app.get_app().next_update_async()
                self.assertGreater(steps[0], 0, "The reopened PhysX stage did not simulate")
            finally:
                timeline.stop()
                subscription = None
                joint = stage = None
                stage_utils.create_new_stage()
                gc.collect()
