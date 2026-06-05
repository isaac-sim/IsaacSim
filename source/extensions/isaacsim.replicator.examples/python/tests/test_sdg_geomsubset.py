# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb.settings
import omni.kit
import omni.replicator.core as rep
import omni.usd
import pxr


class TestSDGGeomSubset(omni.kit.test.AsyncTestCase):

    PER_SUBSET_SETTING = "/syntheticdata/sensors/perSubsetSegmentation"

    EXPECTED_CLASSES_PER_SUBSET_TRUE = frozenset(
        {
            "BACKGROUND",
            "UNLABELLED",
            "middle_cube_no_geomsubset",
            "left_cube_semantics_on_mesh",
            "face_0",
            "face_2",
            "face_5",
        }
    )
    EXPECTED_CLASSES_PER_SUBSET_FALSE = frozenset(
        {
            "BACKGROUND",
            "UNLABELLED",
            "middle_cube_no_geomsubset",
            "left_cube_semantics_on_mesh",
        }
    )

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self._original_per_subset = carb.settings.get_settings().get(self.PER_SUBSET_SETTING)

    async def tearDown(self):
        if self._original_per_subset is not None:
            carb.settings.get_settings().set(self.PER_SUBSET_SETTING, self._original_per_subset)
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def _capture_classes_async(self, per_subset_segmentation: bool) -> frozenset[str]:
        carb.settings.get_settings().set(self.PER_SUBSET_SETTING, per_subset_segmentation)
        await omni.usd.get_context().new_stage_async()

        stage = omni.usd.get_context().get_stage()
        rep.functional.create.xform(name="World")
        rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")

        left_cube = rep.functional.create.cube(
            position=(2, 0, 0),
            name="left_cube_semantics_on_mesh",
            parent="/World",
            as_mesh=True,
            semantics={"class": "left_cube_semantics_on_mesh"},
        )
        left_cube.CreateAttribute("subsetFamily:materialBind:familyType", pxr.Sdf.ValueTypeNames.Token).Set("partition")
        left_cube.CreateAttribute("subsetFamily:metadata:familyType", pxr.Sdf.ValueTypeNames.Token).Set("partition")
        for face_idx in range(len(left_cube.GetAttribute("faceVertexCounts").Get())):
            face = stage.DefinePrim(f"{str(left_cube.GetPath())}/face_{face_idx}", "GeomSubset")
            face.CreateAttribute("elementType", pxr.Sdf.ValueTypeNames.Token).Set("face")
            face.CreateAttribute("familyName", pxr.Sdf.ValueTypeNames.Token).Set("materialBind")
            face.GetAttribute("indices").Set([face_idx])

        right_cube = rep.functional.create.cube(
            position=(-2, 0, 0),
            name="right_cube_semantics_on_geomsubset",
            parent="/World",
            as_mesh=True,
        )
        right_cube.CreateAttribute("subsetFamily:materialBind:familyType", pxr.Sdf.ValueTypeNames.Token).Set(
            "partition"
        )
        right_cube.CreateAttribute("subsetFamily:metadata:familyType", pxr.Sdf.ValueTypeNames.Token).Set("partition")
        for face_idx in range(len(right_cube.GetAttribute("faceVertexCounts").Get())):
            face = stage.DefinePrim(f"{str(right_cube.GetPath())}/face_{face_idx}", "GeomSubset")
            face.GetAttribute("indices").Set([face_idx])
            face.CreateAttribute("elementType", pxr.Sdf.ValueTypeNames.Token).Set("face")
            face.CreateAttribute("familyName", pxr.Sdf.ValueTypeNames.Token).Set("materialBind")
            rep.functional.modify.semantics(face, {"class": f"face_{face_idx}"})

        rep.functional.create.cube(
            position=(0, 0, 0),
            name="middle_cube_no_geomsubset",
            parent="/World",
            as_mesh=True,
            semantics={"class": "middle_cube_no_geomsubset"},
        )

        camera = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
        render_product = rep.create.render_product(camera, (720, 480))
        annot = rep.annotators.get("semantic_segmentation")
        annot.attach(render_product)
        await rep.orchestrator.step_async()
        id_to_labels = annot.get_data()["info"]["idToLabels"]
        annot.detach()
        render_product.destroy()

        return frozenset(
            str(entry["class"]) for entry in id_to_labels.values() if isinstance(entry, dict) and "class" in entry
        )

    async def test_sdg_geomsubset_per_subset_toggle(self):
        classes_false_a = await self._capture_classes_async(False)
        self.assertEqual(
            classes_false_a,
            self.EXPECTED_CLASSES_PER_SUBSET_FALSE,
            f"perSubsetSegmentation=False (first run) produced unexpected classes: {sorted(classes_false_a)}",
        )

        classes_true = await self._capture_classes_async(True)
        self.assertEqual(
            classes_true,
            self.EXPECTED_CLASSES_PER_SUBSET_TRUE,
            f"perSubsetSegmentation=True produced unexpected classes: {sorted(classes_true)}",
        )

        classes_false_b = await self._capture_classes_async(False)
        self.assertEqual(
            classes_false_b,
            self.EXPECTED_CLASSES_PER_SUBSET_FALSE,
            f"perSubsetSegmentation=False (after True) produced unexpected classes: {sorted(classes_false_b)}",
        )
