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
"""Demonstrate semantic segmentation with GeomSubset prims and per-subset segmentation."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import sys

import carb.settings
import omni.replicator.core as rep
import omni.usd
import pxr

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test",
    action="store_true",
    help="Check semantic class labels.",
)
args, _unknown = parser.parse_known_args()

# Expected results if perSubsetSegmentation is True
_EXPECTED_CLASSES_PER_SUBSET_TRUE = frozenset(
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

# Expected results if perSubsetSegmentation is False
_EXPECTED_CLASSES_PER_SUBSET_FALSE = frozenset(
    {
        "BACKGROUND",
        "UNLABELLED",
        "middle_cube_no_geomsubset",
        "left_cube_semantics_on_mesh",
    }
)


def run_example(run_test: bool) -> None:
    """Run per-subset semantic segmentation capture and optional validation."""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    carb_settings = carb.settings.get_settings()
    per_subset = carb_settings.get_as_bool("/syntheticdata/sensors/perSubsetSegmentation")
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
    right_cube.CreateAttribute("subsetFamily:materialBind:familyType", pxr.Sdf.ValueTypeNames.Token).Set("partition")
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
    rep.orchestrator.step()
    id_to_labels = annot.get_data()["info"]["idToLabels"]
    annot.detach()
    render_product.destroy()

    # Get the semantic classes from the annotator
    classes = frozenset(
        str(entry["class"]) for entry in id_to_labels.values() if isinstance(entry, dict) and "class" in entry
    )

    # Check the semantic classes against the expected results
    if run_test:
        expected = _EXPECTED_CLASSES_PER_SUBSET_TRUE if per_subset else _EXPECTED_CLASSES_PER_SUBSET_FALSE
        if classes != expected:
            print(
                "Semantic class mismatch "
                f"(/syntheticdata/sensors/perSubsetSegmentation={per_subset!r}).\n"
                f"Expected: {sorted(expected)}\nActual: {sorted(classes)}",
                file=sys.stderr,
            )
            simulation_app.close()
            sys.exit(1)
        return

    # Print the semantic classes and the perSubsetSegmentation setting
    for k, val in sorted(
        id_to_labels.items(),
        key=lambda kv: (
            (0, int(kv[0]))
            if isinstance(kv[0], int) or (isinstance(kv[0], str) and kv[0].isdigit())
            else (1, str(kv[0]))
        ),
    ):
        print(f"{k}={val}")
    print(f"perSubsetSegmentation={per_subset!r}")


run_example(args.test)
simulation_app.close()
