# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate multi-camera data capture with custom writers and annotators."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os
from typing import Any

import carb.settings
import omni.replicator.core as rep
import omni.usd
from omni.replicator.core import Writer
from omni.replicator.core.backends import DiskBackend
from omni.replicator.core.functional import write_image

NUM_FRAMES = 5


def cube_color_randomizer() -> Any:
    """Randomize cube color every frame using a graph-based replicator randomizer."""
    cube_prims = rep.get.prims(path_pattern="Cube")
    with cube_prims:
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
    return cube_prims.node


class MyWriter(Writer):
    """Write RGB annotator data to disk from multiple render products."""

    def __init__(self, rgb: bool = True) -> None:
        # Organize data from render product perspective (legacy, annotator, renderProduct)
        self.data_structure = "renderProduct"
        self.annotators = []
        self._frame_id = 0
        if rgb:
            # Create a new rgb annotator and add it to the writer's list of annotators
            self.annotators.append(rep.annotators.get("rgb"))
        # Create writer output directory and initialize DiskBackend
        output_dir = os.path.join(os.getcwd(), "_out_mc_writer")
        print(f"Writing writer data to {output_dir}")
        self.backend = DiskBackend(output_dir=output_dir, overwrite=True)

    def write(self, data: dict[str, Any]) -> None:
        """Write RGB frames from each render product to disk."""
        if "renderProducts" in data:
            for rp_name, rp_data in data["renderProducts"].items():
                if "rgb" in rp_data:
                    file_path = f"{rp_name}_frame_{self._frame_id}.png"
                    self.backend.schedule(write_image, data=rp_data["rgb"]["data"], path=file_path)
        self._frame_id += 1


rep.WriterRegistry.register(MyWriter)

# Create a new stage
omni.usd.get_context().new_stage()

# Set global random seed for the replicator randomizer
rep.set_global_seed(11)

# Disable capture on play to capture data manually using step
rep.orchestrator.set_capture_on_play(False)

# Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

# Setup stage
rep.functional.create.xform(name="World")
rep.functional.create.dome_light(intensity=900, parent="/World", name="DomeLight")
cube = rep.functional.create.cube(parent="/World", name="Cube", semantics={"class": "my_cube"})

# Register the graph-based cube color randomizer to trigger on every frame
rep.randomizer.register(cube_color_randomizer)
with rep.trigger.on_frame():
    rep.randomizer.cube_color_randomizer()

# Create cameras
cam_top = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), parent="/World", name="CamTop")
cam_side = rep.functional.create.camera(position=(2, 2, 0), look_at=(0, 0, 0), parent="/World", name="CamSide")
cam_persp = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="CamPersp")

# Create the render products
rp_top = rep.create.render_product(cam_top, resolution=(320, 320), name="RpTop")
rp_side = rep.create.render_product(cam_side, resolution=(640, 640), name="RpSide")
rp_persp = rep.create.render_product(cam_persp, resolution=(1024, 1024), name="RpPersp")

# Example of accessing the data through a custom writer
writer = rep.WriterRegistry.get("MyWriter")
writer.initialize(rgb=True)
writer.attach([rp_top, rp_side, rp_persp])

# Example of accessing the data directly through annotators
rgb_annotators = []
for rp in [rp_top, rp_side, rp_persp]:
    # Create a new rgb annotator for each render product
    rgb = rep.annotators.get("rgb")
    # Attach the annotator to the render product
    rgb.attach(rp)
    rgb_annotators.append(rgb)

# Create annotator output directory
output_dir_annot = os.path.join(os.getcwd(), "_out_mc_annot")
print(f"Writing annotator data to {output_dir_annot}")
os.makedirs(output_dir_annot, exist_ok=True)

for i in range(NUM_FRAMES):
    print(f"Step {i}")
    # The step function triggers registered graph-based randomizers, collects data from annotators,
    # and invokes the write function of attached writers with the annotator data
    rep.orchestrator.step(rt_subframes=32)
    for j, rgb_annot in enumerate(rgb_annotators):
        file_path = os.path.join(output_dir_annot, f"rp{j}_step_{i}.png")
        write_image(path=file_path, data=rgb_annot.get_data())

# Wait for the data to be written and release resources
rep.orchestrator.wait_until_complete()
writer.detach()
for annot in rgb_annotators:
    annot.detach()
for rp in [rp_top, rp_side, rp_persp]:
    rp.destroy()

# <start-multi-camera-test>
import argparse
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
args, _ = parser.parse_known_args()

if args.test:
    # 3 render products x NUM_FRAMES captures each, both writer and annotator dirs.
    expected_png_count = 3 * NUM_FRAMES
    output_dir_writer = os.path.join(os.getcwd(), "_out_mc_writer")
    if not validate_folder_contents(
        path=output_dir_writer,
        recursive=True,
        expected_counts={"png": expected_png_count},
        fail_on_empty_files=True,
    ):
        print(f"[SDG][Test][FAIL] Output validation failed for {output_dir_writer}")
        sys.exit(1)
    if not validate_folder_contents(
        path=output_dir_annot,
        recursive=True,
        expected_counts={"png": expected_png_count},
        fail_on_empty_files=True,
    ):
        print(f"[SDG][Test][FAIL] Output validation failed for {output_dir_annot}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {output_dir_writer} and {output_dir_annot}")
# <end-multi-camera-test>

simulation_app.close()
