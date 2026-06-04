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

"""Demonstrate basic synthetic data generation with a writer and render product."""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb.settings
import omni.replicator.core as rep
import omni.usd


def run_example():
    """Run a basic SDG pipeline capturing RGB and bounding box data."""
    # Create a new stage and disable capture on play
    omni.usd.get_context().new_stage()
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Setup the stage with a dome light and a cube
    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
    cube = rep.functional.create.cube(parent="/World", name="Cube")
    rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")

    # Create a render product using the viewport perspective camera
    cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
    rp = rep.create.render_product(cam, (512, 512), name="MyRenderProduct")

    # Write data using the basic writer with the rgb and bounding box annotators
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), "_out_basic_writer")
    backend.initialize(output_dir=out_dir)
    print(f"Output directory: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(backend=backend, rgb=True, bounding_box_2d_tight=True)
    writer.attach(rp)

    # Trigger a data capture request (data will be written to disk by the writer)
    for i in range(3):
        print(f"Step {i}")
        rep.orchestrator.step()

    # Wait for the data to be written to disk and clean up resources
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


run_example()

# <start-sdg-getting-started-01-test>
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
    # BasicWriter with rgb + bounding_box_2d_tight writes per capture:
    # 1 rgb png, 1 bbox npy, 1 bbox labels json, 1 bbox prim_paths json (= 2 json per capture).
    num_captures = 3
    out_dir = os.path.join(os.getcwd(), "_out_basic_writer")
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": num_captures, "npy": num_captures, "json": num_captures * 2},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-sdg-getting-started-01-test>

simulation_app.close()
