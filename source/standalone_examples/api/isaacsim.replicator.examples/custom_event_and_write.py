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

"""Demonstrate custom event-triggered randomization and data capture."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os

import carb.settings
import omni.replicator.core as rep
import omni.usd

omni.usd.get_context().new_stage()

# Set global random seed for the replicator randomizer to ensure reproducibility
rep.set_global_seed(11)

# Setting capture on play to False will prevent the replicator from capturing data each frame
rep.orchestrator.set_capture_on_play(False)

# Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

rep.functional.create.xform(name="World")
rep.functional.create.distant_light(intensity=4000, rotation=(315, 0, 0), parent="/World", name="DistantLight")
small_cube = rep.functional.create.cube(scale=0.75, position=(-1.5, 1.5, 0), parent="/World", name="SmallCube")
large_cube = rep.functional.create.cube(scale=1.25, position=(1.5, -1.5, 0), parent="/World", name="LargeCube")

# Graph-based randomizations triggered on custom events
with rep.trigger.on_custom_event(event_name="randomize_small_cube"):
    small_cube_node = rep.get.prim_at_path(small_cube.GetPath())
    with small_cube_node:
        rep.randomizer.rotation()

with rep.trigger.on_custom_event(event_name="randomize_large_cube"):
    large_cube_node = rep.get.prim_at_path(large_cube.GetPath())
    with large_cube_node:
        rep.randomizer.rotation()

# Use the disk backend to write the data to disk
out_dir = os.path.join(os.getcwd(), "_out_custom_event")
print(f"Writing data to {out_dir}")
backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir=out_dir)

cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
rp = rep.create.render_product(cam, (512, 512))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(backend=backend, rgb=True)
writer.attach(rp)


def run_example() -> None:
    """Run custom event randomization and capture sequence."""
    print("Capturing at original positions")
    rep.orchestrator.step(rt_subframes=8)

    print("Randomizing small cube rotation (graph-based) and capturing...")
    rep.utils.send_og_event(event_name="randomize_small_cube")
    rep.orchestrator.step(rt_subframes=8)

    print("Moving small cube position (USD API) and capturing...")
    small_cube.GetAttribute("xformOp:translate").Set((-1.5, 1.5, -2))
    rep.orchestrator.step(rt_subframes=8)

    print("Randomizing large cube rotation (graph-based) and capturing...")
    rep.utils.send_og_event(event_name="randomize_large_cube")
    rep.orchestrator.step(rt_subframes=8)

    print("Moving large cube position (USD API) and capturing...")
    large_cube.GetAttribute("xformOp:translate").Set((1.5, -1.5, 2))
    rep.orchestrator.step(rt_subframes=8)

    # Wait until all the data is saved to disk and cleanup writer and render product
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


run_example()

# <start-custom-event-and-write-test>
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
    # 5 captures (initial + 4 events), BasicWriter rgb-only writes 1 png per capture.
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": 5},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-custom-event-and-write-test>

simulation_app.close()
