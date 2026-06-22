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

"""Demonstrate SDG performance with fabric writes and render wait options."""

import os
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb.settings
import omni.replicator.core as rep
import omni.usd

NUM_CUBES = 100
NUM_CAPTURES = 10


def run_example(wait_for_render: bool, write_to_fabric: bool) -> None:
    """Run SDG with the given render wait and fabric write settings."""
    print(f"\n[SDG] Running with wait_for_render={wait_for_render}, write_to_fabric={write_to_fabric}")
    omni.usd.get_context().new_stage()
    rep.orchestrator.set_capture_on_play(False)

    settings = carb.settings.get_settings()
    settings.set("rtx/post/dlss/execMode", 2)
    settings.set("/exts/omni.replicator.core/enableWriteToFabric", write_to_fabric)

    rng = rep.rng.ReplicatorRNG(seed=42)

    # Setup stage with a dome light and batch-created cubes
    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
    cubes = rep.functional.create_batch.cube(
        count=NUM_CUBES,
        parent="/World",
        name="Cube",
        semantics={"class": "my_cube"},
    )
    rep.functional.modify.scale(cubes, (0.2, 0.2, 0.2))

    # Create the camera and render product
    cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
    rp = rep.create.render_product(cam, (512, 512))

    # Write data using BasicWriter with rgb annotator
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), f"_out_fabric_{write_to_fabric}_wait_{wait_for_render}")
    backend.initialize(output_dir=out_dir)
    print(f"[SDG] Output directory: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(backend=backend, rgb=True)
    writer.attach(rp)

    # Randomize and capture, measuring timing for each phase
    randomization_times_ms = []
    capture_times_ms = []
    total_start = time.perf_counter()

    for i in range(NUM_CAPTURES):
        random_positions = rng.generator.uniform((-3.0, -3.0, -3.0), (3.0, 3.0, 3.0), size=(NUM_CUBES, 3))
        random_rotations = rng.generator.uniform((0.0, 0.0, 0.0), (360.0, 360.0, 360.0), size=(NUM_CUBES, 3))
        random_scales = rng.generator.uniform(0.1, 0.4, size=(NUM_CUBES, 3))

        rand_start = time.perf_counter()
        rep.functional.modify.pose(
            cubes,
            position_value=random_positions,
            rotation_value=random_rotations,
            scale_value=random_scales,
        )
        rep.functional.randomizer.display_color(cubes, rng=rng)
        rand_ms = (time.perf_counter() - rand_start) * 1000.0
        randomization_times_ms.append(rand_ms)

        cap_start = time.perf_counter()
        rep.orchestrator.step(wait_for_render=wait_for_render)
        cap_ms = (time.perf_counter() - cap_start) * 1000.0
        capture_times_ms.append(cap_ms)

        print(f"[SDG] Step {i}: randomization {rand_ms:.1f} ms, capture {cap_ms:.1f} ms")

    # Wait for all data to be written to disk
    print("[SDG] Waiting for all data to be written to disk..")
    rep.orchestrator.wait_until_complete()
    total_ms = (time.perf_counter() - total_start) * 1000.0

    avg_rand = sum(randomization_times_ms) / len(randomization_times_ms)
    avg_cap = sum(capture_times_ms) / len(capture_times_ms)
    print(f"[SDG] Avg randomization: {avg_rand:.1f} ms, avg capture: {avg_cap:.1f} ms, total: {total_ms:.1f} ms")

    writer.detach()
    rp.destroy()


# Run with different configurations to compare performance
run_example(wait_for_render=True, write_to_fabric=False)
run_example(wait_for_render=False, write_to_fabric=False)
run_example(wait_for_render=False, write_to_fabric=True)

# <start-sdg-getting-started-05-test>
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
    # BasicWriter rgb-only writes 1 png per capture, one output dir per configuration.
    configurations = [
        (True, False),
        (False, False),
        (False, True),
    ]
    for wait_for_render, write_to_fabric in configurations:
        out_dir = os.path.join(os.getcwd(), f"_out_fabric_{write_to_fabric}_wait_{wait_for_render}")
        if not validate_folder_contents(
            path=out_dir,
            recursive=True,
            expected_counts={"png": NUM_CAPTURES},
            fail_on_empty_files=True,
        ):
            print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
            sys.exit(1)
        print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-sdg-getting-started-05-test>

simulation_app.close()
