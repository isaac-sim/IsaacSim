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

"""Demonstrate simulation-driven SDG with physics-based capture triggers."""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb.settings
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim
from pxr import UsdGeom


def run_example() -> None:
    """Run physics simulation and capture data at height-based intervals."""
    # Create a new stage and disable capture on play
    omni.usd.get_context().new_stage()
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Add a light
    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")

    # Create a cube with colliders and rigid body dynamics at a specific location
    cube = rep.functional.create.cube(name="Cube", parent="/World")
    rep.functional.modify.position(cube, (0, 0, 2))
    rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")
    rep.functional.physics.apply_rigid_body(cube, with_collider=True)

    # Createa a sphere with colliders and rigid body dynamics next to the cube
    sphere = rep.functional.create.sphere(name="Sphere", parent="/World")
    rep.functional.modify.position(sphere, (-1, -1, 2))
    rep.functional.modify.semantics(sphere, {"class": "my_sphere"}, mode="add")
    rep.functional.physics.apply_rigid_body(sphere, with_collider=True)

    # Create a render product using the viewport perspective camera
    cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
    rp = rep.create.render_product(cam, (512, 512))

    # Write data using the basic writer with the rgb and bounding box annotators
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), "_out_basic_writer_sim")
    backend.initialize(output_dir=out_dir)
    print(f"Output directory: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(backend=backend, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True)
    writer.attach(rp)

    # Start the timeline (will only advance with app update)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Wrap the cube with as a RigidPrim for easy access to its world poses and velocities
    cube_rigid = RigidPrim(str(cube.GetPrimPath()))

    # Wrap the cube as an Imageable object to toggle visibility during capture
    cube_imageable = UsdGeom.Imageable(cube)

    # Define the capture interval in meters
    capture_interval_meters = 0.5
    cube_pos = cube_rigid.get_world_poses(indices=[0])[0].numpy()
    previous_capture_height = cube_pos[0, 2]

    # Update the app which will advance the timeline (and implicitly the simulation)
    for i in range(100):
        simulation_app.update()
        cube_pos = cube_rigid.get_world_poses(indices=[0])[0].numpy()
        current_height = cube_pos[0, 2]
        distance_dropped = previous_capture_height - current_height
        print(f"Step {i}; cube height: {current_height:.3f}; drop since last capture: {distance_dropped:.3f}")

        # Stop the simulation if the cube falls below the ground
        if current_height < 0:
            print(f"\t Cube fell below the ground at height {current_height:.3f}, stopping simulation..")
            break

        # Capture every time the cube drops by the threshold distance
        if distance_dropped >= capture_interval_meters:
            print(f"\t Capturing at height {current_height:.3f}")
            previous_capture_height = current_height

            # Setting delta_time to 0.0 will make sure the timeline is not advanced during capture
            rep.orchestrator.step(delta_time=0.0)

            # Capture again with the cube hidden
            print("\t Capturing with cube hidden")
            cube_imageable.MakeInvisible()
            rep.orchestrator.step(delta_time=0.0)
            cube_imageable.MakeVisible()

            # Resume the timeline to continue the simulation
            timeline.play()

    # Pause the simulation
    timeline.pause()

    # Wait for the data to be written to disk and clean up resources
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


# Run the example
run_example()

# <start-sdg-getting-started-04-test>
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
    # The cube falls ~2 m and is captured every 0.5 m drop with 2 captures per trigger
    # (visible + hidden), yielding 6 captures. BasicWriter writes 1 rgb png + 1 colorized
    # semantic_segmentation png + 1 labels json per capture, plus a single metadata.txt.
    num_captures = 6
    out_dir = os.path.join(os.getcwd(), "_out_basic_writer_sim")
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": num_captures * 2, "json": num_captures, "txt": 1},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-sdg-getting-started-04-test>

simulation_app.close()
