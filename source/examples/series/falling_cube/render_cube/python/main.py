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

"""Render every simulated frame of a cube falling onto a ground plane."""

from __future__ import annotations

import argparse
import ctypes
import struct
import tempfile
import zlib
from pathlib import Path

import numpy as np

_CAMERA_PATH = "/Environment/Camera"
_LIGHT_PATH = "/Environment/Light"
_RENDER_PRODUCT_PATH = "/Render/Product"
_RENDER_VAR_NAME = "LdrColor"
_RENDER_VAR_PATH = f"{_RENDER_PRODUCT_PATH}/{_RENDER_VAR_NAME}"
# Python exposes the schema object APIs used by the scene but not generic attribute or
# relationship creation yet. Keep this boundary layer limited to the render product.
_RENDER_LAYER = """
def Scope "Render"
{
    def RenderProduct "Product"
    {
        int2 resolution = (480, 320)
        rel camera = </Environment/Camera>
        rel orderedVars = [<LdrColor>]

        def RenderVar "LdrColor"
        {
            token dataType = "color4f"
            string sourceName = "LdrColor"
        }
    }
}
"""
_TIME_STEP = 1.0 / 60.0
_CAPTURE_INTERVAL = 15
# The cube has reached the ground once it stops descending. The step budget only bounds
# the run if it never settles.
_MAX_SIMULATION_STEPS = 300
_REST_TRAVEL = 1.0e-3
_REST_STEPS = 3
# Population writes the imported scene at ordinal 1, and the OVStage backend
# commits it once the first pose is published. Every render therefore requires at least
# this publication, and observes the newer poses committed by later steps.
_POPULATION_ORDINAL = 1


class _AttachedOvstage:
    """Native OVStage handle in the form :meth:`ovrtx.Renderer.attach_ovstage` accepts.

    The renderer resolves an attachment through the ``_inst`` attribute of an OVStage
    object. A stage owns its OVStage instance and publishes the address as an integer, so
    this adapter presents that address the same way
    ``isaacsim.physics_engines.ovstage.get_native_handle`` consumes it.
    """

    def __init__(self, native_handle: int) -> None:
        """Wrap a native OVStage address.

        Args:
            native_handle: Address of a live OVStage instance.

        """
        self._inst = ctypes.c_void_p(native_handle)


def _author_stage() -> str:
    """Author the simulated and rendered scene and return its USDA representation.

    Returns:
        Flattened USDA representation of the scene.

    """
    from isaacsim.foundation.objects import Camera, Cube, DistantLight, PhysicsScene, Stage
    from isaacsim.foundation.prims import ColliderBody, GroundPlane, RigidBody

    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")
        GroundPlane("/World/Ground", sizes=10.0)
        Cube("/World/Cube", sizes=1.0).set_local_poses(translations=[0.0, 0.0, 2.0])

        PhysicsScene("/World/PhysicsScene").set_gravities([0.0, 0.0, -9.81])
        ColliderBody("/World/Cube")
        RigidBody("/World/Cube")

        light = DistantLight(_LIGHT_PATH)
        light.set_angles([1.0])
        light.set_intensities([4000.0])
        light.set_local_poses(orientations=[0.4545, 0.0801, 0.1540, 0.8733])

        camera = Camera(_CAMERA_PATH)
        # Camera lengths use scene units; 2.4 authors the USD focalLength value 24.
        camera.set_focal_lengths([2.4])
        camera.set_local_poses(
            translations=[7.0, -7.0, 5.0],
            orientations=[0.5166, 0.2140, 0.3171, 0.7658],
        )

        return stage.export_stage_to_string() + _RENDER_LAYER
    finally:
        stage.close_stage()


def _write_png(path: Path, pixels: np.ndarray) -> None:
    """Write an RGBA image to an uncompressed-filter PNG file.

    Args:
        path: Destination file.
        pixels: Image of shape ``(height, width, 4)`` and dtype ``uint8``.

    """
    height, width = pixels.shape[0], pixels.shape[1]
    scanlines = b"".join(b"\x00" + pixels[row].tobytes() for row in range(height))

    def chunk(tag: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(tag + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", checksum)

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(scanlines, 6))
        + chunk(b"IEND", b"")
    )


def _parse_arguments() -> argparse.Namespace:
    """Parse the command line.

    Returns:
        Parsed arguments.

    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Directory for the captured frames. Defaults to a new temporary directory.",
    )
    return parser.parse_args()


def main() -> None:
    """Simulate a falling cube and render each captured frame."""
    arguments = _parse_arguments()
    output_directory = arguments.output or Path(tempfile.mkdtemp(prefix="render_falling_cube_"))
    output_directory.mkdir(parents=True, exist_ok=True)

    import isaacsim.physics_engines.ovphysx  # noqa: F401
    import ovrtx
    from isaacsim.foundation.objects import Stage
    from isaacsim.physics.entities import RigidBodyEntity
    from isaacsim.physics.manager import PhysicsManager

    renderer = None
    ovstage_stage = None
    renderer_attached = False
    physics_manager = None
    try:
        # The renderer and the physics engine share one OVStage: the engine reads the
        # scene it simulates from the stage, and the renderer reads the poses that this
        # example publishes back onto it.
        renderer = ovrtx.Renderer()
        ovstage_stage = Stage("ovstage").import_stage_from_string(_author_stage(), make_default=False)
        renderer.attach_ovstage(_AttachedOvstage(ovstage_stage.get_stage_ptr()))
        renderer_attached = True

        physics_manager = PhysicsManager.get_instance()
        if not physics_manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX physics engine is unavailable.")
        physics_manager.setup(dt=_TIME_STEP)
        if not physics_manager.initialize(ovstage_stage.get_stage_ptr(), ovstage_stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        simulated_cube = RigidBodyEntity("ovphysx", "/World/Cube")

        def cube_height() -> float:
            """Return the current simulated cube height."""
            positions, _ = simulated_cube.get_world_poses()
            return float(positions.numpy()[0, 2])

        def capture_frame(step: int) -> int:
            """Render one frame, write it out, and report a checksum of its pixels."""
            products = renderer.step(
                render_products={_RENDER_PRODUCT_PATH},
                delta_time=_TIME_STEP,
                ordinal=_POPULATION_ORDINAL,
            )
            checksum = None
            for product in products.values():
                for frame in product.frames:
                    mapped_output = frame.render_vars[_RENDER_VAR_PATH].map(device=ovrtx.Device.CPU)
                    try:
                        pixels = np.from_dlpack(mapped_output)
                        _write_png(output_directory / f"frame_{step:04d}.png", pixels)
                        checksum = zlib.crc32(pixels)
                    finally:
                        mapped_output.unmap()
            if checksum is None:
                raise RuntimeError("The renderer produced no frame.")
            return checksum

        if not physics_manager.publish_transforms_to_stage():
            raise RuntimeError("Physics transform publication failed.")
        initial_height = cube_height()
        first_checksum = capture_frame(0)
        last_checksum = first_checksum
        captured_frames = 1
        height = initial_height
        resting_steps = 0
        step = 0
        while resting_steps < _REST_STEPS and step < _MAX_SIMULATION_STEPS:
            step += 1
            physics_manager.step()
            if not physics_manager.publish_transforms_to_stage():
                raise RuntimeError("Physics transform publication failed.")
            previous_height, height = height, cube_height()
            resting_steps = resting_steps + 1 if abs(height - previous_height) < _REST_TRAVEL else 0
            if step % _CAPTURE_INTERVAL == 0:
                last_checksum = capture_frame(step)
                captured_frames += 1
        # Capture where the cube settled, so the last frame always shows it landed.
        if step % _CAPTURE_INTERVAL != 0:
            last_checksum = capture_frame(step)
            captured_frames += 1

        print(f"Cube height: {initial_height:.3f} -> {height:.3f} over {step} steps.")
        print(f"Rendered {captured_frames} frames to {output_directory}.")
        if resting_steps < _REST_STEPS:
            raise RuntimeError("The cube did not reach the ground.")
        if height >= initial_height:
            raise RuntimeError("The cube did not fall.")
        print("The cube fell.")
        if first_checksum == last_checksum:
            raise RuntimeError("The render did not follow the simulation.")
        print("The render followed the simulation.")
    finally:
        if physics_manager is not None and physics_manager.is_initialized():
            physics_manager.invalidate()
        if renderer_attached:
            renderer.detach_ovstage()
        if renderer is not None:
            renderer.destroy()
        if ovstage_stage is not None:
            ovstage_stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
