# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""
================================================================================
This file contains code snippets that are displayed in the Newton Actuators
"Authoring and Parsing Actuators from USD" tutorial.  Keep the
``<start-...-snippet>`` / ``<end-...-snippet>`` markers in sync with
``docs/isaacsim/newton_actuators_tutorials/newton_actuators_usd.rst``.
================================================================================

Runs end-to-end as a standalone script:

    ./python.sh standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_usd_example.py
"""

from __future__ import annotations

# ============================================================================
# 1. Launch Simulation App
# ============================================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import pathlib
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf

FRANKA_USD_REL_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"


async def setup_stage_with_franka() -> str:
    """Open the Franka USD asset directly as the active stage.

    Authoring goes into the in-memory stage; the source asset on Nucleus is
    never modified.  We later flatten everything to a temp ``.usda`` via
    :func:`export_stage` so the modified asset travels as a single file.

    Async so the kit app stays responsive while the Franka USD is fetched
    (especially from Nucleus).  Drive it via ``simulation_app.run_coroutine``.

    Returns:
        Path of the articulation root prim (the asset's default prim).
    """
    assets_root_path = await get_assets_root_path_async()
    franka_url = f"{assets_root_path}/{FRANKA_USD_REL_PATH}"
    success, stage = await stage_utils.open_stage_async(franka_url)
    assert success, f"Failed to open Franka asset at {franka_url}"

    # The asset doesn't ship with a PhysicsScene; add one so the stage can be
    # played after re-opening the exported file.
    stage_utils.define_prim("/PhysicsScene", "PhysicsScene")

    await omni.kit.app.get_app().next_update_async()

    default_prim = stage.GetDefaultPrim()
    assert default_prim and default_prim.IsValid(), "franka.usd must have a default prim"
    return default_prim.GetPath().pathString


# ============================================================================
# 2. Authoring actuators with `add_actuator`
# ============================================================================
def author_actuators_on_franka(franka_path: str) -> None:
    """Author one PD actuator with effort clamping on each Franka arm joint.

    ``franka_path`` is the articulation root prim path (e.g. ``"/panda"`` when
    the Franka USD has been opened as the active stage and its default prim
    is ``/panda``).
    """
    # <start-author-actuators-snippet>
    from isaacsim.core.experimental.actuators import (
        MaxEffortClampingConfig,
        PDControlConfig,
        add_actuator,
    )

    # Per-joint kp, kd, and effort limits
    JOINT_PARAMS = {
        "panda_joint1": dict(kp=67.0, kd=8.0, max_effort=1000.0),
        "panda_joint2": dict(kp=66.0, kd=8.0, max_effort=1000.0),
        "panda_joint3": dict(kp=65.0, kd=8.0, max_effort=1000.0),
        "panda_joint4": dict(kp=64.0, kd=8.0, max_effort=1000.0),
        "panda_joint5": dict(kp=63.0, kd=8.0, max_effort=1000.0),
        "panda_joint6": dict(kp=62.0, kd=8.0, max_effort=1000.0),
        "panda_joint7": dict(kp=61.0, kd=8.0, max_effort=1000.0),
    }

    for joint_name, p in JOINT_PARAMS.items():
        add_actuator(
            franka_path,
            target_names=joint_name,
            name=f"{joint_name}_actuator",
            controller=PDControlConfig(kp=p["kp"], kd=p["kd"]),
            clamping=[MaxEffortClampingConfig(max_effort=p["max_effort"])],
        )
    # <end-author-actuators-snippet>


# ============================================================================
# 3. Saving the stage so the actuators travel with the asset
# ============================================================================
def export_stage(out_path: pathlib.Path) -> None:
    """Flatten the current stage to a single USD file at `out_path`."""
    # <start-export-stage-snippet>
    stage = stage_utils.get_current_stage(backend="usd")
    stage.Export(out_path.as_posix())
    # <end-export-stage-snippet>


def print_authored_actuators_usda(franka_path: str) -> None:
    """Print the USDA text for the authored ``{franka_path}/Actuators`` subtree."""
    stage = stage_utils.get_current_stage(backend="usd")
    src_layer = stage.GetRootLayer()
    actuators_path = Sdf.Path(f"{franka_path}/Actuators")

    # Copy just the Actuators subtree into an anonymous USDA layer and dump it.
    out_layer = Sdf.Layer.CreateAnonymous(".usda")
    Sdf.CreatePrimInLayer(out_layer, actuators_path)
    Sdf.CopySpec(src_layer, actuators_path, out_layer, actuators_path)

    print("\n=== Authored Newton actuator prims (USDA) ===")
    print(out_layer.ExportToString())


# ============================================================================
# 4. Discovering authored actuators at runtime
# ============================================================================
def discover_actuators_from_usd(franka_path: str) -> ArticulationActuators:
    """Construct `ArticulationActuators` from every `NewtonActuator` prim under ``franka_path``.

    Returns:
        The `ArticulationActuators` instance built from the USD-authored actuators.
    """
    # <start-discover-from-usd-snippet>
    from isaacsim.core.experimental.actuators import ArticulationActuators

    actuated = ArticulationActuators(franka_path)
    print(f"Discovered {len(actuated.actuators)} actuators")
    print(f"Actuated DOF indices: {actuated.actuated_dof_indices}")
    # <end-discover-from-usd-snippet>
    return actuated


# ============================================================================
# 5. Round-trip: author → save → re-open → discover
# ============================================================================
def round_trip_demo(out_path: pathlib.Path) -> None:
    """Run an end-to-end demo: author actuators, save, re-open, and discover."""
    # Asset loading is async so the app stays responsive during Nucleus fetch.
    franka_path = simulation_app.run_coroutine(setup_stage_with_franka())
    author_actuators_on_franka(franka_path)
    print_authored_actuators_usda(franka_path)
    export_stage(out_path)

    # Re-open the exported stage to confirm the schema travels with the file.
    # <start-reopen-and-discover-snippet>
    success, stage = stage_utils.open_stage(out_path.as_posix())
    assert success, f"Failed to open exported stage at {out_path}"

    # The articulation root is the asset's default prim, preserved by Export.
    franka_path = stage.GetDefaultPrim().GetPath().pathString

    from isaacsim.core.experimental.actuators import ArticulationActuators

    # Use ArticulationActuators as a context manager so its SimulationManager
    # callbacks are deregistered deterministically on exit.
    with ArticulationActuators(franka_path) as actuated:
        # For sim stability, and simulation of the reflected inertia (armature) of the actuator.
        actuated.articulation.set_dof_armatures(0.1)
        try:
            assert len(actuated.actuators) == 7, "Expected 7 actuators on the Franka arm"

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            simulation_app.update()
            for _ in range(120):
                simulation_app.update()
        finally:
            omni.timeline.get_timeline_interface().stop()
    # <end-reopen-and-discover-snippet>


def main() -> None:
    SimulationManager.set_physics_dt(1.0 / 60.0)
    # Use the platform-specific temp directory so this works on Linux and Windows.
    out_path = pathlib.Path(tempfile.gettempdir()) / "franka_with_actuators.usda"
    round_trip_demo(out_path)
    print(f"Exported stage with authored Newton actuators to {out_path}")


if __name__ == "__main__":
    main()
    simulation_app.close()
