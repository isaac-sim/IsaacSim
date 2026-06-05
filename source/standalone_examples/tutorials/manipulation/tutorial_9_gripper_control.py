# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tutorial 9, Part 1: Gripper Control

Introduces the Articulation API by controlling the 2F-140 gripper on a UR10e
robot. The gripper cycles between open and closed using set_dof_position_targets.
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

if args.headless:
    from isaacsim.core.experimental.utils.app import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path_async

_OPEN_POS: float = 0.0
_CLOSED_POS: float = 0.7
_HOLD_STEPS: int = 120

# ========================================================


async def setup_scene() -> Articulation:
    assets_root_path = await get_assets_root_path_async()
    stage_utils.add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd",
        path="/World/ur",
    )

    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(1000)

    await omni.kit.app.get_app().next_update_async()
    set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.5], camera_prim_path="/OmniverseKit_Persp")
    robot = Articulation("/World/ur")
    await omni.kit.app.get_app().next_update_async()
    return robot


# ========================================================


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    SimulationManager.setup_simulation(dt=1.0 / 60.0)

    robot = app.run_coroutine(setup_scene())
    app.update()

    if args.headless:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
        while app.is_running() and not app_utils.is_playing():
            app.update()
    else:
        app_utils.play()
        app.update()

    print(f"Robot DOFs: {robot.dof_names}")

    # <start-gripper-control-snippet>
    finger_idx = robot.dof_names.index("finger_joint")
    frame_count = 0

    while app.is_running():
        for target_pos, label in [(_CLOSED_POS, "closing"), (_OPEN_POS, "opening")]:
            print(f"Gripper {label}...")
            for _ in range(_HOLD_STEPS):
                robot.set_dof_position_targets(target_pos, dof_indices=finger_idx)
                app.update()
                frame_count += 1
                if args.test and frame_count >= _HOLD_STEPS * 2:
                    return
    # <end-gripper-control-snippet>


if __name__ == "__main__":
    try:
        main(args, simulation_app)
    except Exception:
        import traceback

        traceback.print_exc()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()
