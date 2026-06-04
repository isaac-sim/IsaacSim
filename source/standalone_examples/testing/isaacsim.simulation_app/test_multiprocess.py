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

"""Test running Isaac Sim simulation in a separate process."""

import multiprocessing
import sys


def run_simulation():
    """Run the Isaac Sim simulation in a separate process.

    This function contains the main simulation logic that will be executed
    in a separate process using multiprocessing.
    """
    from isaacsim import SimulationApp

    simulation_app = SimulationApp()

    import isaacsim.core.experimental.utils.app as app_utils
    import omni.kit.app
    from isaacsim.core.experimental.objects import GroundPlane

    GroundPlane("/World/ground_plane")
    app_utils.play()

    frame_idx = 0
    while simulation_app.is_running():
        if app_utils.is_playing():
            simulation_app.update()
        else:
            simulation_app.update()
            # we should exit this loop before we hit frame 200 unless we are stuck on an exit screen
            assert frame_idx < 200
        # try exiting, it should exit unless a save file dialog shows up.
        if frame_idx == 100:
            omni.kit.app.get_app().post_quit()
        frame_idx += 1

    simulation_app.close()


if __name__ == "__main__":
    # Create and start the simulation process
    simulation_process = multiprocessing.Process(target=run_simulation)
    simulation_process.start()

    # Wait for the process to complete
    simulation_process.join()

    # Check if the process completed successfully
    if simulation_process.exitcode == 0:
        print("Simulation completed successfully")
    else:
        print(f"Simulation process exited with code: {simulation_process.exitcode}")
        sys.exit(simulation_process.exitcode)
