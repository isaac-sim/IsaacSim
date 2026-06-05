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

"""Control simulation playback: play, pause, stop, step, or query status.

Uses isaacsim.core.experimental.utils.app and isaacsim.core.simulation_manager.
Works in both windowed and --no-window headless modes.

Injected globals (via isaacsim_send.py --arg):
    action: str — "status" (default), "play", "pause", "stop", or "step".
    num_steps: int — Number of physics steps for "step" action (default: 1).
    dt: float — Physics timestep in seconds for setup (default: None, uses current).
"""

if "action" not in dir():
    action = "status"
if "num_steps" not in dir():
    num_steps = 1
if "dt" not in dir():
    dt = None

import isaacsim.core.experimental.utils.app as app_utils

if action == "status":
    playing = app_utils.is_playing()
    paused = app_utils.is_paused()
    stopped = app_utils.is_stopped()

    if playing and not paused:
        state = "PLAYING"
    elif paused:
        state = "PAUSED"
    else:
        state = "STOPPED"

    print(f"Simulation state: {state}")

    try:
        from isaacsim.core.simulation_manager import SimulationManager

        sm = SimulationManager
        print(f"Physics DT: {sm.get_physics_dt()}")
        print(f"Simulation time: {sm.get_simulation_time()}")
        print(f"Physics steps: {sm.get_num_physics_steps()}")
        print(f"Is simulating: {sm.is_simulating()}")
        print(f"Backend: {sm.get_backend()}")
        print(f"Device: {sm.get_device()}")
        scenes = sm.get_physics_scenes()
        if scenes:
            print(f"Physics scenes: {scenes}")
    except Exception as e:
        print(f"SimulationManager info: {e}")

elif action == "play":
    if dt is not None:
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.setup_simulation(dt=float(dt))
        print(f"Physics DT set to {dt}")

    app_utils.play()
    app_utils.update_app(steps=5)
    print("Simulation: PLAYING")

elif action == "pause":
    app_utils.pause()
    app_utils.update_app(steps=2)
    print("Simulation: PAUSED")

elif action == "stop":
    app_utils.stop()
    app_utils.update_app(steps=5)
    print("Simulation: STOPPED")

elif action == "step":
    was_playing = app_utils.is_playing()
    if not was_playing:
        app_utils.play()
        app_utils.update_app(steps=2)

    steps = int(num_steps)
    app_utils.update_app(steps=steps)
    print(f"Advanced {steps} simulation step(s)")

    try:
        from isaacsim.core.simulation_manager import SimulationManager

        print(f"Simulation time: {SimulationManager.get_simulation_time()}")
        print(f"Physics steps: {SimulationManager.get_num_physics_steps()}")
    except Exception:
        pass

else:
    print(f"ERROR: Unknown action '{action}'. Use: status, play, pause, stop, step")
