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

import argparse
import os
import sys
import traceback

from isaacsim import SimulationApp

BASE_EXP_PATH = os.path.join(os.environ["EXP_PATH"], "isaacsim.exp.action_and_event_data_generation.base.kit")
APP_CONFIG = {"headless": True, "width": 1920, "height": 1080}


class ActorSDG:
    def __init__(self, sim_app, config_file_path):
        self._sim_app = sim_app
        # Inputs
        self.config_file_path = config_file_path
        self._sim_manager = None

    async def run(self):
        # Set up global settings
        self._set_simulation_settings()
        await self._sim_app.app.next_update_async()

        from isaacsim.replicator.agent.core import api as IRA

        can_load_config = IRA.load_config_file(self.config_file_path)
        if not can_load_config:
            print(f"Failed to load config file at {self.config_file_path}. Exit.")
            return False

        print(f"config file loaded: {IRA.get_config_file()}")

        await IRA.setup_simulation()
        await IRA.start_data_generation_async()
        return True

    def _set_simulation_settings(self):
        import carb
        import omni.replicator.core as rep

        rep.settings.carb_settings("/omni/replicator/backend/writeThreads", 16)
        self._settings = carb.settings.get_settings()
        self._settings.set("/app/scripting/ignoreWarningDialog", True)
        self._settings.set("/app/omni.graph.scriptnode/enable_opt_in", False)  # To bypass action graph scriptnode check
        self._settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)  # Needed for the DH characters

        # Logging and debug print
        self._settings.set("/log/level", "info")
        self._settings.set("/log/channels/omni.replicator.core", "info")
        self._settings.set("/log/channels/omni.behavior.composer.core", "info")
        self._settings.set("/log/channels/isaacsim.anim.robot", "info")
        self._settings.set("/log/channels/isaacsim.sensors.rtx.placement", "info")
        self._settings.set("/log/channels/isaacsim.replicator.agent.core", "info")


def get_args():
    parser = argparse.ArgumentParser("Actor SDG")
    parser.add_argument("-c", "--config_file", required=True, help="Path to a IRA config file")
    args, _ = parser.parse_known_args()
    return args


def main():
    # Read command line arguments
    args = get_args()
    config_file_path = args.config_file
    print("Config file path: {}".format(config_file_path))

    # Check files exist
    if not os.path.isfile(config_file_path):
        print("Invalid config file path. Exit.", file=sys.stderr)
        return

    # Start SimApp
    sim_app = SimulationApp(launch_config=APP_CONFIG, experience=BASE_EXP_PATH)

    # Start SDG
    sdg = ActorSDG(
        sim_app,
        os.path.abspath(config_file_path),
    )

    from omni.kit.async_engine import run_coroutine

    task = run_coroutine(sdg.run())
    try:
        while not task.done():
            sim_app.update()

        exc = task.exception()
        if exc:
            traceback.print_exception(exc, file=sys.stderr)
        elif not task.result():
            print("Failed to run SDG")

    finally:
        sim_app.close()
        print("SDG finished, exiting...")


if __name__ == "__main__":
    main()
