# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import sys

if sys.platform == "win32":
    pass
else:

    this_dir = os.path.dirname(os.path.realpath(__file__))

    icon_path = os.path.join(this_dir, "omni.isaac.sim.png")
    exec_path = os.path.normpath(os.path.join(this_dir, "..", "..", "isaac-sim.sh"))
    desktop_file_path = os.path.expanduser("~/.local/share/applications/IsaacSim.desktop")

    # write the .desktop file to the user's applications folder
    # (will appear under ubuntu logo)
    if os.path.exists(os.path.dirname(desktop_file_path)):
        with open(desktop_file_path, "w") as file:
            print(f"Writing Isaac Sim icon file to {desktop_file_path}")
            file.write(f"""\
[Desktop Entry]
Version=1.0
Name=Isaac Sim
Icon={icon_path}
Terminal=false
Type=Application
StartupWMClass=IsaacSim
Exec={exec_path}\
""")

    # set trusted flag on the .desktop file via shell:
    os.system(f'gio set "{desktop_file_path}" metadata::trusted true')

