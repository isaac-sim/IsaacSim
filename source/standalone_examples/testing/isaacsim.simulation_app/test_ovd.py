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

"""Test OmniPVD OVD recording file generation."""

import sys
import tempfile
from pathlib import Path

from isaacsim import SimulationApp

if not any(argument.startswith("--ovd") for argument in sys.argv[1:]):
    pvd_output_dir = Path(tempfile.mkdtemp(prefix="isaacsim_ovd_"))
    sys.argv.append(f"--ovd={pvd_output_dir}")

kit = SimulationApp()

import carb  # noqa: E402

for _ in range(10):
    kit.update()

# get the current output path and check if the file exists
pvd_output_dir = carb.settings.get_settings().get_as_string("/persistent/physics/omniPvdOvdRecordingDirectory")

print("omniPvdOvdRecordingDirectory: ", pvd_output_dir)
my_file = Path(pvd_output_dir) / "tmp.ovd"
recording_exists = my_file.is_file()
if not recording_exists:
    print(f"[fatal] {my_file} does not exist")

exit_code = 0 if recording_exists else 1
kit.close(exit_code=exit_code)
sys.exit(exit_code)
