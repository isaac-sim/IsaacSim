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

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

kit = SimulationApp()

import carb

for _ in range(10):
    kit.update()

# get the current output path and check if the file exists
pvd_output_dir = carb.settings.get_settings().get_as_string("/persistent/physics/omniPvdOvdRecordingDirectory")

print("omniPvdOvdRecordingDirectory: ", pvd_output_dir)
my_file = Path(os.path.join(pvd_output_dir, "tmp.ovd"))
if not my_file.is_file():
    print(f"[fatal] {my_file} does not exist")
    sys.exit(1)

kit.close()
