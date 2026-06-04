# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test to verify that the app starts without rendering."""

from isaacsim import SimulationApp

# Extensions required for physics-only simulation
required_extensions = [
    "omni.physx",
    "omni.physx.tensors",
    "omni.physx.fabric",
    "omni.warp.core",
    "usdrt.scenegraph",
    "omni.kit.telemetry",
    "omni.kit.loop",
    "omni.kit.usd.mdl",
    "omni.usd.metrics.assembler.ui",
    "omni.hydra.usdrt_delegate",
]

# Create extra_args list using a loop
extra_args = [f"--/app/exts/folders=['apps','exts','extscache','extsUser','extsDeprecated']"]
for extension in required_extensions:
    extra_args.extend(["--enable", extension])

# Create simulation app with minimal set of extension to do physics only simulation
kit = SimulationApp(
    {
        "headless": True,
        "extra_args": extra_args,
    },
    experience=None,
)

# Cleanup
kit.close()
