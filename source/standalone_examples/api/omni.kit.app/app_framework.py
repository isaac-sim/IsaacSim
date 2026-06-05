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

"""Demonstrate AppFramework usage for running asynchronous Kit applications."""

import asyncio
import os

from isaacsim import AppFramework

argv = [
    "--empty",
    "--ext-folder",
    f'{os.path.abspath(os.environ["ISAAC_PATH"])}/exts',
    "--no-window",
    "--/app/asyncRendering=False",
    "--/app/fastShutdown=True",
    "--enable",
    "omni.usd",
    "--enable",
    "omni.kit.uiapp",
    "--enable",
    "omni.hydra.usdrt_delegate",
]
# startup
app = AppFramework("test_app", argv)

import omni.usd

stage_task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())

while not stage_task.done():
    app.update()

print("exiting")
app.close()
