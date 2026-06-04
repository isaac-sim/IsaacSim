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

"""Capture a viewport screenshot and save it to disk.

Injected globals (via isaacsim_send.py --arg):
    output_path: str — File path for the output PNG (default: /tmp/viewport_capture.png)
"""

# Defaults (overridden by injected globals)
if "output_path" not in dir():
    output_path = "/tmp/viewport_capture.png"  # noqa: F841


async def _capture():
    from isaacsim.test.utils.image_capture import capture_viewport_screenshot_async

    await capture_viewport_screenshot_async(output_path)


await _capture()
