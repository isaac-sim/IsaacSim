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

"""Verify ROS 2 helpers handle unavailable platform and render-product data."""

from unittest.mock import patch

import omni.kit.test
from isaacsim.ros2.core.impl.collect_namespace import collect_namespace
from isaacsim.ros2.core.impl.ros2_common import get_ubuntu_version


class TestPythonUtils(omni.kit.test.AsyncTestCase):
    """Exercise pure-Python ROS 2 helper failure paths."""

    async def test_collect_namespace_with_stale_render_product(self) -> None:
        """Return an empty namespace when a render product no longer resolves to a camera."""
        self.assertEqual(collect_namespace("", "/Definitely/Stale/RenderProduct"), "")

    async def test_get_ubuntu_version_without_version_id(self) -> None:
        """Return None when the OS release metadata omits VERSION_ID."""
        with (
            patch("isaacsim.ros2.core.impl.ros2_common.sys.platform", "linux"),
            patch(
                "isaacsim.ros2.core.impl.ros2_common.platform.freedesktop_os_release",
                return_value={"NAME": "Ubuntu"},
            ),
        ):
            self.assertIsNone(get_ubuntu_version())
