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

"""Test suite for robot motion planning USD schema registration and application."""

import omni.kit.test
import omni.usd
from isaacsim.robot_motion.schema import (
    MOTION_PLANNING_API_NAME,
    MOTION_PLANNING_ENABLED_ATTR,
    apply_motion_planning_api,
)
from pxr import Plug, Sdf, Usd


class CodelessTests(omni.kit.test.AsyncTestCase):
    """Test robot motion planning codeless schema registration and application."""

    async def _new_stage(self):
        await omni.usd.get_context().new_stage_async()
        return omni.usd.get_context().get_stage()

    async def test_api_schema_registration(self):
        """Test that the motion planning API schema is correctly registered."""
        codeless_plugin = Plug.Registry().GetPluginWithName("RobotMotionSchema")
        self.assertIsNotNone(codeless_plugin)

        registry = Usd.SchemaRegistry()
        api_def = registry.FindAppliedAPIPrimDefinition(MOTION_PLANNING_API_NAME)

        self.assertTrue(api_def)
        self.assertTrue(registry.IsAppliedAPISchema(MOTION_PLANNING_API_NAME))
        self.assertFalse(registry.IsConcrete(MOTION_PLANNING_API_NAME))

    async def test_apply_api_on_prim(self):
        """Test applying the motion planning API schema on a prim."""
        stage = await self._new_stage()
        self.assertIsNotNone(stage)

        prim = stage.DefinePrim("/World/Robot", "Xform")
        self.assertTrue(prim)

        prim.AddAppliedSchema(MOTION_PLANNING_API_NAME)
        attr = prim.GetAttribute(MOTION_PLANNING_ENABLED_ATTR)
        if not attr:
            attr = prim.CreateAttribute(MOTION_PLANNING_ENABLED_ATTR, Sdf.ValueTypeNames.Bool, True)
        attr.Set(True)

        self.assertTrue(prim.HasAPI(MOTION_PLANNING_API_NAME))
        self.assertTrue(prim.GetAttribute(MOTION_PLANNING_ENABLED_ATTR).Get())

    async def test_apply_helper_sets_attribute(self):
        """Test that the apply helper correctly set the enabled attribute."""
        stage = await self._new_stage()
        self.assertIsNotNone(stage)

        prim = stage.DefinePrim("/World/RobotHelper", "Xform")
        apply_motion_planning_api(prim, enabled=False)

        self.assertTrue(prim.HasAPI(MOTION_PLANNING_API_NAME))
        self.assertFalse(prim.GetAttribute(MOTION_PLANNING_ENABLED_ATTR).Get())

    async def test_read_applied_schemas(self):
        """Test reading applied schemas from a prim with the motion planning API."""
        stage = await self._new_stage()
        self.assertIsNotNone(stage)

        prim = stage.DefinePrim("/World/ReadSchemas", "Xform")
        prim.AddAppliedSchema(MOTION_PLANNING_API_NAME)

        applied_schemas = prim.GetAppliedSchemas()
        self.assertIn(MOTION_PLANNING_API_NAME, applied_schemas)
