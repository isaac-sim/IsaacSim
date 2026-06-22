# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for route_chase_through_ppisp: repoint an authored RenderProduct at the chase camera."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import omni.usd
from isaacsim.replicator.experimental.mobility_gen import route_chase_through_ppisp
from pxr import UsdGeom


class TestRouteChaseThroughPPISP(omni.kit.test.AsyncTestCase):
    """Route the chase camera through an export's authored PPISP RenderProduct."""

    async def setUp(self) -> None:
        """Create an empty stage with a chase camera."""
        await stage_utils.create_new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._stage.DefinePrim("/World", "Xform")
        UsdGeom.Camera.Define(self._stage, "/World/chase")

    async def tearDown(self) -> None:
        """Nothing to release."""

    def _add_render_product(self, rp_path: str, camera_path: str) -> None:
        """Author a typed RenderProduct under /Render targeting `camera_path`.

        Args:
            rp_path: Path at which to author the RenderProduct.
            camera_path: Camera path the RenderProduct targets.
        """
        self._stage.DefinePrim("/Render", "Scope")
        rp = self._stage.DefinePrim(rp_path, "RenderProduct")
        rp.CreateRelationship("camera").SetTargets([camera_path])

    async def test_repoints_render_product_to_chase_camera(self) -> None:
        """The authored RenderProduct's camera is repointed at the chase camera, with identity exposure."""
        UsdGeom.Camera.Define(self._stage, "/World/authored_cam")
        self._add_render_product("/Render/rp", "/World/authored_cam")

        result = route_chase_through_ppisp(self._stage, "/World/chase")

        # The RenderProduct camera is repointed regardless of whether a viewport was available to bind.
        targets = self._stage.GetPrimAtPath("/Render/rp").GetRelationship("camera").GetTargets()
        self.assertEqual([str(t) for t in targets], ["/World/chase"])
        # Identity exposure is forced on the chase camera (PPISP is the sole exposure authority).
        chase = self._stage.GetPrimAtPath("/World/chase")
        self.assertFalse(chase.GetAttribute("omni:rtx:autoExposure:enabled").Get())
        self.assertIn("OmniRtxCameraExposureAPI_1", chase.GetAppliedSchemas())
        # When a viewport is bound the RP path is returned; headless (no viewport) returns None.
        self.assertIn(result, (None, "/Render/rp"))

    async def test_no_render_product_returns_none(self) -> None:
        """With no authored RenderProduct, routing is a no-op returning None."""
        result = route_chase_through_ppisp(self._stage, "/World/chase")
        self.assertIsNone(result)
