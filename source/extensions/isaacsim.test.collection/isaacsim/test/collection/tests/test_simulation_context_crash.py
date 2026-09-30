# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for simulation context crash scenarios to ensure timeline operations don't cause crashes."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from pxr import UsdPhysics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestSimulationContextCrash(omni.kit.test.AsyncTestCase):
    """Tests for simulation context crash scenarios."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with new stage."""
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        app_utils.stop(commit=False)
        await app_utils.update_app_async()

    async def test_simulation_context_crash(self) -> None:
        """Test that stopping timeline after articulation creation does not crash."""
        stage = stage_utils.get_current_stage()
        robot_prim_path = "/World/Robot"
        base_prim_path = f"{robot_prim_path}/base"
        link_prim_path = f"{robot_prim_path}/link"

        stage_utils.define_prim(robot_prim_path)
        links = Cube([base_prim_path, link_prim_path])
        RigidPrim(links.paths)
        GeomPrim(links.paths, apply_collision_apis=True)

        root_joint = UsdPhysics.FixedJoint.Define(stage, f"{robot_prim_path}/root_joint")
        root_joint.CreateBody1Rel().SetTargets([links.prims[0].GetPath()])
        UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())

        link_joint = UsdPhysics.RevoluteJoint.Define(stage, f"{robot_prim_path}/link_joint")
        link_joint.CreateBody0Rel().SetTargets([links.prims[0].GetPath()])
        link_joint.CreateBody1Rel().SetTargets([links.prims[1].GetPath()])

        # Start Simulation and wait
        app_utils.play(commit=False)
        await app_utils.update_app_async()

        # Create Articulation while timeline is playing
        self._robot = Articulation(robot_prim_path)
        await app_utils.update_app_async()

        # Stop the timeline to mimic the old World initialization behavior
        app_utils.stop(commit=False)
        await app_utils.update_app_async()

        self.assertFalse(app_utils.is_playing())

        # Make sure this call doesn't crash due to invalid physx handles
        # Use experimental API to disable gravity on the articulation links
        self._robot.set_link_enabled_gravities(False)
