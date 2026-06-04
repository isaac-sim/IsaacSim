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

"""Test remote scene manipulation via the Python server TCP socket.

These tests exercise ``isaacsim.core.experimental`` APIs by sending Python code
through the TCP socket server, simulating what an LLM or external automation
client would do when connected to a running Isaac Sim instance.
"""

from __future__ import annotations

import asyncio
import json
import textwrap

import carb
import omni.kit.test

from ._auth import add_auth_header

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_HOST = "127.0.0.1"


async def _send_and_receive(port: int, source: str) -> dict:
    """Send Python source to the server and return the parsed JSON response.

    Args:
        port: The TCP port to connect to.
        source: The Python source code to send.

    Returns:
        The parsed JSON response dictionary.
    """
    reader, writer = await asyncio.open_connection(_HOST, port)
    writer.write(add_auth_header(source).encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())


class TestRemoteScene(omni.kit.test.AsyncTestCase):
    """Test Isaac Sim scene manipulation via the Python server TCP socket.

    Each test sends Python code over TCP that uses ``isaacsim.core.experimental``
    APIs to create stages, prims, lights, rigid bodies, and control simulation,
    then verifies the results through the JSON response.
    """

    async def setUp(self) -> None:
        """Set up the TCP connection port and import utility modules."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")
        await self._exec(textwrap.dedent("""\
                import isaacsim.core.experimental.utils.stage as stage_utils
                import isaacsim.core.experimental.utils.app as app_utils
            """))

    async def tearDown(self) -> None:
        """Stop the simulation and close the stage."""
        await self._exec("app_utils.stop()")
        await self._exec("stage_utils.close_stage()")

    async def _exec(self, source: str) -> dict:
        """Send code, assert ``status == "ok"``, and return the full response.

        Args:
            source: Python source code to execute.

        Returns:
            The parsed JSON response dictionary.
        """
        data = await _send_and_receive(self._port, source)
        self.assertEqual("ok", data.get("status"), f"Execution failed: {data}")
        return data

    async def _eval(self, source: str) -> object:
        """Send an expression, assert success, and return ``result``.

        Args:
            source: Python expression to evaluate.

        Returns:
            The ``result`` field from the JSON response.
        """
        data = await self._exec(source)
        self.assertIn("result", data, f"No result in response: {data}")
        return data["result"]

    # ------------------------------------------------------------------
    # Stage management
    # ------------------------------------------------------------------

    async def test_create_new_stage(self) -> None:
        """Verify that a new empty stage can be created."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        is_valid = await self._eval("stage_utils.get_current_stage() is not None")
        self.assertTrue(is_valid)

    # ------------------------------------------------------------------
    # Prim creation
    # ------------------------------------------------------------------

    async def test_define_prims(self) -> None:
        """Verify that multiple prims can be defined on a new stage."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec(textwrap.dedent("""\
                stage_utils.define_prim("/World", "Xform")
                stage_utils.define_prim("/World/MyCube", "Cube")
                stage_utils.define_prim("/World/MySphere", "Sphere")
                stage_utils.define_prim("/World/MyCylinder", "Cylinder")
            """))

        for path in ["/World", "/World/MyCube", "/World/MySphere", "/World/MyCylinder"]:
            exists = await self._eval(f"stage_utils.get_current_stage().GetPrimAtPath('{path}').IsValid()")
            self.assertTrue(exists, f"Prim {path} should exist")

    async def test_create_shapes(self) -> None:
        """Verify that shape objects can be created via remote execution."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec("stage_utils.define_prim('/World', 'Xform')")
        await self._exec(textwrap.dedent("""\
                from isaacsim.core.experimental.objects import Cube, Sphere, Capsule
                Cube("/World/BoxA", sizes=0.5)
                Sphere("/World/Ball", radii=0.25)
                Capsule("/World/Pill", radii=0.1, heights=0.5)
            """))

        for path in ["/World/BoxA", "/World/Ball", "/World/Pill"]:
            exists = await self._eval(f"stage_utils.get_current_stage().GetPrimAtPath('{path}').IsValid()")
            self.assertTrue(exists, f"Shape prim {path} should exist")

    async def test_create_lights(self) -> None:
        """Verify that light objects can be created via remote execution."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec("stage_utils.define_prim('/World', 'Xform')")
        await self._exec(textwrap.dedent("""\
                from isaacsim.core.experimental.objects import DomeLight, DistantLight, SphereLight
                DomeLight("/World/DomeLight")
                DistantLight("/World/DistantLight")
                SphereLight("/World/SphereLight", radii=0.5)
            """))

        for path in ["/World/DomeLight", "/World/DistantLight", "/World/SphereLight"]:
            exists = await self._eval(f"stage_utils.get_current_stage().GetPrimAtPath('{path}').IsValid()")
            self.assertTrue(exists, f"Light prim {path} should exist")

    async def test_create_rigid_body(self) -> None:
        """Verify that a rigid body with collision can be created."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec(textwrap.dedent("""\
                from pxr import UsdPhysics
                stage_utils.define_prim("/World", "Xform")
                cube_prim = stage_utils.define_prim("/World/RigidCube", "Cube")
                UsdPhysics.RigidBodyAPI.Apply(cube_prim)
                UsdPhysics.CollisionAPI.Apply(cube_prim)
            """))

        has_rigid = await self._eval(
            "stage_utils.get_current_stage().GetPrimAtPath('/World/RigidCube')" ".HasAPI(UsdPhysics.RigidBodyAPI)"
        )
        self.assertTrue(has_rigid, "Cube should have RigidBodyAPI applied")

        has_collision = await self._eval(
            "stage_utils.get_current_stage().GetPrimAtPath('/World/RigidCube')" ".HasAPI(UsdPhysics.CollisionAPI)"
        )
        self.assertTrue(has_collision, "Cube should have CollisionAPI applied")

    async def test_create_ground_plane(self) -> None:
        """Verify that a ground plane prim can be created."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec("stage_utils.define_prim('/World', 'Xform')")
        await self._exec(textwrap.dedent("""\
                from isaacsim.core.experimental.objects import GroundPlane
                GroundPlane("/World/GroundPlane")
            """))

        exists = await self._eval("stage_utils.get_current_stage().GetPrimAtPath('/World/GroundPlane').IsValid()")
        self.assertTrue(exists, "GroundPlane prim should exist")

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    async def test_simulation_play_stop(self) -> None:
        """Verify that the simulation can be played and stopped."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")

        await self._exec("app_utils.play()")
        is_playing = await self._eval("app_utils.is_playing()")
        self.assertTrue(is_playing, "Timeline should be playing after play()")

        await self._exec("app_utils.stop()")
        is_stopped = await self._eval("app_utils.is_stopped()")
        self.assertTrue(is_stopped, "Timeline should be stopped after stop()")

    async def test_simulation_step(self) -> None:
        """Verify that the simulation can be stepped forward."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec("app_utils.play()")
        await self._exec("app_utils.update_app(steps=10)")
        await self._exec("app_utils.stop()")

        is_stopped = await self._eval("app_utils.is_stopped()")
        self.assertTrue(is_stopped, "Timeline should be stopped after stepping and stop()")

    # ------------------------------------------------------------------
    # Prim deletion
    # ------------------------------------------------------------------

    async def test_delete_prim(self) -> None:
        """Verify that a prim can be deleted from the stage."""
        await self._exec("await stage_utils.create_new_stage_async(template='empty')")
        await self._exec("stage_utils.define_prim('/World', 'Xform')")
        await self._exec("stage_utils.define_prim('/World/Ephemeral', 'Cube')")

        exists = await self._eval("stage_utils.get_current_stage().GetPrimAtPath('/World/Ephemeral').IsValid()")
        self.assertTrue(exists, "Prim should exist before deletion")

        await self._exec("stage_utils.delete_prim('/World/Ephemeral')")

        gone = await self._eval("not stage_utils.get_current_stage().GetPrimAtPath('/World/Ephemeral').IsValid()")
        self.assertTrue(gone, "Prim should not exist after deletion")
