# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test for cloner."""

import os
from unittest import mock

import numpy as np
import omni.kit
import usdrt
from isaacsim.core.cloner import Cloner, GridCloner
from isaacsim.storage.native import get_assets_root_path_async
from omni.physics.core import get_physics_simulation_interface
from omni.physx import get_physxunittests_interface
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils, Vt


class TestSimpleCloner(omni.kit.test.AsyncTestCase):
    """Test simple cloner."""

    async def setUp(self):
        """Set up test environment."""
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.core.cloner")
        self._extension_path = ext_manager.get_extension_path(ext_id)
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_simple_cloner(self):
        """Test simple cloner."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_enable_change_listener_without_prior_disable(self):
        """Test enable_change_listener is safe before disable_change_listener."""
        cloner = Cloner()

        cloner.enable_change_listener()

    async def test_clone_rejects_none_source_prim_path(self):
        """Test clone rejects None source path before USD binding calls."""
        cloner = Cloner()

        with self.assertRaisesRegex(TypeError, "source_prim_path"):
            cloner.clone(source_prim_path=None, prim_paths=[])

    async def test_clone_rejects_invalid_source_prim_path(self):
        """Test clone distinguishes invalid SdfPath strings from missing prims."""
        cloner = Cloner()

        with self.assertRaisesRegex(ValueError, "valid SdfPath"):
            cloner.clone(source_prim_path="not a valid path!!!", prim_paths=[])

    async def test_clone_restores_change_listener_when_validation_raises(self):
        """Test clone restores listeners when an exception interrupts cloning."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Cube.Define(stage, "/World/Cube_0")
        cloner = Cloner()

        with mock.patch.object(cloner, "disable_change_listener") as disable_listener:
            with mock.patch.object(cloner, "enable_change_listener") as enable_listener:
                with self.assertRaisesRegex(ValueError, "positions"):
                    cloner.clone(
                        source_prim_path="/World/Cube_0",
                        prim_paths=["/World/Cube_0"],
                        positions=np.array([[0, 0, 0], [1, 1, 1]]),
                    )

        disable_listener.assert_called_once()
        enable_listener.assert_called_once()

    async def test_quatf_cloner(self):
        """Test quatf cloner."""
        await omni.usd.get_context().open_stage_async(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/quatf_d.usda")
        )

        stage = omni.usd.get_context().get_stage()
        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/Clones/Xform" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/Clones/Xform", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the Xform at target paths at specified positions
        cloner.clone(
            source_prim_path="/quatf", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/Clones/Xform_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/Clones/Xform_{i}").GetTypeName() == "Xform")
            self.assertTrue(
                stage.GetPrimAtPath(f"/Clones/Xform_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_quatd_cloner(self):
        """Test quatd cloner."""
        await omni.usd.get_context().open_stage_async(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/quatf_d.usda")
        )

        stage = omni.usd.get_context().get_stage()
        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/Clones/Xform" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/Clones/Xform", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the Xform at target paths at specified positions
        cloner.clone(
            source_prim_path="/quatd", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/Clones/Xform_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/Clones/Xform_{i}").GetTypeName() == "Xform")
            self.assertTrue(
                stage.GetPrimAtPath(f"/Clones/Xform_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_simple_cloner_physics_replication(self):
        """Test simple cloner physics replication."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        cube = UsdGeom.Cube.Define(stage, base_env_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0",
            prim_paths=target_paths,
            positions=cube_positions,
            replicate_physics=True,
            base_env_path="/World",
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_simple_cloner_copy_randomization(self):
        """Test simple cloner copy randomization."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner()

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0",
            prim_paths=target_paths,
            positions=cube_positions,
            replicate_physics=False,
            copy_from_source=True,
        )

        colors = [
            Vt.Vec3fArray(1, (Gf.Vec3f(1.0, 0.0, 0.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.0, 1.0, 0.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.0, 0.0, 1.0))),
            Vt.Vec3fArray(1, (Gf.Vec3f(0.5, 0.5, 0.5))),
        ]
        for i in range(4):
            stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("primvars:displayColor").Set(colors[i])

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("primvars:displayColor").Get() == colors[i]
            )

    async def test_grid_cloner(self):
        """Test grid cloner."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a GridCloner instance
        cloner = GridCloner(spacing=3)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        # clone the cube at target paths
        cloner.clone(source_prim_path="/World/Cube_0", prim_paths=target_paths, replicate_physics=False)

        target_translations = [
            Gf.Vec3d(1.5, -1.5, 0),
            Gf.Vec3d(1.5, 1.5, 0),
            Gf.Vec3d(-1.5, -1.5, 0),
            Gf.Vec3d(-1.5, 1.5, 0),
        ]
        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_grid_cloner_recomputes_transforms_when_count_changes(self):
        """Test grid cloner recomputes transforms when clone count changes."""
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        cloner = GridCloner(spacing=2.0)

        positions_4, _ = cloner.get_clone_transforms(4)
        positions_9, _ = cloner.get_clone_transforms(9)

        self.assertEqual(len(positions_4), 4)
        self.assertEqual(len(positions_9), 9)
        self.assertEqual(positions_9[0], [2.0, -2.0, 0])
        self.assertEqual(positions_9[-1], [-2.0, 2.0, 0])

    async def test_grid_cloner_empty_clone_count_returns_empty_transforms(self):
        """Test grid cloner returns empty transforms for zero clones."""
        cloner = GridCloner(spacing=2.0)

        positions, orientations = cloner.get_clone_transforms(0)

        self.assertEqual(positions, [])
        self.assertEqual(orientations, [])

    async def test_grid_cloner_physics_replication(self):
        """Test grid cloner physics replication."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        cube = UsdGeom.Cube.Define(stage, base_env_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())

        # create a GridCloner instance
        cloner = GridCloner(spacing=3)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        # clone the cube at target paths
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, replicate_physics=True, base_env_path="/World"
        )

        target_translations = [
            Gf.Vec3d(1.5, -1.5, 0),
            Gf.Vec3d(1.5, 1.5, 0),
            Gf.Vec3d(-1.5, -1.5, 0),
            Gf.Vec3d(-1.5, 1.5, 0),
        ]
        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_grid_cloner_articulation(self):
        """Test grid cloner articulation."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0", prim_paths=target_paths, replicate_physics=False
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

    async def test_grid_cloner_articulation_physics_replication(self):
        """Test grid cloner articulation physics replication."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

    async def test_grid_cloner_articulation_physics_replication_envIds(self):
        """Test grid cloner articulation physics replication envIds."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            enable_env_ids=True,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

    async def test_grid_cloner_copy_addition(self):
        """Test grid cloner copy addition."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=True,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

        UsdGeom.Cube.Define(stage, "/World/envs/env_0/Cube")
        UsdGeom.Sphere.Define(stage, "/World/envs/env_1/Sphere")
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Sphere").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Cube").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Sphere").IsValid() == True)

    async def test_grid_cloner_inherit_addition(self):
        """Test grid cloner inherit addition."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=False,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )

        UsdGeom.Cube.Define(stage, "/World/envs/env_0/Cube")
        UsdGeom.Sphere.Define(stage, "/World/envs/env_1/Sphere")
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_0/Sphere").IsValid() == False)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Cube").IsValid() == True)
        self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_1/Sphere").IsValid() == True)

    async def test_grid_cloner_offsets(self):
        """Test grid cloner offsets."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one cube
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        position_offsets = [[0, 0, 1.0]] * 100
        orientation_offsets = [[0, 0, 0, 1.0]] * 100

        # clone the cube at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=False,
            position_offsets=position_offsets,
            orientation_offsets=orientation_offsets,
        )

        for i in range(100):
            self.assertTrue(stage.GetPrimAtPath(f"/World/envs/env_{i}") is not None)
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}/Ant/torso").HasAPI(UsdPhysics.ArticulationRootAPI)
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()
                == Gf.Vec3d(*target_translations[i])
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:translate").Get()[2] == 1.0
            )
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/envs/env_{i}").GetAttribute("xformOp:orient").Get()
                == Gf.Quatd(0.0, Gf.Vec3d(0.0, 0.0, 1.0))
            )

    async def test_simple_cloner_on_stage_in_memory(self):
        """Test simple cloner on stage in memory."""
        stage = Usd.Stage.CreateInMemory()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner(stage=stage)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cube_positions = np.array([[0, 0, 0], [3, 0, 0], [6, 0, 0], [9, 0, 0]])
        target_translations = []
        for pos in cube_positions:
            target_translations.append(Gf.Vec3d(*pos.tolist()))

        # clone the cube at target paths at specified positions
        cloner.clone(
            source_prim_path="/World/Cube_0", prim_paths=target_paths, positions=cube_positions, replicate_physics=False
        )

        for i in range(4):
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}") is not None)
            self.assertTrue(stage.GetPrimAtPath(f"/World/Cube_{i}").GetTypeName() == "Cube")
            self.assertTrue(
                stage.GetPrimAtPath(f"/World/Cube_{i}").GetAttribute("xformOp:translate").Get()
                == target_translations[i]
            )

    async def test_fabric_cloner(self):
        """Test fabric cloner."""
        stage = Usd.Stage.CreateInMemory()

        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)

        stage_id = cache.GetId(stage).ToLongInt()

        # create our base environment with one cube
        base_env_path = "/World/Cube_0"
        UsdGeom.Cube.Define(stage, base_env_path)

        # create a Cloner instance
        cloner = Cloner(stage=stage)

        # generate 4 paths that begin with "/World/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/Cube", 4)

        cloner.clone(
            source_prim_path=base_env_path, prim_paths=target_paths, replicate_physics=False, clone_in_fabric=True
        )

        # check that the prims are not in USD
        for path in target_paths:
            if path != base_env_path:
                prim = stage.GetPrimAtPath(str(path))
                self.assertTrue(not prim.IsValid())

        usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)
        for path in target_paths:
            prim = usdrt_stage.GetPrimAtPath(str(path))
            self.assertTrue(prim is not None)
            self.assertTrue(prim.IsValid())
            self.assertTrue(prim.GetTypeName() == "Cube")

        cache.Erase(stage)

    async def test_fabric_grid_cloner_offsets(self):
        """Test fabric grid cloner offsets."""
        stage = omni.usd.get_context().get_stage()

        # create our base environment with one ant
        base_env_path = "/World/envs"

        cloner = GridCloner(spacing=3)
        cloner.define_base_env(base_env_path + "/env_0")
        UsdGeom.Xform.Define(stage, base_env_path + "/env_0")
        prim = stage.DefinePrim(base_env_path + "/env_0/Ant", "Xform")
        asset_root_path = await get_assets_root_path_async()
        prim.GetReferences().AddReference(asset_root_path + "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd")

        target_paths = cloner.generate_paths("/World/envs/env", 100)

        position_offsets = [[0, 0, 1.0]] * 100
        orientation_offsets = [[0, 0, 0, 1.0]] * 100

        # clone the ants at target paths
        target_translations = cloner.clone(
            source_prim_path="/World/envs/env_0",
            prim_paths=target_paths,
            replicate_physics=True,
            base_env_path="/World/envs",
            copy_from_source=False,
            position_offsets=position_offsets,
            orientation_offsets=orientation_offsets,
            clone_in_fabric=True,
        )

        for i in range(100):
            usdrt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
            prim = usdrt_stage.GetPrimAtPath(f"/World/envs/env_{i}")
            self.assertTrue(prim is not None)
            self.assertTrue(prim.IsValid())

            world_matrix_attr = prim.GetAttribute("omni:fabric:worldMatrix")
            self.assertTrue(world_matrix_attr is not None)

            transform = usdrt.Gf.Transform(world_matrix_attr.Get())
            self.assertTrue(transform.GetTranslation() == usdrt.Gf.Vec3d(Gf.Vec3d(*target_translations[i])))

            self.assertTrue(transform.GetTranslation()[2] == 1.0)
            self.assertTrue(
                transform.GetRotation() == usdrt.Gf.Rotation(usdrt.Gf.Quatd(0.0, usdrt.Gf.Vec3d(0.0, 0.0, 1.0)))
            )

    def get_num_dynamic_rigid_bodies(self):
        """Get num dynamic rigid bodies."""
        sim_stats = get_physxunittests_interface().get_physics_stats()
        return sim_stats["numDynamicRigids"]

    async def test_fabric_physics_cloner(self):
        """Test fabric physics cloner."""
        stage = Usd.Stage.CreateInMemory()

        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)

        stage_id = cache.GetId(stage).ToLongInt()

        # create our base environment with one cube
        base_env_path = "/World/envs"
        UsdGeom.Xform.Define(stage, base_env_path)
        source_prim_path = "/World/envs/Cube_0"
        cube = UsdGeom.Cube.Define(stage, source_prim_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # create a Cloner instance
        cloner = Cloner(stage=stage)

        # generate 4 paths that begin with "/World/envs/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/envs/Cube", 4)

        cloner.clone(
            source_prim_path=source_prim_path,
            base_env_path=base_env_path,
            prim_paths=target_paths,
            replicate_physics=True,
            clone_in_fabric=True,
        )

        # attach physics to the stage
        get_physics_simulation_interface().initialize(stage_id)

        # check that the prims are in physics
        num_dynamic_rigid_bodies = self.get_num_dynamic_rigid_bodies()
        self.assertTrue(num_dynamic_rigid_bodies == 4)

        get_physics_simulation_interface().close()

        cache.Erase(stage)

    async def test_fabric_physics_cloner_usd_context(self):
        """Test fabric physics cloner usd context."""

        await omni.usd.get_context().new_stage_async()

        stage = Usd.Stage.CreateInMemory()

        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)

        stage_id = cache.GetId(stage).ToLongInt()

        # create our base environment with one cube
        base_env_path = "/World/envs"
        UsdGeom.Xform.Define(stage, base_env_path)
        source_prim_path = "/World/envs/Cube_0"
        cube = UsdGeom.Cube.Define(stage, source_prim_path)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # create a Cloner instance
        cloner = Cloner(stage=stage)

        # generate 4 paths that begin with "/World/envs/Cube" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/envs/Cube", 4)

        cloner.clone(
            source_prim_path=source_prim_path,
            base_env_path=base_env_path,
            prim_paths=target_paths,
            replicate_physics=True,
            clone_in_fabric=True,
        )

        omni.usd.get_context().attach_stage_with_callback(stage_id, None)

        # attach physics to the stage
        get_physics_simulation_interface().initialize(stage_id)

        # check that the prims are in physics
        num_dynamic_rigid_bodies = self.get_num_dynamic_rigid_bodies()
        self.assertTrue(num_dynamic_rigid_bodies == 4)

        get_physics_simulation_interface().close()

        await omni.usd.get_context().new_stage_async()

        cache.Erase(stage)
