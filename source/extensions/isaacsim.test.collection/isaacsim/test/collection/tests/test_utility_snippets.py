# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for utility code snippets from Isaac Sim documentation."""

import omni.kit.test

################################################################################
### !!!IMPORTANT!!!
### All of the tests below are for utility snippets from the isaac sim docs.
### If you fix an issue here make sure to update the code in the docs as well
### The idea is that we can catch any api changes and update the docs appropriately
################################################################################


class TestUtilitySnippets(omni.kit.test.AsyncTestCase):
    """Tests for utility code snippets from Isaac Sim documentation."""

    async def setUp(self):
        """Set up test environment with new stage."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up test environment and wait for assets to load."""
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    # simple fastcache smoke test
    async def test_physics_scene(self):
        """Test physics scene creation with gravity and PhysX settings."""
        ### Code Start

        import omni
        from pxr import Gf, Sdf, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        # Add a physics scene prim to stage
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
        # Set gravity vector
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        ### Code End

        ### Code Start

        from pxr import PhysxSchema

        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")

        ### Code End

        ### Code Start

        import omni
        from pxr import PhysicsSchemaTools

        stage = omni.usd.get_context().get_stage()
        PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, -100), Gf.Vec3f(1.0))

        ### Code End

    async def test_enable_physics_collision_convex(self):
        """Test enabling physics collision with convex hull approximation."""
        ###
        import omni
        from omni.physx.scripts import utils

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Enable physics on prim
        # If a tighter collision approximation is desired use convexDecomposition instead of convexHull
        utils.setRigidBody(cube_prim, "convexHull", False)

        ###

    async def test_enable_physics_collision_decomp(self):
        """Test enabling physics collision with convex decomposition."""
        ###
        import omni
        from omni.physx.scripts import utils

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Enable physics on prim
        # If a tighter collision approximation is desired use convexDecomposition instead of convexHull
        utils.setRigidBody(cube_prim, "convexDecomposition", False)
        ###

    async def test_mass_properties(self):
        """Test setting mass and density properties on rigid bodies."""
        ###
        import omni
        from omni.physx.scripts import utils
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Make it a rigid body
        utils.setRigidBody(cube_prim, "convexHull", False)

        mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
        mass_api.CreateMassAttr(10)
        ### Alternatively set the density
        mass_api.CreateDensityAttr(1000)

    async def test_traverse_assign_collision(self):
        """Test traversing stage and assigning collision to all meshes."""
        import omni
        from omni.physx.scripts import utils
        from pxr import Gf, Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()

        def add_cube(stage, path, size: float = 10, offset: Gf.Vec3d = Gf.Vec3d(0, 0, 0)):
            cubeGeom = UsdGeom.Cube.Define(stage, path)
            cubeGeom.CreateSizeAttr(size)
            cubeGeom.AddTranslateOp().Set(offset)

        ### The following prims are added for illustrative purposes
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Torus")
        # all prims under AddCollision will get collisons assigned
        add_cube(stage, "/World/Cube_0", offset=Gf.Vec3d(100, 100, 0))
        # create a prim nested under without a parent
        add_cube(stage, "/World/Nested/Cube", offset=Gf.Vec3d(100, 0, 100))
        ###

        # Traverse all prims in the stage starting at this path
        curr_prim = stage.GetPrimAtPath("/")

        for prim in Usd.PrimRange(curr_prim):
            # only process shapes and meshes
            if (
                prim.IsA(UsdGeom.Cylinder)
                or prim.IsA(UsdGeom.Capsule)
                or prim.IsA(UsdGeom.Cone)
                or prim.IsA(UsdGeom.Sphere)
                or prim.IsA(UsdGeom.Cube)
            ):
                # use a ConvexHull for regular prims
                utils.setCollider(prim, approximationShape="convexHull")
            elif prim.IsA(UsdGeom.Mesh):
                # "None" will use the base triangle mesh if available
                # Can also use "convexDecomposition", "convexHull", "boundingSphere", "boundingCube"
                utils.setCollider(prim, approximationShape="none")

    async def test_material(self):
        """Test creating and binding MDL material to a prim."""
        ###
        import omni
        from pxr import Gf, Sdf, UsdShade

        mtl_created_list = []
        # Create a new material using OmniGlass.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniGlass.mdl",
            mtl_name="OmniGlass",
            mtl_created_list=mtl_created_list,
        )
        # Get reference to created material
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(mtl_prim, "glass_color", Gf.Vec3f(0, 1, 0), Sdf.ValueTypeNames.Color3f)
        omni.usd.create_material_input(mtl_prim, "glass_ior", 1.0, Sdf.ValueTypeNames.Float)
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
        ###

    async def test_material_texture(self):
        """Test creating material with texture and binding to a prim."""
        ###
        import omni
        from isaacsim.storage.native import get_assets_root_path_async
        from pxr import Sdf, UsdShade

        assets_root_path = await get_assets_root_path_async()
        mtl_created_list = []
        # Create a new material using OmniPBR.mdl
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniPBR.mdl",
            mtl_name="OmniPBR",
            mtl_created_list=mtl_created_list,
        )
        stage = omni.usd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        # Set material inputs, these can be determined by looking at the .mdl file
        # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
        omni.usd.create_material_input(
            mtl_prim,
            "diffuse_texture",
            assets_root_path + "/Isaac/Samples/DR/Materials/Textures/marble_tile.png",
            Sdf.ValueTypeNames.Asset,
        )
        # Create a prim to apply the material to
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the path to the prim
        cube_prim = stage.GetPrimAtPath(path)
        # Bind the material to the prim
        cube_mat_shade = UsdShade.Material(mtl_prim)
        UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)
        ###

    async def test_add_transform(self):
        """Test adding transform operations to a prim."""
        ###
        import omni
        from pxr import Gf, UsdGeom

        # Create a cube mesh in the stage
        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the prim and set its transform matrix
        cube_prim = stage.GetPrimAtPath(path)
        xform = UsdGeom.Xformable(cube_prim)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        ###

    async def test_align_prims(self):
        """Test aligning one prim's pose to another."""
        ###
        import omni
        from isaacsim.core.experimental.prims import XformPrim
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        # Create a cube
        result, path_a = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        prim_a = stage.GetPrimAtPath(path_a)
        # change the cube pose
        xform = UsdGeom.Xformable(prim_a)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        # Create a second cube
        result, path_b = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        # Get the transform of the first cube using experimental API
        xform_prim_a = XformPrim(path_a)
        positions, orientations = xform_prim_a.get_world_poses()
        # Set the pose of prim_b to that of prim_a using experimental API
        xform_prim_b = XformPrim(path_b, reset_xform_op_properties=True)
        xform_prim_b.set_world_poses(positions, orientations)
        ###

    async def test_get_world_transform(self):
        """Test getting world transform of selected prims."""
        ###
        import omni
        from isaacsim.core.experimental.prims import XformPrim
        from pxr import Gf, UsdGeom

        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()

        #### For testing purposes we create and select a prim
        #### This section can be removed if you already have a prim selected
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
        cube_prim = stage.GetPrimAtPath(path)
        # change the cube pose
        xform = UsdGeom.Xformable(cube_prim)
        transform = xform.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(10.0, 1.0, 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), 290))
        transform.Set(mat)
        omni.usd.get_context().get_selection().set_prim_path_selected(path, True, True, True, False)
        ####

        # Get list of selected primitives
        selected_prims = usd_context.get_selection().get_selected_prim_paths()
        # Loop through all prims and print their transforms using experimental API
        for s in selected_prims:
            print("Selected", s)
            xform_prim = XformPrim(s)
            positions, orientations = xform_prim.get_world_poses()
            pos = positions.numpy()[0]
            quat = orientations.numpy()[0]  # wxyz format
            print("Translation: ", pos)
            print("Rotation (wxyz): ", quat[0], ",", quat[1], ",", quat[2], ",", quat[3])
        ###

    async def test_async_task(self):
        """Test using async tasks to pause simulation after a frame."""
        ###
        import asyncio

        import omni

        # Async task that pauses simulation once the incoming task is complete
        async def pause_sim(task):
            done, pending = await asyncio.wait({task})
            if task in done:
                print("Waited until next frame, pausing")
                omni.timeline.get_timeline_interface().pause()

        # Start simulation, then wait a frame and run the pause_sim task
        omni.timeline.get_timeline_interface().play()
        task = asyncio.ensure_future(omni.kit.app.get_app().next_update_async())
        asyncio.ensure_future(pause_sim(task))
        ###

    async def test_camera_intrinsics(self):
        """Test getting camera intrinsic parameters from viewport."""
        import math

        import omni
        from omni.syntheticdata import helpers

        stage = omni.usd.get_context().get_stage()
        viewport_api = omni.kit.viewport.utility.get_active_viewport()
        # Set viewport resolution, changes will occur on next frame
        viewport_api.set_texture_resolution((512, 512))
        # get resolution
        width, height = viewport_api.get_texture_resolution()
        aspect_ratio = width / height
        # get camera prim attached to viewport
        camera = stage.GetPrimAtPath(viewport_api.get_active_camera())
        focal_length = camera.GetAttribute("focalLength").Get()
        horiz_aperture = camera.GetAttribute("horizontalAperture").Get()
        vert_aperture = camera.GetAttribute("horizontalAperture").Get() * (float(height) / width)
        # Pixels are square so we can also do:
        # vert_aperture = height / width * horiz_aperture
        near, far = camera.GetAttribute("clippingRange").Get()
        fov = 2 * math.atan(horiz_aperture / (2 * focal_length))

        # helper to compute projection matrix
        proj_mat = helpers.get_projection_matrix(fov, aspect_ratio, near, far)

        # compute focal point and center
        focal_x = height * focal_length / vert_aperture
        focal_y = width * focal_length / horiz_aperture
        center_x = height * 0.5
        center_y = width * 0.5

    async def test_get_mesh_size(self):
        """Test getting bounding box size of a mesh prim."""
        import omni
        from pxr import Usd, UsdGeom

        stage = omni.usd.get_context().get_stage()
        result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cone")
        # Get the prim
        prim = stage.GetPrimAtPath(path)
        # Get the size
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bbox_cache.Clear()
        prim_bbox = bbox_cache.ComputeWorldBound(prim)
        prim_range = prim_bbox.ComputeAlignedRange()
        prim_size = prim_range.GetSize()

    async def test_apply_semantics_on_entire_stage(self):
        """Test applying semantic labels to all meshes on the stage."""
        import omni.kit.commands

        omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cone")

        ### Code Start
        import omni.usd
        from isaacsim.core.experimental.utils.semantics import add_labels

        def remove_prefix(name, prefix):
            if name.startswith(prefix):
                return name[len(prefix) :]
            return name

        def remove_numerical_suffix(name):
            suffix = name.split("_")[-1]
            if suffix.isnumeric():
                return name[: -len(suffix) - 1]
            return name

        def remove_underscores(name):
            return name.replace("_", "")

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Mesh":
                label = str(prim.GetPrimPath()).split("/")[-1]
                label = remove_prefix(label, "SM_")
                label = remove_numerical_suffix(label)
                label = remove_underscores(label)
                add_labels(prim, labels=[label], taxonomy="class")

    # Test will not run with S3 assets
    # async def test_save_to_file(self):
    #     import carb
    #     import omni
    #     from isaacsim.storage.native import get_assets_root_path_async

    #     assets_root = await get_assets_root_path_async()
    #     # Create a prim
    #     result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
    #     # Change the path as needed
    #     omni.usd.get_context().save_as_stage(assets_root + "/Users/test/saved.usd", None)
