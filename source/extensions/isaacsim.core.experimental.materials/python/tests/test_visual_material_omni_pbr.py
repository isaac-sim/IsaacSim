# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
)
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Sdf, UsdShade


def parametrize(
    *,
    devices: list[Literal["cpu", "cuda"]] = ["cpu", "cuda"],
    backends: list[Literal["usd", "fabric", "tensor"]] = ["usd", "fabric", "tensor"],
    instances: list[Literal["one", "many"]] = ["one", "many"],
    operations: list[Literal["wrap", "create"]] = ["wrap", "create"],
    prim_class: type = OmniPbrMaterial,
    prim_class_kwargs: dict = {},
    max_num_prims: int = 5,
):
    def decorator(func):
        async def wrapper(self):
            for device in devices:
                for backend in backends:
                    for instance in instances:
                        for operation in operations:
                            assert backend in ["usd", "fabric", "tensor"], f"Invalid backend: {backend}"
                            assert instance in ["one", "many"], f"Invalid instance: {instance}"
                            assert operation in ["wrap", "create"], f"Invalid operation: {operation}"
                            cprint(
                                f"  |-- device: {device}, backend: {backend}, instance: {instance}, operation: {operation}"
                            )
                            # create new stage
                            await stage_utils.create_new_stage_async()
                            # define prims
                            if operation == "wrap":
                                for i in range(max_num_prims):
                                    omni.kit.commands.execute(
                                        "CreateAndBindMdlMaterialFromLibrary", mdl_name="OmniPBR.mdl", mtl_name=f"A_{i}"
                                    )
                            # configure simulation manager
                            SimulationManager.set_physics_sim_device(device)
                            # parametrize test
                            if operation == "wrap":
                                paths = "/Looks/A_0" if instance == "one" else "/Looks/A_.*"
                            elif operation == "create":
                                paths = (
                                    "/World/A_0"
                                    if instance == "one"
                                    else [f"/World/A_{i}" for i in range(max_num_prims)]
                                )
                            prim = prim_class(paths, **prim_class_kwargs)
                            num_prims = 1 if instance == "one" else max_num_prims
                            # call test method according to backend
                            if backend == "tensor":
                                omni.timeline.get_timeline_interface().play()
                                await omni.kit.app.get_app().next_update_async()
                                await func(self, prim=prim, num_prims=num_prims, device=device, backend=backend)
                                omni.timeline.get_timeline_interface().stop()
                            elif backend in ["usd", "fabric"]:
                                await func(self, prim=prim, num_prims=num_prims, device=device, backend=backend)

        return wrapper

    return decorator


class TestOmniPBR(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"])
    async def test_len(self, prim, num_prims, device, backend):
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"])
    async def test_properties_and_getters(self, prim, num_prims, device, backend):
        # test cases (properties)
        # - materials
        self.assertEqual(len(prim.materials), num_prims, f"Invalid materials len ({num_prims} prims)")
        for usd_prim in prim.prims:
            self.assertTrue(usd_prim.IsValid() and usd_prim.IsA(UsdShade.Material), f"Invalid material")
        # - shaders
        self.assertEqual(len(prim.shaders), num_prims, f"Invalid shaders len ({num_prims} prims)")
        for shader in prim.shaders:
            self.assertTrue(isinstance(shader, UsdShade.Shader), f"Invalid shader")

    @parametrize(devices=["cpu"], backends=["usd"], instances=["one"], operations=["wrap", "create"])
    async def test_input_definitions(self, prim, num_prims, device, backend):
        mdl_path = prim.shaders[0].GetSourceAsset("mdl").resolvedPath
        self.assertTrue(mdl_path.endswith("OmniPBR.mdl"), f"Invalid MDL path: {mdl_path}")
        with open(mdl_path) as file:
            mdl_content = file.read()
        for name in prim._inputs:
            assert name in mdl_content, f"Wrong input: {name}"

    @parametrize(backends=["usd"])
    async def test_input_values(self, prim, num_prims, device, backend):
        cases = {
            # Albedo
            "diffuse_color_constant": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "diffuse_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "albedo_desaturation": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "albedo_add": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "albedo_brightness": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "diffuse_tint": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            # Reflectivity
            "reflection_roughness_constant": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "reflection_roughness_texture_influence": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "reflectionroughness_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "metallic_constant": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "metallic_texture_influence": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "metallic_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "specular_level": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            # Reflectivity (ORM)
            "enable_ORM_texture": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "ORM_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            # AO
            "ao_to_diffuse": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "ao_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            # Emissive
            "enable_emission": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "emissive_color": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "emissive_color_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "emissive_mask_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "emissive_intensity": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            # Opacity
            "enable_opacity": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "opacity_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "opacity_constant": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "enable_opacity_texture": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "opacity_mode": lambda count: draw_sample(
                shape=(count, 1), dtype=wp.int32, high=5
            ),  # one above (exclusive)
            "opacity_threshold": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            # Normal
            "geometry_normal_roughness_strength": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "bump_factor": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "normalmap_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "detail_bump_factor": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "detail_normalmap_texture": lambda count: draw_choice(shape=(count,), choices=["/a", "/bc", "/def"]),
            "flip_tangent_u": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "flip_tangent_v": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            # UV (UVW Projection)
            "project_uvw": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "world_or_object": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
            "uv_space_index": lambda count: draw_sample(
                shape=(count, 1), dtype=wp.int32, high=4
            ),  # one above (exclusive)
            # UV (UVW Adjustments)
            "texture_translate": lambda count: draw_sample(shape=(count, 2), dtype=wp.float32),
            "texture_rotate": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "texture_scale": lambda count: draw_sample(shape=(count, 2), dtype=wp.float32),
            "detail_texture_translate": lambda count: draw_sample(shape=(count, 2), dtype=wp.float32),
            "detail_texture_rotate": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "detail_texture_scale": lambda count: draw_sample(shape=(count, 2), dtype=wp.float32),
            # Geometry
            "round_edges_radius": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "round_edges_roundness": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "round_edges_across_materials": lambda count: draw_sample(shape=(count, 1), dtype=wp.bool),
        }
        # check inputs
        for name in cases:
            assert name in prim._inputs, f"Invalid input: {name}"
        for name in prim._inputs:
            assert name in cases, f"Missing case: {name}"
        # test cases
        for name, sample_func in cases.items():
            sdf_type = prim._inputs[name]
            cprint(f"  |   input: {name}, sdf_type: {sdf_type}")
            for indices, expected_count in draw_indices(count=num_prims, step=2):
                cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
                for v0, expected_v0 in sample_func(expected_count):
                    prim.set_input_values(name, values=v0, indices=indices)
                    output = prim.get_input_values(name, indices=indices)
                    # check
                    if sdf_type == Sdf.ValueTypeNames.Asset:
                        check_lists(output, expected_v0)
                    else:
                        check_array(output, shape=expected_v0.shape, device=device)
                        check_allclose(expected_v0, output, given=(v0,))
