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

"""Verifies XformPrim runtime creation and transform-oriented property access. Covers visibility, world and local poses, local scales, default state, visual materials, and event handling."""

from typing import Any, Literal

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.backend import use_backend
from isaacsim.core.simulation_manager import IsaacEvents
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .common import (
    check_allclose,
    check_array,
    check_equal,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Populate stage.

    Args:
        max_num_prims: Maximum number of prims to create for a test case.
        operation: Stage population operation to use.
        **kwargs: Additional keyword arguments.
    """
    assert operation == "wrap", "Other operations except 'wrap' are not supported"
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    stage_utils.define_prim(f"/World", "Xform")
    stage_utils.define_prim(f"/World/PhysicsScene", "PhysicsScene")
    for i in range(max_num_prims):
        xform_prim = stage_utils.define_prim(f"/World/A_{i}", "Xform")
        stage_utils.define_prim(f"/World/A_{i}/B", "Cube")
        UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
        mass_api.GetMassAttr().Set(1.0)


async def populate_stage_with_rotation(
    max_num_prims: int,
    operation: Literal["wrap", "create"],
    *,
    xform_op: Literal["orient", "rotateZYX", "transform"],
    **kwargs: Any,
) -> None:
    """Populate stage with prims whose rotation is authored using the given xformOp.

    Args:
        max_num_prims: Maximum number of prims to create for a test case.
        operation: Stage population operation to use.
        xform_op: Name of the xformOp used to author each prim's rotation.
        **kwargs: Additional keyword arguments.
    """
    await populate_stage(max_num_prims, operation, **kwargs)
    stage = stage_utils.get_current_stage(backend="usd")
    translation = Gf.Vec3d(1.0, 2.0, 3.0)
    rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 45.0) * Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), 30.0)
    for i in range(max_num_prims):
        xformable = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/A_{i}"))
        if xform_op == "orient":
            xformable.AddTranslateOp().Set(translation)
            xformable.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
        elif xform_op == "rotateZYX":
            xformable.AddTranslateOp().Set(translation)
            xformable.AddRotateZYXOp().Set(Gf.Vec3f(30.0, 45.0, 60.0))
        elif xform_op == "transform":
            xformable.AddTransformOp().Set(Gf.Matrix4d().SetRotate(rotation) * Gf.Matrix4d().SetTranslate(translation))
        else:
            raise ValueError(f"Unsupported xformOp: {xform_op}")


class TestXformPrim(omni.kit.test.AsyncTestCase):
    """Test xform prim."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        max_num_prims=1,
    )
    async def test_runtime_instance_creation(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test runtime instance creation.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        XformPrim("/World/A_0")

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=XformPrim, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid XformPrim ({num_prims} prims) len")

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=XformPrim, populate_stage_func=populate_stage)
    async def test_visibilities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test visibilities.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_visibilities(v0, indices=indices)
                    output = prim.get_visibilities(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_equal(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["usd", "usdrt", "fabric"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage,
    )
    async def test_world_poses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test world poses.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        else:
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_world_poses(v0, v1, indices=indices)
                    output = prim.get_world_poses(indices=indices)
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["usd", "usdrt", "fabric"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage,
    )
    async def test_local_poses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test local poses.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        else:
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_local_poses(v0, v1, indices=indices)
                    output = prim.get_local_poses(indices=indices)
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["usd", "usdrt", "fabric"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage,
    )
    async def test_world_scales(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test world scales.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        else:
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    # we set local scales, when we get the global scale, we should get the same scale
                    prim.set_local_scales(v0, indices=indices)
                    output = prim.get_world_scales(indices=indices)
                check_array(output, shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["usd", "usdrt", "fabric"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage,
    )
    async def test_local_scales(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test local scales.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        else:
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_local_scales(v0, indices=indices)
                    output = prim.get_local_scales(indices=indices)
                check_array(output, shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=XformPrim, populate_stage_func=populate_stage)
    async def test_default_state(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test default state.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_default_state(v0, v1, indices=indices)
                    output = prim.get_default_state(indices=indices)
                    prim.reset_to_default_state()
                await omni.kit.app.get_app().next_update_async()
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=XformPrim, populate_stage_func=populate_stage)
    async def test_visual_materials(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test visual materials.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        from isaacsim.core.experimental.materials import OmniGlassMaterial, OmniPbrMaterial, PreviewSurfaceMaterial

        choices = [
            OmniGlassMaterial("/materials/omni_glass"),
            OmniPbrMaterial("/materials/omni_pbr"),
            PreviewSurfaceMaterial("/materials/preview_surface"),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_visual_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_visual_materials(v0, indices=indices)
                    output = prim.get_applied_visual_materials(indices=indices)
                check_lists(expected_v0, output, check_value=False)
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_visual_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_visual_materials(v0)
                output = prim.get_applied_visual_materials()
            check_lists(expected_v0, output, check_value=False)
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_visual_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    @parametrize(
        backends=["usd"],
        instances=["one"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage,
    )
    async def test_physics_materials(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test physics material binding on an Xform prim.

        Args:
            prim: Prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        from isaacsim.core.experimental.materials import RigidBodyMaterial
        from pxr import UsdShade

        material = RigidBodyMaterial("/materials/physics", static_frictions=[0.9])
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            prim.apply_physics_materials(material, weaker_than_descendants=True)

        binding = UsdShade.MaterialBindingAPI(prim.prims[0]).GetDirectBinding(materialPurpose="physics")
        self.assertEqual(binding.GetMaterialPath(), material.materials[0].GetPath())
        self.assertEqual(
            binding.GetBindingRel().GetMetadata(UsdShade.Tokens.bindMaterialAs),
            UsdShade.Tokens.weakerThanDescendants,
        )

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=XformPrim, populate_stage_func=populate_stage)
    async def test_events(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test events.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        await omni.kit.app.get_app().next_update_async()
        prim.reset_xform_op_properties()
        # trigger events automatically
        timeline = omni.timeline.get_timeline_interface()
        for _ in range(2):
            timeline.play()
            for _ in range(2):
                await omni.kit.app.get_app().next_update_async()
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()
        # trigger events manually
        event_dispatcher = carb.eventdispatcher.get_eventdispatcher()
        event_dispatcher.dispatch_event(IsaacEvents.PHYSICS_WARMUP.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.SIMULATION_VIEW_CREATED.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.PHYSICS_READY.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.POST_RESET.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.PRIM_DELETION.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.PRE_PHYSICS_STEP.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.POST_PHYSICS_STEP.value, payload={})
        event_dispatcher.dispatch_event(IsaacEvents.TIMELINE_STOP.value, payload={})

    # -- reset_xform_op_properties: world pose preservation --

    async def _check_world_transforms_after_reset(self, prim: XformPrim) -> None:
        """Reset the transformation operation attributes of the wrapped prims and compare world transforms.

        Args:
            prim: Prim wrapper under test.
        """
        # let the app tick so that graph execution on the newly opened stage settles before it is replaced
        await omni.kit.app.get_app().next_update_async()
        compute_transforms = lambda: [
            np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64)
            for p in prim.prims
        ]
        expected = compute_transforms()
        prim.reset_xform_op_properties()
        output = compute_transforms()
        check_allclose(expected, output)
        for p in prim.prims:
            cprint(f"  |    |-- xformOpOrder: {list(p.GetAttribute('xformOpOrder').Get())}")
            check_lists(
                ["xformOp:translate", "xformOp:orient", "xformOp:scale"],
                list(p.GetAttribute("xformOpOrder").Get()),
            )

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage_with_rotation,
        populate_stage_func_kwargs={"xform_op": "orient"},
    )
    async def test_reset_xform_op_properties_from_orient(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Test world pose preservation when resetting prims authored with 'xformOp:orient'.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        await self._check_world_transforms_after_reset(prim)

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage_with_rotation,
        populate_stage_func_kwargs={"xform_op": "rotateZYX"},
    )
    async def test_reset_xform_op_properties_from_rotate_zyx(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Test world pose preservation when resetting prims authored with 'xformOp:rotateZYX'.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        await self._check_world_transforms_after_reset(prim)

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        prim_class=XformPrim,
        populate_stage_func=populate_stage_with_rotation,
        populate_stage_func_kwargs={"xform_op": "transform"},
    )
    async def test_reset_xform_op_properties_from_transform(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Test world pose preservation when resetting prims authored with 'xformOp:transform'.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        await self._check_world_transforms_after_reset(prim)
