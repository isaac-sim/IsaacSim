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

"""Verifies the PrimDataReader interface, view manager lifecycle, buffer allocation, metadata callbacks, and transform helpers. Covers articulation, rigid body, and xform views, Isaac name overrides, articulation link discovery, and PhysX transform reads."""

import ctypes
from typing import Any

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import BufferDtype, GeomPrim, RigidPrim, XformPrim
from pxr import UsdGeom, UsdPhysics, UsdUtils


class TestPrimDataReaderInterface(omni.kit.test.AsyncTestCase):
    """Tests for the IPrimDataReader C++ Carbonite interface and pybind11 bindings."""

    async def setUp(self) -> None:
        """Acquire a fresh reader and initialize it for CPU."""
        self.reader = None
        self.timeline = omni.timeline.get_timeline_interface()
        self._stage_id = 0
        from isaacsim.core.experimental.prims.impl.extension import get_prim_data_reader

        self.reader = get_prim_data_reader()
        if self.reader is not None:
            self.reader.initialize(0, -1)

    async def tearDown(self) -> None:
        """Shut down the reader to clean up all views."""
        if self.reader is not None:
            self.reader.shutdown()
        if self.timeline.is_playing():
            self.timeline.stop()
            await omni.kit.app.get_app().next_update_async()

    async def _setup_stage(self) -> Any:
        """Create a fresh USD stage and (re-)initialize the reader against it.

        Returns:
            Requested value.
        """
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.reader.initialize(self._stage_id, -1)
        return stage

    async def test_acquire_interface(self) -> None:
        """Verify the Carbonite interface can be acquired and released."""
        from isaacsim.core.experimental.prims import _prims_reader

        reader = _prims_reader.acquire_prim_data_reader_interface()
        self.assertIsNotNone(reader)
        _prims_reader.release_prim_data_reader_interface(reader)

    async def test_get_prim_data_reader_singleton(self) -> None:
        """Verify get_prim_data_reader returns a non-None singleton."""
        self.assertIsNotNone(self.reader)

    async def test_provider_extension_setting_defaults_to_primdata(self) -> None:
        """Default provider setting should point to isaacsim.core.experimental.primdata."""
        settings = carb.settings.get_settings()
        provider = settings.get("/exts/isaacsim.core.experimental.prims/prim_data_reader_provider_extension")
        self.assertEqual(provider, "isaacsim.core.experimental.primdata")

    async def test_provider_extension_enabled_for_reader(self) -> None:
        """Provider extension should be enabled before/while acquiring the reader interface."""
        app = omni.kit.app.get_app()
        ext_manager = app.get_extension_manager()
        settings = carb.settings.get_settings()
        provider = settings.get("/exts/isaacsim.core.experimental.prims/prim_data_reader_provider_extension")
        self.assertTrue(ext_manager.is_extension_enabled(provider))
        self.assertIsNotNone(self.reader)

    # -- View creation and type safety --

    async def test_create_articulation_view_has_correct_methods(self) -> None:
        """IArticulationDataView should expose DOF getters and inherited XformPrim getters."""
        view = self.reader.create_articulation_view("art_v", ["/World/test"], "physx")
        self.assertIsNotNone(view)
        for method in [
            "get_dof_positions",
            "get_dof_velocities",
            "get_dof_efforts",
            "get_root_transforms",
            "get_root_velocities",
            "get_world_positions",
            "get_world_orientations",
            "get_dof_index",
            "get_dof_names",
            "get_dof_types",
            "get_dof_types_host",
            "get_articulation_links",
            "update",
            "allocate_buffer",
            "get_buffer_ptr",
            "get_buffer_size",
            "get_buffer_device",
            "register_field_callback",
        ]:
            self.assertTrue(hasattr(view, method), f"Missing method: {method}")

    async def test_create_rigid_body_view_has_correct_methods(self) -> None:
        """IRigidBodyDataView should expose velocity/mass getters and inherited XformPrim getters."""
        view = self.reader.create_rigid_body_view("rb_v", ["/World/test"], "physx")
        self.assertIsNotNone(view)
        for method in [
            "get_linear_velocities",
            "get_angular_velocities",
            "get_world_positions",
            "update",
            "allocate_buffer",
        ]:
            self.assertTrue(hasattr(view, method), f"Missing method: {method}")
        self.assertFalse(hasattr(view, "get_dof_positions"))

    async def test_create_xform_view_has_correct_methods(self) -> None:
        """IXformDataView should expose transform getters only."""
        view = self.reader.create_xform_view("xf_v", ["/World/test"], "physx")
        self.assertIsNotNone(view)
        for method in [
            "get_world_positions",
            "get_world_orientations",
            "get_local_translations",
            "get_local_orientations",
            "get_local_scales",
            "update",
            "allocate_buffer",
            "get_prim_frame_name",
            "get_prim_world_transform",
        ]:
            self.assertTrue(hasattr(view, method), f"Missing method: {method}")
        self.assertFalse(hasattr(view, "get_linear_velocities"))
        self.assertFalse(hasattr(view, "get_dof_positions"))

    # -- Buffer allocation and pointer access --

    async def test_buffer_allocation_returns_valid_pointer(self) -> None:
        """allocate_buffer should produce a non-zero pointer and correct size/device."""
        view = self.reader.create_articulation_view("buf_test", ["/World/t"], "physx")
        view.allocate_buffer("field_a", 100, BufferDtype.FLOAT)

        ptr = view.get_buffer_ptr("field_a")
        self.assertIsInstance(ptr, int)
        self.assertGreater(ptr, 0)

        self.assertEqual(view.get_buffer_size("field_a"), 100)
        self.assertEqual(view.get_buffer_device(), -1)

    async def test_buffer_ptr_for_unknown_field_returns_zero(self) -> None:
        """Querying a non-existent field should return 0."""
        view = self.reader.create_articulation_view("buf_miss", ["/World/t"], "physx")
        self.assertEqual(view.get_buffer_ptr("no_such_field"), 0)
        self.assertEqual(view.get_buffer_size("no_such_field"), 0)

    async def test_buffer_wrapping_as_warp_array(self) -> None:
        """C++ buffer should be wrappable as a wp.array via the ptr constructor."""
        view = self.reader.create_articulation_view("wp_wrap", ["/World/t"], "physx")
        view.allocate_buffer("test_field", 30, BufferDtype.FLOAT)

        ptr = view.get_buffer_ptr("test_field")
        device_ord = view.get_buffer_device()
        device = f"cuda:{device_ord}" if device_ord >= 0 else "cpu"

        arr = wp.array(
            ptr=ptr,
            shape=(5, 6),
            dtype=wp.float32,
            device=device,
            capacity=30 * 4,
            deleter=None,
        )
        self.assertEqual(arr.shape, (5, 6))
        self.assertEqual(arr.dtype, wp.float32)

    async def test_allocate_buffer_uint8_via_dtype(self) -> None:
        """allocate_buffer(field_name, count, dtype='uint8') creates uint8 buffer."""
        view = self.reader.create_articulation_view("buf_u8", ["/World/t"], "physx")
        self.assertTrue(view.allocate_buffer("dof_types", 5, BufferDtype.UINT8))
        self.assertEqual(view.get_buffer_size("dof_types"), 5)
        self.assertGreater(view.get_buffer_ptr("dof_types"), 0)

    async def test_allocate_buffer_accepts_buffer_dtype_enum(self) -> None:
        """allocate_buffer accepts BufferDtype enum as well as string."""
        view = self.reader.create_articulation_view("buf_enum", ["/World/t"], "physx")
        self.assertTrue(view.allocate_buffer("field_f", 10, BufferDtype.FLOAT))
        self.assertEqual(view.get_buffer_size("field_f"), 10)
        self.assertTrue(view.allocate_buffer("field_u8", 5, BufferDtype.UINT8))
        self.assertEqual(view.get_buffer_size("field_u8"), 5)

    async def test_allocate_buffer_invalid_dtype_raises(self) -> None:
        """allocate_buffer with unsupported dtype string should raise."""
        view = self.reader.create_articulation_view("buf_bad", ["/World/t"], "physx")
        with self.assertRaises(ValueError):
            view.allocate_buffer("x", 10, "int32")

    # -- DOF names and types --

    async def test_get_dof_names_returns_list(self) -> None:
        """get_dof_names should return a list (may be empty if no articulation/DOFs)."""
        view = self.reader.create_articulation_view("dof_meta_v", ["/World/test"], "physx")
        self.assertIsNotNone(view)
        names = view.get_dof_names()
        self.assertIsInstance(names, list)

    async def test_get_dof_types_returns_tuple_ptr_count(self) -> None:
        """get_dof_types should return (ptr, count) like other buffer getters."""
        view = self.reader.create_articulation_view("dof_types_v", ["/World/test"], "physx")
        self.assertIsNotNone(view)
        result = view.get_dof_types()
        self.assertIsInstance(result, tuple)
        self.assertEqual(len(result), 2)
        ptr, count = result
        self.assertIsInstance(ptr, int)
        self.assertIsInstance(count, int)

    async def test_reader_has_set_articulation_dof_metadata(self) -> None:
        """Reader should expose set_articulation_dof_metadata for Newton backend."""
        self.assertTrue(hasattr(self.reader, "set_articulation_dof_metadata"))

    async def test_set_articulation_dof_metadata_then_get_dof_names_and_types(self) -> None:
        """After set_articulation_dof_metadata, get_dof_names and get_dof_types return that data (Newton path)."""
        import ctypes

        view_id = "dof_newton_meta"
        view = self.reader.create_articulation_view(view_id, ["/World/t"], "newton")
        self.assertIsNotNone(view)
        self.reader.set_articulation_dof_metadata(view_id, ["joint_a", "joint_b"], [0, 1])
        names = view.get_dof_names()
        ptr, count = view.get_dof_types()
        self.assertEqual(names, ["joint_a", "joint_b"])
        self.assertEqual(count, 2)
        if ptr and count >= 2:
            buf = (ctypes.c_uint8 * count).from_address(ptr)
            self.assertEqual(buf[0], 0)
            self.assertEqual(buf[1], 1)

    # -- Callback registration and invocation --

    async def test_callback_invoked_on_first_getter_call(self) -> None:
        """A registered callback should fire on the first getter call."""
        view = self.reader.create_articulation_view("cb_test", ["/World/t"], "newton")
        view.allocate_buffer("dof_positions", 10, BufferDtype.FLOAT)

        call_count = [0]

        def counter_callback() -> None:
            call_count[0] += 1

        view.register_field_callback("dof_positions", counter_callback)
        view.get_dof_positions()
        self.assertEqual(call_count[0], 1)

    async def test_callback_not_invoked_twice_same_step(self) -> None:
        """Within the same physics step, the callback should fire at most once."""
        view = self.reader.create_articulation_view("cb_dedup", ["/World/t"], "newton")
        view.allocate_buffer("dof_positions", 10, BufferDtype.FLOAT)

        call_count = [0]
        view.register_field_callback("dof_positions", lambda: call_count.__setitem__(0, call_count[0] + 1))

        view.get_dof_positions()
        view.get_dof_positions()
        self.assertEqual(call_count[0], 1)

    async def test_update_batch_prefetch_triggers_all_callbacks(self) -> None:
        """update() should trigger callbacks for all stale fields."""
        view = self.reader.create_articulation_view("cb_batch", ["/World/t"], "newton")
        view.allocate_buffer("dof_positions", 10, BufferDtype.FLOAT)
        view.allocate_buffer("dof_velocities", 10, BufferDtype.FLOAT)

        calls = {"pos": 0, "vel": 0}
        view.register_field_callback("dof_positions", lambda: calls.__setitem__("pos", calls["pos"] + 1))
        view.register_field_callback("dof_velocities", lambda: calls.__setitem__("vel", calls["vel"] + 1))

        view.update()
        self.assertEqual(calls["pos"], 1)
        self.assertEqual(calls["vel"], 1)

        # Subsequent getters should not re-trigger
        view.get_dof_positions()
        view.get_dof_velocities()
        self.assertEqual(calls["pos"], 1)
        self.assertEqual(calls["vel"], 1)

    # -- View lifecycle --

    async def test_view_creation_and_removal(self) -> None:
        """Views can be created and then cleanly removed."""
        self.reader.create_articulation_view("lifecycle_a", ["/World/t"], "physx")
        self.reader.create_rigid_body_view("lifecycle_b", ["/World/t"], "physx")
        self.reader.remove_view("lifecycle_a")
        self.reader.remove_view("lifecycle_b")

    async def test_shutdown_cleans_all_views(self) -> None:
        """shutdown() should destroy all views; subsequent create works after re-initialize."""
        self.reader.create_articulation_view("sd_test", ["/World/t"], "physx")
        self.reader.shutdown()
        self.reader.initialize(0, -1)
        view = self.reader.create_articulation_view("sd_test2", ["/World/t"], "physx")
        self.assertIsNotNone(view)

    async def test_manager_ensure_initialized_is_idempotent_for_same_stage(self) -> None:
        """Repeated ensure_initialized(stage, device) should not bump generation."""
        from isaacsim.core.experimental.prims import _prims_reader

        await stage_utils.create_new_stage_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.assertNotEqual(stage_id, 0)

        manager = _prims_reader.acquire_prim_data_reader_manager_interface()
        self.assertIsNotNone(manager)
        try:
            self.assertTrue(manager.ensure_initialized(stage_id, -1))
            gen_after_first = manager.get_generation()
            self.assertGreater(gen_after_first, 0)

            self.assertTrue(manager.ensure_initialized(stage_id, -1))
            gen_after_second = manager.get_generation()
            self.assertEqual(gen_after_first, gen_after_second)
        finally:
            _prims_reader.release_prim_data_reader_manager_interface(manager)
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()

    async def test_manager_ensure_initialized_is_stable_across_multiple_acquirers(self) -> None:
        """Multiple manager callers should share one initialization generation for a stage."""
        from isaacsim.core.experimental.prims import _prims_reader

        await stage_utils.create_new_stage_async()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        stage = omni.usd.get_context().get_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.assertNotEqual(stage_id, 0)

        manager_a = _prims_reader.acquire_prim_data_reader_manager_interface()
        manager_b = _prims_reader.acquire_prim_data_reader_manager_interface()
        self.assertIsNotNone(manager_a)
        self.assertIsNotNone(manager_b)
        try:
            self.assertTrue(manager_a.ensure_initialized(stage_id, -1))
            generation_after_first = manager_a.get_generation()
            self.assertGreater(generation_after_first, 0)

            self.assertTrue(manager_b.ensure_initialized(stage_id, -1))
            generation_after_second = manager_b.get_generation()
            self.assertEqual(generation_after_first, generation_after_second)
        finally:
            _prims_reader.release_prim_data_reader_manager_interface(manager_b)
            _prims_reader.release_prim_data_reader_manager_interface(manager_a)
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()

    # -- getArticulationLinks / getPrimFrameName / getPrimWorldTransform (on view interfaces) --

    async def test_xform_view_has_get_prim_frame_name(self) -> None:
        """IXformDataView should expose get_prim_frame_name."""
        view = self.reader.create_xform_view("v_framename", ["/World/t"], "physx")
        self.assertTrue(hasattr(view, "get_prim_frame_name"))

    async def test_xform_view_has_get_prim_world_transform(self) -> None:
        """IXformDataView should expose get_prim_world_transform."""
        view = self.reader.create_xform_view("v_worldxform", ["/World/t"], "physx")
        self.assertTrue(hasattr(view, "get_prim_world_transform"))

    async def test_articulation_view_has_get_articulation_links(self) -> None:
        """IArticulationDataView should expose get_articulation_links."""
        view = self.reader.create_articulation_view("v_artlinks", ["/World/t"], "physx")
        self.assertTrue(hasattr(view, "get_articulation_links"))

    # -- Behavior without a valid stage (stageId == 0) --

    async def test_get_prim_frame_name_without_stage_returns_none(self) -> None:
        """get_prim_frame_name should return None when no stage is loaded."""
        view = self.reader.create_xform_view("v_noframe", ["/World/t"], "physx")
        result = view.get_prim_frame_name("/World/Prim")
        self.assertIsNone(result)

    async def test_get_articulation_links_without_stage_returns_empty(self) -> None:
        """get_articulation_links should return an empty list when no stage is loaded."""
        view = self.reader.create_articulation_view("v_nolinks", ["/World/t"], "physx")
        result = view.get_articulation_links("/World/Robot")
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 0)

    async def test_get_prim_world_transform_without_stage_returns_none(self) -> None:
        """get_prim_world_transform should return None when no stage is loaded."""
        view = self.reader.create_xform_view("v_noworldxf", ["/World/t"], "physx")
        result = view.get_prim_world_transform("/World/Prim")
        self.assertIsNone(result)

    # -- getPrimFrameName functional tests --

    async def test_get_prim_frame_name_returns_prim_name(self) -> None:
        """get_prim_frame_name returns the prim's name for a valid prim path."""
        stage = await self._setup_stage()
        UsdGeom.Xform.Define(stage, "/World/MyRobot")
        view = self.reader.create_xform_view("v_framename2", ["/World/MyRobot"], "physx")
        result = view.get_prim_frame_name("/World/MyRobot")
        self.assertEqual(result, "MyRobot")

    async def test_get_prim_frame_name_on_missing_prim_returns_none(self) -> None:
        """get_prim_frame_name returns None for a path that does not exist in the stage."""
        stage = await self._setup_stage()
        UsdGeom.Xform.Define(stage, "/World/Exists")
        view = self.reader.create_xform_view("v_missingprim", ["/World/Exists"], "physx")
        result = view.get_prim_frame_name("/World/DoesNotExist")
        self.assertIsNone(result)

    async def test_get_prim_frame_name_respects_isaac_name_override(self) -> None:
        """get_prim_frame_name should return the isaac:nameOverride value when set."""
        from pxr import Sdf

        stage = await self._setup_stage()
        prim = UsdGeom.Xform.Define(stage, "/World/Link0").GetPrim()
        prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set("base_link")
        view = self.reader.create_xform_view("v_override", ["/World/Link0"], "physx")
        result = view.get_prim_frame_name("/World/Link0")
        self.assertEqual(result, "base_link")

    # -- getArticulationLinks functional tests --

    async def test_get_articulation_links_on_non_articulation_returns_empty(self) -> None:
        """get_articulation_links returns an empty list for a prim without ArticulationRootAPI."""
        stage = await self._setup_stage()
        UsdGeom.Xform.Define(stage, "/World/NotAnArticulation")
        view = self.reader.create_articulation_view("v_noart", ["/World/NotAnArticulation"], "physx")
        result = view.get_articulation_links("/World/NotAnArticulation")
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 0)

    async def test_get_articulation_links_returns_links_and_parent_hierarchy(self) -> None:
        """get_articulation_links enumerates links and resolves parent-child relationships."""
        stage = await self._setup_stage()

        # Two-link articulation: Robot (root) -> Link0 -> Link1
        root = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(root)

        link0 = UsdGeom.Xform.Define(stage, "/World/Robot/Link0").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link0)

        link1 = UsdGeom.Xform.Define(stage, "/World/Robot/Link0/Link1").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link1)

        view = self.reader.create_articulation_view("v_artlinks2", ["/World/Robot"], "physx")
        links = view.get_articulation_links("/World/Robot")
        self.assertIsInstance(links, list)
        self.assertEqual(len(links), 2)

        paths = {e["path"] for e in links}
        self.assertIn("/World/Robot/Link0", paths)
        self.assertIn("/World/Robot/Link0/Link1", paths)

        link0_entry = next(e for e in links if e["path"] == "/World/Robot/Link0")
        link1_entry = next(e for e in links if e["path"] == "/World/Robot/Link0/Link1")

        # Root link has no articulation-link ancestor → empty parent path
        self.assertEqual(link0_entry["parent_path"], "")
        # Link1's closest link ancestor is Link0
        self.assertEqual(link1_entry["parent_path"], "/World/Robot/Link0")

    async def test_get_articulation_links_on_missing_prim_returns_empty(self) -> None:
        """get_articulation_links returns an empty list for a path that does not exist."""
        await self._setup_stage()
        view = self.reader.create_articulation_view("v_noartprim", ["/World/t"], "physx")
        result = view.get_articulation_links("/World/NoSuchPrim")
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 0)

    # -- getPrimWorldTransform functional tests --

    async def test_get_prim_world_transform_returns_origin_for_default_xform(self) -> None:
        """A default Xform at the origin should report (0,0,0) position and identity orientation."""
        stage = await self._setup_stage()
        self.timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # Re-initialize to pick up the Fabric (usdrt) stage after timeline start
        self.reader.initialize(self._stage_id, -1)

        UsdGeom.Xform.Define(stage, "/World/Origin")
        view = self.reader.create_xform_view("v_origin", ["/World/Origin"], "physx")
        result = view.get_prim_world_transform("/World/Origin")
        self.assertIsNotNone(result)

        pos, ori = result
        self.assertEqual(len(pos), 3)
        self.assertEqual(len(ori), 4)

        self.assertAlmostEqual(pos[0], 0.0, places=5)
        self.assertAlmostEqual(pos[1], 0.0, places=5)
        self.assertAlmostEqual(pos[2], 0.0, places=5)

        # Identity quaternion: qw=1, qx=qy=qz=0
        self.assertAlmostEqual(abs(ori[0]), 1.0, places=5)
        self.assertAlmostEqual(ori[1], 0.0, places=5)
        self.assertAlmostEqual(ori[2], 0.0, places=5)
        self.assertAlmostEqual(ori[3], 0.0, places=5)

    async def test_get_prim_world_transform_on_missing_prim_returns_none(self) -> None:
        """get_prim_world_transform returns None for a path that does not exist."""
        stage = await self._setup_stage()
        UsdGeom.Xform.Define(stage, "/World/Exists")
        self.timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.reader.initialize(self._stage_id, -1)

        view = self.reader.create_xform_view("v_missingxf", ["/World/Exists"], "physx")
        result = view.get_prim_world_transform("/World/DoesNotExist")
        self.assertIsNone(result)


def _read_float_buffer(ptr: Any, count: Any) -> Any:
    """Read *count* floats from a raw C pointer into a Python list.

    Args:
        ptr: Value passed by the caller.
        count: Expected number of contact records.

    Returns:
        Requested value.
    """
    if not ptr or count <= 0:
        return []
    return list((ctypes.c_float * count).from_address(ptr))


async def _add_rigid_cube(path: Any, positions: Any, orientations: Any = None) -> None:
    """Create a rigid body cube at *path* using the experimental prim/object APIs.

    Args:
        path: USD prim path.
        positions: [x, y, z] world position.
        orientations: [w, x, y, z] quaternion or None for identity.
    """
    Cube(path, positions=positions, orientations=orientations)
    await omni.kit.app.get_app().next_update_async()
    RigidPrim(path, masses=1.0)
    await omni.kit.app.get_app().next_update_async()
    GeomPrim(path, apply_collision_apis=True)
    await omni.kit.app.get_app().next_update_async()


class TestPrimDataReaderPhysxTransforms(omni.kit.test.AsyncTestCase):
    """Tests verifying PhysX tensor-backed world transforms via the prim data reader.

    These exercise the bulk rigid body tensor read path added to _setupTransformCallbacks,
    ensuring numerical correctness, correct quaternion reordering, and proper handling of
    mixed physics / non-physics prim sets.
    """

    async def setUp(self) -> None:
        """Set up test environment."""
        self.timeline = omni.timeline.get_timeline_interface()
        self._stage_id = 0
        from isaacsim.core.experimental.prims.impl.extension import get_prim_data_reader

        self.reader = get_prim_data_reader()
        if self.reader is not None:
            self.reader.initialize(0, -1)

    async def tearDown(self) -> None:
        """Tear down test environment."""
        if self.reader is not None:
            self.reader.shutdown()
        if self.timeline.is_playing():
            self.timeline.stop()
            await omni.kit.app.get_app().next_update_async()

    async def _setup_physics_stage(self) -> Any:
        """Create a stage with a PhysicsScene and return it.

        Returns:
            Requested value.
        """
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        return stage

    async def _start_simulation(self) -> None:
        """Play timeline, step a few frames to let physics settle, reinit reader."""
        self.timeline.play()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        self.reader.initialize(self._stage_id, -1)

    # -- Numerical correctness for rigid bodies --

    async def test_rigid_body_world_positions_match_set_translation(self) -> None:
        """Rigid bodies at known positions should report matching world_positions via tensor path."""
        await self._setup_physics_stage()
        await _add_rigid_cube("/World/CubeA", [2.0, 3.0, 4.0])
        await _add_rigid_cube("/World/CubeB", [-1.0, 0.0, 5.0])

        await self._start_simulation()

        view = self.reader.create_xform_view("test_rb_pos", ["/World/CubeA", "/World/CubeB"], "physx")
        ptr, count = view.get_world_positions_host()
        self.assertEqual(count, 6)
        positions = _read_float_buffer(ptr, count)

        self.assertAlmostEqual(positions[0], 2.0, delta=0.2, msg="CubeA x")
        self.assertAlmostEqual(positions[1], 3.0, delta=0.2, msg="CubeA y")
        self.assertAlmostEqual(positions[2], 4.0, delta=0.5, msg="CubeA z (may settle due to gravity)")
        self.assertAlmostEqual(positions[3], -1.0, delta=0.2, msg="CubeB x")
        self.assertAlmostEqual(positions[4], 0.0, delta=0.2, msg="CubeB y")

    async def test_rigid_body_world_orientations_identity(self) -> None:
        """Rigid bodies with no rotation should report identity quaternion (qw=1,qx=qy=qz=0)."""
        await self._setup_physics_stage()
        await _add_rigid_cube("/World/Cube", [0.0, 0.0, 2.0])

        await self._start_simulation()

        view = self.reader.create_xform_view("test_rb_ori_id", ["/World/Cube"], "physx")
        ptr, count = view.get_world_orientations_host()
        self.assertEqual(count, 4)
        ori = _read_float_buffer(ptr, count)

        self.assertAlmostEqual(abs(ori[0]), 1.0, delta=0.01, msg="qw should be ~1")
        self.assertAlmostEqual(ori[1], 0.0, delta=0.01, msg="qx should be ~0")
        self.assertAlmostEqual(ori[2], 0.0, delta=0.01, msg="qy should be ~0")
        self.assertAlmostEqual(ori[3], 0.0, delta=0.01, msg="qz should be ~0")

    async def test_rigid_body_world_orientations_rotated(self) -> None:
        """Rigid body with a known rotation should report matching quaternion via tensor path."""
        import math

        await self._setup_physics_stage()
        angle_deg = 90.0
        half = math.radians(angle_deg) / 2.0
        qw, qx, qy, qz = math.cos(half), 0.0, 0.0, math.sin(half)
        await _add_rigid_cube("/World/RotCube", [0.0, 0.0, 2.0], orientations=[qw, qx, qy, qz])

        await self._start_simulation()

        view = self.reader.create_xform_view("test_rb_ori_rot", ["/World/RotCube"], "physx")
        ptr, count = view.get_world_orientations_host()
        ori = _read_float_buffer(ptr, count)

        norm_sq = sum(v * v for v in ori)
        self.assertAlmostEqual(norm_sq, 1.0, delta=0.01, msg="Quaternion should be unit length")

        self.assertAlmostEqual(abs(ori[0]), abs(qw), delta=0.05, msg="qw component")
        self.assertAlmostEqual(abs(ori[3]), abs(qz), delta=0.05, msg="qz component (90-deg Z rotation)")

    # -- Hybrid path: mix of physics and non-physics prims --

    async def test_mixed_physics_and_xform_prims(self) -> None:
        """An xform view containing both rigid bodies and plain xforms returns correct positions for all."""
        await self._setup_physics_stage()
        await _add_rigid_cube("/World/RigidCube", [5.0, 0.0, 2.0])
        stage_utils.define_prim("/World/PlainXform", "Xform")
        XformPrim("/World/PlainXform", positions=[10.0, 20.0, 30.0], reset_xform_op_properties=True)
        await omni.kit.app.get_app().next_update_async()

        await self._start_simulation()

        view = self.reader.create_xform_view("test_mixed", ["/World/RigidCube", "/World/PlainXform"], "physx")
        ptr, count = view.get_world_positions_host()
        self.assertEqual(count, 6)
        positions = _read_float_buffer(ptr, count)

        self.assertAlmostEqual(positions[0], 5.0, delta=0.2, msg="RigidCube x (tensor path)")
        self.assertAlmostEqual(positions[3], 10.0, delta=0.01, msg="PlainXform x (Fabric path)")
        self.assertAlmostEqual(positions[4], 20.0, delta=0.01, msg="PlainXform y (Fabric path)")
        self.assertAlmostEqual(positions[5], 30.0, delta=0.01, msg="PlainXform z (Fabric path)")

    async def test_mixed_physics_and_xform_orientations(self) -> None:
        """Mixed physics + non-physics prims all report valid orientations."""
        await self._setup_physics_stage()
        await _add_rigid_cube("/World/RigidCube", [0.0, 0.0, 2.0])
        stage_utils.define_prim("/World/PlainXform", "Xform")
        await omni.kit.app.get_app().next_update_async()

        await self._start_simulation()

        view = self.reader.create_xform_view("test_mixed_ori", ["/World/RigidCube", "/World/PlainXform"], "physx")
        ptr, count = view.get_world_orientations_host()
        self.assertEqual(count, 8)
        ori = _read_float_buffer(ptr, count)

        for i in range(2):
            norm_sq = sum(ori[i * 4 + c] ** 2 for c in range(4))
            self.assertAlmostEqual(norm_sq, 1.0, delta=0.01, msg=f"Prim {i} quaternion should be unit length")

    # -- Tensor vs Fabric comparison --

    async def test_tensor_matches_per_prim_fabric_transform(self) -> None:
        """Bulk tensor world positions/orientations match per-prim get_prim_world_transform (Fabric)."""
        await self._setup_physics_stage()
        paths = [f"/World/Cube_{i}" for i in range(3)]
        for i, p in enumerate(paths):
            await _add_rigid_cube(p, [float(i) * 3.0, 0.0, 2.0])

        await self._start_simulation()

        view = self.reader.create_xform_view("test_tensor_vs_fabric", paths, "physx")

        pos_ptr, pos_count = view.get_world_positions_host()
        ori_ptr, ori_count = view.get_world_orientations_host()
        bulk_positions = _read_float_buffer(pos_ptr, pos_count)
        bulk_orientations = _read_float_buffer(ori_ptr, ori_count)

        for i, p in enumerate(paths):
            result = view.get_prim_world_transform(p)
            self.assertIsNotNone(result, f"get_prim_world_transform returned None for {p}")
            fabric_pos, fabric_ori = result

            for c in range(3):
                self.assertAlmostEqual(
                    bulk_positions[i * 3 + c],
                    fabric_pos[c],
                    delta=0.01,
                    msg=f"{p} position[{c}]: tensor={bulk_positions[i*3+c]} vs fabric={fabric_pos[c]}",
                )

            bulk_q = bulk_orientations[i * 4 : i * 4 + 4]
            sign = 1.0 if bulk_q[0] * fabric_ori[0] >= 0 else -1.0
            for c in range(4):
                self.assertAlmostEqual(
                    bulk_q[c],
                    sign * fabric_ori[c],
                    delta=0.01,
                    msg=f"{p} orientation[{c}]: tensor={bulk_q[c]} vs fabric={fabric_ori[c]}",
                )

    # -- Multiple rigid bodies scatter correctness --

    async def test_many_rigid_bodies_all_positions_valid(self) -> None:
        """With many rigid bodies, all world positions should be finite and near their initial positions."""
        import math

        await self._setup_physics_stage()
        n = 8
        paths = []
        for i in range(n):
            p = f"/World/Body_{i}"
            await _add_rigid_cube(p, [float(i) * 2.0, 0.0, 2.0])
            paths.append(p)

        await self._start_simulation()

        view = self.reader.create_xform_view("test_many_rb", paths, "physx")
        ptr, count = view.get_world_positions_host()
        self.assertEqual(count, n * 3)
        positions = _read_float_buffer(ptr, count)

        for i in range(n):
            for c in range(3):
                self.assertTrue(
                    math.isfinite(positions[i * 3 + c]),
                    f"Body_{i} position[{c}] is not finite: {positions[i*3+c]}",
                )
            self.assertAlmostEqual(positions[i * 3 + 0], float(i) * 2.0, delta=0.5, msg=f"Body_{i} x")
