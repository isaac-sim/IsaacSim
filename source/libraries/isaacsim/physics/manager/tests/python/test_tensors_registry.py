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

"""Verify tensor registry, dispatch, ownership, and error contracts.

These tests use mock factories and CPU Warp arrays to exercise the native
registry and its Python dispatch layer independently of a physics engine.
"""

from __future__ import annotations

import _physics_setup  # noqa: F401  -- registers the ovphysx backend at import
import isaacsim.physics.manager.impl.tensors as t
import numpy as np
import pytest
import warp as wp


def _asarray(x: object, dtype: str | None = None) -> np.ndarray | None:
    """Convert a Warp array, DLPack capsule, or NumPy array to NumPy.

    Args:
        x: Array-compatible object, or None.
        dtype: Optional NumPy data type name for the result.

    Returns:
        NumPy array with the requested data type, or None for a None input.

    """
    if x is None:
        return None
    if isinstance(x, wp.array):
        arr = x.numpy()
    else:
        try:
            arr = wp.from_dlpack(x).numpy()
        except Exception:
            arr = x  # already a numpy array
    return arr.astype(dtype) if dtype is not None else arr


def teardown_module() -> None:
    """Restore production tensor registry state after TensorsRegistryTestCase clears it.

    ``TensorsRegistryTestCase`` calls ``clear_for_testing()``, which wipes every entity factory; cycle each
    engine backend so later tests in the process still find their factories registered.
    """
    # Cycle the ovphysx backend to re-register its entity factories.
    import isaacsim.physics_engines.ovphysx as _ovphysx

    _ovphysx.shutdown()
    _ovphysx.activate()

    # Re-register Newton's tensor factories. `register_with_umbrella` is a no-op for a name it already
    # registered, so clear the guard it keys on first -- the registry itself no longer holds those entries.
    from isaacsim.physics_engines.ovnewton.impl.tensors import entity_factories as _newton_factories

    _newton_factories._REGISTERED.clear()
    _newton_factories.register_with_umbrella()


class TensorsRegistryTestCase:
    """Common base that wipes the registry on entry/exit so test order doesn't matter."""

    def setup_method(self) -> None:
        """Clear all tensor factories before each test."""
        t.get_registry().clear_for_testing()

    def teardown_method(self) -> None:
        """Clear all tensor factories after each test."""
        t.get_registry().clear_for_testing()


class TestRegistryRoundTrip(TensorsRegistryTestCase):
    """Verify entity-factory registration and creation."""

    def test_register_and_create_entity(self) -> None:
        """Verify registration and creation with preserved entity paths."""

        # Engine factory returns a plain EntityView with the requested paths.
        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            return t.EntityView(paths)

        reg = t.get_registry()
        was_new = reg.register_entity("mock", "articulation", factory)
        assert was_new

        view = t.create_entity("mock", "articulation", ["/World/A", "/World/B"])
        assert view is not None
        assert list(view.paths) == ["/World/A", "/World/B"]

    def test_register_replaces_existing(self) -> None:
        """Verify that a duplicate entity key replaces its factory."""
        reg = t.get_registry()
        reg.register_entity("mock", "articulation", lambda paths, options=None: t.EntityView(paths))
        # Re-registering returns False (replaced rather than added).
        was_new = reg.register_entity("mock", "articulation", lambda paths, options=None: t.EntityView(paths))
        assert not (was_new)

    def test_unregister_entity(self) -> None:
        """Verify entity-factory removal and idempotent missing removal."""
        reg = t.get_registry()
        reg.register_entity("mock", "articulation", lambda paths, options=None: t.EntityView(paths))
        assert reg.has_entity("mock", "articulation")
        assert reg.unregister_entity("mock", "articulation")
        assert not (reg.has_entity("mock", "articulation"))
        assert not (reg.unregister_entity("mock", "articulation"))

    def test_module_level_helpers(self) -> None:
        """Verify module-level registration and creation helpers."""
        # Module-level register_entity/create_entity mirror the registry.
        t.register_entity("mock", "rigid_body", lambda paths, options=None: t.EntityView(paths))
        view = t.create_entity("mock", "rigid_body", ["/World/Cube"])
        assert list(view.paths) == ["/World/Cube"]


class TestPerEngineIsolation(TensorsRegistryTestCase):
    """Verify isolation of equal entity names across engine keys."""

    def test_two_engines_independent(self) -> None:
        """Verify independent factories, views, enumeration, and removal."""
        reg = t.get_registry()

        def newton_factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            v = t.EntityView(paths)
            v.count = 7
            return v

        def ovphysx_factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            v = t.EntityView(paths)
            v.count = 11
            return v

        reg.register_entity("newton", "articulation", newton_factory)
        reg.register_entity("ovphysx", "articulation", ovphysx_factory)

        nv = t.create_entity("newton", "articulation", ["/World/X"])
        ov = t.create_entity("ovphysx", "articulation", ["/World/X"])

        assert nv.count == 7
        assert ov.count == 11

        # Listing simulations and entities surfaces both registrations.
        engines = sorted(reg.list_simulations())
        assert engines == ["newton", "ovphysx"]

        # Removing one leaves the other intact.
        reg.unregister_entity("newton", "articulation")
        assert not (reg.has_entity("newton", "articulation"))
        assert reg.has_entity("ovphysx", "articulation")


class TestImplDispatch(TensorsRegistryTestCase):
    """Verify tensor get/set dispatch and result ownership."""

    def _make_view_with_dof_position(self, *, indexed_read: bool = False) -> t.EntityView:
        view = t.EntityView(["/World/Robot"])
        view.count = 1

        # Backing buffer for the engine — the impl returns a view onto it.
        backing = wp.zeros((1, 4), dtype=wp.float32, device="cpu").numpy()

        def get_dof_position(indices: object | None, out: object | None) -> np.ndarray:
            del out
            if indices is not None:
                rows = _asarray(indices, dtype="int32")
                return backing[rows, :].copy()
            return backing

        def set_dof_position(data: object, indices: object | None) -> None:
            arr = _asarray(data, dtype="float32")
            if indices is not None:
                rows = _asarray(indices, dtype="int32")
                backing[rows, :] = arr
            else:
                backing[...] = arr

        spec_get = t.TensorSpec(
            dtype=t.DType.FLOAT32,
            shape_hint=[1, 4],
            device_kind=t.DeviceKind.CPU,
            supports_indexed_read=indexed_read,
        )
        spec_set = t.TensorSpec(
            dtype=t.DType.FLOAT32,
            shape_hint=[1, 4],
            device_kind=t.DeviceKind.CPU,
            supports_indexed_write=True,
        )

        view._register_impl("dof-position", t.ImplKind.Get, get_dof_position, spec_get)
        view._register_impl("dof-position", t.ImplKind.Set, set_dof_position, spec_set)
        view._backing = backing  # keep alive for the test
        return view

    def test_round_trip_read_then_write(self) -> None:
        """Verify full-buffer read, write, and readback."""
        view = self._make_view_with_dof_position()
        out = _asarray(view.get_data("dof-position"))
        assert out is not None
        assert out.tolist() == [[0.0, 0.0, 0.0, 0.0]]

        new_data = wp.array([[1.0, 2.0, 3.0, 4.0]], dtype=wp.float32, device="cpu")
        view.set_data("dof-position", new_data)

        out2 = _asarray(view.get_data("dof-position"))
        assert out2.tolist() == [[1.0, 2.0, 3.0, 4.0]]

    def test_indexed_write(self) -> None:
        """Verify an indexed write updates only the selected row."""
        view = self._make_view_with_dof_position()
        view._backing[...] = 0.0

        view.set_data(
            "dof-position",
            wp.array([[9.0, 9.0, 9.0, 9.0]], dtype=wp.float32, device="cpu"),
            indices=wp.array([0], dtype=wp.int32, device="cpu"),
        )
        assert view._backing.tolist() == [[9.0, 9.0, 9.0, 9.0]]

    def test_indexed_read_when_supported(self) -> None:
        """Verify indexed reads when the implementation advertises support."""
        view = self._make_view_with_dof_position(indexed_read=True)
        view._backing[...] = [[1.0, 2.0, 3.0, 4.0]]
        out = _asarray(view.get_data("dof-position", indices=wp.array([0], dtype=wp.int32, device="cpu")))
        assert out.tolist() == [[1.0, 2.0, 3.0, 4.0]]

    def test_indexed_read_rejected_when_not_supported(self) -> None:
        """Verify rejection of indexed reads without advertised support."""
        # Default spec disables indexed reads.
        view = self._make_view_with_dof_position(indexed_read=False)
        with pytest.raises(Exception):
            view.get_data("dof-position", indices=wp.array([0], dtype=wp.int32, device="cpu"))

    def test_get_result_pins_fresh_backing(self) -> None:
        """Verify that a get result retains its freshly allocated backing array.

        The returned Warp array must remain valid after garbage collection and
        same-sized allocation pressure.
        """
        import gc

        import numpy as np

        wp.init()  # this test's first warp call is from_dlpack, which won't auto-init

        view = t.EntityView(["/World/Robot"])
        view.count = 1

        def get_fresh(indices: object | None, out: object | None) -> np.ndarray:
            del indices, out
            # Fresh array each call -> wrapPyGet's local is the only reference.
            return np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)

        view._register_impl(
            "fresh",
            t.ImplKind.Get,
            get_fresh,
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1, 4], device_kind=t.DeviceKind.CPU),
        )

        # get_data on a raw EntityView returns a single-use DLPack capsule;
        # import it into a warp array once (which retains the keepalive chain),
        # then drop the capsule so only the pin keeps the fresh backing alive.
        warr = wp.from_dlpack(view.get_data("fresh"))  # C++ getData -> wrapPyGet -> pinned desc
        assert warr.numpy().tolist() == [[1.0, 2.0, 3.0, 4.0]]

        # GC + reuse the freed slab with same-size arrays; a dangling result
        # (no keepalive) would now read clobbered memory.
        gc.collect()
        _pressure = [np.full((1, 4), 7.0, dtype=np.float32) for _ in range(4096)]
        assert warr.numpy().tolist() == [
            [1.0, 2.0, 3.0, 4.0]
        ], "wrapPyGet result dangled: the fresh backing was freed and clobbered"

    def _assert_result_survives_pressure(
        self,
        view: t.EntityView,
        impl: str,
        out: np.ndarray | None = None,
    ) -> None:
        """Verify a raw get result remains valid under allocation pressure.

        Args:
            view: Entity view that owns the get implementation.
            impl: Registered get implementation name.
            out: Optional caller-provided output buffer.

        """
        import gc

        import numpy as np

        warr = wp.from_dlpack(view.get_data(impl, out=out) if out is not None else view.get_data(impl))
        assert warr.numpy().tolist() == [[1.0, 2.0, 3.0, 4.0]]
        out = None  # drop the only reference to the caller out (case A)
        gc.collect()
        _pressure = [np.full((1, 4), 7.0, dtype=np.float32) for _ in range(4096)]
        assert warr.numpy().tolist() == [
            [1.0, 2.0, 3.0, 4.0]
        ], f"{impl} result dangled: its backing was freed and clobbered"
        # Release the result chain so its keepalive (an nb::ndarray owner) is
        # collected within the test, not at interpreter teardown.
        del warr, _pressure
        gc.collect()

    def _fresh_view(self) -> t.EntityView:
        """Create a one-entity view for ownership tests.

        Returns:
            Unregistered entity view representing one entity.

        """
        view = t.EntityView(["/World/Robot"])
        view.count = 1
        return view

    def test_get_result_pins_returned_out(self) -> None:
        """Verify retention when a get implementation returns the caller's output."""
        import numpy as np

        wp.init()
        view = self._fresh_view()
        view._register_impl(
            "echo-out",
            t.ImplKind.Get,
            lambda indices, out: out,
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1, 4], device_kind=t.DeviceKind.CPU),
        )
        # The array passed as `out` is the SOLE reference to the caller storage;
        # the helper drops it before the GC/pressure so only the binding's
        # retention (of the caller's real out) can keep the result alive.
        self._assert_result_survives_pressure(view, "echo-out", out=np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32))

    def test_get_result_pins_desc_ndarray(self) -> None:
        """Verify that a tensor descriptor retains its wrapped NumPy array."""
        import numpy as np

        wp.init()
        view = self._fresh_view()
        view._register_impl(
            "desc-ndarray",
            t.ImplKind.Get,
            lambda indices, out: t.TensorDesc(np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)),
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1, 4], device_kind=t.DeviceKind.CPU),
        )
        self._assert_result_survives_pressure(view, "desc-ndarray")

    def test_get_result_pins_dlpack_capsule(self) -> None:
        """Verify that a raw DLPack result retains its imported managed tensor."""
        import numpy as np

        wp.init()
        view = self._fresh_view()
        view._register_impl(
            "dlpack-capsule",
            t.ImplKind.Get,
            lambda indices, out: np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32).__dlpack__(),
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1, 4], device_kind=t.DeviceKind.CPU),
        )
        self._assert_result_survives_pressure(view, "dlpack-capsule")

    def test_get_multi_result_pins_fresh_backing(self) -> None:
        """Verify ownership of fresh NumPy and DLPack outputs from a multi-get."""
        import gc

        import numpy as np

        wp.init()
        view = self._fresh_view()
        view._register_multi_impl(
            "multi-fresh",
            t.ImplKind.Get,
            lambda indices, out: [
                np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32),
                np.array([[5.0, 6.0, 8.0, 9.0]], dtype=np.float32).__dlpack__(),
            ],
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1, 4], device_kind=t.DeviceKind.CPU),
        )
        results = view.get_data_multi("multi-fresh")
        warrs = [wp.from_dlpack(r) for r in results]
        assert warrs[0].numpy().tolist() == [[1.0, 2.0, 3.0, 4.0]]
        assert warrs[1].numpy().tolist() == [[5.0, 6.0, 8.0, 9.0]]
        del results
        gc.collect()
        _pressure = [np.full((1, 4), 7.0, dtype=np.float32) for _ in range(4096)]
        assert warrs[0].numpy().tolist() == [[1.0, 2.0, 3.0, 4.0]], "multi output 0 (ndarray) dangled"
        assert warrs[1].numpy().tolist() == [[5.0, 6.0, 8.0, 9.0]], "multi output 1 (dlpack) dangled"
        del warrs, _pressure
        gc.collect()

    def test_desc_to_array_pins_backing(self) -> None:
        """Verify that descriptor conversion retains a temporary NumPy backing."""
        import gc

        import numpy as np

        wp.init()

        # The TensorDesc and its source ndarray are both temporaries: after
        # to_array() returns, only the returned array's own keepalive can keep the
        # backing alive.
        warr = wp.from_dlpack(t.TensorDesc(np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)).to_array())
        assert warr.numpy().tolist() == [[1.0, 2.0, 3.0, 4.0]]

        gc.collect()
        _pressure = [np.full((1, 4), 7.0, dtype=np.float32) for _ in range(4096)]
        assert warr.numpy().tolist() == [
            [1.0, 2.0, 3.0, 4.0]
        ], "TensorDesc.to_array() result dangled: the source ndarray was freed and clobbered"
        del warr, _pressure
        gc.collect()


class TestRawIndexDtypeGuard(TensorsRegistryTestCase):
    """Verify index data-type guards on raw entity-view entry points.

    Integer indices must use 32-bit elements. Floating-point payloads remain
    valid because signed-distance-field queries use the same argument slot.
    """

    def _view(self) -> t.EntityView:
        """Create a raw entity view with single and multi implementations.

        Returns:
            Two-entity view backed by a CPU NumPy array.

        """
        import numpy as np

        wp.init()
        view = t.EntityView(["/World/Robot"])
        view.count = 2
        backing = np.zeros((2, 4), dtype=np.float32)

        def get_op(indices: object | None, out: object | None) -> np.ndarray:
            del out
            if indices is not None:
                rows = _asarray(indices)
                if rows.dtype.kind in ("i", "u"):
                    return backing[rows.reshape(-1), :].copy()
            return backing  # None or float payload -> full buffer

        gspec = t.TensorSpec(
            dtype=t.DType.FLOAT32, shape_hint=[2, 4], device_kind=t.DeviceKind.CPU, supports_indexed_read=True
        )
        sspec = t.TensorSpec(
            dtype=t.DType.FLOAT32, shape_hint=[2, 4], device_kind=t.DeviceKind.CPU, supports_indexed_write=True
        )
        view._register_impl("op", t.ImplKind.Get, get_op, gspec)
        view._register_impl("op", t.ImplKind.Set, lambda data, indices: None, sspec)
        view._register_multi_impl("op-multi", t.ImplKind.Get, lambda indices, out: [backing], gspec)
        view._register_multi_impl("op-multi", t.ImplKind.Set, lambda data, indices: None, sspec)
        view._backing = backing
        return view

    def _int64(self) -> wp.array:
        """Create an invalid 64-bit integer index buffer.

        Returns:
            CPU Warp array containing one 64-bit index.

        """
        return wp.array([0], dtype=wp.int64, device="cpu")

    def test_raw_get_rejects_int64(self) -> None:
        """Verify that a raw single-get rejects 64-bit integer indices."""
        with pytest.raises(ValueError, match="int32"):
            self._view().get_data("op", self._int64())

    def test_raw_set_rejects_int64(self) -> None:
        """Verify that a raw single-set rejects 64-bit integer indices."""
        with pytest.raises(ValueError, match="int32"):
            self._view().set_data("op", wp.zeros((1, 4), dtype=wp.float32, device="cpu"), self._int64())

    def test_raw_get_multi_rejects_int64(self) -> None:
        """Verify that a raw multi-get rejects 64-bit integer indices."""
        with pytest.raises(ValueError, match="int32"):
            self._view().get_data_multi("op-multi", self._int64())

    def test_raw_set_multi_rejects_int64(self) -> None:
        """Verify that a raw multi-set rejects 64-bit integer indices."""
        with pytest.raises(ValueError, match="int32"):
            self._view().set_data_multi("op-multi", [wp.zeros((2, 4), dtype=wp.float32, device="cpu")], self._int64())

    def test_raw_get_allows_int32(self) -> None:
        """Verify that a raw get accepts 32-bit integer indices."""
        result = self._view().get_data("op", wp.array([0], dtype=wp.int32, device="cpu"))
        assert result is not None

    def test_raw_get_allows_float_sdf_payload(self) -> None:
        """Verify that a raw get accepts floating-point SDF query payloads."""
        # A float indices payload is the SDF query-point path, not an index array;
        # the guard must let it through (any downstream error is not the int32 one).
        try:
            self._view().get_data("op", wp.array([0.0], dtype=wp.float32, device="cpu"))
        except ValueError as e:
            assert "int32" not in str(e), "float indices payload wrongly rejected as non-int32"


class TestDiscoverability(TensorsRegistryTestCase):
    """Verify implementation enumeration and specification lookup."""

    def test_list_and_has_impl(self) -> None:
        """Verify get/set enumeration and support queries."""
        view = t.EntityView([])
        view._register_impl(
            "dof-position",
            t.ImplKind.Get,
            lambda i, o: wp.zeros((1,), dtype=wp.float32, device="cpu"),
            t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[1]),
        )
        # No Set impl registered.
        assert view.list_impls(t.ImplKind.Get) == ["dof-position"]
        assert view.list_impls(t.ImplKind.Set) == []
        assert view.has_impl("dof-position", t.ImplKind.Get)
        assert not (view.has_impl("dof-position", t.ImplKind.Set))

    def test_supports_false_makes_has_impl_false(self) -> None:
        """Verify that an unsupported registration remains listed but unavailable."""
        # An impl registered with supports=False is queryable but `has_impl`
        # reports False — the API surface stays uniform without false claims.
        view = t.EntityView([])
        view._register_impl(
            "fixed-tendon-stiffness",
            t.ImplKind.Get,
            lambda i, o: None,
            t.TensorSpec(supports=False),
        )
        assert "fixed-tendon-stiffness" in view.list_impls(t.ImplKind.Get)
        assert not (view.has_impl("fixed-tendon-stiffness", t.ImplKind.Get))

    def test_get_impl_spec_round_trip(self) -> None:
        """Verify that implementation specification fields round-trip."""
        view = t.EntityView([])
        spec = t.TensorSpec(
            dtype=t.DType.FLOAT64,
            shape_hint=[3, 4],
            device_kind=t.DeviceKind.GPU,
            supports_indexed_read=True,
            supports_indexed_write=True,
            supports_masked_write=True,
        )
        view._register_impl("dof-position", t.ImplKind.Get, lambda i, o: None, spec)
        got = view.get_impl_spec("dof-position", t.ImplKind.Get)
        assert got.dtype == t.DType.FLOAT64
        assert list(got.shape_hint) == [3, 4]
        assert got.device_kind == t.DeviceKind.GPU
        assert got.supports_indexed_read
        assert got.supports_indexed_write
        assert got.supports_masked_write


class TestErrorPaths(TensorsRegistryTestCase):
    """Verify registry and dispatch failures for invalid operations."""

    def test_get_unknown_impl_raises(self) -> None:
        """Verify that reading an unknown implementation raises."""
        view = t.EntityView([])
        with pytest.raises(Exception):
            view.get_data("does-not-exist")

    def test_set_unknown_impl_raises(self) -> None:
        """Verify that writing an unknown implementation raises."""
        view = t.EntityView([])
        with pytest.raises(Exception):
            view.set_data("does-not-exist", wp.zeros((1,), dtype=wp.float32, device="cpu"))

    def test_create_unknown_engine_raises(self) -> None:
        """Verify that entity creation for an unknown engine raises.

        The paths are non-empty so the engine lookup is what fails; an empty list is rejected earlier and
        would let this pass without ever reaching the registry.
        """
        with pytest.raises(IndexError):
            t.create_entity("nonexistent", "articulation", ["/World/X"])

    def test_supports_false_raises_on_call(self) -> None:
        """Verify that calling an explicitly unsupported implementation raises."""
        view = t.EntityView([])
        view._register_impl(
            "no-go",
            t.ImplKind.Get,
            lambda i, o: wp.zeros((1,), dtype=wp.float32, device="cpu"),
            t.TensorSpec(supports=False),
        )
        with pytest.raises(
            RuntimeError, match="EntityView::getData: implementation 'no-go' is registered but reports supports=false"
        ):
            view.get_data("no-go")

    def test_python_dispatch_supports_false_does_not_call_single(self) -> None:
        """Verify that unsupported single-get dispatch does not invoke its callback."""
        calls = []
        view = t.EntityView([])
        view._register_impl(
            "no-go",
            t.ImplKind.Get,
            lambda indices, out: calls.append((indices, out)),
            t.TensorSpec(supports=False),
        )
        t._attach_warp_dispatch(view)

        with pytest.raises(
            RuntimeError, match="EntityView::getData: implementation 'no-go' is registered but reports supports=false"
        ):
            view.get_data("no-go")
        assert calls == []

    def test_python_dispatch_uses_registered_spec_copy(self) -> None:
        """Verify that dispatch retains a copy of the registered specification."""
        calls = []
        spec = t.TensorSpec(supports=False)
        view = t.EntityView([])
        view._register_impl(
            "no-go",
            t.ImplKind.Get,
            lambda indices, out: calls.append((indices, out)),
            spec,
        )
        spec.supports = True
        t._attach_warp_dispatch(view)

        assert not (view.has_impl("no-go", t.ImplKind.Get))
        with pytest.raises(
            RuntimeError, match="EntityView::getData: implementation 'no-go' is registered but reports supports=false"
        ):
            view.get_data("no-go")
        assert calls == []

    def test_python_dispatch_rejects_unadvertised_indexed_single_set(self) -> None:
        """Verify rejection of indexed single-set without advertised support."""
        calls = []
        view = t.EntityView([])
        view._register_impl(
            "plain-set",
            t.ImplKind.Set,
            lambda data, indices: calls.append((data, indices)),
            t.TensorSpec(supports=True, supports_indexed_write=False),
        )
        t._attach_warp_dispatch(view)

        with pytest.raises(
            ValueError, match="EntityView::setData: implementation 'plain-set' does not support indexed writes"
        ):
            view.set_data(
                "plain-set",
                wp.zeros((1,), dtype=wp.float32, device="cpu"),
                wp.array([0], dtype=wp.int32, device="cpu"),
            )
        assert calls == []

    def test_python_dispatch_supports_false_does_not_call_multi_set(self) -> None:
        """Verify that unsupported multi-set dispatch does not invoke its callback."""
        calls = []
        view = t.EntityView([])
        view._register_multi_impl(
            "no-go-multi",
            t.ImplKind.Set,
            lambda data, indices: calls.append((data, indices)),
            t.TensorSpec(supports=False),
        )
        t._attach_warp_dispatch(view)

        with pytest.raises(
            RuntimeError,
            match="EntityView::setDataMulti: implementation 'no-go-multi' is registered but reports supports=false",
        ):
            view.set_data_multi(
                "no-go-multi",
                [wp.zeros((1,), dtype=wp.float32, device="cpu")],
            )
        assert calls == []

    def test_python_dispatch_rejects_unadvertised_indexed_multi_get(self) -> None:
        """Verify rejection of indexed multi-get without advertised support."""
        calls = []
        view = t.EntityView([])
        view._register_multi_impl(
            "plain-multi-get",
            t.ImplKind.Get,
            lambda indices, out: calls.append((indices, out)),
            t.TensorSpec(supports=True, supports_indexed_read=False),
        )
        t._attach_warp_dispatch(view)

        with pytest.raises(
            ValueError,
            match="EntityView::getDataMulti: implementation 'plain-multi-get' does not support indexed reads",
        ):
            view.get_data_multi(
                "plain-multi-get",
                wp.array([0], dtype=wp.int32, device="cpu"),
            )
        assert calls == []

    def test_cross_cardinality_get_registration_is_rejected(self) -> None:
        """Verify rejection of equal get names across single and multi cardinality."""
        view = t.EntityView([])
        view._register_impl(
            "shared-name",
            t.ImplKind.Get,
            lambda indices, out: wp.zeros((1,), dtype=wp.float32, device="cpu"),
            t.TensorSpec(supports=True),
        )
        with pytest.raises(ValueError):
            view._register_multi_impl(
                "shared-name",
                t.ImplKind.Get,
                lambda indices, out: [],
                t.TensorSpec(supports=True),
            )

        reverse = t.EntityView([])
        reverse._register_multi_impl(
            "shared-name",
            t.ImplKind.Get,
            lambda indices, out: [],
            t.TensorSpec(supports=True),
        )
        with pytest.raises(ValueError):
            reverse._register_impl(
                "shared-name",
                t.ImplKind.Get,
                lambda indices, out: None,
                t.TensorSpec(supports=True),
            )

    def test_cross_cardinality_set_registration_is_rejected(self) -> None:
        """Verify rejection of equal set names across single and multi cardinality."""
        view = t.EntityView([])
        view._register_multi_impl(
            "shared-name",
            t.ImplKind.Set,
            lambda data, indices: None,
            t.TensorSpec(supports=True),
        )
        with pytest.raises(ValueError):
            view._register_impl(
                "shared-name",
                t.ImplKind.Set,
                lambda data, indices: None,
                t.TensorSpec(supports=True),
            )

        reverse = t.EntityView([])
        reverse._register_impl(
            "shared-name",
            t.ImplKind.Set,
            lambda data, indices: None,
            t.TensorSpec(supports=True),
        )
        with pytest.raises(ValueError):
            reverse._register_multi_impl(
                "shared-name",
                t.ImplKind.Set,
                lambda data, indices: None,
                t.TensorSpec(supports=True),
            )


class TestEntityOptions(TensorsRegistryTestCase):
    """Verify the construction arguments that paths alone cannot express."""

    def test_options_reach_the_factory(self) -> None:
        """Verify that an options mapping arrives at the registered factory unchanged."""
        seen: list[object] = []

        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            seen.append(options)
            return t.EntityView(paths)

        t.register_entity("mock", "rigid-contact", factory)
        t.create_entity(
            "mock",
            "rigid-contact",
            ["/World/Sensor"],
            {"filter-patterns": ["/World/Ground"], "max-contact-data-count": 64},
        )

        assert seen == [{"filter-patterns": ["/World/Ground"], "max-contact-data-count": 64}]

    def test_options_default_to_none(self) -> None:
        """Verify that omitting options hands the factory None rather than an empty mapping.

        The two are not interchangeable: an engine distinguishes "use your defaults" from "the caller
        asked for nothing", and only the first may substitute its own capacities.
        """
        seen: list[object] = []

        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            seen.append(options)
            return t.EntityView(paths)

        t.register_entity("mock", "articulation", factory)
        t.create_entity("mock", "articulation", ["/World/Robot"])

        assert seen == [None]

    def test_single_pattern_string_is_one_path(self) -> None:
        """Verify that a bare pattern is one path rather than a sequence of characters.

        ``str`` satisfies ``Sequence[str]``, so a naive ``list(paths)`` would split a pattern into
        per-character entries that match nothing.
        """
        seen: list[list[str]] = []

        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            seen.append(list(paths))
            return t.EntityView(paths)

        t.register_entity("mock", "articulation", factory)
        t.create_entity("mock", "articulation", "/World/envs/*/Cube")

        assert seen == [["/World/envs/*/Cube"]]

    def test_empty_string_rejected(self) -> None:
        """Verify that an empty pattern string is refused rather than treated as a path."""
        t.register_entity("mock", "articulation", lambda paths, options=None: t.EntityView(paths))
        with pytest.raises(ValueError):
            t.create_entity("mock", "articulation", "")

    def test_empty_paths_rejected(self) -> None:
        """Verify that an empty pattern list is refused before it reaches the factory."""
        called: list[object] = []

        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            called.append(paths)
            return t.EntityView(paths)

        t.register_entity("mock", "articulation", factory)
        with pytest.raises(ValueError):
            t.create_entity("mock", "articulation", [])
        assert not called

    def test_device_ordinal_applied_to_view(self) -> None:
        """Verify that a caller-supplied device ordinal lands on the created view."""
        t.register_entity("mock", "articulation", lambda paths, options=None: t.EntityView(paths))

        view = t.create_entity("mock", "articulation", ["/World/Robot"], device_ordinal=3)
        assert view.device_ordinal == 3

    def test_device_ordinal_left_alone_when_unset(self) -> None:
        """Verify that omitting the device ordinal preserves what the engine reported."""

        def factory(paths: list[str], options: dict[str, object] | None = None) -> t.EntityView:
            view = t.EntityView(paths)
            view.device_ordinal = 5
            return view

        t.register_entity("mock", "articulation", factory)
        view = t.create_entity("mock", "articulation", ["/World/Robot"])
        assert view.device_ordinal == 5
