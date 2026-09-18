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

"""Provide Newton entity-view adapters for the physics tensor registry.

The physics manager owns simulation lifecycle, while the USD-backed Newton stage owns model state. This module
registers the Newton operations backed by that state. Unsupported view families return empty views without advertising
operations.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

# The impl module (not the public facade): the backend wires warp dispatch through the
# internal `_attach_warp_dispatch` helper, which `from .impl import *` does not re-export.
import isaacsim.physics.manager.impl.tensors as t
import warp as wp

from .articulation_view import NewtonArticulationView
from .backend import NewtonSimView
from .contact_kernels import (
    clamp_raw_contact_counts_to_capacity,
    gather_raw_contact_counts,
    pack_raw_contact_data,
)
from .kernels import (
    fill_contiguous_indices,
    gather_float_matrix,
    gather_float_rows,
    sanitize_indices,
    scatter_float_matrix,
    scatter_float_rows,
)
from .rigid_body_view import NewtonRigidBodyView
from .rigid_contact_view import NewtonRigidContactView


class _WarpFrontendShim:
    """Provide Warp-only tensor allocation for the Newton view implementations.

    Args:
        device_ordinal: Warp device ordinal, with a negative value selecting the CPU.

    """

    name: str = "warp"
    #: Frontend name used by the physics tensor registry.

    def __init__(self, device_ordinal: int = -1) -> None:
        from isaacsim.physics.manager.impl.tensors import frontends

        self.device_ordinal = device_ordinal
        self.device = frontends.parse_device(device_ordinal)

    @staticmethod
    def wrap_tensor(value: Any) -> Any:
        """Wrap a supported tensor for Warp access.

        Args:
            value: Tensor to wrap.

        Returns:
            Tensor wrapper produced by the manager frontend utilities.

        """
        from isaacsim.physics.manager.impl.tensors import frontends

        return frontends.wrap_tensor(value)

    @staticmethod
    def unwrap_to_array(value: Any) -> Any:
        """Return a tensor value without additional unwrapping.

        Args:
            value: Tensor value to preserve.

        Returns:
            The original value.

        """
        return value

    @staticmethod
    def dtype_to_warp(dtype: object) -> object:
        """Convert a tensor-registry data type to its Warp representation.

        Args:
            dtype: Registry data type.

        Returns:
            Corresponding Warp data type.

        """
        from isaacsim.physics.manager.impl.tensors import frontends

        return frontends.dtype_to_warp(dtype)

    @staticmethod
    def dtype_from_warp(dtype: object) -> object:
        """Convert a Warp data type to its tensor-registry representation.

        Args:
            dtype: Warp data type.

        Returns:
            Corresponding registry data type.

        """
        from isaacsim.physics.manager.impl.tensors import frontends

        return frontends.dtype_from_warp(dtype)

    def create_tensor(self, shape: Sequence[int], dtype: object) -> tuple[object, None]:
        """Create a tensor on the configured device.

        Args:
            shape: Tensor dimensions.
            dtype: Tensor data type.

        Returns:
            Created tensor and a None descriptor placeholder.

        """
        from isaacsim.physics.manager.impl.tensors import frontends

        tensor = frontends.create_tensor(shape, dtype, self.device_ordinal)
        return tensor, None


def _create_contiguous_indices(count: int, device: object) -> wp.array:
    indices = wp.empty(count, dtype=wp.int32, device=device)
    wp.launch(fill_contiguous_indices, dim=count, outputs=[indices], device=device)
    return indices


def _validate_indices(
    indices: wp.array,
    device: object,
    count: int,
) -> wp.array:
    """Validate public int32 indices before indexed data access.

    Only the array's metadata is checked -- data type, rank, contiguity and device.
    Those are host-side properties, so they cost nothing and are rejected loudly.

    Index *values* are not checked. They live in device memory, so bounds or
    uniqueness could only be established by reading them back, and the data path
    does not synchronize. Out-of-range values are dropped by the kernels that
    consume them: a read leaves the dropped-row marker in place, a write skips the
    entity. Repeated values are permitted on reads and undefined on writes, where
    the surviving value depends on device scheduling.

    Empty and reordered selections remain valid.

    Args:
        indices: Indices to validate.
        device: Device required for the index array.
        count: Number of addressable entities.

    Returns:
        The validated index array.

    Raises:
        TypeError: If the indices are not a one-dimensional integer array.
        ValueError: If the indices are noncontiguous, on the wrong device, or
            validated against a negative entity count.

    """
    if indices.ndim != 1 or indices.dtype != wp.int32:
        raise TypeError("EntityView indices must be a one-dimensional int32 Warp array")
    if not indices.is_contiguous:
        raise ValueError("EntityView indices must be contiguous")
    if indices.device != device:
        raise ValueError(f"EntityView indices are on {indices.device}, expected {device}")
    if count < 0:
        raise ValueError(f"EntityView count must be non-negative, got {count}")
    return indices


def _sanitized_indices(indices: wp.array, count: int, device: object) -> tuple[wp.array, wp.array]:
    """Make a selection safe to dereference and flag the entries that were not.

    Out-of-range values are clamped so every downstream dereference is in bounds, and
    marked inactive so the consuming operation writes nothing for them. Runs entirely
    on device -- the values are never read back.

    Args:
        indices: Caller selection, unvalidated.
        count: Number of addressable entities.
        device: Device holding the selection.

    Returns:
        The clamped selection and its active-entry mask.

    """
    safe = wp.empty(indices.shape[0], dtype=wp.int32, device=device)
    active = wp.empty(indices.shape[0], dtype=int, device=device)
    wp.launch(
        sanitize_indices,
        dim=indices.shape[0],
        inputs=[indices, int(count)],
        outputs=[safe, active],
        device=device,
    )
    return safe, active


def _validate_float_rows(data: wp.array, rows: int, columns: int, device: object, label: str) -> None:
    expected_shape = (rows, columns)
    if tuple(data.shape) != expected_shape:
        raise ValueError(f"{label} has shape {tuple(data.shape)}, expected {expected_shape}")
    if data.dtype != wp.float32:
        raise TypeError(f"{label} has dtype {data.dtype}, expected {wp.float32}")
    if data.device != device:
        raise ValueError(f"{label} is on {data.device}, expected {device}")


def _read_float_rows(
    getter: Callable[[], wp.array],
    count: int,
    columns: int,
    device: object,
    indices: wp.array | None,
    out: wp.array | None,
) -> wp.array:
    selected_indices = None
    if indices is not None:
        selected_indices = _validate_indices(indices, device, count)

    full = getter()
    _validate_float_rows(full, count, columns, device, "Newton row getter result")

    if indices is None:
        if out is None:
            return full
        _validate_float_rows(out, full.shape[0], full.shape[1], full.device, "output buffer")
        wp.copy(out, full)
        return out

    result = out
    if result is None:
        result = wp.empty((indices.shape[0], full.shape[1]), dtype=wp.float32, device=full.device)
    else:
        _validate_float_rows(result, indices.shape[0], full.shape[1], full.device, "output buffer")
    wp.launch(
        gather_float_rows,
        dim=(indices.shape[0], full.shape[1]),
        inputs=[full, selected_indices, float(t._invalid_fill_value(wp.float32))],
        outputs=[result],
        device=full.device,
    )
    return result


def _write_float_rows(
    setter: Callable[[wp.array, wp.array], None],
    count: int,
    columns: int,
    device: object,
    data: wp.array,
    indices: wp.array | None,
) -> None:
    if data.ndim != 2 or data.dtype != wp.float32 or data.shape[1] != columns:
        raise TypeError(f"EntityView row data must have shape (N, {columns}) and dtype float32")

    if data.device != device:
        raise ValueError(f"input buffer is on {data.device}, expected {device}")

    if indices is None:
        _validate_float_rows(data, count, columns, device, "input buffer")
        setter(data, _create_contiguous_indices(count, device))
        return

    selected_indices = _validate_indices(indices, device, count)
    safe_indices, active_mask = _sanitized_indices(selected_indices, count, device)
    if data.shape[0] == count:
        setter(data, safe_indices, active_mask)
        return
    if data.shape[0] != indices.shape[0]:
        raise ValueError(
            f"Indexed write expects {count} full rows or {indices.shape[0]} compact rows, got {data.shape[0]}"
        )

    expanded = wp.empty((count, columns), dtype=wp.float32, device=device)
    wp.launch(
        scatter_float_rows,
        dim=(indices.shape[0], columns),
        inputs=[data, safe_indices, active_mask],
        outputs=[expanded],
        device=device,
    )
    setter(expanded, safe_indices, active_mask)


def _register_float_rows(
    view: t.EntityView,
    name: str,
    shape: Sequence[int],
    device: object,
    getter: Callable[[], wp.array],
    setter: Callable[[wp.array, wp.array], None] | None = None,
) -> None:
    """Register a two-dimensional entity-view operation.

    The operation always supports reads and supports writes when ``setter`` is
    provided. Indexed reads and writes use rows along the leading dimension.

    Args:
        view: View receiving the operation.
        name: Registered operation name.
        shape: Full ``[count, columns]`` data shape.
        device: Device containing the operation data.
        getter: Function returning all rows.
        setter: Function that writes selected rows, or None to register a read-only operation.

    Note:
        A read without an output array may return reusable view-owned storage.
        Supply an output array or copy the result to retain a stable snapshot.

    """
    view._register_impl(
        name,
        t.ImplKind.Get,
        lambda indices, out: _read_float_rows(getter, shape[0], shape[1], device, indices, out),
        t.TensorSpec(
            dtype=t.DType.FLOAT32,
            shape_hint=list(shape),
            device_kind=t.DeviceKind.ENGINE_DEFAULT,
            supports=True,
            supports_indexed_read=True,
        ),
    )
    if setter is not None:
        view._register_impl(
            name,
            t.ImplKind.Set,
            lambda data, indices: _write_float_rows(setter, shape[0], shape[1], device, data, indices),
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=list(shape),
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_write=True,
            ),
        )


def _validate_float_matrix(data: wp.array, count: int, rows: int, columns: int, device: object, label: str) -> None:
    expected_shape = (count, rows, columns)
    if tuple(data.shape) != expected_shape:
        raise ValueError(f"{label} has shape {tuple(data.shape)}, expected {expected_shape}")
    if data.dtype != wp.float32:
        raise TypeError(f"{label} has dtype {data.dtype}, expected {wp.float32}")
    if data.device != device:
        raise ValueError(f"{label} is on {data.device}, expected {device}")


def _read_float_matrix(
    getter: Callable[[], wp.array],
    count: int,
    rows: int,
    columns: int,
    device: object,
    indices: wp.array | None,
    out: wp.array | None,
) -> wp.array:
    selected_indices = None
    if indices is not None:
        selected_indices = _validate_indices(indices, device, count)

    full = getter()
    _validate_float_matrix(full, count, rows, columns, device, "Newton matrix getter result")

    if indices is None:
        if out is None:
            return full
        _validate_float_matrix(out, full.shape[0], rows, columns, full.device, "output buffer")
        wp.copy(out, full)
        return out

    result = out
    if result is None:
        result = wp.empty((indices.shape[0], rows, columns), dtype=wp.float32, device=full.device)
    else:
        _validate_float_matrix(result, indices.shape[0], rows, columns, full.device, "output buffer")
    wp.launch(
        gather_float_matrix,
        dim=(indices.shape[0], rows, columns),
        inputs=[full, selected_indices, float(t._invalid_fill_value(wp.float32))],
        outputs=[result],
        device=full.device,
    )
    return result


def _write_float_matrix(
    setter: Callable[[wp.array, wp.array], None],
    count: int,
    rows: int,
    columns: int,
    device: object,
    data: wp.array,
    indices: wp.array | None,
) -> None:
    if data.ndim != 3 or data.dtype != wp.float32 or data.shape[1] != rows or data.shape[2] != columns:
        raise TypeError(f"EntityView matrix data must have shape (N, {rows}, {columns}) and dtype float32")
    if data.device != device:
        raise ValueError(f"input buffer is on {data.device}, expected {device}")

    if indices is None:
        _validate_float_matrix(data, count, rows, columns, device, "input buffer")
        setter(data, _create_contiguous_indices(count, device))
        return

    selected_indices = _validate_indices(indices, device, count)
    safe_indices, active_mask = _sanitized_indices(selected_indices, count, device)
    if data.shape[0] == count:
        setter(data, safe_indices, active_mask)
        return
    if data.shape[0] != indices.shape[0]:
        raise ValueError(
            f"Indexed write expects {count} full rows or {indices.shape[0]} compact rows, got {data.shape[0]}"
        )

    # Scatter the compact ``[K, rows, cols]`` payload into a full buffer at the
    # selected rows, mirroring the 2D ``_write_float_rows`` compact path.
    expanded = wp.empty((count, rows, columns), dtype=wp.float32, device=device)
    wp.launch(
        scatter_float_matrix,
        dim=(indices.shape[0], rows, columns),
        inputs=[data, safe_indices, active_mask],
        outputs=[expanded],
        device=device,
    )
    setter(expanded, safe_indices, active_mask)


def _register_float_matrix(
    view: t.EntityView,
    name: str,
    shape: Sequence[int],
    device: object,
    getter: Callable[[], wp.array],
    setter: Callable[[wp.array, wp.array], None] | None = None,
) -> None:
    """Register a three-dimensional entity-view operation.

    The operation always supports reads and supports writes when ``setter`` is
    provided. The shape retains separate row and column axes so entity metadata
    can derive the per-entity row count.

    Args:
        view: View receiving the operation.
        name: Registered operation name.
        shape: Full ``[count, rows, columns]`` data shape.
        device: Device containing the operation data.
        getter: Function returning all matrices.
        setter: Function that writes selected matrices, or None to register a read-only operation.

    Note:
        A read without an output array may return reusable view-owned storage.
        Supply an output array or copy the result to retain a stable snapshot.

    """
    view._register_impl(
        name,
        t.ImplKind.Get,
        lambda indices, out: _read_float_matrix(getter, shape[0], shape[1], shape[2], device, indices, out),
        t.TensorSpec(
            dtype=t.DType.FLOAT32,
            shape_hint=list(shape),
            device_kind=t.DeviceKind.ENGINE_DEFAULT,
            supports=True,
            supports_indexed_read=True,
        ),
    )
    if setter is not None:
        view._register_impl(
            name,
            t.ImplKind.Set,
            lambda data, indices: _write_float_matrix(setter, shape[0], shape[1], shape[2], device, data, indices),
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=list(shape),
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_write=True,
            ),
        )


def _register_unsupported(view: t.EntityView, name: str, kind: t.ImplKind) -> None:
    """Register an explicitly unsupported Newton operation.

    The operation remains queryable through capability metadata but rejects direct data access.

    Args:
        view: Entity view receiving the unsupported operation.
        name: Operation name.
        kind: Read or write implementation kind.

    """
    spec = t.TensorSpec(
        dtype=t.DType.FLOAT32,
        shape_hint=[],
        device_kind=t.DeviceKind.ENGINE_DEFAULT,
        supports=False,
    )
    if kind == t.ImplKind.Get:
        view._register_impl(name, kind, lambda indices, out: out, spec)
    else:
        view._register_impl(name, kind, lambda data, indices: None, spec)


def _uniquify_names(names: Sequence[str]) -> list[str]:
    """Make names unique while preserving each first occurrence.

    A repeated name receives the lowest available ``_<k>`` suffix.

    Args:
        names: Names in source order.

    Returns:
        Collision-free names in the same order.

    """
    seen: set[str] = set()
    result: list[str] = []
    for name in names:
        candidate = name
        k = 0
        while candidate in seen:
            candidate = f"{name}_{k}"
            k += 1
        seen.add(candidate)
        result.append(candidate)
    return result


def _validate_output_like(destination: wp.array, source: wp.array, label: str) -> None:
    if tuple(destination.shape) != tuple(source.shape):
        raise ValueError(f"{label} has shape {tuple(destination.shape)}, expected {tuple(source.shape)}")
    if destination.dtype != source.dtype:
        raise TypeError(f"{label} has dtype {destination.dtype}, expected {source.dtype}")
    if destination.device != source.device:
        raise ValueError(f"{label} is on {destination.device}, expected {source.device}")


def _copy_multi_result(result: list[wp.array], out: Sequence[wp.array] | None) -> list[wp.array]:
    # The source buffers are Newton view-owned scratch storage overwritten on the
    # next read, so a caller wanting a stable snapshot must supply `out` to copy into.
    if not out:
        return result
    if len(out) != len(result):
        raise ValueError(f"Expected {len(result)} output buffers, got {len(out)}")
    for index, (source, destination) in enumerate(zip(result, out)):
        _validate_output_like(destination, source, f"output buffer {index}")
        wp.copy(destination, source)
    return list(out)


def _make_indexed_raw_contact_buffers(
    full: list[wp.array],
    selected_count: int,
    out: Sequence[wp.array] | None,
) -> list[wp.array]:
    record_capacity = full[0].shape[0]
    expected_shapes = [
        (record_capacity, 1),
        (record_capacity, 3),
        (record_capacity, 3),
        (record_capacity, 1),
        (selected_count,),
        (selected_count,),
        (record_capacity,),
    ]
    expected_dtypes = [wp.float32, wp.float32, wp.float32, wp.float32, wp.uint32, wp.uint32, wp.uint64]
    device = full[0].device

    if out:
        if len(out) != len(expected_shapes):
            raise ValueError(f"Expected {len(expected_shapes)} output buffers, got {len(out)}")
        result = list(out)
        for index, (buffer, shape, dtype) in enumerate(zip(result, expected_shapes, expected_dtypes)):
            if tuple(buffer.shape) != shape:
                raise ValueError(f"output buffer {index} has shape {tuple(buffer.shape)}, expected {shape}")
            if buffer.dtype != dtype:
                raise TypeError(f"output buffer {index} has dtype {buffer.dtype}, expected {dtype}")
            if buffer.device != device:
                raise ValueError(f"output buffer {index} is on {buffer.device}, expected {device}")
            buffer.zero_()
        return result

    return [wp.zeros(shape, dtype=dtype, device=device) for shape, dtype in zip(expected_shapes, expected_dtypes)]


def _read_raw_contact_data(
    legacy: NewtonRigidContactView,
    indices: wp.array | None,
    out: Sequence[wp.array] | None,
) -> list[wp.array]:
    selected_indices = None
    if indices is not None:
        selected_indices = _validate_indices(indices, legacy._model.device, legacy.sensor_count)

    full = list(legacy.get_raw_contact_data())
    if len(full) != 7:
        raise RuntimeError(f"Newton raw-contact-data returned {len(full)} buffers, expected 7")
    if indices is None:
        return _copy_multi_result(full, out)

    selected_count = indices.shape[0]
    result = _make_indexed_raw_contact_buffers(full, selected_count, out)
    if selected_count == 0:
        return result

    selected_count_i32 = wp.empty(selected_count, dtype=wp.int32, device=full[0].device)
    selected_start_i32 = wp.empty(selected_count, dtype=wp.int32, device=full[0].device)
    wp.launch(
        gather_raw_contact_counts,
        dim=selected_count,
        inputs=[full[4], full[5], selected_indices, full[0].shape[0]],
        outputs=[selected_count_i32],
        device=full[0].device,
    )
    wp.utils.array_scan(selected_count_i32, selected_start_i32, inclusive=False)
    wp.launch(
        clamp_raw_contact_counts_to_capacity,
        dim=selected_count,
        inputs=[selected_start_i32, full[0].shape[0]],
        outputs=[selected_count_i32],
        device=full[0].device,
    )
    wp.utils.array_scan(selected_count_i32, selected_start_i32, inclusive=False)
    wp.utils.array_cast(selected_count_i32, result[4])
    wp.utils.array_cast(selected_start_i32, result[5])
    wp.launch(
        pack_raw_contact_data,
        dim=selected_count,
        inputs=[
            full[0],
            full[1],
            full[2],
            full[3],
            full[4],
            full[5],
            full[6],
            selected_indices,
            selected_start_i32,
            full[0].shape[0],
        ],
        outputs=[result[0], result[1], result[2], result[3], result[6]],
        device=full[0].device,
    )
    return result


class _LegacyAdapter(t.EntityView):
    """Expose direct Newton view methods alongside registered entity operations.

    Args:
        legacy: Direct Newton view implementation to adapt.
        paths: Selected USD paths, or None to use paths supplied by the implementation.

    """

    def __init__(self, legacy: Any, paths: Sequence[str] | None = None) -> None:
        super().__init__(list(paths or getattr(legacy, "paths", []) or []))
        self._legacy = legacy
        for attr in ("count", "num_entities", "num_articulations", "num_rigid_bodies", "num_sensors"):
            if hasattr(legacy, attr):
                self.count = int(getattr(legacy, attr))
                break

    def __getattr__(self, name: str) -> Any:
        legacy = self.__dict__.get("_legacy")
        if legacy is not None and hasattr(legacy, name):
            return getattr(legacy, name)
        raise AttributeError(name)


class _NewtonArticulationViewAdapter(_LegacyAdapter):
    """Expose registered entity-view operations for Newton articulations.

    Args:
        legacy: Direct Newton articulation view implementation to adapt.
        paths: Selected USD paths, or None to use paths supplied by the implementation.

    """

    def __init__(self, legacy: NewtonArticulationView, paths: Sequence[str] | None = None) -> None:
        super().__init__(legacy, paths)
        # The direct Newton backend flattens DOF maps without padding. Dense
        # (count, max_dofs) operations are therefore valid only when every row
        # has max_dofs entries. A variable-width view makes no capability claim
        # for operations whose maps are not rectangular.
        uniform_dof_width = all(meta.dof_count == legacy.max_dofs for meta in legacy._backend.meta_types)
        expected_dof_map_size = legacy.count * legacy.max_dofs
        position_map_is_dense = legacy._backend.dof_position_indices.shape[0] == expected_dof_map_size
        velocity_map_is_dense = legacy._backend.dof_velocity_indices.shape[0] == expected_dof_map_size
        axis_map_is_dense = legacy._backend.dof_axis_indices.shape[0] == expected_dof_map_size
        # Each op is guarded by the density of the specific DOF map its
        # getter/setter indexes: positions -> dof_position_indices, velocities
        # -> dof_velocity_indices, targets/forces/stiffnesses -> dof_axis_indices.
        # These maps diverge for multi-DOF joints (free-floating, spherical), so
        # a shared guard would register an op against an under-sized index array.
        if uniform_dof_width and position_map_is_dense:
            _register_float_rows(
                self,
                "dof-positions",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_positions,
                legacy.set_dof_positions,
            )
        if uniform_dof_width and velocity_map_is_dense:
            _register_float_rows(
                self,
                "dof-velocities",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_velocities,
                legacy.set_dof_velocities,
            )
        if uniform_dof_width and axis_map_is_dense:
            _register_float_rows(
                self,
                "dof-position-targets",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_position_targets,
                legacy.set_dof_position_targets,
            )
            _register_float_rows(
                self,
                "dof-velocity-targets",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_velocity_targets,
                legacy.set_dof_velocity_targets,
            )
            _register_float_rows(
                self,
                "dof-actuation-forces",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_actuation_forces,
                legacy.set_dof_actuation_forces,
            )
            _register_float_rows(
                self,
                "dof-stiffnesses",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_stiffnesses,
                legacy.set_dof_stiffnesses,
            )
            _register_float_rows(
                self,
                "dof-dampings",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_dampings,
                legacy.set_dof_dampings,
            )
            _register_float_rows(
                self,
                "dof-armatures",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_armatures,
                legacy.set_dof_armatures,
            )
            # Joint limits are 3D [count, max_dofs, 2] ([lower, upper] on the
            # trailing axis); XPBD enforces them and MuJoCo consumes the
            # limit ke/kd, so get + set are both honest.
            _register_float_matrix(
                self,
                "dof-limits",
                [legacy.count, legacy.max_dofs, 2],
                legacy._model.device,
                legacy.get_dof_limits,
                legacy.set_dof_limits,
            )
            _register_float_rows(
                self,
                "dof-max-forces",
                [legacy.count, legacy.max_dofs],
                legacy._model.device,
                legacy.get_dof_max_forces,
                legacy.set_dof_max_forces,
            )
        _register_float_rows(
            self,
            "masses",
            [legacy.count, legacy.max_links],
            legacy._model.device,
            legacy.get_masses,
            legacy.set_masses,
        )
        _register_float_rows(
            self,
            "inv-masses",
            [legacy.count, legacy.max_links],
            legacy._model.device,
            legacy.get_inv_masses,
        )
        # Per-link inertial frame: COM pose [pos(3)+quat(4)] and the 3x3 inertia
        # tensor (flattened to 9). All back real model attributes (body_com /
        # body_inertia / body_inv_inertia), so both directions round-trip.
        _register_float_matrix(
            self,
            "coms",
            [legacy.count, legacy.max_links, 7],
            legacy._model.device,
            legacy.get_coms,
            legacy.set_coms,
        )
        _register_float_matrix(
            self,
            "inertias",
            [legacy.count, legacy.max_links, 9],
            legacy._model.device,
            legacy.get_inertias,
            legacy.set_inertias,
        )
        _register_float_matrix(
            self,
            "inv-inertias",
            [legacy.count, legacy.max_links, 9],
            legacy._model.device,
            legacy.get_inv_inertias,
        )
        # Per-link spatial state, shape (count, max_links, N); read-only,
        # matching the ovphysx link-pose/link-velocity bindings. The 3D shape
        # keeps max_links at shape_hint[1] so view.max_links resolves correctly.
        _register_float_matrix(
            self,
            "link-transforms",
            [legacy.count, legacy.max_links, 7],
            legacy._model.device,
            legacy.get_link_transforms,
        )
        _register_float_matrix(
            self,
            "link-velocities",
            [legacy.count, legacy.max_links, 6],
            legacy._model.device,
            legacy.get_link_velocities,
        )
        # Read-only dynamics matrices, sized from the model's per-articulation
        # dimensions to match the eval_jacobian / eval_mass_matrix output shapes
        # exactly (model_dofs includes the 6 root DOFs for a floating base).
        model = legacy._model
        _register_float_matrix(
            self,
            "jacobians",
            [legacy.count, model.max_joints_per_articulation * 6, model.max_dofs_per_articulation],
            model.device,
            legacy.get_jacobians,
        )
        _register_float_matrix(
            self,
            "generalized-mass-matrices",
            [legacy.count, model.max_dofs_per_articulation, model.max_dofs_per_articulation],
            model.device,
            legacy.get_generalized_mass_matrices,
        )
        self._register_metadata("num-dofs", lambda: int(legacy.max_dofs))
        self._register_metadata("prim-paths", lambda: list(legacy.prim_paths))
        # Root body spatial velocity [linear(3) + angular(3)] per articulation.
        _register_float_rows(
            self,
            "root-velocities",
            [legacy.count, 6],
            legacy._model.device,
            legacy.get_root_velocities,
            legacy.set_root_velocities,
        )
        # Root body transform [position(3) + quaternion(4)] per articulation.
        _register_float_rows(
            self,
            "root-transforms",
            [legacy.count, 7],
            legacy._model.device,
            legacy.get_root_transforms,
            legacy.set_root_transforms,
        )
        # Per-articulation joint/link names (metatype-uniform view -> first
        # metatype's list, matching the ovphysx backend's flat name metadata).
        self._register_metadata(
            "joint-names",
            lambda: _uniquify_names(legacy.joint_names[0]) if legacy.joint_names else [],
        )
        self._register_metadata(
            "link-names",
            lambda: _uniquify_names(legacy.link_names[0]) if legacy.link_names else [],
        )
        self._register_metadata(
            "dof-names",
            lambda: _uniquify_names(legacy.dof_names[0]) if legacy.dof_names else [],
        )
        self._register_metadata("num-links", lambda: int(legacy.max_links))
        self._register_metadata(
            "num-joints",
            lambda: len(legacy.joint_names[0]) if legacy.joint_names else 0,
        )
        # Representative metatype's fixed-base flag, matching the ovphysx
        # single-bool metadata. Valid for a homogeneous view; an empty view
        # (count 0) has no articulation, so default to False.
        self._register_metadata(
            "is-fixed-base",
            lambda: bool(legacy.shared_metatype.fixed_base) if legacy.count > 0 else False,
        )
        # Total shapes across the articulation's links. The per-shape property
        # surface (material-properties / offsets) is not wired on Newton, but the
        # count itself is backed and mirrors the ovphysx shape-dimension metadata.
        self._register_metadata("num-shapes", lambda: int(legacy.max_shapes))
        # No Newton API backs these (verified against Newton 1.3): register as
        # supports=False stubs so they are honestly reported as unsupported
        # rather than left unregistered.
        _register_unsupported(self, "coriolis-and-centrifugal-compensation-forces", t.ImplKind.Get)
        _register_unsupported(self, "gravity-compensation-forces", t.ImplKind.Get)
        _register_unsupported(self, "disable-gravities", t.ImplKind.Get)
        _register_unsupported(self, "disable-gravities", t.ImplKind.Set)
        # Aggregate mass-center / centroidal-momentum dynamics have no Newton
        # equivalent; a direct call reports "unsupported" instead of "not
        # registered". The scenario tests stay explicitly skipped.
        _register_unsupported(self, "articulation-mass-center", t.ImplKind.Get)
        _register_unsupported(self, "articulation-mass-center-local", t.ImplKind.Get)
        _register_unsupported(self, "articulation-centroidal-momentum", t.ImplKind.Get)
        # joint_friction is a real solver input, but the maximal (XPBD) solver
        # ignores it, so a friction-dynamics write cannot be honored there.
        # A generalized-coordinate solver (MuJoCo) consumes it, so gate the
        # unsupported marker on the wired solver being maximal -- the op then
        # un-registers (and the guarded tests un-skip) once such a solver backs it.
        if legacy._newton_stage.solver_is_maximal:
            _register_unsupported(self, "dof-friction-properties", t.ImplKind.Set)
        # Drive-model performance envelope (speed/effort gradient, actuator
        # velocity limits) is a PhysX concept with no Newton equivalent on any
        # solver, so it is always reported unsupported rather than faked.
        _register_unsupported(self, "dof-drive-model-properties", t.ImplKind.Set)
        # The USD DriveAPI "type" (force vs. acceleration) is not preserved by the
        # Newton USD import -- joint_target_mode encodes the position/velocity
        # control mode, not the drive type -- so no Newton attribute backs this.
        _register_unsupported(self, "drive-types", t.ImplKind.Get)

        def apply_link_forces_and_torques_at_position(
            data: wp.array,
            indices: wp.array | None,
        ) -> None:
            # Per-link packed wrench [count, max_links, 9] = [force(3), torque(3),
            # position(3)] in the world frame, matching the ovphysx contract; split
            # into three [count, max_links, 3] buffers consumed by the direct Newton view.
            device = legacy._model.device
            if data.dtype != wp.float32:
                data = data.view(wp.float32)
            _validate_float_matrix(data, legacy.count, legacy.max_links, 9, device, "wrench buffer")
            selected = indices
            active_mask = None
            if selected is None:
                selected = _create_contiguous_indices(legacy.count, device)
            else:
                selected = _validate_indices(selected, device, legacy.count)
                selected, active_mask = _sanitized_indices(selected, legacy.count, device)
            legacy.apply_forces_and_torques_at_position(
                data[:, :, 0:3],
                data[:, :, 3:6],
                data[:, :, 6:9],
                selected,
                active_mask,
            )

        self._register_impl(
            "apply-forces-and-torques-at-position",
            t.ImplKind.Set,
            apply_link_forces_and_torques_at_position,
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=[legacy.count, legacy.max_links, 9],
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_write=True,
            ),
        )
        t._attach_warp_dispatch(self)


class _NewtonRigidBodyViewAdapter(_LegacyAdapter):
    """Expose registered entity-view operations for Newton rigid bodies.

    Args:
        legacy: Direct Newton rigid-body view implementation to adapt.
        paths: Selected USD paths, or None to use paths supplied by the implementation.

    """

    def __init__(self, legacy: NewtonRigidBodyView, paths: Sequence[str] | None = None) -> None:
        super().__init__(legacy, paths)
        _register_float_rows(
            self,
            "velocities",
            [legacy.count, 6],
            legacy._model.device,
            legacy.get_velocities,
            legacy.set_velocities,
        )
        _register_float_rows(
            self,
            "masses",
            [legacy.count, 1],
            legacy._model.device,
            legacy.get_masses,
            legacy.set_masses,
        )
        _register_float_rows(
            self,
            "transforms",
            [legacy.count, 7],
            legacy._model.device,
            legacy.get_transforms,
            legacy.set_transforms,
        )
        _register_float_rows(
            self,
            "inv-masses",
            [legacy.count, 1],
            legacy._model.device,
            legacy.get_inv_masses,
        )
        # coms orientation is host-side only: Newton stores no COM orientation,
        # so get_coms returns the cached quaternion (identity until set_coms);
        # it never round-trips through physics and resets to identity on re-init.
        _register_float_rows(
            self,
            "coms",
            [legacy.count, 7],
            legacy._model.device,
            legacy.get_coms,
            legacy.set_coms,
        )
        _register_float_rows(
            self,
            "inertias",
            [legacy.count, 9],
            legacy._model.device,
            legacy.get_inertias,
            legacy.set_inertias,
        )
        _register_float_rows(
            self,
            "inv-inertias",
            [legacy.count, 9],
            legacy._model.device,
            legacy.get_inv_inertias,
        )

        def apply_forces(data: wp.array, indices: wp.array | None) -> None:
            """Apply forces through the registered rigid-body operation.

            Args:
                data: Force rows for all or selected bodies.
                indices: Selected body indices, or None to address every body.

            """
            device = legacy._model.device
            # Accept a vec3 (count,) buffer as well as (count, 3) float32, matching the
            # ovphysx backend. view() reinterprets in place (no copy); the Newton kernel
            # consumes a (count, 3) float array.
            if data.dtype != wp.float32:
                data = data.view(wp.float32)
            _validate_float_rows(data, legacy.count, 3, device, "force buffer")
            selected = indices
            active_mask = None
            if selected is None:
                selected = _create_contiguous_indices(legacy.count, device)
            else:
                selected = _validate_indices(selected, device, legacy.count)
                selected, active_mask = _sanitized_indices(selected, legacy.count, device)
            legacy.apply_forces(data, selected, active_mask)

        self._register_impl(
            "apply-forces",
            t.ImplKind.Set,
            apply_forces,
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=[legacy.count, 3],
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_write=True,
            ),
        )

        def apply_forces_and_torques_at_position(
            data: wp.array,
            indices: wp.array | None,
        ) -> None:
            """Apply packed world-frame force, torque, and position rows.

            Args:
                data: Packed force, torque, and position rows.
                indices: Selected body indices, or None to address every body.

            """
            # Single packed wrench [count, 9] = [force(3), torque(3), position(3)],
            # matching the ovphysx contract; split into the three [count, 3] buffers
            # consumed by the direct Newton view.
            device = legacy._model.device
            if data.dtype != wp.float32:
                data = data.view(wp.float32)
            _validate_float_rows(data, legacy.count, 9, device, "wrench buffer")
            selected = indices
            active_mask = None
            if selected is None:
                selected = _create_contiguous_indices(legacy.count, device)
            else:
                selected = _validate_indices(selected, device, legacy.count)
                selected, active_mask = _sanitized_indices(selected, legacy.count, device)
            legacy.apply_forces_and_torques_at_position(
                data[:, 0:3],
                data[:, 3:6],
                data[:, 6:9],
                selected,
                active_mask,
            )

        self._register_impl(
            "apply-forces-and-torques-at-position",
            t.ImplKind.Set,
            apply_forces_and_torques_at_position,
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=[legacy.count, 9],
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_write=True,
            ),
        )
        # No Newton per-body simulation-enable toggle (verified against Newton 1.3).
        _register_unsupported(self, "disable-simulations", t.ImplKind.Get)
        _register_unsupported(self, "disable-simulations", t.ImplKind.Set)
        # body_qdd (accelerations) is produced only by generalized-coordinate
        # solvers (MuJoCo); the wired maximal XPBD solver never computes it, so
        # report it unsupported under XPBD rather than returning velocities.
        # Gated on the solver being maximal so it un-registers once a solver that
        # computes accelerations is wired.
        if legacy._newton_stage.solver_is_maximal:
            _register_unsupported(self, "accelerations", t.ImplKind.Get)
        t._attach_warp_dispatch(self)


class _NewtonRigidContactViewAdapter(_LegacyAdapter):
    """Expose registered entity-view operations for Newton contact sensors.

    Args:
        legacy: Direct Newton rigid-contact view implementation to adapt.
        paths: Selected USD paths, or None to use paths supplied by the implementation.

    """

    def __init__(self, legacy: NewtonRigidContactView, paths: Sequence[str] | None = None) -> None:
        super().__init__(legacy, paths)
        self._register_multi_impl(
            "raw-contact-data",
            t.ImplKind.Get,
            lambda indices, out: _read_raw_contact_data(legacy, indices, out),
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
                supports_indexed_read=True,
            ),
        )

        def _read_net_contact_forces(indices: wp.array | None, out: wp.array | None) -> wp.array:
            result = legacy.get_net_contact_forces()
            if out is not None:
                wp.copy(out, result)
                return out
            return result

        self._register_impl(
            "net-contact-forces",
            t.ImplKind.Get,
            _read_net_contact_forces,
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=[legacy.sensor_count, 3],
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
            ),
        )

        def _read_contact_force_matrix(indices: wp.array | None, out: wp.array | None) -> wp.array:
            result = legacy.get_contact_force_matrix()
            if out is not None:
                wp.copy(out, result)
                return out
            return result

        self._register_impl(
            "contact-force-matrix",
            t.ImplKind.Get,
            _read_contact_force_matrix,
            t.TensorSpec(
                dtype=t.DType.FLOAT32,
                shape_hint=[legacy.sensor_count, legacy.filter_count, 3],
                device_kind=t.DeviceKind.ENGINE_DEFAULT,
                supports=True,
            ),
        )
        self._register_metadata("num-sensors", lambda: int(legacy.sensor_count))
        self._register_metadata("num-filters", lambda: int(legacy.filter_count))
        self._register_metadata("sensor-names", lambda: list(legacy.sensor_names))
        t._attach_warp_dispatch(self)


# ---------------------------------------------------------------------------
# The Newton simulation context owns the direct ``NewtonSimView`` backend and
# Warp frontend that every entity factory below builds its views from. It is
# internal: callers reach Newton entity views through
# ``TensorRegistry.create_entity``, never through this object.
# ---------------------------------------------------------------------------


class _NewtonSimulationContext:
    """Hold the Newton backend and frontend shared by one simulation's entity views.

    Unsupported entity families produce empty views without registered operations.

    Args:
        newton_stage: Newton stage providing model and simulation state.

    Raises:
        RuntimeError: If no Newton stage is supplied.

    """

    def __init__(self, newton_stage: Any | None) -> None:
        if newton_stage is None:
            raise RuntimeError(
                "The Newton tensor context requires a NewtonStage — wire one through "
                "`isaacsim.physics_engines.ovnewton.impl.register(...)`'s registry."
            )

        self.newton_stage = newton_stage
        self._backend = NewtonSimView(newton_stage)
        # Direct view constructors use this frontend to allocate Warp storage.
        self._frontend = _WarpFrontendShim(self._backend.device_ordinal)

    @property
    def device_ordinal(self) -> int:
        """Ordinal of the device holding this simulation's tensors, or -1 for host memory."""
        return self._backend.device_ordinal

    def create_articulation_view(self, paths: Sequence[str]) -> t.EntityView:
        """Create an articulation view for matching USD paths.

        Args:
            paths: Path patterns or explicit prim paths.

        Returns:
            Registered Newton articulation view, or a view reporting no support when no articulation matches.

        """
        path_list = list(paths)
        backend_view = self._backend.create_articulation_view(path_list)
        if backend_view is None or backend_view.count == 0:
            return _make_unsupported_view(path_list, "articulation")
        legacy = NewtonArticulationView(backend_view, self._frontend)
        return self._finish(_NewtonArticulationViewAdapter(legacy, path_list))

    def create_rigid_body_view(self, paths: Sequence[str]) -> t.EntityView:
        """Create a rigid-body view for matching USD paths.

        Args:
            paths: Path patterns or explicit prim paths.

        Returns:
            Registered Newton rigid-body view, or a view reporting no support when no body matches.

        """
        path_list = list(paths)
        backend_view = self._backend.create_rigid_body_view(path_list)
        if backend_view is None or backend_view.count == 0:
            return _make_unsupported_view(path_list, "rigid-body")
        legacy = NewtonRigidBodyView(backend_view, self._frontend)
        return self._finish(_NewtonRigidBodyViewAdapter(legacy, path_list))

    def create_rigid_contact_view(
        self,
        paths: Sequence[str],
        filter_patterns: Sequence[str] | None = None,
        max_contact_data_count: int = 0,
    ) -> t.EntityView:
        """Create a rigid-contact view for matching sensor and filter paths.

        Args:
            paths: Sensor path patterns or explicit sensor prim paths.
            filter_patterns: Filter patterns applied to every sensor. Per-sensor filter groups are not
                expressible through the registry's option map, whose values are flat, so the backend
                receives one group shared by all sensors.
            max_contact_data_count: Maximum raw contact records stored by the view.

        Returns:
            Registered Newton contact view, or a view reporting no support when no sensor matches.

        """
        path_list = list(paths)
        # The backend takes one filter group per sensor pattern; a flat list becomes the single group
        # that applies to all of them.
        backend_filters = [list(filter_patterns)] if filter_patterns else []
        backend_view = self._backend.create_rigid_contact_view(path_list, backend_filters, max_contact_data_count)
        if backend_view is None or backend_view.count == 0:
            return _make_unsupported_view(path_list, "rigid-contact")
        legacy = NewtonRigidContactView(backend_view, self._frontend)
        return self._finish(_NewtonRigidContactViewAdapter(legacy, path_list))

    def _finish(self, view: t.EntityView) -> t.EntityView:
        """Record the simulation device on a view before handing it to the caller.

        Args:
            view: Entity view built from this context.

        Returns:
            The same view, carrying this simulation's device ordinal.

        """
        view.device_ordinal = self.device_ordinal
        return view


# ---------------------------------------------------------------------------
# Registration helpers
# ---------------------------------------------------------------------------


def _active_simulation_context(simulation_name: str) -> _NewtonSimulationContext | None:
    """Build a tensor context for the active simulation registered under a name.

    Naming each simulation distinctly is what lets several coexist: the lookup is exact, so nothing is
    inferred from the name.

    Args:
        simulation_name: Name the simulation and its factories were registered under.

    Returns:
        Context for that simulation, or ``None`` when none of that name is active or its registry no longer
        holds a stage.

    """
    # Lazy import to avoid a circular dependency at module load.
    from isaacsim.physics.registration import get_active_simulation_id, k_invalid_simulation_id

    from .. import get_registry as get_newton_registry

    simulation_id = get_active_simulation_id(simulation_name)
    if simulation_id == k_invalid_simulation_id:
        return None
    try:
        registry = get_newton_registry(int(simulation_id.id))
    except Exception:
        return None
    newton_stage = registry.stage if registry is not None else None
    if newton_stage is None:
        return None
    return _NewtonSimulationContext(newton_stage)


def _make_unsupported_view(paths: Sequence[str], entity: str) -> t.EntityView:
    """Build a view that reports the entity type as unsupported.

    Args:
        paths: Prim-path patterns supplied by the caller.
        entity: Tensor entity type the view stands in for.

    Returns:
        Entity view carrying a single unsupported read operation.

    """
    view = t.EntityView(list(paths))
    view.count = 0
    _register_unsupported(view, f"{entity}-data", t.ImplKind.Get)
    return view


# Every entity factory resolves the same context from whichever Newton simulation the registration layer
# reports as active, so one implementation builds every view regardless of which entity was asked for.

#: Option names the factories below read. Paths alone cannot describe a contact view, so its remaining
#: construction arguments arrive through the registry's options map. Unrecognized names are ignored.
FILTER_PATTERNS_OPTION = "filter-patterns"
MAX_CONTACT_DATA_COUNT_OPTION = "max-contact-data-count"

_EntityFactory = Callable[[Sequence[str], dict[str, Any] | None], t.EntityView]


def _make_articulation_factory(simulation_name: str) -> _EntityFactory:
    """Build an articulation factory bound to a simulation name.

    Args:
        simulation_name: Name the simulation and its factories were registered under.

    Returns:
        Factory returning an articulation view, or a view reporting no support when that simulation is not active.

    """

    def factory(paths: Sequence[str], options: dict[str, Any] | None = None) -> t.EntityView:
        context = _active_simulation_context(simulation_name)
        if context is None:
            return _make_unsupported_view(paths, "articulation")
        return context.create_articulation_view(paths)

    return factory


def _make_rigid_body_factory(simulation_name: str) -> _EntityFactory:
    """Build a rigid-body factory bound to a simulation name.

    Args:
        simulation_name: Name the simulation and its factories were registered under.

    Returns:
        Factory returning a rigid-body view, or a view reporting no support when that simulation is not active.

    """

    def factory(paths: Sequence[str], options: dict[str, Any] | None = None) -> t.EntityView:
        context = _active_simulation_context(simulation_name)
        if context is None:
            return _make_unsupported_view(paths, "rigid-body")
        return context.create_rigid_body_view(paths)

    return factory


def _make_rigid_contact_factory(simulation_name: str) -> _EntityFactory:
    """Build a rigid-contact factory bound to a simulation name.

    Args:
        simulation_name: Name the simulation and its factories were registered under.

    Returns:
        Factory returning a rigid-contact view, or a view reporting no support when that simulation is not active.

    """

    def factory(paths: Sequence[str], options: dict[str, Any] | None = None) -> t.EntityView:
        context = _active_simulation_context(simulation_name)
        if context is None:
            return _make_unsupported_view(paths, "rigid-contact")
        options = options or {}
        filter_patterns = options.get(FILTER_PATTERNS_OPTION)
        if isinstance(filter_patterns, str):
            filter_patterns = [filter_patterns]
        max_contact_data_count = options.get(MAX_CONTACT_DATA_COUNT_OPTION, 0)
        return context.create_rigid_contact_view(paths, filter_patterns, int(max_contact_data_count))

    return factory


def _make_unsupported_factory(entity: str) -> _EntityFactory:
    """Build a factory for an entity type Newton does not implement.

    Args:
        entity: Tensor entity type the factory stands in for.

    Returns:
        Factory returning a view that reports no support.

    """

    def factory(paths: Sequence[str], options: dict[str, Any] | None = None) -> t.EntityView:
        return _make_unsupported_view(paths, entity)

    return factory


# Names this module has registered factories under, so a repeated registration is a no-op and a teardown
# only removes what it registered.
_REGISTERED: set[str] = set()

#: The engine this module implements. An engine is a kind of simulator, declared once and unaffected by how
#: many of its simulations run, so this is deliberately separate from the simulation name below even though
#: the default simulation is named after it.
ENGINE_NAME = "newton"

#: Name used when the caller registers no other. Several simulations coexist by registering distinct names.
DEFAULT_SIMULATION_NAME = "newton"


def register_with_umbrella(simulation_name: str = DEFAULT_SIMULATION_NAME) -> bool:
    """Register Newton entity-view factories under a simulation name.

    The name is what callers pass to ``create_entity``, and what the entity factories resolve their stage
    from. Registering a distinct name per simulation is what allows several to coexist, since the resolution
    is an exact match.

    Repeated calls for one name return without replacing the registered factories.

    Args:
        simulation_name: Name to register the factories under.

    Returns:
        Whether this call registered the factories. A caller that tears down what it created should
        unregister only when this returned ``True``, since another owner may hold the name.

    """
    if simulation_name in _REGISTERED:
        return False

    # The engine is a kind of simulator and is declared once; the factories below are keyed by simulation
    # name, of which there may be several.
    t.get_registry().register_engine(ENGINE_NAME)

    t.register_entity(simulation_name, "articulation", _make_articulation_factory(simulation_name))
    t.register_entity(simulation_name, "rigid-body", _make_rigid_body_factory(simulation_name))
    t.register_entity(simulation_name, "rigid-contact", _make_rigid_contact_factory(simulation_name))

    # Newton has no implementation for these. They register a view that says so, rather than an empty one:
    # a caller cannot tell an empty view from a broken registration, and only one of those is a bug.
    for entity in (
        "volume-deformable-body",
        "surface-deformable-body",
        "deformable-material",
        "sdf-shape",
    ):
        t.register_entity(simulation_name, entity, _make_unsupported_factory(entity))

    _REGISTERED.add(simulation_name)
    return True


def unregister_from_umbrella(simulation_name: str = DEFAULT_SIMULATION_NAME) -> None:
    """Remove the factories registered under a simulation name.

    Args:
        simulation_name: Name the factories were registered under.

    """
    if simulation_name not in _REGISTERED:
        return

    for entity in (
        "articulation",
        "rigid-body",
        "rigid-contact",
        "volume-deformable-body",
        "surface-deformable-body",
        "deformable-material",
        "sdf-shape",
    ):
        t.get_registry().unregister_entity(simulation_name, entity)

    _REGISTERED.discard(simulation_name)
