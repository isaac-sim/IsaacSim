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

"""Provide the string-keyed Warp data plane for the physics manager.

Engines register explicit single-buffer callbacks with ``_register_impl`` and
multi-buffer callbacks with ``_register_multi_impl``. Consumers discover named
operations and call ``get_data``, ``set_data``, ``get_data_multi``, or
``set_data_multi``. Newton operation names use their plural contract spelling,
including ``dof-positions`` and ``dof-stiffnesses``.

The native implementation is exposed through the manager and registration
nanobind modules.

Warp-only data plane
--------------------

The public contract is Warp-only. Python-registered callbacks are mirrored and
invoked directly with Warp values so callback-owned storage and caller-provided
output-buffer identity are preserved. Native dispatch retains the backing owner
of each returned array and remains the fallback for C++-registered backends.

Index contract
--------------

The data path never synchronizes the device or reads back to the host. That
splits validation of an ``indices`` selection in two:

* **Metadata** -- data type, rank, contiguity, device -- is a host-side property
  of the array, so it costs nothing and is rejected loudly. Indices must be a
  contiguous one-dimensional ``int32`` Warp array on the operation's device.
* **Values** -- whether an index is in range, or repeated -- live in device
  memory. Establishing either would require reading them back, so neither is
  checked and neither raises.

Out-of-range indices are therefore *dropped*, uniformly across CPU and GPU, both
engines, and both the Python and C++ layers:

===========================  ====================================================
operation                    result for an out-of-range index
===========================  ====================================================
``get_data``                 that output row carries the dropped-row marker
``set_data``                 that entity is not written; engine state unchanged
``get_data_multi``           that entity contributes zero records
===========================  ====================================================

The dropped-row marker is the all-bits-set pattern for the buffer's data type:
``NaN`` for floating point, ``-1`` for signed integers, and the maximum for
unsigned. ``bool`` is the documented exception -- every byte is either zero or
truthy, so a dropped row comes back as ``255``, a non-canonical true that a
caller cannot tell from a real one.

Repeated indices are well defined on a read -- the source row is copied to each
output row. On a write they are **undefined**: every write is attempted and the
surviving value depends on device scheduling.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any, NamedTuple, Protocol

import warp as wp
from isaacsim.physics.registration.bindings._bindings import (
    DeviceKind,
    DofDriveType,
    DofMotion,
    DofType,
    DType,
    Float3,
    ImplKind,
    JointType,
    ObjectType,
    TensorDesc,
    TensorRegistry,
    TensorSpec,
    get_registry,
    register_entity,
)

from ...bindings._bindings import EntityView
from ...bindings._bindings import create_entity as _create_entity_raw
from . import frontends

# ---------------------------------------------------------------------------
# Warp dispatch layer
#
# The C++ binding's `get_data` hands back a DLPack-typed `nb::ndarray`
# (works for CPU and CUDA buffers symmetrically). The wrapper below
# converts that to a `wp.array` via `wp.from_dlpack`; that is the only
# data plane we support. `set_data` accepts a `wp.array` (or anything
# else DLPack-capable, e.g. numpy ndarrays) and forwards it through to
# the C++ binding unchanged.
# ---------------------------------------------------------------------------


def _wrap_warp(
    result: object | list[object] | None,
) -> wp.array | list[wp.array | None] | None:
    """Convert a binding result to Warp arrays.

    The C++ ``get_data`` binding returns a DLPack-compatible nanobind array, or
    a list of arrays for ``get_data_multi``. The frontend imports each value
    with ``wp.from_dlpack``. ``None``, lists, and native Warp arrays
    pass through with the right structure.

    Args:
        result: Single-buffer or multi-buffer result from a tensor binding.

    Returns:
        A Warp array for a single-buffer result, a list preserving the
        multi-buffer structure, or ``None`` for an empty result.

    """
    if result is None:
        return result
    if isinstance(result, list):
        return [_wrap_warp(item) for item in result]
    return frontends.wrap_tensor(result)


def _alloc_gpu_out(view: EntityView, impl: str, device_ordinal: int) -> wp.array | None:
    """Allocate a Warp GPU buffer matching the GET specification.

    The engine reads directly into the buffer so ``get_data`` can return data
    on the requested CUDA device. Return ``None`` when the specification cannot
    be read, has no positive shape or data type, or allocation fails; the caller
    then uses native dispatch.

    Args:
        view: Entity view that owns the registered implementation.
        impl: Registered getter implementation name.
        device_ordinal: CUDA device on which to allocate the output.

    Returns:
        Device-resident output buffer, or ``None`` when no valid buffer can be
        allocated.

    """
    try:
        spec = view.get_impl_spec(impl, ImplKind.Get)
        shape = [int(s) for s in (getattr(spec, "shape_hint", None) or [])]
        dtype = getattr(spec, "dtype", None)
        # Bail on an empty or zero/negative dim (e.g. a view that currently
        # resolves to 0 entities): a non-empty shape like [0, D] would otherwise
        # pass and allocate a zero-size GPU buffer, silently taking the GPU path
        # with an empty result. Fall back to the default (CPU-staged) read.
        if not shape or dtype is None or any(d <= 0 for d in shape):
            return None
        return frontends.create_tensor(shape, dtype, device_ordinal)
    except Exception:
        return None


def _alloc_gpu_out_multi(view: EntityView, impl: str, device_ordinal: int) -> list[wp.array] | None:
    """Allocate one Warp GPU buffer per output of a multi-get implementation.

    The buffers use the implementation's per-output specifications so a native
    GPU multi-read receives a destination on the simulation device. Return
    ``None`` when the implementation exposes no per-output specifications or
    any specification lacks a positive shape or data type. Buffer-allocation
    failures propagate to the caller.

    Args:
        view: Entity view that owns the registered implementation.
        impl: Registered multi-get implementation name.
        device_ordinal: CUDA device on which to allocate the outputs.

    Returns:
        Device-resident output buffers, or ``None`` when the implementation
        specifications cannot determine valid allocations.

    """
    try:
        specs = view.get_impl_spec_multi(impl, ImplKind.Get)
    except Exception:
        return None
    if not specs:
        return None
    bufs = []
    for s in specs:
        shape = [int(x) for x in (getattr(s, "shape_hint", None) or [])]
        dtype = getattr(s, "dtype", None)
        if not shape or dtype is None or any(d <= 0 for d in shape):
            return None
        bufs.append(frontends.create_tensor(shape, dtype, device_ordinal))
    return bufs


def _spec_requires_host(view: EntityView, impl: str, kind: ImplKind) -> bool:
    """Check whether an implementation requires host buffers.

    Host-computed implementations, such as USD traversal or CPU rotation, cannot take device buffers. The frontend
    therefore stages data and indices to the CPU.

    Args:
        view: Entity view that owns the registered implementation.
        impl: Registered implementation name.
        kind: Getter or setter implementation kind.

    Returns:
        ``True`` when the implementation specification requires host data;
        ``False`` when it does not or its specification cannot be read.

    """
    try:
        return bool(getattr(view.get_impl_spec(impl, kind), "requires_host_data", False))
    except Exception:
        return False


def _to_host(x: object | None) -> object | None:
    """Move a CUDA Warp array to host memory.

    Args:
        x: Tensor value to stage.

    Returns:
        A NumPy copy when ``x`` is a CUDA Warp array; otherwise, the original
        value.

    """
    try:
        if x is not None and hasattr(x, "device") and getattr(x.device, "is_cuda", False):
            return x.numpy()
    except Exception:
        pass
    return x


def _invalid_fill_value(dtype: object) -> object:
    """Value marking an output row that no valid index produced.

    Index values live in device memory, so an out-of-range index cannot be rejected
    without a host readback. It is dropped instead, and its output row carries this
    value rather than zero -- zero is a legitimate reading for most bindings, so it
    would be indistinguishable from real data.

    The value is the all-bits-set pattern, derived from the data type rather than
    tabulated: ``NaN`` for floating point (propagates through arithmetic), ``-1``
    for signed integers (counts and offsets are non-negative), and the maximum for
    unsigned integers.

    ``bool`` is the documented exception: every byte is either zero or truthy, so
    the value comes back as a non-canonical true and a caller cannot tell it from a
    real one.

    Args:
        dtype: Warp data type of the output buffer.

    Returns:
        Scalar to fill dropped rows with.

    """
    import numpy as _np

    scalar = _np.zeros(1, dtype=wp.dtype_to_numpy(dtype))
    scalar.view(_np.uint8)[:] = 0xFF
    return scalar[0]


@wp.kernel
def _gather_rows_kernel(
    full: wp.array2d(dtype=Any),
    indices: wp.array(dtype=wp.int32),
    n: int,
    dropped: Any,
    out: wp.array2d(dtype=Any),
) -> None:
    """Gather ``out[k] = full[indices[k]]`` row-wise.

    An out-of-range index yields the dropped-row marker. Writing it here rather than
    pre-filling ``out`` keeps the read to a single pass: every element is written
    exactly once, and the common all-valid selection does not pay for a buffer-wide
    fill it immediately overwrites.

    Args:
        full: Full source matrix.
        indices: Source row index for each output row.
        n: Number of valid rows in ``full``.
        dropped: Value marking a row no valid index produced.
        out: Destination matrix.

    """
    k, j = wp.tid()
    idx = indices[k]
    if idx >= 0 and idx < n:
        out[k, j] = full[idx, j]
    else:
        out[k, j] = dropped


def _gpu_gather_rows(
    full_gpu: wp.array,
    indices: wp.array,
    device_ordinal: int,
    out: wp.array | None = None,
) -> wp.array:
    """Gather selected rows from a full GPU read without a host transfer.

    OvPhysX has no on-device gather, so the framework reads the full binding into
    a device buffer (``full_gpu``); a single warp kernel then gathers the requested
    rows along axis 0 (any trailing shape and dtype) into the caller's device
    ``out`` when given, or into a fresh buffer otherwise. Out-of-range indices are
    ignored, and their rows remain zero.

    Args:
        full_gpu: Full device read, shape ``(N, ...)``.
        indices: Row indices to gather; GPU-resident int32 on the sim device.
        device_ordinal: CUDA ordinal the result should live on.
        out: Optional contiguous caller buffer on the same device and with the
            same data type and element count as the gathered result. Its
            original shape is preserved. A fresh row-shaped buffer is
            allocated when ``None``.

    Returns:
        ``out`` unchanged when provided, or a new device-resident array with
        one row per index otherwise.

    Raises:
        ValueError: If ``indices`` is not GPU-resident int32, or ``out`` is
            provided but does not have the required device, size, data type, or
            contiguous layout.

    """
    device = frontends.parse_device(device_ordinal)

    # Indices must be a warp int32 array on the sim device. Reject a device mismatch
    # loudly rather than silently staging -- the same stance as the C++ CPU gather.
    if not (isinstance(indices, wp.array) and indices.device == device):
        raise ValueError("indexed get on a GPU simulation requires GPU indices")
    if indices.dtype != wp.int32:
        raise ValueError("indexed get: indices must be int32")

    # The kernel treats the index buffer as a flat 1-D int32[K]; a 2-D or strided view would
    # select the wrong rows, so require 1-D and contiguous.
    if indices.ndim != 1 or not indices.is_contiguous:
        raise ValueError("indexed get: indices must be 1-D and contiguous")

    n = int(full_gpu.shape[0])
    k = int(indices.shape[0])
    row_shape = tuple(int(d) for d in full_gpu.shape[1:])
    row_elems = 1
    for d in row_shape:
        row_elems *= d

    # The kernel writes the marker for any row no valid index produced, so an
    # out-of-range index is visible in the result instead of reading as a plausible
    # zero. It is passed in rather than pre-filled so the gather stays single-pass.
    # Warp cannot infer a generic scalar parameter from a NumPy value, so hand it one
    # constructed in the buffer's own Warp type.
    dropped = full_gpu.dtype(_invalid_fill_value(full_gpu.dtype).item())
    if out is None:
        result = wp.empty((k,) + row_shape, dtype=full_gpu.dtype, device=device)
    else:
        if not (isinstance(out, wp.array) and out.device == device):
            raise ValueError("indexed get: out buffer must be on the sim device")
        if out.dtype != full_gpu.dtype or out.size != k * row_elems:
            raise ValueError("indexed get: out buffer must be index-size with the read's dtype")
        # The gather reshapes `out` to (k, row_elems), which requires row-major contiguity
        # (the C++ CPU gather enforces the same via requireMatchingOut). Reject a strided
        # buffer with a clear message rather than an opaque warp reshape error.
        if not out.is_contiguous:
            raise ValueError("indexed get: out buffer must be contiguous")
        result = out

    # Gather on-device in one kernel. ovphysx has no on-device gather and wp.indexedarray
    # can't bounds-check (a negative index is an illegal access, a positive out-of-range one
    # reads garbage), so gather explicitly and ignore out-of-range indices -- their row is
    # left zero. No host round-trip.
    wp.launch(
        _gather_rows_kernel,
        dim=(k, row_elems),
        inputs=[full_gpu.reshape((n, row_elems)), indices, n, dropped],
        outputs=[result.reshape((k, row_elems))],
        device=device,
    )
    return result


@wp.kernel
def _scatter_rows_kernel(
    compact: wp.array2d(dtype=Any), indices: wp.array(dtype=wp.int32), n: int, full: wp.array2d(dtype=Any)
) -> None:
    """Scatter ``full[indices[k]] = compact[k]`` row-wise.

    An out-of-range index is ignored, so no destination row is written. This
    behavior mirrors the indexed-read gather kernel.

    Args:
        compact: Compact source matrix.
        indices: Destination row index for each source row.
        n: Number of valid rows in ``full``.
        full: Full destination matrix initialized to zero.

    """
    k, j = wp.tid()
    idx = indices[k]
    if idx >= 0 and idx < n:
        full[idx, j] = compact[k, j]


def _scatter_compact_to_full(data: object, indices: wp.array, n: int) -> wp.array:
    """Expand a compact set buffer to the entity view's full width.

    Input row ``k`` is placed at ``indices[k]``. Non-indexed and out-of-range
    destination rows remain zero.

    The ovphysx write path takes a full ``[N, ...]`` source and writes only the
    indexed rows (``src[idx] -> engine row idx``), so an index-sized buffer must be
    scattered to full width before the raw write. The scatter runs on CPU and CUDA in
    a single warp kernel.

    Indices are validated with metadata-only checks (``int32``, one-dimensional, contiguous, matching
    device) and out-of-range rows are skipped on-device, so there is **no host
    readback**: a negative or out-of-range index cannot write outside ``full``.
    Duplicate destinations are not rejected (that would need a readback); as with any
    scatter their result is device-dependent.

    Args:
        data: Compact tensor with one row per index. Non-Warp array-like
            values are copied to a CPU Warp array.
        indices: Destination row indices.
        n: Number of rows in the full destination.

    Returns:
        Full-width Warp array on the same device as ``data``.

    Raises:
        ValueError: If ``indices`` is not a one-dimensional, contiguous Warp
            ``int32`` array on the data device.

    """
    # `data` may be a plain numpy ndarray (a valid set_data input), not a warp array:
    # numpy has no `.device`, its dtype isn't a warp dtype, and it can't feed the kernel.
    # Coerce a non-warp buffer to a host warp array; warp arrays pass through.
    device = data.device if isinstance(data, wp.array) else "cpu"
    if not isinstance(data, wp.array):
        data = wp.array(data, device=device)

    full = wp.zeros((int(n),) + tuple(data.shape[1:]), dtype=data.dtype, device=device)

    # Indices must be a warp int32 array on `full`'s device -- the same contract as the
    # indexed-read gather. Reject a non-warp input, a non-int32 dtype (no silent
    # conversion), a non-1-D / strided view (the kernel reads a flat int32[K]), and a
    # device mismatch. All metadata checks -- no host readback.
    if not isinstance(indices, wp.array):
        raise ValueError("indexed set: indices must be a warp int32 array")
    if indices.dtype != wp.int32:
        raise ValueError("indexed set: indices must be int32")
    if indices.ndim != 1 or not indices.is_contiguous:
        raise ValueError("indexed set: indices must be 1-D and contiguous")
    if indices.device != full.device:
        raise ValueError("indexed set: indices must be on the data's device")

    k = int(data.shape[0])
    row_elems = 1
    for d in data.shape[1:]:
        row_elems *= int(d)
    # Scatter on-device in one kernel. wp.indexedarray can't bounds-check (a negative
    # index is an illegal access, an out-of-range one writes outside `full`), so scatter
    # explicitly and skip out-of-range indices -- their row is not written. No readback.
    wp.launch(
        _scatter_rows_kernel,
        dim=(k, row_elems),
        inputs=[data.reshape((k, row_elems)), indices, int(n)],
        outputs=[full.reshape((int(n), row_elems))],
        device=full.device,
    )
    return full


def _require_int32_indices(indices: object | None, op: str) -> None:
    """Enforce the ``int32`` index contract uniformly.

    Reject non-``int32`` integer arrays instead of silently coercing them,
    matching the gather/scatter kernels and native descriptor validation.
    Accept Warp or NumPy ``int32`` arrays. Pass ``None`` and untyped inputs,
    such as lists, through for implementation-specific handling.

    Args:
        indices: Optional index buffer to validate.
        op: Operation name to include in an error message.

    Raises:
        ValueError: If ``indices`` is an integer array whose data type is not
            ``int32``.

    """
    if indices is None:
        return
    dt = getattr(indices, "dtype", None)
    if dt is None or dt is wp.int32:  # None/untyped pass through; warp int32 is the fast path
        return
    # Only a non-int32 INTEGER index array is a contract violation (e.g. int64). A
    # float dtype here is not an index array -- the SDF view rides its float32 query
    # points through the `indices` slot -- so it passes through to the impl.
    bad = dt in (wp.int8, wp.int16, wp.int64, wp.uint8, wp.uint16, wp.uint32, wp.uint64)
    if not bad:
        try:
            import numpy as _np

            npdt = _np.dtype(dt)
            bad = npdt != _np.int32 and npdt.kind in ("i", "u")
        except Exception:
            bad = False
    if bad:
        raise ValueError(f"{op}: indices must be int32 (the umbrella convention); got {dt}")


# TODO: temporary. The generic ovphysx write (`ovphysx_write_tensor_binding`) accepts only a
# full [N, ...] source, so set_data scatters a compact [K, ...] buffer to full width. Remove
# the compact->full scatter once the ovphysx side supports a compact indexed write on the
# generic path; the real fix belongs there.


def _ensure_py_impl_mirror(view: EntityView) -> None:
    """Ensure an EntityView has mirrors for Python single and multi callbacks.

    Initialize any missing callback and specification dictionaries required by
    direct Python dispatch.

    Args:
        view: Entity view on which to initialize the callback dictionaries.

    """
    if not hasattr(view, "_py_get_impls"):
        view._py_get_impls = {}
    if not hasattr(view, "_py_set_impls"):
        view._py_set_impls = {}
    if not hasattr(view, "_py_get_multi_impls"):
        view._py_get_multi_impls = {}
    if not hasattr(view, "_py_set_multi_impls"):
        view._py_set_multi_impls = {}
    if not hasattr(view, "_py_specs"):
        view._py_specs = {}


class _PythonDispatchSpec(NamedTuple):
    supports: bool
    supports_indexed_read: bool
    supports_indexed_write: bool
    requires_host_data: bool


class _ArticulationMetatype(Protocol):
    link_names: list[str]
    joint_names: list[str]


def _snapshot_python_dispatch_spec(spec: TensorSpec | None) -> _PythonDispatchSpec:
    """Capture the by-value C++ registration semantics for direct dispatch.

    Args:
        spec: Mutable tensor specification supplied during registration, or
            ``None`` to use the default capabilities.

    Returns:
        Immutable snapshot of the dispatch capabilities.

    """
    if spec is None:
        return _PythonDispatchSpec(True, False, False, False)
    return _PythonDispatchSpec(
        bool(spec.supports),
        bool(spec.supports_indexed_read),
        bool(spec.supports_indexed_write),
        bool(spec.requires_host_data),
    )


def _validate_python_dispatch(
    view: EntityView,
    impl: str,
    kind: ImplKind,
    indices: object | None,
    operation: str,
    is_multi: bool,
) -> None:
    """Apply the C++ EntityView capability checks before direct dispatch.

    Python callbacks bypass ``EntityView::{get,set}Data*`` to preserve their
    returned Warp owners, so the wrapper must preserve the same TensorSpec
    contract rather than invoking callbacks that advertise no support.

    Args:
        view: Entity view that owns the mirrored callback.
        impl: Registered implementation name.
        kind: Getter or setter implementation kind.
        indices: Optional entity index buffer.
        operation: C++ operation name to include in error messages.
        is_multi: Whether to validate a multi-buffer implementation.

    Raises:
        RuntimeError: If the implementation reports that it is unsupported.
        ValueError: If indices are provided for an implementation that does
            not support the corresponding indexed operation.

    """
    # The C++ registry stores TensorSpec by value. Mirror an immutable snapshot
    # keyed by dispatch mode so later mutation of the caller's Python spec
    # cannot change direct-dispatch behavior. The core separately rejects a
    # single/multi callback pair sharing one (name, kind) identity.
    spec = view._py_specs[(impl, kind, is_multi)]

    if not spec.supports:
        raise RuntimeError(
            f"EntityView::{operation}: implementation '{impl}' is registered but reports supports=false "
            "(operation not implemented for this engine)"
        )
    if indices is None:
        return

    if kind == ImplKind.Get and not spec.supports_indexed_read:
        raise ValueError(f"EntityView::{operation}: implementation '{impl}' does not support indexed reads")
    if kind == ImplKind.Set and not spec.supports_indexed_write:
        raise ValueError(f"EntityView::{operation}: implementation '{impl}' does not support indexed writes")


def _install_register_impl_mirror() -> None:
    """Mirror Python-registered single and multi callbacks on each view.

    Direct dispatch passes Warp arrays to Python callbacks without a native
    tensor-descriptor conversion. The mirrors preserve callback-owned storage
    and allow callbacks to return caller-provided output buffers without
    changing identity, for both single- and multi-buffer operations.
    """
    _raw_register_impl = EntityView._register_impl
    _raw_register_multi_impl = EntityView._register_multi_impl

    def _register_impl_with_mirror(
        self: EntityView,
        impl: str,
        kind: ImplKind,
        fn: Callable[..., object],
        spec: TensorSpec | None = None,
    ) -> bool:
        _ensure_py_impl_mirror(self)
        snapshot = _snapshot_python_dispatch_spec(spec)
        if spec is None:
            result = _raw_register_impl(self, impl, kind, fn)
        else:
            result = _raw_register_impl(self, impl, kind, fn, spec)
        if kind == ImplKind.Get:
            self._py_get_impls[impl] = fn
        else:
            self._py_set_impls[impl] = fn
        self._py_specs[(impl, kind, False)] = snapshot
        return result

    def _register_multi_impl_with_mirror(
        self: EntityView,
        impl: str,
        kind: ImplKind,
        fn: Callable[..., object],
        spec: TensorSpec | None = None,
    ) -> bool:
        _ensure_py_impl_mirror(self)
        snapshot = _snapshot_python_dispatch_spec(spec)
        if spec is None:
            result = _raw_register_multi_impl(self, impl, kind, fn)
        else:
            result = _raw_register_multi_impl(self, impl, kind, fn, spec)
        if kind == ImplKind.Get:
            self._py_get_multi_impls[impl] = fn
        else:
            self._py_set_multi_impls[impl] = fn
        self._py_specs[(impl, kind, True)] = snapshot
        return result

    EntityView._register_impl = _register_impl_with_mirror
    EntityView._register_multi_impl = _register_multi_impl_with_mirror


_install_register_impl_mirror()


def _attach_warp_dispatch(
    view: EntityView | None,
    view_kind: str | None = None,
) -> EntityView | None:
    """Attach Warp dispatch for single- and multi-buffer EntityView methods.

    ``view_kind`` is the registered entity name, such as ``rigid-contact``, and
    selects view-specific metadata behavior even when a no-match view registers
    no operations.

    Dynamic instance methods invoke mirrored Python callbacks directly with
    Warp values. When no Python callback is mirrored, they call the native
    methods, whose returned arrays retain their backing storage owner.

    Single-buffer results are returned as ``wp.array``. Multi-buffer results
    retain their list structure and each element is returned as ``wp.array``.
    Each native ``get_data`` call re-reads ``view.device_ordinal`` rather than
    caching it, which supports engines that select their device during stepping,
    after the entity view was created. Eligible native GPU reads receive device
    output buffers so returned arrays reside on the simulation device.

    Args:
        view: Entity view to augment, or ``None`` when the native factory
            returned no view.
        view_kind: Registered entity name used to identify view-specific
            metadata behavior.

    Returns:
        The augmented entity view, or ``None`` when ``view`` is ``None``.

    """
    if view is None:
        return view
    if getattr(view, "_warp_dispatch_attached", False):
        return view
    _ensure_py_impl_mirror(view)
    raw_get = view.__class__.get_data
    raw_set = view.__class__.set_data
    raw_get_multi = getattr(view.__class__, "get_data_multi", None)
    raw_set_multi = getattr(view.__class__, "set_data_multi", None)

    def get_data(
        impl: str,
        indices: object | None = None,
        out: object | None = None,
        _self: EntityView = view,
        _raw: Callable[..., object] = raw_get,
    ) -> wp.array | None:
        """Read one tensor from a registered implementation.

        Args:
            impl: Registered getter implementation name.
            indices: Optional implementation-specific entity-index or query
                buffer. Integer index arrays must use ``int32``. An out-of-range
                index is dropped rather than rejected and its output row carries
                the dropped-row marker; a repeated index copies the source row to
                each output row. See the module's index contract.
            out: Optional caller-owned output buffer. When the implementation
                writes into this buffer, the returned array aliases its
                storage.
            _self: Entity view captured by the dispatch closure.
            _raw: Native getter captured by the dispatch closure.

        Returns:
            A Warp array containing the requested data, or ``None`` when the
            implementation returns no data.

        Raises:
            IndexError: If ``impl`` has no registered single-buffer getter.
            RuntimeError: If the registered implementation is unsupported.
            TypeError: If an input or output buffer has an unsupported data
                type.
            ValueError: If the indices or output buffer violate the
                implementation contract.

        """
        _require_int32_indices(indices, "indexed get")
        py_fn = _self._py_get_impls.get(impl)
        if py_fn is not None:
            _validate_python_dispatch(_self, impl, ImplKind.Get, indices, "getData", False)
            return _wrap_warp(py_fn(indices, out))
        # Read the view's device at CALL time: ovphysx resolves it on the first
        # simulate(), so it is only authoritative once stepping has begun.
        try:
            _devord = int(_self.device_ordinal)
        except Exception:
            _devord = -1
        # GPU sim: read into a device buffer so results land on the consumer's
        # device. Host-only impls (USD walk) stay on CPU -- they fall through to the
        # C++ host gather, whose indexed read requires CPU indices and returns a host
        # result even on a GPU sim (see OvPhysxEntityViews). An indexed read does a
        # full device read then gathers to index-size (into the caller's device
        # `out` when given). A non-indexed read fills a fresh buffer only when no
        # `out` is given; a non-indexed external `out` is read into directly by
        # the C++ binding below (DirectGPU), so it skips this block.
        if _devord >= 0 and not _spec_requires_host(_self, impl, ImplKind.Get) and (indices is not None or out is None):
            gout = _alloc_gpu_out(_self, impl, _devord)
            if gout is not None:
                warr = _wrap_warp(_raw(_self, impl, None, gout))
                # Only when the impl actually read into `gout` (result aliases it
                # by pointer). A GET impl that ignores `out` and returns its OWN
                # buffer (a custom CPU impl's persistent view) has
                # warr.ptr != gout.ptr -- fall through to the raw read below so we
                # don't drop the real result.
                aliases_gout = warr is not None and getattr(warr, "ptr", None) == getattr(gout, "ptr", None)
                if aliases_gout:
                    if indices is None:
                        return gout
                    # Gather the requested rows to index-size on-device, into the
                    # caller's device `out` when provided (else a fresh buffer).
                    return _gpu_gather_rows(gout, indices, _devord, out=out)
                # The impl returned its OWN buffer instead of reading into `gout`.
                if indices is None:
                    return warr
                # Indexed: gather from that buffer if it is device-resident (e.g. a
                # custom GPU GET) so the completed read is not discarded; a
                # CPU-resident warr falls through to the raw read rather than feed a
                # device-mismatched gather.
                if warr is not None and getattr(getattr(warr, "device", None), "is_cuda", False):
                    return _gpu_gather_rows(warr, indices, _devord, out=out)
        return _wrap_warp(_raw(_self, impl, indices, out))

    def set_data(
        impl: str,
        data: object | None,
        indices: object | None = None,
        _self: EntityView = view,
        _raw: Callable[..., object] = raw_set,
    ) -> None:
        """Write one tensor through a registered implementation.

        Args:
            impl: Registered setter implementation name.
            data: Tensor data to write, or ``None`` for Python callbacks that
                accept an empty data buffer.
            indices: Optional implementation-specific entity-index buffer.
                Integer index arrays must use ``int32``. An out-of-range index is
                dropped rather than rejected and its entity is left unwritten; a
                repeated index leaves the surviving value undefined. See the
                module's index contract. An empty index buffer
                performs no write.
            _self: Entity view captured by the dispatch closure.
            _raw: Native setter captured by the dispatch closure.

        Raises:
            IndexError: If ``impl`` has no registered single-buffer setter.
            RuntimeError: If the registered implementation is unsupported.
            TypeError: If an input buffer has an unsupported data type.
            ValueError: If the indices violate the implementation contract.

        """
        _require_int32_indices(indices, "indexed set")
        py_fn = _self._py_set_impls.get(impl)
        # A 0-row indexed set writes nothing -- an explicit no-op. Empty indices must not
        # reach the raw write, where "no indices" means a full write of all N rows.
        if indices is not None and len(indices) == 0:
            return
        if py_fn is not None:
            # Newton path: Warp-native and CUDA-graph-capturable. Its impl runs on the same
            # Warp stream as the producer, so it needs none of the ovphysx-only GPU-buffer
            # handling below, and a device sync here would break graph capture. Pass through.
            _validate_python_dispatch(_self, impl, ImplKind.Set, indices, "setData", False)
            py_fn(data, indices)
            return
        # ovphysx (raw) path only. Its C ABI write is streamless and host-blocking (not
        # graph-capturable regardless), so the GPU-buffer requirements below are applied
        # here and never on the Newton path above.
        #
        # ovphysx requires C-contiguous GPU tensors: make a non-contiguous GPU buffer
        # contiguous on-device (a no-op when already contiguous), never via a host round-trip.
        if isinstance(indices, wp.array) and indices.device.is_cuda and not indices.is_contiguous:
            indices = indices.contiguous()
        if isinstance(data, wp.array) and data.device.is_cuda and not data.is_contiguous:
            data = data.contiguous()
        # Scatter a compact [K, ...] buffer to full [N, ...] (the generic ovphysx write takes
        # only a full source).
        if indices is not None and getattr(data, "shape", None):
            n = int(_self.count)
            k = int(data.shape[0])
            if 0 < k < n and k == len(indices):
                data = _scatter_compact_to_full(data, indices, n)
        # Host-only impls (USD writes / CPU rotation) can't take device buffers.
        if _spec_requires_host(_self, impl, ImplKind.Set):
            data = _to_host(data)
            indices = _to_host(indices)
        _raw(_self, impl, data, indices)

    view.get_data = get_data
    view.set_data = set_data

    if raw_get_multi is not None:

        def get_data_multi(
            impl: str,
            indices: object | None = None,
            out: Sequence[object | None] = (),
            _self: EntityView = view,
            _raw: Callable[..., object] = raw_get_multi,
        ) -> list[wp.array | None]:
            """Read multiple tensors from a registered implementation.

            Args:
                impl: Registered multi-get implementation name.
                indices: Optional implementation-specific entity-index or
                    query buffer. Integer index arrays must use ``int32``. An
                    out-of-range index is dropped rather than rejected and
                    contributes zero records, which is indistinguishable from an
                    entity that had none. See the module's index contract.
                out: Optional caller-owned output buffers. Returned arrays
                    alias entries that the implementation uses as outputs.
                _self: Entity view captured by the dispatch closure.
                _raw: Native multi-getter captured by the dispatch closure.

            Returns:
                Warp arrays preserving the implementation's output order.
                Empty outputs remain ``None``.

            Raises:
                IndexError: If ``impl`` has no registered multi-buffer getter.
                RuntimeError: If the registered implementation is
                    unsupported.
                TypeError: If an input or output buffer has an unsupported
                    data type.
                ValueError: If the indices or output buffers violate the
                    implementation contract.

            """
            _require_int32_indices(indices, "indexed get")
            py_fn = _self._py_get_multi_impls.get(impl)
            if py_fn is not None:
                _validate_python_dispatch(_self, impl, ImplKind.Get, indices, "getDataMulti", True)
                return _wrap_warp(py_fn(indices, list(out or [])))
            buffers = list(out or [])
            # On a GPU sim with no caller `out`, allocate a device buffer per output from the
            # multi-spec so ovphysx's DirectGPU device check passes (it requires a device-matching
            # dst on each output). Host-only impls (requires_host_data) stay CPU-staged, mirroring
            # get_data; impls that expose no per-output specs also fall back (the alloc returns None).
            if not buffers:
                try:
                    _devord = int(_self.device_ordinal)
                except Exception:
                    _devord = -1
                if _devord >= 0 and not _spec_requires_host(_self, impl, ImplKind.Get):
                    buffers = _alloc_gpu_out_multi(_self, impl, _devord) or []
            result = _raw(_self, impl, indices, buffers)
            return _wrap_warp(result)

        view.get_data_multi = get_data_multi

    if raw_set_multi is not None:

        def set_data_multi(
            impl: str,
            data: Sequence[object] | None,
            indices: object | None = None,
            _self: EntityView = view,
            _raw: Callable[..., object] = raw_set_multi,
        ) -> None:
            """Write multiple tensors through a registered implementation.

            Args:
                impl: Registered multi-set implementation name.
                data: Tensor buffers in implementation-defined order.
                indices: Optional implementation-specific entity-index
                    buffer. Integer index arrays must use ``int32``.
                _self: Entity view captured by the dispatch closure.
                _raw: Native multi-setter captured by the dispatch closure.

            Raises:
                IndexError: If ``impl`` has no registered multi-buffer setter.
                RuntimeError: If the registered implementation is
                    unsupported.
                TypeError: If an input buffer has an unsupported data type.
                ValueError: If the indices violate the implementation
                    contract.

            """
            _require_int32_indices(indices, "indexed set")
            py_fn = _self._py_set_multi_impls.get(impl)
            buffers = list(data or [])
            if py_fn is not None:
                _validate_python_dispatch(_self, impl, ImplKind.Set, indices, "setDataMulti", True)
                py_fn(buffers, indices)
            else:
                _raw(_self, impl, buffers, indices)

        view.set_data_multi = set_data_multi

    def _shape_dim(impl_name: str, dim_idx: int, _v: EntityView = view) -> int:
        """Get one dimension from an implementation's shape hint.

        Args:
            impl_name: Registered getter implementation name.
            dim_idx: Dimension index to read.
            _v: Entity view captured by the dispatch closure.

        Returns:
            Requested dimension, or ``0`` when the implementation is
            unregistered, reports no support, or has no matching shape
            dimension.

        """
        if _v.has_impl(impl_name, ImplKind.Get):
            spec = _v.get_impl_spec(impl_name, ImplKind.Get)
            if spec is not None and len(spec.shape_hint) > dim_idx:
                return spec.shape_hint[dim_idx]
        return 0

    # Articulation shape metadata
    if view.has_impl("dof-positions", ImplKind.Get):
        view.max_dofs = _shape_dim("dof-positions", 1)
    if view.has_impl("link-transforms", ImplKind.Get):
        view.max_links = _shape_dim("link-transforms", 1)
    if view.has_impl("material-properties", ImplKind.Get):
        view.max_shapes = _shape_dim("material-properties", 1)
    if view.has_impl("fixed-tendon-stiffnesses", ImplKind.Get):
        view.max_fixed_tendons = _shape_dim("fixed-tendon-stiffnesses", 1)
    if view.has_impl("spatial-tendon-stiffnesses", ImplKind.Get):
        view.max_spatial_tendons = _shape_dim("spatial-tendon-stiffnesses", 1)

    # Contact view shape metadata
    if view.has_impl("contact-force-matrix", ImplKind.Get):
        view.filter_count = _shape_dim("contact-force-matrix", 1)
    view.sensor_count = view.count  # count is set to sensor count by contact view

    # sensor_names: prefer the backend-registered `sensor-names` metadata (the
    # authoritative resolved sensor paths, in binding-row order). A contact view
    # reports exactly its sensors -- empty for a zero-sensor (no-match) view,
    # which registers nothing, so the generic prim-path fallback would otherwise
    # surface the unmatched query glob. Only a non-contact view uses that fallback.
    _sensor_meta = view.get_metadata("sensor-names")
    if _sensor_meta is not None:
        view.sensor_names = list(_sensor_meta)
    elif view_kind == "rigid-contact":
        view.sensor_names = []
    else:
        view.sensor_names = list(view.resolved_prim_paths)

    # prim_paths: resolved prim paths after pattern expansion.
    # Use resolved_prim_paths (virtual, overridden by ovphysx) if available,
    # otherwise fall back to the construction paths.
    try:
        view.prim_paths = list(view.resolved_prim_paths)
    except AttributeError:
        view.prim_paths = list(view.paths)

    # ------------------------------------------------------------------
    # Legacy method shims
    # ------------------------------------------------------------------

    def apply_forces_and_torques_at_position(
        forces: object | None,
        torques: object | None = None,
        positions: object | None = None,
        indices: object | None = None,
        _v: EntityView = view,
    ) -> None:
        """Apply world-frame forces and torques at optional positions.

        When only forces are supplied and the view provides a pure-force
        operation, apply them at each body's center of mass. Other
        combinations are packed as force, torque, and position triples, with
        zero vectors substituted for omitted components. Supplying neither
        forces nor torques performs no operation.

        Args:
            forces: Force vectors in backend force units, or ``None`` when
                only torques are applied.
            torques: Optional torque vectors in backend torque units.
            positions: Optional application positions in stage distance units.
            indices: Optional ``int32`` entity-index buffer. Use ``None`` to
                address every entity in the view.
            _v: Entity view captured by the dispatch closure.

        """
        import numpy as _np

        if forces is None and torques is None:
            return
        _first = next(x for x in (forces, torques, positions) if x is not None)
        device = _first.device if isinstance(_first, wp.array) else "cpu"
        n = _first.shape[0] if hasattr(_first, "shape") else len(_first)

        def _to_np(x: object | None, n: int) -> object:
            if x is None:
                return _np.zeros((n, 3), dtype=_np.float32)
            if isinstance(x, wp.array):
                return x.numpy().reshape(-1, 3)
            return _np.asarray(x, dtype=_np.float32).reshape(-1, 3)

        # When no position is given, apply force at COM via the pure-force
        # binding to avoid a phantom torque from the world-origin moment arm.
        _force_impl = (
            "forces"
            if _v.has_impl("forces", ImplKind.Set)
            else "apply-forces" if _v.has_impl("apply-forces", ImplKind.Set) else None
        )
        if forces is not None and positions is None and torques is None and _force_impl is not None:
            # Pass GPU warp arrays directly to avoid GPU->CPU->GPU roundtrip.
            if isinstance(forces, wp.array) and forces.dtype == wp.float32:
                f_data = forces.reshape((-1, 3)) if forces.ndim != 2 else forces
            else:
                f_np = _to_np(forces, n)
                f_data = wp.from_numpy(f_np, dtype=wp.float32, device=device) if str(device) != "cpu" else f_np
            _v.set_data(_force_impl, f_data, indices)
            return

        f = _to_np(forces, n)
        tau = _to_np(torques, n)
        pos = _to_np(positions, n)
        wrench_np = _np.concatenate([f, tau, pos], axis=-1).astype(_np.float32)

        # Convert to GPU warp array if input forces were on GPU
        wrench = wp.from_numpy(wrench_np, dtype=wp.float32, device=device) if str(device) != "cpu" else wrench_np

        _v.set_data("apply-forces-and-torques-at-position", wrench, indices)

    view.apply_forces_and_torques_at_position = apply_forces_and_torques_at_position

    # DOF getter/setter shims
    for _impl, _method in (
        ("dof-positions", "get_dof_positions"),
        ("dof-velocities", "get_dof_velocities"),
        ("dof-position-targets", "get_dof_position_targets"),
        ("dof-velocity-targets", "get_dof_velocity_targets"),
        ("dof-actuation-forces", "get_dof_actuation_forces"),
        ("link-transforms", "get_link_transforms"),
        ("link-velocities", "get_link_velocities"),
        ("root-transforms", "get_root_transforms"),
        ("root-velocities", "get_root_velocities"),
        ("transforms", "get_transforms"),
        ("velocities", "get_velocities"),
        ("masses", "get_masses"),
        ("inv-masses", "get_inv_masses"),
    ):
        if view.has_impl(_impl, ImplKind.Get):

            def _make_getter(
                impl_key: str,
                _v: EntityView = view,
            ) -> Callable[[object | None], wp.array | None]:
                def _getter(indices: object | None = None) -> wp.array | None:
                    return _v.get_data(impl_key, indices)

                return _getter

            setattr(view, _method, _make_getter(_impl))

    # Additional getter shims not covered by the loop above
    for _impl2, _method2 in (
        ("accelerations", "get_accelerations"),
        ("articulation-mass-center", "get_articulation_mass_center"),
        ("articulation-centroidal-momentum", "get_articulation_centroidal_momentum"),
        ("contact-force-matrix", "get_contact_force_matrix"),
        ("net-contact-forces", "get_net_contact_forces"),
    ):
        if view.has_impl(_impl2, ImplKind.Get):

            def _make_getter2(
                key: str,
                _v: EntityView = view,
            ) -> Callable[[float | None, object | None], wp.array | None]:
                def _g(dt: float | None = None, indices: object | None = None) -> wp.array | None:
                    return _v.get_data(key, indices)

                return _g

            setattr(view, _method2, _make_getter2(_impl2))

    if view.has_impl("raw-contact-data", ImplKind.Get):

        def get_raw_contact_data(
            dt: float | None = None,
            _v: EntityView = view,
        ) -> list[wp.array | None]:
            """Get raw contact records and pair indexing data.

            Args:
                dt: Compatibility time-step argument. Raw contact values are
                    not scaled by this value.
                _v: Entity view captured by the dispatch closure.

            Returns:
                Seven buffers containing normal-force values, contact
                points, contact normals, separations, per-sensor contact
                counts, per-sensor start indices, and other-actor identifiers,
                in that order.

            """
            return _v.get_data_multi("raw-contact-data")

        view.get_raw_contact_data = get_raw_contact_data

    if view.has_impl("contact-data", ImplKind.Get):

        def get_contact_data(
            dt: float | None = None,
            _v: EntityView = view,
        ) -> list[wp.array | None]:
            """Get aggregated contact data and pair indexing buffers.

            Args:
                dt: Compatibility time-step argument. Contact values are not
                    scaled by this value.
                _v: Entity view captured by the dispatch closure.

            Returns:
                Six buffers containing normal-force values, contact
                points, contact normals, separations, per-sensor/filter
                contact counts, and per-sensor/filter start indices, in that
                order.

            """
            return _v.get_data_multi("contact-data")

        view.get_contact_data = get_contact_data

    # get_articulation_mass_center(local_frame=False) -- pick world vs local impl
    if view.has_impl("articulation-mass-center", ImplKind.Get):

        def get_articulation_mass_center(
            local_frame: bool = False,
            _v: EntityView = view,
        ) -> wp.array | None:
            """Get articulation centers of mass.

            Args:
                local_frame: Whether to return centers in each articulation's
                    local frame instead of the world frame.
                _v: Entity view captured by the dispatch closure.

            Returns:
                Articulation center-of-mass tensor, or ``None`` when the
                backend returns no data.

            Raises:
                IndexError: If the requested local- or world-frame operation
                    is not registered.

            """
            key = "articulation-mass-center-local" if local_frame else "articulation-mass-center"
            return _v.get_data(key)

        view.get_articulation_mass_center = get_articulation_mass_center

    # get_metatype(env_index) -- USD traversal to extract link/joint names for one env.
    # Only added for articulation views (those with dof-positions).
    if view.has_impl("dof-positions", ImplKind.Get) and not hasattr(view, "get_metatype"):

        def get_metatype(
            env_index: int = 0,
            _v: EntityView = view,
        ) -> _ArticulationMetatype | None:
            """Get resolved link and joint names for one articulation.

            The traversal includes rigid-body prims as links and joints whose
            two bodies are both set. Duplicate leaf names receive a numeric
            suffix.

            Args:
                env_index: Index into the view's resolved articulation-root
                    paths.
                _v: Entity view captured by the dispatch closure.

            Returns:
                An object with ``link_names`` and ``joint_names`` lists, or
                ``None`` when the USD stage identifier, articulation root, or
                requested index cannot be resolved.

            """
            try:
                from pxr import Usd, UsdPhysics, UsdUtils

                paths = getattr(_v, "prim_paths", [])
                root_path = paths[env_index] if env_index < len(paths) else None
                if not root_path:
                    return None
                # Use the C++-exposed usd_stage_id for exact stage lookup.
                sid = _v.usd_stage_id
                stage = None
                for s in UsdUtils.StageCache.Get().GetAllStages():
                    if sid and UsdUtils.StageCache.Get().GetId(s).ToLongInt() == sid:
                        stage = s
                        break
                    # Fallback: match by prim path when sid is 0 (non-ovphysx).
                    if not sid and s.GetPrimAtPath(root_path).IsValid():
                        stage = s
                        break
                if stage is None:
                    return None
                root = stage.GetPrimAtPath(root_path)
                if not root or not root.IsValid():
                    return None

                _JOINT_TYPES = frozenset(
                    (
                        "PhysicsRevoluteJoint",
                        "PhysicsPrismaticJoint",
                        "PhysicsFixedJoint",
                        "PhysicsSphericalJoint",
                        "PhysicsJoint",
                        "PhysicsD6Joint",
                    )
                )
                link_names, joint_names = [], []
                seen_l, seen_j = set(), set()

                def _unique(name: str, seen: set[str]) -> str:
                    n, k = name, 0
                    while n in seen:
                        n = f"{name}_{k}"
                        k += 1
                    seen.add(n)
                    return n

                for prim in Usd.PrimRange(root):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        link_names.append(_unique(prim.GetName(), seen_l))
                    if prim.GetTypeName() in _JOINT_TYPES:
                        # Exclude world-attachment joints (body0 not targeting an articulation prim)
                        j = UsdPhysics.Joint(prim)
                        b0 = j.GetBody0Rel().GetTargets() if j else []
                        b1 = j.GetBody1Rel().GetTargets() if j else []
                        if b0 and b1:  # both bodies set = inter-link joint
                            joint_names.append(_unique(prim.GetName(), seen_j))

                class _Meta:
                    pass

                m = _Meta()
                m.link_names = link_names
                m.joint_names = joint_names
                return m
            except Exception:
                return None

        view.get_metatype = get_metatype

    # wake_up / put_to_sleep -- delegate to "wake-up"/"put-to-sleep" SET impls.
    if view.has_impl("wake-up", ImplKind.Set):

        def wake_up(
            indices: object | None = None,
            _v: EntityView = view,
        ) -> None:
            """Wake selected rigid bodies or articulations.

            Args:
                indices: Optional entity index buffer. Use ``None`` to wake
                    every entity in the view.
                _v: Entity view captured by the dispatch closure.

            """
            _v.set_data("wake-up", wp.zeros(0, dtype=wp.int32), indices)

        def put_to_sleep(
            indices: object | None = None,
            _v: EntityView = view,
        ) -> None:
            """Put selected rigid bodies or articulations to sleep.

            Args:
                indices: Optional entity index buffer. Use ``None`` to affect
                    every entity in the view.
                _v: Entity view captured by the dispatch closure.

            """
            _v.set_data("put-to-sleep", wp.zeros(1, dtype=wp.float32), indices)

        view.wake_up = wake_up
        view.put_to_sleep = put_to_sleep

    # "apply-forces-and-torques-at-position" Python fallback (when C++ did not
    # register the impl, e.g. transforms binding failed or view is articulation).
    if not view.has_impl("apply-forces-and-torques-at-position", ImplKind.Set):
        _ensure_py_impl_mirror(view)
        if view.has_impl("link-wrenches", ImplKind.Set):

            def _arti_force_at_pos(
                data: object,
                indices: object | None = None,
                _v: EntityView = view,
            ) -> None:
                if (isinstance(data, wp.array) and data.ndim == 2) or (
                    not isinstance(data, wp.array) and hasattr(data, "ndim") and data.ndim == 2
                ):
                    N = _v.count
                    L = (data.shape[0] // N) if N > 0 else 0
                    out = data.reshape((N, L, 9))
                else:
                    out = data
                _v.set_data("link-wrenches", out, indices)

            view._py_set_impls["apply-forces-and-torques-at-position"] = _arti_force_at_pos
        elif view.has_impl("wrenches", ImplKind.Set):

            def _rb_force_at_pos(
                data: object,
                indices: object | None = None,
                _v: EntityView = view,
            ) -> None:
                if isinstance(data, wp.array):
                    out = data.reshape(-1, 9) if data.ndim > 2 else data
                else:
                    import numpy as _np

                    out = _np.asarray(data, dtype=_np.float32).reshape(-1, 9)
                _v.set_data("wrenches", out, indices)

            view._py_set_impls["apply-forces-and-torques-at-position"] = _rb_force_at_pos

    view._warp_dispatch_attached = True
    return view


def create_entity(
    engine: str,
    entity_name: str,
    paths: str | Sequence[str],
    options: dict[str, object] | None = None,
    device_ordinal: int | None = None,
) -> EntityView | None:
    """Create a Warp-backed entity view from a registered engine factory.

    Args:
        engine: Exact, case-sensitive name of a registered simulation.
        entity_name: Registered entity type, such as ``"articulation"`` or
            ``"rigid-contact"``.
        paths: One prim-path pattern, or a sequence of patterns and explicit
            prim paths, selecting the entities. Elements are passed to the
            engine verbatim as globs. A single pattern is expanded by the
            engine; several are resolved and concatenated in the given order.
        options: Engine-defined construction arguments that paths alone cannot
            express, such as a contact view's ``"filter-patterns"`` and
            ``"max-contact-data-count"`` or an SDF view's ``"num-points"``. An
            engine ignores the option names it does not read.
        device_ordinal: Ordinal of the device holding the view's tensors, or
            ``None`` to leave the engine's own value in place. Only engines
            that cannot report their device need this; the value may also be
            assigned later through the view's ``device_ordinal`` attribute,
            since each read re-reads it.

    Returns:
        Entity view augmented with Warp dispatch, or ``None`` when the
        registered factory elects not to create one. A valid empty view is
        returned unchanged rather than converted to ``None``.

    Raises:
        ValueError: If ``paths`` is empty or contains an empty pattern.
        IndexError: If ``engine`` has no factory for ``entity_name``.

    """
    # A bare pattern is one path, not a sequence of characters. `str` satisfies `Sequence[str]`, so
    # list() would otherwise split it silently into a per-character path list that matches nothing.
    path_list = [paths] if isinstance(paths, str) else list(paths)
    # Reject an empty selection, whether written as an empty list or an empty pattern: neither has
    # anything to resolve, and an empty pattern is the ambiguous match-nothing/match-all case in C++.
    if not path_list or any(not path for path in path_list):
        raise ValueError(f"create_entity: {entity_name!r} requires non-empty prim-path patterns")
    view = _create_entity_raw(engine, entity_name, path_list, options)
    if view is not None and device_ordinal is not None:
        view.device_ordinal = int(device_ordinal)
    return _attach_warp_dispatch(view, entity_name)


__all__ = [
    "DType",
    "DeviceKind",
    "DofDriveType",
    "DofMotion",
    "DofType",
    "EntityView",
    "Float3",
    "ImplKind",
    "JointType",
    "ObjectType",
    "TensorDesc",
    "TensorRegistry",
    "TensorSpec",
    "create_entity",
    "frontends",
    "get_registry",
    "register_entity",
]
