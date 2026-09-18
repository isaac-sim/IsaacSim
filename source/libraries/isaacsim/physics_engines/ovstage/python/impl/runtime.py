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

"""Configure the private OVStage runtime for the current process.

The runtime root contains the curated, ctypes-based ``ovstage`` package under
``ovstage_py/`` and the physics-owned native SDK under ``ovphysx/``.
:func:`setup` makes those two components discoverable without importing or
modifying the process-wide ``pxr`` package. OVStage's namespaced OpenUSD
runtime remains private to its native library.
"""

from __future__ import annotations

import ctypes
import functools
import os
import sys
import weakref
from pathlib import Path

_OVSTAGE_LIBRARY_NAME = "ovstage.dll" if sys.platform == "win32" else "libovstage.so"

_done = False
_dll_directory_handles: list[object] = []
_windows_native_root: Path | None = None
# Map native ovstage_instance_t addresses to the owning Python Stage. Physics
# backends receive only the integer handle from physics_manager.initialize; Python
# consumers such as PyPI ovnewton.attach_ovstage need the Stage object.
_stages_by_handle: weakref.WeakValueDictionary[int, object] = weakref.WeakValueDictionary()


class _DlInfo(ctypes.Structure):
    """Linux ``Dl_info`` result used to locate an already-loaded library."""

    _fields_ = [
        ("dli_fname", ctypes.c_char_p),
        ("dli_fbase", ctypes.c_void_p),
        ("dli_sname", ctypes.c_char_p),
        ("dli_saddr", ctypes.c_void_p),
    ]


def _runtime_roots() -> list[Path]:
    """Get the installed Isaac Sim runtime roots.

    Returns:
        Runtime roots from every entry in the ``isaacsim`` namespace package.

    """
    import isaacsim

    return [Path(entry) / "lib" for entry in isaacsim.__path__]


def _find_runtime_root() -> Path:
    """Find the runtime containing the curated OVStage Python package.

    Returns:
        Runtime root containing ``ovstage_py``.

    Raises:
        RuntimeError: If the packaged OVStage runtime is missing.

    """
    for root in _runtime_roots():
        if (root / "ovstage_py" / "ovstage" / "__init__.py").is_file():
            return root
    raise RuntimeError("Bundled OVStage Python runtime was not found under the isaacsim namespace package.")


def _loaded_ovstage_root() -> Path | None:
    """Get the directory of an already-loaded Linux OVStage library.

    Returns:
        Library directory, or None when OVStage has not been loaded.

    """
    if sys.platform != "linux":
        return None
    try:
        library = ctypes.CDLL(_OVSTAGE_LIBRARY_NAME, mode=os.RTLD_NOLOAD | ctypes.RTLD_LOCAL)
    except OSError:
        return None

    dladdr = ctypes.CDLL(None).dladdr
    dladdr.argtypes = [ctypes.c_void_p, ctypes.POINTER(_DlInfo)]
    dladdr.restype = ctypes.c_int
    info = _DlInfo()
    symbol = ctypes.cast(library.ovstage_initialize, ctypes.c_void_p)
    if not dladdr(symbol, ctypes.byref(info)) or not info.dli_fname:
        return None
    return Path(os.fsdecode(info.dli_fname)).resolve().parent


def _configure_windows_runtime() -> Path:
    """Configure the Windows loader for the packaged OvPhysX runtime.

    Returns:
        Directory containing the OVStage library loaded by OvPhysX.

    Raises:
        RuntimeError: If the packaged OvPhysX bindings are missing.

    """
    import isaacsim

    for entry in isaacsim.__path__:
        binding_root = Path(entry) / "physics_engines" / "ovphysx" / "bindings"
        if not (binding_root / "ovphysx.dll").is_file():
            continue
        for runtime_directory in (
            binding_root,
            binding_root / "plugins",
            binding_root.parent / "plugins",
            binding_root.parent / "plugins" / "bin" / "deps",
        ):
            if runtime_directory.is_dir():
                _dll_directory_handles.append(os.add_dll_directory(runtime_directory))
        return binding_root
    raise RuntimeError("Bundled OvPhysX Windows runtime was not found under the isaacsim namespace package.")


def setup() -> None:
    """Make the private OVStage runtime discoverable.

    Repeated calls are safe. On Linux, each call refreshes the loader hint to
    select an OVStage library that an engine binding has loaded since the prior
    call. This function does not import or modify ``pxr``.

    Raises:
        RuntimeError: If the bundled runtime is missing.

    Example:

    .. code-block:: python

        >>> from isaacsim.physics_engines.ovstage import setup

        >>> setup()

    """
    global _done, _windows_native_root
    runtime_root = _find_runtime_root()
    ovstage_python = runtime_root / "ovstage_py"
    if str(ovstage_python) not in sys.path:
        sys.path.insert(0, str(ovstage_python))

    if sys.platform == "win32":
        if not _done:
            _windows_native_root = _configure_windows_runtime()
        if _windows_native_root is None:
            raise RuntimeError("Bundled OvPhysX Windows runtime was not configured.")
        native_root = _windows_native_root
    else:
        native_root = _loaded_ovstage_root() or runtime_root / "ovphysx" / "bin"
        native_library = native_root / _OVSTAGE_LIBRARY_NAME
        if not native_library.is_file():
            raise RuntimeError(f"Bundled OVStage runtime not found at {native_library}")
        # Load it globally so the backend's NEEDED entry binds to this copy by SONAME.
        # An installed layout puts the library beside libovphysx and $ORIGIN finds it,
        # but the SDK package the build tree links against has no such sibling.
        ctypes.CDLL(str(native_library), mode=ctypes.RTLD_GLOBAL)

    # The curated Python package intentionally excludes its duplicate native
    # payload. Point its documented loader hint at the OVStage library already
    # packaged with the OvPhysX backend.
    os.environ["OVSTAGE_LIBRARY_PATH_HINT"] = str(native_root)
    _done = True


def get_native_handle(stage: object) -> int:
    """Get the native address for a public OVStage object.

    Registers ``stage`` so :func:`lookup_stage` can recover it from the integer
    handle passed through ``physics_manager.initialize``.

    Args:
        stage: OVStage object whose native address is required by a consumer.

    Returns:
        Native OVStage address.

    Raises:
        TypeError: If `stage` is not an OVStage-compatible object.
        RuntimeError: If the OVStage object has been destroyed.

    """
    instance = getattr(stage, "_inst", None)
    if instance is None:
        raise TypeError("stage must be an OVStage object")
    handle = ctypes.cast(instance, ctypes.c_void_p).value
    if handle is None:
        raise RuntimeError("OVStage object has been destroyed")
    try:
        _stages_by_handle[int(handle)] = stage
    except TypeError:
        # Registration is a convenience for `lookup_stage`; a Stage type without
        # weakref support must not break the documented handle contract.
        pass
    return int(handle)


@functools.lru_cache(maxsize=1)
def _capsule_accessors() -> tuple[object, object]:
    """Configure the PyCapsule C-API accessors once.

    ``ctypes.pythonapi`` caches one function object per symbol process-wide, so the
    signatures are set a single time instead of on every handle conversion.

    Returns:
        The configured ``PyCapsule_GetPointer`` and ``PyCapsule_GetName`` callables.

    """
    get_pointer = ctypes.pythonapi.PyCapsule_GetPointer
    get_pointer.restype = ctypes.c_void_p
    get_pointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    get_name = ctypes.pythonapi.PyCapsule_GetName
    get_name.restype = ctypes.c_char_p
    get_name.argtypes = [ctypes.py_object]
    return get_pointer, get_name


def as_native_handle(value: object | None) -> int:
    """Normalize a native ovstage address from an int or nanobind ``void*`` capsule.

    ``physics_manager.initialize`` accepts an ``int`` handle, but when the manager
    invokes a Python ``SimulationFns.initialize`` callback the C++ ``void*`` is
    delivered as a nanobind ``PyCapsule`` named ``nb_handle``.

    Args:
        value: Native address as ``int``, nanobind ``PyCapsule``, or ``None``/``0``.

    Returns:
        Native address as ``int``, or ``0`` when ``value`` is null.

    Raises:
        TypeError: If ``value`` is neither an integer nor a recognized capsule.

    """
    if value is None:
        return 0
    if isinstance(value, int):
        # `int()` normalizes bool and int subclasses to the documented plain-int return.
        return int(value)

    # nanobind void* → PyCapsule (named "nb_handle" today). PyCapsule_GetPointer
    # requires an exact name match, so read the capsule's own name rather than
    # guessing; that accepts any capsule name a future nanobind might use.
    get_pointer, get_name = _capsule_accessors()
    try:
        pointer = get_pointer(value, get_name(value))
    except Exception as exc:
        raise TypeError(f"ovstage handle must be int or PyCapsule, got {type(value).__name__}") from exc
    return int(pointer) if pointer else 0


def lookup_stage(handle: object | None) -> object | None:
    """Look up a Python OVStage previously registered via :func:`get_native_handle`.

    Args:
        handle: Native OVStage address as ``int`` or nanobind ``void*`` capsule.

    Returns:
        Owning Python Stage when still alive, otherwise None.

    """
    native = as_native_handle(handle)
    if not native:
        return None
    stage = _stages_by_handle.get(native)
    if stage is None:
        return None
    # The registry is keyed by native address, and the weak reference only expires when
    # the *Python* Stage dies. A Stage destroyed via `destroy()` keeps its entry (with a
    # freed native instance), and a new Stage may be allocated at the same address before
    # `get_native_handle` re-registers it. Re-read the address so a stale entry is never
    # handed back for attach -- that would dereference a dangling pointer natively.
    instance = getattr(stage, "_inst", None)
    try:
        alive = instance is not None and ctypes.cast(instance, ctypes.c_void_p).value == native
    except (ctypes.ArgumentError, TypeError):
        # `destroy()` may leave `_inst` as a non-pointer sentinel. Treat that as dead
        # instead of letting ctypes raise out of the physics `initialize` callback,
        # where the exception would cross the nanobind boundary.
        alive = False
    if not alive:
        _stages_by_handle.pop(native, None)
        return None
    return stage
