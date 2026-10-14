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

"""Configure the pip-owned OVStage and OvPhysX runtimes for this process.

:func:`setup` loads the native libraries supplied by the declared ``ovstage``
and ``ovphysx`` package dependencies before an Isaac Sim binding is imported.
It also registers OVNewton's required USD schemas before anything can populate
the stage. This keeps OVStage's namespaced OpenUSD runtime private and does not
import or modify the process-wide ``pxr`` package.
"""

from __future__ import annotations

import ctypes
import functools
import importlib.util
import os
import sys
import weakref
from importlib.metadata import distribution
from pathlib import Path

_OVSTAGE_LIBRARY_NAME = "ovstage.dll" if sys.platform == "win32" else "libovstage.so"
_OVSTAGE_LOADER_NAME = "ovstage-dynamic.dll" if sys.platform == "win32" else "libovstage-dynamic.so"
_OVPHYSX_LIBRARY_NAME = "ovphysx.dll" if sys.platform == "win32" else "libovphysx.so"

_done = False
_dll_directory_handles: list[object] = []
_native_library_handles: list[ctypes.CDLL] = []
# Map native ovstage_instance_t addresses to the owning Python Stage. Physics
# backends receive only the integer handle from physics_manager.initialize; Python
# consumers such as PyPI ovnewton.attach_ovstage need the Stage object.
_stages_by_handle: weakref.WeakValueDictionary[int, object] = weakref.WeakValueDictionary()


def _package_root(package: str) -> Path:
    """Get an installed dependency package's root without importing it.

    Returns:
        Installed package directory.

    Raises:
        RuntimeError: If the required dependency is not installed as a package.

    """
    spec = importlib.util.find_spec(package)
    if spec is None or not spec.submodule_search_locations:
        raise RuntimeError(f"Required Python package '{package}' is not installed.")
    return Path(next(iter(spec.submodule_search_locations))).resolve()


def _require_library(path: Path) -> Path:
    """Validate and return a dependency-owned native library path."""
    if not path.is_file():
        raise RuntimeError(f"Required native dependency was not found: {path}")
    return path


def _load_library(path: Path) -> None:
    """Load a dependency-owned native library and retain its handle."""
    mode = None if sys.platform == "win32" else ctypes.RTLD_GLOBAL
    if mode is None:
        library = ctypes.CDLL(str(_require_library(path)))
    else:
        library = ctypes.CDLL(str(_require_library(path)), mode=mode)
    _native_library_handles.append(library)


def _configure_windows_runtime(ovphysx_root: Path, ovstage_root: Path) -> None:
    """Register every directory needed by the dependency wheels' DLL closures."""
    directories = [
        ovphysx_root / "lib",
        ovphysx_root / "plugins",
        ovstage_root / "bin",
        ovstage_root / "bin" / "plugins",
    ]
    plugins = ovstage_root / "bin" / "plugins"
    if plugins.is_dir():
        directories.extend(path for path in plugins.iterdir() if path.is_dir())
    for directory in directories:
        if directory.is_dir():
            _dll_directory_handles.append(os.add_dll_directory(directory))


def setup() -> None:
    """Load the declared native runtimes and register their USD schemas.

    Repeated calls are safe. Schema registration occurs before the OvPhysX
    runtime is loaded. This function does not import or modify ``pxr``.

    Raises:
        RuntimeError: If a dependency wheel or its native runtime is missing.

    Example:

    .. code-block:: python

        >>> from isaacsim.physics_engines.ovstage import setup

        >>> setup()

    """
    global _done
    if _done:
        return

    ovstage_root = _package_root("ovstage")
    ovphysx_root = _package_root("ovphysx")
    ovstage_bin = ovstage_root / "bin"
    ovphysx_lib = ovphysx_root / "lib"
    os.environ["OVSTAGE_LIBRARY_PATH_HINT"] = str(ovstage_bin)

    if sys.platform == "win32":
        _configure_windows_runtime(ovphysx_root, ovstage_root)

    # The Isaac adapter links the OVStage loader and OvPhysX directly. Preload
    # all three SONAMEs/DLLs before importing its nanobind module so the dynamic
    # loader reuses the dependency wheels' copies.
    _load_library(ovstage_bin / _OVSTAGE_LOADER_NAME)
    _load_library(ovstage_bin / _OVSTAGE_LIBRARY_NAME)

    # Register through OVStage: public OVNewton 0.1.1 has no registration helper.
    # Locate plugin data without importing schemas into the process-wide USD.
    from ovstage import population  # noqa: PLC0415

    population.register_usd_schemas(
        [
            distribution("newton-usd-schemas").locate_file("newton_usd_schemas/plugInfo.json"),
            distribution("physx-usd-schemas").locate_file("physx_usd_schemas/PhysxSchema/resources/plugInfo.json"),
        ]
    )
    _load_library(ovphysx_lib / _OVPHYSX_LIBRARY_NAME)
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
