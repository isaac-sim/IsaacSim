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

"""Implement the OvPhysX simulation backend.

The backend wraps the OvPhysX SDK behind the physics manager simulation and
tensor APIs. Importing :mod:`isaacsim.physics_engines.ovphysx` activates and
registers the native backend. This implementation module retains explicit
lifecycle and readback helpers for compatibility and testing.
"""

from __future__ import annotations

from types import ModuleType

_bindings: ModuleType | None = None


def _backend() -> ModuleType:
    """Load and return the native OvPhysX bindings.

    Returns:
        Cached native bindings module.

    """
    global _bindings
    if _bindings is None:
        from isaacsim.physics_engines import ovstage

        # OVStage has to be in place before the binding loads: libovphysx imports it
        # by basename on Windows and by SONAME on Linux, and neither reaches a copy
        # that is not beside it. setup() registers the DLL directories on Windows and
        # loads the library globally on Linux, so the binding's import resolves to
        # that one instance either way.
        ovstage.setup()
        from isaacsim.physics_engines.ovphysx.bindings import _bindings as native

        # This does not import or modify process-wide `pxr`.
        _bindings = native
    return _bindings


def activate() -> bool:
    """Activate and register the OvPhysX backend if needed.

    Returns:
        True when the native backend reports successful activation.

    """
    return _backend().activate()


def shutdown() -> None:
    """Shut down the OvPhysX backend if its bindings were loaded."""
    if _bindings is not None:
        _bindings.shutdown()


def set_suppress_readback(enable: bool) -> None:
    """Set the process-wide PhysX readback-suppression option.

    Args:
        enable: True to suppress CPU readback for the DirectGPU workflow.

    """
    _backend().set_suppress_readback(enable)


def set_tensor_device_ordinal(ordinal: int) -> None:
    """Declare the device holding this backend's tensors.

    OvPhysX exposes no query for the device its scene selected, so the caller that configured the tensor
    device states it here. Every entity view reports this value and re-reads it on each access, so it may
    be declared after those views exist.

    Args:
        ordinal: CUDA device ordinal, or -1 for host memory.

    """
    _backend().set_tensor_device_ordinal(ordinal)


def get_tensor_device_ordinal() -> int:
    """Get the declared tensor-device ordinal.

    Returns:
        The ordinal passed to :func:`set_tensor_device_ordinal`, or -1 when none was declared.

    """
    return _backend().get_tensor_device_ordinal()


__all__ = [
    "activate",
    "shutdown",
    "set_suppress_readback",
    "set_tensor_device_ordinal",
    "get_tensor_device_ordinal",
]

if not activate():
    raise RuntimeError("OvPhysX backend activation failed.")
