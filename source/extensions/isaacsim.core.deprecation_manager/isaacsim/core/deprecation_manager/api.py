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

"""Functions for importing Python packages and modules with enhanced error handling and deprecation notices."""

from __future__ import annotations

import importlib
import sys
from types import ModuleType

import carb
import omni.kit.app


class _StubModule:
    """No-op mock module returned during stub generation when a dependency is unavailable.

    Attribute access returns another ``_StubModule``, and calling an instance
    acts as an identity decorator so that patterns like ``@torch.jit.script``
    do not crash.
    """

    def __getattr__(self, _name: str) -> _StubModule:
        return _StubModule()

    def __call__(self, *args, **kwargs) -> object:  # noqa: ANN002, ANN003
        if args and callable(args[0]):
            return args[0]
        return _StubModule()


def import_module(name: str) -> ModuleType:
    """Try to import a Python package/module and return it.

    If a package or module is not found, an error message will be logged and the application will exit.
    A notice with further instructions will also be logged for the following packages:

    - ``torch``

    Args:
        name: The name of the package/module to import.

    Returns:
        The imported module.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.deprecation_manager import import_module
        >>>
        >>> numpy = import_module("numpy")
    """

    def exit_app() -> None:
        if carb.settings.get_settings().get_as_bool("/app/stubgen/enabled"):
            return
        if carb.settings.get_settings().get_as_bool("/exts/omni.kit.test/runTestsAndQuit"):
            sys.exit(1)
        omni.kit.app.get_app().shutdown()

    # PyTorch
    if name == "torch":
        try:
            return importlib.import_module(name)
        except (ModuleNotFoundError, ImportError) as e:
            if carb.settings.get_settings().get_as_bool("/app/stubgen/enabled"):
                return _StubModule()
            msg = """
============================================================================
========================== IMPLEMENTATION NOTICE ===========================
============================================================================

PyTorch (torch) dependency is not installed/enabled by default in Isaac Sim.
Please, follow the instructions below to install and use it.

For a specific PyTorch version, see: https://pytorch.org/get-started/locally
----------------------------------------------------------------------------

 * Isaac Sim - Binary installation (Linux: ./python.sh, Windows: python.bat):
  
    ./python.sh -m pip install torch
  
 * Isaac Sim - Python Packages (pip) installation:
  
    pip install torch

============================================================================
"""
            carb.log_error(f"Import error: {str(e)}")
            carb.log_warn(msg)
            exit_app()
    # any other module
    else:
        try:
            return importlib.import_module(name)
        except (ModuleNotFoundError, ImportError) as e:
            if carb.settings.get_settings().get_as_bool("/app/stubgen/enabled"):
                return _StubModule()
            carb.log_error(f"Import error: {str(e)}")
            exit_app()
