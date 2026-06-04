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

"""PINK (Python Inverse Kinematics) integration for Isaac Sim motion generation."""

import sys
from pathlib import Path

# Fallback: ensure the cmeel-installed pinocchio sitelib is on sys.path even if the
# [[python.module]] path entry in extension.toml was not processed
_CMEEL_SITELIB = (
    Path(__file__).parents[3]
    / "pip_prebundle"
    / "cmeel.prefix"
    / "lib"
    / f"python{sys.version_info.major}.{sys.version_info.minor}"
    / "site-packages"
)
if _CMEEL_SITELIB.is_dir() and str(_CMEEL_SITELIB) not in sys.path:
    sys.path.append(str(_CMEEL_SITELIB))

from .extension import *

try:
    from .impl import *
except ModuleNotFoundError as _exc:
    if "pinocchio_pywrap_default" not in str(_exc):
        raise
    # The pinocchio submodules (rpy, cholesky, …) were registered
    # in sys.modules under truncated keys.
    _pywrap = sys.modules.get("pinocchio.pinocchio_pywrap_default")
    if _pywrap is None:
        raise
    import inspect as _inspect

    _fixups = {}
    for _name, _obj in _inspect.getmembers(_pywrap, _inspect.ismodule):
        _fixups[f"pinocchio.pinocchio_pywrap_default.{_name}"] = _obj

    # Purge all pinocchio entries so the retry gets a fresh module.
    for _key in [k for k in sys.modules if k == "pinocchio" or k.startswith("pinocchio.")]:
        del sys.modules[_key]
    for _key in [k for k in sys.modules if k.startswith("isaacsim.robot_motion.pink.impl")]:
        del sys.modules[_key]

    sys.modules.update(_fixups)
    del _inspect, _fixups, _pywrap
    from .impl import *

__all__ = [
    "load_pink_robot",
    "load_pink_supported_robot",
    "PinkRobot",
    "PinkIKController",
]
