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

"""Verify that generated physics binding stubs contain valid Python-facing type names."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path


def test_generated_binding_stubs_use_python_type_names() -> None:
    """Generated annotations must not expose C++ namespace or standard-library spellings."""
    binding_modules = (
        "isaacsim.physics.registration.bindings._bindings",
        "isaacsim.physics.manager.bindings._bindings",
    )
    for module_name in binding_modules:
        module = importlib.import_module(module_name)
        stub_path = Path(module.__file__).with_name("_bindings.pyi")
        stub_text = stub_path.read_text(encoding="utf-8")
        ast.parse(stub_text, filename=str(stub_path))
        assert "::" not in stub_text, f"{stub_path} exposes a C++ type name"
