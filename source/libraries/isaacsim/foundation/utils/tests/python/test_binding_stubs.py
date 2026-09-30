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

"""Verify that generated utility binding stubs expose public Python type names."""

from __future__ import annotations

import ast
from pathlib import Path

from isaacsim.foundation.utils.bindings import _bindings


def test_generated_stage_annotations_use_public_python_type() -> None:
    """Stage helpers must name the public facade instead of the native C++ type."""
    stub_path = Path(_bindings.__file__).with_name("_bindings.pyi")
    stub_text = stub_path.read_text(encoding="utf-8")
    ast.parse(stub_text, filename=str(stub_path))

    stage_type = "isaacsim.foundation.objects.Stage"
    assert f"def set_default_stage(stage: {stage_type}) -> None:" in stub_text
    assert f"def get_default_stage() -> {stage_type}:" in stub_text
    assert f"def get_active_stage() -> {stage_type}:" in stub_text
    assert "isaacsim::" not in stub_text
    assert "std::" not in stub_text
