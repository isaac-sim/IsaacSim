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

"""Verify Newton releases its native registry callbacks before interpreter shutdown."""

from __future__ import annotations

import subprocess
import sys

import pytest


@pytest.mark.parametrize("explicit_cleanup", [False, True])
def test_registry_shutdown_releases_python_factories(explicit_cleanup: bool) -> None:
    """Release owned registrations without clearing factories belonging to other engines."""
    script = """
from isaacsim.physics_engines.ovnewton.impl.tensors import entity_factories as factories
from isaacsim.physics.manager.impl import tensors as t
"""
    if explicit_cleanup:
        script += """
factories.register_with_umbrella("shutdown-second")
t.register_entity("other-engine", "entity", lambda paths, options=None: t.EntityView(paths))
factories._unregister_at_shutdown()
assert not factories._REGISTERED
view = t.create_entity("other-engine", "entity", ["/World"])
assert view is not None
t.get_registry().unregister_entity("other-engine", "entity")
del view
factories.register_with_umbrella("shutdown-again")
"""
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=30, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "nanobind: leaked" not in result.stderr, result.stderr
