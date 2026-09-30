# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test physics manager against ovphysx behavior."""

import ctypes
import os
from typing import Any

import _physics_setup  # noqa: F401  (registers the ovphysx backend)
from isaacsim.physics.manager import PhysicsManager
from pxr import Usd, UsdUtils

_ASSETS = os.environ["ISAACSIM_TEST_RESOURCE_ROOT"]


def _build_ovstage(usd_path: str) -> tuple[object, int, int]:
    disk_stage = Usd.Stage.Open(usd_path)
    usd_stage_id = UsdUtils.StageCache.Get().Insert(disk_stage).ToLongInt()

    import ovstage

    ovstage_stage = ovstage.Stage("physmgr-test")
    ovstage.population.open_usd(
        ovstage_stage,
        usd_path,
        ordinal=1,
        time_code=0.0,
        domains=ovstage.PopulationDomain.RENDERING | ovstage.PopulationDomain.PHYSICS,
    )
    ovstage_stage.advance_write_floor(1).wait()
    handle = ctypes.cast(ovstage_stage._inst, ctypes.c_void_p).value
    return ovstage_stage, handle, usd_stage_id


"""
Test cases.
"""


def test_physics_manager(capsys: Any) -> None:
    """Verify initialization, stepping, and state queries via PhysicsManager.

    Args:
        capsys: Pytest output-capture fixture.

    """
    _physics_setup.set_suppress_readback(False)  # CPU tensors
    _, handle, usd_stage_id = _build_ovstage(os.path.join(_ASSETS, "CartPole.usda"))

    physics_manager = PhysicsManager.get_instance()
    physics_manager.switch_physics_engine("ovphysx")
    physics_manager.setup(1.0 / 60.0)
    assert physics_manager.initialize(handle, usd_stage_id), "physics initialize failed"

    start_steps = physics_manager.get_simulated_physics_steps()
    assert start_steps == 0, "step counter not reset by initialize()"
    physics_manager.step(steps=5)
    assert physics_manager.get_simulated_physics_steps() - start_steps == 5
    assert physics_manager.get_simulated_time() > 0.0

    physics_manager.invalidate()
