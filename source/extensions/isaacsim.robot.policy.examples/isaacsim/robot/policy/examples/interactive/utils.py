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

"""Shared helpers for the interactive policy examples."""

from __future__ import annotations

import carb
from isaacsim.core.simulation_manager import SimulationManager


def snapshot_physics_simulation_state() -> tuple[str | None, bool | None]:
    """Capture the current physics sim device and fabric-enabled flag.

    Returns:
        A ``(prev_physics_sim_device, prev_fabric_enabled)`` tuple. Either element is ``None``
        if the corresponding state could not be queried.
    """
    prev_device: str | None
    prev_fabric: bool | None
    try:
        prev_device = SimulationManager.get_physics_sim_device()
    except Exception as e:
        carb.log_warn(f"Could not snapshot physics sim device: {e}")
        prev_device = None
    try:
        prev_fabric = SimulationManager.is_fabric_enabled()
    except Exception as e:
        carb.log_warn(f"Could not snapshot fabric enabled state: {e}")
        prev_fabric = None
    return prev_device, prev_fabric


def restore_physics_simulation_state(prev_device: str | None, prev_fabric_enabled: bool | None) -> None:
    """Restore physics sim device and fabric-enabled flag captured via ``snapshot_physics_simulation_state``.

    Setting the device to ``cuda`` enables fabric and PhysX direct-GPU API; leaving that
    state in place after an example is cleared causes errors (e.g. ``setDriveTarget`` is
    illegal with ``PxSceneFlag::eENABLE_DIRECT_GPU_API``) the next time the user modifies
    USD while a simulation is running.

    Args:
        prev_device: Device string to restore (e.g. ``"cpu"`` or ``"cuda:0"``). Ignored if ``None``.
        prev_fabric_enabled: Fabric-enabled flag to restore. Ignored if ``None``.
    """
    try:
        if prev_device is not None:
            SimulationManager.set_physics_sim_device(prev_device)
        if prev_fabric_enabled is not None:
            SimulationManager.enable_fabric(prev_fabric_enabled)
    except Exception as e:
        carb.log_warn(f"Could not restore physics simulation state: {e}")
