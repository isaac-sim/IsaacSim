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

"""Provide core simulation management functionality for Isaac Sim including physics scenes, event handling, and simulation state control."""

from __future__ import annotations

from typing import Any

__all__ = [
    "Extension",
    "IsaacEvents",
    "PhysicsScene",
    "PhysxGpuCfg",
    "PhysxScene",
    "SimulationEvent",
    "SimulationManager",
]

import omni.ext
import omni.kit.app

from .. import _simulation_manager
from .isaac_events import IsaacEvents
from .physics_scene import PhysicsScene
from .physx_scene import PhysxGpuCfg, PhysxScene
from .simulation_event import SimulationEvent
from .simulation_manager import SimulationManager

# expose pybind interface/API
_simulation_manager_interface = None


def acquire_simulation_manager_interface() -> Any:
    """Acquires the simulation manager interface from the isaacsim.core.simulation_manager extension.

    Returns:
        The simulation manager interface instance, or None if not initialized.
    """
    return _simulation_manager_interface


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.core.simulation_manager extension.

    This extension provides core simulation management functionality for Isaac Sim, including physics scene
    management, event handling, and simulation state control. It exposes a comprehensive API for managing
    simulation lifecycles, physics configurations, and event-driven simulation workflows.

    The extension initializes the simulation manager interface and provides access to key simulation
    components including physics scenes, simulation events, and GPU-accelerated physics configurations.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is started.

        Acquires the pybind simulation manager interface and initializes the SimulationManager.

        Args:
            ext_id: The extension identifier.
        """
        # acquire the pybind interface
        global _simulation_manager_interface
        _simulation_manager_interface = _simulation_manager.acquire_simulation_manager_interface()
        SimulationManager._simulation_manager_interface = _simulation_manager_interface
        SimulationManager._startup()

    def on_shutdown(self) -> None:
        """Called when the extension is shut down.

        Shuts down the SimulationManager and releases the pybind simulation manager interface.
        """
        # release the pybind interface
        global _simulation_manager_interface
        SimulationManager._shutdown()
        _simulation_manager.release_simulation_manager_interface(_simulation_manager_interface)
        _simulation_manager_interface = None
