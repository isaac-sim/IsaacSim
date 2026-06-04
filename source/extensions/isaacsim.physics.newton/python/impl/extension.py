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

"""Newton physics simulation extension for Isaac Sim."""

from __future__ import annotations

import carb
import omni.ext
from isaacsim.core.simulation_manager import SimulationManager
from omni.physics.core import get_physics_interface, k_invalid_simulation_id

from .interface import NewtonPhysicsInterface
from .newton_config import NewtonConfig
from .newton_stage import NewtonStage
from .register_simulation import NewtonSimulationRegistry

# Global public interface objects
_newton_physics_interface = None
_newton_stage = None


def acquire_physics_interface() -> NewtonPhysicsInterface | None:
    """Get the Newton physics interface.

    Returns:
        The physics interface for controlling simulation, or None if not initialized.
    """
    return _newton_physics_interface


def acquire_stage() -> NewtonStage | None:
    """Get the Newton simulation stage.

    Function name has a typo ("acuire") but is kept for backward compatibility.

    Returns:
        The simulation stage object, or None if not initialized.
    """
    return _newton_stage


def get_active_physics_engine() -> str:
    """Get the name of the currently active physics engine.

    Returns:
        Name of the active engine ("newton", "physx", etc.) or "Unknown" if none active.
    """
    try:
        physics = get_physics_interface()
        if not physics:
            return "Unknown"

        simulation_ids = physics.get_simulation_ids()
        for sim_id in simulation_ids:
            if physics.is_simulation_active(sim_id):
                return physics.get_simulation_name(sim_id)

        return "Unknown"
    except Exception:
        return "Unknown"


def get_available_physics_engines(verbose: bool = False) -> list[tuple[str, bool]]:
    """Get list of all available physics engines.

    Args:
        verbose: If True, print available engines to console.

    Returns:
        List of tuples (engine_name, is_active) for all registered engines.
    """
    try:
        physics = get_physics_interface()
        if not physics:
            return []

        engines = []
        simulation_ids = physics.get_simulation_ids()
        for sim_id in simulation_ids:
            sim_name = physics.get_simulation_name(sim_id)
            is_active = physics.is_simulation_active(sim_id)
            engines.append((sim_name, is_active))

        if verbose:
            print("Available physics engines:")
            for engine in engines:
                print(f"  {engine[0]}: {'active' if engine[1] else 'inactive'}")
            print("-" * 60)

        return engines
    except Exception:
        return []


class NewtonSimExtension(omni.ext.IExt):
    """Newton physics simulation extension for Isaac Sim."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        global _newton_stage, _newton_physics_interface

        # Load config based on settings
        cfg = NewtonConfig()
        _newton_stage = NewtonStage(cfg=cfg)
        _newton_physics_interface = NewtonPhysicsInterface(_newton_stage)

        # Register Newton with unified physics interface
        self._newton_registry = NewtonSimulationRegistry()
        simulation_id = self._newton_registry.register_newton(_newton_stage)

        if simulation_id != k_invalid_simulation_id:
            carb.log_info(
                f"[isaacsim.physics.newton] Newton registered with unified physics interface (solver: {cfg.solver_cfg.solver_type})"
            )

            # Check if auto-switching is enabled (default: False)
            settings = carb.settings.get_settings()
            auto_switch = settings.get("/exts/isaacsim.physics.newton/auto_switch_on_startup")

            # Explicitly check for True (not just truthy)
            if auto_switch is True:
                success = SimulationManager.switch_physics_engine("newton")
                if success:
                    self._auto_switched = True
                    carb.log_warn("[isaacsim.physics.newton] Auto-switched to newton on startup via SimulationManager")
                else:
                    self._auto_switched = False
                    carb.log_error("[isaacsim.physics.newton] Failed to auto-switch to newton")
            else:
                self._auto_switched = False
                carb.log_warn(
                    "[isaacsim.physics.newton] newton registered but not auto-activated (auto_switch_on_startup=false)"
                )
                carb.log_warn(
                    "[isaacsim.physics.newton] Use isaacsim.physics.newton.switch_physics_engine('newton') to activate"
                )
        else:
            carb.log_error(
                f"[isaacsim.physics.newton] Failed to register Newton (solver: {cfg.solver_cfg.solver_type})"
            )

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        global _newton_stage

        # Switch back to physx if we auto-switched to newton on startup
        success = SimulationManager.switch_physics_engine("physx")
        if success:
            carb.log_warn("[isaacsim.physics.newton] Switched back to physx on shutdown")
        else:
            carb.log_warn("[isaacsim.physics.newton] Failed to switch back to physx on shutdown")

        # Unregister Newton from unified physics interface
        if hasattr(self, "_newton_registry"):
            self._newton_registry.unregister_newton()

        if _newton_stage:
            _newton_stage.init()
