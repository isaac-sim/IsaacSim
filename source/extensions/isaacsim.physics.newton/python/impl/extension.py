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

from typing import Any

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
_pending_newton_config: NewtonConfig | None = None


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


def configure_newton(cfg: NewtonConfig) -> None:
    """Set the Newton runtime configuration.

    Applied on the next play or re-initialization. Call before play from a
    standalone script or extension.

    Args:
        cfg: Newton simulation configuration.
    """
    global _pending_newton_config

    _pending_newton_config = cfg
    if _newton_stage is None:
        return
    _newton_stage.cfg = cfg
    _newton_stage.initialized = False
    carb.settings.get_settings().set("/exts/isaacsim.physics.newton/capture_graph_physics_step", cfg.use_cuda_graph)


def get_newton_config() -> NewtonConfig | None:
    """Return the live Newton configuration.

    Returns:
        The current or pending configuration, or None if Newton has not been configured.
    """
    return _newton_stage.cfg if _newton_stage is not None else _pending_newton_config


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


#: Whether Newton loads material textures at all. Off by default: nothing in
#: Isaac Sim reads them, and loading them dominates stage init.
_LOAD_TEXTURES_SETTING = "/exts/isaacsim.physics.newton/load_textures"


def _install_texture_policy() -> None:
    """Monkey-patch Newton's texture loader to respect ``load_textures``.

    Textures feed Newton's viewer only; Isaac Sim uses Hydra. Skipping them
    avoids a significant stage-init cost and suppresses spurious PIL warnings on
    ``omniverse://`` paths. Remote URLs are resolved through the local cache.
    """
    try:
        import omni.client
        from newton._src.utils import texture as newton_texture
    except Exception:
        return

    if getattr(newton_texture, "_isaac_texture_policy_installed", False):
        return

    original_loader = newton_texture.load_texture_from_file
    resolved: dict[str, str] = {}
    settings = carb.settings.get_settings()

    def load_texture_from_file(texture_path: str) -> Any:
        if not settings.get(_LOAD_TEXTURES_SETTING):
            return None
        if isinstance(texture_path, str) and texture_path:
            try:
                is_remote = not omni.client.is_local_url(texture_path)
            except Exception:
                is_remote = False
            if is_remote:
                local = resolved.get(texture_path)
                if local is None:
                    try:
                        result, candidate = omni.client.get_local_file(texture_path)
                        local = candidate if result == omni.client.Result.OK and candidate else ""
                    except Exception:
                        local = ""
                    resolved[texture_path] = local
                if not local:
                    # Unreachable asset: skip rather than let PIL fail on a URL.
                    return None
                texture_path = local
        return original_loader(texture_path)

    newton_texture.load_texture_from_file = load_texture_from_file
    newton_texture._isaac_texture_policy_installed = True
    carb.log_info("[isaacsim.physics.newton] texture policy installed")


class NewtonSimExtension(omni.ext.IExt):
    """Newton physics simulation extension for Isaac Sim."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        global _newton_stage, _newton_physics_interface, _pending_newton_config

        _install_texture_policy()

        cfg = _pending_newton_config or NewtonConfig()
        _pending_newton_config = cfg
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
