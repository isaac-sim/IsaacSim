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

"""Provide centralized management of physics simulation operations, events, callbacks, and time-related functionality."""

from __future__ import annotations

import weakref
from collections.abc import Callable
from typing import Any, Literal

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.physics.core
import omni.physics.tensors
import omni.timeline
import omni.usd
import warp as wp
from omni.physics.core import SimulationRegistryEventType
from pxr import PhysxSchema

from .isaac_events import IsaacEvents
from .physics_scene import PhysicsScene
from .physx_scene import PhysxScene
from .simulation_event import SimulationEvent

_SETTING_PLAY_SIMULATION = "/app/player/playSimulations"
_SETTING_PHYSICS_CUDA_DEVICE = "/physics/cudaDevice"
_SETTING_PHYSICS_SUPPRESS_READBACK = "/physics/suppressReadback"


class SimulationManager:
    """Manage physics simulation operations, events, callbacks, and time-related functionality.

    This class provides centralized control over physics simulation lifecycle, including warm starting
    simulation for physics data retrieval, managing callback functions triggered by simulation events
    such as physics steps, timeline changes, and stage operations.
    """

    _warmup_needed = True
    """Flag indicating whether physics warmup is needed before simulation can run."""
    _timeline = omni.timeline.get_timeline_interface()
    """Timeline interface for managing simulation playback and timeline events."""
    _message_bus = carb.eventdispatcher.get_eventdispatcher()
    """Event dispatcher for handling simulation and application events."""
    _physics_sim_interface = omni.physics.core.get_physics_simulation_interface()
    """Physics simulation interface for controlling physics simulation."""
    _physics_stage_update_interface = omni.physics.core.get_physics_stage_update_interface()
    """Physics stage update interface for managing physics stage updates."""
    _physics_interface = omni.physics.core.get_physics_interface()
    """Core physics interface for physics engine management."""
    _physx_fabric_interface = None
    """PhysX fabric interface for fabric-enabled physics operations."""
    _physics_sim_view = None
    """Deprecated physics simulation view for tensor operations."""
    _physics_sim_view__warp = None
    """Warp-based physics simulation view for tensor operations."""
    _carb_settings = carb.settings.get_settings()
    """Carb settings interface for accessing application settings."""
    _callbacks = {}
    """Dictionary storing registered callback subscriptions by unique ID."""
    _simulation_manager_interface = None
    """Internal simulation manager interface for core operations."""
    _simulation_view_created = False
    """Flag indicating whether the simulation view has been created."""
    _assets_loaded = True
    """Flag indicating whether stage assets have finished loading."""
    _assets_loading_callback = None
    """Callback subscription for asset loading events."""
    _assets_loaded_callback = None
    """Callback subscription for asset loaded events."""

    _physics_scenes: dict[str, PhysicsScene] = {}
    """Dictionary mapping physics scene paths to PhysicsScene objects."""

    # physics engine
    _engine = "physx"
    """Name of the currently active physics engine."""
    _simulation_registry_sub = None
    """Subscription to simulation registry events for engine synchronization."""
    _device: wp.Device | None = None  # Explicitly requested device (for Newton)
    """Explicitly requested Warp device for physics simulation."""

    # default callbacks
    _default_callback_on_stop = None
    """Default callback subscription for timeline stop events."""
    _default_callback_stage_open = None
    """Default callback subscription for stage opened events."""
    _default_callback_stage_close = None
    """Default callback subscription for stage closed events."""
    _default_callback_warm_start = None
    """Default callback subscription for timeline play events."""
    _default_callbacks_state = {
        "on_stop": True,
        "stage_open": True,
        "stage_close": True,
        "warm_start": True,
        "post_warm_start": True,  # deprecated
    }
    """Dictionary tracking the enabled state of default callbacks."""

    # deprecated variables
    _backend = "numpy"
    """Deprecated backend type used by the simulation manager."""
    _default_physics_scene_path = None

    """
    Internal methods.
    """

    @classmethod
    def get_active_physics_engine(cls) -> Literal["physx", "newton", "remotesim"]:
        """Get the currently active physics engine.

        Returns:
            Currently active engine name.
        """
        return cls._engine

    @classmethod
    def get_default_engine(cls) -> str:
        """Get the default physics engine from settings.

        Returns:
            Default engine name from settings, or empty string if not set.
        """
        return cls._carb_settings.get_as_string("/exts/isaacsim.core.simulation_manager/default_engine") or ""

    @classmethod
    def get_available_physics_engines(cls, verbose: bool = False) -> list[tuple[str, bool]]:
        """Get list of all available physics engines.

        Args:
            verbose: If True, print available engines.

        Returns:
            List of tuples (engine_name, is_active) for all registered engines.
        """
        if not cls._physics_interface:
            return []

        engines = []
        simulation_ids = cls._physics_interface.get_simulation_ids()
        for sim_id in simulation_ids:
            sim_name = cls._physics_interface.get_simulation_name(sim_id)
            is_active = cls._physics_interface.is_simulation_active(sim_id)
            engines.append((sim_name.lower(), is_active))

        if verbose:
            print("Available physics engines:")
            for engine in engines:
                print(f"  {engine[0]}: {'active' if engine[1] else 'inactive'}")
            print("-" * 60)
        return engines

    @classmethod
    def switch_physics_engine(cls, engine_name: Literal["physx", "newton", "remotesim"], verbose: bool = False) -> bool:
        """Switch to a specific physics engine.

        Args:
            engine_name: Name of the engine to switch to.
            verbose: If True, log switch details to console.

        Returns:
            True if switch was successful, False otherwise.
        """
        if not cls._physics_interface:
            carb.log_error(f"Cannot switch to {engine_name}: physics interface not available")
            return False

        old_engine = cls._engine
        simulation_ids = cls._physics_interface.get_simulation_ids()
        target_id = None

        # Find target engine (case-insensitive lookup)
        for sim_id in simulation_ids:
            sim_name = cls._physics_interface.get_simulation_name(sim_id)
            if sim_name.lower() == engine_name.lower():
                target_id = sim_id
                break

        if target_id is None:
            available = [cls._physics_interface.get_simulation_name(sid) for sid in simulation_ids]
            carb.log_error(f"Engine '{engine_name}' not found. Available: {', '.join(available)}")
            return False

        # Deactivate all other engines for mutual exclusivity
        deactivated_any = False
        for sim_id in simulation_ids:
            if sim_id != target_id:
                if cls._physics_interface.is_simulation_active(sim_id):
                    simulation = cls._physics_interface.get_simulation(sim_id)
                    if simulation and simulation.stage_update_fns and simulation.stage_update_fns.on_detach:
                        try:
                            simulation.stage_update_fns.on_detach()
                        except Exception as e:
                            carb.log_warn(
                                f"on_detach failed for {cls._physics_interface.get_simulation_name(sim_id)}: {e}"
                            )

                    cls._physics_interface.deactivate_simulation(sim_id)
                    deactivated_engine = cls._physics_interface.get_simulation_name(sim_id)
                    if verbose:
                        carb.log_info(f"Deactivated {deactivated_engine} engine")
                    deactivated_any = True

        # Activate target engine
        was_already_active = cls._physics_interface.is_simulation_active(target_id)
        if not was_already_active:
            cls._physics_interface.activate_simulation(target_id)

            simulation = cls._physics_interface.get_simulation(target_id)
            if simulation and simulation.stage_update_fns and simulation.stage_update_fns.on_attach:
                try:
                    stage_id = omni.usd.get_context().get_stage_id()
                    simulation.stage_update_fns.on_attach(stage_id)
                except Exception as e:
                    carb.log_warn(f"on_attach failed for {engine_name}: {e}")

        # Update internal state after engine switch is complete
        cls._on_engine_switched(engine_name.lower())

        if verbose:
            if was_already_active and not deactivated_any:
                carb.log_warn(f"Physics engine is already set to {engine_name}")
            else:
                carb.log_warn(f"Simulation engine switched from {old_engine} to {engine_name}")

        return True

    @classmethod
    def _on_engine_switched(cls, engine_name: str) -> None:
        """Handle internal state updates when the physics engine changes.

        Args:
            engine_name: The name of the newly active engine (lowercase).
        """
        cls._engine = engine_name
        if cls._physics_sim_view:
            cls._physics_sim_view.invalidate()
            cls._physics_sim_view = None
        if cls._physics_sim_view__warp:
            cls._physics_sim_view__warp.invalidate()
            cls._physics_sim_view__warp = None
        cls._simulation_view_created = False

        cls._warmup_needed = True

        for path in list(cls._physics_scenes):
            prim = cls._physics_scenes[path].prim
            if engine_name == "physx":
                prim.RemoveAPI("MjcSceneAPI")
                prim.RemoveAPI("NewtonXpbdSceneAPI")
            else:
                if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                    prim.RemoveAPI(PhysxSchema.PhysxSceneAPI)
            cls._physics_scenes[path] = cls._create_physics_scene(path)

    @classmethod
    def _sync_engine_state(cls) -> None:
        """Sync the _engine variable with the actual active physics engine.

        This should be called at initialization to ensure the cached engine name
        matches the actual active engine in the unified physics interface.
        """
        if not cls._physics_interface:
            return

        simulation_ids = cls._physics_interface.get_simulation_ids()

        # First check if an engine is already active
        for sim_id in simulation_ids:
            if cls._physics_interface.is_simulation_active(sim_id):
                cls._engine = cls._physics_interface.get_simulation_name(sim_id).lower()
                carb.log_info(f"Initialized physics engine: {cls._engine}")
                return

        # Check if a specific engine is requested via settings
        default_engine = cls.get_default_engine()
        if default_engine:
            available_engines = {cls._physics_interface.get_simulation_name(sid).lower() for sid in simulation_ids}
            default_engine_lower = default_engine.lower()
            if default_engine_lower in available_engines:
                cls._engine = default_engine_lower
                carb.log_info(f"Using physics engine from settings: {cls._engine}")
                return
            else:
                carb.log_warn(
                    f"Requested engine '{default_engine}' not available. "
                    f"Available: {', '.join(available_engines)}. Using first available engine."
                )

        # Use first available engine
        if simulation_ids:
            cls._engine = cls._physics_interface.get_simulation_name(simulation_ids[0]).lower()
            carb.log_info(f"Using physics engine: {cls._engine}")
        else:
            cls._engine = "physx"
            carb.log_warn("No physics engines found, defaulting to PhysX")

    @staticmethod
    def _on_simulation_registry_event(event_type: SimulationRegistryEventType, simulation_id: int, name: str) -> None:
        """Handle simulation registry events to keep _engine in sync.

        Args:
            event_type: Type of simulation registry event.
            simulation_id: ID of the simulation.
            name: Name of the simulation.
        """
        engine_name = name.lower()
        if event_type == SimulationRegistryEventType.SIMULATION_ACTIVATED:
            if SimulationManager._engine != engine_name:
                SimulationManager._on_engine_switched(engine_name)
        elif event_type == SimulationRegistryEventType.SIMULATION_DEACTIVATED:
            if engine_name == SimulationManager._engine:
                for sim_id in SimulationManager._physics_interface.get_simulation_ids():
                    if SimulationManager._physics_interface.is_simulation_active(sim_id):
                        new_engine = SimulationManager._physics_interface.get_simulation_name(sim_id).lower()
                        SimulationManager._on_engine_switched(new_engine)
                        return

    @classmethod
    def _startup(cls) -> None:
        """Initialize the simulation manager by enabling default callbacks and resetting state.

        This method synchronizes the engine state with the active physics engine and subscribes
        to simulation registry events to maintain consistency with UI and other systems.
        """
        cls.enable_all_default_callbacks(True)
        cls._reset(
            reset_assets=True,
            reset_callbacks=True,
            reset_physics=True,
            reset_physics_scenes=True,
            track_physics_scenes=True,
        )

        # Sync engine variable with actual active engine
        cls._sync_engine_state()

        # Subscribe to simulation registry events to stay in sync with UI and other systems
        cls._simulation_registry_sub = cls._physics_interface.subscribe_simulation_registry_events(
            cls._on_simulation_registry_event
        )

    @classmethod
    def _shutdown(cls) -> None:
        """Shutdown the simulation manager by disabling default callbacks and cleaning up state.

        This method unsubscribes from simulation registry events and resets all internal state
        to prepare for shutdown.
        """
        # Unsubscribe from simulation registry events
        cls._simulation_registry_sub = None

        cls.enable_all_default_callbacks(False)
        cls._reset(
            reset_assets=True,
            reset_callbacks=True,
            reset_physics=True,
            reset_physics_scenes=True,
            track_physics_scenes=False,
        )

    @classmethod
    def _reset(
        cls,
        *,
        reset_assets: bool = False,
        reset_callbacks: bool = False,
        reset_physics: bool = False,
        reset_physics_scenes: bool = False,
        track_physics_scenes: bool = False,
    ) -> None:
        """Reset various components of the simulation manager state.

        Args:
            reset_assets: Whether to reset asset-related state including loading callbacks.
            reset_callbacks: Whether to clear all registered callbacks.
            reset_physics: Whether to reset physics-related state and invalidate physics.
            reset_physics_scenes: Whether to clear tracked physics scenes.
            track_physics_scenes: Whether to start tracking physics scene additions and deletions.
        """
        cls._simulation_manager_interface.reset()
        # asset-related state
        if reset_assets:
            cls._assets_loaded = True
            cls._assets_loading_callback = None
            cls._assets_loaded_callback = None
        # callbacks
        if reset_callbacks:
            cls._callbacks.clear()
        # physics scenes
        if reset_physics_scenes:
            cls._physics_scenes.clear()
            cls._default_physics_scene_path = None
            if track_physics_scenes:
                cls._track_physics_scenes()
        # physics
        if reset_physics:
            cls._device = None
            cls.invalidate_physics()

    @classmethod
    def _setup_warm_start_callback(cls) -> None:
        """Set up the warm start callback to initialize physics when simulation starts.

        Creates a timeline subscription to listen for PLAY events that trigger physics initialization.
        """
        if cls._default_callbacks_state["warm_start"] and cls._default_callback_warm_start is None:
            cls._default_callback_warm_start = (
                cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                    int(omni.timeline.TimelineEventType.PLAY), cls._on_play
                )
            )

    @classmethod
    def _setup_on_stop_callback(cls) -> None:
        """Set up the callback to handle simulation stop events.

        Creates a timeline subscription to listen for STOP events and clean up physics resources.
        """
        if cls._default_callbacks_state["on_stop"] and cls._default_callback_on_stop is None:
            cls._default_callback_on_stop = (
                cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                    int(omni.timeline.TimelineEventType.STOP), cls._on_stop
                )
            )

    @classmethod
    def _setup_stage_open_callback(cls) -> None:
        """Set up the callback to handle stage open events.

        Creates a message bus subscription to listen for OPENED stage events and reset simulation state.
        """
        if cls._default_callbacks_state["stage_open"] and cls._default_callback_stage_open is None:
            cls._default_callback_stage_open = cls._message_bus.observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=cls._on_stage_opened,
                observer_name="SimulationManager._default_callback_stage_open",
            )

    @classmethod
    def _setup_stage_close_callback(cls) -> None:
        """Set up the callback to handle stage close events.

        Creates a message bus subscription to listen for CLOSED stage events and clean up simulation resources.
        """
        if cls._default_callbacks_state["stage_close"] and cls._default_callback_stage_close is None:
            cls._default_callback_stage_close = cls._message_bus.observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=cls._on_stage_closed,
                observer_name="SimulationManager._default_callback_stage_close",
            )

    @classmethod
    def _track_physics_scenes(cls) -> None:
        """Set up tracking of physics scenes in the stage.

        Registers callbacks to automatically add and remove physics scenes when they are created or deleted.
        """

        def add_physics_scene(path: str) -> None:
            prim = prim_utils.get_prim_at_path(path)
            if prim.GetTypeName() == "PhysicsScene":
                cls._physics_scenes[path] = cls._create_physics_scene(path)

        def remove_physics_scene(path: str) -> None:
            # TODO: search for child prims that are also physics scenes???
            if path in cls._physics_scenes:
                del cls._physics_scenes[path]

        cls._simulation_manager_interface.register_physics_scene_addition_callback(add_physics_scene)
        cls._simulation_manager_interface.register_deletion_callback(remove_physics_scene)

    @classmethod
    def _create_physics_scene(cls, path: str = "/PhysicsScene") -> PhysicsScene | None:
        """Create a physics scene at the specified path.

        Args:
            path: Path where the physics scene will be created.

        Returns:
            The created physics scene, or None if creation failed.
        """
        try:
            if cls._engine == "physx":
                return PhysxScene(path)
            elif cls._engine == "newton":
                # Lazy import to avoid loading heavy Newton dependencies at module load time
                from .mjc_scene import NewtonMjcScene

                return NewtonMjcScene(path)
            elif cls._engine == "remotesim":
                # Remote-sim backend only needs a lightweight scene wrapper for dt/config access.
                return PhysicsScene(path)
            else:
                carb.log_warn(f"Unknown engine '{cls._engine}', defaulting to PhysX")
                return PhysxScene(path)
        except RuntimeError as e:
            carb.log_warn(f"Failed to create physics scene at '{path}': {e}")
            return None

    @classmethod
    def _cleanup_stale_physics_scenes(cls) -> bool:
        """Remove stale physics scene references with invalid/expired prims.

        This handles cases where physics scene prims become invalid without triggering
        the deletion callback (e.g., layer clear/reload operations, certain USD changes).
        Also removes PhysxScene objects whose prims no longer have PhysxSceneAPI applied.

        Returns:
            True if any stale entries were removed, False otherwise.
        """
        # First, cleanup in C++ layer (also triggers deletion callbacks)
        cpp_removed = cls._simulation_manager_interface.cleanup_invalid_physics_scenes()

        # Then cleanup Python-side cache
        stale_paths = []
        for path, scene in cls._physics_scenes.items():
            try:
                if not scene.prim or not scene.prim.IsValid():
                    stale_paths.append(path)
                elif isinstance(scene, PhysxScene) and not scene.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                    stale_paths.append(path)
            except Exception:
                stale_paths.append(path)

        for path in stale_paths:
            carb.log_warn(f"Removing stale physics scene reference at '{path}' (prim is no longer valid)")
            del cls._physics_scenes[path]

        return len(stale_paths) > 0 or len(cpp_removed) > 0

    """
    Internal callbacks.
    """

    @staticmethod
    def _on_stage_opened(event: Any) -> None:
        """Handle stage opened events to reset simulation state and track asset loading.

        Args:
            event: Stage opened event payload.
        """

        def _assets_loading(event: Any) -> None:
            SimulationManager._assets_loaded = False

        def _assets_loaded(event: object) -> None:
            """Flag indicating whether stage assets have finished loading.

            Args:
                event: The stage event indicating assets have loaded.
            """
            SimulationManager._assets_loaded = True

        SimulationManager._reset(
            reset_assets=True,
            reset_callbacks=True,
            reset_physics=True,
            reset_physics_scenes=True,
            track_physics_scenes=True,
        )
        SimulationManager._assets_loading_callback = SimulationManager._message_bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.ASSETS_LOADING),
            on_event=_assets_loading,
            observer_name="SimulationManager._assets_loading_callback",
        )
        SimulationManager._assets_loaded_callback = SimulationManager._message_bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
            on_event=_assets_loaded,
            observer_name="SimulationManager._assets_loaded_callback",
        )

    @staticmethod
    def _on_stage_closed(event: Any) -> None:
        """Handle stage closed events to reset all simulation resources.

        Args:
            event: Stage closed event payload.
        """
        SimulationManager._reset(
            reset_assets=True,
            reset_callbacks=True,
            reset_physics=True,
            reset_physics_scenes=True,
            track_physics_scenes=True,
        )

    @staticmethod
    def _on_play(event: Any) -> None:
        """Handle timeline play events to initialize physics simulation.

        Args:
            event: Timeline play event payload.
        """
        if SimulationManager._carb_settings.get_as_bool(_SETTING_PLAY_SIMULATION):
            # Verify the stage is valid before attempting any physics operations
            # This handles cases where play is triggered on an expired/invalid stage
            try:
                stage = omni.usd.get_context().get_stage()
                if stage is None:
                    carb.log_warn("Cannot initialize physics: no stage available")
                    return
                root_layer = stage.GetRootLayer()
                if root_layer is None or root_layer.expired:
                    carb.log_warn("Cannot initialize physics: stage root layer is expired or invalid")
                    return
            except Exception as e:
                carb.log_warn(f"Cannot initialize physics: failed to verify stage validity: {e}")
                return

            # Cleanup any stale physics scene references before simulation starts
            # This handles cases where prims become invalid without triggering deletion callbacks
            # (e.g., layer clear/reload operations, certain USD reference changes)
            SimulationManager._cleanup_stale_physics_scenes()

            if SimulationManager._warmup_needed:
                SimulationManager.initialize_physics()
                SimulationManager._message_bus.dispatch_event(SimulationEvent.SIMULATION_STARTED.value, payload={})
            else:
                SimulationManager._message_bus.dispatch_event(SimulationEvent.SIMULATION_RESUMED.value, payload={})

    @staticmethod
    def _on_stop(event: Any) -> None:
        """Handle timeline stop events to invalidate physics.

        Args:
            event: Timeline stop event payload.
        """
        SimulationManager.invalidate_physics()

    """
    Public methods.
    """

    @classmethod
    def initialize_physics(cls) -> None:
        """Initialize Physics.

        .. important::

            This method is called automatically when the simulation runs (i.e., when the timeline is played).
            Therefore, it is not intended to be called directly unless a manual initialization is desired/required.

        This method initializes the physics engine by loading physics from USD and starting the engine simulation.
        It also creates the physics simulation view required by the Isaac Sim Core extensions.
        After initializing physics, the :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.SIMULATION_SETUP`
        event is dispatched.
        """
        if not cls._warmup_needed:
            return
        # create physics scene (if not exists)
        if not cls._physics_scenes:
            physics_scene = cls._create_physics_scene()
            if physics_scene is None:
                carb.log_warn("Cannot initialize physics: failed to create physics scene")
                return
        # initialize physics engine
        cls._physics_stage_update_interface.force_load_physics_from_usd()
        # Engine-specific pre-initialization, initialize Newton stage before calling simulate
        if cls._engine == "newton":
            try:
                import isaacsim.physics.newton

                newton_stage = isaacsim.physics.newton.acquire_stage()
                if newton_stage is None:
                    raise Exception("newton stage not available - isaacsim.physics.newton extension may not be loaded")

                # Update newton device to match requested device
                requested_device = cls.get_physics_sim_device()
                requested_device_str = requested_device if isinstance(requested_device, str) else str(requested_device)

                if not newton_stage.initialized or newton_stage.device_str != requested_device_str:
                    if newton_stage.device_str != requested_device_str:
                        carb.log_warn(
                            f"newton device mismatch: initialized on {newton_stage.device_str}, requested {requested_device_str}. Reinitializing..."
                        )
                    newton_stage.initialize_newton(requested_device_str)

                newton_stage.device_str = requested_device_str
                newton_stage.device = wp.get_device(newton_stage.device_str)
            except Exception as e:
                carb.log_error(f"Failed to initialize Newton: {e}")

        cls._physics_stage_update_interface.start_simulation()

        cls._physics_sim_interface.simulate(cls.get_physics_dt(), 0.0)
        # create simulation view
        stage_id = stage_utils.get_stage_id(stage_utils.get_current_stage(backend="usd"))
        # - deprecated simulation view
        create_simulation_view = True
        if "cuda" in cls.get_physics_sim_device() and cls._backend == "numpy":
            cls._backend = "torch"
            carb.log_warn("Changing backend from 'numpy' to 'torch' since NumPy cannot be used with GPU piplines")
        if cls.get_backend() == "torch":
            try:
                import importlib.util

                if importlib.util.find_spec("torch") is None:
                    raise ModuleNotFoundError("torch not found")
            except ModuleNotFoundError:
                create_simulation_view = False

        # Create simulation views (unified path for all backends)
        cls._physics_sim_view__warp = omni.physics.tensors.create_simulation_view(
            "warp", stage_id=stage_id, backend=cls._engine
        )
        cls._physics_sim_view__warp.set_subspace_roots("/")
        if create_simulation_view:
            cls._physics_sim_view = omni.physics.tensors.create_simulation_view(
                cls.get_backend(), stage_id=stage_id, backend=cls._engine
            )
            cls._physics_sim_view.set_subspace_roots("/")
        carb.log_info(f"Created simulation views (engine: {cls._engine}, backend: {cls.get_backend()})")

        cls._physics_sim_interface.simulate(cls.get_physics_dt(), 0.0)
        # set internal states
        cls._warmup_needed = False
        cls._simulation_view_created = True
        # dispatch events
        cls._message_bus.dispatch_event(SimulationEvent.SIMULATION_SETUP.value, payload={})
        # - deprecated events
        cls._message_bus.dispatch_event(IsaacEvents.PHYSICS_WARMUP.value, payload={})
        cls._message_bus.dispatch_event(IsaacEvents.SIMULATION_VIEW_CREATED.value, payload={})
        cls._message_bus.dispatch_event(IsaacEvents.PHYSICS_READY.value, payload={})

    @classmethod
    def invalidate_physics(cls) -> None:
        """Invalidate Physics.

        .. important::

            This method is called automatically when the simulation stops (i.e., when the timeline is stopped).
            Therefore, it is not intended to be called directly unless a manual invalidation is desired/required.

        This method invalidates the physics simulation view and resets the internal state in preparation
        for a new simulation cycle.
        """
        cls._warmup_needed = True
        if cls._physics_sim_view__warp:
            cls._physics_sim_view__warp.invalidate()
            cls._physics_sim_view__warp = None
        # deprecated simulation view
        if cls._physics_sim_view:
            cls._physics_sim_view.invalidate()
            cls._physics_sim_view = None
            cls._simulation_view_created = False

    @classmethod
    def setup_simulation(cls, dt: float | None = None, device: str | wp.Device | None = None) -> None:
        """Set up the (physics) simulation.

        .. hint::

            This method is a convenient implementation that allows for quick configuration of the simulation in one go.
            Same behavior can be achieved by calling other individual methods in sequence.

        Args:
            dt: Physics delta time (DT).
            device: Physics simulation device.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cuda:0")
        """
        # create physics scene (if not exists)
        if not cls._physics_scenes:
            cls._create_physics_scene()
        # apply given parameters
        if dt is not None:
            for physics_scene in cls._physics_scenes.values():
                physics_scene.set_dt(dt)
        if device is not None:
            cls.set_device(device)

    @classmethod
    def get_physics_scenes(cls) -> list[PhysicsScene]:
        """Get the available physics scenes.

        Returns:
            List of physics scenes.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager, PhysicsScene
            >>>
            >>> SimulationManager.get_physics_scenes()
            [<isaacsim.core.simulation_manager.impl.physx_scene.PhysxScene object at 0x...>]
        """
        return list(cls._physics_scenes.values())

    @classmethod
    def get_physics_simulation_view(cls) -> Any | None:
        """Get the physics (tensor API) simulation view.

        Returns:
            Physics (tensor API) simulation view, or ``None`` if the physics is not initialized.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.get_physics_simulation_view()
            <omni.physics.tensors.api.SimulationView object at 0x...>
        """
        return cls._physics_sim_view__warp

    @classmethod
    def get_simulation_time(cls) -> float:
        """Get the current simulation time.

        Returns:
            The current simulation time.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.get_simulation_time()
            0.0333...
        """
        return cls._simulation_manager_interface.get_simulation_time()

    @classmethod
    def get_num_physics_steps(cls) -> int:
        """Get the current number of physics steps performed.

        Returns:
            The current number of physics steps.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.get_num_physics_steps()
            2
        """
        return cls._simulation_manager_interface.get_num_physics_steps()

    @classmethod
    def is_simulating(cls) -> bool:
        """Check if the simulation is currently running.

        Returns:
            True if the simulation is running, False otherwise.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.is_simulating()
            True
        """
        return cls._simulation_manager_interface.is_simulating()

    @classmethod
    def is_paused(cls) -> bool:
        """Check if the simulation is currently paused.

        Returns:
            True if the simulation is paused, False otherwise.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.is_paused()
            False
        """
        return cls._simulation_manager_interface.is_paused()

    @classmethod
    def step(
        cls, *, steps: int = 1, callback: Callable[[int, int], bool | None] | None = None, update_fabric: bool = False
    ) -> None:
        """Step the physics simulation.

        Args:
            steps: Number of steps to perform.
            callback: Optional callback function to call after each step.
                The function should take two arguments: the current step number and the total number of steps.
                If no return value is provided, the internal loop will run for the specified number of steps.
                However, if the function returns ``False``, no more steps will be performed.
            update_fabric: Whether to update fabric with the latest physics results after each step.

        Raises:
            ValueError: If the fabric is not enabled and ``update_fabric`` is set to True.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> # perform one physics step
            >>> SimulationManager.step(steps=1)
            >>>
            >>> # perform 10 physics steps
            >>> SimulationManager.step(steps=10)
            >>>
            >>> # perform 10 physics steps with a callback
            >>> def callback(step, steps):
            ...     print(f"physics step {step}/{steps}")
            ...     return step < 3  # stop after 3 steps (return False to break the loop)
            ...
            >>> SimulationManager.step(steps=10, callback=callback)
            physics step 1/10
            physics step 2/10
            physics step 3/10
        """
        dt = cls.get_physics_dt()
        for step in range(steps):
            simulation_time = cls.get_simulation_time()
            # step physics simulation
            cls._physics_sim_interface.simulate(dt, simulation_time)
            # update fabric (PhysX only - Newton handles fabric updates differently)
            if update_fabric and cls._engine == "physx":
                if not cls.is_fabric_enabled():
                    raise ValueError("PhysX support for fabric is not enabled. Call '.enable_fabric()' first")
                if cls._physx_fabric_interface is None:
                    cls._physx_fabric_interface = omni.physxfabric.get_physx_fabric_interface()
                cls._physx_fabric_interface.update(simulation_time, dt)
            # call callback
            if callback is not None:
                if callback(step + 1, steps) is False:
                    break

    @classmethod
    def set_device(cls, device: str | wp.Device) -> None:
        """Set the simulation device.

        Args:
            device: Simulation device.

        Raises:
            ValueError: If the device is invalid.
            Exception: If the device is unknown.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.set_device("cuda:0")
        """
        device = ops_utils.parse_device(device, raise_on_invalid=True)
        # Store the requested device as wp.Device (used by Newton)
        cls._device = device
        # GPU device
        if device.is_cuda:
            cls._carb_settings.set_int(_SETTING_PHYSICS_CUDA_DEVICE, device.ordinal)
            cls._carb_settings.set_bool(_SETTING_PHYSICS_SUPPRESS_READBACK, True)
            cls.enable_fabric(enable=True)
            for physics_scene in cls._physics_scenes.values():
                if isinstance(physics_scene, PhysxScene):
                    physics_scene.set_broadphase_type("GPU")
                    physics_scene.set_enabled_gpu_dynamics(True)
        # CPU device
        elif device.is_cpu:
            # cls._carb_settings.set_int(_SETTING_PHYSICS_CUDA_DEVICE, -1)
            cls._carb_settings.set_bool(_SETTING_PHYSICS_SUPPRESS_READBACK, False)
            for physics_scene in cls._physics_scenes.values():
                if isinstance(physics_scene, PhysxScene):
                    physics_scene.set_broadphase_type("MBP")
                    physics_scene.set_enabled_gpu_dynamics(False)
        # unknown device
        else:
            raise Exception(f"Unknown device: {device}")

    @classmethod
    def get_device(cls) -> wp.Device:
        """Get the simulation device.

        Returns:
            Simulation device.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> device = SimulationManager.get_device()
            >>> print(type(device), device)
            <class 'warp._src.context.Device'> cpu
        """
        supress_readback = cls._carb_settings.get_as_bool(_SETTING_PHYSICS_SUPPRESS_READBACK)

        # For Newton, check explicitly set device first, then fall back to warp default (CUDA)
        if cls._engine == "newton":
            # If device was explicitly set via set_device(), use it
            if cls._device is not None:
                return cls._device

            # Fall back to warp's default device (usually CUDA)
            return wp.get_device()

        if supress_readback:
            is_gpu_scene = False
            if cls._physics_scenes:
                first_scene = next(iter(cls._physics_scenes.values()))
                if isinstance(first_scene, PhysxScene):
                    is_gpu_scene = first_scene.get_broadphase_type() == "GPU" and first_scene.get_enabled_gpu_dynamics()
            if not cls._physics_scenes or is_gpu_scene:
                ordinal = cls._carb_settings.get_as_int(_SETTING_PHYSICS_CUDA_DEVICE)
                if ordinal < 0:
                    cls._carb_settings.set_int(_SETTING_PHYSICS_CUDA_DEVICE, 0)
                    carb.log_warn("No CUDA device configured under '/physics/cudaDevice'. Using 'cuda:0'")
                return ops_utils.parse_device(f"cuda:{ordinal}", raise_on_invalid=True)
        return ops_utils.parse_device("cpu", raise_on_invalid=True)

    @classmethod
    def enable_fabric(cls, enable: bool) -> None:
        """Enable or disable physics fabric integration and associated settings.

        .. note::

            This only applies to PhysX. For other physics engines (like Newton), this is a no-op
            since they handle fabric/USD updates differently.

        Args:
            enable: Whether to enable or disable fabric.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.enable_fabric(True)
        """
        # Only enable/disable PhysX fabric for PhysX engine
        # Newton and other engines handle fabric differently
        if cls._engine != "physx":
            return
        # enable/disable the omni.physx.fabric extension
        app_utils.enable_extension("omni.physx.fabric", enabled=enable)
        cls._physx_fabric_interface = omni.physxfabric.get_physx_fabric_interface() if enable else None
        # enable/disable USD updates
        cls._carb_settings.set_bool("/physics/updateToUsd", not enable)
        cls._carb_settings.set_bool("/physics/updateParticlesToUsd", not enable)
        cls._carb_settings.set_bool("/physics/updateVelocitiesToUsd", not enable)
        cls._carb_settings.set_bool("/physics/updateForceSensorsToUsd", not enable)

    @classmethod
    def is_fabric_enabled(cls) -> bool:
        """Check if fabric is enabled.

        Returns:
            True if fabric is enabled, otherwise False.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.is_fabric_enabled()
            True
        """
        return app_utils.is_extension_enabled("omni.physx.fabric")

    @classmethod
    def register_callback(
        cls, callback: Callable, event: SimulationEvent | IsaacEvents, *, order: int = 0, **kwargs: Any
    ) -> int:
        """Register/subscribe a callback to be triggered when a specific simulation event occurs.

        .. warning::

            The parameter ``name`` is not longer supported. A warning message will be logged if it is defined.
            Future versions will completely remove it. At that time, defining it will result in an exception.

        Args:
            callback: The callback function.
            event: The simulation event to subscribe to.
            order: The subscription order.
                Callbacks registered within the same order will be triggered in the order they were registered.
            **kwargs: Additional arguments. The ``name`` parameter is deprecated and will result in a warning.

        Returns:
            The unique identifier of the callback subscription.

        Raises:
            ValueError: If event is invalid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
            >>>
            >>> def callback(dt, context):
            ...     print(dt, context)
            ...
            >>> # subscribe to the PHYSICS_POST_STEP event
            >>> callback_id = SimulationManager.register_callback(callback, event=SimulationEvent.PHYSICS_POST_STEP)
            >>> callback_id
            2
            >>> # perform a physics step in order to trigger the callback and print the event
            >>> SimulationManager.step()
            0.01666... <omni.physics.core.bindings._physics.PhysicsStepContext object at 0x...>
            >>>
            >>> # deregister all callbacks
            >>> SimulationManager.deregister_all_callbacks()
        """
        if event not in SimulationEvent and event not in IsaacEvents:
            raise ValueError(f"Invalid simulation event: {event}. Supported events are: {list(SimulationEvent)}")
        # handle deprecations
        if "name" in kwargs:
            carb.log_warn("The parameter 'name' is not longer supported and will be removed in a future version")
        # check for weak reference support (when the callback is a method of a class)
        if hasattr(callback, "__self__"):

            def on_event(e: Any, obj: Any = weakref.proxy(callback.__self__)) -> Any:
                return getattr(obj, callback.__name__)(e)

        else:
            on_event = callback
        # get a unique id for the callback
        uid = cls._simulation_manager_interface.get_callback_iter()
        name = f"isaacsim.core.simulation_manager:callback.{event.value}.{uid}"
        # register the callback
        # - Physics-related events
        if event in [
            IsaacEvents.PHYSICS_WARMUP,
            IsaacEvents.PHYSICS_READY,
            IsaacEvents.POST_RESET,
            IsaacEvents.SIMULATION_VIEW_CREATED,
        ]:
            cls._callbacks[uid] = cls._message_bus.observe_event(
                observer_name=name,
                event_name=event.value,
                on_event=on_event,
                order=order,
            )
        elif event in [
            SimulationEvent.PHYSICS_PRE_STEP,
            SimulationEvent.PHYSICS_POST_STEP,
            IsaacEvents.PRE_PHYSICS_STEP,
            IsaacEvents.POST_PHYSICS_STEP,
        ]:
            if hasattr(callback, "__self__"):

                def on_event(step_dt: float, context: Any, obj: Any = weakref.proxy(callback.__self__)) -> Any:
                    return getattr(obj, callback.__name__)(step_dt, context) if cls._simulation_view_created else None

            else:

                def on_event(step_dt: float, context: Any) -> Any:
                    return callback(step_dt, context) if cls._simulation_view_created else None

            cls._callbacks[uid] = cls._physics_sim_interface.subscribe_physics_on_step_events(
                on_update=on_event,
                pre_step=event in [SimulationEvent.PHYSICS_PRE_STEP, IsaacEvents.PRE_PHYSICS_STEP],
                order=order,
            )
        # - Simulation lifecycle events
        elif event in [
            SimulationEvent.SIMULATION_SETUP,
            SimulationEvent.SIMULATION_STARTED,
            SimulationEvent.SIMULATION_RESUMED,
        ]:
            cls._callbacks[uid] = cls._message_bus.observe_event(
                observer_name=name,
                event_name=event.value,
                on_event=on_event,
                order=order,
            )
        elif event in [SimulationEvent.SIMULATION_PAUSED]:
            cls._callbacks[uid] = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                event_type=int(omni.timeline.TimelineEventType.PAUSE), fn=on_event, order=order, name=name
            )
        elif event in [SimulationEvent.SIMULATION_STOPPED, IsaacEvents.TIMELINE_STOP]:
            cls._callbacks[uid] = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                event_type=int(omni.timeline.TimelineEventType.STOP), fn=on_event, order=order, name=name
            )
        # - USD-related events
        elif event in [SimulationEvent.PRIM_DELETED, IsaacEvents.PRIM_DELETION]:
            cls._simulation_manager_interface.register_deletion_callback(on_event)
        else:
            raise RuntimeError(f"Unable to register callback for event '{event}' with uid '{uid}'")
        cls._simulation_manager_interface.set_callback_iter(uid + 1)
        return uid

    @classmethod
    def deregister_callback(cls, uid: int) -> bool:
        """Deregister a callback registered via :py:meth:`register_callback`.

        Args:
            uid: The unique identifier of the callback to deregister. If the unique identifier does not exist
                or has already been deregistered, a warning is logged and the method does nothing.

        Returns:
            True if the callback was successfully deregistered, False otherwise.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> # deregister the callback with the unique identifier 0
            >>> SimulationManager.deregister_callback(0)
            True
        """
        if uid in cls._callbacks:
            del cls._callbacks[uid]
            return True
        elif cls._simulation_manager_interface.deregister_callback(uid):
            return True
        carb.log_warn(f"Unable to deregister callback with uid '{uid}'. It might have been already deregistered")
        return False

    @classmethod
    def deregister_all_callbacks(cls) -> None:
        """Deregister all callbacks registered via :py:meth:`register_callback`.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.deregister_all_callbacks()
        """
        cls._callbacks.clear()

    @classmethod
    def enable_usd_notice_handler(cls, enable: bool) -> None:
        """Enable or disable the USD notice handler.

        If the USD notice handler is disabled, the simulation manager will not receive USD notice events when a new
        Physics Scene is added/removed or when a prim is deleted (in such case, the simulation manager will not
        trigger the :py:attr:`~isaacsim.core.simulation_manager.SimulationEvent.PRIM_DELETED` event).

        Args:
            enable: Whether to enable or disable the USD notice handler.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.enable_usd_notice_handler(True)
        """
        cls._simulation_manager_interface.enable_usd_notice_handler(enable)

    @classmethod
    def enable_fabric_usd_notice_handler(cls, stage_id: int, enable: bool) -> None:
        """Enable or disable the fabric USD notice handler.

        Args:
            stage_id: The stage ID to enable or disable the handler for.
            enable: Whether to enable or disable the fabric USD notice handler.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> stage_id = stage_utils.get_stage_id(stage_utils.get_current_stage())
            >>> SimulationManager.enable_fabric_usd_notice_handler(stage_id, True)
        """
        cls._simulation_manager_interface.enable_fabric_usd_notice_handler(stage_id, enable)

    @classmethod
    def is_fabric_usd_notice_handler_enabled(cls, stage_id: int) -> bool:
        """Check if the fabric USD notice handler is enabled.

        Args:
            stage_id: The stage ID to check for.

        Returns:
            True if the fabric USD notice handler is enabled, otherwise False.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.stage as stage_utils
            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> stage_id = stage_utils.get_stage_id(stage_utils.get_current_stage())
            >>> SimulationManager.is_fabric_usd_notice_handler_enabled(stage_id)
            True
        """
        return cls._simulation_manager_interface.is_fabric_usd_notice_handler_enabled(stage_id)

    @classmethod
    def assets_loading(cls) -> bool:
        """Check if the textures of the assets are being loaded.

        Returns:
            True if the textures of the assets are loading and have not finished yet, otherwise False.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import SimulationManager
            >>>
            >>> SimulationManager.assets_loading()
            False
        """
        return not cls._assets_loaded

    @classmethod
    def enable_default_callbacks(
        cls,
        *,
        enable_warm_start: bool | None = None,
        enable_on_stop: bool | None = None,
        enable_stage_open: bool | None = None,
        enable_stage_close: bool | None = None,
    ) -> None:
        """Enable or disable the default callbacks.

        .. note:

            Disabling the stage open callback also disables the assets loading and loaded callbacks.
            In such case, the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.assets_loading`
            method will always return ``True``.

        Args:
            enable_warm_start: Whether to enable/disable the warm start callback.
            enable_on_stop: Whether to enable/disable the on stop callback.
            enable_stage_open: Whether to enable/disable the stage open callback.
            enable_stage_close: Whether to enable/disable the stage close callback.
        """
        # on play (warm-up)
        if enable_warm_start is not None:
            cls._default_callbacks_state["warm_start"] = enable_warm_start
            if enable_warm_start:
                cls._setup_warm_start_callback()
            else:
                cls._default_callback_warm_start = None
        # on stop
        cls._default_callbacks_state["on_stop"] = enable_on_stop
        if enable_on_stop:
            cls._setup_on_stop_callback()
        else:
            cls._default_callback_on_stop = None
        # on stage opened
        cls._default_callbacks_state["stage_open"] = enable_stage_open
        if enable_stage_open:
            cls._setup_stage_open_callback()
        else:
            cls._default_callback_stage_open = None
            cls._reset(reset_assets=True)
        # on stage closed
        cls._default_callbacks_state["stage_close"] = enable_stage_close
        if enable_stage_close:
            cls._setup_stage_close_callback()
        else:
            cls._default_callback_stage_close = None
            cls._reset(reset_assets=True)

    # Convenience methods for bulk operations
    @classmethod
    def enable_all_default_callbacks(cls, enable: bool = True) -> None:
        """Enable or disable all default callbacks.

        Args:
            enable: Whether to enable/disable all default callbacks.
        """
        cls.enable_default_callbacks(
            enable_warm_start=enable,
            enable_on_stop=enable,
            enable_stage_open=enable,
            enable_stage_close=enable,
        )
        # deprecated callbacks
        cls.enable_post_warm_start_callback(enable)

    @classmethod
    def is_default_callback_enabled(cls, callback_name: str) -> bool:
        """Check if a specific default callback is enabled.

        Default callbacks names are: ``warm_start``, ``on_stop``, ``stage_open``, ``stage_close``.

        Args:
            callback_name: Name of the callback to check.

        Returns:
            Whether the callback is enabled.
        """
        return cls._default_callbacks_state.get(callback_name, False)

    @classmethod
    def get_default_callback_status(cls) -> dict:
        """Get the status of all default callbacks.

        Default callbacks names are: ``warm_start``, ``on_stop``, ``stage_open``, ``stage_close``.

        Returns:
            Dictionary with callback names and their enabled status.
        """
        return cls._default_callbacks_state.copy()

    """
    Deprecated internal methods.
    """

    @classmethod
    def _get_backend_utils(cls) -> str:
        """Get the backend utilities module based on the current backend.

        Returns:
            The backend utilities module corresponding to the current backend (numpy, torch, or warp).

        Raises:
            Exception: If the backend is not supported.
        """
        # defer imports to avoid an explicit dependency on the deprecated core API
        if SimulationManager._backend == "numpy":
            import isaacsim.core.utils.numpy as np_utils

            return np_utils
        elif SimulationManager._backend == "torch":
            import isaacsim.core.utils.torch as torch_utils

            return torch_utils
        elif SimulationManager._backend == "warp":
            import isaacsim.core.utils.warp as warp_utils

            return warp_utils
        else:
            raise Exception(
                f"Provided backend is not supported: {SimulationManager.get_backend()}. Supported: torch, numpy, warp."
            )

    @classmethod
    def _get_physics_scene_api(cls, physics_scene: str | None = None) -> Any:
        """Get the PhysX scene API for the specified physics scene.

        Args:
            physics_scene: Path to the physics scene prim.

        Returns:
            The PhysX scene API for the specified physics scene, or None if not found.
        """
        if physics_scene:
            if physics_scene in cls._physics_scenes:
                return prim_utils.ensure_api(cls._physics_scenes[physics_scene].prim, PhysxSchema.PhysxSceneAPI)
            carb.log_warn(f"The physics scene at path '{physics_scene}' doesn't exist")
            return None

        if cls._physics_scenes:
            _physics_scene = next(iter(cls._physics_scenes.values()))
            if cls._default_physics_scene_path:
                try:
                    _physics_scene = cls._physics_scenes[cls._default_physics_scene_path]
                except KeyError:
                    carb.log_warn(
                        f"Invalid default physics scene path: {cls._default_physics_scene_path}. "
                        f"Using first physics scene found in stage: {_physics_scene.path}"
                    )
            return prim_utils.ensure_api(_physics_scene.prim, PhysxSchema.PhysxSceneAPI)
        carb.log_warn("No physics scene is found in stage")
        return None

    """
    Deprecated methods.
    """

    @classmethod
    def enable_post_warm_start_callback(cls, enable: bool = True) -> None:
        """Enable or disable the post warm start callback.

        .. deprecated:: 1.8.0

            |br| The :py:attr:`~isaacsim.core.simulation_manager.IsaacEvents.PHYSICS_WARMUP` event is deprecated.
            Calling this method will have no effect.

        Args:
            enable: Whether to enable the callback.
        """
        cls._default_callbacks_state["post_warm_start"] = enable

    @classmethod
    def enable_warm_start_callback(cls, enable: bool = True) -> None:
        """Enable or disable the warm start callback.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.enable_default_callbacks` method instead.

        Args:
            enable: Whether to enable the callback.
        """
        cls._default_callbacks_state["warm_start"] = enable
        if enable:
            cls._setup_warm_start_callback()
        else:
            if cls._default_callback_warm_start is not None:
                cls._default_callback_warm_start = None

    @classmethod
    def enable_on_stop_callback(cls, enable: bool = True) -> None:
        """Enable or disable the on stop callback.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.enable_default_callbacks` method instead.

        Args:
            enable: Whether to enable the callback.
        """
        cls._default_callbacks_state["on_stop"] = enable
        if enable:
            cls._setup_on_stop_callback()
        else:
            if cls._default_callback_on_stop is not None:
                cls._default_callback_on_stop = None

    @classmethod
    def enable_stage_open_callback(cls, enable: bool = True) -> None:
        """Enable or disable the stage open callback.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.enable_default_callbacks` method instead.

        Note: This also enables/disables the assets loading and loaded callbacks. If disabled, assets_loading() will always return True.

        Args:
            enable: Whether to enable the callback.
        """
        cls._default_callbacks_state["stage_open"] = enable
        if enable:
            cls._setup_stage_open_callback()
        else:
            if cls._default_callback_stage_open is not None:
                cls._default_callback_stage_open = None
                # Reset assets loading and loaded callbacks
                cls._assets_loaded = True
                cls._assets_loading_callback = None
                cls._assets_loaded_callback = None

    @classmethod
    def set_backend(cls, val: str) -> None:
        """Set the backend used by the simulation manager.

        .. deprecated:: 1.8.0

            |br| No replacement is provided, as the core experimental API relies solely on Warp.

        Args:
            val: Backend name.
        """
        SimulationManager._backend = val

    @classmethod
    def get_backend(cls) -> str:
        """Get the backend used by the simulation manager.

        .. deprecated:: 1.8.0

            |br| No replacement is provided, as the core experimental API relies solely on Warp.

        Returns:
            Backend name.
        """
        return SimulationManager._backend

    @classmethod
    def get_physics_sim_view(cls) -> Any:
        """Get the physics simulation view.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.get_physics_simulation_view` method instead.

        Returns:
            Physics simulation view.
        """
        return cls._physics_sim_view

    @classmethod
    def set_default_physics_scene(cls, physics_scene_prim_path: str) -> None:
        """Set the default physics scene.

        .. deprecated:: 1.8.0

            |br| No replacement is provided, as there is no default physics scene.

        Args:
            physics_scene_prim_path: The path to the physics scene prim.
        """
        if cls._default_callback_warm_start is None:
            carb.log_warn("Calling set_default_physics_scene while SimulationManager is not tracking physics scenes")
            return
        if physics_scene_prim_path in cls._physics_scenes:
            cls._default_physics_scene_path = physics_scene_prim_path
        elif prim_utils.get_prim_at_path(physics_scene_prim_path).IsValid():
            prim = prim_utils.get_prim_at_path(physics_scene_prim_path)
            if prim.GetTypeName() == "PhysicsScene":
                cls._physics_scenes[physics_scene_prim_path] = cls._create_physics_scene(physics_scene_prim_path)
                cls._default_physics_scene_path = physics_scene_prim_path
        else:
            raise Exception(f"Physics scene at path '{physics_scene_prim_path}' doesn't exist")

    @classmethod
    def get_default_physics_scene(cls) -> str:
        """Get the default physics scene.

        .. deprecated:: 1.8.0

            |br| No replacement is provided, as there is no default physics scene.

        Returns:
            The path to the default physics scene.
        """
        if cls._physics_scenes:
            if cls._default_physics_scene_path in cls._physics_scenes:
                return cls._default_physics_scene_path
            else:
                carb.log_warn(f"Invalid default physics scene path: {cls._default_physics_scene_path}")
                return None
        carb.log_warn("Unable to get default physics scene. No physics scene found")
        return None

    @classmethod
    def set_physics_sim_device(cls, val: str) -> None:
        """Set the physics simulation device.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.set_device` method instead.

        Args:
            val: Physics simulation device.
        """
        cls.set_device(val)

    @classmethod
    def get_physics_sim_device(cls) -> str:
        """Get the physics simulation device.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.SimulationManager.get_device` method instead.

        Returns:
            Physics simulation device as a string.
        """
        device = cls.get_device()
        if device.is_cuda:
            return f"cuda:{device.ordinal}"
        elif device.is_cpu:
            return "cpu"
        else:
            raise Exception(f"Unknown device: {device}")

    @classmethod
    @classmethod
    def set_physics_dt(cls, dt: float = 1.0 / 60.0, physics_scene: str = None) -> None:
        """Set the physics dt on the physics scene provided.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysicsScene.set_dt` method
            or the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_steps_per_second` method
            instead for each targeted Physics Scene.

        Args:
            dt: Physics dt.
            physics_scene: Physics scene prim path.

        Raises:
            RuntimeError: If the simulation is running/playing and dt is being set.
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
            ValueError: If the dt is not in the range [0.0, 1.0].
        """
        if app_utils.is_playing():
            raise RuntimeError("The physics dt cannot be set while the simulation is running/playing")
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            _physics_scene.set_dt(dt)

    @classmethod
    def get_physics_dt(cls, physics_scene: str | None = None) -> float:
        """Get the current physics dt.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysicsScene.get_dt` method
            or the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_steps_per_second` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Physics dt.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        # Get the specific physics scene or the first one available
        _physics_scene = cls._physics_scenes.get(physics_scene) if physics_scene else None
        if _physics_scene is None and cls._physics_scenes:
            _physics_scene = next(iter(cls._physics_scenes.values()))
        if _physics_scene is None:
            return 1.0 / 60.0
        # Use the physics scene's get_dt() method which handles engine-specific attributes
        return _physics_scene.get_dt()

    @classmethod
    def get_broadphase_type(cls, physics_scene: str | None = None) -> str:
        """Get current broadphase algorithm type.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_broadphase_type` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Broadphase algorithm used.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scene_api = cls._get_physics_scene_api(physics_scene=physics_scene)
        return physics_scene_api.GetBroadphaseTypeAttr().Get()

    @classmethod
    def set_broadphase_type(cls, val: str, physics_scene: str | None = None) -> None:
        """Broadphase algorithm used in simulation.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_broadphase_type` method
            instead for each targeted Physics Scene.

        Args:
            val: Broadphase algorithm type (e.g. "MBP", "GPU", "SAP").
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            if isinstance(_physics_scene, PhysxScene):
                _physics_scene.set_broadphase_type(val)

    @classmethod
    def enable_ccd(cls, flag: bool, physics_scene: str | None = None) -> None:
        """Enable Continuous Collision Detection (CCD).

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_enabled_ccd` method
            instead for each targeted Physics Scene.

        Args:
            flag: Enables or disables CCD on the PhysicsScene.
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if flag and "cuda" in cls.get_physics_sim_device():
            carb.log_warn("CCD is not supported on GPU, ignoring request to enable it")
            return
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            if isinstance(_physics_scene, PhysxScene):
                _physics_scene.set_enabled_ccd(flag)

    @classmethod
    def is_ccd_enabled(cls, physics_scene: str | None = None) -> bool:
        """Check if Continuous Collision Detection (CCD) is enabled.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_enabled_ccd` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if CCD is enabled, otherwise False.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physx_scene_api = cls._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableCCDAttr().Get()

    @classmethod
    def enable_gpu_dynamics(cls, flag: bool, physics_scene: str | None = None) -> None:
        """Enable gpu dynamics pipeline, required for deformables for instance.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_enabled_gpu_dynamics` method
            instead for each targeted Physics Scene.

        Args:
            flag: Enables or disables gpu dynamics on the PhysicsScene. If flag is true, CCD is disabled.
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            if isinstance(_physics_scene, PhysxScene):
                _physics_scene.set_enabled_gpu_dynamics(flag)

    @classmethod
    def is_gpu_dynamics_enabled(cls, physics_scene: str | None = None) -> bool:
        """Check if Gpu Dynamics is enabled.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_enabled_gpu_dynamics` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if Gpu Dynamics is enabled, otherwise False.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physx_scene_api = cls._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableGPUDynamicsAttr().Get()

    @classmethod
    def set_solver_type(cls, solver_type: str, physics_scene: str | None = None) -> None:
        """Set the solver type used for simulation.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_solver_type` method
            instead for each targeted Physics Scene.

        Args:
            solver_type: Can be "TGS" or "PGS".
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            if isinstance(_physics_scene, PhysxScene):
                _physics_scene.set_solver_type(solver_type)

    @classmethod
    def get_solver_type(cls, physics_scene: str | None = None) -> str:
        """Get current solver type.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_solver_type` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Solver used for simulation.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physx_scene_api = cls._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetSolverTypeAttr().Get()

    @classmethod
    def enable_stablization(cls, flag: bool, physics_scene: str | None = None) -> None:
        """Enable additional stabilization pass in the solver.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_enabled_stabilization` method
            instead for each targeted Physics Scene.

        Args:
            flag: Enables or disables stabilization on the PhysicsScene
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physics_scenes = [item] if (item := cls._physics_scenes.get(physics_scene)) else cls._physics_scenes.values()
        for _physics_scene in physics_scenes:
            if isinstance(_physics_scene, PhysxScene):
                _physics_scene.set_enabled_stabilization(flag)

    @classmethod
    def is_stablization_enabled(cls, physics_scene: str = None) -> bool:
        """Check if stabilization is enabled.

        .. deprecated:: 1.8.0

            |br| Use the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_enabled_stabilization` method
            instead for each targeted Physics Scene.

        Args:
            physics_scene: Physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if stabilization is enabled, otherwise False.
        """
        if not cls._physics_scenes:
            cls._create_physics_scene()
        physx_scene_api = cls._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableStabilizationAttr().Get()
