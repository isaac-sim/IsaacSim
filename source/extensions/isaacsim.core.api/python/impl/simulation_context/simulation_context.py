# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import builtins
import gc

# python
from typing import Callable, Optional

import carb

# isaac-core
import isaacsim.core.utils.numpy as np_utils
import isaacsim.core.utils.torch as torch_utils
import isaacsim.core.utils.warp as warp_utils

# omniverse
import omni.kit.app
import omni.physics.tensors
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.carb import get_carb_setting, set_carb_setting
from isaacsim.core.utils.prims import get_prim_type_name, is_prim_ancestral, is_prim_no_delete
from isaacsim.core.utils.stage import (
    clear_stage,
    create_new_stage,
    create_new_stage_async,
    get_current_stage,
    set_stage_units,
    set_stage_up_axis,
    update_stage_async,
    use_stage,
)
from pxr import Usd


class SimulationContext:
    """This class provide functions that take care of many time-related events such as
    perform a physics or a render step for instance. Adding/ removing callback functions that
    gets triggered with certain events such as a physics step, timeline event
    (pause or play..etc), stage open/ close..etc.

    It also includes an instance of PhysicsContext which takes care of many physics related
    settings such as setting physics dt, solver type..etc.

    Args:
        physics_dt (Optional[float], optional): dt between physics steps. Defaults to None.
        rendering_dt (Optional[float], optional):  dt between rendering steps. Note: rendering means
                                                   rendering a frame of the current application and not
                                                   only rendering a frame to the viewports/cameras. So UI
                                                   elements of Isaac Sim will be refreshed with this dt
                                                   as well if running non-headless.
                                                   Defaults to None.
        stage_units_in_meters (Optional[float], optional): The metric units of assets. This will affect gravity value..etc.
                                                  Defaults to None.
        physics_prim_path (Optional[str], optional): specifies the prim path to create a PhysicsScene at,
                                             only in the case where no PhysicsScene already defined.
                                             Defaults to "/physicsScene".
        set_defaults (bool, optional): set to True to use the defaults settings
                                        [physics_dt = 1.0/ 60.0,
                                        stage units in meters = 0.01 (i.e in cms),
                                        rendering_dt = 1.0 / 60.0,
                                        gravity = -9.81 m / s
                                        ccd_enabled,
                                        stabilization_enabled,
                                        gpu dynamics turned off,
                                        broadcast type is MBP,
                                        solver type is TGS]. Defaults to True.
        backend (str, optional): specifies the backend to be used (numpy or torch or warp). Defaults to numpy.
        device (Optional[str], optional): specifies the device to be used if running on the gpu with torch or warp backend.
        stage (Optional[Usd.Stage], optional): specifies the stage to be used. Defaults to None.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api import SimulationContext
        >>>
        >>> simulation_context = SimulationContext()
        >>> simulation_context
        <isaacsim.core.api.simulation_context.simulation_context.SimulationContext object at 0x...>
    """

    _instance = None
    _sim_context_initialized = False

    def __init__(
        self,
        physics_dt: Optional[float] = None,
        rendering_dt: Optional[float] = None,
        stage_units_in_meters: Optional[float] = None,
        physics_prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
        backend: str = "numpy",
        device: Optional[str] = None,
        stage: Optional[Usd.Stage] = None,
    ) -> None:
        if SimulationContext._sim_context_initialized:
            return
        SimulationContext._sim_context_initialized = True
        self._app = omni.kit.app.get_app_interface()
        self._extension_manager = omni.kit.app.get_app().get_extension_manager()
        self._framework = carb.get_framework()
        self._initial_stage_units_in_meters = stage_units_in_meters
        self._initial_physics_dt = physics_dt
        self._initial_rendering_dt = rendering_dt
        self._initial_physics_prim_path = physics_prim_path
        self._set_defaults = set_defaults
        self._sim_params = sim_params
        SimulationManager.set_backend(backend)
        self._device = device
        self._stage = stage
        self._settings = carb.settings.get_settings()
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.set_auto_update(True)
        self._physx_fabric_interface = None
        self._physics_callback_functions = dict()
        self._physics_functions = dict()
        self._stage_callback_functions = dict()
        self._timeline_callback_functions = dict()
        self._render_callback_functions = dict()
        self._physics_context = None
        self._current_time = 0
        self._skip_next_stage_open_callback_fn = False
        if self._set_defaults:
            if self._initial_rendering_dt is None:
                self._initial_rendering_dt = 1.0 / 60.0
            if self._initial_stage_units_in_meters is None:
                self._initial_stage_units_in_meters = 1.0
        else:
            if self._initial_rendering_dt is None:
                self._initial_rendering_dt = 1.0 / get_current_stage().GetTimeCodesPerSecond()

        if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:

            if self.is_playing():
                self.stop()
            self._init_stage(
                physics_dt=physics_dt,
                rendering_dt=self._initial_rendering_dt,
                stage_units_in_meters=self._initial_stage_units_in_meters,
                physics_prim_path=physics_prim_path,
                sim_params=sim_params,
                set_defaults=set_defaults,
                backend=backend,
                device=device,
            )
            self._setup_default_callback_fns()
            self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._stage_open_callback_fn,
                observer_name="isaacsim.core.api.SimulationContext._stage_open_callback",
            )
        self._message_bus = carb.eventdispatcher.get_eventdispatcher()
        self._on_post_physics_ready_callback = self._message_bus.observe_event(
            event_name=IsaacEvents.PHYSICS_READY.value,
            on_event=self._on_post_physics_ready,
            observer_name="SimulationContext._on_post_physics_ready",
        )
        return

    def __new__(cls, *args, **kwargs) -> SimulationContext:
        """Makes the class a singleton.

        Returns:
            SimulationContext: The instance of the simulation context.
        """
        if SimulationContext._instance is None:
            SimulationContext._instance = super(SimulationContext, cls).__new__(cls)
        else:
            carb.log_info("Simulation Context is defined already, returning the previously defined one")
        return SimulationContext._instance

    """
    Instance handling.
    """

    @classmethod
    def instance(cls) -> SimulationContext:
        """Get the instance of the class, if it was instantiated before

        Returns:
            SimulationContext: SimulationContext object or None

        Example:

        .. code-block:: python

            >>> # given that the class has already been instantiated before
            >>> simulation_context = SimulationContext.instance()
            >>> simulation_context
            <isaacsim.core.api.simulation_context.simulation_context.SimulationContext object at 0x...>
        """
        return SimulationContext._instance

    @classmethod
    def clear_instance(cls) -> None:
        """Delete the simulation context object, if it was instantiated before, and destroy any subscribed callback

        Example:

        .. code-block:: python

            >>> SimulationContext.clear_instance()
        """
        if SimulationContext._instance is not None:
            SimulationContext._instance.clear_all_callbacks()
            SimulationContext._instance._stage_open_callback = None
            SimulationContext._instance._physics_timer_callback = None
            SimulationContext._instance._event_timer_callback = None
            SimulationContext._instance = None
            SimulationContext._sim_context_initialized = False
            SimulationManager.set_backend("numpy")
            SimulationManager.set_physics_sim_device("cpu")
        return

    """
    Properties.
    """

    @property
    def app(self) -> omni.kit.app.IApp:
        """Returns:
            omni.kit.app.IApp: Omniverse Kit Application interface

        Example:

        .. code-block:: python

            >>> simulation_context.app
            <omni.kit.app._app.IApp object at 0x...>
        """
        return self._app

    @property
    def current_time_step_index(self) -> int:
        """
        Returns:
            int: current number of physics steps that have elapsed since the simulation was played

        Example:

        .. code-block:: python

            >>> # given a running Isaac Sim instance and after approximately 15 seconds of physics simulation at 60 Hz
            >>> simulation_context.current_time_step_index
            911
        """
        return self._number_of_steps

    @property
    def current_time(self) -> float:
        """
        Returns:
            float: current time (simulated physical time) that have elapsed since the simulation was played

        Example:

        .. code-block:: python

            >>> # given a running Isaac Sim instance and after 911 physics steps at 60 Hz
            >>> simulation_context.current_time
            15.183334125205874
        """
        return self._current_time

    @property
    def stage(self) -> Usd.Stage:
        """
        Returns:
            Usd.Stage: current open USD stage

        Example:

        .. code-block:: python

            >>> simulation_context.stage
            Usd.Stage.Open(rootLayer=Sdf.Find('anon:0x...:World....usd'),
                           sessionLayer=Sdf.Find('anon:0x...:World...-session.usda'),
                           pathResolverContext=<invalid repr>)
        """
        if self._stage is None:
            return get_current_stage()
        return self._stage

    @property
    def backend(self) -> str:
        """
        Returns:
            str: current backend. Supported backends are: ``"numpy"``, ``"torch"`` and ``"warp"``

        Example:

        .. code-block:: python

            >>> simulation_context.backend
            numpy
        """
        return SimulationManager.get_backend()

    @property
    def device(self) -> str:
        """
        Returns:
            str: Device used by the physics context. None for numpy backend

        Example:

        .. code-block:: python

            >>> simulation_context.device
            None
        """
        return SimulationManager.get_physics_sim_device()

    @property
    def backend_utils(self):
        """Get the current backend utils module

        .. list-table::
            :header-rows: 1

            * - Backend
              - Utils module
            * - ``"numpy"``
              - ``isaacsim.core.utils.numpy``
            * - ``"torch"``
              - ``isaacsim.core.utils.torch``
            * - ``"warp"``
              - ``isaacsim.core.utils.warp``

        Returns:
            str: current backend utils module

        Example:

        .. code-block:: python

            >>> simulation_context.backend_utils
            <module 'isaacsim.core.utils.numpy'>
        """
        return SimulationManager._get_backend_utils()

    @property
    def physics_sim_view(self):
        """
        .. note::

            The physics simulation view instance will be only available after initializing the physics
            (see ``initialize_physics``) or resetting the simulation context (see ``reset``)

        Returns:
            PhysicsContext: Physics simulation view instance

        Example:

        .. code-block:: python

            >>> simulation_context.physics_sim_view
            <omni.physics.tensors.impl.api.SimulationView object at 0x...>
        """
        return SimulationManager.get_physics_sim_view()

    """
    Operations - Physics.
    """

    def get_physics_context(self) -> PhysicsContext:
        """Get the physics context (a class to deal with a physics scene and its settings) instance

        Raises:
            Exception: if there is no stage currently opened

        Returns:
            PhysicsContext: physics context object

        Example:

        .. code-block:: python

            >>> simulation_context.get_physics_context()
            <isaacsim.core.api.physics_context.physics_context.PhysicsContext object at 0x...>
        """
        if self.stage is None:
            raise Exception("There is no stage currently opened")
        return self._physics_context

    """
    Operations- Simulation time.
    """

    def set_simulation_dt(self, physics_dt: Optional[float] = None, rendering_dt: Optional[float] = None) -> None:
        """Specify the physics step and rendering step size to use when stepping and rendering.

        Args:
            physics_dt (float): The physics time-step. None means it won't change the current setting. (default: None).
            rendering_dt (float):  The rendering time-step. None means it won't change the current setting. (default: None)

        .. hint::

            It is recommended that the two values be divisible, with the ``rendering_dt`` being equal to or greater
            than the ``physics_dt``

        Example:

        .. code-block:: python

            >>> set physics dt to 120 Hz and rendering dt to 60Hz (2 physics steps for each rendering)
            >>> simulation_context.set_simulation_dt(physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
        """
        if self.stage is None:
            raise Exception("There is no stage currently opened, init_stage needed before calling this func")
        # If the user sets none we assume they don't care and want to use defaults (1.0/60.0)
        if rendering_dt is None:
            rendering_dt = self.get_rendering_dt()
        elif rendering_dt < 0:
            raise ValueError("rendering_dt cannot be <0")
        # if rendering is called the substeps term is used to determine how many physics steps to perform per rendering step
        # is is not used if step(render=False)
        if physics_dt is not None:
            if physics_dt > 0:
                substeps = max(int(rendering_dt / physics_dt), 1)
            else:
                substeps = 1
            self._physics_context.set_physics_dt(physics_dt, substeps)

        rendering_hz = 0
        if rendering_dt > 0:
            rendering_hz = 1.0 / rendering_dt

        # If rate limiting is enabled, set the rendering rate to the specified value
        # Otherwise run the app as fast as possible and do not specify the target rate
        if get_carb_setting(self._settings, "/app/runLoops/main/rateLimitEnabled"):
            set_carb_setting(self._settings, "/app/runLoops/main/rateLimitFrequency", rendering_hz)
            self._timeline.set_target_framerate(rendering_hz)
        with Usd.EditContext(get_current_stage(), get_current_stage().GetRootLayer()):
            get_current_stage().SetTimeCodesPerSecond(rendering_hz)
        self._timeline.set_time_codes_per_second(rendering_hz)
        self._rendering_dt = rendering_dt
        # The isaac sim loop runner is enabled by default in isaac sim apps, but in case we are in an app with the kit loop runner, protect against this
        try:
            import omni.kit.loop._loop as omni_loop

            _loop_runner = omni_loop.acquire_loop_interface()
            _loop_runner.set_manual_step_size(rendering_dt)
            _loop_runner.set_manual_mode(True)
        except Exception:
            carb.log_warn(
                "Isaac Sim loop runner not found, enabling rate limiting to support rendering at specified rate"
            )
            set_carb_setting(self._settings, "/app/runLoops/main/rateLimitEnabled", True)
            set_carb_setting(self._settings, "/app/runLoops/main/rateLimitFrequency", rendering_hz)
            self._timeline.set_target_framerate(rendering_hz)
            pass

        return

    def get_physics_dt(self) -> float:
        """Get the current physics dt of the physics context

        Raises:
            Exception: if there is no stage currently opened

        Returns:
            float: current physics dt of the PhysicsContext

        Example:

        .. code-block:: python

            >>> simulation_context.get_physics_dt()
            0.016666666666666666
        """
        if self.stage is None:
            raise Exception("There is no stage currently opened")
        return self._physics_context.get_physics_dt()

    def get_rendering_dt(self) -> float:
        """Get the current rendering dt

        Raises:
            Exception: if there is no stage currently opened

        Returns:
            float: current rendering dt

        Example:

        .. code-block:: python

            >>> simulation_context.get_rendering_dt()
            0.016666666666666666
        """
        if self.stage is None:
            raise Exception("There is no stage currently opened")

        # Helper function to get dt from frequency
        def _get_dt_from_frequency():
            frequency = get_carb_setting(self._settings, "/app/runLoops/main/rateLimitFrequency")
            return 1.0 / frequency if frequency else 0

        if get_carb_setting(self._settings, "/app/runLoops/main/rateLimitEnabled"):
            return _get_dt_from_frequency()

        try:
            import omni.kit.loop._loop as omni_loop

            _loop_runner = omni_loop.acquire_loop_interface()
            if _loop_runner.get_manual_mode():
                return _loop_runner.get_manual_step_size()
            else:
                return _get_dt_from_frequency()
        except Exception:
            return _get_dt_from_frequency()

    def set_block_on_render(self, block: bool) -> None:
        """Set block on render flag for the simulation thread

        .. note::

            This guarantee a one frame lag between any data captured from the render products and the current USD stage if enabled.

        Args:
            block (bool): True to block the thread until the renderer is done.

        Example:

        .. code-block:: python

            >>> simulation_context.set_block_on_render(False)
        """
        set_carb_setting(self._settings, "/app/hydraEngine/waitIdle", block)

    def get_block_on_render(self) -> bool:
        """Get the block on render flag for the simulation thread

        Returns:
            bool: True if one frame lag between any data captured from the render products and the current USD stage is guaranteed by blocking the step call.

        Example:

        .. code-block:: python

            >>> simulation_context.get_block_on_render()
            False
        """
        return get_carb_setting(self._settings, "/app/hydraEngine/waitIdle")

    """
    Operations.
    """

    async def initialize_simulation_context_async(self) -> None:
        """Initialize the simulation context

        .. hint::

            This method is intended to be used in the Isaac Sim's Extensions workflow where
            the Kit application has the control over timing of physics and rendering steps

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await simulation_context.initialize_simulation_context_async()
            ...
            >>> run_coroutine(task())
        """
        if self.is_playing():
            await self.stop_async()
        await omni.kit.app.get_app().next_update_async()
        await self._initialize_stage_async(
            physics_dt=self._initial_physics_dt,
            rendering_dt=self._initial_rendering_dt,
            stage_units_in_meters=self._initial_stage_units_in_meters,
            physics_prim_path=self._initial_physics_prim_path,
            sim_params=self._sim_params,
            device=self._device,
        )
        await omni.kit.app.get_app().next_update_async()
        self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.core.api.SimulationContext._stage_open_callback",
        )
        await omni.kit.app.get_app().next_update_async()
        self._setup_default_callback_fns()
        await omni.kit.app.get_app().next_update_async()
        return

    def initialize_physics(self) -> None:
        """Initialize the physics simulation view

        Example:

        .. code-block:: python

            >>> simulation_context.initialize_physics()
        """
        if self.is_stopped() and not builtins.ISAAC_LAUNCHED_FROM_TERMINAL:
            self.play()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()
        return

    def reset(self, soft: bool = False) -> None:
        """Reset the physics simulation view.

        .. warning::

            This method is not intended to be used in the Isaac Sim's Extensions workflow since the Kit application
            has the control over the rendering steps. For the Extensions workflow use the ``reset_async`` method instead

        Args:
            soft (bool, optional): if set to True simulation won't be stopped and start again. It only calls the reset on the scene objects.

        Example:

        .. code-block:: python

            >>> simulation_context.reset()
        """
        if not soft:
            if not self.is_stopped():
                self.stop()
            self.play()
            if SimulationManager.get_physics_sim_view() is None:
                SimulationManager.initialize_physics()

    async def reset_async(self, soft: bool = False) -> None:
        """Reset the physics simulation view (asynchronous version).

        Args:
            soft (bool, optional): if set to True simulation won't be stopped and start again. It only calls the reset on the scene objects.

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            >>>     await simulation_context.reset_async()
            >>>
            >>> run_coroutine(task())
        """
        if not soft:
            if not self.is_stopped():
                await self.stop_async()
            await self.play_async()
            if SimulationManager.get_physics_sim_view() is None:
                SimulationManager.initialize_physics()

    def step(self, render: bool = True) -> None:
        """Steps the physics simulation while rendering or without.

        .. warning::

            Calling this method with the ``render`` parameter set to True (default value) is not intended to be used
            in the Isaac Sim's Extensions workflow since the Kit application has the control over the rendering steps

        Args:
            render (bool, optional): Set to False to only do a physics simulation without rendering. Note:
                                     app UI will be frozen (since its not rendering) in this case.
                                     Defaults to True.

        Raises:
            Exception: if there is no stage currently opened

        Example:

        .. code-block:: python

            >>> simulation_context.step()
        """
        if self.stage is None:
            raise Exception("There is no stage currently opened, init_stage needed before calling this func")
        if render:
            # physics dt is zero, no need to step physics, just render
            if self.get_physics_dt() == 0:
                SimulationContext.render(self)
            # rendering dt is zero, but physics is not, call step and then render
            elif self.get_rendering_dt() == 0 and self.get_physics_dt() != 0:
                if self.is_playing():
                    self._physics_context._step(current_time=self.current_time)
                SimulationContext.render(self)
            else:
                self._app.update()
        else:
            if self.is_playing():
                self._physics_context._step(current_time=self.current_time)
        return

    def render(self) -> None:
        """Refresh the Isaac Sim app rendering components including UI elements, viewports and others

        .. warning::

            This method is not intended to be used in the Isaac Sim's Extensions workflow
            since the Kit application has the control over the rendering steps

        Example:

        .. code-block:: python

            >>> simulation_context.render()
        """
        if (
            self.device is not None
            and "cuda" in self.device
            and self.physics_sim_view is not None
            and self.is_playing()
        ):
            self.physics_sim_view.update_articulations_kinematic()
        if self._physx_fabric_interface is None:
            if self.current_time > 0 and self._extension_manager.is_extension_enabled("omni.physx.fabric"):
                from omni.physxfabric import get_physx_fabric_interface

                self._physx_fabric_interface = get_physx_fabric_interface()
        if self._physx_fabric_interface:
            self._physx_fabric_interface.force_update(self._physics_context.get_physics_dt(), self.current_time)
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        self._app.update()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    async def render_async(self) -> None:
        """Refresh the Isaac Sim app rendering components including UI elements, viewports and others

        .. hint::

            This method is intended to be used in the Isaac Sim's Extensions workflow where
            the Kit application has the control over timing of physics and rendering steps

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await simulation_context.render_async()
            ...
            >>> run_coroutine(task())
        """
        if (
            self.device is not None
            and "cuda" in self.device
            and self.physics_sim_view is not None
            and self.is_playing()
        ):
            self.physics_sim_view.update_articulations_kinematic()

        if self._physx_fabric_interface is None:
            if self.current_time > 0 and self._extension_manager.is_extension_enabled("omni.physx.fabric"):
                from omni.physxfabric import get_physx_fabric_interface

                self._physx_fabric_interface = get_physx_fabric_interface()
        if self._physx_fabric_interface:
            self._physx_fabric_interface.force_update(self._physics_context.get_physics_dt(), self.current_time)
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        await omni.kit.app.get_app().next_update_async()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    def clear(self) -> None:
        """Clear the current stage leaving the PhysicsScene and /World

        Example:

        .. code-block:: python

            >>> simulation_context.clear()
        """

        def check_deletable_prim(prim_path):
            if is_prim_no_delete(prim_path):
                return False
            if is_prim_ancestral(prim_path):
                return False
            if get_prim_type_name(prim_path=prim_path) == "PhysicsScene":
                return False
            if prim_path == "/World":
                return False
            if prim_path == "/":
                return False
            # Don't remove any /Render prims as that can cause crashes
            if prim_path.startswith("/Render"):
                return False
            return True

        clear_stage(predicate=check_deletable_prim)

    """
    Operations (will be deprecated).
    """

    def is_simulating(self) -> bool:
        """Check whether the simulation is running or not

        .. warning::

            With deprecation of Dynamic Control Toolbox, this function is not needed

            It can return True if start_simulation is called even if play was pressed/called.

        Returns:
            bool" True if physics simulation is happening.

        Example:

        .. code-block:: python

            >>> # given a running simulation
            >>> simulation_context.is_simulating()
            True
        """
        return SimulationManager.get_physics_sim_view() is not None

    """
    Operations- Timeline.
    """

    def is_playing(self) -> bool:
        """Check whether the simulation is playing

        Returns:
            bool: True if the simulator is playing.

        Example:

        .. code-block:: python

            >>> # given a simulation in play
            >>> simulation_context.is_playing()
            True
        """
        return self._timeline.is_playing()

    def is_stopped(self) -> bool:
        """Check whether the simulation is playing

        Returns:
            bool: True if the simulator is stopped.

        Example:

        .. code-block:: python

            >>> # given a simulation in play
            >>> simulation_context.is_stopped()
            False
        """
        return self._timeline.is_stopped()

    async def play_async(self) -> None:
        """Start playing simulation

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await simulation_context.play_async()
            ...
            >>> run_coroutine(task())
        """
        self._timeline.play()
        self._timeline.commit()
        self.get_physics_context().warm_start()
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        await omni.kit.app.get_app().next_update_async()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    def play(self) -> None:
        """Start playing simulation

        .. note::

           It does one step internally to propagate all physics handles properly.

        Example:

        .. code-block:: python

            >>> simulation_context.play()
        """
        self._timeline.play()
        self._timeline.commit()
        if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:
            set_carb_setting(self._settings, "/app/player/playSimulations", False)
            self._app.update()
            set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    async def pause_async(self) -> None:
        """Pause the physics simulation

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await simulation_context.pause_async()
            ...
            >>> run_coroutine(task())
        """
        self._timeline.pause()
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        await omni.kit.app.get_app().next_update_async()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    def pause(self) -> None:
        """Pause the physics simulation

        Example:

        .. code-block:: python

            >>> simulation_context.pause()
        """
        self._timeline.pause()
        if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:
            set_carb_setting(self._settings, "/app/player/playSimulations", False)
            self._app.update()
            set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    async def stop_async(self) -> None:
        """Stop the physics simulation

        Example:

        .. code-block:: python

            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await simulation_context.stop_async()
            ...
            >>> run_coroutine(task())
        """
        self._timeline.stop()
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        await omni.kit.app.get_app().next_update_async()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    def stop(self) -> None:
        """Stop the physics simulation

        Example:

        .. code-block:: python

            >>> simulation_context.stop()
        """
        self._timeline.stop()
        if builtins.ISAAC_LAUNCHED_FROM_TERMINAL is False:
            set_carb_setting(self._settings, "/app/player/playSimulations", False)
            self._app.update()
            set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return

    """
    Operations- Callbacks Management.
    """

    def add_physics_callback(self, callback_name: str, callback_fn: Callable[[float], None]) -> None:
        """Add a callback which will be called before each physics step.

        ``callback_fn`` should take a float argument (e.g., ``step_size``)

        Args:
            callback_name (str): should be unique.
            callback_fn (Callable[[float], None]): [description]

        Example:

        .. code-block:: python

            >>> def callback_physics(step_size):
            ...     print("physics callback -> step_size:", step_size)
            ...
            >>> simulation_context.add_physics_callback("callback_physics", callback_physics)
        """
        if callback_name in self._physics_callback_functions:
            carb.log_error(f"Physics callback `{callback_name}` already exists")
            return
        self._physics_callback_functions[callback_name] = (
            self._physics_context._physx_interface.subscribe_physics_step_events(callback_fn)
        )
        self._physics_functions[callback_name] = callback_fn
        return

    def remove_physics_callback(self, callback_name: str) -> None:
        """Remove a physics callback by its name

        Args:
            callback_name (str): callback name

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_physics'
            >>> simulation_context.remove_physics_callback("callback_physics")
        """
        if callback_name in self._physics_callback_functions:
            del self._physics_callback_functions[callback_name]
            del self._physics_functions[callback_name]
        else:
            carb.log_error(f"Physics callback `{callback_name}` doesn't exist")
        return

    def physics_callback_exists(self, callback_name: str) -> bool:
        """Check if a physics callback exists

        Args:
            callback_name (str): callback name

        Returns:
            bool: whether the callback is registered

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_physics'
            >>> simulation_context.physics_callback_exists("callback_physics")
            True
        """
        if callback_name in self._physics_callback_functions:
            return True
        else:
            return False

    def clear_physics_callbacks(self) -> None:
        """Remove all registered physics callbacks

        Example:

        .. code-block:: python

            >>> simulation_context.clear_physics_callbacks()
        """
        self._physics_callback_functions = dict()
        self._physics_functions = dict()
        return

    def add_stage_callback(self, callback_name: str, callback_fn: Callable) -> None:
        """Add a callback which will be called after each stage event such as open/close among others

        ``callback_fn`` should take an argument of type ``omni.usd.StageEvent`` (e.g., ``event``)

        Args:
            callback_name (str): [description]
            callback_fn (Callable[[omni.usd.StageEvent], None]): [description]

        Example:

        .. code-block:: python

            >>> def callback_stage(event):
            ...     print("stage callback -> event:", event)
            ...
            >>> simulation_context.add_stage_callback("callback_stage", callback_stage)
        """
        if callback_name in self._stage_callback_functions:
            carb.log_error(f"Stage callback `{callback_name}` already exists")
            return
        self._stage_callback_functions[callback_name] = (
            omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(callback_fn)
        )
        return

    def remove_stage_callback(self, callback_name: str) -> None:
        """Remove a stage callback by its name

        Args:
            callback_name (str): callback name

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_stage'
            >>> simulation_context.remove_stage_callback("callback_stage")
        """
        if callback_name in self._stage_callback_functions:
            del self._stage_callback_functions[callback_name]
        else:
            carb.log_error(f"Stage callback `{callback_name}` doesn't exist")
        return

    def stage_callback_exists(self, callback_name: str) -> bool:
        """Check if a stage callback exists

        Args:
            callback_name (str): callback name

        Returns:
            bool: whether the callback is registered

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_stage'
            >>> simulation_context.stage_callback_exists("callback_stage")
            True
        """
        if callback_name in self._stage_callback_functions:
            return True
        else:
            return False

    def clear_stage_callbacks(self) -> None:
        """Remove all registered stage callbacks

        Example:

        .. code-block:: python

            >>> simulation_context.clear_stage_callbacks()
        """
        self._stage_callback_functions = dict()
        return

    def add_timeline_callback(self, callback_name: str, callback_fn: Callable) -> None:
        """Add a callback which will be called after each timeline event such as play/pause.

        ``callback_fn`` should take an argument of type ``omni.timeline.TimelineEvent`` (e.g., ``event``)

        Args:
            callback_name (str): [description]
            callback_fn (Callable[[omni.timeline.TimelineEvent], None]): [description]

        Example:

        .. code-block:: python

            >>> def callback_timeline(event):
            ...     print("timeline callback -> event:", event)
            ...
            >>> simulation_context.add_timeline_callback("callback_timeline", callback_timeline)
        """
        if callback_name in self._timeline_callback_functions:
            carb.log_error(f"Timeline callback `{callback_name}` already exists")
            return
        self._timeline_callback_functions[callback_name] = (
            self._timeline.get_timeline_event_stream().create_subscription_to_pop(callback_fn)
        )
        return

    def remove_timeline_callback(self, callback_name: str) -> None:
        """Remove a timeline callback by its name

        Args:
            callback_name (str): callback name

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'timeline'
            >>> simulation_context.timeline_callback("timeline")
        """
        if callback_name in self._timeline_callback_functions:
            del self._timeline_callback_functions[callback_name]
        else:
            carb.log_error(f"Timeline callback `{callback_name}` doesn't exist")
        return

    def timeline_callback_exists(self, callback_name: str) -> bool:
        """Check if a timeline callback exists

        Args:
            callback_name (str): callback name

        Returns:
            bool: whether the callback is registered

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_timeline'
            >>> simulation_context.timeline_callback_exists("callback_timeline")
            True
        """
        if callback_name in self._timeline_callback_functions:
            return True
        else:
            return False

    def clear_timeline_callbacks(self) -> None:
        """Remove all registered timeline callbacks

        Example:

        .. code-block:: python

            >>> simulation_context.clear_timeline_callbacks()
        """
        self._timeline_callback_functions = dict()
        return

    def add_render_callback(self, callback_name: str, callback_fn: Callable) -> None:
        """Add a callback which will be called after each rendering event such as .render().

        ``callback_fn`` should take an argument of type (e.g., ``event``)

        Args:
            callback_name (str): [description]
            callback_fn (Callable): [description]

        Example:

        .. code-block:: python

            >>> def callback_render(event):
            ...     print("render callback -> event:", event)
            ...
            >>> simulation_context.add_render_callback("callback_render", callback_render)
        """
        if callback_name in self._render_callback_functions:
            carb.log_error(f"Render callback `{callback_name}` already exists")
            return
            # TODO: should we raise exception?
        self._render_callback_functions[callback_name] = self.app.get_update_event_stream().create_subscription_to_pop(
            callback_fn
        )
        return

    def remove_render_callback(self, callback_name: str) -> None:
        """Remove a render callback by its name

        Args:
            callback_name (str): callback name

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_render'
            >>> simulation_context.remove_render_callback("callback_render")
        """
        if callback_name in self._render_callback_functions:
            del self._render_callback_functions[callback_name]
        else:
            carb.log_error(f"Editor callback `{callback_name}` doesn't exist")
        return

    def render_callback_exists(self, callback_name: str) -> bool:
        """Check if a render callback exists

        Args:
            callback_name (str): callback name

        Returns:
            bool: whether the callback is registered

        Example:

        .. code-block:: python

            >>> # given a registered callback named 'callback_render'
            >>> simulation_context.render_callback_exists("callback_render")
            True
        """
        if callback_name in self._render_callback_functions:
            return True
        else:
            return False

    def clear_render_callbacks(self) -> None:
        """Remove all registered render callbacks

        Example:

        .. code-block:: python

            >>> simulation_context.clear_render_callbacks()
        """
        self._render_callback_functions = dict()
        return

    def clear_all_callbacks(self) -> None:
        """Clear all callbacks which were added using any ``add_*_callback`` method

        Example:

        .. code-block:: python

            >>> simulation_context.clear_render_callbacks()
        """
        self._physics_callback_functions = dict()
        self._physics_functions = dict()
        self._stage_callback_functions = dict()
        self._timeline_callback_functions = dict()
        self._render_callback_functions = dict()
        gc.collect()
        return

    def skip_next_stage_open_callback(self):
        """Skip the next stage_open_callback_fn trigger"""
        self._skip_next_stage_open_callback_fn = True

    """
    Private helpers.
    """

    def _init_stage(
        self,
        physics_dt: Optional[float] = None,
        rendering_dt: Optional[float] = None,
        stage_units_in_meters: Optional[float] = None,
        physics_prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
        backend: str = "numpy",
        device: Optional[str] = None,
    ) -> Usd.Stage:
        if self.stage is None:
            create_new_stage()
            SimulationContext.render(self)

        with use_stage(self.stage):
            set_stage_up_axis("z")
            if stage_units_in_meters is not None:
                set_stage_units(stage_units_in_meters=stage_units_in_meters)
            SimulationContext.render(self)
            self._physics_context = PhysicsContext(
                physics_dt=physics_dt,
                prim_path=physics_prim_path,
                sim_params=sim_params,
                set_defaults=set_defaults,
            )
            if device is not None:
                SimulationManager.set_physics_sim_device(device)
            self.set_simulation_dt(physics_dt=physics_dt, rendering_dt=rendering_dt)
        return self.stage

    async def _initialize_stage_async(
        self,
        physics_dt: Optional[float] = None,
        rendering_dt: Optional[float] = None,
        stage_units_in_meters: Optional[float] = None,
        physics_prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
        device: Optional[str] = None,
    ) -> Usd.Stage:
        if get_current_stage() is None:
            await create_new_stage_async()
        set_stage_up_axis("z")
        if stage_units_in_meters is not None:
            set_stage_units(stage_units_in_meters=stage_units_in_meters)
        await omni.kit.app.get_app().next_update_async()
        self._physics_context = PhysicsContext(
            physics_dt=physics_dt, prim_path=physics_prim_path, sim_params=sim_params, set_defaults=set_defaults
        )
        if device is not None:
            SimulationManager.set_physics_sim_device(device)
        self.set_simulation_dt(physics_dt=physics_dt, rendering_dt=rendering_dt)
        return self.stage

    def _setup_default_callback_fns(self):
        self._physics_timer_callback = self._physics_context._physx_interface.subscribe_physics_step_events(
            self._physics_timer_callback_fn
        )
        self._event_timer_callback = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self._timeline_timer_callback_fn
        )
        self._physics_callback_functions = dict()
        self._physics_functions = dict()
        self._stage_callback_functions = dict()
        self._timeline_callback_functions = dict()
        self._render_callback_functions = dict()
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.set_auto_update(True)
        self._number_of_steps = 0
        self._current_time = 0
        return

    """
    Default Callbacks.
    """

    def _physics_timer_callback_fn(self, step_size: int):
        self._current_time += step_size
        self._number_of_steps += 1
        return

    def _timeline_timer_callback_fn(self, event):
        # because we use create_subscription_to_pop_by_type for omni.timeline.TimelineEventType.STOP, there is no need to check the type here
        self._current_time = 0
        self._number_of_steps = 0
        for callback_name in list(self._physics_callback_functions.keys()):
            del self._physics_callback_functions[callback_name]

    def _stage_open_callback_fn(self, event):
        # skip the callback if required
        if self._skip_next_stage_open_callback_fn:
            self._skip_next_stage_open_callback_fn = False
            return

        self._physics_callback_functions = dict()
        self._physics_functions = dict()
        self._stage_callback_functions = dict()
        self._timeline_callback_functions = dict()
        self._render_callback_functions = dict()
        if SimulationContext._instance is not None:
            SimulationContext._instance.clear_instance()
            carb.log_warn(
                "A new stage was opened, World or Simulation Object are invalidated and you would need to initialize them again before using them."
            )
        self._stage_open_callback = None
        return

    def _on_post_physics_ready(self, event):
        for callback_name, callback_function in self._physics_functions.items():
            self._physics_callback_functions[callback_name] = (
                self._physics_context._physx_interface.subscribe_physics_step_events(callback_function)
            )
