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

"""Core rendering management APIs for controlling rendering operations and frame updates."""

from __future__ import annotations

import weakref
from fractions import Fraction

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, Usd

_SETTING_PLAY_SIMULATION = "/app/player/playSimulations"
_SETTING_RATE_LIMIT_ENABLED = "/app/runLoops/main/rateLimitEnabled"
_SETTING_RATE_LIMIT_FREQUENCY = "/app/runLoops/main/rateLimitFrequency"
_SETTING_FABRIC_DEFAULT_SIM_PERIOD_NUMERATOR = "/app/settings/fabricDefaultSimPeriodNumerator"
_SETTING_FABRIC_DEFAULT_SIM_PERIOD_DENOMINATOR = "/app/settings/fabricDefaultSimPeriodDenominator"

from enum import Enum


class RenderingEvent(Enum):
    """Rendering event types."""

    NEW_FRAME = "isaacsim.rendering.new_frame"
    """Event triggered when a new frame is rendered."""


class RenderingManager:
    """Core class that provides APIs for controlling rendering."""

    _app = omni.kit.app.get_app()
    """The Omniverse Kit application instance used for rendering operations."""
    _callbacks = {}
    """Dictionary storing registered callback functions mapped by their unique identifiers."""
    _callback_registry = 0
    """Counter for generating unique identifiers for callback registrations."""
    _carb_settings = carb.settings.get_settings()
    """Carbonite settings interface for accessing and modifying application configuration."""
    _event_dispatcher = carb.eventdispatcher.get_eventdispatcher()
    """Carbonite event dispatcher for managing event subscriptions and notifications."""
    _timeline = omni.timeline.get_timeline_interface()
    """Timeline interface used by :meth:`set_dt` to configure ``set_target_framerate`` and ``set_time_codes_per_second``."""
    _fabric_time_stage_id = None
    """Cached stage ID for :meth:`_ensure_fabric_simulation_time` to avoid redundant Fabric writes."""
    try:
        from omni.kit.loop import _loop as kit_loop

        _loop_runner = kit_loop.acquire_loop_interface()
    except Exception as e:
        carb.log_warn(f"Isaac Sim's loop runner not found. Its functionalities will not be used: {e}")
        _loop_runner = None

    @classmethod
    def _ensure_fabric_simulation_time(cls) -> None:
        """Seed ``/ExternalSimulationTime`` in Fabric so the multitick renderer can proceed.

        When :obj:`isaacsim.core.simulation_manager` is loaded it maintains this prim with the
        real physics time on every step. When it is **not** loaded (e.g. rendering-only tests),
        the prim would be missing and the multitick renderer would stall because it cannot
        determine the current simulation time. This method creates the prim with ``time=0.0``
        as a one-time fallback per stage so the viewport can initialise.
        """
        try:
            stage_id = omni.usd.get_context().get_stage_id()
            if not stage_id or stage_id == cls._fabric_time_stage_id:
                return
            fabric_stage = stage_utils.get_current_stage(backend="fabric")
            prim = fabric_stage.GetPrimAtPath("/ExternalSimulationTime")
            if prim and prim.HasAttribute("omni:time"):
                cls._fabric_time_stage_id = stage_id
                return
            prim = fabric_stage.DefinePrim("/ExternalSimulationTime", "")
            attr = prim_utils.create_prim_attribute(prim, name="omni:time", type_name=Sdf.ValueTypeNames.Double)
            attr.Set(0.0)
            cls._fabric_time_stage_id = stage_id
        except Exception:
            pass

    @classmethod
    def render(cls) -> None:
        """Render the stage.

        This method performs an app update without stepping the simulation or physics.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingManager
            >>>
            >>> RenderingManager.render()
        """
        cls._ensure_fabric_simulation_time()
        play_simulation = cls._carb_settings.get_as_bool(_SETTING_PLAY_SIMULATION)
        if play_simulation:
            cls._carb_settings.set_bool(_SETTING_PLAY_SIMULATION, False)
        cls._app.update()
        if play_simulation:
            cls._carb_settings.set_bool(_SETTING_PLAY_SIMULATION, True)

    @classmethod
    async def render_async(cls) -> None:
        """Render the stage.

        This method is the asynchronous version of :py:meth:`render`.
        """
        cls._ensure_fabric_simulation_time()
        play_simulation = cls._carb_settings.get_as_bool(_SETTING_PLAY_SIMULATION)
        if play_simulation:
            cls._carb_settings.set_bool(_SETTING_PLAY_SIMULATION, False)
        await cls._app.next_update_async()
        if play_simulation:
            cls._carb_settings.set_bool(_SETTING_PLAY_SIMULATION, True)

    @classmethod
    def set_dt(cls, dt: float) -> None:
        """Set the application's coherent dt across the run loop, timeline, and loop runner.

        Sets the following, in order:

        1. **If ``/app/runLoops/main/rateLimitEnabled`` is already true**, writes
           ``/app/runLoops/main/rateLimitFrequency = 1/dt`` and calls
           ``timeline.set_target_framerate(1/dt)``. If rate-limit is not already enabled,
           this step is skipped and the app continues to run unthrottled - this method
           does not enable rate-limiting on its own.
        2. Writes ``stage.SetTimeCodesPerSecond(1/dt)`` to the root layer and
           ``timeline.set_time_codes_per_second(1/dt)``. This is the value the timeline uses
           as its per-tick ``dt`` whenever ``/app/player/useFixedTimeStepping`` is true (the
           default in the full Isaac Sim GUI).
        3. Calls ``loop_runner.set_manual_step_size(dt)`` and ``loop_runner.set_manual_mode(True)``
           on the Isaac loop runner, so the loop dispatches a fixed ``dt`` instead of the
           wall-clock measured value (relevant when ``useFixedTimeStepping`` is false, e.g.
           standalone Python). If the Isaac loop runner is not available, falls back to
           enabling the carb rate-limit (``rateLimitEnabled = True``) and writing
           ``rateLimitFrequency`` / ``timeline.set_target_framerate`` regardless of the
           prior ``rateLimitEnabled`` value.
        4. Writes the Fabric default simulation-period numerator/denominator carb settings
           from ``dt``. These are read by Fabric ``SimStageWithHistory`` instances created
           after this call; existing histories are not updated.

        This does **not** modify the physics scene's ``timeStepsPerSecond`` attribute - use
        :py:meth:`isaacsim.core.simulation_manager.SimulationManager.setup_simulation` for that.
        For real-time playback at a chosen rate, call both APIs with the same ``dt``.

        Args:
            dt: Application dt in seconds (e.g. ``1.0 / 60`` for 60 Hz).

        Raises:
            ValueError: If ``dt`` is not positive.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingManager
            >>>
            >>> RenderingManager.set_dt(1 / 120.0)  # 120 Hz
        """

        if dt <= 0:
            raise ValueError(f"Rendering dt must be positive, got {dt}")

        def _set_rate_limit_frequency(frequency: float) -> None:
            cls._carb_settings.set_bool(_SETTING_RATE_LIMIT_ENABLED, True)
            cls._carb_settings.set_float(_SETTING_RATE_LIMIT_FREQUENCY, frequency)
            cls._timeline.set_target_framerate(frequency)

        frequency = 1.0 / dt
        # Set the rate limit only if it is enabled.
        # Otherwise, run the app as fast as possible without specifying a target rate
        if cls._carb_settings.get_as_bool(_SETTING_RATE_LIMIT_ENABLED):
            _set_rate_limit_frequency(frequency)
        # Set the stage's `timeCodesPerSecond` value
        stage = stage_utils.get_current_stage(backend="usd")
        with Usd.EditContext(stage, stage.GetRootLayer()):
            stage.SetTimeCodesPerSecond(frequency)
        cls._timeline.set_time_codes_per_second(frequency)
        # Set Isaac Sim's loop runner-specific configuration. If it is not available, fall back to the app settings
        if hasattr(cls._loop_runner, "set_manual_step_size") and hasattr(cls._loop_runner, "set_manual_mode"):
            cls._loop_runner.set_manual_step_size(dt)
            cls._loop_runner.set_manual_mode(True)
        else:
            carb.log_warn(f"Isaac Sim's loop runner not found. Setting a rate limit instead ({frequency} Hz)")
            _set_rate_limit_frequency(frequency)
        # Kit reads these defaults when creating Fabric SimStageWithHistory instances
        # (for example graph/fabric caches). Existing histories expose getSimPeriod()
        # but no setter, so callers must set dt before creating the Fabric-backed stage
        # and render product that need this period.
        sim_period = Fraction(dt).limit_denominator(1_000_000_000)
        cls._carb_settings.set_int(_SETTING_FABRIC_DEFAULT_SIM_PERIOD_NUMERATOR, sim_period.numerator)
        cls._carb_settings.set_int(_SETTING_FABRIC_DEFAULT_SIM_PERIOD_DENOMINATOR, sim_period.denominator)

    @classmethod
    def get_dt(cls) -> float:
        """Get the application's currently-configured dt.

        Reads from one of two sources, in priority order:

        1. If ``/app/runLoops/main/rateLimitEnabled`` is true, returns
           ``1 / /app/runLoops/main/rateLimitFrequency``.
        2. Otherwise, if the Isaac loop runner is in manual mode, returns its
           ``manual_step_size``.
        3. As a final fallback, returns ``1 / /app/runLoops/main/rateLimitFrequency`` even
           though the rate-limit isn't enabled (the value may not actually be in effect).

        Note that this returns the loop / timeline dt, not the physics scene's ``physics_dt``;
        for that use :py:meth:`isaacsim.core.simulation_manager.PhysicsScene.get_dt`.

        Returns:
            Application dt in seconds.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingManager
            >>>
            >>> RenderingManager.get_dt()
            0.0166666...
        """

        def _get_rate_limit_dt() -> float:
            frequency = cls._carb_settings.get_as_float(_SETTING_RATE_LIMIT_FREQUENCY)
            return 1.0 / frequency if frequency else 0.0

        # Read from the app settings only if the rate limit is enabled in the first instance
        if cls._carb_settings.get_as_bool(_SETTING_RATE_LIMIT_ENABLED):
            return _get_rate_limit_dt()
        # Read from Isaac Sim's loop runner-specific configuration. If it is not available, fall back to the app settings
        if hasattr(cls._loop_runner, "get_manual_step_size") and hasattr(cls._loop_runner, "get_manual_mode"):
            if cls._loop_runner.get_manual_mode():
                return cls._loop_runner.get_manual_step_size()
        return _get_rate_limit_dt()

    @classmethod
    def register_callback(cls, event: RenderingEvent, *, callback: callable, order: int = 0) -> int:
        """Register/subscribe a callback to be triggered when a specific rendering event occurs.

        Args:
            event: The rendering event to subscribe to.
            callback: The callback function.
            order: The subscription order.
                Callbacks registered within the same order will be triggered in the order they were registered.

        Returns:
            The unique identifier of the callback subscription.

        Raises:
            ValueError: If the rendering event is not supported.
            RuntimeError: If unable to register the callback.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager
            >>>
            >>> def callback(event, *args, **kwargs):
            ...     print(event)
            ...
            >>> # subscribe to the NEW_FRAME event
            >>> callback_id = RenderingManager.register_callback(RenderingEvent.NEW_FRAME, callback=callback)
            >>> callback_id
            0
            >>> # perform a rendering step in order to trigger the callback and print the event
            >>> RenderingManager.render()  # doctest: +NO_CHECK
            <carb.eventdispatcher._eventdispatcher.Event object at 0x...>
            >>>
            >>> # deregister all callbacks
            >>> RenderingManager.deregister_all_callbacks()
        """
        if event not in RenderingEvent:
            raise ValueError(f"Invalid rendering event: {event}. Supported events are: {list(RenderingEvent)}")
        # check for weak reference support (when the callback is a method of a class)
        if hasattr(callback, "__self__"):
            on_event = lambda event, obj=weakref.proxy(callback.__self__): getattr(obj, callback.__name__)(event)
        else:
            on_event = callback
        # get a unique id for the callback
        uid = cls._callback_registry
        cls._callback_registry += 1
        # register the callback
        if event in [RenderingEvent.NEW_FRAME]:
            cls._callbacks[uid] = cls._event_dispatcher.observe_event(
                observer_name=f"isaacsim.core.rendering_manager:callback.{event.value}.{uid}",
                event_name=omni.usd.get_context().stage_rendering_event_name(
                    omni.usd.StageRenderingEventType.NEW_FRAME, True
                ),
                on_event=on_event,
                order=order,
            )
        else:
            raise RuntimeError(f"Unable to register callback for event '{event}' with uid '{uid}'")
        return uid

    @classmethod
    def deregister_callback(cls, uid: int) -> None:
        """Deregister a callback registered via :py:meth:`register_callback`.

        Args:
            uid: The unique identifier of the callback to deregister. If the unique identifier does not exist
                or has already been deregistered, a warning is logged and the method does nothing.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingManager
            >>>
            >>> # deregister the callback with the unique identifier 0
            >>> RenderingManager.deregister_callback(0)
        """
        if uid not in cls._callbacks:
            carb.log_warn(f"Unable to deregister callback with uid '{uid}'. It might have been already deregistered")
            return
        del cls._callbacks[uid]

    @classmethod
    def deregister_all_callbacks(cls) -> None:
        """Deregister all callbacks registered via :py:meth:`register_callback`.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.rendering_manager import RenderingManager
            >>>
            >>> RenderingManager.deregister_all_callbacks()
        """
        cls._callbacks.clear()
