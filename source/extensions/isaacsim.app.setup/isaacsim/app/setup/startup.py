# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Startup task utilities for Isaac Sim application.

This module provides async functions for handling application startup tasks
including viewport initialization, ROS bridge enabling, and benchmark recording.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from typing import TYPE_CHECKING

import carb
import omni.kit.app
import omni.kit.stage_templates as stage_templates
import omni.usd

if TYPE_CHECKING:
    from carb.settings import ISettings
    from omni.kit.app import IApp, IExtensionManager


async def create_new_stage(update_callback: Callable[[], Awaitable[None]]) -> None:
    """Create a new empty stage on startup.

    Args:
        update_callback: Async callback to update the app without signaling ready.

    Example:

        .. code-block:: python

            async def update_callback() -> None:
                await omni.kit.app.get_app().next_update_async()

            await create_new_stage(update_callback)
    """
    await update_callback()
    if omni.usd.get_context().can_open_stage():
        stage_templates.new_stage(template=None)
    await update_callback()


async def await_viewport(
    app: IApp,
    ext_manager: IExtensionManager,
    app_title: str,
    update_callback: Callable[[], Awaitable[None]],
) -> None:
    """Wait for viewport to be ready and log application startup completion.

    Monitors the viewport for initialization and triggers startup benchmarking
    when the application is ready for use.

    Args:
        app: The Kit application instance.
        ext_manager: Extension manager for checking enabled extensions.
        app_title: Application title for logging.
        update_callback: Async callback to update the app without signaling ready.

    Example:

        .. code-block:: python

            app = omni.kit.app.get_app()
            ext_manager = app.get_extension_manager()
            app_title = app.get_app_name()

            async def update_callback() -> None:
                await app.next_update_async()

            await await_viewport(app, ext_manager, app_title, update_callback)
    """
    import carb.eventdispatcher
    from omni.kit.viewport.utility import get_active_viewport
    from omni.usd import StageRenderingEventType

    viewport_api = get_active_viewport()

    # Wait for viewport handle if not already available
    if viewport_api.frame_info.get("viewport_handle", None) is None:
        carb.log_info("await_viewport: viewport handle not yet available, waiting for first frame...")
        future: asyncio.Future = asyncio.Future()

        def on_frame_event(e: carb.eventdispatcher.Event) -> None:
            vp_handle = viewport_api.frame_info.get("viewport_handle", None)
            if vp_handle is not None and not future.done():
                carb.log_info(f"await_viewport: viewport handle acquired ({vp_handle})")
                future.set_result(None)

        usd_context = omni.usd.get_context()
        event_name = usd_context.stage_rendering_event_name(StageRenderingEventType.NEW_FRAME, True)
        sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=event_name,
            on_event=on_frame_event,
            observer_name="isaacsim.app.setup.wait_for_viewport",
        )
        frame_count = 0
        while not future.done():
            frame_count += 1
            carb.log_info(f"await_viewport: waiting for viewport handle... frame {frame_count}")
            await update_callback()
        sub = None

    app.print_and_log(f"{app_title} App is loaded.")

    record_startup_benchmark(ext_manager)
    await update_callback()


def record_startup_benchmark(ext_manager: IExtensionManager) -> None:
    """Record startup time as a benchmark metric if benchmarking is enabled.

    Args:
        ext_manager: Extension manager for checking if benchmarking extension is enabled.

    Example:

        .. code-block:: python

            app = omni.kit.app.get_app()
            record_startup_benchmark(app.get_extension_manager())
    """
    if ext_manager.is_extension_enabled("isaacsim.benchmark.services"):
        from isaacsim.benchmark.services import BaseIsaacBenchmark

        benchmark = BaseIsaacBenchmark(
            benchmark_name="app_startup",
            workflow_metadata={
                "metadata": [
                    {"name": "mode", "data": "async"},
                ]
            },
        )
        benchmark.set_phase("startup", start_recording_frametime=False, start_recording_runtime=False)
        benchmark.store_measurements()
        benchmark.stop()


async def enable_ros_extensions(
    settings: ISettings,
    ext_manager: IExtensionManager,
    update_callback: Callable[[], Awaitable[None]],
) -> None:
    """Enable ROS2 bridge and sim control extensions if configured in settings.

    Waits for viewport to be ready before enabling ROS2 extensions,
    as they require certain systems to be fully initialized.

    Args:
        settings: Carb settings interface for reading configuration.
        ext_manager: Extension manager for enabling extensions.
        update_callback: Async callback to update the app without signaling ready.

    Example:

        .. code-block:: python

            settings = carb.settings.get_settings()
            app = omni.kit.app.get_app()

            async def update_callback() -> None:
                await app.next_update_async()

            await enable_ros_extensions(settings, app.get_extension_manager(), update_callback)
    """
    import os

    try:
        ros_bridge_name = settings.get("isaac/startup/ros_bridge_extension")
        ros_sim_control_enabled = settings.get("isaac/startup/ros_sim_control_extension")

        # Nothing to do if neither ROS2 extension is configured
        if not ros_bridge_name and not ros_sim_control_enabled:
            return

        # Log the ROS environment status for debugging
        ros_distro = os.environ.get("ROS_DISTRO", "<not set>")
        carb.log_info(f"Enabling ROS2 extensions (ROS_DISTRO={ros_distro})")

        # Wait for viewport to be available before loading ROS2 extensions.
        # ROS2 extensions depend on systems that may not be initialized during early startup.
        # We can't wait for app.is_app_ready() because _update_without_ready delays it.
        from omni.kit.viewport.utility import get_active_viewport

        viewport_api = get_active_viewport()
        while viewport_api.frame_info.get("viewport_handle", None) is None:
            await update_callback()

        # Additional update frames to ensure all systems are stable
        for _ in range(5):
            await update_callback()

        # Enable ROS2 bridge if configured
        if ros_bridge_name:
            ext_manager.set_extension_enabled_immediate(ros_bridge_name, True)
            for _ in range(3):
                await update_callback()

            if ext_manager.is_extension_enabled(ros_bridge_name):
                carb.log_info(f"ROS bridge extension {ros_bridge_name} enabled successfully")
            else:
                carb.log_warn(f"ROS bridge extension {ros_bridge_name} may not have loaded correctly")

        # Enable ROS2 sim control if configured
        if ros_sim_control_enabled:
            ext_manager.set_extension_enabled_immediate("isaacsim.ros2.sim_control", True)
            for _ in range(3):
                await update_callback()
            carb.log_info("ROS2 sim control extension enabled")

    except asyncio.CancelledError:
        carb.log_warn("isaacsim.app.setup shutdown before ROS2 extensions enabled")
    except Exception as e:
        carb.log_error(f"Failed to enable ROS2 extensions: {type(e).__name__}: {e}")
