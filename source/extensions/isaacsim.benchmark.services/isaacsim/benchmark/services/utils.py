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

"""Shared utility helpers for benchmark services."""

import functools
import inspect
import logging
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any
from urllib import parse as urlparse

import carb
import carb.eventdispatcher
import omni.kit.app

original_persistent_settings: dict[str, dict[str, Any]] = {}
settings_interface: Any | None = None

logger = logging.getLogger(__name__)


def set_up_logging(name: str) -> logging.Logger:
    """Set up a logger with consistent formatting.

    Args:
        name: Logger name.

    Returns:
        Configured logger instance.

    Example:

    .. code-block:: python

        logger = set_up_logging(__name__)
    """
    fmt = "{asctime} [{relativeCreated:,.0f}ms] [{levelname}] [{name}] {message}"
    datfmt = "%Y-%m-%d %H:%M:%S"
    formatter = logging.Formatter(fmt, datfmt, "{")
    formatter.converter = time.gmtime
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(formatter)
    logger = logging.getLogger(name)
    logger.handlers = [stdout_handler]
    logger.propagate = False
    logger.setLevel(logging.INFO)
    return logger


def set_persistent_setting(name: str, value: Any, type: type) -> None:
    """Set a persistent setting and remember the original value.

    Args:
        name: Setting path.
        value: Setting value.
        type: Setting type.

    Raises:
        RuntimeError: If settings_interface is not initialized.

    Example:

    .. code-block:: python

        set_persistent_setting("/app/window/title", "Benchmark", str)
    """
    global original_persistent_settings, settings_interface

    if settings_interface is None:
        raise RuntimeError("settings_interface is not initialized")

    _orig = settings_interface.get(name)  # noqa
    original_persistent_settings[name] = {"value": _orig, "type": type}

    _set_settings_value(name, value, type)


def restore_persistent_settings() -> None:
    """Restore all previously captured persistent settings.

    Example:

    .. code-block:: python

        restore_persistent_settings()
    """
    for name, _dict in original_persistent_settings.items():
        _set_settings_value(name, _dict["value"], _dict["type"])


def _set_settings_value(name: str, value: Any, type: type) -> None:
    """Set a settings value directly.

    Args:
        name: Setting path.
        value: Setting value.
        type: Setting type.

    Raises:
        RuntimeError: If settings_interface is not initialized.
    """
    global settings_interface
    if settings_interface is None:
        raise RuntimeError("settings_interface is not initialized")

    settings_interface.set(name, value)


@functools.lru_cache(maxsize=10)
def generate_event_map() -> dict[int, str]:
    """Build a mapping of stage event enum values to names.

    Returns:
        Mapping from event integer values to names.

    Example:

    .. code-block:: python

        event_map = generate_event_map()
    """
    event_map = {}
    for _val in dir(omni.usd.StageEventType):
        if _val.isupper():
            event_map[int(getattr(omni.usd.StageEventType, _val))] = _val
    return event_map


async def stage_event() -> int:
    """Await the next stage event and log it.

    Returns:
        Integer event value.

    Example:

    .. code-block:: python

        event = await stage_event()
    """
    result = await omni.usd.get_context().next_stage_event_async()
    event, _ = result
    event = int(event)
    logger.info(f"*** omni.kit.tests.basic_validation: stage_event() -> ({generate_event_map()[event]}, {_})")
    return event


async def capture_next_frame(app: Any, capture_file_path: str, timeout_sec: float = 2.0) -> None:
    """Capture the next frame to a file using viewport capture APIs.

    Args:
        app: Kit application instance.
        capture_file_path: Output image path.
        timeout_sec: Timeout in seconds.

    Raises:
        RuntimeError: If the viewport never produces valid resources.

    Example:

    .. code-block:: python

        await capture_next_frame(app, "/tmp/capture.png")
    """
    _renderer = None
    _viewport_interface = None

    try:
        import omni.kit.viewport_legacy
        import omni.renderer_capture
    except ImportError as ie:
        logger.error(f"*** screenshot: capture_next_frame: can't load {ie}")

    _renderer = omni.renderer_capture.acquire_renderer_capture_interface()
    _viewport_interface = omni.kit.viewport_legacy.acquire_viewport_interface()
    viewport_ldr_rp = _viewport_interface.get_viewport_window(None).get_drawable_ldr_resource()

    # Wait until the viewport has valid resources
    start_time = time.time()
    while viewport_ldr_rp is None and time.time() - start_time < timeout_sec:
        await app.next_update_async()
        viewport_ldr_rp = _viewport_interface.get_viewport_window(None).get_drawable_ldr_resource()

    if viewport_ldr_rp is None:
        raise RuntimeError(f"Timeout waiting for viewport to have valid resources after {timeout_sec} seconds.")

    _renderer.capture_next_frame_rp_resource(capture_file_path, viewport_ldr_rp)
    await app.next_update_async()
    _renderer.wait_async_capture()
    print("written", capture_file_path)


def omni_url_parser(url: str) -> tuple[str, str | None, str | None, str]:
    """Parse an Omni URL into connection components.

    Args:
        url: Omni URL.

    Returns:
        A tuple containing (netloc, username, password, path).

    Example:

    .. code-block:: python

        netloc, username, password, path = omni_url_parser("omniverse://server/asset.usd")
    """
    res = urlparse.urlparse(url)
    username = os.getenv("OMNI_USER")
    password = os.getenv("OMNI_PASS")
    return res.netloc, username, password, res.path


async def load_stage(stage_path: str, syncloads: bool, num_assets_loaded: int = 2) -> float:
    """Load a stage and wait for assets to finish loading.

    Args:
        stage_path: USD stage path.
        syncloads: True to wait for a single asset load event.
        num_assets_loaded: Number of asset load events to wait for.

    Returns:
        Stage load time in seconds.

    Raises:
        RuntimeError: If loading fails or assets load aborts.
        SystemExit: If the stage closes while waiting.

    Example:

    .. code-block:: python

        load_time = await load_stage("/path/to/stage.usd", syncloads=False, num_assets_loaded=3)
    """
    start = time.time()
    success, explanation = await omni.usd.get_context().open_stage_async(stage_path)
    logger.info(f"*** omni.kit.tests.basic_validation: Initial stage load success: {success}")
    if not success:
        raise RuntimeError(explanation)

    # we'll try to track all the ASSETS_LOADED events to figure out when the MDLs
    # are complete
    assets_loaded_count = 0
    required_assets_loaded = 1
    if not syncloads:
        required_assets_loaded = int(num_assets_loaded)

    if required_assets_loaded == 0:
        load_time = time.time() - start
        logger.info("*** omni.kit.tests.basic_validation: Not waiting for ASSETS LOADED at all, stage load complete.")
        return load_time

    logger.info(f"*** omni.kit.tests.basic_validation: Waiting for {required_assets_loaded} ASSETS LOADED event(s)")
    while True:
        event = await stage_event()
        # TODO: compare to actual enum value when Kit fixes its return types
        if event == int(omni.usd.StageEventType.ASSETS_LOADED):
            assets_loaded_count += 1
            logger.info(f"*** omni.kit.tests.basic_validation: Received ASSETS_LOADED #{assets_loaded_count}")
            # The user can specify how many assets_loaded to wait for in async mode
            if assets_loaded_count < required_assets_loaded:
                continue
            logger.info(
                f"*** omni.kit.tests.basic_validation: Met threshold of {required_assets_loaded}, all assets loaded"
            )
            break
        # error that something went wrong
        elif event == int(omni.usd.StageEventType.OPEN_FAILED):
            raise RuntimeError("Received OPEN_FAILED")
        elif event == int(omni.usd.StageEventType.ASSETS_LOAD_ABORTED):
            raise RuntimeError("Received ASSETS_LOAD_ABORTED")
        elif event == int(omni.usd.StageEventType.CLOSING):
            raise SystemExit("Received CLOSING")
        elif event == int(omni.usd.StageEventType.CLOSED):
            raise SystemExit("Received CLOSED")

    load_time = time.time() - start
    return load_time


def getStageDefaultPrimPath(stage: Any) -> Any:  # noqa: N802
    """Get the default prim path for a stage.

    Args:
        stage: USD stage.

    Returns:
        Default prim path.

    Example:

    .. code-block:: python

        path = getStageDefaultPrimPath(stage)
    """
    if stage.HasDefaultPrim():
        return stage.GetDefaultPrim().GetPath()
    else:
        from pxr import Sdf

        return Sdf.Path.absoluteRootPath


class LogErrorChecker:
    """Monitor log events and count errors during a test."""

    def __init__(self) -> None:
        # Setup this test case to fail if any error is produced
        self._error_count = 0

        def on_log_event(e: Any) -> None:
            if e["level"] >= carb.logging.LEVEL_ERROR:
                self._error_count = self._error_count + 1

        self._log_stream = omni.kit.app.get_app().get_log_event_stream()
        self._log_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_LOG,
            on_event=on_log_event,
            observer_name="isaacsim.benchmark.services.utils.on_log_event",
        )

    def shutdown(self) -> None:
        """Unsubscribe from log events.

        Example:

        .. code-block:: python

            checker.shutdown()
        """
        self._log_stream = None
        self._log_sub = None

    def get_error_count(self) -> int:
        """Get the current error count.

        Returns:
            Number of logged errors.

        Example:

        .. code-block:: python

            count = checker.get_error_count()
        """
        self._log_stream.pump()
        return self._error_count


def get_calling_test_id() -> str:
    """Return the fully qualified name of a calling test, if any.

    Returns:
        Fully qualified test name, or an empty string if not found.

    Example:

    .. code-block:: python

        test_id = get_calling_test_id()
    """

    def get_class_from_frame(fr: Any) -> type | None:
        args, _, _, value_dict = inspect.getargvalues(fr)
        # we check the first parameter for the frame function is
        # named 'self'
        if len(args) and args[0] == "self":
            # in that case, 'self' will be referenced in value_dict
            instance = value_dict.get("self", None)
            if instance:
                # return its class
                class_name = getattr(instance, "__class__", None)
                # print("get_class_from_frame, returning", class_name)
                return class_name
        # return None otherwise
        # print("get_class_from_frame, returning none")
        return None

    for frame, _ in traceback.walk_stack(None):
        func_name = frame.f_code.co_name
        if func_name.startswith("test_"):
            cls = get_class_from_frame(frame)
            if cls:
                name = f"{cls.__module__}.{cls.__name__}.{func_name}"
                return name
            break
    return ""


def ensure_dir(file_path: str | Path) -> None:
    """Create a directory if it does not exist.

    Args:
        file_path: Directory path.

    Example:

    .. code-block:: python

        ensure_dir("/tmp/metrics")
    """
    if not os.path.exists(file_path):
        logger.info(f"Creating dir {file_path}")
        os.makedirs(file_path)


def get_kit_version_branch() -> tuple[str, str, str]:
    """Get Kit version, branch, and combined version_branch string.

    Returns:
        A tuple containing (version, branch, version_branch).

    Example:

    .. code-block:: python

        version, branch, version_branch = get_kit_version_branch()
    """
    app = omni.kit.app.get_app()
    build_version = app.get_build_version()
    version = build_version.split("+")[0]
    branch = build_version.split("+")[1].split(".")[0]
    version_branch = version + "_" + branch
    return version, branch, version_branch


# Run a given number of app updates after loading a stage to fully loaded materials/textures and co.
# early stop if a frame time threshold (frametime_threshold) is reached
# or if the time ratio (time_ratio_treshold) between the current and the previous frame is reached
# e.g. current frame needed X times less time than the previous one
async def wait_until_stage_is_fully_loaded_async(
    max_frames: int = 10, frametime_threshold: float = 0.1, time_ratio_treshold: float = 5
) -> None:
    """Wait for stage to fully load by observing frame times.

    Args:
        max_frames: Maximum frames to wait.
        frametime_threshold: Frametime threshold to consider fully loaded.
        time_ratio_treshold: Ratio threshold between frames.

    Example:

    .. code-block:: python

        await wait_until_stage_is_fully_loaded_async()
    """
    prev_frametime = 0.0
    for i in range(max_frames):
        start_time = time.time()
        await omni.kit.app.get_app().next_update_async()
        elapsed_time = time.time() - start_time
        logger.info(f"Frame {i} frametime: {elapsed_time}")
        if elapsed_time < frametime_threshold or elapsed_time * time_ratio_treshold < prev_frametime:
            logger.info(f"Stage fully loaded at frame {i}, last frametime: {elapsed_time}")
            break
        prev_frametime = elapsed_time


# Run a given number of app updates after loading a stage to fully loaded materials/textures and co.
# early stop if a frame time threshold (frametime_threshold) is reached
# or if the time ratio (time_ratio_treshold) between the current and the previous frame is reached
# e.g. current frame needed X times less time than the previous one
def wait_until_stage_is_fully_loaded(
    max_frames: int = 10, frametime_threshold: float = 0.1, time_ratio_treshold: float = 5
) -> None:
    """Wait for stage to fully load by observing frame times.

    Args:
        max_frames: Maximum frames to wait.
        frametime_threshold: Frametime threshold to consider fully loaded.
        time_ratio_treshold: Ratio threshold between frames.

    Example:

    .. code-block:: python

        wait_until_stage_is_fully_loaded()
    """
    prev_frametime = 0.0
    for i in range(max_frames):
        start_time = time.time()
        omni.kit.app.get_app().update()
        elapsed_time = time.time() - start_time
        logger.info(f"Frame {i} frametime: {elapsed_time}")
        if elapsed_time < frametime_threshold or elapsed_time * time_ratio_treshold < prev_frametime:
            logger.info(f"Stage fully loaded at frame {i}, last frametime: {elapsed_time}")
            break
        prev_frametime = elapsed_time
