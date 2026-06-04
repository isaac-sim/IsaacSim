# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Functions for interacting with the application and its available extensions."""

from __future__ import annotations

from collections.abc import Callable

import omni.kit.app
import omni.timeline


def update_app(*, steps: int = 1, callback: Callable[[int, int], bool | None] | None = None) -> None:
    """Perform one or more update steps of the application.

    Args:
        steps: Number of update steps to perform.
        callback: Optional callback function to call after each update step.
            The function should take two arguments: the current step number and the total number of steps.
            If no return value is provided, the update loop will run for the specified number of steps.
            However, if the function returns ``False``, no more update steps will be performed.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # perform one update step
        >>> app_utils.update_app()  # doctest: +SKIP
        >>>
        >>> # perform 10 update steps
        >>> app_utils.update_app(steps=10)  # doctest: +SKIP
        >>>
        >>> # perform 10 update steps with a callback
        >>> def callback(step, steps):
        ...     print(f"update step {step}/{steps}")
        ...     return step < 3  # stop after 3 steps (return False to break the loop)
        ...
        >>> app_utils.update_app(steps=10, callback=callback)  # doctest: +SKIP
        update step 1/10
        update step 2/10
        update step 3/10
    """
    for step in range(steps):
        omni.kit.app.get_app().update()
        if callback is not None:
            if callback(step + 1, steps) is False:
                break


async def update_app_async(*, steps: int = 1, callback: Callable[[int, int], bool | None] | None = None) -> None:
    """Perform one or more update steps of the application.

    This function is the asynchronous version of :py:func:`update_app`.

    Args:
        steps: Number of update steps to perform.
        callback: Optional callback function to call after each update step.
            The function should take two arguments: the current step number and the total number of steps.
            If no return value is provided, the update loop will run for the specified number of steps.
            However, if the function returns ``False``, no more update steps will be performed.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> from omni.kit.async_engine import run_coroutine
        >>>
        >>> async def task():
        ...     await app_utils.update_app_async()
        ...
        >>> run_coroutine(task())  # doctest: +NO_CHECK
    """
    for step in range(steps):
        await omni.kit.app.get_app().next_update_async()
        if callback is not None:
            if callback(step + 1, steps) is False:
                break


def play(*, commit: bool | None = True) -> None:
    """Play the application timeline.

    .. note::

        After committing the timeline state (silently or not), the new value will be available immediately
        when querying it (e.g.: via :py:func:`is_playing`). Otherwise, one app update step is required to reflect
        the new state and trigger any registered callback.

    Args:
        commit: Whether to commit the "play" state. The following values are supported:

            - ``True``: Commit the state and trigger callbacks.
            - ``False``: Do not commit the state.
            - ``None``: Commit the state silently (without triggering callbacks).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.play()
    """
    omni.timeline.get_timeline_interface().play()
    if commit is None:
        omni.timeline.get_timeline_interface().commit_silently()
    elif commit:
        omni.timeline.get_timeline_interface().commit()


def pause(*, commit: bool | None = True) -> None:
    """Pause the application timeline.

    .. note::

        After committing the timeline state (silently or not), the new value will be available immediately
        when querying it (e.g.: via :py:func:`is_paused`). Otherwise, one app update step is required to reflect
        the new state and trigger any registered callback.

    Args:
        commit: Whether to commit the "pause" state. The following values are supported:

            - ``True``: Commit the state and trigger callbacks.
            - ``False``: Do not commit the state.
            - ``None``: Commit the state silently (without triggering callbacks).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.pause()
    """
    omni.timeline.get_timeline_interface().pause()
    if commit is None:
        omni.timeline.get_timeline_interface().commit_silently()
    elif commit:
        omni.timeline.get_timeline_interface().commit()


def stop(*, commit: bool | None = True) -> None:
    """Stop the application timeline.

    .. note::

        After committing the timeline state (silently or not), the new value will be available immediately
        when querying it (e.g.: via :py:func:`is_stopped`). Otherwise, one app update step is required to reflect
        the new state and trigger any registered callback.

    Args:
        commit: Whether to commit the "stop" state. The following values are supported:

            - ``True``: Commit the state and trigger callbacks.
            - ``False``: Do not commit the state.
            - ``None``: Commit the state silently (without triggering callbacks).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.stop()
    """
    omni.timeline.get_timeline_interface().stop()
    if commit is None:
        omni.timeline.get_timeline_interface().commit_silently()
    elif commit:
        omni.timeline.get_timeline_interface().commit()


def is_playing() -> bool:
    """Check if the application timeline is playing.

    Returns:
        Whether the application timeline is playing.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.is_playing()
        False
    """
    return omni.timeline.get_timeline_interface().is_playing()


def is_paused() -> bool:
    """Check if the application timeline is paused.

    Returns:
        Whether the application timeline is paused.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.is_paused()
        False
    """
    timeline = omni.timeline.get_timeline_interface()
    return not (timeline.is_playing() or timeline.is_stopped())


def is_stopped() -> bool:
    """Check if the application timeline is stopped.

    Returns:
        Whether the application timeline is stopped.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> app_utils.is_stopped()
        True
    """
    return omni.timeline.get_timeline_interface().is_stopped()


def enable_extension(name: str, *, enabled: bool = True) -> bool:
    """Enable/disable an extension from the extension manager.

    Args:
        name: Name of the extension.
        enabled: Whether the extension should be enabled.

    Returns:
        Whether the extension was enabled/disabled successfully.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # enable the 'omni.pip.cloud' extension
        >>> app_utils.enable_extension("omni.pip.cloud")
        True
        >>>
        >>> # disable the 'omni.pip.cloud' extension
        >>> app_utils.enable_extension("omni.pip.cloud", enabled=False)
        True
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(name, enabled)


def is_extension_enabled(name: str) -> bool:
    """Check if an extension is enabled.

    Args:
        name: Name of the extension.

    Returns:
        Whether the extension is enabled.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # check if the 'omni.pip.cloud' extension is enabled
        >>> app_utils.enable_extension("omni.pip.cloud", enabled=False)  # doctest: +NO_CHECK
        >>> app_utils.is_extension_enabled("omni.pip.cloud")
        False
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.is_extension_enabled(name)


def get_extension_path(name: str) -> str:
    """Get the path of an extension.

    Args:
        name: Name/ID of the extension.

    Returns:
        Path of the extension root directory.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # get the path of the 'omni.pip.cloud' extension
        >>> app_utils.enable_extension("omni.pip.cloud")  # doctest: +NO_CHECK
        >>> app_utils.get_extension_path("omni.pip.cloud")  # doctest: +NO_CHECK
        '.../exts/omni.pip.cloud'
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(extension_manager.get_enabled_extension_id(name))


def get_extension_id(name: str) -> str | None:
    """Get the ID of an extension.

    Args:
        name: Name/ID of the extension.

    Returns:
        Extension ID (name-version) or ``None`` if the extension is not enabled/found.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # get the id of the 'omni.pip.cloud' extension
        >>> app_utils.enable_extension("omni.pip.cloud")  # doctest: +NO_CHECK
        >>> app_utils.get_extension_id("omni.pip.cloud")  # doctest: +NO_CHECK
        'omni.pip.cloud-4.2.0'
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_enabled_extension_id(name)


def get_extension_dict(name: str) -> dict | None:
    """Get the extension configuration (``extension.toml`` file) as a Python dictionary.

    Args:
        name: Name/ID of the extension.

    Returns:
        Configuration dictionary, or ``None`` if the extension is not enabled/found.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>>
        >>> # get the configuration dictionary of the 'omni.pip.cloud' extension
        >>> app_utils.enable_extension("omni.pip.cloud")  # doctest: +NO_CHECK
        >>> app_utils.get_extension_dict("omni.pip.cloud")  # doctest: +NO_CHECK
        {
            'name': 'omni.pip.cloud',
            'package': {
                'version': '4.2.0',
                'description': "...",
            },
            ...
        }
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    carb_dict = extension_manager.get_extension_dict(extension_manager.get_enabled_extension_id(name))
    return carb_dict.get_dict() if carb_dict else None
