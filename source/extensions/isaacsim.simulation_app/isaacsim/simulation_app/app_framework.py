# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides AppFramework class for creating minimal Omniverse applications without application configuration."""

import builtins
import os
import sys
import typing

import carb
import omni.kit.app


class AppFramework:
    """Minimal omniverse application that launches without any application config.

    Args:
        name: Name of the application.
        argv: Command line arguments to pass to the application.
    """

    def __init__(self, name: str = "kit", argv: list[str] | None = None) -> None:

        if argv is None:
            argv = []

        builtins.ISAAC_LAUNCHED_FROM_TERMINAL = False

        self._framework = carb.get_framework()
        self._framework.load_plugins(
            loaded_file_wildcards=["omni.kit.app.plugin"],
            search_paths=[os.path.abspath(f'{os.environ["CARB_APP_PATH"]}/kernel/plugins')],
        )
        self._app = omni.kit.app.get_app()
        # Path to where kit was built to
        app_root = os.environ["CARB_APP_PATH"]

        # first arg should be the file
        argv.insert(0, os.path.abspath(__file__))

        # allow root so apps don't fail when running in docker/root environments
        if sys.platform.startswith("linux") and os.geteuid() == 0 and "--allow-root" not in argv:
            argv.append("--allow-root")
        self._app.startup(name, app_root, argv)

    def update(self) -> None:
        """Convenience function to step the application forward one frame."""
        self._app.update()

    def close(self) -> None:
        """Close the running Omniverse Toolkit."""
        self._app.shutdown()
        self._framework.unload_all_plugins()

    @property
    def app(self) -> omni.kit.app.IApp:
        """Omniverse Kit application object.

        Returns:
            The omni.kit.app.IApp instance.
        """
        return self._app

    @property
    def framework(self) -> typing.Any:
        """Carb framework object.

        Returns:
            The Carb framework instance.
        """
        return self._framework
