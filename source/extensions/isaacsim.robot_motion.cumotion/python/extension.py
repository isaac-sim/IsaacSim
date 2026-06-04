# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


"""Extension for cuMotion robot motion planning integration within Isaac Sim."""

import os
import pathlib

import omni.ext

# Work around a (not understood) issue on Windows where the cumotion python extension module (pyd file)
# is loaded properly but the DLLs on which it depends are not, despite being in the same directory.
if os.name == "nt":
    file_dir = pathlib.Path(os.path.dirname(os.path.abspath(__file__)))
    cumotion_dir = file_dir.joinpath(pathlib.Path("../../../pip_prebundle")).resolve()
    os.add_dll_directory(cumotion_dir.__str__())


class Extension(omni.ext.IExt):
    """Extension for cuMotion robot motion planning integration.

    This extension enables cuMotion robot motion planning capabilities within Isaac Sim. It handles the setup
    and integration of the cuMotion library, providing robot motion planning functionality for robotics
    applications. On Windows systems, it automatically configures DLL loading paths to ensure proper
    functioning of the cuMotion Python extension module and its dependencies.
    """

    def on_startup(self, ext_id) -> None:
        """Startup the extension.

        Args:
            ext_id: The extension ID.
        """

    def on_shutdown(self) -> None:
        """Shutdown the extension."""
