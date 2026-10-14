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

_DLL_DIRECTORY_HANDLES = []

# Keep the staged cuMotion runtime directories available while `_cumotion.pyd` and its dependencies are loaded.
if os.name == "nt":
    prebundle_directory = pathlib.Path(__file__).resolve().parents[3]
    dll_directories = [prebundle_directory]
    dll_directories.extend(path for path in prebundle_directory.glob("_cumotion*") if path.is_dir())
    _DLL_DIRECTORY_HANDLES.extend(os.add_dll_directory(str(path)) for path in dll_directories)


class Extension(omni.ext.IExt):
    """Extension for cuMotion robot motion planning integration.

    This extension enables cuMotion robot motion planning capabilities within Isaac Sim. It handles the setup
    and integration of the cuMotion library, providing robot motion planning functionality for robotics
    applications. On Windows systems, it automatically configures DLL loading paths to ensure proper
    functioning of the cuMotion Python extension module and its dependencies.
    """
