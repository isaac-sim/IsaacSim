# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Cortex-specific utilities and helper methods."""

from types import ModuleType

from isaacsim.storage.native import get_assets_root_path


def load_behavior_module(behavior_filepath: str, module_name: str = "behavior") -> ModuleType:
    """Load and return a behavior module.

    A behavior module is a python module with a make_decider_network() method. If that method is not
    found, a RuntimeError is raised.

    Args:
        behavior_filepath: The path to the module file to be loaded.
        module_name: An optional name of the module to load. Defaults to "behavior".

    Returns:
        The loaded module. One can call module.make_decider_network(robot) to create and return the
        decider network representing the behavior.

    Raises:
        RuntimeError: If the module doesn't have a make_decider_network() method.
    """
    from importlib.machinery import SourceFileLoader

    module = SourceFileLoader(module_name, behavior_filepath).load_module()

    if not hasattr(module, "make_decider_network"):
        raise RuntimeError(f"Module at path {behavior_filepath} does not have a make_decider_network() method.")

    return module


def get_assets_root_path_or_die() -> str:
    """Find the assets root path and check for errors.

    Returns:
        The root path.

    Raises:
        RuntimeError: If the assets folder could not be found.
    """
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        err_str = "Could not find Isaac Sim assets folder"
        carb.log_error(err_str)
        raise RuntimeError(err_str)
    return assets_root_path
