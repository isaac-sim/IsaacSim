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

"""Demonstrate replicator behavior scripts for randomizing prims in Isaac Sim."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import inspect
import os
from typing import Any

import carb.settings
import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core.experimental.utils.app import get_extension_path as get_extension_path_from_name
from isaacsim.replicator.behavior.behaviors import (
    LightRandomizer,
    LocationRandomizer,
    LookAtBehavior,
    RotationRandomizer,
    TextureRandomizer,
)
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS
from isaacsim.replicator.behavior.utils.behavior_utils import add_behavior_script
from pxr import Sdf

SCRIPTS_ATTR = "omni:scripting:scripts"

# Get the path to the behavior scripts python files
EXTENSION_PATH = get_extension_path_from_name("isaacsim.replicator.behavior")
SCRIPTS_PATH = os.path.join(EXTENSION_PATH, "isaacsim/replicator/behavior/behaviors")

# Enable scripting extension in standalone mode
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.behavior.scripting.core", True)
simulation_app.update()

# Enable behavior scripts (running python scripts attached to USD assets)
carb.settings.get_settings().set_bool("/app/scripting/ignoreWarningDialog", True)
simulation_app.update()


def setup_stage() -> None:
    """Set up a new stage with a dome light."""
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)


def add_behavior_script_with_parameters(
    prim_path: str, behavior_class: type, exposed_variables: dict[str, Any] | None = None
) -> None:
    """Add a behavior script to a prim and set exposed variable values."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        raise RuntimeError(f"No prim found at path: {prim_path}")

    # Get the script path from the behavior class
    script_path = inspect.getfile(behavior_class)

    # Add the behavior script to the prim
    add_behavior_script(prim, script_path)

    # NOTE: 2-3 updates are needed to ensure the script is loaded and the exposed variables are set
    for _ in range(3):
        simulation_app.update()

    # Append the exposed variables with the corresponding namespace and set them as properties on the prim
    variable_ns = f"{EXPOSED_ATTR_NS}:{behavior_class.BEHAVIOR_NS}"
    for var_name, var_value in (exposed_variables or {}).items():
        full_var_name = f"{variable_ns}:{var_name}"
        exposed_var_attr = prim.GetAttribute(full_var_name)
        if not exposed_var_attr:
            raise RuntimeError(f"No exposed variable attribute {full_var_name} found on prim: {prim_path}")
        exposed_var_attr.Set(var_value)


def remove_all_scripts(prim_paths: list[str]) -> None:
    """Remove all behavior scripts from the given prim paths."""
    stage = omni.usd.get_context().get_stage()
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            raise RuntimeError(f"No prim found at path: {prim_path}")
        scripts_attr = prim.GetAttribute(SCRIPTS_ATTR)
        if not scripts_attr:
            raise RuntimeError(f"No '{SCRIPTS_ATTR}' attribute found on prim: {prim_path}")
        scripts_attr.Set(Sdf.AssetPathArray())


def create_prims_single(prim_path: str, prim_type: str) -> None:
    """Create a single prim at the given path for randomization."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, prim_type)
    if not prim.IsValid():
        raise RuntimeError(f"Failed to create prim of type {prim_type} at {prim_path}")


def create_prims_multi(
    root_path: str, num_prims: int = 1, prim_type: str = "SphereLight", prim_name: str = "light"
) -> None:
    """Create multiple child prims under a root path for randomization."""
    stage = omni.usd.get_context().get_stage()

    for i in range(num_prims):
        prim_path = f"{root_path}/{prim_name}_{i}" if num_prims > 1 else root_path
        prim = stage.DefinePrim(prim_path, prim_type)

        if not prim.IsValid():
            raise RuntimeError(f"Failed to create prim of type {prim_type} at {prim_path}")


# Create a new stage
setup_stage()


# Create a light with behavior scripts
light_path = "/Single/Light"
create_prims_single(light_path, "SphereLight")
add_behavior_script_with_parameters(light_path, LightRandomizer)
add_behavior_script_with_parameters(light_path, LocationRandomizer)
add_behavior_script_with_parameters(light_path, RotationRandomizer)

# Create a cube with behavior scripts
cube_path = "/Single/Cube"
create_prims_single(cube_path, "Cube")
add_behavior_script_with_parameters(cube_path, LocationRandomizer)
add_behavior_script_with_parameters(cube_path, RotationRandomizer)
add_behavior_script_with_parameters(cube_path, TextureRandomizer)

# Create a camera with behavior scripts
camera_path = "/Single/Camera"
create_prims_single(camera_path, "Camera")
add_behavior_script_with_parameters(camera_path, LookAtBehavior)
add_behavior_script_with_parameters(camera_path, LocationRandomizer)


# Create lights nested under a root prim and add behavior scripts to the root prim with includeChildren flag
nested_lights_path = "/Nested/Lights"
create_prims_multi(root_path=nested_lights_path, num_prims=3, prim_type="SphereLight", prim_name="light")
add_behavior_script_with_parameters(nested_lights_path, LightRandomizer, exposed_variables={f"includeChildren": True})
add_behavior_script_with_parameters(
    nested_lights_path, LocationRandomizer, exposed_variables={f"includeChildren": True}
)
add_behavior_script_with_parameters(
    nested_lights_path, RotationRandomizer, exposed_variables={f"includeChildren": True}
)

# Create cubes nested under a root prim and add behavior scripts to the root prim with includeChildren flag
nested_cubes_path = "/Nested/Cubes"
create_prims_multi(root_path=nested_cubes_path, num_prims=3, prim_type="Cube", prim_name="cube")
add_behavior_script_with_parameters(nested_cubes_path, LocationRandomizer, exposed_variables={f"includeChildren": True})
add_behavior_script_with_parameters(nested_cubes_path, RotationRandomizer, exposed_variables={f"includeChildren": True})
add_behavior_script_with_parameters(nested_cubes_path, TextureRandomizer, exposed_variables={f"includeChildren": True})

# Create cameras nested under a root prim and add behavior scripts to the root prim with includeChildren flag
nested_cameras_path = "/Nested/Cameras"
create_prims_multi(root_path=nested_cameras_path, num_prims=3, prim_type="Camera", prim_name="camera")
add_behavior_script_with_parameters(nested_cameras_path, LookAtBehavior, exposed_variables={f"includeChildren": True})
add_behavior_script_with_parameters(
    nested_cameras_path, LocationRandomizer, exposed_variables={f"includeChildren": True}
)


# Run the randomization scripts through the timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(100):
    simulation_app.update()
timeline.stop()
for _ in range(3):
    simulation_app.update()


# Remove the scripts from the prims
prim_paths = [light_path, cube_path, camera_path, nested_lights_path, nested_cubes_path, nested_cameras_path]
remove_all_scripts(prim_paths=prim_paths)
for _ in range(3):
    simulation_app.update()

# Close the simulation app
simulation_app.close()
