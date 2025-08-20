# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import random
import string

import carb
import omni.kit.window.property
import omni.usd
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS, SCOPE_NAME
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_empty_scopes,
    remove_exposed_variables,
)
from isaacsim.replicator.behavior.utils.scene_utils import create_mdl_material
from isaacsim.storage.native import get_assets_root_path
from omni.kit.scripting import BehaviorScript
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


class TextureRandomizer(BehaviorScript):
    """
    Behavior script that creates texture materials from the given list and randomly applies them to visual prim(s).
    """

    BEHAVIOR_NS = "textureRandomizer"
    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "includeChildren",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Include valid prim children to the behavior.",
        },
        {
            "attr_name": "interval",
            "attr_type": Sdf.ValueTypeNames.UInt,
            "default_value": 0,
            "doc": "Interval for updating the behavior. Value 0 means every frame.",
        },
        {
            "attr_name": "textures:assets",
            "attr_type": Sdf.ValueTypeNames.AssetArray,
            "default_value": [],
            "doc": "Asset list of textures to randomize.",
        },
        {
            "attr_name": "textures:csv",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": (
                "/Isaac/Materials/Textures/Patterns/nv_bamboo_desktop.jpg,"
                "/Isaac/Materials/Textures/Patterns/nv_brick_grey.jpg"
            ),
            "doc": "CSV list of texture URLs to randomize.",
        },
        {
            "attr_name": "projectUvwProbability",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 0.9,
            "doc": "Probability that 'project_uvw' is set to True.",
        },
        {
            "attr_name": "textureScaleRange",
            "attr_type": Sdf.ValueTypeNames.Float2,
            "default_value": Gf.Vec2f(0.1, 1.0),
            "doc": "Texture scale range as (min, max).",
        },
        {
            "attr_name": "textureRotateRange",
            "attr_type": Sdf.ValueTypeNames.Float2,
            "default_value": Gf.Vec2f(0.0, 45.0),
            "doc": "Texture rotation range in degrees as (min, max).",
        },
    ]

    def on_init(self):
        """Called when the script is assigned to a prim."""
        self._update_counter = 0
        self._interval = 0
        self._texture_urls = []
        self._valid_prims = []
        self._initial_materials = {}
        self._texture_materials = []

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

        # Refresh the property windows to show the exposed variables
        omni.kit.window.property.get_window().request_rebuild()

    def on_destroy(self):
        """Called when the script is unassigned from a prim."""
        self._reset()
        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)
            omni.kit.window.property.get_window().request_rebuild()

    def on_play(self):
        """Called when `play` is pressed."""
        self._setup()
        # Make sure the initial behavior is applied if the interval is larger than 0
        if self._interval > 0:
            self._apply_behavior()

    def on_stop(self):
        """Called when `stop` is pressed."""
        self._reset()

    def on_update(self, current_time: float, delta_time: float):
        """Called on per frame update events that occur when `playing`."""
        if delta_time <= 0:
            return
        if self._interval <= 0:
            self._apply_behavior()
        else:
            self._update_counter += 1
            if self._update_counter >= self._interval:
                self._apply_behavior()
                self._update_counter = 0

    def _setup(self):
        # Fetch the exposed attributes
        include_children = self._get_exposed_variable("includeChildren")
        self._interval = self._get_exposed_variable("interval")
        textures_assets = self._get_exposed_variable("textures:assets")
        textures_csv = self._get_exposed_variable("textures:csv")
        self._project_uvw_probability = self._get_exposed_variable("projectUvwProbability")
        self._texture_scale_range = self._get_exposed_variable("textureScaleRange")
        self._texture_rotate_range = self._get_exposed_variable("textureRotateRange")

        # Get the valid prims
        if include_children:
            self._valid_prims = [prim for prim in Usd.PrimRange(self.prim) if prim.IsA(UsdGeom.Gprim)]
        elif self.prim.IsA(UsdGeom.Gprim):
            self._valid_prims = [self.prim]
        else:
            self._valid_prims = []
            carb.log_warn(f"[{self.prim_path}] No valid prims found.")

        # Cache original materials to restore after the randomization has stopped
        for prim in self._valid_prims:
            orig_mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            self._initial_materials[prim] = orig_mat

        # Create materials to randomize for each valid prim
        self._create_materials()

        # Store the texture urls
        self._texture_urls = []

        # Add the texture paths from the assets list
        for texture_asset in textures_assets:
            self._texture_urls.append(texture_asset.path)

        # Use the assets root path if the texture URLs are relative
        assets_root_path = get_assets_root_path()

        # Add the texture paths from the csv string
        for texture_url in textures_csv.split(","):
            # Skip empty strings
            if not texture_url:
                continue
            # Check if absolute or relative path
            if texture_url.startswith(("omniverse://", "http://", "https://", "file://")):
                self._texture_urls.append(texture_url)
            else:
                if not texture_url.startswith("/"):
                    texture_url = "/" + texture_url
                self._texture_urls.append(assets_root_path + texture_url)

    def _reset(self):
        # Bind the original materials back to the prims
        self._restore_original_materials()

        # Delete the materials created for randomization
        self._remove_texture_materials()

        # Clear any empty scopes under the prim behavior scope
        if self.stage:
            scope_root_prim = self.stage.GetPrimAtPath(f"{SCOPE_NAME}")
            remove_empty_scopes(scope_root_prim, self.stage)
        else:
            carb.log_warn(f"[{self.prim_path}] Stage is not valid to remove empty scopes.")

        self._valid_prims.clear()
        self._update_counter = 0

    def _apply_behavior(self):
        # Randomize the textures and parameters for each material
        for mat in self._texture_materials:
            shader = UsdShade.Shader(omni.usd.get_shader_from_material(mat.GetPrim(), get_prim=True))
            diffuse_texture = random.choice(self._texture_urls)
            shader.GetInput("diffuse_texture").Set(diffuse_texture)
            project_uvw = random.choices(
                [True, False],
                weights=[self._project_uvw_probability, 1 - self._project_uvw_probability],
                k=1,
            )[0]
            shader.GetInput("project_uvw").Set(bool(project_uvw))
            texture_scale = random.uniform(self._texture_scale_range[0], self._texture_scale_range[1])
            shader.GetInput("texture_scale").Set((texture_scale, texture_scale))
            texture_rotate = random.uniform(self._texture_rotate_range[0], self._texture_rotate_range[1])
            shader.GetInput("texture_rotate").Set(texture_rotate)

    def _create_materials(self):
        if not self.stage:
            carb.log_warn(f"[{self.prim_path}] Stage is not valid to create materials.")
            return

        # Create a new material for each valid prim
        MDL = "OmniPBR.mdl"
        mtl_name = MDL.rsplit(".", 1)[0]  # Extract the material name without the extension

        # Create a unique looks path in the given behavior scope
        looks_path = omni.usd.get_stage_next_free_path(self.stage, f"{SCOPE_NAME}/{self.BEHAVIOR_NS}/Looks", False)
        for prim in self._valid_prims:
            # Create a unique path for the material (WAR for ISIM-4054)
            rand_postfix = "".join(random.choices(string.ascii_letters + string.digits, k=4))
            mtl_path = omni.usd.get_stage_next_free_path(self.stage, f"{looks_path}/{mtl_name}_{rand_postfix}", False)

            # Create the material and bind it to the prim
            material = create_mdl_material(MDL, mtl_name, mtl_path)
            UsdShade.MaterialBindingAPI(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)

            # Cache the material for randomization
            self._texture_materials.append(material)

    def _get_exposed_variable(self, attr_name):
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)

    def _restore_original_materials(self):
        for prim in self._valid_prims:
            orig_mat = self._initial_materials.get(prim)
            if orig_mat:
                UsdShade.MaterialBindingAPI(prim).Bind(orig_mat, UsdShade.Tokens.strongerThanDescendants)
            else:
                UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
        self._initial_materials.clear()

    def _remove_texture_materials(self):
        for mat in self._texture_materials:
            # Unbind from any still bound prims
            UsdShade.MaterialBindingAPI(mat.GetPrim()).UnbindAllBindings()
            self.stage.RemovePrim(mat.GetPath())
        self._texture_materials.clear()
