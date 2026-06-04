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

"""Provides writer functionality for mobility generation data output including sensor states, configurations, and USD stages."""

import os
import shutil

import numpy as np
import PIL.Image

from .config import Config
from .occupancy_map import OccupancyMap


class MobilityGenWriter:
    """Writer class for mobility generation data output.

    This class handles the structured output of mobility generation simulation data including sensor states,
    configurations, occupancy maps, and USD stages. It organizes data into a hierarchical directory structure
    with separate folders for different data types such as RGB images, segmentation masks, depth data, surface
    normals, and common state information.

    The writer creates timestamped files for state data and provides methods to save configuration files,
    occupancy maps in ROS format, and copy USD stage files. It supports various image formats and numpy
    arrays for different sensor modalities.

    Args:
        path: Output directory path where all mobility generation data will be written.
    """

    def __init__(self, path: str) -> None:
        self.path = path

    def write_state_dict_common(self, state_dict: dict, step: int) -> None:
        """Writes the common state dictionary data to a NumPy file.

        Args:
            state_dict: The common state data to write.
            step: The step number used for file naming.
        """
        dict_folder = os.path.join(self.path, "state", "common")
        if not os.path.exists(dict_folder):
            os.makedirs(dict_folder)
        state_dict_path = os.path.join(dict_folder, f"{step:08d}.npy")
        np.save(state_dict_path, state_dict)

    def write_state_dict_rgb(self, state_rgb: dict, step: int) -> None:
        """Writes RGB state data as JPEG images.

        Args:
            state_rgb: Dictionary mapping names to RGB image arrays.
            step: The step number used for file naming.
        """
        for name, value in state_rgb.items():
            if value is not None:
                image_folder = os.path.join(self.path, "state", "rgb", name)
                if not os.path.exists(image_folder):
                    os.makedirs(image_folder)
                image_path = os.path.join(image_folder, f"{step:08d}.jpg")
                image = PIL.Image.fromarray(value)
                image.save(image_path)

    def write_state_dict_segmentation(self, state_segmentation: dict, step: int) -> None:
        """Writes segmentation state data as PNG images.

        Args:
            state_segmentation: Dictionary mapping names to segmentation image arrays.
            step: The step number used for file naming.
        """
        for name, value in state_segmentation.items():
            if value is not None:
                image_folder = os.path.join(self.path, "state", "segmentation", name)
                if not os.path.exists(image_folder):
                    os.makedirs(image_folder)
                image_path = os.path.join(image_folder, f"{step:08d}.png")
                image = PIL.Image.fromarray(value)
                image.save(image_path)

    def write_state_dict_depth(self, state_np: dict, step: int) -> None:
        """Writes depth state data as 16-bit inverse depth PNG images.

        Args:
            state_np: Dictionary mapping names to depth arrays.
            step: The step number used for file naming.
        """
        for name, value in state_np.items():
            if value is not None:
                output_folder = os.path.join(self.path, "state", "depth", name)
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)

                # Inverse depth 16bit
                inverse_depth = 1.0 / (1.0 + value)
                inverse_depth = (65535 * inverse_depth).astype(np.uint16)
                image = PIL.Image.fromarray(inverse_depth, "I;16")

                output_path = os.path.join(output_folder, f"{step:08d}.png")

                image.save(output_path)

    def write_state_dict_normals(self, state_np: dict, step: int) -> None:
        """Writes normals state data as NumPy files.

        Args:
            state_np: Dictionary mapping names to normal arrays.
            step: The step number used for file naming.
        """
        for name, value in state_np.items():
            if value is not None:
                output_folder = os.path.join(self.path, "state", "normals", name)
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)
                output_path = os.path.join(output_folder, f"{step:08d}.npy")
                np.save(output_path, value)

    def copy_stage(self, input_path: str) -> None:
        """Copies a USD or USDZ stage file to the output directory.

        Args:
            input_path: Path to the source stage file.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if input_path.endswith(".usdz"):
            shutil.copyfile(input_path, os.path.join(self.path, "stage.usdz"))
        else:
            shutil.copyfile(input_path, os.path.join(self.path, "stage.usd"))

    def write_config(self, config: Config) -> None:
        """Writes the configuration to a JSON file.

        Args:
            config: The configuration object to write.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        with open(os.path.join(self.path, "config.json"), "w") as f:
            f.write(config.to_json())

    def write_occupancy_map(self, occupancy_map: OccupancyMap) -> None:
        """Writes the occupancy map in ROS format.

        Args:
            occupancy_map: The occupancy map object to write.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        occupancy_map.save_ros(os.path.join(self.path, "occupancy_map"))

    def copy_init(self, other_path: str) -> None:
        """Copies initialization files from another path including stage, config, and occupancy map.

        Args:
            other_path: Path to the source directory containing initialization files.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if os.path.exists(os.path.join(other_path, "stage.usdz")):
            shutil.copyfile(os.path.join(other_path, "stage.usdz"), os.path.join(self.path, "stage.usdz"))
        else:
            shutil.copyfile(os.path.join(other_path, "stage.usd"), os.path.join(self.path, "stage.usd"))
        shutil.copyfile(os.path.join(other_path, "config.json"), os.path.join(self.path, "config.json"))
        shutil.copytree(os.path.join(other_path, "occupancy_map"), os.path.join(self.path, "occupancy_map"))
