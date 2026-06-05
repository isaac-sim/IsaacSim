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

"""Reader module for accessing mobility generation recording data from structured directories."""

import glob
import os
from collections import OrderedDict

import numpy as np
import PIL.Image

from .config import Config
from .occupancy_map import OccupancyMap


class MobilityGenReader:
    """Reads and processes mobility generation recording data from a structured directory.

    This class provides access to various types of recorded data including RGB images, depth maps,
    segmentation masks, surface normals, configuration settings, and occupancy maps. The recording
    directory contains timestamped data organized by data type and camera/sensor name.

    The reader automatically discovers available timesteps and sensor names from the recording structure.
    It supports reading individual data types or complete state dictionaries that combine all available
    data for a given timestep.

    Data types supported:
    - RGB images (JPEG format)
    - Depth maps (PNG format with depth encoding)
    - Segmentation masks (PNG format)
    - Surface normals (NumPy arrays)
    - Common state data (NumPy pickle files)
    - Configuration files (JSON format)
    - Occupancy maps (ROS YAML format)

    The class implements sequence-like behavior, allowing iteration and indexing to access state
    dictionaries by timestep index.

    Args:
        recording_path: Path to the recording directory containing the structured mobility generation data.
    """

    def __init__(self, recording_path: str) -> None:
        self.recording_path = recording_path

        state_dict_paths = glob.glob(os.path.join(self.recording_path, "state", "common", "*.npy"))

        steps = [int(os.path.basename(path).split(".")[0]) for path in state_dict_paths]
        self.steps = sorted(steps)

        self.rgb_folders = glob.glob(os.path.join(self.recording_path, "state", "rgb", "*"))
        self.segmentation_folders = glob.glob(os.path.join(self.recording_path, "state", "segmentation", "*"))
        self.depth_folders = glob.glob(os.path.join(self.recording_path, "state", "depth", "*"))
        self.normals_folders = glob.glob(os.path.join(self.recording_path, "state", "normals", "*"))

        self.rgb_names = [os.path.basename(folder) for folder in self.rgb_folders]
        self.segmentation_names = [os.path.basename(folder) for folder in self.segmentation_folders]
        self.depth_names = [os.path.basename(folder) for folder in self.depth_folders]
        self.normals_names = [os.path.basename(folder) for folder in self.normals_folders]

    def read_config(self) -> Config:
        """Reads the configuration from the recording.

        Returns:
            The configuration object loaded from config.json.
        """
        with open(os.path.join(self.recording_path, "config.json")) as f:
            config = Config.from_json(f.read())
        return config

    def read_occupancy_map(self) -> object:
        """Reads the occupancy map from the recording.

        Returns:
            The occupancy map loaded from the ROS YAML file.
        """
        return OccupancyMap.from_ros_yaml(os.path.join(self.recording_path, "occupancy_map", "map.yaml"))

    def read_rgb(self, name: str, index: int) -> object:
        """Reads RGB image data for a specific camera at a given step index.

        Args:
            name: Name of the RGB camera.
            index: Step index to read from.

        Returns:
            RGB image data as a NumPy array.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "rgb", name, f"{step:08d}.jpg"))
        return np.asarray(image)

    def read_state_dict_rgb(self, index: int) -> object:
        """Reads RGB data from all cameras at a given step index.

        Args:
            index: Step index to read from.

        Returns:
            Dictionary mapping camera names to RGB image arrays.
        """
        rgb_dict = OrderedDict()
        for name in self.rgb_names:
            data = self.read_rgb(name, index)
            rgb_dict[name] = data
        return rgb_dict

    def read_segmentation(self, name: str, index: int) -> object:
        """Reads segmentation image data for a specific camera at a given step index.

        Args:
            name: Name of the segmentation camera.
            index: Step index to read from.

        Returns:
            Segmentation image data as a NumPy array.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "segmentation", name, f"{step:08d}.png"))
        return np.asarray(image)

    def read_normals(self, name: str, index: int) -> object:
        """Reads normals data for a specific camera at a given step index.

        Args:
            name: Name of the normals camera.
            index: Step index to read from.

        Returns:
            Normals data as a NumPy array.
        """
        step = self.steps[index]
        data = np.load(os.path.join(self.recording_path, "state", "normals", name, f"{step:08d}.npy"))
        return data

    def read_state_dict_segmentation(self, index: int) -> object:
        """Reads segmentation data from all cameras at a given step index.

        Args:
            index: Step index to read from.

        Returns:
            Dictionary mapping camera names to segmentation image arrays.
        """
        segmentation_dict = OrderedDict()
        for name in self.segmentation_names:
            data = self.read_segmentation(name, index)
            segmentation_dict[name] = data
        return segmentation_dict

    def read_depth(self, name: str, index: int, eps: float = 1e-6) -> object:
        """Reads depth image data for a specific camera at a given step index.

        Args:
            name: Name of the depth camera.
            index: Step index to read from.
            eps: Small epsilon value to prevent division by zero.

        Returns:
            Depth image data as a NumPy array with converted depth values.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "depth", name, f"{step:08d}.png")).convert(
            "I;16"
        )
        depth = 65535 / (np.asarray(image).astype(np.float32) + eps) - 1.0
        return depth

    def read_state_dict_depth(self, index: int) -> object:
        """Reads depth data from all cameras at a given step index.

        Args:
            index: Step index to read from.

        Returns:
            Dictionary mapping camera names to depth image arrays.
        """
        depth_dict = OrderedDict()
        for name in self.depth_names:
            data = self.read_depth(name, index)
            depth_dict[name] = data
        return depth_dict

    def read_state_dict_normals(self, index: int) -> object:
        """Reads normals data from all cameras at a given step index.

        Args:
            index: Step index to read from.

        Returns:
            Dictionary mapping camera names to normals data arrays.
        """
        normals_dict = OrderedDict()
        for name in self.normals_names:
            data = self.read_normals(name, index)
            normals_dict[name] = data
        return normals_dict

    def read_state_dict_common(self, index: int) -> object:
        """Reads the common state dictionary for a specific step.

        Args:
            index: Index of the step to read from.

        Returns:
            The common state dictionary loaded from the numpy file.
        """
        step = self.steps[index]
        state_dict = np.load(
            os.path.join(self.recording_path, "state", "common", f"{step:08d}.npy"), allow_pickle=True
        ).item()
        return state_dict

    def read_state_dict(self, index: int) -> object:
        """Reads the complete state dictionary including common data and all sensor data.

        Args:
            index: Index of the step to read from.

        Returns:
            Complete state dictionary containing common state, RGB, segmentation, depth, and normals data.
        """
        state_dict = self.read_state_dict_common(index)
        rgb_dict = self.read_state_dict_rgb(index)
        segmentation_dict = self.read_state_dict_segmentation(index)
        depth_dict = self.read_state_dict_depth(index)
        normals_dict = self.read_state_dict_normals(index)

        full_dict = OrderedDict()
        full_dict.update(state_dict)
        full_dict.update(rgb_dict)
        full_dict.update(segmentation_dict)
        full_dict.update(depth_dict)
        full_dict.update(normals_dict)

        return full_dict

    def __len__(self) -> int:
        """Number of recorded steps available for reading.

        Returns:
            The number of recorded steps.
        """
        return len(self.steps)

    def __getitem__(self, index: int) -> object:
        """Retrieves the complete state dictionary for a specific step index.

        Args:
            index: Index of the step to retrieve.

        Returns:
            Complete state dictionary for the specified step.
        """
        return self.read_state_dict(index)
