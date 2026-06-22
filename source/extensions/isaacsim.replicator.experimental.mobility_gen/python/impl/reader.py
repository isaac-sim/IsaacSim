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

"""Reader for loading MobilityGen recorded data from disk."""

from __future__ import annotations

import glob
import os
from collections import OrderedDict

import numpy as np
import PIL.Image

from .config import Config
from .occupancy_map import OccupancyMap


class MobilityGenReader:
    """Reader for accessing recorded MobilityGen data from a directory.

    Args:
        recording_path: The path to the recorded data directory.
    """

    def __init__(self, recording_path: str) -> None:
        self.recording_path = recording_path

        state_dict_paths = glob.glob(os.path.join(self.recording_path, "state", "common", "*.npz"))
        if not state_dict_paths and glob.glob(os.path.join(self.recording_path, "state", "common", "*.npy")):
            import carb

            carb.log_error(
                f"[MobilityGenReader] Recording at '{recording_path}' uses the legacy .npy format. "
                "Run migrate_recordings.py to convert it to .npz before replaying."
            )
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
        """Read and return the scenario configuration.

        Returns:
            The deserialized Config object.
        """
        with open(os.path.join(self.recording_path, "config.json")) as f:
            config = Config.from_json(f.read())
        return config

    def read_occupancy_map(self) -> OccupancyMap:
        """Read and return the occupancy map.

        Returns:
            The loaded OccupancyMap.
        """
        return OccupancyMap.from_ros_yaml(os.path.join(self.recording_path, "occupancy_map", "map.yaml"))

    def read_rgb(self, name: str, index: int) -> np.ndarray:
        """Read an RGB image frame by camera name and step index.

        Args:
            name: The camera name subfolder.
            index: The step index.

        Returns:
            The RGB image as a numpy array.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "rgb", name, f"{step:08d}.jpg"))
        return np.asarray(image)

    def read_state_dict_rgb(self, index: int) -> dict:
        """Read all RGB images for the given step index.

        Args:
            index: The step index.

        Returns:
            A dict mapping camera name to RGB numpy array.
        """
        rgb_dict = OrderedDict()
        for name in self.rgb_names:
            data = self.read_rgb(name, index)
            rgb_dict[name] = data
        return rgb_dict

    def read_segmentation(self, name: str, index: int) -> np.ndarray:
        """Read a segmentation image frame by camera name and step index.

        Args:
            name: The camera name subfolder.
            index: The step index.

        Returns:
            The segmentation image as a numpy array.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "segmentation", name, f"{step:08d}.png"))
        return np.asarray(image)

    def read_normals(self, name: str, index: int) -> np.ndarray:
        """Read a surface normals frame by camera name and step index.

        Args:
            name: The camera name subfolder.
            index: The step index.

        Returns:
            The normals data as a numpy array.
        """
        step = self.steps[index]
        data = np.load(os.path.join(self.recording_path, "state", "normals", name, f"{step:08d}.npy"))
        return data

    def read_state_dict_segmentation(self, index: int) -> dict:
        """Read all segmentation images for the given step index.

        Args:
            index: The step index.

        Returns:
            A dict mapping camera name to segmentation numpy array.
        """
        segmentation_dict = OrderedDict()
        for name in self.segmentation_names:
            data = self.read_segmentation(name, index)
            segmentation_dict[name] = data
        return segmentation_dict

    def read_depth(self, name: str, index: int, eps: float = 1e-6) -> np.ndarray:
        """Read a depth image frame by camera name and step index.

        Args:
            name: The camera name subfolder.
            index: The step index.
            eps: Small value to avoid division by zero when decoding depth. Defaults to 1e-6.

        Returns:
            The depth image as a float32 numpy array in meters.
        """
        step = self.steps[index]
        image = PIL.Image.open(os.path.join(self.recording_path, "state", "depth", name, f"{step:08d}.png")).convert(
            "I;16"
        )
        depth = 65535 / (np.asarray(image).astype(np.float32) + eps) - 1.0
        return depth

    def read_state_dict_depth(self, index: int) -> dict:
        """Read all depth images for the given step index.

        Args:
            index: The step index.

        Returns:
            A dict mapping camera name to depth numpy array.
        """
        depth_dict = OrderedDict()
        for name in self.depth_names:
            data = self.read_depth(name, index)
            depth_dict[name] = data
        return depth_dict

    def read_state_dict_normals(self, index: int) -> dict:
        """Read all surface normals frames for the given step index.

        Args:
            index: The step index.

        Returns:
            A dict mapping camera name to normals numpy array.
        """
        normals_dict = OrderedDict()
        for name in self.normals_names:
            data = self.read_normals(name, index)
            normals_dict[name] = data
        return normals_dict

    def read_state_dict_common(self, index: int) -> dict:
        """Read the common (non-image) state dictionary for the given step index.

        Args:
            index: The step index.

        Returns:
            The common state dictionary loaded from disk.
        """
        step = self.steps[index]
        path = os.path.join(self.recording_path, "state", "common", f"{step:08d}.npz")
        return dict(np.load(path))

    def read_state_dict(self, index: int) -> dict:
        """Read the full state dictionary (all modalities) for the given step index.

        Args:
            index: The step index.

        Returns:
            The merged state dictionary with all data modalities.
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
        """Return the number of recorded steps.

        Returns:
            Number of recorded steps.
        """
        return len(self.steps)

    def __getitem__(self, index: int) -> dict:
        """Return the full state dictionary for the given step index.

        Args:
            index: Step index to read.

        Returns:
            The full state dictionary for the step.
        """
        return self.read_state_dict(index)
