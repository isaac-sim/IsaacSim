# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide data logging functionality for collecting, storing, and replaying simulation data."""

import json
from collections.abc import Callable

from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks.base_task import BaseTask
from isaacsim.core.utils.types import DataFrame


class DataLogger:
    """Provide data collection, storage, and replay functionality for simulation data.

    Collect simulation data at runtime and save to disk for later replay or analysis.
    Support pausing and resuming data collection during simulation.
    """

    def __init__(self) -> None:
        self._pause = True
        self._data_frames = []
        self._data_frame_logging_func = None

    def add_data(self, data: dict, current_time_step: float, current_time: float) -> None:
        """Add data to the log.

        Args:
            data: Dictionary representing the data to be logged at this time index.
            current_time_step: Time step corresponding to the data collected.
            current_time: Time in seconds corresponding to the data collected.

        """
        if self._pause:
            return
        self._data_frames.append(DataFrame(current_time_step=current_time_step, current_time=current_time, data=data))
        return

    def get_num_of_data_frames(self) -> int:
        """Get the number of data frames in the logger.

        Returns:
            The number of data frames collected in the data logger.

        """
        return len(self._data_frames)

    def pause(self) -> None:
        """Pause data collection."""
        self._pause = True
        return

    def start(self) -> None:
        """Resume or starts data collection."""
        self._pause = False
        return

    def is_started(self) -> bool:
        """Check if data collection is currently active.

        Returns:
            True if data collection is started or resumed, False otherwise.

        """
        return not self._pause

    def reset(self) -> None:
        """Clears the data in the logger."""
        self._pause = True
        self._data_frames = []
        return

    def get_data_frame(self, data_frame_index: int) -> DataFrame:
        """Retrieves a specific data frame from the logger.

        Args:
            data_frame_index: Index of the data frame to retrieve.

        Returns:
            Data Frame collected at the specified data frame index.

        """
        return self._data_frames[data_frame_index]

    def add_data_frame_logging_func(self, func: Callable[[list[BaseTask], Scene], dict]) -> None:
        """Add a data collection function to be called at every step when the logger is started.

        Args:
            func: Function to be called at every step when the logger is started.
                Should follow:

                .. code-block:: python

                    def dummy_data_collection_fn(tasks, scene):
                        return {"data 1": [data]}

        """
        self._data_frame_logging_func = func
        return

    def save(self, log_path: str) -> None:
        """Save the current data in the logger to a json file.

        Args:
            log_path: Path of the json file to be used to save the data.

        """
        data = {}
        data["Isaac Sim Data"] = [data_frame.get_dict() for data_frame in self._data_frames]
        with open(log_path, "w") as outfile:
            json.dump(data, outfile)
        return

    def load(self, log_path: str) -> None:
        """Load data from a json file to read back a previous saved data or to resume recording data from another time step.

        Args:
            log_path: Path of the json file to be used to load the data.

        """
        self._pause = True
        self._data_frames = []
        with open(log_path) as json_file:
            json_data = json.load(json_file)
            data_frames = json_data["Isaac Sim Data"]
            data_frames = [DataFrame.init_from_dict(dict_representation=data_frame) for data_frame in data_frames]
            self._data_frames = data_frames
        return
