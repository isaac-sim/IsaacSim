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

"""Configuration dataclass for MobilityGen recordings."""

import json
from dataclasses import asdict, dataclass


@dataclass
class Config:
    """Configuration for a MobilityGen recording.

    Args:
        scenario_type: The type of scenario.
        robot_type: The type of robot.
        scene_usd: The path to the scene USD file.
    """

    scenario_type: str
    robot_type: str
    scene_usd: str

    def to_json(self) -> str:
        """Serialize the config to a JSON string.

        Returns:
            The JSON-serialized config string.
        """
        return json.dumps(asdict(self), indent=2)

    @staticmethod
    def from_json(data: str) -> "Config":
        """Deserialize a config from a JSON string.

        Args:
            data: The JSON string to deserialize.

        Returns:
            The deserialized Config object.
        """
        data = json.loads(data)
        return Config(scenario_type=data["scenario_type"], robot_type=data["robot_type"], scene_usd=data["scene_usd"])
