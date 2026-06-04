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

"""Configuration dataclass for Isaac Sim Replicator mobility generation scenarios."""

import json
from dataclasses import asdict, dataclass


@dataclass
class Config:
    """Config(scenario_type: str, robot_type: str, scene_usd: str).

    Args:
        scenario_type: The type of scenario to generate.
        robot_type: The type of robot to use in the scenario.
        scene_usd: Path to the USD file containing the scene.
    """

    scenario_type: str
    robot_type: str
    scene_usd: str

    def to_json(self) -> str:
        """Serializes the configuration to a JSON string.

        Returns:
            JSON representation of the configuration.
        """
        return json.dumps(asdict(self), indent=2)

    @staticmethod
    def from_json(data: str) -> "Config":
        """Creates a Config instance from a JSON string.

        Args:
            data: JSON string containing the configuration data.

        Returns:
            A new Config instance with the deserialized data.
        """
        data = json.loads(data)
        return Config(scenario_type=data["scenario_type"], robot_type=data["robot_type"], scene_usd=data["scene_usd"])
