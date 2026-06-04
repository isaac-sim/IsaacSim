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

"""Kinematics solver configuration for the Franka robot."""

from __future__ import annotations

import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for Franka robot. This class loads a LulaKinematicsSolver object.

    Args:
        robot_articulation: An initialized Articulation object representing this Franka.
        end_effector_frame_name: The name of the Franka end effector. If None, an end effector link will
            be automatically selected.
    """

    def __init__(self, robot_articulation: SingleArticulation, end_effector_frame_name: str | None = None) -> None:
        kinematics_config = interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
        self._kinematics = LulaKinematicsSolver(**kinematics_config)

        if end_effector_frame_name is None:
            end_effector_frame_name = "right_gripper"

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return
