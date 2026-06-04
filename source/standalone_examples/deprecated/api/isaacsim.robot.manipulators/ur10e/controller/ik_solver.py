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

"""Provide inverse kinematics solver controller for UR10e robot."""

import os
from typing import Optional

from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics solver for UR10e robot using Lula IK."""

    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        # TODO: change the config path
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=os.path.join(os.path.dirname(__file__), "../rmpflow/robot_descriptor.yaml"),
            urdf_path=os.path.join(os.path.dirname(__file__), "../rmpflow/ur10e.urdf"),
        )
        if end_effector_frame_name is None:
            end_effector_frame_name = "ee_link_robotiq_arg2f_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return
