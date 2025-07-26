# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import os
from typing import Optional

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver


class KinematicsSolver(ArticulationKinematicsSolver):
    """Kinematics Solver for UR10 robot.  This class loads a LulaKinematicsSolver object

    Args:
        robot_articulation (SingleArticulation): An initialized Articulation object representing this UR10
        end_effector_frame_name (Optional[str]): The name of the UR10 end effector.  If None, an end effector link will
            be automatically selected.  Defaults to None.
        attach_gripper (Optional[bool]): If True, a URDF will be loaded that includes a suction gripper.  Defaults to False.
    """

    def __init__(
        self,
        robot_articulation: SingleArticulation,
        end_effector_frame_name: Optional[str] = None,
        attach_gripper: Optional[bool] = False,
    ) -> None:

        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")

        if attach_gripper:
            robot_urdf_path = os.path.join(
                mg_extension_path, "motion_policy_configs/universal_robots/ur10/ur10_robot_suction.urdf"
            )
        else:
            robot_urdf_path = os.path.join(
                mg_extension_path, "motion_policy_configs/universal_robots/ur10/ur10_robot.urdf"
            )
        if attach_gripper:
            robot_description_yaml_path = os.path.join(
                mg_extension_path,
                "motion_policy_configs/universal_robots/ur10/rmpflow_suction/ur10_robot_description.yaml",
            )
        else:
            robot_description_yaml_path = os.path.join(
                mg_extension_path, "motion_policy_configs/universal_robots/ur10/rmpflow/ur10_robot_description.yaml"
            )

        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_yaml_path, urdf_path=robot_urdf_path
        )

        if end_effector_frame_name is None:
            if attach_gripper:
                end_effector_frame_name = "ee_suction_link"
            else:
                end_effector_frame_name = "ee_link"

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return
