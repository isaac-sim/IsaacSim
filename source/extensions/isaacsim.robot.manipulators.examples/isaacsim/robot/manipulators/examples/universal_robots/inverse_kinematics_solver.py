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

from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.robot_motion.motion_generation.kinematics import InverseKinematicsSolver as BaseInverseKinematicsSolver


class InverseKinematicsSolver(BaseInverseKinematicsSolver):
    """[summary]

    Args:
        name (str): [description]
        robot_prim_path (str): [description]
        robot_urdf_path (Optional[str], optional): [description]. Defaults to None.
        robot_description_yaml_path (Optional[str], optional): [description]. Defaults to None.
        end_effector_frame_name (Optional[str], optional): [description]. Defaults to None.
        attach_gripper (bool, optional): [description]. Defaults to False.
    """

    def __init__(
        self,
        name: str,
        robot_prim_path: str,
        robot_urdf_path: Optional[str] = None,
        robot_description_yaml_path: Optional[str] = None,
        end_effector_frame_name: Optional[str] = None,
        attach_gripper: bool = False,
    ) -> None:
        mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
        if robot_urdf_path is None:
            if attach_gripper:
                robot_urdf_path = os.path.join(
                    mg_extension_path, "motion_policy_configs/universal_robots/ur10/ur10_robot_suction.urdf"
                )
            else:
                robot_urdf_path = os.path.join(
                    mg_extension_path, "motion_policy_configs/universal_robots/ur10/ur10_robot.urdf"
                )
        if robot_description_yaml_path is None:
            if attach_gripper:
                robot_description_yaml_path = os.path.join(
                    mg_extension_path,
                    "motion_policy_configs/universal_robots/ur10/rmpflow_suction/ur10_robot_description.yaml",
                )
            else:
                robot_description_yaml_path = os.path.join(
                    mg_extension_path, "motion_policy_configs/universal_robots/ur10/rmpflow/ur10_robot_description.yaml"
                )
        if end_effector_frame_name is None:
            if attach_gripper:
                end_effector_frame_name = "ee_suction_link"
            else:
                end_effector_frame_name = "ee_link"
        BaseInverseKinematicsSolver.__init__(
            self,
            name=name,
            robot_urdf_path=robot_urdf_path,
            robot_description_yaml_path=robot_description_yaml_path,
            robot_prim_path=robot_prim_path,
            end_effector_frame_name=end_effector_frame_name,
        )
        return
