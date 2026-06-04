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

"""Define RMPFlow motion policy controller for the Cobotta Pro 900 robot."""

import os

import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import SingleArticulation


class RMPFlowController(mg.MotionPolicyController):
    """Control the Cobotta Pro 900 end effector using RMPFlow motion policy."""

    def __init__(self, name: str, robot_articulation: SingleArticulation, physics_dt: float = 1.0 / 60.0) -> None:
        self.rmpflow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=os.path.join(os.path.dirname(__file__), "../rmpflow/robot_descriptor.yaml"),
            rmpflow_config_path=os.path.join(os.path.dirname(__file__), "../rmpflow/denso_rmpflow_common.yaml"),
            urdf_path=os.path.join(os.path.dirname(__file__), "../rmpflow/cobotta_pro_900.urdf"),
            end_effector_frame_name="gripper_center",
            maximum_substep_size=0.00334,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        (
            self._default_position,
            self._default_orientation,
        ) = self._articulation_motion_policy._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        """Reset the controller and restore the robot base pose."""
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
