# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import numpy as np
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.nodes.ogn.OgnIsaacArticulationControllerDatabase import OgnIsaacArticulationControllerDatabase
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


class OgnIsaacArticulationControllerInternalState(BaseResetNode):
    """
    nodes for moving an articulated robot with joint commands
    """

    def __init__(self):
        self.robot_prim = None
        self.controller_handle = None
        self.joint_names = None
        self.joint_indices = None
        self.joint_picked = False
        self.node = None
        super().__init__(initialize=False)

    def initialize_controller(self):
        self.controller_handle = SingleArticulation(self.robot_prim, reset_xform_properties=False)
        self.controller_handle.initialize()
        self.num_dof = self.controller_handle.num_dof
        self.initialized = True

    def joint_indicator(self):
        if self.joint_names:
            self.joint_indices = []
            for name in self.joint_names:
                self.joint_indices.append(self.controller_handle.get_dof_index(name))
        elif np.size(self.joint_indices) > 0:
            self.joint_indices = self.joint_indices
        else:
            # when indices is none (not []), it defaults too all DOFs
            self.joint_indices = None
        self.joint_picked = True

    def apply_action(self, joint_positions, joint_velocities, joint_efforts):
        if self.initialized:
            joint_actions = ArticulationAction()
            joint_actions.joint_indices = self.joint_indices
            if np.size(joint_positions) > 0:
                joint_actions.joint_positions = joint_positions
            if np.size(joint_velocities) > 0:
                joint_actions.joint_velocities = joint_velocities
            if np.size(joint_efforts) > 0:
                joint_actions.joint_efforts = joint_efforts
            self.controller_handle.apply_action(control_actions=joint_actions)

    def custom_reset(self):
        self.controller_handle = None
        if self.initialized:
            self.node.get_attribute("inputs:positionCommand").set(np.empty(shape=(0, 0), dtype=np.double))
            self.node.get_attribute("inputs:velocityCommand").set(np.empty(shape=(0, 0), dtype=np.double))
            self.node.get_attribute("inputs:effortCommand").set(np.empty(shape=(0, 0), dtype=np.double))

        pass


class OgnIsaacArticulationController:
    """
    nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def init_instance(node, graph_instance_id):
        state = OgnIsaacArticulationControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node

    @staticmethod
    def internal_state():
        return OgnIsaacArticulationControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        try:
            if not state.initialized:
                if len(db.inputs.robotPath) != 0:
                    state.robot_prim = db.inputs.robotPath
                else:
                    if len(db.inputs.targetPrim) == 0:
                        db.log_error("No robot prim found for the articulation controller")
                        return False
                    else:
                        state.robot_prim = db.inputs.targetPrim[0].GetString()

                # initialize the controller handle for the robot
                state.initialize_controller()

            # pick the joints that are being commanded, this can be different at every step
            joint_names = db.inputs.jointNames
            if joint_names and np.asarray([joint_names != state.joint_names]).flatten().any():
                state.joint_names = joint_names
                state.joint_picked = False

            joint_indices = db.inputs.jointIndices
            if np.asarray(joint_indices).any() and not np.array_equal(joint_indices, state.joint_indices):
                state.joint_indices = np.array(joint_indices)
                state.joint_picked = False

            if not state.joint_picked:
                state.joint_indicator()

            joint_positions = db.inputs.positionCommand
            joint_velocities = db.inputs.velocityCommand
            joint_efforts = db.inputs.effortCommand

            state.apply_action(joint_positions, joint_velocities, joint_efforts)

        except Exception as error:
            db.log_warn(str(error))
            return False

        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnIsaacArticulationControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
            state.initialized = False
