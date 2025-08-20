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
from isaacsim.core.nodes.ogn.OgnIsaacArticulationStateDatabase import OgnIsaacArticulationStateDatabase
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import get_current_stage
from pxr import UsdPhysics


class OgnIsaacArticulationStateInternalState(BaseResetNode):
    """
    Internal node state for queuing articulation state
    """

    def __init__(self):
        self.robot_prim = None
        self.dof_names = None
        self.dof_indices = None

        self._dof_names = []
        self._dof_indices = None
        self._joint_indices = None
        self._articulation = None
        super().__init__(initialize=False)

    def initialize_articulation(self):
        self._articulation = SingleArticulation(self.robot_prim, reset_xform_properties=False)
        self._articulation.initialize()
        self.initialized = True

    def pick_dofs(self, dof_names, dof_indices):
        self.dof_names = dof_names
        self.dof_indices = dof_indices
        # names given
        if len(self.dof_names):
            self._dof_names = self.dof_names[:]
            self._dof_indices = np.array([self._articulation.get_dof_index(name) for name in self.dof_names])
        # DOF indexes given
        elif self.dof_indices.size:
            self._dof_names = [self._articulation.dof_names[index] for index in self.dof_indices]
            self._dof_indices = self.dof_indices.copy()
        # no names or indexes (all DOFs)
        else:
            self._dof_names = self._articulation.dof_names
            self._dof_indices = np.array([self._articulation.get_dof_index(name) for name in self._dof_names])
        # get joint indices
        self._joint_indices = []
        stage = get_current_stage()
        dof_paths = {path.split("/")[-1]: path for path in self._articulation._articulation_view._dof_paths[0]}
        for name in self._dof_names:
            joint = UsdPhysics.Joint.Get(stage, dof_paths[name])
            link_name = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0]).GetName()
            self._joint_indices.append(self._articulation._articulation_view.get_link_index(link_name))

    def get_dof_names(self):
        return self._dof_names

    def get_articulation_state(self):
        positions, velocities, efforts, forces, torques = [], [], [], [], []
        if self.initialized:
            positions = self._articulation.get_joint_positions(self._dof_indices)
            velocities = self._articulation.get_joint_velocities(self._dof_indices)
            efforts = self._articulation.get_measured_joint_efforts(self._dof_indices)
            forces_torques = self._articulation.get_measured_joint_forces(self._joint_indices)
            forces = forces_torques[:, :3]
            torques = forces_torques[:, 3:]
        return positions, velocities, efforts, forces, torques

    def custom_reset(self):
        self._articulation = None
        pass


class OgnIsaacArticulationState:
    """
    Node for queuing articulation state
    """

    @staticmethod
    def internal_state():
        return OgnIsaacArticulationStateInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        try:
            if not state.initialized:
                if len(db.inputs.robotPath) != 0:
                    state.robot_prim = db.inputs.robotPath
                else:
                    if not len(db.inputs.targetPrim):
                        db.log_error("No robot prim found for the articulation state")
                        return False
                    else:
                        state.robot_prim = db.inputs.targetPrim[0].GetString()

                # initialize the articulation handle for the robot
                state.initialize_articulation()

            # pick the articulation DOFs to be queried, they can be different at every step
            dof_names = db.inputs.jointNames
            dof_indices = np.array(db.inputs.jointIndices)
            if not np.array_equal(dof_names, state.dof_names) or not np.array_equal(dof_indices, state.dof_indices):
                state.pick_dofs(dof_names, dof_indices)

            # get joint names
            db.outputs.jointNames = state.get_dof_names()

            # get articulation state
            positions, velocities, efforts, forces, torques = state.get_articulation_state()
            db.outputs.jointPositions = positions
            db.outputs.jointVelocities = velocities
            db.outputs.measuredJointEfforts = efforts
            db.outputs.measuredJointForces = forces
            db.outputs.measuredJointTorques = torques

        except Exception as error:
            db.log_warn(str(error))
            return False

        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnIsaacArticulationStateDatabase.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
