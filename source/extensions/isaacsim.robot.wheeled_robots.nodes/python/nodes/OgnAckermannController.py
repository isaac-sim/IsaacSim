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

"""OmniGraph node implementation for the Ackermann steering controller."""

import numpy as np
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode
from isaacsim.robot.experimental.wheeled_robots.controllers import AckermannController
from isaacsim.robot.wheeled_robots.nodes.ogn.OgnAckermannControllerDatabase import OgnAckermannControllerDatabase


class OgnAckermannControllerInternalState(BaseResetNode):
    """Per-instance state for the Ackermann controller OmniGraph node."""

    def __init__(self) -> None:
        self.wheel_base = 0.0
        self.track_width = 0.0
        self.front_wheel_radius = 0.0
        self.back_wheel_radius = 0.0
        self.max_wheel_velocity = 1.0e20
        self.invert_steering = False
        self.max_wheel_rotation_angle = 6.28
        self.max_acceleration = 0.0
        self.max_steering_angle_velocity = 0.0

        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        """Create the `AckermannController` from the current parameter values."""
        self.controller_handle = AckermannController(
            wheel_base=self.wheel_base,
            track_width=self.track_width,
            front_wheel_radius=self.front_wheel_radius,
            back_wheel_radius=self.back_wheel_radius,
            max_wheel_velocity=self.max_wheel_velocity,
            invert_steering=self.invert_steering,
            max_wheel_rotation_angle=self.max_wheel_rotation_angle,
            max_acceleration=self.max_acceleration,
            max_steering_angle_velocity=self.max_steering_angle_velocity,
        )

        self.initialized = True

    def forward(self, command: np.ndarray) -> tuple:
        """Run the controller forward pass and return (positions, velocities)."""
        return self.controller_handle.forward(command)

    def custom_reset(self) -> None:
        """Reset wheel angle and velocity outputs to zero."""
        self.node.get_attribute("outputs:wheelAngles").set([0.0, 0.0])
        self.node.get_attribute("outputs:wheelRotationVelocity").set([0.0, 0.0, 0.0, 0.0])


class OgnAckermannController:
    """OmniGraph node for computing Ackermann steering wheel commands."""

    @staticmethod
    def init_instance(node: og.Node, graph_instance_id: int) -> None:
        """Initialize the per-instance state for this node."""
        state = OgnAckermannControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def internal_state() -> OgnAckermannControllerInternalState:
        """Return a new internal state instance."""
        return OgnAckermannControllerInternalState()

    @staticmethod
    def compute(db: OgnAckermannControllerDatabase) -> bool:
        """Compute wheel angles and velocities from the Ackermann steering model."""
        state = db.per_instance_state

        try:
            if not state.initialized:

                state.wheel_base = db.inputs.wheelBase
                state.track_width = db.inputs.trackWidth
                state.front_wheel_radius = db.inputs.frontWheelRadius
                state.back_wheel_radius = db.inputs.backWheelRadius
                state.max_wheel_velocity = db.inputs.maxWheelVelocity
                state.invert_steering = db.inputs.invertSteering
                state.max_wheel_rotation_angle = db.inputs.maxWheelRotation
                state.max_acceleration = db.inputs.maxAcceleration
                state.max_steering_angle_velocity = db.inputs.maxSteeringAngleVelocity

                state.initialize_controller()

            positions, velocities = state.forward(
                [
                    db.inputs.steeringAngle,
                    db.inputs.steeringAngleVelocity,
                    db.inputs.speed,
                    db.inputs.acceleration,
                    db.inputs.dt,
                ]
            )

            if velocities is not None:
                db.outputs.wheelRotationVelocity = velocities

            if positions is not None:
                db.outputs.wheelAngles = positions

        except Exception as error:
            db.log_warning(str(error))
            return False

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True

    @staticmethod
    def release_instance(node: og.Node, graph_instance_id: int) -> None:
        """Release the per-instance state when the node instance is removed."""
        try:
            state = OgnAckermannControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None

        if state is not None:
            state.reset()
            state.initialized = False
