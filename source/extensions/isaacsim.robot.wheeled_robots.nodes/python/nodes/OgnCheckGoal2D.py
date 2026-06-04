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

"""OmniGraph node implementation for 2D goal-reached checking."""

import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.graph.core as og
from isaacsim.core.nodes import BaseResetNode
from isaacsim.robot.experimental.wheeled_robots.controllers import normalize_angle
from isaacsim.robot.wheeled_robots.nodes.ogn.OgnCheckGoal2DDatabase import OgnCheckGoal2DDatabase


class OgnCheckGoal2DInternalState(BaseResetNode):
    """Per-instance state for the CheckGoal2D OmniGraph node."""

    # modeled after OgnDifferentialController state layout
    def __init__(self) -> None:
        # store target pos to prevent repeated & unnecessary db.inputs.target access
        self.node = None
        self.target = [0, 0, 0]  # [x, y, z_rot]
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Reset the target to origin."""
        # reset target to origin (not an ideal reset solution but technically works)
        self.node.get_attribute("inputs:target").set([0.0, 0.0, 0.0])


class OgnCheckGoal2D:
    """OmniGraph node that checks whether a 2D goal position and orientation have been reached."""

    @staticmethod
    def init_instance(node: og.Node, graph_instance_id: int) -> None:
        """Initialize the per-instance state for this node."""
        state = OgnCheckGoal2DDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def release_instance(node: og.Node, graph_instance_id: int) -> None:
        """Release the per-instance state when the node instance is removed."""
        try:
            state = OgnCheckGoal2DDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None

        if state is not None:
            state.reset()

    @staticmethod
    def internal_state() -> OgnCheckGoal2DInternalState:
        """Return a new internal state instance."""
        return OgnCheckGoal2DInternalState()

    @staticmethod
    def compute(db: OgnCheckGoal2DDatabase) -> bool:
        """Compare current position/orientation against the target and output whether the goal is reached."""
        state = db.per_instance_state

        # if planner outputs targetChanged = True, new target data will be accessed and stored
        if db.inputs.targetChanged:
            state.target = db.inputs.target

        # get current pos/rot data
        pos = db.inputs.currentPosition
        x = pos[0]
        y = pos[1]
        _, _, rot = quatd4_to_euler(db.inputs.currentOrientation)

        # compare & output if diff between current pos/rot and target pos/rot is above threshold limits
        t = db.inputs.thresholds
        db.outputs.reachedGoal = [np.hypot(x - state.target[0], y - state.target[1]) <= t[0], rot <= t[1]]

        # begin next node (steering control)
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True


def quatd4_to_euler(orientation: np.ndarray) -> tuple[float, float, float]:
    """Convert a (x, y, z, w) quaternion to normalized Euler angles (roll, pitch, yaw).

    Args:
        orientation: Quaternion in (x, y, z, w) order.

    Returns:
        Tuple of normalized (roll, pitch, yaw) in radians.

    """
    orientation = np.asarray(orientation, dtype=float)
    if orientation.size != 4 or not np.all(np.isfinite(orientation)) or np.linalg.norm(orientation) == 0.0:
        return 0.0, 0.0, 0.0

    orientation = orientation.reshape(4)
    x, y, z, w = tuple(orientation)
    roll, pitch, yaw = transform_utils.quaternion_to_euler_angles(np.array([w, x, y, z])).numpy()

    return normalize_angle(roll), normalize_angle(pitch), normalize_angle(yaw)
