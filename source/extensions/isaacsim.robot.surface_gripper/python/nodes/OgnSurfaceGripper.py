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

"""OmniGraph node for toggling surface gripper open and close states."""

import omni.graph.core as og
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper


class OgnSurfaceGripper:
    """OmniGraph node that opens, closes, or toggles a surface gripper.

    The node exposes three execution pins on its input side. Their precedence
    within a single compute call is:

    - ``Close`` -- force the gripper closed (no-op if already closed).
    - ``Open`` -- force the gripper open (no-op if already open).
    - ``Toggle`` -- flip the gripper between open and closed based on current status.

    Precedence matters only when multiple pins fire in the same tick: an
    explicit ``Open``/``Close`` pulse always wins over a simultaneous
    ``Toggle``, so a downstream graph that routes both through the same
    node gets deterministic behaviour.
    """

    @staticmethod
    def compute(db) -> bool:
        """Compute the gripper action for the pin that fired this tick."""
        if not db.inputs.enabled or len(db.inputs.SurfaceGripper) == 0:
            return True

        input_prim = db.inputs.SurfaceGripper[0].pathString
        gripper_interface = surface_gripper.acquire_surface_gripper_interface()

        enabled = og.ExecutionAttributeState.ENABLED
        close_fired = db.inputs.Close == enabled
        open_fired = db.inputs.Open == enabled
        toggle_fired = db.inputs.Toggle == enabled

        if close_fired:
            gripper_interface.close_gripper(input_prim)
            return True
        if open_fired:
            gripper_interface.open_gripper(input_prim)
            return True
        if toggle_fired:
            status = gripper_interface.get_gripper_status(input_prim)
            if status == surface_gripper.GripperStatus.Open:
                gripper_interface.close_gripper(input_prim)
            else:
                gripper_interface.open_gripper(input_prim)
        return True
