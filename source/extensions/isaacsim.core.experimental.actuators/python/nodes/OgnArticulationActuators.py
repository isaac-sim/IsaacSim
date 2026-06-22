# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""OmniGraph node wrapping ArticulationActuators."""

from __future__ import annotations

import traceback
from typing import TYPE_CHECKING, Any

import numpy as np
from isaacsim.core.experimental.actuators.ogn.OgnArticulationActuatorsDatabase import OgnArticulationActuatorsDatabase

if TYPE_CHECKING:
    from isaacsim.core.experimental.actuators import ArticulationActuators


class OgnArticulationActuatorsInternalState:
    """Per-instance state for the ArticulationActuators OmniGraph node."""

    def __init__(self) -> None:
        self._robot_path: str | None = None
        self._auto_step_pre_physics: bool | None = None
        self._actuators: ArticulationActuators | None = None

    @property
    def initialized(self) -> bool:
        """Report whether the inner `ArticulationActuators` has been constructed.

        Returns:
            ``True`` once `initialize()` has built the inner wrapper, ``False`` otherwise.
        """
        return self._actuators is not None

    def initialize(self, robot_path: str, auto_step_pre_physics: bool) -> None:
        """Construct the inner `ArticulationActuators` for the given `robot_path`.

        Args:
            robot_path: USD path of the articulation root prim.
            auto_step_pre_physics: Forwarded to `ArticulationActuators.__init__`.
        """
        from isaacsim.core.experimental.actuators import ArticulationActuators

        self._robot_path = robot_path
        self._auto_step_pre_physics = auto_step_pre_physics
        self._actuators = ArticulationActuators(robot_path, auto_step_pre_physics=auto_step_pre_physics)

    def release(self) -> None:
        """Tear down the inner `ArticulationActuators` and clear cached inputs.

        Closes the wrapped `ArticulationActuators` (deregistering its
        `SimulationManager` callbacks) and resets the cached `robot_path` and
        `auto_step_pre_physics` so the next `compute()` triggers a fresh
        `initialize()`. Safe to call multiple times.
        """
        if self._actuators is not None:
            self._actuators.close()
            self._actuators = None
        self._robot_path = None
        self._auto_step_pre_physics = None


class OgnArticulationActuators:
    """OmniGraph node that wraps ArticulationActuators.

    Lazy-initializes the underlying ArticulationActuators on the first execIn
    pulse.  Re-initializes automatically when robotPath changes.  Dynamically
    enables or disables the pre-physics callback when autoStepPrePhysics changes
    after initialization.
    """

    @staticmethod
    def internal_state() -> OgnArticulationActuatorsInternalState:
        """Return a new per-instance node state.

        Returns:
            A fresh `OgnArticulationActuatorsInternalState` for one node instance.
        """
        return OgnArticulationActuatorsInternalState()

    @staticmethod
    def release_instance(node: object, graph_instance_id: int) -> None:
        """Tear down the per-graph-instance state when the node is removed.

        Called by OmniGraph when this node's graph instance is destroyed
        (graph teardown, target removal, or node deletion). Releases the inner
        `ArticulationActuators` so its `SimulationManager` callbacks are
        deregistered deterministically rather than relying on garbage collection.

        Args:
            node: The OmniGraph node being released.
            graph_instance_id: Identifier of the graph instance being released.
        """
        try:
            state = OgnArticulationActuatorsDatabase.get_internal_state(node, graph_instance_id)
        except Exception:  # noqa: BLE001 - best-effort during graph teardown
            state = None
        if state is not None:
            state.release()

    @staticmethod
    def compute(db: Any) -> bool:
        """Compute actuator commands from the current OmniGraph inputs.

        Args:
            db: OmniGraph database object for the current compute call.

        Returns:
            True if commands were processed successfully, otherwise False.
        """
        state: OgnArticulationActuatorsInternalState = db.per_instance_state
        try:
            robot_path: str = db.inputs.robotPath
            if not robot_path:
                db.log_error("robotPath must be set")
                return False

            auto_step: bool = db.inputs.autoStepPrePhysics

            # (Re-)initialize when the robot path changes or on first compute.
            if not state.initialized or state._robot_path != robot_path:
                state.release()
                state.initialize(robot_path, auto_step)
            elif state._auto_step_pre_physics != auto_step:
                # Reflect a runtime change to autoStepPrePhysics.
                if auto_step:
                    state._actuators.enable_auto_step_pre_physics()
                else:
                    state._actuators.disable_auto_step_pre_physics()
                state._auto_step_pre_physics = auto_step

            # Apply feedforward command when provided.
            feedforward = db.inputs.feedforwardCommand
            if len(feedforward) > 0:
                indices_in = db.inputs.indices
                dof_indices_in = db.inputs.dofIndices
                state._actuators.set_dof_feedforward_effort_targets(
                    np.asarray(feedforward, dtype=np.float32),
                    indices=np.asarray(indices_in, dtype=np.int32) if len(indices_in) > 0 else None,
                    dof_indices=np.asarray(dof_indices_in, dtype=np.int32) if len(dof_indices_in) > 0 else None,
                )

            # Step manually when auto-stepping is disabled.
            if not auto_step:
                state._actuators.step_actuators(db.inputs.stepDt)

        except (
            Exception
        ) as error:  # noqa: BLE001 - log everything (incl. programming bugs) at error level with traceback
            db.log_error(
                f"OgnArticulationActuators.compute failed with "
                f"{type(error).__name__}: {error}\n{traceback.format_exc()}"
            )
            return False

        return True
