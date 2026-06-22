# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Write randomized values into the registered physics simulation context."""

from typing import Any

import carb
import numpy as np
import omni.graph.core as og
from isaacsim.replicator.experimental.domain_randomization import SIMULATION_CONTEXT_ATTRIBUTES
from isaacsim.replicator.experimental.domain_randomization import physics_view as physics

OPERATION_TYPES = ["direct", "additive", "scaling"]


def apply_randomization_operation(operation: Any, attribute_name: Any, samples: Any, on_reset: Any) -> Any:
    """Return the simulation-context value to write for the requested operation.

    Args:
        operation: Type of operation ("direct", "additive", or "scaling").
        attribute_name: Name of the attribute being randomized.
        samples: Sample values to apply.
        on_reset: Whether this is called during a reset operation. Reset writes
            return the stored reset value without applying the current sample.

    Returns:
        The direct sample, additive/scaled value relative to the stored reset
        value, or the stored reset value when ``on_reset`` is true.
    """
    if on_reset:
        return physics._simulation_context_reset_values[attribute_name]
    if operation == "additive":
        return physics._simulation_context_reset_values[attribute_name] + samples
    elif operation == "scaling":
        return physics._simulation_context_reset_values[attribute_name] * samples
    else:
        return samples


def modify_initial_values(operation: Any, attribute_name: Any, samples: Any) -> None:
    """Update the stored reset value for a simulation-context attribute.

    Args:
        operation: Type of operation ("direct", "additive", or "scaling").
        attribute_name: Name of the attribute being modified.
        samples: Sample values to use for modification.
    """
    if operation == "additive":
        physics._simulation_context_reset_values[attribute_name] = (
            physics._simulation_context_initial_values[attribute_name] + samples
        )
    elif operation == "scaling":
        physics._simulation_context_reset_values[attribute_name] = (
            physics._simulation_context_initial_values[attribute_name] * samples
        )
    else:
        physics._simulation_context_reset_values[attribute_name] = samples


class OgnWritePhysicsSimulationContext:
    """OmniGraph writer for global physics simulation-context attributes."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Apply a sampled global physics value.

        The node expects an attribute from ``SIMULATION_CONTEXT_ATTRIBUTES`` and
        currently writes the first sampled gravity vector through the registered
        physics simulation view. Empty indices keep ``execOut`` enabled but
        perform no write. Invalid context state, attributes, or operations log
        an error, disable ``execOut``, and return ``False``.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True when the simulation-context value is written, False when inputs
            are empty or invalid.
        """
        attribute_name = db.inputs.attribute
        operation = db.inputs.operation
        values = db.inputs.values

        if db.inputs.indices is None or len(db.inputs.indices) == 0:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return False
        indices = np.array(db.inputs.indices)
        on_reset = db.inputs.on_reset

        try:
            physics_sim_view = physics.resolve_physics_sim_view()
            if physics_sim_view is None:
                raise ValueError("Expected a registered simulation context with physics_sim_view")
            if attribute_name not in SIMULATION_CONTEXT_ATTRIBUTES:
                raise ValueError(
                    f"Expected an attribute in {SIMULATION_CONTEXT_ATTRIBUTES}, but instead received {attribute_name}"
                )
            if operation not in OPERATION_TYPES:
                raise ValueError(f"Expected an operation type in {OPERATION_TYPES}, but instead received {operation}")

            # Gravity is global (one physics scene), so use first sample only
            samples = np.array(values).reshape(len(indices), -1)[0]
        except Exception as error:
            db.log_error(f"WritePhysics Error: {error}")
            db.outputs.execOut = og.ExecutionAttributeState.DISABLED
            return False

        if on_reset:
            modify_initial_values(operation, attribute_name, samples)

        if attribute_name == "gravity":
            gravity = apply_randomization_operation(operation, attribute_name, samples, on_reset)
            physics_sim_view.set_gravity(carb.Float3(gravity[0], gravity[1], gravity[2]))

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
