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

"""Wheeled robot using core.experimental.prims.Articulation."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import Articulation


class WheeledRobot(Articulation):
    """Wheeled robot wrapper built on the experimental Articulation API.

    Args:
        paths: USD prim path for the robot articulation.
        wheel_dof_names: Joint names identifying the wheel DOFs.
        wheel_dof_indices: Joint indices identifying the wheel DOFs.
        usd_path: Optional USD asset path to reference onto the stage.
        positions: Initial world position(s) as [x, y, z].
        orientations: Initial world orientation(s) as [w, x, y, z] quaternion.

    Raises:
        ValueError: If neither wheel_dof_names nor wheel_dof_indices is specified.
    """

    def __init__(
        self,
        paths: str,
        *,
        wheel_dof_names: list[str] | None = None,
        wheel_dof_indices: list[int] | None = None,
        usd_path: str | None = None,
        positions: list | np.ndarray | None = None,
        orientations: list | np.ndarray | None = None,
    ) -> None:
        if wheel_dof_names is None and wheel_dof_indices is None:
            raise ValueError("WheeledRobot requires either wheel_dof_names or wheel_dof_indices")
        self._wheel_dof_names = wheel_dof_names
        self._wheel_dof_indices_input = wheel_dof_indices
        self._wheel_dof_indices_resolved: list[int] | None = None

        if usd_path is not None:
            stage_utils.add_reference_to_stage(usd_path=usd_path, path=paths)

        reset_xform = positions is not None or orientations is not None
        super().__init__(
            paths,
            positions=positions,
            orientations=orientations,
            reset_xform_op_properties=reset_xform,
        )

    def _resolve_wheel_dof_indices(self) -> list[int]:
        """Resolve and cache the DOF indices for the wheel joints.

        Returns:
            List of DOF indices for the wheel joints.
        """
        if self._wheel_dof_indices_resolved is not None:
            return self._wheel_dof_indices_resolved
        if self._wheel_dof_indices_input is not None:
            self._wheel_dof_indices_resolved = list(self._wheel_dof_indices_input)
            return self._wheel_dof_indices_resolved
        indices_wp = self.get_dof_indices(self._wheel_dof_names)
        self._wheel_dof_indices_resolved = indices_wp.numpy().tolist()
        return self._wheel_dof_indices_resolved

    def apply_wheel_actions(self, velocities: np.ndarray) -> None:
        """Apply angular velocity targets to the wheel joints.

        Args:
            velocities: Target angular velocities, one per wheel.

        Raises:
            ValueError: If the length of `velocities` does not match the number of wheels.
        """
        dof_indices = self._resolve_wheel_dof_indices()
        if len(velocities) != len(dof_indices):
            raise ValueError(f"velocities length ({len(velocities)}) must match number of wheels ({len(dof_indices)})")
        velocities = np.asarray(velocities, dtype=np.float32)
        self.set_dof_velocity_targets(velocities, dof_indices=dof_indices)
