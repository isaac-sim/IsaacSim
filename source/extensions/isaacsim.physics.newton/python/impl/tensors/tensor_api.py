# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Newton Tensor API for Isaac Sim.

This module provides tensor-based interfaces for Newton physics simulation,
compatible with different tensor frontend frameworks (NumPy, PyTorch, Warp).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .articulation_view import NewtonArticulationView
from .backend import NewtonSimView
from .rigid_body_view import NewtonRigidBodyView
from .rigid_contact_view import NewtonRigidContactView

if TYPE_CHECKING:
    from ..newton_stage import NewtonStage
    from .frontends import NumpyFrontend, TorchFrontend, WarpFrontend


def create_simulation_view(frontend_name: str, newton_stage: NewtonStage, stage_id: int = -1) -> NewtonSimulationView:
    """Create a simulation view with the specified tensor frontend.

    Args:
        frontend_name: Name of the frontend framework.
        newton_stage: The Newton stage object from isaacsim.physics.newton.
        stage_id: USD stage ID (currently unused, kept for API compatibility).

    Returns:
        Simulation view object with the specified frontend.

    Raises:
        Exception: If backend creation fails or invalid frontend name is provided.
    """
    backend = NewtonSimView(newton_stage)
    if backend is None:
        raise Exception("Failed to create simulation view backend")

    device_ordinal = backend.device_ordinal

    frontend_id = frontend_name.lower()
    if frontend_id == "numpy" or frontend_id == "np":
        if device_ordinal == -1:
            try:
                from omni.physics.tensors.frontend_np import FrontendNumpy

                frontend = FrontendNumpy()
                return NewtonSimulationView(backend, frontend)
            except ImportError:
                raise Exception("NumPy frontend not available")
        else:
            raise Exception("The Numpy frontend cannot be used with GPU pipelines")

    elif frontend_id == "torch" or frontend_id == "pytorch":
        try:
            from omni.physics.tensors.frontend_torch import FrontendTorch

            frontend = FrontendTorch(device_ordinal)
            return NewtonSimulationView(backend, frontend)
        except ImportError:
            raise Exception("PyTorch frontend not available")

    elif frontend_id == "warp" or frontend_id == "wp":
        try:
            from omni.physics.tensors.frontend_warp import FrontendWarp

            frontend = FrontendWarp(device_ordinal)
            return NewtonSimulationView(backend, frontend)
        except ImportError:
            raise Exception("Warp frontend not available")

    else:
        raise Exception(f"Unrecognized frontend name '{frontend_name}'")


class NewtonSimulationView:
    """Simulation view for Newton physics with tensor interface.

    This class provides a unified interface to Newton physics simulation
    with support for different tensor frameworks.

    Args:
        backend: NewtonSimView backend instance.
        frontend: Tensor framework frontend (NumPy, PyTorch, or Warp).
    """

    def __init__(self, backend: NewtonSimView, frontend: "NumpyFrontend | TorchFrontend | WarpFrontend") -> None:
        self._backend = backend
        self._frontend = frontend

    def create_articulation_view(self, pattern: str | list[str]) -> NewtonArticulationView:
        """Create a view for articulations matching the pattern.

        Args:
            pattern: Path pattern or list of paths to articulation roots.

        Returns:
            View object for the matching articulations.
        """
        return NewtonArticulationView(self._backend.create_articulation_view(pattern), self._frontend)

    def create_rigid_body_view(self, pattern: str | list[str]) -> NewtonRigidBodyView:
        """Create a view for rigid bodies matching the pattern.

        Args:
            pattern: Path pattern or list of paths to rigid bodies.

        Returns:
            View object for the matching rigid bodies.
        """
        return NewtonRigidBodyView(self._backend.create_rigid_body_view(pattern), self._frontend)

    def create_rigid_contact_view(
        self, pattern: str | list[str], filter_patterns: list[list[str]] | None = None, max_contact_data_count: int = 0
    ) -> NewtonRigidContactView:
        """Create a view for contact sensors matching the pattern.

        Args:
            pattern: Path pattern or list of paths to bodies with contact sensors.
            filter_patterns: Optional list of filter patterns for each sensor.
            max_contact_data_count: Maximum number of contact data points to track.

        Returns:
            View object for the matching contact sensors.
        """
        if filter_patterns is None:
            filter_patterns = []
        return NewtonRigidContactView(
            self._backend.create_rigid_contact_view(pattern, filter_patterns, max_contact_data_count), self._frontend  # type: ignore[arg-type]
        )

    def invalidate(self) -> None:
        """Invalidate the simulation view.

        Called when the simulation is stopped to clean up resources.
        """
        if hasattr(self._backend, "invalidate"):
            self._backend.invalidate()

    def is_valid(self) -> bool:
        """Check if the simulation view is valid.

        Returns:
            True if the simulation view is valid.
        """
        return self._backend.is_valid()

    def set_subspace_roots(self, pattern: str | list[str]) -> bool:
        """Set subspace roots for the simulation view.

        Args:
            pattern: Path pattern for subspace roots.

        Returns:
            True if successful.
        """
        if hasattr(self._backend, "set_subspace_roots"):
            return self._backend.set_subspace_roots(pattern)
        return True
