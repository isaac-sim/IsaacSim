# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide engine-neutral deformable-material tensor scenarios."""

from __future__ import annotations

import os
import sys

import isaacsim.physics.manager.impl.tensors as tensors
import numpy as np
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
    get_asset_root,
)
from isaacsim.physics.manager.impl.tensors import EntityView  # noqa: E402


class DeformableMaterialPropertiesCommon(GridTestBase):
    """Validate every deformable-material property exposed by OvPhysX.

    Args:
        test_case: Test case that owns the scenario.
        device_params: Simulation device parameters.

    """

    MATERIAL_PATH = "/envs/*/Asset/DeformableMaterial"
    EXPECTED_PROPERTIES = {
        "youngs-modulus": 500000.0,
        "poissons-ratio": 0.49,
        "dynamic-friction": 0.5,
        "elasticity-damping": 0.02,
        "bending-stiffness": 100.0,
        "thickness": 0.01,
        "bending-damping": 0.05,
    }

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0
        sim_params.add_default_ground = False
        super().__init__(test_case, GridParams(num_envs=1, env_spacing=2.5), sim_params, device_params)
        self.maxsteps = 3

        asset_path = os.path.join(get_asset_root(), "SurfaceDeformable.usda")
        actor_path = self.env_template_path.AppendChild("Asset")
        self.create_actor_from_asset(actor_path, Transform(), asset_path)
        self.view: EntityView

    def on_start(self, sim: SimulationEntities) -> None:
        """Create and validate the deformable-material view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.view = sim.create_deformable_material_view(self.MATERIAL_PATH)
        self.check_deformable_material_view(self.view, 1)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Round-trip material properties on the first physics step.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step.
            dt: Simulation step duration.

        """
        if stepno != 0:
            return

        for property_name, expected in self.EXPECTED_PROPERTIES.items():
            self._assert_property_round_trip(property_name, expected)

        assert not self.view.has_impl(
            "density", tensors.ImplKind.Get
        ), "density is not part of the PhysX deformable-material tensor API"
        self.finish()

    def _assert_property_round_trip(self, property_name: str, expected: float) -> None:
        """Validate an authored material property and its indexed write path.

        Args:
            property_name: Tensor implementation name.
            expected: Authored scalar value.

        """
        assert self.view.has_impl(property_name, tensors.ImplKind.Get), f"missing GET impl {property_name!r}"
        assert self.view.has_impl(property_name, tensors.ImplKind.Set), f"missing SET impl {property_name!r}"

        value = self.view.get_data(property_name)
        assert value is not None, f"{property_name} returned no data"
        assert value.size == 1, f"{property_name} must expose one scalar per material"
        assert np.allclose(
            value.numpy(), [expected], rtol=1.0e-5, atol=1.0e-5
        ), f"{property_name} authored value mismatch"

        target = np.array([expected * 0.9], dtype=np.float32)
        source = self.to_warp(target)
        indices = wp.array([0], dtype=wp.int32, device=self.wp_device)
        self.view.set_data(property_name, source, indices)
        readback = self.view.get_data(property_name).numpy()
        assert np.allclose(
            readback, target, rtol=1.0e-5, atol=1.0e-5
        ), f"{property_name} indexed write did not round-trip"
