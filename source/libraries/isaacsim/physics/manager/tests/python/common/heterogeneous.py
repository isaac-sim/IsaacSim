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

"""Exercise heterogeneous articulation views across replicated environments.

Each environment contains cabinet and Franka articulations. The scenarios
verify view separation and consistent degrees of freedom, Jacobians, and
dynamics compensation across environment replicas.
"""

from __future__ import annotations

import os
import sys
from typing import Protocol

import pytest

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

import warp_utils as wp_utils  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
    get_asset_root,
)
from pxr import Gf  # noqa: E402


class _ArticulationFactory(Protocol):
    def create_articulation_view(self, pattern: str) -> object: ...


class HeterogeneousSceneArticulationsCommon(GridTestBase):
    """Scenario with cabinet and Franka articulations in each environment.

    The scenario skips when either required test asset is unavailable.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Missing fixture assets are classified explicitly here. Runtime
        # operation and adapter failures are not converted into skips.
        cabinet_asset = os.path.join(get_asset_root(), "cabinet.usda")
        if not os.path.isfile(cabinet_asset):
            pytest.skip(f"asset not bundled with umbrella: {cabinet_asset}")
        franka_asset = os.path.join(get_asset_root(), "franka.usda")
        if not os.path.isfile(franka_asset):
            pytest.skip(f"asset not bundled with umbrella: {franka_asset}")

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cabinet"),
            Transform((0.0, 0.0, 1.0)),
            cabinet_asset,
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("franka"),
            Transform((0.0, 0.0, 1.0)),
            franka_asset,
        )

    def on_start(self, sim: _ArticulationFactory) -> None:
        """Create cabinet and Franka views for every replicated environment.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cabinet = sim.create_articulation_view("/envs/*/cabinet")
        self.franka = sim.create_articulation_view("/envs/*/franka")
        self.all_indices = wp_utils.arange(self.cabinet.count, device=self.wp_device)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Compare environment-local link transforms after one step.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            # `link-transforms` returns world-frame poses; envs sit on
            # a grid (env_spacing=2.5) so absolute positions never
            # match. Subtract each env's root-link position from its
            # links so the comparison runs in env-local frame, which
            # is what the original consistency check intends.
            p_cabinet = (
                self.cabinet.get_data("link-transforms")
                .numpy()
                .reshape(self.cabinet.count, self.cabinet.get_metadata("num-links"), 7)
            )
            p_franka = (
                self.franka.get_data("link-transforms")
                .numpy()
                .reshape(self.franka.count, self.franka.get_metadata("num-links"), 7)
            )
            p_cab_local = p_cabinet.copy()
            p_cab_local[..., :3] -= p_cabinet[:, 0:1, :3]
            p_franka_local = p_franka.copy()
            p_franka_local[..., :3] -= p_franka[:, 0:1, :3]
            assert wp_utils.wp_allclose(
                p_cab_local[0], p_cab_local, rtol=1e-3, atol=1e-3
            ), "cabinet link transforms consistent across envs (env-local frame)"
            assert wp_utils.wp_allclose(
                p_franka_local[0], p_franka_local, rtol=1e-3, atol=1e-3
            ), "franka link transforms consistent across envs (env-local frame)"
            self.finish()


class HeterogeneousBaseDynamicsCommon(GridTestBase):
    """Scenario that compares Cartpole and Ant dynamics across replicas.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole"),
            Transform((0.0, 0.0, 5.0)),
            os.path.join(get_asset_root(), "CartPole.usda"),
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("ant"),
            Transform((0.0, 0.0, -5.0)),
            os.path.join(get_asset_root(), "Ant.usda"),
        )

    def on_start(self, sim: _ArticulationFactory) -> None:
        """Create Cartpole and Ant articulation views.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Compare state and dynamics tensors at scheduled simulation steps.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 3:
            p_cartpoles = (
                self.cartpoles.get_data("dof-positions")
                .numpy()
                .reshape(self.cartpoles.count, self.cartpoles.get_metadata("num-dofs"))
            )
            p_ants = (
                self.ants.get_data("dof-positions").numpy().reshape(self.ants.count, self.ants.get_metadata("num-dofs"))
            )
            p_cartpoles = (p_cartpoles + 0.5).astype("float32")
            p_ants = (p_ants + 0.1).astype("float32")
            self.cartpoles.set_data("dof-positions", self.to_warp(p_cartpoles), self.all_indices)
            self.ants.set_data("dof-positions", self.to_warp(p_ants), self.all_indices)

        if stepno == 4:
            p_cartpoles = (
                self.cartpoles.get_data("dof-positions")
                .numpy()
                .reshape(self.cartpoles.count, self.cartpoles.get_metadata("num-dofs"))
            )
            p_ants = (
                self.ants.get_data("dof-positions").numpy().reshape(self.ants.count, self.ants.get_metadata("num-dofs"))
            )
            assert wp_utils.wp_allclose(
                p_cartpoles[0:1].repeat(self.cartpoles.count, axis=0), p_cartpoles, rtol=1e-3, atol=1e-3
            ), "cartpole DOF positions consistent across envs"
            assert wp_utils.wp_allclose(
                p_ants[0:1].repeat(self.ants.count, axis=0), p_ants, rtol=1e-3, atol=1e-3
            ), "ant DOF positions consistent across envs"

        if stepno == 7:
            j_cartpoles = self.cartpoles.get_data("jacobians").numpy().reshape(self.cartpoles.count, -1)
            j_ants = self.ants.get_data("jacobians").numpy().reshape(self.ants.count, -1)
            assert wp_utils.wp_allclose(
                j_cartpoles[0:1].repeat(self.cartpoles.count, axis=0), j_cartpoles, rtol=1e-3, atol=1e-3
            ), "cartpole Jacobians consistent across envs"
            assert wp_utils.wp_allclose(
                j_ants[0:1].repeat(self.ants.count, axis=0), j_ants, rtol=1e-3, atol=1e-3
            ), "ant Jacobians consistent across envs"

        if stepno == 8:
            cc_cartpoles = (
                self.cartpoles.get_data("coriolis-and-centrifugal-compensation-forces")
                .numpy()
                .reshape(self.cartpoles.count, -1)
            )
            cc_ants = (
                self.ants.get_data("coriolis-and-centrifugal-compensation-forces").numpy().reshape(self.ants.count, -1)
            )
            assert wp_utils.wp_allclose(
                cc_cartpoles[0:1].repeat(self.cartpoles.count, axis=0), cc_cartpoles, rtol=1e-3, atol=1e-3
            ), "cartpole Coriolis/Centrifugal forces consistent across envs"
            assert wp_utils.wp_allclose(
                cc_ants[0:1].repeat(self.ants.count, axis=0), cc_ants, rtol=1e-3, atol=1e-3
            ), "ant Coriolis/Centrifugal forces consistent across envs"

        if stepno == 9:
            g_cartpoles = (
                self.cartpoles.get_data("gravity-compensation-forces").numpy().reshape(self.cartpoles.count, -1)
            )
            g_ants = self.ants.get_data("gravity-compensation-forces").numpy().reshape(self.ants.count, -1)
            assert wp_utils.wp_allclose(
                g_cartpoles[0:1].repeat(self.cartpoles.count, axis=0), g_cartpoles, rtol=1e-3, atol=1e-3
            ), "cartpole gravity compensation forces consistent across envs"
            assert wp_utils.wp_allclose(
                g_ants[0:1].repeat(self.ants.count, axis=0), g_ants, rtol=1e-3, atol=1e-3
            ), "ant gravity compensation forces consistent across envs"

        if stepno == 10:
            mm_cartpoles = (
                self.cartpoles.get_data("generalized-mass-matrices").numpy().reshape(self.cartpoles.count, -1)
            )
            mm_ants = self.ants.get_data("generalized-mass-matrices").numpy().reshape(self.ants.count, -1)
            assert wp_utils.wp_allclose(
                mm_cartpoles[0:1].repeat(self.cartpoles.count, axis=0), mm_cartpoles, rtol=1e-3, atol=1e-3
            ), "cartpole mass matrices consistent across envs"
            assert wp_utils.wp_allclose(
                mm_ants[0:1].repeat(self.ants.count, axis=0), mm_ants, rtol=1e-3, atol=1e-3
            ), "ant mass matrices consistent across envs"

        if stepno == 11:
            self.finish()
