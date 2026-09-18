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

"""Exercise articulation invariance under swapped joint body ordering.

The scenarios create paired CartPole articulations with reversed joint body
relationships and compare their tensor properties. Backend-specific subclasses
can add schemas through ``_apply_engine_specifics`` without changing the shared
assertions.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

import warp_utils as wp_utils  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
    get_asset_root,
)
from pxr import Gf, UsdPhysics  # noqa: E402

# ---------------------------------------------------------------------------
# Common base — stock UsdPhysics only.
# ---------------------------------------------------------------------------


class JointBodyOrderCommon(GridTestBase):
    """Build paired CartPole articulations with opposite joint body ordering.

    The second CartPole reverses each joint's body relationships and swaps the
    pole joint's local frames. Both copies receive identical drive parameters.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole1"),
            Transform((0.0, 0.0, 1.0)),
            asset_path,
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("cartpole2"),
            Transform((0.0, 0.0, 3.0)),
            asset_path,
        )

        self.atol = 1e-05

        for i in range(self.num_envs):
            cart_joint = UsdPhysics.RevoluteJoint.Get(self.stage, f"/envs/env{i}/cartpole2/cartJoint")
            root_path = cart_joint.CreateBody0Rel().GetTargets()[0]
            child_path = cart_joint.CreateBody1Rel().GetTargets()[0]
            cart_joint.CreateBody0Rel().SetTargets([child_path])
            cart_joint.CreateBody1Rel().SetTargets([root_path])

            pole_joint = UsdPhysics.PrismaticJoint.Get(self.stage, f"/envs/env{i}/cartpole2/poleJoint")
            root_path = pole_joint.CreateBody0Rel().GetTargets()[0]
            child_path = pole_joint.CreateBody1Rel().GetTargets()[0]
            pole_joint.CreateBody0Rel().SetTargets([child_path])
            pole_joint.CreateBody1Rel().SetTargets([root_path])

            local_pos_0 = pole_joint.GetLocalPos0Attr().Get()
            local_pos_1 = pole_joint.GetLocalPos1Attr().Get()
            pole_joint.CreateLocalPos1Attr().Set(local_pos_0)
            pole_joint.CreateLocalPos0Attr().Set(local_pos_1)

            local_rot_0 = pole_joint.GetLocalRot0Attr().Get()
            local_rot_1 = pole_joint.GetLocalRot1Attr().Get()
            pole_joint.CreateLocalRot1Attr().Set(local_rot_0)
            pole_joint.CreateLocalRot0Attr().Set(local_rot_1)

        self._apply_drive_api()
        self._apply_engine_specifics()

    def _apply_drive_api(self) -> None:
        for i in range(self.num_envs):
            for env in (1, 2):
                drive = UsdPhysics.DriveAPI.Apply(
                    self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/cartJoint"),
                    "linear",
                )
                drive.CreateTargetPositionAttr(1.0)
                drive.CreateTargetVelocityAttr(2.0)
                drive.CreateStiffnessAttr(800.0)
                drive.CreateDampingAttr(50.0)

                drive = UsdPhysics.DriveAPI.Apply(
                    self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env}/poleJoint"),
                    "angular",
                )
                drive.CreateTargetPositionAttr(90.0)
                drive.CreateTargetVelocityAttr(10.0)
                drive.CreateStiffnessAttr(800.0)
                drive.CreateDampingAttr(50.0)

    def _apply_engine_specifics(self) -> None:
        """Apply backend-specific schemas when a subclass requires them."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create views for both articulation variants.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cartpoles_1 = sim.create_articulation_view("/envs/*/cartpole1")
        self.cartpoles_2 = sim.create_articulation_view("/envs/*/cartpole2")
        self.cartpoles_1_indices = wp_utils.arange(self.cartpoles_1.count, device=self.wp_device)
        self.cartpoles_2_indices = wp_utils.arange(self.cartpoles_2.count, device=self.wp_device)
        self.check_articulation_view(self.cartpoles_1, self.num_envs, 3, 2, True)
        self.check_articulation_view(self.cartpoles_2, self.num_envs, 3, 2, True)


# ---------------------------------------------------------------------------
# Per-property scenario mixins.
# ---------------------------------------------------------------------------


class _LimitsExtraSetup:
    """Author identical lower and upper limits on every pole joint."""

    def _apply_extra_init(self) -> None:
        for i in range(self.num_envs):
            for env in (1, 2):
                pole_joint = UsdPhysics.RevoluteJoint.Get(self.stage, f"/envs/env{i}/cartpole{env}/poleJoint")
                pole_joint.CreateLowerLimitAttr().Set(-2)
                pole_joint.CreateUpperLimitAttr().Set(3)


class JointBodyOrderLimitsCommon(_LimitsExtraSetup, JointBodyOrderCommon):
    """Compare joint limits across reversed joint body relationships.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        self._apply_extra_init()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare degree-of-freedom limits after simulation starts.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 5:
            num_dof = 2
            l1 = self.cartpoles_1.get_data("dof-limits").numpy().reshape(self.cartpoles_1.count, num_dof, 2)
            l2 = self.cartpoles_2.get_data("dof-limits").numpy().reshape(self.cartpoles_2.count, num_dof, 2)
            assert wp_utils.wp_allclose(
                l1, l2, rtol=1e-3, atol=self.atol
            ), "swapped-body-order articulations should report identical dof limits"
            self.finish()


def _make_property_step(
    impl: str,
    label: str,
) -> Callable[[JointBodyOrderCommon, SimulationEntities, int, float], None]:
    def on_physics_step(self: JointBodyOrderCommon, sim: SimulationEntities, stepno: int, dt: float) -> None:
        if stepno == 5:
            num_dof = 2
            v1 = self.cartpoles_1.get_data(impl).numpy().reshape(self.cartpoles_1.count, num_dof)
            v2 = self.cartpoles_2.get_data(impl).numpy().reshape(self.cartpoles_2.count, num_dof)
            assert wp_utils.wp_allclose(
                v1, v2, rtol=1e-3, atol=self.atol
            ), f"swapped-body-order articulations should report identical {label}"
            self.finish()

    return on_physics_step


class JointBodyOrderPositionCommon(JointBodyOrderCommon):
    """Compare joint positions across reversed joint body relationships."""

    on_physics_step = _make_property_step("dof-positions", "dof positions")


class JointBodyOrderVelocityCommon(JointBodyOrderCommon):
    """Compare joint velocities across reversed joint body relationships."""

    on_physics_step = _make_property_step("dof-velocities", "dof velocities")


class JointBodyOrderPositionTargetCommon(JointBodyOrderCommon):
    """Compare position targets across reversed joint body relationships."""

    on_physics_step = _make_property_step("dof-position-targets", "position targets")


class JointBodyOrderVelocityTargetCommon(JointBodyOrderCommon):
    """Compare velocity targets across reversed joint body relationships."""

    on_physics_step = _make_property_step("dof-velocity-targets", "velocity targets")


class JointBodyOrderDofForceCommon(JointBodyOrderCommon):
    """Compare degree-of-freedom force tensors across joint body orderings.

    The scenario checks actuation, gravity-compensation, and
    Coriolis-and-centrifugal compensation forces before and after an actuation
    force write.
    """

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare force tensors and finish after the fourth step.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 4:
            num_dof = 2
            f1 = self.cartpoles_1.get_data("dof-actuation-forces").numpy().reshape(self.cartpoles_1.count, num_dof)
            f2 = self.cartpoles_2.get_data("dof-actuation-forces").numpy().reshape(self.cartpoles_2.count, num_dof)
            assert wp_utils.wp_allclose(
                f1, f2, rtol=1e-3, atol=1e-5
            ), "actuation forces consistent across body0/body1-swapped cartpoles"

            submitted_1 = (f1 + 1).astype("float32")
            submitted_2 = (f2 + 1).astype("float32")
            self.cartpoles_1.set_data("dof-actuation-forces", self.to_warp(submitted_1), self.cartpoles_1_indices)
            self.cartpoles_2.set_data("dof-actuation-forces", self.to_warp(submitted_2), self.cartpoles_2_indices)

            new_f1 = self.cartpoles_1.get_data("dof-actuation-forces").numpy().reshape(self.cartpoles_1.count, num_dof)
            new_f2 = self.cartpoles_2.get_data("dof-actuation-forces").numpy().reshape(self.cartpoles_2.count, num_dof)
            assert wp_utils.wp_allclose(
                new_f1, new_f2, rtol=1e-3, atol=1e-5
            ), "after-set actuation forces remain consistent"

            g1 = (
                self.cartpoles_1.get_data("gravity-compensation-forces")
                .numpy()
                .reshape(self.cartpoles_1.count, num_dof)
            )
            g2 = (
                self.cartpoles_2.get_data("gravity-compensation-forces")
                .numpy()
                .reshape(self.cartpoles_2.count, num_dof)
            )
            assert wp_utils.wp_allclose(g1, g2, rtol=1e-3, atol=1e-5), "gravity compensation forces consistent"

            c1 = (
                self.cartpoles_1.get_data("coriolis-and-centrifugal-compensation-forces")
                .numpy()
                .reshape(self.cartpoles_1.count, num_dof)
            )
            c2 = (
                self.cartpoles_2.get_data("coriolis-and-centrifugal-compensation-forces")
                .numpy()
                .reshape(self.cartpoles_2.count, num_dof)
            )
            assert wp_utils.wp_allclose(
                c1, c2, rtol=1e-3, atol=1e-5
            ), "coriolis/centrifugal compensation forces consistent"

            self.finish()


class JointBodyOrderLinkForceCommon(JointBodyOrderCommon):
    """Compare projected joint and incoming link forces across body orderings.

    Drive targets on the reversed articulation use opposite signs to compensate
    for its joint orientation.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    target_p_pole = 45.0
    target_p_cart = 1.0

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, device_params)
        for i in range(self.num_envs):
            for env_idx in (1, 2):
                cart_target = self.target_p_cart if env_idx == 1 else -self.target_p_cart
                pole_target = self.target_p_pole if env_idx == 1 else -self.target_p_pole
                cart_drive = UsdPhysics.DriveAPI.Apply(
                    self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env_idx}/cartJoint"),
                    "linear",
                )
                cart_drive.CreateTargetPositionAttr(cart_target)
                cart_drive.CreateTargetVelocityAttr(0.0)
                pole_drive = UsdPhysics.DriveAPI.Apply(
                    self.stage.GetPrimAtPath(f"/envs/env{i}/cartpole{env_idx}/poleJoint"),
                    "angular",
                )
                pole_drive.CreateTargetPositionAttr(pole_target)
                pole_drive.CreateTargetVelocityAttr(0.0)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare force tensors after the drive targets take effect.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 5:
            num_dof = 2
            p1 = (
                self.cartpoles_1.get_data("dof-projected-joint-forces").numpy().reshape(self.cartpoles_1.count, num_dof)
            )
            p2 = (
                self.cartpoles_2.get_data("dof-projected-joint-forces").numpy().reshape(self.cartpoles_2.count, num_dof)
            )
            assert wp_utils.wp_allclose(
                p1, -p2, rtol=1e-3, atol=1e-5
            ), "projected DOF forces are sign-flipped across body0/body1 swap"

            link_forces_1 = self.cartpoles_1.get_data("link-incoming-joint-force").numpy()
            link_forces_2 = self.cartpoles_2.get_data("link-incoming-joint-force").numpy()
            assert link_forces_1.shape == link_forces_2.shape, "link incoming force shapes must match"
            self.finish()
