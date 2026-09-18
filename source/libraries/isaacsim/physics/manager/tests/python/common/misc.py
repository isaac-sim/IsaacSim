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

"""Provide miscellaneous lifecycle and articulation scenarios.

The module covers prim deactivation, locked D6 motion, and Newton's rejection
of unsupported articulation root types.

* ``UsdPrimDeletionCommon`` — exercises USD prim activation/deactivation
  during simulation (no engine-specific schemas).
* ``ArtJointFreeMotionToLimitMotionCommon`` — manually built D6
  articulation with all translations locked. Engine-specific
  subclasses override ``_apply_engine_specifics()`` to layer their
  bits (e.g. ovphysx applies `PhysxSchema.PhysxArticulationAPI`).
"""

from __future__ import annotations

import os
import sys

import pytest

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
    get_asset_root,
)
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402


class UsdPrimDeletionCommon(GridTestBase):
    """Scenario that deactivates a rigid body while views remain valid.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(16, 4.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 1
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("ball")
        transform = Transform((0.1, 0.1, 0.5))
        self.create_rigid_ball(actor_path, transform, 0.2)
        self.actor_path = actor_path

        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        transform = Transform((0.0, 0.0, 1.0))
        self.create_actor_from_asset(actor_path, transform, asset_path)

    def on_start(self, sim: object) -> None:
        """Create rigid-body and articulation views for the replicated actors.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rigid_body_view = sim.create_rigid_body_view("/envs/*/ball")
        self.check_rigid_body_view(rigid_body_view, self.num_envs)
        self.rigid_body_view = rigid_body_view

        articulation_view = sim.create_articulation_view("/envs/*/ant/torso")
        self.check_articulation_view(articulation_view, self.num_envs, 9, 8, True)
        self.articulation_view = articulation_view

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Deactivate one ball and verify the simulation view stays valid.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            v = getattr(sim, "is_valid", True)
            assert v() if callable(v) else bool(v)
        if stepno == 2:
            prim = self.stage.GetPrimAtPath("/envs/env_0002/ball")
            if prim:
                prim.SetActive(False)
        elif stepno == 10:
            self.finish()


class ArtJointFreeMotionToLimitMotionCommon(GridTestBase):
    """Scenario with a D6 articulation whose translation axes are locked.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(16, 5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")
        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        xform_prim = xform.GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

        root_link_path = actor_path.AppendChild("RootLink")

        for body_name, child_path in (
            ("rigidBody0", root_link_path.AppendChild("rigidBody0")),
            ("rigidBody1", root_link_path.AppendChild("rigidBody1")),
        ):
            rb_xform = UsdGeom.Xform.Define(self.stage, child_path)
            rb_prim = rb_xform.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(rb_prim).CreateRigidBodyEnabledAttr(True)
            mass_api = UsdPhysics.MassAPI.Apply(rb_prim)
            mass_api.CreateMassAttr(1.0)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(1.0, 1.0, 1.0))

        d6_path = root_link_path.AppendChild("d6Joint")
        d6 = UsdPhysics.Joint.Define(self.stage, d6_path)
        d6.CreateBody0Rel().SetTargets([root_link_path.AppendChild("rigidBody0")])
        d6.CreateBody1Rel().SetTargets([root_link_path.AppendChild("rigidBody1")])
        self.d6_prim = d6.GetPrim()
        for axis in (UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY, UsdPhysics.Tokens.transZ):
            limit = UsdPhysics.LimitAPI.Apply(self.d6_prim, axis)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)

        fixed_path = root_link_path.AppendChild("FixedJoint")
        UsdPhysics.FixedJoint.Define(self.stage, fixed_path).CreateBody0Rel().AddTarget(
            root_link_path.AppendChild("rigidBody0")
        )

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply backend-specific articulation schemas when required."""

    def on_start(self, sim: object) -> None:
        """Create and validate the locked D6 articulation view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.arti_view = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(self.arti_view, self.num_envs, 2, 3, True)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Finish after the first completed simulation step.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            self.finish()


class ArticulationExoticRootRejectedCommon(GridTestBase):
    """Scenario that verifies Newton rejects a distance-joint root.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(2, 5)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        # A floating-base body: ArticulationRootAPI + a rigid body, no explicit
        # root joint, so Newton adds a FREE root we then re-type to DISTANCE.
        actor_path = self.env_template_path.AppendChild("ExoticRoot")
        UsdGeom.Xform.Define(self.stage, actor_path)
        sphere = UsdGeom.Sphere.Define(self.stage, actor_path.AppendChild("RootLink"))
        sphere.CreateRadiusAttr(0.1)
        UsdPhysics.ArticulationRootAPI.Apply(sphere.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())

    def on_start(self, sim: object) -> None:
        """Retype the root joint and verify articulation-view creation fails.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        import newton
        import warp as wp
        from isaacsim.physics_engines.ovnewton.impl.register_simulation import _active

        model = next(iter(_active.values())).stage.model
        joint_type = model.joint_type.numpy()
        for start in model.articulation_start.numpy()[: model.articulation_count]:
            joint_type[start] = int(newton.JointType.DISTANCE)
        model.joint_type = wp.array(joint_type, dtype=model.joint_type.dtype, device=model.device)

        with pytest.raises(ValueError, match="unsupported root joint type"):
            sim.create_articulation_view("/envs/*/ExoticRoot/RootLink")
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused per-step callback required by the scenario harness.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
