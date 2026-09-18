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

"""Exercise articulation-view creation, metadata, and tensor properties.

The shared scenarios cover asset-backed views, degree-of-freedom property
round trips, indexed limit writes, duplicate topology names, centroidal
quantities, and a fixed single-link articulation with no degrees of freedom.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import warp as wp

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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402


class ArticulationViewCommon(GridTestBase):
    """Validate CartPole articulation counts and resolved prim paths.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16)
        grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2
        grid_params.col_spacing = 6.5
        super().__init__(test_case, grid_params, SimParams(), device_params)

        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), asset_path)

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the CartPole view and validate its resolved paths.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        cartpoles = sim.create_articulation_view("/envs/*/cartpole")
        self.check_articulation_view(cartpoles, self.num_envs, 3, 2, True)

        prim_paths = cartpoles.get_metadata("prim-paths") or []
        if prim_paths:
            assert len(prim_paths) == self.num_envs
            for i in range(self.num_envs):
                assert f"/envs/env{i}/cartpole" in prim_paths

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class HumanoidViewCommon(GridTestBase):
    """Validate the resolved shape of a replicated Humanoid view.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)

        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        actor_path = self.env_template_path.AppendChild("humanoid")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.5)), asset_path)

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the Humanoid view and validate its dimensions.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class ArticulationDofPropertiesCommon(GridTestBase):
    """Round-trip Humanoid joint limits and drive properties.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("humanoid"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and verify degree-of-freedom properties.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        n = humanoids.count
        d = humanoids.get_metadata("num-dofs")
        all_indices = wp_utils.arange(n, device=self.wp_device)

        # joint limits
        limits = wp.zeros((n, d, 2), dtype=wp.float32, device="cpu").numpy()
        limits[:, :, 0] = -0.1
        limits[:, :, 1] = 0.1
        humanoids.set_data(
            "dof-limits",
            wp.from_numpy(limits, dtype=wp.float32, device=self.wp_device),
            all_indices,
        )
        assert wp_utils.wp_allclose(humanoids.get_data("dof-limits").numpy(), limits)

        # joint stiffness
        stiffness = wp.full((n, d), 100.0, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("dof-stiffnesses", stiffness, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("dof-stiffnesses"), stiffness)

        # joint damping
        damping = wp.full((n, d), 100.0, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("dof-dampings", damping, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("dof-dampings"), damping)

        # joint max_force
        max_force = wp.full((n, d), 1000.0, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("dof-max-forces", max_force, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("dof-max-forces"), max_force)

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class ArticulationDofLimitsSubsetCommon(GridTestBase):
    """Validate indexed joint-limit writes with full and compact payloads.

    The scenario addresses only even-numbered environments and verifies that
    odd-numbered environments preserve their original limits.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("humanoid"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write full-width and compact joint-limit subsets.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        n = humanoids.count
        d = humanoids.get_metadata("num-dofs")

        original = humanoids.get_data("dof-limits").numpy().reshape(n, d, 2).copy()

        even = np.arange(0, n, 2, dtype=np.int32)
        odd = np.arange(1, n, 2, dtype=np.int32)
        even_idx = wp.from_numpy(even, dtype=wp.int32, device=self.wp_device)

        # Full [n, d, 2] payload; only the even rows carry new limits and only
        # the even envs are addressed, so the odd envs must be left untouched.
        limits = original.copy()
        limits[even, :, 0] = -0.25
        limits[even, :, 1] = 0.25
        humanoids.set_data("dof-limits", wp.from_numpy(limits, dtype=wp.float32, device=self.wp_device), even_idx)

        result = humanoids.get_data("dof-limits").numpy().reshape(n, d, 2)
        assert wp_utils.wp_allclose(result[even], limits[even]), "even envs not updated"
        assert wp_utils.wp_allclose(result[odd], original[odd]), "odd envs must be unchanged"

        # Compact [K, d, 2] payload addressed by the same K indices: the 3D
        # subset write scatter-expands it, mirroring the 2D compact path.
        compact = np.full((even.shape[0], d, 2), 0.0, dtype=np.float32)
        compact[:, :, 0] = -0.4
        compact[:, :, 1] = 0.4
        humanoids.set_data("dof-limits", wp.from_numpy(compact, dtype=wp.float32, device=self.wp_device), even_idx)

        result2 = humanoids.get_data("dof-limits").numpy().reshape(n, d, 2)
        assert wp_utils.wp_allclose(result2[even], compact), "even envs not updated from compact"
        assert wp_utils.wp_allclose(result2[odd], original[odd]), "odd envs must stay unchanged"
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class ArticulationViewDuplicateNamesCommon(GridTestBase):
    """Validate unique metadata names for duplicate topology leaf names.

    Two parallel chains end in links and joints with identical leaf names. The
    backend must provide unique names in its link and joint metadata.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        def add_rigid_child_link(stage: Usd.Stage, parent_path: Sdf.Path, child_name: str) -> Sdf.Path:
            child_path = parent_path.AppendChild(child_name)
            child_xform = UsdGeom.Xform.Define(stage, child_path)
            UsdPhysics.RigidBodyAPI.Apply(child_xform.GetPrim())
            mass_api = UsdPhysics.MassAPI.Apply(child_xform.GetPrim())
            mass_api.CreateMassAttr(1.0)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(0.1, 0.1, 0.1))
            return child_path

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")
        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        xform_prim = xform.GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

        root_link_path = actor_path.AppendChild("RootLink")
        self.create_rigid_ball(root_link_path, Transform((0.0, 0.0, 1.0)), 0.1)
        fixed_joint_link_path = root_link_path.AppendChild("FixedJointLink")
        UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path).CreateBody1Rel().SetTargets([root_link_path])

        # Two parallel chains, each with a child + a `TipLink` / `TipJoint`
        # of the same name. The engine must uniquify the duplicates.
        for chain_idx in (1, 2):
            child_path = add_rigid_child_link(self.stage, root_link_path, f"ChildLink{chain_idx}")
            joint = UsdPhysics.RevoluteJoint.Define(self.stage, root_link_path.AppendChild(f"ChildJoint{chain_idx}"))
            joint.CreateBody0Rel().SetTargets([root_link_path])
            joint.CreateBody1Rel().SetTargets([child_path])
            joint.CreateAxisAttr("X")

            tip_path = add_rigid_child_link(self.stage, child_path, "TipLink")
            tip_joint = UsdPhysics.RevoluteJoint.Define(self.stage, child_path.AppendChild("TipJoint"))
            tip_joint.CreateBody0Rel().SetTargets([child_path])
            tip_joint.CreateBody1Rel().SetTargets([tip_path])
            tip_joint.CreateAxisAttr("X")

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply backend-specific schemas when a subclass requires them."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the articulation view and validate unique topology names.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        view = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(view, self.num_envs, 5, 4, True)

        # Missing topology metadata is an unported adapter operation, not an
        # engine capability skip, so let it fail visibly.
        joint_names = view.get_metadata("joint-names")
        link_names = view.get_metadata("link-names")
        assert joint_names is not None, "joint-names metadata is not registered"
        assert link_names is not None, "link-names metadata is not registered"
        assert len(joint_names) == 4, "expected 4 joints"
        assert len(link_names) == 5, "expected 5 links"
        # Duplicate-name uniquification is part of this test's observable
        # contract. Keep failures visible instead of turning an unported
        # metadata behavior into a capability skip.
        assert len(set(joint_names)) == 4, f"duplicate joint names: {joint_names!r}"
        assert len(set(link_names)) == 5, f"duplicate link names: {link_names!r}"
        assert joint_names.count("ChildJoint1") == 1
        assert joint_names.count("ChildJoint2") == 1
        assert any(n.startswith("TipJoint") for n in joint_names)
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class ArticulationCentroidalMomentumAndMassCommon(GridTestBase):
    """Validate articulation mass centers and centroidal momentum.

    The scenario applies root and joint velocities to Ant and Humanoid
    articulations, checks the Ant's local center of mass, and verifies the
    Humanoid centroidal-momentum row count.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(16, 5.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(test_case, grid_params, sim_params, device_params)

        humanoid_asset = os.path.join(get_asset_root(), "Humanoid.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("Humanoid"),
            Transform((1.0, 0.0, 1.5)),
            humanoid_asset,
        )
        ant_asset = os.path.join(get_asset_root(), "Ant.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("ant"),
            Transform((-1.0, 0.0, 0.0)),
            ant_asset,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create Ant and Humanoid views and their shared index buffer.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.humanoids = sim.create_articulation_view("/envs/*/Humanoid/torso")
        self.ants = sim.create_articulation_view("/envs/*/ant/torso")
        self.all_indices = wp_utils.arange(self.ants.count, device=self.wp_device)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply velocities and validate mass properties over three steps.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        if stepno == 1:
            vr_ants = self.ants.get_data("root-velocities").numpy().reshape(self.ants.count, 6).copy()
            vr_humanoids = self.humanoids.get_data("root-velocities").numpy().reshape(self.humanoids.count, 6).copy()
            vr_ants[:, 3] += 0.1
            vr_humanoids[:, 3] += 0.5
            self.ants.set_data("root-velocities", self.to_warp(vr_ants), self.all_indices)
            self.humanoids.set_data("root-velocities", self.to_warp(vr_humanoids), self.all_indices)
            com_ants = self.ants.get_data("articulation-mass-center-local").numpy().reshape(self.ants.count, 3)
            assert wp_utils.wp_allclose(com_ants[:, 0:1], 0.0, rtol=1e-1, atol=1e-1), "symmetrical articulation"
        if stepno == 2:
            v_humanoids = (
                self.humanoids.get_data("dof-velocities")
                .numpy()
                .reshape(self.humanoids.count, self.humanoids.get_metadata("num-dofs"))
                .copy()
                + 0.5
            )
            v_ants = (
                self.ants.get_data("dof-velocities")
                .numpy()
                .reshape(self.ants.count, self.ants.get_metadata("num-dofs"))
                .copy()
                + 0.1
            )
            self.humanoids.set_data("dof-velocities", self.to_warp(v_humanoids), self.all_indices)
            self.ants.set_data("dof-velocities", self.to_warp(v_ants), self.all_indices)
        if stepno == 3:
            cmm = self.humanoids.get_data("articulation-centroidal-momentum").numpy()
            assert cmm.shape[0] == self.humanoids.count
            self.finish()


class ArticulationSpecialCasesCommon(GridTestBase):
    """Validate joint-property access on an articulation with no movable joints.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=16, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        actor_path = self.env_template_path.AppendChild("SimpleArticulation")
        xform = UsdGeom.Xform.Define(self.stage, actor_path)
        xform_prim = xform.GetPrim()

        UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

        root_link_path = actor_path.AppendChild("RootLink")
        fixed_joint_link_path = root_link_path.AppendChild("FixedJointLink")

        # Fixed root link — sphere with rigid body API at z=1.
        self.create_rigid_ball(root_link_path, Transform((0.0, 0.0, 1.0)), 0.1)
        joint = UsdPhysics.FixedJoint.Define(self.stage, fixed_joint_link_path)
        joint.CreateBody1Rel().SetTargets([root_link_path])

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply backend-specific schemas when a subclass requires them."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the fixed single-link articulation view.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        articulations = sim.create_articulation_view("/envs/*/SimpleArticulation")
        self.check_articulation_view(articulations, self.num_envs, 1, 0, True)
        self.articulations = articulations

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Round-trip the empty stiffness tensor and finish.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """
        articulations = self.articulations
        # Trigger the (n, 0) special case; round-trip get/set should be a no-op.
        articulations.get_data("dof-stiffnesses")
        stiffness = wp.zeros(
            (articulations.count, articulations.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        stiffness.fill(100.0)
        all_indices = wp_utils.arange(articulations.count, device=self.wp_device)
        wp_stiffness = wp.from_numpy(stiffness, dtype=wp.float32, device=self.wp_device)
        articulations.set_data("dof-stiffnesses", wp_stiffness, all_indices)
        actual = articulations.get_data("dof-stiffnesses")
        actual_np = actual.numpy()
        assert wp_utils.wp_allclose(actual_np, stiffness)
        if stepno == 1:
            self.finish()
