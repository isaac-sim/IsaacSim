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

"""Provide engine-neutral force-sensor and joint-friction scenarios.

Revolute pendulum scenarios validate incoming joint wrenches and projected DOF
forces against analytical equilibrium references. Additional variants apply an
external force to either pendulum link and validate the resulting reactions.
The joint-friction scenario verifies that high static friction holds an
explicitly displaced joint. Backend subclasses can add engine-specific
articulation configuration through override hooks.
"""

from __future__ import annotations

import os
import sys

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
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402


def _create_pendulum_articulation(
    stage: Usd.Stage,
    pendulum_path: str | Sdf.Path,
    transform: Transform,
    link_half_length: float = 0.5,
    link_mass: float = 1.0,
    add_fixed_joint_link: bool = False,
    revolute_joint_axis: str = "Z",
    revolute_joint_frame_quat: Gf.Quatf = Gf.Quatf(1.0),
    fixed_joint_frame_quat: Gf.Quatf = Gf.Quatf(1.0),
) -> dict[str, Sdf.Path]:
    """Create a two- or three-link revolute pendulum with USD Physics.

    The articulation has a world-fixed root sphere, a cube child attached by a
    revolute joint, and optionally a second cube attached by a fixed joint.

    Args:
        stage: Stage on which to create the articulation.
        pendulum_path: Path for the articulation root.
        transform: Root translation and orientation.
        link_half_length: Half of each non-root link's length.
        link_mass: Mass assigned to each non-root link.
        add_fixed_joint_link: Whether to append a fixed second link.
        revolute_joint_axis: Axis token for the revolute joint.
        revolute_joint_frame_quat: Revolute-joint frame orientation.
        fixed_joint_frame_quat: Optional fixed-joint frame orientation.

    Returns:
        Paths of the root, child, and optional fixed link. The fixed-link path is
        empty when ``add_fixed_joint_link`` is false.

    """
    pendulum_path = Sdf.Path(pendulum_path)
    xform = UsdGeom.Xform.Define(stage, pendulum_path)
    xform.AddTranslateOp().Set(transform.p)
    xform.AddOrientOp().Set(transform.q)
    xform_prim = xform.GetPrim()

    UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

    aspect_ratio = 0.1
    link_length = 2.0 * link_half_length
    link_width = link_length * aspect_ratio
    root_radius = link_width * 0.7
    link_size = Gf.Vec3f(link_length, link_width, link_width)

    root_link_path = pendulum_path.AppendChild("RootLink")
    sphere = UsdGeom.Sphere.Define(stage, root_link_path)
    sphere.CreateRadiusAttr(root_radius)
    sphere.AddTranslateOp().Set(Gf.Vec3f(0.0))
    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    UsdPhysics.MassAPI.Apply(sphere.GetPrim())

    fixed = UsdPhysics.FixedJoint.Define(stage, root_link_path.AppendChild("FixedJoint"))
    fixed.CreateBody1Rel().SetTargets([root_link_path])

    child_link_path = pendulum_path.AppendChild("ChildLink")
    cube = UsdGeom.Cube.Define(stage, child_link_path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3f(link_length, 0, 0))
    cube.AddScaleOp().Set(link_size)
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(link_mass)

    rev = UsdPhysics.RevoluteJoint.Define(stage, child_link_path.AppendChild("RevoluteJoint"))
    rev.CreateAxisAttr(revolute_joint_axis)
    rev.CreateBody0Rel().SetTargets([root_link_path])
    rev.CreateBody1Rel().SetTargets([child_link_path])
    rev.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    rev.CreateLocalRot0Attr().Set(revolute_joint_frame_quat)
    rev.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))
    rev.CreateLocalRot1Attr().Set(revolute_joint_frame_quat)

    fixed_joint_link_path = pendulum_path.AppendChild("FixedJointLink")
    if add_fixed_joint_link:
        cube = UsdGeom.Cube.Define(stage, fixed_joint_link_path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3f(2 * link_length, 0, 0))
        cube.AddScaleOp().Set(link_size)
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(link_mass)

        fix2 = UsdPhysics.FixedJoint.Define(stage, fixed_joint_link_path.AppendChild("FixedJoint"))
        fix2.CreateBody0Rel().SetTargets([child_link_path])
        fix2.CreateBody1Rel().SetTargets([fixed_joint_link_path])
        fix2.CreateLocalPos0Attr().Set(Gf.Vec3f(0.5, 0, 0))
        fix2.CreateLocalRot0Attr().Set(fixed_joint_frame_quat)
        fix2.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))
        fix2.CreateLocalRot1Attr().Set(fixed_joint_frame_quat)
    else:
        fixed_joint_link_path = Sdf.Path()

    return {"root": root_link_path, "child": child_link_path, "fixed": fixed_joint_link_path}


class ForceTorqueSensorCommon(GridTestBase):
    """Validate revolute- and fixed-joint force observations at equilibrium.

    A motor torque holds a three-link pendulum in its initial pose. The scenario
    compares incoming wrenches for both joints with analytical gravity and motor
    references, then compares the projected DOF force with the motor torque.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    rotation_axis: str = "Y"

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        gravity_mag = 10.0
        grid_params = GridParams(num_envs=16, env_spacing=3.0)
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        sim_params.gravity_mag = gravity_mag
        sim_params.add_default_ground = False
        sim_params.time_steps_per_second = 1000
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.gravity_mag = gravity_mag
        self.link_mass = 1.0
        self.link_half_length = 0.5

        self.FT_frame_quat = Gf.Quatf(1.0)
        self.revolute_joint_frame_quat = Gf.Quatf(1.0)
        if self.rotation_axis == "X":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
        if self.rotation_axis == "Z":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())

        pendulum_path = self.env_template_path.AppendChild("pendulum")
        self.link_paths = _create_pendulum_articulation(
            self.stage,
            pendulum_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_mass=self.link_mass,
            add_fixed_joint_link=True,
            revolute_joint_axis=self.rotation_axis,
            revolute_joint_frame_quat=self.revolute_joint_frame_quat,
            fixed_joint_frame_quat=self.FT_frame_quat,
        )

        mxg = self.link_mass * self.gravity_mag
        self.motor_torque = -4 * self.link_mass * self.gravity_mag * self.link_half_length

        revolute_joint_force_W = Gf.Vec3f(0.0, 0.0, mxg - self.motor_torque / (4 * self.link_half_length))
        revolute_joint_torque_W = Gf.Vec3f(0.0, self.motor_torque, 0.0)
        FT_sensor_force_W = Gf.Vec3f(0.0, 0.0, -self.motor_torque / (4 * self.link_half_length))
        FT_sensor_torque_W = Gf.Vec3f(0.0, mxg * self.link_half_length + self.motor_torque / 2.0, 0.0)

        revolute_joint_force_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(
            revolute_joint_force_W
        )
        revolute_joint_torque_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(
            revolute_joint_torque_W
        )
        FT_sensor_force_J = Gf.Rotation(self.FT_frame_quat.GetInverse()).TransformDir(FT_sensor_force_W)
        FT_sensor_torque_J = Gf.Rotation(self.FT_frame_quat.GetInverse()).TransformDir(FT_sensor_torque_W)

        self.reference_forces = wp.zeros((3, 6), dtype=wp.float32, device="cpu").numpy()
        self.reference_forces[1, 0:3] = revolute_joint_force_J
        self.reference_forces[1, 3:6] = revolute_joint_torque_J
        self.reference_forces[2, 0:3] = FT_sensor_force_J
        self.reference_forces[2, 3:6] = FT_sensor_torque_J
        self.reference_forces_rep = self.reference_forces[None].repeat(self.num_envs, axis=0)
        self.reference_forces_mag = wp.zeros((self.num_envs, 3, 6), dtype=wp.float32, device="cpu").numpy()
        for i in range(6):
            self.reference_forces_mag[:, :, i] = (self.reference_forces_rep**2).sum(axis=2) ** 0.5

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply optional backend-specific articulation configuration."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the articulation view and apply the equilibrium motor torque.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)
        dof_pos_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        dof_pos = wp.from_numpy(dof_pos_np, dtype=wp.float32, device=self.wp_device)
        self.all_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)
        self.pendulums.set_data("dof-positions", dof_pos, self.all_indices)

        forces = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        forces.fill(self.motor_torque)
        self.applied_dof_forces = wp.from_numpy(forces, dtype=wp.float32, device=self.wp_device)
        self.pendulums.set_data("dof-actuation-forces", self.applied_dof_forces, self.all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare measured joint and projected forces with references.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno >= 1:
            joint_forces = self.pendulums.get_data("link-incoming-joint-force").numpy().reshape(self.num_envs, 3, 6)
            assert (
                abs(joint_forces - self.reference_forces_rep) <= 0.04 * self.reference_forces_mag
            ).all(), f"joint forces match analytical reference (axis={self.rotation_axis})"

            dof_forces = (
                self.pendulums.get_data("dof-projected-joint-forces")
                .numpy()
                .reshape(self.num_envs, self.pendulums.get_metadata("num-dofs"))
            )
            assert wp_utils.wp_allclose(
                dof_forces, self.motor_torque, rtol=0.05, atol=0.05
            ), "projected DOF forces match motor torque at equilibrium"
            self.finish()


class ForceTorqueSensorXCommon(ForceTorqueSensorCommon):
    """Validate force observations with an X-axis revolute joint."""

    rotation_axis = "X"


class ForceTorqueSensorYCommon(ForceTorqueSensorCommon):
    """Validate force observations with a Y-axis revolute joint."""

    rotation_axis = "Y"


class ForceTorqueSensorZCommon(ForceTorqueSensorCommon):
    """Validate force observations with a Z-axis revolute joint."""

    rotation_axis = "Z"


class LinkIncomingJointForceCommon(GridTestBase):
    """Validate joint reactions to an external force on either pendulum link.

    Subclasses select the revolute axis and the link that receives a world-space
    force. The scenario injects that force through a rigid-body tensor view and
    compares both incoming joint wrenches with analytical gravity and external
    force references.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    revolute_joint_axis = "Y"
    force_on_fixed_link = False

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        gravity_mag = 10.0
        grid_params = GridParams(num_envs=42)
        grid_params.num_rows = 6
        grid_params.row_spacing = 3
        grid_params.col_spacing = 3
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, -1.0, 0.0)
        sim_params.gravity_mag = gravity_mag
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.gravity_mag = gravity_mag
        self.link_mass = 1.5
        self.link_half_length = 0.45

        self.revolute_joint_frame_quat = Gf.Quatf(1.0)
        if self.revolute_joint_axis == "X":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90).GetQuat())
        if self.revolute_joint_axis == "Y":
            self.revolute_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat())

        self.fixed_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0.1, 5), 25).GetQuat())

        pendulum_path = self.env_template_path.AppendChild("pendulum")
        self.link_paths = _create_pendulum_articulation(
            self.stage,
            pendulum_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_mass=self.link_mass,
            add_fixed_joint_link=True,
            revolute_joint_axis=self.revolute_joint_axis,
            revolute_joint_frame_quat=self.revolute_joint_frame_quat,
            fixed_joint_frame_quat=self.fixed_joint_frame_quat,
        )

        mxg = self.link_mass * self.gravity_mag
        force_z = 5.0
        self.child_force = Gf.Vec3f(0.0)
        self.fixed_force = Gf.Vec3f(0.0)
        if self.force_on_fixed_link:
            self.fixed_force[1] = 4.0 / 3.0 * mxg
            self.fixed_force[2] = force_z
        else:
            self.child_force[1] = 4.0 * mxg
            self.child_force[2] = force_z

        self._apply_engine_specifics()

        # Compute expected joint forces in world frame.
        revolute_joint_force_W = Gf.Vec3f(0.0)
        revolute_joint_torque_W = Gf.Vec3f(0.0)
        fixed_joint_force_W = Gf.Vec3f(0.0)
        fixed_joint_torque_W = Gf.Vec3f(0.0)
        if self.force_on_fixed_link:
            revolute_joint_force_W[1] = 2.0 / 3.0 * mxg
            revolute_joint_force_W[2] = -force_z
            revolute_joint_torque_W[1] = 3 * force_z * self.link_half_length
            fixed_joint_force_W[1] = -1.0 / 3.0 * mxg
            fixed_joint_force_W[2] = -force_z
            fixed_joint_torque_W[2] = -1.0 / 3.0 * mxg * self.link_half_length
            fixed_joint_torque_W[1] = force_z * self.link_half_length
        else:
            revolute_joint_force_W[1] = -2.0 * mxg
            revolute_joint_force_W[2] = -force_z
            revolute_joint_torque_W[1] = 1 * force_z * self.link_half_length
            fixed_joint_force_W[1] = mxg
            fixed_joint_torque_W[2] = mxg * self.link_half_length

        fjf_J = Gf.Rotation(self.fixed_joint_frame_quat.GetInverse()).TransformDir(fixed_joint_force_W)
        fjt_J = Gf.Rotation(self.fixed_joint_frame_quat.GetInverse()).TransformDir(fixed_joint_torque_W)
        rjf_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(revolute_joint_force_W)
        rjt_J = Gf.Rotation(self.revolute_joint_frame_quat.GetInverse()).TransformDir(revolute_joint_torque_W)

        self.reference_forces = wp.zeros((3, 6), dtype=wp.float32, device="cpu").numpy()
        self.reference_forces[1, 0:3] = rjf_J
        self.reference_forces[1, 3:6] = rjt_J
        self.reference_forces[2, 0:3] = fjf_J
        self.reference_forces[2, 3:6] = fjt_J
        self.reference_forces_rep = self.reference_forces[None].repeat(self.num_envs, axis=0)
        self.reference_forces_mag = wp.zeros((self.num_envs, 3, 6), dtype=wp.float32, device="cpu").numpy()
        for i in range(6):
            self.reference_forces_mag[:, :, i] = (self.reference_forces_rep**2).sum(axis=2) ** 0.5

    def _apply_engine_specifics(self) -> None:
        """Apply optional backend-specific articulation configuration."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create views for observing joints and applying the selected force.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)

        # Use rigid-body tensor views so the common scenario controls when the
        # one-step force input is consumed.
        self.child_links = sim.create_rigid_body_view("/envs/*/pendulum/ChildLink")
        self.fixed_links = sim.create_rigid_body_view("/envs/*/pendulum/FixedJointLink")
        self._force_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)
        if self.force_on_fixed_link:
            self._ext_force_target = self.fixed_links
            self._ext_force_value = self.fixed_force
        else:
            self._ext_force_target = self.child_links
            self._ext_force_value = self.child_force

    def _apply_ext_force(self) -> None:
        """Apply the configured world-space force at each target's center of mass."""
        v = self._ext_force_value
        forces = wp_utils.fill_vec3(
            self.pendulums.count,
            value=wp.vec3(float(v[0]), float(v[1]), float(v[2])),
            device=self.wp_device,
        )
        self._ext_force_target.set_data("apply-forces", forces, self._force_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Apply the force for one step and validate both incoming wrenches.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 0:
            self._apply_ext_force()
        if stepno == 1:
            joint_forces = self.pendulums.get_data("link-incoming-joint-force").numpy().reshape(self.num_envs, 3, 6)
            assert (abs(joint_forces - self.reference_forces_rep) <= 0.05 * self.reference_forces_mag).all(), (
                f"link incoming joint forces (axis={self.revolute_joint_axis}, "
                f"force_on_fixed={self.force_on_fixed_link}) match analytical reference"
            )
            self.finish()


class LinkIncomingJointForceChildYCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the child of a Y-axis joint."""

    revolute_joint_axis = "Y"
    force_on_fixed_link = False


class LinkIncomingJointForceFixedYCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the fixed link after a Y-axis joint."""

    revolute_joint_axis = "Y"
    force_on_fixed_link = True


class LinkIncomingJointForceChildXCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the child of an X-axis joint."""

    revolute_joint_axis = "X"
    force_on_fixed_link = False


class LinkIncomingJointForceFixedXCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the fixed link after an X-axis joint."""

    revolute_joint_axis = "X"
    force_on_fixed_link = True


class LinkIncomingJointForceChildZCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the child of a Z-axis joint."""

    revolute_joint_axis = "Z"
    force_on_fixed_link = False


class LinkIncomingJointForceFixedZCommon(LinkIncomingJointForceCommon):
    """Apply the external force to the fixed link after a Z-axis joint."""

    revolute_joint_axis = "Z"
    force_on_fixed_link = True


class JointFrictionCommon(GridTestBase):
    """Verify that high static joint friction holds a displaced pendulum.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        gravity_mag = 10.0
        grid_params = GridParams(num_envs=42)
        grid_params.num_rows = 6
        grid_params.row_spacing = 3
        grid_params.col_spacing = 3
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, -1.0, 0.0)
        sim_params.gravity_mag = gravity_mag
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        self.link_mass = 1.5
        self.link_half_length = 0.45

        self.revolute_joint_frame_quat = Gf.Quatf(1.0)
        self.fixed_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0.1, 5), 25).GetQuat())

        pendulum_path = self.env_template_path.AppendChild("pendulum")
        self.link_paths = _create_pendulum_articulation(
            self.stage,
            pendulum_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_mass=self.link_mass,
            add_fixed_joint_link=True,
            revolute_joint_axis="Z",
            revolute_joint_frame_quat=self.revolute_joint_frame_quat,
            fixed_joint_frame_quat=self.fixed_joint_frame_quat,
        )

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply optional backend-specific articulation configuration."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the articulation view and assign high joint friction.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.all_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)
        self.cpu_all_indices = wp_utils.arange(self.pendulums.count, device="cpu")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, 1, True)

        # Set high static friction.
        friction_properties = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs"), 3),
            dtype=wp.float32,
            device="cpu",
        ).numpy()
        friction_properties[:, :, 0] = 100.0
        friction_properties[:, :, 1] = 0.7
        friction_properties[:, :, 2] = 0.6
        wp_fc = wp.from_numpy(friction_properties, dtype=wp.float32, device="cpu")
        self.skip_if_unsupported(self.pendulums, "dof-friction-properties", "set")
        self.pendulums.set_data("dof-friction-properties", wp_fc, self.cpu_all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Displace each joint and verify friction holds its position.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            p = (
                self.pendulums.get_data("dof-positions")
                .numpy()
                .reshape(self.pendulums.count, self.pendulums.get_metadata("num-dofs"))
                .copy()
            )
            p[:, :] = 0.5
            self.pendulums.set_data("dof-positions", self.to_warp(p), self.all_indices)
        if stepno == 2:
            p = (
                self.pendulums.get_data("dof-positions")
                .numpy()
                .reshape(self.pendulums.count, self.pendulums.get_metadata("num-dofs"))
            )
            assert wp_utils.wp_allclose(p, 0.5, atol=1e-4), "high static friction should keep pendulum at 0.5"
            self.finish()
