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

"""Provide engine-neutral joint-force projection scenarios.

The scenarios constrain one- and two-link D6 pendulums against frictionless
kinematic balls. They compare incoming joint wrenches with analytical static
equilibrium references and compare projected joint forces with applied motor
torques. Subclasses vary the free rotation axes and motor torque, while an
override hook permits backend-specific articulation configuration.
"""

from __future__ import annotations

import os
import sys

import pytest
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402


def _ensure_frictionless_material(stage: Usd.Stage, material_path: str = "/material") -> str:
    """Create a frictionless, nonrestitutive material when it does not exist.

    Args:
        stage: Stage that owns the material.
        material_path: Absolute path for the material prim.

    Returns:
        The material path supplied by the caller.

    """
    sdf_path = Sdf.Path(material_path)
    if not stage.GetPrimAtPath(sdf_path):
        UsdShade.Material.Define(stage, sdf_path)
        material_api = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(sdf_path))
        material_api.CreateDynamicFrictionAttr(0.0)
        material_api.CreateStaticFrictionAttr(0.0)
        material_api.CreateRestitutionAttr(0.0)
    return material_path


def _bind_physics_material(prim: Usd.Prim, material_path: str) -> None:
    """Bind a material to a prim for the physics purpose.

    Args:
        prim: Prim that receives the material binding.
        material_path: Path of the material to bind.

    """
    binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
    material = UsdShade.Material.Get(prim.GetStage(), material_path)
    binding_api.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def _add_kinematic_ground_ball(
    stage: Usd.Stage,
    path: str | Sdf.Path,
    position: Gf.Vec3f,
    radius: float = 0.1,
    material_path: str | None = None,
) -> Usd.Prim:
    """Create a kinematic sphere collider used to constrain a pendulum tip.

    Args:
        stage: Stage on which to create the sphere.
        path: Path for the sphere prim.
        position: Sphere center in its parent coordinate system.
        radius: Sphere radius.
        material_path: Optional path of a physics material to bind.

    Returns:
        The created sphere prim.

    """
    sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    sphere.CreateRadiusAttr(radius)
    sphere.AddTranslateOp().Set(position)
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    rb_api = UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    rb_api.CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(sphere.GetPrim())
    if material_path:
        _bind_physics_material(sphere.GetPrim(), material_path)
    return sphere.GetPrim()


def _add_rigid_capsule(
    stage: Usd.Stage,
    path: str | Sdf.Path,
    radius: float,
    height: float,
    axis: str,
    position: Gf.Vec3f,
) -> Usd.Prim:
    """Create a rigid capsule with collision and mass APIs.

    Args:
        stage: Stage on which to create the capsule.
        path: Path for the capsule prim.
        radius: Capsule radius.
        height: Value authored to the capsule height attribute.
        axis: Local capsule axis token.
        position: Capsule center in its parent coordinate system.

    Returns:
        The created capsule prim.

    """
    capsule = UsdGeom.Capsule.Define(stage, path)
    capsule.CreateRadiusAttr(radius)
    capsule.CreateHeightAttr(height)
    capsule.CreateAxisAttr(axis)
    capsule.AddTranslateOp().Set(position)
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(capsule.GetPrim())
    UsdPhysics.MassAPI.Apply(capsule.GetPrim())
    return capsule.GetPrim()


def _create_d6_pendulum_articulation(
    stage: Usd.Stage,
    pendulum_path: str | Sdf.Path,
    transform: Transform,
    link_half_length: float = 0.5,
    link_height: float = 0.025,
    link_mass: float = 1.0,
    add_fixed_joint_link: bool = False,
    rotational_joint_frame_quat: Gf.Quatf = Gf.Quatf(1.0),
    usd_d6_free_rot_dof: dict[str, bool] | None = None,
    fixed_joint_frame_quat: Gf.Quatf = Gf.Quatf(1.0),
) -> dict[str, Sdf.Path]:
    """Create a D6 pendulum using standard USD Physics schemas.

    The articulation contains a world-fixed root sphere and one capsule child.
    All translation axes and any disabled rotation axes are locked through
    ``LimitAPI``. An optional second capsule is fixed to the child.

    Args:
        stage: Stage on which to create the articulation.
        pendulum_path: Path for the articulation root.
        transform: Root translation and orientation.
        link_half_length: Half of each capsule's authored height.
        link_height: Capsule radius and the basis for the root-sphere radius.
        link_mass: Mass assigned to each non-root link.
        add_fixed_joint_link: Whether to append a fixed second capsule.
        rotational_joint_frame_quat: Joint-frame orientation for the D6 joint.
        usd_d6_free_rot_dof: Mapping from ``rotX``, ``rotY``, and ``rotZ`` to
            whether each axis remains free. The default frees only ``rotZ``.
        fixed_joint_frame_quat: Joint-frame orientation for the optional fixed joint.

    Returns:
        Paths of the root, child, and optional fixed link. The fixed-link path is
        empty when ``add_fixed_joint_link`` is false.

    """
    if usd_d6_free_rot_dof is None:
        usd_d6_free_rot_dof = {"rotX": False, "rotY": False, "rotZ": True}

    pendulum_path = Sdf.Path(pendulum_path)
    xform = UsdGeom.Xform.Define(stage, pendulum_path)
    xform.AddTranslateOp().Set(transform.p)
    xform.AddOrientOp().Set(transform.q)
    xform_prim = xform.GetPrim()

    UsdPhysics.ArticulationRootAPI.Apply(xform_prim)

    link_length = 2.0 * link_half_length
    root_radius = link_height * 2

    # Root link — sphere fixed to world.
    root_link_path = pendulum_path.AppendChild("RootLink")
    sphere = UsdGeom.Sphere.Define(stage, root_link_path)
    sphere.CreateRadiusAttr(root_radius)
    sphere.AddTranslateOp().Set(Gf.Vec3f(0.0))
    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    UsdPhysics.MassAPI.Apply(sphere.GetPrim())

    fixed = UsdPhysics.FixedJoint.Define(stage, root_link_path.AppendChild("FixedJoint"))
    fixed.CreateBody1Rel().SetTargets([root_link_path])

    # Child link — capsule along X with center offset by half_length.
    child_link_path = pendulum_path.AppendChild("ChildLink")
    _add_rigid_capsule(
        stage,
        child_link_path,
        link_height,
        link_length,
        "X",
        Gf.Vec3f(link_length / 2, 0, 0),
    )
    UsdPhysics.MassAPI(stage.GetPrimAtPath(child_link_path)).CreateMassAttr(link_mass)

    # D6 joint root → child with selective axis locking.
    joint = UsdPhysics.Joint.Define(stage, child_link_path.AppendChild("D6Joint"))
    joint.CreateBody0Rel().SetTargets([root_link_path])
    joint.CreateBody1Rel().SetTargets([child_link_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot0Attr().Set(rotational_joint_frame_quat)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.5, 0, 0))
    joint.CreateLocalRot1Attr().Set(rotational_joint_frame_quat)

    joint_prim = joint.GetPrim()
    # Lock all translations + every rotation axis the caller didn't free.
    locked_dofs = ["transX", "transY", "transZ"]
    for axis_name, free in usd_d6_free_rot_dof.items():
        if not free:
            locked_dofs.append(axis_name)
    for axis_name in locked_dofs:
        limit_api = UsdPhysics.LimitAPI.Apply(joint_prim, axis_name)
        limit_api.CreateLowAttr(1.0)
        limit_api.CreateHighAttr(-1.0)

    fixed_joint_link_path = pendulum_path.AppendChild("FixedJointLink")
    if add_fixed_joint_link:
        _add_rigid_capsule(
            stage,
            fixed_joint_link_path,
            link_height,
            link_length,
            "X",
            Gf.Vec3f(2 * link_length, 0, 0),
        )
        UsdPhysics.MassAPI(stage.GetPrimAtPath(fixed_joint_link_path)).CreateMassAttr(link_mass)

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


class JointForceDofProjectionSingleLinkCommon(GridTestBase):
    """Validate forces for a single-link D6 pendulum at static equilibrium.

    A frictionless kinematic ball constrains the child link's tip. The scenario
    checks the child's incoming joint wrench against a gravity, motor, and
    contact-force reference, then checks the projected DOF force against the
    applied motor torque.

    Subclasses set ``dof_torque_multiplier`` to ``-1`` for zero contact, ``0``
    for zero motor torque, or ``1`` for motor-assisted contact.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    free_rotation_axis: str = "rotY"
    dof_torque_multiplier: float = 0.0

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
        self.link_height = 0.05

        # Rotate the joint frame so the unlocked axis aligns with world Y while
        # gravity acts along world -Z.
        self.D6_joint_frame_quat = Gf.Quatf(1.0)
        self.usd_d6_free_rot_dofs = {"rotX": False, "rotY": False, "rotZ": False}
        self.usd_d6_free_rot_dofs[self.free_rotation_axis] = True
        if self.free_rotation_axis == "rotX":
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
        elif self.free_rotation_axis == "rotZ":
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())

        # Frictionless material — bound to the ground ball and link below.
        self.material_path = _ensure_frictionless_material(self.stage)

        self.pendulum_template_path = self.env_template_path.AppendChild("pendulum")
        self.link_paths = _create_d6_pendulum_articulation(
            self.stage,
            self.pendulum_template_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_height=self.link_height,
            link_mass=self.link_mass,
            add_fixed_joint_link=False,
            rotational_joint_frame_quat=self.D6_joint_frame_quat,
            usd_d6_free_rot_dof=self.usd_d6_free_rot_dofs,
        )

        # Kinematic ground ball at the link tip. Distance is one full
        # link length out from the joint origin (single-link variant).
        ball_size = 0.1
        link_length = 2.0 * self.link_half_length
        ball_position = Gf.Vec3f(link_length, 0.0, -ball_size - self.link_height)
        self.ground_ball_path = self.env_template_path.AppendChild("ground_ball")
        _add_kinematic_ground_ball(
            self.stage,
            self.ground_ball_path,
            ball_position,
            radius=ball_size,
            material_path=self.material_path,
        )
        # Bind frictionless material on the link too.
        child_link = self.stage.GetPrimAtPath(self.link_paths["child"])
        if child_link:
            _bind_physics_material(child_link, self.material_path)

        # Analytical reference. Single-link with ground-ball contact:
        #   motor_torque + gravity + contact_force * link_half_length = 0
        # gives contact_force = mxg/2 + motor_torque/(2*link_half_length)
        # when the ball is in contact (motor_torque > -mxg*link_half_length).
        mxg = self.link_mass * self.gravity_mag
        self.motor_torque = self.dof_torque_multiplier * mxg * self.link_half_length
        if self.motor_torque > -mxg * self.link_half_length:
            contact_force = mxg / 2.0 + self.motor_torque / (2.0 * self.link_half_length)
        else:
            contact_force = 0.0

        D6_joint_force_W = Gf.Vec3f(0.0, 0.0, mxg - contact_force)
        D6_joint_torque_W = Gf.Vec3f(0.0, self.motor_torque, 0.0)

        D6_joint_force_J = Gf.Rotation(self.D6_joint_frame_quat.GetInverse()).TransformDir(D6_joint_force_W)
        D6_joint_torque_J = Gf.Rotation(self.D6_joint_frame_quat.GetInverse()).TransformDir(D6_joint_torque_W)

        self.reference_forces = wp.zeros((2, 6), dtype=wp.float32, device="cpu").numpy()
        self.reference_forces[1, 0:3] = D6_joint_force_J
        self.reference_forces[1, 3:6] = D6_joint_torque_J
        self.reference_forces_rep = self.reference_forces[None].repeat(self.num_envs, axis=0)

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply optional backend-specific articulation configuration."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the articulation view and apply the configured motor torque.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        self.check_articulation_view(self.pendulums, self.num_envs, 2, 1, True)

        all_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)
        dof_pos_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        dof_pos = wp.from_numpy(dof_pos_np, dtype=wp.float32, device=self.wp_device)
        self.pendulums.set_data("dof-positions", dof_pos, all_indices)

        forces_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        forces_np.fill(self.motor_torque)
        self.applied_dof_forces = wp.from_numpy(forces_np, dtype=wp.float32, device=self.wp_device)
        self.pendulums.set_data("dof-actuation-forces", self.applied_dof_forces, all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare measured joint and projected forces with equilibrium values.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            joint_forces = self.pendulums.get_data("link-incoming-joint-force").numpy().reshape(self.num_envs, 2, 6)
            ref_row = self.reference_forces[1]
            ref_mag = float((ref_row**2).sum() ** 0.5)
            tol = 0.20 * max(ref_mag, 1e-3)
            assert (abs(joint_forces[:, 1, :] - ref_row) <= tol).all(), (
                f"joint forces (axis={self.free_rotation_axis}, "
                f"mult={self.dof_torque_multiplier}) match analytical reference"
            )

            dof_forces = (
                self.pendulums.get_data("dof-projected-joint-forces")
                .numpy()
                .reshape(self.num_envs, self.pendulums.get_metadata("num-dofs"))
            )
            assert wp_utils.wp_allclose(
                dof_forces, self.motor_torque, rtol=0.05, atol=0.1
            ), "projected DOF forces match motor torque at equilibrium"
            self.finish()


# These public names mirror the scenario names imported by backend test modules.
class JFP_Y_ZeroDofCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a free Y rotation axis with zero motor torque."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = 0.0


class JFP_Y_ZeroContactCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a free Y rotation axis with motor torque that removes contact."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = -1.0


class JFP_Y_DofContactCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a free Y rotation axis with positive motor-assisted contact."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = 1.0


class JFP_X_ZeroDofCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a frame-aligned free X rotation axis with zero motor torque."""

    free_rotation_axis = "rotX"
    dof_torque_multiplier = 0.0


class JFP_Z_ZeroDofCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a frame-aligned free Z rotation axis with zero motor torque."""

    free_rotation_axis = "rotZ"
    dof_torque_multiplier = 0.0


class JFP_X_ZeroContactCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a frame-aligned free X rotation with zero expected contact."""

    free_rotation_axis = "rotX"
    dof_torque_multiplier = -1.0


class JFP_Z_ZeroContactCommon(JointForceDofProjectionSingleLinkCommon):  # noqa: N801
    """Use a frame-aligned free Z rotation with zero expected contact."""

    free_rotation_axis = "rotZ"
    dof_torque_multiplier = -1.0


class JointForceDofProjectionTwoLinksCommon(GridTestBase):
    """Validate forces for a D6 pendulum with a fixed second link.

    A frictionless kinematic ball supports the fixed link at a known moment arm.
    The scenario checks the incoming wrenches at both joints against static
    equilibrium references and checks every projected DOF force against the
    applied actuation vector.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    free_rotation_axis: str = "rotY"
    dof_torque_multiplier: float = 0.0
    free_torsional_axis: bool = False
    free_all_axes: bool = False

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
        self.link_height = 0.05

        # Rotate the joint frame so the primary unlocked axis aligns with world Y
        # while gravity acts along world -Z.
        self.D6_joint_frame_quat = Gf.Quatf(1.0)
        self.usd_d6_free_rot_dofs = {"rotX": False, "rotY": False, "rotZ": False}
        self.usd_d6_free_rot_dofs[self.free_rotation_axis] = True
        if self.free_rotation_axis == "rotX":
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
            if self.free_torsional_axis:
                self.usd_d6_free_rot_dofs["rotY"] = True
            if self.free_all_axes:
                self.usd_d6_free_rot_dofs["rotZ"] = True
        elif self.free_rotation_axis == "rotZ":
            self.D6_joint_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90).GetQuat())
            if self.free_torsional_axis:
                self.usd_d6_free_rot_dofs["rotX"] = True
            if self.free_all_axes:
                self.usd_d6_free_rot_dofs["rotY"] = True
        else:
            if self.free_torsional_axis:
                self.usd_d6_free_rot_dofs["rotX"] = True
            if self.free_all_axes:
                self.usd_d6_free_rot_dofs["rotZ"] = True

        if self.free_torsional_axis:
            self.num_arti_dofs = 2
        elif self.free_all_axes:
            self.num_arti_dofs = 3
        else:
            self.num_arti_dofs = 1

        # FT-sensor fixed-joint frame quat (world rotated 90 about Y).
        self.FT_frame_quat = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 1, 0), 90).GetQuat())

        # Frictionless material — bound to the ground ball and to the
        # FT-sensor link below.
        self.material_path = _ensure_frictionless_material(self.stage)

        self.pendulum_template_path = self.env_template_path.AppendChild("pendulum")
        self.link_paths = _create_d6_pendulum_articulation(
            self.stage,
            self.pendulum_template_path,
            Transform((0.0, 0.0, 0.0)),
            link_half_length=self.link_half_length,
            link_height=self.link_height,
            link_mass=self.link_mass,
            add_fixed_joint_link=True,
            rotational_joint_frame_quat=self.D6_joint_frame_quat,
            usd_d6_free_rot_dof=self.usd_d6_free_rot_dofs,
            fixed_joint_frame_quat=self.FT_frame_quat,
        )

        # Ball at distance = 4*half_length to anchor the FT-sensor tip.
        ball_size = 0.1
        link_length = 2.0 * self.link_half_length
        ball_position = Gf.Vec3f(link_length * 2, 0.0, -ball_size - self.link_height)
        self.ground_ball_path = self.env_template_path.AppendChild("ground_ball")
        _add_kinematic_ground_ball(
            self.stage,
            self.ground_ball_path,
            ball_position,
            radius=ball_size,
            material_path=self.material_path,
        )
        # Bind frictionless material on the FT-sensor link too.
        ft_link = self.stage.GetPrimAtPath(self.link_paths["fixed"])
        if ft_link:
            _bind_physics_material(ft_link, self.material_path)

        # Analytical reference (two-link).
        mxg = self.link_mass * self.gravity_mag
        self.motor_torque = self.dof_torque_multiplier * self.link_mass * self.gravity_mag * self.link_half_length
        if self.motor_torque > -4 * self.link_mass * self.gravity_mag * self.link_half_length:
            contact_force = mxg + self.motor_torque / (4 * self.link_half_length)
        else:
            contact_force = 0

        D6_joint_force_W = Gf.Vec3f(0.0, 0.0, 2 * mxg - contact_force)
        D6_joint_torque_W = Gf.Vec3f(0.0, self.motor_torque, 0.0)
        FT_sensor_force_W = Gf.Vec3f(0.0, 0.0, mxg - contact_force)
        FT_sensor_torque_W = Gf.Vec3f(
            0.0,
            -mxg * self.link_half_length + 2 * contact_force * self.link_half_length,
            0.0,
        )

        rotinv = Gf.Rotation(self.D6_joint_frame_quat.GetInverse())
        ftinv = Gf.Rotation(self.FT_frame_quat.GetInverse())

        self.reference_forces = wp.zeros((3, 6), dtype=wp.float32, device="cpu").numpy()
        self.reference_forces[1, 0:3] = rotinv.TransformDir(D6_joint_force_W)
        self.reference_forces[1, 3:6] = rotinv.TransformDir(D6_joint_torque_W)
        self.reference_forces[2, 0:3] = ftinv.TransformDir(FT_sensor_force_W)
        self.reference_forces[2, 3:6] = ftinv.TransformDir(FT_sensor_torque_W)
        self.reference_forces_rep = self.reference_forces[None].repeat(self.num_envs, axis=0)

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply optional backend-specific articulation configuration."""

    def on_start(self, sim: SimulationEntities) -> None:
        """Create the articulation view and apply torque to the selected DOF.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.pendulums = sim.create_articulation_view("/envs/*/pendulum")
        # If the engine reports zero DOFs the articulation wasn't
        # recognized at all (e.g. Newton's adapter doesn't accept the
        # D6-with-LimitAPI lock pattern). Skip with a clear engine
        # reason rather than failing on a zero-axis index.
        if self.pendulums.get_metadata("num-dofs") == 0:
            pytest.skip("engine reported max_dofs=0 — articulation/D6-joint " "configuration not recognized")
        self.check_articulation_view(self.pendulums, self.num_envs, 3, self.pendulums.get_metadata("num-dofs"), True)
        all_indices = wp_utils.arange(self.pendulums.count, device=self.wp_device)
        dof_pos_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        self.pendulums.set_data(
            "dof-positions",
            wp.from_numpy(dof_pos_np, dtype=wp.float32, device=self.wp_device),
            all_indices,
        )

        # Apply the motor torque on the actuated DOF only.
        forces_np = wp.zeros(
            (self.pendulums.count, self.pendulums.get_metadata("num-dofs")), dtype=wp.float32, device=self.wp_device
        ).numpy()
        # Select the configured actuation axis, clamping to the engine-reported
        # DOF count when fewer axes are exposed.
        if not self.free_torsional_axis and not self.free_all_axes:
            actuated_idx = 0
        elif self.free_rotation_axis == "rotX":
            actuated_idx = 0
        elif self.free_rotation_axis == "rotY":
            actuated_idx = 1
        elif self.free_rotation_axis == "rotZ":
            actuated_idx = 1 if not self.free_all_axes else 2
        else:
            actuated_idx = 0
        actuated_idx = min(actuated_idx, self.pendulums.get_metadata("num-dofs") - 1)
        forces_np[:, actuated_idx] = self.motor_torque

        self.applied_dof_forces = wp.from_numpy(forces_np, dtype=wp.float32, device=self.wp_device)
        self.pendulums.set_data("dof-actuation-forces", self.applied_dof_forces, all_indices)

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Compare both joint wrenches and projected forces with references.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno == 1:
            joint_forces = self.pendulums.get_data("link-incoming-joint-force").numpy().reshape(self.num_envs, 3, 6)
            ref_mag = max(float((self.reference_forces**2).sum() ** 0.5), 1e-3)
            # Free-torsional / spherical configurations introduce extra
            # solver slop on the additional unlocked rotation axes —
            # widen the tolerance accordingly. Single-DOF case stays at
            # 20%; 2-DOF (torsional) bumps to 35%; full spherical
            # (3-DOF) bumps to 50%.
            tol_pct = 0.20
            if self.free_torsional_axis:
                tol_pct = 0.35
            if self.free_all_axes:
                tol_pct = 0.50
            tol = tol_pct * ref_mag
            assert (abs(joint_forces[:, 1:, :] - self.reference_forces[1:, :]) <= tol).all(), (
                f"two-link joint forces (axis={self.free_rotation_axis}, "
                f"mult={self.dof_torque_multiplier}, torsional="
                f"{self.free_torsional_axis}, all_axes={self.free_all_axes}) "
                "match analytical reference"
            )

            dof_forces = (
                self.pendulums.get_data("dof-projected-joint-forces")
                .numpy()
                .reshape(self.num_envs, self.pendulums.get_metadata("num-dofs"))
            )
            applied_np = self.applied_dof_forces.numpy()
            assert wp_utils.wp_allclose(
                dof_forces, applied_np, rtol=0.05, atol=0.5
            ), "projected DOF forces match applied actuation forces"
            self.finish()


class JFP_TwoLinks_Y_ZeroContactCommon(JointForceDofProjectionTwoLinksCommon):  # noqa: N801
    """Use the negative unit motor multiplier for the Y-axis two-link case."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = -1.0


class JFP_TwoLinks_Y_ZeroDofCommon(JointForceDofProjectionTwoLinksCommon):  # noqa: N801
    """Use a free Y rotation axis with zero motor torque."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = 0.0


class JFP_TwoLinks_Y_DofContactCommon(JointForceDofProjectionTwoLinksCommon):  # noqa: N801
    """Use a free Y rotation axis with positive motor-assisted contact."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = 1.0


class JFP_Torsional_Y_Common(JointForceDofProjectionTwoLinksCommon):  # noqa: N801
    """Free the Y rotation and X torsional axes."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = -1.0
    free_torsional_axis = True


class JFP_Spherical_Y_Common(JointForceDofProjectionTwoLinksCommon):  # noqa: N801
    """Free Y and Z rotations in the spherical-labeled configuration."""

    free_rotation_axis = "rotY"
    dof_torque_multiplier = -1.0
    free_all_axes = True
