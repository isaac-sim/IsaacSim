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

"""Contains the base PolicyController class for loading and executing robot control policies."""

from __future__ import annotations

import io
from abc import ABC
from typing import Literal

import carb
import omni
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.simulation_manager import SimulationManager
from omni.physics.core import get_physics_simulation_interface
from pxr import PhysxSchema, Usd, UsdPhysics

from .config_loader import get_articulation_props, get_physics_properties, get_robot_joint_properties, parse_env_config


class PolicyController(ABC):
    """A controller that loads and executes a policy from a file.

    Args:
        prim_path: The path to the prim in the stage
        root_path: The path to the articulation root of the robot
        usd_path: The path to the USD file
        position: The initial position of the robot
        orientation: The initial orientation of the robot
    """

    def __init__(
        self,
        prim_path: str,
        root_path: str | None = None,
        usd_path: str | None = None,
        position: list[float] | None = None,
        orientation: list[float] | None = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)

        if not prim.IsValid():
            prim = define_prim(prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                carb.log_error("unable to add robot usd, usd_path not provided")

        self._prim_path = prim_path

        # Variant must be selected before Articulation construction to author ArticulationRootAPI.
        self._set_physics_variant(prim_path)

        self.robot = Articulation(
            paths=prim_path if root_path is None else root_path,
            positions=position,
            orientations=orientation,
            reset_xform_op_properties=True,
        )

    _ENGINE_TO_VARIANT = {"physx": "physx", "newton": "mujoco"}

    def _set_physics_variant(self, prim_path: str) -> None:
        """Set the Physics variant on the multi-physics asset to match the active engine.

        Args:
            prim_path: The USD prim path of the robot asset.
        """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        variant_sets = prim.GetVariantSets()
        if "Physics" not in variant_sets.GetNames():
            return
        engine = (SimulationManager.get_active_physics_engine() or "").lower()
        if engine == "physx":
            self._ensure_physx_articulation_api()
        target_variant = self._ENGINE_TO_VARIANT.get(engine, engine)
        variant_set = variant_sets.GetVariantSet("Physics")
        available_variants = variant_set.GetVariantNames()
        for available in available_variants:
            if available.lower() == target_variant.lower():
                variant_set.SetVariantSelection(available)
                return
        carb.log_warn(
            f"PolicyController: requested Physics variant {target_variant!r} not available on "
            f"{prim_path}; available variants: {list(available_variants)}. Variant left unchanged."
        )

    def load_policy(self, policy_file_path: str, policy_env_path: str) -> None:
        """Loads a policy from a file.

        Args:
            policy_file_path: The path to the policy file
            policy_env_path: The path to the environment configuration file
        """
        torch = import_module("torch")
        file_content = omni.client.read_file(policy_file_path)[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self.policy = torch.jit.load(file).to(torch.device(str(self.robot._device)))
        self.policy_env_params = parse_env_config(policy_env_path)

        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)

    def initialize(
        self,
        effort_modes: Literal["force", "acceleration"] = "force",
        control_mode: Literal["position", "velocity", "effort"] = "position",
        set_gains: bool = True,
        set_limits: bool = True,
        set_articulation_props: bool = True,
    ) -> None:
        """Initializes the robot and sets up the controller.

        Args:
            effort_modes: The effort modes ("force" or "acceleration")
            control_mode: The control mode ("position", "velocity", or "effort")
            set_gains: Whether to set the joint gains
            set_limits: Whether to set the limits
            set_articulation_props: Whether to set the articulation properties
        """
        active_engine = SimulationManager.get_active_physics_engine()
        is_newton = active_engine == "newton"

        # Skip set_dof_drive_types for Newton
        if not is_newton:
            self.robot.set_dof_drive_types(effort_modes)
        get_physics_simulation_interface().flush_changes()

        self.robot.switch_dof_control_mode(control_mode)
        max_effort, max_vel, stiffness, damping, armature, default_pos, default_vel = get_robot_joint_properties(
            self.policy_env_params, self.robot.dof_names
        )
        self.robot.set_dof_positions(default_pos)
        self.robot.set_dof_velocities(default_vel)
        self.robot.set_dof_armatures(armature)

        self.robot.set_default_state(
            dof_positions=default_pos,
            dof_velocities=default_vel,
        )

        torch = import_module("torch")
        self.default_pos = torch.tensor(default_pos, device=torch.device(str(self.robot._device)))
        self.default_vel = torch.tensor(default_vel, device=torch.device(str(self.robot._device)))

        if set_gains:
            self.robot.set_dof_gains(stiffness, damping)
        if set_limits and not is_newton:
            self.robot.set_dof_max_efforts(max_effort)
            get_physics_simulation_interface().flush_changes()
            self.robot.set_dof_max_velocities(max_vel)
        if set_articulation_props and not is_newton:
            self._set_articulation_props()

    def _set_articulation_props(self) -> None:
        """Sets the articulation root properties from the policy environment parameters."""
        articulation_prop = get_articulation_props(self.policy_env_params)

        solver_position_iteration_count = articulation_prop.get("solver_position_iteration_count")
        solver_velocity_iteration_count = articulation_prop.get("solver_velocity_iteration_count")
        stabilization_threshold = articulation_prop.get("stabilization_threshold")
        enabled_self_collisions = articulation_prop.get("enabled_self_collisions")
        sleep_threshold = articulation_prop.get("sleep_threshold")

        if solver_position_iteration_count not in [None, float("inf")] and solver_velocity_iteration_count not in [
            None,
            float("inf"),
        ]:
            self.robot.set_solver_iteration_counts(
                position_counts=[solver_position_iteration_count],
                velocity_counts=[solver_velocity_iteration_count],
            )
        if stabilization_threshold not in [None, float("inf")]:
            self.robot.set_stabilization_thresholds([stabilization_threshold])
        if isinstance(enabled_self_collisions, bool):
            self.robot.set_enabled_self_collisions([enabled_self_collisions])
        if sleep_threshold not in [None, float("inf")]:
            self.robot.set_sleep_thresholds([sleep_threshold])

    def _ensure_physx_articulation_api(self) -> None:
        """Apply ``PhysxArticulationAPI`` to descendants carrying an articulation root API.

        Walks the subtree rooted at ``self._prim_path`` and applies ``PhysxArticulationAPI``
        to any prim that has ``UsdPhysics.ArticulationRootAPI`` or ``NewtonArticulationRootAPI``
        but is missing ``PhysxArticulationAPI``.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        root_prim = stage.GetPrimAtPath(self._prim_path)
        if not (root_prim and root_prim.IsValid()):
            return
        for prim in Usd.PrimRange(root_prim):
            has_root_api = prim.HasAPI(UsdPhysics.ArticulationRootAPI) or prim.HasAPI("NewtonArticulationRootAPI")
            if not has_root_api:
                continue
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                continue
            PhysxSchema.PhysxArticulationAPI.Apply(prim)
            carb.log_info(f"PolicyController: applied PhysxArticulationAPI to {prim.GetPath()}.")

    def _compute_action(self, obs: "torch.Tensor") -> "torch.Tensor":
        """Compute the action from the observation using the loaded policy.

        This method runs the policy network in inference mode to convert
        the current observation into an action command.

        Args:
            obs: The observation tensor matching the format specified in env.yaml

        Returns:
            The action tensor matching the format expected by the robot controller
        """
        torch = import_module("torch")
        with torch.no_grad():
            action = self.policy(obs).detach().view(-1)
        return action

    def _compute_observation(self) -> "torch.Tensor":
        """Compute the current observation vector for the policy.

        This method must be implemented by derived classes to construct
        the observation vector in the format specified by env.yaml.
        The observation typically includes robot state like joint positions,
        velocities, base pose, etc.

        Returns:
            torch.Tensor: The observation tensor.

        Raises:
            NotImplementedError: This base method must be overridden
        """
        raise NotImplementedError(
            "Compute observation needs to be implemented, expects observation tensor in the structure specified by env.yaml"
        )

    def forward(self) -> None:
        r"""Execute one step of the policy controller.

        This method must be implemented by derived classes to\:
        1. Compute the current observation
        2. Run the policy to get an action
        3. Apply the action to the robot

        The specific implementation depends on the robot and control mode
        (position, velocity, torque, etc.).

        Returns:
            None: This base method raises NotImplementedError.

        Raises:
            NotImplementedError: This base method must be overridden
        """
        raise NotImplementedError(
            "Forward needs to be implemented to compute and apply robot control from observations"
        )

    def post_reset(self) -> None:
        """Called after the controller is reset."""
        self.robot.reset_to_default_state()
