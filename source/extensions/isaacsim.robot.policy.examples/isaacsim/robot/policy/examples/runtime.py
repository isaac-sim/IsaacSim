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

"""Policy deployment runtime: spawn/initialize helpers, actuators, and the runner.

Merges the former ``deployment``, ``session``, and ``interactive.runner`` layers; the articulation
state bridge lives in :mod:`.application`.
:class:`RobotPolicyRunner` is driven from a caller-owned physics callback: ``spawn()`` authors the robot
pre-play, ``initialize()`` builds or resets the runtime on a playing timeline — deriving the
binding from the artifact's IO descriptor unless the spec supplies an explicit hook — and
``step(dt, command)`` runs the cadence-gated control loop once per physics tick. Callers reset
or teleport the robot before reinitializing for replay.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Literal

import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.usd
import warp as wp
from isaacsim.core.experimental.actuators import ArticulationActuators
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from omni.physics.core import get_physics_simulation_interface

from .application import RobotStateReadSpec, apply_robot_state, read_robot_state
from .binding import bind_policy, derive_binding
from .controller import IsaacLabPolicyController
from .env_config import PolicyEnvConfig
from .model import load_policy_model
from .spec import PolicyArtifact, PolicySpec, read_artifact_bytes
from .usd_props import _apply_newton_import_defaults, apply_startup_material_events, spawn_policy_robot

__all__ = ["RobotPolicyRunner", "initialize_articulation"]


# == Initialization helpers ==================================================================


def initialize_articulation(
    articulation: Articulation,
    env_config: PolicyEnvConfig,
    policy_joint_names: Sequence[str],
    joint_control_modes: Mapping[str, str],
    *,
    effort_modes: Literal["force", "acceleration"] = "force",
    set_gains: bool = True,
    set_limits: bool = True,
    reset_to_default_state: bool = False,
) -> None:
    """Configure the articulation DOFs referenced by the policy interface.

    The established write order is preserved (drive types, control-mode switch, joint properties,
    default state, gains, simulation limits). Unmatched DOFs from an attached subsystem are left untouched;
    unset simulation limits keep imported USD values and the default root pose is not authored
    here. Passive/mimic policy joints are configured, never commanded.

    Args:
        articulation: Initialized articulation to configure.
        env_config: Env config supplying the per-joint defaults, gains, armatures, and limits.
        policy_joint_names: Policy robot joints to configure.
        joint_control_modes: Commanded joint name to its drive mode.
        effort_modes: Drive effort semantics written on PhysX (Newton has no drive types).
        set_gains: Write the config's stiffness/damping; False keeps the authored drive gains.
        set_limits: Write configured effort/velocity limits (PhysX only); unset entries keep USD.
        reset_to_default_state: Also snap the articulation to its default state at the end.
    """
    dof_names = list(articulation.dof_names)
    policy_joint_set = {str(name) for name in policy_joint_names}
    policy_joint_names = tuple(name for name in dof_names if name in policy_joint_set)
    policy_dof_indices = [index for index, name in enumerate(dof_names) if name in policy_joint_set]
    dof_indices = None if len(policy_dof_indices) == len(dof_names) else policy_dof_indices

    is_newton = SimulationManager.get_active_physics_engine() == "newton"
    if not is_newton:
        articulation.set_dof_drive_types(effort_modes, dof_indices=dof_indices)
    get_physics_simulation_interface().flush_changes()

    control_modes = tuple(dict.fromkeys(joint_control_modes.values()))
    if len(control_modes) == 1:
        articulation.switch_dof_control_mode(control_modes[0], dof_indices=dof_indices)
    else:
        for control_mode in control_modes:
            control_mode_indices = [
                index for index, name in enumerate(dof_names) if joint_control_modes.get(name) == control_mode
            ]
            articulation.switch_dof_control_mode(control_mode, dof_indices=control_mode_indices)

    joint_properties = env_config.joint_properties(policy_joint_names)
    effort_limits = list(joint_properties.effort_limits)
    velocity_limits = list(joint_properties.velocity_limits)
    default_positions = list(joint_properties.default_positions)
    default_velocities = list(joint_properties.default_velocities)

    articulation.set_dof_positions(default_positions, dof_indices=dof_indices)
    articulation.set_dof_velocities(default_velocities, dof_indices=dof_indices)
    property_dof_indices = range(len(dof_names)) if dof_indices is None else dof_indices
    armature_entries = [(index, value) for index, value in enumerate(joint_properties.armature) if value is not None]
    if armature_entries:
        articulation.set_dof_armatures(
            [value for _, value in armature_entries],
            dof_indices=[property_dof_indices[index] for index, _ in armature_entries],
        )
    articulation.set_default_state(
        linear_velocities=[[0.0, 0.0, 0.0]],
        angular_velocities=[[0.0, 0.0, 0.0]],
        dof_positions=default_positions,
        dof_velocities=default_velocities,
        dof_indices=dof_indices,
    )
    if set_gains:
        articulation.set_dof_gains(
            list(joint_properties.stiffness), list(joint_properties.damping), dof_indices=dof_indices
        )

    if set_limits and not is_newton:
        effort_entries = [(index, limit) for index, limit in enumerate(effort_limits) if limit is not None]
        if effort_entries:
            effort_dof_indices = [property_dof_indices[index] for index, _ in effort_entries]
            authored_effort_limits = [limit for _, limit in effort_entries]
            articulation.set_dof_max_efforts(authored_effort_limits, dof_indices=effort_dof_indices)
            get_physics_simulation_interface().flush_changes()

        velocity_entries = [(index, limit) for index, limit in enumerate(velocity_limits) if limit is not None]
        if velocity_entries:
            velocity_dof_indices = [property_dof_indices[index] for index, _ in velocity_entries]
            authored_velocity_limits = [limit for _, limit in velocity_entries]
            articulation.set_dof_max_velocities(authored_velocity_limits, dof_indices=velocity_dof_indices)

    if reset_to_default_state:
        articulation.reset_to_default_state()


def _build_actuators(
    articulation: Articulation,
    env_config: PolicyEnvConfig,
    policy_joint_names: Sequence[str],
) -> ArticulationActuators | None:
    """Build explicit actuator models, or return None when every group uses an engine drive.

    Args:
        articulation: Deployed robot articulation.
        env_config: Parsed env config of the deployed artifact.
        policy_joint_names: Policy joints to configure, in DOF order.

    Returns:
        The resulting :class:`ArticulationActuators | None`.
    """
    specs = env_config.actuator_model_specs(policy_joint_names)
    if not specs:
        return None

    from .utils.newton_actuators import build_newton_actuator_configs

    device = SimulationManager.get_physics_sim_device()
    actuator_configs: list[tuple[object, list[str]]] = []
    for spec in specs:
        network_data = read_artifact_bytes(spec.network_file) if spec.network_file else None
        actuator_configs.extend(build_newton_actuator_configs(spec, network_data, num_robots=1, device=device))

    return ArticulationActuators.from_actuators(
        articulation.paths[0],
        actuator_configs,
        auto_step_pre_physics=False,
        device=device,
    )


# == RobotPolicyRunner =======================================================================


class RobotPolicyRunner:
    """Generic policy deployment driven by a caller-owned physics callback.

    The runner owns the deployment chain (env config, binding, model, controller, actuators, and
    control cadence) but never the stage, timeline, or physics callback. On a due control tick
    (tick zero is due) it reads fresh state and task context, runs the controller, and applies the
    result. Intermediate ticks do nothing: the engine keeps applying the retained command.

    Args:
        spec: Frozen policy spec to deploy.
        prim_path: Stage path where the robot is spawned.
        position: World-frame spawn position; None means spec default -> env config -> asset.
        orientation: World-frame WXYZ spawn orientation, or None for the same chain.
        training_engine: Engine the deployed policy was trained with (``physx``/``newton``),
            or None to match the active physics engine.
        task_state_provider: Factory called at initialize with the articulation, returning an
            object with ``read() -> Mapping``; None for policies without task context.
    """

    def __init__(
        self,
        spec: PolicySpec,
        *,
        prim_path: str,
        position: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        training_engine: str | None = None,
        task_state_provider: Callable[[Articulation], object] | None = None,
    ) -> None:
        # Owned runtime resources, released by close() and the destruction fallback.
        self._controller: IsaacLabPolicyController | None = None
        self._actuators: ArticulationActuators | None = None
        self._task_provider: object | None = None

        # Construction inputs (the spec is fixed for the runner's lifetime).
        self._spec = spec
        self._prim_path = prim_path
        self._position = [float(value) for value in position] if position is not None else None
        self._orientation = [float(value) for value in orientation] if orientation is not None else None
        self._training_engine = training_engine
        self._task_state_provider = task_state_provider

        # Deployment chain resolved by spawn().
        self._engine: str | None = None
        self._env_config: PolicyEnvConfig | None = None
        self._articulation: Articulation | None = None

        # Articulation I/O seams: module functions by default, replaced by fakes in the sim-free tests.
        self._state_reader: Callable[[object, RobotStateReadSpec | None], mg.RobotState] = read_robot_state
        self._state_applier: Callable[[object, mg.RobotState], None] = apply_robot_state
        self._state_read_spec: RobotStateReadSpec | None = None

        self._takes_command = False
        self._initialized = False

        self._tick = 0
        self._held_target: mg.RobotState | None = None

    def __del__(self) -> None:
        """Release policy and actuator resources before destruction."""
        self.close()

    @property
    def articulation(self) -> Articulation | None:
        """The spawned articulation, or None before :meth:`spawn`."""
        return self._articulation

    @property
    def engine(self) -> str | None:
        """The engine the deployed artifact was resolved for, or None before :meth:`spawn`."""
        return self._engine

    @property
    def _artifact(self) -> PolicyArtifact | None:
        """The spec artifact resolved for the deployed engine, or None before :meth:`spawn`."""
        return self._spec.engines[self._engine] if self._engine is not None else None

    @property
    def physics_dt(self) -> float:
        """The physics step size the artifact's env config was exported with (requires spawn())."""
        return self._require_env_config().timing.physics_dt

    @property
    def decimation(self) -> int:
        """Physics ticks per control tick from the artifact's env config (requires spawn())."""
        return self._require_env_config().timing.decimation

    @property
    def env_config(self) -> PolicyEnvConfig:
        """The parsed artifact environment config (requires :meth:`spawn`)."""
        return self._require_env_config()

    # -- lifecycle --------------------------------------------------------------------------

    def spawn(self) -> Articulation:
        """Author the robot (spec/artifact/env-config/USD/pose + ordered spawn props), pre-play.

        Returns:
            The resulting :class:`Articulation`.
        """
        if self._articulation is not None:
            return self._articulation

        spec = self._spec
        active_engine = (SimulationManager.get_active_physics_engine() or "").lower()
        engine = (self._training_engine or active_engine).lower()
        if engine not in spec.engines:
            raise ValueError(
                f"RobotPolicyRunner({self._name()}): the spec declares no artifact for engine {engine!r}; "
                f"available engines are {sorted(spec.engines)}."
            )

        artifact = spec.engines[engine]
        env_config = PolicyEnvConfig.from_file(artifact.env_config_path)
        usd_path = spec.usd_path
        if isinstance(usd_path, Mapping):
            usd_path = usd_path[engine]
        if usd_path is None:
            usd_path = env_config.spawn.usd_path
        if usd_path is None:
            raise ValueError(
                f"RobotPolicyRunner({self._name()}): neither the spec nor the env config names a robot USD path."
            )

        position, orientation = self._resolve_spawn_pose(env_config)
        spawn_policy_robot(self._prim_path, usd_path, env_config.spawn, env_config.newton_shape_defaults)

        self._engine = engine
        self._env_config = env_config
        self._articulation = Articulation(
            self._prim_path,
            positions=position,
            orientations=orientation,
            reset_xform_op_properties=True,
        )
        return self._articulation

    def apply_scene_properties(self, scene_entities: Mapping[str, Articulation] | None = None) -> set[str]:
        """Apply exported startup properties to the policy robot and task articulations, pre-play.

        Args:
            scene_entities: Additional articulations keyed by their exported scene names.

        Returns:
            Names of scene entities that received startup material bindings.
        """
        if self._articulation is None:
            raise RuntimeError("RobotPolicyRunner.apply_scene_properties requires spawn() first.")
        stage = omni.usd.get_context().get_stage()
        env_config = self._require_env_config()
        entities = {"robot": self._articulation}
        entities.update(scene_entities or {})
        for entity in (scene_entities or {}).values():
            for path in entity.paths:
                _apply_newton_import_defaults(stage, path, env_config.newton_shape_defaults)
        return apply_startup_material_events(
            stage,
            env_config,
            entities,
            f"{self._prim_path}/PolicyMaterials",
            root_paths={"robot": self._prim_path},
        )

    def initialize(self) -> None:
        """Initialize the robot and (re)start the policy runtime on a playing timeline.

        The first call derives and binds the interface from the artifact's IO descriptor (or the spec's hook),
        loads the model, and resets the tick machinery; later calls reset only the controller and
        runtime from the articulation's current state. Callers own any reset or teleport and must
        perform it before this call. Use :meth:`restart_from_default_state` when the configured
        default state is the desired reset state. Raises when unspawned or rejected.
        """
        if self._articulation is None:
            raise RuntimeError("RobotPolicyRunner.initialize requires spawn() to have authored the robot first.")
        if self._controller is None:
            self._build_runtime()
        self._reset_runtime()

    def restart_from_default_state(self, command: Sequence[float] | None = None) -> None:
        """Reset the robot and policy runtime, then establish the first control target.

        On the first call, the runtime is built before the articulation reset so exported defaults
        have been configured. The controller and actuator state are then reset from that clean
        articulation state, and one policy step writes targets before the next physics tick.

        Args:
            command: Base-frame planar velocity ``[vx, vy, wz]``, or None for policies without
                a command channel.
        """
        if self._articulation is None:
            raise RuntimeError(
                "RobotPolicyRunner.restart_from_default_state requires spawn() to have authored the robot first."
            )
        if self._controller is None:
            self._build_runtime()
        self._validate_command(command)
        try:
            self._articulation.reset_to_default_state()
            self._reset_runtime()
            self.step(self.physics_dt, command)
        except Exception:
            self._initialized = False
            self._tick = 0
            self._held_target = None
            if self._actuators is not None:
                self._actuators.disable_auto_step_pre_physics()
            raise

    def reset(self) -> None:
        """Reset robot, task state, controller feedback, actuators, and cadence in place."""
        if not self._initialized:
            raise RuntimeError("RobotPolicyRunner.reset requires initialize() first.")
        self._articulation.reset_to_default_state()
        task_reset = getattr(self._task_provider, "reset", None)
        if callable(task_reset):
            task_reset()
        simulation_view = SimulationManager.get_physics_simulation_view()
        if simulation_view is not None:
            simulation_view.update_articulations_kinematic()
        self._reset_runtime()

    def step(self, dt: float, command: Sequence[float] | None = None) -> None:
        """Advance the runner by one physics tick, called once per physics callback.

        The artifact timing establishes the controller clock; ``dt`` is retained for the physics-callback API
        but is not a second timing authority. ``command`` is the base-frame planar velocity
        ``[vx, vy, wz]`` when the policy observes a command channel and None otherwise. Control
        ticks run inference and write the resulting desired state; the engine holds that command
        until the next control tick, so intermediate ticks write nothing. Raises RuntimeError when
        uninitialized and ValueError on bad input.

        Args:
            dt: Physics step duration.
            command: Base-frame planar velocity ``[vx, vy, wz]``, or None.
        """
        del dt
        if not self._initialized:
            raise RuntimeError("RobotPolicyRunner.step() requires a successful initialize().")

        validated_command = self._validate_command(command)

        if self._tick % self.decimation == 0:
            self._control_tick(validated_command)
            self._state_applier(self._articulation, self._held_target)
        self._tick += 1

    def close(self) -> None:
        """Release the controller, model, and actuators."""
        self._initialized = False
        if self._actuators is not None:
            self._actuators.close()
            self._actuators = None
        self._held_target = None

        if self._controller is not None:
            self._controller.close()
            self._controller = None
        self._task_provider = None

    # -- runtime assembly --------------------------------------------------------------------

    def _build_runtime(self) -> None:
        """Build the runtime on the first initialize.

        Derive and bind the interface, configure the robot, load the model, and assemble the
        controller and actuators.
        """
        articulation = self._articulation
        env_config = self._require_env_config()
        spec = self._spec

        # Resolve the artifact interface against the articulation's actual joint space.
        if spec.binding is not None:
            policy_binding = spec.binding(env_config)
        else:
            policy_binding = derive_binding(self._artifact.load_descriptor())
        robot_joint_space = tuple(str(name) for name in articulation.dof_names)
        bound_policy = bind_policy(policy_binding, robot_joint_space=robot_joint_space)
        referenced_joints = {
            joint_name
            for term in policy_binding.observation_terms + policy_binding.action_terms
            for joint_name in term.joint_names
        }
        policy_joint_names = tuple(name for name in robot_joint_space if name in referenced_joints)

        # Configure the articulation before any runtime callbacks can become active.
        initialize_articulation(articulation, env_config, policy_joint_names, bound_policy.joint_control_modes)
        resolved_position, resolved_orientation = self._resolve_spawn_pose(env_config)
        default_root_state: dict[str, list[list[float]]] = {}
        if resolved_position is not None:
            default_root_state["positions"] = [list(resolved_position)]
        if resolved_orientation is not None:
            default_root_state["orientations"] = [list(resolved_orientation)]
        if default_root_state:
            articulation.set_default_state(**default_root_state)
        if spec.zero_targets_on_initialize:
            articulation.reset_to_default_state()
            self._zero_targets()

        task_provider = None
        if self._task_state_provider is not None:
            task_provider = self._task_state_provider(articulation)
        accepts_command = any(term.semantic == "generated_commands" for term in policy_binding.observation_terms)

        # Construct every owned resource before publishing any of them on the runner.
        policy_model = load_policy_model(
            self._artifact,
            bound_policy,
            SimulationManager.get_physics_sim_device(),
        )
        controller: IsaacLabPolicyController | None = None
        actuators: ArticulationActuators | None = None
        try:
            controller = IsaacLabPolicyController(policy_model, bound_policy)
            actuators = _build_actuators(articulation, env_config, policy_joint_names)
        except Exception:
            if actuators is not None:
                actuators.close()
            if controller is not None:
                controller.close()
            else:
                policy_model.close()
            raise

        self._controller = controller
        self._actuators = actuators
        self._task_provider = task_provider
        self._state_read_spec = bound_policy.estimated_state_read_spec
        self._takes_command = accepts_command

    # -- per-tick path ------------------------------------------------------------------------

    def _reset_runtime(self) -> None:
        """Reset the controller and actuator state from the current articulation state."""
        self._initialized = False
        self._tick = 0
        self._held_target = None
        if self._actuators is not None:
            self._actuators.disable_auto_step_pre_physics()
            self._actuators.reset()

        estimated_state = self._state_reader(self._articulation, self._state_read_spec)
        accepted = self._controller.reset(estimated_state, None, 0.0)
        if not accepted:
            raise RuntimeError(
                f"RobotPolicyRunner({self._name()}): the controller rejected the reset; see the error log."
            )

        self._initialized = True

    def _control_tick(self, command: np.ndarray | None) -> None:
        """Read current state, run inference, and retain the next desired state.

        Args:
            command: Base-frame planar velocity ``[vx, vy, wz]``, or None.
        """
        estimated_state, task_context = self._read_state_and_context()
        setpoint = self._make_setpoint(command)
        controller_time = self._tick * self.physics_dt
        self._held_target = self._controller.forward(
            estimated_state,
            setpoint,
            controller_time,
            context=task_context,
        )
        if self._actuators is not None:
            self._actuators.enable_auto_step_pre_physics()

    def _read_state_and_context(self) -> tuple[mg.RobotState, Mapping[str, object] | None]:
        """Read the articulation state and optional task-specific controller context.

        Returns:
            The resulting tuple.
        """
        estimated_state = self._state_reader(self._articulation, self._state_read_spec)
        task_context = self._task_provider.read() if self._task_provider is not None else None
        return estimated_state, task_context

    # -- helpers ------------------------------------------------------------------------------

    def _name(self) -> str:
        return repr(self._spec.name or "<unnamed>")

    def _validate_command(self, command: Sequence[float] | None) -> np.ndarray | None:
        if self._takes_command:
            if command is None:
                raise ValueError(f"RobotPolicyRunner({self._name()}): this policy takes a [vx, vy, wz] command.")
            values = np.asarray(command, dtype=np.float32).reshape(-1)
            if values.shape[0] != 3:
                raise ValueError(f"Policy commands are [vx, vy, wz] vectors, got {values.shape[0]} elements.")
            return values

        if command is not None:
            raise ValueError(f"RobotPolicyRunner({self._name()}): this policy takes no command.")
        return None

    def _make_setpoint(self, command: np.ndarray | None) -> mg.RobotState | None:
        if command is None:
            return None

        linear_velocity = np.array([command[0], command[1], 0.0], dtype=np.float32)
        angular_velocity = np.array([0.0, 0.0, command[2]], dtype=np.float32)
        with wp.ScopedDevice("cpu"):
            root = mg.RootState(
                linear_velocity=wp.array(linear_velocity, dtype=wp.float32),
                angular_velocity=wp.array(angular_velocity, dtype=wp.float32),
            )
        return mg.RobotState(root=root)

    def _resolve_spawn_pose(self, env_config: PolicyEnvConfig) -> tuple[list[float] | None, list[float] | None]:
        spec = self._spec
        position = self._position
        orientation = self._orientation
        if position is None and spec.default_spawn_position is not None:
            position = list(spec.default_spawn_position)
        if orientation is None and spec.default_spawn_orientation is not None:
            orientation = list(spec.default_spawn_orientation)

        initial_position, initial_orientation = env_config.initial_root_pose
        if position is None:
            position = initial_position
        if orientation is None:
            orientation = initial_orientation

        return position, orientation

    def _zero_targets(self) -> None:
        dof_count = len(self._articulation.dof_names)
        zeros = np.zeros((1, dof_count), dtype=np.float32)

        self._articulation.set_dof_position_targets(zeros)
        self._articulation.set_dof_velocity_targets(zeros)
        self._articulation.set_dof_efforts(zeros)

    def _require_env_config(self) -> PolicyEnvConfig:
        if self._env_config is None:
            raise RuntimeError("RobotPolicyRunner: call spawn() first; the env config is resolved from the artifact.")
        return self._env_config
