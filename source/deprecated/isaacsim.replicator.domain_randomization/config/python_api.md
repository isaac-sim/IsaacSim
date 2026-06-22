# Public API for module isaacsim.replicator.domain_randomization:

## Classes

- class Extension(omni.ext.IExt)
  - def on_startup(self)
  - def on_shutdown(self)

## Variables

- ARTICULATION_ATTRIBUTES: List
- RIGID_PRIM_ATTRIBUTES: List
- SIMULATION_CONTEXT_ATTRIBUTES: List
- TENDON_ATTRIBUTES: List

# Public API for module isaacsim.replicator.domain_randomization.scripts.context:

## Classes

- class ReplicatorIsaacContext
  - def __init__(self, num_envs: Any, action_graph_entry_node: Any)
  - def trigger_randomization(self, reset_inds: Any)
  - [property] def reset_inds(self) -> list[int]
  - def get_tendon_exec_context(self) -> Any
  - def add_tendon_exec_context(self, node: Any)

## Functions

- def initialize_context(num_envs: Any, action_graph_entry_node: Any)
- def get_reset_inds() -> list[int]
- def trigger_randomization(reset_inds: Any)

# Public API for module isaacsim.replicator.domain_randomization.scripts.gate:

## Functions

- def on_interval(interval: Any) -> ReplicatorItem
- def on_env_reset() -> ReplicatorItem

# Public API for module isaacsim.replicator.domain_randomization.scripts.physics_view:

## Classes

- class Articulation(XFormPrim)
  - def __init__(self, prim_paths_expr: str | list[str], name: str = 'articulation_prim_view', positions: np.ndarray | torch.Tensor | wp.array | None = None, translations: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, scales: np.ndarray | torch.Tensor | wp.array | None = None, visibilities: np.ndarray | torch.Tensor | wp.array | None = None, reset_xform_properties: bool = True)
  - [property] def num_dof(self) -> int | None
  - [property] def num_bodies(self) -> int | None
  - [property] def num_shapes(self) -> int | None
  - [property] def num_joints(self) -> int | None
  - [property] def num_fixed_tendons(self) -> int | None
  - [property] def body_names(self) -> list[str] | None
  - [property] def dof_names(self) -> list[str] | None
  - [property] def joint_names(self) -> list[str] | None
  - def is_physics_handle_valid(self) -> bool
  - def get_body_index(self, body_name: str) -> int
  - def get_dof_index(self, dof_name: str) -> int
  - def get_dof_types(self, dof_names: list[str] = None) -> list[str]
  - def get_dof_limits(self) -> np.ndarray | torch.Tensor
  - def get_drive_types(self) -> np.ndarray | torch.Tensor
  - def get_joint_index(self, joint_name: str) -> int
  - def get_link_index(self, link_name: str) -> int
  - def set_friction_coefficients(self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def get_friction_coefficients(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.array
  - def set_armatures(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def get_armatures(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_articulation_body_count(self) -> int
  - def set_joint_position_targets(self, positions: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def set_joint_positions(self, positions: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def set_joint_velocity_targets(self, velocities: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def set_joint_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def set_joint_efforts(self, efforts: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def get_applied_joint_efforts(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_measured_joint_efforts(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_measured_joint_forces(self, indices: np.ndarray | list | torch.Tensor | None = None, joint_indices: np.ndarray | list | torch.Tensor | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor
  - def get_joint_positions(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_joint_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def apply_action(self, control_actions: ArticulationActions, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_applied_actions(self, clone: bool = True) -> ArticulationActions
  - def set_world_poses(self, positions: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, usd: bool = True)
  - def get_world_poses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, usd: bool = True) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]
  - def get_local_poses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]
  - def set_local_poses(self, translations: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_linear_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_linear_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_angular_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_angular_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_joints_default_state(self, positions: np.ndarray | torch.Tensor | wp.array | None = None, velocities: np.ndarray | torch.Tensor | wp.array | None = None, efforts: np.ndarray | torch.Tensor | wp.array | None = None)
  - def get_joints_default_state(self) -> JointsState
  - def get_joints_state(self) -> JointsState
  - def get_effort_modes(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None) -> list[str]
  - def set_effort_modes(self, mode: str, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | None = None, joint_names: list[str] | None = None)
  - def set_max_efforts(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def get_max_efforts(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_max_joint_velocities(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def get_joint_max_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_gains(self, kps: np.ndarray | torch.Tensor | wp.array | None = None, kds: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, save_to_usd: bool = False)
  - def get_gains(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> tuple[np.ndarray | torch.Tensor, np.ndarray | torch.Tensor, wp.indexedarray | wp.index]
  - def switch_control_mode(self, mode: str, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None)
  - def switch_dof_control_mode(self, mode: str, dof_index: int, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_solver_position_iteration_counts(self, counts: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_solver_position_iteration_counts(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_solver_velocity_iteration_counts(self, counts: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_solver_velocity_iteration_counts(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_stabilization_thresholds(self, thresholds: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_stabilization_thresholds(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_enabled_self_collisions(self, flags: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_enabled_self_collisions(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_sleep_thresholds(self, thresholds: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_sleep_thresholds(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_jacobian_shape(self) -> np.ndarray | torch.Tensor | wp.array
  - def get_mass_matrix_shape(self) -> np.ndarray | torch.Tensor | wp.array
  - def get_jacobians(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_mass_matrices(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_coriolis_and_centrifugal_forces(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_generalized_gravity_forces(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, joint_names: list[str] | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_masses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_inv_masses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_coms(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_inertias(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_inv_inertias(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_body_disable_gravity(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_body_masses(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_body_inertias(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_body_coms(self, positions: np.ndarray | torch.Tensor | wp.array = None, orientations: np.ndarray | torch.Tensor | wp.array = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_body_disable_gravity(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, body_indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_fixed_tendon_stiffnesses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_fixed_tendon_dampings(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_fixed_tendon_limit_stiffnesses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_fixed_tendon_limits(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_fixed_tendon_rest_lengths(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_fixed_tendon_offsets(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_fixed_tendon_properties(self, stiffnesses: np.ndarray | torch.Tensor | wp.array = None, dampings: np.ndarray | torch.Tensor | wp.array = None, limit_stiffnesses: np.ndarray | torch.Tensor | wp.array = None, limits: np.ndarray | torch.Tensor | wp.array = None, rest_lengths: np.ndarray | torch.Tensor | wp.array = None, offsets: np.ndarray | torch.Tensor | wp.array = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def pause_motion(self)
  - def resume_motion(self)
  - def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None)

- class RigidPrim(XFormPrim)
  - def __init__(self, prim_paths_expr: str | list[str], name: str = 'rigid_prim_view', positions: np.ndarray | torch.Tensor | wp.array | None = None, translations: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, scales: np.ndarray | torch.Tensor | wp.array | None = None, visibilities: np.ndarray | torch.Tensor | wp.array | None = None, reset_xform_properties: bool = True, masses: np.ndarray | torch.Tensor | wp.array | None = None, densities: np.ndarray | torch.Tensor | wp.array | None = None, linear_velocities: np.ndarray | torch.Tensor | wp.array | None = None, angular_velocities: np.ndarray | torch.Tensor | wp.array | None = None, track_contact_forces: bool = False, prepare_contact_sensors: bool = True, disable_stablization: bool = True, contact_filter_prim_paths_expr: list[str] | None = None, max_contact_count: int = 0)
  - [property] def num_shapes(self) -> int
  - def is_physics_handle_valid(self) -> bool
  - def set_world_poses(self, positions: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, usd: bool = True)
  - def get_world_poses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, usd: bool = True) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]
  - def get_local_poses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> tuple[np.ndarray, np.ndarray] | tuple[torch.Tensor, torch.Tensor] | tuple[wp.indexedarray, wp.indexedarray]
  - def set_local_poses(self, translations: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_linear_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_linear_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_angular_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_angular_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_velocities(self, velocities: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_velocities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def apply_forces(self, forces: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, is_global: bool = True)
  - def apply_forces_and_torques_at_pos(self, forces: np.ndarray | torch.Tensor | wp.array | None = None, torques: np.ndarray | torch.Tensor | wp.array | None = None, positions: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, is_global: bool = True)
  - def get_masses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def get_inv_masses(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def get_coms(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def get_inertias(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def get_inv_inertias(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def set_masses(self, masses: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_inertias(self, values: np.ndarray | torch.Tensor | wp.array, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_coms(self, positions: np.ndarray | torch.Tensor | wp.array = None, orientations: np.ndarray | torch.Tensor | wp.array = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_densities(self, densities: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_densities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def set_sleep_thresholds(self, thresholds: np.ndarray | torch.Tensor | wp.array | None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_sleep_thresholds(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> np.ndarray | torch.Tensor | wp.indexedarray
  - def enable_rigid_body_physics(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def disable_rigid_body_physics(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def enable_gravities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def disable_gravities(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def set_default_state(self, positions: np.ndarray | torch.Tensor | wp.array | None = None, orientations: np.ndarray | torch.Tensor | wp.array | None = None, linear_velocities: np.ndarray | torch.Tensor | wp.array | None = None, angular_velocities: np.ndarray | torch.Tensor | wp.array | None = None, indices: np.ndarray | list | torch.Tensor | wp.array | None = None)
  - def get_default_state(self) -> DynamicsViewState
  - def get_current_dynamic_state(self) -> DynamicsViewState
  - def get_net_contact_forces(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, dt: float = 1.0) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def get_contact_force_matrix(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, dt: float = 1.0) -> np.ndarray | torch.Tensor | wp.indexedarray | None
  - def get_contact_force_data(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, dt: float = 1.0) -> tuple[np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray]
  - def get_friction_data(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None, clone: bool = True, dt: float = 1.0) -> tuple[np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray, np.ndarray | torch.Tensor | wp.indexedarray]
  - def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None)

## Functions

- def import_module(name: str) -> ModuleType
- def quats_to_euler_angles(quaternions: np.ndarray, degrees: bool = False, extrinsic: bool = True, device: object = None) -> np.ndarray
- def get_euler_xyz(q: torch.Tensor, extrinsic: bool = True) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]
- def trigger_randomization(reset_inds: Any)
- def register_simulation_context(simulation_context: isaacsim.core.api.SimulationContext | isaacsim.core.api.World)
- def register_rigid_prim_view(rigid_prim_view: RigidPrim)
- def register_articulation_view(articulation_view: Articulation)
- def step_randomization(reset_inds: Optional[list | np.ndarray | torch.Tensor] = None)
- def randomize_rigid_prim_view(view_name: str, operation: str = 'direct', num_buckets: int = None, position: ReplicatorItem = None, orientation: ReplicatorItem = None, linear_velocity: ReplicatorItem = None, angular_velocity: ReplicatorItem = None, velocity: ReplicatorItem = None, force: ReplicatorItem = None, mass: ReplicatorItem = None, inertia: ReplicatorItem = None, material_properties: ReplicatorItem = None, contact_offset: ReplicatorItem = None, rest_offset: ReplicatorItem = None)
- def randomize_articulation_view(view_name: str, operation: str = 'direct', num_buckets: int = None, stiffness: ReplicatorItem = None, damping: ReplicatorItem = None, joint_friction: ReplicatorItem = None, position: ReplicatorItem = None, orientation: ReplicatorItem = None, linear_velocity: ReplicatorItem = None, angular_velocity: ReplicatorItem = None, velocity: ReplicatorItem = None, joint_positions: ReplicatorItem = None, joint_velocities: ReplicatorItem = None, lower_dof_limits: ReplicatorItem = None, upper_dof_limits: ReplicatorItem = None, max_efforts: ReplicatorItem = None, joint_armatures: ReplicatorItem = None, joint_max_velocities: ReplicatorItem = None, joint_efforts: ReplicatorItem = None, body_masses: ReplicatorItem = None, body_inertias: ReplicatorItem = None, material_properties: ReplicatorItem = None, tendon_stiffnesses: ReplicatorItem = None, tendon_dampings: ReplicatorItem = None, tendon_limit_stiffnesses: ReplicatorItem = None, tendon_lower_limits: ReplicatorItem = None, tendon_upper_limits: ReplicatorItem = None, tendon_rest_lengths: ReplicatorItem = None, tendon_offsets: ReplicatorItem = None)
- def randomize_simulation_context(operation: str = 'direct', gravity: ReplicatorItem = None)

## Variables

- TENDON_ATTRIBUTES: List
- torch: Unknown

# Public API for module isaacsim.replicator.domain_randomization.scripts.trigger:

## Functions

- def initialize_context(num_envs: Any, action_graph_entry_node: Any)
- def on_rl_frame(num_envs: int) -> Any

# Public API for module isaacsim.replicator.domain_randomization.scripts.utils:

## Classes

- class NumpyEncoder(json.JSONEncoder)
  - def default(self, obj: Any) -> Any

## Functions

- def set_distribution_params(distribution: ReplicatorItem, parameters: dict)
- def get_distribution_params(distribution: ReplicatorItem, parameters: list[str]) -> list
- def get_semantics(num_semantics: Any, num_semantic_tokens: Any, instance_semantic_map: Any, min_semantic_idx: Any, max_semantic_hierarchy_depth: Any, semantic_token_map: Any, required_semantic_types: Any) -> tuple
- def get_image_space_points(points: Any, view_proj_matrix: Any) -> np.ndarray
- def calculate_truncation_ratio_simple(corners: Any, img_width: int, img_height: int) -> float
