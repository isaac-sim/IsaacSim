# Public API for module isaacsim.replicator.experimental.domain_randomization:

## Variables

- ARTICULATION_ATTRIBUTES: List
- RIGID_PRIM_ATTRIBUTES: List
- SIMULATION_CONTEXT_ATTRIBUTES: List
- TENDON_ATTRIBUTES: List

# Public API for module isaacsim.replicator.experimental.domain_randomization.scripts.context:

## Classes

- class ReplicatorIsaacContext
  - def __init__(self, num_envs: Any, action_graph_entry_node: Any)
  - def trigger_randomization(self, reset_inds: Any)
  - [property] def reset_inds(self) -> Any
  - def get_tendon_exec_context(self) -> Any
  - def add_tendon_exec_context(self, node: Any)

## Functions

- def initialize_context(num_envs: Any, action_graph_entry_node: Any)
- def get_reset_inds() -> Any
- def resolve_context() -> Any
- def trigger_randomization(reset_inds: Any)

# Public API for module isaacsim.replicator.experimental.domain_randomization.scripts.gate:

## Functions

- def on_interval(interval: Any) -> Any
- def on_env_reset() -> Any

# Public API for module isaacsim.replicator.experimental.domain_randomization.scripts.physics_view:

## Classes

- class Articulation(XformPrim)
  - def __init__(self, paths: str | list[str])
  - [property] def num_dofs(self) -> int
  - [property] def dof_names(self) -> list[str]
  - [property] def dof_paths(self) -> list[list[str]]
  - [property] def dof_types(self) -> list[omni.physics.tensors.DofType]
  - [property] def num_joints(self) -> int
  - [property] def joint_names(self) -> list[str]
  - [property] def joint_paths(self) -> list[list[str]]
  - [property] def joint_types(self) -> list[omni.physics.tensors.JointType]
  - [property] def num_links(self) -> int
  - [property] def link_names(self) -> list[str]
  - [property] def link_paths(self) -> list[list[str]]
  - [property] def num_shapes(self) -> int
  - [property] def num_fixed_tendons(self) -> int
  - [property] def jacobian_matrix_shape(self) -> tuple[int, int, int]
  - [property] def mass_matrix_shape(self) -> tuple[int, int]
  - static def fetch_articulation_root_api_prim_paths(paths: str | list[str]) -> list[str | None]
  - def is_physics_tensor_entity_valid(self) -> bool
  - def is_physics_tensor_entity_initialized(self) -> bool
  - def get_link_indices(self, names: str | list[str]) -> wp.array
  - def get_joint_indices(self, names: str | list[str]) -> wp.array
  - def get_dof_indices(self, names: str | list[str]) -> wp.array
  - def get_dof_limits(self) -> tuple[wp.array, wp.array]
  - def set_dof_limits(self, lower: float | list | np.ndarray | wp.array | None = None, upper: float | list | np.ndarray | wp.array | None = None)
  - def get_dof_friction_properties(self) -> tuple[wp.array, wp.array, wp.array]
  - def set_dof_friction_properties(self, static_frictions: float | list | np.ndarray | wp.array | None = None, dynamic_frictions: float | list | np.ndarray | wp.array | None = None, viscous_frictions: float | list | np.ndarray | wp.array | None = None)
  - def get_dof_drive_model_properties(self) -> tuple[wp.array, wp.array, wp.array]
  - def set_dof_drive_model_properties(self, speed_effort_gradients: float | list | np.ndarray | wp.array | None = None, maximum_actuator_velocities: float | list | np.ndarray | wp.array | None = None, velocity_dependent_resistances: float | list | np.ndarray | wp.array | None = None)
  - def set_dof_armatures(self, armatures: float | list | np.ndarray | wp.array)
  - def get_dof_armatures(self) -> wp.array
  - def set_dof_position_targets(self, positions: float | list | np.ndarray | wp.array)
  - def set_dof_positions(self, positions: float | list | np.ndarray | wp.array)
  - def set_dof_velocity_targets(self, velocities: float | list | np.ndarray | wp.array)
  - def set_dof_velocities(self, velocities: float | list | np.ndarray | wp.array)
  - def set_dof_efforts(self, efforts: float | list | np.ndarray | wp.array)
  - def get_dof_efforts(self) -> wp.array
  - def get_dof_projected_joint_forces(self) -> wp.array
  - def get_link_incoming_joint_force(self) -> tuple[wp.array, wp.array]
  - def get_dof_positions(self) -> wp.array
  - def get_dof_position_targets(self) -> wp.array
  - def get_dof_velocities(self) -> wp.array
  - def get_dof_velocity_targets(self) -> wp.array
  - def set_world_poses(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def get_world_poses(self) -> tuple[wp.array, wp.array]
  - def get_local_poses(self) -> tuple[wp.array, wp.array]
  - def set_local_poses(self, translations: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def set_velocities(self, linear_velocities: list | np.ndarray | wp.array | None = None, angular_velocities: list | np.ndarray | wp.array | None = None)
  - def get_velocities(self) -> tuple[wp.array, wp.array]
  - def set_default_state(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None, linear_velocities: list | np.ndarray | wp.array | None = None, angular_velocities: list | np.ndarray | wp.array | None = None, dof_positions: float | list | np.ndarray | wp.array | None = None, dof_velocities: float | list | np.ndarray | wp.array | None = None, dof_efforts: float | list | np.ndarray | wp.array | None = None)
  - def get_default_state(self) -> tuple[wp.array | None, wp.array | None, wp.array | None, wp.array | None, wp.array | None, wp.array | None, wp.array | None]
  - def reset_to_default_state(self)
  - def get_dof_drive_types(self) -> list[list[str]]
  - def set_dof_drive_types(self, types: str | list[list[str]])
  - def set_dof_max_efforts(self, max_efforts: float | list | np.ndarray | wp.array)
  - def get_dof_max_efforts(self) -> wp.array
  - def set_dof_max_velocities(self, max_velocities: float | list | np.ndarray | wp.array)
  - def get_dof_max_velocities(self) -> wp.array
  - def set_dof_gains(self, stiffnesses: float | list | np.ndarray | wp.array | None = None, dampings: float | list | np.ndarray | wp.array | None = None)
  - def get_dof_gains(self) -> tuple[wp.array, wp.array]
  - def switch_dof_control_mode(self, mode: str)
  - def set_solver_iteration_counts(self, position_counts: int | list | np.ndarray | wp.array | None = None, velocity_counts: int | list | np.ndarray | wp.array | None = None)
  - def get_solver_iteration_counts(self) -> tuple[wp.array, wp.array]
  - def set_stabilization_thresholds(self, thresholds: float | list | np.ndarray | wp.array)
  - def get_stabilization_thresholds(self) -> wp.array
  - def set_enabled_self_collisions(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_self_collisions(self) -> wp.array
  - def set_sleep_thresholds(self, thresholds: float | list | np.ndarray | wp.array)
  - def get_sleep_thresholds(self) -> wp.array
  - def get_jacobian_matrices(self) -> wp.array
  - def get_mass_matrices(self) -> wp.array
  - def get_dof_coriolis_and_centrifugal_compensation_forces(self) -> wp.array
  - def get_dof_gravity_compensation_forces(self) -> wp.array
  - def get_link_masses(self) -> wp.array
  - def get_link_coms(self) -> tuple[wp.array, wp.array]
  - def get_link_inertias(self) -> wp.array
  - def get_link_enabled_gravities(self) -> wp.array
  - def set_link_masses(self, masses: float | list | np.ndarray | wp.array)
  - def set_link_inertias(self, inertias: list | np.ndarray | wp.array)
  - def set_link_coms(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def set_link_enabled_gravities(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_fixed_tendon_stiffnesses(self) -> wp.array
  - def get_fixed_tendon_dampings(self) -> wp.array
  - def get_fixed_tendon_limit_stiffnesses(self) -> wp.array
  - def get_fixed_tendon_limits(self) -> tuple[wp.array, wp.array]
  - def get_fixed_tendon_rest_lengths(self) -> wp.array
  - def get_fixed_tendon_offsets(self) -> wp.array
  - def set_fixed_tendon_properties(self)
  - def initialize_cpp_data_view(self)

- class RigidPrim(XformPrim)
  - def __init__(self, paths: str | list[str])
  - [property] def num_shapes(self) -> int
  - [property] def num_contact_filters(self) -> int
  - def is_physics_tensor_entity_valid(self) -> bool
  - def set_world_poses(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def get_world_poses(self) -> tuple[wp.array, wp.array]
  - def get_local_poses(self) -> tuple[wp.array, wp.array]
  - def set_local_poses(self, translations: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def set_velocities(self, linear_velocities: list | np.ndarray | wp.array | None = None, angular_velocities: list | np.ndarray | wp.array | None = None)
  - def get_velocities(self) -> tuple[wp.array, wp.array]
  - def apply_forces(self, forces: list | np.ndarray | wp.array)
  - def apply_forces_and_torques_at_pos(self, forces: list | np.ndarray | wp.array | None = None, torques: list | np.ndarray | wp.array | None = None)
  - def get_masses(self) -> wp.array
  - def get_coms(self) -> tuple[wp.array, wp.array]
  - def get_inertias(self) -> wp.array
  - def set_masses(self, masses: float | list | np.ndarray | wp.array)
  - def set_inertias(self, inertias: list | np.ndarray | wp.array)
  - def set_coms(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def set_densities(self, densities: float | list | np.ndarray | wp.array)
  - def get_densities(self) -> wp.array
  - def set_sleep_thresholds(self, thresholds: float | list | np.ndarray | wp.array)
  - def get_sleep_thresholds(self) -> wp.array
  - def set_enabled_rigid_bodies(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_rigid_bodies(self) -> wp.array
  - def set_enabled_gravities(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_gravities(self) -> wp.array
  - def set_enabled_contact_tracking(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_contact_tracking(self) -> wp.array
  - def get_net_contact_forces(self) -> wp.array
  - def get_contact_force_matrix(self) -> wp.array
  - def get_contact_force_data(self) -> tuple[wp.array, wp.array, wp.array, wp.array, wp.array, wp.array]
  - def get_friction_data(self) -> tuple[wp.array, wp.array, wp.array, wp.array]
  - def get_raw_contact_data(self) -> tuple[wp.array, wp.array, wp.array, wp.array, wp.array, wp.array, wp.array]
  - def get_actor_paths_from_ids(self, actor_ids: wp.array) -> list[str]
  - def set_default_state(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None, linear_velocities: list | np.ndarray | wp.array | None = None, angular_velocities: list | np.ndarray | wp.array | None = None)
  - def get_default_state(self) -> tuple[wp.array | None, wp.array | None, wp.array | None, wp.array | None]
  - def reset_to_default_state(self)
  - def initialize_cpp_data_view(self)

- class SimulationManager
  - class def get_active_physics_engine(cls) -> Literal[physx, newton, remotesim]
  - class def get_default_engine(cls) -> str
  - class def get_available_physics_engines(cls, verbose: bool = False) -> list[tuple[str, bool]]
  - class def switch_physics_engine(cls, engine_name: Literal[physx, newton, remotesim], verbose: bool = False) -> bool
  - class def initialize_physics(cls)
  - class def invalidate_physics(cls)
  - class def setup_simulation(cls, dt: float | None = None, device: str | wp.Device | None = None)
  - class def get_physics_scenes(cls) -> list[PhysicsScene]
  - class def get_physics_simulation_view(cls) -> Any | None
  - class def get_simulation_time(cls) -> float
  - class def get_num_physics_steps(cls) -> int
  - class def is_simulating(cls) -> bool
  - class def is_paused(cls) -> bool
  - class def step(cls)
  - class def set_device(cls, device: str | wp.Device)
  - class def get_device(cls) -> wp.Device
  - class def enable_fabric(cls, enable: bool)
  - class def is_fabric_enabled(cls) -> bool
  - class def register_callback(cls, callback: Callable, event: SimulationEvent | IsaacEvents, **kwargs: Any) -> int
  - class def deregister_callback(cls, uid: int) -> bool
  - class def deregister_all_callbacks(cls)
  - class def enable_usd_notice_handler(cls, enable: bool)
  - class def enable_fabric_usd_notice_handler(cls, stage_id: int, enable: bool)
  - class def is_fabric_usd_notice_handler_enabled(cls, stage_id: int) -> bool
  - class def assets_loading(cls) -> bool
  - class def enable_default_callbacks(cls)
  - class def enable_all_default_callbacks(cls, enable: bool = True)
  - class def is_default_callback_enabled(cls, callback_name: str) -> bool
  - class def get_default_callback_status(cls) -> dict
  - class def enable_post_warm_start_callback(cls, enable: bool = True)
  - class def enable_warm_start_callback(cls, enable: bool = True)
  - class def enable_on_stop_callback(cls, enable: bool = True)
  - class def enable_stage_open_callback(cls, enable: bool = True)
  - class def set_backend(cls, val: str)
  - class def get_backend(cls) -> str
  - class def get_physics_sim_view(cls) -> Any
  - class def set_default_physics_scene(cls, physics_scene_prim_path: str)
  - class def get_default_physics_scene(cls) -> str
  - class def set_physics_sim_device(cls, val: str)
  - class def get_physics_sim_device(cls) -> str
  - class def set_physics_dt(cls, dt: float = 1.0 / 60.0, physics_scene: str = None)
  - class def get_physics_dt(cls, physics_scene: str | None = None) -> float
  - class def get_broadphase_type(cls, physics_scene: str | None = None) -> str
  - class def set_broadphase_type(cls, val: str, physics_scene: str | None = None)
  - class def enable_ccd(cls, flag: bool, physics_scene: str | None = None)
  - class def is_ccd_enabled(cls, physics_scene: str | None = None) -> bool
  - class def enable_gpu_dynamics(cls, flag: bool, physics_scene: str | None = None)
  - class def is_gpu_dynamics_enabled(cls, physics_scene: str | None = None) -> bool
  - class def set_solver_type(cls, solver_type: str, physics_scene: str | None = None)
  - class def get_solver_type(cls, physics_scene: str | None = None) -> str
  - class def enable_stablization(cls, flag: bool, physics_scene: str | None = None)
  - class def is_stablization_enabled(cls, physics_scene: str = None) -> bool

## Functions

- def quaternion_to_euler_angles(quaternion: list | np.ndarray | wp.array) -> wp.array
- def trigger_randomization(reset_inds: Any)
- def cleanup()
- def resolve_rigid_prim_view(view_name: Any) -> Any
- def resolve_articulation_view(view_name: Any) -> Any
- def resolve_physics_sim_view() -> Any
- def register_simulation_context(simulation_context: Optional[type[SimulationManager]] = None)
- def register_rigid_prim_view(rigid_prim_view: RigidPrim, name: str)
- def register_articulation_view(articulation_view: Articulation, name: str)
- def step_randomization(reset_inds: Optional[list | np.ndarray] = None)
- def randomize_rigid_prim_view(view_name: str, operation: str = 'direct', num_buckets: int = None, position: ReplicatorItem = None, orientation: ReplicatorItem = None, linear_velocity: ReplicatorItem = None, angular_velocity: ReplicatorItem = None, velocity: ReplicatorItem = None, force: ReplicatorItem = None, mass: ReplicatorItem = None, inertia: ReplicatorItem = None, material_properties: ReplicatorItem = None, contact_offset: ReplicatorItem = None, rest_offset: ReplicatorItem = None)
- def randomize_articulation_view(view_name: str, operation: str = 'direct', num_buckets: int = None, stiffness: ReplicatorItem = None, damping: ReplicatorItem = None, joint_friction: ReplicatorItem = None, position: ReplicatorItem = None, orientation: ReplicatorItem = None, linear_velocity: ReplicatorItem = None, angular_velocity: ReplicatorItem = None, velocity: ReplicatorItem = None, joint_positions: ReplicatorItem = None, joint_velocities: ReplicatorItem = None, lower_dof_limits: ReplicatorItem = None, upper_dof_limits: ReplicatorItem = None, max_efforts: ReplicatorItem = None, joint_armatures: ReplicatorItem = None, joint_max_velocities: ReplicatorItem = None, joint_efforts: ReplicatorItem = None, body_masses: ReplicatorItem = None, body_inertias: ReplicatorItem = None, material_properties: ReplicatorItem = None, tendon_stiffnesses: ReplicatorItem = None, tendon_dampings: ReplicatorItem = None, tendon_limit_stiffnesses: ReplicatorItem = None, tendon_lower_limits: ReplicatorItem = None, tendon_upper_limits: ReplicatorItem = None, tendon_rest_lengths: ReplicatorItem = None, tendon_offsets: ReplicatorItem = None)
- def randomize_simulation_context(operation: str = 'direct', gravity: ReplicatorItem = None)

## Variables

- TENDON_ATTRIBUTES: List

# Public API for module isaacsim.replicator.experimental.domain_randomization.scripts.trigger:

## Functions

- def initialize_context(num_envs: Any, action_graph_entry_node: Any)
- def on_rl_frame(num_envs: int) -> Any

# Public API for module isaacsim.replicator.experimental.domain_randomization.scripts.utils:

## Classes

- class NumpyEncoder(json.JSONEncoder)
  - def default(self, obj: Any) -> Any

## Functions

- def set_distribution_params(distribution: ReplicatorItem, parameters: dict)
- def get_distribution_params(distribution: ReplicatorItem, parameters: list[str]) -> list
- def get_image_space_points(points: Any, view_proj_matrix: Any) -> Any
- def calculate_truncation_ratio_simple(corners: Any, img_width: Any, img_height: Any) -> Any
