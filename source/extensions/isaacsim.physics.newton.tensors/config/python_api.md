# Public API for module isaacsim.physics.newton.tensors:

## Classes

- class NewtonSimulationView
  - def __init__(self, backend: NewtonSimView, frontend: NumpyFrontend | TorchFrontend | WarpFrontend)
  - def create_articulation_view(self, pattern: str | list[str]) -> NewtonArticulationView
  - def create_rigid_body_view(self, pattern: str | list[str]) -> NewtonRigidBodyView
  - def create_rigid_contact_view(self, pattern: str | list[str], filter_patterns: list[list[str]] | None = None, max_contact_data_count: int = 0) -> NewtonRigidContactView
  - def invalidate(self)
  - def is_valid(self) -> bool
  - def set_subspace_roots(self, pattern: str | list[str]) -> bool

- class NewtonSimView
  - def __init__(self, newton_stage: NewtonStage)
  - [property] def model(self) -> Any
  - def get_gravity(self, gravity: list[float]) -> bool
  - def set_gravity(self, gravity: list[float])
  - def update_articulations_kinematic(self) -> bool
  - def initialize_kinematic_bodies(self)
  - def invalidate(self)
  - def is_valid(self) -> bool
  - def create_rigid_contact_view(self, pattern: list[str], filter_patterns: list[list[str]] | None = None, max_contact_data_count: int = 0) -> RigidContactSet | None
  - def create_rigid_body_view(self, pattern: str | list[str]) -> RigidBodySet
  - def create_articulation_view(self, pattern: str | list[str]) -> ArticulationSet
  - def set_subspace_roots(self, pattern: str | list[str]) -> bool

- class ArticulationSet
  - def __init__(self, newton_stage: NewtonStage, articulation_indices: wp.array, root_body_indices: wp.array, dof_position_indices: wp.array, dof_velocity_indices: wp.array, dof_axis_indices: wp.array, joint_indices: wp.array, shape_indices: wp.array, link_indices: wp.array, meta_types: list[ArticulationMetaType], count: int, max_dofs: int)
  - [property] def model(self) -> Any
  - [property] def shared_metatype(self) -> ArticulationMetaType
  - [property] def max_links(self) -> int
  - [property] def link_paths(self) -> list[list[str]]
  - [property] def max_shapes(self) -> int

- class RigidBodySet
  - def __init__(self, newton_stage: NewtonStage, body_indices: wp.array, body_paths: list[str], body_names: list[str])
  - [property] def model(self) -> Any

- class RigidContactSet
  - def __init__(self, newton_stage: NewtonStage, sensor_indices: wp.array, sensor_paths: list[str], sensor_names: list[str], filter_indices: wp.array, filter_paths: list[list[str]], filter_names: list[list[str]], max_filters: int, body_sensor_map: wp.array, world_body_idx: int, max_contact_data_count: int = 0)
  - [property] def model(self) -> Any

- class NewtonArticulationView
  - def __init__(self, backend: Any, frontend: Any)
  - [property] def count(self) -> int
  - [property] def max_dofs(self) -> int
  - [property] def max_links(self) -> int
  - [property] def max_shapes(self) -> int
  - [property] def max_fixed_tendons(self) -> int
  - [property] def dof_paths(self) -> Any
  - [property] def dof_names(self) -> list[list[str]]
  - [property] def link_paths(self) -> list[list[str]]
  - [property] def link_names(self) -> list[list[str]]
  - [property] def joint_paths(self) -> list[list[str]]
  - [property] def joint_names(self) -> list[list[str]]
  - [property] def prim_paths(self) -> list[str]
  - [property] def shared_metatype(self) -> Any | None
  - [property] def is_homogeneous(self) -> bool
  - [property] def jacobian_shape(self) -> tuple[int, int]
  - [property] def generalized_mass_matrix_shape(self) -> tuple[int, int]
  - def get_metatype(self, index: int) -> Any
  - def update(self, dt: float)
  - def get_root_transforms(self, copy: bool = copy_data) -> Any
  - def get_root_velocities(self, copy: bool = copy_data) -> Any
  - def get_masses(self, copy: bool = copy_data) -> Any
  - def get_inv_masses(self, copy: bool = copy_data) -> Any
  - def get_inertias(self, copy: bool = copy_data) -> Any
  - def get_inv_inertias(self, copy: bool = copy_data) -> Any
  - def get_coms(self, copy: bool = copy_data) -> Any
  - def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def get_dof_positions(self, copy: bool = copy_data) -> Any
  - def get_dof_velocities(self, copy: bool = copy_data) -> Any
  - def get_dof_limits(self, copy: bool = copy_data) -> Any
  - def get_dof_stiffnesses(self, copy: bool = copy_data) -> Any
  - def get_dof_dampings(self, copy: bool = copy_data) -> Any
  - def get_dof_armatures(self, copy: bool = copy_data) -> Any
  - def get_dof_position_targets(self, copy: bool = copy_data) -> Any
  - def get_dof_velocity_targets(self, copy: bool = copy_data) -> Any
  - def set_root_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_root_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_masses(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_inertias(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_positions(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_stiffnesses(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_dampings(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_armatures(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_position_targets(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_velocity_targets(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_actuation_forces(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def get_dof_actuation_forces(self, copy: bool = copy_data) -> Any
  - def get_dof_max_forces(self, copy: bool = copy_data) -> Any
  - def set_dof_max_forces(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_limits(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_max_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_drive_model_properties(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_dof_friction_properties(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def update_joints(self, indices: Any, indices_mask: Any | None = None)
  - def apply_forces(self, force_data: Any, indices: Any | None = None, is_global: bool = True, indices_mask: Any | None = None)
  - def apply_forces_and_torques_at_position(self, force_data: Any | None, torque_data: Any | None, position_data: Any | None, indices: Any, is_global: bool = True, indices_mask: Any | None = None)
  - def get_generalized_mass_matrices(self, copy: bool = copy_data) -> Any
  - def get_jacobians(self, copy: bool = copy_data) -> Any
  - def get_disable_gravities(self, copy: bool = copy_data) -> Any
  - def get_dof_max_velocities(self, copy: bool = copy_data) -> Any
  - def get_dof_projected_joint_forces(self, copy: bool = copy_data) -> Any
  - def get_gravity_compensation_forces(self, copy: bool = copy_data) -> Any
  - def get_coriolis_and_centrifugal_compensation_forces(self, copy: bool = copy_data) -> Any
  - def get_dof_friction_properties(self, copy: bool = copy_data) -> Any
  - def get_drive_types(self, copy: bool = copy_data) -> Any
  - def get_dof_drive_model_properties(self, copy: bool = copy_data) -> Any
  - def get_link_incoming_joint_force(self, copy: bool = copy_data) -> Any
  - def check(self) -> bool

- class NewtonRigidBodyView
  - def __init__(self, backend: Any, frontend: Any)
  - [property] def count(self) -> int
  - [property] def max_shapes(self) -> int
  - [property] def body_paths(self) -> list[str]
  - [property] def body_names(self) -> list[str]
  - def update(self, dt: float)
  - def get_transforms(self, copy: bool = copy_data) -> Any
  - def get_velocities(self, copy: bool = copy_data) -> Any
  - def get_accelerations(self, copy: bool = copy_data) -> Any
  - def get_masses(self, copy: bool = copy_data) -> Any
  - def get_inv_masses(self, copy: bool = copy_data) -> Any
  - def get_coms(self, copy: bool = copy_data) -> Any
  - def get_inertias(self, copy: bool = copy_data) -> Any
  - def get_inv_inertias(self, copy: bool = copy_data) -> Any
  - def set_transforms(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_velocities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_masses(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_coms(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def set_inertias(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def get_disable_simulations(self, copy: bool = copy_data) -> Any
  - def set_disable_simulations(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def get_disable_gravities(self, copy: bool = copy_data) -> Any
  - def set_disable_gravities(self, data: Any, indices: Any, indices_mask: Any | None = None)
  - def apply_forces(self, force_data: Any, indices: Any | None = None, is_global: bool = True, indices_mask: Any | None = None)
  - def apply_forces_and_torques_at_position(self, force_data: Any | None, torque_data: Any | None, position_data: Any | None, indices: Any, is_global: bool = True, indices_mask: Any | None = None)
  - def check(self) -> bool

- class NewtonRigidContactView
  - def __init__(self, backend: RigidContactSet, frontend: NumpyFrontend | TorchFrontend | WarpFrontend)
  - [property] def count(self) -> int
  - [property] def sensor_count(self) -> int
  - [property] def filter_count(self) -> int
  - [property] def sensor_names(self) -> list[str]
  - [property] def sensor_paths(self) -> list[str]
  - [property] def filter_paths(self) -> list[list[str]]
  - [property] def filter_names(self) -> list[list[str]]
  - [property] def max_contact_data_count(self) -> int
  - def update(self, dt: float)
  - def get_net_contact_forces(self, dt: float, copy: bool = copy_data) -> Any
  - def get_contact_force_matrix(self, dt: float, copy: bool = copy_data) -> wp.array
  - def get_contact_data(self, dt: float, max_contact_data_count: int = 0, copy: bool = copy_data) -> Any
  - def get_raw_contact_data(self, dt: float, copy: bool = copy_data) -> Any
  - def get_actor_paths_from_ids(self, actor_ids: wp.array) -> list[str]
  - def check(self) -> bool

## Functions

- def create_simulation_view(frontend_name: str, newton_stage: NewtonStage, stage_id: int = -1) -> NewtonSimulationView
- def find_matching_paths(stage: Usd.Stage, pattern: str | list[str]) -> list[str]

# Public API for module isaacsim.physics.newton.tensors.kernels:

## Functions

- def get_body_pose(body_q: wp.array(dtype=wp.transform), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_velocity(body_qd: wp.array(dtype=wp.spatial_vector), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_mass(body_mass: wp.array(dtype=wp.float32), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_inv_mass(body_inv_mass: wp.array(dtype=wp.float32), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_com(body_com: wp.array(dtype=wp.vec3), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_com_position_only(body_com: wp.array(dtype=wp.vec3), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def cache_body_com(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), com_cache: wp.array2d(dtype=float))
- def get_body_inertia(body_inertia: wp.array(dtype=wp.mat33), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_body_inv_inertia(body_inv_inertia: wp.array(dtype=wp.mat33), index: wp.array(dtype=int), tensor: wp.array2d(dtype=float))
- def get_link_inv_mass(body_mass: wp.array(dtype=wp.float32), index: wp.array2d(dtype=int), tensor: wp.array2d(dtype=float))
- def get_link_mass(body_mass: wp.array(dtype=wp.float32), index: wp.array2d(dtype=int), tensor: wp.array2d(dtype=float))
- def set_body_mass(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_mass: wp.array(dtype=wp.float32))
- def update_body_inv_mass(tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_mass: wp.array(dtype=wp.float32), body_inv_mass: wp.array(dtype=wp.float32))
- def set_body_com(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_com: wp.array(dtype=wp.vec3))
- def set_body_inertia(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_inertia: wp.array(dtype=wp.mat33))
- def update_body_inv_inertia(tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_inertia: wp.array(dtype=wp.mat33), body_inv_inertia: wp.array(dtype=wp.mat33))
- def get_link_inertia(body_inertia: wp.array(dtype=wp.mat33), index: wp.array2d(dtype=int), tensor: wp.array3d(dtype=float))
- def get_link_inv_inertia(body_inv_inertia: wp.array(dtype=wp.mat33), index: wp.array2d(dtype=int), tensor: wp.array3d(dtype=float))
- def get_link_com(body_com: wp.array(dtype=wp.vec3), index: wp.array2d(dtype=int), tensor: wp.array3d(dtype=float))
- def get_link_com_position_only(body_com: wp.array(dtype=wp.vec3), index: wp.array2d(dtype=int), tensor: wp.array3d(dtype=float))
- def cache_link_com(tensor: wp.array3d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), com_cache: wp.array3d(dtype=float))
- def set_link_com(tensor: wp.array3d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_com: wp.array(dtype=wp.vec3))
- def get_dof_attributes(joint_attr: wp.array(dtype=wp.float32), index: wp.array(dtype=int), max_dofs: int, tensor: Any)
- def get_dof_limits(lower_limits: wp.array(dtype=wp.float32), upper_limits: wp.array(dtype=wp.float32), index: wp.array(dtype=int), max_dofs: int, tensor: Any)
- def set_dof_limits(tensor: wp.array3d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), index: wp.array(dtype=int), max_dofs: int, lower_limits: wp.array(dtype=wp.float32), upper_limits: wp.array(dtype=wp.float32))
- def set_body_pose(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_q: wp.array(dtype=wp.transform))
- def update_free_joint_coords_from_body_q(body_q: wp.array(dtype=wp.transform), tensor_idx: Any, body_idx: wp.array(dtype=int), joint_child: wp.array(dtype=int), joint_type: wp.array(dtype=int), joint_q_start: wp.array(dtype=int), joint_q: wp.array(dtype=wp.float32))
- def set_body_velocity(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_qd: wp.array(dtype=wp.spatial_vector))
- def set_link_mass(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_mass: wp.array(dtype=wp.float32))
- def update_inv_mass(tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_mass: wp.array(dtype=wp.float32), body_inv_mass: wp.array(dtype=wp.float32))
- def set_link_inertia(tensor: wp.array3d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_inertia: wp.array(dtype=wp.mat33))
- def update_inv_inertia(tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_inertia: wp.array(dtype=wp.mat33), body_inv_inertia: wp.array(dtype=wp.mat33))
- def update_joint_coords_from_root(body_q: wp.array(dtype=wp.transform), arti_indices: wp.array(dtype=wp.int32), arti_mask: wp.array(dtype=int), root_body_indices: wp.array(dtype=int), articulation_start: wp.array(dtype=int), joint_q_start: wp.array(dtype=int), joint_type: wp.array(dtype=int), joint_q: wp.array(dtype=wp.float32), joint_X_p: wp.array(dtype=wp.transform))
- def set_dof_attributes(tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), index: wp.array(dtype=int), max_dofs: int, joint_attr: wp.array(dtype=wp.float32))
- def assign_articulation_root_states(body_q: wp.array(dtype=wp.transform), body_qd: wp.array(dtype=wp.spatial_vector), tensor_idx: wp.array(dtype=wp.int64), tenor_idx_mask: wp.array(dtype=int), articulation_indices: wp.array(dtype=int), body_index: wp.array(dtype=int), update_fixed_base_articulations: bool, relative_transforms: bool, joint_type: wp.array(dtype=int), articulation_start: wp.array(dtype=int), joint_q_start: wp.array(dtype=int), joint_qd_start: wp.array(dtype=int), joint_q: wp.array(dtype=float), joint_qd: wp.array(dtype=float), joint_X_p: wp.array(dtype=wp.transform))
- def apply_body_forces_at_position(force_tensor: wp.array2d(dtype=float), torque_tensor: wp.array2d(dtype=float), position_tensor: wp.array2d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), body_idx: wp.array(dtype=int), body_q: wp.array(dtype=wp.transform), body_com: wp.array(dtype=wp.vec3), is_global: bool, has_force: bool, has_torque: bool, has_position: bool, body_f: wp.array(dtype=wp.spatial_vector))
- def apply_link_forces_at_position(force_tensor: wp.array3d(dtype=float), torque_tensor: wp.array3d(dtype=float), position_tensor: wp.array3d(dtype=float), tensor_idx: Any, tenor_idx_mask: wp.array(dtype=int), link_indices: wp.array2d(dtype=int), body_q: wp.array(dtype=wp.transform), body_com: wp.array(dtype=wp.vec3), is_global: bool, has_force: bool, has_torque: bool, has_position: bool, body_f: wp.array(dtype=wp.spatial_vector))
- def sync_ctrl_direct_targets(dof_to_act: wp.array(dtype=wp.int32), joint_target_pos: wp.array(dtype=wp.float32), dofs_per_world: wp.int32, ctrls_per_world: wp.int32, mujoco_ctrl: wp.array(dtype=wp.float32))
- def sync_ctrl_direct_gains(dof_to_act: wp.array(dtype=wp.int32), joint_target_ke: wp.array(dtype=wp.float32), joint_target_kd: wp.array(dtype=wp.float32), actuator_gainprm: wp.array(dtype=vec10), actuator_biasprm: wp.array(dtype=vec10))
- def build_ctrl_direct_dof_mapping(model: newton.Model) -> wp.array | None

## Variables

- vec10: Unknown
