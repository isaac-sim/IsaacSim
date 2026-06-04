# Public API for module isaacsim.ros2.sim_control:

## Classes

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
  - def set_default_state(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None, linear_velocities: list | np.ndarray | wp.array | None = None, angular_velocities: list | np.ndarray | wp.array | None = None)
  - def get_default_state(self) -> tuple[wp.array | None, wp.array | None, wp.array | None, wp.array | None]
  - def reset_to_default_state(self)
  - def initialize_cpp_data_view(self)

- class XformPrim(Prim)
  - def __init__(self, paths: str | list[str])
  - [property] def is_non_root_articulation_link(self) -> bool
  - def set_visibilities(self, visibilities: bool | list | np.ndarray | wp.array)
  - def get_visibilities(self) -> wp.array
  - def get_default_state(self) -> tuple[wp.array | None, wp.array | None]
  - def set_default_state(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def apply_visual_materials(self, materials: type['VisualMaterial'] | list[type['VisualMaterial']])
  - def get_applied_visual_materials(self) -> list[type['VisualMaterial'] | None]
  - def get_world_poses(self) -> tuple[wp.array, wp.array]
  - def set_world_poses(self, positions: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def get_local_poses(self) -> tuple[wp.array, wp.array]
  - def set_local_poses(self, translations: list | np.ndarray | wp.array | None = None, orientations: list | np.ndarray | wp.array | None = None)
  - def set_local_scales(self, scales: list | np.ndarray | wp.array | None = None)
  - def get_local_scales(self) -> wp.array
  - def reset_xform_op_properties(self)
  - def reset_to_default_state(self)

- class ROS2ServiceManager
  - def __init__(self)
  - def initialize(self)
  - def shutdown(self)
  - def register_service(self, service_name, service_type, callback)
  - def unregister_service(self, service_name, remove_from_dict = True)
  - def register_action_server(self, action_name, action_type, execute_callback, goal_callback = None, cancel_callback = None)
  - def unregister_action_server(self, action_name, remove_from_dict = True)

- class SimulationControl
  - def __init__(self)
  - def shutdown(self)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id)
  - def on_shutdown(self)

## Functions

- def get_filtered_entities(usdrt_stage, filter_pattern = None)
- async def get_entity_state(entity_path)
- def create_empty_entity_state()
- async def find_filtered_files_async(root_path: str, filter_patterns: List[str] = [], match_all: bool = False, filepath_excludes: List[str] = [], max_depth: int = None) -> set
- async def get_assets_root_path_async() -> str
- def is_local_path(path: str) -> bool
- def is_valid_usd_file(item: str, excludes: list) -> bool
- async def resolve_asset_path_async(original_path: str) -> str | None
- def Singleton(class_)

## Variables

- SERVICE_PREFIX: str
- SERVICE_TYPES: List
- ACTION_TYPES: List