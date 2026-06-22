# Public API for module isaacsim.robot_motion.experimental.motion_generation:

## Classes

- class BaseController(ABC)
  - def forward(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> Optional[RobotState]
  - def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> bool

- class ControllerContainer(BaseController)
  - def __init__(self, controller_options: dict[Enum, BaseController], initial_controller_selection: Enum)
  - def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> bool
  - def forward(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> Optional[RobotState]
  - def set_next_controller(self, next_controller_selection: Enum)
  - def get_active_controller_enum(self) -> Enum
  - def get_controller(self, controller_selection: Enum) -> BaseController

- class ParallelController(BaseController)
  - def __init__(self, controllers: list[BaseController])
  - def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> bool
  - def forward(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> Optional[RobotState]

- class SequentialController(BaseController)
  - def __init__(self, controllers: list[BaseController])
  - def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> bool
  - def forward(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> Optional[RobotState]

- class ObstacleConfiguration
  - def __init__(self, representation: ObstacleRepresentation | str, safety_tolerance: float)

- class ObstacleRepresentation(StrEnum)
  - SPHERE: Unknown
  - CONE: Unknown
  - CUBE: Unknown
  - PLANE: Unknown
  - CAPSULE: Unknown
  - CYLINDER: Unknown
  - MESH: Unknown
  - TRIANGULATED_MESH: Unknown
  - OBB: Unknown

- class ObstacleStrategy
  - def __init__(self)
  - def set_default_configuration(self, prim_type: type[Shape], configuration: ObstacleConfiguration, allow_negative_tolerance: bool = False)
  - def set_default_safety_tolerance(self, safety_tolerance: float, allow_negative_tolerance: bool = False)
  - def set_configuration_overrides(self, configurations: dict[str, ObstacleConfiguration], allow_negative_tolerance: bool = False)
  - def get_obstacle_configuration(self, prim_path: str) -> ObstacleConfiguration

- class Path
  - def __init__(self, waypoints: list | np.ndarray | wp.array)
  - def get_waypoints(self) -> wp.array
  - def get_waypoints_count(self) -> int
  - def get_waypoint_by_index(self, index: int) -> np.ndarray
  - def to_minimal_time_joint_trajectory(self, max_velocities: list | np.ndarray | wp.array, max_accelerations: list | np.ndarray | wp.array, robot_joint_space: list[str], active_joints: list[str], waypoint_relative_difference_tolerance: float = 1e-06, waypoint_absolute_difference_tolerance: float = 1e-10) -> Trajectory

- class SceneQuery
  - def __init__(self)
  - def get_prims_in_aabb(self, search_box_origin: wp.array | list[float] | np.ndarray, search_box_minimum: wp.array | list[float] | np.ndarray, search_box_maximum: wp.array | list[float] | np.ndarray, tracked_api: TrackableApi, include_prim_paths: list[str] | str | None = None, exclude_prim_paths: list[str] | str | None = None) -> list[str]
  - def get_robots_in_stage(self) -> list[str]

- class TrackableApi(StrEnum)
  - PHYSICS_COLLISION: str
  - PHYSICS_RIGID_BODY: str
  - MOTION_GENERATION_COLLISION: str

- class Trajectory(ABC)
  - [property] def duration(self) -> float
  - def get_target_state(self, time: float) -> Optional[RobotState]

- class TrajectoryFollower(BaseController)
  - def __init__(self)
  - def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> bool
  - def forward(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs: object) -> Optional[RobotState]
  - def set_trajectory(self, trajectory: Trajectory)
  - def has_trajectory(self) -> bool

- class JointState
  - def __init__(self, robot_joint_space: list[str], data_array: wp.array, valid_array: wp.array)
  - class def from_name(cls, robot_joint_space: list[str], positions: tuple[list[str], wp.array] | None = None, velocities: tuple[list[str], wp.array] | None = None, efforts: tuple[list[str], wp.array] | None = None) -> JointState
  - class def from_index(cls, robot_joint_space: list[str], positions: tuple[wp.array, wp.array] | None = None, velocities: tuple[wp.array, wp.array] | None = None, efforts: tuple[wp.array, wp.array] | None = None) -> JointState
  - [property] def robot_joint_space(self) -> list[str]
  - [property] def position_names(self) -> list[str]
  - [property] def position_indices(self) -> wp.array
  - [property] def positions(self) -> wp.array | None
  - [property] def velocity_names(self) -> list[str]
  - [property] def velocity_indices(self) -> wp.array
  - [property] def velocities(self) -> wp.array | None
  - [property] def effort_names(self) -> list[str]
  - [property] def effort_indices(self) -> wp.array
  - [property] def efforts(self) -> wp.array | None
  - [property] def data_array(self) -> wp.array
  - [property] def valid_array(self) -> wp.array

- class RobotState
  - def __init__(self, joints: JointState | None = None, root: RootState | None = None, links: SpatialState | None = None, sites: SpatialState | None = None)
  - [property] def joints(self) -> JointState | None
  - [property] def root(self) -> RootState | None
  - [property] def links(self) -> SpatialState | None
  - [property] def sites(self) -> SpatialState | None

- class RootState
  - def __init__(self, position: wp.array | None = None, orientation: wp.array | None = None, linear_velocity: wp.array | None = None, angular_velocity: wp.array | None = None)
  - [property] def position(self) -> wp.array | None
  - [property] def orientation(self) -> wp.array | None
  - [property] def linear_velocity(self) -> wp.array | None
  - [property] def angular_velocity(self) -> wp.array | None

- class SpatialState
  - def __init__(self, spatial_space: list[str], position_data: wp.array, linear_velocity_data: wp.array, orientation_data: wp.array, angular_velocity_data: wp.array, valid_array: wp.array)
  - class def from_name(cls, spatial_space: list[str], positions: tuple[list[str], wp.array] | None = None, orientations: tuple[list[str], wp.array] | None = None, linear_velocities: tuple[list[str], wp.array] | None = None, angular_velocities: tuple[list[str], wp.array] | None = None) -> SpatialState
  - class def from_index(cls, spatial_space: list[str], positions: tuple[wp.array, wp.array] | None = None, orientations: tuple[wp.array, wp.array] | None = None, linear_velocities: tuple[wp.array, wp.array] | None = None, angular_velocities: tuple[wp.array, wp.array] | None = None) -> SpatialState
  - [property] def spatial_space(self) -> list[str]
  - [property] def position_names(self) -> list[str]
  - [property] def position_indices(self) -> wp.array
  - [property] def positions(self) -> wp.array | None
  - [property] def orientation_names(self) -> list[str]
  - [property] def orientation_indices(self) -> wp.array
  - [property] def orientations(self) -> wp.array | None
  - [property] def linear_velocity_names(self) -> list[str]
  - [property] def linear_velocity_indices(self) -> wp.array
  - [property] def linear_velocities(self) -> wp.array | None
  - [property] def angular_velocity_names(self) -> list[str]
  - [property] def angular_velocity_indices(self) -> wp.array
  - [property] def angular_velocities(self) -> wp.array | None
  - [property] def position_data(self) -> wp.array
  - [property] def orientation_data(self) -> wp.array
  - [property] def linear_velocity_data(self) -> wp.array
  - [property] def angular_velocity_data(self) -> wp.array
  - [property] def valid_array(self) -> wp.array

- class WorldBinding(Generic[TWorldInterface])
  - def __init__(self, world_interface: TWorldInterface, obstacle_strategy: ObstacleStrategy, tracked_prims: list[str], tracked_collision_api: TrackableApi)
  - def initialize(self)
  - def synchronize(self)
  - def synchronize_transforms(self)
  - def synchronize_properties(self)
  - def get_world_interface(self) -> WorldInterface

- class WorldInterface
  - def add_spheres(self, prim_paths: list[str], radii: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_cubes(self, prim_paths: list[str], sizes: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_cones(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], radii: wp.array, lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_planes(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], lengths: wp.array, widths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_capsules(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], radii: wp.array, lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_cylinders(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], radii: wp.array, lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_meshes(self, prim_paths: list[str], points: list[wp.array], face_vertex_indices: list[wp.array], face_vertex_counts: list[wp.array], normals: list[wp.array], scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_triangulated_meshes(self, prim_paths: list[str], points: list[wp.array], face_vertex_indices: list[wp.array], scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def add_oriented_bounding_boxes(self, prim_paths: list[str], centers: wp.array, rotations: wp.array, half_side_lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array) -> NoReturn
  - def update_obstacle_transforms(self, prim_paths: list[str], poses: tuple[wp.array, wp.array]) -> NoReturn
  - def update_obstacle_twists(self, prim_paths: list[str], poses: tuple[wp.array, wp.array]) -> NoReturn
  - def update_obstacle_enables(self, prim_paths: list[str], enabled_array: wp.array) -> NoReturn
  - def update_obstacle_scales(self, prim_paths: list[str], scales: wp.array) -> NoReturn
  - def update_sphere_properties(self, prim_paths: list[str], radii: wp.array | None) -> NoReturn
  - def update_cube_properties(self, prim_paths: list[str], sizes: wp.array | None) -> NoReturn
  - def update_cone_properties(self, prim_paths: list[str], axes: list[Literal['X', 'Y', 'Z']] | None, radii: wp.array | None, lengths: wp.array | None) -> NoReturn
  - def update_plane_properties(self, prim_paths: list[str], axes: list[Literal['X', 'Y', 'Z']] | None, lengths: wp.array | None, widths: wp.array | None) -> NoReturn
  - def update_capsule_properties(self, prim_paths: list[str], axes: list[Literal['X', 'Y', 'Z']] | None, radii: wp.array | None, lengths: wp.array | None) -> NoReturn
  - def update_cylinder_properties(self, prim_paths: list[str], axes: list[Literal['X', 'Y', 'Z']] | None, radii: wp.array | None, lengths: wp.array | None) -> NoReturn
  - def update_mesh_properties(self, prim_paths: list[str], points: list[wp.array] | None, face_vertex_indices: list[wp.array] | None, face_vertex_counts: list[wp.array] | None, normals: list[wp.array] | None) -> NoReturn
  - def update_triangulated_mesh_properties(self, prim_paths: list[str], points: list[wp.array] | None, face_vertex_indices: list[wp.array] | None) -> NoReturn
  - def update_oriented_bounding_box_properties(self, prim_paths: list[str], centers: wp.array | None, rotations: wp.array | None, half_side_lengths: wp.array | None) -> NoReturn

## Functions

- def combine_robot_states(robot_state_1: RobotState | None, robot_state_2: RobotState | None) -> RobotState | None
