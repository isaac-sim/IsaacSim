# Public API for module isaacsim.replicator.experimental.mobility_gen:

## Classes

- class MobilityGenCamera(Module)
  - def __init__(self, prim_path: str, resolution: tuple[int, int])
  - def enable_rendering(self)
  - def finalize_rendering(self)
  - def disable_rendering(self)
  - def enable_rgb_rendering(self)
  - def enable_segmentation_rendering(self)
  - def enable_instance_id_segmentation_rendering(self)
  - def enable_depth_rendering(self)
  - def enable_normals_rendering(self)
  - def update_state(self)

- class Buffer
  - def __init__(self, value = None, tags: list[str] | None = None)
  - def get_value(self)
  - def set_value(self, value)
  - def includes_tags(self, tags: list[str])
  - def excludes_tags(self, tags: list[str])

- class Module
  - def children(self) -> dict[str, Module]
  - def buffers(self) -> dict[str, Buffer]
  - def named_modules(self, prefix: str = '') -> dict[str, Module]
  - def named_buffers(self, prefix: str = '', include_tags: list[str] | None = None, exclude_tags: list[str] | None = None) -> dict[str, Buffer]
  - def state_dict(self, prefix: str = '', include_tags: list[str] | None = None, exclude_tags: list[str] | None = None) -> dict[str, object]
  - def state_dict_common(self, prefix: str = '')
  - def state_dict_rgb(self, prefix: str = '')
  - def state_dict_segmentation(self, prefix: str = '')
  - def state_dict_depth(self, prefix: str = '')
  - def state_dict_normals(self, prefix: str = '')
  - def enable_rgb_rendering(self)
  - def enable_segmentation_rendering(self)
  - def enable_depth_rendering(self)
  - def enable_instance_id_segmentation_rendering(self)
  - def enable_normals_rendering(self)
  - def finalize_rendering(self)
  - def disable_rendering(self)
  - def write_replay_data(self)
  - def update_state(self)
  - def load_state_dict(self, state_dict)

- class Config
  - scenario_type: str
  - robot_type: str
  - scene_usd: str
  - def to_json(self)
  - static def from_json(data: str)

- class Gamepad(Module)
  - def __init__(self)
  - def update_state(self)

- class GamepadDriver(object)
  - def __init__(self)
  - static def connect()
  - static def disconnect()
  - static def instance()
  - def get_axis_values(self) -> np.ndarray
  - def get_button_values(self) -> np.ndarray

- class Keyboard(Module)
  - def __init__(self)
  - def update_state(self)

- class KeyboardDriver(object)
  - def __init__(self)
  - static def connect()
  - static def disconnect()
  - static def instance()
  - def get_button_values(self) -> np.ndarray

- class OccupancyMap
  - ROS_IMAGE_FILENAME: str
  - ROS_YAML_FILENAME: str
  - ROS_YAML_TEMPLATE: str
  - def __init__(self, data: np.ndarray, resolution: int, origin: tuple[int, int, int])
  - def freespace_mask(self) -> np.ndarray
  - def unknown_mask(self) -> np.ndarray
  - def occupied_mask(self) -> np.ndarray
  - def ros_image(self, negate: bool = False) -> PIL.Image.Image
  - def ros_yaml(self, negate: bool = False) -> str
  - def save_ros(self, path: str)
  - static def from_ros_yaml(ros_yaml_path: str) -> OccupancyMap
  - static def from_ros_image(ros_image: PIL.Image.Image, resolution: int, origin: tuple[float, float, float], negate: bool = False, occupied_thresh: float = ROS_OCCUPIED_THRESH_DEFAULT, free_thresh: float = ROS_FREESPACE_THRESH_DEFAULT) -> OccupancyMap
  - static def from_masks(freespace_mask: np.ndarray, occupied_mask: np.ndarray, resolution: int, origin: tuple[float, float, float]) -> OccupancyMap
  - def width_pixels(self) -> int
  - def height_pixels(self) -> int
  - def width_meters(self)
  - def height_meters(self)
  - def bottom_left_pixel_world_coords(self) -> tuple[float, float]
  - def top_left_pixel_world_coords(self) -> tuple[float, float]
  - def bottom_right_pixel_world_coords(self) -> tuple[float, float]
  - def top_right_pixel_world_coords(self) -> tuple[float, float]
  - def buffered(self, buffer_distance_pixels: int) -> OccupancyMap
  - def buffered_meters(self, buffer_distance_meters: float) -> OccupancyMap
  - def pixel_to_world(self, point: Point2d)
  - def pixel_to_world_numpy(self, points: np.ndarray)
  - def world_to_pixel_numpy(self, points: np.ndarray)
  - def check_world_point_in_bounds(self, point: Point2d) -> bool
  - def check_world_point_in_freespace(self, point: Point2d) -> bool

- class GridPoseSampler(PoseSampler)
  - grid_size_meters: float
  - def __init__(self, grid_size_meters: float)
  - def sample_px(self, occupancy_map: OccupancyMap) -> Pose2d

- class UniformPoseSampler(PoseSampler)
  - def sample_px(self, occupancy_map: OccupancyMap) -> Pose2d

- class MobilityGenReader
  - def __init__(self, recording_path: str)
  - def read_config(self) -> Config
  - def read_occupancy_map(self)
  - def read_rgb(self, name: str, index: int)
  - def read_state_dict_rgb(self, index: int)
  - def read_segmentation(self, name: str, index: int)
  - def read_normals(self, name: str, index: int)
  - def read_state_dict_segmentation(self, index: int)
  - def read_depth(self, name: str, index: int, eps = 1e-06)
  - def read_state_dict_depth(self, index: int)
  - def read_state_dict_normals(self, index: int)
  - def read_state_dict_common(self, index: int)
  - def read_state_dict(self, index: int)

- class MobilityGenRobot(Module)
  - physics_dt: float
  - z_offset: float
  - chase_camera_base_path: str
  - chase_camera_x_offset: float
  - chase_camera_z_offset: float
  - chase_camera_tilt_angle: float
  - occupancy_map_radius: float
  - occupancy_map_collision_radius: float
  - front_camera_type: type[Module]
  - front_camera_base_path: str
  - front_camera_rotation: tuple[float, float, float]
  - front_camera_translation: tuple[float, float, float]
  - keyboard_linear_velocity_gain: float
  - keyboard_angular_velocity_gain: float
  - gamepad_linear_velocity_gain: float
  - gamepad_angular_velocity_gain: float
  - random_action_linear_velocity_range: tuple[float, float]
  - random_action_angular_velocity_range: tuple[float, float]
  - random_action_linear_acceleration_std: float
  - random_action_angular_acceleration_std: float
  - random_action_grid_pose_sampler_grid_size: float
  - path_following_speed: float
  - path_following_angular_gain: float
  - path_following_stop_distance_threshold: float
  - path_following_forward_angle_threshold: Unknown
  - path_following_target_point_offset_meters: float
  - def __init__(self, prim_path: str, articulation: Articulation, front_camera: Module)
  - class def build_front_camera(cls, prim_path)
  - def build_chase_camera(self) -> str
  - class def build(cls, prim_path: str) -> MobilityGenRobot
  - def write_action(self, step_size: float)
  - def is_physics_ready(self) -> bool
  - def update_state(self)
  - def write_replay_data(self)
  - def set_pose_2d(self, pose: Pose2d)
  - def get_pose_2d(self) -> Pose2d

- class MobilityGenScenario(Module)
  - def __init__(self, robot: MobilityGenRobot, occupancy_map: OccupancyMap)
  - class def from_robot_occupancy_map(cls, robot: MobilityGenRobot, occupancy_map: OccupancyMap)
  - def reset(self)
  - def step(self, step_size: float) -> bool
  - def get_visualization_image(self) -> Image

- class Pose2d(Point2d)
  - theta: float

- class PathHelper
  - def __init__(self, points: np.ndarray)
  - def get_path_length(self) -> float
  - def get_point_by_distance(self, distance: float) -> np.ndarray
  - def find_nearest(self, point: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int], float]

- class MobilityGenWriter
  - def __init__(self, path: str)
  - def write_state_dict_common(self, state_dict: dict, step: int)
  - def write_state_dict_rgb(self, state_rgb: dict, step: int)
  - def write_state_dict_segmentation(self, state_segmentation: dict, step: int)
  - def write_state_dict_depth(self, state_np: dict, step: int)
  - def write_state_dict_normals(self, state_np: dict, step: int)
  - def copy_stage(self, input_path: str)
  - def write_config(self, config: Config)
  - def write_occupancy_map(self, occupancy_map: OccupancyMap)
  - def copy_init(self, other_path: str)

## Functions

- def load_scenario(path: str) -> MobilityGenScenario
- def compress_path(path: np.ndarray, eps = 0.001)
- def generate_paths(start: tuple[int, int], freespace: np.ndarray) -> GeneratePathsOutput

## Variables

- ROBOTS: Unknown
- SCENARIOS: Unknown
