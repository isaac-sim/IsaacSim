# Public API for module isaacsim.replicator.experimental.mobility_gen:

## Classes

- class Buffer
  - def __init__(self, value: object | None = None, tags: list[str] | None = None)
  - def get_value(self) -> object
  - def set_value(self, value: object)
  - def includes_tags(self, tags: list[str]) -> bool
  - def excludes_tags(self, tags: list[str]) -> bool

- class CameraConfig
  - name: str
  - sensor_prim_path: str
  - width_px: int
  - height_px: int
  - frame_id: str
  - class def from_dict(cls, data: dict) -> CameraConfig

- class Config
  - scenario_type: str
  - robot_type: str
  - scene_usd: str
  - def to_json(self) -> str
  - static def from_json(data: str) -> Config

- class Gamepad(Module)
  - def __init__(self)
  - def update_state(self)

- class GamepadDriver(object)
  - def __init__(self)
  - static def connect() -> GamepadDriver
  - static def disconnect()
  - static def instance() -> GamepadDriver
  - def get_axis_values(self) -> np.ndarray
  - def get_button_values(self) -> np.ndarray

- class GridPoseSampler(PoseSampler)
  - grid_size_meters: float
  - def __init__(self, grid_size_meters: float)
  - def sample_px(self, occupancy_map: OccupancyMap) -> Pose2d

- class Keyboard(Module)
  - def __init__(self)
  - def update_state(self)

- class KeyboardDriver(object)
  - def __init__(self)
  - static def connect() -> KeyboardDriver
  - static def disconnect()
  - static def instance() -> KeyboardDriver
  - def get_button_values(self) -> np.ndarray

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

- class MobilityGenMultiSensorRobot(MobilityGenRobot)
  - robot_config_path: str
  - sensor_configs: list[SensorConfig]
  - [property] def front_camera(self)
  - [front_camera.setter] def front_camera(self, _value: object)
  - def __init__(self, prim_path: str, articulation: Articulation, sensor_rig: Module | None = None)
  - class def build_sensor_rig(cls, prim_path: str) -> Module | None

- class MobilityGenReader
  - def __init__(self, recording_path: str)
  - def read_config(self) -> Config
  - def read_occupancy_map(self) -> OccupancyMap
  - def read_rgb(self, name: str, index: int) -> np.ndarray
  - def read_state_dict_rgb(self, index: int) -> dict
  - def read_segmentation(self, name: str, index: int) -> np.ndarray
  - def read_normals(self, name: str, index: int) -> np.ndarray
  - def read_state_dict_segmentation(self, index: int) -> dict
  - def read_depth(self, name: str, index: int, eps: float = 1e-06) -> np.ndarray
  - def read_state_dict_depth(self, index: int) -> dict
  - def read_state_dict_normals(self, index: int) -> dict
  - def read_state_dict_common(self, index: int) -> dict
  - def read_state_dict(self, index: int) -> dict

- class MobilityGenRobot(Module, ABC)
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
  - class def build_front_camera(cls, prim_path: str) -> Module
  - def build_chase_camera(self) -> str
  - class def build(cls, prim_path: str) -> MobilityGenRobot
  - def write_action(self, step_size: float)
  - def is_physics_ready(self) -> bool
  - def update_state(self)
  - def write_replay_data(self)
  - def set_pose_2d(self, pose: Pose2d)
  - def get_pose_2d(self) -> Pose2d

- class MobilityGenScenario(Module, ABC)
  - def __init__(self, robot: MobilityGenRobot, occupancy_map: OccupancyMap)
  - class def from_robot_occupancy_map(cls, robot: MobilityGenRobot, occupancy_map: OccupancyMap) -> MobilityGenScenario
  - def reset(self)
  - def step(self, step_size: float) -> bool
  - def get_visualization_image(self) -> Image.Image

- class MobilityGenSensorRig(Module)
  - class def from_sensor_configs(cls, configs: list[SensorConfig], robot_root_path: str) -> MobilityGenSensorRig

- class MobilityGenWriter
  - def __init__(self, path: str, async_write: bool = True, max_pending: int = 8)
  - def flush(self)
  - def close(self)
  - def write_state_dict_common(self, state_dict: dict, step: int)
  - def write_state_dict_rgb(self, state_rgb: dict, step: int)
  - def write_state_dict_segmentation(self, state_segmentation: dict, step: int)
  - def write_state_dict_depth(self, state_np: dict, step: int)
  - def write_state_dict_normals(self, state_np: dict, step: int)
  - def copy_stage(self, input_path: str)
  - def write_config(self, config: Config)
  - def write_occupancy_map(self, occupancy_map: OccupancyMap)
  - def copy_init(self, other_path: str)

- class Module
  - def children(self) -> dict[str, Module]
  - def buffers(self) -> dict[str, Buffer]
  - def named_modules(self, prefix: str = '') -> dict[str, Module]
  - def named_buffers(self, prefix: str = '', include_tags: list[str] | None = None, exclude_tags: list[str] | None = None) -> dict[str, Buffer]
  - def state_dict(self, prefix: str = '', include_tags: list[str] | None = None, exclude_tags: list[str] | None = None) -> dict[str, object]
  - def state_dict_common(self, prefix: str = '') -> dict[str, object]
  - def state_dict_rgb(self, prefix: str = '') -> dict[str, object]
  - def state_dict_segmentation(self, prefix: str = '') -> dict[str, object]
  - def state_dict_depth(self, prefix: str = '') -> dict[str, object]
  - def state_dict_normals(self, prefix: str = '') -> dict[str, object]
  - def enable_rgb_rendering(self)
  - def enable_segmentation_rendering(self)
  - def enable_depth_rendering(self)
  - def enable_instance_id_segmentation_rendering(self)
  - def enable_normals_rendering(self)
  - def finalize_rendering(self)
  - def disable_rendering(self)
  - def write_replay_data(self)
  - def update_state(self)
  - def load_state_dict(self, state_dict: dict)

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
  - def width_meters(self) -> float
  - def height_meters(self) -> float
  - def bottom_left_pixel_world_coords(self) -> tuple[float, float]
  - def top_left_pixel_world_coords(self) -> tuple[float, float]
  - def bottom_right_pixel_world_coords(self) -> tuple[float, float]
  - def top_right_pixel_world_coords(self) -> tuple[float, float]
  - def buffered(self, buffer_distance_pixels: int) -> OccupancyMap
  - def buffered_meters(self, buffer_distance_meters: float) -> OccupancyMap
  - def pixel_to_world(self, point: Point2d) -> Point2d
  - def pixel_to_world_numpy(self, points: np.ndarray) -> np.ndarray
  - def world_to_pixel_numpy(self, points: np.ndarray) -> np.ndarray
  - def check_world_point_in_bounds(self, point: Point2d) -> bool
  - def check_world_point_in_freespace(self, point: Point2d) -> bool

- class PathHelper
  - def __init__(self, points: np.ndarray)
  - def get_path_length(self) -> float
  - def get_point_by_distance(self, distance: float) -> np.ndarray
  - def find_nearest(self, point: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int], float]

- class Pose2d(Point2d)
  - theta: float

- class RecordingSession
  - def __init__(self)
  - def build(self, robot_type: type, scenario_type: type, occupancy_map: OccupancyMap) -> MobilityGenScenario
  - def initialize(self)
  - def reset(self)
  - def step(self, step_size: float) -> bool
  - def enable_recording(self)
  - def disable_recording(self)
  - def clear_recording(self)
  - def clear(self)
  - def close(self)

- class UniformPoseSampler(PoseSampler)
  - def sample_px(self, occupancy_map: OccupancyMap) -> Pose2d

## Functions

- def apply_nurec_replay_overrides(args: argparse.Namespace, stage: Usd.Stage | None) -> bool
- def apply_sensor_overrides(robot_prim_path: str, recording_path: str, stage: Usd.Stage | None = None)
- async def collect_input(input_path: str, dest_dir: str) -> str
- def compress_path(path: np.ndarray, eps: float = 0.001) -> tuple[np.ndarray, np.ndarray]
- def generate_paths(start: tuple[int, int], freespace: np.ndarray) -> GeneratePathsOutput
- def is_complete(output_path: str, expected_config: dict[str, Any]) -> bool
- def is_nurec_stage(stage: Usd.Stage | None) -> bool
- def load_scenario(path: str) -> MobilityGenScenario
- def log_camera_properties(stage: Usd.Stage, robot_prim_path: str)
- def mark_replay_complete(output_path: str, frames_rendered: int)
- def replay_config_from_args(source_recording: str, args: argparse.Namespace) -> dict[str, Any]
- def save_sensor_overrides(robot_prim_path: str, output_dir: str, root_layer: Sdf.Layer | None = None, stage: Usd.Stage | None = None)
- def write_replay_config(output_path: str, replay_config: dict[str, Any])

## Variables

- COMPLETE_MARKER_NAME: str
- REPLAY_CONFIG_NAME: str
- ROBOTS: Unknown
- SCENARIOS: Unknown
- SensorConfig: CameraConfig
