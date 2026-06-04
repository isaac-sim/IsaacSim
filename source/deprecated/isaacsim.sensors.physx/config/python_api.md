# Public API for module isaacsim.sensors.physx:

## Classes

- class RangeSensorCreatePrim(omni.kit.commands.Command)
  - def __init__(self, path: str = '', parent: str = '', schema_type = RangeSensorSchema.Lidar, translation: Optional[Gf.Vec3d] = Gf.Vec3d(0, 0, 0), orientation: Optional[Gf.Quatd] = Gf.Quatd(1, 0, 0, 0), visibility: Optional[bool] = False, min_range: Optional[float] = 0.4, max_range: Optional[float] = 100.0, draw_points: Optional[bool] = False, draw_lines: Optional[bool] = False)
  - def do(self)
  - def undo(self)

- class RangeSensorCreateLidar(omni.kit.commands.Command)
  - def __init__(self, path: str = '/Lidar', parent = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, horizontal_fov: float = 360.0, vertical_fov: float = 30.0, horizontal_resolution: float = 0.4, vertical_resolution: float = 4.0, rotation_rate: float = 20.0, high_lod: bool = False, yaw_offset: float = 0.0, enable_semantics: bool = False)
  - def do(self)
  - def undo(self)

- class RangeSensorCreateGeneric(omni.kit.commands.Command)
  - def __init__(self, path: str = '/GenericSensor', parent = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, sampling_rate: int = 60)
  - def do(self)
  - def undo(self)

- class IsaacSensorCreateLightBeamSensor(omni.kit.commands.Command)
  - def __init__(self, path: str = '/LightBeam_Sensor', parent: str = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), num_rays: int = 1, curtain_length: float = 0.0, forward_axis: Gf.Vec3d = Gf.Vec3d(1, 0, 0), curtain_axis: Gf.Vec3d = Gf.Vec3d(0, 0, 1), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, **kwargs)
  - def do(self)
  - def undo(self)

- class ProximitySensor
  - def __init__(self, parent: Usd.Prim, callback_fns = [None, None, None], exclusions = [])
  - def update(self)
  - def report_hit(self, hit) -> bool
  - def check_for_overlap(self) -> int
  - def status(self) -> tuple[bool, dict[str, dict[str, float]]]
  - def reset(self)
  - def get_data(self) -> Dict[str, Dict[str, float]]
  - def to_string(self) -> str
  - def is_overlapping(self) -> bool
  - def get_active_zones(self) -> List[str]
  - def get_entered_zones(self) -> List[str]
  - def get_exited_zones(self) -> List[str]

- class RotatingLidarPhysX(BaseSensor)
  - def __init__(self, prim_path: str, name: str = 'rotating_lidar_physX', rotation_frequency: Optional[float] = None, rotation_dt: Optional[float] = None, position: Optional[np.ndarray] = None, translation: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None, fov: Optional[Tuple[float, float]] = None, resolution: Optional[Tuple[float, float]] = None, valid_range: Optional[Tuple[float, float]] = None)
  - def initialize(self, physics_sim_view = None)
  - def post_reset(self)
  - def add_depth_data_to_frame(self)
  - def remove_depth_data_from_frame(self)
  - def add_linear_depth_data_to_frame(self)
  - def remove_linear_depth_data_from_frame(self)
  - def add_intensity_data_to_frame(self)
  - def remove_intensity_data_from_frame(self)
  - def add_zenith_data_to_frame(self)
  - def remove_zenith_data_from_frame(self)
  - def add_azimuth_data_to_frame(self)
  - def remove_azimuth_data_from_frame(self)
  - def add_point_cloud_data_to_frame(self)
  - def remove_point_cloud_data_from_frame(self)
  - def add_semantics_data_to_frame(self)
  - def remove_semantics_data_from_frame(self)
  - def get_num_rows(self) -> int
  - def get_num_cols(self) -> int
  - def get_num_cols_in_last_step(self) -> int
  - def get_current_frame(self) -> dict
  - def resume(self)
  - def pause(self)
  - def is_paused(self) -> bool
  - def set_fov(self, value: Tuple[float, float])
  - def get_fov(self) -> Tuple[float, float]
  - def set_resolution(self, value: float)
  - def get_resolution(self) -> float
  - def set_rotation_frequency(self, value: int)
  - def get_rotation_frequency(self) -> int
  - def set_valid_range(self, value: Tuple[float, float])
  - def get_valid_range(self) -> Tuple[float, float]
  - def enable_semantics(self)
  - def disable_semantics(self)
  - def is_semantics_enabled(self) -> bool
  - def enable_visualization(self, high_lod: bool = False, draw_points: bool = True, draw_lines: bool = True)
  - def disable_visualization(self)
