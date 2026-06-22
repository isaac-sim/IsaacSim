# Public API for module isaacsim.sensors.physx:

## Classes

- class RangeSensorCreatePrim(omni.kit.commands.Command)
  - def __init__(self, path: str = '', parent: str = '', schema_type: type = RangeSensorSchema.Lidar, translation: Optional[Gf.Vec3d] = Gf.Vec3d(0, 0, 0), orientation: Optional[Gf.Quatd] = Gf.Quatd(1, 0, 0, 0), visibility: Optional[bool] = False, min_range: Optional[float] = 0.4, max_range: Optional[float] = 100.0, draw_points: Optional[bool] = False, draw_lines: Optional[bool] = False)
  - def do(self) -> Any
  - def undo(self) -> Any

- class RangeSensorCreateLidar(omni.kit.commands.Command)
  - def __init__(self, path: str = '/Lidar', parent: object = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, horizontal_fov: float = 360.0, vertical_fov: float = 30.0, horizontal_resolution: float = 0.4, vertical_resolution: float = 4.0, rotation_rate: float = 20.0, high_lod: bool = False, yaw_offset: float = 0.0, enable_semantics: bool = False)
  - def do(self) -> Any
  - def undo(self) -> Any

- class RangeSensorCreateGeneric(omni.kit.commands.Command)
  - def __init__(self, path: str = '/GenericSensor', parent: object = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, sampling_rate: int = 60)
  - def do(self) -> Any
  - def undo(self) -> Any

- class IsaacSensorCreateLightBeamSensor(omni.kit.commands.Command)
  - def __init__(self, path: str = '/LightBeam_Sensor', parent: str = None, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), num_rays: int = 1, curtain_length: float = 0.0, forward_axis: Gf.Vec3d = Gf.Vec3d(1, 0, 0), curtain_axis: Gf.Vec3d = Gf.Vec3d(0, 0, 1), min_range: float = 0.4, max_range: float = 100.0, draw_points: bool = False, draw_lines: bool = False, **kwargs: Any)
  - def do(self) -> Any
  - def undo(self) -> Any

- class ProximitySensorManager(object)
  - def register_sensor(self, sensor: ProximitySensor)
  - def clear_sensors(self)
  - def update(self)

- class Extension(omni.ext.IExt)
  - def on_startup(self)
  - def on_shutdown(self)

- class ProximitySensor
  - def __init__(self, parent: Usd.Prim, callback_fns: list | None = None, exclusions: list | None = None)
  - def update(self)
  - def report_hit(self, hit: object) -> bool
  - def check_for_overlap(self) -> int
  - def status(self) -> tuple[bool, dict[str, dict[str, float]]]
  - def reset(self)
  - def get_data(self) -> dict[str, dict[str, float]]
  - def to_string(self) -> str
  - def is_overlapping(self) -> bool
  - def get_active_zones(self) -> list[str]
  - def get_entered_zones(self) -> list[str]
  - def get_exited_zones(self) -> list[str]

- class RotatingLidarPhysX(BaseSensor)
  - def __init__(self, prim_path: str, name: str = 'rotating_lidar_physX', rotation_frequency: Optional[float] = None, rotation_dt: Optional[float] = None, position: Optional[np.ndarray] = None, translation: Optional[np.ndarray] = None, orientation: Optional[np.ndarray] = None, fov: Optional[tuple[float, float]] = None, resolution: Optional[tuple[float, float]] = None, valid_range: Optional[tuple[float, float]] = None)
  - def initialize(self, physics_sim_view: object = None)
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
  - def set_fov(self, value: tuple[float, float])
  - def get_fov(self) -> tuple[float, float]
  - def set_resolution(self, value: float)
  - def get_resolution(self) -> float
  - def set_rotation_frequency(self, value: int)
  - def get_rotation_frequency(self) -> int
  - def set_valid_range(self, value: tuple[float, float])
  - def get_valid_range(self) -> tuple[float, float]
  - def enable_semantics(self)
  - def disable_semantics(self)
  - def is_semantics_enabled(self) -> bool
  - def enable_visualization(self, high_lod: bool = False, draw_points: bool = True, draw_lines: bool = True)
  - def disable_visualization(self)

## Functions

- def get_next_free_path(path: str, parent: str = None) -> str
- def reset_and_set_xform_ops(prim: Usd.Prim, translation: Gf.Vec3d, orientation: Gf.Quatd, scale: Gf.Vec3d = Gf.Vec3d([1.0, 1.0, 1.0]))
- def setup_base_prim(prim: object, schema_type: type, enabled: bool, draw_points: bool, draw_lines: bool, min_range: float, max_range: float)
- def clear_sensors()
- def register_sensor(sensor: ProximitySensor)
