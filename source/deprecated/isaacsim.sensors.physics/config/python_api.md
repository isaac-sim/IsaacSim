# Public API for module isaacsim.sensors.physics:

## Classes

- class IsaacSensorCreatePrim(omni.kit.commands.Command)
  - def __init__(self, path: str = '', parent: str = '', translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), schema_type: type = IsaacSensorSchema.IsaacBaseSensor)
  - def do(self) -> object
  - def undo(self) -> object

- class IsaacSensorCreateContactSensor(omni.kit.commands.Command)
  - def __init__(self, path: str = '/Contact_Sensor', parent: str = None, min_threshold: float = 0, max_threshold: float = 100000, color: Gf.Vec4f = Gf.Vec4f(1, 1, 1, 1), radius: float = -1, sensor_period: float = -1, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0))
  - def do(self) -> object
  - def undo(self)

- class IsaacSensorCreateImuSensor(omni.kit.commands.Command)
  - def __init__(self, path: str = '/Imu_Sensor', parent: str = None, sensor_period: float = -1, translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0), linear_acceleration_filter_size: int = 1, angular_velocity_filter_size: int = 1, orientation_filter_size: int = 1)
  - def do(self) -> object
  - def undo(self)

- class ContactSensor(BaseSensor)
  - def __init__(self, prim_path: str, name: str | None = 'contact_sensor', frequency: int | None = None, dt: float | None = None, translation: np.ndarray | None = None, position: np.ndarray | None = None, min_threshold: float | None = None, max_threshold: float | None = None, radius: float | None = None)
  - def initialize(self, physics_sim_view: object = None)
  - def get_current_frame(self) -> dict
  - def add_raw_contact_data_to_frame(self)
  - def remove_raw_contact_data_from_frame(self)
  - def resume(self)
  - def pause(self)
  - def is_paused(self) -> bool
  - def set_frequency(self, value: float)
  - def get_frequency(self) -> int
  - def get_dt(self) -> float
  - def set_dt(self, value: float)
  - def get_radius(self) -> float
  - def set_radius(self, value: float)
  - def get_min_threshold(self) -> float
  - def set_min_threshold(self, value: float)
  - def get_max_threshold(self) -> float
  - def set_max_threshold(self, value: float)

- class EffortSensor(SingleArticulation)
  - def __init__(self, prim_path: str, sensor_period: float = -1, use_latest_data: bool = False, enabled: bool = True)
  - def initialize_callbacks(self)
  - def lerp(self, start: float, end: float, time: float) -> float
  - def get_sensor_reading(self, interpolation_function: object = None, use_latest_data: bool = False) -> EsSensorReading
  - def update_dof_name(self, dof_name: str)
  - def change_buffer_size(self, new_buffer_size: int)

- class EsSensorReading
  - def __init__(self, is_valid: bool = False, time: float = 0, value: float = 0)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class IMUSensor(BaseSensor)
  - def __init__(self, prim_path: str, name: str | None = 'imu_sensor', frequency: int | None = None, dt: float | None = None, translation: np.ndarray | None = None, position: np.ndarray | None = None, orientation: np.ndarray | None = None, linear_acceleration_filter_size: int | None = 1, angular_velocity_filter_size: int | None = 1, orientation_filter_size: int | None = 1)
  - def initialize(self, physics_sim_view: object = None)
  - def get_current_frame(self, read_gravity: bool = True) -> dict
  - def resume(self)
  - def pause(self)
  - def is_paused(self) -> bool
  - def set_frequency(self, value: int)
  - def get_frequency(self) -> int
  - def get_dt(self) -> float
  - def set_dt(self, value: float)
