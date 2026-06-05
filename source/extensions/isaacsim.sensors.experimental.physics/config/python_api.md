# Public API for module isaacsim.sensors.experimental.physics:

## Classes

- class ContactSensorReading
  - value: float
  - time: float
  - is_valid: bool
  - [property] def in_contact(self) -> bool
  - [in_contact.setter] def in_contact(self, value: bool)

- class IMURawData
  - time: float
  - dt: float
  - linear_velocity_x: float
  - linear_velocity_y: float
  - linear_velocity_z: float
  - angular_velocity_x: float
  - angular_velocity_y: float
  - angular_velocity_z: float
  - orientation_w: float
  - orientation_x: float
  - orientation_y: float
  - orientation_z: float

- class IMUSensorReading
  - linear_acceleration_x: float
  - linear_acceleration_y: float
  - linear_acceleration_z: float
  - angular_velocity_x: float
  - angular_velocity_y: float
  - angular_velocity_z: float
  - orientation: Quaternion
  - time: float
  - is_valid: bool

- class Contact(_PhysicsSensorAuthoring)
  - def __init__(self, path: str)
  - def get_radius(self) -> float | None
  - def set_radius(self, value: float)
  - def get_min_threshold(self) -> float | None
  - def set_min_threshold(self, value: float)
  - def get_max_threshold(self) -> float | None
  - def set_max_threshold(self, value: float)

- class ContactSensor(_PhysicsSensorRuntime)
  - def __init__(self, path: str | Contact)
  - [property] def contact(self) -> Contact
  - def on_physics_step(self, step_dt: float)
  - def on_timeline_stop(self)
  - def get_sensor_reading(self) -> ContactSensorReading
  - def get_raw_data(self) -> list[dict[str, object]]
  - def get_data(self) -> dict
  - def add_raw_contact_data_to_frame(self)
  - def remove_raw_contact_data_from_frame(self)

- class EffortSensor(_PhysicsSensorRuntimeBase)
  - def __init__(self, path: str, enabled: bool = True)
  - def on_timeline_stop(self)
  - def get_sensor_reading(self) -> EffortSensorReading
  - def get_data(self) -> dict
  - def update_dof_name(self, dof_name: str)
  - def change_buffer_size(self, new_buffer_size: int)

- class EffortSensorReading
  - def __init__(self, is_valid: bool = False, time: float = 0, value: float = 0)

- class IMU(_PhysicsSensorAuthoring)
  - def __init__(self, path: str)

- class IMUSensor(_PhysicsSensorRuntime)
  - [property] def imu(self) -> IMU
  - def get_sensor_reading(self, read_gravity: bool = True) -> object
  - def get_data(self, read_gravity: bool = True) -> dict

- class JointStateSensor(_PhysicsSensorRuntimeBase)
  - def __init__(self, path: str, enabled: bool = True)
  - def get_sensor_reading(self) -> JointStateSensorReading
  - def get_data(self) -> dict

- class JointStateSensorReading
  - def __init__(self, is_valid: bool = False, time: float = 0.0, dof_names: list[str] | None = None, positions: np.ndarray | None = None, velocities: np.ndarray | None = None, efforts: np.ndarray | None = None, dof_types: np.ndarray | None = None, stage_meters_per_unit: float = 0.0)

- class Raycast(_PhysicsSensorAuthoring)
  - def __init__(self, path: str)

- class RaycastSensor(_PhysicsSensorRuntime)
  - [property] def raycast(self) -> Raycast
  - def get_sensor_reading(self) -> object
  - def get_data(self) -> dict

## Functions

- def get_imu_sensor_interface() -> object | None
- def get_contact_sensor_interface() -> object | None
- def get_effort_sensor_interface() -> object | None
- def get_joint_state_sensor_interface() -> object | None
- def get_raycast_sensor_interface() -> object | None
