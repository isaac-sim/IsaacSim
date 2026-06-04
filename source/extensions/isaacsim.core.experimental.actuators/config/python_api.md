# Public API for module isaacsim.core.experimental.actuators:

## Classes

- class ArticulationActuators
  - class def from_actuators(cls, paths: str | list[str], actuators: list[tuple[ActuatorConfig, str]]) -> ArticulationActuators
  - def __init__(self, paths: str | list[str])
  - [property] def articulation(self) -> Articulation
  - [property] def actuators(self) -> list[Actuator]
  - [property] def actuated_dof_indices(self) -> list[int]
  - def close(self)
  - def set_dof_feedforward_effort_targets(self, target_feedforward_efforts: float | list | np.ndarray | wp.array)
  - def enable_auto_step_pre_physics(self)
  - def disable_auto_step_pre_physics(self)
  - def reset(self)
  - def step_actuators(self, step_dt: float, context: Any = None)

- class ActuatorConfig
  - controller: Controller
  - clamping: list[Clamping]
  - delay: Delay | None

- class DCMotorClampingConfig
  - saturation_effort: float
  - velocity_limit: float
  - max_motor_effort: float

- class DelayConfig
  - delay_steps: int

- class MaxEffortClampingConfig
  - max_effort: float

- class NeuralControlConfig
  - model_path: str

- class PDControlConfig
  - kp: float
  - kd: float
  - const_effort: float

- class PIDControlConfig
  - kp: float
  - ki: float
  - kd: float
  - integral_max: float
  - const_effort: float

- class PositionBasedClampingConfig
  - lookup_positions: list[float]
  - lookup_efforts: list[float]

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

## Functions

- def add_actuator(articulation_root: str | Sdf.Path, target_names: str | list[str], name: str, controller: PDControlConfig | PIDControlConfig | NeuralControlConfig) -> Usd.Prim
