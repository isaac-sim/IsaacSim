# Public API for module isaacsim.robot.wheeled_robots:

## Classes

- class AckermannController(BaseController)
  - def __init__(self, name: str, wheel_base: float, track_width: float, front_wheel_radius: float = 0.0, back_wheel_radius: float = 0.0, max_wheel_velocity: float = 0.0, invert_steering: bool = False, max_wheel_rotation_angle: float = 6.28, max_acceleration: float = 0.0, max_steering_angle_velocity: float = 0.0)
  - def forward(self, command: np.ndarray) -> ArticulationAction

- class DifferentialController(BaseController)
  - def __init__(self, name: str, wheel_radius: float, wheel_base: float, max_linear_speed: float = 1e+20, max_angular_speed: float = 1e+20, max_wheel_speed: float = 1e+20)
  - def forward(self, command: np.ndarray) -> ArticulationAction

- class HolonomicController(BaseController)
  - def __init__(self, name: str, wheel_radius: Optional[np.ndarray] = None, wheel_positions: Optional[np.ndarray] = None, wheel_orientations: Optional[np.ndarray] = None, mecanum_angles: Optional[np.ndarray] = None, wheel_axis: float = np.array([1, 0, 0]), up_axis: float = np.array([0, 0, 1]), max_linear_speed: float = 1e+20, max_angular_speed: float = 1e+20, max_wheel_speed: float = 1e+20, linear_gain: float = 1.0, angular_gain: float = 1.0)
  - def build_base(self)
  - def forward(self, command: np.ndarray) -> ArticulationAction
  - def reset(self)

- class QuinticPolynomial
  - def __init__(self, xs: float, vxs: float, axs: float, xe: float, vxe: float, axe: float, time: float)
  - def calc_point(self, t: float) -> float
  - def calc_first_derivative(self, t: float) -> float
  - def calc_second_derivative(self, t: float) -> float
  - def calc_third_derivative(self, t: float) -> float

- class State(object)
  - def __init__(self, wheel_base: float, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, v: float = 0.0, Ks: float = np.radians(5.0))
  - def update(self, acceleration: float, delta: float, dt: float)

- class WheelBasePoseController(BaseController)
  - def __init__(self, name: str, open_loop_wheel_controller: BaseController, is_holonomic: bool = False)
  - def forward(self, start_position: np.ndarray, start_orientation: np.ndarray, goal_position: np.ndarray, lateral_velocity: float = 0.2, yaw_velocity: float = 0.5, heading_tol: float = 0.05, position_tol: float = 0.04) -> ArticulationAction
  - def reset(self)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

## Functions

- def quintic_polynomials_planner(sx: float, sy: float, syaw: float, sv: float, sa: float, gx: float, gy: float, gyaw: float, gv: float, ga: float, max_accel: float, max_jerk: float, dt: float) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]
- def calc_target_index(state: State, cx: list[float], cy: list[float]) -> tuple[int, float]
- def normalize_angle(angle: float) -> float
- def pid_control(target: float, current: float, Kp: float = 0.1) -> float
- def stanley_control(state: State, cx: list[float], cy: list[float], cyaw: list[float], last_target_idx: int, p: float = 0.5, i: float = 0.01, d: float = 10, k: float = 0.5) -> tuple[float, int]
