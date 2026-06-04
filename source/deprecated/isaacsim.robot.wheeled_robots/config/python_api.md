# Public API for module isaacsim.robot.wheeled_robots:

## Classes

- class DifferentialController(BaseController)
  - def __init__(self, name: str, wheel_radius: float, wheel_base: float, max_linear_speed: float = 1e+20, max_angular_speed: float = 1e+20, max_wheel_speed: float = 1e+20)
  - def forward(self, command: np.ndarray) -> ArticulationAction

- class HolonomicController(BaseController)
  - def __init__(self, name: str, wheel_radius: Optional[np.ndarray] = None, wheel_positions: Optional[np.ndarray] = None, wheel_orientations: Optional[np.ndarray] = None, mecanum_angles: Optional[np.ndarray] = None, wheel_axis: float = np.array([1, 0, 0]), up_axis: float = np.array([0, 0, 1]), max_linear_speed: float = 1e+20, max_angular_speed: float = 1e+20, max_wheel_speed: float = 1e+20, linear_gain: float = 1.0, angular_gain: float = 1.0)
  - def build_base(self)
  - def forward(self, command: np.ndarray) -> ArticulationAction
  - def reset(self)

- class WheelBasePoseController(BaseController)
  - def __init__(self, name: str, open_loop_wheel_controller: BaseController, is_holonomic: bool = False)
  - def forward(self, start_position: np.ndarray, start_orientation: np.ndarray, goal_position: np.ndarray, lateral_velocity: float = 0.2, yaw_velocity: float = 0.5, heading_tol: float = 0.05, position_tol: float = 0.04) -> ArticulationAction
  - def reset(self)

- class QuinticPolynomial
  - def __init__(self, xs, vxs, axs, xe, vxe, axe, time)
  - def calc_point(self, t)
  - def calc_first_derivative(self, t)
  - def calc_second_derivative(self, t)
  - def calc_third_derivative(self, t)

## Functions

- def quintic_polynomials_planner(sx: float, sy: float, syaw: float, sv: float, sa: float, gx: float, gy: float, gyaw: float, gv: float, ga: float, max_accel: float, max_jerk: float, dt: float) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]
- def stanley_control(state, cx: list[float], cy: list[float], cyaw: list[float], last_target_idx: int, p: float = 0.5, i: float = 0.01, d: float = 10, k: float = 0.5) -> tuple[float, int]
- def pid_control(target: float, current: float, Kp: float = 0.1) -> float
