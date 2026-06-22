# Public API for module isaacsim.robot.experimental.wheeled_robots:

## Classes

- class AckermannController
  - def __init__(self)
  - def forward(self, command: np.ndarray) -> tuple[tuple[float, float] | None, tuple[float, float, float, float] | None]

- class DifferentialController
  - def __init__(self)
  - def forward(self, command: np.ndarray) -> np.ndarray

- class HolonomicController
  - def __init__(self)
  - def forward(self, command: np.ndarray) -> np.ndarray

- class QuinticPolynomial
  - def __init__(self, xs: float, vxs: float, axs: float, xe: float, vxe: float, axe: float, time: float)
  - def calc_point(self, t: float) -> float
  - def calc_first_derivative(self, t: float) -> float
  - def calc_second_derivative(self, t: float) -> float
  - def calc_third_derivative(self, t: float) -> float

- class State
  - def __init__(self, wheel_base: float, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, v: float = 0.0, max_steering_angle: float = np.radians(5.0))
  - def update(self, acceleration: float, delta: float, dt: float)

## Functions

- def quintic_polynomials_planner(sx: float, sy: float, syaw: float, sv: float, sa: float, gx: float, gy: float, gyaw: float, gv: float, ga: float, max_accel: float, max_jerk: float, dt: float) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]
- def calc_target_index(state: State, cx: list[float], cy: list[float]) -> tuple[int, float]
- def normalize_angle(angle: float) -> float
- def pid_control(target: float, current: float, kp: float = 0.1) -> float
- def stanley_control(state: State, cx: list[float], cy: list[float], cyaw: list[float], last_target_idx: int, p: float = 0.5, i: float = 0.01, d: float = 10.0, k: float = 0.5) -> tuple[float, int]
