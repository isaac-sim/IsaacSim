# Public API for module isaacsim.robot.surface_gripper:

## Classes

- class GripperView(XformPrim)
  - def __init__(self, paths: str = None, max_grip_distance: np.ndarray | wp.array | None = None, coaxial_force_limit: np.ndarray | wp.array | None = None, shear_force_limit: np.ndarray | wp.array | None = None, retry_interval: np.ndarray | wp.array | None = None, positions: np.ndarray | wp.array | None = None, translations: np.ndarray | wp.array | None = None, orientations: np.ndarray | wp.array | None = None, scales: np.ndarray | wp.array | None = None, reset_xform_op_properties: bool = True)
  - def get_surface_gripper_status(self, indices: list | np.ndarray | wp.array | None = None) -> list[str]
  - def get_gripped_objects(self, indices: list | np.ndarray | wp.array | None = None) -> list[str]
  - def get_surface_gripper_properties(self, indices: list | np.ndarray | wp.array | None = None) -> tuple[list[float], list[float], list[float], list[float]]
  - def apply_gripper_action(self, values: list[float], indices: list | np.ndarray | wp.array | None = None)
  - def set_surface_gripper_properties(self, max_grip_distance: list[float] | None = None, coaxial_force_limit: list[float] | None = None, shear_force_limit: list[float] | None = None, retry_interval: list[float] | None = None, indices: list | np.ndarray | wp.array | None = None)

## Functions

- def create_surface_gripper(stage: Usd.Stage, prim_path: str) -> Usd.Prim
