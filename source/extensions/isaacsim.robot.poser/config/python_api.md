# Public API for module isaacsim.robot.poser:

## Classes

- class RobotPoser
  - def __init__(self, stage: Usd.Stage, robot_prim: Usd.Prim, start_prim: Usd.Prim | None = None, end_prim: Usd.Prim | None = None, solver_name: str | None = None)
  - [property] def stage(self) -> Usd.Stage
  - [property] def robot_prim(self) -> Usd.Prim
  - [property] def start_prim(self) -> Usd.Prim | None
  - [property] def end_prim(self) -> Usd.Prim | None
  - [property] def joints(self) -> list
  - [property] def chain(self) -> KinematicChain | None
  - [property] def solver(self) -> IKSolver
  - [solver.setter] def solver(self, value: IKSolver)
  - def set_chain(self, start_prim: Usd.Prim, end_prim: Usd.Prim)
  - def set_seed(self, seed: dict[str, float] | np.ndarray | list[float] | None)
  - def solve_ik(self, target: Transform, seed: dict[str, float] | np.ndarray | list[float] | None = None, **solver_kwargs: Any) -> PoseResult
  - def joints_to_native_values(self, joint_dict: dict[str, float]) -> list[float]
  - def apply_pose(self, joint_dict: dict[str, float] | PoseResult)
  - class def apply_pose_by_target(cls, stage: Usd.Stage, robot_prim: Usd.Prim, start_prim: Usd.Prim, end_prim: Usd.Prim, target: Transform, seed: VecN | None = None) -> PoseResult

- class PoseResult
  - success: bool
  - joints: dict[str, float]
  - joint_fixed: dict[str, bool]
  - start_link: str
  - end_link: str
  - target_position: list[float] | None
  - target_orientation: list[float] | None

## Functions

- def validate_robot_schema(robot_prim: Usd.Prim) -> bool
- def apply_joint_state(stage: Usd.Stage, robot_prim: Usd.Prim, joint_dict: dict[str, float])
- def apply_joint_state_anchored(stage: Usd.Stage, robot_prim: Usd.Prim, joint_dict: dict[str, float], anchor_prim: Usd.Prim)
- def store_named_pose(stage: Usd.Stage, robot_prim: Usd.Prim, pose_name: str, pose_result: PoseResult) -> bool
- def apply_pose_by_name(stage: Usd.Stage, robot_prim: Usd.Prim, pose_name: str) -> bool
- def get_named_pose(stage: Usd.Stage, robot_prim: Usd.Prim, pose_name: str) -> PoseResult | None
- def list_named_poses(stage: Usd.Stage, robot_prim: Usd.Prim) -> list[str]
- def delete_named_pose(stage: Usd.Stage, robot_prim: Usd.Prim, pose_name: str) -> bool
- def export_poses(stage: Usd.Stage, robot_prim: Usd.Prim, filepath: str) -> bool
- def import_poses(stage: Usd.Stage, robot_prim: Usd.Prim, filepath: str) -> int
