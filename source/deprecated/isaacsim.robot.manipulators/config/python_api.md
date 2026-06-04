# Public API for module isaacsim.robot.manipulators:

## Classes

- class SingleManipulator(SingleArticulation)
  - def __init__(self, prim_path: str, end_effector_prim_path: str, name: str = 'single_manipulator', position: Optional[Sequence[float]] = None, translation: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None, scale: Optional[Sequence[float]] = None, visible: Optional[bool] = None, gripper: Gripper = None)
  - [property] def end_effector(self) -> SingleRigidPrim
  - [property] def gripper(self) -> Gripper
  - def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None)
  - def post_reset(self)
