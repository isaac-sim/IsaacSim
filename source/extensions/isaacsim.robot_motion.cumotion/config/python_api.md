# Public API for module isaacsim.robot_motion.cumotion:

## Classes

- class CumotionRobot
  - directory: pathlib.Path
  - robot_description: cumotion.RobotDescription
  - kinematics: cumotion.Kinematics
  - controlled_joint_names: list[str]

- class CumotionWorldInterface(mg.WorldInterface)
  - def __init__(self, world_to_robot_base: tuple[wp.array, wp.array] | None = None, visualize_debug_prims: bool = False, visual_debug_enabled_prim_rgb: list[float] | None = None, visual_debug_disabled_prim_rgb: list[float] | None = None, visual_debug_prim_alpha: float = 0.3)
  - [property] def world_view(self) -> cumotion.WorldView
  - def add_spheres(self, prim_paths: list[str], radii: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def add_cubes(self, prim_paths: list[str], sizes: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def add_triangulated_meshes(self, prim_paths: list[str], points: list[wp.array], face_vertex_indices: list[wp.array], scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def add_planes(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], lengths: wp.array, widths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def add_capsules(self, prim_paths: list[str], axes: list[Literal[X, Y, Z]], radii: wp.array, lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def add_oriented_bounding_boxes(self, prim_paths: list[str], centers: wp.array, rotations: wp.array, half_side_lengths: wp.array, scales: wp.array, safety_tolerances: wp.array, poses: tuple[wp.array, wp.array], enabled_array: wp.array)
  - def update_obstacle_transforms(self, prim_paths: list[str], poses: tuple[wp.array, wp.array])
  - def update_obstacle_enables(self, prim_paths: list[str], enabled_array: wp.array)
  - def get_world_to_robot_base_transform(self) -> tuple[wp.array, wp.array]
  - def update_world_to_robot_root_transforms(self, poses: tuple[wp.array, wp.array])

- class RmpFlowController(mg.BaseController)
  - def __init__(self, cumotion_robot: CumotionRobot, cumotion_world_interface: CumotionWorldInterface, robot_joint_space: list[str], robot_site_space: list[str], rmp_flow_configuration_filename: pathlib.Path | str = 'rmp_flow.yaml', tool_frame: str | None = None)
  - def get_rmp_flow_config(self) -> cumotion.RmpFlowConfig
  - def forward(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs: Any) -> mg.RobotState | None
  - def reset(self, estimated_state: mg.RobotState, setpoint_state: mg.RobotState | None, t: float, **kwargs: Any) -> bool

- class GraphBasedMotionPlanner
  - def __init__(self, cumotion_robot: CumotionRobot, cumotion_world_interface: CumotionWorldInterface, tool_frame: str | None = None, graph_planner_config_filename: pathlib.Path | str | None = None)
  - def get_cumotion_robot(self) -> CumotionRobot
  - def get_graph_planner_config(self) -> cumotion.MotionPlannerConfig
  - def plan_to_cspace_target(self, q_initial: wp.array | np.ndarray | list[float], q_final: wp.array | np.ndarray | list[float]) -> Path | None
  - def plan_to_pose_target(self, q_initial: wp.array | np.ndarray | list[float], position: wp.array | np.ndarray | list[float], orientation: wp.array | np.ndarray | list[float]) -> Path | None
  - def plan_to_translation_target(self, q_initial: wp.array | np.ndarray | list[float], translation_target: wp.array | np.ndarray | list[float]) -> Path | None

- class TrajectoryGenerator
  - def __init__(self, cumotion_robot: CumotionRobot, robot_joint_space: list[str])
  - def get_cspace_trajectory_generator(self) -> cumotion.CSpaceTrajectoryGenerator
  - def generate_trajectory_from_path_specification(self, path_specification: cumotion.CSpacePathSpec | cumotion.TaskSpacePathSpec | cumotion.CompositePathSpec, tool_frame_name: str | None = None, task_space_conversion_config: cumotion.TaskSpacePathConversionConfig | None = None, inverse_kinematics_config: cumotion.IkConfig | None = None) -> CumotionTrajectory | None
  - def generate_trajectory_from_cspace_waypoints(self, waypoints: wp.array | np.ndarray | list[list[float]], times: wp.array | np.ndarray | list[float] | None = None, interpolation_mode: cumotion.CSpaceTrajectoryGenerator.InterpolationMode | None = None) -> CumotionTrajectory | None

- class TrajectoryOptimizer
  - def __init__(self, cumotion_robot: CumotionRobot, robot_joint_space: list[str], cumotion_world_interface: CumotionWorldInterface, tool_frame: str | None = None, trajectory_optimizer_config_filename: pathlib.Path | str | None = None)
  - def get_cumotion_robot(self) -> CumotionRobot
  - def get_trajectory_optimizer_config(self) -> cumotion.TrajectoryOptimizerConfig
  - def plan_to_goal(self, initial_cspace_position: wp.array | np.ndarray | list[float], goal: cumotion.TrajectoryOptimizer.CSpaceTarget | cumotion.TrajectoryOptimizer.TaskSpaceTargetGoalset | cumotion.TrajectoryOptimizer.TaskSpaceTarget) -> CumotionTrajectory | None

- class CumotionTrajectory(mg.Trajectory)
  - def __init__(self, trajectory: cumotion.Trajectory, robot_joint_space: list[str], cumotion_robot: CumotionRobot, device: wp.Device | None = None)
  - [property] def duration(self) -> float
  - def get_active_joints(self) -> list[str]
  - def get_target_state(self, time: float) -> mg.RobotState | None

## Functions

- def load_cumotion_robot(directory: pathlib.Path | str, urdf_filename: pathlib.Path | str = 'robot.urdf', xrdf_filename: pathlib.Path | str = 'robot.xrdf') -> CumotionRobot
- def load_cumotion_supported_robot(robot_name: str) -> CumotionRobot
