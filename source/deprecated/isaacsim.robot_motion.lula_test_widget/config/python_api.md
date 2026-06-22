# Public API for module isaacsim.robot_motion.lula_test_widget:

## Classes

- class LulaTestScenarios
  - def __init__(self)
  - def visualize_ee_frame(self, articulation: object, ee_frame: str)
  - def stop_visualize_ee_frame(self)
  - def toggle_rmpflow_debug_mode(self)
  - def initialize_ik_solver(self, robot_description_path: str, urdf_path: str)
  - def get_ik_frames(self) -> list
  - def on_ik_follow_target(self, articulation: object, ee_frame_name: str)
  - def on_custom_trajectory(self, robot_description_path: str, urdf_path: str)
  - def create_trajectory_controller(self, articulation: object, ee_frame: str)
  - def delete_waypoint(self)
  - def add_waypoint(self)
  - def on_rmpflow_follow_target_obstacles(self, articulation: object, **rmp_config: object)
  - def on_rmpflow_follow_sinusoidal_target(self, articulation: object, **rmp_config: object)
  - def get_rmpflow(self) -> RmpFlow | None
  - def set_use_orientation(self, use_orientation: bool)
  - def full_reset(self)
  - def scenario_reset(self)
  - def update_scenario(self, **scenario_params: object)
  - def get_next_action(self, **scenario_params: object) -> ArticulationAction

## Functions

- def is_yaml_file(path: str) -> bool
- def is_urdf_file(path: str) -> bool
- def on_filter_yaml_item(item: object) -> bool
- def on_filter_urdf_item(item: object) -> bool
