# Public API for module isaacsim.robot_motion.lula_test_widget:

## Classes

- class LulaTestScenarios
  - def __init__(self)
  - def visualize_ee_frame(self, articulation, ee_frame)
  - def stop_visualize_ee_frame(self)
  - def toggle_rmpflow_debug_mode(self)
  - def initialize_ik_solver(self, robot_description_path, urdf_path)
  - def get_ik_frames(self)
  - def on_ik_follow_target(self, articulation, ee_frame_name)
  - def on_custom_trajectory(self, robot_description_path, urdf_path)
  - def create_trajectory_controller(self, articulation, ee_frame)
  - def delete_waypoint(self)
  - def add_waypoint(self)
  - def on_rmpflow_follow_target_obstacles(self, articulation, **rmp_config)
  - def on_rmpflow_follow_sinusoidal_target(self, articulation, **rmp_config)
  - def get_rmpflow(self)
  - def set_use_orientation(self, use_orientation)
  - def full_reset(self)
  - def scenario_reset(self)
  - def update_scenario(self, **scenario_params)
  - def get_next_action(self, **scenario_params)

## Functions

- def is_yaml_file(path: str) -> bool
- def is_urdf_file(path: str) -> bool
- def on_filter_yaml_item(item) -> bool
- def on_filter_urdf_item(item) -> bool
