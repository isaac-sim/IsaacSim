# Public API for module isaacsim.ros2.core:

## Classes

- class ROS2CoreExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)
  - def check_status(self, distro: str) -> bool

## Functions

- def read_camera_info(render_product_path: str) -> tuple
- def compute_relative_pose(left_camera_prim: Usd.Prim, right_camera_prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]
- def collect_namespace(namespace_input: str, render_product_path: str) -> str
- def get_ubuntu_version() -> str | None
- def print_environment_setup_instructions(extension_path: str, ros_distro: str)
- def restore_ros2_python_paths()
- def setup_ros2_environment(extension_path: str, ros_distro: str)
