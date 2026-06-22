# Public API for module isaacsim.ros2.core:

## Classes

- class ViewportManager
  - class def wait_for_viewport(cls) -> tuple[bool, int]
  - class async def wait_for_viewport_async(cls) -> tuple[bool, int]
  - class def set_camera(cls, camera: str | Usd.Prim | UsdGeom.Camera)
  - class def get_camera(cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | 'ViewportAPI' | None = None) -> UsdGeom.Camera
  - class def get_viewport_api(cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | 'ViewportAPI' | None = None) -> 'ViewportAPI' | None
  - class def get_render_product(cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | 'ViewportAPI' | None = None) -> UsdRender.Product | None
  - class def get_resolution(cls, render_product_or_viewport: str | Usd.Prim | UsdRender.Product | 'ViewportAPI' | None = None) -> tuple[int, int]
  - class def set_resolution(cls, resolution: tuple[int, int] | str)
  - class def create_viewport_window(cls) -> ViewportWindow
  - class def get_viewport_windows(cls) -> list
  - class def destroy_viewport_windows(cls) -> list[str]
  - class def set_camera_view(cls, camera: str | Usd.Prim | UsdGeom.Camera)

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

## Variables

- OPENCV_PINHOLE_ATTRIBUTE_MAP: List
- OPENCV_FISHEYE_ATTRIBUTE_MAP: List
- SUPPORTED_ROS_DISTROS: Dict
- BRIDGE_NAME: str
- BRIDGE_PREFIX: str
