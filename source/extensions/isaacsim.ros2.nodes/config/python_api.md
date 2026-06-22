# Public API for module isaacsim.ros2.nodes:

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

- class ROS2NodesExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)
  - def register_nodes(self)
  - def unregister_nodes(self)

- class SrtxSensorSetConfig
  - name: str
  - render_product_paths: list[str] | None

- class SrtxCaptureState
  - def __init__(self)
  - def start_or_extend(self, srtx_instance: object, sensor_set_name: str, output_path: str)
  - def stop_or_shrink(self, srtx_instance: object, sensor_set_name: str, output_paths_to_remove: list[str])

- class CompressedImageManager
  - class def reset(cls)
  - class def attach(cls, render_product_path: str)
  - class def detach(cls, render_product_path: str)
  - class def get_writer(cls, render_product_path: str, use_system_time: bool = False) -> rep.Writer

## Functions

- def read_camera_info(render_product_path: str) -> tuple
- def compute_relative_pose(left_camera_prim: Usd.Prim, right_camera_prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]
- def collect_namespace(namespace_input: str, render_product_path: str) -> str
- def register_node_writer_with_telemetry(*args: Any, **kwargs: Any)
- def acquire_interface(plugin_name: str = None, library_path: str = None) -> IRos2Nodes
- def release_interface(arg0: IRos2Nodes)
- def is_srtx_supported_platform() -> bool
- def validate_srtx_platform() -> bool
- def get_srtx_sensor_set_config(render_product_path: str | None = None) -> SrtxSensorSetConfig
- def get_srtx_sensor_set_name(render_product_path: str | None = None) -> str
- def prepare_srtx_sensor_set(srtx_instance: object, render_product_path: str) -> str | None
- def ensure_render_var_on_product(stage: object, render_product_path: str, aov_name: str, compression_type: str | None = None, is_image: bool = False) -> tuple[bool, str | None]
- def cleanup_srtx_state(state: object)

## Variables

- OPENCV_PINHOLE_ATTRIBUTE_MAP: List
- OPENCV_FISHEYE_ATTRIBUTE_MAP: List
- BRIDGE_NAME: str
- BRIDGE_PREFIX: str
- USE_SRTX_SETTING: str
- SRTX_SENSOR_SET_NAME_SETTING: str
- SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING: str
- SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING: str
- NON_IMAGE_COMPRESSION_FALLBACK: str
- AOV_ALIASES: dict[str, set[str]]
