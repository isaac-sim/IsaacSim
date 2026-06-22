# Public API for module isaacsim.ros2.examples.ros_moveit_sample:

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

- class PhysicsScene
  - def __init__(self, prim: str | Usd.Prim)
  - [property] def path(self) -> str
  - [property] def prim(self) -> Usd.Prim
  - [property] def physics_scene(self) -> UsdPhysics.Scene
  - static def get_physics_scene_paths(stage: Usd.Stage | None = None) -> list[str]
  - def get_gravity(self) -> Gf.Vec3f
  - def set_gravity(self, gravity: Gf.Vec3f | tuple[float, float, float] | list[float])
  - def get_dt(self) -> float
  - def set_dt(self, dt: float)
  - def get_enabled_gravity(self) -> bool
  - def set_enabled_gravity(self, enabled: bool)
  - def get_max_solver_iterations(self) -> int
  - def set_max_solver_iterations(self, iterations: int)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def build_ui(self)
  - def create_ros_action_graph(self, franka_stage_path: str)
  - def create_franka(self, stage_path: str)
  - def on_shutdown(self)

## Functions

- def get_instance() -> Optional[ExampleBrowserExtension]
- def setup_ui_headers(ext_id: str, file_path: str, title: str = 'My Custom Extension', doc_link: str = 'https://docs.isaacsim.omniverse.nvidia.com/latest/index.html', overview: str = '', info_collapsed: bool = True)
- def get_assets_root_path() -> str

## Variables

- MENU_NAME: str
- MENU_CATEGORY: str
- FRANKA_STAGE_PATH: str

# Public API for module isaacsim.ros2.examples.ros_samples:

## Classes

- class Extension(omni.ext.IExt)
  - NOVA_CARTER_NAME: str
  - NOVA_CARTER_JOINT_STATES_NAME: str
  - IW_HUB_NAME: str
  - SAMPLE_SCENE_NAME: str
  - PERCEPTOR_SCENE_NAME: str
  - HOSPITAL_SCENE_NAME: str
  - OFFICE_SCENE_NAME: str
  - ROS2_NAVIGATION_CATEGORY: str
  - ROS2_ISAAC_ROS_CATEGORY: str
  - ROS2_MULTIPLE_ROBOTS_CATEGORY: str
  - def on_startup(self, ext_id: str)
  - def build_ui(self, name: str, file_path: str)
  - def on_shutdown(self)

## Functions

- def get_instance() -> Optional[ExampleBrowserExtension]
- def setup_ui_headers(ext_id: str, file_path: str, title: str = 'My Custom Extension', doc_link: str = 'https://docs.isaacsim.omniverse.nvidia.com/latest/index.html', overview: str = '', info_collapsed: bool = True)
- def get_assets_root_path() -> str

# Public API for module isaacsim.ros2.examples.ros_waypoint_follower_sample:

## Classes

- class ParamWidget
  - FieldDef: Unknown
  - def __init__(self, field_def: FieldDef)
  - def get_value(self) -> Any
  - def destroy(self)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def build_ui(self)
  - def on_shutdown(self)

## Functions

- def get_instance() -> Optional[ExampleBrowserExtension]
- def setup_ui_headers(ext_id: str, file_path: str, title: str = 'My Custom Extension', doc_link: str = 'https://docs.isaacsim.omniverse.nvidia.com/latest/index.html', overview: str = '', info_collapsed: bool = True)

## Variables

- MENU_NAME: str
- MENU_CATEGORY: str
- WAYPOINT_SCRIPT: str
- PATROLLING_SCRIPT: str
- GATHER_WAYPOINTS_SCRIPT: str
