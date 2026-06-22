# Public API for module isaacsim.util.camera_inspector:

## Classes

- class Camera(XformPrim)
  - def __init__(self, paths: str | list[str])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_focal_lengths(self, focal_lengths: float | list | np.ndarray | wp.array)
  - def get_focal_lengths(self) -> wp.array
  - def set_focus_distances(self, focus_distances: float | list | np.ndarray | wp.array)
  - def get_focus_distances(self) -> wp.array
  - def set_stereo_roles(self, roles: Literal['mono', 'left', 'right'] | list[Literal['mono', 'left', 'right']])
  - def get_stereo_roles(self) -> list[Literal[mono, left, right]]
  - def set_fstops(self, fstops: float | list | np.ndarray | wp.array)
  - def get_fstops(self) -> wp.array
  - def set_apertures(self, horizontal_apertures: float | list | np.ndarray | wp.array = None, vertical_apertures: float | list | np.ndarray | wp.array = None)
  - def get_apertures(self) -> tuple[wp.array, wp.array]
  - def set_aperture_offsets(self, horizontal_offsets: float | list | np.ndarray | wp.array = None, vertical_offsets: float | list | np.ndarray | wp.array = None)
  - def get_aperture_offsets(self) -> tuple[wp.array, wp.array]
  - def set_projections(self, projections: Literal['perspective', 'orthographic'] | list[Literal['perspective', 'orthographic']])
  - def get_projections(self) -> list[Literal[perspective, orthographic]]
  - def set_clipping_ranges(self, near_distances: float | list | np.ndarray | wp.array = None, far_distances: float | list | np.ndarray | wp.array = None)
  - def get_clipping_ranges(self) -> tuple[wp.array, wp.array]
  - def set_shutter_times(self, open_times: float | list | np.ndarray | wp.array = None, close_times: float | list | np.ndarray | wp.array = None)
  - def get_shutter_times(self) -> tuple[wp.array, wp.array]
  - def enforce_square_pixels(self, resolutions: list | np.ndarray | wp.array)

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

- class TextBlock(UIWidgetWrapper)
  - def __init__(self, label: str, text: str = '', tooltip: str = '', num_lines: int = 5, include_copy_button: bool = True)
  - [property] def label(self) -> ui.Label
  - [property] def scrolling_frame(self) -> ui.ScrollingFrame
  - [property] def copy_btn(self) -> ui.Button
  - [property] def text_block(self) -> ui.Label
  - def get_text(self) -> str
  - def set_text(self, text: str)
  - def set_num_lines(self, num_lines: int)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

## Functions

- def make_menu_item_description(ext_id: str, name: str, onclick_fun: object, action_name: str = '')
- def add_line_rect_flourish(draw_line: bool = True)
- def btn_builder(label: str = '', type: str = 'button', text: str = 'button', tooltip: str = '', on_clicked_fn: object = None) -> ui.Button
- def get_style() -> dict[str, Any]
- def setup_ui_headers(ext_id: str, file_path: str, title: str = 'My Custom Extension', doc_link: str = 'https://docs.isaacsim.omniverse.nvidia.com/latest/index.html', overview: str = '', info_collapsed: bool = True)

## Variables

- COLOR_W: int
- COLOR_X: int
- COLOR_Y: int
- COLOR_Z: int
- BUTTON_WIDTH: int
- EXTENSION_NAME: Final[str]
- SUPPORTED_AXES: Final[list[str]]
