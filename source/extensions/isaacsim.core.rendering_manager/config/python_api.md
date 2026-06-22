# Public API for module isaacsim.core.rendering_manager:

## Classes

- class RenderingManager
  - class def render(cls)
  - class async def render_async(cls)
  - class def set_dt(cls, dt: float)
  - class def get_dt(cls) -> float
  - class def register_callback(cls, event: RenderingEvent) -> int
  - class def deregister_callback(cls, uid: int)
  - class def deregister_all_callbacks(cls)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class RenderingEvent(Enum)
  - NEW_FRAME: str

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
