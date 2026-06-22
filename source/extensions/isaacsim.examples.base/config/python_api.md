# Public API for module isaacsim.examples.base:

## Classes

- class BaseSample(object)
  - def __init__(self)
  - def set_world_settings(self, physics_dt: float | None = None, stage_units_in_meters: float | None = None, rendering_dt: float | None = None, device: str | None = None)
  - async def load_world_async(self)
  - async def reset_async(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def log_info(self, info: str)
  - def physics_cleanup(self)
  - async def clear_async(self)

- class BaseSampleUITemplate
  - def __init__(self, *args: object, **kwargs: object)
  - [property] def sample(self) -> BaseSample
  - [sample.setter] def sample(self, sample: BaseSample)
  - def build_ui(self)
  - def build_default_frame(self)
  - def get_extra_frames_handle(self) -> object
  - def build_extra_frames(self)
  - def post_reset_button_event(self)
  - def post_load_button_event(self)
  - def post_clear_button_event(self)
  - def on_shutdown(self)
  - def on_stage_event(self, event: carb.eventdispatcher.Event)
