# Public API for module isaacsim.ros2.tf_viewer:

## Classes

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.ros2.tf_viewer.impl.ui_builder:

## Classes

- class ViewportScene
  - def __init__(self, viewport_window: ui.Window, ext_id: str)
  - def destroy(self)

- class UIBuilder
  - def __init__(self, menu_path: str, window_title: str, viewport_scene: ViewportScene, on_visibility_changed_callback: Callable[[bool], None], on_reset_callback: Callable[[], None])
  - [property] def root_frame(self) -> str
  - [property] def update_frequency(self) -> int
  - def show_window(self, value: bool)
  - def update(self, frames: set[str])
  - def shutdown(self)

# Public API for module isaacsim.ros2.tf_viewer.impl.viewport_scene:

## Classes

- class ViewManipulator(sc.Manipulator)
  - def __init__(self, **kwargs: object)
  - def update_transforms(self, transforms: dict, relations: list)
  - def set_root_frame(self, value: str)
  - def set_frames_show(self, value: bool)
  - def set_frames_color(self, channel: int, value: float)
  - def set_frames_size(self, value: float)
  - def set_names_show(self, value: bool)
  - def set_names_color(self, channel: int, value: float)
  - def set_names_size(self, value: float)
  - def set_axes_show(self, value: bool)
  - def set_axes_length(self, value: float)
  - def set_axes_thickness(self, value: float)
  - def set_arrows_show(self, value: bool)
  - def set_arrows_color(self, channel: int, value: float)
  - def set_arrows_thickness(self, value: float)
  - def clear(self)
  - def on_build(self)

- class ViewportScene
  - def __init__(self, viewport_window: ui.Window, ext_id: str)
  - def destroy(self)
