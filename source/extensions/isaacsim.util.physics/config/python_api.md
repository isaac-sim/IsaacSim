# Public API for module isaacsim.util.physics:

## Classes

- class ScrollingWindow(ui.Window)
  - def __init__(self, **kwargs: object)
  - [property] def frame(self) -> ui.ScrollingFrame

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def apply_collision_on_selected(self)
  - def clear_collision_on_selected(self)
  - def remove_physics_apis_on_selected(self)
  - def traverse_prims(self, selection: list, include_xform: bool = False, ignore_rigid: bool = True, visible_only: bool = True) -> list
  - def prim_is_valid(self, prim: Usd.Prim, include_xform: bool = False, visible_only: bool = True) -> bool
  - def apply_collision_to_prim(self, prim: Usd.Prim, approximationShape: str = 'none')
  - def unapply_collision_on_prim(self, prim: Usd.Prim)
  - def remove_physics_apis_on_prim(self, prim: Usd.Prim)
  - def on_shutdown(self)

## Functions

- def make_menu_item_description(ext_id: str, name: str, onclick_fun: object, action_name: str = '')
- def btn_builder(label: str = '', type: str = 'button', text: str = 'button', tooltip: str = '', on_clicked_fn: object = None) -> ui.Button
- def cb_builder(label: str = '', type: str = 'checkbox', default_val: bool = False, tooltip: str = '', on_clicked_fn: object = None) -> ui.SimpleBoolModel
- def dropdown_builder(label: str = '', type: str = 'dropdown', default_val: int = 0, items: list[str] | None = None, tooltip: str = '', on_clicked_fn: Callable | None = None, identifier: str | None = None, show_flourish: bool = True, label_width: int | None = None) -> ui.AbstractItemModel
- def multi_btn_builder(label: str = '', type: str = 'multi_button', count: int = 2, text: list = None, tooltip: list = None, on_clicked_fn: list = None) -> list[ui.Button]
- def progress_bar_builder(label: str = '', type: str = 'progress_bar', default_val: float = 0, tooltip: str = 'Progress') -> object

## Variables

- EXTENSION_NAME: str
