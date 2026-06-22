# Public API for module isaacsim.util.merge_mesh:

## Classes

- class MeshMerger(object)
  - def __init__(self, stage: object)
  - [property] def total_meshes(self) -> int
  - [property] def total_subsets(self) -> int
  - [property] def total_materials(self) -> int
  - [property] def meshes_to_merge(self) -> list
  - [property] def clear_parent_xform(self) -> bool
  - [clear_parent_xform.setter] def clear_parent_xform(self, value: bool)
  - [property] def deactivate_source(self) -> bool
  - [deactivate_source.setter] def deactivate_source(self, value: bool)
  - [property] def selected_objects(self) -> list
  - [property] def combine_materials(self) -> bool
  - [combine_materials.setter] def combine_materials(self, value: bool)
  - [property] def materials_destination(self) -> str
  - [materials_destination.setter] def materials_destination(self, value: str)
  - [property] def output_mesh(self) -> str
  - [output_mesh.setter] def output_mesh(self, value: str)
  - def fix_material_sources(self, mat: object)
  - def update_selection(self, selection: list, stage: object = None)
  - def merge_meshes(self)
  - def reactivate_sources(self)
  - def remove_created_materials(self)

- class MergeMeshesCommand(omni.kit.commands.Command)
  - def __init__(self, source: str, clear_transform: bool = False, deactivate_source: bool = False, combine_materials: bool = False, materials_destination: str = '/World/Looks')
  - def do(self) -> str
  - def undo(self)

- class ScrollingWindow(ui.Window)
  - def __init__(self, **kwargs: object)
  - [property] def frame(self) -> ui.ScrollingFrame

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def build_ui(self)
  - def on_mat_changed(self, value: object)
  - def on_mat_dest_changed(self, value: str)
  - def on_shutdown(self)

## Functions

- def make_menu_item_description(ext_id: str, name: str, onclick_fun: object, action_name: str = '')
- def get_style() -> dict[str, Any]
- def btn_builder(label: str = '', type: str = 'button', text: str = 'button', tooltip: str = '', on_clicked_fn: object = None) -> ui.Button
- def cb_builder(label: str = '', type: str = 'checkbox', default_val: bool = False, tooltip: str = '', on_clicked_fn: object = None) -> ui.SimpleBoolModel
- def combo_cb_str_builder(label: str = '', type: str = 'checkbox_stringfield', default_val: list = None, tooltip: str = '', on_clicked_fn: object = lambda x: None, use_folder_picker: bool = False, read_only: bool = False, folder_dialog_title: str = 'Select Output Folder', folder_button_title: str = 'Select Folder') -> object
- def str_builder(label: str = '', type: str = 'stringfield', default_val: str = ' ', tooltip: str = '', on_clicked_fn: object = None, use_folder_picker: bool = False, read_only: bool = False, item_filter_fn: object = None, bookmark_label: str | None = None, bookmark_path: str | None = None, folder_dialog_title: str = 'Select Output Folder', folder_button_title: str = 'Select Folder', identifier: str | None = None, label_width: int | None = None) -> ui.AbstractValueModel

## Variables

- EXTENSION_NAME: str
