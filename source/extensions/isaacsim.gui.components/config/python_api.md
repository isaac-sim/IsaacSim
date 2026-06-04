# Public API for module isaacsim.gui.components:

## Classes

- class Frame(UIWidgetWrapper)
  - def __init__(self, enabled: bool = True, visible: bool = True, build_fn: Callable = None)
  - [property] def frame(self) -> ui.Frame
  - def rebuild(self)
  - def set_build_fn(self, build_fn: Callable)

- class CollapsableFrame(Frame)
  - def __init__(self, title: str, collapsed: bool = True, enabled: bool = True, visible: bool = True, build_fn: Callable = None)
  - [property] def collapsed(self) -> bool
  - [collapsed.setter] def collapsed(self, value: bool)
  - [property] def title(self) -> str
  - [title.setter] def title(self, value: str)

- class ScrollingFrame(Frame)
  - def __init__(self, num_lines: int = None, enabled: bool = True, visible: bool = True, build_fn: Callable = None)
  - def set_num_lines(self, num_lines: int)

- class IntField(UIWidgetWrapper)
  - def __init__(self, label: str, tooltip: str = '', default_value: int = 0, lower_limit: int = None, upper_limit: int = None, on_value_changed_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def int_field(self) -> ui.IntField
  - def get_value(self) -> int
  - def get_upper_limit(self) -> int
  - def get_lower_limit(self) -> int
  - def set_value(self, val: int)
  - def set_upper_limit(self, upper_limit: int)
  - def set_lower_limit(self, lower_limit: int)
  - def set_on_value_changed_fn(self, on_value_changed_fn: Callable)

- class FloatField(UIWidgetWrapper)
  - def __init__(self, label: str, tooltip: str = '', default_value: float = 0.0, step: float = 0.01, format: str = '%.2f', lower_limit: float = None, upper_limit: float = None, on_value_changed_fn: Callable = None, on_end_edit_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def float_field(self) -> ui.FloatField
  - def get_value(self) -> float
  - def get_upper_limit(self) -> float
  - def get_lower_limit(self) -> float
  - def set_value(self, val: float)
  - def set_upper_limit(self, upper_limit: float)
  - def set_lower_limit(self, lower_limit: float)
  - def set_on_value_changed_fn(self, on_value_changed_fn: Callable)
  - def set_on_end_edit_fn(self, on_end_edit_fn: Callable)

- class StringField(UIWidgetWrapper)
  - def __init__(self, label: str, tooltip: str = '', default_value: str = '', read_only: bool = False, multiline_okay: bool = False, on_value_changed_fn: Callable = None, use_folder_picker: bool = False, item_filter_fn: Callable = None, bookmark_label: str = None, bookmark_path: str = None, folder_dialog_title: str = 'Select Output Folder', folder_button_title: str = 'Select Folder')
  - [property] def label(self) -> ui.Label
  - [property] def string_field(self) -> ui.StringField
  - [property] def file_picker_frame(self) -> ui.Frame
  - [property] def file_picker_btn(self) -> ui.Button
  - def get_value(self) -> str
  - def set_value(self, val: str)
  - def set_on_value_changed_fn(self, on_value_changed_fn: Callable)
  - def set_item_filter_fn(self, item_filter_fn: Callable)
  - def set_read_only(self, read_only: bool)
  - def set_multiline_okay(self, multiline_okay: bool)
  - def add_folder_picker_icon(self, on_click_fn: Callable, item_filter_fn: Callable = None, bookmark_label: str = None, bookmark_path: str = None, dialog_title: str = 'Select Output Folder', button_title: str = 'Select Folder')

- class Button(UIWidgetWrapper)
  - def __init__(self, label: str, text: str, tooltip: str = '', on_click_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def button(self) -> ui.Button
  - def set_on_click_fn(self, on_click_fn: Callable)
  - def trigger_click(self)

- class CheckBox(UIWidgetWrapper)
  - def __init__(self, label: str, default_value: bool = False, tooltip: str = '', on_click_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def checkbox(self) -> ui.CheckBox
  - def get_value(self) -> bool
  - def set_value(self, val: bool)
  - def set_on_click_fn(self, on_click_fn: Callable)

- class StateButton(UIWidgetWrapper)
  - def __init__(self, label: str, a_text: str, b_text: str, tooltip: str = '', on_a_click_fn: Callable = None, on_b_click_fn: Callable = None, physics_callback_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def state_button(self) -> ui.Button
  - def set_physics_callback_fn(self, physics_callback_fn: Callable)
  - def set_on_a_click_fn(self, on_a_click_fn: Callable)
  - def set_on_b_click_fn(self, on_b_click_fn: Callable)
  - def is_in_a_state(self) -> bool
  - def trigger_click_if_a_state(self)
  - def trigger_click_if_b_state(self)
  - def get_current_text(self) -> str
  - def reset(self)
  - def cleanup(self)

- class DropDown(UIWidgetWrapper)
  - def __init__(self, label: str, tooltip: str = '', populate_fn: Callable = None, on_selection_fn: Callable = None, keep_old_selections: bool = False, add_flourish: bool = True)
  - [property] def label(self) -> ui.Label
  - [property] def combobox(self) -> ui.ComboBox
  - def repopulate(self)
  - def set_populate_fn(self, populate_fn: Callable, repopulate: bool = True)
  - def get_items(self) -> list[str]
  - def get_selection_index(self) -> int
  - def get_selection(self) -> str
  - def set_items(self, items: list[str], select_index: int = None)
  - def set_selection(self, selection: str)
  - def set_selection_by_index(self, select_index: int)
  - def set_on_selection_fn(self, on_selection_fn: Callable)
  - def set_keep_old_selection(self, val: bool)
  - def set_populate_fn_to_find_all_usd_objects_of_type(self, object_type: str, repopulate: bool = True)
  - def trigger_on_selection_fn_with_current_selection(self)

- class ColorPicker(UIWidgetWrapper)
  - def __init__(self, label: str, default_value: list[float] = None, tooltip: str = '', on_color_picked_fn: Callable = None)
  - [property] def label(self) -> ui.Label
  - [property] def color_picker(self) -> ui.ColorWidget
  - def get_color(self) -> list[float]
  - def set_color(self, color: list[float])
  - def set_on_color_picked_fn(self, on_color_picked_fn: Callable)

- class TextBlock(UIWidgetWrapper)
  - def __init__(self, label: str, text: str = '', tooltip: str = '', num_lines: int = 5, include_copy_button: bool = True)
  - [property] def label(self) -> ui.Label
  - [property] def scrolling_frame(self) -> ui.ScrollingFrame
  - [property] def copy_btn(self) -> ui.Button
  - [property] def text_block(self) -> ui.Label
  - def get_text(self) -> str
  - def set_text(self, text: str)
  - def set_num_lines(self, num_lines: int)

- class XYPlot(UIWidgetWrapper)
  - def __init__(self, label: str, tooltip: str = '', x_data: list[list] | list = None, y_data: list[list] | list = None, x_min: float = None, x_max: float = None, y_min: float = None, y_max: float = None, x_label: str = 'X', y_label: str = 'Y', plot_height: int = 10, show_legend: bool = False, legends: list[str] = None, plot_colors: list[list[int]] = None)
  - def get_x_data(self) -> list[list[float]]
  - def get_y_data(self) -> list[list[float]]
  - def get_x_min(self) -> float
  - def get_y_min(self) -> float
  - def get_x_max(self) -> float
  - def get_y_max(self) -> float
  - def get_legends(self) -> list[str]
  - def get_plot_height(self) -> int
  - def get_plot_colors(self) -> list[list[int]]
  - def set_plot_color_by_index(self, index: int, r: int, g: int, b: int)
  - def set_plot_colors(self, plot_colors: list[list[int]])
  - def set_x_min(self, x_min: float)
  - def set_y_min(self, y_min: float)
  - def set_x_max(self, x_max: float)
  - def set_y_max(self, y_max: float)
  - def set_legend_by_index(self, idx: int, legend: str)
  - def set_legends(self, legends: list[str])
  - def set_plot_height(self, plot_height: int)
  - def set_show_legend(self, show_legend: bool)
  - def set_data(self, x_data: list[list] | list, y_data: list[list] | list)
  - def set_plot_visible_by_index(self, index: int, visible: bool)

- class ScreenPrinter
  - def __init__(self, screen_pos_x: float = 78, screen_pos_y: float = 95, text_size: float = 14.0, max_width: int = 0, color: np.ndarray | None = None)
  - def set_text(self, text: str)
  - def set_text_position(self, screen_pos_x: float, screen_pos_y: float)
  - def set_text_size(self, size: float)
  - def set_text_max_width(self, max_width: int)
  - def set_text_color(self, color4f: np.ndarray)
  - def clear_text(self)
  - def exit(self)

## Functions

- def setup_ui_headers(ext_id: str, file_path: str, title: str = 'My Custom Extension', doc_link: str = 'https://docs.isaacsim.omniverse.nvidia.com/latest/index.html', overview: str = '', info_collapsed: bool = True)
- def btn_builder(label: str = '', type: str = 'button', text: str = 'button', tooltip: str = '', on_clicked_fn: object = None) -> ui.Button
- def state_btn_builder(label: str = '', type: str = 'state_button', a_text: str = 'STATE A', b_text: str = 'STATE B', tooltip: str = '', on_clicked_fn: object = None) -> ui.Button
- def multi_btn_builder(label: str = '', type: str = 'multi_button', count: int = 2, text: list = None, tooltip: list = None, on_clicked_fn: list = None) -> list[ui.Button]
- def cb_builder(label: str = '', type: str = 'checkbox', default_val: bool = False, tooltip: str = '', on_clicked_fn: object = None) -> ui.SimpleBoolModel
- def multi_cb_builder(label: str = '', type: str = 'multi_checkbox', count: int = 2, text: list = None, default_val: list = None, tooltip: list = None, on_clicked_fn: list = None) -> list[ui.SimpleBoolModel]
- def str_builder(label: str = '', type: str = 'stringfield', default_val: str = ' ', tooltip: str = '', on_clicked_fn: object = None, use_folder_picker: bool = False, read_only: bool = False, item_filter_fn: object = None, bookmark_label: str | None = None, bookmark_path: str | None = None, folder_dialog_title: str = 'Select Output Folder', folder_button_title: str = 'Select Folder', identifier: str | None = None, label_width: int | None = None) -> ui.AbstractValueModel
- def int_builder(label: str = '', type: str = 'intfield', default_val: int = 0, tooltip: str = '', min: int = sys.maxsize * -1, max: int = sys.maxsize) -> ui.AbstractValueModel
- def float_builder(label: str = '', type: str = 'floatfield', default_val: float = 0, tooltip: str = '', min: float = -inf, max: float = inf, step: float = 0.1, format: str = '%.2f') -> object
- def combo_cb_str_builder(label: str = '', type: str = 'checkbox_stringfield', default_val: list = None, tooltip: str = '', on_clicked_fn: object = lambda x: None, use_folder_picker: bool = False, read_only: bool = False, folder_dialog_title: str = 'Select Output Folder', folder_button_title: str = 'Select Folder') -> object
- def dropdown_builder(label: str = '', type: str = 'dropdown', default_val: int = 0, items: list[str] | None = None, tooltip: str = '', on_clicked_fn: Callable | None = None, identifier: str | None = None, show_flourish: bool = True, label_width: int | None = None) -> ui.AbstractItemModel
- def multi_dropdown_builder(label: str = '', type: str = 'multi_dropdown', count: int = 2, default_val: list = None, items: list = None, tooltip: str = '', on_clicked_fn: list = None) -> object
- def combo_cb_dropdown_builder(label: str = '', type: str = 'checkbox_dropdown', default_val: list = None, items: list = None, tooltip: str = '', on_clicked_fn: list = None) -> object
- def combo_intfield_slider_builder(label: str = '', type: str = 'intfield_stringfield', default_val: float = 0.5, min: int = 0, max: int = 1, step: float = 0.01, tooltip: list = None) -> object
- def combo_floatfield_slider_builder(label: str = '', type: str = 'floatfield_stringfield', default_val: float = 0.5, min: int = 0, max: int = 1, step: float = 0.01, tooltip: list = None) -> object
- def scrolling_frame_builder(label: str = '', type: str = 'scrolling_frame', default_val: str = 'No Data', tooltip: str = '') -> object
- def combo_cb_scrolling_frame_builder(label: str = '', type: str = 'cb_scrolling_frame', default_val: list = None, tooltip: str = '', on_clicked_fn: object = lambda x: None) -> object
- def xyz_builder(label: str = '', tooltip: str = '', axis_count: int = 3, default_val: list[float] = None, min: float = float('-inf'), max: float = float('inf'), step: float = 0.001, on_value_changed_fn: list = None) -> list
- def color_picker_builder(label: str = '', type: str = 'color_picker', default_val: list = None, tooltip: str = 'Color Picker') -> object
- def progress_bar_builder(label: str = '', type: str = 'progress_bar', default_val: float = 0, tooltip: str = 'Progress') -> object
- def plot_builder(label: str = '', data: object = None, min: float = -1, max: float = 1, type: object = ui.Type.LINE, value_stride: int = 1, color: object = None, tooltip: str = '') -> object
- def combo_cb_plot_builder(label: str = '', default_val: bool = False, on_clicked_fn: object = lambda x: None, data: object = None, min: int = -1, max: int = 1, type: object = ui.Type.LINE, value_stride: int = 1, color: object = None, tooltip: str = '') -> object
- def xyz_plot_builder(label: str = '', data: list = None, min: int = -1, max: int = 1, tooltip: str = '') -> list[ui.Plot]
- def combo_cb_xyz_plot_builder(label: str = '', default_val: bool = False, on_clicked_fn: object = lambda x: None, data: list = None, min: int = -1, max: int = 1, type: object = ui.Type.LINE, value_stride: int = 1, tooltip: str = '') -> object
- def add_separator()
