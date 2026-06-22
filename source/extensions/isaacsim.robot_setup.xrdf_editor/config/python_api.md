# Public API for module isaacsim.robot_setup.xrdf_editor:

## Classes

- class CollisionSphereEditor
  - def __init__(self)
  - def clear_spheres(self, store_op: bool = True)
  - def clear_link_spheres(self, link_path: str, store_op: bool = True)
  - def delete_sphere(self, sphere_path: str)
  - def set_sphere_colors(self, filter: str, color_in: np.ndarray | None = None, color_out: np.ndarray | None = None)
  - def set_sphere_color(self, sphere_path: str, ensure_visual_material: bool = True)
  - def copy_all_sphere_data(self)
  - def undo(self)
  - def redo(self)
  - def generate_spheres(self, link_path: str, points: np.ndarray, face_inds: np.ndarray, vert_cts: np.ndarray, num_spheres: int, radius_offset: float, is_preview: bool)
  - def clear_preview(self)
  - def add_sphere(self, link_path: str, center: object, radius: float, store_op: bool = True) -> str
  - def load_xrdf_spheres(self, robot_prim_path: str, parsed_file: dict[str, Any])
  - def load_spheres(self, robot_prim_path: str, robot_description_file_path: str)
  - def interpolate_spheres(self, path1: str, path2: str, num_spheres: int)
  - def scale_spheres(self, path: str, factor: float)
  - def get_sphere_names_by_link(self, link_path: str) -> list[str]
  - def write_spheres_to_dict(self, robot_prim_path: str, link_to_spheres: dict[str, Any])
  - def save_spheres(self, robot_prim_path: str, f: object)
  - def on_shutdown(self)

- class EditorState
  - def __init__(self)
  - def add_articulation_changed_callback(self, callback: Callable[[], None])
  - def select_articulation(self, prim_path: str | None)
  - def refresh_dof_properties(self, refresh_dof_state: bool = False)
  - def refresh_link_meshes(self)
  - def link_path(self, link_name: str) -> str
  - def ignore_dict(self) -> dict[str, list[str]]
  - def articulation_frames(self) -> set[str]
  - def export_xrdf(self, path: str)
  - def import_xrdf(self, path: str)
  - def export_lula(self, path: str)
  - def import_lula(self, path: str)
  - def on_shutdown(self)

- class UIBuilder
  - def __init__(self, ext_id: str, source_file: str)
  - def on_menu_callback(self)
  - def on_timeline_event(self, event: object)
  - def on_physics_step(self, step: float)
  - def on_selection_changed(self, event: object)
  - def on_stage_opened(self, event: object)
  - def on_stage_closed(self, event: object)
  - def on_simulation_start_play(self, event: object)
  - def on_simulation_stop_play(self, event: object)
  - def cleanup(self)
  - def build_ui(self)

## Functions

- def is_yaml_file(path: str) -> bool
- def is_xrdf_file(path: str) -> bool
- def on_filter_xrdf_item(item: object) -> bool
- def on_filter_item(item: object) -> bool

# Public API for module isaacsim.robot_setup.xrdf_editor.articulation_discovery:

## Functions

- def find_all_articulation_base_paths(stage: Usd.Stage | None) -> list[str]
- def find_mimic_joint_names(stage: Usd.Stage | None, articulation_base_path: str | None) -> set[str]
- def get_ignore_dict(articulation_base_path: str, ordered_links: list[str]) -> dict[str, list[str]]
