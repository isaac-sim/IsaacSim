# Public API for module isaacsim.test.utils:

## Classes

- class MenuUITestCase(OmniUiTest)
  - async def setUp(self)
  - async def tearDown(self)
  - async def wait_for_stage_loading(self)
  - async def new_stage(self)
  - async def wait_n_frames(self, n: int = 10)
  - async def get_viewport_context_menu(self, max_attempts: int = 5) -> dict[str, Any]
  - async def menu_click_with_retry(self, menu_path: str, delays: list[int] = None, window_name: str = None, wait_n_frames: int = 10) -> Any
  - async def find_widget_with_retry(self, query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None) -> Any
  - async def find_enabled_widget_with_retry(self, query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None) -> Any
  - async def scroll_to_widget(self, widget_ref: Any, settle_frames: int = 3) -> bool
  - async def wait_for_widget_enabled(self, widget: Any, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool
  - def count_prims_by_type(self, prim_type: str) -> int
  - async def run_timeline_frames(self, n: int = 50)
  - def select_prim(self, prim_path: str)

- class TimedAsyncTestCase(omni.kit.test.AsyncTestCase)
  - async def setUp(self)
  - async def tearDown(self)

## Functions

- def get_widget_screen_center(widget: object) -> tuple[float, float]
- def deferred_click(x: float, y: float)
- def deferred_click_widget(widget: object) -> tuple[float, float]
- def discover_template_buttons(template: object) -> dict[str, object]
- def validate_folder_contents(path: str | Path, expected_counts: dict[str, int]) -> bool
- def get_folder_file_summary(path: str | Path) -> dict[str, int | dict[str, list]]
- def validate_file_list(file_paths: list[str | Path]) -> dict[str, bool | list[str]]
- async def capture_annotator_data_async(annotator_name: str, camera_position: tuple[float, float, float] = (5, 5, 5), camera_look_at: tuple[float, float, float] = (0, 0, 0), resolution: tuple[int, int] = (1280, 720), camera_prim_path: str | None = None, render_product: Any = None, do_array_copy: bool = True) -> Any
- async def capture_rgb_data_async(camera_position: tuple[float, float, float] = (5, 5, 5), camera_look_at: tuple[float, float, float] = (0, 0, 0), resolution: tuple[int, int] = (1280, 720), camera_prim_path: str | None = None, render_product: Any = None) -> np.ndarray
- async def capture_depth_data_async(depth_type: str = 'distance_to_camera', camera_position: tuple[float, float, float] = (5, 5, 5), camera_look_at: tuple[float, float, float] = (0, 0, 0), resolution: tuple[int, int] = (1280, 720), camera_prim_path: str | None = None, render_product: Any = None) -> np.ndarray
- async def capture_viewport_annotator_data_async(viewport_api: Any, annotator_name: str = 'rgb') -> Any
- async def capture_app_screenshot_async(output_path: str) -> bool
- async def capture_viewport_screenshot_async(output_path: str) -> bool
- async def capture_frame_sequence_async(output_dir: str, num_frames: int = 30, updates_per_frame: int = 2, mode: str = 'app') -> list[str]
- def compute_difference_metrics(golden_array: np.ndarray, test_array: np.ndarray) -> dict[str, object]
- def print_difference_statistics(metrics: dict[str, object])
- def compare_arrays_within_tolerances(golden_array: np.ndarray, test_array: np.ndarray, allclose_rtol: float | None = 1e-05, allclose_atol: float | None = 1e-08, mean_tolerance: float | None = None, max_tolerance: float | None = None, absolute_tolerance: float | None = None, percentile_tolerance: tuple | None = None, rmse_tolerance: float | None = None, print_all_stats: bool = False) -> dict[str, object]
- def compare_images_within_tolerances(golden_file_path: str, test_file_path: str, allclose_rtol: float | None = 1e-05, allclose_atol: float | None = 1e-08, mean_tolerance: float | None = None, max_tolerance: float | None = None, absolute_tolerance: float | None = None, percentile_tolerance: tuple | None = None, rmse_tolerance: float | None = None, print_all_stats: bool = False) -> dict[str, object]
- def compare_images_in_directories(golden_dir: str, test_dir: str, path_pattern: str | None = None, allclose_rtol: float | None = 1e-05, allclose_atol: float | None = 1e-08, mean_tolerance: float | None = None, max_tolerance: float | None = None, absolute_tolerance: float | None = None, percentile_tolerance: tuple | None = None, rmse_tolerance: float | None = None, print_all_stats: bool = False, print_per_file_results: bool = True) -> dict[str, object]
- def save_rgb_image(rgb_data: np.ndarray, out_dir: str, file_name: str)
- def save_depth_image(depth_data: np.ndarray, out_dir: str, file_name: str, normalize: bool = False)
- def save_annotator_data(data: Any, output_path: str)
- def read_image_as_array(file_path: str, squeeze_singleton_channel: bool = True) -> np.ndarray
- def close_windows(titles: list[str]) -> list[str]
- def ensure_dock_height(window_title: str, min_height: int = 400) -> bool
- async def ensure_dock_height_async(window_title: str, min_height: int = 400) -> bool
- def ensure_window_visible(window_title: str, focus: bool = True) -> bool
- async def ensure_window_visible_async(window_title: str, focus: bool = True) -> bool
- def reset_to_default_layout(close_extra_windows: list[str] | None = None, focus_window: str = 'Content')
- async def reset_to_default_layout_async(close_extra_windows: list[str] | None = None, focus_window: str = 'Content')
- async def find_widget_with_retry(query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None) -> Any
- async def find_enabled_widget_with_retry(query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None) -> Any
- async def wait_for_widget_enabled(widget: Any, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool
- async def scroll_to_widget(widget_ref: Any, settle_frames: int = 3) -> bool
- async def menu_click_with_retry(menu_path: str, delays: list[int] = None, window_name: str = None, wait_n_frames: int = 10) -> Any
- def list_menu_paths(max_depth: int = 3) -> list[str]
- async def perform_widget_action(query: str, action: str = 'click', text: str = '', max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> dict
- def get_all_menu_paths(menu_dict: dict, current_path: str = '', root_path: str = '') -> list[str]
- def count_menu_items(menu_dict: dict) -> int
- async def navigate_menu_visual(menu_path: str) -> bool
- def poll_until(check_fn: Callable[[], object], timeout_frames: int = 1800, poll_steps: int = 30, label: str = '') -> int | None
- async def poll_until_async(check_fn: Callable[[], object], timeout_frames: int = 1800, poll_steps: int = 30, label: str = '') -> int | None
- def wait_for_prim(prim_path: str, timeout_frames: int = 1800, poll_steps: int = 30) -> bool
- async def wait_for_prim_async(prim_path: str, timeout_frames: int = 1800, poll_steps: int = 30) -> bool
- def wait_for_stage_prims(min_prims: int = 20, timeout_frames: int = 1800, poll_steps: int = 30) -> bool
- async def wait_for_stage_prims_async(min_prims: int = 20, timeout_frames: int = 1800, poll_steps: int = 30) -> bool
- def project_world_to_screen(position: tuple[float, float, float], viewport: object | None = None) -> tuple[float, float]
- async def compare_usd_files(paths: list[str]) -> bool
