# Changelog

## [0.14.6] - 2026-05-19
### Added
- Add `scroll_to_widget` helper (and `MenuUITestCase.scroll_to_widget`) that locates the enclosing `ui.ScrollingFrame` for a widget and scrolls it into view, used to click widgets pushed off-screen by dynamic sibling content.


## [0.14.5] - 2026-05-18
### Changed
- `compare_images_in_directories` prints the golden and test directory paths in its header and the full file paths on each failure to make per-image failures unambiguous.

## [0.14.4] - 2026-05-12
### Changed
- `compare_usd_files` articulation check will check if the asset contains newton articulation root api or physx articulation root api and test the attributes accordingly

## [0.14.3] - 2026-04-28
### Fixed
- `menu_click_with_retry` waits for the requested menu path to resolve before clicking, absorbing the async menubar rebuild that ran after a prior test's window destruction (fixes flaky `ui.Menu item failed to become show`).

## [0.14.2] - 2026-04-13
### Added
- Add `omni.kit.viewport.window` as a dependency

## [0.14.1] - 2026-04-08
### Added
- Add `_apply_novaluetype_numeric_patch` coverage workaround giving `numpy._globals._NoValueType` `__int__`, `__float__`, and `__index__` methods so `int()`/`float()` calls inside osqp/scipy do not crash under coverage

## [0.14.0] - 2026-03-31
### Added
- Add `usd_utils` module with `compare_usd_files` for loading and comparing articulation properties across USD files
- Add `compare_articulation_properties` for pairwise comparison of articulation member values
- Add `isaacsim.core.experimental.prims` as a dependency

## [0.13.0] - 2026-03-27
### Added
- Add `button_utils` module with `get_widget_screen_center`, `deferred_click`, `deferred_click_widget`, and `discover_template_buttons` for UI button discovery and deferred click automation
- Add `viewport_utils` module with `project_world_to_screen` for world-to-screen coordinate projection
- Add `capture_frame_sequence_async` to `image_capture` for capturing multi-frame sequences in app, viewport, or replicator modes
- Add `navigate_menu_visual` to `menu_utils` for visual menu navigation via mouse emulation
- Add `_poll_async` helper to `menu_utils` to consolidate frame-polling patterns
- Add `layout_utils` module with `ensure_dock_height`, `ensure_window_visible`, `close_windows`, and `reset_to_default_layout` for layout and dock management
- Add `stage_utils` module with `poll_until`, `wait_for_prim`, and `wait_for_stage_prims` for stage polling during UI automation

### Changed
- Add type annotations throughout all modules

## [0.12.0] - 2026-03-25
### Added
- Add `capture_app_screenshot_async` to `image_capture` for full-application swapchain screenshots (headless and windowed)
- Add `capture_viewport_screenshot_async` to `image_capture` for viewport-only screenshots via replicator annotators
- Add `save_annotator_data` to `image_io` to dispatch and save any annotator output (array, dict, or PNG) to disk
- Add `list_menu_paths` to `menu_utils` to enumerate all live menubar paths up to a configurable depth
- Add `perform_widget_action` to `menu_utils` combining widget find-with-retry and action dispatch in a single call

## [0.11.1] - 2026-03-07
### Fixed
- Add `omni.kit.material.library` as a dependency

## [0.11.0] - 2026-03-06
### Added
- Move `find_widget_with_retry` from `MenuUITestCase` to `menu_utils` as a standalone function
- Add `find_enabled_widget_with_retry` to poll for a widget that is both found and enabled
- Add `wait_for_widget_enabled` to poll until an already-found widget becomes enabled

## [0.10.1] - 2026-03-04
### Changed
- Fix API errors

## [0.10.0] - 2026-03-04
### Changed
- Added Overview.md and python_api.md and updated docstrings

## [0.9.1] - 2026-02-22
### Changed
- Add `omni.kit.material.library.get_mdl_list_async` and `omni.kit.menu.utils.rebuild_menus` to `MenuUITestCase.wait_for_stage_loading` to fix menu rebuild issues

## [0.9.0] - 2026-02-21
### Changed
- Add `find_widget_with_retry` to `MenuUITestCase` to find a widget with retry

## [0.8.2] - 2026-02-16
### Changed
- Replace `omni.kit.ui_test.menu_click` with custom step-by-step menu navigation that polls for each submenu to become findable and visible before proceeding, avoiding the `carb.log_error` and `AttributeError` that `menu_click` produces when submenus are slow to appear
- Add `carb.log_info` diagnostics throughout `menu_click_with_retry` for log debugging

## [0.8.1] - 2026-02-13
### Changed
- Suppress transient error logs from `omni.kit.ui_test.query` during intermediate retries in `menu_click_with_retry`; errors are only surfaced on the final retry attempt

## [0.8.0] - 2026-02-09
### Changed
- Add pycoverage patch for numpy `_CopyMode.__bool__` to prevent `ValueError` when scipy imports trigger `_CopyMode.IF_NEEDED` evaluation under coverage

## [0.7.3] - 2026-02-02
### Changed
- Fix pycoverage compatibility issue with numpy sum and prod functions

## [0.7.2] - 2026-02-02
### Changed
- Add a pycoverage compatible amin and amax implementation that is monkeypatched into numpy on extension startup. Removed from image_comparison.py as it is no longer needed.
- This is only used if --/exts/omni.kit.test/pyCoverageEnabled=1 is set

## [0.7.1] - 2026-01-24
### Changed
- Refactor menu_click_with_retry into a separate function
- Add new_stage to MenuUITestCase

## [0.7.0] - 2025-12-22
### Added
- Add new utility functions (`get_all_menu_paths`, `count_menu_items`) and `MenuUITestCase` base class for menu UI tests.

## [0.6.4] - 2025-12-07
### Changed
- Update description

## [0.6.3] - 2025-12-02
### Added
- Specify --/app/settings/fabricDefaultStageFrameHistoryCount=3 for startup test

### Removed
- Remove omni.replicator.core as an explicit test dependency

## [0.6.2] - 2025-11-20
### Changed
- Add omni.replicator.core as an explicit dependency for image capture utils

## [0.6.1] - 2025-11-10
### Changed
- Fix invalid escape sequence

## [0.6.0] - 2025-11-07
### Added
- Added `compare_images_in_directories()` function to compare images in two directories

## [0.5.1] - 2025-09-26
### Changed
- Update license headers

## [0.5.0] - 2025-09-22
### Changed
- Can now optionally exclude blank pixels from image_comparison.compute_difference_metrics.

## [0.4.0] - 2025-09-19
### Added
- Added `image_io.py` module with image I/O utilities:
  - `save_rgb_image()` function for saving RGB/RGBA image data to disk
  - `save_depth_image()` function for saving depth data as TIFF (float32) or grayscale visualization
  - `read_image_as_array()` function for reading images as numpy arrays compatible with comparison utilities
- Added `capture_viewport_annotator_data_async()` function to capture annotator data from existing viewports

### Changed
- Moved image saving functions (`save_rgb_image`, `save_depth_image`) from `image_capture.py` to new `image_io.py` module
- Added optional `render_product` parameter to `capture_annotator_data_async()`, `capture_rgb_data_async()`, `capture_depth_data_async()

## [0.3.0] - 2025-09-15
### Changed
- Added file_validation.py for utilities to validate folder contents and file lists

## [0.2.1] - 2025-09-08
### Added
- Base test class stores the asset root path using fast path

## [0.2.0] - 2025-08-28
### Added
- Added image comparison utilities
- Added image capture utilities
- Added test utilities

## [0.1.0] - 2025-08-11
### Added
- Initial release
