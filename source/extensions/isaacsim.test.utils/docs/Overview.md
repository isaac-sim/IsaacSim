# Overview

The isaacsim.test.utils extension provides a comprehensive testing framework for Isaac Sim, featuring utilities for image capture, comparison, validation, and UI testing. This extension streamlines automated test workflows by offering specialized tools for capturing synthetic data, validating file systems, and comparing images with configurable tolerances.

## Functionality

### Image Capture

**Synthetic data capture capabilities** enable tests to capture RGB and depth images from virtual cameras in the simulation. The {func}`capture_rgb_data_async <isaacsim.test.utils.capture_rgb_data_async>` and {func}`capture_depth_data_async <isaacsim.test.utils.capture_depth_data_async>` functions support flexible camera positioning, existing camera prims, or render products.

The extension supports various depth measurement types including radial distance (`distance_to_camera`) and Z-depth (`distance_to_image_plane`). For advanced use cases, {func}`capture_annotator_data_async <isaacsim.test.utils.capture_annotator_data_async>` provides access to any replicator annotator including semantic segmentation, normals, and camera parameters.

High-level screenshot helpers simplify the most common capture workflows. {func}`capture_viewport_screenshot_async <isaacsim.test.utils.capture_viewport_screenshot_async>` captures the rendered 3D viewport directly to a PNG file via replicator annotators and works in headless mode. {func}`capture_app_screenshot_async <isaacsim.test.utils.capture_app_screenshot_async>` captures the full application window including UI chrome using the swapchain capture interface, working in both windowed and headless modes.

### Image Comparison

**Tolerance-based image comparison** forms the core validation capability. The {func}`compare_arrays_within_tolerances <isaacsim.test.utils.compare_arrays_within_tolerances>` function evaluates images against multiple criteria including mean absolute difference, max tolerance, percentile-based thresholds, and RMSE metrics.

Directory-wide comparison through {func}`compare_images_in_directories <isaacsim.test.utils.compare_images_in_directories>` enables batch validation of test outputs, automatically matching files between golden and test directories while reporting missing or extra files.

### File System Validation

**File validation utilities** verify test output completeness and quality. The {func}`validate_folder_contents <isaacsim.test.utils.validate_folder_contents>` function checks directories against expected file extension counts, supporting recursive traversal and minimum file size requirements.

File list validation through {func}`validate_file_list <isaacsim.test.utils.validate_file_list>` ensures test artifacts exist and meet size criteria, while {func}`get_folder_file_summary <isaacsim.test.utils.get_folder_file_summary>` provides detailed breakdowns of directory contents by extension.

### UI Testing Framework

**Menu testing capabilities** address the timing challenges of UI automation. The {class}`MenuUITestCase <isaacsim.test.utils.MenuUITestCase>` provides base functionality for menu-driven tests, while {func}`menu_click_with_retry <isaacsim.test.utils.menu_click_with_retry>` handles unreliable menu interactions through intelligent retry mechanisms with exponential backoff.

{func}`list_menu_paths <isaacsim.test.utils.list_menu_paths>` discovers all clickable paths from the live menubar up to a configurable depth without opening any menus, returning slash-separated paths ready for use with `menu_click_with_retry`. {func}`perform_widget_action <isaacsim.test.utils.perform_widget_action>` combines widget discovery with action dispatch — supporting click, double-click, right-click, text input, and property read — in a single polling call.

The {class}`TimedAsyncTestCase <isaacsim.test.utils.TimedAsyncTestCase>` automatically measures test execution duration, providing performance insights for optimization.

## Key Components

### Image Metrics

The {func}`compute_difference_metrics <isaacsim.test.utils.compute_difference_metrics>` function calculates comprehensive comparison statistics including `np.allclose` validation, mean/max absolute differences, RMSE, and configurable percentiles. These metrics support both strict equality checks and tolerance-based validation suitable for rendering variations.

### Data Persistence

Image I/O functions preserve test data in appropriate formats. The {func}`save_depth_image <isaacsim.test.utils.save_depth_image>` function automatically selects between lossless float32 TIFF for metric data and 8-bit grayscale for visualization, while {func}`save_rgb_image <isaacsim.test.utils.save_rgb_image>` handles RGB/RGBA data with format-appropriate transparency support. The {func}`save_annotator_data <isaacsim.test.utils.save_annotator_data>` function accepts any annotator output — numpy arrays, dicts, or image data — and automatically selects the correct format (`.npy`, `.png`, or `.json`) based on the data type and output path extension.

## Dependencies

The extension uses **omni.replicator.core** for synthetic data generation through annotator systems, enabling capture of RGB, depth, and semantic data from virtual cameras. Integration with **omni.kit.ui_test** provides the underlying menu interaction capabilities that the retry mechanisms enhance with improved reliability.
