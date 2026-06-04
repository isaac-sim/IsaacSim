# Isaac Sim UI Automation API Reference

## TCP Python Server Protocol

- **Extension**: `isaacsim.code_editor.python_server` (enable with `--enable isaacsim.code_editor.python_server`)
- **Default**: host `127.0.0.1`, port `8226`
- **Protocol**: Connect via TCP, send UTF-8 Python source, send EOF (half-close), read JSON response
- **Async support**: Code containing `await` is detected and awaited automatically
- **Execution scope**: Code runs in Isaac Sim's main Python environment with full access to all loaded extensions

### Response JSON Schema

```json
{
  "status": "ok" | "error",
  "output": "captured stdout (minus trailing newline)",
  "result": "<evaluated value if expression, omitted for statements>",
  "traceback": ["formatted traceback string"],
  "ename": "ExceptionType",
  "evalue": "exception message"
}
```

## Screenshot APIs

### Viewport Screenshot (Replicator Annotators)

```python
# From active viewport
from isaacsim.test.utils.image_capture import capture_viewport_annotator_data_async
import omni.kit.viewport.utility as viewport_utils

viewport_api = viewport_utils.get_active_viewport()
rgb_data = await capture_viewport_annotator_data_async(viewport_api, "rgb")
# Returns: np.ndarray shape (H, W, 4) dtype uint8

# From a specific camera with custom resolution
from isaacsim.test.utils.image_capture import capture_rgb_data_async
rgb = await capture_rgb_data_async(
    camera_position=(10, 10, 10),
    camera_look_at=(0, 0, 0),
    resolution=(1920, 1080),
)

# Save to disk
from isaacsim.test.utils.image_io import save_rgb_image
save_rgb_image(rgb_data, "/path/to/output_dir", "screenshot.png")
```

### Full-App Screenshot (Swapchain Capture)

```python
import omni.kit.renderer.capture
import isaacsim.core.experimental.utils.app as app_utils

renderer = omni.kit.renderer.capture.acquire_renderer_capture_interface()
await app_utils.update_app_async(steps=1)
renderer.capture_next_frame_swapchain("/path/to/output.png")
await app_utils.update_app_async(steps=1)
renderer.wait_async_capture()
```

This captures the entire composited application window: menus, panels, viewport, etc.
Works in both `--no-window` headless and windowed modes.

### Viewport-Only File Capture

```python
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
viewport = get_active_viewport()
await capture_viewport_to_file(viewport, file_path="/path/to/output.png", is_hdr=False).wait_for_result()
```

## Menu Navigation APIs

### omni.kit.ui_test (Kit's built-in UI test framework)

```python
import omni.kit.ui_test as ui_test

# Simple menu click (may fail on slow submenus)
await ui_test.menu_click("File/New")

# Get the menubar widget
menubar = ui_test.get_menubar()

# Find a widget by name
widget = ui_test.find("Viewport")
widget = ui_test.find("Stage")

# Mouse emulation
from omni.kit.ui_test import Vec2, emulate_mouse_move
await emulate_mouse_move(Vec2(100, 200))

# Frame waiting
from omni.kit.ui_test import wait_n_updates
await wait_n_updates(10)

# Context menu
viewport_window = ui_test.find("Viewport")
await viewport_window.right_click()
context_menu = await ui_test.get_context_menu()
```

### isaacsim.test.utils.menu_utils (robust retry wrappers)

```python
from isaacsim.test.utils.menu_utils import (
    menu_click_with_retry,
    find_widget_with_retry,
    find_enabled_widget_with_retry,
    wait_for_widget_enabled,
    get_all_menu_paths,
)

# Click with retry (polls at each menu level, retries with different delays)
await menu_click_with_retry("Create/Sensors/Contact Sensor")

# Click and wait for window
window = await menu_click_with_retry(
    "Tools/Robotics/OmniGraph Controllers/Differential Controller",
    window_name="Differential Controller",
)

# Find widget with polling
widget = await find_widget_with_retry("Load", max_frames=100)
await widget.click()

# Find and wait for enabled
widget = await find_enabled_widget_with_retry("Play", max_frames=200)

# Extract leaf paths from context menu dict
paths = get_all_menu_paths(context_menu_dict, root_path="Create")
```

## Widget Interaction

```python
import omni.kit.ui_test as ui_test

# Find and click
widget = ui_test.find("Button Name")
await widget.click()
await widget.double_click()
await widget.right_click()

# Widget properties
w = widget.widget  # underlying omni.ui widget
w.visible
w.enabled
w.text  # for labels
w.model.get_value_as_string()  # for fields

# Window management
import omni.ui as ui
windows = ui.Workspace.get_windows()
for w in windows:
    print(w.title, w.visible, w.width, w.height)
```

## Stage Operations

Prefer `isaacsim.core.experimental.utils.stage` over raw `omni.usd` calls.

```python
import isaacsim.core.experimental.utils.stage as stage_utils

# New stage
stage = await stage_utils.create_new_stage_async(template="empty")

# Open stage
success, stage = await stage_utils.open_stage_async("/path/to/stage.usd")

# Get current stage
stage = stage_utils.get_current_stage()

# Define a prim
stage_utils.define_prim("/World/MyPrim", "Xform")

# Delete a prim
stage_utils.delete_prim("/World/MyPrim")

# Move a prim
stage_utils.move_prim("/World/Source", "/World/Destination")

# Traverse prims
stage = stage_utils.get_current_stage()
for prim in stage.Traverse():
    print(prim.GetPath(), prim.GetTypeName())

# Check if stage is loading
is_loading = stage_utils.is_stage_loading()

# Save stage
stage_utils.save_stage("/path/to/output.usd")

# Get/set stage units and up axis
meters, kilograms = stage_utils.get_stage_units()
up_axis = stage_utils.get_stage_up_axis()
```

## Prim Operations

Prefer `isaacsim.core.experimental.utils.prim` over raw `pxr` calls when possible.

```python
import isaacsim.core.experimental.utils.prim as prim_utils

# Get a prim
prim = prim_utils.get_prim_at_path("/World/MyPrim")

# Find prims by pattern
paths = prim_utils.find_matching_prim_paths("/World/Robot*")

# Get/set prim attributes
value = prim_utils.get_prim_attribute_value("/World/Cube", "size")
names = prim_utils.get_prim_attribute_names("/World/Cube")

# Check API schemas
has_rb = prim_utils.has_api("/World/Cube", "PhysicsRigidBodyAPI")
```

## App / Simulation Control

Prefer `isaacsim.core.experimental.utils.app` over raw `omni.kit.app` and `omni.timeline` calls.

```python
import isaacsim.core.experimental.utils.app as app_utils

# Render frames (required before capturing images)
app_utils.update_app(steps=120)

# Play/pause/stop simulation
app_utils.play()
app_utils.update_app(steps=100)
app_utils.pause()
app_utils.stop()

# Query simulation state
app_utils.is_playing()
app_utils.is_stopped()

# Enable/disable extensions at runtime
app_utils.enable_extension("isaacsim.test.utils")
app_utils.is_extension_enabled("isaacsim.test.utils")
```

## Transform Operations

Prefer `XformPrim` over raw xformOp manipulation.

```python
from isaacsim.core.experimental.prims import XformPrim
import numpy as np

# Get world pose (returns warp arrays)
xp = XformPrim(paths="/World/Cube")
poses = xp.get_world_poses()  # (positions[N,3], orientations[N,4]) as warp arrays
pos = np.array(poses[0].numpy())[0]   # [x, y, z]
rot = np.array(poses[1].numpy())[0]   # [w, x, y, z]

# Set world pose (numpy arrays, shape (1,3) and (1,4))
xp.set_world_poses(
    positions=np.array([[3, 0, 1]], dtype=np.float32),
    orientations=np.array([[1, 0, 0, 0]], dtype=np.float32),
)

# Scale
xp.set_local_scales(np.array([[2, 2, 2]], dtype=np.float32))

# Reset xform ops (required when switching between xform op styles)
xp.reset_xform_op_properties()
```

**Notes:**
- `xform.set_world_pose()` raises `NotImplementedError` for USD backend. Use `XformPrim.set_world_poses()`.
- Parameter name is `positions` for `set_world_poses()` but `translations` for `set_local_poses()`:
  - `xp.set_world_poses(positions=..., orientations=...)`
  - `xp.set_local_poses(translations=..., orientations=...)`
- Both take `np.ndarray` with shape `(N, 3)` for positions/translations and `(N, 4)` for orientations (wxyz).

## Camera Control

### ViewportManager (recommended)

Use `ViewportManager.set_camera_view()` for reliable camera positioning. It handles
the viewport's center-of-interest (COI) attribute correctly and works with the
viewport camera controller (which overrides raw xform ops on perspective cameras).

```python
from isaacsim.core.rendering_manager import ViewportManager
import isaacsim.core.experimental.utils.app as app_utils

# Set eye (camera position) and target (look-at point)
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[2.5, -2.0, 1.8], target=[0, 0, 0.4])
app_utils.update_app(steps=30)

# Other viewport operations
ViewportManager.get_resolution()                        # (1280, 720)
ViewportManager.set_resolution((1920, 1080))
ViewportManager.set_camera("/OmniverseKit_Top")         # switch to top view
ViewportManager.get_camera()                            # UsdGeom.Camera
```

### Camera properties

```python
from isaacsim.core.experimental.objects import Camera
import numpy as np

cam = Camera(paths="/OmniverseKit_Persp")
cam.set_focal_lengths(np.array([35.0]))
cam.get_clipping_ranges()
```

### 3D-to-screen projection

Convert world coordinates to app screen pixels (for cursor tracking in recordings).
Uses `isaacsim.test.utils.viewport_utils.project_world_to_screen`:

```python
from isaacsim.test.utils.viewport_utils import project_world_to_screen

# Project a world point to app-window screen coordinates
screen_x, screen_y = project_world_to_screen((0.0, 0.0, 0.5))
```

The function handles viewport view/projection matrix transposition, NDC conversion,
render-to-window pixel scaling, and toolbar height offset automatically.

## Kit Commands

Isaac Sim registers 370+ commands for physics, materials, meshes, etc.

```python
import omni.kit.commands

# List all commands
cmds = sorted(omni.kit.commands.get_commands().keys())

# Execute a command
omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube")
omni.kit.commands.execute("AddGroundPlane")
omni.kit.commands.execute("BindMaterial", prim_path="/World/Cube", material_path="/World/Mat")

# Undo last command
omni.kit.commands.undo()
```

## Selection

```python
import omni.usd

sel = omni.usd.get_context().get_selection()
sel.set_selected_prim_paths(["/World/Cube", "/World/Sphere"], True)
paths = sel.get_selected_prim_paths()
sel.clear_selected_prim_paths()
```

## Console Log

```python
import carb.settings
log_file = carb.settings.get_settings().get("/log/file")
# Read log_file for debugging errors
```

## Interactive Examples Browser

### High-level API

```python
from isaacsim.examples.browser import (
    get_examples,
    find_example,
    get_example_sample,
)

# List all registered examples (dict of category -> list of names)
for cat, names in get_examples().items():
    print(f"{cat}: {names}")

# Find a specific example (returns ExampleDetailItem or None)
item = find_example("General", "Hello World")
if item:
    # Execute its entrypoint
    item.example.execute_entrypoint()

# Get the BaseSample object for BaseSampleUITemplate-based examples
sample = get_example_sample("General", "Hello World")
if sample:
    await sample.load_world_async()
    await sample.reset_async()
    await sample.clear_async()
```

### Button discovery and deferred clicks (isaacsim.test.utils.button_utils)

Generic utilities for discovering and clicking UI buttons. These work with any
`BaseSampleUITemplate` example regardless of specific button names.

```python
from isaacsim.test.utils.button_utils import (
    discover_template_buttons,
    deferred_click,
    deferred_click_widget,
    get_widget_screen_center,
)

# Discover all buttons on a template (merges _buttons + task_ui_elements)
template = detail.example.ui_hook.__self__
buttons = discover_template_buttons(template)
for name in buttons:
    print(name)  # e.g. "Load World", "Reset", "Start Party", etc.

# Get screen coordinates for any widget
cx, cy = get_widget_screen_center(buttons["Load World"])

# Deferred click — fires on the NEXT event loop cycle
deferred_click_widget(buttons["Load World"])
# RETURN from the server call — click fires after the server releases the loop

# Or click by raw coordinates
deferred_click(400, 300)
```

Button callbacks use `asyncio.ensure_future()` internally. The python_server holds
the event loop, so the callback cannot run during the same TCP call. Deferred clicks
schedule via `asyncio.ensure_future()` to fire after the server returns — exactly
how a real human mouse click arrives via the OS event queue.

**What doesn't need deferred clicks:** `await sample.load_world_async()` runs in the
same event loop task (no reentrancy) — this is the code path used by `example_browser.py`.

## Asset Root Configuration

```python
import carb.settings

s = carb.settings.get_settings()

# Read current asset root
current = s.get("/persistent/isaac/asset_root/default")

# Set to staging (recommended for 6.x builds)
s.set("/persistent/isaac/asset_root/default",
      "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0")

# Set to production
s.set("/persistent/isaac/asset_root/default",
      "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0")

# Verify
from isaacsim.storage.native import get_assets_root_path
result = get_assets_root_path()
```

## Frame Sequence Capture

```python
from isaacsim.test.utils.image_capture import capture_frame_sequence_async

# Capture 60 frames of the full app (including UI chrome)
paths = await capture_frame_sequence_async("/tmp/recording", num_frames=60)

# Capture viewport-only frames with more simulation steps between captures
paths = await capture_frame_sequence_async(
    "/tmp/recording", num_frames=30, updates_per_frame=4, mode="viewport"
)

# Capture RGB at 1080p from a specific camera via replicator
paths = await capture_frame_sequence_async(
    "/tmp/recording",
    num_frames=60,
    mode="replicator",
    resolution=(1920, 1080),
    camera_prim_path="/World/Camera",
)

# Capture depth frames via replicator (saves as .npy)
paths = await capture_frame_sequence_async(
    "/tmp/depth_recording",
    num_frames=30,
    mode="replicator",
    annotator_name="distance_to_camera",
)

# Reuse an existing render product
import omni.replicator.core as rep
rp = rep.create.render_product("/World/Camera", (1920, 1080))
paths = await capture_frame_sequence_async(
    "/tmp/recording",
    num_frames=60,
    mode="replicator",
    render_product=rp,
)
rp.destroy()  # caller owns the render product
```

## Visual Menu Navigation (for video capture)

```python
from isaacsim.test.utils.menu_utils import navigate_menu_visual

# Simple navigation and click
await navigate_menu_visual("Create/Mesh/Cube")

# With per-frame callback for recording
positions = []
async def on_frame(x, y):
    positions.append((x, y))
    # Capture screenshot, log cursor, etc.

await navigate_menu_visual(
    "Create/Physics/Ground Plane",
    hover_frames=6,          # intermediate menu items
    leaf_hover_frames=12,    # final item before click
    on_frame=on_frame,
)
```

Uses L-shaped cursor movement (horizontal then vertical) to keep submenus open.
Diagonal movement causes the cursor to exit the menu bounds, closing submenus.

## Mouse Drag and Gizmo Interaction

```python
from omni.kit.ui_test import Vec2, emulate_mouse_drag_and_drop, emulate_mouse_move_and_click

# Click to select an object (e.g. in viewport)
await emulate_mouse_move_and_click(Vec2(screen_x, screen_y))

# Drag transform gizmo (reliable for all gizmo axes)
start = Vec2(gizmo_x, gizmo_y)       # gizmo handle position
end = Vec2(gizmo_x, gizmo_y - 100)   # drag upward for Z axis
await emulate_mouse_drag_and_drop(start, end)
```

For gizmo screen positions, either:
1. Use 3D-to-screen projection (see Camera Control section above)
2. Place cursor at known object position + offset for the axis handle

## Common Menu Paths

These are typical paths in Isaac Sim (may vary by configuration):

- `File/New` — New empty stage
- `File/Open` — Open stage dialog
- `File/Save` — Save current stage
- `Create/Mesh/Cube` — Add a cube
- `Create/Mesh/Sphere` — Add a sphere
- `Create/Light/Distant Light` — Add distant light
- `Create/Physics/Ground Plane` — Add ground plane
- `Create/Physics/Physics Scene` — Add physics scene
- `Edit/Capture Screenshot` — Built-in screenshot (F10)
- `Window/Examples/Robotics Examples` — Open the examples browser
- `Tools/...` — Various tool windows
