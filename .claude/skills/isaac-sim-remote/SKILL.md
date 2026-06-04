---
name: isaac-sim-remote
description: >
  Connect to a running Isaac Sim via the `isaacsim.code_editor.python_server`
  TCP socket (port 8226) to execute Python remotely. Launch Isaac Sim, send
  code, create/modify USD stages, run simulations, take viewport or full-app
  screenshots, inspect/modify prims, control the camera, step physics, read
  console logs, execute Kit commands. Works in `--no-window` headless mode.
---

# Isaac Sim Remote

Execute Python inside a running Isaac Sim via the `isaacsim.code_editor.python_server` TCP socket.

Related: `debug-with-local-kit` (when behavior depends on a Kit-from-source build), `profile-isaac-sim` (to attach Tracy to the running process), `isaac-sim-validator` (final QA gate on any rendered output).

Upstream `isaac-sim-ui` (menu/widget OmniUIQuery automation) and `isaac-sim-recording` (cursor tracking, tutorial video capture) are not imported. The inline UI patterns here (Play-button click via `OmniUIQuery`, full-app vs viewport screenshots) cover the common cases.

## Launching Isaac Sim

```bash
cd _build/linux-x86_64/release

# Headless — supports all features: code execution, viewport screenshots,
# full-app screenshots, menu clicks, and widget interaction
bash isaac-sim.sh --no-window --no-ros-env \
    --enable isaacsim.code_editor.python_server

# With display (optional — only needed if you want to visually observe the UI)
DISPLAY=:99 bash isaac-sim.sh --no-ros-env \
    --enable isaacsim.code_editor.python_server

# With --reset-user to clear persistent user settings (recommended for clean state)
DISPLAY=:99 bash isaac-sim.sh --reset-user --no-ros-env \
    --enable isaacsim.code_editor.python_server
```

Wait for `app ready` in the output before sending commands. The TCP server listens on `127.0.0.1:8226`.

**Extension enable flags (important):**
- Use `--enable isaacsim.code_editor.python_server` — this is the **only** way to enable it.
- `--/exts/isaacsim.code_editor.python_server/enabled=true` does **NOT** work. That syntax sets a carb setting, not an extension enable flag. The python_server extension is not enabled by default in the app .kit file, so it must be explicitly enabled via `--enable`.
- The same applies to `isaacsim.test.utils` and any other extension not in the default app config.

**Other notes:**
- Full-app screenshots require a display (`DISPLAY` env var). Viewport screenshots work headless.
- Use `--reset-user` when settings seem stale (wrong asset root, unexpected defaults).

### Verifying the server started

After launch, verify the TCP port is open before sending commands:

```bash
# Wait for port 8226 to open (poll every 2s, timeout 120s)
for i in $(seq 1 60); do
    nc -z 127.0.0.1 8226 2>/dev/null && echo "Port open" && break
    sleep 2
done

# Or check the log for the extension loading
grep "python_server" /tmp/isaac_sim.log
```

If the port never opens, check that `isaacsim.code_editor.python_server` appears in the startup log. If it doesn't, the `--enable` flag was not passed correctly.

### Enabling additional extensions

```bash
bash isaac-sim.sh --no-window --no-ros-env \
    --enable isaacsim.code_editor.python_server \
    --enable isaacsim.test.utils \
    --enable isaacsim.sensors.experimental.rtx
```

## Health Check

Verify the server is running and get environment info:

```bash
python scripts/isaacsim_send.py --file scripts/health_check.py
```

Reports: Isaac Sim version, asset root, stage state, timeline, display, enabled extensions count.

## Asset Root Configuration

Isaac Sim assets (robots, environments) require either a Nucleus server or S3 cloud fallback. If Nucleus is unavailable, configure S3 assets:

```bash
python scripts/isaacsim_send.py --file scripts/set_asset_root.py --arg asset_root=staging
```

**Asset servers:** `staging` (S3, Isaac 6.0 paths), `production` (S3, Isaac 5.0 paths), `nucleus` (default).

## Sending Code

Use `scripts/isaacsim_send.py`:

```bash
# Inline code
python scripts/isaacsim_send.py 'print("hello")'

# From a .py file with injected variables
python scripts/isaacsim_send.py --file scripts/app_screenshot.py \
    --arg output_path=/tmp/shot.png

# Custom timeout
python scripts/isaacsim_send.py --timeout 120 'long_running()'

# Raw JSON output
python scripts/isaacsim_send.py --raw 'print("hello")'
```

### Response format

```json
{"status": "ok", "output": "hello", "result": null}
{"status": "error", "output": "", "ename": "NameError", "evalue": "name 'x' is not defined", "traceback": ["..."]}
```

Exit codes: `0` = ok, `1` = error or connection failure.

## Async Code

The server supports top-level `await`:

```bash
python scripts/isaacsim_send.py '
import isaacsim.core.experimental.utils.stage as stage_utils
await stage_utils.create_new_stage_async(template="empty")
'
```

## State Persistence & Named Contexts

The server supports named execution contexts — each is an independent globals dict. Variables set in `--context A` are invisible in `--context B`.  The default context (no `--context` flag) is a shared namespace where variables persist across calls.

```bash
# Default context — shared globals, variables persist
python scripts/isaacsim_send.py 'MY_PATHS = ["/World/A"]'
python scripts/isaacsim_send.py 'print(MY_PATHS)'  # still available

# Named contexts — fully isolated
python scripts/isaacsim_send.py --context rec 'fc = 0; frames = []'
python scripts/isaacsim_send.py --context browser 'detail = None; cat = None'
# rec and browser are fully isolated from each other and from the default context
```

**When to use named contexts:**
- **Recording** — frame counters, cursor state, output dir
- **Browser automation** — widget references, button positions
- **Scene setup** — stage references, prim paths
- **Any multi-tool workflow** where globals must not leak between tools

Delete a context when done:
```bash
python scripts/isaacsim_send.py --introspect delete_context --context rec
```

## JSON Envelope Protocol

The client supports a JSON envelope for advanced features.  It auto-detects when to use the envelope (any of `--context`, `--fire-and-forget`, `--execution-timeout`, or `--args-json` triggers it).  Raw Python source still works for simple calls.

```bash
# Named context with file
python scripts/isaacsim_send.py --context recording --file setup.py

# Per-request server-side timeout (kills async code cleanly)
python scripts/isaacsim_send.py --execution-timeout 30 'await long_operation()'

# Inject args via JSON (type-safe, no string parsing)
python scripts/isaacsim_send.py --args-json '{"x": 42, "name": "robot"}' 'print(f"{name}={x}")'

# Fire-and-forget — immediate ACK, code runs in background
python scripts/isaacsim_send.py --fire-and-forget 'heavy_computation()'
# Returns: Task submitted. task_id: <uuid>

# Query background task result
python scripts/isaacsim_send.py --introspect task <task_id>

# Server introspection
python scripts/isaacsim_send.py --introspect status    # uptime, connections, tasks
python scripts/isaacsim_send.py --introspect contexts   # list all named contexts
python scripts/isaacsim_send.py --introspect tasks      # list completed background tasks
```

### Execution Timeouts

Async code is cancelled cleanly via `asyncio.wait_for()`.  Sync code uses a background watchdog — the code finishes running but the client gets a `TimeoutError` response.

```bash
# This returns TimeoutError after 5s (the sleep(100) is cancelled)
python scripts/isaacsim_send.py --execution-timeout 5 'import asyncio; await asyncio.sleep(100)'
```

### Fire-and-Forget

For deferred UI clicks, background data loading, or any operation where you don't need to wait for the result:

```bash
ack=$(python scripts/isaacsim_send.py --raw --fire-and-forget 'load_large_usd()')
task_id=$(echo "$ack" | python3 -c "import sys,json; print(json.load(sys.stdin)['task_id'])")

# Later...
python scripts/isaacsim_send.py --introspect task "$task_id"
```

Up to 100 completed results are retained (FIFO eviction).

## Screenshots

```bash
# Full-app (UI chrome + viewport, requires DISPLAY)
python scripts/isaacsim_send.py --file scripts/app_screenshot.py --arg output_path=/tmp/full.png

# Viewport only (3D render, works headless)
python scripts/isaacsim_send.py --file scripts/viewport_screenshot.py --arg output_path=/tmp/vp.png

# Annotator data (depth, normals, segmentation)
python scripts/isaacsim_send.py --file scripts/capture_annotator.py \
    --arg annotator=distance_to_camera --arg output_path=/tmp/depth.npy
```

## Stage Management

```bash
# Create new empty stage
python scripts/isaacsim_send.py --file scripts/open_stage.py --arg action=new

# Open a USD file
python scripts/isaacsim_send.py --timeout 120 --file scripts/open_stage.py \
    --arg usd_path=/path/to/scene.usd

# Save stage
python scripts/isaacsim_send.py --file scripts/open_stage.py \
    --arg action=save --arg usd_path=/tmp/saved.usd

# Stage info
python scripts/isaacsim_send.py --file scripts/open_stage.py --arg action=info
```

## Scene Inspection

```bash
# Stage tree
python scripts/isaacsim_send.py --file scripts/stage_info.py

# Detailed prim view
python scripts/isaacsim_send.py --file scripts/stage_info.py --arg prim_path=/World/Cube

# Filter by type
python scripts/isaacsim_send.py --file scripts/stage_info.py --arg filter_type=Camera

# List/read/set prim attributes
python scripts/isaacsim_send.py --file scripts/prim_properties.py \
    --arg prim_path=/World/Cube --arg action=list

# Select prims
python scripts/isaacsim_send.py --file scripts/select_prim.py \
    --arg action=set --arg prim_path=/World/Cube
```

## Transform Control

```bash
python scripts/isaacsim_send.py --file scripts/prim_transform.py \
    --arg prim_path=/World/Cube --arg action=set --arg "position=3,0,1"
```

## Camera Control

```bash
# Get pose
python scripts/isaacsim_send.py --file scripts/camera_control.py

# Set position + look-at target
python scripts/isaacsim_send.py --file scripts/camera_control.py \
    --arg action=set --arg "position=1.5,1.5,1.2" --arg "target=0,0,0.5"

# List cameras
python scripts/isaacsim_send.py --file scripts/camera_control.py --arg action=list_cameras
```

**Inline camera control** (recommended — handles viewport COI and camera controller correctly):

```python
from isaacsim.core.rendering_manager import ViewportManager
import isaacsim.core.experimental.utils.app as app_utils

ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.5, 1.5, 1.0], target=[0.0, 0.0, 0.4])
app_utils.update_app(steps=30)
```

## Simulation Control

```bash
python scripts/isaacsim_send.py --file scripts/simulation_control.py --arg action=play
python scripts/isaacsim_send.py --file scripts/simulation_control.py --arg action=step --arg num_steps=100
python scripts/isaacsim_send.py --file scripts/simulation_control.py --arg action=stop
```

### Playing simulation via UI (real click on Play button)

When you need a real button click for recording or testing (not just API):

```python
import omni.ui as ui
from omni.ui_query import OmniUIQuery
from omni.kit.ui_test import Vec2, emulate_mouse_move_and_click

# Find the Play button in the toolbar
toolbar_win = next(w for w in ui.Workspace.get_windows() if w.title == "Main ToolBar")
for path in OmniUIQuery.get_window_widget_paths(toolbar_win):
    widget = OmniUIQuery.find_widget(path)
    if widget and getattr(widget, "name", "") == "play":
        px = widget.screen_position_x + widget.computed_width / 2
        py = widget.screen_position_y + widget.computed_height / 2
        break

# Direct click — toolbar buttons do NOT need deferred clicks
await emulate_mouse_move_and_click(Vec2(px, py))
await app_utils.update_app_async(steps=30)
assert app_utils.is_playing(), "Play button click failed"
```

**Note:** Toolbar buttons (Play, Pause, Stop) use direct clicks. Browser buttons (extension/example browsers) need deferred clicks via `omni.kit.ui_test`. See `references/pitfalls.md` for the deferred-click pattern.

## Command Execution

```bash
# List commands
python scripts/isaacsim_send.py --file scripts/execute_command.py --arg action=list --arg filter=Physics

# Execute
python scripts/isaacsim_send.py --file scripts/execute_command.py \
    --arg command_name=CreateMeshPrimWithDefaultXform --arg 'kwargs={"prim_type":"Cone"}'
```

## Console Log

```bash
python scripts/isaacsim_send.py --file scripts/console_log.py                    # tail
python scripts/isaacsim_send.py --file scripts/console_log.py --arg action=errors  # errors only
python scripts/isaacsim_send.py --file scripts/console_log.py --arg action=search --arg query=PhysX
```

## Moving Prims (Setting World Pose)

**Do NOT use** `xform.set_world_pose()` — it raises `NotImplementedError`.

**Before simulation starts**, use `XformPrim`:

```python
import numpy as np
from isaacsim.core.experimental.prims import XformPrim

target = XformPrim(paths="/World/TargetCube")
target.set_world_poses(positions=np.array([[0.3, 0.2, 0.5]]))
```

**During simulation** (after `play()`), `XformPrim.set_world_poses` may fail with `RuntimeError: Item indexing is not supported on wp.array objects` because warp arrays replace numpy arrays at runtime. Use raw USD instead:

```python
from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/TargetCube")
xformable = UsdGeom.Xformable(prim)
for op in xformable.GetOrderedXformOps():
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        op.Set(Gf.Vec3d(0.3, 0.2, 0.5))
        break
```

**Reading** works fine with xform utils (both before and during simulation):

```python
from isaacsim.core.experimental.utils import xform
pos, rot = xform.get_world_pose("/World/TargetCube")
```

## Common Patterns

### Create a new stage with objects

```python
import numpy as np
from isaacsim.core.experimental.objects import Cube, DomeLight
import isaacsim.core.experimental.utils.stage as stage_utils

await stage_utils.create_new_stage_async(template="empty")
stage_utils.define_prim("/World", "Xform")
DomeLight("/World/DomeLight").set_intensities(np.array([3000.0]))
Cube("/World/RedCube", sizes=1.0, colors="red", positions=(0, 0, 0.5))
```

### Step the renderer / simulation

**Prefer `await update_app_async()` over `update_app()`** when running code that uses `await` (which includes all python_server scripts with top-level `await`). The sync `update_app()` pumps the event loop from inside an asyncio Task, which causes "Cannot enter into task" errors in other extensions. The async version yields properly.

```python
import isaacsim.core.experimental.utils.app as app_utils

# Async (preferred — yields to event loop, no reentrancy errors)
await app_utils.update_app_async(steps=120)  # Render frames (warm-up)
app_utils.play()
await app_utils.update_app_async(steps=100)
app_utils.stop()

# Sync (use in non-async contexts only, e.g. tests, standalone scripts)
app_utils.update_app(steps=120)
```

## Key APIs

**Isaac Sim APIs (preferred):**
- `isaacsim.core.experimental.utils.stage` — stage ops
- `isaacsim.core.experimental.utils.app` — `update_app()`, `play()`, `stop()`, `enable_extension()`
- `isaacsim.core.experimental.utils.prim` — prim queries
- `isaacsim.core.experimental.objects` — `Cube`, `Sphere`, `DomeLight`, `GroundPlane`
- `isaacsim.core.experimental.prims` — `XformPrim`, `RigidPrim`, `GeomPrim`
- `isaacsim.storage.native` — `get_assets_root_path()`

**Kit APIs:**
- `omni.kit.renderer.capture` — `capture_next_frame_swapchain()` for full-app capture
- `omni.kit.actions.core` — `get_action_registry()` for registered actions

See `references/api-reference.md` for full details. See `references/pitfalls.md` for common issues.

## Important Notes

- **Lighting in headless mode**: No lights = black image. Always add a `DomeLight` with intensity 1000–5000.
- **Renderer warm-up**: After new stage, call `app_utils.update_app(steps=120)` before rendering.
- **Connection errors**: Ensure Isaac Sim shows `app ready` and python_server is `--enable`d.
- **Server hangs**: Crashed examples can wedge the server. Restart Isaac Sim. Use `health_check.py` to verify.
- **Assets not found**: Configure S3 fallback with `set_asset_root.py`.

