# Pitfalls and Gotchas — Python Server Client

Hard-won lessons from debugging Isaac Sim remote execution.

## asyncio Reentrancy (RESOLVED in v1.2.1)

**Previous problem (v1.0–v1.2.0):** Async user code ran inside an asyncio Task
(`_await_and_reply`). When that code yielded to the event loop (e.g. `create_new_stage_async`,
`update_app_async`), other pending tasks tried to wake up but hit
`RuntimeError: Cannot enter into task ... while another task is being executed` on Python 3.12+.

**Fix (v1.2.1):** Async user coroutines are now driven manually via `_drive_coroutine()`,
which steps the coroutine with `coro.send()` and `loop.call_soon()` without creating a Task.
User code never runs as the "current task", so other tasks are free to run concurrently.

**Remaining edge case:** UI button callbacks that use `asyncio.ensure_future()` internally
still need the deferred-click pattern when triggered from inside python_server code:

```python
import asyncio
async def _click():
    from omni.kit.ui_test import Vec2, emulate_mouse_move_and_click
    await emulate_mouse_move_and_click(Vec2(btn_x, btn_y))
asyncio.ensure_future(_click())
# RETURN from server call — click fires on next loop cycle
```

This is because the button's callback creates its OWN Task, and if it calls
`update_app()` internally, reentrancy can still occur.

**Affected:** UI button callbacks that create tasks (LOAD, START buttons).
**NOT affected:** Direct API calls like `await stage_utils.create_new_stage_async()`,
`await app_utils.update_app_async()`, `app_utils.play()`, etc.

## XformPrim Parameter Names

- `set_world_poses(positions=..., orientations=...)`
- `set_local_poses(translations=..., orientations=...)` ← NOT `positions`!
- Both take `np.ndarray` shape `(N, 3)` or `(N, 4)`, dtype `float32`

## Camera Positioning

The viewport camera controller overrides raw xform ops on perspective cameras.
`XformPrim.set_world_poses()` on `/OmniverseKit_Persp` appears to work but the
viewport controller snaps it back next frame.

**Fix:** Use `ViewportManager.set_camera_view()`:

```python
from isaacsim.core.rendering_manager import ViewportManager
ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[2.5, -2.0, 1.8], target=[0, 0, 0.4])
```

## Browser TreeView Items Not Discoverable

`omni.kit.ui_test.find("Manipulation")` returns None because TreeView categories
and thumbnails are custom-rendered, not standard named widgets.

**Fix:** Use the model API: `inner.category_selection = [item]` for categories,
`detail_view.selection = [item]` for examples. See `scripts/browser_helpers.py`.

## Two Button Dicts on Templates

`BaseSampleUITemplate` stores buttons in two separate dicts:
- `template._buttons` — world control buttons (typically Load World, Reset)
- `template.task_ui_elements` — task-specific buttons (names vary per example)

Always use `discover_buttons()` from `browser_helpers.py` to enumerate both.

## Menu L-shaped Path

Diagonal cursor movement between menu levels exits the menu bounds, closing submenus.
`navigate_menu_visual()` uses horizontal-first then vertical movement (L-shaped path)
to keep the cursor within the submenu column.

## SystemExit Handled Gracefully (v1.2.1+)

`raise SystemExit(N)` and `sys.exit(N)` in user code are now caught by the executor.
The server stays alive and returns a proper error response:

```json
{"status": "error", "ename": "RuntimeError", "evalue": "SystemExit(0) intercepted — use raise RuntimeError() instead"}
```

**Previous behavior (v1.0–v1.2.0):** `SystemExit` propagated to Kit's framework,
triggering an app shutdown attempt. The server either died or became unresponsive.

## Extension Singleton Lost on Module Reload

`get_instance()` may return None if the extension was disabled/re-enabled, because
the module-level singleton reference is lost.

**Recovery:** `import gc; ext = next(o for o in gc.get_objects() if type(o).__name__ == "ExampleBrowserExtension")`

This works because `isinstance()` fails after reload (different class objects) but
`type(o).__name__` string comparison still works.

## --enable vs Settings Flag

`--/exts/isaacsim.code_editor.python_server/enabled=true` does NOT work for
the python server extension. Must use `--enable isaacsim.code_editor.python_server`.

## Dict Literals as Expressions

Sending a raw Python dict literal like `{"a": 1}` as code will be auto-detected
as a JSON envelope (because it starts with `{`), not as Python. The server
silently treats it as an empty-code envelope request.

**Fix:** Either wrap in `print()` — `print({"a": 1})` — or prefix with a space/newline
to avoid JSON detection. Note: even `' {"a": 1}'` won't work because the server
uses `source.lstrip().startswith("{")` which strips leading whitespace.

The reliable workaround is to assign the dict: `x = {"a": 1}; print(x)`.

## create_new_stage_async vs File/New Menu

`File/New` menu triggers a "Would you like to save?" dialog if the stage is dirty.
Use `await stage_utils.create_new_stage_async()` to bypass the dialog.

## Stale User Settings

Isaac Sim persists user settings (asset root, window layouts) across sessions.
If settings seem wrong, launch with `--reset-user` to clear them.

## Renderer Warm-up

After loading a stage or creating objects, call `app_utils.update_app(steps=120)`
before taking screenshots. The renderer needs time to compile shaders and
converge the path tracer. For minor changes, `steps=30` is usually enough.

## Display for Full-App Screenshots

Full-app (swapchain) screenshots require a `DISPLAY` env var even in `--no-window` mode.
Viewport screenshots (via replicator annotators) work without any display.
Use `DISPLAY=:99` with Xvfb for headless servers.

## Docked Tab Focus — Clicks Landing on Wrong Panel

When opening a window that docks as a tab (e.g. "Robotics Examples" alongside Content),
the new tab may NOT be in front. Deferred clicks at the button's screen coordinates
will hit the foreground panel (Content) instead — **silently doing nothing**.

**Symptom:** `click_button_by_name()` returns without error but the button never fires.
Stage stays empty. Screenshot reveals the wrong panel is in front.

**Fix:** After opening a docked window, explicitly focus it:

```python
import omni.ui as ui
for w in ui.Workspace.get_windows():
    if w.title == "Robotics Examples":
        w.focus()
        break
await app_utils.update_app_async(steps=10)
```

Repeat after any dock resize operation (e.g. `ensure_buttons_visible_async()`).

## Button Enable Timing After LOAD

After clicking LOAD, `wait_for_prim("/World/Franka")` confirms the USD loaded, but
the `BaseSampleExtension._on_load_world_async()` has MORE steps:

1. `await self._sample.load_world_async()` — loads USD ← this is what `wait_for_prim` catches
2. `await next_update_async()` — one more frame
3. Subscribe to stage events
4. Subscribe to timeline stop events
5. `_enable_all_buttons(True)` ← task buttons become clickable HERE

**Symptom:** START/task buttons show `enabled=False` even though the scene loaded.
Deferred click does nothing.

**Fix:** After `wait_for_prim`, poll until task buttons are enabled:

```python
for i in range(20):
    await app_utils.update_app_async(steps=10)
    if discover_buttons(detail)["<task_button>"].enabled:
        break
```

## Example Workflow: LOAD → Play → START

Most interactive examples (Follow Target, Path Planning, etc.) require THREE steps:

1. **LOAD** — creates World, loads USD, registers physics callbacks
2. **Play** (timeline) — starts the physics simulation loop
3. **START** (task button) — activates the specific task (IK, navigation, etc.)

**Symptom:** Robot appears on stage but doesn't move. Target prim moves (via USD) but
the robot ignores it. The IK solver runs inside a physics step callback that only
executes when the simulation is playing.

**Fix:** Always click Play (real click or `timeline.play()`) between LOAD and START.

## Toolbar vs Browser Button Click Types

| Button type | Click method | Reason |
|---|---|---|
| Toolbar (Play, Pause, Stop) | Direct `emulate_mouse_move_and_click()` | Simple sync callback |
| Browser (LOAD, START, task) | `deferred_click` / `click_button_by_name()` | Uses `ensure_future()` internally |

**Symptom:** Direct click on browser buttons — tooltip appears but action never fires.
Deferred click on toolbar — works but unnecessarily complex.

## Module-Level Global Corruption Across Calls

When multiple scripts share a context, module-level mutable variables
(`_fc = 0`, `_cursor_log = []`) persist across re-loads of the same script.
Even re-assigning them may not take effect if old function closures still
reference the prior value.

**Symptom:** Frame counters jump to stale values. State from a prior recording
session bleeds into a new one.

**Fix:** Use named execution contexts (`--context recording`, `--context browser`)
to isolate tools from each other. Each context is an independent globals dict.
Delete a context when done:

```bash
python scripts/isaacsim_send.py --introspect delete_context --context recording
```

Within a context, store all mutable session state in a single dict (e.g.
`_tut = {"fc": 0, ...}`). Functions read/write from the dict directly.
New sessions delete and recreate the entire dict.
