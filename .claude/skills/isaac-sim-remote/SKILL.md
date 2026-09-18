---
name: isaac-sim-remote
description: "Localhost IPC client for a trusted Isaac Sim python_server (127.0.0.1:8226). Use for live stage iteration; intentional full Kit/Python surface on loopback only."
license: Apache-2.0
metadata:
  author: Hammad Mazhar <info@nvidia.com>
---

# Isaac Sim Remote

## Purpose

Drive an already-running Isaac Sim over the code-editor TCP socket so agents can iterate on stages, prims, physics, and screenshots without restarting the app.

## Security

This skill is a **local developer control plane**, not a remote-execution service.

- **Bind**: clients default to `127.0.0.1:8226`. Do not publish that port off-host.
- **Trust boundary**: equal to the OS user who launched Isaac Sim on this machine.
- **Design (intentional)**: `isaacsim_send.py` delivers Python source to the local
  code-editor server; `execute_command.py` dispatches any name already present in
  `omni.kit.commands.get_commands()` after an identifier check and audit log. An
  allowlist would defeat live Kit iteration — treat registry-wide dispatch as the
  product, not a hole.
- **Mitigations**: loopback-only default; `--dry-run` prints payloads without send;
  CLI `--arg` values use `ast.literal_eval` (literals only).

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR` or `_build/linux-x86_64/release`).
- NVIDIA GPU with a current driver (`nvidia-smi`).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- Requires Isaac Sim launched with the Python code-editor server enabled.
- Remote scripts cannot import arbitrary repo modules unless paths are injected.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Connection refused on 8226 | Python server disabled | Launch Isaac Sim with code-editor server enabled |
| `NameError` in remote cell | Module not on `sys.path` | Inline helpers or inject repo paths before import |
| Stale stage state | Prior remote edits | Reset prims or reopen stage before repro steps |

Related: `debug-with-local-kit` (when behavior depends on a Kit-from-source build), `profile-isaac-sim` (to attach Tracy to the running process), `isaac-sim-validator` (final QA gate on any rendered output).

Upstream `isaac-sim-ui` (menu/widget OmniUIQuery automation) and `isaac-sim-recording` (cursor tracking, tutorial video capture) are not imported. The inline UI patterns here (Play-button click via `OmniUIQuery`, full-app vs viewport screenshots) cover the common cases.

Every helper in `scripts/` is invoked through `scripts/isaacsim_send.py`. From agent runtimes that expose skill execution helpers, call e.g. `run_script("scripts/app_screenshot.py", args=["--help"])`; from a built tree run the file with `./python.sh` (Linux) / `python.bat` (Windows) under `_build/*/release`. Command recipes live in [`references/workflows.md`](references/workflows.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/app_screenshot.py` | App screenshot | see script --help |
| `scripts/camera_control.py` | Camera control | see script --help |
| `scripts/capture_annotator.py` | Capture annotator | see script --help |
| `scripts/console_log.py` | Console log | see script --help |
| `scripts/execute_command.py` | Dispatch a registered `omni.kit.commands` entry (loopback trust; see Security) | see script --help |
| `scripts/health_check.py` | Health check | see script --help |
| `scripts/isaacsim_send.py` | Localhost python_server IPC client (`--dry-run` supported) | CLI flags via argparse (see script --help) |
| `scripts/open_stage.py` | Open stage | see script --help |
| `scripts/prim_properties.py` | Prim properties | see script --help |
| `scripts/prim_transform.py` | Prim transform | see script --help |
| `scripts/select_prim.py` | Select prim | see script --help |
| `scripts/set_asset_root.py` | Set asset root | see script --help |
| `scripts/simulation_control.py` | Simulation control | see script --help |
| `scripts/stage_info.py` | Stage info | see script --help |
| `scripts/verify_asset.py` | Verify asset | see script --help |
| `scripts/viewport_screenshot.py` | Viewport screenshot | see script --help |
| `scripts/viewport_video.py` | Viewport video | see script --help |

## Launching Isaac Sim

```bash
cd _build/linux-x86_64/release
bash isaac-sim.sh --no-window --no-ros-env \
    --enable isaacsim.code_editor.python_server
```

Wait for `app ready`. TCP server: `127.0.0.1:8226`. Use `--enable isaacsim.code_editor.python_server` (not carb `/enabled=true`). Full launch variants and health/asset/send/screenshot/control workflows: [`references/workflows.md`](references/workflows.md). Pitfalls: [`references/pitfalls.md`](references/pitfalls.md). APIs: [`references/api-reference.md`](references/api-reference.md).

## Important Notes

- **Lighting in headless mode**: No lights = black image. Always add a `DomeLight` with intensity 1000–5000.
- **Renderer warm-up**: After new stage, call `await app_utils.update_app_async(steps=120)` before rendering.
- **Connection errors**: Ensure Isaac Sim shows `app ready` and python_server is `--enable`d.
- **Server hangs**: Crashed examples can wedge the server. Restart Isaac Sim. Use `health_check.py` to verify.
- **Assets not found**: Configure S3 fallback with `set_asset_root.py`.
