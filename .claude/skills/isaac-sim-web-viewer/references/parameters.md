# Parameters, status, and local testing

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `ISAAC_SIM_IMAGE` | latest from NGC tags page | Default is whatever the catalog marks **Latest** (today: `nvcr.io/nvidia/isaac-sim:6.0.1`). Override with `--image`. |
| Compose file | `<skill-dir>/docker/docker-compose.yml` | Shipped with the skill. Optional `--compose-file` / `--repo-root` only if the operator points at another copy. |
| Compose project | `isim` | `-p isim` |
| `WEB_VIEWER_PORT` | `8210` | HTTP port for the browser. |
| `ISAACSIM_HOST` | `127.0.0.1` | **Ask** local vs remote first. Local → `127.0.0.1`. Remote or both → `--host auto` or the reachable IP. Must be a single IPv4 or DNS name, not `0.0.0.0`. Rebuild (`--build`) after changing it. |
| `ISAACSIM_SIGNAL_PORT` | `49100` | WebRTC signaling (TCP). |
| `ISAACSIM_STREAM_PORT` | `47998` | WebRTC media (UDP). |
| `ISAAC_SIM_DATA` | `~/docker/isaac-sim` | Isaac caches/config/logs. Isolated from other `~/docker/*` trees. |
| `ISAACSIM_HUB_CACHE_PATH` | `~/docker/isaac-sim/hub` | Hub Workstation cache for this stack only. Set to `~/.cache/ov/hub` only if the operator wants to share Hub with other Omniverse apps. |
| `GPU_DEVICE` | `all` | Pin to a GPU index for multi-instance. |

The web viewer bakes `ISAACSIM_HOST`, `ISAACSIM_SIGNAL_PORT`, and `ISAACSIM_STREAM_PORT` into the JavaScript bundle at **build** time. Changing any of them requires `up --build`. The HTTP URL you give the operator must use that same host.

## Checking status

While Isaac Sim is starting, poll compose logs for the URL and for fatal isaac-sim errors (warnings are OK):

```bash
<skill-dir>/scripts/poll_compose_logs.sh
```

Equivalent: `docker compose -p isim -f <skill-dir>/docker/docker-compose.yml logs` (optionally `web-viewer` / `isaac-sim`) and `... ps`.

`launch_prebuilt_web_viewer.sh` waits for isaac-sim logs to contain `Isaac Sim Full Streaming App is loaded`, then prints the HTTP URL. Do not intimate the operator with the URL before that line. Stop and report if isaac-sim logs show `[Error]`, a Python traceback, or a segfault. Fabric version warnings and `Failed to open [/var/run/utmp]` are not fatal.

## Local testing

No Isaac Sim source checkout is required. Product default remains `ISAACSIM_SIGNAL_PORT=49100`. If 49100/tcp is already bound on the host, pass `--signal-port 49101` (or another free port). Do not change the script default.

1. Offline self-test (`self_test.sh` uses `--signal-port 49101` when `LOCAL_TEST_SIGNAL_PORT` is unset): `<skill-dir>/scripts/self_test.sh`
2. Preflight: `<skill-dir>/scripts/preflight.sh --signal-port 49101`
3. Dry-run URL (`http://127.0.0.1:8210`): `<skill-dir>/scripts/launch_prebuilt_web_viewer.sh --signal-port 49101 --dry-run`
4. Resolve Latest from the NGC tags page, then pull **before** compose. Ask local vs remote before a real launch; use `--host auto` for a browser on another machine.
