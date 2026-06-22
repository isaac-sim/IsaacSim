# Testing Docker Compose (Isaac Sim + Web Viewer)

Manual test procedures for verifying the Docker Compose deployment and web viewer functionality.

## Tested Configurations

**Isaac Sim host (server):**

| Platform | GPU |
| -------- | --- |
| Ubuntu 24.04 (x86_64) | NVIDIA RTX 6000 Ada |
| DGX Spark | NVIDIA Blackwell |

**Web viewer client (browser):**

| OS | Browser |
| -- | ------- |
| Windows 11 | Chrome |
| Ubuntu 24.04 | Chrome |
| macOS | Chrome |

## Prerequisites

- Docker and NVIDIA Container Toolkit installed on the host
- Isaac Sim Docker image available (locally built or prebuilt NGC image)
- Google Chrome on the client machine

## 1. Deploy the stack

```bash
cd tools/docker
docker compose -p isim -f docker-compose.yml up --build -d
```

- **First run**: builds the `web-viewer` image (Node/Vite), then starts Isaac Sim and the web viewer.
- Isaac Sim healthcheck waits for `AppReady` in logs (up to ~3 min start period), then the web viewer starts.

To use a prebuilt NGC image instead of building locally:

```bash
ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1 docker compose -p isim -f docker-compose.yml up --build -d
```

## 2. Check status and get the web viewer URL

```bash
docker compose -p isim logs web-viewer
```

Open the URL shown in the logs (e.g. `http://<host-ip>:8210`) in a Chromium-based browser. For cloud VMs, see [Cloud Deployment](README.md#cloud-deployment-aws-gcp-azure) in the README.

## 3. Verify streaming

1. Open the web viewer URL in Chrome/Edge.
2. Wait for the stream to connect (the "WAITING FOR STREAM..." message should disappear).
3. You should see the Isaac Sim viewport with the default stage.
4. Verify you can interact with the viewport (mouse orbit, pan, zoom).

## 4. Keyboard shortcut tests

Focus the stream area (click inside the viewer), then verify each shortcut:

| Test | Windows / Linux | Mac | Expected |
| ---- | --------------- | --- | -------- |
| Copy / paste | **Ctrl+C** / **Ctrl+V** | **Ctrl+C** / **Ctrl+V** | Clipboard works between host and Isaac Sim (requires [secure context setup](README.md#troubleshooting)) |
| Refresh page | **F5** | **Fn+F5** or **Cmd+R** | Page reloads, stream reconnects |
| Maximize viewport | **F7** | **Fn+F7** | Isaac Sim viewport maximizes (no caret-browsing dialog) |
| Fullscreen | **F11** | **Shift+Fn+F11** | Browser toggles fullscreen; press again to exit |
| Open DevTools | **F12** | **Fn+F12** or **Cmd+Option+I** | DevTools panel opens |
| Open Console | **Ctrl+Shift+J** | **Cmd+Option+J** | Console panel opens |

If any shortcut is captured by the stream instead of the browser, the keyboard patch in `web-viewer/Dockerfile` may need updating. Rebuild the image after changes:

```bash
docker compose -p isim -f docker-compose.yml up --build -d
```

## 5. Multi-instance test

Deploy two instances on separate GPUs with different ports:

```bash
# Instance 1 on GPU 0
GPU_DEVICE=0 ISAACSIM_SIGNAL_PORT=49100 WEB_VIEWER_PORT=8210 \
ISAAC_SIM_DATA=~/docker/isaac-sim-1 \
  docker compose -p isim1 -f docker-compose.yml up --build -d

# Instance 2 on GPU 1
GPU_DEVICE=1 ISAACSIM_SIGNAL_PORT=49200 WEB_VIEWER_PORT=8211 \
ISAAC_SIM_DATA=~/docker/isaac-sim-2 \
  docker compose -p isim2 -f docker-compose.yml up --build -d
```

Verify each instance sees only its assigned GPU:

```bash
docker exec isim1-isaac-sim-1 nvidia-smi -L   # GPU 0 only
docker exec isim2-isaac-sim-1 nvidia-smi -L   # GPU 1 only
```

Open both URLs and confirm independent streams.

## 6. Tear down

```bash
# Single instance
docker compose -p isim -f docker-compose.yml down

# Multi-instance
docker compose -p isim1 -f docker-compose.yml down
docker compose -p isim2 -f docker-compose.yml down
```
