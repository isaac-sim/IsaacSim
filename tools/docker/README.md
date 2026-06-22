# Docker Build Tools

This directory contains scripts for building Docker images of Isaac Sim. The build process involves two main steps: preparing the build environment and building the Docker image.

## Contents

- [Prerequisites](#prerequisites)
  - [Installing Prerequisites](#installing-prerequisites)
  - [Clone the Repository](#clone-the-repository)
  - [Verify Compiler Version](#verify-compiler-version)
- [Build Process](#build-process)
- [Example Usage](#example-usage)
- [Environment Variables for `docker run`](#environment-variables-for-docker-run)
- [Ports for streaming](#ports-for-streaming)
- [Docker Compose (Isaac Sim + Web Viewer)](#docker-compose-isaac-sim--web-viewer)
  - [Quick start](#quick-start)
  - [Using a prebuilt Isaac Sim image](#using-a-prebuilt-isaac-sim-image)
  - [Checking status and endpoints](#checking-status-and-endpoints)
  - [Rebuilding the web viewer](#rebuilding-the-web-viewer)
  - [Multiple instances with dedicated GPUs](#multiple-instances-with-dedicated-gpus)
- [Hub Workstation Cache](#hub-workstation-cache)
- [Cloud Deployment (AWS, GCP, Azure)](#cloud-deployment-aws-gcp-azure)
  - [Retrieving the VM's public IP](#retrieving-the-vms-public-ip)
  - [Launching with the public IP](#launching-with-the-public-ip)
  - [Restricting access with firewall rules](#restricting-access-with-firewall-rules)
- [Important Notes](#important-notes)
- [Keyboard Shortcuts (Web Viewer)](#keyboard-shortcuts-web-viewer)
- [Troubleshooting](#troubleshooting)

## Prerequisites

Before running these scripts, ensure you have the following installed on your host machine:

- **Git** - For version control and repository management
- **Git LFS** - For managing large files within the repository
- **build-essential** - Includes `make` and other essential build tools
- **GCC/G++ 11** - Required compiler version (GCC/G++ 12+ is not supported)
- **rsync** - Required for file synchronization during the preparation phase
- **python3** - Required for running the preparation scripts and installing dependencies
- **Docker** - Required for building the final image
- **NVIDIA Container Toolkit** - Required for GPU access inside the container when running the image
- **GPU** - See [NVIDIA GPU Requirements](https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html)
- **Driver** - See [NVIDIA Driver Requirements](https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html)
- **Internet Access** - Required for downloading the Omniverse Kit SDK, extensions, and tools

See also: [Container Setup](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html#container-setup) in the Isaac Sim documentation.

### Installing Prerequisites

On Ubuntu/Debian:

```bash
sudo apt-get update
sudo apt-get install -y git git-lfs build-essential rsync python3 docker.io
```

**GCC/G++ 11** (required — higher versions are not supported):

```bash
sudo apt-get install -y gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
```

> **Ubuntu 24.04 ⚠️**
> Ubuntu 24.04 ships with GCC/G++ 12+ by default. You must install and select GCC/G++ 11 using the commands above.

**Additional packages for aarch64 hosts** (e.g. DGX Spark): Some Python packages such as `imgui-bundle` do not publish pre-built wheels for `linux_aarch64`, so pip builds them from source. This requires X11 development headers:

```bash
sudo apt-get install -y libx11-dev xorg-dev
```

**NVIDIA Container Toolkit** (required for running the built image with GPU support):

```bash
# Configure the repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && sudo apt-get update

# Install the NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify (optional) — uses the public NGC CUDA base image to avoid Docker Hub anonymous pull rate limits (HTTP 429)
docker run --rm --runtime=nvidia --gpus all nvcr.io/nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
```

On other systems, install these packages using your system's package manager. For full details and alternatives, see [Container Setup](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html#container-setup).

### Clone the Repository

```bash
git clone -b main https://github.com/isaac-sim/IsaacSim.git isaacsim
cd isaacsim
git lfs install
git lfs pull
```

### Verify Compiler Version

Confirm that GCC/G++ 11 is being used before building:

```bash
gcc --version
g++ --version
```

## Build Process

### Step 1: Prepare the Build Environment

Use `prep_docker_build.sh` to prepare the Docker build context:

```bash
./tools/docker/prep_docker_build.sh [OPTIONS]
```

#### Options:

- `--build` - Run the full Isaac Sim build sequence before preparing Docker files:
  - Executes `build.sh -r`
- `--x86_64` - Build x86_64 container (default)
- `--aarch64` - Build aarch64 container
- `--skip-dedupe` - Skip the file deduplication process (faster but larger image)
- `-h, --help` - Show help message

#### What this script does:

1. **Build Verification**: Checks that `_build/$CONTAINER_PLATFORM/release` exists (required for Docker build)
2. **Dependency Installation**: Installs Python requirements from `tools/docker/requirements.txt`
3. **File Preparation**: Generates and runs an rsync script to copy necessary files to `_container_temp`
4. **Data Copying**: Copies additional data from `tools/docker/data` and `tools/docker/oss`
5. **Deduplication**: Finds duplicate files and replaces them with symlinks to reduce image size
6. **Symlink Cleanup**: Fixes any chained symlinks that may have been created

### Step 2: Build the Docker Image

Use `build_docker.sh` to build the actual Docker image:

```bash
./tools/docker/build_docker.sh [OPTIONS]
```

#### Options:

- `--tag TAG` - Specify the Docker image tag (default: `isaac-sim-docker:latest`)
- `--x86_64` - Build for x86_64 platform (default)
- `--aarch64` - Build for arm64 platform
- `--push` - Push docker image tag
- `-h, --help` - Show help message

## Example Usage

### Basic build:

```bash
# Prepare build environment (includes full build)
./tools/docker/prep_docker_build.sh --build

# Build Docker image with default tag
./tools/docker/build_docker.sh
```

### Custom tag:

```bash
# Prepare build environment
./tools/docker/prep_docker_build.sh --build

# Build with custom tag
./tools/docker/build_docker.sh --tag my-isaac-sim:v1.0
```

### Quick rebuild (skip deduplication):

```bash
# If you've already built once and want to rebuild quickly
./tools/docker/prep_docker_build.sh --skip-dedupe

# Build the image
./tools/docker/build_docker.sh
```

## Environment Variables for `docker run`

You can pass these environment variables when running the container (e.g. `docker run --rm -e VAR=value ...`) to control the entrypoint behavior. **ACCEPT_EULA** is required to run the container; the rest are optional. Optional flags are only added when the corresponding variable is set.


| Variable                 | Required? | Description                                                                                                                 |
| ------------------------ | --------- | --------------------------------------------------------------------------------------------------------------------------- |
| **ACCEPT_EULA**          | **Yes**   | Accept the license agreement (required to run; set e.g. to `Y`).                                                            |
| **OMNI_SERVER**          | No        | Override the default asset root (passed as `--/persistent/isaac/asset_root/default`).                                       |
| **ISAACSIM_HOST**        | No        | Public or private IP of the host for livestream (passed as livestream `publicIp`). Default: `127.0.0.1`. See [Cloud Deployment](#cloud-deployment-aws-gcp-azure) for cloud VMs. |
| **ISAACSIM_SIGNAL_PORT** | No        | Signal port for WebRTC streaming. Default: `49100`.                                                                         |
| **ISAACSIM_STREAM_PORT** | No        | Streaming port for WebRTC. Default: `47998`.                                                                                |


### Minimal run:

```bash
docker run --rm -it --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  isaac-sim-docker:latest
```

Alternatively, use the provided helper script which drops into a bash shell inside the container:

```bash
./tools/docker/run_docker.sh
```

### Run with volume mounts:

```bash
# Create cache/log mounts (optional; use uid 1234 to match container user)
mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
mkdir -p ~/.cache/ov/hub
sudo chown -R 1234:1234 ~/docker ~/.cache/ov/hub

docker run --name isaac-sim --rm -it --gpus all --network=host \
  -e ACCEPT_EULA=Y \
  -e ISAACSIM_HOST=<host-ip> \
  -e ISAACSIM_SIGNAL_PORT=<signal-port> \
  -e ISAACSIM_STREAM_PORT=<stream-port> \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v ~/.cache/ov/hub:/var/cache/hub:rw \
  -u 1234:1234 \
  isaac-sim-docker:latest
```

**`--network=host` is required for WebRTC livestreaming.** The NVIDIA streaming SDK binds its UDP media socket to the `ISAACSIM_HOST` address, which must be a real network interface inside the container. Docker bridge networking (`-p` port publishing) does not satisfy this requirement because the host IP is not available inside the container's network namespace.

Replace `<host-ip>` with the host's LAN/public IP (e.g. `192.168.1.100`), `<signal-port>` with the desired signaling port (default `49100`), and `<stream-port>` with the desired streaming port (default `47998`). With `--network=host`, no `-p` flags are needed -- the container shares the host's network directly.

Open the ports listed below (e.g. UFW) if the host firewall is active.

### Why not Docker bridge networking (`-p`)?

The NVIDIA streaming SDK binds its UDP media socket to the IP specified by `ISAACSIM_HOST`. With Docker bridge networking, this IP does not exist inside the container (the container only has a `172.17.x.x` address), so the UDP bind fails and no media stream is established. Signaling (TCP 49100) may connect, but the actual video stream will not. **Use `--network=host` for livestreaming.**

## Ports for streaming

When using `--network=host`, the container shares the host's network. If the host firewall is active (e.g. UFW), allow these ports:


| Port      | Protocol | Purpose                          |
| --------- | -------- | -------------------------------- |
| **8210**  | TCP      | Web viewer (Docker Compose only) |
| **49100** | TCP      | WebRTC signal (signaling)        |
| **47998** | UDP      | WebRTC stream (media)            |


Minimal UFW rules:

```bash
sudo ufw allow 8210/tcp  comment 'Web viewer'
sudo ufw allow 49100/tcp comment 'Isaac Sim WebRTC signal'
sudo ufw allow 47998/udp comment 'Isaac Sim WebRTC stream'
sudo ufw reload
```

If you override ports via `ISAACSIM_SIGNAL_PORT`, `ISAACSIM_STREAM_PORT` or `WEB_VIEWER_PORT`, open those instead.

## Docker Compose (Isaac Sim + Web Viewer)

A `docker-compose.yml` is provided that launches both Isaac Sim (headless streaming) and a WebRTC web-viewer container side by side. The web viewer is built from `@nvidia/create-ov-web-rtc-app` in local streaming mode and connects the browser directly to the Isaac Sim WebRTC endpoints.

> **Support note:** Docker Compose web viewer deployment is supported only on Ubuntu hosts and DGX Spark systems. Windows hosts, including WSL, are not supported.

> **Security notice:** Isaac Sim and the web viewer are designed for use on private/trusted networks. They do not include authentication or encryption. Do **not** expose them on the public Internet without additional safeguards. If you need remote access, restrict the ports with firewall rules or add a reverse proxy with HTTPS/TLS and authentication (e.g. nginx with SSL certificates and basic auth). See [Cloud Deployment](#cloud-deployment-aws-gcp-azure) for cloud-specific guidance. Users are responsible for securing any public-facing deployments.

### Quick start

```bash
# Create cache/log mounts (use uid 1234 to match container user)
mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
mkdir -p ~/.cache/ov/hub
sudo chown -R 1234:1234 ~/docker ~/.cache/ov/hub

# 1. Build the Isaac Sim image (existing workflow)
./tools/docker/prep_docker_build.sh --build --x86_64
./tools/docker/build_docker.sh --x86_64

# 2. Launch both services (logs stream to the terminal; Ctrl+C stops everything)
docker compose -p isim -f tools/docker/docker-compose.yml up --build

# Or, launch in detached mode (runs in the background)
docker compose -p isim -f tools/docker/docker-compose.yml up --build -d
```

> **Note:** On DGX Spark, use `--aarch64` instead of `--x86_64` in the build commands above.

Running without `-d` (foreground) streams combined logs from both containers to your terminal, which is useful for debugging. Press `Ctrl+C` to stop all services. Running with `-d` (detached) starts everything in the background.

On first run, Docker Compose will build the `web-viewer` image automatically. The web viewer waits for the Isaac Sim healthcheck (`AppReady` in logs) before starting.

> **Single browser connection:** Only one browser tab/window can be connected to Isaac Sim at a time. If a browser is already connected to the streaming session, other browsers or tabs will not be able to connect until that session is closed.

### Using a prebuilt Isaac Sim image

By default the compose file uses the locally built `isaac-sim-docker:latest` image. To use a prebuilt NGC image instead, set `ISAAC_SIM_IMAGE`:

```bash
ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1 docker compose -p isim -f tools/docker/docker-compose.yml up --build -d
```

This skips the local Isaac Sim build steps (`prep_docker_build.sh` and `build_docker.sh`).

### Checking status and endpoints

```bash
docker compose -p isim logs              # combined logs from both containers
docker compose -p isim logs web-viewer   # web viewer only (shows the URL)
docker compose -p isim logs isaac-sim    # Isaac Sim only (look for "app ready")
docker compose -p isim logs -f           # follow live logs (Ctrl+C to stop)
```

### Environment variables

Override any variable via the shell or a `.env` file next to `docker-compose.yml`:


| Variable                 | Default                      | Description                                                              |
| ------------------------ | ---------------------------- | ------------------------------------------------------------------------ |
| **ISAAC_SIM_IMAGE**      | `isaac-sim-docker:latest`    | Docker image to run. Set to a prebuilt NGC image (e.g. `nvcr.io/nvidia/isaac-sim:6.0.1`) to skip local build steps. |
| **ISAACSIM_HOST**        | `127.0.0.1`                  | Host IP for WebRTC streaming (used by both services). See [Cloud Deployment](#cloud-deployment-aws-gcp-azure) for cloud VMs. |
| **ISAACSIM_SIGNAL_PORT** | `49100`                      | WebRTC signaling port (TCP)                                              |
| **ISAACSIM_STREAM_PORT** | `47998`                      | WebRTC media port (UDP)                                                  |
| **WEB_VIEWER_PORT**      | `8210`                       | Host port for the web viewer                                             |
| **GPU_DEVICE**           | `all`                        | GPU index to pin the Isaac Sim container to (e.g. `0`, `1`)             |
| **ISAAC_SIM_DATA**       | `~/docker/isaac-sim`         | Host path for persistent cache, config, logs, and data. Use a full absolute path in `.env` files (`~` is not expanded by Docker Compose). |


### Rebuilding the web viewer

The web viewer bakes `ISAACSIM_HOST`, `ISAACSIM_SIGNAL_PORT`, and `ISAACSIM_STREAM_PORT` into the JavaScript bundle at build time. If you change any of these values (e.g. in a `.env` file), add `--build` when bringing the stack up so the web viewer image is rebuilt and the browser receives the updated values:

```bash
docker compose -p isim -f tools/docker/docker-compose.yml up --build -d
```

### Stopping

```bash
docker compose -p isim -f tools/docker/docker-compose.yml down
```

### Multiple instances with dedicated GPUs

You can run multiple Isaac Sim instances on the same host by using Docker Compose project names (`-p`) to namespace each deployment. Each instance needs unique ports and its own data directory. Use `GPU_DEVICE` to pin each instance to a dedicated GPU.

```bash
# Prepare separate data directories (one per instance, owned by uid 1234)
mkdir -p ~/docker/isaac-sim-1/{cache/main,cache/computecache,config,data,logs,pkg}
mkdir -p ~/docker/isaac-sim-2/{cache/main,cache/computecache,config,data,logs,pkg}
mkdir -p ~/.cache/ov/hub
sudo chown -R 1234:1234 ~/docker ~/.cache/ov/hub

# Instance 1 — GPU 0, default ports, web viewer on 8210
GPU_DEVICE=0 \
ISAACSIM_SIGNAL_PORT=49100 \
ISAACSIM_STREAM_PORT=47998 \
WEB_VIEWER_PORT=8210 \
ISAAC_SIM_DATA=~/docker/isaac-sim-1 \
  docker compose -p isim1 -f tools/docker/docker-compose.yml up --build -d

# Instance 2 — GPU 1, custom ports, web viewer on 8211
GPU_DEVICE=1 \
ISAACSIM_SIGNAL_PORT=49200 \
ISAACSIM_STREAM_PORT=48100 \
WEB_VIEWER_PORT=8211 \
ISAAC_SIM_DATA=~/docker/isaac-sim-2 \
  docker compose -p isim2 -f tools/docker/docker-compose.yml up --build -d
```

Check status and endpoints for each instance:

```bash
docker compose -p isim1 logs
docker compose -p isim2 logs
```

Each instance sees only its assigned GPU. Verify with:

```bash
docker exec isim1-isaac-sim-1 nvidia-smi -L   # GPU 0 only
docker exec isim2-isaac-sim-1 nvidia-smi -L   # GPU 1 only
```

To stop a specific instance:

```bash
docker compose -p isim1 -f tools/docker/docker-compose.yml down
docker compose -p isim2 -f tools/docker/docker-compose.yml down
```

## Hub Workstation Cache

[Hub Workstation Cache](https://docs.omniverse.nvidia.com/utilities/latest/cache/hub-workstation.html) is a service that speeds up USD workflows by caching storage-derived data locally. When running Isaac Sim in a container, Hub should also run as a container on the same host so that all Kit-based clients can benefit from the shared cache.

> **Note:** Hub Workstation Cache is designed for **local workstation use only** — for example, bare-metal runs or containers on a local workstation. It is not intended for multi-user servers or cloud deployments. For distributed or cloud caching, see [Derived Data Cache Service (DDCS)](https://docs.nvidia.com/cloud-functions/current/latest/ddcs.html).

The Hub Workstation Cache image is public and can be pulled without logging in to `nvcr.io`. If your Docker client is already logged in to `nvcr.io`, NGC checks whether the governing terms have been accepted for your NGC organization before allowing the pull.

Before pulling the Hub image with Docker credentials, open the [Hub Workstation Cache container page](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/omniverse/containers/hub_workstation_cache?version=2.0.0) in a browser, sign in to NGC, select version `2.0.0`, and accept the governing terms. NGC requires terms acceptance once per NGC organization. For the official NGC procedure, see [Accepting Terms Before Downloading](https://docs.nvidia.com/ngc/latest/ngc-catalog-user-guide.html#accepting-terms-before-downloading).

If Docker reports `DENIED` with `Please accept license on the browser to be able to download`, either accept the terms in the browser for the same NGC organization used by `docker login nvcr.io`, or log out of `nvcr.io` before pulling this public image anonymously:

```bash
docker logout nvcr.io
docker pull nvcr.io/nvidia/omniverse/hub_workstation_cache:2.0.0
```

![Hub Workstation Cache NGC Governing Terms panel with an Accept Terms button](../../docs/isaacsim/images/isim_6.0_full_ref_gui_hub_workstation_cache_accept_terms.png)

Start the Hub container **before** launching Isaac Sim:

```bash
mkdir -p ~/.cache/ov/hub
sudo chown -R 1234:1234 ~/.cache/ov/hub
docker run --name hub-cache --rm -d --network=host \
  -v ~/.cache/ov/hub:/var/cache/hub:rw \
  -u 1234:1234 \
  nvcr.io/nvidia/omniverse/hub_workstation_cache:2.0.0
```

Once the container is running, the Hub settings UI is available at `http://localhost:14090/index.html`.

The Isaac Sim container is already configured to discover Hub at runtime via the following environment variables baked into the image:

| Variable                  | Value                 | Purpose                                                    |
| ------------------------- | --------------------- | ---------------------------------------------------------- |
| `HUB__CACHE__PATH`        | `/var/cache/hub`      | Tells the local Hub executable where to find the cache     |
| `HUB__ARGS__DETECT_ONLY`  | `true`                | Prevents the client from starting its own Hub instance     |
| `OMNICLIENT_HUB_EXE`     | `/usr/local/bin/hub`  | Path to the Hub executable used for client coordination    |

The `~/.cache/ov/hub` volume mount in the Isaac Sim `docker run` examples maps the same host directory into both containers so they share the cache. `--network=host` is required so the Hub client inside Isaac Sim can reach the Hub service on `localhost`.

For more details, see the [Hub as a Docker Container](https://docs.omniverse.nvidia.com/utilities/latest/cache/hub-workstation.html#hub-as-a-docker-container) documentation.

## Cloud Deployment (AWS, GCP, Azure)

When running Isaac Sim on a cloud VM (e.g. AWS EC2, GCP Compute Engine, Azure VM), you must set `ISAACSIM_HOST` to the VM's public IP so the WebRTC stream is reachable from your browser, and restrict the ports with firewall rules so the unauthenticated stream is not exposed to the open Internet.

### Retrieving the VM's public IP

Cloud VMs do not have their public IP assigned to a local network interface — commands like `hostname -I` only show the private IP. You must retrieve the public IP yourself and pass it explicitly via `ISAACSIM_HOST`.

```bash
# AWS EC2 (IMDSv2)
PUBLIC_IP=$(TOKEN=$(curl -s -X PUT "http://169.254.169.254/latest/api/token" \
  -H "X-aws-ec2-metadata-token-ttl-seconds: 21600") && \
  curl -s -H "X-aws-ec2-metadata-token: $TOKEN" \
  http://169.254.169.254/latest/meta-data/public-ipv4)
echo "$PUBLIC_IP"

# GCP
PUBLIC_IP=$(curl -s -H "Metadata-Flavor: Google" \
  http://metadata.google.internal/computeMetadata/v1/instance/network-interfaces/0/access-configs/0/external-ip)

# Azure
PUBLIC_IP=$(curl -s -H Metadata:true \
  "http://169.254.169.254/metadata/instance/network/interface/0/ipv4/ipAddress/0/publicIpAddress?api-version=2021-02-01&format=text")

# Or simply check the cloud console for the instance's public IP.
```

### Launching with the public IP

```bash
ISAACSIM_HOST=$PUBLIC_IP docker compose -p isim -f tools/docker/docker-compose.yml up --build -d
```

Then open `http://<PUBLIC_IP>:8210` in your browser (the same IP you set in `ISAACSIM_HOST`).

### Restricting access with firewall rules

Before launching, lock down the Isaac Sim ports so only your client IP can reach them.

**AWS Security Group** — allow inbound only from your IP:


| Port      | Protocol | Source         |
| --------- | -------- | -------------- |
| **8210**  | TCP      | `<your-ip>/32` |
| **49100** | TCP      | `<your-ip>/32` |
| **47998** | UDP      | `<your-ip>/32` |


**GCP Firewall Rule:**

```bash
gcloud compute firewall-rules create allow-isaacsim \
  --allow tcp:8210,tcp:49100,udp:47998 \
  --source-ranges <your-ip>/32 \
  --target-tags isaacsim
```

**Azure NSG:** Create inbound rules for the same ports restricted to your client IP.

> **Never** use `0.0.0.0/0` (all traffic) for these ports in production. Doing so exposes an unauthenticated stream to anyone on the Internet.

## Important Notes

- **Build Requirements**: The `_build/$CONTAINER_PLATFORM/release` directory must exist before running the Docker preparation. Use `--build` option if you haven't built Isaac Sim yet.
- **Deduplication**: The deduplication process can significantly reduce Docker image size by replacing duplicate files with symlinks, but it takes time. Use `--skip-dedupe` for faster rebuilds during development.
- **File Paths**: The deduplication process skips files with spaces in their paths for reliability.
- **Build Context**: The final Docker build uses `_container_temp` as the build context and `tools/docker/Dockerfile` as the Dockerfile.
- **Platform**: Add the `--aarch64` flag to build for arm64 platform. It is recommended to use this flag when on an arm64 host.

## Keyboard Shortcuts (Web Viewer)


| Action                         | Windows / Linux         | Mac                              |
| ------------------------------ | ----------------------- | -------------------------------- |
| Copy / paste                   | **Ctrl+C** / **Ctrl+V** | **Ctrl+C** / **Ctrl+V (in app)** |
| Refresh the browser page       | **F5** or **Ctrl+R**    | **Fn+F5** or **Cmd+R**           |
| Maximize viewport in Isaac Sim | **F7**                  | **Fn+F7**                        |
| Toggle browser fullscreen      | **F11**                 | **Shift+Fn+F11**                 |
| Open DevTools                  | **F12**                 | **Fn+F12** or **Cmd+Option+I**   |


## Troubleshooting

- **Cannot connect to livestream / black screen / ports not listening**: (1) Confirm the Isaac Sim app reached the loaded state. (2) Ensure you are using `--network=host` (required for WebRTC streaming). (3) Set `ISAACSIM_HOST` to the IP address the client uses to reach the host (e.g. LAN IP or cloud public IP). (4) Allow ports 8210/tcp, 49100/tcp, and 47998/udp in the host firewall (e.g. UFW). Opening TCP ports alone is not sufficient because WebRTC media uses UDP 47998.
- **Hub startup or connectivity issues after a previous test**: Restart the Hub container from the [Hub Workstation Cache](#hub-workstation-cache) section, then retry Docker Compose.
- **Stale volume mounts causing issues (e.g. crashes, config errors, or livestream failures)**: Old cached data in the Docker volume mount directories can cause unexpected behavior. Remove the existing mounts and recreate them:
  ```bash
  sudo rm -rf ~/docker
  mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
  sudo rm -rf ~/.cache/ov/hub
  mkdir -p ~/.cache/ov/hub
  sudo chown -R 1234:1234 ~/docker ~/.cache/ov/hub
  ```
- **Docker image starts with missing module or plugin errors after an interrupted build, failed extraction, or disk-full event**: Remove the generated Docker build context and prepare the image again:
  ```bash
  rm -rf _container_temp
  ./tools/docker/prep_docker_build.sh --x86_64
  ./tools/docker/build_docker.sh --x86_64
  ```
  If the build output itself may be incomplete, remove `_build` as well and rebuild before preparing the Docker context:
  ```bash
  rm -rf _build _container_temp
  ./tools/docker/prep_docker_build.sh --build --x86_64
  ./tools/docker/build_docker.sh --x86_64
  ```
- **Second browser or tab cannot connect**: Only one browser connection to Isaac Sim is supported at a time. Close the existing browser tab or window that is connected to the web viewer, then open the URL again in a single tab.
- **Clipboard (Ctrl+C/V) not working in the web viewer**: The browser Clipboard API requires a secure context. When accessing the web viewer over HTTP from a non-localhost address, clipboard forwarding to Isaac Sim is blocked. To enable it, open `chrome://flags/#unsafely-treat-insecure-origin-as-secure` in Chrome, add your web viewer URL (e.g. `http://192.168.1.100:8210`), and relaunch the browser.
- **Error: "_build/$CONTAINER_PLATFORM/release does not exist"**: Run the script with `--build` option to build Isaac Sim first.
- **rsync not found**: Install rsync using your system's package manager.
- **Python requirements installation fails**: Ensure python3 and pip are properly installed.
- **Docker build fails**: Check that Docker daemon is running and you have sufficient disk space.
- **ERROR: This host's buildx builder does NOT support linux/arm64**: The current Docker buildx builder does not list `linux/arm64`. On DGX Spark (native aarch64), use the default builder so it uses the host platform:
  ```bash
  docker buildx use default
  docker buildx inspect --bootstrap
  ./tools/docker/build_docker.sh --aarch64
  ```
  If you are on an x86_64 host and cross-building for arm64, enable QEMU and create a multi-platform builder:
  ```bash
  docker run --privileged --rm tonistiigi/binfmt --install all
  docker buildx create --name multiarch --driver docker-container --use
  docker buildx inspect --bootstrap
  ./tools/docker/build_docker.sh --aarch64
  ```
