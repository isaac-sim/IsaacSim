# Docker Image Installation

Use this workflow only on native Linux x86_64 or supported DGX Spark Linux aarch64 to install the Isaac Sim Docker image into Docker-managed storage. The same public image tag supports both architectures. Windows and WSL hosts are unsupported; stop before preflight or dry-run planning and offer Standalone binary or Python package instead. On aarch64, require the system preflight and Compatibility Checker to confirm the DGX Spark environment.

Current public image:

```text
nvcr.io/nvidia/isaac-sim:6.2.0
```

## Installation boundary

- Pull the image with `docker pull`.
- Inspect and report the image reference, immutable image ID, and Docker root directory.
- Do not use `docker run`, create a container, prepare runtime mounts, publish ports, invoke `warmup.sh`, or invoke `runheadless.sh`.
- Image layers are managed by Docker; do not claim that a Docker image has a normal extracted host directory.

Dry-run:

```bash
python3 scripts/install_container.py --image nvcr.io/nvidia/isaac-sim:6.2.0
```

Execute only after the preflight, compatibility, EULA, and plan-approval gates:

```bash
python3 scripts/install_container.py \
  --image nvcr.io/nvidia/isaac-sim:6.2.0 \
  --execute
```

The successful result prints:

- image reference;
- immutable image ID;
- Docker-managed storage root;
- confirmation that no container was created or started.

## Pre-install checks

Run the Docker GPU preflight:

```bash
python3 scripts/preflight.py --docker-gpu-check
```

Run the Compatibility Checker as a separately approved gate:

```bash
python3 scripts/validate_install.py container
python3 scripts/validate_install.py container --execute
```

The Compatibility Checker creates a temporary container solely for compatibility testing. It is not part of image installation and must finish before the installer is executed.

## Security

- Use images under `nvcr.io/nvidia/` or `nvcr.io/nvidia-omniverse/`.
- Require explicit EULA acknowledgment before pulling the Isaac Sim image.
- Follow NVIDIA’s official Container Toolkit installation guide when setup is required: <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>.
