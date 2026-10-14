# Permitted downloads and licenses

This skill may fetch **only** NVIDIA-published artifacts already used by Isaac Sim Docker Compose web viewer. Do **not** download extra software on the host or into the web-viewer image.

**Allowed**

| What | Source | License |
|---|---|---|
| Isaac Sim container | `nvcr.io/nvidia/isaac-sim:<tag>` only | NVIDIA Isaac Sim EULA (not this skill's Apache-2.0) |
| Web-viewer base image | `nvcr.io/nvidia/base/ubuntu:noble-20251013` (Dockerfile `FROM`) | NGC Ubuntu base, same as Isaac Sim `tools/docker/web-viewer/Dockerfile` |
| Web-viewer scaffold | NVIDIA npm `@nvidia/create-ov-web-rtc-app@2.1.5` from `edge.urm.nvidia.com` … `omniverse-client-npm` | NVIDIA Omniverse client packages |
| Web-viewer Node.js | NodeSource Node 22 **inside the image build only**, matching that product Dockerfile | Do not add packages beyond the product file |
| Tag lookup | NGC catalog tags page via `scripts/resolve_ngc_isaac_sim_image.py` | NVIDIA catalog HTML |

Scripts refuse any `--image` that is not `nvcr.io/nvidia/isaac-sim:<tag>`.

**Forbidden (stop; do not install a workaround)**

- Host `apt-get` / `yum` / `dnf` / `pip` / `npm` / `npx` / `conda` / `snap`
- `curl | bash` or `wget` installers on the host
- Docker Hub, GHCR, Quay, or public images (`ubuntu:`, `node:`, `nginx:`, …)
- Installing Chromium, Chrome, Node.js, git, or other tools on the host
- `git clone` of third-party software
- Extra apt/npm lines in `docker/web-viewer/Dockerfile`
- Treating Isaac Sim as Apache-2.0. This skill is Apache-2.0; the simulator is not.

**EULA (mandatory before pull)**

Present these URLs and wait for an explicit yes:

- https://docs.omniverse.nvidia.com/eula
- https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/

> Running Isaac Sim in Docker acknowledges the NVIDIA Isaac Sim / Omniverse EULA at the URLs above on your behalf (`ACCEPT_EULA=Y`). Do you agree? (yes/no)

If no, stop. Do not pull, build, or `compose up`. Dry-run, preflight, and NGC catalog tag lookup may run without this yes.
