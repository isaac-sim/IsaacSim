---
name: isaac-sim-ros-workspaces
description: "Clone, configure, or build IsaacSim-ros_workspaces using native ROS, Docker, custom interfaces, or Pixi. Use only for Isaac Sim ROS workspace requests, not generic workspace setup."
license: Apache-2.0
metadata:
  author: Krishna Patil <info@nvidia.com>
---

# Isaac Sim ROS Workspaces

## Purpose

Build the [IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces)
repository and install its workspace dependencies. This skill never installs a public ROS 2
distribution.

Use this skill for requests to clone, configure, or build `humble_ws` or `jazzy_ws`, including
native colcon, Docker container, Python 3.12 custom-interface, and Pixi workflows.

Require Isaac Sim context together with ROS workspace setup/build intent. A generic request such
as “set up a workspace” must not trigger this skill.

Do not use it for ROS 2 bridge graphs, topics, Nav2, or namespacing; use
`isaac-sim-ros2-bridge` for those tasks.

## Security

This skill installs build tooling and clones/builds a ROS workspace on the developer machine.
That is intentional.

- **Privilege**: native builds may call `sudo` for missing apt packages or one-time `rosdep init`
  only after dependency checks. Prefer an interactive terminal or pre-authenticated credentials.
- **Clean reclone**: `--clean-reclone` / `-CleanReclone` deletes a resolved git checkout after
  refusing unsafe paths (filesystem root, home directory, reparse points). On a TTY the script
  prints the exact path and requires typing `y` before deletion; non-interactive runs also need
  `--yes-i-know` / `-YesIKnow`. Confirm the target path before enabling clean reclone.
- **Pixi**: when missing, Linux `--method pixi` downloads the official GitHub release
  binary for the host arch and verifies its `.sha256` sidecar with `sha256sum` before
  installing to `~/.pixi/bin`. It does not run a remote shell installer.
- **Docker**: workspace *builds* use default container networking. Host-network mode is only for
  *runtime* DDS containers the user launches separately — not applied by the build script.

## Prerequisites

- Ubuntu 22.04 or 24.04 on `x86_64`/`aarch64`, or Windows 11 x64.
- Git available on the host.
- Native Linux builds: matching ROS 2 already installed (or pass `--ros-setup`). The script
  checks build tools first and uses sudo only for missing packages or one-time `rosdep init`.
- Docker/custom builds: Docker CE installed, daemon running, and current-user access.
- Windows Pixi: MSVC Build Tools 2022 with Desktop development with C++; `winget` available.
- Linux Pixi (explicit only): selected workspace must contain `pixi.toml`. Pixi is
  auto-installed from the official GitHub release (checksum-verified) when absent.

If Docker is missing or inaccessible, stop. Do not install Docker automatically. Tell the user
to install Docker CE, start its daemon, add their user to the `docker` group, then re-login or
run `newgrp docker` and retry.

## Supported configurations

| Platform | Configuration | Default method |
|---|---|---|
| Ubuntu 24.04, x86_64/aarch64 | ROS 2 Jazzy | Native when installed; otherwise Docker |
| Ubuntu 22.04, x86_64/aarch64 | ROS 2 Humble | Native when installed; otherwise Docker |
| Ubuntu 24.04, x86_64/aarch64 | Humble workspace via Docker | Prebuilt Humble image |
| Ubuntu 22.04, x86_64/aarch64 | Jazzy workspace via Docker | Prebuilt Jazzy image |
| Ubuntu 22.04, x86_64/aarch64 | Humble or Jazzy custom interfaces | Repository `build_ros.sh` |
| Windows 11 x64 | ROS 2 Jazzy | Pixi |

Linux Pixi is available only when the user explicitly asks for it. Cross-distro native builds
(Humble on Ubuntu 24.04, Jazzy on Ubuntu 22.04) must stop and use Docker instead. Unsupported or
deprecated flows—including Windows Humble and WSL2—must stop with a clear message.

See [build matrix](references/build-matrix.md) for prerequisites, images, and outputs. See
[prompt catalog](references/prompt-catalog.md) for routing examples.

## Critical Docker distinction

There are two different Docker-backed workflows. Never substitute one for the other.

### Container workspace build

Use `scripts/setup_ros_workspace.sh --method docker`.

- Trigger: the user says Docker/container, or Linux has no native ROS 2 and no method was given.
- Uses a prebuilt ROS image (`osrf/ros:*` or `arm64v8/ros:*`).
- Mounts `<distro>_ws`, installs package dependencies with `rosdep`, and runs `colcon build`.
- Produces `<repo>/<distro>_ws/install`.
- Intended for tutorial packages and external ROS nodes.

### Custom-interface Python 3.12 build

Use `scripts/setup_ros_workspace.sh --method custom`.

- Trigger: the user says custom ROS interfaces/messages, Python 3.12, or `build_ros.sh`.
- Calls the cloned repository's `./build_ros.sh -d <distro> -v 22.04`.
- Builds repository Dockerfiles rather than starting a prebuilt ROS desktop image.
- Produces `build_ws/<distro>/<distro>_ws` and
  `build_ws/<distro>/isaac_sim_ros_ws`.
- Intended for generated Python ROS interfaces loaded by Isaac Sim's Python 3.12 runtime.

If a prompt mentions both Docker and custom interfaces/Python 3.12, choose `custom`. If it
only says "build the workspace in Docker", choose `docker`.

## Routing

Resolve an explicit user method before inspecting defaults:

1. `custom interfaces`, `custom messages`, `Python 3.12`, or `build_ros.sh` → `custom`.
2. Explicit `Docker` or `container` → `docker`.
3. Explicit `Pixi` → `pixi`.
4. Explicit `native` or `colcon using installed ROS` → `native`.
5. Windows with no method → Windows Pixi.
6. Linux with no method:
   - matching native ROS setup exists → `native`;
   - otherwise → `docker`.

On Linux, let `setup_ros_workspace.sh` perform this detection. It checks the matching
`/opt/ros/<distro>/setup.bash`, or the explicit `--ros-setup` path. Do not use `which ros2` or
`command -v ros2` to conclude ROS is absent: a clean shell can have ROS installed without its
setup file sourced, so the `ros2` command may not yet be on `PATH`.

An internal repository URL, branch, or clone path changes only the clone source—not the build
method.

## Existing checkout policy

Before running a setup script, inspect the target repository path. If a Git checkout already
exists, ask one focused question: reuse the existing checkout, or create a clean clone? State
that a clean clone permanently deletes local and untracked changes.

- If the user chooses reuse, run without a clean flag. Pass the checkout's current branch with
  `--branch` or `-Branch`, or use another repository path when the requested branch differs.
  The scripts warn about a dirty tree and never fetch, pull, switch, reset, or delete it.
- If the user explicitly chooses a clean clone, pass `--clean-reclone` on Linux or
  `-CleanReclone` on Windows. In agent or other non-interactive sessions, also pass
  `--yes-i-know` / `-YesIKnow`. These flags remove only an existing Git checkout after rejecting
  root, home, symbolic-link, and reparse-point paths, then clone the requested URL and branch.
  Interactive terminals still prompt for a typed `y` before deletion.
- If the path exists but is not a Git checkout, stop. Never delete an unrelated occupied path.
- Never infer a clean clone, pass a clean flag automatically, or ask the user to manually
  delete the git checkout.

## Instructions

1. Confirm the request is Isaac Sim ROS workspace setup/build, not bridge/Nav2 work.
2. Resolve method via [Routing](#routing); prefer letting `setup_ros_workspace.sh` auto-detect on Linux.
3. Apply [Existing checkout policy](#existing-checkout-policy) before any clone or clean flag.
4. Run the matching script below; never install public ROS 2 or Docker CE.
5. Report the produced `local_setup.bash` / Pixi workspace path and how to source it.

### Linux auto-selection

```bash
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh
```

The distro defaults to Jazzy on Ubuntu 24.04 and Humble on Ubuntu 22.04. Native ROS is preferred;
Docker is selected only when matching native ROS is absent.

### Linux explicit methods

```bash
# Native ROS already installed
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method native

# Prebuilt ROS image and mounted workspace
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method docker

# Explicit Linux Pixi
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method pixi

# Python 3.12 custom interfaces, Ubuntu 22.04 only
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh \
  --method custom --distro humble
```

Use an internal mirror or release branch:

```bash
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh \
  --repo-url https://git.example.internal/robotics/IsaacSim-ros_workspaces.git \
  --branch release/6.0 --repo-path "$HOME/IsaacSim-ros_workspaces"

# Only after the user explicitly chooses a clean clone
bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh \
  --repo-path "$HOME/IsaacSim-ros_workspaces" --branch main --clean-reclone --yes-i-know
```

Pass `--ros-setup <path>/install/setup.bash` for a source-built ROS installation.

### Windows Jazzy Pixi

Run the skill script with PowerShell `-File`. Do **not** invent separate Bash-wrapped
PowerShell prerequisite one-liners for MSVC, `vswhere`, Git, or Pixi. The `.ps1` already
owns those gates and expands `${env:ProgramFiles(x86)}` correctly.

```powershell
powershell.exe -ExecutionPolicy Bypass -File skills/isaac-sim-ros-workspaces/scripts/setup_pixi_workspace.ps1

powershell.exe -ExecutionPolicy Bypass -File skills/isaac-sim-ros-workspaces/scripts/setup_pixi_workspace.ps1 `
  -RepoUrl "https://git.example.internal/robotics/IsaacSim-ros_workspaces.git" `
  -Branch "release/6.0"

# Only after the user explicitly chooses a clean clone
powershell.exe -ExecutionPolicy Bypass -File skills/isaac-sim-ros-workspaces/scripts/setup_pixi_workspace.ps1 `
  -RepoPath "C:\IsaacSim-ros_workspaces" -Branch main -CleanReclone -YesIKnow
```

Use `-SkipBuild` when you only need the script's prerequisite and clone/prepare steps.
If a manual path probe is unavoidable, use the literal
`C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe` path. Never conclude
MSVC is missing from a single-quoted `${env:ProgramFiles(x86)}` string; PowerShell does not
expand variables inside single quotes.

## Examples

| User request | Method | Command |
|---|---|---|
| Jazzy already installed on Ubuntu 24.04 | Native | `bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method native` |
| ROS not installed; build the tutorial workspace | Docker | `bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method docker` |
| Custom messages for Isaac Sim Python 3.12 | Custom | `bash skills/isaac-sim-ros-workspaces/scripts/setup_ros_workspace.sh --method custom --distro humble` |
| Set up the workspace on Windows 11 | Pixi | `powershell.exe -ExecutionPolicy Bypass -File skills/isaac-sim-ros-workspaces/scripts/setup_pixi_workspace.ps1` |

More routing cases live in [prompt catalog](references/prompt-catalog.md).

## Source built artifacts

Native:

```bash
source /opt/ros/<distro>/setup.bash
source <repo>/<distro>_ws/install/local_setup.bash
```

Container builds must be sourced inside a compatible ROS container. The Docker build itself does
not enable host-network mode or set host `ROS_DOMAIN_ID`; add those only when launching a runtime
container that talks over DDS (pass Docker host-network mode plus `ROS_DOMAIN_ID`):

```bash
docker run --rm -it \
  --env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
  --volume "$HOME/IsaacSim-ros_workspaces/<distro>_ws:/<distro>_ws" \
  <same-image-as-build> \
  bash -lc 'source /opt/ros/<distro>/setup.bash && source /<distro>_ws/install/local_setup.bash && bash'
```

Custom-interface builds use:

```bash
source <repo>/build_ws/<distro>/<distro>_ws/install/local_setup.bash
source <repo>/build_ws/<distro>/isaac_sim_ros_ws/install/local_setup.bash
```

Pixi commands run through `pixi run` or inside `pixi shell`.

## Limitations

- Does not install public ROS 2 distributions or Docker CE.
- Does not support deprecated WSL2 or unsupported distro/platform pairs.
- Native builds require the host-default distro; cross-distro Humble/Jazzy builds use Docker.
- Windows Pixi supports Jazzy only.
- Custom-interface builds support Ubuntu 22.04 only and require Docker.
- Linux Pixi is an explicit opt-in extension; it fails if the selected workspace has no
  `pixi.toml`.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Docker command missing | Docker CE not installed | Install Docker CE and retry |
| Cannot reach Docker daemon | Daemon stopped or broken | Start Docker and verify `docker info` |
| Docker permission denied | User not in docker group | Add user to `docker` group, re-login or run `newgrp docker` |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` was active while sourcing ROS setup | Use this skill's scripts; they source ROS with nounset disabled, then re-enable it |
| Root-owned Docker `build`/`install`/`log` | Older runs wrote artifacts as root inside the container | Rerun this skill's Docker method; it chowns the mounted workspace back to the caller even if the build fails |
| `Failed to open terminal` / whiptail during rosdep | apt/debconf tried to show an interactive dialog in a non-interactive session | Use this skill's native script; it sets `DEBIAN_FRONTEND=noninteractive` and `NEEDRESTART_MODE=a` |
| rosdep pip fails with PEP 668 / `break-system-packages` | Python ≥3.11 blocks system-wide pip installs | Use this skill's scripts; they set `PIP_BREAK_SYSTEM_PACKAGES=1` for native and Docker rosdep |
| Native ROS setup missing | Matching ROS not installed | Use Docker, or provide `--ros-setup` for an existing source build |
| `sudo` cannot prompt | Native dependency or rosdep setup is missing in a non-interactive session | Run the printed install commands or `sudo -v` in an interactive terminal, then retry |
| `pixi.toml` missing | Repo revision lacks Pixi project | Use native/Docker, or select a revision with Pixi support |
| Existing checkout branch differs | Requested branch does not match the checkout | Ask whether to reuse its branch, use another path, or clean-reclone; never switch or delete automatically |
| Existing path is not a Git checkout | Target path is occupied by unrelated content | Stop and ask for another repository path; never delete it |
| False "MSVC missing" from agent probe | Bash-wrapped PowerShell used a single-quoted `${env:ProgramFiles(x86)}` path | Ignore that probe; run `setup_pixi_workspace.ps1` with `-File`, or check the literal `C:\Program Files (x86)\...` path |
| Windows MSVC check fails | C++ Build Tools missing | Install VS 2022 Build Tools with Desktop development with C++ |
| `colcon` finds no packages | Submodules not initialized | Ensure repository submodules initialized successfully |

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/setup_ros_workspace.sh` | Clone and build IsaacSim-ros_workspaces on Linux (native, docker, custom, or pixi) | `--method`, `--distro`, `--repo-url`, `--repo-path`, `--branch`, `--ros-setup`, `--clean-reclone`, `--yes-i-know`, `--skip-build` |
| `scripts/setup_pixi_workspace.ps1` | Clone and build the Jazzy workspace with Pixi on Windows 11 x64 | `-RepoUrl`, `-RepoPath`, `-Branch`, `-CleanReclone`, `-YesIKnow`, `-SkipBuild` |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/setup_ros_workspace.sh", args=["--help"])
```

```python
run_script("scripts/setup_pixi_workspace.ps1", args=[])
```

From a shell, run the helpers directly with `bash` (Linux) or PowerShell (Windows). These scripts
do not require a built Isaac Sim tree.
