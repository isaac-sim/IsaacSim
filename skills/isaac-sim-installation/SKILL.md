---
name: "isaac-sim-installation"
description: "Install Isaac Sim from a public standalone build, Docker image, or Python package behind preflight, compatibility, EULA, and --execute gates. Use when installing Isaac Sim; never launches the app."
license: "Apache-2.0"
metadata:
  author: "ropardeshi <info@nvidia.com>"
  tags:
    - isaac-sim
    - installation
    - docker
    - binary
    - python
    - compatibility-check
  languages:
    - python
    - bash
  frameworks:
    - isaac-sim
    - docker
  domain: "simulation"
  team: "isaac-qa"
---

# Isaac-Sim Installation (Public Builds)

## Purpose

Install Isaac Sim while enforcing system-requirement, compatibility, EULA, and execution-approval gates. End with the installed path or Docker image identity.

## Installation-only boundary

This boundary is mandatory:

- Do not launch `isaac-sim.sh`, `isaac-sim.bat`, `runheadless.sh`, a `SimulationApp`, or any other Isaac Sim application.
- Standalone installation must download, verify, extract, set packaged executable bits, run `post_install`, and then run `warmup.sh` or `warmup.bat`. Stop after warmup; do not invoke the Isaac Sim application launcher.
- Treat standalone warmup as complete only when its process exits successfully and its log contains no fatal assertion, abort, or child-process launch signature. Write `.isaac-sim-warmup-complete` only after that clean result.
- Docker installation may only pull and inspect the image. Do not create or start a container.
- Python installation may create a virtual environment and install packages. Do not import `isaacsim`.
- The Compatibility Checker is the only permitted application execution in this skill. On Linux, run it as a pre-install eligibility gate. On Windows, run it inside the selected installer at the platform-specific point defined below.

## Prerequisites

- Obtain explicit operator approval before any `--execute` action.
- Obtain explicit EULA acknowledgment before installation.
- Require at least 50 GB free storage, a visible NVIDIA RTX-capable GPU and compatible driver, and workflow-specific dependencies reported by `scripts/preflight.py`.
- For Python installs, require Python 3.12 with working `venv` and `ensurepip`. On Debian/Ubuntu, install `python3.12-venv`.
- For Docker installs, require Linux, Docker, NVIDIA Container Toolkit, and a passing Docker GPU check.

## Limitations

- Covers public Isaac Sim builds only. Source builds belong to the repository build instructions.
- Scope ends at the Compatibility Checker: the skill never launches Isaac Sim, starts a container, or imports `isaacsim`.
- Installs exactly one method per confirmed selection; multiple methods require separate selections and approvals.
- Docker is unsupported on Windows and WSL.
- Cannot repair a failing GPU, driver, or storage requirement; a hard preflight failure stops the workflow.

## Installation-method prompt

Present this selection before preflight or method-specific planning:

> Which Isaac Sim installation method do you want?
>
> 1. **Standalone binary (Default)** — download, verify, extract, run `post_install`, and warm up.
> 2. **Docker image** — on native Linux x86_64 or supported DGX Spark aarch64 only, pull and inspect the image without starting a container.
> 3. **Python package** — create a Python 3.12 venv and install `isaacsim[...]` without importing it.
>
> Select 1, 2, or 3. Press Enter or say “default” to select Standalone binary.

- Use the runtime's structured single-choice prompt when available and mark Standalone binary as the default/recommended choice.
- Always show the prompt for an installation request. If the user already named a method, preselect that method and ask for confirmation.
- Treat an empty response or “default” as Standalone binary.
- Do not run preflight, resolve downloads, or construct an installation plan until the selection is confirmed.
- If Docker is selected on Windows, WSL, or another non-native-Linux host, stop before preflight or planning and offer Standalone binary or Python package instead.
- Continue with exactly one method. Install multiple methods only when the user separately selects and confirms each one.

## Workflow

1. Present the installation-method prompt and record exactly one confirmed selection. Default to Standalone binary only when the user presses Enter, says “default,” or confirms the preselected default.
2. Run preflight:
   ```bash
   python3 <skill-dir>/scripts/preflight.py
   ```
   For Docker, also run:
   ```bash
   python3 <skill-dir>/scripts/preflight.py --docker-gpu-check
   ```
3. Enforce the system-requirements gate. Confirm supported OS/architecture, NVIDIA GPU and driver/CUDA visibility, RTX capability or explicit override, adequate memory/storage, and workflow-specific prerequisites. Stop before installation if a hard requirement fails.
4. Run the Compatibility Checker using the platform-specific sequence:
   - On native Linux with Docker GPU support, plan the container-based checker with:
   ```bash
   python3 <skill-dir>/scripts/validate_install.py container
   ```
   - On Windows standalone, do not run a separate pre-install checker. `install_binary.py` must download, verify, and extract the selected archive; run `isaac-sim.compatibility_check.bat --no-window`, stop the checker after its semantic result, and require `System checking result: PASSED`; then run `post_install.bat` and `warmup.bat`.
   - On Windows Python, do not create a separate minimal checker environment. `install_python.py` must create the Python 3.12 environment and install the full selected package first; then run `isaacsim isaacsim.exp.compatibility_check --no-window` from that environment, stop the checker after its semantic result, and require `System checking result: PASSED`.
   - Docker remains unsupported on Windows.
   Show and approve the complete installer plan containing the Windows checker. Stop the remaining sequence if the checker fails.
5. Select exactly one installer:
   - Standalone: `scripts/install_binary.py`
   - Docker: `scripts/install_container.py`
   - Python package: `scripts/install_python.py`
6. Run the installer without `--execute` and show its complete dry-run plan.
7. Present the EULA URL (<https://docs.omniverse.nvidia.com/eula>) and ask: “This install acknowledges the NVIDIA Isaac Sim / Omniverse EULA on your behalf. Do you agree to the EULA at the URL above? (yes/no)”
8. Ask for explicit approval of the installation plan. Do not execute without both approvals.
9. Re-run the identical installer command with `--execute`.
10. Report the installation result and stop:
    - Standalone: absolute extraction directory and downloaded archive path.
    - Docker: image reference, immutable image ID, and Docker-managed storage root.
    - Python: absolute virtual-environment path, Python executable, and installed package request.
11. For standalone, verify that `install-warmup.log` is free of fatal signatures and `.isaac-sim-warmup-complete` exists. If warmup fails, report “files installed, warmup incomplete,” the first fatal line, and the log path. Do not report installation success.
12. For every method, explicitly state that the Isaac Sim main application was not launched. On Windows, report that the Compatibility Checker was launched and passed.
13. If execution fails, diagnose the first real failure once, provide the smallest repair and one non-launch validation command, then stop. Retry only after an explicit user request. For an approved standalone warmup retry, use:
    ```bash
    python3 <skill-dir>/scripts/install_binary.py \
      --platform <platform> \
      --install-dir <existing-install-dir> \
      --warmup-only \
      --execute
    ```
    Do not download, extract, or run `post_install` again.

## Platform support

- Standalone binary: Linux x86_64, Linux aarch64 on supported DGX Spark systems, and Windows x86_64.
- Docker image: native Linux x86_64 and supported DGX Spark Linux aarch64 only; Windows and WSL are unsupported. The public image tag is multi-architecture.
- Python package: Python 3.12 on supported Linux and Windows platforms.

## Scripts

| Script | Purpose |
|---|---|
| `scripts/preflight.py` | Detect OS, architecture, RAM, disk, GPU, Docker, NVIDIA Container Toolkit, and Python 3.12 `venv`/`ensurepip`. |
| `scripts/install_binary.py` | Download, verify, and extract a public standalone build; on Windows run the Compatibility Checker before `post_install` and warmup; retry only warmup with `--warmup-only`; never launch the main app. |
| `scripts/install_container.py` | Pull and inspect the Isaac Sim Docker image; never create or start a container. |
| `scripts/install_python.py` | Create a Python 3.12 venv and install `isaacsim[...]`; on Windows run the installed Compatibility Checker afterward; never import or launch the main app. |
| `scripts/validate_install.py` | Run the Compatibility Checker through a Linux container, an extracted Windows/Linux workstation install, or a Windows/Linux Python environment. |
| `scripts/diagnose_logs.py` | Diagnose installation failures. |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/preflight.py", args=["--help"])
```

Otherwise run the same files with the host `python3` from the skill directory, as shown in the examples below.

## Examples

All installer examples are dry-run only. Add `--execute` only after passing preflight and Compatibility Checker, accepting the EULA, and approving the exact plan.

```bash
# First ask the installation-method prompt. Standalone binary is preselected.

# Readiness
python3 <skill-dir>/scripts/preflight.py
python3 <skill-dir>/scripts/preflight.py --docker-gpu-check

# Separately approved pre-install Compatibility Checker
python3 <skill-dir>/scripts/validate_install.py container

# Latest public standalone build
python3 <skill-dir>/scripts/install_binary.py \
  --platform linux-x86_64 \
  --download \
  --verify-md5 \
  --install-dir ~/isaacsim

# Existing standalone archive
python3 <skill-dir>/scripts/install_binary.py \
  --zip ~/Downloads/<isaac-sim-standalone>.zip \
  --install-dir ~/isaacsim

# Docker image pull only
python3 <skill-dir>/scripts/install_container.py \
  --image nvcr.io/nvidia/isaac-sim:6.0.1

# Python package installation only
python3 <skill-dir>/scripts/install_python.py \
  --env-dir ~/env_isaacsim \
  --version 6.0.1 \
  --accept-eula
```

On Windows, the standalone and Python dry-run plans automatically include their required post-download/post-install Compatibility Checker step. Do not run a separate minimal pip checker environment.

## Safety rules

- Never add a launcher command to an installation plan.
- Run warmup only for the standalone installer. Never infer authorization to import, validate by launching, start a Docker container, or run Docker warmup after installation.
- Never accept warmup exit code alone as success. Require a clean log and completion marker; fail on assertions or aborts even when the packaged script returns zero.
- Never run an installer with `--execute` until the user confirms the target, side effects, and EULA.
- For Python installs, require an exact `--version`; use pip isolation so host index configuration cannot alter package resolution.
- Do not perform destructive cleanup.
- Do not embed credentials, tokens, usernames, or private IPs in skill files.
- For standalone downloads, show the resolved source, URL, MD5, archive path, and install directory before execution.
- For Docker, allow only NVIDIA registry images unless the operator explicitly verifies a different source.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Preflight reports no NVIDIA GPU or driver | Driver not installed or GPU not visible to the host | Install/repair the NVIDIA driver, confirm `nvidia-smi`, then rerun `scripts/preflight.py` |
| MD5 mismatch on a standalone archive | Truncated or tampered download | Delete the archive and rerun `install_binary.py --download --verify-md5` |
| Warmup exits zero but leaves fatal log lines | Assertion, abort, or child-process launch during warmup | Report "files installed, warmup incomplete", then retry with `--warmup-only --execute` after approval |
| `python3 -m venv` fails on Debian/Ubuntu | `python3.12-venv` / `ensurepip` missing | Install `python3.12-venv` and rerun `install_python.py` |
| Docker pull fails or GPU check fails | Docker, NVIDIA Container Toolkit, or registry access missing | Fix the Docker/toolkit setup, or select standalone binary or Python package instead |
| `System checking result` not `PASSED` | Host fails Isaac Sim compatibility requirements | Stop the sequence; resolve the reported requirement before installing |

## References

- Read `references/install-types.md` to select an install type.
- Read `references/binary.md` for standalone details.
- Read `references/docker.md` for Docker pull and Compatibility Checker details.
- Read `references/python.md` for Python package details.
- Read `references/troubleshooting.md` only after an installation failure.
