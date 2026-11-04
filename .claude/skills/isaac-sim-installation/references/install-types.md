# Install Types

Before preflight or planning, prompt the user to select Standalone binary, Docker image, or Python package. Present Standalone binary first and mark it as the default/recommended choice. If the user explicitly names a method, preselect it and ask for confirmation.

## Standalone Binary — Default

Use for standalone zip installation on Linux x86_64, Linux aarch64 on a supported DGX Spark system, or Windows x86_64. Download, verify, extract, optionally run `post_install`, run the required warmup, report the install directory, and stop without launching Isaac Sim.

Select this method when the user presses Enter, says “default,” or confirms the preselected Standalone binary choice.

Default script:

```bash
python3 scripts/install_binary.py --zip ~/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64.zip --install-dir ~/isaacsim
```

## Docker Image

Use only on native Linux x86_64 or supported DGX Spark Linux aarch64 hosts to pull the multi-architecture Isaac Sim image into Docker-managed storage. Windows and WSL are unsupported. Report the image identity and Docker storage root without creating or starting a container.

Default script:

```bash
python3 scripts/install_container.py --image nvcr.io/nvidia/isaac-sim:6.2.0
```

## Python Package

Use for venv or conda workflows, notebooks, standalone scripts, and package-based automation on supported Linux and Windows platforms. Current public docs use Python 3.12 and NVIDIA PyPI.

Default script:

```bash
python3 scripts/install_python.py --env-dir ~/env_isaacsim --version 6.0.1
```

## Public Docs

- Installation overview: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html
- Requirements: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- Workstation: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html
- Container: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- Python: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html
