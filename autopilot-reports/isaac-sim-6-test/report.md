# Isaac Sim 6 Headless Validation Report

**Date:** 2026-03-24
**Branch:** autopilot/isaac-sim-6-test
**Status:** PASSED

## Environment

| Component | Value |
|-----------|-------|
| Isaac Sim | 6.0.0 (v6.0.0-dev2 tag) |
| Kit SDK | 110.0.0 (hash 4a5123f4) |
| Python | 3.12 |
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| Kernel | 5.15.0-113-generic |
| GPU | NVIDIA L40 (49140 MB VRAM) |
| Driver | 570.158.01 |
| CUDA | 12.8 |
| CPU | Intel Xeon Platinum 8362 @ 2.80GHz (16 cores quota / 64 bare metal) |
| RAM | 65536 MB |

## Test Results

### SimulationApp Launch
- **Result:** SUCCESS
- **Startup time:** 11.65s
- **Extensions loaded:** ~200+ (all resolved successfully)
- **GPU VRAM after launch:** 2719 MB

### World Creation
- **Result:** SUCCESS
- **Time:** 9.74s
- **Scene:** Default ground plane loaded from S3 asset server
- **Physics scene:** Created at `/physicsScene`

### Physics Stepping (10 steps, no render)
- **Result:** SUCCESS
- **Total time:** 0.0055s
- **Average step time:** 0.0005s
- **Min step:** 0.0003s
- **Max step:** 0.0012s

### Render Stepping (5 steps)
- **Result:** SUCCESS
- **Total time:** 0.0376s
- **Average render step time:** 0.0075s
- **GPU VRAM after simulation:** 3466 MB

## GPU VRAM Usage

| Phase | VRAM Used (MB) | Delta |
|-------|---------------|-------|
| Before launch | ~1 | — |
| After app launch | 2719 | +2718 |
| After simulation | 3466 | +747 |

## Success Criteria

- [x] Isaac Sim 6 installs (headless)
- [x] SimulationApp launches without errors
- [x] Basic scene loads and steps 10+ physics frames
- [x] Report includes version, step timing, GPU VRAM usage, errors

## Installation Notes

Isaac Sim 6.0.0 was installed via a hybrid approach:
1. `pip install isaacsim==6.0.0.0` (from PyPI/NVIDIA index) for core Python package + compiled extensions
2. Source build (`./build.sh` from v6.0.0-dev2 tag) for additional pure-Python extensions not available via pip
3. Extension registries configured to use Azure prod CDN (`ovextensionsprod.blob.core.windows.net`) instead of CloudFront (which returns 403 for many extension archives)

### Key Configuration
- `DISPLAY=` (unset for headless)
- `VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json` (Vulkan ICD path)
- `os._exit(0)` used instead of `app.close()` to avoid TaskGroup destructor crash during cleanup

### API Changes from Isaac Sim 4/5
- `omni.isaac.core` → `isaacsim.core.api` (module rename in v6)
- `SimulationApp` import path: `from isaacsim import SimulationApp`

## Warnings (non-blocking)

- `omni.platforminfo.plugin`: failed to open the default display (expected in headless mode)
- `carb.audio.device`: audio device misconfigured (expected — no audio hardware)
- `gpu.foundation.plugin`: IOMMU is enabled (informational)
- `omni.usd`: USD Diagnostics muted (informational)

## Errors

None — all tests passed without errors.
