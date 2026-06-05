---
description: How to build Isaac Sim - prerequisites, commands, and configuration options
---

# Building Isaac Sim

## Overview
This codebase uses a custom build system based on `repo.sh`/`repo.bat` scripts with premake5 configuration.
Dependencies are automatically managed via packman XML files.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 (Linux) or Windows 10/11
- **GPU**: NVIDIA RTX 4080+ or datacenter GPU (A40, L40S, etc.)
- **Driver**: Latest NVIDIA drivers (see [Omniverse requirements](https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html))
- **Internet**: Required for downloading dependencies

### Required Dependencies (Linux)
```bash
# Build essentials
sudo apt-get install build-essential

# GCC/G++ 11 (required - newer versions not supported)
sudo apt-get install gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200

# Git LFS
sudo apt-get install git-lfs
git lfs install
```

### Required Dependencies (Windows)
- Visual Studio 2019 or 2022 with "Desktop development with C++" workload
- Windows SDK (installed with Visual Studio)

## Build Commands

### Basic Build

**Internal (NVIDIA) builds** use linbuild (Docker-based). Omit the `--no-docker` flag:
```bash
# Linux - builds both debug and release configurations
./build.sh

# Windows
build.bat
```

**External / open-source builds** cannot use linbuild. Pass `--no-docker`:
```bash
./build.sh --no-docker
```

### Build Configurations

```bash
# Build only release configuration (default for most development)
./build.sh --release

# Build only debug configuration
./build.sh --debug

# Build both configurations explicitly
./build.sh --config release debug
```

### Build Performance Options

```bash
# Use percentage of CPU cores (recommended for faster builds)
./build.sh --core-percentage 75

# Specify exact number of parallel jobs
./build.sh --jobs 16

# Control memory per core (default auto-calculated)
./build.sh --mem-per-core 4
```

### Rebuild Options

```bash
# Clean build (removes all build artifacts first)
./build.sh --rebuild

# Fast rebuild (only cleans build target folders)
./build.sh --fast-rebuild

# Clean only (removes build artifacts and exits)
./build.sh --clean
```

### Selective Build Options

```bash
# Only fetch dependencies (step 1)
./build.sh --fetch-only

# Only generate projects (steps 1, 2, 3)
./build.sh --generate

# Only build (skip dependency fetch and project generation)
./build.sh --build-only

# Build specific target
./build.sh --target carb.dictionary.plugin
```

### Advanced Build Options

```bash
# Skip compiler version check (use at your own risk)
./build.sh --skip-compiler-version-check

# Keep building despite errors
./build.sh --keep-going

# Use ccache for distributed builds
./build.sh --ccache

# Verbose output
./build.sh --verbose

# Quiet output
./build.sh --quiet
```

### Sanitizer Options (Debug Builds)

```bash
# Enable address sanitizer (memory error detection)
./build.sh --enable-asan --debug

# Enable undefined behavior sanitizer
./build.sh --enable-ubsan --debug

# Enable thread sanitizer (data race detection)
./build.sh --enable-tsan --debug

# Enable memory leak detection
./build.sh --enable-lsan --debug

# Enable code coverage
./build.sh --enable-gcov
```

### Compiler Options

```bash
# Specify compiler toolset (gcc, clang, or msc)
./build.sh --compiler-toolset gcc

# Set C++11 ABI usage (Linux only)
./build.sh --linux-x86_64-cxx11-abi true
```

## Build System Structure

### Key Files
- `build.sh` / `build.bat`: Main build entry points
- `repo.sh` / `repo.bat`: Core build system wrapper
- `premake5.lua`: Main build configuration
- `premake5-isaacsim.lua`: Isaac Sim specific build configuration
- `premake5-tests.lua`: Test configuration

### Dependency Management
Dependencies are defined in `deps/` directory:

**Packman dependencies** (C++ libraries, SDKs):
- `isaac-sim.packman.xml`: Isaac Sim core dependencies
- `kit-sdk.packman.xml`: Omniverse Kit SDK
- `kit-sdk-deps.packman.xml`: Kit SDK dependencies
- `ext-deps.packman.xml`: External C++ dependencies
- `omni-physics.packman.xml`: PhysX and physics dependencies
- `repo-deps.packman.xml`: Repo build system dependencies

**Python dependencies** (.toml files):
- `pip.toml`: Core Python packages
- `pip_compute.toml`: Compute/GPU packages
- `pip_ml.toml`: Machine learning packages
- `pip_lula.toml`: Lula motion planning packages
- `pip_cloud.toml`: Cloud-related packages
- `pip_ucx.toml`: UCX packages

### Build Outputs
- `_build/linux-x86_64/release/`: Linux release build
- `_build/linux-x86_64/debug/`: Linux debug build
- `_build/windows-x86_64/release/`: Windows release build
- `_build/windows-x86_64/debug/`: Windows debug build
- `_build/target-deps/`: Downloaded packman dependencies
- `_compiler/`: Compiler toolchain
- `_repo/`: Build system internals

## Common Build Issues

### Compiler Version Issues
**Problem**: Build fails with GCC version mismatch
**Solution**: Install GCC 11 or use `--skip-compiler-version-check` (not recommended)
```bash
sudo apt-get install gcc-11 g++-11
sudo update-alternatives --config gcc
```

### EULA Not Accepted
**Problem**: Build fails with EULA error
**Solution**: Accept NVIDIA Software License Agreement when prompted, or run:
```bash
./tools/eula_check.sh
```

### Missing Dependencies
**Problem**: Build fails due to missing packman dependencies
**Solution**: Ensure internet access and re-run build (dependencies auto-download). To manually fetch:
```bash
./build.sh --fetch-only
```

### Git LFS Issues
**Problem**: Large files not downloaded properly
**Solution**: Fetch all LFS objects:
```bash
git lfs pull
```

### Out of Memory During Build
**Problem**: Build process runs out of memory
**Solution**: Reduce parallel jobs or use memory-per-core limit:
```bash
./build.sh --core-percentage 50 --mem-per-core 4
```

### Incremental Build Issues
**Problem**: Changes not being picked up or build behaving strangely
**Solution**: Try a fast rebuild:
```bash
./build.sh --fast-rebuild
```

If that doesn't work, do a full clean rebuild:
```bash
./build.sh --rebuild
```

## Build Process Steps

The build system executes these steps in order:

1. **Fetch Dependencies**: Downloads packman dependencies (C++ libs, SDKs)
2. **Generate Projects**: Runs premake5 to generate build files
3. **Stage Files**: Copies files and links folders per `prebuild.toml`
4. **Build Projects**: Compiles C++ extensions and libraries
5. **Post-Build**: Runs post-build commands (extension precaching, etc.)

Use flags like `--fetch-only`, `--generate`, or `--build-only` to run specific steps.

## Extension Development

When building extensions:
- Extensions are in `source/extensions/`
- Each extension has `extension.toml` (metadata) and optional `premake5.lua` (C++ build)
- Build system auto-discovers extensions
- C++ extensions use `premake5.lua` for build configuration
- Python-only extensions don't need premake configuration

## Typical Build Workflow

```bash
# First time: Full build (takes 30-60 minutes)
./build.sh

# During development: Fast incremental build
./build.sh --release

# After pulling latest changes: Fetch new dependencies
./build.sh --fetch-only
./build.sh --release

# If things break: Clean rebuild
./build.sh --rebuild
```
