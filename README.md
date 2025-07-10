![Isaac Sim](docs/readme/hero_shot_compressed.png)

---
# Isaac Sim

[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![License](https://img.shields.io/badge/license-Apache--2.0-yellow.svg)](https://opensource.org/license/apache-2-0)

> **⚠️ PRE-RELEASE SOFTWARE NOTICE**
> This is pre-release, currently in development. You may encounter bugs, incomplete features, and other issues that will be addressed in future releases. Please [report](#support) any issues you encounter. This will be finalized into a stable release in the future.

NVIDIA Isaac Sim™ is a simulation platform built on NVIDIA Omniverse, designed to develop, test, train, and deploy AI-powered robots in realistic virtual environments. It supports importing robotic systems from common formats such as URDF, MJCF, and CAD. The simulator leverages high-fidelity, GPU-accelerated physics engines to simulate accurate dynamics and support multi-sensor RTX rendering at scale. It comes equipped with end-to-end workflows including synthetic data generation, reinforcement learning, ROS integration, and digital twin simulation. Isaac Sim provides the infrastructure needed to support robotics development at any stage.


## Key Features

- [Asset Import & Export](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/importer_exporter/importers_exporters.html): Importing and exporting robots and environments from and to non-USD format.
- [Robot Tuning](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_setup/index.html): Optimize robot for physics accuracy, computation efficiency, or photorealism
- [Robot Simulation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/index.html): Tools for moving robots, such as controllers, motion generation and kinematics solvers, and policy integration.
- [Sensors](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/sensors/index.html): RTX and physics-based sensors

## Key Applications

- [Isaac Lab](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/isaac_lab_tutorials/index.html): GPU-accelerated framework built for reinforcement learning, imitation learning, and motion planning.
- [ROS Bridge](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/ros2_landing_page.html): Integration with Robot Operating System (ROS).
- [Synthetic Data Generation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/synthetic_data_generation/index.html): Collection of SDG tools

## Documentation

For the latest Isaac Sim documentation, see [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html).
Follow these links to get started:

- [Tutorials](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/introduction/quickstart_index.html)
- [Assets](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/assets/usd_assets_overview.html)


## Prerequisites and Environment Setup

Ensure your system is set up with the following before building Isaac Sim:

- **Operating System**: Windows 10/11 or Linux (Ubuntu 22.04)

  > **(Linux) Ubuntu 24.04**
  > Ubuntu 24.04 is not fully supported at this time. Building with Ubuntu 24.04 requires GCC/G++ 11 to be installed, GCC/G++ 12+ is not supported.

- **GPU**: For additional information on GPU features and requirements, see [NVIDIA GPU Requirements](https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html)

  #### Local Workstation

  | Min | Recommended | Best |
  |-----|-------------|------|
  | RTX 4080 | RTX 5080 | RTX PRO 6000 Blackwell Workstation |
  |  | RTX 5880 Ada | RTX PRO 5000 Blackwell Workstation |

  #### Datacenter

  | Min | Recommended | Best |
  |-----|-------------|------|
  | A40 | L40S | RTX PRO 6000 Blackwell Server |
  |  | L20 | |

- **Driver**: See [NVIDIA Driver Requirements](https://docs.omniverse.nvidia.com/dev-guide/latest/common/technical-requirements.html)

- **Internet Access**: Required for downloading the Omniverse Kit SDK, extensions, and tools.



### Required Software Dependencies

- [**Git**](https://git-scm.com/downloads): For version control and repository management

- [**Git LFS**](https://git-lfs.com/): For managing large files within the repository

- **(Windows - C++ Only) Microsoft Visual Studio (2019 or 2022)**: You can install the latest version from [Visual Studio Downloads](https://visualstudio.microsoft.com/downloads/). Ensure that the **Desktop development with C++** workload is selected.  [Additional information on Windows development configuration](docs/readme/windows_developer_configuration.md)

- **(Windows - C++ Only) Windows SDK**: Install this alongside MSVC. You can find it as part of the Visual Studio Installer. [Additional information on Windows development configuration](docs/readme/windows_developer_configuration.md)

- **(Linux) build-essentials**: A package that includes `make` and other essential tools for building applications.  For Ubuntu, install with:

  ```bash
  sudo apt-get install build-essential
  ```

  > **(Linux) ⚠️**
  > Please use GCC/G++ 11, higher versions are not supported yet. To install GCC/G++ 11, run the following commands:
  > ```bash
  > sudo apt-get install gcc-11 g++-11
  > sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
  > sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
  > ```

### Recommended Software

- [**(Linux) Docker**](https://docs.docker.com/engine/install/ubuntu/): For containerized development and deployment. **Ensure non-root users have Docker permissions.**

- [**(Linux) NVIDIA Container Toolkit**](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html): For GPU-accelerated containerized development and deployment. **Installation and Configuring Docker steps are required.**

- [**VSCode**](https://code.visualstudio.com/download) (or your preferred IDE): For code editing and development

## Quick Start

This section guides you through building Isaac Sim from source code.

### 1. Clone the Repository


```bash
git clone https://github.com/isaac-sim/IsaacSim.git isaacsim
cd isaacsim
git lfs install
git lfs pull
```

### 2. Build

Run the following command to initiate the configuration wizard:

**Linux:**

Confirm that GCC/G++ 11 is being used before building using the following commands:

```bash
gcc --version
g++ --version
```

```bash
./build.sh
```

**Windows:**

> **⚠️ Windows Path Length Limitation**
> Windows has a path length limitation of 260 characters. If you encounter errors related missing files or other build errors, try moving the repository to a shorter path.

```powershell
build.bat
```

### 3. Run

> **⚠️ Startup Time**
> The first time loading Isaac Sim may take up to several minutes as Extensions and Shader are loaded and cached. The subsequent startup time should be in the ranges of 10-30 seconds depending on hardware configuration.



Navigate to the corresponding binary directory for your platform and run the executable.

**Linux:**
```bash
cd _build/linux-x86_64/release
./isaac-sim.sh
```

**Windows:**
```powershell
cd _build/windows-x86_64/release
isaac-sim.bat
```

> NOTE: If this is your first time building Isaac Sim, you will be prompted to accept the Omniverse Licensing Terms.



## Advanced Build Options


Isaac Sim uses a custom build system with the following key options:


### Core Build Options
- `-c, --clean`: Clean the repository and exit
- `-x, --rebuild`: Clean the repository before building (full rebuild)
- `-h, --help`: Show all available build options


### Configuration Options
- `--config [debug|release]`: Specify build configuration (default: both)
- `-d, --debug`: Build only debug configuration
- `-r, --release`: Build only release configuration


### Advanced Options
- `-j NUM_CORES, --jobs NUM_CORES`: Limit the number of parallel compilation jobs
- `-v, --verbose`: Enable verbose build output
- `-q, --quiet`: Suppress build output


### Build Steps Control
- `--fetch-only`: Only fetch dependencies and stop
- `-g, --generate`: Generate projects, stage files and stop
- `-s, --stage`: Stage files, skip generation step
- `-b, --build-only`: Only perform building step, skip others
- `--post-build-only`: Only perform post-build step


### Run in docker
```bash
xhost +"local:docker@"
docker build -t isaac -f build.dockerfile .
docker run -it --rm --gpus all \
           -p 8226:8226
           -e DISPLAY=$DISPLAY \
           -e XAUTHORITY=$XAUTHORITY \
           -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
           -e OMNI_KIT_ALLOW_ROOT=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v $XAUTHORITY=$$XAUTHORITY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
           isaac
```

## Troubleshooting

Please see the [FAQ](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/overview/faq_index.html), [Troubleshooting](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/overview/troubleshooting.html), and [Known Issues](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/overview/known_issues.html) for common questions, fixes, and workarounds.


## Support

* Please use GitHub [Discussions](https://github.com/isaac-sim/IsaacSim/discussions) for discussing ideas, asking questions, and requests for new features.
* Github [Issues](https://github.com/isaac-sim/IsaacSim/issues) should only be used to track executable pieces of work with a definite scope and a clear deliverable. These can be fixing bugs, documentation issues, new features, or general updates.

## Connect with the NVIDIA Omniverse Community

Have a project or resource you'd like to share more widely? We'd love to hear from you! Reach out to the
NVIDIA Omniverse Community team at OmniverseCommunity@nvidia.com to discuss potential opportunities
for broader dissemination of your work.

## License

Licensing terms can be found in the [License File](LICENSE).

## Contributing

We do not support direct community contributions at the moment.

