# IsaacSim XR Input Devices Extension

This extension provides XR input device support for Manus gloves and Vive trackers, with a unified Python API and automatic wrist mapping between trackers and OpenXR wrists.

## Features

- **Manus gloves**: Real-time hand/finger joint poses via C++ tracker with Python bindings
- **Vive trackers**: 6DOF poses via libsurvive (`pysurvive`) with mock fallback
- **Left/Right mapping**: Resolves which Vive tracker corresponds to left/right wrist
- **Scene alignment**: Estimates a stable scene↔lighthouse transform from early samples
- **Unified API**: Single call to fetch all device data and device status
- **Visualization sample**: Renders gloves/trackers as cubes in Isaac Sim

## Prerequisites

### Hardware
- **Manus Gloves**: Manus Prime/MetaGloves with license dongle
- **Vive Trackers**: Lighthouse tracking (SteamVR base stations + trackers)

### Software
- **Manus SDK**: Bundled in Isaac Sim target-deps (mock used if missing)
- **libsurvive / pysurvive**: Required for real Vive tracking; otherwise mock data is used

Install libsurvive (either system-wide or in your home directory):

System-wide:
```bash
sudo apt update
sudo apt install -y build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev \
  freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake

git clone https://github.com/cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
make && cmake . && make -j"$(nproc)"
sudo make install && sudo ldconfig
```

User (home) install (recommended during development):
```bash
git clone https://github.com/cntools/libsurvive.git ~/libsurvive
cd ~/libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
make && cmake . && make -j"$(nproc)"

# Ensure Python can find pysurvive bindings
export PYTHONPATH="$HOME/libsurvive/bindings/python:$PYTHONPATH"
```

Note: The Vive tracker wrapper currently prepends a libsurvive Python path. Adjust it as needed for your environment or set `PYTHONPATH` as shown above.

## Building

```bash
# Build Isaac Sim (includes this extension)
./build.sh
```

## Usage

### Sample
Run the visualization sample that renders Manus joints (red cubes) and Vive trackers (blue cubes):

```bash
cd /path/to/IsaacSim
./_build/linux-x86_64/release/python.sh \
  source/standalone_examples/api/isaacsim.xr.input_devices/manus_vive_tracking_sample.py
```

### Python API

```python
from isaacsim.xr.input_devices.impl.xr_device_integration import get_xr_device_integration

# Obtain the shared integration instance from the extension
integration = get_xr_device_integration()

# Fetch all device data
all_data = integration.get_all_device_data()

manus_data = all_data.get('manus_gloves', {})
vive_data = all_data.get('vive_trackers', {})
status = all_data.get('device_status', {})

print(f"Manus connected: {status.get('manus_gloves', {}).get('connected', False)}")
print(f"Vive connected: {status.get('vive_trackers', {}).get('connected', False)}")
```

### Data Format

```python
{
  'manus_gloves': {
    'left_0': {
      'position': [x, y, z],
      'orientation': [w, x, y, z]
    },
    'left_1': { ... },
    'right_0': { ... },
    # ... per-joint entries
  },
  'vive_trackers': {
    '<device_id>': {
      'position': [x, y, z],
      'orientation': [w, x, y, z]
    },
    # e.g., 'WM0', 'WM1', or device names from libsurvive
  },
  'device_status': {
    'manus_gloves': {'connected': bool, 'last_data_time': float},
    'vive_trackers': {'connected': bool, 'last_data_time': float},
    'left_hand_connected': bool,
    'right_hand_connected': bool
  }
}
```

## How left/right mapping is determined

- Detect connected OpenXR wrists (left/right) and available Vive wrist markers (e.g., WM0/WM1)
- For each frame, compute candidate transforms for both pairings:
  - Pair A: WM0→Left, WM1→Right
  - Pair B: WM1→Left, WM0→Right
- Accumulate translation/rotation deltas per pairing when both wrists and trackers are present
- Choose the pairing:
  - Prefer the pairing with more samples initially
  - Once there are enough paired frames, choose the one with lower accumulated error
- Cluster the chosen pairing’s transforms and average them to estimate a stable
  scene↔lighthouse transform (`scene_T_lighthouse_static`)
- Use the resolved mapping and transform to place all Vive and Manus data in scene coordinates

## Troubleshooting

- Manus license issues: replug license dongle; ensure SDK libraries are discoverable
- libsurvive conflicts: ensure SteamVR is NOT running concurrently
- No Vive devices: verify udev rules and USB connections (solid tracker LED)
- Python import for `pysurvive`: set `PYTHONPATH` to libsurvive bindings path

## Extension Layout

```
isaacsim.xr.input_devices/
├── bindings/                    # Pybind11 bindings
├── include/                     # C++ headers
├── plugins/                     # C++ implementations
├── python/
│   └── impl/
│       ├── extension.py                 # Extension lifecycle
│       ├── xr_device_integration.py     # Orchestrates devices & transforms
│       ├── manus_tracker.py             # Manus wrapper
│       └── vive_tracker.py              # Vive wrapper
└── docs/
    ├── README.md
    └── CHANGELOG.md
```

## License

Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES.

SPDX-License-Identifier: Apache-2.0 
