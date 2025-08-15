# IsaacSim XR Input Devices Extension

This extension provides XR input device support for Manus gloves and Vive trackers, adapted from isaac-deploy Holoscan implementations.

## Features

- **Manus Glove Integration**: Real-time hand and finger tracking with full skeletal data
- **Vive Tracker Integration**: 6DOF pose tracking for HMDs, controllers, and trackers  
- **Live Data Visualization**: Red cubes appear at device positions in Isaac Sim
- **Automatic Setup**: Library paths configured automatically
- **Device Status Monitoring**: Connection state and data freshness tracking
- **Error Handling**: Graceful degradation when hardware/SDKs are unavailable
- **Mock Data Support**: Testing without physical hardware

## Prerequisites

### Hardware Requirements
- **Manus Gloves**: Manus Prime or MetaGloves with license dongle
- **Vive Trackers**: SteamVR/Lighthouse tracking system with trackers

### Software Dependencies

#### Manus SDK
- ManusSDK is automatically included in Isaac Sim's target dependencies
- Requires Manus license dongle for operation
- Falls back to mock data if unavailable

#### libsurvive (for Vive trackers)
Currently requires manual installation. Choose one option:

**Option 1: System Installation**
```bash
sudo apt update
sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake

git clone https://github.com/cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
make && cmake . && make -j$(nproc)
sudo make install && sudo ldconfig
```

**Option 2: User Installation (Recommended)**
```bash
# Build in home directory (detected automatically)
git clone https://github.com/cntools/libsurvive.git ~/libsurvive
cd ~/libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
make && cmake . && make -j$(nproc)
```

## Building

```bash
# Build Isaac Sim (includes this extension)
./build.sh
```

The extension is automatically built as part of Isaac Sim's build process.

## Usage

### Running the Sample
A complete sample script demonstrates both Manus and Vive tracking:

```bash
cd /path/to/IsaacSim
./_build/linux-x86_64/release/python.sh source/standalone_examples/api/isaacsim.xr.input_devices/manus_vive_tracking_sample.py
```

### Python API Usage

```python
from isaacsim.xr.input_devices.impl.xr_device_integration import get_xr_device_integration

# Get the extension's device integration instance
integration = get_xr_device_integration()

# Get all device data
all_data = integration.get_all_device_data()

# Access specific device data
manus_data = all_data.get('manus_gloves', {})
vive_data = all_data.get('vive_trackers', {})

# Check device status
status = all_data.get('device_status', {})
manus_connected = status.get('manus_gloves', {}).get('connected', False)
vive_connected = status.get('vive_trackers', {}).get('connected', False)

print(f"Manus connected: {manus_connected}")
print(f"Vive connected: {vive_connected}")
```

### Data Format

**Manus Glove Data:**
```python
{
    'manus_gloves': {
        'left_glove': {
            'position': np.array([...]),      # 75 joint positions (x,y,z per joint)
            'orientation': np.array([...]),   # 75 joint orientations (w,x,y,z per joint)  
            'valid': True
        },
        'right_glove': { ... }
    }
}
```

**Vive Tracker Data:**
```python
{
    'vive_trackers': {
        'WM0_position': [x, y, z],           # Tracker position
        'WM0_orientation': [w, x, y, z],     # Tracker orientation
        'LH1_position': [x, y, z],           # Lighthouse position
        'LH1_orientation': [w, x, y, z]      # Lighthouse orientation
    }
}
```

## Extension Architecture

```
isaacsim.xr.input_devices/
├── bindings/               # Pybind11 C++ bindings
├── include/               # C++ headers
├── plugins/               # C++ implementation
│   └── ManusTracker.cpp   # Manus SDK integration
├── python/
│   └── impl/
│       ├── extension.py           # Extension lifecycle
│       ├── xr_device_integration.py  # Main orchestrator
│       ├── manus_tracker.py       # Manus wrapper
│       └── vive_tracker.py        # Vive wrapper
└── config/extension.toml   # Extension configuration
```

## Troubleshooting

### Manus Issues
- **License Error**: Unplug and replug the Manus license dongle
- **No Data**: Check if `ManusSDK_Integrated.so` is in `LD_LIBRARY_PATH`
- **Connection Failed**: Ensure Manus Core isn't running simultaneously

### Vive Issues  
- **No Trackers Found**: Verify SteamVR is not running (conflicts with libsurvive)
- **Lighthouse Only**: Check that trackers have tracking lock (LED should be solid)
- **pysurvive Import Error**: Install libsurvive (see Prerequisites section)

### Build Issues
- Verify Isaac Sim build environment is set up correctly
- Check that Manus SDK is in `_build/target-deps/manus_sdk/`
- Ensure all C++ dependencies are available

### Runtime Issues
```bash
# Check device connections
lsusb | grep -i manus
lsusb | grep -i lighthouse

# Monitor Isaac Sim logs for device initialization
# Look for: "Manus glove tracker initialized with SDK"
# Look for: "Adding tracked object WM0 from HTC"
```

## Contributing

This extension follows Isaac Sim's extension development guidelines:
- C++ code uses pybind11 for Python bindings
- Python code follows Isaac Sim coding standards  
- All changes should maintain backward compatibility
- Include tests for new functionality

## License

Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0 
