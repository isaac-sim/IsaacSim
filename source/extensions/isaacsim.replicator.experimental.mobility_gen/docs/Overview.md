# Overview

**MobilityGen** (`isaacsim.replicator.experimental.mobility_gen`) is an experimental toolkit for
building **synthetic mobility datasets** in Isaac Sim.  A dataset contains synchronized robot
trajectories, occupancy maps, and multi-camera sensor outputs (RGB, depth, segmentation, normals)
suitable for navigation and mobility research.

## Workflow

Dataset generation happens in two separate steps.

**Step 1 — Record.**  Load a scene (warehouse, office, etc.), spawn a robot, and drive it around
(keyboard, gamepad, or autonomous navigation).  MobilityGen saves the scene stage and recording
metadata (robot type, scenario type, occupancy map) to a recording directory, then captures the
robot's full state at every physics step: position, orientation, joint angles, velocities.  No
sensor readings are written — recording is purely physics state and runs at full simulation speed.

**Step 2 — Replay.**  Given a recording directory, MobilityGen restores the recorded robot
trajectory pose-by-pose into the simulation and renders the scene from the robot's camera sensors
at each step.  The output is a new directory containing the original state plus rendered images.
You choose which modalities to render (RGB, depth, segmentation, normals) at replay time without
re-running the physics simulation.

This two-step approach keeps recording fast and decouples the choice of render modalities from data
collection.  The same recording can be replayed multiple times with different render settings — for example,
enabling only RGB on the first pass for quick inspection, then re-running with depth and
segmentation enabled for the final dataset.

## What's in a dataset

```text
my_recording/
├── config.json                          # scenario_type, robot_type, scene_usd_rel_path
├── stage.usd                            # flattened scene — sublayers inlined; references textures/MDL under assets/
├── assets/                              # external assets (textures, MDL, sub-USDs) copied alongside stage.usd
├── occupancy_map/
│   ├── map.yaml                         # ROS-style map metadata
│   └── map.png                          # freespace bitmap
└── state/
    ├── common/
    │   ├── 00000000.npy                 # robot pose, joint state, velocities at step 0
    │   ├── 00000001.npy                 # step 1 …
    │   └── …                            # written during recording
    ├── rgb/
    │   └── sensor_rig.front_hawk_left.rgb_image/
    │       ├── 00000000.jpg             # rendered during replay
    │       └── …
    ├── depth/
    │   └── sensor_rig.front_hawk_left.depth_image_m/
    │       ├── 00000000.png             # 16-bit inverse-depth PNG, converted to metres on read
    │       └── …
    ├── segmentation/
    │   └── sensor_rig.front_hawk_left.segmentation_image/
    │       └── …
    └── normals/
        └── sensor_rig.front_hawk_left.normals_image/
            └── …
```

`state/common/` is written during **recording**.  All sensor subfolders under `state/` are written
during **replay** — one subfolder per camera per modality, named by the camera's full buffer path.

## Main building blocks

- **`MobilityGenRobot`** — Abstract articulation-backed robot: sensor rig, chase camera, velocity/command buffers, 2D pose helpers, keyboard/gamepad hooks, and hooks for writing actions vs. replaying recorded state. Robot parameters (USD asset, wheel geometry, control gains, sensor rig) are declared in a YAML file referenced via `robot_config_path`; they are loaded once at class-definition time. Concrete robots are registered on `ROBOTS`.
- **`MobilityGenScenario`** — Abstract scenario tying a robot to a buffered region of the occupancy map (`from_robot_occupancy_map`). Concrete scenarios are registered on `SCENARIOS`.
- **`MobilityGenCamera`** — Replicator-based rendering module. Attaches a render product and annotators (RGB, semantic segmentation, depth, normals) to an existing `UsdGeom.Camera` prim in the stage. Exposes captured data via `Buffer` instances for dataset export.
- **`MobilityGenSensorRig`** — Python `Module` container that groups all sensors on a robot. Cascade methods (`update_state`, `enable_rgb_rendering`, `state_dict_rgb`) propagate to all sensors automatically. Sensors are declared in the robot YAML under `sensor_rig.sensors`. Currently only `type: camera` is supported; `lidar`, `imu`, and `radar` entries are logged and skipped. See `sensor_rig.md` for details.
- **`OccupancyMap`** — Load/save and freespace masks from ROS map conventions; used for planning and visualization.
- **Path utilities** — Grid path generation and simplification (`generate_paths`, `compress_path`) backed by the extension's native path-planner bindings for efficient coverage of freespace.

## Architecture

**Component tree** — every class is a `Module`; cascade calls propagate top-down automatically.

```{mermaid}
graph TD
    SC["MGScenario<br/>path planning · reset logic"]
    RB["MGRobot<br/>pose · joints · velocities · action"]
    SR["MGSensorRig<br/>groups cameras"]
    CL["MGCamera<br/>front_hawk_left"]
    CR["MGCamera<br/>front_hawk_right"]
    AR["Articulation<br/>Isaac Sim physics"]
    UP["UsdGeom.Camera<br/>in robot USD"]

    SC --> RB
    RB --> AR
    RB --> SR
    SR --> CL
    SR --> CR
    CL -. "render product<br/>(RTX renders pixels, capture sensor-readings from buffers)" .-> UP
```

**Record / replay data flow**

```{mermaid}
graph LR
    U(("drive<br/>robot"))
    SC1["MGScenario"]
    W["MGWriter"]
    D[("recording/<br/>config.json<br/>stage.usd<br/>state/common/")]

    RE["MGReader"]
    SC2["MGScenario<br/>(rebuilt from config.json)"]
    W2["MGWriter"]
    OUT[("output/<br/>state/common/<br/>state/rgb/<br/>state/depth/ · · ·")]

    U -->|physics step| SC1
    SC1 -->|state dict| W
    W --> D

    D --> RE
    RE -->|restore poses| SC2
    SC2 -->|rendered frames| W2
    W2 --> OUT
```

## How to run

### Recording (UI)

Recording is driven through the MobilityGen panel in Isaac Sim:

1. Open Isaac Sim and enable the `isaacsim.replicator.experimental.mobility_gen` extension.
2. Open the panel: **Window → MobilityGen**.
3. Fill in the fields: scene USD path, robot type, scenario type, occupancy map path.
4. Click **Build** — this loads the scene, spawns the robot, and starts the simulation.
5. Click **Start Recording**, drive the robot (keyboard / gamepad / autonomous), then click
   **Stop Recording**.

Recordings are saved to `~/MobilityGenData/recordings/<timestamp>/` by default.  Set the
environment variable `MOBILITY_GEN_DATA` to change the root directory.

### Replay (headless script)

Use the provided replay script to process all recordings in a directory:

```bash
./_build/linux-x86_64/release/python.sh \
    source/standalone_examples/replicator/mobility_gen/replay.py \
    --input               ~/MobilityGenData/recordings \
    --output              ~/MobilityGenData/replays \
    --render_interval     1 \
    --rgb_enabled         True \
    --depth_enabled       True \
    --segmentation_enabled True \
    --normals_enabled     False
```

`--input` and `--output` are required; all other flags are optional and shown with their defaults.
`--render_interval` is the most useful tuning knob — increase it to skip frames and run faster.

Each output directory has the same layout as a recording plus the rendered sensor subfolders
(`state/rgb/`, `state/depth/`, etc.).

> **Note:** the `--enable isaacsim.replicator.mobility_gen.examples` flag is required if your
> robot classes are defined in that extension (e.g. `CarterRobot`, `JetbotRobot`).

### Migrating older recordings

Recordings made before version 0.2.0 store `state/common/` steps as `.npy` files (pickled dicts).
The current reader expects `.npz` and will log a warning if `.npy` files are detected — replay
will produce 0 steps without migrating.  Convert a recording directory in-place with:

```bash
./_build/linux-x86_64/release/python.sh \
    source/standalone_examples/replicator/mobility_gen/migrate_recordings.py \
    --input ~/MobilityGenData/recordings
```

## Current support

### Scenarios

| Scenario | Type | Description |
|----------|------|-------------|
| `KeyboardTeleoperationScenario` | Manual | WASD keyboard control |
| `GamepadTeleoperationScenario` | Manual | Joystick / gamepad control |
| `RandomAccelerationScenario` | Autonomous | Random walk — applies acceleration noise each step; terminates on collision |
| `RandomPathFollowingScenario` | Autonomous | Plans a path to a random free-space destination and follows it; terminates on arrival or collision |

### Robots

There are two tiers of robot classes, both defined in `isaacsim.replicator.mobility_gen.examples`:

**YAML-driven (recommended for new robots)** — inherit from `MobilityGenMultiSensorRobot` via
`WheeledMultiSensorRobot` or `PolicyMultiSensorRobot`.  All parameters (asset path, wheel
geometry, sensor rig) come from a YAML file; `__init_subclass__` applies them at import time.

| Robot | Base class | Asset |
|-------|-----------|-------|
| `CarterMultiSensorRobot` | `WheeledMultiSensorRobot` | NVIDIA Nova Carter |
| `JetbotMultiSensorRobot` | `WheeledMultiSensorRobot` | NVIDIA Jetbot |
| `H1MultiSensorRobot` | `PolicyMultiSensorRobot` | Unitree H1 |
| `SpotMultiSensorRobot` | `PolicyMultiSensorRobot` | Boston Dynamics Spot |

**Single front-camera** — inherit directly from `MobilityGenRobot` via
`WheeledMobilityGenRobot` or `PolicyMobilityGenRobot`.  Each robot has one fixed front-facing
camera; parameters are set as class attributes.

| Robot | Base class | Asset |
|-------|-----------|-------|
| `JetbotRobot` | `WheeledMobilityGenRobot` | NVIDIA Jetbot |
| `CarterRobot` | `WheeledMobilityGenRobot` | NVIDIA Nova Carter |
| `H1Robot` | `PolicyMobilityGenRobot` | Unitree H1 |
| `SpotRobot` | `PolicyMobilityGenRobot` | Boston Dynamics Spot |

### Sensors

| Type | Support | Render outputs |
|------|---------|----------------|
| Camera (`UsdGeom.Camera`) | Full — multi-camera | RGB, depth, segmentation, normals |
| LiDAR | Not supported | — |
| IMU | Not supported | — |
| Radar | Not supported | — |

Unsupported sensor types are discovered and listed by `generate_sensor_rigs.py` but skipped at
runtime with a log message.

## See also

- [Adding a robot](adding_a_robot.md) — step-by-step guide for adding a new robot type
- [Sensor rig](sensor_rig.md) — sensor rig architecture, YAML format, and robot integration
- [Module and Buffer](module.md) — cascade tree pattern, buffer tagging, and how camera data flows from render product to disk

```{toctree}
:hidden:

adding_a_robot.md
sensor_rig.md
module.md
```

## API reference

Generated Python API documentation is in **`docs/api.rst`** (module `isaacsim.replicator.experimental.mobility_gen`).
