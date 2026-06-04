# Adding a Robot

## Overview

A robot in MobilityGen is a Python class registered on `ROBOTS` plus a YAML file that holds all
its parameters.  At import time, `__init_subclass__` reads the YAML and populates every class
attribute automatically — no boilerplate `__init__` required.

**Two base classes — choose based on how the robot is controlled:**

`WheeledMultiSensorRobot` — differential-drive robots (Jetbot, Nova Carter).  `build()` spawns
the USD asset, creates an `Articulation` directly from the robot prim path, and wraps it with a
`DifferentialController`.  Wheel DOF names, wheelbase, and radius come from the `wheel:` section
of the YAML.

```python
@ROBOTS.register()
class CarterMultiSensorRobot(WheeledMultiSensorRobot):
    robot_config_path = "data/robots/carter.yaml"
```

`PolicyMultiSensorRobot` — policy-driven humanoids and quadrupeds (H1, Spot).  The articulation
is not created directly; instead `build()` instantiates a policy controller (e.g.
`H1FlatTerrainPolicy`) and uses `controller.robot` as the articulation.  The policy class is set
on the subclass alongside `robot_config_path`.

```python
@ROBOTS.register()
class H1MultiSensorRobot(PolicyMultiSensorRobot):
    robot_config_path = "data/robots/h1.yaml"
    policy_class = H1FlatTerrainPolicy
```

Both inherit from `MobilityGenMultiSensorRobot` (defined in
`isaacsim.replicator.experimental.mobility_gen`).  The sensor rig, chase camera, occupancy map,
and control parameters are identical and come entirely from the YAML.

---

## Step 1 — Write the YAML

Create `data/robots/my_robot.yaml` next to the existing robot YAMLs.  Copy an existing one as a
starting point and fill in the robot-specific values.  Required fields:

```yaml
asset_path: /Isaac/Robots/MyVendor/my_robot/my_robot.usd  # path on Nucleus asset server

physics_dt: 0.005       # physics step size in seconds
z_offset: 0.25          # spawn height above ground plane (metres)

chase_camera:
  base_path: chassis_link   # prim under robot root to parent the chase camera
  x_offset: -1.5
  z_offset: 0.8
  tilt_angle: 60.0

occupancy_map:
  radius: 0.55              # robot footprint radius for collision checking (metres)
  collision_radius: 0.5
  z_min: 0.1
  z_max: 0.62
  cell_size: 0.05

control:
  keyboard_linear_velocity_gain: 1.0
  keyboard_angular_velocity_gain: 1.0
  gamepad_linear_velocity_gain: 1.0
  gamepad_angular_velocity_gain: 1.0

random_action:            # velocity distribution for random-action scenario
  linear_velocity_range: [-0.3, 1.0]
  angular_velocity_range: [-0.75, 0.75]
  linear_acceleration_std: 5.0
  angular_acceleration_std: 5.0
  grid_pose_sampler_grid_size: 5.0

path_following:           # parameters for autonomous path-following scenario
  speed: 1.0
  angular_gain: 1.0
  stop_distance_threshold: 0.5
  forward_angle_threshold: 0.785
  target_point_offset_m: 1.0

wheel:                    # wheeled robots only
  dof_names: [left_wheel_joint, right_wheel_joint]
  base_m: 0.413           # wheelbase (metres)
  radius_m: 0.14          # wheel radius (metres)
  chassis_subpath: chassis_link
```

## Step 2 — Scaffold the sensor_rig section

Run `generate_sensor_rigs.py` to discover all `UsdGeom.Camera` prims in the robot USD and
write their prim paths into the YAML automatically.  The script opens the robot USD from Nucleus,
so run it from the repo root with absolute YAML paths:

```bash
BUILD=./_build/linux-x86_64/release
SCRIPT=source/extensions/isaacsim.replicator.mobility_gen.examples/scripts/generate_sensor_rigs.py
DATA=source/extensions/isaacsim.replicator.mobility_gen.examples/isaacsim/replicator/mobility_gen/examples/data/robots

# Inspect without writing:
"$BUILD/python.sh" "$SCRIPT" --list "$DATA/my_robot.yaml"

# Write in-place:
"$BUILD/python.sh" "$SCRIPT" "$DATA/my_robot.yaml"

# Write to a separate directory (originals untouched):
"$BUILD/python.sh" "$SCRIPT" --output-dir /tmp/generated "$DATA/my_robot.yaml"

# Process all robots at once:
"$BUILD/python.sh" "$SCRIPT" "$DATA"/*.yaml
```

This appends a `sensor_rig:` block with auto-generated names and `# TODO` markers:

```yaml
sensor_rig:
  sensors:
    - name: chassis_link_sensors_front_camera  # TODO: rename
      type: camera
      sensor_prim_path: chassis_link/sensors/front_camera
      width_px: 960  # TODO: verify
      height_px: 600  # TODO: verify
```

Edit the output: rename sensors to something readable, verify resolutions.  Unsupported sensor
types (lidar, IMU, radar) are listed with `# unsupported` — leave or delete them as needed.

### Why only width_px / height_px — not focal length, aperture, or distortion?

`width_px` and `height_px` control the **render product resolution**: the pixel buffer
`MobilityGenCamera` allocates when it calls `rep.create.render_product(prim_path, (width, height))`.
This is a MobilityGen-specific rendering parameter that is not stored anywhere in the USD prim.

All other camera parameters — focal length, apertures, distortion coefficients, extrinsics — are
**already stored in the USD camera prim** and travel with the robot USD asset.  MobilityGen reads
them directly from the stage at render time, so there is nothing to configure in the YAML.  If you
need to override them (e.g. apply measured calibration on top of the nominal USD values), use
`save_sensor_overrides` / `apply_sensor_overrides` from `isaacsim.replicator.experimental.mobility_gen`.

## Step 3 — Write the class

Add a 3-line class in `isaacsim.replicator.mobility_gen.examples/robots.py`:

```python
@ROBOTS.register()
class MyRobot(WheeledMultiSensorRobot):
    robot_config_path = "data/robots/my_robot.yaml"
```

`__init_subclass__` reads the YAML at import time and populates all class attributes
automatically.  The robot is immediately available by name in the UI and in `load_scenario()`.
