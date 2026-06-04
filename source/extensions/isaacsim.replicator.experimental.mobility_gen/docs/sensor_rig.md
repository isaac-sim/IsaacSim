# Sensor Rig

`MobilityGenSensorRig` is a Python `Module` container that groups all sensors on a robot so
that cascade operations — `update_state()`, `enable_rgb_rendering()`, `state_dict_rgb()` —
propagate to all sensors automatically without manual loops.

Each sensor is attached as a named attribute on the rig (e.g. `rig.front_hawk_left`).  The
sensor name comes from the robot YAML and becomes the key in output data
(e.g. `"sensor_rig.front_hawk_left.rgb_image"`).

**Currently supported:** `type: camera` — full support via `MobilityGenCamera`.
**Logged and skipped:** `type: lidar`, `imu`, `radar`.

## Sensor attachment

`sensor_prim_path` in the YAML is relative to the robot root prim.  At build time it is joined
with `robot_root_path` to form an absolute stage path.  For cameras,
`rep.create.render_product(abs_path, resolution)` attaches Replicator to the existing
`UsdGeom.Camera` prim.  If the prim is missing, the sensor is skipped with a `carb.log_error`.

## Intrinsics (cameras)

USD is the source of truth.  `focal_length`, `aperture`, and other optical parameters live in the
`UsdGeom.Camera` prim baked into the robot asset.  They are intentionally absent from the YAML —
duplicating them would create a divergence risk with no benefit.  The YAML only carries
`sensor_prim_path` and pixel resolution.

## YAML format

```yaml
sensor_rig:
  sensors:
    - name: front_hawk_left
      type: camera
      sensor_prim_path: chassis_link/sensors/front_hawk/left/camera_left
      width_px: 960
      height_px: 600

    - name: front_hawk_right
      type: camera
      sensor_prim_path: chassis_link/sensors/front_hawk/right/camera_right
      width_px: 960
      height_px: 600

    - name: chassis_imu        # unsupported — logged and skipped
      type: imu
      sensor_prim_path: chassis_link/sensors/chassis_imu/Imu_Sensor
```

## Scaffolding the YAML

`scripts/generate_sensor_rigs.py` opens a robot USD, discovers all sensor prims, and writes
`sensor_prim_path` entries with `# TODO` comments on fields requiring human review (name,
resolution).  Unsupported sensor types are included as comments for inventory.  Run once per
robot, edit the output, then commit.

```bash
BUILD=./_build/linux-x86_64/release
SCRIPT=source/extensions/isaacsim.replicator.mobility_gen.examples/scripts/generate_sensor_rigs.py
DATA=source/extensions/isaacsim.replicator.mobility_gen.examples/isaacsim/replicator/mobility_gen/examples/data/robots

"$BUILD/python.sh" "$SCRIPT" --list "$DATA/carter.yaml"   # inspect only
"$BUILD/python.sh" "$SCRIPT" "$DATA/carter.yaml"          # write in-place
```

## Robot integration

`MobilityGenRobot.__init_subclass__` reads the YAML at class-definition time and stores
`cls.sensor_configs`.  `build()` calls `cls.build_sensor_rig(prim_path)` after spawning the
articulation; `build_sensor_rig` calls `MobilityGenSensorRig.build_mounted`.  Robots with no
`sensor_configs` (e.g. `H1Robot`) get `sensor_rig = None`.

```python
robot = CarterRobot.build("/World/carter")
robot.sensor_rig.enable_rgb_rendering()
robot.sensor_rig.finalize_rendering()   # re-enables Hydra — must be called after all enable_*

# each simulation step
robot.update_state()
images = robot.state_dict_rgb()
# → {"sensor_rig.front_hawk_left.rgb_image": np.ndarray,
#    "sensor_rig.front_hawk_right.rgb_image": np.ndarray}
```

> **Footgun:** `enable_rgb_rendering()` disables Hydra texture updates while annotators are
> attached to avoid OmniGraph crashes on partially-constructed nodes.  `finalize_rendering()`
> re-enables Hydra.  Omitting it causes cameras to silently produce no frames.

## Recording and replay

The UI recording callback writes **pose/state only** — no sensor readings.
Sensor data is generated at **replay time** by `replay_scenario()` in `build.py`, which restores
recorded poses to the stage, steps the renderer, and writes annotator outputs.  Render modalities
(RGB, depth, segmentation, normals) are selected at replay time via `ReplayConfig`.

Multi-sensor output is automatic: `state_dict_rgb()` traverses the Module tree recursively,
keying each sensor buffer by its full dotted path.  The writer uses the key as a folder name;
the reader discovers folders via glob.  Adding sensors to the YAML requires no changes to
writer, reader, or scenario code.
