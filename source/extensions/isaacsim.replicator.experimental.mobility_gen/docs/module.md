# Module and Buffer

`Module` and `Buffer` are MobilityGen's core building blocks.  A `Buffer` holds a single piece
of simulation data — a sensor reading, a robot pose, a joint state.  A `Module` owns a set of
Buffers and controls when to read from the simulation into them, and when to apply them back.

## Interaction with simulation

Two methods define how a module exchanges data with the simulation:

- **`update_state()`** — reads the current simulation state (robot pose, sensor readings) and writes it into the module's Buffers.
- **`write_replay_data()`** — reads values from the module's Buffers and applies them back into the simulation (e.g. sets articulation pose and joint positions during replay).

All other code — writers, readers, scenario logic — only touches Buffers, never simulation APIs directly.
These methods call into Omniverse APIs such as `isaacsim.core.experimental` (articulation) and
`omni.replicator.core` (render products, annotators).  The relevant APIs by component:

- **Robot state** — `isaacsim.core.experimental`: `Articulation` → world poses, joint positions, joint velocities, linear/angular velocities
- **Camera** — `omni.replicator.core`: render product + annotator → RGB (`LdrColor`), depth (`distance_to_camera`), segmentation, surface normals
- **LiDAR** — `omni.replicator.core`: RTX LiDAR render product + annotator → point cloud (xyz, intensity, range)
- **IMU** — `isaacsim.sensors.physx` → linear acceleration, angular velocity

`MobilityGenRobot.write_replay_data()` applies buffered values back:

```text
position.get_value()          → articulation.set_world_poses(…)
joint_positions.get_value()   → articulation.set_dof_positions(…)
```

`MobilityGenCamera.update_state()` reads from a render product via an annotator:

```text
UsdGeom.Camera prim
  → rep.create.render_product()     # RTX renders pixels each frame
      → annotator (LdrColor, distance_to_camera, …)
          → annotator.get_data()    # called inside MobilityGenCamera.update_state()
              → Buffer.set_value()  # stored in rgb_image, depth_image_m, …
```

## The component tree

At each simulation step MobilityGen needs to capture a lot of state — robot pose, joint
positions, and images from every camera.  Rather than writing explicit loops to collect each
piece, MobilityGen organises components into a tree and propagates operations top-down
automatically.  Instead of:

```python
robot.update_state()
robot.sensor_rig.front_hawk_left.update_state()
robot.sensor_rig.front_hawk_right.update_state()
```

A single call at the top cascades automatically:

```python
scenario.update_state()   # robot, sensor rig, all cameras — one call
```

```text
MobilityGenScenario
└─ MobilityGenRobot
   ├─ position, orientation, joint_positions, …   ← Buffers
   └─ MobilityGenSensorRig
        ├─ front_hawk_left   (MobilityGenCamera)
        │    └─ rgb_image, depth_image_m, …       ← Buffers
        └─ front_hawk_right  (MobilityGenCamera)
             └─ rgb_image, depth_image_m, …       ← Buffers
```

The three cascade methods are:

- **`update_state()`** — each node reads its current state from the simulation and writes into its Buffers:
  ```python
  # robot — after update_state()
  robot.position.get_value()         # → np.array([x, y, z])
  robot.joint_positions.get_value()  # → np.array([j0, j1, …])

  # camera — after update_state()
  camera.rgb_image.get_value()       # → np.ndarray (H, W, 3)
  camera.depth_image_m.get_value()   # → np.ndarray (H, W)
  ```
- **`write_replay_data()`** — each node takes its buffered state and applies it back to the simulation.
  Used during replay: the robot's recorded pose and joint positions are loaded from disk into Buffers,
  then `write_replay_data()` pushes them into the articulation so the robot appears at the recorded position:
  ```python
  # load recorded state from disk into buffers
  scenario.load_state_dict(reader.read_state_dict_common(index=20))

  # apply buffered pose/joints back to the simulation
  scenario.write_replay_data()
  # → robot articulation is now at the recorded position and joint configuration
  ```
- **`enable_rgb_rendering()`** (and other `enable_*`) — each camera attaches its annotator to
  its render product.

So `scenario.update_state()` at the top captures everything — robot state and all camera images
— in a single call.

## Buffers and tags

A `Buffer` is a value slot with optional string tags:

```python
self.rgb_image    = Buffer(tags=["rgb"])
self.depth_image  = Buffer(tags=["depth"])
self.pose         = Buffer()          # no tags
```

Tags let you collect only the data you need.  `state_dict()` walks the entire tree and returns
all buffers matching a tag filter:

```python
scenario.state_dict(include_tags=["rgb"])
# → {"robot.sensor_rig.front_hawk_left.rgb_image": array,
#    "robot.sensor_rig.front_hawk_right.rgb_image": array}

scenario.state_dict(exclude_tags=["rgb", "depth", "segmentation", "normals"])
# → {"robot.position": array, "robot.joint_positions": array, …}
#    (pose and joints only — heavy image buffers excluded)
```

The keys in the returned dictionary identify which buffer the value came from:

```text
scenario → robot → sensor_rig → front_hawk_left → rgb_image (Buffer)
                                                   key: "robot.sensor_rig.front_hawk_left.rgb_image"
```

## Recording and replay

**Recording** — each physics step calls `scenario.update_state()`, which fills all Buffers from
the simulation.  Only robot state is saved: `state_dict(exclude_tags=["rgb", "depth", ...])` collects
pose and joint Buffers and the writer persists them to `state/common/`.  Sensor images are skipped.

**Replay** — for each recorded step, the reader loads saved values back into Buffers via
`scenario.load_state_dict(...)`, then `scenario.write_replay_data()` applies the buffered pose and
joints to the articulation.  With the robot at the recorded position, `scenario.update_state()`
captures fresh sensor readings into image Buffers; `state_dict(include_tags=["rgb", "depth", ...])`
collects them and the writer saves to `state/rgb/`, `state/depth/`, etc.
