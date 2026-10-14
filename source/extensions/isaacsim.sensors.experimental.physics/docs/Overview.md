# Overview

The isaacsim.sensors.experimental.physics extension provides experimental physics-based sensors for Isaac Sim robotics applications. It offers five types of sensors — contact sensors for detecting collisions and forces, effort sensors for measuring joint torque and force, IMU sensors for capturing inertial measurements, joint state sensors for reading full articulation DOF state, and raycast sensors for emitting cast rays — with a runtime/authoring split that mirrors `isaacsim.sensors.experimental.rtx`.

```{image} ../../../../source/extensions/isaacsim.sensors.experimental.physics/data/preview.png
---
align: center
---
```


## Key Components

### Contact Sensor

{class}`ContactSensor <isaacsim.sensors.experimental.physics.ContactSensor>` provides collision detection and force measurement with configurable thresholds and radius filtering. Pair it with the {class}`Contact <isaacsim.sensors.experimental.physics.Contact>` authoring class to create the sensor prim. Runtime data comes from the physics tensor API (`IRigidContactView`), which works with both PhysX and Newton.

The sensor must be authored under an enabled rigid-body ancestor. Collision APIs are still required on the geometry that produces contacts. A collider without a rigid body is not a valid host: the tensor contact view is body-centric. Static collision shapes alone cannot host the sensor.

Backend differences:

- PhysX: `Contact.create()` applies `PhysxContactReportAPI` on the parent rigid body so PhysX can populate contact buffers consumed by the tensor view.
- Newton: `PhysxContactReportAPI` is not applied. Newton exposes contacts through its tensor backend; this path does not use the PhysX Contact Report API. Apply `UsdPhysics.MassAPI` to each dynamic rigid body and assign a nonzero mass. `RigidBodyAPI` alone does not provide the mass needed to create a free joint for the default MuJoCo solver.

```python
from isaacsim.sensors.experimental.physics import Contact, ContactSensor

# Create the prim with the authoring class, then wrap with the runtime
sensor = ContactSensor(
    Contact.create(
        "/World/Robot/foot/contact_sensor",
        min_threshold=1.0,
        max_threshold=1000.0,
        radius=0.05,
    )
)

# Get current contact data
frame = sensor.get_data()
if frame["in_contact"]:
    print(f"Contact force: {frame['force']}")
```

The sensor returns structured frame data including contact status, force magnitude, simulation time, and optional raw contact details when enabled via `add_raw_contact_data_to_frame()`.

### Effort Sensor

{class}`EffortSensor <isaacsim.sensors.experimental.physics.EffortSensor>` measures joint effort (torque or force) from articulated bodies using the physics tensor API. It requires a valid articulation hierarchy and monitors specific degrees of freedom within joints.

```python
from isaacsim.sensors.experimental.physics import EffortSensor

# Create sensor for a robot joint
sensor = EffortSensor("/World/Robot/joint_1")

# Read joint effort
reading = sensor.get_sensor_reading()
if reading.is_valid:
    print(f"Joint torque: {reading.value}")
```

The sensor provides {class}`EffortSensorReading <isaacsim.sensors.experimental.physics.EffortSensorReading>` objects containing validity status, simulation time, and effort values. Newton does not currently expose projected joint forces, so effort readings are invalid under Newton rather than being substituted with commanded actuation force. The sensor supports configurable data buffering and dynamic DOF name updates.

### IMU Sensor

{class}`IMUSensor <isaacsim.sensors.experimental.physics.IMUSensor>` captures inertial measurements including linear acceleration, angular velocity, and orientation. Pair it with the {class}`IMU <isaacsim.sensors.experimental.physics.IMU>` authoring class to create a new prim. It supports configurable rolling average filters for each measurement type to reduce noise.

```python
from isaacsim.sensors.experimental.physics import IMU, IMUSensor

sensor = IMUSensor(
    IMU.create(
        "/World/Robot/body/imu",
        linear_acceleration_filter_size=5,
    )
)

# Get current IMU data
frame = sensor.get_data()
print(f"Linear acceleration: {frame['linear_acceleration']}")
print(f"Angular velocity: {frame['angular_velocity']}")
print(f"Orientation: {frame['orientation']}")
```

The sensor returns structured frame data with filtered measurements. `read_gravity` (deprecated) selects what the linear acceleration channel reports: `True` gives specific force, what an accelerometer measures (`+g` at rest, `0` in free fall), and `False` gives the body's coordinate acceleration (`0` at rest, `-g` in free fall). `g` lands on whichever sensor axis opposes gravity, in stage linear units per second squared — `9.81` on a metre stage, `981` on a centimetre one. The reading models an accelerometer at the body origin, so it excludes lever-arm terms.

### Joint State Sensor

{class}`JointStateSensor <isaacsim.sensors.experimental.physics.JointStateSensor>` reads positions, velocities, and efforts for every degree of freedom in an articulation in a single call, analogous to a ROS2 JointState message. It is backed by the C++ IJointStateSensor plugin and requires a valid articulation root prim.

```python
from isaacsim.sensors.experimental.physics import JointStateSensor

# Create sensor for an articulation
sensor = JointStateSensor("/World/Robot")

# After playing the simulation, get full joint state
reading = sensor.get_sensor_reading()
if reading.is_valid:
    for name, pos in zip(reading.dof_names, reading.positions):
        print(f"{name}: {pos:.4f} rad")
```

The sensor returns {class}`JointStateSensorReading <isaacsim.sensors.experimental.physics.JointStateSensorReading>` objects with validity, simulation time, DOF names, and arrays for positions (rad or m), velocities (rad/s or m/s), efforts (Nm or N), and per-DOF joint types. It supports pause/resume via the `enabled` property.

### Raycast Sensor

{class}`RaycastSensor <isaacsim.sensors.experimental.physics.RaycastSensor>` emits a configurable pattern of physics rays from the sensor's local frame and reports the closest hit on each ray. Pair it with the {class}`Raycast <isaacsim.sensors.experimental.physics.Raycast>` authoring class to create a new prim. It supports both static patterns and time-offset rotating sweeps.

```python
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor

sensor = RaycastSensor(
    Raycast.create(
        "/World/Robot/body/raycast",
        ray_origins=[[0, 0, 0]],
        ray_directions=[[1, 0, 0]],
        min_range=0.4,
        max_range=100.0,
        output_frame="WORLD",
    )
)

frame = sensor.get_data()
print(f"Depths: {frame['depths']}")
```

## Functionality

### Programmatic Sensor Creation

`Contact`, `IMU`, and `Raycast` (the authoring classes) each provide a `create()` static method that handles USD prim creation, schema application, and attribute configuration. The returned authoring object is then wrapped with the matching runtime sensor (`ContactSensor`, `IMUSensor`, `RaycastSensor`) for data access. `EffortSensor` and `JointStateSensor` have no schema-bearing prim and are constructed directly from a path. This shape mirrors `isaacsim.sensors.experimental.rtx`, where `create()` similarly lives only on authoring classes.

### Frame-Based Data Access

All sensors implement a consistent frame-based data interface through `get_data()`, returning dictionaries with measurement values, timestamps, and validity information. For lower-level access, each sensor also exposes `get_sensor_reading()`. Raycast and joint-state readings are mutable owned snapshots: their NumPy arrays have a persistent lifetime, and their values and `is_valid` flags remain unchanged after later physics steps, sensor reset, or removal.

Raycast and joint-state sensors additionally expose `get_sensor_reading_view()` for allocation-sensitive code. A reading view provides read-only, zero-copy NumPy arrays over the sensor's current native buffers. The numeric values and `time` update on each physics step. The view's `is_valid` property updates dynamically and becomes `False` when the sensor is reset or removed or the timeline stops. Previously acquired arrays remain safely readable with their last values until the last array or view reference is released. Each numeric property lookup creates a new NumPy array header but does not copy its data, so cache properties such as `depths = view.depths` when reading them repeatedly. The string metadata properties (`hit_prim_paths` and `dof_names`) return read-only tuples copied when accessed. A view performs no synchronization with the physics step that writes its buffers, so reading it while a step is in flight yields torn values that mix the current and the previous reading. Read a view only from the thread that drives physics and only between steps; use a snapshot when a reader cannot be ordered against the step.

Use a snapshot when values must be retained, compared across steps, or modified. Use a view when values are consumed before the next physics step and avoiding the array copy matters.

`ContactSensor.get_data()` and `IMUSensor.get_data()` return an independent frame on every call — a new dictionary, holding newly allocated numpy arrays for the IMU — so two results can be held and compared safely. Use `get_sensor_reading()` on a hot path to read the same values without allocating a frame. Both frames are refreshed only when the underlying reading is valid, so an invalid reading reports the previous values unchanged; use `get_sensor_reading()` and check `is_valid` to distinguish fresh data from stale data.

### Sensor Control

Each sensor supports pause/resume functionality for runtime control of data collection, and provides methods for querying and modifying sensor-specific parameters like thresholds, filter sizes, and detection radius.

## Integration

The extension integrates with isaacsim.core.experimental.prims for prim management, using XformPrim and Articulation base classes. It leverages **omni.timeline** for simulation synchronization and requires the physics simulation framework for sensor data generation.
