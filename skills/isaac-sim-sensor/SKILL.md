---
name: isaac-sim-sensor
description: >
  Sensor simulation in Isaac Sim 6 / Kit 110 with Replicator. Modern stack: `isaacsim.sensors.experimental.rtx` (RTX cameras, lidar, radar, acoustic)  and `isaacsim.sensors.experimental.physics` (contact, IMU, effort, joint state, raycast). Covers the `SUPPORTED_LIDAR_CONFIGS` vendor catalog (Ouster, Hesai, Velodyne, Robosense, SICK, Zvision, NVIDIA examples),  USDA asset attachment to robot mounts, custom scan-pattern authoring (emitter-state arrays), `omni.replicator.core` annotators, writers  (`BasicWriter`, `PoseWriter`, `KittiWriter`, `CosmosWriter`),  randomization, and the `isaacsim.replicator.episode_recorder` /  `isaacsim.replicator.grasping` workflows. Use when setting up RGB / depth  / segmentation / lidar / radar / acoustic / IMU / contact sensors, attaching vendor lidar/radar USD to a robot, authoring scan patterns, generating synthetic training data with domain randomization, or building multi-AOV perception pipelines.
---

# Omniverse Sensor Simulation

Cameras (RGB/depth/seg/bbox), lidar/radar/acoustic, IMU/contact/effort, and Replicator domain randomization. Targets Isaac Sim 6 / Kit 110.

Modern namespaces (use these in new code):

| Family | Module |
|---|---|
| RTX sensors (lidar, radar, acoustic, RTX camera) | `isaacsim.sensors.experimental.rtx` |
| Physics sensors (contact, IMU, effort, joint state, raycast) | `isaacsim.sensors.experimental.physics` |
| Replicator core | `omni.replicator.core as rep` |
| Replicator examples / SDG | `isaacsim.replicator.examples` |
| Mobile-robot SDG | `isaacsim.replicator.mobility_gen` |
| Grasping SDG | `isaacsim.replicator.grasping` |
| Episode record / replay | `isaacsim.replicator.episode_recorder` |
| Teleop record / replay | `isaacsim.replicator.teleop` |
| Domain randomization helpers | `isaacsim.replicator.domain_randomization` |

Legacy `isaacsim.sensors.physics.*` and `isaacsim.sensors.camera.Camera` classes still load but the implementation has moved to the `experimental.*` extensions; prefer those for new work.

> **Migration:** update scripts using the per-family migration guides — [physics sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_physics_to_experimental_physics.html#isaacsim-sensors-physics-migration), [camera sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_camera_to_experimental_rtx.html#isaacsim-sensors-camera-migration), and [RTX sensors (lidar / radar / acoustic)](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_rtx_to_experimental_rtx.html#isaacsim-sensors-rtx-migration).

## Related skills

- `isaac-camera`: deep dive on `RtxCamera` / `CameraSensor`, calibration, distortion.
- `isaac-sim-rendering`: headless capture pipeline, RT2, ACES tonemap.
- `physics-simulation`: scene config + physics sensors (`Contact`, `IMU`, etc.).
- `data-collection-sim`: static-scene SDG writer pipelines.
- `mobility-gen`: mobile-robot trajectory-driven SDG.

## 0. Vendor sensor catalog (`SUPPORTED_LIDAR_CONFIGS`)

`isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS` is the authoritative registry of vendor lidar/radar/acoustic USD assets. Keys are asset paths under `get_assets_root_path() + "/Isaac/Sensors/..."`; values are either a `set` of flat variant names (against the `"sensor"` variant set) or a list of explicit `{variant_set: value}` dicts. The companion constant `SUPPORTED_LIDAR_VARIANT_SET_NAME = "sensor"` names the default variant set.

| Manufacturer | Models (asset path under `/Isaac/Sensors/`) |
|---|---|
| NVIDIA | `NVIDIA/Example_Rotary.usda`, `Example_Rotary_2D.usda`, `Example_Solid_State.usda`, `Simple_Example_Solid_State.usda` |
| Ouster | `Ouster/OS0/OS0.usd`, `OS1/OS1.usd`, `OS2/OS2.usd`, `VLS_128/Ouster_VLS_128.usd` (rev6/rev7 variants, 32/128ch, 10/20 Hz, 512/1024/2048 res) |
| Hesai | `HESAI/XT32_SD10/HESAI_XT32_SD10.usd` (plus Pandar series via additional assets) |
| Velodyne | VLP/HDL series (under `Velodyne/`) |
| Robosense, SICK, Zvision, others | see registry / vendor folders |

For depth / stereo cameras (Intel RealSense D415/D435/D455/L515, Stereolabs ZED 2/2i/Mini/X), use the camera-style `RtxCamera` wrapper plus `SingleViewDepthCameraSensor`; see `isaac-camera`.

`Lidar.create(config=, variant=)` is the short form: `config` is the registry key minus path/extension; `variant` is required for assets that expose multiple variants (Ouster OS*, Hesai Pandar, ...).

## 0.1 Attaching a vendor sensor to a robot mount

### USDA (root scene)

```usda
over "Robot" {
    def Xform "sensor_mount" {
        double3 xformOp:translate = (0.0, 0.0, 0.35)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def "lidar" (
            prepend references = @${assetsRoot}/Isaac/Sensors/Ouster/OS1/OS1.usd@
            variants = { string sensor = "OS1_REV6_32ch20hz512res" }
        ) {}
    }
}
```

### Python

```python
from pxr import UsdGeom, Gf
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
mount = UsdGeom.Xform.Define(stage, "/World/Robot/sensor_mount")
UsdGeom.Xformable(mount.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.35))

sensor_prim = stage.DefinePrim("/World/Robot/sensor_mount/lidar")
sensor_prim.GetReferences().AddReference(
    assets_root + "/Isaac/Sensors/Ouster/OS1/OS1.usd"
)
vset = sensor_prim.GetVariantSets().GetVariantSet("sensor")
vset.SetVariantSelection("OS1_REV6_32ch20hz512res")

# Or use Lidar.create() at the mount path — it does both for you and
# returns the wrapper for runtime usage.
```

## 0.2 Custom scan patterns (USDA emitter-state arrays)

For custom sensors not in `SUPPORTED_LIDAR_CONFIGS`, define a USDA prim with `OmniSensorGenericLidarCoreAPI` and emitter-state arrays. Each beam is described by an azimuth, elevation, and fire-time entry; arrays can run into the tens of thousands for solid-state lidar.

```usda
def Xform "Lidar" (
    prepend apiSchemas = ["OmniSensorGenericLidarCoreAPI"]
)
{
    float minRange = 0.1
    float maxRange = 120.0
    float horizontalFov = 360.0
    float horizontalResolution = 0.1
    float verticalFov = 45.0
    float verticalResolution = 1.0

    # Emitter state arrays define exact beam directions and timing.
    float[] omni:sensor:Core:emitterState:s001:azimuthDeg   = [0.0, 0.1, 0.2, ...]
    float[] omni:sensor:Core:emitterState:s001:elevationDeg = [-22.5, -20.0, ...]
    int[]   omni:sensor:Core:emitterState:s001:fireTimeNs   = [0, 55, 110, ...]

    bool highLod    = true
    bool drawPoints = false
}
```

To wrap a custom prim from Python, call `Lidar("/World/.../my_lidar")` (no `config=` argument); the constructor wraps an existing prim instead of creating one.

## 1. Camera (multi-AOV capture)

`RtxCamera` + `CameraSensor` is the recommended path. Fall back to plain `UsdGeom.Camera` + Replicator render product when you don't need tick-rate / lens distortion / tiled batching. See `isaac-camera` for the full surface.

```python
from pxr import UsdGeom, Gf
import omni.replicator.core as rep

def create_camera_sensor(stage, path, focal_length=24.0, resolution=(1280, 720)):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(focal_length)
    cam.GetHorizontalApertureAttr().Set(36.0)
    cam.GetVerticalApertureAttr().Set(20.25)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    return rep.create.render_product(path, resolution)


def attach_annotators(rp):
    names = ["rgb", "distance_to_camera",
             "semantic_segmentation", "instance_segmentation",
             "motion_vectors",
             "bounding_box_2d_tight", "bounding_box_3d",
             "normals"]
    ann = {}
    for n in names:
        init = {"colorize": True} if "segmentation" in n else {}
        ann[n] = rep.AnnotatorRegistry.get_annotator(n, init_params=init)
        ann[n].attach([rp])
    return ann
```

Tiled multi-view / stereo / lens distortion: use `isaacsim.sensors.experimental.rtx.{TiledCameraSensor, SingleViewDepthCameraSensor, RtxCamera}`.

## 2. Lidar (RTX) — Writer-based GMO consumption

RTX sensors (`Lidar`, `Radar`, `Acoustic`) all produce a `GenericModelOutput` (GMO) buffer that the sensor scheduler emits asynchronously under multitick — zero or many GMO frames can land between consecutive `simulation_app.update()` / `app_utils.update()` calls. Polling `sensor.get_data("generic-model-output")` each tick drops or duplicates frames.

**Use a `Writer` for GMO.** The Replicator scheduler invokes `Writer.write` on every produced render product, so the consumer sees every event with no gaps. This is the pattern used by every canonical `source/standalone_examples/api/isaacsim.sensors.experimental.rtx/*` example (`inspect_lidar_gmo.py`, `inspect_radar_gmo.py`, `inspect_acoustic_gmo.py`, `lidar_robot_integration.py`, `resolve_lidar_object_ids.py`, `apply_nonvisual_materials.py`). The pattern below transposes directly to `Radar` / `RadarSensor` (Section 3) and `Acoustic` / `AcousticSensor` (Section 4).

```python
import omni.replicator.core as rep
from omni.replicator.core import Writer
from isaacsim.sensors.experimental.rtx import (
    Lidar, LidarSensor, parse_generic_model_output_data,
    SUPPORTED_LIDAR_CONFIGS,
)

lidar = Lidar.create(
    path="/World/lidar",
    config="OS1",                          # see SUPPORTED_LIDAR_CONFIGS (Section 0)
    variant="OS1_REV6_32ch20hz512res",
    aux_output_level="FULL",               # NONE | BASIC | EXTRA | FULL
    tick_rate=20.0,                        # Hz; None preserves authored value
    accumulate_outputs=True,               # True = full scan per output
)

# Sensor brings no annotators — the writer brings its own GMO annotator.
sensor = LidarSensor(lidar, annotators=[])


class GmoInspectWriter(Writer):
    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]

    def write(self, data):
        if "renderProducts" not in data:
            return
        for _rp, rp_data in data["renderProducts"].items():
            raw = rp_data.get("GenericModelOutput")
            if isinstance(raw, dict):
                raw = raw.get("data")
            gmo = parse_generic_model_output_data(raw)
            if gmo.numElements > 0:
                ...    # use gmo.x / gmo.y / gmo.z / gmo.scalar / gmo.matId / ...


rep.WriterRegistry.register(GmoInspectWriter)
sensor.attach_writer("GmoInspectWriter")

import omni.timeline
omni.timeline.get_timeline_interface().play()
while simulation_app.is_running():
    simulation_app.update()
```

For real-time viewport visualization (not data capture), attach the built-in `draw-point-cloud` writer instead:

```python
sensor.attach_writer("draw-point-cloud", size=0.05, color=[0, 1, 0.5, 1.0])
```

GMO decode helpers in `isaacsim.sensors.experimental.rtx`: `parse_generic_model_output_data`, `parse_stable_id_map_data`, `parse_object_ids`, `draw_annotator_data_to_image`.

For ROS 2 publishing of lidar scans, see `isaac-sim-ros2-bridge` (`ROS2PublishLaserScan`, `ROS2PublishPointCloud`).

## 3. Radar (RTX)

```python
from isaacsim.sensors.experimental.rtx import Radar, RadarSensor
radar  = Radar("/World/radar", tick_rate=20.0, aux_output_level="BASIC")
sensor = RadarSensor(radar, annotators=[])
```

Attach a `Writer` to consume GMO — see Section 2. Motion BVH must be enabled (`enable_motion_bvh=True` in the `SimulationApp` init, or `/renderer/raytracingMotion/enabled=true`).

## 4. Acoustic (RTX, ultrasonic)

```python
from isaacsim.sensors.experimental.rtx import Acoustic, AcousticSensor

acoustic = Acoustic(
    "/World/acoustic",
    tick_rate=30.0,
    aux_output_level="BASIC",
    attributes={
        "omni:sensor:WpmAcoustic:centerFrequency": 51200.0,
        "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.0, 0.0, 0.0),
    },
)
sensor = AcousticSensor(acoustic, annotators=[])
```

Attach a `Writer` to consume GMO — see Section 2. Acoustic GMO encodes signal ways: `gmo.x` is transmitter mount ID, `gmo.y` is receiver mount ID, `gmo.z` is channel ID, `gmo.scalar` is amplitude.

## 5. IMU (physics)

```python
from isaacsim.sensors.experimental.physics import IMU, IMUSensor
import isaacsim.core.experimental.utils.app as app_utils

imu = IMU.create(path="/World/Robot/imu", tick_rate=200.0)
sensor = IMUSensor(imu, annotators=["linear_acceleration",
                                    "angular_velocity",
                                    "orientation"])
app_utils.play(commit=True)
frame = sensor.get_data()                # IMUSensorReading
```

## 6. Contact / force (physics)

```python
from isaacsim.sensors.experimental.physics import Contact, ContactSensor
contact = Contact.create(
    path="/World/Robot/foot/contact",
    min_threshold=0.0, max_threshold=1e6, radius=-1,   # -1 = collision shape
)
sensor = ContactSensor(contact)
frame = sensor.get_data()
```

`EffortSensor` and `JointStateSensor` (same module) are runtime-only: wrap an existing joint prim by path, then call `get_data()` after `app_utils.play(commit=True)`. There is no separate authoring class for these — the joint already exists from the URDF/MJCF import.

```python
from isaacsim.sensors.experimental.physics import EffortSensor, JointStateSensor

effort = EffortSensor("/World/Robot/joint_arm_1")
joint  = JointStateSensor("/World/Robot/joint_arm_1")
```

## 7. Replicator domain randomization

Mark prims as randomization targets via `rep.functional.modify.semantics` (modern functional API) or per-prim `UsdSemantics` schemas via `isaacsim.core.experimental.utils.semantics.add_labels`. Then drive randomization with `rep.new_layer()` triggers.

```python
import omni.replicator.core as rep

with rep.new_layer():
    # Lights tagged with semantic ("type", "light")
    lights = rep.get.prims(semantics=[("type", "light")])
    with rep.trigger.on_frame(num_frames=100):
        with lights:
            rep.modify.attribute("intensity",
                                 rep.distribution.uniform(500, 5000))
            rep.modify.attribute("color",
                                 rep.distribution.uniform((0.8,)*3, (1.0,)*3))

    # Material swap on objects tagged ("class", "box")
    objects = rep.get.prims(semantics=[("class", "box")])
    with rep.trigger.on_frame():
        with objects:
            rep.randomizer.materials(rep.get.material("OmniPBR.*"))

rep.orchestrator.run()
```

For pre-built randomization behaviors (scatter, jitter, environment swap), look at `isaacsim.replicator.domain_randomization` and the `isaacsim.replicator.examples` standalone examples.

## 8. Writers

`omni.replicator.core` ships `BasicWriter`, `KittiWriter`, `CosmosWriter`. `isaacsim.replicator.writers` adds `PoseWriter` and `DataVisualizationWriter`. Deprecated: `DOPEWriter`, `YCBVideoWriter`, `PytorchWriter`.

```python
import omni.replicator.core as rep

writer = rep.WriterRegistry.get("KittiWriter")
writer.initialize(
    output_dir="/output/synthetic_dataset",
    semantic_types=["class"],
    write_binary_pointcloud=False,
    mapping={"box": "Car", "pallet": "Misc"},
)
writer.attach([render_product])
```

## 9. Workflows

| Workflow | Module / example |
|---|---|
| Static-scene SDG with randomization | `data-collection-sim` skill + `isaacsim.replicator.examples` (`sdg_getting_started_0[1-5].py`, `sdg_workflow_0[12].py`) |
| Mobile-robot trajectory record + replay | `mobility-gen` skill + `isaacsim.replicator.mobility_gen` |
| Grasp dataset generation | `isaacsim.replicator.grasping` (`GraspingManager`, `GraspPhase`) — see `source/standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py` |
| Episode record + replay | `isaacsim.replicator.episode_recorder` |
| Teleoperation record + replay | `isaacsim.replicator.teleop` |
| Cosmos warehouse rendering | `isaacsim.replicator.examples/cosmos_writer_simple.py` and `source/standalone_examples/replicator/cosmos_writer_warehouse.py` |

## Gotchas

- **RTX-GMO sensors (lidar, radar, acoustic): use a `Writer`, not `get_data`.** Under multitick the sensor scheduler emits GMO frames asynchronously of `simulation_app.update()`; polling `sensor.get_data("generic-model-output")` drops or duplicates frames. `attach_writer` is event-driven and sees every output. `sensor.get_data(...)` remains the documented surface for **camera AOVs** (`rgb`, `distance_to_image_plane`, ...) on `CameraSensor`, and for physics sensors (`IMU`, `Contact`, `Effort`, `JointState`).
- `Lidar.create(config=..., variant=...)` requires the variant for assets that expose multiple sensor variants; the registry value tells you the set.
- `aux_output_level` levels are modality-specific: lidar supports `NONE`/`BASIC`/`EXTRA`/`FULL`; radar/acoustic only `NONE`/`BASIC`; `RtxCamera` only `NONE`.
- High-LOD (`highLod=true`) mode produces more points; slower. Set `drawPoints=false` in headless mode for performance.
- The "harmless" `usdrt.population.plugin Unhandled attribute type VtArray<std::string>` log message is expected when `aux_output_level` is set; the Replicator pipeline still picks the attribute up.
- Attach Replicator annotators before `rep.orchestrator.run()`.
- RTX sensors auto-enable `omni.sensors.nv.lidar` / `.nv.radar` / `.nv.acoustic` when loading `isaacsim.sensors.experimental.rtx`; no manual enable needed.
- Physics sensors need `app_utils.play(commit=True)` (or `timeline.play()`) before `get_data()`. Sensors do not stream while the timeline is paused.
- Randomization triggers accumulate; isolate each pass with `rep.new_layer()`.
- Use `rep.functional.modify.semantics(prim, {"class": "..."}, mode="add")` for the modern functional API; older `prims.create_prim(semantic_label=...)` paths are still tolerated but not the documented current path.
