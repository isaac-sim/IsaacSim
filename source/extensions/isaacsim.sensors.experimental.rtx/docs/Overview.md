# Overview

The isaacsim.sensors.experimental.rtx extension provides experimental Python APIs for RTX-based sensor simulation in Isaac Sim, covering lidar, radar, acoustic (ultrasonic), and camera sensors. Each sensor type is split into an **authoring** class for USD prim creation and configuration, and a **runtime sensor** class for attaching annotators and retrieving data at simulation time.

## Key Components

### Authoring classes

Authoring classes inherit from `_SensorAuthoring` (which inherits `XformPrim`) and manage the underlying USD sensor prim. They handle prim creation (or wrapping existing prims), schema application, attribute setting, and transform operations.

- {class}`Lidar <isaacsim.sensors.experimental.rtx.Lidar>` — Creates or wraps `OmniLidar` prims. Supports creating from known configurations via {data}`SUPPORTED_LIDAR_CONFIGS <isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS>`. Accepts `accumulate_outputs` (default ``True``).
- {class}`Radar <isaacsim.sensors.experimental.rtx.Radar>` — Creates or wraps `OmniRadar` prims. Requires Motion BVH to be enabled (`/renderer/raytracingMotion/enabled`).
- {class}`Acoustic <isaacsim.sensors.experimental.rtx.Acoustic>` — Creates or wraps `OmniAcoustic` prims. Automatically applies multi-instance schemas (`OmniSensorWpmAcousticSensorMountAPI`, `OmniSensorWpmAcousticRxGroupAPI`) when attributes with matching prefixes are provided.
- {class}`RtxCamera <isaacsim.sensors.experimental.rtx.RtxCamera>` — Creates or wraps USD Camera prims with `OmniSensorAPI` applied. Provides a `.camera` property for accessing optical parameters (focal length, clipping range, aperture, etc.).

All authoring classes accept:

- `tick_rate` — Sensor tick rate in Hz (default ``0`` for autotrigger).
- `aux_output_level` — Auxiliary data level for GenericModelOutput (default ``"NONE"``). Valid values are modality-specific: ``NONE``/``BASIC``/``EXTRA``/``FULL`` for Lidar, ``NONE``/``BASIC`` for Radar and Acoustic. RtxCamera only supports ``NONE``.
- `schemas` — Additional API schemas to apply to the prim (e.g. ``["OmniLensDistortionOpenCvFisheyeAPI"]``). Supports multi-instance schemas via ``"SchemaName:instanceName"`` syntax.
- `attributes` — USD attributes to set on the prim after schema application.

### Runtime sensor classes

Runtime sensor classes wrap an authoring object, create a Replicator render product, and manage annotator attachment and data retrieval.

**RTX sensors** (lidar, radar, acoustic) support `generic-model-output` and `stable-id-map` annotators:

- {class}`LidarSensor <isaacsim.sensors.experimental.rtx.LidarSensor>` — Wraps a `Lidar` object (or creates one from a path).
- {class}`RadarSensor <isaacsim.sensors.experimental.rtx.RadarSensor>` — Wraps a `Radar` object (or creates one from a path).
- {class}`AcousticSensor <isaacsim.sensors.experimental.rtx.AcousticSensor>` — Wraps an `Acoustic` object (or creates one from a path).

**Camera sensors** support standard rendering annotators (RGB, depth, normals, segmentation, bounding boxes, etc.):

- {class}`CameraSensor <isaacsim.sensors.experimental.rtx.CameraSensor>` — Wraps a `RtxCamera` object. Requires a `resolution` parameter (``(height, width)``).
- {class}`TiledCameraSensor <isaacsim.sensors.experimental.rtx.TiledCameraSensor>` — Batched rendering of multiple cameras into a single tiled texture. Supports both tiled and per-camera batched output.
- {class}`SingleViewDepthCameraSensor <isaacsim.sensors.experimental.rtx.SingleViewDepthCameraSensor>` — Extends `CameraSensor` with stereoscopic depth simulation via `OmniSensorDepthSensorSingleViewAPI`.

### Lidar configuration registry

**Supported USD lidar assets and variants.** The package exports {data}`SUPPORTED_LIDAR_CONFIGS <isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS>` (paths to known Isaac Sim lidar assets mapped to optional variant names) and {data}`SUPPORTED_LIDAR_VARIANT_SET_NAME <isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_VARIANT_SET_NAME>` (expected variant set name on those prims).

### Parser utilities

- {func}`parse_generic_model_output_data <isaacsim.sensors.experimental.rtx.parse_generic_model_output_data>` — Decodes `generic-model-output` annotator data into a `GenericModelOutput` structure provided by the bundled `isaacsim.sensors.experimental.rtx.generic_model_output` module.
- {func}`parse_stable_id_map_data <isaacsim.sensors.experimental.rtx.parse_stable_id_map_data>` — Decodes `stable-id-map` data into a mapping from stable object IDs to prim paths.
- {func}`parse_object_ids <isaacsim.sensors.experimental.rtx.parse_object_ids>` — Extracts 128-bit object IDs from a `GenericModelOutput.objId` buffer as Python ints matching the keys from `parse_stable_id_map_data`.
- {func}`draw_annotator_data_to_image <isaacsim.sensors.experimental.rtx.draw_annotator_data_to_image>` — Converts camera annotator data to BGR NumPy arrays for visualization.

### Auxiliary modules

The extension ships {mod}`isaacsim.sensors.experimental.rtx.generic_model_output` and {mod}`isaacsim.sensors.experimental.rtx.sensor_checker` alongside the main package. The former defines the binary layout used by `parse_generic_model_output_data`; the latter provides helpers such as `SensorCheckerUtil` and `ModelInfo` for working with supported sensor assets (see extension tests for typical usage).

### Settings

**Kit settings contributed by this extension.** Defaults and inline comments live in `config/extension.toml` under `[settings]`. The keys cover GPU-resident lidar and radar return buffers (`app.sensors.nv.lidar.outputBufferOnGPU`, `app.sensors.nv.radar.outputBufferOnGPU`), optional non-visual material semantics from USD (`rtx.materialDb.nonVisualMaterialCSV.enabled`, `rtx.materialDb.nonVisualMaterialSemantics.prefix`), and Hydra timeline use for RTX sensor models (`rtx.rtxsensor.useHydraTimeAlways`).

## Code examples

### Lidar

```python
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
import isaacsim.core.experimental.utils.app as app_utils

# authoring: create a lidar prim from a known config
lidar = Lidar.create(path="/World/lidar", config="OS1", variant="OS1_REV6_32ch20hz512res",
                     aux_output_level="FULL")

# runtime: attach annotators and retrieve data
sensor = LidarSensor(lidar, annotators=["generic-model-output"])
app_utils.play(commit=True)

data, _ = sensor.get_data("generic-model-output")
gmo = parse_generic_model_output_data(data)
```

### Radar

```python
from isaacsim.sensors.experimental.rtx import Radar, RadarSensor

radar = Radar("/World/radar", tick_rate=20.0, aux_output_level="BASIC")
sensor = RadarSensor(radar, annotators=["generic-model-output"])
```

### Acoustic

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
sensor = AcousticSensor(acoustic, annotators=["generic-model-output"])
```

### Camera

```python
from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor

cam = RtxCamera("/World/cam", tick_rate=30.0)
cam.camera.set_focal_lengths(24.0)
cam.camera.set_clipping_ranges(0.01, 1000.0)

sensor = CameraSensor(cam, resolution=(480, 640), annotators=["rgb", "distance_to_image_plane"])
```

### Camera with lens distortion

```python
from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
from pxr import Gf

cam = RtxCamera(
    "/World/cam",
    schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
    attributes={
        "omni:lensdistortion:opencvFisheye:fx": 500.0,
        "omni:lensdistortion:opencvFisheye:fy": 500.0,
        "omni:lensdistortion:opencvFisheye:cx": 640.0,
        "omni:lensdistortion:opencvFisheye:cy": 360.0,
        "omni:lensdistortion:opencvFisheye:k1": 0.05,
        "omni:lensdistortion:opencvFisheye:imageSize": Gf.Vec2i(1280, 720),
    },
)
```

## Known warnings

When `aux_output_level` is set, the following warning may appear in the log:

```
[usdrt.population.plugin] [UsdNoticeHandler] Unhandled attribute type VtArray<std::string>
    (prim attribute: _replicator:rendervar:GenericModelOutput:channels)
```

This is harmless. The `usdrt` Fabric cache does not mirror `VtArray<std::string>` attributes, but the attribute is correctly set on the USD prim and read by the Replicator pipeline.

## Integration

Dependencies include **isaacsim.core.experimental.prims** (transform and prim utilities), **isaacsim.core.experimental.objects** (Camera prim wrapper), **omni.replicator.core** (annotators and writers), **omni.sensors.nv.lidar**, **omni.sensors.nv.radar**, **omni.sensors.nv.acoustic**, **omni.sensors.nv.common**, **omni.sensors.nv.ids**, **omni.usd.schema.omni_sensors**, and **isaacsim.storage.native** for asset paths. Enable the extension from **Window > Extensions** and turn on `isaacsim.sensors.experimental.rtx`.
