---
name: isaac-camera
description: >
  Camera creation, configuration, calibration, render products, lens
  distortion, and runtime sensor wrapping in Isaac Sim 6 / Kit 110.
  Modern path: `isaacsim.sensors.experimental.rtx.RtxCamera` (authoring) +
  `CameraSensor` / `TiledCameraSensor` / `SingleViewDepthCameraSensor`
  (runtime) on top of `UsdGeomCamera`. Covers AOVs / annotators,
  OpenCV / fisheye / LUT lens distortion via `OmniLensDistortion*API`
  schemas, OpenCV intrinsics conversion, animation, and Replicator
  randomization. Use when creating/configuring camera prims, attaching
  annotators, applying distortion models, converting real-world
  calibration to Omniverse units, animating cameras, or generating
  synthetic data with Replicator.
---

# Camera (Isaac Sim 6 / Kit 110)

Two layers stack:

1. **`UsdGeomCamera`** — the USD camera prim (focal length, apertures, clipping, transform).
2. **`isaacsim.sensors.experimental.rtx`** — the runtime sensor wrapper (authoring class + runtime sensor class). Use this for any new code that captures frames, attaches annotators, or applies lens distortion.

The legacy `isaacsim.sensors.camera.Camera` class still works but the extension itself is a stub (no Python). The implementation moved to `isaacsim.sensors.experimental.rtx`; new code should use that.

> **Migration:** see the [`Migration from isaacsim.sensors.camera.Camera`](#migration-from-isaacsimsensorscameracamera) table below, plus the official [camera migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_camera_to_experimental_rtx.html#isaacsim-sensors-camera-migration) for the full API change set.

## When to use

- Create or configure camera prims in USD stages.
- Attach RGB / depth / segmentation / bbox annotators for capture.
- Apply lens distortion (OpenCV pinhole, OpenCV fisheye, LUT).
- Convert real-world calibration (OpenCV / ROS) to Omniverse units.
- Build stereo, multi-sensor, or tiled multi-camera rigs.
- Animate cameras (keyframes, motion paths, orbit, fly-through).
- Generate synthetic data with Replicator randomization.
- Set up Metropolis cameras for MEGA scenarios.

## Fundamentals

### Unit system

OpenUSD expresses optical properties in **tenths of a scene unit**.

| Property | Units | Notes |
|---|---|---|
| Focal length | tenths of scene unit | `35` = 35 mm when scene units are cm |
| Horizontal aperture | tenths of scene unit | sensor / film width |
| Vertical aperture | tenths of scene unit | sensor / film height |
| Focus distance | scene units | perfectly-sharp distance |
| Clipping range | scene units | near / far clip planes |

Meter stage: a 25 mm focal length is `0.025`.

### Coordinate system

Camera looks down `-Z`; `+Y` is up, `+X` is right. Same regardless of stage up-axis.

## Path A — RtxCamera + CameraSensor (recommended)

`RtxCamera` wraps a `UsdGeom.Camera` prim with `OmniSensorAPI` applied; `CameraSensor` builds a Replicator render product on top and exposes annotators. Both extend the `isaacsim.core.experimental.prims.XformPrim` family, so the standard pose / transform methods apply.

```python
from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
import isaacsim.core.experimental.utils.app as app_utils

cam = RtxCamera(
    "/World/cam",
    tick_rate=30.0,                       # 0 = autotrigger
    aux_output_level="NONE",              # RtxCamera supports NONE only
)
cam.camera.set_focal_lengths(24.0)
cam.camera.set_clipping_ranges(0.01, 1000.0)

sensor = CameraSensor(
    cam,
    resolution=(480, 640),                # (height, width)
    annotators=["rgb", "distance_to_image_plane"],
)
app_utils.play(commit=True)
data = sensor.get_data("rgb")
```

`TiledCameraSensor` batches many cameras into one render call; useful for multi-view RL data collection. `SingleViewDepthCameraSensor` simulates stereo depth via `OmniSensorDepthSensorSingleViewAPI`.

### Lens distortion (OpenCV fisheye / pinhole)

Set via API schemas + USD attributes at authoring time. The schema and attribute names are stable; use them through `RtxCamera`'s `schemas` and `attributes` constructor args, or `Apply` them on the camera prim directly.

```python
from isaacsim.sensors.experimental.rtx import RtxCamera
from pxr import Gf

cam = RtxCamera(
    "/World/fisheye",
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

Equivalent OpenCV-pinhole schema: `OmniLensDistortionOpenCvPinholeAPI` (attributes `omni:lensdistortion:opencvPinhole:{fx,fy,cx,cy,k1,k2,p1,p2,...}`). LUT-based distortion uses `OmniLensDistortionLutAPI`.

## Path B — `UsdGeomCamera` + `omni.replicator.core` (no RTX wrapper)

When you don't need RTX sensor features (lens distortion, ISP, tick-rate control), straight USD + Replicator works:

```python
from pxr import UsdGeom, Gf
import omni.replicator.core as rep

cam = UsdGeom.Camera.Define(stage, "/World/Cam")
cam.GetFocalLengthAttr().Set(24.0)
cam.GetHorizontalApertureAttr().Set(36.0)            # full-frame 35 mm equiv
cam.GetVerticalApertureAttr().Set(20.25)
cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))

rp = rep.create.render_product(cam.GetPath(), (1920, 1080))
rgb     = rep.AnnotatorRegistry.get_annotator("rgb")
depth   = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
semseg  = rep.AnnotatorRegistry.get_annotator("semantic_segmentation",
                                              init_params={"colorize": True})
normals = rep.AnnotatorRegistry.get_annotator("normals")
for ann in (rgb, depth, semseg, normals):
    ann.attach([rp])
```

For pose authoring use `isaacsim.core.experimental.objects.Camera` (wraps `UsdGeom.Camera` with the standard pose / scale API), or the look-at helper:

```python
from isaacsim.core.experimental.objects import Camera

cam = Camera(paths="/World/Cam", positions=[2.0, 2.0, 2.0])
cam.set_world_poses(positions=[[2.0, 2.0, 2.0]],
                    orientations=[[1.0, 0.0, 0.0, 0.0]])
```

## OpenCV intrinsics -> USD

```python
from pxr import UsdGeom

def opencv_to_usd_camera(stage, path, fx, fy, cx, cy, image_w, image_h,
                         sensor_w_m=0.036):
    """OpenCV intrinsics -> USD camera on a meter-stage."""
    focal_length_m = fx * sensor_w_m / image_w
    h_aperture     = sensor_w_m
    v_aperture     = fy * sensor_w_m / fx   # preserve aspect
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(focal_length_m)
    cam.GetHorizontalApertureAttr().Set(h_aperture)
    cam.GetVerticalApertureAttr().Set(v_aperture)
    # Principal-point offset:
    cam.GetHorizontalApertureOffsetAttr().Set((cx - image_w / 2) * sensor_w_m / image_w)
    cam.GetVerticalApertureOffsetAttr().Set((cy - image_h / 2) * sensor_w_m / image_w)
    return cam
```

## Animation

```python
from pxr import Usd, Gf

for frame, pos in enumerate(camera_positions):
    cam.GetPrim().GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(*pos), Usd.TimeCode(frame)
    )
```

## Replicator randomization

```python
import omni.replicator.core as rep

with rep.new_layer():
    cameras = rep.get.prims(path_pattern="/World/Camera.*")
    with rep.trigger.on_frame(num_frames=200):
        with cameras:
            rep.modify.pose(
                position=rep.distribution.uniform((-5, -5, 2), (5, 5, 8)),
                look_at="/World/Target",
            )
```

## Sensor checker / supported configs

For asset-driven lidar / camera configs and validation helpers, see `isaacsim.sensors.experimental.rtx.SUPPORTED_LIDAR_CONFIGS` and the `sensor_checker` module bundled with the extension. The Replicator AOV annotator names match the standard registry: `rgb`, `distance_to_image_plane`, `distance_to_camera`, `normals`, `semantic_segmentation`, `instance_segmentation`, `bounding_box_2d_tight`, `bounding_box_2d_loose`, `bounding_box_3d`, `camera_params`, `pointcloud`, `occlusion`.

## Migration from `isaacsim.sensors.camera.Camera`

Full guide: [`isaacsim.sensors.camera` → `isaacsim.sensors.experimental.rtx`](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_camera_to_experimental_rtx.html#isaacsim-sensors-camera-migration).

| Old | New |
|---|---|
| `isaacsim.sensors.camera.Camera("/World/cam", resolution=(1280, 720))` | `RtxCamera("/World/cam")` + `CameraSensor(cam, resolution=(720, 1280), annotators=["rgb"])` |
| `camera.set_focal_length(24)` | `cam.camera.set_focal_lengths(24.0)` |
| `camera.set_clipping_range(0.01, 1000)` | `cam.camera.set_clipping_ranges(0.01, 1000)` |
| `camera.initialize()` then `camera.get_current_frame()` | `app_utils.play(commit=True)` then `sensor.get_data(annotator)` |
| Carb-settings distortion (`/<path>/distortionModel`) | `OmniLensDistortion*API` schemas on the camera prim |
