# Overview

```{deprecated} 6.0.0
This extension is deprecated. Use {mod}`isaacsim.sensors.experimental.rtx` instead, which provides
{class}`RtxCamera <isaacsim.sensors.experimental.rtx.RtxCamera>`,
{class}`CameraSensor <isaacsim.sensors.experimental.rtx.CameraSensor>`,
{class}`TiledCameraSensor <isaacsim.sensors.experimental.rtx.TiledCameraSensor>`, and
{class}`SingleViewDepthCameraSensor <isaacsim.sensors.experimental.rtx.SingleViewDepthCameraSensor>`.
```

The isaacsim.sensors.camera extension provides APIs for creating and managing camera sensors in Isaac Sim. This extension enables programmatic control of camera properties, lens distortion models, and batch processing of camera data for robotics simulation and computer vision applications.

## Key Components

### {class}`Camera <isaacsim.sensors.camera.Camera>`

The {class}`Camera <isaacsim.sensors.camera.Camera>` class provides high-level functions for managing individual camera prims and their properties. It encapsulates a single camera sensor with comprehensive control over camera parameters, lens distortion models, and data collection through annotators.

**{class}`Camera <isaacsim.sensors.camera.Camera>` properties include:**
- Resolution, focal length, and aperture settings
- Multiple lens distortion models (OpenCV pinhole/fisheye, F-theta, Kannala-Brandt, etc.)
- Projection modes (perspective/orthographic) and stereo configurations
- Clipping ranges and shutter properties for motion blur

**Data collection capabilities:**
- RGB, depth, and pointcloud data acquisition
- Segmentation masks (semantic, instance, instance ID)
- Bounding boxes (2D tight/loose, 3D)
- Motion vectors, normals, and occlusion data
- Configurable annotator attachment with device-specific tensor placement

### {class}`CameraView <isaacsim.sensors.camera.CameraView>`

The {class}`CameraView <isaacsim.sensors.camera.CameraView>` class extends XFormPrim to handle batched/tiled data from multiple cameras simultaneously. It provides efficient processing for scenarios requiring multiple camera views, such as multi-agent robotics or panoramic vision systems.

**Tiled rendering features:**
- Batch processing of camera data as single tiled images or separate batches
- Configurable output annotators for different sensor types
- Support for multiple coordinate systems (world, ROS, USD)
- Efficient memory management with pre-allocated output arrays

**Coordinate system support:**
- World coordinates (+Z up, +X forward)
- ROS coordinates (+Y up, +Z forward)
- USD coordinates (+Y up, -Z forward)

## Functionality

### Lens Distortion Models

The extension supports multiple lens distortion models for realistic camera simulation:
- OpenCV pinhole and fisheye models
- F-theta distortion for wide-angle lenses
- Kannala-Brandt K3 for fisheye cameras
- Radial-Tangential Thin Prism for complex distortions
- LUT-based distortion using texture lookups

### Annotator Integration

Both {class}`Camera <isaacsim.sensors.camera.Camera>` and {class}`CameraView <isaacsim.sensors.camera.CameraView>` integrate with Omniverse Replicator annotators to provide various computer vision data types. The extension handles annotator lifecycle management and provides normalized access keys for consistent data retrieval across different annotator variants.

### Coordinate Transformations

The extension provides transformation utilities for converting between different coordinate systems and performing perspective projections. This includes methods for projecting 3D world points to image coordinates and inverse projection from image pixels to 3D points using depth information.
