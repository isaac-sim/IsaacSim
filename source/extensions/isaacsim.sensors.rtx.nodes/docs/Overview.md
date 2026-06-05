# Overview

The `isaacsim.sensors.rtx.nodes` extension provides OmniGraph nodes for
extracting and visualising point cloud data from RTX sensors (Lidar and Radar).

It is a companion to `isaacsim.sensors.experimental.rtx` and keeps OmniGraph /
native-plugin dependencies out of that base extension.

## Provided Components

### OmniGraph Node

- **IsaacExtractRTXSensorPointCloud** — reads the `GenericModelOutput` buffer
  produced by an `OmniLidar` or `OmniRadar` prim and outputs a Cartesian
  (x, y, z) point cloud.  Spherical-to-Cartesian conversion is performed in
  parallel when the buffer contains spherical coordinates.

### Annotator

- **IsaacExtractRTXSensorPointCloud** — Replicator annotator backed by the node
  above.  Attach it to a render product to get per-frame point cloud data.

### Writer

- **RtxSensorDebugDrawPointCloud** — Replicator writer that feeds the annotator
  output into the `DebugDrawPointCloud` node for viewport visualisation.
  Works with both Lidar and Radar sensors.

## Quick Start

```python
import omni.replicator.core as rep
from isaacsim.sensors.experimental.rtx import Lidar

lidar = Lidar.create("/World/lidar", config="Example_Rotary")
render_product = rep.create.render_product(lidar.paths[0], resolution=(1, 1))

writer = rep.writers.get("RtxSensorDebugDrawPointCloud")
writer.initialize(size=0.05, color=[0.0, 1.0, 0.5, 1.0])
writer.attach([render_product.path])
```
