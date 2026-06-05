```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## app.sensors.nv.lidar.profileBaseFolder
- **Default Value**: [
    "${omni.sensors.nv.common}/data/lidar/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/HESAI/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/NVIDIA/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/Ouster/OS0/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/Ouster/OS1/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/Ouster/OS2/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/SICK/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/SLAMTEC/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/Velodyne/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/ZVISION/",
    "${isaacsim.sensors.rtx}/data/lidar_configs/"
]
- **Description**: List of directories which renderer will search to find Lidar profile for (deprecated) camera-based Lidar.

## app.sensors.nv.lidar.outputBufferOnGPU
- **Default Value**: false
- **Description**: Keeps Lidar return buffer on GPU for post-processing operations.

## app.sensors.nv.radar.outputBufferOnGPU
- **Default Value**: false
- **Description**: Keeps Radar return buffer on GPU for post-processing operations.

## rtx.materialDb.nonVisualMaterialCSV.enabled
- **Default Value**: false
- **Description**: Enables non-visual materials using USD attributes for material database processing.

## rtx.materialDb.nonVisualMaterialSemantics.prefix
- **Default Value**: "omni:simready:nonvisual"
- **Description**: Specifies the USD attribute prefix for non-visual material semantics identification.

## rtx.rtxsensor.useHydraTimeAlways
- **Default Value**: true
- **Description**: Uses Hydra time from omni.timeline in RTX Sensor models for time synchronization.
