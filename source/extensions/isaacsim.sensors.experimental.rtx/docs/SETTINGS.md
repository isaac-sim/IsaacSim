```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

### app.sensors.nv.lidar.outputBufferOnGPU
- **Default Value**: false
- **Description**: Renderer keeps Lidar return buffer on GPU for post-processing.

### app.sensors.nv.radar.outputBufferOnGPU
- **Default Value**: false
- **Description**: Renderer keeps Radar return buffer on GPU for post-processing.

### rtx.materialDb.nonVisualMaterialCSV.enabled
- **Default Value**: false
- **Description**: Enable non-visual materials using USD attributes.

### rtx.materialDb.nonVisualMaterialSemantics.prefix
- **Default Value**: "omni:simready:nonvisual"
- **Description**: Specify the non-visual material USD attribute prefix.

### rtx.rtxsensor.useHydraTimeAlways
- **Default Value**: true
- **Description**: Use Hydra time (`omni.timeline`) in RTX sensor models. Applies only if multi-tick rendering is disabled.
