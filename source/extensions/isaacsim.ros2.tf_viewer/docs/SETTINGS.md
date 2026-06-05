```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## exts."isaacsim.ros2.tf_viewer".cpp
- **Default Value**: true
- **Description**: Whether to use the C++ implementation to listen to the tf and process the incoming data; if false, the Python implementation will be used (may degrade performance considerably).

## exts."isaacsim.ros2.tf_viewer".include_root_frame
- **Default Value**: true
- **Description**: Whether to include the pose of the root frame when drawing the TF in the viewport if not listed; root frame's pose is defined with position (0,0,0) and orientation (0,0,0,1) as xyzw quaternion.
