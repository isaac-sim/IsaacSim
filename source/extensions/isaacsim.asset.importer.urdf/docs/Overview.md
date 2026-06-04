# Overview

```{image} ../../../../source/extensions/isaacsim.asset.importer.urdf/data/preview.png
```

To enable this extension, go to the Extension Manager menu and enable isaacsim.asset.importer.urdf extension.

## High Level Code Overview

### Python
The URDF Importer extension provides a Python API for importing URDF files into USD format. The main classes are:

- **`URDFImporterConfig`**: A dataclass that stores configuration settings for the import operation, including:
  - `urdf_path`: Path to the URDF file to import
  - `usd_path`: Directory where the USD file will be saved
  - `merge_mesh`: Whether to merge meshes for optimization
  - `debug_mode`: Whether to enable debug mode with intermediate outputs
  - `collision_from_visuals`: Whether to generate collision geometry from visual geometries
  - `collision_type`: Type of collision geometry to use
  - `allow_self_collision`: Whether to allow self-collision
  - `ros_package_paths`: List of ROS package name/path mappings for resolving package:// URLs

- **`URDFImporter`**: The main importer class that converts URDF files to USD format.

The import workflow is as follows:
1. Create a `URDFImporterConfig` instance with the desired settings
2. Create a `URDFImporter` instance with the config
3. Call `import_urdf()` which:
   - Uses `urdf-usd-converter` to convert the URDF file to an intermediate USD format
   - Applies post-processing operations (rigid body schemas, joint schemas, mesh merging, collision geometry generation, etc.)
   - Runs the asset transformer profile to structure the output according to Isaac Sim conventions
   - Returns the path to the final USD file

When the import button is pressed in the UI, the extension creates a `URDFImporter` instance and calls `import_urdf()` with the user's configuration settings.
