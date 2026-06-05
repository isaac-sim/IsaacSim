# Overview

To enable this extension, go to the Extension Manager menu and enable isaacsim.asset.importer.mjcf.ui extension.
This extension provides a graphical user interface for importing MJCF files through the Asset Importer window.


## High Level Code Overview

### Python
The `MJCF Importer UI` extension provides a user interface for the MJCF importer. It depends on
`isaacsim.asset.importer.mjcf` for the core import functionality and uses `MJCFImporterConfig` and
`MJCFImporter` classes.

The extension registers itself with the Asset Importer system (`omni.kit.tool.asset_importer`) to handle
`.xml` files. When a user selects an MJCF file through the UI, the extension:

1. Creates a `MJCFImporterConfig` instance to store import settings
2. Provides UI widgets (in `python/impl/option_widget.py`) that allow users to configure:
   - Collision type (Convex Hull, Convex Decomposition, Bounding Sphere, Bounding Cube)
   - Merge mesh option
   - Debug mode
   - Collision from visuals
   - Allow self-collision
3. Uses `MJCFImporter` from the core extension to perform the actual import
4. Opens the resulting USD file in the current stage


### UI Options

The MJCF importer UI provides several configuration options in the right panel of the Asset Importer window.
See `../data/preview.png` for a visual reference of the interface.

#### Output Options

- **USD Output**: Specifies where the generated USD file will be saved. By default, this is set to "Same as Imported Model(Default)",
  which saves the USD file in the same directory as the source MJCF file. Users can click the folder icon to select a different
  output location.

#### Colliders Options

- **Collision From Visuals**: When enabled, collision geometry is generated from the visual meshes in the MJCF file. This is useful
  when the MJCF file doesn't have explicit collision geometry defined. When this option is checked, the Collision Type dropdown
  becomes visible.

- **Collision Type**: This dropdown appears when "Collision From Visuals" is enabled. It allows users to select the type of
  collision approximation to use:
  - **Convex Hull**: Creates a convex hull around the visual mesh
  - **Convex Decomposition**: Decomposes the mesh into multiple convex pieces for more accurate collision detection
  - **Bounding Sphere**: Uses a simple bounding sphere approximation
  - **Bounding Cube**: Uses a simple bounding box approximation

#### General Options
- **Allow Self-Collision**: When enabled, allows the robot model to collide with itself. This can be useful for certain simulation
  scenarios but may cause instability if collision meshes between links are self-intersecting.


- **Merge Mesh**: When enabled, merges meshes where possible to optimize the model. This can reduce the number of prims in the
  resulting USD file and improve performance.

- **Debug Mode**: When enabled, activates debug mode to preserve the intemediate files and asset transformer reports
