# Overview

This extension provides the UI components for the URDF Importer. It includes:

- **Asset Importer Integration**: Registers URDF file type with the Asset Importer
- **Options Panel**: Configuration for links, colliders, and import settings

![URDF Importer UI](../data/preview.png)

## Dependencies

This extension depends on:
- `isaacsim.asset.importer.urdf`: Core URDF parsing and importing functionality

## Usage

### Accessing the URDF Importer

1. **Enable the Extension**: Go to the Extension Manager menu and enable `isaacsim.asset.importer.urdf.ui`
2. **Open the Asset Importer**: Navigate to **File > Import** in the main menu
3. **Select URDF File**: In the file picker, navigate to and select a `.urdf` or `.URDF` file
4. **Configure Import Settings**: Use the options panel on the right side of the file picker to configure import settings
5. **Import**: Click the **Import** button to begin the import process

### UI Options

The URDF Importer UI provides three main configuration sections:

#### Model Section
- **USD Output**: Specify the output directory for the generated USD file. By default, the USD file is saved in the same directory as the imported URDF file.
- **ROS Package List**: Add ROS package name/path mappings to resolve `package://` URLs in the URDF file. Click "Add Row" to add multiple package mappings.

#### Colliders Section
- **Collision From Visuals**: When enabled, collision geometry is generated from visual geometries instead of using the collision meshes defined in the URDF.
- **Collision Type**: (Visible when "Collision From Visuals" is enabled) Select the type of collision geometry to generate:
  - **Convex Hull**: Creates a convex hull around the visual mesh
  - **Convex Decomposition**: Decomposes the mesh into multiple convex shapes
  - **Bounding Sphere**: Uses a sphere that bounds the visual mesh
  - **Bounding Cube**: Uses a box that bounds the visual mesh
- **Allow Self-Collision**: When enabled, allows the robot to collide with itself. Note that this can cause instability if collision meshes between links are self-intersecting.

#### Options Section
- **Merge Mesh**: When enabled, merges meshes where possible to optimize the model and reduce memory usage.
- **Debug Mode**: When enabled, keeps intermediate output files and generates additional logging information for debugging purposes.
