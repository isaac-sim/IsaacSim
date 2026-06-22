# Overview

The URDF Importer UI extension adds a graphical import workflow for robot URDF files in Isaac Sim. Use it from **File > Import** to convert a URDF into USD and place the robot on the stage. Core parsing and conversion are handled by the companion `isaacsim.asset.importer.urdf` extension.

![URDF Importer UI](../data/preview.png)

## Dependencies

- **isaacsim.asset.importer.urdf** — converts URDF files to USD

## Usage

### Accessing the URDF Importer

1. In **Window > Extensions**, enable **isaacsim.asset.importer.urdf.ui** if it is not already loaded.
2. Choose **File > Import** and select a `.urdf` file.
3. Adjust settings in the panel on the right side of the import window.
4. Click **Import** to add the robot to the stage.

### Import settings

The options panel is organized into three sections you can expand or collapse:

#### Model

- **USD Output** — Where to save the generated USD. The default **Same as Imported Model (Default)** saves next to your URDF file. Click **Select File** to pick another folder.
- **ROS Package List** — Maps ROS package names to folders on disk so `package://` links in the URDF resolve correctly. Click **Add Row** to add entries; use the row delete control to remove them.

#### Colliders

- **Collision From Visuals** — Builds collision shapes from the robot’s visual meshes instead of collision meshes in the URDF. When checked, **Collision Type** appears below.
- **Collision Type** — How to approximate collision when generating from visuals:
  - **Convex Hull**
  - **Convex Decomposition**
  - **Bounding Sphere**
  - **Bounding Cube**
- **Allow Self-Collision** — Lets parts of the robot collide with each other. Turn this off if the model becomes unstable or parts interpenetrate at rest.

#### Options

- **Robot Type** — Category label for the imported robot (for example Manipulator, Humanoid, or Wheeled). Choose **Default** if unsure.
- **Base Type** — Whether the robot is fixed in the world or free to move:
  - **Source** — Keep the URDF as authored.
  - **Fixed** — Bolt the base to the world.
  - **Mobile** — Allow the base to move and rotate.
- **Merge Mesh** — Combines meshes where possible for a lighter, faster asset.
- **Debug Mode** — Keeps extra output files and log detail for troubleshooting imports.
