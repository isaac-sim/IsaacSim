# Overview

The MJCF Importer UI extension adds a graphical import workflow for MuJoCo MJCF files in Isaac Sim. Use it from **File > Import** to convert an MJCF XML file into USD and place the model on the stage. Core parsing and conversion are handled by the companion **isaacsim.asset.importer.mjcf** extension.

![MJCF Importer UI](../data/preview.png)

## Dependencies

- **isaacsim.asset.importer.mjcf** — converts MJCF files to USD

## Usage

### Accessing the MJCF Importer

1. In **Window > Extensions**, enable **isaacsim.asset.importer.mjcf.ui** if it is not already loaded.
2. Choose **File > Import** and select an MJCF `.xml` file.
3. Adjust settings in the panel on the right side of the import window.
4. Click **Import** to add the model to the stage.

### Import settings

The options panel is organized into three sections you can expand or collapse:

#### Output

- **USD Output** — Where to save the generated USD. The default **Same as Imported Model (Default)** saves next to your MJCF file. Click **Select File** to pick another folder.

#### Colliders

- **Collision From Visuals** — Builds collision shapes from visual meshes instead of collision geometry in the MJCF. When checked, **Collision Type** appears below.
- **Collision Type** — How to approximate collision when generating from visuals:
  - **Convex Hull**
  - **Convex Decomposition**
  - **Bounding Sphere**
  - **Bounding Cube**
- **Allow Self-Collision** — Lets parts of the model collide with each other. Turn this off if the simulation becomes unstable or parts interpenetrate at rest.

#### Options

- **Robot Type** — Category label for the imported robot (for example Manipulator, Humanoid, or Quadruped). Choose **Default** if unsure.
- **Base Type** — Whether the model is fixed in the world or free to move:
  - **Source** — Keep the MJCF as authored.
  - **Fixed** — Bolt the base to the world.
  - **Mobile** — Allow the base to move and rotate.
- **Import Scene** — Includes MJCF simulation settings with the model. Enabled by default.
- **Merge Mesh** — Combines meshes where possible for a lighter, faster asset.
- **Debug Mode** — Keeps extra output files and log detail for troubleshooting imports.
