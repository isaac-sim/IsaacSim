# Overview

```{image} ../../../../source/extensions/isaacsim.asset.gen.omap.ui/data/preview.png
```

To enable this extension, go to the Extension Manager menu and enable isaacsim.asset.gen.omap extension.

## Generating an Occupancy Map

1. Open the panel via **Tools > Robotics > Occupancy Map**.
2. Set the origin, upper/lower bounds, and cell size to define the mapping region.
3. Click **Calculate** to compute the occupancy grid from collision geometry in the stage.
4. Click **Visualize** to open the visualization window.

## Visualization Window

The **Occupancy Map Visualization** window lets you configure the output before saving:

- **Occupied / Freespace / Unknown Color** — colors used for each cell type in the image.
- **Rotate Image** — clockwise rotation (0°, 90°, −90°, 180°) applied to the output image.
- **Coordinate Type** — format of the config data shown in the text field: ROS YAML or stage-space coordinates.
- **Re-Generate Image** — recompute the image after changing settings.
- **Filename** — base name used for both the PNG and YAML saves; also written into the YAML `image` field. Updates the YAML preview automatically as you type.

## Saving the Outputs

After generating the image, two save buttons appear below the map preview:

- **Save Image** — saves the occupancy map as a `.png` file.
- **Save YAML** — saves the ROS occupancy map parameters as a `.yaml` file, usable with tools like `map_server`. Both dialogs pre-fill the filename from the **Filename** field.

> **Note:** The YAML file is always saved in ROS format regardless of which Coordinate Type is selected.
