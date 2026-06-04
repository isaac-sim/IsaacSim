# Overview

```{image} ../../../../source/extensions/isaacsim.asset.importer.heightmap/data/preview.png
```

The isaacsim.asset.importer.heightmap extension converts 2D heightmap and occupancy map images into 3D terrain environments in USD format. It reads grayscale images where pixel intensity represents elevation or occupancy, and generates corresponding 3D geometry using USD point instancers for efficient rendering.

## Functionality

- **Image to terrain conversion**: Transforms heightmap images into 3D environments with configurable cell size and height scaling
- **Occupancy map support**: Interprets dark pixels (below a configurable threshold) as occupied cells for navigation and obstacle mapping
- **Efficient rendering**: Uses USD point instancers with cube instances for memory-efficient representation of large terrains
- **Scene setup**: Automatically generates ground planes and lighting for the imported environment
