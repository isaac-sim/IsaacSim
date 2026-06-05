# Overview

```{image} ../../../../source/extensions/isaacsim.asset.gen.omap/data/preview.png
```

The Isaac Sim Occupancy Map extension provides tools for generating 2D and 3D occupancy maps from USD stages. These maps represent spatial information about environments for robotics applications such as navigation, path planning, and collision avoidance.

## Features

- **2D and 3D Map Generation**: Create both 2D projections and full 3D volumetric occupancy maps
- **PhysX-Based Collision Detection**: Leverages PhysX for accurate collision geometry detection
- **Configurable Resolution**: Adjustable cell size for different map resolutions
- **Flexible Bounds**: Define custom mapping regions with origin and boundary settings
- **Python and C++ API**: Access functionality from both Python and C++
- **Efficient Storage**: Uses octomap library for efficient spatial data representation

## Installation

This extension is part of Isaac Sim. To enable it:

1. Open Isaac Sim
2. Go to `Window > Extensions`
3. Search for `isaacsim.asset.gen.omap`
4. Enable the extension

## Usage

### Basic Usage (Python)

```python
import omni.physx
from isaacsim.asset.gen.omap.bindings import _omap

# Get PhysX interface and stage
physx = omni.physx.get_physx_interface()
stage_id = omni.usd.get_context().get_stage_id()

# Create generator
generator = _omap.Generator(physx, stage_id)

# Configure settings
generator.update_settings(
    0.05,  # cell_size: 5cm resolution
    1.0,   # occupied_value
    0.0,   # unoccupied_value
    0.5    # unknown_value
)

# Set mapping region
generator.set_transform(
    (0, 0, 0),      # origin
    (-5, -5, 0),    # min_bound
    (5, 5, 2)       # max_bound
)

# Generate 2D map
generator.generate2d()

# Get results
occupied_positions = generator.get_occupied_positions()
buffer = generator.get_buffer()
dimensions = generator.get_dimensions()
```

### Using the Interface API

```python
from isaacsim.asset.gen.omap.bindings import _omap
from isaacsim.asset.gen.omap.utils import update_location

# Acquire interface
om = _omap.acquire_omap_interface()

# Set parameters
om.set_cell_size(0.05)
update_location(om, (0, 0, 0), (-5, -5, 0), (5, 5, 2))

# Generate map
om.generate()

# Get results
positions = om.get_occupied_positions()
buffer = om.get_buffer()

# Release when done
_omap.release_omap_interface(om)
```

### Utility Functions

The extension provides utility functions for common operations:

```python
from isaacsim.asset.gen.omap.utils import (
    compute_coordinates,
    generate_image,
    update_location
)

# Compute image coordinates
top_left, top_right, bottom_left, bottom_right, coords = compute_coordinates(om, cell_size=0.05)

# Generate colored visualization
image = generate_image(
    om,
    occupied_col=[0, 0, 0, 255],      # Black
    unknown_col=[127, 127, 127, 255], # Gray
    freespace_col=[255, 255, 255, 255] # White
)
```

## API Reference

### Generator Class

Main class for generating occupancy maps.

**Constructor**: `Generator(physx_ptr, stage_id)`
- `physx_ptr`: PhysX interface pointer
- `stage_id`: USD stage identifier

**Methods**:
- `update_settings(cell_size, occupied_value, unoccupied_value, unknown_value)`: Configure map parameters
- `set_transform(origin, min_bound, max_bound)`: Define mapping region
- `generate2d()`: Generate 2D occupancy map
- `generate3d()`: Generate 3D occupancy map
- `get_occupied_positions()`: Get positions of occupied cells
- `get_free_positions()`: Get positions of free cells
- `get_min_bound()`: Get minimum boundary coordinates
- `get_max_bound()`: Get maximum boundary coordinates
- `get_dimensions()`: Get map dimensions in cells
- `get_buffer()`: Get raw occupancy data
- `get_colored_byte_buffer(occupied, unoccupied, unknown)`: Get RGBA visualization buffer

### OccupancyMap Interface

Plugin interface for occupancy map functionality.

**Methods**:
- `generate()`: Generate the occupancy map
- `update()`: Update visualization
- `set_transform(origin, min_point, max_point)`: Set mapping transform
- `set_cell_size(cell_size)`: Set cell resolution
- `get_occupied_positions()`: Get occupied cell positions
- `get_free_positions()`: Get free cell positions
- `get_min_bound()`: Get minimum bounds
- `get_max_bound()`: Get maximum bounds
- `get_dimensions()`: Get dimensions
- `get_buffer()`: Get occupancy buffer
- `get_colored_byte_buffer(occupied, unoccupied, unknown)`: Get colored buffer

## Configuration

### Cell Size

The cell size determines the resolution of the occupancy map:
- Smaller cells = Higher resolution, more memory/computation
- Larger cells = Lower resolution, less memory/computation
- Default: 0.05 meters (5cm)
- Recommended range: 0.01m to 0.5m depending on application

### Mapping Bounds

Define the region to map:
- **Origin**: Reference point for the map (typically robot position)
- **Min Bound**: Lower corner relative to origin
- **Max Bound**: Upper corner relative to origin

### Occupancy Values

Three types of cells:
- **Occupied** (default: 1.0): Contains obstacles
- **Unoccupied** (default: 0.0): Known free space
- **Unknown** (default: 0.5): Unexplored or unreachable areas

## Examples

### Simple Room Mapping

```python
# Map a 10m x 10m room at 5cm resolution
generator.update_settings(0.05, 1.0, 0.0, 0.5)
generator.set_transform((0, 0, 0), (-5, -5, 0), (5, 5, 2))
generator.generate2d()
```

### High-Resolution Local Map

```python
# Map a 2m x 2m area at 1cm resolution
generator.update_settings(0.01, 1.0, 0.0, 0.5)
generator.set_transform((0, 0, 0), (-1, -1, 0), (1, 1, 1))
generator.generate2d()
```

### 3D Volumetric Map

```python
# Generate full 3D map
generator.update_settings(0.1, 1.0, 0.0, 0.5)
generator.set_transform((0, 0, 0), (-5, -5, -2), (5, 5, 2))
generator.generate3d()
```

## Performance Considerations

- **2D vs 3D**: 2D mapping is significantly faster for planar navigation
- **Cell Size**: Doubling cell size reduces memory/computation by ~4x (2D) or ~8x (3D)
- **Bounds**: Limit mapping area to only what's needed
- **PhysX Geometry**: Use collision approximations for faster results

## Troubleshooting

### Empty Map Generated

**Problem**: `get_buffer()` returns empty array

**Solutions**:
- Ensure PhysX scene is present in stage
- Verify collision geometry exists on objects
- Check that bounds encompass the scene
- Confirm timeline is playing during generation

### Performance Issues

**Problem**: Map generation is slow

**Solutions**:
- Increase cell size
- Reduce mapping bounds
- Use 2D instead of 3D mapping
- Enable PhysX collision approximations

### Incorrect Occupancy

**Problem**: Objects not appearing as occupied

**Solutions**:
- Verify objects have collision geometry
- Check object visibility
- Ensure bounds are set correctly
- Confirm cell size is appropriate for object sizes

## Related Extensions

- `isaacsim.asset.gen.omap.ui`: UI components for interactive map generation
- `isaacsim.util.debug_draw`: Visualization utilities
- `omni.physx`: Physics simulation engine

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Octomap Library](https://octomap.github.io/)
- [PhysX Documentation](https://docs.omniverse.nvidia.com/extensions/latest/ext_physics.html)
