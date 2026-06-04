# Overview

The isaacsim.asset.gen.conveyor.ui extension provides the graphical interface for creating and configuring conveyor belts in Isaac Sim. It adds menu entries for one-click conveyor creation and a Conveyor Track Builder tool for assembling multi-segment conveyor systems from Digital Twin Assets.

## UI Components

### Menu Entries

- **Create > Isaac Sim > Warehouse Items > Conveyor** — Creates a single conveyor belt Action Graph on the selected rigid body prim using the `CreateConveyorBelt` command from `isaacsim.asset.gen.conveyor`.
- **Tools > Conveyor Track Builder** — Opens the Conveyor Builder window for designing conveyor track systems.

### Conveyor Track Builder

The Conveyor Builder window (`Tools > Conveyor Track Builder`) provides an interactive tool for constructing conveyor track systems from a library of pre-built segments. Key capabilities include:

- **Track segment library**: Select from straight, curved, ramp, and angled conveyor segments with configurable styles and dimensions
- **System assembly**: Build connected conveyor systems by chaining track segments, with automatic alignment and placement
- **Visual preview**: Preview geometry shows segment placement before committing, highlighted with a green tint
- **Conveyor selection widget**: Inspect and modify properties of existing conveyor prims in the stage

### Preferences

The extension registers a preferences page under **Edit > Preferences** where users can configure default conveyor track builder settings such as the asset source path for conveyor segment models.

## Dependencies

This extension depends on:
- `isaacsim.asset.gen.conveyor`: Core conveyor belt plugin, OmniGraph node, and commands
- `isaacsim.storage.native`: Asset access for Digital Twin conveyor models
