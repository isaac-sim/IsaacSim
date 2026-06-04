# Overview

The isaacsim.core.deprecation_manager extension provides automated migration handling for deprecated configuration settings and OmniGraph nodes in Isaac Sim. This extension monitors for deprecated items during startup and stage operations, automatically updating them to their current equivalents while providing detailed logging and user notifications about the changes.

## Functionality

### Automatic Settings Migration

The extension manages deprecated configuration settings by automatically migrating them to their updated counterparts during startup. It checks against a predefined list of deprecated settings paths and updates them to their new locations, ensuring existing configurations continue to work seamlessly. Deprecation warnings are logged to inform users of the changes.

### OmniGraph Node Updates

When a USD stage is opened, the extension traverses all prims to identify deprecated OmniGraph node types and updates them to their current equivalents. This includes nodes from various Isaac Sim domains such as sensors, manipulators, conveyor systems, and ROS2 bridge components. After making changes, the extension attempts to reload all graphs to ensure proper functionality.

### Module Import Safety

The {func}`import_module <isaacsim.core.deprecation_manager.import_module>` function provides a robust mechanism for importing Python packages with enhanced error handling. If a required package is not found, it logs an error message and exits the application gracefully. For specific packages like `torch`, it provides additional notices with further instructions to help users resolve import issues.

## Key Components

### Settings Configuration

The extension uses a comprehensive configuration list that maps deprecated settings to their new equivalents across multiple Isaac Sim domains:

- Benchmark services settings migration from `**omni.isaac.benchmark.services**` to `isaacsim.benchmark.services`
- Code editor settings for VS Code and Jupyter notebook environments
- ROS2 bridge configuration updates

### OmniGraph Node Mapping

The extension maintains a mapping of deprecated OmniGraph node type names to their current equivalents, covering nodes across sensor, robotics, domain randomization, and physics simulation domains.

## Integration

The extension integrates with `**omni.graph.core**` for OmniGraph node operations and `**omni.kit.notification_manager**` for user notifications about migration activities. It uses `**omni.usd**` for stage event monitoring and USD prim traversal during the update process.
