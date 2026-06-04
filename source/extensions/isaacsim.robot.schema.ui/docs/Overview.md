# Overview

The Isaac Sim Schema UI extension provides a specialized interface for visualizing and interacting with robot joint hierarchies in Isaac Sim. It creates a hierarchical tree view of robot joints as parent-child relationships and displays 3D connection lines between joints in the viewport, making it easier to understand complex robot structures.


## Key Components

### Robot Hierarchy Window

The Robot Hierarchy Window presents robot joints in a tree structure where the parent-child relationships are represented by the tree hierarchy rather than USD prim hierarchy. This provides a more intuitive view of how joints connect within a robot's kinematic chain.

**Key features:**
- Hierarchical tree view of robot joints organized by their kinematic relationships
- Bidirectional selection synchronization between the tree view and main stage
- Expand/collapse functionality for navigating complex robot structures
- Search and filtering capabilities inherited from the stage widget

### Viewport Connection Visualization

The extension renders visual connection lines between parent and child joints directly in the 3D viewport. These connections help users understand the robot's joint structure and kinematic chain visually.

**Visualization features:**
- Dynamic connection lines drawn between related joints in 3D space
- Directional arrows indicating parent-child relationships
- Overlay indicators for joints that share the same screen position
- Camera-aware visibility culling to optimize performance
- Clickable overlay menus for selecting from overlapping joints

### Path Translation System

A bidirectional path mapping system translates between the original USD stage paths and the generated hierarchy stage paths. This enables seamless integration between the hierarchical view and the standard USD stage operations.

## Functionality

### Hierarchy Generation

The extension automatically scans the current stage for prims with the Robot API applied and builds an in-memory USD stage representing the joint hierarchy. Each robot's link tree is analyzed to create parent-child relationships based on joint connections rather than USD prim hierarchy.

### Real-time Updates

The system monitors USD stage changes and timeline events to keep the visualization current. Connection positions are dynamically recalculated as the robot moves or transforms change, with optimization to avoid unnecessary redraws during minor camera movements.

### Selection Synchronization

Selection changes in the hierarchy tree view are automatically reflected in the main USD stage, and vice versa. The path mapping system ensures that selections translate correctly between the hierarchical representation and the original stage structure.

### Overlay Management

When multiple joints occupy the same screen position in the viewport, the extension provides overlay indicators with context menus for joint selection. This solves the problem of selecting specific joints in dense robot configurations.

## Integration

The extension integrates with Isaac Sim's robot schema system through the `isaacsim.robot.schema` dependency, using robot API data to understand joint relationships and generate the hierarchical representation. It leverages `**omni.kit.widget.stage**` to provide familiar stage widget functionality within the specialized hierarchy view and uses `**omni.ui.scene**` framework for 3D viewport manipulator rendering.
