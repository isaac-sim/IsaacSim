# Overview

The isaacsim.replicator.grasping.ui extension provides a comprehensive user interface for interactive robotic grasp generation and evaluation within Isaac Sim. This extension creates a dedicated window accessible through the Tools/Replicator menu that enables users to configure grasping scenarios, generate grasp poses, simulate grasp execution, and evaluate grasp success metrics.

## Key Components

### {class}`GraspingWindow <isaacsim.replicator.grasping.ui.GraspingWindow>`

The {class}`GraspingWindow <isaacsim.replicator.grasping.ui.GraspingWindow>` class serves as the main interface for all grasping-related functionality. The window is organized into several collapsible sections that cover the complete grasping workflow:

**Gripper Configuration** - Users can define gripper joint pregrasp states and configure grasp phases to specify how the gripper should move during grasp execution.

**Object Selection and Pose Sampling** - The interface provides controls for selecting target objects and configuring surface sampling parameters for generating potential grasp poses.

**Grasp Pose Generation** - Supports generating antipodal grasp poses using surface sampling algorithms, allowing users to create multiple grasp candidates for evaluation.

**Visualization Tools** - Includes options for visualizing grasp poses and object meshes to help users understand the spatial relationships and grasp configurations.

**Simulation Settings** - Integrates with USD stages and physics scenes to enable realistic grasp simulation, including physics scene configuration and rendering options.

**Workflow Automation** - Provides batch grasp evaluation capabilities to test multiple grasp candidates systematically.

**Configuration Management** - Allows users to save and load complete grasping setup configurations for reusability across sessions.

## Integration

The extension uses **omni.kit.menu.utils** to integrate with the Omniverse Kit SDK menu system, creating menu items in the Tools/Replicator group. This provides consistent access to grasping tools alongside other Isaac Sim replicator functionality.
