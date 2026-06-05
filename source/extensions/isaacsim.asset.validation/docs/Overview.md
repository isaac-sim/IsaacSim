# Overview

The isaacsim.asset.validation extension provides custom validation rules for checking robot and simulation assets in Isaac Sim. It integrates with the Omniverse Asset Validator framework to verify that USD assets follow Isaac Sim conventions for physics, materials, joints, drives, and file structure.

## Validation Rules

- **Robot rules**: Validates robot asset naming conventions (Manufacturer/Robot/robot.usd), folder structure, and required schemas such as IsaacRobotAPI
- **Joint rules**: Checks joint configurations for proper types, limits, and parent-child relationships
- **Drive rules**: Validates actuator and drive configurations on articulated joints
- **Material rules**: Verifies material property assignments and detects missing or overridden material bindings
- **Physics rules**: Checks physics API application, collision geometry, and rigid body configurations
