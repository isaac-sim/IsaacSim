# Overview

The isaacsim.robot_setup.assembler extension enables the assembly of multiple articulated robots into single unified structures. It provides tools to physically connect separate robot components by creating fixed joints between them, managing their transforms, and handling collision masking to prevent interference during simulation.

```{image} ../../../../source/extensions/isaacsim.robot_setup.assembler/data/preview.png
---
align: center
---
```


## Key Components

### {class}`RobotAssembler <isaacsim.robot_setup.assembler.RobotAssembler>`

The {class}`RobotAssembler <isaacsim.robot_setup.assembler.RobotAssembler>` class serves as the primary interface for robot assembly operations. It manages the complete assembly workflow from initial positioning through final composition.

**Assembly Process**: The assembler follows a multi-stage process where robots are first positioned relative to each other using mount frames, then physically connected through fixed joints. The process supports variant management, allowing different assembly configurations to be stored and selected.

**Transform Alignment**: The assembler automatically calculates and applies the necessary transforms to align attachment points between robots. It uses mount frames on both the base and attachment robots to determine the proper positioning and orientation.

### {class}`AssembledBodies <isaacsim.robot_setup.assembler.AssembledBodies>`

The {class}`AssembledBodies <isaacsim.robot_setup.assembler.AssembledBodies>` class represents the result of a successful assembly operation between two rigid bodies. It maintains references to the base and attachment components along with their connecting joint.

**Joint Management**: The class tracks the fixed joint that physically connects the two bodies and manages the root joints of the attachment body. Root joints are disabled during assembly to prevent the attachment from being independently controlled, and can be re-enabled during disassembly.

**Collision Control**: Each assembled body includes collision masking relationships that prevent physics conflicts between the connected components during simulation.

### {class}`AssembledRobot <isaacsim.robot_setup.assembler.AssembledRobot>`

The {class}`AssembledRobot <isaacsim.robot_setup.assembler.AssembledRobot>` class provides a simplified interface for working with assembled robot data. It wraps an {class}`AssembledBodies <isaacsim.robot_setup.assembler.AssembledBodies>` instance and exposes key properties through a streamlined API focused on robot-specific use cases.

## Functionality

### Physical Assembly

The extension creates fixed joints between robot components at specified mount points, effectively making separate articulated structures behave as a single physical entity during simulation.

### Variant Management

Assembly configurations can be stored as USD variants, allowing users to switch between different robot configurations or attachment states within the same asset.

### Collision Masking

The system automatically creates collision filter relationships between assembled components to prevent physics solver conflicts while maintaining collision detection with external objects.

### Transform Calculation

Advanced transform utilities handle the complex calculations needed to properly align robot components based on their mount frames and current world positions.
