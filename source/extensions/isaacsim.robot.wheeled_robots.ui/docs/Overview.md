# Overview

The Wheeled Robots UI extension provides a user interface for creating and configuring differential drive controllers for wheeled robots in Isaac Sim. It adds a specialized window accessible through the menu system that allows users to generate OmniGraph controllers for differential drive systems.

## UI Components

### Differential Controller Window

The `DifferentialControllerWindow` serves as the primary interface for setting up differential drive robot controllers. This window provides a form-based interface where users can configure parameters for wheeled robot control systems.

**Graph Generation**: The window's main functionality centers around the `make_graph()` method, which creates complete OmniGraph networks for differential drive control. The generated graphs include:

- Differential controller nodes for processing drive commands
- Articulation controller nodes for robot joint management
- Optional keyboard control nodes for manual robot operation
- Proper node connections and parameter configurations based on user input

The window handles both creating new graphs and extending existing ones, allowing users to iteratively build complex control systems.

## Functionality

### Menu Integration

The extension integrates with Isaac Sim's menu system by adding a "Differential Controller" entry under "Tools/Robotics/OmniGraph Controllers". This provides users with easy access to the differential drive configuration tools from the main application interface.

### Controller Configuration

The extension focuses specifically on differential drive systems, which are common in wheeled mobile robots. Users can configure wheel parameters, joint mappings, and control logic through the provided interface. The resulting OmniGraph networks can then be used to control actual wheeled robots in simulation.

The differential controller's velocity command uses the wheel order `[left, right]`. When selecting joints by name or index, configure the articulation controller in the same order. For example, use `["wheel_left_joint", "wheel_right_joint"]`, or the corresponding left-wheel index followed by the right-wheel index. Reversing the joint order swaps the generated wheel commands and can cause the robot to rotate when a straight-line command is expected.

The Differential Controller window preserves this ordering when it constructs the joint name or joint index array: the **Left Joint** value is written first and the **Right Joint** value second.

## Integration

The extension works closely with the core `isaacsim.robot.wheeled_robots` extension to provide UI access to wheeled robot functionality. It uses `**omni.graph**` and `**omni.graph.tools**` to create and manage the OmniGraph networks that control robot behavior, while `**omni.kit.menu.utils**` provides the menu integration framework for accessing the differential controller window.
