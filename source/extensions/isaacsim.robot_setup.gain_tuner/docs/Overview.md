# Overview

The isaacsim.robot_setup.gain_tuner extension provides a UI-based tool for tuning PD (Proportional-Derivative) gains on robot articulations. This extension creates an interactive interface accessible through the Tools > Robotics menu, allowing users to adjust stiffness and damping parameters for robot joints and validate their settings through testing functionality.

```{image} ../../../../source/extensions/isaacsim.robot_setup.gain_tuner/data/preview.png
---
align: center
---
```


## Key Components

### {class}`UIBuilder <isaacsim.robot_setup.gain_tuner.UIBuilder>`

The {class}`UIBuilder <isaacsim.robot_setup.gain_tuner.UIBuilder>` class manages the complete user interface and serves as the primary component for robot joint gain tuning operations. It provides three main functional areas organized as collapsible frames:

**Tune Gains Frame**: Interactive table interface for adjusting robot joint parameters, supporting both stiffness/damping mode and natural frequency mode for gain configuration.

**Test Gains Settings Frame**: Configuration panel for setting up and executing gain validation tests, including test duration, target positions, and execution controls.

**Charts Frame**: Visualization component displaying test results through position and velocity plots to help users analyze gain performance.

### Robot Selection and Joint Management

The interface includes robot selection dropdown functionality that automatically populates available articulated robots in the scene. Once a robot is selected, the system identifies and displays all relevant joints with their current gain parameters.

### Real-time Synchronization

The {class}`UIBuilder <isaacsim.robot_setup.gain_tuner.UIBuilder>` integrates with multiple event systems to maintain synchronization between the simulation state and the interface:

- Timeline events handle play/pause/stop states to control when gain testing is active
- Physics step callbacks execute during simulation playback to collect test data
- Stage events manage initialization and cleanup when stages are opened or closed
- Render events coordinate the cancellation of physics subscriptions during test completion

### Testing and Validation

The extension provides comprehensive testing capabilities through the GainTuner backend integration. Users can configure test parameters, execute gain validation runs, and visualize results through integrated plotting functionality. The testing system operates during timeline playback and automatically manages data collection and analysis.
