# Overview

The isaacsim.robot.poser.ui extension provides a user interface for creating, managing, and exporting mimic training points for robots using an IK (inverse kinematics) goal workflow. It enables users to define poses for robotic articulations and track them through named pose systems.

```{image} ../../../../source/extensions/isaacsim.robot.poser.ui/data/preview.png
---
align: center
---
```


## Key Components

### {class}`UIBuilder <isaacsim.robot.poser.ui.UIBuilder>`

The {class}`UIBuilder <isaacsim.robot.poser.ui.UIBuilder>` class serves as the primary interface component that constructs and manages the Robot Poser window. It handles the main UI construction and coordinates interactions between different parts of the robot posing workflow.

The {class}`UIBuilder <isaacsim.robot.poser.ui.UIBuilder>` integrates timeline event handling to refresh articulation dropdowns when playback state changes, and provides asset loading callbacks to update available articulations when stage content changes.

```python
ui_builder = UIBuilder()
ui_builder.build_ui()  # Constructs the main Robot Poser window
```

### Named Pose Management

The extension includes a named poses table system that allows users to create and manage collections of robot poses. These poses can be tracked and applied to robotic articulations during runtime.

The tracking system enables real-time IK solving for poses that are marked as "dirty" or requiring updates:

```python
# Toggle tracking for a specific named pose
success = ui_builder.toggle_tracking_for_path("/World/RobotPose", enable=True)
```

### IK Integration

The extension integrates with inverse kinematics functionality to solve robot poses dynamically. It maintains tracking caches and applies IK chain outlines to visualize the kinematic relationships during pose manipulation.

The per-frame update system continuously processes tracked poses and applies IK solutions as needed.

## Functionality

**Pose Creation and Export**: Users can define robot poses and export them as mimic training points for robotic applications.

**Real-time Tracking**: The system tracks named poses and applies IK solutions in real-time during timeline playback or simulation.

**Articulation Management**: The interface automatically refreshes available articulations when stage content changes or simulation states transition.

**Visual Feedback**: IK chain outlines provide visual representation of kinematic relationships during pose manipulation.
