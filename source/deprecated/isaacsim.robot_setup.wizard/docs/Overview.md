# Overview

```{deprecated} 6.0.0
This extension is deprecated. No replacement.
```

The isaacsim.robot_setup.wizard extension provided a wizard interface for importing and configuring robots in Isaac Sim. This extension guided users through a step-by-step process to set up robots, from initial import through final configuration, with visual progress tracking and integrated tools.

```{image} ../../../../source/deprecated/isaacsim.robot_setup.wizard/data/preview.png
---
align: center
---
```


## Functionality

The extension centers around the {class}`RobotWizardWindow <isaacsim.robot_setup.wizard.RobotWizardWindow>` class, which implements a six-step wizard workflow for robot setup:

1. **Add Robot** - Initial robot import and selection
2. **Robot Hierarchy** - Structure configuration and organization  
3. **Add Colliders** - Collision detection setup
4. **Joints and Drives** - Joint configuration and drive parameters
5. **Prepare Files** - File preparation and validation
6. **Save Robot** - Final robot saving and export

Each step includes validation, progress tracking, and contextual guidance to ensure proper robot configuration.

## Key Components

### {class}`RobotWizardWindow <isaacsim.robot_setup.wizard.RobotWizardWindow>`

The main wizard interface provides a structured workflow with visual progress indicators. The window automatically docks to the viewport and maintains state across wizard steps. Users can navigate between steps, with each step building upon the previous configuration.

The wizard integrates additional robot setup tools including:
- Joint Drive Gain Tuner for fine-tuning joint parameters
- Robot Assembler for component organization
- USD to URDF Exporter for format conversion

### Progress Tracking

The wizard implements a comprehensive progress system using `ProgressState` objects to track completion status and step visibility. The visual progress tracker shows users their current position in the workflow and highlights completed steps.

### Integration Tools

Beyond the core wizard steps, the extension provides access to specialized robot configuration tools through an integrated interface, allowing users to perform advanced setup tasks without leaving the wizard environment.

## Usage Examples

To access the robot wizard:

```python
import isaacsim.robot_setup.wizard

# Get the current wizard window instance
wizard_window = isaacsim.robot_setup.wizard.get_window()

# Create a new wizard window
wizard = isaacsim.robot_setup.wizard.RobotWizardWindow("Robot Setup")
wizard.set_visible(True)
```

The extension also provides menu integration, making the wizard accessible through Isaac Sim's interface without requiring direct API calls.
