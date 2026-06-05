# Overview

The isaacsim.replicator.mobility_gen.examples extension provides a collection of pre-configured robot implementations and control scenarios for mobility generation and autonomous navigation within Isaac Sim. This extension offers ready-to-use robot classes for various platforms including wheeled robots (Jetbot, Carter), humanoid robots (H1), and quadruped robots (Spot), along with comprehensive teleoperation and autonomous navigation scenarios.

<div align="center">

```{mermaid}
graph TD
    %% Inheritance relationships
    WheeledMobilityGenRobot --> JetbotRobot
    WheeledMobilityGenRobot --> CarterRobot
    PolicyMobilityGenRobot --> H1Robot
    PolicyMobilityGenRobot --> SpotRobot
```

</div>

## Functionality

### Robot Implementations

The extension provides two primary robot categories with distinct control approaches:

**Wheeled Robots**: {class}`WheeledMobilityGenRobot <isaacsim.replicator.mobility_gen.examples.WheeledMobilityGenRobot>` serves as the base class for differential drive robots using DifferentialController for wheel-based movement. {class}`JetbotRobot <isaacsim.replicator.mobility_gen.examples.JetbotRobot>` and {class}`CarterRobot <isaacsim.replicator.mobility_gen.examples.CarterRobot>` extend this foundation with platform-specific configurations including wheel parameters, camera positioning, occupancy mapping settings, and control gains for different operational modes.

**Policy-Controlled Robots**: {class}`PolicyMobilityGenRobot <isaacsim.replicator.mobility_gen.examples.PolicyMobilityGenRobot>` provides the foundation for robots using reinforcement learning policies for locomotion. {class}`H1Robot <isaacsim.replicator.mobility_gen.examples.H1Robot>` implements humanoid bipedal locomotion using H1FlatTerrainPolicy, while {class}`SpotRobot <isaacsim.replicator.mobility_gen.examples.SpotRobot>` provides quadruped navigation capabilities through SpotFlatTerrainPolicy. These robots feature sophisticated terrain navigation and articulation management.

All robot implementations include front-facing {class}`HawkCamera <isaacsim.replicator.mobility_gen.examples.HawkCamera>` integration for visual perception, configurable occupancy mapping for navigation, and support for chase cameras with adjustable positioning and tilt angles.

### Control Scenarios

The extension includes four distinct control scenarios for robot operation:

**Manual Control**: {class}`KeyboardTeleoperationScenario <isaacsim.replicator.mobility_gen.examples.KeyboardTeleoperationScenario>` enables WASD keyboard control for linear and angular velocities, while {class}`GamepadTeleoperationScenario <isaacsim.replicator.mobility_gen.examples.GamepadTeleoperationScenario>` provides gamepad-based control using analog stick inputs for more precise movement commands.

**Autonomous Navigation**: {class}`RandomAccelerationScenario <isaacsim.replicator.mobility_gen.examples.RandomAccelerationScenario>` generates unpredictable movement patterns through random acceleration changes applied to current velocities, useful for diverse training data generation. {class}`RandomPathFollowingScenario <isaacsim.replicator.mobility_gen.examples.RandomPathFollowingScenario>` implements complete autonomous navigation with random target generation, path planning, obstacle avoidance, and proportional control for path following.

## Key Components

### Robot Configuration Parameters

Each robot implementation includes comprehensive parameter sets covering physics simulation timesteps, occupancy mapping specifications (radius, cell size, collision detection), camera configurations (positioning, rotation, translation), and control gains for different input modalities. Velocity ranges and acceleration parameters are pre-configured for realistic robot behavior across keyboard, gamepad, random action, and path following scenarios.

### Navigation and Safety Systems

The robots feature integrated collision detection through buffered occupancy maps that account for physical robot dimensions. Path following implementations use look-ahead controllers with configurable target point offsets and angular gain parameters. Safety mechanisms include automatic scenario termination upon collision detection or boundary violations.

### Visualization Support

{class}`RandomPathFollowingScenario <isaacsim.replicator.mobility_gen.examples.RandomPathFollowingScenario>` provides visualization capabilities through occupancy map rendering with path overlay functionality, displaying planned routes in green for debugging and monitoring autonomous navigation behavior.

## Dependencies

The extension integrates with isaacsim.replicator.mobility_gen for core mobility generation functionality, isaacsim.robot.policy.examples for reinforcement learning policies (H1FlatTerrainPolicy, SpotFlatTerrainPolicy), and isaacsim.robot.wheeled_robots for differential drive control systems.
