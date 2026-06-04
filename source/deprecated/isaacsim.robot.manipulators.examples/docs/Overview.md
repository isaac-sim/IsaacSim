# Overview

```{deprecated} 6.0.0
This extension is deprecated. Use `isaacsim.robot.experimental.manipulators.examples` instead.
```

The `isaacsim.robot.manipulators.examples` extension contains legacy (non-experimental) implementations for Franka and Universal Robots manipulators. All experimental examples (FrankaExperimental, UR10Experimental, interactive UI extensions, pick-and-place, stacking, and follow-target tasks using experimental APIs) have been migrated to `isaacsim.robot.experimental.manipulators.examples`.

## Remaining Legacy Components

- **Franka** - Legacy Franka robot class, kinematics solver, controllers (pick-place, RMPFlow, stacking), and task definitions.
- **Universal Robots UR10** - Legacy UR10 robot class, kinematics solvers, controllers (pick-place, RMPFlow, stacking), and task definitions (bin filling, follow target, pick-place, stacking).
