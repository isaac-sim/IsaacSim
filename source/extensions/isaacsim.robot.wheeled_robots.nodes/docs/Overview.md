```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Overview

The `isaacsim.robot.wheeled_robots.nodes` extension provides OmniGraph nodes for wheeled robot controllers in Isaac Sim. These nodes can be placed in Action Graphs to control differential-drive, holonomic, and Ackermann-steered robots, as well as perform path planning and goal checking.

## Functionality

The extension registers OmniGraph nodes that wrap the controller classes from `isaacsim.robot.wheeled_robots`. Available nodes include DifferentialController (C++), HolonomicController, AckermannController, QuinticPathPlanner, StanleyControlPID, CheckGoal2D, and HolonomicRobotUsdSetup.

## Integration

The extension depends on `isaacsim.robot.wheeled_robots` for the underlying controller implementations, `isaacsim.core.nodes` for OGN base classes, and `omni.graph` for the OmniGraph framework.
