# Overview

The isaacsim.robot_motion.schema extension provides a USD schema for motion planning functionality in Isaac Sim. The extension defines the IsaacMotionPlanningAPI, which can be applied to USD prims to enable collision-aware motion planning capabilities for robotic applications.

## Key Components

### IsaacMotionPlanningAPI Schema

The extension defines a single-apply API schema that adds motion planning attributes to USD prims. The schema includes collision detection configuration for motion planning operations, allowing developers to specify whether collision checking should be enabled for specific prims during motion planning calculations.

### Schema Application API

The extension provides the {func}`apply_motion_planning_api <isaacsim.robot_motion.schema.apply_motion_planning_api>` function to programmatically apply the IsaacMotionPlanningAPI to USD prims. This function handles the proper application of the schema and optionally sets the collision-enabled attribute in a single operation.

```python
from isaacsim.robot_motion.schema import apply_motion_planning_api
import omni.usd

stage = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath("/World/Robot")
apply_motion_planning_api(robot_prim, enabled=True)
```

## Functionality

The schema enables motion planning systems to identify which prims should participate in collision detection during path planning operations. By applying the IsaacMotionPlanningAPI to robot components or environmental objects, motion planning algorithms can determine which elements require collision avoidance calculations.

The schema's collision-enabled attribute provides fine-grained control over collision detection behavior, allowing developers to selectively enable or disable collision checking for specific prims based on their motion planning requirements.
