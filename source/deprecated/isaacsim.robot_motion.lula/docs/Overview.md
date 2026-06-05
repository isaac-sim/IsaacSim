# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.robot_motion.experimental.motion_generation` and `isaacsim.robot_motion.cumotion`.
```

The isaacsim.robot_motion.lula extension provides a comprehensive Python interface to the Lula library for robotic motion planning and control. This extension enables forward and inverse kinematics, sampling-based global planning, and smooth reactive motion generation through RMPflow and geometric fabrics for robotic manipulators.

## Functionality

**Kinematics and Motion Planning** - The extension supports both forward and inverse kinematics calculations along with collision-free path planning. It provides interfaces for computing joint configurations, end-effector poses, and generating optimal trajectories between waypoints.

**Path Specification and Conversion** - Users can define paths in both configuration space (joint space) and task space (Cartesian space), with automated conversion capabilities between the two representations. The system supports linear paths, circular arcs, and complex composite path specifications.

**RMPflow Motion Generation** - The extension implements RMPflow (Riemannian Motion Policies) for generating smooth, reactive robot motions. This includes real-time obstacle avoidance, end-effector tracking, and joint limit compliance through a unified motion policy framework.

**Collision Detection and World Modeling** - Built-in collision detection capabilities allow robots to navigate safely around obstacles. The system supports geometric primitives like spheres, cubes, and cylinders, with efficient distance calculations and collision queries.

**Trajectory Generation** - Advanced trajectory generation features include time-optimal trajectory planning with configurable velocity, acceleration, and jerk limits. Both linear and cubic spline interpolation modes are supported.

## Key Components

### Motion Planning Interface

The MotionPlanner class provides sampling-based path planning algorithms including RRT variants for finding collision-free paths in complex environments. It supports planning to both configuration space targets and task space pose targets.

### RMPflow Configuration

RmpFlowConfig manages the parameters and settings for reactive motion generation, allowing fine-tuning of behavior for different robot applications. This includes obstacle avoidance weights, attractor strengths, and convergence criteria.

### Path Specifications

The extension offers multiple path specification types:
- CSpacePathSpec for joint space waypoint sequences
- TaskSpacePathSpec for Cartesian space paths with linear segments and circular arcs  
- CompositePathSpec for combining both types into complex motion sequences

### Robot Modeling

RobotDescription encapsulates robot kinematics, collision geometry, and joint limits. It supports loading robot definitions from URDF and YAML configuration files.

## Usage Examples

```python
import isaacsim.robot_motion.lula as lula

# Set logging level for debug information
lula.set_log_level(lula.LogLevel.INFO)

# Load robot description from files
robot_desc = lula.load_robot("robot_config.yaml", "robot.urdf")

# Create a motion planner with collision avoidance
world = lula.create_world()
world_view = world.add_world_view()
motion_planner = lula.create_motion_planner(robot_desc, world_view)

# Generate trajectories with kinematic constraints
trajectory_gen = lula.create_c_space_trajectory_generator(robot_desc.kinematics())
trajectory_gen.set_velocity_limits(max_velocities)
trajectory_gen.set_acceleration_limits(max_accelerations)
```
