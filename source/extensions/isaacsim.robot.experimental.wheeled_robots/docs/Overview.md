# Overview

Extension providing wheeled robot and controller APIs: `WheeledRobot`, `HolonomicRobotUsdSetup`, `DifferentialController`, `HolonomicController`, `AckermannController`, `StanleyControl`, and `QuinticPathPlanner`.

## Basic Usage

The `isaacsim.robot.experimental.wheeled_robots` extension provides wheeled robot and controller APIs for the stage. Controllers return NumPy arrays (or tuples for Ackermann); the wheeled robot applies those directly as joint velocities. Use the stage utilities to set up the scene and add a `PhysicsScene` so physics runs.

### Overview

* **Robots**: Use `WheeledRobot` to wrap an articulation prim by path, with optional stage reference creation and placement. Apply commands with `apply_wheel_actions(velocities)`, where `velocities` is a NumPy array of wheel joint velocities.
* **Controllers**: `DifferentialController`, `HolonomicController`, and `AckermannController` convert high-level commands (e.g. linear/angular speed, or forward/lateral/yaw) into wheel velocities or angles.
* **Stage**: Use the stage utilities (e.g. `stage_utils.set_stage_up_axis`, `set_stage_units`, `add_reference_to_stage`) and add a `PhysicsScene` so physics can run.

### Differential drive (e.g. Jetbot)

Create the robot with `WheeledRobot`, then a `DifferentialController`. The controller takes a command `[linear_speed, angular_speed]` and returns a length-2 array of wheel velocities. Pass that array to `apply_wheel_actions`.

```python
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot

stage_utils.set_stage_up_axis("Z")
stage_utils.set_stage_units(meters_per_unit=1.0)

my_jetbot = WheeledRobot(
    paths="/World/Jetbot",
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
    usd_path=jetbot_asset_path,
    positions=[0.0, 0.0, 0.05],
)
my_controller = DifferentialController(
    wheel_radius=0.03,
    wheel_base=0.1125,
)

# In your simulation loop (e.g. each physics step):
velocities = my_controller.forward([0.1, 0.0])  # [linear_speed, angular_speed]
my_jetbot.apply_wheel_actions(velocities)
```
### Holonomic / mecanum (e.g. Kaya)

For holonomic robots, use `HolonomicRobotUsdSetup` to read wheel geometry from USD, then build `HolonomicController` from the returned parameters. The controller takes a command `[forward, lateral, yaw]` and returns an array of wheel joint velocities (one per wheel).

```python
from isaacsim.robot.experimental.wheeled_robots.controllers import HolonomicController
from isaacsim.robot.experimental.wheeled_robots.robots import (
    HolonomicRobotUsdSetup,
    WheeledRobot,
)

my_kaya = WheeledRobot(
    paths="/World/Kaya",
    wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
    usd_path=kaya_asset_path,
    positions=[0.0, 0.0, 0.02],
    orientations=[1.0, 0.0, 0.0, 0.0],
)

kaya_setup = HolonomicRobotUsdSetup(
    robot_prim_path="/World/Kaya",
    com_prim_path="/World/Kaya/base_link/control_offset",
)
(
    wheel_radius,
    wheel_positions,
    wheel_orientations,
    mecanum_angles,
    wheel_axis,
    up_axis,
) = kaya_setup.get_holonomic_controller_params()

my_controller = HolonomicController(
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

# In your simulation loop:
velocities = my_controller.forward([0.4, 0.0, 0.0])  # [forward, lateral, yaw]
my_kaya.apply_wheel_actions(velocities)
```
### Path planning and tracking

`StanleyControl` implements the Stanley lateral path-tracking algorithm for path following with configurable gains. `QuinticPathPlanner` generates smooth quintic polynomial trajectories between waypoints for wheeled robot navigation.

### Ackermann steering

`AckermannController` uses a bicycle model and returns steering angles and per-wheel rotation velocities. The command is a length-5 array: `[steering_angle, steering_angle_velocity, speed, acceleration, dt]`. The return value is a tuple `(joint_positions, joint_velocities)`. Use these to drive the robot's steering and wheel joints according to your robot's USD structure.

### Physics scene

Add a `PhysicsScene` so that physics runs. Set the simulation timestep with `SimulationManager.set_physics_dt(dt=1.0 / 60.0)` and run the timeline.

### Standalone examples

* **Jetbot (differential)**: `source/standalone_examples/api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py` (uses this extension)
* **Kaya (holonomic)**: `source/standalone_examples/api/isaacsim.robot.wheeled_robots.examples/kaya_holonomic_move.py` (uses this extension)
