# Overview

The isaacsim.robot_motion.controllers extension provides portable, GPU-accelerated robot motion
controllers that implement the
{class}`BaseController <isaacsim.robot_motion.experimental.motion_generation.BaseController>`
interface from `isaacsim.robot_motion.experimental.motion_generation`.

Each controller reads its setpoint from a named **control-point site** in
`setpoint_state.sites` and writes joint velocity and position targets into the returned
`RobotState`.  On a CUDA device the projection and kinematics are captured into a CUDA graph
at construction time, keeping per-step overhead to a pair of device-to-device copies plus a
graph launch.

## Controllers

**{class}`DifferentialDriveController <isaacsim.robot_motion.controllers.DifferentialDriveController>`**
converts a control-point velocity setpoint into left and right wheel angular velocity targets
using the differential drive kinematic model:

```
omega_L = (2 * v - omega * wheel_base) / (2 * wheel_radius)
omega_R = (2 * v + omega * wheel_base) / (2 * wheel_radius)
```

Linear speed `v` and yaw rate `omega` are extracted from the site by projecting the 3D
linear and angular velocity vectors onto configurable `forward_direction` and
`rotation_direction` unit vectors.

**{class}`AckermannController <isaacsim.robot_motion.controllers.AckermannController>`**
converts a control-point linear-velocity setpoint into steerable-wheel angular velocities,
steering-angle position targets, and (optionally) non-steerable-wheel angular velocities,
using the Ackermann kinematic model.  The setpoint encodes speed `v` and body turning angle
`θ` as the velocity vector `[v·cos θ, v·sin θ, 0]`.  Alternatively, `direct_command=True`
allows passing `linear_speed` and `turning_angle` as keyword arguments directly to
`forward()`, bypassing the velocity-vector projection step.

**{class}`HolonomicController <isaacsim.robot_motion.controllers.HolonomicController>`**
converts a control-point body twist setpoint `[vx, vy, wz]` into per-wheel angular velocity
targets for mecanum-wheeled robots using closed-form
inverse kinematics:

```text
phi_dot_i = K_i * (M[i, :] @ [vx, vy, wz])
```

The kinematic matrix `M` (N × 3) and conversion diagonal `K` (N,) are precomputed at
construction from the authored wheel positions, orientations, radii, and mecanum roller
angles, after transforming that geometry into the command-site frame given by
`command_site_position` and `command_site_quaternion` — so the wheels may be measured in
whatever frame is convenient and the site moved independently of them.  Wheel geometry is
conveniently obtained from USD assets via
`HolonomicRobotUsdSetup.get_holonomic_controller_params()`.  Planar linear speed and yaw
rate are clamped independently before the matrix multiply; per-wheel speed is clamped after.

## Example

```python
import math

import isaacsim.robot_motion.controllers as ctrl
import isaacsim.robot_motion.experimental.motion_generation as mg
import warp as wp

# --- Differential drive ---

dd_controller = ctrl.DifferentialDriveController(
    robot_joint_space=robot.dof_names,
    left_wheel_joint="left_wheel_joint",
    right_wheel_joint="right_wheel_joint",
    wheel_radius=0.03,
    wheel_base=0.1125,
)

dd_setpoint = mg.RobotState(
    sites=mg.SpatialState.from_name(
        spatial_space=["control_point"],
        linear_velocities=(["control_point"], wp.array([[0.2, 0.0, 0.0]], dtype=wp.float32)),
        angular_velocities=(["control_point"], wp.array([[0.0, 0.0, 1.0]], dtype=wp.float32)),
    )
)

desired_state = dd_controller.forward(estimated_state, dd_setpoint, t)

# --- Ackermann ---

ack_controller = ctrl.AckermannController(
    robot_joint_space=robot.dof_names,
    left_steerable_wheel_joint="front_left_wheel",
    right_steerable_wheel_joint="front_right_wheel",
    left_steering_joint="front_left_steering",
    right_steering_joint="front_right_steering",
    steerable_wheel_radius=0.3,
    wheel_base=1.5,
    track_width=1.2,
)

v, theta = 1.0, 0.3  # speed [m/s], turning angle [rad]
ack_setpoint = mg.RobotState(
    sites=mg.SpatialState.from_name(
        spatial_space=["control_point"],
        linear_velocities=(
            ["control_point"],
            wp.array([[v * math.cos(theta), v * math.sin(theta), 0.0]], dtype=wp.float32),
        ),
    )
)

desired_state = ack_controller.forward(estimated_state, ack_setpoint, t)

# --- Holonomic (omni / mecanum) ---

holo_controller = ctrl.HolonomicController(
    robot_joint_space=robot.dof_names,
    wheel_joint_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
    wheel_radius=wheel_radius,        # from HolonomicRobotUsdSetup
    wheel_positions=wheel_positions,  # from HolonomicRobotUsdSetup
    wheel_orientations=wheel_orientations,
    mecanum_angles=[90.0, 90.0, 90.0],  # 90° = omni; 45°/135° = standard mecanum
)

holo_setpoint = mg.RobotState(
    sites=mg.SpatialState.from_name(
        spatial_space=["control_point"],
        linear_velocities=(["control_point"], wp.array([[0.4, 0.0, 0.0]], dtype=wp.float32)),
        angular_velocities=(["control_point"], wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)),
    )
)

desired_state = holo_controller.forward(estimated_state, holo_setpoint, t)
```

## Integration

This extension depends on `isaacsim.robot_motion.experimental.motion_generation` for the
`BaseController` interface and state types.
