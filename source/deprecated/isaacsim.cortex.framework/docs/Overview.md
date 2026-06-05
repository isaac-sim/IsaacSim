# Overview

```{deprecated} 6.0.0
This extension has been deprecated and will be replaced by open source equivalents and simple examples.
```

The Cortex tutorials give a comprehensive overview of the framework and the concepts behind it.
That's the best introduction. Here we just briefly describe some highlights and specifics of the
extension.

The cortex tools provide a decision framework for orchestrating the tools provided in Isaac Sim to
design behavior and execute it on physical robots. It consists of:
- A main cortex loop runner. This is the mind of the robot, with a belief model of the world and
  robot itself, and tools to analyzing the logical state of the world and choosing action. It uses
  motion generation tools built into Isaac Sim to control the belief robot.
- An extension `cortex_ros` that connects this belief to and from the physical world using ROS.
  Perception transforms stream in to the belief model, and actuation streams out to the physical
  robot.
- An extension `cortex_sim` that represents a simulated version of the real world for software and
  hardware in the loop development. It implements the ROS interfacing protocols expected from the
  physical robot's control system and the real-world perception module.
- Some example environments and behaviors, including a blocks world and scripts implementing a
  reactive block stacking behavior demonstrating the cortex decision framework.

## Quickstart -- block stacking demo

The following is a brief overview of running the system using the Franka block stacking demo as an
example.

These commands are relative to `standalone_examples/cortex`.

Note: When starting multiple terminals as outlined below, it's convenient to use the `Terminator` app.

###  Starting the system with belief robot only

Running a belief robot only. Launch the main cortex loop runner without ROS (default).
```text
Terminal 1: Launch cortex loop runner passing in the blocks world USD env.
cd standalone_examples/cortex
./cortex launch --usd_env=Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief.usd

# The `cortex` script is an alias to the `cortex_main.py` loop runner. Alternatively, from the base
# dirctory of Isaac Sim you can execute the loop runner directly using:
./python.sh exts/isaacsim.cortex.framework/omni/isaac/cortex/cortex_main.py \
    --usd_env=Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief.usd

Terminal 2: Activate behavior.
cd standalone_examples/cortex
./cortex activate build_block_tower.py
# It starts runner the block stacking behavior. At any point we can switch behaviors.
./cortex activate go_home.py # Sends the robot to home and allows manual control using target prim.
./cortex activate reset_world.py # Reset blocks to home.
```
In this example, you can interact with the blocks as its trying to build the
tower and the robot will react.

###  Starting the system with belief and sim robots

Running both the belief and sim robots. This setup is similar, except it uses a
different USD environment file, and you need to run the simulated controller
from `cortex_control` to connect the sim robot to the belief robot making decisions.
```text
Terminal 1: Start a roscore

Terminal 2: Launch cortex loop runner passing in the blocks world USD env and using --enable_ros.
cd standalone_examples/cortex
./cortex launch \
    --usd_env=Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief_sim.usd \
    --enable_ros

Terminal 3: Activate behavior.
cd standalone_examples/cortex
./cortex activate build_block_tower.py

# At this point the belief robot will start trying to grab the first block, but
# the sim robot isn't following because the controller isn't running. We need
# to start the controller.
Terminal 4: Start the simulated controller
rosrun cortex_control sim_controller
```

##  Connecting to a physical robot

The physical robot will take the place of the sim robot, and we'll run a
real-world controller rather than the simulated controller. Here, we'll show
how to send the robot home and use manual control since otherwise you'd need a
real-world perception module.

Start the system with belief only. This is the same procedure outlined above.
```text
Terminal 1: Start a roscore

Terminal 2: Launch cortex loop runner passing in the blocks world USD env.
cd standalone_examples/cortex
./cortex launch \
    --usd_env=Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief.usd \
    --enable_ros
```
At this point, we can run behaviors as before, but the system will only run the
simulated belief robot. The physical robot isn't yet connected.

Now start up the Franka robot, and start the `cortex_control_franka` controllers. At
the point were we launch the joint position controller in terminal 3 below, you
should see the simulated cortex belief robot synchronize with the physical
robot. It will engage the robot at that point, so you may see some slight
movement, but it shouldn't be much. Make sure you have the e-stop ready in case
anything goes wrong.
```text
Terminal 1: Start the Franka controller manager
source ~/catkin_ws/devel/setup.bash
roslaunch cortex_control_franka franka_control_lula.launch

Terminal 2: Set high torque thresholds for Franka
rosrun cortex_control_franka set_high_collision_thresholds

Terminal 3: Startup the position controller -- launching this controller syncs the belief with the physical robot.
roslaunch cortex_control_franka joint_position_controller.launch

Terminal 4: Start the gripper commander listener
rosrun cortex_control_franka franka_gripper_command_relay.py
```

At this point, we can run some behaviors and we'll see the physical robot
following the simulated robot. Try the following:
```text
cd standalone_examples/cortex
./cortex activate open_gripper.py  # Opens the physical gripper
./cortex activate close_gripper.py  # Closes the physical gripper
./cortex activate go_home.py  # Sends the robot to its home position
```
Once the robot gets to its home position, you'll be able to control the robot manually by moving the 
`motion_controller_target` prim in the stage located at
```text
/cortex/belief/motion_controller_target
```
Select the prim, then select the "Move" tool from the toolbar along the left edge of the viewport.
Then drag the arrows. 


## World setup conventions

Cortex USD worlds follow a particular path naming convention. Good examples are:
```text
Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief.usd
Isaac/Samples/Cortex/Franka/BlocksWorld/cortex_franka_blocks_belief_sim.usd
Isaac/Samples/Cortex/UR10/Basic/cortex_ur10_basic_belief.usd
Isaac/Samples/Cortex/UR10/Basic/cortex_ur10_basic_belief_sim.usd
```

It is assumed these environments are setup in units of centimeters.

The belief env is added to the path `/cortex/belief` and the sim env (if it exists) it's used is
added to the path `/cortex/sim`. Each of these envs contain `robot` and `objects` subprims. The
robot should have a string metadata attribute `cortex:robot_type` telling the system the robot type.
Currently supported values are `cortex:robot_type = {'franka', 'ur10'}`. Additionally, objects added
to the scene can have an optional `cortex:is_obstacle` attribute which, when set to True, loads the
object in as an obstacle. This is used in the block stacking example so the tower becomes an
obstacle automatically as it's created. Objects do not need to have a `cortex:is_obstacle`
attribute. If one is not present, the object is assumed to not be an obstacle.

All xform prims representing robots and objects should follow the Isaac Sim core API transform
specification USD conventions. Specifically, they should have `Transform` attributes specified by
`xformOp:translate`, `xformOp:orient`, `xformOp:scale` with `xformOpOrder` given as
`[xformOp:translate, xformOp:orient, xformOp:scale]`.

For instance, the block stacking env containing both belief and sim is laid out as:
```text
/cortex
  /belief
    /robot  # Franka USD with cortex:robot_type of 'franka'
    /objects
      /red_block    # cortex:is_obstacle = True
      /yellow_block # cortex:is_obstacle = True
      /green_block  # cortex:is_obstacle = True
      /blue_block   # cortex:is_obstacle = True
  /sim
    /robot  # Franka USD with cortex:robot_type of 'franka'
    /objects
      /red_block
      /yellow_block
      /green_block
      /blue_block

```
It's often useful to setup a common environment that's simply shared by both `/cortex/belief` and
`/cortex/sim`. That makes it easy to setup both belief-only and belief-sim variants of the USD env.

Other attributes and prims
- The belief robot has `cortex:adaptive_cycle_dt` (Double) and `cortex:is_suppressed` (Bool). These
  are used internally by cortex and will be automatically added on startup. It's ok if the USD
  environment already has them.
- `/cortex/belief/motion_controller_target` prim. This is a simple cube prim used for manually controlling
  the robot. If it's already in the environment, cortex will use that one. Otherwise, it will create
  its own when initializing the motion commander.


## Breakdown of files

Main cortex loop runner: Standalone python app that runs the main cortex loop runner and starts the
`cortex_{ros,sim}` extensions.
- `cortex_main.py` : Primary entry point and main cortex loop runner. This runs the main standalone
cortex python app. It points to its own experience file which includes the isaacsim.cortex.framework
extension. A cortex compatible USD env is passed in via a flag. It automatically starts up the
`cortex_ros` and `cortex_sim` extensions. The former is always running, so a physical robot can be
connected at any time. If the USD env has a sim environment, that robot will be used in place of a
physical robot.

Extensions: Extensions loaded on startup handing ROS communication to physical robot and simulated
robot.
- `cortex_ros.py` : Handles ROS connections to get perceptual information into cortex and to send
  control information out of cortex.
- `cortex_sim.py` : Handles creating the ROS communication interface to mimic a physical robot using
  a simulated environment.

Decision framework: The core decision framework
- `df.py` : Core framework tools, including an implementation of decider networks and state
  machines.
- `dfb.py` : Decision framework behaviors useful across multiple behavior scripts. These include
  specific decider node types and actions.
- `df_behavior_watcher.py` : Monitors the `df_behavior_module.py` file watching for changes. Reloads
  when a change is detected.
- `df_behavior_module.py` : This file is constantly monitored by the main cortex loop runner. When
  it changes, the behavior is loaded and run. On startup, nothing is run until a behavior is
  explicitly activated.

`standalone_examples/cortex` directory: Contains example user defined behavior scripts.
- `activate` : simple script for copying a behavior to the monitored `df_behavior_module.py` in the
  main cortex directory to activate it. `df_behavior_module.py` is monitored by the behavior watcher
  in `df_behavior_watcher.py`.
- `clear` : clear the current behavior. The robot won't be running any behavior can be manually
  controlled after this by moving the `motion_controller_target` prim.
- `go_home.py` : Send the robot to the home position. Once the robot arrives at the home position,
  it can be controller using the `motion_controller_target` prim.
- `manual_control.py` : Set the robot to manual control. The robot can be controlled by moving the
  `motion_controller_target` prim. This is a specific behavior that does nothing (as opposed to no
  behavior as in `clear`).
- `reset_world.py` : resets the objects in both the belief and sim environments back to their
  initial configurations.
- franka-specific behaviors inside `behaviors/franka`
    - `build_block_tower.py` : Build a block tower in the blocks world. This behavior is reactive to
      unexpected changes the blocks / tower.
    - `block_tower_monitors.py` : Start up the block tower monitors only.
    - `send_blocks_to_tower.py` : Send the blocks to the correct goal tower.
    - `send_blocks_to_bad_tower.py` : Send the block to a bad tower where the robot will have to
      tear down then reconstruct the correct tower.
    - `open_gripper.py` : Open the gripper.
    - `close_gripper.py` : Close the gripper.

cortex tools:
- `cortex_utils.py` : Utilities for setting up cortex.
- `cortex_object.py`: An object representation wrapping core API objects that simplifies accessing
  and using the cortex attributes. For instance, cortex objects have measured poses written into the
  USD. The cortex object has APIs for reading that measured pose and syncing the (belief) object's
  pose to that measured pose.
- `motion_commander.py` : A wrapper around Isaac Sim's intelligent motion policies providing a
  command API interface with a pose target and accompanying approach direction. Provides convenience
  methods for accessing forward kinematics to the control frame and opening and closing the gripper.
  Also, automatically smooths commands sent to the commander using `smoothed_command.py` and uses
  the `RmpFlowSmoothed` to make the resulting motions safe to run on the real robot.
- `smoothed_command.py` : A tool for smoothing commands automatically.
- `synchronized_time.py` : A ROS utility used to implement the clock synchronization protocol with
  the controller to adapt to slighly different clock speeds between the embedded robot controller
  and the machine running cortex. (E.g. the Franka controller's clock sometimes runs slightly fast
  causing the controller's interpolator to overrun the buffer over time. This synchronization
  protocol enables constant monitoring of the time delta to enable continual long-term runs on the
  physical robot.)

utils:
- `cli.py` : Simple tools for setting up convenient command line interfaces. Used especially in some
  of the tests.
- `gf_conversions.py` : Tools for more easily reading information to and from USD through the Gf
  interface.
- `math_util.py` : Math tools and utilities.
- `ros_tf_util.py` : ROS-based utilities.
- `tools.py` : Common utilities for running steady loops and profiling.

tests:
- `tests/test_df.py` : Unit tests for the decision framework (df.py).
- `tests/test_motion_commander.py` : A standalone python app that starts up the motion commander in a
  basic Franka environment to test and demo the motion commander interface.


## Details

### building

`cortex_control` and `cortex_control_franka` are both located in `ros_workspace/src`. Follow the
instructions for building that ROS workspace to build. You can also copy or symlink them into a
separate ROS workspace. `cortex_contro_franka` depends on `cortex_control` and `franka_ros`, but
`cortex_control` is standalone, and `franka_ros` should be installed already with the ROS
distribution. If not, you can find it linked from Franka's website.

When trying these tools in simulation, you need only the `cortex_control` library (which has the
`sim_controller` binary).  However, for controlling the physical robot, you need to install both
`cortex_control` and `cortex_control_franka` in a catkin workspace on the Franka's realtime control
machine.

Build the catkin workspace.
```bash
cd ~/catkin_ws
catkin_make
```

## Troubleshooting

When restarting the controller for the physical robot, it's best to bring down the entire controller
manager (i.e. everything on the real-time machine) and restart everything.
