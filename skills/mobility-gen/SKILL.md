---
name: mobility-gen
description: >
  Generate synthetic sensor datasets for mobile robots using Isaac Sim's MobilityGen
  extension. Two-phase pipeline: headless trajectory recording → replay-and-render
  for RGB/depth/seg/normals/pose data. Covers `RandomPathFollowingScenario`,
  `RandomAccelerationScenario`, and custom robot subclassing
  (`WheeledMobilityGenRobot`, `PolicyMobilityGenRobot`, holonomic). Use when generating
  mobile-robot training data, recording sim trajectories, replaying with sensors, or
  implementing a custom MobilityGenRobot. Distinct from `data-collection-sim`
  (static-scene SDG) — this skill is robot-trajectory-driven. See `navigation-primitives`
  for shared occupancy-map / A* / kinematics substrate; `isaac-sim-robot-navigation`
  for non-SDG runtime nav.
---

# MobilityGen Synthetic Data Generation

Two-phase pipeline: **record trajectories** (physics, no rendering) → **replay & render** (sensors added).

## Read These Skills First

- **navigation-primitives** — `OccupancyMap`, A* planner, robot footprints (Spot Z=0.69), differential/holonomic kinematics, look-at chase cameras, shared gotchas. MobilityGen consumes this substrate; this skill assumes you know it.
- **occupancy-map** — produces the `map.yaml` consumed by `OccupancyMap.from_ros_yaml`
- **data-collection-sim** — sibling SDG path for static scenes with randomized object/camera poses (no robot trajectory)

## When To Use This Skill (vs siblings)

| Goal | Use |
|---|---|
| Record trajectories then re-render with sensors for SDG (training data) | **this skill** |
| Drive a robot through a scene in real time, see it move | `isaac-sim-robot-navigation` |
| Annotated frames with no robot motion (object pose randomization) | `data-collection-sim` |

## Related Skills

- `navigation-primitives` — shared navigation substrate (read first)
- `data-collection-sim` — static-scene SDG sibling
- `isaac-sim-sensor` — sensor primitives (camera, LiDAR, IMU, contact)
- `isaac-sim-robot-navigation` — runtime navigation sibling
- `isaac-sim-headless-deployment` — `--no-window` headless launch and `SimulationApp` batch pattern

## Environment

- **Isaac Sim source tree**: `$ISAAC_SIM_DIR/source/` for source builds. The variable `$ISAAC_SIM_SRC` is a convenience alias for that path; declare it once at the top of your launcher (e.g. `ISAAC_SIM_SRC="$ISAAC_SIM_DIR/source"`).
- **Python launcher**: `$ISAAC_SIM_DIR/python.sh`
- **Replay script**: `$ISAAC_SIM_SRC/standalone_examples/replicator/mobility_gen/replay_directory.py`
- **Extension examples**: `$ISAAC_SIM_SRC/extensions/isaacsim.replicator.mobility_gen.examples/`
- **Data dir**: `$MOBILITY_GEN_DATA` (env var). Default to a workspace-local path such as `$WORKSPACE_DIR/MobilityGenData` or `$HOME/MobilityGenData`.
  - `recordings/` — timestamped trajectory dirs
  - `replays/` — rendered output
  - `maps/` — occupancy map YAML + PNG files

## Extension Loading (Critical)

Extensions are **not auto-loaded**. Always pass `--enable` flags when running `python.sh`:

```bash
"$ISAAC/python.sh" my_script.py --enable isaacsim.replicator.mobility_gen.examples

"$ISAAC/python.sh" my_script.py \
  --enable isaacsim.asset.gen.omap \
  --enable isaacsim.replicator.mobility_gen.examples
```

Enabling `isaacsim.replicator.mobility_gen.examples` auto-loads `isaacsim.replicator.mobility_gen` as a dependency. All extension-dependent imports must come AFTER `SimulationApp(...)` is initialized.

## Phase 1: Automated Trajectory Recording (Headless)

`KeyboardTeleoperationScenario` and `GamepadTeleoperationScenario` require an interactive UI. For headless batch recording use `RandomPathFollowingScenario` or `RandomAccelerationScenario`.

> **API note (Kit 110):** the snippet below uses `isaacsim.core.api.objects`, `isaacsim.core.utils.stage`, and the MobilityGen `get_world` / `new_world` helpers. These wrap the legacy `isaacsim.core.api.World` flow that MobilityGen's scenario/writer plumbing expects. For new code outside MobilityGen, prefer the experimental APIs (`isaacsim.core.experimental.utils.stage`, the `isaacsim.core.simulation_manager.SimulationManager` lifecycle, and `isaacsim.core.experimental.objects` for primitives) — see the cheatsheet. Do not mix the two within a single MobilityGen session.
>
> **Migration:** for the full `omni.isaac.*` → `isaacsim.*` mapping when porting scripts off the legacy World flow, see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html).

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": True})

import datetime, os, tempfile
import isaacsim.core.api.objects as objects
import isaacsim.replicator.mobility_gen.examples
from isaacsim.core.utils.stage import open_stage, save_stage
from isaacsim.replicator.mobility_gen.impl.config import Config
from isaacsim.replicator.mobility_gen.impl.occupancy_map import OccupancyMap
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS
from isaacsim.replicator.mobility_gen.impl.scenario import SCENARIOS
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world, new_world
from isaacsim.replicator.mobility_gen.impl.writer import MobilityGenWriter

DATA_DIR = os.environ.get(
    "MOBILITY_GEN_DATA",
    os.path.join(os.environ.get("WORKSPACE_DIR", os.path.expanduser("~")), "MobilityGenData"),
)

SCENE_USD    = "/path/to/warehouse.usd"
OMAP_YAML    = f"{DATA_DIR}/maps/warehouse/map.yaml"
ROBOT_TYPE   = "CarterRobot"               # JetbotRobot | CarterRobot | H1Robot | SpotRobot
SCENARIO     = "RandomPathFollowingScenario"  # or RandomAccelerationScenario
NUM_EPISODES = 5
MAX_STEPS    = 2000

robot_cls    = ROBOTS.get(ROBOT_TYPE)
scenario_cls = SCENARIOS.get(SCENARIO)

config = Config(scenario_type=SCENARIO, robot_type=ROBOT_TYPE, scene_usd=SCENE_USD)
occupancy_map = OccupancyMap.from_ros_yaml(OMAP_YAML)

open_stage(SCENE_USD)
cached_stage = os.path.join(tempfile.mkdtemp(), "stage.usd")
save_stage(cached_stage, save_and_reload_in_place=False)

world = new_world(physics_dt=robot_cls.physics_dt)
world.initialize_simulation_context()
objects.GroundPlane("/World/ground_plane", visible=False)

robot    = robot_cls.build("/World/robot")
scenario = scenario_cls.from_robot_occupancy_map(robot, occupancy_map)

os.makedirs(os.path.join(DATA_DIR, "recordings"), exist_ok=True)

for episode in range(NUM_EPISODES):
    world.reset()
    scenario.reset()

    name   = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
    path   = os.path.join(DATA_DIR, "recordings", name)
    writer = MobilityGenWriter(path)
    writer.write_config(config)
    writer.write_occupancy_map(occupancy_map)
    writer.copy_stage(cached_stage)

    step = 0
    while True:
        world.step(render=False)
        is_alive = scenario.step(step_size=robot_cls.physics_dt)
        writer.write_state_dict_common(scenario.state_dict_common(), step)
        step += 1
        if not is_alive or step >= MAX_STEPS:
            break

    print(f"Episode {episode+1}/{NUM_EPISODES}: {step} steps -> {path}")

simulation_app.close()
```

## Phase 2: Replay & Render

Replay all recordings in `$MOBILITY_GEN_DATA/recordings/` and write sensor data to `replays/`.

```bash
: "${MOBILITY_GEN_DATA:=${WORKSPACE_DIR:-$HOME}/MobilityGenData}"
ISAAC="$ISAAC_SIM_DIR"
SRC="$ISAAC_SIM_DIR/source"

CUDA_VISIBLE_DEVICES=0 DISPLAY=:99 nohup \
  "$ISAAC/python.sh" \
  "$SRC/standalone_examples/replicator/mobility_gen/replay_directory.py" \
  --input  "$MOBILITY_GEN_DATA/recordings" \
  --output "$MOBILITY_GEN_DATA/replays" \
  --render_interval 40 \
  --rgb_enabled True \
  --depth_enabled True \
  --segmentation_enabled True \
  --normals_enabled False \
  --render_rt_subframes 1 \
  --enable isaacsim.replicator.mobility_gen.examples \
  > /tmp/mobility_gen_replay.log 2>&1 &
```

`--render_interval 40` = 1 frame per 40 physics steps (~5 Hz at 200 Hz physics). Increase `--render_rt_subframes` for better quality at the cost of speed.

### Replay Output Structure

```
replays/<recording_name>/
  config.json
  stage.usd
  occupancy_map/map.yaml, map.png
  state/
    common/<step>.npy          # robot pose, joint positions, velocities
    rgb/<camera_name>/<step>.jpg
    segmentation/<camera_name>/<step>.png
    depth/<camera_name>/<step>.png   # 16-bit inverse depth
    normals/<camera_name>/<step>.npy
```

## Available Robots

| Name | Type | Notes |
|---|---|---|
| `JetbotRobot` | Wheeled (differential) | Small, `physics_dt=0.005`, Jetbot USD |
| `CarterRobot` | Wheeled (differential) | Nova Carter, `physics_dt=0.005` |
| `H1Robot` | Humanoid (policy) | Unitree H1, flat-terrain RL policy |
| `SpotRobot` | Quadruped (policy) | Boston Dynamics Spot, flat-terrain RL policy |

## Available Scenarios

| Name | Mode | Headless? |
|---|---|---|
| `KeyboardTeleoperationScenario` | Manual (WASD) | No — needs UI |
| `GamepadTeleoperationScenario` | Manual (gamepad) | No — needs UI |
| `RandomAccelerationScenario` | Automated (brownian) | Yes |
| `RandomPathFollowingScenario` | Automated (A* path following) | Yes |

`RandomPathFollowingScenario` plans an A* path from the robot's current position to a random free-space goal and follows it with proportional steering. Episode ends when goal is reached or robot collides.

## Add a Custom Robot

Two base classes exist depending on robot type. Both handle `build()` and `write_action()` — set class-level attributes only.

### Wheeled (differential drive)

Subclass `WheeledMobilityGenRobot`. No need to override `build()` or `write_action()`:

```python
from isaacsim.replicator.mobility_gen.examples.robots import WheeledMobilityGenRobot
from isaacsim.replicator.mobility_gen.examples.misc import HawkCamera
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS

@ROBOTS.register()
class MyRobot(WheeledMobilityGenRobot):
    physics_dt: float = 0.005
    z_offset: float = 0.25

    chase_camera_base_path = "chassis"
    chase_camera_x_offset: float = -1.5
    chase_camera_z_offset: float = 0.8
    chase_camera_tilt_angle: float = 60.0

    front_camera_base_path = "chassis/front_hawk"
    front_camera_rotation = (0.0, 0.0, 0.0)
    front_camera_translation = (0.2, 0.0, 0.1)
    front_camera_type = HawkCamera

    occupancy_map_radius: float = 0.5
    occupancy_map_z_min: float = 0.1
    occupancy_map_z_max: float = 0.5
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.5

    keyboard_linear_velocity_gain: float = 1.0
    keyboard_angular_velocity_gain: float = 1.0
    gamepad_linear_velocity_gain: float = 1.0
    gamepad_angular_velocity_gain: float = 1.0

    random_action_linear_velocity_range = (-0.3, 1.0)
    random_action_angular_velocity_range = (-0.75, 0.75)
    random_action_linear_acceleration_std: float = 5.0
    random_action_angular_acceleration_std: float = 5.0
    random_action_grid_pose_sampler_grid_size: float = 5.0
    path_following_speed: float = 1.0
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.5
    path_following_forward_angle_threshold = 0.785
    path_following_target_point_offset_meters: float = 1.0

    wheel_dof_names = ["left_wheel_joint", "right_wheel_joint"]
    usd_url: str = "/path/to/my_robot.usd"
    chassis_subpath: str = "chassis"
    wheel_base: float = 0.5
    wheel_radius: float = 0.1
```

Reference implementations in `isaacsim.replicator.mobility_gen.examples.robots`:
- `JetbotRobot`: NVIDIA Jetbot, `wheel_base=0.1125`, `wheel_radius=0.03`, `chassis_subpath="chassis"`
- `CarterRobot`: Nova Carter, `wheel_base=0.413`, `wheel_radius=0.14`, `chassis_subpath="chassis_link"`

### Holonomic (e.g. Kaya 3-wheel)

Override `build()` to use a different controller and `write_action()` to remap the 2D action:

```python
from isaacsim.replicator.mobility_gen.examples.robots import WheeledMobilityGenRobot
from isaacsim.replicator.mobility_gen.examples.misc import HawkCamera
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world, join_sdf_paths
from isaacsim.core.prims import Articulation as _ArticulationView
from isaacsim.robot.wheeled_robots.robots import WheeledRobot as _WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from isaacsim.storage.native import get_assets_root_path

@ROBOTS.register()
class KayaRobot(WheeledMobilityGenRobot):
    physics_dt: float = 0.005
    z_offset: float = 0.02
    chase_camera_base_path = "base_link"
    chase_camera_x_offset: float = -0.5
    chase_camera_z_offset: float = 0.3
    chase_camera_tilt_angle: float = 60.0
    front_camera_base_path = "base_link/front_hawk"
    front_camera_rotation = (0.0, 0.0, 0.0)
    front_camera_translation = (0.1, 0.0, 0.05)
    front_camera_type = HawkCamera

    occupancy_map_radius: float = 0.2
    occupancy_map_z_min: float = 0.02
    occupancy_map_z_max: float = 0.3
    occupancy_map_cell_size: float = 0.05
    occupancy_map_collision_radius: float = 0.2
    random_action_linear_velocity_range = (-0.2, 0.4)
    random_action_angular_velocity_range = (-0.5, 0.5)
    random_action_linear_acceleration_std: float = 1.0
    random_action_angular_acceleration_std: float = 2.0
    random_action_grid_pose_sampler_grid_size: float = 5.0
    path_following_speed: float = 0.4
    path_following_angular_gain: float = 1.0
    path_following_stop_distance_threshold: float = 0.3
    path_following_forward_angle_threshold = 0.785
    path_following_target_point_offset_meters: float = 0.5
    keyboard_linear_velocity_gain: float = 0.4
    keyboard_angular_velocity_gain: float = 0.5
    gamepad_linear_velocity_gain: float = 0.4
    gamepad_angular_velocity_gain: float = 0.5

    wheel_dof_names = ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
    usd_url: str = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd"
    chassis_subpath: str = "base_link"
    wheel_radius: float = 0.04
    wheel_base: float = 0.1
    com_prim_subpath: str = "base_link/control_offset"

    @classmethod
    def build(cls, prim_path: str):
        world = get_world()
        robot = world.scene.add(
            _WheeledRobot(prim_path, wheel_dof_names=cls.wheel_dof_names, create_robot=True, usd_path=cls.usd_url)
        )
        view = _ArticulationView(join_sdf_paths(prim_path, cls.chassis_subpath))
        world.scene.add(view)
        kaya_setup = HolonomicRobotUsdSetup(
            robot_prim_path=prim_path,
            com_prim_path=join_sdf_paths(prim_path, cls.com_prim_subpath),
        )
        (wheel_radius, wheel_positions, wheel_orientations, mecanum_angles, wheel_axis, up_axis) = \
            kaya_setup.get_holonomic_controller_params()
        controller = HolonomicController(
            name="kaya_controller", wheel_radius=wheel_radius, wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations, mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis, up_axis=up_axis,
        )
        camera = cls.build_front_camera(prim_path)
        return cls(prim_path=prim_path, robot=robot, articulation_view=view, controller=controller, front_camera=camera)

    def write_action(self, step_size: float):
        action = self.action.get_value()
        # MobilityGen 2D action [linear_vel, angular_vel] -> holonomic [forward, lateral=0, yaw]
        self.robot.apply_wheel_actions(self.controller.forward(command=[action[0], 0.0, action[1]]))
```

### Policy-based (legged robots)

Subclass `PolicyMobilityGenRobot` and implement `build_policy()`. `write_action()` converts the 2D action `[lin_vel, ang_vel]` into the 3D command `[x, 0, yaw]` automatically.

```python
from isaacsim.replicator.mobility_gen.examples.robots import PolicyMobilityGenRobot
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
import numpy as np

@ROBOTS.register()
class MyLegedRobot(PolicyMobilityGenRobot):
    physics_dt: float = 0.005
    z_offset: float = 1.05
    usd_url = "/path/to/robot.usd"
    articulation_path = "pelvis"
    controller_z_offset: float = 1.05
    # ... same occupancy_map_*, random_action_*, path_following_* attrs as Wheeled ...

    @classmethod
    def build_policy(cls, prim_path: str):
        return H1FlatTerrainPolicy(prim_path=prim_path, position=np.array([0.0, 0.0, cls.controller_z_offset]))
```

Reference implementations: `H1Robot` (`articulation_path="pelvis"`) and `SpotRobot` (`articulation_path="/"`) in the same module.

### Replay with a custom robot

`replay_directory.py` calls `load_scenario()` which does `ROBOTS.get(config.robot_type)`. If the robot isn't in the built-in extension, this raises `KeyError`. **You cannot pass `--enable` to load an ad-hoc Python file** — either create a proper Isaac extension, or copy the replay loop into your own script and register the robot class before calling `load_scenario()`:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": True})

import glob, os
import isaacsim.replicator.mobility_gen.examples  # built-in robots
from isaacsim.replicator.mobility_gen.impl.build import load_scenario
from isaacsim.replicator.mobility_gen.impl.robot import ROBOTS
from isaacsim.replicator.mobility_gen.impl.utils.global_utils import get_world

@ROBOTS.register()
class KayaRobot(WheeledMobilityGenRobot):
    ...  # full class definition

for recording_path in sorted(glob.glob(os.path.join(INPUT_DIR, "*"))):
    scenario = load_scenario(recording_path)   # now finds KayaRobot
    world = get_world()
    world.reset()
    scenario.enable_rgb_rendering()
    # ... rest of render loop (same as replay_directory.py)
```

See `code/mobility-gen/05_replay_kaya.py` for the complete working example.

## Config / Data Format

`config.json` per recording:
```json
{
  "scenario_type": "RandomPathFollowingScenario",
  "robot_type": "CarterRobot",
  "scene_usd": "/path/to/warehouse.usd"
}
```

`state/common/<step>.npy` is a numpy dict: `position`, `orientation`, `joint_positions`, `joint_velocities`, `linear_velocity`, `angular_velocity`.

## Common Pitfalls

- **`ModuleNotFoundError: No module named 'isaacsim.replicator.mobility_gen'`**: Extensions aren't auto-loaded. Pass `--enable isaacsim.replicator.mobility_gen.examples` to `python.sh`. All extension imports must come AFTER `SimulationApp(...)`.
- **`AttributeError: 'World' object has no attribute 'initialize_simulation_context'`**: Only the async version exists (`initialize_simulation_context_async`). For standalone scripts, call `world.reset()` instead.
- **Replay `KeyError: 'MyRobot'`**: `replay_directory.py` only knows built-in robots. Write a wrapper script that registers your robot class before calling `load_scenario()`.
- **Custom robot produces no images during replay**: Missing `front_camera_*` attributes, or `build()` passes `front_camera=None`. Add the attributes and call `cls.build_front_camera(prim_path)` in `build()`.
- **`AttributeError: 'MyRobot' has no attribute 'chase_camera_base_path'`**: `chase_camera_base_path`, `chase_camera_x_offset`, `chase_camera_z_offset`, `chase_camera_tilt_angle` are required by `load_scenario()` even for headless recording.
- **Built-in replay fails to find robot class**: Pass `--enable isaacsim.replicator.mobility_gen.examples` so the examples extension registers its robots/scenarios before `load_scenario()` runs.
- **`physics_dt` mismatch**: Recording stores the physics timestep in `config.json`; replay uses the same `robot_type.physics_dt`. Do not change robot params between record and replay.
- **Occupancy map scale**: MobilityGen consumes the `OccupancyMap` produced by `occupancy-map`. Ensure `map.yaml` origin and resolution match the USD world coordinates.
- **Headless GPU**: Set `CUDA_VISIBLE_DEVICES=0 DISPLAY=:99` to avoid GPU contention with vLLM on GPUs 1-3.
