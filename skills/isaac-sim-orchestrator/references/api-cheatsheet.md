# Isaac Sim 6 / Kit 110 — API Cheatsheet

Modern public API surface for orchestrating sims. Pair with the skill files
referenced at the end for full context.

## Bootstrap

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
# ALL isaacsim.* / omni.* imports MUST come after this line.
```

## Stage / app utilities

```python
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.app   as app_utils
from isaacsim.core.simulation_manager   import SimulationManager

stage_utils.create_new_stage(template="sunlight")
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.add_reference_to_stage(usd_path=path, path="/World/MyAsset")

# Sim engine selection (default `physx`; `isaacsim.physics.newton` switches
# to `newton` on startup when enabled — see physics-simulation skill).
SimulationManager.switch_physics_engine("newton")        # or "physx"
SimulationManager.setup_simulation(dt=1.0/60.0, device="cuda:0")
```

## Primitives, materials, lights

```python
from isaacsim.core.experimental.objects import (
    Cube, Sphere, GroundPlane,
    DistantLight, DomeLight, RectLight, SphereLight, DiskLight, CylinderLight,
    Camera,
)
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial, OmniPbrMaterial

Cube(paths="/World/cube", positions=[0, 0, 1], sizes=0.3)
GroundPlane("/World/ground", positions=[0, 0, 0])
DomeLight("/World/dome").set_intensities(500)
DistantLight("/World/sun").set_intensities(300)

# Apply physics by adding APIs:
RigidPrim(paths="/World/cube")
GeomPrim(paths="/World/cube", apply_collision_apis=True)

mat = PreviewSurfaceMaterial("/Materials/red")
mat.set_input_values("diffuseColor", [1.0, 0.0, 0.0])
```

## Robots / articulations

```python
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.storage.native           import get_assets_root_path
import omni.timeline

assets = get_assets_root_path()
stage_utils.add_reference_to_stage(
    usd_path=assets + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    path="/World/Franka",
)

# Pose before creating articulation.
XformPrim("/World/Franka").set_world_poses(positions=[[0, 0, 0]])

arm = Articulation("/World/Franka")
omni.timeline.get_timeline_interface().play()
app_utils.update_app(steps=5)            # init tensors

arm.set_dof_position_targets(targets)
positions = arm.get_dof_positions()
velocities = arm.get_dof_velocities()
J = arm.get_jacobian_matrices()          # for differential IK
M = arm.get_mass_matrices()
```

Legacy `isaacsim.core.api.articulations.Articulation` /
`isaacsim.core.api.prims.RigidPrim` still load but are superseded.

## URDF / MJCF import

```python
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
URDFImporter(URDFImporterConfig(
    urdf_path="/path/robot.urdf",
    usd_path="/path/out",
    merge_fixed_joints=True,
    robot_type="Manipulator",
    run_asset_transformer=True,    # runs isaacsim.asset.transformer
)).import_urdf()

from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
MJCFImporter(MJCFImporterConfig(
    mjcf_path="/path/robot.xml",
    usd_path="/path/out",
    import_scene=True,
    robot_type="Quadruped",
)).import_mjcf()

# Reverse direction (USD -> URDF)
from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
UsdToUrdfConverter(stage, "/World/robot").export("/path/urdf_out")
```

See `urdf-mjcf-to-usd-conversion`.

## Robot Schema + Named Poses + IK

```python
from usd.schema.isaac.robot_schema import (
    Classes, Attributes, ApplyRobotAPI, ApplyLinkAPI, ApplyJointAPI,
    ApplySiteAPI, PopulateRobotSchemaFromArticulation, GetAllNamedPoses,
    get_allowed_tokens,
)
from isaacsim.robot.poser import (
    RobotPoser, Transform,
    store_named_pose, apply_pose_by_name,
    list_named_poses, export_poses, import_poses,
    apply_joint_state, apply_joint_state_anchored,
    validate_robot_schema,
)

# Robot Schema overlay (URDF/MJCF importer applies it automatically).
ApplyRobotAPI(robot_prim)
robot_prim.GetAttribute(Attributes.ROBOT_TYPE).Set("Manipulator")
PopulateRobotSchemaFromArticulation(stage, robot_prim)

# Schema-native IK + persisted poses.
validate_robot_schema(stage, robot_prim)
poser  = RobotPoser(stage, robot_prim, start_prim, end_prim)
result = poser.solve_ik(Transform(position=[0.5, 0.2, 0.8], orientation=[1, 0, 0, 0]))
if result.success:
    poser.apply_pose(result.joints)
    store_named_pose(stage, robot_prim, "pick_position", result)

# Later session:
apply_pose_by_name(stage, robot_prim, "pick_position")
```

See `manipulation-ik` and `usd-articulation`.

## Camera (RTX wrapper)

```python
from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor, TiledCameraSensor

cam = RtxCamera("/World/cam", tick_rate=30.0)
cam.camera.set_focal_lengths(24.0)
cam.camera.set_clipping_ranges(0.01, 1000.0)
sensor = CameraSensor(cam, resolution=(720, 1280),
                     annotators=["rgb", "distance_to_image_plane"])
app_utils.play(commit=True)
rgb = sensor.get_data("rgb")
```

Plain USD camera + Replicator render product still works; see
`isaac-camera`.

## RTX lidar / radar / acoustic + physics sensors

```python
from isaacsim.sensors.experimental.rtx     import Lidar, LidarSensor, parse_generic_model_output_data
from isaacsim.sensors.experimental.physics import Contact, ContactSensor, IMU, IMUSensor, EffortSensor, JointStateSensor

lidar  = Lidar.create(path="/World/lidar", config="OS1", variant="OS1_REV6_32ch20hz512res")
LidarSensor(lidar, annotators=["generic-model-output"])

imu = IMU.create(path="/World/Robot/imu", tick_rate=200.0)
IMUSensor(imu, annotators=["linear_acceleration", "angular_velocity", "orientation"])

# Runtime-only (no authoring class — wrap an existing joint by path):
EffortSensor("/World/Robot/joint_arm_1")
JointStateSensor("/World/Robot/joint_arm_1")
```

Legacy `isaacsim.sensors.physics.*` (`ContactSensor(prim_path=...)`,
`IMUSensor(...)`) still works; prefer `experimental.*` for new code.

## Simulation loop

```python
import omni.timeline

SimulationManager.set_physics_dt(1.0 / 60.0)
omni.timeline.get_timeline_interface().play()
omni.timeline.get_timeline_interface().commit()

for step in range(num_steps):
    app_utils.update_app(steps=1)
    # read sensors, apply actions, ...

simulation_app.close()
```

## Replicator / SDG (current functional API)

Examples: `$ISAAC_SIM_DIR/source/standalone_examples/api/isaacsim.replicator.examples/`
(`sdg_getting_started_0[1-5].py`, `sdg_workflow_0[12].py`, `multi_camera.py`,
`cosmos_writer_simple.py`, ...) and `replicator/{scene_based_sdg,object_based_sdg,augmentation,infinigen,mobility_gen}/`.

Workflows: `isaacsim.replicator.grasping`, `isaacsim.replicator.episode_recorder`,
`isaacsim.replicator.teleop`.

```python
import omni.replicator.core as rep
prim = stage_utils.get_current_stage().GetPrimAtPath("/World/MyObj")
rep.functional.modify.semantics(prim, {"class": "cracker_box"}, mode="add")
rp = rep.create.render_product("/World/cam", (720, 1280))
w  = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir="/tmp/sdg_out", rgb=True, bounding_box_2d_tight=True)
w.attach(rp)
```

## Isaac Lab

```bash
cd "$ISAAC_LAB_DIR"
./isaaclab.sh -p <script.py>     # run with Isaac Lab Python
./isaaclab.sh -t                  # run tests
./isaaclab.sh -f                  # pre-commit
```

## Built-in robot asset paths (via `get_assets_root_path()`)

- `/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`
- `/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd`
- `/Isaac/Robots/UniversalRobots/ur10/ur10.usd`
- `/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd`
- `/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd`

## Save images

```python
import matplotlib.pyplot as plt
plt.imsave("output.png", rgb_array)
# or
from PIL import Image
Image.fromarray(rgb_array).save("output.png")
```

## Per-skill deep-dives

- Asset import / export: `urdf-mjcf-to-usd-conversion`.
- Articulation + Robot Schema: `usd-articulation`.
- IK and named poses: `manipulation-ik`.
- Physics scene config: `physics-simulation`.
- Camera authoring + capture: `isaac-camera`, `isaac-sim-rendering`.
- Sensors: `isaac-sim-sensor`.
- SDG pipelines: `data-collection-sim`, `mobility-gen`.
- ROS 2: `isaac-sim-ros2-bridge`.
