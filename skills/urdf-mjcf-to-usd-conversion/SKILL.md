---
name: urdf-mjcf-to-usd-conversion
description: >
  Convert URDF and MJCF robot descriptions to USD for Isaac Sim 6 and Isaac
  Lab via the modern `isaacsim.asset.importer.urdf` /
  `isaacsim.asset.importer.mjcf` APIs (`URDFImporter` + `URDFImporterConfig`,
  `MJCFImporter` + `MJCFImporterConfig`). Covers the
  `isaacsim.asset.transformer` post-import pipeline, the
  `usd.schema.isaac.robot_schema` application (IsaacRobotAPI, robot-type
  tokens), URDF round-tripping via `isaacsim.asset.exporter.urdf`, RL vs
  teleop drive setup, `make_instanceable`, and the Isaac Lab
  `convert_urdf.py` / `convert_mjcf.py` `config.yaml` flow. XACRO is not
  parsed by the URDF importer core; either import directly from a running
  `robot_description` node via `isaacsim.ros2.urdf` (`RobotDefinitionReader`
  / `File -> Import from ROS2 URDF Node`), or pre-expand offline with
  `xacro robot.xacro > robot.urdf`.
  Use when bringing a new robot from URDF/MJCF into Isaac Sim or Isaac Lab,
  troubleshooting wrong masses / joint behavior / self-collision, or
  exporting USD back to URDF.
---

# URDF / MJCF -> USD Conversion

Two conversion paths and one export path.

| Path | When | Driver |
|---|---|---|
| 1. Full Isaac Sim import (CLI) | RL/Lab asset, scripted batch | `isaacsim.asset.importer.urdf` / `.mjcf` via `urdf_import.py` / `mjcf_import.py` |
| 2. Isaac Lab convert script | Isaac Lab-native config.yaml workflow | `$ISAAC_LAB_DIR/scripts/tools/convert_urdf.py` / `convert_mjcf.py` |
| Export | USD -> URDF round-trip | `isaacsim.asset.exporter.urdf` via `urdf_export.py` |

## XACRO inputs

The `URDFImporter` core does not parse XACRO. Two supported paths:

### Recommended — import directly from a running ROS 2 node

`isaacsim.ros2.urdf` adds a dedicated import path that queries the `robot_description` parameter on any node (typically `robot_state_publisher`) via the standard `GetParameters` service, resolves `package://` URLs, writes the URDF to a temp file, and feeds it to `URDFImporter`. The node is responsible for XACRO expansion, so this also covers launch-file-only distributions that never ship a static URDF.

UI: `File -> Import from ROS2 URDF Node` (opens an import window with the same collider / robot-type / mesh options as the standard URDF importer).

Python (preferred over the deprecated `URDFImportFromROS2Node` Kit command):

```python
from functools import partial

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.ros2.urdf import RobotDefinitionReader

config = URDFImporterConfig(
    usd_path="/path/out",
    merge_fixed_joints=True,
    fix_base=False,
    robot_type="Manipulator",
)
importer = URDFImporter()

def _on_description(urdf_abs_path: str, package_found: bool) -> None:
    # `urdf_abs_path` is a temp file with package:// already resolved.
    config.urdf_path = urdf_abs_path
    importer.config = config
    importer.import_urdf()

reader = RobotDefinitionReader()        # singleton
reader.description_received_fn = partial(_on_description)
reader.start_get_robot_description("robot_state_publisher")
```

Requires the `isaacsim.ros2.urdf` extension (depends on `isaacsim.ros2.bridge` for the ROS 2 runtime), the ROS 2 environment sourced before launching Isaac Sim, and a reachable node publishing `robot_description`. The reader runs asynchronously — the callback fires once the `GetParameters` service replies.

### Fallback — offline xacro CLI

Use when there is no live ROS graph. Requires only the `xacro` package (`pip install xacro` or `apt install ros-$ROS_DISTRO-xacro`):

```bash
xacro robot.xacro > robot.urdf
xacro robot.xacro arm_id:=fr3 hand:=true > robot.urdf

# Inside a sourced ROS 2 workspace so package:// / $(find-pkg-share) work:
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
xacro $(ros2 pkg prefix --share my_robot_description)/urdf/robot.xacro \
  > robot.urdf
```

Pass the resulting `.urdf` to Path 1 or Path 2.

## Path 1 — Isaac Sim importer (recommended for RL/Lab)

Modern public API: `isaacsim.asset.importer.urdf.URDFImporter` + `URDFImporterConfig` (dataclass) and the matching `isaacsim.asset.importer.mjcf` pair. The post-import `isaacsim.asset.transformer` runs by default and restructures the USD output (collects dependencies, runs registered rules for physics conversion, materials routing, etc.).

```python
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig(
    urdf_path="/path/robot.urdf",
    usd_path="/path/out",
    merge_fixed_joints=True,
    fix_base=False,
    collision_from_visuals=True,
    collision_type="Convex Decomposition",
    joint_drive_type="force",
    joint_target_type="position",
    override_joint_stiffness=800.0,
    override_joint_damping=40.0,
    robot_type="Manipulator",          # robot-schema token
    run_asset_transformer=True,         # default True; applies transformer profile
    run_multi_physics_conversion=True,  # URDF -> PhysX/MuJoCo physics
)
output_usd = URDFImporter(config).import_urdf()
```

### `URDFImporterConfig` fields (defaults)

| Field | Default | Notes |
|---|---|---|
| `urdf_path`, `usd_path` | `None` | input/output |
| `merge_fixed_joints` | `False` | collapse fixed joints |
| `merge_mesh` | `False` | merge meshes per link |
| `debug_mode` | `False` | extra logging + intermediates |
| `collision_from_visuals` | `False` | derive collision geom from visuals |
| `collision_type` | `"Convex Hull"` | `Convex Hull` / `Convex Decomposition` / `Bounding Sphere` / `Bounding Cube` |
| `allow_self_collision` | `False` | leave off for training |
| `ros_package_paths` | `[]` | resolve `package://` URLs |
| `robot_type` | `"Default"` | robot-schema token; see below |
| `fix_base` | `False` | adds fixed joint world -> root; relocates `ArticulationRootAPI` |
| `link_density` | `None` | kg/m^3 fallback when URDF has no mass |
| `joint_drive_type` | `None` | `force` / `acceleration`; or `{regex: value}` per-joint |
| `joint_target_type` | `None` | `none` / `position` / `velocity`; or per-joint dict |
| `override_joint_stiffness` | `None` | Nm/rad (rev) or N/m (pris); or per-joint dict |
| `override_joint_damping` | `None` | Nm*s/rad / N*s/m; or per-joint dict |
| `run_asset_transformer` | `True` | run transformer profile post-import |
| `run_multi_physics_conversion` | `True` | URDF -> PhysX joint attr conversion |

### CLI (Isaac Sim)

`source/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py` auto-enables `omni.scene.optimizer.core` and `isaacsim.robot.schema`, then applies the config.

```bash
"$ISAAC_SIM_DIR/python.sh" \
  "$ISAAC_SIM_DIR/source/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py" \
  --urdf      /path/robot.urdf \
  --usd-path  /path/out \
  --merge-fixed-joints \
  --fix-base \
  --joint-drive-type force \
  --joint-target-type position \
  --collision-from-visuals --collision-type "Convex Decomposition" \
  --robot-type Manipulator \
  --ros-package my_pkg:/abs/path/to/my_pkg
# --no-run-asset-transformer to skip the transformer profile.
```

`--robot-type` choices come from `usd.schema.isaac.robot_schema.get_allowed_tokens(Attributes.ROBOT_TYPE)`: `Default`, `End Effector`, `Manipulator`, `Humanoid`, `Wheeled`, `Holonomic`, `Quadruped`, `Mobile Manipulators`, `Aerial`.

### MJCF import (parallel API)

```python
from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

config = MJCFImporterConfig(
    mjcf_path="/path/robot.xml",
    usd_path="/path/out",
    import_scene=True,            # include MJCF scene settings
    merge_mesh=True,
    robot_type="Quadruped",
    override_gain_type="fixed",   # MuJoCo actuator gain type
    override_bias_type="affine",  # MuJoCo actuator bias type
    override_gain_prm=[kp, 0, 0, 0, 0, 0, 0, 0, 0, 0],          # position control
    override_bias_prm=[0, -kp, -kd, 0, 0, 0, 0, 0, 0, 0],       # position control
)
output_usd = MJCFImporter(config).import_mjcf()
```

CLI: `source/standalone_examples/api/isaacsim.asset.importer.mjcf/mjcf_import.py` (mirrors URDF: `--mjcf`, `--usd-path`, `--import-scene`, `--robot-type`, `--override-gain-type`, `--override-bias-type`, etc.).

Legacy MJCF commands `MJCFCreateAsset` / `MJCFCreateImportConfig` are deprecated; use `MJCFImporter` directly.

> **Migration:** for the broader `omni.importer.mjcf` / `omni.importer.urdf` → `isaacsim.asset.importer.*` rename map, see [Renaming Extensions](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_4_5/extensions_renaming.html).

### Asset Transformer (what `run_asset_transformer=True` runs)

`isaacsim.asset.transformer` executes ordered USD rule pipelines. Default post-import rules include `UrdfToMjcPhysxConversionRule` / `MjcToPhysxConversionRule` and material routing. To run manually on an existing USD:

```python
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile

manager = AssetTransformerManager()
profile = RuleProfile.from_json("path/to/profile.json")
report  = manager.run("input.usd", profile, package_root="/tmp/out")
```

CLI: `source/standalone_examples/api/isaacsim.asset.transformer/run_asset_transformer.py` (`--input`, `--profile`, `--output`).

### Robot Schema applied during import

The importer applies the modern `usd.schema.isaac.robot_schema`:

- `IsaacRobotAPI` on the robot root prim (stores `robot_type`, ordered link/joint relations, named-pose container).
- `IsaacLinkAPI` on rigid links.
- `IsaacJointAPI` on joints.
- `IsaacSiteAPI` on sites (replaces deprecated `IsaacReferencePointAPI`).
- `IsaacNamedPose` prims for named poses; manage via the `isaacsim.robot.poser` module (see `manipulation-ik`).

Validate after import:

```python
from pxr import Usd
from usd.schema.isaac.robot_schema import Classes, get_allowed_tokens, Attributes

stage = Usd.Stage.Open("/path/out/robot.usd")
robot = next(p for p in stage.Traverse() if p.HasAPI(Classes.ROBOT_API))
print(robot.GetAttribute(Attributes.ROBOT_TYPE).Get())
```

## Path 2 — Isaac Lab `convert_urdf.py` / `convert_mjcf.py` (config.yaml)

For Isaac Lab-native workflows you have a `config.yaml` per robot under `assets/isaaclab/Robots/<Robot>/`:

```yaml
asset_path: /path/to/robot.urdf      # pre-expand XACRO first
usd_file_name: robot_name.usd

force_usd_conversion: true
make_instanceable: true              # critical for parallel envs
import_inertia_tensor: true          # use URDF inertia
merge_fixed_joints: true
self_collision: false

fix_base: false
default_drive_type: none             # "none" | "position" | "velocity"
default_drive_stiffness: 0.0
default_drive_damping: 0.0

link_density: 0.0
convex_decompose_mesh: false
```

Run:

```bash
cd "$ISAAC_LAB_DIR"
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  --config /path/MyRobot/config.yaml --output /path/MyRobot/
# MJCF equivalent:
./isaaclab.sh -p scripts/tools/convert_mjcf.py \
  --config /path/MyRobot/config.yaml --output /path/MyRobot/
```

### Drive presets

| Use case | Drive config |
|---|---|
| RL training (agent controls torques) | `default_drive_type: none`, stiffness `0.0`, damping `0.0` |
| Position-controlled teleop | `default_drive_type: position`, stiffness `800.0`, damping `40.0` |
| Assembly / manipulation objects (AutoMate) | `default_drive_type: force`, `joint_drive.target_type: position`, `gains: {stiffness: 100, damping: 1}`, `collider_type: convex_hull` |

### `make_instanceable: true`

GPU mesh instancing. Without it, 4096 envs * full mesh = VRAM blow-up. With it, parallel envs share one mesh in VRAM. Always set for RL.

### `fix_base` by robot type

| Robot type | `fix_base` |
|---|---|
| Manipulator arm (table/wall mounted) | `true` |
| Mobile robot (wheels) | `false` |
| Humanoid / legged | `false` |
| Aerial (drone) | `false` |

## URDF export (USD -> URDF round-trip)

`isaacsim.asset.exporter.urdf` is the inverse path; useful for sharing imported USD assets back to ROS or other URDF-consumers.

```python
from pxr import Usd
from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

stage = Usd.Stage.Open("/path/robot.usd")
converter = UsdToUrdfConverter(
    stage,
    root_prim_path="/World/robot",
    mesh_dir_name="meshes",
    mesh_path_prefix="./",
    visualize_collision_meshes=False,
    variant_selections=None,
)
converter.export("/path/urdf_out")
```

CLI: `source/standalone_examples/api/isaacsim.asset.exporter.urdf/urdf_export.py` (`--usd-path`, `--output-dir`, `--root-prim`, `--mesh-prefix`, `--variant SET=SELECTION`).

## Common pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| OOM during training | `make_instanceable: false` | set `true` |
| Robot flies apart | `merge_fixed_joints: false` + stiff PD | set true or reduce gains |
| Wrong masses | `import_inertia_tensor: false` + bad geometry | set `true` |
| Self-collision slowdown | `self_collision: true` during training | set `false` |
| XACRO not converting | `URDFImporter` core does not parse XACRO | import from a running node via `isaacsim.ros2.urdf` (`RobotDefinitionReader` / `File -> Import from ROS2 URDF Node`), or pre-expand offline with `xacro robot.xacro > robot.urdf` |
| `package://` URLs unresolved | missing mapping | pass `ros_package_paths=[{"name":..., "path":...}]` or `--ros-package NAME:PATH` |
| MJCF actuators behave wrong | gain/bias type left at MJCF default | set `override_gain_type` / `override_bias_type` / `override_gain_prm` / `override_bias_prm` |
| Transformer output ignored | running an old test asset | delete `usd_path` and re-import with `run_asset_transformer=True` |

## Pip-wheel availability (standalone usage)

Each importer/exporter is also published as a standalone pip wheel for non-Kit consumers. See `.cursor/rules/pip_packaging.mdc` for the design.

| Extension | Wheel |
|---|---|
| `isaacsim.asset.importer.urdf` | `isaacsim-asset-importer-urdf` (deps: `isaacsim-asset-transformer`, `isaacsim-asset-transformer-rules`, `urdf-usd-converter`) |
| `isaacsim.asset.importer.mjcf` | `isaacsim-asset-importer-mjcf` (deps: `mujoco-usd-converter`) |
| `isaacsim.asset.exporter.urdf` | `isaacsim-asset-exporter-urdf` |
| `isaacsim.asset.transformer` | `isaacsim-asset-transformer` |
| `isaacsim.asset.transformer.rules` | `isaacsim-asset-transformer-rules` |
| `isaacsim.robot.schema` | `isaacsim-robot-schema` |

Build via `./repo.sh build_standalone_wheels --ext <name>`.
