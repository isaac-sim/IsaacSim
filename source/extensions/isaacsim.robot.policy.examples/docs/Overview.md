# Overview

The isaacsim.robot.policy.examples extension deploys reinforcement-learning policies exported
from Isaac Lab into Isaac Sim. One policy-specific runner drives the six bundled examples
(ANYmal-C, Spot, Go2, H1, Franka drawer opening, and Cartpole) on the supported PhysX and
Newton configurations.

## Quick Start

Deploy the bundled Spot policy through the same public path used by the standalone,
interactive, and MobilityGen examples:

~~~python
import numpy as np

from isaacsim.robot.policy.examples import RobotPolicyRunner, get_spot_spec

# Author the robot while the timeline is stopped.
spot = RobotPolicyRunner(get_spot_spec(), prim_path="/World/Spot")
spot.spawn()

# Start from the configured default state and establish the first target after play begins.
spot.restart_from_default_state([0.0, 0.0, 0.0])

# Call once per physics tick from the caller-owned callback.
def on_physics_step(step_size: float) -> None:
    spot.step(step_size, np.array([1.0, 0.0, 0.0], dtype=np.float32))
~~~

The selected artifact supplies the policy model, environment configuration, and IO descriptor.
The runner deploys the policy trained for the active physics engine unless training_engine
selects another, authors the selected robot USD, binds the policy interface, loads the model,
and owns cadence, target writes, explicit actuators, and cleanup.

A replay reset is deliberately explicit: use restart_from_default_state(command) to restore the
configured state and establish its first target atomically. For a custom state, restore or teleport
the robot first, then call initialize() so controller state is reset after the discontinuity. Later
initialize calls never teleport the robot. Call close() to release model, controller, and actuator
resources.

## Architecture

There is one intentionally narrow MotionGen boundary:

~~~text
PolicyArtifact + PolicyEnvConfig + PolicySpec
                    |
                    v
          IsaacLabPolicyController
              (BaseController)
                    |
             desired RobotState
                    v
               RobotPolicyRunner
       read -> infer -> hold -> apply
                    |
                    v
          Articulation + actuators
~~~

{class}`IsaacLabPolicyController <isaacsim.robot.policy.examples.controller.IsaacLabPolicyController>`
is articulation-independent. One forward call builds an observation, runs one inference, updates
its own previous-action feedback, and decodes a desired RobotState. It owns no USD, articulation,
physics cadence, or target application.

{class}`RobotPolicyRunner <isaacsim.robot.policy.examples.runtime.RobotPolicyRunner>` is a policy
deployment facade, not a generic controller-composition runtime.

The root package exports the deployment types, the bundled `get_*_spec` factories, the Franka
provider factory, and the 6.x class names as raising migration shims. Descriptor binding, model
backends, articulation bridging, and actuator resources stay subsystem internals.

## Artifacts and Custom Policies

{class}`PolicyArtifact <isaacsim.robot.policy.examples.spec.PolicyArtifact>` groups one policy
model with its env config and IO descriptor:

~~~python
from isaacsim.robot.policy.examples import PolicyArtifact, RobotPolicyRunner, PolicySpec

artifact = PolicyArtifact.from_training_run(
    "/path/to/logs/rsl_rl/my_task/2026-07-20_10-00-00",
    model_sha256=trusted_release_manifest["policy.pt"],
)
spec = PolicySpec(name="my_robot", engines={"physx": artifact})
runner = RobotPolicyRunner(spec, prim_path="/World/MyRobot")
~~~

from_files() accepts explicit local or Omniverse paths. from_bundle() reads a flat deployment
bundle. from_training_run() reads an Isaac Lab run, prefers its deployable exported policy over
raw checkpoints, and fails on ambiguous or missing files. Every TorchScript artifact requires its
full SHA-256 from a trusted release manifest; the runner verifies the bytes again before every
deserialization, including cache hits.

The two exported configuration files have separate authority:

- IO_descriptors.yaml defines ordered observation/action terms, shapes, joint order, scales,
  clips, and offsets.
- env.yaml defines physics timing, spawn properties, joint gains and limits, actuator models,
  and initial state.

PolicySpec adds only deployment facts not carried by those exports: per-engine artifacts, an
optional robot USD and spawn defaults, and an advanced binding hook for an interface that the
descriptor exporter cannot represent. When no USD or pose is in the spec, the env config is the
fallback.

## Descriptor Contract

Binding is internal to the deployment path. The supported observation vocabulary is
base linear/angular velocity, projected gravity, a planar [vx, vy, wz] command, absolute or
default-relative joint position/velocity, and previous raw action. Supported outputs are named
joint position, velocity, and effort commands.

Derivation preserves descriptor order and each record's selected joint names and offsets.
Compilation resolves those names against the live articulation, applies the exported affine
transforms, and records the requested control modes. Joint names bind literally: the deployed
articulation must expose the trained names, and unsupported term paths or missing joints fail
at binding; the model runtime separately validates its flat float input and output widths. The deployment path trusts the staged descriptor and does not interpret its dtype,
history, modifier, or articulation-sidecar metadata.

Franka is the one bundled exception: Isaac Lab's exporter omits its custom drawer-distance term.
Its bundled module therefore supplies a concrete binding and a task provider with explicit robot
and cabinet paths. That policy-specific adapter does not create a general scene-query schema.

## Bundled Examples

The isaacsim.robot.policy.examples.bundled package provides one get_<robot>_spec() factory per
example; the factories are also root-package conveniences. Calls without arguments return the
cached bundled default. Their named keyword arguments mirror PolicySpec: ``engines``, ``usd_path``,
``name``, ``default_spawn_position``, ``default_spawn_orientation``,
``zero_targets_on_initialize``, and ``binding``. Omitted fields keep the bundled value, while an
explicit ``None`` replaces it. For example: ``get_h1_spec(default_spawn_position=(0.0, 0.0, 1.2))``.
MobilityGen uses the same H1 and Spot specs; RobotPolicyRunner authors the selected engine asset, the
policy artifact determines the physics timestep, and session teardown closes the runner.

Hosted policy bundles do not yet carry every IO descriptor beside the model and env config, so
validated exports are staged under data/descriptors. Cartpole shares one engine-neutral
descriptor; Franka binds explicitly.

## Lifecycle and Failure Behavior

Use this ordering:

1. Construct the runner and call spawn() while stopped.
2. Start physics and call initialize().
3. Call step(dt, command) exactly once per physics tick.
4. Before a replay or external teleport, restore the robot state, then call initialize().
5. Call close() during teardown.

Tick zero runs inference. Later control ticks follow the artifact decimation and write the
desired state; the engine holds that command, so intermediate ticks write nothing. A reset occurs immediately before the next forward call, controller time
is absolute within each run, and state is read only on control ticks. A failed reset leaves the
runner uninitialized; step errors propagate to the caller.

## Migrating from extension 6.x

Version 7.0.0 removed `PolicyController` and the six articulation-owning policy classes. This is
an extension API version change, not an Isaac Sim product-version migration. The old import paths
remain as shims: imports still resolve, but constructing one of the classes raises a RuntimeError
carrying its replacement code.

| Extension 6.x symbol | Extension 7.0.0 construction |
|---|---|
| AnymalFlatTerrainPolicy | RobotPolicyRunner(get_anymal_spec(), prim_path=...) |
| SpotFlatTerrainPolicy | RobotPolicyRunner(get_spot_spec(), prim_path=...) |
| Go2FlatTerrainPolicy | RobotPolicyRunner(get_go2_spec(), prim_path=...) |
| H1FlatTerrainPolicy | RobotPolicyRunner(get_h1_spec(), prim_path=...) |
| CartpolePolicy | RobotPolicyRunner(get_cartpole_spec(), prim_path=...), stepped without a command |
| FrankaOpenDrawerPolicy | RobotPolicyRunner(get_franka_spec(), ..., task_state_provider=make_franka_task_state_provider(cabinet, env_config)) |
| PolicyController.load_policy | PolicyArtifact.from_files(...) plus PolicySpec |

The lifecycle changes with it: author the robot while the timeline is stopped, `initialize()`
after physics starts, then `step()` once per physics tick. The runner owns policy decimation, so
callers no longer count ticks.

Worked examples for each case -- shipped policies, non-shipped policies, exporting
`IO_descriptors.yaml` from an Isaac Lab run, and the supported term vocabulary -- are in the
{external+isaac-sim-user-guide:doc}`migration guide <migration_guides/isaac_sim_6_1/robot_policy_examples>`.

## Integration

The controller uses the BaseController and RobotState contract from
isaacsim.robot_motion.experimental.motion_generation. Explicit exported actuator models come from
isaacsim.core.experimental.actuators; articulation access uses
isaacsim.core.experimental.prims. ONNX policies run through isaacsim.pip.onnx.
