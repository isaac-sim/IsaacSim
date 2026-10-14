# Deploy a robot policy in Isaac Sim

Use this workflow for a bundled or custom robot policy supported by `isaacsim.robot.policy.examples`.
Training and export happen in the training framework; simulation and inference run in Isaac Sim.
The [Unitree H2 sample](h2-policy-deployment.md) supplies concrete export and launch commands for one robot.
Its task name, binding hook, joint count, observation history, and motion thresholds are sample-specific.

## Select the robot, task, and artifacts

Identify the robot USD, training task/run, physics engine, model format, and intended behavior before choosing
an example. Inspect the installed `standalone_examples/api/isaacsim.robot.policy.examples/` examples
(`source/standalone_examples/` in a checkout) for a matching robot/task. A bundled `get_*_spec` factory is
appropriate only when its robot and task match. For a custom policy, construct its own `PolicySpec`.

Export the actor using the selected training framework's supported export command. Export the IO descriptor
from the same task configuration and run. Export flags differ between Isaac Lab, AGILE, and other training
tools; do not substitute the H2 task or assume its export command applies to another robot.
Do not silently select the newest run or load a raw training checkpoint as a deployable actor.

| Artifact | Deployment role |
|---|---|
| `policy.pt` or `policy.onnx` | Exported TorchScript or ONNX actor |
| `env.yaml` | Initial state, actuators, gains, limits, and simulation/control timing |
| `IO_descriptors.yaml` | Ordered observations/actions, scales, joint names, and history |
| `agent.yaml` | Training record; not a deployment input |
| `model_sha256` (custom TorchScript) | Full 64-character SHA-256 of the exported actor, from a trusted producer's export or release manifest |

Reuse the exported `env.yaml` directly. Do not reconstruct its gains, pose, or timing, or strip YAML tags to
bypass parsing. Keep all deployment artifacts from the same task/run and verify the USD joint names match.
For a custom TorchScript actor, record the digest's source and the exact exported `.pt` it identifies.
Pass it as `model_sha256`; a missing digest is rejected. ONNX does not require this keyword, and shipped
examples already supply the digest for their bundled actor.

Choose the artifact constructor that matches the files you have:

- `PolicyArtifact.from_training_run(run_dir, model_sha256=trusted_digest)` reads `exported/policy.pt`,
  `params/env.yaml`, and `io_descriptors/IO_descriptors.yaml` from an Isaac Lab-style run.
- `PolicyArtifact.from_bundle` reads a flat deployment bundle. Specify the backend
  when both model formats are present.
- `PolicyArtifact.from_files(...)` accepts separate local paths or Omniverse URLs, including ONNX exports
  outside that training-run layout.

When selecting TorchScript with `from_bundle` or `from_files`, also pass `model_sha256=trusted_digest`.
For example, `PolicyArtifact.from_files(model_path, env_config_path, descriptor_path, model_sha256=trusted_digest)`
constructs the artifact for a custom `.pt` actor; the runner verifies the model bytes before loading it.

## Construct and run the deployment

After starting `SimulationApp`, a custom ONNX deployment can construct the runner as follows. Set
`model_path` to the exported `.onnx` file; the other paths and engine come from the selected artifacts and robot.

```python
from isaacsim.robot.policy.examples import PolicyArtifact, PolicySpec, RobotPolicyRunner

artifact = PolicyArtifact.from_files(model_path, env_config_path, descriptor_path)
spec = PolicySpec(name="custom_policy", engines={engine: artifact}, usd_path=robot_usd_path)
runner = RobotPolicyRunner(spec, prim_path="/World/Robot", training_engine=engine)
runner.spawn()  # While the timeline is stopped.
```

Let the runner derive its binding from the descriptor. Add an explicit binding or task-state provider only
when the task needs it. Unsupported terms require a deployment decision; do not drop nonzero action terms
or copy another robot's binding to force model dimensions to fit. Observation history and policy decimation
are handled by the controller; do not implement them again in the client.

Set physics/render timing from the exports, register a physics callback, start the timeline, and initialize
the runner after physics starts. Call `runner.step(dt, command)` once per physics tick for policies with a
command channel, where `command` is the base-frame planar velocity `[vx, vy, wz]`. For policies without a
command channel, call `runner.step(dt)` instead.

Supply task observations through the `task_state_provider` constructor argument, not through `step()`.
The provider factory receives the articulation during initialization and returns an object with a
`read()` method that supplies the task-state mapping. See `franka_standalone.py` for a manipulation example.
On reset, restore the robot state before reinitializing the runner.

Preserve the trained physical environment, including the actual robot roots, ground/contact surfaces, and
manipulated objects. Do not use the navigation skill's blanket physics-stripping recipe for deployment,
even on a heavy scene or when the robot is outside `/World/Robot`. Keep required rigid-body, collision, and
joint APIs intact; investigate scene performance without removing task physics.

Launch the selected example or custom deployment with Isaac Sim's `python.sh` (Linux) or `python.bat`
(Windows), from the installation/build root. Inspect that script's arguments before supplying model/config
overrides: `--model-path`, `--env-config-path`, `--descriptor-path`, and `--test` are not universal runner flags.
Quote paths and capture the process status immediately with `$?` in Bash or `$LASTEXITCODE` in PowerShell.

## Validate behavior and failures

Record the command, resolved artifacts, robot USD, build/skill/MCP versions, console output, and process
status. Require status zero plus task-specific evidence: for locomotion, command tracking and stability;
for manipulation, task completion and appropriate joint/contact behavior. Compare with the training
evaluation, including zero commands and resets where applicable. A window or startup banner proves only
startup. A bounded frame count alone does not prove that inference ran or the task succeeded.

Use disposable paths or artifact copies for negative checks, preserving originals:

| Check | Expected result / diagnosis |
|---|---|
| Missing environment config, descriptor, or model | Nonzero status and a diagnostic identifying the failed artifact path |
| Custom TorchScript model with a wrong, well-formed digest | Nonzero status and a digest-mismatch diagnostic before model deserialization |
| Descriptor/model width or joint mismatch | Binding error; verify artifacts, task, and robot USD agree |
| Policy callback or task validation fails | Nonzero status; inspect physics stepping, initialization, timing, and actuator configuration |

Do not accept a zero exit on a failed deployment, even if the application closed. Older builds may hide
exceptions during fast shutdown. Launchers may normalize failure codes, so check for nonzero status rather
than one exact code.

## Teardown and lookup

The caller owns the stage, timeline, physics callbacks, and reset operation. On completion or failure, stop
the timeline, remove callbacks, close the runner, and close `SimulationApp`. Report failures before Kit fast
shutdown and preserve nonzero status through cleanup, including when graceful shutdown returns to Python.
Confirm the launched process exits and callbacks do not survive a restart.

For MCP search, combine the robot/task name with `RobotPolicyRunner`, `PolicyArtifact`, `IO_descriptors.yaml`,
and `tutorial_policy_deployment`. Check results against the installed build. If retrieval misses, inspect
the installed example and this workflow instead of inventing APIs. The general source tutorial is
`docs/isaacsim/isaac_lab_tutorials/tutorial_policy_deployment.rst`; the linked H2 sample provides a worked case.
