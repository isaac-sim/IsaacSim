# Reference sample: deploy a Unitree H2 policy

Apply the [robot policy deployment workflow](policy-deployment.md), using the H2-specific details below.
These AGILE commands, binding rules, and validation thresholds apply to `Velocity-H2-History-v0`.

Use the shipped `standalone_examples/api/isaacsim.robot.policy.examples/h2_standalone.py` as the executable
reference. In a source checkout it is under `source/standalone_examples/`. Run it with Isaac Sim's
`python.sh` or `python.bat`; training and export happen upstream in AGILE/Isaac Lab.

## Discover and export the artifacts

For a shipped-policy smoke test, the example resolves these defaults from the configured Isaac Sim asset root:

- Robot: `/Isaac/Robots/Unitree/H2/H2.usda`.
- Policy bundle: `/Isaac/Samples/Policies/h2/`, containing `policy.pt`, `env.yaml`, and `IO_descriptors.yaml`.

For a newly trained policy, select the exact timestamped AGILE `Velocity-H2-History-v0` training run.
Do not silently choose the newest run or pass a raw `model_*.pt` training checkpoint to the deployment script.
Evaluation of a raw checkpoint exports the actor under `exported/`; from the AGILE repository, use:

```bash
uv run scripts/eval.py --task Velocity-H2-History-v0 --load_run RUN_DIRECTORY_NAME \
    --num_envs 1 --num_steps 10000 --real-time --viz kit
```

Before changing the training task configuration, export its IO descriptor into that same run:

```bash
uv run scripts/train.py --task Velocity-H2-History-v0 --num_envs 1 --max_iterations 0 \
    --run_name export_io_descriptors --headless --viz none \
    env.export_io_descriptors=true \
    env.log_dir=logs/rsl_rl/velocity_h2_history/RUN_DIRECTORY_NAME
```

The zero-iteration invocation also creates an unused timestamped run. Use the descriptor under the explicit
`env.log_dir`, and verify these files belong to the selected training run:

| File relative to the run | Deployment role |
|---|---|
| `exported/policy.pt` | Exported TorchScript actor, not the training checkpoint |
| `params/env.yaml` | Initial state, actuators, gains, limits, and simulation/control timing |
| `io_descriptors/IO_descriptors.yaml` | Ordered observations/actions, scales, joint names, and history |
| `params/agent.yaml` | Training record; not a deployment input |

Reuse `params/env.yaml` directly. Do not reconstruct its gains, initial pose, or timing manually, and do not
rewrite its tagged YAML to bypass parsing.

## Run and validate

Run from the Isaac Sim installation/build root. For the shipped policy on Linux:

```bash
./python.sh standalone_examples/api/isaacsim.robot.policy.examples/h2_standalone.py --headless --test
deployment_status=$?
```

Windows PowerShell:

```powershell
.\python.bat standalone_examples/api/isaacsim.robot.policy.examples/h2_standalone.py --headless --test
$deploymentStatus = $LASTEXITCODE
```

For a custom TorchScript run, set `H2_RUN_DIR` to the absolute training-run directory and `H2_POLICY_SHA256`
to the exported actor's full digest from its trusted producer's export or release manifest. Record that
source with the artifacts. Overriding only `--model-path` retains the shipped actor's default digest, so
pass the matching `--model-sha256` for the replacement actor. The shipped-policy commands above need no override.

Linux:

```bash
./python.sh standalone_examples/api/isaacsim.robot.policy.examples/h2_standalone.py \
    --model-path "$H2_RUN_DIR/exported/policy.pt" --model-sha256 "$H2_POLICY_SHA256" \
    --env-config-path "$H2_RUN_DIR/params/env.yaml" \
    --descriptor-path "$H2_RUN_DIR/io_descriptors/IO_descriptors.yaml" --headless --test
deployment_status=$?
```

Windows PowerShell, with the same values assigned to `$H2_RUN_DIR` and `$H2_POLICY_SHA256`:

```powershell
.\python.bat standalone_examples/api/isaacsim.robot.policy.examples/h2_standalone.py `
    --model-path "$H2_RUN_DIR/exported/policy.pt" --model-sha256 "$H2_POLICY_SHA256" `
    --env-config-path "$H2_RUN_DIR/params/env.yaml" `
    --descriptor-path "$H2_RUN_DIR/io_descriptors/IO_descriptors.yaml" --headless --test
$deploymentStatus = $LASTEXITCODE
```

Use `--usd-path` to override the compatible H2 asset; another robot/task needs its own policy specification
and binding.

Require exit status zero and `Forward displacement: ... m` greater than 0.1 m. The smoke test commands
forward motion for two simulated seconds. Its startup output names the policy, environment config, and IO
descriptor. An open window or the startup banner alone is not a successful deployment.
For visual inspection, omit `--headless --test`; the command loop runs forward, backward, left, right, and
turns both ways. `--max-frames` bounds a visual run; it is not a substitute for the smoke-test assertions.

## H2 binding and lifecycle

H2 has no bundled `get_h2_spec()` factory. The example constructs `PolicyArtifact.from_files(...)`, a
`PolicySpec` for PhysX with the H2 USD and `derive_h2_policy_binding`, and a `RobotPolicyRunner`.

Preserve H2's binding hook: it removes exactly the zero-width AGILE `RandomPositionAction`, which consumed no
actor output during training. The remaining policy controls 14 leg/waist joints, with five-frame observation
history handled by the controller. Do not delete unsupported nonzero action terms or duplicate history in
the client to force a model to fit. For another humanoid, check its actual descriptor and model dimensions.

The shipped example owns the timeline, callback, runner, and application cleanup described in the general
workflow. It initializes/restarts from the default state on the first physics step and after reset, then
calls `runner.step(dt, command)` on subsequent steps.

## H2 negative-path check

Append `--env-config-path "/absolute/nonexistent/env.yaml"` to the shipped smoke-test command
and capture `$?` immediately afterward (`$LASTEXITCODE` in PowerShell). A zero exit on this negative test is
a failure even if the application closed. Older builds may hide exceptions during shutdown; report that
behavior rather than accepting the run. Do not promise an exact nonzero code: launchers can normalize it.

For the custom TorchScript command, replace `--model-sha256` with a deliberately wrong 64-character hex
value, such as 64 zeroes, while keeping the model/config/descriptor paths valid. Require nonzero status and
a digest-mismatch diagnostic before model deserialization. A malformed short value tests argument validation
instead. Preserve the original artifacts and restore the correct digest for subsequent runs.

## H2 documentation and MCP lookup

When using MCP semantic search, query `h2_standalone`, `RobotPolicyRunner`, `PolicyArtifact`,
`IO_descriptors.yaml`, and `tutorial_humanoid_workflow_deploy`. Check retrieved examples against the installed
build. If the index misses the workflow, inspect the installed example and this skill instead of inventing APIs.
In a source checkout, the maintained tutorials are:

- `docs/isaacsim/robot_setup_tutorials/tutorial_humanoid_workflow_train.rst`: training evaluation and model export.
- `docs/isaacsim/robot_setup_tutorials/tutorial_humanoid_workflow_deploy.rst`: H2 IO export and deployment.
- `docs/isaacsim/isaac_lab_tutorials/tutorial_policy_deployment.rst`: artifact contract and runner lifecycle.
