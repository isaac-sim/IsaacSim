# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deploy the trained Unitree H2 policy in a scripted standalone Isaac Sim demo.

This example keeps the artifact and spec construction visible to demonstrate how to deploy a
custom robot that has no bundled policy spec. Isaac Lab is only used upstream to train and export
the policy; simulation and inference run entirely in Isaac Sim. The runner reads the descriptor's
history settings, so this client does not duplicate policy-input assembly. Run this file with the
Isaac Sim ``python.sh`` launcher, not ``isaaclab.sh``.
"""

from __future__ import annotations

import argparse
import math
import traceback
from functools import partial

from isaacsim import SimulationApp

DEFAULT_H2_POLICY_SHA256 = "0a47182bbe203eddc2b7a9185f822c14d3799723f8b8392c7a2895d25d449c83"

parser = argparse.ArgumentParser(description="Run a trained Unitree H2 policy through a scripted locomotion loop.")
parser.add_argument("--model-path", help="Override policy.pt")
parser.add_argument(
    "--model-sha256",
    default=DEFAULT_H2_POLICY_SHA256,
    help="Full policy.pt SHA-256 obtained from a trusted release manifest; override with --model-path",
)
parser.add_argument("--env-config-path", help="Override env.yaml")
parser.add_argument(
    "--descriptor-path",
    help="Override IO_descriptors.yaml",
)
parser.add_argument(
    "--usd-path",
    help="Override the Unitree H2 robot USD; defaults to the configured Isaac Sim assets root",
)
parser.add_argument(
    "--env-url",
    default="/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    help="Environment path relative to the Isaac Sim assets root",
)
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda", help="PhysX simulation device")
parser.add_argument("--headless", action="store_true", help="Run without a window")
parser.add_argument("--test", action="store_true", help="Run a smoke test, then exit")
parser.add_argument(
    "--max-frames",
    type=int,
    default=0,
    help="Close after this many rendered frames; zero runs until the window closes",
)
args, unknown = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "extra_args": ["--/exts/isaacsim.core.simulation_manager/default_engine=physx"],
    }
)

import carb
import newton
import numpy as np
import omni.timeline
from isaacsim.core.experimental.utils.stage import define_prim, get_current_stage
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import PolicyArtifact, PolicyEnvConfig, PolicySpec, RobotPolicyRunner
from isaacsim.robot.policy.examples.binding import PolicyBinding, derive_binding
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdLux

# Each translation is held for the same duration, and each turn requests one full revolution at
# the maximum trained yaw rate. Opposing phases therefore cancel under perfect command tracking.
TRANSLATION_DURATION = 2.0
TURN_DURATION = math.tau
COMMAND_PHASES = (
    (TRANSLATION_DURATION, (0.5, 0.0, 0.0)),
    (TRANSLATION_DURATION, (-0.5, 0.0, 0.0)),
    (TRANSLATION_DURATION, (0.0, 0.5, 0.0)),
    (TRANSLATION_DURATION, (0.0, -0.5, 0.0)),
    (TURN_DURATION, (0.0, 0.0, 1.0)),
    (TURN_DURATION, (0.0, 0.0, -1.0)),
)
COMMAND_LOOP_DURATION = sum(duration for duration, _ in COMMAND_PHASES)
AUTONOMOUS_ACTION_PATH = "agile.rl_env.mdp.actions.random_actions.RandomPositionAction"

first_step = True
reset_needed = False
policy_failed = False
command_time = 0.0
robot: RobotPolicyRunner | None = None


def derive_h2_policy_binding(artifact: PolicyArtifact, env_config: PolicyEnvConfig) -> PolicyBinding:
    """Derive H2's actor interface with an explicit choice for its custom autonomous action.

    The Agile ``RandomPositionAction`` has exported shape ``[0]``: it consumes no actor output
    and commands upper-body joints autonomously inside the Isaac Lab environment. That manager
    action has no Isaac Sim deployment implementation. This explicit binding therefore removes
    exactly that one zero-width record and does not command those joints. Any descriptor change
    fails instead of being silently accepted.

    Args:
        artifact: H2 policy artifact whose descriptor defines the actor interface.
        env_config: Exported deployment configuration supplied by ``RobotPolicyRunner``.

    Returns:
        The descriptor-derived binding for the model-controlled terms.
    """
    del env_config
    descriptor = artifact.load_descriptor()
    action_records = list(descriptor.get("actions", ()))
    autonomous_records = [record for record in action_records if record.get("full_path") == AUTONOMOUS_ACTION_PATH]
    if len(autonomous_records) != 1 or int(np.prod(autonomous_records[0].get("shape", ()))) != 0:
        raise ValueError(
            "h2_standalone expected exactly one zero-width Agile RandomPositionAction; "
            "the exported descriptor has changed and needs an explicit deployment decision."
        )

    policy_descriptor: dict[str, object] = dict(descriptor)
    policy_descriptor["actions"] = [
        record for record in action_records if record.get("full_path") != AUTONOMOUS_ACTION_PATH
    ]
    return derive_binding(policy_descriptor)


def get_command(elapsed_time: float) -> np.ndarray:
    """Get the scripted command at one time in the repeating loop.

    Args:
        elapsed_time: Simulated time since the loop started.

    Returns:
        Base-frame velocity command as ``[vx, vy, wz]``.
    """
    phase_time = elapsed_time % COMMAND_LOOP_DURATION
    for duration, command in COMMAND_PHASES:
        if phase_time < duration:
            return np.asarray(command, dtype=np.float32)
        phase_time -= duration
    return np.zeros(3, dtype=np.float32)


def on_physics_step(step_size: float, context: object) -> None:
    """Initialize, reset, or advance the H2 policy runner after a physics step.

    Args:
        step_size: Duration of the completed physics step.
        context: User context supplied when the callback was registered.
    """
    del context
    global command_time, first_step, policy_failed, reset_needed
    if policy_failed or robot is None:
        return
    try:
        if first_step or reset_needed:
            command_time = 0.0
            command = get_command(command_time)
            robot.restart_from_default_state(command)
            first_step = False
            reset_needed = False
        else:
            command_time = (command_time + step_size) % COMMAND_LOOP_DURATION
            command = get_command(0.0 if args.test else command_time)
            robot.step(step_size, command)
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        policy_failed = True
        carb.log_error(f"h2_standalone: policy deployment failed: {error}")


def main() -> None:
    """Create the pure Isaac Sim scene and run the H2 policy until the app closes."""
    global command_time, reset_needed, robot

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        raise RuntimeError("Could not find the Isaac Sim assets folder.")
    robot_usd_path = args.usd_path or assets_root_path + "/Isaac/Robots/Unitree/H2/H2.usda"

    ground_prim = define_prim("/World/Ground", "Xform")
    ground_prim.GetReferences().AddReference(assets_root_path + args.env_url)
    define_prim("/World/PhysicsScene", "PhysicsScene")
    light = UsdLux.DistantLight.Define(get_current_stage(), "/World/DistantLight")
    light.CreateIntensityAttr(1500.0)

    policy_directory = f"{assets_root_path}/Isaac/Samples/Policies/h2"
    default_model_path = f"{policy_directory}/policy.pt"
    default_env_config_path = f"{policy_directory}/env.yaml"
    default_descriptor_path = f"{policy_directory}/IO_descriptors.yaml"
    artifact = PolicyArtifact.from_files(
        args.model_path or str(default_model_path),
        args.env_config_path or str(default_env_config_path),
        args.descriptor_path or str(default_descriptor_path),
        model_sha256=args.model_sha256,
    )
    env_config = PolicyEnvConfig.from_file(artifact.env_config_path)

    SimulationManager.set_physics_sim_device(args.device)
    # ``ArticulationActuators`` exposes Newton's canonical target names (``joint_target_q`` and
    # ``joint_target_qd``), while Newton otherwise defaults to its legacy target names.
    newton.use_coord_layout_targets = True
    spec = PolicySpec(
        name="unitree_h2_velocity_history",
        engines={"physx": artifact},
        usd_path=robot_usd_path,
        binding=partial(derive_h2_policy_binding, artifact),
    )
    robot = RobotPolicyRunner(spec, prim_path="/World/H2", training_engine="physx")
    robot.spawn()

    physics_dt = robot.physics_dt
    SimulationManager.set_physics_dt(physics_dt)
    RenderingManager.set_dt(physics_dt * env_config.timing.render_interval)

    callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)
    timeline = omni.timeline.get_timeline_interface()

    print("H2 scripted locomotion loop:")
    print("  forward -> backward -> left -> right -> spin left 360 -> spin right 360 -> repeat")
    print(f"  Policy: {artifact.model_path}")
    print(f"  Environment config: {artifact.env_config_path}")
    print(f"  IO descriptor: {artifact.descriptor_path}")

    test_start_x = None
    try:
        timeline.play()
        simulation_app.update()
        rendered_frames = 0
        while simulation_app.is_running():
            simulation_app.update()
            rendered_frames += 1
            if not SimulationManager.is_simulating() and not reset_needed:
                robot.close()
                command_time = 0.0
                reset_needed = True
            if args.test and test_start_x is None and not first_step:
                positions, _ = robot.articulation.get_world_poses()
                test_start_x = float(positions.numpy()[0, 0])
            if args.test and (policy_failed or command_time >= TRANSLATION_DURATION):
                break
            if args.max_frames > 0 and rendered_frames >= args.max_frames:
                break
        if args.test:
            if policy_failed or test_start_x is None or command_time < TRANSLATION_DURATION:
                raise RuntimeError("H2 forward-motion smoke test failed or did not complete.")
            positions, _ = robot.articulation.get_world_poses()
            forward_distance = float(positions.numpy()[0, 0]) - test_start_x
            print(f"Forward displacement: {forward_distance:.2f} m")
            if forward_distance <= 0.1:
                raise RuntimeError(f"H2 did not move forward: displacement was {forward_distance:.2f} m.")
    finally:
        timeline.stop()
        SimulationManager.deregister_callback(callback_id)
        if not reset_needed:
            robot.close()


exit_code = 0
try:
    main()
except BaseException:  # Report failures, including interrupts, before Kit fast shutdown.
    exit_code = 1
    traceback.print_exc()
finally:
    simulation_app.close(exit_code=exit_code)
raise SystemExit(exit_code)
