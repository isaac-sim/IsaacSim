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

"""Copy-adapt template for standalone cuMotion RMPflow phase-machine demos.

This is intentionally generic. Replace ``phase_pose`` and validation with the
task-specific object, grasp point, and success oracle.
"""

from __future__ import annotations

import argparse
import json

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
# Window by default: a standalone demo is run by a person, so do NOT default to
# headless. Keep --headless opt-in (store_true). If the script also produces a
# video, gate that behind a separate explicit flag (e.g. --render OUTPUT.MP4)
# that switches to headless capture; never capture/encode unless asked.
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test", action="store_true", help="Validate all motion phases; timeouts fail the run.")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
parser.add_argument("--asset-root", default=None, help="Optional /persistent/isaac/asset_root/default override.")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

import carb.settings
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim, XformPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import CumotionWorldInterface, RmpFlowController, load_cumotion_supported_robot
from isaacsim.storage.native import get_assets_root_path_async

if args.asset_root:
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", args.asset_root)


class CumotionPhaseDemoTemplate:
    ROBOT_PATH = "/World/ur10e_robot"
    TARGET_PATH = "/World/target"
    PHYSICS_DT = 1.0 / 60.0
    PHASE_TIMEOUTS = (240, 240, 120)
    POSITION_TOLERANCE = 0.01
    ORIENTATION_TOLERANCE = 0.05
    CONVERGED_STEPS = 5
    PHASE_LABELS = ("move above target", "descend to target", "retract")
    # WXYZ quaternion for 180 degrees about Y. Avoids running Warp-backed Euler conversion during startup.
    DOWN_ORI = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32)

    def __init__(self) -> None:
        self.robot = None
        self.target = None
        self.world_binding = None
        self.cumotion_robot = None
        self.controller = None
        self.site_space = None
        self.tool_frame = None
        self.tool = None
        self.pose_backend = (
            "usdrt" if carb.settings.get_settings().get_as_bool("/app/useFabricSceneDelegate") else "usd"
        )
        self.converged_steps = 0
        self.phase = 0
        self.phase_step = 0
        self.controller_time = 0.0

    async def setup_scene(self) -> None:
        stage_utils.create_new_stage(template="default stage")
        assets_root = await get_assets_root_path_async(skip_check=True)
        stage_utils.add_reference_to_stage(
            usd_path=assets_root + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd",
            path=self.ROBOT_PATH,
        )
        GroundPlane("/World/GroundPlane")
        DomeLight("/World/DomeLight").set_intensities(np.array([1000.0]))
        self.target = Cube(self.TARGET_PATH, sizes=0.04, positions=[0.5, 0.0, 0.18], colors="green")
        GeomPrim(self.TARGET_PATH, apply_collision_apis=True)
        await omni.kit.app.get_app().next_update_async()

        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.4, 1.2, 0.9], target=[0.45, 0.0, 0.2])
        self.robot = Articulation(self.ROBOT_PATH)
        self.cumotion_robot = load_cumotion_supported_robot("ur10")
        self._build_world_binding()
        self._build_controller()

    def initialize_after_play(self) -> None:
        zeros = np.zeros(len(self.robot.dof_names), dtype=np.float32)
        self.robot.set_dof_positions(wp.array(zeros, dtype=wp.float32))
        self.robot.set_dof_position_targets(wp.array(zeros, dtype=wp.float32))
        # Resolve the measured tool frame explicitly; never substitute the gripper tip.
        stage = stage_utils.get_current_stage()
        paths = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(self.ROBOT_PATH + "/") and prim.GetName() == self.tool_frame
        ]
        if len(paths) != 1:
            raise RuntimeError(f"Resolve a unique measured prim for tool frame {self.tool_frame!r}: {paths}")
        self.tool = XformPrim(paths[0])
        self.reset()

    def _build_world_binding(self) -> None:
        robot_pos, robot_ori = self.robot.get_world_poses()
        objects = mg.SceneQuery().get_prims_in_aabb(
            search_box_origin=robot_pos.numpy()[0],
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[self.ROBOT_PATH, self.TARGET_PATH],
        )
        obstacle_strategy = mg.ObstacleStrategy()
        for prim_type in (Mesh, Cone, Cylinder):
            obstacle_strategy.set_default_configuration(prim_type, mg.ObstacleConfiguration("obb", 0.01))
        self.world_binding = mg.WorldBinding(
            world_interface=CumotionWorldInterface(),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        self.world_binding.initialize()
        self.world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
        self.world_binding.synchronize_transforms()

    def _build_controller(self) -> None:
        self.site_space = self.cumotion_robot.robot_description.tool_frame_names()
        if not self.site_space:
            raise RuntimeError("No cuMotion tool frames found.")
        self.tool_frame = self.site_space[0]
        self.controller = RmpFlowController(
            cumotion_robot=self.cumotion_robot,
            cumotion_world_interface=self.world_binding.get_world_interface(),
            robot_joint_space=self.robot.dof_names,
            robot_site_space=self.site_space,
            tool_frame=self.tool_frame,
        )
        self.controller.get_rmp_flow_config().set_param("cspace_target_rmp/metric_scalar", 1.0)

    def reset(self) -> None:
        self.converged_steps = 0
        self.phase = 0
        self.phase_step = 0
        self.controller_time = 0.0

    def phase_pose(self) -> tuple[np.ndarray, np.ndarray]:
        target_pos = self.target.get_world_poses()[0].numpy()[0]
        positions = (
            target_pos + np.array([0.0, 0.0, 0.30]),
            target_pos + np.array([0.0, 0.0, 0.08]),
            target_pos + np.array([0.0, 0.0, 0.35]),
        )
        return positions[self.phase].astype(np.float32), self.DOWN_ORI

    def estimated_state(self) -> mg.RobotState:
        names = self.robot.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self.robot.get_dof_positions()),
                velocities=(names, self.robot.get_dof_velocities()),
            )
        )

    def setpoint_state(self, position: np.ndarray, orientation: np.ndarray) -> mg.RobotState:
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=self.site_space,
                positions=([self.tool_frame], wp.array([position.tolist()], dtype=wp.float32)),
                orientations=([self.tool_frame], wp.array([orientation.tolist()], dtype=wp.float32)),
            )
        )

    def measure_target_error(self, position: np.ndarray, orientation: np.ndarray) -> tuple[float, float]:
        """Measure tool position and shortest quaternion angle errors in world space."""
        # Read live Fabric transforms when FSD is active; authored USD may be stale.
        with backend_utils.use_backend(self.pose_backend):
            positions, orientations = self.tool.get_world_poses()
        measured_position = positions.numpy()[0]
        measured_orientation = orientations.numpy()[0]
        position_error = float(np.linalg.norm(measured_position - position))
        norm = float(np.linalg.norm(measured_orientation) * np.linalg.norm(orientation))
        if not np.isfinite(position_error) or not np.isfinite(norm) or norm <= 0.0:
            raise RuntimeError("Invalid measured tool pose or target.")
        cosine = abs(float(np.dot(measured_orientation, orientation))) / norm
        orientation_error = float(2.0 * np.arccos(np.clip(cosine, 0.0, 1.0)))
        return position_error, orientation_error

    def step(self) -> bool:
        if self.phase >= len(self.PHASE_LABELS):
            return False
        target_pos, target_orn = self.phase_pose()
        if self.phase_step == 0:
            print(f"phase {self.phase}: {self.PHASE_LABELS[self.phase]} target={target_pos.tolist()}")
            ok = self.controller.reset(self.estimated_state(), self.setpoint_state(target_pos, target_orn), t=0.0)
            if not ok:
                raise RuntimeError("RmpFlowController reset failed.")
            self.controller_time = 0.0

        self.world_binding.get_world_interface().update_world_to_robot_root_transforms(self.robot.get_world_poses())
        self.world_binding.synchronize_transforms()
        desired = self.controller.forward(
            self.estimated_state(),
            self.setpoint_state(target_pos, target_orn),
            self.controller_time,
        )
        self.controller_time += self.PHYSICS_DT
        if desired is not None and desired.joints is not None:
            joints = desired.joints
            if joints.positions is not None:
                self.robot.set_dof_position_targets(joints.positions, dof_indices=joints.position_indices)
            if joints.velocities is not None:
                self.robot.set_dof_velocity_targets(joints.velocities, dof_indices=joints.velocity_indices)
            if joints.efforts is not None:
                self.robot.set_dof_efforts(joints.efforts, dof_indices=joints.effort_indices)

        self.phase_step += 1
        position_error, orientation_error = self.measure_target_error(target_pos, target_orn)
        converged = position_error <= self.POSITION_TOLERANCE and orientation_error <= self.ORIENTATION_TOLERANCE
        self.converged_steps = self.converged_steps + 1 if converged else 0
        if self.converged_steps >= self.CONVERGED_STEPS:
            print(
                json.dumps(
                    {
                        "phase": self.PHASE_LABELS[self.phase],
                        "status": "CONVERGED",
                        "position_error_m": position_error,
                        "orientation_error_rad": orientation_error,
                    }
                )
            )
            self.phase += 1
            self.phase_step = 0
            self.converged_steps = 0
        elif self.phase_step >= self.PHASE_TIMEOUTS[self.phase]:
            raise TimeoutError(
                f"Phase {self.PHASE_LABELS[self.phase]!r} timed out after {self.phase_step} steps: "
                f"position_error_m={position_error:.6f}, orientation_error_rad={orientation_error:.6f}"
            )
        return self.phase < len(self.PHASE_LABELS)


def main() -> None:
    SimulationManager.setup_simulation(dt=CumotionPhaseDemoTemplate.PHYSICS_DT, device=args.device)
    demo = CumotionPhaseDemoTemplate()
    simulation_app.run_coroutine(demo.setup_scene())
    app_utils.play()
    simulation_app.update()
    demo.initialize_after_play()

    while simulation_app.is_running():
        simulation_app.update()
        if app_utils.is_playing() and SimulationManager.is_simulating():
            running = demo.step()
            if not running:
                break

    if demo.phase != len(demo.PHASE_LABELS):
        raise RuntimeError("Run interrupted before all required motion phases converged.")
    # This scaffold validates tool motion only. Adapted manipulation demos must also
    # validate physical grasp, transport, release, and settled object pose before PASS.
    print(json.dumps({"status": "MOTION_COMPLETE", "scope": "tool_motion_only"}))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}))
        raise
    finally:
        simulation_app.close()
