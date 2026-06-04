# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tutorial 9, Part 4: Pick and Place (cuMotion RMPflow)

A self-contained pick-and-place controller using cuMotion RMPflow and an 8-phase state machine.
"""

import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
parser.add_argument("--headless", action="store_true")
parser.add_argument(
    "--xrdf-dir",
    type=str,
    default=None,
    help="Directory containing URDF/XRDF robot config files; omit to use the built-in ur10 model",
)
parser.add_argument("--urdf", type=str, default="robot.urdf")
parser.add_argument("--xrdf", type=str, default="robot.xrdf")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless, "hide_ui": False})

if args.headless:
    from isaacsim.core.experimental.utils.app import enable_extension

    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, DomeLight, GroundPlane, Mesh
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.cumotion import (
    CumotionRobot,
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_robot,
    load_cumotion_supported_robot,
)
from isaacsim.storage.native import get_assets_root_path_async

# ========================================================


# <start-pick-place-sequence-snippet>
class UR10ePickPlace:
    """Pick-and-place controller for the UR10e + 2F-140 gripper using cuMotion RMPflow.

    Phases:
        0  Pre-grasp  — arm moves above the cube
        1  Approach   — arm descends to grasp height
        2  Grasp      — gripper closes
        3  Lift       — arm rises with the cube
        4  Transport  — arm moves above the target location
        5  Lower      — arm descends to place height
        6  Release    — gripper opens
        7  Retract    — arm lifts away
    """

    _ROBOT_PRIM_PATH = "/World/ur10e_robot"
    _CUBE_PRIM_PATH = "/World/cube"
    _EE_LINK_NAME = "right_inner_finger"
    _GRIPPER_JOINT = "finger_joint"

    _OPEN_POS: float = 0.0
    _CLOSED_POS: float = 0.5

    _ABOVE_HEIGHT: float = 0.50
    _NEAR_HEIGHT: float = 0.185
    _TOOL_OFFSET: dict[str, float] = {
        "tool0": 0.0,
        "wrist_3_link": 0.04,
    }
    _EE_THRESHOLD: float = 0.02
    _GRIPPER_THRESHOLD: float = 0.04
    _MIN_STEPS: int = 60
    _WARMUP_FRAMES: int = 120
    _PHYSICS_DT: float = 1.0 / 60.0

    _DOWN_ORI: np.ndarray = transform_utils.euler_angles_to_quaternion(np.array([0.0, np.pi, 0.0])).numpy()

    _PHASE_LABELS: tuple[str, ...] = (
        "Pre-grasp: moving above cube",
        "Approach: descending to cube",
        "Grasp: closing gripper",
        "Lift: raising arm",
        "Transport: moving to target",
        "Lower: descending to place",
        "Release: opening gripper",
        "Retract: lifting arm away",
    )

    def __init__(
        self,
        xrdf_dir: str | None = None,
        urdf_filename: str = "robot.urdf",
        xrdf_filename: str = "robot.xrdf",
        cube_position: np.ndarray | None = None,
        target_position: np.ndarray | None = None,
        events_dt: list[int] | None = None,
    ) -> None:
        self._xrdf_dir = xrdf_dir
        self._urdf_filename = urdf_filename
        self._xrdf_filename = xrdf_filename

        self.cube_position = cube_position if cube_position is not None else np.array([0.5, 0.0, 0.025])
        self.target_position = target_position if target_position is not None else np.array([0.5, 0.5, 0.05])
        self.events_dt = events_dt or [250, 150, 100, 50, 150, 100, 100, 100]

        self._event: int = 0
        self._step: int = 0
        self._t: float = 0.0
        self._warmup_remaining: int = self._WARMUP_FRAMES

        self._articulation: Articulation | None = None
        self._ee_prim: GeomPrim | None = None
        self._finger_idx: int | None = None
        self._cumotion_robot: CumotionRobot | None = None
        self._controller: RmpFlowController | None = None
        self._world_binding: mg.WorldBinding | None = None
        self._tool_frame: str | None = None
        self._site_space: list[str] | None = None

    async def setup_scene(self) -> None:
        """Build the scene and initialize the RMPflow controller."""
        assets_root_path = await get_assets_root_path_async()
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path
            + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd",
            path=self._ROBOT_PRIM_PATH,
        )

        GroundPlane("/World/GroundPlane")
        DomeLight("/World/DomeLight").set_intensities(1000)

        cube_obj = Cube(
            paths=self._CUBE_PRIM_PATH, positions=[self.cube_position], sizes=1.0, scales=[0.05, 0.05, 0.05]
        )
        RigidPrim(paths=cube_obj.paths)
        GeomPrim(paths=cube_obj.paths, apply_collision_apis=True)

        await omni.kit.app.get_app().next_update_async()
        set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.5, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

        self._articulation = Articulation(self._ROBOT_PRIM_PATH)
        await omni.kit.app.get_app().next_update_async()

        robot_pos, robot_ori = self._articulation.get_world_poses()
        objects = mg.SceneQuery().get_prims_in_aabb(
            search_box_origin=robot_pos.numpy()[0],
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=[self._ROBOT_PRIM_PATH, self._CUBE_PRIM_PATH],
        )

        obstacle_strategy = mg.ObstacleStrategy()
        for prim_type in (Mesh, Cone, Cylinder):
            obstacle_strategy.set_default_configuration(prim_type, mg.ObstacleConfiguration("obb", 0.01))

        self._world_binding = mg.WorldBinding(
            world_interface=CumotionWorldInterface(),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        self._world_binding.initialize()
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
        self._world_binding.synchronize_transforms()

        if self._xrdf_dir is not None:
            self._cumotion_robot = load_cumotion_robot(
                directory=self._xrdf_dir,
                urdf_filename=self._urdf_filename,
                xrdf_filename=self._xrdf_filename,
            )
        else:
            self._cumotion_robot = load_cumotion_supported_robot("ur10")
        tool_frames = self._cumotion_robot.robot_description.tool_frame_names()
        if len(tool_frames) == 0:
            raise ValueError("No tool frames found in the robot description.")
        self._tool_frame = tool_frames[0]
        if self._tool_frame not in self._TOOL_OFFSET:
            raise ValueError(
                f"Tool frame '{self._tool_frame}' has no entry in _TOOL_OFFSET. "
                f"Add it: {list(self._TOOL_OFFSET.keys())}"
            )
        self._site_space = tool_frames

        self._controller = RmpFlowController(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface(),
            robot_joint_space=self._articulation.dof_names,
            robot_site_space=self._site_space,
            tool_frame=self._tool_frame,
        )
        cfg = self._controller.get_rmp_flow_config()
        # cspace_target_rmp weights delta error from initial position
        # doesn't really matter in our case so decrease from default 50 to 1.0
        cfg.set_param("cspace_target_rmp/metric_scalar", 1.0)

    def initialize_after_play(self) -> None:
        """Resolve EE link and gripper DOF index. Call once after physics starts."""
        link_names = self._articulation.link_names
        if self._EE_LINK_NAME in link_names:
            self._ee_prim = GeomPrim(paths=self._articulation.link_paths[0][link_names.index(self._EE_LINK_NAME)])
        else:
            print(f"WARNING: '{self._EE_LINK_NAME}' not found. Available: {link_names}")

        dof_names = self._articulation.dof_names
        if self._GRIPPER_JOINT in dof_names:
            self._finger_idx = dof_names.index(self._GRIPPER_JOINT)
        else:
            print(f"WARNING: '{self._GRIPPER_JOINT}' not found. Available: {dof_names}")

        n_dofs = len(dof_names)
        init_pos = np.zeros(n_dofs)
        self._articulation.set_dof_positions(init_pos.tolist())
        self._articulation.set_dof_position_targets(init_pos.tolist())

    def _phase_ee_target(self) -> np.ndarray:
        c, p = self.cube_position, self.target_position
        offset = self._TOOL_OFFSET[self._tool_frame]
        hi = self._ABOVE_HEIGHT + offset
        lo = self._NEAR_HEIGHT + offset
        targets = {
            0: [c[0], c[1], c[2] + hi],
            1: [c[0], c[1], c[2] + lo],
            2: [c[0], c[1], c[2] + lo],
            3: [c[0], c[1], c[2] + hi],
            4: [p[0], p[1], p[2] + hi],
            5: [p[0], p[1], p[2] + lo],
            6: [p[0], p[1], p[2] + lo],
            7: [p[0], p[1], p[2] + hi],
        }
        return np.array(targets[self._event], dtype=np.float32)

    def _make_setpoint(self, position: np.ndarray) -> mg.RobotState:
        return mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=self._site_space,
                positions=([self._tool_frame], wp.array([position.tolist()], dtype=wp.float32)),
                orientations=([self._tool_frame], wp.array([self._DOWN_ORI.tolist()], dtype=wp.float32)),
            ),
        )

    def _estimated_state(self) -> mg.RobotState:
        names = self._articulation.dof_names
        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=names,
                positions=(names, self._articulation.get_dof_positions()),
                velocities=(names, self._articulation.get_dof_velocities()),
            )
        )

    def _set_gripper(self, pos: float) -> None:
        if self._finger_idx is not None:
            self._articulation.set_dof_position_targets(
                wp.array([pos], dtype=wp.float32), dof_indices=[self._finger_idx]
            )

    def _ee_near_target(self) -> bool:
        if self._ee_prim is None:
            return False
        ee_pos = self._ee_prim.get_world_poses()[0].numpy()[0]
        return bool(np.linalg.norm(ee_pos - self._phase_ee_target()) < self._EE_THRESHOLD)

    def _gripper_at(self, target: float) -> bool:
        if self._finger_idx is None:
            return False
        pos = float(self._articulation.get_dof_positions().numpy().flatten()[self._finger_idx])
        return abs(pos - target) < self._GRIPPER_THRESHOLD

    def _phase_converged(self) -> bool:
        if self._step < self._MIN_STEPS:
            return False
        if self._event == 2:
            return self._gripper_at(self._CLOSED_POS)
        if self._event == 6:
            return self._gripper_at(self._OPEN_POS)
        return self._ee_near_target()

    def forward(self) -> bool:
        """Advance one simulation step. Returns False when the sequence is complete."""
        if self.is_done():
            return False

        if self._warmup_remaining > 0:
            n_dofs = len(self._articulation.dof_names)
            targets = np.zeros(n_dofs)
            self._articulation.set_dof_position_targets(
                wp.array(targets, dtype=wp.float32), dof_indices=list(range(n_dofs))
            )
            self._warmup_remaining -= 1
            if np.abs(self._articulation.get_dof_positions().numpy().flatten() - targets).max() < 0.1:
                self._warmup_remaining = 0
            return True

        if self._step == 0:
            print(f"  Phase {self._event}: {self._PHASE_LABELS[self._event]}")
            if self._event in (0, 3, 7):
                if not self._controller.reset(
                    self._estimated_state(), self._make_setpoint(self._phase_ee_target()), t=0.0
                ):
                    raise RuntimeError("RmpFlowController reset failed.")
                self._t = 0.0

        if self._event == 2:
            self._set_gripper(self._CLOSED_POS)
        elif self._event == 6:
            self._set_gripper(self._OPEN_POS)
        else:
            self._world_binding.get_world_interface().update_world_to_robot_root_transforms(
                self._articulation.get_world_poses()
            )
            self._world_binding.synchronize_transforms()
            desired = self._controller.forward(
                self._estimated_state(), self._make_setpoint(self._phase_ee_target()), self._t
            )
            if desired is not None and desired.joints.positions is not None:
                self._articulation.set_dof_position_targets(
                    positions=desired.joints.positions, dof_indices=desired.joints.position_indices
                )

        self._t += self._PHYSICS_DT
        self._step += 1
        if self._phase_converged() or self._step >= self.events_dt[self._event]:
            if self._step >= self.events_dt[self._event]:
                print(f"  Phase {self._event} timed out after {self.events_dt[self._event]} frames")
            self._event += 1
            self._step = 0

        return True

    def is_done(self) -> bool:
        return self._event >= len(self.events_dt)

    def reset(self) -> None:
        self._event = 0
        self._step = 0
        self._t = 0.0
        self._warmup_remaining = self._WARMUP_FRAMES


# <end-pick-place-sequence-snippet>


# ========================================================


def main(args: argparse.Namespace, app: SimulationApp) -> None:
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device=args.device)

    scenario = UR10ePickPlace(xrdf_dir=args.xrdf_dir, urdf_filename=args.urdf, xrdf_filename=args.xrdf)
    app.run_coroutine(scenario.setup_scene())
    app.update()

    if args.headless:
        print("Headless mode: simulation is paused. Press Play in the livestream UI to begin.")
    else:
        app_utils.play()
        app.update()
        scenario.initialize_after_play()

    needs_reset = True
    initialized = not args.headless
    frame_count = 0
    while app.is_running():
        app.update()
        if app_utils.is_playing() and SimulationManager.is_simulating():
            if not initialized:
                scenario.initialize_after_play()
                initialized = True
            if needs_reset:
                scenario.reset()
                needs_reset = False
            scenario.forward()
            frame_count += 1
            if args.test and frame_count >= 100:
                break
        elif not app_utils.is_playing():
            needs_reset = True


if __name__ == "__main__":
    try:
        main(args, simulation_app)
    except Exception:
        import traceback

        traceback.print_exc()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        simulation_app.close()
