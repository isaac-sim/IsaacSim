# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""PINK (Pinocchio) IK controller -- task-based QP inverse kinematics.

Uses the PINK library to solve IK as a quadratic program over weighted
kinematic tasks (end-effector tracking + posture regularisation).  The
underlying Pinocchio model is loaded from a URDF that is auto-exported
from the current USD stage at construction time.

Drop-in compatible with the other IK solvers -- matches the same
three-method interface::

    set_target(position, orientation)
    compute() -> np.ndarray | None
    reset()

If the USD-to-URDF conversion fails (kinematic loops, inconsistent
joint transforms, etc.), construction raises an error and the user should
switch to one of the built-in solvers or fix the USD asset.

Reference:
    PINK IK Solver: https://github.com/stephane-caron/pink
"""

from __future__ import annotations

import importlib
import os
import time
import warnings
import xml.etree.ElementTree as ET
from functools import lru_cache
from typing import Any

import numpy as np
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from .._xform_utils import WorldPosePrimCache, read_world_pose_arrays
from ._utils import ema_blend
from .pink_urdf_export import _export_urdf

_PINK_QP_SOLVER_IMPORTS = {
    "daqp": "daqp",
    "osqp": "osqp",
}


def _resolve_name_from_candidates(name: str, candidates: list[str], kind: str) -> str:
    """Resolve a USD name against prefixed URDF / Pinocchio names."""
    if name in candidates:
        return name

    suffix = f"_{name}"
    suffix_candidates = [candidate for candidate in candidates if candidate.endswith(suffix)]
    if len(suffix_candidates) == 1:
        return suffix_candidates[0]

    contains_candidates = [candidate for candidate in candidates if name in candidate]
    if len(contains_candidates) == 1:
        return contains_candidates[0]

    raise RuntimeError(
        f"Cannot resolve {kind} '{name}'. "
        f"Suffix candidates: {suffix_candidates}; "
        f"Contains candidates: {contains_candidates}; "
        f"Available candidates: {candidates}"
    )


def _read_urdf_joint_names(urdf_path: str, movable_only: bool = True) -> list[str]:
    """Read joint names from a URDF file."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    names: list[str] = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        joint_type = joint.get("type", "")
        if not name:
            continue
        if movable_only and joint_type == "fixed":
            continue
        names.append(name)
    return names


def _freeze_uncontrolled_urdf_joints(urdf_path: str, controlled_joint_names: list[str]) -> list[str]:
    """Convert all movable URDF joints not in *controlled_joint_names* to fixed."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    controlled = set(controlled_joint_names)
    frozen_joints: list[str] = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        joint_type = joint.get("type", "")
        if not name or joint_type == "fixed":
            continue
        if name in controlled:
            continue
        joint.set("type", "fixed")
        frozen_joints.append(name)

    if frozen_joints:
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return frozen_joints


# ---------------------------------------------------------------------------
# Frame name resolution
# ---------------------------------------------------------------------------


def _resolve_frame_name(model: Any, ee_link_name: str) -> str:
    """Find the Pinocchio frame matching the Isaac Sim EE link name.

    The USD-to-URDF exporter may prefix link names with the prim hierarchy
    (e.g. ``GR1T2_fourier_hand_6dof_left_hand_pitch_link`` for an Isaac Sim
    link named ``left_hand_pitch_link``).  This helper tries, in order:

    1. Exact match
    2. Suffix match (frame name ends with ``_<ee_link_name>``)
    3. Exact match on the last ``_``-segment (for deeply prefixed names)

    Returns:
        The resolved frame name string.

    Raises:
        RuntimeError: If no matching frame is found.
    """
    all_frames = [f.name for f in model.frames]
    return _resolve_name_from_candidates(ee_link_name, all_frames, "EE frame")


# ---------------------------------------------------------------------------
# PINK IK solver
# ---------------------------------------------------------------------------


@lru_cache(maxsize=1)
def _get_pink_backend_status() -> tuple[bool, str]:
    """Check whether the optional PINK backend modules are importable."""
    try:
        importlib.import_module("isaacsim.robot_motion.pink")
    except ModuleNotFoundError as exc:
        missing_name = exc.name or "isaacsim.robot_motion.pink"
        return False, f"PINK IK cannot be used because optional Python modules are missing: {missing_name}"
    except Exception as exc:
        return False, f"PINK IK cannot be used because 'isaacsim.robot_motion.pink' failed to import: {exc}"

    missing_modules: list[str] = []
    for module_name in ("pinocchio", "pink.tasks", "qpsolvers"):
        try:
            importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            missing_name = exc.name or module_name
            if missing_name not in missing_modules:
                missing_modules.append(missing_name)
        except Exception as exc:
            return False, f"PINK IK cannot be used because '{module_name}' failed to import: {exc}"

    if missing_modules:
        missing = ", ".join(missing_modules)
        return False, f"PINK IK cannot be used because optional Python modules are missing: {missing}"

    return True, ""


@lru_cache(maxsize=None)
def _get_pink_qp_solver_status(solver_name: str) -> tuple[bool, str]:
    """Check whether a supported PINK QP solver backend is importable."""
    normalized = solver_name.lower()
    module_name = _PINK_QP_SOLVER_IMPORTS.get(normalized)
    if module_name is None:
        supported = ", ".join(sorted(_PINK_QP_SOLVER_IMPORTS))
        return False, f"Unsupported PINK QP solver '{solver_name}'. Choose one of: {supported}"

    available, reason = _get_pink_backend_status()
    if not available:
        return False, reason

    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        missing_name = exc.name or module_name
        return (
            False,
            f"PINK IK solver '{normalized}' is unavailable because optional Python modules are missing: {missing_name}",
        )
    except Exception as exc:
        return False, f"PINK IK solver '{normalized}' failed to import: {exc}"

    return True, ""


class PinkIKController:
    """Per-arm IK solver using PINK (Pinocchio-based task-space QP IK).

    Solves IK via a quadratic program that minimises weighted task errors
    (end-effector tracking + posture regularisation) while respecting
    joint-velocity limits.  A ``PinkKinematicsConfiguration`` maintains
    both a *full model* (all joints, for FK) and a *reduced model*
    (arm joints only, for the QP solve).

    Implements the same interface as ``PositionBasedIKController``::

        set_target(position, orientation)
        compute() -> np.ndarray | None
        reset()
    """

    @classmethod
    def get_backend_status(cls) -> tuple[bool, str]:
        """Return whether the optional PINK backend is available."""
        available, reason = _get_pink_backend_status()
        if not available:
            return False, reason

        for solver_name in cls.supported_qp_solvers():
            solver_available, _ = cls.get_qp_solver_status(solver_name)
            if solver_available:
                return True, ""

        supported = ", ".join(cls.supported_qp_solvers())
        return False, f"PINK IK cannot be used because no supported QP solvers are available: {supported}"

    @classmethod
    def supported_qp_solvers(cls) -> tuple[str, ...]:
        """Return the ordered list of supported QP solver backends."""
        return tuple(_PINK_QP_SOLVER_IMPORTS)

    @classmethod
    def normalize_qp_solver_name(cls, solver_name: str) -> str:
        """Normalizes and validates a user-provided QP solver name."""
        normalized = solver_name.strip().lower()
        if normalized not in _PINK_QP_SOLVER_IMPORTS:
            supported = ", ".join(cls.supported_qp_solvers())
            raise ValueError(f"Unsupported PINK QP solver '{solver_name}'. Choose one of: {supported}")
        return normalized

    @classmethod
    def get_qp_solver_status(cls, solver_name: str) -> tuple[bool, str]:
        """Return whether a specific PINK QP solver backend is available."""
        try:
            normalized = cls.normalize_qp_solver_name(solver_name)
        except ValueError as exc:
            return False, str(exc)
        return _get_pink_qp_solver_status(normalized)

    def __init__(
        self,
        robot: Articulation,
        ee_link: RigidPrim,
        ee_link_index: int,
        num_arm_dofs: int,
        ee_link_name: str,
        articulation_path: str,
        export_root_path: str | None = None,
        position_cost: float = 1.0,
        orientation_cost: float = 1.0,
        posture_cost: float = 1e-3,
        lm_damping: float = 1.0,
        gain: float = 0.5,
        max_joint_step_rad: float = 0.0,
        vr_target_filter: float = 0.0,
        solver: str = "daqp",
    ) -> None:
        """Initialise the PINK IK solver.

        Args:
            robot: Isaac Sim Articulation wrapper.
            ee_link: End-effector RigidPrim.
            ee_link_index: Index of the EE link in the articulation.
            num_arm_dofs: Number of arm DOFs to control.
            ee_link_name: Name of the EE link (resolved against URDF frames).
            articulation_path: USD path of the articulation root.
            export_root_path: USD path of the robot root to export to URDF.
            position_cost: FrameTask position cost weight.
            orientation_cost: FrameTask orientation cost weight.
            posture_cost: PostureTask regularisation cost.
            lm_damping: Levenberg-Marquardt damping on the FrameTask.
            gain: FrameTask gain (low-pass, 0.1=very smooth, 1.0=instant).
            max_joint_step_rad: Max joint change per step in radians. ``0`` disables the clamp.
            vr_target_filter: EMA low-pass filter on the VR target (default 0=no filtering).
            solver: QP solver backend name (``"daqp"`` or ``"osqp"``).

        Raises:
            ImportError: If pinocchio or pink are not installed.
            RuntimeError: If URDF conversion or model loading fails.
        """
        available, reason = self.get_backend_status()
        if not available:
            raise ImportError(reason)

        import pinocchio as pin
        from pink import solve_ik
        from pink.tasks import FrameTask, PostureTask

        from .pink_kinematics_configuration import PinkKinematicsConfiguration

        self._robot = robot
        self._ee_link = ee_link
        self._ee_link_index = ee_link_index
        self._num_arm_dofs = num_arm_dofs
        self._max_joint_step_rad = max(0.0, max_joint_step_rad)
        self._vr_target_filter = np.clip(vr_target_filter, 0.0, 0.99)
        self._solver_name = self.normalize_qp_solver_name(solver)
        solver_available, solver_reason = self.get_qp_solver_status(self._solver_name)
        if not solver_available:
            raise ImportError(solver_reason)
        self._task_gain = max(0.01, gain)
        self._posture_cost = max(0.0, posture_cost)
        self._lm_damping = max(1e-6, lm_damping)
        self._reachable = True
        self._last_time: float = 0.0
        self._ee_frame_name = ee_link_name
        self._pin = pin
        self._solve_ik_fn = solve_ik

        self._raw_position: np.ndarray | None = None
        self._raw_orientation: np.ndarray | None = None
        self._filtered_position: np.ndarray | None = None
        self._filtered_orientation: np.ndarray | None = None
        self._root_link_path: str | None = None
        self._root_link_world_pose_cache = WorldPosePrimCache()

        sim_dof_names = list(robot.dof_names)
        sim_arm_names = sim_dof_names[:num_arm_dofs]

        # ---- 1. Export USD to temp URDF ----
        urdf_path, mesh_dir, root_link_path = _export_urdf(
            articulation_path,
            export_root_path=export_root_path,
        )
        self._temp_dir = os.path.dirname(urdf_path)
        self._root_link_path = root_link_path
        self._root_link_world_pose_cache.set_prim_path(root_link_path)

        # ---- 2. Match joint names (Isaac Sim DOF names vs exported URDF) ----
        urdf_joint_names = _read_urdf_joint_names(urdf_path, movable_only=True)
        resolved_sim_to_urdf = {
            sim_name: _resolve_name_from_candidates(sim_name, urdf_joint_names, "URDF joint")
            for sim_name in sim_arm_names
        }
        resolved_arm_names = [resolved_sim_to_urdf[sim_name] for sim_name in sim_arm_names]
        if len(set(resolved_arm_names)) != len(resolved_arm_names):
            raise RuntimeError(f"Resolved URDF controlled joints are not unique: {resolved_sim_to_urdf}")

        _freeze_uncontrolled_urdf_joints(urdf_path, resolved_arm_names)

        # ---- 3. Build PinkKinematicsConfiguration ----
        self._configuration = PinkKinematicsConfiguration(
            controlled_joint_names=resolved_arm_names,
            urdf_path=urdf_path,
            mesh_path=mesh_dir,
        )

        pin_all_names = self._configuration.all_joint_names_pinocchio_order
        pin_ctrl_names = self._configuration.controlled_joint_names_pinocchio_order

        if len(pin_ctrl_names) != num_arm_dofs:
            missing_in_pin = [name for name in resolved_arm_names if name not in pin_all_names]
            raise RuntimeError(
                f"Joint name mismatch: matched {len(pin_ctrl_names)}/{num_arm_dofs} "
                f"arm DOFs.  Pinocchio all: {pin_all_names}, "
                f"Isaac Sim arm: {sim_arm_names}, "
                f"Resolved URDF arm: {resolved_arm_names}, "
                f"missing in Pinocchio: {missing_in_pin}"
            )

        # ---- 4. Build joint ordering maps ----
        resolved_to_sim = {resolved_name: sim_name for sim_name, resolved_name in resolved_sim_to_urdf.items()}
        # ALL joints: Isaac Sim -> Pinocchio (for configuration.update())
        try:
            self._all_sim_to_pin = np.array([sim_dof_names.index(resolved_to_sim[name]) for name in pin_all_names])
        except KeyError as exc:
            raise RuntimeError(
                f"Pinocchio joint '{exc.args[0]}' does not map back to an Isaac Sim DOF. "
                f"Pinocchio all: {pin_all_names}; resolved mapping: {resolved_sim_to_urdf}"
            ) from exc
        # Controlled joints: Pinocchio -> Isaac Sim arm (for result reorder)
        self._pin_ctrl_to_sim = np.array([sim_arm_names.index(resolved_to_sim[n]) for n in pin_ctrl_names])
        # Controlled joints: Isaac Sim arm -> Pinocchio (for extracting result)
        self._sim_to_pin_ctrl = np.array([pin_ctrl_names.index(resolved_sim_to_urdf[n]) for n in sim_arm_names])

        # ---- 5. Resolve EE frame name in the Pinocchio model ----
        resolved_ee = _resolve_frame_name(self._configuration.full_model, ee_link_name)
        self._ee_frame_name = resolved_ee

        # ---- 6. Create PINK tasks ----
        self._ee_task = FrameTask(
            resolved_ee,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=self._lm_damping,
            gain=self._task_gain,
        )
        self._posture_task = PostureTask(cost=self._posture_cost)
        self._tasks = [self._ee_task, self._posture_task]
        self._apply_task_tuning()

        for task in self._tasks:
            task.set_target_from_configuration(self._configuration)

    # ------------------------------------------------------------------
    # Properties (matching other solver interfaces)
    # ------------------------------------------------------------------

    @property
    def reachable(self) -> bool:
        """Whether the last ``compute()`` produced a valid solution."""
        return self._reachable

    @property
    def vr_target_filter(self) -> float:
        """Return the EMA low-pass filter strength for VR targets."""
        return float(self._vr_target_filter)

    @vr_target_filter.setter
    def vr_target_filter(self, value: float) -> None:
        """Set the EMA low-pass filter strength for VR targets."""
        self._vr_target_filter = np.clip(value, 0.0, 0.99)

    @property
    def max_joint_step_rad(self) -> float:
        """Return the maximum allowed joint change per step in radians."""
        return float(self._max_joint_step_rad)

    @max_joint_step_rad.setter
    def max_joint_step_rad(self, value: float) -> None:
        """Set the maximum allowed joint change per step in radians."""
        self._max_joint_step_rad = max(0.0, value)

    @property
    def task_gain(self) -> float:
        """Return the PINK FrameTask gain."""
        return float(self._task_gain)

    @task_gain.setter
    def task_gain(self, value: float) -> None:
        """Set the PINK FrameTask gain."""
        self._task_gain = max(0.01, value)
        self._apply_task_tuning()

    @property
    def posture_cost(self) -> float:
        """Return the PINK PostureTask regularisation cost."""
        return float(self._posture_cost)

    @posture_cost.setter
    def posture_cost(self, value: float) -> None:
        """Set the PINK PostureTask regularisation cost."""
        self._posture_cost = max(0.0, value)
        self._apply_task_tuning()

    @property
    def lm_damping(self) -> float:
        """Return the Levenberg-Marquardt damping on the FrameTask."""
        return float(self._lm_damping)

    @lm_damping.setter
    def lm_damping(self, value: float) -> None:
        """Set the Levenberg-Marquardt damping on the FrameTask."""
        self._lm_damping = max(1e-6, value)
        self._apply_task_tuning()

    @property
    def qp_solver(self) -> str:
        """Return the current QP solver backend name."""
        return self._solver_name

    @qp_solver.setter
    def qp_solver(self, value: str) -> None:
        """Set the QP solver backend by name."""
        normalized = self.normalize_qp_solver_name(value)
        available, reason = self.get_qp_solver_status(normalized)
        if not available:
            raise ImportError(reason)
        self._solver_name = normalized

    # ------------------------------------------------------------------
    # Interface: set_target / compute / reset
    # ------------------------------------------------------------------

    def set_target(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float] | None,
    ) -> None:
        """Set the 6-DOF goal pose (sim coordinates, xyzw quaternion)."""
        self._raw_position = np.array(position, dtype=np.float64)
        self._raw_orientation = np.array(orientation, dtype=np.float64) if orientation is not None else None

        self._filtered_position = ema_blend(
            self._filtered_position,
            self._raw_position,
            self._vr_target_filter,
        )
        self._filtered_orientation = (
            ema_blend(
                self._filtered_orientation,
                self._raw_orientation,
                self._vr_target_filter,
                normalize=True,
            )
            if self._raw_orientation is not None
            else None
        )

    def compute(self) -> np.ndarray | None:
        """Compute one IK step using PINK's QP solver.

        Returns:
            Absolute joint positions for the first ``num_arm_dofs`` DOFs
            (Isaac Sim order), or ``None`` if no target / physics not ready.
        """
        if self._filtered_position is None:
            return None

        now = time.monotonic()
        dt = now - self._last_time if self._last_time > 0 else 1.0 / 60.0
        dt = min(dt, 0.1)
        self._last_time = now

        try:
            current_dofs = self._robot.get_dof_positions().numpy()
        except (AssertionError, RuntimeError):
            return None

        all_q = current_dofs[0].astype(np.float64)
        arm_q = all_q[: self._num_arm_dofs]

        # Reorder ALL joints Isaac Sim -> Pinocchio and update FK + config
        pin_all_q = all_q[self._all_sim_to_pin]
        target_se3 = self._world_goal_to_root_frame_se3(
            self._filtered_position,
            self._filtered_orientation,
        )
        self._ee_task.set_target(target_se3)
        self._configuration.update(pin_all_q)

        # Solve IK (operates on the reduced / controlled model)
        try:
            if self._solver_name == "osqp":
                from qpsolvers.warnings import SparseConversionWarning

                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", SparseConversionWarning)
                    velocity = self._solve_ik_fn(
                        self._configuration,
                        self._tasks,
                        dt,
                        solver=self._solver_name,
                        safety_break=False,
                    )
            else:
                velocity = self._solve_ik_fn(
                    self._configuration,
                    self._tasks,
                    dt,
                    solver=self._solver_name,
                    safety_break=False,
                )
            delta_q_pin = velocity * dt
        except Exception as exc:
            print(f"[Teleop][PINK] Solve IK failed ({type(exc).__name__}): {exc}")
            print(
                f"[Teleop][PINK]   Solver='{self._solver_name}', "
                f"EE='{self._ee_frame_name}', dt={dt:.5f}, "
                f"arm_q={arm_q.tolist()}"
            )
            self._reachable = False
            return arm_q.astype(np.float32)

        if self._max_joint_step_rad > 0.0:
            delta_q_pin = np.clip(delta_q_pin, -self._max_joint_step_rad, self._max_joint_step_rad)

        # Current controlled joints in Pinocchio order
        arm_q_pin = arm_q[self._sim_to_pin_ctrl]
        new_arm_q_pin = arm_q_pin + delta_q_pin[: len(arm_q_pin)]

        # Reorder back: Pinocchio controlled -> Isaac Sim arm order
        new_arm_q = new_arm_q_pin[self._pin_ctrl_to_sim]

        # Reachability heuristic
        try:
            ee_pos = self._ee_link.get_world_poses()[0].numpy()
            self._reachable = float(np.linalg.norm(self._filtered_position - ee_pos)) < 0.5
        except Exception:
            self._reachable = True

        return new_arm_q.astype(np.float32)

    def reset(self) -> None:
        """Clear target pose, filter state, and temporary files."""
        self._raw_position = None
        self._raw_orientation = None
        self._filtered_position = None
        self._filtered_orientation = None
        self._last_time = 0.0
        self._reachable = True

        if hasattr(self, "_temp_dir") and self._temp_dir:
            import shutil

            if os.path.isdir(self._temp_dir) and "teleop_pink_" in self._temp_dir:
                try:
                    shutil.rmtree(self._temp_dir)
                except OSError:
                    pass
            self._temp_dir = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _apply_task_tuning(self) -> None:
        """Apply runtime-tunable PINK task parameters when tasks exist."""
        if hasattr(self, "_ee_task"):
            try:
                self._ee_task.gain = self._task_gain
                self._ee_task.lm_damping = self._lm_damping
            except Exception:
                pass
        if hasattr(self, "_posture_task"):
            try:
                self._posture_task.cost = self._posture_cost
            except Exception:
                pass

    def _get_root_link_world_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Read the live root-link pose, preferring the articulation tensor state."""
        pin = self._pin

        if self._robot is not None:
            try:
                root_pos_wp, root_quat_wp = self._robot.get_world_poses()
                root_pos = root_pos_wp.numpy().reshape(-1, 3)[0].astype(np.float64)
                root_quat_wxyz = root_quat_wp.numpy().reshape(-1, 4)[0].astype(np.float64)
                root_rot = pin.Quaternion(
                    root_quat_wxyz[0],
                    root_quat_wxyz[1],
                    root_quat_wxyz[2],
                    root_quat_wxyz[3],
                ).toRotationMatrix()
                return root_pos, root_rot
            except Exception as exc:
                print(
                    f"[Teleop][PINK]   Articulation root pose read failed "
                    f"('{self._root_link_path}'): {type(exc).__name__}: {exc}"
                )

        return self._get_root_link_world_pose_from_usd()

    def _get_root_link_world_pose_from_usd(self) -> tuple[np.ndarray, np.ndarray]:
        """Fallback root-link pose read from authored USD transforms."""
        if not self._root_link_path:
            raise RuntimeError("Root link path is unavailable")

        pos_arr, quat_arr = read_world_pose_arrays(self._root_link_world_pose_cache)
        root_pos = np.asarray(pos_arr.reshape(-1, 3)[0], dtype=np.float64)
        w, x, y, z = quat_arr.reshape(-1, 4)[0]
        root_rot = self._pin.Quaternion(float(w), float(x), float(y), float(z)).toRotationMatrix()
        return root_pos, root_rot

    def _world_goal_to_root_frame_se3(
        self,
        position_world: np.ndarray,
        orientation_xyzw: np.ndarray | None,
    ) -> Any:
        """Convert a world-space target into the current URDF root-link frame."""
        pin = self._pin

        if not self._root_link_path:
            return self._xyzw_to_se3(position_world, orientation_xyzw)

        try:
            root_pos, root_rot = self._get_root_link_world_pose()
        except Exception as exc:
            print(
                f"[Teleop][PINK]   Root-link world pose read failed "
                f"('{self._root_link_path}'): {type(exc).__name__}: {exc}"
            )
            return self._xyzw_to_se3(position_world, orientation_xyzw)

        root_rot_inv = root_rot.T
        local_pos = root_rot_inv @ (position_world.reshape(3) - root_pos.reshape(3))

        if orientation_xyzw is not None:
            x, y, z, w = orientation_xyzw
            goal_rot_world = pin.Quaternion(w, x, y, z).toRotationMatrix()
            local_rot = root_rot_inv @ goal_rot_world
        else:
            local_rot = np.eye(3)

        return pin.SE3(local_rot, local_pos.reshape(3, 1))

    def _xyzw_to_se3(self, position: np.ndarray, orientation_xyzw: np.ndarray | None) -> Any:
        """Convert position + xyzw quaternion to ``pinocchio.SE3``."""
        pin = self._pin
        if orientation_xyzw is not None:
            x, y, z, w = orientation_xyzw
            rotation = pin.Quaternion(w, x, y, z).toRotationMatrix()
        else:
            rotation = np.eye(3)
        return pin.SE3(rotation, position.reshape(3, 1))
