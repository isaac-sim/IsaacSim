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

"""Robot arm IK lifecycle manager - orchestrates per-arm IK solvers for VR teleop.

``RobotIKController`` is the entry point called by ``TeleopManager``.  It
creates and drives a per-arm IK solver (position-based or velocity-based) and
only touches the solver through three methods::

    set_target(position, orientation)
    compute() -> np.ndarray | None
    reset()

Swapping in a different IK backend (Lula, learned policy, etc.) is a drop-in
replacement - just match the same interface.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum
from typing import Literal

import omni.timeline
from isaacsim.core.experimental.prims import Articulation, RigidPrim

from ..coordinate_utils import CoordinateSystem, transform_pose
from ._utils import (
    DEFAULT_ROTATION_OFFSET_DEG,
    quat_mul_xyzw,
    rotation_offset_quat_xyzw,
)
from .lm_ik import LMIKController
from .pink_ik import PinkIKController
from .position_ik import PositionBasedIKController
from .velocity_ik import VelocityBasedIKController


class IKMethod(Enum):
    """Differential IK update methods for position-based solver.

    DAMPED_LEAST_SQUARES:
        Most stable default for VR. Handles singularities better and damps jitter.
    PSEUDOINVERSE:
        Direct tracking when well-conditioned, but can be unstable near singularities.
    TRANSPOSE:
        Lightweight and sometimes reactive, but gain-sensitive and less robust.
    SVD:
        Numerically robust with singular-value thresholding, usually heaviest compute.
    """

    DAMPED_LEAST_SQUARES = "damped-least-squares"
    PSEUDOINVERSE = "pseudoinverse"
    TRANSPOSE = "transpose"
    SVD = "singular-value-decomposition"

    @property
    def description(self) -> str:
        """Return a human-readable description of this IK method."""
        return {
            IKMethod.DAMPED_LEAST_SQUARES: "Most stable default; damped near singularities.",
            IKMethod.PSEUDOINVERSE: "Direct tracking, can be unstable near singularities.",
            IKMethod.TRANSPOSE: "Cheapest update; robust but usually less accurate.",
            IKMethod.SVD: "Robust singular-value filtering; typically heaviest compute.",
        }[self]


class IKSolverType(Enum):
    """Available IK solver backends.

    Each member carries capability metadata so UI and lifecycle code can
    query ``supports_method`` / ``supports_gain`` instead of hard-coding
    solver-specific ``if`` checks.
    """

    POSITION_BASED = "position-based"
    VELOCITY_BASED = "velocity-based"
    LEVENBERG_MARQUARDT = "levenberg-marquardt"
    PINK = "pink"

    @property
    def supports_method(self) -> bool:
        """Whether this solver uses the :class:`IKMethod` selection."""
        return self in (IKSolverType.POSITION_BASED, IKSolverType.VELOCITY_BASED)

    @property
    def supports_gain(self) -> bool:
        """Whether this solver exposes a gain parameter."""
        return self == IKSolverType.VELOCITY_BASED

    @property
    def supports_pink_advanced(self) -> bool:
        """Whether this solver exposes PINK-specific task tuning."""
        return self == IKSolverType.PINK

    @property
    def label(self) -> str:
        """Human-readable name for UI display."""
        return {
            "position-based": "Position-based",
            "velocity-based": "Velocity-based",
            "levenberg-marquardt": "Levenberg-Marquardt",
            "pink": "PINK",
        }[self.value]

    @property
    def description(self) -> str:
        """One-line description for tooltips."""
        return {
            "position-based": "Single-step Jacobian differential IK with configurable inversion method.",
            "velocity-based": "Velocity-space IK with proportional gain.",
            "levenberg-marquardt": "Multi-iteration Levenberg-Marquardt per frame (uses damped least-squares internally).",
            "pink": "PINK task-based QP IK (Pinocchio backend) with joint limits and posture regularisation.",
        }[self.value]


@dataclass
class IKValidationResult:
    """Result of IK arm validation."""

    valid: bool
    message: str
    articulation_path: str = ""
    link_names: list[str] = field(default_factory=list)
    dof_names: list[str] = field(default_factory=list)
    num_dofs: int = 0
    arm_dofs: int | None = None


@dataclass
class _ArmState:
    """Per-arm configuration and runtime state."""

    path: str | None = None
    ee_link_name: str = ""
    ee_rot_x_deg: float = DEFAULT_ROTATION_OFFSET_DEG
    ee_rot_y_deg: float = DEFAULT_ROTATION_OFFSET_DEG
    ee_rot_z_deg: float = DEFAULT_ROTATION_OFFSET_DEG
    ik_method: IKMethod = IKMethod.SVD
    num_arm_dofs: int = 7
    scale: float = 1.0
    damping: float = 0.05
    vr_target_filter: float = 0.0
    max_joint_step: float = 0.0
    solver_type: IKSolverType = IKSolverType.POSITION_BASED
    gain: float = 5.0
    pink_qp_solver: str = "daqp"
    pink_task_gain: float = 0.5
    pink_posture_cost: float = 1e-3
    pink_lm_damping: float = 1.0
    running: bool = False
    prev_reachable: bool = True
    resolved_path: str | None = None
    robot: Articulation | None = None
    ee_link: RigidPrim | None = None
    ee_link_index: int = -1
    ctrl: PositionBasedIKController | VelocityBasedIKController | LMIKController | PinkIKController | None = None


_SolverFactory = Callable[
    [_ArmState],
    PositionBasedIKController | VelocityBasedIKController | LMIKController | PinkIKController,
]
_StatusChangedCallback = Callable[[str, bool], None]


# ---------------------------------------------------------------------------
# Solver factory registry
# ---------------------------------------------------------------------------


def _make_position_ik(arm: _ArmState) -> PositionBasedIKController:
    return PositionBasedIKController(
        robot=arm.robot,
        ee_link=arm.ee_link,
        ee_link_index=arm.ee_link_index,
        num_arm_dofs=arm.num_arm_dofs,
        method=arm.ik_method.value,
        scale=arm.scale,
        damping=arm.damping,
        vr_target_filter=arm.vr_target_filter,
        max_joint_step_rad=arm.max_joint_step,
    )


def _make_velocity_ik(arm: _ArmState) -> VelocityBasedIKController:
    return VelocityBasedIKController(
        robot=arm.robot,
        ee_link=arm.ee_link,
        ee_link_index=arm.ee_link_index,
        num_arm_dofs=arm.num_arm_dofs,
        method=arm.ik_method.value,
        damping=arm.damping,
        gain=arm.gain,
        max_joint_step_rad=arm.max_joint_step,
    )


def _make_lm_ik(arm: _ArmState) -> LMIKController:
    return LMIKController(
        robot=arm.robot,
        ee_link=arm.ee_link,
        ee_link_index=arm.ee_link_index,
        num_arm_dofs=arm.num_arm_dofs,
        damping=arm.damping,
        vr_target_filter=arm.vr_target_filter,
        max_joint_step_rad=arm.max_joint_step,
    )


def _make_pink_ik(arm: _ArmState) -> PinkIKController:
    return PinkIKController(
        robot=arm.robot,
        ee_link=arm.ee_link,
        ee_link_index=arm.ee_link_index,
        num_arm_dofs=arm.num_arm_dofs,
        ee_link_name=arm.ee_link_name,
        articulation_path=arm.resolved_path,
        export_root_path=arm.resolved_path,
        solver=arm.pink_qp_solver,
        posture_cost=arm.pink_posture_cost,
        lm_damping=arm.pink_lm_damping,
        gain=arm.pink_task_gain,
        vr_target_filter=arm.vr_target_filter,
        max_joint_step_rad=arm.max_joint_step,
    )


_SOLVER_FACTORY: dict[IKSolverType, _SolverFactory] = {
    IKSolverType.POSITION_BASED: _make_position_ik,
    IKSolverType.VELOCITY_BASED: _make_velocity_ik,
    IKSolverType.LEVENBERG_MARQUARDT: _make_lm_ik,
    IKSolverType.PINK: _make_pink_ik,
}


def _count_chain_dofs(art_path: str, ee_link_name: str) -> int | None:
    """Count movable joints from articulation root to a given EE link.

    Builds a body-to-body adjacency graph from USD joints, then BFS-walks
    from the root link to the EE link.  Fixed joints contribute to
    connectivity but not to the DOF count.

    Returns:
        Number of movable joints along the path, or ``None`` when the
        path cannot be determined (missing prim, disconnected link, etc.).
    """
    from collections import deque

    import omni.usd
    from pxr import Sdf, Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if not stage:
        return None

    try:
        robot = Articulation(art_path)
        link_names = list(robot.link_names)
        if ee_link_name not in link_names:
            return None

        all_link_paths = [str(p) for p in robot.link_paths[0]]
        ee_idx = robot.get_link_indices(ee_link_name).numpy().item()
        ee_link_path = all_link_paths[ee_idx]
        root_link_path = all_link_paths[0]
    except Exception:
        return None

    if ee_link_path == root_link_path:
        return 0

    link_path_set = set(all_link_paths)

    # Search for joints under the articulation root and under the parent
    # of every link (covers assembled robots where gripper links live in
    # a separate subtree).
    search_roots: set[str] = {art_path}
    for lp in all_link_paths:
        search_roots.add(str(Sdf.Path(lp).GetParentPath()))

    visited_joint_paths: set[str] = set()
    adjacency: dict[str, list[tuple[str, bool]]] = {}

    for root_path in search_roots:
        prim = stage.GetPrimAtPath(root_path)
        if not prim:
            continue
        for p in Usd.PrimRange(prim):
            jp = str(p.GetPath())
            if jp in visited_joint_paths or not p.IsA(UsdPhysics.Joint):
                continue
            visited_joint_paths.add(jp)

            joint_api = UsdPhysics.Joint(p)
            b0_targets = joint_api.GetBody0Rel().GetTargets()
            b1_targets = joint_api.GetBody1Rel().GetTargets()
            b0 = str(b0_targets[0]) if b0_targets else None
            b1 = str(b1_targets[0]) if b1_targets else None
            if not b0 or not b1 or b0 not in link_path_set or b1 not in link_path_set:
                continue

            movable = not p.IsA(UsdPhysics.FixedJoint)
            adjacency.setdefault(b0, []).append((b1, movable))
            adjacency.setdefault(b1, []).append((b0, movable))

    queue: deque[tuple[str, int]] = deque([(root_link_path, 0)])
    visited: set[str] = {root_link_path}

    while queue:
        current, dof_count = queue.popleft()
        if current == ee_link_path:
            return dof_count
        for neighbor, movable in adjacency.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, dof_count + (1 if movable else 0)))

    return None


# ---------------------------------------------------------------------------
# Lifecycle manager
# ---------------------------------------------------------------------------


class RobotIKController:
    """Drives robot arms from VR 6DOF targets via IK.

    Creates a per-arm IK solver (position-based or velocity-based).  The
    manager only calls ``set_target`` / ``compute`` / ``reset`` on the solver
    - replacing the backend requires no changes here.

    Per-side configuration:
    - Articulation prim path (e.g. ``/World/Franka``)
    - End-effector link name (e.g. ``panda_hand``)
    - Number of arm DOFs to control (e.g. 7 for Franka, excludes gripper)
    """

    def __init__(self, target_coordinate_system: CoordinateSystem = CoordinateSystem.ISAAC_SIM) -> None:
        self._target_coordinate_system = target_coordinate_system
        self._arms: dict[str, _ArmState] = {
            "left": _ArmState(),
            "right": _ArmState(),
        }
        self._on_status_changed: _StatusChangedCallback | None = None

    def _arm(self, side: str) -> _ArmState:
        return self._arms[side]

    # ------------------------------------------------------------------
    # Global settings
    # ------------------------------------------------------------------

    def set_on_status_changed(self, callback: _StatusChangedCallback | None) -> None:
        """Register a callback invoked when reachability changes.

        Signature: ``callback(side: str, reachable: bool)``.
        """
        self._on_status_changed = callback

    def set_coordinate_system(self, target_coordinate_system: CoordinateSystem) -> None:
        """Set the source coordinate system for input VR wrist pose data."""
        self._target_coordinate_system = target_coordinate_system

    # ------------------------------------------------------------------
    # Per-arm configuration
    # ------------------------------------------------------------------

    def set_articulation_path(self, side: Literal["left", "right"], prim_path: str | None) -> None:
        """Set the articulation prim path for one side."""
        arm = self._arm(side)
        if arm.path == prim_path:
            return
        self.destroy(side)
        arm.path = prim_path

    def set_ee_link_name(self, side: Literal["left", "right"], name: str) -> None:
        """Set the end-effector link name for one side."""
        self._arm(side).ee_link_name = name or ""

    def set_ee_rotation_offsets(
        self,
        side: Literal["left", "right"],
        x_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
        y_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
        z_deg: float = DEFAULT_ROTATION_OFFSET_DEG,
    ) -> None:
        """Set the local-frame XYZ end-effector rotation offsets for one side."""
        arm = self._arm(side)
        arm.ee_rot_x_deg = float(x_deg)
        arm.ee_rot_y_deg = float(y_deg)
        arm.ee_rot_z_deg = float(z_deg)

    def compute_arm_dofs(self, side: Literal["left", "right"]) -> int | None:
        """Return the number of movable joints from root to the current EE link.

        Uses BFS over the USD joint graph.  Returns ``None`` when the
        chain cannot be determined (no path set, no EE link, etc.).
        """
        arm = self._arm(side)
        if not arm.resolved_path or not arm.ee_link_name:
            return None
        return _count_chain_dofs(arm.resolved_path, arm.ee_link_name)

    def set_num_arm_dofs(self, side: Literal["left", "right"], n: int) -> None:
        """Set the number of arm DOFs to control for one side."""
        self._arm(side).num_arm_dofs = max(1, n)

    def set_ik_method(self, side: Literal["left", "right"], method: IKMethod) -> None:
        """Set the differential IK method for one side."""
        arm = self._arm(side)
        arm.ik_method = method
        if arm.ctrl is not None and arm.solver_type.supports_method:
            arm.ctrl.method = method.value

    def get_ik_method(self, side: Literal["left", "right"]) -> IKMethod:
        """Return the current IK method for one side."""
        return self._arm(side).ik_method

    def set_scale(self, side: Literal["left", "right"], scale: float) -> None:
        """Set the IK step scale factor for one side."""
        self._arm(side).scale = scale

    def set_damping(self, side: Literal["left", "right"], damping: float) -> None:
        """Set the DLS damping factor for one side."""
        arm = self._arm(side)
        arm.damping = damping
        if arm.ctrl is not None and hasattr(arm.ctrl, "damping"):
            arm.ctrl.damping = damping

    def set_vr_target_filter(self, side: Literal["left", "right"], value: float) -> None:
        """VR target low-pass filter strength (0.0 = off, ~0.9 = heavy)."""
        clamped = max(0.0, min(0.99, value))
        arm = self._arm(side)
        arm.vr_target_filter = clamped
        if arm.ctrl:
            arm.ctrl.vr_target_filter = clamped

    def set_max_joint_step(self, side: Literal["left", "right"], value: float) -> None:
        """Max joint change per step in radians. ``0.0`` disables the clamp."""
        clamped = max(0.0, value)
        arm = self._arm(side)
        arm.max_joint_step = clamped
        if arm.ctrl:
            arm.ctrl.max_joint_step_rad = clamped

    def set_solver_type(self, side: Literal["left", "right"], solver_type: IKSolverType) -> tuple[bool, str]:
        """Switch IK solver type. Lightweight swap if already set up."""
        arm = self._arm(side)
        if arm.solver_type == solver_type:
            return True, f"Switched to {solver_type.value}"
        available, reason = self.get_solver_availability(solver_type)
        if not available:
            return False, reason
        previous_solver_type = arm.solver_type
        previous_ctrl = arm.ctrl
        arm.solver_type = solver_type
        if arm.robot is not None and arm.ee_link is not None:
            try:
                new_ctrl = self._create_solver(arm)
            except (ImportError, RuntimeError) as exc:
                arm.solver_type = previous_solver_type
                arm.ctrl = previous_ctrl
                message = f"Solver creation failed ({solver_type.value}): {exc}"
                print(f"[Teleop][IK] {message}")
                return False, message
            if previous_ctrl is not None:
                previous_ctrl.reset()
            arm.ctrl = new_ctrl
        return True, f"Switched to {solver_type.value}"

    def get_solver_type(self, side: Literal["left", "right"]) -> IKSolverType:
        """Return the current IK solver type for one side."""
        return self._arm(side).solver_type

    @staticmethod
    def get_solver_availability(solver_type: IKSolverType) -> tuple[bool, str]:
        """Return whether the requested solver backend is currently available."""
        if solver_type == IKSolverType.PINK:
            return PinkIKController.get_backend_status()
        return True, ""

    def set_gain(self, side: Literal["left", "right"], value: float) -> None:
        """Set the gain for solvers that support it. Applied live if running."""
        clamped = max(0.01, value)
        arm = self._arm(side)
        arm.gain = clamped
        if arm.ctrl is not None and arm.solver_type.supports_gain:
            arm.ctrl.gain = clamped

    def get_gain(self, side: Literal["left", "right"]) -> float:
        """Return the current gain value for one side."""
        return self._arm(side).gain

    def set_pink_task_gain(self, side: Literal["left", "right"], value: float) -> None:
        """Set PINK FrameTask gain."""
        clamped = max(0.01, value)
        arm = self._arm(side)
        arm.pink_task_gain = clamped
        if isinstance(arm.ctrl, PinkIKController):
            arm.ctrl.task_gain = clamped

    def get_pink_task_gain(self, side: Literal["left", "right"]) -> float:
        """Return the PINK FrameTask gain for one side."""
        return self._arm(side).pink_task_gain

    def set_pink_qp_solver(self, side: Literal["left", "right"], solver_name: str) -> tuple[bool, str]:
        """Set the QP backend used by the PINK solver."""
        try:
            normalized = PinkIKController.normalize_qp_solver_name(solver_name)
        except ValueError as exc:
            return False, str(exc)

        available, reason = PinkIKController.get_qp_solver_status(normalized)
        if not available:
            return False, reason

        arm = self._arm(side)
        arm.pink_qp_solver = normalized
        if isinstance(arm.ctrl, PinkIKController):
            arm.ctrl.qp_solver = normalized
        return True, f"PINK QP solver set to {normalized}"

    def get_pink_qp_solver(self, side: Literal["left", "right"]) -> str:
        """Return the PINK QP solver name for one side."""
        return self._arm(side).pink_qp_solver

    @staticmethod
    def get_pink_qp_solver_names() -> tuple[str, ...]:
        """Return the names of all supported PINK QP solver backends."""
        return PinkIKController.supported_qp_solvers()

    @staticmethod
    def get_pink_qp_solver_availability(solver_name: str) -> tuple[bool, str]:
        """Return availability status of a PINK QP solver backend."""
        return PinkIKController.get_qp_solver_status(solver_name)

    def set_pink_posture_cost(self, side: Literal["left", "right"], value: float) -> None:
        """Set PINK posture regularisation cost."""
        clamped = max(0.0, value)
        arm = self._arm(side)
        arm.pink_posture_cost = clamped
        if isinstance(arm.ctrl, PinkIKController):
            arm.ctrl.posture_cost = clamped

    def get_pink_posture_cost(self, side: Literal["left", "right"]) -> float:
        """Return the PINK posture regularisation cost for one side."""
        return self._arm(side).pink_posture_cost

    def set_pink_lm_damping(self, side: Literal["left", "right"], value: float) -> None:
        """Set PINK FrameTask damping."""
        clamped = max(1e-6, value)
        arm = self._arm(side)
        arm.pink_lm_damping = clamped
        if isinstance(arm.ctrl, PinkIKController):
            arm.ctrl.lm_damping = clamped

    def get_pink_lm_damping(self, side: Literal["left", "right"]) -> float:
        """Return the PINK Levenberg-Marquardt damping for one side."""
        return self._arm(side).pink_lm_damping

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self, side: Literal["left", "right"]) -> IKValidationResult:
        """Validate and discovers the articulation under the given prim path.

        Searches the prim and its descendants for the first ArticulationRootAPI.
        On success, populates link_names, dof_names, and num_dofs so the UI can
        auto-fill the EE dropdown and DOF count.
        """
        arm = self._arm(side)
        if not arm.path or not arm.path.strip():
            return IKValidationResult(valid=False, message="Set prim path first.")

        try:
            art_paths = Articulation.fetch_articulation_root_api_prim_paths(arm.path)
        except (AssertionError, RuntimeError) as e:
            return IKValidationResult(valid=False, message=f"Prim not found: {e}")

        art_path = art_paths[0] if art_paths else None
        if art_path is None:
            return IKValidationResult(
                valid=False,
                message=f"No ArticulationRootAPI found at or under '{arm.path}'.",
            )

        arm.resolved_path = art_path

        try:
            robot = Articulation(art_path)
        except Exception as e:
            return IKValidationResult(valid=False, message=f"Invalid articulation: {e}")

        link_names = list(robot.link_names)
        dof_names = list(robot.dof_names)
        num_dofs = len(dof_names)

        if arm.num_arm_dofs > num_dofs:
            arm.num_arm_dofs = num_dofs

        partial = IKValidationResult(
            valid=False,
            message="",
            articulation_path=art_path,
            link_names=link_names,
            dof_names=dof_names,
            num_dofs=num_dofs,
        )

        if not arm.ee_link_name or arm.ee_link_name not in link_names:
            partial.message = f"Articulation: {art_path} ({num_dofs} DOFs). Select EE link."
            return partial

        partial.valid = True
        partial.message = f"EE: {arm.ee_link_name}, DOFs: {arm.num_arm_dofs}/{num_dofs}"
        return partial

    # ------------------------------------------------------------------
    # Lifecycle: configure / start / stop / destroy
    # ------------------------------------------------------------------

    @staticmethod
    def _create_solver(
        arm: _ArmState,
    ) -> PositionBasedIKController | VelocityBasedIKController | LMIKController | PinkIKController:
        """Instantiate a solver from the current arm config via the factory registry."""
        factory = _SOLVER_FACTORY.get(arm.solver_type)
        if factory is None:
            raise ValueError(f"Unknown solver type: {arm.solver_type}")
        return factory(arm)

    def configure(self, side: Literal["left", "right"]) -> bool:
        """Validate the articulation and creates the IK solver.

        Heavy operation: validates the prim, creates ``Articulation`` and
        ``RigidPrim`` wrappers, and instantiates the solver.  Call once after
        setting the path and configuration.  Use :meth:`enable` / :meth:`disable`
        to toggle tracking without re-creating resources.

        Returns:
            True if configuration succeeded.
        """
        self.destroy(side)

        result = self.validate(side)
        if not result.valid:
            return False

        arm = self._arm(side)
        if not arm.resolved_path:
            return False

        robot = Articulation(arm.resolved_path)
        ee_link_index = robot.get_link_indices(arm.ee_link_name).numpy().item()
        ee_link_path = robot.link_paths[0][ee_link_index]

        arm.robot = robot
        arm.ee_link = RigidPrim(ee_link_path, reset_xform_op_properties=False)
        arm.ee_link_index = ee_link_index
        try:
            arm.ctrl = self._create_solver(arm)
        except (ImportError, RuntimeError) as exc:
            print(f"[Teleop][IK] Solver creation failed ({arm.solver_type.value}): {exc}")
            arm.robot = None
            arm.ee_link = None
            return False
        return True

    def enable(self, side: Literal["left", "right"]) -> bool:
        """Enable IK tracking for the given side.

        If the solver has not been created yet (no prior :meth:`configure`),
        ``configure`` is called automatically.  Otherwise this is a lightweight
        flag toggle - the solver and articulation wrapper are preserved.

        Returns:
            True if the side is now running.
        """
        arm = self._arm(side)
        if arm.ctrl is None:
            if not self.configure(side):
                return False
        arm.running = True
        arm.prev_reachable = True
        return True

    def disable(self, side: Literal["left", "right"]) -> None:
        """Disable IK tracking without destroying the solver.

        The solver and articulation wrapper stay alive so that
        :meth:`enable` can resume instantly without re-validation.
        """
        arm = self._arm(side)
        arm.running = False
        arm.prev_reachable = True

    def destroy(self, side: Literal["left", "right"]) -> None:
        """Tears down the solver and articulation wrapper for a side.

        Called automatically when the prim path changes or on stage close.
        """
        arm = self._arm(side)
        if arm.ctrl is not None:
            arm.ctrl.reset()
        arm.running = False
        arm.ctrl = None
        arm.ee_link = None
        arm.ee_link_index = -1
        arm.robot = None

    def is_configured(self, side: Literal["left", "right"]) -> bool:
        """True if the solver has been created (via configure or auto-start)."""
        return self._arm(side).ctrl is not None

    def is_running(self, side: Literal["left", "right"]) -> bool:
        """Return True if IK tracking is active for one side."""
        return self._arm(side).running

    def is_reachable(self, side: Literal["left", "right"]) -> bool:
        """True if the last IK step for this side produced a valid solution."""
        ctrl = self._arm(side).ctrl
        return ctrl.reachable if ctrl is not None else True

    # ------------------------------------------------------------------
    # Per-frame update
    # ------------------------------------------------------------------

    def update_targets(
        self,
        left_pos: tuple[float, float, float] | None,
        left_orient: tuple[float, float, float, float] | None,
        right_pos: tuple[float, float, float] | None,
        right_orient: tuple[float, float, float, float] | None,
    ) -> None:
        """Called each frame with VR wrist poses.

        Skips compute/apply when the timeline is not playing - the physics
        tensor is invalid in that state.  Targets are still stored so IK
        starts immediately when the timeline resumes.
        """
        timeline_playing = omni.timeline.get_timeline_interface().is_playing()
        poses = {"left": (left_pos, left_orient), "right": (right_pos, right_orient)}

        for side, (pos, orient) in poses.items():
            arm = self._arm(side)
            if not arm.running or arm.ctrl is None or pos is None:
                continue
            t_pos, t_orient = transform_pose(pos, orient, self._target_coordinate_system)
            if t_orient is not None:
                rotation_offset = rotation_offset_quat_xyzw(arm.ee_rot_x_deg, arm.ee_rot_y_deg, arm.ee_rot_z_deg)
                t_orient = quat_mul_xyzw(t_orient, rotation_offset)
            arm.ctrl.set_target(t_pos, t_orient)
            if timeline_playing:
                self._apply_ik_result(arm)
                reachable = arm.ctrl.reachable
                if reachable != arm.prev_reachable:
                    arm.prev_reachable = reachable
                    if self._on_status_changed is not None:
                        self._on_status_changed(side, reachable)

    def _apply_ik_result(self, arm: _ArmState) -> None:
        """Compute IK and applies joint positions to the articulation."""
        if arm.robot is None or arm.ctrl is None:
            return

        joint_positions = arm.ctrl.compute()
        if joint_positions is None:
            return

        try:
            dof_indices = list(range(arm.num_arm_dofs))
            arm.robot.set_dof_position_targets(joint_positions, dof_indices=dof_indices)
        except (AssertionError, RuntimeError):
            return


_METHOD_VALUE_MAP: dict[str, IKMethod] = {m.value: m for m in IKMethod}
_METHOD_VALUE_MAP.update(
    {
        "dls": IKMethod.DAMPED_LEAST_SQUARES,
        "svd": IKMethod.SVD,
        "pinv": IKMethod.PSEUDOINVERSE,
        "transpose": IKMethod.TRANSPOSE,
    }
)
