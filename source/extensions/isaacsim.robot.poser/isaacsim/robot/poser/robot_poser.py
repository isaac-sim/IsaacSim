# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Robot Poser API.

Provides inverse-kinematics solving, named-pose CRUD, and joint-state
application for robots that carry the ``IsaacRobotAPI`` schema.

Functional requirements covered: FR-01 through FR-14, FR-16, FR-19 through
FR-22.  Behavioral requirements: BR-01 through BR-05.
"""

from __future__ import annotations

import json
import logging
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any

import numpy as np

# Ensure the default LM solver is registered at import time.
import usd.schema.isaac.robot_schema.lm_ik as _lm_ik  # noqa: F401
from pxr import Gf, Sdf, Tf, Usd, UsdGeom
from usd.schema.isaac.robot_schema import Attributes, Classes, Relations
from usd.schema.isaac.robot_schema.ik_solver import IKSolver, IKSolverRegistry, pose_error
from usd.schema.isaac.robot_schema.kinematic_chain import (
    KinematicChain,
    _joint_is_revolute,
)
from usd.schema.isaac.robot_schema.math import (
    Transform,
    VecN,
    _prim_pose_in_robot_frame,
)
from usd.schema.isaac.robot_schema.utils import GetAllNamedPoses

logger = logging.getLogger(__name__)

NAMED_POSES_SCOPE = "NamedPoses"

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class PoseResult:
    """Result of an IK solve or named-pose query.

    Args:
        success: Whether the IK solve converged or the stored pose is valid.
        joints: Mapping of joint prim path to joint value
            (radians for revolute, meters for prismatic).
        joint_fixed: Mapping of joint prim path to fixed flag.
            ``False`` for every movable joint in the chain.
        start_link: Prim path of the chain start link.
        end_link: Prim path of the chain end link / site.
        target_position: Target position ``[x, y, z]`` in robot-base frame.
        target_orientation: Target orientation ``[w, x, y, z]`` quaternion in
            robot-base frame.
    """

    success: bool
    joints: dict[str, float] = field(default_factory=dict)
    joint_fixed: dict[str, bool] = field(default_factory=dict)
    start_link: str = ""
    end_link: str = ""
    target_position: list[float] | None = None
    target_orientation: list[float] | None = None


# ---------------------------------------------------------------------------
# FR-20: Schema Validation
# ---------------------------------------------------------------------------


def validate_robot_schema(robot_prim: Usd.Prim) -> bool:
    """Return True if robot_prim carries the IsaacRobotAPI schema.

    Args:
        robot_prim: Robot root USD prim to check.

    Returns:
        True if the prim has IsaacRobotAPI.
    """
    return robot_prim.IsValid() and robot_prim.HasAPI(Classes.ROBOT_API.value)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _is_simulation_running() -> bool:
    """Return True when the Omniverse timeline is playing.

    Returns:
        True if the timeline is playing.
    """
    try:
        import omni.timeline

        return omni.timeline.get_timeline_interface().is_playing()
    except Exception:
        return False


def _sanitize_name(name: str) -> str:
    """Coerce ``name`` to a valid USD identifier.

    Delegates to :func:`pxr.Tf.MakeValidIdentifier`, which replaces every
    character outside ASCII ``[A-Za-z0-9_]`` with ``_`` and prepends ``_``
    when the result would otherwise start with a digit. The returned string
    is always a valid USD identifier (``Sdf.Path.IsValidIdentifier`` is
    ``True``).

    Empty input is not accepted by ``MakeValidIdentifier``; it is mapped to
    a single underscore here so callers can still address an unnamed pose.

    Args:
        name: Original name string.

    Returns:
        Sanitized name safe for use as a USD prim name.
    """
    if not name:
        return "_"
    return Tf.MakeValidIdentifier(name)


def _to_native_value(stage: Usd.Stage, joint_path: str, value: float) -> float:
    """Convert a joint value from radians to native USD units (degrees for revolute).

    Args:
        stage: USD stage.
        joint_path: Prim path of the joint.
        value: Value in radians (revolute) or meters (prismatic).

    Returns:
        Value in degrees (revolute) or meters (prismatic).
    """
    jprim = stage.GetPrimAtPath(joint_path)
    if jprim and _joint_is_revolute(jprim):
        return float(np.degrees(value))
    return float(value)


def _from_native_value(stage: Usd.Stage, joint_path: str, value: float) -> float:
    """Convert a joint value from native USD units (degrees) to radians for revolute.

    Args:
        stage: USD stage.
        joint_path: Prim path of the joint.
        value: Value in degrees (revolute) or meters (prismatic).

    Returns:
        Value in radians (revolute) or meters (prismatic).
    """
    jprim = stage.GetPrimAtPath(joint_path)
    if jprim and _joint_is_revolute(jprim):
        return float(np.radians(value))
    return float(value)


def _joint_dict_to_array(joint_dict: dict[str, float], joints: list) -> np.ndarray:
    """Convert a joint dict to a numpy array in joint-chain order.

    Args:
        joint_dict: Mapping of joint prim path to value.
        joints: Joint chain list (each element has ``prim_path``).

    Returns:
        Array of values in joint-chain order, defaulting to 0.0 for missing keys.
    """
    return np.array([joint_dict.get(j.prim_path, 0.0) for j in joints], dtype=float)


# Default number of random restarts attempted on cold-start IK when no explicit
# seed and no cached previous solution are available.  Empirically chosen to
# escape the zero-configuration local minimum on 7-DOF arms (e.g. Franka Panda)
# while keeping the cost bounded for callers that solve on the hot path.
_COLD_START_RANDOM_RESTARTS = 3
# Process-global RNG seed for the cold-start random restarts.  Determinism
# (same restarts on every machine, every process) keeps the cold-start path
# reproducible across runs and is required by the ladder unit tests.
# Trade-off: ensemble / parallel callers that solve the same configuration
# concurrently will explore identical restarts; if true ensemble diversity
# is needed pass explicit ``seed=`` instead of relying on the ladder.
_COLD_START_RNG_SEED = 0


def _build_cold_start_seeds(
    joints: list,
    *,
    n_random: int = _COLD_START_RANDOM_RESTARTS,
    rng_seed: int = _COLD_START_RNG_SEED,
) -> list[np.ndarray]:
    """Return a prioritized list of cold-start IK seed candidates.

    The Levenberg-Marquardt solver is sensitive to initialization on
    redundant arms (>=6 DOF).  Starting from the all-zero configuration
    leaves the solver in a flat region of the cost landscape for many
    7-DOF kinematic structures (Franka Panda, etc.), where it converges
    to a wrong-basin local minimum even for trivially reachable targets.
    This helper produces a small, deterministic ladder of starting
    configurations the caller can try in order:

    1. The joint-limit midpoint for joints with finite limits (zeros for
       joints without authored limits).  Mid-range is biased away from
       singularities and matches the null-space centering used inside
       :func:`ik_lm`.
    2. ``n_random`` deterministic uniform-random samples within the
       finite joint limits.  For joints without authored limits the
       sampling range falls back to ``[-1.0, 1.0]`` rad/m so unlimited
       chains still benefit from restarts.
    3. The all-zero configuration (legacy default) as a final fallback.

    Determinism (fixed RNG seed) keeps the cold-start path reproducible
    across runs, across processes, and across machines.  See
    ``_COLD_START_RNG_SEED`` for the ensemble-diversity trade-off.

    Args:
        joints: Chain joints (each must expose ``lower`` and ``upper``).
        n_random: Number of random restart seeds to append.
        rng_seed: RNG seed for the random-restart samples.

    Returns:
        List of joint-value arrays in chain order, ordered by attempt
        priority.  Always non-empty (returns ``[zeros(0)]`` for an empty
        chain).
    """
    n = len(joints)
    if n == 0:
        return [np.zeros(0)]

    lo = np.array([j.lower for j in joints], dtype=float)
    hi = np.array([j.upper for j in joints], dtype=float)
    finite_mask = np.isfinite(lo) & np.isfinite(hi)

    # Compute midpoint only on finite entries; for unbounded joints lo+hi would
    # be -inf + +inf = NaN and emit a RuntimeWarning even though np.where would
    # discard the value.
    midpoint = np.zeros_like(lo)
    midpoint[finite_mask] = 0.5 * (lo[finite_mask] + hi[finite_mask])
    seeds: list[np.ndarray] = [midpoint]

    if n_random > 0:
        rng = np.random.default_rng(rng_seed)
        sample_lo = np.where(finite_mask, lo, -1.0)
        sample_hi = np.where(finite_mask, hi, 1.0)
        for _ in range(n_random):
            seeds.append(rng.uniform(sample_lo, sample_hi))

    seeds.append(np.zeros(n))
    return seeds


# ---------------------------------------------------------------------------
# RobotPoser class
# ---------------------------------------------------------------------------


class RobotPoser:
    """High-level IK controller bound to a specific robot.

    Wraps a :class:`~usd.schema.isaac.robot_schema.kinematic_chain.KinematicChain` (which
    owns the joint chain, kinematic tree, and FK/USD I/O) and adds IK solving,
    solution seeding, and unit conversion on top.

    Construct with a :class:`Usd.Stage` and robot prim, optionally providing
    start/end prims to configure the kinematic chain immediately. The chain can
    be switched at any time via :meth:`set_chain`. Once configured,
    :meth:`solve_ik` only requires a target transform and optional solver
    keyword arguments.

    Args:
        stage: USD stage containing the robot.
        robot_prim: Robot root prim (must carry IsaacRobotAPI).
        start_prim: Start of the IK chain. Optional.
        end_prim: End of the IK chain. Optional.
        solver_name: Name of the registered IK solver. Defaults to registry default.
        debug: Enable verbose debug output for chain building and FK.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        robot_prim: Usd.Prim,
        start_prim: Usd.Prim | None = None,
        end_prim: Usd.Prim | None = None,
        solver_name: str | None = None,
        *,
        debug: bool = False,
    ) -> None:
        self._stage = stage
        self._robot_prim = robot_prim
        self._chain: KinematicChain | None = None
        self._solver: IKSolver = IKSolverRegistry.get(solver_name)
        self._last_solution: np.ndarray | None = None
        self._is_revolute: dict[str, bool] = {}
        self._debug = debug

        if start_prim is not None and end_prim is not None:
            self.set_chain(start_prim, end_prim)

    # -- properties --------------------------------------------------------

    @property
    def stage(self) -> Usd.Stage:
        """The USD stage."""
        return self._stage

    @property
    def robot_prim(self) -> Usd.Prim:
        """The robot root prim."""
        return self._robot_prim

    @property
    def start_prim(self) -> Usd.Prim | None:
        """Start prim of the current IK chain, or ``None``."""
        return self._chain.start_prim if self._chain is not None else None

    @property
    def end_prim(self) -> Usd.Prim | None:
        """End prim of the current IK chain, or ``None``."""
        return self._chain.end_prim if self._chain is not None else None

    @property
    def joints(self) -> list:
        """Copy of the internal joint chain (list of :class:`~usd.schema.isaac.robot_schema.math.Joint`)."""
        return self._chain.joints if self._chain is not None else []

    @property
    def chain(self) -> KinematicChain | None:
        """The underlying :class:`~usd.schema.isaac.robot_schema.kinematic_chain.KinematicChain`, or ``None``."""
        return self._chain

    @property
    def solver(self) -> IKSolver:
        """The active :class:`IKSolver` instance."""
        return self._solver

    @solver.setter
    def solver(self, value: IKSolver) -> None:
        self._solver = value

    # -- chain management --------------------------------------------------

    def set_chain(self, start_prim: Usd.Prim, end_prim: Usd.Prim) -> None:
        """Set or switch the kinematic chain.

        Builds a new :class:`~usd.schema.isaac.robot_schema.kinematic_chain.KinematicChain`
        and resets the solution seed.

        Args:
            start_prim: Start of the IK chain (link or site).
            end_prim: End of the IK chain (link or site).
        """
        self._chain = KinematicChain(
            self._stage,
            self._robot_prim,
            start_prim,
            end_prim,
            debug=self._debug,
        )
        self._last_solution = None

        self._is_revolute = {}
        for j in self._chain.joints:
            jprim = self._stage.GetPrimAtPath(j.prim_path)
            self._is_revolute[j.prim_path] = bool(jprim and jprim.IsValid() and _joint_is_revolute(jprim))

    def set_seed(self, seed: dict[str, float] | np.ndarray | list[float] | None) -> None:
        """Set the solution seed for the next solve_ik call.

        Args:
            seed: When a dict, maps joint prim paths to values (as in PoseResult.joints).
                When array-like, values are in joint-chain order. None clears the seed.
        """
        joints = self._chain.joints if self._chain is not None else []
        if seed is None:
            self._last_solution = None
        elif isinstance(seed, dict):
            self._last_solution = _joint_dict_to_array(seed, joints)
        else:
            self._last_solution = np.asarray(seed, dtype=float)

    # -- IK solve ----------------------------------------------------------

    def solve_ik(
        self,
        target: Transform,
        seed: dict[str, float] | np.ndarray | list[float] | None = None,
        *,
        tolerance: float = 1e-4,
        **solver_kwargs: Any,
    ) -> PoseResult:
        """Solve inverse kinematics for the configured chain.

        Seeding strategy
        ----------------
        The candidates the solver is started from depend on what the caller
        provides:

        * ``seed=...`` (explicit caller-supplied seed): exactly one solve
          attempt from the given configuration. Lowest latency; recommended
          on hot paths.
        * No ``seed=`` and a cached ``_last_solution``: the cached
          configuration is tried first (lowest latency for tracking targets
          near the previous solve). If that attempt does not converge, the
          full cold-start ladder is run in parallel as a fallback so a
          stale cache cannot trap the solver in the wrong basin.
        * No ``seed=`` and no cached solution: the full cold-start ladder
          (joint-limit midpoint, deterministic random restarts within
          joint limits, and the all-zero configuration) runs in parallel
          via a thread pool. The highest-priority converged result wins.

        On every path, when no candidate converges the lowest-error attempt
        is returned with ``success=False`` and a single warning is logged
        recommending an explicit ``seed=``.

        Parallel ladder execution
        -------------------------
        Cold-start (and ``_last_solution`` fallback) ladders run candidate
        seeds concurrently on a :class:`~concurrent.futures.ThreadPoolExecutor`.
        The LM solver in this project is stateless and ``chain.compute_fk``
        does not mutate the chain, so the only shared mutable state across
        attempts is the result aggregation, which happens after the threads
        join. The latency of a cold-start ladder is therefore bounded by the
        slowest single solve, not by the sum of all attempts.

        Args:
            target: Desired end-effector pose in the robot-base frame.
            seed: Initial joint-value guess.  When ``None``, uses the last
                successful solution (with cold-start fallback) if available,
                otherwise runs the cold-start ladder directly.
            tolerance: Convergence threshold on the pose-error norm.
            **solver_kwargs: Forwarded to the IK solver (e.g. lam, iters,
                null_space_bias, joint_fixed). joint_fixed can be a dict
                mapping joint prim path to bool to lock DOFs.

        Returns:
            PoseResult with success, joints, and target info.  On failure
            the joints field still contains the lowest-error attempt so
            callers can inspect or visualize near-misses; callers must
            gate calls to :meth:`apply_pose` on ``success`` to avoid
            teleporting the robot to a random-restart configuration.
        """
        if self._chain is None or not self._chain.joints:
            return PoseResult(success=False)

        joints = self._chain.joints
        n = len(joints)
        joint_fixed_dict = solver_kwargs.pop("joint_fixed", None)
        fixed_mask = (
            [joint_fixed_dict.get(j.prim_path, False) for j in joints]
            if isinstance(joint_fixed_dict, dict)
            else [False] * n
        )
        solver_kwargs["joint_fixed"] = fixed_mask

        # Recompute the start-link pose every call so that changes made
        # by other tracked chains (which move the robot's joints) are
        # reflected in the chain-local target transformation.
        start_pose = _prim_pose_in_robot_frame(self._robot_prim, self._chain.start_prim)
        target_local = start_pose.inv() @ target

        tol_sq = tolerance * tolerance

        def _solve_one(q0: np.ndarray) -> tuple[np.ndarray, float]:
            """Run a single solver attempt and return ``(q_sol, err_sq)``.

            Args:
                q0: Initial joint configuration for the solver attempt.

            Returns:
                Solved joint configuration and squared pose error.
            """
            q_sol = self._solver.solve(self._chain, target_local, q0, **solver_kwargs)
            T_final, _ = self._chain.compute_fk(q_sol)
            err = pose_error(target_local, T_final)
            return q_sol, float(err @ err)

        def _run_ladder(candidate_seeds: list[np.ndarray]) -> tuple[np.ndarray, float, bool, int]:
            """Run the ladder in parallel and return the best result.

            Returns the highest-priority converged result, or the lowest-error
            attempt if none converge.

            Args:
                candidate_seeds: Initial joint configurations to try.

            Returns:
                ``(best_q, best_err_sq, success, winning_index)`` where
                ``winning_index`` is the priority index of the seed whose
                result was selected.
            """
            if len(candidate_seeds) == 1:
                q_sol, err_sq = _solve_one(candidate_seeds[0])
                return q_sol, err_sq, err_sq < tol_sq, 0

            with ThreadPoolExecutor(max_workers=len(candidate_seeds)) as pool:
                # ``pool.map`` preserves submission order, which is the priority
                # order produced by ``_build_cold_start_seeds``.
                results = list(pool.map(_solve_one, candidate_seeds))

            # Pick the highest-priority candidate that converged.
            for idx, (q_sol, err_sq) in enumerate(results):
                if err_sq < tol_sq:
                    return q_sol, err_sq, True, idx
            # No convergence: return the lowest-error attempt (priority is the
            # tie-breaker so the result is deterministic across runs).
            best_idx = 0
            best_err = results[0][1]
            for idx, (_q, err_sq) in enumerate(results):
                if err_sq < best_err:
                    best_err = err_sq
                    best_idx = idx
            return results[best_idx][0], best_err, False, best_idx

        used_ladder_fallback = False
        if seed is not None:
            # Caller-supplied seed: single attempt, no ladder, no fallback.
            if isinstance(seed, dict):
                q0 = _joint_dict_to_array(seed, joints)
            else:
                q0 = np.asarray(seed, dtype=float)
            best_q, best_err_sq, success, _ = _run_ladder([q0])
            is_cold_start = False
        elif self._last_solution is not None:
            # Cache priority: try the cached seed first (single solve, cheap),
            # only fall back to the ladder if it fails so a stale cache cannot
            # trap the solver in the wrong basin.
            cached_q, cached_err_sq = _solve_one(self._last_solution.copy())
            if cached_err_sq < tol_sq:
                best_q, best_err_sq, success = cached_q, cached_err_sq, True
            else:
                # Fall back to the full cold-start ladder in parallel.
                used_ladder_fallback = True
                ladder_seeds = _build_cold_start_seeds(joints)
                best_q, best_err_sq, success, _ = _run_ladder(ladder_seeds)
                # If the cached attempt happens to be better than the best
                # ladder attempt (rare but possible), keep it.
                if cached_err_sq < best_err_sq:
                    best_q, best_err_sq = cached_q, cached_err_sq
                    success = cached_err_sq < tol_sq
            is_cold_start = used_ladder_fallback
        else:
            ladder_seeds = _build_cold_start_seeds(joints)
            best_q, best_err_sq, success, _ = _run_ladder(ladder_seeds)
            is_cold_start = True

        if not success and is_cold_start:
            logger.warning(
                "solve_ik: cold-start IK failed across the seed ladder%s; "
                "best pose-error was %.4f. Provide a near-solution joint "
                "configuration via the 'seed=' parameter for reliable "
                "convergence on redundant arms.",
                " (with stale-cache fallback)" if used_ladder_fallback else "",
                float(np.sqrt(best_err_sq)),
            )

        joint_dict: dict[str, float] = {j.prim_path: float(qval) for j, qval in zip(joints, best_q)}
        fixed_dict: dict[str, bool] = {j.prim_path: fixed_mask[i] for i, j in enumerate(joints)}

        if success:
            self._last_solution = best_q

        return PoseResult(
            success=success,
            joints=joint_dict,
            joint_fixed=fixed_dict,
            start_link=str(self._chain.start_prim.GetPath()),
            end_link=str(self._chain.end_prim.GetPath()),
            target_position=target.t.tolist(),
            target_orientation=target.q.tolist(),
        )

    # -- unit conversion ---------------------------------------------------

    def joints_to_native_values(self, joint_dict: dict[str, float]) -> list[float]:
        """Convert a joint dict (radians) to native USD units.

        Revolute joints are converted to degrees; prismatic joints are
        left unchanged. Values are returned in joint-chain order.

        Args:
            joint_dict: Mapping of joint prim path to value in radians (or meters).

        Returns:
            Values in joint-chain order (degrees for revolute, meters for prismatic).
        """
        joints = self._chain.joints if self._chain is not None else []
        return [
            (
                float(np.degrees(joint_dict.get(j.prim_path, 0.0)))
                if self._is_revolute.get(j.prim_path, False)
                else float(joint_dict.get(j.prim_path, 0.0))
            )
            for j in joints
        ]

    # -- pose application --------------------------------------------------

    def apply_pose(self, joint_dict: dict[str, float] | PoseResult) -> None:
        """Apply joint values to the robot, anchoring at the start link.

        For chains that include backward (child-to-parent) joints the
        standard root-anchored teleport would move the start link. This
        method applies the joints and then rigidly corrects the entire
        robot so the start link remains at its original world position.
        During simulation delegates to _drive_robot directly.

        Args:
            joint_dict: Either a mapping of joint prim path to value
                (radians or meters), or a :class:`PoseResult`. When a
                ``PoseResult`` with ``success=False`` is passed, the call
                is a no-op and a warning is logged: failed solves carry
                the lowest-error attempt across the cold-start ladder,
                which may be a random-restart configuration, and applying
                it would teleport the robot to an arbitrary pose.
        """
        if self._chain is None:
            return

        if isinstance(joint_dict, PoseResult):
            if not joint_dict.success:
                logger.warning(
                    "apply_pose: refusing to apply a failed PoseResult "
                    "(start_link=%r, end_link=%r). The ``joints`` payload "
                    "of a failed solve holds the lowest-error cold-start "
                    "attempt and may be a random-restart configuration; "
                    "applying it would teleport the robot. Pass the dict "
                    "explicitly via ``apply_pose(result.joints)`` to "
                    "override.",
                    joint_dict.start_link,
                    joint_dict.end_link,
                )
                return
            joint_dict = joint_dict.joints

        if _is_simulation_running():
            _drive_robot(self._stage, self._robot_prim, joint_dict)
        elif self._chain.start_prim is not None:
            # Always anchor at start_prim so the root body is written on every
            # call.  The root body prim has reliable authored xform attributes,
            # making the resulting USD change detectable by the viewport
            # manipulator model regardless of whether the chain contains
            # backward joints.
            self._chain.teleport_anchored(joint_dict)
        else:
            self._chain.teleport(joint_dict)

    @classmethod
    def apply_pose_by_target(
        cls,
        stage: Usd.Stage,
        robot_prim: Usd.Prim,
        start_prim: Usd.Prim,
        end_prim: Usd.Prim,
        target: Transform,
        seed: VecN | None = None,
    ) -> PoseResult:
        """Solve IK and immediately apply the result to the robot.

        Constructs a :class:`RobotPoser`, solves IK, and applies the
        solution in a single call — reusing the same kinematic chain for
        both operations.

        Args:
            stage: USD stage containing the robot.
            robot_prim: Robot root prim (must carry IsaacRobotAPI).
            start_prim: Start of the IK chain (link or site).
            end_prim: End of the IK chain (link or site).
            target: Desired end-effector pose in robot-base frame.
            seed: Initial joint guess, or None for zero.

        Returns:
            PoseResult with success, joints, and target info.
        """
        if not validate_robot_schema(robot_prim):
            return PoseResult(success=False)
        poser = cls(stage, robot_prim, start_prim, end_prim)
        result = poser.solve_ik(target, seed=seed)
        if result.success:
            poser.apply_pose(result.joints)
        return result


# ---------------------------------------------------------------------------
# FR-03, FR-04, FR-14: Joint-State Application
# ---------------------------------------------------------------------------


_articulation_cache: dict[str, tuple] = {}


def _get_articulation(robot_path: str) -> tuple[Any, dict[str, int]]:
    """Return a cached (Articulation, dof_path_to_idx) pair for *robot_path*.

    Args:
        robot_path: USD path to the robot prim.

    Returns:
        Tuple of (Articulation, dof_path_to_idx mapping).
    """
    cached = _articulation_cache.get(robot_path)
    if cached is not None:
        return cached

    from isaacsim.core.experimental.prims import Articulation

    articulation = Articulation(robot_path)
    all_dof_paths = articulation.dof_paths[0]
    dof_path_to_idx = {p: i for i, p in enumerate(all_dof_paths)}
    _articulation_cache[robot_path] = (articulation, dof_path_to_idx)
    return articulation, dof_path_to_idx


def invalidate_articulation_cache(robot_path: str | None = None) -> None:
    """Drop cached Articulation instances.

    Args:
        robot_path: If given, drop only that entry; otherwise clear all.
    """
    if robot_path is None:
        _articulation_cache.clear()
    else:
        _articulation_cache.pop(robot_path, None)


def _drive_robot(stage: Usd.Stage, robot_prim: Usd.Prim, joint_dict: dict[str, float]) -> None:
    """Send joint targets to the physics engine (simulation running).

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        joint_dict: Joint path to value (radians or meters).
    """
    robot_path = str(robot_prim.GetPath())
    articulation, dof_path_to_idx = _get_articulation(robot_path)

    positions: list[float] = []
    dof_indices: list[int] = []
    for joint_path, value in joint_dict.items():
        idx = dof_path_to_idx.get(joint_path)
        if idx is not None:
            positions.append(float(value))
            dof_indices.append(idx)

    if positions:
        articulation.set_dof_position_targets(
            np.array([positions]),
            dof_indices=dof_indices,
        )


def apply_joint_state(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    joint_dict: dict[str, float],
) -> None:
    """Apply a joint-state dictionary to the robot.

    When simulation is stopped, teleports via FK and joint attributes.
    When simulation is running, sends DOF targets via Articulation.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim (must carry IsaacRobotAPI).
        joint_dict: Joint prim path to value (radians or meters).
    """
    if _is_simulation_running():
        _drive_robot(stage, robot_prim, joint_dict)
    else:
        KinematicChain(stage, robot_prim).teleport(joint_dict)


def apply_joint_state_anchored(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    joint_dict: dict[str, float],
    anchor_prim: Usd.Prim,
) -> None:
    """Apply a joint-state dictionary, keeping anchor_prim fixed.

    Like apply_joint_state but rigidly corrects so anchor_prim stays at
    its original world position. During simulation sends DOF targets directly.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        joint_dict: Joint prim path to value (radians or meters).
        anchor_prim: Prim to keep fixed.
    """
    if _is_simulation_running():
        _drive_robot(stage, robot_prim, joint_dict)
    else:
        KinematicChain(stage, robot_prim).teleport_anchored(joint_dict, anchor_prim=anchor_prim)


# ---------------------------------------------------------------------------
# FR-11, FR-12, FR-21: Named-Pose Storage
# ---------------------------------------------------------------------------


def _get_named_poses_scope(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    *,
    create: bool = False,
) -> Usd.Prim | None:
    """Return (and optionally create) the Named_Poses scope under the robot.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        create: If True, create the scope if it does not exist.

    Returns:
        The scope prim, or None if not found and create is False.
    """
    scope_path = robot_prim.GetPath().AppendChild(NAMED_POSES_SCOPE)
    scope_prim = stage.GetPrimAtPath(scope_path)
    if not scope_prim or not scope_prim.IsValid():
        if create:
            scope_prim = stage.DefinePrim(scope_path, "Scope")
        else:
            return None
    return scope_prim


def store_named_pose(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_name: str,
    pose_result: PoseResult,
) -> bool:
    """Store a named pose in the robot asset.

    Creates an IsaacNamedPose prim under Named_Poses and registers it
    in the robot's namedPoses relationship. The prim's Xform is set to
    the end-link target pose.

    ``pose_name`` is sanitized to a valid USD identifier via
    :func:`pxr.Tf.MakeValidIdentifier` before use. Distinct inputs that
    sanitize to the same identifier (for example ``"home:v2"`` and
    ``"home/v2"``) refer to the same stored pose; this function logs a
    warning and overwrites the existing prim in that case.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        pose_name: Human-readable name for the pose. Sanitized as described
            above before being used as a USD prim name.
        pose_result: Must have success=True.

    Returns:
        True when the pose was persisted.
    """
    if not pose_result.success:
        return False

    scope = _get_named_poses_scope(stage, robot_prim, create=True)
    if scope is None:
        return False

    safe = _sanitize_name(pose_name)
    pose_path = scope.GetPath().AppendChild(safe)
    existing_prim = stage.GetPrimAtPath(pose_path)
    if existing_prim and existing_prim.IsValid() and safe != pose_name:
        logger.warning(
            "Named pose %r sanitizes to %r, which already exists at %s; overwriting. "
            "Use a name composed only of ASCII letters, digits, and underscores to avoid collisions.",
            pose_name,
            safe,
            pose_path,
        )
    pose_prim = stage.DefinePrim(pose_path, Classes.NAMED_POSE.value)

    # -- Relationships --
    pose_prim.CreateRelationship(Relations.POSE_START_LINK.name).SetTargets([Sdf.Path(pose_result.start_link)])
    pose_prim.CreateRelationship(Relations.POSE_END_LINK.name).SetTargets([Sdf.Path(pose_result.end_link)])

    joint_paths = list(pose_result.joints.keys())
    pose_prim.CreateRelationship(Relations.POSE_JOINTS.name).SetTargets([Sdf.Path(p) for p in joint_paths])

    # -- Attributes --
    joint_values = [_to_native_value(stage, p, pose_result.joints[p]) for p in joint_paths]
    pose_prim.CreateAttribute(Attributes.POSE_JOINT_VALUES.name, Attributes.POSE_JOINT_VALUES.type).Set(joint_values)

    fixed_flags = [pose_result.joint_fixed.get(p, False) for p in joint_paths]
    pose_prim.CreateAttribute(Attributes.POSE_JOINT_FIXED.name, Attributes.POSE_JOINT_FIXED.type).Set(fixed_flags)

    pose_prim.CreateAttribute(Attributes.POSE_VALID.name, Attributes.POSE_VALID.type).Set(True)

    # -- Xform: place the prim at the end-link target pose --
    if pose_result.target_position is not None and pose_result.target_orientation is not None:
        xformable = UsdGeom.Xformable(pose_prim)
        xformable.ClearXformOpOrder()
        for prop in pose_prim.GetPropertiesInNamespace("xformOp"):
            pose_prim.RemoveProperty(prop.GetName())
        t = pose_result.target_position
        q = pose_result.target_orientation
        xformable.AddTranslateOp().Set(Gf.Vec3d(t[0], t[1], t[2]))
        xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        )

    # -- Register in robot's namedPoses relationship --
    rel = robot_prim.GetRelationship(Relations.NAMED_POSES.name)
    if not rel:
        rel = robot_prim.CreateRelationship(Relations.NAMED_POSES.name)
    targets = list(rel.GetTargets())
    if pose_path not in targets:
        targets.append(pose_path)
        rel.SetTargets(targets)

    return True


# ---------------------------------------------------------------------------
# FR-08: Apply Pose by Name
# ---------------------------------------------------------------------------


def apply_pose_by_name(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_name: str,
) -> bool:
    """Apply a previously stored named pose.

    Teleports when simulation is stopped, drives via joint targets when running.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        pose_name: Name of the stored pose. Sanitized to a valid USD
            identifier using the same rules as :func:`store_named_pose`.

    Returns:
        True if the pose was found and applied.
    """
    pose = get_named_pose(stage, robot_prim, pose_name)
    if pose is None or not pose.success:
        return False
    apply_joint_state(stage, robot_prim, pose.joints)
    return True


# ---------------------------------------------------------------------------
# FR-13: Pose Retrieval
# ---------------------------------------------------------------------------


def _read_named_pose_prim(pose_prim: Usd.Prim) -> PoseResult | None:
    """Deserialize a PoseResult from an IsaacNamedPose prim.

    Args:
        pose_prim: USD prim with IsaacNamedPose schema.

    Returns:
        PoseResult, or None if the prim is invalid.
    """
    valid_attr = pose_prim.GetAttribute(Attributes.POSE_VALID.name)
    is_valid = bool(valid_attr.Get()) if valid_attr and valid_attr.Get() is not None else False

    joints_rel = pose_prim.GetRelationship(Relations.POSE_JOINTS.name)
    joint_paths = [str(p) for p in joints_rel.GetTargets()] if joints_rel else []

    values_attr = pose_prim.GetAttribute(Attributes.POSE_JOINT_VALUES.name)
    joint_values = list(values_attr.Get()) if values_attr and values_attr.Get() is not None else []

    fixed_attr = pose_prim.GetAttribute(Attributes.POSE_JOINT_FIXED.name)
    fixed_flags = list(fixed_attr.Get()) if fixed_attr and fixed_attr.Get() is not None else []

    start_rel = pose_prim.GetRelationship(Relations.POSE_START_LINK.name)
    start_link = str(start_rel.GetTargets()[0]) if start_rel and start_rel.GetTargets() else ""

    end_rel = pose_prim.GetRelationship(Relations.POSE_END_LINK.name)
    end_link = str(end_rel.GetTargets()[0]) if end_rel and end_rel.GetTargets() else ""

    # Read the target transform from Xform ops
    target_pos = None
    target_orient = None
    xformable = UsdGeom.Xformable(pose_prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            v = op.Get()
            target_pos = [float(v[0]), float(v[1]), float(v[2])]
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            q = op.Get()
            img = q.GetImaginary()
            target_orient = [
                float(q.GetReal()),
                float(img[0]),
                float(img[1]),
                float(img[2]),
            ]

    stage = pose_prim.GetStage()
    joints_dict: dict[str, float] = {}
    fixed_dict: dict[str, bool] = {}
    for i, path in enumerate(joint_paths):
        val = float(joint_values[i]) if i < len(joint_values) else 0.0
        if stage:
            val = _from_native_value(stage, path, val)
        joints_dict[path] = val
        fixed_dict[path] = bool(fixed_flags[i]) if i < len(fixed_flags) else False

    return PoseResult(
        success=is_valid,
        joints=joints_dict,
        joint_fixed=fixed_dict,
        start_link=start_link,
        end_link=end_link,
        target_position=target_pos,
        target_orientation=target_orient,
    )


def get_named_pose(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_name: str,
) -> PoseResult | None:
    """Retrieve a named pose from the robot asset.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        pose_name: Name of the stored pose. Sanitized to a valid USD
            identifier using the same rules as :func:`store_named_pose`.

    Returns:
        PoseResult, or None when no pose with that name exists.
    """
    scope = _get_named_poses_scope(stage, robot_prim)
    if scope is None:
        return None
    pose_path = scope.GetPath().AppendChild(_sanitize_name(pose_name))
    pose_prim = stage.GetPrimAtPath(pose_path)
    if not pose_prim or not pose_prim.IsValid():
        return None
    return _read_named_pose_prim(pose_prim)


def list_named_poses(stage: Usd.Stage, robot_prim: Usd.Prim) -> list[str]:
    """Return the names of all named poses registered on robot_prim.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.

    Returns:
        List of pose names.
    """
    prims = GetAllNamedPoses(stage, robot_prim)
    return [p.GetName() for p in prims]


def delete_named_pose(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_name: str,
) -> bool:
    """Remove a named pose from the robot asset.

    Deletes the IsaacNamedPose prim and removes it from namedPoses.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        pose_name: Name of the pose to remove. Sanitized to a valid USD
            identifier using the same rules as :func:`store_named_pose`.

    Returns:
        True when the pose existed and was removed.
    """
    scope = _get_named_poses_scope(stage, robot_prim)
    if scope is None:
        return False
    pose_path = scope.GetPath().AppendChild(_sanitize_name(pose_name))
    pose_prim = stage.GetPrimAtPath(pose_path)
    if not pose_prim or not pose_prim.IsValid():
        return False

    # Remove from robot relationship
    rel = robot_prim.GetRelationship(Relations.NAMED_POSES.name)
    if rel:
        targets = [t for t in rel.GetTargets() if t != pose_path]
        rel.SetTargets(targets)

    stage.RemovePrim(pose_path)
    return True


# ---------------------------------------------------------------------------
# FR-16: Import / Export Poses
# ---------------------------------------------------------------------------


def export_poses(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    filepath: str,
    *,
    degrees: bool = False,
) -> bool:
    """Export all named poses on robot_prim to a JSON file.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        filepath: Destination file path.
        degrees: If True, revolute joint values are written in degrees
            (native USD units) instead of the default radians.

    Returns:
        True on success.
    """
    names = list_named_poses(stage, robot_prim)
    data: dict = {}
    for name in names:
        pose = get_named_pose(stage, robot_prim, name)
        if pose is not None:
            joints = pose.joints
            if degrees:
                joints = {
                    p: float(np.degrees(v)) if _joint_is_revolute(stage.GetPrimAtPath(p)) else v
                    for p, v in joints.items()
                }
            data[name] = {
                "joints": joints,
                "joint_fixed": pose.joint_fixed,
                "start_link": pose.start_link,
                "end_link": pose.end_link,
                "target_position": pose.target_position,
                "target_orientation": pose.target_orientation,
                "valid": pose.success,
            }
    units = "degrees" if degrees else "radians"
    meta: dict = {
        "units": units,
        "note": f"Revolute joint values are stored in {units}. USD natively uses degrees.",
    }
    with open(filepath, "w") as fh:
        json.dump({"_meta": meta, "poses": data}, fh, indent=2)
    return True


def import_poses(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    filepath: str,
) -> int:
    """Import named poses from a JSON file and store them on robot_prim.

    Args:
        stage: USD stage.
        robot_prim: Robot root prim.
        filepath: Source file path (as written by export_poses).

    Returns:
        Number of poses successfully imported.
    """
    with open(filepath) as fh:
        raw = json.load(fh)

    # Support both legacy (flat) and new (envelope) formats.
    if "_meta" in raw and "poses" in raw:
        units = raw["_meta"].get("units", "radians")
        data = raw["poses"]
    else:
        units = "radians"
        data = raw

    count = 0
    for name, pd in data.items():
        joints = pd.get("joints", {})
        if units == "degrees":
            joints = {
                p: float(np.radians(v)) if _joint_is_revolute(stage.GetPrimAtPath(p)) else v for p, v in joints.items()
            }
        result = PoseResult(
            success=pd.get("valid", True),
            joints=joints,
            joint_fixed=pd.get("joint_fixed", {}),
            start_link=pd.get("start_link", ""),
            end_link=pd.get("end_link", ""),
            target_position=pd.get("target_position"),
            target_orientation=pd.get("target_orientation"),
        )
        if store_named_pose(stage, robot_prim, name, result):
            count += 1
    return count
