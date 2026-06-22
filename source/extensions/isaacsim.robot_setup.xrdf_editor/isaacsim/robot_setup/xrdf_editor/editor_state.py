# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Domain state and high-level import/export operations for the XRDF editor.

This class deliberately holds zero UI references. The :class:`Extension`
orchestrator owns a single :class:`EditorState` and the UI panels read/write
through it. Tests construct one directly.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.prims import Articulation

from . import articulation_discovery, lula_io, sphere_generation, xrdf_io
from .collision_sphere_editor import CollisionSphereEditor
from .constants import (
    DEFAULT_ACCELERATION_LIMIT,
    DEFAULT_JERK_LIMIT,
)


class EditorState:
    """Domain-level state for one editing session.

    Holds the currently-selected articulation, per-DOF properties, the link-to-mesh
    inventory used by sphere generation, and the collision sphere editor. Exposes
    high-level operations (``select_articulation``, ``export_xrdf``,
    ``import_xrdf``, ``export_lula``, ``import_lula``) that the UI delegates to.

    Observers can subscribe via :meth:`add_articulation_changed_callback` to react
    to articulation selection changes.
    """

    def __init__(self) -> None:
        self.articulation_base_path: str | None = None
        self.articulation: Articulation | None = None
        self.num_dof: int = 0
        self.dof_names: list[str] = []
        self.upper_joint_limits: np.ndarray = np.zeros(0)
        self.lower_joint_limits: np.ndarray = np.zeros(0)

        self.joint_positions: np.ndarray = np.zeros(0)
        self.active_joints: np.ndarray = np.zeros(0, dtype=bool)
        self.acceleration_limits: np.ndarray = np.zeros(0)
        self.jerk_limits: np.ndarray = np.zeros(0)

        # Names of joints (in `dof_names`) that have `PhysxSchema.PhysxMimicJointAPI`
        # applied. cuMotion rejects mimic followers in `default_joint_positions`,
        # so XRDF/Lula exports omit them and the joint properties UI shows them
        # as read-only.
        self.mimic_joint_names: set[str] = set()

        # Link-subpath (relative to articulation base) -> list of mesh subpaths.
        self.link_to_meshes: OrderedDict[str, list[str]] = OrderedDict()

        self.collision_sphere_editor = CollisionSphereEditor()

        self._articulation_changed_callbacks: list[Callable[[], None]] = []

    # ------------------------------------------------------------------
    # Observer registration
    # ------------------------------------------------------------------
    def add_articulation_changed_callback(self, callback: Callable[[], None]) -> None:
        """Register ``callback`` to be invoked after :meth:`select_articulation`.

        Args:
            callback: Function to invoke after the selected articulation changes.
        """
        self._articulation_changed_callbacks.append(callback)

    def _notify_articulation_changed(self) -> None:
        for cb in self._articulation_changed_callbacks:
            cb()

    # ------------------------------------------------------------------
    # Selection / property refresh
    # ------------------------------------------------------------------
    def select_articulation(self, prim_path: str | None) -> None:
        """Switch the active articulation to ``prim_path``.

        Sizes per-DOF arrays to the new articulation, refreshes the link/mesh
        inventory, and notifies observers. Passing ``None`` or an empty path
        clears the selection.

        Args:
            prim_path: Path to the articulation prim to select, or None to clear
                the selection.
        """
        if not prim_path or prim_path == "None":
            self.articulation_base_path = None
            self.articulation = None
            self.num_dof = 0
            self.dof_names = []
            self.upper_joint_limits = np.zeros(0)
            self.lower_joint_limits = np.zeros(0)
            self.joint_positions = np.zeros(0)
            self.active_joints = np.zeros(0, dtype=bool)
            self.acceleration_limits = np.zeros(0)
            self.jerk_limits = np.zeros(0)
            self.mimic_joint_names = set()
            self.link_to_meshes = OrderedDict()
            self._notify_articulation_changed()
            return

        self.articulation_base_path = prim_path
        self.articulation = Articulation(prim_path)
        self.refresh_dof_properties(refresh_dof_state=True)
        self.refresh_link_meshes()
        self._notify_articulation_changed()

    def refresh_dof_properties(self, refresh_dof_state: bool = False) -> None:
        """Resize and refresh per-DOF arrays from the current articulation.

        Args:
            refresh_dof_state: If True, also re-read the current joint positions
                from the articulation and reset active-joint/limit arrays to
                defaults. Pass False during physics ticks to preserve
                user-edited values.
        """
        if self.articulation is None:
            return

        self.num_dof = self.articulation.num_dofs
        self.dof_names = list(self.articulation.dof_names)

        if refresh_dof_state:
            self.joint_positions = self.articulation.get_dof_positions().numpy()[0].astype(float)
            self.active_joints = np.zeros(self.num_dof, dtype=bool)
            self.acceleration_limits = np.full(self.num_dof, DEFAULT_ACCELERATION_LIMIT, dtype=float)
            self.jerk_limits = np.full(self.num_dof, DEFAULT_JERK_LIMIT, dtype=float)

        lower, upper = self.articulation.get_dof_limits()
        self.lower_joint_limits = lower.numpy()[0]
        self.upper_joint_limits = upper.numpy()[0]

        # Mimic-follower joints are filtered out of the XRDF / Lula exports
        # because their positions are derived from a reference joint by the
        # physics engine; cuMotion rejects independent defaults for them.
        stage = stage_utils.get_current_stage()
        self.mimic_joint_names = articulation_discovery.find_mimic_joint_names(stage, self.articulation_base_path)
        if self.mimic_joint_names and self.active_joints.size:
            for i, name in enumerate(self.dof_names):
                if name in self.mimic_joint_names:
                    self.active_joints[i] = False

    def refresh_link_meshes(self) -> None:
        """Re-discover the per-link mesh inventory under the current articulation."""
        if self.articulation is None or self.articulation_base_path is None:
            self.link_to_meshes = OrderedDict()
            return

        stage = stage_utils.get_current_stage()
        if stage is None:
            self.link_to_meshes = OrderedDict()
            return

        self.link_to_meshes = sphere_generation.find_link_meshes(
            stage, self.articulation_base_path, list(self.articulation.link_names)
        )

    # ------------------------------------------------------------------
    # Helpers exposed to UI / tests
    # ------------------------------------------------------------------
    def link_path(self, link_name: str) -> str:
        """Return the absolute USD path for ``link_name`` under the articulation.

        Args:
            link_name: Link subpath relative to the articulation base path.

        Returns:
            Absolute USD path for the link.
        """
        if self.articulation_base_path is None:
            raise RuntimeError("No articulation selected")
        return self.articulation_base_path + link_name

    def ignore_dict(self) -> dict[str, list[str]]:
        """Return the self-collision ignore-rule dict for the current articulation.

        Returns:
            Self-collision ignore rules for the selected articulation.
        """
        if self.articulation is None or self.articulation_base_path is None:
            return {}
        return articulation_discovery.get_ignore_dict(self.articulation_base_path, list(self.articulation.link_names))

    def articulation_frames(self) -> set[str]:
        """Return link names for buffer-distance reconciliation.

        Returns:
            Link names without leading slashes.
        """
        return {link_path[1:] for link_path in self.link_to_meshes}

    # ------------------------------------------------------------------
    # XRDF import / export
    # ------------------------------------------------------------------
    def export_xrdf(
        self,
        path: str,
        *,
        format_version: float,
        merge_with_existing: bool = False,
    ) -> None:
        """Serialise the current state as an XRDF file at ``path``.

        Args:
            path: Destination path.
            format_version: XRDF format version (1.0 or 2.0).
            merge_with_existing: If True, preserve passthrough fields from any
                existing valid XRDF file at ``path``.
        """
        if self.articulation is None or self.articulation_base_path is None:
            raise RuntimeError("Cannot export XRDF without a selected articulation")

        art_view = Articulation(self.articulation_base_path)
        ordered_links = list(art_view.link_names)

        inputs = xrdf_io.XrdfWriteInputs(
            path=path,
            format_version=format_version,
            articulation_base_path=self.articulation_base_path,
            dof_names=list(self.dof_names),
            active_joints_mask=self.active_joints,
            joint_positions=self.joint_positions,
            acceleration_limits=self.acceleration_limits,
            jerk_limits=self.jerk_limits,
            ordered_links=ordered_links,
            ignore_dict=articulation_discovery.get_ignore_dict(self.articulation_base_path, ordered_links),
            sphere_dict_writer=self.collision_sphere_editor.write_spheres_to_dict,
            merge_existing=path if merge_with_existing else None,
            articulation_frames=self.articulation_frames(),
            mimic_joint_names=set(self.mimic_joint_names),
        )
        xrdf_io.write_xrdf_file(inputs)

    def import_xrdf(self, path: str) -> None:
        """Load an XRDF file and update this state from it.

        Args:
            path: Path to the XRDF file.
        """
        if self.articulation is None or self.articulation_base_path is None:
            raise RuntimeError("Cannot import XRDF without a selected articulation")

        result = xrdf_io.read_xrdf_file(
            path,
            self.dof_names,
            default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
            default_jerk_limit=DEFAULT_JERK_LIMIT,
            joint_limits_lower=self.lower_joint_limits[: self.num_dof],
            joint_limits_upper=self.upper_joint_limits[: self.num_dof],
        )
        self.active_joints = result.active_joints_mask
        self.acceleration_limits = result.acceleration_limits
        self.jerk_limits = result.jerk_limits
        self.joint_positions = result.joint_positions

        self.collision_sphere_editor.load_xrdf_spheres(self.articulation_base_path, result.parsed_file)

    # ------------------------------------------------------------------
    # Lula import / export
    # ------------------------------------------------------------------
    def export_lula(self, path: str) -> None:
        """Serialise the current state as a Lula robot description YAML at ``path``.

        Args:
            path: Destination path.
        """
        if self.articulation is None or self.articulation_base_path is None:
            raise RuntimeError("Cannot export Lula description without a selected articulation")

        inputs = lula_io.LulaWriteInputs(
            path=path,
            articulation_base_path=self.articulation_base_path,
            dof_names=list(self.dof_names),
            active_joints_mask=self.active_joints,
            joint_positions=self.joint_positions,
            acceleration_limits=self.acceleration_limits,
            jerk_limits=self.jerk_limits,
            collision_sphere_editor=self.collision_sphere_editor,
            mimic_joint_names=set(self.mimic_joint_names),
        )
        lula_io.write_lula_robot_description_file(inputs)

    def import_lula(self, path: str) -> None:
        """Load a Lula robot description YAML and update this state from it.

        Args:
            path: Path to the Lula robot description YAML file.
        """
        if self.articulation is None or self.articulation_base_path is None:
            raise RuntimeError("Cannot import Lula description without a selected articulation")

        result = lula_io.read_lula_robot_description_file(
            path,
            self.dof_names,
            default_acceleration_limit=DEFAULT_ACCELERATION_LIMIT,
            default_jerk_limit=DEFAULT_JERK_LIMIT,
        )
        self.active_joints = result.active_joints_mask
        self.acceleration_limits = result.acceleration_limits
        self.jerk_limits = result.jerk_limits
        self.joint_positions = result.joint_positions

        self.collision_sphere_editor.load_spheres(self.articulation_base_path, path)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_shutdown(self) -> None:
        """Release resources held by the embedded :class:`CollisionSphereEditor`."""
        self.collision_sphere_editor.on_shutdown()
