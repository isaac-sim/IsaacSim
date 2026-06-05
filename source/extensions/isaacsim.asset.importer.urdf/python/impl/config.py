# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Configuration for the URDF importer."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class URDFImporterConfig:
    """Configuration for URDF import operations.

    Stores settings that control how URDF files are converted to USD.

    Args:
        urdf_path: Path to the URDF (.urdf) file to import.
        usd_path: Directory path where the USD file will be saved.  When left at
            its default of ``None``, :meth:`URDFImporter.import_urdf` will mutate
            this field in-place to ``os.path.dirname(urdf_path)`` (the directory
            containing the source URDF) and write all USD output there.  Pass an
            explicit value to keep import outputs out of the source tree.
        merge_fixed_joints: If True, merges fixed joints where possible to optimize the model.
        merge_mesh: If True, merges meshes where possible to optimize the model.
        debug_mode: If True, enables debug mode with additional logging and visualization.
        collision_from_visuals: If True, collision geometry is generated from visual geometries.
        collision_type: Type of collision geometry to use. Options: "Convex Hull",
            "Convex Decomposition", "Bounding Sphere", "Bounding Cube".
        allow_self_collision: If True, allows the model to collide with itself.
        ros_package_paths: List of ROS package name/path mappings for resolving package:// URLs.
        fix_base: Tri-state base-type toggle.
            - ``True``: adds a fixed joint from the world to the root rigid-body link and
              relocates ArticulationRootAPI to the correct ancestor prim.
            - ``False``: removes any existing world-to-root fixed joint so the robot
              becomes floating-base.
            - ``None`` (default): leaves the source asset's base authoring untouched.
        link_density: Default density (kg/m^3) applied to rigid body links that have no
            explicit mass.  ``None`` means no density override.
        joint_drive_type: Joint drive type (``"force"`` or ``"acceleration"``), or a dict
            mapping joint-name regex patterns to per-joint values.  ``None`` leaves drives
            unchanged.
        joint_target_type: Joint target type (``"none"``, ``"position"``, or ``"velocity"``),
            or a dict of patterns.  ``None`` leaves targets unchanged.
        override_joint_stiffness: Joint stiffness in Nm/rad (revolute) or N/m (prismatic), or
            a dict of patterns.  ``None`` leaves stiffness unchanged.
        override_joint_damping: Joint damping in Nm*s/rad (revolute) or N*s/m (prismatic), or
            a dict of patterns.  ``None`` leaves damping unchanged.
        run_asset_transformer: If True, runs the asset transformer profile after conversion
            to restructure the USD output.
        run_multi_physics_conversion: If True, runs URDF-to-PhysX joint attribute conversion
            on the imported stage.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.urdf import URDFImporterConfig

        >>> config = URDFImporterConfig(
        ...     urdf_path="/tmp/robot.urdf",
        ...     usd_path="/tmp/output",
        ...     merge_mesh=True
        ... )
        >>> config.urdf_path
        '/tmp/robot.urdf'
    """

    urdf_path: str | None = None
    usd_path: str | None = None
    merge_fixed_joints: bool = False
    merge_mesh: bool = False
    debug_mode: bool = False
    collision_from_visuals: bool = False
    collision_type: str = "Convex Hull"
    allow_self_collision: bool = False
    ros_package_paths: list[dict[str, str]] = field(default_factory=list)
    robot_type: str = "Default"
    fix_base: bool | None = None
    link_density: float | None = None
    joint_drive_type: str | dict[str, str] | None = None
    joint_target_type: str | dict[str, str] | None = None
    override_joint_stiffness: float | dict[str, float] | None = None
    override_joint_damping: float | dict[str, float] | None = None
    run_asset_transformer: bool = True
    run_multi_physics_conversion: bool = True
