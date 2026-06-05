# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""MJCF importer configuration utilities."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MJCFImporterConfig:
    """Configuration for MJCF import operations.

    Stores settings that control how MJCF files are converted to USD.

    Args:
        mjcf_path: Path to the MJCF (.xml) file to import.
        usd_path: Directory path where the USD file will be saved.  When left at
            its default of ``None``, :meth:`MJCFImporter.import_mjcf` will mutate
            this field in-place to ``os.path.dirname(mjcf_path)`` (the directory
            containing the source MJCF) and write all USD output there.  Pass an
            explicit value to keep import outputs out of the source tree.
        import_scene: If True, imports the MJCF simulation settings along with the model.
        merge_mesh: If True, merges meshes where possible to optimize the model.
        debug_mode: If True, enables debug mode with additional logging and visualization.
        collision_from_visuals: If True, collision geometry is generated from visual geometries.
        collision_type: Type of collision geometry to use. Options: "Convex Hull",
            "Convex Decomposition", "Bounding Sphere", "Bounding Cube".
        allow_self_collision: If True, allows the model to collide with itself.
        fix_base: Tri-state base-type toggle.
            - ``True``: adds a fixed joint from the world to the root rigid-body link and
              relocates ArticulationRootAPI to the correct ancestor prim.
            - ``False``: removes any existing world-to-root fixed joint so the robot
              becomes floating-base.
            - ``None`` (default): leaves the source asset's base authoring untouched.
        link_density: Default density (kg/m^3) applied to rigid body links that have no
            explicit mass.  ``None`` means no density override.
        override_gain_type: MuJoCo actuator gain type (e.g. ``"fixed"``).  ``None`` leaves
            existing value.
        override_bias_type: MuJoCo actuator bias type (e.g. ``"affine"``).  ``None`` leaves
            existing value.
        override_gain_prm: MuJoCo actuator gain parameter array (10 floats).  ``None`` leaves
            existing value.  Position control example: ``[kp, 0, 0, 0, 0, 0, 0, 0, 0, 0]``.
        override_bias_prm: MuJoCo actuator bias parameter array (10 floats).  ``None`` leaves
            existing value.  Position control example: ``[0, -kp, -kd, 0, 0, 0, 0, 0, 0, 0]``.
        run_asset_transformer: If True, runs the asset transformer profile after conversion
            to restructure the USD output.
        run_multi_physics_conversion: If True, runs MuJoCo-to-PhysX physics conversion on
            the imported stage.

    Example:

    .. code-block:: python

        >>> from isaacsim.asset.importer.mjcf import MJCFImporterConfig

        >>> config = MJCFImporterConfig(
        ...     mjcf_path="/tmp/robot.xml",
        ...     usd_path="/tmp/output",
        ...     merge_mesh=True
        ... )
        >>> config.mjcf_path
        '/tmp/robot.xml'
    """

    mjcf_path: str | None = None
    usd_path: str | None = None
    import_scene: bool = True
    merge_mesh: bool = False
    debug_mode: bool = False
    collision_from_visuals: bool = False
    collision_type: str = "Convex Hull"
    allow_self_collision: bool = False
    robot_type: str = "Default"
    fix_base: bool | None = None
    link_density: float | None = None
    joint_drive_type: str | dict[str, str] | None = None
    joint_target_type: str | dict[str, str] | None = None
    override_gain_type: str | None = None
    override_bias_type: str | None = None
    override_gain_prm: list[float] | None = None
    override_bias_prm: list[float] | None = None
    run_asset_transformer: bool = True
    run_multi_physics_conversion: bool = True
