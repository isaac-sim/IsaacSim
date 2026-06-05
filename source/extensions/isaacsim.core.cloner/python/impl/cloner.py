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

"""Cloner module."""

from __future__ import annotations

__all__ = ["Cloner"]

import carb
import carb.settings
import numpy as np
import omni.usd
import usdrt
from isaacsim.core.cloner.bindings._isaac_cloner import _fabric_clone
from isaacsim.core.simulation_manager import SimulationManager
from omni.physx import get_physx_replicator_interface
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdUtils, Vt


class Cloner:
    """This class provides a set of simple APIs to make duplication of objects simple.

    Objects can be cloned using this class to create copies of the same object,
    placed at user-specified locations in the scene.

    Note that the cloning process is performed in a for-loop, so performance should
    be expected to follow linear scaling with an increase of clones.

    Args:
        stage: USD stage where source prim and clones are added to.
            Defaults to the current stage from the USD context.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.cloner import Cloner
        >>>
        >>> cloner = Cloner()
        >>> cloner.define_base_env("/World/envs")
        >>> prim_paths = cloner.generate_paths("/World/envs/env", 4)
        >>> cloner.clone(
        ...     source_prim_path="/World/envs/env_0",
        ...     prim_paths=prim_paths,
        ... )

    .. code-block:: python

        >>> from isaacsim.core.cloner import Cloner
        >>> import omni.usd
        >>>
        >>> # Use current stage
        >>> cloner = Cloner()
        >>>
        >>> # Or provide a specific stage
        >>> stage = omni.usd.get_context().get_stage()
        >>> cloner = Cloner(stage=stage)
    """

    def __init__(self, stage: Usd.Stage = None) -> None:
        self._base_env_path = None
        self._root_path = None
        self._physx_ui_notice_enabled = False
        self._fabric_usd_notice_enabled = False
        self._stage = stage
        if stage is None:
            self._stage = omni.usd.get_context().get_stage()

    def define_base_env(self, base_env_path: str) -> None:
        """Create a USD Scope at base_env_path.

        This is designed to be the parent that holds all clones.

        Args:
            base_env_path: Path to create the USD Scope at.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.define_base_env("/World/envs")
        """
        UsdGeom.Scope.Define(self._stage, base_env_path)
        self._base_env_path = base_env_path

    def generate_paths(self, root_path: str, num_paths: int) -> list[str]:
        """Generate a list of paths under the root path specified.

        Args:
            root_path: Base path where new paths will be created under.
            num_paths: Number of paths to generate.

        Returns:
            A list of paths in the format ``{root_path}_{i}`` where i ranges from 0 to num_paths - 1.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> paths = cloner.generate_paths("/World/envs/env", 4)
            >>> paths
            ['/World/envs/env_0', '/World/envs/env_1', '/World/envs/env_2', '/World/envs/env_3']
        """
        self._root_path = root_path + "_"
        return [f"{root_path}_{i}" for i in range(num_paths)]

    def replicate_physics(
        self,
        source_prim_path: str,
        prim_paths: list,
        base_env_path: str,
        root_path: str,
        enable_env_ids: bool = False,
        clone_in_fabric: bool = False,
    ) -> None:
        """Replicate physics properties directly in omni.physics.

        This avoids performance bottlenecks when parsing physics by using the
        PhysX replicator interface to replicate physics properties directly.

        Args:
            source_prim_path: Path of the source object.
            prim_paths: List of destination paths.
            base_env_path: Path to namespace for all environments.
            root_path: Prefix path for each environment.
            enable_env_ids: Whether to use envIDs functionality in physics to enable
                co-location of clones. Clones will be filtered automatically.
            clone_in_fabric: Whether to perform cloning in Fabric.

        Raises:
            ValueError: If base_env_path is None and define_base_env() was not called.
            ValueError: If root_path is None and generate_paths() was not called.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.define_base_env("/World/envs")
            >>> prim_paths = cloner.generate_paths("/World/envs/env", 4)
            >>> cloner.replicate_physics(
            ...     source_prim_path="/World/envs/env_0",
            ...     prim_paths=prim_paths,
            ...     base_env_path="/World/envs",
            ...     root_path="/World/envs/env_",
            ... )
        """
        if base_env_path is None and self._base_env_path is None:
            raise ValueError("base_env_path needs to be specified!")
        if root_path is None and self._root_path is None:
            raise ValueError("root_path needs to be specified!")

        # resolve number of replications being made
        replicate_first = source_prim_path not in prim_paths
        # resolve inputs
        clone_base_path = self._root_path if root_path is None else root_path
        clone_root = self._base_env_path if base_env_path is None else base_env_path
        num_replications = len(prim_paths) if replicate_first else len(prim_paths) - 1

        stageId = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()

        # if enable_env_ids, set the envIdInBoundsBitCount to 4
        if enable_env_ids:
            usdrt_stage = usdrt.Usd.Stage.Attach(stageId)
            for prim_path in usdrt_stage.GetPrimsWithTypeName("UsdPhysicsScene"):
                prim = self._stage.GetPrimAtPath(str(prim_path))
                attr = prim.CreateAttribute("physxScene:envIdInBoundsBitCount", Sdf.ValueTypeNames.Int)
                attr.Set(4)

        def replicationAttachFn(stageId: int) -> list[str]:  # noqa: N802
            exclude_paths = [clone_root]
            return exclude_paths

        def replicationAttachEndFn(stageId: int) -> None:  # noqa: N802
            get_physx_replicator_interface().replicate(
                stageId, source_prim_path, num_replications, enable_env_ids, clone_in_fabric
            )

        def hierarchyRenameFn(replicatePath: str, index: int) -> str:  # noqa: N802
            if replicate_first:
                stringPath = clone_base_path + str(index)
            else:
                stringPath = clone_base_path + str(index + 1)
            return stringPath

        get_physx_replicator_interface().register_replicator(
            stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn
        )

    def disable_change_listener(self) -> None:
        """Disable USD notice handlers to improve cloning performance.

        This method temporarily disables various USD notice handlers including
        the PhysX UI notice handler, Fabric USD notice handler, and SimulationManager
        notice handler. This prevents expensive recomputation during batch cloning
        operations.

        Note:
            Call :meth:`enable_change_listener` after cloning is complete to restore
            the notice handlers.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.disable_change_listener()
            >>> # Perform cloning operations...
            >>> cloner.enable_change_listener()
        """
        # first try to disable omni physx UI notice handler
        try:
            from omni.physxui import get_physxui_interface

            self._physx_ui_notice_enabled = get_physxui_interface().is_usd_notice_handler_enabled()
            if self._physx_ui_notice_enabled:
                get_physxui_interface().block_usd_notice_handler(True)
        except Exception:
            pass

        # second disable Fabric USD notice handler
        stage_id = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
        self._fabric_usd_notice_enabled = SimulationManager.is_fabric_usd_notice_handler_enabled(stage_id)
        if self._fabric_usd_notice_enabled:
            SimulationManager.enable_fabric_usd_notice_handler(stage_id, False)

        # third disable SimulationManager notice handler
        SimulationManager.enable_usd_notice_handler(False)

    def enable_change_listener(self) -> None:
        """Re-enable USD notice handlers after cloning operations.

        This method restores the USD notice handlers that were disabled by
        :meth:`disable_change_listener`. It re-enables the PhysX UI notice handler,
        Fabric USD notice handler, and SimulationManager notice handler if they
        were previously enabled.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.disable_change_listener()
            >>> # Perform cloning operations...
            >>> cloner.enable_change_listener()
        """
        try:
            from omni.physxui import get_physxui_interface

            if self._physx_ui_notice_enabled:
                get_physxui_interface().block_usd_notice_handler(False)
        except Exception:
            pass

        if self._fabric_usd_notice_enabled:
            stage_id = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
            SimulationManager.enable_fabric_usd_notice_handler(stage_id, True)

        SimulationManager.enable_usd_notice_handler(True)

    def clone(
        self,
        source_prim_path: str,
        prim_paths: list[str],
        positions: np.ndarray | "torch.Tensor" = None,
        orientations: np.ndarray | "torch.Tensor" = None,
        replicate_physics: bool = False,
        base_env_path: str = None,
        root_path: str = None,
        copy_from_source: bool = False,
        unregister_physics_replication: bool = False,
        enable_env_ids: bool = False,
        clone_in_fabric: bool = False,
    ) -> None:
        """Clone a source prim at user-specified destination paths.

        Clones will be placed at user-specified positions and orientations.

        Args:
            source_prim_path: Path of the source object.
            prim_paths: List of destination paths.
            positions: An array containing target positions of clones. Dimension must
                equal the length of prim_paths. Defaults to None, in which case clones
                will be placed at (0, 0, 0).
            orientations: An array containing target orientations of clones as quaternions
                (w, x, y, z). Dimension must equal the length of prim_paths. Defaults to None,
                in which case clones will have identity orientation (1, 0, 0, 0).
            replicate_physics: Uses omni.physics replication. This will replicate physics
                properties directly for paths beginning with root_path and skip physics
                parsing for anything under the base_env_path.
            base_env_path: Path to namespace for all environments. Required if
                replicate_physics=True and define_base_env() was not called.
            root_path: Prefix path for each environment. Required if replicate_physics=True
                and generate_paths() was not called.
            copy_from_source: Setting this to False will inherit all clones from the source
                prim; any changes made to the source prim will be reflected in the clones.
                Setting this to True will make copies of the source prim when creating new
                clones; changes to the source prim will not be reflected in clones.
                Defaults to False. Note that setting this to True will take longer to execute.
            unregister_physics_replication: Setting this to True will unregister the physics
                replicator on the current stage.
            enable_env_ids: Setting this enables co-location of clones in physics with
                automatic filtering of collisions between clones.
            clone_in_fabric: Whether to perform cloning operations in Fabric for improved
                performance.

        Raises:
            ValueError: If the dimension of positions does not match the number of prim_paths.
            ValueError: If the dimension of orientations does not match the number of prim_paths.
            Exception: If the source prim does not exist.

        Example:

        .. code-block:: python

            >>> import numpy as np
            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.define_base_env("/World/envs")
            >>> prim_paths = cloner.generate_paths("/World/envs/env", 4)
            >>>
            >>> # Clone with positions
            >>> positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
            >>> cloner.clone(
            ...     source_prim_path="/World/envs/env_0",
            ...     prim_paths=prim_paths,
            ...     positions=positions,
            ... )
        """
        if not isinstance(source_prim_path, str):
            raise TypeError(f"source_prim_path must be a str, got {type(source_prim_path).__name__}")
        if not Sdf.Path.IsValidPathString(source_prim_path):
            raise ValueError(f"source_prim_path {source_prim_path!r} is not a valid SdfPath")

        self.disable_change_listener()
        try:
            # check if inputs are valid
            if positions is not None:
                if len(positions) != len(prim_paths):
                    raise ValueError("Dimension mismatch between positions and prim_paths!")
                # convert to numpy array
                # - convert from torch (without explicit importing it)
                try:
                    positions = positions.detach().cpu().numpy()
                except Exception:
                    pass
                # - convert from other types
                if not isinstance(positions, np.ndarray):
                    positions = np.asarray(positions)
                # convert to pxr gf
                positions = Vt.Vec3fArray.FromNumpy(positions)
            if orientations is not None:
                if len(orientations) != len(prim_paths):
                    raise ValueError("Dimension mismatch between orientations and prim_paths!")
                # convert to numpy array
                # - convert from torch (without explicit importing it)
                try:
                    orientations = orientations.detach().cpu().numpy()
                except Exception:
                    pass
                # - convert from other types
                if not isinstance(orientations, np.ndarray):
                    orientations = np.asarray(orientations)
                # convert to pxr gf -- wxyz to xyzw
                orientations = np.roll(orientations, -1, -1)
                orientations = Vt.QuatdArray.FromNumpy(orientations)

            # make sure source prim has valid xform properties
            source_prim = self._stage.GetPrimAtPath(source_prim_path)
            if not source_prim:
                raise Exception("Source prim does not exist")
            properties = source_prim.GetPropertyNames()
            xformable = UsdGeom.Xformable(source_prim)
            # get current position and orientation
            T_p_w = xformable.ComputeParentToWorldTransform(Usd.TimeCode.Default())
            T_l_w = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            T_l_p = Gf.Transform()
            T_l_p.SetMatrix(Gf.Matrix4d(np.matmul(T_l_w, np.linalg.inv(T_p_w)).tolist()))
            current_translation = T_l_p.GetTranslation()
            current_orientation = T_l_p.GetRotation().GetQuat()
            # get current scale
            current_scale = Gf.Vec3d(1, 1, 1)
            if "xformOp:scale" in properties:
                current_scale = Gf.Vec3d(source_prim.GetAttribute("xformOp:scale").Get())

            # remove all xform ops except for translate, orient, and scale
            properties_to_remove = [
                "xformOp:rotateX",
                "xformOp:rotateXZY",
                "xformOp:rotateY",
                "xformOp:rotateYXZ",
                "xformOp:rotateYZX",
                "xformOp:rotateZ",
                "xformOp:rotateZYX",
                "xformOp:rotateZXY",
                "xformOp:rotateXYZ",
                "xformOp:transform",
                "xformOp:scale",
            ]
            xformable.ClearXformOpOrder()
            for prop_name in properties:
                if prop_name in properties_to_remove:
                    source_prim.RemoveProperty(prop_name)

            properties = source_prim.GetPropertyNames()
            # add xform ops if they don't exist
            if "xformOp:translate" not in properties:
                xform_op_translate = xformable.AddXformOp(
                    UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
                )
            else:
                xform_op_translate = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:translate"))
            xform_op_translate.Set(current_translation)

            if "xformOp:orient" not in properties:
                xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
            else:
                xform_op_rot = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:orient"))
            if xform_op_rot.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                current_orientation = Gf.Quatf(current_orientation)
            else:
                current_orientation = Gf.Quatd(current_orientation)
            xform_op_rot.Set(current_orientation)

            if "xformOp:scale" not in properties:
                xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
            else:
                xform_op_scale = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:scale"))
            xform_op_scale.Set(current_scale)
            # set xform op order
            xformable.SetXformOpOrder([xform_op_translate, xform_op_rot, xform_op_scale])

            # set source actor transform
            if source_prim_path in prim_paths:
                idx = prim_paths.index(source_prim_path)
                prim = UsdGeom.Xform(self._stage.GetPrimAtPath(source_prim_path))

                if positions is not None:
                    translation = positions[idx]
                else:
                    translation = current_translation

                if orientations is not None:
                    orientation = orientations[idx]
                else:
                    orientation = current_orientation

                # overwrite translation and orientation to values specified
                prim.GetPrim().GetAttribute("xformOp:translate").Set(translation)
                prim.GetPrim().GetAttribute("xformOp:orient").Set(orientation)

            has_clones = False

            if clone_in_fabric:
                stageId = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
                ret_val = _fabric_clone(stageId, source_prim_path, prim_paths)
                if ret_val:
                    usdrt_stage = usdrt.Usd.Stage.Attach(stageId)
                    for i, prim_path in enumerate(prim_paths):
                        if prim_path != source_prim_path:
                            has_clones = True
                            # setup transformations for cloned environments
                            prim = usdrt_stage.GetPrimAtPath(prim_path)
                            attr = prim.GetAttribute("omni:fabric:localMatrix")

                            local_matrix = attr.Get()

                            transform = usdrt.Gf.Transform(local_matrix)

                            if positions is not None:
                                translation = positions[i]  # use specified translation
                            else:
                                translation = current_translation  # use the same translation as source

                            if orientations is not None:
                                orientation = orientations[i]  # use specified orientation
                            else:
                                orientation = current_orientation  # use the same orientation as source

                            transform.SetTranslation(usdrt.Gf.Vec3d(translation))
                            gf_quat = Gf.Quatd(orientation)

                            transform.SetRotation(
                                usdrt.Gf.Rotation(
                                    usdrt.Gf.Quatd(gf_quat.GetReal(), usdrt.Gf.Vec3d(gf_quat.GetImaginary()))
                                )
                            )
                            attr.Set(transform.GetMatrix())

                    # update fabric hierarchy
                    fabric_id = usdrt_stage.GetFabricId()
                    hier = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(fabric_id, stageId)
                    hier.update_world_xforms()
                else:
                    carb.log_error("Failed to clone in Fabric")
            else:
                with Sdf.ChangeBlock():
                    for i, prim_path in enumerate(prim_paths):
                        if prim_path != source_prim_path:
                            has_clones = True

                            env_spec = Sdf.CreatePrimInLayer(self._stage.GetRootLayer(), prim_path)
                            stack = UsdGeom.Xform(self._stage.GetPrimAtPath(source_prim_path)).GetPrim().GetPrimStack()

                            if copy_from_source:
                                Sdf.CopySpec(
                                    env_spec.layer, Sdf.Path(source_prim_path), env_spec.layer, Sdf.Path(prim_path)
                                )
                            else:
                                env_spec.inheritPathList.Prepend(source_prim_path)

                            if positions is not None:
                                translation = positions[i]  # use specified translation
                            else:
                                translation = current_translation  # use the same translation as source

                            if orientations is not None:
                                orientation = orientations[i]  # use specified orientation
                            else:
                                orientation = current_orientation  # use the same orientation as source

                            translate_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:translate")
                            if translate_spec is None:
                                translate_spec = Sdf.AttributeSpec(
                                    env_spec, "xformOp:translate", Sdf.ValueTypeNames.Double3
                                )
                            translate_spec.default = translation

                            orient_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:orient")
                            default_precision = carb.settings.get_settings().get_as_string(
                                "app/primCreation/DefaultXformOpPrecision"
                            )
                            if orient_spec is None:
                                if len(default_precision) > 0 and default_precision == "Float":
                                    orient_spec = Sdf.AttributeSpec(
                                        env_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatf
                                    )
                                    orient_spec.default = Gf.Quatf(orientation)
                                else:
                                    orient_spec = Sdf.AttributeSpec(
                                        env_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd
                                    )
                                    orient_spec.default = Gf.Quatd(orientation)
                            elif orient_spec.default is not None and type(orient_spec.default) == Gf.Quatf:
                                orient_spec.default = Gf.Quatf(orientation)
                            else:
                                orient_spec.default = Gf.Quatd(orientation)

                            scale_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:scale")
                            if scale_spec is None:
                                scale_spec = Sdf.AttributeSpec(env_spec, "xformOp:scale", Sdf.ValueTypeNames.Double3)
                            scale_spec.default = current_scale

                            op_order_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOpOrder")
                            if op_order_spec is None:
                                op_order_spec = Sdf.AttributeSpec(
                                    env_spec, UsdGeom.Tokens.xformOpOrder, Sdf.ValueTypeNames.TokenArray
                                )
                            op_order_spec.default = Vt.TokenArray(
                                ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
                            )

            if replicate_physics and has_clones:
                self.replicate_physics(
                    source_prim_path, prim_paths, base_env_path, root_path, enable_env_ids, clone_in_fabric
                )
            elif unregister_physics_replication:
                get_physx_replicator_interface().unregister_replicator(
                    UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
                )
        finally:
            self.enable_change_listener()

    def filter_collisions(
        self,
        physicsscene_path: str,
        collision_root_path: str,
        prim_paths: list[str],
        global_paths: list[str] | None = None,
    ) -> None:
        """Filter collisions between clones.

        Clones will not collide with each other, but can collide with objects
        specified in global_paths. This method uses inverted collision group
        filtering for more efficient collision filtering across environments.

        Args:
            physicsscene_path: Path to PhysicsScene object in stage.
            collision_root_path: Path to place collision groups under.
            prim_paths: Paths of objects to filter out collision.
            global_paths: Paths of objects to generate collision with all clones
                (e.g. ground plane).

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import Cloner
            >>>
            >>> cloner = Cloner()
            >>> cloner.define_base_env("/World/envs")
            >>> prim_paths = cloner.generate_paths("/World/envs/env", 4)
            >>> cloner.clone(
            ...     source_prim_path="/World/envs/env_0",
            ...     prim_paths=prim_paths,
            ... )
            >>> cloner.filter_collisions(
            ...     physicsscene_path="/World/PhysicsScene",
            ...     collision_root_path="/World/collisions",
            ...     prim_paths=prim_paths,
            ...     global_paths=["/World/ground_plane"],
            ... )
        """
        if global_paths is None:
            global_paths = []

        physx_scene = PhysxSchema.PhysxSceneAPI(self._stage.GetPrimAtPath(physicsscene_path))

        # We invert the collision group filters for more efficient collision filtering across environments
        physx_scene.CreateInvertCollisionGroupFilterAttr().Set(True)

        # Make sure we create the collision_scope in the RootLayer since the edit target may be a live layer in the case of Live Sync.
        with Usd.EditContext(self._stage, Usd.EditTarget(self._stage.GetRootLayer())):
            collision_scope = UsdGeom.Scope.Define(self._stage, collision_root_path)

        with Sdf.ChangeBlock():
            if len(global_paths) > 0:
                global_collision_group_path = collision_root_path + "/global_group"
                # add collision group prim
                global_collision_group = Sdf.PrimSpec(
                    self._stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    "global_group",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                global_collision_group.SetInfo(
                    Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"})
                )

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    global_collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                global_includes_rel = Sdf.RelationshipSpec(
                    global_collision_group, "collection:colliders:includes", False
                )
                for global_path in global_paths:
                    global_includes_rel.targetPathList.Append(global_path)

                # filteredGroups rel
                global_filtered_groups = Sdf.RelationshipSpec(global_collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                global_filtered_groups.targetPathList.Append(global_collision_group_path)

            # set collision groups and filters
            for i, prim_path in enumerate(prim_paths):
                collision_group_path = collision_root_path + f"/group{i}"
                # add collision group prim
                collision_group = Sdf.PrimSpec(
                    self._stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    f"group{i}",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                collision_group.SetInfo(Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"}))

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                includes_rel = Sdf.RelationshipSpec(collision_group, "collection:colliders:includes", False)
                includes_rel.targetPathList.Append(prim_path)

                # filteredGroups rel
                filtered_groups = Sdf.RelationshipSpec(collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                filtered_groups.targetPathList.Append(collision_group_path)
                if len(global_paths) > 0:
                    filtered_groups.targetPathList.Append(global_collision_group_path)
                    global_filtered_groups.targetPathList.Append(collision_group_path)
