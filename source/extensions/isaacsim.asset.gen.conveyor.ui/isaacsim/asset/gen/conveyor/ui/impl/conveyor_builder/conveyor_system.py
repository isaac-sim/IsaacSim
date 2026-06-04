"""Conveyor system module for building and managing conveyor track layouts."""

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

from __future__ import annotations

from math import asin, degrees
from typing import Any

import omni
from isaacsim.asset.gen.conveyor import create_conveyor_belt
from pxr import Gf, Usd, UsdGeom
from pxr.Usd import Stage

from ..preferences import ConveyorBuilderPreferences
from .conveyor_track import Angle, ConveyorTrack, Curvature, Ramp, Style, Type


def set_pose_from_transform(prim: Usd.Prim, pose: Gf.Matrix4d, scale: Gf.Vec3d = Gf.Vec3d(1, 1, 1)) -> None:
    """Set the pose of a prim from a transform matrix.

    Args:
        prim: USD prim whose transform will be replaced.
        pose: World-space transform matrix (translation and rotation used; scale applied separately).
        scale: Scale vector applied as a dedicated scale xform op.
    """
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op_s = xform.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
    scale_mat = Gf.Matrix4d().SetScale(scale)
    t = Gf.Matrix4d(pose)
    # t = Gf.Matrix4d(scale_mat*pose)
    r = t.ExtractRotationQuat().GetNormalized()
    pos_vec = t.ExtractTranslation()
    xform_op_t.Set(pos_vec)
    xform_op_r.Set(r)
    xform_op_s.Set(scale)


def remove_scale_from_matrix(matrix: Gf.Matrix4d) -> Gf.Matrix4d:
    """Remove scale component from a 4x4 transformation matrix.

    Args:
        matrix: Input 4x4 matrix whose upper 3x3 may include non-uniform scale.

    Returns:
        A copy of ``matrix`` with the upper 3x3 normalized to unit-length column
        vectors; translation row preserved.
    """
    out = Gf.Matrix4d()
    x = Gf.Vec3d(0)
    y = Gf.Vec3d(0)
    z = Gf.Vec3d(0)
    for i in range(3):
        x[i] = matrix[0][i]
        y[i] = matrix[1][i]
        z[i] = matrix[2][i]
    l_x = x.GetLength()
    l_y = y.GetLength()
    l_z = z.GetLength()
    # print(l_x, l_y, l_z)
    for i in range(3):
        out[0][i] = matrix[0][i] / l_x
        out[1][i] = matrix[1][i] / l_y
        out[2][i] = matrix[2][i] / l_z
    for i in range(4):
        out[i][3] = matrix[i][3]
    out[3] = matrix[3]
    return out


class ConveyorFilter:
    """Filter criteria for selecting conveyor tracks.

    Args:
        styles: List of conveyor styles to filter by.
        angles: List of angle configurations to filter by.
        curvatures: List of curvature values to filter by.
        ramps: List of ramp configurations to filter by.
        types: List of track types to filter by.
    """

    def __init__(
        self,
        styles: list[Style] | None = None,
        angles: list[Angle] | None = None,
        curvatures: list[Curvature] | None = None,
        ramps: list[Ramp] | None = None,
        types: list[Type] | None = None,
    ) -> None:
        if styles is None:
            styles = []
        if angles is None:
            angles = []
        if curvatures is None:
            curvatures = []
        if ramps is None:
            ramps = []
        if types is None:
            types = []
        self.style = styles
        self.angle = angles
        self.curvature = curvatures
        self.ramp = ramps
        self.type = types

    def clear_filter(self) -> None:
        """Reset all filter criteria to empty lists."""
        self.style = []
        self.angle = []
        self.curvature = []
        self.ramp = []
        self.type = []


class ConveyorSelector:
    """Manage available conveyor track assets and provide filtered selection.

    Args:
        config_file: Configuration dictionary containing asset definitions.
        **kwargs: Additional keyword arguments including `thumb_loaded_callback`.
    """

    def __init__(self, config_file: Any, **kwargs: Any) -> None:
        preferences = ConveyorBuilderPreferences()
        # config_file = preferences.config_file
        self.asset_path = preferences.assets_location
        self.tracks = {}
        for track in config_file["assets"]:
            base_usd = f"{track}.usd"
            self.tracks[base_usd] = ConveyorTrack(
                base_usd=f"{self.asset_path}{base_usd}",
                **config_file["assets"][track],
                thumb_loaded_callback=self.on_thumb_loaded,
            )
        self._thumb_loaded_callback = kwargs.get("thumb_loaded_callback", None)

    def on_thumb_loaded(self, track: ConveyorTrack) -> None:
        """Handle thumbnail loaded event for a track.

        Args:
            track: Track whose thumbnail finished loading.
        """
        if self._thumb_loaded_callback:
            self._thumb_loaded_callback(track)

    def list_tracks(self, config: ConveyorFilter) -> list[str]:
        """Return a list of track names matching the given filter criteria.

        Args:
            config: Filter specifying allowed styles, angles, curvatures, ramps, and types.

        Returns:
            Asset base filenames (for example ``straight.usd``) that satisfy the filter.
        """
        v = list(self.tracks.values())
        tracks = [
            a
            for a in self.tracks
            if (not config.style or self.tracks[a].style in config.style)
            and (not config.angle or self.tracks[a].angle in config.angle)
            and (not config.curvature or self.tracks[a].curvature in config.curvature)
            and (not config.ramp or self.tracks[a].ramp in config.ramp)
            and (not config.type or self.tracks[a].type in config.type)
        ]

        return tracks


class ConveyorBuilder:
    """Build and manage a system of connected conveyor tracks on a USD stage.

    Args:
        stage: The USD stage to build conveyor tracks on.
        conveyor_selector: Selector providing available conveyor track assets.
    """

    def __init__(self, stage: Any, conveyor_selector: Any = None) -> None:
        self.stage = stage
        if conveyor_selector:
            self._conveyor_selector = conveyor_selector
        self._tracks = {}  # usd path, ConveyorTrack
        self._anchor_connections = {}  # usd path, dict(anchor ID,usd_path)
        self._parent_anchor = {}  # usd path, anchor ID

    def clear_system(self, stage: Stage) -> None:
        """Reset the conveyor system and reinitialize with the given stage.

        Args:
            stage: USD stage the builder should attach to after reset.
        """
        self.__init__(stage)

    def get_available_connections(self, track_path: str) -> list:
        """Get unconnected anchor points for the specified track.

        Args:
            track_path: USD path of an instantiated conveyor track prim.

        Returns:
            Anchor identifiers on ``track_path`` that are not yet wired in
            ``self._anchor_connections``, or ``None`` if ``track_path`` is unknown.
        """
        if track_path in self._tracks:
            available_connections = [
                a for a in self._tracks[track_path].get_anchors() if a not in self._anchor_connections[track_path]
            ]
            return available_connections

    def is_track(self, prim_path: str) -> bool:
        """Check if the prim at the given path is a conveyor track asset.

        Args:
            prim_path: Path of the prim to inspect.

        Returns:
            True if the prim references or payloads a conveyor asset from this selector's path.
        """
        sel = self.stage.GetPrimAtPath(prim_path)
        if sel.GetReferences():
            refs = []
            refs += omni.usd.get_composed_references_from_prim(sel, False)
            if sel.GetPayloads():
                refs += omni.usd.get_composed_payloads_from_prim(sel, False)
            for a in refs:
                if self._conveyor_selector.asset_path in a[0].assetPath:
                    return True
        return False

    def add_track(
        self,
        track: ConveyorTrack,
        track_anchor: str = "",
        x_direction: int = 1,
        y_direction: int = 1,
        parent: str | None = None,
        parent_anchor: str = "",
    ) -> str:
        """Add a new conveyor track to the system and connect it to a parent track.

        Args:
            track: Conveyor track definition and asset to instance.
            track_anchor: Anchor on the new track used when connecting to ``parent``.
            x_direction: Horizontal scale sign for placement (left/right flip).
            y_direction: Depth scale sign for placement (forward/back flip).
            parent: USD path of the parent prim or track; ``None`` uses default prim context.
            parent_anchor: Anchor on ``parent`` to connect to ``track_anchor``.

        Returns:
            USD path string of the newly created track prim.
        """
        # print("adding", track_anchor, parent_anchor)

        next_pose = self.get_next_pose(track, track_anchor, x_direction, y_direction, parent, parent_anchor)
        parent_pose = Gf.Matrix4d()
        if parent in self._tracks:
            world_prim = self.stage.GetPrimAtPath(parent).GetParent()
            _prim_path = omni.usd.get_stage_next_free_path(
                self.stage, str(world_prim.GetPath()) + "/ConveyorTrack", False
            )
            parent_pose = omni.usd.get_world_transform_matrix(world_prim)
        elif self.is_track(parent):
            parent_path = self.stage.GetPrimAtPath(parent).GetParent().GetPath()
            _prim_path = omni.usd.get_stage_next_free_path(self.stage, str(parent_path) + "/ConveyorTrack", False)
        else:
            parent_path = ""
            if parent and parent != "/" and self.stage.GetPrimAtPath(parent):
                parent_path = parent
            else:
                parent_path = self.stage.GetDefaultPrim().GetPath()
            _prim_path = omni.usd.get_stage_next_free_path(self.stage, str(parent_path) + "/ConveyorTrack", False)
        prim = self.stage.DefinePrim(_prim_path, "Xform")
        prim.GetReferences().AddReference(track.base_usd)
        part_stage = Stage.Open(track.base_usd)
        scale = Gf.Vec3d(x_direction, y_direction, 1)
        a = UsdGeom.GetStageMetersPerUnit(self.stage)
        b = UsdGeom.GetStageMetersPerUnit(part_stage)
        scale_unit = b / a
        # pose = Gf.Matrix4d().SetScale(scale) * next_pose
        set_pose_from_transform(prim, parent_pose.GetInverse() * next_pose, scale * scale_unit)

        new_track = str(prim.GetPath())
        self._tracks[new_track] = track
        self._anchor_connections[new_track] = {}
        self._parent_anchor[new_track] = track_anchor
        if parent in self._tracks:
            self._anchor_connections[new_track][track_anchor] = parent
            self._anchor_connections[parent][parent_anchor] = new_track
        for node in track.conveyor_nodes:
            conveyor_prim = create_conveyor_belt(self.stage, self.stage.GetPrimAtPath(prim.GetPath().AppendChild(node)))
            if conveyor_prim:
                conveyor_prim.GetAttribute("inputs:animateDirection").Set(
                    Gf.Vec2f(*track.conveyor_nodes[node]["animate_direction"])
                )
                forward_direction = 1
                if track_anchor != "":
                    if "1" not in node and "1" not in track_anchor:
                        forward_direction = -1
                    if "1" in node and "1" in track_anchor:
                        forward_direction = -1
                # if track.angle != Angle.NONE:
                #     forward_direction *= y_direction

                conveyor_prim.GetAttribute("inputs:animateScale").Set(track.conveyor_nodes[node]["animate_scale"])
                direction = forward_direction * Gf.Vec3f(*track.conveyor_nodes[node]["direction"])
                conveyor_prim.GetAttribute("inputs:direction").Set(direction)
                conveyor_prim.GetAttribute("inputs:curved").Set(track.conveyor_nodes[node]["curved"])

        return new_track

    def remove_track(self, usd_path: str) -> str:
        """Remove a track from the system and return the previous connected track path.

        Args:
            usd_path: Path of the track prim to remove.

        Returns:
            Path of the track connected via the removed track's parent anchor, or empty string.
        """
        previous_track = ""
        parent_anchor = self._parent_anchor[usd_path]
        if parent_anchor in self._anchor_connections[usd_path]:
            previous_track = self._anchor_connections[usd_path][parent_anchor]
        for anchor in self._anchor_connections[usd_path]:
            parent = self._anchor_connections[usd_path][anchor]
            if parent in self._anchor_connections:
                for cn in list(self._anchor_connections[parent].keys()):
                    if self._anchor_connections[parent][cn] == usd_path:
                        self._anchor_connections[parent].pop(cn)
                        break
        self._anchor_connections.pop(usd_path)
        self._parent_anchor.pop(usd_path)
        self._tracks.pop(usd_path)
        return previous_track

    def get_direction(self, usd_path: str) -> int:
        """Get the direction multiplier for the specified track.

        Args:
            usd_path: Path of the track prim.

        Returns:
            ``1`` or ``-1`` used to flip motion along the track based on ramp and anchors.
        """
        if usd_path in self._tracks:
            if self._tracks[usd_path].ramp == Ramp.FLAT:
                return 1
            conns = self.get_available_connections(usd_path)
            if conns and conns[0] == "":
                return -1
        return 1

    def get_next_pose(
        self,
        track: ConveyorTrack,
        track_anchor: str = "",
        x_direction: int = 1,
        y_direction: int = 1,
        parent: str | None = None,
        parent_anchor: str = "",
    ) -> Gf.Matrix4d:
        """Compute the world-space pose for placing the next track segment.

        Args:
            track: Conveyor track being placed.
            track_anchor: Anchor on ``track`` aligned to the parent.
            x_direction: Horizontal placement direction multiplier.
            y_direction: Depth placement direction multiplier.
            parent: USD path of parent track or container; ``None`` for world/default placement.
            parent_anchor: Anchor on ``parent`` for alignment.

        Returns:
            4x4 matrix expressing where to place the new track in parent/world space.
        """
        base_pose = Gf.Matrix4d()
        x_scale = 1
        y_scale = 1
        track_stage = Stage.Open(track.base_usd)
        a = UsdGeom.GetStageMetersPerUnit(self.stage)
        b = UsdGeom.GetStageMetersPerUnit(track_stage)
        scale_unit = b / a
        # print("Get base pose", parent, [parent_anchor], [track_anchor])
        if parent:
            anchor_prim = self.stage.GetPrimAtPath(f"{parent}{parent_anchor}")
            base_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(parent))
            base_pose = omni.usd.get_world_transform_matrix(base_xform.GetPrim())

            scale_ops = [o for o in base_xform.GetOrderedXformOps() if o.GetOpType() in [UsdGeom.XformOp.TypeScale]]
            anchor_pose = Gf.Matrix4d()
            if parent_anchor:
                anchor_pose = omni.usd.get_local_transform_matrix(anchor_prim)
            if scale_ops:
                # first, get the global pose of the base

                # Revert the scaling
                scale = Gf.Matrix4d().SetScale(scale_ops[0].Get())
                base_pose = scale.GetInverse() * base_pose
                base_pose.SetTranslateOnly(base_pose.ExtractTranslation() / scale_unit)

                # Rotate 180 degrees on the Z axis

                x_scale = scale_ops[0].Get()[0]
                y_scale = scale_ops[0].Get()[1]
            if y_scale < 0:
                t = anchor_pose.ExtractTranslation()
                t[1] = -t[1]
                anchor_pose.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = anchor_pose.ExtractRotation().TransformDir(Gf.Vec3d(1, 0, 0))
                prev_angle = anchor_pose.ExtractRotation().GetAngle()
                dot = direction * Gf.Vec3d(1, 0, 0)
                anchor_pose.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * anchor_pose.ExtractRotation()
                )
                if prev_angle < 89.0 and prev_angle > 1.0:
                    anchor_pose.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180) * anchor_pose.ExtractRotation())
            if x_scale < 0:
                t = anchor_pose.ExtractTranslation()
                t[0] = -t[0]
                anchor_pose.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = anchor_pose.ExtractRotation().TransformDir(Gf.Vec3d(1, 0, 0))

                dot = direction * Gf.Vec3d(1, 0, 0)
                anchor_pose.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * anchor_pose.ExtractRotation()
                )
            if not parent_anchor:
                anchor_pose.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))

            base_pose = anchor_pose * base_pose

        slope_and_direction = Gf.Vec3f(x_direction, y_direction, 1)
        # if track.angle == Angle.HALF and y_direction == 1 and track_anchor:
        #     y_direction = -1
        m = Gf.Matrix4d()
        m.SetRotateOnly(base_pose.ExtractRotation())
        m.SetTranslateOnly(base_pose.ExtractTranslation())
        # print(track, track_anchor)
        if track_anchor:
            anchor_prim = track_stage.GetPrimAtPath(str(track_stage.GetDefaultPrim().GetPath()) + track_anchor)
            mat = omni.usd.get_local_transform_matrix(anchor_prim)
            if y_direction < 0:
                t = mat.ExtractTranslation()
                t[1] = -t[1]
                mat.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                prev_angle = mat.ExtractRotation().GetAngle()
                direction = mat.ExtractRotation().TransformDir(Gf.Vec3d(1, 0, 0))

                dot = direction * Gf.Vec3d(1, 0, 0)
                mat.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * mat.ExtractRotation()
                )
                if prev_angle < 89.0 and prev_angle > 1.0:
                    mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180) * mat.ExtractRotation())
            if x_direction < 0:
                t = mat.ExtractTranslation()
                t[0] = -t[0]
                mat.SetTranslateOnly(t)
                # anchor_pose.SetRotateOnly(anchor_pose.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
                direction = mat.ExtractRotation().TransformDir(Gf.Vec3d(1, 0, 0))

                dot = direction * Gf.Vec3d(1, 0, 0)
                mat.SetRotateOnly(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0 + 2 * degrees(asin(dot))) * mat.ExtractRotation()
                )
            pivot_angle = mat.ExtractRotationQuat().GetNormalized()
            m.SetRotateOnly(
                m.ExtractRotationQuat()
                * (mat.ExtractRotationQuat().GetInverse() * Gf.Quatd(0, Gf.Vec3d(0, 0, -1))).GetNormalized()
            )
            t = Gf.Transform()
            t.SetRotation(Gf.Rotation(pivot_angle))
            t.SetTranslation(-mat.ExtractTranslation())
            m.SetTranslateOnly((t.GetMatrix().GetInverse() * base_pose).ExtractTranslation())
        if slope_and_direction[0] < 0:
            m.SetRotateOnly(m.ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 0, 1), 180))
        scale = Gf.Matrix4d().SetScale(scale_unit)
        # print(scale)
        # print(m)
        m.SetTranslateOnly((Gf.Matrix4d().SetTranslate(m.ExtractTranslation()) * scale).ExtractTranslation())
        # print(m)
        # print()
        return m
