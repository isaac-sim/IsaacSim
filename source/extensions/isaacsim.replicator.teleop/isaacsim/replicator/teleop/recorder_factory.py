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

"""Assemble teleop episode recorders.

The factory combines :class:`EpisodeRecorder` with teleop plus the standard articulation,
xform, and rigid-body recordables the UI exposes today.
"""

from __future__ import annotations

from typing import Any, Literal

from isaacsim.replicator.episode_recorder import (
    ArticulationRecordable,
    EpisodeRecorder,
    RigidBodyRecordable,
    SamplingConfig,
    XformRecordable,
)

from .recordables import TeleopControllerRecordable, TeleopHeadRecordable

PoseBackend = Literal["usd", "usdrt", "fabric"]


def build_teleop_recorder(
    output_dir: str,
    *,
    teleop_manager: Any,
    articulations: dict[str, str] | None = None,
    xforms: dict[str, str] | None = None,
    rigid_bodies: dict[str, str] | None = None,
    record_aim_pose: bool = True,
    record_head_pose: bool = True,
    sampling: SamplingConfig | None = None,
    session_metadata: dict[str, Any] | None = None,
    session_id: str | None = None,
    file_prefix: str = "episode",
    pose_backend: PoseBackend = "usd",
) -> EpisodeRecorder:
    """Build an :class:`EpisodeRecorder` preconfigured with teleop + scene recordables.

    The recordables are added under these HDF5 groups by default:

    - ``state/<name>`` for every articulation / xform / rigid body
    - ``teleop/left`` and ``teleop/right`` for controller inputs
    - ``teleop/head`` for the headset pose (when available)
    - ``meta/time`` for sim / physics step / wall time

    Args:
        output_dir: Session HDF5 output directory.
        teleop_manager: Live :class:`TeleopManager` instance.
        articulations: ``{name: prim_path}`` articulations to record.
        xforms: ``{name: prim_path}`` xform prims to record.
        rigid_bodies: ``{name: prim_path}`` rigid-body prims to record.
        record_aim_pose: Record OpenXR aim pose per controller.
        record_head_pose: Record headset pose (if the manager exposes it).
        sampling: :class:`SamplingConfig`. Defaults to physics post-step, decimation 1.
        session_metadata: Extra session attrs written to the manifest.
        session_id: Opaque id for command bus filtering. Auto-generated when omitted.
        file_prefix: Filename prefix for the session HDF5.
        pose_backend: Backend used by the recorder's shared pose-batch ``XformPrim.get_world_poses`` read each tick. Defaults to ``"usd"``. Reads cannot trigger the nested-articulation parent-lag bug because no writes happen during sampling, so ``"fabric"`` / ``"usdrt"`` are safe speedups when Fabric Scene Delegate is enabled. The replayer's backend is independent and is configured at :class:`EpisodeReplayer` construction time.

    Returns:
        The requested value.
    """
    recorder = EpisodeRecorder(
        output_dir,
        file_prefix=file_prefix,
        sampling=sampling or SamplingConfig(),
        session_metadata=session_metadata,
        session_id=session_id,
        pose_backend=pose_backend,
    )

    for name, path in (articulations or {}).items():
        recorder.add(ArticulationRecordable(group=f"state/{name}", prim_path=path))
    for name, path in (xforms or {}).items():
        recorder.add(XformRecordable(group=f"state/{name}", prim_path=path))
    for name, path in (rigid_bodies or {}).items():
        recorder.add(RigidBodyRecordable(group=f"state/{name}", prim_path=path))

    for side in ("left", "right"):
        recorder.add(
            TeleopControllerRecordable(
                group=f"teleop/{side}",
                side=side,
                record_aim_pose=record_aim_pose,
                teleop_manager=teleop_manager,
            )
        )
    if record_head_pose and hasattr(teleop_manager, "add_head_observer"):
        recorder.add(TeleopHeadRecordable(group="teleop/head", teleop_manager=teleop_manager))

    return recorder
