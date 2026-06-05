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

"""Scene-level USD stage snapshot exporter.

Sessions reference a single ``<output_dir>/stage_snapshot.usd`` so multiple
recording sessions in the same directory share one flattened stage export. The
session HDF5 records only the stage filename; the replayer is responsible for
opening the stage before :meth:`EpisodeReplayer.prepare_episode`.
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone

import carb

STAGE_SNAPSHOT_BASENAME = "stage_snapshot"
"""Default basename (no extension) for scene-level stage snapshots."""

_SIDECAR_FORMAT_VERSION = 1


def export_stage_snapshot(
    output_dir: str,
    *,
    basename: str = STAGE_SNAPSHOT_BASENAME,
    flatten: bool = True,
    include_sidecar: bool = True,
) -> str:
    """Export the current USD stage to ``<output_dir>/<basename>.usd``.

    Args:
        output_dir: Target directory. Created if missing.
        basename: File basename (no extension).
        flatten: Export ``stage.Flatten()`` when ``True`` (default, self-contained);
            root-layer-only when ``False`` (faster but needs sublayers at replay time).
        include_sidecar: Also emit ``<basename>.sidecar.json``.

    Returns:
        Absolute path to the exported ``.usd`` file.

    Raises:
        RuntimeError: If no USD stage is currently loaded.
    """
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("export_stage_snapshot: no USD stage is currently loaded.")

    output_dir = os.path.abspath(os.path.expanduser(output_dir))
    os.makedirs(output_dir, exist_ok=True)

    usd_path = os.path.join(output_dir, f"{basename}.usd")
    if flatten:
        stage.Flatten().Export(usd_path)
    else:
        stage.GetRootLayer().Export(usd_path)

    if include_sidecar:
        sidecar_path = os.path.join(output_dir, f"{basename}.sidecar.json")
        payload = {
            "format_version": _SIDECAR_FORMAT_VERSION,
            "stage_snapshot": os.path.basename(usd_path),
            "stage_snapshot_flattened": bool(flatten),
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        try:
            with open(sidecar_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
        except OSError as exc:
            carb.log_warn(f"[export_stage_snapshot] Failed to write sidecar {sidecar_path}: {exc}")

    carb.log_info(f"[export_stage_snapshot] Wrote {usd_path}")
    return usd_path
