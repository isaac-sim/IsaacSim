# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""USD render-product / render-var helpers for the RTSP streaming extension.

These helpers exist to satisfy ``omni.replicator.srtx``'s ``AnnotatorSRTX.attach``
validation, which requires that the AOV's ``RenderVar`` prim already exists as
a child of the render product (matched by ``sourceName``) before a Replicator
``Writer`` is attached.
"""

from __future__ import annotations

import carb
from pxr import Sdf, Usd

_SRTX_COMPRESSION_TYPE_ATTR = "srtx:compression:type"


def _add_render_var(stage: Usd.Stage, rendervar_path: str, aov_name: str) -> bool:
    """Create a ``RenderVar`` USD prim at ``rendervar_path`` for ``aov_name`` if it does not already exist.

    Args:
        stage: The USD stage.
        rendervar_path: Absolute USD path for the new ``RenderVar`` prim.
        aov_name: AOV source name to set on the prim's ``sourceName`` attribute.

    Returns:
        True on success, False if the prim could not be created.
    """
    if stage.GetPrimAtPath(rendervar_path).IsValid():
        carb.log_error(f"Cannot create RenderVar at {rendervar_path}: a prim already exists there")
        return False

    render_var_prim = stage.DefinePrim(rendervar_path, "RenderVar")
    if not render_var_prim.IsValid():
        carb.log_error(f"Failed to create RenderVar at {rendervar_path}")
        return False
    render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)
    carb.log_info(f"Created RenderVar at {rendervar_path} for AOV '{aov_name}'")
    return True


def ensure_render_var_on_product(
    stage: Usd.Stage,
    render_product_path: str,
    aov_name: str,
    compression_type: str,
) -> tuple[bool, str | None]:
    """Ensure a ``RenderVar`` for the given AOV exists as a child of the render product and is in ``orderedVars``.

    The helper is authoritative for the ``srtx:compression:type`` attribute on
    the rendervar prim: any pre-existing value is overwritten with
    ``compression_type`` (which may be the empty string, the canonical SRTX
    "no compression / raw" signal). The attribute is created if missing.

    Args:
        stage: The USD stage.
        render_product_path: Path to the render product prim.
        aov_name: The AOV source name to match or create.
        compression_type: SRTX compression type to author on the rendervar's
            ``srtx:compression:type`` attribute. Pass ``""`` for raw / no
            compression, or one of the SRTX-recognised codec names
            (e.g. ``"h264"``, ``"h265"``, ``"hevc"``).

    Returns:
        A ``(success, rendervar_path)`` tuple. On failure ``rendervar_path``
        is ``None`` and ``success`` is ``False``.
    """
    rp_prim = stage.GetPrimAtPath(render_product_path)
    if not rp_prim or not rp_prim.IsValid():
        carb.log_error(f"Render product prim '{render_product_path}' does not exist")
        return False, None

    rendervar_path: str | None = None
    for child in rp_prim.GetChildren():
        if child.GetTypeName() != "RenderVar":
            continue
        if child.HasAttribute("sourceName"):
            src = child.GetAttribute("sourceName").Get()
            if src == aov_name:
                rendervar_path = str(child.GetPath())
                break

    if rendervar_path is None:
        rendervar_path = render_product_path + "/" + aov_name
        if not _add_render_var(stage, rendervar_path, aov_name):
            return False, None

    render_var_prim = stage.GetPrimAtPath(rendervar_path)
    if not render_var_prim.HasAttribute(_SRTX_COMPRESSION_TYPE_ATTR):
        render_var_prim.CreateAttribute(_SRTX_COMPRESSION_TYPE_ATTR, Sdf.ValueTypeNames.String)
    render_var_prim.GetAttribute(_SRTX_COMPRESSION_TYPE_ATTR).Set(compression_type)

    ordered_vars_rel = rp_prim.GetRelationship("orderedVars")
    if not ordered_vars_rel:
        ordered_vars_rel = rp_prim.CreateRelationship("orderedVars")
    rendervar_sdf_path = Sdf.Path(rendervar_path)
    if rendervar_sdf_path not in ordered_vars_rel.GetTargets():
        ordered_vars_rel.AddTarget(rendervar_sdf_path)

    return True, rendervar_path
