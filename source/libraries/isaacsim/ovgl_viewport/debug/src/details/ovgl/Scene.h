// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Build the GL renderer's in-memory Scene from an OVStage instance.
 * The loader reads supported geometry, transforms, and display attributes through
 * the public OVStage API and fills the internal GL Scene representation. */
#pragma once

#include "gl/Gpu.h" /* GpuMaterialParams */
#include "gl/Scene.h" /* Scene (the GL draw input) */

#include <ovstage/ovstage.h> /* ovstage_instance_t, ovstage_ordinal_t */

#ifdef __cplusplus
extern "C"
{
#endif

    /* Build `out`'s geometry + initial transforms from the live Mesh/Cube/Sphere prims
     * on `stage` at `ordinal` (topology pass). Frees any prior contents first. Returns
     * 1 on success, 0 on failure (ovgl_scene_last_error() set). Call once / on topology
     * change; use ovgl_scene_refresh_xforms() for the cheap per-frame update. */
    int ovgl_scene_build(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* out);

    /* Per-frame fast path: re-read only worldMatrix for the meshes already in `s`
     * (keyed by SceneMesh.path) and update their world_xform. No geometry re-read,
     * no reallocation. Returns 1 on success. Topology is assumed unchanged since the
     * last ovgl_scene_build (re-attach the stage / rebuild on add/remove). */
    int ovgl_scene_refresh_xforms(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* s);

    /* Atomically replace every resident mesh world transform from a dense array in mesh order and re-derive per-mesh
     * and scene bounds. The caller validates hierarchy and instance dependencies before supplying the already-composed
     * matrices. Returns 1 on success and leaves the prior scene untouched on failure. */
    int ovgl_scene_apply_xforms(Scene* s, const double* world_matrices, size_t mesh_count);

    /* Companion to ovgl_scene_refresh_xforms for the transform-only fast path:
     * re-derive the authored USD lights at `ordinal` into the scratch read by
     * ovgl_scene_lights(). Lights bake their composed world pose into GpuLight at
     * build time, so a transform edit that moves a light (or a light's ancestor)
     * must refresh them too — without this the fast path lights/shadows from the
     * stale pose forever (2026-07-19 fast-path adversary F1). Returns 1 on
     * success, 0 on failure (ovgl_scene_last_error() set; caller should fall back
     * to a full rebuild). Serialize with ovgl_scene_build (shared scratch). */
    int ovgl_scene_refresh_lights(ovstage_instance_t* stage, ovstage_ordinal_t ordinal);

    /* The DomeLight half of the same fast-path contract. The dome is not a
     * GpuLight — it becomes the IBL environment (Ovgl.cpp::apply_environment) — so
     * ovgl_scene_refresh_lights does not touch it, and read_scene_dome ran ONLY
     * inside ovgl_scene_build. A fast window therefore published its ordinal with
     * `s->dome_*` still holding cold-attach values, and apply_environment, which
     * early-outs on an unchanged resolved request, early-outed forever: a live
     * inputs:intensity/color/texture/rotation edit on a UsdLuxDomeLight was
     * dropped entirely by the attached lane (measured bit-identical frames across
     * dome 500 -> 15 -> 5000 -> 0; the Rule-0 cross-gate's capability (7)).
     *
     * Re-derives `s`'s dome_* fields AND nlights_unsupported at `ordinal` exactly
     * as a full rebuild would: both are inputs to apply_environment's resolved
     * request (the unsupported-light count decides whether a dome may own the
     * lighting), and both were build-only. Resets to the UsdLux defaults first, so
     * a dome that was deleted or made invisible inside the window turns the
     * environment OFF rather than keeping the last one it saw. Returns 1 on
     * success, 0 on failure (ovgl_scene_last_error() set; caller should fall back
     * to a full rebuild). Serialize with ovgl_scene_build (shared read scratch). */
    int ovgl_scene_refresh_dome(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* s);

    /* True when the resident scene contains PointInstancer-derived state whose
     * transforms require a full sealed-snapshot rebuild rather than a prototype
     * path-only worldMatrix refresh. */
    int ovgl_scene_requires_point_instancer_rebuild(const Scene* s);

    /* Material-value refresh for the targeted-refresh tier
     * (DESIGN_CHANGE_TRACKING.md D4): re-resolve each cached mesh's bound
     * material at `ordinal` and re-read its shader constants into the SAME
     * per-mesh slots of the CALLER-OWNED `materials` array
     * (SceneMesh.material_index indexes it — one material per pre-expansion
     * mesh, in mesh order), so the caller can re-upload materials without
     * touching geometry. Deliberately does NOT touch the build's global
     * material scratch: that is shared across renderers and may belong to
     * whichever scene was built last (the caller passes ITS retained
     * snapshot). Value-only by contract: a re-read whose texture indices
     * differ from the slot's existing ones fails the refresh — the global
     * texture registry has the same cross-renderer hazard, so equality
     * against the old slot is the only binding that is provably consistent
     * with the caller's texture snapshot. Meshes without a bound material
     * keep their displayColor-derived entry untouched. Serialize with
     * ovgl_scene_build (shared read scratch). Returns 0 (with
     * ovgl_scene_last_error set) on any failed or texture-changing re-read —
     * the caller must then fall back to the full rebuild. */
    int ovgl_scene_refresh_materials(ovstage_instance_t* stage,
                                     ovstage_ordinal_t ordinal,
                                     const Scene* s,
                                     GpuMaterialParams* materials,
                                     int nmaterials);

    /* Free heap-owned mesh payloads + the meshes array allocated by ovgl_scene_build. */
    void ovgl_scene_free(Scene* s);

    /* After ovgl_scene_build: the per-mesh PBR materials (one per SceneMesh, in mesh
     * order, so SceneMesh.material_index == its index here). Reads bound UsdPreviewSurface,
     * supported MDL, and direct-constant USD MaterialX Standard Surface/OpenPBR terminals, or
     * falls back to displayColor when the prim has no material. Valid until the next
     * ovgl_scene_build/free. Returns the count. */
    int ovgl_scene_materials(const GpuMaterialParams** out_materials);

    /* After ovgl_scene_build: the decoded RGBA8 textures the materials reference, indexed by
     * GpuMaterialParams::tex_indices. Pixels are owned by the scene and valid until the next
     * build/free. Returns the count (0 when nothing is textured). */
    int ovgl_scene_textures(const GpuTextureData** out_textures);

    /* After ovgl_scene_build: the authored USD lights (DistantLight/RectLight/SphereLight)
     * as gl GpuLight, for gpu_upload_lights. Empty when the scene has none (the
     * renderer then keeps its IBL/fallback lighting). Returns the count. */
    int ovgl_scene_lights(const GpuLight** out_lights);

    const char* ovgl_scene_last_error(void);

#ifdef __cplusplus
}
#endif
