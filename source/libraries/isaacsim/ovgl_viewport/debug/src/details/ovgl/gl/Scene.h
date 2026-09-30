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

#pragma once

/*
 * scene.h — USD scene data extracted via the nanousd C API (nanousdapi.h).
 *
 * All mesh data is arena-allocated for one-shot free.
 * Uses zero-copy (nanousd_arraydataf/i) where possible.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        /* Vertex data — flat arrays, 3 floats per vertex */
        float* positions; /* float[nvertices * 3] */
        float* normals; /* float[nvertices * 3], may be NULL */
        float* colors; /* float[nvertices * 3], per-vertex displayColor, may be NULL */
        float* texcoords; /* float[nvertices * 2], UV coordinates, may be NULL */

        /* Index data — triangulated */
        uint32_t* indices; /* uint32_t[nindices] */

        /* Authored Ptex surface color sampled once per triangulated corner.
         * Packed as RGBA8 in final draw index order: three colors per output
         * triangle. ptex_color_offset is the offset into the GPU-side packed
         * color buffer, or 0xFFFFFFFF when absent. */
        uint32_t* ptex_tri_colors;
        int ptex_tri_color_count;
        uint32_t ptex_color_offset;

        int nvertices;
        int nindices;

        /* Debug identity: composed USD prim path. Heap-owned so it survives
         * scene_release_mesh_payloads(), which frees the arena after upload. */
        char* path;

        /* World-space transform (row-major 4x4) */
        double world_xform[16];

        /* Per-mesh world-space bounds (computed during load) */
        float bounds_min[3];
        float bounds_max[3];

        /* Object-space bounds. Used to bound expanded instances without
         * re-transforming every shared prototype vertex per instance. */
        float local_bounds_min[3];
        float local_bounds_max[3];

        /* Fallback displayColor (used if colors == NULL) */
        float display_color[3];
        int has_display_color;

        /* Material assignment (-1 = no material) */
        int material_index;

        /* Offsets into merged buffers (set by viewer) */
        uint32_t vertex_offset;
        uint32_t index_offset;
        uint32_t buffer_index;
        uint64_t index_byte_offset;
        uint8_t index_type_bits; /* 16 or 32 for chunked raster IBs */

        /* Instancing: index of prototype mesh in meshes[] array.
         * Equal to own index if this IS the prototype or non-instanced.
         * When prototype_idx != own index, positions/normals/indices
         * are shared pointers (not separately allocated). */
        int prototype_idx;

        /* Meshes under a PointInstancer prototype subtree are kept only as
         * shared geometry sources for expanded instances. They must not draw
         * standalone. */
        int is_proto_only;

        /* Runtime visibility for OVRTX/nu_set_visibility mutation. Meshes hidden
         * by authored USD visibility are not loaded; this flag handles live edits. */
        int visible;

        /* USD Gprim sidedness + authored winding (orientation == "leftHanded").
         *
         * This used to say "Hydra/Storm culls backfaces when doubleSided=false;
         * ovgl mirrors that". It does not, and ovgl did not mirror it: measured
         * with `usdrecord --renderer GL` on the same wrap USDA and camera, Storm
         * DRAWS the away-facing single-sided face, as does official ovrtx 0.4.
         * UsdGeomGprim only permits the cull as an optimization. So ovgl no
         * longer culls by default either -- double_sided now feeds nothing but
         * the opt-in OVGL_BACKFACE_CULL path (Ovgl.cpp::apply_mesh_sidedness),
         * where it still suppresses the cull per its USD meaning. */
        int double_sided;
        int front_face_cw;

        /* Tier 3 lazy extraction (see docs/plans/TIER_3_LAZY_MESH.md).
         * Set by scene_load when NUSD_LAZY_MESH=1: the renderer's not-yet-
         * implemented extract-deferred worker (step 3) re-resolves the
         * prim via ``nanousd_prim(stage, lazy_prim_idx)`` and reads
         * attributes on demand. ``0`` (calloc default) in eager-loaded
         * scenes — step 3 gates on positions == NULL rather than this
         * sentinel.
         *
         * Lifetime caveat (activates with step 3): the owning Scene MUST
         * keep the nanousd stage alive past the current scene_release-
         * after-attach behavior once step 3 lands. Dormant in step 2. */
        int lazy_prim_idx;

        /* Preprocessed meshlet range — populated only by the geometry cache
         * (geo_cache.c) on a cache hit; meshlet_count 0 = none. Instances share
         * their prototype's range. The meshlets index Scene.meshlet_vertices /
         * meshlet_triangles. */
        uint32_t meshlet_offset;
        uint32_t meshlet_count;
    } SceneMesh;

    /* A meshoptimizer meshlet: a small triangle cluster with cull bounds, in the
     * mesh-shader-native compact layout — a per-meshlet table of unique vertex
     * indices plus 8-bit micro-indices into that table. Populated only by the
     * geometry cache (geo_cache.c) on a cache hit, which compacts the cache's
     * expanded index stream at load time; the USD parse path leaves
     * Scene.meshlets NULL. */
    typedef struct
    {
        float center[3];
        float radius;
        float cone_axis[3];
        float cone_cutoff;
        uint32_t vertex_offset; /* into Scene.meshlet_vertices */
        uint32_t vertex_count; /* unique vertices used by this meshlet (<= 64) */
        uint32_t triangle_offset; /* byte offset into Scene.meshlet_triangles */
        uint32_t triangle_count; /* triangles; 3*triangle_count micro-indices */
    } SceneMeshlet;

    /* BasisCurves topology. Mirrors USD's `basis`/`type`/`wrap` token
     * tuple. The renderer in phase 2 only consumes `type=cubic`,
     * `basis=bezier`, `wrap=nonperiodic`; the others are loaded (so
     * bounds + the prim count are right) but skipped at patch-emission
     * time with a single dedup'd warning. */
    typedef enum
    {
        CURVE_BASIS_BEZIER = 0,
        CURVE_BASIS_BSPLINE = 1,
        CURVE_BASIS_CATMULLROM = 2,
        CURVE_BASIS_LINEAR = 3 /* type=linear, basis ignored */
    } SceneCurveBasis;

    typedef enum
    {
        CURVE_WRAP_NONPERIODIC = 0,
        CURVE_WRAP_PERIODIC = 1,
        CURVE_WRAP_PINNED = 2
    } SceneCurveWrap;

    typedef struct
    {
        /* Per-CV data in object space. The curve shader applies world_xform
         * through the per-curve model UBO so transformed tangents/normals stay
         * consistent with transformed positions. */
        float* cvs; /* float[nv * 3] */
        float* widths; /* float[nv] — per-CV after fan-out */
        float* colors; /* float[nv * 3] or NULL */
        int nv;
        int ncurves_in_prim; /* len(curveVertexCounts) — number of distinct curves */
        int* curve_vertex_counts; /* int[ncurves_in_prim] for periodic/pinned wrap math */

        /* 4-CV-per-patch index buffer (uint32, pre-offset to absolute vertex
         * space at viewer-merge time). Built per Storm's _BuildCubicIndexArray. */
        uint32_t* patch_indices;
        int npatches;

        /* World-space transform (row-major 4x4), applied by the renderer as a
         * per-curve model matrix. */
        double world_xform[16];

        /* Per-curve world-space bounds. */
        float bounds_min[3];
        float bounds_max[3];

        /* Fallback displayColor (used if colors == NULL). */
        float display_color[3];
        int has_display_color;

        /* Topology metadata (from USD attributes). */
        SceneCurveBasis basis;
        SceneCurveWrap wrap;
        int type_is_cubic; /* 1 = cubic, 0 = linear */

        /* Offsets into merged buffers (set by viewer at upload time). */
        uint32_t vertex_offset;
        uint32_t index_offset;
    } SceneCurve;

/* Light kinds. Keep numeric values aligned with the Vulkan renderer so
 * debug captures and shader-side code can use the same conventions. */
#define SCENE_LIGHT_RECT 0
#define SCENE_LIGHT_DISTANT 1
#define SCENE_LIGHT_SPHERE 2

    typedef struct
    {
        int kind; /* SCENE_LIGHT_* */

        /* World-space frame.
         *   RectLight/DiskLight: position is the emitter center; normal is the
         *     emit direction; u_axis/v_axis are world-space half-extents. Disks
         *     are approximated as equal-area rects.
         *   DistantLight: normal is the emit direction, light travels along
         *     -normal. u_axis/v_axis are unused.
         *   SphereLight: position is the center; u_axis[0] stores radius. */
        float position[3];
        float normal[3];
        float u_axis[3];
        float v_axis[3];

        /* UsdLuxLight-style authored properties. */
        float color[3];
        float intensity; /* intensity * 2^exposure */
        int normalize;
        float angle_deg;
    } SceneLight;

    /* Per-instance transform for the compact PointInstancer / native-instance
     * model (ported from the Vulkan renderer). 12 floats = column-major affine
     * matching the composed inst_world (cols 0..2 = R*S basis vectors, col 3 =
     * translation; the always-(0,0,0,1) bottom row is dropped). PI/native-instance
     * placements live ONLY here — never as per-instance SceneMesh clones. */
    typedef struct
    {
        float m[12];
    } SceneInstanceTransform;

    /* Expand a SceneInstanceTransform (packed by scene_pack_affine12 from the USD
     * row-major world matrix: m[0..2]=row0, m[3..5]=row1, m[6..8]=row2,
     * m[9..11]=translation) back to the 16-float USD row-major matrix. A GLES
     * instanced vertex shader reads four consecutive vec4s as mat4 COLUMNS
     * (column-major), so this row-major layout reconstructs to the correct
     * column-vector world matrix (mat4(attribs) == transpose(usd) == the world
     * matrix used as MeshBlock.model in the non-instanced path). */
    static inline void scene_instance_transform_to_model16(const SceneInstanceTransform* t, float dst[16])
    {
        dst[0] = t->m[0];
        dst[1] = t->m[1];
        dst[2] = t->m[2];
        dst[3] = 0.0f;
        dst[4] = t->m[3];
        dst[5] = t->m[4];
        dst[6] = t->m[5];
        dst[7] = 0.0f;
        dst[8] = t->m[6];
        dst[9] = t->m[7];
        dst[10] = t->m[8];
        dst[11] = 0.0f;
        dst[12] = t->m[9];
        dst[13] = t->m[10];
        dst[14] = t->m[11];
        dst[15] = 1.0f;
    }

    typedef enum
    {
        SCENE_INSTANCE_SOURCE_POINT_INSTANCER = 0,
        SCENE_INSTANCE_SOURCE_NATIVE_INSTANCE = 1,
    } SceneInstanceSourceKind;

    /* Compact instance batch: one prototype mesh (meshes[prototype_mesh_idx],
     * is_proto_only) instanced by a slice of Scene.pi_transforms. */
    typedef struct
    {
        int prototype_mesh_idx; /* index into Scene.meshes (a proto sub-mesh) */
        uint32_t transform_offset; /* slice base into Scene.pi_transforms */
        uint32_t transform_count;
        int source_prim_idx; /* flat-prim index of the authoring PI (-1 = n/a) */
        int material_or_binding_id; /* -1 = inherit from prototype */
        int source_kind; /* SceneInstanceSourceKind */
    } SceneInstanceBatch;

    typedef struct
    {
        SceneMesh* meshes;
        int nmeshes;
        SceneCurve* curves;
        int ncurves;

        /* Compact instance batches (PointInstancer + native-instance replay). PI
         * clones live ONLY here, never in meshes[]. Heap-allocated; freed in
         * scene_free. Drawn via GLES instancing (glDrawElementsInstanced). */
        SceneInstanceBatch* pi_batches;
        int npi_batches;
        SceneInstanceTransform* pi_transforms;
        uint64_t npi_transforms;

        /* Scene bounds (world space) */
        float bounds_min[3];
        float bounds_max[3];

        /* Authored up axis: 0 = X, 1 = Y (USD default), 2 = Z (Omniverse /
         * Isaac Sim convention). Detected from `upAxis` stage metadata when
         * present, otherwise inferred from bounds shape. The viewer reads
         * this to set the Camera's frame of reference; the scene itself is
         * left in its authored coordinates (no Y-up rotation). */
        int up_axis;
        int has_authored_light; /* any active USD light prim, including DomeLight */
        SceneLight* lights; /* RectLight/DistantLight/SphereLight */
        int nlights;

        /* USD UsdLuxDomeLight metadata (first active DomeLight wins, matching
         * Hydra). dome_hdr_path[0] == '\0' means no authored HDR texture.
         *
         * These fields existed from the beginning and NOTHING ever wrote them:
         * Scene.cpp hardcoded dome_intensity = 1.0f and no code path ever
         * looked at a DomeLight prim, so an authored dome contributed exactly
         * zero. Measured on assets/robot_ground_scene.usda (DomeLight 900 +
         * DistantLight 1200) at matched framing: deactivating the dome, doubling
         * its intensity, 10x-ing it and recolouring it pure red all produced a
         * BIT-IDENTICAL ovgl frame (MAE 0.000, max 0, 0.00% px) in both the
         * attach and file lanes, while official ovrtx 0.4 moved 100% of pixels on
         * every one of the four. The sky was (0,0,0) exactly over all 252,166
         * background pixels against official's (222,223,225).
         *
         * Scene.cpp::read_scene_dome now fills them and Ovgl.cpp turns them
         * into the renderer's environment. `has_dome` is the authored-ness flag:
         * intensity 0 is a legal authoring that means "black dome", which is NOT
         * the same as no dome at all (the latter keeps the synthetic headlight
         * rig). */
        int has_dome;
        /* Active, visible UsdLux lights whose schema this renderer cannot
         * evaluate (DiskLight, CylinderLight -- there is no GpuLight::kind for
         * either). Ovgl.cpp refuses to let a DomeLight replace the synthetic
         * fill rig while this is non-zero: a scene whose key light is invisible
         * to the renderer is not a scene the dome can light on its own. */
        int nlights_unsupported;
        char dome_hdr_path[512];
        float dome_intensity; /* inputs:intensity * 2^inputs:exposure */
        float dome_color[3]; /* inputs:color, default (1,1,1) */
        float dome_rotation_y; /* degrees, future xform plumbing */

        /* Arena memory (opaque, freed by scene_free) */
        void* _arena;

        /* Keep stage handle alive for zero-copy pointers */
        void* _stage;

        int _meshes_heap;

        /* Preprocessed meshlets — populated only by the geometry cache on a
         * cache hit; the USD parse path leaves these NULL/0. Compact layout: each
         * SceneMeshlet indexes a slice of meshlet_vertices (its unique global
         * vertex indices) and meshlet_triangles (its 8-bit micro-indices). */
        SceneMeshlet* meshlets;
        int nmeshlets;
        uint32_t* meshlet_vertices; /* per-meshlet vertex tables, concatenated */
        int nmeshlet_vertices;
        unsigned char* meshlet_triangles; /* per-meshlet micro-indices, concatenated */
        int nmeshlet_triangles;
    } Scene;

    enum
    {
        SCENE_LOAD_SKIP_TEXCOORDS = 1u << 0,
    };

    /* Set process-local scene loading flags for the next scene_load call.
     * The OpenGL geometry-only path uses this to avoid UV-driven vertex
     * expansion when no material pipeline will consume texcoords. */
    void scene_set_load_flags(unsigned flags);

    /* Set the time used by subsequent scene_load calls when reading USD xform
     * attributes. Pass NaN for authored default time. */
    void scene_set_load_time(double t);

    /* Load a USD file (USDA or USDC). Returns NULL on failure. */
    Scene* scene_load(const char* filepath);

    /* Free all scene memory + close the USD stage. */
    void scene_free(Scene* scene);

    /* After the renderer has built GPU buffers and material bindings, mesh-only
     * scenes no longer need source vertex/index payloads or the open USD stage.
     * Returns 1 when payloads were released, 0 when retained. */
    int scene_release_mesh_payloads(Scene* scene);

#ifdef __cplusplus
}
#endif
