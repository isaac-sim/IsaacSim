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

/* Internal OpenGL renderer interface for OVStage scenes.
 * ovgl_render_frame writes framebuffer output into a host RGBA8 buffer. */
#pragma once

#include <ovstage/ovstage.h> /* ovstage_instance_t, ovstage_ordinal_t, ovx_string_t */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ovgl_renderer ovgl_renderer_t;

    typedef struct
    {
        int32_t status; /* 0 == success */
    } ovgl_result_t;

    typedef struct
    {
        int gpu_device_id; /* GL/EGL device index (0 = default) */
        int samples_per_pixel; /* MSAA samples (1 = off); GL rasterizer, not path tracing */
        int max_bounces; /* accepted for ABI parity; ignored by the rasterizer */
        const char* name;
    } ovgl_renderer_desc_t;

    typedef struct
    {
        double eye[3];
        double target[3];
        double up[3];
        double fov_y_rad;
        int image_width;
        int image_height;
    } ovgl_camera_t;

    typedef struct
    {
        size_t mesh_count;
        size_t triangle_count;
        ovstage_ordinal_t last_render_ordinal;
        uint64_t last_structure_gen;
        int32_t bounds_valid;
        int32_t up_axis; /* 1 = Y, 2 = Z */
        float bounds_min[3];
        float bounds_max[3];
    } ovgl_stats_t;

    void ovgl_get_version(uint32_t* out_major, uint32_t* out_minor, uint32_t* out_patch);

    /* Lifecycle */
    ovgl_result_t ovgl_create_renderer(const ovgl_renderer_desc_t* desc, ovgl_renderer_t** out_renderer);
    /* An adopted renderer must be destroyed while its pinned GL context is
     * current; otherwise this fails without releasing or invalidating `renderer`,
     * so the caller can make the owner current and retry. Qt embedders should
     * destroy from QOpenGLContext::aboutToBeDestroyed after makeCurrent(). */
    ovgl_result_t ovgl_destroy_renderer(ovgl_renderer_t* renderer);

    /* Bind to an OVStage instance, clear cached geometry, and force a full
     * topology synchronization on the next frame. */
    ovgl_result_t ovgl_attach_ovstage(ovgl_renderer_t* renderer, ovstage_instance_t* stage);

    ovgl_result_t ovgl_set_camera(ovgl_renderer_t* renderer, const ovgl_camera_t* camera);

    /* Pause/resume the per-frame ovstage worldMatrix pulls (freezes scene state, camera stays live). */
    ovgl_result_t ovgl_set_pull_paused(ovgl_renderer_t* renderer, int paused);

    /* Load an equirectangular HDR as the IBL environment: image-based lighting for the
     * PBR material pipeline (real metallic/glossy reflections) plus a visible background.
     * Disables the synthetic fallback rig while loaded. `intensity` scales the env
     * (1.0 = as authored). Pass NULL/"" to clear. Applied on the next render_frame (it
     * needs the GL context); a load failure silently falls back to the synthetic rig. */
    ovgl_result_t ovgl_set_environment(ovgl_renderer_t* renderer, const char* hdr_path, float intensity);

    /* Render one frame against `ordinal`. `output_rgba8_host` is a HOST buffer of
     * width*height*4 bytes (GL framebuffer readback). Headless path: ovgl owns its own
     * offscreen EGL context. */
    ovgl_result_t ovgl_render_frame(ovgl_renderer_t* renderer,
                                    ovstage_ordinal_t ordinal,
                                    void* output_rgba8_host,
                                    int output_width,
                                    int output_height);

    /* GUI / zero-readback path: render against `ordinal` in the CALLER's already-current GL
     * context (e.g. a Qt QOpenGLWidget's paintGL) and return the GL_TEXTURE_2D (RGBA8) the frame
     * was resolved into, for the caller to draw with a fullscreen quad. NO host readback, NO EGL
     * context switch. The texture is owned by ovgl and valid until the next render/resize. The
     * GL state the render touches is saved + restored. The first successful adopted render pins
     * this renderer to the exact current context; every later adopted render and destruction must
     * run with that same context current. Fails closed on no/different context. */
    ovgl_result_t ovgl_render_to_texture(ovgl_renderer_t* renderer,
                                         ovstage_ordinal_t ordinal,
                                         int output_width,
                                         int output_height,
                                         unsigned* out_gl_texture,
                                         int* out_tex_width,
                                         int* out_tex_height);

    /* GUI present: render against `ordinal` in the caller's current GL context and blit the
     * frame straight into `dst_fbo`. The source rectangle uses top-down output coordinates. The
     * destination is cleared and the source is aspect-fitted into destination_width x
     * destination_height. Optional overlay text is composed after the blit and is not part of any
     * host capture. dst_fbo must be single-sample. The first successful adopted render pins the
     * renderer to the exact current context; later adopted renders and destruction require that
     * same context and fail closed otherwise. */
    ovgl_result_t ovgl_render_to_fbo(ovgl_renderer_t* renderer,
                                     ovstage_ordinal_t ordinal,
                                     unsigned dst_fbo,
                                     int output_width,
                                     int output_height,
                                     int source_x0,
                                     int source_y0,
                                     int source_x1,
                                     int source_y1,
                                     int destination_width,
                                     int destination_height,
                                     const char* overlay_text);

    /* Cheap transform-only update: re-read worldMatrix (and re-derive world bounds) for the
     * already-cached meshes at `ordinal`, WITHOUT re-reading geometry or re-uploading GL buffers,
     * and mark the renderer current at that ordinal so the next render_frame draws the cached
     * buffers instead of rebuilding the scene.
     *
     * render_frame rebuilds the whole scene whenever the ordinal moves, because it does not
     * itself ask what changed. The OVGL viewport adapter uses the public OVStage query and
     * ordinal-window read APIs to decide transforms-only vs full; this entry point remains the
     * transforms-only lever that classifier pulls. On a large stage the full rebuild — re-reading every mesh and
     * re-uploading every GL buffer — costs ~1s, which makes an interactive drag unusable.
     * A caller that knows only transforms moved can use this path to retain geometry and GL buffers while
     * ovgl_scene_refresh_xforms re-reads worldMatrix.
     *
     * Requires a prior successful render_frame (topology built); fails otherwise so a caller
     * cannot skip the initial build. Does nothing while pull is paused. */
    ovgl_result_t ovgl_refresh_transforms(ovgl_renderer_t* renderer, ovstage_ordinal_t ordinal);

    /* Replace the resident mesh transforms from an already-composed dense batch
     * in ovgl mesh order. This is the attached backend's transform-only fast
     * path: it avoids publishing and rereading the private mirror when the
     * retained pulled snapshot can compose the exact worlds itself. The update
     * is atomic and also refreshes mesh/scene bounds. Curves and compact
     * instances are currently unsupported; failure leaves the resident scene
     * and published ordinal unchanged so the caller can use its mirror path. */
    ovgl_result_t ovgl_refresh_transform_batch(ovgl_renderer_t* renderer,
                                               ovstage_ordinal_t ordinal,
                                               const double* world_matrices,
                                               size_t mesh_count);

    /* Force the next render to rebuild its resident scene from the attached
     * OVStage, even if the numeric ordinal happens to match the last direct
     * transform batch. CPU/GPU resources remain owned until that rebuild. */
    ovgl_result_t ovgl_invalidate_scene(ovgl_renderer_t* renderer);

    /* Targeted material refresh (companion to ovgl_refresh_transforms, for change
     * windows carrying only shader-input value edits): re-resolve each cached
     * mesh's bound material at `ordinal` and replace ONLY the GPU material state
     * on the next render_frame — no geometry re-read, no vertex/index buffer
     * re-upload, no scene rebuild. Mesh->material bindings stay valid (materials
     * are one-per-mesh in mesh order). Does NOT mark the renderer current: call
     * ovgl_refresh_transforms AFTER this succeeds — it re-reads transforms and
     * lights and is the single publish point, so a failure of either refresh
     * leaves the ordinal unpublished and the next render_frame takes the
     * full-rebuild correctness baseline (never a cheap frame with one stale
     * aspect). A no-op while pulls are paused or the scene needs a
     * PointInstancer rebuild; fails when topology is not built or a material
     * re-read fails. */
    ovgl_result_t ovgl_refresh_materials(ovgl_renderer_t* renderer, ovstage_ordinal_t ordinal);

    typedef struct
    {
        int32_t hit; /* 1 = a mesh was hit, 0 = miss (empty space) */
        double world_pos[3]; /* nearest hit point, world space */
        double distance; /* along the normalized world ray from the eye */
        char path[1024]; /* composed USD prim path of the hit mesh */
    } ovgl_pick_result_t;

    /* Exact CPU object pick: cast the camera ray through NDC (x right, y up, both
     * in [-1,1]) against the resident scene cache (AABB broadphase + ray/triangle
     * narrowphase; both triangle sides count, hidden meshes and prototype-only
     * instancer sources are skipped). Uses the camera set by ovgl_set_camera and
     * the geometry from the last render_frame's topology pass. Same-thread as
     * render_frame (the renderer is single-threaded by contract); needs no GL
     * context. A miss inside an attached-but-empty scene is a success with
     * out->hit == 0; an invalid camera or no attached stage fails. */
    ovgl_result_t ovgl_pick(ovgl_renderer_t* renderer, double ndc_x, double ndc_y, ovgl_pick_result_t* out);

    /* Read-only view of one resident mesh, for SIBLING consumers of the same scene
     * (a ray-cast sensor backend, a collision probe, an exporter) that must see
     * exactly the geometry ovgl draws rather than re-extracting it from the stage.
     *
     * Geometry is OBJECT space with a separate world transform — deliberately not
     * pre-transformed, because that is the layout hardware ray tracing wants
     * (one acceleration structure per mesh + a per-instance transform) and it is
     * how ovgl already stores it.
     *
     * LIFETIME: every pointer here belongs to the renderer's scene cache. A
     * moved-ordinal render_frame REBUILDS that cache and frees the previous
     * contents, so views must be re-fetched after any render at a new ordinal.
     * ovgl_refresh_transforms mutates in place instead, so after a refresh the
     * pointers stay valid and only `world_xform` changes. */
    typedef struct
    {
        const float* positions; /* [vertex_count * 3], object space, packed xyz */
        uint32_t vertex_count;
        const uint32_t* indices; /* [index_count], triangle list */
        uint32_t index_count;
        double world_xform[16]; /* row-major, USD row-vector: p_world = p_obj * W */
        const char* path; /* composed USD prim path */
        int32_t visible;
        int32_t is_proto_only; /* instancer prototype source; not drawn */
    } ovgl_mesh_view_t;

    /* Number of meshes in the resident scene, or -1 if topology is not built yet
     * (call render_frame once first). Includes hidden and prototype-only meshes;
     * consumers filter on the view's flags. */
    int32_t ovgl_get_mesh_count(ovgl_renderer_t* renderer);

    /* Fill `out` with mesh `index` in [0, ovgl_get_mesh_count). */
    ovgl_result_t ovgl_get_mesh_view(ovgl_renderer_t* renderer, int32_t index, ovgl_mesh_view_t* out);

    /* Grid and axes overlay (stock-viewport parity).
     *
     * Opt-in per renderer, default OFF. While enabled, every render path
     * (render_frame / render_to_texture / render_to_fbo) draws a world-space
     * reference grid on the stage's ground plane after the opaque scene pass:
     *
     *   * up-axis aware: the grid lies in the XY plane for Z-up stages and the
     *     XZ plane for Y-up stages (Scene.up_axis, mirrored from the portable
     *     `usd-stage-up-axis` column; an EMPTY scene falls back to the reserved
     *     population stage-info prim's `upAxis` token, else Z);
     *   * fixed extent ±100 stage units around the origin, 1-unit minor lines
     *     (muted gray, auto-fading when subpixel-dense) and 10-unit major lines
     *     (brighter gray), all fading out with camera distance;
     *   * the two IN-PLANE origin axes are drawn slightly brighter and colored
     *     by world axis (X red, Y green, Z blue): Z-up shows red X + green Y,
     *     Y-up shows red X + blue Z;
     *   * depth semantics: the grid is depth-TESTED against the scene (opaque
     *     geometry occludes it, matching stock) but never depth-WRITTEN — it is
     *     alpha-blended over the frame before the transparent pass, cannot
     *     occlude scene content, and leaves the Depth AOV pure scene depth.
     *
     * The overlay never affects captures while disabled: with enabled == 0 the
     * render byte streams are bit-identical to a build without this feature.
     * Interactive front-ends (the Kit OVGL viewport) are expected to enable it
     * for editor sessions; offscreen tests/harnesses keep the default OFF.
     * Env override for ad-hoc debugging: OVGL_GRID_OVERLAY=1 makes newly
     * created renderers start with the overlay enabled. */
    ovgl_result_t ovgl_set_grid_overlay(ovgl_renderer_t* renderer, int enabled);

    /* Screen-space ambient occlusion.
     *
     * ovgl computes geometric AO from a depth+normal prepass and applies it to the
     * AMBIENT (environment/IBL) term only — never to direct light, which already
     * has shadow maps. Before this existed, a subject that loaded no occlusion
     * TEXTURE had a uniform `ao` over its whole surface, so every crevice received
     * the full dome ambient; that is what "washed out" looked like on a dense
     * articulated assembly.
     *
     *   enabled    0 skips the prepass and the SSAO pass entirely. With it off the
     *              render byte stream is identical to a build without the feature.
     *   radius     STAGE UNITS. The distance within which geometry counts as
     *              occluding, and the one knob that must suit the scene: 0.6 suits
     *              a metres-per-unit robotics scene, a centimetres one wants ~60.
     *   bias       COSINE threshold, not a depth offset: an occluder whose
     *              direction from the shaded point lies within acos(bias) of the
     *              tangent plane is ignored. Discards grazing taps that depth
     *              quantization makes meaningless.
     *   intensity  scales the occlusion before it is subtracted from 1. Raising it
     *              deepens crevices but also darkens places nothing occludes —
     *              which is a parity regression, not a look preference.
     *   strength   blends the whole effect in, 0..1. 0 reproduces the pre-SSAO
     *              frame exactly, so it is the safe live A/B control.
     *
     * Applies from the NEXT rendered frame; safe to call between frames on a live
     * renderer. Defaults come from OVGL_AO / OVGL_AO_RADIUS / _BIAS / _INTENSITY /
     * _STRENGTH at renderer creation. Pass a negative float to leave that field
     * unchanged; pass enabled < 0 to leave the switch unchanged. */
    ovgl_result_t ovgl_set_ao_settings(
        ovgl_renderer_t* renderer, int enabled, float radius, float bias, float intensity, float strength);

    /* Read back what ovgl_set_ao_settings would apply. Any out pointer may be
     * NULL. Lets a front-end populate its controls from the renderer rather than
     * duplicating the defaults. */
    ovgl_result_t ovgl_get_ao_settings(
        ovgl_renderer_t* renderer, int* enabled, float* radius, float* bias, float* intensity, float* strength);

    /* Depth AOV (linear image-plane depth readback).
     *
     * Opt-in per renderer. While enabled, every successful HEADLESS
     * ovgl_render_frame additionally reads the frame's depth buffer back into a
     * float32 host plane holding, per pixel, the PERPENDICULAR distance from the
     * camera (image plane) to the nearest depth-writing surface, in STAGE UNITS
     * — the rasterizer analog of the official ovrtx "DistanceToImagePlaneSD"
     * render var (which publishes meters; the caller owns the stage-unit →
     * meter conversion, ovgl does not know metersPerUnit). Pixels the frame did
     * not draw (the cleared far plane) read +INFINITY.
     *
     * Semantics and limits, all documented behavior:
     *   * hit depths lie inside the renderer's fixed view frustum
     *     [0.02, 10000] stage units; geometry outside it is not rasterized and
     *     reads +INFINITY like any other no-hit pixel;
     *   * precision is z-buffer-shaped: DEPTH_COMPONENT24 is read back through a
     *     lossless 24-bit fixed-point pack, so the linearized error grows
     *     quadratically with distance (~d^2 * (far-near)/(near*far) / 2^24);
     *   * transparent (alpha-blend) surfaces do not write depth: the plane
     *     reports the nearest OPAQUE or alpha-cutout surface behind them, which
     *     is exactly what the color pass depth-tested against;
     *   * with supersampling the depth plane is POINT-sampled at pixel centers
     *     while color is box-averaged, so silhouette pixels may disagree;
     *   * the CPU selection-outline composite touches only color — depth stays
     *     scene depth;
     *   * rows are top-down, matching the color buffer orientation;
     *   * the adopted-context GUI paths (render_to_texture / render_to_fbo) do
     *     not capture depth; they invalidate any previously captured plane so a
     *     stale depth can never pair with a newer color frame.
     */
    ovgl_result_t ovgl_set_depth_capture(ovgl_renderer_t* renderer, int enabled);

    /* The depth plane captured by the LAST successful ovgl_render_frame, and the
     * resolution it was rendered at. Fails closed unless capture was enabled for
     * that exact render (no frame yet, capture off, or a later adopted-path
     * render all fail). The pointer stays valid until the next render/destroy.
     * Same-frame guarantee: a success here always describes the same frame as
     * the RGBA8 buffer that render_frame call produced. */
    ovgl_result_t ovgl_get_depth_view(ovgl_renderer_t* renderer, const float** out_depth, int* out_width, int* out_height);

    ovgl_result_t ovgl_get_stats(ovgl_renderer_t* renderer, ovgl_stats_t* out_stats);
    ovx_string_t ovgl_get_last_error(void);

#ifdef __cplusplus
} /* extern "C" */
#endif
