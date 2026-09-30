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

/* C ABI and per-frame orchestration for the OpenGL renderer over OVStage.
 *
 * attach_ovstage stores the stage; render_frame rebuilds the GL Scene from OVStage
 * (ovgl_scene_build), draws it with the module-private GLES rasterizer, and reads
 * the framebuffer back into the caller's host RGBA8 buffer.
 *
 * Per-mesh transform/color reach the shader through the gl MeshBlock UBO
 * (binding=1, std140, ROW-major: mvp/model/color/ptex) via gpu_alloc_mesh_buffer +
 * gpu_begin_mesh_writes + gpu_cmd_bind_mesh_data -- NOT push constants (those only
 * carry eye_pos). All matrices here are therefore row-major math matrices
 * (A[r][c] = a[r*4+c]); USD worldMatrix is also row-major (p_world = p_obj * W), so
 * the column-vector model M (world_col = M * obj_col) is W^T, i.e. M[r][c] = W[c][r].
 */
#include "Ovgl.h"

#include "AssetReader.hpp"
#include "MeshClassification.hpp"
#include "Scene.h"
#include "VertexBuffer.hpp"
#include "gl/EglHeadless.h"
#include "gl/GlApi.h"
#include "gl/GlLog.h"
#include "gl/Gpu.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "gl/ShadersGles.h" /* k_pbr_vert_gles / k_pbr_frag_gles (others unused) */
#pragma GCC diagnostic pop

#include <algorithm>
#include <cfloat> /* FLT_MAX — shadow-atlas light-space extent fitting */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace
{
thread_local std::string g_err;

/* egl_headless owns process-global display/refcount state on EGL platforms.
 * Serialize the complete create/destroy operations so distinct renderers may
 * cross lifecycle boundaries on different threads without losing a reference
 * or terminating a peer's display. The CGL implementation benefits from the
 * same conservative lifecycle ordering. */
std::mutex g_owned_context_lifecycle_mutex;
/* Scene.cpp uses process-global vectors as build scratch for materials,
 * decoded texture pixels, and lights.  Keep one build plus its renderer-local
 * snapshot atomic so concurrent renderers cannot observe each other's scratch
 * state while the bridge is being populated. */
std::mutex g_scene_build_mutex;

EglHeadless* createOwnedContext(int width, int height)
{
    std::lock_guard<std::mutex> lock(g_owned_context_lifecycle_mutex);
    return egl_headless_create(width, height);
}

void destroyOwnedContext(EglHeadless* context)
{
    std::lock_guard<std::mutex> lock(g_owned_context_lifecycle_mutex);
    egl_headless_destroy(context);
}

/* Closes the bracket egl_headless_make_current / egl_headless_create open when
 * they suspend a host GLX context (egl_headless.h). Scope-guarded rather than
 * hand-placed: the owned render path has ~15 early returns, and a host whose UI
 * context is not handed back stops drawing — the ovui viewer would trade a
 * black viewport for a frozen window. A no-op when nothing was suspended, i.e.
 * on every headless lane. */
struct ForeignGlGuard
{
    ForeignGlGuard() = default;
    ~ForeignGlGuard()
    {
        egl_headless_restore_foreign();
    }
    ForeignGlGuard(const ForeignGlGuard&) = delete;
    ForeignGlGuard& operator=(const ForeignGlGuard&) = delete;
};

/* --- optional per-stage profiling (OVGL_PROFILE=1) ---------------------- */
static const bool g_kProfilingEnabled = (std::getenv("OVGL_PROFILE") != nullptr);
static inline double nowMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/* Row-major 4x4 math matrices (A[r][c] == a[r*4+c]); used directly by the
 * MeshBlock UBO's `row_major` mat4 members. */
struct M4
{
    float m[16];
};

M4 mul(const M4& a, const M4& b)
{ /* C = A*B, row-major */
    M4 c{};
    for (int r = 0; r < 4; r++)
        for (int col = 0; col < 4; col++)
        {
            float s = 0;
            for (int k = 0; k < 4; k++)
                s += a.m[r * 4 + k] * b.m[k * 4 + col];
            c.m[r * 4 + col] = s;
        }
    return c;
}

M4 perspective(float fovy, float aspect, float znear, float zfar)
{ /* row-major */
    float f = 1.0f / std::tan(fovy * 0.5f);
    M4 m{};
    m.m[0] = f / aspect;
    m.m[5] = f;
    m.m[10] = (zfar + znear) / (znear - zfar);
    m.m[11] = (2.0f * zfar * znear) / (znear - zfar);
    m.m[14] = -1.0f;
    return m;
}

/* The renderer's fixed view frustum, in stage units. Shared by the scene
 * pass's projection and the Depth-AOV linearization, which inverts exactly
 * that projection (documented in ovgl.h::ovgl_set_depth_capture). */
constexpr float g_kOvglNearPlane = 0.02f;
constexpr float g_kOvglFarPlane = 10000.0f;

/* Per-frame clip planes fitted to the camera eye + scene AABB. A fixed
 * 0.02–10000 span (near/far ratio ~5e5) starves the 24-bit depth buffer and
 * z-fights coplanar surfaces (e.g. Kitchen_set walls/floor). Fitting the planes
 * to the scene's bounding sphere from the eye pushes the near plane out as far
 * as it can go without clipping the nearest geometry and pulls the far plane in
 * to the back of the scene, holding the ratio near ~1e1–1e4. The depth AOV
 * inverts the SAME planes, so linearized distances stay correct (and gain
 * precision). Degenerate/empty bounds fall back to the fixed span. */
static void computeClipPlanes(const double eye[3], const float bmin[3], const float bmax[3], float* out_znear, float* out_zfar)
{
    const double cx = 0.5 * ((double)bmin[0] + bmax[0]);
    const double cy = 0.5 * ((double)bmin[1] + bmax[1]);
    const double cz = 0.5 * ((double)bmin[2] + bmax[2]);
    const double dx = (double)bmax[0] - bmin[0];
    const double dy = (double)bmax[1] - bmin[1];
    const double dz = (double)bmax[2] - bmin[2];
    /* An empty scene leaves the AABB inverted (min=+FLT_MAX, max=-FLT_MAX);
     * the squared extents still produce a huge FINITE positive R, which would
     * push the near plane out ~1e34 and clip everything — including the grid
     * overlay over an empty stage. Inverted bounds are degenerate: fall back. */
    if (!(dx >= 0.0 && dy >= 0.0 && dz >= 0.0))
    {
        *out_znear = g_kOvglNearPlane;
        *out_zfar = g_kOvglFarPlane;
        return;
    }
    const double R = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz); /* bounding-sphere radius */
    const double ex = eye[0] - cx, ey = eye[1] - cy, ez = eye[2] - cz;
    const double dc = std::sqrt(ex * ex + ey * ey + ez * ez); /* eye -> scene center */
    if (!(R > 0.0) || !std::isfinite(R) || !std::isfinite(dc))
    {
        *out_znear = g_kOvglNearPlane;
        *out_zfar = g_kOvglFarPlane;
        return;
    }
    double zf = dc + R * 1.05 + 1e-4; /* just past the far side of the scene */
    double zn = (dc - R) * 0.5; /* half-way to the nearest geometry (margin) */
    const double zn_floor = zf * 1e-4; /* bound the ratio for 24-bit depth precision */
    if (!(zn > zn_floor))
        zn = zn_floor; /* also covers eye-inside-scene (dc < R) */
    if (!(zf > zn))
        zf = zn * 1.001 + 1e-3;
    *out_znear = (float)zn;
    *out_zfar = (float)zf;
}

M4 ortho(float l, float r, float b, float t, float n, float f)
{ /* row-major, GL clip */
    M4 m{};
    m.m[0] = 2.0f / (r - l);
    m.m[3] = -(r + l) / (r - l);
    m.m[5] = 2.0f / (t - b);
    m.m[7] = -(t + b) / (t - b);
    m.m[10] = -2.0f / (f - n);
    m.m[11] = -(f + n) / (f - n);
    m.m[15] = 1.0f;
    return m;
}

M4 lookAt(const double e[3], const double t[3], const double up[3])
{ /* row-major view */
    double fx = t[0] - e[0], fy = t[1] - e[1], fz = t[2] - e[2];
    double fn = std::sqrt(fx * fx + fy * fy + fz * fz);
    if (fn < 1e-12)
        fn = 1;
    fx /= fn;
    fy /= fn;
    fz /= fn;
    double sx = fy * up[2] - fz * up[1], sy = fz * up[0] - fx * up[2], sz = fx * up[1] - fy * up[0];
    double sn = std::sqrt(sx * sx + sy * sy + sz * sz);
    if (sn < 1e-12)
        sn = 1;
    sx /= sn;
    sy /= sn;
    sz /= sn;
    double ux = sy * fz - sz * fy, uy = sz * fx - sx * fz, uz = sx * fy - sy * fx;
    M4 m{};
    m.m[0] = (float)sx;
    m.m[1] = (float)sy;
    m.m[2] = (float)sz;
    m.m[3] = (float)-(sx * e[0] + sy * e[1] + sz * e[2]);
    m.m[4] = (float)ux;
    m.m[5] = (float)uy;
    m.m[6] = (float)uz;
    m.m[7] = (float)-(ux * e[0] + uy * e[1] + uz * e[2]);
    m.m[8] = (float)-fx;
    m.m[9] = (float)-fy;
    m.m[10] = (float)-fz;
    m.m[11] = (float)(fx * e[0] + fy * e[1] + fz * e[2]);
    m.m[15] = 1.0f;
    return m;
}

/* USD row-major worldMatrix W (p_world = p_obj * W) -> column-vector model M
 * (world_col = M * obj_col) = W^T, stored row-major: M[r][c] = W[c][r]. */
M4 modelFromWorld(const double w[16])
{
    M4 m{};
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            m.m[r * 4 + c] = (float)w[c * 4 + r];
    return m;
}

/* Per-mesh winding + face culling. `backface_cull` is the renderer's
 * OVGL_BACKFACE_CULL opt-in (default 0 = draw both sides); see the field of
 * that name in ovgl_renderer for why the USD-permitted optimization is not the
 * default here. */
void applyMeshSidedness(const SceneMesh& m, int backface_cull)
{
    const double* w = m.world_xform;
    const double determinant =
        w[0] * (w[5] * w[10] - w[6] * w[9]) - w[1] * (w[4] * w[10] - w[6] * w[8]) + w[2] * (w[4] * w[9] - w[5] * w[8]);
    /* A negative-determinant transform reverses projected winding. USD's
     * orientation token describes the authored topology, so toggle it for
     * mirrored world transforms rather than culling a valid reflected mesh.
     * Set unconditionally now, not just on the culling path: glFrontFace also
     * decides gl_FrontFacing, and leaving it at whatever the previous mesh
     * happened to want made the GL state depend on draw order. */
    const bool front_face_cw = (m.front_face_cw != 0) ^ (determinant < 0.0);
    glFrontFace(front_face_cw ? GL_CW : GL_CCW);
    if (!backface_cull || m.double_sided)
    {
        glDisable(GL_CULL_FACE);
        return;
    }
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
}

/* General 4x4 inverse. Row-major in/out: the classic column-major glu algorithm
 * applied to a row-major array yields the row-major inverse (it inverts Aᵀ and
 * stores the result transposed, which is A⁻¹ row-major). Identity if singular.
 * Used to feed gpu_draw_env_background's view_inv / proj_inv (also row-major). */
M4 inverse(const M4& A)
{
    const float* m = A.m;
    M4 R;
    float* o = R.m;
    o[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] +
           m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    o[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] -
           m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    o[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] +
           m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    o[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] -
            m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    o[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] -
           m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    o[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] +
           m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    o[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] -
           m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    o[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] +
            m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    o[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] -
           m[13] * m[3] * m[6];
    o[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] -
           m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    o[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] +
            m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    o[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] -
            m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    o[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] +
           m[9] * m[3] * m[6];
    o[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] -
           m[8] * m[3] * m[6];
    o[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];
    o[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];
    float det = m[0] * o[0] + m[1] * o[4] + m[2] * o[8] + m[3] * o[12];
    if (det == 0.0f)
    {
        M4 I{};
        I.m[0] = I.m[5] = I.m[10] = I.m[15] = 1.0f;
        return I;
    }
    det = 1.0f / det;
    for (int i = 0; i < 16; i++)
        o[i] *= det;
    return R;
}

/* The PBR material pipeline uses gl's k_pbr_vert_gles / k_pbr_frag_gles
 * (full UsdPreviewSurface BRDF + fallback lighting); ovgl defines no shaders. */

/* Depth-only shadow pass: `u_shadowPassVP * mesh.model`, NOT the MeshBlock's
 * own mvp. The pre-atlas pass reused mvp, which meant the whole MeshBlock had
 * to be re-uploaded with light-space matrices before the pass and re-uploaded
 * again with camera-space ones after it. With the atlas that would be one
 * extra full UBO rewrite PER SHADOW-CASTING LIGHT per frame. Reading `model`
 * instead lets every tile share the color pass's single upload and change
 * exactly one uniform -- see gpu_shadow_begin's pass-VP push. */
#if defined(NUSD_DESKTOP_GL)
constexpr const char* g_kShadowVertexShader =
    "#version 410\n"
    "layout(std140, row_major) uniform MeshBlock {\n"
    "    mat4 mvp; mat4 model; vec4 color; uvec4 ptex;\n"
    "} mesh;\n"
    "uniform mat4 u_shadowPassVP;\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "void main(){ gl_Position = u_shadowPassVP * mesh.model * vec4(inPosition, 1.0); }\n";
constexpr const char* g_kShadowFragmentShader =
    "#version 410\n"
    "void main(){}\n";
#else
constexpr const char* g_kShadowVertexShader =
    "#version 310 es\n"
    "precision highp float;\n"
    "layout(std140, binding = 1, row_major) uniform MeshBlock {\n"
    "    mat4 mvp; mat4 model; vec4 color; uvec4 ptex;\n"
    "} mesh;\n"
    "uniform mat4 u_shadowPassVP;\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "void main(){ gl_Position = u_shadowPassVP * mesh.model * vec4(inPosition, 1.0); }\n";
constexpr const char* g_kShadowFragmentShader =
    "#version 310 es\n"
    "precision highp float;\n"
    "void main(){}\n";
#endif

/* Ambient-occlusion prepass: window depth + world-space normal, camera view.
 *
 * SSAO has to read the depth of the frame it is shading, and during the color
 * pass that depth texture is the BOUND depth attachment -- sampling it there is
 * a feedback loop, which GL leaves undefined. So the geometry is rasterized
 * once more, first, into a target the color pass does not touch. That is the
 * whole reason this pass exists; it is not a Z-prepass optimization and it
 * deliberately does NOT share render_fbo's depth (see ensureAoTargets).
 *
 * It reads the MeshBlock's own `mvp` -- unlike the shadow pass above, which
 * needs a light-space VP -- so it rasterizes with exactly the matrix the color
 * pass will use and their depths agree bit for bit.
 *
 * The normal math is copied verbatim from k_pbr_vert_gles rather than
 * simplified: an SSAO hemisphere oriented off a normal that disagrees with the
 * shading normal self-occludes, and the disagreement would show up precisely on
 * the non-uniformly-scaled instanced meshes where it is hardest to attribute. */
#if defined(NUSD_DESKTOP_GL)
constexpr const char* g_kAmbientOcclusionPrepassVertexShader =
    "#version 410\n"
    "layout(std140, row_major) uniform MeshBlock {\n"
    "    mat4 mvp; mat4 model; vec4 color; uvec4 ptex;\n"
    "} mesh;\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec3 inNormal;\n"
    "out vec3 vNormal;\n"
    "void main(){\n"
    "    gl_Position = mesh.mvp * vec4(inPosition, 1.0);\n"
    "    mat3 normalModel = mat3(mesh.model);\n"
    "    vec3 n = normalModel * inNormal;\n"
    "    if (abs(determinant(normalModel)) > 1e-12)\n"
    "        n = transpose(inverse(normalModel)) * inNormal;\n"
    "    if (dot(n, n) < 1e-20) n = inNormal;\n"
    "    vNormal = normalize(n);\n"
    "}\n";
constexpr const char* g_kAmbientOcclusionPrepassFragmentShader =
    "#version 410\n"
    "in vec3 vNormal;\n"
    "out vec4 outNormal;\n"
    "void main(){ outNormal = vec4(normalize(vNormal) * 0.5 + 0.5, 1.0); }\n";
#else
constexpr const char* g_kAmbientOcclusionPrepassVertexShader =
    "#version 310 es\n"
    "precision highp float;\n"
    "layout(std140, binding = 1, row_major) uniform MeshBlock {\n"
    "    mat4 mvp; mat4 model; vec4 color; uvec4 ptex;\n"
    "} mesh;\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec3 inNormal;\n"
    "out vec3 vNormal;\n"
    "void main(){\n"
    "    gl_Position = mesh.mvp * vec4(inPosition, 1.0);\n"
    "    mat3 normalModel = mat3(mesh.model);\n"
    "    vec3 n = normalModel * inNormal;\n"
    "    if (abs(determinant(normalModel)) > 1e-12)\n"
    "        n = transpose(inverse(normalModel)) * inNormal;\n"
    "    if (dot(n, n) < 1e-20) n = inNormal;\n"
    "    vNormal = normalize(n);\n"
    "}\n";
constexpr const char* g_kAmbientOcclusionPrepassFragmentShader =
    "#version 310 es\n"
    "precision highp float;\n"
    "in vec3 vNormal;\n"
    "out vec4 outNormal;\n"
    "void main(){ outNormal = vec4(normalize(vNormal) * 0.5 + 0.5, 1.0); }\n";
#endif

/* Grid + axes overlay (ovgl.h::ovgl_set_grid_overlay): one ground-plane quad
 * with a procedural fwidth-anti-aliased line pattern. The up-axis-dependent
 * plane mapping and axis-line colors are baked into the VERTEX DATA (world
 * position + in-plane coordinates + the two in-plane axis colors), so the
 * shader itself is up-axis agnostic and needs only the per-frame u_view /
 * u_proj (gpu_cmd_set_view_proj) and u_eyePos (gpu_cmd_set_eye_pos) uniforms
 * gl already knows how to set. `vPlane` interpolates the fragment's
 * ground-plane coordinates in STAGE UNITS; fwidth(vPlane) is therefore the
 * world-units-per-pixel footprint that turns world distances into pixel
 * distances for ~1px anti-aliased lines at any zoom.
 *
 * The line at plane v == 0 runs ALONG the u direction and is colored by u's
 * world axis (vAxisColorU); symmetrically for u == 0 / vAxisColorV. Priority
 * when patterns overlap: axis > major (10-unit) > minor (1-unit). Minor
 * lines fade out once a 1-unit cell nears the pixel footprint (moire guard);
 * everything fades quadratically with eye distance like the stock viewport's
 * horizon falloff. Fragments that end up fully transparent are discarded so
 * the overlay writes nothing outside its lines. */
#if defined(NUSD_DESKTOP_GL)
#    define OVGL_GRID_GLSL_HEADER "#version 410\n"
#else
#    define OVGL_GRID_GLSL_HEADER "#version 310 es\nprecision highp float;\n"
#endif
constexpr const char* g_kGridVertexShader = OVGL_GRID_GLSL_HEADER
    "uniform mat4 u_view;\n"
    "uniform mat4 u_proj;\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec2 inPlane;\n"
    "layout(location = 2) in vec3 inAxisColorU;\n"
    "layout(location = 3) in vec3 inAxisColorV;\n"
    "out vec2 vPlane;\n"
    "out vec3 vWorld;\n"
    "out vec3 vAxisColorU;\n"
    "out vec3 vAxisColorV;\n"
    "void main() {\n"
    "    vPlane = inPlane;\n"
    "    vWorld = inPosition;\n"
    "    vAxisColorU = inAxisColorU;\n"
    "    vAxisColorV = inAxisColorV;\n"
    "    gl_Position = u_proj * (u_view * vec4(inPosition, 1.0));\n"
    "}\n";
constexpr const char* g_kGridFragmentShader = OVGL_GRID_GLSL_HEADER
    "uniform vec3 u_eyePos;\n"
    "in vec2 vPlane;\n"
    "in vec3 vWorld;\n"
    "in vec3 vAxisColorU;\n"
    "in vec3 vAxisColorV;\n"
    "layout(location = 0) out vec4 outColor;\n"
    "float lineCoverage(vec2 plane, float spacing, vec2 fw) {\n"
    "    vec2 dist = abs(fract(plane / spacing - 0.5) - 0.5) * spacing / fw;\n"
    "    return 1.0 - smoothstep(0.45, 1.0, min(dist.x, dist.y));\n"
    "}\n"
    "void main() {\n"
    "    vec2 fw = max(fwidth(vPlane), vec2(1e-6));\n"
    "    /* Minor 1-unit lines vanish as their cells shrink toward the pixel\n"
    "     * footprint; major 10-unit lines carry the pattern from there. */\n"
    "    float minorVis = 1.0 - smoothstep(0.50, 0.95, max(fw.x, fw.y));\n"
    "    float alpha = lineCoverage(vPlane, 1.0, fw) * 0.52 * minorVis;\n"
    "    vec3 color = vec3(0.40);\n"
    "    float major = lineCoverage(vPlane, 10.0, fw) * 0.72;\n"
    "    if (major > alpha) { alpha = major; color = vec3(0.56); }\n"
    "    /* In-plane origin axes, slightly brighter and a touch wider. */\n"
    "    float axisU = (1.0 - smoothstep(0.5, 1.5, abs(vPlane.y) / fw.y)) * 0.9;\n"
    "    float axisV = (1.0 - smoothstep(0.5, 1.5, abs(vPlane.x) / fw.x)) * 0.9;\n"
    "    if (axisU > alpha) { alpha = axisU; color = vAxisColorU; }\n"
    "    if (axisV > alpha) { alpha = axisV; color = vAxisColorV; }\n"
    "    float fade = clamp(1.0 - length(vWorld - u_eyePos) / 200.0, 0.0, 1.0);\n"
    "    alpha *= fade;\n"
    "    if (alpha < 0.004) discard;\n"
    "    outColor = vec4(color, alpha);\n"
    "}\n";

/* Grid quad geometry: ±g_kGridExtent stage units in the up-axis' ground plane,
 * through the origin. 6 vertices × (pos3 | plane2 | axisColorU3 | axisColorV3)
 * = 11 floats, stride 44 bytes. */
constexpr float g_kGridExtent = 100.0f;
constexpr int g_kGridVertexFloats = 11;
constexpr int g_kGridVertexStride = g_kGridVertexFloats * (int)sizeof(float);
constexpr int g_kGridVertexCount = 6;

void buildGridQuad(int up_axis, float out[g_kGridVertexCount * g_kGridVertexFloats])
{
    static const float s_kAxisColor[3][3] = {
        { 0.85f, 0.30f, 0.30f }, /* X — red   */
        { 0.30f, 0.78f, 0.32f }, /* Y — green */
        { 0.32f, 0.48f, 0.92f }, /* Z — blue  */
    };
    /* World-axis indices of the two in-plane directions (u, v) per up axis.
     * ovgl_scene emits 1 (Y-up) or 2 (Z-up, default); 0 (X-up) is scene.h's
     * remaining enumerant, mapped to the YZ plane for completeness. */
    int axis_u = 0, axis_v = 1, axis_up = 2; /* Z-up: XY plane */
    if (up_axis == 1)
    {
        axis_u = 0;
        axis_v = 2;
        axis_up = 1;
    } /* Y-up: XZ */
    else if (up_axis == 0)
    {
        axis_u = 1;
        axis_v = 2;
        axis_up = 0;
    } /* X-up: YZ */
    static const float s_kCorners[g_kGridVertexCount][2] = {
        { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f },
    };
    for (int i = 0; i < g_kGridVertexCount; i++)
    {
        float* v = out + i * g_kGridVertexFloats;
        const float u = s_kCorners[i][0] * g_kGridExtent;
        const float w = s_kCorners[i][1] * g_kGridExtent;
        v[axis_u] = u;
        v[axis_v] = w;
        v[axis_up] = 0.0f;
        v[3] = u;
        v[4] = w;
        for (int c = 0; c < 3; c++)
        {
            v[5 + c] = s_kAxisColor[axis_u][c];
            v[8 + c] = s_kAxisColor[axis_v][c];
        }
    }
}
} // namespace

struct ovgl_renderer
{
    ovgl_renderer_desc_t desc{};
    ovstage_instance_t* stage = nullptr;
    ovgl_camera_t cam{};
    bool cam_valid = false;
    int pull_paused = 0;
    int ss = 2; /* supersampling factor: render ss*W x ss*H, box-downsample */
    ovstage_ordinal_t last_ordinal = 0;
    ovstage_ordinal_t last_shadow_ordinal = (ovstage_ordinal_t)-1; /* shadow map is camera-independent */
    bool first_render = true;
    std::string env_path; /* host IBL override HDR ("" = none) */
    float env_intensity = 1.0f;
    bool env_loaded = false;
    bool env_failed = false;
    bool env_dirty = false;
    /* The environment actually loaded on the GPU right now, so a rebuilt scene
     * whose DomeLight changed reloads and one whose dome did not keeps its
     * textures. Source: 0 = none, 1 = host ovgl_set_environment, 2 = an
     * authored UsdLuxDomeLight. See applyEnvironment(). */
    int env_source = 0;
    /* Initialized to the request applyEnvironment() computes for "no
     * environment", so a dome-free scene never even enters that branch: the
     * no-environment GPU state (fallback rig on, neutral tone, no IBL) is
     * already what renderer creation establishes, and re-applying it would be
     * a pointless per-renderer no-op. */
    std::string env_applied_path;
    float env_applied_intensity = 1.0f;
    float env_applied_color[3] = { 1.0f, 1.0f, 1.0f };
    /* Every non-dome light prim in the scene: the ones ovgl can evaluate
     * (scene_lights) plus the ones it had to drop (Scene::nlights_unsupported).
     * Part of the reload key because env_owns_lighting is a function of exactly
     * this count, and a rebuild -- or the transform fast lane, which re-reads
     * the light set at ovgl_refresh_transforms():1341 -- can add or remove a
     * light without touching the dome, which flips ownership while the dome
     * request itself stays byte-identical. */
    int env_applied_nlights = 0;
    /* The environment is the scene's whole lighting rig, so the synthetic
     * MuJoCo headlight and hemisphere fill stand down (applyEnvironment).
     * Distinct from env_loaded: a dome that shares the stage with authored
     * lights is loaded and contributes its real radiance on top of a fill rig
     * that stays. */
    bool env_owns_lighting = false;

    EglHeadless* egl = nullptr;
    Gpu* gpu = nullptr;
    GpuPipeline pipe = nullptr;
    GpuPipeline shadow_pipe = nullptr; /* depth-only pass for shadow maps */
    /* Procedural MuJoCo ground checker for floor-named meshes with no resolved
     * appearance (MeshClassification.hpp::isFloorMesh; env
     * OVGL_FLOOR_CHECKER). Default OFF: libovrtx_ovgl exists to be a drop-in
     * for official ovrtx 0.4, official has no counterpart, and leaving it on
     * made any scene with an unmaterialed floor-named prim impossible to
     * pixel-compare -- measured 89% of the whole ovgl-vs-official frame-0
     * divergence on the spot orbit scene (MAE 52.16 -> 5.52, 86.6% -> 1.4% of
     * pixels differing by >32; `max` stays 207, that residual is the contact
     * shadow ovgl does not cast). Set OVGL_FLOOR_CHECKER=1 for the MuJoCo
     * look -- with it set, frames are bit-identical to the pre-switch build,
     * 12/12 by md5 on that same orbit. */
    int floor_checker = 0;
    /* Back-face culling of gprims with doubleSided=false (env
     * OVGL_BACKFACE_CULL; applyMeshSidedness). Default OFF -- i.e. ovgl
     * draws BOTH sides of every mesh, and doubleSided only ever mattered here
     * as a culling hint.
     *
     * UsdGeomGprim (gprim.h:170-190) does not require the cull: it says only
     * that "some renderers derive significant optimizations by considering
     * these surfaces to have only a single outward side", and defines
     * doubleSided=true as the instruction to DISABLE that optimization. So
     * culling on doubleSided=false is permitted, not mandated -- and neither
     * of the two reference implementations takes it. Official ovrtx 0.4 draws
     * the away-facing face, and so does OpenUSD's own Hydra Storm
     * (`usdrecord --renderer GL`), which the same paragraph promises "always
     * will" provide forward-facing normals on each side. 2 of 3 renderers
     * drew it and ovgl was the outlier, which for a library whose whole
     * purpose is to substitute for ovrtx 0.4 is the defect.
     *
     * Measured, 3-quad control (+Z single-sided, -Z single-sided, -Z
     * doubleSided; camera above): ovgl drew 2 of 3, missing exactly the
     * away-facing single-sided quad (0 green pixels of official's 1057), at
     * 2.8% coverage against official's 4.4%. On Kitchen_set, whose ceiling
     * `Arch_grp/Kitchen_1/Geom/Ceiling/pPlane5`+`pPlane20` are single quads
     * with a -Z normal that the orbit camera sees from behind, ovgl rendered
     * straight through the roof into the room: orbit frame_000 28.63% of
     * pixels differing from official, MAE 33.99.
     *
     * Lighting is already two-sided and needed no shader change: the PBR
     * fragment shader flips N when dot(N, V) < 0 (shaders_gles.h, "Two-sided
     * rendering"), which is exactly the forward-facing-normal-per-side
     * behavior gprim.h ascribes to the reference GL renderer.
     *
     * Set OVGL_BACKFACE_CULL=1 to take the optimization back on closed-mesh
     * content where the hidden back halves are pure fill-rate waste; =0 pins
     * it off. It is a fill-rate/parity trade, not a correctness knob, so the
     * parity-correct state is the default. */
    int backface_cull = 0;
    /* Grid + axes overlay (ovgl.h::ovgl_set_grid_overlay). Default OFF so
     * every existing offscreen capture stays bit-identical; resources are
     * created lazily on the first ENABLED frame and rebuilt when the scene's
     * up axis changes (the quad bakes the plane orientation). */
    int grid_overlay = 0;
    GpuPipeline grid_pipe = nullptr;
    GpuBuffer grid_vb = nullptr;
    int grid_vb_up_axis = -1;
    int w = 0, h = 0;
    int mesh_ubo_cap = 0;
    /* Offscreen render target (iw x ih) the meshes draw into + a 1x resolve target
     * (w x h) it is blit-downscaled into -- the GPU replacement for the per-frame
     * CPU box-downsample. Recreated only when dimensions change. */
    unsigned render_fbo = 0, render_color = 0, render_depth = 0;
    unsigned resolve_fbo = 0, resolve_color = 0; /* resolve_color is a GL_TEXTURE_2D (samplable) */
    int fbo_iw = 0, fbo_ih = 0, fbo_w = 0, fbo_h = 0;
    /* Ambient-occlusion prepass target (iw x ih): world-space normal in RGBA8 +
     * its OWN depth texture. Both are sampled by the SSAO pass while the color
     * pass is not bound, which is the point -- see g_kAmbientOcclusionPrepassVertexShader. */
    GpuPipeline ao_pipe = nullptr;
    unsigned ao_fbo = 0, ao_normal = 0, ao_depth = 0;
    int ao_iw = 0, ao_ih = 0;
    /* Master switch (env OVGL_AO, default ON). 0 skips the prepass entirely, so
     * the frame is produced by exactly the GL command stream a build without any
     * of this would produce -- the escape hatch if AO is ever implicated in a
     * parity regression. */
    int ao_enabled = 1;
    /* SSAO tuning, all env-overridable (OVGL_AO_RADIUS / _BIAS / _INTENSITY /
     * _STRENGTH) so parity can be swept without a rebuild.
     *
     * radius is in STAGE UNITS and is the one knob that has to suit the scene:
     * it is the distance within which geometry counts as occluding. 0.6 suits
     * the metres-per-unit robotics scenes this renderer exists for (the G1's
     * links are 0.05-0.2 m and the probe spheres are r=0.4); a scene authored
     * in centimetres wants 100x that.
     *
     * radius=0.6 / intensity=1.0 was CHOSEN on the probe set, against official,
     * because it lands probe_no_occluder at min 62 / mean 218.2 where official
     * is 64 / 219.4 -- that fixture has nothing to occlude against, so agreeing
     * there is the evidence the estimator is calibrated rather than merely
     * dark. intensity=2.0 reaches deeper crevices but takes that same fixture
     * to min 49, i.e. it buys contrast by inventing occlusion. */
    float ao_radius = 0.6f;
    float ao_bias = 0.02f;
    float ao_intensity = 1.0f;
    float ao_strength = 1.0f;
    /* Optional double-buffered async readback for the live GUI path (OVGL_ASYNC_READBACK):
     * glReadPixels packs into pbo[cur] (non-blocking) while the host copy maps pbo[prev] (the
     * PREVIOUS frame, already transferred) -> hides the GPU->CPU stall at a 1-frame latency.
     * Synchronous by default so the headless render_frame contract (this ordinal's pixels) holds. */
    unsigned pack_pbo[2] = { 0, 0 };
    int pack_cur = 0;
    bool pack_primed = false;
    int async_readback = -1; /* -1 unset; resolved from env once */
    /* Depth AOV capture (ovgl.h): while enabled, every successful headless
     * render_frame also reads the depth buffer back as linear image-plane
     * distance (stage units; +inf = no hit). have_depth describes the LAST
     * frame only; adopted-path renders invalidate it. */
    int depth_capture = 0;
    bool have_depth = false;
    int depth_w = 0, depth_h = 0;
    std::vector<float> depth_host;
    std::vector<unsigned char> depth_pack_staging; /* packed RGBA8 readback scratch */
    /* GL-context mode: -1 unset, 0 headless (ovgl owns an EGL pbuffer context, render_frame
     * reads back to a host buffer), 1 adopt (render_to_texture runs in the CALLER's already-
     * current GL context -- e.g. a Qt QOpenGLWidget -- and returns the resolve texture). */
    int gl_adopt = -1;
    uintptr_t adopted_context_id = 0; /* pinned on first successful adopt */
    bool overlay_initialized = false;

    Scene scene{};
    /* ovgl_scene's auxiliary accessors expose build-scratch storage.  Snapshot
     * every payload into the renderer immediately after a successful build;
     * cached frames must never consult whichever scene another renderer built
     * most recently.  Texture descriptors point into scene_texture_pixels. */
    std::vector<GpuMaterialParams> scene_materials;
    std::vector<std::vector<unsigned char>> scene_texture_pixels;
    std::vector<GpuTextureData> scene_textures;
    std::vector<GpuLight> scene_lights;
    bool topo_built = false; /* geometry + GL buffers cached */
    /* Clip planes actually used by the LAST render's projection, fitted per
     * frame to the camera + scene bounds (computeClipPlanes). The depth-AOV
     * linearization must invert the SAME projection, so it reads these rather
     * than the kOvgl*Plane fallbacks. */
    float cur_znear = g_kOvglNearPlane;
    float cur_zfar = g_kOvglFarPlane;
    /* Set by ovgl_refresh_materials after a CPU-side re-resolution: the next
     * render_frame replaces the GPU material state (gpu_replace_materials)
     * on the render thread before drawing. Full rebuilds clear it — they
     * re-upload everything anyway. */
    bool materials_dirty = false;
    /* The cache is the sole owner of mesh GL objects. Draw-index arrays below are
     * non-owning views and are therefore safe to discard when a scene is replaced.
     *
     * Entries retain every source byte that can affect the uploaded vertex/index
     * payload. Reuse requires exact byte equality, not a sampled hash: normals-only
     * and bounds-preserving edits cannot go stale. A vector (rather than a path-keyed
     * map) permits unnamed meshes and multiple different payloads with the same path.
     * Identical same-path entries, including PointInstancer clones, share one owner. */
    struct GlMeshEntry
    {
        GpuBuffer vb = nullptr;
        GpuBuffer ib = nullptr;
        unsigned vao = 0;
        std::string path;
        std::vector<float> positions;
        std::vector<float> normals;
        std::vector<float> colors;
        std::vector<float> texcoords;
        std::vector<uint32_t> indices;
        bool has_normals = false;
        bool has_colors = false;
        bool has_texcoords = false;
        bool used = false;
    };
    std::vector<GlMeshEntry> gl_cache;

    std::vector<GpuBuffer> mesh_vb, mesh_ib; /* per-mesh, uploaded once */
    std::vector<uint8_t> mesh_is_floor; /* cached ground flag (path test, once) */
    std::vector<unsigned> mesh_vao; /* per-mesh VAO: baked attribs (one bind/draw) */
    size_t mesh_count = 0, tri_count = 0;
    uint64_t structure_gen = 0;
};

static bool finiteOrderedBounds(const float lower[3], const float upper[3])
{
    for (int component = 0; component < 3; ++component)
    {
        if (!std::isfinite(lower[component]) || !std::isfinite(upper[component]) || lower[component] > upper[component])
        {
            return false;
        }
    }
    return true;
}

/* ---- Shadow-atlas frustum fitting ----
 *
 * The single most important number in a shadow map is the world size of one
 * texel, and ovgl used to derive it from the WHOLE scene: a symmetric ortho
 * cube of half-extent 1.3 * the scene's bounding-sphere radius. That is fine
 * for a tabletop and catastrophic for a locomotion scene, because those author
 * a ground plane far larger than anything on it. Measured on this bench,
 * spot_view.usda + render_orbit's OrbitSun:
 *
 *   scene bounds     = the 200 x 200 m `def Plane "GroundCollider"`
 *   -> radius 141.4 m, ortho half-extent 183.8 m, span 367.7 m
 *   -> 2048 map      = 179 mm per texel, and Spot's whole body is ~1.1 m
 *   -> ortho depth   = [0.01, 565.7], so the fixed 0.0065 window-depth bias
 *                      is 3.68 METRES of world depth -- five times the
 *                      robot's height. The map was rendered every frame and
 *                      then biased out of existence.
 *
 * Both failures are the same root cause and both are fixed by fitting to the
 * CASTERS rather than to the scene. Note the fit is exact, not a heuristic
 * margin: for a directional light the shadow of a caster travels along the
 * light-space z axis, so it lands inside that caster's light-space x/y box by
 * construction, at any obliquity. Receivers outside the box are lit, which is
 * right -- nothing casts over them. The depth range still spans the whole
 * scene so a receiver behind the casters is testable, and the bias is derived
 * from the tile's own texel footprint instead of being a constant that means a
 * different world distance in every scene. */

/* Meshes that are environment-scale relative to the scene's median mesh are
 * receivers, not the thing the map should resolve. Rather than a name test
 * (ovgl already has one of those, and divergence #3 is what it cost), rank by
 * AABB diagonal and drop the outliers: a ground plane is 283 m of diagonal
 * against a robot link's 0.2 m, a factor of 1400. g_kShadowCoreRatio is
 * deliberately loose so a room's WALLS -- a few times the props, not a
 * thousand -- stay in the fit and keep casting. */
static constexpr float g_kShadowCoreRatio = 12.0f;
/* Then re-admit context around the casters, bounded: the fitted box is grown
 * to this multiple of its own half-extent before being clipped back to the
 * real scene bounds, so a modest ground still receives beyond the robot's
 * silhouette without a 200 m plane dragging the extent with it. */
static constexpr float g_kShadowContextGrowth = 3.0f;
/* Depth bias, in TEXELS of the tile's own world footprint. Scale-free by
 * construction: 2 texels is 2 texels whether a texel is 5 mm or 5 m. */
static constexpr float g_kShadowBiasTexels = 2.0f;

/* Fit one shadow-casting light's view-projection.
 *   core_lo/hi  the caster box the map should resolve (world AABB)
 *   scene_lo/hi the full drawable bounds, used only to extend the depth range
 *   tile_px     the atlas tile edge, for the texel-derived bias
 * Returns false for light kinds with no supported map (Sphere: needs a cube
 * or dual-paraboloid map, not one frustum). */
static bool buildShadowLightVp(const GpuLight& light,
                               const float core_lo[3],
                               const float core_hi[3],
                               const float scene_lo[3],
                               const float scene_hi[3],
                               int tile_px,
                               M4* out_vp,
                               float* out_bias)
{
    if (!out_vp || !out_bias)
        return false;
    const bool distant = (light.kind == 1);
    const bool rect = (light.kind == 0);
    if (!distant && !rect)
        return false;

    double fwd[3] = { light.normal[0], light.normal[1], light.normal[2] };
    double fn = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
    if (!(fn > 1e-8))
    {
        fwd[0] = 0.0;
        fwd[1] = 0.0;
        fwd[2] = -1.0;
        fn = 1.0;
    }
    fwd[0] /= fn;
    fwd[1] /= fn;
    fwd[2] /= fn;

    double up[3] = { 0.0, 0.0, 1.0 };
    if (std::fabs(fwd[2]) > 0.95)
    {
        up[0] = 0.0;
        up[1] = 1.0;
        up[2] = 0.0;
    }

    /* Grow the caster box by g_kShadowContextGrowth about its own center, then
     * clip to the scene: context without the scene's own extent. */
    float lo[3], hi[3];
    for (int a = 0; a < 3; ++a)
    {
        const float c = 0.5f * (core_lo[a] + core_hi[a]);
        const float h = 0.5f * (core_hi[a] - core_lo[a]) * g_kShadowContextGrowth;
        lo[a] = std::max(c - h, scene_lo[a]);
        hi[a] = std::min(c + h, scene_hi[a]);
        if (!(hi[a] > lo[a]))
        {
            lo[a] = c - 1e-3f;
            hi[a] = c + 1e-3f;
        }
    }

    const double cx = 0.5 * ((double)lo[0] + hi[0]);
    const double cy = 0.5 * ((double)lo[1] + hi[1]);
    const double cz = 0.5 * ((double)lo[2] + hi[2]);
    const double sdx = (double)scene_hi[0] - scene_lo[0];
    const double sdy = (double)scene_hi[1] - scene_lo[1];
    const double sdz = (double)scene_hi[2] - scene_lo[2];
    double scene_diag = std::sqrt(sdx * sdx + sdy * sdy + sdz * sdz);
    if (!(scene_diag > 1e-5))
        scene_diag = 1.0;

    double eye[3], tgt[3] = { cx, cy, cz };
    if (distant)
    {
        /* Park the light origin outside the whole scene along -shine, so every
         * scene point is in front of the near plane no matter which corner. */
        eye[0] = cx - fwd[0] * scene_diag;
        eye[1] = cy - fwd[1] * scene_diag;
        eye[2] = cz - fwd[2] * scene_diag;
    }
    else
    {
        eye[0] = light.position[0];
        eye[1] = light.position[1];
        eye[2] = light.position[2];
    }
    const M4 view = lookAt(eye, tgt, up);

    /* Light-space extents. x/y come from the fitted box only; the depth range
     * spans the FULL scene so a receiver behind the casters still resolves. */
    float minx = FLT_MAX, maxx = -FLT_MAX, miny = FLT_MAX, maxy = -FLT_MAX;
    float near_d = FLT_MAX, far_d = -FLT_MAX; /* distances in front of the light */
    float max_tan = 0.0f;
    auto accumulate = [&](const float b_lo[3], const float b_hi[3], bool lateral)
    {
        for (int i = 0; i < 8; ++i)
        {
            const float p[3] = {
                (i & 1) ? b_hi[0] : b_lo[0],
                (i & 2) ? b_hi[1] : b_lo[1],
                (i & 4) ? b_hi[2] : b_lo[2],
            };
            const float qx = view.m[0] * p[0] + view.m[1] * p[1] + view.m[2] * p[2] + view.m[3];
            const float qy = view.m[4] * p[0] + view.m[5] * p[1] + view.m[6] * p[2] + view.m[7];
            const float qz = view.m[8] * p[0] + view.m[9] * p[1] + view.m[10] * p[2] + view.m[11];
            const float d = -qz; /* GL view space looks down -Z */
            if (d < near_d)
                near_d = d;
            if (d > far_d)
                far_d = d;
            if (!lateral)
                continue;
            if (qx < minx)
                minx = qx;
            if (qx > maxx)
                maxx = qx;
            if (qy < miny)
                miny = qy;
            if (qy > maxy)
                maxy = qy;
            if (d > 1e-4f)
            {
                const float t = std::max(std::fabs(qx), std::fabs(qy)) / d;
                if (t > max_tan)
                    max_tan = t;
            }
        }
    };
    accumulate(lo, hi, true);
    accumulate(scene_lo, scene_hi, false);
    if (!(maxx > minx) || !(maxy > miny) || !(far_d > near_d))
        return false;

    /* Square the lateral box so one texel is square, then a small margin. */
    float half = 0.5f * std::max(maxx - minx, maxy - miny);
    half *= 1.04f;
    if (!(half > 1e-6f))
        half = 1e-6f;
    const float mx = 0.5f * (minx + maxx), my = 0.5f * (miny + maxy);

    const float zm = std::max(0.01f * (far_d - near_d), 1e-3f);
    float zn = near_d - zm;
    const float zf = far_d + zm;
    if (!(zn > 0.0f))
        zn = std::max(zf * 1e-5f, 1e-4f);

    if (distant)
    {
        *out_vp = mul(ortho(mx - half, mx + half, my - half, my + half, zn, zf), view);
        /* World bias -> window depth. An ortho map's window depth is linear in
         * distance, so the conversion is exactly 1/(zf - zn). */
        const float texel_world = (2.0f * half) / (float)std::max(tile_px, 1);
        *out_bias = (g_kShadowBiasTexels * texel_world) / (zf - zn);
    }
    else
    {
        /* Symmetric perspective covering the fitted box from the light's own
         * position. Window depth is nonlinear here, so the world-bias
         * conversion above does not apply; use a small constant and let the
         * shader's NdotL term and the map's polygon offset do the rest. */
        float t = std::min(max_tan * 1.15f + 1e-3f, 57.0f);
        M4 proj{};
        proj.m[0] = 1.0f / t;
        proj.m[5] = 1.0f / t;
        proj.m[10] = -(zf + zn) / (zf - zn);
        proj.m[11] = -2.0f * zf * zn / (zf - zn);
        proj.m[14] = -1.0f;
        *out_vp = mul(proj, view);
        *out_bias = 0.0015f;
    }
    return std::isfinite(out_vp->m[0]) && std::isfinite(*out_bias);
}

/* Scene.nmeshes includes PointInstancer prototype storage that is deliberately
 * retained but never drawn.  Such a scene has sentinel aggregate bounds and
 * must not seed either public bounds or the light-space shadow matrix. */
static bool sceneHasDrawableBounds(const Scene& scene)
{
    if (scene.nmeshes <= 0 || !scene.meshes || !finiteOrderedBounds(scene.bounds_min, scene.bounds_max))
    {
        return false;
    }
    for (int index = 0; index < scene.nmeshes; ++index)
    {
        const SceneMesh& mesh = scene.meshes[index];
        if (mesh.visible && !mesh.is_proto_only && mesh.nvertices > 0 && mesh.nindices >= 3 && mesh.positions &&
            mesh.indices)
        {
            return true;
        }
    }
    return false;
}

/* Bound dimensions before any signed multiplication or GL allocation.  The
 * backend-wide portable ceiling plus aggregate limit prevents a legal-but-
 * hostile skinny or square request from committing unbounded color/depth
 * storage. */
constexpr int g_kMaxSupersampledDimension = 16384;
constexpr uint64_t g_kMaxSupersampledPixels = 64ull * 1024ull * 1024ull;

static bool computeRenderDimensions(const ovgl_renderer* r, int width, int height, int* out_width, int* out_height, int* out_ss)
{
    if (!r || width <= 0 || height <= 0)
    {
        g_err = "render dimensions must be positive";
        return false;
    }
    const int ss = r->ss > 0 ? r->ss : 1;
    if (width > std::numeric_limits<int>::max() / ss || height > std::numeric_limits<int>::max() / ss)
    {
        g_err = "supersampled render dimensions overflow";
        return false;
    }
    const int supersampled_width = width * ss;
    const int supersampled_height = height * ss;
    if (supersampled_width > g_kMaxSupersampledDimension || supersampled_height > g_kMaxSupersampledDimension)
    {
        g_err = "supersampled render dimensions exceed the portable limit";
        return false;
    }
    const uint64_t pixels = static_cast<uint64_t>(supersampled_width) * static_cast<uint64_t>(supersampled_height);
    if (pixels > g_kMaxSupersampledPixels)
    {
        g_err = "supersampled render target exceeds the portable pixel limit";
        return false;
    }
    if (out_width)
        *out_width = supersampled_width;
    if (out_height)
        *out_height = supersampled_height;
    if (out_ss)
        *out_ss = ss;
    return true;
}

static bool validateCamera(const ovgl_camera_t& camera)
{
    for (int component = 0; component < 3; ++component)
    {
        if (!std::isfinite(camera.eye[component]) || !std::isfinite(camera.target[component]) ||
            !std::isfinite(camera.up[component]))
        {
            g_err = "camera vectors must be finite";
            return false;
        }
    }
    const double forward[3] = {
        camera.target[0] - camera.eye[0],
        camera.target[1] - camera.eye[1],
        camera.target[2] - camera.eye[2],
    };
    const double forward_norm = std::hypot(std::hypot(forward[0], forward[1]), forward[2]);
    const double up_norm = std::hypot(std::hypot(camera.up[0], camera.up[1]), camera.up[2]);
    if (!std::isfinite(forward_norm) || !std::isfinite(up_norm) || forward_norm <= 1e-12 || up_norm <= 1e-12)
    {
        g_err = "camera eye/target direction and up vector must be non-zero";
        return false;
    }
    const double side[3] = {
        forward[1] * camera.up[2] - forward[2] * camera.up[1],
        forward[2] * camera.up[0] - forward[0] * camera.up[2],
        forward[0] * camera.up[1] - forward[1] * camera.up[0],
    };
    const double side_norm = std::hypot(std::hypot(side[0], side[1]), side[2]);
    if (!std::isfinite(side_norm) || side_norm <= 1e-12 * forward_norm * up_norm)
    {
        g_err = "camera up vector must not be parallel to the view direction";
        return false;
    }
    constexpr double pi = 3.14159265358979323846;
    if (!std::isfinite(camera.fov_y_rad) || camera.fov_y_rad <= 0.0 || camera.fov_y_rad >= pi)
    {
        g_err = "camera field of view must be finite and in (0, pi)";
        return false;
    }
    return true;
}

static bool snapshotSceneAuxiliaries(ovgl_renderer* r)
{
    const GpuMaterialParams* materials = nullptr;
    const GpuTextureData* textures = nullptr;
    const GpuLight* lights = nullptr;
    const int material_count = ovgl_scene_materials(&materials);
    const int texture_count = ovgl_scene_textures(&textures);
    const int light_count = ovgl_scene_lights(&lights);
    if (material_count < 0 || texture_count < 0 || light_count < 0 || (material_count > 0 && !materials) ||
        (texture_count > 0 && !textures) || (light_count > 0 && !lights))
    {
        g_err = "invalid ovgl scene auxiliary snapshot";
        return false;
    }

    try
    {
        std::vector<GpuMaterialParams> material_snapshot;
        std::vector<std::vector<unsigned char>> pixel_snapshot(static_cast<size_t>(texture_count));
        std::vector<GpuTextureData> texture_snapshot(static_cast<size_t>(texture_count));
        std::vector<GpuLight> light_snapshot;
        if (material_count > 0)
        {
            material_snapshot.assign(materials, materials + static_cast<size_t>(material_count));
        }
        if (light_count > 0)
        {
            light_snapshot.assign(lights, lights + static_cast<size_t>(light_count));
        }
        for (int index = 0; index < texture_count; ++index)
        {
            const GpuTextureData& source = textures[index];
            if (source.width <= 0 || source.height <= 0 || !source.pixels)
            {
                g_err = "invalid ovgl scene texture snapshot";
                return false;
            }
            const size_t width = static_cast<size_t>(source.width);
            const size_t height = static_cast<size_t>(source.height);
            if (width > std::numeric_limits<size_t>::max() / height ||
                width * height > std::numeric_limits<size_t>::max() / 4)
            {
                g_err = "ovgl scene texture snapshot size overflow";
                return false;
            }
            const size_t byte_count = width * height * 4;
            pixel_snapshot[static_cast<size_t>(index)].assign(source.pixels, source.pixels + byte_count);
            texture_snapshot[static_cast<size_t>(index)] = source;
            texture_snapshot[static_cast<size_t>(index)].pixels = pixel_snapshot[static_cast<size_t>(index)].data();
        }

        r->scene_materials.swap(material_snapshot);
        r->scene_texture_pixels.swap(pixel_snapshot);
        r->scene_textures.swap(texture_snapshot);
        r->scene_lights.swap(light_snapshot);
        return true;
    }
    catch (const std::bad_alloc&)
    {
        g_err = "allocating ovgl renderer scene auxiliaries failed";
        return false;
    }
}

static const char* glErrorName(GLenum err)
{
    switch (err)
    {
    case GL_INVALID_ENUM:
        return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
        return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
        return "GL_INVALID_OPERATION";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
        return "GL_INVALID_FRAMEBUFFER_OPERATION";
    case GL_OUT_OF_MEMORY:
        return "GL_OUT_OF_MEMORY";
    default:
        return "unknown";
    }
}

/* OpenGL commands do not return status.  Turn every error raised inside an OVGL
 * render boundary into the C API's explicit failure result instead of letting a
 * failed draw/blit look like a successful (but blank) frame. */
static bool glErrorsOk(const char* where)
{
    GLenum first = glGetError();
    if (first == GL_NO_ERROR)
        return true;
    int count = 1;
    while (glGetError() != GL_NO_ERROR)
        ++count;
    char msg[256];
    std::snprintf(msg, sizeof(msg), "%s: %s (0x%04x)%s", where, glErrorName(first), (unsigned)first,
                  count > 1 ? " (additional GL errors drained)" : "");
    g_err = msg;
    return false;
}

static void glDiscardErrors()
{
    while (glGetError() != GL_NO_ERROR)
    {
    }
}

static bool ensureGlFunctionsLoaded()
{
#if defined(_WIN32)
    if (!ovgl_gl_load())
    {
        g_err = "failed to load OpenGL functions for the current context";
        return false;
    }
#endif
    return true;
}

static bool glFramebufferComplete(GLenum target, const char* label)
{
    GLenum status = glCheckFramebufferStatus(target);
    if (status == GL_FRAMEBUFFER_COMPLETE)
        return true;
    char msg[192];
    std::snprintf(msg, sizeof(msg), "%s incomplete (0x%04x)", label, (unsigned)status);
    g_err = msg;
    return false;
}

static bool validateAdoptedContext(const ovgl_renderer_t* r, const char* operation)
{
    uintptr_t current = egl_headless_current_context_id();
    if (!current)
    {
        g_err = std::string(operation) + ": no current GL context";
        return false;
    }
    if (r->adopted_context_id && current != r->adopted_context_id)
    {
        g_err = std::string(operation) + ": current GL context does not match the renderer's adopted context";
        return false;
    }
    return true;
}

template <typename T>
static bool exactPayloadEquals(const std::vector<T>& stored, const T* source, size_t count)
{
    return stored.size() == count &&
           (count == 0 || (source && std::memcmp(stored.data(), source, count * sizeof(T)) == 0));
}

static bool cacheEntryMatches(const ovgl_renderer::GlMeshEntry& entry, const SceneMesh& mesh)
{
    const char* path = mesh.path ? mesh.path : "";
    if (entry.path != path || entry.has_normals != (mesh.normals != nullptr) ||
        entry.has_colors != (mesh.colors != nullptr) || entry.has_texcoords != (mesh.texcoords != nullptr))
    {
        return false;
    }
    const size_t vertex_count = static_cast<size_t>(mesh.nvertices);
    const size_t index_count = static_cast<size_t>(mesh.nindices);
    return exactPayloadEquals(entry.positions, mesh.positions, vertex_count * 3) &&
           exactPayloadEquals(entry.normals, mesh.normals, entry.has_normals ? vertex_count * 3 : 0) &&
           exactPayloadEquals(entry.colors, mesh.colors, entry.has_colors ? vertex_count * 3 : 0) &&
           exactPayloadEquals(entry.texcoords, mesh.texcoords, entry.has_texcoords ? vertex_count * 2 : 0) &&
           exactPayloadEquals(entry.indices, mesh.indices, index_count);
}

static void destroyGlMeshEntry(Gpu* gpu, ovgl_renderer::GlMeshEntry& entry)
{
    if (entry.vao)
        glDeleteVertexArrays(1, &entry.vao);
    if (entry.vb)
        gpu_destroy_buffer(gpu, entry.vb);
    if (entry.ib)
        gpu_destroy_buffer(gpu, entry.ib);
    entry.vao = 0;
    entry.vb = nullptr;
    entry.ib = nullptr;
}

static void snapshotMeshPayload(const SceneMesh& mesh, ovgl_renderer::GlMeshEntry& entry)
{
    const size_t vertex_count = static_cast<size_t>(mesh.nvertices);
    const size_t index_count = static_cast<size_t>(mesh.nindices);
    entry.path = mesh.path ? mesh.path : "";
    entry.has_normals = mesh.normals != nullptr;
    entry.has_colors = mesh.colors != nullptr;
    entry.has_texcoords = mesh.texcoords != nullptr;
    entry.positions.assign(mesh.positions, mesh.positions + vertex_count * 3);
    if (entry.has_normals)
        entry.normals.assign(mesh.normals, mesh.normals + vertex_count * 3);
    if (entry.has_colors)
        entry.colors.assign(mesh.colors, mesh.colors + vertex_count * 3);
    if (entry.has_texcoords)
        entry.texcoords.assign(mesh.texcoords, mesh.texcoords + vertex_count * 2);
    entry.indices.assign(mesh.indices, mesh.indices + index_count);
}

/* Build exactly the vertex bytes consumed by MAT_VERTEX_STRIDE. This work is
 * needed only for a new/changed cache entry; exact source comparisons let the
 * common transform-only edit skip both interleaving and GL upload. */
static bool rebuildGlBuffers(ovgl_renderer_t* r)
{
    const int mesh_count = r->scene.nmeshes;
    if (mesh_count < 0 || (mesh_count > 0 && !r->scene.meshes))
    {
        g_err = "invalid scene mesh storage";
        return false;
    }

    try
    {
        const size_t additional = static_cast<size_t>(mesh_count);
        if (r->gl_cache.size() > r->gl_cache.max_size() - additional)
        {
            g_err = "mesh GL cache capacity overflow";
            return false;
        }
        /* Reserve before creating any GL object. Once this succeeds, moving a
         * completed owner into the cache cannot allocate or strand resources. */
        r->gl_cache.reserve(r->gl_cache.size() + additional);
        r->mesh_vb.assign(additional, nullptr);
        r->mesh_ib.assign(additional, nullptr);
        r->mesh_vao.assign(additional, 0);
        r->mesh_is_floor.assign(additional, 0);
    }
    catch (const std::bad_alloc&)
    {
        g_err = "allocating mesh GL cache state failed";
        return false;
    }

    for (auto& entry : r->gl_cache)
        entry.used = false;
    for (int index = 0; index < mesh_count; ++index)
    {
        r->mesh_is_floor[static_cast<size_t>(index)] =
            isaacsim::ovgl_viewport::debug::details::ovgl::isFloorMesh(r->scene.meshes[index].path) ? 1u : 0u;
    }

    std::vector<float> interleaved;
    for (int index = 0; index < mesh_count; ++index)
    {
        const SceneMesh& mesh = r->scene.meshes[index];
        if (mesh.nvertices <= 0 || mesh.nindices <= 0)
            continue;
        if (!mesh.positions || !mesh.indices)
        {
            g_err = "mesh is missing required vertex or index storage";
            return false;
        }

        ovgl_renderer::GlMeshEntry* match = nullptr;
        for (auto& entry : r->gl_cache)
        {
            if (cacheEntryMatches(entry, mesh) && entry.vb && entry.ib && entry.vao)
            {
                match = &entry;
                break;
            }
        }
        if (match)
        {
            match->used = true;
            r->mesh_vb[static_cast<size_t>(index)] = match->vb;
            r->mesh_ib[static_cast<size_t>(index)] = match->ib;
            r->mesh_vao[static_cast<size_t>(index)] = match->vao;
            continue;
        }

        ovgl_renderer::GlMeshEntry entry;
        try
        {
            snapshotMeshPayload(mesh, entry);
            isaacsim::ovgl_viewport::debug::details::ovgl::interleaveMeshVertices(mesh, interleaved);
        }
        catch (const std::bad_alloc&)
        {
            g_err = "allocating exact mesh cache payload failed";
            return false;
        }
        entry.used = true;

        /* GL_ELEMENT_ARRAY_BUFFER is VAO state. Create and bind the VAO before
         * uploading the index buffer, and keep all partially-created objects in
         * the local owner until every setup command succeeds. */
        glGenVertexArrays(1, &entry.vao);
        if (entry.vao)
            glBindVertexArray(entry.vao);
        GpuBufferDesc vertex_desc{ GPU_BUFFER_VERTEX, static_cast<uint64_t>(mesh.nvertices) * 15 * sizeof(float),
                                   interleaved.data() };
        GpuBufferDesc index_desc{ GPU_BUFFER_INDEX, static_cast<uint64_t>(entry.indices.size()) * sizeof(uint32_t),
                                  entry.indices.data() };
        if (entry.vao)
        {
            entry.vb = gpu_create_buffer(r->gpu, &vertex_desc);
            entry.ib = gpu_create_buffer(r->gpu, &index_desc);
        }
        if (!entry.vao || !entry.vb || !entry.ib || !gpu_buffer_gl_handle(entry.vb) || !gpu_buffer_gl_handle(entry.ib))
        {
            glBindVertexArray(0);
            destroyGlMeshEntry(r->gpu, entry);
            g_err = "failed to create mesh VAO/buffers";
            return false;
        }

        glBindBuffer(GL_ARRAY_BUFFER, gpu_buffer_gl_handle(entry.vb));
        const struct
        {
            int location, size, offset;
        } attributes[6] = { { 0, 3, 0 }, { 1, 3, 12 }, { 2, 3, 24 }, { 3, 2, 36 }, { 4, 3, 44 }, { 5, 1, 56 } };
        for (const auto& attribute : attributes)
        {
            glEnableVertexAttribArray(attribute.location);
            glVertexAttribPointer(attribute.location, attribute.size, GL_FLOAT, GL_FALSE, 60,
                                  reinterpret_cast<const void*>(static_cast<uintptr_t>(attribute.offset)));
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu_buffer_gl_handle(entry.ib));
        glBindVertexArray(0);
        if (!glErrorsOk("rebuildGlBuffers"))
        {
            destroyGlMeshEntry(r->gpu, entry);
            return false;
        }

        /* The reserved vector becomes the sole owner. The draw arrays receive
         * only raw views, so duplicate instances can reference this entry safely. */
        r->gl_cache.push_back(std::move(entry));
        ovgl_renderer::GlMeshEntry& owner = r->gl_cache.back();
        r->mesh_vb[static_cast<size_t>(index)] = owner.vb;
        r->mesh_ib[static_cast<size_t>(index)] = owner.ib;
        r->mesh_vao[static_cast<size_t>(index)] = owner.vao;
    }

    /* Entries not referenced by the rebuilt scene are retired only after the
     * complete pass succeeds. This keeps failure/retry and scene reattachment
     * safe while still releasing deleted or changed geometry promptly. */
    for (auto entry = r->gl_cache.begin(); entry != r->gl_cache.end();)
    {
        if (entry->used)
        {
            ++entry;
            continue;
        }
        destroyGlMeshEntry(r->gpu, *entry);
        entry = r->gl_cache.erase(entry);
    }
    return true;
}

static ovgl_result_t ok()
{
    return ovgl_result_t{ 0 };
}
static ovgl_result_t fail(const std::string& s)
{
    g_err = s;
    return ovgl_result_t{ 1 };
}

extern "C" void ovgl_get_version(uint32_t* a, uint32_t* b, uint32_t* c)
{
    if (a)
        *a = 0;
    if (b)
        *b = 3;
    if (c)
        *c = 0; /* 0.3.0: ovgl_set_grid_overlay (0.2.1: ovgl_pick) */
}

extern "C" ovgl_result_t ovgl_create_renderer(const ovgl_renderer_desc_t* desc, ovgl_renderer_t** out)
{
    if (!desc || !out)
        return fail("null arg");
    auto* r = new ovgl_renderer();
    r->desc = *desc;
    /* Allow overriding the supersample factor for profiling (default ss=2). */
    if (const char* e = std::getenv("OVGL_SS"))
    {
        int v = std::atoi(e);
        if (v >= 1 && v <= 4)
            r->ss = v;
    }
    /* Ad-hoc debugging override for the grid overlay (default OFF; the API
     * contract lives at ovgl.h::ovgl_set_grid_overlay). */
    if (const char* e = std::getenv("OVGL_GRID_OVERLAY"))
    {
        if (e[0] == '1')
            r->grid_overlay = 1;
    }
    /* Kill-switch for the MuJoCo ground checker (default OFF -- see the
     * floor_checker field). Accepts BOTH directions, unlike the grid override
     * above, so a caller can pin the state it wants without depending on what
     * this build happens to default to. */
    if (const char* e = std::getenv("OVGL_FLOOR_CHECKER"))
    {
        r->floor_checker = (e[0] == '1') ? 1 : 0;
    }
    /* Opt-in back-face culling of doubleSided=false gprims (default OFF -- see
     * the backface_cull field). Both directions, same reasoning as above. */
    if (const char* e = std::getenv("OVGL_BACKFACE_CULL"))
    {
        r->backface_cull = (e[0] == '1') ? 1 : 0;
    }
    /* Screen-space ambient occlusion (default ON). Both directions, so a caller
     * can pin the state it wants; OVGL_AO=0 skips the prepass and the SSAO pass
     * and reproduces a build without either. */
    if (const char* e = std::getenv("OVGL_AO"))
    {
        r->ao_enabled = (e[0] == '1') ? 1 : 0;
    }
    /* Tuning knobs. Each is taken only when it parses to a finite, sensible
     * value, so a typo leaves the default rather than silently zeroing AO. */
    struct AoKnob
    {
        const char* name;
        float* dst;
        float lo, hi;
    };
    const AoKnob ao_knobs[] = {
        { "OVGL_AO_RADIUS", &r->ao_radius, 1e-4f, 1e4f },
        { "OVGL_AO_BIAS", &r->ao_bias, 0.0f, 1e3f },
        { "OVGL_AO_INTENSITY", &r->ao_intensity, 0.0f, 8.0f },
        { "OVGL_AO_STRENGTH", &r->ao_strength, 0.0f, 1.0f },
    };
    for (const AoKnob& k : ao_knobs)
    {
        const char* e = std::getenv(k.name);
        if (!e || !e[0])
            continue;
        char* end = nullptr;
        const double v = std::strtod(e, &end);
        if (end && end != e && std::isfinite(v) && v >= k.lo && v <= k.hi)
            *k.dst = (float)v;
        else
            gl_log(GL_LOG_WARN, "ovgl", "ignoring %s='%s' (want %.4g..%.4g)", k.name, e, (double)k.lo, (double)k.hi);
    }
    *out = r;
    return ok();
}

extern "C" ovgl_result_t ovgl_destroy_renderer(ovgl_renderer_t* r)
{
    if (!r)
        return ok();
    if (r->gpu)
    {
        /* GL objects can only be deleted with their creating context current. On an
         * owned context the shared, process-global EGL display may already have been
         * terminated by another OVGL/ovraycast context's teardown, in which case
         * egl_headless_make_current fails with EGL_NOT_INITIALIZED (0x3001). This used
         * to hard-fail here (`return fail(...)`), which leaked the renderer and raised on
         * shutdown (the "ovgl_destroy_renderer: failed to make owned GL context current"
         * teardown error). When the context is gone the GL objects are already invalid,
         * so skip GL deletion and still free the CPU-side state below. */
        bool gl_current;
        if (r->gl_adopt == 1)
            gl_current = validateAdoptedContext(r, "ovgl_destroy_renderer");
        else
            gl_current = r->egl && egl_headless_make_current(r->egl);
        if (gl_current)
        {
            /* Delete every object while its creating context is current.  Pipelines,
             * GPU-owned material/environment state, and renderer-owned buffers/FBOs
             * all precede gpu_shutdown; the latter releases remaining internal
             * shadow/overlay/instance objects and the CPU-side Gpu allocation. */
            if (r->shadow_pipe)
                gpu_destroy_pipeline(r->gpu, r->shadow_pipe);
            if (r->pipe)
                gpu_destroy_pipeline(r->gpu, r->pipe);
            if (r->ao_pipe)
                gpu_destroy_pipeline(r->gpu, r->ao_pipe);
            r->shadow_pipe = r->pipe = r->ao_pipe = nullptr;
            if (r->grid_pipe)
                gpu_destroy_pipeline(r->gpu, r->grid_pipe);
            r->grid_pipe = nullptr;
            if (r->grid_vb)
                gpu_destroy_buffer(r->gpu, r->grid_vb);
            r->grid_vb = nullptr;
            gpu_destroy_materials(r->gpu);
            gpu_destroy_environment(r->gpu);
            /* Destroy only owner entries. mesh_vb/mesh_ib/mesh_vao are views and
             * must never participate in lifetime decisions, even after the live
             * scene has been freed by a stage reattachment. */
            for (auto& entry : r->gl_cache)
                destroyGlMeshEntry(r->gpu, entry);
            r->gl_cache.clear();
            r->mesh_vb.clear();
            r->mesh_ib.clear();
            r->mesh_vao.clear();
            if (r->render_fbo)
                glDeleteFramebuffers(1, &r->render_fbo);
            if (r->render_color)
                glDeleteRenderbuffers(1, &r->render_color);
            if (r->render_depth)
                glDeleteTextures(1, &r->render_depth); /* depth TEXTURE (Depth AOV samples it) */
            if (r->resolve_fbo)
                glDeleteFramebuffers(1, &r->resolve_fbo);
            if (r->resolve_color)
                glDeleteTextures(1, &r->resolve_color);
            if (r->ao_fbo)
                glDeleteFramebuffers(1, &r->ao_fbo);
            if (r->ao_normal)
                glDeleteTextures(1, &r->ao_normal);
            if (r->ao_depth)
                glDeleteTextures(1, &r->ao_depth);
            if (r->pack_pbo[0])
                glDeleteBuffers(2, r->pack_pbo);
            gpu_shutdown(r->gpu);
        }
        /* else: owned context already gone; GL objects invalid. gpu_shutdown is skipped
         * (it would glDelete without a context); the CPU-side Gpu allocation is left to
         * process teardown. destroyOwnedContext below is safe on a terminated display. */
        r->gpu = nullptr;
    }
    ovgl_scene_free(&r->scene);
    if (r->egl)
        destroyOwnedContext(r->egl);
    delete r;
    return ok();
}

extern "C" ovgl_result_t ovgl_attach_ovstage(ovgl_renderer_t* r, ovstage_instance_t* stage)
{
    if (!r)
        return fail("null renderer");
    if (!stage)
        return fail("null stage");
    if (r->stage != stage)
    {
        ovgl_scene_free(&r->scene);
        r->scene_materials.clear();
        r->scene_textures.clear();
        r->scene_texture_pixels.clear();
        r->scene_lights.clear();
        /* Draw-index storage is non-owning. Drop stale views immediately; the
         * owner cache remains alive until the next current-context rebuild (or
         * renderer destruction), where entries can be reused or retired. */
        r->mesh_vb.clear();
        r->mesh_ib.clear();
        r->mesh_vao.clear();
        r->mesh_is_floor.clear();
        r->structure_gen = 0;
        r->last_ordinal = 0;
        r->last_shadow_ordinal = (ovstage_ordinal_t)-1;
        r->first_render = true;
        r->topo_built = false; /* next render rebuilds geometry + GL buffers */
    }
    r->stage = stage;
    return ok();
}

extern "C" ovgl_result_t ovgl_set_camera(ovgl_renderer_t* r, const ovgl_camera_t* c)
{
    if (!r || !c)
        return fail("null arg");
    if (!validateCamera(*c))
        return fail(g_err);
    if (!computeRenderDimensions(r, c->image_width, c->image_height, nullptr, nullptr, nullptr))
        return fail(g_err);
    r->cam = *c;
    r->cam_valid = true;
    return ok();
}

extern "C" ovgl_result_t ovgl_set_pull_paused(ovgl_renderer_t* r, int paused)
{
    if (!r)
        return fail("null renderer");
    r->pull_paused = paused ? 1 : 0;
    return ok();
}

extern "C" ovgl_result_t ovgl_set_grid_overlay(ovgl_renderer_t* r, int enabled)
{
    if (!r)
        return fail("ovgl_set_grid_overlay: null renderer");
    r->grid_overlay = enabled ? 1 : 0;
    return ok();
}

/* AO settings (ovgl.h::ovgl_set_ao_settings). A NEGATIVE float means "leave
 * this one alone", so a front-end driving one slider does not have to resend
 * -- or invent -- the other three. Values are clamped to the same ranges the
 * env knobs accept, because a UI slider and a typo deserve the same floor. */
extern "C" ovgl_result_t ovgl_set_ao_settings(
    ovgl_renderer_t* r, int enabled, float radius, float bias, float intensity, float strength)
{
    if (!r)
        return fail("ovgl_set_ao_settings: null renderer");
    if (enabled >= 0)
        r->ao_enabled = enabled ? 1 : 0;
    auto take = [](float v, float* dst, float lo, float hi)
    {
        if (v < 0.0f || !std::isfinite(v))
            return; /* unchanged */
        *dst = v < lo ? lo : (v > hi ? hi : v);
    };
    take(radius, &r->ao_radius, 1e-4f, 1e4f);
    take(bias, &r->ao_bias, 0.0f, 1.0f);
    take(intensity, &r->ao_intensity, 0.0f, 8.0f);
    take(strength, &r->ao_strength, 0.0f, 1.0f);
    return ok();
}

extern "C" ovgl_result_t ovgl_get_ao_settings(
    ovgl_renderer_t* r, int* enabled, float* radius, float* bias, float* intensity, float* strength)
{
    if (!r)
        return fail("ovgl_get_ao_settings: null renderer");
    if (enabled)
        *enabled = r->ao_enabled;
    if (radius)
        *radius = r->ao_radius;
    if (bias)
        *bias = r->ao_bias;
    if (intensity)
        *intensity = r->ao_intensity;
    if (strength)
        *strength = r->ao_strength;
    return ok();
}

extern "C" ovgl_result_t ovgl_refresh_transforms(ovgl_renderer_t* r, ovstage_ordinal_t ordinal)
{
    if (!r)
        return fail("ovgl_refresh_transforms: null renderer");
    if (!r->stage)
        return fail("ovgl_refresh_transforms: no attached ovstage");
    if (!r->topo_built)
        return fail("ovgl_refresh_transforms: topology is not built yet (render once first)");
    if (r->pull_paused)
        return ok();
    /* Expanded instances share their prototype prim path, so a path-only
     * worldMatrix refresh would collapse every clone onto the prototype. Keep
     * last_ordinal unchanged and let the next render rebuild the authoritative
     * instance transforms from the sealed snapshot. */
    if (ovgl_scene_requires_point_instancer_rebuild(&r->scene))
        return ok();
    if (!ovgl_scene_refresh_xforms(r->stage, ordinal, &r->scene))
        return fail(std::string("ovgl_scene_refresh_xforms: ") + ovgl_scene_last_error());
    /* Lights bake their composed world pose into GpuLight at scene-build time,
     * so the transform-only refresh must re-derive them as well — otherwise a
     * moved DistantLight keeps lighting/shadowing from its stale pose on every
     * cheap-path frame (2026-07-19 fast-path adversary F1). Same lock as the
     * builds: readSceneLights fills the shared scene-light scratch. The
     * per-frame render re-uploads r->scene_lights unconditionally, so a
     * refreshed snapshot is all a cheap-path frame needs. Any failure returns
     * BEFORE last_ordinal is published, so the next render falls back to the
     * full-rebuild correctness baseline. */
    {
        std::lock_guard<std::mutex> lock(g_scene_build_mutex);
        if (!ovgl_scene_refresh_lights(r->stage, ordinal))
            return fail(std::string("ovgl_scene_refresh_lights: ") + ovgl_scene_last_error());
        const GpuLight* lights = nullptr;
        const int light_count = ovgl_scene_lights(&lights);
        if (light_count < 0 || (light_count > 0 && !lights))
            return fail("invalid ovgl scene light snapshot");
        try
        {
            r->scene_lights.assign(lights, lights + static_cast<size_t>(light_count));
        }
        catch (const std::bad_alloc&)
        {
            return fail("allocating ovgl renderer light snapshot failed");
        }
        /* The DomeLight is NOT in that snapshot — it becomes the IBL
         * environment instead (applyEnvironment) — and readSceneDome ran
         * only inside the full ovgl_scene_build. So publishing last_ordinal
         * below with r->scene.dome_* untouched froze the environment at
         * cold-attach values for the life of the attach, and a live dome
         * inputs:intensity edit rendered BIT-IDENTICALLY to no edit at all
         * (dome 500 -> 15 -> 5000 -> 0 all at frame mean 156.8241; official
         * 210.4 / 85.0 / 252.4 / 73.8). Same aspect-forgotten shape as the
         * 2026-07-19 F1 light-pose finding one block up, and the same
         * remedy: re-derive it here, inside the same lock, before the
         * publish. Refreshes nlights_unsupported too — applyEnvironment's
         * request depends on it and it was build-only as well. */
        if (!ovgl_scene_refresh_dome(r->stage, ordinal, &r->scene))
            return fail(std::string("ovgl_scene_refresh_dome: ") + ovgl_scene_last_error());
    }
    /* Draw the cached geometry at this ordinal: render_frame rebuilds only when the ordinal
     * it is handed differs from the last one it consumed, so publishing it here is what makes
     * the next frame take the cheap path. */
    r->last_ordinal = ordinal;
    return ok();
}

extern "C" ovgl_result_t ovgl_refresh_transform_batch(ovgl_renderer_t* r,
                                                      ovstage_ordinal_t ordinal,
                                                      const double* world_matrices,
                                                      size_t mesh_count)
{
    if (!r)
        return fail("ovgl_refresh_transform_batch: null renderer");
    if (!r->topo_built)
        return fail("ovgl_refresh_transform_batch: topology is not built yet (render once first)");
    if (r->pull_paused)
        return fail("ovgl_refresh_transform_batch: pulls are paused");
    if (r->scene.ncurves != 0)
        return fail("ovgl_refresh_transform_batch: curves require a sealed-snapshot refresh");
    if (r->scene.npi_batches != 0)
        return fail("ovgl_refresh_transform_batch: compact instances require a sealed-snapshot refresh");
    if (!ovgl_scene_apply_xforms(&r->scene, world_matrices, mesh_count))
        return fail(std::string("ovgl_scene_apply_xforms: ") + ovgl_scene_last_error());

    /* The backend has already proven that no changed transform affects a light,
     * dome, or bound camera. Publishing only after the atomic scene update keeps
     * render_frame from observing a partly applied batch. */
    r->last_ordinal = ordinal;
    return ok();
}

extern "C" ovgl_result_t ovgl_invalidate_scene(ovgl_renderer_t* r)
{
    if (!r)
        return fail("ovgl_invalidate_scene: null renderer");
    r->topo_built = false;
    return ok();
}

/* Targeted material refresh (DESIGN_CHANGE_TRACKING.md D4, the tier past
 * kColumnsOnly): re-resolve the cached meshes' bound materials at `ordinal`
 * CPU-side and mark the GPU material state for replacement on the next
 * render_frame — geometry, mesh constants, and vertex/index buffers stay
 * untouched, and SceneMesh.material_index stays valid because materials are
 * one-per-mesh in mesh order. Companion to ovgl_refresh_transforms: callers
 * whose change window carries only shader-input edits (and optionally
 * transforms) call these instead of letting the ordinal move force a full
 * scene rebuild.
 *
 * Deliberately does NOT publish last_ordinal: with two independent refresh
 * entry points, whichever published first would leave the cheap path serving
 * the OTHER aspect stale if its refresh then failed. The publish point is
 * ovgl_refresh_transforms — call it AFTER this succeeds (it also re-reads
 * lights, whose inputs:* edits classify like shader inputs). Any failure
 * here therefore leaves the renderer un-published and the next render_frame
 * takes the full-rebuild correctness baseline. */
extern "C" ovgl_result_t ovgl_refresh_materials(ovgl_renderer_t* r, ovstage_ordinal_t ordinal)
{
    if (!r)
        return fail("ovgl_refresh_materials: null renderer");
    if (!r->stage)
        return fail("ovgl_refresh_materials: no attached ovstage");
    if (!r->topo_built)
        return fail("ovgl_refresh_materials: topology is not built yet (render once first)");
    if (r->pull_paused)
        return ok();
    /* Expanded instances share their prototype prim path; the per-slot
     * material update is idempotent across clones, but stay symmetric with
     * ovgl_refresh_transforms' conservative bail: no publish, so the next
     * render rebuilds from the sealed snapshot. */
    if (ovgl_scene_requires_point_instancer_rebuild(&r->scene))
        return ok();
    if (r->scene_materials.empty())
        return fail("ovgl_refresh_materials: no retained material snapshot");
    {
        /* Refresh IN the renderer's own retained snapshot — never the build's
         * global scratch, which belongs to whichever scene was built last.
         * The scene-side refresh guarantees texture bindings are unchanged
         * (slot equality), so r->scene_textures stays valid as-is and no
         * auxiliary re-snapshot happens. The lock covers the shared read
         * scratch the re-resolution uses. */
        std::lock_guard<std::mutex> lock(g_scene_build_mutex);
        if (!ovgl_scene_refresh_materials(
                r->stage, ordinal, &r->scene, r->scene_materials.data(), static_cast<int>(r->scene_materials.size())))
            return fail(std::string("ovgl_scene_refresh_materials: ") + ovgl_scene_last_error());
    }
    r->materials_dirty = true;
    return ok();
}

/* --- ovgl_pick: CPU ray cast against the resident scene cache ------------- */

/* General 4x4 inverse in double, row-major in/out (same glu algorithm as the
 * float `inverse` above). Returns false if singular. */
static bool inverse4x4D(const double* a, double* o)
{
    double inv[16];
    inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] + a[9] * a[7] * a[14] +
             a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
    inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] - a[8] * a[7] * a[14] -
             a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
    inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] + a[8] * a[7] * a[13] +
             a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
    inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] - a[8] * a[6] * a[13] -
              a[12] * a[5] * a[10] + a[12] * a[6] * a[9];
    inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] - a[9] * a[3] * a[14] -
             a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
    inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] + a[8] * a[3] * a[14] +
             a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
    inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] - a[8] * a[3] * a[13] -
             a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
    inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] + a[8] * a[2] * a[13] +
              a[12] * a[1] * a[10] - a[12] * a[2] * a[9];
    inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] + a[5] * a[3] * a[14] +
             a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
    inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] - a[4] * a[3] * a[14] -
             a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
    inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] + a[4] * a[3] * a[13] +
              a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
    inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] - a[4] * a[2] * a[13] -
              a[12] * a[1] * a[6] + a[12] * a[2] * a[5];
    inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] - a[5] * a[3] * a[10] -
             a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
    inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] + a[4] * a[3] * a[10] +
             a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
    inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] - a[4] * a[3] * a[9] -
              a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
    inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] + a[4] * a[2] * a[9] + a[8] * a[1] * a[6] -
              a[8] * a[2] * a[5];
    double det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12];
    if (det == 0.0)
        return false;
    det = 1.0 / det;
    for (int i = 0; i < 16; i++)
        o[i] = inv[i] * det;
    return true;
}

/* Row-vector transforms against a USD row-major matrix (p' = p * M). */
static void xformPointRv(const double m[16], const double p[3], double o[3])
{
    o[0] = p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12];
    o[1] = p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13];
    o[2] = p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14];
}
static void xformVectorRv(const double m[16], const double v[3], double o[3])
{
    o[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8];
    o[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9];
    o[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10];
}

/* Ray/AABB slab test; true when [tmin_out, tmax_out] intersects t >= 0. */
static bool rayAabb(const double o[3], const double d[3], const float lo[3], const float hi[3], double* t_enter)
{
    double tmin = 0.0, tmax = std::numeric_limits<double>::max();
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(d[i]) < 1e-15)
        {
            if (o[i] < (double)lo[i] || o[i] > (double)hi[i])
                return false;
            continue;
        }
        double inv = 1.0 / d[i];
        double t0 = ((double)lo[i] - o[i]) * inv;
        double t1 = ((double)hi[i] - o[i]) * inv;
        if (t0 > t1)
            std::swap(t0, t1);
        if (t0 > tmin)
            tmin = t0;
        if (t1 < tmax)
            tmax = t1;
        if (tmin > tmax)
            return false;
    }
    *t_enter = tmin;
    return true;
}

/* Möller–Trumbore, both sides. Returns t > eps on hit. */
static bool rayTriangle(const double o[3], const double d[3], const float* v0, const float* v1, const float* v2, double* t_out)
{
    const double eps = 1e-9;
    double e1[3] = { v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] };
    double e2[3] = { v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };
    double p[3] = { d[1] * e2[2] - d[2] * e2[1], d[2] * e2[0] - d[0] * e2[2], d[0] * e2[1] - d[1] * e2[0] };
    double det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];
    if (std::abs(det) < eps)
        return false;
    double inv_det = 1.0 / det;
    double s[3] = { o[0] - v0[0], o[1] - v0[1], o[2] - v0[2] };
    double u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;
    double q[3] = { s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0] };
    double v = (d[0] * q[0] + d[1] * q[1] + d[2] * q[2]) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;
    double t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv_det;
    if (t <= eps)
        return false;
    *t_out = t;
    return true;
}

extern "C" ovgl_result_t ovgl_pick(ovgl_renderer_t* r, double ndc_x, double ndc_y, ovgl_pick_result_t* out)
{
    if (!r || !out)
        return fail("ovgl_pick: null arg");
    std::memset(out, 0, sizeof(*out));
    if (!r->stage)
        return fail("ovgl_pick: no attached ovstage");
    if (!r->cam_valid)
        return fail("ovgl_pick: no camera set");
    if (r->cam.image_width <= 0 || r->cam.image_height <= 0)
        return fail("ovgl_pick: camera has no image size");

    /* World ray through NDC, matching the render's perspective(fov_y, w/h). */
    const double* e = r->cam.eye;
    double f[3] = { r->cam.target[0] - e[0], r->cam.target[1] - e[1], r->cam.target[2] - e[2] };
    double fn = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if (fn < 1e-12)
        return fail("ovgl_pick: degenerate camera (eye == target)");
    for (double& c : f)
        c /= fn;
    double s[3] = { f[1] * r->cam.up[2] - f[2] * r->cam.up[1], f[2] * r->cam.up[0] - f[0] * r->cam.up[2],
                    f[0] * r->cam.up[1] - f[1] * r->cam.up[0] };
    double sn = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    if (sn < 1e-12)
        return fail("ovgl_pick: degenerate camera (up parallel to view)");
    for (double& c : s)
        c /= sn;
    double u[3] = { s[1] * f[2] - s[2] * f[1], s[2] * f[0] - s[0] * f[2], s[0] * f[1] - s[1] * f[0] };
    const double th = std::tan(r->cam.fov_y_rad * 0.5);
    const double aspect = (double)r->cam.image_width / (double)r->cam.image_height;
    double dir[3];
    for (int i = 0; i < 3; i++)
        dir[i] = f[i] + ndc_x * th * aspect * s[i] + ndc_y * th * u[i];
    double dn = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    for (double& c : dir)
        c /= dn;

    double best_t = std::numeric_limits<double>::max();
    const SceneMesh* best = nullptr;
    for (int i = 0; i < r->scene.nmeshes; i++)
    {
        const SceneMesh& m = r->scene.meshes[i];
        if (m.is_proto_only || !m.visible)
            continue;
        if (m.nvertices <= 0 || m.nindices < 3 || !m.positions || !m.indices)
            continue;
        double t_enter;
        if (!rayAabb(e, dir, m.bounds_min, m.bounds_max, &t_enter))
            continue;
        if (t_enter > best_t)
            continue; /* box is entirely behind the best hit */
        double winv[16];
        if (!inverse4x4D(m.world_xform, winv))
            continue;
        double lo[3], ld[3];
        xformPointRv(winv, e, lo);
        xformVectorRv(winv, dir, ld);
        for (int k = 0; k + 2 < m.nindices; k += 3)
        {
            const float* v0 = m.positions + 3 * m.indices[k];
            const float* v1 = m.positions + 3 * m.indices[k + 1];
            const float* v2 = m.positions + 3 * m.indices[k + 2];
            double t_obj;
            if (!rayTriangle(lo, ld, v0, v1, v2, &t_obj))
                continue;
            /* Object-space t is not world t under scale: take the world-space
             * distance of the transformed hit point along the world ray. */
            double hp_obj[3] = { lo[0] + t_obj * ld[0], lo[1] + t_obj * ld[1], lo[2] + t_obj * ld[2] };
            double hp_w[3];
            xformPointRv(m.world_xform, hp_obj, hp_w);
            double t_w = (hp_w[0] - e[0]) * dir[0] + (hp_w[1] - e[1]) * dir[1] + (hp_w[2] - e[2]) * dir[2];
            if (t_w <= 0.0 || t_w >= best_t)
                continue;
            best_t = t_w;
            best = &m;
            out->world_pos[0] = hp_w[0];
            out->world_pos[1] = hp_w[1];
            out->world_pos[2] = hp_w[2];
        }
    }
    if (best)
    {
        out->hit = 1;
        out->distance = best_t;
        if (best->path)
        {
            std::strncpy(out->path, best->path, sizeof(out->path) - 1);
            out->path[sizeof(out->path) - 1] = '\0';
        }
    }
    return ok();
}

static bool ensureGl(ovgl_renderer_t* r, int w, int h, bool adopt)
{
    const int requested_mode = adopt ? 1 : 0;
    const uintptr_t current_adopted_context = adopt ? egl_headless_current_context_id() : 0;
    if (adopt && !current_adopted_context)
    {
        g_err = "adopted renderer requires a current GL context";
        return false;
    }
    if (r->gpu)
    {
        if (r->gl_adopt != requested_mode)
        {
            g_err = "renderer cannot switch between owned and adopted GL contexts";
            return false;
        }
        if (!r->pipe || !r->shadow_pipe)
        {
            g_err = "renderer GL initialization is incomplete";
            return false;
        }
        if (adopt && r->adopted_context_id != current_adopted_context)
        {
            g_err = "current GL context does not match the renderer's adopted context";
            return false;
        }
        if (r->w == w && r->h == h)
            return true;
        if (!adopt)
        {
            if (!r->egl || !egl_headless_make_current(r->egl))
            {
                g_err = "egl_headless_make_current failed";
                return false;
            }
            if (!egl_headless_resize(r->egl, w, h))
            {
                g_err = "egl_headless_resize failed";
                return false;
            }
        }
        gpu_resize(r->gpu, w, h);
        if (!glErrorsOk("gpu_resize"))
            return false;
        r->w = w;
        r->h = h;
        return true;
    }

    /* Build into locals and publish only after every context/pipeline step has
     * succeeded.  Previously a shader failure left r->gpu non-null; the next
     * frame then skipped pipeline creation and permanently reported success
     * while drawing with a null program. */
    EglHeadless* new_egl = nullptr;
    Gpu* new_gpu = nullptr;
    GpuPipeline new_pipe = nullptr;
    GpuPipeline new_shadow_pipe = nullptr;
    GpuPipeline new_ao_pipe = nullptr;
    auto discard_partial = [&]()
    {
        if (new_ao_pipe)
            gpu_destroy_pipeline(new_gpu, new_ao_pipe);
        if (new_shadow_pipe)
            gpu_destroy_pipeline(new_gpu, new_shadow_pipe);
        if (new_pipe)
            gpu_destroy_pipeline(new_gpu, new_pipe);
        if (new_gpu)
            gpu_shutdown(new_gpu);
        if (new_egl)
            destroyOwnedContext(new_egl);
    };

    {
        if (!adopt)
        {
            new_egl = createOwnedContext(w, h);
            if (!new_egl)
            {
                g_err = "egl_headless_create failed";
                return false;
            }
            if (!egl_headless_make_current(new_egl))
            {
                g_err = "egl_headless_make_current failed";
                discard_partial();
                return false;
            }
        } /* adopt: the caller's GL context (e.g. a Qt QOpenGLWidget) is already current */
        new_gpu = gpu_init(nullptr, w, h);
        if (!new_gpu)
        {
            g_err = "gpu_init failed";
            discard_partial();
            return false;
        }
        if (!glErrorsOk("gpu_init"))
        {
            discard_partial();
            return false;
        }
        /* PBR material pipeline: 60-byte vertex (pos|normal|color|uv|tangent|sign). */
        GpuVertexAttrib attrs[6] = {
            { 0, 0, GPU_FORMAT_FLOAT3 }, /* pos          */
            { 1, 12, GPU_FORMAT_FLOAT3 }, /* normal       */
            { 2, 24, GPU_FORMAT_FLOAT3 }, /* color        */
            { 3, 36, GPU_FORMAT_FLOAT2 }, /* texcoord     */
            { 4, 44, GPU_FORMAT_FLOAT3 }, /* tangent      */
            { 5, 56, GPU_FORMAT_FLOAT1 }, /* tangent sign */
        };
        GpuPipelineDesc pd{};
        pd.vert_glsl = k_pbr_vert_gles;
        pd.frag_glsl = k_pbr_frag_gles;
        pd.push_constant_size = sizeof(GpuMeshPushConstants);
        pd.vertex_stride = 60; /* MAT_VERTEX_STRIDE */
        pd.attribs = attrs;
        pd.nattribs = 6;
        new_pipe = gpu_create_material_pipeline(new_gpu, &pd);
        if (!new_pipe)
        {
            g_err = "gpu_create_material_pipeline failed";
            discard_partial();
            return false;
        }
        /* Depth-only shadow pipeline: position attribute only, same 60-byte stride. */
        GpuVertexAttrib spos = { 0, 0, GPU_FORMAT_FLOAT3 };
        GpuPipelineDesc spd{};
        spd.vert_glsl = g_kShadowVertexShader;
        spd.frag_glsl = g_kShadowFragmentShader;
        spd.push_constant_size = sizeof(GpuMeshPushConstants);
        spd.vertex_stride = 60;
        spd.attribs = &spos;
        spd.nattribs = 1;
        new_shadow_pipe = gpu_create_pipeline(new_gpu, &spd);
        if (!new_shadow_pipe)
        {
            g_err = "gpu_create_pipeline (shadow) failed";
            discard_partial();
            return false;
        }
        /* AO prepass pipeline: position + normal, same 60-byte stride. */
        GpuVertexAttrib aoattrs[2] = {
            { 0, 0, GPU_FORMAT_FLOAT3 }, /* pos    */
            { 1, 12, GPU_FORMAT_FLOAT3 }, /* normal */
        };
        GpuPipelineDesc apd{};
        apd.vert_glsl = g_kAmbientOcclusionPrepassVertexShader;
        apd.frag_glsl = g_kAmbientOcclusionPrepassFragmentShader;
        apd.push_constant_size = sizeof(GpuMeshPushConstants);
        apd.vertex_stride = 60;
        apd.attribs = aoattrs;
        apd.nattribs = 2;
        new_ao_pipe = gpu_create_pipeline(new_gpu, &apd);
        if (!new_ao_pipe)
        {
            g_err = "gpu_create_pipeline (AO prepass) failed";
            discard_partial();
            return false;
        }
        /* No scene lights -> synthetic fallback rig; neutral tone mapping. */
        if (!gpu_upload_lights(new_gpu, nullptr, 0))
        {
            g_err = "gpu_upload_lights failed";
            discard_partial();
            return false;
        }
        gpu_set_authored_light_count(new_gpu, 0);
        gpu_set_fallback_lighting(new_gpu, 1);
        gpu_set_tone_mapping(new_gpu, 1.0f, 1.0f, 1.0f, 0u);
        if (!glErrorsOk("GL pipeline initialization"))
        {
            discard_partial();
            return false;
        }
    }

    r->egl = new_egl;
    r->gpu = new_gpu;
    r->pipe = new_pipe;
    r->shadow_pipe = new_shadow_pipe;
    r->ao_pipe = new_ao_pipe;
    r->gl_adopt = requested_mode;
    r->adopted_context_id = current_adopted_context;
    r->w = w;
    r->h = h;
    return true;
}

extern "C" ovgl_result_t ovgl_set_environment(ovgl_renderer_t* r, const char* hdr_path, float intensity)
{
    if (!r)
        return fail("null renderer");
    r->env_path = (hdr_path && hdr_path[0]) ? hdr_path : "";
    r->env_intensity = (intensity > 0.0f) ? intensity : 1.0f;
    r->env_loaded = false; /* (re)load on the next render_frame */
    r->env_failed = false;
    r->env_dirty = true;
    return ok();
}

/* (Re)create the offscreen render target (iw×ih: RGBA8 color + DEPTH24) the meshes
 * draw into, plus a 1x resolve target (w×h: RGBA8) it is blit-downscaled into. Only
 * recreated when the dimensions change. GL must be current. */
static bool ensureFbos(ovgl_renderer_t* r, int iw, int ih, int w, int h)
{
    if (r->render_fbo && r->fbo_iw == iw && r->fbo_ih == ih && r->fbo_w == w && r->fbo_h == h)
        return true;
    if (r->render_fbo)
        glDeleteFramebuffers(1, &r->render_fbo);
    if (r->render_color)
        glDeleteRenderbuffers(1, &r->render_color);
    if (r->render_depth)
        glDeleteTextures(1, &r->render_depth);
    if (r->resolve_fbo)
        glDeleteFramebuffers(1, &r->resolve_fbo);
    if (r->resolve_color)
        glDeleteTextures(1, &r->resolve_color); /* it's a TEXTURE, not a renderbuffer */
    r->render_fbo = r->render_color = r->render_depth = 0;
    r->resolve_fbo = r->resolve_color = 0;

    glGenFramebuffers(1, &r->render_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, r->render_fbo);
    glGenRenderbuffers(1, &r->render_color);
    glBindRenderbuffer(GL_RENDERBUFFER, r->render_color);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, iw, ih);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, r->render_color);
    /* Depth is a TEXTURE (same DEPTH_COMPONENT24 storage a renderbuffer would
     * carry, so rasterization is unchanged): the Depth-AOV pack pass samples
     * it (gpu_depth_pack_read) — GLES renderbuffers cannot be sampled or
     * read back. NEAREST + compare-mode NONE = plain depth-value sampling. */
    glGenTextures(1, &r->render_depth);
    glBindTexture(GL_TEXTURE_2D, r->render_depth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, iw, ih, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, r->render_depth, 0);
    if (!glFramebufferComplete(GL_FRAMEBUFFER, "render FBO"))
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glGenFramebuffers(1, &r->resolve_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, r->resolve_fbo);
    /* resolve_color is a TEXTURE (not a renderbuffer) so the adopt/GUI path can sample it
     * directly into the Qt widget -- and glReadPixels still works for the headless path. */
    glGenTextures(1, &r->resolve_color);
    glBindTexture(GL_TEXTURE_2D, r->resolve_color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r->resolve_color, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    if (!glFramebufferComplete(GL_FRAMEBUFFER, "resolve FBO"))
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    /* The adopted/FBO path never reads pixels.  Do not allocate or perturb PBO
     * state there; keep these buffers exclusive to the owned headless path. */
    if (r->gl_adopt == 0)
    {
        if (!r->pack_pbo[0])
            glGenBuffers(2, r->pack_pbo);
        for (int i = 0; i < 2; i++)
        {
            glBindBuffer(GL_PIXEL_PACK_BUFFER, r->pack_pbo[i]);
            glBufferData(GL_PIXEL_PACK_BUFFER, (GLsizeiptr)w * (GLsizeiptr)h * 4, nullptr, GL_STREAM_READ);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        r->pack_primed = false; /* size changed -> prior pixels are invalid */
    }

    if (!glErrorsOk("ensureFbos"))
        return false;

    r->fbo_iw = iw;
    r->fbo_ih = ih;
    r->fbo_w = w;
    r->fbo_h = h;

    return true;
}

/* (Re)create the AO prepass target: an RGBA8 world-normal color attachment and
 * its OWN DEPTH_COMPONENT24 texture, both iw x ih -- the same internal
 * (supersampled) grid the color pass rasterizes on, so a fragment's AO is
 * looked up at its own gl_FragCoord with no rescale.
 *
 * It does NOT reuse render_fbo's depth, even though the two passes compute
 * identical values. Sharing would mean the color pass either re-clears it (and
 * throws the prepass away) or skips its depth clear (and depends on the prepass
 * having run) -- and either way the depth texture would be attached to the
 * color pass while the SSAO result derived from it is being sampled. One
 * extra depth texture, ~7.7 MB at 800x600 ss=2, buys a prepass the color pass
 * cannot interact with at all. GL must be current. */
static bool ensureAoTargets(ovgl_renderer_t* r, int iw, int ih)
{
    if (r->ao_fbo && r->ao_iw == iw && r->ao_ih == ih)
        return true;
    if (r->ao_fbo)
        glDeleteFramebuffers(1, &r->ao_fbo);
    if (r->ao_normal)
        glDeleteTextures(1, &r->ao_normal);
    if (r->ao_depth)
        glDeleteTextures(1, &r->ao_depth);
    r->ao_fbo = r->ao_normal = r->ao_depth = 0;
    r->ao_iw = r->ao_ih = 0;

    glGenFramebuffers(1, &r->ao_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, r->ao_fbo);

    glGenTextures(1, &r->ao_normal);
    glBindTexture(GL_TEXTURE_2D, r->ao_normal);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, iw, ih, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    /* NEAREST both ways: a filtered normal on a silhouette is the average of
     * two unrelated surfaces, which tilts the SSAO hemisphere into the geometry
     * and draws a dark rim exactly where the occlusion estimate is already
     * least reliable. */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r->ao_normal, 0);

    glGenTextures(1, &r->ao_depth);
    glBindTexture(GL_TEXTURE_2D, r->ao_depth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, iw, ih, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    /* Plain depth-VALUE sampling, not a shadow compare -- the SSAO pass needs
     * the number, not a <= test against a reference. */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glBindTexture(GL_TEXTURE_2D, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, r->ao_depth, 0);

    if (!glFramebufferComplete(GL_FRAMEBUFFER, "AO prepass FBO"))
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!glErrorsOk("ensureAoTargets"))
        return false;
    r->ao_iw = iw;
    r->ao_ih = ih;
    return true;
}

/* Snapshot + restore every caller-owned state value the adopted-context path
 * changes.  Qt guarantees only a current context, its FBO, and a viewport on
 * entry to paintGL; all other state may be inherited from its compositor or a
 * prior QPainter pass. */
struct IndexedBufferSave
{
    GLint name = 0;
    GLint64 start = 0;
    GLint64 size = 0;
};

struct GlStateSave
{
    static constexpr int s_kTextureUnits = 12; /* material 0..9 + shadow 10..11 */
    GLint fbo, read_fbo, vp[4], prog, vao;
    GLint active_tex, tex2d[s_kTextureUnits], sampler[s_kTextureUnits];
    GLint ubuf, abuf, ebuf, pack_buf, unpack_buf, renderbuffer;
    GLint pack_alignment, pack_row_length, pack_skip_pixels, pack_skip_rows;
    GLint unpack_alignment, unpack_row_length, unpack_skip_pixels, unpack_skip_rows;
    IndexedBufferSave ubo_indexed[2];
    GLint front_face, cull_face_mode, depth_func;
    GLint blend_src_rgb, blend_dst_rgb, blend_src_alpha, blend_dst_alpha;
    GLint blend_eq_rgb, blend_eq_alpha;
    GLboolean color_mask[4], depth_mask;
    GLboolean depth, cull, blend, scissor, stencil, rasterizer_discard;
    GLboolean polygon_offset, sample_alpha_to_coverage, sample_coverage, dither;
#if defined(NUSD_DESKTOP_GL)
    GLboolean framebuffer_srgb, color_logic_op;
    GLboolean primitive_restart, depth_clamp;
    GLint primitive_restart_index, polygon_mode[2];
    std::vector<GLboolean> clip_distance;
#endif
    GLfloat clear[4], clear_depth, depth_range[2], blend_color[4];
    GLfloat polygon_offset_factor, polygon_offset_units;
};

static void glIndexedBufferSave(IndexedBufferSave* state, GLenum target, GLenum startTarget, GLenum sizeTarget, GLuint index)
{
    glGetIntegeri_v(target, index, &state->name);
    if (state->name)
    {
        glGetInteger64i_v(startTarget, index, &state->start);
        glGetInteger64i_v(sizeTarget, index, &state->size);
    }
}

static void glIndexedBufferRestore(const IndexedBufferSave* s, GLenum target, GLuint index)
{
    if (s->name && s->size > 0)
    {
        glBindBufferRange(target, index, (GLuint)s->name, (GLintptr)s->start, (GLsizeiptr)s->size);
    }
    else
    {
        glBindBufferBase(target, index, (GLuint)s->name);
    }
}

static void glStateSave(GlStateSave* s)
{
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &s->fbo);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &s->read_fbo);
    glGetIntegerv(GL_VIEWPORT, s->vp);
    glGetIntegerv(GL_CURRENT_PROGRAM, &s->prog);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &s->vao);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &s->active_tex);
    for (int i = 0; i < GlStateSave::s_kTextureUnits; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &s->tex2d[i]);
        glGetIntegerv(GL_SAMPLER_BINDING, &s->sampler[i]);
    }
    glActiveTexture((GLenum)s->active_tex);
    glGetIntegerv(GL_UNIFORM_BUFFER_BINDING, &s->ubuf);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &s->abuf);
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &s->ebuf);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &s->pack_buf);
    glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &s->unpack_buf);
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &s->renderbuffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &s->pack_alignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &s->pack_row_length);
    glGetIntegerv(GL_PACK_SKIP_PIXELS, &s->pack_skip_pixels);
    glGetIntegerv(GL_PACK_SKIP_ROWS, &s->pack_skip_rows);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &s->unpack_alignment);
    glGetIntegerv(GL_UNPACK_ROW_LENGTH, &s->unpack_row_length);
    glGetIntegerv(GL_UNPACK_SKIP_PIXELS, &s->unpack_skip_pixels);
    glGetIntegerv(GL_UNPACK_SKIP_ROWS, &s->unpack_skip_rows);
    glIndexedBufferSave(
        &s->ubo_indexed[0], GL_UNIFORM_BUFFER_BINDING, GL_UNIFORM_BUFFER_START, GL_UNIFORM_BUFFER_SIZE, 0);
    glIndexedBufferSave(
        &s->ubo_indexed[1], GL_UNIFORM_BUFFER_BINDING, GL_UNIFORM_BUFFER_START, GL_UNIFORM_BUFFER_SIZE, 1);
    glGetIntegerv(GL_FRONT_FACE, &s->front_face);
    glGetIntegerv(GL_CULL_FACE_MODE, &s->cull_face_mode);
    glGetIntegerv(GL_DEPTH_FUNC, &s->depth_func);
    glGetIntegerv(GL_BLEND_SRC_RGB, &s->blend_src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &s->blend_dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &s->blend_src_alpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &s->blend_dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &s->blend_eq_rgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &s->blend_eq_alpha);
    glGetBooleanv(GL_COLOR_WRITEMASK, s->color_mask);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &s->depth_mask);
    s->depth = glIsEnabled(GL_DEPTH_TEST);
    s->cull = glIsEnabled(GL_CULL_FACE);
    s->blend = glIsEnabled(GL_BLEND);
    s->scissor = glIsEnabled(GL_SCISSOR_TEST);
    s->stencil = glIsEnabled(GL_STENCIL_TEST);
    s->rasterizer_discard = glIsEnabled(GL_RASTERIZER_DISCARD);
    s->polygon_offset = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    s->sample_alpha_to_coverage = glIsEnabled(GL_SAMPLE_ALPHA_TO_COVERAGE);
    s->sample_coverage = glIsEnabled(GL_SAMPLE_COVERAGE);
    s->dither = glIsEnabled(GL_DITHER);
#if defined(NUSD_DESKTOP_GL)
    s->framebuffer_srgb = glIsEnabled(GL_FRAMEBUFFER_SRGB);
    s->color_logic_op = glIsEnabled(GL_COLOR_LOGIC_OP);
    s->primitive_restart = glIsEnabled(GL_PRIMITIVE_RESTART);
    s->depth_clamp = glIsEnabled(GL_DEPTH_CLAMP);
    glGetIntegerv(GL_PRIMITIVE_RESTART_INDEX, &s->primitive_restart_index);
    glGetIntegerv(GL_POLYGON_MODE, s->polygon_mode);
    GLint max_clip_distances = 0;
    glGetIntegerv(GL_MAX_CLIP_DISTANCES, &max_clip_distances);
    s->clip_distance.resize((size_t)max_clip_distances);
    for (GLint i = 0; i < max_clip_distances; ++i)
        s->clip_distance[(size_t)i] = glIsEnabled(GL_CLIP_DISTANCE0 + i);
#endif
    glGetFloatv(GL_COLOR_CLEAR_VALUE, s->clear);
    glGetFloatv(GL_DEPTH_CLEAR_VALUE, &s->clear_depth);
    glGetFloatv(GL_DEPTH_RANGE, s->depth_range);
    glGetFloatv(GL_BLEND_COLOR, s->blend_color);
    glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &s->polygon_offset_factor);
    glGetFloatv(GL_POLYGON_OFFSET_UNITS, &s->polygon_offset_units);
}

static void glSetEnabled(GLenum cap, GLboolean enabled)
{
    if (enabled)
        glEnable(cap);
    else
        glDisable(cap);
}

/* Establish the state the renderer actually assumes.  gpu_init used to set
 * some of this once, but the adopted path restores Qt's state after every
 * frame, so relying on initialization made all later frames caller-dependent. */
static void glStatePrepareForOvgl(const GlStateSave* caller_state = nullptr)
{
#if !defined(NUSD_DESKTOP_GL)
    (void)caller_state;
#endif
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_RASTERIZER_DISCARD);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glDisable(GL_SAMPLE_COVERAGE);
#if defined(NUSD_DESKTOP_GL)
    /* OVGL's shaders already gamma-encode their output. */
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_COLOR_LOGIC_OP);
    glDisable(GL_PRIMITIVE_RESTART);
    glDisable(GL_DEPTH_CLAMP);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLint max_clip_distances = caller_state ? (GLint)caller_state->clip_distance.size() : 0;
    if (!caller_state)
        glGetIntegerv(GL_MAX_CLIP_DISTANCES, &max_clip_distances);
    for (GLint i = 0; i < max_clip_distances; ++i)
        glDisable(GL_CLIP_DISTANCE0 + i);
#endif
    glEnable(GL_DITHER); /* match a fresh headless GL context */
    glDisable(GL_CULL_FACE); /* applyMeshSidedness sets it per mesh */
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    /* Empty background is BLACK, matching the measured official ovrtx 0.4
     * product output (rig frames 2026-07-18: official corner pixels (0,0,0);
     * the previous 0.66 gray read as (168,168,168) and alone dominated the
     * whole-frame MAE against the official engine). */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
#if defined(NUSD_DESKTOP_GL)
    glClearDepth(1.0);
    glDepthRange(0.0, 1.0);
#else
    glClearDepthf(1.0f);
    glDepthRangef(0.0f, 1.0f);
#endif
    /* Client pointers passed to texture upload must not be interpreted as PBO
     * offsets inherited from the embedding application. */
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    /* A sampler object overrides the filtering/wrap parameters on the texture
     * itself.  Inherited Qt/caller samplers must not affect OVGL units. */
    for (int i = 0; i < GlStateSave::s_kTextureUnits; ++i)
        glBindSampler((GLuint)i, 0);
    glActiveTexture(GL_TEXTURE0);
}

static void glStateRestore(const GlStateSave* s)
{
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, (GLuint)s->fbo);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, (GLuint)s->read_fbo);
    glViewport(s->vp[0], s->vp[1], s->vp[2], s->vp[3]);
    glUseProgram((GLuint)s->prog);
    glBindVertexArray((GLuint)s->vao);
#if defined(NUSD_DESKTOP_GL)
    if (s->vao)
#endif
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, (GLuint)s->ebuf);
    glIndexedBufferRestore(&s->ubo_indexed[0], GL_UNIFORM_BUFFER, 0);
    glIndexedBufferRestore(&s->ubo_indexed[1], GL_UNIFORM_BUFFER, 1);
    glBindBuffer(GL_UNIFORM_BUFFER, (GLuint)s->ubuf);
    glBindBuffer(GL_ARRAY_BUFFER, (GLuint)s->abuf);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, (GLuint)s->pack_buf);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, (GLuint)s->unpack_buf);
    glBindRenderbuffer(GL_RENDERBUFFER, (GLuint)s->renderbuffer);
    glPixelStorei(GL_PACK_ALIGNMENT, s->pack_alignment);
    glPixelStorei(GL_PACK_ROW_LENGTH, s->pack_row_length);
    glPixelStorei(GL_PACK_SKIP_PIXELS, s->pack_skip_pixels);
    glPixelStorei(GL_PACK_SKIP_ROWS, s->pack_skip_rows);
    glPixelStorei(GL_UNPACK_ALIGNMENT, s->unpack_alignment);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, s->unpack_row_length);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, s->unpack_skip_pixels);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, s->unpack_skip_rows);
    glFrontFace((GLenum)s->front_face);
    glCullFace((GLenum)s->cull_face_mode);
    glDepthFunc((GLenum)s->depth_func);
    glDepthMask(s->depth_mask);
    glColorMask(s->color_mask[0], s->color_mask[1], s->color_mask[2], s->color_mask[3]);
    glBlendFuncSeparate(
        (GLenum)s->blend_src_rgb, (GLenum)s->blend_dst_rgb, (GLenum)s->blend_src_alpha, (GLenum)s->blend_dst_alpha);
    glBlendEquationSeparate((GLenum)s->blend_eq_rgb, (GLenum)s->blend_eq_alpha);
    glBlendColor(s->blend_color[0], s->blend_color[1], s->blend_color[2], s->blend_color[3]);
    glPolygonOffset(s->polygon_offset_factor, s->polygon_offset_units);
    for (int i = 0; i < GlStateSave::s_kTextureUnits; ++i)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, (GLuint)s->tex2d[i]);
        glBindSampler((GLuint)i, (GLuint)s->sampler[i]);
    }
    glActiveTexture((GLenum)s->active_tex);
    glClearColor(s->clear[0], s->clear[1], s->clear[2], s->clear[3]);
#if defined(NUSD_DESKTOP_GL)
    glClearDepth((GLdouble)s->clear_depth);
    glDepthRange((GLdouble)s->depth_range[0], (GLdouble)s->depth_range[1]);
#else
    glClearDepthf(s->clear_depth);
    glDepthRangef(s->depth_range[0], s->depth_range[1]);
#endif
    glSetEnabled(GL_DEPTH_TEST, s->depth);
    glSetEnabled(GL_CULL_FACE, s->cull);
    glSetEnabled(GL_BLEND, s->blend);
    glSetEnabled(GL_SCISSOR_TEST, s->scissor);
    glSetEnabled(GL_STENCIL_TEST, s->stencil);
    glSetEnabled(GL_RASTERIZER_DISCARD, s->rasterizer_discard);
    glSetEnabled(GL_POLYGON_OFFSET_FILL, s->polygon_offset);
    glSetEnabled(GL_SAMPLE_ALPHA_TO_COVERAGE, s->sample_alpha_to_coverage);
    glSetEnabled(GL_SAMPLE_COVERAGE, s->sample_coverage);
    glSetEnabled(GL_DITHER, s->dither);
#if defined(NUSD_DESKTOP_GL)
    glSetEnabled(GL_FRAMEBUFFER_SRGB, s->framebuffer_srgb);
    glSetEnabled(GL_COLOR_LOGIC_OP, s->color_logic_op);
    glPrimitiveRestartIndex((GLuint)s->primitive_restart_index);
    glSetEnabled(GL_PRIMITIVE_RESTART, s->primitive_restart);
    glSetEnabled(GL_DEPTH_CLAMP, s->depth_clamp);
    /* Core profile exposes one front-and-back mode; the query returns the
     * same value in both slots and GL_FRONT/GL_BACK are invalid enums. */
    glPolygonMode(GL_FRONT_AND_BACK, (GLenum)s->polygon_mode[0]);
    for (size_t i = 0; i < s->clip_distance.size(); ++i)
        glSetEnabled(GL_CLIP_DISTANCE0 + (GLenum)i, s->clip_distance[i]);
#endif
}

static ovgl_result_t glStateFinishAdopted(const GlStateSave* s, ovgl_result_t rc, const char* where)
{
    /* Preserve the render/setup diagnostic if restoration also fails. */
    std::string primary_error = g_err;
    glStateRestore(s);
    bool restored = glErrorsOk(where);
    if (rc.status != 0)
    {
        g_err = primary_error;
        return rc;
    }
    return restored ? rc : fail(g_err);
}

/* Lazily create (or up-axis-rebuild) the grid overlay's pipeline + quad
 * vertex buffer. Called only from an ENABLED frame with GL current, so a
 * disabled renderer never allocates GL objects for the overlay. */
static bool ensureGridResources(ovgl_renderer_t* r)
{
    if (!r->grid_pipe)
    {
        GpuVertexAttrib attrs[4] = {
            { 0, 0, GPU_FORMAT_FLOAT3 }, /* world position       */
            { 1, 12, GPU_FORMAT_FLOAT2 }, /* ground-plane coords  */
            { 2, 20, GPU_FORMAT_FLOAT3 }, /* u-axis line color    */
            { 3, 32, GPU_FORMAT_FLOAT3 }, /* v-axis line color    */
        };
        GpuPipelineDesc gd{};
        gd.vert_glsl = g_kGridVertexShader;
        gd.frag_glsl = g_kGridFragmentShader;
        gd.vertex_stride = g_kGridVertexStride;
        gd.attribs = attrs;
        gd.nattribs = 4;
        r->grid_pipe = gpu_create_pipeline(r->gpu, &gd);
        if (!r->grid_pipe)
        {
            g_err = "gpu_create_pipeline (grid overlay) failed";
            return false;
        }
    }
    const int up_axis = r->scene.up_axis;
    if (!r->grid_vb || r->grid_vb_up_axis != up_axis)
    {
        if (r->grid_vb)
        {
            gpu_destroy_buffer(r->gpu, r->grid_vb);
            r->grid_vb = nullptr;
        }
        float verts[g_kGridVertexCount * g_kGridVertexFloats];
        buildGridQuad(up_axis, verts);
        GpuBufferDesc vd{ GPU_BUFFER_VERTEX, sizeof(verts), verts };
        r->grid_vb = gpu_create_buffer(r->gpu, &vd);
        if (!r->grid_vb)
        {
            g_err = "gpu_create_buffer (grid overlay quad) failed";
            return false;
        }
        r->grid_vb_up_axis = up_axis;
    }
    return true;
}

/* Draw the grid + axes overlay into the current render target. Runs between
 * the opaque and transparent scene passes: depth-TESTED (opaque geometry
 * occludes the grid, matching stock) with depth WRITES off (the grid can
 * never occlude scene content or leak into the Depth AOV), alpha-blended
 * over the frame so transparent geometry later blends on top of it. The
 * caller re-binds the material pipeline afterwards. */
static bool drawGridOverlay(ovgl_renderer_t* r, const M4& view, const M4& proj, const float eye3[3])
{
    if (!ensureGridResources(r))
        return false;
    gpu_cmd_bind_pipeline(r->gpu, r->grid_pipe);
    gpu_cmd_set_view_proj(r->gpu, view.m, proj.m);
    gpu_cmd_set_eye_pos(r->gpu, eye3);
    gpu_cmd_bind_vertex_buffer(r->gpu, r->grid_vb);
    glDisable(GL_CULL_FACE); /* plane is visible from both sides */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    /* Pull the editor grid slightly toward the camera so an authored floor on
     * the same mathematical plane cannot z-fight it into a blurred/dim pattern.
     * Depth testing remains enabled, so real geometry still occludes the grid. */
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);
    glDepthMask(GL_FALSE);
    gpu_cmd_draw(r->gpu, g_kGridVertexCount, 0);
    glDepthMask(GL_TRUE);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
    return glErrorsOk("grid overlay draw");
}

/* Make the GPU's environment match what this scene + host asked for. Called
 * once per frame with GL current (the environment creates GL textures) and
 * AFTER any scene rebuild, because the scene is where the DomeLight comes from.
 *
 * Precedence:
 *   1. ovgl_set_environment() -- an explicit host override, e.g. robot-usdview's
 *      OVGL_ENV_HDR inspection knob. A host that names an HDR means it.
 *   2. the scene's first authored UsdLuxDomeLight (Scene::has_dome). Textured
 *      domes go through the same HDR loader; a TEXTURELESS dome is a uniform
 *      environment of color * intensity * 2^exposure, which is the whole point:
 *      it is what makes official ovrtx render this scene's sky (222,223,225)
 *      where ovgl rendered (0,0,0) over all 252,166 background pixels.
 *   3. nothing -- the synthetic hemisphere rig + MuJoCo headlight, unchanged.
 *
 * An environment removes the synthetic rig only when it IS the scene's rig --
 * see env_owns_lighting below. Keying that on has_dome alone was a measured
 * regression (robot-usdview/welcome.usda authors a dome AND a DistantLight,
 * and lost 13% of its brightness to a 0.84-nit dome evicting the fill), and so
 * was the radiance threshold that replaced it: a threshold makes the frame
 * discontinuous in a number the author is entitled to sweep.
 *
 * Reloading is keyed on the resolved request, not on a dirty bit alone, so a
 * per-frame rebuild of an unchanged dome does not re-project SH and re-upload
 * textures every frame, while a live intensity/color edit does. Clearing
 * first means NULL really disables IBL, replacing an HDR cannot leak the
 * previous textures/program, and a failed replacement falls back cleanly
 * instead of retaining half of either environment. */
static void applyEnvironment(ovgl_renderer_t* r)
{
    int want_source = 0;
    std::string want_path;
    float want_color[3] = { 1.0f, 1.0f, 1.0f };
    float want_intensity = 1.0f;
    if (!r->env_path.empty())
    {
        want_source = 1;
        want_path = r->env_path;
        want_intensity = r->env_intensity;
    }
    else if (r->scene.has_dome)
    {
        want_source = 2;
        want_path = r->scene.dome_hdr_path;
        /* Floor at 0. A negative dome intensity is not a thing a light can
         * emit, and gpu.h reserves env_intensity < -1 as an INTERNAL
         * visible-dome marker the fragment shaders branch on -- an authored
         * negative would silently land on that sentinel. */
        want_intensity = r->scene.dome_intensity > 0.0f ? r->scene.dome_intensity : 0.0f;
        for (int c = 0; c < 3; c++)
            want_color[c] = r->scene.dome_color[c];
    }
    /* Every non-dome light prim the scene declares, whether or not ovgl has a
     * GpuLight::kind for it. Resolved here, above the early-out, because
     * env_owns_lighting is a function of exactly this count, so a light
     * appearing or disappearing has to re-open a dome request that is otherwise
     * byte-identical. Only source 2 reads it: a host-named HDR owns the frame
     * whatever else is authored, and a dome-free scene must keep resolving to
     * the no-environment state the renderer already holds (see the field
     * comment on env_applied_path) rather than be dragged through a pointless
     * destroy/reset every time a light count changes. */
    const int want_nlights =
        want_source == 2 ? static_cast<int>(r->scene_lights.size()) + r->scene.nlights_unsupported : 0;
    if (!r->env_dirty && want_source == r->env_source && want_path == r->env_applied_path &&
        want_intensity == r->env_applied_intensity && want_nlights == r->env_applied_nlights &&
        want_color[0] == r->env_applied_color[0] && want_color[1] == r->env_applied_color[1] &&
        want_color[2] == r->env_applied_color[2])
    {
        return;
    }
    r->env_dirty = false;
    r->env_source = want_source;
    r->env_applied_path = want_path;
    r->env_applied_intensity = want_intensity;
    r->env_applied_nlights = want_nlights;
    for (int c = 0; c < 3; c++)
        r->env_applied_color[c] = want_color[c];

    gpu_destroy_environment(r->gpu);
    gpu_set_fallback_lighting(r->gpu, 1);
    gpu_set_tone_mapping(r->gpu, 1.0f, 1.0f, 1.0f, 0u);
    r->env_loaded = false;
    r->env_failed = false;
    r->env_owns_lighting = false;
    if (want_source == 0)
        return;

    /* Sign of the intensity argument selects the texel convention (gl/
     * gpu.h): a DomeLight is RADIOMETRIC -- the authored number is baked into
     * the radiance texels and the shaders see a +1.0 marker -- while the host
     * override stays on the LEGACY auto-exposed path its 4.5/2000 tone pair
     * was calibrated against. */
    const float load_intensity = (want_source == 1) ? -want_intensity : want_intensity;
    bool ok = false;
    if (want_path.empty())
    {
        ok = gpu_load_environment_uniform(r->gpu, want_color, load_intensity) != 0;
    }
    else
    {
        isaacsim::ovgl_viewport::debug::details::ovgl::LocalAssetFile local_environment;
        std::string asset_error;
        if (!local_environment.open(want_path, asset_error))
        {
            gl_log(GL_LOG_ERROR, "ovgl", "%s", asset_error.c_str());
        }
        else
        {
            ok = gpu_load_environment_tinted_intensity(r->gpu, local_environment.getPath().c_str(), load_intensity,
                                                       want_source == 1 ? nullptr : want_color) != 0;
        }
    }
    if (!ok)
    {
        gpu_destroy_environment(r->gpu);
        gpu_set_fallback_lighting(r->gpu, 1);
        gpu_set_tone_mapping(r->gpu, 1.0f, 1.0f, 1.0f, 0u);
        r->env_failed = true; /* sticky: do not retry a bad HDR every frame */
        return;
    }
    gpu_set_environment_intensity(r->gpu, load_intensity);
    /* Tone mapping stays on the HOST override only. A radiometric dome needs no
     * lift at all -- that is the entire point of baking the authored intensity
     * into the texels: the sky and the surfaces then share one camera transform
     * and both are exactly linear in the authored number, so there is nothing
     * left for a per-source tone pair to compensate. The host override's
     * `intensity` is a different quantity (a viewer knob defaulting to 1.0 on
     * an auto-exposed HDR), so it keeps its long-standing lift. */
    if (want_source == 1)
        gpu_set_tone_mapping(r->gpu, 4.5f, 2000.0f, 1.0f, 0u);
    else
        gpu_set_tone_mapping(r->gpu, 1.0f, 1.0f, 1.0f, 0u);
    r->env_loaded = true;

    /* Does this environment actually LIGHT the frame, or is it merely present?
     *
     * A host that names an HDR always means it (source 1). For a DomeLight the
     * question is answered by ONE fact about the scene graph -- is the dome the
     * only light prim in it? -- and by NOTHING about how bright it is:
     *
     *     owns == (no other authored light prim exists)
     *
     * `scene_lights` holds the authored lights ovgl can evaluate and
     * `nlights_unsupported` the active ones it cannot (DiskLight, CylinderLight
     * -- there is no GpuLight::kind for either), so the two together are every
     * non-dome light prim in the scene and the predicate is their sum being 0.
     * This is the shape upstream uses (nanousd viewer.c:1620,
     * `gpu_set_fallback_lighting(gpu, has_authored_light ? 0 : 1)`): a boolean
     * fact about what the author put in the stage, no radiometric threshold.
     *
     * It REPLACES a `radiance >= 67.6 nits` gate that was a cliff, not a
     * partition. That gate switched off BOTH the headlight here and the
     * shader's hemisphere fill (u_envOwnsLighting, shaders_gles.h:983) the
     * instant a dome crossed it, and the two together are worth ~20 frame
     * luminance -- so the frame got DARKER as the author made the light
     * brighter. Measured on this bench, sweeping only inputs:intensity:
     *   robot_ground_scene 71 -> 72 (67.25 -> 68.20 nits): ovgl frame luminance
     *     80.742 -> 61.058 (-24%) while official went 95.033 -> 95.615 (+0.6%);
     *     cross-stack MAE 19.029 -> 34.997.
     *   franka_factory 67 -> 68 (its dome is white, so nits == intensity):
     *     87.091 -> 20.244 (-77%), MAE 68.994 -> 2.364.
     * Under the scene-graph predicate those same steps are 80.742 -> 81.379
     * (official 94.979 -> 95.615) and 20.004 -> 20.244, and both scenes are
     * monotone in intensity over 1..2000 with no reversal anywhere.
     *
     * The dark end is what proves the threshold was answering the wrong
     * question. franka_factory is dome-ONLY, and at intensity 1 official
     * renders it at frame luminance 0.029 -- essentially black, because the
     * attach lane has no synthetic rig at all. The 67.6-nit gate kept ovgl's
     * fill there and rendered 75.211, MAE 74.822 / 98.61% of pixels off by
     * >32. Owning by scene graph renders 0.001, MAE 0.046. A dome that is the
     * whole rig is the whole answer at EVERY intensity, including a dim one.
     *
     * Both halves of the sum are load-bearing, each measured by knocking it
     * out on the scene that needs it:
     *   - evaluable lights: robot-usdview/welcome.usda is a 0.84-nit dome plus
     *     a DistantLight. Letting the dome own it drops frame luminance
     *     107.060 -> 93.091 (-13%) and MAE 22.002 -> 32.748. The synthetic
     *     hemisphere there is not standing in for the dome; it stands in for
     *     the bounce ovgl does not compute, which is why the shader dims it to
     *     0.18x once a scene light exists (shaders_gles.h:836) rather than
     *     dropping it. A dome supplies no bounce.
     *   - unsupported lights: franka_factory_pickplace is a 226-nit dome plus a
     *     DiskLight of intensity 100 at exposure 5 that ovgl drops on the
     *     floor. Letting the dome own it gives MAE 57.116 / 83.73% against
     *     22.062 / 15.37%: a scene whose key light is invisible to this
     *     renderer is not a scene its dome can light on its own. */
    r->env_owns_lighting = (want_source == 1) || (want_nlights == 0);
    gpu_set_fallback_lighting(r->gpu, r->env_owns_lighting ? 0 : 1);
    gpu_set_environment_owns_lighting(r->gpu, r->env_owns_lighting ? 1 : 0);
    /* Host-override HDRs have no authored rotation; only a DomeLight does. */
    gpu_set_environment_rotation(r->gpu, want_source == 2 ? r->scene.dome_rotation_y : 0.0f);
}

/* Shared render core: draws the scene into r->resolve_fbo (the resolve_color TEXTURE) at
 * w×h, supersampled to iw×ih. GL must already be current and the FBOs ensured. Does NOT read
 * back or unbind to a default framebuffer. ovgl_render_frame (headless host readback) and
 * ovgl_render_to_texture (GUI, returns the texture) both call this. */
static ovgl_result_t renderSceneToResolve(ovgl_renderer_t* r, ovstage_ordinal_t ordinal, int w, int h, int iw, int ih, int ss)
{
    double t_xform0 = g_kProfilingEnabled ? nowMs() : 0.0;
    /* Rebuild from the sealed ovstage snapshot whenever its ordinal changes.
     * This correctness baseline covers topology, geometry, primitive params,
     * materials, visibility/purpose, lights, bounds, and transforms. A future
     * fast path may use explicit per-column value/layout generations, but a
     * worldMatrix-only refresh silently leaves most Property edits stale. */
    if (!r->topo_built || (!r->pull_paused && ordinal != r->last_ordinal))
    {
        double t_build0 = g_kProfilingEnabled ? nowMs() : 0.0;
        {
            std::lock_guard<std::mutex> lock(g_scene_build_mutex);
            if (!ovgl_scene_build(r->stage, ordinal, &r->scene))
                return fail(std::string("ovgl_scene_build: ") + ovgl_scene_last_error());
            if (!snapshotSceneAuxiliaries(r))
                return fail(g_err);
        }
        double t_glbuf0 = g_kProfilingEnabled ? nowMs() : 0.0;
        /* A topology rebuild replaces all material/mesh UBO state as well as
         * vertex/index buffers.  Reset it before upload so a failed attempt is
         * retryable and a later scene does not leak the prior resources. */
        gpu_destroy_materials(r->gpu);
        r->mesh_ubo_cap = 0;
        if (!rebuildGlBuffers(r))
            return fail(g_err);
        double t_matup0 = g_kProfilingEnabled ? nowMs() : 0.0;
        const int nmat = static_cast<int>(r->scene_materials.size());
        const int ntex = static_cast<int>(r->scene_textures.size());
        if (nmat > 0 && !gpu_upload_materials(r->gpu, r->scene_materials.data(), nmat,
                                              ntex > 0 ? r->scene_textures.data() : nullptr, ntex))
            return fail("gpu_upload_materials failed");
        if (!glErrorsOk("scene resource upload"))
            return fail(g_err);
        r->materials_dirty = false; /* the rebuild re-uploaded everything */
        if (g_kProfilingEnabled)
        {
            std::fprintf(stderr, "OVGLPROF rebuild build=%.3f glbuf=%.3f matup=%.3f\n", t_glbuf0 - t_build0,
                         t_matup0 - t_glbuf0, nowMs() - t_matup0);
        }

        r->topo_built = true;
        ++r->structure_gen;
        r->first_render = false;
    }
    /* Targeted material refresh (cheap path): ovgl_refresh_materials already
     * re-resolved the CPU-side snapshot; replace only the GPU material state
     * here, on the render thread, against unchanged geometry. Failure falls
     * back to the full-rebuild baseline on the next frame by re-marking the
     * ordinal stale. */
    if (r->materials_dirty)
    {
        const int nmat = static_cast<int>(r->scene_materials.size());
        const int ntex = static_cast<int>(r->scene_textures.size());
        if (nmat > 0 && !gpu_replace_materials(r->gpu, r->scene_materials.data(), nmat,
                                               ntex > 0 ? r->scene_textures.data() : nullptr, ntex))
        {
            r->last_ordinal = (ovstage_ordinal_t)-1;
            return fail("gpu_replace_materials failed");
        }
        if (!glErrorsOk("material refresh upload"))
        {
            r->last_ordinal = (ovstage_ordinal_t)-1;
            return fail(g_err);
        }
        r->materials_dirty = false;
    }
    /* The no-IBL haze fades geometry toward the backdrop over a distance measured in
     * scene radii, so it needs the scene's size.  Without this it uses absolute world
     * units tuned for a sim-sized scene, and an asset authored in centimetres (hundreds
     * of units across) renders as a flat wash into the gray backdrop.
     *
     * EVERY frame, not just a rebuild. It lived inside the rebuild block above
     * until 2026-07-25, which made the cheap path haze with whatever radius the
     * last FULL rebuild computed — while ovgl_scene_refresh_xforms recomputes
     * r->scene.bounds_* on exactly those frames, so the two disagreed. The
     * shader divides by it (`hazeDist = length(fragWorldPos - u_eyePos) /
     * sceneRadius`), so a stale radius perturbs precisely the pixels inside the
     * fade window and nothing else: that is the 566 px / max_abs=1 band at
     * y[73..106] across the full width of smoke_fastpath_delete_color's o6
     * frame, whose scene grows 16x2.0x16 -> 16x2.8x16 (radius 11.3578 ->
     * 11.3999, 0.37%) when the cube lifts. Only reachable with u_hasIBL == 0 —
     * a scene with a dome never runs this branch, which is why the two dome-free
     * fast-path fixtures were the ones that saw it. Recomputing per frame costs
     * one sqrt against a float store. */
    {
        const float* lo = r->scene.bounds_min;
        const float* hi = r->scene.bounds_max;
        const float ex = hi[0] - lo[0], ey = hi[1] - lo[1], ez = hi[2] - lo[2];
        const float rad = 0.5f * std::sqrt(ex * ex + ey * ey + ez * ez);
        gpu_set_scene_radius(r->gpu, std::isfinite(rad) ? rad : 0.0f);
    }
    /* After the rebuild, never before it: the scene owns the DomeLight, and
     * the light-upload block below has to know whether an environment took
     * over (it drops the synthetic headlight when one did). */
    applyEnvironment(r);
    r->last_ordinal = ordinal;
    const int n = r->scene.nmeshes;
    r->mesh_count = (size_t)n;
    double t_ubo0 = g_kProfilingEnabled ? nowMs() : 0.0;

    M4 view = lookAt(r->cam.eye, r->cam.target, r->cam.up);
    computeClipPlanes(r->cam.eye, r->scene.bounds_min, r->scene.bounds_max, &r->cur_znear, &r->cur_zfar);
    M4 proj = perspective((float)r->cam.fov_y_rad, (float)w / (float)h, r->cur_znear, r->cur_zfar);
    if (g_kProfilingEnabled)
    {
        std::fprintf(stderr,
                     "[ovgl] camera eye=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
                     "bounds=[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)] "
                     "clip=(%.4f, %.4f) meshes=%d\n",
                     r->cam.eye[0], r->cam.eye[1], r->cam.eye[2], r->cam.target[0], r->cam.target[1], r->cam.target[2],
                     r->scene.bounds_min[0], r->scene.bounds_min[1], r->scene.bounds_min[2], r->scene.bounds_max[0],
                     r->scene.bounds_max[1], r->scene.bounds_max[2], r->cur_znear, r->cur_zfar, n);
    }
    M4 vp = mul(proj, view);
    const GpuMaterialParams* scene_mats = r->scene_materials.empty() ? nullptr : r->scene_materials.data();
    const int scene_nmat = static_cast<int>(r->scene_materials.size());
    auto material_for_mesh = [&](const SceneMesh& mesh) -> const GpuMaterialParams*
    {
        if (!scene_mats || mesh.material_index < 0 || mesh.material_index >= scene_nmat)
        {
            return nullptr;
        }
        return &scene_mats[mesh.material_index];
    };
    auto is_transparent = [&](const SceneMesh& mesh)
    {
        const GpuMaterialParams* material = material_for_mesh(mesh);
        if (!material)
            return false;
        /* A positive opacity cutoff makes opacity binary. Surviving fragments
         * belong in the depth-writing opaque pass even when a map is present. */
        const bool alpha_cutout = material->opacity_threshold > 0.0f;
        return !alpha_cutout && (material->opacity < 0.999f || material->tex_indices[6] >= 0);
    };
    auto has_zero_scalar_opacity = [&](const SceneMesh& mesh)
    {
        const GpuMaterialParams* material = material_for_mesh(mesh);
        /* A non-positive opacity/base-alpha factor multiplies every fragment
         * to zero, including textured and cutout materials. Do not broadly
         * reject blend/cutout materials: nonzero translucent and masked
         * geometry still casts. */
        return material && (material->opacity <= 0.0f || material->base_color[3] <= 0.0f);
    };

    /* Per-frame lights: authored USD lights + (no-IBL) a MuJoCo-style HEADLIGHT -- a
     * directional fill aimed along the view so surfaces facing the camera stay
     * readable (MuJoCo's signature). The headlight casts no shadow; the sun (light 0)
     * does. When the environment OWNS the lighting, no headlight is added.
     *
     * "Owns the lighting" is r->env_owns_lighting, not merely env_loaded: a
     * host-named HDR always owns it, and an authored DomeLight owns it exactly
     * when it is the scene's ONLY light prim (applyEnvironment). Gating on
     * has_dome/env_loaded instead was a measured regression -- it dropped the
     * fill for robot-usdview/welcome.usda, which authors a dome AND a
     * DistantLight, costing it 13% of its frame luminance.
     *
     * The headlight is a SYNTHETIC stand-in that no USD prim asked for and it
     * follows the camera, so it cannot be part of a view-independent model.
     * Official ovrtx 0.4 has no counterpart ON THE ATTACH LANE -- measured on
     * this bench, robot_ground_scene with BOTH lights removed renders pure
     * black there (mean luminance 0.00, 1 unique color) while ovgl renders
     * 28.07, which is the 79%-of-lit-brightness term the ledger records. It
     * does have one on the FILE lane (open_usd_from_string): the same
     * light-free scene renders at 35.72 through official's own default
     * environment, against ovgl's 21.61. The headlight is therefore kept: it
     * is right for one official lane and wrong for the other, and removing it
     * would black out every lightless locomotion demo. */
    {
        GpuLight lts[GPU_MAX_SCENE_LIGHTS];
        int nl = 0;
        const int na = static_cast<int>(r->scene_lights.size());
        /* Authored lights own the capacity. Reserve a headlight slot only by
         * appending it when room remains; an IBL frame has no headlight at all,
         * and a no-IBL fallback must not evict the 32nd authored light. */
        for (int i = 0; i < na && nl < GPU_MAX_SCENE_LIGHTS; i++)
            lts[nl++] = r->scene_lights[static_cast<size_t>(i)];
        if (!r->env_owns_lighting && nl < GPU_MAX_SCENE_LIGHTS)
        {
            GpuLight hl;
            std::memset(&hl, 0, sizeof(hl));
            hl.kind = 1;
            hl.intensity = 3000.0f;
            hl.normalize = 1; /* ~MuJoCo headlight diffuse 0.4 */
            hl.color[0] = hl.color[1] = hl.color[2] = 1.0f;
            double vx = r->cam.target[0] - r->cam.eye[0], vy = r->cam.target[1] - r->cam.eye[1],
                   vz = r->cam.target[2] - r->cam.eye[2];
            double vn = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (vn < 1e-9)
                vn = 1.0;
            hl.normal[0] = (float)(vx / vn);
            hl.normal[1] = (float)(vy / vn);
            hl.normal[2] = (float)(vz / vn); /* shine along view */
            lts[nl++] = hl;
        }
        /* Zero is an important state: when the environment owns the lighting
         * there is no synthetic headlight, so removing the final USD light must
         * explicitly erase the prior GPU payload. `na`, unlike `nl`, excludes
         * the synthetic headlight. */
        if (!gpu_upload_lights(r->gpu, nl > 0 ? lts : nullptr, nl))
            return fail("gpu_upload_lights failed");
        gpu_set_authored_light_count(r->gpu, na);
        if (!r->env_owns_lighting && nl > 0)
            gpu_set_fallback_lighting(r->gpu, 0);
    }

    /* ---- Shadow-caster selection ----
     *
     * Up to GPU_MAX_SHADOW_LIGHTS authored lights get an atlas tile, taken in
     * SOURCE ORDER so the assignment is deterministic across frames and across
     * runs. The pre-atlas rule was "light 0, and only if it is a DistantLight":
     * on the spot orbit scene render_orbit's WRAP authors OrbitSun AND
     * OrbitFill and only the first could ever cast, and any scene whose first
     * light happened to be a Rect got no shadow at all.
     *
     * SphereLight is skipped on purpose (one frustum cannot cover a point
     * light; it needs a cube or dual-paraboloid map) and falls through to
     * unoccluded lighting rather than to a wrong single-frustum guess. */
    const GpuLight* slights = r->scene_lights.empty() ? nullptr : r->scene_lights.data();
    const int n_slights = static_cast<int>(r->scene_lights.size());
    int shadow_light_idx[GPU_MAX_SHADOW_LIGHTS];
    int n_shadow_lights = 0;
    if (sceneHasDrawableBounds(r->scene))
    {
        for (int i = 0; i < n_slights && n_shadow_lights < GPU_MAX_SHADOW_LIGHTS; i++)
        {
            if (slights[i].kind == 1 || slights[i].kind == 0)
                shadow_light_idx[n_shadow_lights++] = i;
        }
    }
    if (n_shadow_lights == 0)
    {
        /* Keep the allocated atlas for reuse, but make it impossible for a
         * Sphere/no-light replacement to sample a departed light's tile. */
        gpu_shadow_clear(r->gpu);
        r->last_shadow_ordinal = (ovstage_ordinal_t)-1;
    }
    else if (ordinal != r->last_shadow_ordinal)
    {
        /* Every tile is from its light's POV and the fit uses only scene
         * geometry, so the whole atlas stays camera-independent: it is
         * re-rendered on a scene edit (ordinal), and an orbit of N frames over
         * a static scene still pays for it exactly once. */
        double t_shadow0 = g_kProfilingEnabled ? nowMs() : 0.0;
        if (g_kProfilingEnabled)
        {
            const float* blo = r->scene.bounds_min;
            const float* bhi = r->scene.bounds_max;
            const float ex = bhi[0] - blo[0], ey = bhi[1] - blo[1], ez = bhi[2] - blo[2];
            std::fprintf(stderr,
                         "[ovgl] shadow fit input: scene %.2f x %.2f x %.2f m "
                         "(radius %.2f)\n",
                         ex, ey, ez, 0.5f * std::sqrt(ex * ex + ey * ey + ez * ez));
        }

        /* Caster box: the drawable meshes minus the environment-scale
         * outliers (see g_kShadowCoreRatio). Rank by AABB diagonal and take
         * everything within g_kShadowCoreRatio of the MEDIAN, which is robust to
         * a scene having several ground planes -- excluding "the largest mesh"
         * would keep the second one and fit to it. */
        float core_lo[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
        float core_hi[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
        {
            std::vector<float> diagonals;
            std::vector<int> drawn;
            try
            {
                diagonals.reserve(static_cast<size_t>(n));
                drawn.reserve(static_cast<size_t>(n));
            }
            catch (const std::bad_alloc&)
            {
                return fail("allocating shadow caster ranking failed");
            }
            for (int i = 0; i < n; i++)
            {
                const SceneMesh& mm = r->scene.meshes[i];
                if (!mm.visible || mm.is_proto_only || mm.nvertices <= 0 || has_zero_scalar_opacity(mm) ||
                    !finiteOrderedBounds(mm.bounds_min, mm.bounds_max))
                {
                    continue;
                }
                const float dx = mm.bounds_max[0] - mm.bounds_min[0];
                const float dy = mm.bounds_max[1] - mm.bounds_min[1];
                const float dz = mm.bounds_max[2] - mm.bounds_min[2];
                drawn.push_back(i);
                diagonals.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
            }
            float limit = FLT_MAX;
            if (!diagonals.empty())
            {
                std::vector<float> sorted = diagonals;
                std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
                const float median = sorted[sorted.size() / 2];
                if (median > 0.0f)
                    limit = median * g_kShadowCoreRatio;
            }
            for (size_t k = 0; k < drawn.size(); k++)
            {
                if (diagonals[k] > limit)
                    continue;
                const SceneMesh& mm = r->scene.meshes[drawn[k]];
                for (int a = 0; a < 3; a++)
                {
                    core_lo[a] = std::min(core_lo[a], mm.bounds_min[a]);
                    core_hi[a] = std::max(core_hi[a], mm.bounds_max[a]);
                }
            }
        }
        /* Every mesh an outlier (a scene of nothing but ground): fall back to
         * the full bounds, i.e. exactly the pre-atlas behavior. */
        if (!finiteOrderedBounds(core_lo, core_hi))
        {
            for (int a = 0; a < 3; a++)
            {
                core_lo[a] = r->scene.bounds_min[a];
                core_hi[a] = r->scene.bounds_max[a];
            }
        }

        /* The MeshBlock still holds the PREVIOUS frame's matrices at this
         * point in the frame; the color pass rewrites it below. The shadow
         * vertex shader reads mesh.model, so it needs this frame's models --
         * upload them here and let the color pass overwrite mvp/color/ptex
         * afterwards. Only `model` is read, so the rest is left at the
         * color pass's defaults rather than written twice. */
        if (r->mesh_ubo_cap < n)
        {
            if (!gpu_alloc_mesh_buffer(r->gpu, n))
                return fail("gpu_alloc_mesh_buffer failed");
            r->mesh_ubo_cap = n;
        }
        if (n > 0)
        {
            unsigned char* sb = (unsigned char*)gpu_begin_mesh_writes(r->gpu);
            if (!sb)
                return fail("gpu_begin_mesh_writes failed (shadow pass)");
            const int sstride = gpu_mesh_stride(r->gpu);
            for (int i = 0; i < n; i++)
            {
                const M4 model = modelFromWorld(r->scene.meshes[i].world_xform);
                GpuMeshData* dd = (GpuMeshData*)(sb + (size_t)i * sstride);
                std::memcpy(dd->model, model.m, sizeof(dd->model));
            }
            gpu_end_mesh_writes(r->gpu);
            if (!glErrorsOk("shadow mesh-data upload"))
                return fail(g_err);
        }

        int filled = 0;
        for (int s = 0; s < n_shadow_lights; s++)
        {
            const int li = shadow_light_idx[s];
            M4 lvp{};
            float bias = 0.0f;
            if (!buildShadowLightVp(slights[li], core_lo, core_hi, r->scene.bounds_min, r->scene.bounds_max,
                                    GPU_SHADOW_TILE_SIZE, &lvp, &bias))
            {
                continue;
            }
            if (g_kProfilingEnabled)
            {
                /* The two numbers that decide whether a shadow is visible at
                 * all: the world size of one tile texel, and the bias
                 * expressed back in world units. Both come straight out of
                 * the fitted projection (row 0 scale = 2/width, row 2 scale
                 * = -2/(far-near)), so this reports the matrix actually used. */
                const float ortho_w = 2.0f / std::fabs(lvp.m[0] != 0.0f ? lvp.m[0] : 1.0f);
                const float depth_span = 2.0f / std::fabs(lvp.m[10] != 0.0f ? lvp.m[10] : 1.0f);
                std::fprintf(stderr,
                             "[ovgl] shadow tile %d light %d kind %d: extent %.3f m, "
                             "texel %.4f m, bias %.5f (=%.4f m)\n",
                             filled, li, slights[li].kind, ortho_w, ortho_w / (float)GPU_SHADOW_TILE_SIZE, bias,
                             bias * depth_span);
            }
            if (!gpu_shadow_begin(r->gpu, filled, n_shadow_lights, lvp.m, li, bias))
                return fail("gpu_shadow_begin failed");
            gpu_cmd_bind_pipeline(r->gpu, r->shadow_pipe);
            for (int i = 0; i < n; i++)
            {
                const SceneMesh& mm = r->scene.meshes[i];
                if (!mm.visible || mm.is_proto_only || mm.nvertices <= 0 || has_zero_scalar_opacity(mm))
                {
                    continue;
                }
                if ((size_t)i >= r->mesh_vb.size() || !r->mesh_vb[i] || !r->mesh_ib[i])
                    continue;
                /* Same sidedness rule as the color pass on purpose: a face that
                 * gets drawn must also be able to occlude the light, or the
                 * shadow map disagrees with the image it is shading. */
                applyMeshSidedness(mm, r->backface_cull);
                gpu_cmd_bind_mesh_data(r->gpu, i);
                glBindVertexArray(r->mesh_vao[i]);
                gpu_cmd_draw_indexed(r->gpu, (uint32_t)mm.nindices, 0, 0);
            }
            gpu_shadow_end(r->gpu);
            if (!glErrorsOk("shadow pass"))
                return fail(g_err);
            filled++;
        }
        if (filled == 0)
            gpu_shadow_clear(r->gpu);
        if (g_kProfilingEnabled)
            std::fprintf(stderr, "[ovgl] shadow atlas: %d tile(s) %.2f ms\n", filled, nowMs() - t_shadow0);
        r->last_shadow_ordinal = ordinal;
    }

    /* Upload per-mesh transforms into the MeshBlock UBO (gl's bind path). */
    if (n > 0)
    {
        if (r->mesh_ubo_cap < n)
        {
            if (!gpu_alloc_mesh_buffer(r->gpu, n))
                return fail("gpu_alloc_mesh_buffer failed");
            r->mesh_ubo_cap = n;
        }
        unsigned char* base = (unsigned char*)gpu_begin_mesh_writes(r->gpu);
        if (!base)
            return fail("gpu_begin_mesh_writes failed");
        int stride = gpu_mesh_stride(r->gpu);
        for (int i = 0; i < n; i++)
        {
            const SceneMesh& m = r->scene.meshes[i];
            M4 model = modelFromWorld(m.world_xform);
            M4 mvp = mul(vp, model);
            GpuMeshData* d = (GpuMeshData*)(base + (size_t)i * stride);
            std::memcpy(d->mvp, mvp.m, sizeof(d->mvp));
            std::memcpy(d->model, model.m, sizeof(d->model));
            d->color[0] = m.has_display_color ? m.display_color[0] : 0.7f;
            d->color[1] = m.has_display_color ? m.display_color[1] : 0.7f;
            d->color[2] = m.has_display_color ? m.display_color[2] : 0.7f;
            d->color[3] = 1.0f; /* .w > 0.5 => use mesh.color (we have no vertex colors) */
            d->ptex[0] = d->ptex[2] = d->ptex[3] = 0xFFFFFFFFu;
            /* MuJoCo-style ground checker: an OPT-IN fallback convenience for
             * floor-named meshes with no appearance of their own. Off unless
             * OVGL_FLOOR_CHECKER=1, because official ovrtx 0.4 has no such
             * behavior and this silently repainted the ground of the spot,
             * anymal, go2 and g1 locomotion scenes, which all author the same
             * `def Plane "GroundCollider"`.
             *
             * `has_display_color` is the "no resolved appearance" test, not
             * merely "no displayColor": Scene.cpp:2372-2374 also sets it
             * when a bound render material resolves, so a mesh named
             * GroundSlab carrying a UsdPreviewSurface never checkers either.
             * That guard predates the switch and stays -- official renders the
             * rig's Ground cube with its authored displayColor, and the
             * checker overriding it was itself a measured parity defect.
             * Spot's GroundCollider still qualified because its only binding
             * is material:binding:physics, which is not a render material. */
            d->ptex[1] =
                (r->floor_checker && i < (int)r->mesh_is_floor.size() && r->mesh_is_floor[i] && !m.has_display_color) ?
                    1u :
                    0xFFFFFFFFu;
        }
        gpu_end_mesh_writes(r->gpu);
        if (!glErrorsOk("mesh-data upload"))
            return fail(g_err);
    }

    struct TransparentDraw
    {
        int mesh_index = 0;
        double view_depth = 0.0;
        bool finite = false;
    };
    std::vector<TransparentDraw> transparent_draws;
    try
    {
        transparent_draws.reserve(static_cast<size_t>(n));
        for (int index = 0; index < n; ++index)
        {
            const SceneMesh& mesh = r->scene.meshes[index];
            if (!mesh.visible || mesh.is_proto_only || mesh.nvertices <= 0 || !is_transparent(mesh))
            {
                continue;
            }
            const double center_x = (static_cast<double>(mesh.bounds_min[0]) + mesh.bounds_max[0]) * 0.5;
            const double center_y = (static_cast<double>(mesh.bounds_min[1]) + mesh.bounds_max[1]) * 0.5;
            const double center_z = (static_cast<double>(mesh.bounds_min[2]) + mesh.bounds_max[2]) * 0.5;
            const double depth = static_cast<double>(view.m[8]) * center_x + static_cast<double>(view.m[9]) * center_y +
                                 static_cast<double>(view.m[10]) * center_z + static_cast<double>(view.m[11]);
            transparent_draws.push_back(TransparentDraw{ index, depth, std::isfinite(depth) });
        }
        /* GL view space looks down -Z, so the more-negative finite depth is
         * farther and must draw first. stable_sort preserves scene order for
         * equal-depth and non-finite fallback entries, while the explicit
         * finite partition keeps NaNs out of the comparator. */
        std::stable_sort(transparent_draws.begin(), transparent_draws.end(),
                         [](const TransparentDraw& left, const TransparentDraw& right)
                         {
                             if (left.finite != right.finite)
                                 return left.finite;
                             if (!left.finite)
                                 return false;
                             return left.view_depth < right.view_depth;
                         });
    }
    catch (const std::bad_alloc&)
    {
        return fail("allocating transparent draw order failed");
    }

    /* ---- AO prepass: window depth + world normal, before anything is bound ----
     *
     * Placed HERE, after the MeshBlock upload above, precisely so it can reuse
     * that upload: it reads each mesh's camera `mvp` and needs no matrices of
     * its own. Opaque geometry only -- a transparent surface does not occlude
     * in this model, and letting it write depth here would carve an AO shadow
     * out of something the color pass then blends straight through. */
    double t_ao0 = g_kProfilingEnabled ? nowMs() : 0.0;
    if (r->ao_enabled && r->ao_pipe && n > 0)
    {
        if (!ensureAoTargets(r, iw, ih))
            return fail(g_err);
        glBindFramebuffer(GL_FRAMEBUFFER, r->ao_fbo);
        if (!glErrorsOk("bind AO prepass FBO") || !glFramebufferComplete(GL_FRAMEBUFFER, "AO prepass FBO before draw"))
            return fail(g_err);
        glViewport(0, 0, iw, ih);
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_BLEND);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        /* Clear to a +Z normal at the baseline clear depth of 1 (nothing).
         * Where no geometry is drawn the SSAO pass reads depth == 1.0 and skips
         * the pixel, so the cleared normal is never consumed; it is a defined
         * value rather than a meaningful one.
         *
         * Put the clear COLOR back immediately. gpu_begin_frame() issues the
         * color pass's glClear without setting one -- it inherits whatever
         * glStatePrepareForOvgl established, and that black is load-bearing:
         * it is the measured official ovrtx 0.4 empty background, and leaving
         * this pass's (0.5,0.5,1.0) behind would repaint every backgroundless
         * frame light blue. Clear depth is untouched, so the 1.0 the baseline
         * already set is what this clear uses. */
        glClearColor(0.5f, 0.5f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        gpu_cmd_bind_pipeline(r->gpu, r->ao_pipe);
        for (int index = 0; index < n; ++index)
        {
            const SceneMesh& mesh = r->scene.meshes[index];
            if (!mesh.visible || mesh.is_proto_only || mesh.nvertices <= 0 || is_transparent(mesh))
            {
                continue;
            }
            if ((size_t)index >= r->mesh_vb.size() || !r->mesh_vb[index] || !r->mesh_ib[index])
            {
                continue;
            }
            /* Same sidedness rule as the color and shadow passes: a face that
             * gets drawn must also be able to occlude. */
            applyMeshSidedness(mesh, r->backface_cull);
            gpu_cmd_bind_mesh_data(r->gpu, index);
            glBindVertexArray(r->mesh_vao[index]);
            gpu_cmd_draw_indexed(r->gpu, (uint32_t)mesh.nindices, 0, 0);
        }
        glBindVertexArray(0);
        if (!glErrorsOk("AO prepass"))
            return fail(g_err);

        /* SSAO at HALF the internal resolution -- for the default ss=2 that is
         * exactly the output resolution. The estimate is noisy by construction
         * and gets blurred and then sampled with LINEAR, so the halving costs
         * far less than it saves. */
        const int aw = (iw / 2) > 0 ? iw / 2 : 1;
        const int ah = (ih / 2) > 0 ? ih / 2 : 1;
        M4 proj_inv_ao = inverse(proj);
        if (gpu_ssao_compute(r->gpu, r->ao_depth, r->ao_normal, aw, ah, proj.m, proj_inv_ao.m, view.m, r->ao_radius,
                             r->ao_bias, r->ao_intensity))
        {
            gpu_set_ssao(r->gpu, gpu_ssao_result(r->gpu), r->ao_strength);
        }
        else
        {
            /* Fail closed: no AO texture bound means the material pass forces
             * strength to 0 and renders exactly as it did before SSAO. */
            gpu_set_ssao(r->gpu, 0, 0.0f);
        }
        if (!glErrorsOk("SSAO pass"))
            return fail(g_err);
        if (g_kProfilingEnabled)
            std::fprintf(stderr, "[ovgl] AO prepass %dx%d + SSAO %dx%d %.2f ms\n", iw, ih, aw, ah, nowMs() - t_ao0);
    }
    else
    {
        /* Disabled, or nothing to draw. Unbind rather than leave the previous
         * frame's AO applied to this one. */
        gpu_set_ssao(r->gpu, 0, 0.0f);
    }

    double t_draw0 = g_kProfilingEnabled ? nowMs() : 0.0;
    glBindFramebuffer(GL_FRAMEBUFFER, r->render_fbo); /* draw into the SS render target */
    if (!glErrorsOk("bind render FBO") || !glFramebufferComplete(GL_FRAMEBUFFER, "render FBO before draw"))
        return fail(g_err);
    if (!gpu_begin_frame(r->gpu))
        return fail("gpu_begin_frame failed");
    /* Environment background FIRST: gpu_draw_env_background disables the depth test
     * and writes only color (a full-screen pass), so it must run before the meshes
     * -- they then draw on top with depth testing on. */
    if (r->env_loaded)
    {
        M4 view_inv = inverse(view);
        M4 proj_inv = inverse(proj);
        gpu_draw_env_background(r->gpu, view_inv.m, proj_inv.m);
    }
    gpu_cmd_bind_pipeline(r->gpu, r->pipe);
    gpu_cmd_begin_material_pass(r->gpu);
    float eye3[3] = { (float)r->cam.eye[0], (float)r->cam.eye[1], (float)r->cam.eye[2] };
    gpu_cmd_set_eye_pos(r->gpu, eye3);
    r->tri_count = 0;
    auto draw_mesh = [&](int index)
    {
        const SceneMesh& mesh = r->scene.meshes[index];
        if ((size_t)index >= r->mesh_vb.size() || !r->mesh_vb[index] || !r->mesh_ib[index])
        {
            return;
        }
        applyMeshSidedness(mesh, r->backface_cull);
        gpu_cmd_bind_mesh_data(r->gpu, index);
        gpu_cmd_bind_material(r->gpu, mesh.material_index >= 0 ? mesh.material_index : 0);
        glBindVertexArray(r->mesh_vao[index]);
        gpu_cmd_draw_indexed(r->gpu, static_cast<uint32_t>(mesh.nindices), 0, 0);
        r->tri_count += static_cast<size_t>(mesh.nindices / 3);
    };
    /* Draw opaque geometry first to seed depth.  Transparent geometry then blends over it
     * without writing depth, so a direct MaterialX opacity value reaches the resolved frame. */
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    for (int index = 0; index < n; ++index)
    {
        const SceneMesh& mesh = r->scene.meshes[index];
        if (!mesh.visible || mesh.is_proto_only || mesh.nvertices <= 0 || is_transparent(mesh))
        {
            continue;
        }
        draw_mesh(index);
    }
    /* Grid + axes overlay: after opaque (their depth occludes it), before
     * transparent (which then blends OVER the grid). When disabled this
     * frame's GL command stream is bit-identical to a build without the
     * overlay — the hard no-effect contract in ovgl.h. */
    if (r->grid_overlay)
    {
        if (!drawGridOverlay(r, view, proj, eye3))
            return fail(g_err);
        /* Restore the material pipeline for the transparent pass. Program
         * uniforms are per-program state (still intact), but the bind also
         * re-applies gl's per-pipeline tone/light/shadow bookkeeping. */
        gpu_cmd_bind_pipeline(r->gpu, r->pipe);
        gpu_cmd_set_eye_pos(r->gpu, eye3);
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const TransparentDraw& draw : transparent_draws)
        draw_mesh(draw.mesh_index);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    gpu_end_frame(r->gpu);

    /* In profile mode, force a GPU sync here so t_draw measures GPU render time
     * and t_readback measures only the glReadPixels transfer (normally
     * glReadPixels implicitly absorbs both). */
    if (g_kProfilingEnabled)
        glFinish();
    double t_read0 = g_kProfilingEnabled ? nowMs() : 0.0;

    /* GPU downscale-resolve: blit (scale ss->1 + vertical flip) the iw×ih render target into
     * the w×h resolve TEXTURE. GL_LINEAR at an exact ss:1 ratio is the ss×ss box average; the
     * inverted dst-Y rect (0,h -> w,0) flips GL's bottom-up origin for free. Leaves the result
     * in resolve_color; the caller reads it back (host) or samples it (GUI). */
    if (!glErrorsOk("scene draw before resolve blit"))
        return fail(g_err);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, r->render_fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, r->resolve_fbo);
    if (!glErrorsOk("bind resolve-blit FBOs") || !glFramebufferComplete(GL_READ_FRAMEBUFFER, "resolve-blit source FBO") ||
        !glFramebufferComplete(GL_DRAW_FRAMEBUFFER, "resolve-blit destination FBO"))
        return fail(g_err);
    glBlitFramebuffer(0, 0, iw, ih, 0, h, w, 0, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    if (!glErrorsOk("resolve glBlitFramebuffer"))
        return fail(g_err);
    if (g_kProfilingEnabled)
    {
        glFinish();
        double t_end = nowMs();
        std::fprintf(stderr, "OVGLPROF res=%dx%d ss=%d xform=%.3f ubo=%.3f gpu=%.3f blit=%.3f total=%.3f\n", w, h, ss,
                     t_ubo0 - t_xform0, t_draw0 - t_ubo0, t_read0 - t_draw0, t_end - t_read0, t_end - t_xform0);
    }
    return ok();
}

extern "C" ovgl_result_t ovgl_render_frame(ovgl_renderer_t* r, ovstage_ordinal_t ordinal, void* out_rgba8_host, int w, int h)
{
    if (!r)
        return fail("null renderer");
    /* Hand any suspended host GLX context back on EVERY exit from here. */
    ForeignGlGuard foreign_gl_guard;
    /* Whatever happens below, a previously captured depth plane no longer
     * describes the newest frame; only a successful capture re-arms it. */
    r->have_depth = false;
    if (!r->stage)
        return fail("no stage attached");
    if (!r->cam_valid)
        return fail("no camera set");
    if (!out_rgba8_host)
        return fail("bad output buffer");
    int iw = 0, ih = 0, ss = 1;
    if (!computeRenderDimensions(r, w, h, &iw, &ih, &ss))
        return fail(g_err);
    if (!ensureGl(r, iw, ih, false))
        return fail(g_err);
    if (!egl_headless_make_current(r->egl))
        return fail("egl_headless_make_current failed");
    if (!glErrorsOk("headless render entry"))
        return fail(g_err);
    glStatePrepareForOvgl();
    if (!glErrorsOk("headless render state baseline"))
        return fail(g_err);
    if (!ensureFbos(r, iw, ih, w, h))
        return fail(g_err);
    ovgl_result_t rc = renderSceneToResolve(r, ordinal, w, h, iw, ih, ss);
    if (rc.status != 0)
        return rc;
    /* Read the resolve texture back into the caller's host RGBA8 buffer. */
    glBindFramebuffer(GL_READ_FRAMEBUFFER, r->resolve_fbo);
    if (r->async_readback < 0)
    {
        const char* e = getenv("OVGL_ASYNC_READBACK");
        r->async_readback = (e && e[0] == '1') ? 1 : 0;
    }
    if (r->async_readback && r->pack_pbo[0])
    {
        /* Live GUI path: glReadPixels packs into pbo[cur] without blocking; the host copy maps
         * pbo[prev] -- the PREVIOUS frame, whose transfer the GPU finished during this frame --
         * so the GPU->CPU stall is hidden at a 1-frame latency (first frame is cleared). */
        const size_t sz = (size_t)w * (size_t)h * 4;
        const int prev = 1 - r->pack_cur;
        glBindBuffer(GL_PIXEL_PACK_BUFFER, r->pack_pbo[r->pack_cur]);
        glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, (void*)0);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, r->pack_pbo[prev]);
        if (r->pack_primed)
        {
            void* p = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, (GLsizeiptr)sz, GL_MAP_READ_BIT);
            if (p)
            {
                std::memcpy(out_rgba8_host, p, sz);
                glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            }
            else
            {
                std::memset(out_rgba8_host, 0, sz);
            }
        }
        else
        {
            std::memset(out_rgba8_host, 0, sz); /* no prior frame yet */
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        r->pack_cur = prev; /* next frame maps what we just kicked */
        r->pack_primed = true;
    }
    else
    {
        /* Headless contract: this ordinal's pixels are in out_rgba8_host when we return. */
        glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, out_rgba8_host);
    }
    if (!glErrorsOk("headless glReadPixels"))
        return fail(g_err);
    if (r->depth_capture)
    {
        /* Depth AOV: pack THIS frame's depth buffer (24-bit lossless) into an
         * RGBA8 target, read it back, and linearize to image-plane distance
         * in stage units (+inf where the frame drew nothing). Synchronous by
         * design; under OVGL_ASYNC_READBACK=1 the color above lags one frame
         * while depth stays current — that opt-in GUI mode trades exactness
         * for latency and is never used by the render-var backend. */
        r->depth_pack_staging.resize(static_cast<size_t>(w) * h * 4);
        if (!gpu_depth_pack_read(r->gpu, r->render_depth, w, h, r->depth_pack_staging.data()))
            return fail("depth capture: gpu_depth_pack_read failed");
        r->depth_host.resize(static_cast<size_t>(w) * h);
        const double zn = r->cur_znear, zf = r->cur_zfar; /* the planes THIS frame drew with */
        const unsigned char* px = r->depth_pack_staging.data();
        for (size_t i = 0, np = static_cast<size_t>(w) * h; i < np; ++i, px += 4)
        {
            const uint32_t s = (static_cast<uint32_t>(px[0]) << 16) | (static_cast<uint32_t>(px[1]) << 8) | px[2];
            if (s >= 16777215u)
            {
                r->depth_host[i] = std::numeric_limits<float>::infinity();
            }
            else
            {
                const double zw = s / 16777215.0; /* window depth in [0,1) */
                r->depth_host[i] = static_cast<float>(zn * zf / (zf - zw * (zf - zn)));
            }
        }
        r->depth_w = w;
        r->depth_h = h;
        r->have_depth = true;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return ok();
}

/* GUI / zero-readback path: render in the CALLER's already-current GL context (e.g. a Qt
 * QOpenGLWidget paintGL) and return the resolve color texture for the caller to draw with a
 * fullscreen quad -- no glReadPixels, no host buffer, no EGL context switch. The GL state the
 * render touches is saved + restored so it cannot corrupt the caller's compositor. */
extern "C" ovgl_result_t ovgl_render_to_texture(
    ovgl_renderer_t* r, ovstage_ordinal_t ordinal, int w, int h, unsigned* out_gl_texture, int* out_w, int* out_h)
{
    if (!r)
        return fail("null renderer");
    /* Adopted paths never capture depth; drop any older plane so it cannot
     * be paired with the newer frame this call produces. */
    r->have_depth = false;
    if (!r->stage)
        return fail("no stage attached");
    if (!r->cam_valid)
        return fail("no camera set");
    if (!out_gl_texture)
        return fail("bad output");
    int iw = 0, ih = 0, ss = 1;
    if (!computeRenderDimensions(r, w, h, &iw, &ih, &ss))
        return fail(g_err);
    if (!validateAdoptedContext(r, "ovgl_render_to_texture"))
        return fail(g_err);
    if (!ensureGlFunctionsLoaded())
        return fail(g_err);
    /* Error flags are not restorable GL state.  Drain caller-owned errors at
     * the boundary so capture validation only diagnoses commands OVGL issued. */
    glDiscardErrors();
    GlStateSave gs{};
    glStateSave(&gs);
    ovgl_result_t rc;
    if (!glErrorsOk("ovgl_render_to_texture state capture"))
    {
        rc = fail(g_err);
    }
    else
    {
        glStatePrepareForOvgl(&gs);
        bool setup = glErrorsOk("ovgl_render_to_texture state baseline") && ensureGl(r, iw, ih, true) &&
                     ensureFbos(r, iw, ih, w, h);
        rc = setup ? renderSceneToResolve(r, ordinal, w, h, iw, ih, ss) : fail(g_err);
    }
    rc = glStateFinishAdopted(&gs, rc, "ovgl_render_to_texture state restore");
    if (rc.status != 0)
        return rc;
    *out_gl_texture = r->resolve_color;
    if (out_w)
        *out_w = w;
    if (out_h)
        *out_h = h;
    return ok();
}

/* GUI present: render against `ordinal` in the caller's current GL context, aspect-fit the
 * result into `dst_fbo`, and optionally compose an overlay. dst_fbo must be single-sample.
 * NO host readback, NO context switch. */
extern "C" ovgl_result_t ovgl_render_to_fbo(ovgl_renderer_t* r,
                                            ovstage_ordinal_t ordinal,
                                            unsigned dst_fbo,
                                            int w,
                                            int h,
                                            int source_x0,
                                            int source_y0,
                                            int source_x1,
                                            int source_y1,
                                            int destination_w,
                                            int destination_h,
                                            const char* overlay_text)
{
    if (!r)
        return fail("null renderer");
    /* Adopted paths never capture depth (see ovgl_render_to_texture). */
    r->have_depth = false;
    if (!r->stage)
        return fail("no stage attached");
    if (!r->cam_valid)
        return fail("no camera set");
    if (source_x0 < 0 || source_y0 < 0 || source_x1 <= source_x0 || source_y1 <= source_y0 || source_x1 > w ||
        source_y1 > h || destination_w <= 0 || destination_h <= 0)
        return fail("invalid source or destination dimensions");
    int iw = 0, ih = 0, ss = 1;
    if (!computeRenderDimensions(r, w, h, &iw, &ih, &ss))
        return fail(g_err);
    if (!validateAdoptedContext(r, "ovgl_render_to_fbo"))
        return fail(g_err);
    if (!ensureGlFunctionsLoaded())
        return fail(g_err);
    /* Error flags are not restorable GL state.  Drain caller-owned errors at
     * the boundary so capture validation only diagnoses commands OVGL issued. */
    glDiscardErrors();
    GlStateSave gs{};
    glStateSave(&gs);
    ovgl_result_t rc;
    if (!glErrorsOk("ovgl_render_to_fbo state capture"))
    {
        rc = fail(g_err);
    }
    else
    {
        glStatePrepareForOvgl(&gs);
        bool setup =
            glErrorsOk("ovgl_render_to_fbo state baseline") && ensureGl(r, iw, ih, true) && ensureFbos(r, iw, ih, w, h);
        rc = setup ? renderSceneToResolve(r, ordinal, w, h, iw, ih, ss) : fail(g_err);
    }
    if (rc.status == 0)
    {
        /* Present: blit the resolve (top-down, as the host path wants it) into the caller's
         * FBO with the destination Y inverted so it lands GL bottom-up and displays upright. */
        if (dst_fbo != 0 && !glIsFramebuffer(dst_fbo))
        {
            rc = fail("present destination is not a framebuffer object");
        }
        else
        {
            glBindFramebuffer(GL_READ_FRAMEBUFFER, r->resolve_fbo);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dst_fbo);
            if (!glErrorsOk("bind present FBOs") || !glFramebufferComplete(GL_READ_FRAMEBUFFER, "present source FBO") ||
                !glFramebufferComplete(GL_DRAW_FRAMEBUFFER, "present destination FBO"))
            {
                rc = fail(g_err);
            }
            else
            {
                GLint dst_samples = 0;
                glGetIntegerv(GL_SAMPLES, &dst_samples);
                if (!glErrorsOk("query present destination samples"))
                {
                    rc = fail(g_err);
                }
                else if (dst_samples != 0)
                {
                    char msg[160];
                    std::snprintf(msg, sizeof(msg), "present destination must be single-sample (got %d samples)",
                                  (int)dst_samples);
                    rc = fail(msg);
                }
                else
                {
                    const int source_w = source_x1 - source_x0;
                    const int source_h = source_y1 - source_y0;
                    const double scale = std::min(
                        static_cast<double>(destination_w) / source_w, static_cast<double>(destination_h) / source_h);
                    const int presented_w =
                        std::max(1, std::min(destination_w, static_cast<int>(std::lround(source_w * scale))));
                    const int presented_h =
                        std::max(1, std::min(destination_h, static_cast<int>(std::lround(source_h * scale))));
                    const int destination_x = (destination_w - presented_w) / 2;
                    const int destination_y = (destination_h - presented_h) / 2;
                    glViewport(0, 0, destination_w, destination_h);
                    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                    glClear(GL_COLOR_BUFFER_BIT);
                    glBlitFramebuffer(source_x0, source_y0, source_x1, source_y1, destination_x,
                                      destination_y + presented_h, destination_x + presented_w, destination_y,
                                      GL_COLOR_BUFFER_BIT, GL_LINEAR);
                    if (!glErrorsOk("present glBlitFramebuffer"))
                        rc = fail(g_err);
                    else if (overlay_text && overlay_text[0] != '\0')
                    {
                        if (!r->overlay_initialized)
                            r->overlay_initialized = gpu_overlay_init(r->gpu) != 0;
                        if (!r->overlay_initialized)
                        {
                            rc = fail("initializing present overlay failed");
                        }
                        else
                        {
                            const float text_width = static_cast<float>(std::strlen(overlay_text) * 8);
                            gpu_overlay_rect(r->gpu, 10.0f, 10.0f, text_width + 12.0f, 28.0f, 0.0f, 0.0f, 0.0f, 1.0f);
                            gpu_overlay_text(r->gpu, 16.0f, 16.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, overlay_text);
                            gpu_overlay_flush(r->gpu, destination_w, destination_h);
                            if (!glErrorsOk("present overlay"))
                                rc = fail(g_err);
                        }
                    }
                }
            }
        }
    }
    return glStateFinishAdopted(&gs, rc, "ovgl_render_to_fbo state restore");
}

/* --- read-only scene views for sibling consumers -------------------------- */

extern "C" int32_t ovgl_get_mesh_count(ovgl_renderer_t* r)
{
    if (!r)
    {
        fail("ovgl_get_mesh_count: null renderer");
        return -1;
    }
    if (!r->topo_built)
    {
        fail("ovgl_get_mesh_count: topology is not built yet (render once first)");
        return -1;
    }
    return (int32_t)r->scene.nmeshes;
}

extern "C" ovgl_result_t ovgl_get_mesh_view(ovgl_renderer_t* r, int32_t index, ovgl_mesh_view_t* out)
{
    if (!r || !out)
        return fail("ovgl_get_mesh_view: null arg");
    if (!r->topo_built)
        return fail("ovgl_get_mesh_view: topology is not built yet (render once first)");
    if (index < 0 || index >= r->scene.nmeshes)
        return fail("ovgl_get_mesh_view: index out of range");

    const SceneMesh& m = r->scene.meshes[index];
    out->positions = m.positions;
    out->vertex_count = (uint32_t)(m.nvertices > 0 ? m.nvertices : 0);
    out->indices = m.indices;
    out->index_count = (uint32_t)(m.nindices > 0 ? m.nindices : 0);
    std::memcpy(out->world_xform, m.world_xform, sizeof(out->world_xform));
    out->path = m.path;
    out->visible = m.visible;
    out->is_proto_only = m.is_proto_only;
    return ok();
}

/* --- Depth AOV (contract in ovgl.h) --------------------------------------- */

extern "C" ovgl_result_t ovgl_set_depth_capture(ovgl_renderer_t* r, int enabled)
{
    if (!r)
        return fail("ovgl_set_depth_capture: null renderer");
    r->depth_capture = enabled ? 1 : 0;
    /* Disabling is also an invalidation: a consumer that turned capture off
     * must not read a plane from an older frame. */
    if (!r->depth_capture)
        r->have_depth = false;
    return ok();
}

extern "C" ovgl_result_t ovgl_get_depth_view(ovgl_renderer_t* r, const float** out_depth, int* out_width, int* out_height)
{
    if (!r || !out_depth)
        return fail("ovgl_get_depth_view: null arg");
    if (!r->have_depth)
        return fail(
            "ovgl_get_depth_view: no captured depth plane "
            "(enable ovgl_set_depth_capture, then render a headless frame)");
    *out_depth = r->depth_host.data();
    if (out_width)
        *out_width = r->depth_w;
    if (out_height)
        *out_height = r->depth_h;
    return ok();
}

extern "C" ovgl_result_t ovgl_get_stats(ovgl_renderer_t* r, ovgl_stats_t* s)
{
    if (!r || !s)
        return fail("null arg");
    s->mesh_count = r->mesh_count;
    s->triangle_count = r->tri_count;
    s->last_render_ordinal = r->last_ordinal;
    s->last_structure_gen = r->structure_gen;
    s->bounds_valid = r->topo_built && sceneHasDrawableBounds(r->scene);
    s->up_axis = r->scene.up_axis;
    for (int i = 0; i < 3; ++i)
    {
        s->bounds_min[i] = r->scene.bounds_min[i];
        s->bounds_max[i] = r->scene.bounds_max[i];
        s->bounds_valid = s->bounds_valid && std::isfinite(s->bounds_min[i]) && std::isfinite(s->bounds_max[i]) &&
                          s->bounds_min[i] <= s->bounds_max[i];
    }
    return ok();
}

extern "C" ovx_string_t ovgl_get_last_error(void)
{
    ovx_string_t out{};
    out.ptr = g_err.c_str();
    out.length = g_err.size();
    return out;
}
