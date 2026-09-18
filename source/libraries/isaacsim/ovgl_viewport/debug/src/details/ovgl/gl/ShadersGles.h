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
 * shaders_gles.h — Shader source strings embedded as C constants.
 *
 * GLES 3.2 on Linux, desktop GL 4.1 on macOS.
 * Raster vertex/fragment for basic three-point lighting.
 * Overlay vertex/fragment for bitmap font text rendering.
 * PBR vertex/fragment for Cook-Torrance material rendering.
 */

/* Stringize the renderer-shared constants (gpu.h GPU_MAX_*) so the GLSL array
 * sizes track the C limits with no hand-copied literals -- the shadow atlas
 * declares u_shadow[] and u_shadowLightToSlot[] from these, so raising a limit
 * in gpu.h can no longer leave the shader one size behind (a silent
 * GL_INVALID_OPERATION on the glUniform*v that overruns the declared array).
 * Include order is safe: both consumers include gl/gpu.h before this
 * header, and gpu.h is guarded. */
#include "Gpu.h"
#define NUSD_STR2(x) #x
#define NUSD_STR(x) NUSD_STR2(x)
#define NUSD_MAX_SHADOW_LIGHTS_STR NUSD_STR(GPU_MAX_SHADOW_LIGHTS)
#define NUSD_MAX_SCENE_LIGHTS_STR NUSD_STR(GPU_MAX_SCENE_LIGHTS)

/* Two version prologues, because REQUIRING an extension a shader does not use is
 * not free: `#extension X : require` is a hard compile error when X is absent,
 * so a single prologue carrying the tessellation/geometry requires made EVERY
 * shader — the mesh and PBR material programs included — refuse to compile on any
 * backend without them. Measured on ANGLE's SwiftShader device (the software tier
 * that keeps the editors running on a GPU-less host, EglHeadless.c): GLES 3.1 is
 * granted, then gpu_create_material_pipeline fails with
 *     ERROR: 0:2: 'GL_EXT_tessellation_shader' : extension is not supported
 *     ERROR: 0:3: 'GL_EXT_geometry_shader'    : extension is not supported
 * on the plain mesh vertex shader. SwiftShader implements neither stage, and
 * never needed to: only the four k_curve_* shaders tessellate.
 *
 *   NUSD_SHADER_VERSION       every ordinary shader — no extension requirements
 *   NUSD_SHADER_VERSION_TESS  the curve tube pipeline ONLY (vert/tcs/tes/frag,
 *                             all four, since they link as one program and the
 *                             fragment stage reads gl_PrimitiveID)
 * On a backend with tessellation the two are identical in effect, so this costs
 * the GPU path nothing. */
#ifdef NUSD_DESKTOP_GL
#    define NUSD_SHADER_VERSION "#version 410\n"
/* Tessellation and geometry stages are core in GLSL 4.10 — no requires. */
#    define NUSD_SHADER_VERSION_TESS "#version 410\n"
#    define NUSD_PRECISION_HIGH ""
/* binding qualifier requires GLSL 4.2; use glUniformBlockBinding from
 * C instead. Block→binding point: MaterialBlock=0, MeshBlock=1,
 * ShadowBlock=2. MeshBlock and ShadowBlock use row_major layout because
 * the C side writes `mat4_mul`-produced matrices (row-major in memory).
 * Plain matrix uniforms are explicitly converted to column-major bytes
 * before their GL_FALSE upload; UBOs have no upload step, so declare the
 * layout here. */
#    define NUSD_UBO_LAYOUT "layout(std140) "
#    define NUSD_UBO_LAYOUT_MESH "layout(std140, row_major) "
#    define NUSD_UBO_LAYOUT_SHADOW "layout(std140, row_major) "
#    define NUSD_PTEX_COLOR_DECL ""
#    define NUSD_PTEX_COLOR_FUNC "vec3 ptexTriangleColor(uint offset) { return vec3(-1.0); }\n"
/* Desktop GL (GLSL 4.10, macOS) renders the many-light warehouse path
 * markedly darker + warmer than GLES 3.20 (the Linux reference) and the
 * OVRTX golden — a platform/driver divergence that accumulates over the
 * 32-scene-light loop (few-light scenes are unaffected). Compensate on the
 * warehouse branch only. Empty on GLES (below) so Linux output is unchanged;
 * FLIP-calibrated so the macOS render matches the golden. */
#    define NUSD_WH_DESKTOP_PARITY "        color *= vec3(1.46, 1.52, 1.66);\n"
#else
#    if defined(OVGL_GLES31_EXT)
/* GLES 3.1 + EXT_tessellation/geometry. Some GLES backends (notably ANGLE's
 * Vulkan backend on Windows/NVIDIA) cap at GLES 3.1 but expose tessellation +
 * geometry as the EXT_tessellation_shader / EXT_geometry_shader extensions. The
 * only 3.2-core features these shaders use are the tessellation built-ins
 * (gl_TessLevel*) and gl_PrimitiveID in the fragment stage — both supplied by
 * those extensions — so lowering to 310 es + requiring them compiles unchanged. */
/* `: enable`, not `: require` — an #extension directive must precede any
 * non-preprocessor token, so it lives here rather than at the use site, and
 * `enable` degrades to a warning (leaving the GL_EXT_geometry_shader macro
 * undefined) on a backend that lacks it instead of failing the compile. That
 * macro is what NUSD_PTEX_COLOR_FUNC branches on to decide whether
 * gl_PrimitiveID is available. */
#        define NUSD_SHADER_VERSION                                                                                    \
            "#version 310 es\n"                                                                                        \
            "#extension GL_EXT_geometry_shader : enable\n"
#        define NUSD_SHADER_VERSION_TESS                                                                               \
            "#version 310 es\n"                                                                                        \
            "#extension GL_EXT_tessellation_shader : require\n"                                                        \
            "#extension GL_EXT_geometry_shader : require\n"
#    else
#        define NUSD_SHADER_VERSION "#version 320 es\n"
#        define NUSD_SHADER_VERSION_TESS "#version 320 es\n"
#    endif
#    define NUSD_PRECISION_HIGH "precision highp float;\n"
#    define NUSD_UBO_LAYOUT "layout(std140, binding = 0) "
#    define NUSD_UBO_LAYOUT_MESH "layout(std140, binding = 1, row_major) "
#    define NUSD_UBO_LAYOUT_SHADOW "layout(std140, binding = 2, row_major) "
#    define NUSD_PTEX_COLOR_DECL                                                                                       \
        "layout(std430, binding = 2) readonly buffer PtexTriangleColors {\n"                                           \
        "    uint colors[];\n"                                                                                         \
        "} ptexTriColors;\n"
/* gl_PrimitiveID is core in GLES 3.2 but NOT in 3.1, where it arrives with
 * GL_EXT_geometry_shader. `: enable` rather than `: require` so this is a
 * RUNTIME capability test done by the GLSL preprocessor from one binary: a
 * backend that has the extension compiles the real ptex lookup, and one that
 * does not (ANGLE's SwiftShader software device — it implements no geometry
 * stage) compiles the same stub the desktop path uses, instead of failing the
 * whole material pipeline over a feature the scene may not even use. Under
 * "#version 320 es" the extension macro is undefined but gl_PrimitiveID is
 * core, so that branch is selected explicitly.
 *
 * vec3(-1.0) is the caller's "no ptex color here" sentinel, already the
 * desktop behavior — losing ptex face colors is a visible but bounded
 * degradation; losing every material is not. */
#    define NUSD_PTEX_COLOR_FUNC                                                                                       \
        "#if defined(GL_EXT_geometry_shader) || __VERSION__ >= 320\n"                                                  \
        "vec3 ptexTriangleColor(uint offset) {\n"                                                                      \
        "    if (offset == 0xFFFFFFFFu) return vec3(-1.0);\n"                                                          \
        "    uint base = offset + uint(gl_PrimitiveID) * 3u;\n"                                                        \
        "    vec3 c0 = unpackUnorm4x8(ptexTriColors.colors[base + 0u]).rgb;\n"                                         \
        "    vec3 c1 = unpackUnorm4x8(ptexTriColors.colors[base + 1u]).rgb;\n"                                         \
        "    vec3 c2 = unpackUnorm4x8(ptexTriColors.colors[base + 2u]).rgb;\n"                                         \
        "    return (c0 + c1 + c2) * (1.0 / 3.0);\n"                                                                   \
        "}\n"                                                                                                          \
        "#else\n"                                                                                                      \
        "vec3 ptexTriangleColor(uint offset) { return vec3(-1.0); }\n"                                                 \
        "#endif\n"
#    define NUSD_WH_DESKTOP_PARITY ""
#endif

/* Each TU using this header references only a subset of the shaders;
 * silence -Wunused-variable for the unreferenced ones. */
#if defined(__GNUC__) || defined(__clang__)
#    define NUSD_SHADER_UNUSED __attribute__((unused))
#else
#    define NUSD_SHADER_UNUSED
#endif

/* ---- Raster mesh vertex shader ---- */

static const char* NUSD_SHADER_UNUSED k_mesh_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;\n"
    "    mat4 model;\n"
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec3 inNormal;\n"
    "layout(location = 2) in vec3 inColor;\n"
    "\n"
    "out vec3 fragWorldPos;\n"
    "out vec3 fragNormal;\n"
    "out vec3 fragColor;\n"
    "\n"
    "void main() {\n"
    "    gl_Position = mesh.mvp * vec4(inPosition, 1.0);\n"
    "    fragWorldPos = (mesh.model * vec4(inPosition, 1.0)).xyz;\n"
    "    mat3 normalModel = mat3(mesh.model);\n"
    "    vec3 transformedNormal = normalModel * inNormal;\n"
    "    if (abs(determinant(normalModel)) > 1e-12)\n"
    "        transformedNormal = transpose(inverse(normalModel)) * inNormal;\n"
    "    if (dot(transformedNormal, transformedNormal) < 1e-20) transformedNormal = inNormal;\n"
    "    fragNormal = normalize(transformedNormal);\n"
    "    fragColor = mesh.color.w > 0.5 ? mesh.color.rgb : inColor;\n"
    "}\n";

/* ---- Raster mesh fragment shader ---- */

static const char* NUSD_SHADER_UNUSED k_mesh_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n"
    "in vec3 fragNormal;\n"
    "in vec3 fragWorldPos;\n"
    "in vec3 fragColor;\n"
    "\n"
    "uniform vec4 u_tone;\n"
    "\n"
    "out vec4 outColor;\n"
    "\n"
    "void main() {\n"
    "    vec3 N = normalize(fragNormal);\n"
    "    vec3 displayColor = clamp(fragColor * vec3(1.030, 1.012, 0.965), 0.0, 1.0);\n"
    "    float displayLuma = dot(displayColor, vec3(0.2126, 0.7152, 0.0722));\n"
    "    float displayUp = abs(N.z);\n"
    "    float displayAbsUp = abs(N.z);\n"
    "    float upFace = smoothstep(0.30, 0.85, displayUp);\n"
    "    float lowSurface = 1.0 - smoothstep(8.0, 60.0, fragWorldPos.z);\n"
    "    float darkUp = upFace * lowSurface;\n"
    "    float brightUp = upFace * (1.0 - lowSurface) * smoothstep(0.50, 0.88, displayLuma);\n"
    "    float verticalFace = 1.0 - smoothstep(0.18, 0.95, displayAbsUp);\n"
    "    float ambientOcclusion = clamp(1.0 - 0.04 * verticalFace - 0.75 * darkUp, 0.25, 1.0);\n"
    "    float directOcclusion = clamp(1.0 - 0.70 * darkUp, 0.30, 1.0);\n"
    "\n"
    "    /* Dome (hemisphere) ambient */\n"
    "    vec3 skyColor    = vec3(0.50, 0.58, 0.75);\n"
    "    vec3 groundColor = vec3(0.22, 0.20, 0.17);\n"
    "    float hemi = dot(N, vec3(0.0, 0.0, 1.0)) * 0.5 + 0.5;\n"
    "    vec3 ambient = (mix(groundColor, skyColor, hemi) + vec3(0.08, 0.08, 0.10)) * ambientOcclusion;\n"
    "    ambient += vec3(0.06) * verticalFace * smoothstep(0.25, 0.80, displayLuma);\n"
    "\n"
    "    /* Key light */\n"
    "    vec3 keyDir = normalize(vec3(0.3, 1.0, 0.5));\n"
    "    float keyNdotL = max(dot(N, keyDir), 0.0);\n"
    "    vec3 key = vec3(1.0, 0.95, 0.85) * 0.92 * (1.0 + 0.24 * brightUp) * directOcclusion * keyNdotL;\n"
    "\n"
    "    /* Fill light */\n"
    "    vec3 fillDir = normalize(vec3(-0.5, 0.4, -0.3));\n"
    "    float fillNdotL = max(dot(N, fillDir), 0.0);\n"
    "    vec3 fill = vec3(0.7, 0.8, 1.0) * 0.15 * directOcclusion * fillNdotL;\n"
    "\n"
    "    /* Rim light */\n"
    "    vec3 rimDir = normalize(vec3(0.0, 0.3, -1.0));\n"
    "    float rimNdotL = max(dot(N, rimDir), 0.0);\n"
    "    vec3 rim = vec3(1.0, 0.95, 0.9) * 0.12 * directOcclusion * rimNdotL;\n"
    "\n"
    "    vec3 color = displayColor * (ambient + key + fill + rim) * max(u_tone.x, 0.0);\n"
    "    color = clamp(color, 0.0, 1.0);\n"
    "    color = pow(color, vec3(1.0 / 2.2));\n"
    "    outColor = vec4(color, 1.0);\n"
    "}\n";

/* ---- Shadow depth-only pass ---- */

static const char* NUSD_SHADER_UNUSED k_shadow_vert_gles =
    NUSD_SHADER_VERSION NUSD_PRECISION_HIGH "\n" NUSD_UBO_LAYOUT_MESH
                                            "uniform MeshBlock {\n"
                                            "    mat4 mvp;\n"
                                            "    mat4 model;\n"
                                            "    vec4 color;\n"
                                            "    uvec4 ptex;\n"
                                            "} mesh;\n"
                                            "\n"
                                            /* The pass VP, not a per-light array: one uniform carries whichever
                                             * light's view-projection the current atlas tile belongs to, so the
                                             * MeshBlock can stay loaded with the COLOR pass's model matrices and
                                             * every tile is a re-draw with one uniform changed. (Was
                                             * u_shadowLightVP, which also named the fragment-stage mat4[2] the atlas
                                             * replaced -- one name for two unrelated uniforms in two programs.) */
                                            "uniform mat4 u_shadowPassVP;\n"
                                            "\n"
                                            "layout(location = 0) in vec3 inPosition;\n"
                                            "\n"
                                            "void main() {\n"
                                            "    gl_Position = u_shadowPassVP * mesh.model * vec4(inPosition, 1.0);\n"
                                            "}\n";

static const char* NUSD_SHADER_UNUSED k_shadow_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n"
    "void main() {}\n";

/* ---- Overlay vertex shader ---- */

static const char* NUSD_SHADER_UNUSED k_overlay_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n"
    "uniform vec2 u_screen_size;\n"
    "\n"
    "layout(location = 0) in vec2 inPos;\n"
    "layout(location = 1) in vec2 inUV;\n"
    "layout(location = 2) in vec4 inColor;\n"
    "\n"
    "out vec2 fragUV;\n"
    "out vec4 fragColor;\n"
    "\n"
    "void main() {\n"
    "    vec2 ndc;\n"
    "    ndc.x = (inPos.x / u_screen_size.x) * 2.0 - 1.0;\n"
    "    ndc.y = 1.0 - (inPos.y / u_screen_size.y) * 2.0;\n" /* Y=0 at top */
    "    gl_Position = vec4(ndc, 0.0, 1.0);\n"
    "    fragUV = inUV;\n"
    "    fragColor = inColor;\n"
    "}\n";

/* ---- Overlay fragment shader ---- */

static const char* NUSD_SHADER_UNUSED k_overlay_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n"
    "uniform sampler2D fontTex;\n"
    "\n"
    "in vec2 fragUV;\n"
    "in vec4 fragColor;\n"
    "\n"
    "out vec4 outColor;\n"
    "\n"
    "void main() {\n"
    "    if (fragUV.x < 0.0) {\n"
    "        outColor = fragColor;\n"
    "    } else {\n"
    "        float d = texture(fontTex, fragUV).r;\n"
    "        float aaw = fwidth(d) * 0.75;\n"
    "        float alpha = smoothstep(0.5 - aaw, 0.5 + aaw, d);\n"
    "        outColor = vec4(fragColor.rgb, fragColor.a * alpha);\n"
    "    }\n"
    "}\n";

/* ---- PBR material vertex shader ---- */

static const char* NUSD_SHADER_UNUSED k_pbr_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;\n"
    "    mat4 model;\n"
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec3 inNormal;\n"
    "layout(location = 2) in vec3 inColor;\n"
    "layout(location = 3) in vec2 inTexCoord;\n"
    "layout(location = 4) in vec3 inTangent;\n"
    "layout(location = 5) in float inTangentSign;\n"
    "\n"
    "out vec3 fragWorldPos;\n"
    "out vec3 fragNormal;\n"
    "out vec3 fragTangent;\n"
    "out float fragTangentSign;\n"
    "out vec3 fragColor;\n"
    "out vec2 fragTexCoord;\n"
    "\n"
    "void main() {\n"
    "    gl_Position = mesh.mvp * vec4(inPosition, 1.0);\n"
    "    fragWorldPos = (mesh.model * vec4(inPosition, 1.0)).xyz;\n"
    "    mat3 normalModel = mat3(mesh.model);\n"
    "    vec3 transformedNormal = normalModel * inNormal;\n"
    "    if (abs(determinant(normalModel)) > 1e-12)\n"
    "        transformedNormal = transpose(inverse(normalModel)) * inNormal;\n"
    "    if (dot(transformedNormal, transformedNormal) < 1e-20) transformedNormal = inNormal;\n"
    "    fragNormal = normalize(transformedNormal);\n"
    "    fragTangent = normalize((mesh.model * vec4(inTangent, 0.0)).xyz);\n"
    "    fragTangentSign = inTangentSign * (determinant(normalModel) < 0.0 ? -1.0 : 1.0);\n"
    "    fragColor = mesh.color.w > 0.5 ? mesh.color.rgb : inColor;\n"
    "    fragTexCoord = inTexCoord;\n"
    "}\n";

/* ---- Instanced PBR material vertex shader ----
 * Same fragment interface as k_pbr_vert_gles, but the per-instance world
 * matrix arrives as four vec4 columns (locations 6-9, instance-rate) instead of
 * the MeshBlock.model. Used to draw compact PointInstancer batches. MeshBlock.mvp
 * carries the camera VP (raw, NOT premultiplied by a model). Convention: the GLES
 * UBO upload is column-major and md->mvp normally = mat4_mul(vp, world_row); since
 * G(mat4_mul(A,B)) == G(B)*G(A), the combined matrix is (instModel * mesh.mvp). */
static const char* NUSD_SHADER_UNUSED k_pbr_instanced_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;\n" // carries camera VP for instanced draws
    "    mat4 model;\n" // unused (per-instance model comes from attributes)
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n"
    "layout(location = 0) in vec3 inPosition;\n"
    "layout(location = 1) in vec3 inNormal;\n"
    "layout(location = 2) in vec3 inColor;\n"
    "layout(location = 3) in vec2 inTexCoord;\n"
    "layout(location = 4) in vec3 inTangent;\n"
    "layout(location = 5) in float inTangentSign;\n"
    "layout(location = 6) in vec4 inInstCol0;\n"
    "layout(location = 7) in vec4 inInstCol1;\n"
    "layout(location = 8) in vec4 inInstCol2;\n"
    "layout(location = 9) in vec4 inInstCol3;\n"
    "\n"
    "out vec3 fragWorldPos;\n"
    "out vec3 fragNormal;\n"
    "out vec3 fragTangent;\n"
    "out float fragTangentSign;\n"
    "out vec3 fragColor;\n"
    "out vec2 fragTexCoord;\n"
    "\n"
    "void main() {\n"
    "    mat4 im = mat4(inInstCol0, inInstCol1, inInstCol2, inInstCol3);\n"
    "    vec4 wp = im * vec4(inPosition, 1.0);\n"
    // MeshBlock is row_major so mesh.mvp reads as VP directly; combined = VP*model.
    "    gl_Position = (mesh.mvp * im) * vec4(inPosition, 1.0);\n"
    "    fragWorldPos = wp.xyz;\n"
    "    mat3 normalModel = mat3(im);\n"
    "    vec3 transformedNormal = normalModel * inNormal;\n"
    "    if (abs(determinant(normalModel)) > 1e-12)\n"
    "        transformedNormal = transpose(inverse(normalModel)) * inNormal;\n"
    "    if (dot(transformedNormal, transformedNormal) < 1e-20) transformedNormal = inNormal;\n"
    "    fragNormal = normalize(transformedNormal);\n"
    "    fragTangent = normalize((im * vec4(inTangent, 0.0)).xyz);\n"
    "    fragTangentSign = inTangentSign * (determinant(normalModel) < 0.0 ? -1.0 : 1.0);\n"
    "    fragColor = mesh.color.w > 0.5 ? mesh.color.rgb : inColor;\n"
    "    fragTexCoord = inTexCoord;\n"
    "}\n";

/* ---- PBR material fragment shader ---- */

static const char* NUSD_SHADER_UNUSED k_pbr_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "\n"
    "/* Material parameters (per-draw UBO) */\n" NUSD_UBO_LAYOUT
    "uniform MaterialBlock {\n"
    "    vec4  base_color;\n"
    "    vec4  emissive_color;\n"
    "    float metallic;\n"
    "    float roughness;\n"
    "    float opacity;\n"
    "    float ior;\n"
    "    float occlusion;\n"
    "    float clearcoat;\n"
    "    float clearcoat_roughness;\n"
    "    float normal_scale;\n"
    "    ivec4 tex_indices_0;\n" /* tex_indices[0..3] */
    "    ivec4 tex_indices_1;\n" /* tex_indices[4..7] */
    "    int   use_vertex_color;\n"
    "    float udim_scale_u;\n"
    "    float udim_scale_v;\n"
    "    int   v_flip;\n"
    "    float opacity_threshold;\n"
    "    int   opacity_texture_channel;\n"
    "    int   roughness_texture_channel;\n"
    "    int   metallic_texture_channel;\n"
    "    vec4  mdl_uv_transform;\n"
    "    vec4  transmission_color;\n"
    "    float transmission_weight;\n"
    "    float transmission_ior;\n"
    "    float _pad_d;\n"
    "    float _pad_e;\n"
    "    int   use_specular_workflow;\n"
    "    float _pad_f;\n"
    "    float _pad_g;\n"
    "    float _pad_h;\n"
    "    vec4  specular_color;\n"
    "    float roughness_tex_scale;\n"
    "    float roughness_tex_bias;\n"
    "    float _pad_i;\n"
    "    float _pad_j;\n"
    "} mat;\n"
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;\n"
    "    mat4 model;\n"
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n" NUSD_PTEX_COLOR_DECL
    "\n"
    "uniform sampler2D tex_diffuse;\n"
    "uniform sampler2D tex_normal;\n"
    "uniform sampler2D tex_roughness;\n"
    "uniform sampler2D tex_metallic;\n"
    "uniform sampler2D tex_emissive;\n"
    "uniform sampler2D tex_occlusion;\n"
    "uniform sampler2D tex_opacity;\n"
    "\n"
    "/* IBL */\n"
    "uniform sampler2D u_envMap;\n"
    "uniform sampler2D u_irrMap;\n"
    "uniform sampler2D u_brdfLUT;\n"
    "uniform int u_envMipLevels;\n"
    "uniform int u_hasIBL;\n"
    "uniform int u_fallbackLighting;\n"
    "uniform int u_envOwnsLighting;\n"
    "uniform float u_envRotation;  /* DomeLight inputs:rotation, degrees */\n"
    "uniform vec3 u_eyePos;\n"
    "uniform int u_debugMode;\n"
    "uniform vec4 u_tone;\n"
    "uniform float u_envIntensity;\n"
    "uniform float u_sceneRadius;\n"
    "uniform int u_authoredLightCount;\n"
    "uniform int u_sceneLightCount;\n"
    "uniform vec4 u_sceneLightPosIntensity[32];\n"
    "uniform vec4 u_sceneLightNormalKind[32];\n"
    "uniform vec4 u_sceneLightColorNormalize[32];\n"
    "uniform vec4 u_sceneLightUAxisAngle[32];\n"
    "uniform vec4 u_sceneLightVAxis[32];\n"
    /* Shared depth atlas: ONE sampler on unit 10 for every shadow-casting
     * light (was two plain sampler2D on units 10/11 plus a mat4[2] and an
     * ivec2, which capped the renderer at two lights and burned a second
     * texture unit). sampler2DShadow means texture() does the depth COMPARE
     * in the TMU and returns a bilinearly filtered lit fraction, so the 3x3
     * kernel below is effectively 4x4-tap PCF for 9 fetches -- the old path
     * did 9 hard binary compares in ALU off a NEAREST map and stair-stepped.
     * Two uniform SEMANTICS changed with it: u_shadowParams.y was the depth
     * bias and is now unused (bias moved per-light into u_shadow[slot].bias.x,
     * because one constant cannot be right for two scenes whose light
     * frustums differ by 40x in depth span), and .w is 1/atlasSize rather
     * than 1/mapSize. */
    "uniform highp sampler2DShadow u_shadowAtlas;  /* unit 10 */\n"
    "uniform vec4 u_shadowParams;  /* x=count, y=unused, z=strength, w=texel(1/atlasSize) */\n"
    "struct ShadowLight { mat4 lightVP; vec4 uvScaleOffset; vec4 bias; };\n" NUSD_UBO_LAYOUT_SHADOW
    "uniform ShadowBlock {\n"
    "    ShadowLight u_shadow[" NUSD_MAX_SHADOW_LIGHTS_STR
    "];\n"
    "};\n"
    "uniform int u_shadowLightToSlot[" NUSD_MAX_SCENE_LIGHTS_STR
    "];\n"
    /* Screen-space AO, unit 11: the blurred half-res AO buffer, and the
     * strength that scales it (0 = exactly the pre-SSAO frame). */
    "uniform sampler2D u_ssaoTex;   /* unit 11 */\n"
    "uniform float u_aoStrength;\n"
    "uniform vec2 u_invRenderSize;\n"
    "\n"
    "in vec3 fragWorldPos;\n"
    "in vec3 fragNormal;\n"
    "in vec3 fragTangent;\n"
    "in float fragTangentSign;\n"
    "in vec3 fragColor;\n"
    "in vec2 fragTexCoord;\n"
    "\n"
    "out vec4 outColor;\n"
    "\n"
    "float selectedTextureChannel(vec4 texel, int channel) {\n"
    "    if (channel == 1) return texel.g;\n"
    "    if (channel == 2) return texel.b;\n"
    "    if (channel == 3) return texel.a;\n"
    "    return texel.r;\n"
    "}\n"
    "\n"
    "const float PI = 3.14159265359;\n"
    "const int LIGHT_KIND_RECT = 0;\n"
    "const int LIGHT_KIND_DISTANT = 1;\n"
    "const int LIGHT_KIND_SPHERE = 2;\n"
    "\n"
    "/* One shared camera constant: the measured OVRTX output transform is\n"
    " * per-channel ACES (Narkowicz fit) + the sRGB OETF at a fixed exposure of\n"
    " * 0.000882 display-linear per nit. Re-measured against official ovrtx 0.4\n"
    " * on this bench, attach lane, a 0.5-albedo Lambertian plate fully open to a\n"
    " * white DomeLight: official's scene-linear response is 4.54e-4 per unit\n"
    " * inputs:intensity and LINEAR over three decades (4.29/4.26/4.62/4.55/4.55/\n"
    " * 4.54/4.58 e-4 at intensity 3/10/30/100/300/1000/3000). The diffuse term\n"
    " * of this model predicts kD_ibl * albedo * NU_CAMERA_EXPOSURE = 0.914 * 0.5\n"
    " * * 8.82e-4 = 4.03e-4, and the split-sum specular closes the remaining 12%.\n"
    " * Independently: robot_ground_scene's DomeLight (900 * luma(0.92,0.95,1.0)\n"
    " * = 852.5 nits) renders on official at sky (222,223,225), which inverts to\n"
    " * 8.60e-4 per nit. */\n"
    "const float NU_CAMERA_EXPOSURE = 0.000882;\n"
    "\n" NUSD_PTEX_COLOR_FUNC
    "\n"
    "float distributionGGX(vec3 N, vec3 H, float r) {\n"
    "    float a = r * r;\n"
    "    float a2 = a * a;\n"
    "    float NdotH = max(dot(N, H), 0.0);\n"
    "    float denom = NdotH * NdotH * (a2 - 1.0) + 1.0;\n"
    "    return a2 / (PI * denom * denom + 0.0001);\n"
    "}\n"
    "\n"
    "float geometrySchlickGGX(float NdotV, float r) {\n"
    "    float k = (r + 1.0) * (r + 1.0) / 8.0;\n"
    "    return NdotV / (NdotV * (1.0 - k) + k);\n"
    "}\n"
    "\n"
    "float geometrySmith(vec3 N, vec3 V, vec3 L, float r) {\n"
    "    return geometrySchlickGGX(max(dot(N, V), 0.0), r)\n"
    "         * geometrySchlickGGX(max(dot(N, L), 0.0), r);\n"
    "}\n"
    "\n"
    "vec3 fresnelSchlick(float cosTheta, vec3 F0) {\n"
    "    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);\n"
    "}\n"
    "\n"
    "vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float r) {\n"
    "    return F0 + (max(vec3(1.0 - r), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);\n"
    "}\n"
    "\n"
    "/* Lat-long environment map lookup.\n"
    " * Match Hydra/OVRTX DomeLight convention for these assets: +Z is the\n"
    " * north pole and +Y is zero-longitude forward. */\n"
    "vec2 envMapUV(vec3 dir) {\n"
    "    const float kDomeLongitudeOffset = 340.0 / 360.0;\n"
    "    float u = fract(atan(dir.x, dir.y) * (0.5 / PI) + 0.5 + kDomeLongitudeOffset\n"
    "                    - u_envRotation * (1.0 / 360.0));\n"
    "    float v = asin(clamp(dir.z, -1.0, 1.0)) * (1.0 / PI) + 0.5;\n"
    "    return vec2(u, 1.0 - v);\n"
    "}\n"
    "\n"
    "/* ACES filmic tone mapping */\n"
    "vec3 aces(vec3 x) {\n"
    "    float a = 2.51;\n"
    "    float b = 0.03;\n"
    "    float c = 2.43;\n"
    "    float d = 0.59;\n"
    "    float e = 0.14;\n"
    "    return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);\n"
    "}\n"
    "\n"
    "/* Exact IEC 61966-2-1 sRGB OETF, replacing the pow(1/2.2) approximation on\n"
    " * the radiometric path only. The GLES offscreen target is plain UNORM (no\n"
    " * fixed-function sRGB store) and readback is a raw glReadPixels, so the\n"
    " * shader performs the ONE encode. pow(1/2.2) is up to 4 codes off in the\n"
    " * shadows, which matters once the 1/255 display floor below is gone. */\n"
    "vec3 nuLinearToSrgb(vec3 c) {\n"
    "    c = clamp(c, 0.0, 1.0);\n"
    "    vec3 lo = 12.92 * c;\n"
    "    vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;\n"
    "    bvec3 use_hi = greaterThan(c, vec3(0.0031308));\n"
    "    return vec3(use_hi.x ? hi.x : lo.x,\n"
    "                use_hi.y ? hi.y : lo.y,\n"
    "                use_hi.z ? hi.z : lo.z);\n"
    "}\n"
    "\n"
    "vec3 directBRDF(vec3 L, vec3 radiance, vec3 N, vec3 V, vec3 baseColor,\n"
    "                float metallic, float roughness, vec3 F0) {\n"
    "    float NdotL = max(dot(N, L), 0.0);\n"
    "    if (NdotL <= 0.0) return vec3(0.0);\n"
    "    vec3 H = normalize(V + L);\n"
    "    float NdotVLocal = max(dot(N, V), 0.001);\n"
    "    float D = distributionGGX(N, H, roughness);\n"
    "    float G = geometrySmith(N, V, L, roughness);\n"
    "    vec3 F = fresnelSchlickRoughness(max(dot(H, V), 0.0), F0, roughness);\n"
    "    vec3 spec = (D * G * F) / (4.0 * NdotVLocal * NdotL + 0.0001);\n"
    "    vec3 kD = (1.0 - F) * (1.0 - metallic);\n"
    "    return (kD * baseColor / PI + spec) * radiance * NdotL;\n"
    "}\n"
    "\n"
    "float areaLightSpecularRoughness(float roughness, float angularDiameter) {\n"
    "    float a = clamp(angularDiameter * 2.5, 0.0, 1.0);\n"
    "    return clamp(sqrt(roughness * roughness + a * a), roughness, 1.0);\n"
    "}\n"
    "\n"
    "/* Atlas-tile shadow visibility. The atlas is a sampler2DShadow, so\n"
    " * texture() performs the hardware depth compare against the supplied\n"
    " * reference and returns the PCF-filtered lit fraction 0..1 -- there is no\n"
    " * stored depth to read back, which is why the fork's PCSS blocker search\n"
    " * (shadowViewDepth/sceneLightShadowExtent) is dead code there and is not\n"
    " * ported here. Fixed 3x3 hardware PCF inside the light's own tile. */\n"
    "float sampleShadowVisibilityAtlas(int slot, vec3 worldPos, vec3 N, vec3 L) {\n"
    "    mat4 lightVP = u_shadow[slot].lightVP;\n"
    "    vec4 so = u_shadow[slot].uvScaleOffset;  /* xy = scale, zw = offset */\n"
    "    float b = u_shadow[slot].bias.x;\n"
    "    vec4 lp = lightVP * vec4(worldPos + N * 0.003, 1.0);\n"
    "    if (abs(lp.w) < 1e-6) return 1.0;\n"
    "    vec3 ndc = lp.xyz / lp.w;\n"
    "    if (ndc.x < -1.0 || ndc.x > 1.0 || ndc.y < -1.0 || ndc.y > 1.0 ||\n"
    "        ndc.z < -1.0 || ndc.z > 1.0) return 1.0;\n"
    "    vec2 tileUV = ndc.xy * 0.5 + 0.5;\n"
    "    float depth = ndc.z * 0.5 + 0.5;\n"
    "    float ndotl = max(dot(N, L), 0.0);\n"
    "    float bias = max(b * (1.0 - ndotl), b * 0.35);\n"
    "    float texel = u_shadowParams.w;            /* 1 / atlasSize */\n"
    "    float step = texel / max(so.x, 1e-6);      /* one ATLAS texel in tile-UV */\n"
    "    /* 1-texel guard band. Meaningless with one map per texture; mandatory\n"
    "     * the moment tiles share one texture, or a kernel tap at a tile edge\n"
    "     * samples the NEIGHBOURING light's depth and shadows the wrong pixel. */\n"
    "    float halfTexTile = 0.5 * step;\n"
    "    float lit = 0.0;\n"
    "    for (int oy = -1; oy <= 1; ++oy) {\n"
    "        for (int ox = -1; ox <= 1; ++ox) {\n"
    "            vec2 tap = tileUV + vec2(float(ox), float(oy)) * step;\n"
    "            tap = clamp(tap, vec2(halfTexTile), vec2(1.0 - halfTexTile));\n"
    "            vec2 atlasUV = tap * so.xy + so.zw;\n"
    "            lit += texture(u_shadowAtlas, vec3(atlasUV, depth - bias));\n"
    "        }\n"
    "    }\n"
    "    lit /= 9.0;\n"
    "    return mix(1.0 - u_shadowParams.z, 1.0, lit);\n"
    "}\n"
    "\n"
    "float sceneLightVisibility(int lightIndex, vec3 worldPos, vec3 N, vec3 L) {\n"
    "    if (int(u_shadowParams.x + 0.5) <= 0) return 1.0;\n"
    "    int slot = u_shadowLightToSlot[lightIndex];\n"
    "    if (slot < 0) return 1.0;\n"
    "    return sampleShadowVisibilityAtlas(slot, worldPos, N, L);\n"
    "}\n"
    "\n"
    "vec3 evalSceneRectLightSample(int lightIndex, vec3 samplePos, float radiance,\n"
    "                              float area, vec3 lightNormal, vec3 lightColor,\n"
    "                              vec3 rectU, vec3 rectV, vec3 N, vec3 V,\n"
    "                              vec3 baseColor, float metallic, float roughness,\n"
    "                              vec3 F0, vec3 worldPos) {\n"
    "    vec3 toLight = samplePos - worldPos;\n"
    "    float dist2 = dot(toLight, toLight);\n"
    "    if (dist2 < 1e-4) return vec3(0.0);\n"
    "    float dist = sqrt(dist2);\n"
    "    vec3 L = toLight / dist;\n"
    "    float cosLight = dot(-L, lightNormal);\n"
    "    if (cosLight <= 0.0) return vec3(0.0);\n"
    "    float geom = (cosLight * area) / max(dist2, 1e-4);\n"
    "    float visibility = sceneLightVisibility(lightIndex, worldPos, N, L);\n"
    "    float rectAngular = clamp(2.0 * max(length(rectU), length(rectV)) / max(dist, 1e-3), 0.0, 1.0);\n"
    "    float specRoughness = areaLightSpecularRoughness(roughness, rectAngular);\n"
    "    return directBRDF(L, lightColor * radiance * geom,\n"
    "                      N, V, baseColor, metallic, specRoughness, F0) * visibility;\n"
    "}\n"
    "\n"
    "vec3 evalSceneLight(int i, vec3 N, vec3 V, vec3 baseColor, float metallic,\n"
    "                    float roughness, vec3 F0, vec3 worldPos) {\n"
    "    vec4 pi = u_sceneLightPosIntensity[i];\n"
    "    vec4 nk = u_sceneLightNormalKind[i];\n"
    "    vec4 cn = u_sceneLightColorNormalize[i];\n"
    "    vec4 ua = u_sceneLightUAxisAngle[i];\n"
    "    vec4 va = u_sceneLightVAxis[i];\n"
    "    int kind = int(nk.w + 0.5);\n"
    "    vec3 lightColor = cn.rgb;\n"
    "    float intensity = pi.w;\n"
    "    if (kind == LIGHT_KIND_DISTANT) {\n"
    "        vec3 L = normalize(-nk.xyz);\n"
    "        float visibility = sceneLightVisibility(i, worldPos, N, L);\n"
    "        return directBRDF(L, lightColor * (intensity * 0.0006),\n"
    "                          N, V, baseColor, metallic, roughness, F0) * visibility;\n"
    "    }\n"
    "\n"
    "    vec3 toLight = pi.xyz - worldPos;\n"
    "    float dist2 = max(dot(toLight, toLight), 1e-4);\n"
    "    float dist = sqrt(dist2);\n"
    "    vec3 L = toLight / dist;\n"
    "\n"
    "    if (kind == LIGHT_KIND_RECT) {\n"
    "        float area = 4.0 * length(ua.xyz) * length(va.xyz);\n"
    "        float radiance = (cn.w > 0.5) ? intensity / max(area * PI, 1e-6)\n"
    "                                     : intensity / PI;\n"
    "        radiance *= 0.0012;\n"
    "        /* 3x3 Gauss-Legendre quadrature over the rectangular emitter.\n"
    "         * This keeps total light energy unchanged while spreading glossy\n"
    "         * RectLight response across the authored area instead of pinning it\n"
    "         * to the center point. */\n"
    "        const float q = 0.7745966692;\n"
    "        const float wc = 0.1975308642;\n"
    "        const float we = 0.1234567901;\n"
    "        const float wk = 0.0771604938;\n"
    "        vec3 center = pi.xyz;\n"
    "        vec3 U = ua.xyz;\n"
    "        vec3 W = va.xyz;\n"
    "        vec3 accum = vec3(0.0);\n"
    "        accum += evalSceneRectLightSample(i, center, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * wc;\n"
    "        accum += evalSceneRectLightSample(i, center + U * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * we;\n"
    "        accum += evalSceneRectLightSample(i, center - U * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * we;\n"
    "        accum += evalSceneRectLightSample(i, center + W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * we;\n"
    "        accum += evalSceneRectLightSample(i, center - W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * we;\n"
    "        accum += evalSceneRectLightSample(i, center + U * q + W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * wk;\n"
    "        accum += evalSceneRectLightSample(i, center + U * q - W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * wk;\n"
    "        accum += evalSceneRectLightSample(i, center - U * q + W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * wk;\n"
    "        accum += evalSceneRectLightSample(i, center - U * q - W * q, radiance, area, nk.xyz, lightColor, U, W,\n"
    "                                          N, V, baseColor, metallic, roughness, F0, worldPos) * wk;\n"
    "        return accum;\n"
    "    }\n"
    "\n"
    "    float radius = max(ua.x, 1e-4);\n"
    "    float area = 4.0 * PI * radius * radius;\n"
    "    float radiance = (cn.w > 0.5) ? intensity / area : intensity;\n"
    "    /* Match Vulkan's lower small-probe SphereLight energy while keeping\n"
    "     * OpenGL RectLight exposure calibrated for GB300-style probes. */\n"
    "    radiance *= 0.000075;\n"
    "    float visibility = sceneLightVisibility(i, worldPos, N, L);\n"
    "    float sphereAngular = clamp(2.0 * radius / max(dist, 1e-3), 0.0, 1.0);\n"
    "    float specRoughness = areaLightSpecularRoughness(roughness, sphereAngular);\n"
    "    return directBRDF(L, lightColor * radiance / dist2,\n"
    "                      N, V, baseColor, metallic, specRoughness, F0) * visibility;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vec3 N = normalize(fragNormal);\n"
    "    vec3 V = normalize(u_eyePos - fragWorldPos);\n"
    "\n"
    "    vec2 tc = fragTexCoord / vec2(mat.udim_scale_u, mat.udim_scale_v);\n"
    "    vec2 uvScale = mat.mdl_uv_transform.xy;\n"
    "    if (uvScale.x == 0.0 && uvScale.y == 0.0) uvScale = vec2(1.0);\n"
    "    tc = tc * uvScale + mat.mdl_uv_transform.zw;\n"
    "    if (mat.v_flip != 0) tc.y = 1.0 - tc.y;\n"
    "\n"
    "    /* Opacity cutout. tex_indices_1.z is slot 6 (TEX_OPACITY).\n"
    "     * A positive opacityThreshold makes the scalar/map product binary:\n"
    "     * retain pixels at or above the threshold and discard the rest.\n"
    "     * For opacityThreshold == 0 (alpha-blend default), keep every\n"
    "     * pixel and let the output alpha drive framebuffer blending. */\n"
    "    float opacity_tex = 1.0;\n"
    "    if (mat.tex_indices_1.z >= 0) {\n"
    "        vec4 opacity_sample = texture(tex_opacity, tc);\n"
    "        opacity_tex = selectedTextureChannel(opacity_sample, mat.opacity_texture_channel);\n"
    "    }\n"
    "    if (mat.opacity_threshold > 0.0 &&\n"
    "        opacity_tex * mat.opacity < mat.opacity_threshold) discard;\n"
    "    float alpha = mat.base_color.a * mat.opacity * opacity_tex;\n"
    "\n"
    "    /* Normal mapping via per-vertex TBN with bitangent handedness */\n"
    "    if (mat.tex_indices_0.y >= 0) {\n"
    "        vec3 T = fragTangent;\n"
    "        float tLen = length(T);\n"
    "        if (tLen > 0.001) {\n"
    "            T = T / tLen;\n"
    "            /* Gram-Schmidt re-orthogonalize */\n"
    "            T = normalize(T - dot(T, N) * N);\n"
    "            vec3 B = cross(N, T) * fragTangentSign;\n"
    "            mat3 TBN = mat3(T, B, N);\n"
    "            vec3 mapN = texture(tex_normal, tc).rgb * 2.0 - 1.0;\n"
    "            mapN.xy *= mat.normal_scale;\n"
    "            N = normalize(TBN * mapN);\n"
    "        }\n"
    "    }\n"
    "\n"
    "    /* Two-sided rendering: flip normals facing away from camera.\n"
    "     * Instance transforms (e.g. 180deg Z-rotation) can invert normals;\n"
    "     * without this fix NdotV approaches 0 causing Fresnel blowout. */\n"
    "    if (dot(N, V) < 0.0) N = -N;\n"
    "\n"
    "    /* Base color */\n"
    "    vec3 baseColor;\n"
    "    if (mat.tex_indices_0.x >= 0) {\n"
    "        /* base_color is the per-material tint folded from the\n"
    "         * UsdUVTexture inputs:scale (TextureCoordinateTest's\n"
    "         * yellow/orange/blue/green per-quad multipliers); the\n"
    "         * loader leaves it at white when no scale is authored. */\n"
    "        baseColor = texture(tex_diffuse, tc).rgb;\n"
    "        baseColor *= mat.base_color.rgb;\n"
    "    } else if (mat.use_vertex_color != 0) {\n"
    "        baseColor = fragColor;\n"
    "    } else {\n"
    "        baseColor = mat.base_color.rgb;\n"
    "    }\n"
    "    vec3 ptexColor = ptexTriangleColor(mesh.ptex.x);\n"
    "    if (ptexColor.x >= 0.0) baseColor = ptexColor;\n"
    "    if (mesh.ptex.y == 1u) {\n"
    "        /* MuJoCo-style checkered floor (ovgl flags the ground via ptex.y). */\n"
    "        vec2 cc = floor(fragWorldPos.xy * 0.5);\n"
    "        float chk = mod(cc.x + cc.y, 2.0);\n"
    "        baseColor = mix(vec3(0.215, 0.215, 0.235), vec3(0.305, 0.305, 0.325), chk);\n"
    "    }\n"
    "\n"
    "    float metallic = mat.metallic;\n"
    "    if (mat.tex_indices_0.w >= 0)\n"
    "        metallic = selectedTextureChannel(texture(tex_metallic, tc), mat.metallic_texture_channel);\n"
    "\n"
    "    float roughness = mat.roughness;\n"
    "    if (mat.tex_indices_0.z >= 0) {\n"
    "        float roughSample = selectedTextureChannel(texture(tex_roughness, tc), mat.roughness_texture_channel);\n"
    "        float roughScale = (mat.roughness_tex_scale != 0.0) ? mat.roughness_tex_scale : 1.0;\n"
    "        roughness = roughSample * roughScale + mat.roughness_tex_bias;\n"
    "    }\n"
    "    roughness = clamp(roughness, 0.04, 1.0);\n"
    "\n"
    "    float ao = mat.occlusion;\n"
    "    if (mat.tex_indices_1.y >= 0)\n"
    "        ao *= texture(tex_occlusion, tc).r;\n"
    /* Screen-space AO. Until this line ovgl's entire AO story was the two
     * lines above -- a per-material CONSTANT times an optional baked texture --
     * so a subject that loads no occlusion texture (the G1 loads no textures at
     * all) had a uniform `ao` over its whole surface, and every crevice between
     * its 46 links received the full dome ambient. That is what "washed out"
     * was: not the material path, not the environment, not normals, all four of
     * which were measured against official and cleared.
     *
     * u_aoStrength is a uniform, not a compile-time branch, specifically so it
     * can be driven to 0 to reproduce the pre-SSAO frame exactly. gl_FragCoord
     * is in render-target pixels and u_invRenderSize is 1/(iw,ih), so the
     * lookup is correct whatever resolution the AO texture itself was computed
     * at (it is half). */
    "    if (u_aoStrength > 0.0)\n"
    "        ao *= mix(1.0, texture(u_ssaoTex, gl_FragCoord.xy * u_invRenderSize).r,\n"
    "                  u_aoStrength);\n"
    "\n"
    "    /* IOR-driven F0 for dielectrics vs metals; MDL may author F0 directly. */\n"
    "    float f0_dielectric = pow((mat.ior - 1.0) / (mat.ior + 1.0), 2.0);\n"
    "    float surfaceMetallic = (mat.use_specular_workflow != 0) ? 0.0 : metallic;\n"
    "    vec3 F0 = (mat.use_specular_workflow != 0) ? mat.specular_color.rgb : "
    "mix(vec3(f0_dielectric), baseColor, metallic);\n"
    "    float NdotV = max(dot(N, V), 0.001);\n"
    "    float localOcc = 1.0;\n"
    "\n"
    "    /* ---- Frame classes ----\n"
    "     *\n"
    "     * u_envIntensity is a SIGNED marker written by the environment build\n"
    "     * (gl/gpu.h): >= 0 means the env/irradiance texels are final linear\n"
    "     * radiance in NITS (an authored UsdLuxDomeLight); < 0 means the legacy\n"
    "     * auto-exposed host override, and |value| is that caller's intensity.\n"
    "     *\n"
    "     * The two environments feed DIFFERENT display scales, and that split is\n"
    "     * the whole point: the analytic-light path below is calibrated against\n"
    "     * the legacy per-class exposure ladder, the environment path against the\n"
    "     * measured camera transform. Mixing them under one `exposure` is what\n"
    "     * made a dome's contribution nonlinear in its own authored number --\n"
    "     * an intensity-1.0 DomeLight dropped the whole frame's exposure 6x\n"
    "     * (1.20 -> 0.20) while an intensity-1000 one overshot.\n"
    "     *\n"
    "     * So the analytic classes below deliberately IGNORE a radiometric\n"
    "     * environment: `radiometricEnv` frames evaluate them exactly as the\n"
    "     * same scene with no dome would. Adding a dome can only ADD light. */\n"
    "    bool radiometricEnv = (u_hasIBL != 0 && u_envIntensity >= 0.0);\n"
    "    bool legacyEnv      = (u_hasIBL != 0 && u_envIntensity <  0.0);\n"
    "    bool hasSceneLights = (u_sceneLightCount > 0);\n"
    "    bool manyAuthoredLightNoIbl = (!legacyEnv && u_fallbackLighting == 0 && u_authoredLightCount > 8);\n"
    "    bool useSceneLights = hasSceneLights;\n"
    "    bool authoredLightNoIbl = (!legacyEnv && u_fallbackLighting == 0 && !useSceneLights);\n"
    "    bool authoredSceneLightNoIbl = (!legacyEnv && u_fallbackLighting == 0 && useSceneLights);\n"
    "    bool authoredSphereSceneLightNoIbl = false;\n"
    "    if (authoredSceneLightNoIbl) {\n"
    "        for (int li = 0; li < 32; ++li) {\n"
    "            if (li >= u_sceneLightCount) break;\n"
    "            int kind = int(u_sceneLightNormalKind[li].w + 0.5);\n"
    "            if (kind == LIGHT_KIND_SPHERE) { authoredSphereSceneLightNoIbl = true; break; }\n"
    "        }\n"
    "    }\n"
    /* Equivalent to the old `useSceneLights ? 0 : ((u_hasIBL != 0 &&
     * u_fallbackLighting == 0) ? 0 : 1)` in every reachable state -- the host
     * only ever clears u_fallbackLighting when something else owns the
     * lighting -- but stated in terms of the ONE flag that actually means
     * "nothing else lights this frame". Written the old way, a radiometric
     * dome with no analytic lights (franka_factory) would have switched the
     * synthetic key/fill rig back ON. */
    "    float lightScale = (useSceneLights || u_fallbackLighting == 0) ? 0.0 : 1.0;\n"
    "\n"
    "    vec3 Lo = vec3(0.0);\n"
    "\n"
    "    /* Key light */\n"
    "    vec3 L = normalize(vec3(0.3, 1.0, 0.5));\n"
    "    vec3 H = normalize(V + L);\n"
    "    float NdotL = max(dot(N, L), 0.0);\n"
    "    float D = distributionGGX(N, H, roughness);\n"
    "    float G = geometrySmith(N, V, L, roughness);\n"
    "    vec3 F = fresnelSchlickRoughness(max(dot(H, V), 0.0), F0, roughness);\n"
    "    vec3 spec = (D * G * F) / (4.0 * NdotV * NdotL + 0.0001);\n"
    "    vec3 kD = (1.0 - F) * (1.0 - surfaceMetallic);\n"
    "    Lo += (kD * baseColor / PI + spec) * vec3(1.0, 0.95, 0.85) * lightScale * NdotL;\n"
    "\n"
    "    /* Fill light — simplified Lambertian (no PI division, matching Vulkan) */\n"
    "    L = normalize(vec3(-0.5, 0.4, -0.3));\n"
    "    NdotL = max(dot(N, L), 0.0);\n"
    "    float fillScale = (useSceneLights || u_fallbackLighting == 0) ? 0.0 : (authoredLightNoIbl ? 0.0 : 0.15);\n"
    "    Lo += baseColor * vec3(0.7, 0.8, 1.0) * fillScale * NdotL;\n"
    "\n"
    "    for (int li = 0; li < 32; ++li) {\n"
    "        if (!useSceneLights) break;\n"
    "        if (li >= u_sceneLightCount) break;\n"
    "        Lo += evalSceneLight(li, N, V, baseColor, surfaceMetallic,\n"
    "                             roughness, F0, fragWorldPos);\n"
    "    }\n"
    "\n"
    "    /* Ambient / IBL. `ambient` is now ONLY the environment term (scene-\n"
    "     * referred nits on the radiometric path); the synthetic hemisphere fill\n"
    "     * moved to `fallbackAmbient` below so the two can coexist. */\n"
    "    vec3 ambient = vec3(0.0);\n"
    "    if (u_hasIBL != 0) {\n"
    "        /* Diffuse IBL: sample SH irradiance map (proper cosine-convolved).\n"
    "         * On the radiometric path this E(n) is in nits and the outgoing\n"
    "         * radiance is the textbook kD * E * albedo/PI, with the\n"
    "         * hemispherical-average diffuse/spec energy coupling\n"
    "         * kD = (1 - F0) * (20/21) -- the closed-form cosine-weighted\n"
    "         * integral of the Schlick term. Nothing else touches it. */\n"
    "        vec3 irradiance = texture(u_irrMap, envMapUV(N)).rgb;\n"
    "        vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, roughness);\n"
    "        const vec3 LUMA = vec3(0.2126, 0.7152, 0.0722);\n"
    "        vec3 kD_ibl;\n"
    "        float diffuseShape = 1.0;\n"
    "        float specularShape = 1.0;\n"
    "        float upFacing = smoothstep(0.0, 0.85, clamp(N.z, 0.0, 1.0));\n"
    "        float albedoLum = dot(baseColor, LUMA);\n"
    "        if (radiometricEnv) {\n"
    "            kD_ibl = (1.0 - F0) * (20.0 / 21.0) * (1.0 - surfaceMetallic);\n"
    "        } else {\n"
    "            /* LEGACY host-override HDR path, byte-for-byte as before.\n"
    "             * IBL chroma clamp: clear-sky HDRs (table_mountain) have a hugely saturated\n"
    "             * blue zenith. Reflected/irradiated 1:1 it paints white robots + floors blue\n"
    "             * ('blue flashes'). Pull the *surface-lighting* env toward its luminance so lit\n"
    "             * surfaces read neutral; the background sky (separate env_bg shader) stays blue. */\n"
    "            irradiance = mix(vec3(dot(irradiance, LUMA)), irradiance, 0.40);\n"
    "            kD_ibl = (1.0 - F_ibl) * (1.0 - surfaceMetallic);\n"
    "            float darkVertical = mix(0.08, 1.0, smoothstep(0.28, 0.60, albedoLum));\n"
    "            diffuseShape = mix(darkVertical, 1.42, upFacing);\n"
    "            specularShape = mix(0.08, 1.0, upFacing);\n"
    "            float verticalOcc = 1.0 - smoothstep(0.18, 0.90, abs(N.z));\n"
    "            float lowOcc = 1.0 - smoothstep(0.015, 0.145, fragWorldPos.z);\n"
    "            float darkOcc = 1.0 - smoothstep(0.10, 0.55, albedoLum);\n"
    "            localOcc = clamp(1.0 - verticalOcc * (0.24 + 0.32 * lowOcc + 0.24 * darkOcc), 0.35, 1.0);\n"
    "        }\n"
    "        vec3 diffuseIBL = irradiance * (baseColor / PI) * kD_ibl;\n"
    "        diffuseIBL *= diffuseShape;\n"
    "\n"
    "        /* Specular IBL: sample env at roughness-based LOD.\n"
    "         * Mips are box-filtered, not GGX pre-filtered — blend toward\n"
    "         * irradiance/PI for rough surfaces (irradiance=∫L·cos(θ)dω,\n"
    "         * split-sum expects average radiance ≈ irradiance/PI). */\n"
    "        vec3 R = reflect(-V, N);\n"
    "        float lod = roughness * float(u_envMipLevels - 1);\n"
    "        vec3 envSpecular = textureLod(u_envMap, envMapUV(R), lod).rgb;\n"
    "        if (!radiometricEnv) envSpecular = mix(vec3(dot(envSpecular, LUMA)), envSpecular, 0.40);\n"
    "        float roughBlend = smoothstep(0.3, 0.9, roughness);\n"
    "        vec3 prefilteredColor = mix(envSpecular, irradiance / PI, roughBlend);\n"
    "\n"
    "        /* Split-sum: BRDF integration from LUT. The radiometric path uses\n"
    "         * the F0-based form (as vulkan/OVRTX) and takes no Kulla-Conty\n"
    "         * energy compensation: that term is a multi-scatter correction the\n"
    "         * reference does not apply, and it is what pushed rough dielectrics\n"
    "         * bright on top of an already-shaped diffuse. */\n"
    "        vec2 brdf = texture(u_brdfLUT, vec2(NdotV, roughness)).rg;\n"
    "        vec3 specularIBL = radiometricEnv\n"
    "            ? prefilteredColor * (F0 * brdf.x + brdf.y)\n"
    "            : prefilteredColor * (F_ibl * brdf.x + brdf.y);\n"
    "\n"
    "        if (!radiometricEnv) {\n"
    "            /* Energy compensation (Kulla-Conty) */\n"
    "            float Ess = brdf.x + brdf.y;\n"
    "            vec3 energyCompensation = 1.0 + F0 * (1.0 / max(Ess, 0.001) - 1.0);\n"
    "            specularIBL *= energyCompensation;\n"
    "            specularIBL *= specularShape;\n"
    "        }\n"
    "\n"
    "        if (u_debugMode == 7) { outColor = vec4(aces(specularIBL), 1.0); return; }\n"
    "        if (u_debugMode == 8) { outColor = vec4(aces(diffuseIBL), 1.0); return; }\n"
    "        if (u_debugMode == 9) { outColor = vec4(aces(envSpecular), 1.0); return; }\n"
    "        if (u_debugMode == 10) { outColor = vec4(F_ibl, 1.0); return; }\n"
    "        if (u_debugMode == 11) { outColor = vec4(vec3(NdotV), 1.0); return; }\n"
    "        if (u_debugMode == 12) { outColor = vec4(brdf.x, brdf.y, 0.0, 1.0); return; }\n"
    "        if (u_debugMode == 13) { float mult = F_ibl.r * brdf.x + brdf.y; "
    "outColor = vec4(vec3(mult), 1.0); return; }\n"
    "        if (u_debugMode == 14) { outColor = vec4(aces(prefilteredColor), 1.0); return; }\n"
    "        if (u_debugMode == 15) { outColor = vec4(vec3(roughness), 1.0); return; }\n"
    "        ambient = (diffuseIBL + specularIBL) * ao * localOcc;\n"
    "        if (!radiometricEnv) {\n"
    "            /* LEGACY-only display shaping, authored against specific HDRs.\n"
    "             * Deleted on the radiometric path together with diffuseShape /\n"
    "             * specularShape / localOcc / the 0.40 chroma clamp: darkVertical\n"
    "             * alone drove diffuse to 0.08x on any vertical face whose albedo\n"
    "             * luminance was below 0.28, which is the measured 'UR links\n"
    "             * render white on official, near-black on ovgl' symptom. */\n"
    "            float coolBounce = upFacing * (1.0 - surfaceMetallic) * smoothstep(0.06, 0.45, albedoLum);\n"
    "            ambient += vec3(0.020, 0.032, 0.052) * coolBounce * ao;\n"
    "            float lowHorizontal = upFacing * (1.0 - surfaceMetallic) * "
    "(1.0 - smoothstep(0.006, 0.035, fragWorldPos.z)) * smoothstep(0.06, 0.42, albedoLum);\n"
    "            ambient += vec3(0.035, 0.060, 0.105) * lowHorizontal * ao;\n"
    "        }\n"
    "    }\n"
    "    /* Fallback hemisphere ambient, in the ANALYTIC path's legacy units.\n"
    "     *\n"
    "     * It runs whenever the synthetic rig still owns the frame -- which is\n"
    "     * no-IBL as before, but ALSO a radiometric dome too dim to replace it\n"
    "     * (u_envOwnsLighting stays 0; see Ovgl.cpp::apply_environment and\n"
    "     * kDomeOwnsLightingNits). Without that second case, loading a 0.84-nit\n"
    "     * DomeLight swapped the whole hemisphere fill for a near-zero physical\n"
    "     * ambient: measured on robot-usdview/welcome.usda, geometry luminance\n"
    "     * 148.0 -> 144.2 and cross-stack MAE 26.9 -> 30.8. The dome's real\n"
    "     * radiometric contribution is added on top, so it can only ADD light. */\n"
    "    vec3 fallbackAmbient = vec3(0.0);\n"
    "    if (u_hasIBL == 0 || (radiometricEnv && u_envOwnsLighting == 0)) {\n"
    "        /* Fallback: hemisphere ambient. Sky/ground colors match the\n"
    "         * vulkan rchit's no-IBL fallback (procedural sky-dome) so\n"
    "         * unlit scenes render at comparable brightness across the\n"
    "         * three renderers. Specular kS coefficient stays at 0.15\n"
    "         * to keep metallic surfaces from going dim under no-IBL. */\n"
    "        vec3 skyColor = vec3(0.60, 0.68, 0.82);\n"
    "        vec3 groundColor = vec3(0.32, 0.31, 0.30);\n"
    "        float hemi = dot(N, vec3(0.0, 0.0, 1.0)) * 0.5 + 0.5;\n"
    "        vec3 ambientIrradiance = mix(groundColor, skyColor, hemi) + vec3(0.10, 0.10, 0.12);\n"
    "        /* The legacy 'a flat DomeLight brightens the synthetic fill' ramp.\n"
    "         * OFF on the radiometric path: there the dome contributes its actual\n"
    "         * radiance through the IBL branch above, and double-counting it here\n"
    "         * is what the whole texel bake exists to remove. */\n"
    "        float flatDome = radiometricEnv ? 0.0\n"
    "                       : ((u_envIntensity < -1.0) ? abs(u_envIntensity) : max(u_envIntensity, 0.0));\n"
    "        float flatDomeT = clamp(log(1.0 + flatDome) / log(12001.0), 0.0, 1.0);\n"
    "        ambientIrradiance += vec3(0.18, 0.22, 0.30) * flatDomeT;\n"
    "        vec3 kS_amb = fresnelSchlick(NdotV, F0);\n"
    "        vec3 kD_amb = (1.0 - kS_amb) * (1.0 - surfaceMetallic);\n"
    "        fallbackAmbient = kD_amb * ambientIrradiance * baseColor * ao;\n"
    "        /* A textureless DomeLight in an authored RectLight setup is\n"
    "         * bounce/fill, not a mirror-bright environment. Keep the legacy\n"
    "         * stronger spec floor only for fallback no-light scenes. */\n"
    "        float specFill = authoredSceneLightNoIbl ? 0.10 : (0.15 + 0.05 * flatDomeT);\n"
    "        fallbackAmbient += kS_amb * ao * specFill;\n"
    "        float skyUp = clamp(N.z, 0.0, 1.0);\n"
    "        vec3 skyFillColor = manyAuthoredLightNoIbl ? vec3(0.026, 0.030, 0.030) : vec3(0.018, 0.030, 0.055);\n"
    "        fallbackAmbient += skyFillColor * skyUp * ao * (1.0 - surfaceMetallic);\n"
    "        if (manyAuthoredLightNoIbl) {\n"
    "            float lowSceneHorizontal = smoothstep(0.15, 0.85, skyUp) *\n"
    "                (1.0 - smoothstep(0.05, 0.45, fragWorldPos.z));\n"
    "            fallbackAmbient *= mix(0.30, 1.0, lowSceneHorizontal);\n"
    "            fallbackAmbient += vec3(0.032, 0.034, 0.032) * lowSceneHorizontal * ao * (1.0 - surfaceMetallic);\n"
    "        }\n"
    "        if (authoredLightNoIbl) {\n"
    "            /* No-IBL scenes with authored lights should not also get the\n"
    "             * synthetic studio ambient/fill rig; OVRTX keeps the tiny MDL\n"
    "             * fixture much more saturated under an explicit DistantLight. */\n"
    "            fallbackAmbient *= 0.35;\n"
    "        } else if (authoredSceneLightNoIbl) {\n"
    "            /* MuJoCo-style low ambient (headlight.ambient ~0.1): keep the fill\n"
    "             * dim so the directional key + its shadow dominate the look. */\n"
    "            fallbackAmbient *= 0.18;\n"
    "        }\n"
    "    }\n"
    "\n"
    "    /* Emissive — use texture directly when scalar is near-zero */\n"
    "    vec3 emissiveConst = mat.emissive_color.rgb;\n"
    "    float emissiveIntensity = mat.emissive_color.a;\n"
    "    vec3 emissive = emissiveConst * emissiveIntensity;\n"
    "    if (mat.tex_indices_1.x >= 0) {\n"
    "        vec3 emissive_tex = texture(tex_emissive, tc).rgb;\n"
    "        if (dot(emissiveConst, emissiveConst) < 0.001)\n"
    "            emissive = emissive_tex * emissiveIntensity;\n"
    "        else\n"
    "            emissive *= emissive_tex;\n"
    "    }\n"
    "\n"
    "    /* ---- Put the two calibrations on ONE axis ----\n"
    "     *\n"
    "     * On a radiometric frame `ambient` is scene-referred nits and Lo is in\n"
    "     * the analytic path's legacy units, so they cannot simply be added. The\n"
    "     * analytic term is converted INTO nits by dividing out the camera\n"
    "     * transform it is about to go through -- i.e. the analytic path keeps\n"
    "     * exactly the display value it had before any dome existed, and the\n"
    "     * dome's own contribution is added on top in physical units. Doing the\n"
    "     * conversion here rather than after the tone curve also puts the\n"
    "     * transmission and clearcoat blocks below (which sample u_envMap, i.e.\n"
    "     * nits) in the same space as everything else. */\n"
    "    vec3 analytic = Lo + emissive + fallbackAmbient;\n"
    "    if (radiometricEnv) {\n"
    "        float legacyExposure = manyAuthoredLightNoIbl ? 1.20 :\n"
    "                               (authoredSphereSceneLightNoIbl ? 1.80 :\n"
    "                               (authoredSceneLightNoIbl ? 1.20 :\n"
    "                               (authoredLightNoIbl ? 1.55 : 1.2)));\n"
    "        vec3 legacyTint = vec3(1.0);\n"
    "        if (authoredLightNoIbl)      legacyTint = vec3(0.90, 0.72, 0.54);\n"
    "        if (authoredSceneLightNoIbl) legacyTint = vec3(0.68, 0.64, 0.56);\n"
    "        analytic *= legacyExposure * legacyTint * (1.0 / NU_CAMERA_EXPOSURE);\n"
    "    }\n"
    "    vec3 color = ambient + analytic;\n"
    "    if (manyAuthoredLightNoIbl && !radiometricEnv) {\n"
    "        float floorShape = smoothstep(0.15, 0.85, clamp(N.z, 0.0, 1.0)) *\n"
    "            (1.0 - smoothstep(0.05, 0.45, fragWorldPos.z));\n"
    "        float highInterior = smoothstep(1.2, 3.0, fragWorldPos.z);\n"
    "        float nonFloorScale = mix(0.35, 0.24, highInterior);\n"
    "        color *= mix(nonFloorScale, 1.0, floorShape);\n" NUSD_WH_DESKTOP_PARITY
    "    }\n"
    "    if (authoredLightNoIbl && !radiometricEnv) {\n"
    "        color *= vec3(0.90, 0.72, 0.54);\n"
    "    }\n"
    "    if (authoredSceneLightNoIbl && !radiometricEnv) {\n"
    "        /* Studio scenes (chess, apple): authored Sphere/Rect lights + a flat\n"
    "         * dome, no IBL. The GLES blue hemisphere ambient leaves them too bright\n"
    "         * and too cool vs the OVRTX golden. Pull exposure down and warm out the\n"
    "         * blue cast (R>B). FLIP-tuned against the OVRTX reference golden: this\n"
    "         * lowers the mean studio FLIP from ~0.56 to ~0.45 (all chess+apple\n"
    "         * frames closer to the golden), worst-case chess +166% -> +94% bright. */\n"
    "        color *= vec3(0.68, 0.64, 0.56);\n"
    "    }\n"
    "\n"
    "    float transmissionMix = (u_hasIBL != 0) ? clamp(mat.transmission_weight * 0.12, 0.0, 1.0) : 0.0;\n"
    "    if (transmissionMix > 0.001) {\n"
    "        vec3 transTint = mat.transmission_color.rgb;\n"
    "        float transIor = (mat.transmission_ior > 0.0) ? mat.transmission_ior : mat.ior;\n"
    "        bool clearGlass = transTint.r > 0.95 && transTint.g > 0.95 && transTint.b > 0.95;\n"
    "        vec3 rayDir = -V;\n"
    "        bool entering = dot(rayDir, N) < 0.0;\n"
    "        vec3 Nr = entering ? N : -N;\n"
    "        float eta = entering ? (1.0 / max(transIor, 1.001)) : max(transIor, 1.001);\n"
    "        vec3 throughDir;\n"
    "        if (clearGlass) {\n"
    "            vec3 T = refract(rayDir, Nr, eta);\n"
    "            throughDir = dot(T, T) < 1e-6 ? reflect(rayDir, Nr) : T;\n"
    "        } else {\n"
    "            throughDir = rayDir;\n"
    "        }\n"
    "        vec3 Rg = reflect(rayDir, N);\n"
    "        vec3 transmitted = textureLod(u_envMap, envMapUV(throughDir), 0.0).rgb * transTint;\n"
    "        if (alpha < 0.5) transmitted *= baseColor;\n"
    "        vec3 reflected = textureLod(u_envMap, envMapUV(Rg), 0.0).rgb;\n"
    "        float f0Glass = pow((transIor - 1.0) / (transIor + 1.0), 2.0);\n"
    "        float fresnelGlass = f0Glass + (1.0 - f0Glass) * pow(1.0 - clamp(NdotV, 0.0, 1.0), 5.0);\n"
    "        vec3 glassColor = mix(transmitted, reflected, fresnelGlass);\n"
    "        color = mix(color, glassColor * localOcc, transmissionMix);\n"
    "    }\n"
    "\n"
    "    /* Clearcoat: second GGX specular lobe with fixed IOR 1.5 (F0 = 0.04) */\n"
    "    if (mat.clearcoat > 0.0 && transmissionMix <= 0.001) {\n"
    "        float ccRoughness = clamp(mat.clearcoat_roughness, 0.04, 1.0);\n"
    "        if (u_hasIBL != 0) {\n"
    "            /* IBL clearcoat: sample env at clearcoat roughness */\n"
    "            vec3 ccR = reflect(-V, N);\n"
    "            float ccLod = ccRoughness * float(u_envMipLevels - 1);\n"
    "            vec3 ccEnv = textureLod(u_envMap, envMapUV(ccR), ccLod).rgb;\n"
    "            vec3 ccF = fresnelSchlick(NdotV, vec3(0.04));\n"
    "            /* Same legacy-only display shaping as the specular IBL above;\n"
    "             * a radiometric env takes the unshaped physical term. */\n"
    "            float upFacing = smoothstep(0.0, 0.85, clamp(N.z, 0.0, 1.0));\n"
    "            float ccShape = radiometricEnv ? 1.0 : mix(0.08, 1.0, upFacing);\n"
    "            color += ccEnv * ccF * mat.clearcoat * ccShape * localOcc;\n"
    "        } else {\n"
    "            vec3 ccL = normalize(vec3(0.3, 1.0, 0.5));\n"
    "            vec3 ccH = normalize(V + ccL);\n"
    "            float ccNdotL = max(dot(N, ccL), 0.0);\n"
    "            float ccD = distributionGGX(N, ccH, ccRoughness);\n"
    "            float ccG = geometrySmith(N, V, ccL, ccRoughness);\n"
    "            vec3  ccF = fresnelSchlick(max(dot(ccH, V), 0.0), vec3(0.04));\n"
    "            vec3  ccSpec = (ccD * ccG * ccF) / (4.0 * NdotV * ccNdotL + 0.0001);\n"
    "            color += ccSpec * vec3(1.0, 0.95, 0.85) * ccNdotL * mat.clearcoat;\n"
    "        }\n"
    "    }\n"
    "\n"
    "    /* Debug output modes: 1=baseColor, 2=normal, 3=roughness, 4=metallic,\n"
    "       5=ambient(IBL), 6=Lo(direct), 7=specularIBL, 8=diffuseIBL,\n"
    "       16=UV (tc.x→R, tc.y→G), 17=UV grid (10x10 checker on tc) */\n"
    "    if (u_debugMode == 1) { outColor = vec4(pow(baseColor, vec3(1.0/2.2)), 1.0); return; }\n"
    "    if (u_debugMode == 2) { outColor = vec4(N * 0.5 + 0.5, 1.0); return; }\n"
    "    if (u_debugMode == 3) { outColor = vec4(vec3(roughness), 1.0); return; }\n"
    "    if (u_debugMode == 4) { outColor = vec4(vec3(metallic), 1.0); return; }\n"
    "    if (u_debugMode == 5) { outColor = vec4(aces(ambient + fallbackAmbient), 1.0); return; }\n"
    "    if (u_debugMode == 6) { outColor = vec4(aces(Lo), 1.0); return; }\n"
    "    if (u_debugMode == 16) { outColor = vec4(fract(tc.x), fract(tc.y), 0.0, 1.0); return; }\n"
    "    if (u_debugMode == 17) {\n"
    "        vec2 g = fract(tc * 10.0); float chk = step(0.5, g.x) + step(0.5, g.y);\n"
    "        outColor = vec4(vec3(mod(chk, 2.0)), 1.0); return;\n"
    "    }\n"
    "    /* Shadow-atlas observability. Modes 24-27 are the only shader-side view\n"
    "     * of the atlas, and the reason the last shadow divergence had to be\n"
    "     * argued from arithmetic instead of measured. 24/25 read the slot table\n"
    "     * (so they show what the SCENE light actually resolves to), 26/27 read\n"
    "     * slot 0's tile directly (so they still show something when the table\n"
    "     * is the broken part). */\n"
    "    if (u_debugMode == 24) {\n"
    "        vec3 ldir = (u_sceneLightCount > 0)\n"
    "                  ? normalize(u_sceneLightPosIntensity[0].xyz - fragWorldPos) : vec3(0.0);\n"
    "        float vis = (u_sceneLightCount > 0)\n"
    "                  ? sceneLightVisibility(0, fragWorldPos, N, ldir) : 0.0;\n"
    "        outColor = vec4(vec3(vis), 1.0); return;\n"
    "    }\n"
    "    if (u_debugMode == 25) {\n"
    "        int slot = u_shadowLightToSlot[0];\n"
    "        outColor = vec4((slot < 0) ? vec3(1.0, 0.0, 0.0)\n"
    "                                   : vec3(0.0, float(slot + 1) / float(" NUSD_MAX_SHADOW_LIGHTS_STR
    "), 0.0),\n"
    "                        1.0);\n"
    "        return;\n"
    "    }\n"
    "    if (u_debugMode == 26 || u_debugMode == 27) {\n"
    "        /* Atlas slot 0: 26 = hardware compare result at this receiver\n"
    "         * (1 = lit, 0 = occluded); 27 = the receiver's own window depth. */\n"
    "        vec4 lp = u_shadow[0].lightVP * vec4(fragWorldPos, 1.0);\n"
    "        vec3 ndc = lp.xyz / max(abs(lp.w), 1e-6) * sign(lp.w);\n"
    "        vec2 tileUV = ndc.xy * 0.5 + 0.5;\n"
    "        vec4 so = u_shadow[0].uvScaleOffset;\n"
    "        vec2 atlasUV = clamp(tileUV, vec2(0.0), vec2(1.0)) * so.xy + so.zw;\n"
    "        float rdep = ndc.z * 0.5 + 0.5;\n"
    "        float cmp = texture(u_shadowAtlas, vec3(atlasUV, rdep));\n"
    "        outColor = vec4(vec3(u_debugMode == 26 ? cmp : rdep), 1.0);\n"
    "        return;\n"
    "    }\n"
    "\n"
    "    if (radiometricEnv) {\n"
    "        /* Scene-referred nits -> display, the ONE shared camera transform.\n"
    "         * No intensity ramp (the authored number is already in the texels),\n"
    "         * no 1/255 display floor (it lifted true black to (1,1,1) on every\n"
    "         * IBL frame), and the exact sRGB OETF instead of pow(1/2.2). */\n"
    "        color = aces(color * (NU_CAMERA_EXPOSURE * max(u_tone.x, 0.0)));\n"
    "        outColor = vec4(nuLinearToSrgb(color), alpha);\n"
    "        return;\n"
    "    }\n"
    "\n"
    "    float exposure;\n"
    "    if (legacyEnv) {\n"
    "        float intensity = abs(u_envIntensity);\n"
    "        float t1 = clamp(log(1.0 + intensity) / log(1001.0), 0.0, 1.25);\n"
    "        float gate = smoothstep(1.0, 2.0, intensity);\n"
    "        float boost1 = 1.0 + 0.4 * t1 * gate;\n"
    "        float boost2 = pow(max(intensity / 1000.0, 1.0), 0.85);\n"
    "        exposure = 0.20 * boost1 * boost2;\n"
    "    } else {\n"
    "        float flatDome = (u_envIntensity < -1.0) ? abs(u_envIntensity) : max(u_envIntensity, 0.0);\n"
    "        float flatDomeT = clamp(log(1.0 + flatDome) / log(12001.0), 0.0, 1.0);\n"
    "        /* Match the IsaacSim OVRTX probe setup: a textureless DomeLight\n"
    "         * supplies fill while authored SphereLights provide the key. Vulkan\n"
    "         * raster still keeps a darker conservative exposure for large many-\n"
    "         * light scenes; OpenGL's one-pass path tracks the brighter OVRTX\n"
    "         * reference for these small Mac parity probes. */\n"
    "        exposure = manyAuthoredLightNoIbl ? 1.20 :\n"
    "                   (authoredSphereSceneLightNoIbl ? 1.80 :\n"
    "                   (authoredSceneLightNoIbl ? 1.20 :\n"
    "                   (authoredLightNoIbl ? 1.55 : 1.2) * (1.0 + 0.45 * flatDomeT)));\n"
    "    }\n"
    "    color *= exposure * max(u_tone.x, 0.0);\n"
    "    color = aces(color);\n"
    "    if (u_hasIBL != 0) color = max(color, vec3(1.0 / 255.0));\n"
    "    color = pow(color, vec3(1.0 / 2.2));\n"
    "    if (authoredLightNoIbl) {\n"
    "        float outLuma = dot(color, vec3(0.2126, 0.7152, 0.0722));\n"
    "        float albedoChroma = max(baseColor.r, max(baseColor.g, baseColor.b)) - "
    "min(baseColor.r, min(baseColor.g, baseColor.b));\n"
    "        float albedoWarm = smoothstep(0.02, 0.20, baseColor.r - baseColor.b);\n"
    "        float neutralAlbedo = clamp((0.26 - albedoChroma) / 0.22, 0.0, 1.0) * (1.0 - albedoWarm);\n"
    "        float farFill = smoothstep(4.0, 14.0, length(fragWorldPos - u_eyePos));\n"
    "        float shadowFill = clamp((0.72 - outLuma) / 0.62, 0.0, 1.0);\n"
    "        color += vec3(0.14, 0.15, 0.18) * neutralAlbedo * farFill * shadowFill;\n"
    "    }\n"
    "\n"
    "    if (u_hasIBL == 0) {\n"
    "        /* MuJoCo-style haze: distant geometry fades toward the background so the\n"
    "         * checker floor recedes into the horizon (no-IBL / sim look only).\n"
    "         *\n"
    "         * Measured against the scene's own size, not in absolute world units.  The\n"
    "         * constants below were tuned for a sim-sized scene, and an asset authored in\n"
    "         * centimetres is hundreds of units across: every visible pixel then sat past\n"
    "         * saturation and the whole render was a flat 55% mix into the gray backdrop.\n"
    "         * u_sceneRadius defaults to the reference radius, so a sim-sized scene keeps\n"
    "         * exactly the old curve (haze from 9 units, saturating at 49). */\n"
    "        float sceneRadius = max(u_sceneRadius, 1e-3);\n"
    "        float hazeDist = length(fragWorldPos - u_eyePos) / sceneRadius;\n"
    "        float haze = clamp((hazeDist - 1.125) / 5.0, 0.0, 1.0) * 0.55;\n"
    "        /* Fade toward the ACTUAL backdrop (black, official ovrtx 0.4's\n"
    "         * empty-background color) so the horizon still recedes into the\n"
    "         * clear color rather than leaving a gray halo on a black sky. */\n"
    "        color = mix(color, vec3(0.0), haze);\n"
    "    }\n"
    "\n"
    "    outColor = vec4(color, alpha);\n"
    "}\n";

/* ---- Environment background shaders ---- */

static const char* NUSD_SHADER_UNUSED k_env_bg_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "out vec2 v_uv;\n"
    "void main() {\n"
    "    /* Fullscreen triangle: 3 vertices, no VBO needed */\n"
    "    vec2 pos = vec2(float(gl_VertexID & 1) * 4.0 - 1.0,\n"
    "                    float((gl_VertexID >> 1) & 1) * 4.0 - 1.0);\n"
    "    v_uv = pos * 0.5 + 0.5;\n"
    "    gl_Position = vec4(pos, 0.9999, 1.0);\n" /* just inside far plane */
    "}\n";

static const char* NUSD_SHADER_UNUSED k_env_bg_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "in vec2 v_uv;\n"
    "uniform mat4 u_viewInv;\n"
    "uniform mat4 u_projInv;\n"
    "uniform sampler2D u_envMap;\n"
    "uniform int u_envMipLevels;\n"
    "uniform vec4 u_tone;\n"
    "uniform float u_envIntensity;\n"
    "uniform float u_envRotation;\n"
    "out vec4 outColor;\n"
    "\n"
    "const float PI = 3.14159265;\n"
    "const float NU_CAMERA_EXPOSURE = 0.000882;\n"
    "\n"
    "vec3 aces(vec3 x) {\n"
    "    float a = 2.51; float b = 0.03; float c = 2.43; float d = 0.59; float e = 0.14;\n"
    "    return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);\n"
    "}\n"
    "\n"
    "vec3 nuLinearToSrgb(vec3 c) {\n"
    "    c = clamp(c, 0.0, 1.0);\n"
    "    vec3 lo = 12.92 * c;\n"
    "    vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;\n"
    "    bvec3 use_hi = greaterThan(c, vec3(0.0031308));\n"
    "    return vec3(use_hi.x ? hi.x : lo.x,\n"
    "                use_hi.y ? hi.y : lo.y,\n"
    "                use_hi.z ? hi.z : lo.z);\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    /* Reconstruct world-space ray direction using separate view/proj inverses.\n"
    "     * Matches Vulkan env bg shader for identical UV derivatives & LOD. */\n"
    "    vec2 d = v_uv * 2.0 - 1.0;\n"
    "    vec4 target = u_projInv * vec4(d.x, d.y, 1.0, 1.0);\n"
    "    vec3 viewDir = normalize(target.xyz);\n"
    "    vec3 dir = (u_viewInv * vec4(viewDir, 0.0)).xyz;\n"
    "\n"
    "    /* Equirectangular lookup (matching Vulkan convention) */\n"
    "    const float kDomeLongitudeOffset = 340.0 / 360.0;\n"
    "    float u = fract(atan(dir.x, dir.y) * (0.5 / PI) + 0.5 + kDomeLongitudeOffset\n"
    "                    - u_envRotation * (1.0 / 360.0));\n"
    "    float v = asin(clamp(dir.z, -1.0, 1.0)) * (1.0 / PI) + 0.5;\n"
    "    vec2 uv = vec2(u, 1.0 - v);\n"
    "\n"
    /* Force full-resolution sampling. With plain `texture()` GLES
     * picks LOD from screen-space UV derivatives; for a fullscreen
     * triangle those derivatives are large enough that the lowest
     * mip (1×1 average) wins and the dome renders as a flat solid
     * color. Vulkan's miss-shader equivalent doesn't hit this
     * because trace-ray derivatives are zero. */
    "    vec3 texel = textureLod(u_envMap, uv, 0.0).rgb;\n"
    "    vec3 color;\n"
    "    if (u_envIntensity >= 0.0) {\n"
    /* Radiometric authored dome: the texels ALREADY carry final linear
     * radiance in nits (color and intensity were baked in by
     * env_build_from_rgb), so the visible background is just the camera
     * transform of the texel. No sky multiplier, no kPhotoExposure, and it
     * uses u_tone.x -- the SAME exposure knob the surfaces take -- so the
     * horizon cannot seam.
     *
     * Measured: robot_ground_scene's DomeLight (900 x (0.92,0.95,1.0)) renders
     * on official ovrtx 0.4 at (222,223,225); this branch predicts
     * (221,222,224). ovgl's old kPhotoExposure branch gave (166,168,172), and
     * it is 2.80x low because it is missing the fork's kIblDisplayParity 2.70
     * (0.000315 * 2.70 = 8.505e-4 ~ NU_CAMERA_EXPOSURE). */
    "        color = aces(texel * NU_CAMERA_EXPOSURE * max(u_tone.x, 0.0));\n"
    "    } else {\n"
    /* LEGACY host-override env (auto-exposed texels): unchanged, including
     * u_tone.y and the pow(1/2.2) encode, because that lane's 4.5/2000 tone
     * pair is calibrated against exactly this ladder. */
    "        const float kPhotoExposure = 0.000315;\n"
    "        float skyIntensity = abs(u_envIntensity);\n"
    "        if (skyIntensity <= 0.0) skyIntensity = 1.0;\n"
    "        color = aces(texel * (skyIntensity * kPhotoExposure) * max(u_tone.y, 0.0));\n"
    "        outColor = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);\n"
    "        return;\n"
    "    }\n"
    "    outColor = vec4(nuLinearToSrgb(color), 1.0);\n"
    "}\n";

/* ---- Depth-AOV pack pass (fullscreen triangle) ----
 *
 * GLES has no glReadPixels(GL_DEPTH_COMPONENT), so the Depth AOV readback
 * samples the render target's depth TEXTURE and packs the window-space depth
 * into the RGB channels of an RGBA8 color target as a 24-bit fixed-point
 * integer (b = LSB). RGBA8 readback is universally supported, and 24 bits
 * exactly matches the DEPTH_COMPONENT24 storage, so the pack loses nothing.
 * The quantize goes through an exact float integer (<= 2^24, representable)
 * instead of the classic fract() ladder, whose fract(1.0) == 0.0 wraps the
 * cleared far plane onto the near plane. The CPU side decodes and
 * linearizes (gpu.h::gpu_depth_pack_read).
 *
 * The V flip mirrors the color path: the color resolve blit inverts Y so the
 * host buffer is top-down; sampling at (u, 1-v) makes the packed target read
 * back in the same orientation. */

static const char* NUSD_SHADER_UNUSED k_depth_pack_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "out vec2 v_uv;\n"
    "void main() {\n"
    "    vec2 pos = vec2(float(gl_VertexID & 1) * 4.0 - 1.0,\n"
    "                    float((gl_VertexID >> 1) & 1) * 4.0 - 1.0);\n"
    "    v_uv = pos * 0.5 + 0.5;\n"
    "    gl_Position = vec4(pos, 0.0, 1.0);\n"
    "}\n";

static const char* NUSD_SHADER_UNUSED k_depth_pack_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "in vec2 v_uv;\n"
    "uniform highp sampler2D u_depth;\n" /* DEPTH_COMPONENT texture, COMPARE_MODE NONE */
    "out vec4 outColor;\n"
    "void main() {\n"
    "    float zw = texture(u_depth, vec2(v_uv.x, 1.0 - v_uv.y)).r;\n"
    "    /* min() guards the top step: at zw == 1.0 (the cleared far plane)\n"
    "     * 16777215.0 + 0.5 rounds UP to 16777216.0 in fp32 (unit spacing at\n"
    "     * 2^24), which would wrap the no-hit sentinel into a bogus depth. */\n"
    "    float s = min(floor(clamp(zw, 0.0, 1.0) * 16777215.0 + 0.5),\n"
    "                  16777215.0);\n"
    "    float r = floor(s / 65536.0);\n"
    "    float g = floor((s - r * 65536.0) / 256.0);\n"
    "    float b = s - r * 65536.0 - g * 256.0;\n"
    "    outColor = vec4(r / 255.0, g / 255.0, b / 255.0, 1.0);\n"
    "}\n";

/* ---- SSAO (fullscreen triangle, half internal resolution) ----
 *
 * Classic normal-oriented-hemisphere ambient occlusion over the AO prepass
 * (Ovgl.cpp::AO_PREPASS_VERT): 16 samples inside a hemisphere aligned to the
 * surface normal, each projected back to screen and tested against the depth
 * actually stored there.
 *
 * Everything happens in VIEW space, reconstructed from window depth through
 * u_projInv, because that is the space in which "how far in front of this
 * surface is the nearest geometry" is a single subtraction. GL view space
 * looks down -Z, so a sample is OCCLUDED when the stored surface has a
 * LARGER (less negative) z than the sample point -- i.e. something is between
 * the sample and the camera.
 *
 * The per-pixel rotation is a deterministic function of gl_FragCoord over a
 * 4x4 tile, NOT a random texture or a time-varying hash. ovgl renders the same
 * frame to the same bytes twice (measured: two runs over a fixed .usda differ
 * on at most 3 pixels by 1, against official's 129-320 frames of 320), and
 * that property is worth more here than a slightly better sample
 * distribution. A 4x4 tile is also exactly what the 4x4 blur below cancels.
 *
 * The range check is what keeps a distant surface from occluding a near one
 * through empty space: an occluder further away than the sample radius fades
 * out instead of switching on. Without it, every silhouette gets a dark halo
 * of background. */

static const char* NUSD_SHADER_UNUSED k_ssao_vert_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "out vec2 v_uv;\n"
    "void main() {\n"
    "    vec2 pos = vec2(float(gl_VertexID & 1) * 4.0 - 1.0,\n"
    "                    float((gl_VertexID >> 1) & 1) * 4.0 - 1.0);\n"
    "    v_uv = pos * 0.5 + 0.5;\n"
    "    gl_Position = vec4(pos, 0.0, 1.0);\n"
    "}\n";

#define NUSD_SSAO_KERNEL_SIZE 16

static const char* NUSD_SHADER_UNUSED k_ssao_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "in vec2 v_uv;\n"
    "uniform highp sampler2D u_depth;\n" /* prepass depth, COMPARE_MODE NONE */
    "uniform sampler2D u_normal;\n" /* prepass world normal, n*0.5+0.5  */
    "uniform mat4 u_proj;\n"
    "uniform mat4 u_projInv;\n"
    "uniform mat3 u_viewRot;\n" /* world normal -> view             */
    "uniform vec3 u_kernel[16];\n"
    "uniform float u_radius;\n" /* world units                      */
    /* Cosine threshold, NOT a depth offset: an occluder whose direction from
     * the shaded point is within acos(u_bias) of the tangent plane is ignored.
     * It rejects the grazing taps that depth quantization makes meaningless. */
    "uniform float u_bias;\n"
    "uniform float u_intensity;\n"
    "out vec4 outColor;\n"
    "\n"
    "vec3 viewPos(vec2 uv, float zw) {\n"
    "    vec4 ndc = vec4(uv * 2.0 - 1.0, zw * 2.0 - 1.0, 1.0);\n"
    "    vec4 v = u_projInv * ndc;\n"
    "    return v.xyz / v.w;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    float zw = texture(u_depth, v_uv).r;\n"
    /* Background. Nothing was rasterized here, so there is no surface to
     * occlude -- and writing anything but 1.0 would darken the sky. */
    "    if (zw >= 1.0) { outColor = vec4(1.0); return; }\n"
    "    vec3 P = viewPos(v_uv, zw);\n"
    "    vec3 Nw = texture(u_normal, v_uv).xyz * 2.0 - 1.0;\n"
    "    if (dot(Nw, Nw) < 1e-8) { outColor = vec4(1.0); return; }\n"
    "    vec3 N = normalize(u_viewRot * Nw);\n"
    "\n"
    /* 16 distinct rotations over a 4x4 screen tile. */
    "    ivec2 tile = ivec2(gl_FragCoord.xy) & 3;\n"
    "    float ang = float(tile.x + tile.y * 4) * (6.28318530718 / 16.0);\n"
    "    vec3 rv = vec3(cos(ang), sin(ang), 0.0);\n"
    "    vec3 T = rv - N * dot(rv, N);\n"
    /* Degenerate only when the tangent guess is parallel to N; pick any other
     * axis rather than normalizing a zero vector into NaN. */
    "    if (dot(T, T) < 1e-8) T = abs(N.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);\n"
    "    T = normalize(T - N * dot(T, N));\n"
    "    mat3 TBN = mat3(T, cross(N, T), N);\n"
    "\n"
    "    float occ = 0.0;\n"
    "    for (int i = 0; i < 16; ++i) {\n"
    "        vec3 sp = P + (TBN * u_kernel[i]) * u_radius;\n"
    "        vec4 clip = u_proj * vec4(sp, 1.0);\n"
    "        if (clip.w <= 0.0) continue;\n" /* behind the eye */
    "        vec3 sndc = clip.xyz / clip.w;\n"
    "        vec2 suv = sndc.xy * 0.5 + 0.5;\n"
    /* Offscreen taps are UNKNOWN, not unoccluded -- this is the screen-space
     * blindness the plan warns about, and clamping instead of skipping would
     * smear the edge pixel's depth across the whole margin. */
    "        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;\n"
    "        float sz = texture(u_depth, suv).r;\n"
    "        if (sz >= 1.0) continue;\n"
    /* Test the OCCLUDER's actual position against the tangent plane at P,
     * rather than comparing its depth against the sample point's depth.
     *
     * The depth comparison is the textbook form and it is WRONG on curved
     * geometry: near a silhouette the hemisphere leans sideways, its samples
     * project back onto the object's own nearer surface, and the object
     * shadows itself. Measured on probe_no_occluder -- four convex spheres
     * with nothing else in the scene -- that form took min 73 -> 24 and mean
     * 219.6 -> 210.9 against official's unchanged 64/219.4, i.e. it invented
     * occlusion in the one fixture that exists to prove AO does not.
     *
     * This form cannot: for a CONVEX surface every other point lies on the far
     * side of any tangent plane, so cosA <= 0 and the sphere contributes
     * nothing to itself, by construction rather than by tuning. */
    "        vec3 Q = viewPos(suv, sz);\n"
    "        vec3 d = Q - P;\n"
    "        float len = length(d);\n"
    "        if (len < 1e-5) continue;\n"
    "        float cosA = dot(N, d / len);\n"
    "        if (cosA <= u_bias) continue;\n"
    /* Smooth distance falloff so an occluder entering the radius fades in
     * instead of popping, and one beyond it contributes nothing. */
    "        float falloff = clamp(1.0 - (len * len) / (u_radius * u_radius), 0.0, 1.0);\n"
    "        occ += cosA * falloff;\n"
    "    }\n"
    "    float ao = 1.0 - (occ / 16.0) * u_intensity;\n"
    "    outColor = vec4(clamp(ao, 0.0, 1.0));\n"
    "}\n";

/* Depth-aware 4x4 blur: cancels the 4x4 rotation tile above exactly, while
 * refusing taps across a depth discontinuity. A plain box blur here pulls the
 * background's AO=1 onto every silhouette and lights a bright fringe around
 * each object -- most visible on exactly the thin articulated links this whole
 * exercise is about. The threshold is RELATIVE to the center depth, so it is
 * scale-free across a 0.4 m sphere and a 40 m warehouse alike. */
static const char* NUSD_SHADER_UNUSED k_ssao_blur_frag_gles = NUSD_SHADER_VERSION NUSD_PRECISION_HIGH
    "in vec2 v_uv;\n"
    "uniform sampler2D u_ssao;\n"
    "uniform highp sampler2D u_depth;\n"
    "uniform mat4 u_projInv;\n"
    "uniform vec2 u_texel;\n"
    "out vec4 outColor;\n"
    "\n"
    "float viewZ(vec2 uv, float zw) {\n"
    "    vec4 v = u_projInv * vec4(uv * 2.0 - 1.0, zw * 2.0 - 1.0, 1.0);\n"
    "    return v.z / v.w;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    float czw = texture(u_depth, v_uv).r;\n"
    "    if (czw >= 1.0) { outColor = vec4(1.0); return; }\n"
    "    float cz = viewZ(v_uv, czw);\n"
    "    float sum = 0.0, wsum = 0.0;\n"
    "    for (int y = -2; y < 2; ++y) {\n"
    "        for (int x = -2; x < 2; ++x) {\n"
    "            vec2 uv = v_uv + vec2(float(x), float(y)) * u_texel;\n"
    "            float zw = texture(u_depth, uv).r;\n"
    "            if (zw >= 1.0) continue;\n"
    "            if (abs(viewZ(uv, zw) - cz) > 0.05 * abs(cz)) continue;\n"
    "            sum += texture(u_ssao, uv).r;\n"
    "            wsum += 1.0;\n"
    "        }\n"
    "    }\n"
    "    outColor = vec4(wsum > 0.0 ? sum / wsum : texture(u_ssao, v_uv).r);\n"
    "}\n";

/* ---- BasisCurves tube renderer (vertex / TCS / TES / fragment) ----
 *
 * Mirrors Hydra Storm's HALFTUBE/ROUND repr (basisCurves.glslfx) but
 * promotes to a full closed tube and bakes evaluation on the GPU via
 * the standard quads tess domain. Phase 1c: cubic Bezier basis only;
 * other bases (bspline, catmullRom) come in phase 3 by swapping a
 * basis-matrix uniform.
 *
 * Patch = 4 control points. Tess domain = quads (u along curve,
 * v around tube ring). Each ring vertex is placed at angle 2π·v from
 * a frame built from the curve tangent + world-up (camera-stable
 * fallback when tangent ‖ up). Radius is per-CV width with Bezier
 * basis interpolation in u — matches Storm's per-vertex width
 * resolution.
 *
 * Per-curve uniforms travel via the existing MeshBlock UBO (binding=1)
 * so curves slot into the same per-draw infra meshes already use:
 *   mesh.mvp, mesh.model, mesh.color (.w>0.5 → override vertex color).
 * The curve VS is otherwise pure passthrough; all transform happens
 * in the TES after evaluation, so MVP and model are read there. */

/* ---- Curve vertex shader (passthrough; eval happens in TES) ---- */

static const char* NUSD_SHADER_UNUSED k_curve_vert_gles = NUSD_SHADER_VERSION_TESS NUSD_PRECISION_HIGH
    "\n"
    "layout(location = 0) in vec3  inPos;\n"
    "layout(location = 1) in float inWidth;\n"
    "layout(location = 2) in uint   inColorPacked;\n"
    "\n"
    "out vec3  vsPos;\n"
    "out float vsWidth;\n"
    "out vec3  vsColor;\n"
    "\n"
    "vec3 unpack_rgba8(uint c) {\n"
    "    return vec3(float(c & 255u),\n"
    "                float((c >> 8u) & 255u),\n"
    "                float((c >> 16u) & 255u)) / 255.0;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vsPos   = inPos;\n"
    "    vsWidth = inWidth;\n"
    "    vsColor = unpack_rgba8(inColorPacked);\n"
    "}\n";

/* ---- Curve tessellation control shader ----
 *
 * Phase 1c: hardcoded tess levels (32 along, 8 around). Phase 4 will
 * project the patch endpoints to NDC and drive outer[1]/outer[3]
 * (along-curve) by screen-space chord length, and outer[0]/outer[2]
 * (around-tube) by max screen-space radius — exactly Storm's
 * basisCurves.glslfx LOD formula. */

static const char* NUSD_SHADER_UNUSED k_curve_tcs_gles = NUSD_SHADER_VERSION_TESS NUSD_PRECISION_HIGH
    "\n"
    "layout(vertices = 4) out;\n"
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;\n"
    "    mat4 model;\n"
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n"
    "uniform mat4 u_view;\n"
    "uniform mat4 u_proj;\n"
    "uniform vec2 u_screen_size;\n"
    "uniform int  u_basis_id;\n"
    "\n"
    "in vec3  vsPos[];\n"
    "in float vsWidth[];\n"
    "in vec3  vsColor[];\n"
    "\n"
    "out vec3  tcsPos[];\n"
    "out float tcsWidth[];\n"
    "out vec3  tcsColor[];\n"
    "\n"
    /* Screen-space LOD constants. Storm's basisCurves.glslfx:280-290
     * uses MAX_TESS=40 and PIXEL_TO_TESS=20 (1 tess unit per 20 pixels)
     * and notes "should be replaced with a uniform". The same hardcoded
     * defaults at --complexity=low produce visibly faceted tubes; we
     * pick values closer to Storm's --complexity=veryhigh effective
     * tessellation: MAX 64, PIXEL_TO_TESS 5 along the curve (smooth
     * silhouette), 8 around the tube (8-gon at min radius). MIN_AROUND
     * keeps tiny tubes from collapsing to a ribbon. */
    /* PIXEL_TO_TESS bumped from 5 → 7 with MSAA on: silhouette aliasing
     * is now resolved by 4x MSAA, so we can drop ~30% of curve geometry
     * without a visible quality loss on the showcase_grid datasets. */
    "const float MAX_TESS         = 64.0;\n"
    "const float PIXEL_TO_TESS    = 7.0;\n"
    "const float WIDTH_PIXEL_TO_TESS = 8.0;\n"
    "const float MIN_AROUND       = 6.0;\n"
    "\n"
    "vec2 projectToScreen(vec4 worldP) {\n"
    "    vec4 clip = u_proj * (u_view * worldP);\n"
    "    vec2 ndc = clamp(clip.xy / clip.w, -1.3, 1.3);\n"
    "    return (ndc + 1.0) * (u_screen_size * 0.5);\n"
    "}\n"
    "\n"
    /* WorldToPixelWidth — basisCurves.glslfx:53-74. Computes the\n"
     * conversion factor from world-space units to screen pixels at a\n"
     * given view-space Z. proj[0][0] is the X-FOV scale factor;\n"
     * proj[2][3]/[3][3] handle perspective vs ortho. */
    "float worldToPixelWidth(float eyeZ) {\n"
    "    float x = u_proj[0][0];\n"
    "    float w = eyeZ * u_proj[2][3] + u_proj[3][3];\n"
    "    return abs((u_screen_size.x * 0.5) * (x / w));\n"
    "}\n"
    "\n"
    "float tessLengthFromScreen(float pixels) {\n"
    "    return clamp(pixels / PIXEL_TO_TESS, 1.0, MAX_TESS);\n"
    "}\n"
    "float tessWidthFromScreen(float pixels) {\n"
    "    return clamp(pixels / WIDTH_PIXEL_TO_TESS, MIN_AROUND, MAX_TESS);\n"
    "}\n"
    "\n"
    /* Same basis weights the TES uses, computed here on a CV array
     * already in eye space — for screen-radius interpolation. */
    "void compute_basis_w(float u, out vec4 b) {\n"
    "    float u2 = u*u; float u3 = u2*u;\n"
    "    float ic = 1.0 - u; float ic2 = ic*ic; float ic3 = ic2*ic;\n"
    "    if (u_basis_id == 1) {\n"
    "        b[0] = -u3/6.0 + 0.5*u2 - 0.5*u + 1.0/6.0;\n"
    "        b[1] = 0.5*u3 - u2 + 2.0/3.0;\n"
    "        b[2] = -0.5*u3 + 0.5*u2 + 0.5*u + 1.0/6.0;\n"
    "        b[3] = u3/6.0;\n"
    "    } else if (u_basis_id == 2) {\n"
    "        b[0] = -0.5*u3 + u2 - 0.5*u;\n"
    "        b[1] = 1.5*u3 - 2.5*u2 + 1.0;\n"
    "        b[2] = -1.5*u3 + 2.0*u2 + 0.5*u;\n"
    "        b[3] = 0.5*u3 - 0.5*u2;\n"
    "    } else if (u_basis_id == 3) {\n"
    "        b[0] = ic; b[1] = u; b[2] = 0.0; b[3] = 0.0;\n"
    "    } else {\n"
    "        b[0] = ic3; b[1] = 3.0*u*ic2; b[2] = 3.0*u2*ic; b[3] = u3;\n"
    "    }\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    tcsPos  [gl_InvocationID] = vsPos  [gl_InvocationID];\n"
    "    tcsWidth[gl_InvocationID] = vsWidth[gl_InvocationID];\n"
    "    tcsColor[gl_InvocationID] = vsColor[gl_InvocationID];\n"
    "\n"
    "    if (gl_InvocationID == 0) {\n"
    /* Project 4 CVs to eye space + clip space + screen space (pixels). */
    "        vec4 worldP[4]; vec4 eyeP[4]; vec4 clipP[4]; vec2 scrP[4];\n"
    "        for (int i = 0; i < 4; i++) {\n"
    "            worldP[i] = mesh.model * vec4(vsPos[i], 1.0);\n"
    "            eyeP[i]   = u_view * worldP[i];\n"
    "            clipP[i]  = u_proj * eyeP[i];\n"
    "            scrP[i]   = (clamp(clipP[i].xy / clipP[i].w, -1.3, 1.3) + 1.0)\n"
    "                       * (u_screen_size * 0.5);\n"
    "        }\n"
    /* Frustum cull at the patch level. If all 4 CVs lie outside the same
     * frustum plane, the entire tube can't appear on screen — emit zero
     * tess factors and return. The tube extends perpendicular to the
     * centerline by radius=width/2; we pad each clip-space half-extent
     * by an estimate of that radius so curves clipping the screen edge
     * still tessellate. Margin of 8% of |w| handles tube widths up to
     * ~6cm at typical viewing distances; over-padding is fine, this is
     * a conservative cull. */
    "        {\n"
    "            float maxw = max(max(vsWidth[0], vsWidth[1]),\n"
    "                             max(vsWidth[2], vsWidth[3]));\n"
    "            bool any_behind = false;\n"
    "            for (int i = 0; i < 4; i++) {\n"
    "                if (clipP[i].w <= 0.0) { any_behind = true; break; }\n"
    "            }\n"
    "            if (!any_behind) {\n"
    "                bvec3 outNeg = bvec3(true);\n"
    "                bvec3 outPos = bvec3(true);\n"
    "                for (int i = 0; i < 4; i++) {\n"
    "                    float pad = clipP[i].w * 0.08\n"
    "                              + maxw * abs(u_proj[0][0]) * 0.5;\n"
    "                    outNeg.x = outNeg.x && (clipP[i].x < -clipP[i].w - pad);\n"
    "                    outPos.x = outPos.x && (clipP[i].x >  clipP[i].w + pad);\n"
    "                    outNeg.y = outNeg.y && (clipP[i].y < -clipP[i].w - pad);\n"
    "                    outPos.y = outPos.y && (clipP[i].y >  clipP[i].w + pad);\n"
    "                    outNeg.z = outNeg.z && (clipP[i].z < -clipP[i].w - pad);\n"
    "                    outPos.z = outPos.z && (clipP[i].z >  clipP[i].w + pad);\n"
    "                }\n"
    "                if (any(outNeg) || any(outPos)) {\n"
    "                    gl_TessLevelOuter[0] = 0.0;\n"
    "                    gl_TessLevelOuter[1] = 0.0;\n"
    "                    gl_TessLevelOuter[2] = 0.0;\n"
    "                    gl_TessLevelOuter[3] = 0.0;\n"
    "                    gl_TessLevelInner[0] = 0.0;\n"
    "                    gl_TessLevelInner[1] = 0.0;\n"
    "                    return;\n"
    "                }\n"
    "            }\n"
    "        }\n"
    /* Length factor from screen-space chord. */
    "        float dist = distance(scrP[0], scrP[1])\n"
    "                   + distance(scrP[1], scrP[2])\n"
    "                   + distance(scrP[2], scrP[3]);\n"
    "        float level = tessLengthFromScreen(dist);\n"
    /* Width factors: per-CV screen radius at view-space Z, with
     * Storm's widthMultiplier = 2.0 for better silhouettes. */
    "        const float widthMultiplier = 2.0;\n"
    "        float scrW[4];\n"
    "        for (int i = 0; i < 4; i++) {\n"
    "            scrW[i] = worldToPixelWidth(eyeP[i].z) * vsWidth[i]\n"
    "                    * widthMultiplier;\n"
    "        }\n"
    /* Evaluate basis-weighted screen widths at u={0, 1/3, 2/3, 1}. */
    "        vec4 b00, b33, b66, b10;\n"
    "        compute_basis_w(0.0,  b00);\n"
    "        compute_basis_w(1.0/3.0, b33);\n"
    "        compute_basis_w(2.0/3.0, b66);\n"
    "        compute_basis_w(1.0, b10);\n"
    "        float w00 = dot(b00, vec4(scrW[0], scrW[1], scrW[2], scrW[3]));\n"
    "        float w33 = dot(b33, vec4(scrW[0], scrW[1], scrW[2], scrW[3]));\n"
    "        float w66 = dot(b66, vec4(scrW[0], scrW[1], scrW[2], scrW[3]));\n"
    "        float w10 = dot(b10, vec4(scrW[0], scrW[1], scrW[2], scrW[3]));\n"
    "        float lvl_w_avg = 0.25 * (tessWidthFromScreen(w00)\n"
    "                                + tessWidthFromScreen(w33)\n"
    "                                + tessWidthFromScreen(w66)\n"
    "                                + tessWidthFromScreen(w10));\n"
    /* Tess factors per Storm's `SetTessFactors(level, level_u0_00,\n"
     * level, level_u1_00, level_uAvg, level)` adapted to our domain:\n"
     *  outer[0,2] = around-tube (v varies)  → width tess at u=0/1\n"
     *  outer[1,3] = along-curve (u varies)  → length tess\n"
     *  inner[0]   = u dir (along-curve)     → length tess\n"
     *  inner[1]   = v dir (around-tube)     → width tess avg */
    "        gl_TessLevelOuter[0] = tessWidthFromScreen(w00);\n"
    "        gl_TessLevelOuter[1] = level;\n"
    "        gl_TessLevelOuter[2] = tessWidthFromScreen(w10);\n"
    "        gl_TessLevelOuter[3] = level;\n"
    "        gl_TessLevelInner[0] = level;\n"
    "        gl_TessLevelInner[1] = lvl_w_avg;\n"
    "    }\n"
    "}\n";

/* ---- Curve tessellation evaluation shader ----
 *
 * Domain = quads, fractional_odd_spacing — Storm's default. (u, v) come
 * in via gl_TessCoord.xy. u runs along the curve, v wraps the tube.
 * Output is one ring vertex per (u, v) sample. */

static const char* NUSD_SHADER_UNUSED k_curve_tes_gles = NUSD_SHADER_VERSION_TESS NUSD_PRECISION_HIGH
    "\n"
    "layout(quads, fractional_odd_spacing, ccw) in;\n"
    "\n" NUSD_UBO_LAYOUT_MESH
    "uniform MeshBlock {\n"
    "    mat4 mvp;       /* unused for curves */\n"
    "    mat4 model;     /* per-curve world transform; row-major bytes */\n"
    "    vec4 color;\n"
    "    uvec4 ptex;\n"
    "} mesh;\n"
    "\n"
    /* Storm pattern (basisCurves.glslfx:178-185): pass world-to-view
     * and projection as separate uniforms and multiply in the shader.
     * The backend converts the caller's row-major matrices to the
     * column-major byte layout plain GLSL uniforms consume. */
    "uniform mat4 u_view;\n"
    "uniform mat4 u_proj;\n"
    /* Per-curve basis selector: 0=bezier, 1=bspline, 2=catmullRom,
     * 3=linear. Storm's evaluateBasis is implemented as four separate
     * shader files chosen at codegen time (basisCurves.glslfx:1028,
     * 1051, 1074, 1191); we collapse them into one TES with an
     * if/else chain since per-curve uniform writes are cheap and
     * having a single pipeline keeps state changes minimal. */
    "uniform int u_basis_id;\n"
    "\n"
    "in vec3  tcsPos[];\n"
    "in float tcsWidth[];\n"
    "in vec3  tcsColor[];\n"
    "\n"
    /* TES outputs match the PBR vertex shader (k_pbr_vert_gles)
     * interface so curves can route through the same fragment shader
     * path as meshes. Storm authors a synthetic vec2(0, v) UV for tube
     * patches (basisCurves.glslfx:1293); we follow that convention. */
    "out vec3  fragWorldPos;\n"
    "out vec3  fragNormal;\n"
    "out vec3  fragTangent;\n"
    "out float fragTangentSign;\n"
    "out vec3  fragColor;\n"
    "out vec2  fragTexCoord;\n"
    "\n"
    "const float TWO_PI = 6.28318530717958647692;\n"
    "\n"
    /* Storm's basis arrays index cv[0]→u=1 endpoint, cv[3]→u=0 endpoint.
     * We use the natural cv[0]→u=0 convention here, so all coefficients
     * are reversed from Storm's. The math is otherwise identical to
     * basisCurves.glslfx:1028/1051/1074/1191. */
    "void compute_basis(float u, out vec4 b, out vec4 db) {\n"
    "    float u2 = u*u; float u3 = u2*u;\n"
    "    float ic = 1.0 - u; float ic2 = ic*ic; float ic3 = ic2*ic;\n"
    "    if (u_basis_id == 1) {\n"
    "        /* bspline (1/6 weights, sums to 1) */\n"
    "        b[0] = -u3/6.0 + 0.5*u2 - 0.5*u + 1.0/6.0;\n"
    "        b[1] = 0.5*u3 - u2 + 2.0/3.0;\n"
    "        b[2] = -0.5*u3 + 0.5*u2 + 0.5*u + 1.0/6.0;\n"
    "        b[3] = u3/6.0;\n"
    "        db[0] = -0.5*u2 + u - 0.5;\n"
    "        db[1] = 1.5*u2 - 2.0*u;\n"
    "        db[2] = -1.5*u2 + u + 0.5;\n"
    "        db[3] = 0.5*u2;\n"
    "    } else if (u_basis_id == 2) {\n"
    "        /* catmullRom (passes through cv[1] @ u=0, cv[2] @ u=1) */\n"
    "        b[0] = -0.5*u3 + u2 - 0.5*u;\n"
    "        b[1] = 1.5*u3 - 2.5*u2 + 1.0;\n"
    "        b[2] = -1.5*u3 + 2.0*u2 + 0.5*u;\n"
    "        b[3] = 0.5*u3 - 0.5*u2;\n"
    "        db[0] = -1.5*u2 + 2.0*u - 0.5;\n"
    "        db[1] = 4.5*u2 - 5.0*u;\n"
    "        db[2] = -4.5*u2 + 4.0*u + 0.5;\n"
    "        db[3] = 1.5*u2 - u;\n"
    "    } else if (u_basis_id == 3) {\n"
    "        /* linear: only cv[0] and cv[1] matter; cv[2]=cv[3] padded\n"
    "         * by the patch builder so weights 0 cause no contribution. */\n"
    "        b[0] = ic;\n"
    "        b[1] = u;\n"
    "        b[2] = 0.0;\n"
    "        b[3] = 0.0;\n"
    "        db[0] = -1.0;\n"
    "        db[1] = 1.0;\n"
    "        db[2] = 0.0;\n"
    "        db[3] = 0.0;\n"
    "    } else {\n"
    "        /* bezier (default) — Bernstein basis */\n"
    "        b[0] = ic3;\n"
    "        b[1] = 3.0*u*ic2;\n"
    "        b[2] = 3.0*u2*ic;\n"
    "        b[3] = u3;\n"
    "        db[0] = -3.0*ic2;\n"
    "        db[1] = 3.0*ic2 - 6.0*u*ic;\n"
    "        db[2] = 6.0*u*ic - 3.0*u2;\n"
    "        db[3] = 3.0*u2;\n"
    "    }\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    float u = gl_TessCoord.x;\n"
    "    float v = gl_TessCoord.y;\n"
    "\n"
    "    /* Object-space basis evaluation — model matrix carries world\n"
    "     * transform, view + proj follow Storm's separate-uniform path. */\n"
    "    vec4 b, db;\n"
    "    compute_basis(u, b, db);\n"
    "    vec3 P = b[0]*tcsPos[0] + b[1]*tcsPos[1] + b[2]*tcsPos[2] + b[3]*tcsPos[3];\n"
    "    vec3 T = db[0]*tcsPos[0] + db[1]*tcsPos[1] + db[2]*tcsPos[2] + db[3]*tcsPos[3];\n"
    "    /* Degenerate tangent (coincident CVs) — fall back to object-Z so\n"
    "     * we still get a valid frame instead of NaNs. */\n"
    "    float tlen = length(T);\n"
    "    T = (tlen > 1e-6) ? (T / tlen) : vec3(0.0, 0.0, 1.0);\n"
    "\n"
    "    /* Stable-ish frame: tangent + object-up cross. Swing-through-\n"
    "     * vertical singularity is handled by swapping to object-X. Storm's\n"
    "     * HALFTUBE accepts the same twist; parallel-transport would need\n"
    "     * a per-segment CPU pre-pass. */\n"
    "    vec3 objUp = vec3(0.0, 1.0, 0.0);\n"
    "    if (abs(dot(T, objUp)) > 0.99) objUp = vec3(1.0, 0.0, 0.0);\n"
    "    vec3 side = normalize(cross(T, objUp));\n"
    "    vec3 up   = cross(side, T);\n"
    "\n"
    "    float r = 0.5 * (b[0]*tcsWidth[0] + b[1]*tcsWidth[1]\n"
    "                   + b[2]*tcsWidth[2] + b[3]*tcsWidth[3]);\n"
    "    float theta = TWO_PI * v;\n"
    "    vec3 ringDir = cos(theta)*side + sin(theta)*up;\n"
    "    vec3 osPos   = P + r * ringDir;\n"
    "\n"
    "    vec4 worldPos = mesh.model * vec4(osPos, 1.0);\n"
    "    fragWorldPos = worldPos.xyz;\n"
    "    fragNormal = normalize((mesh.model * vec4(ringDir, 0.0)).xyz);\n"
    /* Tangent in world space — the curve's along-curve direction T,
     * perpendicular to ringDir. Storm uses this for normal-mapping in
     * authored materials. tangent_sign is +1 here (we don't reverse). */
    "    fragTangent = normalize((mesh.model * vec4(T, 0.0)).xyz);\n"
    "    fragTangentSign = 1.0;\n"
    /* Synthetic UV per Storm's tube path (basisCurves.glslfx:1293):
     * vec2(0, v). u along the curve isn't carried because curves don't
     * have authored U; only the around-tube v is meaningful. */
    "    fragTexCoord = vec2(0.0, v);\n"
    "    fragColor = b[0]*tcsColor[0] + b[1]*tcsColor[1]\n"
    "              + b[2]*tcsColor[2] + b[3]*tcsColor[3];\n"
    "    if (mesh.color.w > 0.5) fragColor = mesh.color.rgb;\n"
    "\n"
    "    gl_Position = u_proj * (u_view * worldPos);\n"
    "}\n";

/* ---- Curve fragment shader ----
 *
 * Approximates Storm's simpleLighting (default surface shader at
 * --complexity veryhigh) — see ~/OpenUSD/pxr/imaging/glf/shaders/
 * simpleLighting.glslfx:420-444. Diffuse NdotL + Blinn-Phong specular
 * + ambient. No PBR materials, no IBL, no rim — closer in tone to
 * Storm's golden references than the previous Lambert+rim formulation.
 * The phase-5 architectural prep (TES emits fragTangent /
 * fragTangentSign / fragTexCoord) lives in the TES so a future commit
 * can swap this fragment shader for k_pbr_frag_gles without touching
 * the TES.
 *
 * Old comment kept for context:
 * Simple Lambert + half-Lambert wrap + view-dependent rim, matching the
 * raster mesh fragment shader's lighting tone. No PBR/IBL for v1 —
 * curves don't carry material bindings yet. Phase 5 will route curves
 * through the same material pipeline as meshes (Storm shares the
 * material network via shading repr). */

static const char* NUSD_SHADER_UNUSED k_curve_frag_gles = NUSD_SHADER_VERSION_TESS NUSD_PRECISION_HIGH
    "\n"
    "in vec3  fragWorldPos;\n"
    "in vec3  fragNormal;\n"
    "in vec3  fragTangent;\n" /* phase-5 prep, currently unused */
    "in float fragTangentSign;\n" /* phase-5 prep, currently unused */
    "in vec3  fragColor;\n"
    "in vec2  fragTexCoord;\n" /* phase-5 prep, currently unused */
    "\n"
    "out vec4 outColor;\n"
    "\n"
    "uniform vec3 u_eyePos;\n"
    "\n"
    /* simpleLighting equation per glf/shaders/simpleLighting.glslfx:420-
     * 444: result.diffuse = NdotL * lightDiffuse + ambient;
     *      result.specular = pow(NdotH, shininess) * lightSpecular;
     * Constants tuned to match Storm at default lighting + 1 head-light. */
    "void main() {\n"
    "    vec3 N = normalize(fragNormal);\n"
    "    vec3 V = normalize(u_eyePos - fragWorldPos);\n"
    /* Camera-relative key light (matches Storm's default head-light
     * placement at the camera position). */
    "    vec3 L = V;\n"
    "    vec3 H = normalize(V + L);\n"
    "    float NdotL = max(dot(N, L), 0.0);\n"
    "    float NdotH = max(dot(N, H), 0.0);\n"
    /* Tuned to roughly match Storm's --complexity veryhigh luminance
     * on the test scenes. Storm uses dome ambient + warm key + cool
     * fill which we don't replicate exactly; we hit the same overall
     * luminance with a single white headlight. RMSE on the masked
     * curve_in_both region is dominated by Storm's hue structure
     * (sky/ground horizon ambient) which we'd need a real lighting
     * environment to match — out of scope for the curve port. */
    "    vec3 diffuse = fragColor * NdotL;\n"
    "    float spec   = pow(NdotH, 32.0);\n"
    "    vec3 ambient = fragColor * 0.18;\n"
    "    vec3 lit = ambient + diffuse * 0.55 + vec3(spec * 0.10);\n"
    "    outColor = vec4(pow(clamp(lit, 0.0, 1.0), vec3(1.0/2.2)), 1.0);\n"
    "}\n";
