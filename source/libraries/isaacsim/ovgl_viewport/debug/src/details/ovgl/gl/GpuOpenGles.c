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

/*
 * GpuOpenGles.c — OpenGL ES 3.2 implementation of gpu.h
 *
 * Lightweight GPU backend for the OpenGL renderer.
 * No ray tracing, no DLSS, no SSBO for materials (uses UBO per-draw).
 * Targets low GPU memory machines.
 */

#include "Font8x16.h"
#include "GlApi.h"
#include "GlLog.h"
#include "Gpu.h"
#include "ShadersGles.h"

#include <math.h>
#include <stb_image.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Internal types ---- */

#define MAX_OVERLAY_CHARS 4096
#define OVERLAY_VERTS_PER_CHAR 6
#define OVERLAY_FLOATS_PER_VERT 8 /* x, y, u, v, r, g, b, a */

/* OpenGL ES requires glUniformMatrix*'s transpose argument to be GL_FALSE.
 * The renderer's public/internal math convention is row-major, so convert to
 * the column-major byte layout GL expects instead of relying on desktop GL's
 * permitted transpose=GL_TRUE behavior. */
static void mat4_row_major_to_gl(float dst[16], const float src[16])
{
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            dst[col * 4 + row] = src[row * 4 + col];
}

struct GpuBuffer_s
{
    GLuint gl_buf;
    GLenum target; /* GL_ARRAY_BUFFER or GL_ELEMENT_ARRAY_BUFFER */
    uint64_t size;
};

struct GpuPipeline_s
{
    GLuint program;
    GLuint vao;
    GLint loc_mvp;
    GLint loc_model;
    GLint loc_color;
    GLint loc_screen_size; /* overlay only */
    GLint loc_eyePos;
    GLint loc_u_view; /* curve TES (Storm-style separate matrices) */
    GLint loc_u_proj; /* curve TES (Storm-style separate matrices) */
    GLint loc_u_basis_id; /* curve TES per-curve basis selector */
    /* Material-pipeline uniforms cached at create time so per-mesh
     * `gpu_cmd_bind_material` doesn't `glGetUniformLocation` 12+ times
     * per call (was 85% of frame time on the warehouse). -1 = not present. */
    GLint loc_tex_diffuse;
    GLint loc_tex_normal;
    GLint loc_tex_roughness;
    GLint loc_tex_metallic;
    GLint loc_tex_emissive;
    GLint loc_tex_occlusion;
    GLint loc_tex_opacity;
    GLint loc_u_envMap;
    GLint loc_u_irrMap;
    GLint loc_u_brdfLUT;
    GLint loc_u_envMipLevels;
    GLint loc_u_hasIBL;
    GLint loc_u_fallbackLighting;
    GLint loc_u_envOwnsLighting;
    GLint loc_u_envRotation;
    GLint loc_u_debugMode;
    GLint loc_u_tone;
    GLint loc_u_envIntensity;
    GLint loc_u_sceneRadius;
    GLint loc_u_authoredLightCount;
    GLint loc_u_sceneLightCount;
    GLint loc_u_sceneLightPosIntensity;
    GLint loc_u_sceneLightNormalKind;
    GLint loc_u_sceneLightColorNormalize;
    GLint loc_u_sceneLightUAxisAngle;
    GLint loc_u_sceneLightVAxis;
    GLint loc_u_shadowAtlas;
    GLint loc_u_shadowParams;
    GLint loc_u_shadowLightToSlot;
    GLint loc_u_shadowPassVP; /* depth-only shadow pipeline's pass VP */
    GLint loc_u_ssaoTex;
    GLint loc_u_aoStrength;
    GLint loc_u_invRenderSize;
    int samplers_assigned; /* sampler unit assignments done once */
    uint32_t vertex_stride;
    uint32_t nattribs;
    GpuVertexAttrib attribs[8];
    /* Tessellation: 0 patch_vertices means non-tess (primitive=GL_TRIANGLES).
     * Set when tcs_glsl + tes_glsl were provided to gpu_create_pipeline. */
    GLenum primitive;
    GLint patch_vertices;
};

struct Gpu
{
    void* window; /* GLFWwindow* */
    int width, height;

    /* Currently bound state */
    GpuPipeline current_pipeline;
    GpuBuffer current_vb;
    GpuBuffer current_ib;

    /* Overlay state */
    GLuint overlay_font_tex;
    struct GpuPipeline_s overlay_pipeline;
    GLuint overlay_vbo;
    float overlay_verts[MAX_OVERLAY_CHARS * OVERLAY_VERTS_PER_CHAR * OVERLAY_FLOATS_PER_VERT];
    int overlay_nchars;

    /* Material state */
    GpuMaterialParams* mat_params;
    int mat_count;
    GLuint* mat_textures;
    int mat_tex_count;
    GLuint mat_ubo;
    GLuint mat_dummy_tex; /* 1x1 white */
    GLuint last_bound_tex[10]; /* 0..5 PBR slots, 6=opacity (cache to skip redundant binds) */
    int last_bound_material; /* -1 = none; cache to skip redundant UBO upload + texture rebind */
    int mat_stride; /* per-material UBO slice size, padded to UNIFORM_BUFFER_OFFSET_ALIGNMENT */
    GLuint ptex_color_ssbo;
    uint32_t ptex_color_count;

    /* Per-mesh data UBO — see gpu.h::GpuMeshData. One slot per mesh,
     * stride padded to UNIFORM_BUFFER_OFFSET_ALIGNMENT so each slot is
     * a valid bind range. We re-fill the whole UBO each frame via
     * glMapBufferRange + INVALIDATE_BUFFER + UNSYNCHRONIZED so per-mesh
     * transform/color edits don't stall on in-flight GPU reads. */
    GLuint mesh_ubo;
    int mesh_stride;
    int mesh_count;

    /* Per-instance world-matrix VBO (16 floats each) for compact-PI batches. */
    GLuint instance_vbo;
    uint32_t instance_count;

    /* IBL environment */
    GLuint env_map; /* pre-filtered HDR lat-long, with mip chain */
    GLuint irr_map; /* SH irradiance map (cosine-convolved) */
    GLuint brdf_lut; /* 2D BRDF integration LUT (RG16F) */
    int env_mip_levels; /* number of mip levels in env_map */
    int has_ibl;
    float env_intensity; /* signed; < -1 is an internal visible-dome fallback marker */
    int fallback_lighting;
    /* The loaded environment is bright enough to be the frame's light
     * source. Distinct from fallback_lighting, which the driver clears the
     * moment ANY light is uploaded -- including its own synthetic
     * headlight -- and so cannot answer "does the environment own this
     * frame". See gpu_set_environment_owns_lighting. */
    int env_owns_lighting;
    float env_rotation_deg; /* UsdLuxDomeLight inputs:rotation, degrees */
    GpuLight scene_lights[GPU_MAX_SCENE_LIGHTS];
    int scene_light_count;
    int authored_light_count;
    /* Shared depth atlas (gpu.h GPU_SHADOW_*). One DEPTH_COMPONENT24 texture
     * tiled per shadow-casting light, one std140 ShadowBlock UBO holding the
     * per-tile records, and a scene-light -> slot table so the fragment shader
     * resolves a light index to a tile in O(1) instead of comparing against a
     * fixed pair of light indices. */
    GLuint shadow_fbo;
    GLint shadow_saved_fbo; /* FB bound before the shadow pass, restored after */
    GLuint shadow_atlas_tex;
    GLuint shadow_dummy_tex; /* 1x1 depth+compare, bound when count == 0 */
    GLuint shadow_atlas_ubo;
    int shadow_atlas_size; /* current allocation, <= GPU_SHADOW_ATLAS_MAX */
    int shadow_tile_size;
    int shadow_tiles_per_row;
    int shadow_count;
    int shadow_active_slot;
    int shadow_light_to_slot[GPU_MAX_SCENE_LIGHTS]; /* -1 = casts no shadow */
    GpuShadowLight shadow_lights[GPU_MAX_SHADOW_LIGHTS];

    /* Environment background */
    GLuint env_bg_program; /* fullscreen env map shader */
    GLuint env_bg_vao; /* empty VAO for fullscreen triangle */

    /* Depth-AOV pack pass (k_depth_pack_*): samples a DEPTH_COMPONENT
     * texture and packs window-space depth into an RGBA8 target for a
     * GLES-portable readback (GLES has no glReadPixels(GL_DEPTH_COMPONENT)).
     * Created lazily by gpu_depth_pack_read; target re-allocated on resize. */
    GLuint depth_pack_program;
    GLuint depth_pack_vao;
    GLuint depth_pack_fbo;
    GLuint depth_pack_tex;
    int depth_pack_w, depth_pack_h;

    /* SSAO (k_ssao_*): two fullscreen passes over the AO prepass, computed at
     * HALF the internal resolution and blurred back over its own 4x4 rotation
     * tile. Created lazily by gpu_ssao_compute; targets re-allocated on resize.
     * ssao_tex holds the raw estimate, ssao_blur_tex the one the material pass
     * samples -- two targets rather than one because the blur reads every
     * neighbour of the pixel it writes. */
    GLuint ssao_program;
    GLuint ssao_blur_program;
    GLuint ssao_vao;
    GLuint ssao_fbo, ssao_tex;
    GLuint ssao_blur_fbo, ssao_blur_tex;
    int ssao_w, ssao_h;
    float ssao_kernel[GPU_SSAO_KERNEL_SIZE * 3];
    int ssao_kernel_built;
    /* What the material pass binds on unit 11, and how hard it applies it.
     * strength 0 makes the shader take its `ao` untouched. */
    GLuint ssao_bound_tex;
    float ssao_strength;

    /* Memory tracking */
    uint64_t allocated_bytes;

    /* Debug */
    int debug_mode;

    /* Bounding radius of the scene, in world units.  The no-IBL haze fades geometry
     * toward the backdrop and is expressed relative to this, so it behaves the same way
     * whatever units the asset is authored in.  kHazeReferenceRadius keeps a radius-8
     * scene on exactly the curve the haze constants were originally tuned for. */
    float scene_radius;

    float tone_exposure_scale;
    float tone_sky_scale;
    float tone_white_point;
    uint32_t tone_flags;
};

/* ---- Shader compilation ---- */

static GLuint compile_shader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (!status)
    {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        gl_log(GL_LOG_ERROR, "gpu_gles", "shader compile error:\n%s", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static GLuint link_program(GLuint vert, GLuint frag)
{
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);

    GLint status;
    glGetProgramiv(prog, GL_LINK_STATUS, &status);
    if (!status)
    {
        char log[2048];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        gl_log(GL_LOG_ERROR, "gpu_gles", "program link error:\n%s", log);
        glDeleteProgram(prog);
        glDeleteShader(vert);
        glDeleteShader(frag);
        return 0;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    return prog;
}

/* ---- Lifecycle ---- */

Gpu* gpu_init(void* glfw_window, int width, int height)
{
#ifdef _WIN32
    if (!ovgl_gl_load())
        return NULL;
#endif
    Gpu* gpu = (Gpu*)calloc(1, sizeof(Gpu));
    if (!gpu)
        return NULL;

    gpu->window = glfw_window;
    gpu->width = width;
    gpu->height = height;

    /* NU_OPENGL_DEBUG_MODE=N picks a debug visualization. Search
     * shaders_gles.h for `u_debugMode ==` for the authoritative list —
     * 1=baseColor, 2=normal, 3=roughness, 4=metallic, 5=ambient, 6=Lo,
     * 7=specularIBL, 8=diffuseIBL, 9..15 IBL internals, 16=UV, 17=UV checker,
     * and the shadow-atlas set: 24=resolved visibility for scene light 0,
     * 25=its atlas slot (red = none), 26=slot 0's hardware compare at this
     * receiver, 27=the receiver's own window depth in slot 0's frustum.
     * 0 = production lighting (default). */
    const char* dbg_env = getenv("NU_OPENGL_DEBUG_MODE");
    if (dbg_env && *dbg_env)
    {
        gpu->debug_mode = atoi(dbg_env);
    }
    /* calloc leaves the slot table at 0, which reads as "scene light 0 casts
     * from tile 0". The shader gates on shadow count, but leaving a live-
     * looking mapping in a struct that outlives many frames is a trap. */
    for (int i = 0; i < GPU_MAX_SCENE_LIGHTS; i++)
        gpu->shadow_light_to_slot[i] = -1;

    /* Log GL info. glGetString returning NULL right after a successful
     * eglMakeCurrent means the process mixed two glvnd copies (e.g. a staged
     * conda libGL/libGLX shadowing the system stack) and GL dispatch is not
     * bound to this context; fail loudly instead of passing NULL to %s
     * (glibc FORTIFY aborts inside __fprintf_chk on NULL strings). */
    const GLubyte* gl_version = glGetString(GL_VERSION);
    const GLubyte* gl_renderer = glGetString(GL_RENDERER);
    if (!gl_version || !gl_renderer)
    {
        gl_log(GL_LOG_ERROR, "gpu_gles",
               "glGetString returned NULL after makeCurrent -- split glvnd "
               "dispatch? Ensure exactly one libEGL/libGL/libGLES provider "
               "(the host driver stack) is on the loader path.");
        free(gpu);
        return NULL;
    }
    gl_log(GL_LOG_INFO, "gpu_gles", "GL_VERSION  = %s", gl_version);
    gl_log(GL_LOG_INFO, "gpu_gles", "GL_RENDERER = %s", gl_renderer);
    if (gpu->debug_mode != 0)
    {
        gl_log(GL_LOG_VERBOSE, "gpu_gles", "NU_OPENGL_DEBUG_MODE=%d", gpu->debug_mode);
    }
    gpu->scene_radius = kHazeReferenceRadius;
    gpu->tone_exposure_scale = 1.0f;
    gpu->tone_sky_scale = 1.0f;
    gpu->tone_white_point = 1.0f;
    gpu->tone_flags = 0u;
    gpu->env_intensity = 0.0f;

    /* Default state */
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    /* No backface culling, matching the vulkan viewer's
     * VK_CULL_MODE_NONE. Authored asset winding is inconsistent across
     * sources (Pixar/Lightwheel quads, glTF-converted USDs, MaterialX
     * test rigs) and culling here was hiding the colored quads on
     * TextureCoordinateTest where winding doesn't agree with our
     * expected CW front-face. CULL_FACE was a perf optimization for
     * the warehouse-scale Apple build; toggle this back when needed
     * via a renderer config rather than as a hardcoded default. */
    glDisable(GL_CULL_FACE);
    glFrontFace(GL_CW);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); /* Black no-IBL backdrop, matching official ovrtx 0.4 */
    glViewport(0, 0, width, height);

    return gpu;
}

void gpu_shutdown(Gpu* gpu)
{
    if (!gpu)
        return;
    /* Keep shutdown complete for every embedding, not only ovgl's current
     * resource subset. All of these functions are idempotent after an
     * explicit higher-level teardown. */
    gpu_overlay_shutdown(gpu);
    gpu_destroy_environment(gpu);
    gpu_destroy_materials(gpu);
    if (gpu->instance_vbo)
        glDeleteBuffers(1, &gpu->instance_vbo);
    if (gpu->shadow_atlas_tex)
        glDeleteTextures(1, &gpu->shadow_atlas_tex);
    if (gpu->shadow_dummy_tex)
        glDeleteTextures(1, &gpu->shadow_dummy_tex);
    if (gpu->shadow_atlas_ubo)
        glDeleteBuffers(1, &gpu->shadow_atlas_ubo);
    if (gpu->shadow_fbo)
        glDeleteFramebuffers(1, &gpu->shadow_fbo);
    if (gpu->depth_pack_program)
        glDeleteProgram(gpu->depth_pack_program);
    if (gpu->depth_pack_vao)
        glDeleteVertexArrays(1, &gpu->depth_pack_vao);
    if (gpu->depth_pack_tex)
        glDeleteTextures(1, &gpu->depth_pack_tex);
    if (gpu->depth_pack_fbo)
        glDeleteFramebuffers(1, &gpu->depth_pack_fbo);
    if (gpu->ssao_program)
        glDeleteProgram(gpu->ssao_program);
    if (gpu->ssao_blur_program)
        glDeleteProgram(gpu->ssao_blur_program);
    if (gpu->ssao_vao)
        glDeleteVertexArrays(1, &gpu->ssao_vao);
    if (gpu->ssao_tex)
        glDeleteTextures(1, &gpu->ssao_tex);
    if (gpu->ssao_blur_tex)
        glDeleteTextures(1, &gpu->ssao_blur_tex);
    if (gpu->ssao_fbo)
        glDeleteFramebuffers(1, &gpu->ssao_fbo);
    if (gpu->ssao_blur_fbo)
        glDeleteFramebuffers(1, &gpu->ssao_blur_fbo);
    free(gpu);
}

void gpu_resize(Gpu* gpu, int width, int height)
{
    if (!gpu)
        return;
    gpu->width = width;
    gpu->height = height;
    glViewport(0, 0, width, height);
}

/* ---- Resources ---- */

unsigned gpu_buffer_gl_handle(GpuBuffer buf)
{
    return buf ? (unsigned)buf->gl_buf : 0u;
}

GpuBuffer gpu_create_buffer(Gpu* gpu, const GpuBufferDesc* desc)
{
    if (!gpu || !desc)
        return NULL;

    GpuBuffer buf = (GpuBuffer)calloc(1, sizeof(struct GpuBuffer_s));
    if (!buf)
        return NULL;

    buf->target = (desc->usage == GPU_BUFFER_INDEX) ? GL_ELEMENT_ARRAY_BUFFER : GL_ARRAY_BUFFER;
    buf->size = desc->size;

    glGenBuffers(1, &buf->gl_buf);
    glBindBuffer(buf->target, buf->gl_buf);
    /* Avoid SIGBUS in NVIDIA 595.58.03's aarch64 buffer upload path. */
    glBufferData(buf->target, (GLsizeiptr)desc->size, NULL, GL_STATIC_DRAW);
    if (desc->data && desc->size > 0)
    {
        void* mapped =
            glMapBufferRange(buf->target, 0, (GLsizeiptr)desc->size, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
        if (!mapped)
        {
            glBindBuffer(buf->target, 0);
            glDeleteBuffers(1, &buf->gl_buf);
            free(buf);
            return NULL;
        }
        memcpy(mapped, desc->data, (size_t)desc->size);
        if (!glUnmapBuffer(buf->target))
        {
            glBindBuffer(buf->target, 0);
            glDeleteBuffers(1, &buf->gl_buf);
            free(buf);
            return NULL;
        }
    }
    glBindBuffer(buf->target, 0);

    gpu->allocated_bytes += desc->size;

    return buf;
}

void gpu_destroy_buffer(Gpu* gpu, GpuBuffer buf)
{
    if (!gpu || !buf)
        return;
    if (gpu->allocated_bytes >= buf->size)
        gpu->allocated_bytes -= buf->size;
    glDeleteBuffers(1, &buf->gl_buf);
    free(buf);
}

/* ---- Pipeline ---- */

GpuPipeline gpu_create_pipeline(Gpu* gpu, const GpuPipelineDesc* desc)
{
    if (!gpu || !desc || !desc->vert_glsl || !desc->frag_glsl)
        return NULL;

    /* TCS and TES are paired — either both or neither. */
    int has_tess = desc->tcs_glsl && desc->tes_glsl;
    if ((desc->tcs_glsl != NULL) ^ (desc->tes_glsl != NULL))
    {
        gl_log(GL_LOG_ERROR, "gpu_gles", "tcs_glsl and tes_glsl must both be set or both NULL");
        return NULL;
    }
    if (has_tess && desc->patch_vertices == 0)
    {
        gl_log(GL_LOG_ERROR, "gpu_gles", "tess pipeline requires patch_vertices > 0");
        return NULL;
    }

    GLuint vert = compile_shader(GL_VERTEX_SHADER, desc->vert_glsl);
    if (!vert)
        return NULL;

    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, desc->frag_glsl);
    if (!frag)
    {
        glDeleteShader(vert);
        return NULL;
    }

    GLuint tcs = 0, tes = 0;
    if (has_tess)
    {
        tcs = compile_shader(GL_TESS_CONTROL_SHADER, desc->tcs_glsl);
        if (!tcs)
        {
            glDeleteShader(vert);
            glDeleteShader(frag);
            return NULL;
        }
        tes = compile_shader(GL_TESS_EVALUATION_SHADER, desc->tes_glsl);
        if (!tes)
        {
            glDeleteShader(vert);
            glDeleteShader(frag);
            glDeleteShader(tcs);
            return NULL;
        }
    }

    GLuint prog;
    if (!has_tess)
    {
        prog = link_program(vert, frag);
        if (!prog)
            return NULL;
    }
    else
    {
        /* Inline link: link_program() only handles vert+frag and frees them.
         * For tess we attach all four, link, then detach + delete. */
        prog = glCreateProgram();
        glAttachShader(prog, vert);
        glAttachShader(prog, tcs);
        glAttachShader(prog, tes);
        glAttachShader(prog, frag);
        glLinkProgram(prog);
        GLint status;
        glGetProgramiv(prog, GL_LINK_STATUS, &status);
        if (!status)
        {
            char log[2048];
            glGetProgramInfoLog(prog, sizeof(log), NULL, log);
            gl_log(GL_LOG_ERROR, "gpu_gles", "tess program link error:\n%s", log);
            glDeleteProgram(prog);
            glDeleteShader(vert);
            glDeleteShader(tcs);
            glDeleteShader(tes);
            glDeleteShader(frag);
            return NULL;
        }
        glDeleteShader(vert);
        glDeleteShader(tcs);
        glDeleteShader(tes);
        glDeleteShader(frag);
    }

    GpuPipeline pipe = (GpuPipeline)calloc(1, sizeof(struct GpuPipeline_s));
    if (!pipe)
    {
        glDeleteProgram(prog);
        return NULL;
    }

    pipe->program = prog;
    pipe->primitive = has_tess ? GL_PATCHES : GL_TRIANGLES;
    pipe->patch_vertices = has_tess ? (GLint)desc->patch_vertices : 0;
    pipe->loc_mvp = glGetUniformLocation(prog, "u_mvp");
    pipe->loc_model = glGetUniformLocation(prog, "u_model");
    pipe->loc_color = glGetUniformLocation(prog, "u_color");
    pipe->loc_screen_size = glGetUniformLocation(prog, "u_screen_size");
    pipe->loc_eyePos = glGetUniformLocation(prog, "u_eyePos");
    pipe->loc_u_view = glGetUniformLocation(prog, "u_view");
    pipe->loc_u_proj = glGetUniformLocation(prog, "u_proj");
    pipe->loc_u_basis_id = glGetUniformLocation(prog, "u_basis_id");
    /* Cache material/IBL uniform locations once. -1 if not present. */
    pipe->loc_tex_diffuse = glGetUniformLocation(prog, "tex_diffuse");
    pipe->loc_tex_normal = glGetUniformLocation(prog, "tex_normal");
    pipe->loc_tex_roughness = glGetUniformLocation(prog, "tex_roughness");
    pipe->loc_tex_metallic = glGetUniformLocation(prog, "tex_metallic");
    pipe->loc_tex_emissive = glGetUniformLocation(prog, "tex_emissive");
    pipe->loc_tex_occlusion = glGetUniformLocation(prog, "tex_occlusion");
    pipe->loc_tex_opacity = glGetUniformLocation(prog, "tex_opacity");
    pipe->loc_u_envMap = glGetUniformLocation(prog, "u_envMap");
    pipe->loc_u_irrMap = glGetUniformLocation(prog, "u_irrMap");
    pipe->loc_u_brdfLUT = glGetUniformLocation(prog, "u_brdfLUT");
    pipe->loc_u_envMipLevels = glGetUniformLocation(prog, "u_envMipLevels");
    pipe->loc_u_hasIBL = glGetUniformLocation(prog, "u_hasIBL");
    pipe->loc_u_fallbackLighting = glGetUniformLocation(prog, "u_fallbackLighting");
    pipe->loc_u_envOwnsLighting = glGetUniformLocation(prog, "u_envOwnsLighting");
    pipe->loc_u_envRotation = glGetUniformLocation(prog, "u_envRotation");
    pipe->loc_u_debugMode = glGetUniformLocation(prog, "u_debugMode");
    pipe->loc_u_tone = glGetUniformLocation(prog, "u_tone");
    pipe->loc_u_envIntensity = glGetUniformLocation(prog, "u_envIntensity");
    pipe->loc_u_sceneRadius = glGetUniformLocation(prog, "u_sceneRadius");
    pipe->loc_u_authoredLightCount = glGetUniformLocation(prog, "u_authoredLightCount");
    pipe->loc_u_sceneLightCount = glGetUniformLocation(prog, "u_sceneLightCount");
    pipe->loc_u_sceneLightPosIntensity = glGetUniformLocation(prog, "u_sceneLightPosIntensity[0]");
    pipe->loc_u_sceneLightNormalKind = glGetUniformLocation(prog, "u_sceneLightNormalKind[0]");
    pipe->loc_u_sceneLightColorNormalize = glGetUniformLocation(prog, "u_sceneLightColorNormalize[0]");
    pipe->loc_u_sceneLightUAxisAngle = glGetUniformLocation(prog, "u_sceneLightUAxisAngle[0]");
    pipe->loc_u_sceneLightVAxis = glGetUniformLocation(prog, "u_sceneLightVAxis[0]");
    pipe->loc_u_shadowAtlas = glGetUniformLocation(prog, "u_shadowAtlas");
    pipe->loc_u_shadowParams = glGetUniformLocation(prog, "u_shadowParams");
    pipe->loc_u_shadowLightToSlot = glGetUniformLocation(prog, "u_shadowLightToSlot[0]");
    pipe->loc_u_shadowPassVP = glGetUniformLocation(prog, "u_shadowPassVP");
    pipe->loc_u_ssaoTex = glGetUniformLocation(prog, "u_ssaoTex");
    pipe->loc_u_aoStrength = glGetUniformLocation(prog, "u_aoStrength");
    pipe->loc_u_invRenderSize = glGetUniformLocation(prog, "u_invRenderSize");
    pipe->samplers_assigned = 0;
    pipe->vertex_stride = desc->vertex_stride;
    pipe->nattribs = desc->nattribs;
    if (desc->nattribs <= 8)
        memcpy(pipe->attribs, desc->attribs, desc->nattribs * sizeof(GpuVertexAttrib));

    glGenVertexArrays(1, &pipe->vao);

#ifdef NUSD_DESKTOP_GL
    /* Both basic and PBR pipelines have a MeshBlock at binding=1 holding
     * per-mesh mvp/model/color. Desktop GLSL 4.1 needs the binding set
     * from C via glUniformBlockBinding. Pipelines that don't have the
     * block (overlay, env background) just hit GL_INVALID_INDEX and
     * are skipped. */
    if (pipe)
    {
        GLuint mesh_idx = glGetUniformBlockIndex(pipe->program, "MeshBlock");
        if (mesh_idx != GL_INVALID_INDEX)
            glUniformBlockBinding(pipe->program, mesh_idx, 1);
        /* Same reason, binding point 2: the shadow atlas's ShadowBlock. */
        GLuint shadow_idx = glGetUniformBlockIndex(pipe->program, "ShadowBlock");
        if (shadow_idx != GL_INVALID_INDEX)
            glUniformBlockBinding(pipe->program, shadow_idx, 2);
    }
#endif

    return pipe;
}

GpuPipeline gpu_create_material_pipeline(Gpu* gpu, const GpuPipelineDesc* desc)
{
    GpuPipeline pipe = gpu_create_pipeline(gpu, desc);
#ifdef NUSD_DESKTOP_GL
    /* Desktop GLSL 4.1 doesn't support layout(binding=N) in shaders,
     * so bind the MaterialBlock UBO to binding point 0 from the C side. */
    if (pipe)
    {
        GLuint idx = glGetUniformBlockIndex(pipe->program, "MaterialBlock");
        if (idx != GL_INVALID_INDEX)
            glUniformBlockBinding(pipe->program, idx, 0);
    }
#endif
    return pipe;
}

void gpu_destroy_pipeline(Gpu* gpu, GpuPipeline pipe)
{
    if (!gpu || !pipe)
        return;
    glDeleteVertexArrays(1, &pipe->vao);
    glDeleteProgram(pipe->program);
    free(pipe);
}

/* ---- Frame ---- */

int gpu_begin_frame(Gpu* gpu)
{
    if (!gpu)
        return 0;
    glViewport(0, 0, gpu->width, gpu->height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    /* Frame-fresh: forget last-bound textures + material so the
     * env-background pass doesn't poison the cache for the per-mesh
     * material loop. */
    for (int i = 0; i < 10; i++)
        gpu->last_bound_tex[i] = 0;
    gpu->last_bound_material = -1;
    return 1;
}

void gpu_end_frame(Gpu* gpu)
{
    if (!gpu)
        return;
    gpu_overlay_flush(gpu, gpu->width, gpu->height);
    /* Swap is done by GLFW in the viewer */
}

/* ---- Draw commands ---- */

static void gpu_apply_light_uniforms(Gpu* gpu, GpuPipeline pipe)
{
    if (!gpu || !pipe)
        return;
    if (pipe->loc_u_authoredLightCount >= 0)
        glUniform1i(pipe->loc_u_authoredLightCount, gpu->authored_light_count);
    if (pipe->loc_u_sceneLightCount >= 0)
        glUniform1i(pipe->loc_u_sceneLightCount, gpu->scene_light_count);
    if (gpu->scene_light_count <= 0)
        return;
    if (pipe->loc_u_sceneLightPosIntensity < 0 || pipe->loc_u_sceneLightNormalKind < 0 ||
        pipe->loc_u_sceneLightColorNormalize < 0 || pipe->loc_u_sceneLightUAxisAngle < 0 ||
        pipe->loc_u_sceneLightVAxis < 0)
    {
        return;
    }

    float pos_int[GPU_MAX_SCENE_LIGHTS][4] = { { 0 } };
    float norm_kind[GPU_MAX_SCENE_LIGHTS][4] = { { 0 } };
    float color_norm[GPU_MAX_SCENE_LIGHTS][4] = { { 0 } };
    float u_angle[GPU_MAX_SCENE_LIGHTS][4] = { { 0 } };
    float v_axis[GPU_MAX_SCENE_LIGHTS][4] = { { 0 } };
    int n = gpu->scene_light_count;
    if (n > GPU_MAX_SCENE_LIGHTS)
        n = GPU_MAX_SCENE_LIGHTS;

    for (int i = 0; i < n; i++)
    {
        const GpuLight* L = &gpu->scene_lights[i];
        pos_int[i][0] = L->position[0];
        pos_int[i][1] = L->position[1];
        pos_int[i][2] = L->position[2];
        pos_int[i][3] = L->intensity;
        norm_kind[i][0] = L->normal[0];
        norm_kind[i][1] = L->normal[1];
        norm_kind[i][2] = L->normal[2];
        norm_kind[i][3] = (float)L->kind;
        color_norm[i][0] = L->color[0];
        color_norm[i][1] = L->color[1];
        color_norm[i][2] = L->color[2];
        color_norm[i][3] = (float)L->normalize;
        u_angle[i][0] = L->u_axis[0];
        u_angle[i][1] = L->u_axis[1];
        u_angle[i][2] = L->u_axis[2];
        u_angle[i][3] = L->angle_deg;
        v_axis[i][0] = L->v_axis[0];
        v_axis[i][1] = L->v_axis[1];
        v_axis[i][2] = L->v_axis[2];
    }

    glUniform4fv(pipe->loc_u_sceneLightPosIntensity, n, &pos_int[0][0]);
    glUniform4fv(pipe->loc_u_sceneLightNormalKind, n, &norm_kind[0][0]);
    glUniform4fv(pipe->loc_u_sceneLightColorNormalize, n, &color_norm[0][0]);
    glUniform4fv(pipe->loc_u_sceneLightUAxisAngle, n, &u_angle[0][0]);
    glUniform4fv(pipe->loc_u_sceneLightVAxis, n, &v_axis[0][0]);
}

/* A sampler2DShadow needs a texture with a DEPTH format AND a compare mode.
 * The 1x1 RGBA8 mat_dummy_tex the pre-atlas path bound when count == 0 was
 * legal only while the shadow sampler was a plain sampler2D; binding it to a
 * shadow sampler is undefined behavior, so keep a dedicated one. */
static GLuint gpu_shadow_dummy_tex(Gpu* gpu)
{
    if (gpu->shadow_dummy_tex)
        return gpu->shadow_dummy_tex;
    GLuint tex = 0;
    glGenTextures(1, &tex);
    if (!tex)
        return 0;
    const GLuint one = 0xFFFFFFFFu; /* farthest depth: everything reads lit */
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 1, 1, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, &one);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    gpu->shadow_dummy_tex = tex;
    return tex;
}

static void gpu_apply_shadow_uniforms(Gpu* gpu, GpuPipeline pipe)
{
    if (!gpu || !pipe)
        return;
    int count = gpu->shadow_count;
    if (count < 0)
        count = 0;
    if (count > GPU_MAX_SHADOW_LIGHTS)
        count = GPU_MAX_SHADOW_LIGHTS;
    if (!gpu->shadow_atlas_tex)
        count = 0;
    /* .w is one ATLAS texel now, not one map texel: the shader scales it into
     * tile-UV by the tile's own uvScale, so this stays right at every atlas
     * size. .y is dead -- the bias is per-light in the ShadowBlock record. */
    float texel = (gpu->shadow_atlas_size > 0) ? 1.0f / (float)gpu->shadow_atlas_size : 0.0f;
    /* Shadow strength 1.0, not the pre-atlas 0.85: a fully occluded point
     * receives NO direct light from that source, which is what the vulkan
     * raster reference's binary ray-query visibility does and what official
     * ovrtx shows. The 0.15 floor was compensating for a hard-edged binary
     * compare that the hardware PCF above now softens properly. */
    if (pipe->loc_u_shadowParams >= 0)
        glUniform4f(pipe->loc_u_shadowParams, (float)count, 0.0f, 1.0f, texel);
    /* The table is authoritative on its own -- gpu_shadow_clear resets every
     * entry to -1 -- so it uploads unconditionally rather than being gated on
     * count, and a light whose caster was dropped stops sampling immediately. */
    if (pipe->loc_u_shadowLightToSlot >= 0)
        glUniform1iv(pipe->loc_u_shadowLightToSlot, GPU_MAX_SCENE_LIGHTS, gpu->shadow_light_to_slot);
    if (pipe->loc_u_shadowAtlas >= 0)
    {
        glUniform1i(pipe->loc_u_shadowAtlas, 10);
        glActiveTexture(GL_TEXTURE10);
        glBindTexture(GL_TEXTURE_2D, (count > 0) ? gpu->shadow_atlas_tex : gpu_shadow_dummy_tex(gpu));
    }
    if (count > 0 && gpu->shadow_atlas_ubo)
        glBindBufferBase(GL_UNIFORM_BUFFER, 2, gpu->shadow_atlas_ubo);
    /* The depth-only shadow pipeline's pass VP. Pushed on bind so a tile's
     * draws pick up whichever light gpu_shadow_begin made active, which is
     * what lets every tile reuse ONE MeshBlock upload (mesh.model) instead of
     * re-uploading light-space MVPs per light per frame. */
    if (pipe->loc_u_shadowPassVP >= 0)
    {
        int slot = gpu->shadow_active_slot;
        float vp_gl[16];
        if (slot >= 0 && slot < GPU_MAX_SHADOW_LIGHTS)
            mat4_row_major_to_gl(vp_gl, gpu->shadow_lights[slot].lightVP);
        else
            memset(vp_gl, 0, sizeof(vp_gl));
        glUniformMatrix4fv(pipe->loc_u_shadowPassVP, 1, GL_FALSE, vp_gl);
    }
}

/* Bind the AO buffer + its strength for the material pass.
 *
 * Strength is forced to 0 whenever there is no texture to sample, so a frame
 * where the SSAO pass did not run (disabled, or it failed to compile) takes
 * the `ao` branch in the shader untouched rather than multiplying by whatever
 * unit 11 happens to hold. A white 1x1 is bound in that case for the same
 * reason gpu_shadow_dummy_tex exists: some drivers fault on sampling an
 * unbound unit even when the result is dead. */
static void gpu_apply_ssao_uniforms(Gpu* gpu, GpuPipeline pipe)
{
    if (pipe->loc_u_invRenderSize >= 0)
    {
        glUniform2f(pipe->loc_u_invRenderSize, gpu->width > 0 ? 1.0f / (float)gpu->width : 0.0f,
                    gpu->height > 0 ? 1.0f / (float)gpu->height : 0.0f);
    }
    const int have = (gpu->ssao_bound_tex != 0);
    if (pipe->loc_u_aoStrength >= 0)
        glUniform1f(pipe->loc_u_aoStrength, have ? gpu->ssao_strength : 0.0f);
    if (pipe->loc_u_ssaoTex >= 0)
    {
        glUniform1i(pipe->loc_u_ssaoTex, 11);
        glActiveTexture(GL_TEXTURE11);
        glBindTexture(GL_TEXTURE_2D, have ? gpu->ssao_bound_tex : gpu_shadow_dummy_tex(gpu));
    }
}

void gpu_cmd_bind_pipeline(Gpu* gpu, GpuPipeline pipe)
{
    if (!gpu || !pipe)
        return;
    gpu->current_pipeline = pipe;
    glUseProgram(pipe->program);
    if (pipe->loc_u_tone >= 0)
    {
        glUniform4f(pipe->loc_u_tone, gpu->tone_exposure_scale, gpu->tone_sky_scale, gpu->tone_white_point,
                    (float)gpu->tone_flags);
    }
    if (pipe->loc_u_fallbackLighting >= 0)
        glUniform1i(pipe->loc_u_fallbackLighting, gpu->fallback_lighting ? 1 : 0);
    if (pipe->loc_u_envOwnsLighting >= 0)
        glUniform1i(pipe->loc_u_envOwnsLighting, gpu->env_owns_lighting ? 1 : 0);
    if (pipe->loc_u_envRotation >= 0)
        glUniform1f(pipe->loc_u_envRotation, gpu->env_rotation_deg);
    if (pipe->loc_u_envIntensity >= 0)
        glUniform1f(pipe->loc_u_envIntensity, gpu->env_intensity);
    if (pipe->loc_u_sceneRadius >= 0)
        glUniform1f(pipe->loc_u_sceneRadius, gpu->scene_radius);
    gpu_apply_light_uniforms(gpu, pipe);
    gpu_apply_shadow_uniforms(gpu, pipe);
    gpu_apply_ssao_uniforms(gpu, pipe);
    if (pipe->patch_vertices > 0)
        glPatchParameteri(GL_PATCH_VERTICES, pipe->patch_vertices);
}

void gpu_cmd_bind_vertex_buffer(Gpu* gpu, GpuBuffer buf)
{
    if (!gpu || !buf)
        return;
    gpu->current_vb = buf;

    GpuPipeline pipe = gpu->current_pipeline;
    if (!pipe)
        return;

    glBindVertexArray(pipe->vao);
    glBindBuffer(GL_ARRAY_BUFFER, buf->gl_buf);

    /* Set up vertex attributes */
    for (uint32_t i = 0; i < pipe->nattribs; i++)
    {
        glEnableVertexAttribArray(pipe->attribs[i].location);
        switch (pipe->attribs[i].format)
        {
        case GPU_FORMAT_FLOAT3:
            glVertexAttribPointer(pipe->attribs[i].location, 3, GL_FLOAT, GL_FALSE, (GLsizei)pipe->vertex_stride,
                                  (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        case GPU_FORMAT_FLOAT2:
            glVertexAttribPointer(pipe->attribs[i].location, 2, GL_FLOAT, GL_FALSE, (GLsizei)pipe->vertex_stride,
                                  (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        case GPU_FORMAT_UINT:
            glVertexAttribIPointer(pipe->attribs[i].location, 1, GL_UNSIGNED_INT, (GLsizei)pipe->vertex_stride,
                                   (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        case GPU_FORMAT_FLOAT1:
            glVertexAttribPointer(pipe->attribs[i].location, 1, GL_FLOAT, GL_FALSE, (GLsizei)pipe->vertex_stride,
                                  (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        case GPU_FORMAT_SNORM16X4:
            glVertexAttribPointer(pipe->attribs[i].location, 4, GL_SHORT, GL_TRUE, (GLsizei)pipe->vertex_stride,
                                  (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        case GPU_FORMAT_UNORM8X4:
            glVertexAttribPointer(pipe->attribs[i].location, 4, GL_UNSIGNED_BYTE, GL_TRUE, (GLsizei)pipe->vertex_stride,
                                  (void*)(uintptr_t)pipe->attribs[i].offset);
            break;
        }
    }
}

void gpu_cmd_bind_index_buffer(Gpu* gpu, GpuBuffer buf)
{
    if (!gpu || !buf)
        return;
    gpu->current_ib = buf;
    /* Bind IB to the current VAO */
    if (gpu->current_pipeline)
        glBindVertexArray(gpu->current_pipeline->vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf->gl_buf);
}

void gpu_cmd_push_constants(Gpu* gpu, const void* data, uint32_t size)
{
    /* Legacy API kept for compatibility. The OpenGL renderer no longer
     * routes per-mesh transform/color through `glUniform*` per draw —
     * see gpu_alloc_mesh_buffer + gpu_cmd_bind_mesh_data + gpu_cmd_set_eye_pos
     * below. The shaders' loose mvp/model/color uniforms were replaced
     * by a MeshBlock UBO (binding=1) so this function is mostly a
     * no-op now. We still set u_eyePos here for the rare caller that
     * uses push_constants without the new API. */
    if (!gpu || !data || !gpu->current_pipeline)
        return;
    const GpuMeshPushConstants* pc = (const GpuMeshPushConstants*)data;
    GpuPipeline pipe = gpu->current_pipeline;
    if (pipe->loc_eyePos >= 0)
        glUniform3fv(pipe->loc_eyePos, 1, pc->eye_pos);
    (void)size;
}

/* ---- Per-mesh data UBO ---- */

int gpu_alloc_mesh_buffer(Gpu* gpu, int nmeshes)
{
    if (!gpu || nmeshes <= 0)
        return 0;

    GLint align = 256;
    glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &align);
    if (align < 1)
        align = 256;
    int data_sz = (int)sizeof(GpuMeshData);
    /* stride MUST be a multiple of the queried GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT so every
     * per-mesh glBindBufferRange(offset = i*stride) is legally aligned. round_up(data_sz,
     * align) is already >= data_sz, so do NOT clamp align up to data_sz: that produced a
     * stride == data_sz that need not be a multiple of the hw alignment, and on ANGLE
     * (align 64, GpuMeshData 160) gave stride 160 -> offsets 160,480,... are not multiples
     * of 64 -> ANGLE drops those binds and the mesh inherits a neighbour's model matrix
     * (the exploded-robot bug on Windows; Linux/NVIDIA desktop reports align 256 >= 160). */
    int stride = (data_sz + align - 1) / align * align;

    if (gpu->mesh_ubo)
    {
        glDeleteBuffers(1, &gpu->mesh_ubo);
        gpu->mesh_ubo = 0;
    }
    glGenBuffers(1, &gpu->mesh_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, gpu->mesh_ubo);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)stride * (GLsizeiptr)nmeshes, NULL, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    gpu->mesh_stride = stride;
    gpu->mesh_count = nmeshes;
    gl_log(GL_LOG_INFO, "gpu_gles", "mesh UBO %d bytes (stride=%d × %d meshes, align=%d)", stride * nmeshes, stride,
           nmeshes, (int)align);
    return 1;
}

void* gpu_begin_mesh_writes(Gpu* gpu)
{
    if (!gpu || !gpu->mesh_ubo || gpu->mesh_count <= 0)
        return NULL;
    glBindBuffer(GL_UNIFORM_BUFFER, gpu->mesh_ubo);
    /* INVALIDATE_BUFFER + UNSYNCHRONIZED → orphan + no GPU sync; the
     * driver gets a fresh storage handle so writes can land while last
     * frame's draws are still consuming the prior one. Standard AZDO
     * pattern; the matched Apple GL-on-Metal path we proved on the
     * material UBO. */
    void* p = glMapBufferRange(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)gpu->mesh_stride * (GLsizeiptr)gpu->mesh_count,
                               GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
    return p;
}

int gpu_mesh_stride(Gpu* gpu)
{
    return gpu ? gpu->mesh_stride : 0;
}

void gpu_end_mesh_writes(Gpu* gpu)
{
    if (!gpu || !gpu->mesh_ubo)
        return;
    glUnmapBuffer(GL_UNIFORM_BUFFER);
}

void gpu_cmd_bind_mesh_data(Gpu* gpu, int mesh_index)
{
    if (!gpu || !gpu->mesh_ubo || mesh_index < 0 || mesh_index >= gpu->mesh_count)
        return;
    GLintptr offset = (GLintptr)mesh_index * (GLintptr)gpu->mesh_stride;
#if defined(_WIN32)
    /* ANGLE (GL-on-Vulkan/D3D) workaround: a repeated glBindBufferRange to the SAME
     * buffer + indexed binding that changes ONLY the offset can be dropped, leaving the
     * previously-bound slice in place. The mesh then draws with the prior mesh's (or the
     * ground's identity) model matrix and snaps to the world origin -- a stray dark blob
     * on the floor. Resetting the indexed binding to 0 first forces the offset change to
     * take. Native desktop / Apple GL honour the offset change, so this is Windows-only.
     * Set OVGL_UBO_NOREBIND=1 to disable for comparison. */
    static int rebind = -1;
    if (rebind < 0)
    {
        const char* e = getenv("OVGL_UBO_NOREBIND");
        rebind = (e && e[0] == '1') ? 0 : 1;
    }
    if (rebind)
        glBindBufferBase(GL_UNIFORM_BUFFER, 1, 0);
#endif
    glBindBufferRange(GL_UNIFORM_BUFFER, 1, gpu->mesh_ubo, offset, (GLsizeiptr)sizeof(GpuMeshData));
}

void gpu_cmd_set_eye_pos(Gpu* gpu, const float eye[3])
{
    if (!gpu || !gpu->current_pipeline || !eye)
        return;
    if (gpu->current_pipeline->loc_eyePos >= 0)
        glUniform3fv(gpu->current_pipeline->loc_eyePos, 1, eye);
}

void gpu_cmd_set_view_proj(Gpu* gpu, const float view16[16], const float proj16[16])
{
    if (!gpu || !gpu->current_pipeline || !view16 || !proj16)
        return;
    GpuPipeline pipe = gpu->current_pipeline;
    /* Caller passes row-major bytes for the same column-vector math matrices
     * used by mesh UBOs. Convert explicitly because GLES forbids GL_TRUE. */
    float view_gl[16], proj_gl[16];
    mat4_row_major_to_gl(view_gl, view16);
    mat4_row_major_to_gl(proj_gl, proj16);
    if (pipe->loc_u_view >= 0)
        glUniformMatrix4fv(pipe->loc_u_view, 1, GL_FALSE, view_gl);
    if (pipe->loc_u_proj >= 0)
        glUniformMatrix4fv(pipe->loc_u_proj, 1, GL_FALSE, proj_gl);
    /* Also push viewport size for curve TCS screen-space LOD. */
    if (pipe->loc_screen_size >= 0)
        glUniform2f(pipe->loc_screen_size, (float)gpu->width, (float)gpu->height);
}

void gpu_cmd_set_basis_id(Gpu* gpu, int basis_id)
{
    if (!gpu || !gpu->current_pipeline)
        return;
    if (gpu->current_pipeline->loc_u_basis_id >= 0)
        glUniform1i(gpu->current_pipeline->loc_u_basis_id, basis_id);
}

void gpu_set_tone_mapping(Gpu* gpu, float exposure_scale, float sky_scale, float white_point_scale, uint32_t flags)
{
    if (!gpu)
        return;
    if (!(exposure_scale > 0.0f) || !isfinite(exposure_scale))
        exposure_scale = 1.0f;
    if (!(sky_scale > 0.0f) || !isfinite(sky_scale))
        sky_scale = exposure_scale;
    if (!(white_point_scale > 0.0f) || !isfinite(white_point_scale))
        white_point_scale = 1.0f;
    gpu->tone_exposure_scale = exposure_scale;
    gpu->tone_sky_scale = sky_scale;
    gpu->tone_white_point = white_point_scale;
    gpu->tone_flags = flags;
}

/* Pushed at pipeline-bind time only, with no immediate-update path: unlike
 * gpu_set_fallback_lighting this is never called mid-frame -- the driver
 * resolves the environment once, before it binds anything. */
void gpu_set_environment_rotation(Gpu* gpu, float degrees)
{
    if (!gpu)
        return;
    gpu->env_rotation_deg = degrees;
}

void gpu_set_environment_owns_lighting(Gpu* gpu, int owns)
{
    if (!gpu)
        return;
    gpu->env_owns_lighting = owns ? 1 : 0;
}

void gpu_set_fallback_lighting(Gpu* gpu, int enabled)
{
    if (!gpu)
        return;
    gpu->fallback_lighting = enabled ? 1 : 0;
    /* current_pipeline is a logical cache.  In the adopted-context path the
     * actual GL program is restored to the embedder after every frame, so only
     * preserve the immediate-update behavior when that program is truly
     * bound.  gpu_cmd_bind_pipeline() always uploads the cached value later. */
    if (gpu->current_pipeline && gpu->current_pipeline->loc_u_fallbackLighting >= 0)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
            glUniform1i(gpu->current_pipeline->loc_u_fallbackLighting, gpu->fallback_lighting);
    }
}

void gpu_set_authored_light_count(Gpu* gpu, int nlights)
{
    if (!gpu)
        return;
    if (nlights < 0)
        nlights = 0;
    gpu->authored_light_count = nlights;
    if (gpu->current_pipeline && gpu->current_pipeline->loc_u_authoredLightCount >= 0)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
            glUniform1i(gpu->current_pipeline->loc_u_authoredLightCount, gpu->authored_light_count);
    }
}

int gpu_upload_lights(Gpu* gpu, const GpuLight* lights, int nlights)
{
    if (!gpu)
        return 0;
    if (nlights < 0)
        nlights = 0;
    int n = nlights;
    if (n > GPU_MAX_SCENE_LIGHTS)
    {
        gl_log(
            GL_LOG_WARN, "gpu_gles", "truncating %d scene lights to %d OpenGL uniforms", nlights, GPU_MAX_SCENE_LIGHTS);
        n = GPU_MAX_SCENE_LIGHTS;
    }
    memset(gpu->scene_lights, 0, sizeof(gpu->scene_lights));
    if (lights && n > 0)
        memcpy(gpu->scene_lights, lights, (size_t)n * sizeof(GpuLight));
    gpu->scene_light_count = n;
    if (gpu->current_pipeline)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
            gpu_apply_light_uniforms(gpu, gpu->current_pipeline);
    }
    return 1;
}

/* Lazily create (or grow) the shared depth atlas + its FBO and UBO.
 * `slots` is how many tiles this frame needs; the atlas is the smallest
 * square of whole tiles that holds them, capped at GPU_SHADOW_ATLAS_MAX and
 * at GL_MAX_TEXTURE_SIZE. It never shrinks -- a scene that oscillates between
 * one and two casters would otherwise reallocate a 67 MB texture per frame. */
static int gpu_shadow_ensure_atlas(Gpu* gpu, int slots)
{
    if (slots < 1)
        slots = 1;
    if (slots > GPU_MAX_SHADOW_LIGHTS)
        slots = GPU_MAX_SHADOW_LIGHTS;
    int tpr = 1;
    while (tpr * tpr < slots)
        tpr++;

    int want = tpr * GPU_SHADOW_TILE_SIZE;
    GLint max_tex = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_tex);
    if (max_tex > 0 && want > max_tex)
        want = max_tex;
    if (want > GPU_SHADOW_ATLAS_MAX)
        want = GPU_SHADOW_ATLAS_MAX;
    if (want < gpu->shadow_atlas_size)
        want = gpu->shadow_atlas_size;

    if (gpu->shadow_atlas_tex && gpu->shadow_atlas_size == want && gpu->shadow_fbo && gpu->shadow_atlas_ubo)
        return 1;

    if (gpu->shadow_atlas_tex && gpu->shadow_atlas_size != want)
    {
        glDeleteTextures(1, &gpu->shadow_atlas_tex);
        gpu->shadow_atlas_tex = 0;
    }
    if (!gpu->shadow_atlas_tex)
    {
        glGenTextures(1, &gpu->shadow_atlas_tex);
        glBindTexture(GL_TEXTURE_2D, gpu->shadow_atlas_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, want, want, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, NULL);
        /* GL_LINEAR + COMPARE_REF_TO_TEXTURE is what makes each of the
         * shader's 9 taps a BILINEAR depth comparison done in the TMU -- a
         * 4x4-quality kernel for 9 fetches, free. The pre-atlas map was
         * NEAREST with no compare mode and the shader did 9 binary compares
         * in ALU, which is where the stair-stepped penumbra came from. */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    }
    gpu->shadow_atlas_size = want;
    gpu->shadow_tile_size = GPU_SHADOW_TILE_SIZE;
    gpu->shadow_tiles_per_row = want / GPU_SHADOW_TILE_SIZE;
    if (gpu->shadow_tiles_per_row < 1)
    {
        /* A driver whose GL_MAX_TEXTURE_SIZE is below one tile: fall back to
         * a single tile the size of the whole atlas rather than dividing by
         * zero and writing tiles outside the texture. */
        gpu->shadow_tile_size = want;
        gpu->shadow_tiles_per_row = 1;
    }

    if (!gpu->shadow_fbo)
        glGenFramebuffers(1, &gpu->shadow_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, gpu->shadow_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, gpu->shadow_atlas_tex, 0);
#ifdef NUSD_DESKTOP_GL
    /* Depth-only FBO: desktop GL needs NONE draw/read buffers to call it
     * complete with no color attachment; GLES is complete as-is. */
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
#endif
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)gpu->shadow_saved_fbo);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        gl_log(GL_LOG_ERROR, "gpu_gles", "shadow atlas framebuffer incomplete: 0x%x", (unsigned)status);
        return 0;
    }
    if (!gpu->shadow_atlas_ubo)
    {
        glGenBuffers(1, &gpu->shadow_atlas_ubo);
        glBindBuffer(GL_UNIFORM_BUFFER, gpu->shadow_atlas_ubo);
        glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)sizeof(gpu->shadow_lights), NULL, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_UNIFORM_BUFFER, 0);
    }
    /* The atlas is the largest single allocation this backend makes on a
     * shadowed frame, and it is demand-sized -- say so, once per (re)alloc,
     * so a VRAM surprise is attributable instead of mysterious. */
    gl_log(GL_LOG_INFO, "gpu_gles", "shadow atlas %dx%d (%d x %d-px tiles, %d slots, %.1f MB)", want, want,
           gpu->shadow_tiles_per_row, gpu->shadow_tile_size, gpu->shadow_tiles_per_row * gpu->shadow_tiles_per_row,
           (double)want * want * 4.0 / (1024.0 * 1024.0));
    return 1;
}

int gpu_shadow_begin(Gpu* gpu, int slot, int slot_count, const float light_vp[16], int light_index, float depth_bias)
{
    if (!gpu || !light_vp)
        return 0;
    if (slot < 0 || slot >= GPU_MAX_SHADOW_LIGHTS)
        return 0;
    if (light_index < 0 || light_index >= GPU_MAX_SCENE_LIGHTS)
        return 0;
    /* Remember the render target bound before diverting to the shadow FBO so
     * gpu_shadow_end can restore it. Hardcoding FB 0 only works when FB 0 is the
     * real target (a window); a headless offscreen FBO would otherwise lose the
     * entire main pass to the windowless zero-size default framebuffer. Guard
     * against clobbering when a prior shadow slot is still bound. */
    {
        GLint cur = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &cur);
        if ((GLuint)cur != gpu->shadow_fbo)
            gpu->shadow_saved_fbo = cur;
    }
    if (slot == 0)
        gpu_shadow_clear(gpu);
    if (!gpu_shadow_ensure_atlas(gpu, (slot == 0) ? slot_count : slot + 1))
        return 0;

    const int tile = gpu->shadow_tile_size;
    const int tpr = gpu->shadow_tiles_per_row;
    if (slot >= tpr * tpr)
        return 0; /* atlas too small for this slot */
    const int tx = slot % tpr, ty = slot / tpr;
    const int ox = tx * tile, oy = ty * tile;

    glBindFramebuffer(GL_FRAMEBUFFER, gpu->shadow_fbo);
    if (slot == 0)
    {
        /* Clear the WHOLE atlas ONCE, scissor DISABLED: glClear ignores
         * glViewport but obeys glScissor, so clearing per tile (or with the
         * scissor still on from a previous tile) would wipe earlier tiles. */
        glDisable(GL_SCISSOR_TEST);
        glViewport(0, 0, gpu->shadow_atlas_size, gpu->shadow_atlas_size);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        glDepthMask(GL_TRUE);
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    /* This slot's atlas record. uvScaleOffset places the tile: scale 1/tpr,
     * offset (tx, ty)/tpr. bias is already in window-depth units. */
    GpuShadowLight* s = &gpu->shadow_lights[slot];
    memcpy(s->lightVP, light_vp, sizeof(s->lightVP));
    s->uvScaleOffset[0] = s->uvScaleOffset[1] = 1.0f / (float)tpr;
    s->uvScaleOffset[2] = (float)tx / (float)tpr;
    s->uvScaleOffset[3] = (float)ty / (float)tpr;
    s->bias[0] = (depth_bias > 0.0f) ? depth_bias : 0.0f;
    s->bias[1] = s->bias[2] = s->bias[3] = 0.0f;
    gpu->shadow_light_to_slot[light_index] = slot;

    /* Confine depth writes AND polygon offset to this tile with the scissor. */
    glViewport(ox, oy, tile, tile);
    glEnable(GL_SCISSOR_TEST);
    glScissor(ox, oy, tile, tile);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);
    gpu->shadow_active_slot = slot;
    /* The pass VP must reach an ALREADY-bound shadow pipeline too: tiles after
     * the first call begin() without rebinding, and without this they would
     * re-render slot 0's frustum into slot 1's tile. */
    if (gpu->current_pipeline && gpu->current_pipeline->loc_u_shadowPassVP >= 0)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
        {
            float vp_gl[16];
            mat4_row_major_to_gl(vp_gl, s->lightVP);
            glUniformMatrix4fv(gpu->current_pipeline->loc_u_shadowPassVP, 1, GL_FALSE, vp_gl);
        }
    }
    return 1;
}

void gpu_shadow_end(Gpu* gpu)
{
    if (!gpu)
        return;
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    int slot = gpu->shadow_active_slot;
    if (slot >= 0 && slot < GPU_MAX_SHADOW_LIGHTS)
    {
        if (gpu->shadow_count < slot + 1)
            gpu->shadow_count = slot + 1;
        /* Upload just this slot's 96 bytes into the std140 ShadowBlock. */
        if (gpu->shadow_atlas_ubo)
        {
            glBindBuffer(GL_UNIFORM_BUFFER, gpu->shadow_atlas_ubo);
            glBufferSubData(GL_UNIFORM_BUFFER, (GLintptr)(slot * (int)sizeof(GpuShadowLight)), sizeof(GpuShadowLight),
                            &gpu->shadow_lights[slot]);
            glBindBuffer(GL_UNIFORM_BUFFER, 0);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)gpu->shadow_saved_fbo);
    glViewport(0, 0, gpu->width, gpu->height);
    gpu->shadow_active_slot = -1;
}

void gpu_shadow_clear(Gpu* gpu)
{
    if (!gpu)
        return;
    gpu->shadow_count = 0;
    gpu->shadow_active_slot = -1;
    for (int i = 0; i < GPU_MAX_SCENE_LIGHTS; i++)
        gpu->shadow_light_to_slot[i] = -1;
    memset(gpu->shadow_lights, 0, sizeof(gpu->shadow_lights));
    /* Clearing is observable state, just like uploading an empty light list.
     * Refresh the active material program immediately when possible; callers
     * must not depend on a later pipeline bind to stop sampling an old map. */
    if (gpu->current_pipeline)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
            gpu_apply_shadow_uniforms(gpu, gpu->current_pipeline);
    }
}

void gpu_cmd_draw(Gpu* gpu, uint32_t vertex_count, uint32_t first_vertex)
{
    if (!gpu)
        return;
    GLenum prim =
        (gpu->current_pipeline && gpu->current_pipeline->primitive) ? gpu->current_pipeline->primitive : GL_TRIANGLES;
    glDrawArrays(prim, (GLint)first_vertex, (GLsizei)vertex_count);
}

void gpu_cmd_draw_indexed(Gpu* gpu, uint32_t index_count, uint32_t first_index, int32_t vertex_offset)
{
    if (!gpu)
        return;
    /*
     * Indices are pre-offset to absolute vertex space at build time, so
     * vertex_offset is ignored. (GLES 3.2 has glDrawElementsBaseVertex
     * but switching to it would require regenerating IBs as per-mesh
     * relative — out of scope for the spec bump.)
     */
    (void)vertex_offset;
    GLenum prim =
        (gpu->current_pipeline && gpu->current_pipeline->primitive) ? gpu->current_pipeline->primitive : GL_TRIANGLES;
    glDrawElements(prim, (GLsizei)index_count, GL_UNSIGNED_INT, (void*)(uintptr_t)(first_index * sizeof(uint32_t)));
}

void gpu_cmd_draw_indexed_typed(
    Gpu* gpu, uint32_t index_count, uint64_t index_byte_offset, int32_t vertex_offset, int index_type_bits)
{
    if (!gpu)
        return;
    GLenum prim =
        (gpu->current_pipeline && gpu->current_pipeline->primitive) ? gpu->current_pipeline->primitive : GL_TRIANGLES;
    GLenum type = (index_type_bits == 16) ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT;
    glDrawElementsBaseVertex(prim, (GLsizei)index_count, type, (void*)(uintptr_t)index_byte_offset, (GLint)vertex_offset);
}

void gpu_upload_instance_transforms(Gpu* gpu, const float* matrices16, uint32_t count)
{
    if (!gpu)
        return;
    if (gpu->instance_vbo)
    {
        glDeleteBuffers(1, &gpu->instance_vbo);
        gpu->instance_vbo = 0;
    }
    gpu->instance_count = 0;
    if (!matrices16 || count == 0)
        return;
    glGenBuffers(1, &gpu->instance_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, gpu->instance_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)count * 16 * sizeof(float), matrices16, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    gpu->instance_count = count;
}

void gpu_cmd_draw_instanced(
    Gpu* gpu, uint32_t index_count, uint32_t first_index, uint32_t instance_count, uint32_t first_instance)
{
    if (!gpu || !gpu->instance_vbo || instance_count == 0 || index_count == 0)
        return;
    GpuPipeline pipe = gpu->current_pipeline;
    if (pipe)
        glBindVertexArray(pipe->vao);
    /* Per-instance world matrix as 4 vec4 columns at locations 6-9, divisor 1.
     * GLES 3.2 has no baseInstance, so select the batch's slice via the
     * attribute pointer offset (first_instance * 64 bytes). */
    glBindBuffer(GL_ARRAY_BUFFER, gpu->instance_vbo);
    const GLsizei istride = 16 * (GLsizei)sizeof(float);
    const size_t base = (size_t)first_instance * (size_t)istride;
    for (int c = 0; c < 4; c++)
    {
        GLuint loc = (GLuint)(6 + c);
        glEnableVertexAttribArray(loc);
        glVertexAttribPointer(loc, 4, GL_FLOAT, GL_FALSE, istride, (void*)(base + (size_t)c * 4 * sizeof(float)));
        glVertexAttribDivisor(loc, 1);
    }
    GLenum prim = (pipe && pipe->primitive) ? pipe->primitive : GL_TRIANGLES;
    glDrawElementsInstanced(prim, (GLsizei)index_count, GL_UNSIGNED_INT,
                            (void*)(uintptr_t)((size_t)first_index * sizeof(uint32_t)), (GLsizei)instance_count);
}

/* ---- Materials ---- */

int gpu_upload_materials(
    Gpu* gpu, const GpuMaterialParams* materials, int nmaterials, const GpuTextureData* textures, int ntextures)
{
    if (!gpu)
        return 0;

    /* Store material params on CPU (uploaded per-draw as UBO) */
    gpu->mat_params = (GpuMaterialParams*)malloc((size_t)nmaterials * sizeof(GpuMaterialParams));
    if (!gpu->mat_params)
        return 0;
    memcpy(gpu->mat_params, materials, (size_t)nmaterials * sizeof(GpuMaterialParams));
    gpu->mat_count = nmaterials;

    /* One big UBO containing every material laid out at slice strides
     * aligned to UNIFORM_BUFFER_OFFSET_ALIGNMENT (256 bytes on macOS).
     * Per-draw cost becomes a `glBindBufferRange` (cheap) instead of a
     * `glBufferSubData` (~65 μs each on Apple's GL-on-Metal). */
    GLint align = 256;
    glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &align);
    if (align < 1)
        align = 256;
    /* round the struct size up to the offset alignment (always >= the struct size); do NOT
     * clamp align up to the struct size or the stride stops being a multiple of the hw
     * alignment and the per-material bind is dropped on ANGLE -- see gpu_alloc_mesh_buffer. */
    int param_sz = (int)sizeof(GpuMaterialParams);
    int stride = (param_sz + align - 1) / align * align;
    gpu->mat_stride = stride;

    GLsizeiptr total = (GLsizeiptr)stride * (GLsizeiptr)nmaterials;
    unsigned char* packed = (unsigned char*)calloc(1, (size_t)total);
    if (!packed)
    {
        free(gpu->mat_params);
        gpu->mat_params = NULL;
        gpu->mat_count = 0;
        return 0;
    }
    for (int i = 0; i < nmaterials; i++)
    {
        memcpy(packed + (size_t)i * (size_t)stride, &materials[i], (size_t)param_sz);
    }
    glGenBuffers(1, &gpu->mat_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, gpu->mat_ubo);
    glBufferData(GL_UNIFORM_BUFFER, total, packed, GL_STATIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    free(packed);
    gl_log(GL_LOG_INFO, "gpu_gles", "material UBO %lld bytes (stride=%d × %d materials, align=%d)", (long long)total,
           stride, nmaterials, (int)align);

    /* Create 1x1 white dummy texture */
    unsigned char white[4] = { 255, 255, 255, 255 };
    glGenTextures(1, &gpu->mat_dummy_tex);
    glBindTexture(GL_TEXTURE_2D, gpu->mat_dummy_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    /* Upload textures */
    if (ntextures > 0 && textures)
    {
        gpu->mat_textures = (GLuint*)malloc((size_t)ntextures * sizeof(GLuint));
        if (!gpu->mat_textures)
        {
            /* Roll back partial state so the caller sees a clean failure. */
            free(gpu->mat_params);
            gpu->mat_params = NULL;
            gpu->mat_count = 0;
            glDeleteBuffers(1, &gpu->mat_ubo);
            gpu->mat_ubo = 0;
            glDeleteTextures(1, &gpu->mat_dummy_tex);
            gpu->mat_dummy_tex = 0;
            return 0;
        }
        glGenTextures(ntextures, gpu->mat_textures);

/* Probe max supported anisotropy for this renderer's current context.
 * EXT_texture_filter_anisotropic
 * has been universally implemented since GLES 3.0 / GL 4.6, but the
 * enum lives in an extension header and is not in the GLES core
 * tokens we've pulled in — define it locally if missing. */
#ifndef GL_TEXTURE_MAX_ANISOTROPY_EXT
#    define GL_TEXTURE_MAX_ANISOTROPY_EXT 0x84FE
#endif
#ifndef GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT
#    define GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT 0x84FF
#endif
        GLfloat max_aniso = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_aniso);
        if (max_aniso < 1.0f)
            max_aniso = 1.0f; /* not supported */
        GLfloat aniso = max_aniso > 16.0f ? 16.0f : max_aniso; /* match vulkan */

        for (int i = 0; i < ntextures; i++)
        {
            glBindTexture(GL_TEXTURE_2D, gpu->mat_textures[i]);
            /* sRGB internal format for color textures (diffuse, emissive,
             * opacity); linear RGBA8 for data textures (normal, roughness,
             * metallic, AO). Sampling sRGB data through a linear sampler
             * makes color textures look too dark / wrong gamma; sampling
             * linear data through sRGB corrupts normals + PBR. The
             * is_srgb flag is decided by vote across material slots
             * (see material.c); mirrors vulkan's classification. */
            GLint internal_fmt = textures[i].is_srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8;
            glTexImage2D(GL_TEXTURE_2D, 0, internal_fmt, textures[i].width, textures[i].height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, textures[i].pixels);
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            if (aniso > 1.0f)
            {
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, aniso);
            }

            gpu->allocated_bytes += (uint64_t)textures[i].width * (uint64_t)textures[i].height * 4;
        }
        gpu->mat_tex_count = ntextures;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    gl_log(GL_LOG_INFO, "gpu_gles", "uploaded %d materials, %d textures", nmaterials, ntextures);
    return 1;
}

int gpu_upload_ptex_triangle_colors(Gpu* gpu, const uint32_t* colors, uint32_t count)
{
    if (!gpu)
        return 0;
    if (gpu->ptex_color_ssbo)
    {
#ifndef NUSD_DESKTOP_GL
        glDeleteBuffers(1, &gpu->ptex_color_ssbo);
#endif
        uint64_t old_bytes = (uint64_t)gpu->ptex_color_count * sizeof(uint32_t);
        if (gpu->allocated_bytes >= old_bytes)
            gpu->allocated_bytes -= old_bytes;
        gpu->ptex_color_ssbo = 0;
        gpu->ptex_color_count = 0;
    }
    if (!colors || count == 0)
        return 1;

#ifdef NUSD_DESKTOP_GL
    gl_log(GL_LOG_ERROR, "gpu_gles", "Ptex triangle-color SSBO unavailable on desktop GL 4.1");
    return 0;
#else
    glGenBuffers(1, &gpu->ptex_color_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gpu->ptex_color_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)((uint64_t)count * sizeof(uint32_t)), colors, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gpu->ptex_color_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    gpu->ptex_color_count = count;
    gpu->allocated_bytes += (uint64_t)count * sizeof(uint32_t);
    gl_log(GL_LOG_INFO, "gpu_gles",
           "uploaded Ptex triangle-corner colors: %u colors "
           "(%u triangles, %.1f MB)",
           count, count / 3u, (double)((uint64_t)count * sizeof(uint32_t)) / (1024.0 * 1024.0));
    return 1;
#endif
}

/* Once-per-frame: bind UBO/IBL & set sampler→unit mappings + IBL uniforms
 * & last-bound-pipeline-wide uniforms. Caller invokes after binding the
 * material pipeline + buffers and BEFORE the per-mesh loop, so
 * `gpu_cmd_bind_material` can stay tight. */
void gpu_cmd_begin_material_pass(Gpu* gpu)
{
    if (!gpu || !gpu->current_pipeline)
        return;
    GpuPipeline pipe = gpu->current_pipeline;

    /* No glBindBufferBase here — gpu_cmd_bind_material uses
     * glBindBufferRange per draw to point at this material's slice of
     * the all-materials UBO. */

    if (!pipe->samplers_assigned)
    {
        if (pipe->loc_tex_diffuse >= 0)
            glUniform1i(pipe->loc_tex_diffuse, 0);
        if (pipe->loc_tex_normal >= 0)
            glUniform1i(pipe->loc_tex_normal, 1);
        if (pipe->loc_tex_roughness >= 0)
            glUniform1i(pipe->loc_tex_roughness, 2);
        if (pipe->loc_tex_metallic >= 0)
            glUniform1i(pipe->loc_tex_metallic, 3);
        if (pipe->loc_tex_emissive >= 0)
            glUniform1i(pipe->loc_tex_emissive, 4);
        if (pipe->loc_tex_occlusion >= 0)
            glUniform1i(pipe->loc_tex_occlusion, 5);
        if (pipe->loc_u_envMap >= 0)
            glUniform1i(pipe->loc_u_envMap, 6);
        if (pipe->loc_u_brdfLUT >= 0)
            glUniform1i(pipe->loc_u_brdfLUT, 7);
        if (pipe->loc_u_irrMap >= 0)
            glUniform1i(pipe->loc_u_irrMap, 8);
        if (pipe->loc_tex_opacity >= 0)
            glUniform1i(pipe->loc_tex_opacity, 9);
        /* Unit 10 only: the atlas collapsed the former shadowMap0/1 pair onto
         * one sampler, which frees unit 11 again. */
        if (pipe->loc_u_shadowAtlas >= 0)
            glUniform1i(pipe->loc_u_shadowAtlas, 10);
        if (pipe->loc_u_ssaoTex >= 0)
            glUniform1i(pipe->loc_u_ssaoTex, 11);
        pipe->samplers_assigned = 1;
    }

#ifndef NUSD_DESKTOP_GL
    if (gpu->ptex_color_ssbo)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gpu->ptex_color_ssbo);
#endif

    if (pipe->loc_u_debugMode >= 0)
        glUniform1i(pipe->loc_u_debugMode, gpu->debug_mode);

    if (gpu->has_ibl)
    {
        glActiveTexture(GL_TEXTURE6);
        glBindTexture(GL_TEXTURE_2D, gpu->env_map);
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, gpu->brdf_lut);
        if (gpu->irr_map)
        {
            glActiveTexture(GL_TEXTURE8);
            glBindTexture(GL_TEXTURE_2D, gpu->irr_map);
        }
        if (pipe->loc_u_envMipLevels >= 0)
            glUniform1i(pipe->loc_u_envMipLevels, gpu->env_mip_levels);
        if (pipe->loc_u_hasIBL >= 0)
            glUniform1i(pipe->loc_u_hasIBL, 1);
    }
    else if (pipe->loc_u_hasIBL >= 0)
    {
        GLuint dummy = gpu->mat_dummy_tex;
        glActiveTexture(GL_TEXTURE6);
        glBindTexture(GL_TEXTURE_2D, dummy);
        glActiveTexture(GL_TEXTURE7);
        glBindTexture(GL_TEXTURE_2D, dummy);
        glActiveTexture(GL_TEXTURE8);
        glBindTexture(GL_TEXTURE_2D, dummy);
        glUniform1i(pipe->loc_u_hasIBL, 0);
    }
    gpu_apply_shadow_uniforms(gpu, pipe);
}

void gpu_cmd_bind_material(Gpu* gpu, int material_index)
{
    if (!gpu || !gpu->mat_params || material_index < 0 || material_index >= gpu->mat_count)
        return;

    /* Same material as last bind? Bail — the UBO + textures are still
     * correctly bound. Important when meshes are sorted by material:
     * runs of N same-material meshes collapse to one bind + N-1 cheap
     * cached calls. */
    if (gpu->last_bound_material == material_index)
    {
        return;
    }
    gpu->last_bound_material = material_index;

    const GpuMaterialParams* mat = &gpu->mat_params[material_index];

    /* Bind the slice of the all-materials UBO that holds this material.
     * We pre-uploaded everything once at load time so per-draw cost is
     * a binding (essentially a few ints in driver state) instead of a
     * 200-byte glBufferSubData copy. On Apple's GL-on-Metal driver
     * subdata costs ~65 μs per call due to validation + staging — the
     * single biggest frame-time win we found, ~75 ms → ~3 ms / frame. */
    GLintptr offset = (GLintptr)material_index * (GLintptr)gpu->mat_stride;
    glBindBufferRange(GL_UNIFORM_BUFFER, 0, gpu->mat_ubo, offset, (GLsizeiptr)sizeof(GpuMaterialParams));
    (void)mat;

    /* Bind PBR textures to units 0-5: diffuse, normal, roughness, metallic,
     * emissive, occlusion. Skip slots that match the previously-bound
     * material — Apple's GL driver still validates state on every bind
     * even when the texture is unchanged, and most adjacent materials
     * in the warehouse share the same diffuse/normal/ORM. */
    for (int slot = 0; slot < 6; slot++)
    {
        GLuint tex = gpu->mat_dummy_tex;
        int idx = mat->tex_indices[slot];
        if (idx >= 0 && idx < gpu->mat_tex_count)
            tex = gpu->mat_textures[idx];
        if (tex == gpu->last_bound_tex[slot])
            continue;
        glActiveTexture(GL_TEXTURE0 + (GLenum)slot);
        glBindTexture(GL_TEXTURE_2D, tex);
        gpu->last_bound_tex[slot] = tex;
    }

    /* Opacity texture on unit 9 (cached too). */
    GLuint opa_tex = gpu->mat_dummy_tex;
    int opa_idx = mat->tex_indices[6]; /* TEX_OPACITY */
    if (opa_idx >= 0 && opa_idx < gpu->mat_tex_count)
        opa_tex = gpu->mat_textures[opa_idx];
    if (opa_tex != gpu->last_bound_tex[6])
    {
        glActiveTexture(GL_TEXTURE9);
        glBindTexture(GL_TEXTURE_2D, opa_tex);
        gpu->last_bound_tex[6] = opa_tex;
    }
}

void gpu_cmd_bind_materials(Gpu* gpu)
{
    /* For compatibility — bind material 0 as default */
    if (gpu && gpu->mat_count > 0)
        gpu_cmd_bind_material(gpu, 0);
}

int gpu_replace_materials(
    Gpu* gpu, const GpuMaterialParams* materials, int nmaterials, const GpuTextureData* textures, int ntextures)
{
    if (!gpu)
        return 0;
    /* Free exactly the material-owned resources gpu_upload_materials will
     * recreate. Deliberately NOT gpu_destroy_materials: that also drops the
     * mesh UBO and ptex colors, which belong to the (unchanged) geometry. */
    if (gpu->mat_textures)
    {
        glDeleteTextures(gpu->mat_tex_count, gpu->mat_textures);
        free(gpu->mat_textures);
        gpu->mat_textures = NULL;
    }
    gpu->mat_tex_count = 0;
    if (gpu->mat_ubo)
    {
        glDeleteBuffers(1, &gpu->mat_ubo);
        gpu->mat_ubo = 0;
    }
    if (gpu->mat_dummy_tex)
    {
        glDeleteTextures(1, &gpu->mat_dummy_tex);
        gpu->mat_dummy_tex = 0;
    }
    free(gpu->mat_params);
    gpu->mat_params = NULL;
    gpu->mat_count = 0;
    return gpu_upload_materials(gpu, materials, nmaterials, textures, ntextures);
}

void gpu_destroy_materials(Gpu* gpu)
{
    if (!gpu)
        return;
    if (gpu->mat_textures)
    {
        glDeleteTextures(gpu->mat_tex_count, gpu->mat_textures);
        free(gpu->mat_textures);
        gpu->mat_textures = NULL;
    }
    if (gpu->mat_ubo)
    {
        glDeleteBuffers(1, &gpu->mat_ubo);
        gpu->mat_ubo = 0;
    }
    if (gpu->mesh_ubo)
    {
        glDeleteBuffers(1, &gpu->mesh_ubo);
        gpu->mesh_ubo = 0;
    }
    if (gpu->ptex_color_ssbo)
    {
#ifndef NUSD_DESKTOP_GL
        glDeleteBuffers(1, &gpu->ptex_color_ssbo);
#endif
        uint64_t bytes = (uint64_t)gpu->ptex_color_count * sizeof(uint32_t);
        if (gpu->allocated_bytes >= bytes)
            gpu->allocated_bytes -= bytes;
        gpu->ptex_color_ssbo = 0;
        gpu->ptex_color_count = 0;
    }
    if (gpu->mat_dummy_tex)
    {
        glDeleteTextures(1, &gpu->mat_dummy_tex);
        gpu->mat_dummy_tex = 0;
    }
    free(gpu->mat_params);
    gpu->mat_params = NULL;
    gpu->mat_count = 0;
    gpu->mat_tex_count = 0;
    gpu->mesh_count = 0;
    gpu->mesh_stride = 0;
}

/* ---- IBL Environment ---- */

/* SH3 projection of equirectangular environment map */
static void sh_project_environment(const float* rgb_data, int w, int h, float sh_coeffs[9][3])
{
    memset(sh_coeffs, 0, 9 * 3 * sizeof(float));
    const float PI = 3.14159265358979323846f;
    for (int y = 0; y < h; y++)
    {
        float theta = PI * ((float)y + 0.5f) / (float)h;
        float sin_theta = sinf(theta), cos_theta = cosf(theta);
        float da = (2.0f * PI / (float)w) * (PI / (float)h) * sin_theta;
        for (int x = 0; x < w; x++)
        {
            float phi = 2.0f * PI * ((float)x + 0.5f) / (float)w;
            const float dome_offset = 340.0f / 360.0f;
            float phi_aligned = phi - (0.5f + dome_offset) * 2.0f * PI;
            float dx = sin_theta * sinf(phi_aligned);
            float dy = sin_theta * cosf(phi_aligned);
            float dz = cos_theta;
            int idx = (y * w + x) * 3;
            float r = rgb_data[idx + 0] * da, g = rgb_data[idx + 1] * da, b = rgb_data[idx + 2] * da;
            float Y[9];
            Y[0] = 0.282095f;
            Y[1] = 0.488603f * dy;
            Y[2] = 0.488603f * dz;
            Y[3] = 0.488603f * dx;
            Y[4] = 1.092548f * dx * dy;
            Y[5] = 1.092548f * dy * dz;
            Y[6] = 0.315392f * (3.0f * dz * dz - 1.0f);
            Y[7] = 1.092548f * dx * dz;
            Y[8] = 0.546274f * (dx * dx - dy * dy);
            for (int c = 0; c < 9; c++)
            {
                sh_coeffs[c][0] += r * Y[c];
                sh_coeffs[c][1] += g * Y[c];
                sh_coeffs[c][2] += b * Y[c];
            }
        }
    }
}

/* Render cosine-convolved SH irradiance to equirectangular RGBA map */
static void sh_render_irradiance(const float* sh_coeffs, float* rgba_out, int w, int h)
{
    const float PI = 3.14159265358979323846f;
    const float c1 = 0.429043f, c2 = 0.511664f;
    const float c3 = 0.743125f, c4 = 0.886227f, c5 = 0.247708f;
    for (int y = 0; y < h; y++)
    {
        float theta = PI * ((float)y + 0.5f) / (float)h;
        float sin_theta = sinf(theta), cos_theta = cosf(theta);
        for (int x = 0; x < w; x++)
        {
            float phi = 2.0f * PI * ((float)x + 0.5f) / (float)w;
            const float dome_offset = 340.0f / 360.0f;
            float phi_aligned = phi - (0.5f + dome_offset) * 2.0f * PI;
            float nx = sin_theta * sinf(phi_aligned);
            float ny = sin_theta * cosf(phi_aligned);
            float nz = cos_theta;
            int idx = (y * w + x) * 4;
            for (int ch = 0; ch < 3; ch++)
            {
                float L[9];
                for (int i = 0; i < 9; i++)
                    L[i] = sh_coeffs[i * 3 + ch];
                float irr = c4 * L[0] + 2.0f * c2 * (L[1] * ny + L[2] * nz + L[3] * nx) +
                            2.0f * c1 * (L[4] * nx * ny + L[5] * ny * nz + L[7] * nx * nz) + c3 * L[6] * (nz * nz) -
                            c5 * L[6] + c1 * L[8] * (nx * nx - ny * ny);
                if (irr < 0.0f)
                    irr = 0.0f;
                rgba_out[idx + ch] = irr;
            }
            rgba_out[idx + 3] = 1.0f;
        }
    }
}

/*
 * Generate BRDF integration LUT on CPU using importance-sampled GGX.
 * Output: 128x128 RG16F texture where R=scale, G=bias for split-sum.
 */
static void generate_brdf_lut(float* out, int size)
{
    for (int y = 0; y < size; y++)
    {
        float roughness = (float)(y + 0.5f) / (float)size;
        float a = roughness * roughness;
        float a2 = a * a;

        for (int x = 0; x < size; x++)
        {
            float NdotV = (float)(x + 0.5f) / (float)size;
            if (NdotV < 0.001f)
                NdotV = 0.001f;

            float V[3] = { sqrtf(1.0f - NdotV * NdotV), 0.0f, NdotV };
            float scale = 0.0f, bias = 0.0f;
            const int N_SAMPLES = 256;

            for (int i = 0; i < N_SAMPLES; i++)
            {
                /* Hammersley sequence */
                float xi1 = (float)i / (float)N_SAMPLES;
                unsigned int bits = (unsigned int)i;
                bits = (bits << 16u) | (bits >> 16u);
                bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
                bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
                bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
                bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
                float xi2 = (float)bits * 2.3283064365386963e-10f;

                /* GGX importance sampling */
                float phi = 2.0f * 3.14159265f * xi1;
                float cosTheta = sqrtf((1.0f - xi2) / (1.0f + (a2 - 1.0f) * xi2));
                float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);

                float H[3] = { cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta };

                /* Reflect V around H */
                float VdotH = V[0] * H[0] + V[1] * H[1] + V[2] * H[2];
                float L[3] = { 2.0f * VdotH * H[0] - V[0], 2.0f * VdotH * H[1] - V[1], 2.0f * VdotH * H[2] - V[2] };
                float NdotL = L[2];

                if (NdotL > 0.0f)
                {
                    float NdotH = H[2];
                    if (VdotH < 0.0f)
                        VdotH = 0.0f;

                    /* Smith G2 (height-correlated) */
                    float k = a / 2.0f;
                    float G_V = NdotV / (NdotV * (1.0f - k) + k);
                    float G_L = NdotL / (NdotL * (1.0f - k) + k);
                    float G = G_V * G_L;

                    float G_Vis = (G * VdotH) / (NdotH * NdotV);
                    float Fc = powf(1.0f - VdotH, 5.0f);

                    scale += (1.0f - Fc) * G_Vis;
                    bias += Fc * G_Vis;
                }
            }

            scale /= (float)N_SAMPLES;
            bias /= (float)N_SAMPLES;

            int idx = (y * size + x) * 2;
            out[idx + 0] = scale;
            out[idx + 1] = bias;
        }
    }
}

int gpu_load_environment(Gpu* gpu, const char* hdr_path)
{
    return gpu_load_environment_intensity(gpu, hdr_path, -1.0f);
}

void gpu_set_scene_radius(Gpu* gpu, float radius)
{
    if (!gpu)
        return;
    gpu->scene_radius = (radius > 1e-3f) ? radius : kHazeReferenceRadius;
}

/* Re-stamp the shader marker AFTER a load. Same sign convention as
 * env_build_from_rgb: >= 0 is the radiometric marker (+1.0, never the authored
 * number -- that is baked into the texels), < 0 keeps the legacy auto-exposed
 * marker whose magnitude is the caller's intensity. */
void gpu_set_environment_intensity(Gpu* gpu, float intensity)
{
    if (!gpu)
        return;
    gpu->env_intensity = (intensity < 0.0f) ? intensity : 1.0f;
}

/* Shared environment build: SH-project `rgb_data` (equirectangular, 3 floats
 * per texel), derive the irradiance map, upload the radiance map + BRDF LUT and
 * compile the background shader. Split out of gpu_load_environment_intensity so
 * a synthesized environment can reach the same machinery as a decoded HDR file;
 * the caller owns rgb_data. `tint` (may be NULL) is UsdLux `inputs:color`,
 * multiplied in BEFORE the SH projection so the irradiance map carries it too.
 *
 * TWO texel conventions, selected by the SIGN of `intensity`, and the sign is
 * also what the shaders branch on through u_envIntensity:
 *
 *   intensity >= 0  RADIOMETRIC (an authored UsdLuxDomeLight). Texels carry
 *                   FINAL linear radiance in nits, hdr * inputs:color *
 *                   intensity * 2^exposure, and gpu->env_intensity degrades to
 *                   a +1.0 MARKER the shaders must not multiply by. Both the
 *                   surface ambient and the sky then go through the ONE shared
 *                   camera transform aces(nits * NU_CAMERA_EXPOSURE), so the
 *                   frame is exactly linear in the authored number at every
 *                   intensity.
 *   intensity <  0  LEGACY (the host ovgl_set_environment override). Keeps the
 *                   old pi/avg_irr auto-exposure and stores the negative
 *                   marker; |value| is the caller's intensity. Unchanged, and
 *                   deliberately so -- that lane's 4.5/2000 tone pair is
 *                   calibrated against exactly this normalization.
 *
 * Why the split is not optional: auto-exposure normalizes an environment to
 * unit average irradiance, which for a CONSTANT map is precisely a divide by
 * its own radiance -- it erases the intensity it is handed. Keeping intensity
 * as a live uniform instead (what ovgl did) forces a per-source tone lift and
 * a nonlinear ramp: measured surface/sky consistency ratio under that scheme
 * was 634.9 at intensity 1, 0.98 at 900 and 0.74 at 5000, i.e. self-consistent
 * only in a narrow band around 900-1200. Under the bake it is 1.0 everywhere.
 *
 * Calibration evidence (official ovrtx 0.4, attach lane, 0.5-albedo Lambertian
 * plate fully open to a white dome, intensity swept 3..3000): official's
 * scene-linear response is 4.54e-4 per unit intensity and LINEAR across three
 * decades (4.29/4.26/4.62/4.55/4.55/4.54/4.58 e-4 at I=3/10/30/100/300/1000/
 * 3000). This model predicts kD*albedo*NU_CAMERA_EXPOSURE = 0.914*0.5*8.82e-4
 * = 4.03e-4 for the diffuse term alone, and the split-sum specular closes the
 * rest. */
static int env_build_from_rgb(Gpu* gpu, const float* rgb_data, int w, int h, float intensity, const float tint[3])
{
    float tint_rgb[3] = { 1.0f, 1.0f, 1.0f };
    if (tint)
    {
        /* Negative radiance is not a thing a light can emit. */
        tint_rgb[0] = tint[0] > 0.0f ? tint[0] : 0.0f;
        tint_rgb[1] = tint[1] > 0.0f ? tint[1] : 0.0f;
        tint_rgb[2] = tint[2] > 0.0f ? tint[2] : 0.0f;
    }
    float* tinted = (float*)malloc((size_t)w * (size_t)h * 3 * sizeof(float));
    if (!tinted)
        return 0;
    for (int i = 0; i < w * h; i++)
    {
        tinted[i * 3 + 0] = rgb_data[i * 3 + 0] * tint_rgb[0];
        tinted[i * 3 + 1] = rgb_data[i * 3 + 1] * tint_rgb[1];
        tinted[i * 3 + 2] = rgb_data[i * 3 + 2] * tint_rgb[2];
    }

    /* Project environment to SH3 for irradiance map and auto-exposure */
    float sh_coeffs[9][3];
    sh_project_environment(tinted, w, h, sh_coeffs);

    const int radiometric = (intensity >= 0.0f);
    float env_scale;
    if (radiometric)
    {
        env_scale = intensity;
        gpu->env_intensity = 1.0f; /* marker only; shaders must not scale by it */
    }
    else
    {
        /* Auto-exposure from SH DC coefficient (matches Vulkan viewer).
         * SH band-0 DC = average irradiance * Y00 (0.282095).
         * avg_irr = DC / Y00 * (PI convolution factor 0.886227). */
        double avg_lum = 0.2126 * sh_coeffs[0][0] + 0.7152 * sh_coeffs[0][1] + 0.0722 * sh_coeffs[0][2];
        double avg_irr = 0.886227 * avg_lum;
        env_scale = (avg_irr > 0.001) ? (float)(3.14159265 / avg_irr) : 1.0f;
        if (env_scale > 20.0f)
            env_scale = 20.0f;
        gpu->env_intensity = intensity;
    }
    gl_log(GL_LOG_INFO, "gpu_gles", "env %dx%d %s: tint=(%.3f,%.3f,%.3f) texel-scale=%.4f marker=%.3f", w, h,
           radiometric ? "radiometric" : "auto-exposed", tint_rgb[0], tint_rgb[1], tint_rgb[2], env_scale,
           gpu->env_intensity);

    {
        const int irr_w = 256, irr_h = 128;
        float* irr_data = (float*)malloc((size_t)irr_w * irr_h * 4 * sizeof(float));
        if (irr_data)
        {
            sh_render_irradiance(&sh_coeffs[0][0], irr_data, irr_w, irr_h);
            /* Apply the texel scale (authored radiance for a radiometric dome,
             * auto-exposure otherwise). Clamp for half-float safety, mirroring
             * the Vulkan host: a 5000-nit dome times a bright HDR texel
             * overflows fp16 (max 65504) to +inf, and +inf propagates through
             * the whole irradiance map. */
            for (int i = 0; i < irr_w * irr_h; i++)
            {
                irr_data[i * 4 + 0] = fminf(irr_data[i * 4 + 0] * env_scale, 60000.0f);
                irr_data[i * 4 + 1] = fminf(irr_data[i * 4 + 1] * env_scale, 60000.0f);
                irr_data[i * 4 + 2] = fminf(irr_data[i * 4 + 2] * env_scale, 60000.0f);
            }
            glGenTextures(1, &gpu->irr_map);
            glBindTexture(GL_TEXTURE_2D, gpu->irr_map);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, irr_w, irr_h, 0, GL_RGBA, GL_FLOAT, irr_data);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            free(irr_data);
            gl_log(GL_LOG_INFO, "gpu_gles", "SH irradiance map generated (%dx%d)", irr_w, irr_h);
        }
    }

    /* Convert RGB float to RGBA float with the texel scale, half-float clamped
     * for the same reason as the irradiance map above. */
    float* rgba_data = (float*)malloc((size_t)w * (size_t)h * 4 * sizeof(float));
    if (!rgba_data)
    {
        free(tinted);
        return 0;
    }
    for (int i = 0; i < w * h; i++)
    {
        rgba_data[i * 4 + 0] = fminf(tinted[i * 3 + 0] * env_scale, 60000.0f);
        rgba_data[i * 4 + 1] = fminf(tinted[i * 3 + 1] * env_scale, 60000.0f);
        rgba_data[i * 4 + 2] = fminf(tinted[i * 3 + 2] * env_scale, 60000.0f);
        rgba_data[i * 4 + 3] = 1.0f;
    }
    free(tinted);

    /* Upload as RGBA16F texture with mip chain */
    glGenTextures(1, &gpu->env_map);
    glBindTexture(GL_TEXTURE_2D, gpu->env_map);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA, GL_FLOAT, rgba_data);

    GLenum err = glGetError();
    if (err != GL_NO_ERROR)
    {
        gl_log(GL_LOG_WARN, "gpu_gles", "env map upload error 0x%x, falling back to RGBA8", err);
        /* Fallback: convert to RGBA8 */
        unsigned char* ldr = (unsigned char*)malloc((size_t)w * (size_t)h * 4);
        if (ldr)
        {
            for (int i = 0; i < w * h * 4; i++)
            {
                float v = rgba_data[i];
                /* Simple Reinhard tonemap for LDR fallback */
                v = v / (1.0f + v);
                ldr[i] = (unsigned char)(v * 255.0f + 0.5f);
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, ldr);
            free(ldr);
        }
    }
    free(rgba_data);

    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    /* Count mip levels */
    int mips = 1;
    int mw = w, mh = h;
    while (mw > 1 || mh > 1)
    {
        mw /= 2;
        mh /= 2;
        mips++;
    }
    gpu->env_mip_levels = mips;

    gpu->allocated_bytes += (uint64_t)w * (uint64_t)h * 8; /* RGBA16F = 8 bytes/pixel */

    /* Generate BRDF integration LUT (128x128 RG float) */
    const int lut_size = 128;
    float* lut_data = (float*)malloc((size_t)lut_size * (size_t)lut_size * 2 * sizeof(float));
    if (!lut_data)
        return 0;

    generate_brdf_lut(lut_data, lut_size);

    glGenTextures(1, &gpu->brdf_lut);
    glBindTexture(GL_TEXTURE_2D, gpu->brdf_lut);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, lut_size, lut_size, 0, GL_RG, GL_FLOAT, lut_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    gpu->allocated_bytes += (uint64_t)lut_size * (uint64_t)lut_size * 4;
    free(lut_data);

    glBindTexture(GL_TEXTURE_2D, 0);
    gpu->has_ibl = 1;

    /* Compile env background shader for skybox rendering */
    {
        GLuint vs = compile_shader(GL_VERTEX_SHADER, k_env_bg_vert_gles);
        GLuint fs = compile_shader(GL_FRAGMENT_SHADER, k_env_bg_frag_gles);
        if (vs && fs)
        {
            gpu->env_bg_program = link_program(vs, fs);
            /* link_program owns both shaders on success and failure. */
            vs = fs = 0;
            if (gpu->env_bg_program)
            {
                glGenVertexArrays(1, &gpu->env_bg_vao);
                gl_log(GL_LOG_INFO, "gpu_gles", "env background shader compiled");
            }
        }
        if (vs)
            glDeleteShader(vs);
        if (fs)
            glDeleteShader(fs);
    }

    gl_log(GL_LOG_INFO, "gpu_gles", "IBL loaded (%d mip levels, 128x128 BRDF LUT)", mips);
    return 1;
}

int gpu_load_environment_tinted_intensity(Gpu* gpu, const char* hdr_path, float intensity, const float tint[3])
{
    if (!gpu || !hdr_path)
        return 0;

    /* Load HDR image via stb_image */
    int w, h, channels;
    float* hdr_data = stbi_loadf(hdr_path, &w, &h, &channels, 3);
    if (!hdr_data)
    {
        gl_log(GL_LOG_ERROR, "gpu_gles", "failed to load HDR: %s", hdr_path);
        return 0;
    }
    gl_log(GL_LOG_INFO, "gpu_gles", "loading HDR environment %dx%d from %s", w, h, hdr_path);
    int ok = env_build_from_rgb(gpu, hdr_data, w, h, intensity, tint);
    stbi_image_free(hdr_data);
    return ok;
}

int gpu_load_environment_intensity(Gpu* gpu, const char* hdr_path, float intensity)
{
    return gpu_load_environment_tinted_intensity(gpu, hdr_path, intensity, NULL);
}

int gpu_load_environment_uniform(Gpu* gpu, const float rgb[3], float intensity)
{
    if (!gpu)
        return 0;
    /* 32x16 rather than 1x1: the equirect map is also the specular source, and
     * the shader samples it at textureLod(roughness * (mips-1)), so it needs a
     * real mip chain to sample. Every texel is the same color, so the only
     * cost is the SH projection's 512 iterations -- and 16 rows keep its
     * midpoint quadrature of sin(theta) within ~0.1% of the analytic 4*pi, so
     * the DC band lands on the true pi*radiance irradiance.
     *
     * The color rides in as the `tint` rather than being baked into the texels
     * here, so a textureless dome and a textured one take byte-identical code
     * paths through env_build_from_rgb -- including the clamp and the SH
     * projection order. */
    float px[32 * 16 * 3];
    for (int i = 0; i < 32 * 16 * 3; i++)
        px[i] = 1.0f;
    gl_log(GL_LOG_INFO, "gpu_gles", "uniform environment rgb=(%.3f,%.3f,%.3f) intensity=%.3f", rgb ? rgb[0] : 1.0f,
           rgb ? rgb[1] : 1.0f, rgb ? rgb[2] : 1.0f, intensity);
    return env_build_from_rgb(gpu, px, 32, 16, intensity, rgb);
}

void gpu_draw_env_background(Gpu* gpu, const float view_inv[16], const float proj_inv[16])
{
    if (!gpu || !gpu->has_ibl || !gpu->env_bg_program)
        return;

    GLboolean cull_face = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    glUseProgram(gpu->env_bg_program);
    glBindVertexArray(gpu->env_bg_vao);

    float view_inv_gl[16], proj_inv_gl[16];
    mat4_row_major_to_gl(view_inv_gl, view_inv);
    mat4_row_major_to_gl(proj_inv_gl, proj_inv);

    GLint loc = glGetUniformLocation(gpu->env_bg_program, "u_viewInv");
    if (loc >= 0)
        glUniformMatrix4fv(loc, 1, GL_FALSE, view_inv_gl);

    loc = glGetUniformLocation(gpu->env_bg_program, "u_projInv");
    if (loc >= 0)
        glUniformMatrix4fv(loc, 1, GL_FALSE, proj_inv_gl);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gpu->env_map);
    loc = glGetUniformLocation(gpu->env_bg_program, "u_envMap");
    if (loc >= 0)
        glUniform1i(loc, 0);

    loc = glGetUniformLocation(gpu->env_bg_program, "u_envMipLevels");
    if (loc >= 0)
        glUniform1i(loc, gpu->env_mip_levels);

    loc = glGetUniformLocation(gpu->env_bg_program, "u_tone");
    if (loc >= 0)
    {
        glUniform4f(loc, gpu->tone_exposure_scale, gpu->tone_sky_scale, gpu->tone_white_point, (float)gpu->tone_flags);
    }

    loc = glGetUniformLocation(gpu->env_bg_program, "u_envIntensity");
    if (loc >= 0)
        glUniform1f(loc, gpu->env_intensity);

    /* Must match the surface shader's envMapUV exactly, or the visible sky
     * and the sky the surfaces are lit by rotate apart. */
    loc = glGetUniformLocation(gpu->env_bg_program, "u_envRotation");
    if (loc >= 0)
        glUniform1f(loc, gpu->env_rotation_deg);

    glDrawArrays(GL_TRIANGLES, 0, 3);

    /* Restore state for geometry rendering */
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    if (cull_face)
        glEnable(GL_CULL_FACE);
    glBindVertexArray(0);
}

/* ---- Depth-AOV pack + readback (see gpu.h) ---- */

int gpu_depth_pack_read(Gpu* gpu, unsigned depth_tex, int out_w, int out_h, unsigned char* out_rgba8)
{
    if (!gpu || !depth_tex || out_w <= 0 || out_h <= 0 || !out_rgba8)
        return 0;

    /* Lazy one-time program + empty VAO (env-background pattern). A compile
     * or link failure leaves everything zero and the call fails closed. */
    if (!gpu->depth_pack_program)
    {
        GLuint vs = compile_shader(GL_VERTEX_SHADER, k_depth_pack_vert_gles);
        GLuint fs = compile_shader(GL_FRAGMENT_SHADER, k_depth_pack_frag_gles);
        if (vs && fs)
        {
            gpu->depth_pack_program = link_program(vs, fs);
            /* link_program owns both shaders on success and failure. */
            vs = fs = 0;
            if (gpu->depth_pack_program)
            {
                glGenVertexArrays(1, &gpu->depth_pack_vao);
                gl_log(GL_LOG_INFO, "gpu_gles", "depth pack shader compiled");
            }
        }
        if (vs)
            glDeleteShader(vs);
        if (fs)
            glDeleteShader(fs);
        if (!gpu->depth_pack_program)
            return 0;
    }

    /* Save the caller's render target + viewport (gpu_shadow_begin pattern). */
    GLint saved_fbo = 0, saved_vp[4] = { 0, 0, 0, 0 };
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &saved_fbo);
    glGetIntegerv(GL_VIEWPORT, saved_vp);

    /* Lazy RGBA8 target, re-allocated when the requested size changes. */
    if (!gpu->depth_pack_fbo)
        glGenFramebuffers(1, &gpu->depth_pack_fbo);
    if (!gpu->depth_pack_tex || gpu->depth_pack_w != out_w || gpu->depth_pack_h != out_h)
    {
        if (!gpu->depth_pack_tex)
            glGenTextures(1, &gpu->depth_pack_tex);
        glBindTexture(GL_TEXTURE_2D, gpu->depth_pack_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, out_w, out_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, gpu->depth_pack_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gpu->depth_pack_tex, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            gl_log(GL_LOG_ERROR, "gpu_gles", "depth pack framebuffer incomplete");
            glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)saved_fbo);
            return 0;
        }
        gpu->depth_pack_w = out_w;
        gpu->depth_pack_h = out_h;
    }
    else
    {
        glBindFramebuffer(GL_FRAMEBUFFER, gpu->depth_pack_fbo);
    }

    glViewport(0, 0, out_w, out_h);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    GLboolean cull_face = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    glUseProgram(gpu->depth_pack_program);
    glBindVertexArray(gpu->depth_pack_vao);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, (GLuint)depth_tex);
    /* Plain (non-shadow) depth sampling: the texel value lands in .r. Force
     * compare mode off in case a future embedder shares the texture. */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    GLint loc = glGetUniformLocation(gpu->depth_pack_program, "u_depth");
    if (loc >= 0)
        glUniform1i(loc, 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);

    glReadPixels(0, 0, out_w, out_h, GL_RGBA, GL_UNSIGNED_BYTE, out_rgba8);

    /* Restore the caller's target, viewport, and raster state. */
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    if (cull_face)
        glEnable(GL_CULL_FACE);
    glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)saved_fbo);
    glViewport(saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3]);
    return glGetError() == GL_NO_ERROR;
}

/* ---- SSAO (see gpu.h) ---- */

/* A fixed, deterministic hemisphere kernel.
 *
 * Points are pushed toward the origin by a quadratic ramp so most samples land
 * near the shaded point: occlusion within a few centimetres of a surface is
 * what reads as contact darkening, and spending samples out at the radius
 * mostly measures things the range check then fades away again.
 *
 * Generated once from a fixed table rather than rand(): ovgl renders the same
 * frame to the same bytes twice, and a kernel that varied per process would
 * cost that for nothing -- there is no temporal accumulation here to decorrelate
 * against. */
static void gpu_ssao_build_kernel(Gpu* gpu)
{
    if (gpu->ssao_kernel_built)
        return;
    /* Low-discrepancy-ish fixed directions: golden-angle spiral over the
     * hemisphere, which spreads far more evenly than sampled uniform noise at
     * only 16 points. */
    for (int i = 0; i < GPU_SSAO_KERNEL_SIZE; i++)
    {
        /* R2 low-discrepancy sequence: the two dimensions are mutually
         * irrational, so direction and radius are effectively DECOUPLED. That
         * matters -- deriving both from the loop index put every tangential
         * direction at full radius, which is the exact pairing that grazes a
         * curved surface. */
        const float u1 = fmodf((float)i * 0.7548776662f + 0.5f, 1.0f);
        const float u2 = fmodf((float)i * 0.5698402909f + 0.5f, 1.0f);
        /* Cosine-weighted hemisphere: z = sqrt(1-u1) concentrates samples
         * around the normal, which is where the AO integrand actually has its
         * weight, so 16 samples go further than a uniform set would. */
        const float z = sqrtf(1.0f - u1);
        const float r = sqrtf(u1);
        const float phi = 6.28318530718f * u2;
        /* Radius ramp, floored so no sample degenerates to the origin. */
        const float t = ((float)i + 0.5f) / (float)GPU_SSAO_KERNEL_SIZE;
        const float scale = 0.3f + 0.7f * t * t;
        gpu->ssao_kernel[i * 3 + 0] = cosf(phi) * r * scale;
        gpu->ssao_kernel[i * 3 + 1] = sinf(phi) * r * scale;
        gpu->ssao_kernel[i * 3 + 2] = z * scale;
    }
    gpu->ssao_kernel_built = 1;
}

static int gpu_ssao_ensure_targets(Gpu* gpu, int w, int h)
{
    if (gpu->ssao_fbo && gpu->ssao_w == w && gpu->ssao_h == h)
        return 1;
    if (gpu->ssao_tex)
    {
        glDeleteTextures(1, &gpu->ssao_tex);
        gpu->ssao_tex = 0;
    }
    if (gpu->ssao_blur_tex)
    {
        glDeleteTextures(1, &gpu->ssao_blur_tex);
        gpu->ssao_blur_tex = 0;
    }
    if (!gpu->ssao_fbo)
        glGenFramebuffers(1, &gpu->ssao_fbo);
    if (!gpu->ssao_blur_fbo)
        glGenFramebuffers(1, &gpu->ssao_blur_fbo);

    GLuint* texs[2] = { &gpu->ssao_tex, &gpu->ssao_blur_tex };
    GLuint fbos[2] = { gpu->ssao_fbo, gpu->ssao_blur_fbo };
    for (int i = 0; i < 2; i++)
    {
        glGenTextures(1, texs[i]);
        glBindTexture(GL_TEXTURE_2D, *texs[i]);
        /* R8: AO is one scalar in [0,1]. 8 bits is what the frame is quantized
         * to anyway, and this is the buffer the material pass samples every
         * fragment. */
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);
        /* LINEAR on the blurred result: it is sampled at HALF the resolution of
         * the pass that reads it, so nearest would show the half-res grid as
         * stair-stepping along every crevice. */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindFramebuffer(GL_FRAMEBUFFER, fbos[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *texs[i], 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            gl_log(GL_LOG_ERROR, "gpu_gles", "SSAO framebuffer %d incomplete", i);
            glBindTexture(GL_TEXTURE_2D, 0);
            return 0;
        }
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    gpu->ssao_w = w;
    gpu->ssao_h = h;
    return 1;
}

int gpu_ssao_compute(Gpu* gpu,
                     unsigned depth_tex,
                     unsigned normal_tex,
                     int out_w,
                     int out_h,
                     const float proj[16],
                     const float proj_inv[16],
                     const float view[16],
                     float radius,
                     float bias,
                     float intensity)
{
    if (!gpu || !depth_tex || !normal_tex || out_w <= 0 || out_h <= 0)
        return 0;
    if (!proj || !proj_inv || !view)
        return 0;

    if (!gpu->ssao_program)
    {
        GLuint vs = compile_shader(GL_VERTEX_SHADER, k_ssao_vert_gles);
        GLuint fs = compile_shader(GL_FRAGMENT_SHADER, k_ssao_frag_gles);
        if (vs && fs)
        {
            gpu->ssao_program = link_program(vs, fs);
            vs = fs = 0;
        }
        if (vs)
            glDeleteShader(vs);
        if (fs)
            glDeleteShader(fs);
        GLuint bvs = compile_shader(GL_VERTEX_SHADER, k_ssao_vert_gles);
        GLuint bfs = compile_shader(GL_FRAGMENT_SHADER, k_ssao_blur_frag_gles);
        if (bvs && bfs)
        {
            gpu->ssao_blur_program = link_program(bvs, bfs);
            bvs = bfs = 0;
        }
        if (bvs)
            glDeleteShader(bvs);
        if (bfs)
            glDeleteShader(bfs);
        if (!gpu->ssao_program || !gpu->ssao_blur_program)
        {
            /* Fail CLOSED and once: leave ssao_bound_tex at 0 so the material
             * pass forces strength to 0 and renders exactly as it did before. */
            gl_log(GL_LOG_ERROR, "gpu_gles", "SSAO shaders failed to build; AO disabled");
            return 0;
        }
        if (!gpu->ssao_vao)
            glGenVertexArrays(1, &gpu->ssao_vao);
        gl_log(GL_LOG_INFO, "gpu_gles", "SSAO shaders compiled");
    }
    if (!gpu_ssao_ensure_targets(gpu, out_w, out_h))
        return 0;
    gpu_ssao_build_kernel(gpu);

    GLint saved_fbo = 0, saved_vp[4] = { 0, 0, 0, 0 };
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &saved_fbo);
    glGetIntegerv(GL_VIEWPORT, saved_vp);

    float proj_gl[16], proj_inv_gl[16], view_gl[16];
    mat4_row_major_to_gl(proj_gl, proj);
    mat4_row_major_to_gl(proj_inv_gl, proj_inv);
    mat4_row_major_to_gl(view_gl, view);
    /* Rotation part of the view matrix, column-major for GL: turns the
     * prepass's WORLD normal into the view space the estimate works in. */
    const float view_rot[9] = {
        view_gl[0], view_gl[1], view_gl[2], view_gl[4], view_gl[5], view_gl[6], view_gl[8], view_gl[9], view_gl[10],
    };

    GLboolean cull_face = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    glViewport(0, 0, out_w, out_h);
    glBindVertexArray(gpu->ssao_vao);

    /* --- estimate --- */
    glBindFramebuffer(GL_FRAMEBUFFER, gpu->ssao_fbo);
    glUseProgram(gpu->ssao_program);
    GLint loc;
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, (GLuint)depth_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, (GLuint)normal_tex);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_depth")) >= 0)
        glUniform1i(loc, 0);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_normal")) >= 0)
        glUniform1i(loc, 1);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_proj")) >= 0)
        glUniformMatrix4fv(loc, 1, GL_FALSE, proj_gl);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_projInv")) >= 0)
        glUniformMatrix4fv(loc, 1, GL_FALSE, proj_inv_gl);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_viewRot")) >= 0)
        glUniformMatrix3fv(loc, 1, GL_FALSE, view_rot);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_kernel[0]")) >= 0)
        glUniform3fv(loc, GPU_SSAO_KERNEL_SIZE, gpu->ssao_kernel);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_radius")) >= 0)
        glUniform1f(loc, radius);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_bias")) >= 0)
        glUniform1f(loc, bias);
    if ((loc = glGetUniformLocation(gpu->ssao_program, "u_intensity")) >= 0)
        glUniform1f(loc, intensity);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    /* --- blur --- */
    glBindFramebuffer(GL_FRAMEBUFFER, gpu->ssao_blur_fbo);
    glUseProgram(gpu->ssao_blur_program);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gpu->ssao_tex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, (GLuint)depth_tex);
    if ((loc = glGetUniformLocation(gpu->ssao_blur_program, "u_ssao")) >= 0)
        glUniform1i(loc, 0);
    if ((loc = glGetUniformLocation(gpu->ssao_blur_program, "u_depth")) >= 0)
        glUniform1i(loc, 1);
    if ((loc = glGetUniformLocation(gpu->ssao_blur_program, "u_projInv")) >= 0)
        glUniformMatrix4fv(loc, 1, GL_FALSE, proj_inv_gl);
    if ((loc = glGetUniformLocation(gpu->ssao_blur_program, "u_texel")) >= 0)
        glUniform2f(loc, 1.0f / (float)out_w, 1.0f / (float)out_h);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glBindVertexArray(0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    if (cull_face)
        glEnable(GL_CULL_FACE);
    glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)saved_fbo);
    glViewport(saved_vp[0], saved_vp[1], saved_vp[2], saved_vp[3]);
    if (glGetError() != GL_NO_ERROR)
        return 0;
    return 1;
}

void gpu_set_ssao(Gpu* gpu, unsigned tex, float strength)
{
    if (!gpu)
        return;
    gpu->ssao_bound_tex = (GLuint)tex;
    gpu->ssao_strength = strength;
    /* Observable state, like gpu_shadow_clear: refresh the live program now
     * rather than depending on a later pipeline bind. */
    if (gpu->current_pipeline)
    {
        GLint current_program = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current_program);
        if ((GLuint)current_program == gpu->current_pipeline->program)
            gpu_apply_ssao_uniforms(gpu, gpu->current_pipeline);
    }
}

unsigned gpu_ssao_result(Gpu* gpu)
{
    return gpu ? gpu->ssao_blur_tex : 0u;
}

void gpu_destroy_environment(Gpu* gpu)
{
    if (!gpu)
        return;
    if (gpu->env_map)
    {
        glDeleteTextures(1, &gpu->env_map);
        gpu->env_map = 0;
    }
    if (gpu->irr_map)
    {
        glDeleteTextures(1, &gpu->irr_map);
        gpu->irr_map = 0;
    }
    if (gpu->brdf_lut)
    {
        glDeleteTextures(1, &gpu->brdf_lut);
        gpu->brdf_lut = 0;
    }
    if (gpu->env_bg_program)
    {
        glDeleteProgram(gpu->env_bg_program);
        gpu->env_bg_program = 0;
    }
    if (gpu->env_bg_vao)
    {
        glDeleteVertexArrays(1, &gpu->env_bg_vao);
        gpu->env_bg_vao = 0;
    }
    gpu->has_ibl = 0;
    gpu->env_intensity = 0.0f;
    gpu->env_owns_lighting = 0;
    gpu->env_rotation_deg = 0.0f;
}

/* ---- Debug ---- */

void gpu_set_debug_mode(Gpu* gpu, int mode)
{
    if (gpu)
        gpu->debug_mode = mode;
}

/* ---- Screenshot ---- */

int gpu_screenshot(Gpu* gpu, const char* path)
{
    if (!gpu || !path)
        return 0;

    int w = gpu->width, h = gpu->height;

    /* GLES only guarantees GL_RGBA for glReadPixels; read RGBA then strip alpha */
    unsigned char* rgba = (unsigned char*)malloc((size_t)w * (size_t)h * 4);
    if (!rgba)
        return 0;

    glFinish(); /* ensure all rendering is complete */
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgba);

    FILE* f = fopen(path, "wb");
    if (!f)
    {
        free(rgba);
        return 0;
    }

    fprintf(f, "P6\n%d %d\n255\n", w, h);
    /* Flip vertically (OpenGL origin is bottom-left), strip alpha */
    for (int y = h - 1; y >= 0; y--)
    {
        for (int x = 0; x < w; x++)
        {
            fwrite(rgba + (y * w + x) * 4, 1, 3, f);
        }
    }

    fclose(f);
    free(rgba);
    gl_log(GL_LOG_INFO, "gpu_gles", "screenshot saved to %s", path);
    return 1;
}

/* ---- Text overlay ---- */

/*
 * Generate a signed-distance-field font texture from the 8x16 bitmap font.
 * Each glyph cell is CELL_W x CELL_H in the output texture.  The source
 * bitmap is rasterized at output resolution, then a distance transform
 * produces smooth edges at any display scale.
 */
#define SDF_SCALE 4 /* output pixels per source pixel */
#define SDF_SPREAD 8 /* search radius in output pixels */
#define SDF_CELL_W (8 * SDF_SCALE) /* 32 */
#define SDF_CELL_H (16 * SDF_SCALE) /* 64 */
#define SDF_COLS 16
#define SDF_ROWS 8
#define SDF_TEX_W (SDF_COLS * SDF_CELL_W) /* 512 */
#define SDF_TEX_H (SDF_ROWS * SDF_CELL_H) /* 512 */

static int font_bit(int ch, int bx, int by)
{
    if (ch < FONT_FIRST_CHAR || ch >= FONT_FIRST_CHAR + FONT_NUM_CHARS)
        return 0;
    if (bx < 0 || bx >= 8 || by < 0 || by >= 16)
        return 0;
    return (font8x16_data[ch - FONT_FIRST_CHAR][by] >> (7 - bx)) & 1;
}

/* Bilinearly sample the source 8x16 bitmap at a continuous coordinate.
 * Returns 0.0–1.0 with smooth gradients at every edge. */
static float font_bilerp(int ch, float sx, float sy)
{
    /* Shift to texel centers: source pixel (bx,by) has center at (bx+0.5, by+0.5) */
    float fx = sx - 0.5f, fy = sy - 0.5f;
    int x0 = (int)floorf(fx), y0 = (int)floorf(fy);
    float tx = fx - (float)x0, ty = fy - (float)y0;

    float s00 = (float)font_bit(ch, x0, y0);
    float s10 = (float)font_bit(ch, x0 + 1, y0);
    float s01 = (float)font_bit(ch, x0, y0 + 1);
    float s11 = (float)font_bit(ch, x0 + 1, y0 + 1);

    return (s00 * (1 - tx) * (1 - ty) + s10 * tx * (1 - ty) + s01 * (1 - tx) * ty + s11 * tx * ty);
}

/* Rasterize one glyph at 4x with bilinear interpolation of the source
 * bitmap.  This creates a smooth 1-source-pixel-wide gradient at every
 * edge — wide enough for the shader smoothstep to anti-alias, narrow
 * enough to preserve all thin strokes. */
static void sdf_glyph(int ch, unsigned char* out, int tex_w)
{
    int cell_x = (ch % SDF_COLS) * SDF_CELL_W;
    int cell_y = (ch / SDF_COLS) * SDF_CELL_H;

    for (int py = 0; py < SDF_CELL_H; py++)
    {
        for (int px = 0; px < SDF_CELL_W; px++)
        {
            /* Map output texel center to source glyph coordinates */
            float sx = ((float)px + 0.5f) / (float)SDF_SCALE;
            float sy = ((float)py + 0.5f) / (float)SDF_SCALE;
            float v = font_bilerp(ch, sx, sy);
            out[(cell_y + py) * tex_w + cell_x + px] = (unsigned char)(v * 255.0f + 0.5f);
        }
    }
}

int gpu_overlay_init(Gpu* gpu)
{
    if (!gpu)
        return 0;

    /* Build SDF font texture (512x512 R8). */
    unsigned char* pixels = (unsigned char*)calloc(SDF_TEX_W * SDF_TEX_H, 1);
    if (!pixels)
        return 0;

    for (int ch = FONT_FIRST_CHAR; ch < FONT_FIRST_CHAR + FONT_NUM_CHARS; ch++)
        sdf_glyph(ch, pixels, SDF_TEX_W);

    glGenTextures(1, &gpu->overlay_font_tex);
    glBindTexture(GL_TEXTURE_2D, gpu->overlay_font_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, SDF_TEX_W, SDF_TEX_H, 0, GL_RED, GL_UNSIGNED_BYTE, pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    free(pixels);

    /* Create overlay pipeline */
    GLuint vert = compile_shader(GL_VERTEX_SHADER, k_overlay_vert_gles);
    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, k_overlay_frag_gles);
    if (!vert || !frag)
    {
        if (vert)
            glDeleteShader(vert);
        if (frag)
            glDeleteShader(frag);
        return 0;
    }

    /* link_program owns both shaders on success and failure. */
    gpu->overlay_pipeline.program = link_program(vert, frag);
    if (!gpu->overlay_pipeline.program)
        return 0;

    gpu->overlay_pipeline.loc_screen_size = glGetUniformLocation(gpu->overlay_pipeline.program, "u_screen_size");

    /* Set font texture sampler uniform */
    glUseProgram(gpu->overlay_pipeline.program);
    GLint loc = glGetUniformLocation(gpu->overlay_pipeline.program, "fontTex");
    if (loc >= 0)
        glUniform1i(loc, 0);
    glUseProgram(0);

    glGenVertexArrays(1, &gpu->overlay_pipeline.vao);
    glGenBuffers(1, &gpu->overlay_vbo);

    gpu->overlay_nchars = 0;

    return 1;
}

void gpu_overlay_shutdown(Gpu* gpu)
{
    if (!gpu)
        return;
    if (gpu->overlay_font_tex)
        glDeleteTextures(1, &gpu->overlay_font_tex);
    if (gpu->overlay_pipeline.program)
        glDeleteProgram(gpu->overlay_pipeline.program);
    if (gpu->overlay_pipeline.vao)
        glDeleteVertexArrays(1, &gpu->overlay_pipeline.vao);
    if (gpu->overlay_vbo)
        glDeleteBuffers(1, &gpu->overlay_vbo);
    gpu->overlay_font_tex = 0;
    gpu->overlay_pipeline.program = 0;
    gpu->overlay_pipeline.vao = 0;
    gpu->overlay_vbo = 0;
    gpu->overlay_nchars = 0;
}

void gpu_overlay_text(Gpu* gpu, float x, float y, float scale, float r, float g, float b, float a, const char* text)
{
    if (!gpu || !text)
        return;

    float cx = x, cy = y;
    float char_w = 8.0f * scale;
    float char_h = 16.0f * scale;

    while (*text && gpu->overlay_nchars < MAX_OVERLAY_CHARS)
    {
        unsigned char ch = (unsigned char)*text++;
        if (ch >= 128)
            ch = '?';

        float u0 = (float)(ch % SDF_COLS) * (float)SDF_CELL_W / (float)SDF_TEX_W;
        float v0 = (float)(ch / SDF_COLS) * (float)SDF_CELL_H / (float)SDF_TEX_H;
        float u1 = u0 + (float)SDF_CELL_W / (float)SDF_TEX_W;
        float v1 = v0 + (float)SDF_CELL_H / (float)SDF_TEX_H;

        float* dst = &gpu->overlay_verts[gpu->overlay_nchars * OVERLAY_VERTS_PER_CHAR * OVERLAY_FLOATS_PER_VERT];

        /* Pixel-snap positions for crisp rendering */
        float x0 = floorf(cx);
        float y0 = floorf(cy);
        float x1 = floorf(cx + char_w);
        float y1 = floorf(cy + char_h);

        /* Triangle 1 */
        float verts[6][8] = {
            { x0, y0, u0, v0, r, g, b, a },
            { x1, y0, u1, v0, r, g, b, a },
            { x1, y1, u1, v1, r, g, b, a },
            /* Triangle 2 */
            { x0, y0, u0, v0, r, g, b, a },
            { x1, y1, u1, v1, r, g, b, a },
            { x0, y1, u0, v1, r, g, b, a },
        };
        memcpy(dst, verts, sizeof(verts));

        cx += char_w;
        gpu->overlay_nchars++;
    }
}

void gpu_overlay_rect(Gpu* gpu, float x, float y, float w, float h, float r, float g, float b, float a)
{
    if (!gpu || gpu->overlay_nchars + 1 > MAX_OVERLAY_CHARS)
        return;

    /* 6 vertices, UV = (-1,-1) signals solid fill to the fragment shader */
    float* base = gpu->overlay_verts + gpu->overlay_nchars * OVERLAY_VERTS_PER_CHAR * OVERLAY_FLOATS_PER_VERT;

    float verts[6][8] = {
        { x, y, -1, -1, r, g, b, a }, { x + w, y, -1, -1, r, g, b, a },     { x + w, y + h, -1, -1, r, g, b, a },
        { x, y, -1, -1, r, g, b, a }, { x + w, y + h, -1, -1, r, g, b, a }, { x, y + h, -1, -1, r, g, b, a },
    };
    memcpy(base, verts, sizeof(verts));
    gpu->overlay_nchars++;
}

void gpu_overlay_flush(Gpu* gpu, int screen_width, int screen_height)
{
    if (!gpu || gpu->overlay_nchars == 0 || screen_width <= 0 || screen_height <= 0)
        return;

    /* Save state */
    GLboolean depth_test, cull_face;
    glGetBooleanv(GL_DEPTH_TEST, &depth_test);
    glGetBooleanv(GL_CULL_FACE, &cull_face);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); /* Y-flip in shader reverses triangle winding */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int nverts = gpu->overlay_nchars * OVERLAY_VERTS_PER_CHAR;
    GLsizeiptr data_size = (GLsizeiptr)(nverts * OVERLAY_FLOATS_PER_VERT * sizeof(float));

    /* Upload vertex data */
    glBindBuffer(GL_ARRAY_BUFFER, gpu->overlay_vbo);
    glBufferData(GL_ARRAY_BUFFER, data_size, gpu->overlay_verts, GL_STREAM_DRAW);

    /* Bind overlay pipeline */
    glUseProgram(gpu->overlay_pipeline.program);
    glBindVertexArray(gpu->overlay_pipeline.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu->overlay_vbo);

    /* Attributes: pos(2), uv(2), color(4) = 32 bytes */
    GLsizei stride = OVERLAY_FLOATS_PER_VERT * (GLsizei)sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void*)(4 * sizeof(float)));

    /* Set screen size uniform */
    if (gpu->overlay_pipeline.loc_screen_size >= 0)
        glUniform2f(gpu->overlay_pipeline.loc_screen_size, (float)screen_width, (float)screen_height);

    /* Bind font texture */
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, gpu->overlay_font_tex);

    glDrawArrays(GL_TRIANGLES, 0, nverts);

    /* Restore state */
    glDisable(GL_BLEND);
    if (depth_test)
        glEnable(GL_DEPTH_TEST);
    if (cull_face)
        glEnable(GL_CULL_FACE);

    glBindVertexArray(0);
    glUseProgram(0);

    gpu->overlay_nchars = 0;
}

/* ---- Diagnostics ---- */

uint64_t gpu_get_allocated_memory(Gpu* gpu)
{
    return gpu ? gpu->allocated_bytes : 0;
}
