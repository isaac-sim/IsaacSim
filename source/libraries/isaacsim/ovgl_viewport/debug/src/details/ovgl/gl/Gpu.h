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
 * gpu.h — Render Hardware Interface (RHI)
 *
 * Thin abstraction over OpenGL ES 3.2 for the OpenGL renderer.
 * Same interface as the Vulkan viewer's gpu.h, with RT/DLSS
 * functions compiled as no-ops.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ---- Opaque handles ---- */

    typedef struct Gpu Gpu;
    typedef struct GpuBuffer_s* GpuBuffer;
    typedef struct GpuPipeline_s* GpuPipeline;

    /* ---- Enums ---- */

    typedef enum
    {
        GPU_BUFFER_VERTEX = 0,
        GPU_BUFFER_INDEX = 1,
    } GpuBufferUsage;

    typedef enum
    {
        GPU_FORMAT_FLOAT3 = 0,
        GPU_FORMAT_FLOAT2 = 1,
        GPU_FORMAT_UINT = 2,
        GPU_FORMAT_FLOAT1 = 3,
        GPU_FORMAT_SNORM16X4 = 4,
        GPU_FORMAT_UNORM8X4 = 5,
    } GpuVertexFormat;

    /* ---- Descriptors ---- */

    typedef struct
    {
        GpuBufferUsage usage;
        uint64_t size;
        const void* data;
    } GpuBufferDesc;

    typedef struct
    {
        uint32_t location;
        uint32_t offset;
        GpuVertexFormat format;
    } GpuVertexAttrib;

    typedef struct
    {
        const char* vert_glsl; /* GLES: GLSL source string */
        const char* frag_glsl; /* GLES: GLSL source string */
        /* Optional tessellation stages. Both must be set or both NULL. When
         * set, the pipeline draws GL_PATCHES with `patch_vertices` CVs per
         * patch (used for BasisCurves tube rendering). */
        const char* tcs_glsl;
        const char* tes_glsl;
        uint32_t patch_vertices; /* 0 = non-tess pipeline */
        const uint32_t* vert_spv; /* ignored in GLES */
        uint32_t vert_size;
        const uint32_t* frag_spv; /* ignored in GLES */
        uint32_t frag_size;
        uint32_t push_constant_size;
        uint32_t vertex_stride;
        const GpuVertexAttrib* attribs;
        uint32_t nattribs;
    } GpuPipelineDesc;

    /* Push constant layout: MVP + model + color + eye_pos */
    typedef struct
    {
        float mvp[16];
        float model[16];
        float color[4];
        float eye_pos[3];
        float _pad_eye;
    } GpuMeshPushConstants;

    /* ---- Lifecycle ---- */

    Gpu* gpu_init(void* glfw_window, int width, int height);
    void gpu_shutdown(Gpu* gpu);
    void gpu_resize(Gpu* gpu, int width, int height);

    /* ---- Resources ---- */

    GpuBuffer gpu_create_buffer(Gpu* gpu, const GpuBufferDesc* desc);
    void gpu_destroy_buffer(Gpu* gpu, GpuBuffer buf);
    unsigned gpu_buffer_gl_handle(GpuBuffer buf); /* underlying GL buffer id (for VAO setup) */

    GpuPipeline gpu_create_pipeline(Gpu* gpu, const GpuPipelineDesc* desc);
    void gpu_destroy_pipeline(Gpu* gpu, GpuPipeline pipe);

    /* ---- Frame ---- */

    int gpu_begin_frame(Gpu* gpu);
    void gpu_end_frame(Gpu* gpu);

    /* ---- Draw commands ---- */

    void gpu_cmd_bind_pipeline(Gpu* gpu, GpuPipeline pipe);
    void gpu_cmd_bind_vertex_buffer(Gpu* gpu, GpuBuffer buf);
    void gpu_cmd_bind_index_buffer(Gpu* gpu, GpuBuffer buf);
    void gpu_cmd_push_constants(Gpu* gpu, const void* data, uint32_t size);
    void gpu_cmd_draw(Gpu* gpu, uint32_t vertex_count, uint32_t first_vertex);
    void gpu_cmd_draw_indexed(Gpu* gpu, uint32_t index_count, uint32_t first_index, int32_t vertex_offset);
    void gpu_cmd_draw_indexed_typed(
        Gpu* gpu, uint32_t index_count, uint64_t index_byte_offset, int32_t vertex_offset, int index_type_bits);

    /* Compact-PointInstancer instancing. Upload all per-instance world matrices
     * (16 floats each, column-vector row-major — see scene_instance_transform_to_model16)
     * once; then draw a prototype's geometry instance_count times, reading the matrix
     * slice starting at first_instance (GLES 3.2 has no baseInstance, so the slice is
     * selected by the instance attribute pointer offset). */
    void gpu_upload_instance_transforms(Gpu* gpu, const float* matrices16, uint32_t count);
    void gpu_cmd_draw_instanced(
        Gpu* gpu, uint32_t index_count, uint32_t first_index, uint32_t instance_count, uint32_t first_instance);
/* ---- Shared depth-atlas many-light shadows ----
 *
 * One depth texture is tiled into GPU_SHADOW_TILE_SIZE squares and each
 * shadow-casting light owns one tile, so N lights cost one texture, one
 * sampler unit and one UBO instead of N of each. Slot -> tile:
 * tx = slot % tiles_per_row, ty = slot / tiles_per_row; pixel origin
 * (tx*TILE, ty*TILE); uvScale = 1/tiles_per_row, uvOffset = (tx, ty)/tpr.
 *
 * SIZING DEVIATES FROM nanousd-opengl-renderer ON PURPOSE. The fork fixes an
 * 8192 atlas of 1024 tiles = 64 lights, which is 8192*8192*4 = 268 MB of
 * DEPTH_COMPONENT24 allocated up front. ovgl is a library: one Gpu per
 * renderer and several renderers routinely coexist in one process (that is
 * exactly why EglHeadless.c refcounts the shared EGLDisplay), and its single
 * pre-atlas map was 2048^2 = 16.8 MB. So the tile stays at 2048 -- halving it
 * to the fork's 1024 would visibly soften the one shadow ovgl already casts --
 * and the ATLAS grows on demand instead: 1 caster keeps the historical
 * 2048^2/16.8 MB exactly, 2..4 casters take 4096^2/67 MB. Raising the ceiling
 * is the two numbers below plus the VRAM to pay for it; the shader array size
 * and the C limit track them through NUSD_MAX_SHADOW_LIGHTS_STR. */
#define GPU_SHADOW_TILE_SIZE 2048
#define GPU_SHADOW_ATLAS_MAX 4096
#define GPU_SHADOW_TILES_PER_ROW (GPU_SHADOW_ATLAS_MAX / GPU_SHADOW_TILE_SIZE) /* 2 */
#define GPU_MAX_SHADOW_LIGHTS (GPU_SHADOW_TILES_PER_ROW * GPU_SHADOW_TILES_PER_ROW) /* 4 */

    /* One shadow-casting light's atlas record. std140-clean at 96 bytes (a
     * 64-byte mat4 + two 16-byte vec4s, every member 16-aligned), so the std140
     * array stride is also 96 and one C GpuShadowLight maps to exactly one GLSL
     * u_shadow[slot] element of ShadowBlock (binding 2). lightVP is row-major
     * (`mul()` memory order) to match the GLSL (std140, row_major) qualifier --
     * a UBO has no per-upload transpose flag, so the layout is declared instead. */
    typedef struct
    {
        float lightVP[16]; /* row-major proj*view into [-1,1] clip */
        float uvScaleOffset[4]; /* xy = uvScale, zw = uvOffset (tile placement) */
        float bias[4]; /* x = depth bias in WINDOW-depth units; yzw reserved */
    } GpuShadowLight;
/* `_Static_assert` is C11 only; this header is inside `extern "C"` but is
 * still compiled as C++ by Ovgl.cpp / Scene.cpp, where the spelling is
 * `static_assert`. The fork never had to spell both -- it has no C++ TU. */
#ifdef __cplusplus
    static_assert(sizeof(GpuShadowLight) == 96, "GpuShadowLight must be 96 bytes for std140 array-of-structs");
#else
_Static_assert(sizeof(GpuShadowLight) == 96, "GpuShadowLight must be 96 bytes for std140 array-of-structs");
#endif

    /* Render light `light_index`'s depth into tile `slot`. `slot_count` is the
     * number of tiles this frame will fill and is read ONLY when slot == 0, where
     * it sizes (and clears) the atlas -- every later slot inherits that sizing, so
     * uvScaleOffset stays consistent across the frame's tiles.
     *
     * `depth_bias` is in WINDOW-depth units (the shader compares window depth), so
     * the caller converts its world-space bias by the light frustum's own depth
     * span. The pre-atlas path hardcoded 0.0065 for every light, which is a
     * different world distance in every scene: measured against ovgl's own
     * whole-scene ortho fit, 30 mm on robot_ground_scene (scene radius 1.15 m)
     * but 3.68 METRES on the spot orbit scene (radius 141.42 m, because its
     * ground is a 200 x 200 UsdGeomPlane) -- nearly five times that scene's
     * entire 0.78 m vertical extent. The map was rendered every frame and then
     * biased out of existence.
     *
     * Unlike the fork this takes no `frustum_info[4]`: that parameter is the
     * legacy PCSS penumbra estimate, and the fork's own gpu_shadow_begin opens
     * with `(void)frustum_info;` under the fixed 3x3 hardware-PCF atlas. */
    int gpu_shadow_begin(Gpu* gpu, int slot, int slot_count, const float light_vp[16], int light_index, float depth_bias);
    void gpu_shadow_end(Gpu* gpu);
    /* Invalidate every cached shadow without destroying the reusable GL targets.
     * Scene/light edits that remove the shadow source must not leave the prior
     * map bound to a different light at the same array index. */
    void gpu_shadow_clear(Gpu* gpu);

    /* ---- Materials ---- */

    typedef struct
    {
        const unsigned char* pixels;
        int width;
        int height;
        int is_srgb; /* 1 = upload as GL_SRGB8_ALPHA8 (color); 0 = GL_RGBA8 (data) */
    } GpuTextureData;

    typedef struct
    {
        float base_color[4];
        float emissive_color[4];
        float metallic;
        float roughness;
        float opacity;
        float ior;
        float occlusion;
        float clearcoat;
        float clearcoat_roughness;
        float normal_scale;
        int tex_indices[8];
        int use_vertex_color;
        float udim_scale_u;
        float udim_scale_v;
        int v_flip; /* 1 → extra shader V flip (cancels the vertex-pack flip for MDL) */
        /* UsdPreviewSurface opacityThreshold. > 0 = alpha-cutout (discard
         * pixels where opacity*opacity_tex < threshold); 0 = alpha-blend
         * (no discard, output blended alpha). Mirrors the vulkan path. */
        float opacity_threshold;
        int opacity_texture_channel; /* 0/1/2/3 = r/g/b/a */
        int roughness_texture_channel; /* preserves UsdUVTexture output selection */
        int metallic_texture_channel; /* defaults to ORM's blue channel */
        float mdl_uv_transform[4]; /* xy texture scale + zw bias; zero means identity */
        float transmission_color[4];
        float transmission_weight;
        float transmission_ior;
        float _pad_d, _pad_e; /* std140 vec4 */
        int use_specular_workflow;
        float _pad_f, _pad_g, _pad_h;
        float specular_color[4];
        float roughness_tex_scale; /* sampled roughness = selected channel * scale + bias */
        float roughness_tex_bias; /* Isaac MDL RoughnessMin/Max remap */
        float _pad_i, _pad_j; /* std140 align trailing block */
    } GpuMaterialParams;

    /* Per-mesh data uploaded to one big UBO, indexed via glBindBufferRange.
     * std140-friendly: mat4s are 64-byte aligned, vec4 is 16-byte aligned.
     * Total = 64 + 64 + 16 + 16 = 160 bytes; pad in the UBO to 256-byte stride
     * to match GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT on macOS. */
    typedef struct
    {
        float mvp[16]; /* 64 B */
        float model[16]; /* 64 B */
        float color[4]; /* 16 B — .w > 0.5 = override vertex color */
        uint32_t ptex[4]; /* x = packed Ptex color offset, 0xFFFFFFFF = none */
    } GpuMeshData;

    /* Allocate the mesh-data UBO sized for `nmeshes` slices. Stored on Gpu
     * so subsequent map/bind calls find it. Returns 1 on success. */
    int gpu_alloc_mesh_buffer(Gpu* gpu, int nmeshes);
    /* Map the entire mesh-data UBO for write. Returns the base of a CPU-
     * visible region; caller writes one GpuMeshData per slot at
     * `(unsigned char*)base + slot * stride`. Use INVALIDATE_BUFFER +
     * UNSYNCHRONIZED so writes don't stall on in-flight GPU reads. */
    void* gpu_begin_mesh_writes(Gpu* gpu);
    /* Per-mesh stride (bytes) for `gpu_begin_mesh_writes` slot pointer math. */
    int gpu_mesh_stride(Gpu* gpu);
    void gpu_end_mesh_writes(Gpu* gpu);
    /* Bind one mesh's slice of the mesh UBO at binding=1 for the next draw. */
    void gpu_cmd_bind_mesh_data(Gpu* gpu, int mesh_index);
    /* Set the per-frame eye-position uniform (shared across all meshes). */
    void gpu_cmd_set_eye_pos(Gpu* gpu, const float eye[3]);

    /* Set the per-frame u_view + u_proj plain uniforms on the current
     * pipeline (Storm-style separate-matrix path used by the curve TES).
     * Matrices use the renderer's row-major convention, 16 floats. The GL
     * backend converts them to column-major uniform bytes. No-op if the
     * pipeline lacks either uniform. */
    void gpu_cmd_set_view_proj(Gpu* gpu, const float view16[16], const float proj16[16]);

    /* Set the curve TES's u_basis_id uniform: 0=bezier, 1=bspline,
     * 2=catmullRom, 3=linear. No-op if the current pipeline doesn't
     * declare u_basis_id. */
    void gpu_cmd_set_basis_id(Gpu* gpu, int basis_id);

    /* OVRTX-style exposure/tonemap scale applied by built-in shaders.
     * Values of 1,1,1,0 preserve legacy output. */
    void gpu_set_tone_mapping(Gpu* gpu, float exposure_scale, float sky_scale, float white_point_scale, uint32_t flags);
    void gpu_set_fallback_lighting(Gpu* gpu, int enabled);
    /* Whether the loaded environment is the frame's light source, as opposed to
     * merely being loaded. gpu_set_fallback_lighting cannot answer that: the
     * driver clears it whenever it uploads any light, its own synthetic headlight
     * included. The shaders keep the synthetic hemisphere fill on an environment
     * that does NOT own the frame, so a DomeLight far below the UsdLux radiance
     * convention adds its (small) real contribution instead of replacing the rig
     * with nothing. Reset by gpu_destroy_environment. */
    void gpu_set_environment_owns_lighting(Gpu* gpu, int owns);
    /* UsdLuxDomeLight `inputs:rotation`, degrees about the up axis. Applied to the
     * equirectangular longitude in BOTH the surface shader and the background
     * shader, so the sky a surface is lit by and the sky the camera sees stay the
     * same sky. A textureless dome is rotation-invariant by construction; this
     * only moves a textured one. Reset by gpu_destroy_environment. */
    void gpu_set_environment_rotation(Gpu* gpu, float degrees);

    int gpu_upload_materials(
        Gpu* gpu, const GpuMaterialParams* materials, int nmaterials, const GpuTextureData* textures, int ntextures);
    /* Replace ONLY the material state (params copy, material UBO, material
     * textures, dummy texture) and re-upload — mesh constants (mesh_ubo) and
     * ptex colors stay untouched, so SceneMesh.material_index bindings into the
     * new array remain valid against unchanged geometry. The targeted
     * material-refresh path; gpu_destroy_materials + gpu_upload_materials is
     * the full-rebuild pairing (it also drops mesh state). */
    int gpu_replace_materials(
        Gpu* gpu, const GpuMaterialParams* materials, int nmaterials, const GpuTextureData* textures, int ntextures);
    int gpu_upload_ptex_triangle_colors(Gpu* gpu, const uint32_t* colors, uint32_t count);

#define GPU_MAX_SCENE_LIGHTS 32

    typedef struct
    {
        float position[3];
        float intensity;
        float normal[3];
        int kind; /* 0 = RectLight, 1 = DistantLight, 2 = SphereLight */
        float u_axis[3];
        int normalize;
        float v_axis[3];
        float angle_deg;
        float color[3];
        float _pad;
    } GpuLight;

    int gpu_upload_lights(Gpu* gpu, const GpuLight* lights, int nlights);
    void gpu_set_authored_light_count(Gpu* gpu, int nlights);

    GpuPipeline gpu_create_material_pipeline(Gpu* gpu, const GpuPipelineDesc* desc);
    void gpu_cmd_begin_material_pass(Gpu* gpu);
    void gpu_cmd_bind_material(Gpu* gpu, int material_index);
    void gpu_cmd_bind_materials(Gpu* gpu);
    void gpu_destroy_materials(Gpu* gpu);

    /* ---- Debug ---- */

    void gpu_set_debug_mode(Gpu* gpu, int mode);

    /* ---- Screenshot ---- */

    int gpu_screenshot(Gpu* gpu, const char* path);

    /* ---- Text overlay ---- */

    int gpu_overlay_init(Gpu* gpu);
    void gpu_overlay_shutdown(Gpu* gpu);
    void gpu_overlay_text(Gpu* gpu, float x, float y, float scale, float r, float g, float b, float a, const char* text);
    void gpu_overlay_rect(Gpu* gpu, float x, float y, float w, float h, float r, float g, float b, float a);
    void gpu_overlay_flush(Gpu* gpu, int screen_width, int screen_height);

    /* ---- Environment (IBL) ---- */

    /* The SIGN of `intensity` selects the texel convention on every entry point
     * below, and the shaders branch on the same sign through u_envIntensity:
     *
     *   >= 0  RADIOMETRIC. Texels carry final linear radiance in NITS
     *         (hdr * inputs:color * inputs:intensity * 2^inputs:exposure) and
     *         u_envIntensity becomes a +1.0 marker the shaders must NOT multiply
     *         by. Surface ambient and sky then share one camera transform, so the
     *         frame is exactly linear in the authored number. This is the
     *         UsdLuxDomeLight path.
     *   <  0  LEGACY auto-exposed (pi / average-irradiance), u_envIntensity keeps
     *         the negative marker and |value| is the caller's intensity. This is
     *         the host ovgl_set_environment override path, whose 4.5/2000 tone
     *         pair is calibrated against exactly this normalization.
     *
     * All three replace whatever environment is loaded and return 1 on success. */
    int gpu_load_environment(Gpu* gpu, const char* hdr_path);
    int gpu_load_environment_intensity(Gpu* gpu, const char* hdr_path, float intensity);
    /* As above, plus UsdLuxDomeLight `inputs:color`. The tint multiplies the HDR
     * texels BEFORE the SH projection, so the irradiance map carries it too --
     * ovgl previously computed the dome color and then dropped it on the textured
     * branch, which both rendered a tinted dome untinted and made every
     * inputs:color edit re-project SH for a byte-identical result. NULL = white. */
    int gpu_load_environment_tinted_intensity(Gpu* gpu, const char* hdr_path, float intensity, const float tint[3]);
    /* Build the environment from a CONSTANT radiance instead of an HDR file: the
     * physical reading of a textureless UsdLuxDomeLight, whose surface is a
     * uniform emitter of color * intensity * 2^exposure. Textureless is the far
     * more common authoring (robot_ground_scene, franka_factory and the Isaac
     * locomotion rigs all author one), and routing it through the SAME build as a
     * decoded HDR is what stops an intensity-1.0 dome falling off a cliff that an
     * intensity-1000 dome does not. */
    int gpu_load_environment_uniform(Gpu* gpu, const float rgb[3], float intensity);
/* The no-IBL haze constants in the shader were tuned against a sim-sized scene; a scene of
 * this bounding radius reproduces them exactly (haze starts at 9 world units, saturates at 49). */
#define kHazeReferenceRadius 8.0f

    void gpu_set_environment_intensity(Gpu* gpu, float intensity);

    /* Radius of the scene's bounding sphere, in world units.  The no-IBL haze is expressed
     * relative to it; a scene of kHazeReferenceRadius reproduces the original absolute curve. */
    void gpu_set_scene_radius(Gpu* gpu, float radius);
    void gpu_draw_env_background(Gpu* gpu, const float view_inv[16], const float proj_inv[16]);
    void gpu_destroy_environment(Gpu* gpu);

    /* ---- Depth-AOV pack + readback ---- */

    /* Read back the window-space depth of `depth_tex` (a GL_DEPTH_COMPONENT
     * texture with TEXTURE_COMPARE_MODE none) into out_rgba8 (out_w*out_h*4
     * bytes) as 24-bit fixed-point depth packed into RGB (r = MSB, b = LSB;
     * a = 255). GLES has no glReadPixels(GL_DEPTH_COMPONENT), so this runs a
     * fullscreen pack pass into an internal RGBA8 target and reads that back —
     * portable across GLES 3.1/3.2 and desktop GL. The pass samples with
     * NEAREST at the output pixel centers (point sampling when depth_tex is
     * supersampled) and flips V so rows read back top-down, matching the color
     * resolve. Decode: zw = (r<<16 | g<<8 | b) / 16777215.0; zw == 1.0 is the
     * cleared far plane (no hit). Caller must have no GL_PIXEL_PACK_BUFFER
     * bound. Restores the caller's framebuffer binding + viewport. Returns 1
     * on success. */
    int gpu_depth_pack_read(Gpu* gpu, unsigned depth_tex, int out_w, int out_h, unsigned char* out_rgba8);

    /* ---- Screen-space ambient occlusion ---- */

#define GPU_SSAO_KERNEL_SIZE 16

    /* Estimate ambient occlusion from an AO prepass (`depth_tex`, a
     * GL_DEPTH_COMPONENT texture with COMPARE_MODE none, plus `normal_tex`, RGBA8
     * WORLD-space normals encoded n*0.5+0.5) into an internal R8 target of
     * out_w x out_h, then blur it. Pass HALF the render resolution: the estimate is
     * noisy by construction and the material pass samples the result with LINEAR.
     *
     * `proj` / `proj_inv` / `view` are row-major (the same convention as
     * gpu_cmd_set_view_proj); only view's rotation is used, to bring the prepass's
     * world normals into the view space the estimate works in.
     *
     * `radius` is in WORLD units and is the single knob that has to match the scene:
     * it is the distance within which geometry counts as occluding. `bias` is a
     * COSINE threshold, not a depth offset -- an occluder whose direction from the
     * shaded point lies within acos(bias) of the tangent plane is ignored, which
     * discards the grazing taps depth quantization makes meaningless. `intensity`
     * scales the result before it is subtracted from 1.
     *
     * Restores the caller's framebuffer binding and viewport. Returns 1 on success;
     * on failure it returns 0 having bound nothing, and the caller should leave
     * gpu_set_ssao at 0 so the material pass renders exactly as it did before. */
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
                         float intensity);

    /* The blurred AO texture gpu_ssao_compute produced (0 if it never ran). */
    unsigned gpu_ssao_result(Gpu* gpu);

    /* Bind `tex` on unit 11 for the material pass and scale it by `strength`.
     * strength 0 -- or tex 0 -- makes the shader take its material `ao` untouched,
     * which reproduces the pre-SSAO frame exactly. */
    void gpu_set_ssao(Gpu* gpu, unsigned tex, float strength);

    /* ---- Diagnostics ---- */

    uint64_t gpu_get_allocated_memory(Gpu* gpu);

    /* ---- RT/DLSS stubs (no-ops for GLES) ---- */

    static inline int gpu_rt_available(Gpu* gpu)
    {
        (void)gpu;
        return 0;
    }
    static inline void gpu_destroy_rt_scene(Gpu* gpu)
    {
        (void)gpu;
    }
    static inline int gpu_dlss_available(Gpu* gpu)
    {
        (void)gpu;
        return 0;
    }
    static inline void gpu_dlss_shutdown(Gpu* gpu)
    {
        (void)gpu;
    }
    static inline GpuPipeline gpu_create_shadow_pipeline(Gpu* gpu, const GpuPipelineDesc* desc)
    {
        (void)gpu;
        (void)desc;
        return NULL;
    }
    static inline void gpu_cmd_bind_shadow(Gpu* gpu)
    {
        (void)gpu;
    }
    static inline void gpu_get_render_extent(Gpu* gpu, uint32_t* w, uint32_t* h)
    {
        (void)gpu;
        (void)w;
        (void)h;
    }

#ifdef __cplusplus
}
#endif
