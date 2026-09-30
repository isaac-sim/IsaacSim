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

/* OVStage-to-OpenGL scene bridge. Reads supported scene data through the
 * public OVStage API and builds the renderer's private CPU scene model. */
#include "Scene.h"

#include "AssetReader.hpp"
#include "BatchPaths.hpp"
#include "MeshSoup.hpp"
#include "PointInstancer.h"
#include "Skinning.hpp"
#include "UsdzArchive.hpp"
#include "details/OvstageHelpers.hpp"

#include <ovstage/ovstage.h>
#include <ovstage/ovstage_instancing.h>
#include <ovstage/ovstage_population.h> /* stage-info prim path (empty-scene up-axis fallback) */
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/types.h>

#include <stb_image.h> /* implementation lives in gl/StbImageImplementation.c */

#if defined(OVGL_HAS_CUDA)
#    include <cuda_runtime.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

thread_local std::string g_err;

bool finishEnqueue(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (!stage || enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return false;
    ovstage_op_wait_result_t wait{};
    const ovstage_api_status_t waited = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
    const ovstage_api_status_t released = ovstage_release_op(stage, enqueue.op_index);
    return waited == OVSTAGE_OK && wait.error_op_id_count == 0 && released == OVSTAGE_OK;
}

struct ScopedPathListReference
{
    path_dictionary_instance_t* dict = nullptr;
    ovx_primpath_list_t handle = OVX_INVALID_PRIMPATH_LIST;
    ~ScopedPathListReference()
    {
        if (dict && handle != OVX_INVALID_PRIMPATH_LIST)
            path_dictionary_release_path_list_reference(dict, handle);
    }
};

struct ScopedQueryHandle
{
    ovstage_instance_t* stage = nullptr;
    ovstage_query_handle_t handle = OVSTAGE_INVALID_QUERY_HANDLE;
    ~ScopedQueryHandle()
    {
        if (stage && handle != OVSTAGE_INVALID_QUERY_HANDLE)
            (void)finishEnqueue(stage, ovstage_release_query(stage, handle));
    }
};

struct ScopedReadHandle
{
    ovstage_instance_t* stage = nullptr;
    ovstage_read_handle_t handle = OVSTAGE_INVALID_READ_HANDLE;
    ~ScopedReadHandle()
    {
        if (stage && handle != OVSTAGE_INVALID_READ_HANDLE)
            (void)finishEnqueue(stage, ovstage_release_read(stage, handle));
    }
};

/* Per-attribute prefetch cache for the scene build: attr -> column (bytes per prim), keyed by
 * g_prim_index. readAttributeHost builds and tears down a query per single prim, ~27x per mesh;
 * ovstage is columnar, so the build prefetches each column once with one batched read and
 * serves the loop from here. A covered prim with an empty entry has no value. Build-scoped;
 * cleared at the end so refresh_xforms/pick read the live store. Correctness relies on
 * the ovstage multi-prim read returning owned bytes per row (fixed 2026-07: it aliased a reused
 * ragged scratch buffer, clobbering multi-chunk values). */
struct BatchColumn
{
    std::vector<std::vector<uint8_t>> rows;
    /* A later material-discovery phase can append paths after geometry columns were read.
     * Empty bytes mean "authored value absent" only when covered[slot] is set; an uncovered
     * slot must fall back to a live point read instead of being mistaken for an absent value. */
    std::vector<uint8_t> covered;
    /* Per-row column semantic (ovstage_attribute_semantic_t), captured from the
     * batched read groups so cached rows carry the same decode-space signal as
     * live point reads: readStringAttribute picks token-id vs path-id resolution by
     * it (token and path ids are overlapping small counters — see the
     * ID-semantic decode block below). NONE when the producer left the column
     * untagged. */
    std::vector<uint8_t> semantics;
};
std::unordered_map<std::string, BatchColumn> g_batch;
std::unordered_map<std::string, size_t> g_prim_index;
void batchClear()
{
    g_batch.clear();
    g_prim_index.clear();
}

long g_reads = 0;
bool profileLoad()
{
    static const bool s_kEnabled = std::getenv("OVGL_PROFILE_LOAD") != nullptr;
    return s_kEnabled;
}
double loadNowMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::vector<GpuMaterialParams> g_materials; /* per-mesh PBR materials from the last build */
std::vector<GpuLight> g_lights; /* authored USD lights from the last build */

/* Texture set for the last build. gl wants (pixels, w, h, is_srgb) and indexes them from
 * GpuMaterialParams::tex_indices; we own the pixel storage so it outlives the upload. */
std::vector<GpuTextureData> g_textures;
std::vector<std::shared_ptr<std::vector<uint8_t>>> g_texture_pixels;
std::unordered_map<std::string, int> g_texture_index; /* resolved path -> index (dedupe) */

/* Cross-rebuild texture DECODE memo. Timeline playback re-samples time-varying
 * attributes into the store every tick, which makes the renderer take the
 * full-rebuild correctness baseline on every animation frame; without this
 * memo each of those rebuilds re-decoded every PNG/JPEG in the scene from
 * scratch (measured on Apple's toy_drummer.usdz: ~443 ms of a ~470 ms frame
 * — skeletal animation "played" at ~2 FPS). Decoded RGBA8 pixels are content-
 * addressed by resolved path + underlying file identity (size + mtime), so an
 * unchanged texture is decoded once per session while an edited or replaced
 * file misses and re-decodes.
 *
 * Retention: entries not referenced by either of the last TWO builds are
 * swept at the start of the next build, so resident pixel memory is bounded
 * by the union of two consecutive scenes — exactly the lifetime contract the
 * per-build stores already had ("valid until the next build/free"), because
 * g_texture_pixels retains a shared_ptr on every entry the current build
 * uses. Serialize with ovgl_scene_build (same shared-scratch discipline as
 * every other build-scoped store here). */
struct TextureDecodeMemoEntry
{
    std::shared_ptr<std::vector<uint8_t>> pixels; /* RGBA8, w*h*4 bytes, POST-cap */
    int width = 0;
    int height = 0;
    uint64_t last_used_build = 0;
    /* Digest of the SOURCE bytes this was decoded from, carried so a memo hit
     * can still join the content dedup below without re-reading and re-hashing
     * the file. Empty only for entries whose bytes were never hashed. */
    std::string content;
};
std::unordered_map<std::string, TextureDecodeMemoEntry> g_texture_decode_memo;
uint64_t g_texture_build_serial = 0;

/* How many stbi decodes this build actually paid for. Distinct from the number
 * of texture paths it resolved (the memo and the content dedup below both hand
 * back pixels without decoding) and from g_textures.size() (two color spaces
 * over one image are two GPU textures off ONE decode). Reset per build; only
 * read by the OVGL_PROFILE_LOAD line, as is the digesting time beside it -- the
 * price content dedup charges up front for the decodes it then skips. */
size_t g_texture_decodes = 0;
double g_texture_hash_ms = 0.0;

/* Content-hash texture deduplication.
 *
 * g_texture_index dedups by PATH, which is the wrong key for how real asset
 * libraries ship. The mirrored franka_factory corpus is 23 PNGs, 11 unique by
 * md5: six SimReady CubeBox maps (4096x4096 Tile/Trim albedo, ORM, normal,
 * opacity) are stored three times over under cubebox_a02/a03/a04,
 * byte-identical -- so three paths meant three reads, three PNG inflates and
 * three GL textures of the same image.
 *
 * MEASURED on this tree, franka_factory at 960x700, at the 2048 default cap,
 * medians of 5 interleaved runs a side (the A/B is OVGL_TEXTURE_DEDUP below, so
 * it is one binary against itself):
 *
 *      by path       19 GPU textures / 19 decodes / 389.3 MB of mip chains
 *                    meshloop 5778.2 ms, step 0 total 7.05 s
 *      by content     7 GPU textures /  7 decodes / 133.3 MB
 *                    meshloop 2083.0 ms, step 0 total 3.07 s, of which 30.3 ms
 *                    is digesting the 230.6 MB of source bytes it reads
 *
 * and the rendered frame is BYTE-IDENTICAL across all ten runs (one md5,
 * 5ed45cea10933aa3b7504224b2e2d816) -- deduplicating byte-identical inputs is a
 * pure optimization, so any pixel difference would mean something merged that
 * should not have. 19 -> 7 rather than 19 -> 11 because this scene binds only 19
 * of the mirror's 23 PNGs: 18 CubeBox copies of 6 uniques (all 4096-square,
 * capped to 2048 here) plus one 1024-square plastic normal map. Two further
 * paths are missing from the mirror and negative-cache, which is why the
 * profile line reports paths=21.
 *
 * HASH THE SOURCE BYTES, NOT THE DECODED PIXELS. Hashing decoded pixels would
 * save the upload and the VRAM but not the decode, and the decode is the whole
 * cold start -- a PNG has no partial-resolution decode, so the only way not to
 * pay for it is to not run it. Reading the file to hash it costs nothing extra
 * because the decode had to read it anyway (stbi_load's stdio path is replaced
 * by read-then-stbi_load_from_memory, which is also what the packaged .usdz
 * branch already did).
 *
 * THE KEY IS (CONTENT, IS_SRGB), NOT CONTENT ALONE. GpuOpenGles.c picks the GL
 * internal format per texture from is_srgb -- GL_SRGB8_ALPHA8 vs GL_RGBA8
 * (GpuOpenGles.c:1417-1418) -- so ONE image bound to a color slot and to a data slot
 * legitimately wants TWO GPU textures. Merging those on content alone silently
 * corrupts one of them: a normal/ORM map sampled through sRGB, or an albedo
 * sampled linear and rendering too dark. The two entries still share ONE decode
 * and ONE pixel buffer (shared_ptr), so the disagreement costs GPU memory only,
 * never CPU time, and it is the format -- not the pixels -- that differs.
 *
 * HOW THIS COMPOSES WITH THE DECODED-SIZE CAP (boxDownsampleRgba8, :811). The
 * cap is NOT in the key and must never be: it runs BETWEEN the decode and the
 * memo/content-cache inserts, and it is a pure function of (decoded w, decoded
 * h, the process-static maxTextureSize()), so identical source bytes decode to
 * identical dimensions and therefore take an identical cap decision. Two copies
 * of one image can never end up one capped and one not. This ordering is also
 * the only one that pays: the digest is taken BEFORE the decode precisely so a
 * content hit can skip the decode, and a hit that skipped the decode has nothing
 * to cap -- it adopts the already-capped pixels of the entry it joined. Putting
 * the cap after registration, or the digest after the decode, breaks one of
 * those two properties each.
 *
 * COLLISIONS. The digest is MurmurHash3 x64_128 over the bytes, paired with the
 * byte length, so a false merge needs a 128-bit collision that also matches the
 * length. That is the same bet every content-addressed cache makes. It is NOT
 * backed by a byte comparison, deliberately: verifying would mean either
 * retaining every unique source buffer for the whole build (+79.5 MB of RSS on
 * this corpus, measured over its 7 unique images) or re-reading the twin on
 * every hit (the same 230.6 MB again, spent to confirm decodes we already know
 * we want to skip). Murmur is not cryptographic, so a hostile asset could in
 * principle be built to collide; the failure mode is one texture rendered in
 * place of another, not memory unsafety. */
struct TextureRegistration
{
    std::string content; /* source digest; empty when the bytes were not hashed */
    int paths = 0; /* distinct resolved paths registered onto this entry */
};
std::vector<TextureRegistration> g_texture_reg; /* parallel to g_textures */
/* (content digest, is_srgb) -> texture index, for THIS build only. Cross-build
 * reuse stays the path-keyed decode memo above, which is exact. */
std::unordered_map<std::string, int> g_texture_content_index;

/* The is_srgb byte is the whole safety property of this key. Dropping it -- i.e.
 * returning `content` alone -- was BUILT AND MEASURED on a fixture that binds
 * three byte-identical copies of one PNG to two color slots and one linear
 * occlusion/normal slot: the content-only key collapses all three to 1 GPU
 * texture instead of 2, and the data slot, sampled through GL_SRGB8_ALPHA8,
 * renders max_channel_error 112 with 7.69% of the frame off by more than 32.
 * With this key the same fixture is byte-identical to OVGL_TEXTURE_DEDUP=0.
 * Do not "simplify" it away. */
std::string contentSlotKey(const std::string& content, bool srgb)
{
    return content + (srgb ? "\x01" : "\x00");
}

/* Index of an existing entry for `content`: the one whose color space matches
 * `srgb`, or -- when `exact` is false -- the other color space's entry, whose
 * DECODED PIXELS are what the caller is really after. -1 when neither exists. */
int findContentEntry(const std::string& content, bool srgb, bool exact)
{
    if (content.empty())
        return -1;
    auto hit = g_texture_content_index.find(contentSlotKey(content, srgb));
    if (hit != g_texture_content_index.end())
        return hit->second;
    if (exact)
        return -1;
    hit = g_texture_content_index.find(contentSlotKey(content, !srgb));
    return hit != g_texture_content_index.end() ? hit->second : -1;
}

/* MurmurHash3 x64_128 (Austin Appleby, public domain), the block loads done
 * through memcpy so unaligned source buffers are defined behavior. The digest
 * is little-endian-dependent and process-local -- nothing persists it, so that
 * costs nothing here. */
inline uint64_t mmRotl64(uint64_t x, int r)
{
    return (x << r) | (x >> (64 - r));
}
inline uint64_t mmFmix64(uint64_t k)
{
    k ^= k >> 33;
    k *= 0xff51afd7ed558ccdULL;
    k ^= k >> 33;
    k *= 0xc4ceb9fe1a85ec53ULL;
    k ^= k >> 33;
    return k;
}
void murmur3X64128(const uint8_t* data, size_t len, uint64_t out[2])
{
    const uint64_t c1 = 0x87c37b91114253d5ULL, c2 = 0x4cf5ad432745937fULL;
    uint64_t h1 = 0, h2 = 0;
    const size_t nblocks = len / 16;
    for (size_t i = 0; i < nblocks; ++i)
    {
        uint64_t k1, k2;
        std::memcpy(&k1, data + i * 16, 8);
        std::memcpy(&k2, data + i * 16 + 8, 8);
        k1 *= c1;
        k1 = mmRotl64(k1, 31);
        k1 *= c2;
        h1 ^= k1;
        h1 = mmRotl64(h1, 27);
        h1 += h2;
        h1 = h1 * 5 + 0x52dce729;
        k2 *= c2;
        k2 = mmRotl64(k2, 33);
        k2 *= c1;
        h2 ^= k2;
        h2 = mmRotl64(h2, 31);
        h2 += h1;
        h2 = h2 * 5 + 0x38495ab5;
    }
    const uint8_t* tail = data + nblocks * 16;
    uint64_t k1 = 0, k2 = 0;
    switch (len & 15)
    {
    case 15:
        k2 ^= (uint64_t)tail[14] << 48;
        [[fallthrough]];
    case 14:
        k2 ^= (uint64_t)tail[13] << 40;
        [[fallthrough]];
    case 13:
        k2 ^= (uint64_t)tail[12] << 32;
        [[fallthrough]];
    case 12:
        k2 ^= (uint64_t)tail[11] << 24;
        [[fallthrough]];
    case 11:
        k2 ^= (uint64_t)tail[10] << 16;
        [[fallthrough]];
    case 10:
        k2 ^= (uint64_t)tail[9] << 8;
        [[fallthrough]];
    case 9:
        k2 ^= (uint64_t)tail[8];
        k2 *= c2;
        k2 = mmRotl64(k2, 33);
        k2 *= c1;
        h2 ^= k2;
        [[fallthrough]];
    case 8:
        k1 ^= (uint64_t)tail[7] << 56;
        [[fallthrough]];
    case 7:
        k1 ^= (uint64_t)tail[6] << 48;
        [[fallthrough]];
    case 6:
        k1 ^= (uint64_t)tail[5] << 40;
        [[fallthrough]];
    case 5:
        k1 ^= (uint64_t)tail[4] << 32;
        [[fallthrough]];
    case 4:
        k1 ^= (uint64_t)tail[3] << 24;
        [[fallthrough]];
    case 3:
        k1 ^= (uint64_t)tail[2] << 16;
        [[fallthrough]];
    case 2:
        k1 ^= (uint64_t)tail[1] << 8;
        [[fallthrough]];
    case 1:
        k1 ^= (uint64_t)tail[0];
        k1 *= c1;
        k1 = mmRotl64(k1, 31);
        k1 *= c2;
        h1 ^= k1;
        break;
    default:
        break;
    }
    h1 ^= len;
    h2 ^= len;
    h1 += h2;
    h2 += h1;
    h1 = mmFmix64(h1);
    h2 = mmFmix64(h2);
    h1 += h2;
    h2 += h1;
    out[0] = h1;
    out[1] = h2;
}

/* OVGL_TEXTURE_DEDUP=0 turns content dedup off: an empty digest disables every
 * merge, so registration falls back to path-keyed entries and the renderer takes
 * exactly the pre-dedup path (one entry per path, in-place sRGB demotion, one
 * decode each). It exists as the A/B lever this change was measured with and as
 * the field escape hatch if a digest ever merges something it should not. Read
 * once into a function-local static, like OVGL_PROFILE_LOAD (:112),
 * OVGL_MAX_TEX_SIZE (:787) and OVGL_GRID_OVERLAY / OVGL_SS (Ovgl.cpp), so it
 * cannot change mid-process and split one build between deduped and non-deduped
 * textures. */
bool textureDedupEnabled()
{
    static const bool s_kEnabled = []
    {
        const char* e = std::getenv("OVGL_TEXTURE_DEDUP");
        return !(e && e[0] == '0' && e[1] == '\0');
    }();
    return s_kEnabled;
}

std::string contentDigest(const std::vector<uint8_t>& bytes)
{
    if (!textureDedupEnabled())
        return {};
    const double t0 = profileLoad() ? loadNowMs() : 0.0;
    uint64_t h[2] = { 0, 0 };
    murmur3X64128(bytes.data(), bytes.size(), h);
    if (profileLoad())
        g_texture_hash_ms += loadNowMs() - t0;
    char buf[64];
    std::snprintf(buf, sizeof buf, "%016llx%016llx:%llu", (unsigned long long)h[0], (unsigned long long)h[1],
                  (unsigned long long)bytes.size());
    return std::string(buf);
}

/* Slurp a plain file. Replaces stbi_load(path) so the bytes can be hashed once
 * and then decoded from memory — the same two-step the packaged .usdz branch
 * has always taken. false leaves `out` unspecified and the caller negative-
 * caches the path, exactly as a decode failure did. */
bool readWholeFile(const std::string& path, std::vector<uint8_t>& out)
{
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    bool ok = false;
    if (std::fseek(f, 0, SEEK_END) == 0)
    {
        const long n = std::ftell(f);
        if (n >= 0 && std::fseek(f, 0, SEEK_SET) == 0)
        {
            out.resize((size_t)n);
            ok = (n == 0) || std::fread(out.data(), 1, (size_t)n, f) == (size_t)n;
        }
    }
    std::fclose(f);
    return ok;
}

/* Identity of the bytes behind a resolved texture path: size + mtime of the
 * plain file, or of the .usdz archive for a packaged "/pkg.usdz[inner]" path.
 * Empty when the file cannot be stat'ed — the caller then skips the memo and
 * decodes fresh (fail-open to today's behavior). */
std::string textureFileIdentity(const std::string& resolved)
{
    std::string file = resolved;
    const size_t bracket = resolved.find('[');
    if (bracket != std::string::npos && resolved.back() == ']')
        file = resolved.substr(0, bracket);
    std::error_code size_error, time_error;
    const auto size = std::filesystem::file_size(file, size_error);
    const auto mtime = std::filesystem::last_write_time(file, time_error);
    if (size_error || time_error)
        return {};
    return std::to_string((unsigned long long)size) + ":" + std::to_string((long long)mtime.time_since_epoch().count());
}

/* mesh path -> material path, for meshes whose binding lives on a child GeomSubset. USD lets a
 * mesh carry no binding of its own and bind per-face through GeomSubset children; Apple's
 * biplane binds its whole body, wings and wheels that way (one subset covering every face), so
 * without this they resolve to no material and render as flat gray displayColor. The nanousd
 * renderer has no GeomSubset support at all, so there is nothing to port -- this is the minimal
 * correct reading. NOT handled: a mesh split across SEVERAL subsets with DIFFERENT materials;
 * that needs the mesh broken into per-subset draws, and we take the first subset's material. */
std::unordered_map<std::string, std::string> g_subset_binding;

/* UsdPreviewSurface input -> gl texture slot. Mirrors the nanousd renderer's
 * material.c TEX_* enum, which is the layout gl's PBR fragment shader samples. */
enum
{
    TEX_DIFFUSE_COLOR = 0,
    TEX_NORMAL = 1,
    TEX_ROUGHNESS = 2,
    TEX_METALLIC = 3,
    TEX_EMISSIVE_COLOR = 4,
    TEX_OCCLUSION = 5,
    TEX_OPACITY = 6,
};

struct TextureSlotDescription
{
    const char* input; /* UsdPreviewSurface input name */
    int slot; /* gl texture slot */
    bool srgb; /* color data (sRGB-encoded) vs data (linear) */
};

/* Color slots are sRGB-encoded; normals/roughness/metallic/AO are linear DATA. Sampling data
 * as sRGB corrupts normals and PBR; sampling color as linear renders it too dark. */
constexpr TextureSlotDescription g_kTextureSlots[] = {
    { "inputs:diffuseColor", TEX_DIFFUSE_COLOR, true },
    { "inputs:normal", TEX_NORMAL, false },
    { "inputs:roughness", TEX_ROUGHNESS, false },
    { "inputs:metallic", TEX_METALLIC, false },
    { "inputs:emissiveColor", TEX_EMISSIVE_COLOR, true },
    { "inputs:occlusion", TEX_OCCLUSION, false },
    { "inputs:opacity", TEX_OPACITY, false },
};

ovx_string_t toOvx(const std::string& s)
{
    ovx_string_t out{};
    out.ptr = s.c_str();
    out.length = s.size();
    return out;
}

ovx_token_t internAttribute(ovstage_instance_t* stage, const std::string& name)
{
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
        return OVX_INVALID_TOKEN;
    ovx_string_t s = toOvx(name);
    ovx_token_t tok = OVX_INVALID_TOKEN;
    if (path_dictionary_create_tokens_from_strings(dict, &s, 1, &tok).status != OVX_API_SUCCESS)
        return OVX_INVALID_TOKEN;
    return tok;
}

/* POINT read of one attribute on one prim -> host bytes (cudaMemcpy if device-resident).
 * `semantic_out` (optional) receives the column's authored semantic from the read group
 * (OVSTAGE_SEMANTIC_NONE when untagged/unavailable) so string readers can pick the id
 * decode space deterministically instead of guessing. */
bool readAttributeHost(ovstage_instance_t* stage,
                       ovstage_ordinal_t ordinal,
                       const std::string& prim_path,
                       const std::string& attr_name,
                       std::vector<uint8_t>& out,
                       ovstage_attribute_semantic_t* semantic_out = nullptr)
{
    out.clear();
    if (semantic_out)
        *semantic_out = OVSTAGE_SEMANTIC_NONE;
    if (!g_batch.empty())
    {
        auto col = g_batch.find(attr_name);
        if (col != g_batch.end())
        {
            auto s = g_prim_index.find(prim_path);
            if (s != g_prim_index.end() && s->second < col->second.rows.size() &&
                s->second < col->second.covered.size() && col->second.covered[s->second])
            {
                if (col->second.rows[s->second].empty())
                    return false;
                out = col->second.rows[s->second];
                if (semantic_out && s->second < col->second.semantics.size())
                    *semantic_out = (ovstage_attribute_semantic_t)col->second.semantics[s->second];
                return true;
            }
        }
    }
    if (profileLoad())
        ++g_reads;
    ovx_token_t tok = internAttribute(stage, attr_name);
    if (tok == OVX_INVALID_TOKEN)
    {
        g_err = "intern(" + attr_name + ")";
        return false;
    }

    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
    {
        g_err = "no path dictionary";
        return false;
    }
    const ovx_string_t path_string = toOvx(prim_path);
    ovx_primpath_list_t path_list = OVX_INVALID_PRIMPATH_LIST;
    ScopedPathListReference path_ref{ dict, path_list };
    if (path_dictionary_create_path_list_from_strings(dict, &path_string, 1, &path_list).status != OVX_API_SUCCESS ||
        path_list == OVX_INVALID_PRIMPATH_LIST)
    {
        g_err = "path(" + prim_path + ")";
        return false;
    }
    path_ref.handle = path_list;
    ovstage_query_handle_t qh = OVSTAGE_INVALID_QUERY_HANDLE;
    ScopedQueryHandle query_ref{ stage, qh };
    if (ovstage_query_from_path_list(stage, path_list, &qh) != OVSTAGE_OK || qh == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        g_err = "handle(" + prim_path + ")";
        return false;
    }
    query_ref.handle = qh;

    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    ScopedReadHandle read_ref{ stage, rh };
    ovstage_ordinal_range_t range{ 0, ordinal, false }; /* latest <= ordinal */
    auto er = ovstage_read_attributes(stage, qh, &tok, 1, range, &rh);
    read_ref.handle = rh;
    if (er.status != OVSTAGE_OK || rh == OVSTAGE_INVALID_READ_HANDLE)
    {
        g_err = "read_attributes(" + prim_path + "." + attr_name + ")";
        return false;
    }
    bool got = false, err = false;
    for (;;)
    {
        ovstage_read_group_t g{};
        auto fr = ovstage_fetch_read_next(stage, rh, OVSTAGE_TIMEOUT_INFINITE, &g);
        if (fr == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fr != OVSTAGE_OK)
        {
            err = true;
            break;
        }
        if (!got && !g.is_delete && g.data.tensor_count >= 1 && g.data.tensors && g.prims.count > 0)
        {
            const DLTensor& t0 = g.data.tensors[0];
            const size_t bits = static_cast<size_t>(t0.dtype.bits) * t0.dtype.lanes;
            const size_t elem = bits == 0 ? 0 : (bits + 7) / 8;
            bool valid = elem != 0 && t0.ndim == 1 && t0.shape && t0.shape[0] >= 0 &&
                         static_cast<uint64_t>(t0.shape[0]) <= std::numeric_limits<size_t>::max() / elem;
            const size_t total = valid ? static_cast<size_t>(t0.shape[0]) * elem : 0;
            const uint8_t* base = t0.data ? static_cast<const uint8_t*>(t0.data) + t0.byte_offset : nullptr;
            size_t b0 = 0, b1 = total;
            if (valid && g.data.tensor_count >= 2)
            {
                const DLTensor& offsets = g.data.tensors[1];
                valid = offsets.data && offsets.device.device_type == kDLCPU && offsets.dtype.code == kDLUInt &&
                        offsets.dtype.bits == 64 && offsets.dtype.lanes == 1 && offsets.ndim == 1 && offsets.shape &&
                        offsets.shape[0] >= 2;
                if (valid)
                {
                    const uint8_t* offset_bytes = static_cast<const uint8_t*>(offsets.data) + offsets.byte_offset;
                    uint64_t begin = 0, end = 0;
                    std::memcpy(&begin, offset_bytes, sizeof(begin));
                    std::memcpy(&end, offset_bytes + sizeof(begin), sizeof(end));
                    valid = begin <= std::numeric_limits<size_t>::max() && end <= std::numeric_limits<size_t>::max();
                    if (valid)
                    {
                        b0 = static_cast<size_t>(begin);
                        b1 = static_cast<size_t>(end);
                    }
                }
            }
            valid = valid && b1 >= b0 && b1 <= total && (b1 == b0 || base);
            if (valid)
            {
                out.resize(b1 - b0);
                if (t0.device.device_type == kDLCUDA)
                {
#if defined(OVGL_HAS_CUDA)
                    const cudaError_t copy_status = cudaMemcpy(out.data(), base + b0, b1 - b0, cudaMemcpyDeviceToHost);
                    if (copy_status != cudaSuccess)
                    {
                        g_err = "cudaMemcpy(" + prim_path + "." + attr_name + "): " + cudaGetErrorString(copy_status);
                        out.clear();
                        err = true;
                    }
                    else
                    {
                        got = true;
                    }
#else
                    g_err = "CUDA-resident attribute " + prim_path + "." + attr_name +
                            " cannot be read by this CPU-only ovgl build";
                    out.clear();
                    err = true;
#endif
                }
                else if (t0.device.device_type == kDLCPU)
                {
                    if (b1 > b0)
                        std::memcpy(out.data(), base + b0, b1 - b0);
                    got = true;
                }
                else
                {
                    g_err = "unsupported tensor device for " + prim_path + "." + attr_name;
                    out.clear();
                    err = true;
                }
                if (got && semantic_out)
                    *semantic_out = g.semantic;
            }
            else
            {
                g_err = "invalid value storage for " + prim_path + "." + attr_name;
                err = true;
            }
        }
        if (ovstage_release_group(stage, &g) != OVSTAGE_OK)
        {
            g_err = "release_group(" + prim_path + "." + attr_name + ")";
            err = true;
            break;
        }
    }
    if (!finishEnqueue(stage, er))
    {
        g_err = "read operation(" + prim_path + "." + attr_name + ")";
        err = true;
    }
    return got && !err;
}

/* Batched reads accept groups from either ovstage backend, so treat tensor metadata as
 * untrusted.  The bridge copies rows on the CPU and can only consume contiguous,
 * host-addressable tensors whose byte extent can be computed without overflow. */
bool batchTensorExtent(const DLTensor& tensor, size_t& element_count, size_t& element_bytes, size_t& total_bytes)
{
    element_count = 0;
    element_bytes = 0;
    total_bytes = 0;

    const size_t bits = (size_t)tensor.dtype.bits * (size_t)tensor.dtype.lanes;
    if (bits == 0 || bits > std::numeric_limits<size_t>::max() - 7)
        return false;
    element_bytes = (bits + 7) / 8;
    if (tensor.ndim < 0 || (tensor.ndim > 0 && !tensor.shape))
        return false;

    size_t count = 1;
    size_t contiguous_stride = 1;
    for (int32_t dim = tensor.ndim; dim-- > 0;)
    {
        if (tensor.shape[dim] < 0)
            return false;
        const size_t width = (size_t)tensor.shape[dim];
        if (tensor.strides && width > 1 && tensor.strides[dim] != (int64_t)contiguous_stride)
            return false;
        if (width != 0 && count > std::numeric_limits<size_t>::max() / width)
            return false;
        count *= width;
        if (width != 0 && contiguous_stride > std::numeric_limits<size_t>::max() / width)
            return false;
        contiguous_stride *= width;
    }
    if (count != 0 && element_bytes > std::numeric_limits<size_t>::max() / count)
        return false;
    const size_t bytes = count * element_bytes;
    if (bytes != 0 && !tensor.data)
        return false;
    if (tensor.byte_offset > std::numeric_limits<size_t>::max() - bytes)
        return false;

    element_count = count;
    total_bytes = bytes;
    return true;
}

bool batchTensorHostAccessible(const DLTensor& tensor)
{
    switch (tensor.device.device_type)
    {
    case kDLCPU:
    case kDLCUDAHost:
    case kDLROCMHost:
    case kDLCUDAManaged:
        return true;
    default:
        return false;
    }
}

uint64_t batchLoadU64(const uint8_t* ptr)
{
    uint64_t value = 0;
    std::memcpy(&value, ptr, sizeof(value));
    return value;
}

bool batchMaskTest(ovstage_mask_t mask, uint32_t index)
{
    return !mask || ((mask[index / 64] >> (index % 64)) & uint64_t{ 1 }) != 0;
}

/* Read ONE attribute for MANY prims in ONE query. Rows land in `out` indexed to match
 * `paths`; a prim with no value gets an empty entry. Positions come from the read group's
 * prim.index_map (query positions into prims.list). The logical prim row is mapped separately
 * through data.index_map/mask before reading its tensor slot; conflating those two maps breaks
 * reordered, deduplicated, or masked data returned by another ovstage producer.
 * `semantics_out` (optional, sized/indexed like `out`) receives each assigned row's column
 * semantic (ovstage_attribute_semantic_t; NONE for unassigned rows) for the batch cache. */
bool readBatchedAttributes(ovstage_instance_t* stage,
                           ovstage_ordinal_t ordinal,
                           const std::vector<std::string>& paths,
                           const std::string& attr,
                           std::vector<std::vector<uint8_t>>& out,
                           std::vector<uint8_t>* semantics_out = nullptr)
{
    out.assign(paths.size(), {});
    if (semantics_out)
        semantics_out->assign(paths.size(), (uint8_t)OVSTAGE_SEMANTIC_NONE);
    if (paths.empty())
        return true;
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
    {
        g_err = "no path dictionary";
        return false;
    }

    std::vector<ovx_string_t> ps;
    ps.reserve(paths.size());
    for (const auto& s : paths)
        ps.push_back(ovx_string_t{ s.c_str(), s.size() });
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    ScopedPathListReference list_ref{ dict, list };
    const auto create_list = path_dictionary_create_path_list_from_strings(dict, ps.data(), ps.size(), &list);
    list_ref.handle = list;
    if (create_list.status != OVX_API_SUCCESS || list == OVX_INVALID_PRIMPATH_LIST)
    {
        g_err = "create_path_list(" + attr + ")";
        return false;
    }
    ovstage_query_handle_t qh = OVSTAGE_INVALID_QUERY_HANDLE;
    ScopedQueryHandle query_ref{ stage, qh };
    const auto query_status = ovstage_query_from_path_list(stage, list, &qh);
    query_ref.handle = qh;
    const bool okq = query_status == OVSTAGE_OK && qh != OVSTAGE_INVALID_QUERY_HANDLE;
    if (!okq)
    {
        g_err = "query_from_path_list(" + attr + ")";
        return false;
    }

    ovx_token_t tok = internAttribute(stage, attr);
    if (tok == OVX_INVALID_TOKEN)
    {
        g_err = "intern(batch " + attr + ")";
        return false;
    }
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    ScopedReadHandle read_ref{ stage, rh };
    ovstage_ordinal_range_t range{ 0, ordinal, false };
    const auto read_status = ovstage_read_attributes(stage, qh, &tok, 1, range, &rh);
    read_ref.handle = rh;
    if (read_status.status != OVSTAGE_OK || rh == OVSTAGE_INVALID_READ_HANDLE)
    {
        g_err = "read_attributes(batch " + attr + ")";
        return false;
    }

    std::unordered_map<std::string, size_t> want;
    want.reserve(paths.size());
    for (size_t i = 0; i < paths.size(); i++)
        want.emplace(paths[i], i);

    bool invalid_batch = false;
    std::vector<uint8_t> assigned(paths.size(), 0);
    /* The original query list remains referenced for the whole read, making exact handle
     * identity a valid fast-path test. A different returned list can have the same cardinality
     * in a different order, so resolve every distinct non-query handle once per read. */
    std::unordered_map<ovx_primpath_list_t, std::vector<std::string>> resolved;
    for (;;)
    {
        ovstage_read_group_t g{};
        auto fr = ovstage_fetch_read_next(stage, rh, OVSTAGE_TIMEOUT_INFINITE, &g);
        if (fr == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fr != OVSTAGE_OK)
        {
            invalid_batch = true;
            g_err = "fetch_read_next(batch " + attr + ")";
            break;
        }
        const std::vector<std::string>* gpp = nullptr;
        if (g.prims.list != OVX_INVALID_PRIMPATH_LIST)
        {
            gpp = isaacsim::ovgl_viewport::debug::details::ovgl::resolveBatchGroupPaths(
                g.prims.list, list, paths, resolved,
                [dict](ovx_primpath_list_t group_list)
                { return isaacsim::ovgl_viewport::debug::details::resolvePrimPaths(dict, group_list); });
        }
        if (!gpp || gpp->empty())
        {
            invalid_batch = true;
            g_err = "unresolvable list(batch " + attr + ")";
            if (ovstage_release_group(stage, &g) != OVSTAGE_OK)
                g_err = "release_group(batch " + attr + ")";
            continue;
        }
        const std::vector<std::string>& gp = *gpp;
        if (!g.is_delete)
        {
            const uint32_t logical_count = g.prims.count;
            const bool has_index_map = g.data.index_map != nullptr;
            const bool has_mask = g.data.mask != nullptr;
            bool group_valid = logical_count > 0 && g.data.tensor_count > 0 && g.data.tensors;
            if (has_index_map && has_mask)
                group_valid = false;
            if ((has_index_map || has_mask) && g.data.count != logical_count)
                group_valid = false;
            if (!has_index_map && !has_mask && g.data.count != 0 && g.data.count != logical_count)
                group_valid = false;
            if (g.data.cuda_sync.stream || g.data.cuda_sync.wait_event)
                group_valid = false;

            /* Decode precedence (read-layout port): the in-repo engine now
             * emits the OFFICIAL transports — one tensor per row for ragged
             * (is_array) groups, one stacked tensor for fixed groups. The
             * legacy [values, uint64 byte-offsets] CSR pair is tolerated for
             * FOREIGN producers only, and only where per-row cannot claim the
             * shape first (a 2-row ragged group whose second row tensor is
             * itself {kDLUInt,64,1} must decode per-row, not as CSR). */
            const size_t data_rows = g.data.count ? (size_t)g.data.count : (size_t)logical_count;
            const bool is_per_row = group_valid && g.is_array && g.data.tensor_count == data_rows;
            bool is_csr = false;
            if (group_valid && !is_per_row && g.data.tensor_count == 2)
            {
                const DLTensor& offsets = g.data.tensors[1];
                is_csr = offsets.dtype.code == kDLUInt && offsets.dtype.bits == 64 && offsets.dtype.lanes == 1 &&
                         offsets.ndim == 1 && offsets.shape && (size_t)offsets.shape[0] == data_rows + 1;
            }

            size_t value_elements = 0, value_element_bytes = 0, value_total = 0;
            size_t value_rows = 0, value_row_bytes = 0;
            size_t offset_elements = 0, offset_element_bytes = 0, offset_total = 0;
            const uint8_t* values = nullptr;
            const uint8_t* offsets = nullptr;
            if (group_valid && is_csr)
            {
                const DLTensor& tv = g.data.tensors[0];
                const DLTensor& to = g.data.tensors[1];
                group_valid = batchTensorHostAccessible(tv) && batchTensorHostAccessible(to) &&
                              batchTensorExtent(tv, value_elements, value_element_bytes, value_total) &&
                              batchTensorExtent(to, offset_elements, offset_element_bytes, offset_total) &&
                              offset_element_bytes == sizeof(uint64_t);
                if (group_valid)
                {
                    values = tv.data ? (const uint8_t*)tv.data + (size_t)tv.byte_offset : nullptr;
                    offsets = (const uint8_t*)to.data + to.byte_offset;
                }
            }
            else if (group_valid && !g.is_array)
            {
                /* Fixed-size attributes have one stacked tensor with storage
                 * rows along its leading extent; the row stride is
                 * total_bytes / rows (NOT one dtype element per prim — a
                 * fixed multi-element row like extent is 2 float3 elements).
                 * Reject multi-tensor shapes instead of guessing. */
                const DLTensor& tv = g.data.tensors[0];
                group_valid = g.data.tensor_count == 1 && batchTensorHostAccessible(tv) &&
                              batchTensorExtent(tv, value_elements, value_element_bytes, value_total);
                if (group_valid)
                {
                    value_rows = data_rows ? data_rows : 1;
                    group_valid = value_total % value_rows == 0;
                    value_row_bytes = group_valid ? value_total / value_rows : 0;
                    values = tv.data ? (const uint8_t*)tv.data + (size_t)tv.byte_offset : nullptr;
                }
            }
            else if (group_valid && !is_per_row)
            {
                /* is_array without a per-row tensor set and no CSR pair —
                 * malformed under every known producer. */
                group_valid = false;
            }

            for (uint32_t logical = 0; group_valid && logical < logical_count; ++logical)
            {
                const uint64_t prim_row =
                    g.prims.index_map ? (uint64_t)g.prims.index_map[logical] : (uint64_t)g.prims.offset + logical;
                if (prim_row >= gp.size())
                {
                    group_valid = false;
                    break;
                }
                if (!batchMaskTest(g.data.mask, logical))
                    continue;

                const uint32_t data_slot = g.data.index_map ? g.data.index_map[logical] : logical;
                const auto wanted = want.find(gp[(size_t)prim_row]);
                if (wanted == want.end() || assigned[wanted->second])
                    continue;

                const uint8_t* row_begin = nullptr;
                size_t row_bytes = 0;
                if (is_csr)
                {
                    if ((uint64_t)data_slot + 1 >= offset_elements)
                    {
                        group_valid = false;
                        break;
                    }
                    const uint64_t b0 = batchLoadU64(offsets + (size_t)data_slot * sizeof(uint64_t));
                    const uint64_t b1 = batchLoadU64(offsets + ((size_t)data_slot + 1) * sizeof(uint64_t));
                    if (b1 < b0 || b1 > value_total)
                    {
                        group_valid = false;
                        break;
                    }
                    row_begin = values ? values + (size_t)b0 : nullptr;
                    row_bytes = (size_t)(b1 - b0);
                }
                else if (g.is_array)
                {
                    if (data_slot >= g.data.tensor_count)
                    {
                        group_valid = false;
                        break;
                    }
                    const DLTensor& row_tensor = g.data.tensors[data_slot];
                    size_t row_elements = 0, row_element_bytes = 0;
                    group_valid = batchTensorHostAccessible(row_tensor) &&
                                  batchTensorExtent(row_tensor, row_elements, row_element_bytes, row_bytes);
                    if (!group_valid)
                        break;
                    row_begin =
                        row_tensor.data ? (const uint8_t*)row_tensor.data + (size_t)row_tensor.byte_offset : nullptr;
                }
                else
                {
                    if (data_slot >= value_rows)
                    {
                        group_valid = false;
                        break;
                    }
                    row_begin = values ? values + (size_t)data_slot * value_row_bytes : nullptr;
                    row_bytes = value_row_bytes;
                }

                if (row_bytes != 0)
                    out[wanted->second].assign(row_begin, row_begin + row_bytes);
                if (semantics_out)
                    (*semantics_out)[wanted->second] = (uint8_t)g.semantic;
                assigned[wanted->second] = 1;
            }
            if (!group_valid)
            {
                invalid_batch = true;
                g_err = "unsupported or malformed batch group(" + attr + ")";
            }
        }
        if (ovstage_release_group(stage, &g) != OVSTAGE_OK)
        {
            invalid_batch = true;
            g_err = "release_group(batch " + attr + ")";
            break;
        }
    }
    if (!finishEnqueue(stage, read_status))
    {
        invalid_batch = true;
        g_err = "read operation(batch " + attr + ")";
    }
    if (invalid_batch)
    {
        out.assign(paths.size(), {});
        if (semantics_out)
            semantics_out->assign(paths.size(), (uint8_t)OVSTAGE_SEMANTIC_NONE);
        return false;
    }
    return true;
}

/* Extend the build-local column cache for one discovery phase. Each successful column read is
 * a snapshot at the SAME sealed ordinal passed to ovgl_scene_build(). Paths can be discovered
 * incrementally (geometry -> materials -> shaders -> texture nodes); explicit coverage keeps
 * an attribute prefetched for an earlier path set from shadowing a later, unqueried path.
 * Failed/device-resident batch reads remain uncovered and therefore use readAttributeHost's exact
 * point-read fallback. Nothing survives batchClear() at the end of this build. */
void batchPrefetchAttributes(ovstage_instance_t* stage,
                             ovstage_ordinal_t ordinal,
                             const std::vector<std::string>& paths,
                             const char* const* attrs,
                             size_t attr_count)
{
    std::vector<std::string> unique_paths;
    unique_paths.reserve(paths.size());
    std::unordered_set<std::string> seen;
    seen.reserve(paths.size());
    for (const std::string& path : paths)
    {
        if (path.empty() || !seen.insert(path).second)
            continue;
        unique_paths.push_back(path);
        if (g_prim_index.find(path) == g_prim_index.end())
            g_prim_index.emplace(path, g_prim_index.size());
    }
    if (unique_paths.empty())
        return;

    for (auto& entry : g_batch)
    {
        entry.second.rows.resize(g_prim_index.size());
        entry.second.covered.resize(g_prim_index.size(), 0);
        entry.second.semantics.resize(g_prim_index.size(), (uint8_t)OVSTAGE_SEMANTIC_NONE);
    }

    std::unordered_set<std::string> seen_attrs;
    seen_attrs.reserve(attr_count);
    for (size_t a = 0; a < attr_count; ++a)
    {
        if (!attrs[a] || !seen_attrs.insert(attrs[a]).second)
            continue;
        BatchColumn& dst = g_batch[attrs[a]];
        dst.rows.resize(g_prim_index.size());
        dst.covered.resize(g_prim_index.size(), 0);
        dst.semantics.resize(g_prim_index.size(), (uint8_t)OVSTAGE_SEMANTIC_NONE);

        std::vector<std::string> query_paths;
        std::vector<size_t> query_slots;
        query_paths.reserve(unique_paths.size());
        query_slots.reserve(unique_paths.size());
        for (const std::string& path : unique_paths)
        {
            const size_t slot = g_prim_index.find(path)->second;
            if (dst.covered[slot])
                continue;
            query_paths.push_back(path);
            query_slots.push_back(slot);
        }
        if (query_paths.empty())
            continue;

        std::vector<std::vector<uint8_t>> rows;
        std::vector<uint8_t> semantics;
        if (!readBatchedAttributes(stage, ordinal, query_paths, attrs[a], rows, &semantics))
            continue;
        for (size_t row = 0; row < query_slots.size(); ++row)
        {
            const size_t slot = query_slots[row];
            dst.rows[slot] = std::move(rows[row]);
            dst.covered[slot] = 1;
            dst.semantics[slot] = semantics[row];
        }
    }
}

float readScalar(ovstage_instance_t* stage,
                 ovstage_ordinal_t ordinal,
                 const std::string& path,
                 const std::string& attributeName,
                 float fallback)
{
    std::vector<uint8_t> bytes;
    if (!readAttributeHost(stage, ordinal, path, attributeName, bytes) || bytes.empty())
        return fallback;
    if (bytes.size() >= sizeof(double))
        return (float)*(const double*)bytes.data();
    if (bytes.size() >= sizeof(float))
        return *(const float*)bytes.data();
    return fallback;
}

bool readBooleanAttribute(ovstage_instance_t* stage,
                          ovstage_ordinal_t ordinal,
                          const std::string& path,
                          const std::string& attributeName,
                          bool fallback)
{
    std::vector<uint8_t> bytes;
    if (!readAttributeHost(stage, ordinal, path, attributeName, bytes) || bytes.empty())
        return fallback;
    return bytes[0] != 0;
}

/* First AUTHORED scalar among a list of spelling aliases; false (leaving *out
 * untouched) when none is authored.
 *
 * The MDL families spell one input several ways -- RoughnessMin /
 * Roughness_Min / roughnessMin / roughness_min are the same MDL parameter under
 * four exporters. material.c could afford to enumerate every attribute on the
 * shader prim and substring-match a lowercased copy (mdl_apply_float_param,
 * :3200-3253) because nanousd exposes nattribs/attribname; the ovstage bridge
 * reads BY NAME off an ordinal, so that sweep has to become an explicit alias
 * list. readScalar has no authored/unauthored signal, hence the NaN sentinel:
 * every authored float is finite, so a finite read is an authored read. */
bool readScalarAny(ovstage_instance_t* stage,
                   ovstage_ordinal_t ordinal,
                   const std::string& primPath,
                   const char* const* names,
                   size_t count,
                   float* output)
{
    for (size_t i = 0; i < count; ++i)
    {
        const float value = readScalar(stage, ordinal, primPath, names[i], std::numeric_limits<float>::quiet_NaN());
        if (std::isfinite(value))
        {
            *output = value;
            return true;
        }
    }
    return false;
}

/* worldMatrix (row-major double[16]) -> out16; identity if absent. */
void readWorldMatrix(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& path, double output[16])
{
    static const double s_kIdentity[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    std::vector<uint8_t> bytes;
    if (readAttributeHost(stage, ordinal, path, "omni:fabric:worldMatrix", bytes) && bytes.size() >= 16 * sizeof(double))
        std::memcpy(output, bytes.data(), 16 * sizeof(double));
    else
        std::memcpy(output, s_kIdentity, sizeof(s_kIdentity));
}

/* Texture loading (port of nanousd-opengl-renderer/src/material.c).
 *
 * That renderer resolved texture paths against a scene directory and stbi_load()'d them, and
 * explicitly gave up on packages ("USDZ package contents are not exposed by the nanousd C
 * API"). Two things change here:
 *
 *  - the path comes from ovstage, not a resolver walk. ovpopulation mirrors an asset attribute
 *    as NUL-separated <authored>\0<resolved>, and USD has already done the resolution, so
 *    there is no scene_dir to guess against and no recursive file-index fallback to maintain.
 *  - USDZ IS supported, because that resolved path is USD's packaged form,
 *    "/abs/thing.usdz[0/tex.png]". A usdz is a zip whose entries are STORED (never deflated)
 *    and 64-byte aligned precisely so readers can mmap/point at them, so pulling the bytes out
 *    is a local-header walk and a memcpy — which is what makes the Apple sample assets, whose
 *    textures ALL live inside the package, actually show their materials.
 */

/* Decoded-texture size cap (port of material.c's
 * materials_set_max_tex_size, lines 773-869).
 *
 * The fork hard-capped EVERY texture at 512 px for its low-VRAM GLES target.
 * ovgl caps at g_kDefaultMaxTexSize and exposes the knob OVGL_MAX_TEX_SIZE=<px>;
 * an explicit 0 disables the cap entirely, and a negative or non-numeric value
 * falls back to the default rather than silently uncapping (an unparseable knob
 * must not be the one that puts 1.3 GB back on the GPU). Knob shape follows
 * OVGL_GRID_OVERLAY / OVGL_SS (Ovgl.cpp:931-940) and OVGL_PROFILE_LOAD (:112) --
 * read once into a function-local static, so it cannot change mid-process and
 * split the decode memo between capped and uncapped pixels for the same key.
 *
 * WHY 2048 IS THE DEFAULT. Measured over the mirrored franka corpus (23 PNGs,
 * 328.3 Mpx, IHDR-read): nineteen 4096x4096 maps are 1275.1 MB of the 1313.1 MB
 * total, i.e. 97%, and the whole rest of the corpus is 38 MB. One halving of
 * those nineteen takes the set to 356.8 MB -- a 3.7x cut for a single mip step
 * off the source. The remaining steps are worth far less (2048->1024 saves
 * another 264 MB, 1024->512 only 69 MB) while the fidelity cost grows: 512
 * measures max 5/255 on 1.53% of pixels, 1024 max 2/255. A cap of 4096 is a
 * no-op on this corpus -- nothing exceeds it -- so 2048 is the first setting
 * that does anything at all, and it does most of the available work.
 *
 * THIS IS NOT FREE, AND IT IS NOT BYTE-IDENTICAL TO AN UNCAPPED BUILD. Any
 * frame using a >2048 texture changes. That is a deliberate trade, not an
 * oversight; set OVGL_MAX_TEX_SIZE=0 to render at authored resolution.
 *
 * BE CLEAR ABOUT WHAT THIS DOES NOT BUY. The cap runs AFTER stbi_load, because
 * a PNG has no partial-resolution decode: the full-size image must be
 * inflated + unfiltered before anything can be thrown away. It saves upload
 * bandwidth and VRAM. It does NOT save the cold-start PNG decode, which is
 * where franka_factory's ~9 s first step actually goes -- the first halving
 * still reads every source texel, so it costs a little decode-side time.
 *
 * THE BIGGER LEVER IS NOT THIS, AND IT IS NOW IN. 18 of those 19 textures are
 * six SimReady CubeBox maps stored three times over (cubebox_a02/a03/a04),
 * byte-identical by md5 -- pure duplication that a PATH key cannot see, so three
 * paths meant three decodes and three uploads. The content-hash dedup above
 * collapses them with zero fidelity loss and cuts the decode time this cap
 * cannot: at this 2048 default, 19 -> 7 textures, 19 -> 7 decodes, 389.3 -> 133.3
 * MB of mip chains and step 0 from 7.05 s to 3.07 s.
 *
 * ORDERING CONTRACT WITH THAT DEDUP. The cap must run after the decode and
 * before the memo/content-cache inserts, which is exactly where the call below
 * sits. It stays out of the dedup key because it is a pure function of the
 * decoded dimensions and a process-static cap, so identical source bytes always
 * take the identical cap decision -- two copies of one image cannot end up one
 * capped and one not. Content hits return before the decode and so adopt the
 * already-capped pixels of the entry they join. */
constexpr int g_kDefaultMaxTexSize = 2048;

int maxTextureSize()
{
    static const int s_kMaximumTextureSize = []
    {
        const char* e = std::getenv("OVGL_MAX_TEX_SIZE");
        if (!e || !e[0])
            return g_kDefaultMaxTexSize;
        char* end = nullptr;
        const long v = std::strtol(e, &end, 10);
        if (end == e || *end)
            return g_kDefaultMaxTexSize; /* unparseable */
        if (v == 0)
            return 0; /* explicit opt-out */
        return v > 0 ? (int)v : g_kDefaultMaxTexSize;
    }();
    return s_kMaximumTextureSize;
}

/* Halving BOX downsample of an RGBA8 image until both dimensions are <= cap.
 * Returns false (leaving the caller's pixels alone) when nothing to do.
 *
 * The fork's version (material.c:833-866) is labeled "Simple box downsample"
 * but actually POINT-samples the source at the reduced grid -- `memcpy(resized +
 * ..., pixels + (oy * *w + ox) * 4, 4)` with ox/oy the truncated scaled index.
 * On a 4096 -> 512 reduction that keeps 1 texel in 64 and aliases hard on
 * exactly the normal/ORM detail maps this corpus ships. Averaging the 2x2 block
 * per halving step is the same single linear pass over each level, produces the
 * identical output dimensions (the fork's nw = (nw + 1) / 2 sequence), and is
 * what the comment already claimed. */
bool boxDownsampleRgba8(std::vector<uint8_t>& pixels, int& w, int& h, int cap)
{
    if (cap <= 0 || (w <= cap && h <= cap) || w <= 0 || h <= 0)
        return false;
    while (w > cap || h > cap)
    {
        const int nw = (w + 1) / 2;
        const int nh = (h + 1) / 2;
        std::vector<uint8_t> next((size_t)nw * (size_t)nh * 4);
        for (int y = 0; y < nh; ++y)
        {
            const int y0 = y * 2;
            const int y1 = (y0 + 1 < h) ? y0 + 1 : y0; /* clamp on odd sizes */
            for (int x = 0; x < nw; ++x)
            {
                const int x0 = x * 2;
                const int x1 = (x0 + 1 < w) ? x0 + 1 : x0;
                const uint8_t* a = &pixels[((size_t)y0 * w + x0) * 4];
                const uint8_t* b = &pixels[((size_t)y0 * w + x1) * 4];
                const uint8_t* c = &pixels[((size_t)y1 * w + x0) * 4];
                const uint8_t* d = &pixels[((size_t)y1 * w + x1) * 4];
                uint8_t* o = &next[((size_t)y * nw + x) * 4];
                for (int k = 0; k < 4; ++k)
                    o[k] = (uint8_t)((a[k] + b[k] + c[k] + d[k] + 2) / 4);
            }
        }
        pixels.swap(next);
        w = nw;
        h = nh;
    }
    return true;
}

/* Point `resolved` at a GPU texture for these decoded pixels, reusing an entry
 * when one already carries the same (content, color space) and appending one
 * that SHARES the pixel buffer when it does not. `content` empty disables
 * content dedup for this registration (unhashable bytes, or OVGL_TEXTURE_DEDUP=0)
 * and leaves the entry addressable by path alone, i.e. exactly the pre-dedup
 * behavior.
 *
 * `pixels` and `content` are taken BY VALUE on purpose: every caller but one
 * hands over an element of g_texture_pixels / g_texture_reg, and this function
 * push_back()s onto both -- so a by-reference parameter would dangle the moment
 * either vector reallocated, mid-function.
 *
 * The pixels handed in are always POST-cap: the only site that decodes calls
 * boxDownsampleRgba8 before it gets here, and every other site passes pixels
 * that already came through that one. */
int registerTexture(const std::string& resolved,
                    bool srgb,
                    std::shared_ptr<std::vector<uint8_t>> pixels,
                    int w,
                    int h,
                    std::string content)
{
    const int shared = findContentEntry(content, srgb, /*exact=*/true);
    if (shared >= 0)
    {
        ++g_texture_reg[(size_t)shared].paths;
        g_texture_index[resolved] = shared;
        return shared;
    }

    GpuTextureData tex{};
    tex.pixels = pixels->data();
    tex.width = w;
    tex.height = h;
    tex.is_srgb = srgb ? 1 : 0;
    g_texture_pixels.push_back(std::move(pixels));
    g_textures.push_back(tex);
    g_texture_reg.push_back(TextureRegistration{ content, 1 });

    const int index = (int)g_textures.size() - 1;
    if (!content.empty())
        g_texture_content_index[contentSlotKey(content, srgb)] = index;
    g_texture_index[resolved] = index;
    return index;
}

/* Decode an image to RGBA8 and register it, returning its texture index (dedup'd by path,
 * then by source-byte content). Handles both a plain file and a "package[inner]" packaged
 * path. -1 on failure. */
int findOrAddTexture(const std::string& resolved,
                     bool srgb,
                     isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache& archive_cache)
{
    if (resolved.empty())
        return -1;
    auto cached = g_texture_index.find(resolved);
    if (cached != g_texture_index.end())
    {
        const int index = cached->second;
        if (index < 0)
            return index; /* negative cache */
        /* A texture referenced as BOTH color and data (rare, but a UsdUVTexture can be reused)
         * must be decoded once; keep the first slot's color space -- linear is the safe loser,
         * since sampling data through sRGB corrupts it. */
        if (!srgb && g_textures[(size_t)index].is_srgb)
        {
            if (g_texture_reg[(size_t)index].paths <= 1)
            {
                g_textures[(size_t)index].is_srgb = 0;
                /* The entry no longer occupies the sRGB slot for its content, and
                 * it now occupies the linear one unless something else already
                 * does (in which case leave that one alone -- two linear entries
                 * for one image is a missed merge, never a wrong pixel). */
                const std::string& content = g_texture_reg[(size_t)index].content;
                if (!content.empty())
                {
                    g_texture_content_index.erase(contentSlotKey(content, true));
                    g_texture_content_index.emplace(contentSlotKey(content, false), index);
                }
            }
            else
            {
                /* CONTENT-MERGED entry: other paths are bound to it as sRGB, so
                 * demoting in place would drag them linear too -- the exact
                 * corruption the (content, is_srgb) key exists to prevent. Split
                 * instead: this path gets its own linear entry over the SAME
                 * decoded pixels. Unreachable until a content merge has actually
                 * happened, so the single-path case above stays bit-for-bit the
                 * pre-dedup behavior.
                 *
                 * The one semantic this cannot preserve: pre-dedup, the in-place
                 * demotion above reached BACKWARDS -- a material that had already
                 * captured this index as a color slot silently became linear
                 * too. Splitting cannot reproduce that, because those captured
                 * indices are already sitting in GpuMaterialParams::tex_indices
                 * and there is nothing left to rewrite. What happens instead is
                 * that every binding keeps the color space it asked for, which
                 * is the better answer as well as the only implementable one.
                 *
                 * MEASURED, and it is the ONLY frame difference this whole
                 * change can produce. A fixture that binds one path as color on
                 * two materials and as a linear slot on a third (so the entry is
                 * content-merged, paths=2, before the demote) renders 84339
                 * pixels differently between the two arms: MAE 5.012, max
                 * channel 90. Judged against a third render in which the linear
                 * binding is simply absent -- where the color materials are
                 * unarguably sRGB -- THIS branch is bit-exact (MAE 0.000 over
                 * those pixels) and the in-place demotion is MAE 39.937, max 90,
                 * mean level 188.1 against the correct 148.2. The split is the
                 * arm that is right; the divergence is a bug fix, not a
                 * regression. No scene on this bench reaches it: franka_factory
                 * is byte-identical across the arms. */
                --g_texture_reg[(size_t)index].paths;
                return registerTexture(resolved, false, g_texture_pixels[(size_t)index], g_textures[(size_t)index].width,
                                       g_textures[(size_t)index].height, g_texture_reg[(size_t)index].content);
            }
        }
        return index;
    }

    /* OmniClient returns a cache-local file while the request remains alive.
     * A packaged path localizes only the outer archive and retains its member
     * suffix for the USDZ reader below. Keep this object alive through the
     * complete file read so OmniClient cannot reclaim the cache entry. */
    using isaacsim::ovgl_viewport::debug::details::ovgl::LocalAssetFile;
    const size_t source_bracket = resolved.find('[');
    const bool is_packaged = source_bracket != std::string::npos && resolved.back() == ']';
    const std::string outer_source = is_packaged ? resolved.substr(0, source_bracket) : resolved;
    LocalAssetFile local_asset;
    std::string asset_error;
    if (!local_asset.open(outer_source, asset_error))
    {
        g_err = std::move(asset_error);
        g_texture_index[resolved] = -1;
        return -1;
    }
    const std::string local_source =
        is_packaged ? local_asset.getPath() + resolved.substr(source_bracket) : local_asset.getPath();

    /* Decode memo: an unchanged file (same resolved path, same size + mtime)
     * reuses the pixels decoded by an earlier build. Keyed by path AND file
     * identity so replacing the file's contents in place misses and decodes.
     * Carries the source digest, so this hit still joins the content dedup
     * without re-reading the file. Its pixels are already capped. */
    const std::string identity = textureFileIdentity(local_source);
    const std::string memo_key = identity.empty() ? std::string() : resolved + "|" + identity;
    if (!memo_key.empty())
    {
        auto memo = g_texture_decode_memo.find(memo_key);
        if (memo != g_texture_decode_memo.end())
        {
            memo->second.last_used_build = g_texture_build_serial;
            return registerTexture(
                resolved, srgb, memo->second.pixels, memo->second.width, memo->second.height, memo->second.content);
        }
    }

    /* Source bytes first, digest second, decode LAST -- see the block comment on
     * g_texture_content_index. The packaged branch already had to materialize
     * the bytes; the plain-file branch now does too, replacing stbi_load()'s own
     * stdio read rather than adding one. */
    std::vector<uint8_t> bytes;
    const size_t bracket = local_source.find('[');
    if (bracket != std::string::npos && local_source.back() == ']')
    {
        const std::string archive = local_source.substr(0, bracket);
        const std::string inner = local_source.substr(bracket + 1, local_source.size() - bracket - 2);
        std::string archive_error;
        if (!archive_cache.readEntry(archive, inner, bytes, archive_error))
        {
            g_err = std::move(archive_error);
            g_texture_index[resolved] = -1;
            return -1;
        }
    }
    else if (!readWholeFile(local_source, bytes))
    {
        g_err = "texture read failed: " + resolved;
        g_texture_index[resolved] = -1; /* negative-cache: do not retry every rebuild */
        return -1;
    }
    if (bytes.empty())
    {
        /* stbi would reject this anyway, but reaching it with an empty vector
         * means hashing and decoding from a possibly-null data() pointer. */
        g_err = "texture is empty: " + resolved;
        g_texture_index[resolved] = -1;
        return -1;
    }
    /* USDZ note: this hashes the EXTRACTED MEMBER bytes, not the package path --
     * the same image shipped inside two .usdz packages still merges. */
    const std::string content = contentDigest(bytes);

    /* Same bytes, same color space, already registered this build: no read of
     * the twin, no decode, no second GL texture. The pixels adopted here were
     * capped by whichever registration decoded them. */
    const int exact = findContentEntry(content, srgb, /*exact=*/true);
    if (exact >= 0)
    {
        ++g_texture_reg[(size_t)exact].paths;
        g_texture_index[resolved] = exact;
        if (!memo_key.empty())
            g_texture_decode_memo[memo_key] =
                TextureDecodeMemoEntry{ g_texture_pixels[(size_t)exact], g_textures[(size_t)exact].width,
                                        g_textures[(size_t)exact].height, g_texture_build_serial, content };
        return exact;
    }
    /* Same bytes under the OTHER color space: reuse the decode, take a second
     * GL texture for the differing internal format. */
    const int sibling = findContentEntry(content, srgb, /*exact=*/false);
    if (sibling >= 0)
    {
        const int index = registerTexture(resolved, srgb, g_texture_pixels[(size_t)sibling],
                                          g_textures[(size_t)sibling].width, g_textures[(size_t)sibling].height, content);
        if (!memo_key.empty())
            g_texture_decode_memo[memo_key] =
                TextureDecodeMemoEntry{ g_texture_pixels[(size_t)index], g_textures[(size_t)index].width,
                                        g_textures[(size_t)index].height, g_texture_build_serial, content };
        return index;
    }

    int w = 0, h = 0, channels = 0;
    stbi_uc* pixels = stbi_load_from_memory(bytes.data(), (int)bytes.size(), &w, &h, &channels, 4);
    if (!pixels || w <= 0 || h <= 0)
    {
        if (pixels)
            stbi_image_free(pixels);
        g_err = "texture decode failed: " + resolved;
        g_texture_index[resolved] = -1; /* negative-cache: do not retry every rebuild */
        return -1;
    }

    ++g_texture_decodes;
    const size_t nbytes = (size_t)w * (size_t)h * 4;
    auto owned = std::make_shared<std::vector<uint8_t>>(pixels, pixels + nbytes);
    stbi_image_free(pixels);
    /* Cap HERE -- after the decode, before BOTH the memo store and the content
     * index -- so every later sharer of this digest, whether it arrives through
     * the memo or through findContentEntry, gets the one capped result. The
     * cap is deliberately absent from the content key: it is a pure function of
     * (w, h, the process-static cap), so equal source bytes are already
     * guaranteed the same decision, and keying on it would only split entries
     * that are in fact identical. */
    boxDownsampleRgba8(*owned, w, h, maxTextureSize());
    if (!memo_key.empty())
        g_texture_decode_memo[memo_key] = TextureDecodeMemoEntry{ owned, w, h, g_texture_build_serial, content };
    return registerTexture(resolved, srgb, owned, w, h, content);
}

/* Load-profile texture accounting (OVGL_PROFILE_LOAD=1). Reports what the build
 * actually paid for textures: distinct GPU textures handed to gl, distinct
 * paths resolved into them, stbi decodes performed, and resident VRAM.
 *
 * VRAM is the FULL MIP CHAIN, not level 0. GpuOpenGles.c uploads RGBA8 and
 * calls glGenerateMipmap on every material texture (GpuOpenGles.c:1422), and nothing in
 * this renderer is compressed -- there is not one glCompressedTexImage2D call
 * in gl -- so the resident cost is sum over levels of max(1, w>>l) *
 * max(1, h>>l) * 4, i.e. ~4/3 of level 0. gpu->allocated_bytes counts level 0
 * only (GpuOpenGles.c:1432-1433) and so understates by that third; count it properly here
 * rather than "fix" the GPU counter, which other subsystems compare against.
 *
 * The dimensions counted are post-cap, so this reports what the GPU actually
 * holds under OVGL_MAX_TEX_SIZE, not what the source authored. */
void logTextureProfile()
{
    uint64_t level0 = 0, mipchain = 0;
    for (const GpuTextureData& t : g_textures)
    {
        int w = t.width > 0 ? t.width : 1;
        int h = t.height > 0 ? t.height : 1;
        level0 += (uint64_t)w * (uint64_t)h * 4;
        for (;;)
        {
            mipchain += (uint64_t)w * (uint64_t)h * 4;
            if (w == 1 && h == 1)
                break;
            w = w > 1 ? w / 2 : 1;
            h = h > 1 ? h / 2 : 1;
        }
    }
    std::fprintf(stderr,
                 "[tex] gpu_textures=%zu paths=%zu decodes=%zu hash=%.1fms "
                 "level0=%.1fMB mipchain=%.1fMB\n",
                 g_textures.size(), g_texture_index.size(), g_texture_decodes, g_texture_hash_ms, level0 / 1048576.0,
                 mipchain / 1048576.0);
}

static std::string resolveTokenId(ovstage_instance_t* stage, uint64_t identifier);

/* Read an asset-valued attribute. Official OVStage stores the authored and
 * resolved values as a pair of token IDs. Retain the legacy NUL-delimited
 * representation for stages produced by the earlier prototype backend. */
std::string readAssetResolved(ovstage_instance_t* stage,
                              ovstage_ordinal_t ordinal,
                              const std::string& primPath,
                              const std::string& attributeName)
{
    std::vector<uint8_t> bytes;
    if (!readAttributeHost(stage, ordinal, primPath, attributeName, bytes) || bytes.empty())
        return "";

    if (bytes.size() == 2 * sizeof(uint64_t))
    {
        uint64_t tokenIdentifiers[2]{};
        std::memcpy(tokenIdentifiers, bytes.data(), sizeof(tokenIdentifiers));
        const std::string authored = resolveTokenId(stage, tokenIdentifiers[0]);
        if (!authored.empty())
        {
            const std::string resolved = resolveTokenId(stage, tokenIdentifiers[1]);
            return resolved.empty() ? authored : resolved;
        }
    }

    const auto authoredEnd = std::find(bytes.begin(), bytes.end(), uint8_t{ 0 });
    const std::string authored(bytes.begin(), authoredEnd);
    if (authoredEnd != bytes.end() && std::next(authoredEnd) != bytes.end())
    {
        const auto resolvedBegin = std::next(authoredEnd);
        const auto resolvedEnd = std::find(resolvedBegin, bytes.end(), uint8_t{ 0 });
        const std::string resolved(resolvedBegin, resolvedEnd);
        if (!resolved.empty())
            return resolved;
    }
    return authored;
}

/* Official 0.1 ID-semantic decode (P1.3).
 * TOKEN_ID / RELATIONSHIP_PATH_ID / CONNECTION_PATH_ID columns now carry
 * pre-interned u64 ids; NONE-tagged columns from the ovrtx backend still carry
 * raw UTF-8. readAttributeHost/readBatchedAttributes surface the column's SEMANTIC
 * tag from the read group (ovpopulation writes it at serialize time), and
 * readStringAttribute picks the decode space by that tag: TOKEN_ID rows resolve
 * token-ONLY, RELATIONSHIP/CONNECTION rows resolve path-first.
 *
 * For untagged (NONE) columns the decode stays dictionary-led: dictionary ids
 * are small sequential counters — resolving succeeds only for a live handle —
 * while any >= 1-char UTF-8 string read as a little-endian u64 is
 * astronomically larger than any live id, so a successful resolution
 * discriminates IDS from RAW UTF-8 (no corruption possible there).
 *
 * CAUTION: that discriminator does NOT extend to token id vs path id. Token
 * ids and path ids are independent small counters in overlapping numeric
 * ranges, so a token id nearly always also resolves as some unrelated path
 * (and vice versa). The COLUMN SEMANTIC — or, for untagged columns, the
 * attribute's known value kind — must choose the space: token-valued columns
 * (SEMANTIC_TOKEN_ID: purpose, visibility, axis, orientation, joints, ...) go
 * through resolveTokenId(); only relationship/connection targets may use
 * the path-first resolveU64Id(). Violations decode a token id into some
 * unrelated prim path SILENTLY: see the skel deformer reader below (collapsed
 * joint hierarchies) and test_axis_orientation_token_decode.py (quadric
 * `axis` fell back to Z, `orientation` comparisons went false). */
static std::string resolveTokenId(ovstage_instance_t* stage, uint64_t identifier)
{
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary || identifier == 0)
        return "";
    ovx_string_t stringValue{};
    if (path_dictionary_get_strings_from_tokens(dictionary, &identifier, 1, &stringValue).status == OVX_API_SUCCESS &&
        stringValue.length)
        return std::string(stringValue.ptr, stringValue.length);
    return "";
}

static std::string resolveU64Id(ovstage_instance_t* stage, uint64_t identifier, uint64_t tokenPairIdentifier)
{
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary || identifier == 0)
        return "";
    /* path id first (relationship/connection); fall back to token id. */
    size_t capacity = 64;
    while (capacity <= (1u << 12))
    {
        std::vector<ovx_token_t> buffer(capacity);
        ovx_token_t* tokensForPath = nullptr;
        size_t tokenCount = 0;
        size_t processedCount = 0;
        if (path_dictionary_get_tokens_from_paths(
                dictionary, &identifier, 1, buffer.data(), capacity, &tokensForPath, &tokenCount, &processedCount)
                    .status == OVX_API_SUCCESS &&
            processedCount >= 1)
        {
            std::string output;
            for (size_t i = 0; i < tokenCount; ++i)
            {
                ovx_string_t stringValue{};
                if (path_dictionary_get_strings_from_tokens(dictionary, &tokensForPath[i], 1, &stringValue).status !=
                    OVX_API_SUCCESS)
                    return "";
                output += '/';
                output.append(stringValue.ptr ? stringValue.ptr : "", stringValue.length);
            }
            if (tokenPairIdentifier)
            {
                ovx_string_t tokenPairString{};
                if (path_dictionary_get_strings_from_tokens(dictionary, &tokenPairIdentifier, 1, &tokenPairString).status ==
                        OVX_API_SUCCESS &&
                    tokenPairString.length)
                    output += "." + std::string(tokenPairString.ptr, tokenPairString.length);
            }
            return output;
        }
        if (processedCount == 0 && tokenCount == 0 && capacity < (1u << 12))
        {
            capacity *= 2;
            continue;
        }
        break;
    }
    ovx_string_t stringValue{};
    if (path_dictionary_get_strings_from_tokens(dictionary, &identifier, 1, &stringValue).status == OVX_API_SUCCESS &&
        stringValue.length)
        return std::string(stringValue.ptr, stringValue.length);
    return "";
}

/* If `bytes` is an ID row (8- or 16-byte stride uint64 values that resolve through the
 * dictionary), return the FIRST element's string; else empty. */
static std::string tryDecodeIdRow(ovstage_instance_t* stage, const std::vector<uint8_t>& bytes)
{
    if (bytes.size() >= 16 && bytes.size() % 16 == 0)
    {
        uint64_t pathIdentifier = 0;
        uint64_t tokenIdentifier = 0;
        std::memcpy(&pathIdentifier, bytes.data(), 8);
        std::memcpy(&tokenIdentifier, bytes.data() + 8, 8);
        const std::string decoded = resolveU64Id(stage, pathIdentifier, tokenIdentifier);
        if (!decoded.empty())
            return decoded;
    }
    if (bytes.size() >= 8 && bytes.size() % 8 == 0)
    {
        uint64_t identifier = 0;
        std::memcpy(&identifier, bytes.data(), 8);
        return resolveU64Id(stage, identifier, 0);
    }
    return "";
}

/* Read a token/path-valued attribute as a string (material:binding target,
 * outputs:surface connection, quadric axis, mesh orientation). Official ID
 * columns carry u64 ids (resolved through the dictionary); NONE-tagged
 * backend columns carry UTF-8. */
std::string readStringAttribute(ovstage_instance_t* stage,
                                ovstage_ordinal_t ordinal,
                                const std::string& primPath,
                                const std::string& attributeName)
{
    std::vector<uint8_t> bytes;
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    if (!readAttributeHost(stage, ordinal, primPath, attributeName, bytes, &semantic) || bytes.empty())
        return "";
    /* Semantic-driven decode: the store's column tag picks the handle space,
     * so a token/path id collision can never select the wrong dictionary. */
    if (semantic == OVSTAGE_SEMANTIC_TOKEN_ID)
    {
        if (bytes.size() < 8)
            return "";
        uint64_t identifier = 0;
        std::memcpy(&identifier, bytes.data(), 8);
        /* Token ids are interned eagerly at population; an unresolvable id is
         * a dead handle — treat as unauthored. NEVER fall back to the path
         * space: that is exactly the axis/orientation collision hazard. */
        return resolveTokenId(stage, identifier);
    }
    if (semantic == OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID || semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID)
    {
        /* Path-first (bit-identical to the pre-semantic decode for these). */
        return tryDecodeIdRow(stage, bytes);
    }
    {
        /* Untagged (NONE) columns from foreign producers: fall back to the
         * dictionary-led decode. Token ids and path ids are separate handle
         * spaces in the same dictionary, both small counters — the ATTRIBUTE
         * must decide which space an 8-byte row belongs to. Known TOKEN-kind
         * attributes resolve as tokens; everything else (relationship/
         * connection targets) resolves path-first with a token fallback. */
        static const std::unordered_set<std::string> s_kTokenKindAttrs = {
            "purpose", "visibility", "info:id", "inputs:sourceColorSpace", "info:mdl:sourceAsset:subIdentifier",
            "axis",    "orientation"
        };
        if (s_kTokenKindAttrs.count(attributeName) && bytes.size() == 8)
        {
            uint64_t identifier = 0;
            std::memcpy(&identifier, bytes.data(), 8);
            const std::string token = resolveTokenId(stage, identifier);
            if (!token.empty())
                return token;
        }
        const std::string identifier = tryDecodeIdRow(stage, bytes);
        if (!identifier.empty())
            return identifier;
    }
    if (std::getenv("OVGL_DEBUG_MAT"))
        std::fprintf(stderr, "[ovgl-mat] %s.%s -> %zu bytes, first='%c'(0x%02x)\n", primPath.c_str(),
                     attributeName.c_str(), bytes.size(), (bytes[0] >= 32 ? (char)bytes[0] : '?'), bytes[0]);
    std::string value((const char*)bytes.data(), bytes.size());
    const size_t nullPosition = value.find('\0');
    if (nullPosition != std::string::npos)
        value.resize(nullPosition);
    return value;
}

bool includedPurpose(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& primPath)
{
    std::string path = primPath;
    for (;;)
    {
        const std::string purpose = readStringAttribute(stage, ordinal, path, "purpose");
        if (purpose == "default" || purpose == "render")
            return true;
        if (purpose == "guide" || purpose == "proxy")
            return false;
        /* ovpopulation omits the unauthored schema fallback, so missing means
         * keep walking. An authored default is present above and terminates
         * inheritance, matching UsdGeomImageable::ComputePurpose(). */
        size_t slash = path.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break;
        path.resize(slash);
    }
    /* Match the ovrtx/usdview default render-product policy:
     * includedPurposes = ["default", "render"]. Missing purpose inherits/defaults to
     * "default"; guide/proxy collision or proxy geometry is excluded. */
    return true;
}

bool includedVisibility(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& primPath)
{
    std::string path = primPath;
    for (;;)
    {
        /* USD visibility is inherited and has no descendant override for an
         * invisible ancestor. Missing/"inherited" therefore keeps walking. */
        if (readStringAttribute(stage, ordinal, path, "visibility") == "invisible")
            return false;
        const size_t slash = path.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break;
        path.resize(slash);
    }
    return true;
}

void defaultMaterial(GpuMaterialParams* m)
{
    std::memset(m, 0, sizeof(*m));
    m->base_color[0] = m->base_color[1] = m->base_color[2] = 0.62f;
    m->base_color[3] = 1.0f;
    // Neutral mid-gray (0.62) matte-ish plastic for the unresolved-material fallback
    // (e.g. g1's MDL OmniPBR, which doesn't read as a UsdPreviewSurface). roughness 0.55:
    // raising it further pulls the IBL toward the cosine-averaged env *irradiance* (a very
    // saturated blue for the clear-sky table_mountain HDR), which turns floors/feet blue --
    // the real "blue flashes" fix is the IBL chroma clamp in the PBR shader, not roughness.
    m->metallic = 0.0f;
    m->roughness = 0.55f;
    m->opacity = 1.0f;
    m->ior = 1.5f;
    m->occlusion = 1.0f;
    m->normal_scale = 1.0f;
    m->udim_scale_u = 1.0f;
    m->udim_scale_v = 1.0f;
    for (int i = 0; i < 8; i++)
        m->tex_indices[i] = -1;
    m->opacity_texture_channel = 0;
    m->roughness_texture_channel = 1;
    m->metallic_texture_channel = 2;
    m->specular_color[0] = m->specular_color[1] = m->specular_color[2] = m->specular_color[3] = 1.0f;
    m->roughness_tex_scale = 1.0f;
}

/* float4-valued shader input (UsdUVTexture inputs:scale / inputs:bias). */
bool readColor4(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& prim, const std::string& attr, float out[4])
{
    std::vector<uint8_t> b;
    if (readAttributeHost(stage, ordinal, prim, attr, b) && b.size() >= 4 * sizeof(float))
    {
        const float* c = (const float*)b.data();
        out[0] = c[0];
        out[1] = c[1];
        out[2] = c[2];
        out[3] = c[3];
        return true;
    }
    return false;
}

bool readColor3(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& prim, const std::string& attr, float out[3])
{
    std::vector<uint8_t> b;
    if (readAttributeHost(stage, ordinal, prim, attr, b) && b.size() >= 3 * sizeof(float))
    {
        const float* c = (const float*)b.data();
        out[0] = c[0];
        out[1] = c[1];
        out[2] = c[2];
        return true;
    }
    return false;
}

/* Return the selected scalar output from a UsdUVTexture connection target such as
 * "/Looks/Roughness.outputs:r". Unknown/custom output names retain the caller's
 * material-family default (e.g. G/B for a packed ORM map). */
int textureOutputChannel(const std::string& target, int fallback)
{
    const size_t dot = target.rfind('.');
    if (dot == std::string::npos)
        return fallback;
    const size_t colon = target.rfind(':');
    const size_t begin = colon != std::string::npos && colon > dot ? colon + 1 : dot + 1;
    const std::string output = target.substr(begin);
    if (output == "r")
        return 0;
    if (output == "g")
        return 1;
    if (output == "b")
        return 2;
    if (output == "a")
        return 3;
    return fallback;
}

std::string connectionPrimPath(std::string target)
{
    if (target.empty() || target[0] != '/')
        return "";
    const size_t dot = target.rfind('.');
    if (dot != std::string::npos)
        target.resize(dot);
    return target;
}

/* Validate one exact, single-source MaterialX arithmetic node. This is deliberately not a
 * general graph evaluator: following a multiply/subtract without its second operand would
 * silently change authored materials. Apple's active normal graph is specifically
 * subtract(in2=1) <- multiply(in2=2) <- image, which is equivalent to the built-in GLSL
 * normal decode (2 * texel - 1). Connected or different operands fail closed. */
bool exactTextureArithmeticInput(ovstage_instance_t* stage,
                                 ovstage_ordinal_t ordinal,
                                 const std::string& node,
                                 const char* expectedIdentifier,
                                 float expectedSecondInput,
                                 std::string& inputNode)
{
    inputNode.clear();
    if (readStringAttribute(stage, ordinal, node, "info:id") != expectedIdentifier)
        return false;
    if (!readStringAttribute(stage, ordinal, node, "inputs:in2.connect").empty())
        return false;
    const float secondInput = readScalar(stage, ordinal, node, "inputs:in2", std::numeric_limits<float>::quiet_NaN());
    if (!std::isfinite(secondInput) || std::fabs(secondInput - expectedSecondInput) > 1e-6f)
        return false;
    inputNode = connectionPrimPath(readStringAttribute(stage, ordinal, node, "inputs:in1.connect"));
    return !inputNode.empty();
}

std::string textureImageSourcePath(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, std::string target, int textureSlot)
{
    std::string node = connectionPrimPath(std::move(target));
    if (node.empty())
        return "";
    if (!readAssetResolved(stage, ordinal, node, "inputs:file").empty())
        return node;
    if (textureSlot != TEX_NORMAL)
        return "";

    std::string multiply;
    if (!exactTextureArithmeticInput(stage, ordinal, node, "ND_subtract_vector3FA", 1.0f, multiply))
        return "";
    std::string image;
    if (!exactTextureArithmeticInput(stage, ordinal, multiply, "ND_multiply_vector3FA", 2.0f, image))
        return "";
    if (readStringAttribute(stage, ordinal, image, "info:id") != "ND_image_vector3")
        return "";
    return readAssetResolved(stage, ordinal, image, "inputs:file").empty() ? "" : image;
}

/* Resolve the inherited Material path separately from its terminal. Scene-build discovery first
 * batches material:binding over meshes and all ancestors, then batches output targets over the
 * distinct returned materials. Keeping these steps separate avoids one point query per graph
 * edge while preserving the exact same binding and fallback rules. */
std::string boundMaterialPath(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& primPath)
{
    /* Material bindings INHERIT: a mesh with no binding of its own uses the nearest ancestor's.
     * Reading only the mesh's own material:binding would leave every asset that binds on a
     * parent Xform (Apple's biplane binds on the rig group, so its whole body) falling back to
     * the gray displayColor default -- textured in the file, flat gray on screen. nanousd's
     * materials_find_binding walks up the hierarchy for exactly this reason; do the same.
     * (The walk below IS that fix, not a description of current behavior.) */
    std::string materialPath;
    for (std::string currentPath = primPath; !currentPath.empty();)
    {
        materialPath = readStringAttribute(stage, ordinal, currentPath, "material:binding");
        if (!materialPath.empty() && materialPath[0] == '/')
            break;
        materialPath.clear();
        const size_t slash = currentPath.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break; /* stop above the root */
        currentPath.resize(slash);
    }
    if (materialPath.empty() || materialPath[0] != '/')
    {
        auto subsetBinding = g_subset_binding.find(primPath);
        if (subsetBinding != g_subset_binding.end())
            materialPath = subsetBinding->second;
    }
    return (!materialPath.empty() && materialPath[0] == '/') ? materialPath : "";
}

static const char* g_kMaterialOutputAttrs[] = {
    "outputs:surface.connect", "outputs:mdl:surface.connect", "outputs:mtlx:surface.connect",
    "outputs:surface",         "outputs:mdl:surface",         "outputs:mtlx:surface",
};

/* Return surface targets in renderer-context priority order. A material can retain a connection
 * to an inactive shader while publishing an active terminal for another context (Apple's USDZ
 * assets do exactly this). Population intentionally excludes the inactive prim itself, but the
 * connection authored on the active Material remains visible, so terminal selection must later
 * verify that the target prim is present in the populated scene. */
std::vector<std::string> materialShaderCandidates(ovstage_instance_t* stage,
                                                  ovstage_ordinal_t ordinal,
                                                  const std::string& materialPath)
{
    std::vector<std::string> candidates;
    std::unordered_set<std::string> seen;
    for (const char* output : g_kMaterialOutputAttrs)
    {
        std::string target = readStringAttribute(stage, ordinal, materialPath, output);
        if (target.empty() || target[0] != '/')
            continue;
        /* The target is normally a property path (".../Shader.outputs:surface"). */
        const size_t dot = target.rfind('.');
        if (dot != std::string::npos)
            target.resize(dot);
        if (seen.insert(target).second)
            candidates.push_back(std::move(target));
    }
    return candidates;
}

/* Derive one material's populated surface-shader prim path. Output columns and candidate
 * usd-prim-type metadata are prefetched for every distinct material at the build ordinal; the
 * naming-convention fallback remains unchanged. */
std::string materialShaderPath(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::string& materialPath)
{
    if (materialPath.empty() || materialPath[0] != '/')
        return "";
    /* The Material's surface output is a CONNECTION to the shader, and ovpopulation stores a
     * connection in its own "<name>.connect" column (the plain column stays empty, because a
     * connected property has no authored value). Reading only the plain name therefore always
     * missed, and every asset silently fell through to the "<Material>/Shader" naming
     * convention below -- which is fine for Kit-authored materials and wrong for everything
     * else: Apple's usdz names its shader after the material ("teapotmaterial"), so the lookup
     * landed on a prim that does not exist and the asset rendered untextured and unshaded. */
    for (const std::string& candidate : materialShaderCandidates(stage, ordinal, materialPath))
    {
        std::vector<uint8_t> prim_type;
        if (readAttributeHost(stage, ordinal, candidate, "usd-prim-type", prim_type) && !prim_type.empty())
        {
            return candidate;
        }
    }
    return materialPath + "/Shader";
}

/* Fill GpuMaterialParams (+ base color) from an already-resolved shader graph.
 * Returns true when the caller supplied a shader path. */
bool readMaterial(ovstage_instance_t* stage,
                  ovstage_ordinal_t ordinal,
                  const std::string& shader,
                  GpuMaterialParams* out,
                  float base_out[3],
                  isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache& archive_cache)
{
    defaultMaterial(out);
    if (shader.empty())
        return false;
    float c[3];
    if (readColor3(stage, ordinal, shader, "inputs:diffuseColor", c))
    {
        out->base_color[0] = c[0];
        out->base_color[1] = c[1];
        out->base_color[2] = c[2];
        base_out[0] = c[0];
        base_out[1] = c[1];
        base_out[2] = c[2];
    }
    out->metallic = readScalar(stage, ordinal, shader, "inputs:metallic", 0.0f);
    out->roughness = readScalar(stage, ordinal, shader, "inputs:roughness", 0.5f);
    out->ior = readScalar(stage, ordinal, shader, "inputs:ior", 1.5f);
    out->opacity = readScalar(stage, ordinal, shader, "inputs:opacity", 1.0f);
    out->opacity_threshold = readScalar(stage, ordinal, shader, "inputs:opacityThreshold", 0.0f);
    out->clearcoat = readScalar(stage, ordinal, shader, "inputs:clearcoat", 0.0f);
    out->clearcoat_roughness = readScalar(stage, ordinal, shader, "inputs:clearcoatRoughness", 0.01f);
    out->use_specular_workflow =
        readBooleanAttribute(stage, ordinal, shader, "inputs:useSpecularWorkflow", false) ? 1 : 0;
    if (readColor3(stage, ordinal, shader, "inputs:specularColor", c))
    {
        out->specular_color[0] = c[0];
        out->specular_color[1] = c[1];
        out->specular_color[2] = c[2];
        out->specular_color[3] = 1.0f;
    }
    if (readColor3(stage, ordinal, shader, "inputs:emissiveColor", c))
    {
        out->emissive_color[0] = c[0];
        out->emissive_color[1] = c[1];
        out->emissive_color[2] = c[2];
        /* UsdPreviewSurface's emissiveColor is already the emitted radiance color; it has
         * no separate intensity input. The built-in shader stores an internal multiplier in
         * .a, so an authored constant must explicitly enable it. */
        out->emissive_color[3] = 1.0f;
    }

    /* USD-embedded MaterialX terminal nodes.
     * MaterialX authored into USD has a normal UsdShade material binding, but chooses the
     * terminal through outputs:mtlx:surface and names its PBR inputs differently from
     * UsdPreviewSurface.  Map the two ubiquitous terminal nodes to the renderer's built-in PBR
     * model.  This is intentionally a direct-constant subset: it does not pretend to evaluate
     * arbitrary MaterialX node graphs or raw .mtlx documents (which require MaterialX codegen). */
    const std::string shader_id = readStringAttribute(stage, ordinal, shader, "info:id");
    const bool is_mtlx_terminal =
        shader_id == "ND_standard_surface_surfaceshader" || shader_id == "ND_open_pbr_surface_surfaceshader";
    if (is_mtlx_terminal)
    {
        const bool is_standard_surface = shader_id == "ND_standard_surface_surfaceshader";
        /* Both nodedefs default their transmission tint and emission color to white.  The
         * generic PBR defaults are black for those fields, so establish the MaterialX defaults
         * before applying any authored direct constants. */
        out->transmission_color[0] = 1.0f;
        out->transmission_color[1] = 1.0f;
        out->transmission_color[2] = 1.0f;
        out->emissive_color[0] = 1.0f;
        out->emissive_color[1] = 1.0f;
        out->emissive_color[2] = 1.0f;
        /* The MaterialX nodedefs for both supported direct terminals default to a neutral
         * .8 base color.  Those defaults are not materialized as USD shader attributes. */
        out->base_color[0] = 0.8f;
        out->base_color[1] = 0.8f;
        out->base_color[2] = 0.8f;
        base_out[0] = 0.8f;
        base_out[1] = 0.8f;
        base_out[2] = 0.8f;
        out->emissive_color[3] = is_standard_surface ?
                                     readScalar(stage, ordinal, shader, "inputs:emission", 0.0f) :
                                     readScalar(stage, ordinal, shader, "inputs:emission_luminance", 0.0f);
        if (readColor3(stage, ordinal, shader, "inputs:base_color", c))
        {
            out->base_color[0] = c[0];
            out->base_color[1] = c[1];
            out->base_color[2] = c[2];
            base_out[0] = c[0];
            base_out[1] = c[1];
            base_out[2] = c[2];
        }
        /* Standard Surface calls the diffuse lobe multiplier `base`; OpenPBR calls the
         * equivalent input `base_weight`.  Folding it into the renderer's base color is the
         * direct-constant approximation of that lobe without MaterialX code generation. */
        const float base_weight = is_standard_surface ? readScalar(stage, ordinal, shader, "inputs:base", 1.0f) :
                                                        readScalar(stage, ordinal, shader, "inputs:base_weight", 1.0f);
        out->base_color[0] *= base_weight;
        out->base_color[1] *= base_weight;
        out->base_color[2] *= base_weight;
        base_out[0] = out->base_color[0];
        base_out[1] = out->base_color[1];
        base_out[2] = out->base_color[2];
        out->metallic = readScalar(stage, ordinal, shader, "inputs:metalness",
                                   readScalar(stage, ordinal, shader, "inputs:base_metalness", out->metallic));
        /* These direct terminals do not materialize nodedef defaults into the USD shader.
         * Preserve their authored MaterialX defaults instead of inheriting the generic
         * UsdPreviewSurface fallback (.5): Standard Surface=.2, OpenPBR=.3. */
        out->roughness =
            readScalar(stage, ordinal, shader, "inputs:specular_roughness", is_standard_surface ? 0.2f : 0.3f);
        out->ior = readScalar(stage, ordinal, shader, "inputs:specular_IOR",
                              readScalar(stage, ordinal, shader, "inputs:specular_ior", out->ior));
        if (is_standard_surface && readColor3(stage, ordinal, shader, "inputs:opacity", c))
        {
            /* Standard Surface opacity is color3 while the portable PBR shader has a scalar
             * alpha.  Use perceptual luminance as the stable scalar approximation. */
            out->opacity = 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
        }
        else
        {
            out->opacity = readScalar(stage, ordinal, shader, "inputs:opacity",
                                      readScalar(stage, ordinal, shader, "inputs:geometry_opacity", out->opacity));
        }
        out->clearcoat = readScalar(stage, ordinal, shader, "inputs:coat",
                                    readScalar(stage, ordinal, shader, "inputs:coat_weight", out->clearcoat));
        /* Standard Surface inherits coat_roughness=.1 from its nodedef; OpenPBR defaults to
         * zero. Direct USD terminals omit inherited values, so do not fall back to generic PBR. */
        out->clearcoat_roughness =
            readScalar(stage, ordinal, shader, "inputs:coat_roughness", is_standard_surface ? 0.1f : 0.0f);
        if (readColor3(stage, ordinal, shader, "inputs:emission_color", c))
        {
            out->emissive_color[0] = c[0];
            out->emissive_color[1] = c[1];
            out->emissive_color[2] = c[2];
        }
        if (readColor3(stage, ordinal, shader, "inputs:transmission_color", c))
        {
            out->transmission_color[0] = c[0];
            out->transmission_color[1] = c[1];
            out->transmission_color[2] = c[2];
        }
        out->transmission_weight =
            readScalar(stage, ordinal, shader, "inputs:transmission_weight",
                       readScalar(stage, ordinal, shader, "inputs:transmission", out->transmission_weight));
    }

    /* MDL materials (port of material.c's MDL paths).
     * NVIDIA's SimReady/Isaac assets author no UsdPreviewSurface at all: the Shader carries
     * info:mdl:sourceAsset (OmniPBR.mdl) and names its textures directly as asset-valued
     * inputs. Nothing below the UsdPreviewSurface branch would ever fire for them, so the whole
     * warehouse rendered as untextured gray. */
    const std::string mdl_source = readAssetResolved(stage, ordinal, shader, "info:mdl:sourceAsset");
    const bool is_mdl = !mdl_source.empty();
    if (is_mdl)
    {
        const size_t mdl_basename_offset = mdl_source.find_last_of("/\\");
        const std::string mdl_basename =
            mdl_source.substr(mdl_basename_offset == std::string::npos ? 0 : mdl_basename_offset + 1);
        /* Most supported MDL opacity maps use red. Main DrivesimPBR explicitly uses .w,
         * while DrivesimPBR_Opacity uses .x, so this must stay a source-specific flag. */
        out->opacity_texture_channel = mdl_basename == "DrivesimPBR.mdl" ? 3 : 0;

        /* V orientation. The GPU vertex pack already converts USD st for stb_image's
         * top-down rows, and that single conversion is what every shipping MDL family
         * needs: OmniPBR builds its UV from base::coordinate_source (an unflipped st)
         * and looks it up straight, while the Isaac/Unreal exports flip twice
         * (CustomizedUV0 = (u, 1-v), then a lookup at 1-y) and cancel back to the same
         * orientation. An extra shader flip here mirrors MDL textures against the
         * UsdPreviewSurface path -- including the distilled preview surfaces the
         * coordinator authors for these very materials -- so keep one convention. */
        out->v_flip = 0;

        /* Base color: diffuse_color_constant, multiplied by diffuse_tint.
         *
         * The MDL default for the constant is NOT white. OmniPBR.mdl declares
         * `diffuse_color_constant = color(0.2)`, and this comment used to claim
         * white -- a 5x albedo error on every OmniPBR material that leaves the
         * constant unauthored. Measured three ways against official ovrtx 0.4
         * on the mirrored franka_factory, whose ground_plane binds OmniPBR and
         * authors only an unresolvable `inputs:diffuse_texture`:
         *   authored nothing                -> official ground plane px 146
         *   diffuse_color_constant = (0.2)  -> 146   (identical: it IS the default)
         *   diffuse_tint = (1,1,1) only     -> 146   (the default survives a tint)
         *   diffuse_tint = (0.5,0.5,0.5)    -> 106   (= 0.5x in linear: 0.144 vs 0.287)
         *   diffuse_color_constant = (1)    -> 227
         * ovgl rendered all of those at 209-227. Applying the real default takes
         * the whole-frame cross-stack MAE on that scene from 60.4 to 7.0
         * (px>32 97.4% -> 2.7%), against a documented previous best of 10.6.
         *
         * Scoped to the OmniPBR family, which is the one measured here and the
         * one the Isaac/SimReady content uses. Other families (AdvancedPBR,
         * DrivesimPBR, OmniSurface) keep the old white default until someone
         * measures them the same way -- guessing a shared default across MDL
         * families is exactly the error being fixed.
         *
         * The constant written here holds only while NO diffuse texture binds:
         * OmniPBR's texture REPLACES it rather than modulating it, which the
         * block after the MDL texture loop below applies (that ordering is why
         * it cannot be decided here -- the textures are not resolved yet). */
        const bool mdl_omnipbr = mdl_basename.rfind("OmniPBR", 0) == 0;
        float base[3] = { 1.0f, 1.0f, 1.0f };
        if (mdl_omnipbr)
            base[0] = base[1] = base[2] = 0.2f;
        const bool base_set = readColor3(stage, ordinal, shader, "inputs:diffuseColor", base) ||
                              readColor3(stage, ordinal, shader, "inputs:diffuse_color_constant", base);
        float tint[3] = { 1.0f, 1.0f, 1.0f };
        const bool tint_set = readColor3(stage, ordinal, shader, "inputs:diffuse_tint", tint);
        if (base_set || tint_set || mdl_omnipbr)
        {
            out->base_color[0] = base[0] * tint[0];
            out->base_color[1] = base[1] * tint[1];
            out->base_color[2] = base[2] * tint[2];
            base_out[0] = out->base_color[0];
            base_out[1] = out->base_color[1];
            base_out[2] = out->base_color[2];
        }
        out->metallic = readScalar(stage, ordinal, shader, "inputs:metallic_constant", out->metallic);
        out->roughness = readScalar(stage, ordinal, shader, "inputs:reflection_roughness_constant",
                                    readScalar(stage, ordinal, shader, "inputs:roughness_constant", out->roughness));

        /* Isaac/Unreal MDL roughness REMAP (port of material.c:3312-3350,
         * mdl_apply_isaac_roughness_remap). Those exporters author no roughness
         * constant at all -- they author the two ENDS of a remap the MDL applies
         * to the sampled roughness channel, r' = min + sample * (max - min).
         * gl's fragment shader has consumed exactly that shape since
         * shaders_gles.h:716-717 (roughSample * roughness_tex_scale +
         * roughness_tex_bias), but NOTHING in this file ever wrote the two
         * fields, so defaultMaterial's 1.0 / 0.0 stood and every Isaac
         * roughness map ran through unremapped.
         *
         * One deliberate divergence from the fork: a degenerate min == max is
         * not written as scale = 0, because the shader reads scale == 0 as
         * "unset" and substitutes 1.0 -- which would turn a constant-roughness
         * remap into (sample + min). Collapse that case to the constant. */
        {
            static const char* const s_kRoughnessMin[] = {
                "inputs:RoughnessMin",
                "inputs:Roughness_Min",
                "inputs:roughnessMin",
                "inputs:roughness_min",
            };
            static const char* const s_kRoughnessMax[] = {
                "inputs:RoughnessMax",
                "inputs:Roughness_Max",
                "inputs:roughnessMax",
                "inputs:roughness_max",
            };
            float rmin = 0.0f, rmax = 0.0f;
            const bool has_min = readScalarAny(
                stage, ordinal, shader, s_kRoughnessMin, sizeof(s_kRoughnessMin) / sizeof(s_kRoughnessMin[0]), &rmin);
            const bool has_max = readScalarAny(
                stage, ordinal, shader, s_kRoughnessMax, sizeof(s_kRoughnessMax) / sizeof(s_kRoughnessMax[0]), &rmax);
            if (has_min && has_max && rmax != rmin)
            {
                out->roughness = 0.5f * (rmin + rmax);
                out->roughness_tex_bias = rmin;
                out->roughness_tex_scale = rmax - rmin;
            }
            else if (has_max)
            {
                out->roughness = rmax;
            }
            else if (has_min)
            {
                out->roughness = rmin;
            }
            /* The family's own scalar spelling wins over the remap midpoint. */
            out->roughness = readScalar(stage, ordinal, shader, "inputs:Roughness", out->roughness);
        }

        /* MDL bump / normal-map strength (port of material.c:3231-3250's
         * normal_keys = {normal_scale, bump_factor, bump_scale}). normal_scale
         * is in the UBO and the shader multiplies the decoded tangent-space
         * mapN.xy by it (shaders_gles.h:676); the MDL branch never wrote it, so
         * an authored bump_factor was silently pinned at defaultMaterial's 1.0.
         * Kept to the fork's three names exactly -- a wider alias list would be
         * invention, and a false positive here rewrites shading. */
        {
            static const char* const s_kNormalScale[] = {
                "inputs:normal_scale",
                "inputs:bump_factor",
                "inputs:bump_scale",
            };
            float normalScale = 1.0f;
            if (readScalarAny(stage, ordinal, shader, s_kNormalScale,
                              sizeof(s_kNormalScale) / sizeof(s_kNormalScale[0]), &normalScale))
                out->normal_scale = normalScale;
        }

        /* AdvancedPBR/OmniPBR gate emission with enable_emission and use direct scalar
         * color/intensity inputs in addition to their optional mask texture.  The shipped
         * MDL families spell those scalar inputs both emissive_* and emission_*.  Reset an
         * explicitly-disabled lobe so a generic PreviewSurface field cannot leak through. */
        if (!readBooleanAttribute(stage, ordinal, shader, "inputs:enable_emission", true))
        {
            out->emissive_color[0] = 0.0f;
            out->emissive_color[1] = 0.0f;
            out->emissive_color[2] = 0.0f;
            out->emissive_color[3] = 0.0f;
        }
        else
        {
            if (readColor3(stage, ordinal, shader, "inputs:emissive_color", c) ||
                readColor3(stage, ordinal, shader, "inputs:emission_color", c))
            {
                out->emissive_color[0] = c[0];
                out->emissive_color[1] = c[1];
                out->emissive_color[2] = c[2];
            }
            out->emissive_color[3] = readScalar(stage, ordinal, shader, "inputs:emissive_intensity",
                                                readScalar(stage, ordinal, shader, "inputs:emission_intensity", 1.0f));
        }

        /* DrivesimPBR and OmniUe4Base use the alpha/opacity aliases below rather than
         * UsdPreviewSurface's inputs:opacity.  Their optional cutout mode is binary, so feed
         * its cutoff into the same depth-writing alpha-cutout path used by Preview Surface.
         * DrivesimPBR_Opacity calls the switch enable_alpha; the other shipped family calls
         * it enable_opacity. */
        const bool mdl_opacity_enabled =
            readBooleanAttribute(stage, ordinal, shader, "inputs:enable_alpha",
                                 readBooleanAttribute(stage, ordinal, shader, "inputs:enable_opacity", true));
        if (!mdl_opacity_enabled)
        {
            out->opacity = 1.0f;
            out->opacity_threshold = 0.0f;
        }
        else
        {
            out->opacity =
                readScalar(stage, ordinal, shader, "inputs:alpha",
                           readScalar(stage, ordinal, shader, "inputs:alpha_constant",
                                      readScalar(stage, ordinal, shader, "inputs:opacity_constant", out->opacity)));
            const bool mdl_alpha_cutout = readBooleanAttribute(
                stage, ordinal, shader, "inputs:enable_alpha_cutout",
                readBooleanAttribute(stage, ordinal, shader, "inputs:enable_opacity_cutout", false));
            out->opacity_threshold =
                mdl_alpha_cutout ? readScalar(stage, ordinal, shader, "inputs:alpha_cutout_cutoff", 1.0f) : 0.0f;
        }

        /* Texture inputs authored straight on the Shader. Two naming families coexist, exactly
         * as in material.c: OmniPBR (diffuse_texture, normalmap_texture, ORM_texture ...) and
         * Isaac/Unreal (AlbedoTexture, MainNormalInput, MergeMapInput ...). */
        static const struct
        {
            const char* input;
            int slot;
            bool srgb;
            const char* enable_input; /* empty = active whenever an asset is authored */
        } kMdlInputs[] = {
            /* OmniPBR */
            { "inputs:diffuse_texture", TEX_DIFFUSE_COLOR, true, "inputs:enable_diffuse_texture" },
            { "inputs:normalmap_texture", TEX_NORMAL, false, "inputs:enable_normalmap_texture" },
            { "inputs:reflectionroughness_texture", TEX_ROUGHNESS, false, "" },
            { "inputs:metallic_texture", TEX_METALLIC, false, "" },
            { "inputs:ORM_texture", TEX_ROUGHNESS, false, "inputs:enable_ORM_texture" },
            { "inputs:orm_texture", TEX_ROUGHNESS, false, "inputs:enable_orm_texture" },
            { "inputs:emissive_color_texture", TEX_EMISSIVE_COLOR, true, "inputs:enable_emission" },
            { "inputs:emissive_mask_texture", TEX_EMISSIVE_COLOR, true, "inputs:enable_emission" },
            { "inputs:ao_texture", TEX_OCCLUSION, false, "" },
            { "inputs:opacity_texture", TEX_OPACITY, false, "inputs:enable_opacity_texture" },
            /* Isaac / Unreal MDL */
            { "inputs:AlbedoTexture", TEX_DIFFUSE_COLOR, true, "" },
            { "inputs:BaseColor_Texture", TEX_DIFFUSE_COLOR, true, "" },
            { "inputs:TextureSelection", TEX_DIFFUSE_COLOR, true, "" },
            { "inputs:MainNormalInput", TEX_NORMAL, false, "" },
            { "inputs:MergeMapInput", TEX_ROUGHNESS, false, "" }, /* ORM-packed */
        };
        for (const auto& mi : kMdlInputs)
        {
            if (out->tex_indices[mi.slot] >= 0)
                continue;
            /* Drivesim retains opacity texture paths even while alpha is disabled.  Its MDL
             * contract says that state is fully opaque, so never bind the map in that mode. */
            if (mi.slot == TEX_OPACITY && !mdl_opacity_enabled)
                continue;
            /* OmniPBR-family MDLs retain texture asset paths while an explicit feature switch
             * disables that lobe.  A missing switch belongs to another MDL family, so retain
             * the direct-texture behavior; an authored false switch must win. */
            if (mi.enable_input[0] != '\0' && !readBooleanAttribute(stage, ordinal, shader, mi.enable_input, true))
                continue;
            const std::string file = readAssetResolved(stage, ordinal, shader, mi.input);
            if (file.empty())
                continue;
            const int index = findOrAddTexture(file, mi.srgb, archive_cache);
            if (index < 0)
                continue;
            out->tex_indices[mi.slot] = index;

            /* An ORM map packs occlusion/roughness/metallic into r/g/b -- which is exactly the
             * channels gl's shader reads from tex_occlusion.r / tex_roughness.g /
             * tex_metallic.b. Point all three slots at it so the packed map drives all three,
             * instead of only roughness with metallic left on its constant. */
            const bool is_orm =
                std::strstr(mi.input, "ORM") || std::strstr(mi.input, "orm") || std::strstr(mi.input, "MergeMap");
            if (is_orm)
            {
                if (out->tex_indices[TEX_METALLIC] < 0)
                    out->tex_indices[TEX_METALLIC] = index;
                if (out->tex_indices[TEX_OCCLUSION] < 0)
                    out->tex_indices[TEX_OCCLUSION] = index;
            }
        }

        /* ── OmniPBR: a bound diffuse texture REPLACES diffuse_color_constant ──
         * OmniPBR.mdl:342 forwards every diffuse input straight to
         * OmniPBR_ClearCoat.mdl, whose :630-631 is literally
         *     color diffuse = tex::texture_isvalid(diffuse_texture)
         *                       ? desaturated_base : diffuse_color_constant;
         *     color tinted_diffuse = multiply_colors(diffuse, diffuse_tint, 1.0).tint;
         * -- the constant is dropped on the floor whenever the texture is valid,
         * and only diffuse_tint survives. ovgl's shader computes texel *
         * base_color, so any constant left in base_color scales the whole albedo:
         * the 0.2 default alone is a 5x darkening of every textured OmniPBR
         * surface, which is precisely the SimReady content the default was added
         * for.
         *
         * The UsdPreviewSurface loop further down already neutralises base_color
         * for a diffuse texture, and it CANNOT reach this case: its first
         * statement requires an authored "inputs:diffuseColor.connect", while an
         * MDL shader authors inputs:diffuse_texture as a plain asset value with no
         * connection anywhere -- so it `continue`s on the first line and never
         * looks at the slot. That is why the fix has to live here.
         *
         * Measured 640x640 on a 1000-nit-dome plate bound to OmniPBR.mdl with the
         * mirrored T_CubeBox_A01_Tile_Albedo, whole-frame cross-stack MAE / px>32
         * against official ovrtx 0.4:
         *   diffuse_color_constant   official mean       before        after
         *   unauthored (=0.2)        192.1,162.3,116.5   91.746/100%   2.960/0.00%
         *   (1,1,1)                  192.2,162.3,116.0    2.788/0.00%  2.788/0.00%
         *   (0.2,0.2,0.2) authored   192.3,162.3,116.1   91.655/100%   2.771/0.00%
         *   (0.5,0.5,0.5) authored   192.3,162.3,116.0   45.191/100%   2.807/0.00%
         * Official's mean is the SAME in all four rows -- direct evidence that it
         * ignores the constant once the texture is valid. So "skip only the 0.2
         * default" is not enough: it would have left the two authored rows at 91.7
         * and 45.2. After the fix ovgl's own mean is 188.7,159.6,117.1 in all four,
         * bit-identical; the residual ~2.8 is official varying run to run (its four
         * means spread 0.2) plus the 2048 texture cap, and it is the same number the
         * constant-forced-to-white control already scored before the change.
         * The untextured controls on the same plate are BYTE-identical before and
         * after (MAE 1.780 unauthored, 2.001 with the constant at white), which is
         * the 0.2 default this block deliberately leaves alone.
         *
         * Keyed on the texture actually BINDING, not on the attribute being
         * authored, because tex::texture_isvalid is false for a texture that
         * failed to load: franka_factory's ground_plane authors an UNRESOLVABLE
         * inputs:diffuse_texture and official renders it at the constant (px 146).
         * readAssetResolved / findOrAddTexture failing leaves the slot at -1,
         * which is exactly that predicate, so the block never fires there: that
         * scene still measures 7.038 and 7.013 MAE / 2.68% px>32 over two official
         * runs (its post-port number is 7.007, and official's own run-to-run MAE on
         * it is 0.232 -- the ovgl side is byte-identical run to run).
         *
         * Scoped to the OmniPBR family for the same reason the 0.2 default is:
         * this is what OmniPBR*.mdl's own source says, and AdvancedPBR /
         * DrivesimPBR / OmniSurface are different files nobody has measured. They
         * are left EXACTLY as they were, verified on this same plate with only the
         * MDL basename swapped (ovgl-only, three arms each): constant at white ->
         * mean 188.7,159.6,117.1; at (0.5) -> 144.2,113.2,77.6 (MAE 43.486 vs the
         * white arm, 100% px>32); unauthored -> 158.8,127.2,88.6 (MAE 30.281).
         * Byte-for-byte the same three numbers for all three families. A leak would
         * have collapsed the three arms onto one frame -- which is exactly what the
         * OmniPBR control does here (MAE 0.000 between all three).
         *
         * That third arm also records a SEPARATE, pre-existing gap this change
         * deliberately does not touch: on a NON-OmniPBR MDL family with a bound
         * diffuse texture and no authored constant, base_color is never written at
         * all -- `base_set || tint_set || mdl_omnipbr` above is false -- so
         * defaultMaterial's 0.62 fallback gray tints the sampled albedo (158.8
         * where white gives 188.7). It predates the MDL port (480a93f already read
         * `if (base_set || tint_set)`), and the fork handles it with a different
         * post-pass at material.c:4766 keyed on "is base_color still the default
         * gray". Closing it means changing those families, which is out of scope
         * for this fix and wants its own cross-stack measurement. */
        if (mdl_omnipbr && out->tex_indices[TEX_DIFFUSE_COLOR] >= 0)
        {
            out->base_color[0] = tint[0];
            out->base_color[1] = tint[1];
            out->base_color[2] = tint[2];
            base_out[0] = tint[0];
            base_out[1] = tint[1];
            base_out[2] = tint[2];
        }

        /* ── OmniGlass, NVIDIA's MDL dielectric (port of material.c:4189-4212) ──
         * Everything above reads diffuse/metallic/roughness and knows nothing
         * about glass, so an OmniGlass shader left transmission_weight at 0 and
         * came out as the opaque gray default -- while ovgl's refraction +
         * transmission block (shaders_gles.h:912-935) sits fully implemented and
         * fed by the MaterialX branch alone. Map the OmniGlass inputs onto the
         * transmission params so that already-paid-for path activates.
         *
         * Applied LAST rather than as the fork's early `return`: the fork bails
         * out of a void function, whereas this one still has the UsdPreviewSurface
         * texture loop below to run. Ordering it last gives the same precedence.
         *
         * NOT ported, deliberately: the fork also sets p->solid_glass (and its
         * glass_class classifier consumes thin_walled). Those are fork-only
         * MaterialParams fields. ovgl's GpuMaterialParams has _pad_d/_pad_e in
         * exactly those bytes and its three channel ints occupy the fork's
         * _pad_a/_pad_b/_pad_c -- the two structs LOOK interchangeable and are
         * not. ovgl's shader gates on transmission_weight > 0 alone, so nothing
         * else is needed. */
        const std::string mdl_sub = readStringAttribute(stage, ordinal, shader, "info:mdl:sourceAsset:subIdentifier");
        if (mdl_basename.rfind("OmniGlass", 0) == 0 || mdl_sub.find("OmniGlass") != std::string::npos)
        {
            if (readColor3(stage, ordinal, shader, "inputs:glass_color", c))
            {
                out->transmission_color[0] = c[0];
                out->transmission_color[1] = c[1];
                out->transmission_color[2] = c[2];
            }
            else
            {
                /* OmniGlass defaults glass_color to white. The generic PBR
                 * default for the tint is BLACK, which would render the
                 * transmitted lobe as total absorption. */
                out->transmission_color[0] = 1.0f;
                out->transmission_color[1] = 1.0f;
                out->transmission_color[2] = 1.0f;
            }
            const float glass_ior =
                readScalar(stage, ordinal, shader, "inputs:glass_ior", std::numeric_limits<float>::quiet_NaN());
            if (std::isfinite(glass_ior) && glass_ior > 0.0f)
            {
                out->ior = glass_ior;
                out->transmission_ior = glass_ior;
            }
            else
            {
                /* transmission_ior == 0 makes the shader fall back to mat.ior;
                 * set it explicitly so the two can never disagree. */
                out->transmission_ior = out->ior;
            }
            static const char* const s_kFrosting[] = { "inputs:frosting_roughness" };
            float frosting = 0.0f;
            if (readScalarAny(stage, ordinal, shader, s_kFrosting, 1, &frosting))
                out->roughness = frosting;
            out->transmission_weight = 1.0f; /* OmniGlass is transmissive */
        }
    }

    /* Textures. A UsdPreviewSurface input is EITHER an authored value (read above) or a
     * connection to a UsdUVTexture, which is how every real asset ships its material: the
     * Apple samples connect diffuseColor/normal/roughness/metallic/occlusion and author no
     * constants at all, which is exactly why they came up untextured. Follow the connection
     * to the texture prim and pull its `inputs:file`.
     *
     * ovpopulation stores a connection in a separate "<name>.connect" column (a connected
     * input has no value, so only that column lands) and its target is a PROPERTY path
     * (".../BaseColor.rgb"), so trim at the dot to get the prim. */
    for (const TextureSlotDescription& desc : g_kTextureSlots)
    {
        std::string target = readStringAttribute(stage, ordinal, shader, std::string(desc.input) + ".connect");
        if (target.empty() || target[0] != '/')
            continue;
        int selected_channel = textureOutputChannel(target, -1);
        target = textureImageSourcePath(stage, ordinal, std::move(target), desc.slot);
        if (target.empty())
            continue;

        const std::string file = readAssetResolved(stage, ordinal, target, "inputs:file");
        if (file.empty())
            continue;
        /* A MaterialX float image publishes outputs:out rather than outputs:r, but its generated
         * implementation samples the red component. Keep the legacy G/B defaults only for
         * packed maps whose output did not identify a scalar MaterialX image. */
        if (selected_channel < 0 && readStringAttribute(stage, ordinal, target, "info:id") == "ND_image_float")
            selected_channel = 0;

        /* sourceColorSpace overrides the slot default when the asset says so ("raw" on a
         * color map, "sRGB" on a data map). */
        bool srgb = desc.srgb;
        const std::string cs = readStringAttribute(stage, ordinal, target, "inputs:sourceColorSpace");
        if (cs == "raw")
            srgb = false;
        else if (cs == "sRGB")
            srgb = true;

        const int index = findOrAddTexture(file, srgb, archive_cache);
        if (index >= 0)
        {
            out->tex_indices[desc.slot] = index;
            if (selected_channel >= 0)
            {
                if (desc.slot == TEX_OPACITY)
                    out->opacity_texture_channel = selected_channel;
                else if (desc.slot == TEX_ROUGHNESS)
                    out->roughness_texture_channel = selected_channel;
                else if (desc.slot == TEX_METALLIC)
                    out->metallic_texture_channel = selected_channel;
            }
            if (desc.slot == TEX_EMISSIVE_COLOR)
            {
                /* A connected PreviewSurface emissiveColor has no authored constant for the
                 * scalar read above. Enable the shader's internal multiplier so its texture
                 * contributes instead of being multiplied by the zero-initialized default. */
                out->emissive_color[3] = 1.0f;
            }

            /* The shader TINTS the sampled texel: baseColor = texture(tex_diffuse, tc) *
             * base_color. base_color still holds the fallback gray here, because a connected
             * input authors no value for readColor3 to find -- so a fully textured asset came
             * out uniformly gray-tinted. The tint is the UsdUVTexture's inputs:scale (white
             * when unauthored), exactly as the nanousd loader does it. */
            if (desc.slot == TEX_DIFFUSE_COLOR)
            {
                float scale[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
                if (!readColor4(stage, ordinal, target, "inputs:scale", scale))
                {
                    scale[0] = scale[1] = scale[2] = 1.0f;
                }
                out->base_color[0] = scale[0];
                out->base_color[1] = scale[1];
                out->base_color[2] = scale[2];
                base_out[0] = scale[0];
                base_out[1] = scale[1];
                base_out[2] = scale[2];
            }
        }
    }
    return true;
}

/* Append a triangle's 3 positions to P and 3 normals to N (parallel soups).
 * pushFlat: the face normal (crisp box/mesh edges). pushSmooth: per-vertex
 * normalize(pos) for a centered sphere (round shading). */
inline void faceNormal(const float a[3], const float b[3], const float c[3], float o[3])
{
    float u[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    float v[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    o[0] = u[1] * v[2] - u[2] * v[1];
    o[1] = u[2] * v[0] - u[0] * v[2];
    o[2] = u[0] * v[1] - u[1] * v[0];
    float n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
    if (n < 1e-12f)
        n = 1.0f;
    o[0] /= n;
    o[1] /= n;
    o[2] /= n;
}
inline void pushFlat(std::vector<float>& P, std::vector<float>& N, const float a[3], const float b[3], const float c[3])
{
    P.insert(P.end(), a, a + 3);
    P.insert(P.end(), b, b + 3);
    P.insert(P.end(), c, c + 3);
    float fn[3];
    faceNormal(a, b, c, fn);
    for (int k = 0; k < 3; k++)
        N.insert(N.end(), fn, fn + 3);
}
inline void pushSmooth(std::vector<float>& positions,
                       std::vector<float>& normals,
                       const float firstPoint[3],
                       const float secondPoint[3],
                       const float thirdPoint[3])
{
    positions.insert(positions.end(), firstPoint, firstPoint + 3);
    positions.insert(positions.end(), secondPoint, secondPoint + 3);
    positions.insert(positions.end(), thirdPoint, thirdPoint + 3);
    for (const float* point : { firstPoint, secondPoint, thirdPoint })
    {
        float length = std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
        if (length < 1e-12f)
            length = 1.0f;
        normals.push_back(point[0] / length);
        normals.push_back(point[1] / length);
        normals.push_back(point[2] / length);
    }
}

void synthCube(float h, std::vector<float>& P, std::vector<float>& N)
{
    const float v[8][3] = { { -h, -h, -h }, { h, -h, -h }, { h, h, -h }, { -h, h, -h },
                            { -h, -h, h },  { h, -h, h },  { h, h, h },  { -h, h, h } };
    static const int s_kFaces[12][3] = { { 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 }, { 0, 5, 4 },
                                         { 3, 7, 6 }, { 3, 6, 2 }, { 0, 4, 7 }, { 0, 7, 3 }, { 1, 2, 6 }, { 1, 6, 5 } };
    for (auto& f : s_kFaces)
        pushFlat(P, N, v[f[0]], v[f[1]], v[f[2]]);
}

/* Recursively subdivide a sphere triangle `depth` levels, projecting midpoints onto
 * the radius-r sphere; pushSmooth emits normalize(pos) normals (round shading). */
void sphereSubdiv(std::vector<float>& P,
                  std::vector<float>& N,
                  const float a[3],
                  const float b[3],
                  const float c[3],
                  float radius,
                  int depth)
{
    if (depth <= 0)
    {
        pushSmooth(P, N, a, b, c);
        return;
    }
    auto mid = [&](const float* p, const float* q, float* o)
    {
        o[0] = (p[0] + q[0]) * 0.5f;
        o[1] = (p[1] + q[1]) * 0.5f;
        o[2] = (p[2] + q[2]) * 0.5f;
        float n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
        if (n < 1e-12f)
            n = 1.0f;
        o[0] = o[0] / n * radius;
        o[1] = o[1] / n * radius;
        o[2] = o[2] / n * radius;
    };
    float ab[3], bc[3], ca[3];
    mid(a, b, ab);
    mid(b, c, bc);
    mid(c, a, ca);
    sphereSubdiv(P, N, a, ab, ca, radius, depth - 1);
    sphereSubdiv(P, N, ab, b, bc, radius, depth - 1);
    sphereSubdiv(P, N, ca, bc, c, radius, depth - 1);
    sphereSubdiv(P, N, ab, bc, ca, radius, depth - 1);
}

void synthSphere(float radius, std::vector<float>& P, std::vector<float>& N)
{
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    float v[12][3] = { { -1, t, 0 },  { 1, t, 0 },  { -1, -t, 0 }, { 1, -t, 0 }, { 0, -1, t },  { 0, 1, t },
                       { 0, -1, -t }, { 0, 1, -t }, { t, 0, -1 },  { t, 0, 1 },  { -t, 0, -1 }, { -t, 0, 1 } };
    auto norm = [&](float* a)
    {
        float n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (n < 1e-12f)
            n = 1.0f;
        a[0] = a[0] / n * radius;
        a[1] = a[1] / n * radius;
        a[2] = a[2] / n * radius;
    };
    for (auto& a : v)
        norm(a);
    static const int s_kFaces[20][3] = { { 0, 11, 5 }, { 0, 5, 1 },  { 0, 1, 7 },   { 0, 7, 10 }, { 0, 10, 11 },
                                         { 1, 5, 9 },  { 5, 11, 4 }, { 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 },
                                         { 3, 9, 4 },  { 3, 4, 2 },  { 3, 2, 6 },   { 3, 6, 8 },  { 3, 8, 9 },
                                         { 4, 9, 5 },  { 2, 4, 11 }, { 6, 2, 10 },  { 8, 6, 7 },  { 9, 8, 1 } };
    /* 2 subdivision levels -> 20*4^2 = 320 tris: a noticeably rounder icosphere. */
    for (auto& f : s_kFaces)
        sphereSubdiv(P, N, v[f[0]], v[f[1]], v[f[2]], radius, 2);
}

/* USD's quadric gprims (Cylinder/Cone/Capsule) carry an `axis` token: the local axis the
 * shape is swept around. Map it to the component index the synth builders extrude along. */
static int axisIndex(const std::string& axis)
{
    if (axis == "X")
        return 0;
    if (axis == "Y")
        return 1;
    return 2; /* USD default */
}

/* Place (radial_u, radial_v, along_axis) into world component order for `ax`. */
static void axial(int ax, float u, float v, float along, float o[3])
{
    const int a = ax, b = (ax + 1) % 3, c = (ax + 2) % 3;
    o[a] = along;
    o[b] = u;
    o[c] = v;
}

constexpr int g_kQuadricSegments = 32;

/* Cylinder: side wall + two caps, swept around `axis`. Smooth-ish shading comes for free
 * from pushFlat's per-triangle normals, which is what Cube already uses. */
void synthCylinder(float radius, float height, int ax, std::vector<float>& P, std::vector<float>& N)
{
    const float h = height * 0.5f;
    for (int i = 0; i < g_kQuadricSegments; i++)
    {
        const float t0 = (float)(2.0 * M_PI * i / g_kQuadricSegments);
        const float t1 = (float)(2.0 * M_PI * (i + 1) / g_kQuadricSegments);
        const float c0 = std::cos(t0) * radius, s0 = std::sin(t0) * radius;
        const float c1 = std::cos(t1) * radius, s1 = std::sin(t1) * radius;
        float b0[3], b1[3], u0[3], u1[3], cb[3], ct[3];
        axial(ax, c0, s0, -h, b0);
        axial(ax, c1, s1, -h, b1);
        axial(ax, c0, s0, h, u0);
        axial(ax, c1, s1, h, u1);
        axial(ax, 0.0f, 0.0f, -h, cb);
        axial(ax, 0.0f, 0.0f, h, ct);
        pushFlat(P, N, b0, b1, u1); /* wall */
        pushFlat(P, N, b0, u1, u0);
        pushFlat(P, N, cb, b1, b0); /* bottom cap */
        pushFlat(P, N, ct, u0, u1); /* top cap */
    }
}

/* Cone: side wall to the apex + base cap. */
void synthCone(float radius, float height, int ax, std::vector<float>& P, std::vector<float>& N)
{
    const float h = height * 0.5f;
    float apex[3], cb[3];
    axial(ax, 0.0f, 0.0f, h, apex);
    axial(ax, 0.0f, 0.0f, -h, cb);
    for (int i = 0; i < g_kQuadricSegments; i++)
    {
        const float t0 = (float)(2.0 * M_PI * i / g_kQuadricSegments);
        const float t1 = (float)(2.0 * M_PI * (i + 1) / g_kQuadricSegments);
        float b0[3], b1[3];
        axial(ax, std::cos(t0) * radius, std::sin(t0) * radius, -h, b0);
        axial(ax, std::cos(t1) * radius, std::sin(t1) * radius, -h, b1);
        pushFlat(P, N, b0, b1, apex);
        pushFlat(P, N, cb, b1, b0);
    }
}

/* Capsule: a cylinder of `height` with a hemisphere of `radius` on each end (USD's
 * definition: height is the CYLINDER length, the caps add radius beyond it). */
void synthCapsule(float radius, float height, int ax, std::vector<float>& P, std::vector<float>& N)
{
    synthCylinder(radius, height, ax, P, N);
    const float h = height * 0.5f;
    const int rings = 8;
    for (int cap = 0; cap < 2; cap++)
    {
        const float dir = cap == 0 ? 1.0f : -1.0f; /* +axis cap, then -axis cap */
        const float base = dir * h;
        for (int r = 0; r < rings; r++)
        {
            const float p0 = (float)(M_PI * 0.5 * r / rings);
            const float p1 = (float)(M_PI * 0.5 * (r + 1) / rings);
            const float r0 = std::cos(p0) * radius, a0 = std::sin(p0) * radius * dir;
            const float r1 = std::cos(p1) * radius, a1 = std::sin(p1) * radius * dir;
            for (int i = 0; i < g_kQuadricSegments; i++)
            {
                const float t0 = (float)(2.0 * M_PI * i / g_kQuadricSegments);
                const float t1 = (float)(2.0 * M_PI * (i + 1) / g_kQuadricSegments);
                float v00[3], v01[3], v10[3], v11[3];
                axial(ax, std::cos(t0) * r0, std::sin(t0) * r0, base + a0, v00);
                axial(ax, std::cos(t1) * r0, std::sin(t1) * r0, base + a0, v01);
                axial(ax, std::cos(t0) * r1, std::sin(t0) * r1, base + a1, v10);
                axial(ax, std::cos(t1) * r1, std::sin(t1) * r1, base + a1, v11);
                if (cap == 0)
                {
                    pushFlat(P, N, v00, v01, v11);
                    pushFlat(P, N, v00, v11, v10);
                }
                else
                { /* keep the winding consistent on the mirrored cap */
                    pushFlat(P, N, v00, v11, v01);
                    pushFlat(P, N, v00, v10, v11);
                }
            }
        }
    }
}

/* Plane: a flat width x length quad whose NORMAL is `axis` (so the quad spans the other two
 * components). Unlike the quadrics, UsdGeomPlane pins WHICH spanned component each dimension
 * covers (schema docs + pxr's registered ComputeExtent plugin, the ground truth here):
 *   axis=Z: width -> X, length -> Y
 *   axis=X: width -> Z, length -> Y
 *   axis=Y: width -> X, length -> Z
 * i.e. width spans X unless X is the normal (then Z), length spans Y unless Y is the normal
 * (then Z). The quadrics' cyclic `axial` order transposed width/length for the non-Z axes.
 * Emitted double-sided — two windings — because a single-sided ground plane
 * disappears the moment the camera orbits under it, which reads as a renderer bug. */
void synthPlane(float width, float length, int ax, std::vector<float>& P, std::vector<float>& N)
{
    const float w = width * 0.5f, l = length * 0.5f;
    const int wc = ax == 0 ? 2 : 0; /* component width spans */
    const int lc = ax == 1 ? 2 : 1; /* component length spans */
    float v00[3] = { 0.0f, 0.0f, 0.0f }, v10[3] = { 0.0f, 0.0f, 0.0f };
    float v11[3] = { 0.0f, 0.0f, 0.0f }, v01[3] = { 0.0f, 0.0f, 0.0f };
    v00[wc] = -w;
    v00[lc] = -l;
    v10[wc] = w;
    v10[lc] = -l;
    v11[wc] = w;
    v11[lc] = l;
    v01[wc] = -w;
    v01[lc] = l;
    pushFlat(P, N, v00, v10, v11);
    pushFlat(P, N, v00, v11, v01);
    pushFlat(P, N, v00, v11, v10);
    pushFlat(P, N, v00, v01, v11);
}

/* Turn a flat triangle soup + per-prim color + xform into a heap-owned SceneMesh. */
SceneMesh makeMesh(const std::vector<float>& P,
                   const std::vector<float>& Nrm,
                   const std::vector<float>& UV,
                   const float color[3],
                   int has_color,
                   const double world16[16],
                   const std::string& path,
                   int double_sided,
                   int front_face_cw)
{
    SceneMesh m;
    std::memset(&m, 0, sizeof(m));
    int nverts = (int)(P.size() / 3);
    m.nvertices = nverts;
    m.nindices = nverts;
    if (nverts > 0)
    {
        m.positions = (float*)std::malloc(P.size() * sizeof(float));
        std::memcpy(m.positions, P.data(), P.size() * sizeof(float));
        m.indices = (uint32_t*)std::malloc((size_t)nverts * sizeof(uint32_t));
        for (int i = 0; i < nverts; i++)
            m.indices[i] = (uint32_t)i;
    }
    if (nverts > 0 && Nrm.size() == P.size())
    {
        m.normals = (float*)std::malloc(Nrm.size() * sizeof(float));
        std::memcpy(m.normals, Nrm.data(), Nrm.size() * sizeof(float));
    }
    else
    {
        m.normals = nullptr;
    }
    m.colors = nullptr;
    /* One UV per emitted vertex, or nothing: gl reads texcoords in lockstep with
     * positions, so a partial UV array would sample garbage. */
    if (nverts > 0 && UV.size() == (size_t)nverts * 2)
    {
        m.texcoords = (float*)std::malloc(UV.size() * sizeof(float));
        std::memcpy(m.texcoords, UV.data(), UV.size() * sizeof(float));
    }
    else
    {
        m.texcoords = nullptr;
    }
    m.ptex_color_offset = 0xFFFFFFFFu;
    std::memcpy(m.world_xform, world16, sizeof(m.world_xform));
    m.display_color[0] = color[0];
    m.display_color[1] = color[1];
    m.display_color[2] = color[2];
    m.has_display_color = has_color;
    m.material_index = -1;
    m.prototype_idx = -1; /* set to own index by caller */
    m.visible = 1;
    m.double_sided = double_sided;
    m.front_face_cw = front_face_cw;
    m.path = (char*)std::malloc(path.size() + 1);
    std::memcpy(m.path, path.c_str(), path.size() + 1);
    /* Preserve both object- and world-space bounds. The transform-only path
     * reuses the former when a new worldMatrix arrives; storing the initial
     * world bounds in both fields would apply a non-identity transform twice. */
    float local_lo[3] = { 1e30f, 1e30f, 1e30f };
    float local_hi[3] = { -1e30f, -1e30f, -1e30f };
    float lo[3] = { 1e30f, 1e30f, 1e30f };
    float hi[3] = { -1e30f, -1e30f, -1e30f };
    const double* W = world16;
    for (int v = 0; v < nverts; v++)
    {
        const float* p = &P[(size_t)v * 3];
        for (int c = 0; c < 3; ++c)
        {
            if (p[c] < local_lo[c])
                local_lo[c] = p[c];
            if (p[c] > local_hi[c])
                local_hi[c] = p[c];
        }
        /* row-major world * point (USD row-vector convention: p' = p * W) */
        float wp[3];
        for (int c = 0; c < 3; c++)
            wp[c] = (float)(p[0] * W[0 * 4 + c] + p[1] * W[1 * 4 + c] + p[2] * W[2 * 4 + c] + W[3 * 4 + c]);
        for (int c = 0; c < 3; c++)
        {
            if (wp[c] < lo[c])
                lo[c] = wp[c];
            if (wp[c] > hi[c])
                hi[c] = wp[c];
        }
    }
    std::memcpy(m.bounds_min, lo, sizeof(lo));
    std::memcpy(m.bounds_max, hi, sizeof(hi));
    std::memcpy(m.local_bounds_min, local_lo, sizeof(local_lo));
    std::memcpy(m.local_bounds_max, local_hi, sizeof(local_hi));
    return m;
}

bool collectSchema(ovstage_instance_t* stage,
                   ovstage_ordinal_t ordinal,
                   const char* schema,
                   std::vector<std::pair<std::string, std::string>>& out,
                   std::unordered_set<std::string>& seen)
{
    // Do not route this through toOvx(const std::string&): converting the
    // const char* would create a temporary std::string and leave schema_val.ptr
    // dangling before ovstage_query consumes the predicate.
    ovx_string_t schema_val{};
    schema_val.ptr = schema;
    schema_val.length = std::strlen(schema);
    ovstage_predicate_t pred{};
    pred.attribute.string.ptr = "usd-prim-type";
    pred.attribute.string.length = std::strlen("usd-prim-type");
    pred.op = OVSTAGE_FILTER_OP_IN;
    pred.values = &schema_val;
    pred.value_count = 1;
    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;
    ovstage_query_handle_t qh = OVSTAGE_INVALID_QUERY_HANDLE;
    ScopedQueryHandle query_ref{ stage, qh };
    auto qe = ovstage_query(stage, &filter, nullptr, 0, &qh);
    query_ref.handle = qh;
    if (qe.status != OVSTAGE_OK || qh == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        g_err = std::string("query(") + schema + ")";
        return false;
    }
    ovstage_query_result_t qr{};
    auto fr = ovstage_fetch_query_result(stage, qh, OVSTAGE_TIMEOUT_INFINITE, &qr);
    if (fr != OVSTAGE_OK)
    {
        (void)finishEnqueue(stage, qe);
        g_err = "fetch_query_result";
        return false;
    }
    auto paths = isaacsim::ovgl_viewport::debug::details::resolveQueryPrimPaths(
        stage, qh, qr.attributes, qr.attribute_count, ordinal);
    if (profileLoad() && std::strcmp(schema, "Mesh") == 0)
        std::fprintf(stderr, "[load] Mesh query matched %zu prims\n", paths ? paths->size() : 0);
    if (paths)
    {
        for (auto& p : *paths)
            if (seen.insert(p).second)
                out.emplace_back(p, schema);
    }
    const bool result_released = ovstage_release_query_result(stage, &qr) == OVSTAGE_OK;
    const bool query_finished = finishEnqueue(stage, qe);
    if (!paths || !result_released || !query_finished)
    {
        g_err = std::string("query lifecycle(") + schema + ")";
        return false;
    }
    return true;
}

/* ── Scene-graph instancing: prototype-subtree exclusion ─────────────────
 *
 * Population authors each USD prototype subtree under an opaque
 * `/__Prototype_<hash>` root (PrototypeRootAPI in usd-schemas) so the
 * official ovstage_instancing_* services can evaluate real scenes. For
 * attached-stage rendering, OvglBackend materializes the one-level flattened
 * instance-proxy view consumed here, filling any proxy rows the population
 * implementation did not provide. Other callers must provide that same view.
 * Drawing the prototype rows as well duplicates each prototype once at its
 * prototype-local placement — a ghost that never follows ancestor transform
 * edits (the 2026-07-18 "two robots" finding). Official ovrtx and stock Kit
 * never draw prototype subtrees directly; neither may ovgl. */
std::vector<std::string> prototypeRootPaths(ovstage_instance_t* stage)
{
    std::vector<std::string> roots;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovstage_instancing_get_prototype_roots(stage, &list) != OVSTAGE_OK || list == OVX_INVALID_PRIMPATH_LIST)
        return roots;
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (dict)
    {
        roots = isaacsim::ovgl_viewport::debug::details::resolvePrimPaths(dict, list);
        path_dictionary_release_path_list_reference(dict, list);
    }
    return roots;
}

bool underPrototypeRoot(const std::string& path, const std::vector<std::string>& roots)
{
    for (const std::string& root : roots)
    {
        if (root.empty() || root == "/")
            continue;
        if (path.size() >= root.size() && path.compare(0, root.size(), root) == 0 &&
            (path.size() == root.size() || path[root.size()] == '/'))
            return true;
    }
    return false;
}

void dropPrototypePrims(std::vector<std::pair<std::string, std::string>>& prims,
                        const std::vector<std::string>& prototypeRoots)
{
    if (prototypeRoots.empty())
        return;
    prims.erase(std::remove_if(prims.begin(), prims.end(),
                               [&](const std::pair<std::string, std::string>& ps)
                               { return underPrototypeRoot(ps.first, prototypeRoots); }),
                prims.end());
}

/* Read the first authored UsdLuxDomeLight into the Scene's dome_* fields.
 *
 * A DomeLight is an infinitely distant environment, not a positioned emitter,
 * so it does NOT belong in g_lights with the punctual/area lights: the
 * renderer turns it into the IBL environment instead (Ovgl.cpp::
 * applyEnvironment). Hydra takes the first one and ignores the rest; so do
 * we, in the order ovstage hands them back.
 *
 * `intensity` and `exposure` collapse into one number here for the same
 * reason they do for the punctual lights above -- UsdLux defines emitted
 * radiance as color * intensity * 2^exposure -- and the caller only ever
 * needs the product.
 *
 * Returns false only when the DomeLight query itself failed (g_err set), which
 * the caller treats exactly like a failed light query: "no authored dome".
 * A scene with no DomeLight leaves has_dome 0, and that is a MEANINGFUL state,
 * not just an absence -- Ovgl.cpp keeps its synthetic MuJoCo headlight for it. */
bool readSceneDome(ovstage_instance_t* stage,
                   ovstage_ordinal_t ordinal,
                   const std::vector<std::string>& prototypeRoots,
                   Scene* out)
{
    std::vector<std::pair<std::string, std::string>> dp;
    std::unordered_set<std::string> seen;
    if (!collectSchema(stage, ordinal, "DomeLight", dp, seen))
        return false;
    dropPrototypePrims(dp, prototypeRoots);
    for (auto& ps : dp)
    {
        const std::string& path = ps.first;
        /* An invisible dome emits nothing and must NOT claim the
         * first-dome-wins slot: `continue`, not `break`, so a later visible
         * dome can still install the IBL. (Same rule as the fork's
         * scene.c DomeLight pass.) */
        if (!includedPurpose(stage, ordinal, path) || !includedVisibility(stage, ordinal, path))
            continue;
        out->has_dome = 1;
        out->dome_intensity = readScalar(stage, ordinal, path, "inputs:intensity", 1.0f) *
                              std::exp2(readScalar(stage, ordinal, path, "inputs:exposure", 0.0f));
        float col[3] = { 1.0f, 1.0f, 1.0f };
        readColor3(stage, ordinal, path, "inputs:color", col);
        out->dome_color[0] = col[0];
        out->dome_color[1] = col[1];
        out->dome_color[2] = col[2];
        /* An authored texture makes this a real IBL; the resolved path feeds
         * the existing HDR loader. Textureless is the far more common
         * authoring (both robot_ground_scene and the Isaac locomotion rigs),
         * and it means a uniform-radiance environment of color*intensity --
         * which is what makes official's sky read (222,223,225) rather than
         * black. */
        const std::string tex = readAssetResolved(stage, ordinal, path, "inputs:texture:file");
        std::snprintf(out->dome_hdr_path, sizeof(out->dome_hdr_path), "%s", tex.c_str());
        /* Layer-relative asset resolution is ovstage's job, not ours, and
         * readAssetResolved already prefers the RESOLVED column ovpopulation
         * writes beside the authored string. The fork resolves relative paths
         * itself against `filepath` only because scene_load owns the layer; a
         * renderer driven by a sealed ovstage snapshot has no layer path to
         * resolve against, so inventing a base directory here would be a
         * guess. If the resolved column is absent the authored string is
         * passed through unchanged and the HDR load fails loudly. */
        /* inputs:rotation is the dome's Y (up-axis) spin in degrees. Read here
         * so the field stops being a lie; see Ovgl.cpp for where it reaches
         * the equirect lookup. */
        out->dome_rotation_y = readScalar(stage, ordinal, path, "inputs:rotation", 0.0f);
        return true;
    }
    return true;
}

/* Collect authored USD lights (Distant/Rect/Sphere) into g_lights as gl
 * GpuLight (kind 0=Rect, 1=Distant, 2=Sphere). DomeLight is read separately by
 * readSceneDome, which drives the IBL environment.
 * worldMatrix is row-major: translation = row 3; local +X/+Y/+Z = rotation rows
 * 0/1/2. A light shines along its local -Z, so normal = -(local +Z in world).
 * Returns false when light discovery failed (g_err set); g_lights is then
 * empty. The build has always rendered such a failure as "no authored lights";
 * the transform-only refresh instead fails closed on it (full rebuild). */
/* Count active, visible UsdLux lights whose schema this renderer does NOT
 * evaluate. Not a diagnostic: Ovgl.cpp uses it to decide whether an authored
 * DomeLight may replace the synthetic fill rig. A scene whose key light is a
 * kind ovgl cannot see is a scene ovgl cannot light on its own, however bright
 * the dome is -- measured on the mirrored franka_factory_pickplace, whose
 * DiskLight is intensity 100 at exposure 5 (= 3200) with a 46.8-degree shaping
 * cone: official's frame luminance is 177.5 with it and 112.3 with it removed,
 * while ovgl renders 123.4 EITHER WAY. Drop this count and a 226-nit dome
 * evicts the only thing standing in for that key. */
int countUnsupportedLights(ovstage_instance_t* stage,
                           ovstage_ordinal_t ordinal,
                           const std::vector<std::string>& prototypeRoots)
{
    std::vector<std::pair<std::string, std::string>> unsupportedLights;
    std::unordered_set<std::string> seen;
    /* The two concrete UsdLux kinds with no GpuLight::kind here (gl/gpu.h
     * documents 0=Rect 1=Distant 2=Sphere). Porting kinds 3/4 is a live item;
     * when it lands, delete the corresponding collectSchema call. */
    if (!collectSchema(stage, ordinal, "DiskLight", unsupportedLights, seen))
        return 0;
    if (!collectSchema(stage, ordinal, "CylinderLight", unsupportedLights, seen))
        return 0;
    dropPrototypePrims(unsupportedLights, prototypeRoots);
    int count = 0;
    for (auto& pathAndType : unsupportedLights)
    {
        if (!includedPurpose(stage, ordinal, pathAndType.first) || !includedVisibility(stage, ordinal, pathAndType.first))
            continue;
        ++count;
    }
    return count;
}

bool readSceneLights(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const std::vector<std::string>& prototypeRoots)
{
    g_lights.clear();
    std::vector<std::pair<std::string, std::string>> lp;
    std::unordered_set<std::string> seen;
    bool collected = true;
    collected &= collectSchema(stage, ordinal, "DistantLight", lp, seen);
    collected &= collectSchema(stage, ordinal, "RectLight", lp, seen);
    collected &= collectSchema(stage, ordinal, "SphereLight", lp, seen);
    if (!collected)
    {
        g_lights.clear();
        return false;
    }
    dropPrototypePrims(lp, prototypeRoots);
    for (auto& ps : lp)
    {
        const std::string& path = ps.first;
        const std::string& schema = ps.second;
        if (!includedPurpose(stage, ordinal, path) || !includedVisibility(stage, ordinal, path))
            continue;
        double w[16];
        readWorldMatrix(stage, ordinal, path, w);
        GpuLight L;
        std::memset(&L, 0, sizeof(L));
        /* UsdLux photometric units: emitted radiance is
         * color * intensity * 2^exposure. Exposure was previously ignored,
         * which silently dropped every stop of authored exposure. */
        L.intensity = readScalar(stage, ordinal, path, "inputs:intensity", 1.0f) *
                      std::exp2(readScalar(stage, ordinal, path, "inputs:exposure", 0.0f));
        float col[3] = { 1.0f, 1.0f, 1.0f };
        readColor3(stage, ordinal, path, "inputs:color", col);
        L.color[0] = col[0];
        L.color[1] = col[1];
        L.color[2] = col[2];
        /* `inputs:normalize` decides whether the shader divides L_e by the
         * emitter area, and UsdLux defaults it to FALSE. Hardcoding 1 divided
         * by the area unconditionally, so a large authored emitter went dark in
         * proportion to its own size: the Isaac warehouse's ceiling lamps are
         * 200 x 28 with normalize unauthored, giving area = 4*100*14 = 5600 and
         * turning intensity 400000 into ~71 — a 5600x underexposure that
         * rendered the whole scene near-black. */
        L.normalize = readBooleanAttribute(stage, ordinal, path, "inputs:normalize", false) ? 1 : 0;
        L.position[0] = (float)w[12];
        L.position[1] = (float)w[13];
        L.position[2] = (float)w[14];
        float zx = (float)w[8], zy = (float)w[9], zz = (float)w[10];
        float zn = std::sqrt(zx * zx + zy * zy + zz * zz);
        if (zn < 1e-6f)
            zn = 1.0f;
        L.normal[0] = -zx / zn;
        L.normal[1] = -zy / zn;
        L.normal[2] = -zz / zn;
        if (schema == "DistantLight")
        {
            L.kind = 1;
            L.angle_deg = readScalar(stage, ordinal, path, "inputs:angle", 0.53f);
        }
        else if (schema == "RectLight")
        {
            L.kind = 0;
            float wd = readScalar(stage, ordinal, path, "inputs:width", 1.0f) * 0.5f;
            float ht = readScalar(stage, ordinal, path, "inputs:height", 1.0f) * 0.5f;
            L.u_axis[0] = (float)w[0] * wd;
            L.u_axis[1] = (float)w[1] * wd;
            L.u_axis[2] = (float)w[2] * wd;
            L.v_axis[0] = (float)w[4] * ht;
            L.v_axis[1] = (float)w[5] * ht;
            L.v_axis[2] = (float)w[6] * ht;
        }
        else
        { /* SphereLight */
            L.kind = 2;
            /* The GLSL sphere branch reads the radius from ua.x, i.e.
             * u_axis[0] (gpu_apply_light_uniforms packs u_axis into
             * u_sceneLightUAxisAngle[i].xyz; .w = angle_deg is never read for
             * kind 2). Parking the radius only in angle_deg left the shader
             * seeing 1e-4: a normalize=1 SphereLight then saturated every
             * fragment it lit, so even a FULL rebuild rendered its position
             * changes as 0 changed pixels (fast-path adversary F1
             * sub-finding, probe p3c, 2026-07-19). */
            const float radius = readScalar(stage, ordinal, path, "inputs:radius", 0.5f);
            L.u_axis[0] = radius;
            L.angle_deg = radius;
        }
        g_lights.push_back(L);
        if ((int)g_lights.size() >= GPU_MAX_SCENE_LIGHTS)
            break;
    }
    return true;
}

} // namespace

extern "C" int ovgl_scene_build(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* out)
{
    if (!stage || !out)
    {
        g_err = "null arg";
        return 0;
    }
    const double t_start = profileLoad() ? loadNowMs() : 0.0;
    ovgl_scene_free(out);

    /* Textures live for the lifetime of the built scene: drop the previous set (and its
     * negative-cache entries) so a rebuild re-resolves against the current stage. */
    g_textures.clear();
    g_texture_pixels.clear();
    g_texture_index.clear();
    g_texture_reg.clear();
    g_texture_content_index.clear();
    g_texture_decodes = 0;
    g_texture_hash_ms = 0.0;
    /* Decode-memo sweep: retire pixels that neither the previous build nor
     * the one before it referenced. This keeps timeline-playback rebuilds
     * (same texture set every tick) at zero decode cost while bounding
     * resident pixels to the union of two consecutive scenes across a stage
     * switch. Runs before this build stamps its own serial. */
    ++g_texture_build_serial;
    for (auto it = g_texture_decode_memo.begin(); it != g_texture_decode_memo.end();)
    {
        if (it->second.last_used_build + 2 <= g_texture_build_serial)
            it = g_texture_decode_memo.erase(it);
        else
            ++it;
    }

    /* Every prefetched value below belongs only to this sealed ordinal. Clear before discovery
     * as well as after the build so an earlier failed/aborted consumer cannot influence it. */
    batchClear();
    g_reads = 0;

    /* Collect graph roots without reading their attributes yet. GeomSubset material bindings
     * join the geometry/ancestor batch instead of doing one point query per subset. */
    g_subset_binding.clear();
    std::vector<std::pair<std::string, std::string>> subsets;
    std::unordered_set<std::string> subset_seen;
    if (!collectSchema(stage, ordinal, "GeomSubset", subsets, subset_seen))
        return 0;

    std::vector<std::pair<std::string, std::string>> prims;
    std::unordered_set<std::string> seen;
    for (const char* s : { "Mesh", "Cube", "Sphere", "Cylinder", "Cone", "Capsule", "Plane" })
        if (!collectSchema(stage, ordinal, s, prims, seen))
            return 0;

    /* Prototype subtrees are instancing METADATA, not drawable scene rows —
     * their per-instance expansions were collected above at the instance
     * paths. See prototypeRootPaths for the ghost defect this prevents. */
    const std::vector<std::string> prototypeRoots = prototypeRootPaths(stage);
    dropPrototypePrims(prims, prototypeRoots);
    dropPrototypePrims(subsets, prototypeRoots);

    /* Phase 1: mesh data plus inherited visibility/purpose/material bindings. Correct now that
     * ovstage owns each read row; before that fix this dropped ~10% of shared geometry. */
    std::vector<std::string> geometry_paths;
    geometry_paths.reserve(prims.size() + subsets.size());
    for (const auto& ps : prims)
        geometry_paths.push_back(ps.first);
    {
        std::unordered_set<std::string> covered(geometry_paths.begin(), geometry_paths.end());
        const size_t n_geometry = geometry_paths.size();
        for (size_t p = 0; p < n_geometry; ++p)
        {
            std::string cur = geometry_paths[p];
            for (;;)
            {
                const size_t slash = cur.rfind('/');
                if (slash == std::string::npos || slash == 0)
                    break;
                cur.resize(slash);
                if (!covered.insert(cur).second)
                    break;
                geometry_paths.push_back(cur);
            }
        }
    }
    for (const auto& subset : subsets)
        geometry_paths.push_back(subset.first);
    static const char* s_kGeometryAttrs[] = {
        "points",
        "faceVertexCounts",
        "faceVertexIndices",
        "normals",
        "primvars:normals",
        "primvars:st",
        "primvars:st:indices",
        "primvars:st0",
        "primvars:st0:indices",
        "primvars:UVMap",
        "primvars:UVMap:indices",
        "primvars:uv",
        "primvars:uv:indices",
        "primvars:displayColor",
        "omni:fabric:worldMatrix",
        "material:binding",
        "skel:skeleton",
        "skel:joints",
        "primvars:skel:jointIndices",
        "primvars:skel:jointWeights",
        "primvars:skel:geomBindTransform",
        "doubleSided",
        "orientation",
        "purpose",
        "visibility",
        "size",
        "radius",
        "height",
        "axis",
        "width",
        "length",
        "usd-stage-up-axis",
    };
    batchPrefetchAttributes(
        stage, ordinal, geometry_paths, s_kGeometryAttrs, sizeof(s_kGeometryAttrs) / sizeof(s_kGeometryAttrs[0]));

    for (const auto& subset : subsets)
    {
        const std::string& sub_path = subset.first;
        const std::string bound = readStringAttribute(stage, ordinal, sub_path, "material:binding");
        if (bound.empty() || bound[0] != '/')
            continue;
        const size_t slash = sub_path.rfind('/');
        if (slash == std::string::npos || slash == 0)
            continue;
        const std::string mesh_path = sub_path.substr(0, slash);
        g_subset_binding.emplace(mesh_path, bound); /* first subset wins */
    }

    /* Phase 2: walk only cached mesh/ancestor bindings, then prefetch terminal outputs once for
     * each distinct material. These maps are local to this build and never cross ordinals. */
    std::unordered_map<std::string, std::string> material_by_prim;
    std::vector<std::string> material_paths;
    std::unordered_set<std::string> distinct_materials;
    material_by_prim.reserve(prims.size());
    for (const auto& prim : prims)
    {
        const std::string material = boundMaterialPath(stage, ordinal, prim.first);
        material_by_prim.emplace(prim.first, material);
        if (!material.empty() && distinct_materials.insert(material).second)
            material_paths.push_back(material);
    }
    batchPrefetchAttributes(stage, ordinal, material_paths, g_kMaterialOutputAttrs,
                            sizeof(g_kMaterialOutputAttrs) / sizeof(g_kMaterialOutputAttrs[0]));

    /* A connection on an active Material can still point at an inactive shader. Gather every
     * authored context target, then batch its reserved existence metadata before selecting a
     * terminal. This preserves active-only population semantics without adding point reads. */
    std::vector<std::string> surface_candidates;
    std::unordered_set<std::string> distinct_surface_candidates;
    for (const std::string& material : material_paths)
    {
        for (std::string candidate : materialShaderCandidates(stage, ordinal, material))
        {
            if (distinct_surface_candidates.insert(candidate).second)
                surface_candidates.push_back(std::move(candidate));
        }
    }
    static const char* s_kSurfaceCandidateAttrs[] = { "usd-prim-type" };
    batchPrefetchAttributes(stage, ordinal, surface_candidates, s_kSurfaceCandidateAttrs,
                            sizeof(s_kSurfaceCandidateAttrs) / sizeof(s_kSurfaceCandidateAttrs[0]));

    std::unordered_map<std::string, std::string> shader_by_material;
    std::vector<std::string> shaderPaths;
    std::unordered_set<std::string> distinct_shaders;
    shader_by_material.reserve(material_paths.size());
    for (const std::string& material : material_paths)
    {
        const std::string shader = materialShaderPath(stage, ordinal, material);
        shader_by_material.emplace(material, shader);
        if (!shader.empty() && distinct_shaders.insert(shader).second)
            shaderPaths.push_back(shader);
    }

    /* Phase 3: prefetch every scalar, switch, asset, and direct connection consumed by the
     * supported PreviewSurface/MaterialX/MDL bridge. New fields remain correctness-safe: an
     * attribute omitted here simply takes the coverage-aware point-read fallback. */
    static const char* s_kShaderAttrs[] = {
        "inputs:diffuseColor",
        "inputs:metallic",
        "inputs:roughness",
        "inputs:ior",
        "inputs:opacity",
        "inputs:opacityThreshold",
        "inputs:emissiveColor",
        "inputs:clearcoat",
        "inputs:clearcoatRoughness",
        "inputs:useSpecularWorkflow",
        "inputs:specularColor",
        "info:id",
        "inputs:emission",
        "inputs:emission_luminance",
        "inputs:base_color",
        "inputs:base",
        "inputs:base_weight",
        "inputs:metalness",
        "inputs:base_metalness",
        "inputs:specular_roughness",
        "inputs:specular_IOR",
        "inputs:specular_ior",
        "inputs:geometry_opacity",
        "inputs:coat",
        "inputs:coat_weight",
        "inputs:coat_roughness",
        "inputs:emission_color",
        "inputs:transmission_color",
        "inputs:transmission_weight",
        "inputs:transmission",
        "info:mdl:sourceAsset",
        "inputs:diffuse_color_constant",
        "inputs:diffuse_tint",
        "inputs:metallic_constant",
        "inputs:reflection_roughness_constant",
        "inputs:roughness_constant",
        "inputs:enable_emission",
        "inputs:emissive_color",
        "inputs:emissive_intensity",
        "inputs:emission_intensity",
        "inputs:enable_alpha",
        "inputs:enable_opacity",
        "inputs:alpha",
        "inputs:alpha_constant",
        "inputs:opacity_constant",
        "inputs:enable_alpha_cutout",
        "inputs:enable_opacity_cutout",
        "inputs:alpha_cutout_cutoff",
        "inputs:diffuse_texture",
        "inputs:enable_diffuse_texture",
        "inputs:normalmap_texture",
        "inputs:enable_normalmap_texture",
        "inputs:reflectionroughness_texture",
        "inputs:metallic_texture",
        "inputs:ORM_texture",
        "inputs:enable_ORM_texture",
        "inputs:orm_texture",
        "inputs:enable_orm_texture",
        "inputs:emissive_color_texture",
        "inputs:emissive_mask_texture",
        "inputs:ao_texture",
        "inputs:opacity_texture",
        "inputs:enable_opacity_texture",
        "inputs:AlbedoTexture",
        "inputs:BaseColor_Texture",
        "inputs:TextureSelection",
        "inputs:MainNormalInput",
        "inputs:MergeMapInput",
        /* Isaac MDL roughness remap + MDL bump strength + OmniGlass. */
        "inputs:RoughnessMin",
        "inputs:Roughness_Min",
        "inputs:roughnessMin",
        "inputs:roughness_min",
        "inputs:RoughnessMax",
        "inputs:Roughness_Max",
        "inputs:roughnessMax",
        "inputs:roughness_max",
        "inputs:Roughness",
        "inputs:normal_scale",
        "inputs:bump_factor",
        "inputs:bump_scale",
        "info:mdl:sourceAsset:subIdentifier",
        "inputs:glass_color",
        "inputs:glass_ior",
        "inputs:frosting_roughness",
        "inputs:diffuseColor.connect",
        "inputs:normal.connect",
        "inputs:roughness.connect",
        "inputs:metallic.connect",
        "inputs:emissiveColor.connect",
        "inputs:occlusion.connect",
        "inputs:opacity.connect",
    };
    batchPrefetchAttributes(
        stage, ordinal, shaderPaths, s_kShaderAttrs, sizeof(s_kShaderAttrs) / sizeof(s_kShaderAttrs[0]));

    /* Phase 4: prefetch direct texture nodes plus the exact active Apple normal remap consumed
     * above. This remains three fixed batched frontiers, not arbitrary MaterialX evaluation. */
    std::vector<std::string> texturePaths;
    std::vector<std::string> unknownOutputPaths;
    std::vector<std::string> normalRoots;
    std::unordered_set<std::string> distinctTexturePaths;
    std::unordered_set<std::string> distinctUnknownOutputPaths;
    std::unordered_set<std::string> distinctNormalRoots;
    for (const std::string& shader : shaderPaths)
    {
        for (const TextureSlotDescription& desc : g_kTextureSlots)
        {
            const std::string target = readStringAttribute(stage, ordinal, shader, std::string(desc.input) + ".connect");
            const std::string node = connectionPrimPath(target);
            if (node.empty())
                continue;
            if (distinctTexturePaths.insert(node).second)
                texturePaths.push_back(node);
            if (textureOutputChannel(target, -1) < 0 && distinctUnknownOutputPaths.insert(node).second)
                unknownOutputPaths.push_back(node);
            if (desc.slot == TEX_NORMAL && distinctNormalRoots.insert(node).second)
                normalRoots.push_back(node);
        }
    }
    static const char* s_kTextureImageAttrs[] = {
        "inputs:file",
        "inputs:sourceColorSpace",
        "inputs:scale",
    };
    static const char* s_kTextureNodeIdAttrs[] = { "info:id" };
    static const char* s_kTextureArithmeticAttrs[] = {
        "info:id",
        "inputs:in1.connect",
        "inputs:in2",
        "inputs:in2.connect",
    };
    batchPrefetchAttributes(stage, ordinal, texturePaths, s_kTextureImageAttrs,
                            sizeof(s_kTextureImageAttrs) / sizeof(s_kTextureImageAttrs[0]));
    /* Only outputs:out-style edges need their leaf ID to derive scalar-channel semantics. */
    batchPrefetchAttributes(stage, ordinal, unknownOutputPaths, s_kTextureNodeIdAttrs,
                            sizeof(s_kTextureNodeIdAttrs) / sizeof(s_kTextureNodeIdAttrs[0]));

    std::vector<std::string> unresolvedNormalRoots;
    for (const std::string& root : normalRoots)
    {
        if (readAssetResolved(stage, ordinal, root, "inputs:file").empty())
            unresolvedNormalRoots.push_back(root);
    }
    batchPrefetchAttributes(stage, ordinal, unresolvedNormalRoots, s_kTextureArithmeticAttrs,
                            sizeof(s_kTextureArithmeticAttrs) / sizeof(s_kTextureArithmeticAttrs[0]));

    std::vector<std::string> multiplyPaths;
    std::unordered_set<std::string> distinctMultiplyPaths;
    for (const std::string& root : unresolvedNormalRoots)
    {
        std::string multiply;
        if (exactTextureArithmeticInput(stage, ordinal, root, "ND_subtract_vector3FA", 1.0f, multiply) &&
            distinctMultiplyPaths.insert(multiply).second)
            multiplyPaths.push_back(std::move(multiply));
    }
    batchPrefetchAttributes(stage, ordinal, multiplyPaths, s_kTextureArithmeticAttrs,
                            sizeof(s_kTextureArithmeticAttrs) / sizeof(s_kTextureArithmeticAttrs[0]));

    std::vector<std::string> imagePaths;
    std::unordered_set<std::string> distinctImagePaths;
    for (const std::string& multiply : multiplyPaths)
    {
        std::string image;
        if (exactTextureArithmeticInput(stage, ordinal, multiply, "ND_multiply_vector3FA", 2.0f, image) &&
            distinctImagePaths.insert(image).second)
            imagePaths.push_back(std::move(image));
    }
    batchPrefetchAttributes(stage, ordinal, imagePaths, s_kTextureImageAttrs,
                            sizeof(s_kTextureImageAttrs) / sizeof(s_kTextureImageAttrs[0]));
    batchPrefetchAttributes(stage, ordinal, imagePaths, s_kTextureNodeIdAttrs,
                            sizeof(s_kTextureNodeIdAttrs) / sizeof(s_kTextureNodeIdAttrs[0]));

    const double prefetchedTime = profileLoad() ? loadNowMs() : 0.0;

    int authoredUpAxis = 2;
    for (const auto& prim : prims)
    {
        const std::string axis = readStringAttribute(stage, ordinal, prim.first, "usd-stage-up-axis");
        if (axis == "Y" || axis == "y")
        {
            authoredUpAxis = 1;
            break;
        }
        if (axis == "Z" || axis == "z")
        {
            authoredUpAxis = 2;
            break;
        }
    }
    if (prims.empty())
    {
        /* GEOMETRY-free scene: no drawable row carries the per-prim
         * `usd-stage-up-axis` byte, but population always authors the
         * reserved stage-info prim, whose `upAxis` is an interned token
         * ({kDLUInt,64,1}, ovpopulation populate_stage_info). Decode it
         * token-ONLY — the column is untagged, and the path-first decode
         * space is exactly the axis/orientation collision hazard documented
         * at resolveTokenId. This keeps the grid overlay (and stats
         * up-axis) honest for an empty Y-up stage; unreadable/absent rows
         * keep the Z default. Non-empty scenes are untouched. */
        const ovx_string_t info = ovstage_population_stage_info_path();
        std::vector<uint8_t> row;
        if (readAttributeHost(stage, ordinal, std::string(info.ptr, info.length), "upAxis", row) && row.size() >= 8)
        {
            uint64_t token = 0;
            std::memcpy(&token, row.data(), sizeof(token));
            const std::string axis = resolveTokenId(stage, token);
            if (axis == "Y" || axis == "y")
                authoredUpAxis = 1;
        }
    }

    std::vector<SceneMesh> meshes;
    meshes.reserve(prims.size());
    g_materials.clear();
    /* A package can contain many textures. Keep exactly one immutable archive
     * snapshot for this build instead of reopening and rereading the whole USDZ
     * for every image; decoded pixels remain in the existing per-build stores. */
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache usdz_archives;
    /* Resolved-material memo for THIS build, keyed by shader-prim path (see the call site). */
    std::unordered_map<std::string, GpuMaterialParams> material_cache;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdSkelDeformer skel_deformer(
        [&](const std::string& prim_path, const std::string& attribute, std::vector<uint8_t>& bytes)
        {
            if (!readAttributeHost(stage, ordinal, prim_path, attribute, bytes))
                return false;
            /* Official ID columns (P1.3): the skel string attributes now carry
             * u64 token/path ids; the deformer's own decoders expect the
             * legacy NUL-separated UTF-8 rows, so translate here (numeric
             * attrs are untouched — gated by name).
             *
             * Token ids and path ids are SEPARATE handle spaces in the same
             * dictionary, and both are small sequential counters, so the
             * ATTRIBUTE — not a successful resolution — must pick the space.
             * `joints` / `skel:joints` are VtArray<TfToken> columns
             * (SEMANTIC_TOKEN_ID, ovpopulation serialize_value) and decode
             * token-ONLY; `skel:skeleton` / `skel:animationSource` are
             * relationship targets and keep the path-first decode. Decoding
             * the joint tokens path-first returned unrelated prim paths,
             * collapsing the joint hierarchy (every joint parentless), which
             * drew every mesh bound to a depth>=2 joint without its ancestor
             * joints' transforms. */
            static const std::unordered_set<std::string> s_kSkelTokenAttrs = { "joints", "skel:joints" };
            static const std::unordered_set<std::string> s_kSkelPathAttrs = { "skel:skeleton", "skel:animationSource" };
            const bool is_token_attr = s_kSkelTokenAttrs.count(attribute) != 0;
            if ((is_token_attr || s_kSkelPathAttrs.count(attribute)) && !bytes.empty() && bytes.size() % 8 == 0)
            {
                std::vector<uint8_t> decoded;
                bool all = true;
                for (size_t off = 0; off < bytes.size(); off += 8)
                {
                    uint64_t v = 0;
                    std::memcpy(&v, bytes.data() + off, 8);
                    const std::string sv = is_token_attr ? resolveTokenId(stage, v) : resolveU64Id(stage, v, 0);
                    if (sv.empty())
                    {
                        all = false;
                        break;
                    }
                    decoded.insert(decoded.end(), sv.begin(), sv.end());
                    decoded.push_back('\0');
                }
                if (all)
                    bytes = std::move(decoded);
            }
            return true;
        });
    for (auto& ps : prims)
    {
        const std::string& path = ps.first;
        const std::string& schema = ps.second;
        if (!includedPurpose(stage, ordinal, path) || !includedVisibility(stage, ordinal, path))
            continue;
        std::vector<float> P, N, UV;
        if (schema == "Cube")
        {
            synthCube(0.5f * readScalar(stage, ordinal, path, "size", 2.0f), P, N);
        }
        else if (schema == "Sphere")
        {
            synthSphere(readScalar(stage, ordinal, path, "radius", 1.0f), P, N);
        }
        else if (schema == "Plane")
        {
            synthPlane(readScalar(stage, ordinal, path, "width", 2.0f), readScalar(stage, ordinal, path, "length", 2.0f),
                       axisIndex(readStringAttribute(stage, ordinal, path, "axis")), P, N);
        }
        else if (schema == "Cylinder" || schema == "Cone" || schema == "Capsule")
        {
            /* USD quadrics: radius + height, swept around `axis` (default Z). */
            const float radius = readScalar(stage, ordinal, path, "radius", 1.0f);
            const float height = readScalar(stage, ordinal, path, "height", 2.0f);
            const int ax = axisIndex(readStringAttribute(stage, ordinal, path, "axis"));
            if (schema == "Cylinder")
            {
                synthCylinder(radius, height, ax, P, N);
            }
            else if (schema == "Cone")
            {
                synthCone(radius, height, ax, P, N);
            }
            else
            {
                synthCapsule(radius, height, ax, P, N);
            }
        }
        else
        {
            std::vector<uint8_t> pb, cb, ib, nb_, ub;
            if (!readAttributeHost(stage, ordinal, path, "points", pb) || pb.empty())
                continue;
            readAttributeHost(stage, ordinal, path, "faceVertexCounts", cb);
            readAttributeHost(stage, ordinal, path, "faceVertexIndices", ib);
            if (!readAttributeHost(stage, ordinal, path, "normals", nb_) || nb_.empty())
                readAttributeHost(stage, ordinal, path, "primvars:normals", nb_);
            /* UVs: primvars:st is the UsdPreviewSurface convention (what UsdUVTexture reads
             * through a UsdPrimvarReader_float2); primvars:uv is the older spelling. */
            /* Same primvar names, in the same order, as the nanousd renderer's scene.c
             * (uv_names[] / uv_idx_names[]). */
            std::vector<uint8_t> uib;
            static const char* s_kUvNames[] = { "primvars:st", "primvars:st0", "primvars:UVMap", "primvars:uv" };
            static const char* s_kUvIndexNames[] = { "primvars:st:indices", "primvars:st0:indices",
                                                     "primvars:UVMap:indices", "primvars:uv:indices" };
            for (size_t u = 0; u < sizeof(s_kUvNames) / sizeof(s_kUvNames[0]); u++)
            {
                if (readAttributeHost(stage, ordinal, path, s_kUvNames[u], ub) && !ub.empty())
                {
                    readAttributeHost(stage, ordinal, path, s_kUvIndexNames[u], uib);
                    break;
                }
            }
            std::vector<float> points(pb.size() / sizeof(float));
            std::memcpy(points.data(), pb.data(), points.size() * sizeof(float));
            std::vector<float> normals(nb_.size() / sizeof(float));
            if (!normals.empty())
                std::memcpy(normals.data(), nb_.data(), normals.size() * sizeof(float));
            skel_deformer.deform(path, ib.empty() ? nullptr : reinterpret_cast<const int*>(ib.data()),
                                 ib.size() / sizeof(int), points, normals);
            isaacsim::ovgl_viewport::debug::details::ovgl::buildMeshSoup(
                points.data(), points.size() / 3, (const int*)cb.data(), cb.size() / sizeof(int), (const int*)ib.data(),
                ib.size() / sizeof(int), normals.empty() ? nullptr : normals.data(), normals.size() / 3,
                (const float*)ub.data(), ub.size() / (2 * sizeof(float)),
                uib.empty() ? nullptr : (const int*)uib.data(), uib.size() / sizeof(int), P, N, UV);
        }
        if (P.empty())
            continue;

        float color[3] = { 0.7f, 0.7f, 0.7f };
        int has_color = 0;
        std::vector<uint8_t> col;
        if (readAttributeHost(stage, ordinal, path, "primvars:displayColor", col) && col.size() >= 3 * sizeof(float))
        {
            const float* c = (const float*)col.data();
            color[0] = c[0];
            color[1] = c[1];
            color[2] = c[2];
            has_color = 1;
        }
        /* PBR material: a bound UsdPreviewSurface/MDL shader drives base/metallic/roughness;
         * else a default material whose base color is the displayColor. One material per mesh
         * (pushed in mesh order), so material_index == the mesh's index.
         *
         * The graph inputs above were batched at this ordinal. Memoise the final resolved
         * material by shader identity as well (the graph is shared; only the mesh's own
         * displayColor fallback differs, and that applies only when there is no material). */
        GpuMaterialParams mat;
        std::string shader_key;
        const auto material_it = material_by_prim.find(path);
        if (material_it != material_by_prim.end() && !material_it->second.empty())
        {
            const auto shader_it = shader_by_material.find(material_it->second);
            if (shader_it != shader_by_material.end())
                shader_key = shader_it->second;
        }
        auto mc = shader_key.empty() ? material_cache.end() : material_cache.find(shader_key);
        if (mc != material_cache.end())
        {
            mat = mc->second;
            has_color = 1;
        }
        else if (readMaterial(stage, ordinal, shader_key, &mat, color, usdz_archives))
        {
            has_color = 1;
            if (!shader_key.empty())
                material_cache.emplace(shader_key, mat);
        }
        else
        {
            defaultMaterial(&mat);
            mat.base_color[0] = color[0];
            mat.base_color[1] = color[1];
            mat.base_color[2] = color[2];
        }
        int mat_idx = (int)g_materials.size();
        g_materials.push_back(mat);
        /* Debug aid (OVGL_DEBUG_MATERIALS=1): one line per mesh showing how its
         * material graph resolved. Diagnoses binding/terminal/texture gaps. */
        static const bool s_kDebugMaterials = std::getenv("OVGL_DEBUG_MATERIALS") != nullptr;
        if (s_kDebugMaterials)
        {
            const auto dbg_material = material_by_prim.find(path);
            std::fprintf(stderr, "[mat] %s material=%s shader=%s base=(%.3f,%.3f,%.3f) diffuse_tex=%d uv=%zu\n",
                         path.c_str(), dbg_material != material_by_prim.end() ? dbg_material->second.c_str() : "<none>",
                         shader_key.empty() ? "<none>" : shader_key.c_str(), mat.base_color[0], mat.base_color[1],
                         mat.base_color[2], mat.tex_indices[0], UV.size() / 2);
        }

        double world16[16];
        readWorldMatrix(stage, ordinal, path, world16);

        const int double_sided = readBooleanAttribute(stage, ordinal, path, "doubleSided", false) ? 1 : 0;
        const std::string orientation = readStringAttribute(stage, ordinal, path, "orientation");
        const int front_face_cw = (orientation == "leftHanded") ? 1 : 0;

        SceneMesh m = makeMesh(P, N, UV, color, has_color, world16, path, double_sided, front_face_cw);
        m.material_index = mat_idx;
        m.prototype_idx = (int)meshes.size();
        meshes.push_back(m);
    }

    const double t_meshloop = profileLoad() ? loadNowMs() : 0.0;
    if (profileLoad())
    {
        std::fprintf(stderr,
                     "[load] uncached single-prim reads=%ld prims=%zu "
                     "prefetch=%.3fms meshloop=%.3fms\n",
                     g_reads, prims.size(), prefetchedTime - t_start, t_meshloop - prefetchedTime);
        logTextureProfile();
    }
    batchClear();
    std::memset(out, 0, sizeof(*out));
    out->up_axis = authoredUpAxis;
    /* UsdLux defaults, overwritten by readSceneDome below when the stage
     * actually has a DomeLight. has_dome (memset to 0) is what decides
     * whether they are consulted at all. */
    out->dome_intensity = 1.0f;
    out->dome_color[0] = out->dome_color[1] = out->dome_color[2] = 1.0f;
    out->_meshes_heap = 1;
    out->nmeshes = (int)meshes.size();
    if (!meshes.empty())
    {
        out->meshes = (SceneMesh*)std::malloc(meshes.size() * sizeof(SceneMesh));
        std::memcpy(out->meshes, meshes.data(), meshes.size() * sizeof(SceneMesh));
    }
    if (!ovgl_expand_point_instancers(stage, ordinal, out))
    {
        g_err = ovgl_point_instancer_last_error();
        ovgl_scene_free(out);
        g_materials.clear();
        return 0;
    }
    float lo[3] = { 1e30f, 1e30f, 1e30f }, hi[3] = { -1e30f, -1e30f, -1e30f };
    for (int i = 0; i < out->nmeshes; i++)
    {
        if (!out->meshes[i].visible || out->meshes[i].is_proto_only)
            continue;
        for (int c = 0; c < 3; c++)
        {
            if (out->meshes[i].bounds_min[c] < lo[c])
                lo[c] = out->meshes[i].bounds_min[c];
            if (out->meshes[i].bounds_max[c] > hi[c])
                hi[c] = out->meshes[i].bounds_max[c];
        }
    }
    std::memcpy(out->bounds_min, lo, sizeof(lo));
    std::memcpy(out->bounds_max, hi, sizeof(hi));
    /* Build keeps its historical leniency: a failed light discovery renders
     * as "no authored lights" rather than failing the whole build. The dome
     * query gets the same treatment for the same reason. */
    (void)readSceneLights(stage, ordinal, prototypeRoots);
    (void)readSceneDome(stage, ordinal, prototypeRoots, out);
    out->nlights_unsupported = countUnsupportedLights(stage, ordinal, prototypeRoots);
    return 1;
}

/* Read worldMatrix for EVERY cached mesh in ONE query.
 *
 * The obvious loop — readWorldMatrix() per mesh — builds and tears down a query handle and a
 * read iterator per prim: on Kitchen_set (1788 meshes) that is ~197ms, which alone makes an
 * interactive drag impossible even after the geometry rebuild is skipped. One batched read of
 * the same column costs a few ms. Matrices land in `out` indexed by mesh order; meshes ovstage
 * has no worldMatrix for keep whatever they had (identity from the build). */
static bool readWorldMatricesBatched(ovstage_instance_t* stage,
                                     ovstage_ordinal_t ordinal,
                                     Scene* s,
                                     std::vector<std::array<double, 16>>& out)
{
    out.assign((size_t)s->nmeshes, std::array<double, 16>{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 });
    if (s->nmeshes <= 0)
        return true;

    std::vector<std::string> paths;
    paths.reserve((size_t)s->nmeshes);
    for (int i = 0; i < s->nmeshes; i++)
    {
        const char* p = s->meshes[i].path ? s->meshes[i].path : "";
        paths.emplace_back(p);
    }
    std::vector<std::vector<uint8_t>> rows;
    if (!readBatchedAttributes(stage, ordinal, paths, "omni:fabric:worldMatrix", rows))
        return false;
    constexpr size_t matrix_bytes = 16 * sizeof(double);
    for (size_t index = 0; index < rows.size(); ++index)
    {
        if (rows[index].empty())
            continue;
        if (rows[index].size() != matrix_bytes)
        {
            g_err = "worldMatrix has an invalid byte extent for " + paths[index];
            return false;
        }
        std::memcpy(out[index].data(), rows[index].data(), matrix_bytes);
    }
    return true;
}

extern "C" int ovgl_scene_requires_point_instancer_rebuild(const Scene* s)
{
    if (!s || !s->meshes)
        return 0;
    for (int index = 0; index < s->nmeshes; ++index)
    {
        const SceneMesh& mesh = s->meshes[index];
        if (mesh.is_proto_only || mesh.prototype_idx != index)
            return 1;
    }
    return 0;
}

extern "C" int ovgl_scene_refresh_materials(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, const Scene* s, GpuMaterialParams* materials, int nmaterials)
{
    if (!stage || !s || !materials || nmaterials <= 0)
    {
        g_err = "null arg";
        return 0;
    }
    /* Value-only re-resolution: the binding chain (mesh -> material ->
     * surface shader) is re-derived per mesh path — identical to the build's
     * result when only shader input values changed, and honestly newer when
     * a binding DID change — then the shader constants are re-read at
     * `ordinal` into the mesh's existing slot of the CALLER's array. Clones
     * share their prototype's path and slot, so repeated writes are
     * idempotent. Texture indices must come back EQUAL to the slot's
     * existing ones: the texture registry is global build scratch that may
     * belong to another renderer's scene, so slot-equality is the only
     * provably-consistent binding into the caller's texture snapshot — any
     * difference (a changed inputs:file, a registry rebuilt by another
     * build) fails the refresh and the caller rebuilds. */
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache usdz_archives;
    std::unordered_map<std::string, std::string> shader_by_mesh_memo;
    std::unordered_map<std::string, GpuMaterialParams> params_by_shader_memo;
    for (int i = 0; i < s->nmeshes; i++)
    {
        const SceneMesh& m = s->meshes[i];
        if (!m.path)
            continue;
        if (m.material_index < 0 || m.material_index >= nmaterials)
            continue;
        const std::string path = m.path;
        auto sit = shader_by_mesh_memo.find(path);
        if (sit == shader_by_mesh_memo.end())
        {
            const std::string material = boundMaterialPath(stage, ordinal, path);
            sit = shader_by_mesh_memo
                      .emplace(path, material.empty() ? std::string() : materialShaderPath(stage, ordinal, material))
                      .first;
        }
        const std::string& shader = sit->second;
        if (shader.empty())
            continue; /* displayColor fallback entry: keep */
        auto mit = params_by_shader_memo.find(shader);
        if (mit == params_by_shader_memo.end())
        {
            GpuMaterialParams mat;
            float base[3] = { 0.7f, 0.7f, 0.7f };
            if (!readMaterial(stage, ordinal, shader, &mat, base, usdz_archives))
            {
                g_err = "material refresh: re-read failed for " + shader;
                return 0;
            }
            mit = params_by_shader_memo.emplace(shader, mat).first;
        }
        GpuMaterialParams& slot = materials[m.material_index];
        if (std::memcmp(mit->second.tex_indices, slot.tex_indices, sizeof(slot.tex_indices)) != 0)
        {
            g_err = "material refresh: texture bindings changed for " + shader +
                    " (value-only refresh cannot rebind textures)";
            return 0;
        }
        slot = mit->second;
    }
    return 1;
}

extern "C" int ovgl_scene_refresh_xforms(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* s)
{
    if (!stage || !s)
    {
        g_err = "null arg";
        return 0;
    }
    std::vector<std::array<double, 16>> world;
    if (!readWorldMatricesBatched(stage, ordinal, s, world))
        return 0;

    std::vector<double> dense;
    try
    {
        dense.reserve(world.size() * 16);
        for (const auto& matrix : world)
            dense.insert(dense.end(), matrix.begin(), matrix.end());
    }
    catch (const std::bad_alloc&)
    {
        g_err = "allocating transform batch failed";
        return 0;
    }
    return ovgl_scene_apply_xforms(s, dense.empty() ? nullptr : dense.data(), world.size());
}

extern "C" int ovgl_scene_apply_xforms(Scene* s, const double* world_matrices, size_t mesh_count)
{
    if (!s || mesh_count != static_cast<size_t>(std::max(s->nmeshes, 0)) || (mesh_count > 0 && !world_matrices))
    {
        g_err = "transform batch does not match the resident mesh set";
        return 0;
    }

    struct PendingMesh
    {
        std::array<double, 16> world{};
        std::array<float, 3> boundsMin{};
        std::array<float, 3> boundsMax{};
    };
    std::vector<PendingMesh> pending;
    try
    {
        pending.resize(mesh_count);
    }
    catch (const std::bad_alloc&)
    {
        g_err = "allocating transform batch state failed";
        return 0;
    }

    float lo[3] = { 1e30f, 1e30f, 1e30f };
    float hi[3] = { -1e30f, -1e30f, -1e30f };
    for (int i = 0; i < s->nmeshes; i++)
    {
        const SceneMesh& m = s->meshes[i];
        PendingMesh& update = pending[static_cast<size_t>(i)];
        const double* world = world_matrices + static_cast<size_t>(i) * 16;
        if (!std::all_of(world, world + 16, [](double value) { return std::isfinite(value); }))
        {
            g_err = "transform batch contains a non-finite matrix";
            return 0;
        }
        std::memcpy(update.world.data(), world, sizeof(update.world));
        if (!m.path)
            continue;

        /* World bounds must follow the transform.  ovgl_scene_build derives them from the
         * object-space bounds and the world matrix; a refresh that moved the prim but left
         * the old world AABB behind would leave every consumer of bounds reading the prim's
         * PREVIOUS location: the shadow-map fit, the framing bounds, and ovgl_pick (whose
         * broadphase rejects against exactly these boxes, so a moved prim would stop being
         * pickable where it is drawn).  Re-derive from the 8 object-space corners. */
        const float* olo = m.local_bounds_min;
        const float* ohi = m.local_bounds_max;
        float mlo[3] = { 1e30f, 1e30f, 1e30f };
        float mhi[3] = { -1e30f, -1e30f, -1e30f };
        for (int corner = 0; corner < 8; corner++)
        {
            const double p[3] = { (corner & 1) ? (double)ohi[0] : (double)olo[0],
                                  (corner & 2) ? (double)ohi[1] : (double)olo[1],
                                  (corner & 4) ? (double)ohi[2] : (double)olo[2] };
            /* USD row-vector convention: p_world = p_obj * W. */
            for (int c = 0; c < 3; c++)
            {
                const double w = p[0] * world[c] + p[1] * world[4 + c] + p[2] * world[8 + c] + world[12 + c];
                if ((float)w < mlo[c])
                    mlo[c] = (float)w;
                if ((float)w > mhi[c])
                    mhi[c] = (float)w;
            }
        }
        std::memcpy(update.boundsMin.data(), mlo, sizeof(mlo));
        std::memcpy(update.boundsMax.data(), mhi, sizeof(mhi));
        if (m.is_proto_only || m.nvertices <= 0)
            continue;
        for (int c = 0; c < 3; c++)
        {
            if (mlo[c] < lo[c])
                lo[c] = mlo[c];
            if (mhi[c] > hi[c])
                hi[c] = mhi[c];
        }
    }
    for (int i = 0; i < s->nmeshes; ++i)
    {
        SceneMesh& mesh = s->meshes[i];
        if (!mesh.path)
            continue;
        const PendingMesh& update = pending[static_cast<size_t>(i)];
        std::memcpy(mesh.world_xform, update.world.data(), sizeof(mesh.world_xform));
        std::memcpy(mesh.bounds_min, update.boundsMin.data(), sizeof(mesh.bounds_min));
        std::memcpy(mesh.bounds_max, update.boundsMax.data(), sizeof(mesh.bounds_max));
    }
    if (lo[0] <= hi[0])
    {
        std::memcpy(s->bounds_min, lo, sizeof(lo));
        std::memcpy(s->bounds_max, hi, sizeof(hi));
    }
    return 1;
}

extern "C" int ovgl_scene_refresh_lights(ovstage_instance_t* stage, ovstage_ordinal_t ordinal)
{
    if (!stage)
    {
        g_err = "null arg";
        return 0;
    }
    /* Same reader as the build (point reads against the live store — the
     * build's batch prefetch is already torn down before readSceneLights
     * runs there too), so a refreshed light is byte-identical to what a full
     * rebuild at this ordinal would derive. Re-reading worldMatrix here is
     * what makes lights follow ANCESTOR transform edits as well: the public
     * hierarchy computation recomposed every world matrix before sealing. */
    if (!readSceneLights(stage, ordinal, prototypeRootPaths(stage)))
    {
        if (g_err.empty())
            g_err = "light discovery failed";
        return 0;
    }
    return 1;
}

extern "C" int ovgl_scene_refresh_dome(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* s)
{
    if (!stage || !s)
    {
        g_err = "null arg";
        return 0;
    }
    const std::vector<std::string> prototypeRoots = prototypeRootPaths(stage);
    /* Same reset the build does before it reads (ovgl_scene_build memsets the
     * whole Scene and then re-establishes these UsdLux defaults), so a dome
     * that is no longer authored/visible at this ordinal resolves to the
     * no-environment state instead of keeping the previous dome forever. Only
     * the dome fields: everything else in `s` is the live cached scene. */
    s->has_dome = 0;
    s->dome_hdr_path[0] = '\0';
    s->dome_intensity = 1.0f;
    s->dome_color[0] = s->dome_color[1] = s->dome_color[2] = 1.0f;
    s->dome_rotation_y = 0.0f;
    /* The build is deliberately lenient here (a failed dome query renders as
     * "no authored dome"); the fast path is not. readSceneDome returning
     * false means the QUERY failed, and a fast path that shrugged that off
     * would publish an ordinal whose environment silently reverted to the
     * defaults just written above. Fail closed and let the caller rebuild. */
    if (!readSceneDome(stage, ordinal, prototypeRoots, s))
    {
        if (g_err.empty())
            g_err = "dome discovery failed";
        return 0;
    }
    /* applyEnvironment's resolved request also carries the count of active
     * lights ovgl cannot evaluate (it is what decides whether a dome may
     * replace the synthetic fill rig), and that count was build-only too. */
    s->nlights_unsupported = countUnsupportedLights(stage, ordinal, prototypeRoots);
    return 1;
}

extern "C" void ovgl_scene_free(Scene* s)
{
    if (!s || !s->meshes)
    {
        if (s)
            std::memset(s, 0, sizeof(*s));
        return;
    }
    for (int i = 0; i < s->nmeshes; i++)
    {
        SceneMesh& m = s->meshes[i];
        std::free(m.positions);
        std::free(m.normals);
        std::free(m.colors);
        std::free(m.texcoords);
        std::free(m.indices);
        std::free(m.ptex_tri_colors);
        std::free(m.path);
    }
    std::free(s->meshes);
    std::memset(s, 0, sizeof(*s));
}

extern "C" int ovgl_scene_textures(const GpuTextureData** out)
{
    if (out)
        *out = g_textures.data();
    return (int)g_textures.size();
}

extern "C" int ovgl_scene_materials(const GpuMaterialParams** out)
{
    if (out)
        *out = g_materials.data();
    return (int)g_materials.size();
}

extern "C" int ovgl_scene_lights(const GpuLight** out)
{
    if (out)
        *out = g_lights.data();
    return (int)g_lights.size();
}

extern "C" const char* ovgl_scene_last_error(void)
{
    return g_err.c_str();
}
