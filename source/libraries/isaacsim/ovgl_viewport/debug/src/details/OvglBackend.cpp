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

/* OvglBackend.cpp — the private OVStage-to-OVGL adapter.
 *
 * This translation unit uses only the official public OVStage API and the
 * module-private OVGL rasterizer. OvglBackend.hpp declares the small
 * subset consumed by Renderer.cpp; the remaining helpers implement stage
 * synchronization and RenderProduct derivation behind that boundary.
 *
 * Data-plane composition follows OVGL's Scene.cpp read path:
 *   create_prims("Mesh") == UPSERT write of interned usd-prim-type token
 *   points/counts/indices/displayColor == ragged per-prim byte writes
 *   localMatrix == flat double[16] write
 *   step == seal(o) -> ovstage_compute_hierarchy(o, o+1) -> seal(o+1)
 *           -> ovgl.render_frame(o+1)
 */
#include "OvglBackend.hpp"

#include "OvstageHelpers.hpp"
#include "ovgl/Ovgl.h"

#include <isaacsim/common/ovstage/TransformJournal.hpp>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_instancing.h>
#include <ovstage/ovstage_population.h>
#include <ovx/path_dictionary/path_dictionary.h>
#include <ovx/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{

namespace
{

enum
{
    BE_VAR_FMT_RGBA8 = 0,
    BE_VAR_FMT_F32_DEPTH = 1,
};

thread_local std::string g_backendError;
void setBackendError(const std::string& s)
{
    g_backendError = s;
}

/* Per-step profiling, same switch as ovgl's render breakdown (OVGL_PROFILE=1). */
bool isProfilingEnabled()
{
    static const bool s_kEnabled = std::getenv("OVGL_PROFILE") != nullptr;
    return s_kEnabled;
}
double getCurrentTimeMilliseconds()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ovx_string_t toOvx(const std::string& s)
{
    ovx_string_t out{};
    out.ptr = s.c_str();
    out.length = s.size();
    return out;
}

} // namespace

namespace
{

// One pulled attribute column for one prim, at the latest ordinal <= N.
// (Defined before OvglBackend: the attached-step fast paths retain the previous
// step's pulled scene there.)
struct PulledColumn
{
    std::vector<uint8_t> bytes;
    DLDataType dtype{};
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    bool is_array = false;
    ovstage_ordinal_t ordinal = 0;
    bool deleted = false;
    bool recorded = false;
};
// prim path -> attr name -> column
using PulledScene = std::map<std::string, std::map<std::string, PulledColumn>>;

constexpr std::array<const char*, 2> g_kResetXformStackColumns = {
    "omni:resetXformStack",
    "omni:fabric:resetXformStack",
};

struct SceneGraphInstanceIndex
{
    std::map<std::string, std::vector<std::string>> instanceRootsByPrototype;
    std::set<std::string> instanceRoots;
};

} // namespace

struct DerivedProduct
{
    std::string path;
    std::string camera_path;
    int32_t width = 0, height = 0;
    std::vector<std::string> var_names;
    // Optional RenderProduct dataWindowNDC crop (x0, y0, x1, y1 in [0,1]).
    bool has_data_window = false;
    float data_window[4] = { 0.f, 0.f, 1.f, 1.f };
    // Camera projection scalars + stage meters-per-unit, resolved from the
    // same pulled snapshot the product was derived from. Carrying them here
    // lets a cached derivation render without re-pulling the scene (the
    // local lane's per-step full pull measured 292 ms on Kitchen_set); any
    // edit to these authored values is a non-transform change, which the
    // fast-path classifier routes back through a full re-derive.
    float focal = 50.0f; // g_kUsdDefaultFocalLength
    float h_aperture = 20.955f; // g_kUsdDefaultHorizontalAperture
    // No verticalAperture: the vertical half-angle is conformed from
    // horizontalAperture and this product's own aspect, so the camera's
    // verticalAperture never reaches the projection. See the aperture-axis
    // note in renderDerivedProduct for the measurements behind that.
    double meters_per_unit = 0.0;
};

struct PresentRequest
{
    unsigned framebuffer = 0;
    int destinationWidth = 0;
    int destinationHeight = 0;
    const char* overlayText = nullptr;
};

struct TransformJournal
{
    uint64_t generation = 0;
    ovstage_ordinal_t baseOrdinal = 0;
    ovstage_ordinal_t ordinal = 0;
    std::vector<std::string> paths;
    std::vector<std::array<double, 16>> worlds;
};

struct JournalMeshRoute
{
    size_t driverIndex = std::numeric_limits<size_t>::max();
    std::array<double, 16> relativeOrStatic{};
};

struct OvglBackend
{
    ovstage_instance_t* stage = nullptr;
    ovgl_renderer_t* renderer = nullptr;

    // Accumulating write ordinal. Writes target cur_ordinal; readers seal and
    // advance it before observing pending data.
    ovstage_ordinal_t cur_ordinal = 1;

    // Prim / attribute existence bookkeeping for ovrtx prim-mode semantics.
    std::set<std::string> prims;

    // Last rendered RGBA8 buffer.
    std::vector<uint8_t> frame;

    // ── Attach lane (ovrtx 0.4) ──
    // Borrowed external instance (never written; consumed only through its
    // official 28-slot public vtable + path-dictionary vtable).
    ovstage_instance_t* ext = nullptr;
    // Prims currently materialized in the mirror from a pull (so a prim that
    // disappears from the attached stage can be tombstoned in the mirror).
    std::set<std::string> mirror_pulled;
    // Columns currently LIVE in the mirror per pulled prim (verbatim-mirrored
    // attrs + the derived localMatrix). The mirror is an accumulating UPSERT
    // store, so a column that drops out of a later pull — deleted on the
    // attached stage — must be tombstoned explicitly; this set is the ground
    // truth the full pass diffs against (fast-path adversary F2, 2026-07-19).
    std::map<std::string, std::set<std::string>> mirror_cols;
    // Steady-state cache for attached renders (perf; 2026-07-18 rig profile:
    // ~2.8 s/step of which >99% re-derived unchanged state). Holds the merged
    // committed snapshot of the attached stage as of `attached_pull_ordinal`.
    // Sound because committed data is sealed and immutable, so a step at the
    // same ordinal is a byte-identical pull and a later ordinal is exactly the
    // prior snapshot + the changes in (attached_pull_ordinal, ordinal].
    // Invalidated whenever the mirror stops matching it (resetStage —
    // which attach/detach also route through).
    PulledScene attached_pull;
    // Unexpanded source snapshot and membership. Keeping this separate from
    // attached_pull lets ordinal-window reads update ordinary frames without
    // re-reading every value column or confusing source prims with generated
    // instance proxies.
    PulledScene attached_source_pull;
    std::set<std::string> attached_source_attributes;
    // Prefix-queryable dependencies used to invalidate only instance expansions
    // affected by a transform-only ordinal window.
    SceneGraphInstanceIndex attached_scene_graph_instances;
    ovstage_ordinal_t attached_pull_ordinal = 0;
    bool attached_pull_valid = false;
    // Mirror ordinal the last successful attached step rendered at; renders
    // for proven-unchanged steps reuse it to read the current camera.
    ovstage_ordinal_t attached_render_ordinal = 0;
    // Mirror ordinal backing OVGL's cached geometry/material/light scene. A
    // camera-only edit advances attached_render_ordinal while this stays put:
    // the camera is applied separately and the resident scene remains valid.
    ovstage_ordinal_t attached_scene_ordinal = 0;
    // Direct transform batches intentionally leave the private mirror at its
    // previous ordinal. Consecutive direct/unchanged frames remain valid, but
    // any edit that needs the mirror must first take a full synchronization.
    bool mirror_transform_dirty = false;
    // Authored light and dome paths from the retained expanded snapshot. A
    // transform of one of these paths (or an ancestor) must use the mirror path
    // because OVGL lights bake their world pose at refresh/build time.
    std::set<std::string> attached_light_paths;
    bool attached_light_paths_valid = false;
    // A producer journal can advance the resident scene without updating the
    // retained pulled snapshots. While dirty, only a continuous compatible
    // journal (or an unchanged ordinal) may remain on the fast path.
    bool pulled_transform_dirty = false;
    uint64_t transform_journal_generation = 0;
    std::vector<std::string> transform_journal_paths;
    std::vector<JournalMeshRoute> transform_journal_routes;
    // Per-product / per-var outputs of the last attached render. `data` is RGBA8 for
    // BE_VAR_FMT_RGBA8 and row-major float32 meters for BE_VAR_FMT_F32_DEPTH
    // (both w*h*4 bytes; see the iface enumeration contract).
    struct AttachedVar
    {
        std::string name;
        std::vector<uint8_t> data;
        int format = BE_VAR_FMT_RGBA8;
        int w = 0, h = 0;
    };
    struct AttachedProduct
    {
        std::string path;
        int width = 0;
        int height = 0;
        std::vector<AttachedVar> vars;
    };
    std::vector<AttachedProduct> attached_products;

    void note(const std::string& path)
    {
        prims.insert(path);
    }
};

namespace
{

bool completeEnqueue(OvglBackend* be,
                     ovstage_enqueue_result_t enqueue,
                     const std::string& operation,
                     bool* out_completed = nullptr)
{
    if (out_completed)
        *out_completed = false;
    if (!be || !be->stage || enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
    {
        setBackendError(operation + " enqueue failed");
        return false;
    }

    ovstage_op_wait_result_t wait_result{};
    const ovstage_api_status_t wait_status =
        ovstage_wait_op(be->stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait_result);
    const ovstage_api_status_t release_status = ovstage_release_op(be->stage, enqueue.op_index);
    if (wait_status == OVSTAGE_OK && wait_result.error_op_id_count == 0 && out_completed)
    {
        *out_completed = true;
    }
    if (wait_status != OVSTAGE_OK || wait_result.error_op_id_count != 0)
    {
        setBackendError(operation + " wait failed");
        return false;
    }
    if (release_status != OVSTAGE_OK)
    {
        setBackendError(operation + " op release failed");
        return false;
    }
    return true;
}

// An explicit OVStage query borrows its caller-owned path list. Keep both
// handles together so no call site can release the list before the query has
// been released and its asynchronous release operation has completed.
struct owned_path_query
{
    path_dictionary_instance_t* dictionary = nullptr;
    ovx_primpath_list_t paths = OVX_INVALID_PRIMPATH_LIST;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
};

bool createOwnedPathQuery(
    OvglBackend* be, const ovx_string_t* paths, size_t path_count, const std::string& operation, owned_path_query* out)
{
    if (!be || !be->stage || !paths || path_count == 0 || !out)
    {
        setBackendError(operation + ": invalid explicit-query arguments");
        return false;
    }

    *out = {};
    out->dictionary = ovstage_get_path_dictionary(be->stage);
    if (!out->dictionary)
    {
        setBackendError(operation + ": no path dictionary");
        return false;
    }
    const ovx_api_result_t path_status =
        path_dictionary_create_path_list_from_strings(out->dictionary, paths, path_count, &out->paths);
    if (path_status.status != OVX_API_SUCCESS || out->paths == OVX_INVALID_PRIMPATH_LIST)
    {
        if (out->paths != OVX_INVALID_PRIMPATH_LIST)
        {
            (void)path_dictionary_release_path_list_reference(out->dictionary, out->paths);
            out->paths = OVX_INVALID_PRIMPATH_LIST;
        }
        setBackendError(operation + ": failed to create path list");
        return false;
    }

    const ovstage_api_status_t query_status = ovstage_query_from_path_list(be->stage, out->paths, &out->query);
    if (query_status == OVSTAGE_OK && out->query != OVSTAGE_INVALID_QUERY_HANDLE)
        return true;

    bool query_released = true;
    if (out->query != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        query_released =
            completeEnqueue(be, ovstage_release_query(be->stage, out->query), operation + ": failed-query cleanup");
        if (query_released)
            out->query = OVSTAGE_INVALID_QUERY_HANDLE;
    }
    if (query_released)
    {
        (void)path_dictionary_release_path_list_reference(out->dictionary, out->paths);
        out->paths = OVX_INVALID_PRIMPATH_LIST;
    }
    setBackendError(operation + ": failed to create query");
    return false;
}

bool releaseOwnedPathQuery(OvglBackend* be, owned_path_query* owned, const std::string& operation)
{
    if (!be || !be->stage || !owned || !owned->dictionary)
    {
        setBackendError(operation + ": invalid owned query");
        return false;
    }
    if (owned->query != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        if (!completeEnqueue(be, ovstage_release_query(be->stage, owned->query), operation))
            return false;
        owned->query = OVSTAGE_INVALID_QUERY_HANDLE;
    }
    if (owned->paths != OVX_INVALID_PRIMPATH_LIST)
    {
        if (path_dictionary_release_path_list_reference(owned->dictionary, owned->paths).status != OVX_API_SUCCESS)
        {
            setBackendError(operation + ": failed to release path list");
            return false;
        }
        owned->paths = OVX_INVALID_PRIMPATH_LIST;
    }
    owned->dictionary = nullptr;
    return true;
}

size_t dlElemBytes(const DLDataType& dt);
bool tensorTotalBytes(const DLTensor& tensor, size_t* out);

// Seal an ordinal (advance the global write floor + drain the enqueue).
bool sealOrdinal(OvglBackend* be, ovstage_ordinal_t ordinal, bool* out_sealed = nullptr)
{
    ovstage_write_floor_desc_t d{};
    d.ordinal = ordinal;
    d.scope = OVSTAGE_SCOPE_ALL;
    d.attributes = nullptr;
    d.attribute_count = 0;
    return completeEnqueue(be, ovstage_advance_write_floor(be->stage, &d), "advance_write_floor", out_sealed);
}

// Write raw bytes to (path, attr) at ordinal. `ragged` => one variable-length
// row per prim (2-D [1,nbytes]); otherwise a flat fixed buffer (1-D [nbytes]).
// Scene.cpp::readAttributeHost (CPU-only; the backend authors CPU tensors).
// When out_dtype/out_is_array are non-null they receive the stored column's
// DLPack dtype and ragged flag (finding 44's store-typed no-hint reads).
bool readAttributeHost(OvglBackend* be,
                       ovstage_ordinal_t ordinal,
                       const std::string& prim_path,
                       const std::string& attr_name,
                       std::vector<uint8_t>& out,
                       DLDataType* out_dtype = nullptr,
                       bool* out_is_array = nullptr,
                       ovstage_attribute_semantic_t* out_semantic = nullptr)
{
    out.clear();
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(be->stage);
    if (!dict)
    {
        setBackendError("no path dictionary");
        return false;
    }

    ovx_string_t an = toOvx(attr_name);
    ovx_token_t tok = OVX_INVALID_TOKEN;
    if (path_dictionary_create_tokens_from_strings(dict, &an, 1, &tok).status != OVX_API_SUCCESS ||
        tok == OVX_INVALID_TOKEN)
    {
        setBackendError("intern(" + attr_name + ")");
        return false;
    }
    ovx_string_t ps = toOvx(prim_path);
    owned_path_query query;
    if (!createOwnedPathQuery(be, &ps, 1, "query(" + prim_path + ")", &query))
        return false;

    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    ovstage_ordinal_range_t range{ 0, ordinal, false }; // latest <= ordinal
    ovstage_enqueue_result_t er = ovstage_read_attributes(be->stage, query.query, &tok, 1, range, &rh);
    if (er.status != OVSTAGE_OK || rh == OVSTAGE_INVALID_READ_HANDLE)
    {
        (void)releaseOwnedPathQuery(be, &query, "release query after failed read");
        setBackendError("read_attributes(" + prim_path + "." + attr_name + ")");
        return false;
    }
    bool got = false, err = false;
    for (;;)
    {
        ovstage_read_group_t g{};
        ovstage_api_status_t fr = ovstage_fetch_read_next(be->stage, rh, OVSTAGE_TIMEOUT_INFINITE, &g);
        if (fr == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fr != OVSTAGE_OK)
        {
            setBackendError("fetch_read_next(" + prim_path + "." + attr_name + ")");
            err = true;
            break;
        }
        if (!got && !g.is_delete && g.data.tensor_count >= 1 && g.data.tensors && g.prims.count > 0)
        {
            const DLTensor& t0 = g.data.tensors[0];
            const uint8_t* base = t0.data ? static_cast<const uint8_t*>(t0.data) + t0.byte_offset : nullptr;
            size_t total = 0;
            bool valid = t0.device.device_type == kDLCPU && tensorTotalBytes(t0, &total);
            size_t b0 = 0, b1 = total;
            if (valid && g.data.tensor_count >= 2)
            {
                const DLTensor& offsets = g.data.tensors[1];
                size_t offset_bytes = 0;
                valid = offsets.data && offsets.device.device_type == kDLCPU && offsets.dtype.code == kDLUInt &&
                        offsets.dtype.bits == 64 && offsets.dtype.lanes == 1 && offsets.ndim == 1 && offsets.shape &&
                        offsets.shape[0] >= 2 && tensorTotalBytes(offsets, &offset_bytes) &&
                        offset_bytes >= 2 * sizeof(uint64_t);
                if (valid)
                {
                    const uint8_t* offset_data = static_cast<const uint8_t*>(offsets.data) + offsets.byte_offset;
                    uint64_t begin = 0, end = 0;
                    std::memcpy(&begin, offset_data, sizeof(begin));
                    std::memcpy(&end, offset_data + sizeof(begin), sizeof(end));
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
                if (b1 > b0)
                    std::memcpy(out.data(), base + b0, b1 - b0);
                if (out_dtype)
                    *out_dtype = t0.dtype;
                if (out_is_array)
                    *out_is_array = g.is_array;
                if (out_semantic)
                    *out_semantic = g.semantic;
                got = true;
            }
            else
            {
                setBackendError("invalid read group(" + prim_path + "." + attr_name + ")");
                err = true;
            }
        }
        if (ovstage_release_group(be->stage, &g) != OVSTAGE_OK)
        {
            setBackendError("release_group(" + prim_path + "." + attr_name + ")");
            err = true;
            break;
        }
    }
    if (!completeEnqueue(be, er, "read(" + prim_path + "." + attr_name + ")"))
        err = true;
    if (!completeEnqueue(be, ovstage_release_read(be->stage, rh), "release read(" + prim_path + "." + attr_name + ")"))
        err = true;
    if (!releaseOwnedPathQuery(be, &query, "release query after read(" + prim_path + "." + attr_name + ")"))
        err = true;
    return !err && got;
}

/* Attach-lane helpers (ovrtx 0.4).
 *
 * Everything touching the EXTERNAL instance goes through the official public
 * vtable (the `ovstage_*` static-inline wrappers dispatch
 * instance->vtable->slot) and the path-dictionary vtable — never through
 * engine internals. The mirror (be->stage) is this backend's own instance and
 * uses the same public surface.
 */

// Wait for + release an enqueued op on an arbitrary instance (external or
// mirror). Returns false (with g_backendError set) on any failure.
bool instComplete(ovstage_instance_t* inst, ovstage_enqueue_result_t enqueue, const std::string& operation)
{
    if (!inst || enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
    {
        setBackendError(operation + " enqueue failed");
        return false;
    }
    ovstage_op_wait_result_t wr{};
    const ovstage_api_status_t ws = ovstage_wait_op(inst, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wr);
    const ovstage_api_status_t rs = ovstage_release_op(inst, enqueue.op_index);
    if (ws != OVSTAGE_OK || wr.error_op_id_count != 0)
    {
        setBackendError(operation + " wait failed");
        return false;
    }
    if (rs != OVSTAGE_OK)
    {
        setBackendError(operation + " op release failed");
        return false;
    }
    return true;
}

// Compute the renderer-owned derived hierarchy through the official OVStage
// API. The input ordinal is already sealed; the output ordinal is sealed by
// the caller only after this operation completes.
bool computeWorldXforms(ovstage_instance_t* stage,
                        ovstage_ordinal_t input_ordinal,
                        ovstage_ordinal_t output_ordinal,
                        const std::string& operation)
{
    return instComplete(
        stage,
        ovstage_compute_hierarchy(stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU, input_ordinal, output_ordinal),
        operation);
}

// Token -> string through a dictionary's vtable (dict-owned storage; copied).
bool getDictionaryTokenString(path_dictionary_instance_t* dict, ovx_token_t tok, std::string* out)
{
    if (!dict || !dict->vtable || !out || tok == OVX_INVALID_TOKEN)
        return false;
    ovx_string_t s{};
    if (dict->vtable->get_strings_from_tokens(dict->context, &tok, 1, &s).status != OVX_API_SUCCESS)
        return false;
    out->assign(s.ptr ? s.ptr : "", s.ptr ? s.length : 0);
    return true;
}

// Prim-path handle -> "/a/b/c" through a dictionary's vtable.
bool getDictionaryPathString(path_dictionary_instance_t* dict, ovx_primpath_t path, std::string* out)
{
    if (!dict || !dict->vtable || !out || path == OVX_INVALID_PRIMPATH)
        return false;
    std::vector<ovx_token_t> tok_buf(64);
    ovx_token_t* per_path = nullptr;
    size_t per_path_n = 0;
    size_t processed = 0;
    for (;;)
    {
        if (dict->vtable
                ->get_tokens_from_paths(
                    dict->context, &path, 1, tok_buf.data(), tok_buf.size(), &per_path, &per_path_n, &processed)
                .status != OVX_API_SUCCESS)
            return false;
        if (processed >= 1)
            break;
        tok_buf.resize(tok_buf.size() * 2);
    }
    std::string s;
    for (size_t j = 0; j < per_path_n; ++j)
    {
        ovx_string_t comp{};
        if (dict->vtable->get_strings_from_tokens(dict->context, &per_path[j], 1, &comp).status != OVX_API_SUCCESS)
            return false;
        s.push_back('/');
        s.append(comp.ptr ? comp.ptr : "", comp.ptr ? comp.length : 0);
    }
    if (s.empty())
        return false;
    *out = std::move(s);
    return true;
}

size_t dlElemBytes(const DLDataType& dt)
{
    const size_t bits = static_cast<size_t>(dt.bits) * static_cast<size_t>(dt.lanes);
    return (bits + 7) / 8;
}

bool tensorTotalBytes(const DLTensor& t, size_t* out)
{
    if (!out || t.ndim < 0 || (t.ndim > 0 && !t.shape))
        return false;
    size_t elems = 1;
    for (int i = 0; i < t.ndim; ++i)
    {
        if (t.shape[i] < 0)
            return false;
        const size_t extent = static_cast<size_t>(t.shape[i]);
        if (extent != 0 && elems > std::numeric_limits<size_t>::max() / extent)
            return false;
        elems *= extent;
    }
    const size_t eb = dlElemBytes(t.dtype);
    if (eb == 0 && elems != 0)
        return false;
    if (eb != 0 && elems > std::numeric_limits<size_t>::max() / eb)
        return false;
    *out = elems * eb;
    return true;
}

uint64_t loadU64At(const uint8_t* p)
{
    uint64_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

// Decode one read group's rows into `scene`, keeping the payload from the
// highest ordinal per (prim, attr). Handles the OFFICIAL transports the
// in-repo engine now emits (read-layout port) — one tensor per row for ragged
// groups, one stacked tensor for fixed groups — plus the legacy two-tensor
// CSR pair, tolerated for FOREIGN producers only and never where per-row can
// claim the shape first (a 2-row ragged group whose second row tensor is
// itself {kDLUInt,64,1} decodes per-row). Fails closed on anything else.
bool decodeReadGroup(ovstage_instance_t* src,
                     const ovstage_read_group_t& g,
                     const std::string& attr_name,
                     const std::vector<std::string>& group_paths,
                     PulledScene* scene)
{
    const uint32_t logical_count = g.prims.count;
    if (logical_count == 0)
        return true;

    auto record = [&](uint32_t logical, const uint8_t* row, size_t nbytes, bool deleted) -> bool
    {
        const uint64_t prim_row = g.prims.index_map ? static_cast<uint64_t>(g.prims.index_map[logical]) :
                                                      static_cast<uint64_t>(g.prims.offset) + logical;
        if (prim_row >= group_paths.size())
        {
            setBackendError("pull: group prim row out of range for " + attr_name);
            return false;
        }
        const std::string& prim_path = group_paths[static_cast<size_t>(prim_row)];
        // A zero-component path is the USD pseudo-root. Population can expose
        // pseudo-root metadata in a match-all read, but it is not a prim and
        // cannot be re-interned into the renderer's mirror dictionary.
        if (prim_path.empty())
            return true;
        PulledColumn& col = (*scene)[prim_path][attr_name];
        // Point reads may surface one group per ordinal; keep the newest.
        if (col.recorded && col.ordinal > g.ordinal)
            return true;
        col.recorded = true;
        col.ordinal = g.ordinal;
        col.deleted = deleted;
        col.dtype = deleted ? DLDataType{} : g.data.tensors ? g.data.tensors[0].dtype : DLDataType{};
        col.semantic = g.semantic;
        col.is_array = g.is_array;
        if (nbytes == 0)
            col.bytes.clear();
        else
            col.bytes.assign(row, row + nbytes);
        return true;
    };

    if (g.is_delete)
    {
        for (uint32_t logical = 0; logical < logical_count; ++logical)
            if (!record(logical, nullptr, 0, true))
                return false;
        return true;
    }

    if (g.data.cuda_sync.stream || g.data.cuda_sync.wait_event)
    {
        setBackendError("pull: device-resident group for " + attr_name);
        return false;
    }
    if (g.data.index_map && g.data.mask)
    {
        setBackendError("pull: group carries both index_map and mask for " + attr_name);
        return false;
    }
    if (g.data.tensor_count == 0 || !g.data.tensors)
    {
        setBackendError("pull: value group without tensors for " + attr_name);
        return false;
    }

    // Classify transport (decode precedence, read-layout port): per-row wins
    // for ragged groups; the fixed stacked tensor for !is_array; the legacy
    // CSR pair only where neither claims the shape (foreign producers).
    const size_t data_rows = g.data.count ? static_cast<size_t>(g.data.count) : static_cast<size_t>(logical_count);
    const bool is_per_row = g.is_array && g.data.tensor_count == data_rows;
    bool is_csr = false;
    if (!is_per_row && g.data.tensor_count == 2)
    {
        const DLTensor& to = g.data.tensors[1];
        is_csr = to.dtype.code == kDLUInt && to.dtype.bits == 64 && to.dtype.lanes == 1 && to.ndim == 1 && to.shape &&
                 static_cast<size_t>(to.shape[0]) == data_rows + 1;
    }
    if (!is_per_row && !is_csr && (g.is_array || g.data.tensor_count != 1))
    {
        setBackendError("pull: unrecognized group transport for " + attr_name);
        return false;
    }

    const DLTensor& tv = g.data.tensors[0];
    size_t value_total = 0;
    if (!tensorTotalBytes(tv, &value_total))
    {
        setBackendError("pull: malformed value tensor for " + attr_name);
        return false;
    }
    const uint8_t* values = tv.data ? static_cast<const uint8_t*>(tv.data) + tv.byte_offset : nullptr;

    size_t offset_elems = 0;
    const uint8_t* offsets = nullptr;
    size_t value_rows = 0, value_row_bytes = 0;
    if (is_csr)
    {
        const DLTensor& to = g.data.tensors[1];
        size_t offs_total = 0;
        if (!tensorTotalBytes(to, &offs_total) || !to.data)
        {
            setBackendError("pull: malformed CSR offsets for " + attr_name);
            return false;
        }
        offset_elems = offs_total / sizeof(uint64_t);
        offsets = static_cast<const uint8_t*>(to.data) + to.byte_offset;
    }
    else if (!g.is_array)
    {
        // Fixed row stride = total_bytes / rows (NOT one dtype element per
        // prim — a fixed multi-element row like extent is 2 float3 elements).
        value_rows = data_rows ? data_rows : 1;
        if (value_total % value_rows != 0)
        {
            setBackendError("pull: fixed group rows do not divide the payload for " + attr_name);
            return false;
        }
        value_row_bytes = value_total / value_rows;
    }

    for (uint32_t logical = 0; logical < logical_count; ++logical)
    {
        if (g.data.mask)
        {
            const uint64_t word = g.data.mask[logical / 64];
            if ((word & (uint64_t(1) << (logical % 64))) == 0)
                continue;
        }
        const uint32_t data_slot = g.data.index_map ? g.data.index_map[logical] : logical;
        const uint8_t* row = nullptr;
        size_t nbytes = 0;
        if (is_csr)
        {
            if (static_cast<size_t>(data_slot) + 1 >= offset_elems)
            {
                setBackendError("pull: CSR slot out of range for " + attr_name);
                return false;
            }
            const uint64_t b0 = loadU64At(offsets + static_cast<size_t>(data_slot) * 8);
            const uint64_t b1 = loadU64At(offsets + (static_cast<size_t>(data_slot) + 1) * 8);
            if (b1 < b0 || b1 > value_total)
            {
                setBackendError("pull: CSR offsets out of range for " + attr_name);
                return false;
            }
            row = values ? values + b0 : nullptr;
            nbytes = static_cast<size_t>(b1 - b0);
        }
        else if (g.is_array)
        {
            if (data_slot >= g.data.tensor_count)
            {
                setBackendError("pull: per-row tensor slot out of range for " + attr_name);
                return false;
            }
            const DLTensor& rt = g.data.tensors[data_slot];
            if (!tensorTotalBytes(rt, &nbytes))
            {
                setBackendError("pull: malformed row tensor for " + attr_name);
                return false;
            }
            row = rt.data ? static_cast<const uint8_t*>(rt.data) + rt.byte_offset : nullptr;
        }
        else
        {
            if (data_slot >= value_rows)
            {
                setBackendError("pull: fixed row slot out of range for " + attr_name);
                return false;
            }
            row = values ? values + static_cast<size_t>(data_slot) * value_row_bytes : nullptr;
            nbytes = value_row_bytes;
        }
        if (nbytes != 0 && !row)
        {
            setBackendError("pull: NULL row payload for " + attr_name);
            return false;
        }
        if (!record(logical, row, nbytes, false))
            return false;
    }
    (void)src;
    return true;
}

bool isLocalTransformColumn(const std::string& attribute);
bool isTransformColumn(const std::string& attribute);
const PulledColumn* pickLocalAuthority(const std::map<std::string, PulledColumn>& columns);
const PulledColumn* pickWorldAuthority(const std::map<std::string, PulledColumn>& columns);
const PulledColumn* pickSourceWorld(const std::map<std::string, PulledColumn>& columns);

bool hasPathPrefix(const std::string& path, const std::string& prefix)
{
    if (prefix == "/")
        return !path.empty() && path.front() == '/';
    return path == prefix ||
           (path.size() > prefix.size() && path.compare(0, prefix.size(), prefix) == 0 && path[prefix.size()] == '/');
}

std::string replacePathPrefix(const std::string& path, const std::string& prefix, const std::string& replacement)
{
    if (path == prefix)
        return replacement;
    return replacement + path.substr(prefix.size());
}

std::string getParentPath(const std::string& path)
{
    const size_t slash = path.rfind('/');
    return slash == std::string::npos || slash == 0 ? "/" : path.substr(0, slash);
}

const std::vector<std::string>* findPrototypeInstanceRoots(const SceneGraphInstanceIndex& instances,
                                                           const std::string& path)
{
    std::string ancestor = path;
    for (;;)
    {
        const auto prototype = instances.instanceRootsByPrototype.find(ancestor);
        if (prototype != instances.instanceRootsByPrototype.end())
            return &prototype->second;
        if (ancestor == "/")
            return nullptr;
        ancestor = getParentPath(ancestor);
    }
}

void collectAffectedInstanceRoots(const SceneGraphInstanceIndex& instances,
                                  const std::string& changedPath,
                                  std::set<std::string>* affectedRoots)
{
    // A changed ancestor affects all instance roots in its subtree. The ordered
    // root set makes this proportional to the number of matching instances.
    for (auto root = instances.instanceRoots.lower_bound(changedPath);
         root != instances.instanceRoots.end() && hasPathPrefix(*root, changedPath); ++root)
    {
        affectedRoots->insert(*root);
    }

    // A change inside an instance affects that instance and any enclosing
    // instance. This also handles overlapping roots without an all-roots scan.
    std::string ancestor = changedPath;
    for (;;)
    {
        if (instances.instanceRoots.count(ancestor) != 0)
            affectedRoots->insert(ancestor);
        if (ancestor == "/")
            break;
        ancestor = getParentPath(ancestor);
    }
}

void setIdentityMatrix(std::array<double, 16>* matrix)
{
    *matrix = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
}

void multiplyMatrices(const std::array<double, 16>& left,
                      const std::array<double, 16>& right,
                      std::array<double, 16>* output)
{
    std::array<double, 16> result{};
    for (size_t row = 0; row < 4; ++row)
    {
        for (size_t column = 0; column < 4; ++column)
        {
            for (size_t inner = 0; inner < 4; ++inner)
                result[row * 4 + column] += left[row * 4 + inner] * right[inner * 4 + column];
        }
    }
    *output = result;
}

bool decodeMatrixColumn(const PulledColumn* column, const std::string& path, std::array<double, 16>* matrix)
{
    if (!column || column->deleted)
    {
        setIdentityMatrix(matrix);
        return true;
    }
    if (column->is_array || column->semantic != OVSTAGE_SEMANTIC_MATRIX || column->dtype.code != kDLFloat ||
        column->dtype.bits != 64 || column->dtype.lanes != 16 || column->bytes.size() != sizeof(*matrix))
    {
        setBackendError("instance expansion: transform for '" + path + "' is not a 4x4 double matrix");
        return false;
    }
    std::memcpy(matrix->data(), column->bytes.data(), sizeof(*matrix));
    if (!std::all_of(matrix->begin(), matrix->end(), [](double value) { return std::isfinite(value); }))
    {
        setBackendError("instance expansion: transform for '" + path + "' contains a non-finite value");
        return false;
    }
    return true;
}

bool readsResetXformStack(const std::map<std::string, PulledColumn>& columns,
                          const std::string& path,
                          bool* reset,
                          ovstage_ordinal_t* ordinal)
{
    *reset = false;
    *ordinal = 0;
    for (const char* name : g_kResetXformStackColumns)
    {
        const auto column = columns.find(name);
        if (column == columns.end() || column->second.deleted)
            continue;
        if (column->second.bytes.size() != 1 || column->second.bytes[0] > 1)
        {
            setBackendError("instance expansion: resetXformStack for '" + path + "' is not a boolean");
            return false;
        }
        if (column->second.ordinal >= *ordinal)
        {
            *reset = column->second.bytes[0] != 0;
            *ordinal = column->second.ordinal;
        }
    }
    return true;
}

struct ComposedMatrix
{
    std::array<double, 16> value{};
    ovstage_ordinal_t newestTransformOrdinal = 0;
};

bool composePulledWorld(const PulledScene& scene,
                        const std::string& path,
                        std::map<std::string, ComposedMatrix>* memo,
                        ComposedMatrix* output)
{
    const auto cached = memo->find(path);
    if (cached != memo->end())
    {
        *output = cached->second;
        return true;
    }

    ComposedMatrix parent;
    setIdentityMatrix(&parent.value);
    const std::string parentPath = getParentPath(path);
    if (parentPath != "/" && !composePulledWorld(scene, parentPath, memo, &parent))
        return false;

    const auto prim = scene.find(path);
    if (prim == scene.end())
    {
        *output = parent;
        memo->emplace(path, *output);
        return true;
    }

    const PulledColumn* local = pickLocalAuthority(prim->second);
    const PulledColumn* world = pickSourceWorld(prim->second);
    bool reset = false;
    ovstage_ordinal_t resetOrdinal = 0;
    if (!readsResetXformStack(prim->second, path, &reset, &resetOrdinal))
        return false;
    const ovstage_ordinal_t localOrdinal = local ? local->ordinal : 0;
    const ovstage_ordinal_t inheritedOrdinal = reset ? 0 : parent.newestTransformOrdinal;
    const ovstage_ordinal_t newestInput = std::max({ inheritedOrdinal, localOrdinal, resetOrdinal });

    output->newestTransformOrdinal = std::max(newestInput, world ? world->ordinal : 0);
    if (world && world->ordinal >= newestInput)
    {
        if (!decodeMatrixColumn(world, path, &output->value))
            return false;
    }
    else
    {
        std::array<double, 16> localMatrix;
        if (!decodeMatrixColumn(local, path, &localMatrix))
            return false;
        if (reset)
            output->value = localMatrix;
        else
            multiplyMatrices(localMatrix, parent.value, &output->value);
    }
    memo->emplace(path, *output);
    return true;
}

bool collectLightPaths(const PulledScene& scene, path_dictionary_instance_t* dictionary, std::set<std::string>* lightPaths)
{
    lightPaths->clear();
    for (const auto& prim : scene)
    {
        const auto type = prim.second.find("usd-prim-type");
        if (type == prim.second.end() || type->second.deleted)
            continue;
        if (type->second.is_array || type->second.bytes.size() != sizeof(uint64_t))
            return false;
        std::string typeName;
        if (!getDictionaryTokenString(
                dictionary, static_cast<ovx_token_t>(loadU64At(type->second.bytes.data())), &typeName))
            return false;
        constexpr const char* suffix = "Light";
        constexpr size_t suffixLength = 5;
        if (typeName.size() >= suffixLength && typeName.compare(typeName.size() - suffixLength, suffixLength, suffix) == 0)
        {
            lightPaths->insert(prim.first);
        }
    }
    return true;
}

bool changesAffectPaths(const std::map<std::string, std::vector<std::string>>& changes,
                        const std::set<std::string>& targets)
{
    for (const auto& change : changes)
    {
        for (const std::string& target : targets)
        {
            if (hasPathPrefix(target, change.first))
                return true;
        }
    }
    return false;
}

bool pathsAffectTargets(const std::vector<std::string>& paths, const std::set<std::string>& targets)
{
    for (const std::string& path : paths)
    {
        for (const std::string& target : targets)
        {
            if (hasPathPrefix(target, path))
                return true;
        }
    }
    return false;
}

bool composeResidentMeshWorlds(ovgl_renderer_t* renderer, const PulledScene& scene, std::vector<double>* worldMatrices)
{
    const int32_t meshCount = ovgl_get_mesh_count(renderer);
    if (meshCount < 0)
        return false;
    worldMatrices->clear();
    worldMatrices->reserve(static_cast<size_t>(meshCount) * 16);
    std::map<std::string, ComposedMatrix> memo;
    for (int32_t index = 0; index < meshCount; ++index)
    {
        ovgl_mesh_view_t mesh{};
        if (ovgl_get_mesh_view(renderer, index, &mesh).status != 0 || !mesh.path || !mesh.path[0])
            return false;
        ComposedMatrix world;
        if (!composePulledWorld(scene, mesh.path, &memo, &world))
            return false;
        worldMatrices->insert(worldMatrices->end(), world.value.begin(), world.value.end());
    }
    return true;
}

bool invertAffineMatrix(const std::array<double, 16>& matrix, std::array<double, 16>* output)
{
    const double a00 = matrix[0], a01 = matrix[1], a02 = matrix[2];
    const double a10 = matrix[4], a11 = matrix[5], a12 = matrix[6];
    const double a20 = matrix[8], a21 = matrix[9], a22 = matrix[10];
    const double c00 = a11 * a22 - a12 * a21;
    const double c01 = -(a10 * a22 - a12 * a20);
    const double c02 = a10 * a21 - a11 * a20;
    const double c10 = -(a01 * a22 - a02 * a21);
    const double c11 = a00 * a22 - a02 * a20;
    const double c12 = -(a00 * a21 - a01 * a20);
    const double c20 = a01 * a12 - a02 * a11;
    const double c21 = -(a00 * a12 - a02 * a10);
    const double c22 = a00 * a11 - a01 * a10;
    const double determinant = a00 * c00 + a01 * c01 + a02 * c02;
    if (std::fabs(determinant) < 1e-12)
        return false;
    const double inverseDeterminant = 1.0 / determinant;
    *output = {
        c00 * inverseDeterminant,
        c10 * inverseDeterminant,
        c20 * inverseDeterminant,
        0,
        c01 * inverseDeterminant,
        c11 * inverseDeterminant,
        c21 * inverseDeterminant,
        0,
        c02 * inverseDeterminant,
        c12 * inverseDeterminant,
        c22 * inverseDeterminant,
        0,
        0,
        0,
        0,
        1,
    };
    const double tx = matrix[12], ty = matrix[13], tz = matrix[14];
    (*output)[12] = -(tx * (*output)[0] + ty * (*output)[4] + tz * (*output)[8]);
    (*output)[13] = -(tx * (*output)[1] + ty * (*output)[5] + tz * (*output)[9]);
    (*output)[14] = -(tx * (*output)[2] + ty * (*output)[6] + tz * (*output)[10]);
    return true;
}

bool composeJournalMeshWorlds(OvglBackend* backend,
                              const PulledScene& scene,
                              const TransformJournal& journal,
                              std::vector<double>* worldMatrices)
{
    const int32_t meshCount = ovgl_get_mesh_count(backend->renderer);
    if (meshCount < 0 || journal.paths.size() != journal.worlds.size())
        return false;
    const bool sameRoutes = backend->transform_journal_generation == journal.generation &&
                            backend->transform_journal_paths == journal.paths &&
                            backend->transform_journal_routes.size() == static_cast<size_t>(meshCount);
    if (!sameRoutes)
    {
        if (backend->pulled_transform_dirty)
            return false;
        std::set<std::string> uniquePaths(journal.paths.begin(), journal.paths.end());
        if (uniquePaths.size() != journal.paths.size())
            return false;
        std::vector<JournalMeshRoute> routes(static_cast<size_t>(meshCount));
        std::map<std::string, ComposedMatrix> memo;
        for (int32_t meshIndex = 0; meshIndex < meshCount; ++meshIndex)
        {
            ovgl_mesh_view_t mesh{};
            if (ovgl_get_mesh_view(backend->renderer, meshIndex, &mesh).status != 0 || !mesh.path || !mesh.path[0])
                return false;
            size_t driverIndex = std::numeric_limits<size_t>::max();
            size_t driverLength = 0;
            for (size_t candidate = 0; candidate < journal.paths.size(); ++candidate)
            {
                if (journal.paths[candidate].size() >= driverLength && hasPathPrefix(mesh.path, journal.paths[candidate]))
                {
                    driverIndex = candidate;
                    driverLength = journal.paths[candidate].size();
                }
            }
            JournalMeshRoute& route = routes[static_cast<size_t>(meshIndex)];
            route.driverIndex = driverIndex;
            std::copy(std::begin(mesh.world_xform), std::end(mesh.world_xform), route.relativeOrStatic.begin());
            if (driverIndex != std::numeric_limits<size_t>::max())
            {
                ComposedMatrix oldDriver;
                std::array<double, 16> inverseDriver;
                if (!composePulledWorld(scene, journal.paths[driverIndex], &memo, &oldDriver) ||
                    !invertAffineMatrix(oldDriver.value, &inverseDriver))
                    return false;
                multiplyMatrices(route.relativeOrStatic, inverseDriver, &route.relativeOrStatic);
            }
        }
        backend->transform_journal_generation = journal.generation;
        backend->transform_journal_paths = journal.paths;
        backend->transform_journal_routes = std::move(routes);
    }

    worldMatrices->clear();
    worldMatrices->reserve(static_cast<size_t>(meshCount) * 16);
    for (const JournalMeshRoute& route : backend->transform_journal_routes)
    {
        std::array<double, 16> world = route.relativeOrStatic;
        if (route.driverIndex != std::numeric_limits<size_t>::max())
            multiplyMatrices(route.relativeOrStatic, journal.worlds[route.driverIndex], &world);
        worldMatrices->insert(worldMatrices->end(), world.begin(), world.end());
    }
    return true;
}

void storeExpandedWorld(std::map<std::string, PulledColumn>* columns, const ComposedMatrix& world)
{
    PulledColumn column;
    column.bytes.resize(sizeof(world.value));
    std::memcpy(column.bytes.data(), world.value.data(), sizeof(world.value));
    column.dtype = DLDataType{ kDLFloat, 64, 16 };
    column.semantic = OVSTAGE_SEMANTIC_MATRIX;
    column.ordinal = world.newestTransformOrdinal;
    column.recorded = true;
    (*columns)["worldMatrix"] = std::move(column);
}

bool internPath(path_dictionary_instance_t* dict, const std::string& path, ovx_primpath_t* out)
{
    if (!dict || !dict->vtable || !out)
        return false;
    const ovx_string_t value = toOvx(path);
    *out = OVX_INVALID_PRIMPATH;
    return dict->vtable->create_paths_from_strings(dict->context, &value, 1, out).status == OVX_API_SUCCESS &&
           *out != OVX_INVALID_PRIMPATH;
}

bool resolveOwnedPathList(path_dictionary_instance_t* dict,
                          ovx_primpath_list_t list,
                          const std::string& operation,
                          std::vector<std::string>* out)
{
    out->clear();
    if (!dict || !dict->vtable || list == OVX_INVALID_PRIMPATH_LIST)
    {
        setBackendError(operation + ": invalid path list");
        return false;
    }
    size_t expected = 0;
    const bool counted =
        dict->vtable->get_num_paths_from_path_list(dict->context, list, &expected).status == OVX_API_SUCCESS;
    if (counted)
        *out = isaacsim::ovgl_viewport::debug::details::resolvePrimPaths(dict, list);
    const bool released = path_dictionary_release_path_list_reference(dict, list).status == OVX_API_SUCCESS;
    if (!counted || out->size() != expected || !released)
    {
        setBackendError(operation + ": failed to resolve path list");
        out->clear();
        return false;
    }
    return true;
}

bool remapPrototypePathIds(path_dictionary_instance_t* dict,
                           const std::string& prototype_root,
                           const std::string& instance_root,
                           PulledColumn* column)
{
    const bool relationship = column->semantic == OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID;
    const bool connection = column->semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID;
    if (!relationship && !connection)
        return true;
    const size_t tuple_bytes = connection ? 2 * sizeof(uint64_t) : sizeof(uint64_t);
    if (column->bytes.size() % tuple_bytes != 0)
    {
        setBackendError("instance expansion: path-id payload has an invalid tuple extent");
        return false;
    }

    const size_t id_count = column->bytes.size() / sizeof(uint64_t);
    for (size_t index = 0; index < id_count; index += connection ? 2 : 1)
    {
        const uint64_t id = loadU64At(column->bytes.data() + index * sizeof(uint64_t));
        if (id == 0)
            continue;
        std::string path;
        if (!getDictionaryPathString(dict, static_cast<ovx_primpath_t>(id), &path))
        {
            setBackendError("instance expansion: failed to resolve a path id");
            return false;
        }
        if (!hasPathPrefix(path, prototype_root))
            continue;

        const std::string remapped = replacePathPrefix(path, prototype_root, instance_root);
        ovx_primpath_t remapped_id = OVX_INVALID_PRIMPATH;
        if (!internPath(dict, remapped, &remapped_id))
        {
            setBackendError("instance expansion: failed to intern path '" + remapped + "'");
            return false;
        }
        const uint64_t stored = static_cast<uint64_t>(remapped_id);
        std::memcpy(column->bytes.data() + index * sizeof(uint64_t), &stored, sizeof(stored));
    }
    return true;
}

// OVStage publishes shared topology below /__Prototype_* and instance roots
// separately. Some population implementations also emit partial flattened
// instance-proxy rows. Build the complete one-level compatibility view expected
// by OVGL through the public instancing API: prototype values fill only missing
// columns at visible paths, while visible transforms and authored overrides
// remain authoritative. Nested prototype dependencies require a stage-native
// materialization contract and are outside this adapter's compatibility scope.
bool expandSceneGraphInstances(ovstage_instance_t* stage,
                               PulledScene* scene,
                               SceneGraphInstanceIndex* retainedInstances = nullptr,
                               const std::set<std::string>* instanceRootFilter = nullptr)
{
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
    {
        setBackendError("instance expansion: stage has no path dictionary");
        return false;
    }

    ovx_primpath_list_t prototype_list = OVX_INVALID_PRIMPATH_LIST;
    if (ovstage_instancing_get_prototype_roots(stage, &prototype_list) != OVSTAGE_OK)
    {
        setBackendError("instance expansion: get_prototype_roots failed");
        return false;
    }
    std::vector<std::string> prototype_roots;
    if (!resolveOwnedPathList(dict, prototype_list, "instance expansion prototypes", &prototype_roots))
        return false;
    SceneGraphInstanceIndex instances;
    size_t expanded_root_count = 0;
    size_t expanded_column_count = 0;
    size_t normalized_type_count = 0;
    for (const std::string& prototype_root : prototype_roots)
    {
        ovx_primpath_t prototype_path = OVX_INVALID_PRIMPATH;
        if (!internPath(dict, prototype_root, &prototype_path))
        {
            setBackendError("instance expansion: failed to intern prototype root '" + prototype_root + "'");
            return false;
        }

        ovx_primpath_list_t instance_list = OVX_INVALID_PRIMPATH_LIST;
        if (ovstage_instancing_get_instance_roots(stage, prototype_path, &instance_list) != OVSTAGE_OK)
        {
            setBackendError("instance expansion: get_instance_roots failed for '" + prototype_root + "'");
            return false;
        }
        std::vector<std::string> instance_roots;
        if (!resolveOwnedPathList(dict, instance_list, "instance expansion roots for " + prototype_root, &instance_roots))
            return false;
        if (instance_roots.empty())
            continue;
        instances.instanceRootsByPrototype.emplace(prototype_root, instance_roots);
        instances.instanceRoots.insert(instance_roots.begin(), instance_roots.end());
        expanded_root_count += instance_roots.size();
    }
    if (retainedInstances)
        *retainedInstances = instances;

    for (const std::string& prototype_root : prototype_roots)
    {
        if (instances.instanceRootsByPrototype.count(prototype_root) == 0)
            continue;
        const std::vector<std::string>& instance_roots = instances.instanceRootsByPrototype.at(prototype_root);
        std::vector<std::pair<std::string, std::map<std::string, PulledColumn>>> prototype_prims;
        for (auto prim = scene->lower_bound(prototype_root);
             prim != scene->end() && hasPathPrefix(prim->first, prototype_root); ++prim)
        {
            prototype_prims.push_back(*prim);
        }

        std::map<std::string, ComposedMatrix> worldMemo;
        for (const std::string& instance_root : instance_roots)
        {
            if (instanceRootFilter && instanceRootFilter->count(instance_root) == 0)
                continue;
            for (const auto& prototype_prim : prototype_prims)
            {
                const std::string instance_path = replacePathPrefix(prototype_prim.first, prototype_root, instance_root);
                auto& instance_columns = (*scene)[instance_path];
                for (const auto& attribute : prototype_prim.second)
                {
                    bool replaces_instance_type = false;
                    if (attribute.first == "usd-prim-type" && attribute.second.bytes.size() == sizeof(uint64_t))
                    {
                        std::string prototype_type;
                        if (!getDictionaryTokenString(
                                dict, static_cast<ovx_token_t>(loadU64At(attribute.second.bytes.data())), &prototype_type))
                        {
                            setBackendError("instance expansion: failed to resolve prototype type");
                            return false;
                        }
                        const std::string suffix = "Instance";
                        replaces_instance_type =
                            prototype_type.size() < suffix.size() ||
                            prototype_type.compare(prototype_type.size() - suffix.size(), suffix.size(), suffix) != 0;
                    }
                    const bool is_prototype_root = prototype_prim.first == prototype_root;
                    const bool copies_prototype_local = !is_prototype_root && isLocalTransformColumn(attribute.first);
                    if (attribute.second.deleted || (isTransformColumn(attribute.first) && !copies_prototype_local) ||
                        attribute.first == "_protoPath" || attribute.first == "_isSceneGraphInstancingRoot" ||
                        (instance_columns.count(attribute.first) != 0 && !replaces_instance_type))
                        continue;

                    PulledColumn expanded = attribute.second;
                    if (!remapPrototypePathIds(dict, prototype_root, instance_root, &expanded))
                        return false;
                    if (replaces_instance_type)
                        instance_columns[attribute.first] = std::move(expanded);
                    else
                        instance_columns.emplace(attribute.first, std::move(expanded));
                    ++expanded_column_count;
                    if (replaces_instance_type)
                        ++normalized_type_count;
                }
                if (prototype_prim.first != prototype_root)
                {
                    ComposedMatrix expandedWorld;
                    if (!composePulledWorld(*scene, instance_path, &worldMemo, &expandedWorld))
                        return false;
                    storeExpandedWorld(&instance_columns, expandedWorld);
                }
            }
        }
    }
    if (isProfilingEnabled() && !prototype_roots.empty())
    {
        size_t mesh_count = 0;
        size_t mesh_with_points_count = 0;
        for (const auto& prim : *scene)
        {
            const auto type = prim.second.find("usd-prim-type");
            if (type == prim.second.end() || type->second.bytes.size() != sizeof(uint64_t))
                continue;
            std::string type_name;
            if (!getDictionaryTokenString(
                    dict, static_cast<ovx_token_t>(loadU64At(type->second.bytes.data())), &type_name) ||
                type_name != "Mesh")
                continue;
            ++mesh_count;
            if (prim.second.count("points") != 0)
                ++mesh_with_points_count;
        }
        std::fprintf(stderr,
                     "OVGLBEPROF instances prototypes=%zu roots=%zu columns=%zu types=%zu "
                     "meshes=%zu meshes_with_points=%zu\n",
                     prototype_roots.size(), expanded_root_count, expanded_column_count, normalized_type_count,
                     mesh_count, mesh_with_points_count);
    }
    return true;
}

bool addRequiredPullAttributes(path_dictionary_instance_t* dictionary, std::vector<ovx_token_t>* attributes)
{
    // Query discovery intentionally hides reserved metadata and can omit
    // columns created after USD population. Request the public transform
    // spellings explicitly so Foundation camera edits and simulation writes
    // participate in both full snapshots and ordinal-window reads.
    for (const char* required :
         { "usd-prim-type", "usd-schemas", "worldMatrix", "localMatrix", "omni:xform", "omni:resetXformStack",
           "xformOpOrder", "omni:fabric:localMatrix", "omni:fabric:worldMatrix", "omni:fabric:resetXformStack" })
    {
        const ovx_string_t name{ required, std::strlen(required) };
        ovx_token_t token = OVX_INVALID_TOKEN;
        if (path_dictionary_create_tokens_from_strings(dictionary, &name, 1, &token).status != OVX_API_SUCCESS ||
            token == OVX_INVALID_TOKEN)
            return false;
        if (std::find(attributes->begin(), attributes->end(), token) == attributes->end())
            attributes->push_back(token);
    }
    return true;
}

bool readQueryAttributes(ovstage_instance_t* stage,
                         path_dictionary_instance_t* dictionary,
                         ovstage_query_handle_t query,
                         const std::vector<ovx_token_t>& attributes,
                         const ovstage_ordinal_range_t& range,
                         PulledScene* output)
{
    if (attributes.empty())
        return true;
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t operation =
        ovstage_read_attributes(stage, query, attributes.data(), attributes.size(), range, &read);
    if (operation.status != OVSTAGE_OK || read == OVSTAGE_INVALID_READ_HANDLE)
    {
        setBackendError("pull: read_attributes enqueue failed on the attached stage");
        return false;
    }
    bool succeeded = true;
    std::unordered_map<ovx_primpath_list_t, std::vector<std::string>> resolved;
    for (;;)
    {
        ovstage_read_group_t group{};
        const ovstage_api_status_t fetched = ovstage_fetch_read_next(stage, read, OVSTAGE_TIMEOUT_INFINITE, &group);
        if (fetched == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        if (fetched != OVSTAGE_OK)
        {
            setBackendError("pull: fetch_read_next failed on the attached stage");
            succeeded = false;
            break;
        }

        std::string attributeName;
        const std::vector<std::string>* groupPaths = nullptr;
        if (group.prims.list != OVX_INVALID_PRIMPATH_LIST)
        {
            auto existing = resolved.find(group.prims.list);
            if (existing == resolved.end())
                existing = resolved
                               .emplace(group.prims.list, isaacsim::ovgl_viewport::debug::details::resolvePrimPaths(
                                                              dictionary, group.prims.list))
                               .first;
            groupPaths = &existing->second;
        }
        if (!groupPaths || groupPaths->empty() || !getDictionaryTokenString(dictionary, group.attribute, &attributeName))
        {
            setBackendError("pull: unresolvable group list or attribute token");
            succeeded = false;
        }
        else if (!decodeReadGroup(stage, group, attributeName, *groupPaths, output))
        {
            succeeded = false;
        }
        if (ovstage_release_group(stage, &group) != OVSTAGE_OK)
            succeeded = false;
        if (!succeeded)
            break;
    }
    if (!instComplete(stage, operation, "pull read"))
        succeeded = false;
    if (!instComplete(stage, ovstage_release_read(stage, read), "pull read release"))
        succeeded = false;
    return succeeded;
}

bool readTransformJournal(ovstage_instance_t* stage,
                          path_dictionary_instance_t* dictionary,
                          ovstage_ordinal_t ordinal,
                          TransformJournal* journal)
{
    using namespace isaacsim::common::ovstage;
    const ovx_string_t path{ g_kTransformJournalPrimPath, std::strlen(g_kTransformJournalPrimPath) };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    if (path_dictionary_create_path_list_from_strings(dictionary, &path, 1, &pathList).status != OVX_API_SUCCESS ||
        pathList == OVX_INVALID_PRIMPATH_LIST)
        return false;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(stage, pathList, &query) != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        path_dictionary_release_path_list_reference(dictionary, pathList);
        return false;
    }
    const ovx_string_t attribute{ g_kTransformJournalAttribute, std::strlen(g_kTransformJournalAttribute) };
    ovx_token_t token = OVX_INVALID_TOKEN;
    const bool tokenCreated =
        path_dictionary_create_tokens_from_strings(dictionary, &attribute, 1, &token).status == OVX_API_SUCCESS &&
        token != OVX_INVALID_TOKEN;
    PulledScene pulled;
    ovstage_ordinal_range_t range{};
    range.has_start_ordinal = true;
    range.start_ordinal = ordinal;
    range.end_ordinal = ordinal;
    const bool read = tokenCreated && readQueryAttributes(stage, dictionary, query, { token }, range, &pulled);
    const bool queryReleased = instComplete(stage, ovstage_release_query(stage, query), "journal query release");
    const bool pathsReleased =
        path_dictionary_release_path_list_reference(dictionary, pathList).status == OVX_API_SUCCESS;
    if (!read || !queryReleased || !pathsReleased)
        return false;
    const auto prim = pulled.find(g_kTransformJournalPrimPath);
    if (prim == pulled.end())
        return false;
    const auto column = prim->second.find(g_kTransformJournalAttribute);
    if (column == prim->second.end() || column->second.deleted || !column->second.is_array ||
        column->second.semantic != OVSTAGE_SEMANTIC_NONE || column->second.dtype.code != kDLUInt ||
        column->second.dtype.bits != 8 || column->second.dtype.lanes != 1 ||
        column->second.bytes.size() < sizeof(TransformJournalHeader))
        return false;

    TransformJournalHeader header{};
    std::memcpy(&header, column->second.bytes.data(), sizeof(header));
    if (header.magic != g_kTransformJournalMagic || header.version != g_kTransformJournalVersion ||
        header.ordinal != ordinal || header.entryCount == 0 ||
        header.entryCount > (std::numeric_limits<size_t>::max() - sizeof(header)) / sizeof(TransformJournalEntry))
        return false;
    const size_t entryCount = static_cast<size_t>(header.entryCount);
    if (column->second.bytes.size() != sizeof(header) + entryCount * sizeof(TransformJournalEntry))
        return false;

    journal->generation = header.generation;
    if (header.baseOrdinal >= header.ordinal)
        return false;
    journal->baseOrdinal = static_cast<ovstage_ordinal_t>(header.baseOrdinal);
    journal->ordinal = static_cast<ovstage_ordinal_t>(header.ordinal);
    journal->paths.clear();
    journal->worlds.clear();
    journal->paths.reserve(entryCount);
    journal->worlds.reserve(entryCount);
    const uint8_t* entries = column->second.bytes.data() + sizeof(header);
    for (size_t index = 0; index < entryCount; ++index)
    {
        TransformJournalEntry entry{};
        std::memcpy(&entry, entries + index * sizeof(entry), sizeof(entry));
        std::string entryPath;
        if (entry.primPath == OVX_INVALID_PRIMPATH ||
            !getDictionaryPathString(dictionary, static_cast<ovx_primpath_t>(entry.primPath), &entryPath) ||
            !std::all_of(std::begin(entry.worldMatrix), std::end(entry.worldMatrix),
                         [](double value) { return std::isfinite(value); }))
            return false;
        journal->paths.push_back(std::move(entryPath));
        std::array<double, 16> world{};
        std::copy(std::begin(entry.worldMatrix), std::end(entry.worldMatrix), world.begin());
        journal->worlds.push_back(world);
    }
    return true;
}

void applyPulledChanges(PulledScene* scene, const PulledScene& changes)
{
    for (const auto& prim : changes)
    {
        for (const auto& column : prim.second)
        {
            if (column.second.deleted)
            {
                if (auto existingPrim = scene->find(prim.first); existingPrim != scene->end())
                {
                    existingPrim->second.erase(column.first);
                    if (existingPrim->second.empty())
                        scene->erase(existingPrim);
                }
            }
            else
            {
                (*scene)[prim.first][column.first] = column.second;
            }
        }
    }
}

void buildSnapshotDiff(const PulledScene& current,
                       const PulledScene& previous,
                       ovstage_ordinal_t ordinal,
                       PulledScene* changes)
{
    changes->clear();
    for (const auto& currentPrim : current)
    {
        const auto previousPrim = previous.find(currentPrim.first);
        for (const auto& currentColumn : currentPrim.second)
        {
            bool changed = true;
            if (previousPrim != previous.end())
            {
                const auto previousColumn = previousPrim->second.find(currentColumn.first);
                if (previousColumn != previousPrim->second.end())
                {
                    const PulledColumn& lhs = currentColumn.second;
                    const PulledColumn& rhs = previousColumn->second;
                    changed = lhs.bytes != rhs.bytes || lhs.dtype.code != rhs.dtype.code ||
                              lhs.dtype.bits != rhs.dtype.bits || lhs.dtype.lanes != rhs.dtype.lanes ||
                              lhs.semantic != rhs.semantic || lhs.is_array != rhs.is_array || lhs.deleted != rhs.deleted;
                }
            }
            if (changed)
                (*changes)[currentPrim.first][currentColumn.first] = currentColumn.second;
        }
    }
    for (const auto& previousPrim : previous)
    {
        const auto currentPrim = current.find(previousPrim.first);
        for (const auto& previousColumn : previousPrim.second)
        {
            if (currentPrim != current.end() && currentPrim->second.count(previousColumn.first) != 0)
                continue;
            PulledColumn tombstone = previousColumn.second;
            tombstone.bytes.clear();
            tombstone.ordinal = ordinal;
            tombstone.deleted = true;
            tombstone.recorded = true;
            (*changes)[previousPrim.first][previousColumn.first] = std::move(tombstone);
        }
    }
}

// Pull the full committed state <= `ordinal` through the public vtable. A copy
// of the unexpanded source snapshot is optional and is retained only by the
// attached lane for subsequent ordinal-window reads.
bool pullScene(ovstage_instance_t* ext,
               ovstage_ordinal_t ordinal,
               PulledScene* out,
               PulledScene* sourceOut = nullptr,
               std::set<std::string>* sourceAttributes = nullptr,
               SceneGraphInstanceIndex* sceneGraphInstances = nullptr)
{
    out->clear();
    path_dictionary_instance_t* ext_dict = ovstage_get_path_dictionary(ext);
    if (!ext_dict)
    {
        setBackendError("pull: attached stage has no path dictionary");
        return false;
    }

    // Match-all query (empty filter), discovering every stored attribute.
    ovstage_filter_t filter{};
    filter.predicates = nullptr;
    filter.count = 0;
    ovstage_query_handle_t qh = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_enqueue_result_t qe = ovstage_query(ext, &filter, nullptr, 0, &qh);
    if (qe.status != OVSTAGE_OK || qh == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        setBackendError("pull: match-all query enqueue failed on the attached stage");
        return false;
    }
    ovstage_query_result_t qres{};
    const ovstage_api_status_t fq = ovstage_fetch_query_result(ext, qh, OVSTAGE_TIMEOUT_INFINITE, &qres);
    const bool queryCompleted = instComplete(ext, qe, "pull query");
    if (!queryCompleted)
    {
        if (fq == OVSTAGE_OK)
            (void)ovstage_release_query_result(ext, &qres);
        (void)instComplete(ext, ovstage_release_query(ext, qh), "pull query release");
        return false;
    }
    if (fq != OVSTAGE_OK)
    {
        setBackendError("pull: fetch_query_result failed on the attached stage");
        (void)instComplete(ext, ovstage_release_query(ext, qh), "pull query release");
        return false;
    }
    std::vector<ovx_token_t> attrs;
    if (qres.attribute_count > 0)
        attrs.assign(qres.attributes, qres.attributes + qres.attribute_count);
    const size_t total_prims = qres.total_prim_count;
    if (ovstage_release_query_result(ext, &qres) != OVSTAGE_OK)
    {
        setBackendError("pull: release_query_result failed on the attached stage");
        (void)instComplete(ext, ovstage_release_query(ext, qh), "pull query release");
        return false;
    }

    /* Official discovery hides the reserved metadata columns (query-read
     * port, F-QR-2) but they remain explicitly readable — and the mirror
     * NEEDS them: usd-prim-type drives typed-prim handling (RenderProduct/
     * Camera derivation) and usd-schemas the applied-API decisions. Request
     * them by name on top of the discovered set. */
    if (!addRequiredPullAttributes(ext_dict, &attrs))
    {
        setBackendError("pull: unable to create reserved OVStage attribute tokens");
        (void)instComplete(ext, ovstage_release_query(ext, qh), "pull query release");
        return false;
    }

    bool ok = true;
    if (total_prims > 0 && !attrs.empty())
    {
        ovstage_ordinal_range_t range{};
        range.has_start_ordinal = false;
        range.end_ordinal = ordinal;
        ok = readQueryAttributes(ext, ext_dict, qh, attrs, range, out);
    }
    if (!instComplete(ext, ovstage_release_query(ext, qh), "pull query release"))
        ok = false;
    if (!ok)
        return false;
    if (sourceAttributes)
    {
        sourceAttributes->clear();
        for (const auto& prim : *out)
            for (const auto& column : prim.second)
                sourceAttributes->insert(column.first);
    }
    if (sourceOut)
        *sourceOut = *out;
    return expandSceneGraphInstances(ext, out, sceneGraphInstances);
}

enum class PublicPullStatus
{
    eSuccess,
    eTopologyChanged,
    eError,
};

PublicPullStatus pullPublicChanges(OvglBackend* backend,
                                   ovstage_ordinal_t endOrdinal,
                                   PulledScene* changes,
                                   PulledScene* sourceDelta,
                                   PulledScene* nextSource,
                                   std::set<std::string>* nextSourceAttributes,
                                   SceneGraphInstanceIndex* nextSceneGraphInstances)
{
    const double profileStart = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
    ovstage_instance_t* stage = backend->ext;
    path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
    {
        setBackendError("pull: attached stage has no path dictionary");
        if (isProfilingEnabled())
            std::fprintf(stderr, "OVGLBEPROF delta fallback=no-dictionary\n");
        return PublicPullStatus::eError;
    }

    ovstage_filter_t filter{};
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t queryOperation = ovstage_query(stage, &filter, nullptr, 0, &query);
    if (queryOperation.status != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        setBackendError("pull: match-all query enqueue failed on the attached stage");
        if (isProfilingEnabled())
            std::fprintf(stderr, "OVGLBEPROF delta fallback=query-enqueue\n");
        return PublicPullStatus::eError;
    }
    ovstage_query_result_t queryResult{};
    const ovstage_api_status_t fetched = ovstage_fetch_query_result(stage, query, OVSTAGE_TIMEOUT_INFINITE, &queryResult);
    const bool queryCompleted = instComplete(stage, queryOperation, "pull query");
    if (!queryCompleted || fetched != OVSTAGE_OK)
    {
        if (fetched == OVSTAGE_OK)
            (void)ovstage_release_query_result(stage, &queryResult);
        (void)instComplete(stage, ovstage_release_query(stage, query), "pull query release");
        setBackendError("pull: fetch_query_result failed on the attached stage");
        if (isProfilingEnabled())
            std::fprintf(stderr, "OVGLBEPROF delta fallback=query-fetch\n");
        return PublicPullStatus::eError;
    }

    std::vector<ovx_token_t> attributes;
    if (queryResult.attribute_count > 0)
        attributes.assign(queryResult.attributes, queryResult.attributes + queryResult.attribute_count);
    if (ovstage_release_query_result(stage, &queryResult) != OVSTAGE_OK)
    {
        (void)instComplete(stage, ovstage_release_query(stage, query), "pull query release");
        setBackendError("pull: release_query_result failed on the attached stage");
        if (isProfilingEnabled())
            std::fprintf(stderr, "OVGLBEPROF delta fallback=query-result-release\n");
        return PublicPullStatus::eError;
    }
    const double profileQuery = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
    // Do not reconstruct membership by reading only `usd-prim-type`: valid
    // OVStage scenes can contain untyped prims. Do not compare the query's raw
    // total either: it can include a prim with no readable current attribute,
    // as the warehouse does. Both approaches made every camera edit look like
    // topology churn and forced a multi-second full rebuild. Instead, fold each
    // touched prim into its retained readable row below and compare whether that
    // row existed before and after the window. This detects additions, removals,
    // and equal-cardinality replacements in the renderer-consumable scene
    // without copying or enumerating the entire snapshot.

    if (!addRequiredPullAttributes(dictionary, &attributes))
    {
        setBackendError("pull: unable to create reserved OVStage attribute tokens");
        (void)instComplete(stage, ovstage_release_query(stage, query), "pull query release");
        return PublicPullStatus::eError;
    }
    *nextSourceAttributes = backend->attached_source_attributes;
    *nextSceneGraphInstances = backend->attached_scene_graph_instances;
    for (const std::string& retainedAttribute : backend->attached_source_attributes)
    {
        const ovx_string_t name{ retainedAttribute.c_str(), retainedAttribute.size() };
        ovx_token_t token = OVX_INVALID_TOKEN;
        if (path_dictionary_create_tokens_from_strings(dictionary, &name, 1, &token).status != OVX_API_SUCCESS ||
            token == OVX_INVALID_TOKEN)
        {
            setBackendError("pull: unable to recreate a retained OVStage attribute token");
            (void)instComplete(stage, ovstage_release_query(stage, query), "pull query release");
            return PublicPullStatus::eError;
        }
        if (std::find(attributes.begin(), attributes.end(), token) == attributes.end())
            attributes.push_back(token);
    }
    PulledScene sourceChanges;
    ovstage_ordinal_range_t range{};
    range.has_start_ordinal = true;
    range.start_ordinal = backend->attached_pull_ordinal + 1;
    range.end_ordinal = endOrdinal;
    const double profilePrepared = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
    const bool readSucceeded = readQueryAttributes(stage, dictionary, query, attributes, range, &sourceChanges);
    const double profileRead = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
    const bool releaseSucceeded = instComplete(stage, ovstage_release_query(stage, query), "pull query release");
    if (!readSucceeded || !releaseSucceeded)
        return PublicPullStatus::eError;
    if (isProfilingEnabled())
    {
        const double profileDone = getCurrentTimeMilliseconds();
        std::fprintf(stderr, "OVGLBEPROF delta timings query=%.3f prepare=%.3f read=%.3f release=%.3f\n",
                     profileQuery - profileStart, profilePrepared - profileQuery, profileRead - profilePrepared,
                     profileDone - profileRead);
    }

    *sourceDelta = sourceChanges;
    nextSource->clear();
    for (const auto& prim : sourceChanges)
        for (const auto& column : prim.second)
            nextSourceAttributes->insert(column.first);
    for (const auto& prim : sourceChanges)
    {
        const auto current = backend->attached_source_pull.find(prim.first);
        std::map<std::string, PulledColumn> nextColumns;
        if (current != backend->attached_source_pull.end())
            nextColumns = current->second;
        for (const auto& column : prim.second)
        {
            if (column.second.deleted)
                nextColumns.erase(column.first);
            else
                nextColumns[column.first] = column.second;
        }
        if ((current != backend->attached_source_pull.end()) != !nextColumns.empty())
        {
            if (isProfilingEnabled())
                std::fprintf(stderr, "OVGLBEPROF delta fallback=topology-window path=%s\n", prim.first.c_str());
            return PublicPullStatus::eTopologyChanged;
        }
    }

    bool rebuildAllInstances = false;
    std::set<std::string> affectedInstanceRoots;
    for (const auto& prim : sourceChanges)
    {
        const std::vector<std::string>* prototypeInstanceRoots =
            findPrototypeInstanceRoots(backend->attached_scene_graph_instances, prim.first);
        const bool instanceMetadata =
            prim.second.count("_protoPath") != 0 || prim.second.count("_isSceneGraphInstancingRoot") != 0;
        const bool transformChanged = std::any_of(
            prim.second.begin(), prim.second.end(), [](const auto& column) { return isTransformColumn(column.first); });
        const bool nonTransformChanged = std::any_of(
            prim.second.begin(), prim.second.end(), [](const auto& column) { return !isTransformColumn(column.first); });
        // Metadata can change the dependency index, while non-transform
        // prototype edits can add or remove expanded values. Rebuild the
        // compatibility view in those cases. Transform-only changes retain
        // the topology and can refresh just this prototype's visible roots.
        if (instanceMetadata || (prototypeInstanceRoots && nonTransformChanged))
        {
            rebuildAllInstances = true;
            break;
        }
        if (!transformChanged)
            continue;
        collectAffectedInstanceRoots(backend->attached_scene_graph_instances, prim.first, &affectedInstanceRoots);
        if (prototypeInstanceRoots)
            affectedInstanceRoots.insert(prototypeInstanceRoots->begin(), prototypeInstanceRoots->end());
    }

    if (!rebuildAllInstances && affectedInstanceRoots.empty())
    {
        *changes = std::move(sourceChanges);
        return PublicPullStatus::eSuccess;
    }

    *nextSource = backend->attached_source_pull;
    applyPulledChanges(nextSource, sourceChanges);
    PulledScene expanded;
    if (rebuildAllInstances)
    {
        expanded = *nextSource;
        if (!expandSceneGraphInstances(stage, &expanded, nextSceneGraphInstances))
            return PublicPullStatus::eError;
    }
    else
    {
        expanded = backend->attached_pull;
        applyPulledChanges(&expanded, sourceChanges);
        for (const std::string& instanceRoot : affectedInstanceRoots)
        {
            for (auto prim = expanded.lower_bound(instanceRoot);
                 prim != expanded.end() && hasPathPrefix(prim->first, instanceRoot);)
            {
                prim = expanded.erase(prim);
            }
            for (auto prim = nextSource->lower_bound(instanceRoot);
                 prim != nextSource->end() && hasPathPrefix(prim->first, instanceRoot); ++prim)
            {
                expanded.emplace(*prim);
            }
        }
        if (!expandSceneGraphInstances(stage, &expanded, nullptr, &affectedInstanceRoots))
            return PublicPullStatus::eError;
    }
    buildSnapshotDiff(expanded, backend->attached_pull, endOrdinal, changes);
    return PublicPullStatus::eSuccess;
}

// Columns the attached mirror treats as pure transform state. The mirror
// never copies these verbatim: localMatrix/omni:xform feed the transform-
// authority rule, an authoritative worldMatrix is forwarded to the mirror's
// hierarchy model, and raw xformOp:* stay flattened. A change window containing only
// these columns cannot alter topology, materials, products, or prim
// existence — but it CAN move prims whose pose matters beyond their own
// vertices: cameras (re-derived per step from the merged view) and LIGHTS,
// whose composed world pose ovgl bakes into GpuLight at scene-build time.
// The kXformOnly path therefore refreshes light state alongside the mesh
// worldMatrix refresh (ovgl_refresh_transforms -> ovgl_scene_refresh_lights;
// 2026-07-19 fast-path adversary F1). Deleted columns never classify here:
// any tombstone in the window forces kFull.
// omni:fabric:{local,world}Matrix are the Kit/Fabric spellings of the same two
// columns. ovrtx_write_attribute rejects them on the OFFICIAL write path
// (deprecated; omni:xform is the sanctioned input), but a client that writes
// its own attached ovstage directly — Kit-ported samples do, and ovphysx's
// OvstageSource reads them as a transform source (OvstageSource.h:45) —
// deposits them in the change window all the same. Treating them as anything
// but transforms costs a full scene rebuild per frame AND drops the pose.
bool isLocalTransformColumn(const std::string& attribute)
{
    return attribute == "localMatrix" || attribute == "omni:xform" || attribute == "omni:fabric:localMatrix";
}

bool isTransformColumn(const std::string& attribute)
{
    return isLocalTransformColumn(attribute) || attribute == "worldMatrix" || attribute == "xformOpOrder" ||
           attribute == "omni:fabric:worldMatrix" || attribute.rfind("xformOp:", 0) == 0;
}

// Pick the newest local transform spelling. Ties break toward the last
// spelling listed, so the order is
// population-derived, then deprecated-Fabric, then the official attach-era
// live-edit column: a client that writes both omni:xform and the Fabric
// name at one ordinal keeps exactly the pre-Fabric behavior.
const PulledColumn* pickLocalAuthority(const std::map<std::string, PulledColumn>& cols)
{
    static const char* const s_kLocalSpellings[] = {
        "localMatrix",
        "omni:fabric:localMatrix",
        "omni:xform",
    };
    const PulledColumn* best = nullptr;
    for (const char* name : s_kLocalSpellings)
    {
        auto it = cols.find(name);
        if (it == cols.end() || it->second.deleted)
            continue;
        if (!best || it->second.ordinal >= best->ordinal)
            best = &it->second;
    }
    return best;
}

// Use the official hierarchy input spelling in the renderer-owned mirror.
// Population may expose the deprecated Fabric local column as well, but new
// generic writes must use omni:xform to participate in hierarchy computation.
constexpr const char* g_kMirrorLocalTransform = "omni:xform";
constexpr const char* g_kMirrorWorldTransform = "omni:fabric:worldMatrix";

const PulledColumn* pickWorldAuthority(const std::map<std::string, PulledColumn>& cols)
{
    static const char* const s_kWorldSpellings[] = {
        "omni:fabric:worldMatrix",
        "worldMatrix",
    };
    const PulledColumn* best = nullptr;
    for (const char* name : s_kWorldSpellings)
    {
        auto it = cols.find(name);
        if (it == cols.end() || it->second.deleted)
            continue;
        if (!best || it->second.ordinal >= best->ordinal)
            best = &it->second;
    }
    return best;
}

// The attached OVStage is the composed scene-data authority. Forward its world
// transform unless a newer local edit still needs composition. This rule is
// producer-neutral: stage writers publish normal columns without an
// OVGL-specific provenance marker.
const PulledColumn* pickSourceWorld(const std::map<std::string, PulledColumn>& cols)
{
    const PulledColumn* world = pickWorldAuthority(cols);
    if (!world)
        return nullptr;
    const PulledColumn* local = pickLocalAuthority(cols);
    if (local && local->ordinal > world->ordinal)
        return nullptr;
    return world;
}

// Write one pulled column into the mirror at `ord` (single-prim UPSERT).
bool mirrorWriteColumn(OvglBackend* be,
                       ovstage_ordinal_t ord,
                       const std::string& path,
                       const std::string& attr,
                       const void* data,
                       size_t nbytes,
                       DLDataType dtype,
                       ovstage_attribute_semantic_t sem,
                       bool is_array)
{

    ovx_string_t ps = toOvx(path);
    owned_path_query query;
    if (!createOwnedPathQuery(be, &ps, 1, "mirror: query(" + path + ")", &query))
        return false;

    const size_t eb = dlElemBytes(dtype);
    if (eb == 0 || nbytes % eb != 0)
    {
        (void)releaseOwnedPathQuery(be, &query, "mirror release invalid payload query");
        setBackendError("mirror: payload for " + attr + " does not divide by its element size");
        return false;
    }
    const int64_t rows = static_cast<int64_t>(nbytes / eb);

    DLTensor t{};
    t.data = const_cast<void*>(data);
    t.device.device_type = kDLCPU;
    t.device.device_id = 0;
    t.dtype = dtype;
    int64_t shape[2];
    if (is_array)
    {
        t.ndim = 2; // one ragged row for this single prim
        shape[0] = 1;
        shape[1] = rows;
    }
    else
    {
        t.ndim = 1; // one fixed row of `rows` elements
        shape[0] = rows;
    }
    t.shape = shape;
    t.strides = nullptr;
    t.byte_offset = 0;

    ovstage_write_data_t wd{};
    wd.tensors = &t;
    wd.managed_tensors = nullptr;
    wd.tensor_count = 1;
    wd.count = 0;
    wd.index_map = nullptr;
    wd.mask = nullptr;
    wd.cuda_sync = ovstage_cuda_sync_t{};
    wd.semantic = sem;
    wd.is_array = is_array;

    ovx_string_or_token_t sot{};
    sot.token = 0;
    sot.string = toOvx(attr);

    ovstage_enqueue_result_t er = ovstage_write_attribute(be->stage, query.query, sot, ord, wd, OVSTAGE_PRIM_MODE_UPSERT);
    if (er.status != OVSTAGE_OK)
    {
        (void)releaseOwnedPathQuery(be, &query, "mirror release after failed write");
        setBackendError("mirror: write(" + path + "." + attr + ")");
        return false;
    }
    const bool wrote = completeEnqueue(be, er, "mirror write(" + path + "." + attr + ")");
    const bool released = releaseOwnedPathQuery(be, &query, "mirror release after write(" + path + "." + attr + ")");
    return wrote && released;
}

// Delete a whole prim from the mirror at `ord`.
bool mirrorDeletePrim(OvglBackend* be, ovstage_ordinal_t ord, const std::string& path)
{
    ovx_string_t ps = toOvx(path);
    owned_path_query query;
    if (!createOwnedPathQuery(be, &ps, 1, "mirror: delete query(" + path + ")", &query))
        return false;
    const bool ok = instComplete(
        be->stage, ovstage_delete_attributes(be->stage, query.query, nullptr, 0, ord), "mirror delete " + path);
    const bool rel_ok = releaseOwnedPathQuery(be, &query, "mirror delete release");
    return ok && rel_ok;
}

// Tombstone named attributes on one mirrored prim at `ord` (per-attribute
// delete: the prim and its sibling columns live on). The mirror is an
// accumulating UPSERT store, so a column the attached stage deleted must be
// tombstoned here too or every warmed renderer keeps rendering it forever
// (fast-path adversary F2, 2026-07-19). Deleting a column the mirror never
// carried is an engine-level idempotent no-op; the queried prim must exist.
bool mirrorDeleteColumns(OvglBackend* be,
                         ovstage_ordinal_t ord,
                         const std::string& path,
                         const std::vector<std::string>& attrs)
{
    if (attrs.empty())
        return true;
    ovx_string_t ps = toOvx(path);
    owned_path_query query;
    if (!createOwnedPathQuery(be, &ps, 1, "mirror: delete-column query(" + path + ")", &query))
        return false;
    std::vector<ovx_string_or_token_t> names(attrs.size());
    for (size_t i = 0; i < attrs.size(); ++i)
    {
        names[i].token = 0;
        names[i].string = toOvx(attrs[i]);
    }
    const bool ok =
        instComplete(be->stage, ovstage_delete_attributes(be->stage, query.query, names.data(), names.size(), ord),
                     "mirror delete columns on " + path);
    const bool rel_ok = releaseOwnedPathQuery(be, &query, "mirror delete-column release");
    return ok && rel_ok;
}

// Re-intern a u64 id payload (token / path / (path,token) pair columns) from
// the external dictionary into the mirror dictionary. `kind`: 0 = tokens,
// 1 = paths, 2 = (path, token) pairs.
bool reinternIds(OvglBackend* be,
                 path_dictionary_instance_t* ext_dict,
                 const std::vector<uint8_t>& src,
                 int kind,
                 std::vector<uint8_t>* out)
{
    out->clear();
    if (src.size() % sizeof(uint64_t) != 0)
    {
        setBackendError("mirror: id payload is not a multiple of 8 bytes");
        return false;
    }
    if (kind == 2 && (src.size() / sizeof(uint64_t)) % 2 != 0)
    {
        setBackendError("mirror: connection id payload is not made of path/token pairs");
        return false;
    }
    path_dictionary_instance_t* mir_dict = ovstage_get_path_dictionary(be->stage);
    if (!mir_dict)
    {
        setBackendError("mirror: no path dictionary");
        return false;
    }
    const size_t n = src.size() / sizeof(uint64_t);
    out->resize(src.size());
    for (size_t i = 0; i < n; ++i)
    {
        const uint64_t id = loadU64At(src.data() + i * 8);
        uint64_t mapped = 0;
        if (id == 0)
        {
            mapped = 0; // empty/invalid stays empty (e.g. connection sans property)
        }
        else
        {
            const bool as_path = (kind == 1) || (kind == 2 && (i % 2 == 0));
            std::string s;
            const bool resolved = as_path ? getDictionaryPathString(ext_dict, static_cast<ovx_primpath_t>(id), &s) :
                                            getDictionaryTokenString(ext_dict, static_cast<ovx_token_t>(id), &s);
            if (!resolved)
            {
                setBackendError("mirror: failed to resolve an id from the attached stage's dictionary");
                return false;
            }
            ovx_string_t sv = toOvx(s);
            if (as_path)
            {
                ovx_primpath_t ph = OVX_INVALID_PRIMPATH;
                if (mir_dict->vtable->create_paths_from_strings(mir_dict->context, &sv, 1, &ph).status != OVX_API_SUCCESS ||
                    ph == OVX_INVALID_PRIMPATH)
                {
                    setBackendError("mirror: failed to re-intern path '" + s + "'");
                    return false;
                }
                mapped = static_cast<uint64_t>(ph);
            }
            else
            {
                ovx_token_t th = OVX_INVALID_TOKEN;
                if (path_dictionary_create_tokens_from_strings(mir_dict, &sv, 1, &th).status != OVX_API_SUCCESS ||
                    th == OVX_INVALID_TOKEN)
                {
                    setBackendError("mirror: failed to re-intern token '" + s + "'");
                    return false;
                }
                mapped = static_cast<uint64_t>(th);
            }
        }
        std::memcpy(out->data() + i * 8, &mapped, sizeof(mapped));
    }
    return true;
}

struct PreparedMirrorColumn
{
    std::vector<uint8_t> bytes;
    DLDataType dtype{};
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    bool is_array = false;
};

bool prepareMirrorColumn(OvglBackend* be,
                         const std::string& attribute,
                         const PulledColumn& column,
                         path_dictionary_instance_t* external_dictionary,
                         PreparedMirrorColumn* prepared)
{
    prepared->semantic = column.semantic;
    prepared->is_array = column.is_array;
    if (attribute == "usd-prim-type" || attribute == "usd-schemas" || column.semantic == OVSTAGE_SEMANTIC_TOKEN_ID)
    {
        prepared->dtype = DLDataType{ kDLUInt, 64, 1 };
        return reinternIds(be, external_dictionary, column.bytes, 0, &prepared->bytes);
    }
    if (column.semantic == OVSTAGE_SEMANTIC_RELATIONSHIP_PATH_ID)
    {
        prepared->dtype = DLDataType{ kDLUInt, 64, 1 };
        return reinternIds(be, external_dictionary, column.bytes, 1, &prepared->bytes);
    }
    if (column.semantic == OVSTAGE_SEMANTIC_CONNECTION_PATH_ID)
    {
        prepared->dtype = DLDataType{ kDLUInt, 64, 2 };
        return reinternIds(be, external_dictionary, column.bytes, 2, &prepared->bytes);
    }
    prepared->bytes = column.bytes;
    prepared->dtype = column.dtype;
    if (dlElemBytes(prepared->dtype) == 0 || prepared->bytes.size() % dlElemBytes(prepared->dtype) != 0)
        prepared->dtype = DLDataType{ kDLUInt, 8, 1 };
    return true;
}

bool mirrorPulledColumn(OvglBackend* be,
                        ovstage_ordinal_t ordinal,
                        const std::string& path,
                        const std::string& attribute,
                        const PulledColumn& column,
                        path_dictionary_instance_t* external_dictionary)
{
    PreparedMirrorColumn prepared;
    return prepareMirrorColumn(be, attribute, column, external_dictionary, &prepared) &&
           mirrorWriteColumn(be, ordinal, path, attribute, prepared.bytes.data(), prepared.bytes.size(), prepared.dtype,
                             prepared.semantic, prepared.is_array);
}

struct MirrorBatchKey
{
    std::string attribute;
    DLDataType dtype{};
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    bool is_array = false;
    size_t fixed_row_bytes = 0;

    bool operator<(const MirrorBatchKey& other) const
    {
        return std::tie(attribute, dtype.code, dtype.bits, dtype.lanes, semantic, is_array, fixed_row_bytes) <
               std::tie(other.attribute, other.dtype.code, other.dtype.bits, other.dtype.lanes, other.semantic,
                        other.is_array, other.fixed_row_bytes);
    }
};

struct MirrorBatchRow
{
    std::string path;
    std::vector<uint8_t> bytes;
};

using MirrorBatches = std::map<MirrorBatchKey, std::vector<MirrorBatchRow>>;

void addMirrorBatch(MirrorBatches* batches,
                    const std::string& path,
                    const std::string& attribute,
                    PreparedMirrorColumn prepared)
{
    MirrorBatchKey key;
    key.attribute = attribute;
    key.dtype = prepared.dtype;
    key.semantic = prepared.semantic;
    key.is_array = prepared.is_array;
    key.fixed_row_bytes = prepared.is_array ? 0 : prepared.bytes.size();
    (*batches)[std::move(key)].push_back(MirrorBatchRow{ path, std::move(prepared.bytes) });
}

bool addMatrixMirrorBatch(MirrorBatches* batches,
                          const std::string& path,
                          const std::string& attribute,
                          const PulledColumn& column)
{
    if (column.bytes.size() != 16 * sizeof(double))
    {
        setBackendError("transform for '" + path + "' is not a 4x4 double matrix");
        return false;
    }
    PreparedMirrorColumn prepared;
    prepared.bytes = column.bytes;
    prepared.dtype = DLDataType{ kDLFloat, 64, 16 };
    prepared.semantic = OVSTAGE_SEMANTIC_MATRIX;
    addMirrorBatch(batches, path, attribute, std::move(prepared));
    return true;
}

bool mirrorWriteBatch(OvglBackend* be,
                      ovstage_ordinal_t ordinal,
                      const MirrorBatchKey& key,
                      const std::vector<MirrorBatchRow>& rows)
{
    if (rows.empty())
        return true;
    if (rows.size() > std::numeric_limits<uint32_t>::max())
    {
        setBackendError("mirror batch has too many rows for " + key.attribute);
        return false;
    }
    const size_t element_bytes = dlElemBytes(key.dtype);
    if (element_bytes == 0)
    {
        setBackendError("mirror batch has invalid dtype for " + key.attribute);
        return false;
    }

    std::vector<ovx_string_t> path_strings(rows.size());
    for (size_t index = 0; index < rows.size(); ++index)
        path_strings[index] = toOvx(rows[index].path);
    owned_path_query query;
    if (!createOwnedPathQuery(
            be, path_strings.data(), path_strings.size(), "mirror batch query for " + key.attribute, &query))
        return false;
    const auto release_resources = [&]()
    { return releaseOwnedPathQuery(be, &query, "mirror batch release for " + key.attribute); };

    std::vector<uint8_t> fixed_bytes;
    std::vector<DLTensor> tensors;
    std::vector<std::array<int64_t, 2>> shapes;
    if (key.is_array)
    {
        tensors.resize(rows.size());
        shapes.resize(rows.size());
        for (size_t index = 0; index < rows.size(); ++index)
        {
            if (rows[index].bytes.size() % element_bytes != 0)
            {
                (void)release_resources();
                setBackendError("mirror batch array row does not divide for " + key.attribute);
                return false;
            }
            if (rows[index].bytes.size() / element_bytes > static_cast<size_t>(std::numeric_limits<int64_t>::max()))
            {
                (void)release_resources();
                setBackendError("mirror batch array row is too large for " + key.attribute);
                return false;
            }
            DLTensor& tensor = tensors[index];
            tensor.data = rows[index].bytes.empty() ? nullptr : const_cast<uint8_t*>(rows[index].bytes.data());
            tensor.device = DLDevice{ kDLCPU, 0 };
            tensor.dtype = key.dtype;
            shapes[index][0] = static_cast<int64_t>(rows[index].bytes.size() / element_bytes);
            if (rows.size() == 1)
            {
                tensor.ndim = 2;
                shapes[index][1] = shapes[index][0];
                shapes[index][0] = 1;
            }
            else
            {
                tensor.ndim = 1;
            }
            tensor.shape = shapes[index].data();
        }
    }
    else
    {
        if (key.fixed_row_bytes == 0 || key.fixed_row_bytes % element_bytes != 0 ||
            key.fixed_row_bytes > std::numeric_limits<size_t>::max() / rows.size())
        {
            (void)release_resources();
            setBackendError("mirror batch fixed row has an invalid extent for " + key.attribute);
            return false;
        }
        fixed_bytes.reserve(key.fixed_row_bytes * rows.size());
        for (const MirrorBatchRow& row : rows)
        {
            if (row.bytes.size() != key.fixed_row_bytes)
            {
                (void)release_resources();
                setBackendError("mirror batch fixed row changed size for " + key.attribute);
                return false;
            }
            fixed_bytes.insert(fixed_bytes.end(), row.bytes.begin(), row.bytes.end());
        }
        tensors.resize(1);
        shapes.resize(1);
        DLTensor& tensor = tensors[0];
        tensor.data = fixed_bytes.empty() ? nullptr : fixed_bytes.data();
        tensor.device = DLDevice{ kDLCPU, 0 };
        tensor.dtype = key.dtype;
        const int64_t row_elements = static_cast<int64_t>(key.fixed_row_bytes / element_bytes);
        if (row_elements == 1)
        {
            tensor.ndim = 1;
            shapes[0][0] = static_cast<int64_t>(rows.size());
        }
        else
        {
            tensor.ndim = 2;
            shapes[0][0] = static_cast<int64_t>(rows.size());
            shapes[0][1] = row_elements;
        }
        tensor.shape = shapes[0].data();
    }

    ovstage_write_data_t write{};
    write.tensors = tensors.data();
    write.tensor_count = static_cast<uint32_t>(tensors.size());
    write.semantic = key.semantic;
    write.is_array = key.is_array;
    ovx_string_or_token_t attribute{};
    attribute.string = toOvx(key.attribute);
    const ovstage_enqueue_result_t enqueued =
        ovstage_write_attribute(be->stage, query.query, attribute, ordinal, write, OVSTAGE_PRIM_MODE_UPSERT);
    if (enqueued.status != OVSTAGE_OK)
    {
        (void)release_resources();
        setBackendError("mirror batch write for " + key.attribute);
        return false;
    }
    const bool wrote = completeEnqueue(be, enqueued, "mirror batch write for " + key.attribute);
    return release_resources() && wrote;
}

bool mirrorWriteBatches(OvglBackend* be, ovstage_ordinal_t ordinal, const MirrorBatches& batches)
{
    for (const auto& batch : batches)
        if (!mirrorWriteBatch(be, ordinal, batch.first, batch.second))
            return false;
    return true;
}

// USD Camera schema defaults (authored values override; population only
// mirrors AUTHORED attributes, so unauthored ones fall back to the schema).
constexpr float g_kUsdDefaultFocalLength = 50.0f;
constexpr float g_kUsdDefaultHorizontalAperture = 20.955f;

bool pulledScalarF32(const PulledScene& scene, const std::string& prim, const std::string& attr, float* out)
{
    auto p = scene.find(prim);
    if (p == scene.end())
        return false;
    auto a = p->second.find(attr);
    if (a == p->second.end() || a->second.deleted || a->second.bytes.size() < sizeof(float))
        return false;
    std::memcpy(out, a->second.bytes.data(), sizeof(float));
    return true;
}

// The Depth-class image var ovgl expresses: the official 0.4 depth AOV with
// image-plane (perpendicular, camera-linear) semantics. The rasterizer's
// z-buffer linearizes to exactly this quantity; DepthSD (reverse-z unitless)
// is pinned broken in the official 0.4.0 C readback and DistanceToCameraSD
// (Euclidean) is a possible follow-on — see the contract-ledger notes.
constexpr const char* g_kDepthVarName = "DistanceToImagePlaneSD";

// Stage units -> meters for the depth payload. Population authors the
// RESOLVED metersPerUnit ({kDLFloat,64,1}) onto the reserved stage-info prim
// on every ingest; a stage that never went through population (raw-authored
// columns) has no row, and the official USD fallback 0.01 applies — measured
// on the official 0.4 engine: with metersPerUnit unauthored, DistanceTo*SD
// payloads read stage units x 0.01 (official-depth probe, 2026-07-20).
constexpr double g_kUsdFallbackMetersPerUnit = 0.01;

double pulledMetersPerUnit(const PulledScene& scene)
{
    const ovx_string_t info = ovstage_population_stage_info_path();
    auto p = scene.find(std::string(info.ptr ? info.ptr : "", info.ptr ? info.length : 0));
    if (p == scene.end())
        return g_kUsdFallbackMetersPerUnit;
    auto a = p->second.find("metersPerUnit");
    if (a == p->second.end() || a->second.deleted || a->second.bytes.size() < sizeof(double))
        return g_kUsdFallbackMetersPerUnit;
    double mpu = 0.0;
    std::memcpy(&mpu, a->second.bytes.data(), sizeof(double));
    return (std::isfinite(mpu) && mpu > 0.0) ? mpu : g_kUsdFallbackMetersPerUnit;
}

} // namespace

OvglBackend* createOvglBackend(const char* name)
{
    OvglBackend* be = new OvglBackend();

    ovstage_instance_desc_t desc{};
    desc.name = name;
    if (ovstage_create_instance(&desc, &be->stage) != OVSTAGE_OK || !be->stage)
    {
        setBackendError("ovstage_create_instance failed");
        delete be;
        return nullptr;
    }
    ovgl_renderer_desc_t rd{};
    rd.gpu_device_id = 0;
    rd.samples_per_pixel = 1;
    rd.max_bounces = 1;
    rd.name = name ? name : "isaacsim.ovgl_viewport.debug";
    if (ovgl_create_renderer(&rd, &be->renderer).status != 0 || !be->renderer)
    {
        setBackendError("ovgl_create_renderer failed");
        ovstage_destroy_instance(be->stage);
        delete be;
        return nullptr;
    }
    if (ovgl_attach_ovstage(be->renderer, be->stage).status != 0)
    {
        setBackendError("ovgl_attach_ovstage failed");
        ovgl_destroy_renderer(be->renderer);
        ovstage_destroy_instance(be->stage);
        delete be;
        return nullptr;
    }
    return be;
}

int destroyOvglBackend(OvglBackend* be)
{
    if (!be)
        return 0;
    if (be->renderer)
    {
        const ovgl_result_t result = ovgl_destroy_renderer(be->renderer);
        if (result.status != 0)
        {
            const ovx_string_t error = ovgl_get_last_error();
            setBackendError(std::string("ovgl_destroy_renderer: ") +
                            std::string(error.ptr ? error.ptr : "", error.ptr ? error.length : 0));
            return 1;
        }
        be->renderer = nullptr;
    }
    if (be->stage)
        ovstage_destroy_instance(be->stage);
    delete be;
    return 0;
}

int resetStage(OvglBackend* be)
{
    if (!be || !be->stage)
    {
        setBackendError("reset_stage: null backend");
        return 1;
    }
    // Tombstone every tracked prim at a fresh ordinal. Bookkeeping is retained
    // unless every delete and release completes and the ordinal seals, so a
    // failed reset can be retried without losing the paths that still need
    // deletion.
    ovstage_ordinal_t o = be->cur_ordinal;
    if (o == std::numeric_limits<ovstage_ordinal_t>::max())
    {
        setBackendError("reset_stage: ordinal space exhausted");
        return 1;
    }
    // Keep using this ordinal until it is sealed. Whole-prim deletes are
    // idempotent at a given ordinal, so a partial failure can safely retry
    // instead of skipping past an open accumulating frame.
    for (const std::string& p : be->prims)
    {
        ovx_string_t ps = toOvx(p);
        owned_path_query query;
        if (!createOwnedPathQuery(be, &ps, 1, "reset_stage query for " + p, &query))
            return 1;

        const bool delete_ok = completeEnqueue(
            be, ovstage_delete_attributes(be->stage, query.query, nullptr, 0, o), "reset_stage delete " + p);
        const bool query_release_ok = releaseOwnedPathQuery(be, &query, "reset_stage query release " + p);
        if (!delete_ok || !query_release_ok)
        {
            return 1;
        }
    }
    bool ordinal_sealed = false;
    if (!sealOrdinal(be, o, &ordinal_sealed))
    {
        // A successful floor advance followed by an op-handle release failure
        // still closes the ordinal; preserve that fact while retaining the
        // prim bookkeeping so a later reset can retry at the next ordinal.
        if (ordinal_sealed)
            be->cur_ordinal = o + 1;
        return 1;
    }
    be->cur_ordinal = o + 1;
    be->prims.clear();
    // The mirror no longer matches any retained attached pull; the next
    // attached step must take the full pull + mirror path. (Attach and detach
    // both route through this reset.)
    be->attached_pull.clear();
    be->attached_source_pull.clear();
    be->attached_source_attributes.clear();
    be->attached_scene_graph_instances = {};
    be->attached_pull_valid = false;
    be->attached_pull_ordinal = 0;
    be->attached_render_ordinal = 0;
    be->attached_scene_ordinal = 0;
    be->mirror_transform_dirty = false;
    be->attached_light_paths.clear();
    be->attached_light_paths_valid = false;
    be->pulled_transform_dirty = false;
    be->transform_journal_generation = 0;
    be->transform_journal_paths.clear();
    be->transform_journal_routes.clear();
    return 0;
}

static int setCamera(OvglBackend* be,
                     const double eye[3],
                     const double target[3],
                     const double up[3],
                     double fov_y_rad,
                     int width,
                     int height)
{
    if (!be || !eye || !target || !up)
        return 1;
    ovgl_camera_t cam{};
    for (int i = 0; i < 3; ++i)
    {
        cam.eye[i] = eye[i];
        cam.target[i] = target[i];
        cam.up[i] = up[i];
    }
    cam.fov_y_rad = fov_y_rad;
    cam.image_width = width;
    cam.image_height = height;
    if (ovgl_set_camera(be->renderer, &cam).status != 0)
    {
        ovx_string_t e = ovgl_get_last_error();
        setBackendError(std::string("ovgl_set_camera: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0));
        return 1;
    }
    return 0;
}

const char* getOvglBackendError(void)
{
    return g_backendError.c_str();
}

// Attach lane (ovrtx 0.4).

int attachStage(OvglBackend* be, ovstage_instance_t* externalInstance, char* err, size_t errlen)
{
    auto fail = [&](const std::string& m) -> int
    {
        setBackendError(m);
        if (err && errlen)
            std::snprintf(err, errlen, "%s", m.c_str());
        return 1;
    };
    if (!be)
        return 1;
    if (be->ext)
        return fail("an ovstage is already attached");
    if (!externalInstance || !externalInstance->vtable || !externalInstance->context)
        return fail("attached instance is not a {vtable, context} ovstage bundle");

    // Vtable-version guard: this renderer speaks the official 0.1 28-slot ABI.
    // A skewed instance must fail loudly at attach, never dispatch wrong slots.
    uint32_t maj = 0, min = 0, pat = 0;
    ovstage_get_version(externalInstance, &maj, &min, &pat);
    if (maj != 0 || min != 1)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "attached ovstage reports ABI %u.%u.%u; this renderer requires the "
                      "official 0.1 vtable",
                      maj, min, pat);
        return fail(buf);
    }

    // Probe the private accessor extension by name and proceed WITHOUT it —
    // this adapter uses only the public ABI, so every attached read goes
    // through the 28 public slots. A conformant Fabric-free ovstage answers
    // NOT_FOUND here; if some implementation does publish it, we still do not
    // use it.
    const void* accessors = nullptr;
    (void)ovstage_query_extension(externalInstance, "ovstage.internal.accessors", &accessors);

    // Clear the internal mirror before attaching the borrowed OVStage.
    if (resetStage(be) != 0)
        return fail(std::string("attach: mirror reset failed: ") + g_backendError);
    be->mirror_pulled.clear();
    be->mirror_cols.clear();
    be->attached_products.clear();
    be->ext = externalInstance;
    return 0;
}

int detachStage(OvglBackend* be)
{
    if (!be)
        return 1;
    if (!be->ext)
    {
        setBackendError("no ovstage is attached");
        return 1;
    }
    be->ext = nullptr;
    be->attached_products.clear();
    // Return the mirror to a clean standalone state.
    if (resetStage(be) != 0)
        return 1;
    be->mirror_pulled.clear();
    be->mirror_cols.clear();
    return 0;
}

int getStageWriteFloor(OvglBackend* be, ovstage_ordinal_t* ordinal)
{
    if (!be || !be->ext || !ordinal)
        return 1;
    ovx_string_or_token_t global{};
    global.token = 0;
    global.string = ovx_string_t{ nullptr, 0 };
    ovstage_ordinal_query_handle_t oh = 0;
    ovstage_enqueue_result_t oe = ovstage_get_attribute_write_floor(be->ext, global, &oh);
    if (oe.status != OVSTAGE_OK)
    {
        setBackendError("attached stage: get_attribute_write_floor failed");
        return 1;
    }
    ovstage_ordinal_t floor_ord = 0;
    const ovstage_api_status_t fe = ovstage_fetch_ordinal(be->ext, oh, OVSTAGE_TIMEOUT_INFINITE, &floor_ord);
    const bool released =
        instComplete(be->ext, ovstage_release_ordinal_query(be->ext, oh), "attached write-floor query release");
    if (!instComplete(be->ext, oe, "attached write-floor query") || fe != OVSTAGE_OK || !released)
    {
        setBackendError("attached stage: fetch_ordinal(write floor) failed");
        return 1;
    }
    *ordinal = floor_ord;
    return 0;
}

namespace
{

// One derived render product (camera binding, resolution, ordered vars),
// resolved from a pulled scene BEFORE any mirror mutation so a bad product
// fails the step without advancing the mirror ordinal. `stage_word` provides
// context in validation errors.
bool deriveRenderProduct(const PulledScene& scene,
                         path_dictionary_instance_t* dict,
                         const std::string& product,
                         const char* stage_word,
                         DerivedProduct* out,
                         std::string* err)
{
    auto product_it = scene.find(product);
    std::string product_type;
    if (product_it != scene.end())
    {
        auto pt = product_it->second.find("usd-prim-type");
        if (pt != product_it->second.end() && pt->second.bytes.size() == sizeof(uint64_t))
            (void)getDictionaryTokenString(dict, loadU64At(pt->second.bytes.data()), &product_type);
    }
    if (product_type != "RenderProduct")
    {
        *err = "render product '" + product + "' is not a RenderProduct prim on the " + stage_word;
        return false;
    }

    auto product_col = [&](const char* name) -> const PulledColumn*
    {
        auto it = product_it->second.find(name);
        return (it == product_it->second.end() || it->second.deleted) ? nullptr : &it->second;
    };
    const PulledColumn* cam_rel = product_col("camera");
    if (!cam_rel || cam_rel->bytes.size() < sizeof(uint64_t))
    {
        *err = "render product '" + product + "' has no camera relationship";
        return false;
    }
    if (!getDictionaryPathString(dict, loadU64At(cam_rel->bytes.data()), &out->camera_path))
    {
        *err = "render product camera target could not be resolved";
        return false;
    }
    const PulledColumn* resolution = product_col("resolution");
    if (!resolution || resolution->bytes.size() < 2 * sizeof(int32_t))
    {
        *err = "render product '" + product + "' has no resolution";
        return false;
    }
    int32_t res_wh[2];
    std::memcpy(res_wh, resolution->bytes.data(), sizeof(res_wh));
    if (res_wh[0] <= 0 || res_wh[1] <= 0)
    {
        *err = "render product resolution must be positive";
        return false;
    }
    out->width = res_wh[0];
    out->height = res_wh[1];

    if (const PulledColumn* window = product_col("dataWindowNDC"))
    {
        // Official RenderProduct crop: outputs are clipped to the NDC window
        // (top-left origin). Populated as four float32s.
        if (window->bytes.size() < 4 * sizeof(float))
        {
            *err = "render product dataWindowNDC must hold four floats";
            return false;
        }
        float w4[4];
        std::memcpy(w4, window->bytes.data(), sizeof(w4));
        if (!(w4[0] >= 0.f && w4[1] >= 0.f && w4[2] <= 1.f && w4[3] <= 1.f && w4[0] < w4[2] && w4[1] < w4[3]))
        {
            *err =
                "render product dataWindowNDC must satisfy 0 <= x0 < x1 <= 1 "
                "and 0 <= y0 < y1 <= 1";
            return false;
        }
        out->has_data_window = true;
        for (int i = 0; i < 4; ++i)
            out->data_window[i] = w4[i];
    }

    if (const PulledColumn* ordered = product_col("orderedVars"))
    {
        const size_t n = ordered->bytes.size() / sizeof(uint64_t);
        for (size_t i = 0; i < n; ++i)
        {
            std::string var_path;
            if (!getDictionaryPathString(dict, loadU64At(ordered->bytes.data() + i * 8), &var_path))
            {
                *err = "orderedVars target could not be resolved";
                return false;
            }
            auto var_it = scene.find(var_path);
            if (var_it == scene.end())
                continue;
            auto sn = var_it->second.find("sourceName");
            if (sn == var_it->second.end() || sn->second.deleted)
                continue;
            std::string source_name;
            if (sn->second.semantic == OVSTAGE_SEMANTIC_TOKEN_ID && sn->second.bytes.size() == sizeof(uint64_t))
            {
                if (!getDictionaryTokenString(dict, loadU64At(sn->second.bytes.data()), &source_name))
                {
                    *err = "RenderVar sourceName token could not be resolved";
                    return false;
                }
            }
            else
            {
                source_name.assign(reinterpret_cast<const char*>(sn->second.bytes.data()), sn->second.bytes.size());
            }
            if (!source_name.empty())
                out->var_names.push_back(std::move(source_name));
        }
    }
    out->path = product;
    // Camera projection scalars + meters-per-unit, captured from the same
    // pulled snapshot so a cached derivation renders without a re-pull.
    // verticalAperture is not pulled, and neither are the product's
    // pixelAspectRatio / aspectRatioConformPolicy: official ovrtx 0.4 renders
    // byte-identical frames whatever any of the three say, so reading them
    // could only reintroduce a divergence. The measurements are in
    // renderDerivedProduct.
    out->focal = g_kUsdDefaultFocalLength;
    out->h_aperture = g_kUsdDefaultHorizontalAperture;
    (void)pulledScalarF32(scene, out->camera_path, "focalLength", &out->focal);
    (void)pulledScalarF32(scene, out->camera_path, "horizontalAperture", &out->h_aperture);
    out->meters_per_unit = pulledMetersPerUnit(scene);
    return true;
}

// Render one derived product on the (already sealed + hierarchy-updated)
// backend stage at `render_ordinal`: camera from the bound Camera prim's
// composed world xform + the focalLength/apertures captured at derive time,
// one ovgl_render_frame, publish the rasterizer-expressible vars — exactly
// {"LdrColor", "DistanceToImagePlaneSD"}. Non-expressible vars (HdrColor,
// NormalSD, PointCloud, ...) are ABSENT — an honest, documented gap.
bool renderDerivedProduct(OvglBackend* be,
                          const DerivedProduct& product,
                          ovstage_ordinal_t render_ordinal,
                          ovstage_ordinal_t scene_ordinal,
                          const PresentRequest* present_request,
                          std::string* err)
{
    const DerivedProduct& effective_product = product;
    std::vector<uint8_t> camera_bytes;
    bool has_camera_matrix =
        readAttributeHost(be, render_ordinal, effective_product.camera_path, g_kMirrorWorldTransform, camera_bytes) &&
        camera_bytes.size() == 16 * sizeof(double);
    if (!has_camera_matrix && effective_product.camera_path.find('/', 1) == std::string::npos)
    {
        // A top-level camera created through public OVStage edits has no
        // ancestor transform, so local and world space are identical. Some
        // OVStage hierarchy implementations do not emit a derived world row
        // for a prim first introduced after population; retain interactive
        // camera support without guessing for nested cameras.
        camera_bytes.clear();
        has_camera_matrix = readAttributeHost(be, render_ordinal, effective_product.camera_path,
                                              g_kMirrorLocalTransform, camera_bytes) &&
                            camera_bytes.size() == 16 * sizeof(double);
    }
    if (!has_camera_matrix)
    {
        *err = "camera prim '" + effective_product.camera_path + "' has no composed transform";
        return false;
    }
    double cam[16];
    std::memcpy(cam, camera_bytes.data(), sizeof(cam));
    const double eye[3] = { cam[12], cam[13], cam[14] };
    const double fwd[3] = { -cam[8], -cam[9], -cam[10] };
    const double up[3] = { cam[4], cam[5], cam[6] };
    const double target[3] = { eye[0] + fwd[0], eye[1] + fwd[1], eye[2] + fwd[2] };
    const float focal = effective_product.focal;
    const float h_aperture = effective_product.h_aperture;
    if (!(focal > 0.0f) || !(h_aperture > 0.0f))
    {
        *err = "camera prim has non-positive focal length or aperture";
        return false;
    }
    // Aperture-axis precedence, as official ovrtx 0.4 and GfCamera do it:
    // horizontalAperture fixes the horizontal field of view and the VERTICAL
    // aperture is conformed to the render product's aspect (UsdRender's
    // "adjustApertureHeight", CameraUtilMatchHorizontally). The camera's own
    // verticalAperture is never consumed.
    //
    // This used to prefer an authored verticalAperture and conform only as a
    // fallback — which inverted the model, because the fallback was dead
    // code: population resolves the UsdGeomCamera SCHEMA DEFAULT
    // verticalAperture = 15.2908 for every typed camera, so every real scene
    // took the "authored" branch and horizontalAperture was never read at
    // all. On the repo's own robot-ovui/assets/robot_ground_scene.usda
    // (focalLength 42, horizontalAperture 36, no verticalAperture, square
    // 640x640 product) that rendered a 20.634-degree vertical fov against
    // official's 46.397: 100.00% of pixels differing, MAE 157.1 in the file
    // lane and 160.6 in the attach lane, silhouette IoU 0.547.
    //
    // Conforming unconditionally — rather than only when verticalAperture is
    // unauthored — is both what official does and the only honest option
    // here, since pulledScalarF32 reports success for a schema default just
    // as it does for an authored value and so cannot tell the two apart.
    // Measured on the official wheel, attach lane, same scene: overlaying
    // verticalAperture = 36, 72 or 100 yields three byte-identical PNGs, as
    // does overlaying the product's pixelAspectRatio = 2 or
    // aspectRatioConformPolicy = "adjustApertureWidth" (geometry top edge
    // y=252 and silhouette IoU >= 0.998 in all five; the <= 0.093 MAE
    // residual is official's own frame-to-frame sampling noise). Under USD's
    // schema-default "expandAperture" policy a verticalAperture of 100 would
    // have widened the horizontal aperture to match and opened the frame to a
    // 100-degree vertical fov, so official implements none of that policy
    // machinery — it conforms the height and ignores the rest.
    const double vert_ap =
        static_cast<double>(h_aperture) * (static_cast<double>(effective_product.height) / effective_product.width);
    const double fov_y = 2.0 * std::atan(vert_ap / (2.0 * static_cast<double>(focal)));
    if (setCamera(be, eye, target, up, fov_y, effective_product.width, effective_product.height) != 0)
    {
        *err = std::string("attached camera: ") + g_backendError;
        return false;
    }

    // Optional dataWindowNDC crop: publish or present only the window's pixel rows/cols.
    int x_lo = 0, y_lo = 0;
    int x_hi = effective_product.width, y_hi = effective_product.height;
    if (effective_product.has_data_window)
    {
        x_lo = static_cast<int>(std::lround(effective_product.data_window[0] * effective_product.width));
        y_lo = static_cast<int>(std::lround(effective_product.data_window[1] * effective_product.height));
        x_hi = static_cast<int>(std::lround(effective_product.data_window[2] * effective_product.width));
        y_hi = static_cast<int>(std::lround(effective_product.data_window[3] * effective_product.height));
        x_lo = std::max(0, std::min(x_lo, effective_product.width - 1));
        y_lo = std::max(0, std::min(y_lo, effective_product.height - 1));
        x_hi = std::max(x_lo + 1, std::min(x_hi, effective_product.width));
        y_hi = std::max(y_lo + 1, std::min(y_hi, effective_product.height));
    }

    if (present_request)
    {
        if (std::find(effective_product.var_names.begin(), effective_product.var_names.end(), "LdrColor") ==
            effective_product.var_names.end())
        {
            *err = "The configured RenderProduct has no LdrColor RenderVar output";
            return false;
        }
        if (ovgl_set_depth_capture(be->renderer, 0).status != 0)
        {
            ovx_string_t e = ovgl_get_last_error();
            *err = std::string("ovgl_set_depth_capture: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0);
            return false;
        }
        const ovgl_result_t result =
            ovgl_render_to_fbo(be->renderer, scene_ordinal, present_request->framebuffer, effective_product.width,
                               effective_product.height, x_lo, y_lo, x_hi, y_hi, present_request->destinationWidth,
                               present_request->destinationHeight, present_request->overlayText);
        if (result.status != 0)
        {
            const ovx_string_t e = ovgl_get_last_error();
            *err = std::string("attached ovgl_render_to_fbo: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0);
            return false;
        }
        OvglBackend::AttachedProduct out;
        out.path = effective_product.path;
        out.width = x_hi - x_lo;
        out.height = y_hi - y_lo;
        be->attached_products.push_back(std::move(out));
        return true;
    }

    // Depth capture is armed per product — exactly when its orderedVars
    // request the depth var, so LdrColor-only products pay nothing. The
    // capture rides inside the SAME ovgl_render_frame call that produces
    // the color buffer (same frame, same ordinal), on every step mode:
    // kFull / kColumnsOnly / kXformOnly / kUnchanged all render through
    // here, so depth refreshes on the color cadence by construction.
    const bool want_depth = std::find(effective_product.var_names.begin(), effective_product.var_names.end(),
                                      g_kDepthVarName) != effective_product.var_names.end();
    if (ovgl_set_depth_capture(be->renderer, want_depth ? 1 : 0).status != 0)
    {
        ovx_string_t e = ovgl_get_last_error();
        *err = std::string("ovgl_set_depth_capture: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0);
        return false;
    }

    be->frame.assign(static_cast<size_t>(effective_product.width) * effective_product.height * 4, 0);
    ovgl_result_t rr = ovgl_render_frame(
        be->renderer, scene_ordinal, be->frame.data(), effective_product.width, effective_product.height);
    if (rr.status != 0)
    {
        ovx_string_t e = ovgl_get_last_error();
        *err = std::string("attached ovgl_render_frame: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0);
        return false;
    }

    // Fail closed: a requested depth var either arrives from this exact
    // frame at this exact resolution or the whole step fails — never a
    // fabricated or stale buffer.
    const float* depth_su = nullptr;
    int depth_w = 0, depth_h = 0;
    if (want_depth)
    {
        ovgl_result_t dv = ovgl_get_depth_view(be->renderer, &depth_su, &depth_w, &depth_h);
        if (dv.status != 0 || !depth_su || depth_w != effective_product.width || depth_h != effective_product.height)
        {
            ovx_string_t e = ovgl_get_last_error();
            *err = std::string("depth var readback: ") + std::string(e.ptr ? e.ptr : "", e.ptr ? e.length : 0);
            return false;
        }
    }

    OvglBackend::AttachedProduct out;
    out.path = effective_product.path;
    out.width = effective_product.width;
    out.height = effective_product.height;
    // The image-var filter: exactly {LdrColor, DistanceToImagePlaneSD} are
    // expressible; every other requested var stays ABSENT (fail-closed on
    // the absent side — the contract tests pin both directions).
    const double mpu = want_depth ? effective_product.meters_per_unit : 0.0;
    for (const std::string& name : effective_product.var_names)
    {
        const bool is_color = (name == "LdrColor");
        const bool is_depth = (name == g_kDepthVarName);
        if (!is_color && !is_depth)
            continue;
        OvglBackend::AttachedVar var;
        var.name = name;
        var.w = x_hi - x_lo;
        var.h = y_hi - y_lo;
        if (is_color)
        {
            var.format = BE_VAR_FMT_RGBA8;
            if (!effective_product.has_data_window)
            {
                var.data = be->frame;
            }
            else
            {
                var.data.resize(static_cast<size_t>(var.w) * var.h * 4);
                for (int y = 0; y < var.h; ++y)
                {
                    const uint8_t* src =
                        be->frame.data() + (static_cast<size_t>(y + y_lo) * effective_product.width + x_lo) * 4;
                    std::memcpy(
                        var.data.data() + static_cast<size_t>(y) * var.w * 4, src, static_cast<size_t>(var.w) * 4);
                }
            }
        }
        else
        {
            // Depth: ovgl's stage-unit image-plane distances scaled to the
            // official meter payload (+inf no-hit passes through the scale
            // untouched). The dataWindowNDC crop clips depth exactly like
            // color — same window, same rows.
            var.format = BE_VAR_FMT_F32_DEPTH;
            var.data.resize(static_cast<size_t>(var.w) * var.h * sizeof(float));
            float* dst = reinterpret_cast<float*>(var.data.data());
            for (int y = 0; y < var.h; ++y)
            {
                const float* src = depth_su + static_cast<size_t>(y + y_lo) * effective_product.width + x_lo;
                for (int x = 0; x < var.w; ++x)
                    dst[static_cast<size_t>(y) * var.w + x] = static_cast<float>(src[x] * mpu);
            }
        }
        out.vars.push_back(std::move(var));
    }
    be->attached_products.push_back(std::move(out));
    return true;
}

} // namespace

static int stepAttached(OvglBackend* be,
                        ovstage_ordinal_t ordinal,
                        const char* const* render_product_paths,
                        size_t num_products,
                        const PresentRequest* present_request,
                        char* err,
                        size_t errlen)
{
    auto fail = [&](const std::string& m) -> int
    {
        setBackendError(m);
        if (err && errlen)
            std::snprintf(err, errlen, "%s", m.c_str());
        return 1;
    };
    if (!be)
        return 1;
    if (!be->ext)
        return fail("no ovstage is attached");
    if (!render_product_paths || num_products == 0)
        return fail("render product set is empty");
    for (size_t i = 0; i < num_products; ++i)
    {
        if (!render_product_paths[i] || !render_product_paths[i][0])
            return fail("render product path is empty");
    }
    path_dictionary_instance_t* ext_dict = ovstage_get_path_dictionary(be->ext);
    if (!ext_dict)
        return fail("attached stage has no path dictionary");

    const double t_pull0 = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;

    // 0. Classify this step against the retained committed snapshot (perf:
    //    the 2026-07-18 rig profile measured ~2.8 s/step, >99% of it spent
    //    re-pulling, re-mirroring, and re-building state that had not
    //    changed). Committed data at or below the attached stage's write
    //    floor is sealed and immutable (ovstage_attribute_meta_t contract),
    //    and the shim rejects step ordinals above that floor, so:
    //      - a step at the SAME ordinal as the retained pull is byte-
    //        identical by definition (kUnchanged, zero reads);
    //      - a step at a LATER ordinal revalidates public query membership,
    //        then reads only the changed ordinal window. An empty effective
    //        change is kUnchanged; transform-only changes use kXformOnly;
    //        other value changes use kDelta. Membership changes or public
    //        API failures fall back to a full resynchronization.
    enum StepMode
    {
        kFull,
        kDelta,
        kXformOnly,
        kJournalXform,
        kUnchanged
    };
    StepMode mode = kFull;
    TransformJournal transformJournal;
    PulledScene delta;
    PulledScene sourceDelta;
    PulledScene nextSource;
    std::set<std::string> nextSourceAttributes;
    SceneGraphInstanceIndex nextSceneGraphInstances;
    bool sourceCacheUpdated = false;
    // kDelta working set: the EFFECTIVE (non-restamp) window changes per
    // prim, captured during classification — after the fold below the
    // retained snapshot equals the window, so the diff must be recorded now.
    std::map<std::string, std::vector<std::string>> effective_changes;
    const ovstage_ordinal_t end_ordinal = static_cast<ovstage_ordinal_t>(ordinal);
    const bool cache_was_valid = be->attached_pull_valid;
    be->attached_pull_valid = false; // re-established only on full success
    if (cache_was_valid && end_ordinal >= be->attached_pull_ordinal)
    {
        if (end_ordinal == be->attached_pull_ordinal)
        {
            mode = kUnchanged;
        }
        else
        {
            const bool journalContinuous = readTransformJournal(be->ext, ext_dict, end_ordinal, &transformJournal) &&
                                           transformJournal.baseOrdinal <= be->attached_pull_ordinal &&
                                           be->attached_pull_ordinal < transformJournal.ordinal &&
                                           (be->transform_journal_generation == 0 ||
                                            be->transform_journal_generation == transformJournal.generation);
            if (journalContinuous)
            {
                mode = kJournalXform;
            }
            else if (!be->pulled_transform_dirty)
            {
                // Public OVStage has no wildcard change stream. Revalidate
                // membership, then range-read only columns authored after the
                // retained sealed snapshot. A topology change takes the full
                // resynchronization path below.
                const PublicPullStatus pullStatus = pullPublicChanges(
                    be, end_ordinal, &delta, &sourceDelta, &nextSource, &nextSourceAttributes, &nextSceneGraphInstances);
                if (pullStatus != PublicPullStatus::eSuccess)
                {
                    delta.clear();
                    mode = kFull;
                }
                else
                {
                    // A window column whose payload is byte-identical to the retained
                    // snapshot is a no-op restamp (the engine restamps the reserved
                    // usd-prim-type/usd-schemas rows when a prim gains a column, and
                    // producers may rewrite unchanged values); the full path would
                    // mirror the same bytes again, so skipping it cannot change the
                    // mirror's committed view.
                    bool any_effective = false;
                    bool xform_only = true;
                    if (isProfilingEnabled())
                        std::fprintf(stderr, "OVGLBEPROF delta prims=%zu range=[%llu,%llu]\n", delta.size(),
                                     static_cast<unsigned long long>(be->attached_pull_ordinal + 1),
                                     static_cast<unsigned long long>(end_ordinal));
                    for (const auto& prim : delta)
                    {
                        const auto known_prim = be->attached_pull.find(prim.first);
                        const bool new_prim = !be->mirror_pulled.count(prim.first);
                        for (const auto& col : prim.second)
                        {
                            if (known_prim != be->attached_pull.end())
                            {
                                const auto known = known_prim->second.find(col.first);
                                if (known != known_prim->second.end() && known->second.deleted == col.second.deleted &&
                                    known->second.is_array == col.second.is_array &&
                                    known->second.semantic == col.second.semantic &&
                                    known->second.dtype.code == col.second.dtype.code &&
                                    known->second.dtype.bits == col.second.dtype.bits &&
                                    known->second.dtype.lanes == col.second.dtype.lanes &&
                                    known->second.bytes == col.second.bytes)
                                {
                                    continue; // no-op restamp
                                }
                            }
                            if (isProfilingEnabled() && delta.size() <= 8)
                                std::fprintf(stderr, "OVGLBEPROF delta change path=%s attr=%s ordinal=%llu\n",
                                             prim.first.c_str(), col.first.c_str(),
                                             static_cast<unsigned long long>(col.second.ordinal));
                            any_effective = true;
                            if (new_prim || col.second.deleted || !isTransformColumn(col.first))
                                xform_only = false;
                            effective_changes[prim.first].push_back(col.first);
                        }
                    }
                    mode = !any_effective ? kUnchanged : xform_only ? kXformOnly : kDelta;
                    sourceCacheUpdated = true;
                } /* public snapshot diff succeeded */
            }
        }
    }

    // A direct transform batch advances the resident renderer without
    // advancing the private mirror. General deltas cannot safely build on that
    // stale mirror, so recover through the cold/full path before folding them.
    if (be->mirror_transform_dirty && mode == kDelta)
    {
        be->attached_pull_valid = false;
        return stepAttached(be, ordinal, render_product_paths, num_products, present_request, err, errlen);
    }

    // 1. Pull the committed state <= ordinal through the public vtable
    //    (per-STEP work — one pull no matter how many products render).
    //    Fast modes reuse the retained snapshot and fold the ordinal-window
    //    changes into it; kUnchanged has no effective value change.
    PulledScene scene;
    PulledScene sourceScene;
    std::set<std::string> sourceAttributes;
    SceneGraphInstanceIndex sceneGraphInstances;
    if (mode == kFull)
    {
        if (!pullScene(be->ext, end_ordinal, &scene, &sourceScene, &sourceAttributes, &sceneGraphInstances))
            return fail(g_backendError);
    }
    else
    {
        // Fold the window into the retained snapshot. Tombstones are
        // canonicalized to absence so the cache has the same shape as a
        // cold pull.
        applyPulledChanges(&be->attached_pull, delta);
    }
    const PulledScene& view = (mode == kFull) ? scene : be->attached_pull;

    const double t_mirror0 = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;

    // 2. Derive every stage-authored product up-front.
    std::vector<DerivedProduct> derived(num_products);
    for (size_t i = 0; i < num_products; ++i)
    {
        std::string derr;
        if (!deriveRenderProduct(view, ext_dict, render_product_paths[i], "attached stage", &derived[i], &derr))
            return fail(derr);
    }

    std::set<std::string> cameraPaths;
    for (const DerivedProduct& product : derived)
        cameraPaths.insert(product.camera_path);
    const bool cameraOnlyTransform =
        mode == kXformOnly && !effective_changes.empty() &&
        std::all_of(effective_changes.begin(), effective_changes.end(),
                    [&](const auto& change) { return cameraPaths.count(change.first) != 0; });

    ovstage_ordinal_t render_ordinal = be->attached_render_ordinal;
    ovstage_ordinal_t scene_ordinal = be->attached_scene_ordinal;
    bool directTransformApplied = false;
    if (mode == kJournalXform && be->attached_light_paths_valid &&
        !pathsAffectTargets(transformJournal.paths, cameraPaths) &&
        !pathsAffectTargets(transformJournal.paths, be->attached_light_paths))
    {
        std::vector<double> worldMatrices;
        if (composeJournalMeshWorlds(be, view, transformJournal, &worldMatrices))
        {
            const size_t meshCount = worldMatrices.size() / 16;
            if (ovgl_refresh_transform_batch(
                    be->renderer, end_ordinal, worldMatrices.empty() ? nullptr : worldMatrices.data(), meshCount)
                    .status == 0)
            {
                directTransformApplied = true;
                be->mirror_transform_dirty = true;
                be->pulled_transform_dirty = true;
                scene_ordinal = end_ordinal;
            }
        }
    }
    if (mode == kJournalXform && !directTransformApplied)
    {
        be->attached_pull_valid = false;
        return stepAttached(be, ordinal, render_product_paths, num_products, present_request, err, errlen);
    }
    if (mode == kXformOnly && be->attached_light_paths_valid && !changesAffectPaths(effective_changes, cameraPaths) &&
        !changesAffectPaths(effective_changes, be->attached_light_paths))
    {
        std::vector<double> worldMatrices;
        if (composeResidentMeshWorlds(be->renderer, view, &worldMatrices))
        {
            const size_t meshCount = worldMatrices.size() / 16;
            if (ovgl_refresh_transform_batch(
                    be->renderer, end_ordinal, worldMatrices.empty() ? nullptr : worldMatrices.data(), meshCount)
                    .status == 0)
            {
                directTransformApplied = true;
                be->mirror_transform_dirty = true;
                scene_ordinal = end_ordinal;
            }
        }
    }

    // If the mirror was already stale, an unsupported/failed direct batch
    // cannot use the incremental mirror transform path. Re-enter once with the
    // retained cache invalidated; classification then selects kFull.
    if (mode == kXformOnly && !directTransformApplied && be->mirror_transform_dirty)
    {
        be->attached_pull_valid = false;
        return stepAttached(be, ordinal, render_product_paths, num_products, present_request, err, errlen);
    }

    if (mode == kXformOnly && !directTransformApplied)
    {
        // Transform-only mirror: retain local transforms as hierarchy inputs
        // and apply authoritative source worlds after hierarchy composition.
        const ovstage_ordinal_t o = be->cur_ordinal;
        if (o >= std::numeric_limits<ovstage_ordinal_t>::max() - 2)
            return fail("mirror ordinal space exhausted");
        MirrorBatches worldOverrideBatches;
        for (const auto& prim : delta)
        {
            const auto merged = be->attached_pull.find(prim.first);
            if (merged == be->attached_pull.end())
                continue;
            const auto& cols = merged->second;
            const PulledColumn* world = pickSourceWorld(cols);
            const PulledColumn* local = pickLocalAuthority(cols);
            if (local)
            {
                if (local->bytes.size() != 16 * sizeof(double))
                    return fail("transform for '" + prim.first + "' is not a 4x4 double matrix");
                if (!mirrorWriteColumn(be, o, prim.first, g_kMirrorLocalTransform, local->bytes.data(),
                                       local->bytes.size(), DLDataType{ kDLFloat, 64, 16 }, OVSTAGE_SEMANTIC_MATRIX,
                                       false))
                    return fail(g_backendError);
                be->mirror_cols[prim.first].insert(g_kMirrorLocalTransform);
            }
            const PulledColumn* worldSeed = world ? world : local;
            if (worldSeed)
            {
                if (worldSeed->bytes.size() != 16 * sizeof(double))
                    return fail("world transform seed for '" + prim.first + "' is not a 4x4 double matrix");
                if (!mirrorWriteColumn(be, o, prim.first, g_kMirrorWorldTransform, worldSeed->bytes.data(),
                                       worldSeed->bytes.size(), DLDataType{ kDLFloat, 64, 16 }, OVSTAGE_SEMANTIC_MATRIX,
                                       false))
                    return fail(g_backendError);
            }
            if (world && !addMatrixMirrorBatch(&worldOverrideBatches, prim.first, g_kMirrorWorldTransform, *world))
                return fail(g_backendError);
        }
        if (!sealOrdinal(be, o))
            return fail("attached seal(local) failed");
        if (!computeWorldXforms(be->stage, o, o + 1, "attached compute hierarchy"))
            return fail(g_backendError);
        if (!mirrorWriteBatches(be, o + 1, worldOverrideBatches))
            return fail(g_backendError);
        if (!sealOrdinal(be, o + 1))
            return fail("attached seal(world) failed");
        be->cur_ordinal = o + 2;
        render_ordinal = o + 1;
        if (!cameraOnlyTransform)
        {
            // Cheap-path hint: refresh the cached ovgl scene's worldMatrix
            // column at the new ordinal so render_frame reuses its
            // geometry/material caches. Failure is non-fatal —
            // render_frame rebuilds from the sealed snapshot whenever the
            // ordinal it consumes was not published by a successful
            // refresh, which stays the correctness baseline.
            (void)ovgl_refresh_transforms(be->renderer, render_ordinal);
            scene_ordinal = render_ordinal;
        }
        // A camera-only edit has already been mirrored and composed above,
        // but it does not invalidate resident meshes, materials, lights, or
        // bounds. renderDerivedProduct reads the camera at render_ordinal
        // and deliberately renders OVGL's previous scene_ordinal cache.
    }

    if (mode == kDelta)
    {
        // Delta mirror (D4, DESIGN_CHANGE_TRACKING.md): UPSERT exactly the
        // effective window columns, tombstone exactly the window's deletes,
        // and bring new prims into existence — replacing the full re-pull +
        // full mirror rewrite (the measured ~2.8 s/step term on large
        // stages) with O(changed columns) work for EVERY window the
        // classifier can see without a membership change. kFull remains for
        // cold attach, membership changes, and refused public API reads.
        // The renderer side stays honest by construction: geometry/topology
        // windows get no refresh hint, so render_frame's ordinal-moved
        // rebuild re-reads the mirror — and the per-mesh GL content cache
        // makes that rebuild re-upload only what actually changed.
        const ovstage_ordinal_t o = be->cur_ordinal;
        if (o >= std::numeric_limits<ovstage_ordinal_t>::max() - 2)
            return fail("mirror ordinal space exhausted");
        // Targeted-refresh gate, in two parts.
        //   refreshable — every non-transform change is a shader/light input
        //     value edit ("inputs:*" — USD preview-surface and UsdLux both
        //     spell their value inputs this way). With no topology in the
        //     window, the renderer can publish the new ordinal in place.
        //   needs_materials — an "inputs:*" edit was seen, so the material
        //     network must be re-resolved before the publish. A transform-only
        //     window does not re-upload materials (that re-creates every GL
        //     texture; measured ~440 ms on the franka
        //     factory scene, 78% of a rebuild).
        // Anything else keeps the full-rebuild correctness baseline.
        bool refreshable = true;
        bool needs_materials = false;
        bool any_topology = false;
        MirrorBatches worldOverrideBatches;
        for (const auto& changed : effective_changes)
        {
            const std::string& path = changed.first;
            const auto merged = be->attached_pull.find(path);
            const bool was_mirrored = be->mirror_pulled.count(path) != 0;
            if (merged == be->attached_pull.end())
            {
                // Whole-prim delete: the canonicalized fold erased the prim
                // (no live columns remain). Tombstone the mirror's copy and
                // drop the existence bookkeeping — exactly what the full
                // pass's disappearance diff would conclude.
                any_topology = true;
                if (was_mirrored)
                {
                    if (!mirrorDeletePrim(be, o, path))
                        return fail(g_backendError);
                    be->mirror_pulled.erase(path);
                    be->mirror_cols.erase(path);
                }
                continue;
            }
            const auto& cols = merged->second;
            if (!was_mirrored)
                any_topology = true;
            bool touched_transform = false;
            std::vector<std::string> dead;
            for (const char* metadata : { "usd-prim-type", "usd-schemas" })
            {
                if (std::find(changed.second.begin(), changed.second.end(), metadata) == changed.second.end())
                    continue;
                const auto column = cols.find(metadata);
                if (column == cols.end() || column->second.deleted)
                    continue;
                if (!mirrorPulledColumn(be, o, path, metadata, column->second, ext_dict))
                    return fail(g_backendError);
                be->mirror_cols[path].insert(metadata);
            }
            for (const std::string& attr : changed.second)
            {
                if (isTransformColumn(attr))
                {
                    touched_transform = true; // handled once below
                    continue;
                }
                if (attr == "usd-prim-type" || attr == "usd-schemas")
                    continue; // metadata was written first to establish new prims
                const auto cit = cols.find(attr);
                if (cit == cols.end())
                {
                    // Per-attribute tombstone: the canonicalized fold erased
                    // this column while the prim stays live. Mirror it as a
                    // column delete — only for columns the mirror actually
                    // carries; the reserved rows are engine-rejected and
                    // stay live for as long as the prim does.
                    refreshable = false;
                    if (attr == "usd-prim-type" || attr == "usd-schemas")
                        continue;
                    const auto prev = be->mirror_cols.find(path);
                    if (prev != be->mirror_cols.end() && prev->second.count(attr))
                    {
                        dead.push_back(attr);
                        prev->second.erase(attr);
                    }
                    continue;
                }
                if (attr.rfind("inputs:", 0) == 0)
                    needs_materials = true;
                else
                    refreshable = false;
                if (cit->second.deleted)
                    continue; /* defensive: canonicalized above */
                const PulledColumn& c = cit->second;
                if (!mirrorPulledColumn(be, o, path, attr, c, ext_dict))
                    return fail(g_backendError);
                be->mirror_cols[path].insert(attr);
            }
            if (touched_transform)
            {
                const PulledColumn* world = pickSourceWorld(cols);
                const PulledColumn* local = pickLocalAuthority(cols);
                if (local)
                {
                    if (local->bytes.size() != 16 * sizeof(double))
                        return fail("transform for '" + path + "' is not a 4x4 double matrix");
                    if (!mirrorWriteColumn(be, o, path, g_kMirrorLocalTransform, local->bytes.data(), local->bytes.size(),
                                           DLDataType{ kDLFloat, 64, 16 }, OVSTAGE_SEMANTIC_MATRIX, false))
                        return fail(g_backendError);
                    be->mirror_cols[path].insert(g_kMirrorLocalTransform);
                }
                const PulledColumn* worldSeed = world ? world : local;
                if (worldSeed)
                {
                    if (worldSeed->bytes.size() != 16 * sizeof(double))
                        return fail("world transform seed for '" + path + "' is not a 4x4 double matrix");
                    if (!mirrorWriteColumn(be, o, path, g_kMirrorWorldTransform, worldSeed->bytes.data(),
                                           worldSeed->bytes.size(), DLDataType{ kDLFloat, 64, 16 },
                                           OVSTAGE_SEMANTIC_MATRIX, false))
                        return fail(g_backendError);
                }
                if (world && !addMatrixMirrorBatch(&worldOverrideBatches, path, g_kMirrorWorldTransform, *world))
                    return fail(g_backendError);
                if (!local && !world)
                {
                    // Every transform authority source was deleted this
                    // window: the mirror's derived transform must go too,
                    // exactly as the full pass's now_live diff would decide.
                    refreshable = false;
                    const auto prev = be->mirror_cols.find(path);
                    if (prev != be->mirror_cols.end() && prev->second.count(g_kMirrorLocalTransform))
                    {
                        dead.push_back(g_kMirrorLocalTransform);
                        prev->second.erase(g_kMirrorLocalTransform);
                    }
                }
            }
            if (!dead.empty() && !mirrorDeleteColumns(be, o, path, dead))
                return fail(g_backendError);
            if (!was_mirrored)
            {
                // New prim: its columns all arrived in this window (first
                // writes are always effective) and were mirrored above; add
                // the existence bookkeeping the full pass keeps.
                be->mirror_pulled.insert(path);
                be->note(path);
            }
        }
        if (!sealOrdinal(be, o))
            return fail("attached seal(local) failed");
        if (!computeWorldXforms(be->stage, o, o + 1, "attached compute hierarchy"))
            return fail(g_backendError);
        if (!mirrorWriteBatches(be, o + 1, worldOverrideBatches))
            return fail(g_backendError);
        if (!sealOrdinal(be, o + 1))
            return fail("attached seal(world) failed");
        be->cur_ordinal = o + 2;
        render_ordinal = o + 1;
        scene_ordinal = render_ordinal;
        if (refreshable && !any_topology)
        {
            // Targeted refresh: materials first when (and only when) a shader
            // /light input actually changed (CPU-side re-resolution, no
            // publish), then refresh_transforms as the single publish point
            // (it also re-reads lights, covering UsdLux inputs:* edits). If
            // either fails the ordinal stays unpublished and render_frame
            // takes the full-rebuild correctness baseline for this frame.
            if (!needs_materials || ovgl_refresh_materials(be->renderer, render_ordinal).status == 0)
                (void)ovgl_refresh_transforms(be->renderer, render_ordinal);
        }
    }

    if (mode != kFull)
    {
        // 5+6 (fast modes). Per PRODUCT: camera + one render + publish vars,
        // identical to the full path below but against the retained snapshot.
        const double t_render0 = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
        be->attached_products.clear();
        for (const DerivedProduct& product : derived)
        {
            std::string rerr;
            if (!renderDerivedProduct(be, product, render_ordinal, scene_ordinal, present_request, &rerr))
            {
                be->attached_products.clear();
                return fail(rerr);
            }
        }
        be->attached_pull_ordinal = end_ordinal;
        be->attached_render_ordinal = render_ordinal;
        be->attached_scene_ordinal = scene_ordinal;
        if (sourceCacheUpdated)
        {
            if (nextSource.empty())
                applyPulledChanges(&be->attached_source_pull, sourceDelta);
            else
                be->attached_source_pull = std::move(nextSource);
            be->attached_source_attributes = std::move(nextSourceAttributes);
            be->attached_scene_graph_instances = std::move(nextSceneGraphInstances);
        }
        if (mode == kDelta)
        {
            be->attached_light_paths_valid = collectLightPaths(be->attached_pull, ext_dict, &be->attached_light_paths);
        }
        be->attached_pull_valid = true;
        if (isProfilingEnabled())
        {
            const double t_end = getCurrentTimeMilliseconds();
            std::fprintf(stderr, "OVGLBEPROF mode=%s pull=%.3f mirror+seal=%.3f render=%.3f total=%.3f\n",
                         mode == kUnchanged    ? "unchanged" :
                         mode == kXformOnly    ? (directTransformApplied ? "direct-xform" : "xform") :
                         mode == kJournalXform ? "journal-xform" :
                                                 "delta",
                         t_mirror0 - t_pull0, t_render0 - t_mirror0, t_end - t_render0, t_end - t_pull0);
        }
        return 0;
    }

    // 3. Mirror the pulled state into the backend's own stage.
    const ovstage_ordinal_t o = be->cur_ordinal;
    if (o >= std::numeric_limits<ovstage_ordinal_t>::max() - 2)
        return fail("mirror ordinal space exhausted");

    std::set<std::string> pulled_now;
    std::map<std::string, std::set<std::string>> mirror_cols_now;
    MirrorBatches metadata_batches;
    MirrorBatches data_batches;
    MirrorBatches worldOverrideBatches;
    std::map<std::string, std::vector<std::string>> dead_columns;
    for (const auto& prim : scene)
    {
        const std::string& path = prim.first;
        // Local matrices remain hierarchy inputs. A source world matrix is
        // also retained as an output override after hierarchy composition;
        // treating the two roles as mutually exclusive collapses populated
        // transforms because the mirror hierarchy then sees identity locals.
        const PulledColumn* world = pickSourceWorld(prim.second);
        const PulledColumn* local = pickLocalAuthority(prim.second);
        bool any_live = false;
        std::set<std::string> now_live;

        // Metadata must establish the prim before any data or transform write.
        // The PoC store tolerated later metadata UPSERTs, while official
        // OVStage's typed query index is fixed by the prim's initial bucket.
        for (const char* metadata : { "usd-prim-type", "usd-schemas" })
        {
            const auto column = prim.second.find(metadata);
            if (column == prim.second.end() || column->second.deleted)
                continue;
            PreparedMirrorColumn prepared;
            if (!prepareMirrorColumn(be, metadata, column->second, ext_dict, &prepared))
                return fail(g_backendError);
            addMirrorBatch(&metadata_batches, path, metadata, std::move(prepared));
            any_live = true;
            now_live.insert(metadata);
        }
        if (world)
        {
            if (!addMatrixMirrorBatch(&data_batches, path, g_kMirrorWorldTransform, *world) ||
                !addMatrixMirrorBatch(&worldOverrideBatches, path, g_kMirrorWorldTransform, *world))
                return fail(g_backendError);
            // worldMatrix stays a derived column in the diff bookkeeping (not now_live).
        }
        // Columns this pass keeps LIVE in the mirror under their own name
        // (verbatim-mirrored attrs + the derived localMatrix below). Diffed
        // against the previous pass's set after the writes: the mirror is an
        // accumulating UPSERT store, so a column that dropped out of this
        // pull — deleted on the attached stage (a full range pull coalesces
        // the tombstone away, the column is simply ABSENT) — must be
        // tombstoned explicitly or every warmed renderer keeps rendering it
        // forever while a fresh replay shows the deletion (fast-path
        // adversary F2, 2026-07-19).
        for (const auto& col : prim.second)
        {
            const std::string& attr = col.first;
            const PulledColumn& c = col.second;
            if (c.deleted)
                continue; // not live: drops out of now_live -> tombstoned below
            any_live = true;
            if (isTransformColumn(attr))
                continue; // transforms handled below; raw ops stay flattened
            if (attr == "usd-prim-type" || attr == "usd-schemas")
                continue; // metadata was written first to establish the typed bucket
            now_live.insert(attr);
            PreparedMirrorColumn prepared;
            if (!prepareMirrorColumn(be, attr, c, ext_dict, &prepared))
                return fail(g_backendError);
            addMirrorBatch(&data_batches, path, attr, std::move(prepared));
        }
        if (local)
        {
            if (!addMatrixMirrorBatch(&data_batches, path, g_kMirrorLocalTransform, *local))
                return fail(g_backendError);
            // Seed the official hierarchy model's output column. USD
            // population does this automatically; generic mirror writes do not.
            if (!world && !addMatrixMirrorBatch(&data_batches, path, g_kMirrorWorldTransform, *local))
                return fail(g_backendError);
            now_live.insert(g_kMirrorLocalTransform);
            any_live = true;
        }
        if (any_live)
        {
            // Tombstone the mirrored columns this pull no longer carries,
            // AFTER the live UPSERTs above so a first-seen prim already
            // exists in the mirror. A prim with no live columns at all is
            // handled by the whole-prim disappearance pass below instead.
            // The reserved rows never enter `dead`: the attached engine
            // rejects their per-attribute delete, so they stay live for as
            // long as the prim does (and the mirror-side per-attribute form
            // would be rejected the same way).
            const auto prev = be->mirror_cols.find(path);
            if (prev != be->mirror_cols.end())
            {
                for (const std::string& attr : prev->second)
                {
                    if (now_live.count(attr))
                        continue;
                    if (attr == "usd-prim-type" || attr == "usd-schemas")
                        continue;
                    dead_columns[path].push_back(attr);
                }
            }
            mirror_cols_now[path] = std::move(now_live);
            pulled_now.insert(path);
            be->note(path);
        }
    }
    // Establish every prim's typed metadata first, then submit one write per
    // compatible attribute bucket instead of one query/write/wait per
    // (prim, attribute). This changes cold mirror cost from O(columns) public
    // operations to O(column signatures), while preserving the same rows.
    if (!mirrorWriteBatches(be, o, metadata_batches) || !mirrorWriteBatches(be, o, data_batches))
        return fail(g_backendError);
    for (const auto& dead : dead_columns)
        if (!mirrorDeleteColumns(be, o, dead.first, dead.second))
            return fail(g_backendError);
    // Prims that vanished from the attached stage get tombstoned in the mirror.
    for (const std::string& gone : be->mirror_pulled)
    {
        if (pulled_now.count(gone))
            continue;
        if (!mirrorDeletePrim(be, o, gone))
            return fail(g_backendError);
    }
    be->mirror_pulled = pulled_now;
    be->mirror_cols = std::move(mirror_cols_now);

    const double t_seal0 = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;

    // 4. Seal + hierarchy cadence on the mirror (per-STEP work: the mirror
    //    ordinal advances by 2 once per step exactly as the single-product
    //    step always did, no matter how many products render).
    if (!sealOrdinal(be, o))
        return fail("attached seal(local) failed");
    if (!computeWorldXforms(be->stage, o, o + 1, "attached compute hierarchy"))
        return fail(g_backendError);
    if (!mirrorWriteBatches(be, o + 1, worldOverrideBatches))
        return fail(g_backendError);
    if (!sealOrdinal(be, o + 1))
        return fail("attached seal(world) failed");

    // A preceding direct batch may have published an external ordinal whose
    // numeric value aliases this mirror ordinal. Force the recovery render to
    // consume the newly synchronized mirror regardless of that coincidence.
    if (be->mirror_transform_dirty && ovgl_invalidate_scene(be->renderer).status != 0)
        return fail("failed to invalidate the resident OVGL scene");

    // 5+6. Per PRODUCT: camera from the bound Camera prim's composed world
    //      transform (+ focalLength/apertures), one render, publish vars.
    //      A failure on ANY product fails the whole step (fail-closed;
    //      matches the measured official whole-step failure shape).
    const double t_render0 = isProfilingEnabled() ? getCurrentTimeMilliseconds() : 0.0;
    be->attached_products.clear();
    be->cur_ordinal = o + 2;
    for (const DerivedProduct& product : derived)
    {
        std::string rerr;
        if (!renderDerivedProduct(be, product, o + 1, o + 1, present_request, &rerr))
        {
            be->attached_products.clear();
            return fail(rerr);
        }
    }
    // Retain this step's committed snapshot for the fast-path classification
    // of the next step (only after complete success — any earlier return
    // leaves the cache invalidated and forces the next step onto this path).
    be->attached_pull = std::move(scene);
    be->attached_source_pull = std::move(sourceScene);
    be->attached_source_attributes = std::move(sourceAttributes);
    be->attached_scene_graph_instances = std::move(sceneGraphInstances);
    be->attached_pull_ordinal = end_ordinal;
    be->attached_render_ordinal = o + 1;
    be->attached_scene_ordinal = o + 1;
    be->mirror_transform_dirty = false;
    be->pulled_transform_dirty = false;
    be->transform_journal_generation = 0;
    be->transform_journal_paths.clear();
    be->transform_journal_routes.clear();
    be->attached_light_paths_valid = collectLightPaths(be->attached_pull, ext_dict, &be->attached_light_paths);
    be->attached_pull_valid = true;
    if (isProfilingEnabled())
    {
        const double t_end = getCurrentTimeMilliseconds();
        std::fprintf(stderr, "OVGLBEPROF mode=full pull=%.3f mirror=%.3f seal+hier=%.3f render=%.3f total=%.3f\n",
                     t_mirror0 - t_pull0, t_seal0 - t_mirror0, t_render0 - t_seal0, t_end - t_render0, t_end - t_pull0);
    }
    return 0;
}

int renderStage(OvglBackend* be,
                ovstage_ordinal_t ordinal,
                const char* render_product_path,
                const uint8_t** out_data,
                size_t* out_nbytes,
                int* out_width,
                int* out_height,
                char* err,
                size_t errlen)
{
    auto fail = [&](const std::string& message) -> int
    {
        setBackendError(message);
        if (err && errlen)
            std::snprintf(err, errlen, "%s", message.c_str());
        return 1;
    };
    if (!be || !render_product_path || !out_data || !out_nbytes || !out_width || !out_height)
        return fail("invalid attached-render arguments");

    const char* product_paths[] = { render_product_path };
    if (stepAttached(be, ordinal, product_paths, 1, nullptr, err, errlen) != 0)
        return 1;
    if (be->attached_products.size() != 1 || be->attached_products[0].path != render_product_path)
        return fail("OVGL returned an invalid RenderProduct result");

    for (const OvglBackend::AttachedVar& var : be->attached_products[0].vars)
    {
        if (var.name != "LdrColor")
            continue;
        if (var.format != BE_VAR_FMT_RGBA8)
            return fail("OVGL LdrColor output is not RGBA8");
        *out_data = var.data.data();
        *out_nbytes = var.data.size();
        *out_width = var.w;
        *out_height = var.h;
        return 0;
    }
    return fail("The configured RenderProduct has no LdrColor RenderVar output");
}

int presentStage(OvglBackend* be,
                 ovstage_ordinal_t ordinal,
                 const char* render_product_path,
                 unsigned framebuffer,
                 int destination_width,
                 int destination_height,
                 const char* overlay_text,
                 int* out_width,
                 int* out_height,
                 char* err,
                 size_t errlen)
{
    auto fail = [&](const std::string& message) -> int
    {
        setBackendError(message);
        if (err && errlen)
            std::snprintf(err, errlen, "%s", message.c_str());
        return 1;
    };
    if (!be || !render_product_path || destination_width <= 0 || destination_height <= 0 || !out_width || !out_height)
        return fail("invalid attached-present arguments");

    const PresentRequest request{ framebuffer, destination_width, destination_height, overlay_text };
    const char* product_paths[] = { render_product_path };
    if (stepAttached(be, ordinal, product_paths, 1, &request, err, errlen) != 0)
        return 1;
    if (be->attached_products.size() != 1 || be->attached_products[0].path != render_product_path)
        return fail("OVGL returned an invalid RenderProduct result");
    *out_width = be->attached_products[0].width;
    *out_height = be->attached_products[0].height;
    return 0;
}

} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
