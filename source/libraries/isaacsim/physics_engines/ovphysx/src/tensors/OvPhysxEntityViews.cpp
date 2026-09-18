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

#include "OvPhysxEntityViews.hpp"

#include "OvPhysxResultHelpers.hpp"
#include "OvPhysxTensorHelpers.hpp"

#include <dlpack/dlpack.h>
#include <isaacsim/physics/registration/tensors/TensorTypes.hpp>
#include <isaacsim/physics_engines/ovphysx/Backend.hpp>
#include <ovphysx/ovphysx.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <unordered_set>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

using namespace isaacsim::physics::tensors;

// ----------------------------------------------------------------------------
// TensorDescription -> DLTensor
// ----------------------------------------------------------------------------

// A failed binding-specification query leaves a zero-initialized specification (ndim 0, dtype {0,0,0}) that would
// silently flow into buffer sizing, the entity count, metadata dimensions and the dtype default. The binding
// handle was just created, so a query failure is an internal error -- surface it loudly.
static void checkSpecificationQueryStatus(const ovphysx_result_t& result, const char* label)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        throw std::runtime_error(std::string(label) + ": ovphysx_get_tensor_binding_spec failed");
    }
}

static std::vector<int64_t> checkedTensorSpecificationShape(const ovphysx_tensor_spec_t& specification, const char* label)
{
    constexpr int32_t maximumRank = static_cast<int32_t>(sizeof(specification.shape) / sizeof(specification.shape[0]));
    if (specification.ndim < 0 || specification.ndim > maximumRank)
    {
        throw std::runtime_error(std::string(label) + ": tensor rank is outside the supported range");
    }

    std::vector<int64_t> shape(specification.shape, specification.shape + specification.ndim);
    checkedElementCount(shape, label);
    return shape;
}

// ----------------------------------------------------------------------------
// TensorSpecification from ovphysx binding
// ----------------------------------------------------------------------------

TensorSpecification getTensorSpecificationFromOvphysx(ovphysx_handle_t handle,
                                                      ovphysx_tensor_binding_handle_t binding,
                                                      bool supportsIndexedRead,
                                                      bool supportsIndexedWrite)
{
    ovphysx_tensor_spec_t ovSpecification{};
    checkSpecificationQueryStatus(
        ovphysx_get_tensor_binding_spec(handle, binding, &ovSpecification), "binding specification");

    TensorSpecification specification;
    specification.supports = true;
    specification.supportsIndexedRead = supportsIndexedRead;
    specification.supportsIndexedWrite = supportsIndexedWrite;

    // Map DLPack dtype back to umbrella DType.
    if (ovSpecification.dtype.code == static_cast<uint8_t>(kDLFloat) && ovSpecification.dtype.bits == 32)
    {
        specification.dtype = DType::eFloat32;
    }
    else if (ovSpecification.dtype.code == static_cast<uint8_t>(kDLFloat) && ovSpecification.dtype.bits == 64)
    {
        specification.dtype = DType::eFloat64;
    }
    else if (ovSpecification.dtype.code == static_cast<uint8_t>(kDLInt) && ovSpecification.dtype.bits == 32)
    {
        specification.dtype = DType::eInt32;
    }
    else if (ovSpecification.dtype.code == static_cast<uint8_t>(kDLUInt) && ovSpecification.dtype.bits == 8)
    {
        // uint8, NOT eBool: eBool serializes to DLPack kDLBool(6,8), which warp's
        // from_dlpack rejects ("Unknown DLPack datatype (6,8)"). uint8 round-trips.
        specification.dtype = DType::eUInt8;
    }
    else
    {
        // Defaulting an unmapped dtype to float32 would silently mis-type the data and, worse,
        // desynchronize it from the element width used to size the read buffer.
        throw std::runtime_error("binding specification: unsupported tensor dtype");
    }

    specification.shapeHint = checkedTensorSpecificationShape(ovSpecification, "binding specification");
    specification.deviceKind = DeviceKind::eEngineDefault;
    return specification;
}

// ----------------------------------------------------------------------------
// OvPhysxEntityViewBase
// ----------------------------------------------------------------------------

OvPhysxEntityViewBase::OvPhysxEntityViewBase(ovphysx_handle_t handle, const std::vector<std::string>& paths)
    : EntityView(paths), m_handle(handle)
{
}

OvPhysxEntityViewBase::~OvPhysxEntityViewBase()
{
    for (auto& bindingEntry : m_bindings)
    {
        ovphysx_destroy_tensor_binding(m_handle, bindingEntry.second);
    }
}

namespace
{

bool isWriteOnlyType(ovphysx_tensor_type_t tensorType)
{
    switch (tensorType)
    {
    case OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32:
    case OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32:
    case OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32:
        return true;
    default:
        return false;
    }
}

// Which uint8 bindings are flags. Their documented contract is "nonzero disables",
// so the byte for true is not promised to be 1 and gets normalized on read. A uint8
// binding that is enumerated rather than boolean keeps its value.
bool isFlagType(ovphysx_tensor_type_t tensorType)
{
    switch (tensorType)
    {
    case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL:
    case OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL:
    case OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL:
        return true;
    default:
        return false;
    }
}

bool isReadOnlyType(ovphysx_tensor_type_t tensorType)
{
    switch (tensorType)
    {
    case OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32:
    case OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32:
    case OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32:
    case OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32:
    case OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32:
    case OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32:
    case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32:
    case OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32:
    case OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32:
    case OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32:
    case OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32:
    case OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32:
    case OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8:
    case OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32:
    case OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32:
    case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32:
    case OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32:
    case OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32:
    // Volume deformable body -- topology and rest state are read-only.
    case OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32:
    case OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
    case OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32:
    // Surface deformable body -- topology and rest state are read-only.
    case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32:
    case OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32:
        return true;
    default:
        return false;
    }
}

} // namespace

// Helper: compute the internal-read buffer layout {shape, total byte size} for an
// ovphysx tensor specification, WITHOUT allocating. The buffer itself is allocated lazily
// by the GET implementation on the first call that gets no caller-supplied `out`, so an
// all-external consumer (e.g. the GPU lane, which always passes an `out`) never
// allocates it. Byte size honors the specification dtype (float32 default).
// Bytes per element of a DLPack dtype. Single source of truth: the read buffer's size and the
// indexed gather's row width both come from here, so they cannot disagree about element width.
static size_t getElementByteWidth(const DLDataType& dataType, const char* label)
{
    if (dataType.bits == 0 || dataType.bits % 8 != 0)
    {
        throw std::runtime_error(std::string(label) + ": unsupported tensor element bit width");
    }
    return static_cast<size_t>(dataType.bits) / 8u;
}

static std::pair<std::vector<int64_t>, size_t> outputBufferLayout(const ovphysx_tensor_spec_t& specification,
                                                                  const char* label)
{
    std::vector<int64_t> shape = checkedTensorSpecificationShape(specification, label);
    const size_t elementCount = checkedElementCount(shape, label);
    return { std::move(shape), checkedSizeProduct(elementCount, getElementByteWidth(specification.dtype, label), label) };
}

// Point a binding descriptor at either an explicit resolved prim-path list (multi-pattern
// views that matched something) or a single pattern. Explicit-path mode is used only
// for a NON-empty resolved list: ovphysx 0.5.1 selects it only when prim_paths_count > 0,
// so an empty list would send neither a path list nor a pattern and the binding would
// fail to create. An all-miss multi-pattern view therefore falls back to pattern mode
// with `pattern` (a representative, non-matching pattern), which yields valid zero-sized
// bindings -- exactly like a single non-matching pattern. `pathStrings` is caller-owned scratch
// that must stay alive through the ovphysx_create_tensor_binding call.
static void setBindingSource(ovphysx_tensor_binding_desc_t& descriptor,
                             const std::string& pattern,
                             const std::optional<std::vector<std::string>>& resolvedPaths,
                             std::vector<ovphysx_string_t>& pathStrings)
{
    if (resolvedPaths.has_value() && !resolvedPaths->empty())
    {
        pathStrings.clear();
        pathStrings.reserve(resolvedPaths->size());
        for (const std::string& path : *resolvedPaths)
        {
            pathStrings.push_back(ovphysx_cstr(path.c_str()));
        }
        descriptor.pattern = ovphysx_string_t{};
        descriptor.prim_paths = pathStrings.data();
        descriptor.prim_paths_count = static_cast<uint32_t>(pathStrings.size());
    }
    else
    {
        descriptor.pattern = ovphysx_cstr(pattern.c_str());
        descriptor.prim_paths = nullptr;
        descriptor.prim_paths_count = 0;
    }
}

class ScopedTensorBinding
{
public:
    ScopedTensorBinding(ovphysx_handle_t handle, ovphysx_tensor_binding_handle_t binding)
        : m_handle(handle), m_binding(binding)
    {
    }
    ~ScopedTensorBinding()
    {
        if (m_binding)
        {
            ovphysx_destroy_tensor_binding(m_handle, m_binding);
        }
    }
    ScopedTensorBinding(const ScopedTensorBinding&) = delete;
    ScopedTensorBinding& operator=(const ScopedTensorBinding&) = delete;

    ovphysx_tensor_binding_handle_t release() noexcept
    {
        const ovphysx_tensor_binding_handle_t binding = m_binding;
        m_binding = 0;
        return binding;
    }

private:
    ovphysx_handle_t m_handle;
    ovphysx_tensor_binding_handle_t m_binding;
};

class ScopedContactBinding
{
public:
    ScopedContactBinding(ovphysx_handle_t handle, ovphysx_contact_binding_handle_t binding)
        : m_handle(handle), m_binding(binding)
    {
    }
    ~ScopedContactBinding()
    {
        if (m_binding)
        {
            ovphysx_destroy_contact_binding(m_handle, m_binding);
        }
    }
    ScopedContactBinding(const ScopedContactBinding&) = delete;
    ScopedContactBinding& operator=(const ScopedContactBinding&) = delete;

    ovphysx_contact_binding_handle_t release() noexcept
    {
        const ovphysx_contact_binding_handle_t binding = m_binding;
        m_binding = 0;
        return binding;
    }

private:
    ovphysx_handle_t m_handle;
    ovphysx_contact_binding_handle_t m_binding;
};

class ScopedSdfView
{
public:
    ScopedSdfView(ovphysx_handle_t handle, ovphysx_sdf_view_handle_t view) : m_handle(handle), m_view(view)
    {
    }
    ~ScopedSdfView()
    {
        if (m_view)
        {
            ovphysx_destroy_sdf_view(m_handle, m_view);
        }
    }
    ScopedSdfView(const ScopedSdfView&) = delete;
    ScopedSdfView& operator=(const ScopedSdfView&) = delete;

    ovphysx_sdf_view_handle_t release() noexcept
    {
        const ovphysx_sdf_view_handle_t view = m_view;
        m_view = 0;
        return view;
    }

private:
    ovphysx_handle_t m_handle;
    ovphysx_sdf_view_handle_t m_view;
};

// Resolve a pattern list to explicit prim paths via ovphysx itself (one
// throwaway binding per pattern). Each element may be a glob or an explicit
// path; matches are concatenated in list order with first-wins de-duplication.
// A pattern matching nothing contributes no rows.
static std::vector<std::string> resolvePatterns(ovphysx_handle_t handle,
                                                const std::vector<std::string>& patterns,
                                                ovphysx_tensor_type_t referenceTensorType)
{
    std::vector<std::string> paths;
    std::unordered_set<std::string> seen;
    for (const std::string& pattern : patterns)
    {
        ovphysx_tensor_binding_desc_t descriptor{};
        descriptor.tensor_type = referenceTensorType;
        descriptor.pattern = ovphysx_cstr(pattern.c_str());
        ovphysx_tensor_binding_handle_t bindingHandle = 0;
        // Skip a pattern only when binding creation genuinely fails (unsupported/invalid
        // pattern). A well-formed pattern that matches zero prims still succeeds and
        // contributes zero paths below, so it is not conflated with a failed one. The skip
        // is best-effort: one bad element does not sink resolution of the rest of the list.
        const ovphysx_result_t createResult = ovphysx_create_tensor_binding(handle, &descriptor, &bindingHandle);
        ScopedTensorBinding guard(handle, bindingHandle);
        if (createResult.status != OVPHYSX_API_SUCCESS || bindingHandle == 0)
        {
            continue;
        }
        ovphysx_tensor_spec_t pathSpecification{};
        checkSpecificationQueryStatus(
            ovphysx_get_tensor_binding_spec(handle, bindingHandle, &pathSpecification), "resolve specification");
        const uint32_t pathCapacity = pathSpecification.ndim >= 1 ? static_cast<uint32_t>(pathSpecification.shape[0]) : 0;
        std::vector<ovphysx_string_t> pathBuffer(pathCapacity);
        uint32_t count = 0;
        ovphysx_tensor_binding_get_prim_paths(handle, bindingHandle, pathBuffer.data(), pathCapacity, &count);
        for (uint32_t i = 0; i < count; ++i)
        {
            std::string path(pathBuffer[i].ptr, pathBuffer[i].length);
            if (seen.insert(path).second)
            {
                paths.push_back(std::move(path));
            }
        }
    }
    return paths;
}

std::vector<std::string> OvPhysxEntityViewBase::getResolvedPrimPaths() const
{
    if (m_bindings.empty())
    {
        return {};
    }
    auto bindingHandle = m_bindings.begin()->second;
    ovphysx_tensor_spec_t specification{};
    checkSpecificationQueryStatus(
        ovphysx_get_tensor_binding_spec(m_handle, bindingHandle, &specification), "prim-paths specification");
    uint32_t capacity =
        (specification.ndim >= 1 && specification.shape[0] > 0) ? static_cast<uint32_t>(specification.shape[0]) : 4096u;
    std::vector<ovphysx_string_t> buffer(capacity);
    uint32_t count = 0;
    ovphysx_tensor_binding_get_prim_paths(m_handle, bindingHandle, buffer.data(), capacity, &count);
    std::vector<std::string> result(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        result[i] = std::string(buffer[i].ptr, buffer[i].length);
    }
    return result;
}

int resolveOvPhysxDeviceOrdinal(const isaacsim::physics::tensors::EntityView& view)
{
    // An ordinal assigned to this view wins; `EntityView` stores `-1` until someone assigns one, which is
    // also the value for host memory, so an unassigned view simply follows the backend declaration.
    const int assigned = view.isaacsim::physics::tensors::EntityView::getDeviceOrdinal();
    return assigned >= 0 ? assigned : getTensorDeviceOrdinal();
}

int OvPhysxEntityViewBase::getDeviceOrdinal() const noexcept
{
    return resolveOvPhysxDeviceOrdinal(*this);
}

void OvPhysxEntityViewBase::_setEntityCountFromBindings()
{
    if (m_bindings.empty())
    {
        return;
    }
    ovphysx_tensor_spec_t specification{};
    checkSpecificationQueryStatus(
        ovphysx_get_tensor_binding_spec(m_handle, m_bindings.begin()->second, &specification), "count specification");
    if (specification.ndim >= 1 && specification.shape[0] > 0)
    {
        setEntityCount(static_cast<int>(specification.shape[0]));
    }
}

void OvPhysxEntityViewBase::_initializeBindings(
    const std::vector<std::pair<std::string, ovphysx_tensor_type_t>>& implementationMap,
    const std::string& pattern,
    const std::optional<std::vector<std::string>>& resolvedPaths)
{
    std::vector<ovphysx_string_t> pathStrings;
    for (const auto& implementation : implementationMap)
    {
        const std::string& implementationName = implementation.first;
        const ovphysx_tensor_type_t tensorType = implementation.second;
        ovphysx_tensor_binding_desc_t descriptor{};
        descriptor.tensor_type = tensorType;
        setBindingSource(descriptor, pattern, resolvedPaths, pathStrings);

        ovphysx_tensor_binding_handle_t bindingHandle = 0;
        ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &descriptor, &bindingHandle);
        ScopedTensorBinding bindingGuard(m_handle, bindingHandle);
        if (result.status != OVPHYSX_API_SUCCESS || bindingHandle == 0)
        {
            continue; // binding unsupported for this entity type -- skip
        }

        m_bindings[implementationName] = bindingHandle;
        bindingGuard.release();

        // Expose the per-entity dimension count as metadata for the impls whose
        // count has no other metadata source, so consumers read it through the
        // uniform string-keyed API (num-shapes / num-fixed-tendons /
        // num-spatial-tendons, like num-dofs / num-links).
        const char* dimensionMetadataKey = nullptr;
        if (implementationName == "material-properties")
        {
            dimensionMetadataKey = "num-shapes";
        }
        else if (implementationName == "fixed-tendon-stiffnesses")
        {
            dimensionMetadataKey = "num-fixed-tendons";
        }
        else if (implementationName == "spatial-tendon-stiffnesses")
        {
            dimensionMetadataKey = "num-spatial-tendons";
        }
        if (dimensionMetadataKey)
        {
            ovphysx_tensor_spec_t dimensionSpecification{};
            checkSpecificationQueryStatus(
                ovphysx_get_tensor_binding_spec(m_handle, bindingHandle, &dimensionSpecification), "dim specification");
            // These bindings are laid out [num_entities, count] (shapes / tendons
            // per entity), so axis 1 is the per-entity count. This assumes that
            // layout -- revisit if a binding ever reports the count on another axis.
            const int64_t dimensionCount =
                dimensionSpecification.ndim >= 2 ? static_cast<int64_t>(dimensionSpecification.shape[1]) : 0;
            registerMetadata(dimensionMetadataKey, [dimensionCount]() -> Metadata { return dimensionCount; });
        }

        const bool writeOnly = isWriteOnlyType(tensorType);
        const bool readOnly = isReadOnlyType(tensorType);

        if (!writeOnly) // register Get implementation for non-write-only tensors
        {
            TensorSpecification getSpecification = getTensorSpecificationFromOvphysx(
                m_handle, bindingHandle, /*indexedRead=*/false, /*indexedWrite=*/false);
            getSpecification.supportsIndexedRead = true;

            // Fetch the binding specification once and reuse it for both the output buffer
            // sizing and the DLPack dtype captured by the lambda.
            ovphysx_tensor_spec_t ovSpecification{};
            checkSpecificationQueryStatus(
                ovphysx_get_tensor_binding_spec(m_handle, bindingHandle, &ovSpecification), "get specification");

            // Output-buffer layout (shape + byte size + per-element width). The
            // internal full-read buffer `buffer` is allocated lazily on first use, so an
            // all-external consumer (the GPU lane always passes an `out`) never
            // allocates it. The lambda captures it so a non-indexed result stays valid
            // for the view's lifetime (overwritten each getData, single-writer); an
            // indexed result instead gets fresh per-call storage owned by the result.
            auto outputLayout = outputBufferLayout(ovSpecification, implementationName.c_str());
            std::vector<int64_t> shape = std::move(outputLayout.first);
            const size_t byteSize = outputLayout.second;
            const size_t elementBytes = getElementByteWidth(ovSpecification.dtype, implementationName.c_str());

            auto handle = m_handle;
            // Capture the DLPack dtype so the lambda doesn't need to re-query the specification.
            DLDataType dlDataType = ovSpecification.dtype;

            // uint8 tensors need conversion to float32 because warp's from_dlpack()
            // doesn't handle kDLUInt:8.
            const bool isUint8 = (dlDataType.code == static_cast<uint8_t>(kDLUInt) && dlDataType.bits == 8);
            const bool isFlag = isFlagType(tensorType);
            const size_t elementCount = checkedElementCount(shape, implementationName.c_str());
            auto float32Buffer = isUint8 ? std::make_shared<std::vector<float>>(elementCount, 0.0f) :
                                           std::shared_ptr<std::vector<float>>{};
            const DType outputDataType = isUint8 ? DType::eFloat32 : getSpecification.dtype;
            // Match the registered specification to context the implementation actually returns.
            getSpecification.dtype = outputDataType;

            registerImplementation(
                implementationName, ImplementationKind::eGet,
                GetImplementationFunction{
                    [handle, bindingHandle, buffer = std::shared_ptr<std::vector<uint8_t>>(), shape, byteSize,
                     elementBytes, dlDataType, isUint8, isFlag, float32Buffer, managerDataType = getSpecification.dtype,
                     implementationName](
                        const TensorDescription& indices, const TensorDescription& output) mutable -> TensorDescription
                    {
                        // Distinguish "no selection" (None -> empty shape, full read) from an
                        // explicitly supplied selection (ndim >= 1; K may be 0 -> 0 rows). A
                        // zero-length index array has null data, so isEmpty() would wrongly fold
                        // it into the full-read case.
                        const bool indexed = !indices.shape.empty();
                        // Read straight into a caller-supplied `out` (honoring its device -- a GPU
                        // `out` triggers a DirectGPU read) only for a plain full, non-bool read.
                        // An indexed read gathers from a staged buffer, and a bool binding needs
                        // CPU conversion, so both stage through the lazily-allocated internal buffer.
                        const bool hasOutput = !indexed && !isUint8 && !output.isEmpty() && output.data;
                        const bool outputOnGpu = hasOutput && output.device == DeviceKind::eGpu;
                        void* readInto;
                        if (hasOutput)
                        {
                            requireMatchingOutput(output, shape, managerDataType, implementationName.c_str());
                            readInto = output.data;
                        }
                        else
                        {
                            if (!buffer)
                            {
                                buffer = std::make_shared<std::vector<uint8_t>>(byteSize > 0 ? byteSize : 1, 0u);
                            }
                            readInto = buffer->data();
                        }

                        DLTensor destination{};
                        destination.data = readInto;
                        destination.device =
                            outputOnGpu ? DLDevice{ kDLCUDA, output.deviceOrdinal } : DLDevice{ kDLCPU, 0 };
                        destination.dtype = dlDataType;
                        destination.ndim = static_cast<int32_t>(shape.size());
                        destination.shape = const_cast<int64_t*>(shape.data());
                        destination.strides = nullptr;
                        destination.byte_offset = 0;
                        const ovphysx_result_t readResult =
                            ovphysx_read_tensor_binding(handle, bindingHandle, &destination);
                        if (readResult.status != OVPHYSX_API_SUCCESS)
                        {
                            throw std::runtime_error(implementationName + ": ovphysx_read_tensor_binding failed");
                        }

                        // uint8 bindings are read as bytes and converted to float32 once (warp's
                        // from_dlpack can't consume kDLUInt:8); both the full read and the indexed
                        // gather below then source from the converted buffer.
                        if (isUint8)
                        {
                            const uint8_t* sourceBytes = static_cast<const uint8_t*>(buffer->data());
                            float* destinationValues = float32Buffer->data();
                            // bytes == elems for a uint8 binding, but clamp to the smaller of the
                            // two buffers so a specification/byteSize mismatch can never OOB-read `buffer`.
                            const size_t elementCountToCopy =
                                float32Buffer->size() < buffer->size() ? float32Buffer->size() : buffer->size();
                            // A flag binding promises only "nonzero", so normalize it and keep
                            // handing consumers something they can compare against 1.
                            for (size_t i = 0; i < elementCountToCopy; ++i)
                            {
                                destinationValues[i] =
                                    isFlag ? (sourceBytes[i] ? 1.0f : 0.0f) : static_cast<float>(sourceBytes[i]);
                            }
                        }

                        if (!indexed)
                        {
                            // Full-size result (no indices -> full-size).
                            TensorDescription result;
                            result.data = isUint8 ? static_cast<void*>(float32Buffer->data()) : readInto;
                            result.dtype = managerDataType;
                            result.shape = shape;
                            result.device = outputOnGpu ? DeviceKind::eGpu : DeviceKind::eCpu;
                            result.deviceOrdinal = outputOnGpu ? output.deviceOrdinal : -1;
                            return result;
                        }
                        else
                        {
                            // Indexed gather. A bool binding gathers from the float32-converted buffer
                            // above, so the index-size result carries float32 like the full read.
                            // The gather dereferences the index values on the host and
                            // copies rows out of the full read, so the indices must be
                            // CPU-resident int32 -- the same convention the SET impls
                            // enforce. Reject a GPU pointer or a wider dtype up front
                            // rather than misread it.
                            if (indices.device == DeviceKind::eGpu)
                            {
                                throw std::runtime_error(implementationName +
                                                         ": indexed get requires CPU indices, received GPU indices");
                            }
                            requireInt32Indices(indices, implementationName.c_str());

                            // Indexed: gather requested rows -> index-size. The
                            // read above went into `buffer`; copy rows into the caller's CPU
                            // `out` if given, else a fresh result-owned buffer.
                            const int64_t selectedRowCount = indices.shape.empty() ? 0 : indices.shape[0];
                            const int64_t availableRowCount = shape.empty() ? 0 : shape[0];
                            const std::vector<int64_t> rowShape(
                                shape.begin() + std::min<size_t>(1, shape.size()), shape.end());
                            const size_t rowElementCount = checkedElementCount(rowShape, implementationName.c_str());
                            // A bool result is float32 (converted above), so its rows are wider than
                            // the uint8 read; size by the result element width.
                            const size_t rowBytes = checkedSizeProduct(
                                rowElementCount, isUint8 ? sizeof(float) : elementBytes, implementationName.c_str());
                            const size_t selectedByteCount =
                                checkedSizeProduct(checkedElementCount({ selectedRowCount }, implementationName.c_str()),
                                                   rowBytes, implementationName.c_str());

                            // Index-size result shape [K, *shape[1:]].
                            std::vector<int64_t> indexedShape = shape;
                            if (!indexedShape.empty())
                            {
                                indexedShape[0] = selectedRowCount;
                            }

                            uint8_t* resultData;
                            std::shared_ptr<std::vector<uint8_t>> owned;
                            // A supplied `out` has an explicit shape (K rows, K may be 0);
                            // isEmpty() folds a null-data 0-row buffer into the no-out case, so
                            // key on the shape -- like the `indexed` flag above -- to recognize a
                            // supplied out even when it carries no storage.
                            const bool hasIndexedOutput = !output.shape.empty();
                            if (hasIndexedOutput)
                            {
                                // The gather is host-side (memcpy from the CPU `buffer`), so it can
                                // only fill a CPU out. Reject a wrong-device out loudly rather than
                                // silently returning a different CPU buffer -- this catches a
                                // zero-row GPU out too, which isEmpty() would misread as absent.
                                if (output.device != DeviceKind::eCpu)
                                {
                                    throw std::runtime_error(implementationName +
                                                             ": indexed get requires a CPU out buffer");
                                }
                                // Validate the caller buffer against the index-size result
                                // (K rows) before writing, mirroring the non-indexed path --
                                // a wrong-K or wrong-dtype `out` would otherwise overflow.
                                requireMatchingOutput(output, indexedShape, managerDataType, implementationName.c_str());
                            }
                            if (hasIndexedOutput && output.data)
                            {
                                // Gather into the caller's buffer when it has storage (K > 0).
                                resultData = static_cast<uint8_t*>(output.data);
                            }
                            else
                            {
                                // No out, or a validated zero-row out whose null data can't back a
                                // result. Per-call owned storage handed to the result (freed when
                                // the caller drops the array), NOT a reused view-lifetime buffer --
                                // so a later larger-K read can't realloc it out from under an
                                // earlier result. Sized for K rows (K may exceed N with duplicate
                                // indices); at least 1 byte so a K=0 selection still yields a
                                // non-null 0-row result.
                                owned = std::make_shared<std::vector<uint8_t>>(std::max<size_t>(1, selectedByteCount));
                                resultData = owned->data();
                            }
                            // Out-of-range indices are dropped rather than rejected -- rejecting them
                            // would require reading the index values back from the device. Seed the K
                            // rows with the all-bits-set marker so a dropped row cannot be mistaken
                            // for data: NaN for float, -1 for signed, max for unsigned. The GPU
                            // gather seeds the same pattern via invalid_fill_value().
                            if (selectedRowCount > 0)
                            {
                                std::memset(resultData, 0xFF, selectedByteCount);
                            }
                            const int32_t* indexData = static_cast<const int32_t*>(indices.data);
                            const uint8_t* source =
                                isUint8 ? reinterpret_cast<const uint8_t*>(float32Buffer->data()) : buffer->data();
                            for (int64_t j = 0; j < selectedRowCount; ++j)
                            {
                                const int64_t bodyIndex = indexData[j];
                                if (bodyIndex < 0 || bodyIndex >= availableRowCount)
                                {
                                    continue; // drop out-of-range -> leaves the seeded marker row
                                }
                                std::memcpy(resultData + static_cast<size_t>(j) * rowBytes,
                                            source + static_cast<size_t>(bodyIndex) * rowBytes, rowBytes);
                            }

                            TensorDescription result;
                            result.data = resultData;
                            result.dtype = managerDataType;
                            result.shape = indexedShape;
                            result.device = DeviceKind::eCpu;
                            result.keepAlive = owned; // empty for the external-out path (caller owns `out`)
                            return result;
                        }
                    } },
                getSpecification);
        }

        if (!readOnly) // register Set implementation for non-read-only tensors
        {
            TensorSpecification setSpecification =
                getTensorSpecificationFromOvphysx(m_handle, bindingHandle, /*indexedRead=*/false, /*indexedWrite=*/true);
            setSpecification.supportsIndexedWrite = true;
            auto handle = m_handle;
            ovphysx_tensor_spec_t bindingSpecification{};
            checkSpecificationQueryStatus(
                ovphysx_get_tensor_binding_spec(m_handle, bindingHandle, &bindingSpecification), "set specification");
            const int32_t bindingDimensionCount = bindingSpecification.ndim;
            const int64_t bindingRowCount =
                bindingSpecification.ndim >= 1 ? static_cast<int64_t>(bindingSpecification.shape[0]) : 0;
            const bool isBooleanBinding = bindingSpecification.dtype.code == static_cast<uint8_t>(kDLUInt) &&
                                          bindingSpecification.dtype.bits == 8;
            registerImplementation(
                implementationName, ImplementationKind::eSet,
                SetImplementationFunction{
                    [handle, bindingHandle, bindingDimensionCount, bindingRowCount, isBooleanBinding,
                     implementationName](const TensorDescription& data, const TensorDescription& indices)
                    {
                        DLTensor source = convertToDLTensor(data);
                        std::vector<uint8_t> convertedBooleanData;
                        if (isBooleanBinding && data.dtype == DType::eFloat32)
                        {
                            if (data.device == DeviceKind::eGpu)
                            {
                                throw std::runtime_error(implementationName +
                                                         ": float32 boolean data must be CPU-resident");
                            }
                            if (!isContiguousRowMajor(data))
                            {
                                throw std::runtime_error(implementationName +
                                                         ": float32 boolean data must be C-contiguous");
                            }
                            const size_t elementCount = checkedElementCount(data.shape, implementationName.c_str());
                            convertedBooleanData.resize(elementCount);
                            const float* sourceValues = static_cast<const float*>(data.data);
                            for (size_t i = 0; i < elementCount; ++i)
                            {
                                convertedBooleanData[i] = sourceValues[i] != 0.0f;
                            }
                            source.data = convertedBooleanData.data();
                            source.device = { kDLCPU, 0 };
                            source.dtype = { static_cast<uint8_t>(kDLUInt), 8, 1 };
                            source.strides = nullptr;
                        }
                        else if (isBooleanBinding && data.dtype != DType::eUInt8 && data.dtype != DType::eBool)
                        {
                            throw std::runtime_error(implementationName +
                                                     ": boolean data must be uint8, bool, or float32");
                        }
                        // Squeeze a trailing singleton ONLY when the data carries more
                        // dims than the binding (canonical (N,1) scalar/mass vs a 1-D
                        // [N] binding). A legitimately 2-D binding such as DOF [N,1]
                        // when D==1 must NOT be squeezed.
                        if (source.ndim > bindingDimensionCount && source.ndim >= 2 && source.shape[source.ndim - 1] == 1)
                        {
                            source.ndim -= 1;
                        }
                        if (indices.isEmpty())
                        {
                            const ovphysx_result_t writeResult =
                                ovphysx_write_tensor_binding(handle, bindingHandle, &source, nullptr);
                            if (writeResult.status != OVPHYSX_API_SUCCESS)
                            {
                                throw std::runtime_error(implementationName + ": ovphysx_write_tensor_binding failed");
                            }
                            return;
                        }
                        // ovphysx requires int32[K] indices (the umbrella convention);
                        // reject other dtypes loudly rather than let ovphysx misread them.
                        requireInt32Indices(indices, implementationName.c_str());
                        // Indexed writes take the FULL [N, ...] source; ovphysx scatters rows by
                        // index into the full set. A compact [K, ...] source (K == index count)
                        // is not supported here yet -- reject clearly rather than let ovphysx
                        // misread the row count. Compact-to-full adaptation is a follow-up.
                        const int64_t sourceRowCount = data.shape.empty() ? 0 : data.shape[0];
                        if (bindingRowCount > 0 && sourceRowCount != bindingRowCount)
                        {
                            throw std::runtime_error(implementationName +
                                                     ": indexed write needs full [N, ...] rows "
                                                     "plus indices; compact [K, ...] unsupported");
                        }
                        DLTensor indexTensor = convertToDLTensor(indices);
                        const ovphysx_result_t writeResult =
                            ovphysx_write_tensor_binding(handle, bindingHandle, &source, &indexTensor);
                        if (writeResult.status != OVPHYSX_API_SUCCESS)
                        {
                            throw std::runtime_error(implementationName + ": ovphysx_write_tensor_binding failed");
                        }
                    } },
                setSpecification);
        }
    }
}


// ----------------------------------------------------------------------------
// Implementation tables
// ----------------------------------------------------------------------------

static const std::vector<std::pair<std::string, ovphysx_tensor_type_t>> g_kArticulationImplementations = {
    { "dof-positions", OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32 },
    { "dof-velocities", OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32 },
    { "dof-position-targets", OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32 },
    { "dof-velocity-targets", OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32 },
    { "dof-actuation-forces", OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32 },
    { "dof-limits", OVPHYSX_TENSOR_ARTICULATION_DOF_LIMIT_F32 },
    { "dof-stiffnesses", OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32 },
    { "dof-dampings", OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32 },
    { "dof-armatures", OVPHYSX_TENSOR_ARTICULATION_DOF_ARMATURE_F32 },
    { "dof-max-forces", OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32 },
    { "dof-max-velocities", OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_VELOCITY_F32 },
    { "dof-friction-properties", OVPHYSX_TENSOR_ARTICULATION_DOF_FRICTION_PROPERTIES_F32 },
    { "dof-drive-model-properties", OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32 },
    { "drive-types", OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8 },
    { "root-transforms", OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32 },
    { "root-velocities", OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32 },
    { "link-transforms", OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32 },
    { "link-velocities", OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32 },
    { "link-accelerations", OVPHYSX_TENSOR_ARTICULATION_LINK_ACCELERATION_F32 },
    { "masses", OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32 },
    { "inv-masses", OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32 },
    { "coms", OVPHYSX_TENSOR_ARTICULATION_BODY_COM_POSE_F32 },
    { "inertias", OVPHYSX_TENSOR_ARTICULATION_BODY_INERTIA_F32 },
    { "inv-inertias", OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32 },
    { "disable-gravities", OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL },
    { "jacobians", OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32 },
    { "generalized-mass-matrices", OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32 },
    { "coriolis-and-centrifugal-compensation-forces", OVPHYSX_TENSOR_ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE_F32 },
    { "gravity-compensation-forces", OVPHYSX_TENSOR_ARTICULATION_GRAVITY_FORCE_F32 },
    { "dof-projected-joint-forces", OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32 },
    { "link-incoming-joint-force", OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32 },
    { "material-properties", OVPHYSX_TENSOR_ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION_F32 },
    { "contact-offsets", OVPHYSX_TENSOR_ARTICULATION_CONTACT_OFFSET_F32 },
    { "rest-offsets", OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32 },
    { "apply-forces-and-torques-at-position", OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32 },
    { "fixed-tendon-stiffnesses", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32 },
    { "fixed-tendon-dampings", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_DAMPING_F32 },
    { "fixed-tendon-limit-stiffnesses", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS_F32 },
    { "fixed-tendon-limits", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_LIMIT_F32 },
    { "fixed-tendon-rest-lengths", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_REST_LENGTH_F32 },
    { "fixed-tendon-offsets", OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_OFFSET_F32 },
    { "spatial-tendon-stiffnesses", OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32 },
    { "spatial-tendon-dampings", OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_DAMPING_F32 },
    { "spatial-tendon-limit-stiffnesses", OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS_F32 },
    { "spatial-tendon-offsets", OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_OFFSET_F32 },
    // Both world-frame and local-frame mass center -- the legacy Python API exposed these
    // via a single get_articulation_mass_center(local_frame=bool) method parameter. The
    // umbrella string-keyed API exposes them as two separate impls.
    { "articulation-mass-center", OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32 },
    { "articulation-mass-center-local", OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32 },
    { "articulation-centroidal-momentum", OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32 },
};

static const std::vector<std::pair<std::string, ovphysx_tensor_type_t>> g_kRigidBodyImplementations = {
    { "transforms", OVPHYSX_TENSOR_RIGID_BODY_POSE_F32 },
    { "velocities", OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32 },
    { "accelerations", OVPHYSX_TENSOR_RIGID_BODY_ACCELERATION_F32 },
    { "masses", OVPHYSX_TENSOR_RIGID_BODY_MASS_F32 },
    { "inv-masses", OVPHYSX_TENSOR_RIGID_BODY_INV_MASS_F32 },
    { "inertias", OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32 },
    { "inv-inertias", OVPHYSX_TENSOR_RIGID_BODY_INV_INERTIA_F32 },
    { "coms", OVPHYSX_TENSOR_RIGID_BODY_COM_POSE_F32 },
    { "forces", OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32 },
    { "apply-forces", OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32 }, // legacy alias
    { "wrenches", OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32 },
    { "apply-forces-and-torques-at-position", OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32 },
    { "disable-simulations", OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL },
    { "disable-gravities", OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL },
    { "material-properties", OVPHYSX_TENSOR_RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION_F32 },
    { "contact-offsets", OVPHYSX_TENSOR_RIGID_BODY_CONTACT_OFFSET_F32 },
    { "rest-offsets", OVPHYSX_TENSOR_RIGID_BODY_REST_OFFSET_F32 },
};

// ----------------------------------------------------------------------------
// Articulation EntityView
// ----------------------------------------------------------------------------

// Get solver-ordered DOF names + resolved articulation prim paths by creating a
// temporary DOF-position binding. Used by the ctor to populate the legacy
// metadata surface (dof-names / link-names / prim-paths).
static bool queryArticulationNamesAndPaths(ovphysx_handle_t handle,
                                           const std::string& pattern,
                                           const std::optional<std::vector<std::string>>& resolvedPaths,
                                           std::vector<std::string>& outputDofNames,
                                           std::vector<std::string>& outputBodyNames,
                                           std::vector<std::string>& outputPrimPaths)
{
    ovphysx_tensor_binding_desc_t descriptor{};
    descriptor.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
    std::vector<ovphysx_string_t> pathStrings;
    setBindingSource(descriptor, pattern, resolvedPaths, pathStrings);
    ovphysx_tensor_binding_handle_t bindingHandle = 0;
    const ovphysx_result_t createResult = ovphysx_create_tensor_binding(handle, &descriptor, &bindingHandle);
    ScopedTensorBinding guard(handle, bindingHandle);
    if (createResult.status != OVPHYSX_API_SUCCESS || bindingHandle == 0)
    {
        return false;
    }

    ovphysx_articulation_metadata_t metadata{};
    ovphysx_get_articulation_metadata(handle, bindingHandle, &metadata);

    auto getNames = [&](auto callback, int32_t maximumCount, std::vector<std::string>& outputNames)
    {
        std::vector<ovphysx_string_t> buffer(static_cast<size_t>(maximumCount));
        uint32_t returnedCount = 0;
        callback(handle, bindingHandle, buffer.data(), static_cast<uint32_t>(maximumCount), &returnedCount);
        outputNames.resize(returnedCount);
        for (uint32_t i = 0; i < returnedCount; ++i)
        {
            outputNames[i] = std::string(buffer[i].ptr, buffer[i].length);
        }
    };

    getNames(ovphysx_articulation_get_dof_names, metadata.dof_count, outputDofNames);
    getNames(ovphysx_articulation_get_body_names, metadata.body_count, outputBodyNames);

    {
        ovphysx_tensor_spec_t pathSpecification{};
        checkSpecificationQueryStatus(
            ovphysx_get_tensor_binding_spec(handle, bindingHandle, &pathSpecification), "prim-paths specification");
        const uint32_t pathCapacity = (pathSpecification.ndim >= 1 && pathSpecification.shape[0] > 0) ?
                                          static_cast<uint32_t>(pathSpecification.shape[0]) :
                                          4096u;
        std::vector<ovphysx_string_t> paths(pathCapacity);
        uint32_t count = 0;
        ovphysx_tensor_binding_get_prim_paths(handle, bindingHandle, paths.data(), pathCapacity, &count);
        outputPrimPaths.resize(count);
        for (uint32_t i = 0; i < count; ++i)
        {
            outputPrimPaths[i] = std::string(paths[i].ptr, paths[i].length);
        }
    }

    return true;
}

OvPhysxArticulationEntityView::OvPhysxArticulationEntityView(ovphysx_handle_t handle, const std::string& pattern)
    : OvPhysxArticulationEntityView(handle, pattern, std::nullopt)
{
}

OvPhysxArticulationEntityView::OvPhysxArticulationEntityView(ovphysx_handle_t handle,
                                                             const std::vector<std::string>& patterns)
    : OvPhysxArticulationEntityView(handle,
                                    // Representative fallback pattern for the all-miss case (every pattern
                                    // matched nothing -> pattern mode with a non-matching pattern yields a
                                    // valid 0-prim binding). Ignored when the resolved list is non-empty.
                                    patterns.empty() ? std::string() : patterns.front(),
                                    resolvePatterns(handle, patterns, OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32))
{
}

OvPhysxArticulationEntityView::OvPhysxArticulationEntityView(ovphysx_handle_t handle,
                                                             std::string pattern,
                                                             std::optional<std::vector<std::string>> resolvedPaths)
    : OvPhysxEntityViewBase(handle, resolvedPaths ? *resolvedPaths : std::vector<std::string>{ pattern }),
      m_pattern(std::move(pattern)),
      m_resolvedPaths(std::move(resolvedPaths))
{
    _initializeBindings(g_kArticulationImplementations, m_pattern, m_resolvedPaths);

    // Wire `getEntityCount()` from a reference binding's resolved specification ([N, D] -> N
    // articulations). The framework's `getEntityCount()` defaults to 0 and nothing else
    // sets it for this view; consumers reshape `get_data` results by `getEntityCount()`,
    // so leaving it 0 breaks every shape-by-count test.
    auto bindingIterator = m_bindings.find("dof-positions");
    if (bindingIterator != m_bindings.end())
    {
        ovphysx_tensor_spec_t specification{};
        checkSpecificationQueryStatus(
            ovphysx_get_tensor_binding_spec(m_handle, bindingIterator->second, &specification), "count specification");
        const int64_t articulationCount = specification.ndim >= 1 ? specification.shape[0] : 0;
        if (articulationCount > 0)
        {
            setEntityCount(articulationCount);
        }

        // --- Per-articulation metadata: names, prim paths, fixed-base flag and
        // counts. Consumers read these via get_metadata("dof-names" / "link-names"
        // / "prim-paths" / "is-fixed-base" / "num-dofs" / ...). ---
        std::vector<std::string> dofNames, bodyNames, primPaths, jointNames;
        queryArticulationNamesAndPaths(m_handle, m_pattern, m_resolvedPaths, dofNames, bodyNames, primPaths);
        bool isFixedBase = false;
        {
            ovphysx_articulation_metadata_t metadata{};
            ovphysx_get_articulation_metadata(m_handle, bindingIterator->second, &metadata);
            isFixedBase = metadata.is_fixed_base;
            const int32_t jointCount = std::max(0, metadata.joint_count);
            std::vector<ovphysx_string_t> jointNameBuffer(static_cast<size_t>(jointCount));
            uint32_t jointNameCount = 0;
            if (jointCount > 0)
            {
                ovphysx_articulation_get_joint_names(m_handle, bindingIterator->second, jointNameBuffer.data(),
                                                     static_cast<uint32_t>(jointCount), &jointNameCount);
            }
            for (uint32_t i = 0; i < jointNameCount; ++i)
            {
                jointNames.emplace_back(jointNameBuffer[i].ptr, jointNameBuffer[i].length);
            }
        }
        registerMetadata("is-fixed-base", [isFixedBase]() -> Metadata { return isFixedBase; });
        registerMetadata("dof-names", [dofNames]() -> Metadata { return dofNames; });
        registerMetadata("link-names", [bodyNames]() -> Metadata { return bodyNames; });
        registerMetadata("joint-names", [jointNames]() -> Metadata { return jointNames; });
        registerMetadata("prim-paths", [primPaths]() -> Metadata { return primPaths; });
        registerMetadata("num-dofs", [value = static_cast<int64_t>(dofNames.size())]() -> Metadata { return value; });
        registerMetadata("num-links", [value = static_cast<int64_t>(bodyNames.size())]() -> Metadata { return value; });
        registerMetadata("num-joints", [value = static_cast<int64_t>(jointNames.size())]() -> Metadata { return value; });
    }
}

// ----------------------------------------------------------------------------
// Rigid body EntityView
// ----------------------------------------------------------------------------

// Defined below; used by the ctor for the legacy `prim_paths` metadata.
static std::vector<std::string> getRigidBodyPrimPaths(ovphysx_handle_t handle,
                                                      const std::string& pattern,
                                                      const std::optional<std::vector<std::string>>& resolvedPaths);

OvPhysxRigidBodyEntityView::OvPhysxRigidBodyEntityView(ovphysx_handle_t handle, const std::string& pattern)
    : OvPhysxRigidBodyEntityView(handle, pattern, std::nullopt)
{
}

OvPhysxRigidBodyEntityView::OvPhysxRigidBodyEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns)
    // Representative fallback pattern for the all-miss case (see the articulation ctor).
    : OvPhysxRigidBodyEntityView(handle,
                                 patterns.empty() ? std::string() : patterns.front(),
                                 resolvePatterns(handle, patterns, OVPHYSX_TENSOR_RIGID_BODY_POSE_F32))
{
}

OvPhysxRigidBodyEntityView::OvPhysxRigidBodyEntityView(ovphysx_handle_t handle,
                                                       std::string pattern,
                                                       std::optional<std::vector<std::string>> resolvedPaths)
    : OvPhysxEntityViewBase(handle, resolvedPaths ? *resolvedPaths : std::vector<std::string>{ pattern }),
      m_pattern(std::move(pattern)),
      m_resolvedPaths(std::move(resolvedPaths))
{
    _initializeBindings(g_kRigidBodyImplementations, m_pattern, m_resolvedPaths);

    // Wire `getEntityCount()` from a reference binding's resolved specification ([N,7] -> N bodies).
    // Same as the articulation view: `getEntityCount()` defaults to 0 and nothing else
    // sets it, so shape-by-count consumers break without this.
    {
        auto transformBindingIterator = m_bindings.find("transforms");
        if (transformBindingIterator != m_bindings.end())
        {
            ovphysx_tensor_spec_t specification{};
            checkSpecificationQueryStatus(
                ovphysx_get_tensor_binding_spec(m_handle, transformBindingIterator->second, &specification),
                "count specification");
            if (specification.ndim >= 1)
            {
                setEntityCount(specification.shape[0]);
            }
        }
    }

    // "prim-paths" metadata: resolved prim path per body, read via get_metadata("prim-paths").
    {
        std::vector<std::string> primPaths = getRigidBodyPrimPaths(m_handle, m_pattern, m_resolvedPaths);
        registerMetadata("prim-paths", [primPaths]() -> Metadata { return primPaths; });
    }

    // "wake-up" SET implementation: wakes the bodies named by its index buffer (passed as
    // `data` so consumers call view.set_data("wake-up", indices)).
    // Mirrors PxRigidDynamic::wakeUp via ovphysx_rigid_body_view_wake_up, which
    // has no documented GPU support -- a GPU index buffer is refused loudly (see
    // below) rather than dereferenced on the host.
    {
        auto poseBindingIterator = m_bindings.find("transforms");
        if (poseBindingIterator != m_bindings.end())
        {
            TensorSpecification wakeUpSpecification;
            wakeUpSpecification.supports = true;
            wakeUpSpecification.supportsIndexedWrite = true;
            registerImplementation(
                "wake-up", ImplementationKind::eSet,
                SetImplementationFunction{
                    [handle = m_handle, poseBindingHandle = poseBindingIterator->second](
                        const TensorDescription& data, const TensorDescription& indices)
                    {
                        const TensorDescription& indexSelection = !indices.isEmpty() ? indices : data;
                        if (indexSelection.isEmpty())
                        {
                            checkOvphysxResult(
                                ovphysx_rigid_body_view_wake_up(handle, poseBindingHandle, nullptr), "wake-up");
                            return;
                        }
                        // ovphysx_rigid_body_view_wake_up has no GPU implementation; refuse a device index buffer
                        // loudly rather than dereference it on the host. This specification leaves requiresHostData
                        // false, so a GPU buffer arrives as-is.
                        if (indexSelection.device == DeviceKind::eGpu)
                        {
                            throw std::runtime_error(
                                "wake-up: GPU index buffer not supported -- ovphysx_rigid_body_view_wake_up is "
                                "CPU-only; pass a CPU index array");
                        }
                        // ovphysx interprets the index buffer as int32; forwarding an int64 buffer as-is would
                        // misread its bytes (int64 1 -> int32 [1,0]), so reject non-int32 loudly like every other set
                        // path.
                        requireInt32Indices(indexSelection, "wake-up");
                        DLTensor indexTensor = convertToDLTensor(indexSelection);
                        checkOvphysxResult(
                            ovphysx_rigid_body_view_wake_up(handle, poseBindingHandle, &indexTensor), "wake-up");
                    } },
                wakeUpSpecification);
            registerImplementation(
                "put-to-sleep", ImplementationKind::eSet,
                SetImplementationFunction{
                    [handle = m_handle, poseBindingHandle = poseBindingIterator->second](
                        const TensorDescription& /*data*/, const TensorDescription& indices)
                    {
                        if (indices.isEmpty())
                        {
                            checkOvphysxResult(
                                ovphysx_rigid_body_view_sleep(handle, poseBindingHandle, nullptr), "put-to-sleep");
                            return;
                        }
                        // ovphysx_rigid_body_view_sleep is CPU-only and reads the buffer as int32 -- the same contract
                        // wake-up enforces above.
                        if (indices.device == DeviceKind::eGpu)
                        {
                            throw std::runtime_error(
                                "put-to-sleep: GPU index buffer not supported -- ovphysx_rigid_body_view_sleep is "
                                "CPU-only; pass a CPU index array");
                        }
                        requireInt32Indices(indices, "put-to-sleep");
                        DLTensor indexTensor = convertToDLTensor(indices);
                        checkOvphysxResult(
                            ovphysx_rigid_body_view_sleep(handle, poseBindingHandle, &indexTensor), "put-to-sleep");
                    } },
                wakeUpSpecification);
        }
    }
}

// Helper: get resolved rigid body prim paths from a RIGID_BODY_POSE binding.
static std::vector<std::string> getRigidBodyPrimPaths(ovphysx_handle_t handle,
                                                      const std::string& pattern,
                                                      const std::optional<std::vector<std::string>>& resolvedPaths)
{
    ovphysx_tensor_binding_desc_t descriptor{};
    descriptor.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    std::vector<ovphysx_string_t> pathStrings;
    setBindingSource(descriptor, pattern, resolvedPaths, pathStrings);
    ovphysx_tensor_binding_handle_t bindingHandle = 0;
    const ovphysx_result_t createResult = ovphysx_create_tensor_binding(handle, &descriptor, &bindingHandle);
    ScopedTensorBinding guard(handle, bindingHandle);
    if (createResult.status != OVPHYSX_API_SUCCESS || bindingHandle == 0)
    {
        return {};
    }

    ovphysx_tensor_spec_t pathSpecification{};
    checkSpecificationQueryStatus(
        ovphysx_get_tensor_binding_spec(handle, bindingHandle, &pathSpecification), "prim-paths specification");
    const uint32_t pathCapacity = (pathSpecification.ndim >= 1 && pathSpecification.shape[0] > 0) ?
                                      static_cast<uint32_t>(pathSpecification.shape[0]) :
                                      4096u;
    std::vector<ovphysx_string_t> paths(pathCapacity);
    uint32_t count = 0;
    ovphysx_tensor_binding_get_prim_paths(handle, bindingHandle, paths.data(), pathCapacity, &count);

    std::vector<std::string> result(count);
    for (uint32_t i = 0; i < count; ++i)
    {
        result[i] = std::string(paths[i].ptr, paths[i].length);
    }

    return result;
}

// ----------------------------------------------------------------------------
// Volume deformable body EntityView
// ----------------------------------------------------------------------------

static const std::vector<std::pair<std::string, ovphysx_tensor_type_t>> g_kVolumeDeformableBodyImplementations = {
    { "sim-nodal-positions", OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32 },
    { "sim-nodal-velocities", OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32 },
    { "sim-kinematic-targets", OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32 },
    { "rest-nodal-positions", OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32 },
    { "sim-element-indices", OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32 },
    { "collision-element-indices", OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32 },
};

OvPhysxVolumeDeformableBodyEntityView::OvPhysxVolumeDeformableBodyEntityView(ovphysx_handle_t handle,
                                                                             const std::string& pattern)
    : OvPhysxVolumeDeformableBodyEntityView(handle, pattern, std::nullopt)
{
}

OvPhysxVolumeDeformableBodyEntityView::OvPhysxVolumeDeformableBodyEntityView(ovphysx_handle_t handle,
                                                                             const std::vector<std::string>& patterns)
    // Resolve the list in caller order; a representative pattern backs the all-miss case.
    : OvPhysxVolumeDeformableBodyEntityView(
          handle,
          patterns.empty() ? std::string() : patterns.front(),
          resolvePatterns(handle, patterns, g_kVolumeDeformableBodyImplementations.front().second))
{
}

OvPhysxVolumeDeformableBodyEntityView::OvPhysxVolumeDeformableBodyEntityView(
    ovphysx_handle_t handle, std::string pattern, std::optional<std::vector<std::string>> resolvedPaths)
    : OvPhysxEntityViewBase(handle, resolvedPaths ? *resolvedPaths : std::vector<std::string>{ pattern })
{
    _initializeBindings(g_kVolumeDeformableBodyImplementations, pattern, resolvedPaths);
    _setEntityCountFromBindings();
}

// ----------------------------------------------------------------------------
// Surface deformable body EntityView
// ----------------------------------------------------------------------------

// Note: SURFACE_DEFORMABLE_SIM_KINEMATIC_TARGET (slot 142) is reserved but the
// backend returns "not supported" -- omit from implementation table so _initializeBindings skips it cleanly.
static const std::vector<std::pair<std::string, ovphysx_tensor_type_t>> g_kSurfaceDeformableBodyImplementations = {
    { "sim-positions", OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32 },
    { "sim-velocities", OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32 },
    { "rest-positions", OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32 },
    { "sim-element-indices", OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32 },
};

OvPhysxSurfaceDeformableBodyEntityView::OvPhysxSurfaceDeformableBodyEntityView(ovphysx_handle_t handle,
                                                                               const std::string& pattern)
    : OvPhysxSurfaceDeformableBodyEntityView(handle, pattern, std::nullopt)
{
}

OvPhysxSurfaceDeformableBodyEntityView::OvPhysxSurfaceDeformableBodyEntityView(ovphysx_handle_t handle,
                                                                               const std::vector<std::string>& patterns)
    // Resolve the list in caller order; a representative pattern backs the all-miss case.
    : OvPhysxSurfaceDeformableBodyEntityView(
          handle,
          patterns.empty() ? std::string() : patterns.front(),
          resolvePatterns(handle, patterns, g_kSurfaceDeformableBodyImplementations.front().second))
{
}

OvPhysxSurfaceDeformableBodyEntityView::OvPhysxSurfaceDeformableBodyEntityView(
    ovphysx_handle_t handle, std::string pattern, std::optional<std::vector<std::string>> resolvedPaths)
    : OvPhysxEntityViewBase(handle, resolvedPaths ? *resolvedPaths : std::vector<std::string>{ pattern })
{
    _initializeBindings(g_kSurfaceDeformableBodyImplementations, pattern, resolvedPaths);
    _setEntityCountFromBindings();
}

// ----------------------------------------------------------------------------
// Deformable material EntityView
// ----------------------------------------------------------------------------

static const std::vector<std::pair<std::string, ovphysx_tensor_type_t>> g_kDeformableMaterialImplementations = {
    { "youngs-modulus", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32 },
    { "poissons-ratio", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32 },
    { "dynamic-friction", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32 },
    { "elasticity-damping", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32 },
    { "bending-stiffness", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32 },
    { "thickness", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32 },
    { "bending-damping", OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32 },
};

OvPhysxDeformableMaterialEntityView::OvPhysxDeformableMaterialEntityView(ovphysx_handle_t handle,
                                                                         const std::string& pattern)
    : OvPhysxDeformableMaterialEntityView(handle, pattern, std::nullopt)
{
}

OvPhysxDeformableMaterialEntityView::OvPhysxDeformableMaterialEntityView(ovphysx_handle_t handle,
                                                                         const std::vector<std::string>& patterns)
    // Resolve the list in caller order; a representative pattern backs the all-miss case.
    : OvPhysxDeformableMaterialEntityView(
          handle,
          patterns.empty() ? std::string() : patterns.front(),
          resolvePatterns(handle, patterns, g_kDeformableMaterialImplementations.front().second))
{
}

OvPhysxDeformableMaterialEntityView::OvPhysxDeformableMaterialEntityView(
    ovphysx_handle_t handle, std::string pattern, std::optional<std::vector<std::string>> resolvedPaths)
    : OvPhysxEntityViewBase(handle, resolvedPaths ? *resolvedPaths : std::vector<std::string>{ pattern })
{
    _initializeBindings(g_kDeformableMaterialImplementations, pattern, resolvedPaths);
    _setEntityCountFromBindings();
}

// ----------------------------------------------------------------------------
// SDF shape EntityView
// ----------------------------------------------------------------------------

// Forward declaration -- defined below in the "Unsupported stub view" section.
static void registerUnsupportedGet(isaacsim::physics::tensors::EntityView& view, const char* implementationName);

OvPhysxSdfShapeEntityView::OvPhysxSdfShapeEntityView(ovphysx_handle_t handle,
                                                     const std::string& pattern,
                                                     int maximumQueryPointCount)
    : EntityView({ pattern }), m_handle(handle), m_maximumQueryPointCount(maximumQueryPointCount)
{
    if (handle == OVPHYSX_INVALID_HANDLE)
    {
        registerUnsupportedGet(*this, "distances-and-gradients");
        return;
    }

    ovphysx_sdf_view_handle_t sdfHandle = 0;
    ovphysx_result_t result = ovphysx_create_sdf_view(
        handle, ovphysx_cstr(pattern.c_str()), static_cast<uint32_t>(maximumQueryPointCount), &sdfHandle);
    if (result.status != OVPHYSX_API_SUCCESS || sdfHandle == 0)
    {
        registerUnsupportedGet(*this, "distances-and-gradients");
        return;
    }

    // Owned only once the constructor completes: everything below can throw, and a raw
    // assignment would leak the view (the derived destructor does not run on a throwing ctor).
    ScopedSdfView sdfViewGuard(handle, sdfHandle);

    uint32_t count = 0;
    ovphysx_sdf_view_get_count(handle, sdfHandle, &count);
    m_shapeCount = static_cast<int32_t>(count);
    setEntityCount(m_shapeCount);

    m_pattern = pattern;

    int32_t shapeCount = m_shapeCount;
    int32_t queryPointCount = m_maximumQueryPointCount;

    std::vector<int64_t> outputShape = { shapeCount, queryPointCount, 4 };

    TensorSpecification specification;
    specification.supports = true;
    specification.dtype = DType::eFloat32;
    specification.shapeHint = outputShape;
    specification.supportsIndexedRead = true; // query points are passed via the indices slot
    specification.supportsIndexedWrite = false;

    registerImplementation(
        "distances-and-gradients", ImplementationKind::eGet,
        GetImplementationFunction{
            [this](const TensorDescription& queryPoints, const TensorDescription& output) -> TensorDescription
            {
                if (queryPoints.isEmpty())
                {
                    return {};
                }

                // SDF evaluation is GPU-only: ovphysx_evaluate_sdf requires BOTH the query
                // and the output on the GPU (getSdfAndGradients checks each slot's device).
                // The C++ backend has no device allocator, so there is no host-staging path
                // like the read-tensor-binding GETs have -- the caller must pass a
                // preallocated GPU `out`. Refuse a CPU/absent out (or a CPU query) loudly
                // rather than let the engine fail with an opaque "SDF evaluation failed".
                const char* context = "distances-and-gradients";
                if (queryPoints.device != DeviceKind::eGpu)
                {
                    throw std::runtime_error(std::string(context) +
                                             ": SDF evaluation is GPU-only; query points must be on the GPU");
                }
                // Validate the query against its fixed [N, maxQ, 3] contract, symmetric to
                // requireMatchingOutput below: the engine reads the buffer as a flat, contiguous
                // PxVec3[N * maxQ] (ignoring dtype/shape/strides), so a wrong dtype, a wrong
                // element count, or a strided query is silently misread -- reject it here with
                // a clear message instead of the opaque "SDF evaluation failed".
                if (queryPoints.dtype != DType::eFloat32)
                {
                    throw std::runtime_error(std::string(context) + ": query points must be float32");
                }
                // The query states how many points per shape the caller intends to submit, so adopt that
                // extent before validating against it. Only its own axis is caller-chosen: N is the number of
                // shapes the view resolved and stays fixed.
                if (queryPoints.shape.size() == 3 && queryPoints.shape[0] == m_shapeCount)
                {
                    _ensureQueryPointCapacity(static_cast<int32_t>(queryPoints.shape[1]));
                }
                const std::vector<int64_t> outputShape = { m_shapeCount, m_maximumQueryPointCount, 4 };
                const size_t queryCount = checkedElementCount(queryPoints.shape, context);
                const size_t expectedQueryCount = checkedElementCount({ outputShape[0], outputShape[1], 3 }, context);
                if (queryCount != expectedQueryCount)
                {
                    throw std::runtime_error(std::string(context) + ": query points must be [N, maxQ, 3]");
                }
                if (!isContiguousRowMajor(queryPoints))
                {
                    throw std::runtime_error(std::string(context) + ": query points must be C-contiguous");
                }
                if (output.isEmpty())
                {
                    throw std::runtime_error(std::string(context) +
                                             ": SDF evaluation is GPU-only; provide a preallocated GPU out buffer");
                }
                if (output.device != DeviceKind::eGpu)
                {
                    throw std::runtime_error(std::string(context) +
                                             ": SDF evaluation is GPU-only; out buffer must be on the GPU");
                }
                requireMatchingOutput(output, outputShape, DType::eFloat32, context);

                DLTensor queryTensor = convertToDLTensor(queryPoints);

                DLTensor outputTensor{};
                outputTensor.data = output.data;
                outputTensor.device = { kDLCUDA, output.deviceOrdinal };
                outputTensor.dtype = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                outputTensor.ndim = 3;
                outputTensor.shape = const_cast<int64_t*>(outputShape.data());
                outputTensor.strides = nullptr;

                checkOvphysxResult(
                    ovphysx_evaluate_sdf(m_handle, m_sdfHandle, &queryTensor, &outputTensor), "ovphysx_evaluate_sdf");

                TensorDescription result;
                result.data = output.data;
                result.dtype = DType::eFloat32;
                result.shape = outputShape;
                result.device = DeviceKind::eGpu;
                result.deviceOrdinal = output.deviceOrdinal;
                return result;
            } },
        specification);

    m_sdfHandle = sdfViewGuard.release();
}

void OvPhysxSdfShapeEntityView::_ensureQueryPointCapacity(int32_t requestedQueryPointCount)
{
    if (requestedQueryPointCount <= 0 || requestedQueryPointCount == m_maximumQueryPointCount || m_sdfHandle == 0 ||
        m_handle == OVPHYSX_INVALID_HANDLE)
    {
        return;
    }

    ovphysx_sdf_view_handle_t resizedSdfHandle = 0;
    checkOvphysxResult(ovphysx_create_sdf_view(m_handle, ovphysx_cstr(m_pattern.c_str()),
                                               static_cast<uint32_t>(requestedQueryPointCount), &resizedSdfHandle),
                       "sdf-view-resize");
    if (resizedSdfHandle == 0)
    {
        throw std::runtime_error("sdf-view-resize: ovphysx returned a null SDF view handle");
    }
    // Only swap once the replacement exists, so a failed resize leaves the view usable at its old extent.
    ovphysx_destroy_sdf_view(m_handle, m_sdfHandle);
    m_sdfHandle = resizedSdfHandle;
    m_maximumQueryPointCount = requestedQueryPointCount;

    _setImplementationShapeHint(
        "distances-and-gradients", ImplementationKind::eGet, { m_shapeCount, m_maximumQueryPointCount, 4 });
}

int OvPhysxSdfShapeEntityView::getDeviceOrdinal() const noexcept
{
    return resolveOvPhysxDeviceOrdinal(*this);
}

OvPhysxSdfShapeEntityView::~OvPhysxSdfShapeEntityView()
{
    if (m_sdfHandle != 0 && m_handle != OVPHYSX_INVALID_HANDLE)
    {
        ovphysx_destroy_sdf_view(m_handle, m_sdfHandle);
        m_sdfHandle = 0;
    }
}

// ----------------------------------------------------------------------------
// Unsupported stub view
// ----------------------------------------------------------------------------

std::shared_ptr<isaacsim::physics::tensors::EntityView> makeUnsupportedView(const std::string& pattern,
                                                                            const std::string& category)
{
    auto view = std::make_shared<EntityView>(std::vector<std::string>{ pattern });
    TensorSpecification specification;
    specification.supports = false;
    std::string placeholderImplementation = category + "-data";
    view->registerImplementation(placeholderImplementation, ImplementationKind::eGet,
                                 GetImplementationFunction{ [](const TensorDescription& /*indices*/,
                                                               const TensorDescription& output) { return output; } },
                                 specification);
    return view;
}

// One resolved output of a contact multi-get read: the buffer the engine writes into and the device
// it lives on. Shared by net/raw/contact/friction multi-gets so out-handling is identical everywhere.
struct OutputSlot
{
    void* data;
    bool gpu;
    int32_t ordinal;
};

// Resolve a multi-get output slot: when the framework supplied a caller `out[i]` (a GPU buffer on the
// DirectGPU lane, where ovphysx requires a device-matching destination), validate it against its per-output
// contract (dtype / count / contiguity / ordinal -- same guard as the single-buffer path) and use it;
// otherwise fall back to the persistent host buffer.
static OutputSlot resolveOutputSlot(const std::vector<TensorDescription>& output,
                                    size_t outputIndex,
                                    void* hostBuffer,
                                    const std::vector<int64_t>& shape,
                                    DType dtype,
                                    const char* label)
{
    if (output.size() > outputIndex && output[outputIndex].data)
    {
        requireMatchingOutput(output[outputIndex], shape, dtype, label);
        const bool gpu = output[outputIndex].device == DeviceKind::eGpu;
        return { output[outputIndex].data, gpu, gpu ? output[outputIndex].deviceOrdinal : -1 };
    }
    return { hostBuffer, false, -1 };
}

// The caller states the contact-record capacity through the first axis of the payload outputs, which are the
// outputs holding one row per contact record. Returns 0 when the caller supplied no `out` or the payload
// outputs disagree with each other; the per-slot validation in resolveOutputSlot then reports the mismatch
// with its own message rather than this silently adopting one of the sizes.
static int32_t getRequestedContactCapacity(const std::vector<TensorDescription>& output,
                                           const std::vector<size_t>& payloadSlots)
{
    int32_t capacity = 0;
    for (const size_t payloadSlot : payloadSlots)
    {
        if (output.size() <= payloadSlot || !output[payloadSlot].data || output[payloadSlot].shape.empty())
        {
            return 0;
        }
        const int32_t slotCapacity = static_cast<int32_t>(output[payloadSlot].shape[0]);
        if (capacity != 0 && slotCapacity != capacity)
        {
            return 0;
        }
        capacity = slotCapacity;
    }
    return capacity;
}

static DLTensor makeSlotDLTensor(const OutputSlot& slot, const std::vector<int64_t>& shape, DLDataType dataType)
{
    DLTensor tensor{};
    tensor.data = slot.data;
    tensor.device = slot.gpu ? DLDevice{ kDLCUDA, slot.ordinal } : DLDevice{ kDLCPU, 0 };
    tensor.dtype = dataType;
    tensor.ndim = static_cast<int32_t>(shape.size());
    tensor.shape = const_cast<int64_t*>(shape.data());
    tensor.strides = nullptr;
    return tensor;
}

static TensorDescription makeSlotTensorDescriptor(const OutputSlot& slot, const std::vector<int64_t>& shape, DType dataType)
{
    TensorDescription result;
    result.data = slot.data;
    result.dtype = dataType;
    result.shape = shape;
    result.device = slot.gpu ? DeviceKind::eGpu : DeviceKind::eCpu;
    result.deviceOrdinal = slot.gpu ? slot.ordinal : -1;
    return result;
}

// Per-output TensorSpecification (shape + dtype) for a multi-get registration -- lets the framework
// pre-allocate a device buffer per output on the DirectGPU lane.
static TensorSpecification makeOutputSpecification(std::vector<int64_t> shape, DType dataType)
{
    TensorSpecification specification;
    specification.supports = true;
    specification.dtype = dataType;
    specification.shapeHint = std::move(shape);
    return specification;
}

// Multi-get out-buffer contract: a caller supplies either every output or none
// (the framework passes 0 or `expected`). Reject a partial list -- wrong size or
// any null slot -- clearly rather than silently host-staging the rest, matching
// the single-buffer path's all-or-nothing `out`.
static void requireFullOrEmptyOutput(const std::vector<TensorDescription>& output,
                                     size_t expectedOutputCount,
                                     const char* label)
{
    if (output.empty())
    {
        return;
    }
    if (output.size() != expectedOutputCount)
    {
        throw std::runtime_error(std::string(label) + ": expected " + std::to_string(expectedOutputCount) +
                                 " output buffers (all or none), got " + std::to_string(output.size()));
    }
    for (size_t i = 0; i < expectedOutputCount; ++i)
    {
        if (!output[i].data)
        {
            throw std::runtime_error(std::string(label) + ": output " + std::to_string(i) +
                                     " is null (supply all output buffers or none)");
        }
    }
}

// Register a GET implementation that reports supports=false (the "unsupported" stub).
static void registerUnsupportedGet(EntityView& view, const char* implementationName)
{
    TensorSpecification unsupportedSpecification;
    unsupportedSpecification.supports = false;
    view.registerImplementation(
        implementationName, ImplementationKind::eGet,
        GetImplementationFunction{ [](const TensorDescription&, const TensorDescription& output) { return output; } },
        unsupportedSpecification);
}

// ----------------------------------------------------------------------------
// Rigid contact EntityView
// ----------------------------------------------------------------------------

OvPhysxRigidContactEntityView::OvPhysxRigidContactEntityView(ovphysx_handle_t handle,
                                                             const std::string& sensorPattern,
                                                             const std::vector<std::string>& filterPatterns,
                                                             int maximumContactDataCount)
    : EntityView({ sensorPattern }), m_handle(handle)
{
    if (handle == OVPHYSX_INVALID_HANDLE)
    {
        // Degenerate: register net-contact-forces and contact-force-matrix as unsupported.
        registerUnsupportedGet(*this, "net-contact-forces");
        registerUnsupportedGet(*this, "contact-force-matrix");
        return;
    }

    // Build sensor + filter pattern arrays for the C API.
    // The Python normalizer may have joined multiple explicit paths with '|'
    // and a "prim:" prefix. Strip the prefix (ovphysx doesn't know it) then
    // split on '|' so each path becomes a separate sensor pattern.
    std::vector<std::string> sensorStrings;
    {
        static const std::string s_kPrimPrefix = "prim:";
        std::string_view sensorList(sensorPattern);
        if (sensorList.substr(0, s_kPrimPrefix.size()) == s_kPrimPrefix)
        {
            sensorList = sensorList.substr(s_kPrimPrefix.size());
            size_t start = 0, delimiterPosition = 0;
            while ((delimiterPosition = sensorList.find('|', start)) != std::string_view::npos)
            {
                sensorStrings.emplace_back(sensorList.substr(start, delimiterPosition - start));
                start = delimiterPosition + 1;
            }
            sensorStrings.emplace_back(sensorList.substr(start));
        }
        else
        {
            sensorStrings.emplace_back(std::string(sensorList));
        }
    }
    std::vector<ovphysx_string_t> sensorPatterns;
    sensorPatterns.reserve(sensorStrings.size());
    for (auto& sensorString : sensorStrings)
    {
        sensorPatterns.push_back(ovphysx_cstr(sensorString.c_str()));
    }

    // C API expects filter_patterns to have sensor_count * filters_per_sensor entries.
    // Replicate the caller-supplied filter list once per sensor when there are multiple sensors.
    std::vector<std::string> filterStrings;
    {
        const size_t filtersPerSensor = filterPatterns.size();
        const size_t sensorCount = sensorStrings.size();
        filterStrings.reserve(checkedSizeProduct(filtersPerSensor, sensorCount > 0 ? sensorCount : 1u, "contact filters"));
        if (sensorCount <= 1 || filtersPerSensor == 0)
        {
            filterStrings = filterPatterns;
        }
        else
        {
            for (size_t i = 0; i < sensorCount; ++i)
            {
                filterStrings.insert(filterStrings.end(), filterPatterns.begin(), filterPatterns.end());
            }
        }
    }
    std::vector<ovphysx_string_t> filterPatternViews;
    filterPatternViews.reserve(filterStrings.size());
    for (auto& filterString : filterStrings)
    {
        filterPatternViews.push_back(ovphysx_cstr(filterString.c_str()));
    }

    const size_t sensorCount = sensorStrings.size();
    uint32_t filtersPerSensor =
        filterPatternViews.empty() ?
            0u :
            static_cast<uint32_t>(filterPatternViews.size() / (sensorCount > 0 ? sensorCount : 1u));

    ovphysx_contact_binding_handle_t contactBinding = 0;
    ovphysx_result_t result = ovphysx_create_contact_binding(
        m_handle, sensorPatterns.data(), static_cast<uint32_t>(sensorPatterns.size()),
        filterPatternViews.empty() ? nullptr : filterPatternViews.data(), filtersPerSensor,
        static_cast<uint32_t>(std::max(0, maximumContactDataCount)), &contactBinding);

    ScopedContactBinding contactBindingGuard(m_handle, contactBinding);
    if (result.status != OVPHYSX_API_SUCCESS || contactBinding == 0)
    {
        registerUnsupportedGet(*this, "net-contact-forces");
        registerUnsupportedGet(*this, "contact-force-matrix");
        return;
    }

    m_maximumContactDataCount = maximumContactDataCount;
    m_sensorStrings = sensorStrings;
    m_filterStrings = filterStrings;
    m_filtersPerSensor = filtersPerSensor;
    m_buffers = std::make_shared<ContactBuffers>();

    // Query dimensions for TensorSpecification shape hints.
    checkOvphysxResult(ovphysx_get_contact_binding_spec(m_handle, contactBinding, &m_sensorCount, &m_filterCount),
                       "contact-binding-specification");
    if (m_sensorCount < 0 || m_filterCount < 0)
    {
        throw std::runtime_error("contact-binding-specification: ovphysx returned a negative dimension");
    }
    setEntityCount(m_sensorCount);

    // Contact metadata: sensor prim paths (`sensor_names`) + filter count
    // (`filter_count`), exposed via get_metadata(...) for consumers
    // (the row order matches contact-binding read order).
    {
        std::vector<std::string> sensorNames;
        if (m_sensorCount > 0)
        {
            std::vector<ovphysx_string_t> paths(static_cast<size_t>(m_sensorCount));
            uint32_t count = 0;
            ovphysx_contact_binding_get_sensor_paths(
                m_handle, contactBinding, paths.data(), static_cast<uint32_t>(m_sensorCount), &count);
            for (uint32_t i = 0; i < count; ++i)
            {
                sensorNames.emplace_back(paths[i].ptr, paths[i].length);
            }
        }
        registerMetadata("sensor-names", [sensorNames]() -> Metadata { return sensorNames; });
        registerMetadata("num-filters", [value = static_cast<int64_t>(m_filterCount)]() -> Metadata { return value; });
        registerMetadata("num-sensors", [value = static_cast<int64_t>(m_sensorCount)]() -> Metadata { return value; });
    }

    int32_t registeredSensorCount = m_sensorCount;
    int32_t filterCount = m_filterCount;

    // "net-contact-forces": shape [S, 3]
    {
        TensorSpecification specification;
        specification.dtype = DType::eFloat32;
        specification.shapeHint = { registeredSensorCount, 3 };
        specification.supports = true;
        specification.supportsIndexedRead = false;
        specification.supportsIndexedWrite = false;

        std::vector<int64_t> netForceShape = { registeredSensorCount, 3 };
        auto netForceBuffer = std::make_shared<std::vector<float>>(
            checkedElementCount({ registeredSensorCount, 3 }, "net-contact-forces"), 0.0f);
        registerImplementation(
            "net-contact-forces", ImplementationKind::eGet,
            GetImplementationFunction{
                [this, netForceBuffer, netForceShape](
                    const TensorDescription& /*indices*/, const TensorDescription& output) mutable -> TensorDescription
                {
                    // Read into the caller `out` when given (the framework hands a GPU
                    // buffer on the DirectGPU/suppressReadback lane, where ovphysx
                    // requires a device-matching destination); otherwise stage to the host buffer.
                    const bool hasOutput = !output.isEmpty() && output.data;
                    const bool outputOnGpu = hasOutput && output.device == DeviceKind::eGpu;
                    if (hasOutput)
                    {
                        requireMatchingOutput(output, netForceShape, DType::eFloat32, "net-contact-forces");
                    }
                    void* outputData = hasOutput ? output.data : static_cast<void*>(netForceBuffer->data());

                    DLTensor destination{};
                    destination.data = outputData;
                    destination.device = outputOnGpu ? DLDevice{ kDLCUDA, output.deviceOrdinal } : DLDevice{ kDLCPU, 0 };
                    destination.dtype = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                    destination.ndim = 2;
                    destination.shape = const_cast<int64_t*>(netForceShape.data());
                    destination.strides = nullptr;
                    checkOvphysxResult(ovphysx_read_contact_net_forces(m_handle, m_contactBinding, &destination),
                                       "net-contact-forces");

                    TensorDescription result;
                    result.data = outputData;
                    result.dtype = DType::eFloat32;
                    result.shape = netForceShape;
                    result.device = outputOnGpu ? DeviceKind::eGpu : DeviceKind::eCpu;
                    result.deviceOrdinal = outputOnGpu ? output.deviceOrdinal : -1;
                    return result;
                } },
            specification);
    }

    // "contact-force-matrix": shape [S, F, 3] (requires filters)
    {
        TensorSpecification specification;
        specification.dtype = DType::eFloat32;
        specification.shapeHint = { registeredSensorCount, filterCount > 0 ? filterCount : 1, 3 };
        specification.supports = (filterCount > 0);
        specification.supportsIndexedRead = false;
        specification.supportsIndexedWrite = false;

        std::vector<int64_t> forceMatrixShape = { registeredSensorCount, filterCount > 0 ? filterCount : 1, 3 };
        auto forceMatrixBuffer = std::make_shared<std::vector<float>>(
            checkedElementCount({ registeredSensorCount, filterCount > 0 ? filterCount : 1, 3 }, "contact-force-matrix"),
            0.0f);
        registerImplementation(
            "contact-force-matrix", ImplementationKind::eGet,
            GetImplementationFunction{
                [this, forceMatrixBuffer, forceMatrixShape, hasFilters = (filterCount > 0)](
                    const TensorDescription& /*indices*/, const TensorDescription& output) mutable -> TensorDescription
                {
                    if (!hasFilters)
                    {
                        TensorDescription empty;
                        return empty;
                    }
                    // Honor a caller `out` device (GPU on the DirectGPU lane); else host buffer.
                    const bool hasOutput = !output.isEmpty() && output.data;
                    const bool outputOnGpu = hasOutput && output.device == DeviceKind::eGpu;
                    if (hasOutput)
                    {
                        requireMatchingOutput(output, forceMatrixShape, DType::eFloat32, "contact-force-matrix");
                    }
                    void* outputData = hasOutput ? output.data : static_cast<void*>(forceMatrixBuffer->data());

                    DLTensor destination{};
                    destination.data = outputData;
                    destination.device = outputOnGpu ? DLDevice{ kDLCUDA, output.deviceOrdinal } : DLDevice{ kDLCPU, 0 };
                    destination.dtype = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                    destination.ndim = 3;
                    destination.shape = const_cast<int64_t*>(forceMatrixShape.data());
                    destination.strides = nullptr;
                    checkOvphysxResult(ovphysx_read_contact_force_matrix(m_handle, m_contactBinding, &destination),
                                       "contact-force-matrix");

                    TensorDescription result;
                    result.data = outputData;
                    result.dtype = DType::eFloat32;
                    result.shape = forceMatrixShape;
                    result.device = outputOnGpu ? DeviceKind::eGpu : DeviceKind::eCpu;
                    result.deviceOrdinal = outputOnGpu ? output.deviceOrdinal : -1;
                    return result;
                } },
            specification);
    }

    // "contact-data": 6 tensors -- forces [contactCapacity,1], points [contactCapacity,3], normals [contactCapacity,3],
    // separations [contactCapacity,1], counts [registeredSensorCount,filterCount], start_indices
    // [registeredSensorCount,filterCount]. "friction-data": 4 tensors -- forces [contactCapacity,3], points
    // [contactCapacity,3], counts [registeredSensorCount,filterCount], starts [registeredSensorCount,filterCount]. Only
    // meaningful when max_contact_data_count > 0 and filterCount > 0.
    if (maximumContactDataCount > 0 && filterCount > 0)
    {
        int32_t contactCapacity = maximumContactDataCount;

        // Pre-allocate all contact-data output buffers.
        const size_t scalarContactElementCount = checkedElementCount({ contactCapacity, 1 }, "contact-data");
        const size_t vectorContactElementCount = checkedElementCount({ contactCapacity, 3 }, "contact-data");
        const size_t sensorFilterElementCount =
            checkedElementCount({ registeredSensorCount, filterCount }, "contact-data");
        m_buffers->contactForce.assign(scalarContactElementCount, 0.0f);
        m_buffers->contactPoint.assign(vectorContactElementCount, 0.0f);
        m_buffers->contactNormal.assign(vectorContactElementCount, 0.0f);
        m_buffers->contactSeparation.assign(scalarContactElementCount, 0.0f);
        m_buffers->contactCount.assign(sensorFilterElementCount, 0);
        m_buffers->contactStartIndex.assign(sensorFilterElementCount, 0);

        std::vector<int64_t> scalarContactShape = { contactCapacity, 1 }, vectorContactShape = { contactCapacity, 3 };
        std::vector<int64_t> sensorFilterShape = { registeredSensorCount, filterCount };

        std::vector<TensorSpecification> contactDataSpecifications = {
            makeOutputSpecification(scalarContactShape, DType::eFloat32),
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(scalarContactShape, DType::eFloat32),
            makeOutputSpecification(sensorFilterShape, DType::eInt32),
            makeOutputSpecification(sensorFilterShape, DType::eInt32)
        };
        registerImplementation(
            "contact-data", ImplementationKind::eGet,
            GetMultiImplementationFunction{
                [this, sensorFilterShape](const TensorDescription&, const std::vector<TensorDescription>& output) mutable
                -> std::vector<TensorDescription>
                {
                    const char* context = "contact-data";
                    requireFullOrEmptyOutput(output, 6, context);
                    // Adopt the caller's extent before validating against it, so the shapes below describe the
                    // buffers they actually passed.
                    _ensureContactCapacity(getRequestedContactCapacity(output, { 0, 1, 2, 3 }));
                    const std::vector<int64_t> scalarContactShape = { m_maximumContactDataCount, 1 };
                    const std::vector<int64_t> vectorContactShape = { m_maximumContactDataCount, 3 };
                    OutputSlot forceSlot = resolveOutputSlot(
                        output, 0, m_buffers->contactForce.data(), scalarContactShape, DType::eFloat32, context);
                    OutputSlot pointSlot = resolveOutputSlot(
                        output, 1, m_buffers->contactPoint.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot normalSlot = resolveOutputSlot(
                        output, 2, m_buffers->contactNormal.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot separationSlot = resolveOutputSlot(
                        output, 3, m_buffers->contactSeparation.data(), scalarContactShape, DType::eFloat32, context);
                    OutputSlot countSlot = resolveOutputSlot(
                        output, 4, m_buffers->contactCount.data(), sensorFilterShape, DType::eInt32, context);
                    OutputSlot startIndexSlot = resolveOutputSlot(
                        output, 5, m_buffers->contactStartIndex.data(), sensorFilterShape, DType::eInt32, context);
                    DLDataType float32DataType = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                    DLDataType int32DataType = { static_cast<uint8_t>(kDLInt), 32, 1 };
                    DLTensor forceTensor = makeSlotDLTensor(forceSlot, scalarContactShape, float32DataType);
                    DLTensor pointTensor = makeSlotDLTensor(pointSlot, vectorContactShape, float32DataType);
                    DLTensor normalTensor = makeSlotDLTensor(normalSlot, vectorContactShape, float32DataType);
                    DLTensor separationTensor = makeSlotDLTensor(separationSlot, scalarContactShape, float32DataType);
                    DLTensor countTensor = makeSlotDLTensor(countSlot, sensorFilterShape, int32DataType);
                    DLTensor startIndexTensor = makeSlotDLTensor(startIndexSlot, sensorFilterShape, int32DataType);
                    checkOvphysxResult(
                        ovphysx_read_contact_data(m_handle, m_contactBinding, &forceTensor, &pointTensor, &normalTensor,
                                                  &separationTensor, &countTensor, &startIndexTensor),
                        context);
                    return { makeSlotTensorDescriptor(forceSlot, scalarContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(pointSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(normalSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(separationSlot, scalarContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(countSlot, sensorFilterShape, DType::eInt32),
                             makeSlotTensorDescriptor(startIndexSlot, sensorFilterShape, DType::eInt32) };
                } },
            contactDataSpecifications);

        // Pre-allocate friction-data buffers.
        m_buffers->frictionForce.assign(vectorContactElementCount, 0.0f);
        m_buffers->frictionPoint.assign(vectorContactElementCount, 0.0f);
        m_buffers->frictionCount.assign(sensorFilterElementCount, 0);
        m_buffers->frictionStartIndex.assign(sensorFilterElementCount, 0);

        std::vector<TensorSpecification> frictionDataSpecifications = {
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(sensorFilterShape, DType::eInt32),
            makeOutputSpecification(sensorFilterShape, DType::eInt32)
        };
        registerImplementation(
            "friction-data", ImplementationKind::eGet,
            GetMultiImplementationFunction{
                [this, sensorFilterShape](const TensorDescription&, const std::vector<TensorDescription>& output) mutable
                -> std::vector<TensorDescription>
                {
                    const char* context = "friction-data";
                    requireFullOrEmptyOutput(output, 4, context);
                    _ensureContactCapacity(getRequestedContactCapacity(output, { 0, 1 }));
                    const std::vector<int64_t> vectorContactShape = { m_maximumContactDataCount, 3 };
                    OutputSlot forceSlot = resolveOutputSlot(
                        output, 0, m_buffers->frictionForce.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot pointSlot = resolveOutputSlot(
                        output, 1, m_buffers->frictionPoint.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot countSlot = resolveOutputSlot(
                        output, 2, m_buffers->frictionCount.data(), sensorFilterShape, DType::eInt32, context);
                    OutputSlot startIndexSlot = resolveOutputSlot(
                        output, 3, m_buffers->frictionStartIndex.data(), sensorFilterShape, DType::eInt32, context);
                    DLDataType float32DataType = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                    DLDataType int32DataType = { static_cast<uint8_t>(kDLInt), 32, 1 };
                    DLTensor frictionForceTensor = makeSlotDLTensor(forceSlot, vectorContactShape, float32DataType);
                    DLTensor frictionPointTensor = makeSlotDLTensor(pointSlot, vectorContactShape, float32DataType);
                    DLTensor frictionCountTensor = makeSlotDLTensor(countSlot, sensorFilterShape, int32DataType);
                    DLTensor frictionStartIndexTensor =
                        makeSlotDLTensor(startIndexSlot, sensorFilterShape, int32DataType);
                    checkOvphysxResult(ovphysx_read_friction_data(m_handle, m_contactBinding, &frictionForceTensor,
                                                                  &frictionPointTensor, &frictionCountTensor,
                                                                  &frictionStartIndexTensor),
                                       context);
                    return { makeSlotTensorDescriptor(forceSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(pointSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(countSlot, sensorFilterShape, DType::eInt32),
                             makeSlotTensorDescriptor(startIndexSlot, sensorFilterShape, DType::eInt32) };
                } },
            frictionDataSpecifications);
    }
    else
    {
        // No max_contact_data_count or no filters: register as unsupported stubs.
        registerUnsupportedGet(*this, "contact-data");
        registerUnsupportedGet(*this, "friction-data");
    }

    // "raw-contact-data": unfiltered per-sensor contact data (7 outputs).
    // Always available when max_contact_data_count > 0 (no filters needed).
    if (maximumContactDataCount > 0)
    {
        int32_t contactCapacity = maximumContactDataCount;
        // Allocate 7 pre-allocated buffers for ovphysx_read_raw_contact_data.
        const size_t scalarContactElementCount = checkedElementCount({ contactCapacity, 1 }, "raw-contact-data");
        const size_t vectorContactElementCount = checkedElementCount({ contactCapacity, 3 }, "raw-contact-data");
        const size_t sensorElementCount = checkedElementCount({ registeredSensorCount }, "raw-contact-data");
        m_buffers->rawForce.assign(scalarContactElementCount, 0.0f);
        m_buffers->rawPoint.assign(vectorContactElementCount, 0.0f);
        m_buffers->rawNormal.assign(vectorContactElementCount, 0.0f);
        m_buffers->rawSeparation.assign(scalarContactElementCount, 0.0f);
        m_buffers->rawCount.assign(sensorElementCount, 0);
        m_buffers->rawStartIndex.assign(sensorElementCount, 0);
        m_buffers->rawActorIdentifier.assign(scalarContactElementCount, 0);

        std::vector<int64_t> scalarContactShape = { contactCapacity, 1 }, vectorContactShape = { contactCapacity, 3 };
        std::vector<int64_t> sensorShape = { registeredSensorCount };
        // other_actor_ids is one entry per contact (max_contact_data_count),
        // not per sensor -- matches the legacy (max_c,) buffer shape.
        std::vector<int64_t> actorIdentifierShape = { contactCapacity };

        // Per-output specifications (shape + dtype) so the framework can pre-allocate a device buffer
        // per output on the DirectGPU lane (ovphysx requires a device-matching destination on each).
        std::vector<TensorSpecification> rawDataSpecifications = {
            makeOutputSpecification(scalarContactShape, DType::eFloat32),
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(vectorContactShape, DType::eFloat32),
            makeOutputSpecification(scalarContactShape, DType::eFloat32),
            makeOutputSpecification(sensorShape, DType::eInt32),
            makeOutputSpecification(sensorShape, DType::eInt32),
            makeOutputSpecification(actorIdentifierShape, DType::eInt64)
        };
        registerImplementation(
            "raw-contact-data", ImplementationKind::eGet,
            GetMultiImplementationFunction{
                [this, sensorShape](const TensorDescription&,
                                    const std::vector<TensorDescription>& output) mutable -> std::vector<TensorDescription>
                {
                    const char* context = "raw-contact-data";
                    requireFullOrEmptyOutput(output, 7, context);
                    _ensureContactCapacity(getRequestedContactCapacity(output, { 0, 1, 2, 3, 6 }));
                    const std::vector<int64_t> scalarContactShape = { m_maximumContactDataCount, 1 };
                    const std::vector<int64_t> vectorContactShape = { m_maximumContactDataCount, 3 };
                    const std::vector<int64_t> actorIdentifierShape = { m_maximumContactDataCount };
                    OutputSlot forceSlot = resolveOutputSlot(
                        output, 0, m_buffers->rawForce.data(), scalarContactShape, DType::eFloat32, context);
                    OutputSlot pointSlot = resolveOutputSlot(
                        output, 1, m_buffers->rawPoint.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot normalSlot = resolveOutputSlot(
                        output, 2, m_buffers->rawNormal.data(), vectorContactShape, DType::eFloat32, context);
                    OutputSlot separationSlot = resolveOutputSlot(
                        output, 3, m_buffers->rawSeparation.data(), scalarContactShape, DType::eFloat32, context);
                    OutputSlot countSlot =
                        resolveOutputSlot(output, 4, m_buffers->rawCount.data(), sensorShape, DType::eInt32, context);
                    OutputSlot startIndexSlot = resolveOutputSlot(
                        output, 5, m_buffers->rawStartIndex.data(), sensorShape, DType::eInt32, context);
                    OutputSlot actorIdentifierSlot = resolveOutputSlot(
                        output, 6, m_buffers->rawActorIdentifier.data(), actorIdentifierShape, DType::eInt64, context);
                    DLDataType float32DataType = { static_cast<uint8_t>(kDLFloat), 32, 1 };
                    DLDataType int32DataType = { static_cast<uint8_t>(kDLInt), 32, 1 };
                    DLDataType int64DataType = { static_cast<uint8_t>(kDLInt), 64, 1 };
                    DLTensor forceTensor = makeSlotDLTensor(forceSlot, scalarContactShape, float32DataType);
                    DLTensor pointTensor = makeSlotDLTensor(pointSlot, vectorContactShape, float32DataType);
                    DLTensor normalTensor = makeSlotDLTensor(normalSlot, vectorContactShape, float32DataType);
                    DLTensor separationTensor = makeSlotDLTensor(separationSlot, scalarContactShape, float32DataType);
                    DLTensor countTensor = makeSlotDLTensor(countSlot, sensorShape, int32DataType);
                    DLTensor startIndexTensor = makeSlotDLTensor(startIndexSlot, sensorShape, int32DataType);
                    DLTensor actorIdentifierTensor =
                        makeSlotDLTensor(actorIdentifierSlot, actorIdentifierShape, int64DataType);
                    checkOvphysxResult(ovphysx_read_raw_contact_data(
                                           m_handle, m_contactBinding, &forceTensor, &pointTensor, &normalTensor,
                                           &separationTensor, &countTensor, &startIndexTensor, &actorIdentifierTensor),
                                       context);
                    return { makeSlotTensorDescriptor(forceSlot, scalarContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(pointSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(normalSlot, vectorContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(separationSlot, scalarContactShape, DType::eFloat32),
                             makeSlotTensorDescriptor(countSlot, sensorShape, DType::eInt32),
                             makeSlotTensorDescriptor(startIndexSlot, sensorShape, DType::eInt32),
                             makeSlotTensorDescriptor(actorIdentifierSlot, actorIdentifierShape, DType::eInt64) };
                } },
            rawDataSpecifications);
    }
    else
    {
        registerUnsupportedGet(*this, "raw-contact-data");
    }

    m_contactBinding = contactBindingGuard.release();
}

void OvPhysxRigidContactEntityView::_ensureContactCapacity(int32_t requestedCapacity)
{
    if (requestedCapacity <= 0 || requestedCapacity == m_maximumContactDataCount || m_contactBinding == 0 ||
        m_handle == OVPHYSX_INVALID_HANDLE)
    {
        return;
    }

    std::vector<ovphysx_string_t> sensorPatterns;
    sensorPatterns.reserve(m_sensorStrings.size());
    for (const std::string& sensorString : m_sensorStrings)
    {
        sensorPatterns.push_back(ovphysx_cstr(sensorString.c_str()));
    }
    std::vector<ovphysx_string_t> filterPatternViews;
    filterPatternViews.reserve(m_filterStrings.size());
    for (const std::string& filterString : m_filterStrings)
    {
        filterPatternViews.push_back(ovphysx_cstr(filterString.c_str()));
    }

    ovphysx_contact_binding_handle_t resizedBinding = 0;
    checkOvphysxResult(
        ovphysx_create_contact_binding(m_handle, sensorPatterns.data(), static_cast<uint32_t>(sensorPatterns.size()),
                                       filterPatternViews.empty() ? nullptr : filterPatternViews.data(),
                                       m_filtersPerSensor, static_cast<uint32_t>(requestedCapacity), &resizedBinding),
        "contact-binding-resize");
    if (resizedBinding == 0)
    {
        throw std::runtime_error("contact-binding-resize: ovphysx returned a null contact binding handle");
    }
    // Only swap once the replacement exists, so a failed resize leaves the view usable at its old capacity.
    // Destroying first would be unrecoverable rather than merely wrong: the early return above treats a zero
    // binding as nothing to resize, so the view could never rebuild and every later read would hand a null
    // handle to the engine.
    ovphysx_destroy_contact_binding(m_handle, m_contactBinding);
    m_contactBinding = resizedBinding;
    m_maximumContactDataCount = requestedCapacity;

    const size_t capacity = static_cast<size_t>(requestedCapacity);
    if (!m_buffers->contactForce.empty())
    {
        m_buffers->contactForce.resize(capacity);
        m_buffers->contactPoint.resize(capacity * 3);
        m_buffers->contactNormal.resize(capacity * 3);
        m_buffers->contactSeparation.resize(capacity);
        m_buffers->frictionForce.resize(capacity * 3);
        m_buffers->frictionPoint.resize(capacity * 3);
    }
    if (!m_buffers->rawForce.empty())
    {
        m_buffers->rawForce.resize(capacity);
        m_buffers->rawPoint.resize(capacity * 3);
        m_buffers->rawNormal.resize(capacity * 3);
        m_buffers->rawSeparation.resize(capacity);
        m_buffers->rawActorIdentifier.resize(capacity);
    }

    // The frontend sizes its own device allocations from these hints, so they have to track the binding
    // or the next read would hand back the previous capacity and undo the caller's choice.
    const std::vector<int64_t> scalarShape = { requestedCapacity, 1 };
    const std::vector<int64_t> vectorShape = { requestedCapacity, 3 };
    const std::vector<int64_t> actorIdentifierShape = { requestedCapacity };
    const std::vector<int64_t> sensorFilterShape = { m_sensorCount, m_filterCount };
    const std::vector<int64_t> sensorShape = { m_sensorCount };
    _setImplementationOutputShapeHints(
        "contact-data", ImplementationKind::eGet,
        { scalarShape, vectorShape, vectorShape, scalarShape, sensorFilterShape, sensorFilterShape });
    _setImplementationOutputShapeHints(
        "friction-data", ImplementationKind::eGet, { vectorShape, vectorShape, sensorFilterShape, sensorFilterShape });
    _setImplementationOutputShapeHints(
        "raw-contact-data", ImplementationKind::eGet,
        { scalarShape, vectorShape, vectorShape, scalarShape, sensorShape, sensorShape, actorIdentifierShape });
}

int OvPhysxRigidContactEntityView::getDeviceOrdinal() const noexcept
{
    return resolveOvPhysxDeviceOrdinal(*this);
}

OvPhysxRigidContactEntityView::~OvPhysxRigidContactEntityView()
{
    if (m_contactBinding != 0 && m_handle != OVPHYSX_INVALID_HANDLE)
    {
        ovphysx_destroy_contact_binding(m_handle, m_contactBinding);
        m_contactBinding = 0;
    }
}

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
