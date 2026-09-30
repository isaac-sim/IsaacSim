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

#include <isaacsim/physics/registration/tensors/Metadata.hpp>
#include <isaacsim/physics/registration/tensors/PhysicsEnums.hpp>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>
#include <isaacsim/physics/registration/tensors/TensorSpecification.hpp>
#include <isaacsim/physics/registration/tensors/TensorTypes.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/vector.h>

#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace registration
{
namespace details
{
namespace nb = nanobind;

using tensors::DeviceKind;
using tensors::DType;
using tensors::TensorDescription;

inline DType convertDLPackToDataType(const nb::dlpack::dtype& dataType)
{
    if (dataType.code == static_cast<uint8_t>(nb::dlpack::dtype_code::Float))
    {
        if (dataType.bits == 32)
        {
            return DType::eFloat32;
        }
        if (dataType.bits == 64)
        {
            return DType::eFloat64;
        }
    }
    else if (dataType.code == static_cast<uint8_t>(nb::dlpack::dtype_code::Int))
    {
        if (dataType.bits == 8)
        {
            return DType::eInt8;
        }
        if (dataType.bits == 16)
        {
            return DType::eInt16;
        }
        if (dataType.bits == 32)
        {
            return DType::eInt32;
        }
        if (dataType.bits == 64)
        {
            return DType::eInt64;
        }
    }
    else if (dataType.code == static_cast<uint8_t>(nb::dlpack::dtype_code::UInt))
    {
        if (dataType.bits == 8)
        {
            return DType::eUInt8;
        }
        if (dataType.bits == 16)
        {
            return DType::eUInt16;
        }
        if (dataType.bits == 32)
        {
            return DType::eUInt32;
        }
        if (dataType.bits == 64)
        {
            return DType::eUInt64;
        }
    }
    else if (dataType.code == static_cast<uint8_t>(nb::dlpack::dtype_code::Bool))
    {
        return DType::eBool;
    }
    return DType::eUnknown;
}

inline nb::dlpack::dtype convertDataTypeToDLPack(DType dataType)
{
    using nb::dlpack::dtype_code;
    switch (dataType)
    {
    case DType::eFloat32:
        return { static_cast<uint8_t>(dtype_code::Float), 32, 1 };
    case DType::eFloat64:
        return { static_cast<uint8_t>(dtype_code::Float), 64, 1 };
    case DType::eInt8:
        return { static_cast<uint8_t>(dtype_code::Int), 8, 1 };
    case DType::eInt16:
        return { static_cast<uint8_t>(dtype_code::Int), 16, 1 };
    case DType::eInt32:
        return { static_cast<uint8_t>(dtype_code::Int), 32, 1 };
    case DType::eInt64:
        return { static_cast<uint8_t>(dtype_code::Int), 64, 1 };
    case DType::eUInt8:
        return { static_cast<uint8_t>(dtype_code::UInt), 8, 1 };
    case DType::eUInt16:
        return { static_cast<uint8_t>(dtype_code::UInt), 16, 1 };
    case DType::eUInt32:
        return { static_cast<uint8_t>(dtype_code::UInt), 32, 1 };
    case DType::eUInt64:
        return { static_cast<uint8_t>(dtype_code::UInt), 64, 1 };
    case DType::eBool:
        return { static_cast<uint8_t>(dtype_code::Bool), 8, 1 };
    default:
        return { 0, 0, 0 };
    }
}

inline DeviceKind convertDLPackDeviceToDeviceKind(int deviceType)
{
    if (deviceType == nb::device::cpu::value)
    {
        return DeviceKind::eCpu;
    }
    if (deviceType == nb::device::cuda::value)
    {
        return DeviceKind::eGpu;
    }
    return DeviceKind::eCpu;
}

inline int convertDeviceKindToDLPack(DeviceKind deviceKind)
{
    switch (deviceKind)
    {
    case DeviceKind::eCpu:
        return nb::device::cpu::value;
    case DeviceKind::eGpu:
        return nb::device::cuda::value;
    default:
        return nb::device::cpu::value;
    }
}

// Convert a Python ndarray into a non-owning TensorDescription the C++ side can
// read/write. The underlying buffer stays owned by Python; implementations must not
// retain pointers past their invocation.
inline TensorDescription convertArrayToTensorDescriptor(const nb::ndarray<>& array)
{
    TensorDescription descriptor;
    descriptor.data = const_cast<void*>(array.data());
    descriptor.dtype = convertDLPackToDataType(array.dtype());
    descriptor.shape.reserve(array.ndim());
    for (size_t dimensionIndex = 0; dimensionIndex < array.ndim(); ++dimensionIndex)
    {
        descriptor.shape.push_back(static_cast<int64_t>(array.shape(dimensionIndex)));
    }
    descriptor.strides.reserve(array.ndim());
    for (size_t dimensionIndex = 0; dimensionIndex < array.ndim(); ++dimensionIndex)
    {
        descriptor.strides.push_back(array.stride(dimensionIndex));
    }
    descriptor.device = convertDLPackDeviceToDeviceKind(array.device_type());
    descriptor.deviceOrdinal = array.device_id();
    return descriptor;
}

// Reject a non-int32 INTEGER `indices` array at the raw EntityView boundary,
// mirroring the Python `_require_int32_indices` guard so the whole API rejects
// int64 identically -- a raw `EntityView` bypasses `_attach_warp_dispatch`, so
// without this an int64 index would be misread as int32. A float dtype passes
// (the SDF view rides its float32 query points through the indices slot); int32,
// bool, unknown, and an omitted index pass through to the registered
// implementation. Uses std::invalid_argument, which nanobind maps to a Python
// ValueError.
inline void requireInt32IndexDtype(const TensorDescription& indices, const char* operationName)
{
    switch (indices.dtype)
    {
    case DType::eInt8:
    case DType::eInt16:
    case DType::eInt64:
    case DType::eUInt8:
    case DType::eUInt16:
    case DType::eUInt32:
    case DType::eUInt64:
        throw std::invalid_argument(std::string(operationName) + ": indices must be int32 (the umbrella convention)");
    default:
        return;
    }
}

// Wrap a TensorDescription into a framework-agnostic ndarray (DLPack-typed) that
// aliases the same buffer. Returning generic `nb::ndarray<>` (rather than
// the numpy-typed flavor) means nanobind hands Python a DLPack capsule
// that carries correct device info for either CPU or CUDA buffers — no
// numpy.ndarray-over-GPU-memory trap. The Python frontends layer
// (`omni.physics.tensors.frontends`) wraps it into the caller's native
// tensor type (`wp.array`, `torch.Tensor`, `np.ndarray`) at the public
// `view.get_data(...)` boundary. Used on the *result* path.
inline nb::ndarray<> convertTensorDescriptorToArray(const TensorDescription& descriptor, nb::handle owner)
{
    if (descriptor.data == nullptr)
    {
        return nb::ndarray<>{};
    }

    std::vector<size_t> shape;
    shape.reserve(descriptor.shape.size());
    for (const int64_t dimension : descriptor.shape)
    {
        shape.push_back(static_cast<size_t>(dimension));
    }

    return nb::ndarray<>(descriptor.data, shape.size(), shape.data(), owner,
                         descriptor.strides.empty() ? nullptr : descriptor.strides.data(),
                         convertDataTypeToDLPack(descriptor.dtype), convertDeviceKindToDLPack(descriptor.device),
                         descriptor.deviceOrdinal);
}

// Backward-compatibility alias for the engine-callback paths (`wrapPythonGetCallback`,
// `wrapPythonSetCallback`, etc.). The callback may receive GPU data, so the engine
// adapter is responsible for wrapping the DLPack capsule into its native
// tensor type via the frontend (or `wp.from_dlpack` etc.) before
// forwarding to a method that expects a native tensor.
inline nb::ndarray<> convertTensorDescriptorToNumpy(const TensorDescription& descriptor, nb::handle owner)
{
    return convertTensorDescriptorToArray(descriptor, owner);
}

// Pin a Python object for the lifetime of a non-owning TensorDescription that aliases
// its storage: the returned shared_ptr holds a reference to the object, and its
// deleter re-acquires the GIL before releasing it (the last owner may drop on
// any thread). Attach it to `TensorDescription::keepAlive` so `data` outlives the call
// that produced it -- the `get_data` binding then forwards it to the result
// array's owner.
inline std::shared_ptr<void> createPythonKeepAlive(nb::object object)
{
    auto* retainedObject = new nb::object(std::move(object));
    return std::shared_ptr<void>(retainedObject,
                                 [](void* pointer) noexcept
                                 {
                                     nb::gil_scoped_acquire globalInterpreterLockAcquire;
                                     delete static_cast<nb::object*>(pointer);
                                 });
}

// Like `createPythonKeepAlive()`, but pins an `nb::ndarray` directly: a
// numpy-backed array keeps its Python owner referenced, and a DLPack-imported
// array owns the managed tensor -- so `data` stays valid whether the result was
// a live array or a single-use DLPack capsule (the consumed capsule itself is
// never an owner).
inline std::shared_ptr<void> createArrayKeepAlive(nb::ndarray<> array)
{
    auto* retainedArray = new nb::ndarray<>(std::move(array));
    return std::shared_ptr<void>(retainedArray,
                                 [](void* pointer) noexcept
                                 {
                                     nb::gil_scoped_acquire globalInterpreterLockAcquire;
                                     delete static_cast<nb::ndarray<>*>(pointer);
                                 });
}

// Convert a Python GET-result object into an OWNING TensorDescription, pinning whatever
// actually owns the aliased storage: a TensorDescription's own owner, or the imported
// ndarray for a bare ndarray / DLPack capsule. When the result aliases the
// caller-supplied output alias, drop the keepAlive so the binding pins the caller's
// real `out` -- the synthetic alias handed to the callback is non-owning.
[[maybe_unused]] inline TensorDescription createTensorDescriptionFromResultObject(nb::object object,
                                                                                  const TensorDescription* outputAlias)
{
    TensorDescription descriptor;
    try
    {
        descriptor = nb::cast<TensorDescription>(object);
        // A TensorDescription built from an ndarray carries its own owner (see the
        // TensorDescription ndarray constructor); pin the object itself only as a fallback.
        if (!descriptor.isEmpty() && !descriptor.keepAlive)
        {
            descriptor.keepAlive = createPythonKeepAlive(object);
        }
    }
    catch (const nb::cast_error&)
    {
        nb::ndarray<> array = nb::cast<nb::ndarray<>>(object);
        descriptor = convertArrayToTensorDescriptor(array);
        if (!descriptor.isEmpty())
        {
            descriptor.keepAlive = createArrayKeepAlive(std::move(array));
        }
    }
    if (outputAlias && !outputAlias->isEmpty() && descriptor.data == outputAlias->data)
    {
        descriptor.keepAlive.reset();
    }
    return descriptor;
}

} // namespace details
} // namespace registration
} // namespace physics
} // namespace isaacsim
