// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "isaacsim/common/array/Array.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <vector>

// A nanobind type caster that exchanges isaacsim::common::array::Array with Python as a wp.array.
// Include this header in any binding translation unit that returns or accepts an Array.
// The specialization below teaches nanobind how to convert both directions:
//   - C++ -> Python (from_cpp): zero-copy, via DLPack and warp.from_dlpack().
//   - Python -> C++ (from_python): zero-copy alias of a C-contiguous array-like source on either a
//     CPU or a CUDA device (shared memory); a bare Python scalar built directly as a 0-dim Array; or
//     a Python list/tuple copied via numpy.ascontiguousarray(). Non-contiguous, non-numeric, and
//     unsupported-dtype inputs are rejected (the conversion fails, surfacing as nanobind's standard
//     incompatible-arguments error).

namespace isaacsim
{
namespace common
{
namespace array
{
namespace details
{

// --- Native Python sequence support ------------------------------------------------------------
// nb::ndarray inputs (warp.array / numpy / DLPack) already carry shape, dtype and a contiguous
// data pointer, so they are aliased zero-copy with no walking. A plain Python list/tuple/scalar has
// no contiguous buffer; it is converted to a C-contiguous numpy array (numpy performs the shape and
// dtype inference and rejects ragged input) and then re-enters the zero-copy ndarray path.

// A list/tuple we should attempt to convert. Deliberately excludes str/bytes (also sequences).
inline bool isPySequence(nanobind::handle object)
{
    return PyList_Check(object.ptr()) || PyTuple_Check(object.ptr());
}

// A leaf Python scalar we know how to store.
inline bool isPyScalar(nanobind::handle object)
{
    return PyBool_Check(object.ptr()) || PyLong_Check(object.ptr()) || PyFloat_Check(object.ptr());
}

} // namespace details
} // namespace array
} // namespace common
} // namespace isaacsim

namespace nanobind
{
namespace detail
{

template <>
struct type_caster<isaacsim::common::array::Array>
{
    using Array = isaacsim::common::array::Array;
    using Value = Array;

    static constexpr auto Name = const_name("warp.array");

    template <typename T_>
    using Cast = movable_cast_t<T_>;

    template <typename T_>
    static constexpr bool can_cast()
    {
        return true;
    }

    // Array is not default-constructible, so (unlike NB_TYPE_CASTER) the caster stores the value in
    // an std::optional; this keeps the caster itself default-constructible as required by the
    // std::variant caster.
    std::optional<Array> value;

    explicit operator Value*()
    {
        return value ? &(*value) : nullptr;
    }
    explicit operator Value&()
    {
        return *value;
    }
    explicit operator Value&&()
    {
        return std::move(*value);
    }

    // Python -> C++: three paths. (1) An array-like source (warp.array or any DLPack/buffer-protocol
    // array) already exposes shape, dtype and a contiguous data pointer, so it is aliased zero-copy
    // by fromNdarray(). (2) A bare Python scalar is constructed directly, preserving a 0-dim shape.
    // (3) A list/tuple is converted to a numpy array and re-enters the zero-copy path. A rejected
    // input returns false, surfacing as nanobind's standard incompatible-arguments error.
    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
    {
        namespace ica = isaacsim::common::array;

        // Fast path: zero-copy alias of an array-like source.
        if (fromNdarray(src, flags, cleanup))
        {
            return true;
        }

        // Fallback A (scalar): a bare Python bool/int/float has no buffer to alias. Construct the
        // Array directly to preserve a 0-dim (scalar) shape; numpy.ascontiguousarray() would force
        // ndim >= 1 and turn it into a shape-(1,) array. The dtype mirrors numpy's scalar inference
        // (bool -> Bool, int -> Int64, float -> Float64). PyBool must be checked before PyLong since
        // bool is a subclass of int in CPython.
        if (ica::details::isPyScalar(src))
        {
            try
            {
                if (PyBool_Check(src.ptr()))
                {
                    value = ica::Array(cast<bool>(src), ica::Dtype::Bool());
                }
                else if (PyLong_Check(src.ptr()))
                {
                    value = ica::Array(cast<int64_t>(src), ica::Dtype::Int64());
                }
                else // PyFloat
                {
                    value = ica::Array(cast<double>(src), ica::Dtype::Float64());
                }
                return true;
            }
            catch (...)
            {
                // cast<int64_t>() throws on a Python int that overflows int64; reject cleanly.
                return false;
            }
        }

        // Fallback B (sequence): a list/tuple has no contiguous buffer to alias. Convert it to a
        // C-contiguous numpy array (numpy performs the shape/dtype inference and ragged-input
        // rejection) and re-enter the zero-copy ndarray path, which aliases the numpy buffer and
        // keeps it alive for the Array's lifetime.
        if (ica::details::isPySequence(src))
        {
            try
            {
                object numpy = module_::import_("numpy");
                object array = numpy.attr("ascontiguousarray")(src);

                // Only numeric arrays are supported. numpy produces dtype=object arrays for strings
                // or heterogeneous input (e.g. ["a", "b"] or [1, "b"]), and can also produce
                // complex/datetime kinds; accept only boolean ('b'), signed ('i'), unsigned ('u')
                // and floating ('f') kinds. The exact bit width is still validated downstream by
                // fromNdarray() -> fromDLPack() (which rejects e.g. float16), so this guard only
                // needs to screen out non-numeric kinds that could otherwise slip through.
                object kindObject = array.attr("dtype").attr("kind");
                Py_ssize_t kindLength = 0;
                const char* kind = PyUnicode_AsUTF8AndSize(kindObject.ptr(), &kindLength);
                if (!kind)
                {
                    PyErr_Clear(); // Don't leak a set error out of this noexcept converter.
                    return false;
                }
                if (kindLength != 1 || (kind[0] != 'b' && kind[0] != 'i' && kind[0] != 'u' && kind[0] != 'f'))
                {
                    return false;
                }

                return fromNdarray(array, flags, cleanup);
            }
            catch (...)
            {
                // numpy raises for ragged/inhomogeneous input (e.g. [[1, [2]], [3, [4]]]). A caught
                // nanobind python_error releases its stored Python error on destruction (we never
                // restore()), so the interpreter error flag is left clear.
                return false;
            }
        }
        return false;
    }

    // Import an array-like source (warp.array or any DLPack/buffer-protocol array) and alias its
    // buffer zero-copy into an Array, on the device the source already lives on (CPU or CUDA). Only
    // C-contiguous (row-major) input is accepted; non-contiguous/strided inputs are rejected (Array
    // has no stride support), surfacing as a clean conversion failure rather than a silent copy. The
    // resulting Array shares memory with the Python array (writes on either side are visible to the
    // other, until copy() or clone() is called).
    bool fromNdarray(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
    {
        namespace ica = isaacsim::common::array;

        make_caster<ndarray<>> caster;
        if (!caster.from_python(src, flags, cleanup))
        {
            return false;
        }
        ndarray<>& array = caster.value;

        // Everything below runs inside the try: fromDLPack() validates the imported metadata and
        // throws on unsupported inputs, while this function is noexcept.
        try
        {
            const nanobind::dlpack::dtype arrayDtype = array.dtype();
            DLTensor tensor{};
            tensor.data = array.data();
            tensor.device = { static_cast<DLDeviceType>(array.device_type()), array.device_id() };
            tensor.ndim = static_cast<int32_t>(array.ndim());
            tensor.dtype = { arrayDtype.code, arrayDtype.bits, arrayDtype.lanes };
            tensor.shape = const_cast<int64_t*>(array.shape_ptr());
            tensor.strides = const_cast<int64_t*>(array.stride_ptr());
            tensor.byte_offset = 0;

            const ica::Array borrowed = ica::fromDLPack(tensor);

            // Zero-copy alias of the source buffer (a device pointer for a CUDA source, which
            // fromBuffer() adopts as device storage). Keep both the original argument and nanobind's
            // imported ndarray owner alive for the lifetime of the Array (and any view that shares
            // its buffer). A raw PyObject* is captured (rather than an nb::object) so the decref
            // happens exactly once, inside the deleter, under the GIL — capturing an nb::object
            // would make the lambda's own destructor decref without the GIL held. The deleter may
            // run on any thread, at any time, so it re-acquires the GIL; the Py_IsInitialized()
            // guard covers the case where an Array outlives interpreter finalization.
            object imported = cast(array);
            PyObject* keepSource = src.ptr();
            PyObject* keepImported = imported.ptr();
            Py_INCREF(keepSource);
            Py_INCREF(keepImported);
            std::shared_ptr<std::byte[]> aliased(static_cast<std::byte*>(array.data()),
                                                 [keepSource, keepImported](std::byte*) noexcept
                                                 {
                                                     if (!Py_IsInitialized())
                                                     {
                                                         return;
                                                     }
                                                     gil_scoped_acquire gil;
                                                     Py_DECREF(keepImported);
                                                     Py_DECREF(keepSource);
                                                 });

            value = ica::Array::fromBuffer(std::move(aliased), borrowed.shape(), borrowed.dtype(), borrowed.device());
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // C++ -> Python: wrap the Array's buffer in a DLPack-typed ndarray (zero-copy, kept alive by an
    // owner capsule holding a copy of the shared buffer) and hand it to warp.from_dlpack(), which
    // returns a warp.array aliasing the same memory.
    static handle from_cpp(const Array& value, rv_policy /*policy*/, cleanup_list* /*cleanup*/) noexcept
    {
        namespace ica = isaacsim::common::array;

        try
        {
            const DLTensor tensor = ica::toDLPack(value);
            std::vector<size_t> shape(static_cast<size_t>(tensor.ndim));
            for (size_t axis = 0; axis < shape.size(); ++axis)
            {
                shape[axis] = static_cast<size_t>(tensor.shape[axis]);
            }

            nanobind::dlpack::dtype dtype{ tensor.dtype.code, tensor.dtype.bits, tensor.dtype.lanes };

            // Keep the underlying buffer alive for as long as the ndarray (and any warp.array
            // derived from it) references the memory. The unique_ptr guards against a leak if the
            // capsule constructor throws; ownership transfers to the capsule's deleter on success.
            std::unique_ptr<std::shared_ptr<std::byte[]>> holder(new std::shared_ptr<std::byte[]>(value.buffer()));
            capsule owner(holder.get(),
                          [](void* pointer) noexcept { delete static_cast<std::shared_ptr<std::byte[]>*>(pointer); });
            holder.release();

            ndarray<> array(tensor.data, shape.size(), shape.data(), owner, tensor.strides, dtype,
                            static_cast<int>(tensor.device.device_type), tensor.device.device_id);

            object warp = module_::import_("warp");
            // warp.from_dlpack() requires the runtime to be initialized; init() is idempotent.
            warp.attr("init")();
            object result = warp.attr("from_dlpack")(cast(std::move(array)));
            return result.release();
        }
        catch (python_error& error)
        {
            error.restore();
            return handle();
        }
        catch (const std::exception& error)
        {
            PyErr_SetString(PyExc_RuntimeError, error.what());
            return handle();
        }
    }
};

} // namespace detail
} // namespace nanobind
