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
//   - Python -> C++ (from_python): zero-copy alias of a strided array-like source on either a
//     CPU or a CUDA device (shared memory); a bare Python scalar built directly as a 0-dim Array; or
//     a Python list/tuple copied via numpy.ascontiguousarray(). Meaningful negative-stride, non-numeric, and
//     unsupported-dtype inputs are rejected (the conversion fails, surfacing as nanobind's standard
//     incompatible-arguments error). Because that error names only the expected signature, a
//     conversion that was attempted and failed also reports its specific cause through
//     sys.unraisablehook; see reportConversionFailure().
//
// This library has no Python bindings of its own, so the caster is covered from the nearest module
// that exposes it: isaacsim/physics/manager/tests/python/test_array_caster.py. Keep that file in
// mind when changing the conversion or reporting behavior here.

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

// Reports the in-flight exception as the cause of a conversion that was attempted and genuinely
// failed, as opposed to an argument that simply is not an Array. Call only from inside a catch
// block; rethrowing is how the concrete type is recovered without repeating the handlers at every
// call site, each of which must keep a catch-all to stay noexcept.
//
// A caster signals failure by returning false, which nanobind renders as its generic "incompatible
// function arguments" error -- the same text an unrelated argument type produces. The specific
// cause (an unsupported dtype, a negative stride, an int that overflows int64) is therefore
// invisible at exactly the moment it is most needed. Setting the Python error instead does not
// survive: nanobind's nb_func_error_overload() overwrites any pending error with its own TypeError,
// and while overload resolution is still in flight a stray error would be attributed to whichever
// candidate is tried next. The unraisable hook is the one channel that both reaches the user and
// leaves the error indicator clean for the resolution still in progress.
//
// The report is per conversion attempt, not per call, and nanobind may attempt a conversion more
// than once: it runs two resolution passes over an overloaded function (nb_func.cpp, `pass` loop),
// so a failing argument to one is reported twice, and a std::variant alternative that fails before
// a later one succeeds is reported even though the call goes on to succeed. Both are duplicate or
// premature diagnostics rather than wrong behavior, and are accepted as the price of not discarding
// the cause. Ordering keeps the second case out of reach in practice: Array is the last alternative
// of every variant this library exposes, so an Array that fails fails the whole variant.
inline void reportConversionFailure(nanobind::handle source) noexcept
{
    if (!std::current_exception())
    {
        return; // Defend the bare `throw` below: with nothing in flight it would terminate().
    }

    // Stages `cause` as a Python error and hands it straight to the unraisable hook, which fetches,
    // reports, and clears it. Callers hold the GIL (nanobind casters always do), so the C API is
    // safe to touch here.
    const auto report = [source](const char* cause) noexcept
    {
        PyErr_Clear(); // Drop whatever the failed conversion left behind before staging our own.
        PyErr_Format(PyExc_RuntimeError, "cannot view this object as isaacsim::common::array::Array: %s", cause);
        PyErr_WriteUnraisable(source.ptr());
    };

    try
    {
        throw;
    }
    catch (nanobind::python_error& error)
    {
        // The cause is a Python exception that nanobind stole from the interpreter. Hand it back so
        // the hook reports it with its own type and traceback, which is both richer and safer than
        // stringifying it: python_error::what() builds that string by running Python, and it is
        // itself noexcept, so a failure while formatting terminates the process inside what()
        // rather than surfacing here. Observed with numpy's ragged-input ValueError.
        error.discard_as_unraisable(source);
    }
    catch (const std::exception& error)
    {
        report(error.what());
    }
    catch (...)
    {
        report("unknown C++ exception");
    }
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
    // array) already exposes shape, dtype, strides, and a data pointer, so it is aliased zero-copy
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
                ica::details::reportConversionFailure(src);
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
                // fromNdarray() -> fromDLPackBorrowed() (which rejects e.g. float16), so this guard only
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
                // numpy raises for ragged/inhomogeneous input (e.g. [[1, [2]], [3, [4]]]). The
                // reporter consumes that Python error, leaving the interpreter error flag clear.
                ica::details::reportConversionFailure(src);
                return false;
            }
        }
        return false;
    }

    // Import an array-like source (warp.array or any DLPack/buffer-protocol array) and alias its
    // buffer zero-copy into an Array, on the device the source already lives on (CPU or CUDA).
    // Non-negative strides are preserved; irrelevant negative singleton/empty strides are canonicalized.
    // Meaningful negative strides fail conversion. The resulting Array shares memory with the Python
    // array (writes on either side are visible to the other, until copy() or clone() is called).
    bool fromNdarray(handle src, uint8_t flags, cleanup_list* cleanup) noexcept
    {
        namespace ica = isaacsim::common::array;

        // fromDLPackBorrowed() validates the imported metadata and throws on unsupported inputs, while
        // function is noexcept, so the whole conversion is wrapped. The import is inside the try for
        // scope symmetry only: nanobind's ndarray importer is itself noexcept and reports failure by
        // return value rather than by throwing.
        try
        {
            make_caster<ndarray<>> caster;
            if (!caster.from_python(src, flags, cleanup))
            {
                return false;
            }
            ndarray<>& array = caster.value;

            const nanobind::dlpack::dtype arrayDtype = array.dtype();
            DLTensor tensor{};
            tensor.data = array.data();
            tensor.device = { static_cast<DLDeviceType>(array.device_type()), array.device_id() };
            tensor.ndim = static_cast<int32_t>(array.ndim());
            tensor.dtype = { arrayDtype.code, arrayDtype.bits, arrayDtype.lanes };
            tensor.shape = const_cast<int64_t*>(array.shape_ptr());
            tensor.strides = const_cast<int64_t*>(array.stride_ptr());
            tensor.byte_offset = 0;

            bool hasNegativeStride = false;
            for (int32_t axis = 0; tensor.strides && axis < tensor.ndim; ++axis)
            {
                hasNegativeStride = hasNegativeStride || tensor.strides[axis] < 0;
            }
            const ica::Array borrowed = hasNegativeStride ? ica::fromDLPack(tensor) : ica::fromDLPackBorrowed(tensor);

            // Zero-copy alias of the source buffer (a device pointer for a CUDA source, which
            // fromBuffer() adopts as device storage). Keep both the original argument and nanobind's
            // imported ndarray alive for the lifetime of the Array (and any view that shares its
            // buffer): for a DLPack capsule the source alone is not enough, because importing
            // consumes the capsule and moves ownership of the managed tensor into the ndarray.
            //
            // The ndarray is retained by value rather than as a Python object. nanobind's ndarray is
            // internally reference-counted, so a copy is all that is needed -- and re-exporting one
            // to Python (cast(array)) cannot be used here. For an ndarray imported from a consumed
            // DLPack capsule, nanobind's ndarray_export() fails and raises on its error path; because
            // ndarray_export() is itself noexcept, that raise aborts the process at nanobind's own
            // boundary and never unwinds into the catch below. Verified: re-exporting a consumed
            // capsule terminates with std::bad_cast via __cxa_call_terminate.
            //
            // A raw PyObject* is captured for the source (rather than an nb::object) so the decref
            // happens exactly once, inside the deleter, under the GIL — capturing an nb::object
            // would make the lambda's own destructor decref without the GIL held. The deleter may
            // run on any thread, at any time, so it re-acquires the GIL; the Py_IsInitialized()
            // guard covers the case where an Array outlives interpreter finalization.
            PyObject* keepSource = src.ptr();
            Py_INCREF(keepSource);
            auto* keepImported = new ndarray<>(array);
            std::shared_ptr<std::byte[]> aliased(static_cast<std::byte*>(array.data()),
                                                 [keepSource, keepImported](std::byte*) noexcept
                                                 {
                                                     if (!Py_IsInitialized())
                                                     {
                                                         return;
                                                     }
                                                     gil_scoped_acquire gil;
                                                     delete keepImported;
                                                     Py_DECREF(keepSource);
                                                 });

            value = ica::Array::fromBuffer(std::move(aliased), borrowed.capacity(), borrowed.shape(), borrowed.dtype(),
                                           borrowed.device(), 0, borrowed.strides());
            return true;
        }
        catch (...)
        {
            // Only a source that nanobind already imported as an ndarray reaches here; an argument
            // that is not array-like leaves via the quiet `return false` above. Every arrival is
            // therefore a real import failure and worth reporting.
            ica::details::reportConversionFailure(src);
            return false;
        }
    }

    // Resolve warp.from_dlpack once rather than on every conversion: a physics step converts one array per
    // tensor read, so importing the module and walking two attributes each time lands squarely on a hot
    // path. warp.init() is idempotent, so it only needs to run alongside that one-time resolution.
    //
    // warp.init() is safe here only because this function is reached during an actual conversion, never
    // while a module is being imported. Do NOT give any binding a default argument of a type this caster
    // converts (`nb::arg("x") = Array{...}`): nanobind converts default arguments eagerly, inside the
    // module init function, which would run warp.init() re-entrantly during import. That crashed the
    // nanobind stub generator on Windows CI when a Dtype default argument did it; see the matching note
    // in DtypeCaster.hpp, which for that reason resolves Warp without initializing it.
    //
    // The callable is heap-allocated and never freed on purpose. A static nb::object would be destroyed
    // during static destruction, which can run after the interpreter has been finalized and would then
    // decref a dangling PyObject. The cache correspondingly assumes the embedded interpreter lives for
    // the whole process; it is not invalidated by a finalize, so a finalize-then-reinitialize cycle
    // would leave it pointing at a dead object.
    //
    // Callers hold the GIL (nanobind casters always do), but import_() and init() run arbitrary Python
    // and may release it, so two threads can race here. That is left unsynchronized deliberately: both
    // would resolve the same callable and warp.init() is idempotent, so the only consequence is that
    // the losing thread's `object` leaks once.
    static const object& getWarpFromDLPack()
    {
        static object* cached = nullptr;
        if (!cached)
        {
            object warp = module_::import_("warp");
            warp.attr("init")();
            cached = new object(warp.attr("from_dlpack"));
        }
        return *cached;
    }

    // Retain ownership in Python; the temporary read scope orders CUDA readiness for the export.
    static handle from_cpp(const Array& value, rv_policy /*policy*/, cleanup_list* /*cleanup*/) noexcept
    {
        namespace ica = isaacsim::common::array;

        try
        {
            auto access = ica::acquireRead(value);
            // `tensor.data` already addresses element 0: every export in this library resolves the
            // Array's view offset rather than publishing a base pointer plus byte_offset, which
            // nanobind's ndarray has no field for and warp would ignore.
            const DLTensor tensor = ica::toDLPackBorrowed(value);
            std::vector<size_t> shape(static_cast<size_t>(tensor.ndim));
            for (size_t axis = 0; axis < shape.size(); ++axis)
            {
                shape[axis] = static_cast<size_t>(tensor.shape[axis]);
            }

            nanobind::dlpack::dtype dtype{ tensor.dtype.code, tensor.dtype.bits, tensor.dtype.lanes };

            auto retained = std::make_unique<Array>(value);
            capsule owner(retained.get(), [](void* pointer) noexcept { delete static_cast<Array*>(pointer); });
            retained.release();

            ndarray<> array(tensor.data, shape.size(), shape.data(), owner, tensor.strides, dtype,
                            static_cast<int>(tensor.device.device_type), tensor.device.device_id);

            object result = getWarpFromDLPack()(cast(std::move(array)));
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
