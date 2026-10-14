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

#include "isaacsim/common/array/Dtype.hpp"

#include <nanobind/nanobind.h>

#include <string>

// A nanobind type caster that exchanges isaacsim::common::array::Dtype with Python as a Warp scalar
// data type (`wp.float32`, `wp.int32`, ...). Include this header in any binding translation unit that
// returns or accepts a Dtype. It is the dtype counterpart of ArrayCaster.hpp: together they let a
// binding speak Array and Dtype while Python only ever sees `wp.array` and Warp data types, with no
// intermediate element-type vocabulary of our own to keep in sync.
//
// Warp's scalar types cover every Dtype::Kind, so C++ -> Python is total. The reverse is partial:
// Warp also has `float16`, `bfloat16`, and vector/matrix types that no Dtype represents, and those
// are rejected rather than silently approximated. In particular a vector type is NOT flattened to its
// component type here -- treating a vector width as a trailing shape dimension is a convention of the
// caller that builds the shape, not a property of the element type, and this library has no business
// encoding it.

namespace nanobind
{
namespace detail
{

template <>
struct type_caster<isaacsim::common::array::Dtype>
{
    using Dtype = isaacsim::common::array::Dtype;

    NB_TYPE_CASTER(Dtype, const_name("warp.dtype"))

    // Dtype has no default constructor, so the NB_TYPE_CASTER-declared `value` member needs one
    // supplied explicitly. The initial type is never observed: from_python overwrites it before any
    // conversion succeeds.
    type_caster() : value(Dtype::Kind::eFloat32)
    {
    }

    // Resolve the `warp` module once rather than on every conversion: a spec query converts a dtype
    // per registered operation, so importing the module each time lands on a hot path.
    //
    // Deliberately does NOT call warp.init(): Warp's scalar types are plain classes that `import warp`
    // already defines, so this caster never needs Warp's native runtime.
    //
    // Calling it here is not merely unnecessary but harmful, because this function can run during
    // module import. nanobind converts a default argument eagerly -- `nb::arg("x") = value` calls
    // cast() at that point, inside the module init function -- so a Dtype used as a default argument
    // (see TensorSpec in the physics registration bindings) reaches this code while the binding module
    // is still being imported. warp.init() loads Warp's native runtime via ctypes, and doing that
    // re-entrantly from another module's import crashed the nanobind stub generator outright on
    // Windows CI (the process died with STATUS_STACK_BUFFER_OVERRUN, 0xc0000409, and no traceback).
    //
    // ArrayCaster does call init(), which is safe only because it is reached during an actual
    // conversion and never at import time. Keep it that way: do not give any binding a default
    // argument whose type is cast by ArrayCaster.
    //
    // The module object is heap-allocated and never freed on purpose, matching ArrayCaster: a static
    // nb::object would be destroyed during static destruction, which can run after the interpreter has
    // been finalized and would then decref a dangling PyObject. The cache correspondingly assumes the
    // embedded interpreter lives for the whole process; it is not invalidated by a finalize, so a
    // finalize-then-reinitialize cycle would leave it pointing at a dead object.
    //
    // Callers hold the GIL (nanobind casters always do), but import_() runs arbitrary Python and may
    // release it, so two threads can race here. That is left unsynchronized deliberately: both would
    // import the same module object, so the only consequence is that the losing thread's `object`
    // leaks once. Note this must NOT be simplified to a function-local static initialized in place: a
    // thread blocking on the magic-static guard keeps the GIL, which would deadlock against the
    // initializing thread waiting to reacquire it inside import_().
    static const object& getWarpModule()
    {
        static object* cached = nullptr;
        if (!cached)
        {
            cached = new object(module_::import_("warp"));
        }
        return *cached;
    }

    // The attribute name Warp uses for each Dtype kind, or nullptr for a kind outside the enumeration.
    // Warp spells its scalar types exactly as Dtype::toString() does, but the mapping is written out
    // rather than derived from the string so that adding a kind to Dtype fails to compile here
    // (-Wswitch is an error) instead of failing at runtime on a missing Warp attribute.
    static const char* getWarpAttributeName(Dtype dtype)
    {
        switch (dtype.kind())
        {
        case Dtype::Kind::eBool:
            return "bool";
        case Dtype::Kind::eInt8:
            return "int8";
        case Dtype::Kind::eInt16:
            return "int16";
        case Dtype::Kind::eInt32:
            return "int32";
        case Dtype::Kind::eInt64:
            return "int64";
        case Dtype::Kind::eUInt8:
            return "uint8";
        case Dtype::Kind::eUInt16:
            return "uint16";
        case Dtype::Kind::eUInt32:
            return "uint32";
        case Dtype::Kind::eUInt64:
            return "uint64";
        case Dtype::Kind::eFloat32:
            return "float32";
        case Dtype::Kind::eFloat64:
            return "float64";
        }
        return nullptr;
    }

    // Python -> C++. Warp spells each scalar type as a class whose `__name__` matches Dtype's own
    // spelling ("float32", "int32", ...), so the name resolves the candidate in one step and no table
    // of Warp type objects has to be kept in sync with Dtype::Kind. Dtype's string constructor rejects
    // every name Dtype cannot represent, which is what turns away Warp's `float16`/`bfloat16` and its
    // vector and matrix types. Anything else -- a numpy dtype, a plain string -- fails the conversion,
    // which nanobind surfaces as its standard incompatible-arguments error.
    bool from_python(handle src, uint8_t /*flags*/, cleanup_list* /*cleanup*/) noexcept
    {
        if (!src.is_valid())
        {
            return false;
        }

        try
        {
            object nameObject = steal(PyObject_GetAttrString(src.ptr(), "__name__"));
            if (!nameObject.is_valid())
            {
                PyErr_Clear(); // Don't leak a set error out of this noexcept converter.
                return false;
            }

            Py_ssize_t length = 0;
            const char* text = PyUnicode_AsUTF8AndSize(nameObject.ptr(), &length);
            if (!text)
            {
                PyErr_Clear();
                return false;
            }

            const Dtype candidate{ std::string(text, static_cast<size_t>(length)) };

            // Require the object to be Warp's own type, so an unrelated class that merely happens to
            // be named "float32" is not silently accepted.
            const char* attribute = getWarpAttributeName(candidate);
            if (!attribute || !src.is(getWarpModule().attr(attribute)))
            {
                return false;
            }

            value = candidate;
            return true;
        }
        catch (...)
        {
            // Dtype's string constructor throws std::invalid_argument for a name it cannot represent.
            return false;
        }
    }

    // C++ -> Python. Hands back the Warp scalar type itself, so the result compares equal (and
    // identical) to `wp.float32` and can be passed straight to `wp.zeros(...)`.
    static handle from_cpp(Dtype value, rv_policy /*policy*/, cleanup_list* /*cleanup*/) noexcept
    {
        try
        {
            // Defensive: Dtype's Kind constructor is public and unvalidated, so a Dtype holding a
            // value outside the enumeration can reach here. Report it rather than passing nullptr to
            // attr(), which would dereference it.
            const char* attribute = getWarpAttributeName(value);
            if (!attribute)
            {
                PyErr_SetString(PyExc_ValueError, "Dtype holds a kind outside the supported enumeration");
                return handle();
            }
            object warpType = getWarpModule().attr(attribute);
            return warpType.release();
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
