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

#include "isaacsim/common/array/Device.hpp"

#include <nanobind/nanobind.h>

#include <cstdint>
#include <limits>
#include <string>

// A nanobind type caster that exchanges isaacsim::common::array::Device with Python as a device
// string (`"cpu"`, `"cuda"`, `"cuda:0"`). Include this header in any binding translation unit that
// returns or accepts a Device. It is the device counterpart of ArrayCaster.hpp and DtypeCaster.hpp.
//
// A string is used rather than a Warp device object because Warp accepts these spellings directly
// (`wp.get_device("cuda:0")`, `wp.zeros(..., device="cuda:0")`), so no Warp import is needed on
// either side of the conversion and the caster stays free of Warp's initialization order.
//
// Python -> C++ also accepts a plain integer ordinal (-1 for the CPU, >= 0 for a CUDA device),
// matching Device's own SupportedDeviceSpecification contract. C++ -> Python always produces the
// string form, so a round trip normalizes an ordinal to its canonical spelling.
//
// Wrap the type in std::optional (with <nanobind/stl/optional.h>) to additionally accept and
// produce None; this caster is never asked to represent an absent device itself.

namespace nanobind
{
namespace detail
{

template <>
struct type_caster<isaacsim::common::array::Device>
{
    using Device = isaacsim::common::array::Device;

    NB_TYPE_CASTER(Device, const_name("str | int"))

    // Device has no default constructor, so the NB_TYPE_CASTER-declared `value` member needs one
    // supplied explicitly. The initial device is never observed: from_python overwrites it before
    // any conversion succeeds.
    type_caster() : value(Device::Cpu())
    {
    }

    // Python -> C++. A string is parsed by Device::fromString, which rejects anything that is not
    // "cpu", "cuda", or "cuda:<N>". An integer is taken as an ordinal.
    bool from_python(handle src, uint8_t /*flags*/, cleanup_list* /*cleanup*/) noexcept
    {
        if (!src.is_valid())
        {
            return false;
        }

        try
        {
            if (PyUnicode_Check(src.ptr()))
            {
                Py_ssize_t length = 0;
                const char* text = PyUnicode_AsUTF8AndSize(src.ptr(), &length);
                if (!text)
                {
                    PyErr_Clear(); // Don't leak a set error out of this noexcept converter.
                    return false;
                }
                value = Device::fromString(std::string(text, static_cast<size_t>(length)));
                return true;
            }

            // bool is a subclass of int in Python, so PyLong_Check accepts True/False. Reject them
            // rather than silently reading True as ordinal 1, which would select CUDA device 1.
            if (PyLong_Check(src.ptr()) && !PyBool_Check(src.ptr()))
            {
                const long long ordinal = PyLong_AsLongLong(src.ptr());
                if (ordinal == -1 && PyErr_Occurred())
                {
                    PyErr_Clear(); // Overflowed long long.
                    return false;
                }
                // Device's integer constructor casts without validating, so screen the range here:
                // anything below -1 is not a device, and a value beyond int32_t would wrap.
                if (ordinal < -1 || ordinal > std::numeric_limits<int32_t>::max())
                {
                    return false;
                }
                value = Device(static_cast<int32_t>(ordinal));
                return true;
            }
        }
        catch (...)
        {
            // Device::fromString throws std::invalid_argument for an unrecognized spelling.
            return false;
        }
        return false;
    }

    // C++ -> Python. Hands back the canonical string form, which Warp accepts directly.
    static handle from_cpp(const Device& value, rv_policy /*policy*/, cleanup_list* /*cleanup*/) noexcept
    {
        try
        {
            return str(value.toString().c_str()).release();
        }
        catch (python_error& error)
        {
            // Returning a null handle without a set error makes CPython raise SystemError and lose
            // the real cause, so restore the pending exception (or synthesize one) before failing.
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
