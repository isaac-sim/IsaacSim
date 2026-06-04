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

#include "WarpInterop.h"

#include <carb/logging/Log.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

template <typename T>
wp::array_t<T> warpArrayFromPython(py::object pyArray)
{
    py::gil_scoped_acquire gil;

    wp::array_t<T> result;

    try
    {
        py::object ptr_obj = pyArray.attr("ptr");
        py::object shape_obj = pyArray.attr("shape");
        py::object strides_obj = pyArray.attr("strides");

        // Extract pointer
        result.data = reinterpret_cast<T*>(ptr_obj.cast<uintptr_t>());

        // Extract shape
        py::tuple shape_tuple = shape_obj.cast<py::tuple>();
        result.ndim = shape_tuple.size();
        for (int i = 0; i < result.ndim && i < wp::ARRAY_MAX_DIMS; ++i)
        {
            result.shape[i] = shape_tuple[i].cast<int>();
        }

        // Extract strides
        py::tuple strides_tuple = strides_obj.cast<py::tuple>();
        for (int i = 0; i < result.ndim && i < wp::ARRAY_MAX_DIMS; ++i)
        {
            result.strides[i] = strides_tuple[i].cast<int>();
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to convert Warp array from Python: %s", e.what());
        result.data = nullptr;
    }

    return result;
}

int getWarpArrayDevice(py::object pyArray)
{
    py::gil_scoped_acquire gil;

    try
    {
        py::object device = pyArray.attr("device");
        std::string device_str = py::str(device);

        if (device_str.find("cpu") != std::string::npos)
        {
            return -1;
        }

        // Extract device ordinal for CUDA devices
        py::object ordinal = device.attr("ordinal");
        return ordinal.cast<int>();
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to get Warp array device: %s", e.what());
        return -1;
    }
}

std::vector<int64_t> getWarpArrayShape(py::object pyArray)
{
    py::gil_scoped_acquire gil;

    std::vector<int64_t> shape;
    try
    {
        py::tuple shape_tuple = pyArray.attr("shape").cast<py::tuple>();
        for (size_t i = 0; i < shape_tuple.size(); ++i)
        {
            shape.push_back(shape_tuple[i].cast<int64_t>());
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to get Warp array shape: %s", e.what());
    }

    return shape;
}

size_t getWarpArraySize(py::object pyArray)
{
    py::gil_scoped_acquire gil;

    try
    {
        py::object size_obj = pyArray.attr("size");
        return size_obj.cast<size_t>();
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to get Warp array size: %s", e.what());
        return 0;
    }
}

// Explicit template instantiations
template wp::array_t<float> warpArrayFromPython<float>(py::object);
template wp::array_t<int> warpArrayFromPython<int>(py::object);
template wp::array_t<wp::vec3> warpArrayFromPython<wp::vec3>(py::object);
template wp::array_t<wp::transform> warpArrayFromPython<wp::transform>(py::object);
template wp::array_t<wp::spatial_vector> warpArrayFromPython<wp::spatial_vector>(py::object);
template wp::array_t<wp::mat33> warpArrayFromPython<wp::mat33>(py::object);

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
