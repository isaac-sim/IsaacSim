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

// Pybind11 utilities for reading Warp array metadata from Python.
// Extracts raw data pointers, device ordinals, shapes, and sizes from wp.array objects
// without importing the full Warp C++ headers.

#include "WarpCompat.h"

#include <pybind11/pybind11.h>

#include <cstdint>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

namespace py = pybind11;

/// Extracts a wp::array_t<T> descriptor from a Python wp.array, reading ptr, shape, strides.
template <typename T>
wp::array_t<T> warpArrayFromPython(py::object pyArray);

/// Returns the CUDA device ordinal of a wp.array (-1 for CPU, >= 0 for GPU).
int getWarpArrayDevice(py::object pyArray);

/// Returns the shape of a wp.array as a vector of dimension sizes.
std::vector<int64_t> getWarpArrayShape(py::object pyArray);

/// Returns the total number of elements in a wp.array (product of all dimensions).
size_t getWarpArraySize(py::object pyArray);

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
