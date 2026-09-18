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

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>

#include <cstddef>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace entities
{
namespace details
{

namespace array = isaacsim::common::array;
namespace tensors = isaacsim::physics::tensors;

array::Array convertTensorDescriptionToArray(const tensors::TensorDescription& description);
tensors::TensorDescription convertArrayToTensorDescription(const array::Array& array);

std::vector<array::Array> splitArray(const array::Array& array, size_t index);
array::Array joinArrays(const std::vector<array::Array>& arrays);

} // namespace details
} // namespace entities
} // namespace physics
} // namespace isaacsim
