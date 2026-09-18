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

#include "isaacsim/common/array/Array.hpp"
#include "isaacsim/common/array/Export.h"

#include <dlpack/dlpack.h>

namespace isaacsim
{
namespace common
{
namespace array
{

/**
 * @brief Creates an Array view of a DLPack tensor without copying its data.
 * @details
 * The tensor must use a supported scalar dtype, reside on a CPU or CUDA device, and be C-contiguous. A null strides
 * pointer is accepted as the conventional representation of a contiguous tensor.
 *
 * The returned Array borrows the tensor's data allocation. The owner of @p tensor must keep the allocation alive for
 * the lifetime of the returned Array and every view derived from it. Shape and dtype metadata are copied.
 *
 * This function does not perform stream synchronization.
 *
 * @param[in] tensor DLPack tensor to view.
 * @return An Array that aliases the tensor's data.
 * @throws std::invalid_argument if the tensor metadata is invalid or unsupported.
 * @throws std::overflow_error if the tensor rank, element count, or byte offset cannot be represented.
 */
ISAACSIM_COMMON_ARRAY_API Array fromDLPack(const DLTensor& tensor);

/**
 * @brief Creates a borrowed DLPack tensor view of an Array without copying its data.
 * @details
 * The returned tensor aliases @p array and points into its shape metadata. It remains valid only while @p array is
 * alive and is neither moved from nor assigned. The strides pointer is null to denote the Array's C-contiguous layout.
 *
 * This function does not perform stream synchronization.
 *
 * @param[in] array Array to view.
 * @return A DLPack tensor that aliases the Array's data and metadata.
 * @throws std::invalid_argument if the Array has invalid shape metadata.
 * @throws std::overflow_error if the Array rank or element count cannot be represented by DLPack.
 */
ISAACSIM_COMMON_ARRAY_API DLTensor toDLPack(const Array& array);

} // namespace array
} // namespace common
} // namespace isaacsim
