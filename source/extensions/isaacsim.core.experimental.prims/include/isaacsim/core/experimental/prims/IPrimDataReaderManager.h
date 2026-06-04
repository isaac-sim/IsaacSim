// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Interface.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

/**
 * @brief Shared lifecycle manager for IPrimDataReader.
 *
 * All sensor plugins should acquire this interface and call ensureInitialized()
 * instead of directly calling IPrimDataReader::initialize(). This centralizes
 * stage/timeline lifecycle behavior and prevents cross-plugin initialize churn.
 */
struct IPrimDataReaderManager
{
    CARB_PLUGIN_INTERFACE("isaacsim::core::experimental::prims::IPrimDataReaderManager", 1, 0);

    /**
     * @brief Ensure the shared reader is initialized for the given stage/device.
     * @details This interface owns all calls to IPrimDataReader::initialize().
     * Sensor plugins and nodes should never call initialize() directly.
     * @return true if reader is available and initialized, false otherwise.
     */
    virtual bool ensureInitialized(long stageId, int deviceOrdinal) = 0;

    /**
     * @brief Access the shared reader instance managed by this interface.
     * @return shared IPrimDataReader pointer, or nullptr if unavailable.
     */
    virtual IPrimDataReader* getReader() = 0;

    /**
     * @brief Current reader generation.
     */
    virtual uint64_t getGeneration() const = 0;
};

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim
