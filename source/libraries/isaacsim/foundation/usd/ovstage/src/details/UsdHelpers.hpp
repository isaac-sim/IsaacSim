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
#include <ovstage/ovstage.h>

#include <cstdint>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace ovstage
{
namespace details
{

namespace array = isaacsim::common::array;
using AttributeValue = std::variant<std::string, std::vector<std::string>, array::Array>;

struct StageEntry
{
    ovstage_instance_t* instance = nullptr;
    ovstage_ordinal_t nextOrdinal = 1;
};

int64_t registerInstance(ovstage_instance_t* instance, ovstage_ordinal_t nextOrdinal = 1);
ovstage_instance_t* unregisterInstance(int64_t stageId);
ovstage_instance_t* getInstance(int64_t stageId, bool throwIfInvalid = false);

bool isValidPathString(const std::string& path);
bool validatePrimAtPath(int64_t stageId, const std::string& path, bool throwIfInvalid = false);
bool validatePrimAtPath(ovstage_instance_t* instance, const std::string& path, bool throwIfInvalid = false);

std::string primPathToString(path_dictionary_instance_t* dict, ovx_primpath_t primPath);

ovstage_ordinal_t consumeOrdinal(int64_t stageId);
void waitAndRelease(ovstage_instance_t* instance, ovstage_enqueue_result_t enqueueResult);

} // namespace details
} // namespace ovstage
} // namespace usd
} // namespace foundation
} // namespace isaacsim
