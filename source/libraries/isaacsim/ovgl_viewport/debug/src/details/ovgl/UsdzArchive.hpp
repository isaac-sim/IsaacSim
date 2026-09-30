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

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{
namespace ovgl
{

/* Build-local USDZ reader.  A package is snapshotted at most once, then each
 * texture entry is copied out of that immutable byte buffer for stb_image.
 * Instances are intentionally local to one scene build: archive bytes never
 * become renderer state and cannot leak across renderers or later rebuilds. */
class UsdzArchiveCache
{
public:
    bool readEntry(const std::string& archive, const std::string& inner, std::vector<uint8_t>& out, std::string& error);

    size_t getArchiveLoadCount() const
    {
        return m_archiveLoadCount;
    }

private:
    bool _loadArchive(const std::string& archive, const std::vector<uint8_t>*& bytes, std::string& error);

    std::unordered_map<std::string, std::vector<uint8_t>> m_archives;
    size_t m_archiveLoadCount = 0;
};

} // namespace ovgl
} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
