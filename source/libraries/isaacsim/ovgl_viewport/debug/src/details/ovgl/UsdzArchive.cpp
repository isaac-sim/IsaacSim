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

#include "UsdzArchive.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <memory>

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

namespace
{
struct FileCloser
{
    void operator()(std::FILE* file) const
    {
        std::fclose(file);
    }
};
} // namespace

bool UsdzArchiveCache::_loadArchive(const std::string& archive, const std::vector<uint8_t>*& bytes, std::string& error)
{
    const auto cached = m_archives.find(archive);
    if (cached != m_archives.end())
    {
        bytes = &cached->second;
        return true;
    }

    using File = std::unique_ptr<std::FILE, FileCloser>;
    File file(std::fopen(archive.c_str(), "rb"));
    if (!file)
    {
        error = "usdz open: " + archive;
        return false;
    }
    if (std::fseek(file.get(), 0, SEEK_END) != 0)
    {
        error = "usdz seek: " + archive;
        return false;
    }
    const long end = std::ftell(file.get());
    if (end < 0 ||
        static_cast<unsigned long long>(end) > static_cast<unsigned long long>(std::numeric_limits<size_t>::max()))
    {
        error = "usdz size: " + archive;
        return false;
    }
    if (std::fseek(file.get(), 0, SEEK_SET) != 0)
    {
        error = "usdz seek: " + archive;
        return false;
    }

    std::vector<uint8_t> loaded(static_cast<size_t>(end));
    const size_t got = loaded.empty() ? 0 : std::fread(loaded.data(), 1, loaded.size(), file.get());
    if (got != loaded.size())
    {
        error = "usdz read: " + archive;
        return false;
    }

    auto inserted = m_archives.emplace(archive, std::move(loaded));
    bytes = &inserted.first->second;
    ++m_archiveLoadCount;
    return true;
}

bool UsdzArchiveCache::readEntry(const std::string& archive,
                                 const std::string& inner,
                                 std::vector<uint8_t>& out,
                                 std::string& error)
{
    out.clear();
    error.clear();
    const std::vector<uint8_t>* archive_bytes = nullptr;
    if (!_loadArchive(archive, archive_bytes, error))
        return false;
    const std::vector<uint8_t>& zip = *archive_bytes;

    const auto contains = [&](size_t offset, size_t count)
    { return offset <= zip.size() && count <= zip.size() - offset; };
    const auto u16 = [&](size_t offset) -> uint32_t
    { return static_cast<uint32_t>(zip[offset]) | (static_cast<uint32_t>(zip[offset + 1]) << 8); };
    const auto u32 = [&](size_t offset) -> uint32_t
    {
        return static_cast<uint32_t>(zip[offset]) | (static_cast<uint32_t>(zip[offset + 1]) << 8) |
               (static_cast<uint32_t>(zip[offset + 2]) << 16) | (static_cast<uint32_t>(zip[offset + 3]) << 24);
    };

    size_t header = 0;
    while (contains(header, 30))
    {
        if (u32(header) != 0x04034b50u)
            break; /* local file header */
        const uint32_t flags = u16(header + 6);
        const uint32_t method = u16(header + 8);
        const uint32_t compressed_size = u32(header + 18);
        const uint32_t uncompressed_size = u32(header + 22);
        const uint32_t name_length = u16(header + 26);
        const uint32_t extra_length = u16(header + 28);
        const size_t name_offset = header + 30;
        if (!contains(name_offset, name_length))
            break;
        const std::string name(reinterpret_cast<const char*>(zip.data() + name_offset), name_length);
        const size_t after_name = name_offset + name_length;
        if (!contains(after_name, extra_length))
            break;
        const size_t data_offset = after_name + extra_length;
        if (!contains(data_offset, compressed_size))
            break;

        if (name == inner)
        {
            /* A stored USDZ member must carry its sizes in this local header.
             * Bit 3 moves them to a trailing data descriptor, while encryption
             * and patch/strong-encryption flags change the payload contract.
             * UTF-8 names (bit 11) are the only general-purpose flag needed by
             * this local-header reader. */
            if ((flags & ~0x0800u) != 0)
            {
                error = "usdz entry has unsupported flags (" + std::to_string(flags) + "): " + inner;
                return false;
            }
            if (method != 0)
            {
                /* USDZ forbids compression. Refuse instead of handing compressed
                 * bytes to an image decoder as if they were the texture. */
                error = "usdz entry is compressed (method " + std::to_string(method) + "): " + inner;
                return false;
            }
            if (uncompressed_size != compressed_size || !contains(data_offset, uncompressed_size))
            {
                error = "usdz entry has invalid size: " + inner;
                return false;
            }
            out.assign(zip.data() + data_offset, zip.data() + data_offset + uncompressed_size);
            return true;
        }
        header = data_offset + compressed_size;
    }

    error = "usdz entry not found: " + inner + " in " + archive;
    return false;
}

} // namespace ovgl
} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
