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

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace
{

void appendU16(std::vector<uint8_t>& bytes, uint16_t value)
{
    bytes.push_back(static_cast<uint8_t>(value));
    bytes.push_back(static_cast<uint8_t>(value >> 8));
}

void appendU32(std::vector<uint8_t>& bytes, uint32_t value)
{
    bytes.push_back(static_cast<uint8_t>(value));
    bytes.push_back(static_cast<uint8_t>(value >> 8));
    bytes.push_back(static_cast<uint8_t>(value >> 16));
    bytes.push_back(static_cast<uint8_t>(value >> 24));
}

void appendEntry(std::vector<uint8_t>& bytes,
                 const std::string& name,
                 const std::vector<uint8_t>& payload,
                 uint16_t method = 0,
                 uint32_t declared_uncompressed_size = 0,
                 uint16_t flags = 0,
                 uint32_t declared_compressed_size = 0)
{
    appendU32(bytes, 0x04034b50u);
    appendU16(bytes, 20); /* version needed */
    appendU16(bytes, flags);
    appendU16(bytes, method);
    appendU16(bytes, 0); /* time */
    appendU16(bytes, 0); /* date */
    appendU32(bytes, 0); /* CRC: not consumed by the local-header reader */
    appendU32(bytes, declared_compressed_size == 0 ? static_cast<uint32_t>(payload.size()) : declared_compressed_size);
    appendU32(
        bytes, declared_uncompressed_size == 0 ? static_cast<uint32_t>(payload.size()) : declared_uncompressed_size);
    appendU16(bytes, static_cast<uint16_t>(name.size()));
    appendU16(bytes, 0); /* extra length */
    bytes.insert(bytes.end(), name.begin(), name.end());
    bytes.insert(bytes.end(), payload.begin(), payload.end());
}

bool writeFile(const std::filesystem::path& path, const std::vector<uint8_t>& bytes)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return stream.good();
}

bool expect(bool condition, const char* message)
{
    if (!condition)
        std::cerr << "FAIL: " << message << '\n';
    return condition;
}

} // namespace

int main()
{
    const int unique_marker = 0;
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("ovgl-usdz-archive-cache-test-" + std::to_string(reinterpret_cast<std::uintptr_t>(&unique_marker)) + ".usdz");
    struct RemoveFile
    {
        std::filesystem::path path;
        ~RemoveFile()
        {
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }
    } cleanup{ path };

    std::vector<uint8_t> package;
    appendEntry(package, "0/red.bin", { 255, 0, 0, 255 });
    appendEntry(package, "0/green.bin", { 0, 255, 0, 255 });
    if (!expect(writeFile(path, package), "write valid package"))
        return 1;

    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache cache;
    std::vector<uint8_t> out;
    std::string error;
    bool ok = true;
    ok &= expect(cache.readEntry(path.string(), "0/red.bin", out, error), "read first entry");
    ok &= expect(out == std::vector<uint8_t>({ 255, 0, 0, 255 }), "first entry payload");
    ok &= expect(cache.getArchiveLoadCount() == 1, "first entry loads archive once");
    ok &= expect(cache.readEntry(path.string(), "0/green.bin", out, error), "read second entry");
    ok &= expect(out == std::vector<uint8_t>({ 0, 255, 0, 255 }), "second entry payload");
    ok &= expect(cache.getArchiveLoadCount() == 1, "second entry reuses the build-local archive snapshot");
    ok &= expect(!cache.readEntry(path.string(), "0/missing.bin", out, error), "missing entry is rejected");
    ok &= expect(error.find("not found") != std::string::npos, "missing entry reports the cause");
    ok &= expect(cache.getArchiveLoadCount() == 1, "missing entry does not reread the package");

    package.clear();
    appendEntry(package, "0/green.bin", { 0, 0, 255, 255 });
    if (!expect(writeFile(path, package), "rewrite package between builds"))
        return 1;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache next_build_cache;
    ok &= expect(next_build_cache.readEntry(path.string(), "0/green.bin", out, error),
                 "a later build reads its own archive snapshot");
    ok &= expect(out == std::vector<uint8_t>({ 0, 0, 255, 255 }), "a later build sees the replaced package");
    ok &= expect(
        cache.readEntry(path.string(), "0/green.bin", out, error), "the first build retains its immutable snapshot");
    ok &= expect(out == std::vector<uint8_t>({ 0, 255, 0, 255 }), "archive snapshots are isolated between builds");

    package.clear();
    appendEntry(package, "0/compressed.bin", { 1, 2, 3 }, 8);
    if (!expect(writeFile(path, package), "write compressed package"))
        return 1;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache compressed_cache;
    ok &= expect(
        !compressed_cache.readEntry(path.string(), "0/compressed.bin", out, error), "compressed entry is rejected");
    ok &= expect(error.find("compressed") != std::string::npos, "compressed entry reports the cause");

    package.clear();
    appendEntry(package, "0/oversized.bin", { 1 }, 0, 4);
    if (!expect(writeFile(path, package), "write malformed package"))
        return 1;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache malformed_cache;
    ok &= expect(
        !malformed_cache.readEntry(path.string(), "0/oversized.bin", out, error), "oversized stored entry is rejected");
    ok &= expect(error.find("invalid size") != std::string::npos, "oversized entry reports the cause");

    package.clear();
    appendEntry(package, "0/descriptor.bin", { 1, 2, 3 }, 0, 3, 0x0008);
    if (!expect(writeFile(path, package), "write data-descriptor package"))
        return 1;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache descriptor_cache;
    ok &= expect(!descriptor_cache.readEntry(path.string(), "0/descriptor.bin", out, error),
                 "data-descriptor entry is rejected");
    ok &= expect(error.find("unsupported flags") != std::string::npos, "data-descriptor entry reports unsupported flags");

    package.clear();
    appendEntry(package, "0/truncated.bin", { 1, 2, 3, 4 }, 0, 2, 0, 4);
    if (!expect(writeFile(path, package), "write unequal stored-size package"))
        return 1;
    isaacsim::ovgl_viewport::debug::details::ovgl::UsdzArchiveCache unequal_size_cache;
    ok &= expect(!unequal_size_cache.readEntry(path.string(), "0/truncated.bin", out, error),
                 "stored entry with unequal sizes is rejected");
    ok &= expect(error.find("invalid size") != std::string::npos, "unequal stored sizes report the cause");

    return ok ? 0 : 1;
}
