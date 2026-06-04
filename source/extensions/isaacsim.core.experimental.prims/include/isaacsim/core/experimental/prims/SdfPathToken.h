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

#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <cstring>

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

// Omniverse PhysX stores rigid body identifiers (ContactEventHeader::actor0/actor1)
// as the internal uint64_t representation of pxr::SdfPath. This works because SdfPath
// wraps a single TfToken whose pointer is permanently interned in a global path table.
//
// Guarantees this depends on:
//   - sizeof(SdfPath) == sizeof(uint64_t)  (statically asserted below)
//   - SdfPath(string).GetString() round-trips through the path table correctly
//   - Tokens are process-local: a token value is only valid within the same USD
//     path table instance (same process). For cross-process use (gRPC), convert
//     to/from string form.
//
// These helpers centralise the memcpy pattern so that any future USD layout change
// is caught in one place.

inline uint64_t sdfPathToToken(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "SdfPath size mismatch");
    uint64_t token;
    std::memcpy(&token, static_cast<const void*>(&path), sizeof(uint64_t));
    return token;
}

inline pxr::SdfPath tokenToSdfPath(uint64_t token)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "SdfPath size mismatch");
    pxr::SdfPath path;
    std::memcpy(static_cast<void*>(&path), &token, sizeof(uint64_t));
    return path;
}

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim
