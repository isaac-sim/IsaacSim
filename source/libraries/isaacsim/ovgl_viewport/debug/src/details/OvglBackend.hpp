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

#include <ovstage/ovstage.h>

#include <cstddef>
#include <cstdint>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{

struct OvglBackend;

OvglBackend* createOvglBackend(const char* name);
int destroyOvglBackend(OvglBackend* backend);
int attachStage(OvglBackend* backend, ovstage_instance_t* stage, char* error, std::size_t errorLength);
int detachStage(OvglBackend* backend);
int getStageWriteFloor(OvglBackend* backend, ovstage_ordinal_t* ordinal);
int renderStage(OvglBackend* backend,
                ovstage_ordinal_t ordinal,
                const char* renderProductPath,
                const std::uint8_t** data,
                std::size_t* byteCount,
                int* width,
                int* height,
                char* error,
                std::size_t errorLength);
int presentStage(OvglBackend* backend,
                 ovstage_ordinal_t ordinal,
                 const char* renderProductPath,
                 unsigned framebuffer,
                 int destinationWidth,
                 int destinationHeight,
                 const char* overlayText,
                 int* width,
                 int* height,
                 char* error,
                 std::size_t errorLength);
const char* getOvglBackendError(void);

} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
