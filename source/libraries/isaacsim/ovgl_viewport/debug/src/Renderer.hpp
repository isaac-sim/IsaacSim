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

#include "isaacsim/ovgl_viewport/debug/Viewport.hpp"

#include <memory>
#include <string>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{

class Renderer final
{
public:
    explicit Renderer(ovstage_instance_t* stage);
    ~Renderer();

    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) noexcept;
    Renderer& operator=(Renderer&&) noexcept;

    void render(const std::string& renderProductPath, Frame& frame);
    void present(const std::string& renderProductPath,
                 uint32_t destinationWidth,
                 uint32_t destinationHeight,
                 const char* overlayText,
                 Frame& frame);

private:
    class Implementation;
    std::unique_ptr<Implementation> m_implementation;
};

} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
