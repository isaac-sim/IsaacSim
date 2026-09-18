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

#include "Renderer.hpp"

#include "details/OvglBackend.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace details
{
namespace
{

[[noreturn]] void throwBackendError(const std::string& context, const char* detail = nullptr)
{
    const char* message = detail && detail[0] != '\0' ? detail : getOvglBackendError();
    throw std::runtime_error(message && message[0] != '\0' ? context + ": " + message : context);
}

} // namespace

class Renderer::Implementation
{
public:
    explicit Implementation(ovstage_instance_t* stage)
    {
        if (!stage)
        {
            throw std::invalid_argument("OVStage instance must not be null");
        }
        m_backend = createOvglBackend("isaacsim.ovgl_viewport.debug");
        if (!m_backend)
        {
            throwBackendError("Unable to create OVGL renderer");
        }

        char error[1024]{};
        if (attachStage(m_backend, stage, error, sizeof(error)) != 0)
        {
            (void)destroyOvglBackend(m_backend);
            m_backend = nullptr;
            throwBackendError("Unable to attach OVStage to OVGL", error);
        }
    }

    ~Implementation()
    {
        if (m_backend)
        {
            (void)detachStage(m_backend);
            (void)destroyOvglBackend(m_backend);
        }
    }

    void render(const std::string& renderProductPath, Frame& frame)
    {
        ovstage_ordinal_t ordinal = 0;
        if (getStageWriteFloor(m_backend, &ordinal) != 0 || ordinal == 0)
        {
            throwBackendError("OVStage has no non-zero sealed write floor to render");
        }

        const uint8_t* pixels = nullptr;
        size_t byteCount = 0;
        int width = 0;
        int height = 0;
        char error[1024]{};
        if (renderStage(m_backend, ordinal, renderProductPath.c_str(), &pixels, &byteCount, &width, &height, error,
                        sizeof(error)) != 0)
        {
            throwBackendError("Unable to render OVGL frame", error);
        }
        if (!pixels || width <= 0 || height <= 0 || static_cast<uint64_t>(width) > std::numeric_limits<uint32_t>::max() ||
            static_cast<uint64_t>(height) > std::numeric_limits<uint32_t>::max() ||
            static_cast<size_t>(width) > std::numeric_limits<size_t>::max() / 4 ||
            static_cast<size_t>(height) > std::numeric_limits<size_t>::max() / (static_cast<size_t>(width) * 4) ||
            byteCount != static_cast<size_t>(width) * static_cast<size_t>(height) * 4)
        {
            throw std::runtime_error("OVGL returned an invalid LdrColor byte extent");
        }

        frame = {};
        frame.frameNumber = ++m_frameNumber;
        frame.stageOrdinal = ordinal;
        frame.width = static_cast<uint32_t>(width);
        frame.height = static_cast<uint32_t>(height);
        frame.rgba.resize(byteCount);
        std::memcpy(frame.rgba.data(), pixels, byteCount);
    }

    void present(const std::string& renderProductPath,
                 uint32_t destinationWidth,
                 uint32_t destinationHeight,
                 const char* overlayText,
                 Frame& frame)
    {
        if (destinationWidth == 0 || destinationHeight == 0 ||
            destinationWidth > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            destinationHeight > static_cast<uint32_t>(std::numeric_limits<int>::max()))
        {
            throw std::invalid_argument("Presentation dimensions must be positive OpenGL-compatible pixel extents");
        }
        ovstage_ordinal_t ordinal = 0;
        if (getStageWriteFloor(m_backend, &ordinal) != 0 || ordinal == 0)
        {
            throwBackendError("OVStage has no non-zero sealed write floor to render");
        }

        int width = 0;
        int height = 0;
        char error[1024]{};
        if (presentStage(m_backend, ordinal, renderProductPath.c_str(), 0, static_cast<int>(destinationWidth),
                         static_cast<int>(destinationHeight), overlayText, &width, &height, error, sizeof(error)) != 0)
        {
            throwBackendError("Unable to present OVGL frame", error);
        }
        if (width <= 0 || height <= 0 || static_cast<uint64_t>(width) > std::numeric_limits<uint32_t>::max() ||
            static_cast<uint64_t>(height) > std::numeric_limits<uint32_t>::max())
        {
            throw std::runtime_error("OVGL returned invalid presented-frame dimensions");
        }

        frame = {};
        frame.frameNumber = ++m_frameNumber;
        frame.stageOrdinal = ordinal;
        frame.width = static_cast<uint32_t>(width);
        frame.height = static_cast<uint32_t>(height);
    }

private:
    OvglBackend* m_backend{ nullptr };
    uint64_t m_frameNumber{ 0 };
};

Renderer::Renderer(ovstage_instance_t* stage) : m_implementation(std::make_unique<Implementation>(stage))
{
}

Renderer::~Renderer() = default;
Renderer::Renderer(Renderer&&) noexcept = default;
Renderer& Renderer::operator=(Renderer&&) noexcept = default;

void Renderer::render(const std::string& renderProductPath, Frame& frame)
{
    m_implementation->render(renderProductPath, frame);
}

void Renderer::present(const std::string& renderProductPath,
                       uint32_t destinationWidth,
                       uint32_t destinationHeight,
                       const char* overlayText,
                       Frame& frame)
{
    m_implementation->present(renderProductPath, destinationWidth, destinationHeight, overlayText, frame);
}

} // namespace details
} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
