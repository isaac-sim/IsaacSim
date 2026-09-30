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

#include "isaacsim/ovgl_viewport/debug/Export.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

struct ovstage_instance_t;

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{

/** @brief Initial first-person camera state for debug navigation. */
struct Camera
{
    /** @brief Camera look-at target in stage coordinates. */
    std::array<double, 3> target{ 0.0, 0.0, 0.0 };
    /** @brief Horizontal view angle in radians. */
    double yawRadians{ -0.65 };
    /** @brief Vertical view angle in radians. */
    double pitchRadians{ 0.38 };
    /** @brief Distance from the eye to the target in stage units. */
    double distance{ 6.0 };
};

/** @brief World pose produced by debug camera navigation. */
struct CameraPose
{
    /** @brief Camera position in stage coordinates. */
    std::array<double, 3> position{ 0.0, 0.0, 0.0 };
    /** @brief Camera orientation as an xyzw-ordered quaternion. */
    std::array<double, 4> orientation{ 0.0, 0.0, 0.0, 1.0 };
};

/** @brief Application callback that authors a debug camera pose on its stage. */
using CameraPoseWriter = std::function<void(const CameraPose&)>;

/** @brief Configuration for one concrete OVGL debug viewport. */
struct ViewportConfiguration
{
    /** @brief Absolute path of the authored RenderProduct to display. */
    std::string renderProductPath;
    /** @brief User-facing SDL window title. */
    std::string title{ "Isaac Sim OVGL Viewport" };
    /** @brief Initial window width in pixels. */
    uint32_t width{ 1280 };
    /** @brief Initial window height in pixels. */
    uint32_t height{ 720 };
    /** @brief Frame limit, or zero until the window closes or the application stops. */
    uint64_t maximumFrames{ 0 };
    /** @brief Whether to create and present a user-visible SDL window. */
    bool visible{ true };
    /** @brief Initial and reset debug camera state. */
    Camera camera;
};

/** @brief Metadata and optional pixels for one OVGL-rendered frame. */
struct Frame
{
    /** @brief Monotonic number assigned by this viewport. */
    uint64_t frameNumber{ 0 };
    /** @brief OVStage ordinal rendered by OVGL. */
    uint64_t stageOrdinal{ 0 };
    /** @brief Image width in pixels. */
    uint32_t width{ 0 };
    /** @brief Image height in pixels. */
    uint32_t height{ 0 };
    /** @brief Tightly packed, top-down RGBA8 pixels, populated only for an invisible viewport. */
    std::vector<std::byte> rgba;
};

/**
 * @brief Interactive OVGL viewport intended for tests and debugging.
 *
 * The viewport borrows an already-populated OVStage. The application remains responsible for authoring the camera and
 * RenderProduct, updating the stage, and advancing any simulation. A typical loop calls pollEvents(), performs its
 * updates, and then calls render().
 *
 * Controls are left-drag mouse look, mouse-wheel dolly, hold W/S for forward/back, hold A/D to strafe, hold Q/E for
 * down/up, Shift for faster movement, R to reset the camera, H to toggle the debug HUD, and Escape to close.
 *
 * @note Create, use, and destroy the viewport on the application main thread.
 */
class ISAACSIM_OVGL_VIEWPORT_API Viewport final
{
public:
    /**
     * @brief Create and attach an OVGL viewport to an OVStage.
     * @param[in] stage Borrowed OVStage instance that outlives the viewport.
     * @param[in] cameraPoseWriter Callback used to author poses on the configured camera prim.
     * @param[in] configuration RenderProduct, window, and debug navigation configuration.
     * @throws std::invalid_argument If an argument or configuration value is invalid.
     * @throws std::runtime_error If OVGL, SDL video, OpenGL context, or presentation-window initialization fails.
     */
    Viewport(ovstage_instance_t* stage, CameraPoseWriter cameraPoseWriter, ViewportConfiguration configuration);

    /** @brief Release rendering and window resources. */
    ~Viewport();

    Viewport(const Viewport&) = delete;
    Viewport& operator=(const Viewport&) = delete;

    /**
     * @brief Move-construct a viewport by transferring its attached renderer and window state.
     * @param[in,out] other Viewport whose resources are transferred.
     */
    Viewport(Viewport&& other) noexcept;

    /**
     * @brief Move-assign a viewport by transferring its attached renderer and window state.
     * @param[in,out] other Viewport whose resources are transferred.
     * @return Reference to this viewport.
     */
    Viewport& operator=(Viewport&& other) noexcept;

    /** @brief Release rendering and window resources. Calling this method more than once is safe. */
    void close() noexcept;

    /** @brief Return whether the viewport has released its rendering resources. */
    bool isClosed() const noexcept;

    /**
     * @brief Process pending input and publish a changed debug camera pose.
     * @return False after the window closes or the configured frame limit is reached.
     */
    bool pollEvents();

    /**
     * @brief Render the latest sealed OVStage ordinal and present it when visible.
     *
     * Visible rendering remains entirely on the GPU and returns frame metadata with an empty RGBA payload. Invisible
     * rendering reads back and returns the captured RGBA pixels.
     *
     * @return Borrowed frame data valid until the next render or viewport destruction.
     * @throws std::runtime_error If OVGL cannot render the configured RenderProduct.
     */
    const Frame& render();

private:
    class Implementation;
    std::unique_ptr<Implementation> m_implementation;
};

} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
