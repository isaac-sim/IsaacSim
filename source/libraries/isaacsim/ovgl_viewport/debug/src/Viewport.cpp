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

#include "isaacsim/ovgl_viewport/debug/Viewport.hpp"

#include "Renderer.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace isaacsim
{
namespace ovgl_viewport
{
namespace debug
{
namespace
{

constexpr double g_kPi = 3.14159265358979323846;
constexpr double g_kMaximumPitch = 0.49 * g_kPi;
constexpr double g_kMovementUpdatesPerSecond = 25.0;
constexpr double g_kMaximumMovementDeltaSeconds = 0.1;
constexpr double g_kMaximumMouseDeltaPerPoll = 25.0;
constexpr double g_kFpsSampleSeconds = 0.5;

class FrameRateCounter final
{
public:
    void recordFrame(std::chrono::steady_clock::time_point now)
    {
        if (m_hasPreviousFrame)
        {
            m_sampleSeconds += std::chrono::duration<double>(now - m_previousFrame).count();
            ++m_sampleIntervals;
            if (m_sampleSeconds >= g_kFpsSampleSeconds)
            {
                m_framesPerSecond = static_cast<double>(m_sampleIntervals) / m_sampleSeconds;
                m_sampleSeconds = 0.0;
                m_sampleIntervals = 0;
            }
        }
        m_previousFrame = now;
        m_hasPreviousFrame = true;
    }

    double getFramesPerSecond() const
    {
        return m_framesPerSecond;
    }

private:
    std::chrono::steady_clock::time_point m_previousFrame;
    double m_sampleSeconds{ 0.0 };
    double m_framesPerSecond{ 0.0 };
    uint64_t m_sampleIntervals{ 0 };
    bool m_hasPreviousFrame{ false };
};

std::array<double, 3> normalize(const std::array<double, 3>& value)
{
    const double length = std::hypot(value[0], value[1], value[2]);
    if (!std::isfinite(length) || length <= std::numeric_limits<double>::epsilon())
    {
        throw std::runtime_error("Cannot normalize a zero-length camera vector");
    }
    return { value[0] / length, value[1] / length, value[2] / length };
}

std::array<double, 3> cross(const std::array<double, 3>& left, const std::array<double, 3>& right)
{
    return { left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2],
             left[0] * right[1] - left[1] * right[0] };
}

std::array<double, 3> getCameraEye(const Camera& camera)
{
    const double horizontal = std::cos(camera.pitchRadians);
    const std::array<double, 3> eye{
        camera.target[0] + camera.distance * horizontal * std::cos(camera.yawRadians),
        camera.target[1] + camera.distance * horizontal * std::sin(camera.yawRadians),
        camera.target[2] + camera.distance * std::sin(camera.pitchRadians),
    };
    if (!std::all_of(eye.begin(), eye.end(), [](double value) { return std::isfinite(value); }))
    {
        throw std::runtime_error("Viewport camera eye is outside the finite coordinate range");
    }
    return eye;
}

CameraPose getCameraPose(const Camera& camera)
{
    const std::array<double, 3> eye = getCameraEye(camera);
    const std::array<double, 3> forward =
        normalize({ camera.target[0] - eye[0], camera.target[1] - eye[1], camera.target[2] - eye[2] });
    const std::array<double, 3> right = normalize(cross(forward, { 0.0, 0.0, 1.0 }));
    const std::array<double, 3> up = normalize(cross(right, forward));

    const std::array<double, 3> backward{ -forward[0], -forward[1], -forward[2] };
    const double matrix[3][3] = { { right[0], right[1], right[2] },
                                  { up[0], up[1], up[2] },
                                  { backward[0], backward[1], backward[2] } };
    std::array<double, 4> orientation{};
    const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    if (trace > 0.0)
    {
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        orientation = { (matrix[1][2] - matrix[2][1]) / scale, (matrix[2][0] - matrix[0][2]) / scale,
                        (matrix[0][1] - matrix[1][0]) / scale, 0.25 * scale };
    }
    else
    {
        size_t diagonal = 0;
        if (matrix[1][1] > matrix[diagonal][diagonal])
        {
            diagonal = 1;
        }
        if (matrix[2][2] > matrix[diagonal][diagonal])
        {
            diagonal = 2;
        }
        const size_t next = (diagonal + 1) % 3;
        const size_t last = (diagonal + 2) % 3;
        const double scale = 2.0 * std::sqrt(1.0 + matrix[diagonal][diagonal] - matrix[next][next] - matrix[last][last]);
        orientation[diagonal] = 0.25 * scale;
        orientation[3] = (matrix[next][last] - matrix[last][next]) / scale;
        orientation[next] = (matrix[diagonal][next] + matrix[next][diagonal]) / scale;
        orientation[last] = (matrix[diagonal][last] + matrix[last][diagonal]) / scale;
    }
    return { eye, orientation };
}

void translateCamera(Camera& camera,
                     double forwardAmount,
                     double rightAmount,
                     double verticalAmount,
                     bool accelerated,
                     double elapsedSeconds)
{
    const std::array<double, 3> forward{ -std::cos(camera.yawRadians), -std::sin(camera.yawRadians), 0.0 };
    const std::array<double, 3> right{ -std::sin(camera.yawRadians), std::cos(camera.yawRadians), 0.0 };
    double step = std::clamp(camera.distance * 0.04, 0.1, 2.0);
    if (accelerated)
    {
        step *= 4.0;
    }
    step *= g_kMovementUpdatesPerSecond * elapsedSeconds;
    for (size_t axis = 0; axis < camera.target.size(); ++axis)
    {
        camera.target[axis] += step * (forwardAmount * forward[axis] + rightAmount * right[axis]);
    }
    camera.target[2] += step * verticalAmount;
}

void rotateCamera(Camera& camera, double deltaX, double deltaY, uint32_t viewportWidth, uint32_t viewportHeight)
{
    const std::array<double, 3> eye = getCameraEye(camera);
    const double yawDelta = deltaX * g_kPi / static_cast<double>(std::max(viewportWidth, 1U));
    const double pitchDelta = deltaY * (0.5 * g_kPi) / static_cast<double>(std::max(viewportHeight, 1U));
    camera.yawRadians = std::remainder(camera.yawRadians - yawDelta, 2.0 * g_kPi);
    camera.pitchRadians = std::clamp(camera.pitchRadians + pitchDelta, -g_kMaximumPitch, g_kMaximumPitch);

    const double horizontal = std::cos(camera.pitchRadians);
    const std::array<double, 3> eyeOffset{
        camera.distance * horizontal * std::cos(camera.yawRadians),
        camera.distance * horizontal * std::sin(camera.yawRadians),
        camera.distance * std::sin(camera.pitchRadians),
    };
    for (size_t axis = 0; axis < camera.target.size(); ++axis)
    {
        camera.target[axis] = eye[axis] - eyeOffset[axis];
    }
}

std::runtime_error makeSdlError(std::string_view operation)
{
    const char* detail = SDL_GetError();
    return std::runtime_error(detail && detail[0] != '\0' ? std::string(operation) + ": " + detail :
                                                            std::string(operation));
}

[[noreturn]] void throwSdlError(std::string_view operation)
{
    throw makeSdlError(operation);
}

class SdlWindow final
{
public:
    SdlWindow(uint32_t width, uint32_t height, const std::string& title)
    {
#if !defined(_WIN32)
        // NVIDIA may otherwise override SDL's zero swap interval and quantize a late frame to the next vblank.
        (void)SDL_setenv_unsafe("__GL_SYNC_TO_VBLANK", "0", 0);
#endif
        if (!SDL_InitSubSystem(SDL_INIT_VIDEO))
        {
            throwSdlError("Unable to initialize SDL video");
        }
        m_videoInitialized = true;
#if defined(_WIN32)
        const int contextMajorVersion = 4;
        const int contextMinorVersion = 1;
        const int contextProfile = SDL_GL_CONTEXT_PROFILE_CORE;
#else
        const int contextMajorVersion = 3;
        const int contextMinorVersion = 2;
        const int contextProfile = SDL_GL_CONTEXT_PROFILE_ES;
#endif
        if (!SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, contextMajorVersion) ||
            !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, contextMinorVersion) ||
            !SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, contextProfile) ||
            !SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1) || !SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0) ||
            !SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0) || !SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0))
        {
            const std::runtime_error error = makeSdlError("Unable to configure the SDL OpenGL context");
            _reset();
            throw error;
        }
        m_window = SDL_CreateWindow(
            title.c_str(), static_cast<int>(width), static_cast<int>(height), SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        if (!m_window)
        {
            const std::runtime_error error = makeSdlError("Unable to create SDL viewport window");
            _reset();
            throw error;
        }
        m_windowId = SDL_GetWindowID(m_window);
        if (m_windowId == 0)
        {
            const std::runtime_error error = makeSdlError("Unable to query the SDL viewport window identifier");
            _reset();
            throw error;
        }
        m_context = SDL_GL_CreateContext(m_window);
        if (!m_context)
        {
            const std::runtime_error error = makeSdlError("Unable to create the SDL OpenGL context");
            _reset();
            throw error;
        }
        makeCurrent();
        (void)SDL_GL_SetSwapInterval(0);
    }

    ~SdlWindow()
    {
        _reset();
    }

    SdlWindow(const SdlWindow&) = delete;
    SdlWindow& operator=(const SdlWindow&) = delete;

    SDL_WindowID getId() const
    {
        return m_windowId;
    }

    std::array<uint32_t, 2> getPixelSize() const
    {
        int width = 0;
        int height = 0;
        if (!SDL_GetWindowSizeInPixels(m_window, &width, &height))
        {
            throwSdlError("Unable to query the SDL viewport pixel size");
        }
        return { static_cast<uint32_t>(std::max(width, 1)), static_cast<uint32_t>(std::max(height, 1)) };
    }

    std::array<uint32_t, 2> getSize() const
    {
        int width = 0;
        int height = 0;
        if (!SDL_GetWindowSize(m_window, &width, &height))
        {
            throwSdlError("Unable to query the SDL viewport size");
        }
        return { static_cast<uint32_t>(std::max(width, 1)), static_cast<uint32_t>(std::max(height, 1)) };
    }

    void setMouseCapture(bool enabled)
    {
        // Capture keeps an active drag working outside the window. Motion inside
        // the window remains usable when a platform cannot provide capture.
        (void)SDL_CaptureMouse(enabled);
    }

    void makeCurrent()
    {
        if (!SDL_GL_MakeCurrent(m_window, m_context))
        {
            throwSdlError("Unable to make the SDL OpenGL context current");
        }
    }

    void present()
    {
        if (!SDL_GL_SwapWindow(m_window))
        {
            throwSdlError("Unable to swap the SDL viewport window");
        }
    }

private:
    void _reset()
    {
        if (m_context && SDL_GL_GetCurrentContext() == m_context)
            (void)SDL_GL_MakeCurrent(m_window, nullptr);
        SDL_GL_DestroyContext(m_context);
        m_context = nullptr;
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
        m_windowId = 0;
        if (m_videoInitialized)
        {
            SDL_QuitSubSystem(SDL_INIT_VIDEO);
            m_videoInitialized = false;
        }
    }

    SDL_Window* m_window{ nullptr };
    SDL_WindowID m_windowId{ 0 };
    SDL_GLContext m_context{ nullptr };
    bool m_videoInitialized{ false };
};

} // namespace

class Viewport::Implementation
{
public:
    Implementation(ovstage_instance_t* stage, CameraPoseWriter cameraPoseWriter, ViewportConfiguration configuration)
        : m_cameraPoseWriter(std::move(cameraPoseWriter)),
          m_configuration(std::move(configuration)),
          m_camera(m_configuration.camera),
          m_resetCamera(m_configuration.camera),
          m_width(m_configuration.width),
          m_height(m_configuration.height)
    {
        if (!m_cameraPoseWriter)
        {
            throw std::invalid_argument("Viewport camera-pose writer must not be empty");
        }
        if (m_configuration.width == 0 || m_configuration.height == 0 ||
            m_configuration.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            m_configuration.height > static_cast<uint32_t>(std::numeric_limits<int>::max()))
        {
            throw std::invalid_argument("Viewport dimensions must be positive SDL-compatible pixel extents");
        }
        if (m_configuration.renderProductPath.empty() || m_configuration.renderProductPath.front() != '/')
        {
            throw std::invalid_argument("Viewport RenderProduct path must be absolute");
        }
        const bool finiteTarget = std::all_of(
            m_camera.target.begin(), m_camera.target.end(), [](double value) { return std::isfinite(value); });
        if (!finiteTarget || !std::isfinite(m_camera.yawRadians) || !std::isfinite(m_camera.pitchRadians) ||
            !std::isfinite(m_camera.distance) || m_camera.distance <= 0.0 ||
            std::abs(m_camera.pitchRadians) > g_kMaximumPitch)
        {
            throw std::invalid_argument("Viewport camera pose must be finite, non-singular, and have positive distance");
        }
        if (m_configuration.visible)
        {
            m_window = std::make_unique<SdlWindow>(m_width, m_height, m_configuration.title);
            const auto pixelSize = m_window->getPixelSize();
            m_width = pixelSize[0];
            m_height = pixelSize[1];
        }
        m_renderer = std::make_unique<details::Renderer>(stage);
    }

    ~Implementation()
    {
        close();
    }

    bool pollEvents()
    {
        if (!m_running || (m_configuration.maximumFrames != 0 && m_frame.frameNumber >= m_configuration.maximumFrames))
        {
            m_running = false;
            return false;
        }
        const auto pollTime = std::chrono::steady_clock::now();
        const double elapsedSeconds = std::chrono::duration<double>(pollTime - m_previousPollTime).count();
        m_previousPollTime = pollTime;
        const bool movementWasActive = _hasMovementInput();
        if (m_window)
        {
            SDL_Event event{};
            while (SDL_PollEvent(&event))
            {
                _handleEvent(event);
            }
        }
        _applyMouseLook();
        if (m_running && _hasMovementInput())
        {
            _applyMovement(movementWasActive ? std::clamp(elapsedSeconds, 0.0, g_kMaximumMovementDeltaSeconds) : 0.0);
        }
        if (m_running && m_cameraDirty)
        {
            m_cameraPoseWriter(getCameraPose(m_camera));
            m_cameraDirty = false;
        }
        return m_running;
    }

    void close() noexcept
    {
        m_running = false;
        if (m_window && m_renderer)
        {
            try
            {
                m_window->makeCurrent();
            }
            catch (...)
            {
            }
        }
        m_renderer.reset();
        m_window.reset();
        m_cameraPoseWriter = {};
    }

    bool isClosed() const noexcept
    {
        return !m_renderer;
    }

    const Frame& render()
    {
        if (!m_running)
        {
            throw std::runtime_error("Cannot render a closed OVGL viewport");
        }
        if (m_window)
        {
            m_frameRateCounter.recordFrame(std::chrono::steady_clock::now());
            char fpsText[32]{};
            const char* overlayText = nullptr;
            if (m_hudVisible)
            {
                const double framesPerSecond = m_frameRateCounter.getFramesPerSecond();
                if (framesPerSecond > 0.0)
                    std::snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", framesPerSecond);
                else
                    std::snprintf(fpsText, sizeof(fpsText), "FPS: --");
                overlayText = fpsText;
            }
            m_window->makeCurrent();
            m_renderer->present(m_configuration.renderProductPath, m_width, m_height, overlayText, m_frame);
            m_window->present();
        }
        else
            m_renderer->render(m_configuration.renderProductPath, m_frame);
        return m_frame;
    }

private:
    void _handleEvent(const SDL_Event& event)
    {
        switch (event.type)
        {
        case SDL_EVENT_QUIT:
            m_running = false;
            break;
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            if (event.window.windowID == m_window->getId())
            {
                m_running = false;
            }
            break;
        case SDL_EVENT_KEY_DOWN:
            if (event.key.windowID == m_window->getId() && !event.key.repeat)
            {
                _handleKey(event.key.scancode, true);
            }
            break;
        case SDL_EVENT_KEY_UP:
            if (event.key.windowID == m_window->getId())
            {
                _handleKey(event.key.scancode, false);
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (event.button.windowID == m_window->getId() && event.button.button == SDL_BUTTON_LEFT)
            {
                m_pendingMouseDelta = {};
                m_previousMousePosition = { event.button.x, event.button.y };
                m_dragging = true;
                m_window->setMouseCapture(true);
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event.button.windowID == m_window->getId() && event.button.button == SDL_BUTTON_LEFT)
            {
                m_dragging = false;
                m_window->setMouseCapture(false);
            }
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (event.wheel.windowID == m_window->getId())
            {
                const double wheelY = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -event.wheel.y : event.wheel.y;
                const double scale = std::pow(0.88, wheelY);
                m_camera.distance = std::clamp(m_camera.distance * scale, 0.25, 10000.0);
                m_cameraDirty = true;
            }
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (event.motion.windowID == m_window->getId() && m_dragging)
            {
                const double deltaX = event.motion.x - m_previousMousePosition[0];
                const double deltaY = event.motion.y - m_previousMousePosition[1];
                m_previousMousePosition = { event.motion.x, event.motion.y };
                if (std::isfinite(deltaX) && std::isfinite(deltaY))
                {
                    m_pendingMouseDelta[0] += deltaX;
                    m_pendingMouseDelta[1] += deltaY;
                }
            }
            break;
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            if (event.window.windowID == m_window->getId())
            {
                m_width = static_cast<uint32_t>(std::max(event.window.data1, 1));
                m_height = static_cast<uint32_t>(std::max(event.window.data2, 1));
            }
            break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            if (event.window.windowID == m_window->getId())
            {
                _clearInput();
            }
            break;
        default:
            break;
        }
    }

    void _handleKey(SDL_Scancode key, bool pressed)
    {
        if (key == SDL_SCANCODE_ESCAPE && pressed)
        {
            m_running = false;
            return;
        }
        if (key == SDL_SCANCODE_R)
        {
            if (pressed && !m_resetPressed)
            {
                m_camera = m_resetCamera;
                m_cameraDirty = true;
            }
            m_resetPressed = pressed;
            return;
        }
        if (key == SDL_SCANCODE_H)
        {
            if (pressed && !m_hudPressed)
            {
                m_hudVisible = !m_hudVisible;
            }
            m_hudPressed = pressed;
            return;
        }
        if (key == SDL_SCANCODE_LSHIFT)
        {
            m_leftShiftPressed = pressed;
            return;
        }
        if (key == SDL_SCANCODE_RSHIFT)
        {
            m_rightShiftPressed = pressed;
            return;
        }
        if (key == SDL_SCANCODE_W)
        {
            m_movementKeys.forward = pressed;
        }
        else if (key == SDL_SCANCODE_S)
        {
            m_movementKeys.backward = pressed;
        }
        else if (key == SDL_SCANCODE_A)
        {
            m_movementKeys.left = pressed;
        }
        else if (key == SDL_SCANCODE_D)
        {
            m_movementKeys.right = pressed;
        }
        else if (key == SDL_SCANCODE_Q)
        {
            m_movementKeys.down = pressed;
        }
        else if (key == SDL_SCANCODE_E)
        {
            m_movementKeys.up = pressed;
        }
    }

    bool _hasMovementInput() const
    {
        return m_movementKeys.forward || m_movementKeys.backward || m_movementKeys.left || m_movementKeys.right ||
               m_movementKeys.down || m_movementKeys.up;
    }

    void _applyMouseLook()
    {
        double deltaX = m_pendingMouseDelta[0];
        double deltaY = m_pendingMouseDelta[1];
        m_pendingMouseDelta = {};
        if (!std::isfinite(deltaX) || !std::isfinite(deltaY))
        {
            return;
        }

        // SDL can queue several drag events between slow frames. Bound their
        // combined motion once per poll so a delayed frame cannot fling the camera.
        const double magnitude = std::hypot(deltaX, deltaY);
        if (magnitude > g_kMaximumMouseDeltaPerPoll)
        {
            const double scale = g_kMaximumMouseDeltaPerPoll / magnitude;
            deltaX *= scale;
            deltaY *= scale;
        }
        if (magnitude > std::numeric_limits<double>::epsilon())
        {
            const std::array<uint32_t, 2> viewportSize = m_window->getSize();
            rotateCamera(m_camera, deltaX, deltaY, viewportSize[0], viewportSize[1]);
            m_cameraDirty = true;
        }
    }

    void _applyMovement(double elapsedSeconds)
    {
        double forwardAmount = static_cast<double>(m_movementKeys.forward) - m_movementKeys.backward;
        double rightAmount = static_cast<double>(m_movementKeys.right) - m_movementKeys.left;
        double verticalAmount = static_cast<double>(m_movementKeys.up) - m_movementKeys.down;
        const double magnitude = std::hypot(forwardAmount, rightAmount, verticalAmount);
        if (magnitude <= std::numeric_limits<double>::epsilon() || elapsedSeconds <= 0.0)
        {
            return;
        }
        forwardAmount /= magnitude;
        rightAmount /= magnitude;
        verticalAmount /= magnitude;
        translateCamera(m_camera, forwardAmount, rightAmount, verticalAmount, m_leftShiftPressed || m_rightShiftPressed,
                        elapsedSeconds);
        m_cameraDirty = true;
    }

    void _clearInput()
    {
        if (m_window && m_dragging)
        {
            m_window->setMouseCapture(false);
        }
        m_movementKeys = {};
        m_leftShiftPressed = false;
        m_rightShiftPressed = false;
        m_resetPressed = false;
        m_hudPressed = false;
        m_dragging = false;
        m_pendingMouseDelta = {};
    }

    struct MovementKeys
    {
        bool forward{ false };
        bool backward{ false };
        bool left{ false };
        bool right{ false };
        bool down{ false };
        bool up{ false };
    };

    std::unique_ptr<details::Renderer> m_renderer;
    CameraPoseWriter m_cameraPoseWriter;
    ViewportConfiguration m_configuration;
    Camera m_camera;
    Camera m_resetCamera;
    std::unique_ptr<SdlWindow> m_window;
    Frame m_frame;
    uint32_t m_width;
    uint32_t m_height;
    std::chrono::steady_clock::time_point m_previousPollTime{ std::chrono::steady_clock::now() };
    FrameRateCounter m_frameRateCounter;
    MovementKeys m_movementKeys;
    std::array<double, 2> m_pendingMouseDelta{};
    std::array<double, 2> m_previousMousePosition{};
    bool m_running{ true };
    bool m_cameraDirty{ true };
    bool m_dragging{ false };
    bool m_leftShiftPressed{ false };
    bool m_rightShiftPressed{ false };
    bool m_resetPressed{ false };
    bool m_hudPressed{ false };
    bool m_hudVisible{ false };
};

Viewport::Viewport(ovstage_instance_t* stage, CameraPoseWriter cameraPoseWriter, ViewportConfiguration configuration)
    : m_implementation(std::make_unique<Implementation>(stage, std::move(cameraPoseWriter), std::move(configuration)))
{
}

Viewport::~Viewport() = default;
Viewport::Viewport(Viewport&& other) noexcept = default;
Viewport& Viewport::operator=(Viewport&& other) noexcept = default;

void Viewport::close() noexcept
{
    if (m_implementation)
    {
        m_implementation->close();
    }
}

bool Viewport::isClosed() const noexcept
{
    return !m_implementation || m_implementation->isClosed();
}

bool Viewport::pollEvents()
{
    return m_implementation && m_implementation->pollEvents();
}

const Frame& Viewport::render()
{
    if (!m_implementation)
    {
        throw std::runtime_error("Cannot render a moved-from OVGL viewport");
    }
    return m_implementation->render();
}

} // namespace debug
} // namespace ovgl_viewport
} // namespace isaacsim
