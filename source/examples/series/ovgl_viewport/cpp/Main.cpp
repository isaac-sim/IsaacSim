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

#include "isaacsim/foundation/objects/Camera.hpp"
#include "isaacsim/foundation/objects/Prim.hpp"
#include "isaacsim/foundation/objects/Stage.hpp"
#include "isaacsim/foundation/objects/lights/DistantLight.hpp"
#include "isaacsim/foundation/objects/shapes/Cube.hpp"
#include "isaacsim/ovgl_viewport/debug/Viewport.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{

namespace array = isaacsim::common::array;
namespace foundation = isaacsim::foundation::objects;
namespace viewport = isaacsim::ovgl_viewport::debug;

constexpr uint32_t g_kInitialWidth = 1280;
constexpr uint32_t g_kInitialHeight = 720;
constexpr const char* g_kCameraPath = "/IsaacSimViewportCamera";
constexpr const char* g_kColorPath = "/IsaacSimViewportColor";
constexpr const char* g_kRenderProductPath = "/IsaacSimViewportRenderProduct";

struct CommandLine
{
    std::string stageSource;
    uint64_t maximumFrames{ 0 };
    bool headless{ false };
};

void printUsage(const char* executable)
{
    std::cout << "Usage: " << executable << " [USD_PATH_OR_URL] [--frames COUNT] [--headless]\n\n"
              << "Displays an optional USD scene, or a procedural cube when no source is provided,\n"
              << "using OVStage and isaacsim.ovgl_viewport.debug.\n\n"
              << "Controls:\n"
              << "  Drag with the left mouse button to look around\n"
              << "  Mouse wheel     Dolly\n"
              << "  Hold W/S        Move forward/backward\n"
              << "  Hold A/D        Strafe left/right\n"
              << "  Hold Q/E        Move down/up\n"
              << "  Shift+movement  Move faster\n"
              << "  R               Reset view\n"
              << "  H               Toggle debug HUD\n"
              << "  Escape          Quit\n\n"
              << "Options:\n"
              << "  --frames COUNT  Stop after rendering COUNT frames (zero means no limit)\n"
              << "  --headless      Render without creating a user-visible window\n";
}

CommandLine parseCommandLine(int argumentCount, char** arguments)
{
    CommandLine commandLine;
    bool stageSourceSet = false;
    for (int index = 1; index < argumentCount; ++index)
    {
        const std::string argument = arguments[index];
        if (argument == "--help" || argument == "-h")
        {
            printUsage(arguments[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--frames")
        {
            if (++index >= argumentCount)
            {
                throw std::invalid_argument("--frames requires a non-negative integer");
            }
            const std::string frameCount = arguments[index];
            size_t parsedCharacters = 0;
            if (frameCount.empty() || frameCount.front() == '-')
            {
                throw std::invalid_argument("--frames requires a non-negative integer");
            }
            try
            {
                commandLine.maximumFrames = std::stoull(frameCount, &parsedCharacters);
            }
            catch (const std::exception&)
            {
                throw std::invalid_argument("--frames requires a non-negative integer");
            }
            if (parsedCharacters != frameCount.size())
            {
                throw std::invalid_argument("--frames requires a non-negative integer");
            }
            continue;
        }
        if (argument == "--headless")
        {
            commandLine.headless = true;
            continue;
        }
        if (!argument.empty() && argument[0] == '-')
        {
            throw std::invalid_argument("Unknown option: " + argument);
        }
        if (stageSourceSet)
        {
            throw std::invalid_argument("Only one USD source may be provided");
        }
        commandLine.stageSource = argument;
        stageSourceSet = true;
    }
    return commandLine;
}

bool isRemoteStageSource(std::string_view source)
{
    constexpr std::string_view schemes[] = { "file://", "http://", "https://", "omni://", "omniverse://" };
    for (const std::string_view scheme : schemes)
    {
        if (source.compare(0, scheme.size(), scheme) == 0)
        {
            return true;
        }
    }
    return false;
}

std::string getStageLayerIdentifier(const std::string& stageSource)
{
    const std::string identifier = isRemoteStageSource(stageSource) ?
                                       stageSource :
                                       std::filesystem::absolute(stageSource).lexically_normal().string();
    if (identifier.find_first_of("@\r\n") != std::string::npos)
    {
        throw std::invalid_argument("USD sources containing '@' or a newline are not supported");
    }
    return identifier;
}

void insertSublayer(std::string& stageText, const std::string& stageSource)
{
    const size_t metadataBody = stageText.find("(\n");
    if (metadataBody == std::string::npos)
    {
        throw std::runtime_error("Foundation exported a layer without a metadata block");
    }
    stageText.insert(metadataBody + 2, "    subLayers = [@" + getStageLayerIdentifier(stageSource) + "@]\n");
}

void insertRenderProductRelationships(std::string& stageText)
{
    const std::string declaration =
        "def RenderProduct \"" + std::filesystem::path(g_kRenderProductPath).filename().string() + "\"";
    const size_t primStart = stageText.find(declaration);
    const size_t primBody = stageText.find("{\n", primStart);
    if (primStart == std::string::npos || primBody == std::string::npos)
    {
        throw std::runtime_error("Foundation did not export the viewport RenderProduct");
    }

    const std::string relationships = "    rel camera = <" + std::string(g_kCameraPath) + ">\n" +
                                      "    rel orderedVars = [<" + std::string(g_kColorPath) + ">]\n";
    stageText.insert(primBody + 2, relationships);
}

std::string buildStageText(const std::string& stageSource)
{
    foundation::Stage authoringStage("openusd");
    try
    {
        if (stageSource.empty())
        {
            authoringStage.createStage();
            authoringStage.setUnits(/*metersPerUnit=*/1.0f);
            authoringStage.setUpAxis("Z");
        }
        else
        {
            // Starting from an empty layer avoids overriding the sublayer's units and up-axis with createStage
            // defaults.
            authoringStage.importStageFromString("#usda 1.0\n");
        }

        foundation::Camera camera(g_kCameraPath);
        camera.setFocalLengths(array::Array(1.8147562));
        camera.setApertures(array::Array(2.0955));
        camera.setClippingRanges(array::Array(1.0), array::Array(10000000.0));
        camera.setWorldPoses(array::Array(std::vector<double>{ 0.0, 0.0, 6.0 }),
                             array::Array(std::vector<double>{ 0.0, 0.0, 0.0, 1.0 }));

        authoringStage.definePrim(g_kColorPath, "RenderVar");
        foundation::Prim color(g_kColorPath);
        color.createAttribute("dataType", "token");
        color.createAttribute("sourceName", "string");
        color.setAttributeValues("dataType", std::string("color4f"));
        color.setAttributeValues("sourceName", std::string("LdrColor"));

        authoringStage.definePrim(g_kRenderProductPath, "RenderProduct");
        foundation::Prim renderProduct(g_kRenderProductPath);
        renderProduct.createAttribute("resolution", "int2");
        renderProduct.setAttributeValues(
            "resolution", array::Array(std::vector<int32_t>{ static_cast<int32_t>(g_kInitialWidth),
                                                             static_cast<int32_t>(g_kInitialHeight) }));

        if (stageSource.empty())
        {
            foundation::shapes::Cube cube("/World/Cube", array::Array(1.0));
            foundation::lights::DistantLight light("/World/KeyLight");
            light.setIntensities(array::Array(3000.0f));
            light.setColors(array::Array(std::vector<float>{ 1.0f, 0.92f, 0.78f }));
        }
    }
    catch (...)
    {
        if (authoringStage.isValid())
        {
            authoringStage.closeStage();
        }
        throw;
    }
    std::string stageText = authoringStage.exportStageToString();
    if (!authoringStage.closeStage())
    {
        throw std::runtime_error("Unable to close the Foundation authoring stage");
    }
    if (stageText.empty())
    {
        throw std::runtime_error("Foundation failed to export the viewport stage");
    }

    // Foundation C++ does not yet expose sublayer or relationship-target authoring. Keep the USDA boundary adapter
    // limited to those unsupported composition fields; all schema prims and attributes above use Foundation objects.
    if (!stageSource.empty())
    {
        insertSublayer(stageText, stageSource);
    }
    insertRenderProductRelationships(stageText);
    return stageText;
}

int runViewer(const CommandLine& commandLine)
{
    if (!std::getenv("OVGL_SS"))
    {
#ifdef _WIN32
        _putenv_s("OVGL_SS", "1");
#else
        setenv("OVGL_SS", "1", 0);
#endif
    }
    if (!commandLine.stageSource.empty() && !isRemoteStageSource(commandLine.stageSource) &&
        !std::filesystem::is_regular_file(commandLine.stageSource))
    {
        throw std::runtime_error("USD stage does not exist: " + commandLine.stageSource);
    }

    if (commandLine.stageSource.empty())
    {
        std::cout << "Populating OVStage with the procedural cube ..." << std::endl;
    }
    else
    {
        std::cout << "Populating OVStage from " << commandLine.stageSource << " ..." << std::endl;
    }
    foundation::Stage stage("ovstage");
    stage.importStageFromString(buildStageText(commandLine.stageSource));
    if (!stage.isValid())
    {
        throw std::runtime_error("OVStage failed to populate the stage");
    }

    try
    {
        std::cout << "OVStage is ready." << std::endl;
        foundation::Camera camera(g_kCameraPath, /*resetXformOpProperties=*/false);

        viewport::ViewportConfiguration viewportConfiguration;
        viewportConfiguration.title = "Isaac Sim OVStage + OVGL Viewport";
        viewportConfiguration.width = g_kInitialWidth;
        viewportConfiguration.height = g_kInitialHeight;
        viewportConfiguration.maximumFrames = commandLine.maximumFrames;
        viewportConfiguration.visible = !commandLine.headless;
        viewportConfiguration.renderProductPath = g_kRenderProductPath;
        void* nativeStage = stage.getStagePtr();
        if (!nativeStage)
        {
            throw std::runtime_error("Foundation did not retain a native stage for the loaded scene");
        }
        const viewport::CameraPoseWriter cameraPoseWriter = [&camera](const viewport::CameraPose& pose)
        {
            array::Array positions(
                std::vector<std::vector<double>>{ { pose.position[0], pose.position[1], pose.position[2] } });
            array::Array orientations(std::vector<std::vector<double>>{
                { pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3] } });
            camera.setWorldPoses(positions, orientations);
        };
        viewport::Viewport viewer(static_cast<ovstage_instance_t*>(nativeStage), cameraPoseWriter, viewportConfiguration);
        uint64_t renderedFrames = 0;
        while (viewer.pollEvents())
        {
            renderedFrames = viewer.render().frameNumber;
        }
        std::cout << "Viewport rendered " << renderedFrames << " frame(s)." << std::endl;
    }
    catch (...)
    {
        stage.closeStage();
        throw;
    }
    if (!stage.closeStage())
    {
        throw std::runtime_error("Unable to close OVStage");
    }
    return EXIT_SUCCESS;
}

} // namespace

int main(int argumentCount, char** arguments)
{
    try
    {
        return runViewer(parseCommandLine(argumentCount, arguments));
    }
    catch (const std::exception& exception)
    {
        std::cerr << "OVGL viewport error: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
