// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#define CARB_EXPORTS

#ifdef _WIN32
#    pragma warning(push)
#    pragma warning(disable : 4996)
#endif

// clang-format off
#include <pch/UsdPCH.h>
#include <pxr/usd/usd/inherits.h>
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/dictionary/DictionaryUtils.h>
#include <carb/logging/Log.h>
#include <carb/tasking/ITasking.h>
#include <carb/tokens/ITokens.h>
#include <carb/tokens/TokensUtils.h>

#include <isaacsim/core/includes/LibraryLoader.h>
#include <isaacsim/ros2/bridge/IRos2Bridge.h>
#include <isaacsim/ros2/bridge/Ros2Distro.h>
#include <isaacsim/ros2/bridge/Ros2Factory.h>
#include <omni/graph/core/iComputeGraph.h>
#include <omni/graph/core/ogn/Registration.h>
#include <omni/kit/IApp.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdTypes.h>

#if defined(_WIN32)
#    include <filesystem>
#else
#    include <experimental/filesystem>
#endif
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.ros2.bridge.plugin", "Isaac ROS2 bridge", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::ros2::bridge::Ros2Bridge)
CARB_PLUGIN_IMPL_DEPS(carb::dictionary::ISerializer,
                      carb::dictionary::IDictionary,
                      omni::kit::IStageUpdate,
                      omni::physx::IPhysx,
                      carb::tasking::ITasking,
                      carb::tokens::ITokens)

DECLARE_OGN_NODES()

namespace
{

omni::kit::StageUpdatePtr g_stageUpdate = nullptr;
omni::kit::StageUpdateNode* g_stageUpdateNode = nullptr;
std::shared_ptr<isaacsim::ros2::bridge::Ros2ContextHandle> g_defaultContextHandle;
std::shared_ptr<isaacsim::core::includes::LibraryLoader> g_factoryLoader;
isaacsim::core::includes::MultiLibraryLoader g_backupLibraryLoader;
std::unique_ptr<isaacsim::ros2::bridge::Ros2Factory> g_factory = nullptr;
std::string g_extensionPath;


uint64_t const CARB_ABI getDefaultContextHandleAddr()
{
    return reinterpret_cast<uint64_t>(&g_defaultContextHandle);
}

isaacsim::ros2::bridge::Ros2Factory* const CARB_ABI getFactory()
{
    return g_factory.get();
}

bool const CARB_ABI getStartupStatus()
{
    if (g_factory && g_defaultContextHandle)
    {
        return true;
    }
    return false;
}

} // namespace anonymous


CARB_EXPORT void carbOnPluginStartup()
{
    omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();
    std::vector<std::string> libraryList = {
        "rcutils",
        "rosidl_runtime_c",
        "rmw",
        "yaml",
        "rcl_yaml_param_parser",
        "rcpputils",
        "rmw_implementation",
#ifndef _WIN32
        "spdlog",
#endif
        "rcl_logging_spdlog",
        "rosidl_typesupport_c",
        "builtin_interfaces__rosidl_generator_c",
        "builtin_interfaces__rosidl_typesupport_c",
        "rcl_interfaces__rosidl_typesupport_c",
        "rcl_interfaces__rosidl_generator_c",
        "rcl",
        "action_msgs__rosidl_typesupport_c",
        "std_msgs__rosidl_generator_c",
        "geometry_msgs__rosidl_generator_c",
        "geometry_msgs__rosidl_typesupport_c",
        "unique_identifier_msgs__rosidl_generator_c",
        "unique_identifier_msgs__rosidl_typesupport_c",
        "tf2_msgs__rosidl_typesupport_c",
        "tf2_msgs__rosidl_generator_c",
        "nav_msgs__rosidl_typesupport_c",
        "nav_msgs__rosidl_generator_c",
        "std_msgs__rosidl_typesupport_c",
        "std_msgs__rosidl_generator_c",
        "rosgraph_msgs__rosidl_typesupport_c",
        "rosgraph_msgs__rosidl_generator_c",
        "sensor_msgs__rosidl_typesupport_c",
        "sensor_msgs__rosidl_generator_c",
        "rcl_action",
        "lifecycle_msgs__rosidl_generator_c",
        "lifecycle_msgs__rosidl_typesupport_c",
        "vision_msgs__rosidl_typesupport_c",
        "vision_msgs__rosidl_generator_c",
        "ackermann_msgs__rosidl_generator_c",
        "ackermann_msgs__rosidl_typesupport_c",
        // "fastcdr",
        // "tinyxml2",
        // "crypto",
        // "ssl",
        // "fastrtps",
        // "rmw_dds_common",
        // "rmw_fastrtps_shared_cpp",
        // "rosidl_typesupport_fastrtps_c",
        // "rosidl_typesupport_fastrtps_cpp",
        // "rosidl_typesupport_cpp",
        // "rmw_dds_common__rosidl_typesupport_cpp",
        // "rmw_fastrtps_cpp",
    };

    char* rosDistro = getenv("ROS_DISTRO");

    if (isaacsim::ros2::bridge::stringToRos2Distro(rosDistro).value() == isaacsim::ros2::bridge::Ros2Distro::eHumble)
    {
        libraryList.insert(libraryList.begin() + 5, std::string("ament_index_cpp"));
        libraryList.insert(libraryList.begin() + 8, std::string("rcl_logging_interface"));
        libraryList.insert(libraryList.end(), std::string("rcl_lifecycle"));
    }

    // Attempt to load a ROS 2 library. If it fails, force load internal Distro
    if (!rosDistro || !isaacsim::ros2::bridge::isRos2DistroSupported(rosDistro))
    {
        CARB_LOG_ERROR("Unsupported ROS_DISTRO or ROS_DISTRO env var not specified: %s", rosDistro);
        return;
    }
    else
    {
        // Load test library, print error if it fails
        auto tempLoader = std::make_shared<isaacsim::core::includes::LibraryLoader>("rosidl_runtime_c", "", false);
        if (!tempLoader->isValid())
        {
#ifdef _WIN32
            app->printAndLog(
                "Loading rosidl_runtime_c.dll from sourced ROS_DISTRO failed, falling back to internal libraries.");
#else
            app->printAndLog(
                "Loading librosidl_runtime_c.so from sourced ROS_DISTRO failed, falling back to internal libraries.");
#endif

            carb::tokens::ITokens* tokens = carb::getCachedInterface<carb::tokens::ITokens>();
#if defined(_WIN32)
            std::filesystem::path p = carb::tokens::resolveString(tokens, "${isaacsim.ros2.bridge}");
#else
            std::experimental::filesystem::path p = carb::tokens::resolveString(tokens, "${isaacsim.ros2.bridge}");
#endif

            g_extensionPath = (p / std::string(rosDistro) / "lib").string();

            // Try and load internal lib, this will fail if ENV vars are not set correctly due to dependency tree.
            // Do not print lib specific errors
            auto tempLoader =
                std::make_shared<isaacsim::core::includes::LibraryLoader>("rosidl_runtime_c", g_extensionPath, false);
            if (!tempLoader->isValid())
            {
                CARB_LOG_WARN(
                    "Could not load ROS2 Bridge due to missing library dependencies, please make sure your sourced ROS2 workspace has the correct packages/libraries installed");
                return;
            }
            for (std::string lib : libraryList)
            {
                g_backupLibraryLoader.loadLibrary(lib, g_extensionPath);
            }
        }
        g_factoryLoader =
            std::make_shared<isaacsim::core::includes::LibraryLoader>("isaacsim.ros2.bridge." + std::string(rosDistro));
    }

    g_stageUpdate = carb::getCachedInterface<omni::kit::IStageUpdate>()->getStageUpdate();

    omni::kit::StageUpdateNodeDesc desc = { nullptr };
    desc.displayName = "IsaacRos2Bridge";

    desc.order = 100;
    g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);

    if (g_factoryLoader)
    {
        typedef isaacsim::ros2::bridge::Ros2Factory* (*createFactory_binding)(void);
        createFactory_binding createFactory = (g_factoryLoader->getSymbol<createFactory_binding>("createFactoryC"));

        if (createFactory)
        {
            g_factory = std::unique_ptr<isaacsim::ros2::bridge::Ros2Factory>(createFactory());
        }
        else
        {
            CARB_LOG_WARN(
                "Create Factory Failed: Could not load ROS2 Bridge due to missing library dependencies, please make sure your sourced ROS2 workspace has the correct packages/libraries installed");
            return;
        }
    }

    g_defaultContextHandle = g_factory->createContextHandle();


    if (!g_defaultContextHandle->isValid())
    {
        CARB_LOG_INFO("rcl::init()");
        int argc = 0;
        char** argv = nullptr;
        g_defaultContextHandle->init(argc, argv);
    }
    else
    {
        CARB_LOG_INFO("ROS2 already initialized");
    }
    INITIALIZE_OGN_NODES()
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (g_stageUpdateNode)
    {
        g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
        g_stageUpdateNode = nullptr;
    }

    if (g_defaultContextHandle->isValid())
    {
        CARB_LOG_INFO("rcl::shutdown()");
        g_defaultContextHandle->shutdown();
    }

    g_defaultContextHandle.reset();

    RELEASE_OGN_NODES()
    g_factory.reset();
    g_factoryLoader.reset();
}

void fillInterface(isaacsim::ros2::bridge::Ros2Bridge& iface)
{
    using namespace isaacsim::ros2::bridge;

    memset(&iface, 0, sizeof(iface));
    iface.getDefaultContextHandleAddr = getDefaultContextHandleAddr;
    iface.getFactory = getFactory;
    iface.getStartupStatus = getStartupStatus;
}

#ifdef _WIN32
#    pragma warning(pop)
#endif
