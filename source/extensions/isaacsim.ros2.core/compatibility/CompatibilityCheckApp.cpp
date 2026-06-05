// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/core/includes/LibraryLoader.h>
#include <isaacsim/ros2/core/Ros2Distro.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>


int main(int argc, const char* argv[])
{
    std::vector<std::string> libList = {
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
    };

    char* rosDistro = getenv("ROS_DISTRO");

    if (!rosDistro)
    {
        CARB_LOG_ERROR("ROS_DISTRO env var not specified");
        return EXIT_FAILURE;
    }

    // Humble-specific extra libraries (safe check for unknown distros)
    auto knownDistro = isaacsim::ros2::core::stringToRos2Distro(rosDistro);
    if (knownDistro.has_value() && *knownDistro == isaacsim::ros2::core::Ros2Distro::eHumble)
    {
        libList.insert(libList.begin() + 5, std::string("ament_index_cpp"));
        libList.insert(libList.begin() + 8, std::string("rcl_logging_interface"));
        libList.insert(libList.end(), std::string("lifecycle_msgs__rosidl_generator_c"));
        libList.insert(libList.end(), std::string("lifecycle_msgs__rosidl_typesupport_c"));
        libList.insert(libList.end(), std::string("rcl_lifecycle"));
    }

    isaacsim::core::includes::MultiLibraryLoader backupLibLoader;
    std::string path = "";
    if (argc == 2)
    {
        path = argv[1];
    }
    for (std::string lib : libList)
    {
        if (backupLibLoader.loadLibrary(lib, path).get()->loadedLibrary == nullptr)
        {
            exit(EXIT_FAILURE);
        }
    }
    printf("Checking to see if RMW can be loaded:\n");
    auto rcl = std::make_shared<isaacsim::core::includes::LibraryLoader>("rcl", path, false);
    auto rcutils = std::make_shared<isaacsim::core::includes::LibraryLoader>("rcutils", path, false);

    rcl_init_options_t initOptions = rcl->callSymbolWithArg<rcl_init_options_t>("rcl_get_zero_initialized_init_options");
    auto allocator = rcutils->callSymbolWithArg<rcutils_allocator_t>("rcutils_get_default_allocator");
    rcl_ret_t rc = rcl->callSymbolWithArg<rcl_ret_t>("rcl_init_options_init", &initOptions, allocator);
    if (rc != RCL_RET_OK)
    {
        do
        {
            printf("%s\n", rcutils->callSymbolWithArg<rcutils_error_string_t>("rcutils_get_error_string").str);
            rcutils->callSymbolWithArg<rcutils_error_string_t>("rcutils_reset_error");
        } while (false);
        printf("RMW was not loaded\n");
        exit(EXIT_FAILURE);
    }

    rcl->callSymbolWithArg<rcl_init_options_t>("rcl_init_options_fini", &initOptions);

    return 0;
}
