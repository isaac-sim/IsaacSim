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

#include "isaacsim/common/logging/Logging.hpp"

#include <pxr/base/plug/registry.h>

#if defined(_WIN32)
#    include <windows.h>
#else
#    include <dlfcn.h>
#endif

#include <filesystem>
#include <string>
#if defined(_WIN32)
#    include <vector>
#endif

namespace
{

namespace logging = isaacsim::common::logging;

#if !defined(_WIN32)
std::filesystem::path getSharedLibraryDirectory()
{
    Dl_info info;
    if (dladdr(reinterpret_cast<void*>(&getSharedLibraryDirectory), &info) && info.dli_fname)
    {
        return std::filesystem::path(info.dli_fname).parent_path();
    }
    return {};
}
#else
std::filesystem::path getSharedLibraryDirectory()
{
    HMODULE hModule = nullptr;
    if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           reinterpret_cast<LPCWSTR>(&getSharedLibraryDirectory), &hModule))
    {
        wchar_t path[MAX_PATH];
        if (GetModuleFileNameW(hModule, path, MAX_PATH))
        {
            return std::filesystem::path(path).parent_path();
        }
    }
    return {};
}
#endif

#if defined(_WIN32)
bool registerPluginRoot(const logging::Logger& logger, const std::filesystem::path& searchPath, const char* description)
{
    std::error_code errorCode;
    if (!std::filesystem::is_regular_file(searchPath / "plugInfo.json", errorCode))
    {
        return false;
    }

    const std::string pluginPath = searchPath.lexically_normal().string();
    ISAACSIM_REPORT(logger, "Registering {}: {}", description, pluginPath);
    PXR_NS::PlugRegistry::GetInstance().RegisterPlugins({ pluginPath });
    return true;
}
#endif

struct UsdSchemaRegistration
{
    UsdSchemaRegistration()
    {
        const logging::Logger logger("isaacsim.foundation.usd.openusd");
        const std::filesystem::path sharedLibraryDirectory = getSharedLibraryDirectory();

#if defined(_WIN32)
        // OpenUSD normally discovers these manifests through its package
        // layout. Native Windows tests and installed executables copy the DLLs
        // beside the executable, so register the staged core manifest before
        // anything asks Ar for its default resolver.
        const std::vector<std::filesystem::path> openUsdSearchPaths = {
            sharedLibraryDirectory / "usd",
            sharedLibraryDirectory / ".." / "lib" / "usd",
        };
        bool registeredOpenUsd = false;
        for (const std::filesystem::path& searchPath : openUsdSearchPaths)
        {
            if (registerPluginRoot(logger, searchPath, "OpenUSD plugins"))
            {
                registeredOpenUsd = true;
                break;
            }
        }
        if (!registeredOpenUsd)
        {
            ISAACSIM_LOG_WARNING(logger, "No OpenUSD plugin manifest found beside {}", sharedLibraryDirectory.string());
        }
#endif

        const std::filesystem::path searchPath = sharedLibraryDirectory / "third-party-schemas";
        size_t registeredSchemas = 0;
        std::error_code errorCode;
        for (const auto& entry : std::filesystem::recursive_directory_iterator(searchPath, errorCode))
        {
            if (!entry.is_directory())
            {
                continue;
            }
            if (!std::filesystem::exists(entry.path() / "plugInfo.json"))
            {
                continue;
            }
            const std::string schemaPath = entry.path().string();
            ISAACSIM_REPORT(logger, "Registering third-party schema: {}", schemaPath);
            PXR_NS::PlugRegistry::GetInstance().RegisterPlugins({ schemaPath });
            registeredSchemas++;
        }
        if (!registeredSchemas)
        {
            ISAACSIM_LOG_WARNING(logger, "No third-party schemas found in {}", searchPath.string());
        }
    }
};

static UsdSchemaRegistration s_usdSchemaRegistration;

} // namespace
