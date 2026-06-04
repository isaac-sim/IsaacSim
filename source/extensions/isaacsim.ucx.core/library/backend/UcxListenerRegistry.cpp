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

#include "isaacsim/ucx/core/UcxListenerRegistry.h"

#include <carb/logging/Log.h>

#include <linux/limits.h>
#include <ucxx/api.h>

#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <glob.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace isaacsim::ucx::core
{
std::unordered_map<uint16_t, std::shared_ptr<UCXListener>> UCXListenerRegistry::g_listeners;
std::mutex UCXListenerRegistry::g_registryMutex;

// Sets UCX_MODULE_DIR so the UCS module loader finds libuct_cuda.so.0 / libucm_cuda.so.0
// in the ucx/ subdirectory alongside this library, and pre-loads libucm_cuda.so.0's
// transitive CUDA dependency into the global symbol table so UCX's own dlopen succeeds
// without needing bin/ucx/ in LD_LIBRARY_PATH at process start.
// Must be called before ucxx::createContext() which triggers ucp_config_read().
static void ensureUcxModuleDir()
{
    Dl_info info{};
    if (!dladdr(reinterpret_cast<void*>(&UCXListenerRegistry::addListener), &info) || !info.dli_fname)
    {
        CARB_LOG_WARN("[isaacsim.ucx.core] dladdr failed — UCX_MODULE_DIR not set");
        return;
    }

    // Resolve to absolute path in case LD_LIBRARY_PATH gave a relative path.
    char resolved[PATH_MAX]{};
    const char* absPath = realpath(info.dli_fname, resolved) ? resolved : info.dli_fname;
    std::string libDir = absPath;
    size_t lastSlash = libDir.rfind('/');
    if (lastSlash != std::string::npos)
        libDir = libDir.substr(0, lastSlash);
    std::string moduleDir = libDir + "/ucx";

    // Set UCX_MODULE_DIR so the UCS module loader finds libuct_cuda.so.0 / libucm_cuda.so.0.
    if (!getenv("UCX_MODULE_DIR"))
    {
        setenv("UCX_MODULE_DIR", moduleDir.c_str(), 0);
        CARB_LOG_INFO("[isaacsim.ucx.core] UCX_MODULE_DIR set to: %s (exists=%s)", moduleDir.c_str(),
                      (access(moduleDir.c_str(), F_OK) == 0) ? "yes" : "NO");
    }

    // glibc caches LD_LIBRARY_PATH at process startup — setenv() cannot update the
    // internal path list used by dlopen(). Instead, pre-load libucm_cuda.so.0's
    // dependency using its full absolute path with RTLD_GLOBAL so it is already in
    // the global symbol table when UCX dlopen's libucm_cuda.so.0.
    static bool cudartPreloaded = false;
    if (!cudartPreloaded)
    {
        // Glob for libcudart-*.so.* to avoid hardcoding the hash embedded in the filename,
        // which changes across prebundle versions.
        std::string pattern = moduleDir + "/libcudart-*.so.*";
        glob_t globResult{};
        std::string cudartPath;
        if (glob(pattern.c_str(), GLOB_NOSORT, nullptr, &globResult) == 0 && globResult.gl_pathc > 0)
        {
            cudartPath = globResult.gl_pathv[0];
        }
        globfree(&globResult);

        void* handle = nullptr;
        if (!cudartPath.empty())
        {
            handle = dlopen(cudartPath.c_str(), RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);
            if (!handle)
                handle = dlopen(cudartPath.c_str(), RTLD_NOW | RTLD_GLOBAL);
        }
        if (handle)
        {
            CARB_LOG_INFO("[isaacsim.ucx.core] pre-loaded %s into global symbol table", cudartPath.c_str());
            cudartPreloaded = true;
        }
        if (cudartPath.empty())
        {
            CARB_LOG_WARN("[isaacsim.ucx.core] no libcudart found matching %s", pattern.c_str());
        }
        else if (!handle)
        {
            const char* err = dlerror();
            CARB_LOG_WARN("[isaacsim.ucx.core] could not pre-load %s: %s", cudartPath.c_str(), err ? err : "unknown");
        }
    }
}

std::shared_ptr<UCXListener> UCXListenerRegistry::addListener(uint16_t port)
{
    std::lock_guard<std::mutex> lock(g_registryMutex);
    ensureUcxModuleDir();

    // When port is 0, always create a new listener on an ephemeral port
    if (port == 0)
    {
        CARB_LOG_INFO("UCXListenerRegistry::addListener: creating new listener on ephemeral port");
        try
        {
            auto context =
                ucxx::createContext({ { "TLS", "cuda_copy,sm,self,tcp" } }, ucxx::Context::defaultFeatureFlags);
            if (!context)
            {
                throw std::runtime_error("UCXListenerRegistry: failed to create UCX context");
            }

            // Always create on ephemeral port (0)
            auto listener = std::make_shared<UCXListener>(context, 0);
            uint16_t actualPort = listener->getPort();
            g_listeners[actualPort] = listener;

            CARB_LOG_INFO("UCXListenerRegistry::addListener: listener created on port %u", actualPort);
            return listener;
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error(std::string("UCXListenerRegistry: failed to create listener - ") + e.what());
        }
    }

    // When port is non-zero, return existing listener or create new one
    // Warn about privileged ports (requires root/administrator)
    if (port < 1024)
    {
        CARB_LOG_WARN("Using privileged port %u (ports 1-1023 require elevated privileges)", port);
    }

    if (g_listeners.find(port) == g_listeners.end())
    {
        CARB_LOG_INFO("UCXListenerRegistry::addListener: port %u", port);
        try
        {
            auto context =
                ucxx::createContext({ { "TLS", "cuda_copy,sm,self,tcp" } }, ucxx::Context::defaultFeatureFlags);
            if (!context)
            {
                throw std::runtime_error("UCXListenerRegistry: failed to create UCX context");
            }

            auto listener = std::make_shared<UCXListener>(context, port);
            g_listeners[port] = listener;
            CARB_LOG_INFO("UCXListenerRegistry::addListener: listener created on port %u", port);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error(std::string("UCXListenerRegistry: failed to create listener - ") + e.what());
        }
    }
    return g_listeners[port];
}

void UCXListenerRegistry::removeListener(uint16_t port)
{
    std::shared_ptr<UCXListener> listener;
    {
        std::lock_guard<std::mutex> lock(g_registryMutex);
        auto it = g_listeners.find(port);
        if (it == g_listeners.end())
        {
            return;
        }

        listener = std::move(it->second);
        g_listeners.erase(it);
    }

    try
    {
        CARB_LOG_INFO("UCXListenerRegistry::removeListener: shutting down listener on port %u", port);
        listener->shutdown();
    }
    catch (const std::exception& e)
    {
        // Log error but continue with removal
        CARB_LOG_ERROR("Error shutting down listener on port %u: %s", port, e.what());
    }
}

bool UCXListenerRegistry::isListenerRegistered(uint16_t port)
{
    std::lock_guard<std::mutex> lock(g_registryMutex);
    return g_listeners.find(port) != g_listeners.end();
}

void UCXListenerRegistry::shutdown()
{
    std::lock_guard<std::mutex> lock(g_registryMutex);
    for (auto& [port, listener] : g_listeners)
    {
        try
        {
            if (listener)
            {
                listener->shutdown();
            }
        }
        catch (const std::exception& e)
        {
            // Log error but continue with shutdown
            CARB_LOG_ERROR("Error shutting down listener on port %u: %s", port, e.what());
        }
    }
    g_listeners.clear();
}

bool UCXListenerRegistry::tryRemoveListener(uint16_t port)
{
    std::shared_ptr<UCXListener> listener;
    {
        std::lock_guard<std::mutex> lock(g_registryMutex);
        auto it = g_listeners.find(port);
        if (it == g_listeners.end())
        {
            return false;
        }
        if (it->second.use_count() > 1)
        {
            // Other nodes still hold references
            return false;
        }
        CARB_LOG_INFO("UCXListenerRegistry::tryRemoveListener: removing listener on port %u", port);
        listener = std::move(it->second);
        g_listeners.erase(it);
    }

    // Shutdown outside the lock
    try
    {
        listener->shutdown();
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("Error shutting down listener on port %u: %s", port, e.what());
    }
    return true;
}

} // namespace isaacsim::ucx::core
