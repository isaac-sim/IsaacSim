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

#include <carb/ClientUtils.h>
#include <carb/StartupUtils.h>
#include <carb/profiler/IProfiler.h>

#include <isaacsim/common/profiling/ProfilingHost.h>

#include <cstdint>
#include <mutex>
#include <string>

CARB_STATIC_BINARY_GLOBALS("isaacsim.common.profiling.carbonite_host")

#if CARB_PLATFORM_WINDOWS
#    define ISAACSIM_COMMON_PROFILING_HOST_EXPORT __declspec(dllexport)
#else
#    define ISAACSIM_COMMON_PROFILING_HOST_EXPORT __attribute__((visibility("default")))
#endif

namespace
{

enum class Result
{
    eSuccess = 0,
    eInvalidArgument = 1,
    eAlreadyStarted = 2,
    eNotStarted = 3,
    eFrameworkUnavailable = 4,
    eProfilerUnavailable = 5,
    eProfilingHostUnavailable = 6,
};

class CarboniteProfilerSession
{
public:
    static CarboniteProfilerSession& get() noexcept
    {
        static CarboniteProfilerSession s_session;
        return s_session;
    }

    ~CarboniteProfilerSession() noexcept
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        _stopUnlocked(false);
    }

    Result start(const char* pluginDirectory) noexcept
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (pluginDirectory == nullptr || pluginDirectory[0] == '\0')
        {
            return _fail(Result::eInvalidArgument, "The Carbonite plugin directory is required");
        }
        if (m_framework != nullptr || m_profiler != nullptr || m_hostToken != 0)
        {
            return _fail(Result::eAlreadyStarted, "Standalone profiling is already running");
        }
        if (isaacsimCommonProfilingHasCarboniteHost() != 0)
        {
            return _fail(Result::eProfilingHostUnavailable, "Another profiling host is already attached");
        }

        try
        {
            OmniCoreStartArgs arguments{};
            arguments.flags = fStartFlagDisableIStructuredLog;
            m_framework = carb::acquireFrameworkAndRegisterBuiltins(&arguments);
            if (m_framework == nullptr)
            {
                return _fail(Result::eFrameworkUnavailable, "Could not initialize the Carbonite framework");
            }

            const char* searchPaths[] = { pluginDirectory };
            carb::loadPluginsFromPattern("*carb.profiler-nvtx.plugin*", searchPaths, 1);
            m_profiler = m_framework->tryAcquireInterface<carb::profiler::IProfiler>("carb.profiler-nvtx.plugin");
            if (m_profiler == nullptr)
            {
                _releaseFramework();
                return _fail(Result::eProfilerUnavailable, "Could not load the Carbonite NVTX profiler plugin");
            }

            m_profiler->startup();
            const IsaacSimCommonProfilingHostResult result =
                isaacsimCommonProfilingAttachCarboniteHost(m_profiler, &m_hostToken);
            if (result != ISAACSIM_COMMON_PROFILING_HOST_SUCCESS)
            {
                _shutdownProfiler();
                _releaseFramework();
                return _fail(Result::eProfilingHostUnavailable, "Another profiling host is already attached");
            }
            m_lastError.clear();
            return Result::eSuccess;
        }
        catch (...)
        {
            _shutdownProfiler();
            _releaseFramework();
            return _fail(Result::eFrameworkUnavailable, "Carbonite NVTX profiler startup failed");
        }
    }

    Result stop() noexcept
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        return _stopUnlocked(true);
    }

    const char* getLastError() const noexcept
    {
        static thread_local std::string s_error;
        try
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            s_error = m_lastError;
            return s_error.c_str();
        }
        catch (...)
        {
            return "Could not read the standalone profiling error";
        }
    }

private:
    Result _stopUnlocked(bool requireStarted) noexcept
    {
        if (m_framework == nullptr || m_profiler == nullptr || m_hostToken == 0)
        {
            if (requireStarted)
            {
                return _fail(Result::eNotStarted, "Standalone profiling is not running");
            }
            return Result::eSuccess;
        }

        Result result = Result::eSuccess;
        const IsaacSimCommonProfilingHostResult detachResult = isaacsimCommonProfilingDetachCarboniteHost(m_hostToken);
        m_hostToken = 0;
        if (detachResult != ISAACSIM_COMMON_PROFILING_HOST_SUCCESS)
        {
            result = _fail(Result::eProfilingHostUnavailable, "Could not detach the standalone profiling host");
        }
        if (!_shutdownProfiler() && result == Result::eSuccess)
        {
            result = _fail(Result::eFrameworkUnavailable, "Carbonite NVTX profiler shutdown failed");
        }
        _releaseFramework();
        if (result == Result::eSuccess)
        {
            m_lastError.clear();
        }
        return result;
    }

    Result _fail(Result result, const char* message) noexcept
    {
        try
        {
            m_lastError = message;
        }
        catch (...)
        {
        }
        return result;
    }

    bool _shutdownProfiler() noexcept
    {
        bool succeeded = true;
        if (m_profiler != nullptr)
        {
            try
            {
                m_profiler->shutdown();
            }
            catch (...)
            {
                succeeded = false;
            }
            m_profiler = nullptr;
        }
        return succeeded;
    }

    void _releaseFramework() noexcept
    {
        m_hostToken = 0;
        if (m_framework != nullptr)
        {
            try
            {
                carb::releaseFrameworkAndDeregisterBuiltins();
            }
            catch (...)
            {
            }
            m_framework = nullptr;
        }
    }

    mutable std::mutex m_mutex;
    carb::Framework* m_framework{ nullptr };
    carb::profiler::IProfiler* m_profiler{ nullptr };
    IsaacSimCommonProfilingHostToken m_hostToken{ 0 };
    std::string m_lastError;
};

} // namespace

extern "C"
{

    ISAACSIM_COMMON_PROFILING_HOST_EXPORT int32_t isaacsimCommonProfilingCarboniteHostStart(const char* pluginDirectory)
    {
        return static_cast<int32_t>(CarboniteProfilerSession::get().start(pluginDirectory));
    }

    ISAACSIM_COMMON_PROFILING_HOST_EXPORT int32_t isaacsimCommonProfilingCarboniteHostStop()
    {
        return static_cast<int32_t>(CarboniteProfilerSession::get().stop());
    }

    ISAACSIM_COMMON_PROFILING_HOST_EXPORT const char* isaacsimCommonProfilingCarboniteHostGetLastError()
    {
        return CarboniteProfilerSession::get().getLastError();
    }
}
