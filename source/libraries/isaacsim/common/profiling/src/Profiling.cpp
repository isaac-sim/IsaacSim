// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "isaacsim/common/profiling/Profiling.hpp"

#include "isaacsim/common/profiling/ProfilingHost.h"

#include <carb/profiler/IProfiler.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace isaacsim
{
namespace common
{
namespace profiling
{
namespace details
{

class CarboniteHost;

class ProfilerLease
{
public:
    ProfilerLease() noexcept = default;
    ProfilerLease(CarboniteHost* owner, carb::profiler::IProfiler* profiler) noexcept
        : m_owner(owner), m_profiler(profiler)
    {
    }
    ~ProfilerLease() noexcept;
    ProfilerLease(ProfilerLease&& other) noexcept
        : m_owner(std::exchange(other.m_owner, nullptr)), m_profiler(std::exchange(other.m_profiler, nullptr))
    {
    }
    ProfilerLease& operator=(ProfilerLease&&) = delete;
    ProfilerLease(const ProfilerLease&) = delete;
    ProfilerLease& operator=(const ProfilerLease&) = delete;

    carb::profiler::IProfiler* get() const noexcept
    {
        return m_profiler;
    }

private:
    CarboniteHost* m_owner{ nullptr };
    carb::profiler::IProfiler* m_profiler{ nullptr };
};

struct ZoneRecord
{
    carb::profiler::IProfiler* profiler{ nullptr };
    std::uint64_t mask{ 0 };
    carb::profiler::ZoneId backendZone{ carb::profiler::kNoZoneId };
};

class CarboniteHost
{
public:
    static CarboniteHost& get() noexcept
    {
        static CarboniteHost s_host;
        return s_host;
    }

    std::optional<ProfilerLease> acquire() noexcept
    {
        if (m_profiler.load(std::memory_order_acquire) == nullptr)
        {
            return std::nullopt;
        }
        const std::lock_guard<std::mutex> lock(m_mutex);
        carb::profiler::IProfiler* profiler = m_profiler.load(std::memory_order_relaxed);
        if (profiler == nullptr || m_stopping)
        {
            return std::nullopt;
        }
        ++m_activeCalls;
        return ProfilerLease(this, profiler);
    }

    void release() noexcept
    {
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (m_activeCalls != 0)
        {
            --m_activeCalls;
        }
        if (m_stopping && m_activeCalls == 0)
        {
            m_condition.notify_all();
        }
    }

    IsaacSimCommonProfilingZone storeZone(const ProfilerLease& lease,
                                          std::uint64_t mask,
                                          carb::profiler::ZoneId backendZone) noexcept
    {
        if (backendZone == carb::profiler::kNoZoneId || backendZone == carb::profiler::kUnknownZoneId)
        {
            return 0;
        }
        try
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_stopping && lease.get() == m_profiler.load(std::memory_order_relaxed))
            {
                IsaacSimCommonProfilingZone identifier = m_nextZone.fetch_add(1, std::memory_order_relaxed);
                if (identifier == 0)
                {
                    identifier = m_nextZone.fetch_add(1, std::memory_order_relaxed);
                }
                if (m_zones.emplace(identifier, ZoneRecord{ lease.get(), mask, backendZone }).second)
                {
                    return identifier;
                }
            }
        }
        catch (...)
        {
        }
        try
        {
            lease.get()->endEx(mask, backendZone);
        }
        catch (...)
        {
        }
        return 0;
    }

    void endZone(IsaacSimCommonProfilingZone identifier) noexcept
    {
        ZoneRecord record;
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            const auto found = m_zones.find(identifier);
            if (found == m_zones.end())
            {
                return;
            }
            record = found->second;
            m_zones.erase(found);
            ++m_activeCalls;
        }
        try
        {
            record.profiler->endEx(record.mask, record.backendZone);
        }
        catch (...)
        {
        }
        release();
    }

    IsaacSimCommonProfilingHostResult attach(void* carboniteProfiler, IsaacSimCommonProfilingHostToken* token) noexcept
    {
        if (carboniteProfiler == nullptr || token == nullptr)
        {
            return ISAACSIM_COMMON_PROFILING_HOST_INVALID_ARGUMENT;
        }
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (m_profiler.load(std::memory_order_relaxed) != nullptr || m_stopping)
        {
            return ISAACSIM_COMMON_PROFILING_HOST_ALREADY_ATTACHED;
        }
        m_token = m_nextToken.fetch_add(1, std::memory_order_relaxed);
        if (m_token == 0)
        {
            m_token = m_nextToken.fetch_add(1, std::memory_order_relaxed);
        }
        m_profiler.store(static_cast<carb::profiler::IProfiler*>(carboniteProfiler), std::memory_order_release);
        *token = m_token;
        return ISAACSIM_COMMON_PROFILING_HOST_SUCCESS;
    }

    bool hasProfiler() const noexcept
    {
        return m_profiler.load(std::memory_order_acquire) != nullptr;
    }

    IsaacSimCommonProfilingHostResult detach(IsaacSimCommonProfilingHostToken token) noexcept
    {
        if (token == 0)
        {
            return ISAACSIM_COMMON_PROFILING_HOST_INVALID_ARGUMENT;
        }
        std::map<IsaacSimCommonProfilingZone, ZoneRecord> zones;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_profiler.load(std::memory_order_relaxed) == nullptr || token != m_token)
            {
                return ISAACSIM_COMMON_PROFILING_HOST_NOT_OWNER;
            }
            m_stopping = true;
            m_profiler.store(nullptr, std::memory_order_release);
            m_condition.wait(lock, [this] { return m_activeCalls == 0; });
            zones.swap(m_zones);
        }
        bool failed = false;
        for (const auto& entry : zones)
        {
            try
            {
                entry.second.profiler->endEx(entry.second.mask, entry.second.backendZone);
            }
            catch (...)
            {
                failed = true;
            }
        }
        {
            const std::lock_guard<std::mutex> lock(m_mutex);
            m_token = 0;
            m_stopping = false;
        }
        return failed ? ISAACSIM_COMMON_PROFILING_HOST_UNAVAILABLE : ISAACSIM_COMMON_PROFILING_HOST_SUCCESS;
    }

private:
    std::mutex m_mutex;
    std::condition_variable m_condition;
    std::atomic<carb::profiler::IProfiler*> m_profiler{ nullptr };
    bool m_stopping{ false };
    std::size_t m_activeCalls{ 0 };
    IsaacSimCommonProfilingHostToken m_token{ 0 };
    std::map<IsaacSimCommonProfilingZone, ZoneRecord> m_zones;
    std::atomic<std::uint64_t> m_nextZone{ 1 };
    std::atomic<std::uint64_t> m_nextToken{ 1 };
};

ProfilerLease::~ProfilerLease() noexcept
{
    if (m_owner != nullptr)
    {
        m_owner->release();
    }
}

std::uint64_t getEffectiveMask(std::uint64_t mask) noexcept
{
    return mask == 0 ? carb::profiler::kCaptureMaskDefault : mask;
}

bool isMaskEnabled(carb::profiler::IProfiler* profiler, std::uint64_t mask) noexcept
{
    if (profiler == nullptr)
    {
        return false;
    }
    try
    {
        const std::uint64_t effectiveMask = getEffectiveMask(mask);
        return (effectiveMask & profiler->getCaptureMask()) == effectiveMask;
    }
    catch (...)
    {
        return false;
    }
}

carb::profiler::StaticStringType getSourceString(const char* value, bool dynamicLocations) noexcept
{
    if (value == nullptr || !dynamicLocations)
    {
        return carb::profiler::kInvalidStaticString;
    }
    return reinterpret_cast<carb::profiler::StaticStringType>(value);
}

bool isValidLocation(const IsaacSimCommonProfilingSourceLocation* location) noexcept
{
    return location == nullptr || location->structSize >= sizeof(*location);
}

template <typename Function>
void guardProfilingCall(Function&& function) noexcept
{
    try
    {
        function();
    }
    catch (...)
    {
    }
}

template <typename Result, typename Function>
Result guardProfilingCall(Function&& function, Result fallback) noexcept
{
    try
    {
        return function();
    }
    catch (...)
    {
        return fallback;
    }
}

} // namespace details

Zone::Zone(std::string_view name, Mask mask, SourceLocation location) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    try
    {
        const std::string ownedName(name);
        m_zone = isaacsimCommonProfilingBegin(mask, ownedName.c_str(), &location);
    }
    catch (...)
    {
    }
}

Zone::Zone(IsaacSimCommonProfilingZone zone) noexcept : m_zone(zone)
{
}

Zone::~Zone() noexcept
{
    close();
}

Zone::Zone(Zone&& other) noexcept : m_zone(std::exchange(other.m_zone, 0))
{
}

Zone& Zone::operator=(Zone&& other) noexcept
{
    if (this != &other)
    {
        close();
        m_zone = std::exchange(other.m_zone, 0);
    }
    return *this;
}

void Zone::close() noexcept
{
    if (m_zone != 0)
    {
        isaacsimCommonProfilingEnd(std::exchange(m_zone, 0));
    }
}

Zone::operator bool() const noexcept
{
    return m_zone != 0;
}

bool isEnabled(Mask mask) noexcept
{
    return isaacsimCommonProfilingIsEnabled(mask) != 0;
}

void frame(std::string_view name, Mask mask) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingFrame(mask, ownedName.c_str());
        });
}

void value(std::string_view name, std::int32_t selected, Mask mask) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingValueInt(mask, ownedName.c_str(), selected);
        });
}

void value(std::string_view name, float selected, Mask mask) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingValueFloat(mask, ownedName.c_str(), selected);
        });
}

void instant(std::string_view name, InstantType type, Mask mask, SourceLocation location) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingInstant(mask, static_cast<std::int32_t>(type), ownedName.c_str(), &location);
        });
}

void flow(bool begin, std::uint64_t identifier, std::string_view name, Mask mask, SourceLocation location) noexcept
{
    if (!isEnabled(mask))
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingFlow(mask, begin ? 1u : 0u, identifier, ownedName.c_str(), &location);
        });
}

void setThreadName(std::string_view name) noexcept
{
    if (!isEnabled())
    {
        return;
    }
    details::guardProfilingCall(
        [&]
        {
            const std::string ownedName(name);
            isaacsimCommonProfilingSetThreadName(ownedName.c_str());
        });
}

} // namespace profiling
} // namespace common
} // namespace isaacsim

extern "C"
{

    uint32_t isaacsimCommonProfilingIsEnabled(IsaacSimCommonProfilingMask mask)
    {
        return isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                return lease && isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask) ? 1u : 0u;
            },
            uint32_t{ 0 });
    }

    IsaacSimCommonProfilingZone isaacsimCommonProfilingBegin(IsaacSimCommonProfilingMask mask,
                                                             const char* name,
                                                             const IsaacSimCommonProfilingSourceLocation* location)
    {
        return isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                if (name == nullptr || name[0] == '\0' ||
                    !isaacsim::common::profiling::details::isValidLocation(location))
                {
                    return IsaacSimCommonProfilingZone{ 0 };
                }
                auto& host = isaacsim::common::profiling::details::CarboniteHost::get();
                auto lease = host.acquire();
                if (!lease || !isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    return IsaacSimCommonProfilingZone{ 0 };
                }
                carb::profiler::IProfiler* profiler = lease->get();
                const bool dynamicLocations = profiler->supportsDynamicSourceLocations();
                const char* function = location == nullptr ? nullptr : location->function;
                const char* file = location == nullptr ? nullptr : location->file;
                const int line = location == nullptr ? 0 : static_cast<int>(location->line);
                const std::uint64_t effectiveMask = isaacsim::common::profiling::details::getEffectiveMask(mask);
                const auto backendZone = profiler->beginDynamic(
                    effectiveMask, isaacsim::common::profiling::details::getSourceString(function, dynamicLocations),
                    isaacsim::common::profiling::details::getSourceString(file, dynamicLocations), line, "%s", name);
                return host.storeZone(*lease, effectiveMask, backendZone);
            },
            IsaacSimCommonProfilingZone{ 0 });
    }

    void isaacsimCommonProfilingEnd(IsaacSimCommonProfilingZone zone)
    {
        if (zone != 0)
        {
            isaacsim::common::profiling::details::CarboniteHost::get().endZone(zone);
        }
    }

    void isaacsimCommonProfilingFrame(IsaacSimCommonProfilingMask mask, const char* name)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                if (name == nullptr || name[0] == '\0')
                {
                    return;
                }
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (lease && isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    lease->get()->frameDynamic(isaacsim::common::profiling::details::getEffectiveMask(mask), "%s", name);
                }
            });
    }

    void isaacsimCommonProfilingValueInt(IsaacSimCommonProfilingMask mask, const char* name, int32_t value)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (name != nullptr && name[0] != '\0' && lease &&
                    isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    lease->get()->valueIntDynamic(
                        isaacsim::common::profiling::details::getEffectiveMask(mask), value, "%s", name);
                }
            });
    }

    void isaacsimCommonProfilingValueFloat(IsaacSimCommonProfilingMask mask, const char* name, float value)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (name != nullptr && name[0] != '\0' && lease &&
                    isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    lease->get()->valueFloatDynamic(
                        isaacsim::common::profiling::details::getEffectiveMask(mask), value, "%s", name);
                }
            });
    }

    void isaacsimCommonProfilingInstant(IsaacSimCommonProfilingMask mask,
                                        int32_t type,
                                        const char* name,
                                        const IsaacSimCommonProfilingSourceLocation* location)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                if (name == nullptr || name[0] == '\0' ||
                    !isaacsim::common::profiling::details::isValidLocation(location) ||
                    (type != ISAACSIM_COMMON_PROFILING_INSTANT_THREAD &&
                     type != ISAACSIM_COMMON_PROFILING_INSTANT_PROCESS))
                {
                    return;
                }
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (!lease || !isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    return;
                }
                carb::profiler::IProfiler* profiler = lease->get();
                const bool dynamicLocations = profiler->supportsDynamicSourceLocations();
                profiler->emitInstantDynamic(isaacsim::common::profiling::details::getEffectiveMask(mask),
                                             isaacsim::common::profiling::details::getSourceString(
                                                 location == nullptr ? nullptr : location->function, dynamicLocations),
                                             isaacsim::common::profiling::details::getSourceString(
                                                 location == nullptr ? nullptr : location->file, dynamicLocations),
                                             location == nullptr ? 0 : static_cast<int>(location->line),
                                             type == ISAACSIM_COMMON_PROFILING_INSTANT_PROCESS ?
                                                 carb::profiler::InstantType::Process :
                                                 carb::profiler::InstantType::Thread,
                                             "%s", name);
            });
    }

    void isaacsimCommonProfilingFlow(IsaacSimCommonProfilingMask mask,
                                     uint32_t begin,
                                     uint64_t identifier,
                                     const char* name,
                                     const IsaacSimCommonProfilingSourceLocation* location)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                const bool validName = begin == 0 || (name != nullptr && name[0] != '\0');
                if (begin > 1 || identifier == 0 || !validName ||
                    !isaacsim::common::profiling::details::isValidLocation(location))
                {
                    return;
                }
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (!lease || !isaacsim::common::profiling::details::isMaskEnabled(lease->get(), mask))
                {
                    return;
                }
                carb::profiler::IProfiler* profiler = lease->get();
                const bool dynamicLocations = profiler->supportsDynamicSourceLocations();
                const std::uint64_t effectiveMask = isaacsim::common::profiling::details::getEffectiveMask(mask);
                const auto function = isaacsim::common::profiling::details::getSourceString(
                    location == nullptr ? nullptr : location->function, dynamicLocations);
                const auto file = isaacsim::common::profiling::details::getSourceString(
                    location == nullptr ? nullptr : location->file, dynamicLocations);
                const int line = location == nullptr ? 0 : static_cast<int>(location->line);
                if (begin != 0)
                {
                    profiler->emitFlowDynamic(
                        effectiveMask, function, file, line, carb::profiler::FlowType::Begin, identifier, "%s", name);
                }
                else
                {
                    profiler->emitFlowDynamic(
                        effectiveMask, function, file, line, carb::profiler::FlowType::End, identifier, nullptr);
                }
            });
    }

    void isaacsimCommonProfilingSetThreadName(const char* name)
    {
        isaacsim::common::profiling::details::guardProfilingCall(
            [&]
            {
                auto lease = isaacsim::common::profiling::details::CarboniteHost::get().acquire();
                if (name != nullptr && name[0] != '\0' && lease)
                {
                    lease->get()->nameThreadDynamic(0, "%s", name);
                }
            });
    }

    IsaacSimCommonProfilingHostResult isaacsimCommonProfilingAttachCarboniteHost(void* carboniteProfiler,
                                                                                 IsaacSimCommonProfilingHostToken* token)
    {
        return isaacsim::common::profiling::details::guardProfilingCall(
            [&] { return isaacsim::common::profiling::details::CarboniteHost::get().attach(carboniteProfiler, token); },
            ISAACSIM_COMMON_PROFILING_HOST_UNAVAILABLE);
    }

    IsaacSimCommonProfilingHostResult isaacsimCommonProfilingDetachCarboniteHost(IsaacSimCommonProfilingHostToken token)
    {
        return isaacsim::common::profiling::details::guardProfilingCall(
            [&] { return isaacsim::common::profiling::details::CarboniteHost::get().detach(token); },
            ISAACSIM_COMMON_PROFILING_HOST_UNAVAILABLE);
    }

    uint32_t isaacsimCommonProfilingHasCarboniteHost(void)
    {
        return isaacsim::common::profiling::details::CarboniteHost::get().hasProfiler() ? 1u : 0u;
    }
}
