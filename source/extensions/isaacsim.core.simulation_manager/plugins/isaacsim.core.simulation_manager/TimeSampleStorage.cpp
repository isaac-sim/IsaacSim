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

#include <carb/Interface.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <isaacsim/core/simulation_manager/TimeSampleStorage.h>
#include <omni/fabric/FabricTime.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{

TimeSampleStorage::TimeSampleStorage(omni::fabric::UsdStageId stageId)
    : m_head(0), m_size(0), m_iStageReaderWriter(nullptr), m_usdStageId(stageId)
{
    // Cache interface once in constructor for reuse
    m_iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();

    CARB_LOG_INFO("Initialized TimeSampleStorage with circular buffer capacity %zu for stage %" PRIu64,
                  s_kBufferCapacity, static_cast<uint64_t>(m_usdStageId.id));

    // Log interface availability
    if (!m_iStageReaderWriter)
    {
        CARB_LOG_WARN("IStageReaderWriter interface not available");
    }
}

omni::fabric::RationalTime TimeSampleStorage::getCurrentTime()
{
    using namespace omni::fabric;

    // Returns the rational time that storage entries should be keyed by. The chosen time-base
    // must match what the rendering pipeline publishes as rpFabricTime so that subsequent
    // getSimulationTimeAt() lookups against rpFabricTime resolve to the right sample.
    if (m_usdStageId.id != 0 && m_iStageReaderWriter)
    {
        StageReaderWriterId stageReaderWriterId = m_iStageReaderWriter->get(m_usdStageId);
        if (stageReaderWriterId.id != 0)
        {
            FabricTime frameTime{};

            // Time-base depends on whether multitick rendering is enabled.
            //
            // Multitick: simulation_manager writes the current sim time to the Fabric prim
            // /ExternalSimulationTime in onPhysicsStep, and the renderer consumes that value
            // to derive each frame's rational time (which then flows through to rpFabricTime
            // in the post-render synthdata graph). Storing samples keyed at the same rational
            // time means lookups by rpFabricTime hit an exact match.
            //
            // Non-multitick: the renderer ignores /ExternalSimulationTime and tags frames
            // with its own free-running Hydra frame counter (m_frameNumber * simPeriod).
            // To keep storage keys aligned with what rpFabricTime carries, we read the
            // StageReaderWriter's frame time directly. Lookups for past frames then resolve
            // via exact match / adjacent-sample interpolation in getSimulationTimeAt().
            carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
            const bool multiTickRateEnabled = iSettings && iSettings->getAsBool("/rtx/hydra/supportMultiTickRate");
            if (multiTickRateEnabled)
            {
                static const omni::fabric::Path externalTimePrim =
                    omni::fabric::Path::createImmortal("/ExternalSimulationTime");
                static const omni::fabric::Token timeAttrToken = omni::fabric::Token::createImmortal("omni:time");
                omni::fabric::ConstSpanWithTypeC arraySpan =
                    m_iStageReaderWriter->getAttributeRd(stageReaderWriterId, externalTimePrim, timeAttrToken);
                const double* externalSimTimeAttr = arraySpan.getTypedPointer<const double>();
                if (!externalSimTimeAttr)
                {
                    CARB_LOG_WARN(
                        "getCurrentTime: /ExternalSimulationTime.omni:time not readable from Fabric; "
                        "returning invalid time");
                    return kInvalidRationalTime;
                }
                frameTime = omni::fabric::FabricTime(*externalSimTimeAttr);
            }
            else
            {
                frameTime = m_iStageReaderWriter->getFrameTime(stageReaderWriterId);
            }

            // Check if we got a valid rational time
            if (frameTime.m_type == FabricTime::Type::Rational)
            {
                RationalTime rationalTime = frameTime.m_rep.rationalTime;
                return rationalTime;
            }
            else if (frameTime.m_type == FabricTime::Type::Double)
            {
                // Convert double to rational time
                double doubleTime = frameTime.m_rep.doubleTime;
                const uint64_t denominator = s_kMicrosecondPrecisionDenominator;
                const int64_t numerator = static_cast<int64_t>(doubleTime * denominator);
                return RationalTime(numerator, denominator);
            }
        }
    }

    CARB_LOG_WARN("getCurrentTime: StageReaderWriter not available or unable to get frame time");
    return kInvalidRationalTime;
}

bool TimeSampleStorage::storeSample(double simTime, double simTimeMonotonic, double systemTime)
{
    omni::fabric::RationalTime currentTime = getCurrentTime();

    if (currentTime == omni::fabric::kInvalidRationalTime)
    {
        CARB_LOG_WARN("Could not determine current time for sample writing");
        return false;
    }

    // Validate the current time
    if (!isValidTime(currentTime))
    {
        CARB_LOG_ERROR("storeSample: Invalid RationalTime - zero denominator");
        return false;
    }
    return storeSampleAt(currentTime, simTime, simTimeMonotonic, systemTime);
}

bool TimeSampleStorage::storeSampleAt(const omni::fabric::RationalTime& time,
                                      double simTime,
                                      double simTimeMonotonic,
                                      double systemTime)
{
    // No input validation needed - this private method is only called after validation by storeSample()

    std::unique_lock<std::shared_mutex> lock(m_mutex);

    // Check if time matches the latest (most recent) entry - only allow updating the latest entry
    // As time is monotonic, this is the only entry that can be updated
    // Normally this occurs when multiple physics steps are taken in a single frame
    if (m_size > 0)
    {
        size_t latestIdx = (m_head + s_kBufferCapacity - 1) % s_kBufferCapacity;
        if (m_buffer[latestIdx].valid && m_buffer[latestIdx].time == time)
        {
            CARB_LOG_VERBOSE("Updating latest entry at time %s, simTime %f, simTimeMonotonic %f, systemTime %f",
                             time.toString().c_str(), simTime, simTimeMonotonic, systemTime);
            // Update latest entry only
            m_buffer[latestIdx].data.simTime = simTime;
            m_buffer[latestIdx].data.simTimeMonotonic = simTimeMonotonic;
            m_buffer[latestIdx].data.systemTime = systemTime;
            return true;
        }
    }

    CARB_LOG_VERBOSE("Inserting new entry at time %s, simTime %f, simTimeMonotonic %f, systemTime %f",
                     time.toString().c_str(), simTime, simTimeMonotonic, systemTime);
    // Insert new entry
    m_buffer[m_head] = { time, { simTime, simTimeMonotonic, systemTime }, true };
    m_head = (m_head + 1) % s_kBufferCapacity;
    if (m_size < s_kBufferCapacity)
    {
        ++m_size;
    }

    return true;
}


std::optional<double> TimeSampleStorage::getSimulationTimeAt(const omni::fabric::RationalTime& time)
{
    // Validate input
    if (!isValidTime(time))
    {
        CARB_LOG_WARN("getSimulationTimeAt: Invalid RationalTime - zero denominator");
        return std::nullopt;
    }

    // Multitick path: simulation time was passed directly to the renderer via
    // /ExternalSimulationTime, so the rational time we are being asked about *is* the
    // simulation time at frame submission. Avoid the storage lookup and return it as-is.
    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool multiTickRateEnabled = iSettings && iSettings->getAsBool("/rtx/hydra/supportMultiTickRate");
    if (multiTickRateEnabled)
    {
        return double(time);
    }

    // Non-multitick path: the rational time comes from the renderer's free-running
    // Hydra frame counter, not from sim time. Resolve it against the per-frame samples
    // authored in storeSample()/getCurrentTime() (which on this path read the renderer's
    // own frame time via IStageReaderWriter::getFrameTime).
    std::shared_lock<std::shared_mutex> lock(m_mutex);

    auto exactMatch = findExactMatch(time);
    if (exactMatch.has_value())
    {
        return exactMatch->simTime;
    }

    auto adjacent = findAdjacentSamples(time);
    if (!adjacent.has_value())
    {
        CARB_LOG_INFO("No adjacent samples found for interpolation at time %s", time.toString().c_str());
        return std::nullopt;
    }

    auto interpolated = performInterpolation(time, adjacent->first, adjacent->second);
    return interpolated.simTime;
}

std::optional<double> TimeSampleStorage::getMonotonicSimulationTimeAt(const omni::fabric::RationalTime& time)
{
    // Validate input
    if (!isValidTime(time))
    {
        CARB_LOG_ERROR("getMonotonicSimulationTimeAt: Invalid RationalTime - zero denominator");
        return std::nullopt;
    }

    // Hold lock for entire operation to ensure thread safety
    std::shared_lock<std::shared_mutex> lock(m_mutex);

    // First, try to find an exact match
    auto exactMatch = findExactMatch(time);
    if (exactMatch.has_value())
    {
        return exactMatch->simTimeMonotonic;
    }

    // No exact match found, try interpolation
    auto adjacent = findAdjacentSamples(time);
    if (!adjacent.has_value())
    {
        CARB_LOG_INFO("No adjacent samples found for interpolation at time %s", time.toString().c_str());
        return std::nullopt;
    }

    auto interpolated = performInterpolation(time, adjacent->first, adjacent->second);
    return interpolated.simTimeMonotonic;
}

std::optional<double> TimeSampleStorage::getSystemTimeAt(const omni::fabric::RationalTime& time)
{
    // Validate input
    if (!isValidTime(time))
    {
        CARB_LOG_ERROR("getSystemTimeAt: Invalid RationalTime - zero denominator");
        return std::nullopt;
    }

    // Hold lock for entire operation to ensure thread safety
    std::shared_lock<std::shared_mutex> lock(m_mutex);

    // First, try to find an exact match
    auto exactMatch = findExactMatch(time);
    if (exactMatch.has_value())
    {
        return exactMatch->systemTime;
    }

    // No exact match found, try interpolation
    auto adjacent = findAdjacentSamples(time);
    if (!adjacent.has_value())
    {
        CARB_LOG_INFO("No adjacent samples found for interpolation at time %s", time.toString().c_str());
        return std::nullopt;
    }

    auto interpolated = performInterpolation(time, adjacent->first, adjacent->second);
    return interpolated.systemTime;
}


size_t TimeSampleStorage::getSampleCount() const
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_size;
}

void TimeSampleStorage::clear()
{
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    for (auto& entry : m_buffer)
    {
        entry.valid = false;
    }
    m_head = 0;
    m_size = 0;
}

void TimeSampleStorage::logStatistics() const
{
    std::shared_lock<std::shared_mutex> lock(m_mutex);

    printf("Simulation Time Temporal Storage Stats:\n");
    printf("  Tracked samples: %zu\n", m_size);
    printf("  Circular buffer capacity: %zu\n", s_kBufferCapacity);

    if (m_size > 0)
    {
        // Get all entries and sort them
        auto entries = getAllSamples();
        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.time < b.time; });

        auto earliest = entries.front().time;
        auto latest = entries.back().time;
        printf("  Time range: %s to %s\n", earliest.toString().c_str(), latest.toString().c_str());

        printf("  Detailed sample information:\n");

        // Print detailed info for each sample in sorted order
        for (const auto& entry : entries)
        {
            // Convert rational time to double
            double timeAsDouble = 0.0;
            if (entry.time.denominator != 0)
            {
                timeAsDouble = static_cast<double>(entry.time.numerator) / static_cast<double>(entry.time.denominator);
            }

            printf("    Time=%" PRId64 "/%" PRIu64 " (%.6f sec), SimTime=%.6f, SimTimeMonotonic=%.6f, SystemTime=%.6f\n",
                   entry.time.numerator, entry.time.denominator, timeAsDouble, entry.data.simTime,
                   entry.data.simTimeMonotonic, entry.data.systemTime);
        }
    }
}


bool TimeSampleStorage::isValidTime(const omni::fabric::RationalTime& time)
{
    // Check for zero denominator
    if (time.denominator == 0)
    {
        return false;
    }

    // Check for potential overflow when converting to double
    // This is a basic check - could be more sophisticated
    const int64_t maxSafeInteger = 9007199254740991LL; // 2^53 - 1
    if (std::abs(time.numerator) > maxSafeInteger)
    {


        CARB_LOG_WARN("RationalTime numerator may cause precision loss: %" PRId64, time.numerator);
    }

    return true;
}

std::optional<std::pair<omni::fabric::RationalTime, omni::fabric::RationalTime>> TimeSampleStorage::getSampleRange() const
{
    if (getSampleCount() == 0)
    {
        return std::nullopt;
    }

    auto entries = getAllSamples(); // Already in chronological order
    return std::make_pair(entries.front().time, entries.back().time);
}

// Private methods

TimeSampleStorage::TimeData TimeSampleStorage::performInterpolation(const omni::fabric::RationalTime& time,
                                                                    const Entry& before,
                                                                    const Entry& after)
{
    // If time exactly equals one of the bounds, return that value directly
    if (time == before.time)
    {
        return before.data;
    }
    if (time == after.time)
    {
        return after.data;
    }

    // Convert RationalTime to double for interpolation
    auto timeToDouble = [](const omni::fabric::RationalTime& t) -> double
    {
        if (t.denominator == 0)
        {
            return 0.0;
        }
        return static_cast<double>(t.numerator) / static_cast<double>(t.denominator);
    };

    double t = timeToDouble(time);
    double t0 = timeToDouble(before.time);
    double t1 = timeToDouble(after.time);

    if (t1 == t0) // NOLINT(clang-diagnostic-float-equal)
    {
        // Avoid division by zero - this shouldn't happen with proper adjacent samples
        CARB_LOG_WARN("performInterpolation: before and after times are equal (%s), returning before value",
                      before.time.toString().c_str());
        return before.data;
    }

    // Linear interpolation: v = v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    double alpha = (t - t0) / (t1 - t0);

    TimeData result;
    result.simTime = before.data.simTime + (after.data.simTime - before.data.simTime) * alpha;
    result.simTimeMonotonic =
        before.data.simTimeMonotonic + (after.data.simTimeMonotonic - before.data.simTimeMonotonic) * alpha;
    result.systemTime = before.data.systemTime + (after.data.systemTime - before.data.systemTime) * alpha;

    return result;
}


std::optional<TimeSampleStorage::TimeData> TimeSampleStorage::findExactMatch(const omni::fabric::RationalTime& time) const
{
    // Search from newest to oldest - assumes caller holds lock
    for (size_t i = 0; i < m_size; ++i)
    {
        size_t idx = (m_head + s_kBufferCapacity - 1 - i) % s_kBufferCapacity;
        if (m_buffer[idx].valid && m_buffer[idx].time == time)
        {
            return m_buffer[idx].data;
        }
    }
    return std::nullopt;
}

std::vector<TimeSampleStorage::Entry> TimeSampleStorage::getAllSamples() const
{
    // Assumes caller holds lock
    std::vector<Entry> entries;
    entries.reserve(m_size);

    for (size_t i = 0; i < m_size; ++i)
    {
        size_t idx = (m_head + s_kBufferCapacity - m_size + i) % s_kBufferCapacity;
        if (m_buffer[idx].valid)
        {
            entries.push_back(m_buffer[idx]);
        }
    }
    return entries;
}

std::optional<std::pair<TimeSampleStorage::Entry, TimeSampleStorage::Entry>> TimeSampleStorage::findAdjacentSamples(
    const omni::fabric::RationalTime& time) const
{
    // Get all entries from buffer - assumes caller holds lock
    auto entries = getAllSamples();

    if (entries.size() < 2)
    {
        return std::nullopt;
    }

    // Sort entries by time
    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.time < b.time; });

    // Find the position where time would be inserted
    auto it = std::lower_bound(entries.begin(), entries.end(), time,
                               [](const Entry& entry, const omni::fabric::RationalTime& t) { return entry.time < t; });

    if (it == entries.begin())
    {
        // Time is before all samples
        return std::nullopt;
    }
    else if (it == entries.end())
    {
        // Time is after all samples
        return std::nullopt;
    }
    else if (it->time == time)
    {
        // Found a time that's numerically equal but might have different fraction representation
        // Since we're looking for interpolation bounds, treat this as the "after" time
        // and use the previous sample as "before"
        if (it == entries.begin())
        {
            // Can't interpolate if equal time is the first sample
            return std::nullopt;
        }

        auto beforeEntry = *(it - 1);
        auto afterEntry = *it;

        return std::make_pair(beforeEntry, afterEntry);
    }
    else
    {
        // Found adjacent samples
        auto beforeEntry = *(it - 1);
        auto afterEntry = *it;

        return std::make_pair(beforeEntry, afterEntry);
    }
}

} // namespace simulation_manager
} // namespace core
} // namespace isaacsim
