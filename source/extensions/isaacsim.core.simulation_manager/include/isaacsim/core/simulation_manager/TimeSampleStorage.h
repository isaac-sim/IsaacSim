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

#pragma once

#include <omni/fabric/FabricTime.h>
#include <omni/fabric/core/RationalTime.h>
#include <omni/fabric/stage/interface/IStageReaderWriter.h>

#include <array>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace simulation_manager
{


/**
 * @class TimeSampleStorage
 * @brief Thread-safe circular buffer storage for simulation time data
 *
 * @details
 * Stores simulation time data at specific RationalTime timestamps using an integrated
 * circular buffer. The class maintains three time values per sample:
 * - simTime: The current simulation time
 * - simTimeMonotonic: Monotonic simulation time that doesn't reset
 * - systemTime: System clock time
 *
 * Key features:
 * - RationalTime precision for accurate time representation
 * - FSD-compatible time retrieval with automatic fallback to timeline
 * - Robust time matching that handles equivalent fractions (e.g., 604/30 equals 302/15)
 * - Linear interpolation for queries between stored samples
 * - Bounded memory usage with automatic FIFO eviction when buffer is full
 * - Direct memory access for optimal performance
 * - Thread-safe operations with single-writer, multiple-reader pattern
 *
 */
class TimeSampleStorage
{
public:
    /**
     * @brief Time data stored for each sample
     */
    struct TimeData
    {
        double simTime; ///< Simulation time in seconds (may reset on timeline stop).
        double simTimeMonotonic; ///< Monotonically increasing simulation time in seconds.
        double systemTime; ///< Wall-clock system time in seconds.
    };

    /**
     * @brief Entry in the circular buffer
     */
    struct Entry
    {
        omni::fabric::RationalTime time; ///< Rational time code for this sample.
        TimeData data; ///< Time data associated with this sample.
        bool valid = false; ///< Whether this entry contains valid data.
    };
    /**
     * @brief Constructor
     *
     * @param[in] stageId The USD stage ID for accessing stage frame time
     */
    TimeSampleStorage(omni::fabric::UsdStageId stageId);

    // Time source operations
    /**
     * @brief Get current frame time from StageReaderWriter
     *
     * @details
     * Uses StageReaderWriter's getFrameTime() to provide the correct frame time
     * that corresponds to when we are reading/writing time data. This ensures
     * temporal consistency between time sample storage and frame timing.
     *
     * @return Current rational time or kInvalidRationalTime if unavailable
     */
    omni::fabric::RationalTime getCurrentTime();

    // Sample storage operations
    /**
     * @brief Store simulation time data at the current timestamp the current timestamp is returned by getCurrentTime()
     *
     * @param[in] simTime The simulation time value
     * @param[in] simTimeMonotonic The monotonic simulation time value
     * @param[in] systemTime The system time value
     * @return true if successful, false if current time could not be determined
     */
    bool storeSample(double simTime, double simTimeMonotonic, double systemTime);

    // Time query operations
    /**
     * @brief Get simulation time at a specific timestamp
     *
     * @param[in] time The rational time to look up
     * @return The simulation time value (exact or interpolated), std::nullopt if no data available
     * @note First attempts exact match, then falls back to linear interpolation between adjacent samples
     */
    std::optional<double> getSimulationTimeAt(const omni::fabric::RationalTime& time);

    /**
     * @brief Get monotonic simulation time at a specific timestamp
     *
     * @param[in] time The rational time to look up
     * @return The monotonic simulation time value (exact or interpolated), std::nullopt if no data available
     * @note First attempts exact match, then falls back to linear interpolation between adjacent samples
     */
    std::optional<double> getMonotonicSimulationTimeAt(const omni::fabric::RationalTime& time);

    /**
     * @brief Get system time at a specific timestamp
     *
     * @param[in] time The rational time to look up
     * @return The system time value (exact or interpolated), std::nullopt if no data available
     * @note First attempts exact match, then falls back to linear interpolation between adjacent samples
     */
    std::optional<double> getSystemTimeAt(const omni::fabric::RationalTime& time);

    // Sample management operations
    /**
     * @brief Get all valid samples from the buffer
     *
     * @return Vector of all valid samples in chronological order
     * @note Thread-safe - acquires shared lock internally
     */
    std::vector<Entry> getAllSamples() const;

    /**
     * @brief Get count of stored samples
     * @return Number of samples with data
     */
    size_t getSampleCount() const;

    /**
     * @brief Get maximum buffer capacity
     * @return Maximum number of samples that can be stored in the buffer
     */
    static constexpr size_t getBufferCapacity()
    {
        return s_kBufferCapacity;
    }

    /**
     * @brief Get the time range of stored samples
     *
     * @return Pair of (earliest_time, latest_time) if samples exist, nullopt otherwise
     */
    std::optional<std::pair<omni::fabric::RationalTime, omni::fabric::RationalTime>> getSampleRange() const;

    /**
     * @brief Clear all stored samples
     */
    void clear();

    // Utility operations
    /**
     * @brief Log storage statistics for debugging
     *
     * @note This method is thread-safe and does not modify the storage
     */
    void logStatistics() const;

    /**
     * @brief Validate a RationalTime value
     *
     * @param[in] time The time to validate
     * @return true if the time is valid, false otherwise
     * @note Checks for zero denominator and potential overflow conditions
     */
    static bool isValidTime(const omni::fabric::RationalTime& time);

private:
    // Constants

    /**
     * @brief Buffer capacity
     *
     * @details
     * Set to 31 samples to balance memory usage with interpolation reliability.
     * At 60Hz simulation rate, this retains approximately 0.5 seconds of history.
     */
    static constexpr size_t s_kBufferCapacity = 31;

    /** @brief Denominator for converting double time to rational time (microsecond precision) */
    static constexpr uint64_t s_kMicrosecondPrecisionDenominator = 1000000;

    // Circular buffer members
    /** @brief Thread synchronization for buffer access */
    mutable std::shared_mutex m_mutex;

    /** @brief Fixed-size buffer for storing time entries */
    std::array<Entry, s_kBufferCapacity> m_buffer;

    /** @brief Next write position in the circular buffer */
    size_t m_head = 0;

    /** @brief Current number of valid entries in the buffer */
    size_t m_size = 0;


    /** @brief Cached stage reader/writer interface for fabric operations */
    omni::fabric::IStageReaderWriter* m_iStageReaderWriter;

    /** @brief USD Stage ID for accessing stage frame time */
    omni::fabric::UsdStageId m_usdStageId;

    /**
     * @brief Store simulation time data at a specific timestamp
     *
     * @param[in] time The rational time timestamp
     * @param[in] simTime The simulation time value
     * @param[in] simTimeMonotonic The monotonic simulation time value
     * @param[in] systemTime The system time value
     * @return true if successful
     */
    bool storeSampleAt(const omni::fabric::RationalTime& time, double simTime, double simTimeMonotonic, double systemTime);

    /**
     * @brief Find time data for exact match
     *
     * @param[in] time The rational time to look up
     * @return The time data if found, std::nullopt otherwise
     * @note Assumes caller holds appropriate lock
     */
    std::optional<TimeData> findExactMatch(const omni::fabric::RationalTime& time) const;

    /**
     * @brief Find the adjacent samples for interpolation
     *
     * @param[in] time The time to find adjacent samples for
     * @return Pair of (before_entry, after_entry) if both exist, nullopt otherwise
     * @note Assumes caller holds appropriate lock
     */
    std::optional<std::pair<Entry, Entry>> findAdjacentSamples(const omni::fabric::RationalTime& time) const;

    /**
     * @brief Perform linear interpolation between two time data entries
     *
     * @param[in] time The time to interpolate at
     * @param[in] before The entry before the query time
     * @param[in] after The entry after the query time
     * @return The interpolated time data
     */
    static TimeData performInterpolation(const omni::fabric::RationalTime& time, const Entry& before, const Entry& after);
};

} // namespace simulation_manager
} // namespace core
} // namespace isaacsim
