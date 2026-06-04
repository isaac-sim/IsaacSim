// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "RunLoopSynchronizer.h"

#include <carb/dictionary/IDictionary.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/TaskingUtils.h>

#include <omni/kit/IApp.h>

#include <chrono>
#include <math.h>

namespace omni
{
namespace kit
{

constexpr const uint64_t kProfilerMask = 1;
static constexpr char kPresentThreadEnabledSettingsPath[] = "/exts/omni.kit.renderer.core/present/enabled";
static constexpr char kVsyncEnabledSettingsPath[] = "/app/vsync";

/**
 * The myceil function takes a float input, x, and rounds it up to the next whole number
 * using a specified tolerance value (default 0.05). The function separates the integer and
 * fractional parts of the input using the modff function from the C standard library.
 * If the fractional part of the input is greater than the tolerance value, the integer
 * part is incremented by 1 and returned as the final result.
 */
static float myceil(float x, float tolerance = 0.05f)
{
    float intpart;
    // Separate the integer and fractional parts of the input
    modff(x, &intpart);
    // Check if the fractional part is greater than the tolerance
    if (x - intpart > tolerance)
    {
        intpart += 1;
    }
    // Return the incremented integer part as the final result
    return intpart;
}

/**
 * @brief Rounds a float down with tolerance adjustment
 * @details Similar to floor but with a tolerance threshold. If the fractional
 *          part is greater than (1 - tolerance), the value is rounded up instead.
 * @param[in] x The input value to round
 * @param[in] tolerance The tolerance threshold (default 0.05)
 * @return The rounded integer value as a float
 */
static float myfloor(float x, float tolerance = 0.05f)
{
    float intpart;
    // Separate the integer and fractional parts of the input
    modff(x, &intpart);
    // Check if the fractional part is greater than the tolerance
    if (x - intpart > 1.0f - tolerance)
    {
        intpart += 1;
    }
    // Return the incremented integer part as the final result
    return intpart;
}

/**
 * @brief Finds the closest fraction to B that is a multiple of A
 * @details Takes two float values A and B as input, and returns the closest
 *          fraction to B that is a multiple of A. Works with 2x, 3x, 4x
 *          as well as with x/2, x/3, x/4.
 * @param[in] A The base value to find multiples of
 * @param[in] B The target value to approximate
 * @return The closest fraction to B that is a multiple of A
 */
static float findClosestFraction(float A, float B)
{
    float fraction = A / B;
    if (fraction >= 1)
    {
        return A / myfloor(fraction);
    }

    return A * myceil(B / A);
}

/**
 * @brief Returns max from last several numbers
 */
class SlidingMaximum
{
public:
    /**
     * The keep function updates m_array and m_sum based on the new duration
     * passed in. Then it calculates the average time of a frame, and searches
     * for the max element in the array, skipping up to m_outlierCount frames
     * that are more than m_toleranceFactor times the average.
     *
     * @param ns The duration time of a frame in nanoseconds.
     * @param outlierCount number of outliers to be ignored when calculating
     *                     the max.
     * @param toleranceFactor threshold factor to determine if a value is an
     *                        outlier.
     * @param count number of entries to be considered when doing the
     *              calculations.
     * @return   The maximum duration time of a frame considering the latest
     *           m_count frames and excluding the duration times that are much
     *           greater than the average.
     */
    float keep(float ns, size_t count, size_t outlierCount, float toleranceFactor)
    {
        // If count changed, resize the array and initialize the variables.
        if (count != m_array.size())
        {
            m_array.resize(count);
            std::fill(m_array.begin(), m_array.end(), 0.0f);
            m_index = 0;
            m_sum = 0.0f;
            m_validCount = 0;
        }

        if (count == 0)
        {
            return ns;
        }

        m_sum = m_sum - m_array[m_index] + ns;
        m_array[m_index] = ns;
        m_index = (m_index + 1) % m_array.size();

        if (m_validCount < m_array.size())
        {
            m_validCount++;
        }

        m_average = m_sum / m_validCount;

        // Searching the max element, excluding outliers.
        float maxElement = 0.0f;
        size_t outliers = 0;
        for (size_t i = 0; i < m_validCount; i++)
        {
            float val = m_array[i];

            if (val > m_average * toleranceFactor && outliers < outlierCount)
            {
                outliers++;
            }
            else if (val > maxElement)
            {
                maxElement = val;
            }
        }

        return maxElement;
    }

private:
    // The array storing the latest m_count frames' durations.
    std::vector<float> m_array;
    // The index where to put the next duration time in the m_array.
    size_t m_index = 0;
    // The sum of valid items in m_array.
    float m_sum = 0.0f;
    // The average of valid items in m_array.
    float m_average = 0.0f;
    // The count of valid items currently in m_array.
    size_t m_validCount = 0;
};

/**
 * @brief The gate class is a simple synchronization mechanism that allows one
 * thread to wait for another thread to "open" the gate. This can be used in
 * cases where a thread needs to wait for some condition to be met before it can
 * continue execution.
 *
 * Here are some example use cases for the Gate class:
 *
 * A producer thread that generates data that is consumed by a consumer thread.
 * The consumer thread can wait on the gate until the producer has finished
 * producing the data.
 *
 * A thread that needs to wait for a user input before it can continue. The
 * input can be provided by another thread or process, which can "open" the gate
 * once the input is received.
 *
 * A thread that needs to wait for an external event, such as a network request
 * completing or a timer expiring. The event handler can "open" the gate once
 * the event has occurred.
 *
 */
class IRendererRunLoopGate
{
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration = Clock::duration;

    /**
     * @brief Constructor for the Gate class. Initializes the `m_open` flag to
     * `false`.
     */
    IRendererRunLoopGate() : m_open(false)
    {
    }

    /**
     * @brief Notify the waiting thread that the gate has opened.
     * This function acquires a lock on the mutex before setting the `m_open`
     * flag to `true` and notifying the waiting thread.
     */
    void notify()
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(m_mtx);
        m_open.store(true);
        m_cv.notify_one();
    }

    /**
     * @brief Wait for the gate to open.
     *
     * This function acquires a lock on the mutex and waits on the condition
     * variable until the `m_open` flag is set to `true`. It then sets the
     * `m_open` flag back to `false` before returning.
     */
    void wait()
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(m_mtx);

        m_open.store(false);

        while (!m_open.load())
        {
            m_cv.wait(m_mtx);
        }

        m_open.store(false);
    }

    /**
     * @brief Blocked during relTimeMs, or until notified
     *
     * Returns false if unblocked by timeout
     */
    bool waitUntil(TimePoint wakeUpTime)
    {
        // store the state of the event
        m_open.store(false);

        TimePoint now = Clock::now();
        if (wakeUpTime > now)
        {
            // wait using sleep with a lock
            CARB_PROFILE_ZONE(
                kProfilerMask, "Gate::waitFor sleep %.2fms",
                static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(wakeUpTime - now).count()) *
                    1e-6f);
            std::unique_lock<carb::tasking::MutexWrapper> lock(m_mtx);
            m_cv.wait_until(m_mtx, wakeUpTime, [this]() { return m_open.load(std::memory_order_relaxed); });
        }

        // return the previous state of the event and set it to false
        return m_open.exchange(false);
    }

private:
    // Mutex for synchronizing access to the `m_open` flag.
    carb::tasking::MutexWrapper m_mtx;
    // Condition variable for waiting on the gate to open.
    carb::tasking::ConditionVariableWrapper m_cv;
    // Flag indicating whether the gate is open or closed.
    std::atomic_bool m_open;
};

RunLoopSynchronizer::RunLoopSynchronizer()
    : m_gate(std::make_unique<IRendererRunLoopGate>()), m_sliding(std::make_unique<SlidingMaximum>())
{
    TimePoint now = Clock::now();
    m_presentStartTime = now;
    m_presentCheckpointTime = now;

    m_presentDuration = std::chrono::duration_cast<Duration>(std::chrono::nanoseconds(0));

    // Initial value. Will be synced at first run.
    this->setTargetFPS(120.0f);

    {
        CARB_PROFILE_ZONE(kProfilerMask, "Startup decoupled present runloop");

        // Create or remove present thread when the setting is enables or disabled.
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        m_presentThreadEnabledSubscription = settings->subscribeToNodeChangeEvents(
            kPresentThreadEnabledSettingsPath,
            [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType changeEventType, void* userData)
            {
                if (changeEventType == carb::dictionary::ChangeEventType::eChanged)
                {
                    // Create or remove present thread.
                    reinterpret_cast<RunLoopSynchronizer*>(userData)->_setupPresentThread();
                }
            },
            this);

        this->_setupPresentThread();
    }
}

RunLoopSynchronizer::~RunLoopSynchronizer()
{
    if (m_presentThreadEnabledSubscription)
    {
        carb::getCachedInterface<carb::settings::ISettings>()->unsubscribeToChangeEvents(
            m_presentThreadEnabledSubscription);
        m_presentThreadEnabledSubscription = nullptr;
    }
}

void RunLoopSynchronizer::presentPreNotify()
{
    CARB_PROFILE_ZONE(kProfilerMask, "RunLoopSynchronizer::presentPreNotify");

    {
        TimePoint now = Clock::now();

        std::unique_lock<carb::tasking::MutexWrapper> lock(m_presentMutex);

        m_presentDuration = now - m_presentStartTime;
        m_presentStartTime = now;
    }
}

/**
 * @brief Saves the present time and duration and notifies the condition
 * variable in wait that it has to wake up.
 */
void RunLoopSynchronizer::presentPostNotify()
{
    CARB_PROFILE_ZONE(kProfilerMask, "RunLoopSynchronizer::presentPostNotify");

    // Wake up m_gate->waitUntil
    m_gate->notify();

    {
        TimePoint now = Clock::now();

        std::unique_lock<carb::tasking::MutexWrapper> lock(m_presentMutex);
        // All the threads will be synced to this time point.
        m_presentCheckpointTime = now;
    }
}

/**
 * The wait function uses the high resolution clock to wait for the
 * desired frame duration. It starts by obtaining the current time and
 * determining if waiting is necessary and for how long.
 *
 * It starts by obtaining the current time (now) and determine if waiting is
 * necessary by comparing the current time with the time to wake up, then
 * the function uses a number of duration values, presentDurationNs,
 * targetDurationNs, desireableDurationNs, and durationToAchieveNextFrameNs,
 * to compute the time point (timeToWakeUp) that the function should wake
 * up.
 *
 * Finally, if the current time is less than the timeToWakeUp, it uses the
 * waitFor function to wait for the remaining duration and achieve the
 * desired frame duration.
 */
void RunLoopSynchronizer::wait(float alreadyPassedNs,
                               size_t slidingMaximumCount,
                               size_t slidingMaximumOutlierCount,
                               float slidingMaximumToleranceFactor)
{
    TimePoint now = Clock::now();

    // Pick the maximum from the past durations
    float slidingAlreadyPassedNs =
        m_sliding->keep(alreadyPassedNs, slidingMaximumCount, slidingMaximumOutlierCount, slidingMaximumToleranceFactor);

    // Determine if we need to wait and how long we need to wait
    // TODO: Here is a lot of conversions, we need to rewrite it to have all
    // the computations in TimePoint type
    Duration presentDuration;
    TimePoint presentCheckpointTime;
    float presentDurationNs;
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(m_presentMutex);
        presentDuration = m_presentDuration;
        presentCheckpointTime = m_presentCheckpointTime;
        presentDurationNs =
            static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(presentDuration).count());
    }

    if (slidingAlreadyPassedNs > presentDurationNs)
    {
        // Shortcut: just wait for the next present frame. Gate will wake us up
        CARB_PROFILE_ZONE(kProfilerMask,
                          "RunLoopSynchronizer::wait presentDurationNs:%.2fms, slidingAlreadyPassedNs:%.2fms",
                          presentDurationNs * 1e-6f, slidingAlreadyPassedNs * 1e-6f);

        // Sleep more than expected present duration and wake up with the gate
        TimePoint timeToWakeUp = now +
                                 std::chrono::duration_cast<Duration>(std::chrono::nanoseconds(
                                     static_cast<int64_t>(slidingAlreadyPassedNs - alreadyPassedNs))) +
                                 presentDuration;

        while (m_gate->waitUntil(timeToWakeUp))
        {
            // When woke up by gate check if we are close enough to the present
            // thread time point
            if (timeToWakeUp - Clock::now() < presentDuration)
            {
                break;
            }
        }
        return;
    }

    float targetDurationNs;
    {
        std::unique_lock<carb::tasking::MutexWrapper> lock(m_targetMutex);
        targetDurationNs =
            static_cast<float>(std::chrono::duration_cast<std::chrono::nanoseconds>(m_targetDuration).count());
    }

    // Find the closest fraction of presentDurationNs and targetDurationNs
    float desireableDurationNs =
        findClosestFraction(presentDurationNs, std::max(targetDurationNs, slidingAlreadyPassedNs));

    // Next duration to reach the target frame rate, rounded up to the nearest desireableDurationNs
    float durationToAchieveNextFrameNs =
        ceilf(static_cast<float>(
                  std::chrono::duration_cast<std::chrono::nanoseconds>(now - presentCheckpointTime).count()) /
              desireableDurationNs) *
        desireableDurationNs;

    // Time point to wake up
    TimePoint timeToWakeUp =
        presentCheckpointTime + std::chrono::nanoseconds(static_cast<int64_t>(durationToAchieveNextFrameNs));

    if (now < timeToWakeUp)
    {
        // Wait time
        uint32_t waitTimeNs =
            static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timeToWakeUp - now).count());

#if 0
        // Useful to debug
        CARB_PROFILE_ZONE(kProfilerMask,
                          "RunLoopSynchronizer::wait waitTimeNs:%.2fms"
                          " presentDurationNs:%.2fms"
                          " targetDurationNs:%.2fms"
                          " desireableDurationNs:%.2fms"
                          " durationToAchieveNextFrameNs:%.2fms"
                          " slidingAlreadyPassedNs:%.2fms",
                          (float)waitTimeNs * 1e-6f, presentDurationNs * 1e-6f, targetDurationNs * 1e-6f,
                          desireableDurationNs * 1e-6f, durationToAchieveNextFrameNs * 1e-6f,
                          slidingAlreadyPassedNs * 1e-6f);
#else
        CARB_PROFILE_ZONE(kProfilerMask, "RunLoopSynchronizer::wait waitTimeNs:%.2fms", (float)waitTimeNs * 1e-6f);
#endif

        while (m_gate->waitUntil(timeToWakeUp))
        {
            // When woke up by gate check if we are close enough to the present
            // thread time point
            if (timeToWakeUp - Clock::now() < presentDuration / 2)
            {
                break;
            }
        }
    }
}

void RunLoopSynchronizer::setTargetFPS(double fps)
{
    if (m_fps == fps)
    {
        return;
    }

    m_fps = fps;
    std::unique_lock<carb::tasking::MutexWrapper> lock(m_targetMutex);
    m_targetDuration = std::chrono::duration_cast<Duration>(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / fps)));
}

bool RunLoopSynchronizer::isActive() const
{
    return m_isActive;
}

void RunLoopSynchronizer::setActive(bool active)
{
    if (m_isActive == active)
    {
        return;
    }

    m_isActive = active;
    this->_setupPresentThread();
}

void RunLoopSynchronizer::_setupPresentThread()
{
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool isPresentThreadEnabled = settings->getAsBool(kPresentThreadEnabledSettingsPath);

    bool needPresentThreadUpdate = isPresentThreadEnabled && m_isActive;

    if ((m_presentPreSubscription || m_presentPostSubscription) && !needPresentThreadUpdate)
    {
        m_presentPreSubscription.reset();
        m_presentPostSubscription.reset();
    }
    else if ((!m_presentPreSubscription || !m_presentPostSubscription) && needPresentThreadUpdate)
    {

        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();

        if (!m_presentPreSubscription)
        {
            m_presentPreSubscription =
                ed->observeEvent(carb::RStringKey("omni.kit.loop-default"), carb::eventdispatcher::kDefaultOrder,
                                 carb::RString("runloop:present:preUpdate"),
                                 [this](const carb::eventdispatcher::Event&) { this->presentPreNotify(); });
        }
        if (!m_presentPostSubscription)
        {
            m_presentPostSubscription =
                ed->observeEvent(carb::RStringKey("omni.kit.loop-default"), carb::eventdispatcher::kDefaultOrder,
                                 carb::RString("runloop:present:postUpdate"),
                                 [this](const carb::eventdispatcher::Event&) { this->presentPostNotify(); });
        }
    }
}
}
}
