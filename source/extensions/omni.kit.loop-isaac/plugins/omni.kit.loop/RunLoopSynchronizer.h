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

#pragma once

#include <carb/dictionary/IDictionary.h>
#include <carb/eventdispatcher/EventDispatcherTypes.h>
#include <carb/events/EventsUtils.h>
#include <carb/tasking/TaskingUtils.h>

#include <chrono>
#include <cstddef>
#include <memory>

namespace omni
{
namespace kit
{

class IRendererRunLoopGate;
class SlidingMaximum;

/**
 * @brief Class for synchronizing multiple run loops
 * @details Provides functionality to synchronize multiple threads or run loops
 *          to a common timing source, typically the present/render thread.
 *          Supports frame rate control and timing adjustments.
 */
class RunLoopSynchronizer
{
public:
    RunLoopSynchronizer();

    ~RunLoopSynchronizer();

    /**
     * @brief Saves the present time and duration.
     */
    void presentPreNotify();

    /**
     * @brief Notifies the condition variable in `wait` that it has to wake up.
     * Saves the synchronization point.
     */
    void presentPostNotify();

    /**
     * @brief Waits for the present thread and syncs the calling thread to the
     * present thread.
     *
     * This function calculates how long it needs to wait using a Sliding
     * Maximum of the already passed time. It uses a high resolution clock to
     * wait until the desired frame duration is met. It starts by obtaining the
     * current time, and calculates the sliding maximum of the already passed
     * time, ignoring a specified number of outliers. The function then
     * determines if waiting is necessary by comparing the current time with the
     * computed time point it should wake up at.
     *
     * @param alreadyPassedNs The duration in nanoseconds that has already
     *                        passed.
     * @param slidingMaximumCount The number of recent durations to consider
     *                            when finding the sliding maximum.
     * @param slidingMaximumOutlierCount The number of outlier durations to
     *                                   ignore when finding the sliding
     *                                   maximum.
     * @param slidingMaximumToleranceFactor A multiplier for the average
     *                                      duration. Durations exceeding this
     *                                      are considered outliers.
     */
    void wait(float alreadyPassedNs,
              size_t slidingMaximumCount,
              size_t slidingMaximumOutlierCount,
              float slidingMaximumToleranceFactor);

    /**
     * @brief Sets the target frames per second
     * @param[in] fps Desired frame rate in frames per second
     */
    void setTargetFPS(double fps);

    /**
     * @brief Checks if the synchronizer is active
     * @return True if synchronization is active, false otherwise
     */
    bool isActive() const;

    /**
     * @brief Enables or disables synchronization
     * @param[in] active True to enable synchronization, false to disable
     */
    void setActive(bool active);

private:
    /**
     * @brief Sets up or tears down the present thread subscription
     * @details Creates or deletes subscriptions for present thread notifications
     *          based on current synchronization state
     */
    void _setupPresentThread();

    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;
    using TimePoint = Clock::time_point;

    carb::tasking::MutexWrapper m_presentMutex;
    Duration m_presentDuration;
    TimePoint m_presentStartTime;
    // All the threads will be synced to this time point.
    TimePoint m_presentCheckpointTime;

    carb::tasking::MutexWrapper m_targetMutex;
    Duration m_targetDuration;

    std::unique_ptr<IRendererRunLoopGate> m_gate;
    std::unique_ptr<SlidingMaximum> m_sliding;

    // Present thread
    carb::eventdispatcher::ObserverGuard m_presentPreSubscription;
    carb::eventdispatcher::ObserverGuard m_presentPostSubscription;
    // Present thread setting watch
    carb::dictionary::SubscriptionId* m_presentThreadEnabledSubscription = nullptr;

    bool m_isActive = false;
    double m_fps = -1.0f;
};
}
}
