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


#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <future>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

namespace isaacsim
{
namespace robot
{
namespace surface_gripper
{

template <typename Func>
inline void parallelForIndex(size_t count, Func func)
{
    auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

    carb::tasking::TaskGroup threads;
    if (count == 0)
    {
        return;
    }

    // reduce spawned carb tasks from O(n) to O(min(n, cores)) and chunk to reduce overhead

    // Creates chunks based on available hardware threads
    const unsigned int numThreads = std::thread::hardware_concurrency();
    const size_t maxWorkers = numThreads == 0 ? 1u : static_cast<size_t>(numThreads);

    // TODO: evaluate tasking overhead to determine best strategy for smaller counts
    if (count <= numThreads / 2)
    {
        for (size_t i = 0; i < count; i++)
        {
            func(i);
        }
        return;
    }

    const size_t numTasks = count < maxWorkers ? count : maxWorkers;
    const size_t chunkSize = (count + numTasks - 1) / numTasks;

    for (size_t t = 0; t < numTasks; t++)
    {
        const size_t start = t * chunkSize;
        if (start >= count)
        {
            break;
        }
        const size_t end = ((t + 1) * chunkSize) < count ? ((t + 1) * chunkSize) : count;
        tasking->addTask(carb::tasking::Priority::eHigh, threads,
                         [start, end, &func]()
                         {
                             for (size_t i = start; i < end; i++)
                             {
                                 func(i);
                             }
                         });
    }

    threads.wait();
}

} // namespace surface_gripper
} // namespace robot
} // namespace isaacsim
