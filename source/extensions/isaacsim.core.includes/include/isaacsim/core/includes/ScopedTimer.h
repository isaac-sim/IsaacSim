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

#include <chrono>
#include <iostream>
#include <string>

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class ScopedTimer
 * @brief RAII-style performance timer for code block measurement.
 * @details
 * Provides automatic timing of code blocks using RAII principles.
 * Measures elapsed time between construction and destruction,
 * automatically printing the results with a custom message.
 *
 * Features:
 * - High-resolution timing using std::chrono::steady_clock
 * - Automatic measurement of scoped blocks
 * - Custom message support for timing identification
 * - Millisecond precision output
 *
 * Example usage:
 * @code
 * {
 *     ScopedTimer timer("Operation X");
 *     // Code to measure...
 * } // Timer automatically prints duration when scope ends
 * @endcode
 *
 * @note Uses steady_clock for most reliable timing measurements
 * @warning Output is sent to std::cout, which may affect timing of very short operations
 */
class ScopedTimer
{
public:
    /**
     * @brief Constructs a timer with an identifying message.
     * @details
     * Starts the timer immediately upon construction and stores the message
     * for later output when the timer is destroyed.
     *
     * @param[in] message Descriptive message to identify this timing measurement
     *
     * @note The message will be printed with the elapsed time when the timer is destroyed
     */
    ScopedTimer(const std::string& message)
    {
        m_message = message;
        m_start = std::chrono::steady_clock::now();
    }

    /**
     * @brief Destructor that prints the elapsed time.
     * @details
     * Automatically calculates and prints the elapsed time since construction.
     * Output format: "<message> : <elapsed_time_ms>"
     *
     * @note Time is reported in milliseconds with floating-point precision
     */
    ~ScopedTimer()
    {
        m_stop = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> diff = m_stop - m_start;
        std::cout << m_message << " : " << diff.count() << std::endl;
    }

private:
    /** @brief Start time point of the measurement */
    std::chrono::time_point<std::chrono::steady_clock> m_start;

    /** @brief Stop time point of the measurement */
    std::chrono::time_point<std::chrono::steady_clock> m_stop;

    /** @brief Identifying message for this timing measurement */
    std::string m_message;
};

}
}
}
