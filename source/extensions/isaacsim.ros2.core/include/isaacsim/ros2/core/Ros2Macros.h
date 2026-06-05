// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief ROS 2 error handling macros
 * @details
 * This file provides utility macros for handling and reporting ROS Client Library (rcl)
 * errors in a consistent manner throughout the Isaac Sim ROS 2 core. The macros
 * facilitate error reporting with contextual information about where the error occurred.
 */
#pragma once

#include <carb/logging/Log.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcutils/logging_macros.h>

#include <stdarg.h>
#include <stdio.h>

namespace isaacsim
{
namespace ros2
{
namespace core
{

/**
 * @def RCL_ERROR_MSG
 * @brief Macro for printing RCL errors with context
 * @details
 * Prints the current RCL error string as an ERROR level message, including
 * information about where the error occurred. After printing, it resets
 * the RCL error state to prevent error propagation.
 *
 * Usage example:
 * ```cpp
 * if (rcl_operation_failed) {
 *     RCL_ERROR_MSG(MyClass::MyMethod, rcl_operation);
 *     return false;
 * }
 * ```
 *
 * @param caller The context where the error occurred (e.g., class, method, function)
 * @param called The RCL operation that failed
 */
#define RCL_ERROR_MSG(caller, called)                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
        printf("[" #caller "] error in " #called ": %s\n", rcutils_get_error_string().str);                            \
        rcl_reset_error();                                                                                             \
    } while (0)

/**
 * @def RCL_WARN_MSG
 * @brief Macro for printing RCL warnings with context
 * @details
 * Prints the current RCL error string as a WARNING level message, including
 * information about where the warning occurred. After printing, it resets
 * the RCL error state to prevent warning propagation.
 *
 * Usage example:
 * ```cpp
 * if (rcl_operation_suspicious) {
 *     RCL_WARN_MSG(MyClass::MyMethod, rcl_operation);
 *     // continue execution
 * }
 * ```
 *
 * @param caller The context where the warning occurred (e.g., class, method, function)
 * @param called The RCL operation that triggered the warning
 */
#define RCL_WARN_MSG(caller, called)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        printf("[" #caller "] warning in " #called ": %s\n", rcutils_get_error_string().str);                          \
        rcl_reset_error();                                                                                             \
    } while (0)

} // namespace core
} // namespace ros2
} // namespace isaacsim
