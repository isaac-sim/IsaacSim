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

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{
namespace kit
{

/**
 * @brief Interface for controlling the run loop execution
 * @details Provides functionality to control the simulation loop's execution mode
 *          and timing, allowing for manual stepping and mode control
 */
struct IRunLoopRunnerImpl
{
    CARB_PLUGIN_INTERFACE("omni::kit::IRunLoopRunnerImpl", 1, 3);

    /**
     * @brief Enables or disables manual stepping mode
     * @param[in] enabled True to enable manual stepping, false for automatic
     * @param[in] name Identifier for the run loop instance
     */
    void(CARB_ABI* setManualMode)(const bool enabled, const std::string& name);
    /**
     * @brief Sets the time step size for manual stepping
     * @param[in] dt Time step size in seconds
     * @param[in] name Identifier for the run loop instance
     */
    void(CARB_ABI* setManualStepSize)(const double dt, const std::string& name);

    /**
     * @brief Gets the manual mode for the run loop
     * @param[in] name Identifier for the run loop instance
     * @return True if manual mode is enabled, false otherwise
     */
    bool(CARB_ABI* getManualMode)(const std::string& name);
    /**
     * @brief Gets the manual step size for the run loop
     * @param[in] name Identifier for the run loop instance
     * @return Manual step size in seconds
     */
    double(CARB_ABI* getManualStepSize)(const std::string& name);
};
}
}
