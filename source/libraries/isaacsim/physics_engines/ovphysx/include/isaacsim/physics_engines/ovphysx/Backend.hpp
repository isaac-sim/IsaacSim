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

/**
 * @file
 * @brief Declares lifecycle and configuration functions for the ovphysx simulation backend.
 */

#pragma once

#include "isaacsim/physics_engines/ovphysx/Export.h"

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

/**
 * @brief Activates the process-wide ovphysx simulation backend.
 *
 * Creates an ovphysx adapter, registers it with the physics manager, and registers the tensor simulation-view
 * factory. Calling this function while the backend is active has no effect and succeeds.
 *
 * @return `true` if activation completed or the backend was already active; `false` if activation did not complete.
 *
 * @warning A failure after manager or tensor registration begins may leave partially registered process-wide state.
 *          A `false` return therefore does not guarantee that every activation side effect was rolled back.
 */
ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API bool activate();

/**
 * @brief Shuts down the process-wide ovphysx simulation backend.
 *
 * Unregisters the tensor simulation-view factory and physics adapter, then releases the adapter. Calling this
 * function while the backend is inactive has no effect.
 */
ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API void shutdown();

/**
 * @brief Sets whether PhysX readback is suppressed for subsequently created scenes.
 *
 * Updates the process-wide `/physics/suppressReadback` setting, which controls the PhysX Direct GPU API. The ovphysx
 * backend reads the setting when it creates or recreates a scene, so callers must set the value before the scene's
 * first simulation step.
 *
 * @param[in] enable `true` to suppress readback and enable the PhysX Direct GPU API; `false` to allow readback.
 */
ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API void setSuppressReadback(bool enable);

/**
 * @brief Declares the device on which this backend's tensors reside.
 *
 * ovphysx exposes no query for the device its scene selected, so the caller that configured the tensor device
 * states it here. Entity views report this value, and they read it on each access, so a caller that only learns
 * the device once stepping has begun may set it after its views exist.
 *
 * @param[in] ordinal CUDA device ordinal, or `-1` for host memory.
 */
ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API void setTensorDeviceOrdinal(int ordinal);

/**
 * @brief Gets the declared tensor-device ordinal.
 *
 * @return The ordinal passed to @ref setTensorDeviceOrdinal, or `-1` when none was declared.
 */
ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API int getTensorDeviceOrdinal();

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
