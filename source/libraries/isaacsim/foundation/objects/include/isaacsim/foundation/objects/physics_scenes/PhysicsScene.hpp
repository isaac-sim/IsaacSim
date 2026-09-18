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

#pragma once

#include <isaacsim/foundation/objects/Prim.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace physics_scenes
{

/**
 * @class PhysicsScene
 * @brief Wrapper over one or more USD Physics Scene prims.
 * @details
 * Extends the @c UsdPhysicsScene schema with the @c NewtonSceneAPI schema applied to every wrapped prim.
 *
 * All set/get methods operate in batch over the wrapped prims and accept an optional @p indices
 * parameter to restrict processing to a subset.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API PhysicsScene : public Prim
{
public:
    /**
     * @brief Construct a PhysicsScene wrapper, creating the prims if needed.
     * @details Existing prims must be of type @c PhysicsScene; non-existing paths are defined as
     *          @c PhysicsScene prims. The @c NewtonSceneAPI schema is applied to every wrapped prim.
     * @param[in] paths Single path or list of paths to USD Physics Scene prims. May include regular
     *                  expressions that are expanded against the active stage.
     * @throws std::runtime_error if an existing prim is not a USD Physics Scene prim.
     */
    explicit PhysicsScene(const std::variant<std::string, std::vector<std::string>>& paths);
    ~PhysicsScene() = default;

    /**
     * @brief Check whether the prims at the given paths are of the type handled by this class.
     * @details The paths are resolved against the active stage before being checked.
     *          Since this method is static, the returned array is always allocated on the CPU.
     * @param[in] paths Single path string or list of path strings. May include regular
     *                  expressions that are expanded against the active stage.
     * @return Boolean flags (dtype bool, shape @c (N,1)), one per resolved prim.
     * @throws std::runtime_error if the given paths do not correspond to existing prims.
     */
    static array::Array areOfType(const std::variant<std::string, std::vector<std::string>>& paths);

    /**
     * @brief Get the paths of every USD Physics Scene prim on the active stage.
     * @details The whole stage hierarchy is traversed, so scenes nested at any depth are reported.
     * @return Ordered list of absolute prim paths. Empty if the stage holds no Physics Scene prim.
     * @throws std::runtime_error if no active or default stage has been set.
     */
    static std::vector<std::string> getPhysicsScenePaths();

    /**
     * @brief Set the gravity vectors of the selected Physics Scene prims.
     * @details The vector is decomposed into the @c physics:gravityMagnitude and
     *          @c physics:gravityDirection attributes, both of which USD expresses in the stage's
     *          own distance unit. A zero-length vector is stored as a zero magnitude along the
     *          stage's down direction.
     * @param[in] gravities Gravity vectors in stage units per second squared (shape @c (N,3)).
     *                      Broadcast rules apply.
     * @param[in] indices   Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setGravities(const array::Array& gravities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the gravity vectors of the selected Physics Scene prims.
     * @details USD defines two independent sentinels, both of which are resolved here:
     *          a zero @c physics:gravityDirection requests the stage's down direction, and a
     *          negative @c physics:gravityMagnitude (its @c -inf default included) requests the
     *          standard gravity of 9.81 m/s², expressed in stage units regardless of how the stage
     *          is scaled. Any other magnitude is taken as-is, in stage units per second squared.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Gravity vectors in stage units per second squared (shape @c (N,3)).
     */
    array::Array getGravities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the simulation delta times (DT) of the selected Physics Scene prims.
     * @details Convenience wrapper that computes the step frequency as @c 1/dt and applies it
     *          through @ref setTimeStepsPerSecond.
     * @param[in] deltaTimes     Delta times in seconds (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a delta time is outside the range @c (0.0, 1.0], or is so
     *         small that its step frequency does not fit in a 32-bit integer.
     * @warning Due to the rounding of values in the *time-steps-per-second* (integer) --
     *          *delta-time* (floating point) conversion process, the delta time reported by
     *          @ref getDeltaTimes may not be exactly the one passed to @ref setDeltaTimes.
     */
    void setDeltaTimes(const array::Array& deltaTimes, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the simulation delta times (DT) of the selected Physics Scene prims.
     * @details Convenience wrapper that derives the delta time from the step frequency reported by
     *          @ref getTimeStepsPerSecond.
     * @note These methods are not virtual, so the one that runs is chosen by the static type of the
     *       handle: calling @c getDeltaTimes through a @c PhysicsScene reference to a @c PhysxScene reports
     *       the solver-agnostic delta time.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Delta times in seconds (shape @c (N,1)).
     * @warning Due to the rounding of values in the *time-steps-per-second* (integer) --
     *          *delta-time* (floating point) conversion process, the delta time reported by
     *          @ref getDeltaTimes may not be exactly the one passed to @ref setDeltaTimes.
     */
    array::Array getDeltaTimes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the simulation step frequencies of the selected Physics Scene prims.
     * @details Writes the solver-agnostic @c newton:timeStepsPerSecond attribute. The
     *          solver-specific subclasses declare their own @c setTimeStepsPerSecond, which calls
     *          this one before writing their own attribute, so both stay in sync.
     * @param[in] timeStepsPerSecond Step frequencies in hertz (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices             Indices of prims to process. If omitted, all wrapped prims are
     *                                processed.
     */
    void setTimeStepsPerSecond(const array::Array& timeStepsPerSecond,
                               const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the simulation step frequencies of the selected Physics Scene prims.
     * @details Reports the solver-agnostic @c newton:timeStepsPerSecond attribute. The
     *          solver-specific subclasses declare their own @c getTimeStepsPerSecond, which reports
     *          their own attribute instead, falling back to this one when their schema is not applied.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Step frequencies in hertz (shape @c (N,1)).
     */
    array::Array getTimeStepsPerSecond(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable gravity for the selected Physics Scene prims.
     * @details Toggling gravity leaves the gravity magnitude and direction untouched.
     * @note Backed by @c newton:gravityEnabled.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledGravities(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of gravity for the selected Physics Scene prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)).
     */
    array::Array getEnabledGravities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the maximum number of solver iterations of the selected Physics Scene prims.
     * @note Backed by @c newton:maxSolverIterations.
     * @param[in] iterations Iteration counts (shape @c (N,)). A value of @c -1 lets the solver
     *                       choose its own default. Broadcast rules apply.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setMaxSolverIterations(const array::Array& iterations,
                                const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the maximum number of solver iterations of the selected Physics Scene prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Iteration counts (shape @c (N,1)). A value of @c -1 means the solver default.
     */
    array::Array getMaxSolverIterations(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim
