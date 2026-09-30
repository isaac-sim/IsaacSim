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

#include <isaacsim/foundation/objects/physics_scenes/PhysicsScene.hpp>

namespace isaacsim
{
namespace foundation
{
namespace objects
{
namespace physics_scenes
{

/**
 * @class NewtonMjcScene
 * @brief Wrapper over one or more USD Physics Scene prims configured for the Newton MuJoCo solver.
 * @details
 * Extends PhysicsScene with the integrator, solver and medium settings declared by the
 * @c MjcSceneAPI schema, which the constructor applies to every wrapped prim on top of the schemas
 * applied by the base class.
 *
 * @note Applying @c MjcSceneAPI requires the MuJoCo USD schemas to be registered in the running
 *       process. The constructor throws if they are not.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API NewtonMjcScene : public PhysicsScene
{
public:
    /**
     * @brief Construct a NewtonMjcScene wrapper, creating the prims if needed.
     * @details Existing prims must be of type @c PhysicsScene; non-existing paths are defined as
     *          @c PhysicsScene prims. The @c MjcSceneAPI schema is applied to every wrapped prim.
     * @param[in] paths Single path or list of paths to USD Physics Scene prims. May include regular
     *                  expressions that are expanded against the active stage.
     * @throws std::runtime_error if an existing prim is not a USD Physics Scene prim.
     * @throws std::invalid_argument if the MuJoCo USD schemas are not registered.
     */
    explicit NewtonMjcScene(const std::variant<std::string, std::vector<std::string>>& paths);
    ~NewtonMjcScene() = default;

    /**
     * @brief Set the MuJoCo delta times (DT) of the selected prims.
     * @details Writes the @c mjc:option:timestep attribute, and calls @ref PhysicsScene::setDeltaTimes so
     *          that the solver-agnostic delta time stays in sync. The range is therefore validated
     *          by the base class.
     * @param[in] deltaTimes     Delta times in seconds (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a delta time is outside the range @c (0.0, 1.0], or is so
     *         small that its step frequency does not fit in a 32-bit integer.
     */
    void setDeltaTimes(const array::Array& deltaTimes, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the MuJoCo delta times (DT) of the selected prims.
     * @details Reports the @c mjc:option:timestep attribute. If any selected prim no longer has the
     *          @c MjcSceneAPI schema applied, @ref PhysicsScene::getDeltaTimes is reported for the whole
     *          selection instead.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Delta times in seconds (shape @c (N,1)).
     */
    array::Array getDeltaTimes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the MuJoCo step frequencies of the selected prims.
     * @details Writes the @c mjc:option:timestep attribute as @c 1/timeStepsPerSecond, and calls
     *          @ref PhysicsScene::setTimeStepsPerSecond so that the solver-agnostic step frequency
     *          stays in sync.
     * @param[in] timeStepsPerSecond Step frequencies in hertz (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices             Indices of prims to process. If omitted, all wrapped prims are
     *                                processed.
     * @throws std::invalid_argument if a step frequency is not greater than zero.
     */
    void setTimeStepsPerSecond(const array::Array& timeStepsPerSecond,
                               const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the MuJoCo step frequencies of the selected prims.
     * @details Derives the frequency from the @c mjc:option:timestep attribute. If any selected prim
     *          no longer has the @c MjcSceneAPI schema applied,
     *          @ref PhysicsScene::getTimeStepsPerSecond is reported for the whole selection instead.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Step frequencies in hertz (shape @c (N,1)).
     * @warning Due to the rounding of values in the *delta-time* (floating point) --
     *          *time-steps-per-second* (integer) conversion process, the step frequency reported by
     *          this method may not be exactly the one passed to @ref setTimeStepsPerSecond, and the
     *          delta time reported by @ref getDeltaTimes may not be exactly @c 1/timeStepsPerSecond.
     */
    array::Array getTimeStepsPerSecond(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the numerical integrators of the selected prims.
     * @param[in] integrators Integrator token, one of @c "euler", @c "rk4", @c "implicit" or
     *                        @c "implicitfast". A single string is applied to every selected prim.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported integrator.
     */
    void setIntegrators(const std::variant<std::string, std::vector<std::string>>& integrators,
                        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the numerical integrators of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Integrator tokens, one per selected prim.
     */
    std::vector<std::string> getIntegrators(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the constraint solver algorithms of the selected prims.
     * @param[in] solvers Solver token, one of @c "pgs", @c "cg" or @c "newton". A single string is
     *                    applied to every selected prim.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported solver.
     */
    void setSolvers(const std::variant<std::string, std::vector<std::string>>& solvers,
                    const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the constraint solver algorithms of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Solver tokens, one per selected prim.
     */
    std::vector<std::string> getSolvers(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the friction cone types of the selected prims.
     * @param[in] cones   Cone token, either @c "pyramidal" or @c "elliptic". A single string is
     *                    applied to every selected prim.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported friction cone type.
     */
    void setCones(const std::variant<std::string, std::vector<std::string>>& cones,
                  const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the friction cone types of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Cone tokens, one per selected prim.
     */
    std::vector<std::string> getCones(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the constraint Jacobian types of the selected prims.
     * @param[in] jacobians Jacobian token, one of @c "auto", @c "dense" or @c "sparse". A single
     *                      string is applied to every selected prim.
     * @param[in] indices   Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported Jacobian type.
     */
    void setJacobians(const std::variant<std::string, std::vector<std::string>>& jacobians,
                      const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the constraint Jacobian types of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Jacobian tokens, one per selected prim.
     */
    std::vector<std::string> getJacobians(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the maximum numbers of constraint solver iterations of the selected prims.
     * @param[in] iterations Iteration counts (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setIterations(const array::Array& iterations, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the maximum numbers of constraint solver iterations of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Iteration counts (shape @c (N,1)).
     */
    array::Array getIterations(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the solver tolerances for early termination of the selected prims.
     * @param[in] tolerances Tolerance thresholds (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices    Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setTolerances(const array::Array& tolerances, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the solver tolerances for early termination of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Tolerance thresholds (shape @c (N,1)).
     */
    array::Array getTolerances(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the impedance ratios of the selected prims.
     * @details The ratio of frictional-to-normal constraint impedance. Takes effect only for
     *          elliptic friction cones.
     * @param[in] impedanceRatios Impedance ratios (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices         Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setImpedanceRatios(const array::Array& impedanceRatios,
                            const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the impedance ratios of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Impedance ratios (shape @c (N,1)).
     */
    array::Array getImpedanceRatios(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the wind velocity vectors of the selected prims.
     * @param[in] winds   Velocity vectors of the medium (shape @c (N,3)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setWinds(const array::Array& winds, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the wind velocity vectors of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Velocity vectors of the medium (shape @c (N,3)).
     */
    array::Array getWinds(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the medium densities of the selected prims.
     * @param[in] densities Densities of the medium (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices   Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setDensities(const array::Array& densities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the medium densities of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Densities of the medium (shape @c (N,1)).
     */
    array::Array getDensities(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the medium viscosities of the selected prims.
     * @param[in] viscosities Viscosities of the medium (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setViscosities(const array::Array& viscosities, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the medium viscosities of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Viscosities of the medium (shape @c (N,1)).
     */
    array::Array getViscosities(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim
