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
 * @struct PhysxGpuConfiguration
 * @brief Batched GPU buffer sizing for the PhysX solver.
 * @details
 * Every field holds one value per selected prim (shape @c (N,1)). A field left unset is not
 * written by @ref PhysxScene::setGpuConfiguration, so a partial configuration can be applied
 * without reading the remaining values first. @ref PhysxScene::getGpuConfiguration always
 * returns every field populated.
 */
struct PhysxGpuConfiguration
{
    /** @brief Size of the GPU collision stack, in bytes. */
    std::optional<array::Array> gpuCollisionStackSize;
    /** @brief Capacity for GPU found/lost aggregate pairs. */
    std::optional<array::Array> gpuFoundLostAggregatePairsCapacity;
    /** @brief Capacity for GPU found/lost pairs. */
    std::optional<array::Array> gpuFoundLostPairsCapacity;
    /** @brief Size of the GPU heap, in bytes. */
    std::optional<array::Array> gpuHeapCapacity;
    /** @brief Maximum number of deformable surface contacts on the GPU. */
    std::optional<array::Array> gpuMaximumDeformableSurfaceContacts;
    /** @brief Maximum number of deformable volume (soft body) contacts on the GPU. */
    std::optional<array::Array> gpuMaximumDeformableVolumeContacts;
    /** @brief Maximum number of GPU solver partitions. */
    std::optional<array::Array> gpuMaximumPartitionCount;
    /** @brief Maximum number of particle contacts on the GPU. */
    std::optional<array::Array> gpuMaximumParticleContacts;
    /** @brief Maximum number of rigid body contacts on the GPU. */
    std::optional<array::Array> gpuMaximumRigidContactCount;
    /** @brief Maximum number of rigid body contact patches on the GPU. */
    std::optional<array::Array> gpuMaximumRigidPatchCount;
    /** @brief Size of the GPU temporary buffer, in bytes. */
    std::optional<array::Array> gpuTemporaryBufferCapacity;
    /** @brief Total capacity for GPU aggregate pairs. */
    std::optional<array::Array> gpuTotalAggregatePairsCapacity;
};

/**
 * @class PhysxScene
 * @brief Wrapper over one or more USD Physics Scene prims configured for the PhysX solver.
 * @details
 * Extends PhysicsScene with the settings declared by the @c PhysxSceneAPI schema, which the
 * constructor applies to every wrapped prim on top of the schemas applied by the base class.
 */
class ISAACSIM_FOUNDATION_OBJECTS_API PhysxScene : public PhysicsScene
{
public:
    /**
     * @brief Construct a PhysxScene wrapper, creating the prims if needed.
     * @details Existing prims must be of type @c PhysicsScene; non-existing paths are defined as
     *          @c PhysicsScene prims. The @c PhysxSceneAPI schema is applied to every wrapped prim.
     * @param[in] paths Single path or list of paths to USD Physics Scene prims. May include regular
     *                  expressions that are expanded against the active stage.
     * @throws std::runtime_error if an existing prim is not a USD Physics Scene prim.
     */
    explicit PhysxScene(const std::variant<std::string, std::vector<std::string>>& paths);
    ~PhysxScene() = default;

    /**
     * @brief Set the PhysX delta times (DT) of the selected prims.
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
     * @brief Get the PhysX delta times (DT) of the selected prims.
     * @details Convenience wrapper that derives the delta time from the step frequency reported by
     *          @ref getTimeStepsPerSecond.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Delta times in seconds (shape @c (N,1)).
     * @warning Due to the rounding of values in the *time-steps-per-second* (integer) --
     *          *delta-time* (floating point) conversion process, the delta time reported by
     *          @ref getDeltaTimes may not be exactly the one passed to @ref setDeltaTimes.
     */
    array::Array getDeltaTimes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the PhysX step frequencies of the selected prims.
     * @details Writes the @c physxScene:timeStepsPerSecond attribute, and calls
     *          @ref PhysicsScene::setTimeStepsPerSecond so that the solver-agnostic step frequency
     *          stays in sync.
     * @param[in] timeStepsPerSecond Step frequencies in hertz (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices             Indices of prims to process. If omitted, all wrapped prims are
     *                                processed.
     */
    void setTimeStepsPerSecond(const array::Array& timeStepsPerSecond,
                               const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the PhysX step frequencies of the selected prims.
     * @details Reports the @c physxScene:timeStepsPerSecond attribute. If any selected prim no
     *          longer has the @c PhysxSceneAPI schema applied, @ref PhysicsScene::getTimeStepsPerSecond
     *          is reported for the whole selection instead.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Step frequencies in hertz (shape @c (N,1)).
     */
    array::Array getTimeStepsPerSecond(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the solver types of the selected prims.
     * @param[in] solverTypes Solver type token, either @c "TGS" or @c "PGS". A single string is
     *                        applied to every selected prim.
     * @param[in] indices     Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported solver type.
     */
    void setSolverTypes(const std::variant<std::string, std::vector<std::string>>& solverTypes,
                        const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the solver types of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Solver type tokens, one per selected prim.
     */
    std::vector<std::string> getSolverTypes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the broadphase types of the selected prims.
     * @param[in] broadphaseTypes Broadphase type token, one of @c "MBP", @c "GPU" or @c "SAP".
     *                            A single string is applied to every selected prim.
     * @param[in] indices         Indices of prims to process. If omitted, all wrapped prims are processed.
     * @throws std::invalid_argument if a token is not a supported broadphase type.
     */
    void setBroadphaseTypes(const std::variant<std::string, std::vector<std::string>>& broadphaseTypes,
                            const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the broadphase types of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Broadphase type tokens, one per selected prim.
     */
    std::vector<std::string> getBroadphaseTypes(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable GPU dynamics for the selected prims.
     * @note GPU dynamics does not support Continuous Collision Detection (CCD), so the two settings
     *       are mutually exclusive. Enabling GPU dynamics on a prim therefore disables CCD on that
     *       same prim, and @ref setEnabledCcds applies the reciprocal rule: the setting enabled last
     *       is the one that stays on.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledGpuDynamics(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of GPU dynamics for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)).
     */
    array::Array getEnabledGpuDynamics(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable Continuous Collision Detection (CCD) for the selected prims.
     * @note CCD is not supported by GPU dynamics, so the two settings are mutually exclusive.
     *       Enabling CCD on a prim therefore disables GPU dynamics on that same prim; see
     *       @ref setEnabledGpuDynamics.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledCcds(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of Continuous Collision Detection (CCD) for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)).
     */
    array::Array getEnabledCcds(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Enable or disable solver stabilization for the selected prims.
     * @param[in] enabled Boolean flags (shape @c (N,)). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setEnabledStabilizations(const array::Array& enabled, const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the enabled state of solver stabilization for the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return Boolean flags (shape @c (N,1)).
     */
    array::Array getEnabledStabilizations(const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Set the GPU buffer sizing of the selected prims.
     * @details Only the fields set in @p configuration are written; the others are left untouched.
     * @param[in] configuration     GPU configuration. Each set field has shape @c (N,). Broadcast rules apply.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     */
    void setGpuConfiguration(const PhysxGpuConfiguration& configuration,
                             const std::optional<array::Array>& indices = std::nullopt);

    /**
     * @brief Get the GPU buffer sizing of the selected prims.
     * @param[in] indices Indices of prims to process. If omitted, all wrapped prims are processed.
     * @return GPU configuration with every field populated (each of shape @c (N,1)).
     */
    PhysxGpuConfiguration getGpuConfiguration(const std::optional<array::Array>& indices = std::nullopt);
};

} // namespace physics_scenes
} // namespace objects
} // namespace foundation
} // namespace isaacsim
