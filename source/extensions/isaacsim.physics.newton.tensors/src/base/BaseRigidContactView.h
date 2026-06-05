// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "utils/ViewUtils.h"

#include <omni/physics/tensors/IRigidContactView.h>
#include <pxr/usd/sdf/path.h>
#include <pybind11/pybind11.h>

#include <string>
#include <vector>

namespace py = pybind11;

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using omni::physics::tensors::IRigidContactView;
using omni::physics::tensors::TensorDesc;

/// Base contact view implementing IRigidContactView for Newton.
///
/// Builds sensor-to-body and filter-to-body mapping tables from USD paths. Caches
/// contact data pointers (forces, normals, shape indices, points) from the Newton
/// contacts object, refreshing lazily when the simulation timestamp changes.
/// Pre-allocates scratch buffers for contact counting and indexing.
class BaseRigidContactView : public IRigidContactView
{
public:
    /**
     * @brief Constructs a BaseRigidContactView.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] sensorPaths USD prim paths of the contact sensors.
     * @param[in] filterPaths Filter paths.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     */
    BaseRigidContactView(py::object newtonStage,
                         const std::vector<std::string>& sensorPaths,
                         const std::vector<std::vector<std::string>>& filterPaths,
                         uint32_t maxContactDataCount);
    ~BaseRigidContactView() override;

    /**
     * @brief Gets the sensor count.
     * @return The requested value.
     */
    uint32_t getSensorCount() const override;
    /**
     * @brief Gets the filter count.
     * @return The requested value.
     */
    uint32_t getFilterCount() const override;
    /**
     * @brief Gets the maximum contact data count.
     * @return The requested value.
     */
    uint32_t getMaxContactDataCount() const override;

    /**
     * @brief Gets the friction data.
     * @param[out] frictionForceTensor Destination tensor that receives the friction force values.
     * @param[out] contactPointTensor Destination tensor that receives the contact point values.
     * @param[out] contactCountTensor Destination tensor that receives the contact count values.
     * @param[out] contactStartIndicesTensor Destination tensor that receives the contact start indices values.
     * @param[in] dt Time step, in seconds.
     * @return True on success; false otherwise.
     */
    bool getFrictionData(const TensorDesc* frictionForceTensor,
                         const TensorDesc* contactPointTensor,
                         const TensorDesc* contactCountTensor,
                         const TensorDesc* contactStartIndicesTensor,
                         float dt) const override;

    /**
     * @brief Gets the other actor paths from ids.
     * @param[in] otherActorIdsTensor Tensor of other-actor identifiers to resolve.
     * @param[out] outPaths Receives the resolved USD prim paths.
     */
    void getOtherActorPathsFromIds(const TensorDesc* otherActorIdsTensor,
                                   std::vector<std::string>& outPaths) const override;

    /**
     * @brief Checks whether the view and its backing data are still valid.
     * @return True on success; false otherwise.
     */
    bool check() const override;
    /**
     * @brief Releases the view and frees its associated resources.
     */
    void release() override;

    /**
     * @brief Gets the USD prim path.
     * @param[in] sensorIdx Zero-based sensor index within the view.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdPrimPath(uint32_t sensorIdx) const override;
    /**
     * @brief Gets the USD prim name.
     * @param[in] sensorIdx Zero-based sensor index within the view.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdPrimName(uint32_t sensorIdx) const override;
    /**
     * @brief Gets the filter USD prim path.
     * @param[in] sensorIdx Zero-based sensor index within the view.
     * @param[in] filterIdx Zero-based filter index within the sensor.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const override;
    /**
     * @brief Gets the filter USD prim name.
     * @param[in] sensorIdx Zero-based sensor index within the view.
     * @param[in] filterIdx Zero-based filter index within the sensor.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const override;

protected:
    /// Caches shape_body pointer (static across simulation lifetime).
    void _cacheStaticPointers();
    /// Re-reads contact data pointers from Newton if the simulation timestamp has advanced.
    void _refreshContactPointers() const;
    /**
     * @brief Gets the physics dt scale.
     * @param[in] userDt User-provided time step, in seconds.
     * @return The requested value.
     */
    float _getPhysicsDtScale(float userDt) const;

    py::object m_newtonStage; ///< Newton stage object backing the view.
    py::object m_model; ///< Newton model object backing the view.

    uint32_t m_sensorCount; ///< Sensor count.
    uint32_t m_filterCount; ///< Filter count.
    uint32_t m_maxContactDataCount; ///< Maximum contact data count.
    int m_rigidContactMax; ///< Rigid contact maximum.
    int m_worldBodyIndex; ///< World body index.
    int m_bodyCount; ///< Body count.

    std::vector<std::string> m_sensorPaths; ///< Sensor paths.
    std::vector<std::string> m_sensorNames; ///< Sensor names.
    std::vector<std::vector<std::string>> m_filterPaths; ///< Filter paths.
    std::vector<std::vector<std::string>> m_filterNames; ///< Filter names.

    std::vector<int> m_hostBodySensorMap; ///< [numBodies] → sensor index, -1 if not a sensor.
    std::vector<int> m_hostBodyFilterMap; ///< [sensorCount * numBodies] → filter index per sensor, -1 if no match.

    mutable int* m_cachedContactCount = nullptr; ///< Cached pointer to the contact count data.
    mutable int* m_cachedShape0 = nullptr; ///< Cached pointer to the shape 0 data.
    mutable int* m_cachedShape1 = nullptr; ///< Cached pointer to the shape 1 data.
    mutable float* m_cachedContactNormal = nullptr; ///< Cached pointer to the contact normal data.
    mutable float* m_cachedContactForce = nullptr; ///< Cached pointer to the contact force data.
    mutable float* m_cachedContactPoint0 = nullptr; ///< Cached pointer to the contact point 0 data.
    mutable float* m_cachedContactPoint1 = nullptr; ///< Cached pointer to the contact point 1 data.
    mutable float* m_cachedThickness0 = nullptr; ///< Cached pointer to the thickness 0 data.
    mutable float* m_cachedThickness1 = nullptr; ///< Cached pointer to the thickness 1 data.
    int* m_cachedShapeBody = nullptr; ///< Cached pointer to the shape body data.
    mutable float* m_cachedBodyQ = nullptr; ///< Cached pointer to the body q data.

    mutable float* m_cachedSpatialForce = nullptr; ///< Cached pointer to the spatial force data.
    mutable bool m_hasSpatialForce = false; ///< Has spatial force.

    std::vector<std::string> m_bodyLabels; ///< Body labels.

    mutable uint64_t m_lastRefreshedGeneration = UINT64_MAX; ///< Last-seen simulation_timestamp.
    float m_physicsDt = 0.0f; ///< Physics dt from SimulationManager for force-to-impulse scaling.
    mutable bool m_contactPointsInWorldSpace = false; ///< True when contact points are MuJoCo world-space positions.

    // Pre-allocated scratch buffers for contact counting (sized to sensorCount * max(filterCount, 1)).
    mutable std::vector<uint32_t> m_scratchCounts; ///< Scratch counts.
    mutable std::vector<uint32_t> m_scratchStartIndices; ///< Scratch start indices.
    mutable std::vector<uint32_t> m_scratchFillCounts; ///< Scratch fill counts.
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
