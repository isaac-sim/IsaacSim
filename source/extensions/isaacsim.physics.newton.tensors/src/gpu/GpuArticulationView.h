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

#include "base/BaseArticulationView.h"

#include <cuda_runtime.h>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

/// GPU articulation view. Implements getters via CUDA gather kernels with D2H staging,
/// and setters via fused CUDA scatter kernels. Supports GC (GPU sim, CPU view tensors)
/// by staging CPU index/source data into pre-allocated device buffers.
class GpuArticulationView : public BaseArticulationView
{
public:
    /**
     * @brief Constructs a GpuArticulationView.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] articulationPaths USD prim paths of the articulations.
     * @param[in] deviceOrdinal Device ordinal.
     */
    GpuArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& articulationPaths, int deviceOrdinal);
    ~GpuArticulationView() override;

    /**
     * @brief Gets the DOF positions.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofPositions(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF velocities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofVelocities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the DOF positions.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF velocities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    /**
     * @brief Gets the DOF limits.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofLimits(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF stiffnesses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofStiffnesses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF dampings.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofDampings(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF armatures.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofArmatures(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF maximum forces.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofMaxForces(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF maximum velocities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofMaxVelocities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF position targets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofPositionTargets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF velocity targets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofVelocityTargets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF actuation forces.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofActuationForces(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF types.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofTypes(const TensorDesc* dstTensor) const override;

    /**
     * @brief Sets the DOF limits.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF stiffnesses.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF dampings.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF maximum forces.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF maximum velocities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF armatures.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF actuation forces.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF position targets.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF velocity targets.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    /**
     * @brief Gets the root transforms.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getRootTransforms(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the root velocities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getRootVelocities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the root transforms.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the root velocities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    /**
     * @brief Gets the link transforms.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getLinkTransforms(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the link velocities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getLinkVelocities(const TensorDesc* dstTensor) const override;

    /**
     * @brief Gets the masses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getMasses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the inverse masses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getInvMasses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the masses.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Gets the COMs.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getCOMs(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the COMs.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Gets the inertias.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getInertias(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the inverse inertias.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getInvInertias(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the inertias.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    /**
     * @brief Applies the forces and torques at position.
     * @param[in] srcForceTensor Source tensor of forces to apply.
     * @param[in] srcTorqueTensor Source tensor of torques to apply.
     * @param[in] srcPositionTensor Source tensor of positions at which the forces are applied.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @param[in] isGlobal If true, the inputs are expressed in the world frame; otherwise in the local frame.
     * @return True on success; false otherwise.
     */
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         bool isGlobal) override;

private:
    int m_deviceOrdinal;

    // Device-side copies of index mappings (uploaded once at construction).
    int* m_deviceDofPositionIndices = nullptr;
    int* m_deviceDofVelocityIndices = nullptr;
    int* m_deviceDofAxisIndices = nullptr;
    int* m_deviceLinkFlatIndices = nullptr;
    int* m_deviceRootIndices = nullptr;
    int* m_deviceRootJointQStartIndices = nullptr;
    int* m_deviceFixedRootJointMapping = nullptr; ///< [count] artiIdx → rootJointIdx for fixed roots, -1 for free.
    int* m_deviceIndexScratch = nullptr; ///< Scratch buffer for H2D staging of CPU index tensors.
    float* m_stagingBuffer = nullptr; ///< Reusable device buffer for D2H getter output and H2D setter input.
    size_t m_stagingMaxFloats = 0; ///< Capacity of m_stagingBuffer in float elements.

    uint8_t* m_deviceDofTypes = nullptr;
    float* m_deviceComOrientation = nullptr; ///< Device-side COM orientation cache [count * maxLinks * 4].

    /// Allocates and uploads all host-side index arrays to device memory.
    void _uploadMappingsToGpu();
    /// Allocates m_stagingBuffer and m_deviceIndexScratch sized to the maximum needed.
    void _allocateStagingBuffer();
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
