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

#include <omni/physics/tensors/IRigidBodyView.h>
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

using omni::physics::tensors::IRigidBodyView;
using omni::physics::tensors::TensorDesc;

/// Base rigid body view implementing IRigidBodyView for Newton.
///
/// Resolves USD body paths to Newton model body indices, identifies free-joint bodies for
/// root transform writes, and caches Warp array pointers for body_q, body_qd, mass,
/// inertia, and force arrays. Getter/setter methods are pure virtual and implemented by
/// CpuRigidBodyView and GpuRigidBodyView.
class BaseRigidBodyView : public IRigidBodyView
{
public:
    /**
     * @brief Constructs a BaseRigidBodyView.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] bodyPaths USD prim paths of the rigid bodies.
     */
    BaseRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& bodyPaths);
    ~BaseRigidBodyView() override;

    /**
     * @brief Returns the number of elements in the view.
     * @return The requested value.
     */
    uint32_t getCount() const override;
    /**
     * @brief Returns the maximum shape count across all elements in the view.
     * @return The requested value.
     */
    uint32_t getMaxShapes() const override;
    /**
     * @brief Gets the USD prim path.
     * @param[in] rbIdx Zero-based rigid body index within the view.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdPrimPath(uint32_t rbIdx) const override;

    // Stubs — unsupported by Newton
    /**
     * @brief Sets the kinematic targets.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the kinematic targets for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setKinematicTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Gets the disable gravities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDisableGravities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the disable simulations.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDisableSimulations(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the disable gravities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the disable simulations.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the disable gravities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the disable simulations for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableSimulationsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Wakes up the up.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool wakeUp(const TensorDesc* indexTensor) override;

    // Masked variants — delegate to indexed
    /**
     * @brief Sets the transforms for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setTransformsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the velocities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Applies the forces for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool applyForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Applies the forces and torques at position for the elements selected by a boolean mask.
     * @param[in] srcForceTensor Source tensor of forces to apply.
     * @param[in] srcTorqueTensor Source tensor of torques to apply.
     * @param[in] srcPositionTensor Source tensor of positions at which the forces are applied.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @param[in] isGlobal If true, the inputs are expressed in the world frame; otherwise in the local frame.
     * @return True on success; false otherwise.
     */
    bool applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                               const TensorDesc* srcTorqueTensor,
                                               const TensorDesc* srcPositionTensor,
                                               const TensorDesc* maskTensor,
                                               bool isGlobal) override;
    /**
     * @brief Sets the masses for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setMassesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the COMs for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setCOMsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the inertias for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setInertiasMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    // Material/shape stubs
    /**
     * @brief Gets the material properties.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getMaterialProperties(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the compliant material properties.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @param[out] dstCombineModeTensor Destination tensor that receives the material combine modes.
     * @return True on success; false otherwise.
     */
    bool getCompliantMaterialProperties(const TensorDesc* dstTensor,
                                        const TensorDesc* dstCombineModeTensor) const override;
    /**
     * @brief Gets the rest offsets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getRestOffsets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the contact offsets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getContactOffsets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the material properties.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the compliant material properties.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] srcCombineTensor Source tensor of material combine modes.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setCompliantMaterialProperties(const TensorDesc* srcTensor,
                                        const TensorDesc* srcCombineTensor,
                                        const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the rest offsets.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the contact offsets.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the material properties for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    /**
     * @brief Sets the compliant material properties for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] srcCombineTensor Source tensor of material combine modes.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    /**
     * @brief Sets the rest offsets for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    /**
     * @brief Sets the contact offsets for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;

    /**
     * @brief Checks whether the view and its backing data are still valid.
     * @return True on success; false otherwise.
     */
    bool check() const override;
    /**
     * @brief Releases the view and frees its associated resources.
     */
    void release() override;

protected:
    /**
     * @brief Evaluates the forward kinematics.
     */
    void _evalForwardKinematics();
    /**
     * @brief Evaluates the inverse kinematics.
     */
    void _evalInverseKinematics();
    /**
     * @brief Updates the inverse masses.
     */
    void _updateInverseMasses();
    /**
     * @brief Updates the inverse inertias.
     */
    void _updateInverseInertias();

    /**
     * @brief Resolves the indices.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return Reference to the requested object.
     */
    const std::vector<uint32_t>& _resolveIndices(const TensorDesc* indexTensor) const
    {
        resolveViewIndices(indexTensor, m_count, m_scratchViewIndices);
        return m_scratchViewIndices;
    }

    /**
     * @brief Resolves the GPU indices.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @param[in] devScratch Optional device scratch buffer; allocated internally when null.
     * @return The requested value.
     */
    isaacsim::physics::newton::tensors::GpuIndexGuard _resolveGpuIndices(const TensorDesc* indexTensor,
                                                                         int* devScratch = nullptr) const
    {
        return resolveGpuViewIndices(indexTensor, m_count, devScratch);
    }

    /**
     * @brief Builds the scatter mappings.
     * @param[in] viewIndices View indices selected by the operation.
     * @param[in] elemSize Number of floats per element.
     * @param[out] srcOffsets Receives the source offset for each scattered element.
     * @param[out] dstIndices Receives the destination index for each scattered element.
     */
    void _buildScatterMappings(const std::vector<uint32_t>& viewIndices,
                               int elemSize,
                               std::vector<int>& srcOffsets,
                               std::vector<int>& dstIndices) const;

    py::object m_newtonStage; ///< Newton stage object backing the view.
    py::object m_model; ///< Newton model object backing the view.

    uint32_t m_count; ///< Number of elements in the view.
    uint32_t m_maxShapes; ///< Maximum shape count across all elements in the view.

    std::vector<int> m_bodyIndices; ///< [count] → model body index for each view body.
    std::vector<std::string> m_primPaths; ///< Prim paths.
    std::vector<int> m_freeJointQStartIndices; ///< [count] → joint_q offset for free joints, -1 if fixed.

    mutable std::vector<uint32_t> m_scratchViewIndices; ///< Scratch buffer reused to resolve view indices without
                                                        ///< per-call allocation.

    bool m_cacheValid = false; ///< Cache valid.
    int m_modelDeviceOrdinal = -1; ///< Model device ordinal.
    size_t m_totalBodyCount = 0; ///< Total body count.

    mutable float* m_cachedJointQ = nullptr; ///< Cached pointer to the joint q data.
    mutable float* m_cachedBodyQ = nullptr; ///< Cached pointer to the body q data.
    mutable float* m_cachedBodyQd = nullptr; ///< Cached pointer to the body qd data.
    mutable float* m_cachedBodyQdd = nullptr; ///< Nullable; only available when body_qdd is requested.
    mutable float* m_cachedBodyF = nullptr; ///< Cached pointer to the body f data.
    float* m_cachedBodyMass = nullptr; ///< Cached pointer to the body mass data.
    float* m_cachedBodyInverseMass = nullptr; ///< Cached pointer to the body inverse mass data.
    float* m_cachedBodyInertia = nullptr; ///< Cached pointer to the body inertia data.
    float* m_cachedBodyInverseInertia = nullptr; ///< Cached pointer to the body inverse inertia data.
    float* m_cachedBodyCenterOfMass = nullptr; ///< Cached pointer to the body center of mass data.

    std::vector<float> m_cachedComOrientation; ///< [count * 4] cached COM quaternion (xyzw), write-through from
                                               ///< setCOMs.

    /**
     * @brief Caches the warp pointers.
     */
    void _cacheWarpPointers();
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
