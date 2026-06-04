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

#include "ArticulationMetatype.h"
#include "utils/ViewUtils.h"

#include <omni/physics/tensors/IArticulationView.h>
#include <pxr/usd/sdf/path.h>
#include <pybind11/pybind11.h>

#include <limits>
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

using omni::physics::tensors::IArticulationMetatype;
using omni::physics::tensors::IArticulationView;
using omni::physics::tensors::TensorDesc;

/// Base articulation view implementing IArticulationView for Newton.
///
/// Parses the Newton model at construction to build DOF, link, and root index mappings.
/// Caches raw Warp array pointers for joint_q, joint_qd, body_q, body_qd, and property
/// arrays (stiffness, damping, limits, etc.) to avoid repeated Python lookups.
/// Getter/setter methods are pure virtual and implemented by CpuArticulationView and
/// GpuArticulationView.
class BaseArticulationView : public IArticulationView
{
public:
    /// Parses model.joint_q_start, joint_qd_start, joint_type, body_label, and joint_label
    /// to build all index mappings and construct per-articulation ArticulationMetatype objects.
    BaseArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& articulationPaths);
    ~BaseArticulationView() override;

    /**
     * @brief Returns the number of elements in the view.
     * @return The requested value.
     */
    uint32_t getCount() const override;
    /**
     * @brief Returns the maximum link count across all articulations in the view.
     * @return The requested value.
     */
    uint32_t getMaxLinks() const override;
    /**
     * @brief Returns the maximum DOF count across all articulations in the view.
     * @return The requested value.
     */
    uint32_t getMaxDofs() const override;
    /**
     * @brief Returns the maximum shape count across all elements in the view.
     * @return The requested value.
     */
    uint32_t getMaxShapes() const override;
    /**
     * @brief Returns the maximum fixed-tendon count across all articulations in the view.
     * @return The requested value.
     */
    uint32_t getMaxFixedTendons() const override;
    /**
     * @brief Returns the maximum spatial-tendon count across all articulations in the view.
     * @return The requested value.
     */
    uint32_t getMaxSpatialTendons() const override;

    /**
     * @brief Returns whether all elements in the view share the same metatype.
     * @return True on success; false otherwise.
     */
    bool isHomogeneous() const override;
    /**
     * @brief Returns the metatype shared by all elements when the view is homogeneous.
     * @return Pointer to the requested object, or nullptr if unavailable.
     */
    const IArticulationMetatype* getSharedMetatype() const override;
    /**
     * @brief Gets the metatype.
     * @param[in] artiIdx Zero-based articulation index within the view.
     * @return Pointer to the requested object, or nullptr if unavailable.
     */
    const IArticulationMetatype* getMetatype(uint32_t artiIdx) const override;

    /**
     * @brief Gets the USD prim path.
     * @param[in] artiIdx Zero-based articulation index within the view.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdPrimPath(uint32_t artiIdx) const override;
    /**
     * @brief Gets the USD DOF path.
     * @param[in] artiIdx Zero-based articulation index within the view.
     * @param[in] dofIdx Zero-based DOF index within the articulation.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const override;
    /**
     * @brief Gets the USD link path.
     * @param[in] artiIdx Zero-based articulation index within the view.
     * @param[in] linkIdx Zero-based link index within the articulation.
     * @return Pointer to the requested null-terminated string, or nullptr if unavailable.
     */
    const char* getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const override;

    // Stubs
    /**
     * @brief Gets the link accelerations.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getLinkAccelerations(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF projected joint forces.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofProjectedJointForces(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF motions.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofMotions(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the drive types.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDriveTypes(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF drive model properties.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofDriveModelProperties(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF friction coefficients.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofFrictionCoefficients(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the DOF friction properties.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDofFrictionProperties(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the disable gravities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getDisableGravities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the disable gravities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the disable gravities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Gets the articulation mass center.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @param[in] localFrame If true, the result is expressed in the local frame; otherwise in the world frame.
     * @return True on success; false otherwise.
     */
    bool getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const override;
    /**
     * @brief Gets the articulation centroidal momentum.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const override;
    /**
     * @brief Computes the row and column counts of the articulation Jacobians.
     * @param[out] numRows Receives the number of rows.
     * @param[out] numCols Receives the number of columns.
     * @return True on success; false otherwise.
     */
    bool getJacobianShape(uint32_t* numRows, uint32_t* numCols) const override;
    /**
     * @brief Gets the jacobians.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getJacobians(const TensorDesc* dstTensor) const override;
    /**
     * @brief Computes the row and column counts of the generalized mass matrices.
     * @param[out] numRows Receives the number of rows.
     * @param[out] numCols Receives the number of columns.
     * @return True on success; false otherwise.
     */
    bool getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override;
    /**
     * @brief Gets the generalized mass matrices.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getGeneralizedMassMatrices(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the coriolis and centrifugal compensation forces.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the gravity compensation forces.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getGravityCompensationForces(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the link incoming joint force.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getLinkIncomingJointForce(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the DOF drive model properties.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF friction coefficients.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the DOF friction properties.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    // Masked variants
    /**
     * @brief Sets the DOF limits for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofLimitsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF stiffnesses for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofStiffnessesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF dampings for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofDampingsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF maximum forces for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofMaxForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF drive model properties for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofDriveModelPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF friction coefficients for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofFrictionCoefficientsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF friction properties for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofFrictionPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF maximum velocities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofMaxVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF armatures for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofArmaturesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF positions for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofPositionsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF velocities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF actuation forces for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofActuationForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF position targets for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofPositionTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the DOF velocity targets for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setDofVelocityTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
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
    /**
     * @brief Sets the root transforms for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRootTransformsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    /**
     * @brief Sets the root velocities for the elements selected by a boolean mask.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setRootVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
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

    // Tendon stubs
    /**
     * @brief Gets the fixed tendon stiffnesses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonStiffnesses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the fixed tendon dampings.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonDampings(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the fixed tendon limit stiffnesses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the fixed tendon limits.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonLimits(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the fixed tendonfixed spring rest lengths.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the fixed tendon offsets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getFixedTendonOffsets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the spatial tendon stiffnesses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the spatial tendon dampings.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getSpatialTendonDampings(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the spatial tendon limit stiffnesses.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the spatial tendon offsets.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getSpatialTendonOffsets(const TensorDesc* dstTensor) const override;
    /**
     * @brief Sets the fixed tendon properties.
     * @param[in] stiffnesses Tensor of tendon stiffness values.
     * @param[in] dampings Tensor of tendon damping values.
     * @param[in] limitStiffnesses Tensor of tendon limit stiffness values.
     * @param[in] limits Tensor of tendon limit values.
     * @param[in] restLengths Tensor of tendon rest-length values.
     * @param[in] offsets Tensor of tendon offset values.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setFixedTendonProperties(const TensorDesc* stiffnesses,
                                  const TensorDesc* dampings,
                                  const TensorDesc* limitStiffnesses,
                                  const TensorDesc* limits,
                                  const TensorDesc* restLengths,
                                  const TensorDesc* offsets,
                                  const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the spatial tendon properties.
     * @param[in] stiffnesses Tensor of tendon stiffness values.
     * @param[in] dampings Tensor of tendon damping values.
     * @param[in] limitStiffnesses Tensor of tendon limit stiffness values.
     * @param[in] offsets Tensor of tendon offset values.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setSpatialTendonProperties(const TensorDesc* stiffnesses,
                                    const TensorDesc* dampings,
                                    const TensorDesc* limitStiffnesses,
                                    const TensorDesc* offsets,
                                    const TensorDesc* indexTensor) const override;
    /**
     * @brief Sets the fixed tendon properties for the elements selected by a boolean mask.
     * @param[in] stiffnesses Tensor of tendon stiffness values.
     * @param[in] dampings Tensor of tendon damping values.
     * @param[in] limitStiffnesses Tensor of tendon limit stiffness values.
     * @param[in] limits Tensor of tendon limit values.
     * @param[in] restLengths Tensor of tendon rest-length values.
     * @param[in] offsets Tensor of tendon offset values.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setFixedTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                        const TensorDesc* dampings,
                                        const TensorDesc* limitStiffnesses,
                                        const TensorDesc* limits,
                                        const TensorDesc* restLengths,
                                        const TensorDesc* offsets,
                                        const TensorDesc* maskTensor) const override;
    /**
     * @brief Sets the spatial tendon properties for the elements selected by a boolean mask.
     * @param[in] stiffnesses Tensor of tendon stiffness values.
     * @param[in] dampings Tensor of tendon damping values.
     * @param[in] limitStiffnesses Tensor of tendon limit stiffness values.
     * @param[in] offsets Tensor of tendon offset values.
     * @param[in] maskTensor Boolean mask tensor selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setSpatialTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                          const TensorDesc* dampings,
                                          const TensorDesc* limitStiffnesses,
                                          const TensorDesc* offsets,
                                          const TensorDesc* maskTensor) const override;

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
    /// Notifies the Newton model that joint DOF properties have been modified externally.
    void _notifyJointDofPropertiesChanged();
    /// Syncs CTRL_DIRECT actuator gains from the cached stiffness/damping arrays.
    void _syncCtrlDirectActuatorGains();
    /// Syncs CTRL_DIRECT position targets from the cached target array.
    void _syncCtrlDirectPositionTargets();
    /// Runs Newton forward kinematics to propagate joint_q changes to body_q.
    void _evalForwardKinematics();
    /// Runs Newton inverse kinematics to propagate body_q/body_qd changes back to joint_q/joint_qd.
    void _evalInverseKinematics();
    /// Recomputes inverse mass array from the forward mass array.
    void _updateInverseMasses();
    /// Recomputes inverse inertia array from the forward inertia array.
    void _updateInverseInertias();

    /// Resolves an index tensor to a list of articulation indices, writing into the
    /// pre-allocated m_scratchViewIndices member to avoid per-call allocation.
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
     * @brief Builds the DOF scatter mappings.
     * @param[in] artiIndices Articulation indices selected by the operation.
     * @param[in] dofIndices DOF indices selected by the operation.
     * @param[out] srcOffsets Receives the source offset for each scattered element.
     * @param[out] dstIndices Receives the destination index for each scattered element.
     */
    void _buildDofScatterMappings(const std::vector<uint32_t>& artiIndices,
                                  const std::vector<int>& dofIndices,
                                  std::vector<int>& srcOffsets,
                                  std::vector<int>& dstIndices) const;
    /**
     * @brief Builds the link scatter mappings.
     * @param[in] artiIndices Articulation indices selected by the operation.
     * @param[in] elemSize Number of floats per element.
     * @param[out] srcOffsets Receives the source offset for each scattered element.
     * @param[out] dstIndices Receives the destination index for each scattered element.
     */
    void _buildLinkScatterMappings(const std::vector<uint32_t>& artiIndices,
                                   int elemSize,
                                   std::vector<int>& srcOffsets,
                                   std::vector<int>& dstIndices) const;
    /**
     * @brief Builds the root scatter mappings.
     * @param[in] artiIndices Articulation indices selected by the operation.
     * @param[in] elemSize Number of floats per element.
     * @param[out] srcOffsets Receives the source offset for each scattered element.
     * @param[out] dstIndices Receives the destination index for each scattered element.
     */
    void _buildRootScatterMappings(const std::vector<uint32_t>& artiIndices,
                                   int elemSize,
                                   std::vector<int>& srcOffsets,
                                   std::vector<int>& dstIndices) const;

    py::object m_newtonStage; ///< Newton stage object backing the view.
    py::object m_model; ///< Newton model object backing the view.

    uint32_t m_count; ///< Number of articulations in the view.
    uint32_t m_maxLinks; ///< Maximum link count across all articulations.
    uint32_t m_maxDofs; ///< Maximum DOF count across all articulations.
    uint32_t m_maxShapes; ///< Maximum shape count across all elements in the view.

    std::vector<int> m_articulationIndices; ///< Root body index for each articulation.
    std::vector<int> m_dofPosIndices; ///< [count * maxDofs] → model.joint_q index, -1 = padding.
    std::vector<int> m_dofVelIndices; ///< [count * maxDofs] → model.joint_qd index, -1 = padding.
    std::vector<int> m_dofAxisIndices; ///< [count * maxDofs] → model.joint_axis index, -1 = padding.
    std::vector<int> m_rootBodyIndices; ///< [count] → model body index for each root body.
    std::vector<int> m_rootJointIndices; ///< [count] → model joint index for each root joint.
    std::vector<int> m_rootJointQStartIndices; ///< [count] → joint_q_start offset for root joint.
    std::vector<int> m_fixedRootJointMapping; ///< [count] → rootJointIdx for fixed roots, -1 for free.
    std::vector<std::vector<int>> m_linkIndicesPerArticulation; ///< Model body indices for each link, grouped per
                                                                ///< articulation.
    std::vector<int> m_linkFlatIndices; ///< [count * maxLinks] → model body index, -1 = padding.

    std::vector<std::string> m_articulationPrimPaths; ///< USD prim path of each articulation.
    std::vector<std::vector<std::string>> m_articulationPaths; ///< USD prim paths associated with each articulation.

    std::vector<ArticulationMetatype*> m_metatypes; ///< Per-element articulation metatype objects owned by the view.
    std::vector<uint8_t> m_hostDofTypes; ///< Host-side cache of the DOF type for each DOF.
    std::vector<int> m_rootJointTypes; ///< [count] → joint type for root joint of each articulation.
    std::vector<float> m_cachedComOrientation; ///< [count * maxLinks * 4] cached COM quaternion (xyzw), write-through
                                               ///< from setCOMs.
    std::vector<std::vector<std::string>> m_dofPaths; ///< [count][maxDofs] → USD prim path for each DOF.

    mutable std::vector<uint32_t> m_scratchViewIndices; ///< Scratch buffer reused to resolve view indices without
                                                        ///< per-call allocation.

    bool m_cacheValid = false; ///< True after _cacheWarpPointers() succeeds.
    int m_modelDeviceOrdinal = -1; ///< Device of the Newton model's Warp arrays.
    size_t m_totalBodyCount = 0; ///< Total body count in the Newton model.

    // Cached raw pointers into Warp arrays from state_0 and the model.
    mutable float* m_cachedJointQ = nullptr; ///< Cached pointer to the joint q data.
    mutable float* m_cachedJointQd = nullptr; ///< Cached pointer to the joint qd data.
    mutable float* m_cachedBodyQ = nullptr; ///< Cached pointer to the body q data.
    mutable float* m_cachedBodyQd = nullptr; ///< Cached pointer to the body qd data.
    mutable float* m_cachedBodyF = nullptr; ///< Cached pointer to the body f data.
    float* m_cachedJointTargetKe = nullptr; ///< Cached pointer to the joint target ke data.
    float* m_cachedJointTargetKd = nullptr; ///< Cached pointer to the joint target kd data.
    float* m_cachedJointEffortLimit = nullptr; ///< Cached pointer to the joint effort limit data.
    float* m_cachedJointVelocityLimit = nullptr; ///< Cached pointer to the joint velocity limit data.
    float* m_cachedJointArmature = nullptr; ///< Cached pointer to the joint armature data.
    float* m_cachedJointLimitLower = nullptr; ///< Cached pointer to the joint limit lower data.
    float* m_cachedJointLimitUpper = nullptr; ///< Cached pointer to the joint limit upper data.
    mutable float* m_cachedCtrlTargetPos = nullptr; ///< Cached pointer to the ctrl target pos data.
    mutable float* m_cachedCtrlTargetVel = nullptr; ///< Cached pointer to the ctrl target vel data.
    mutable float* m_cachedJointTorques = nullptr; ///< Cached pointer to the joint torques data.
    float* m_cachedBodyMass = nullptr; ///< Cached pointer to the body mass data.
    float* m_cachedBodyInverseMass = nullptr; ///< Cached pointer to the body inverse mass data.
    float* m_cachedBodyInertia = nullptr; ///< Cached pointer to the body inertia data.
    float* m_cachedBodyInverseInertia = nullptr; ///< Cached pointer to the body inverse inertia data.
    float* m_cachedBodyCenterOfMass = nullptr; ///< Cached pointer to the body center of mass data.
    float* m_cachedJointXp = nullptr; ///< model.joint_X_p — parent transform per joint (7 floats each).

    /// Extracts and caches raw float*/int* pointers from all relevant Warp arrays in the
    /// Newton model and state. Called lazily before the first getter/setter access.
    void _cacheWarpPointers();
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
