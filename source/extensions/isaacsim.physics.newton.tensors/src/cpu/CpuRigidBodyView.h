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

#include "base/BaseRigidBodyView.h"

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

/// CPU rigid body view. Implements getters/setters using host-side gather/scatter loops.
class CpuRigidBodyView : public BaseRigidBodyView
{
public:
    /**
     * @brief Constructs a CpuRigidBodyView.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] bodyPaths USD prim paths of the rigid bodies.
     */
    CpuRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& bodyPaths);
    ~CpuRigidBodyView() override = default;

    /**
     * @brief Gets the transforms.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getTransforms(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the velocities.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getVelocities(const TensorDesc* dstTensor) const override;
    /**
     * @brief Gets the accelerations.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getAccelerations(const TensorDesc* dstTensor) const override;
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
     * @brief Gets the COMs.
     * @param[out] dstTensor Destination tensor that receives the requested values.
     * @return True on success; false otherwise.
     */
    bool getCOMs(const TensorDesc* dstTensor) const override;
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
     * @brief Sets the transforms.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the velocities.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the masses.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the COMs.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    /**
     * @brief Sets the inertias.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    /**
     * @brief Applies the forces.
     * @param[in] srcTensor Source tensor providing the values to write.
     * @param[in] indexTensor Tensor of view indices selecting which elements to write.
     * @return True on success; false otherwise.
     */
    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
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
    mutable std::vector<int> m_scratchSourceOffset;
    mutable std::vector<int> m_scratchDestinationIndex;
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
