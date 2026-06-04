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

#include <omni/physics/tensors/IArticulationMetatype.h>
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

using omni::physics::tensors::DofType;
using omni::physics::tensors::IArticulationMetatype;
using omni::physics::tensors::JointType;

/// Articulation topology descriptor for a single articulation in the view.
///
/// Constructed once per articulation during BaseArticulationView initialization.
/// Extracts and stores all link/joint/DOF names, types, and hierarchy data from the
/// Newton model at construction time so that subsequent queries are pure lookups
/// with no Python interaction.
class ArticulationMetatype : public IArticulationMetatype
{
public:
    /// Build a metatype from a Newton model and the articulation's joint/link subsets.
    ///
    /// @param model Python handle to the ``newton.Model`` instance.
    /// @param jointIndices Model joint indices belonging to this articulation.
    /// @param linkIndices Model body (link) indices belonging to this articulation.
    /// @param linkPaths USD prim paths for each link, parallel to ``linkIndices``.
    /// @param jointPaths USD prim paths for each joint, parallel to ``jointIndices``.
    /// @param firstJointIndex Index of the first (root) joint inside the joint subset,
    ///        used to decide whether the base is fixed.
    ArticulationMetatype(py::object model,
                         const std::vector<int>& jointIndices,
                         const std::vector<int>& linkIndices,
                         const std::vector<std::string>& linkPaths,
                         const std::vector<std::string>& jointPaths,
                         int firstJointIndex);

    /// Number of links in the articulation (including the root).
    uint32_t getLinkCount() const override;
    /// Number of joints in the articulation (including any root free-joint).
    uint32_t getJointCount() const override;
    /// Total number of degrees of freedom across all joints.
    uint32_t getDofCount() const override;

    /// Short name of the link at ``linkIdx`` (final USD prim name).
    const char* getLinkName(uint32_t linkIdx) const override;
    /// Short name of the parent link of ``linkIdx``; empty for the root.
    const char* getLinkParentName(uint32_t linkIdx) const override;
    /// Short name of the joint at ``jointIdx``.
    const char* getJointName(uint32_t jointIdx) const override;
    /// Short name of the DOF at ``dofIdx``.
    const char* getDofName(uint32_t dofIdx) const override;

    /// Look up the link index by short name.
    ///
    /// @return Zero-based index, or ``-1`` if not found.
    int32_t findLinkIndex(const char* linkName) const override;
    /// Look up the index of the parent link of ``linkName``.
    ///
    /// @return Zero-based index, or ``-1`` if the link is not found or has no parent.
    int32_t findLinkParentIndex(const char* linkName) const override;
    /// Look up the joint index by short name.
    ///
    /// @return Zero-based index, or ``-1`` if not found.
    int32_t findJointIndex(const char* jointName) const override;
    /// Look up the DOF index by short name.
    ///
    /// @return Zero-based index, or ``-1`` if not found.
    int32_t findDofIndex(const char* dofName) const override;

    /// Joint kind at ``jointIdx`` (revolute, prismatic, ball, free, fixed, ...).
    JointType getJointType(uint32_t jointIdx) const override;
    /// Offset of the first DOF of joint ``jointIdx`` within the articulation DOF array.
    uint32_t getJointDofOffset(uint32_t jointIdx) const override;
    /// Number of DOFs contributed by joint ``jointIdx``.
    uint32_t getJointDofCount(uint32_t jointIdx) const override;

    /// DOF kind at ``dofIdx`` (translational, rotational, or none).
    DofType getDofType(uint32_t dofIdx) const override;

    /// ``true`` if the root link is attached to the world by a fixed joint.
    bool getFixedBase() const override;

private:
    uint32_t m_linkCount;
    uint32_t m_jointCount;
    uint32_t m_dofCount;
    bool m_isFixedBase;

    std::vector<std::string> m_linkNames;
    std::vector<std::string> m_linkParentNames;
    std::vector<std::string> m_jointNames;
    std::vector<std::string> m_dofNames;
    std::vector<JointType> m_jointTypes;
    std::vector<uint32_t> m_jointDofOffsets;
    std::vector<uint32_t> m_jointDofCounts;
    std::vector<DofType> m_dofTypes;
    std::vector<std::string> m_linkShortNames;
    std::vector<std::string> m_jointShortNames;
    std::vector<std::string> m_dofShortNames;
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
