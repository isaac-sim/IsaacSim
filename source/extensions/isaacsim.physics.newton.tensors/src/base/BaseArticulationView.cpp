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

#include "BaseArticulationView.h"

#include "utils/TensorOps.h"
#include "utils/WarpInterop.h"

#include <carb/logging/Log.h>

#include <pxr/usd/usd/prim.h>
#include <pybind11/numpy.h>

#include <algorithm>
#include <cstring>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

static constexpr int kJointPrismatic = 0;
static constexpr int kJointRevolute = 1;
static constexpr int kJointBall = 2;
static constexpr int kJointFixed = 3;
static constexpr int kJointFree = 4;
static constexpr int kJointDistance = 5;
static constexpr int kJointD6 = 6;

static bool isDofJoint(int jType)
{
    return jType == kJointPrismatic || jType == kJointRevolute || jType == kJointBall || jType == kJointD6;
}

BaseArticulationView::BaseArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& articulationPaths)
    : m_newtonStage(newtonStage), m_count(0), m_maxLinks(0), m_maxDofs(0), m_maxShapes(0)
{

    py::gil_scoped_acquire gil;

    try
    {
        m_model = m_newtonStage.attr("model");

        py::array articulationStart = m_model.attr("articulation_start").attr("numpy")().cast<py::array>();
        py::array jointQStart = m_model.attr("joint_q_start").attr("numpy")().cast<py::array>();
        py::array jointQdStart = m_model.attr("joint_qd_start").attr("numpy")().cast<py::array>();
        py::array jointType = m_model.attr("joint_type").attr("numpy")().cast<py::array>();
        py::array jointChild = m_model.attr("joint_child").attr("numpy")().cast<py::array>();

        py::list jointLabel = m_model.attr("joint_label").cast<py::list>();
        py::list bodyLabel = m_model.attr("body_label").cast<py::list>();
        py::list articulationLabel = m_model.attr("articulation_label").cast<py::list>();
        py::object bodyShapes = m_model.attr("body_shapes");
        int articulationCount = m_model.attr("articulation_count").cast<int>();

        auto artStarts = articulationStart.unchecked<int, 1>();
        auto jointQStarts = jointQStart.unchecked<int, 1>();
        auto jointQdStarts = jointQdStart.unchecked<int, 1>();
        auto jointTypes = jointType.unchecked<int, 1>();
        auto jointChildren = jointChild.unchecked<int, 1>();

        std::vector<int> matchedArticulationIndices;
        for (int artiIdx = 0; artiIdx < articulationCount; ++artiIdx)
        {
            std::string artiPath = py::str(articulationLabel[artiIdx]);
            for (size_t pi = 0; pi < articulationPaths.size(); ++pi)
            {
                if (articulationPaths[pi].GetString() == artiPath)
                {
                    matchedArticulationIndices.push_back(artiIdx);
                    m_articulationPrimPaths.push_back(artiPath);
                    break;
                }
            }
        }

        m_count = matchedArticulationIndices.size();

        struct ArtiInfo
        {
            int jointStart, jointEnd, rootBodyIdx;
            std::vector<int> linkIndices;
            std::vector<std::string> linkPaths;
            std::vector<int> jointIndices;
            std::vector<std::string> jointPaths;
            int totalDofs;
        };
        std::vector<ArtiInfo> artiInfos(m_count);

        for (uint32_t a = 0; a < m_count; ++a)
        {
            int artiIdx = matchedArticulationIndices[a];
            auto& info = artiInfos[a];
            info.jointStart = artStarts(artiIdx);
            info.jointEnd = artStarts(artiIdx + 1);
            int jointCount = info.jointEnd - info.jointStart;

            info.rootBodyIdx = jointChildren(info.jointStart);
            m_rootBodyIndices.push_back(info.rootBodyIdx);
            m_rootJointIndices.push_back(info.jointStart);
            int rootType = jointTypes(info.jointStart);
            m_rootJointTypes.push_back(rootType);
            m_rootJointQStartIndices.push_back(rootType == kJointFree ? jointQStarts(info.jointStart) : -1);
            m_fixedRootJointMapping.push_back(rootType != kJointFree ? info.jointStart : -1);
            m_articulationIndices.push_back(artiIdx);

            info.totalDofs = 0;

            for (int j = 0; j < jointCount; ++j)
            {
                int gj = info.jointStart + j;
                int childBody = jointChildren(gj);
                info.linkIndices.push_back(childBody);
                info.linkPaths.push_back(py::str(bodyLabel[childBody]));

                if (j == 0)
                    continue;

                int jType = jointTypes(gj);
                if (isDofJoint(jType))
                {
                    int qdStart = jointQdStarts(gj);
                    int qdEnd = jointQdStarts(gj + 1);
                    info.totalDofs += (qdEnd - qdStart);
                    info.jointIndices.push_back(gj);
                    info.jointPaths.push_back(py::str(jointLabel[gj]));
                }
            }

            uint32_t totalShapes = 0;
            for (int bodyIdx : info.linkIndices)
            {
                py::object shapes = bodyShapes[py::int_(bodyIdx)];
                totalShapes += static_cast<uint32_t>(py::len(shapes));
            }
            m_maxShapes = std::max(m_maxShapes, totalShapes);
            m_maxDofs = std::max(m_maxDofs, (uint32_t)info.totalDofs);
            m_maxLinks = std::max(m_maxLinks, (uint32_t)info.linkIndices.size());
        }

        m_dofPosIndices.resize(m_count * m_maxDofs, -1);
        m_dofVelIndices.resize(m_count * m_maxDofs, -1);
        m_dofAxisIndices.resize(m_count * m_maxDofs, -1);
        m_linkFlatIndices.resize(m_count * m_maxLinks, -1);

        for (uint32_t a = 0; a < m_count; ++a)
        {
            auto& info = artiInfos[a];

            for (uint32_t j = 0; j < info.linkIndices.size(); ++j)
            {
                m_linkFlatIndices[a * m_maxLinks + j] = info.linkIndices[j];
            }
            m_linkIndicesPerArticulation.push_back(info.linkIndices);

            int dofOff = 0;
            for (int j = 1; j < info.jointEnd - info.jointStart; ++j)
            {
                int gj = info.jointStart + j;
                int jType = jointTypes(gj);
                if (!isDofJoint(jType))
                    continue;

                int qStart = jointQStarts(gj);
                int qEnd = jointQStarts(gj + 1);

                for (int c = 0; c < qEnd - qStart; ++c)
                {
                    if (dofOff < (int)m_maxDofs)
                        m_dofPosIndices[a * m_maxDofs + dofOff] = qStart + c;
                    dofOff++;
                }
            }

            dofOff = 0;
            for (int j = 1; j < info.jointEnd - info.jointStart; ++j)
            {
                int gj = info.jointStart + j;
                int jType = jointTypes(gj);
                if (!isDofJoint(jType))
                    continue;

                int qdStart = jointQdStarts(gj);
                int qdEnd = jointQdStarts(gj + 1);

                for (int c = 0; c < qdEnd - qdStart; ++c)
                {
                    if (dofOff < (int)m_maxDofs)
                    {
                        m_dofVelIndices[a * m_maxDofs + dofOff] = qdStart + c;
                        m_dofAxisIndices[a * m_maxDofs + dofOff] = qdStart + c;
                    }
                    dofOff++;
                }
            }

            m_metatypes.push_back(new ArticulationMetatype(
                m_model, info.jointIndices, info.linkIndices, info.linkPaths, info.jointPaths, info.jointStart));
            m_articulationPaths.push_back(info.linkPaths);

            std::vector<std::string> dofPathsForArti(m_maxDofs, "");
            {
                uint32_t dofIdx = 0;
                for (size_t ji = 0; ji < info.jointIndices.size() && dofIdx < m_maxDofs; ++ji)
                {
                    int gj = info.jointIndices[ji];
                    int qdStart = jointQdStarts(gj);
                    int qdEnd = jointQdStarts(gj + 1);
                    std::string path = (ji < info.jointPaths.size()) ? info.jointPaths[ji] : "";
                    for (int c = 0; c < qdEnd - qdStart && dofIdx < m_maxDofs; ++c, ++dofIdx)
                        dofPathsForArti[dofIdx] = path;
                }
            }
            m_dofPaths.push_back(std::move(dofPathsForArti));
        }

        m_cachedComOrientation.resize(m_count * m_maxLinks * 4, 0.0f);
        for (uint32_t i = 0; i < m_count * m_maxLinks; ++i)
            m_cachedComOrientation[i * 4 + 3] = 1.0f;

        if (!m_metatypes.empty())
        {
            uint32_t n = m_count * m_maxDofs;
            m_hostDofTypes.resize(n, 0);
            for (uint32_t a = 0; a < m_count; ++a)
            {
                auto* mt = m_metatypes[a];
                for (uint32_t d = 0; d < mt->getDofCount() && d < m_maxDofs; ++d)
                    m_hostDofTypes[a * m_maxDofs + d] = static_cast<uint8_t>(mt->getDofType(d));
            }
        }

        if (m_count > 0)
            _cacheWarpPointers();
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to initialize BaseArticulationView: %s", e.what());
    }
}

void BaseArticulationView::_cacheWarpPointers()
{
    auto ptr = [](py::object arr) -> float* { return warpArrayFromPython<float>(arr).data; };
    py::object state0 = m_newtonStage.attr("state_0");
    py::object control = m_newtonStage.attr("control");

    m_cachedJointQ = ptr(state0.attr("joint_q"));
    m_cachedJointQd = ptr(state0.attr("joint_qd"));
    m_cachedBodyQ = ptr(state0.attr("body_q"));
    m_cachedBodyQd = ptr(state0.attr("body_qd"));
    m_cachedBodyF = ptr(state0.attr("body_f"));

    m_cachedJointTargetKe = ptr(m_model.attr("joint_target_ke"));
    m_cachedJointTargetKd = ptr(m_model.attr("joint_target_kd"));
    m_cachedJointEffortLimit = ptr(m_model.attr("joint_effort_limit"));
    m_cachedJointVelocityLimit = ptr(m_model.attr("joint_velocity_limit"));
    m_cachedJointArmature = ptr(m_model.attr("joint_armature"));
    m_cachedJointLimitLower = ptr(m_model.attr("joint_limit_lower"));
    m_cachedJointLimitUpper = ptr(m_model.attr("joint_limit_upper"));
    m_cachedBodyMass = ptr(m_model.attr("body_mass"));
    m_cachedBodyInverseMass = ptr(m_model.attr("body_inv_mass"));
    m_cachedBodyInertia = ptr(m_model.attr("body_inertia"));
    m_cachedBodyInverseInertia = ptr(m_model.attr("body_inv_inertia"));
    m_cachedBodyCenterOfMass = ptr(m_model.attr("body_com"));
    m_cachedJointXp = ptr(m_model.attr("joint_X_p"));

    m_cachedCtrlTargetPos = ptr(control.attr("joint_target_pos"));
    m_cachedCtrlTargetVel = ptr(control.attr("joint_target_vel"));
    m_cachedJointTorques = ptr(control.attr("joint_f"));

    py::object bodyMassArr = m_model.attr("body_mass");
    m_modelDeviceOrdinal = getWarpArrayDevice(bodyMassArr);
    m_totalBodyCount = getWarpArraySize(bodyMassArr);
    m_cacheValid = (m_cachedBodyQ != nullptr);
}

BaseArticulationView::~BaseArticulationView()
{
    for (auto mt : m_metatypes)
        delete mt;
}

uint32_t BaseArticulationView::getCount() const
{
    return m_count;
}
uint32_t BaseArticulationView::getMaxLinks() const
{
    return m_maxLinks;
}
uint32_t BaseArticulationView::getMaxDofs() const
{
    return m_maxDofs;
}
uint32_t BaseArticulationView::getMaxShapes() const
{
    return m_maxShapes;
}
uint32_t BaseArticulationView::getMaxFixedTendons() const
{
    return 0;
}
uint32_t BaseArticulationView::getMaxSpatialTendons() const
{
    return 0;
}

bool BaseArticulationView::isHomogeneous() const
{
    if (m_metatypes.size() <= 1)
        return true;
    uint32_t refDofs = m_metatypes[0]->getDofCount();
    uint32_t refLinks = m_metatypes[0]->getLinkCount();
    for (size_t i = 1; i < m_metatypes.size(); ++i)
    {
        if (m_metatypes[i]->getDofCount() != refDofs || m_metatypes[i]->getLinkCount() != refLinks)
            return false;
    }
    return true;
}

const IArticulationMetatype* BaseArticulationView::getSharedMetatype() const
{
    return m_metatypes.empty() ? nullptr : m_metatypes[0];
}

const IArticulationMetatype* BaseArticulationView::getMetatype(uint32_t artiIdx) const
{
    if (isHomogeneous())
        return m_metatypes.empty() ? nullptr : m_metatypes[0];
    return (artiIdx < m_metatypes.size()) ? m_metatypes[artiIdx] : nullptr;
}

const char* BaseArticulationView::getUsdPrimPath(uint32_t artiIdx) const
{
    if (artiIdx < m_articulationPrimPaths.size())
        return m_articulationPrimPaths[artiIdx].c_str();
    return "";
}
const char* BaseArticulationView::getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const
{
    if (artiIdx < m_dofPaths.size() && dofIdx < m_dofPaths[artiIdx].size())
        return m_dofPaths[artiIdx][dofIdx].c_str();
    return "";
}

const char* BaseArticulationView::getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const
{
    if (artiIdx < m_articulationPaths.size() && linkIdx < m_articulationPaths[artiIdx].size())
        return m_articulationPaths[artiIdx][linkIdx].c_str();
    return "";
}

void BaseArticulationView::_buildDofScatterMappings(const std::vector<uint32_t>& artiIndices,
                                                    const std::vector<int>& dofIndices,
                                                    std::vector<int>& srcOffsets,
                                                    std::vector<int>& dstIndices) const
{
    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        for (uint32_t j = 0; j < m_maxDofs; ++j)
        {
            uint32_t flatIdx = idx * m_maxDofs + j;
            int warpIdx = dofIndices[flatIdx];
            if (warpIdx >= 0)
            {
                srcOffsets.push_back(static_cast<int>(idx * m_maxDofs + j));
                dstIndices.push_back(warpIdx);
            }
        }
    }
}

void BaseArticulationView::_buildLinkScatterMappings(const std::vector<uint32_t>& artiIndices,
                                                     int elemSize,
                                                     std::vector<int>& srcOffsets,
                                                     std::vector<int>& dstIndices) const
{
    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        const auto& links = m_linkIndicesPerArticulation[idx];
        for (uint32_t j = 0; j < m_maxLinks; ++j)
        {
            if (j < links.size())
            {
                for (int e = 0; e < elemSize; ++e)
                {
                    srcOffsets.push_back(static_cast<int>((idx * m_maxLinks + j) * elemSize + e));
                    dstIndices.push_back(links[j] * elemSize + e);
                }
            }
        }
    }
}

void BaseArticulationView::_buildRootScatterMappings(const std::vector<uint32_t>& artiIndices,
                                                     int elemSize,
                                                     std::vector<int>& srcOffsets,
                                                     std::vector<int>& dstIndices) const
{
    for (uint32_t idx : artiIndices)
    {
        if (idx >= m_count)
            continue;
        for (int e = 0; e < elemSize; ++e)
        {
            srcOffsets.push_back(static_cast<int>(idx * elemSize + e));
            dstIndices.push_back(m_rootBodyIndices[idx] * elemSize + e);
        }
    }
}

void BaseArticulationView::_notifyJointDofPropertiesChanged()
{
    try
    {
        py::object solver = m_newtonStage.attr("solver");
        if (!solver.is_none())
        {
            py::module_ newton_solvers = py::module_::import("newton.solvers");
            py::object flags = newton_solvers.attr("SolverNotifyFlags").attr("JOINT_DOF_PROPERTIES");
            solver.attr("notify_model_changed")(flags);
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_WARN("_notifyJointDofPropertiesChanged failed: %s", e.what());
    }
}

void BaseArticulationView::_syncCtrlDirectActuatorGains()
{
    try
    {
        py::module_ mod = py::module_::import("isaacsim.physics.newton.tensors.impl.ctrl_direct_sync");
        mod.attr("sync_actuator_gains")(m_newtonStage, m_model);
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_WARN("_syncCtrlDirectActuatorGains failed: %s", e.what());
    }
}

void BaseArticulationView::_syncCtrlDirectPositionTargets()
{
    try
    {
        py::module_ mod = py::module_::import("isaacsim.physics.newton.tensors.impl.ctrl_direct_sync");
        mod.attr("sync_position_targets")(m_newtonStage, m_model);
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_WARN("_syncCtrlDirectPositionTargets failed: %s", e.what());
    }
}

void BaseArticulationView::_evalForwardKinematics()
{
    py::gil_scoped_acquire gil;
    py::object state = m_newtonStage.attr("state_0");
    py::module_ newton_mod = py::module_::import("newton");
    newton_mod.attr("eval_fk")(m_model, state.attr("joint_q"), state.attr("joint_qd"), state);
}

void BaseArticulationView::_evalInverseKinematics()
{
    py::gil_scoped_acquire gil;
    py::object state = m_newtonStage.attr("state_0");
    py::module_ newton_mod = py::module_::import("newton");
    newton_mod.attr("eval_ik")(m_model, state, state.attr("joint_q"), state.attr("joint_qd"));
}

void BaseArticulationView::_updateInverseMasses()
{
    if (!m_cachedBodyMass || !m_cachedBodyInverseMass || m_totalBodyCount == 0)
        return;
    if (!updateInverseMass(m_cachedBodyMass, m_cachedBodyInverseMass, m_modelDeviceOrdinal, m_totalBodyCount))
        CARB_LOG_WARN("Failed to update inverse masses");
}

void BaseArticulationView::_updateInverseInertias()
{
    if (!m_cachedBodyInertia || !m_cachedBodyInverseInertia || m_totalBodyCount == 0)
        return;
    if (!updateInverseInertia(m_cachedBodyInertia, m_cachedBodyInverseInertia, m_modelDeviceOrdinal, m_totalBodyCount))
        CARB_LOG_WARN("Failed to update inverse inertias");
}

// ---- Stubs ----

bool BaseArticulationView::getLinkAccelerations(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDofProjectedJointForces(const TensorDesc*) const
{
    CARB_LOG_WARN_ONCE("getDofProjectedJointForces is not implemented for the Newton backend");
    return false;
}
bool BaseArticulationView::getDofMotions(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDriveTypes(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDofDriveModelProperties(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDofFrictionCoefficients(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDofFrictionProperties(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getDisableGravities(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setDisableGravities(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDisableGravitiesMasked(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::getArticulationMassCenter(const TensorDesc*, bool) const
{
    return false;
}
bool BaseArticulationView::getArticulationCentroidalMomentum(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getJacobianShape(uint32_t*, uint32_t*) const
{
    return false;
}
bool BaseArticulationView::getJacobians(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getGeneralizedMassMatrixShape(uint32_t*, uint32_t*) const
{
    return false;
}
bool BaseArticulationView::getGeneralizedMassMatrices(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getCoriolisAndCentrifugalCompensationForces(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getGravityCompensationForces(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getLinkIncomingJointForce(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setDofDriveModelProperties(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDofFrictionCoefficients(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDofFrictionProperties(const TensorDesc*, const TensorDesc*)
{
    return false;
}

// ---- Masked variants ----

bool BaseArticulationView::setDofLimitsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofLimits(s, nullptr);
}
bool BaseArticulationView::setDofStiffnessesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofStiffnesses(s, nullptr);
}
bool BaseArticulationView::setDofDampingsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofDampings(s, nullptr);
}
bool BaseArticulationView::setDofMaxForcesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofMaxForces(s, nullptr);
}
bool BaseArticulationView::setDofDriveModelPropertiesMasked(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDofFrictionCoefficientsMasked(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDofFrictionPropertiesMasked(const TensorDesc*, const TensorDesc*)
{
    return false;
}
bool BaseArticulationView::setDofMaxVelocitiesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofMaxVelocities(s, nullptr);
}
bool BaseArticulationView::setDofArmaturesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofArmatures(s, nullptr);
}
bool BaseArticulationView::setDofPositionsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofPositions(s, nullptr);
}
bool BaseArticulationView::setDofVelocitiesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofVelocities(s, nullptr);
}
bool BaseArticulationView::setDofActuationForcesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofActuationForces(s, nullptr);
}
bool BaseArticulationView::setDofPositionTargetsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofPositionTargets(s, nullptr);
}
bool BaseArticulationView::setDofVelocityTargetsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setDofVelocityTargets(s, nullptr);
}
bool BaseArticulationView::setMassesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setMasses(s, nullptr);
}
bool BaseArticulationView::setCOMsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setCOMs(s, nullptr);
}
bool BaseArticulationView::setInertiasMasked(const TensorDesc* s, const TensorDesc*)
{
    return setInertias(s, nullptr);
}
bool BaseArticulationView::setRootTransformsMasked(const TensorDesc* s, const TensorDesc*)
{
    return setRootTransforms(s, nullptr);
}
bool BaseArticulationView::setRootVelocitiesMasked(const TensorDesc* s, const TensorDesc*)
{
    return setRootVelocities(s, nullptr);
}
bool BaseArticulationView::applyForcesAndTorquesAtPositionMasked(
    const TensorDesc* f, const TensorDesc* t, const TensorDesc* p, const TensorDesc* /*mask*/, bool g)
{
    return applyForcesAndTorquesAtPosition(f, t, p, nullptr, g);
}

// ---- Material/shape stubs ----

bool BaseArticulationView::getMaterialProperties(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getCompliantMaterialProperties(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getRestOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getContactOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setMaterialProperties(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setCompliantMaterialProperties(const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setRestOffsets(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setContactOffsets(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setMaterialPropertiesMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setCompliantMaterialPropertiesMasked(const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setRestOffsetsMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setContactOffsetsMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}

// ---- Tendon stubs ----

bool BaseArticulationView::getFixedTendonStiffnesses(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getFixedTendonDampings(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getFixedTendonLimitStiffnesses(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getFixedTendonLimits(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getFixedTendonfixedSpringRestLengths(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getFixedTendonOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getSpatialTendonStiffnesses(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getSpatialTendonDampings(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getSpatialTendonLimitStiffnesses(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::getSpatialTendonOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setFixedTendonProperties(const TensorDesc*,
                                                    const TensorDesc*,
                                                    const TensorDesc*,
                                                    const TensorDesc*,
                                                    const TensorDesc*,
                                                    const TensorDesc*,
                                                    const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setSpatialTendonProperties(
    const TensorDesc*, const TensorDesc*, const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setFixedTendonPropertiesMasked(const TensorDesc*,
                                                          const TensorDesc*,
                                                          const TensorDesc*,
                                                          const TensorDesc*,
                                                          const TensorDesc*,
                                                          const TensorDesc*,
                                                          const TensorDesc*) const
{
    return false;
}
bool BaseArticulationView::setSpatialTendonPropertiesMasked(
    const TensorDesc*, const TensorDesc*, const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}

bool BaseArticulationView::check() const
{
    return m_count > 0;
}
void BaseArticulationView::release()
{
    delete this;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
