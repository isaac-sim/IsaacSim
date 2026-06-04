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

#include "BaseRigidBodyView.h"

#include "utils/TensorOps.h"
#include "utils/WarpInterop.h"

#include <carb/logging/Log.h>

#include <pybind11/numpy.h>

#include <algorithm>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

BaseRigidBodyView::BaseRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& bodyPaths)
    : m_newtonStage(newtonStage), m_count(0), m_maxShapes(0)
{

    py::gil_scoped_acquire gil;

    try
    {
        m_model = m_newtonStage.attr("model");
        py::list bodyLabel = m_model.attr("body_label").cast<py::list>();
        py::object bodyShapes = m_model.attr("body_shapes");

        py::array jointChildArr = m_model.attr("joint_child").attr("numpy")().cast<py::array>();
        py::array jointTypeArr = m_model.attr("joint_type").attr("numpy")().cast<py::array>();
        py::array jointQStartArr = m_model.attr("joint_q_start").attr("numpy")().cast<py::array>();
        auto jointChildren = jointChildArr.unchecked<int, 1>();
        auto jointTypes = jointTypeArr.unchecked<int, 1>();
        auto jointQStarts = jointQStartArr.unchecked<int, 1>();
        int numJoints = static_cast<int>(jointTypeArr.shape(0));

        for (size_t pi = 0; pi < bodyPaths.size(); ++pi)
        {
            const auto& path = bodyPaths[pi];
            std::string pathStr = path.GetString();
            int bodyIdx = -1;
            for (size_t i = 0; i < static_cast<size_t>(py::len(bodyLabel)); ++i)
            {
                if (py::str(bodyLabel[i]).cast<std::string>() == pathStr)
                {
                    bodyIdx = static_cast<int>(i);
                    break;
                }
            }
            if (bodyIdx < 0)
            {
                CARB_LOG_WARN("Rigid body path '%s' not found in Newton model", pathStr.c_str());
                continue;
            }
            m_bodyIndices.push_back(bodyIdx);
            m_primPaths.push_back(pathStr);

            int qStart = -1;
            for (int j = 0; j < numJoints; ++j)
            {
                if (jointChildren(j) == bodyIdx && jointTypes(j) == 4)
                {
                    qStart = jointQStarts(j);
                    break;
                }
            }
            m_freeJointQStartIndices.push_back(qStart);

            py::object shapes = bodyShapes[py::int_(bodyIdx)];
            uint32_t shapeCount = static_cast<uint32_t>(py::len(shapes));
            m_maxShapes = std::max(m_maxShapes, shapeCount);
        }

        m_count = static_cast<uint32_t>(m_bodyIndices.size());

        m_cachedComOrientation.resize(m_count * 4, 0.0f);
        for (uint32_t i = 0; i < m_count; ++i)
            m_cachedComOrientation[i * 4 + 3] = 1.0f; // identity quat xyzw = (0,0,0,1)

        if (m_count > 0)
            _cacheWarpPointers();
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to initialize BaseRigidBodyView: %s", e.what());
    }
}

void BaseRigidBodyView::_cacheWarpPointers()
{
    auto ptr = [](py::object arr) -> float* { return warpArrayFromPython<float>(arr).data; };
    py::object state0 = m_newtonStage.attr("state_0");

    m_cachedJointQ = ptr(state0.attr("joint_q"));
    m_cachedBodyQ = ptr(state0.attr("body_q"));
    m_cachedBodyQd = ptr(state0.attr("body_qd"));
    m_cachedBodyF = ptr(state0.attr("body_f"));

    py::object bodyQdd = state0.attr("body_qdd");
    m_cachedBodyQdd = bodyQdd.is_none() ? nullptr : ptr(bodyQdd);

    py::object bodyMassArr = m_model.attr("body_mass");
    m_cachedBodyMass = ptr(bodyMassArr);
    m_cachedBodyInverseMass = ptr(m_model.attr("body_inv_mass"));
    m_cachedBodyInertia = ptr(m_model.attr("body_inertia"));
    m_cachedBodyInverseInertia = ptr(m_model.attr("body_inv_inertia"));
    m_cachedBodyCenterOfMass = ptr(m_model.attr("body_com"));

    m_modelDeviceOrdinal = getWarpArrayDevice(bodyMassArr);
    m_totalBodyCount = getWarpArraySize(bodyMassArr);
    m_cacheValid = (m_cachedBodyQ != nullptr);
}

BaseRigidBodyView::~BaseRigidBodyView()
{
}

uint32_t BaseRigidBodyView::getCount() const
{
    return m_count;
}
uint32_t BaseRigidBodyView::getMaxShapes() const
{
    return m_maxShapes;
}

const char* BaseRigidBodyView::getUsdPrimPath(uint32_t rbIdx) const
{
    if (rbIdx < m_primPaths.size())
        return m_primPaths[rbIdx].c_str();
    return "";
}

void BaseRigidBodyView::_buildScatterMappings(const std::vector<uint32_t>& viewIndices,
                                              int elemSize,
                                              std::vector<int>& srcOffsets,
                                              std::vector<int>& dstIndices) const
{
    for (uint32_t idx : viewIndices)
    {
        if (idx >= m_count)
            continue;
        int bodyIdx = m_bodyIndices[idx];
        for (int e = 0; e < elemSize; ++e)
        {
            srcOffsets.push_back(static_cast<int>(idx * elemSize + e));
            dstIndices.push_back(bodyIdx * elemSize + e);
        }
    }
}

void BaseRigidBodyView::_evalForwardKinematics()
{
    py::gil_scoped_acquire gil;
    py::object state = m_newtonStage.attr("state_0");
    py::module_ newton_mod = py::module_::import("newton");
    newton_mod.attr("eval_fk")(m_model, state.attr("joint_q"), state.attr("joint_qd"), state);
}

void BaseRigidBodyView::_evalInverseKinematics()
{
    py::gil_scoped_acquire gil;
    py::object state = m_newtonStage.attr("state_0");
    py::module_ newton_mod = py::module_::import("newton");
    newton_mod.attr("eval_ik")(m_model, state, state.attr("joint_q"), state.attr("joint_qd"));
}

void BaseRigidBodyView::_updateInverseMasses()
{
    if (!m_cachedBodyMass || !m_cachedBodyInverseMass || m_totalBodyCount == 0)
        return;
    if (!updateInverseMass(m_cachedBodyMass, m_cachedBodyInverseMass, m_modelDeviceOrdinal, m_totalBodyCount))
        CARB_LOG_WARN("Failed to update inverse masses");
}

void BaseRigidBodyView::_updateInverseInertias()
{
    if (!m_cachedBodyInertia || !m_cachedBodyInverseInertia || m_totalBodyCount == 0)
        return;
    if (!updateInverseInertia(m_cachedBodyInertia, m_cachedBodyInverseInertia, m_modelDeviceOrdinal, m_totalBodyCount))
        CARB_LOG_WARN("Failed to update inverse inertias");
}

// ---- Stubs ----

bool BaseRigidBodyView::setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return false;
}
bool BaseRigidBodyView::setKinematicTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return false;
}
bool BaseRigidBodyView::getDisableGravities(const TensorDesc* dstTensor) const
{
    return false;
}
bool BaseRigidBodyView::getDisableSimulations(const TensorDesc* dstTensor) const
{
    return false;
}
bool BaseRigidBodyView::setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return false;
}
bool BaseRigidBodyView::setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    return false;
}
bool BaseRigidBodyView::setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return false;
}
bool BaseRigidBodyView::setDisableSimulationsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor)
{
    return false;
}
bool BaseRigidBodyView::wakeUp(const TensorDesc* indexTensor)
{
    return false;
}

bool BaseRigidBodyView::setTransformsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return setTransforms(src, nullptr);
}
bool BaseRigidBodyView::setVelocitiesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return setVelocities(src, nullptr);
}
bool BaseRigidBodyView::applyForcesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return applyForces(src, nullptr);
}
bool BaseRigidBodyView::applyForcesAndTorquesAtPositionMasked(
    const TensorDesc* f, const TensorDesc* t, const TensorDesc* p, const TensorDesc* /*mask*/, bool g)
{
    return applyForcesAndTorquesAtPosition(f, t, p, nullptr, g);
}
bool BaseRigidBodyView::setMassesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return setMasses(src, nullptr);
}
bool BaseRigidBodyView::setCOMsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return setCOMs(src, nullptr);
}
bool BaseRigidBodyView::setInertiasMasked(const TensorDesc* src, const TensorDesc* mask)
{
    return setInertias(src, nullptr);
}

bool BaseRigidBodyView::getMaterialProperties(const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::getCompliantMaterialProperties(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::getRestOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::getContactOffsets(const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setMaterialProperties(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setCompliantMaterialProperties(const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setRestOffsets(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setContactOffsets(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setMaterialPropertiesMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setCompliantMaterialPropertiesMasked(const TensorDesc*, const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setRestOffsetsMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}
bool BaseRigidBodyView::setContactOffsetsMasked(const TensorDesc*, const TensorDesc*) const
{
    return false;
}

bool BaseRigidBodyView::check() const
{
    return m_count > 0;
}
void BaseRigidBodyView::release()
{
    delete this;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
