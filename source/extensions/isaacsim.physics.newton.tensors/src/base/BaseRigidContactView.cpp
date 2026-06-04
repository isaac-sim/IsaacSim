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

#include "BaseRigidContactView.h"

#include "utils/TensorOps.h"
#include "utils/WarpInterop.h"

#include <carb/logging/Log.h>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

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

BaseRigidContactView::BaseRigidContactView(py::object newtonStage,
                                           const std::vector<std::string>& sensorPaths,
                                           const std::vector<std::vector<std::string>>& filterPaths,
                                           uint32_t maxContactDataCount)
    : m_newtonStage(newtonStage),
      m_maxContactDataCount(maxContactDataCount > 0 ? maxContactDataCount : 1000),
      m_sensorPaths(sensorPaths),
      m_filterPaths(filterPaths)
{
    py::gil_scoped_acquire gil;

    m_model = m_newtonStage.attr("model");

    m_physicsDt = m_newtonStage.attr("sim_dt").cast<float>();
    try
    {
        py::dict sysModules = py::module_::import("sys").attr("modules").cast<py::dict>();
        if (sysModules.contains("isaacsim.core.simulation_manager"))
        {
            py::object cls = sysModules["isaacsim.core.simulation_manager"].attr("SimulationManager");
            std::string engine = cls.attr("get_active_physics_engine")().cast<std::string>();
            if (engine == "newton")
                m_physicsDt = cls.attr("get_physics_dt")().cast<float>();
        }
    }
    catch (const py::error_already_set&)
    {
    }

    int bodyCount = m_model.attr("body_count").cast<int>();
    m_worldBodyIndex = bodyCount;
    m_bodyCount = bodyCount + 1;
    m_rigidContactMax = m_model.attr("rigid_contact_max").cast<int>();

    py::list bodyLabel = m_model.attr("body_label").cast<py::list>();
    m_bodyLabels.reserve(static_cast<size_t>(py::len(bodyLabel)));
    for (auto item : bodyLabel)
        m_bodyLabels.push_back(py::str(item));

    std::vector<std::string> shapeLabels;
    std::vector<int> shapeBodyArr;
    if (py::hasattr(m_model, "shape_label"))
    {
        py::list sl = m_model.attr("shape_label").cast<py::list>();
        for (auto item : sl)
            shapeLabels.push_back(py::str(item));
    }
    if (py::hasattr(m_model, "shape_body"))
    {
        py::object sb = m_model.attr("shape_body");
        py::array_t<int> sbNp = sb.attr("numpy")().cast<py::array_t<int>>();
        auto sbData = sbNp.unchecked<1>();
        shapeBodyArr.resize(sbData.shape(0));
        for (py::ssize_t i = 0; i < sbData.shape(0); ++i)
            shapeBodyArr[i] = sbData(i);
    }

    m_sensorCount = static_cast<uint32_t>(m_sensorPaths.size());
    m_hostBodySensorMap.assign(m_bodyCount, -1);

    m_sensorNames.reserve(m_sensorCount);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        pxr::SdfPath sdfPath(m_sensorPaths[si]);
        m_sensorNames.push_back(sdfPath.GetName());

        for (size_t bi = 0; bi < m_bodyLabels.size(); ++bi)
        {
            if (m_bodyLabels[bi] == m_sensorPaths[si])
            {
                m_hostBodySensorMap[bi] = static_cast<int>(si);
                break;
            }
        }
    }

    m_filterCount = 0;
    for (const auto& fp : m_filterPaths)
    {
        if (fp.size() > m_filterCount)
            m_filterCount = static_cast<uint32_t>(fp.size());
    }

    m_filterNames.resize(m_sensorCount);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        if (si < m_filterPaths.size())
        {
            for (const auto& fp : m_filterPaths[si])
            {
                pxr::SdfPath sdfPath(fp);
                m_filterNames[si].push_back(sdfPath.GetName());
            }
        }
    }

    m_hostBodyFilterMap.assign(static_cast<size_t>(m_sensorCount) * m_bodyCount, -1);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        if (si >= m_filterPaths.size())
            continue;
        for (uint32_t fi = 0; fi < static_cast<uint32_t>(m_filterPaths[si].size()); ++fi)
        {
            const std::string& filterPath = m_filterPaths[si][fi];
            if (filterPath.empty())
                continue;

            bool resolved = false;
            for (size_t bi = 0; bi < m_bodyLabels.size(); ++bi)
            {
                if (m_bodyLabels[bi] == filterPath)
                {
                    m_hostBodyFilterMap[si * m_bodyCount + bi] = static_cast<int>(fi);
                    resolved = true;
                    break;
                }
            }
            if (resolved)
                continue;

            int bestBi = -1;
            size_t bestLen = 0;
            for (size_t bi = 0; bi < m_bodyLabels.size(); ++bi)
            {
                std::string prefix = m_bodyLabels[bi] + "/";
                if (filterPath.rfind(prefix, 0) == 0 && m_bodyLabels[bi].size() > bestLen)
                {
                    bestBi = static_cast<int>(bi);
                    bestLen = m_bodyLabels[bi].size();
                }
            }
            if (bestBi >= 0)
            {
                m_hostBodyFilterMap[si * m_bodyCount + bestBi] = static_cast<int>(fi);
                continue;
            }

            for (size_t shi = 0; shi < shapeLabels.size() && shi < shapeBodyArr.size(); ++shi)
            {
                const std::string& sl = shapeLabels[shi];
                if (shapeBodyArr[shi] != -1)
                    continue;
                if (sl == filterPath || filterPath.rfind(sl + "/", 0) == 0 || sl.rfind(filterPath + "/", 0) == 0)
                {
                    m_hostBodyFilterMap[si * m_bodyCount + m_worldBodyIndex] = static_cast<int>(fi);
                    resolved = true;
                    break;
                }
            }
        }
    }

    size_t maxScratch = static_cast<size_t>(m_sensorCount) * std::max(m_filterCount, 1u);
    m_scratchCounts.resize(maxScratch, 0);
    m_scratchStartIndices.resize(maxScratch, 0);
    m_scratchFillCounts.resize(maxScratch, 0);

    _cacheStaticPointers();
    _refreshContactPointers();
}

BaseRigidContactView::~BaseRigidContactView()
{
}

void BaseRigidContactView::_cacheStaticPointers()
{
    py::gil_scoped_acquire gil;
    m_cachedShapeBody = warpArrayFromPython<int>(m_model.attr("shape_body")).data;
}

void BaseRigidContactView::_refreshContactPointers() const
{
    py::gil_scoped_acquire gil;

    try
    {
        uint64_t gen = m_newtonStage.attr("simulation_timestamp").cast<uint64_t>();
        if (gen == m_lastRefreshedGeneration)
            return;
        m_lastRefreshedGeneration = gen;
    }
    catch (...)
    {
    }

    auto ptr = [](py::object arr) -> float* { return warpArrayFromPython<float>(arr).data; };
    auto iptr = [](py::object arr) -> int* { return warpArrayFromPython<int>(arr).data; };

    py::object contacts = m_newtonStage.attr("contacts");
    if (contacts.is_none())
        return;

    m_cachedContactCount = iptr(contacts.attr("rigid_contact_count"));
    m_cachedShape0 = iptr(contacts.attr("rigid_contact_shape0"));
    m_cachedShape1 = iptr(contacts.attr("rigid_contact_shape1"));
    m_cachedContactNormal = ptr(contacts.attr("rigid_contact_normal"));
    m_cachedBodyQ = ptr(m_newtonStage.attr("state_0").attr("body_q"));

    py::object spatialForce = py::none();
    if (py::hasattr(contacts, "force"))
        spatialForce = contacts.attr("force");
    if (!spatialForce.is_none())
    {
        m_cachedSpatialForce = ptr(spatialForce);
        m_hasSpatialForce = true;
        m_cachedContactForce = nullptr;
    }
    else
    {
        m_cachedSpatialForce = nullptr;
        m_hasSpatialForce = false;
        py::object rcForce = py::none();
        if (py::hasattr(contacts, "rigid_contact_force"))
            rcForce = contacts.attr("rigid_contact_force");
        m_cachedContactForce = rcForce.is_none() ? nullptr : ptr(rcForce);
    }

    m_cachedContactPoint0 = nullptr;
    m_cachedContactPoint1 = nullptr;
    m_cachedThickness0 = nullptr;
    m_cachedThickness1 = nullptr;
    m_contactPointsInWorldSpace = false;
    if (py::hasattr(contacts, "rigid_contact_point0") && !contacts.attr("rigid_contact_point0").is_none())
        m_cachedContactPoint0 = ptr(contacts.attr("rigid_contact_point0"));
    if (py::hasattr(contacts, "rigid_contact_point1") && !contacts.attr("rigid_contact_point1").is_none())
        m_cachedContactPoint1 = ptr(contacts.attr("rigid_contact_point1"));
    if (py::hasattr(contacts, "rigid_contact_thickness0") && !contacts.attr("rigid_contact_thickness0").is_none())
        m_cachedThickness0 = ptr(contacts.attr("rigid_contact_thickness0"));
    if (py::hasattr(contacts, "rigid_contact_thickness1") && !contacts.attr("rigid_contact_thickness1").is_none())
        m_cachedThickness1 = ptr(contacts.attr("rigid_contact_thickness1"));

    // MuJoCo provides world-space contact positions rather than body-local contact points.
    // Use mjw_data.contact.pos and flag the kernels to skip the body-local→world rotation.
    try
    {
        py::object solver = m_newtonStage.attr("solver");
        if (py::hasattr(solver, "mjw_data"))
        {
            py::object mjwData = solver.attr("mjw_data");
            if (!mjwData.is_none())
            {
                py::object mjContact = mjwData.attr("contact");
                if (!mjContact.is_none() && py::hasattr(mjContact, "pos"))
                {
                    py::object pos = mjContact.attr("pos");
                    if (!pos.is_none())
                    {
                        float* worldPos = ptr(pos);
                        m_cachedContactPoint0 = worldPos;
                        m_cachedContactPoint1 = worldPos;
                        m_contactPointsInWorldSpace = true;
                    }
                }
            }
        }
    }
    catch (const py::error_already_set&)
    {
    }
}

float BaseRigidContactView::_getPhysicsDtScale(float userDt) const
{
    if (m_physicsDt > 0.0f && userDt > 0.0f)
        return m_physicsDt / userDt;
    return 1.0f;
}

uint32_t BaseRigidContactView::getSensorCount() const
{
    return m_sensorCount;
}
uint32_t BaseRigidContactView::getFilterCount() const
{
    return m_filterCount;
}
uint32_t BaseRigidContactView::getMaxContactDataCount() const
{
    return m_maxContactDataCount;
}

bool BaseRigidContactView::check() const
{
    return true;
}
void BaseRigidContactView::release()
{
    delete this;
}

const char* BaseRigidContactView::getUsdPrimPath(uint32_t sensorIdx) const
{
    return (sensorIdx < m_sensorPaths.size()) ? m_sensorPaths[sensorIdx].c_str() : "";
}
const char* BaseRigidContactView::getUsdPrimName(uint32_t sensorIdx) const
{
    return (sensorIdx < m_sensorNames.size()) ? m_sensorNames[sensorIdx].c_str() : "";
}
const char* BaseRigidContactView::getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const
{
    if (sensorIdx < m_filterPaths.size() && filterIdx < m_filterPaths[sensorIdx].size())
        return m_filterPaths[sensorIdx][filterIdx].c_str();
    return "";
}
const char* BaseRigidContactView::getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const
{
    if (sensorIdx < m_filterNames.size() && filterIdx < m_filterNames[sensorIdx].size())
        return m_filterNames[sensorIdx][filterIdx].c_str();
    return "";
}

bool BaseRigidContactView::getFrictionData(
    const TensorDesc*, const TensorDesc*, const TensorDesc*, const TensorDesc*, float) const
{
    return false;
}

void BaseRigidContactView::getOtherActorPathsFromIds(const TensorDesc* otherActorIdsTensor,
                                                     std::vector<std::string>& outPaths) const
{
    if (!otherActorIdsTensor || !otherActorIdsTensor->data)
        return;

    uint32_t n = static_cast<uint32_t>(getTensorTotalSize(*otherActorIdsTensor));
    std::vector<uint64_t> hostIds(n);

    if (otherActorIdsTensor->device >= 0)
    {
        cudaMemcpy(hostIds.data(), otherActorIdsTensor->data, n * sizeof(uint64_t), cudaMemcpyDeviceToHost);
    }
    else
    {
        std::memcpy(hostIds.data(), otherActorIdsTensor->data, n * sizeof(uint64_t));
    }

    outPaths.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        uint64_t bodyIdx = hostIds[i];
        if (bodyIdx == static_cast<uint64_t>(m_worldBodyIndex))
        {
            outPaths[i] = "world";
        }
        else if (bodyIdx < m_bodyLabels.size())
        {
            outPaths[i] = m_bodyLabels[bodyIdx];
        }
        else
        {
            outPaths[i] = "";
        }
    }
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
