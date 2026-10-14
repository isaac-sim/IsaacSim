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

#include "BaseRigidContactView.hpp"

#include "utils/TensorOps.hpp"
#include "utils/WarpInterop.hpp"

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

namespace
{

int resolvePathToBodyIndex(const std::string& path,
                           const std::vector<std::string>& bodyLabels,
                           const std::vector<std::string>& shapeLabels,
                           const std::vector<int>& shapeBodyArr,
                           int worldBodyIndex)
{
    if (path.empty())
    {
        return -1;
    }

    for (size_t bi = 0; bi < bodyLabels.size(); ++bi)
    {
        if (bodyLabels[bi] == path)
        {
            return static_cast<int>(bi);
        }
    }

    int bestBi = -1;
    size_t bestLen = 0;
    for (size_t bi = 0; bi < bodyLabels.size(); ++bi)
    {
        const std::string prefix = bodyLabels[bi] + "/";
        if (path.rfind(prefix, 0) == 0 && bodyLabels[bi].size() > bestLen)
        {
            bestBi = static_cast<int>(bi);
            bestLen = bodyLabels[bi].size();
        }
    }
    if (bestBi >= 0)
    {
        return bestBi;
    }

    bestBi = -1;
    bestLen = 0;
    const std::string pathPrefix = path + "/";
    for (size_t bi = 0; bi < bodyLabels.size(); ++bi)
    {
        if (bodyLabels[bi].rfind(pathPrefix, 0) == 0 && bodyLabels[bi].size() > bestLen)
        {
            bestBi = static_cast<int>(bi);
            bestLen = bodyLabels[bi].size();
        }
    }
    if (bestBi >= 0)
    {
        return bestBi;
    }

    for (size_t shi = 0; shi < shapeLabels.size() && shi < shapeBodyArr.size(); ++shi)
    {
        const std::string& sl = shapeLabels[shi];
        if (sl == path || path.rfind(sl + "/", 0) == 0 || sl.rfind(path + "/", 0) == 0)
        {
            if (shapeBodyArr[shi] >= 0)
            {
                return shapeBodyArr[shi];
            }
            if (shapeBodyArr[shi] == -1)
            {
                return worldBodyIndex;
            }
        }
    }

    return -1;
}

} // namespace

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
    m_contactForcesEnabled = m_newtonStage.attr("cfg").attr("solver_outputs").attr("contact_forces").cast<bool>();

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
    m_shapeLabels = std::move(shapeLabels);
    m_shapeBodyArr = std::move(shapeBodyArr);

    m_sensorCount = static_cast<uint32_t>(m_sensorPaths.size());

    m_sensorNames.reserve(m_sensorCount);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        pxr::SdfPath sdfPath(m_sensorPaths[si]);
        m_sensorNames.push_back(sdfPath.GetName());
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

    _rebuildBodyMaps();

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

bool BaseRigidContactView::_hasContactForces() const
{
    if (m_contactForcesEnabled)
        return true;
    CARB_LOG_WARN_ONCE(
        "Newton contact force reporting is disabled. Set NewtonConfig.solver_outputs.contact_forces=True before "
        "initializing physics, then recreate the simulation and tensor views.");
    return false;
}

void BaseRigidContactView::_cacheStaticPointers()
{
    py::gil_scoped_acquire gil;
    m_cachedShapeBody = warpArrayFromPython<int>(m_model.attr("shape_body")).data;
}

void BaseRigidContactView::_rebuildBodyMaps()
{
    py::gil_scoped_acquire gil;

    int bodyCount = m_model.attr("body_count").cast<int>();
    m_worldBodyIndex = bodyCount;
    m_bodyCount = bodyCount + 1;

    py::list bodyLabel = m_model.attr("body_label").cast<py::list>();
    m_bodyLabels.clear();
    m_bodyLabels.reserve(static_cast<size_t>(py::len(bodyLabel)));
    for (auto item : bodyLabel)
    {
        m_bodyLabels.push_back(py::str(item));
    }

    m_hostBodySensorMap.assign(m_bodyCount, -1);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        const std::string& sensorPath = m_sensorPaths[si];
        const int bi = resolvePathToBodyIndex(sensorPath, m_bodyLabels, m_shapeLabels, m_shapeBodyArr, m_worldBodyIndex);
        if (bi >= 0 && bi < m_bodyCount)
        {
            m_hostBodySensorMap[bi] = static_cast<int>(si);
        }

        const std::string pathPrefix = sensorPath + "/";
        for (size_t bodyIdx = 0; bodyIdx < m_bodyLabels.size(); ++bodyIdx)
        {
            const std::string& label = m_bodyLabels[bodyIdx];
            if (label == sensorPath || label.rfind(pathPrefix, 0) == 0)
            {
                m_hostBodySensorMap[static_cast<int>(bodyIdx)] = static_cast<int>(si);
            }
        }

        for (size_t shi = 0; shi < m_shapeLabels.size() && shi < m_shapeBodyArr.size(); ++shi)
        {
            const int shapeBodyIdx = m_shapeBodyArr[shi];
            if (shapeBodyIdx < 0 || shapeBodyIdx >= m_bodyCount)
            {
                continue;
            }
            const std::string& shapePath = m_shapeLabels[shi];
            if (shapePath == sensorPath || shapePath.rfind(pathPrefix, 0) == 0)
            {
                m_hostBodySensorMap[shapeBodyIdx] = static_cast<int>(si);
            }
        }
    }

    m_hostBodyFilterMap.assign(static_cast<size_t>(m_sensorCount) * static_cast<size_t>(m_bodyCount), -1);
    for (uint32_t si = 0; si < m_sensorCount; ++si)
    {
        if (si >= m_filterPaths.size())
        {
            continue;
        }
        for (uint32_t fi = 0; fi < static_cast<uint32_t>(m_filterPaths[si].size()); ++fi)
        {
            const std::string& filterPath = m_filterPaths[si][fi];
            if (filterPath.empty())
            {
                continue;
            }

            const int bi =
                resolvePathToBodyIndex(filterPath, m_bodyLabels, m_shapeLabels, m_shapeBodyArr, m_worldBodyIndex);
            if (bi >= 0 && bi < m_bodyCount)
            {
                m_hostBodyFilterMap[static_cast<size_t>(si) * static_cast<size_t>(m_bodyCount) + static_cast<size_t>(bi)] =
                    static_cast<int>(fi);
            }
        }
    }
}

void BaseRigidContactView::_refreshContactPointers() const
{
    py::gil_scoped_acquire gil;

    try
    {
        if (!m_valid)
            return;
        py::object model = m_newtonStage.attr("model");
        if (!model.is(m_model))
        {
            // The Newton stage rebuilt or released its model: the cached mappings and pointers are stale.
            m_valid = false;
            m_model = py::none();
            CARB_LOG_WARN("Rigid contact view: the Newton model was rebuilt or released; recreate the view");
            return;
        }

        uint64_t gen = m_newtonStage.attr("simulation_timestamp").cast<uint64_t>();
        if (gen == m_lastRefreshedGeneration)
            return;
        m_lastRefreshedGeneration = gen;
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Rigid contact view: failed to read the Newton model: %s", e.what());
        return;
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
    _refreshContactPointers();
    return m_valid;
}
void BaseRigidContactView::release()
{
    // Native callers may not hold the GIL; keep it through destruction of the Python members.
    py::gil_scoped_acquire gil;
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
    _hasContactForces();
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
        if (bodyIdx < m_bodyLabels.size())
        {
            outPaths[i] = m_bodyLabels[bodyIdx];
        }
        else if (bodyIdx > static_cast<uint64_t>(m_worldBodyIndex))
        {
            const uint64_t shapeIdx = bodyIdx - static_cast<uint64_t>(m_worldBodyIndex) - 1u;
            if (shapeIdx < m_shapeLabels.size())
            {
                outPaths[i] = m_shapeLabels[shapeIdx];
            }
        }
        else if (bodyIdx == static_cast<uint64_t>(m_worldBodyIndex))
        {
            // Retain a valid absolute sentinel for callers holding an ID emitted by an
            // older backend. New raw-contact data encodes the specific static shape.
            outPaths[i] = "/";
        }

        if (!outPaths[i].empty() && outPaths[i][0] != '/')
        {
            outPaths[i].insert(outPaths[i].begin(), '/');
        }
    }
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
