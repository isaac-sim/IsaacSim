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

#include "BaseSimulationView.h"

#include <carb/logging/Log.h>

#include <pxr/base/tf/stringUtils.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pybind11/numpy.h>

#include <regex>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

SimViewInit BaseSimulationView::initNewton(long stageId)
{
    SimViewInit d;

    py::gil_scoped_acquire gil;
    try
    {
        py::module_ newton_ext = py::module_::import("isaacsim.physics.newton.impl.extension");
        d.newtonStage = newton_ext.attr("acquire_stage")();

        if (d.newtonStage.is_none())
        {
            CARB_LOG_ERROR("Newton stage not available - is isaacsim.physics.newton enabled?");
            return d;
        }

        if (!d.newtonStage.attr("initialized").cast<bool>())
        {
            CARB_LOG_WARN("Newton not yet initialized, triggering initialization...");
            d.newtonStage.attr("initialize_newton")(py::none());
        }

        if (!d.newtonStage.attr("initialized").cast<bool>())
        {
            CARB_LOG_ERROR("Newton initialization failed");
            return d;
        }

        d.usdStage = d.newtonStage.attr("usd_stage");
        d.model = d.newtonStage.attr("model");

        if (!d.model.is_none())
        {
            py::object device = d.model.attr("joint_q").attr("device");
            std::string deviceStr = py::str(device);
            if (deviceStr.find("cpu") == std::string::npos)
            {
                d.simDeviceOrdinal = device.attr("ordinal").cast<int>();
            }
        }

        d.valid = true;
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to initialize simulation view: %s", e.what());
    }

    return d;
}

BaseSimulationView::BaseSimulationView(SimViewInit&& init)
    : m_usdStage(std::move(init.usdStage)),
      m_newtonStage(std::move(init.newtonStage)),
      m_model(std::move(init.model)),
      m_simDeviceOrdinal(init.simDeviceOrdinal),
      m_valid(init.valid)
{
}

BaseSimulationView::~BaseSimulationView()
{
}

bool BaseSimulationView::getValid() const
{
    return m_valid;
}

void BaseSimulationView::invalidate()
{
    m_valid = false;
}

bool BaseSimulationView::check() const
{
    return m_valid;
}

bool BaseSimulationView::setSubspaceRoots(const char* pattern)
{
    return true;
}

void BaseSimulationView::step(float dt)
{
    if (!m_valid)
        return;

    py::gil_scoped_acquire gil;
    try
    {
        m_newtonStage.attr("step_sim")(dt);
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to step simulation: %s", e.what());
    }
}

bool BaseSimulationView::setGravity(const carb::Float3& gravity)
{
    if (!m_valid)
        return false;

    m_gravity = gravity;

    py::gil_scoped_acquire gil;
    try
    {
        if (!m_model.is_none())
        {
            py::list gravityList;
            gravityList.append(gravity.x);
            gravityList.append(gravity.y);
            gravityList.append(gravity.z);
            m_model.attr("gravity") = gravityList;
        }
        return true;
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to set gravity: %s", e.what());
        return false;
    }
}

bool BaseSimulationView::getGravity(carb::Float3& gravity)
{
    if (!m_valid)
        return false;

    py::gil_scoped_acquire gil;
    try
    {
        if (!m_model.is_none())
        {
            py::list gravityList = m_model.attr("gravity").cast<py::list>();
            gravity.x = gravityList[0].cast<float>();
            gravity.y = gravityList[1].cast<float>();
            gravity.z = gravityList[2].cast<float>();
        }
        else
        {
            gravity = m_gravity;
        }
        return true;
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to get gravity: %s", e.what());
        return false;
    }
}

static constexpr int kNewtonJointFree = 4;

ObjectType BaseSimulationView::getObjectType(const char* path)
{
    if (!m_valid || !path || m_model.is_none())
        return ObjectType::eInvalid;

    py::gil_scoped_acquire gil;
    try
    {
        std::string pathStr(path);

        py::list bodyLabel = m_model.attr("body_label").cast<py::list>();
        int bodyCount = static_cast<int>(py::len(bodyLabel));

        py::array articulationStartArr = m_model.attr("articulation_start").attr("numpy")().cast<py::array>();
        auto artStarts = articulationStartArr.unchecked<int, 1>();
        int articulationCount = m_model.attr("articulation_count").cast<int>();

        py::array jointTypeArr = m_model.attr("joint_type").attr("numpy")().cast<py::array>();
        py::array jointChildArr = m_model.attr("joint_child").attr("numpy")().cast<py::array>();
        auto jointTypes = jointTypeArr.unchecked<int, 1>();
        auto jointChildren = jointChildArr.unchecked<int, 1>();
        int numJoints = static_cast<int>(jointTypeArr.shape(0));

        for (int bi = 0; bi < bodyCount; ++bi)
        {
            if (py::str(bodyLabel[bi]).cast<std::string>() != pathStr)
                continue;

            int artIdx = -1;
            for (int ai = 0; ai < articulationCount; ++ai)
            {
                int start = artStarts(ai);
                int end = (ai + 1 < artStarts.shape(0)) ? artStarts(ai + 1) : bodyCount;
                if (bi >= start && bi < end)
                {
                    artIdx = ai;
                    break;
                }
            }
            if (artIdx < 0)
                return ObjectType::eInvalid;

            int artStart = artStarts(artIdx);
            int artEnd = (artIdx + 1 < artStarts.shape(0)) ? artStarts(artIdx + 1) : bodyCount;
            int bodyCountInArt = artEnd - artStart;
            bool isRootBody = (bi == artStart);

            if (bodyCountInArt == 1)
            {
                for (int ji = 0; ji < numJoints; ++ji)
                {
                    if (jointChildren(ji) == bi && jointTypes(ji) == kNewtonJointFree)
                        return ObjectType::eRigidBody;
                }
            }
            return isRootBody ? ObjectType::eArticulationRootLink : ObjectType::eArticulationLink;
        }

        if (py::hasattr(m_model, "joint_label"))
        {
            py::list jointLabel = m_model.attr("joint_label").cast<py::list>();
            for (size_t i = 0; i < py::len(jointLabel); ++i)
            {
                if (py::str(jointLabel[i]).cast<std::string>() == pathStr)
                    return ObjectType::eArticulationJoint;
            }
        }

        if (py::hasattr(m_model, "articulation_label"))
        {
            py::list artLabel = m_model.attr("articulation_label").cast<py::list>();
            for (size_t i = 0; i < py::len(artLabel); ++i)
            {
                if (py::str(artLabel[i]).cast<std::string>() == pathStr)
                    return ObjectType::eArticulation;
            }
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("getObjectType failed: %s", e.what());
    }

    return ObjectType::eInvalid;
}

void BaseSimulationView::clearForces()
{
}
bool BaseSimulationView::flush()
{
    return true;
}
void BaseSimulationView::updateArticulationsKinematic()
{
}
void BaseSimulationView::InitializeKinematicBodies()
{
}
void BaseSimulationView::enableGpuUsageWarnings(bool enable)
{
}
void BaseSimulationView::release(bool recursive)
{
}

ISdfShapeView* BaseSimulationView::createSdfShapeView(const char* pattern, uint32_t numSamplePoints)
{
    return nullptr;
}
IDeformableBodyView* BaseSimulationView::createVolumeDeformableBodyView(const char* pattern)
{
    return nullptr;
}
IDeformableBodyView* BaseSimulationView::createVolumeDeformableBodyView(const std::vector<std::string>& patterns)
{
    return nullptr;
}
IDeformableBodyView* BaseSimulationView::createSurfaceDeformableBodyView(const char* pattern)
{
    return nullptr;
}
IDeformableBodyView* BaseSimulationView::createSurfaceDeformableBodyView(const std::vector<std::string>& patterns)
{
    return nullptr;
}
IDeformableMaterialView* BaseSimulationView::createDeformableMaterialView(const char* pattern)
{
    return nullptr;
}
IDeformableMaterialView* BaseSimulationView::createDeformableMaterialView(const std::vector<std::string>& patterns)
{
    return nullptr;
}

void BaseSimulationView::_findMatchingPaths(const std::string& pattern, std::vector<pxr::SdfPath>& pathsRet)
{
    py::gil_scoped_acquire gil;

    try
    {
        if (m_usdStage.is_none())
            return;

        bool hasWildcard = (pattern.find('*') != std::string::npos || pattern.find('[') != std::string::npos);
        if (!hasWildcard && pxr::SdfPath::IsValidPathString(pattern))
        {
            py::object prim = m_usdStage.attr("GetPrimAtPath")(py::str(pattern));
            if (!prim.is_none() && py::bool_(prim))
                pathsRet.push_back(pxr::SdfPath(pattern));
            return;
        }

        std::string clean = pxr::TfStringTrim(pattern, "/");
        std::vector<std::string> segments = pxr::TfStringSplit(clean, "/");
        if (segments.empty())
            return;

        std::vector<std::regex> regexSegments;
        regexSegments.reserve(segments.size());
        for (const auto& seg : segments)
        {
            std::string re = "^";
            for (char c : seg)
            {
                if (c == '*')
                    re += ".*";
                else if (c == '.' || c == '+' || c == '?' || c == '(' || c == ')' || c == '{' || c == '}' ||
                         c == '\\' || c == '^' || c == '$' || c == '|')
                {
                    re += '\\';
                    re += c;
                }
                else
                    re += c;
            }
            re += '$';
            regexSegments.emplace_back(re);
        }

        py::list roots;
        roots.append(m_usdStage.attr("GetPseudoRoot")());

        for (size_t level = 0; level < regexSegments.size(); ++level)
        {
            py::list matches;
            for (auto root : roots)
            {
                for (auto child : root.attr("GetAllChildren")())
                {
                    std::string name = py::str(child.attr("GetName")());
                    if (std::regex_match(name, regexSegments[level]))
                        matches.append(child);
                }
            }

            if (level < regexSegments.size() - 1)
                roots = matches;
            else
            {
                for (auto match : matches)
                    pathsRet.push_back(pxr::SdfPath(py::str(match.attr("GetPath")())));
            }
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to find matching paths: %s", e.what());
    }
}

void BaseSimulationView::_findMatchingChildren(const pxr::UsdPrim& prim,
                                               const std::string& pattern,
                                               std::vector<pxr::UsdPrim>& matchesRet)
{
    std::regex re(pattern);
    for (const auto& child : prim.GetChildren())
    {
        if (std::regex_match(child.GetName().GetString(), re))
        {
            matchesRet.push_back(child);
        }
    }
}

// ---------------------------------------------------------------------------
// Factory methods — shared pattern-matching logic, delegates to newXxxView()
// ---------------------------------------------------------------------------

IArticulationView* BaseSimulationView::createArticulationView(const char* pattern)
{
    return createArticulationView(std::vector<std::string>{ pattern });
}

IArticulationView* BaseSimulationView::createArticulationView(const std::vector<std::string>& patterns)
{
    if (!m_valid)
    {
        CARB_LOG_ERROR("Cannot create articulation view - simulation view is invalid");
        return nullptr;
    }

    py::gil_scoped_acquire gil;
    try
    {
        std::vector<pxr::SdfPath> matchedPaths;
        for (const auto& pattern : patterns)
            _findMatchingPaths(pattern, matchedPaths);

        if (matchedPaths.empty())
            return nullptr;

        return _makeArticulationView(m_newtonStage, matchedPaths);
    }
    catch (std::exception& e)
    {
        CARB_LOG_ERROR("Failed to create articulation view: %s", e.what());
        return nullptr;
    }
}

IRigidBodyView* BaseSimulationView::createRigidBodyView(const char* pattern)
{
    return createRigidBodyView(std::vector<std::string>{ pattern });
}

IRigidBodyView* BaseSimulationView::createRigidBodyView(const std::vector<std::string>& patterns)
{
    if (!m_valid)
    {
        CARB_LOG_ERROR("Cannot create rigid body view - simulation view is invalid");
        return nullptr;
    }

    py::gil_scoped_acquire gil;
    try
    {
        std::vector<pxr::SdfPath> matchedPaths;
        for (const auto& pattern : patterns)
            _findMatchingPaths(pattern, matchedPaths);

        if (matchedPaths.empty())
            return nullptr;

        return _makeRigidBodyView(m_newtonStage, matchedPaths);
    }
    catch (std::exception& e)
    {
        CARB_LOG_ERROR("Failed to create rigid body view: %s", e.what());
        return nullptr;
    }
}

IRigidContactView* BaseSimulationView::createRigidContactView(const char* pattern,
                                                              const char** filterPatterns,
                                                              uint32_t numFilterPatterns,
                                                              uint32_t maxContactDataCount)
{
    std::vector<std::string> fp;
    for (uint32_t i = 0; i < numFilterPatterns; ++i)
        fp.push_back(std::string(filterPatterns[i]));
    return createRigidContactView(std::string(pattern), fp, maxContactDataCount);
}

IRigidContactView* BaseSimulationView::createRigidContactView(std::string pattern,
                                                              const std::vector<std::string>& filterPatterns,
                                                              uint32_t maxContactDataCount)
{
    return createRigidContactView(std::vector<std::string>{ pattern },
                                  std::vector<std::vector<std::string>>{ filterPatterns }, maxContactDataCount);
}

IRigidContactView* BaseSimulationView::createRigidContactView(const std::vector<std::string>& patterns,
                                                              const std::vector<std::vector<std::string>>& filterPatterns,
                                                              uint32_t maxContactDataCount)
{
    if (!m_valid)
    {
        CARB_LOG_ERROR("Cannot create rigid contact view - simulation view is invalid");
        return nullptr;
    }

    py::gil_scoped_acquire gil;
    try
    {
        std::vector<std::string> sensorPaths;
        std::vector<std::vector<std::string>> resolvedFilters;

        for (size_t pi = 0; pi < patterns.size(); ++pi)
        {
            std::vector<pxr::SdfPath> matchedSensors;
            _findMatchingPaths(patterns[pi], matchedSensors);
            if (matchedSensors.empty())
                continue;

            size_t numSensorsInGroup = matchedSensors.size();

            std::vector<std::vector<std::string>> filterPathsPerPattern;
            if (pi < filterPatterns.size())
            {
                for (const auto& fp : filterPatterns[pi])
                {
                    std::vector<pxr::SdfPath> matchedFilters;
                    _findMatchingPaths(fp, matchedFilters);

                    std::vector<std::string> perSensor(numSensorsInGroup);
                    if (matchedFilters.size() == 1)
                    {
                        for (size_t s = 0; s < numSensorsInGroup; ++s)
                            perSensor[s] = matchedFilters[0].GetString();
                    }
                    else if (matchedFilters.size() == numSensorsInGroup)
                    {
                        for (size_t s = 0; s < numSensorsInGroup; ++s)
                            perSensor[s] = matchedFilters[s].GetString();
                    }
                    else
                    {
                        CARB_LOG_ERROR("Filter pattern '%s' matched %zu objects, expected 1 or %zu", fp.c_str(),
                                       matchedFilters.size(), numSensorsInGroup);
                        for (size_t s = 0; s < numSensorsInGroup; ++s)
                            perSensor[s] = "";
                    }
                    filterPathsPerPattern.push_back(std::move(perSensor));
                }
            }

            for (size_t s = 0; s < numSensorsInGroup; ++s)
            {
                sensorPaths.push_back(matchedSensors[s].GetString());
                std::vector<std::string> sensorFilters;
                for (const auto& fpp : filterPathsPerPattern)
                    sensorFilters.push_back(fpp[s]);
                resolvedFilters.push_back(std::move(sensorFilters));
            }
        }

        if (sensorPaths.empty())
            return nullptr;

        return _makeRigidContactView(m_newtonStage, sensorPaths, resolvedFilters, maxContactDataCount);
    }
    catch (std::exception& e)
    {
        CARB_LOG_ERROR("Failed to create rigid contact view: %s", e.what());
        return nullptr;
    }
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
