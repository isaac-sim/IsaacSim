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

#include "ArticulationMetatype.h"

#include <carb/logging/Log.h>

#include <omni/physics/tensors/JointTypes.h>
#include <pybind11/numpy.h>

#include <limits>

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using namespace omni::physics::tensors;

// Newton JointType enum values (from newton._src.sim.enums)
static constexpr int kJointPrismatic = 0;
static constexpr int kJointRevolute = 1;
static constexpr int kJointBall = 2;
static constexpr int kJointFixed = 3;
static constexpr int kJointFree = 4;
static constexpr int kJointDistance = 5;
static constexpr int kJointD6 = 6;

static JointType mapNewtonJointType(int newtonType)
{
    switch (newtonType)
    {
    case kJointPrismatic:
        return JointType::ePrismatic;
    case kJointRevolute:
        return JointType::eRevolute;
    case kJointBall:
        return JointType::eSpherical;
    case kJointFixed:
        return JointType::eFixed;
    case kJointFree:
        return JointType::eInvalid;
    case kJointDistance:
        return JointType::eInvalid;
    case kJointD6:
        return JointType::eInvalid;
    default:
        return JointType::eInvalid;
    }
}

ArticulationMetatype::ArticulationMetatype(py::object model,
                                           const std::vector<int>& jointIndices,
                                           const std::vector<int>& linkIndices,
                                           const std::vector<std::string>& linkPaths,
                                           const std::vector<std::string>& jointPaths,
                                           int firstJointIndex)
{
    py::gil_scoped_acquire gil;

    try
    {
        m_linkCount = linkIndices.size();
        m_jointCount = jointIndices.size();

        m_linkNames = linkPaths;
        m_jointNames = jointPaths;

        // Get model data
        py::array jointType = model.attr("joint_type").attr("numpy")().cast<py::array>();
        py::array jointQdStart = model.attr("joint_qd_start").attr("numpy")().cast<py::array>();
        py::array jointDofDim = model.attr("joint_dof_dim").attr("numpy")().cast<py::array>();
        py::array jointAxis = model.attr("joint_axis").attr("numpy")().cast<py::array>();

        auto jointTypes = jointType.unchecked<int, 1>();
        auto jointQdStarts = jointQdStart.unchecked<int, 1>();

        m_isFixedBase = (jointTypes(firstJointIndex) != kJointFree);

        // Build joint and DOF metadata
        m_dofCount = 0;
        for (size_t i = 0; i < jointIndices.size(); ++i)
        {
            int globalJointIdx = jointIndices[i];
            int jType = jointTypes(globalJointIdx);

            m_jointTypes.push_back(mapNewtonJointType(jType));

            int qdStart = jointQdStarts(globalJointIdx);
            int qdEnd = jointQdStarts(globalJointIdx + 1);
            int dofCountForJoint = qdEnd - qdStart;

            m_jointDofOffsets.push_back(m_dofCount);
            m_jointDofCounts.push_back(dofCountForJoint);

            for (int d = 0; d < dofCountForJoint; ++d)
            {
                if (jType == kJointBall || jType == kJointD6)
                {
                    m_dofNames.push_back(jointPaths[i] + ":" + std::to_string(d));
                    m_dofTypes.push_back(DofType::eRotation);
                }
                else
                {
                    m_dofNames.push_back(jointPaths[i]);
                    if (jType == kJointPrismatic)
                    {
                        m_dofTypes.push_back(DofType::eTranslation);
                    }
                    else
                    {
                        m_dofTypes.push_back(DofType::eRotation);
                    }
                }
            }

            m_dofCount += dofCountForJoint;
        }

        m_linkShortNames.reserve(m_linkNames.size());
        for (const auto& p : m_linkNames)
        {
            size_t lastSlash = p.find_last_of('/');
            m_linkShortNames.push_back((lastSlash != std::string::npos) ? p.substr(lastSlash + 1) : p);
        }
        m_jointShortNames.reserve(m_jointNames.size());
        for (const auto& p : m_jointNames)
        {
            size_t lastSlash = p.find_last_of('/');
            m_jointShortNames.push_back((lastSlash != std::string::npos) ? p.substr(lastSlash + 1) : p);
        }
        m_dofShortNames.reserve(m_dofNames.size());
        for (const auto& p : m_dofNames)
        {
            size_t lastSlash = p.find_last_of('/');
            m_dofShortNames.push_back((lastSlash != std::string::npos) ? p.substr(lastSlash + 1) : p);
        }

        for (size_t i = 0; i < linkPaths.size(); ++i)
        {
            std::string parentName = "";
            if (i > 0)
            {
                size_t lastSlash = linkPaths[i - 1].find_last_of('/');
                if (lastSlash != std::string::npos)
                {
                    parentName = linkPaths[i - 1].substr(lastSlash + 1);
                }
                else
                {
                    parentName = linkPaths[i - 1];
                }
            }
            m_linkParentNames.push_back(parentName);
        }
    }
    catch (py::error_already_set& e)
    {
        CARB_LOG_ERROR("Failed to initialize ArticulationMetatype: %s", e.what());
    }
}

uint32_t ArticulationMetatype::getLinkCount() const
{
    return m_linkCount;
}

uint32_t ArticulationMetatype::getJointCount() const
{
    return m_jointCount;
}

uint32_t ArticulationMetatype::getDofCount() const
{
    return m_dofCount;
}

const char* ArticulationMetatype::getLinkName(uint32_t linkIdx) const
{
    if (linkIdx < m_linkShortNames.size())
        return m_linkShortNames[linkIdx].c_str();
    return nullptr;
}

const char* ArticulationMetatype::getLinkParentName(uint32_t linkIdx) const
{
    if (linkIdx < m_linkParentNames.size())
    {
        return m_linkParentNames[linkIdx].c_str();
    }
    return nullptr;
}

const char* ArticulationMetatype::getJointName(uint32_t jointIdx) const
{
    if (jointIdx < m_jointShortNames.size())
        return m_jointShortNames[jointIdx].c_str();
    return nullptr;
}

const char* ArticulationMetatype::getDofName(uint32_t dofIdx) const
{
    if (dofIdx < m_dofShortNames.size())
        return m_dofShortNames[dofIdx].c_str();
    return nullptr;
}

int32_t ArticulationMetatype::findLinkIndex(const char* linkName) const
{
    if (!linkName)
        return -1;
    std::string name(linkName);
    for (size_t i = 0; i < m_linkNames.size(); ++i)
    {
        const std::string& path = m_linkNames[i];
        size_t lastSlash = path.find_last_of('/');
        std::string extracted = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
        if (extracted == name || path == name)
        {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

int32_t ArticulationMetatype::findLinkParentIndex(const char* linkName) const
{
    int32_t linkIdx = findLinkIndex(linkName);
    if (linkIdx < 0)
    {
        return -1;
    }
    if (linkIdx == 0)
    {
        return -1;
    }
    return linkIdx - 1;
}

int32_t ArticulationMetatype::findJointIndex(const char* jointName) const
{
    if (!jointName)
        return -1;
    std::string name(jointName);
    for (size_t i = 0; i < m_jointNames.size(); ++i)
    {
        const std::string& path = m_jointNames[i];
        size_t lastSlash = path.find_last_of('/');
        std::string extracted = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
        if (extracted == name || path == name)
        {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

int32_t ArticulationMetatype::findDofIndex(const char* dofName) const
{
    if (!dofName)
        return -1;
    std::string name(dofName);
    for (size_t i = 0; i < m_dofNames.size(); ++i)
    {
        const std::string& path = m_dofNames[i];
        size_t lastSlash = path.find_last_of('/');
        std::string extracted = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
        if (extracted == name || path == name)
        {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

JointType ArticulationMetatype::getJointType(uint32_t jointIdx) const
{
    if (jointIdx < m_jointTypes.size())
    {
        return m_jointTypes[jointIdx];
    }
    return JointType::eInvalid;
}

uint32_t ArticulationMetatype::getJointDofOffset(uint32_t jointIdx) const
{
    if (jointIdx < m_jointDofOffsets.size())
    {
        return m_jointDofOffsets[jointIdx];
    }
    return 0;
}

uint32_t ArticulationMetatype::getJointDofCount(uint32_t jointIdx) const
{
    if (jointIdx < m_jointDofCounts.size())
    {
        return m_jointDofCounts[jointIdx];
    }
    return 0;
}

DofType ArticulationMetatype::getDofType(uint32_t dofIdx) const
{
    if (dofIdx < m_dofTypes.size())
    {
        return m_dofTypes[dofIdx];
    }
    return DofType::eInvalid;
}

bool ArticulationMetatype::getFixedBase() const
{
    return m_isFixedBase;
}

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
