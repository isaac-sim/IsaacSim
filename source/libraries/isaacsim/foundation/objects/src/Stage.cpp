// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/usd/openusd/Usd.hpp>
#include <isaacsim/foundation/usd/ovstage/Usd.hpp>

#include <mutex>
#include <shared_mutex>
#include <stdexcept>

namespace isaacsim
{
namespace foundation
{
namespace objects
{

namespace openusd = isaacsim::foundation::usd::openusd;
namespace ovstage = isaacsim::foundation::usd::ovstage;

namespace details
{

std::optional<Stage> g_defaultStage;
std::shared_mutex g_defaultStageMutex;
thread_local std::optional<Stage> g_activeStage;

void setDefaultStage(std::optional<Stage> stage)
{
    if (stage.has_value() && !stage->isValid())
    {
        throw std::runtime_error("Invalid stage. Open, import or create a new stage and forward it");
    }
    std::unique_lock lock(g_defaultStageMutex);
    g_defaultStage = std::move(stage);
}

Stage getDefaultStage()
{
    std::shared_lock lock(g_defaultStageMutex);
    if (!g_defaultStage.has_value())
    {
        throw std::runtime_error("No default stage set. Define a default stage first.");
    }
    return g_defaultStage.value();
}

void setActiveStage(std::optional<Stage> stage)
{
    if (stage.has_value() && !stage->isValid())
    {
        throw std::runtime_error("Invalid stage. Open, import or create a new stage and forward it");
    }
    g_activeStage = std::move(stage);
}

Stage getActiveStage()
{
    if (g_activeStage.has_value())
    {
        // Another thread may have closed this stage; validate lazily to avoid stale ids
        if (g_activeStage->isValid())
        {
            return g_activeStage.value();
        }
        g_activeStage = std::nullopt;
    }
    return getDefaultStage();
}

StageGuard::StageGuard(Stage stage) : m_previous(g_activeStage)
{
    g_activeStage = std::move(stage);
}

StageGuard::~StageGuard()
{
    g_activeStage = std::move(m_previous);
}

} // namespace details

Stage::Stage(std::string backend, std::optional<int64_t> stageId)
{
    // Set backend
    m_backend = backend;
    if (backend != "openusd" && backend != "ovstage")
    {
        throw std::runtime_error("Invalid backend. It must be 'openusd' or 'ovstage'.");
    }
    m_stageOps = _populateStageOps(backend);
    // Set stage ID
    m_stageId = stageId.value_or(-1);
    if (m_stageId != -1)
    {
        _validate();
    }
}

Stage::~Stage()
{
}

Stage::StageOperations Stage::_populateStageOps(const std::string& backend)
{
    if (backend == "openusd")
    {
        return {
            openusd::createStage,
            openusd::openStage,
            openusd::saveStage,
            openusd::closeStage,
            openusd::exportStageToString,
            openusd::importStageFromString,
            openusd::isStageValid,
            openusd::getStagePtr,
            openusd::definePrim,
            openusd::movePrim,
            openusd::removePrim,
            openusd::getStageUnits,
            openusd::setStageUnits,
            openusd::getStageUpAxis,
            openusd::setStageUpAxis,
            openusd::getStageTimeCode,
            openusd::setStageTimeCode,
            openusd::addReferenceToStage,
            openusd::generateStageRepresentation,
        };
    }
    else if (backend == "ovstage")
    {
        return {
            ovstage::createStage,
            ovstage::openStage,
            ovstage::saveStage,
            ovstage::closeStage,
            ovstage::exportStageToString,
            ovstage::importStageFromString,
            ovstage::isStageValid,
            ovstage::getStagePtr,
            ovstage::definePrim,
            ovstage::movePrim,
            ovstage::removePrim,
            ovstage::getStageUnits,
            ovstage::setStageUnits,
            ovstage::getStageUpAxis,
            ovstage::setStageUpAxis,
            ovstage::getStageTimeCode,
            ovstage::setStageTimeCode,
            ovstage::addReferenceToStage,
            ovstage::generateStageRepresentation,
        };
    }
    throw std::runtime_error("Unknown backend '" + backend + "'. Expected 'openusd' or 'ovstage'.");
}

void Stage::_validate() const
{
    if (m_stageId == -1)
    {
        throw std::runtime_error("Invalid stage. It may not have been created yet, or it may have been closed.");
    }
    else if (!m_stageOps.isStageValid(m_stageId))
    {
        throw std::runtime_error("Invalid stage. It may have been closed.");
    }
}

std::string Stage::getBackend() const
{
    return m_backend;
}

int64_t Stage::getStageId() const
{
    return m_stageId;
}

void* Stage::getStagePtr() const
{
    return m_stageOps.getStagePtr(m_stageId);
}

bool Stage::isValid() const
{
    return (m_stageId != -1) && m_stageOps.isStageValid(m_stageId);
}

Stage& Stage::openStage(const std::string& usdPath, bool makeDefault)
{
    if (m_stageId != -1)
    {
        throw std::runtime_error(
            "This instance is already associated with a stage. It must be closed before opening another one.");
    }
    m_stageId = m_stageOps.openStage(usdPath);
    if (makeDefault && m_stageId != -1)
    {
        details::setDefaultStage(*this);
    }
    return *this;
}

Stage& Stage::createStage(const std::optional<std::string>& templateName, bool makeDefault)
{
    if (m_stageId != -1)
    {
        throw std::runtime_error(
            "This instance is already associated with a stage. It must be closed before creating another one.");
    }
    m_stageId = m_stageOps.createStage();
    // TODO: template support.
    (void)templateName;
    if (makeDefault && m_stageId != -1)
    {
        details::setDefaultStage(*this);
    }
    return *this;
}

bool Stage::saveStage(const std::string& usdPath)
{
    _validate();
    return m_stageOps.saveStage(m_stageId, usdPath);
}

std::string Stage::exportStageToString() const
{
    _validate();
    return m_stageOps.exportStageToString(m_stageId);
}

Stage& Stage::importStageFromString(const std::string& usdString, bool makeDefault)
{
    if (m_stageId != -1)
    {
        throw std::runtime_error(
            "This instance is already associated with a stage. It must be closed before importing another one.");
    }
    m_stageId = m_stageOps.importStageFromString(usdString);
    if (makeDefault && m_stageId != -1)
    {
        details::setDefaultStage(*this);
    }
    return *this;
}

bool Stage::closeStage()
{
    // Clear stage registry references
    bool clearedDefaultStage = false;
    bool clearedActiveStage = false;
    {
        std::unique_lock lock(details::g_defaultStageMutex);
        if (details::g_defaultStage.has_value() && details::g_defaultStage->getBackend() == m_backend &&
            details::g_defaultStage->getStageId() == m_stageId)
        {
            details::g_defaultStage = std::nullopt;
            clearedDefaultStage = true;
        }
    }
    if (details::g_activeStage.has_value() && details::g_activeStage->getBackend() == m_backend &&
        details::g_activeStage->getStageId() == m_stageId)
    {
        details::g_activeStage = std::nullopt;
        clearedActiveStage = true;
    }
    // Close stage
    bool result = m_stageOps.closeStage(m_stageId);
    if (result)
    {
        m_stageId = -1;
    }
    else
    {
        // Restore stage registry references if the slot is still empty (nothing raced in)
        if (clearedDefaultStage)
        {
            std::unique_lock lock(details::g_defaultStageMutex);
            if (!details::g_defaultStage.has_value())
            {
                details::g_defaultStage = *this;
            }
        }
        if (clearedActiveStage && !details::g_activeStage.has_value())
        {
            details::g_activeStage = *this;
        }
    }
    return result;
}

bool Stage::addReference(const std::string& usdPath,
                         const std::string& path,
                         const std::string& primType,
                         const std::optional<std::unordered_map<std::string, std::string>>& variants) const
{
    _validate();
    m_stageOps.addReferenceToStage(m_stageId, path, usdPath, primType);
    if (variants.has_value())
    {
        details::StageGuard guard(*this);
        Prim(path).setVariantSelection(*variants);
    }
    return true;
}

std::string Stage::definePrim(const std::string& path, const std::string& typeName) const
{
    _validate();
    m_stageOps.definePrim(m_stageId, path, typeName);
    return path;
}

std::tuple<bool, std::string> Stage::movePrim(const std::string& targetPath, const std::string& destinationPath) const
{
    _validate();
    return m_stageOps.movePrim(m_stageId, targetPath, destinationPath);
}

bool Stage::removePrim(const std::string& path) const
{
    _validate();
    return m_stageOps.removePrim(m_stageId, path);
}

std::tuple<float, float> Stage::getUnits() const
{
    _validate();
    return m_stageOps.getStageUnits(m_stageId);
}

void Stage::setUnits(std::optional<float> metersPerUnit, std::optional<float> kilogramsPerUnit) const
{
    _validate();
    m_stageOps.setStageUnits(m_stageId, metersPerUnit, kilogramsPerUnit);
}

std::string Stage::getUpAxis() const
{
    _validate();
    return m_stageOps.getStageUpAxis(m_stageId);
}

void Stage::setUpAxis(const std::string& upAxis) const
{
    _validate();
    m_stageOps.setStageUpAxis(m_stageId, upAxis);
}

std::tuple<float, float, float> Stage::getTimeCode() const
{
    _validate();
    return m_stageOps.getStageTimeCode(m_stageId);
}

void Stage::setTimeCode(std::optional<float> startTimeCode,
                        std::optional<float> endTimeCode,
                        std::optional<float> timeCodesPerSecond) const
{
    _validate();
    m_stageOps.setStageTimeCode(m_stageId, startTimeCode, endTimeCode, timeCodesPerSecond);
}

std::string Stage::generateStringRepresentation(const std::string& mode) const
{
    _validate();
    return m_stageOps.generateStageRepresentation(m_stageId, mode);
}

} // namespace objects
} // namespace foundation
} // namespace isaacsim
