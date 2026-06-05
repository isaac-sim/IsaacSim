// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "BufferRegistry.h"

#include <carb/PluginUtils.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/events/EventsUtils.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/experimental/prims/SdfPathToken.h>
#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/core/includes/Pose.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <omni/ext/IExt.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physics/simulation/IPhysics.h>
#include <omni/physics/simulation/IPhysicsStageUpdate.h>
#include <omni/physics/tensors/IArticulationMetatype.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/usd/UsdContext.h>
#include <physxSchema/physxContactReportAPI.h>
#include <physxSchema/physxRigidBodyAPI.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#if defined(_WIN32)
#    include <usdrt/scenegraph/usd/usd/stage.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    include <usdrt/scenegraph/usd/usd/stage.h>
#    pragma GCC diagnostic pop
#endif

#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

static_assert(static_cast<uint32_t>(omni::physx::ContactEventType::Enum::eCONTACT_FOUND) ==
                  isaacsim::core::experimental::prims::kContactEventFound,
              "kContactEventFound does not match PhysX enum");
static_assert(static_cast<uint32_t>(omni::physx::ContactEventType::Enum::eCONTACT_LOST) ==
                  isaacsim::core::experimental::prims::kContactEventLost,
              "kContactEventLost does not match PhysX enum");
static_assert(static_cast<uint32_t>(omni::physx::ContactEventType::Enum::eCONTACT_PERSIST) ==
                  isaacsim::core::experimental::prims::kContactEventPersist,
              "kContactEventPersist does not match PhysX enum");

const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.core.experimental.primdata.plugin",
                                                    "C++ read-only prim data reader", "NVIDIA",
                                                    carb::PluginHotReload::eEnabled, "dev" };

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

namespace
{

using simulation_manager::ISimulationManager;
using namespace omni::physics::tensors;

static void fillTensorDesc(TensorDesc& desc, void* dataPtr, int numElements, TensorDataType type, int device)
{
    desc.dtype = type;
    desc.numDims = 1;
    desc.dims[0] = numElements;
    desc.data = dataPtr;
    desc.ownData = true;
    desc.device = device;
}

/**
 * @brief Decompose a 4x4 matrix into a position (3 floats) and a wxyz quaternion (4 floats).
 */
static void decomposeMatrix(const usdrt::GfMatrix4d& m, float* outputPosition, float* outputQuaternion)
{
    auto translation = m.ExtractTranslation();
    outputPosition[0] = static_cast<float>(translation[0]);
    outputPosition[1] = static_cast<float>(translation[1]);
    outputPosition[2] = static_cast<float>(translation[2]);

    usdrt::GfMatrix4d noScale = m;
    noScale.Orthonormalize();
    auto q = noScale.ExtractRotation();
    outputQuaternion[0] = static_cast<float>(q.GetReal());
    auto imaginary = q.GetImaginary();
    outputQuaternion[1] = static_cast<float>(imaginary[0]);
    outputQuaternion[2] = static_cast<float>(imaginary[1]);
    outputQuaternion[3] = static_cast<float>(imaginary[2]);
}

/**
 * @class BaseDataView
 * @brief Shared implementation of buffer fetch, dirty tracking, and callback dispatch.
 * @details The callback stored in each @c FieldEntry is either a C++ lambda (for PhysX
 * and Fabric paths) or a Python callable wrapped by pybind11 (for Newton). The dirty
 * tracking logic is identical for both.
 */
class BaseDataView
{
protected:
    ViewData* m_data = nullptr;
    ISimulationManager* m_simulationManager = nullptr;
    pxr::UsdStageRefPtr m_usdStage;
    usdrt::UsdStageRefPtr m_usdrtStage;

    /**
     * @brief Look up a field by name, run its callback if stale, and return the device buffer pointer.
     * @tparam T Element type (float or uint8_t).
     * @param[in] fields   Map of field name to FieldEntry<T> (fieldsF or fieldsU8).
     * @param[in] name    Field name (e.g. "dof_positions").
     * @param[out] outCount If non-null, receives the number of elements; set to 0 if field not found.
     * @return Pointer to the buffer data, or nullptr if the field does not exist or has no buffer.
     */
    template <typename T>
    const T* _fetchFieldImpl(std::unordered_map<std::string, FieldEntry<T>>& fields, const std::string& name, int* outCount)
    {
        auto it = fields.find(name);
        if (it == fields.end())
        {
            if (outCount)
                *outCount = 0;
            return nullptr;
        }
        auto& field = it->second;
        int64_t step = m_simulationManager ? static_cast<int64_t>(m_simulationManager->getNumPhysicsSteps()) : 0;
        if (field.lastStep < step)
        {
            if (field.callback)
                field.callback();
            field.lastStep = step;
        }
        if (outCount)
            *outCount = static_cast<int>(field.count);
        return field.buffer ? field.buffer->data() : nullptr;
    }

    /**
     * @brief Ensure host staging buffer exists for the field, copy from device if needed, and return host pointer.
     * @tparam T Element type (float or uint8_t).
     * @param[in] fields Map of field name to FieldEntry<T> (fieldsF or fieldsU8).
     * @param[in] name  Field name.
     * @return Pointer to the host staging data, or nullptr if the field does not exist.
     */
    template <typename T>
    const T* _copyToHostAndGet(std::unordered_map<std::string, FieldEntry<T>>& fields, const std::string& name)
    {
        auto it = fields.find(name);
        if (it == fields.end())
            return nullptr;
        auto& field = it->second;
        if (!field.hostStaging)
            field.hostStaging = std::make_unique<includes::GenericBufferBase<T>>(field.count, -1);
        if (field.hostLastStep < field.lastStep)
        {
            field.buffer->copyTo(field.hostStaging->data(), field.count);
            field.hostLastStep = field.lastStep;
        }
        return field.hostStaging->data();
    }

    /**
     * @brief Fetch field data on device (runs callback if stale). Dispatches to float or uint8_t map.
     * @tparam T Element type (float or uint8_t).
     * @param[in] name     Field name (e.g. "dof_positions", "dof_types").
     * @param[out] outCount If non-null, receives the number of elements.
     * @return Pointer to the buffer data, or nullptr if not found.
     */
    template <typename T>
    const T* _fetchFieldT(const std::string& name, int* outCount)
    {
        if constexpr (std::is_same_v<T, float>)
            return _fetchFieldImpl(m_data->fieldsF, name, outCount);
        else if constexpr (std::is_same_v<T, uint8_t>)
            return _fetchFieldImpl(m_data->fieldsU8, name, outCount);
        return nullptr;
    }

    /**
     * @brief Fetch field data on host. If device is GPU, copies to host staging and returns that pointer.
     * @tparam T Element type (float or uint8_t).
     * @param[in] name     Field name.
     * @param[out] outCount If non-null, receives the number of elements.
     * @return Pointer to host-accessible data, or nullptr if field not found.
     */
    template <typename T>
    const T* _fetchFieldHostT(const std::string& name, int* outCount)
    {
        const T* ptr = _fetchFieldT<T>(name, outCount);
        if (!ptr)
            return nullptr;
        if (m_data->deviceOrdinal >= 0)
        {
            if constexpr (std::is_same_v<T, float>)
                return _copyToHostAndGet(m_data->fieldsF, name);
            else if constexpr (std::is_same_v<T, uint8_t>)
                return _copyToHostAndGet(m_data->fieldsU8, name);
            return nullptr;
        }
        return ptr;
    }

    const float* _fetchField(const std::string& name, int* outCount)
    {
        return _fetchFieldT<float>(name, outCount);
    }

    const float* _fetchFieldHost(const std::string& name, int* outCount)
    {
        return _fetchFieldHostT<float>(name, outCount);
    }

    /**
     * @brief Fetch world positions and orientations atomically, running the shared fill callback at most once.
     * @details Both fields share the same fill callback. This method checks staleness once, runs the callback
     * if needed, and marks both fields current — avoiding the double invocation that occurs when calling
     * getWorldPositions and getWorldOrientations separately.
     * @param[in] hostVariant If true, copies GPU buffers to host staging before returning.
     * @return Poses struct with pointers to the positions and orientations buffers and their element counts.
     */
    Poses _fetchWorldPosesImpl(bool hostVariant)
    {
        auto& fieldsF = m_data->fieldsF;
        auto itPos = fieldsF.find("world_positions");
        auto itOri = fieldsF.find("world_orientations");
        if (itPos == fieldsF.end() || itOri == fieldsF.end())
            return { nullptr, 0, nullptr, 0 };

        auto& posField = itPos->second;
        auto& oriField = itOri->second;
        int64_t step = m_simulationManager ? static_cast<int64_t>(m_simulationManager->getNumPhysicsSteps()) : 0;

        // Run the shared transform callback only once and mark both fields current.
        if (posField.lastStep < step || oriField.lastStep < step)
        {
            if (posField.callback)
                posField.callback();
            posField.lastStep = step;
            oriField.lastStep = step;
        }

        const float* positions = posField.buffer ? posField.buffer->data() : nullptr;
        const float* orientations = oriField.buffer ? oriField.buffer->data() : nullptr;

        if (hostVariant && m_data->deviceOrdinal >= 0)
        {
            positions = _copyToHostAndGet(fieldsF, "world_positions");
            orientations = _copyToHostAndGet(fieldsF, "world_orientations");
        }

        return { positions, static_cast<int>(posField.count), orientations, static_cast<int>(oriField.count) };
    }

    const uint8_t* _fetchFieldU8(const std::string& name, int* outCount)
    {
        return _fetchFieldT<uint8_t>(name, outCount);
    }

    const uint8_t* _fetchFieldU8Host(const std::string& name, int* outCount)
    {
        return _fetchFieldHostT<uint8_t>(name, outCount);
    }

    template <typename T>
    static void _runFieldCallbacksForStep(std::unordered_map<std::string, FieldEntry<T>>& map, int64_t step)
    {
        for (auto& [name, field] : map)
        {
            if (field.lastStep < step && field.callback)
            {
                field.callback();
                field.lastStep = step;
            }
        }
    }

    bool _updateImpl()
    {
        int64_t step = m_simulationManager ? static_cast<int64_t>(m_simulationManager->getNumPhysicsSteps()) : 0;
        _runFieldCallbacksForStep(m_data->fieldsF, step);
        _runFieldCallbacksForStep(m_data->fieldsU8, step);
        return true;
    }

    template <typename T>
    bool _allocateBufferImpl(const char* fieldName, size_t count)
    {
        m_data->getOrCreateField<T>(std::string(fieldName), count, m_data->deviceOrdinal);
        return true;
    }

    uintptr_t _getBufferPtrImpl(const char* fieldName)
    {
        const std::string name(fieldName);
        auto itF = m_data->fieldsF.find(name);
        if (itF != m_data->fieldsF.end() && itF->second.buffer)
            return reinterpret_cast<uintptr_t>(itF->second.buffer->data());
        auto itU8 = m_data->fieldsU8.find(name);
        if (itU8 != m_data->fieldsU8.end() && itU8->second.buffer)
            return reinterpret_cast<uintptr_t>(itU8->second.buffer->data());
        return 0;
    }

    size_t _getBufferSizeImpl(const char* fieldName)
    {
        const std::string name(fieldName);
        auto itF = m_data->fieldsF.find(name);
        if (itF != m_data->fieldsF.end())
            return itF->second.count;
        auto itU8 = m_data->fieldsU8.find(name);
        if (itU8 != m_data->fieldsU8.end())
            return itU8->second.count;
        return 0;
    }

    int _getBufferDeviceImpl()
    {
        return m_data->deviceOrdinal;
    }

    void _registerFieldCallbackImpl(const char* fieldName, std::function<void()> callback)
    {
        const std::string name(fieldName);
        auto itF = m_data->fieldsF.find(name);
        if (itF != m_data->fieldsF.end())
        {
            itF->second.callback = std::move(callback);
            return;
        }
        auto itU8 = m_data->fieldsU8.find(name);
        if (itU8 != m_data->fieldsU8.end())
        {
            itU8->second.callback = std::move(callback);
            return;
        }
        FieldEntry<float> entry;
        entry.callback = std::move(callback);
        m_data->fieldsF.emplace(name, std::move(entry));
    }
};

// Macro to avoid duplicating the IXformDataView boilerplate across three classes.
#define IMPL_XFORM_DATA_VIEW                                                                                            \
    const float* getWorldPositions(int* outCount) override                                                              \
    {                                                                                                                   \
        return _fetchField("world_positions", outCount);                                                                \
    }                                                                                                                   \
    const float* getWorldOrientations(int* outCount) override                                                           \
    {                                                                                                                   \
        return _fetchField("world_orientations", outCount);                                                             \
    }                                                                                                                   \
    const float* getLocalTranslations(int* outCount) override                                                           \
    {                                                                                                                   \
        return _fetchField("local_translations", outCount);                                                             \
    }                                                                                                                   \
    const float* getLocalOrientations(int* outCount) override                                                           \
    {                                                                                                                   \
        return _fetchField("local_orientations", outCount);                                                             \
    }                                                                                                                   \
    const float* getLocalScales(int* outCount) override                                                                 \
    {                                                                                                                   \
        return _fetchField("local_scales", outCount);                                                                   \
    }                                                                                                                   \
    const float* getWorldPositionsHost(int* outCount) override                                                          \
    {                                                                                                                   \
        return _fetchFieldHost("world_positions", outCount);                                                            \
    }                                                                                                                   \
    const float* getWorldOrientationsHost(int* outCount) override                                                       \
    {                                                                                                                   \
        return _fetchFieldHost("world_orientations", outCount);                                                         \
    }                                                                                                                   \
    const float* getLocalTranslationsHost(int* outCount) override                                                       \
    {                                                                                                                   \
        return _fetchFieldHost("local_translations", outCount);                                                         \
    }                                                                                                                   \
    const float* getLocalOrientationsHost(int* outCount) override                                                       \
    {                                                                                                                   \
        return _fetchFieldHost("local_orientations", outCount);                                                         \
    }                                                                                                                   \
    const float* getLocalScalesHost(int* outCount) override                                                             \
    {                                                                                                                   \
        return _fetchFieldHost("local_scales", outCount);                                                               \
    }                                                                                                                   \
    Poses getWorldPoses() override                                                                                      \
    {                                                                                                                   \
        return _fetchWorldPosesImpl(false);                                                                             \
    }                                                                                                                   \
    Poses getWorldPosesHost() override                                                                                  \
    {                                                                                                                   \
        return _fetchWorldPosesImpl(true);                                                                              \
    }                                                                                                                   \
    bool getPrimFrameName(const char* primPath, char* outName, size_t maxLen) override                                  \
    {                                                                                                                   \
        if (!m_usdStage || !primPath || !outName || maxLen == 0)                                                        \
            return false;                                                                                               \
        pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(primPath));                                          \
        if (!prim)                                                                                                      \
            return false;                                                                                               \
        std::string name = isaacsim::core::includes::getName(prim);                                                     \
        std::strncpy(outName, name.c_str(), maxLen - 1);                                                                \
        outName[maxLen - 1] = '\0';                                                                                     \
        return true;                                                                                                    \
    }                                                                                                                   \
    bool getPrimWorldTransform(const char* primPath, float* outPos3, float* outOri4) override                           \
    {                                                                                                                   \
        if (!m_usdStage || !m_usdrtStage || !primPath || !outPos3 || !outOri4)                                          \
            return false;                                                                                               \
        pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(primPath));                                          \
        if (!prim)                                                                                                      \
            return false;                                                                                               \
        usdrt::GfMatrix4d xform =                                                                                       \
            isaacsim::core::includes::pose::computeWorldXformNoCache(m_usdStage, m_usdrtStage, pxr::SdfPath(primPath)); \
        decomposeMatrix(xform, outPos3, outOri4);                                                                       \
        return true;                                                                                                    \
    }

// Macro for the buffer/callback management methods shared by all view types.
#define IMPL_BUFFER_MANAGEMENT                                                                                         \
    bool update() override                                                                                             \
    {                                                                                                                  \
        return _updateImpl();                                                                                          \
    }                                                                                                                  \
    bool allocateBufferFloat(const char* fieldName, size_t count) override                                             \
    {                                                                                                                  \
        return _allocateBufferImpl<float>(fieldName, count);                                                           \
    }                                                                                                                  \
    bool allocateBufferUint8(const char* fieldName, size_t count) override                                             \
    {                                                                                                                  \
        return _allocateBufferImpl<uint8_t>(fieldName, count);                                                         \
    }                                                                                                                  \
    uintptr_t getBufferPtr(const char* fieldName) override                                                             \
    {                                                                                                                  \
        return _getBufferPtrImpl(fieldName);                                                                           \
    }                                                                                                                  \
    size_t getBufferSize(const char* fieldName) override                                                               \
    {                                                                                                                  \
        return _getBufferSizeImpl(fieldName);                                                                          \
    }                                                                                                                  \
    int getBufferDevice() override                                                                                     \
    {                                                                                                                  \
        return _getBufferDeviceImpl();                                                                                 \
    }                                                                                                                  \
    void registerFieldCallback(const char* fieldName, std::function<void()> callback) override                         \
    {                                                                                                                  \
        _registerFieldCallbackImpl(fieldName, std::move(callback));                                                    \
    }


/**
 * @class XformDataView
 * @brief Concrete view providing read-only access to XformPrim transform data.
 */
class XformDataView final : public IXformDataView, public BaseDataView
{
public:
    XformDataView(ViewData* data,
                  ISimulationManager* simulationManager,
                  pxr::UsdStageRefPtr usdStage,
                  usdrt::UsdStageRefPtr usdrtStage)
    {
        m_data = data;
        m_simulationManager = simulationManager;
        m_usdStage = std::move(usdStage);
        m_usdrtStage = std::move(usdrtStage);
    }
    IMPL_XFORM_DATA_VIEW
    IMPL_BUFFER_MANAGEMENT
};

/**
 * @class RigidBodyDataView
 * @brief Concrete view providing read-only access to RigidPrim data (transforms + velocities).
 */
class RigidBodyDataView final : public IRigidBodyDataView, public BaseDataView
{
public:
    RigidBodyDataView(ViewData* data,
                      ISimulationManager* simulationManager,
                      pxr::UsdStageRefPtr usdStage,
                      usdrt::UsdStageRefPtr usdrtStage)
    {
        m_data = data;
        m_simulationManager = simulationManager;
        m_usdStage = std::move(usdStage);
        m_usdrtStage = std::move(usdrtStage);
    }
    IMPL_XFORM_DATA_VIEW

    const float* getLinearVelocities(int* outCount) override
    {
        return _fetchField("linear_velocities", outCount);
    }
    const float* getAngularVelocities(int* outCount) override
    {
        return _fetchField("angular_velocities", outCount);
    }

    const float* getLinearVelocitiesHost(int* outCount) override
    {
        return _fetchFieldHost("linear_velocities", outCount);
    }
    const float* getAngularVelocitiesHost(int* outCount) override
    {
        return _fetchFieldHost("angular_velocities", outCount);
    }

    IMPL_BUFFER_MANAGEMENT
};

/**
 * @class ArticulationDataView
 * @brief Concrete view providing read-only access to Articulation data (transforms + DOF/link/dynamics).
 */
class ArticulationDataView final : public IArticulationDataView, public BaseDataView
{
public:
    ArticulationDataView(ViewData* data,
                         ISimulationManager* simulationManager,
                         pxr::UsdStageRefPtr usdStage,
                         usdrt::UsdStageRefPtr usdrtStage)
    {
        m_data = data;
        m_simulationManager = simulationManager;
        m_usdStage = std::move(usdStage);
        m_usdrtStage = std::move(usdrtStage);
    }
    IMPL_XFORM_DATA_VIEW

    const float* getDofPositions(int* outCount) override
    {
        return _fetchField("dof_positions", outCount);
    }
    const float* getDofVelocities(int* outCount) override
    {
        return _fetchField("dof_velocities", outCount);
    }
    const float* getDofEfforts(int* outCount) override
    {
        return _fetchField("dof_efforts", outCount);
    }
    const float* getRootTransforms(int* outCount) override
    {
        return _fetchField("root_transforms", outCount);
    }
    const float* getRootVelocities(int* outCount) override
    {
        return _fetchField("root_velocities", outCount);
    }
    const uint8_t* getDofTypes(int* outCount) override
    {
        return _fetchFieldU8("dof_types", outCount);
    }

    const float* getDofPositionsHost(int* outCount) override
    {
        return _fetchFieldHost("dof_positions", outCount);
    }
    const float* getDofVelocitiesHost(int* outCount) override
    {
        return _fetchFieldHost("dof_velocities", outCount);
    }
    const float* getDofEffortsHost(int* outCount) override
    {
        return _fetchFieldHost("dof_efforts", outCount);
    }
    const float* getRootTransformsHost(int* outCount) override
    {
        return _fetchFieldHost("root_transforms", outCount);
    }
    const float* getRootVelocitiesHost(int* outCount) override
    {
        return _fetchFieldHost("root_velocities", outCount);
    }
    const uint8_t* getDofTypesHost(int* outCount) override
    {
        return _fetchFieldU8Host("dof_types", outCount);
    }

    int getDofIndex(const char* dofPrimPath) override
    {
        if (!m_data || !m_data->physxArticulationView || !dofPrimPath)
            return -1;

        auto* articulationView = m_data->physxArticulationView;
        uint32_t maxDofs = articulationView->getMaxDofs();
        std::string target(dofPrimPath);

        for (uint32_t i = 0; i < maxDofs; ++i)
        {
            const char* path = articulationView->getUsdDofPath(0, i);
            if (path && target == path)
                return static_cast<int>(i);
        }
        return -1;
    }

    const char* const* getDofNames(int* outCount) override
    {
        if (outCount)
            *outCount = 0;
        if (!m_data || m_data->dofNamePtrs.empty())
            return nullptr;
        if (outCount)
            *outCount = static_cast<int>(m_data->dofNamePtrs.size());
        return m_data->dofNamePtrs.data();
    }

    bool getArticulationLinks(const char* rootPath, const LinkInfo** outLinks, size_t* outCount) override
    {
        *outLinks = nullptr;
        *outCount = 0;

        if (!m_usdStage || !rootPath)
            return false;

        pxr::SdfPath sdfRoot(rootPath);
        pxr::UsdPrim rootPrim = m_usdStage->GetPrimAtPath(sdfRoot);
        if (!rootPrim || !rootPrim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
            return false;

        // Collect all descendant link paths (prims with UsdPhysicsRigidBodyAPI).
        std::vector<std::string> linkPaths;
        for (const pxr::UsdPrim& p : pxr::UsdPrimRange(rootPrim))
        {
            if (p.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
                linkPaths.push_back(p.GetPath().GetString());
        }

        // For each link, find the closest ancestor that is also a link.
        m_linkInfoStrings.clear();
        m_linkInfoStrings.reserve(linkPaths.size() * 2);

        struct LinkEntry
        {
            size_t pathIdx;
            size_t parentPathIdx;
        };
        std::vector<LinkEntry> entries;
        entries.reserve(linkPaths.size());

        std::unordered_set<std::string> linkSet(linkPaths.begin(), linkPaths.end());

        for (const std::string& linkPathStr : linkPaths)
        {
            pxr::SdfPath linkSdfPath(linkPathStr);
            pxr::SdfPath ancestor = linkSdfPath.GetParentPath();
            std::string parentStr;
            while (ancestor != pxr::SdfPath::AbsoluteRootPath())
            {
                if (linkSet.count(ancestor.GetString()))
                {
                    parentStr = ancestor.GetString();
                    break;
                }
                ancestor = ancestor.GetParentPath();
            }

            size_t pathIdx = m_linkInfoStrings.size();
            m_linkInfoStrings.push_back(linkPathStr);
            size_t parentIdx = m_linkInfoStrings.size();
            m_linkInfoStrings.push_back(parentStr);

            entries.push_back({ pathIdx, parentIdx });
        }

        m_linkInfoCache.resize(entries.size());
        for (size_t i = 0; i < entries.size(); ++i)
        {
            m_linkInfoCache[i].path = m_linkInfoStrings[entries[i].pathIdx].c_str();
            m_linkInfoCache[i].parentPath = m_linkInfoStrings[entries[i].parentPathIdx].c_str();
        }

        *outLinks = m_linkInfoCache.empty() ? nullptr : m_linkInfoCache.data();
        *outCount = m_linkInfoCache.size();
        return true;
    }

    IMPL_BUFFER_MANAGEMENT

private:
    std::vector<std::string> m_linkInfoStrings;
    std::vector<LinkInfo> m_linkInfoCache;
};

#undef IMPL_XFORM_DATA_VIEW
#undef IMPL_BUFFER_MANAGEMENT

} // anonymous namespace


/**
 * @class PrimDataReaderImpl
 * @brief Implementation of the @c IPrimDataReader Carbonite interface.
 * @details Routes data access by engine:
 * - **PhysX**: Direct C++ TensorApi calls (no Python).
 * - **Newton**: Python callbacks fill C++-owned buffers.
 * - **Transforms**: IFabricHierarchy in C++ (engine-agnostic).
 */
class PrimDataReaderImpl : public IPrimDataReader
{
public:
    void initialize(long stageId, int deviceOrdinal) override
    {
        ++m_generation;
        m_stageId = stageId;
        m_deviceOrdinal = deviceOrdinal;
        m_simulationManager = carb::getCachedInterface<ISimulationManager>();
        m_tensorApi = carb::getCachedInterface<TensorApi>();

        PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
        m_usdStage = cache.Find(PXR_NS::UsdStageCache::Id::FromLongInt(stageId));

        // Subscribe to UsdContext::eClosing so we release m_usdStage / m_usdrtStage
        // before closeStageInternal's refcount sanity check (NVBug 6169671). Without
        // this the strong UsdStageRefPtr lives until shutdown() / next initialize(),
        // which keeps the stage alive past close and produces "Unexpected reference
        // count of 2 for UsdStage" warnings on every stage-close-after-play.
        _subscribeStageClosingEvent();

        if (m_usdStage)
        {
            omni::fabric::UsdStageId fabricStageId = { static_cast<uint64_t>(stageId) };
            omni::fabric::IStageReaderWriter* iStageReaderWriter =
                carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            if (iStageReaderWriter)
            {
                omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(fabricStageId);
                m_usdrtStage = usdrt::UsdStage::Attach(fabricStageId, stageInProgress);
            }
        }

        // Release stale views and simulation view before recreating.
        // After a timeline stop/play cycle within the same stage, the
        // PhysX simulation view is invalidated; we must recreate it.
        for (auto& [id, data] : m_viewData)
            _releasePhysxHandles(data);
        m_views.clear();
        m_viewData.clear();

        if (m_simulationView)
        {
            m_simulationView->release(true);
            m_simulationView = nullptr;
        }

        const char* activeEngine = isaacsim::core::includes::getActivePhysicsEngineName();
        if (m_tensorApi && stageId != 0 && activeEngine)
        {
            m_simulationView = m_tensorApi->createSimulationView(stageId, activeEngine);
            if (m_simulationView)
            {
                m_deviceOrdinal = m_simulationView->getDeviceOrdinal();
            }
        }
    }

    void shutdown() override
    {
        m_stageClosingObserver = {};

        for (auto& [id, data] : m_viewData)
            _releasePhysxHandles(data);
        m_views.clear();
        m_viewData.clear();

        if (m_simulationView)
        {
            m_simulationView->release(true);
            m_simulationView = nullptr;
        }
        m_usdrtStage = nullptr;
        m_usdStage = nullptr;
        m_tensorApi = nullptr;
        m_simulationManager = nullptr;
    }

    IXformDataView* createXformView(const char* viewId, const char** paths, size_t numPaths, const char* engineType) override
    {
        auto& data = _setupViewData(viewId, paths, numPaths, engineType, ViewType::eXform);
        _setupTransformCallbacks(data);

        auto view = std::make_unique<XformDataView>(&data, m_simulationManager, m_usdStage, m_usdrtStage);
        auto* ptr = view.get();
        m_views[viewId] = std::move(view);
        return ptr;
    }

    IRigidBodyDataView* createRigidBodyView(const char* viewId,
                                            const char** paths,
                                            size_t numPaths,
                                            const char* engineType) override
    {
        auto& data = _setupViewData(viewId, paths, numPaths, engineType, ViewType::eRigidBody);
        _setupTransformCallbacks(data);

        if (data.engine == EngineType::ePhysX)
            _setupPhysxRigidBodyCallbacks(data);

        auto view = std::make_unique<RigidBodyDataView>(&data, m_simulationManager, m_usdStage, m_usdrtStage);
        auto* ptr = view.get();
        m_views[viewId] = std::move(view);
        return ptr;
    }

    IArticulationDataView* createArticulationView(const char* viewId,
                                                  const char** paths,
                                                  size_t numPaths,
                                                  const char* engineType) override
    {
        auto& data = _setupViewData(viewId, paths, numPaths, engineType, ViewType::eArticulation);
        _setupTransformCallbacks(data);

        if (data.engine == EngineType::ePhysX)
            _setupPhysxArticulationCallbacks(data);

        auto view = std::make_unique<ArticulationDataView>(&data, m_simulationManager, m_usdStage, m_usdrtStage);
        auto* ptr = view.get();
        m_views[viewId] = std::move(view);
        return ptr;
    }

    void removeView(const char* viewId) override
    {
        auto dataIt = m_viewData.find(viewId);
        if (dataIt != m_viewData.end())
        {
            _releasePhysxHandles(dataIt->second);
        }
        m_views.erase(viewId);
        m_viewData.erase(viewId);
    }

    void setArticulationDofMetadata(
        const char* viewId, const char** names, size_t numNames, const uint8_t* types, size_t numTypes) override
    {
        if (!viewId)
            return;
        auto dataIt = m_viewData.find(viewId);
        if (dataIt == m_viewData.end() || dataIt->second.type != ViewType::eArticulation)
            return;
        ViewData& data = dataIt->second;
        data.dofNames.clear();
        data.dofNamePtrs.clear();
        if (names && numNames > 0)
        {
            data.dofNames.reserve(numNames);
            for (size_t i = 0; i < numNames; ++i)
                data.dofNames.push_back(names[i] ? names[i] : std::string());
            data.dofNamePtrs.resize(data.dofNames.size());
            for (size_t i = 0; i < data.dofNames.size(); ++i)
                data.dofNamePtrs[i] = data.dofNames[i].c_str();
        }
        if (types && numTypes > 0)
        {
            auto& dofTypesField = data.getOrCreateField<uint8_t>("dof_types", numTypes, -1);
            dofTypesField.buffer->copyFrom(types, numTypes);
            int64_t step = m_simulationManager ? static_cast<int64_t>(m_simulationManager->getNumPhysicsSteps()) : 0;
            dofTypesField.lastStep = step;
        }
    }

    uint64_t getGeneration() const override
    {
        return m_generation;
    }

    long getStageId() const override
    {
        return m_stageId;
    }

    int getDeviceOrdinal() const override
    {
        return m_deviceOrdinal;
    }

    bool enableContactReporting(const char* bodyPath) override
    {
        if (!bodyPath || !m_usdStage)
            return false;

        pxr::SdfPath sdfPath(bodyPath);
        pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(sdfPath);
        if (!prim.IsValid())
            return false;

        pxr::PhysxSchemaPhysxContactReportAPI contactReportAPI =
            pxr::PhysxSchemaPhysxContactReportAPI::Get(m_usdStage, sdfPath);
        if (!contactReportAPI)
            contactReportAPI = pxr::PhysxSchemaPhysxContactReportAPI::Apply(prim);

        if (contactReportAPI)
        {
            if (!contactReportAPI.GetReportPairsRel())
                contactReportAPI.CreateReportPairsRel();
            contactReportAPI.GetThresholdAttr().Set(0.0f);
        }

        pxr::PhysxSchemaPhysxRigidBodyAPI rigidBodyAPI = pxr::PhysxSchemaPhysxRigidBodyAPI::Get(m_usdStage, sdfPath);
        if (rigidBodyAPI)
            rigidBodyAPI.CreateSleepThresholdAttr(pxr::VtValue(0.0f));

        return true;
    }

    bool getContactReport(const char** bodyPaths, size_t numPaths, ContactReportData* outReport) override
    {
        m_contactEvents.clear();
        m_contactPoints.clear();

        if (outReport)
            *outReport = {};

        if (!bodyPaths && numPaths > 0)
            return false;

        auto* physxSim = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        if (!physxSim)
            return true;

        const omni::physx::ContactEventHeader* headers = nullptr;
        const omni::physx::ContactData* data = nullptr;
        const omni::physx::FrictionAnchor* frictionData = nullptr;
        uint32_t numContactData = 0;
        uint32_t numFrictionData = 0;
        uint32_t numHeaders =
            physxSim->getFullContactReport(&headers, &data, numContactData, &frictionData, numFrictionData);

        std::unordered_set<uint64_t> requestedTokens;
        for (size_t i = 0; i < numPaths; ++i)
        {
            if (bodyPaths[i])
                requestedTokens.insert(sdfPathToToken(pxr::SdfPath(bodyPaths[i])));
        }

        m_contactEvents.reserve(numHeaders);
        m_contactPoints.reserve(numContactData);

        uint32_t dataIndex = 0;
        for (uint32_t h = 0; h < numHeaders; h++)
        {
            const auto& header = headers[h];

            if (requestedTokens.count(header.actor0) == 0 && requestedTokens.count(header.actor1) == 0)
            {
                dataIndex += header.numContactData;
                continue;
            }

            for (uint32_t i = 0; i < header.numContactData; i++)
            {
                const auto& cd = data[dataIndex + i];
                ContactPointData cp;
                cp.positionX = cd.position.x;
                cp.positionY = cd.position.y;
                cp.positionZ = cd.position.z;
                cp.normalX = cd.normal.x;
                cp.normalY = cd.normal.y;
                cp.normalZ = cd.normal.z;
                cp.impulseX = cd.impulse.x;
                cp.impulseY = cd.impulse.y;
                cp.impulseZ = cd.impulse.z;
                m_contactPoints.push_back(cp);
            }
            dataIndex += header.numContactData;

            ContactEventData event;
            event.body0 = header.actor0;
            event.body1 = header.actor1;
            event.eventType = static_cast<uint32_t>(header.type);
            event.numContacts = header.numContactData;
            event.contacts = nullptr; // patched below
            m_contactEvents.push_back(event);
        }

        // Patch contact pointers now that the vector is fully built
        size_t pointOffset = 0;
        for (auto& ev : m_contactEvents)
        {
            ev.contacts = ev.numContacts > 0 ? m_contactPoints.data() + pointOffset : nullptr;
            pointOffset += ev.numContacts;
        }

        if (outReport)
        {
            outReport->events = m_contactEvents.data();
            outReport->numEvents = static_cast<uint32_t>(m_contactEvents.size());
            outReport->simTime =
                m_simulationManager ? static_cast<float>(m_simulationManager->getSimulationTime()) : 0.0f;
            // dt is left at 0.0f (from zero-init above): local PhysX doesn't expose per-contact-report dt.
            // The caller (ContactSensorImpl) falls back to its own step dt when outReport->dt == 0.
        }
        return true;
    }

private:
    static EngineType _parseEngine(const char* engineType)
    {
        if (engineType && std::string(engineType) == "newton")
            return EngineType::eNewton;
        return EngineType::ePhysX;
    }

    ViewData& _setupViewData(const char* viewId, const char** paths, size_t numPaths, const char* engineType, ViewType viewType)
    {
        auto& data = m_viewData[viewId];
        data.engine = _parseEngine(engineType);
        data.type = viewType;
        data.deviceOrdinal = m_deviceOrdinal;
        data.primPaths.clear();
        for (size_t i = 0; i < numPaths; ++i)
            data.primPaths.emplace_back(paths[i]);
        return data;
    }

    static void _releasePhysxHandles(ViewData& data)
    {
        if (data.physxArticulationView)
        {
            data.physxArticulationView->release();
            data.physxArticulationView = nullptr;
        }
        if (data.physxRigidBodyView)
        {
            data.physxRigidBodyView->release();
            data.physxRigidBodyView = nullptr;
        }
    }

    // ---- Transform callbacks: bulk PhysX tensor read for physics prims, Fabric fallback for the rest ----

    void _setupTransformCallbacks(ViewData& data)
    {
        if (!m_usdStage || !m_usdrtStage)
            return;

        size_t numPrims = data.primPaths.size();
        auto& worldPositionsField = data.getOrCreateField<float>("world_positions", numPrims * 3, -1);
        auto& worldOrientationsField = data.getOrCreateField<float>("world_orientations", numPrims * 4, -1);

        pxr::UsdStageRefPtr usdStage = m_usdStage;
        usdrt::UsdStageRefPtr usdrtStage = m_usdrtStage;
        std::vector<std::string> primPaths = data.primPaths;

        if (data.engine == EngineType::ePhysX && m_simulationView)
        {
            std::vector<std::string> physicsPaths;
            std::vector<size_t> physicsIndices;
            std::vector<size_t> fabricIndices;

            for (size_t i = 0; i < numPrims; ++i)
            {
                ObjectType ot = m_simulationView->getObjectType(primPaths[i].c_str());
                if (ot == ObjectType::eRigidBody || ot == ObjectType::eArticulationLink ||
                    ot == ObjectType::eArticulationRootLink)
                {
                    physicsIndices.push_back(i);
                    physicsPaths.push_back(primPaths[i]);
                }
                else
                {
                    fabricIndices.push_back(i);
                }
            }

            if (!physicsPaths.empty())
            {
                IRigidBodyView* rawView = m_simulationView->createRigidBodyView(physicsPaths);
                if (rawView && rawView->getCount() == static_cast<uint32_t>(physicsPaths.size()))
                {
                    auto rbView = std::shared_ptr<IRigidBodyView>(rawView, [](IRigidBodyView* v) { v->release(); });
                    int device = m_simulationView->getDeviceOrdinal();
                    size_t numPhysics = physicsPaths.size();

                    auto tensorBuffer = std::make_shared<includes::GenericBufferBase<float>>(numPhysics * 7, device);
                    auto hostStaging = std::make_shared<std::vector<float>>(numPhysics * 7);

                    if (fabricIndices.empty())
                    {
                        // All prims are physics — tight loop without index indirection or
                        // Fabric fallback.  Avoids capturing usdStage/usdrtStage/primPaths.
                        auto fillTransforms = [rbView, device, numPhysics, tensorBuffer, hostStaging,
                                               &worldPositionsField, &worldOrientationsField]()
                        {
                            TensorDesc desc;
                            fillTensorDesc(desc, tensorBuffer->data(), static_cast<int>(numPhysics * 7),
                                           TensorDataType::eFloat32, device);
                            rbView->getTransforms(&desc);
                            tensorBuffer->copyTo(hostStaging->data(), numPhysics * 7);

                            const float* tensorData = hostStaging->data();
                            float* positions = worldPositionsField.buffer->data();
                            float* orientations = worldOrientationsField.buffer->data();

                            for (size_t j = 0; j < numPhysics; ++j)
                            {
                                const float* src = tensorData + j * 7;
                                positions[j * 3 + 0] = src[0];
                                positions[j * 3 + 1] = src[1];
                                positions[j * 3 + 2] = src[2];
                                orientations[j * 4 + 0] = src[6]; // qw
                                orientations[j * 4 + 1] = src[3]; // qx
                                orientations[j * 4 + 2] = src[4]; // qy
                                orientations[j * 4 + 3] = src[5]; // qz
                            }
                        };

                        worldPositionsField.callback = fillTransforms;
                        worldOrientationsField.callback = fillTransforms;
                    }
                    else
                    {
                        // Mixed: some physics prims (tensor) + some non-physics (Fabric).
                        auto fillTransforms = [rbView, device, numPhysics, physicsIndices, fabricIndices, tensorBuffer,
                                               hostStaging, usdStage, usdrtStage, primPaths, &worldPositionsField,
                                               &worldOrientationsField]()
                        {
                            TensorDesc desc;
                            fillTensorDesc(desc, tensorBuffer->data(), static_cast<int>(numPhysics * 7),
                                           TensorDataType::eFloat32, device);
                            rbView->getTransforms(&desc);
                            tensorBuffer->copyTo(hostStaging->data(), numPhysics * 7);

                            const float* tensorData = hostStaging->data();
                            float* positions = worldPositionsField.buffer->data();
                            float* orientations = worldOrientationsField.buffer->data();

                            for (size_t j = 0; j < physicsIndices.size(); ++j)
                            {
                                size_t i = physicsIndices[j];
                                const float* src = tensorData + j * 7;
                                positions[i * 3 + 0] = src[0];
                                positions[i * 3 + 1] = src[1];
                                positions[i * 3 + 2] = src[2];
                                orientations[i * 4 + 0] = src[6]; // qw
                                orientations[i * 4 + 1] = src[3]; // qx
                                orientations[i * 4 + 2] = src[4]; // qy
                                orientations[i * 4 + 3] = src[5]; // qz
                            }

                            for (size_t idx : fabricIndices)
                            {
                                auto xform = includes::pose::computeWorldXformNoCache(
                                    usdStage, usdrtStage, pxr::SdfPath(primPaths[idx]));
                                decomposeMatrix(xform, positions + idx * 3, orientations + idx * 4);
                            }
                        };

                        worldPositionsField.callback = fillTransforms;
                        worldOrientationsField.callback = fillTransforms;
                    }
                    return;
                }
                else if (rawView)
                {
                    rawView->release();
                }
            }
        }

        // Fallback: pure Fabric path (Newton engine, no simulation view, or no physics prims matched)
        auto fillTransforms = [usdStage, usdrtStage, primPaths, &worldPositionsField, &worldOrientationsField]()
        {
            for (size_t i = 0; i < primPaths.size(); ++i)
            {
                auto xform = includes::pose::computeWorldXformNoCache(usdStage, usdrtStage, pxr::SdfPath(primPaths[i]));
                decomposeMatrix(
                    xform, worldPositionsField.buffer->data() + i * 3, worldOrientationsField.buffer->data() + i * 4);
            }
        };
        worldPositionsField.callback = fillTransforms;
        worldOrientationsField.callback = fillTransforms;
    }

    // ---- PhysX direct C++ callbacks ----

    /**
     * @brief Helper: create a C++ lambda that calls a PhysX TensorApi getter.
     * @details The lambda fills the FieldEntry buffer directly via a TensorDesc
     * pointing to the buffer's data pointer.
     */
    template <typename T>
    static std::function<void()> _makePhysxFieldCallbackT(FieldEntry<T>& field,
                                                          int device,
                                                          std::function<void(TensorDesc*)> tensorGetter)
    {
        return [&field, device, tensorGetter = std::move(tensorGetter)]()
        {
            TensorDesc desc;
            if constexpr (std::is_same_v<T, float>)
                fillTensorDesc(
                    desc, field.buffer->data(), static_cast<int>(field.count), TensorDataType::eFloat32, device);
            else if constexpr (std::is_same_v<T, uint8_t>)
                fillTensorDesc(desc, field.buffer->data(), static_cast<int>(field.count), TensorDataType::eUint8, -1);
            tensorGetter(&desc);
        };
    }

    void _setupPhysxArticulationCallbacks(ViewData& data)
    {
        if (!m_simulationView || data.primPaths.empty())
            return;

        data.physxArticulationView = m_simulationView->createArticulationView(data.primPaths);
        if (!data.physxArticulationView)
            return;

        IArticulationView* articulationView = data.physxArticulationView;
        int device = data.deviceOrdinal;
        uint32_t count = articulationView->getCount();
        uint32_t maxDofs = articulationView->getMaxDofs();
        uint32_t maxLinks = articulationView->getMaxLinks();

        auto& dofPositions = data.getOrCreateField<float>("dof_positions", count * maxDofs, device);
        dofPositions.callback = _makePhysxFieldCallbackT<float>(
            dofPositions, device, [articulationView](TensorDesc* d) { articulationView->getDofPositions(d); });

        auto& dofVelocities = data.getOrCreateField<float>("dof_velocities", count * maxDofs, device);
        dofVelocities.callback = _makePhysxFieldCallbackT<float>(
            dofVelocities, device, [articulationView](TensorDesc* d) { articulationView->getDofVelocities(d); });

        auto& dofEfforts = data.getOrCreateField<float>("dof_efforts", count * maxDofs, device);
        dofEfforts.callback = _makePhysxFieldCallbackT<float>(
            dofEfforts, device, [articulationView](TensorDesc* d) { articulationView->getDofProjectedJointForces(d); });

        auto& rootTransforms = data.getOrCreateField<float>("root_transforms", count * 7, device);
        rootTransforms.callback = _makePhysxFieldCallbackT<float>(
            rootTransforms, device, [articulationView](TensorDesc* d) { articulationView->getRootTransforms(d); });

        auto& rootVelocities = data.getOrCreateField<float>("root_velocities", count * 6, device);
        rootVelocities.callback = _makePhysxFieldCallbackT<float>(
            rootVelocities, device, [articulationView](TensorDesc* d) { articulationView->getRootVelocities(d); });

        auto& linkMasses = data.getOrCreateField<float>("link_masses", count * maxLinks, device);
        linkMasses.callback = _makePhysxFieldCallbackT<float>(
            linkMasses, device, [articulationView](TensorDesc* d) { articulationView->getMasses(d); });

        uint32_t jacobianRows = 0;
        uint32_t jacobianColumns = 0;
        if (articulationView->getJacobianShape(&jacobianRows, &jacobianColumns))
        {
            auto& jacobians = data.getOrCreateField<float>("jacobians", count * jacobianRows * jacobianColumns, device);
            jacobians.callback = _makePhysxFieldCallbackT<float>(
                jacobians, device, [articulationView](TensorDesc* d) { articulationView->getJacobians(d); });
        }

        uint32_t massMatrixRows = 0;
        uint32_t massMatrixColumns = 0;
        if (articulationView->getGeneralizedMassMatrixShape(&massMatrixRows, &massMatrixColumns))
        {
            auto& massMatrices =
                data.getOrCreateField<float>("mass_matrices", count * massMatrixRows * massMatrixColumns, device);
            massMatrices.callback = _makePhysxFieldCallbackT<float>(
                massMatrices, device,
                [articulationView](TensorDesc* d) { articulationView->getGeneralizedMassMatrices(d); });
        }

        // DOF metadata: names from USD, types via field callback (same pattern as other getters).
        data.dofNames.clear();
        data.dofNamePtrs.clear();
        if (m_usdStage)
        {
            for (uint32_t j = 0; j < maxDofs; ++j)
            {
                const char* path = articulationView->getUsdDofPath(0, j);
                if (path)
                {
                    pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(pxr::SdfPath(path));
                    if (prim.IsValid())
                        data.dofNames.push_back(prim.GetName());
                    else
                        data.dofNames.push_back(std::string());
                }
                else
                {
                    data.dofNames.push_back(std::string());
                }
            }
            data.dofNamePtrs.resize(data.dofNames.size());
            for (size_t i = 0; i < data.dofNames.size(); ++i)
                data.dofNamePtrs[i] = data.dofNames[i].c_str();
        }
        if (maxDofs > 0)
        {
            auto& dofTypesField = data.getOrCreateField<uint8_t>("dof_types", maxDofs, -1);
            dofTypesField.callback = _makePhysxFieldCallbackT<uint8_t>(
                dofTypesField, -1, [articulationView](TensorDesc* d) { articulationView->getDofTypes(d); });
        }
    }

    void _setupPhysxRigidBodyCallbacks(ViewData& data)
    {
        if (!m_simulationView)
            return;

        data.physxRigidBodyView = m_simulationView->createRigidBodyView(data.primPaths);
        if (!data.physxRigidBodyView)
            return;

        IRigidBodyView* rigidBody = data.physxRigidBodyView;
        int device = data.deviceOrdinal;
        uint32_t count = rigidBody->getCount();

        // Transforms: float[N][7] -- split into separate velocity fields after fetch
        auto& transforms = data.getOrCreateField<float>("rigid_transforms_raw", count * 7, device);
        transforms.callback = _makePhysxFieldCallbackT<float>(
            transforms, device, [rigidBody](TensorDesc* d) { rigidBody->getTransforms(d); });

        // Velocities: float[N][6]
        auto& velocities = data.getOrCreateField<float>("rigid_velocities_raw", count * 6, device);
        velocities.callback = _makePhysxFieldCallbackT<float>(
            velocities, device, [rigidBody](TensorDesc* d) { rigidBody->getVelocities(d); });

        // Expose split linear/angular velocity fields by referencing the raw buffer
        auto& linearVelocity = data.getOrCreateField<float>("linear_velocities", count * 3, device);
        auto& angularVelocity = data.getOrCreateField<float>("angular_velocities", count * 3, device);
        linearVelocity.callback = [&velocities, &linearVelocity, &angularVelocity, count, device]()
        {
            if (velocities.callback)
                velocities.callback();
            float* source = velocities.buffer->data();
            float* destinationLinear = linearVelocity.buffer->data();
            float* destinationAngular = angularVelocity.buffer->data();
            if (device >= 0)
            {
                includes::ScopedDevice scopedDevice(device);
                CUDA_CHECK(cudaMemcpy2D(destinationLinear, 3 * sizeof(float), source, 6 * sizeof(float),
                                        3 * sizeof(float), count, cudaMemcpyDeviceToDevice));
                CUDA_CHECK(cudaMemcpy2D(destinationAngular, 3 * sizeof(float), source + 3, 6 * sizeof(float),
                                        3 * sizeof(float), count, cudaMemcpyDeviceToDevice));
            }
            else
            {
                for (size_t i = 0; i < count; ++i)
                {
                    destinationLinear[i * 3 + 0] = source[i * 6 + 0];
                    destinationLinear[i * 3 + 1] = source[i * 6 + 1];
                    destinationLinear[i * 3 + 2] = source[i * 6 + 2];
                    destinationAngular[i * 3 + 0] = source[i * 6 + 3];
                    destinationAngular[i * 3 + 1] = source[i * 6 + 4];
                    destinationAngular[i * 3 + 2] = source[i * 6 + 5];
                }
            }
        };
        angularVelocity.callback = linearVelocity.callback;
    }

    void _subscribeStageClosingEvent()
    {
        // Drop the strong UsdStageRefPtr in m_usdStage before
        // UsdContext::closeStageInternal runs its refcount sanity check
        // (NVBug 6169671). Kit fires eClosing synchronously from closeStageTask
        // and waits for all observers before calling closeStageInternal.
        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
        if (!ed || !usdContext)
        {
            return;
        }

        static const carb::RStringKey kObserverName("isaacsim.core.experimental.primdata.PrimDataReader");

        m_stageClosingObserver = ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                                  usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                                                  [this](const carb::eventdispatcher::Event&)
                                                  {
                                                      if (m_simulationView)
                                                      {
                                                          m_simulationView->release(true);
                                                          m_simulationView = nullptr;
                                                      }
                                                      m_usdrtStage = nullptr;
                                                      m_usdStage = nullptr;
                                                  });
    }

    long m_stageId = 0;
    int m_deviceOrdinal = -1;
    uint64_t m_generation = 0;
    ISimulationManager* m_simulationManager = nullptr;
    TensorApi* m_tensorApi = nullptr;
    ISimulationView* m_simulationView = nullptr;
    pxr::UsdStageRefPtr m_usdStage;
    usdrt::UsdStageRefPtr m_usdrtStage;
    carb::eventdispatcher::ObserverGuard m_stageClosingObserver;

    std::unordered_map<std::string, ViewData> m_viewData;
    std::unordered_map<std::string, std::unique_ptr<IXformDataView>> m_views;

    std::vector<ContactEventData> m_contactEvents;
    std::vector<ContactPointData> m_contactPoints;
};

/**
 * @class PrimDataReaderManagerImpl
 * @brief Implementation of @c IPrimDataReaderManager.
 * @details Centralizes lifecycle management for the shared @c IPrimDataReader instance,
 * ensuring that sensor plugins and nodes do not call initialize() independently.
 * Subscribes to physics simulation events to trigger reinitialization on timeline
 * stop/resume.
 */
class PrimDataReaderManagerImpl : public IPrimDataReaderManager
{
public:
    PrimDataReaderManagerImpl()
    {
        _subscribeToPhysicsEvents();
    }

    ~PrimDataReaderManagerImpl()
    {
        m_physicsEventSubscription.reset();
    }

    bool ensureInitialized(long stageId, int deviceOrdinal) override
    {
        if (stageId == 0)
            return false;

        if (!m_reader)
            m_reader = carb::getCachedInterface<IPrimDataReader>();

        if (!m_reader)
            return false;

        const bool needsInit =
            !m_initialized || m_forceReinitialize || stageId != m_lastStageId || deviceOrdinal != m_lastDeviceOrdinal;
        if (needsInit)
        {
            m_reader->initialize(stageId, deviceOrdinal);
            m_lastStageId = stageId;
            m_lastDeviceOrdinal = deviceOrdinal;
            m_lastGeneration = m_reader->getGeneration();
            m_initialized = true;
            m_forceReinitialize = false;
        }
        else
        {
            m_lastGeneration = m_reader->getGeneration();
        }

        return true;
    }

    IPrimDataReader* getReader() override
    {
        if (!m_reader)
            m_reader = carb::getCachedInterface<IPrimDataReader>();
        return m_reader;
    }

    uint64_t getGeneration() const override
    {
        if (!m_reader)
            return m_lastGeneration;
        return m_reader->getGeneration();
    }

private:
    void _subscribeToPhysicsEvents()
    {
        if (m_physicsEventSubscription)
            return;

        auto* physicsStageUpdate = carb::getCachedInterface<omni::physics::IPhysicsStageUpdate>();
        if (!physicsStageUpdate)
            return;

        m_physicsEventSubscription = carb::events::createSubscriptionToPop(
            physicsStageUpdate->getSimulationEventStream().get(),
            [this](carb::events::IEvent* e)
            {
                if (e->type == omni::physics::SimulationEvent::eStopped ||
                    e->type == omni::physics::SimulationEvent::eResumed)
                {
                    m_forceReinitialize = true;
                }
            },
            0, "IsaacSim.Core.Experimental.Prims.ReaderManager.SimulationEvent");
    }

    IPrimDataReader* m_reader = nullptr;
    carb::events::ISubscriptionPtr m_physicsEventSubscription;
    long m_lastStageId = 0;
    int m_lastDeviceOrdinal = -1;
    uint64_t m_lastGeneration = 0;
    bool m_initialized = false;
    bool m_forceReinitialize = true;
};

/**
 * @class Extension
 * @brief Omniverse extension entry point for the prim data reader plugin.
 */
class Extension : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override
    {
    }

    void onShutdown() override
    {
    }
};

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim

CARB_EXPORT void carbOnPluginStartup()
{
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

CARB_PLUGIN_IMPL(g_kPluginDesc,
                 isaacsim::core::experimental::prims::PrimDataReaderImpl,
                 isaacsim::core::experimental::prims::PrimDataReaderManagerImpl,
                 isaacsim::core::experimental::prims::Extension)
CARB_PLUGIN_IMPL_DEPS(omni::physics::IPhysics)

void fillInterface(isaacsim::core::experimental::prims::PrimDataReaderImpl& iface)
{
}

void fillInterface(isaacsim::core::experimental::prims::Extension& iface)
{
}

void fillInterface(isaacsim::core::experimental::prims::PrimDataReaderManagerImpl& iface)
{
}
