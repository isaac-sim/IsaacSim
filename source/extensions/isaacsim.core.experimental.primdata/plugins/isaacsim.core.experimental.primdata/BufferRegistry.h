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

#pragma once

#include <isaacsim/core/includes/Buffer.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physics
{
namespace tensors
{
struct ISimulationView;
struct IArticulationView;
struct IRigidBodyView;
} // namespace tensors
} // namespace physics
} // namespace omni

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

/**
 * @struct FieldEntry
 * @brief Per-field state: buffer, host staging copy, fill callback, and dirty tracking.
 * @details Each named field in a view owns a GPU/CPU buffer, an optional CPU staging
 * buffer for Host variants, and a callback that fills the buffer from the physics backend.
 * The @c lastStep field tracks the most recent physics step at which the buffer was filled;
 * a value of @c -1 indicates the field has never been fetched.
 * @tparam T Element type (e.g. float for DOF/transform data, uint8_t for dof_types).
 */
template <typename T>
struct FieldEntry
{
    std::unique_ptr<includes::GenericBufferBase<T>> buffer;
    std::unique_ptr<includes::GenericBufferBase<T>> hostStaging;
    std::function<void()> callback;
    int64_t lastStep = -1; ///< -1 sentinel means "never fetched"
    int64_t hostLastStep = -1; ///< -1 sentinel means "host staging not copied yet"
    size_t count = 0;
};

/// Common instantiations
using FieldEntryFloat = FieldEntry<float>;
using FieldEntryU8 = FieldEntry<uint8_t>;

/**
 * @enum EngineType
 * @brief Physics engine backend type.
 */
enum class EngineType
{
    ePhysX,
    eNewton
};

/**
 * @enum ViewType
 * @brief Kind of prim data view, determining which getters are valid.
 */
enum class ViewType
{
    eXform,
    eRigidBody,
    eArticulation
};

/**
 * @class ViewData
 * @brief Internal storage for one named view: engine type, device, prim paths, and field buffers.
 * @details Owned by the @c PrimDataReaderImpl and shared by pointer with the concrete
 * view objects. For PhysX views, this also stores the native PhysX tensor API handles
 * so that C++ lambdas can call TensorApi directly without Python involvement.
 */
class ViewData
{
public:
    EngineType engine = EngineType::ePhysX;
    ViewType type = ViewType::eXform;
    int deviceOrdinal = -1;
    std::unordered_map<std::string, FieldEntry<float>> fieldsF;
    std::unordered_map<std::string, FieldEntry<uint8_t>> fieldsU8;
    std::vector<std::string> primPaths;

    /// DOF names for articulation views (in DOF index order).
    std::vector<std::string> dofNames;
    std::vector<const char*> dofNamePtrs;

    /// PhysX tensor view handles (null for Newton).
    omni::physics::tensors::IArticulationView* physxArticulationView = nullptr;
    omni::physics::tensors::IRigidBodyView* physxRigidBodyView = nullptr;

    /**
     * @brief Get an existing field or create a new one with the given buffer size.
     * @tparam T Element type (e.g. float, uint8_t).
     * @param[in] name   Field name (e.g., "dof_positions").
     * @param[in] count Number of elements in the buffer.
     * @param[in] device CUDA device ordinal (-1 for CPU, >=0 for GPU).
     * @return Reference to the (possibly new) field entry.
     */
    template <typename T>
    FieldEntry<T>& getOrCreateField(const std::string& name, size_t count, int device)
    {
        static_assert(std::is_same_v<T, float> || std::is_same_v<T, uint8_t>,
                      "Invalid field type: only float and uint8_t are supported");
        if constexpr (std::is_same_v<T, float>)
            return getOrCreateFieldImpl(fieldsF, name, count, device);
        else
            return getOrCreateFieldImpl(fieldsU8, name, count, device);
    }

private:
    template <typename T>
    static FieldEntry<T>& getOrCreateFieldImpl(std::unordered_map<std::string, FieldEntry<T>>& fields,
                                               const std::string& name,
                                               size_t count,
                                               int device)
    {
        auto it = fields.find(name);
        if (it == fields.end())
        {
            FieldEntry<T> entry;
            entry.buffer = std::make_unique<includes::GenericBufferBase<T>>(count, device);
            entry.count = count;
            auto result = fields.emplace(name, std::move(entry));
            return result.first->second;
        }
        if (it->second.count < count)
        {
            it->second.buffer->resize(count);
            it->second.count = count;
        }
        return it->second;
    }
};

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim
