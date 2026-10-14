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

#pragma once

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/physics/manager/Export.h>
#include <isaacsim/physics/registration/tensors/ArrayTensorSpecification.hpp>
#include <isaacsim/physics/registration/tensors/IEntityView.hpp>
#include <isaacsim/physics/registration/tensors/Metadata.hpp>
#include <isaacsim/physics/registration/tensors/TensorConversion.hpp>
#include <isaacsim/physics/registration/tensors/TensorTypes.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace isaacsim
{
namespace physics
{
namespace tensors
{

/** @brief Descriptor callbacks preserve caller storage and layout. */
using GetImplementationFunction =
    std::function<TensorDescription(const TensorDescription& indices, const TensorDescription& output)>;
using SetImplementationFunction = std::function<void(const TensorDescription& data, const TensorDescription& indices)>;
using GetMultiImplementationFunction = std::function<std::vector<TensorDescription>(
    const TensorDescription& indices, const std::vector<TensorDescription>& output)>;
using SetMultiImplementationFunction =
    std::function<void(const std::vector<TensorDescription>& data, const TensorDescription& indices)>;

/**
 * @brief Callback that reads one tensor from an entity view as an array.
 *
 * The registered operation defines the selector or query semantics and data type of `indices`. An absent selector
 * requests all entities only for operations that adopt that convention. The callback may write through `output` or
 * return an array over other storage.
 *
 * @param[in] indices Operation-defined selector or query, or `std::nullopt` to request all entities.
 * @param[in,out] output Optional destination. The callback may write through its storage but does not rebind it.
 * @return The resulting array. It may alias the storage supplied by `output` or provider-selected storage, which the
 *         provider is free to overwrite on the next call.
 */
using GetArrayImplementationFunction =
    std::function<array::Array(const std::optional<array::Array>& indices, const std::optional<array::Array>& output)>;

/**
 * @brief Callback that writes one tensor to an entity view from an array.
 *
 * The registered operation defines the selector semantics and data type of `indices`. An absent selector selects all
 * entities only for operations that adopt that convention.
 *
 * @param[in] data Source array.
 * @param[in] indices Operation-defined selector, or `std::nullopt` to select all entities.
 */
using SetArrayImplementationFunction =
    std::function<void(const array::Array& data, const std::optional<array::Array>& indices)>;

/**
 * @brief Callback that reads multiple tensors from an entity view as arrays.
 *
 * The registered operation defines the selector or query semantics and data type of `indices`. An absent selector
 * requests all entities only for operations that adopt that convention.
 *
 * @param[in] indices Operation-defined selector or query, or `std::nullopt` to request all entities.
 * @param[in,out] output Optional destinations, one per returned buffer, supplied either in full or not at all. The
 *                       callback may write through their storage but does not rebind them.
 * @return The resulting arrays. They may alias the storage supplied by `output` or provider-selected storage, which
 *         the provider is free to overwrite on the next call.
 */
using GetArrayMultiImplementationFunction = std::function<std::vector<array::Array>(
    const std::optional<array::Array>& indices, const std::optional<std::vector<array::Array>>& output)>;

/**
 * @brief Callback that writes multiple tensors to an entity view from arrays.
 *
 * The registered operation defines the selector semantics and data type of `indices`. An absent selector selects all
 * entities only for operations that adopt that convention.
 *
 * @param[in] data Source arrays.
 * @param[in] indices Operation-defined selector, or `std::nullopt` to select all entities.
 */
using SetArrayMultiImplementationFunction =
    std::function<void(const std::vector<array::Array>& data, const std::optional<array::Array>& indices)>;

/**
 * @brief Callback that returns provider-specific entity-view metadata.
 *
 * @return Provider-specific metadata.
 */
using MetadataImplementationFunction = std::function<Metadata()>;

/**
 * @class EntityView
 * @brief Dispatches named tensor operations for an engine-specific collection of physics entities.
 *
 * Engine implementations register read, write, and metadata callbacks together with tensor specifications. Consumers
 * discover the registered operations by name and invoke them through the uniform data-access methods. The view stores
 * callback objects by value.
 *
 * Registration accepts empty callback objects. Invoking an empty registered callback throws `std::bad_function_call`.
 * Other exceptions raised by provider callbacks propagate to the caller.
 */
class ISAACSIM_PHYSICS_MANAGER_API EntityView : public IEntityView
{
public:
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    bool registerImplementation(const std::string& operationName,
                                ImplementationKind kind,
                                GetImplementationFunction callback,
                                TensorSpecification specification);
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    bool registerImplementation(const std::string& operationName,
                                ImplementationKind kind,
                                SetImplementationFunction callback,
                                TensorSpecification specification);
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    bool registerImplementation(const std::string& operationName,
                                ImplementationKind kind,
                                GetMultiImplementationFunction callback,
                                TensorSpecification specification);
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    bool registerImplementation(const std::string& operationName,
                                ImplementationKind kind,
                                SetMultiImplementationFunction callback,
                                TensorSpecification specification);
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    bool registerImplementation(const std::string& operationName,
                                ImplementationKind kind,
                                GetMultiImplementationFunction callback,
                                std::vector<TensorSpecification> outputSpecifications);
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    TensorSpecification getImplementationSpecification(const std::string& operationName, ImplementationKind kind) const;
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    std::vector<TensorSpecification> getMultiImplementationSpecifications(const std::string& operationName,
                                                                          ImplementationKind kind) const;
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    TensorDescription getData(const std::string& operationName,
                              const TensorDescription& indices,
                              const TensorDescription& output) const;
    /**
     * @brief Dispatch a descriptor-based multi-buffer read.
     * @note Array-backed providers allocate omitted destinations through their no-output route.
     * Supplied destinations must match result shapes and dtypes and have writable layouts. All
     * destinations are validated before copying; copies may cross devices. Mixed output lists
     * snapshot provider results before writes to protect cross-slot aliases. Results retain supplied
     * destinations or owned snapshots for omitted slots; all-unspecified lists retain provider storage.
     */
    std::vector<TensorDescription> getDataMulti(const std::string& operationName,
                                                const TensorDescription& indices,
                                                const std::vector<TensorDescription>& output) const;
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    void setData(const std::string& operationName, const TensorDescription& data, const TensorDescription& indices) const;
    /** @brief Dispatch or describe a descriptor-based tensor operation. */
    void setDataMulti(const std::string& operationName,
                      const std::vector<TensorDescription>& data,
                      const TensorDescription& indices) const;
    /**
     * @brief Constructs an entity view without path patterns.
     */
    EntityView();

    /**
     * @brief Constructs an entity view from prim-path patterns.
     *
     * @param[in] primPathPatterns Prim-path patterns or explicit prim paths represented by the view.
     */
    explicit EntityView(std::vector<std::string> primPathPatterns);

    /**
     * @brief Destroys the entity view and its registered callbacks.
     */
    ~EntityView() override;

    /**
     * @brief Entity views cannot be copied.
     */
    EntityView(const EntityView&) = delete;

    /**
     * @brief Entity views cannot be copy-assigned.
     */
    EntityView& operator=(const EntityView&) = delete;

    /**
     * @brief Registers a single-buffer read operation.
     *
     * Registering the same name and operation kind again replaces the existing callback and specification. A read
     * operation name cannot be registered as both single-buffer and multi-buffer.
     *
     * @param[in] operationName Name used to discover and invoke the operation.
     * @param[in] kind Operation kind, which must be @c ImplementationKind::eGet.
     * @param[in] callback Read callback to store.
     * @param[in] specification Tensor capabilities and layout advertised by the callback.
     * @return `true` for a new registration; `false` when an existing registration was replaced.
     * @throws std::invalid_argument If `kind` is not @c ImplementationKind::eGet or `operationName` is already
     *         registered as a multi-buffer read operation.
     */
    bool registerArrayImplementation(const std::string& operationName,
                                     ImplementationKind kind,
                                     GetArrayImplementationFunction callback,
                                     ArrayTensorSpecification specification);

    /**
     * @brief Registers a single-buffer write operation.
     *
     * Registering the same name and operation kind again replaces the existing callback and specification. A write
     * operation name cannot be registered as both single-buffer and multi-buffer.
     *
     * @param[in] operationName Name used to discover and invoke the operation.
     * @param[in] kind Operation kind, which must be @c ImplementationKind::eSet.
     * @param[in] callback Write callback to store.
     * @param[in] specification Tensor capabilities and layout advertised by the callback.
     * @return `true` for a new registration; `false` when an existing registration was replaced.
     * @throws std::invalid_argument If `kind` is not @c ImplementationKind::eSet or `operationName` is already
     *         registered as a multi-buffer write operation.
     */
    bool registerArrayImplementation(const std::string& operationName,
                                     ImplementationKind kind,
                                     SetArrayImplementationFunction callback,
                                     ArrayTensorSpecification specification);

    /**
     * @brief Registers a multi-buffer read operation with one aggregate specification.
     *
     * Registering the same name and operation kind again replaces the existing callback and specification. A read
     * operation name cannot be registered as both single-buffer and multi-buffer.
     *
     * @param[in] operationName Name used to discover and invoke the operation.
     * @param[in] kind Operation kind, which must be @c ImplementationKind::eGet.
     * @param[in] callback Multi-buffer read callback to store.
     * @param[in] specification Aggregate tensor capabilities advertised by the callback.
     * @return `true` for a new registration; `false` when an existing registration was replaced.
     * @throws std::invalid_argument If `kind` is not @c ImplementationKind::eGet or `operationName` is already
     *         registered as a single-buffer read operation.
     */
    bool registerArrayImplementation(const std::string& operationName,
                                     ImplementationKind kind,
                                     GetArrayMultiImplementationFunction callback,
                                     ArrayTensorSpecification specification);

    /**
     * @brief Registers a multi-buffer write operation.
     *
     * Registering the same name and operation kind again replaces the existing callback and specification. A write
     * operation name cannot be registered as both single-buffer and multi-buffer.
     *
     * @param[in] operationName Name used to discover and invoke the operation.
     * @param[in] kind Operation kind, which must be @c ImplementationKind::eSet.
     * @param[in] callback Multi-buffer write callback to store.
     * @param[in] specification Aggregate tensor capabilities advertised by the callback.
     * @return `true` for a new registration; `false` when an existing registration was replaced.
     * @throws std::invalid_argument If `kind` is not @c ImplementationKind::eSet or `operationName` is already
     *         registered as a single-buffer write operation.
     */
    bool registerArrayImplementation(const std::string& operationName,
                                     ImplementationKind kind,
                                     SetArrayMultiImplementationFunction callback,
                                     ArrayTensorSpecification specification);

    /**
     * @brief Registers a metadata provider.
     *
     * @param[in] operationName Name used to retrieve the metadata.
     * @param[in] callback Metadata callback to store.
     * @return `true` for a new registration; `false` when an existing callback was replaced.
     */
    bool registerMetadata(const std::string& operationName, MetadataImplementationFunction callback);

    /**
     * @brief Registers a multi-buffer read operation with a specification for each output.
     *
     * The first output specification supplies the aggregate support, data type, shape, indexed-read, and host-data
     * properties used to gate the operation. Registering the same name again replaces the existing multi-buffer read
     * callback and specifications. An empty `outputSpecifications` vector registers the operation with aggregate
     * `supports` set to `false`.
     *
     * @param[in] operationName Name used to discover and invoke the operation.
     * @param[in] kind Operation kind, which must be @c ImplementationKind::eGet.
     * @param[in] callback Multi-buffer read callback to store.
     * @param[in] outputSpecifications Tensor specification for each returned buffer.
     * @return `true` for a new registration; `false` when an existing registration was replaced.
     * @throws std::invalid_argument If `kind` is not @c ImplementationKind::eGet or `operationName` is already
     *         registered as a single-buffer read operation.
     */
    bool registerArrayImplementation(const std::string& operationName,
                                     ImplementationKind kind,
                                     GetArrayMultiImplementationFunction callback,
                                     std::vector<ArrayTensorSpecification> outputSpecifications);

    /**
     * @brief Lists registered operation names for a read or write kind.
     *
     * The order of returned names is unspecified. The list includes operations whose tensor specification has
     * `supports` set to `false`.
     *
     * @param[in] kind @c ImplementationKind::eGet to list read operations or @c ImplementationKind::eSet to list
     *                 write operations.
     * @return Names of registered single-buffer and multi-buffer operations for `kind`.
     */
    std::vector<std::string> listImplementations(ImplementationKind kind) const;

    /**
     * @brief Checks whether a supported operation is registered.
     *
     * @param[in] operationName Operation name to query.
     * @param[in] kind Read or write operation kind to query.
     * @return `true` if the operation is registered and its specification has `supports` set to `true`; otherwise,
     *         `false`.
     */
    bool hasImplementation(const std::string& operationName, ImplementationKind kind) const;

    /**
     * @brief Gets the aggregate tensor specification for an operation.
     *
     * @param[in] operationName Operation name to query.
     * @param[in] kind Read or write operation kind to query.
     * @return The operation's tensor specification.
     * @throws std::out_of_range If no matching operation is registered.
     */
    ArrayTensorSpecification getArrayImplementationSpecification(const std::string& operationName,
                                                                 ImplementationKind kind) const;

    /**
     * @brief Gets per-output specifications for a multi-buffer read operation.
     *
     * @param[in] operationName Operation name to query.
     * @param[in] kind Operation kind, expected to be @c ImplementationKind::eGet.
     * @return Per-output specifications, or an empty vector if the operation was registered with one aggregate
     *         specification, is not a multi-buffer read operation, or is not registered.
     */
    std::vector<ArrayTensorSpecification> getArrayMultiImplementationSpecifications(const std::string& operationName,
                                                                                    ImplementationKind kind) const;

    /**
     * @brief Gets metadata from a registered provider.
     *
     * @param[in] operationName Metadata-provider name to query.
     * @return The provider's metadata, or a default-constructed `Metadata` value if no provider is registered.
     * @throws std::bad_function_call If the registered metadata callback is empty.
     *
     * @note Exceptions raised by the metadata callback propagate to the caller.
     */
    Metadata getMetadata(const std::string& operationName) const;

    /**
     * @brief Invokes a registered single-buffer read operation.
     *
     * @param[in] operationName Read-operation name.
     * @param[in] indices Operation-defined selector or query, or `std::nullopt` to request all entities, which is
     *                    honored only when the registered operation defines that convention.
     * @param[in,out] output Optional destination supplied to the provider. The provider may write through its storage
     *                       but does not rebind it.
     * @return The resulting array. It may alias the storage supplied by `output` or provider-selected storage.
     * @throws std::out_of_range If no matching single-buffer read operation is registered.
     * @throws std::runtime_error If the operation is registered but not supported by the provider.
     * @throws std::invalid_argument If `indices` is present and the operation does not support indexed reads.
     * @throws std::bad_function_call If the registered provider callback is empty.
     *
     * @note Other exceptions raised by the provider callback propagate to the caller.
     */
    array::Array getArrayData(const std::string& operationName,
                              const std::optional<array::Array>& indices = std::nullopt,
                              const std::optional<array::Array>& output = std::nullopt) const;

    /**
     * @brief Invokes a registered multi-buffer read operation.
     *
     * @param[in] operationName Read-operation name.
     * @param[in] indices Operation-defined selector or query, or `std::nullopt` to request all entities, which is
     *                    honored only when the registered operation defines that convention.
     * @param[in,out] output Optional destinations, one per returned buffer, supplied either in full or not at all.
     *                       The provider may write through their storage but does not rebind them.
     * @return The resulting arrays. They may alias the storage supplied by `output` or provider-selected storage.
     * @throws std::out_of_range If no matching multi-buffer read operation is registered.
     * @throws std::runtime_error If the operation is registered but not supported by the provider.
     * @throws std::invalid_argument If `indices` is present and the operation does not support indexed reads.
     * @throws std::bad_function_call If the registered provider callback is empty.
     *
     * @note Other exceptions raised by the provider callback propagate to the caller.
     */
    std::vector<array::Array> getArrayDataMulti(const std::string& operationName,
                                                const std::optional<array::Array>& indices = std::nullopt,
                                                const std::optional<std::vector<array::Array>>& output = std::nullopt) const;

    /**
     * @brief Invokes a registered single-buffer write operation.
     *
     * @param[in] operationName Write-operation name.
     * @param[in] data Source array.
     * @param[in] indices Operation-defined selector, or `std::nullopt` to select all entities, which is honored only
     *                    when the registered operation defines that convention.
     * @throws std::out_of_range If no matching single-buffer write operation is registered.
     * @throws std::runtime_error If the operation is registered but not supported by the provider.
     * @throws std::invalid_argument If `indices` is present and the operation does not support indexed writes.
     * @throws std::bad_function_call If the registered provider callback is empty.
     *
     * @note Other exceptions raised by the provider callback propagate to the caller.
     */
    void setArrayData(const std::string& operationName,
                      const array::Array& data,
                      const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Invokes a registered multi-buffer write operation.
     *
     * @param[in] operationName Write-operation name.
     * @param[in] data Source arrays.
     * @param[in] indices Operation-defined selector, or `std::nullopt` to select all entities, which is honored only
     *                    when the registered operation defines that convention.
     * @throws std::out_of_range If no matching multi-buffer write operation is registered.
     * @throws std::runtime_error If the operation is registered but not supported by the provider.
     * @throws std::invalid_argument If `indices` is present and the operation does not support indexed writes.
     * @throws std::bad_function_call If the registered provider callback is empty.
     *
     * @note Other exceptions raised by the provider callback propagate to the caller.
     */
    void setArrayDataMulti(const std::string& operationName,
                           const std::vector<array::Array>& data,
                           const std::optional<array::Array>& indices = std::nullopt) const;

    /**
     * @brief Gets the prim-path patterns supplied at construction.
     *
     * @return A reference to the stored path patterns. The reference remains valid until a derived class modifies the
     *         stored vector or the view is destroyed.
     */
    const std::vector<std::string>& getPrimPathPatterns() const noexcept
    {
        return m_primPathPatterns;
    }

    /**
     * @brief Gets the prim paths resolved by the engine.
     *
     * The base implementation returns the construction paths without expanding patterns.
     *
     * @return Resolved prim paths, or the original construction paths when the provider does not override this method.
     */
    virtual std::vector<std::string> getResolvedPrimPaths() const
    {
        return m_primPathPatterns;
    }

    /**
     * @brief Gets the USD stage-cache identifier associated with the view.
     *
     * @return The USD stage-cache identifier, or `0` if none was set.
     */
    virtual int64_t getUsdStageId() const noexcept
    {
        return 0;
    }

    /**
     * @brief Gets the entity count resolved by the engine.
     *
     * @return The resolved entity count.
     */
    int64_t getEntityCount() const noexcept
    {
        return m_entityCount;
    }

    /**
     * @brief Sets the number of entities resolved by the engine.
     *
     * @param[in] count Resolved entity count.
     * @pre `count` is nonnegative.
     */
    void setEntityCount(int64_t count) noexcept
    {
        m_entityCount = count;
    }

    /**
     * @brief Gets the device on which this view's tensors reside.
     *
     * Consumers read this on every access rather than caching it, because an engine may select its device
     * after the view is created. An engine that resolves its device later overrides this to report the
     * current one; the base implementation returns the stored value.
     *
     * @return The device, which is @c array::Device::Cpu() for host memory.
     */
    virtual array::Device getDevice() const noexcept
    {
        const int ordinal = getDeviceOrdinal();
        return ordinal < 0 ? array::Device::Cpu() : array::Device::Cuda(ordinal);
    }

    /**
     * @brief Sets the device on which this view's tensors reside.
     *
     * @param[in] device Device holding this view's tensors, or @c array::Device::Cpu() for host memory.
     */
    void setDevice(const array::Device& device) noexcept
    {
        m_deviceOrdinal = device.isCpu() ? -1 : device.ordinal();
    }

    /** @brief Return the currently selected device ordinal, or -1 for CPU. */
    virtual int getDeviceOrdinal() const noexcept
    {
        return m_deviceOrdinal;
    }
    /** @brief Select the device ordinal, or -1 for CPU. */
    void setDeviceOrdinal(int ordinal) noexcept
    {
        m_deviceOrdinal = ordinal;
    }

protected:
    /**
     * @brief Rewrites the declared output shapes of a registered multi-buffer read operation.
     *
     * Engine views whose extent is chosen by the caller keep the declared shapes tracking the buffers they read
     * into. Only the shape hints change; the registered callback is untouched, so this may be called from within
     * that callback.
     *
     * @param[in] operationName Operation name to update.
     * @param[in] kind Operation kind, expected to be @c ImplementationKind::eGet.
     * @param[in] shapeHints One shape per registered output, in registration order.
     * @return @c true if the operation was found and updated; @c false if it is not a registered multi-buffer read.
     * @throws std::invalid_argument If @p shapeHints does not have one entry per registered output.
     */
    bool _setImplementationOutputShapeHints(const std::string& operationName,
                                            ImplementationKind kind,
                                            const std::vector<std::vector<int64_t>>& shapeHints);

    /**
     * @brief Rewrites the declared shape of a registered single-buffer operation.
     *
     * The single-buffer counterpart of @ref _setImplementationOutputShapeHints, with the same guarantee: only the
     * shape hint changes, so it is safe to call from within the registered callback.
     *
     * @param[in] operationName Operation name to update.
     * @param[in] kind Operation kind to update.
     * @param[in] shapeHint The replacement shape.
     * @return @c true if the operation was found and updated; @c false if it is not registered for @p kind.
     */
    bool _setImplementationShapeHint(const std::string& operationName,
                                     ImplementationKind kind,
                                     const std::vector<int64_t>& shapeHint);

    /** @brief Prim-path patterns supplied at construction. */
    std::vector<std::string> m_primPathPatterns;
    /** @brief Number of entities resolved by the engine. */
    int64_t m_entityCount{ 0 };
    /** @brief Device holding this view's tensors; the CPU until an engine selects another. */
    int m_deviceOrdinal{ -1 };

private:
    struct LegacyImplementation;
    std::unique_ptr<LegacyImplementation> m_legacy;
    struct Implementation;
    std::unique_ptr<Implementation> m_implementation;
};

} // namespace tensors
} // namespace physics
} // namespace isaacsim
