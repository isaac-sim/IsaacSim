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

#include <dlpack/dlpack.h>
#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>
#include <isaacsim/physics/registration/tensors/TensorSpecification.hpp>
#include <ovphysx/ovphysx_types.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace isaacsim
{
namespace physics_engines
{
namespace ovphysx
{

// Create a TensorSpecification from an ovphysx binding specification (shape, dtype).
isaacsim::physics::tensors::TensorSpecification getTensorSpecificationFromOvphysx(ovphysx_handle_t handle,
                                                                                  ovphysx_tensor_binding_handle_t binding,
                                                                                  bool supportsIndexedRead,
                                                                                  bool supportsIndexedWrite);

// Resolve the device an ovphysx view's tensors live on. A view keeps whatever ordinal a caller assigned to
// it; otherwise it reports the backend-wide declaration, read on each access because ovphysx selects its
// device when the scene is created, which may be after the view was built. `-1` means "unassigned" as well
// as "host", so declaring the backend device is how a caller asks every view to follow it.
int resolveOvPhysxDeviceOrdinal(const isaacsim::physics::tensors::EntityView& view);

// Base class for OvPhysX tensor entity views. Holds the mapping from implementation name
// to tensor binding handle and registers C++ lambdas as implementations.
class OvPhysxEntityViewBase : public isaacsim::physics::tensors::EntityView
{
public:
    OvPhysxEntityViewBase(ovphysx_handle_t handle, const std::vector<std::string>& paths);
    ~OvPhysxEntityViewBase() override;

    ovphysx_handle_t getHandle() const noexcept
    {
        return m_handle;
    }

protected:
    // Called from subclass constructor to register all implementations. When `resolvedPaths`
    // has a value the bindings are created from those explicit prim paths
    // (multi-pattern views; an empty list means all patterns matched nothing -> a
    // 0-entity view); otherwise (nullopt) from the single `pattern`.
    void _initializeBindings(const std::vector<std::pair<std::string, ovphysx_tensor_type_t>>& implementationMap,
                             const std::string& pattern,
                             const std::optional<std::vector<std::string>>& resolvedPaths = std::nullopt);

    // Query entity count from the first successful binding's `shape[0]` and call `setEntityCount()`.
    // Must be called after _initializeBindings() so m_bindings is populated.
    void _setEntityCountFromBindings();

    // Return the resolved prim paths from the first binding, or empty if unavailable.
    std::vector<std::string> getResolvedPrimPaths() const override;

    // See resolveOvPhysxDeviceOrdinal.
    int getDeviceOrdinal() const noexcept override;

    ovphysx_handle_t m_handle;
    std::unordered_map<std::string, ovphysx_tensor_binding_handle_t> m_bindings;
};

// Articulation entity view: DOF positions/velocities, root transforms, link poses.
class OvPhysxArticulationEntityView : public OvPhysxEntityViewBase
{
public:
    OvPhysxArticulationEntityView(ovphysx_handle_t handle, const std::string& pattern);
    // Pattern-list form: each element may be a glob or an explicit prim path;
    // resolved and concatenated in list order (first-wins de-duplication).
    OvPhysxArticulationEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns);

private:
    OvPhysxArticulationEntityView(ovphysx_handle_t handle,
                                  std::string pattern,
                                  std::optional<std::vector<std::string>> resolvedPaths);
    std::string m_pattern;
    std::optional<std::vector<std::string>> m_resolvedPaths; // nullopt for single-pattern views
};

// Rigid body entity view: transforms, velocities, masses, forces, etc.
class OvPhysxRigidBodyEntityView : public OvPhysxEntityViewBase
{
public:
    OvPhysxRigidBodyEntityView(ovphysx_handle_t handle, const std::string& pattern);
    // Pattern-list form: each element may be a glob or an explicit prim path;
    // resolved and concatenated in list order (first-wins de-duplication).
    OvPhysxRigidBodyEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns);

private:
    OvPhysxRigidBodyEntityView(ovphysx_handle_t handle,
                               std::string pattern,
                               std::optional<std::vector<std::string>> resolvedPaths);
    std::string m_pattern;
    std::optional<std::vector<std::string>> m_resolvedPaths; // nullopt for single-pattern views
};

// Rigid contact entity view: net forces, force matrix, detailed contact/friction data.
// Uses the contact binding C API (ovphysx_create_contact_binding / read functions).
class OvPhysxRigidContactEntityView : public isaacsim::physics::tensors::EntityView
{
public:
    OvPhysxRigidContactEntityView(ovphysx_handle_t handle,
                                  const std::string& sensorPattern,
                                  const std::vector<std::string>& filterPatterns,
                                  int maximumContactDataCount);
    ~OvPhysxRigidContactEntityView() override;

    ovphysx_handle_t getHandle() const noexcept
    {
        return m_handle;
    }
    ovphysx_contact_binding_handle_t getContactBinding() const noexcept
    {
        return m_contactBinding;
    }

    // See resolveOvPhysxDeviceOrdinal.
    int getDeviceOrdinal() const noexcept override;
    int32_t getSensorCount() const noexcept
    {
        return m_sensorCount;
    }

private:
    // Host fallback buffers for the multi-get reads, used when the caller supplies no `out`. Only the
    // per-contact-record buffers scale with capacity; the count and start-index buffers are sized by
    // sensor and filter count and never move.
    struct ContactBuffers
    {
        std::vector<float> contactForce, contactPoint, contactNormal, contactSeparation;
        std::vector<int32_t> contactCount, contactStartIndex;
        std::vector<float> frictionForce, frictionPoint;
        std::vector<int32_t> frictionCount, frictionStartIndex;
        std::vector<float> rawForce, rawPoint, rawNormal, rawSeparation;
        std::vector<int32_t> rawCount, rawStartIndex;
        std::vector<int64_t> rawActorIdentifier;
    };

    // Adopt a caller-requested contact-record capacity: recreate the binding, resize the host buffers and
    // rewrite the declared output shapes. A no-op when the capacity already matches. Capacity is one
    // property of the view, so every read implementation observes the change and none is left on a stale binding.
    void _ensureContactCapacity(int32_t requestedCapacity);

    ovphysx_handle_t m_handle;
    ovphysx_contact_binding_handle_t m_contactBinding{ 0 };
    int32_t m_sensorCount{ 0 };
    int32_t m_filterCount{ 0 };
    int32_t m_maximumContactDataCount{ 0 };
    // Retained so the binding can be recreated at a different capacity.
    std::vector<std::string> m_sensorStrings;
    std::vector<std::string> m_filterStrings;
    uint32_t m_filtersPerSensor{ 0 };
    std::shared_ptr<ContactBuffers> m_buffers;
};

// Volume deformable body entity view: simulation node positions/velocities, kinematic targets,
// rest positions, simulation and collision element indices.
class OvPhysxVolumeDeformableBodyEntityView : public OvPhysxEntityViewBase
{
public:
    OvPhysxVolumeDeformableBodyEntityView(ovphysx_handle_t handle, const std::string& pattern);
    OvPhysxVolumeDeformableBodyEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns);

private:
    OvPhysxVolumeDeformableBodyEntityView(ovphysx_handle_t handle,
                                          std::string pattern,
                                          std::optional<std::vector<std::string>> resolvedPaths);
};

// Surface deformable body entity view: simulation node positions/velocities, kinematic targets,
// rest positions, simulation element indices.
class OvPhysxSurfaceDeformableBodyEntityView : public OvPhysxEntityViewBase
{
public:
    OvPhysxSurfaceDeformableBodyEntityView(ovphysx_handle_t handle, const std::string& pattern);
    OvPhysxSurfaceDeformableBodyEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns);

private:
    OvPhysxSurfaceDeformableBodyEntityView(ovphysx_handle_t handle,
                                           std::string pattern,
                                           std::optional<std::vector<std::string>> resolvedPaths);
};

// Deformable material entity view: Young's modulus, Poisson's ratio, dynamic friction.
// CPU-resident material tensors; prim path metadata not available.
class OvPhysxDeformableMaterialEntityView : public OvPhysxEntityViewBase
{
public:
    OvPhysxDeformableMaterialEntityView(ovphysx_handle_t handle, const std::string& pattern);
    OvPhysxDeformableMaterialEntityView(ovphysx_handle_t handle, const std::vector<std::string>& patterns);

private:
    OvPhysxDeformableMaterialEntityView(ovphysx_handle_t handle,
                                        std::string pattern,
                                        std::optional<std::vector<std::string>> resolvedPaths);
};

// SDF shape entity view: evaluates the signed-distance function of shapes matching pattern.
// Uses the dedicated ovphysx_create_sdf_view / ovphysx_evaluate_sdf API rather than the
// standard tensor binding path.
//
// The "distances-and-gradients" GET implementation takes query points [N, Q, 3] via the indices
// TensorDescription and returns [N, Q, 4] (distance, grad.x, grad.y, grad.z).
class OvPhysxSdfShapeEntityView : public isaacsim::physics::tensors::EntityView
{
public:
    OvPhysxSdfShapeEntityView(ovphysx_handle_t handle, const std::string& pattern, int maximumQueryPointCount);
    ~OvPhysxSdfShapeEntityView() override;

    ovphysx_handle_t getHandle() const noexcept
    {
        return m_handle;
    }

    // See resolveOvPhysxDeviceOrdinal.
    int getDeviceOrdinal() const noexcept override;

private:
    // Adopt a caller-requested query-point extent by recreating the SDF view, which takes the extent at
    // creation. A no-op when it already matches. Unlike contacts, nothing here is discovered from a failed
    // read: the maximum query-point count is how many points the caller intends to submit, and it is present in the
    // query they submit.
    void _ensureQueryPointCapacity(int32_t requestedQueryPointCount);

    ovphysx_handle_t m_handle;
    ovphysx_sdf_view_handle_t m_sdfHandle{ 0 };
    int32_t m_shapeCount{ 0 };
    int32_t m_maximumQueryPointCount{ 0 };
    // Retained so the view can be recreated at a different query-point extent.
    std::string m_pattern;
};

// Unsupported stub view: all implementations are registered with supports=false.
std::shared_ptr<isaacsim::physics::tensors::EntityView> makeUnsupportedView(const std::string& pattern,
                                                                            const std::string& category);

} // namespace ovphysx
} // namespace physics_engines
} // namespace isaacsim
