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

#pragma once

#include <omni/physics/tensors/ISimulationView.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pybind11/pybind11.h>

#include <string>
#include <vector>

namespace py = pybind11;

namespace isaacsim
{
namespace physics
{
namespace newton
{
namespace tensors
{

using omni::physics::tensors::IArticulationView;
using omni::physics::tensors::IDeformableBodyView;
using omni::physics::tensors::IDeformableMaterialView;
using omni::physics::tensors::IRigidBodyView;
using omni::physics::tensors::IRigidContactView;
using omni::physics::tensors::ISdfShapeView;
using omni::physics::tensors::ISimulationView;
using omni::physics::tensors::ObjectType;

/// Initialization data extracted from the Python Newton stage.
/// Produced by BaseSimulationView::initNewton() and consumed by Cpu/GpuSimulationView constructors.
struct SimViewInit
{
    py::object usdStage; ///< USD stage handle.
    py::object newtonStage; ///< Newton stage object.
    py::object model; ///< Newton model object.
    int simDeviceOrdinal = -1; ///< -1 for CPU, >= 0 for GPU CUDA device ordinal.
    bool valid = false; ///< Whether the initialization succeeded.
};

/// Base simulation view implementing ISimulationView for the Newton backend.
///
/// Holds the Python Newton stage and model references, implements shared logic for step,
/// gravity, object-type queries, and USD pattern matching. Factory methods for child views
/// (articulation, rigid body, rigid contact) resolve USD patterns and delegate to pure
/// virtual make*View() methods overridden by Cpu/GpuSimulationView.
class BaseSimulationView : public ISimulationView
{
public:
    /// Acquires the GIL, imports the Newton Python module, obtains the NewtonStage and model,
    /// and reads the simulation device ordinal.
    static SimViewInit initNewton(long stageId);

    ~BaseSimulationView() override;

    /**
     * @brief Returns whether the view is currently valid.
     * @return True on success; false otherwise.
     */
    bool getValid() const override;
    /**
     * @brief Marks the view as invalid.
     */
    void invalidate() override;
    /**
     * @brief Checks whether the view and its backing data are still valid.
     * @return True on success; false otherwise.
     */
    bool check() const override;
    /**
     * @brief Sets the subspace roots.
     * @param[in] pattern USD path pattern used to match prims.
     * @return True on success; false otherwise.
     */
    bool setSubspaceRoots(const char* pattern) override;

    /**
     * @brief Advances the simulation by the given time step.
     * @param[in] dt Time step, in seconds.
     */
    void step(float dt) override;
    /**
     * @brief Sets the gravity.
     * @param[in] gravity Gravity vector to apply, in meters per second squared.
     * @return True on success; false otherwise.
     */
    bool setGravity(const carb::Float3& gravity) override;
    /**
     * @brief Gets the gravity.
     * @param[out] gravity Receives the current gravity vector, in meters per second squared.
     * @return True on success; false otherwise.
     */
    bool getGravity(carb::Float3& gravity) override;

    /**
     * @brief Gets the object type.
     * @param[in] path USD prim path to query.
     * @return The requested value.
     */
    ObjectType getObjectType(const char* path) override;
    /**
     * @brief Clears the accumulated forces on all bodies.
     */
    void clearForces() override;
    /**
     * @brief Flushes pending changes to the simulation backend.
     * @return True on success; false otherwise.
     */
    bool flush() override;
    /**
     * @brief Updates articulation link transforms from the current joint state.
     */
    void updateArticulationsKinematic() override;
    /**
     * @brief Initializes the kinematic bodies tracked by the view.
     */
    void InitializeKinematicBodies() override;
    /**
     * @brief Sets whether to enable the GPU usage warnings.
     * @param[in] enable If true, enables the behavior; otherwise disables it.
     */
    void enableGpuUsageWarnings(bool enable) override;
    /**
     * @brief Releases the view and frees its associated resources.
     * @param[in] recursive If true, also releases child views recursively.
     */
    void release(bool recursive) override;

    // Unsupported view stubs
    /**
     * @brief Creates the SDF shape view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @param[in] numSamplePoints Number of sample points to use.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    ISdfShapeView* createSdfShapeView(const char* pattern, uint32_t numSamplePoints) override;
    /**
     * @brief Creates the volume deformable body view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableBodyView* createVolumeDeformableBodyView(const char* pattern) override;
    /**
     * @brief Creates the volume deformable body view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableBodyView* createVolumeDeformableBodyView(const std::vector<std::string>& patterns) override;
    /**
     * @brief Creates the surface deformable body view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableBodyView* createSurfaceDeformableBodyView(const char* pattern) override;
    /**
     * @brief Creates the surface deformable body view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableBodyView* createSurfaceDeformableBodyView(const std::vector<std::string>& patterns) override;
    /**
     * @brief Creates the deformable material view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableMaterialView* createDeformableMaterialView(const char* pattern) override;
    /**
     * @brief Creates the deformable material view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IDeformableMaterialView* createDeformableMaterialView(const std::vector<std::string>& patterns) override;

    // Factory methods — shared logic in Base, delegates to newXxxView()
    /**
     * @brief Creates the articulation view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IArticulationView* createArticulationView(const char* pattern) override;
    /**
     * @brief Creates the articulation view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IArticulationView* createArticulationView(const std::vector<std::string>& patterns) override;

    /**
     * @brief Creates the rigid body view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidBodyView* createRigidBodyView(const char* pattern) override;
    /**
     * @brief Creates the rigid body view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidBodyView* createRigidBodyView(const std::vector<std::string>& patterns) override;

    /**
     * @brief Creates the rigid contact view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @param[in] filterPatterns USD path patterns used to match filter prims.
     * @param[in] numFilterPatterns Number of filter patterns.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidContactView* createRigidContactView(const char* pattern,
                                              const char** filterPatterns,
                                              uint32_t numFilterPatterns,
                                              uint32_t maxContactDataCount) override;
    /**
     * @brief Creates the rigid contact view matching the given pattern.
     * @param[in] pattern USD path pattern used to match prims.
     * @param[in] filterPatterns USD path patterns used to match filter prims.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidContactView* createRigidContactView(std::string pattern,
                                              const std::vector<std::string>& filterPatterns,
                                              uint32_t maxContactDataCount) override;
    /**
     * @brief Creates the rigid contact view matching the given patterns.
     * @param[in] patterns List of USD path patterns used to match prims.
     * @param[in] filterPatterns USD path patterns used to match filter prims.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    IRigidContactView* createRigidContactView(const std::vector<std::string>& patterns,
                                              const std::vector<std::vector<std::string>>& filterPatterns,
                                              uint32_t maxContactDataCount) override;

    /**
     * @brief Returns the device ordinal of the simulation.
     * @return The requested value.
     */
    int getSimDeviceOrdinal() const
    {
        return m_simDeviceOrdinal;
    }
    /**
     * @brief Returns the Newton stage object backing the view.
     * @return Reference to the requested object.
     */
    py::object& getNewtonStage()
    {
        return m_newtonStage;
    }
    /**
     * @brief Returns the Newton model object backing the view.
     * @return Reference to the requested object.
     */
    py::object& getModel()
    {
        return m_model;
    }

protected:
    /**
     * @brief Constructs a BaseSimulationView.
     * @param[in] init Initialization parameters for the simulation view.
     */
    explicit BaseSimulationView(SimViewInit&& init);

    /// Device-specific factory methods. Overridden by CpuSimulationView and GpuSimulationView
    /// to instantiate the appropriate Cpu* or Gpu* view class.
    virtual IArticulationView* _makeArticulationView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths) = 0;
    /**
     * @brief Creates the rigid body view.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] paths USD prim paths backing the view.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    virtual IRigidBodyView* _makeRigidBodyView(py::object newtonStage, const std::vector<pxr::SdfPath>& paths) = 0;
    /**
     * @brief Creates the rigid contact view.
     * @param[in] newtonStage Newton stage object backing the view.
     * @param[in] sensorPaths USD prim paths of the contact sensors.
     * @param[in] filterPaths Filter paths.
     * @param[in] maxContactDataCount Maximum number of contact data entries to report.
     * @return Pointer to the newly created view, or nullptr on failure.
     */
    virtual IRigidContactView* _makeRigidContactView(py::object newtonStage,
                                                     const std::vector<std::string>& sensorPaths,
                                                     const std::vector<std::vector<std::string>>& filterPaths,
                                                     uint32_t maxContactDataCount) = 0;

    /**
     * @brief Finds the matching paths.
     * @param[in] pattern USD path pattern used to match prims.
     * @param[in] pathsRet Paths ret.
     */
    void _findMatchingPaths(const std::string& pattern, std::vector<pxr::SdfPath>& pathsRet);
    /**
     * @brief Finds the matching children.
     * @param[in] prim Prim.
     * @param[in] pattern USD path pattern used to match prims.
     * @param[in] matchesRet Matches ret.
     */
    void _findMatchingChildren(const pxr::UsdPrim& prim, const std::string& pattern, std::vector<pxr::UsdPrim>& matchesRet);

    py::object m_usdStage; ///< Value.
    py::object m_newtonStage; ///< Value.
    py::object m_model; ///< Value.
    int m_simDeviceOrdinal; ///< Value.
    bool m_valid; ///< Value.
    carb::Float3 m_gravity = { 0.0f, 0.0f, -9.81f }; ///< Value.
};

} // namespace tensors
} // namespace newton
} // namespace physics
} // namespace isaacsim
