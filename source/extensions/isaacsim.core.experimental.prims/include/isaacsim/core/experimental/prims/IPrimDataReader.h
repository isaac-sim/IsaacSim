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

#include <carb/Interface.h>

#include <cstddef>
#include <cstdint>
#include <functional>

namespace isaacsim
{
namespace core
{
namespace experimental
{
namespace prims
{

/**
 * @struct Poses
 * @brief Combined positions and orientations for all prims in a view.
 * @details Positions are tightly packed as (x, y, z) triplets; orientations as
 * (qw, qx, qy, qz) quaternions. Both pointers are nullptr and counts are zero
 * when the underlying fields are unavailable.
 */
struct Poses
{
    const float* positions; ///< float[posCount] packed as (x,y,z) per prim
    int posCount; ///< total number of position floats (numPrims * 3)
    const float* orientations; ///< float[oriCount] packed as (qw,qx,qy,qz) per prim
    int oriCount; ///< total number of orientation floats (numPrims * 4)
};

/**
 * @struct IXformDataView
 * @brief Read-only view for XformPrim data (positions, orientations, scales).
 * @details Engine-agnostic transform data read via IFabricHierarchy.
 * All getters use lazy fetch with step-based staleness detection via
 * ISimulationManager::getNumPhysicsSteps(). Safe to call from multiple
 * onPhysicsStep callbacks.
 */
struct IXformDataView
{
    virtual ~IXformDataView() = default;

    /// @name Transform getters (native device pointers)
    /// @{

    /// Get world positions (float[3] per prim, device memory).
    virtual const float* getWorldPositions(int* outCount) = 0;
    /// Get world orientations (quaternion float[4] per prim, device memory).
    virtual const float* getWorldOrientations(int* outCount) = 0;
    /// Get local translations (float[3] per prim, device memory).
    virtual const float* getLocalTranslations(int* outCount) = 0;
    /// Get local orientations (quaternion float[4] per prim, device memory).
    virtual const float* getLocalOrientations(int* outCount) = 0;
    /// Get local scales (float[3] per prim, device memory).
    virtual const float* getLocalScales(int* outCount) = 0;
    /// @}

    /// @name Host-memory transform getters (always CPU pointers, copied from GPU if needed)
    /// @{

    /// Get world positions (float[3] per prim, host memory).
    virtual const float* getWorldPositionsHost(int* outCount) = 0;
    /// Get world orientations (quaternion float[4] per prim, host memory).
    virtual const float* getWorldOrientationsHost(int* outCount) = 0;
    /// Get local translations (float[3] per prim, host memory).
    virtual const float* getLocalTranslationsHost(int* outCount) = 0;
    /// Get local orientations (quaternion float[4] per prim, host memory).
    virtual const float* getLocalOrientationsHost(int* outCount) = 0;
    /// Get local scales (float[3] per prim, host memory).
    virtual const float* getLocalScalesHost(int* outCount) = 0;
    /// @}

    /// @name Combined pose getters
    /// @{

    /// Get combined world positions and orientations (device memory).
    /// Prefer this over calling getWorldPositions + getWorldOrientations separately
    /// as it invokes the shared fill callback only once instead of twice.
    virtual Poses getWorldPoses() = 0;
    /// Get combined world positions and orientations (host memory).
    /// Prefer this over calling getWorldPositionsHost + getWorldOrientationsHost separately
    /// as it invokes the shared fill callback only once instead of twice.
    virtual Poses getWorldPosesHost() = 0;
    /// @}

    /// Batch pre-fetch all fields for this view
    virtual bool update() = 0;

    /// @name Buffer and callback management (used by Python during setup)
    /// @{

    /// Allocate a named float buffer of the given element count.
    virtual bool allocateBufferFloat(const char* fieldName, size_t count) = 0;
    /// Allocate a named uint8 buffer of the given element count.
    virtual bool allocateBufferUint8(const char* fieldName, size_t count) = 0;
    /// Get the raw pointer to a named buffer.
    virtual uintptr_t getBufferPtr(const char* fieldName) = 0;
    /// Get the byte size of a named buffer.
    virtual size_t getBufferSize(const char* fieldName) = 0;
    /// Get the CUDA device ordinal of the data buffers (-1 for CPU).
    virtual int getBufferDevice() = 0;
    /// Register a callback invoked when the named field is updated.
    virtual void registerFieldCallback(const char* fieldName, std::function<void()> callback) = 0;
    /// @}

    /// Resolve frame name for any prim in the stage: checks isaac:nameOverride, falls back to prim name.
    /// outName: caller buffer of maxLen bytes. Returns false if prim not found or stage unavailable.
    virtual bool getPrimFrameName(const char* primPath, char* outName, size_t maxLen) = 0;

    /// World transform of any prim in the stage via Fabric (no physics required).
    /// outPos3: float[3] (x,y,z). outOri4: float[4] (qw,qx,qy,qz).
    /// Returns false if prim not found or stage unavailable.
    virtual bool getPrimWorldTransform(const char* primPath, float* outPos3, float* outOri4) = 0;
};

/**
 * @struct IRigidBodyDataView
 * @brief Read-only view for RigidPrim data. Extends IXformDataView with physics state.
 */
struct IRigidBodyDataView : public IXformDataView
{
    /// Get linear velocities (float[3] per body, device memory).
    virtual const float* getLinearVelocities(int* outCount) = 0;
    /// Get angular velocities (float[3] per body, device memory).
    virtual const float* getAngularVelocities(int* outCount) = 0;

    /// Get linear velocities (float[3] per body, host memory).
    virtual const float* getLinearVelocitiesHost(int* outCount) = 0;
    /// Get angular velocities (float[3] per body, host memory).
    virtual const float* getAngularVelocitiesHost(int* outCount) = 0;
};

/**
 * @struct LinkInfo
 * @brief Per-link descriptor returned by IArticulationDataView::getArticulationLinks().
 * @details Pointers are owned by the view; valid until the next call to getArticulationLinks() or view removal.
 */
struct LinkInfo
{
    const char* path; ///< USD path of this link
    const char* parentPath; ///< USD path of parent link, or "" for root (world parent)
};

/**
 * @struct IArticulationDataView
 * @brief Read-only view for Articulation data. Extends IXformDataView with DOF/link/dynamics.
 */
struct IArticulationDataView : public IXformDataView
{
    /// Get DOF positions (device memory).
    virtual const float* getDofPositions(int* outCount) = 0;
    /// Get DOF velocities (device memory).
    virtual const float* getDofVelocities(int* outCount) = 0;
    /// Get DOF efforts / forces (device memory).
    virtual const float* getDofEfforts(int* outCount) = 0;
    /// Get root link transforms (float[7] per articulation, device memory).
    virtual const float* getRootTransforms(int* outCount) = 0;
    /// Get root link velocities (float[6] per articulation, device memory).
    virtual const float* getRootVelocities(int* outCount) = 0;
    /// Get DOF types (0 = rotation, 1 = translation, device memory).
    virtual const uint8_t* getDofTypes(int* outCount) = 0;

    /// Get DOF positions (host memory).
    virtual const float* getDofPositionsHost(int* outCount) = 0;
    /// Get DOF velocities (host memory).
    virtual const float* getDofVelocitiesHost(int* outCount) = 0;
    /// Get DOF efforts / forces (host memory).
    virtual const float* getDofEffortsHost(int* outCount) = 0;
    /// Get root link transforms (float[7] per articulation, host memory).
    virtual const float* getRootTransformsHost(int* outCount) = 0;
    /// Get root link velocities (float[6] per articulation, host memory).
    virtual const float* getRootVelocitiesHost(int* outCount) = 0;
    /// Get DOF types (0 = rotation, 1 = translation, host memory).
    virtual const uint8_t* getDofTypesHost(int* outCount) = 0;

    /**
     * @brief Resolve the DOF index for a joint given its USD prim path.
     * @param dofPrimPath USD path to the joint prim.
     * @return DOF index (>= 0), or -1 if the joint is not found in this view.
     */
    virtual int getDofIndex(const char* dofPrimPath) = 0;

    /**
     * @brief Get DOF names in articulation order (index 0 .. N-1).
     * @param outCount Receives the number of DOFs.
     * @return Pointer to array of C-strings, or nullptr if none. Valid until view is removed or reader re-initialized.
     */
    virtual const char* const* getDofNames(int* outCount) = 0;

    /// Enumerate UsdPhysicsRigidBodyAPI descendants of rootPath (must have ArticulationRootAPI).
    /// outLinks owned by the view; valid until next call or view removal.
    /// Returns false if rootPath is not an articulation or stage unavailable.
    virtual bool getArticulationLinks(const char* rootPath, const LinkInfo** outLinks, size_t* outCount) = 0;
};

/**
 * @struct ContactPointData
 * @brief Single contact point within a contact event.
 */
struct ContactPointData
{
    float positionX{ 0.0f }; ///< X component of the contact point position, in meters.
    float positionY{ 0.0f }; ///< Y component of the contact point position, in meters.
    float positionZ{ 0.0f }; ///< Z component of the contact point position, in meters.
    float normalX{ 0.0f }; ///< X component of the contact normal direction.
    float normalY{ 0.0f }; ///< Y component of the contact normal direction.
    float normalZ{ 0.0f }; ///< Z component of the contact normal direction.
    float impulseX{ 0.0f }; ///< X component of the contact impulse, in newton-seconds.
    float impulseY{ 0.0f }; ///< Y component of the contact impulse, in newton-seconds.
    float impulseZ{ 0.0f }; ///< Z component of the contact impulse, in newton-seconds.
};

/// Contact event type constants matching PhysX ContactEventType::Enum values.
/// Defined here so consumers without PhysX headers can use them.
static constexpr uint32_t kContactEventFound = 0;
static constexpr uint32_t kContactEventLost = 1;
static constexpr uint32_t kContactEventPersist = 2;

/**
 * @struct ContactEventData
 * @brief One contact event (header) between a pair of bodies.
 * @details Mirrors the PhysX ContactEventHeader structure. LOST events have
 * numContacts=0 and contacts=nullptr. Body identifiers are uint64_t tokens
 * (SdfPath internal representation) produced by sdfPathToToken() (see SdfPathToken.h).
 */
struct ContactEventData
{
    uint64_t body0{ 0 }; ///< SdfPath token of the first body in the contact pair.
    uint64_t body1{ 0 }; ///< SdfPath token of the second body in the contact pair.
    uint32_t eventType{ 0 }; ///< kContactEventFound / kContactEventLost / kContactEventPersist
    const ContactPointData* contacts{ nullptr }; ///< points to contiguous array, owned by reader
    uint32_t numContacts{ 0 }; ///< Number of contact points referenced by contacts.
};

/**
 * @struct ContactReportData
 * @brief Full contact report returned by IPrimDataReader::getContactReport().
 * @details Pointers are owned by the reader and valid until the next call to
 * getContactReport() or until the reader is shut down / re-initialized.
 */
struct ContactReportData
{
    const ContactEventData* events{ nullptr }; ///< Pointer to contiguous array of contact events, owned by the reader.
    uint32_t numEvents{ 0 }; ///< Number of contact events referenced by events.
    float simTime{ 0.0f }; ///< Simulation time at which the report was generated, in seconds.
    float dt{ 0.0f }; ///< Simulation time step covered by the report, in seconds.
};

/**
 * @struct IPrimDataReader
 * @brief Factory interface for creating typed read-only prim data views.
 * @details Carbonite plugin interface that creates typed views for XformPrim,
 * RigidPrim, and Articulation data. Views provide compile-time safety:
 * only getters valid for the view type are available.
 *
 * Supports both PhysX (direct C++ TensorApi) and Newton (Python callback)
 * backends transparently.
 *
 * For multi-plugin usage, prefer acquiring and using IPrimDataReaderManager
 * so lifecycle initialization is centralized across sensors/nodes. In new
 * call sites, initialize() should be treated as manager-owned API surface.
 *
 * **Threading:** All methods must be called from the main thread (the
 * same thread that drives the simulation). Getter callbacks registered
 * on views may be invoked from physics step callbacks on that thread.
 *
 * **View lifetime:** Pointers returned by create*View() are owned by
 * the plugin. They remain valid until removeView() is called for the
 * same viewId, or until a generation change (detectable via
 * getGeneration()). Consumers should snapshot the generation when
 * creating a view and recheck before dereferencing cached pointers.
 */
struct IPrimDataReader
{
    CARB_PLUGIN_INTERFACE("isaacsim::core::experimental::prims::IPrimDataReader", 2, 2);

    /**
     * @brief Initialize the reader with the current USD stage and simulation device.
     * @details Each call destroys all existing views, recreates the internal
     * PhysX simulation view, and increments the generation counter. This is
     * required because PhysX invalidates simulation views on timeline
     * stop/play even when the stageId is unchanged. Consumers must
     * recreate their views after each initialize() call. Prefer calling
     * IPrimDataReaderManager::ensureInitialized() instead of calling this
     * directly from sensor/node plugins.
     * @param stageId Fabric stage ID (from UsdUtilsStageCache).
     * @param deviceOrdinal CUDA device ordinal (-1 for CPU, >=0 for GPU).
     */
    virtual void initialize(long stageId, int deviceOrdinal) = 0;

    /**
     * @brief Shut down the reader, destroying all views and freeing all buffers.
     */
    virtual void shutdown() = 0;

    /**
     * @brief Create a typed XformPrim data view.
     * @param viewId Unique string identifier for this view.
     * @param paths Array of prim path strings (may include regex patterns).
     * @param numPaths Number of paths in the array.
     * @param engineType Physics engine backend: "physx" or "newton".
     * @return Pointer to view, owned by the plugin. Valid until removeView() or shutdown().
     */
    virtual IXformDataView* createXformView(const char* viewId,
                                            const char** paths,
                                            size_t numPaths,
                                            const char* engineType) = 0;

    /**
     * @brief Create a typed RigidPrim data view (includes XformPrim getters).
     * @param viewId Unique string identifier for this view.
     * @param paths Array of prim path strings.
     * @param numPaths Number of paths in the array.
     * @param engineType Physics engine backend: "physx" or "newton".
     * @return Pointer to view, owned by the plugin. Valid until removeView() or shutdown().
     */
    virtual IRigidBodyDataView* createRigidBodyView(const char* viewId,
                                                    const char** paths,
                                                    size_t numPaths,
                                                    const char* engineType) = 0;

    /**
     * @brief Create a typed Articulation data view (includes XformPrim getters).
     * @param viewId Unique string identifier for this view.
     * @param paths Array of prim path strings.
     * @param numPaths Number of paths in the array.
     * @param engineType Physics engine backend: "physx" or "newton".
     * @return Pointer to view, owned by the plugin. Valid until removeView() or shutdown().
     */
    virtual IArticulationDataView* createArticulationView(const char* viewId,
                                                          const char** paths,
                                                          size_t numPaths,
                                                          const char* engineType) = 0;

    /**
     * @brief Destroy a view and free its buffers.
     * @param viewId Identifier of the view to remove.
     */
    virtual void removeView(const char* viewId) = 0;

    /**
     * @brief Set DOF names and types for an articulation view (used by Newton backend from Python).
     * @param viewId Identifier of the articulation view.
     * @param names Array of DOF name C-strings.
     * @param numNames Number of names.
     * @param types Array of DOF types (0 = rotation, 1 = translation).
     * @param numTypes Number of types.
     */
    virtual void setArticulationDofMetadata(
        const char* viewId, const char** names, size_t numNames, const uint8_t* types, size_t numTypes) = 0;

    /**
     * @brief Monotonically increasing counter incremented on each initialize() call.
     * @details Consumers can snapshot this value when creating views and compare
     * later to detect whether the underlying simulation view has been recreated,
     * which invalidates all previously returned data view pointers.
     */
    virtual uint64_t getGeneration() const = 0;

    /**
     * @brief Get the Fabric stage ID passed to the last initialize() call.
     * @return Stage ID, or 0 if not yet initialized.
     */
    virtual long getStageId() const = 0;

    /**
     * @brief Get the effective CUDA device ordinal.
     * @details After initialize(), this reflects the device ordinal reported by
     * the underlying simulation view (which may differ from the value originally
     * passed to initialize()).
     * @return CUDA device ordinal (>=0 for GPU, -1 for CPU).
     */
    virtual int getDeviceOrdinal() const = 0;

    /**
     * @brief Enable contact reporting for a rigid body on the simulation side.
     * @details Applies PhysxContactReportAPI (threshold=0, sleepThreshold=0) so that
     * subsequent getContactReport() calls return data for this body. Idempotent.
     * For local PhysX: modifies the USD stage directly.
     * For remote: forwards to the World Simulator via gRPC.
     * For Newton: no-op (returns true).
     * @param bodyPath USD path to the rigid body prim.
     * @return true on success, false on error.
     */
    virtual bool enableContactReporting(const char* bodyPath) = 0;

    /**
     * @brief Get the full contact report filtered to the specified body paths.
     * @details Returns a two-level structure: events (one per body pair, including
     * LOST events with zero contact points) containing contact points. Pointers in
     * the returned ContactReportData are owned by the reader and valid until the
     * next call to getContactReport() or reader re-initialization.
     * For local PhysX: calls IPhysxSimulation::getFullContactReport().
     * For remote: fetches via gRPC from the World Simulator.
     * For Newton: returns empty report (contact sensor not supported via this path).
     * @param bodyPaths Array of rigid body prim path strings to filter for.
     * @param numPaths Number of paths in the array.
     * @param outReport Receives the contact report data.
     * @return true if the query succeeded (report may still have zero events), false on error.
     */
    virtual bool getContactReport(const char** bodyPaths, size_t numPaths, ContactReportData* outReport) = 0;
};

} // namespace prims
} // namespace experimental
} // namespace core
} // namespace isaacsim
