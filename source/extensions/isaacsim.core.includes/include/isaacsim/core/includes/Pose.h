// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Conversions.h"
#include "UsdUtilities.h"

#include <foundation/PxTransform.h>
#include <omni/fabric/usd/PathConversion.h>
#include <physx/include/foundation/PxTransform.h>

#if defined(_WIN32)
#    include <usdrt/scenegraph/usd/rt/xformable.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#    include <usdrt/scenegraph/usd/rt/xformable.h>
#    pragma GCC diagnostic pop
#endif

using namespace isaacsim::core::includes::conversions;
using namespace isaacsim::core::includes;
namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @namespace pose
 * @brief Utilities for handling pose transformations in USD scene graphs.
 * @details
 * Provides functions for computing and manipulating poses (transforms) in USD scenes.
 * Handles both world-space and local-space transformations, supporting:
 * - World transform computation without caching
 * - Relative transform computation between prims
 * - Support for both USD and USDRT stages
 * - Automatic handling of position, orientation, and scale
 */
namespace pose
{

/**
 * @brief Computes the world transform of a prim.
 * @details
 * Calculates the complete world transform by:
 * 1. Checking for world transform attributes (position, orientation, scale)
 * 2. Falling back to local transform computation if world transform unavailable
 * 3. Recursively computing parent transforms when needed
 *
 * The function handles three cases:
 * - Prims with world transform attributes
 * - Prims with local transform only
 * - Regular USD prims without USDRT extensions
 *
 * @param[in] usdStage Reference to the USD stage
 * @param[in] usdrtStage Reference to the USDRT stage
 * @param[in] path Path to the prim whose transform to compute
 * @param[in] timecode Time code for the transform evaluation (default: Default())
 * @param[in] useFabricHierarchy Whether to use IFabricHierarchy (default: false). If enabled, but Fabric Scene Delegate
 * (/app/useFabricSceneDelegate) is not enabled, the call will fall back to non-IFabricHierarchy implementation.
 * Note: Using IFabricHierarchy is experimental and enabling it may result in undefined behavior.
 * @return usdrt::GfMatrix4d The computed world transform matrix
 *
 * @warning May be computationally expensive for deep hierarchies
 */
static usdrt::GfMatrix4d computeWorldXformNoCache(pxr::UsdStageRefPtr usdStage,
                                                  usdrt::UsdStageRefPtr usdrtStage,
                                                  const pxr::SdfPath& path,
                                                  pxr::UsdTimeCode timecode = pxr::UsdTimeCode::Default(),
                                                  bool useFabricHierarchy = false)
{
    // Compute the world transform using IFabricHierarchy
    if (useFabricHierarchy)
    {
        // IFabricHierarchy is only available if Fabric Scene Delegate (/app/useFabricSceneDelegate) is enabled
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
        if (settings->getAsBool("/app/useFabricSceneDelegate"))
        {
            // Get the Fabric Hierarchy instance
            auto iFabricHierarchy = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
            if (!iFabricHierarchy)
            {
                // Fall back to non-IFabricHierarchy call
                return computeWorldXformNoCache(usdStage, usdrtStage, path, timecode, false);
            }
            auto fabricHierarchy =
                iFabricHierarchy->getFabricHierarchy(usdrtStage->GetFabricId(), usdrtStage->GetStageId());
            if (!fabricHierarchy)
            {
                // Fall back to non-IFabricHierarchy call
                return computeWorldXformNoCache(usdStage, usdrtStage, path, timecode, false);
            }

            // Get the world transform of the prim
            // - Always-computed transform
            {
                // Regardless of when `fabricHierarchy->updateWorldXforms();` was last called, `getWorldXform()` will
                // calculate the correct value using the omni:fabric:localMatrix values of the prim and its ancestors.
                return fabricHierarchy->getWorldXform(omni::fabric::asInt(path));
            }
            // - Cached transform (with explicit update call)
            {
                // Update all `omni:fabric:worldMatrix` values for any prim whose local matrix value has changed.
                // If there have been no changes since the last call to `updateWorldXforms()`, the call does nothing.
                fabricHierarchy->updateWorldXforms();
                // Get the computed, cached, and read-only world transform for the prim.
                auto worldMatrixAttr =
                    usdrtStage->GetPrimAtPath(path.GetString()).GetAttribute("omni:fabric:worldMatrix");
                if (!worldMatrixAttr.IsValid())
                {
                    // Fall back to non-IFabricHierarchy call
                    return computeWorldXformNoCache(usdStage, usdrtStage, path, timecode, false);
                }
                usdrt::GfMatrix4d transform(1.0);
                worldMatrixAttr.Get(&transform);
                return transform;
            }
        }
    }

    if (usdrtStage->HasPrimAtPath(path.GetString()))
    {
        usdrt::UsdPrim usdrtPrim = usdrtStage->GetPrimAtPath(path.GetString());
        usdrt::RtXformable usdrtXformable = usdrt::RtXformable(usdrtPrim);

        if (usdrtXformable.HasWorldXform())
        {
            usdrt::GfVec3d worldPos(0);
            usdrt::GfQuatf worldOrient(1);
            usdrt::GfVec3f worldScale(1);
            auto worldPositionAttribute = usdrtXformable.GetWorldPositionAttr();
            if (worldPositionAttribute.HasValue())
            {
                usdrtXformable.GetWorldPositionAttr().Get(&worldPos, usdrt::UsdTimeCode(timecode.GetValue()));
            }
            auto worldOrientationAttribute = usdrtXformable.GetWorldOrientationAttr();
            if (worldOrientationAttribute.HasValue())
            {
                usdrtXformable.GetWorldOrientationAttr().Get(&worldOrient, usdrt::UsdTimeCode(timecode.GetValue()));
            }
            auto worldScaleAttribute = usdrtXformable.GetWorldScaleAttr();
            if (worldScaleAttribute.HasValue())
            {
                usdrtXformable.GetWorldScaleAttr().Get(&worldScale, usdrt::UsdTimeCode(timecode.GetValue()));
            }
            usdrt::GfMatrix4d rot, scale, result{};
            scale.SetScale(usdrt::GfVec3d(worldScale));
            rot.SetRotate(usdrt::GfQuatd(worldOrient));
            result = scale * rot;
            result.SetTranslateOnly(worldPos);
            return result;
        }
        else if (usdrtXformable.HasLocalXform())
        {
            usdrt::GfMatrix4d localMat(1);
            usdrtXformable.GetLocalMatrixAttr().Get(&localMat, usdrt::UsdTimeCode(timecode.GetValue()));
            pxr::UsdPrim parentPrim = usdStage->GetPrimAtPath(path).GetParent();
            usdrt::GfMatrix4d parentXform = usdrt::GfMatrix4d(1.0);
            if (parentPrim)
            {
                parentXform = computeWorldXformNoCache(usdStage, usdrtStage, parentPrim.GetPath(), timecode, false);
            }

            return localMat * parentXform;
        }
    }
    {
        pxr::UsdPrim prim = usdStage->GetPrimAtPath(path);

        usdrt::GfMatrix4d localMat(1.0);
        if (PXR_NS::UsdGeomXformable xformable = PXR_NS::UsdGeomXformable(prim))
        {
            bool dontCare;
            xformable.GetLocalTransformation(reinterpret_cast<PXR_NS::GfMatrix4d*>(&localMat), &dontCare, timecode);
        }
        pxr::UsdPrim parentPrim = prim.GetParent();
        usdrt::GfMatrix4d parentXform = usdrt::GfMatrix4d(1.0);
        if (parentPrim)
        {
            parentXform = computeWorldXformNoCache(usdStage, usdrtStage, parentPrim.GetPath(), timecode, false);
        }
        return localMat * parentXform;
    }
}

/**
 * @brief Computes the relative transform matrix between two prims.
 * @details
 * Calculates the transform that converts coordinates from the source prim's
 * frame to the target prim's frame. The computation follows these steps:
 * 1. Compute world transforms for both source and target prims
 * 2. Invert the target's world transform
 * 3. Multiply to get the relative transform
 *
 * @param[in] usdStage Reference to the USD stage
 * @param[in] usdrtStage Reference to the USDRT stage
 * @param[in] sourcePrim Path to the source prim
 * @param[in] targetPrim Path to the target prim
 * @return usdrt::GfMatrix4d The relative transform matrix (column-major)
 *
 * @note The returned matrix is in column-major format
 * @warning Ensure both prims exist in the stage before calling
 */
inline usdrt::GfMatrix4d getRelativeTransform(pxr::UsdStageRefPtr usdStage,
                                              usdrt::UsdStageRefPtr usdrtStage,
                                              const pxr::SdfPath& sourcePrim,
                                              const pxr::SdfPath& targetPrim)
{
    pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
    // column major transform matrix
    usdrt::GfMatrix4d sourceToWorldColumnMajorTransform = computeWorldXformNoCache(usdStage, usdrtStage, sourcePrim);
    usdrt::GfMatrix4d targetToWorldColumnMajorTransform = computeWorldXformNoCache(usdStage, usdrtStage, targetPrim);

    usdrt::GfMatrix4d worldToTargetColumnMajorTransform = targetToWorldColumnMajorTransform.GetInverse();
    usdrt::GfMatrix4d sourceToTargetColumnMajorTransform =
        worldToTargetColumnMajorTransform * sourceToWorldColumnMajorTransform;

    return sourceToTargetColumnMajorTransform;
}

} // namespace pose
} // namespace includes
} // namespace core
} // namespace isaacsim
