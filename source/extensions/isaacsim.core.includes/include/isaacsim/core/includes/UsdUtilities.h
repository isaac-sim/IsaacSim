// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

// clang-format off
#include <omni/usd/UsdContextIncludes.h>
// clang-format on

#include <omni/usd/UsdContext.h>

#include <chrono>
#include <iostream>
#include <string>


PXR_NAMESPACE_OPEN_SCOPE
inline bool GfIsClose(const pxr::GfQuatd& val1, const pxr::GfQuatd& val2, double tolerance)
{
    bool result1 = pxr::GfIsClose(val1.GetReal(), val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), val2.GetImaginary(), tolerance);
    bool result2 = GfIsClose(val1.GetReal(), -val2.GetReal(), tolerance) &&
                   GfIsClose(val1.GetImaginary(), -val2.GetImaginary(), tolerance);
    return result1 || result2;
}
PXR_NAMESPACE_CLOSE_SCOPE

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @brief Token for overriding prim names in Isaac Sim.
 * @details Used to specify custom names for prims that differ from their USD names.
 */
static const PXR_NS::TfToken g_kIsaacNameOveride{ "isaac:nameOverride" };

/**
 * enum to show effective timesamples in layerstacks based on current authoring layer
 */
enum class TimeSamplesOnLayer
{
    eNoTimeSamples,
    eOnCurrentLayer,
    eOnStrongerLayer,
    eOnWeakerLayer
};

/**
 * Checks if a UsdAttribute instance is time sampled.
 *
 * @param attribute The UsdAttribute to be checked.
 * @return True if the attribute is timesampled.
 */
inline bool isTimeSampled(const pxr::UsdAttribute& attribute)
{
    return attribute.GetNumTimeSamples() > 0;
}

/**
 * Checks if a UsdAttribute instance has time sample on key timeCode
 *
 * @param attribute The UsdAttribute to be checked.
 * @param timeCode The timeCode to be checked.
 * @return True if the attribute has timesampled on key timeCode.
 */
inline bool hasTimeSample(const pxr::UsdAttribute& attribute, pxr::UsdTimeCode timeCode)
{
    if (timeCode.IsDefault())
    {
        return false;
    }

    std::vector<double> times;
    if (attribute.GetTimeSamples(&times))
    {
        double timeCodeValue = timeCode.GetValue();
        if (round(timeCodeValue) != timeCode.GetValue())
        {
            CARB_LOG_WARN("Error : Try to identify attribute %s has time sample on a fractinal key frame %f",
                          attribute.GetPath().GetText(), timeCodeValue);
            return false;
        }
        return std::find(times.begin(), times.end(), timeCodeValue) != times.end();
    }
    return false;
}

/**
 * check if attribute has efficient timesample and
 * these data are on currentlayer/strongerlayer/weakerlayer
 * @param[in] stage Current Working Stage.
 * @param[in] attr The attribute to check.
 * @param[out] outLayer Optional pointer to receive the layer containing time samples
 * @return TimeSamplesOnLayer enum indicating where time samples are located
 */
inline TimeSamplesOnLayer getAttributeEffectiveTimeSampleLayerInfo(const pxr::UsdStage& stage,
                                                                   const pxr::UsdAttribute& attr,
                                                                   pxr::SdfLayerRefPtr* outLayer = nullptr)
{
    if (attr.GetNumTimeSamples() == 0)
    {
        return TimeSamplesOnLayer::eNoTimeSamples;
    }

    auto authoringLayer = stage.GetEditTarget().GetLayer();

    bool isOnStrongerLayer = true;
    const pxr::PcpLayerStackPtr& nodeLayers = attr.GetResolveInfo().GetNode().GetLayerStack();
    const pxr::SdfLayerRefPtrVector& layerStack = nodeLayers->GetLayers();
    for (auto layer : layerStack)
    {
        auto attrSpec = layer->GetAttributeAtPath(attr.GetPath());
        if (attrSpec && !attrSpec->GetTimeSampleMap().empty())
        {
            if (outLayer)
            {
                *outLayer = layer;
            }
            if (layer == authoringLayer)
            {
                return TimeSamplesOnLayer::eOnCurrentLayer;
            }
            else if (isOnStrongerLayer)
            {
                return TimeSamplesOnLayer::eOnStrongerLayer;
            }
            else
            {
                return TimeSamplesOnLayer::eOnWeakerLayer;
            }
        }
        else
        {
            if (layer == authoringLayer)
            {
                isOnStrongerLayer = false;
            }
        }
    }
    return TimeSamplesOnLayer::eNoTimeSamples;
}

/**
 * Copy TimeSample From Waker Layer.
 *
 * @param[in,out] stage Current Working Stage.
 * @param[in] attr The attribute to check.
 */
inline void copyTimeSamplesFromWeakerLayer(pxr::UsdStage& stage, const pxr::UsdAttribute& attr)
{
    pxr::SdfLayerRefPtr outLayer;
    if (getAttributeEffectiveTimeSampleLayerInfo(stage, attr, &outLayer) == TimeSamplesOnLayer::eOnWeakerLayer)
    {
        pxr::SdfTimeSampleMap timesamples;
        if (attr.GetMetadata(pxr::TfToken("timeSamples"), &timesamples))
        {
            attr.SetMetadata(pxr::TfToken("timeSamples"), timesamples);
        }
    }
}


/**
 * @brief Finds if the given path has a spec on session layer or its sublayers
 * @details
 * Searches through session layer and its sublayers to find if a specification (prim, attribute,
 * property, or other USD object) exists at the given path. Optionally applies a predicate
 * function to validate the found specification.
 *
 * @param[in] stage USD stage containing the layers to search
 * @param[in] path The USD path to check for specification existence
 * @param[in] predicate Optional validation function called if spec is found on the layer
 * @return SdfLayerRefPtr Layer that contains the spec at given path, or nullptr if not found
 *
 * @note Searches only session layers and their sublayers, stops at root layer
 * @see getLayerIfDefOnSessionOrItsSublayers for prim-specific "def" search
 */
inline PXR_NS::SdfLayerRefPtr getLayerIfSpecOnSessionOrItsSublayers(
    PXR_NS::UsdStageRefPtr stage,
    const PXR_NS::SdfPath& path,
    const std::function<bool(PXR_NS::SdfSpecHandle)>& predicate = nullptr)
{
    const auto rootLayer = stage->GetRootLayer();
    const auto layerStack = stage->GetLayerStack(true);

    auto hasSpec = [&path, &predicate](PXR_NS::SdfLayerRefPtr layer)
    {
        auto spec = layer->GetObjectAtPath(path);

        return spec && (!predicate || predicate(spec));
    };

    for (const auto& layer : layerStack)
    {
        // session layers always come before root layer. We're only interested in session and its
        // sublayers, break when hit root layer.
        if (layer == rootLayer)
        {
            break;
        }

        if (hasSpec(layer))
        {
            return layer;
        }
    }

    return nullptr;
}

/**
 * @brief Finds if the given prim path has a "def" primSpec on session layer or its sublayers
 * @details
 * Specialized version of getLayerIfSpecOnSessionOrItsSublayers that specifically looks for
 * prim specifications with "def" specifier type. This excludes "over" prims and focuses
 * only on defining prim specifications.
 *
 * @param[in] stage USD stage containing the layers to search
 * @param[in] path The prim path to check for "def" primSpec
 * @return SdfLayerRefPtr Layer that contains the "def" prim spec, or nullptr if not found
 *
 * @note If you want to find attributeSpec use getLayerIfSpecOnSessionOrItsSublayers instead
 * @see getLayerIfSpecOnSessionOrItsSublayers for general spec search
 */
inline PXR_NS::SdfLayerRefPtr getLayerIfDefOnSessionOrItsSublayers(PXR_NS::UsdStageRefPtr stage,
                                                                   const PXR_NS::SdfPath& path)
{
    return getLayerIfSpecOnSessionOrItsSublayers(
        stage, path,
        [](PXR_NS::SdfSpecHandle spec)
        {
            auto primSpec = PXR_NS::SdfSpecStatic_cast<PXR_NS::SdfPrimSpecHandle>(spec);
            return primSpec && PXR_NS::SdfIsDefiningSpecifier(primSpec->GetSpecifier());
        });
}


/**
 * @brief Sets attribute value with optional time sampling and session layer targeting
 * @details
 * Provides functionality to set USD attribute values with advanced features:
 * - Automatic session layer targeting for better edit workflow
 * - Time sample management and optimization
 * - Value comparison to avoid redundant writes
 *
 * @tparam ValueType The data type of the attribute value
 * @param[in] attribute The USD attribute to set value on
 * @param[in] val The value to set
 * @param[in] timeCode Time code for time-sampled attributes (default: UsdTimeCode::Default())
 * @param[in] skipEqualSetForTimeSample Whether to skip setting if value is equal for time samples
 * @param[in] autoTargetSessionLayer Whether the edit target should auto switch to session layer
 * @return True if the value was successfully set, false otherwise
 *
 * @note For time-sampled attributes, will copy time samples from weaker layers if needed
 * @warning Requires valid USD context and stage
 */
template <class ValueType>
inline bool setAttribute(const pxr::UsdAttribute& attribute,
                         const ValueType& val,
                         pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default(),
                         bool skipEqualSetForTimeSample = false,
                         bool autoTargetSessionLayer = true)
{
    PXR_NS::UsdTimeCode setTimeCode = timeCode;
    if (!isTimeSampled(attribute))
    {
        setTimeCode = PXR_NS::UsdTimeCode::Default();
    }

    // This is here to prevent the TransformGizmo from writing a translation, rotation and scale on every
    // key where it sets a value. At some point we should revisit the gizmo to simplify the logic, and
    // start setting only the transform value the user intends.
    if (skipEqualSetForTimeSample)
    {
        if (!setTimeCode.IsDefault() && !hasTimeSample(attribute, setTimeCode))
        {
            ValueType value;
            bool result = attribute.Get(&value, timeCode);
            if (result && PXR_NS::GfIsClose(value, val, 1e-6))
            {
                return false;
            }
        }
    }

    // if the prim is defined on session layer, or the attribute itself is on session layer, switch EditTarget to
    // session layer instead
    std::unique_ptr<PXR_NS::UsdEditContext> editCtx;
    auto stage = attribute.GetStage();
    if (autoTargetSessionLayer)
    {
        // check first if the attribute is on session layer
        auto sessionOrSubLayer = getLayerIfSpecOnSessionOrItsSublayers(stage, attribute.GetPath());
        if (!sessionOrSubLayer)
        {
            // if attribute doesn't exist, fallback to prim "def" (but not "over")
            sessionOrSubLayer = getLayerIfDefOnSessionOrItsSublayers(stage, attribute.GetPrim().GetPath());
        }
        if (sessionOrSubLayer)
        {
            editCtx = std::make_unique<PXR_NS::UsdEditContext>(stage, sessionOrSubLayer);
        }
    }

    if (!setTimeCode.IsDefault())
    {
        copyTimeSamplesFromWeakerLayer(*stage, attribute);
    }

    return attribute.Set(val, setTimeCode);
}


/**
 * @brief Retrieves the camera prim associated with a render product.
 * @details
 * Looks up the camera prim referenced by a render product in the current USD stage.
 * The function performs several validation steps:
 * 1. Checks for valid stage and interfaces
 * 2. Creates render product prim handle
 * 3. Extracts camera path and retrieves corresponding prim
 *
 * @param[in] renderProductPathString Path to the render product as a string
 * @return pxr::UsdPrim Camera prim if found, invalid prim otherwise
 *
 * @note Returns invalid prim if any required component is missing
 * @warning Requires valid USD context and stage
 */
inline pxr::UsdPrim getCameraPrimFromRenderProduct(const std::string& renderProductPathString)
{
    // Get the current USD stage
    pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
    if (!stage)
    {
        return pxr::UsdPrim();
    }

    // Get the render product prim
    pxr::UsdPrim renderProductPrim = stage->GetPrimAtPath(pxr::SdfPath(renderProductPathString));
    if (!renderProductPrim.IsValid())
    {
        return pxr::UsdPrim();
    }

    // Get the camera relationship from the render product
    pxr::UsdRelationship cameraPathRel = renderProductPrim.GetRelationship(pxr::TfToken("camera"));
    if (!cameraPathRel.IsValid() || !cameraPathRel.HasAuthoredTargets())
    {
        return pxr::UsdPrim();
    }

    // Get the camera targets
    pxr::SdfPathVector targets;
    cameraPathRel.GetTargets(&targets);
    if (targets.empty())
    {
        return pxr::UsdPrim();
    }

    // Get the camera prim from the first target
    pxr::UsdPrim cameraPrim = stage->GetPrimAtPath(targets[0]);
    if (!cameraPrim.IsValid())
    {
        return pxr::UsdPrim();
    }

    return cameraPrim;
}

/**
 * @brief Retrieves a specific attribute from a camera associated with a render product.
 * @details
 * Combines camera prim lookup with attribute access in a single function.
 * Process:
 * 1. Gets the camera prim from render product
 * 2. Validates the camera prim
 * 3. Retrieves the requested attribute
 *
 * @param[in] attributeString Name of the attribute to retrieve
 * @param[in] renderProductPathString Path to the render product
 * @return pxr::UsdAttribute The requested attribute if found, invalid attribute otherwise
 *
 * @note Returns invalid attribute if camera prim is invalid
 * @see getCameraPrimFromRenderProduct
 */
inline pxr::UsdAttribute getCameraAttributeFromRenderProduct(const std::string& attributeString,
                                                             const std::string& renderProductPathString)
{
    pxr::UsdPrim cameraPrim = getCameraPrimFromRenderProduct(renderProductPathString);
    if (!cameraPrim.IsValid())
    {
        return pxr::UsdAttribute();
    }
    return cameraPrim.GetAttribute(pxr::TfToken(attributeString.c_str()));
}

/**
 * @brief Safely retrieves a USD attribute value with error handling.
 * @details
 * Provides a safe way to get attribute values with:
 * - Value existence checking
 * - Type-safe value retrieval
 *
 * @tparam T Type of the attribute value to retrieve
 * @param[in] attr USD attribute to read
 * @param[out] inputValue Variable to store the attribute value
 *
 * @note Logs a warning if attribute exists but has no value
 * @warning Input value remains unchanged if attribute has no value
 */
template <class T>
void safeGetAttribute(const pxr::UsdAttribute& attr, T& inputValue)
{

    if (attr.HasValue())
    {
        attr.Get(&inputValue);
    }
}

/**
 * @brief Retrieves the name of a USD prim, with support for custom overrides.
 * @details
 * Determines the name of a prim by:
 * 1. Using the default prim name from USD
 * 2. Checking for "isaac:nameOverride" attribute to allow custom naming
 * 3. Using the override value if present and not empty
 *
 * @param[in] prim USD prim whose name to retrieve
 * @return std::string Name of the prim, potentially from override attribute
 *
 * @see g_kIsaacNameOveride
 * @see safeGetAttribute
 */
inline std::string getName(const pxr::UsdPrim& prim)
{
    if (prim.HasAttribute(g_kIsaacNameOveride))
    {
        std::string primNameOverride;
        safeGetAttribute<std::string>(prim.GetAttribute(g_kIsaacNameOveride), primNameOverride);
        if (!primNameOverride.empty())
        {
            return primNameOverride;
        }
    }
    return prim.GetName().GetString();
}

/**
 * @brief Converts a uint64_t token to a USD path.
 * @details
 * Performs a reinterpret cast from uint64_t to SdfPath with:
 * - Platform-specific warning suppression
 * - Safe type conversion
 *
 * @param[in] pathToken Path token as uint64_t
 * @return pxr::SdfPath Converted USD path
 *
 * @warning Uses reinterpret_cast, ensure token is a valid path representation
 */
inline pxr::SdfPath getSdfPathFromUint64(uint64_t pathToken)
{
#if defined(_WIN32)
#    pragma warning(push)
#    pragma warning(disable : 4996)
    return reinterpret_cast<const pxr::SdfPath&>(pathToken);
#    pragma warning(pop)
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wstrict-aliasing"
    return reinterpret_cast<const pxr::SdfPath&>(pathToken);
#    pragma GCC diagnostic pop
#endif
}

/**
 * @brief Gets world transform matrix of a prim.
 *
 * @param[in] prim The prim to get world transform matrix from.
 * @param[in] time Current timecode.
 * @return The world transform matrix.
 */
inline pxr::GfMatrix4d getWorldTransformMatrix(const pxr::UsdPrim& prim,
                                               pxr::UsdTimeCode time = pxr::UsdTimeCode::Default())
{
    pxr::UsdGeomXformable xform(prim);
    return xform.ComputeLocalToWorldTransform(time);
}

/**
 * @brief Gets local transform matrix of a prim.
 *
 * @param[in] prim The prim to get local transform matrix from.
 * @param[in] time Current timecode.
 * @return The local transform matrix.
 */
inline pxr::GfMatrix4d getLocalTransformMatrix(const pxr::UsdPrim& prim,
                                               pxr::UsdTimeCode time = pxr::UsdTimeCode::Default())
{
    bool resetXformStack = false;
    pxr::UsdGeomXformable xform(prim);
    pxr::GfMatrix4d mat;

    xform.GetLocalTransformation(&mat, &resetXformStack, time);

    return mat;
}

/**
 * Given a target local transform matrix for a prim, determine what value to set just
 * the transformOp when other xformOps are present.
 *
 * @param prim The prim in question
 * @param mtx The desired final transform matrix for the prim including all ops
 * @param foundTransformOp returns true if there is a transform xformOp
 */
inline pxr::GfMatrix4d findInnerTransform(pxr::UsdPrim prim,
                                          const pxr::GfMatrix4d& mtx,
                                          bool& foundTransformOp,
                                          pxr::UsdTimeCode timecode = pxr::UsdTimeCode::Default(),
                                          bool skipEqualSetForTimeSample = false)
{
    pxr::UsdGeomXformable xform(prim);

    bool resetXFormStack;
    auto xformOps = xform.GetOrderedXformOps(&resetXFormStack);
    foundTransformOp = false;
    bool foundOtherOps = false;
    pxr::GfMatrix4d preTransform = pxr::GfMatrix4d(1.);
    pxr::GfMatrix4d postTransform = pxr::GfMatrix4d(1.);

    for (auto xformOp : xformOps)
    {
        if (!foundTransformOp && xformOp.GetOpType() == pxr::UsdGeomXformOp::TypeTransform)
        {
            foundTransformOp = true;
        }
        else
        {
            bool isInverseOp = false;
            pxr::UsdGeomXformOp op(xformOp.GetAttr(), isInverseOp);
            if (op)
            {
                static const PXR_NS::TfToken kPivotSuffix("pivot");
                if (op.HasSuffix(kPivotSuffix))
                {
                    continue;
                }

                // possibly check for identity and skip multiplication
                auto opTransform = op.GetOpTransform(timecode);
                if (foundTransformOp)
                {
                    preTransform = opTransform * preTransform;
                }
                else
                {
                    postTransform = opTransform * postTransform;
                }
                foundOtherOps = true;
            }
        }
    }

    if (foundTransformOp && foundOtherOps)
    {
        return preTransform.GetInverse() * mtx * postTransform.GetInverse();
    }

    return mtx;
}

/**
 * Set value with precision based on the UsdGeomXformOp precision type
 */
template <class HalfType, class FloatType, class DoubleType, class ValueType>
inline bool setValueWithPrecision(pxr::UsdGeomXformOp& xformOp,
                                  const ValueType& value,
                                  pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default(),
                                  bool skipEqualSetForTimeSample = false)
{
    switch (xformOp.GetPrecision())
    {
    case pxr::UsdGeomXformOp::PrecisionHalf:
        return setAttribute(xformOp.GetAttr(), HalfType(FloatType(value)), timeCode, skipEqualSetForTimeSample);
    case pxr::UsdGeomXformOp::PrecisionFloat:
        return setAttribute(xformOp.GetAttr(), FloatType(value), timeCode, skipEqualSetForTimeSample);
    case pxr::UsdGeomXformOp::PrecisionDouble:
        return setAttribute(xformOp.GetAttr(), DoubleType(value), timeCode, skipEqualSetForTimeSample);
    }
    return false;
}

/**
 * Sets local transform matrix of a prim.
 *
 * @param[in,out] prim The prim to set local transform matrix to.
 * @param[in] mtxIn The local transform matrix.
 * @param[in] timecode Time code for the transform operation.
 * @param[in] skipEqualSetForTimeSample Whether to skip setting if value is equal.
 * @param[in,out] parentChangeBlock Optional parent change block for batching operations.
 * @return true if the operation was successful, false otherwise.
 */
inline bool setLocalTransformMatrix(pxr::UsdPrim prim,
                                    const pxr::GfMatrix4d& mtxIn,
                                    pxr::UsdTimeCode timecode = pxr::UsdTimeCode::Default(),
                                    bool skipEqualSetForTimeSample = false,
                                    std::unique_ptr<PXR_NS::SdfChangeBlock>* parentChangeBlock = nullptr)
{
    // If prim is defined in session layer, we author in session layer.
    std::unique_ptr<PXR_NS::UsdEditContext> editCtx;
    auto mightDefOnSessionLayer = getLayerIfDefOnSessionOrItsSublayers(prim.GetStage(), prim.GetPath());
    if (mightDefOnSessionLayer)
    {
        editCtx = std::make_unique<PXR_NS::UsdEditContext>(prim.GetStage(), mightDefOnSessionLayer);
    }

    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    static constexpr char kAuthorXformsWithFastUpdatesSettingPath[] = "/omni.kit.plugin/authorXformsWithFastUpdates";
    bool fastUpdates = settings->getAsBool(kAuthorXformsWithFastUpdatesSettingPath);
    std::unique_ptr<PXR_NS::SdfChangeBlock> localChangeBlock;
    std::unique_ptr<PXR_NS::SdfChangeBlock>& changeBlock =
        (parentChangeBlock != nullptr) ? *parentChangeBlock : localChangeBlock;
    if (!changeBlock.get())
    {
        // https://github.com/PixarAnimationStudios/USD/commit/5e38b2aac0693fcf441a607165346e42cd625b59
        // fastUpdates have long been deprecated, and will be removed from the API
        // in nv-usd 22.05, as they not conflict with the "enabled" parameter
        // added by Pixar in USD v22.03
        changeBlock.reset(new PXR_NS::SdfChangeBlock());
    }

    pxr::UsdGeomXformable xform(prim);

    bool resetXFormStack;
    auto xformOps = xform.GetOrderedXformOps(&resetXFormStack);
    PXR_NS::VtTokenArray xformOpOrders;
    xform.GetXformOpOrderAttr().Get(&xformOpOrders);
    bool foundTransformOp = false;
    PXR_NS::UsdGeomXformOp transformOp;
    bool success = true;
    pxr::GfMatrix4d mtx = mtxIn;

    const pxr::GfMatrix4d innerMtx = findInnerTransform(prim, mtx, foundTransformOp, timecode, skipEqualSetForTimeSample);

    for (auto xformOp : xformOps)
    {
        // Found transform op, trying to set its value
        if (xformOp.GetOpType() == pxr::UsdGeomXformOp::TypeTransform)
        {
            foundTransformOp = true;
            success &= setAttribute(xformOp.GetAttr(), innerMtx, timecode, skipEqualSetForTimeSample);
        }
    }

    // If transformOp is not found, make individual xformOp or reuse old ones.
    if (!foundTransformOp)
    {
        // A.B.temp solution to unblock a showstopper for supporting unitsResolve suffix xformOp stacks
        static const PXR_NS::TfToken kUnitsResolve = PXR_NS::TfToken("unitsResolve");
        bool preTransformStack = false;
        std::vector<PXR_NS::UsdGeomXformOp> extraXformOps;

        for (auto& xformOp : xformOps)
        {
            if (xformOp.HasSuffix(kUnitsResolve))
            {
                extraXformOps.push_back(xformOp);
                if (xformOp == xformOps[0])
                {
                    preTransformStack = true;
                }
            }
        }

        // A.B.temp solution to unblock a showstopper for supporting unitsResolve suffix xformOp stacks
        // reconstruct the values back after modifying the incoming transform
        if (!extraXformOps.empty())
        {
            PXR_NS::GfMatrix4d extraTransform(1.0);
            for (auto& extraOp : extraXformOps)
            {
                extraTransform *= extraOp.GetOpTransform(timecode);
            }
            const PXR_NS::GfMatrix4d extraTransformInv = extraTransform.GetInverse();
            if (preTransformStack)
            {
                mtx = mtx * extraTransformInv;
            }
            else
            {
                mtx = extraTransformInv * mtx;
            }
        }

        pxr::GfVec3d translation;
        pxr::GfMatrix4d rotMat(1.0);
        pxr::GfVec3d doubleScale(1.0);
        pxr::GfMatrix4d scaleOrientMatUnused, perspMatUnused;
        mtx.Factor(&scaleOrientMatUnused, &doubleScale, &rotMat, &translation, &perspMatUnused);

        rotMat.Orthonormalize(false);
        pxr::GfRotation rotation = rotMat.ExtractRotation();

        // Don't use UsdGeomXformCommonAPI. It can only manipulate a very limited subset of xformOpOrder
        // combinations Do it manually as non-destructively as possible
        pxr::UsdGeomXformOp xformOp;
        std::vector<pxr::UsdGeomXformOp> newXformOps;

        if (!extraXformOps.empty() && preTransformStack)
        {
            for (auto& copyXformOp : extraXformOps)
            {
                newXformOps.push_back(copyXformOp);
            }
        }

        auto findOrAdd = [&xformOps, &xformOpOrders, &xform, &changeBlock, &fastUpdates](
                             pxr::UsdGeomXformOp::Type xformOpType, pxr::UsdGeomXformOp& outXformOp,
                             bool createIfNotExist, pxr::UsdGeomXformOp::Precision& precision,
                             pxr::TfToken const& opSuffix = pxr::TfToken())
        {
            for (auto xformOp : xformOps)
            {
                if (xformOp.GetOpType() == xformOpType)
                {
                    // To differentiate translate and translate:pivot
                    const pxr::TfToken expectedOpName = pxr::UsdGeomXformOp::GetOpName(xformOpType, opSuffix);
                    const pxr::TfToken opName = xformOp.GetOpName();
                    if (opName == expectedOpName)
                    {
                        precision = xformOp.GetPrecision();
                        outXformOp = xformOp;
                        return true;
                    }
                }
            }

            if (createIfNotExist)
            {
                // It is not safe to create new xformOps inside of SdfChangeBlocks, since
                // new attribute creation via anything above Sdf API requires the PcpCache
                // to be up to date. Flush the current change block before creating
                // the new xformOp.
                changeBlock.reset(nullptr);
                if (std::find(xformOpOrders.begin(), xformOpOrders.end(),
                              pxr::UsdGeomXformOp::GetOpName(xformOpType, opSuffix)) == xformOpOrders.end())
                {
                    outXformOp = xform.AddXformOp(xformOpType, precision, opSuffix);
                }
                else
                {
                    // Sometimes XformOp attributes and XformOpOrder don't match. GetOrderedXformOps() considers both
                    // XformOp attributes and XformOpOrder. But AddXformOp() considers only XformOpOrder. So we need to
                    // fix it here.
                    auto opAttr = xform.GetPrim().CreateAttribute(
                        pxr::UsdGeomXformOp::GetOpName(xformOpType, opSuffix),
                        pxr::UsdGeomXformOp::GetValueTypeName(xformOpType, precision), false);
                    outXformOp = pxr::UsdGeomXformOp(opAttr);
                }

                // Create a new change block to batch the subsequent authoring operations
                // where possible.
                changeBlock.reset(new PXR_NS::SdfChangeBlock());
                // Creation may have failed for a variety of reasons (including instanceable=True)
                return static_cast<bool>(outXformOp);
            }
            return false;
        };

        auto getFirstRotateOpType = [&xformOps](pxr::UsdGeomXformOp::Precision& precision)
        {
            for (auto xformOp : xformOps)
            {
                if (xformOp.GetOpType() >= pxr::UsdGeomXformOp::Type::TypeRotateX &&
                    xformOp.GetOpType() <= pxr::UsdGeomXformOp::Type::TypeOrient && !xformOp.HasSuffix(kUnitsResolve))
                {
                    precision = xformOp.GetPrecision();
                    return xformOp.GetOpType();
                }
            }
            return pxr::UsdGeomXformOp::Type::TypeInvalid;
        };

        auto decomposeAndSetValue = [&rotation, &findOrAdd, &newXformOps](
                                        pxr::UsdGeomXformOp::Type rotationType, const pxr::GfVec3d& axis0,
                                        const pxr::GfVec3d& axis1, const pxr::GfVec3d& axis2, size_t xIndex,
                                        size_t yIndex, size_t zIndex, pxr::UsdGeomXformOp::Precision precision,
                                        pxr::UsdTimeCode timecode, bool skipEqualSetForTimeSample)
        {
            bool ret = false;
            pxr::GfVec3d angles = rotation.Decompose(axis0, axis1, axis2);
            pxr::GfVec3d rotate = { angles[xIndex], angles[yIndex], angles[zIndex] };
            pxr::UsdGeomXformOp xformOp;
            if (findOrAdd(rotationType, xformOp, true, precision))
            {
                ret = setValueWithPrecision<pxr::GfVec3h, pxr::GfVec3f, pxr::GfVec3d, pxr::GfVec3d>(
                    xformOp, rotate, timecode, skipEqualSetForTimeSample);
                newXformOps.push_back(xformOp);
            }
            return ret;
        };

        // Set translation
        pxr::UsdGeomXformOp::Precision precision = pxr::UsdGeomXformOp::PrecisionDouble;
        if (findOrAdd(pxr::UsdGeomXformOp::TypeTranslate, xformOp, true, precision))
        {
            success &= setValueWithPrecision<pxr::GfVec3h, pxr::GfVec3f, pxr::GfVec3d, pxr::GfVec3d>(
                xformOp, translation, timecode, skipEqualSetForTimeSample);
            newXformOps.push_back(xformOp);
        }

        // Set pivot
        static const pxr::TfToken kPivot = pxr::TfToken("pivot");
        precision = pxr::UsdGeomXformOp::PrecisionFloat;
        pxr::UsdGeomXformOp pivotOp;
        pxr::UsdGeomXformOp pivotOpInv;
        pxr::GfVec3d pivotValue(0., 0., 0.);
        const bool hasPivot = findOrAdd(pxr::UsdGeomXformOp::TypeTranslate, pivotOp, false, precision, kPivot);
        if (hasPivot)
        {
            newXformOps.push_back(pivotOp);
            for (size_t k = xformOps.size(); k--;)
            {
                if (xformOps[k].IsInverseOp() && xformOps[k].HasSuffix(kPivot))
                {
                    pivotOpInv = xformOps[k];
                    break;
                }
            }
        }

        // Set rotation
        precision = pxr::UsdGeomXformOp::PrecisionFloat;
        auto firstRotateOpType = getFirstRotateOpType(precision);

        if (firstRotateOpType == pxr::UsdGeomXformOp::TypeInvalid)
        {
            static const std::unordered_map<std::string, PXR_NS::UsdGeomXformOp::Type> s_typeMap{
                { "XYZ", PXR_NS::UsdGeomXformOp::TypeRotateXYZ }, { "XZY", PXR_NS::UsdGeomXformOp::TypeRotateXZY },
                { "YXZ", PXR_NS::UsdGeomXformOp::TypeRotateYXZ }, { "YZX", PXR_NS::UsdGeomXformOp::TypeRotateYZX },
                { "ZXY", PXR_NS::UsdGeomXformOp::TypeRotateZXY }, { "ZYX", PXR_NS::UsdGeomXformOp::TypeRotateZYX },
            };

            firstRotateOpType = PXR_NS::UsdGeomXformOp::TypeRotateXYZ; // fallback
            static constexpr char kDefaultRotationOrderSettingPath[] =
                "/persistent/app/primCreation/DefaultRotationOrder";
            const char* orderStr =
                carb::getCachedInterface<carb::settings::ISettings>()->getStringBuffer(kDefaultRotationOrderSettingPath);
            const auto orderEntry = s_typeMap.find(orderStr);
            if (orderEntry != s_typeMap.end())
            {
                firstRotateOpType = orderEntry->second;
            }
        }

        switch (firstRotateOpType)
        {
        case pxr::UsdGeomXformOp::TypeRotateX:
        case pxr::UsdGeomXformOp::TypeRotateY:
        case pxr::UsdGeomXformOp::TypeRotateZ:
        {
            pxr::GfVec3d angles = rotation.Decompose(pxr::GfVec3d::ZAxis(), pxr::GfVec3d::YAxis(), pxr::GfVec3d::XAxis());
            pxr::GfVec3d rotateZYX = { angles[2], angles[1], angles[0] };
            if (findOrAdd(pxr::UsdGeomXformOp::TypeRotateZ, xformOp, true, precision))
            {
                success &= setValueWithPrecision<pxr::GfHalf, float, double, double>(
                    xformOp, rotateZYX[2], timecode, skipEqualSetForTimeSample);
                newXformOps.push_back(xformOp);
            }
            if (findOrAdd(pxr::UsdGeomXformOp::TypeRotateY, xformOp, true, precision))
            {
                success &= setValueWithPrecision<pxr::GfHalf, float, double, double>(
                    xformOp, rotateZYX[1], timecode, skipEqualSetForTimeSample);
                newXformOps.push_back(xformOp);
            }
            if (findOrAdd(pxr::UsdGeomXformOp::TypeRotateX, xformOp, true, precision))
            {
                success &= setValueWithPrecision<pxr::GfHalf, float, double, double>(
                    xformOp, rotateZYX[0], timecode, skipEqualSetForTimeSample);
                newXformOps.push_back(xformOp);
            }
            break;
        }
        case pxr::UsdGeomXformOp::TypeRotateZYX:
            success &=
                decomposeAndSetValue(firstRotateOpType, pxr::GfVec3d::XAxis(), pxr::GfVec3d::YAxis(),
                                     pxr::GfVec3d::ZAxis(), 0, 1, 2, precision, timecode, skipEqualSetForTimeSample);
            break;
        case pxr::UsdGeomXformOp::TypeRotateXZY:
            success &=
                decomposeAndSetValue(firstRotateOpType, pxr::GfVec3d::YAxis(), pxr::GfVec3d::ZAxis(),
                                     pxr::GfVec3d::XAxis(), 2, 0, 1, precision, timecode, skipEqualSetForTimeSample);
            break;
        case pxr::UsdGeomXformOp::TypeRotateYXZ:
            success &=
                decomposeAndSetValue(firstRotateOpType, pxr::GfVec3d::ZAxis(), pxr::GfVec3d::XAxis(),
                                     pxr::GfVec3d::YAxis(), 1, 2, 0, precision, timecode, skipEqualSetForTimeSample);
            break;
        case pxr::UsdGeomXformOp::TypeRotateYZX:
            success &=
                decomposeAndSetValue(firstRotateOpType, pxr::GfVec3d::XAxis(), pxr::GfVec3d::ZAxis(),
                                     pxr::GfVec3d::YAxis(), 0, 2, 1, precision, timecode, skipEqualSetForTimeSample);
            break;
        case pxr::UsdGeomXformOp::TypeRotateZXY:
            success &=
                decomposeAndSetValue(firstRotateOpType, pxr::GfVec3d::YAxis(), pxr::GfVec3d::XAxis(),
                                     pxr::GfVec3d::ZAxis(), 1, 0, 2, precision, timecode, skipEqualSetForTimeSample);
            break;
        case pxr::UsdGeomXformOp::TypeOrient:
            if (findOrAdd(pxr::UsdGeomXformOp::TypeOrient, xformOp, false, precision))
            {
                success &= setValueWithPrecision<pxr::GfQuath, pxr::GfQuatf, pxr::GfQuatd, pxr::GfQuatd>(
                    xformOp, rotation.GetQuat(), timecode, skipEqualSetForTimeSample);
                newXformOps.push_back(xformOp);
            }
            break;
        case pxr::UsdGeomXformOp::TypeRotateXYZ:
        default:
            success &=
                decomposeAndSetValue(pxr::UsdGeomXformOp::TypeRotateXYZ, pxr::GfVec3d::ZAxis(), pxr::GfVec3d::YAxis(),
                                     pxr::GfVec3d::XAxis(), 2, 1, 0, precision, timecode, skipEqualSetForTimeSample);
            break;
        }

        // Set scale
        precision = pxr::UsdGeomXformOp::PrecisionFloat;
        if (findOrAdd(pxr::UsdGeomXformOp::TypeScale, xformOp, true, precision))
        {
            success &= setValueWithPrecision<pxr::GfVec3h, pxr::GfVec3f, pxr::GfVec3d, pxr::GfVec3d>(
                xformOp, doubleScale, timecode, skipEqualSetForTimeSample);
            newXformOps.push_back(xformOp);
        }

        // Set extra ops from units resolve
        if (!extraXformOps.empty() && !preTransformStack)
        {
            for (auto& copyXformOp : extraXformOps)
            {
                newXformOps.push_back(copyXformOp);
            }
        }

        // Set inverse pivot
        if (hasPivot && pivotOpInv)
        {
            // Assume the last xformOps is the pivot
            newXformOps.push_back(pivotOpInv);
        }

        success &= xform.SetXformOpOrder(newXformOps, resetXFormStack);
    }
    return success;
}

} // namespace includes
} // namespace core
} // namespace isaacsim
