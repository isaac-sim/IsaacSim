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

// clang-format off
#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/base/gf/matrix4d.h>
// clang-format on

/**
 * Select a existing layer as edit target.
 *
 * @param stage The stage of the operation.
 * @param layerIdentifier Layer identifier.
 * @return true if the layer is selected, false otherwise.
 *
 **/
namespace isaacsim
{
namespace core
{
namespace includes
{
namespace utils
{
namespace usd
{


/**
 * Select a existing layer as edit target.
 *
 * @param stage The stage of the operation.
 * @param layerIdentifier Layer identifier.
 * @return true if the layer is selected, false otherwise.
 *
 **/
inline bool setAuthoringLayer(pxr::UsdStageRefPtr stage, const std::string& layerIdentifier)
{
    const auto& sublayer = pxr::SdfLayer::Find(layerIdentifier);
    if (!sublayer || !stage->HasLocalLayer(sublayer))
    {
        return false;
    }

    pxr::UsdEditTarget editTarget = stage->GetEditTargetForLocalLayer(sublayer);
    stage->SetEditTarget(editTarget);

    return true;
}

inline pxr::GfMatrix4d GetGlobalTransform(const pxr::UsdPrim& prim)
{
    pxr::GfMatrix4d globalTransform(1.0);

    pxr::UsdPrim currentPrim = prim;
    while (currentPrim && currentPrim.IsValid())
    {
        if (pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(currentPrim))
        {
            bool resetsXformStack = false;
            pxr::GfMatrix4d localTransform;
            xformable.GetLocalTransformation(&localTransform, &resetsXformStack);

            globalTransform = localTransform * globalTransform;

            if (resetsXformStack)
            {
                break;
            }
        }
        currentPrim = currentPrim.GetParent();
    }

    return globalTransform;
}
// Make a path name that is not already used.
inline std::string GetNewSdfPathString(pxr::UsdStageWeakPtr stage, std::string path, int nameClashNum = 0)
{
    bool appendedNumber = false;
    int numberAppended = std::max<int>(nameClashNum, 0);
    size_t indexOfNumber = 0;
    if (stage->GetPrimAtPath(pxr::SdfPath(path)))
    {
        appendedNumber = true;
        std::string name = pxr::SdfPath(path).GetName();
        size_t last_ = name.find_last_of('_');
        indexOfNumber = path.length() + 1;
        if (last_ == std::string::npos)
        {
            // no '_' found, so just tack on the end.
            path += "_" + std::to_string(numberAppended);
        }
        else
        {
            // There was a _, if the last part of that is a number
            // then replace that number with one higher or nameClashNum,
            // or just tack on the number if it is last character.
            if (last_ == name.length() - 1)
            {
                path += "_" + std::to_string(numberAppended);
            }
            else
            {
                char* p;
                std::string after_ = name.substr(last_ + 1, name.length());
                long converted = strtol(after_.c_str(), &p, 10);
                if (*p)
                {
                    // not a number
                    path += "_" + std::to_string(numberAppended);
                }
                else
                {

                    numberAppended = nameClashNum == -1 ? converted + 1 : nameClashNum;
                    indexOfNumber = path.length() - name.length() + last_ + 1;
                    path = path.substr(0, indexOfNumber);
                    path += std::to_string(numberAppended);
                }
            }
        }
    }
    if (appendedNumber)
    {
        // we just added a number, so we have to make sure the new path is unique.
        while (stage->GetPrimAtPath(pxr::SdfPath(path)))
        {
            path = path.substr(0, indexOfNumber);
            numberAppended += 1;
            path += std::to_string(numberAppended);
        }
    }
#if 0
	else
	{
		while (stage->GetPrimAtPath(pxr::SdfPath(path))) path += ":" + std::to_string(nameClashNum);
	}
#endif
    return path;
}

inline std::string makeValidUSDIdentifier(const std::string& name)
{
    auto validName = pxr::TfMakeValidIdentifier(name);
    if (validName[0] == '_')
    {
        validName = "a" + validName;
    }
    if (pxr::TfIsValidIdentifier(name) == false)
    {
        CARB_LOG_WARN("The path %s is not a valid usd path, modifying to %s", name.c_str(), validName.c_str());
    }

    return validName;
}

} // namespace usd
} // namespace utils
} // namespace core
} // namespace includes
} // namespace isaacsim
