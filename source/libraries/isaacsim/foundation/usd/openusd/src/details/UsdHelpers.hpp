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
#include <pxr/base/gf/quatd.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace openusd
{
namespace details
{

namespace array = isaacsim::common::array;
using AttributeValue = std::variant<std::string, std::vector<std::string>, array::Array>;

int64_t getStageId(const PXR_NS::UsdStageRefPtr& stage);

PXR_NS::UsdStageRefPtr getStage(int64_t stageId, bool throwIfInvalid = false);

PXR_NS::UsdPrim getPrimAtPath(const PXR_NS::UsdStageRefPtr& stage, const std::string& path, bool throwIfInvalid = false);

PXR_NS::UsdPrim getPrimAtPath(int64_t stageId, const std::string& path, bool throwIfInvalid = false);

AttributeValue getAttributeValue(const PXR_NS::UsdAttribute& attribute);

bool setAttributeValue(PXR_NS::UsdAttribute& attribute, const AttributeValue& value);

PXR_NS::UsdGeomXformable getXformableAtPath(int64_t stageId,
                                            const std::string& path,
                                            bool throwIfInvalid = true,
                                            bool throwIfXformOpOrderInvalid = true);

PXR_NS::GfVec3d getXformLocalScale(const PXR_NS::UsdGeomXformable& xformable);
void setXformLocalScale(PXR_NS::UsdGeomXformable xformable, const PXR_NS::GfVec3d& scale);

std::pair<PXR_NS::GfVec3d, PXR_NS::GfQuatd> getXformLocalPose(const PXR_NS::UsdGeomXformable& xformable);
void setXformLocalPose(PXR_NS::UsdGeomXformable xformable,
                       const std::optional<PXR_NS::GfVec3d>& translation,
                       const std::optional<PXR_NS::GfQuatd>& orientation);

std::pair<PXR_NS::GfVec3d, PXR_NS::GfQuatd> getXformWorldPose(const PXR_NS::UsdGeomXformable& xformable);
void setXformWorldPose(PXR_NS::UsdGeomXformable xformable,
                       const std::optional<PXR_NS::GfVec3d>& position,
                       const std::optional<PXR_NS::GfQuatd>& orientation);

} // namespace details
} // namespace openusd
} // namespace usd
} // namespace foundation
} // namespace isaacsim
