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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on


#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/nodes/ICoreNodes.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacReadCameraInfoDatabase.h>


namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacReadCameraInfo
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacReadCameraInfoDatabase::sPerInstanceState<OgnIsaacReadCameraInfo>(nodeObj, instanceId);
        state.m_stage = omni::usd::UsdContext::getContext()->getStage();
    }

    static bool compute(OgnIsaacReadCameraInfoDatabase& db)
    {
        auto& state = db.perInstanceState<OgnIsaacReadCameraInfo>();
        const std::string renderProductPath = std::string(db.tokenToString(db.inputs.renderProductPath()));
        if (renderProductPath.length() == 0)
        {
            CARB_LOG_WARN("Render product path is empty, skipping read camera info");
            return false;
        }
        pxr::UsdPrim camera = isaacsim::core::includes::getCameraPrimFromRenderProduct(renderProductPath);

        if (!camera.IsValid())
        {
            CARB_LOG_ERROR("Render product path %s is invalid or outdated", renderProductPath.c_str());
            return false;
        }

        pxr::UsdPrim renderProduct = state.m_stage->GetPrimAtPath(pxr::SdfPath(renderProductPath));

        pxr::GfVec2i resolution;
        renderProduct.GetAttribute(pxr::TfToken("resolution")).Get(&resolution);

        auto value = resolution.GetArray();
        db.outputs.width() = value[0];
        db.outputs.height() = value[1];


        // width height
        if (db.outputs.height() == 0 || db.outputs.width() == 0)
        {
            CARB_LOG_ERROR("Camera width or height cannot be 0");
            return false;
        }


        camera.GetAttribute(pxr::TfToken("focalLength")).Get(&db.outputs.focalLength());
        camera.GetAttribute(pxr::TfToken("horizontalAperture")).Get(&db.outputs.horizontalAperture());

        float verticalAperture;
        camera.GetAttribute(pxr::TfToken("horizontalAperture")).Get(&verticalAperture);
        db.outputs.verticalAperture() = verticalAperture * (float(db.outputs.height()) / db.outputs.width());

        camera.GetAttribute(pxr::TfToken("horizontalApertureOffset")).Get(&db.outputs.horizontalOffset());
        camera.GetAttribute(pxr::TfToken("verticalApertureOffset")).Get(&db.outputs.verticalOffset());

        pxr::TfToken projectionType;
        camera.GetAttribute(pxr::TfToken("cameraProjectionType")).Get(&projectionType);
        if (projectionType.IsEmpty())
        {
            projectionType = pxr::TfToken("pinhole");
        }

        db.outputs.projectionType() = db.stringToToken(projectionType.GetText());


        omni::graph::core::ogn::array<float>& cameraFisheyeParams = db.outputs.cameraFisheyeParams();
        cameraFisheyeParams.resize(19);
        if (projectionType.GetString() != "pinhole")
        {
            camera.GetAttribute(pxr::TfToken("fthetaWidth")).Get(&cameraFisheyeParams[0]);
            camera.GetAttribute(pxr::TfToken("fthetaHeight")).Get(&cameraFisheyeParams[1]);
            camera.GetAttribute(pxr::TfToken("fthetaCx")).Get(&cameraFisheyeParams[2]);
            camera.GetAttribute(pxr::TfToken("fthetaCy")).Get(&cameraFisheyeParams[3]);
            camera.GetAttribute(pxr::TfToken("fthetaMaxFov")).Get(&cameraFisheyeParams[4]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyA")).Get(&cameraFisheyeParams[5]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyB")).Get(&cameraFisheyeParams[6]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyC")).Get(&cameraFisheyeParams[7]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyD")).Get(&cameraFisheyeParams[8]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyE")).Get(&cameraFisheyeParams[9]);
            camera.GetAttribute(pxr::TfToken("fthetaPolyF")).Get(&cameraFisheyeParams[10]);
            camera.GetAttribute(pxr::TfToken("p0")).Get(&cameraFisheyeParams[11]);
            camera.GetAttribute(pxr::TfToken("p1")).Get(&cameraFisheyeParams[12]);
            camera.GetAttribute(pxr::TfToken("s0")).Get(&cameraFisheyeParams[13]);
            camera.GetAttribute(pxr::TfToken("s1")).Get(&cameraFisheyeParams[14]);
            camera.GetAttribute(pxr::TfToken("s2")).Get(&cameraFisheyeParams[15]);
            camera.GetAttribute(pxr::TfToken("s3")).Get(&cameraFisheyeParams[16]);
            camera.GetAttribute(pxr::TfToken("fisheyeResolutionBudget")).Get(&cameraFisheyeParams[17]);
            camera.GetAttribute(pxr::TfToken("fisheyeFrontFaceResolutionScale")).Get(&cameraFisheyeParams[18]);
        }

        db.outputs.cameraFisheyeParams() = cameraFisheyeParams;


        std::string physicalDistortion;
        camera.GetAttribute(pxr::TfToken("physicalDistortionModel")).Get(&physicalDistortion);
        if (!physicalDistortion.empty())
        {
            db.outputs.physicalDistortionModel() = db.stringToToken(physicalDistortion.c_str());
        }

        pxr::VtArray<float> physicalDistortionCoefs;
        camera.GetAttribute(pxr::TfToken("physicalDistortionCoefficients")).Get(&physicalDistortionCoefs);


        if (!physicalDistortionCoefs.empty())
        {
            omni::graph::core::ogn::array<float>& physicalDistortionCoefficients =
                db.outputs.physicalDistortionCoefficients();
            physicalDistortionCoefficients.resize(physicalDistortionCoefs.size());
            for (size_t i = 0; i < physicalDistortionCoefficients.size(); i++)
            {
                physicalDistortionCoefficients[i] = physicalDistortionCoefs.data()[i];
            }
        }

        return true;
    }

private:
    pxr::UsdStageRefPtr m_stage = nullptr;
};
REGISTER_OGN_NODE()
}
}
}
