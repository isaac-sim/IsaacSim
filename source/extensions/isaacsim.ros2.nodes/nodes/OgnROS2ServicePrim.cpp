// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/ros2/core/Ros2Node.h>
#include <nlohmann/json.hpp>
#include <omni/fabric/FabricUSD.h>

#include <OgnROS2ServicePrimDatabase.h>

using namespace isaacsim::ros2::core;

enum class SdfDataType
{
    eAsset, // SdfAssetPath
    eAssetArray, //
    eBool, // bool
    eBoolArray, //
    eColor3d, // GfVec3d
    eColor3dArray, //
    eColor3f, // GfVec3f
    eColor3fArray, //
    eColor3h, // GfVec3h
    eColor3hArray, //
    eColor4d, // GfVec4d
    eColor4dArray, //
    eColor4f, // GfVec4f
    eColor4fArray, //
    eColor4h, // GfVec4h
    eColor4hArray, //
    eDouble, // double
    eDouble2, // GfVec2d
    eDouble2Array, //
    eDouble3, // GfVec3d
    eDouble3Array, //
    eDouble4, // GfVec4d
    eDouble4Array, //
    eDoubleArray, //
    eFloat, // float
    eFloat2, // GfVec2f
    eFloat2Array, //
    eFloat3, // GfVec3f
    eFloat3Array, //
    eFloat4, // GfVec4f
    eFloat4Array, //
    eFloatArray, //
    eFrame4d, // GfMatrix4d
    eFrame4dArray, //
    eHalf, // GfHalf
    eHalf2, // GfVec2h
    eHalf2Array, //
    eHalf3, // GfVec3h
    eHalf3Array, //
    eHalf4, // GfVec4h
    eHalf4Array, //
    eHalfArray, //
    eInt, // int, int32_t
    eInt2, // GfVec2i
    eInt2Array, //
    eInt3, // GfVec3i
    eInt3Array, //
    eInt4, // GfVec4i
    eInt4Array, //
    eInt64, // int64_t
    eInt64Array, //
    eIntArray, //
    eMatrix2d, // GfMatrix2d
    eMatrix2dArray, //
    eMatrix3d, // GfMatrix3d
    eMatrix3dArray, //
    eMatrix4d, // GfMatrix4d
    eMatrix4dArray, //
    eNormal3d, // GfVec3d
    eNormal3dArray, //
    eNormal3f, // GfVec3f
    eNormal3fArray, //
    eNormal3h, // GfVec3h
    eNormal3hArray, //
    ePoint3d, // GfVec3d
    ePoint3dArray, //
    ePoint3f, // GfVec3f
    ePoint3fArray, //
    ePoint3h, // GfVec3h
    ePoint3hArray, //
    eQuatd, // GfQuatd
    eQuatdArray, //
    eQuatf, // GfQuatf
    eQuatfArray, //
    eQuath, // GfQuath
    eQuathArray, //
    eString, // string
    eStringArray, //
    eTexCoord2d, // GfVec2d
    eTexCoord2dArray, //
    eTexCoord2f, // GfVec2f
    eTexCoord2fArray, //
    eTexCoord2h, // GfVec2h
    eTexCoord2hArray, //
    eTexCoord3d, // GfVec3d
    eTexCoord3dArray, //
    eTexCoord3f, // GfVec3f
    eTexCoord3fArray, //
    eTexCoord3h, // GfVec3h
    eTexCoord3hArray, //
    eTimeCode, // SdfTimeCode
    eTimeCodeArray, //
    eToken, // TfToken
    eTokenArray, //
    eUChar, // unsigned char, uint8_t
    eUCharArray, //
    eUInt, // unsigned int, uint32_t
    eUInt64, // unsigned long, uint64_t
    eUInt64Array, //
    eUIntArray, //
    eVector3d, // GfVec3d
    eVector3dArray, //
    eVector3f, // GfVec3f
    eVector3fArray, //
    eVector3h, // GfVec3h
    eVector3hArray, //
};


std::map<std::string, SdfDataType> g_sMapStringToSdfDataType = {
    { "asset", SdfDataType::eAsset },
    { "asset[]", SdfDataType::eAssetArray },
    { "bool", SdfDataType::eBool },
    { "bool[]", SdfDataType::eBoolArray },
    { "color3d", SdfDataType::eColor3d },
    { "color3d[]", SdfDataType::eColor3dArray },
    { "color3f", SdfDataType::eColor3f },
    { "color3f[]", SdfDataType::eColor3fArray },
    { "color3h", SdfDataType::eColor3h },
    { "color3h[]", SdfDataType::eColor3hArray },
    { "color4d", SdfDataType::eColor4d },
    { "color4d[]", SdfDataType::eColor4dArray },
    { "color4f", SdfDataType::eColor4f },
    { "color4f[]", SdfDataType::eColor4fArray },
    { "color4h", SdfDataType::eColor4h },
    { "color4h[]", SdfDataType::eColor4hArray },
    { "double", SdfDataType::eDouble },
    { "double2", SdfDataType::eDouble2 },
    { "double2[]", SdfDataType::eDouble2Array },
    { "double3", SdfDataType::eDouble3 },
    { "double3[]", SdfDataType::eDouble3Array },
    { "double4", SdfDataType::eDouble4 },
    { "double4[]", SdfDataType::eDouble4Array },
    { "double[]", SdfDataType::eDoubleArray },
    { "float", SdfDataType::eFloat },
    { "float2", SdfDataType::eFloat2 },
    { "float2[]", SdfDataType::eFloat2Array },
    { "float3", SdfDataType::eFloat3 },
    { "float3[]", SdfDataType::eFloat3Array },
    { "float4", SdfDataType::eFloat4 },
    { "float4[]", SdfDataType::eFloat4Array },
    { "float[]", SdfDataType::eFloatArray },
    { "frame4d", SdfDataType::eFrame4d },
    { "frame4d[]", SdfDataType::eFrame4dArray },
    { "half", SdfDataType::eHalf },
    { "half2", SdfDataType::eHalf2 },
    { "half2[]", SdfDataType::eHalf2Array },
    { "half3", SdfDataType::eHalf3 },
    { "half3[]", SdfDataType::eHalf3Array },
    { "half4", SdfDataType::eHalf4 },
    { "half4[]", SdfDataType::eHalf4Array },
    { "half[]", SdfDataType::eHalfArray },
    { "int", SdfDataType::eInt },
    { "int2", SdfDataType::eInt2 },
    { "int2[]", SdfDataType::eInt2Array },
    { "int3", SdfDataType::eInt3 },
    { "int3[]", SdfDataType::eInt3Array },
    { "int4", SdfDataType::eInt4 },
    { "int4[]", SdfDataType::eInt4Array },
    { "int64", SdfDataType::eInt64 },
    { "int64[]", SdfDataType::eInt64Array },
    { "int[]", SdfDataType::eIntArray },
    { "matrix2d", SdfDataType::eMatrix2d },
    { "matrix2d[]", SdfDataType::eMatrix2dArray },
    { "matrix3d", SdfDataType::eMatrix3d },
    { "matrix3d[]", SdfDataType::eMatrix3dArray },
    { "matrix4d", SdfDataType::eMatrix4d },
    { "matrix4d[]", SdfDataType::eMatrix4dArray },
    { "normal3d", SdfDataType::eNormal3d },
    { "normal3d[]", SdfDataType::eNormal3dArray },
    { "normal3f", SdfDataType::eNormal3f },
    { "normal3f[]", SdfDataType::eNormal3fArray },
    { "normal3h", SdfDataType::eNormal3h },
    { "normal3h[]", SdfDataType::eNormal3hArray },
    { "point3d", SdfDataType::ePoint3d },
    { "point3d[]", SdfDataType::ePoint3dArray },
    { "point3f", SdfDataType::ePoint3f },
    { "point3f[]", SdfDataType::ePoint3fArray },
    { "point3h", SdfDataType::ePoint3h },
    { "point3h[]", SdfDataType::ePoint3hArray },
    { "quatd", SdfDataType::eQuatd },
    { "quatd[]", SdfDataType::eQuatdArray },
    { "quatf", SdfDataType::eQuatf },
    { "quatf[]", SdfDataType::eQuatfArray },
    { "quath", SdfDataType::eQuath },
    { "quath[]", SdfDataType::eQuathArray },
    { "string", SdfDataType::eString },
    { "string[]", SdfDataType::eStringArray },
    { "texCoord2d", SdfDataType::eTexCoord2d },
    { "texCoord2d[]", SdfDataType::eTexCoord2dArray },
    { "texCoord2f", SdfDataType::eTexCoord2f },
    { "texCoord2f[]", SdfDataType::eTexCoord2fArray },
    { "texCoord2h", SdfDataType::eTexCoord2h },
    { "texCoord2h[]", SdfDataType::eTexCoord2hArray },
    { "texCoord3d", SdfDataType::eTexCoord3d },
    { "texCoord3d[]", SdfDataType::eTexCoord3dArray },
    { "texCoord3f", SdfDataType::eTexCoord3f },
    { "texCoord3f[]", SdfDataType::eTexCoord3fArray },
    { "texCoord3h", SdfDataType::eTexCoord3h },
    { "texCoord3h[]", SdfDataType::eTexCoord3hArray },
    { "timecode", SdfDataType::eTimeCode },
    { "timecode[]", SdfDataType::eTimeCodeArray },
    { "token", SdfDataType::eToken },
    { "token[]", SdfDataType::eTokenArray },
    { "uchar", SdfDataType::eUChar },
    { "uchar[]", SdfDataType::eUCharArray },
    { "uint", SdfDataType::eUInt },
    { "uint64", SdfDataType::eUInt64 },
    { "uint64[]", SdfDataType::eUInt64Array },
    { "uint[]", SdfDataType::eUIntArray },
    { "vector3d", SdfDataType::eVector3d },
    { "vector3d[]", SdfDataType::eVector3dArray },
    { "vector3f", SdfDataType::eVector3f },
    { "vector3f[]", SdfDataType::eVector3fArray },
    { "vector3h", SdfDataType::eVector3h },
    { "vector3h[]", SdfDataType::eVector3hArray },
};


std::vector<std::string> g_sVecStringArrayType = {
    "asset[]",      "bool[]",       "color3d[]",    "color3f[]",  "color3h[]",    "color4d[]",    "color4f[]",
    "color4h[]",    "double2[]",    "double3[]",    "double4[]",  "double[]",     "float2[]",     "float3[]",
    "float4[]",     "float[]",      "frame4d[]",    "half2[]",    "half3[]",      "half4[]",      "half[]",
    "int2[]",       "int3[]",       "int4[]",       "int64[]",    "int[]",        "matrix2d[]",   "matrix3d[]",
    "matrix4d[]",   "normal3d[]",   "normal3f[]",   "normal3h[]", "point3d[]",    "point3f[]",    "point3h[]",
    "quatd[]",      "quatf[]",      "quath[]",      "string[]",   "texCoord2d[]", "texCoord2f[]", "texCoord2h[]",
    "texCoord3d[]", "texCoord3f[]", "texCoord3h[]", "timecode[]", "token[]",      "uchar[]",      "uint64[]",
    "uint[]",       "vector3d[]",   "vector3f[]",   "vector3h[]",
};


class OgnROS2ServicePrim : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2ServicePrimDatabase::sPerInstanceState<OgnROS2ServicePrim>(nodeObj, instanceId);
        state.m_nodeObj = nodeObj;
    }

    static bool compute(OgnROS2ServicePrimDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2ServicePrim>();

        // Spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
            const GraphContextObj& context = db.abi_context();
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!state.initializeNodeHandle(
                    std::string(nodeObj.iNode->getPrimPath(nodeObj)),
                    collectNamespace(db.inputs.nodeNamespace(),
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)))),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Create messages (isaac_ros2_messages/srv/GetPrims)
        std::string messageType = "isaac_ros2_messages/srv/GetPrims";
        if (!state.m_messageGetPrimsRequest)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create request message for %s", messageType.c_str());
            state.m_messageGetPrimsRequest = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrims", BackendMessageType::eRequest);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetPrimsRequest)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetPrimsRequest.reset();
                return false;
            }
        }
        if (!state.m_messageGetPrimsResponse)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create response message for %s", messageType.c_str());
            state.m_messageGetPrimsResponse = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrims", BackendMessageType::eResponse);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetPrimsResponse)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetPrimsResponse.reset();
                return false;
            }
        }
        // Create messages (isaac_ros2_messages/srv/GetPrimAttributes)
        messageType = "isaac_ros2_messages/srv/GetPrimAttributes";
        if (!state.m_messageGetAttributesRequest)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create request message for %s", messageType.c_str());
            state.m_messageGetAttributesRequest = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrimAttributes", BackendMessageType::eRequest);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributesRequest)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetAttributesRequest.reset();
                return false;
            }
        }
        if (!state.m_messageGetAttributesResponse)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create response message for %s", messageType.c_str());
            state.m_messageGetAttributesResponse = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrimAttributes", BackendMessageType::eResponse);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributesResponse)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetAttributesResponse.reset();
                return false;
            }
        }
        // Create messages (isaac_ros2_messages/srv/GetPrimAttribute)
        messageType = "isaac_ros2_messages/srv/GetPrimAttribute";
        if (!state.m_messageGetAttributeRequest)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create request message for %s", messageType.c_str());
            state.m_messageGetAttributeRequest = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrimAttribute", BackendMessageType::eRequest);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributeRequest)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetAttributeRequest.reset();
                return false;
            }
        }
        if (!state.m_messageGetAttributeResponse)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create response message for %s", messageType.c_str());
            state.m_messageGetAttributeResponse = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "GetPrimAttribute", BackendMessageType::eResponse);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributeResponse)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageGetAttributeResponse.reset();
                return false;
            }
        }
        // Create messages (isaac_ros2_messages/srv/SetPrimAttribute)
        messageType = "isaac_ros2_messages/srv/SetPrimAttribute";
        if (!state.m_messageSetAttributeRequest)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create request message for %s", messageType.c_str());
            state.m_messageSetAttributeRequest = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "SetPrimAttribute", BackendMessageType::eRequest);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageSetAttributeRequest)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageSetAttributeRequest.reset();
                return false;
            }
        }
        if (!state.m_messageSetAttributeResponse)
        {
            CARB_LOG_INFO("OgnROS2ServicePrim: create response message for %s", messageType.c_str());
            state.m_messageSetAttributeResponse = state.m_factory->createDynamicMessage(
                "isaac_ros2_messages", "srv", "SetPrimAttribute", BackendMessageType::eResponse);
            bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageSetAttributeResponse)->isValid();
            if (!status)
            {
                db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
                state.m_messageSetAttributeResponse.reset();
                return false;
            }
        }

        // Check for changes in service names
        std::string primsServiceName = std::string(db.inputs.primsServiceName());
        std::string getAttributesServiceName = std::string(db.inputs.getAttributesServiceName());
        std::string getAttributeServiceName = std::string(db.inputs.getAttributeServiceName());
        std::string setAttributeServiceName = std::string(db.inputs.setAttributeServiceName());
        if (primsServiceName != state.m_getPrimsServiceName)
        {
            state.m_serviceGetPrimsUpdateNeeded = true;
            state.m_getPrimsServiceName = primsServiceName;
        }
        if (getAttributesServiceName != state.m_getAttributesServiceName)
        {
            state.m_serviceGetAttributesUpdateNeeded = true;
            state.m_getAttributesServiceName = getAttributesServiceName;
        }
        if (getAttributeServiceName != state.m_getAttributeServiceName)
        {
            state.m_serviceGetAttributeUpdateNeeded = true;
            state.m_getAttributeServiceName = getAttributeServiceName;
        }
        if (setAttributeServiceName != state.m_setAttributeServiceName)
        {
            state.m_serviceSetAttributeUpdateNeeded = true;
            state.m_setAttributeServiceName = setAttributeServiceName;
        }
        std::string qosProfile = std::string(db.inputs.qosProfile());
        if (qosProfile != state.m_qosProfile)
        {
            state.m_serviceGetPrimsUpdateNeeded = true;
            state.m_serviceGetAttributesUpdateNeeded = true;
            state.m_serviceGetAttributeUpdateNeeded = true;
            state.m_serviceSetAttributeUpdateNeeded = true;
            state.m_qosProfile = qosProfile;
        }

        // Update services
        if (state.m_serviceGetPrimsUpdateNeeded)
        {
            // Destroy previous subscriber
            state.m_serviceGetPrims.reset();
            // Get service name
            std::string fullServiceName = addTopicPrefix(state.m_namespaceName, state.m_getPrimsServiceName);
            if (!state.m_factory->validateTopicName(fullServiceName))
            {
                return false;
            }
            // Create service
            CARB_LOG_INFO("OgnROS2ServicePrim: creating service: %s", fullServiceName.c_str());
            Ros2QoSProfile qos;
            if (qosProfile != "")
            {
                if (!jsonToRos2QoSProfile(qos, state.m_qosProfile))
                {
                    return false;
                }
            }
            state.m_serviceGetPrims =
                state.m_factory->createService(state.m_nodeHandle.get(), fullServiceName.c_str(),
                                               state.m_messageGetPrimsRequest->getTypeSupportHandle(), qos);
            CARB_LOG_INFO("OgnROS2ServicePrim: service: %s", fullServiceName.c_str());
            if (!state.m_serviceGetPrims->isValid())
            {
                db.logWarning(("Invalid service for " + fullServiceName).c_str());
                state.m_serviceGetPrims.reset();
                return false;
            }
            state.m_serviceGetPrimsUpdateNeeded = false;
            return true;
        }
        if (state.m_serviceGetAttributesUpdateNeeded)
        {
            // Destroy previous subscriber
            state.m_serviceGetAttributes.reset();
            // Get service name
            std::string fullServiceName = addTopicPrefix(state.m_namespaceName, state.m_getAttributesServiceName);
            if (!state.m_factory->validateTopicName(fullServiceName))
            {
                return false;
            }
            // Create service
            CARB_LOG_INFO("OgnROS2ServicePrim: creating service: %s", fullServiceName.c_str());
            Ros2QoSProfile qos;
            if (qosProfile != "")
            {
                if (!jsonToRos2QoSProfile(qos, state.m_qosProfile))
                {
                    return false;
                }
            }
            state.m_serviceGetAttributes =
                state.m_factory->createService(state.m_nodeHandle.get(), fullServiceName.c_str(),
                                               state.m_messageGetAttributesRequest->getTypeSupportHandle(), qos);
            CARB_LOG_INFO("OgnROS2ServicePrim: service: %s", fullServiceName.c_str());
            if (!state.m_serviceGetAttributes->isValid())
            {
                db.logWarning(("Invalid service for " + fullServiceName).c_str());
                state.m_serviceGetAttributes.reset();
                return false;
            }
            state.m_serviceGetAttributesUpdateNeeded = false;
            return true;
        }
        if (state.m_serviceGetAttributeUpdateNeeded)
        {
            // Destroy previous subscriber
            state.m_serviceGetAttribute.reset();
            // Get service name
            std::string fullServiceName = addTopicPrefix(state.m_namespaceName, state.m_getAttributeServiceName);
            if (!state.m_factory->validateTopicName(fullServiceName))
            {
                return false;
            }
            // Create service
            CARB_LOG_INFO("OgnROS2ServicePrim: creating service: %s", fullServiceName.c_str());

            Ros2QoSProfile qos;
            if (qosProfile != "")
            {
                if (!jsonToRos2QoSProfile(qos, state.m_qosProfile))
                {
                    return false;
                }
            }
            state.m_serviceGetAttribute =
                state.m_factory->createService(state.m_nodeHandle.get(), fullServiceName.c_str(),
                                               state.m_messageGetAttributeRequest->getTypeSupportHandle(), qos);
            CARB_LOG_INFO("OgnROS2ServicePrim: service: %s", fullServiceName.c_str());
            if (!state.m_serviceGetAttribute->isValid())
            {
                db.logWarning(("Invalid service for " + fullServiceName).c_str());
                state.m_serviceGetAttribute.reset();
                return false;
            }
            state.m_serviceGetAttributeUpdateNeeded = false;
            return true;
        }
        if (state.m_serviceSetAttributeUpdateNeeded)
        {
            // Destroy previous subscriber
            state.m_serviceSetAttribute.reset();
            // Get service name
            std::string fullServiceName = addTopicPrefix(state.m_namespaceName, state.m_setAttributeServiceName);
            if (!state.m_factory->validateTopicName(fullServiceName))
            {
                return false;
            }
            // Create service
            CARB_LOG_INFO("OgnROS2ServicePrim: creating service: %s", fullServiceName.c_str());
            Ros2QoSProfile qos;
            if (qosProfile != "")
            {
                if (!jsonToRos2QoSProfile(qos, state.m_qosProfile))
                {
                    return false;
                }
            }
            state.m_serviceSetAttribute =
                state.m_factory->createService(state.m_nodeHandle.get(), fullServiceName.c_str(),
                                               state.m_messageSetAttributeRequest->getTypeSupportHandle(), qos);
            CARB_LOG_INFO("OgnROS2ServicePrim: service: %s", fullServiceName.c_str());
            if (!state.m_serviceSetAttribute->isValid())
            {
                db.logWarning(("Invalid service for " + fullServiceName).c_str());
                state.m_serviceSetAttribute.reset();
                return false;
            }
            state.m_serviceSetAttributeUpdateNeeded = false;
            return true;
        }

        state.serviceGetPrimsCallback(db);
        state.serviceGetAttributesCallback(db);
        state.serviceGetAttributeCallback(db);
        state.serviceSetAttributeCallback(db);
        return true;
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2ServicePrimDatabase::sPerInstanceState<OgnROS2ServicePrim>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        m_getPrimsServiceName.clear();
        m_getAttributesServiceName.clear();
        m_getAttributeServiceName.clear();
        m_setAttributeServiceName.clear();

        m_serviceGetPrims.reset(); // This should be reset before reset the handle
        m_serviceGetAttributes.reset(); // This should be reset before reset the handle
        m_serviceGetAttribute.reset(); // This should be reset before reset the handle
        m_serviceSetAttribute.reset(); // This should be reset before reset the handle
        Ros2Node::reset();
    }

private:
    NodeObj m_nodeObj;
    bool m_serviceGetPrimsUpdateNeeded = false;
    bool m_serviceGetAttributesUpdateNeeded = false;
    bool m_serviceGetAttributeUpdateNeeded = false;
    bool m_serviceSetAttributeUpdateNeeded = false;

    std::shared_ptr<Ros2Service> m_serviceGetPrims = nullptr;
    std::shared_ptr<Ros2Service> m_serviceGetAttributes = nullptr;
    std::shared_ptr<Ros2Service> m_serviceGetAttribute = nullptr;
    std::shared_ptr<Ros2Service> m_serviceSetAttribute = nullptr;

    std::shared_ptr<Ros2Message> m_messageGetPrimsRequest = nullptr;
    std::shared_ptr<Ros2Message> m_messageGetPrimsResponse = nullptr;
    std::shared_ptr<Ros2Message> m_messageGetAttributesRequest = nullptr;
    std::shared_ptr<Ros2Message> m_messageGetAttributesResponse = nullptr;
    std::shared_ptr<Ros2Message> m_messageGetAttributeRequest = nullptr;
    std::shared_ptr<Ros2Message> m_messageGetAttributeResponse = nullptr;
    std::shared_ptr<Ros2Message> m_messageSetAttributeRequest = nullptr;
    std::shared_ptr<Ros2Message> m_messageSetAttributeResponse = nullptr;

    std::string m_getPrimsServiceName;
    std::string m_getAttributesServiceName;
    std::string m_getAttributeServiceName;
    std::string m_setAttributeServiceName;
    std::string m_qosProfile;

    // Node

    bool serviceGetPrimsCallback(OgnROS2ServicePrimDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2ServicePrim>();
        if (!state.m_serviceGetPrims)
        {
            return false;
        }
        if (!state.m_serviceGetPrims->takeRequest(state.m_messageGetPrimsRequest->getPtr()))
        {
            return false;
        }

        auto requestData = std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetPrimsRequest)->readData(false);
        auto messageFields =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetPrimsRequest)->getMessageFields();
        CARB_ASSERT(messageFields.size() == requestData.size());

        std::string path = *std::static_pointer_cast<std::string>(requestData.at(0));
        CARB_LOG_INFO("OgnROS2ServicePrim: get prims: %s", path.c_str());

        std::vector<std::shared_ptr<void>> responseData;
        std::vector<std::string> paths;
        std::vector<std::string> types;
        bool success = false;
        std::string message;

        // Get all prim path under the specified path
        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (stage)
        {
            const pxr::UsdPrim targetPrim = stage->GetPrimAtPath(pxr::SdfPath(path));
            if (targetPrim.IsValid())
            {
                for (auto prim : pxr::UsdPrimRange(targetPrim))
                {
                    paths.push_back(prim.GetPath().GetAsString());
                    types.push_back(prim.GetTypeName().GetString());
                }
                success = true;
            }
            else
            {
                message = "Invalid prim: '" + path + "'";
            }
        }
        else
        {
            message = "Unable to query the scene";
        }

        responseData.push_back(std::make_shared<std::vector<std::string>>(paths));
        responseData.push_back(std::make_shared<std::vector<std::string>>(types));
        responseData.push_back(std::make_shared<bool>(success));
        responseData.push_back(std::make_shared<std::string>(message));

        CARB_ASSERT(messageFields.size() == responseData.size());
        std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetPrimsResponse)->writeData(responseData, false);
        state.m_serviceGetPrims->sendResponse(state.m_messageGetPrimsResponse->getPtr());

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    bool serviceGetAttributesCallback(OgnROS2ServicePrimDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2ServicePrim>();
        if (!state.m_serviceGetAttributes)
        {
            return false;
        }
        if (!state.m_serviceGetAttributes->takeRequest(state.m_messageGetAttributesRequest->getPtr()))
        {
            return false;
        }

        auto requestData =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributesRequest)->readData(false);
        auto messageFields =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributesRequest)->getMessageFields();
        CARB_ASSERT(messageFields.size() == requestData.size());

        std::string path = *std::static_pointer_cast<std::string>(requestData.at(0));
        CARB_LOG_INFO("OgnROS2ServicePrim: get attributes: %s", path.c_str());

        std::vector<std::shared_ptr<void>> responseData;
        std::vector<std::string> names;
        std::vector<std::string> displays;
        std::vector<std::string> types;
        bool success = false;
        std::string message;

        // Get all prim attribute names and their types
        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (stage)
        {
            const pxr::UsdPrim targetPrim = stage->GetPrimAtPath(pxr::SdfPath(path));
            if (targetPrim.IsValid())
            {
                for (auto attr : targetPrim.GetAttributes())
                {
                    if (attr.GetNamespace().GetString().empty())
                    {
                        names.push_back(attr.GetBaseName().GetString());
                    }
                    else
                    {
                        names.push_back(attr.GetNamespace().GetString() + ":" + attr.GetBaseName().GetString());
                    }
                    displays.push_back(attr.GetDisplayName());
                    types.push_back(attr.GetTypeName().GetAsToken().GetString());
                }
                success = true;
            }
            else
            {
                message = "Invalid prim: '" + path + "'";
            }
        }
        else
        {
            message = "Unable to query the scene";
        }

        responseData.push_back(std::make_shared<std::vector<std::string>>(names));
        responseData.push_back(std::make_shared<std::vector<std::string>>(displays));
        responseData.push_back(std::make_shared<std::vector<std::string>>(types));
        responseData.push_back(std::make_shared<bool>(success));
        responseData.push_back(std::make_shared<std::string>(message));

        CARB_ASSERT(messageFields.size() == responseData.size());
        std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributesResponse)->writeData(responseData, false);
        state.m_serviceGetAttributes->sendResponse(state.m_messageGetAttributesResponse->getPtr());

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    bool serviceGetAttributeCallback(OgnROS2ServicePrimDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2ServicePrim>();
        if (!state.m_serviceGetAttribute)
        {
            return false;
        }
        if (!state.m_serviceGetAttribute->takeRequest(state.m_messageGetAttributeRequest->getPtr()))
        {
            return false;
        }

        auto requestData =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributeRequest)->readData(false);
        auto messageFields =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributeRequest)->getMessageFields();
        CARB_ASSERT(messageFields.size() == requestData.size());

        std::string path = *std::static_pointer_cast<std::string>(requestData.at(0));
        std::string attrName = *std::static_pointer_cast<std::string>(requestData.at(1));
        CARB_LOG_INFO("OgnROS2ServicePrim: get attribute value: %s (%s)", path.c_str(), attrName.c_str());

        std::vector<std::shared_ptr<void>> responseData;
        std::string value;
        std::string type;
        bool success = false;
        std::string message;

        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (stage)
        {
            const pxr::UsdPrim targetPrim = stage->GetPrimAtPath(pxr::SdfPath(path));
            if (targetPrim.IsValid())
            {
                if (targetPrim.HasAttribute(pxr::TfToken(attrName.c_str())))
                {
                    // Get prim attribute
                    auto attr = targetPrim.GetAttribute(pxr::TfToken(attrName.c_str()));
                    auto jsonObj = valueTypeToJson(attr); // pxr::TfStringify(vtValue);
                    // Build message
                    value = jsonObj.dump();
                    type = attr.GetTypeName().GetAsToken().GetString();
                    success = (jsonObj.is_array() || jsonObj.is_object()) || !jsonObj.empty();
                    if (!success)
                    {
                        message = "Unable to serialize the attribute";
                    }
                    CARB_LOG_INFO("OgnROS2ServicePrim: |-- %s (type name: %s)", value.c_str(), type.c_str());
                }
                else
                {
                    message = "Prim has not attribute: '" + attrName + "'";
                }
            }
            else
            {
                message = "Invalid prim: '" + path + "'";
            }
        }
        else
        {
            message = "Unable to query the scene";
        }

        if (!success)
        {
            db.logWarning(message.c_str());
        }

        responseData.push_back(std::make_shared<std::string>(value));
        responseData.push_back(std::make_shared<std::string>(type));
        responseData.push_back(std::make_shared<bool>(success));
        responseData.push_back(std::make_shared<std::string>(message));

        CARB_ASSERT(messageFields.size() == responseData.size());
        std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageGetAttributeResponse)->writeData(responseData, false);
        state.m_serviceGetAttribute->sendResponse(state.m_messageGetAttributeResponse->getPtr());

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    bool serviceSetAttributeCallback(OgnROS2ServicePrimDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2ServicePrim>();
        if (!state.m_serviceSetAttribute)
        {
            return false;
        }
        if (!state.m_serviceSetAttribute->takeRequest(state.m_messageSetAttributeRequest->getPtr()))
        {
            return false;
        }

        auto requestData =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageSetAttributeRequest)->readData(false);
        auto messageFields =
            std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageSetAttributeRequest)->getMessageFields();
        CARB_ASSERT(messageFields.size() == requestData.size());

        std::string path = *std::static_pointer_cast<std::string>(requestData.at(0));
        std::string attrName = *std::static_pointer_cast<std::string>(requestData.at(1));
        std::string attrValueAsString = *std::static_pointer_cast<std::string>(requestData.at(2));
        CARB_LOG_INFO("OgnROS2ServicePrim: set attribute value: %s (%s)", path.c_str(), attrName.c_str());

        std::vector<std::shared_ptr<void>> responseData;
        bool success = false;
        std::string message;

        // Set prim attribute
        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (stage)
        {
            const pxr::UsdPrim targetPrim = stage->GetPrimAtPath(pxr::SdfPath(path));
            if (targetPrim.IsValid())
            {
                if (targetPrim.HasAttribute(pxr::TfToken(attrName.c_str())))
                {
                    auto attr = targetPrim.GetAttribute(pxr::TfToken(attrName.c_str()));
                    if (nlohmann::json::accept(attrValueAsString))
                    {
                        nlohmann::json jsonObj = nlohmann::json::parse(attrValueAsString);
                        CARB_LOG_INFO("OgnROS2ServicePrim: |-- %s (type name: %s)", attrValueAsString.c_str(),
                                      attr.GetTypeName().GetAsToken().GetString().c_str());
                        auto vtValue = valueTypeFromJson(attr, jsonObj);
                        success = !vtValue.IsEmpty();
                        if (success)
                        {
                            attr.Set(vtValue); // pxr::TfUnstringify(attrValueAsString);
                        }
                        else
                        {
                            message = "Unable to deserialize the attribute";
                        }
                    }
                    else
                    {
                        message = "Given value is not a valid JSON: '" + attrValueAsString + "'";
                    }
                }
                else
                {
                    message = "Prim has not attribute: '" + attrName + "'";
                }
            }
            else
            {
                message = "Invalid prim: '" + path + "'";
            }
        }
        else
        {
            message = "Unable to query the scene";
        }

        if (!success)
        {
            db.logWarning(message.c_str());
        }

        responseData.push_back(std::make_shared<bool>(success));
        responseData.push_back(std::make_shared<std::string>(message));

        CARB_ASSERT(messageFields.size() == responseData.size());
        std::static_pointer_cast<Ros2DynamicMessage>(state.m_messageSetAttributeResponse)->writeData(responseData, false);
        state.m_serviceSetAttribute->sendResponse(state.m_messageSetAttributeResponse->getPtr());

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    pxr::VtValue valueTypeFromJson(const pxr::UsdAttribute& attr, const nlohmann::json& jsonObj)
    {
        return valueTypeFromJson(jsonObj, attr.GetTypeName().GetAsToken());
    }

    pxr::VtValue valueTypeFromJson(const nlohmann::json& jsonObj, const pxr::TfToken& typeName)
    {
        auto name = typeName.GetString();
        if (g_sMapStringToSdfDataType.find(name) != g_sMapStringToSdfDataType.end())
        {
            return valueTypeFromJson(jsonObj, g_sMapStringToSdfDataType[name]);
        }
        return pxr::VtValue();
    }

    pxr::VtValue valueTypeFromJson(const nlohmann::json& jsonObj, SdfDataType type)
    {
        bool status = true;
        pxr::VtValue vtValue;
        switch (type)
        {
        case SdfDataType::eAsset:
        {
            if (jsonObj.is_string())
            {
                vtValue = pxr::VtValue(pxr::SdfAssetPath(jsonObj.get<std::string>()));
            }
            break;
        }
        case SdfDataType::eAssetArray:
        {
            vtValue = arrayTypeFromJson<pxr::SdfAssetPath>(jsonObj, SdfDataType::eAsset, &status);
            break;
        }
        case SdfDataType::eBool:
        {
            if (jsonObj.is_boolean())
            {
                vtValue = pxr::VtValue(jsonObj.get<bool>());
            }
            else if (jsonObj.is_number())
            {
                vtValue = pxr::VtValue(static_cast<bool>(jsonObj.get<double>()));
            }
            break;
        }
        case SdfDataType::eBoolArray:
        {
            vtValue = arrayTypeFromJson<bool>(jsonObj, SdfDataType::eBool, &status);
            break;
        }
        case SdfDataType::eColor3d:
        case SdfDataType::eDouble3:
        case SdfDataType::eNormal3d:
        case SdfDataType::ePoint3d:
        case SdfDataType::eTexCoord3d:
        case SdfDataType::eVector3d:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eDouble, 3))
            {
                vtValue = pxr::VtValue(
                    pxr::GfVec3d(jsonObj[0].get<double>(), jsonObj[1].get<double>(), jsonObj[2].get<double>()));
            }
            break;
        }
        case SdfDataType::eColor3dArray:
        case SdfDataType::eDouble3Array:
        case SdfDataType::eNormal3dArray:
        case SdfDataType::ePoint3dArray:
        case SdfDataType::eTexCoord3dArray:
        case SdfDataType::eVector3dArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec3d>(jsonObj, SdfDataType::eDouble3, &status);
            break;
        }
        case SdfDataType::eColor3f:
        case SdfDataType::eFloat3:
        case SdfDataType::eNormal3f:
        case SdfDataType::ePoint3f:
        case SdfDataType::eTexCoord3f:
        case SdfDataType::eVector3f:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eFloat, 3))
            {
                vtValue =
                    pxr::VtValue(pxr::GfVec3f(jsonObj[0].get<float>(), jsonObj[1].get<float>(), jsonObj[2].get<float>()));
            }
            break;
        }
        case SdfDataType::eColor3fArray:
        case SdfDataType::eFloat3Array:
        case SdfDataType::eNormal3fArray:
        case SdfDataType::ePoint3fArray:
        case SdfDataType::eTexCoord3fArray:
        case SdfDataType::eVector3fArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec3f>(jsonObj, SdfDataType::eFloat3, &status);
            break;
        }
        case SdfDataType::eColor3h:
        case SdfDataType::eHalf3:
        case SdfDataType::eNormal3h:
        case SdfDataType::ePoint3h:
        case SdfDataType::eTexCoord3h:
        case SdfDataType::eVector3h:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eHalf, 3))
            {
                vtValue = pxr::VtValue(pxr::GfVec3h(
                    pxr::GfVec3f(jsonObj[0].get<float>(), jsonObj[1].get<float>(), jsonObj[2].get<float>())));
            }
            break;
        }
        case SdfDataType::eColor3hArray:
        case SdfDataType::eHalf3Array:
        case SdfDataType::eNormal3hArray:
        case SdfDataType::ePoint3hArray:
        case SdfDataType::eTexCoord3hArray:
        case SdfDataType::eVector3hArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec3h>(jsonObj, SdfDataType::eHalf3, &status);
            break;
        }
        case SdfDataType::eColor4d:
        case SdfDataType::eDouble4:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eDouble, 4))
            {
                vtValue = pxr::VtValue(pxr::GfVec4d(jsonObj[0].get<double>(), jsonObj[1].get<double>(),
                                                    jsonObj[2].get<double>(), jsonObj[3].get<double>()));
            }
            break;
        }
        case SdfDataType::eColor4dArray:
        case SdfDataType::eDouble4Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec4d>(jsonObj, SdfDataType::eDouble4, &status);
            break;
        }
        case SdfDataType::eColor4f:
        case SdfDataType::eFloat4:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eFloat, 4))
            {
                vtValue = pxr::VtValue(pxr::GfVec4f(jsonObj[0].get<float>(), jsonObj[1].get<float>(),
                                                    jsonObj[2].get<float>(), jsonObj[3].get<float>()));
            }
            break;
        }
        case SdfDataType::eColor4fArray:
        case SdfDataType::eFloat4Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec4f>(jsonObj, SdfDataType::eFloat4, &status);
            break;
        }
        case SdfDataType::eColor4h:
        case SdfDataType::eHalf4:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eHalf, 4))
            {
                vtValue = pxr::VtValue(pxr::GfVec4h(pxr::GfVec4f(jsonObj[0].get<float>(), jsonObj[1].get<float>(),
                                                                 jsonObj[2].get<float>(), jsonObj[3].get<float>())));
            }
            break;
        }
        case SdfDataType::eColor4hArray:
        case SdfDataType::eHalf4Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec4h>(jsonObj, SdfDataType::eHalf4, &status);
            break;
        }
        case SdfDataType::eDouble:
        {
            if (jsonObj.is_number())
            {
                vtValue = pxr::VtValue(jsonObj.get<double>());
            }
            break;
        }
        case SdfDataType::eDoubleArray:
        {
            vtValue = arrayTypeFromJson<double>(jsonObj, SdfDataType::eDouble, &status);
            break;
        }
        case SdfDataType::eDouble2:
        case SdfDataType::eTexCoord2d:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eDouble, 2))
            {
                vtValue = pxr::VtValue(pxr::GfVec2d(jsonObj[0].get<double>(), jsonObj[1].get<double>()));
            }
            break;
        }
        case SdfDataType::eDouble2Array:
        case SdfDataType::eTexCoord2dArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec2d>(jsonObj, SdfDataType::eDouble2, &status);
            break;
        }
        case SdfDataType::eFloat:
        {
            if (jsonObj.is_number())
            {
                vtValue = pxr::VtValue(jsonObj.get<float>());
            }
            break;
        }
        case SdfDataType::eFloatArray:
        {
            vtValue = arrayTypeFromJson<float>(jsonObj, SdfDataType::eFloat, &status);
            break;
        }
        case SdfDataType::eFloat2:
        case SdfDataType::eTexCoord2f:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eFloat, 2))
            {
                vtValue = pxr::VtValue(pxr::GfVec2f(jsonObj[0].get<float>(), jsonObj[1].get<float>()));
            }
            break;
        }
        case SdfDataType::eFloat2Array:
        case SdfDataType::eTexCoord2fArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec2f>(jsonObj, SdfDataType::eFloat2, &status);
            break;
        }
        case SdfDataType::eFrame4d:
        case SdfDataType::eMatrix4d:
        {
            auto value = pxr::GfMatrix4d();
            for (int i = 0; i < 4; ++i)
            {
                if (validateJsonContainer(jsonObj[i], SdfDataType::eDouble, 4))
                {
                    value.SetRow(i, pxr::GfVec4d(jsonObj[i][0].get<double>(), jsonObj[i][1].get<double>(),
                                                 jsonObj[i][2].get<double>(), jsonObj[i][3].get<double>()));
                }
                else
                {
                    return pxr::VtValue();
                }
            }
            vtValue = pxr::VtValue(value);
            break;
        }
        case SdfDataType::eFrame4dArray:
        case SdfDataType::eMatrix4dArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfMatrix4d>(jsonObj, SdfDataType::eMatrix4d, &status);
            break;
        }
        case SdfDataType::eHalf:
        {
            if (jsonObj.is_number())
            {
                vtValue = pxr::VtValue(pxr::GfHalf(jsonObj.get<float>()));
            }
            break;
        }
        case SdfDataType::eHalfArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfHalf>(jsonObj, SdfDataType::eHalf, &status);
            break;
        }
        case SdfDataType::eHalf2:
        case SdfDataType::eTexCoord2h:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eHalf, 2))
            {
                vtValue = pxr::VtValue(pxr::GfVec2h(pxr::GfVec2f(jsonObj[0].get<float>(), jsonObj[1].get<float>())));
            }
            break;
        }
        case SdfDataType::eHalf2Array:
        case SdfDataType::eTexCoord2hArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec2h>(jsonObj, SdfDataType::eHalf2, &status);
            break;
        }
        case SdfDataType::eInt:
        {
            if (jsonObj.is_number_integer())
            {
                vtValue = pxr::VtValue(jsonObj.get<int32_t>());
            }
            break;
        }
        case SdfDataType::eIntArray:
        {
            vtValue = arrayTypeFromJson<int32_t>(jsonObj, SdfDataType::eInt, &status);
            break;
        }
        case SdfDataType::eInt2:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eInt, 2))
            {
                vtValue = pxr::VtValue(pxr::GfVec2i(jsonObj[0].get<int32_t>(), jsonObj[1].get<int32_t>()));
            }
            break;
        }
        case SdfDataType::eInt2Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec2i>(jsonObj, SdfDataType::eInt2, &status);
            break;
        }
        case SdfDataType::eInt3:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eInt, 3))
            {
                vtValue = pxr::VtValue(
                    pxr::GfVec3i(jsonObj[0].get<int32_t>(), jsonObj[1].get<int32_t>(), jsonObj[2].get<int32_t>()));
            }
            break;
        }
        case SdfDataType::eInt3Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec3i>(jsonObj, SdfDataType::eInt3, &status);
            break;
        }
        case SdfDataType::eInt4:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eInt, 4))
            {
                vtValue = pxr::VtValue(pxr::GfVec4i(jsonObj[0].get<int32_t>(), jsonObj[1].get<int32_t>(),
                                                    jsonObj[2].get<int32_t>(), jsonObj[3].get<int32_t>()));
            }
            break;
        }
        case SdfDataType::eInt4Array:
        {
            vtValue = arrayTypeFromJson<pxr::GfVec4i>(jsonObj, SdfDataType::eInt4, &status);
            break;
        }
        case SdfDataType::eInt64:
        {
            if (jsonObj.is_number_integer())
            {
                vtValue = pxr::VtValue(jsonObj.get<int64_t>());
            }
            break;
        }
        case SdfDataType::eInt64Array:
        {
            vtValue = arrayTypeFromJson<int64_t>(jsonObj, SdfDataType::eInt64, &status);
            break;
        }
        case SdfDataType::eMatrix2d:
        {
            auto value = pxr::GfMatrix2d();
            for (int i = 0; i < 2; ++i)
            {
                if (validateJsonContainer(jsonObj[i], SdfDataType::eDouble, 2))
                {
                    value.SetRow(i, pxr::GfVec2d(jsonObj[i][0].get<double>(), jsonObj[i][1].get<double>()));
                }
                else
                {
                    return pxr::VtValue();
                }
            }
            vtValue = pxr::VtValue(value);
            break;
        }
        case SdfDataType::eMatrix2dArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfMatrix2d>(jsonObj, SdfDataType::eMatrix2d, &status);
            break;
        }
        case SdfDataType::eMatrix3d:
        {
            auto value = pxr::GfMatrix3d();
            for (int i = 0; i < 3; ++i)
            {
                if (validateJsonContainer(jsonObj[i], SdfDataType::eDouble, 3))
                {
                    value.SetRow(i, pxr::GfVec3d(jsonObj[i][0].get<double>(), jsonObj[i][1].get<double>(),
                                                 jsonObj[i][2].get<double>()));
                }
                else
                {
                    return pxr::VtValue();
                }
            }
            vtValue = pxr::VtValue(value);
            break;
        }
        case SdfDataType::eMatrix3dArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfMatrix3d>(jsonObj, SdfDataType::eMatrix3d, &status);
            break;
        }
        case SdfDataType::eQuatd:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eDouble, 4))
            {
                vtValue = pxr::VtValue(pxr::GfQuatd(jsonObj[0].get<double>(), jsonObj[1].get<double>(),
                                                    jsonObj[2].get<double>(), jsonObj[3].get<double>()));
            }
            break;
        }
        case SdfDataType::eQuatdArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfQuatd>(jsonObj, SdfDataType::eQuatd, &status);
            break;
        }
        case SdfDataType::eQuatf:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eFloat, 4))
            {
                vtValue = pxr::VtValue(pxr::GfQuatf(jsonObj[0].get<float>(), jsonObj[1].get<float>(),
                                                    jsonObj[2].get<float>(), jsonObj[3].get<float>()));
            }
            break;
        }
        case SdfDataType::eQuatfArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfQuatf>(jsonObj, SdfDataType::eQuatf, &status);
            break;
        }
        case SdfDataType::eQuath:
        {
            if (validateJsonContainer(jsonObj, SdfDataType::eHalf, 4))
            {
                vtValue = pxr::VtValue(pxr::GfQuath(pxr::GfQuatf(jsonObj[0].get<float>(), jsonObj[1].get<float>(),
                                                                 jsonObj[2].get<float>(), jsonObj[3].get<float>())));
            }
            break;
        }
        case SdfDataType::eQuathArray:
        {
            vtValue = arrayTypeFromJson<pxr::GfQuath>(jsonObj, SdfDataType::eQuath, &status);
            break;
        }
        case SdfDataType::eString:
        {
            if (jsonObj.is_string())
            {
                vtValue = pxr::VtValue(jsonObj.get<std::string>());
            }
            break;
        }
        case SdfDataType::eStringArray:
        {
            vtValue = arrayTypeFromJson<std::string>(jsonObj, SdfDataType::eString, &status);
            break;
        }
        case SdfDataType::eTimeCode:
        {
            if (jsonObj.is_number())
            {
                vtValue = pxr::VtValue(pxr::SdfTimeCode(jsonObj.get<double>()));
            }
            break;
        }
        case SdfDataType::eTimeCodeArray:
        {
            vtValue = arrayTypeFromJson<pxr::SdfTimeCode>(jsonObj, SdfDataType::eTimeCode, &status);
            break;
        }
        case SdfDataType::eToken:
        {
            if (jsonObj.is_string())
            {
                vtValue = pxr::VtValue(pxr::TfToken(jsonObj.get<std::string>()));
            }
            break;
        }
        case SdfDataType::eTokenArray:
        {
            vtValue = arrayTypeFromJson<pxr::TfToken>(jsonObj, SdfDataType::eToken, &status);
            break;
        }
        case SdfDataType::eUChar:
        {
            if (jsonObj.is_number_unsigned())
            {
                vtValue = pxr::VtValue(jsonObj.get<uint8_t>());
            }
            break;
        }
        case SdfDataType::eUCharArray:
        {
            vtValue = arrayTypeFromJson<uint8_t>(jsonObj, SdfDataType::eUChar, &status);
            break;
        }
        case SdfDataType::eUInt:
        {
            if (jsonObj.is_number_unsigned())
            {
                vtValue = pxr::VtValue(jsonObj.get<uint32_t>());
            }
            break;
        }
        case SdfDataType::eUIntArray:
        {
            vtValue = arrayTypeFromJson<uint32_t>(jsonObj, SdfDataType::eUInt, &status);
            break;
        }
        case SdfDataType::eUInt64:
        {
            if (jsonObj.is_number_unsigned())
            {
                vtValue = pxr::VtValue(jsonObj.get<uint64_t>());
            }
            break;
        }
        case SdfDataType::eUInt64Array:
        {
            vtValue = arrayTypeFromJson<uint64_t>(jsonObj, SdfDataType::eUInt64, &status);
            break;
        }
        default:
            break;
        }
        if (!status)
        {
            return pxr::VtValue();
        }
        return vtValue;
    }

    template <typename DataType>
    pxr::VtArray<DataType> arrayTypeFromJson(const nlohmann::json& jsonObj, SdfDataType type, bool* status)
    {
        *status = true;
        pxr::VtArray<DataType> array;
        if (jsonObj.is_array())
        {
            for (size_t i = 0; i < jsonObj.size(); ++i)
            {
                auto value = valueTypeFromJson(jsonObj.at(i), type);
                if (value.IsEmpty())
                {
                    *status = false;
                    break;
                }
                else
                {
                    array.push_back(value.Get<DataType>());
                }
            }
        }
        return array;
    }

    bool validateJsonContainer(const nlohmann::json& jsonObj, SdfDataType type, size_t size)
    {
        if (!jsonObj.is_array() || jsonObj.size() != size)
        {
            return false;
        }
        for (size_t i = 0; i < jsonObj.size(); ++i)
        {
            switch (type)
            {
            case SdfDataType::eBool:
                if (!jsonObj.at(i).is_boolean())
                {
                    return false;
                }
                break;
            case SdfDataType::eUChar:
            case SdfDataType::eUInt:
            case SdfDataType::eUInt64:
                if (!jsonObj.at(i).is_number_unsigned())
                {
                    return false;
                }
                break;
            case SdfDataType::eInt:
            case SdfDataType::eInt64:
                if (!jsonObj.at(i).is_number_integer())
                {
                    return false;
                }
                break;
            case SdfDataType::eHalf:
            case SdfDataType::eFloat:
            case SdfDataType::eDouble:
                if (!jsonObj.at(i).is_number())
                {
                    return false;
                }
                break;
            case SdfDataType::eString:
            case SdfDataType::eToken:
                if (!jsonObj.at(i).is_string())
                {
                    return false;
                }
                break;
            default:
                break;
            }
        }
        return true;
    }

    nlohmann::json valueTypeToJson(const pxr::UsdAttribute& attr, bool useDefaultValueIfEmpty = true)
    {
        pxr::VtValue vtValue;
        attr.Get(&vtValue);
        return valueTypeToJson(vtValue, attr.GetTypeName().GetAsToken(), useDefaultValueIfEmpty);
    }

    nlohmann::json valueTypeToJson(const pxr::VtValue& vtValue,
                                   const pxr::TfToken& typeName,
                                   bool useDefaultValueIfEmpty = true)
    {
        auto name = typeName.GetString();
        if (g_sMapStringToSdfDataType.find(name) != g_sMapStringToSdfDataType.end())
        {
            return valueTypeToJson(vtValue, g_sMapStringToSdfDataType[name], useDefaultValueIfEmpty);
        }
        return nlohmann::json();
    }

    nlohmann::json valueTypeToJson(const pxr::VtValue& vtValue, SdfDataType type, bool useDefaultValueIfEmpty = true)
    {
        nlohmann::json jsonObj;
        auto useDefaultValue = useDefaultValueIfEmpty && vtValue.IsEmpty();
        switch (type)
        {
        case SdfDataType::eAsset:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::SdfAssetPath>() : vtValue.Get<pxr::SdfAssetPath>();
            jsonObj = value.GetAssetPath();
            break;
        }
        case SdfDataType::eAssetArray:
        {
            jsonObj = arrayTypeToJson<pxr::SdfAssetPath>(vtValue, SdfDataType::eAsset, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eBool:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<bool>() : vtValue.Get<bool>();
            break;
        }
        case SdfDataType::eBoolArray:
        {
            jsonObj = arrayTypeToJson<bool>(vtValue, SdfDataType::eBool, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor3d:
        case SdfDataType::eDouble3:
        case SdfDataType::eNormal3d:
        case SdfDataType::ePoint3d:
        case SdfDataType::eTexCoord3d:
        case SdfDataType::eVector3d:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec3d>() : vtValue.Get<pxr::GfVec3d>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2] });
            break;
        }
        case SdfDataType::eColor3dArray:
        case SdfDataType::eDouble3Array:
        case SdfDataType::eNormal3dArray:
        case SdfDataType::ePoint3dArray:
        case SdfDataType::eTexCoord3dArray:
        case SdfDataType::eVector3dArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec3d>(vtValue, SdfDataType::eDouble3, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor3f:
        case SdfDataType::eFloat3:
        case SdfDataType::eNormal3f:
        case SdfDataType::ePoint3f:
        case SdfDataType::eTexCoord3f:
        case SdfDataType::eVector3f:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec3f>() : vtValue.Get<pxr::GfVec3f>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2] });
            break;
        }
        case SdfDataType::eColor3fArray:
        case SdfDataType::eFloat3Array:
        case SdfDataType::eNormal3fArray:
        case SdfDataType::ePoint3fArray:
        case SdfDataType::eTexCoord3fArray:
        case SdfDataType::eVector3fArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec3f>(vtValue, SdfDataType::eFloat3, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor3h:
        case SdfDataType::eHalf3:
        case SdfDataType::eNormal3h:
        case SdfDataType::ePoint3h:
        case SdfDataType::eTexCoord3h:
        case SdfDataType::eVector3h:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec3h>() : vtValue.Get<pxr::GfVec3h>();
            jsonObj = nlohmann::json::array(
                { static_cast<float>(value[0]), static_cast<float>(value[1]), static_cast<float>(value[2]) });
            break;
        }
        case SdfDataType::eColor3hArray:
        case SdfDataType::eHalf3Array:
        case SdfDataType::eNormal3hArray:
        case SdfDataType::ePoint3hArray:
        case SdfDataType::eTexCoord3hArray:
        case SdfDataType::eVector3hArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec3h>(vtValue, SdfDataType::eHalf3, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor4d:
        case SdfDataType::eDouble4:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec4d>() : vtValue.Get<pxr::GfVec4d>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2], value[3] });
            break;
        }
        case SdfDataType::eColor4dArray:
        case SdfDataType::eDouble4Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec4d>(vtValue, SdfDataType::eDouble4, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor4f:
        case SdfDataType::eFloat4:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec4f>() : vtValue.Get<pxr::GfVec4f>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2], value[3] });
            break;
        }
        case SdfDataType::eColor4fArray:
        case SdfDataType::eFloat4Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec4f>(vtValue, SdfDataType::eFloat4, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eColor4h:
        case SdfDataType::eHalf4:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec4h>() : vtValue.Get<pxr::GfVec4h>();
            jsonObj = nlohmann::json::array({ static_cast<float>(value[0]), static_cast<float>(value[1]),
                                              static_cast<float>(value[2]), static_cast<float>(value[3]) });
            break;
        }
        case SdfDataType::eColor4hArray:
        case SdfDataType::eHalf4Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec4h>(vtValue, SdfDataType::eHalf4, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eDouble:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<double>() : vtValue.Get<double>();
            break;
        }
        case SdfDataType::eDoubleArray:
        {
            jsonObj = arrayTypeToJson<double>(vtValue, SdfDataType::eDouble, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eDouble2:
        case SdfDataType::eTexCoord2d:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec2d>() : vtValue.Get<pxr::GfVec2d>();
            jsonObj = nlohmann::json::array({ value[0], value[1] });
            break;
        }
        case SdfDataType::eDouble2Array:
        case SdfDataType::eTexCoord2dArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec2d>(vtValue, SdfDataType::eDouble2, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eFloat:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<float>() : vtValue.Get<float>();
            break;
        }
        case SdfDataType::eFloatArray:
        {
            jsonObj = arrayTypeToJson<float>(vtValue, SdfDataType::eFloat, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eFloat2:
        case SdfDataType::eTexCoord2f:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec2f>() : vtValue.Get<pxr::GfVec2f>();
            jsonObj = nlohmann::json::array({ value[0], value[1] });
            break;
        }
        case SdfDataType::eFloat2Array:
        case SdfDataType::eTexCoord2fArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec2f>(vtValue, SdfDataType::eFloat2, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eFrame4d:
        case SdfDataType::eMatrix4d:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfMatrix4d>() : vtValue.Get<pxr::GfMatrix4d>();
            jsonObj = nlohmann::json::array();
            jsonObj.push_back(nlohmann::json::array(
                { value.GetRow(0)[0], value.GetRow(0)[1], value.GetRow(0)[2], value.GetRow(0)[3] }));
            jsonObj.push_back(nlohmann::json::array(
                { value.GetRow(1)[0], value.GetRow(1)[1], value.GetRow(1)[2], value.GetRow(1)[3] }));
            jsonObj.push_back(nlohmann::json::array(
                { value.GetRow(2)[0], value.GetRow(2)[1], value.GetRow(2)[2], value.GetRow(2)[3] }));
            jsonObj.push_back(nlohmann::json::array(
                { value.GetRow(3)[0], value.GetRow(3)[1], value.GetRow(3)[2], value.GetRow(3)[3] }));
            break;
        }
        case SdfDataType::eFrame4dArray:
        case SdfDataType::eMatrix4dArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfMatrix4d>(vtValue, SdfDataType::eMatrix4d, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eHalf:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfHalf>() : vtValue.Get<pxr::GfHalf>();
            jsonObj = static_cast<float>(value);
            break;
        }
        case SdfDataType::eHalfArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfHalf>(vtValue, SdfDataType::eHalf, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eHalf2:
        case SdfDataType::eTexCoord2h:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec2h>() : vtValue.Get<pxr::GfVec2h>();
            jsonObj = nlohmann::json::array({ static_cast<float>(value[0]), static_cast<float>(value[1]) });
            break;
        }
        case SdfDataType::eHalf2Array:
        case SdfDataType::eTexCoord2hArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec2h>(vtValue, SdfDataType::eHalf2, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eInt:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<int32_t>() : vtValue.Get<int32_t>();
            break;
        }
        case SdfDataType::eIntArray:
        {
            jsonObj = arrayTypeToJson<int32_t>(vtValue, SdfDataType::eInt, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eInt2:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec2i>() : vtValue.Get<pxr::GfVec2i>();
            jsonObj = nlohmann::json::array({ value[0], value[1] });
            break;
        }
        case SdfDataType::eInt2Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec2i>(vtValue, SdfDataType::eInt2, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eInt3:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec3i>() : vtValue.Get<pxr::GfVec3i>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2] });
            break;
        }
        case SdfDataType::eInt3Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec3i>(vtValue, SdfDataType::eInt3, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eInt4:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfVec4i>() : vtValue.Get<pxr::GfVec4i>();
            jsonObj = nlohmann::json::array({ value[0], value[1], value[2], value[3] });
            break;
        }
        case SdfDataType::eInt4Array:
        {
            jsonObj = arrayTypeToJson<pxr::GfVec4i>(vtValue, SdfDataType::eInt4, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eInt64:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<int64_t>() : vtValue.Get<int64_t>();
            break;
        }
        case SdfDataType::eInt64Array:
        {
            jsonObj = arrayTypeToJson<int64_t>(vtValue, SdfDataType::eInt64, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eMatrix2d:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfMatrix2d>() : vtValue.Get<pxr::GfMatrix2d>();
            jsonObj = nlohmann::json::array();
            jsonObj.push_back(nlohmann::json::array({ value.GetRow(0)[0], value.GetRow(0)[1] }));
            jsonObj.push_back(nlohmann::json::array({ value.GetRow(1)[0], value.GetRow(1)[1] }));
            break;
        }
        case SdfDataType::eMatrix2dArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfMatrix2d>(vtValue, SdfDataType::eMatrix2d, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eMatrix3d:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfMatrix3d>() : vtValue.Get<pxr::GfMatrix3d>();
            jsonObj = nlohmann::json::array();
            jsonObj.push_back(nlohmann::json::array({ value.GetRow(0)[0], value.GetRow(0)[1], value.GetRow(0)[2] }));
            jsonObj.push_back(nlohmann::json::array({ value.GetRow(1)[0], value.GetRow(1)[1], value.GetRow(1)[2] }));
            jsonObj.push_back(nlohmann::json::array({ value.GetRow(2)[0], value.GetRow(2)[1], value.GetRow(2)[2] }));
            break;
        }
        case SdfDataType::eMatrix3dArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfMatrix3d>(vtValue, SdfDataType::eMatrix3d, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eQuatd:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfQuatd>() : vtValue.Get<pxr::GfQuatd>();
            auto imaginary = value.GetImaginary();
            jsonObj = nlohmann::json::array({ value.GetReal(), imaginary[0], imaginary[1], imaginary[2] });
            break;
        }
        case SdfDataType::eQuatdArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfQuatd>(vtValue, SdfDataType::eQuatd, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eQuatf:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfQuatf>() : vtValue.Get<pxr::GfQuatf>();
            auto imaginary = value.GetImaginary();
            jsonObj = nlohmann::json::array({ value.GetReal(), imaginary[0], imaginary[1], imaginary[2] });
            break;
        }
        case SdfDataType::eQuatfArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfQuatf>(vtValue, SdfDataType::eQuatf, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eQuath:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::GfQuath>() : vtValue.Get<pxr::GfQuath>();
            auto imaginary = value.GetImaginary();
            jsonObj = nlohmann::json::array({ static_cast<float>(value.GetReal()), static_cast<float>(imaginary[0]),
                                              static_cast<float>(imaginary[1]), static_cast<float>(imaginary[2]) });
            break;
        }
        case SdfDataType::eQuathArray:
        {
            jsonObj = arrayTypeToJson<pxr::GfQuath>(vtValue, SdfDataType::eQuath, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eString:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<std::string>() : vtValue.Get<std::string>();
            break;
        }
        case SdfDataType::eStringArray:
        {
            jsonObj = arrayTypeToJson<std::string>(vtValue, SdfDataType::eString, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eTimeCode:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::SdfTimeCode>() : vtValue.Get<pxr::SdfTimeCode>();
            jsonObj = value.GetValue();
            break;
        }
        case SdfDataType::eTimeCodeArray:
        {
            jsonObj = arrayTypeToJson<pxr::SdfTimeCode>(vtValue, SdfDataType::eTimeCode, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eToken:
        {
            auto value = useDefaultValue ? vtValue.GetWithDefault<pxr::TfToken>() : vtValue.Get<pxr::TfToken>();
            jsonObj = value.data();
            break;
        }
        case SdfDataType::eTokenArray:
        {
            jsonObj = arrayTypeToJson<pxr::TfToken>(vtValue, SdfDataType::eToken, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eUChar:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<uint8_t>() : vtValue.Get<uint8_t>();
            break;
        }
        case SdfDataType::eUCharArray:
        {
            jsonObj = arrayTypeToJson<uint8_t>(vtValue, SdfDataType::eUChar, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eUInt:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<uint32_t>() : vtValue.Get<uint32_t>();
            break;
        }
        case SdfDataType::eUIntArray:
        {
            jsonObj = arrayTypeToJson<uint32_t>(vtValue, SdfDataType::eUInt, useDefaultValueIfEmpty);
            break;
        }
        case SdfDataType::eUInt64:
        {
            jsonObj = useDefaultValue ? vtValue.GetWithDefault<uint64_t>() : vtValue.Get<uint64_t>();
            break;
        }
        case SdfDataType::eUInt64Array:
        {
            jsonObj = arrayTypeToJson<uint64_t>(vtValue, SdfDataType::eUInt64, useDefaultValueIfEmpty);
            break;
        }
        default:
            break;
        }
        return jsonObj;
    }

    template <typename DataType>
    nlohmann::json arrayTypeToJson(const pxr::VtValue& vtValue, SdfDataType type, bool useDefaultValueIfEmpty = true)
    {
        auto jsonObj = nlohmann::json::array();
        auto useDefaultValue = useDefaultValueIfEmpty && vtValue.IsEmpty();
        auto array =
            useDefaultValue ? vtValue.GetWithDefault<pxr::VtArray<DataType>>() : vtValue.Get<pxr::VtArray<DataType>>();
        for (size_t i = 0; i < array.size(); ++i)
        {
            auto item = pxr::VtValue(array[i]);
            jsonObj.push_back(valueTypeToJson(item, type, useDefaultValueIfEmpty));
        }
        return jsonObj;
    }
};

REGISTER_OGN_NODE()
