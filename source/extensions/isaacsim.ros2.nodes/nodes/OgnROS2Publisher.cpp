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

#include <OgnROS2PublisherDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2Publisher : public Ros2Node
{
public:
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublisherDatabase::sPerInstanceState<OgnROS2Publisher>(nodeObj, instanceId);
        state.m_nodeObj = nodeObj;

        // Register change event for message type
        AttributeObj attrMessagePackageObj = nodeObj.iNode->getAttribute(nodeObj, "inputs:messagePackage");
        AttributeObj attrMessageSubfolderObj = nodeObj.iNode->getAttribute(nodeObj, "inputs:messageSubfolder");
        AttributeObj attrMessageNameObj = nodeObj.iNode->getAttribute(nodeObj, "inputs:messageName");
        attrMessagePackageObj.iAttribute->registerValueChangedCallback(attrMessagePackageObj, _onMessageChanged, true);
        attrMessageSubfolderObj.iAttribute->registerValueChangedCallback(
            attrMessageSubfolderObj, _onMessageChanged, true);
        attrMessageNameObj.iAttribute->registerValueChangedCallback(attrMessageNameObj, _onMessageChanged, true);
    }

    static bool compute(OgnROS2PublisherDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2Publisher>();

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

        // Check for changes in message type
        std::string messagePackage = std::string(db.inputs.messagePackage());
        std::string messageSubfolder = std::string(db.inputs.messageSubfolder());
        std::string messageName = std::string(db.inputs.messageName());
        if (messagePackage != state.m_messagePackage)
        {
            state.m_messageUpdateNeeded = true;
            state.m_messagePackage = messagePackage;
        }
        if (messageSubfolder != state.m_messageSubfolder)
        {
            state.m_messageUpdateNeeded = true;
            state.m_messageSubfolder = messageSubfolder;
        }
        if (messageName != state.m_messageName)
        {
            state.m_messageUpdateNeeded = true;
            state.m_messageName = messageName;
        }
        // Update message and node attributes
        if (state.m_messageUpdateNeeded || !state.m_message)
        {
            bool status = _createMessageAndAttributes(
                nodeObj, state.m_messagePackage, state.m_messageSubfolder, state.m_messageName);
            if (!status)
            {
                return false;
            }
            state.m_messageUpdateNeeded = false;
            state.m_publisherUpdateNeeded = true;
            return false;
        }

        // Check for changes in publisher
        std::string topicName = std::string(db.inputs.topicName());
        uint64_t queueSize = db.inputs.queueSize();
        std::string qosProfile = db.inputs.qosProfile();
        if (topicName != state.m_topicName)
        {
            state.m_publisherUpdateNeeded = true;
            state.m_topicName = topicName;
        }
        if (queueSize != state.m_queueSize)
        {
            state.m_publisherUpdateNeeded = true;
            state.m_queueSize = queueSize;
        }
        if (qosProfile != state.m_qosProfile)
        {
            state.m_publisherUpdateNeeded = true;
            state.m_qosProfile = qosProfile;
        }
        // Update publisher
        if (state.m_publisherUpdateNeeded)
        {
            // Destroy previous publisher
            state.m_publisher.reset();

            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, state.m_topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            Ros2QoSProfile qos;
            const std::string& currentQosProfile = db.inputs.qosProfile();
            if (currentQosProfile.empty())
            {
                qos.depth = state.m_queueSize;
            }
            else
            {
                if (!jsonToRos2QoSProfile(qos, state.m_qosProfile))
                {
                    return false;
                }
            }

            // Create publisher
            std::string messageType = messagePackage + "/" + messageSubfolder + "/" + messageName;
            CARB_LOG_INFO("OgnROS2Publisher: creating publisher: %s (%s)", fullTopicName.c_str(), messageType.c_str());
            state.m_publisher = state.m_factory->createPublisher(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);
            if (!state.m_publisher->isValid())
            {
                db.logWarning(
                    ("Invalid publication to the topic " + fullTopicName + " for the message type " + messageType).c_str());
                state.m_publisher.reset();
                return false;
            }
            state.m_publisherUpdateNeeded = false;
            return true;
        }

        return state.publisherCallback(db);
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublisherDatabase::sPerInstanceState<OgnROS2Publisher>(nodeObj, instanceId);
        state.reset();
    }

    void reset() override
    {
        m_publisherUpdateNeeded = false;
        m_messageUpdateNeeded = false;

        m_messagePackage.clear();
        m_messageSubfolder.clear();
        m_messageName.clear();
        m_topicName.clear();
        m_qosProfile.clear();
        m_queueSize = 0;

        m_message.reset();
        m_publisher.reset(); // This should be reset before reset the handle

        Ros2Node::reset();
    }

    bool publisherCallback(OgnROS2PublisherDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2Publisher>();
        if (!state.m_publisher)
        {
            return false;
        }

        auto messageData = std::static_pointer_cast<Ros2DynamicMessage>(state.m_message)->getVectorContainer(true);
        auto messageFields = std::static_pointer_cast<Ros2DynamicMessage>(state.m_message)->getMessageFields();

        size_t arraySize;
        for (size_t i = 0; i < messageFields.size(); ++i)
        {
            auto messageField = messageFields.at(i);
            switch (messageField.dataType)
            {
            case omni::fabric::BaseDataType::eBool:
            {
                if (messageField.isArray)
                {
                    auto inputValue =
                        _getAttributeReadableArrayData<bool*>(db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<bool>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    // std::vector<bool> is a specialization that has no ::data
                    for (size_t j = 0; j < arraySize; ++j)
                    {
                        (*data)[j] = *((*inputValue) + j);
                    }
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<bool>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<bool>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eUChar:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<uint8_t*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<uint8_t>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(uint8_t));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<uint8_t>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<uint8_t>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eInt:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<int32_t*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<int32_t>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(int32_t));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<int32_t>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<int32_t>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eUInt:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<uint32_t*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<uint32_t>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(uint32_t));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<uint32_t>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<uint32_t>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eInt64:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<int64_t*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<int64_t>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(int64_t));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<int64_t>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<int64_t>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eUInt64:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<uint64_t*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<uint64_t>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(uint64_t));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<uint64_t>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<uint64_t>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eHalf:
            {
                // No half-precision floating point number in types for ROS message fields
                break;
            }
            case omni::fabric::BaseDataType::eFloat:
            {
                if (messageField.isArray)
                {
                    auto inputValue =
                        _getAttributeReadableArrayData<float*>(db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<float>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(float));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<float>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<float>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eDouble:
            {
                if (messageField.isArray)
                {
                    auto inputValue =
                        _getAttributeReadableArrayData<double*>(db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<double>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    std::memcpy(data->data(), *inputValue, arraySize * sizeof(double));
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<double>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<double>(messageData[i]) = *inputValue;
                }
                break;
            }
            case omni::fabric::BaseDataType::eToken:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<NameToken*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<std::string>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    for (size_t j = 0; j < arraySize; ++j)
                    {
                        (*data)[j] = db.tokenToString(*((*inputValue) + j));
                    }
                }
                else
                {
                    auto inputValue = _getAttributeReadableData<NameToken>(db.abi_node(), "inputs:" + messageField.name);
                    *std::static_pointer_cast<std::string>(messageData[i]) = db.tokenToString(*inputValue);
                }
                break;
            }
            case omni::fabric::BaseDataType::eUnknown:
            {
                if (messageField.isArray)
                {
                    auto inputValue = _getAttributeReadableArrayData<NameToken*>(
                        db.abi_node(), "inputs:" + messageField.name, arraySize);
                    auto data = std::static_pointer_cast<std::vector<nlohmann::json>>(messageData[i]);
                    data->clear();
                    data->resize(arraySize);
                    for (size_t j = 0; j < arraySize; ++j)
                    {
                        (*data)[j] = nlohmann::json::parse(db.tokenToString(*((*inputValue) + j)));
                    }
                }
                break;
            }
            default:
                break;
            }
        }

        std::static_pointer_cast<Ros2DynamicMessage>(state.m_message)->writeData(messageData, true);
        state.m_publisher->publish(state.m_message->getPtr());

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

private:
    NodeObj m_nodeObj;
    bool m_publisherUpdateNeeded = false;
    bool m_messageUpdateNeeded = false;

    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2Message> m_message = nullptr;

    std::string m_messagePackage;
    std::string m_messageSubfolder;
    std::string m_messageName;
    std::string m_topicName;
    uint64_t m_queueSize = 10;
    std::string m_qosProfile;

    // OGN utils

    static AttributeObj _getAttributeObj(const NodeObj& nodeObj, const std::string& attrName)
    {
        AttributeObj attrObj = nodeObj.iNode->getAttribute(nodeObj, attrName.c_str());
        CARB_ASSERT(attrObj.isValid());
        return attrObj;
    }

    template <typename T>
    static const T* _getAttributeReadableData(const NodeObj& nodeObj, const std::string& attrName)
    {
        GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
        GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
        ConstAttributeDataHandle handle = getAttributeR(
            context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
        const T* value = getDataR<T>(context, handle);
        return value;
    }

    template <typename T>
    static T const* _getAttributeReadableArrayData(const NodeObj& nodeObj, const std::string& attrName, size_t& countOut)
    {
        GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
        GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
        ConstAttributeDataHandle handle = getAttributeR(
            context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
        // Get size
        context.iAttributeData->getElementCount(&countOut, context, &handle, 1u);
        // Get readable data
        T const* value = getDataR<T>(context, handle);
        return value;
    }

    static const char* _getTokenText(AttributeObj const& attrObj)
    {
        NodeObj nodeObj = attrObj.iAttribute->getNode(attrObj);
        GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
        GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);

        ConstAttributeDataHandle handle =
            attrObj.iAttribute->getConstAttributeDataHandle(attrObj, kAccordingToContextIndex);
        auto const token = getDataR<NameToken>(context, handle);
        return token->getText();
    }

    static const char* _getTokenText(const NodeObj& nodeObj, const std::string& attrName)
    {
        return _getTokenText(_getAttributeObj(nodeObj, attrName));
    }

    static const char* _getTokenText(const NodeObj& nodeObj, const omni::graph::core::NameToken& token)
    {
        return token.getText();
    }

    static bool _setAllowedTokens(const NodeObj& nodeObj,
                                  const std::string& attrName,
                                  const std::vector<std::string>& allowedTokens)
    {
        // OGN
        std::stringstream stream;
        copy(allowedTokens.begin(), allowedTokens.end(), std::ostream_iterator<std::string>(stream, ","));
        std::string ognAllowedTokens = stream.str();
        AttributeObj attrObj = _getAttributeObj(nodeObj, attrName);
        attrObj.iAttribute->setMetadata(attrObj, kOgnMetadataAllowedTokens, ognAllowedTokens.c_str());
        // USD
        pxr::UsdStagePtr stage = omni::usd::UsdContext::getContext()->getStage();
        if (!stage)
        {
            return false;
        }
        const pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)));
        const pxr::UsdAttribute attr = prim.GetAttribute(pxr::TfToken(attrName));
        if (!attr.IsValid())
        {
            return false;
        }
        attr.SetMetadata(pxr::TfToken(kOgnMetadataAllowedTokens), allowedTokens);
        return true;
    }

    static bool _removeDynamicAttributes(const NodeObj& nodeObj, AttributePortType portType)
    {
        // Get node attributes
        auto attrCount = nodeObj.iNode->getAttributeCount(nodeObj);
        std::vector<AttributeObj> attrObjects(attrCount);
        nodeObj.iNode->getAttributes(nodeObj, attrObjects.data(), attrCount);
        // Iterate and delete requested attributes
        bool status = true;
        for (auto const& attrObj : attrObjects)
        {
            if (attrObj.iAttribute->isDynamic(attrObj))
            {
                if (attrObj.iAttribute->getPortType(attrObj) == portType)
                {
                    // Disconnect attribute if connected
                    if (attrObj.iAttribute->getUpstreamConnectionCount(attrObj))
                    {
                        ConnectionInfo connectionInfo;
                        attrObj.iAttribute->getUpstreamConnectionsInfo(attrObj, &connectionInfo, 1);
                        status = status && attrObj.iAttribute->disconnectAttrs(connectionInfo.attrObj, attrObj, true);
                    }
                    // Remove attribute
                    char const* attrName = attrObj.iAttribute->getName(attrObj);
                    status = status && nodeObj.iNode->removeAttribute(nodeObj, attrName);
                }
            }
        }
        return status;
    }

    // Node

    static bool _createMessageAndAttributes(const NodeObj& nodeObj,
                                            const std::string& messagePackage,
                                            const std::string& messageSubfolder,
                                            const std::string& messageName)
    {
        auto db = OgnROS2PublisherDatabase(nodeObj);
        auto& state = db.perInstanceState<OgnROS2Publisher>();
        std::string messageType = messagePackage + "/" + messageSubfolder + "/" + messageName;
        // Naive check on inputs
        if (messagePackage.empty() || messageSubfolder.empty() || messageName.empty())
        {
            return false;
        }
        // Create message
        CARB_LOG_INFO("OgnROS2Publisher: create message for %s", messageType.c_str());
        state.m_message = state.m_factory->createDynamicMessage(messagePackage, messageSubfolder, messageName);
        auto messageFields = std::static_pointer_cast<Ros2DynamicMessage>(state.m_message)->getMessageFields();
        bool status = std::static_pointer_cast<Ros2DynamicMessage>(state.m_message)->isValid();
        if (!status)
        {
            db.logWarning((messageType + " does not exist or is not available in the ROS 2 environment").c_str());
            state.m_message.reset();
            return false;
        }
        // Check if existing dynamic attributes match the message
        if (_checkForMatchingAttributes(nodeObj, messageFields))
        {
            CARB_LOG_INFO("OgnROS2Publisher: reuse of existing dynamic attributes: %s", messageType.c_str());
            return true;
        }
        // Remove dynamic attributes
        CARB_LOG_INFO("OgnROS2Publisher: remove dynamic attributes: %s", messageType.c_str());
        status = _removeDynamicAttributes(nodeObj, AttributePortType::kAttributePortType_Input);
        if (!status)
        {
            db.logWarning("Unable to remove existing attributes from the node");
            return false;
        }
        // Create dynamic attributes
        CARB_LOG_INFO("OgnROS2Publisher: create dynamic attributes: %s", messageType.c_str());
        for (auto const& messageField : messageFields)
        {
            CARB_LOG_INFO("OgnROS2Publisher: |-- %s (OGN type name: %s, fabric type: %d, ROS type: %d)",
                          messageField.name.c_str(), messageField.ognType.c_str(),
                          static_cast<int>(messageField.dataType), messageField.rosType);
            status = status &&
                     nodeObj.iNode->createAttribute(nodeObj, ("inputs:" + messageField.name).c_str(),
                                                    db.typeFromName(db.stringToToken(messageField.ognType.c_str())),
                                                    nullptr, nullptr, AttributePortType::kAttributePortType_Input,
                                                    ExtendedAttributeType::kExtendedAttributeType_Regular, nullptr);
            if (!status)
            {
                db.logWarning(
                    ("Unable to create attribute " + messageField.name + " of type " + messageField.ognType).c_str());
                _removeDynamicAttributes(nodeObj, AttributePortType::kAttributePortType_Input);
                return false;
            }
        }
        return status;
    }

    static bool _checkForMatchingAttributes(const NodeObj& nodeObj, std::vector<DynamicMessageField> messageFields)
    {
        auto db = OgnROS2PublisherDatabase(nodeObj);
        auto dynamicInputs = db.getDynamicInputs();
        // Check for the number of attributes
        if (dynamicInputs.size() != messageFields.size())
        {
            return false;
        }
        // Check for attribute name and type
        for (auto const& dynamicInput : dynamicInputs)
        {
            auto it =
                std::find_if(messageFields.begin(), messageFields.end(),
                             [&](const auto& messageField)
                             { return db.tokenToString(dynamicInput().name()) == ("inputs:" + messageField.name); });
            bool status = (it != messageFields.end() && dynamicInput().typeName() == it->ognType);
            if (!status)
            {
                return false;
            }
        }
        return true;
    }

    // Node events

    static void _onMessageChanged(AttributeObj const& attrObj, void const* userData)
    {
        // Get message package, subfolder and name
        NodeObj nodeObj = attrObj.iAttribute->getNode(attrObj);
        auto db = OgnROS2PublisherDatabase(nodeObj);
        std::string messagePackage = std::string(db.inputs.messagePackage());
        std::string messageSubfolder = std::string(db.inputs.messageSubfolder());
        std::string messageName = std::string(db.inputs.messageName());
        // Build message attributes
        _createMessageAndAttributes(nodeObj, messagePackage, messageSubfolder, messageName);
    }
};

REGISTER_OGN_NODE()
