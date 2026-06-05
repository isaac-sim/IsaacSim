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

/** @file
 * @brief OmniGraph utilities for the ROS 2 core
 * @details
 * This file provides utility functions for bridging between ROS 2 and OmniGraph data types.
 * It includes functions for creating OmniGraph attributes from ROS 2 message fields,
 * reading and writing data between OmniGraph nodes and ROS 2 messages, and
 * utility functions for handling attributes and data type conversions.
 */
#pragma once

#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/Framework.h>
#include <carb/Types.h>
#include <carb/tokens/TokensUtils.h>

#include <isaacsim/core/includes/Math.h>
#include <isaacsim/ros2/core/Ros2Node.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/graph/core/OgnHelpers.h>
#include <omni/graph/core/Type.h>
#include <omni/graph/core/ogn/ArrayAttribute.h>
#include <omni/graph/core/ogn/SimpleAttribute.h>

using namespace isaacsim::ros2::core;
using omni::graph::core::ogn::OmniGraphDatabase;

namespace isaacsim
{
namespace ros2
{
namespace omnigraph_utils
{

/**
 * @brief Returns the OmniGraph attribute prefix based on direction
 * @details
 * Returns "outputs" for output attributes or "inputs" for input attributes.
 * Used to construct attribute names in OmniGraph compatible format.
 *
 * @param[in] isOutput True for output attributes, false for input attributes
 * @return std::string Either "outputs" or "inputs" based on direction
 */
inline std::string inputOutput(bool isOutput)
{
    return isOutput ? "outputs" : "inputs";
};

/**
 * @brief Verifies that a pointer is not null and logs an error message if it is
 * @details
 * Utility function to check for null pointers and log an error message.
 * Useful for validating data pointers during attribute access operations.
 *
 * @param[in] ptr Pointer to check for nullness
 * @param[in] message Error message to log if pointer is null
 * @return bool True if the pointer is valid (non-null), false otherwise
 */
inline bool checkCondition(const void* ptr, std::string message)
{
    if (!ptr)
    {
        CARB_LOG_ERROR("%s %p", message.c_str(), ptr);
    }
    return ptr;
}

/**
 * @brief Gets writable data for a simple attribute
 * @details
 * Retrieves a writable pointer to an attribute's data.
 * Used for modifying attribute values in an OmniGraph node.
 *
 * @tparam T Data type of the attribute
 * @param[in] nodeObj OmniGraph node object containing the attribute
 * @param[in] attrName Name of the attribute to retrieve
 * @return T* Writable pointer to the attribute data, or nullptr if not found
 */
template <typename T>
inline T* getAttributeWritableData(const NodeObj& nodeObj, const std::string& attrName)
{
    GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
    GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
    AttributeDataHandle handle = getAttributeW(
        context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
    T* value = getDataW<T>(context, handle);
    return value;
}

/**
 * @brief Gets writable data for an array attribute with resizing
 * @details
 * Retrieves a writable pointer to an array attribute's data, resizing the array to the specified size.
 * Used for modifying array attribute values in an OmniGraph node.
 *
 * @tparam T Data type of the array elements
 * @param[in] nodeObj OmniGraph node object containing the attribute
 * @param[in] attrName Name of the attribute to retrieve
 * @param[in] newCount New size to set for the array
 * @return T* Writable pointer to the array data, or nullptr if not found
 */
template <typename T>
inline T* getAttributeWritableArrayData(const NodeObj& nodeObj, const std::string& attrName, size_t newCount)
{
    GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
    GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
    AttributeDataHandle handle = getAttributeW(
        context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
    // Resize first
    context.iAttributeData->setElementCount(context, handle, newCount);
    // Get writable data
    T* value = getDataW<T>(context, handle);
    return value;
}

/**
 * @brief Gets read-only data for a simple attribute
 * @details
 * Retrieves a read-only pointer to an attribute's data.
 * Used for reading attribute values from an OmniGraph node.
 *
 * @tparam T Data type of the attribute
 * @param[in] nodeObj OmniGraph node object containing the attribute
 * @param[in] attrName Name of the attribute to retrieve
 * @return T const* Read-only pointer to the attribute data, or nullptr if not found
 */
template <typename T>
inline T const* getAttributeReadableData(const NodeObj& nodeObj, const std::string& attrName)
{
    GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
    GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
    const ConstAttributeDataHandle handle = getAttributeR(
        context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
    T const* value = getDataR<T>(context, handle);
    return value;
}

/**
 * @brief Gets read-only data for an array attribute
 * @details
 * Retrieves a read-only pointer to an array attribute's data and returns its current size.
 * Used for reading array attribute values from an OmniGraph node.
 *
 * @tparam T Data type of the array elements
 * @param[in] nodeObj OmniGraph node object containing the attribute
 * @param[in] attrName Name of the attribute to retrieve
 * @param[out] newCount Variable to receive the array size
 * @return T const* Read-only pointer to the array data, or nullptr if not found
 */
template <typename T>
inline T const* getAttributeReadableArrayData(const NodeObj& nodeObj, const std::string& attrName, size_t& newCount)
{
    GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
    GraphContextObj context = graphObj.iGraph->getDefaultGraphContext(graphObj);
    const ConstAttributeDataHandle constHandle = getAttributeR(
        context, nodeObj.nodeContextHandle, omni::fabric::Token(attrName.c_str()), kAccordingToContextIndex);
    context.iAttributeData->getElementCount(&newCount, context, &constHandle, 1);
    T const* value = getDataR<T>(context, constHandle);
    return value;
}

/**
 * @brief Removes dynamic attributes from a node
 * @details
 * Selectively removes dynamic input and/or output attributes from an OmniGraph node.
 * Useful for resetting a node when reconfiguring its interface based on a new message type.
 *
 * @tparam removeInputs Whether to remove dynamic input attributes
 * @tparam removeOutputs Whether to remove dynamic output attributes
 * @param[in] nodeObj OmniGraph node object to modify
 * @return bool True if all requested attributes were successfully removed
 */
template <bool removeInputs = false, bool removeOutputs = false>
inline bool removeDynamicAttributes(const NodeObj& nodeObj)
{
    // Get node attributes
    auto attrCount = nodeObj.iNode->getAttributeCount(nodeObj);
    std::vector<AttributeObj> attrObjects(attrCount);
    nodeObj.iNode->getAttributes(nodeObj, attrObjects.data(), attrCount);
    // Iterate and delete requested attributes
    bool status = true;
    for (auto const& attrObj : attrObjects)
    {
        char const* attrName = attrObj.iAttribute->getName(attrObj);
        if (attrObj.iAttribute->isDynamic(attrObj))
        {
            if (attrObj.iAttribute->getPortType(attrObj) == AttributePortType::kAttributePortType_Output && removeOutputs)
            {
                // Disconnect attribute if connected
                if (attrObj.iAttribute->getDownstreamConnectionCount(attrObj))
                {
                    ConnectionInfo connectionInfo;
                    attrObj.iAttribute->getDownstreamConnectionsInfo(attrObj, &connectionInfo, 1);
                    status = status && attrObj.iAttribute->disconnectAttrs(attrObj, connectionInfo.attrObj, true);
                }
                // Remove attribute
                status = status && nodeObj.iNode->removeAttribute(nodeObj, attrName);
            }
            else if (attrObj.iAttribute->getPortType(attrObj) == AttributePortType::kAttributePortType_Input &&
                     removeInputs)
            {
                // Disconnect attribute if connected
                if (attrObj.iAttribute->getUpstreamConnectionCount(attrObj))
                {
                    ConnectionInfo connectionInfo;
                    attrObj.iAttribute->getUpstreamConnectionsInfo(attrObj, &connectionInfo, 1);
                    status = status && attrObj.iAttribute->disconnectAttrs(connectionInfo.attrObj, attrObj, true);
                }
                // Remove attribute
                status = status && nodeObj.iNode->removeAttribute(nodeObj, attrName);
            }
        }
    }
    return status;
}

/**
 * @brief Checks if node attributes match message fields
 * @details
 * Verifies if a node's dynamic attributes match the specified message fields in name and type.
 * Used to determine if a node needs to be reconfigured for a different message type.
 *
 * @tparam OgnROS2DatabaseDerivedType Database type for OmniGraph-ROS2 integration
 * @tparam isOutput Whether to check output (true) or input (false) attributes
 * @tparam clearExistingAttrs Whether to check all attributes match (true) or just that required ones exist (false)
 * @param[in] db Database instance
 * @param[in] nodeObj Node object to check
 * @param[in] messageFields Vector of message fields to match against node attributes
 * @param[in] prependStr Optional prefix for attribute names
 * @return bool True if attributes match message fields according to specified criteria
 */
template <class OgnROS2DatabaseDerivedType, bool isOutput, bool clearExistingAttrs>
inline bool findMatchingAttribute(OgnROS2DatabaseDerivedType& db,
                                  const NodeObj& nodeObj,
                                  std::vector<DynamicMessageField> messageFields,
                                  const std::string prependStr)
{
    if (isOutput)
    {
        auto dynamicAttributes = db.getDynamicOutputs();
        if (dynamicAttributes.size() != messageFields.size() && clearExistingAttrs)
        {
            return false;
        }
        // Check for attribute name and type
        for (auto const& messageField : messageFields)
        {
            bool status = false;
            for (auto const& attribute : dynamicAttributes)
            {
                if (db.tokenToString(attribute().name()) == (inputOutput(isOutput) + ":" + prependStr + messageField.name))
                {
                    status = attribute().typeName() == messageField.ognType;
                    break;
                }
            }
            if (!status)
            {
                return false;
            }
        }
    }
    else
    {
        auto dynamicAttributes = db.getDynamicInputs();
        if (dynamicAttributes.size() != messageFields.size() && clearExistingAttrs)
        {
            return false;
        }
        // Check for attribute name and type
        for (auto const& messageField : messageFields)
        {
            bool status = false;
            for (auto const& attribute : dynamicAttributes)
            {
                if (db.tokenToString(attribute().name()) == (inputOutput(isOutput) + ":" + prependStr + messageField.name))
                {
                    status = attribute().typeName() == messageField.ognType;
                    break;
                }
            }
            if (!status)
            {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief Creates OmniGraph attributes for message fields
 * @details
 * Creates dynamic attributes on an OmniGraph node to match the fields of a ROS 2 message.
 * Optionally clears existing attributes before creating new ones.
 *
 * @tparam OgnROS2DatabaseDerivedType Database type for OmniGraph-ROS2 integration
 * @tparam isOutput Whether to create output (true) or input (false) attributes
 * @tparam clearExistingAttrs Whether to clear existing attributes before creating new ones
 * @param[in] db Database instance
 * @param[in] nodeObj Node object to modify
 * @param[in] messageFields Vector of message fields to create attributes for
 * @param[in] prependStr Optional prefix for attribute names
 * @param[in] messageType Full message type name (for logging)
 * @return bool True if all attributes were successfully created
 */
template <typename OgnROS2DatabaseDerivedType, bool isOutput, bool clearExistingAttrs>
inline bool createOgAttributesForFields(OgnROS2DatabaseDerivedType& db,
                                        const NodeObj& nodeObj,
                                        std::vector<DynamicMessageField> messageFields,
                                        const std::string prependStr,
                                        std::string messageType)
{
    // Return if all message fields have a corresponding OGN attribute
    if (findMatchingAttribute<OgnROS2DatabaseDerivedType, isOutput, clearExistingAttrs>(
            db, nodeObj, messageFields, prependStr))
    {
        return true;
    }

    if (clearExistingAttrs)
    {
        db.logWarning("Removing existing %s attributes ", inputOutput(isOutput).c_str());
        // Remove existing dynamic attributes and add new ones based the message fields
        bool status = removeDynamicAttributes<!isOutput, isOutput>(nodeObj);
        if (!status)
        {
            db.logWarning("Unable to remove existing attributes from the node");
            return false;
        }
    }

    auto attrFlag = isOutput ? AttributePortType::kAttributePortType_Output : AttributePortType::kAttributePortType_Input;
    for (auto const& messageField : messageFields)
    {
        bool status = nodeObj.iNode->createAttribute(
            nodeObj, (inputOutput(isOutput) + ":" + prependStr + messageField.name).c_str(),
            db.typeFromName(db.stringToToken(messageField.ognType.c_str())), nullptr, nullptr, attrFlag,
            ExtendedAttributeType::kExtendedAttributeType_Regular, nullptr);
        if (!status)
        {
            CARB_LOG_ERROR(
                "Unable to create attribute %s of type %s", messageField.name.c_str(), messageField.ognType.c_str());
            removeDynamicAttributes<!isOutput && clearExistingAttrs, isOutput && clearExistingAttrs>(nodeObj);
            return false;
        }
    }
    return true;
}

/**
 * @brief Creates OmniGraph attributes for a ROS 2 message
 * @details
 * Creates dynamic attributes on an OmniGraph node to match the fields of a ROS 2 message.
 * This is a higher-level function that validates the message type, obtains its fields,
 * and calls createOgAttributesForFields to create the node attributes.
 *
 * @tparam OgnROS2DatabaseDerivedType Database type for OmniGraph-ROS2 integration
 * @tparam isOutput Whether to create output (true) or input (false) attributes
 * @tparam clearExistingAttrs Whether to clear existing attributes before creating new ones
 * @param[in] db Database instance
 * @param[in] nodeObj Node object to modify
 * @param[in] messagePackage ROS 2 package containing the message definition
 * @param[in] messageSubfolder Subfolder within the package (e.g., "msg", "srv")
 * @param[in] messageName Name of the message type
 * @param[in] message Shared pointer to the message instance
 * @param[in] prependStr Optional prefix for attribute names
 * @return bool True if all attributes were successfully created
 */
template <typename OgnROS2DatabaseDerivedType, bool isOutput, bool clearExistingAttrs = true>
inline bool createOgAttributesForMessage(OgnROS2DatabaseDerivedType& db,
                                         const NodeObj& nodeObj,
                                         const std::string& messagePackage,
                                         const std::string& messageSubfolder,
                                         const std::string& messageName,
                                         std::shared_ptr<Ros2Message> message,
                                         const std::string prependStr)
{
    std::string messageType = messagePackage + "/" + messageSubfolder + "/" + messageName;
    // Naive check on inputs
    if (messagePackage.empty() || messageSubfolder.empty() || messageName.empty())
    {
        return false;
    }
    // Create message
    auto messageFields = std::static_pointer_cast<Ros2DynamicMessage>(message)->getMessageFields();
    bool status = std::static_pointer_cast<Ros2DynamicMessage>(message)->isValid();
    if (!status)
    {
        CARB_LOG_ERROR("%s does not exist or is not available in the ROS 2 environment", messageType.c_str());
        message.reset();
        return false;
    }

    return createOgAttributesForFields<OgnROS2DatabaseDerivedType, isOutput, clearExistingAttrs>(
        db, nodeObj, messageFields, prependStr, messageType);
}

/**
 * @brief Reads attribute data from an OmniGraph node and writes it to a ROS 2 message
 * @details
 * Transfers data from OmniGraph node attributes to a ROS 2 message. This function handles
 * various data types and their conversion between OmniGraph and ROS 2 representations.
 * It supports both scalar and array attributes of different primitive types.
 *
 * @param[in] db OmniGraph database instance
 * @param[in,out] message ROS 2 message to be populated with node data
 * @param[in] prependStr Prefix for attribute names
 * @param[in] isOutput Whether to read from output (true) or input (false) attributes
 * @return bool True if data was successfully read from node attributes and written to message
 */
inline bool writeMessageDataFromNode(OmniGraphDatabase& db,
                                     std::shared_ptr<Ros2Message> message,
                                     std::string prependStr,
                                     bool isOutput)
{
    auto messageFields = std::static_pointer_cast<Ros2DynamicMessage>(message)->getMessageFields();
    std::vector<std::shared_ptr<void>> messageData;
    for (size_t i = 0; i < messageFields.size(); ++i)
    {
        auto messageField = messageFields.at(i);
        switch (messageField.dataType)
        {
        case omni::fabric::BaseDataType::eBool:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<bool*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<bool> data(inputSize);
                if (checkCondition(inputValue, "Unable to read bool array"))
                {
                    // std::vector<bool> is a specialization that has no ::data
                    for (size_t j = 0; j < data.size(); ++j)
                    {
                        data[j] = *(*inputValue + j);
                    }
                }
                messageData.push_back(std::make_shared<std::vector<bool>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<bool>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read bool value"))
                {
                    messageData.push_back(std::make_shared<bool>(inputValue ? *inputValue : false));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUChar:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<uint8_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<uint8_t> data(inputSize);
                if (checkCondition(inputValue, "Unable to read uchar array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(uint8_t));
                }
                messageData.push_back(std::make_shared<std::vector<uint8_t>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<uint8_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read uchar value"))
                {
                    messageData.push_back(std::make_shared<uint8_t>(inputValue ? *inputValue : 0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eInt:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<int32_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<int32_t> data(inputSize);
                if (checkCondition(inputValue, "Unable to read int32 array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(int32_t));
                }
                messageData.push_back(std::make_shared<std::vector<int32_t>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<int32_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read int32 value"))
                {
                    messageData.push_back(std::make_shared<int32_t>(inputValue ? *inputValue : 0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUInt:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<uint32_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<uint32_t> data(inputSize);
                if (checkCondition(inputValue, "Unable to read uint32 array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(uint32_t));
                }
                messageData.push_back(std::make_shared<std::vector<uint32_t>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<uint32_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read uint32 value"))
                {
                    messageData.push_back(std::make_shared<uint32_t>(inputValue ? *inputValue : 0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eInt64:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<int64_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<int64_t> data(inputSize);
                if (checkCondition(inputValue, "Unable to read int64 array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(int64_t));
                }
                messageData.push_back(std::make_shared<std::vector<int64_t>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<int64_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read int64 value"))
                {
                    messageData.push_back(std::make_shared<int64_t>(inputValue ? *inputValue : 0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUInt64:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<uint64_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<uint64_t> data(inputSize);
                if (checkCondition(inputValue, "Unable to read uint64 array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(uint64_t));
                }
                messageData.push_back(std::make_shared<std::vector<uint64_t>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<uint64_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read uint64 value"))
                {
                    messageData.push_back(std::make_shared<uint64_t>(inputValue ? *inputValue : 0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eFloat:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<float*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<float> data(inputSize);
                if (checkCondition(inputValue, "Unable to read float array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(float));
                }
                messageData.push_back(std::make_shared<std::vector<float>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<float>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read float value"))
                {
                    messageData.push_back(std::make_shared<float>(inputValue ? *inputValue : 0.0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eDouble:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<double*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<double> data(inputSize);
                if (checkCondition(inputValue, "Unable to read double array"))
                {
                    std::memcpy(data.data(), *inputValue, inputSize * sizeof(double));
                }
                messageData.push_back(std::make_shared<std::vector<double>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<double>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(inputValue, "Unable to read double value"))
                {
                    messageData.push_back(std::make_shared<double>(inputValue ? *inputValue : 0.0));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eToken:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<NameToken*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<std::string> data(inputSize);
                if (checkCondition(inputValue, "Unable to read token array"))
                {
                    for (size_t j = 0; j < data.size(); ++j)
                    {
                        data[j] = db.tokenToString(*(*inputValue + j));
                    }
                }
                messageData.push_back(std::make_shared<std::vector<std::string>>(data));
            }
            else
            {
                auto inputValue = getAttributeReadableData<NameToken>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                std::string str = inputValue ? db.tokenToString(*inputValue) : "";
                if (checkCondition(inputValue, "Unable to read token value"))
                {
                    messageData.push_back(std::make_shared<std::string>(str));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUnknown:
        {
            if (messageField.isArray)
            {
                size_t inputSize;
                auto inputValue = getAttributeReadableArrayData<NameToken*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, inputSize);
                std::vector<nlohmann::json> data(inputSize);
                if (checkCondition(inputValue, "Unable to read message array"))
                {
                    for (size_t j = 0; j < data.size(); ++j)
                    {
                        data[j] = db.tokenToString(*(*inputValue + j));
                    }
                }
                messageData.push_back(std::make_shared<std::vector<nlohmann::json>>(data));
            }
            break;
        }

        default:
        {
            CARB_LOG_ERROR("writeMessageDataFromNode data type %d didn't match any of the implemented type.",
                           int(messageField.dataType));
            return false;
            break;
        }
        }
    }

    std::static_pointer_cast<Ros2DynamicMessage>(message)->writeData(messageData, true);
    return true;
}

/**
 * @brief Reads data from a ROS 2 message and writes it to OmniGraph node attributes
 * @details
 * Transfers data from a ROS 2 message to OmniGraph node attributes. This function handles
 * various data types and their conversion between ROS 2 and OmniGraph representations.
 * It supports both scalar and array values of different primitive types.
 *
 * @param[in] db OmniGraph database instance
 * @param[in] message ROS 2 message to read data from
 * @param[in] prependStr Prefix for attribute names
 * @param[in] isOutput Whether to write to output (true) or input (false) attributes
 * @return bool True if data was successfully read from message and written to node attributes
 */
inline bool writeNodeAttributeFromMessage(OmniGraphDatabase& db,
                                          std::shared_ptr<Ros2Message> message,
                                          std::string prependStr,
                                          bool isOutput)
{
    const std::vector<std::shared_ptr<void>> messageData =
        std::static_pointer_cast<Ros2DynamicMessage>(message)->readData(true);
    auto messageFields = std::static_pointer_cast<Ros2DynamicMessage>(message)->getMessageFields();
    CARB_ASSERT(messageFields.size() == messageData.size());

    for (size_t i = 0; i < messageFields.size(); ++i)
    {
        if (!messageData.at(i))
        {
            continue;
        }
        auto messageField = messageFields.at(i);
        switch (messageField.dataType)
        {
        case omni::fabric::BaseDataType::eBool:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<bool>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<bool*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write bool array"))
                {
                    // std::vector<bool> is a specialization that has no ::data
                    for (size_t j = 0; j < data.size(); ++j)
                    {
                        *((*outputValue) + j) = data.at(j);
                    }
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<bool>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write bool value"))
                {
                    *outputValue = *std::static_pointer_cast<bool>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUChar:
        {
            if (messageField.isArray)
            {

                auto data = *std::static_pointer_cast<std::vector<uint8_t>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<uint8_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write uchar array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(uint8_t));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<uint8_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write uchar value"))
                {
                    *outputValue = *std::static_pointer_cast<uint8_t>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eInt:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<int32_t>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<int32_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write int array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(int32_t));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<int32_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write int value"))
                {
                    *outputValue = *std::static_pointer_cast<int32_t>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUInt:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<uint32_t>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<uint32_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "unable to write uint array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(uint32_t));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<uint32_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "unable to write uint value"))
                {
                    *outputValue = *std::static_pointer_cast<uint32_t>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eInt64:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<int64_t>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<int64_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write int64 array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(int64_t));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<int64_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write int64 value"))
                {
                    *outputValue = *std::static_pointer_cast<int64_t>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUInt64:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<uint64_t>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<uint64_t*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write uint64 array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(uint64_t));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<uint64_t>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write uint64 value"))
                {
                    *outputValue = *std::static_pointer_cast<uint64_t>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eFloat:
        {
            if (messageField.isArray)
            {
                auto data = *std::static_pointer_cast<std::vector<float>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<float*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write float array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(float));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<float>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write float value"))
                {
                    *outputValue = *std::static_pointer_cast<float>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eDouble:
        {
            if (messageField.isArray)
            {

                auto data = *std::static_pointer_cast<std::vector<double>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<double*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, data.size());
                if (checkCondition(outputValue, "Unable to write double array"))
                {
                    std::memcpy(*outputValue, data.data(), data.size() * sizeof(double));
                }
            }
            else
            {
                auto outputValue = getAttributeWritableData<double>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write double value"))
                {
                    *outputValue = *std::static_pointer_cast<double>(messageData.at(i));
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eToken:
        {
            if (messageField.isArray)
            {
                auto stringValues = *std::static_pointer_cast<std::vector<std::string>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<NameToken*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, stringValues.size());
                if (checkCondition(outputValue, "Unable to write token array"))
                {
                    for (size_t j = 0; j < stringValues.size(); ++j)
                    {
                        *((*outputValue) + j) = db.stringToToken(stringValues.at(j).c_str());
                    }
                }
            }
            else
            {
                auto stringValue = *std::static_pointer_cast<std::string>(messageData.at(i));
                auto outputValue = getAttributeWritableData<NameToken>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name);
                if (checkCondition(outputValue, "Unable to write token value"))
                {
                    *outputValue = db.stringToToken(stringValue.c_str());
                }
            }
            break;
        }
        case omni::fabric::BaseDataType::eUnknown:
        {
            if (messageField.isArray)
            {
                auto array = *std::static_pointer_cast<const std::vector<nlohmann::json>>(messageData.at(i));
                auto outputValue = getAttributeWritableArrayData<NameToken*>(
                    db.abi_node(), inputOutput(isOutput) + ":" + prependStr + messageField.name, array.size());
                if (checkCondition(outputValue, "Unable to write message array"))
                {
                    for (size_t j = 0; j < array.size(); ++j)
                    {
                        *((*outputValue) + j) = db.stringToToken(array.at(j).dump().c_str());
                    }
                }
            }
            break;
        }
        default:
        {
            CARB_LOG_ERROR("writeNodeAttributeFromMessage data type %d didn't match any of the implemented type.",
                           int(messageField.dataType));
            return false;
            break;
        }
        }
    }
    return true;
}

} // namespace omnigraph_utils
} // namespace ros2
} // namespace isaacsim
