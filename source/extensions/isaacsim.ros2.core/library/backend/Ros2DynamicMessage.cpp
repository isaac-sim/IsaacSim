// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Ros2Impl.h"

#include <carb/logging/Log.h>

#include <nlohmann/json.hpp>

#include <iomanip>

namespace isaacsim
{
namespace ros2
{
namespace core
{

Ros2DynamicMessageImpl::Ros2DynamicMessageImpl(const std::string& pkgName,
                                               const std::string& msgSubfolder,
                                               const std::string& msgName,
                                               BackendMessageType messageType)
    : Ros2MessageInterfaceImpl(pkgName, msgSubfolder, msgName, messageType)
{
    // create message
    m_msg = create();
    // clear message fields and containers
    m_messagesFields.clear();
    m_messageVectorRosContainer.clear();
    m_messageVectorOgnContainer.clear();
    m_messageJsonContainer = nlohmann::json::object();
    // get message fields
    const void* members = Ros2DynamicMessageImpl::getIntrospectionMembers();
    if (members)
    {
        Ros2DynamicMessageImpl::parseMessageFields("", members);
    }
}

Ros2DynamicMessageImpl::~Ros2DynamicMessageImpl()
{
    if (m_msg)
    {
        destroy(static_cast<void*>(m_msg));
    }
}

const void* Ros2DynamicMessageImpl::getTypeSupportHandle()
{
    return getTypeSupportHandleDynamic();
}

const nlohmann::json& Ros2DynamicMessageImpl::readData()
{
    const void* members = getIntrospectionMembers();
    m_messageJsonContainer.clear();
    if (members)
    {
        getMessageValues(members, reinterpret_cast<uint8_t*>(m_msg), m_messageJsonContainer);
    }
    return m_messageJsonContainer;
}

const std::vector<std::shared_ptr<void>>& Ros2DynamicMessageImpl::readData(bool asOgnType)
{
    const void* members = getIntrospectionMembers();
    if (members)
    {
        size_t index = 0;
        getMessageValues(members, reinterpret_cast<uint8_t*>(m_msg),
                         asOgnType ? m_messageVectorOgnContainer : m_messageVectorRosContainer, index, asOgnType);
    }
    return asOgnType ? m_messageVectorOgnContainer : m_messageVectorRosContainer;
}

void Ros2DynamicMessageImpl::writeData(const nlohmann::json& data)
{
    const void* members = getIntrospectionMembers();
    if (members)
    {
        setMessageValues(members, reinterpret_cast<uint8_t*>(m_msg), data);
    }
}

void Ros2DynamicMessageImpl::writeData(const std::vector<std::shared_ptr<void>>& data, bool fromOgnType)
{
    const void* members = getIntrospectionMembers();
    if (members)
    {
        size_t index = 0;
        setMessageValues(members, reinterpret_cast<uint8_t*>(m_msg), data, index, fromOgnType);
    }
}

const void* Ros2DynamicMessageImpl::getIntrospectionMembers()
{
    void* typeSupportHandle = getTypeSupportIntrospectionHandleDynamic();
    if (typeSupportHandle)
    {
        switch (m_msgType)
        {
        case BackendMessageType::eMessage:
        case BackendMessageType::eSendGoalRequest:
        case BackendMessageType::eSendGoalResponse:
        case BackendMessageType::eFeedbackMessage:
        case BackendMessageType::eGetResultRequest:
        case BackendMessageType::eGetResultResponse:
            return static_cast<const rosidl_message_type_support_t*>(typeSupportHandle)->data;
        case BackendMessageType::eRequest:
        {
            auto typeSupportData = static_cast<const rosidl_service_type_support_t*>(typeSupportHandle)->data;
            return reinterpret_cast<const rosidl_typesupport_introspection_c__ServiceMembers*>(typeSupportData)
                ->request_members_;
        }
        case BackendMessageType::eResponse:
        {
            auto typeSupportData = static_cast<const rosidl_service_type_support_t*>(typeSupportHandle)->data;
            return reinterpret_cast<const rosidl_typesupport_introspection_c__ServiceMembers*>(typeSupportData)
                ->response_members_;
        }
        default:
            break;
        }
    }
    return nullptr;
}

std::string Ros2DynamicMessageImpl::generateSummary(bool print)
{
    // backend type
    std::unordered_map<BackendMessageType, std::string> backendType = {
        { BackendMessageType::eMessage, "topic | message" },
        { BackendMessageType::eRequest, "service | request" },
        { BackendMessageType::eResponse, "service | response" },
        { BackendMessageType::eGoal, "action | goal" },
        { BackendMessageType::eResult, "action | result" },
        { BackendMessageType::eFeedback, "action | feedback" },
        { BackendMessageType::eSendGoalRequest, "action | goal | request" },
        { BackendMessageType::eSendGoalResponse, "action | goal | response" },
        { BackendMessageType::eFeedbackMessage, "action | feedback | message" },
        { BackendMessageType::eGetResultRequest, "action | result | request" },
        { BackendMessageType::eGetResultResponse, "action | result | response" },
    };
    // fabric type
    std::unordered_map<omni::fabric::BaseDataType, std::string> fabricType = {
        { omni::fabric::BaseDataType::eUnknown, "eUnknown (nlohmann::json)" },
        { omni::fabric::BaseDataType::eBool, "eBool (bool)" },
        { omni::fabric::BaseDataType::eUChar, "eUChar (uint8_t)" },
        { omni::fabric::BaseDataType::eInt, "eInt (int32_t)" },
        { omni::fabric::BaseDataType::eUInt, "eUInt (uint32_t)" },
        { omni::fabric::BaseDataType::eInt64, "eInt64 (int64_t)" },
        { omni::fabric::BaseDataType::eUInt64, "eUInt64 (uint64_t)" },
        { omni::fabric::BaseDataType::eDouble, "eDouble (double)" },
        { omni::fabric::BaseDataType::eFloat, "eFloat (float)" },
        { omni::fabric::BaseDataType::eToken, "eToken (std::string)" },
    };
    // ROS type
    std::unordered_map<uint8_t, std::string> rosType = {
        { rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT, "FLOAT (float)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE, "DOUBLE (double)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE, "LONG_DOUBLE (long double)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_CHAR, "CHAR (uint8_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR, "WCHAR (uint16_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN, "BOOLEAN (bool)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_OCTET, "OCTET (uint8_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_UINT8, "UINT8 (uint8_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_INT8, "INT8 (int8_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_UINT16, "UINT16 (uint16_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_INT16, "INT16 (int16_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_UINT32, "UINT32 (uint32_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_INT32, "INT32 (int32_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_UINT64, "UINT64 (uint64_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_INT64, "INT64 (int64_t)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_STRING, "STRING (std::string)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING, "WSTRING (std::string)" },
        { rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE, "MESSAGE (nlohmann::json)" },
    };
    std::ostringstream stream;
    stream << std::endl;
    stream << "Message: " << m_pkgName << "/" << m_msgSubfolder << "/" << m_msgName;
    stream << " (" << backendType[m_msgType] << ")" << std::endl;
    stream << "Idx Array ROS type                  OGN type                  Name" << std::endl;
    stream << "=== ===== ========================= ========================= ====" << std::endl;
    for (size_t i = 0; i < m_messagesFields.size(); ++i)
    {
        auto field = m_messagesFields.at(i);
        stream << std::left << std::setw(4) << std::setfill(' ') << i;
        stream << std::left << std::setw(6) << std::setfill(' ') << (field.isArray ? "yes" : "no");
        stream << std::left << std::setw(26) << std::setfill(' ') << rosType[field.rosType];
        stream << std::left << std::setw(26) << std::setfill(' ') << fabricType[field.dataType];
        stream << field.name << std::endl;
    }
    stream << std::endl;
    if (print)
    {
        std::cout << stream.str();
    }
    return stream.str();
}

void Ros2DynamicMessageImpl::parseMessageFields(const std::string& parentName, const void* members)
{
    auto messageMembers = reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(members);
    for (size_t i = 0; i < messageMembers->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_c__MessageMember* member = messageMembers->members_ + i;
        // auxiliary data to define message fields
        std::string name = (parentName.length() ? parentName + ":" : "") + member->name_;
        std::string type;
        omni::fabric::BaseDataType dataType = omni::fabric::BaseDataType::eUnknown;
        // auxiliary data to initialize message containers
        std::shared_ptr<void> rosValue = nullptr;
        std::shared_ptr<void> ognValue = nullptr;
        // parse message fields
        switch (member->type_id_)
        {
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
            type = member->is_array_ ? "float[]" : "float";
            dataType = omni::fabric::BaseDataType::eFloat;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<float>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<float>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
            type = member->is_array_ ? "double[]" : "double";
            dataType = omni::fabric::BaseDataType::eDouble;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<double>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<double>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
            type = member->is_array_ ? "double[]" : "double";
            dataType = omni::fabric::BaseDataType::eDouble;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<long double>>();
                ognValue = std::make_shared<std::vector<double>>();
            }
            else
            {
                rosValue = std::make_shared<long double>();
                ognValue = std::make_shared<double>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
            type = member->is_array_ ? "uchar[]" : "uchar";
            dataType = omni::fabric::BaseDataType::eUChar;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<uint8_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<uint8_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
            type = member->is_array_ ? "uint[]" : "uint";
            dataType = omni::fabric::BaseDataType::eUInt;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<uint16_t>>();
                ognValue = std::make_shared<std::vector<uint32_t>>();
            }
            else
            {
                rosValue = std::make_shared<uint16_t>();
                ognValue = std::make_shared<uint32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
            type = member->is_array_ ? "bool[]" : "bool";
            dataType = omni::fabric::BaseDataType::eBool;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<bool>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<bool>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
            type = member->is_array_ ? "uchar[]" : "uchar";
            dataType = omni::fabric::BaseDataType::eUChar;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<uint8_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<uint8_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
            type = member->is_array_ ? "uint[]" : "uint";
            dataType = omni::fabric::BaseDataType::eUInt;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<uint8_t>>();
                ognValue = std::make_shared<std::vector<uint32_t>>();
            }
            else
            {
                rosValue = std::make_shared<uint8_t>();
                ognValue = std::make_shared<uint32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
            type = member->is_array_ ? "int[]" : "int";
            dataType = omni::fabric::BaseDataType::eInt;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<int8_t>>();
                ognValue = std::make_shared<std::vector<int32_t>>();
            }
            else
            {
                rosValue = std::make_shared<int8_t>();
                ognValue = std::make_shared<int32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
            type = member->is_array_ ? "uint[]" : "uint";
            dataType = omni::fabric::BaseDataType::eUInt;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<uint16_t>>();
                ognValue = std::make_shared<std::vector<uint32_t>>();
            }
            else
            {
                rosValue = std::make_shared<uint16_t>();
                ognValue = std::make_shared<uint32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
            type = member->is_array_ ? "int[]" : "int";
            dataType = omni::fabric::BaseDataType::eInt;
            if (member->is_array_)
            {
                rosValue = std::make_shared<std::vector<int16_t>>();
                ognValue = std::make_shared<std::vector<int32_t>>();
            }
            else
            {
                rosValue = std::make_shared<int16_t>();
                ognValue = std::make_shared<int32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
            type = member->is_array_ ? "uint[]" : "uint";
            dataType = omni::fabric::BaseDataType::eUInt;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<uint32_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<uint32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
            type = member->is_array_ ? "int[]" : "int";
            dataType = omni::fabric::BaseDataType::eInt;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<int32_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<int32_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
            type = member->is_array_ ? "uint64[]" : "uint64";
            dataType = omni::fabric::BaseDataType::eUInt64;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<uint64_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<uint64_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
            type = member->is_array_ ? "int64[]" : "int64";
            dataType = omni::fabric::BaseDataType::eInt64;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<int64_t>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<int64_t>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
            type = member->is_array_ ? "token[]" : "token";
            dataType = omni::fabric::BaseDataType::eToken;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<std::string>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<std::string>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
            type = member->is_array_ ? "token[]" : "token";
            dataType = omni::fabric::BaseDataType::eToken;
            if (member->is_array_)
            {
                rosValue = ognValue = std::make_shared<std::vector<std::string>>();
            }
            else
            {
                rosValue = ognValue = std::make_shared<std::string>();
            }
            break;
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
            if (member->is_array_)
            {
                type = "token[]";
                dataType = omni::fabric::BaseDataType::eUnknown;
                rosValue = ognValue = std::make_shared<std::vector<nlohmann::json>>();
                break;
            }
            // unroll only if not an array
            parseMessageFields(name, member->members_->data);
            continue;
        default:
            break;
        }
        // preallocate message containers
        m_messageVectorRosContainer.push_back(rosValue);
        m_messageVectorOgnContainer.push_back(ognValue);
        // define message field
        m_messagesFields.push_back({ name, member->type_id_, member->is_array_, type, dataType });
    }
}


void Ros2DynamicMessageImpl::getArrayEmbeddedMessage(const rosidl_typesupport_introspection_c__MessageMember* member,
                                                     uint8_t* data,
                                                     nlohmann::json& array)
{
    auto embeddedMembers =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member->members_->data);
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        uint8_t* memberData;
        size_t arrayLength = 0;
        memcpy(&memberData, data, sizeof(memberData));
        memcpy(&arrayLength, data + sizeof(memberData), sizeof(arrayLength)); // read length
        for (size_t i = 0; i < arrayLength; ++i)
        {
            auto jsonObj = nlohmann::json::object();
            getMessageValues(embeddedMembers, memberData + i * embeddedMembers->size_of_, jsonObj);
            array.push_back(jsonObj);
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            auto jsonObj = nlohmann::json::object();
            getMessageValues(embeddedMembers, &data[i * embeddedMembers->size_of_], jsonObj);
            array.push_back(jsonObj);
        }
    }
}

void Ros2DynamicMessageImpl::setArrayEmbeddedMessage(const rosidl_typesupport_introspection_c__MessageMember* member,
                                                     uint8_t* data,
                                                     const nlohmann::json& array)
{
    auto embeddedMembers =
        reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member->members_->data);
    // non-fixed size array
    // TODO: check for sequence capacity
    if (member->is_upper_bound_ || !member->array_size_)
    {
        member->resize_function(data, array.size());
        for (size_t i = 0; i < array.size(); ++i)
        {
            setMessageValues(embeddedMembers, reinterpret_cast<uint8_t*>(member->get_function(data, i)), array.at(i));
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_ && i < array.size(); ++i)
        {
            setMessageValues(embeddedMembers, &data[i * embeddedMembers->size_of_], array.at(i));
        }
    }
}

template <typename ArrayType, typename RosType, typename OgnType>
void Ros2DynamicMessageImpl::_getArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       std::shared_ptr<void>& arrayPtr,
                                       bool asOgnType)
{
    // OGN data type array
    if (asOgnType)
    {
        auto ognArray = std::static_pointer_cast<std::vector<OgnType>>(arrayPtr);
        ognArray->clear();
        // non-fixed size array
        if (member->is_upper_bound_ || !member->array_size_)
        {
            const ArrayType* source = reinterpret_cast<const ArrayType*>(data);
            ognArray->reserve(source->size);
            for (size_t i = 0; i < source->size; ++i)
            {
                ognArray->push_back(static_cast<OgnType>(*reinterpret_cast<const RosType*>(&source->data[i])));
            }
        }
        // fixed size array
        else
        {
            ognArray->reserve(member->array_size_);
            for (size_t i = 0; i < member->array_size_; ++i)
            {
                ognArray->push_back(static_cast<OgnType>(*reinterpret_cast<const RosType*>(&data[i * sizeof(RosType)])));
            }
        }
    }
    // ROS data type array
    else
    {
        auto rosArray = std::static_pointer_cast<std::vector<RosType>>(arrayPtr);
        rosArray->clear();
        // non-fixed size array
        if (member->is_upper_bound_ || !member->array_size_)
        {
            const ArrayType* source = reinterpret_cast<const ArrayType*>(data);
            rosArray->reserve(source->size);
            for (size_t i = 0; i < source->size; ++i)
            {
                rosArray->push_back(*reinterpret_cast<const RosType*>(&source->data[i]));
            }
        }
        // fixed size array
        else
        {
            rosArray->reserve(member->array_size_);
            for (size_t i = 0; i < member->array_size_; ++i)
            {
                rosArray->push_back(*reinterpret_cast<const RosType*>(&data[i * sizeof(RosType)]));
            }
        }
    }
}

template <typename ArrayType, typename RosType>
void Ros2DynamicMessageImpl::_getArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       nlohmann::json& array)
{
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        const ArrayType* source = reinterpret_cast<const ArrayType*>(data);
        for (size_t i = 0; i < source->size; ++i)
        {
            array.push_back(*reinterpret_cast<const RosType*>(&source->data[i]));
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            array.push_back(*reinterpret_cast<const RosType*>(&data[i * sizeof(RosType)]));
        }
    }
}

template <typename ArrayType, typename RosType>
void Ros2DynamicMessageImpl::_getArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       std::vector<RosType>& array)
{
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        const ArrayType* source = reinterpret_cast<const ArrayType*>(data);
        for (size_t i = 0; i < source->size; ++i)
        {
            const uint8_t* sourceData = reinterpret_cast<const uint8_t*>(&source->data[i]);
            array.push_back(*reinterpret_cast<const RosType*>(sourceData));
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            array.push_back(*reinterpret_cast<const RosType*>(&data[i * sizeof(RosType)]));
        }
    }
}

template <typename ArrayType, auto ArrayInit, auto ArrayFini, typename RosType>
void Ros2DynamicMessageImpl::_setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       const nlohmann::json& value)
{
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        ArrayType* dest = reinterpret_cast<ArrayType*>(data);
        // Finalize any prior allocation first: rosidl __init on an already-allocated
        // sequence overwrites the data pointer and leaks the previous buffer (including
        // inner elements). The same message is reused on every publish tick.
        if (dest->data != nullptr)
        {
            ArrayFini(dest);
        }
        ArrayInit(dest, value.size());
        for (size_t i = 0; i < dest->size; ++i)
        {
            *reinterpret_cast<RosType*>(&dest->data[i]) = value.at(i).get<RosType>();
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            *reinterpret_cast<RosType*>(&data[i * sizeof(RosType)]) =
                i < value.size() ? value.at(i).get<RosType>() : RosType{};
        }
    }
}

template <typename ArrayType, auto ArrayInit, auto ArrayFini, typename RosType, typename OgnType>
void Ros2DynamicMessageImpl::_setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       const std::shared_ptr<void>& valuePtr,
                                       bool fromOgnType)
{
    // OGN data type array
    if (fromOgnType)
    {
        auto ognArray = std::static_pointer_cast<std::vector<OgnType>>(valuePtr);
        // non-fixed size array
        if (member->is_upper_bound_ || !member->array_size_)
        {
            ArrayType* dest = reinterpret_cast<ArrayType*>(data);
            // See note below: finalize before re-init to avoid leaking the prior buffer.
            if (dest->data != nullptr)
            {
                ArrayFini(dest);
            }
            ArrayInit(dest, ognArray->size());
            for (size_t i = 0; i < dest->size; ++i)
            {
                *reinterpret_cast<RosType*>(&dest->data[i]) = static_cast<RosType>(ognArray->at(i));
            }
        }
        // fixed size array
        else
        {
            for (size_t i = 0; i < member->array_size_; ++i)
            {
                *reinterpret_cast<RosType*>(&data[i * sizeof(RosType)]) =
                    static_cast<RosType>(i < ognArray->size() ? ognArray->at(i) : OgnType{});
            }
        }
        return;
    }
    // ROS data type array
    auto rosArray = std::static_pointer_cast<std::vector<RosType>>(valuePtr);
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        ArrayType* dest = reinterpret_cast<ArrayType*>(data);
        // Finalize before re-init to avoid leaking the previously allocated buffer.
        if (dest->data != nullptr)
        {
            ArrayFini(dest);
        }
        ArrayInit(dest, rosArray->size());
        for (size_t i = 0; i < dest->size; ++i)
        {
            *reinterpret_cast<RosType*>(&dest->data[i]) = rosArray->at(i);
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            *reinterpret_cast<RosType*>(&data[i * sizeof(RosType)]) = i < rosArray->size() ? rosArray->at(i) : RosType{};
        }
    }
}

template <typename ArrayType, auto ArrayInit, auto ArrayFini, typename RosType>
void Ros2DynamicMessageImpl::_setArray(const rosidl_typesupport_introspection_c__MessageMember* member,
                                       uint8_t* data,
                                       const std::vector<RosType>& array)
{
    // non-fixed size array
    if (member->is_upper_bound_ || !member->array_size_)
    {
        ArrayType* dest = reinterpret_cast<ArrayType*>(data);
        // Finalize before re-init to avoid leaking the previously allocated buffer.
        if (dest->data != nullptr)
        {
            ArrayFini(dest);
        }
        ArrayInit(dest, array.size());
        for (size_t i = 0; i < dest->size; ++i)
        {
            *reinterpret_cast<RosType*>(&dest->data[i]) = array.at(i);
        }
    }
    // fixed size array
    else
    {
        for (size_t i = 0; i < member->array_size_; ++i)
        {
            *reinterpret_cast<RosType*>(&data[i * sizeof(RosType)]) = i < array.size() ? array.at(i) : RosType{};
        }
    }
}

template <typename RosType, typename OgnType>
void Ros2DynamicMessageImpl::_getSingleValue(uint8_t* data, std::shared_ptr<void>& valuePtr, bool asOgnType)
{
    auto value = reinterpret_cast<const RosType*>(data);
    if (asOgnType)
    {
        *std::static_pointer_cast<OgnType>(valuePtr) = static_cast<OgnType>(*value);
    }
    else
    {
        *std::static_pointer_cast<RosType>(valuePtr) = *value;
    }
}

template <typename RosType, typename OgnType>
void Ros2DynamicMessageImpl::_setSingleValue(uint8_t* data, const std::shared_ptr<void>& valuePtr, bool fromOgnType)
{
    if (fromOgnType)
    {
        *reinterpret_cast<RosType*>(data) = static_cast<RosType>(*std::static_pointer_cast<const OgnType>(valuePtr));
    }
    else
    {
        *reinterpret_cast<RosType*>(data) = *std::static_pointer_cast<const RosType>(valuePtr);
    }
}


void Ros2DynamicMessageImpl::getMessageValues(const void* members, uint8_t* messageData, nlohmann::json& container)
{
    auto messageMembers = reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(members);
    for (size_t i = 0; i < messageMembers->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_c__MessageMember* member = messageMembers->members_ + i;
        auto data = &messageData[member->offset_];
        switch (member->type_id_)
        {
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__float__Sequence, float>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const float*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__double__Sequence, double>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const double*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__long_double__Sequence, long double>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const long double*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__char__Sequence, uint8_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint8_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__wchar__Sequence, uint16_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint16_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__boolean__Sequence, bool>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const bool*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__octet__Sequence, uint8_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint8_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__uint8__Sequence, uint8_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint8_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__int8__Sequence, int8_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const int8_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__uint16__Sequence, uint16_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint16_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__int16__Sequence, int16_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const int16_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__uint32__Sequence, uint32_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint32_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__int32__Sequence, int32_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const int32_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__uint64__Sequence, uint64_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const uint64_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                _getArray<rosidl_runtime_c__int64__Sequence, int64_t>(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = *reinterpret_cast<const int64_t*>(data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                std::vector<rosidl_runtime_c__String> array;
                _getArray<rosidl_runtime_c__String__Sequence, rosidl_runtime_c__String>(member, data, array);
                for (auto const& item : array)
                {
                    container[member->name_].push_back(std::string(item.data));
                }
            }
            else
            {
                auto value = reinterpret_cast<const rosidl_runtime_c__String*>(data);
                container[member->name_] = std::string(value->data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
        {
            // TODO: proccess WSTRING (no messages with 'path:*/msgs/*.msg "wstring"')
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
            }
            else
            {
                container[member->name_] = std::string();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
        {
            if (member->is_array_)
            {
                container[member->name_] = nlohmann::json::array();
                getArrayEmbeddedMessage(member, data, container[member->name_]);
            }
            else
            {
                container[member->name_] = nlohmann::json::object();
                getMessageValues(member->members_->data, data, container[member->name_]);
            }
            break;
        }
        default:
            break;
        }
    }
}

void Ros2DynamicMessageImpl::getMessageValues(const void* members,
                                              uint8_t* messageData,
                                              std::vector<std::shared_ptr<void>>& container,
                                              size_t& index,
                                              bool asOgnType)
{
    auto messageMembers = reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(members);
    for (size_t i = 0; i < messageMembers->member_count_; ++i)
    {
        auto valuePtr = container.at(index++);
        const rosidl_typesupport_introspection_c__MessageMember* member = messageMembers->members_ + i;
        auto data = &messageData[member->offset_];
        switch (member->type_id_)
        {
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__float__Sequence, float, float>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<float, float>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__double__Sequence, double, double>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<double, double>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__long_double__Sequence, long double, double>(
                    member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<long double, double>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__char__Sequence, uint8_t, uint8_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<uint8_t, uint8_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__wchar__Sequence, uint16_t, uint32_t>(member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<uint16_t, uint32_t>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__boolean__Sequence, bool, bool>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<bool, bool>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__octet__Sequence, uint8_t, uint8_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<uint8_t, uint8_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__uint8__Sequence, uint8_t, uint32_t>(member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<uint8_t, uint32_t>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__int8__Sequence, int8_t, int32_t>(member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<int8_t, int32_t>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__uint16__Sequence, uint16_t, uint32_t>(member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<uint16_t, uint32_t>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__int16__Sequence, int16_t, int32_t>(member, data, valuePtr, asOgnType);
            }
            else
            {
                _getSingleValue<int16_t, int32_t>(data, valuePtr, asOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__uint32__Sequence, uint32_t, uint32_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<uint32_t, uint32_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__int32__Sequence, int32_t, int32_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<int32_t, int32_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__uint64__Sequence, uint64_t, uint64_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<uint64_t, uint64_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
        {
            if (member->is_array_)
            {
                _getArray<rosidl_runtime_c__int64__Sequence, int64_t, int64_t>(member, data, valuePtr, false);
            }
            else
            {
                _getSingleValue<int64_t, int64_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
        {
            if (member->is_array_)
            {
                std::vector<rosidl_runtime_c__String> rosArray;
                _getArray<rosidl_runtime_c__String__Sequence, rosidl_runtime_c__String>(member, data, rosArray);
                auto array = std::static_pointer_cast<std::vector<std::string>>(valuePtr);
                array->clear();
                array->reserve(rosArray.size());
                std::transform(rosArray.begin(), rosArray.end(), std::back_inserter(*array),
                               [](const auto& item) { return std::string(item.data); });
            }
            else
            {
                auto value = reinterpret_cast<const rosidl_runtime_c__String*>(data);
                *std::static_pointer_cast<std::string>(valuePtr) = std::string(value->data);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
        {
            // TODO: proccess WSTRING (no messages with 'path:*/msgs/*.msg "wstring"')
            if (member->is_array_)
            {
                std::static_pointer_cast<std::vector<std::string>>(valuePtr)->clear();
            }
            else
            {
                *std::static_pointer_cast<std::string>(valuePtr) = std::string();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
        {
            if (member->is_array_)
            {
                auto jsonArray = nlohmann::json::array();
                getArrayEmbeddedMessage(member, data, jsonArray);
                std::static_pointer_cast<std::vector<nlohmann::json>>(valuePtr)->clear();
                for (size_t j = 0; j < jsonArray.size(); ++j)
                {
                    std::static_pointer_cast<std::vector<nlohmann::json>>(valuePtr)->push_back(jsonArray.at(j));
                }
            }
            else
            {
                getMessageValues(member->members_->data, data, container, --index, asOgnType);
                continue;
            }
            break;
        }
        default:
            break;
        }
    }
}


void Ros2DynamicMessageImpl::setMessageValues(const void* members, uint8_t* messageData, const nlohmann::json& container)
{
    if (!container.is_object())
    {
        return;
    }
    auto messageMembers = reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(members);
    for (size_t i = 0; i < messageMembers->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_c__MessageMember* member = messageMembers->members_ + i;
        auto data = &messageData[member->offset_];
        if (!container.contains(member->name_))
        {
            continue;
        }
        auto value = container[member->name_];
        switch (member->type_id_)
        {
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__float__Sequence, rosidl_runtime_c__float__Sequence__init,
                          rosidl_runtime_c__float__Sequence__fini, float>(member, data, value);
            }
            else
            {
                *reinterpret_cast<float*>(data) = value.get<float>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__double__Sequence, rosidl_runtime_c__double__Sequence__init,
                          rosidl_runtime_c__double__Sequence__fini, double>(member, data, value);
            }
            else
            {
                *reinterpret_cast<double*>(data) = value.get<double>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__long_double__Sequence, rosidl_runtime_c__long_double__Sequence__init,
                          rosidl_runtime_c__long_double__Sequence__fini, long double>(member, data, value);
            }
            else
            {
                *reinterpret_cast<long double*>(data) = value.get<long double>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__char__Sequence, rosidl_runtime_c__char__Sequence__init,
                          rosidl_runtime_c__char__Sequence__fini, uint8_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint8_t*>(data) = value.get<uint8_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__wchar__Sequence, rosidl_runtime_c__wchar__Sequence__init,
                          rosidl_runtime_c__wchar__Sequence__fini, uint16_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint16_t*>(data) = value.get<uint16_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__boolean__Sequence, rosidl_runtime_c__bool__Sequence__init,
                          rosidl_runtime_c__bool__Sequence__fini, bool>(member, data, value);
            }
            else
            {
                *reinterpret_cast<bool*>(data) = value.get<bool>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__octet__Sequence, rosidl_runtime_c__octet__Sequence__init,
                          rosidl_runtime_c__octet__Sequence__fini, uint8_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint8_t*>(data) = value.get<uint8_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint8__Sequence, rosidl_runtime_c__uint8__Sequence__init,
                          rosidl_runtime_c__uint8__Sequence__fini, uint8_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint8_t*>(data) = value.get<uint8_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int8__Sequence, rosidl_runtime_c__int8__Sequence__init,
                          rosidl_runtime_c__int8__Sequence__fini, int8_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<int8_t*>(data) = value.get<int8_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint16__Sequence, rosidl_runtime_c__uint16__Sequence__init,
                          rosidl_runtime_c__uint16__Sequence__fini, uint16_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint16_t*>(data) = value.get<uint16_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int16__Sequence, rosidl_runtime_c__int16__Sequence__init,
                          rosidl_runtime_c__int16__Sequence__fini, int16_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<int16_t*>(data) = value.get<int16_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint32__Sequence, rosidl_runtime_c__uint32__Sequence__init,
                          rosidl_runtime_c__uint32__Sequence__fini, uint32_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint32_t*>(data) = value.get<uint32_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int32__Sequence, rosidl_runtime_c__int32__Sequence__init,
                          rosidl_runtime_c__int32__Sequence__fini, int32_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<int32_t*>(data) = value.get<int32_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint64__Sequence, rosidl_runtime_c__uint64__Sequence__init,
                          rosidl_runtime_c__uint64__Sequence__fini, uint64_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<uint64_t*>(data) = value.get<uint64_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int64__Sequence, rosidl_runtime_c__int64__Sequence__init,
                          rosidl_runtime_c__int64__Sequence__fini, int64_t>(member, data, value);
            }
            else
            {
                *reinterpret_cast<int64_t*>(data) = value.get<int64_t>();
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
        {
            if (member->is_array_)
            {
                std::vector<std::string> array = value;
                std::vector<rosidl_runtime_c__String> rosArray(array.size());
                for (size_t j = 0; j < array.size(); ++j)
                {
                    rosidl_runtime_c__String__assign(&rosArray.at(j), array.at(j).c_str());
                }
                _setArray<rosidl_runtime_c__String__Sequence, rosidl_runtime_c__String__Sequence__init,
                          rosidl_runtime_c__String__Sequence__fini, rosidl_runtime_c__String>(member, data, rosArray);
            }
            else
            {
                rosidl_runtime_c__String__assign(
                    reinterpret_cast<rosidl_runtime_c__String*>(data), value.get<std::string>().c_str());
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
        {
            // TODO: proccess WSTRING (no messages with 'path:*/msgs/*.msg "wstring"')
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
        {
            if (member->is_array_)
            {
                setArrayEmbeddedMessage(member, data, value);
            }
            else
            {
                setMessageValues(member->members_->data, data, value);
            }
            break;
        }
        default:
            break;
        }
    }
}

void Ros2DynamicMessageImpl::setMessageValues(const void* members,
                                              uint8_t* messageData,
                                              const std::vector<std::shared_ptr<void>>& container,
                                              size_t& index,
                                              bool fromOgnType)
{
    auto messageMembers = reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(members);
    for (size_t i = 0; i < messageMembers->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_c__MessageMember* member = messageMembers->members_ + i;
        auto data = &messageData[member->offset_];
        auto valuePtr = container.at(index++);
        switch (member->type_id_)
        {
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__float__Sequence, rosidl_runtime_c__float__Sequence__init,
                          rosidl_runtime_c__float__Sequence__fini, float, float>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<float, float>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__double__Sequence, rosidl_runtime_c__double__Sequence__init,
                          rosidl_runtime_c__double__Sequence__fini, double, double>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<double, double>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__long_double__Sequence, rosidl_runtime_c__long_double__Sequence__init,
                          rosidl_runtime_c__long_double__Sequence__fini, long double, double>(
                    member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<long double, double>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__char__Sequence, rosidl_runtime_c__char__Sequence__init,
                          rosidl_runtime_c__char__Sequence__fini, uint8_t, uint8_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<uint8_t, uint8_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__wchar__Sequence, rosidl_runtime_c__wchar__Sequence__init,
                          rosidl_runtime_c__wchar__Sequence__fini, uint16_t, uint32_t>(
                    member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<uint16_t, uint32_t>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__boolean__Sequence, rosidl_runtime_c__bool__Sequence__init,
                          rosidl_runtime_c__bool__Sequence__fini, bool, bool>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<bool, bool>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__octet__Sequence, rosidl_runtime_c__octet__Sequence__init,
                          rosidl_runtime_c__octet__Sequence__fini, uint8_t, uint8_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<uint8_t, uint8_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint8__Sequence, rosidl_runtime_c__uint8__Sequence__init,
                          rosidl_runtime_c__uint8__Sequence__fini, uint8_t, uint32_t>(
                    member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<uint8_t, uint32_t>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int8__Sequence, rosidl_runtime_c__int8__Sequence__init,
                          rosidl_runtime_c__int8__Sequence__fini, int8_t, int32_t>(member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<int8_t, int32_t>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint16__Sequence, rosidl_runtime_c__uint16__Sequence__init,
                          rosidl_runtime_c__uint16__Sequence__fini, uint16_t, uint32_t>(
                    member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<uint16_t, uint32_t>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int16__Sequence, rosidl_runtime_c__int16__Sequence__init,
                          rosidl_runtime_c__int16__Sequence__fini, int16_t, int32_t>(member, data, valuePtr, fromOgnType);
            }
            else
            {
                _setSingleValue<int16_t, int32_t>(data, valuePtr, fromOgnType);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint32__Sequence, rosidl_runtime_c__uint32__Sequence__init,
                          rosidl_runtime_c__uint32__Sequence__fini, uint32_t, uint32_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<uint32_t, uint32_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int32__Sequence, rosidl_runtime_c__int32__Sequence__init,
                          rosidl_runtime_c__int32__Sequence__fini, int32_t, int32_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<int32_t, int32_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__uint64__Sequence, rosidl_runtime_c__uint64__Sequence__init,
                          rosidl_runtime_c__uint64__Sequence__fini, uint64_t, uint64_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<uint64_t, uint64_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
        {
            if (member->is_array_)
            {
                _setArray<rosidl_runtime_c__int64__Sequence, rosidl_runtime_c__int64__Sequence__init,
                          rosidl_runtime_c__int64__Sequence__fini, int64_t, int64_t>(member, data, valuePtr, false);
            }
            else
            {
                _setSingleValue<int64_t, int64_t>(data, valuePtr, false);
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
        {
            if (member->is_array_)
            {
                auto array = std::static_pointer_cast<std::vector<std::string>>(valuePtr);
                std::vector<rosidl_runtime_c__String> rosArray(array->size());
                for (size_t j = 0; j < array->size(); ++j)
                {
                    rosidl_runtime_c__String__assign(&rosArray.at(j), array->at(j).c_str());
                }
                _setArray<rosidl_runtime_c__String__Sequence, rosidl_runtime_c__String__Sequence__init,
                          rosidl_runtime_c__String__Sequence__fini, rosidl_runtime_c__String>(member, data, rosArray);
            }
            else
            {
                rosidl_runtime_c__String__assign(reinterpret_cast<rosidl_runtime_c__String*>(data),
                                                 std::static_pointer_cast<const std::string>(valuePtr)->c_str());
            }
            break;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
        {
            // TODO: proccess WSTRING (no messages with 'path:*/msgs/*.msg "wstring"')
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE:
        {
            if (member->is_array_)
            {
                auto jsonArrayPtr = std::static_pointer_cast<std::vector<nlohmann::json>>(valuePtr);
                setArrayEmbeddedMessage(member, data, *jsonArrayPtr);
            }
            else
            {
                setMessageValues(member->members_->data, data, container, --index, fromOgnType);
                continue;
            }
            break;
        }
        default:
            break;
        }
    }
}

} // namespace core
} // namespace ros2
} // namespace isaacsim
