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

#include "isaacsim/common/exceptions/Exceptions.hpp"

namespace isaacsim
{
namespace common
{
namespace exceptions
{

namespace
{

std::string formatAttributeNameErrorMessage(const std::string& attributeName,
                                            const std::optional<std::vector<std::string>>& validAttributeNames)
{
    std::string message = "Invalid attribute name: '" + attributeName + "'";
    if (validAttributeNames && !validAttributeNames->empty())
    {
        message += ". Valid attribute names: ";
        for (size_t i = 0; i < validAttributeNames->size(); ++i)
        {
            message += (i == 0 ? "'" : ", '") + (*validAttributeNames)[i] + "'";
        }
    }
    return message;
}

} // namespace

IsaacSimException::IsaacSimException(const std::string& message) : m_message(message)
{
}

const char* IsaacSimException::what() const noexcept
{
    return m_message.c_str();
}

PrimPathError::PrimPathError(const std::string& primPath)
    : IsaacSimException("Invalid prim at path: '" + primPath + "'"), m_primPath(primPath)
{
}

const std::string& PrimPathError::getPrimPath() const noexcept
{
    return m_primPath;
}

PrimPathStringError::PrimPathStringError(const std::string& primPathString)
    : IsaacSimException("Invalid prim path string: '" + primPathString + "'"), m_primPath(primPathString)
{
}

const std::string& PrimPathStringError::getPrimPath() const noexcept
{
    return m_primPath;
}

AttributeNameError::AttributeNameError(const std::string& attributeName,
                                       const std::optional<std::vector<std::string>>& validAttributeNames)
    : IsaacSimException(formatAttributeNameErrorMessage(attributeName, validAttributeNames)),
      m_attributeName(attributeName),
      m_validAttributeNames(validAttributeNames.value_or(std::vector<std::string>{}))
{
}

const std::string& AttributeNameError::getAttributeName() const noexcept
{
    return m_attributeName;
}

const std::vector<std::string>& AttributeNameError::getValidAttributeNames() const noexcept
{
    return m_validAttributeNames;
}

ValueTypeError::ValueTypeError(const std::string& attributeName,
                               const std::string& expectedType,
                               const std::string& actualType)
    : IsaacSimException("Invalid value type for attribute '" + attributeName + "': expected " + expectedType +
                        ", got " + actualType),
      m_attributeName(attributeName),
      m_expectedType(expectedType),
      m_actualType(actualType)
{
}

const std::string& ValueTypeError::getAttributeName() const noexcept
{
    return m_attributeName;
}

const std::string& ValueTypeError::getExpectedType() const noexcept
{
    return m_expectedType;
}

const std::string& ValueTypeError::getActualType() const noexcept
{
    return m_actualType;
}

CudaRuntimeError::CudaRuntimeError(const std::string& callerName,
                                   const std::string& errorDescription,
                                   std::optional<int> errorCode)
    : IsaacSimException(callerName + ": " + errorDescription +
                        (errorCode.has_value() ? " (error code: " + std::to_string(errorCode.value()) + ")" : "")),
      m_callerName(callerName),
      m_errorCode(errorCode)
{
}

const std::string& CudaRuntimeError::getCallerName() const noexcept
{
    return m_callerName;
}

std::optional<int> CudaRuntimeError::getErrorCode() const noexcept
{
    return m_errorCode;
}

} // namespace exceptions
} // namespace common
} // namespace isaacsim
