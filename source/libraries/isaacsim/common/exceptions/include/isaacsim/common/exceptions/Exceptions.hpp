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

#include "isaacsim/common/exceptions/Export.h"

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaacsim
{
namespace common
{
namespace exceptions
{

/**
 * @brief Common base class for every Isaac Sim exception.
 * @details Catch this type to handle any exception raised by Isaac Sim code without depending on a specific
 * exception subtype.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API IsaacSimException : public std::exception
{
public:
    /**
     * @brief Construct an exception with an already formatted message.
     * @param[in] message The human-readable description of the failure.
     */
    explicit IsaacSimException(const std::string& message);

    /**
     * @brief Return the human-readable description of the failure.
     */
    const char* what() const noexcept override;

private:
    std::string m_message;
};

/**
 * @brief Raised when a USD prim path refers to an invalid prim.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API PrimPathError : public IsaacSimException
{
public:
    /**
     * @brief Construct the exception for a prim path that refers to an invalid prim.
     * @param[in] primPath The offending prim path.
     */
    explicit PrimPathError(const std::string& primPath);

    /**
     * @brief Return the offending prim path.
     */
    const std::string& getPrimPath() const noexcept;

private:
    std::string m_primPath;
};

/**
 * @brief Raised when a string does not parse as a valid USD prim path.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API PrimPathStringError : public IsaacSimException
{
public:
    /**
     * @brief Construct the exception for a malformed prim path string.
     * @param[in] primPathString The offending prim path string.
     */
    explicit PrimPathStringError(const std::string& primPathString);

    /**
     * @brief Return the offending prim path string.
     */
    const std::string& getPrimPath() const noexcept;

private:
    std::string m_primPath;
};

/**
 * @brief Raised when a caller references an attribute name that does not exist or is not accepted.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API AttributeNameError : public IsaacSimException
{
public:
    /**
     * @brief Construct the exception for an invalid attribute name.
     * @param[in] attributeName The offending attribute name.
     * @param[in] validAttributeNames A list of valid attribute names (optional).
     */
    explicit AttributeNameError(const std::string& attributeName,
                                const std::optional<std::vector<std::string>>& validAttributeNames = std::nullopt);

    /**
     * @brief Return the offending attribute name.
     */
    const std::string& getAttributeName() const noexcept;

    /**
     * @brief Return the valid attribute names.
     */
    const std::vector<std::string>& getValidAttributeNames() const noexcept;

private:
    std::string m_attributeName;
    std::vector<std::string> m_validAttributeNames;
};

/**
 * @brief Raised when a value's type does not match the type expected for an attribute.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API ValueTypeError : public IsaacSimException
{
public:
    /**
     * @brief Construct the exception for a value/attribute type mismatch.
     * @param[in] attributeName The attribute the value was provided for.
     * @param[in] expectedType A human-readable description of the expected type.
     * @param[in] actualType A human-readable description of the type that was provided.
     */
    ValueTypeError(const std::string& attributeName, const std::string& expectedType, const std::string& actualType);

    /**
     * @brief Return the attribute the value was provided for.
     */
    const std::string& getAttributeName() const noexcept;

    /**
     * @brief Return a human-readable description of the expected type.
     */
    const std::string& getExpectedType() const noexcept;

    /**
     * @brief Return a human-readable description of the type that was provided.
     */
    const std::string& getActualType() const noexcept;

private:
    std::string m_attributeName;
    std::string m_expectedType;
    std::string m_actualType;
};

/**
 * @brief Raised when a CUDA runtime call fails.
 */
class ISAACSIM_COMMON_EXCEPTIONS_API CudaRuntimeError : public IsaacSimException
{
public:
    /**
     * @brief Construct the exception for a CUDA runtime call that returned an error.
     * @param[in] callerName The name of the caller that made the CUDA runtime call.
     * @param[in] errorDescription A human-readable description of the error, as reported by the CUDA runtime.
     * @param[in] errorCode The CUDA error code returned by the call (optional).
     */
    CudaRuntimeError(const std::string& callerName,
                     const std::string& errorDescription,
                     std::optional<int> errorCode = std::nullopt);

    /**
     * @brief Return the name of the caller that made the CUDA runtime call.
     */
    const std::string& getCallerName() const noexcept;

    /**
     * @brief Return the CUDA error code returned by the call.
     */
    std::optional<int> getErrorCode() const noexcept;

private:
    std::string m_callerName;
    std::optional<int> m_errorCode;
};

} // namespace exceptions
} // namespace common
} // namespace isaacsim
