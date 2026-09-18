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

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ovsim
{
namespace interfaces
{
namespace details
{

/**
 * @brief Format an invalid-attribute error message.
 *
 * @param[in] attributeName Invalid attribute name.
 * @param[in] validAttributeNames Optional list of accepted attribute names.
 * @return Formatted diagnostic message.
 */
inline std::string formatAttributeErrorMessage(const std::string& attributeName,
                                               const std::optional<std::vector<std::string>>& validAttributeNames)
{
    std::string message = "Invalid attribute name: '" + attributeName + "'";
    if (validAttributeNames && !validAttributeNames->empty())
    {
        message += ". Valid attribute names: ";
        for (std::size_t i = 0; i < validAttributeNames->size(); ++i)
        {
            message += (i == 0 ? "'" : ", '") + (*validAttributeNames)[i] + "'";
        }
    }
    return message;
}

/** @brief Error raised when an OV SIM component is used before initialization. */
class InitializationError : public std::runtime_error
{
public:
    /** @brief Construct an initialization error without a component name. */
    InitializationError() : std::runtime_error("Not initialized")
    {
    }

    /**
     * @brief Construct an initialization error for a component.
     *
     * @param[in] component Name of the component that is not initialized.
     */
    explicit InitializationError(const std::string& component)
        : std::runtime_error("Not initialized: " + component), m_component(component)
    {
    }

    /** @return Name of the uninitialized component, or an empty string when unspecified. */
    [[nodiscard]] const std::string& getComponent() const noexcept
    {
        return m_component;
    }

private:
    std::string m_component;
};

/** @brief Error raised when an OV SIM provider receives an unsupported attribute name. */
class AttributeError : public std::runtime_error
{
public:
    /**
     * @brief Construct an invalid-attribute error.
     *
     * @param[in] attributeName Unsupported attribute name.
     * @param[in] validAttributeNames Optional list of accepted attribute names.
     */
    explicit AttributeError(const std::string& attributeName,
                            const std::optional<std::vector<std::string>>& validAttributeNames = std::nullopt)
        : std::runtime_error(formatAttributeErrorMessage(attributeName, validAttributeNames)),
          m_attributeName(attributeName),
          m_validAttributeNames(validAttributeNames ? *validAttributeNames : std::vector<std::string>())
    {
    }

    /** @return Unsupported attribute name supplied to the provider. */
    [[nodiscard]] const std::string& getAttributeName() const noexcept
    {
        return m_attributeName;
    }

    /** @return Accepted attribute names, or an empty collection when none were supplied. */
    [[nodiscard]] const std::vector<std::string>& getValidAttributeNames() const noexcept
    {
        return m_validAttributeNames;
    }

private:
    std::string m_attributeName;
    std::vector<std::string> m_validAttributeNames;
};

} // namespace details
} // namespace interfaces
} // namespace ovsim
