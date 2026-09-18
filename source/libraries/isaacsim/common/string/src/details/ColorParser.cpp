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

#include "ColorParser.hpp"

#include <cctype>
#include <stdexcept>

namespace isaacsim
{
namespace common
{
namespace string
{
namespace details
{

int hexDigit(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    const char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (lower >= 'a' && lower <= 'f')
    {
        return 10 + (lower - 'a');
    }
    throw std::invalid_argument("Invalid hexadecimal digit: '" + std::string(1, c) + "'");
}

std::vector<float> parseHexColor(const std::string& hex)
{
    if (hex.empty() || hex.front() != '#')
    {
        throw std::invalid_argument("Invalid hex color specifier: '" + hex + "'");
    }

    std::vector<float> rgb(3);
    const std::string digits = hex.substr(1);
    const std::size_t length = digits.size();

    // Shorthand forms: each digit is duplicated (#rgb -> #rrggbb), so its value is digit / 15.
    if (length == 3 || length == 4)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            rgb[i] = static_cast<float>(hexDigit(digits[i])) / 15.0f;
        }
    }
    // Full forms: each channel is a byte, so its value is byte / 255.
    else if (length == 6 || length == 8)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            const int value = hexDigit(digits[2 * i]) * 16 + hexDigit(digits[2 * i + 1]);
            rgb[i] = static_cast<float>(value) / 255.0f;
        }
    }
    // Invalid length.
    else
    {
        throw std::invalid_argument("Invalid hex color specifier: '" + hex + "'");
    }
    return rgb;
}

} // namespace details
} // namespace string
} // namespace common
} // namespace isaacsim
