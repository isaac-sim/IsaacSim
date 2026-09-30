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

#include "isaacsim/common/string/String.hpp"

#include "details/ColorParser.hpp"

#include <algorithm>
#include <functional>
#include <optional>

namespace isaacsim
{
namespace common
{
namespace string
{

std::size_t VectorStringHasher::operator()(const std::vector<std::string>& paths) const
{
    std::size_t seed = paths.size();
    for (const auto& path : paths)
    {
        seed ^= std::hash<std::string>{}(path) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
}

std::string toLower(const std::string& text)
{
    std::string result = text;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
    return result;
}

std::string toUpper(const std::string& text)
{
    std::string result = text;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::toupper(c); });
    return result;
}

std::string join(const std::vector<std::string>& strings, const std::string& separator)
{
    std::size_t totalSize = strings.empty() ? 0 : separator.size() * (strings.size() - 1);
    for (const auto& s : strings)
    {
        totalSize += s.size();
    }

    std::string result;
    result.reserve(totalSize);
    for (std::size_t i = 0; i < strings.size(); ++i)
    {
        if (i > 0)
        {
            result += separator;
        }
        result += strings[i];
    }
    return result;
}

std::vector<std::string> split(const std::string& text, const std::string& separator)
{
    std::vector<std::string> result;
    if (separator.empty())
    {
        result.push_back(text);
        return result;
    }

    std::size_t start = 0;
    std::size_t pos = text.find(separator, start);
    while (pos != std::string::npos)
    {
        result.push_back(text.substr(start, pos - start));
        start = pos + separator.size();
        pos = text.find(separator, start);
    }
    result.push_back(text.substr(start));
    return result;
}

std::vector<std::string> sort(const std::vector<std::string>& strings, bool ascending)
{
    std::vector<std::string> result = strings;
    if (ascending)
    {
        std::sort(result.begin(), result.end());
    }
    else
    {
        std::sort(result.begin(), result.end(), std::greater<std::string>());
    }
    return result;
}

std::vector<float> resolveColor(const std::string& color)
{
    // `CN` cycle specification: 'C' followed by one or more digits, indexing the default color cycle.
    if (color.size() >= 2 && color.front() == 'C' &&
        std::all_of(color.begin() + 1, color.end(), [](unsigned char c) { return std::isdigit(c); }))
    {
        const auto& cycle = details::g_kDefaultColorCycle;
        std::size_t index = 0;
        for (std::size_t i = 1; i < color.size(); ++i)
        {
            index = (index * 10 + static_cast<std::size_t>(color[i] - '0')) % cycle.size();
        }
        return details::parseHexColor(cycle[index]);
    }

    // Fully transparent special value: matplotlib maps `"none"` to (0, 0, 0, 0); RGB is black.
    if (toLower(color) == "none")
    {
        return { 0.0f, 0.0f, 0.0f };
    }

    // Named color lookup: try the exact key first, then a lower-cased key (unless single-character, to
    // match matplotlib, which does not case-fold single letters such as the base colors).
    const auto& base = details::g_kBaseColors;
    const auto& named = details::g_kNamedColors;
    const auto lookupNamed = [&base, &named](const std::string& key) -> std::optional<std::vector<float>>
    {
        if (const auto it = base.find(key); it != base.end())
        {
            return std::vector<float>{ it->second[0], it->second[1], it->second[2] };
        }
        if (const auto it = named.find(key); it != named.end())
        {
            return details::parseHexColor(it->second);
        }
        return std::nullopt;
    };
    if (const auto rgb = lookupNamed(color))
    {
        return *rgb;
    }
    if (color.size() != 1)
    {
        if (const auto rgb = lookupNamed(toLower(color)))
        {
            return *rgb;
        }
    }

    // Hex specification.
    if (!color.empty() && color.front() == '#')
    {
        return details::parseHexColor(color);
    }

    // Grayscale value: a float string in the 0-1 range. std::stof accepts trailing junk and leading
    // whitespace, so only treat the input as a grayscale value when the whole string was consumed.
    bool isFloat = false;
    float value = 0.0f;
    try
    {
        std::size_t consumed = 0;
        value = std::stof(color, &consumed);
        isFloat = (consumed == color.size());
    }
    catch (const std::exception&)
    {
        // Not a float; fall through to the unrecognized-color error below.
    }
    if (isFloat)
    {
        // Reject values outside 0-1 as well as NaN (all comparisons with NaN are false, so the negated
        // in-range test rejects it) to match matplotlib's `not (0 <= value <= 1)` check.
        if (!(value >= 0.0f && value <= 1.0f))
        {
            throw std::invalid_argument("Invalid string grayscale value '" + color +
                                        "'. Value must be within the 0-1 range");
        }
        return { value, value, value };
    }

    throw std::invalid_argument("Invalid color specification: '" + color + "'");
}

} // namespace string
} // namespace common
} // namespace isaacsim
