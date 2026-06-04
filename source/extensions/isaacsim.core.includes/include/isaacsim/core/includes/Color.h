// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Defines.h>
#include <carb/Types.h>

#include <algorithm>

namespace isaacsim
{
namespace core
{
namespace includes
{
/**
 * @namespace color
 * @brief Provides color conversion and gradient utility functions
 * @details
 * Contains utility functions for generating color gradients and converting between
 * different color formats. Useful for data visualization and UI applications.
 */
namespace color
{

/**
 * @brief Converts a ratio value to a carb::ColorRgba struct
 * @details
 * Generates a color from the rainbow color spectrum (ROYGBIV) based on the input ratio.
 * The ratio is mapped to six color regions, creating a smooth gradient transition between them.
 * Similar to distToColor() but returns a different format.
 *
 * @param[in] ratio A value between 0.0 and 1.0 representing position in the color gradient
 * @return carb::ColorRgba struct with values normalized to [0.0, 1.0] and alpha set to 1.0
 */
static inline carb::ColorRgba distToRgba(double ratio)
{
    // Clamp ratio to [0,1] range
    const double clampedRatio = std::max(0.0, std::min(1.0, ratio));

    // Define constants for clarity
    constexpr int kRegionSize = 256;
    constexpr int kNumRegions = 6;

    // Normalize ratio to fit into 6 color regions (0-5)
    const int normalizedValue = static_cast<int>(clampedRatio * kRegionSize * kNumRegions);

    // Region index (0-5) and position within region (0-255)
    const int regionIndex = normalizedValue / kRegionSize;
    const int regionOffset = normalizedValue % kRegionSize;

    // Initialize RGB values
    int red = 0, green = 0, blue = 0;

    // Calculate RGB values based on region
    switch (regionIndex)
    {
    case 0: // Red to Yellow
        red = 255;
        green = regionOffset;
        blue = 0;
        break;
    case 1: // Yellow to Green
        red = 255 - regionOffset;
        green = 255;
        blue = 0;
        break;
    case 2: // Green to Cyan
        red = 0;
        green = 255;
        blue = regionOffset;
        break;
    case 3: // Cyan to Blue
        red = 0;
        green = 255 - regionOffset;
        blue = 255;
        break;
    case 4: // Blue to Magenta
        red = regionOffset;
        green = 0;
        blue = 255;
        break;
    case 5: // Magenta to Red
        red = 255;
        green = 0;
        blue = 255 - regionOffset;
        break;
    }

    // Convert 0-255 values to 0.0-1.0 range
    constexpr float kColorNormalization = 1.0f / 255.0f;
    return carb::ColorRgba({ red * kColorNormalization, green * kColorNormalization, blue * kColorNormalization, 1.0f });
}

}
}
}
}
