// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <OgnIsaacTimeSplitterDatabase.h>
#include <cstdint>
#include <cstdlib>

#define DENOMINATOR 1000000000LL


namespace isaacsim
{
namespace core
{
namespace nodes
{

class OgnIsaacTimeSplitter
{
public:
    static bool compute(OgnIsaacTimeSplitterDatabase& db)
    {
        int32_t seconds = 0;
        uint32_t milliseconds = 0, microseconds = 0, nanoseconds = 0;

        const auto& inputTimeAttr = db.inputs.time();
        switch (inputTimeAttr.type().baseType)
        {
        case BaseDataType::eDouble:
        {
            auto inputTimeData = inputTimeAttr.get<double>();
            auto inputTime = inputTimeData.vectorized(1)[0];
            _timeSplit(inputTime, seconds, milliseconds, microseconds, nanoseconds);
            break;
        }
        case BaseDataType::eFloat:
        {
            auto inputTimeData = inputTimeAttr.get<float>();
            auto inputTime = inputTimeData.vectorized(1)[0];
            _timeSplit(inputTime, seconds, milliseconds, microseconds, nanoseconds);
            break;
        }
        case BaseDataType::eHalf:
        {
            auto inputTimeData = inputTimeAttr.get<pxr::GfHalf>();
            auto inputTime = static_cast<float>(inputTimeData.vectorized(1)[0]);
            _timeSplit(inputTime, seconds, milliseconds, microseconds, nanoseconds);
            break;
        }
        case BaseDataType::eInt:
        {
            auto inputTimeData = inputTimeAttr.get<int32_t>();
            seconds = static_cast<int32_t>(inputTimeData.vectorized(1)[0]);
            break;
        }
        case BaseDataType::eInt64:
        {
            auto inputTimeData = inputTimeAttr.get<int64_t>();
            seconds = static_cast<int32_t>(inputTimeData.vectorized(1)[0]);
            break;
        }
        case BaseDataType::eUInt:
        {
            auto inputTimeData = inputTimeAttr.get<uint32_t>();
            seconds = static_cast<int32_t>(inputTimeData.vectorized(1)[0]);
            break;
        }
        case BaseDataType::eUInt64:
        {
            auto inputTimeData = inputTimeAttr.get<uint64_t>();
            seconds = static_cast<int32_t>(inputTimeData.vectorized(1)[0]);
            break;
        }
        default:
            db.logError("Failed to resolve input type (supported types: double, float, half, int, int64, uint, uint64");
            return false;
        }

        db.outputs.seconds() = seconds;
        db.outputs.milliseconds() = milliseconds;
        db.outputs.microseconds() = microseconds;
        db.outputs.nanoseconds() = nanoseconds;
        return true;
    }

private:
    static void _timeSplit(
        const double& time, int32_t& seconds, uint32_t& milliseconds, uint32_t& microseconds, uint32_t& nanoseconds)
    {
        const auto result = std::div(static_cast<long long>(time * 1e9), DENOMINATOR);
        if (result.rem >= 0)
        {
            seconds = static_cast<int32_t>(result.quot);
            nanoseconds = static_cast<uint32_t>(result.rem);
        }
        else
        {
            seconds = static_cast<int32_t>(result.quot - 1);
            nanoseconds = static_cast<uint32_t>(DENOMINATOR + result.rem);
        }
        milliseconds = static_cast<uint32_t>(nanoseconds / 1000000L);
        microseconds = static_cast<uint32_t>(nanoseconds / 1000L);
    }
};

REGISTER_OGN_NODE()
} // core_nodes
} // isaac
} // omni
