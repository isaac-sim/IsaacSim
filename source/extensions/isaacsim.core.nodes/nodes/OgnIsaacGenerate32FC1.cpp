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

#include "OgnIsaacGenerate32FC1Database.h"

#include <cmath>
#include <string>

namespace isaacsim
{
namespace core
{
namespace nodes
{

/**
 * @brief a node that converts from rgba to rgb
 *
 */
class OgnIsaacGenerate32FC1
{

public:
    static bool compute(OgnIsaacGenerate32FC1Database& db)
    {
        auto& state = db.perInstanceState<OgnIsaacGenerate32FC1>();
        size_t numElements = db.inputs.width() * db.inputs.height();
        state.m_values.resize(numElements);

        std::fill_n(state.m_values.begin(), numElements / 2, db.inputs.value());

        // Fill second hallf of array with different float value for disparity
        std::fill_n(state.m_values.begin() + numElements / 2, numElements / 2, db.inputs.value() / 2);

        size_t buffSize = numElements * sizeof(float);
        db.outputs.data.resize(buffSize);

        memcpy(db.outputs.data().data(), &state.m_values.data()[0], buffSize);

        db.outputs.width() = db.inputs.width();
        db.outputs.height() = db.inputs.height();
        db.outputs.encoding() = db.stringToToken("32FC1");

        return true;
    }

private:
    std::vector<float> m_values;
};
REGISTER_OGN_NODE()
}
}
}
