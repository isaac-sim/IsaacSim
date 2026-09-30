# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

isaacsim_add_module(
    NAME isaacsim.common.string
    EXPORT_HEADER isaacsim/common/string/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_STRING_API
    PUBLIC_CPP_HEADERS
        isaacsim/common/string/String.hpp
    SOURCES
        src/details/ColorParser.hpp
        src/details/ColorParser.cpp
        src/String.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.string
    SOURCES
        tests/cpp/String.cpp
    LABELS
        unit
)
