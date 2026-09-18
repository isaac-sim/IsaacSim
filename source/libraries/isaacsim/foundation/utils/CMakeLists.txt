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
    NAME isaacsim.foundation.utils
    EXPORT_HEADER isaacsim/foundation/utils/Export.h
    EXPORT_MACRO ISAACSIM_FOUNDATION_UTILS_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::foundation-objects
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
        isaacsim::foundation-usd-openusd
        isaacsim::foundation-usd-ovstage
    PUBLIC_CPP_HEADERS
        isaacsim/foundation/utils/Backend.hpp
        isaacsim/foundation/utils/Prim.hpp
        isaacsim/foundation/utils/Semantics.hpp
        isaacsim/foundation/utils/Stage.hpp
    SOURCES
        src/Backend.cpp
        src/Prim.cpp
        src/Semantics.cpp
        src/Stage.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.foundation.utils
    REQUIRES_NO_BINDINGS
    SOURCES
        tests/cpp/Backend.cpp
        tests/cpp/Prim.cpp
        tests/cpp/Semantics.cpp
        tests/cpp/Stage.cpp
    DEPENDENCIES
        isaacsim::foundation-objects
        isaacsim::foundation-usd-openusd
        isaacsim::foundation-usd-ovstage
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.foundation.utils
    SOURCE bindings/python/Bindings.cpp
    STUB_PRELOADS
        isaacsim.foundation.objects
)

isaacsim_add_python_tests(
    MODULE isaacsim.foundation.utils
    REQUIRES_BINDINGS
    LABELS
        unit
)
