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

find_package(Carbonite REQUIRED MODULE)

isaacsim_add_module(
    NAME isaacsim.common.logging
    EXPORT_HEADER isaacsim/common/logging/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_LOGGING_API
    PUBLIC_C_HEADERS
        isaacsim/common/logging/Logging.h
        isaacsim/common/logging/LoggingHost.h
    PUBLIC_CPP_HEADERS
        isaacsim/common/logging/Logging.hpp
        isaacsim/common/logging/details/LogMacros.hpp
    PUBLIC_C_SYMBOLS_FILE abi/c-api.symbols
    SOURCES
        src/CApi.cpp
        src/details/CarboniteBackend.hpp
        src/details/CarboniteBackend.cpp
        src/details/LoggingHostInternal.hpp
        src/Logging.cpp
    PRIVATE_DEPENDENCIES
        carb::static
)

if(NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/fmt/format.h")
    message(FATAL_ERROR
        "isaacsim.common.logging requires the pinned fmt headers at ${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}. "
        "Run the library build without --no-pull."
    )
endif()
target_include_directories(isaacsim-common-logging PUBLIC
    "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}>"
)
target_compile_definitions(isaacsim-common-logging PUBLIC FMT_HEADER_ONLY=1)

# The SDK is self-contained: consumers receive the exact header-only fmt version used by the public C++ templates.
get_target_property(isaacsim_logging_group isaacsim-common-logging ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component("${isaacsim_logging_group}" development isaacsim_logging_development_component)
install(DIRECTORY "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/fmt"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT "${isaacsim_logging_development_component}"
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../licenses/fmt-LICENSE.rst"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${isaacsim_logging_group}"
    RENAME "fmt-LICENSE.txt"
    COMPONENT "${isaacsim_logging_development_component}"
)

target_compile_definitions(isaacsim-common-logging PRIVATE
    CARB_STATIC_PLUGIN_BUILD=1
    CARB_PLUGIN_CLIENT_NAME=isaacsim_common_logging
    carb_ErrorApi=carb_ErrorApi_latest
    carb_Framework=carb_Framework_latest
    carb_filesystem_IFileSystem=carb_filesystem_IFileSystem_latest
    carb_logging_ILogging=carb_logging_ILogging_latest
)
if(NOT APPLE AND NOT WIN32)
    target_link_options(isaacsim-common-logging PRIVATE "-Wl,--exclude-libs,ALL")
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.logging
    SOURCES
        tests/cpp/Logging.cpp
    LABELS
        threading
)

isaacsim_add_c_tests(
    MODULE isaacsim.common.logging
    SOURCES
        tests/c/Logging.cpp
)

if(BUILD_TESTING)
    set_tests_properties(tests-c-isaacsim-common-logging PROPERTIES
        LABELS "unit;c;integration;backend;isaacsim.common.logging"
        PASS_REGULAR_EXPRESSION "\\[isaacsim.test.c\\] isaacsim-c-report-probe"
    )
    add_test(
        NAME tests-c-report-isaacsim-common-logging
        COMMAND $<TARGET_FILE:tests-c-isaacsim-common-logging>
            "--test-case=Report writes to standard output and submits to Carbonite"
    )
    set_tests_properties(tests-c-report-isaacsim-common-logging PROPERTIES
        LABELS "unit;c;integration;stdout;isaacsim.common.logging"
        PASS_REGULAR_EXPRESSION "\\[isaacsim.test.c\\] isaacsim-c-report-probe"
    )
    add_test(
        NAME tests-carbonite-backend-isaacsim-common-logging
        COMMAND $<TARGET_FILE:tests-isaacsim-common-logging>
            "--test-case=First submission initializes the standalone Carbonite backend"
    )
    string(CONCAT isaacsim_logging_output_test_pattern
        "\\[[0-9][0-9]*ms\\] "
        "\\[Warning\\] \\[isaacsim.test.backend\\] isaacsim-carbonite-backend-probe"
    )
    set_tests_properties(tests-carbonite-backend-isaacsim-common-logging PROPERTIES
        LABELS "integration;backend;isaacsim.common.logging"
        PASS_REGULAR_EXPRESSION "${isaacsim_logging_output_test_pattern}"
    )
    add_test(
        NAME tests-report-isaacsim-common-logging
        COMMAND $<TARGET_FILE:tests-isaacsim-common-logging>
            "--test-case=Report writes to standard output and submits to Carbonite"
    )
    set_tests_properties(tests-report-isaacsim-common-logging PROPERTIES
        LABELS "unit;integration;stdout;isaacsim.common.logging"
        PASS_REGULAR_EXPRESSION "\\[isaacsim.test.report\\] isaacsim-cpp-formatted-result-42"
    )
    if(WIN32)
        foreach(test_name IN ITEMS
            tests-c-report-isaacsim-common-logging
            tests-carbonite-backend-isaacsim-common-logging
            tests-report-isaacsim-common-logging
        )
            set_property(TEST ${test_name} APPEND PROPERTY
                ENVIRONMENT_MODIFICATION
                    "PATH=path_list_prepend:$<TARGET_FILE_DIR:isaacsim-common-logging>"
            )
        endforeach()
    endif()
endif()

isaacsim_add_nanobind(
    MODULE isaacsim.common.logging
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.common.logging
    REQUIRES_BINDINGS
    LABELS
        threading
)
