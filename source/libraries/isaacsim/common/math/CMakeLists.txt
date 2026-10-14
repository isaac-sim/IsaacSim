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
    NAME isaacsim.common.math
    EXPORT_HEADER isaacsim/common/math/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_MATH_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
    PUBLIC_CPP_HEADERS
        isaacsim/common/math/Rotation.hpp
    SOURCES
        src/Rotation.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.math
    SOURCES
        tests/cpp/Rotation.cpp
    LABELS
        unit
)

# Rotation.cpp tests Device::Cuda() operands. Nothing here puts the vendored CUDA directory on the
# loader path: isaacsim-common-array binds the CUDA driver, which the display driver installs where
# the loader already looks, and the kernel library carries a statically linked runtime.
if(BUILD_TESTING)
    if(WIN32)
        # Windows has no RPATH, and the kernel library is not a link dependency of
        # isaacsim-common-array (see array/CMakeLists.txt), so the generic test DLL copy misses it
        # here too. array/CMakeLists.txt already copies it next to the array tests; this copies the
        # same build output next to the math tests, whose CUDA cases dlopen() it independently.
        #
        # OUTPUT (not TARGET) POST_BUILD: a TARGET-style command can only be attached to a target
        # created in this directory, and math does not own isaacsim-common-array-cuda-kernels. An
        # OUTPUT command's DEPENDS may name a target from another directory and still reruns
        # whenever that target rebuilds -- unlike hanging this off tests-isaacsim-common-math's own
        # POST_BUILD, which would only rerun when the math test executable itself relinks and could
        # leave a stale kernel DLL behind after an incremental kernel-only rebuild.
        #
        # OUTPUT is a plain stamp file rather than the copied DLL's own path: CMake cannot resolve a
        # target-file generator expression (needed for the real destination, which lives in
        # tests-isaacsim-common-math's own output directory) inside a custom command's OUTPUT, only
        # inside its COMMAND arguments.
        # This copy now runs before tests-isaacsim-common-math links (add_dependencies orders it
        # first), rather than after as a POST_BUILD step, so the linker has not necessarily created
        # the test executable's output directory yet on a clean build; create it explicitly, as the
        # equivalent array copy does.
        set(isaacsim_math_cuda_kernels_stamp "${CMAKE_CURRENT_BINARY_DIR}/copy-cuda-kernels.stamp")
        add_custom_command(
            OUTPUT "${isaacsim_math_cuda_kernels_stamp}"
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:tests-isaacsim-common-math>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:isaacsim-common-array-cuda-kernels>"
                "$<TARGET_FILE_DIR:tests-isaacsim-common-math>"
            COMMAND ${CMAKE_COMMAND} -E touch "${isaacsim_math_cuda_kernels_stamp}"
            DEPENDS isaacsim-common-array-cuda-kernels
            VERBATIM
        )
        add_custom_target(isaacsim-common-math-copy-cuda-kernels ALL
            DEPENDS "${isaacsim_math_cuda_kernels_stamp}"
        )
        add_dependencies(tests-isaacsim-common-math isaacsim-common-math-copy-cuda-kernels)
    endif()
endif()
