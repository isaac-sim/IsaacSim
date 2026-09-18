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
    NAME isaacsim.common.array
    EXPORT_HEADER isaacsim/common/array/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_ARRAY_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-exceptions
    PUBLIC_CPP_HEADERS
        isaacsim/common/array/nanobind/ArrayCaster.hpp
        isaacsim/common/array/Array.hpp
        isaacsim/common/array/Array.tpp
        isaacsim/common/array/Device.hpp
        isaacsim/common/array/DLPack.hpp
        isaacsim/common/array/Dtype.hpp
        isaacsim/common/array/Functions.hpp
        isaacsim/common/array/Shape.hpp
    SOURCES
        src/details/CudaKernel.hpp
        src/details/CudaKernel.cpp
        src/details/CudaRuntime.hpp
        src/details/CudaRuntime.cpp
        src/details/KernelExport.hpp
        src/Array.cpp
        src/Device.cpp
        src/DLPack.cpp
        src/Dtype.cpp
        src/Functions.cpp
        src/Shape.cpp
)

if(NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/dlpack/dlpack.h")
    message(FATAL_ERROR
        "isaacsim.common.array requires the pinned DLPack headers at ${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}. "
        "Run the library build without --no-pull."
    )
endif()
target_include_directories(isaacsim-common-array SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}>"
)

# Keep canonical <dlpack/dlpack.h> self-contained in the installed SDK.
get_target_property(isaacsim_array_group isaacsim-common-array ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component("${isaacsim_array_group}" development isaacsim_array_development_component)
install(DIRECTORY "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/dlpack"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT "${isaacsim_array_development_component}"
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../licenses/dlpack-LICENSE.txt"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${isaacsim_array_group}"
    COMPONENT "${isaacsim_array_development_component}"
)

set(isaacsim_array_cuda_root "${ISAACSIM_TARGET_DEPS_DIR}/cuda")
if(NOT EXISTS "${isaacsim_array_cuda_root}/include/cuda_runtime_api.h")
    message(FATAL_ERROR
        "isaacsim.common.array requires CUDA headers at ${isaacsim_array_cuda_root}. "
        "Fetch deps/isaacsim-libraries.packman.xml before configuring."
    )
endif()
target_include_directories(isaacsim-common-array PUBLIC
    "$<BUILD_INTERFACE:${isaacsim_array_cuda_root}/include>"
)

if(NOT APPLE AND NOT WIN32)
    target_link_options(isaacsim-common-array PRIVATE "-Wl,--exclude-libs,ALL")
endif()

# --- CUDA kernel library --------------------------------------------------------------------
# Requires a vendored `nvcc` (deps/isaacsim-libraries.packman.xml); configuration fails below if it
# is missing. Compiled into a separate shared library, statically linked against the CUDA
# runtime (no libcudart.so dependency), and never linked into isaacsim-common-array itself.
# CudaKernel dlopen's it lazily on first use, mirroring the pattern already used for
# libcudart.so in CudaRuntime.cpp, so isaacsim-common-array keeps loading and running correctly
# at runtime on machines with no CUDA driver installed, even though building it requires nvcc.
if(WIN32)
    set(isaacsim_array_nvcc "${isaacsim_array_cuda_root}/bin/nvcc.exe")
else()
    set(isaacsim_array_nvcc "${isaacsim_array_cuda_root}/bin/nvcc")
endif()

if(EXISTS "${isaacsim_array_nvcc}")
    if(NOT DEFINED CMAKE_CUDA_COMPILER)
        set(CMAKE_CUDA_COMPILER "${isaacsim_array_nvcc}" CACHE FILEPATH
            "Vendored nvcc used to build isaacsim.common.array's CUDA kernel library"
        )
    endif()
    enable_language(CUDA)

    if(NOT CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()

    add_library(isaacsim-common-array-cuda-kernels SHARED
        src/details/cuda_kernels/BinaryOp.cu
        src/details/cuda_kernels/Broadcast.cu
        src/details/cuda_kernels/Cast.cu
        src/details/cuda_kernels/ReduceOp.cu
        src/details/cuda_kernels/Take.cu
        src/details/cuda_kernels/UnaryOp.cu
    )
    get_target_property(isaacsim_array_module_output_dir isaacsim-common-array LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(isaacsim-common-array-cuda-kernels PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        LIBRARY_OUTPUT_DIRECTORY "${isaacsim_array_module_output_dir}"
        RUNTIME_OUTPUT_DIRECTORY "${isaacsim_array_module_output_dir}"
    )
    target_include_directories(isaacsim-common-array-cuda-kernels PRIVATE
        "${isaacsim_array_cuda_root}/include"
        # header-only usage: no isaacsim-common-array symbols are linked in.
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
    )
    # Static CUDA runtime: the resulting library has no libcudart.so DT_NEEDED entry, so it still
    # dlopen()s successfully with no CUDA runtime installed; only an actual kernel launch needs
    # the NVIDIA driver, which the static runtime resolves lazily and reports.
    if(WIN32)
        target_link_libraries(isaacsim-common-array-cuda-kernels PRIVATE
            "${isaacsim_array_cuda_root}/lib/x64/cudart_static.lib"
        )
    else()
        target_link_libraries(isaacsim-common-array-cuda-kernels PRIVATE
            "${isaacsim_array_cuda_root}/lib64/libcudart_static.a"
            "${isaacsim_array_cuda_root}/lib64/libculibos.a"
            dl pthread rt
        )
    endif()

    # Not a link dependency (isaacsim-common-array never links against the kernel library) -- this
    # only orders the build so the sibling .so exists in the output dir before tests dlopen it.
    add_dependencies(isaacsim-common-array isaacsim-common-array-cuda-kernels)

    # Ships alongside the module: CudaKernel resolves this library as a sibling of the module
    # binary, so it must land in the same install directory. Not exported, since consumers load it
    # through Array rather than linking it.
    _isaacsim_get_group_component("${isaacsim_array_group}" runtime isaacsim_array_runtime_component)
    install(TARGETS isaacsim-common-array-cuda-kernels
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT "${isaacsim_array_runtime_component}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT "${isaacsim_array_runtime_component}"
    )
else()
    message(FATAL_ERROR
        "isaacsim.common.array requires the vendored nvcc at ${isaacsim_array_nvcc} to build its "
        "CUDA kernel library. Fetch deps/isaacsim-libraries.packman.xml before configuring."
    )
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.array
    SOURCES
        tests/cpp/Array.cpp
        tests/cpp/Device.cpp
        tests/cpp/DLPack.cpp
        tests/cpp/Dtype.cpp
        tests/cpp/Functions.cpp
        tests/cpp/Shape.cpp
    LABELS
        unit
)
if(BUILD_TESTING)
    target_include_directories(tests-isaacsim-common-array PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
endif()

if(BUILD_TESTING)
    if(WIN32)
        # Windows has no RPATH, so isaacsim-common-array.dll is copied next to the test executable.
        # CudaKernel resolves the kernel library as a sibling of whichever copy is loaded, so it has to
        # follow the module there. It is not a link dependency, so the generic test DLL copy misses it.
        #
        # Attached to the kernel library rather than the test executable: the executable does not link
        # the kernel library, so rebuilding kernel sources never relinks it, and a POST_BUILD copy
        # hanging off the executable would keep serving a stale copy. The test output directory may not
        # exist yet, since the kernel library builds before the executable.
        add_custom_command(TARGET isaacsim-common-array-cuda-kernels POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:tests-isaacsim-common-array>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:isaacsim-common-array-cuda-kernels>"
                "$<TARGET_FILE_DIR:tests-isaacsim-common-array>"
            VERBATIM
        )
        set_property(TEST tests-isaacsim-common-array APPEND PROPERTY
            ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${isaacsim_array_cuda_root}/bin"
        )
    else()
        set_property(TEST tests-isaacsim-common-array APPEND PROPERTY
            ENVIRONMENT_MODIFICATION "LD_LIBRARY_PATH=path_list_prepend:${isaacsim_array_cuda_root}/lib64"
        )
    endif()
endif()
