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

isaacsim_add_python_module(
    NAME isaacsim.robot_motion.cumotion
    REQUIRES_BINDINGS
)

if(ISAACSIM_ENABLE_PYTHON)
    set(cumotion_dependency_root "${ISAACSIM_TARGET_DEPS_DIR}/cumotion")
    file(GLOB cumotion_wheels "${cumotion_dependency_root}/python_wheels/cumotion-*.whl")
    list(LENGTH cumotion_wheels cumotion_wheel_count)
    if(NOT cumotion_wheel_count EQUAL 1)
        message(FATAL_ERROR
            "Expected exactly one platform-specific cuMotion wheel in "
            "${cumotion_dependency_root}/python_wheels, found ${cumotion_wheel_count}"
        )
    endif()
    list(GET cumotion_wheels 0 cumotion_wheel)

    set(cumotion_runtime_stage "${CMAKE_CURRENT_BINARY_DIR}/cumotion-runtime")
    set(cumotion_runtime_stamp "${CMAKE_CURRENT_BINARY_DIR}/cumotion-runtime.stamp")
    add_custom_command(
        OUTPUT "${cumotion_runtime_stamp}"
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${cumotion_runtime_stage}"
        COMMAND
            "${Python_EXECUTABLE}" -s -m pip install
            --disable-pip-version-check
            --no-compile
            --no-deps
            --no-index
            --target "${cumotion_runtime_stage}"
            "${cumotion_wheel}"
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${cumotion_runtime_stage}/cumotion-1.1.0a2.dist-info"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${cumotion_runtime_stage}" "${ISAACSIM_PYTHON_STAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E touch "${cumotion_runtime_stamp}"
        DEPENDS "${cumotion_wheel}"
        VERBATIM
    )
    add_custom_target(stage-isaacsim-robot-motion-cumotion-runtime ALL DEPENDS "${cumotion_runtime_stamp}")
    add_dependencies(isaacsim_robot_motion-python-build stage-isaacsim-robot-motion-cumotion-runtime)

    _isaacsim_get_group_component(isaacsim_robot_motion python python_component)
    install(
        DIRECTORY "${cumotion_runtime_stage}/"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}"
        COMPONENT "${python_component}"
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN "*.pyo" EXCLUDE
        PATTERN "*.dist-info" EXCLUDE
    )
    install(
        DIRECTORY "${cumotion_dependency_root}/PACKAGE-LICENSES/"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/robot_motion/cumotion/_licenses"
        COMPONENT "${python_component}"
    )
endif()

isaacsim_add_python_tests(
    MODULE isaacsim.robot_motion.cumotion
    REQUIRES_BINDINGS
)
