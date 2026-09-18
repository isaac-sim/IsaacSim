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
    NAME isaacsim.physics.manager
    EXPORT_HEADER isaacsim/physics/manager/Export.h
    EXPORT_MACRO ISAACSIM_PHYSICS_MANAGER_API
    PUBLIC_CPP_HEADERS
        isaacsim/physics/manager/PhysicsEvent.hpp
        isaacsim/physics/manager/PhysicsManager.hpp
        isaacsim/physics/manager/PhysicsBenchmark.hpp
        isaacsim/physics/manager/tensors/EntityView.hpp
        isaacsim/physics/manager/PhysicsInteraction.hpp
        isaacsim/physics/manager/PhysicsSceneQuery.hpp
        isaacsim/physics/manager/PhysicsSimulation.hpp
    SOURCES
        src/PhysicsManager.cpp
        src/SimulationSnapshot.hpp
        src/SubscriptionStore.hpp
        src/Simulation.cpp
        src/Benchmark.cpp
        src/Interaction.cpp
        src/SceneQuery.cpp
        src/tensors/EntityView.cpp
    PUBLIC_DEPENDENCIES
        isaacsim::physics-registration
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
)

# Each binding implementation compiles as an explicit translation unit. The module
# imports the registration bindings first so shared vocabulary types are registered.
isaacsim_add_nanobind(
    MODULE isaacsim.physics.manager
    SOURCES
        bindings/python/Bindings.cpp
        bindings/python/BindingsPhysicsBenchmarks.cpp
        bindings/python/BindingsPhysicsInteraction.cpp
        bindings/python/BindingsPhysicsSceneQuery.cpp
        bindings/python/BindingsPhysicsSimulation.cpp
        bindings/python/BindingsTensorCreate.cpp
        bindings/python/BindingsTensorViews.cpp
)

# Share registration-owned nanobind type casters without installing private
# binding headers as part of the public C++ API.
if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    target_include_directories(isaacsim-physics-manager-python PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../registration/bindings/python")
endif()

isaacsim_add_python_tests(
    MODULE isaacsim.physics.manager
    REQUIRES_BINDINGS
    RESOURCE_DIR resources/tests
)

# C++ tests for the physics-manager free-function API and simulation-registry
# surface. Registry state is process-global, so tests reset it per case.
isaacsim_add_cpp_tests(
    MODULE isaacsim.physics.manager
    SOURCES
        tests/cpp/TestStepEventSubscription.cpp
        tests/cpp/TestInitializeResult.cpp
        tests/cpp/TestSimulationInterface.cpp
        tests/cpp/TestSimulationRegistry.cpp
        tests/cpp/TestSimulatorSimulation.cpp
        tests/cpp/TestSimulatorSceneQuery.cpp
        tests/cpp/TestSimulatorInteraction.cpp
        tests/cpp/TestSimulatorBenchmark.cpp
        tests/cpp/TestRegistryEvents.cpp
        tests/cpp/TestMultiSimulationSubscriptions.cpp
        tests/cpp/TestTensorRegistry.cpp
        tests/cpp/TestTensorEntityView.cpp
    RESOURCE_DIR resources/tests
)
