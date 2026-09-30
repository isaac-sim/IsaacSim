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
    NAME isaacsim.physics.registration
    EXPORT_HEADER isaacsim/physics/registration/Export.h
    EXPORT_MACRO ISAACSIM_PHYSICS_REGISTRATION_API
    PUBLIC_CPP_HEADERS
        isaacsim/physics/registration/Types.hpp
        isaacsim/physics/registration/Physics.hpp
        isaacsim/physics/registration/simulator/Simulator.hpp
        isaacsim/physics/registration/simulator/Simulation.hpp
        isaacsim/physics/registration/simulator/Benchmark.hpp
        isaacsim/physics/registration/simulator/Interaction.hpp
        isaacsim/physics/registration/simulator/InteractionTools.hpp
        isaacsim/physics/registration/simulator/SceneQuery.hpp
        isaacsim/physics/registration/simulator/ContactEvent.hpp
        isaacsim/physics/registration/tensors/TensorTypes.hpp
        isaacsim/physics/registration/tensors/TensorDescription.hpp
        isaacsim/physics/registration/tensors/TensorSpecification.hpp
        isaacsim/physics/registration/tensors/PhysicsEnums.hpp
        isaacsim/physics/registration/tensors/Metadata.hpp
        isaacsim/physics/registration/tensors/IEntityView.hpp
        isaacsim/physics/registration/tensors/TensorRegistry.hpp
    SOURCES
        src/SimulationRegistry.hpp
        src/Physics.cpp
        src/SimulationRegistry.cpp
        src/tensors/TensorRegistry.cpp
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
)

# BindingsRegistrationPython.cpp is the NB_MODULE; the tensor register API compiles
# as a separate TU and is wired in via bindTensors(m).
isaacsim_add_nanobind(
    MODULE isaacsim.physics.registration
    SOURCES
        bindings/python/BindingsRegistrationPython.cpp
        bindings/python/BindingsTensorsPython.cpp
)
