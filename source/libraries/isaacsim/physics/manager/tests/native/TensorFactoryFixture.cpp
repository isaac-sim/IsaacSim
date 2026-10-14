// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/common/array/nanobind/ArrayCaster.hpp>
#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>
#include <nanobind/nanobind.h>

#include <memory>
#include <stdexcept>

namespace nb = nanobind;
using namespace isaacsim::physics::tensors;

namespace
{
class DerivedEntityView : public EntityView
{
};

std::shared_ptr<EntityView> retained;
std::weak_ptr<EntityView> observed;
constexpr const char* kEngine = "native_factory_gc_test";
}

NB_MODULE(_isaacsim_tensor_native_fixture, module)
{
    module.def("export_local_array",
               [](bool cuda, bool mutate)
               {
                   using namespace isaacsim::common::array;
                   Array local = Array(std::vector<int32_t>{ 1, 2, 3 }).toDevice(cuda ? Device::Cuda() : Device::Cpu());
                   nb::object exported = nb::cast(local);
                   if (mutate)
                       local.set(std::vector<int32_t>{ 4, 5, 6 });
                   return exported;
               });
    module.def("install",
               [](bool hold, bool derived)
               {
                   TensorRegistry::getInstance().registerEntity(kEngine, "view",
                                                                [hold, derived](const auto&, const auto&)
                                                                {
                                                                    if (hold && retained)
                                                                        return retained;
                                                                    std::shared_ptr<EntityView> view =
                                                                        derived ? std::make_shared<DerivedEntityView>() :
                                                                                  std::make_shared<EntityView>();
                                                                    observed = view;
                                                                    if (hold)
                                                                        retained = view;
                                                                    return view;
                                                                });
               });
    module.def("release", [] { retained.reset(); });
    module.def("owner_count", [] { return observed.use_count(); });
    module.def("read_metadata",
               []
               {
                   if (!retained)
                       throw std::logic_error("No native test owner is retained");
                   return std::get<int64_t>(retained->getMetadata("count"));
               });
    module.def("reset",
               []
               {
                   retained.reset();
                   TensorRegistry::getInstance().unregisterEntity(kEngine, "view");
               });
}
