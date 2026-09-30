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

#include "BindingsPhysics.hpp"
#include "TensorNbHelpers.hpp"

#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/IEntityView.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace manager_details = isaacsim::physics::manager::details;
using namespace isaacsim::physics::tensors;

namespace
{
// Wrap a Python callable as a C++ GetImplementationFunction. The wrapper converts the
// incoming tensor descriptions into ndarray views, calls into Python, and converts
// the returned ndarray back into a TensorDescription the manager can hand out to
// callers. The returned TensorDescription aliases the Python result's storage, so it
// carries a keepAlive pinning that Python object -- `data` stays valid until
// the last owner of the returned descriptor drops.
GetImplementationFunction wrapPythonGetCallback(nb::callable callback)
{
    return [callback = std::move(callback)](
               const TensorDescription& indices, const TensorDescription& output) -> TensorDescription
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object indicesObject = indices.isEmpty() ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        nb::object outputObject = output.isEmpty() ?
                                      nb::object{ nb::none() } :
                                      nb::cast(manager_details::convertTensorDescriptorToNumpy(output, nb::none()));
        nb::object result = callback(indicesObject, outputObject);
        if (result.is_none())
        {
            return TensorDescription{};
        }
        // The descriptor aliases the Python result's storage; pin the real owner so
        // `data` can't dangle once this lambda drops its reference.
        return manager_details::convertResultToTensorDescriptor(result, &output);
    };
}

SetImplementationFunction wrapPythonSetCallback(nb::callable callback)
{
    return [callback = std::move(callback)](const TensorDescription& data, const TensorDescription& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object dataObject = data.isEmpty() ?
                                    nb::object{ nb::none() } :
                                    nb::cast(manager_details::convertTensorDescriptorToNumpy(data, nb::none()));
        nb::object indicesObject = indices.isEmpty() ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        callback(dataObject, indicesObject);
    };
}

GetMultiImplementationFunction wrapPythonMultiGetCallback(nb::callable callback)
{
    return
        [callback = std::move(callback)](const TensorDescription& indices,
                                         const std::vector<TensorDescription>& output) -> std::vector<TensorDescription>
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object indicesObject = indices.isEmpty() ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        nb::list outputList;
        for (const TensorDescription& descriptor : output)
        {
            outputList.append(descriptor.isEmpty() ?
                                  nb::object{ nb::none() } :
                                  nb::cast(manager_details::convertTensorDescriptorToNumpy(descriptor, nb::none())));
        }
        nb::object result = callback(indicesObject, outputList);
        std::vector<TensorDescription> results;
        size_t outputIndex = 0;
        for (auto resultItem : nb::cast<nb::sequence>(result))
        {
            // Pin each output's real storage owner; each result aligns with the corresponding caller output slot.
            const TensorDescription* outputAlias = outputIndex < output.size() ? &output[outputIndex] : nullptr;
            results.push_back(manager_details::convertResultToTensorDescriptor(nb::borrow(resultItem), outputAlias));
            ++outputIndex;
        }
        return results;
    };
}

SetMultiImplementationFunction wrapPythonMultiSetCallback(nb::callable callback)
{
    return [callback = std::move(callback)](const std::vector<TensorDescription>& data, const TensorDescription& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::list dataList;
        for (const TensorDescription& descriptor : data)
        {
            dataList.append(descriptor.isEmpty() ?
                                nb::object{ nb::none() } :
                                nb::cast(manager_details::convertTensorDescriptorToNumpy(descriptor, nb::none())));
        }
        nb::object indicesObject = indices.isEmpty() ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        callback(dataList, indicesObject);
    };
}
} // namespace

void isaacsim::physics::manager::details::bindTensorViews(nb::module_& module)
{
    nb::class_<EntityView, IEntityView>(module, "EntityView", nb::dynamic_attr(),
                                        "Expose tensor operations for a resolved collection of physics entities.")
        .def(nb::init<>(), "Create an empty entity view.")
        .def(nb::init<std::vector<std::string>>(), nb::arg("paths"), "Create an entity view from prim path patterns.")
        .def_prop_ro("paths", &EntityView::getPrimPathPatterns, "Patterns used to create the view.")
        .def_prop_ro("resolved_prim_paths", &EntityView::getResolvedPrimPaths, "USD prim paths resolved by the engine.")
        .def_prop_ro("usd_stage_id", &EntityView::getUsdStageId, "Identifier of the view's USD stage.")
        .def_prop_rw("count", &EntityView::getEntityCount, &EntityView::setEntityCount, "Number of entities in the view.")
        .def_prop_rw("device_ordinal", &EntityView::getDeviceOrdinal, &EntityView::setDeviceOrdinal,
                     "Ordinal of the device holding this view's tensors, or -1 for host memory.")
        .def(
            "_register_impl",
            [](EntityView& self, const std::string& operationName, ImplementationKind kind, nb::callable callback,
               TensorSpecification specification)
            {
                // Forward to whichever overload of registerImplementation matches. The
                // single-tensor variants are the common case; multi-tensor
                // variants are exposed via separate methods below.
                if (kind == ImplementationKind::eGet)
                {
                    return self.registerImplementation(
                        operationName, kind, wrapPythonGetCallback(std::move(callback)), std::move(specification));
                }
                return self.registerImplementation(
                    operationName, kind, wrapPythonSetCallback(std::move(callback)), std::move(specification));
            },
            nb::arg("impl"), nb::arg("kind"), nb::arg("fn"), nb::arg("spec") = TensorSpecification{},
            "Register a single-buffer tensor operation.")
        .def(
            "_register_multi_impl",
            [](EntityView& self, const std::string& operationName, ImplementationKind kind, nb::callable callback,
               TensorSpecification specification)
            {
                if (kind == ImplementationKind::eGet)
                {
                    return self.registerImplementation(
                        operationName, kind, wrapPythonMultiGetCallback(std::move(callback)), std::move(specification));
                }
                return self.registerImplementation(
                    operationName, kind, wrapPythonMultiSetCallback(std::move(callback)), std::move(specification));
            },
            nb::arg("impl"), nb::arg("kind"), nb::arg("fn"), nb::arg("spec") = TensorSpecification{},
            "Register a multi-buffer tensor operation.")
        .def(
            "_register_metadata",
            [](EntityView& self, const std::string& operationName, nb::callable callback)
            {
                return self.registerMetadata(operationName,
                                             [callback = std::move(callback)]() -> Metadata
                                             {
                                                 nb::gil_scoped_acquire globalInterpreterLockGuard;
                                                 nb::object metadataObject = callback();
                                                 if (metadataObject.is_none())
                                                 {
                                                     return Metadata{};
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<bool>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<int64_t>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<double>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<std::string>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<std::vector<std::string>>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 try
                                                 {
                                                     return Metadata{ nb::cast<std::vector<int64_t>>(metadataObject) };
                                                 }
                                                 catch (const nb::cast_error&)
                                                 {
                                                 }
                                                 return Metadata{};
                                             });
            },
            nb::arg("impl"), nb::arg("fn"), "Register a metadata provider.")
        .def("list_impls", &EntityView::listImplementations, nb::arg("kind"),
             "Return the registered operation names for an operation kind.")
        .def("has_impl", &EntityView::hasImplementation, nb::arg("impl"), nb::arg("kind"),
             "Return whether an operation is registered.")
        .def("get_impl_spec", &EntityView::getImplementationSpecification, nb::arg("impl"), nb::arg("kind"),
             "Return the tensor specification for a single-buffer operation.")
        .def("get_impl_spec_multi", &EntityView::getMultiImplementationSpecifications, nb::arg("impl"), nb::arg("kind"),
             "Return the tensor specifications for a multi-buffer operation.")
        .def("get_metadata", &EntityView::getMetadata, nb::arg("impl"), "Return metadata supplied by the named provider.")
        .def(
            "get_data",
            [](nb::object self, const std::string& operationName, std::optional<nb::ndarray<>> indices,
               nb::object outputObject) -> nb::object
            {
                EntityView& view = nb::cast<EntityView&>(self);
                TensorDescription indicesDescriptor =
                    indices ? manager_details::convertArrayToTensorDescriptor(*indices) : TensorDescription{};
                manager_details::validateInt32IndexDataType(indicesDescriptor, "get_data");
                const bool hasOutput = !outputObject.is_none();
                TensorDescription outputDescriptor =
                    hasOutput ? manager_details::convertArrayToTensorDescriptor(nb::cast<nb::ndarray<>>(outputObject)) :
                                TensorDescription{};
                TensorDescription resultDescriptor = view.getData(operationName, indicesDescriptor, outputDescriptor);
                if (resultDescriptor.isEmpty())
                {
                    return nb::none();
                }
                // Hand back a generic DLPack-typed ndarray; the Python wrapper at
                // the public surface (`bindings/__init__.py`) dispatches through
                // `frontend.wrap_tensor(...)` to the caller's native tensor type.
                //
                // Retain the owning Python object so the returned view can't dangle:
                // the caller's `out` when the result aliases it (explicit-output
                // path), otherwise the EntityView itself -- an internal result
                // aliases a buffer owned by the view's stored implementation closure, freed
                // with the view.
                nb::object owner;
                if (resultDescriptor.keepAlive)
                {
                    // The result owns its buffer (per-call storage); pin it via a capsule
                    // so the array frees the buffer when dropped, independent of the view.
                    std::shared_ptr<void>* keepaliveHolder = new std::shared_ptr<void>(resultDescriptor.keepAlive);
                    owner = nb::capsule(keepaliveHolder, [](void* pointer) noexcept
                                        { delete static_cast<std::shared_ptr<void>*>(pointer); });
                }
                else
                {
                    owner = (hasOutput && resultDescriptor.data == outputDescriptor.data) ? outputObject : self;
                }
                return nb::cast(manager_details::convertTensorDescriptorToArray(resultDescriptor, owner));
            },
            nb::arg("impl"), nb::arg("indices") = nb::none(), nb::arg("out") = nb::none(),
            "Read a tensor operation, optionally for selected entity indices.")
        .def(
            "set_data",
            [](EntityView& self, const std::string& operationName, nb::ndarray<> data, std::optional<nb::ndarray<>> indices)
            {
                TensorDescription dataDescriptor = manager_details::convertArrayToTensorDescriptor(data);
                TensorDescription indicesDescriptor =
                    indices ? manager_details::convertArrayToTensorDescriptor(*indices) : TensorDescription{};
                manager_details::validateInt32IndexDataType(indicesDescriptor, "set_data");
                self.setData(operationName, dataDescriptor, indicesDescriptor);
            },
            nb::arg("impl"), nb::arg("data"), nb::arg("indices") = nb::none(),
            "Write a tensor operation, optionally for selected entity indices.")
        .def(
            "get_data_multi",
            [](nb::object self, const std::string& operationName, std::optional<nb::ndarray<>> indices,
               std::vector<nb::object> outputObjects)
            {
                EntityView& view = nb::cast<EntityView&>(self);
                TensorDescription indicesDescriptor =
                    indices ? manager_details::convertArrayToTensorDescriptor(*indices) : TensorDescription{};
                manager_details::validateInt32IndexDataType(indicesDescriptor, "get_data_multi");
                std::vector<TensorDescription> outputDescriptors;
                outputDescriptors.reserve(outputObjects.size());
                for (const nb::object& output : outputObjects)
                {
                    outputDescriptors.push_back(output.is_none() ? TensorDescription{} :
                                                                   manager_details::convertArrayToTensorDescriptor(
                                                                       nb::cast<nb::ndarray<>>(output)));
                }
                std::vector<TensorDescription> resultDescriptors =
                    view.getDataMulti(operationName, indicesDescriptor, outputDescriptors);
                nb::list results;
                for (const TensorDescription& descriptor : resultDescriptors)
                {
                    if (descriptor.isEmpty())
                    {
                        results.append(nb::none());
                        continue;
                    }
                    // Retain the backing so a returned array can't outlive its
                    // storage: a per-call owner (from wrapPythonMultiGetCallback) pinned via a
                    // capsule, else the matching supplied `out` when the result
                    // aliases it, else the EntityView (an internal result aliases a
                    // buffer owned by the view's implementation closure).
                    nb::object owner;
                    if (descriptor.keepAlive)
                    {
                        std::shared_ptr<void>* keepaliveHolder = new std::shared_ptr<void>(descriptor.keepAlive);
                        owner = nb::capsule(keepaliveHolder, [](void* pointer) noexcept
                                            { delete static_cast<std::shared_ptr<void>*>(pointer); });
                    }
                    else
                    {
                        owner = self;
                        for (size_t outputIndex = 0; outputIndex < outputObjects.size(); ++outputIndex)
                        {
                            if (!outputObjects[outputIndex].is_none() &&
                                outputDescriptors[outputIndex].data == descriptor.data)
                            {
                                owner = outputObjects[outputIndex];
                                break;
                            }
                        }
                    }
                    results.append(nb::cast(manager_details::convertTensorDescriptorToArray(descriptor, owner)));
                }
                return results;
            },
            nb::arg("impl"), nb::arg("indices") = nb::none(), nb::arg("out") = std::vector<nb::object>{},
            "Read a multi-buffer tensor operation.")
        .def(
            "set_data_multi",
            [](EntityView& self, const std::string& operationName, std::vector<nb::ndarray<>> data,
               std::optional<nb::ndarray<>> indices)
            {
                std::vector<TensorDescription> dataDescriptors;
                dataDescriptors.reserve(data.size());
                for (const nb::ndarray<>& array : data)
                {
                    dataDescriptors.push_back(manager_details::convertArrayToTensorDescriptor(array));
                }
                TensorDescription indicesDescriptor =
                    indices ? manager_details::convertArrayToTensorDescriptor(*indices) : TensorDescription{};
                manager_details::validateInt32IndexDataType(indicesDescriptor, "set_data_multi");
                self.setDataMulti(operationName, dataDescriptors, indicesDescriptor);
            },
            nb::arg("impl"), nb::arg("data"), nb::arg("indices") = nb::none(), "Write a multi-buffer tensor operation.");
}
