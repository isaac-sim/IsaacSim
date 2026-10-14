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

#include <isaacsim/common/array/nanobind/ArrayCaster.hpp>
#include <isaacsim/common/array/nanobind/DeviceCaster.hpp>
#include <isaacsim/common/array/nanobind/DtypeCaster.hpp>
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

#include <algorithm>
#include <unordered_map>

namespace nb = nanobind;
namespace manager_details = isaacsim::physics::manager::details;
using namespace isaacsim::physics::tensors;

namespace
{
// Native std::function captures are invisible to Python's cyclic collector. The
// wrapper-lifetime registry holds only weak records; native callbacks remain strong owners.
struct CallbackOwners
{
    std::vector<std::weak_ptr<nb::callable>> callbacks;
    std::weak_ptr<IEntityView> nativeOwner;
};
using CallbackRegistry = std::unordered_map<PyObject*, CallbackOwners>;
auto callbackRegistry = std::make_shared<CallbackRegistry>();
Py_ssize_t entityViewDictionaryOffset = 0;

CallbackOwners* findCallbackOwners(PyObject* self) noexcept
{
    const auto found = callbackRegistry->find(self);
    return found == callbackRegistry->end() ? nullptr : &found->second;
}

PyObject** entityViewDictionary(PyObject* self) noexcept
{
    return reinterpret_cast<PyObject**>(reinterpret_cast<char*>(self) + entityViewDictionaryOffset);
}

bool ownsCallbackEdges(PyObject* self, const CallbackOwners& owners) noexcept
{
    // A tracked shared_ptr wrapper contributes exactly one native strong owner.
    // Any additional native owners are external roots. Do not lock the weak_ptr:
    // that would itself change the count being inspected.
    return nb::inst_state(self).second || owners.nativeOwner.use_count() == 1;
}

int traverseEntityView(PyObject* self, visitproc visit, void* arg)
{
    PyObject* dictionary = *entityViewDictionary(self);
    Py_VISIT(dictionary);
    Py_VISIT(Py_TYPE(self));
    if (auto* owners = findCallbackOwners(self); owners && ownsCallbackEdges(self, *owners))
        for (const auto& weak : owners->callbacks)
            if (auto owner = weak.lock())
                Py_VISIT(owner->ptr());
    return 0;
}

int clearEntityView(PyObject* self)
{
    PyObject** dictionary = entityViewDictionary(self);
    if (auto* owners = findCallbackOwners(self); owners && ownsCallbackEdges(self, *owners))
    {
        // Detach before decref: user finalizers can mutate the dictionary.
        std::vector<std::weak_ptr<nb::callable>> pending;
        pending.swap(owners->callbacks);
        for (const auto& weak : pending)
            if (auto owner = weak.lock())
                owner->release().dec_ref();
    }
    Py_CLEAR(*dictionary);
    return 0;
}

PyType_Slot entityViewGcSlots[] = {
    { Py_tp_traverse, reinterpret_cast<void*>(traverseEntityView) },
    { Py_tp_clear, reinterpret_cast<void*>(clearEntityView) },
    { 0, nullptr },
};

CallbackOwners& callbackOwners(nb::handle self)
{
    const auto [entry, inserted] = callbackRegistry->try_emplace(self.ptr());
    if (inserted)
    {
        // No Python references live in this registry. Tie erasure to actual wrapper
        // destruction, independently of its mutable __dict__, using nanobind's
        // same payload-lifetime facility used by its shared_ptr caster.
        struct Lifetime
        {
            std::shared_ptr<CallbackRegistry> registry;
            PyObject* instance;
        };
        auto lifetime = std::make_unique<Lifetime>(Lifetime{ callbackRegistry, self.ptr() });
        nb::detail::keep_alive(self.ptr(), lifetime.get(),
                               [](void* pointer) noexcept
                               {
                                   auto* held = static_cast<Lifetime*>(pointer);
                                   held->registry->erase(held->instance);
                                   delete held;
                               });
        lifetime.release();
    }
    return entry->second;
}

std::shared_ptr<nb::callable> retainPythonCallable(EntityView& view, nb::callable callback)
{
    auto owner = std::shared_ptr<nb::callable>(new nb::callable(std::move(callback)),
                                               [](nb::callable* value) noexcept
                                               {
                                                   nb::gil_scoped_acquire gil;
                                                   delete value;
                                               });
    auto& owners = callbackOwners(nb::find(&view)).callbacks;
    // One record per shared state, even when legacy registration copies that
    // state into both native dispatch maps. Replacements discard expired records.
    owners.erase(
        std::remove_if(owners.begin(), owners.end(), [](const auto& value) { return value.expired(); }), owners.end());
    owners.push_back(owner);
    return owner;
}

template <typename... Arguments>
nb::object invokePythonCallback(const std::shared_ptr<nb::callable>& callback, Arguments&&... arguments)
{
    if (!callback->is_valid())
        throw std::runtime_error("Python tensor callback was cleared by cyclic garbage collection");
    return (*callback)(std::forward<Arguments>(arguments)...);
}

void validateArrayIndexDataType(const std::optional<array::Array>& indices, const char* operation)
{
    if (!indices.has_value() || indices->dtype().isFloating() || indices->dtype() == array::Dtype::Bool())
    {
        return;
    }
    if (indices->dtype() != array::Dtype::Int32())
    {
        throw std::invalid_argument(std::string(operation) + ": indices must be int32 (the manager's convention)");
    }
}

GetArrayImplementationFunction wrapPythonArrayGetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const std::optional<array::Array>& indices, const std::optional<array::Array>& output) -> array::Array
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object result = invokePythonCallback(callback, nb::cast(indices), nb::cast(output));
        if (result.is_none())
        {
            throw std::runtime_error("get implementation returned None; a read must produce an array");
        }
        return nb::cast<array::Array>(result);
    };
}

SetArrayImplementationFunction wrapPythonArraySetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const array::Array& data, const std::optional<array::Array>& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        invokePythonCallback(callback, nb::cast(data), nb::cast(indices));
    };
}

GetArrayMultiImplementationFunction wrapPythonArrayMultiGetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const std::optional<array::Array>& indices,
               const std::optional<std::vector<array::Array>>& output) -> std::vector<array::Array>
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object result = invokePythonCallback(callback, nb::cast(indices), nb::cast(output));
        return nb::cast<std::vector<array::Array>>(result);
    };
}

SetArrayMultiImplementationFunction wrapPythonArrayMultiSetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const std::vector<array::Array>& data, const std::optional<array::Array>& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        invokePythonCallback(callback, nb::cast(data), nb::cast(indices));
    };
}


bool isOmittedTensor(const TensorDescription& descriptor) noexcept
{
    return descriptor.data == nullptr && descriptor.shape.empty();
}

// Wrap a Python callable as a C++ GetImplementationFunction. The wrapper converts the
// incoming tensor descriptions into ndarray views, calls into Python, and converts
// the returned ndarray back into a TensorDescription the manager can hand out to
// callers. The returned TensorDescription aliases the Python result's storage, so it
// carries a keepAlive pinning that Python object -- `data` stays valid until
// the last owner of the returned descriptor drops.
GetImplementationFunction wrapPythonGetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const TensorDescription& indices, const TensorDescription& output) -> TensorDescription
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object indicesObject = isOmittedTensor(indices) ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        nb::object outputObject = isOmittedTensor(output) ?
                                      nb::object{ nb::none() } :
                                      nb::cast(manager_details::convertTensorDescriptorToNumpy(output, nb::none()));
        nb::object result = invokePythonCallback(callback, indicesObject, outputObject);
        if (result.is_none())
        {
            return TensorDescription{};
        }
        // The descriptor aliases the Python result's storage; pin the real owner so
        // `data` can't dangle once this lambda drops its reference.
        return manager_details::convertResultToTensorDescriptor(result, &output);
    };
}

SetImplementationFunction wrapPythonSetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const TensorDescription& data, const TensorDescription& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object dataObject = isOmittedTensor(data) ?
                                    nb::object{ nb::none() } :
                                    nb::cast(manager_details::convertTensorDescriptorToNumpy(data, nb::none()));
        nb::object indicesObject = isOmittedTensor(indices) ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        invokePythonCallback(callback, dataObject, indicesObject);
    };
}

GetMultiImplementationFunction wrapPythonMultiGetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const TensorDescription& indices,
               const std::vector<TensorDescription>& output) -> std::vector<TensorDescription>
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::object indicesObject = isOmittedTensor(indices) ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        nb::list outputList;
        for (const TensorDescription& descriptor : output)
        {
            outputList.append(isOmittedTensor(descriptor) ?
                                  nb::object{ nb::none() } :
                                  nb::cast(manager_details::convertTensorDescriptorToNumpy(descriptor, nb::none())));
        }
        nb::object result = invokePythonCallback(callback, indicesObject, outputList);
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

SetMultiImplementationFunction wrapPythonMultiSetCallback(EntityView& self, nb::callable callback)
{
    return [callback = retainPythonCallable(self, std::move(callback))](
               const std::vector<TensorDescription>& data, const TensorDescription& indices)
    {
        nb::gil_scoped_acquire globalInterpreterLockGuard;
        nb::list dataList;
        for (const TensorDescription& descriptor : data)
        {
            dataList.append(isOmittedTensor(descriptor) ?
                                nb::object{ nb::none() } :
                                nb::cast(manager_details::convertTensorDescriptorToNumpy(descriptor, nb::none())));
        }
        nb::object indicesObject = isOmittedTensor(indices) ?
                                       nb::object{ nb::none() } :
                                       nb::cast(manager_details::convertTensorDescriptorToNumpy(indices, nb::none()));
        invokePythonCallback(callback, dataList, indicesObject);
    };
}
} // namespace

nb::object isaacsim::physics::manager::details::castTensorEntity(const std::shared_ptr<IEntityView>& value)
{
    if (!value)
        return nb::none();
    const bool hasWrapper = nb::find(value.get()).is_valid();
    // Engine subclasses need not have their own Python registration. Cast through the bound concrete base.
    const auto entityView = std::dynamic_pointer_cast<EntityView>(value);
    nb::object result = entityView ? nb::cast(entityView) : nb::cast(value);
    // Only a newly-created shared_ptr caster wrapper has a known native keepalive.
    // Existing raw-reference wrappers must remain conservatively untracked.
    if (!hasWrapper && entityView && !nb::inst_state(result).second)
        callbackOwners(result).nativeOwner = value;
    return result;
}

void isaacsim::physics::manager::details::bindTensorViews(nb::module_& module)
{
    auto entityView =
        nb::class_<EntityView, IEntityView>(module, "EntityView", nb::dynamic_attr(), nb::type_slots(entityViewGcSlots),
                                            "Expose tensor operations for a resolved collection of physics entities.");
    entityViewDictionaryOffset = nb::cast<Py_ssize_t>(entityView.attr("__dictoffset__"));
    entityView.def(nb::init<>(), "Create an empty entity view.")
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
                        operationName, kind, wrapPythonGetCallback(self, std::move(callback)), std::move(specification));
                }
                return self.registerImplementation(
                    operationName, kind, wrapPythonSetCallback(self, std::move(callback)), std::move(specification));
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
                    return self.registerImplementation(operationName, kind,
                                                       wrapPythonMultiGetCallback(self, std::move(callback)),
                                                       std::move(specification));
                }
                return self.registerImplementation(
                    operationName, kind, wrapPythonMultiSetCallback(self, std::move(callback)), std::move(specification));
            },
            nb::arg("impl"), nb::arg("kind"), nb::arg("fn"), nb::arg("spec") = TensorSpecification{},
            "Register a multi-buffer tensor operation.")
        .def(
            "_register_impl",
            [](EntityView& self, const std::string& operationName, ImplementationKind kind, nb::callable callback,
               ArrayTensorSpecification specification)
            {
                // Forward to whichever overload of registerArrayImplementation matches. The
                // single-tensor variants are the common case; multi-tensor
                // variants are exposed via separate methods below.
                if (kind == ImplementationKind::eGet)
                {
                    return self.registerArrayImplementation(operationName, kind,
                                                            wrapPythonArrayGetCallback(self, std::move(callback)),
                                                            std::move(specification));
                }
                return self.registerArrayImplementation(
                    operationName, kind, wrapPythonArraySetCallback(self, std::move(callback)), std::move(specification));
            },
            nb::arg("impl"), nb::arg("kind"), nb::arg("fn"), nb::arg("spec"),
            "Register a single-buffer tensor operation.")
        .def(
            "_register_multi_impl",
            [](EntityView& self, const std::string& operationName, ImplementationKind kind, nb::callable callback,
               ArrayTensorSpecification specification)
            {
                if (kind == ImplementationKind::eGet)
                {
                    return self.registerArrayImplementation(operationName, kind,
                                                            wrapPythonArrayMultiGetCallback(self, std::move(callback)),
                                                            std::move(specification));
                }
                return self.registerArrayImplementation(operationName, kind,
                                                        wrapPythonArrayMultiSetCallback(self, std::move(callback)),
                                                        std::move(specification));
            },
            nb::arg("impl"), nb::arg("kind"), nb::arg("fn"), nb::arg("spec"), "Register a multi-buffer tensor operation.")
        .def_prop_rw("device", &EntityView::getDevice, &EntityView::setDevice, "Device holding this view's tensors.")
        .def(
            "get_array_data",
            [](EntityView& self, const std::string& operationName, const std::optional<array::Array>& indices,
               const std::optional<array::Array>& output) -> array::Array
            {
                validateArrayIndexDataType(indices, "get_array_data");
                return self.getArrayData(operationName, indices, output);
            },
            nb::arg("impl"), nb::arg("indices") = nb::none(), nb::arg("out") = nb::none(),
            "Read a tensor operation, optionally for selected entity indices.")
        .def(
            "set_array_data",
            [](EntityView& self, const std::string& operationName, const array::Array& data,
               const std::optional<array::Array>& indices)
            {
                validateArrayIndexDataType(indices, "set_array_data");
                self.setArrayData(operationName, data, indices);
            },
            nb::arg("impl"), nb::arg("data"), nb::arg("indices") = nb::none(),
            "Write a tensor operation, optionally for selected entity indices.")
        .def(
            "get_array_data_multi",
            [](EntityView& self, const std::string& operationName, const std::optional<array::Array>& indices,
               const std::optional<std::vector<array::Array>>& output) -> std::vector<array::Array>
            {
                validateArrayIndexDataType(indices, "get_array_data_multi");
                return self.getArrayDataMulti(operationName, indices, output);
            },
            nb::arg("impl"), nb::arg("indices") = nb::none(), nb::arg("out") = nb::none(),
            "Read a multi-buffer tensor operation.")
        .def(
            "set_array_data_multi",
            [](EntityView& self, const std::string& operationName, const std::vector<array::Array>& data,
               const std::optional<array::Array>& indices)
            {
                validateArrayIndexDataType(indices, "set_array_data_multi");
                self.setArrayDataMulti(operationName, data, indices);
            },
            nb::arg("impl"), nb::arg("data"), nb::arg("indices") = nb::none(), "Write a multi-buffer tensor operation.")
        .def("get_array_impl_spec", &EntityView::getArrayImplementationSpecification, nb::arg("impl"), nb::arg("kind"),
             "Return the Array transport specification.")
        .def("get_array_impl_spec_multi", &EntityView::getArrayMultiImplementationSpecifications, nb::arg("impl"),
             nb::arg("kind"), "Return Array transport specifications.")
        .def(
            "_register_metadata",
            [](EntityView& self, const std::string& operationName, nb::callable callback)
            {
                return self.registerMetadata(operationName,
                                             [callback = retainPythonCallable(self, std::move(callback))]() -> Metadata
                                             {
                                                 nb::gil_scoped_acquire globalInterpreterLockGuard;
                                                 nb::object metadataObject = invokePythonCallback(callback);
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
                if (isOmittedTensor(resultDescriptor))
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
                    if (isOmittedTensor(descriptor))
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
