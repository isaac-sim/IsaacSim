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

#include "TensorNbHelpers.hpp"

#include <isaacsim/physics/registration/tensors/IEntityView.hpp>
#include <isaacsim/physics/registration/tensors/Metadata.hpp>
#include <isaacsim/physics/registration/tensors/PhysicsEnums.hpp>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>
#include <isaacsim/physics/registration/tensors/TensorRegistry.hpp>
#include <isaacsim/physics/registration/tensors/TensorSpecification.hpp>
#include <isaacsim/physics/registration/tensors/TensorTypes.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace isaacsim
{
namespace physics
{
namespace registration
{
namespace details
{

using tensors::DeviceKind;
using tensors::DofDriveType;
using tensors::DofMotion;
using tensors::DofType;
using tensors::DType;
using tensors::EntityFactory;
using tensors::EntityOptions;
using tensors::IEntityView;
using tensors::ImplementationKind;
using tensors::JointType;
using tensors::ObjectType;
using tensors::TensorDescription;
using tensors::TensorRegistry;
using tensors::TensorSpecification;

namespace
{
// Adapt a Python entity factory to the C++ signature. The options map is handed over as a dict, or as None when
// the caller supplied none, so a factory that ignores options can declare a plain default argument.
EntityFactory wrapPythonEntityFactory(nb::callable factory)
{
    return [factory = std::move(factory)](const std::vector<std::string>& paths,
                                          const std::optional<EntityOptions>& options) -> std::shared_ptr<IEntityView>
    {
        nb::gil_scoped_acquire globalInterpreterLockAcquire;
        nb::object object = factory(paths, options);
        if (object.is_none())
        {
            return nullptr;
        }
        return nb::cast<std::shared_ptr<IEntityView>>(object);
    };
}
} // namespace

void bindTensors(nb::module_& module)
{
    nb::enum_<DType>(module, "DType", "Element types supported by the tensor API.")
        .value("UNKNOWN", DType::eUnknown, "Unknown or unspecified element type.")
        .value("FLOAT32", DType::eFloat32, "32-bit floating-point elements.")
        .value("FLOAT64", DType::eFloat64, "64-bit floating-point elements.")
        .value("INT8", DType::eInt8, "8-bit signed integer elements.")
        .value("INT16", DType::eInt16, "16-bit signed integer elements.")
        .value("INT32", DType::eInt32, "32-bit signed integer elements.")
        .value("INT64", DType::eInt64, "64-bit signed integer elements.")
        .value("UINT8", DType::eUInt8, "8-bit unsigned integer elements.")
        .value("UINT16", DType::eUInt16, "16-bit unsigned integer elements.")
        .value("UINT32", DType::eUInt32, "32-bit unsigned integer elements.")
        .value("UINT64", DType::eUInt64, "64-bit unsigned integer elements.")
        .value("BOOL", DType::eBool, "Boolean elements.")
        .export_values();

    nb::enum_<DeviceKind>(module, "DeviceKind", "Device-placement requirements for a tensor implementation.")
        .value("CPU", DeviceKind::eCpu, "Require host memory.")
        .value("GPU", DeviceKind::eGpu, "Require device memory.")
        .value("ENGINE_DEFAULT", DeviceKind::eEngineDefault, "Use the physics engine's preferred placement.")
        .export_values();

    nb::enum_<ImplementationKind>(module, "ImplKind", "Operation kinds implemented by an entity view.")
        .value("Get", ImplementationKind::eGet, "Read operation.")
        .value("Set", ImplementationKind::eSet, "Write operation.")
        .export_values();

    // Physics-domain enums shared by tensor backends.

    nb::enum_<ObjectType>(module, "ObjectType", "Object types in the physics scene.")
        .value("Invalid", ObjectType::eInvalid, "Invalid or unknown object type.")
        .value("RigidBody", ObjectType::eRigidBody, "Rigid body.")
        .value("Articulation", ObjectType::eArticulation, "Articulation.")
        .value("ArticulationLink", ObjectType::eArticulationLink, "Non-root articulation link.")
        .value("ArticulationRootLink", ObjectType::eArticulationRootLink, "Root articulation link.")
        .value("ArticulationJoint", ObjectType::eArticulationJoint, "Articulation joint.")
        .export_values();

    nb::enum_<JointType>(module, "JointType", "Joint types.")
        .value("Invalid", JointType::eInvalid, "Invalid or unknown joint type.")
        .value("Fixed", JointType::eFixed, "Fixed joint.")
        .value("Revolute", JointType::eRevolute, "Revolute joint.")
        .value("Prismatic", JointType::ePrismatic, "Prismatic joint.")
        .value("Spherical", JointType::eSpherical, "Spherical joint.")
        .export_values();

    nb::enum_<DofType>(module, "DofType", "Degree-of-freedom types.")
        .value("Invalid", DofType::eInvalid, "Invalid or unknown degree-of-freedom type.")
        .value("Rotation", DofType::eRotation, "Rotational degree of freedom.")
        .value("Translation", DofType::eTranslation, "Translational degree of freedom.")
        .export_values();

    nb::enum_<DofMotion>(module, "DofMotion", "Degree-of-freedom motion constraints.")
        .value("Invalid", DofMotion::eInvalid, "Invalid or unknown motion constraint.")
        .value("Free", DofMotion::eFree, "Unconstrained motion.")
        .value("Limited", DofMotion::eLimited, "Motion constrained by limits.")
        .value("Locked", DofMotion::eLocked, "Locked motion.")
        .export_values();

    nb::enum_<DofDriveType>(module, "DofDriveType", "Degree-of-freedom drive types.")
        .value("None_", DofDriveType::eNone, "No drive.")
        .value("Force", DofDriveType::eForce, "Force drive.")
        .value("Acceleration", DofDriveType::eAcceleration, "Acceleration drive.")
        .export_values();

    // Float3 is registered once by the physics bindings (manager::Float3); the
    // tensor data plane's `Float3` aliases it, so it is not re-registered here.

    // ----- TensorSpecification ---------------------------------------------------

    nb::class_<TensorSpecification>(
        module, "TensorSpec", "Describe a tensor operation's shape, type, placement, and capabilities.")
        .def(nb::init<>(), "Create a tensor specification with default capabilities.")
        .def(
            "__init__",
            [](TensorSpecification* self, DType dtype, std::vector<int64_t> shapeHint, DeviceKind deviceKind,
               bool supports, bool supportsIndexedRead, bool supportsIndexedWrite, bool supportsMaskedWrite)
            {
                new (self) TensorSpecification{};
                self->dtype = dtype;
                self->shapeHint = std::move(shapeHint);
                self->deviceKind = deviceKind;
                self->supports = supports;
                self->supportsIndexedRead = supportsIndexedRead;
                self->supportsIndexedWrite = supportsIndexedWrite;
                self->supportsMaskedWrite = supportsMaskedWrite;
            },
            nb::arg("dtype") = DType::eFloat32, nb::arg("shape_hint") = std::vector<int64_t>{},
            nb::arg("device_kind") = DeviceKind::eEngineDefault, nb::arg("supports") = true,
            nb::arg("supports_indexed_read") = false, nb::arg("supports_indexed_write") = false,
            nb::arg("supports_masked_write") = false, "Create a tensor specification from capability fields.")
        .def_rw("dtype", &TensorSpecification::dtype, "Required tensor element type.")
        .def_rw("shape_hint", &TensorSpecification::shapeHint, "Expected tensor dimensions, excluding the view dimension.")
        .def_rw("device_kind", &TensorSpecification::deviceKind, "Required tensor device placement.")
        .def_rw("supports", &TensorSpecification::supports, "Whether the operation is supported.")
        .def_rw("supports_indexed_read", &TensorSpecification::supportsIndexedRead,
                "Whether the operation supports indexed reads.")
        .def_rw("supports_indexed_write", &TensorSpecification::supportsIndexedWrite,
                "Whether the operation supports indexed writes.")
        .def_rw("supports_masked_write", &TensorSpecification::supportsMaskedWrite,
                "Whether the operation supports masked writes.")
        .def_rw("requires_host_data", &TensorSpecification::requiresHostData,
                "Whether write data must be available in host memory.");

    // ----- TensorDescription ---------------------------------------------------
    //
    // Constructible from a Python ndarray so engines that need to forward an
    // ndarray as a Python `TensorDesc` (e.g. for indexed-read fixtures) can do so.

    nb::class_<TensorDescription>(module, "TensorDesc", "Describe a tensor buffer and retain its backing storage.")
        .def(nb::init<>(), "Create an empty tensor descriptor.")
        .def(
            "__init__",
            [](TensorDescription* self, nb::ndarray<> array)
            {
                // Retain the wrapped array so a Python `TensorDesc` built from a temporary
                // (e.g. `TensorDesc(np.array(...))` returned by a get implementation)
                // keeps its storage alive, not just the raw pointer.
                TensorDescription descriptor = convertArrayToTensorDescriptor(array);
                if (!descriptor.isEmpty())
                {
                    descriptor.keepAlive = createArrayKeepAlive(std::move(array));
                }
                new (self) TensorDescription(std::move(descriptor));
            },
            nb::arg("array"), "Create a descriptor that keeps the source array alive.")
        .def_prop_ro(
            "dtype", [](const TensorDescription& descriptor) { return descriptor.dtype; }, "Tensor element type.")
        .def_prop_ro(
            "shape", [](const TensorDescription& descriptor) { return descriptor.shape; }, "Tensor dimensions.")
        .def_prop_ro(
            "strides", [](const TensorDescription& descriptor) { return descriptor.strides; },
            "Tensor strides measured in elements.")
        .def_prop_ro(
            "device", [](const TensorDescription& descriptor) { return descriptor.device; },
            "Device kind containing the tensor buffer.")
        .def_prop_ro(
            "device_ordinal", [](const TensorDescription& descriptor) { return descriptor.deviceOrdinal; },
            "Ordinal of the device containing the tensor buffer.")
        .def_prop_ro(
            "num_elements", [](const TensorDescription& descriptor) { return descriptor.computeElementCount(); },
            "Total number of tensor elements.")
        .def_prop_ro(
            "is_empty", [](const TensorDescription& descriptor) { return descriptor.isEmpty(); },
            "Whether the descriptor has no tensor buffer.")
        .def(
            "to_array",
            [](const TensorDescription& descriptor)
            {
                if (descriptor.isEmpty())
                {
                    return nb::object(nb::none());
                }
                // Retain the descriptor's backing so the returned array can't dangle:
                // a Python `TensorDesc` built from a temporary (`TensorDesc(np.array(...))`)
                // holds its source in `keepAlive`; pin it via a capsule so the array
                // owns the storage instead of aliasing a buffer that dies with the
                // temporary descriptor. Falls back to no owner when the descriptor carries
                // no keepAlive (engine-owned storage), matching prior behavior.
                nb::object owner = nb::none();
                if (descriptor.keepAlive)
                {
                    auto* retainedOwner = new std::shared_ptr<void>(descriptor.keepAlive);
                    owner = nb::capsule(retainedOwner, [](void* pointer) noexcept
                                        { delete static_cast<std::shared_ptr<void>*>(pointer); });
                }
                return nb::cast(convertTensorDescriptorToNumpy(descriptor, owner));
            },
            "Return an array view of the tensor buffer, or None when empty.");

    // ----- EntityView ---------------------------------------------------


    // Register-side base class engines implement; the concrete views deriving it
    // are bound in the manager module.
    nb::class_<IEntityView>(module, "IEntityView", "Base interface implemented by engine entity views.");

    nb::class_<TensorRegistry>(
        module, "TensorRegistry", "Process-wide registry that maps (engine, entity-name) to view factories.")
        .def_static("instance", &TensorRegistry::getInstance, nb::rv_policy::reference,
                    "Return the process-wide tensor registry.")
        .def(
            "register_entity",
            [](TensorRegistry& self, const std::string& engine, const std::string& entityName, nb::callable factory)
            { return self.registerEntity(engine, entityName, wrapPythonEntityFactory(std::move(factory))); },
            nb::arg("engine"), nb::arg("entity_name"), nb::arg("factory"),
            "Register an entity-view factory for an engine.")
        .def("register_engine", &TensorRegistry::registerEngine, nb::arg("engine"),
             "Declare that a physics engine is available. The declaration lasts for the life of the process; it "
             "describes what can be simulated, not what is being simulated.")
        .def("list_simulations", &TensorRegistry::listSimulations,
             "Return the simulation names that have registered factories, which is what create_entity accepts.")
        .def("unregister_entity", &TensorRegistry::unregisterEntity, nb::arg("engine"), nb::arg("entity_name"),
             "Unregister an entity-view factory.")
        .def("has_entity", &TensorRegistry::hasEntity, nb::arg("engine"), nb::arg("entity_name"),
             "Return whether an engine has an entity-view factory.")
        .def("list_engines", &TensorRegistry::listEngines, "Return the names of the declared physics engines.")
        .def("list_entities", &TensorRegistry::listEntities, nb::arg("engine"),
             "Return the entity-view types registered for an engine.")
        .def("clear_for_testing", &TensorRegistry::clearForTesting, "Remove all registered tensor factories.");

    module.def(
        "get_registry", []() -> TensorRegistry& { return TensorRegistry::getInstance(); }, nb::rv_policy::reference,
        "Return the process-wide tensor registry.");
    module.def(
        "register_entity",
        [](const std::string& engine, const std::string& entityName, nb::callable factory)
        {
            return TensorRegistry::getInstance().registerEntity(
                engine, entityName, wrapPythonEntityFactory(std::move(factory)));
        },
        nb::arg("engine"), nb::arg("entity_name"), nb::arg("factory"), "Register an entity-view factory for an engine.");
    // create_entity (the use side) lives in the manager module's bindings
    // (BindingsTensorCreate.cpp) per the register/use split.
}

} // namespace details
} // namespace registration
} // namespace physics
} // namespace isaacsim
