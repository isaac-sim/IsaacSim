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

#include "isaacsim/common/exceptions/Exceptions.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace isaacsim::common::exceptions;

namespace
{

/**
 * @brief Register a translator that raises a Python exception instance carrying the C++ exception's fields as
 * attributes, instead of nanobind's default message-only translation.
 * @tparam ExceptionType The C++ exception type to translate.
 * @tparam AttributeSetter A type exposing `static void apply(nb::object&, const ExceptionType&)`, copying
 * @p ExceptionType's fields onto the Python instance.
 * @param[in] pythonExceptionType The Python exception type previously created with nb::exception.
 */
template <typename ExceptionType, typename AttributeSetter>
void registerAttributeCarryingTranslator(nb::exception<ExceptionType>& pythonExceptionType)
{
    nb::register_exception_translator(
        [](const std::exception_ptr& originalException, void* payload)
        {
            try
            {
                std::rethrow_exception(originalException);
            }
            catch (const ExceptionType& error)
            {
                PyObject* exceptionType = static_cast<PyObject*>(payload);
                nb::object instance = nb::borrow(exceptionType)(error.what());
                AttributeSetter::apply(instance, error);
                PyErr_SetObject(exceptionType, instance.ptr());
            }
        },
        pythonExceptionType.ptr());
}

struct PrimPathErrorAttributeSetter
{
    static void apply(nb::object& instance, const PrimPathError& error)
    {
        instance.attr("prim_path") = error.getPrimPath();
    }
};

struct PrimPathStringErrorAttributeSetter
{
    static void apply(nb::object& instance, const PrimPathStringError& error)
    {
        instance.attr("prim_path") = error.getPrimPath();
    }
};

struct AttributeNameErrorAttributeSetter
{
    static void apply(nb::object& instance, const AttributeNameError& error)
    {
        instance.attr("attribute_name") = error.getAttributeName();
        instance.attr("valid_attribute_names") = error.getValidAttributeNames();
    }
};

struct ValueTypeErrorAttributeSetter
{
    static void apply(nb::object& instance, const ValueTypeError& error)
    {
        instance.attr("attribute_name") = error.getAttributeName();
        instance.attr("expected_type") = error.getExpectedType();
        instance.attr("actual_type") = error.getActualType();
    }
};

} // namespace

NB_MODULE(_bindings, module)
{
    module.doc() = "Custom exceptions raised by Isaac Sim.";

    // Common base class for every Isaac Sim exception.
    nb::exception<IsaacSimException> exception(module, "IsaacSimException");

    // Raised when a USD prim path refers to an invalid prim.
    nb::exception<PrimPathError> primPathError(module, "PrimPathError", exception);
    registerAttributeCarryingTranslator<PrimPathError, PrimPathErrorAttributeSetter>(primPathError);

    // Raised when a string does not parse as a valid USD prim path.
    nb::exception<PrimPathStringError> primPathStringError(module, "PrimPathStringError", exception);
    registerAttributeCarryingTranslator<PrimPathStringError, PrimPathStringErrorAttributeSetter>(primPathStringError);

    // Raised when a caller references an attribute name that does not exist or is not accepted.
    nb::exception<AttributeNameError> attributeNameError(module, "AttributeNameError", exception);
    registerAttributeCarryingTranslator<AttributeNameError, AttributeNameErrorAttributeSetter>(attributeNameError);

    // Raised when a value's type does not match the type expected for an attribute.
    nb::exception<ValueTypeError> valueTypeError(module, "ValueTypeError", exception);
    registerAttributeCarryingTranslator<ValueTypeError, ValueTypeErrorAttributeSetter>(valueTypeError);

    // Test-only helpers used to trigger each C++ exception from Python.
    module.def("_raise_prim_path_error", [](const std::string& primPath) { throw PrimPathError(primPath); });
    module.def("_raise_prim_path_string_error",
               [](const std::string& primPathString) { throw PrimPathStringError(primPathString); });
    module.def(
        "_raise_attribute_name_error",
        [](const std::string& attributeName, const std::optional<std::vector<std::string>>& validAttributeNames)
        { throw AttributeNameError(attributeName, validAttributeNames); },
        nb::arg("attribute_name"), nb::arg("valid_attribute_names") = nb::none());
    module.def("_raise_value_type_error",
               [](const std::string& attributeName, const std::string& expectedType, const std::string& actualType)
               { throw ValueTypeError(attributeName, expectedType, actualType); });
}
