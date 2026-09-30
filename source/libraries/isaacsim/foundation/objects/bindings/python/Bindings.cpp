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

#include "isaacsim/common/array/nanobind/ArrayCaster.hpp"
#include "isaacsim/foundation/objects/Camera.hpp"
#include "isaacsim/foundation/objects/Mesh.hpp"
#include "isaacsim/foundation/objects/Prim.hpp"
#include "isaacsim/foundation/objects/Stage.hpp"
#include "isaacsim/foundation/objects/Xform.hpp"
#include "isaacsim/foundation/objects/lights/CylinderLight.hpp"
#include "isaacsim/foundation/objects/lights/DiskLight.hpp"
#include "isaacsim/foundation/objects/lights/DistantLight.hpp"
#include "isaacsim/foundation/objects/lights/DomeLight.hpp"
#include "isaacsim/foundation/objects/lights/Light.hpp"
#include "isaacsim/foundation/objects/lights/RectLight.hpp"
#include "isaacsim/foundation/objects/lights/SphereLight.hpp"
#include "isaacsim/foundation/objects/physics_scenes/NewtonMjcScene.hpp"
#include "isaacsim/foundation/objects/physics_scenes/PhysicsScene.hpp"
#include "isaacsim/foundation/objects/physics_scenes/PhysxScene.hpp"
#include "isaacsim/foundation/objects/shapes/Capsule.hpp"
#include "isaacsim/foundation/objects/shapes/Cone.hpp"
#include "isaacsim/foundation/objects/shapes/Cube.hpp"
#include "isaacsim/foundation/objects/shapes/Cylinder.hpp"
#include "isaacsim/foundation/objects/shapes/Plane.hpp"
#include "isaacsim/foundation/objects/shapes/Shape.hpp"
#include "isaacsim/foundation/objects/shapes/Sphere.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <variant>

namespace nb = nanobind;
using namespace isaacsim::foundation::objects;

NB_MODULE(_bindings, m)
{
    m.doc() = "Batched USD scene-object wrappers.";

    // Prim.h
    nb::class_<Prim>(m, "Prim", R"doc(
Wrap one or more USD prims on the active stage.

Args:
    paths: A prim path or list of paths, optionally containing regular expressions.
    resolve_paths: Whether to resolve and validate paths during construction.
)doc")
        .def(
            "__init__",
            [](Prim* self, const std::variant<std::string, std::vector<std::string>>& paths, bool resolvePaths)
            { new (self) Prim(paths, resolvePaths); },
            nb::arg("paths"), nb::kw_only(), nb::arg("resolve_paths") = true)
        .def_prop_ro("paths", &Prim::paths, "The resolved paths wrapped by this object.")
        .def("get_stage", &Prim::getStage, nb::rv_policy::reference_internal, "Get the active stage wrapper.")
        .def_static(
            "resolve_paths",
            [](const std::variant<std::string, std::vector<std::string>>& paths, bool raiseOnMixedPaths)
            { return Prim({}, false).resolvePaths(paths, raiseOnMixedPaths); },
            nb::arg("paths"), nb::kw_only(), nb::arg("raise_on_mixed_paths") = true,
            "Resolve literal and regular-expression paths against the active stage.")
        .def("get_name", &Prim::getName, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the names of the selected prims.")
        .def("get_type_name", &Prim::getTypeName, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the USD type names of the selected prims.")
        .def("get_parent", &Prim::getParent, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the parent paths of the selected prims.")
        .def("get_children", &Prim::getChildren, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the child paths of the selected prims.")
        .def("get_variant_sets", &Prim::getVariantSets, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the available variants grouped by variant set for the selected prims.")
        .def("get_variant_selection", &Prim::getVariantSelection, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the current variant selections of the selected prims.")
        .def("set_variant_selection", &Prim::setVariantSelection, nb::arg("variants"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set variant selections on the selected prims.")
        .def("is_a", &Prim::isA, nb::arg("schema_type"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Check whether the selected prims conform to a typed schema.")
        .def("has_api", &Prim::hasApi, nb::arg("schema_type"), nb::kw_only(), nb::arg("instance_name") = nb::none(),
             nb::arg("indices") = nb::none(), "Check whether the selected prims have an applied API schema.")
        .def("apply_api", &Prim::applyApi, nb::arg("schema_type"), nb::kw_only(), nb::arg("instance_name") = nb::none(),
             nb::arg("indices") = nb::none(), "Apply an API schema to the selected prims.")
        .def("remove_api", &Prim::removeApi, nb::arg("schema_type"), nb::kw_only(), nb::arg("instance_name") = nb::none(),
             nb::arg("indices") = nb::none(), "Remove an applied API schema from the selected prims.")
        .def("get_applied_schemas", &Prim::getAppliedSchemas, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get the applied schema names of the selected prims.")
        .def("get_attribute_values", &Prim::getAttributeValues, nb::arg("attribute_name"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get an attribute value from each selected prim.")
        .def("set_attribute_values", &Prim::setAttributeValues, nb::arg("attribute_name"), nb::arg("values"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Set an attribute value on each selected prim.");

    // Stage.h
    nb::class_<Stage>(m, "Stage", R"doc(
Wrap a USD stage and provide lifecycle and scene-level operations.

Args:
    backend: The stage backend, either ``"openusd"`` or ``"ovstage"``.
    stage_id: The identifier of an existing stage, or ``None`` for an invalid stage.
)doc")
        .def(nb::init<std::string, std::optional<int64_t>>(), nb::arg("backend"), nb::arg("stage_id") = nb::none())
        .def("get_backend", &Stage::getBackend, "Get the stage backend name.")
        .def("get_stage_id", &Stage::getStageId, "Get the numeric stage identifier.")
        .def(
            "get_stage_ptr", [](const Stage& self) { return reinterpret_cast<uintptr_t>(self.getStagePtr()); },
            "Get the address of the underlying stage, or zero if the stage is invalid.")
        .def("is_valid", &Stage::isValid, "Check whether the stage is open and accessible.")
        .def(
            "open_stage",
            [](nb::object self, const std::string& usdPath, bool makeDefault) -> nb::object
            {
                nb::cast<Stage&>(self).openStage(usdPath, makeDefault);
                return self;
            },
            nb::arg("usd_path"), nb::kw_only(), nb::arg("make_default") = true,
            "Open a USD file and return this wrapper for method chaining.")
        .def(
            "create_stage",
            [](nb::object self, std::optional<std::string> templateName, bool makeDefault) -> nb::object
            {
                nb::cast<Stage&>(self).createStage(templateName, makeDefault);
                return self;
            },
            nb::kw_only(), nb::arg("template") = nb::none(), nb::arg("make_default") = true,
            "Create a stage, optionally from a template, and return this wrapper.")
        .def("save_stage", &Stage::saveStage, nb::arg("usd_path"), "Save the stage to a USD file.")
        .def("export_stage_to_string", &Stage::exportStageToString, "Serialize the stage root layer to a USDA string.")
        .def(
            "import_stage_from_string",
            [](nb::object self, const std::string& usdString, bool makeDefault) -> nb::object
            {
                nb::cast<Stage&>(self).importStageFromString(usdString, makeDefault);
                return self;
            },
            nb::arg("usd_string"), nb::kw_only(), nb::arg("make_default") = true,
            "Populate the stage from a USDA string and return this wrapper.")
        .def("close_stage", &Stage::closeStage, "Close the stage and release its resources.")
        .def("add_reference", &Stage::addReference, nb::arg("usd_path"), nb::arg("path"), nb::kw_only(),
             nb::arg("prim_type") = "Xform", nb::arg("variants") = nb::none(), "Add a USD reference at a prim path.")
        .def("define_prim", &Stage::definePrim, nb::arg("path"), nb::arg("type_name") = "Xform",
             "Define a prim and return its stage path.")
        .def("move_prim", &Stage::movePrim, nb::arg("target"), nb::arg("destination"),
             "Move a prim and return its success state and final path.")
        .def("remove_prim", &Stage::removePrim, nb::arg("path"), "Remove a prim and all of its descendants.")
        .def("get_up_axis", &Stage::getUpAxis, "Get the stage up-axis token.")
        .def("set_up_axis", &Stage::setUpAxis, nb::arg("up_axis"), "Set the stage up-axis token.")
        .def("get_units", &Stage::getUnits, "Get the meters-per-unit and kilograms-per-unit scales.")
        .def("set_units", &Stage::setUnits, nb::kw_only(), nb::arg("meters_per_unit") = nb::none(),
             nb::arg("kilograms_per_unit") = nb::none(), "Set one or both stage unit scales.")
        .def("get_time_code", &Stage::getTimeCode, "Get the start, end, and per-second stage time-code values.")
        .def("set_time_code", &Stage::setTimeCode, nb::kw_only(), nb::arg("start_time_code") = nb::none(),
             nb::arg("end_time_code") = nb::none(), nb::arg("time_codes_per_second") = nb::none(),
             "Set any provided stage time-code values.")
        .def("generate_string_representation", &Stage::generateStringRepresentation, nb::kw_only(),
             nb::arg("mode") = "tree", "Generate a tree or flat-list representation of the stage hierarchy.")
        .def(
            "__str__", [](const Stage& self) { return self.generateStringRepresentation("tree"); },
            "Return a tree representation of the stage hierarchy.");

    // Xform.h
    nb::class_<Xform, Prim>(m, "Xform", R"doc(
Wrap one or more transformable USD prims.

Args:
    paths: A prim path or list of paths, optionally containing regular expressions.
    resolve_paths: Whether to resolve and validate paths during construction.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](Xform* self, const std::variant<std::string, std::vector<std::string>>& paths, bool resolvePaths,
               bool resetXformOpProperties) { new (self) Xform(paths, resetXformOpProperties, resolvePaths); },
            nb::arg("paths"), nb::kw_only(), nb::arg("resolve_paths") = true, nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &Xform::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify transformable prims.")
        .def("set_visibilities", &Xform::setVisibilities, nb::arg("visibilities"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set visibility flags on the selected prims.")
        .def("get_visibilities", &Xform::getVisibilities, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get visibility flags for the selected prims.")
        .def("get_world_poses", &Xform::getWorldPoses, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("rotation_format") = "xyzw", "Get world positions and orientations for the selected prims.")
        .def("get_local_poses", &Xform::getLocalPoses, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("rotation_format") = "xyzw", "Get local translations and orientations for the selected prims.")
        .def("get_local_scales", &Xform::getLocalScales, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get local scales for the selected prims.")
        .def("set_local_scales", &Xform::setLocalScales, nb::arg("scales"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set local scales on the selected prims.")
        .def("set_local_poses", &Xform::setLocalPoses, nb::arg("translations") = nb::none(),
             nb::arg("orientations") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("rotation_format") = "xyzw", "Set local translations and/or orientations.")
        .def("set_world_poses", &Xform::setWorldPoses, nb::arg("positions") = nb::none(),
             nb::arg("orientations") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("rotation_format") = "xyzw", "Set world positions and/or orientations.")
        .def("reset_xform_op_properties", &Xform::resetXformOpProperties,
             "Normalize the wrapped prims' transform operation stacks.");

    // shapes/Shape.h
    nb::class_<shapes::Shape, Xform>(m, "Shape", "Base wrapper for batched geometric-shape prims.")
        .def_static("are_of_type", &shapes::Shape::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify geometric-shape prims.")
        .def("set_display_colors", &shapes::Shape::setDisplayColors, nb::arg("colors"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set display colors on the selected shapes.")
        .def("get_display_colors", &shapes::Shape::getDisplayColors, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get display colors for the selected shapes.")
        .def("update_extents", &shapes::Shape::updateExtents, "Recompute the authored extents of all wrapped shapes.");

    // shapes/Cube.h
    nb::class_<shapes::Cube, shapes::Shape>(m, "Cube", R"doc(
Wrap or create one or more USD Cube prims.

Args:
    paths: A prim path or list of paths.
    sizes: Optional edge lengths for the cubes.
    colors: Optional display colors for the cubes.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Cube* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& sizes, const std::optional<shapes::ColorType>& colors,
               bool resetXformOpProperties) { new (self) shapes::Cube(paths, sizes, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("sizes") = nb::none(), nb::arg("colors") = nb::none(),
            nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Cube::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Cube prims.")
        .def("set_sizes", &shapes::Cube::setSizes, nb::arg("sizes"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set edge lengths on the selected cubes.")
        .def("get_sizes", &shapes::Cube::getSizes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get edge lengths for the selected cubes.");

    // shapes/Sphere.h
    nb::class_<shapes::Sphere, shapes::Shape>(m, "Sphere", R"doc(
Wrap or create one or more USD Sphere prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the spheres.
    colors: Optional display colors for the spheres.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Sphere* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, const std::optional<shapes::ColorType>& colors,
               bool resetXformOpProperties) { new (self) shapes::Sphere(paths, radii, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("colors") = nb::none(),
            nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Sphere::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Sphere prims.")
        .def("set_radii", &shapes::Sphere::setRadii, nb::arg("radii"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set radii on the selected spheres.")
        .def("get_radii", &shapes::Sphere::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected spheres.");

    // shapes/Cylinder.h
    nb::class_<shapes::Cylinder, shapes::Shape>(m, "Cylinder", R"doc(
Wrap or create one or more USD Cylinder prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the cylinders.
    heights: Optional heights for the cylinders.
    axes: Optional cylinder-axis tokens.
    colors: Optional display colors for the cylinders.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Cylinder* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, const std::optional<array::Array>& heights,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
               const std::optional<shapes::ColorType>& colors, bool resetXformOpProperties)
            { new (self) shapes::Cylinder(paths, radii, heights, axes, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("heights") = nb::none(),
            nb::arg("axes") = nb::none(), nb::arg("colors") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Cylinder::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Cylinder prims.")
        .def("set_radii", &shapes::Cylinder::setRadii, nb::arg("radii"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set radii on the selected cylinders.")
        .def("get_radii", &shapes::Cylinder::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected cylinders.")
        .def("set_heights", &shapes::Cylinder::setHeights, nb::arg("heights"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set heights on the selected cylinders.")
        .def("get_heights", &shapes::Cylinder::getHeights, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get heights for the selected cylinders.")
        .def("set_axes", &shapes::Cylinder::setAxes, nb::arg("axes"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set axis tokens on the selected cylinders.")
        .def("get_axes", &shapes::Cylinder::getAxes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get axis tokens for the selected cylinders.");

    // shapes/Capsule.h
    nb::class_<shapes::Capsule, shapes::Shape>(m, "Capsule", R"doc(
Wrap or create one or more USD Capsule prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the capsules.
    heights: Optional heights for the capsules.
    axes: Optional capsule-axis tokens.
    colors: Optional display colors for the capsules.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Capsule* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, const std::optional<array::Array>& heights,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
               const std::optional<shapes::ColorType>& colors, bool resetXformOpProperties)
            { new (self) shapes::Capsule(paths, radii, heights, axes, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("heights") = nb::none(),
            nb::arg("axes") = nb::none(), nb::arg("colors") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Capsule::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Capsule prims.")
        .def("set_radii", &shapes::Capsule::setRadii, nb::arg("radii"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set radii on the selected capsules.")
        .def("get_radii", &shapes::Capsule::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected capsules.")
        .def("set_heights", &shapes::Capsule::setHeights, nb::arg("heights"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set heights on the selected capsules.")
        .def("get_heights", &shapes::Capsule::getHeights, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get heights for the selected capsules.")
        .def("set_axes", &shapes::Capsule::setAxes, nb::arg("axes"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set axis tokens on the selected capsules.")
        .def("get_axes", &shapes::Capsule::getAxes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get axis tokens for the selected capsules.");

    // shapes/Cone.h
    nb::class_<shapes::Cone, shapes::Shape>(m, "Cone", R"doc(
Wrap or create one or more USD Cone prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the cones.
    heights: Optional heights for the cones.
    axes: Optional cone-axis tokens.
    colors: Optional display colors for the cones.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Cone* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, const std::optional<array::Array>& heights,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
               const std::optional<shapes::ColorType>& colors, bool resetXformOpProperties)
            { new (self) shapes::Cone(paths, radii, heights, axes, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("heights") = nb::none(),
            nb::arg("axes") = nb::none(), nb::arg("colors") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Cone::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Cone prims.")
        .def("set_radii", &shapes::Cone::setRadii, nb::arg("radii"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set radii on the selected cones.")
        .def("get_radii", &shapes::Cone::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected cones.")
        .def("set_heights", &shapes::Cone::setHeights, nb::arg("heights"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set heights on the selected cones.")
        .def("get_heights", &shapes::Cone::getHeights, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get heights for the selected cones.")
        .def("set_axes", &shapes::Cone::setAxes, nb::arg("axes"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set axis tokens on the selected cones.")
        .def("get_axes", &shapes::Cone::getAxes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get axis tokens for the selected cones.");

    // shapes/Plane.h
    nb::class_<shapes::Plane, shapes::Shape>(m, "Plane", R"doc(
Wrap or create one or more USD Plane prims.

Args:
    paths: A prim path or list of paths.
    widths: Optional widths for the planes.
    lengths: Optional lengths for the planes.
    axes: Optional plane-normal axis tokens.
    colors: Optional display colors for the planes.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](shapes::Plane* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& widths, const std::optional<array::Array>& lengths,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& axes,
               const std::optional<shapes::ColorType>& colors, bool resetXformOpProperties)
            { new (self) shapes::Plane(paths, widths, lengths, axes, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("widths") = nb::none(), nb::arg("lengths") = nb::none(),
            nb::arg("axes") = nb::none(), nb::arg("colors") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &shapes::Plane::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Plane prims.")
        .def("set_widths", &shapes::Plane::setWidths, nb::arg("widths"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set widths on the selected planes.")
        .def("get_widths", &shapes::Plane::getWidths, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get widths for the selected planes.")
        .def("set_lengths", &shapes::Plane::setLengths, nb::arg("lengths"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set lengths on the selected planes.")
        .def("get_lengths", &shapes::Plane::getLengths, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get lengths for the selected planes.")
        .def("set_axes", &shapes::Plane::setAxes, nb::arg("axes"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set normal-axis tokens on the selected planes.")
        .def("get_axes", &shapes::Plane::getAxes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get normal-axis tokens for the selected planes.");

    // lights/Light.h
    nb::class_<lights::Light, Xform>(m, "Light", "Base wrapper for batched USD light prims.")
        .def_static("are_of_type", &lights::Light::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify light prims.")
        .def("set_intensities", &lights::Light::setIntensities, nb::arg("intensities"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set intensities on the selected lights.")
        .def("get_intensities", &lights::Light::getIntensities, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get intensities for the selected lights.")
        .def("set_exposures", &lights::Light::setExposures, nb::arg("exposures"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set exposure values on the selected lights.")
        .def("get_exposures", &lights::Light::getExposures, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get exposure values for the selected lights.")
        .def("set_multipliers", &lights::Light::setMultipliers, nb::arg("diffuse_multipliers") = nb::none(),
             nb::arg("specular_multipliers") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set diffuse and/or specular multipliers on the selected lights.")
        .def("get_multipliers", &lights::Light::getMultipliers, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get diffuse and specular multipliers for the selected lights.")
        .def("set_enabled_normalizations", &lights::Light::setEnabledNormalizations, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Enable or disable intensity normalization on the selected lights.")
        .def("get_enabled_normalizations", &lights::Light::getEnabledNormalizations, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get intensity-normalization states for the selected lights.")
        .def("set_enabled_color_temperatures", &lights::Light::setEnabledColorTemperatures, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Enable or disable color-temperature control on the selected lights.")
        .def("get_enabled_color_temperatures", &lights::Light::getEnabledColorTemperatures, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get color-temperature enable states for the selected lights.")
        .def("set_color_temperatures", &lights::Light::setColorTemperatures, nb::arg("color_temperatures"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Set color temperatures on the selected lights.")
        .def("get_color_temperatures", &lights::Light::getColorTemperatures, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get color temperatures for the selected lights.")
        .def("set_colors", &lights::Light::setColors, nb::arg("colors"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set emitted colors on the selected lights.")
        .def("get_colors", &lights::Light::getColors, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get emitted colors for the selected lights.");

    // lights/Sphere.h
    nb::class_<lights::SphereLight, lights::Light>(m, "SphereLight", R"doc(
Wrap or create one or more USD SphereLight prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the lights.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::SphereLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, bool resetXformOpProperties)
            { new (self) lights::SphereLight(paths, radii, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::SphereLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify SphereLight prims.")
        .def("set_radii", &lights::SphereLight::setRadii, nb::arg("radii"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set radii on the selected sphere lights.")
        .def("get_radii", &lights::SphereLight::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected sphere lights.")
        .def("set_enabled_treat_as_points", &lights::SphereLight::setEnabledTreatAsPoints, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Enable or disable point-light treatment on the selected sphere lights.")
        .def("get_enabled_treat_as_points", &lights::SphereLight::getEnabledTreatAsPoints, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get point-light treatment states for the selected sphere lights.");

    // lights/Distant.h
    nb::class_<lights::DistantLight, lights::Light>(m, "DistantLight", R"doc(
Wrap or create one or more USD DistantLight prims.

Args:
    paths: A prim path or list of paths.
    angles: Optional angular sizes for the lights.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::DistantLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& angles, bool resetXformOpProperties)
            { new (self) lights::DistantLight(paths, angles, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("angles") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::DistantLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify DistantLight prims.")
        .def("set_angles", &lights::DistantLight::setAngles, nb::arg("angles"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set angular sizes on the selected distant lights.")
        .def("get_angles", &lights::DistantLight::getAngles, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get angular sizes for the selected distant lights.");

    // lights/Cylinder.h
    nb::class_<lights::CylinderLight, lights::Light>(m, "CylinderLight", R"doc(
Wrap or create one or more USD CylinderLight prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the lights.
    lengths: Optional lengths for the lights.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::CylinderLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, const std::optional<array::Array>& lengths,
               bool resetXformOpProperties)
            { new (self) lights::CylinderLight(paths, radii, lengths, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("lengths") = nb::none(),
            nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::CylinderLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify CylinderLight prims.")
        .def("set_radii", &lights::CylinderLight::setRadii, nb::arg("radii"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set radii on the selected cylinder lights.")
        .def("get_radii", &lights::CylinderLight::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected cylinder lights.")
        .def("set_lengths", &lights::CylinderLight::setLengths, nb::arg("lengths"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set lengths on the selected cylinder lights.")
        .def("get_lengths", &lights::CylinderLight::getLengths, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get lengths for the selected cylinder lights.")
        .def("set_enabled_treat_as_lines", &lights::CylinderLight::setEnabledTreatAsLines, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Enable or disable line-light treatment on the selected cylinder lights.")
        .def("get_enabled_treat_as_lines", &lights::CylinderLight::getEnabledTreatAsLines, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get line-light treatment states for the selected cylinder lights.");

    // lights/Rect.h
    nb::class_<lights::RectLight, lights::Light>(m, "RectLight", R"doc(
Wrap or create one or more USD RectLight prims.

Args:
    paths: A prim path or list of paths.
    widths: Optional widths for the lights.
    heights: Optional heights for the lights.
    texture_files: Optional emission-texture asset paths.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::RectLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& widths, const std::optional<array::Array>& heights,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFiles,
               bool resetXformOpProperties)
            { new (self) lights::RectLight(paths, widths, heights, textureFiles, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("widths") = nb::none(), nb::arg("heights") = nb::none(),
            nb::arg("texture_files") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::RectLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify RectLight prims.")
        .def("set_widths", &lights::RectLight::setWidths, nb::arg("widths"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set widths on the selected rectangular lights.")
        .def("get_widths", &lights::RectLight::getWidths, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get widths for the selected rectangular lights.")
        .def("set_heights", &lights::RectLight::setHeights, nb::arg("heights"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set heights on the selected rectangular lights.")
        .def("get_heights", &lights::RectLight::getHeights, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get heights for the selected rectangular lights.")
        .def("set_texture_files", &lights::RectLight::setTextureFiles, nb::arg("texture_files"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set emission-texture asset paths on the selected lights.")
        .def("get_texture_files", &lights::RectLight::getTextureFiles, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get emission-texture asset paths for the selected lights.");

    // lights/Dome.h
    nb::class_<lights::DomeLight, lights::Light>(m, "DomeLight", R"doc(
Wrap or create one or more USD DomeLight prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional guide radii for the lights.
    texture_files: Optional environment-texture asset paths.
    texture_formats: Optional texture-coordinate format tokens.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::DomeLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFiles,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& textureFormats,
               bool resetXformOpProperties)
            { new (self) lights::DomeLight(paths, radii, textureFiles, textureFormats, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("texture_files") = nb::none(),
            nb::arg("texture_formats") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::DomeLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify DomeLight prims.")
        .def("set_guide_radii", &lights::DomeLight::setGuideRadii, nb::arg("radii"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set guide radii on the selected dome lights.")
        .def("get_guide_radii", &lights::DomeLight::getGuideRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get guide radii for the selected dome lights.")
        .def("set_texture_files", &lights::DomeLight::setTextureFiles, nb::arg("texture_files"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set environment-texture asset paths on the selected dome lights.")
        .def("get_texture_files", &lights::DomeLight::getTextureFiles, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get environment-texture asset paths for the selected dome lights.")
        .def("set_texture_formats", &lights::DomeLight::setTextureFormats, nb::arg("texture_formats"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set texture-coordinate formats on the selected dome lights.")
        .def("get_texture_formats", &lights::DomeLight::getTextureFormats, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get texture-coordinate formats for the selected dome lights.");

    // lights/Disk.h
    nb::class_<lights::DiskLight, lights::Light>(m, "DiskLight", R"doc(
Wrap or create one or more USD DiskLight prims.

Args:
    paths: A prim path or list of paths.
    radii: Optional radii for the lights.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](lights::DiskLight* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& radii, bool resetXformOpProperties)
            { new (self) lights::DiskLight(paths, radii, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("radii") = nb::none(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &lights::DiskLight::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify DiskLight prims.")
        .def("set_radii", &lights::DiskLight::setRadii, nb::arg("radii"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set radii on the selected disk lights.")
        .def("get_radii", &lights::DiskLight::getRadii, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get radii for the selected disk lights.");

    // Mesh.h
    nb::class_<Mesh, Xform>(m, "Mesh", R"doc(
Wrap or create one or more USD Mesh prims.

Args:
    paths: A prim path or list of paths.
    primitives: Optional mesh primitive names used to initialize new prims.
    colors: Optional display colors for the meshes.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](Mesh* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& primitives,
               const std::optional<ColorType>& colors, bool resetXformOpProperties)
            { new (self) Mesh(paths, primitives, colors, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("primitives") = nb::none(), nb::arg("colors") = nb::none(),
            nb::arg("reset_xform_op_properties") = true)
        .def_static(
            "are_of_type", &Mesh::areOfType, nb::arg("paths"), "Check whether the resolved paths identify Mesh prims.")
        .def_prop_ro("num_faces", &Mesh::numFaces, "The number of faces in each wrapped mesh.")
        .def("set_points", &Mesh::setPoints, nb::arg("points"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set vertex positions on the selected meshes.")
        .def("get_points", &Mesh::getPoints, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get vertex positions for the selected meshes.")
        .def("set_normals", &Mesh::setNormals, nb::arg("normals"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set normal vectors on the selected meshes.")
        .def("get_normals", &Mesh::getNormals, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get normal vectors for the selected meshes.")
        .def("set_face_specs", &Mesh::setFaceSpecifications, nb::arg("vertex_indices") = nb::none(),
             nb::arg("vertex_counts") = nb::none(), nb::arg("varying_linear_interpolations") = nb::none(),
             nb::arg("hole_indices") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set face topology, interpolation, and hole data on the selected meshes.")
        .def("get_face_specs", &Mesh::getFaceSpecifications, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get face topology, interpolation, and hole data for the selected meshes.")
        .def("set_crease_specs", &Mesh::setCreaseSpecifications, nb::arg("crease_indices"), nb::arg("crease_lengths"),
             nb::arg("crease_sharpnesses"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set crease indices, lengths, and sharpness values on the selected meshes.")
        .def("get_crease_specs", &Mesh::getCreaseSpecifications, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get crease indices, lengths, and sharpness values for the selected meshes.")
        .def("set_corner_specs", &Mesh::setCornerSpecifications, nb::arg("corner_indices"),
             nb::arg("corner_sharpnesses"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set corner indices and sharpness values on the selected meshes.")
        .def("get_corner_specs", &Mesh::getCornerSpecifications, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get corner indices and sharpness values for the selected meshes.")
        .def("set_subdivision_specs", &Mesh::setSubdivisionSpecifications, nb::arg("subdivision_schemes") = nb::none(),
             nb::arg("interpolate_boundaries") = nb::none(), nb::arg("triangle_subdivision_rules") = nb::none(),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set subdivision schemes, boundary modes, and triangle rules on the selected meshes.")
        .def("get_subdivision_specs", &Mesh::getSubdivisionSpecifications, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get subdivision schemes, boundary modes, and triangle rules for the selected meshes.")
        .def("set_display_colors", &Mesh::setDisplayColors, nb::arg("colors"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set display colors on the selected meshes.")
        .def("get_display_colors", &Mesh::getDisplayColors, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get display colors for the selected meshes.")
        .def("update_extents", &Mesh::updateExtents, "Recompute the authored extents of all wrapped meshes.");

    // Camera.h
    nb::class_<Camera, Xform>(m, "Camera", R"doc(
Wrap or create one or more USD Camera prims.

Args:
    paths: A prim path or list of paths.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.
)doc")
        .def(
            "__init__",
            [](Camera* self, const std::variant<std::string, std::vector<std::string>>& paths,
               bool resetXformOpProperties) { new (self) Camera(paths, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("reset_xform_op_properties") = true)
        .def_static("are_of_type", &Camera::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify Camera prims.")
        .def("set_focal_lengths", &Camera::setFocalLengths, nb::arg("focal_lengths"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set focal lengths on the selected cameras.")
        .def("get_focal_lengths", &Camera::getFocalLengths, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get focal lengths for the selected cameras.")
        .def("set_focus_distances", &Camera::setFocusDistances, nb::arg("focus_distances"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set focus distances on the selected cameras.")
        .def("get_focus_distances", &Camera::getFocusDistances, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get focus distances for the selected cameras.")
        .def("set_stereo_roles", &Camera::setStereoRoles, nb::arg("roles"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set stereo-role tokens on the selected cameras.")
        .def("get_stereo_roles", &Camera::getStereoRoles, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get stereo-role tokens for the selected cameras.")
        .def("set_fstops", &Camera::setFStops, nb::arg("fstops"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set f-stop values on the selected cameras.")
        .def("get_fstops", &Camera::getFStops, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get f-stop values for the selected cameras.")
        .def("set_apertures", &Camera::setApertures, nb::arg("horizontal_apertures") = nb::none(),
             nb::arg("vertical_apertures") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set horizontal and/or vertical apertures on the selected cameras.")
        .def("get_apertures", &Camera::getApertures, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get horizontal and vertical apertures for the selected cameras.")
        .def("set_aperture_offsets", &Camera::setApertureOffsets, nb::arg("horizontal_offsets") = nb::none(),
             nb::arg("vertical_offsets") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set horizontal and/or vertical aperture offsets on the selected cameras.")
        .def("get_aperture_offsets", &Camera::getApertureOffsets, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get horizontal and vertical aperture offsets for the selected cameras.")
        .def("set_projections", &Camera::setProjections, nb::arg("projections"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set projection-type tokens on the selected cameras.")
        .def("get_projections", &Camera::getProjections, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get projection-type tokens for the selected cameras.")
        .def("set_clipping_ranges", &Camera::setClippingRanges, nb::arg("near_distances") = nb::none(),
             nb::arg("far_distances") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set near and/or far clipping distances on the selected cameras.")
        .def("get_clipping_ranges", &Camera::getClippingRanges, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get near and far clipping distances for the selected cameras.")
        .def("set_shutter_times", &Camera::setShutterTimes, nb::arg("open_times") = nb::none(),
             nb::arg("close_times") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set shutter-open and/or shutter-close times on the selected cameras.")
        .def("get_shutter_times", &Camera::getShutterTimes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get shutter-open and shutter-close times for the selected cameras.")
        .def("enforce_square_pixels", &Camera::enforceSquarePixels, nb::arg("resolutions"), nb::kw_only(),
             nb::arg("modes") = "horizontal", nb::arg("indices") = nb::none(),
             "Adjust apertures so the requested output resolutions use square pixels.");

    // physics_scenes/PhysicsScene.h
    nb::class_<physics_scenes::PhysicsScene, Prim>(m, "PhysicsScene", R"doc(
Wrap or create one or more solver-agnostic USD PhysicsScene prims.

Args:
    paths: A prim path or list of paths, optionally containing regular expressions.
)doc")
        .def(
            "__init__",
            [](physics_scenes::PhysicsScene* self, const std::variant<std::string, std::vector<std::string>>& paths)
            { new (self) physics_scenes::PhysicsScene(paths); },
            nb::arg("paths"))
        .def_static("are_of_type", &physics_scenes::PhysicsScene::areOfType, nb::arg("paths"),
                    "Check whether the resolved paths identify PhysicsScene prims.")
        .def_static("get_physics_scene_paths", &physics_scenes::PhysicsScene::getPhysicsScenePaths,
                    "Get every PhysicsScene prim path on the active stage.")
        .def("set_gravities", &physics_scenes::PhysicsScene::setGravities, nb::arg("gravities"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set gravity vectors on the selected physics scenes.")
        .def("get_gravities", &physics_scenes::PhysicsScene::getGravities, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get gravity vectors for the selected physics scenes.")
        .def("set_dts", &physics_scenes::PhysicsScene::setDeltaTimes, nb::arg("dts"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set simulation delta times on the selected physics scenes.")
        .def("get_dts", &physics_scenes::PhysicsScene::getDeltaTimes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get simulation delta times for the selected physics scenes.")
        .def("set_time_steps_per_seconds", &physics_scenes::PhysicsScene::setTimeStepsPerSecond,
             nb::arg("time_steps_per_seconds"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set simulation step frequencies on the selected physics scenes.")
        .def("get_time_steps_per_seconds", &physics_scenes::PhysicsScene::getTimeStepsPerSecond, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get simulation step frequencies for the selected physics scenes.")
        .def("set_enabled_gravities", &physics_scenes::PhysicsScene::setEnabledGravities, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Enable or disable gravity on the selected physics scenes.")
        .def("get_enabled_gravities", &physics_scenes::PhysicsScene::getEnabledGravities, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get gravity-enable states for the selected physics scenes.")
        .def("set_max_solver_iterations", &physics_scenes::PhysicsScene::setMaxSolverIterations, nb::arg("iterations"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set maximum solver iterations on the selected physics scenes.")
        .def("get_max_solver_iterations", &physics_scenes::PhysicsScene::getMaxSolverIterations, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get maximum solver iterations for the selected physics scenes.");

    // physics_scenes/PhysxScene.h
    nb::class_<physics_scenes::PhysxGpuConfiguration>(m, "PhysxGpuCfg", R"doc(
Store optional batched GPU buffer sizes for PhysX scenes.

Each provided field is written by ``PhysxScene.set_gpu_configuration``; omitted fields remain unchanged.

Args:
    gpu_collision_stack_size: Optional collision-stack sizes in bytes.
    gpu_found_lost_aggregate_pairs_capacity: Optional found/lost aggregate-pair capacities.
    gpu_found_lost_pairs_capacity: Optional found/lost pair capacities.
    gpu_heap_capacity: Optional heap sizes in bytes.
    gpu_max_deformable_surface_contacts: Optional deformable-surface contact limits.
    gpu_max_deformable_volume_contacts: Optional deformable-volume contact limits.
    gpu_max_num_partitions: Optional solver-partition limits.
    gpu_max_particle_contacts: Optional particle-contact limits.
    gpu_max_rigid_contact_count: Optional rigid-body contact limits.
    gpu_max_rigid_patch_count: Optional rigid-body contact-patch limits.
    gpu_temp_buffer_capacity: Optional temporary-buffer sizes in bytes.
    gpu_total_aggregate_pairs_capacity: Optional total aggregate-pair capacities.
)doc")
        .def(
            "__init__",
            [](physics_scenes::PhysxGpuConfiguration* self, const std::optional<array::Array>& gpuCollisionStackSize,
               const std::optional<array::Array>& gpuFoundLostAggregatePairsCapacity,
               const std::optional<array::Array>& gpuFoundLostPairsCapacity,
               const std::optional<array::Array>& gpuHeapCapacity,
               const std::optional<array::Array>& gpuMaximumDeformableSurfaceContacts,
               const std::optional<array::Array>& gpuMaximumDeformableVolumeContacts,
               const std::optional<array::Array>& gpuMaximumPartitionCount,
               const std::optional<array::Array>& gpuMaximumParticleContacts,
               const std::optional<array::Array>& gpuMaximumRigidContactCount,
               const std::optional<array::Array>& gpuMaximumRigidPatchCount,
               const std::optional<array::Array>& gpuTemporaryBufferCapacity,
               const std::optional<array::Array>& gpuTotalAggregatePairsCapacity)
            {
                new (self) physics_scenes::PhysxGpuConfiguration{
                    gpuCollisionStackSize,     gpuFoundLostAggregatePairsCapacity,  gpuFoundLostPairsCapacity,
                    gpuHeapCapacity,           gpuMaximumDeformableSurfaceContacts, gpuMaximumDeformableVolumeContacts,
                    gpuMaximumPartitionCount,  gpuMaximumParticleContacts,          gpuMaximumRigidContactCount,
                    gpuMaximumRigidPatchCount, gpuTemporaryBufferCapacity,          gpuTotalAggregatePairsCapacity
                };
            },
            nb::kw_only(), nb::arg("gpu_collision_stack_size") = nb::none(),
            nb::arg("gpu_found_lost_aggregate_pairs_capacity") = nb::none(),
            nb::arg("gpu_found_lost_pairs_capacity") = nb::none(), nb::arg("gpu_heap_capacity") = nb::none(),
            nb::arg("gpu_max_deformable_surface_contacts") = nb::none(),
            nb::arg("gpu_max_deformable_volume_contacts") = nb::none(), nb::arg("gpu_max_num_partitions") = nb::none(),
            nb::arg("gpu_max_particle_contacts") = nb::none(), nb::arg("gpu_max_rigid_contact_count") = nb::none(),
            nb::arg("gpu_max_rigid_patch_count") = nb::none(), nb::arg("gpu_temp_buffer_capacity") = nb::none(),
            nb::arg("gpu_total_aggregate_pairs_capacity") = nb::none())
        .def_rw("gpu_collision_stack_size", &physics_scenes::PhysxGpuConfiguration::gpuCollisionStackSize,
                "The GPU collision-stack size in bytes.")
        .def_rw("gpu_found_lost_aggregate_pairs_capacity",
                &physics_scenes::PhysxGpuConfiguration::gpuFoundLostAggregatePairsCapacity,
                "The GPU found/lost aggregate-pair capacity.")
        .def_rw("gpu_found_lost_pairs_capacity", &physics_scenes::PhysxGpuConfiguration::gpuFoundLostPairsCapacity,
                "The GPU found/lost pair capacity.")
        .def_rw(
            "gpu_heap_capacity", &physics_scenes::PhysxGpuConfiguration::gpuHeapCapacity, "The GPU heap size in bytes.")
        .def_rw("gpu_max_deformable_surface_contacts",
                &physics_scenes::PhysxGpuConfiguration::gpuMaximumDeformableSurfaceContacts,
                "The maximum number of GPU deformable-surface contacts.")
        .def_rw("gpu_max_deformable_volume_contacts",
                &physics_scenes::PhysxGpuConfiguration::gpuMaximumDeformableVolumeContacts,
                "The maximum number of GPU deformable-volume contacts.")
        .def_rw("gpu_max_num_partitions", &physics_scenes::PhysxGpuConfiguration::gpuMaximumPartitionCount,
                "The maximum number of GPU solver partitions.")
        .def_rw("gpu_max_particle_contacts", &physics_scenes::PhysxGpuConfiguration::gpuMaximumParticleContacts,
                "The maximum number of GPU particle contacts.")
        .def_rw("gpu_max_rigid_contact_count", &physics_scenes::PhysxGpuConfiguration::gpuMaximumRigidContactCount,
                "The maximum number of GPU rigid-body contacts.")
        .def_rw("gpu_max_rigid_patch_count", &physics_scenes::PhysxGpuConfiguration::gpuMaximumRigidPatchCount,
                "The maximum number of GPU rigid-body contact patches.")
        .def_rw("gpu_temp_buffer_capacity", &physics_scenes::PhysxGpuConfiguration::gpuTemporaryBufferCapacity,
                "The GPU temporary-buffer size in bytes.")
        .def_rw("gpu_total_aggregate_pairs_capacity",
                &physics_scenes::PhysxGpuConfiguration::gpuTotalAggregatePairsCapacity,
                "The total GPU aggregate-pair capacity.");

    nb::class_<physics_scenes::PhysxScene, physics_scenes::PhysicsScene>(m, "PhysxScene", R"doc(
Wrap or create one or more USD PhysicsScene prims configured for PhysX.

Args:
    paths: A prim path or list of paths, optionally containing regular expressions.
)doc")
        .def(
            "__init__",
            [](physics_scenes::PhysxScene* self, const std::variant<std::string, std::vector<std::string>>& paths)
            { new (self) physics_scenes::PhysxScene(paths); },
            nb::arg("paths"))
        .def("set_dts", &physics_scenes::PhysxScene::setDeltaTimes, nb::arg("dts"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set PhysX delta times on the selected scenes.")
        .def("get_dts", &physics_scenes::PhysxScene::getDeltaTimes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get PhysX delta times for the selected scenes.")
        .def("set_time_steps_per_seconds", &physics_scenes::PhysxScene::setTimeStepsPerSecond,
             nb::arg("time_steps_per_seconds"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set PhysX step frequencies on the selected scenes.")
        .def("get_time_steps_per_seconds", &physics_scenes::PhysxScene::getTimeStepsPerSecond, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get PhysX step frequencies for the selected scenes.")
        .def("set_solver_types", &physics_scenes::PhysxScene::setSolverTypes, nb::arg("solver_types"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set PhysX solver-type tokens on the selected scenes.")
        .def("get_solver_types", &physics_scenes::PhysxScene::getSolverTypes, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get PhysX solver-type tokens for the selected scenes.")
        .def("set_broadphase_types", &physics_scenes::PhysxScene::setBroadphaseTypes, nb::arg("broadphase_types"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Set broadphase-type tokens on the selected scenes.")
        .def("get_broadphase_types", &physics_scenes::PhysxScene::getBroadphaseTypes, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get broadphase-type tokens for the selected scenes.")
        .def("set_enabled_gpu_dynamics", &physics_scenes::PhysxScene::setEnabledGpuDynamics, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Enable or disable GPU dynamics on the selected scenes.")
        .def("get_enabled_gpu_dynamics", &physics_scenes::PhysxScene::getEnabledGpuDynamics, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get GPU-dynamics states for the selected scenes.")
        .def("set_enabled_ccds", &physics_scenes::PhysxScene::setEnabledCcds, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Enable or disable continuous collision detection on the selected scenes.")
        .def("get_enabled_ccds", &physics_scenes::PhysxScene::getEnabledCcds, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get continuous-collision-detection states for the selected scenes.")
        .def("set_enabled_stabilizations", &physics_scenes::PhysxScene::setEnabledStabilizations, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             "Enable or disable solver stabilization on the selected scenes.")
        .def("get_enabled_stabilizations", &physics_scenes::PhysxScene::getEnabledStabilizations, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get solver-stabilization states for the selected scenes.")
        .def("set_gpu_configuration", &physics_scenes::PhysxScene::setGpuConfiguration, nb::arg("cfg"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set provided GPU buffer-size fields on the selected scenes.")
        .def("get_gpu_configuration", &physics_scenes::PhysxScene::getGpuConfiguration, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get all GPU buffer-size fields for the selected scenes.");

    // physics_scenes/NewtonMjcScene.h
    nb::class_<physics_scenes::NewtonMjcScene, physics_scenes::PhysicsScene>(m, "NewtonMjcScene", R"doc(
Wrap or create one or more USD PhysicsScene prims configured for Newton.

Args:
    paths: A prim path or list of paths, optionally containing regular expressions.
)doc")
        .def(
            "__init__",
            [](physics_scenes::NewtonMjcScene* self, const std::variant<std::string, std::vector<std::string>>& paths)
            { new (self) physics_scenes::NewtonMjcScene(paths); },
            nb::arg("paths"))
        .def("set_dts", &physics_scenes::NewtonMjcScene::setDeltaTimes, nb::arg("dts"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton delta times on the selected scenes.")
        .def("get_dts", &physics_scenes::NewtonMjcScene::getDeltaTimes, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get Newton delta times for the selected scenes.")
        .def("set_time_steps_per_seconds", &physics_scenes::NewtonMjcScene::setTimeStepsPerSecond,
             nb::arg("time_steps_per_seconds"), nb::kw_only(), nb::arg("indices") = nb::none(),
             "Set Newton step frequencies on the selected scenes.")
        .def("get_time_steps_per_seconds", &physics_scenes::NewtonMjcScene::getTimeStepsPerSecond, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton step frequencies for the selected scenes.")
        .def("set_integrators", &physics_scenes::NewtonMjcScene::setIntegrators, nb::arg("integrators"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton integrator tokens on the selected scenes.")
        .def("get_integrators", &physics_scenes::NewtonMjcScene::getIntegrators, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton integrator tokens for the selected scenes.")
        .def("set_solvers", &physics_scenes::NewtonMjcScene::setSolvers, nb::arg("solvers"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton solver tokens on the selected scenes.")
        .def("get_solvers", &physics_scenes::NewtonMjcScene::getSolvers, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get Newton solver tokens for the selected scenes.")
        .def("set_cones", &physics_scenes::NewtonMjcScene::setCones, nb::arg("cones"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton friction-cone tokens on the selected scenes.")
        .def("get_cones", &physics_scenes::NewtonMjcScene::getCones, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get Newton friction-cone tokens for the selected scenes.")
        .def("set_jacobians", &physics_scenes::NewtonMjcScene::setJacobians, nb::arg("jacobians"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton Jacobian-type tokens on the selected scenes.")
        .def("get_jacobians", &physics_scenes::NewtonMjcScene::getJacobians, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton Jacobian-type tokens for the selected scenes.")
        .def("set_iterations", &physics_scenes::NewtonMjcScene::setIterations, nb::arg("iterations"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton solver iteration counts on the selected scenes.")
        .def("get_iterations", &physics_scenes::NewtonMjcScene::getIterations, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton solver iteration counts for the selected scenes.")
        .def("set_tolerances", &physics_scenes::NewtonMjcScene::setTolerances, nb::arg("tolerances"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set Newton solver tolerances on the selected scenes.")
        .def("get_tolerances", &physics_scenes::NewtonMjcScene::getTolerances, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton solver tolerances for the selected scenes.")
        .def("set_impedance_ratios", &physics_scenes::NewtonMjcScene::setImpedanceRatios, nb::arg("impedance_ratios"),
             nb::kw_only(), nb::arg("indices") = nb::none(), "Set Newton impedance ratios on the selected scenes.")
        .def("get_impedance_ratios", &physics_scenes::NewtonMjcScene::getImpedanceRatios, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get Newton impedance ratios for the selected scenes.")
        .def("set_winds", &physics_scenes::NewtonMjcScene::setWinds, nb::arg("winds"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set wind velocity vectors on the selected scenes.")
        .def("get_winds", &physics_scenes::NewtonMjcScene::getWinds, nb::kw_only(), nb::arg("indices") = nb::none(),
             "Get wind velocity vectors for the selected scenes.")
        .def("set_densities", &physics_scenes::NewtonMjcScene::setDensities, nb::arg("densities"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set medium densities on the selected scenes.")
        .def("get_densities", &physics_scenes::NewtonMjcScene::getDensities, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get medium densities for the selected scenes.")
        .def("set_viscosities", &physics_scenes::NewtonMjcScene::setViscosities, nb::arg("viscosities"), nb::kw_only(),
             nb::arg("indices") = nb::none(), "Set medium viscosities on the selected scenes.")
        .def("get_viscosities", &physics_scenes::NewtonMjcScene::getViscosities, nb::kw_only(),
             nb::arg("indices") = nb::none(), "Get medium viscosities for the selected scenes.");
}
