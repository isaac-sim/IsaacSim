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
#include "isaacsim/foundation/objects/Xform.hpp"
#include "isaacsim/foundation/prims/physics/Articulation.hpp"
#include "isaacsim/foundation/prims/physics/ColliderBody.hpp"
#include "isaacsim/foundation/prims/physics/GroundPlane.hpp"
#include "isaacsim/foundation/prims/physics/RigidBody.hpp"

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
using namespace isaacsim::foundation::prims;

NB_MODULE(_bindings, m)
{
    m.doc() = R"doc(Provide batched wrappers for physics prims on the active USD stage.

Calls execute synchronously and retain the Python GIL. Selection arrays address wrapped prims along the first
dimension and, where applicable, articulation degrees of freedom or links along the second dimension.
)doc";

    // physics/Articulation.hpp
    nb::class_<physics::Articulation, Xform>(m, "Articulation",
                                             R"doc(Wrap one or more USD articulations.

Resolve each path against the active stage, locate its articulation root, and cache its degree-of-freedom, joint,
and link metadata. Omitted selection arrays address every wrapped articulation or sub-entity.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.

Raises:
    RuntimeError: If no active or default stage is available, a path has no articulation root, or the articulations
        are not homogeneous.
)doc")
        .def(
            "__init__",
            [](physics::Articulation* self, const std::variant<std::string, std::vector<std::string>>& paths,
               bool resetXformOpProperties) { new (self) physics::Articulation(paths, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("reset_xform_op_properties") = true,
            "Initialize the articulation wrapper and cache its metadata.")
        .def_static("are_of_type", &physics::Articulation::areOfType, nb::arg("paths"),
                    R"doc(Check whether paths resolve to transformable articulation roots or their wrappers.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.

Returns:
    Boolean flags shaped ``(N, 1)``, allocated on the CPU.

Raises:
    RuntimeError: If a path does not resolve to an existing prim.
)doc")
        .def_prop_ro("root_paths", &physics::Articulation::rootPaths,
                     "Get the articulation-root path for each wrapped articulation.")
        .def_prop_ro("num_dofs", &physics::Articulation::numDofs,
                     "Get the degree-of-freedom count shared by the wrapped articulations.")
        .def_prop_ro("dof_names", &physics::Articulation::dofNames, "Get the ordered degree-of-freedom names.")
        .def_prop_ro("dof_paths", &physics::Articulation::dofPaths,
                     "Get the degree-of-freedom paths for each wrapped articulation.")
        .def_prop_ro("dof_types", &physics::Articulation::dofTypes, "Get the ordered degree-of-freedom type names.")
        .def_prop_ro(
            "num_joints", &physics::Articulation::numJoints, "Get the joint count shared by the wrapped articulations.")
        .def_prop_ro("joint_names", &physics::Articulation::jointNames, "Get the ordered joint names.")
        .def_prop_ro(
            "joint_paths", &physics::Articulation::jointPaths, "Get the joint paths for each wrapped articulation.")
        .def_prop_ro("joint_types", &physics::Articulation::jointTypes, "Get the ordered joint type names.")
        .def_prop_ro(
            "num_links", &physics::Articulation::numLinks, "Get the link count shared by the wrapped articulations.")
        .def_prop_ro("link_names", &physics::Articulation::linkNames, "Get the ordered link names.")
        .def_prop_ro("link_paths", &physics::Articulation::linkPaths, "Get the link paths for each wrapped articulation.")
        .def("get_dof_indices", &physics::Articulation::getDofIndices, nb::arg("names"),
             R"doc(Get degree-of-freedom indices by name.

Args:
    names: Degree-of-freedom name or names to look up.

Returns:
    Indices in input order.

Raises:
    ValueError: If a name does not identify a degree of freedom.
)doc")
        .def("get_joint_indices", &physics::Articulation::getJointIndices, nb::arg("names"),
             R"doc(Get joint indices by name.

Args:
    names: Joint name or names to look up.

Returns:
    Indices in input order.

Raises:
    ValueError: If a name does not identify a joint.
)doc")
        .def("get_link_indices", &physics::Articulation::getLinkIndices, nb::arg("names"),
             R"doc(Get link indices by name.

Args:
    names: Link name or names to look up.

Returns:
    Indices in input order.

Raises:
    ValueError: If a name does not identify a link.
)doc")
        .def("get_dof_limits", &physics::Articulation::getDofLimits, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(),
             R"doc(Get lower and upper position limits for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Lower and upper limits shaped ``(N, D)``. Angular values are in radians and prismatic values are in stage units.
)doc")
        .def("set_dof_limits", &physics::Articulation::setDofLimits, nb::arg("lower") = nb::none(),
             nb::arg("upper") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(),
             R"doc(Set lower and/or upper position limits for selected degrees of freedom.

Provide at least one limit array. Angular values use radians and prismatic values use stage length units.

Args:
    lower: Lower limits shaped ``(N, D)``, or ``None`` to preserve them.
    upper: Upper limits shaped ``(N, D)``, or ``None`` to preserve them.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.

Raises:
    ValueError: If both ``lower`` and ``upper`` are ``None``.
)doc")
        .def("get_dof_friction_properties", &physics::Articulation::getDofFrictionProperties, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get friction properties for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Static friction efforts, dynamic friction efforts, and viscous friction coefficients, each shaped ``(N, D)``.
)doc")
        .def("set_dof_friction_properties", &physics::Articulation::setDofFrictionProperties,
             nb::arg("static_frictions") = nb::none(), nb::arg("dynamic_frictions") = nb::none(),
             nb::arg("viscous_frictions") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(),
             R"doc(Set friction properties for selected degrees of freedom.

Provide at least one property. Static friction efforts must be at least the corresponding dynamic friction efforts.

Args:
    static_frictions: Static friction efforts shaped ``(N, D)``, or ``None`` to preserve them.
    dynamic_frictions: Dynamic friction efforts shaped ``(N, D)``, or ``None`` to preserve them.
    viscous_frictions: Viscous friction coefficients shaped ``(N, D)``, or ``None`` to preserve them.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.

Raises:
    ValueError: If all friction properties are ``None``.
)doc")
        .def("get_dof_drive_model_properties", &physics::Articulation::getDofDriveModelProperties, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get actuator drive-model properties for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Speed-effort gradients, maximum actuator velocities, and velocity-dependent resistances, each shaped ``(N, D)``.
)doc")
        .def("set_dof_drive_model_properties", &physics::Articulation::setDofDriveModelProperties,
             nb::arg("speed_effort_gradients") = nb::none(), nb::arg("maximum_actuator_velocities") = nb::none(),
             nb::arg("velocity_dependent_resistances") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(),
             R"doc(Set actuator drive-model properties for selected degrees of freedom.

Provide at least one property; omitted properties remain unchanged.

Args:
    speed_effort_gradients: Speed-effort gradients shaped ``(N, D)``, or ``None`` to preserve them.
    maximum_actuator_velocities: Maximum actuator velocities shaped ``(N, D)``, or ``None`` to preserve them.
    velocity_dependent_resistances: Velocity-dependent resistances shaped ``(N, D)``, or ``None`` to preserve them.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.

Raises:
    ValueError: If all drive-model properties are ``None``.
)doc")
        .def("get_dof_armatures", &physics::Articulation::getDofArmatures, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get additional inertia terms for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Armature values shaped ``(N, D)``.
)doc")
        .def("set_dof_armatures", &physics::Articulation::setDofArmatures, nb::arg("armatures"), nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set additional inertia terms for selected degrees of freedom.

Args:
    armatures: Armature values shaped ``(N, D)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
)doc")
        .def("get_dof_drive_types", &physics::Articulation::getDofDriveTypes, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get drive types for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Drive type names for each selected articulation and degree of freedom.
)doc")
        .def("set_dof_drive_types", &physics::Articulation::setDofDriveTypes, nb::arg("types"), nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set drive types for selected degrees of freedom.

Args:
    types: One drive type for all selected degrees of freedom, or values for each articulation and degree of freedom.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.

Raises:
    ValueError: If the nested values cannot broadcast to the selected articulations and degrees of freedom.
)doc")
        .def("get_dof_max_velocities", &physics::Articulation::getDofMaxVelocities, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get maximum velocities for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Maximum velocities shaped ``(N, D)``. Angular values are in rad/s and prismatic values are in m/s.
)doc")
        .def("set_dof_max_velocities", &physics::Articulation::setDofMaxVelocities, nb::arg("max_velocities"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set maximum velocities for selected degrees of freedom.

Args:
    max_velocities: Maximum velocities shaped ``(N, D)``; use rad/s for angular values and m/s for prismatic values.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
)doc")
        .def("get_dof_max_efforts", &physics::Articulation::getDofMaxEfforts, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get maximum efforts for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Maximum efforts shaped ``(N, D)``. Angular values are in N m and prismatic values are in N.
)doc")
        .def("set_dof_max_efforts", &physics::Articulation::setDofMaxEfforts, nb::arg("max_efforts"), nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set maximum efforts for selected degrees of freedom.

Args:
    max_efforts: Maximum efforts shaped ``(N, D)``; use N m for angular values and N for prismatic values.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
)doc")
        .def("get_dof_gains", &physics::Articulation::getDofGains, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(),
             R"doc(Get drive gains for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Stiffness and damping values, each shaped ``(N, D)``.
)doc")
        .def("set_dof_gains", &physics::Articulation::setDofGains, nb::arg("stiffnesses") = nb::none(),
             nb::arg("dampings") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("dof_indices") = nb::none(), nb::arg("update_default_gains") = true,
             R"doc(Set drive gains for selected degrees of freedom.

Provide stiffness, damping, or both.

Args:
    stiffnesses: Stiffness values shaped ``(N, D)``, or ``None`` to preserve them.
    dampings: Damping values shaped ``(N, D)``, or ``None`` to preserve them.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
    update_default_gains: Whether to update the gains cached as control-mode defaults.

Raises:
    ValueError: If both ``stiffnesses`` and ``dampings`` are ``None``.
)doc")
        .def("switch_dof_control_mode", &physics::Articulation::switchDofControlMode, nb::arg("mode"), nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Switch the drive control mode for selected degrees of freedom.

Position control restores the cached stiffness and damping, velocity control uses zero stiffness and cached damping,
and effort control uses zero stiffness and damping.

Args:
    mode: Control mode to activate.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.

Raises:
    ValueError: If ``mode`` is not ``"position"``, ``"velocity"``, or ``"effort"``.
)doc")
        .def("get_dof_position_targets", &physics::Articulation::getDofPositionTargets, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get position targets for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Position targets shaped ``(N, D)``. Angular values are in radians and prismatic values are in stage units.
)doc")
        .def("set_dof_position_targets", &physics::Articulation::setDofPositionTargets, nb::arg("positions"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set desired position targets for selected degrees of freedom.

The simulation approaches the targets according to the configured drive gains.

Args:
    positions: Position targets shaped ``(N, D)``; use radians for angular values and stage units for prismatic values.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
)doc")
        .def("get_dof_velocity_targets", &physics::Articulation::getDofVelocityTargets, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Get velocity targets for selected degrees of freedom.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to query, or ``None`` for all.

Returns:
    Velocity targets shaped ``(N, D)``. Angular values are in rad/s and prismatic values are in m/s.
)doc")
        .def("set_dof_velocity_targets", &physics::Articulation::setDofVelocityTargets, nb::arg("velocities"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("dof_indices") = nb::none(),
             R"doc(Set desired velocity targets for selected degrees of freedom.

The simulation approaches the targets according to the configured damping values.

Args:
    velocities: Velocity targets shaped ``(N, D)``; use rad/s for angular values and m/s for prismatic values.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    dof_indices: Degree-of-freedom indices to modify, or ``None`` for all.
)doc")
        .def("get_link_masses", &physics::Articulation::getLinkMasses, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("link_indices") = nb::none(), nb::arg("inverse") = false,
             R"doc(Get masses or inverse masses for selected links.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    link_indices: Link indices to query, or ``None`` for all.
    inverse: Whether to return reciprocal masses.

Returns:
    Masses in kg or reciprocal masses in 1/kg, shaped ``(N, L)``.
)doc")
        .def("set_link_masses", &physics::Articulation::setLinkMasses, nb::arg("masses"), nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("link_indices") = nb::none(),
             R"doc(Set masses for selected links.

Args:
    masses: Masses in kg shaped ``(N, L)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    link_indices: Link indices to modify, or ``None`` for all.
)doc")
        .def("get_link_enabled_gravities", &physics::Articulation::getLinkEnabledGravities, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("link_indices") = nb::none(),
             R"doc(Get gravity-enabled flags for selected links.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.
    link_indices: Link indices to query, or ``None`` for all.

Returns:
    Gravity-enabled flags shaped ``(N, L)``.
)doc")
        .def("set_link_enabled_gravities", &physics::Articulation::setLinkEnabledGravities, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("link_indices") = nb::none(),
             R"doc(Enable or disable gravity for selected links.

Args:
    enabled: Gravity-enabled flags shaped ``(N, L)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
    link_indices: Link indices to modify, or ``None`` for all.
)doc")
        .def("get_solver_iteration_counts", &physics::Articulation::getSolverIterationCounts, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get PhysX solver iteration counts for selected articulations.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.

Returns:
    Position and velocity iteration counts, each shaped ``(N, 1)``.
)doc")
        .def("set_solver_iteration_counts", &physics::Articulation::setSolverIterationCounts,
             nb::arg("position_counts") = nb::none(), nb::arg("velocity_counts") = nb::none(), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Set PhysX solver iteration counts for selected articulations.

Provide at least one count array. Higher counts can improve accuracy at a performance cost.

Args:
    position_counts: Position iteration counts shaped ``(N, 1)``, or ``None`` to preserve them.
    velocity_counts: Velocity iteration counts shaped ``(N, 1)``, or ``None`` to preserve them.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.

Raises:
    ValueError: If both count arrays are ``None``.
)doc")
        .def("get_stabilization_thresholds", &physics::Articulation::getStabilizationThresholds, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get stabilization thresholds for selected articulations.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.

Returns:
    Energy thresholds below which PhysX applies position-based stabilization, shaped ``(N, 1)``.
)doc")
        .def("set_stabilization_thresholds", &physics::Articulation::setStabilizationThresholds, nb::arg("thresholds"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Set stabilization thresholds for selected articulations.

Args:
    thresholds: Energy thresholds below which PhysX applies position-based stabilization, shaped ``(N, 1)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
)doc")
        .def("get_enabled_self_collisions", &physics::Articulation::getEnabledSelfCollisions, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get self-collision-enabled flags for selected articulations.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.

Returns:
    Self-collision-enabled flags shaped ``(N, 1)``.
)doc")
        .def("set_enabled_self_collisions", &physics::Articulation::setEnabledSelfCollisions, nb::arg("enabled"),
             nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Enable or disable self-collision for selected articulations.

Args:
    enabled: Self-collision-enabled flags shaped ``(N, 1)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
)doc")
        .def("get_sleep_thresholds", &physics::Articulation::getSleepThresholds, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get sleep thresholds for selected articulations.

Args:
    indices: Wrapped-articulation indices to query, or ``None`` for all.

Returns:
    Kinetic-energy-per-mass thresholds below which the solver puts articulations to sleep, shaped ``(N, 1)``.
)doc")
        .def("set_sleep_thresholds", &physics::Articulation::setSleepThresholds, nb::arg("thresholds"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Set sleep thresholds for selected articulations.

Args:
    thresholds: Kinetic-energy-per-mass thresholds below which the solver puts articulations to sleep, shaped
        ``(N, 1)``.
    indices: Wrapped-articulation indices to modify, or ``None`` for all.
)doc");

    // physics/ColliderBody.hpp
    nb::class_<physics::ColliderBody, Xform>(m, "ColliderBody",
                                             R"doc(Wrap one or more USD collider bodies.

Resolve paths against the active stage and provide batched collision-property access. Omitted selection arrays
address every wrapped prim.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.
    approximations: Collision approximation name or per-prim names, or ``None`` to preserve authored values.
    apply_collision_apis: Whether to apply the collision API during initialization.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.

Raises:
    RuntimeError: If no active or default stage is available or a path does not resolve to an existing prim.
)doc")
        .def(
            "__init__",
            [](physics::ColliderBody* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& approximations,
               bool applyCollisionApis, bool resetXformOpProperties)
            { new (self) physics::ColliderBody(paths, approximations, applyCollisionApis, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("approximations") = nb::none(),
            nb::arg("apply_collision_apis") = true, nb::arg("reset_xform_op_properties") = true,
            "Initialize the collider-body wrapper.")
        .def_static("are_of_type", &physics::ColliderBody::areOfType, nb::arg("paths"),
                    R"doc(Check whether paths resolve to transformable prims with the collision API.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.

Returns:
    Boolean flags shaped ``(N, 1)``, allocated on the CPU.

Raises:
    RuntimeError: If a path does not resolve to an existing prim.
)doc")
        .def("apply_collision_apis", &physics::ColliderBody::applyCollisionApis, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Apply the collision API to selected prims.

Args:
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("remove_collision_apis", &physics::ColliderBody::removeCollisionApis, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Remove the collision API from selected prims.

Args:
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("set_offsets", &physics::ColliderBody::setOffsets, nb::arg("contact_offsets") = nb::none(),
             nb::arg("rest_offsets") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Set contact and/or rest offsets for selected collider bodies.

Provide at least one offset array. Contact offsets must be positive and greater than the corresponding rest offsets.

Args:
    contact_offsets: Contact offsets in stage units shaped ``(N, 1)``, or ``None`` to preserve them.
    rest_offsets: Rest offsets in stage units shaped ``(N, 1)``, or ``None`` to preserve them.
    indices: Wrapped-prim indices to modify, or ``None`` for all.

Raises:
    ValueError: If both offset arrays are ``None``.
)doc")
        .def("get_offsets", &physics::ColliderBody::getOffsets, nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Get contact and rest offsets for selected collider bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Contact and rest offsets in stage units, each shaped ``(N, 1)``.
)doc")
        .def("set_torsional_patch_radii", &physics::ColliderBody::setTorsionalPatchRadii, nb::arg("radii"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("minimum") = false,
             R"doc(Set torsional friction patch radii for selected collider bodies.

Args:
    radii: Patch radii in stage units shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
    minimum: Whether to set minimum rather than standard patch radii.
)doc")
        .def("get_torsional_patch_radii", &physics::ColliderBody::getTorsionalPatchRadii, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("minimum") = false,
             R"doc(Get torsional friction patch radii for selected collider bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.
    minimum: Whether to get minimum rather than standard patch radii.

Returns:
    Patch radii in stage units shaped ``(N, 1)``.
)doc")
        .def("set_collision_approximations", &physics::ColliderBody::setCollisionApproximations,
             nb::arg("approximations"), nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Set collision geometry approximations for selected prims.

Args:
    approximations: One approximation name for all selected prims, or one name per prim.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_collision_approximations", &physics::ColliderBody::getCollisionApproximations, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get collision geometry approximations for selected prims.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Approximation names in selected-prim order.
)doc")
        .def("set_enabled_collisions", &physics::ColliderBody::setEnabledCollisions, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Enable or disable collision for selected prims.

Args:
    enabled: Collision-enabled flags shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_enabled_collisions", &physics::ColliderBody::getEnabledCollisions, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get collision-enabled flags for selected prims.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Collision-enabled flags shaped ``(N, 1)``.
)doc");

    // physics/GroundPlane.hpp
    nb::class_<physics::GroundPlane, Xform>(m, "GroundPlane",
                                            R"doc(Wrap or create one or more composite USD ground planes.

Each ground plane contains a collision plane and a renderable mesh. Supply either existing paths to wrap or missing
paths to create; creation options are ignored when wrapping. Omitted selection arrays address every wrapped ground
plane.

Args:
    paths: Existing or missing USD ground-plane path or paths, optionally containing regular expressions.
    sizes: Full extents in stage units shaped ``(N, 1)``, or ``None`` to use creation defaults.
    colors: Normalized RGB values or color names, or ``None`` to use creation defaults.
    axes: Case-insensitive ``"X"``, ``"Y"``, or ``"Z"`` normal axis values, or ``None`` to use the stage up-axis.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.

Raises:
    ValueError: If an axis is invalid.
    RuntimeError: If no active or default stage is available or an existing prim lacks the required composite
        structure.
)doc")
        .def(
            "__init__",
            [](physics::GroundPlane* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& sizes, const std::optional<ColorType>& colors,
               const std::optional<std::variant<std::string, std::vector<std::string>>>& axes, bool resetXformOpProperties)
            { new (self) physics::GroundPlane(paths, sizes, colors, axes, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("sizes") = nb::none(), nb::arg("colors") = nb::none(),
            nb::arg("axes") = nb::none(), nb::arg("reset_xform_op_properties") = true,
            "Initialize the ground-plane wrapper, creating missing prims as needed.")
        .def_static("are_of_type", &physics::GroundPlane::areOfType, nb::arg("paths"),
                    R"doc(Check whether paths resolve to composite ground planes.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.

Returns:
    Boolean flags shaped ``(N, 1)``, allocated on the CPU.

Raises:
    RuntimeError: If a path does not resolve to an existing prim.
)doc")
        .def_prop_ro("planes", &physics::GroundPlane::planes, nb::rv_policy::reference_internal,
                     nb::sig("def planes(self) -> isaacsim.foundation.objects.Plane"),
                     "Get the collision-plane wrapper, whose lifetime is tied to this ground-plane wrapper.")
        .def_prop_ro("meshes", &physics::GroundPlane::meshes, nb::rv_policy::reference_internal,
                     nb::sig("def meshes(self) -> isaacsim.foundation.objects.Mesh"),
                     "Get the render-mesh wrapper, whose lifetime is tied to this ground-plane wrapper.")
        .def("set_offsets", &physics::GroundPlane::setOffsets, nb::arg("contact_offsets") = nb::none(),
             nb::arg("rest_offsets") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Set contact and/or rest offsets for selected ground planes.

Provide at least one offset array. Contact offsets must be positive and greater than the corresponding rest offsets.

Args:
    contact_offsets: Contact offsets in stage units shaped ``(N, 1)``, or ``None`` to preserve them.
    rest_offsets: Rest offsets in stage units shaped ``(N, 1)``, or ``None`` to preserve them.
    indices: Wrapped-ground-plane indices to modify, or ``None`` for all.

Raises:
    ValueError: If both offset arrays are ``None``.
)doc")
        .def("get_offsets", &physics::GroundPlane::getOffsets, nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Get contact and rest offsets for selected ground planes.

Args:
    indices: Wrapped-ground-plane indices to query, or ``None`` for all.

Returns:
    Contact and rest offsets in stage units, each shaped ``(N, 1)``.
)doc")
        .def("set_torsional_patch_radii", &physics::GroundPlane::setTorsionalPatchRadii, nb::arg("radii"),
             nb::kw_only(), nb::arg("indices") = nb::none(), nb::arg("minimum") = false,
             R"doc(Set torsional friction patch radii for selected ground planes.

Args:
    radii: Patch radii in stage units shaped ``(N, 1)``.
    indices: Wrapped-ground-plane indices to modify, or ``None`` for all.
    minimum: Whether to set minimum rather than standard patch radii.
)doc")
        .def("get_torsional_patch_radii", &physics::GroundPlane::getTorsionalPatchRadii, nb::kw_only(),
             nb::arg("indices") = nb::none(), nb::arg("minimum") = false,
             R"doc(Get torsional friction patch radii for selected ground planes.

Args:
    indices: Wrapped-ground-plane indices to query, or ``None`` for all.
    minimum: Whether to get minimum rather than standard patch radii.

Returns:
    Patch radii in stage units shaped ``(N, 1)``.
)doc")
        .def("set_enabled_collisions", &physics::GroundPlane::setEnabledCollisions, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Enable or disable collision for selected ground planes.

Args:
    enabled: Collision-enabled flags shaped ``(N, 1)``.
    indices: Wrapped-ground-plane indices to modify, or ``None`` for all.
)doc")
        .def("get_enabled_collisions", &physics::GroundPlane::getEnabledCollisions, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get collision-enabled flags for selected ground planes.

Args:
    indices: Wrapped-ground-plane indices to query, or ``None`` for all.

Returns:
    Collision-enabled flags shaped ``(N, 1)``.
)doc");

    // physics/RigidBody.hpp
    nb::class_<physics::RigidBody, Xform>(m, "RigidBody",
                                          R"doc(Wrap one or more USD rigid bodies.

Resolve paths against the active stage and provide batched rigid-body-property access. Omitted selection arrays
address every wrapped prim.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.
    masses: Initial masses in kg shaped ``(N, 1)``, or ``None`` to preserve authored values.
    densities: Initial densities in kg/m^3 shaped ``(N, 1)``, or ``None`` to preserve authored values.
    apply_physics_apis: Whether to apply the rigid-body and mass APIs during initialization.
    reset_xform_op_properties: Whether to normalize each prim's transform operation stack.

Raises:
    RuntimeError: If no active or default stage is available or a path does not resolve to an existing prim.
)doc")
        .def(
            "__init__",
            [](physics::RigidBody* self, const std::variant<std::string, std::vector<std::string>>& paths,
               const std::optional<array::Array>& masses, const std::optional<array::Array>& densities,
               bool applyPhysicsApis, bool resetXformOpProperties)
            { new (self) physics::RigidBody(paths, masses, densities, applyPhysicsApis, resetXformOpProperties); },
            nb::arg("paths"), nb::kw_only(), nb::arg("masses") = nb::none(), nb::arg("densities") = nb::none(),
            nb::arg("apply_physics_apis") = true, nb::arg("reset_xform_op_properties") = true,
            "Initialize the rigid-body wrapper.")
        .def_static("are_of_type", &physics::RigidBody::areOfType, nb::arg("paths"),
                    R"doc(Check whether paths resolve to transformable prims with the rigid-body API.

Args:
    paths: USD prim path or paths, optionally containing regular expressions.

Returns:
    Boolean flags shaped ``(N, 1)``, allocated on the CPU.

Raises:
    RuntimeError: If a path does not resolve to an existing prim.
)doc")
        .def("apply_physics_apis", &physics::RigidBody::applyPhysicsApis, nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Apply the rigid-body and mass APIs to selected prims.

Args:
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("remove_physics_apis", &physics::RigidBody::removePhysicsApis, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Remove the rigid-body and mass APIs from selected prims.

Args:
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("set_velocities", &physics::RigidBody::setVelocities, nb::arg("linear_velocities") = nb::none(),
             nb::arg("angular_velocities") = nb::none(), nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Set linear and/or angular velocities for selected rigid bodies.

Provide at least one velocity array.

Args:
    linear_velocities: Linear velocities in m/s shaped ``(N, 3)``, or ``None`` to preserve them.
    angular_velocities: Angular velocities in rad/s shaped ``(N, 3)``, or ``None`` to preserve them.
    indices: Wrapped-prim indices to modify, or ``None`` for all.

Raises:
    ValueError: If both velocity arrays are ``None``.
)doc")
        .def("get_velocities", &physics::RigidBody::getVelocities, nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Get linear and angular velocities for selected rigid bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Linear velocities in m/s and angular velocities in rad/s, each shaped ``(N, 3)``.
)doc")
        .def("get_masses", &physics::RigidBody::getMasses, nb::kw_only(), nb::arg("indices") = nb::none(),
             nb::arg("inverse") = false,
             R"doc(Get masses or inverse masses for selected rigid bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.
    inverse: Whether to return reciprocal masses.

Returns:
    Masses in kg or reciprocal masses in 1/kg, shaped ``(N, 1)``.
)doc")
        .def("set_masses", &physics::RigidBody::setMasses, nb::arg("masses"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Set masses for selected rigid bodies.

Args:
    masses: Masses in kg shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_densities", &physics::RigidBody::getDensities, nb::kw_only(), nb::arg("indices") = nb::none(),
             R"doc(Get densities for selected rigid bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Densities in kg/m^3 shaped ``(N, 1)``.
)doc")
        .def("set_densities", &physics::RigidBody::setDensities, nb::arg("densities"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Set densities for selected rigid bodies.

Args:
    densities: Densities in kg/m^3 shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_sleep_thresholds", &physics::RigidBody::getSleepThresholds, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get sleep thresholds for selected rigid bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Kinetic-energy-per-mass thresholds below which the solver puts bodies to sleep, shaped ``(N, 1)``.
)doc")
        .def("set_sleep_thresholds", &physics::RigidBody::setSleepThresholds, nb::arg("thresholds"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Set sleep thresholds for selected rigid bodies.

Args:
    thresholds: Kinetic-energy-per-mass thresholds below which the solver puts bodies to sleep, shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("set_enabled_rigid_bodies", &physics::RigidBody::setEnabledRigidBodies, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Enable or freeze selected rigid bodies.

Disabled bodies remain in collision detection but are not moved by the physics solver.

Args:
    enabled: Rigid-body-dynamics-enabled flags shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_enabled_rigid_bodies", &physics::RigidBody::getEnabledRigidBodies, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get rigid-body-dynamics-enabled flags for selected prims.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Rigid-body-dynamics-enabled flags shaped ``(N, 1)``.
)doc")
        .def("set_enabled_gravities", &physics::RigidBody::setEnabledGravities, nb::arg("enabled"), nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Enable or disable gravity for selected rigid bodies.

Args:
    enabled: Gravity-enabled flags shaped ``(N, 1)``.
    indices: Wrapped-prim indices to modify, or ``None`` for all.
)doc")
        .def("get_enabled_gravities", &physics::RigidBody::getEnabledGravities, nb::kw_only(),
             nb::arg("indices") = nb::none(),
             R"doc(Get gravity-enabled flags for selected rigid bodies.

Args:
    indices: Wrapped-prim indices to query, or ``None`` for all.

Returns:
    Gravity-enabled flags shaped ``(N, 1)``.
)doc");
}
