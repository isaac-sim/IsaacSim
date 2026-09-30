// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "NanobindHelpers.hpp"

#include <isaacsim/physics/manager/PhysicsSceneQuery.hpp>

namespace nb = nanobind;

namespace
{
template <typename HitType>
std::function<bool(const HitType&)> asReportFunction(nb::callable reportFunction)
{
    return [reportFunction = std::move(reportFunction)](const HitType& hit) -> bool
    {
        nb::gil_scoped_acquire gil;
        return nb::cast<bool>(reportFunction(hit));
    };
}
} // namespace

void isaacsim::physics::manager::details::bindPhysicsSceneQuery(nb::module_& module)
{
    using namespace isaacsim::physics::manager;
    using namespace isaacsim::physics::registration;

    module.def(
        "raycast_closest",
        [](const Float3& origin, const Float3& unitDirection, float distance, bool bothSides)
        {
            RaycastHit hit;
            bool hasHit = raycastClosest(origin, unitDirection, distance, hit, bothSides);
            return nb::make_tuple(hasHit, hit);
        },
        nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return the closest hit along a ray.");
    module.def(
        "raycast_any",
        [](const Float3& origin, const Float3& unitDirection, float distance, bool bothSides)
        { return raycastAny(origin, unitDirection, distance, bothSides); },
        nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return whether a ray hits any physics object.");
    module.def(
        "raycast_all",
        [](const Float3& origin, const Float3& unitDirection, float distance, nb::callable reportFunction, bool bothSides) {
            raycastAll(
                origin, unitDirection, distance, asReportFunction<RaycastHit>(std::move(reportFunction)), bothSides);
        },
        nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("report_fn"), nb::arg("both_sides"),
        "Report every hit along a ray.");
    module.def(
        "sweep_sphere_closest",
        [](float radius, const Float3& origin, const Float3& unitDirection, float distance, bool bothSides)
        {
            SweepHit hit;
            bool hasHit = sweepSphereClosest(radius, origin, unitDirection, distance, hit, bothSides);
            return nb::make_tuple(hasHit, hit);
        },
        nb::arg("radius"), nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return the closest hit from sweeping a sphere.");
    module.def(
        "sweep_sphere_any",
        [](float radius, const Float3& origin, const Float3& unitDirection, float distance, bool bothSides)
        { return sweepSphereAny(radius, origin, unitDirection, distance, bothSides); },
        nb::arg("radius"), nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return whether sweeping a sphere hits any physics object.");
    module.def(
        "sweep_sphere_all",
        [](float radius, const Float3& origin, const Float3& unitDirection, float distance, nb::callable reportFunction,
           bool bothSides)
        {
            sweepSphereAll(radius, origin, unitDirection, distance,
                           asReportFunction<SweepHit>(std::move(reportFunction)), bothSides);
        },
        nb::arg("radius"), nb::arg("origin"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("report_fn"),
        nb::arg("both_sides"), "Report every hit from sweeping a sphere.");
    module.def(
        "overlap_sphere",
        [](float radius, const Float3& position, nb::callable reportFunction)
        { return overlapSphere(radius, position, asReportFunction<OverlapHit>(std::move(reportFunction))); },
        nb::arg("radius"), nb::arg("position"), nb::arg("report_fn"),
        "Report every physics object overlapping a sphere.");
    module.def(
        "overlap_sphere_any", [](float radius, const Float3& position) { return overlapSphereAny(radius, position); },
        nb::arg("radius"), nb::arg("position"), "Return whether a sphere overlaps any physics object.");
    module.def(
        "overlap_box",
        [](const Float3& halfExtent, const Float3& position, const Float4& rotation, nb::callable reportFunction)
        { return overlapBox(halfExtent, position, rotation, asReportFunction<OverlapHit>(std::move(reportFunction))); },
        nb::arg("half_extent"), nb::arg("position"), nb::arg("rotation"), nb::arg("report_fn"),
        "Report every physics object overlapping a box.");
    module.def(
        "overlap_box_any",
        [](const Float3& halfExtent, const Float3& position, const Float4& rotation)
        { return overlapBoxAny(halfExtent, position, rotation); },
        nb::arg("half_extent"), nb::arg("position"), nb::arg("rotation"),
        "Return whether a box overlaps any physics object.");
    module.def(
        "overlap_shape",
        [](PathToken geometryPrimitivePath, nb::callable reportFunction)
        { return overlapShape(geometryPrimitivePath, asReportFunction<OverlapHit>(std::move(reportFunction))); },
        nb::arg("g_prim_path"), nb::arg("report_fn"),
        "Report every physics object overlapping the collision shape at a prim path.");
    module.def(
        "overlap_shape_any", [](PathToken geometryPrimitivePath) { return overlapShapeAny(geometryPrimitivePath); },
        nb::arg("g_prim_path"), "Return whether the collision shape at a prim path overlaps any physics object.");
    module.def(
        "sweep_box_closest",
        [](const Float3& halfExtent, const Float3& position, const Float4& rotation, const Float3& unitDirection,
           float distance, bool bothSides)
        {
            SweepHit hit;
            bool hasHit = sweepBoxClosest(halfExtent, position, rotation, unitDirection, distance, hit, bothSides);
            return nb::make_tuple(hasHit, hit);
        },
        nb::arg("half_extent"), nb::arg("position"), nb::arg("rotation"), nb::arg("unit_dir"), nb::arg("distance"),
        nb::arg("both_sides"), "Return the closest hit from sweeping a box.");
    module.def(
        "sweep_shape_closest",
        [](PathToken geometryPrimitivePath, const Float3& unitDirection, float distance, bool bothSides)
        {
            SweepHit hit;
            bool hasHit = sweepShapeClosest(geometryPrimitivePath, unitDirection, distance, hit, bothSides);
            return nb::make_tuple(hasHit, hit);
        },
        nb::arg("g_prim_path"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return the closest hit from sweeping the collision shape at a prim path.");
    module.def(
        "sweep_box_any",
        [](const Float3& halfExtent, const Float3& position, const Float4& rotation, const Float3& unitDirection,
           float distance, bool bothSides)
        { return sweepBoxAny(halfExtent, position, rotation, unitDirection, distance, bothSides); },
        nb::arg("half_extent"), nb::arg("position"), nb::arg("rotation"), nb::arg("unit_dir"), nb::arg("distance"),
        nb::arg("both_sides"), "Return whether sweeping a box hits any physics object.");
    module.def(
        "sweep_shape_any",
        [](PathToken geometryPrimitivePath, const Float3& unitDirection, float distance, bool bothSides)
        { return sweepShapeAny(geometryPrimitivePath, unitDirection, distance, bothSides); },
        nb::arg("g_prim_path"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("both_sides"),
        "Return whether sweeping the collision shape at a prim path hits any physics object.");
    module.def(
        "sweep_box_all",
        [](const Float3& halfExtent, const Float3& position, const Float4& rotation, const Float3& unitDirection,
           float distance, nb::callable reportFunction, bool bothSides)
        {
            sweepBoxAll(halfExtent, position, rotation, unitDirection, distance,
                        asReportFunction<SweepHit>(std::move(reportFunction)), bothSides);
        },
        nb::arg("half_extent"), nb::arg("position"), nb::arg("rotation"), nb::arg("unit_dir"), nb::arg("distance"),
        nb::arg("report_fn"), nb::arg("both_sides"), "Report every hit from sweeping a box.");
    module.def(
        "sweep_shape_all",
        [](PathToken geometryPrimitivePath, const Float3& unitDirection, float distance, nb::callable reportFunction,
           bool bothSides)
        {
            sweepShapeAll(geometryPrimitivePath, unitDirection, distance,
                          asReportFunction<SweepHit>(std::move(reportFunction)), bothSides);
        },
        nb::arg("g_prim_path"), nb::arg("unit_dir"), nb::arg("distance"), nb::arg("report_fn"), nb::arg("both_sides"),
        "Report every hit from sweeping the collision shape at a prim path.");
}
