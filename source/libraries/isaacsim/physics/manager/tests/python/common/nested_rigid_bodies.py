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

"""Provide engine-neutral view-resolution scenarios for nested rigid bodies.

The articulation fixture nests links at progressively deeper USD paths beneath
``Robot/base_link``. Its scenarios exercise articulation roots, explicit path
lists, one-level wildcards, recursive descent, named recursive tails, and leaf
alternation. A standalone-cluster fixture covers the same rigid-body resolution
behavior outside an articulation.

Engine-specific subclasses may implement ``_apply_engine_specifics`` when a
backend requires additional articulation configuration.
"""

from __future__ import annotations

import os
import sys
from typing import TypedDict

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
)
from pxr import Gf, Sdf, UsdGeom, UsdPhysics  # noqa: E402


class _NestedLinkPaths(TypedDict):
    """Describe the authored articulation paths."""

    robot: Sdf.Path
    base_link: Sdf.Path
    links: list[Sdf.Path]


def _build_nested_link_articulation(
    scenario: GridTestBase,
    robot_path: Sdf.Path,
    transform: Transform,
    num_links: int = 3,
) -> _NestedLinkPaths:
    """Build an articulation whose links occupy progressively deeper USD paths.

    Args:
        scenario: Scenario whose stage receives the articulation.
        robot_path: Path of the articulation root transform.
        transform: World transform applied to the root.
        num_links: Number of revolute child links to create.

    Returns:
        Paths for the root transform, base link, and all rigid links.

    """
    stage = scenario.stage
    robot_path = Sdf.Path(robot_path)

    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    robot_xform.AddTranslateOp().Set(transform.p)
    robot_xform.AddOrientOp().Set(transform.q)
    robot_prim = robot_xform.GetPrim()

    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

    base_link_path = robot_path.AppendChild("base_link")
    sphere = UsdGeom.Sphere.Define(stage, base_link_path)
    sphere.CreateRadiusAttr(0.1)
    sphere.AddTranslateOp().Set(Gf.Vec3f(0.0))
    base_prim = sphere.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(base_prim)
    UsdPhysics.MassAPI.Apply(base_prim)

    fixed_joint = UsdPhysics.FixedJoint.Define(stage, base_link_path.AppendChild("BaseFixedJoint"))
    fixed_joint.CreateBody1Rel().SetTargets([base_link_path])

    link_size = Gf.Vec3f(0.3, 0.05, 0.05)
    parent_link_path = base_link_path
    link_paths = [base_link_path]
    for i in range(num_links):
        link_path = parent_link_path.AppendChild(f"link_{i}")
        link_position = Gf.Vec3f(0.3, 0.0, 0.0) if i > 0 else Gf.Vec3f(0.15, 0.0, 0.0)
        cube = UsdGeom.Cube.Define(stage, link_path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(link_position)
        cube.AddScaleOp().Set(link_size)
        link_prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link_prim)
        UsdPhysics.MassAPI.Apply(link_prim).CreateMassAttr(1.0)

        joint = UsdPhysics.RevoluteJoint.Define(stage, link_path.AppendChild(f"Joint_{i}"))
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([parent_link_path])
        joint.CreateBody1Rel().SetTargets([link_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.15, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.15, 0.0, 0.0))

        link_paths.append(link_path)
        parent_link_path = link_path

    return {"robot": robot_path, "base_link": base_link_path, "links": link_paths}


class _NestedLinkArticulationBase(GridTestBase):
    """Build a grid of articulations with hierarchically nested links.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.
        num_envs: Number of articulation copies to create.

    """

    NUM_CHILD_LINKS = 3
    LINKS_PER_ROBOT = 1 + NUM_CHILD_LINKS  # base_link + link_0..link_{N-1}

    def __init__(self, test_case: object, device_params: DeviceParams, num_envs: int = 4) -> None:
        grid_params = GridParams(num_envs=num_envs, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        robot_path = self.env_template_path.AppendChild("Robot")
        transform = Transform((0.0, 0.0, 1.0))
        self._paths = _build_nested_link_articulation(self, robot_path, transform, num_links=self.NUM_CHILD_LINKS)

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific articulation configuration."""


class ArticulationViewNestedLinksCommon(_NestedLinkArticulationBase):
    """Resolve nested articulations from their outer Robot transforms."""

    def on_start(self, sim: object) -> None:
        """Check articulation counts, topology, and resolved root paths.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        arti_view = sim.create_articulation_view("/envs/*/Robot")
        self.check_articulation_view(
            arti_view,
            expected_count=self.num_envs,
            expected_max_links=self.LINKS_PER_ROBOT,
            expected_max_dofs=self.NUM_CHILD_LINKS,
            require_homogeneous=True,
        )
        for i in range(self.num_envs):
            assert f"/envs/env{i}/Robot" in arti_view.get_metadata("prim-paths")
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class ArticulationViewAtNestedRootLinkCommon(_NestedLinkArticulationBase):
    """Resolve nested articulations directly from their base links."""

    def on_start(self, sim: object) -> None:
        """Check topology when each base link is used as the view pattern.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        arti_view = sim.create_articulation_view("/envs/*/Robot/base_link")
        self.check_articulation_view(
            arti_view,
            expected_count=self.num_envs,
            expected_max_links=self.LINKS_PER_ROBOT,
            expected_max_dofs=self.NUM_CHILD_LINKS,
            require_homogeneous=True,
        )
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


# ---------------------------------------------------------------------------
# Recursive-pattern variants exercise `**` recursive descent, explicit
# per-prim path lists, and `|` alternation in the leaf component.
# ---------------------------------------------------------------------------


class RigidBodyViewExplicitNestedPathsCommon(_NestedLinkArticulationBase):
    """Resolve each articulation's deepest link from an explicit path list."""

    def on_start(self, sim: object) -> None:
        """Check one explicitly selected deepest link per environment.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        # Hit `link_2` (the deepest link in each env), specified as
        # an explicit per-env path list.
        explicit_paths = [f"/envs/env{i}/Robot/base_link/link_0/link_1/link_2" for i in range(self.num_envs)]
        rb_view = sim.create_rigid_body_view(explicit_paths)
        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class RigidBodyViewRecursiveUnderRobotCommon(_NestedLinkArticulationBase):
    """Resolve every link through recursive descent beneath each Robot."""

    def on_start(self, sim: object) -> None:
        """Check that recursive descent selects every rigid link.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/**")
        self.check_rigid_body_view(rb_view, self.num_envs * self.LINKS_PER_ROBOT)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class RigidBodyViewRecursiveNamedLinkCommon(_NestedLinkArticulationBase):
    """Resolve each deepest link with a named recursive-tail pattern."""

    def on_start(self, sim: object) -> None:
        """Check one recursively selected ``link_2`` per environment.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/**/link_2")
        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class ArticulationViewRecursiveRootLinkCommon(_NestedLinkArticulationBase):
    """Resolve nested articulations with a recursive base-link pattern."""

    def on_start(self, sim: object) -> None:
        """Check articulation topology through recursive root-link resolution.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        arti_view = sim.create_articulation_view("/envs/*/Robot/**/base_link")
        self.check_articulation_view(arti_view, self.num_envs, self.LINKS_PER_ROBOT, self.NUM_CHILD_LINKS, True)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class RigidBodyViewWildcardUnderRobotCommon(_NestedLinkArticulationBase):
    """Resolve only base links with a one-level wildcard under each Robot."""

    def on_start(self, sim: object) -> None:
        """Check that a single wildcard does not descend into child links.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/*")
        # Only `base_link` is at one level — link_0/link_1/link_2 are deeper.
        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class RigidBodyViewNestedAlternationCommon(_NestedLinkArticulationBase):
    """Resolve base and first child links through leaf-name alternation."""

    def on_start(self, sim: object) -> None:
        """Check that alternation returns two rigid bodies per environment.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/base_link|link_0")
        self.check_rigid_body_view(rb_view, self.num_envs * 2)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


# Standalone-cluster variants: a separate test fixture with rigid
# bodies parked under a non-articulation `cluster` xform.


class _StandaloneClusterBase(GridTestBase):
    """Build non-articulated rigid bodies beneath a cluster transform.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.
        num_envs: Number of standalone clusters to create.

    """

    NUM_BODIES_PER_CLUSTER = 4

    def __init__(self, test_case: object, device_params: DeviceParams, num_envs: int = 4) -> None:
        grid_params = GridParams(num_envs=num_envs, env_spacing=2.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        cluster_path = self.env_template_path.AppendChild("cluster")
        UsdGeom.Xform.Define(self.stage, cluster_path)
        for i in range(self.NUM_BODIES_PER_CLUSTER):
            body_path = cluster_path.AppendChild(f"body_{i}")
            sphere = UsdGeom.Sphere.Define(self.stage, body_path)
            sphere.CreateRadiusAttr(0.05)
            sphere.AddTranslateOp().Set(Gf.Vec3f(i * 0.2, 0.0, 0.5))
            UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
            UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateMassAttr(1.0)


class RigidBodyViewStandaloneExplicitPathsCommon(_StandaloneClusterBase):
    """Resolve every standalone cluster body from an explicit path list."""

    def on_start(self, sim: object) -> None:
        """Check explicit-path resolution across clusters and body indices.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        explicit = [
            f"/envs/env{i}/cluster/body_{j}" for i in range(self.num_envs) for j in range(self.NUM_BODIES_PER_CLUSTER)
        ]
        rb_view = sim.create_rigid_body_view(explicit)
        self.check_rigid_body_view(rb_view, self.num_envs * self.NUM_BODIES_PER_CLUSTER)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt


class RigidBodyViewStandaloneRecursiveUnderClusterCommon(_StandaloneClusterBase):
    """Resolve every standalone body through recursive cluster descent."""

    def on_start(self, sim: object) -> None:
        """Check recursive resolution across all standalone clusters.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        rb_view = sim.create_rigid_body_view("/envs/*/cluster/**")
        self.check_rigid_body_view(rb_view, self.num_envs * self.NUM_BODIES_PER_CLUSTER)
        self.finish()

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Accept the unused callback after validation completes during startup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt
