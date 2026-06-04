# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Regression tests for known Newton initialization errors.

Each test constructs a minimal USD scene that triggers a specific known
limitation and asserts the expected error is produced. A test passes when
the limitation is still present; a failure indicates it may have been fixed
or the scene setup needs revisiting.
"""

from __future__ import annotations

import ctypes
import os
import sys
import tempfile
import unittest
from contextlib import contextmanager

import carb
import carb.eventdispatcher
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


@contextmanager
def _capture_errors():
    """Capture carb ERROR-level log messages via the event dispatcher."""
    messages: list[str] = []

    def _on_event(e):
        source = e.get("source") or ""
        if source == "omni.kit.app._impl":
            return
        msg = e.get("message") or ""
        if msg:
            messages.append(msg)

    sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_ERROR_LOG_IMMEDIATE,
        on_event=_on_event,
        observer_name="test_init_errors._capture_errors",
    )
    try:
        yield messages
    finally:
        sub = None  # noqa: F841


def _flush_process_output():
    sys.stdout.flush()
    sys.stderr.flush()
    ctypes.CDLL(None).fflush(None)


class _CaptureFds:
    """Redirect process-level output fds to a temp file during expected errors."""

    def __init__(self, fds=(1, 2)):
        self._fds = fds

    def __enter__(self):
        self._tmp = tempfile.TemporaryFile(mode="w+b")
        _flush_process_output()
        self._saved = {fd: os.dup(fd) for fd in self._fds}
        for fd in self._fds:
            os.dup2(self._tmp.fileno(), fd)
        self.output = ""
        return self

    def __exit__(self, *_):
        _flush_process_output()
        for fd, saved in self._saved.items():
            os.dup2(saved, fd)
            os.close(saved)
        self._tmp.seek(0)
        self.output = self._tmp.read().decode(errors="replace")
        self._tmp.close()


class TestNewtonInitErrors(omni.kit.test.AsyncTestCase):
    """Tests that known Newton initialization errors are produced as expected."""

    async def setUp(self):
        await stage_utils.create_new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self):
        self.timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().close_stage_async()

    async def _play_and_capture(self) -> list[str]:
        with _CaptureFds() as output, _capture_errors() as msgs:
            self.timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            self.timeline.stop()
        if output.output:
            msgs.append(output.output)
        return msgs

    def _assert_error(self, msgs, fragment, label):
        combined = "\n".join(msgs)
        self.assertIn(fragment, combined, msg=f"Expected {label}. Captured:\n{combined}")

    # ------------------------------------------------------------------
    # Helpers for building minimal valid articulations
    # ------------------------------------------------------------------

    def _make_articulation_root(self, path: str, mass: float = 1.0) -> Usd.Prim:
        geom = UsdGeom.Cube.Define(self.stage, path)
        geom.GetSizeAttr().Set(0.2)
        prim = geom.GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(mass)
        return prim

    def _make_link(self, path: str, mass: float = 0.5, z: float = 0.3) -> Usd.Prim:
        geom = UsdGeom.Cube.Define(self.stage, path)
        geom.GetSizeAttr().Set(0.1)
        prim = geom.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(mass)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, z))
        return prim

    def _make_joint(self, path: str, parent: str, child: str) -> Usd.Prim:
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, path)
        joint.GetBody0Rel().SetTargets([Sdf.Path(parent)])
        joint.GetBody1Rel().SetTargets([Sdf.Path(child)])
        joint.CreateAxisAttr("Z")
        return joint.GetPrim()

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    async def test_scene_without_joints(self):
        """Rigid body with no collision geometry and no authored mass.

        Without a collider, Newton cannot compute body mass from shapes (mass
        stays zero). Bodies with zero mass are skipped when Newton adds free
        joints to floating bodies, leaving joint_count == 0 and triggering the
        MuJoCo conversion error.

        Expected error:
            The model must have at least one joint to be able to convert
            it to MuJoCo.
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")

        cube = UsdGeom.Cube.Define(self.stage, "/World/Cube")
        cube.GetSizeAttr().Set(1.0)
        prim = cube.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "at least one joint", "at-least-one-joint error")

    async def test_reversed_joints(self):
        """Three-body chain where the second joint has body0/body1 swapped.

        Newton requires physics:body0 to be the parent body and physics:body1
        to be the child body. Newton traverses from the articulation root and
        detects a reversal when a joint's body0 is a leaf (not the current
        parent) while body1 is the body just visited.

        Expected error:
            Reversed joints are not supported
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")
        self._make_articulation_root("/World/Robot")
        self._make_link("/World/Robot/BodyA", z=0.3)
        self._make_link("/World/Robot/BodyB", z=0.6)

        # First joint is correct: Root (body0=parent) → BodyA (body1=child)
        self._make_joint("/World/Robot/JointA", "/World/Robot", "/World/Robot/BodyA")

        # Second joint is REVERSED: body0=BodyB (child), body1=BodyA (parent)
        reversed_joint = UsdPhysics.RevoluteJoint.Define(self.stage, "/World/Robot/JointB")
        reversed_joint.GetBody0Rel().SetTargets([Sdf.Path("/World/Robot/BodyB")])
        reversed_joint.GetBody1Rel().SetTargets([Sdf.Path("/World/Robot/BodyA")])
        reversed_joint.CreateAxisAttr("Z")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "Reversed joints are not supported", "reversed joints error")

    async def test_joint_graph_cycle(self):
        """Three-body articulation where the last joint closes a loop back to root.

        Newton does not support closed kinematic chains. A joint connecting a
        downstream body back to an ancestor creates a cycle in the joint graph.

        Expected error:
            Joint graph contains a cycle
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")
        self._make_articulation_root("/World/Robot")
        self._make_link("/World/Robot/BodyA", z=0.3)
        self._make_link("/World/Robot/BodyB", z=0.6)

        self._make_joint("/World/Robot/JointA", "/World/Robot", "/World/Robot/BodyA")
        self._make_joint("/World/Robot/JointB", "/World/Robot/BodyA", "/World/Robot/BodyB")
        # Closing joint: BodyB → Root creates a cycle
        self._make_joint("/World/Robot/JointC", "/World/Robot/BodyB", "/World/Robot")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "Joint graph contains a cycle", "joint cycle error")

    async def test_mass_inertia_below_mjminval(self):
        """Link with mass far below Newton's minimum enforced value (mjMINVAL).

        Newton's MuJoCo solver requires all dynamic bodies to have mass and
        inertia above mjMINVAL (~1e-6). PhysX has no such minimum.

        Expected error:
            mass and inertia of moving bodies must be larger than mjMINVAL
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")
        self._make_articulation_root("/World/Robot")
        link = self._make_link("/World/Robot/TinyLink")

        # Overwrite mass and inertia to values far below mjMINVAL
        mass_api = UsdPhysics.MassAPI.Apply(link)
        mass_api.GetMassAttr().Set(1e-20)
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(1e-20, 1e-20, 1e-20))

        self._make_joint("/World/Robot/Joint", "/World/Robot", "/World/Robot/TinyLink")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "must be larger than mjMINVAL", "mjMINVAL error")

    async def test_zero_size_collision_shape(self):
        """Rigid body with a zero-size cube collision shape.

        Newton requires all collision geometry to have non-zero dimensions.
        PhysX accepts zero-size shapes silently.

        Expected error:
            Only plane shapes are allowed to have a size of zero
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")
        self._make_articulation_root("/World/Robot")

        geom = UsdGeom.Cube.Define(self.stage, "/World/Robot/ZeroLink")
        geom.GetSizeAttr().Set(0.0)  # zero size
        prim = geom.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(0.5)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.3))

        self._make_joint("/World/Robot/Joint", "/World/Robot", "/World/Robot/ZeroLink")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "Only plane shapes are allowed to have a size of zero", "zero-size shape error")

    async def test_orphan_joints(self):
        """Two rigid bodies connected by a joint with no ArticulationRootAPI.

        All joints must belong to an articulation. Newton reports joints that
        exist outside any articulation root.

        Expected error:
            joint(s) not belonging to any articulation
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")

        for path, z in [("/World/Body0", 0.0), ("/World/Body1", 0.4)]:
            geom = UsdGeom.Cube.Define(self.stage, path)
            geom.GetSizeAttr().Set(0.2)
            prim = geom.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(1.0)
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, z))

        # Joint exists but there is no ArticulationRootAPI anywhere in the stage
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, "/World/OrphanJoint")
        joint.GetBody0Rel().SetTargets([Sdf.Path("/World/Body0")])
        joint.GetBody1Rel().SetTargets([Sdf.Path("/World/Body1")])
        joint.CreateAxisAttr("Z")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "not belonging to any articulation", "orphan joints error")

    async def test_usd_composition_errors(self):
        """Stage with an unresolved USD reference.

        Newton's stage parser is stricter than PhysX about USD composition
        validity. A prim referencing a non-existent file creates a composition
        error that Newton detects and reports.

        Expected error:
            USD stage has composition errors
        """
        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")

        # Unresolved reference on a prim creates a USD composition error
        broken = self.stage.DefinePrim("/World/BrokenRef", "Xform")
        broken.GetReferences().AddReference("./does_not_exist.usd")

        # Valid articulation alongside so we don't hit "at least one joint" first
        self._make_articulation_root("/World/Robot")
        self._make_link("/World/Robot/Link")
        self._make_joint("/World/Robot/Joint", "/World/Robot", "/World/Robot/Link")

        SimulationManager.switch_physics_engine("newton")
        msgs = await self._play_and_capture()
        self._assert_error(msgs, "USD stage has composition errors", "composition errors")

    @unittest.skip(
        "Contact count warning requires a dense contact scene (200+ simultaneous contacts). "
        "Newton auto-estimates nconmax from geometry and uses max(user, estimated), so a "
        "small programmatic scene cannot reliably exceed it. Test via validation sweep instead."
    )
    async def test_contact_count_exceeded(self):
        """Simulation with more contacts than the pre-allocated nconmax buffer.

        The MuJoCo Warp solver prints a warning via CUDA kernel printf when
        the contact count exceeds nconmax. Setting nconmax=1 on the solver
        config guarantees any multi-contact scene triggers the message.

        nconmax must be set AFTER switch_physics_engine so the stage reset
        that occurs during the switch does not discard the modification.

        Expected output (captured from fd 1):
            Number of Newton contacts (N) exceeded MJWarp limit (1). Increase nconmax.
        """
        import isaacsim.physics.newton as newton_ext
        import warp as wp

        UsdPhysics.Scene.Define(self.stage, "/PhysicsScene")

        # Stack of 5 cubes: each pair of adjacent cubes generates multiple contacts.
        # With nconmax=1, even 2 contacts trigger the warning.
        prev_path = None
        for i in range(5):
            path = f"/World/Cube{i}"
            geom = UsdGeom.Cube.Define(self.stage, path)
            geom.GetSizeAttr().Set(0.2)
            prim = geom.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).GetMassAttr().Set(1.0)
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, i * 0.21))
            if i == 0:
                UsdPhysics.ArticulationRootAPI.Apply(prim)
            elif prev_path is not None:
                self._make_joint(f"/World/Joint{i}", prev_path, path)
            prev_path = path

        # Switch first, then set nconmax=1 so the modification survives the
        # stage reset that switch_physics_engine triggers internally.
        SimulationManager.switch_physics_engine("newton")
        await omni.kit.app.get_app().next_update_async()

        ns = newton_ext.acquire_stage()
        if ns is not None:
            ns.cfg.solver_cfg.nconmax = 1

        with _CaptureFds(fds=(1,)) as cap:
            self.timeline.play()
            for _ in range(10):
                await omni.kit.app.get_app().next_update_async()
            self.timeline.stop()
            wp.synchronize()  # flush CUDA printf buffer before restoring fd 1
            _flush_process_output()

        self.assertIn(
            "exceeded MJWarp limit",
            cap.output,
            msg=f"Expected nconmax exceeded warning in stdout. Captured:\n{cap.output[:500]}",
        )

        # Restore default so subsequent tests are unaffected
        if ns is not None:
            ns.cfg.solver_cfg.nconmax = 200
