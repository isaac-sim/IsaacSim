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

"""Unit tests for the virtual gantry force law.

The real physics-tensors articulation view only exists inside a running Kit;
these tests use a duck-typed stand-in that records every
``apply_forces_and_torques_at_position`` call so we can assert magnitude,
direction and one-sided damping against the same math the MuJoCo plugin runs.
The force law is pure Python, so no stage or simulation is required.
"""

import math
from unittest import mock

import omni.kit.test
from isaacsim.robot_setup.virtual_gantry import virtual_gantry as vg
from isaacsim.robot_setup.virtual_gantry._runtime import VirtualGantryManager
from isaacsim.robot_setup.virtual_gantry.virtual_gantry import (
    VirtualGantry,
    VirtualGantryConfig,
    _quat_rotate_wxyz,
)


class _FakeView:
    """Minimal stand-in for the physics-tensors articulation view.

    Args:
        link_transforms: Canned link transforms the fake view returns.
    """

    def __init__(self, link_transforms: list) -> None:
        self._link_transforms = link_transforms
        self.count = len(link_transforms)
        self.max_links = max(len(row) for row in link_transforms)
        self.applied_calls = []

    def get_link_transforms(self) -> list:
        """Return the canned link transforms the gantry reads each step.

        Returns:
            The resulting list.
        """
        return self._link_transforms

    def apply_forces_and_torques_at_position(
        self,
        force_data: object,
        torque_data: object,
        position_data: object,
        indices: object,
        is_global: bool,
    ) -> None:
        """Record one wrench application so a test can assert on what was applied.

        Args:
            force_data: Per-link force buffer.
            torque_data: Per-link torque buffer.
            position_data: Per-link application points.
            indices: Indices of the articulations to write.
            is_global: Whether the wrench is expressed in world space.
        """
        self.applied_calls.append(
            {
                "force": [list(map(float, row)) for row in force_data[0]],
                "position": [list(map(float, row)) for row in position_data[0]],
                "is_global": is_global,
            }
        )


class _FakeArticulation:
    """Mimics enough of isaacsim.core.experimental.prims.Articulation.

    Args:
        link_names: Names of the articulation links.
        link_transforms: Canned link transforms the fake view returns.
    """

    def __init__(self, link_names: list, link_transforms: list) -> None:
        self.link_names = link_names
        self._physics_articulation_view = _FakeView(link_transforms)


def _make_articulation(attach_xyz: tuple, body_offset: tuple = (0.0, 0.0, 0.3)) -> "_FakeArticulation":
    """Build a fake articulation whose attach point sits at ``attach_xyz``.

    Args:
        attach_xyz: Desired world position of the attach point.
        body_offset: Body-local offset from the link origin to the attach point.

    Returns:
        The requested value.
    """
    lx, ly, lz = (attach_xyz[i] - body_offset[i] for i in range(3))
    return _FakeArticulation(["torso_link"], [[(lx, ly, lz, 0.0, 0.0, 0.0, 1.0)]])


def _move_articulation(art: "_FakeArticulation", attach_xyz: tuple, body_offset: tuple = (0.0, 0.0, 0.3)) -> None:
    """Move the fake articulation so its attach point sits at ``attach_xyz``.

    Args:
        art: The fake articulation to move.
        attach_xyz: Desired world position of the attach point.
        body_offset: Body-local offset from the link origin to the attach point.
    """
    lx, ly, lz = (attach_xyz[i] - body_offset[i] for i in range(3))
    art._physics_articulation_view._link_transforms = [[(lx, ly, lz, 0.0, 0.0, 0.0, 1.0)]]


class TestVirtualGantryForceLaw(omni.kit.test.AsyncTestCase):
    """Pure-math force-law tests; warp is forced off so the fake view sees lists."""

    _WARP_SENTINEL = object()

    async def setUp(self) -> None:
        """Force the list fallback path so the fake view sees plain lists."""
        # In a live Kit, warp is importable and apply() would feed real warp
        # arrays to the fake view (which can't item-index them). Force the
        # list fallback path so apply() delivers plain lists instead.
        self._saved_warp = getattr(vg._import_warp, "_module", self._WARP_SENTINEL)
        vg._import_warp._module = None

    async def tearDown(self) -> None:
        """Restore the warp module state saved in :meth:`setUp`."""
        if self._saved_warp is self._WARP_SENTINEL:
            if hasattr(vg._import_warp, "_module"):
                delattr(vg._import_warp, "_module")
        else:
            vg._import_warp._module = self._saved_warp

    def test_empty_reset_does_not_touch_debug_draw(self) -> None:
        """Resetting an empty manager must not touch debug draw during stage teardown."""
        manager = VirtualGantryManager()
        manager._clear_overlay = mock.Mock()

        manager.reset()

        manager._clear_overlay.assert_not_called()

    def test_quat_rotate_identity(self) -> None:
        """The identity quaternion must leave a vector unchanged."""
        v = _quat_rotate_wxyz((1.0, 0.0, 0.0, 0.0), (0.1, 0.2, 0.3))
        for got, exp in zip(v, (0.1, 0.2, 0.3)):
            self.assertAlmostEqual(got, exp, places=9)

    def test_rope_force_zero_when_slack(self) -> None:
        """A body above the rope length is unsupported: the rope applies nothing."""
        cfg = VirtualGantryConfig(rope_length=1.0, anchor_z=1.5)
        art = _make_articulation((0.0, 0.0, 1.0))  # rope_dist 0.5 < 1.0
        cmd = VirtualGantry(art, cfg).step(1.0 / 60.0)
        self.assertFalse(cmd.applied)
        self.assertEqual(art._physics_articulation_view.applied_calls, [])

    def test_rope_force_direction_and_spring(self) -> None:
        """Past the rope length the force pulls back toward the anchor and scales with kp."""
        cfg = VirtualGantryConfig(rope_length=0.5, anchor_z=1.5, kp_pos=5000.0, kd_pos=800.0)
        art = _make_articulation((0.0, 0.0, 0.5))
        cmd = VirtualGantry(art, cfg).step(1.0 / 60.0)
        self.assertTrue(cmd.applied)
        fx, fy, fz = cmd.force_world
        self.assertAlmostEqual(fx, 0.0, places=6)
        self.assertAlmostEqual(fy, 0.0, places=6)
        self.assertAlmostEqual(fz, 2500.0, places=3)  # 5000 * (1.0 - 0.5), no damping yet

    def test_one_sided_damping(self) -> None:
        """Damping resists falling only; it must never add force to a rising body."""
        cfg = VirtualGantryConfig(rope_length=0.5, anchor_z=1.5, kp_pos=5000.0, kd_pos=800.0, ema_alpha=1.0)
        art = _make_articulation((0.0, 0.0, 0.5))
        g = VirtualGantry(art, cfg)
        dt = 1.0 / 60.0
        g.step(dt)
        _move_articulation(art, (0.0, 0.0, 0.44))  # extending
        ce = g.step(dt)
        expected = 5000.0 * (1.06 - 0.5) + 800.0 * (0.06 / dt)
        self.assertAlmostEqual(ce.force_world[2], expected, delta=abs(expected) * 1e-5)
        _move_articulation(art, (0.0, 0.0, 0.48))  # contracting -> no damping
        cc = g.step(dt)
        self.assertAlmostEqual(cc.force_world[2], 5000.0 * (1.02 - 0.5), delta=5e-2)

    def test_force_matches_reference_formula(self) -> None:
        """The applied wrench matches the reference spring-damper formula exactly."""
        cfg = VirtualGantryConfig(rope_length=0.5, anchor_z=1.5, kp_pos=5000.0, kd_pos=800.0, ema_alpha=1.0)
        art = _make_articulation((0.1, -0.2, 0.4))
        g = VirtualGantry(art, cfg)
        dt = 1.0 / 240.0
        g.step(dt)
        new_attach = (0.12, -0.18, 0.38)
        _move_articulation(art, new_attach)
        cmd = g.step(dt)
        anchor = (0.1, -0.2, 1.5)
        dx, dy, dz = (new_attach[i] - anchor[i] for i in range(3))
        rope_dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        prev_dx, prev_dy, prev_dz = 0.1 - anchor[0], -0.2 - anchor[1], 0.4 - anchor[2]
        prev_rope_dist = math.sqrt(prev_dx**2 + prev_dy**2 + prev_dz**2)
        rate = (rope_dist - prev_rope_dist) / dt
        damp = 800.0 * rate if rate > 0 else 0.0
        tension = 5000.0 * (rope_dist - 0.5) + damp
        for got, comp in zip(cmd.force_world, (dx, dy, dz)):
            self.assertAlmostEqual(got, -tension * comp / rope_dist, delta=1e-5)

    def test_unknown_body_disables(self) -> None:
        """An attach body missing from the articulation disables the gantry instead of raising."""
        cfg = VirtualGantryConfig(body_name="nonexistent", enabled=True)
        art = _FakeArticulation(["torso_link"], [[(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)]])
        g = VirtualGantry(art, cfg)
        self.assertFalse(g.is_enabled())
        self.assertFalse(g.step(1.0 / 60.0).applied)

    def test_toggle(self) -> None:
        """Toggling off stops applying force; toggling on resumes it."""
        cfg = VirtualGantryConfig(rope_length=0.5, ema_alpha=1.0)
        g = VirtualGantry(_make_articulation((0.0, 0.0, 0.5)), cfg)
        self.assertTrue(g.is_enabled())
        g.toggle()
        self.assertFalse(g.is_enabled())
        g.toggle()
        self.assertTrue(g.is_enabled())

    def test_adjust_rope_length_clamps_max(self) -> None:
        """Lengthening the rope saturates at the configured maximum."""
        cfg = VirtualGantryConfig(anchor_z=1.5, rope_length=0.5, ema_alpha=1.0)
        g = VirtualGantry(_make_articulation((0.0, 0.0, 0.5)), cfg)
        g.step(1.0 / 60.0)
        g.adjust_rope_length(+10.0)
        self.assertAlmostEqual(g.current_rope_length(), 2.25, places=9)  # 1.5 * 1.5

    def test_anchor_provider_drives_anchor(self) -> None:
        """With a provider the anchor follows the provided point, not the captured spawn x/y."""
        # With a provider, the anchor is the provided point (not auto-captured x/y).
        cfg = VirtualGantryConfig(rope_length=0.5, ema_alpha=1.0)
        art = _make_articulation((0.0, 0.0, 0.5))
        g = VirtualGantry(art, cfg, anchor_provider=lambda: (1.0, 0.0, 1.5))
        cmd = g.step(1.0 / 60.0)
        self.assertTrue(cmd.applied)
        # rope_vec = attach(0,0,0.5) - anchor(1,0,1.5) = (-1,0,-1); force points
        # toward the anchor, i.e. +x and +z.
        self.assertGreater(cmd.force_world[0], 0.0)
        self.assertGreater(cmd.force_world[2], 0.0)
        self.assertEqual(g._anchor[0], 1.0)

    def test_anchor_provider_moves_live(self) -> None:
        """Moving the provider between steps moves the anchor on the next step."""
        cfg = VirtualGantryConfig(rope_length=0.5, ema_alpha=1.0)
        art = _make_articulation((0.0, 0.0, 0.5))
        anchor = {"p": (0.0, 0.0, 1.5)}
        g = VirtualGantry(art, cfg, anchor_provider=lambda: anchor["p"])
        g.step(1.0 / 60.0)
        anchor["p"] = (0.5, 0.0, 1.5)  # operator drags the anchor
        g.step(1.0 / 60.0)
        self.assertEqual(g._anchor[0], 0.5)

    def test_transient_adjust_does_not_survive_reenable(self) -> None:
        """``[`` / ``]`` tweak the live rope only; re-enabling re-derives the length."""
        # [ / ] adjustments mutate the live rope only; a re-enable re-derives.
        cfg = VirtualGantryConfig(anchor_z=1.5, ema_alpha=1.0)  # rope_length auto
        art = _make_articulation((0.0, 0.0, 0.5))
        g = VirtualGantry(art, cfg)
        g.step(1.0 / 60.0)
        self.assertAlmostEqual(g.current_rope_length(), 1.0, places=6)  # |1.5 - 0.5|
        g.adjust_rope_length(-0.3)
        self.assertAlmostEqual(g.current_rope_length(), 0.7, places=6)
        g.disable()
        g.enable()
        g.step(1.0 / 60.0)
        self.assertAlmostEqual(g.current_rope_length(), 1.0, places=6)  # back to auto

    def test_configured_rope_length_survives_reenable(self) -> None:
        """A configured rope length is persistent and survives a re-enable."""
        # set_configured_rope_length is persistent: a re-enable restores it.
        cfg = VirtualGantryConfig(anchor_z=1.5, ema_alpha=1.0)  # rope_length auto
        art = _make_articulation((0.0, 0.0, 0.5))
        g = VirtualGantry(art, cfg)
        g.step(1.0 / 60.0)
        g.set_configured_rope_length(0.6)
        self.assertAlmostEqual(g.current_rope_length(), 0.6, places=6)
        g.disable()
        g.enable()
        g.step(1.0 / 60.0)
        self.assertAlmostEqual(g.current_rope_length(), 0.6, places=6)  # persisted
        # A negative value reverts to auto-derivation on the next capture.
        g.set_configured_rope_length(-1.0)
        g.disable()
        g.enable()
        g.step(1.0 / 60.0)
        self.assertAlmostEqual(g.current_rope_length(), 1.0, places=6)


class TestVirtualGantrySchemaBridge(omni.kit.test.AsyncTestCase):
    """Schema-driven authoring + prim<->config bridge, against a real USD stage."""

    async def setUp(self) -> None:
        """Create the in-memory stage each schema-bridge test authors onto."""
        from pxr import Usd

        self._stage = Usd.Stage.CreateInMemory()

    async def tearDown(self) -> None:
        """Drop the in-memory stage."""
        self._stage = None

    def test_create_unwired_authors_typed_prim(self) -> None:
        """Creating without a robot still authors a typed prim carrying the schema defaults."""
        from isaacsim.robot_setup.virtual_gantry.create import create_virtual_gantry

        prim = create_virtual_gantry(self._stage, parent_path="/World", anchor_z=2.0)
        self.assertEqual(prim.GetTypeName(), vg.GANTRY_PRIM_TYPE)
        # Schema defaults are present on the fresh prim.
        self.assertIsNotNone(prim.GetAttribute(vg._ATTR_STIFFNESS).Get())
        # No articulation on the stage -> relationships stay empty.
        self.assertEqual(prim.GetRelationship(vg._REL_ATTACH_BODY).GetTargets(), [])
        # Anchor sits at the requested world height.
        self.assertAlmostEqual(vg.anchor_from_prim(prim)[2], 2.0, places=6)

    def test_create_is_unique(self) -> None:
        """A second create must not collide with the first prim's path."""
        from isaacsim.robot_setup.virtual_gantry.create import create_virtual_gantry

        a = create_virtual_gantry(self._stage, parent_path="/World")
        b = create_virtual_gantry(self._stage, parent_path="/World")
        self.assertNotEqual(a.GetPath(), b.GetPath())

    def test_create_autowires_articulation(self) -> None:
        """With a single robot on stage both relationships are wired automatically."""
        from isaacsim.robot_setup.virtual_gantry.create import create_virtual_gantry
        from pxr import UsdGeom, UsdPhysics

        robot = UsdGeom.Xform.Define(self._stage, "/World/robot").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        torso = UsdGeom.Xform.Define(self._stage, "/World/robot/torso_link").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(torso)

        prim = create_virtual_gantry(self._stage, parent_path="/World")
        self.assertEqual(vg.attach_body_name_from_prim(prim), "torso_link")
        self.assertEqual(vg.articulation_path_from_prim(prim), "/World/robot")

    def test_create_wires_the_hinted_robot_on_a_multi_robot_stage(self) -> None:
        """The hint must select both the articulation *and* its own attach link.

        With ArticulationRootAPI on each robot's top-level Xform, widening the
        link search to the root's parent reaches the shared scope and can pick a
        sibling robot's torso.
        """
        from isaacsim.robot_setup.virtual_gantry.create import create_virtual_gantry
        from pxr import UsdGeom, UsdPhysics

        for name in ("g1_a", "g1_b"):
            robot = UsdGeom.Xform.Define(self._stage, f"/World/{name}").GetPrim()
            UsdPhysics.ArticulationRootAPI.Apply(robot)
            torso = UsdGeom.Xform.Define(self._stage, f"/World/{name}/torso_link").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(torso)

        # g1_b is second in depth-first order, so a stage-wide link search
        # would return g1_a's torso.
        prim = create_virtual_gantry(self._stage, parent_path="/World", wire_hint="/World/g1_b")
        self.assertEqual(vg.articulation_path_from_prim(prim), "/World/g1_b")
        self.assertEqual(
            vg._first_target(prim, vg._REL_ATTACH_BODY).pathString,
            "/World/g1_b/torso_link",
        )

    def test_create_hint_does_not_prefix_match_a_sibling(self) -> None:
        """A hint of /World/g1 must not match /World/g1_a by string prefix."""
        from isaacsim.robot_setup.virtual_gantry.create import create_virtual_gantry
        from pxr import UsdGeom, UsdPhysics

        for name in ("g1_a", "g1"):
            robot = UsdGeom.Xform.Define(self._stage, f"/World/{name}").GetPrim()
            UsdPhysics.ArticulationRootAPI.Apply(robot)
            torso = UsdGeom.Xform.Define(self._stage, f"/World/{name}/torso_link").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(torso)

        prim = create_virtual_gantry(self._stage, parent_path="/World", wire_hint="/World/g1")
        self.assertEqual(vg.articulation_path_from_prim(prim), "/World/g1")

    def test_config_from_prim_round_trips(self) -> None:
        """Every schema attribute survives a write-then-read round trip."""
        import isaacsim.robot_setup.virtual_gantry_schema as gantry_schema

        path = "/World/VirtualGantry"
        prim = gantry_schema.CreateVirtualGantry(self._stage, path)
        A = gantry_schema.Attributes
        prim.GetAttribute(A.GANTRY_STIFFNESS.name).Set(1234.0)
        prim.GetAttribute(A.GANTRY_DAMPING.name).Set(56.0)
        prim.GetAttribute(A.GANTRY_ROPE_LENGTH.name).Set(0.7)
        prim.GetAttribute(A.GANTRY_EMA_ALPHA.name).Set(0.3)
        prim.GetAttribute(A.GANTRY_MIN_ROPE_LENGTH.name).Set(0.05)
        prim.GetAttribute(A.GANTRY_ENABLED.name).Set(False)

        cfg = vg.config_from_prim(prim)
        self.assertAlmostEqual(cfg.kp_pos, 1234.0, places=6)
        self.assertAlmostEqual(cfg.kd_pos, 56.0, places=6)
        self.assertAlmostEqual(cfg.rope_length, 0.7, places=6)
        self.assertAlmostEqual(cfg.ema_alpha, 0.3, places=6)
        self.assertAlmostEqual(cfg.min_rope_length, 0.05, places=6)
        self.assertFalse(cfg.enabled)

    def test_negative_rope_length_means_auto(self) -> None:
        """A negative rope length is the sentinel for auto-derive, not a literal length."""
        import isaacsim.robot_setup.virtual_gantry_schema as gantry_schema

        prim = gantry_schema.CreateVirtualGantry(self._stage, "/World/VirtualGantry")
        prim.GetAttribute(gantry_schema.Attributes.GANTRY_ROPE_LENGTH.name).Set(-1.0)
        self.assertIsNone(vg.config_from_prim(prim).rope_length)

    def test_write_status_to_prim(self) -> None:
        """The runtime status token is written back onto the prim."""
        import isaacsim.robot_setup.virtual_gantry_schema as gantry_schema

        prim = gantry_schema.CreateVirtualGantry(self._stage, "/World/VirtualGantry")
        vg.write_status_to_prim(prim, "Taut")
        self.assertEqual(prim.GetAttribute(vg._ATTR_STATUS).Get(), "Taut")
