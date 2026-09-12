# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for render-product retargeting in IsaacAttachHydraTexture."""

import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import omni.kit.test

MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnIsaacAttachHydraTexture.py"
SPEC = importlib.util.spec_from_file_location("_attach_hydra_texture_retarget", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
State = MODULE.OgnIsaacAttachHydraTextureInternalState


class TestAttachHydraTextureRetarget(omni.kit.test.AsyncTestCase):
    """Verify cached Hydra state follows the current render product target."""

    async def test_new_render_product_clears_cached_attachment_state(self) -> None:
        """Switching targets should force texture and render-var recreation."""
        state = State.__new__(State)
        old_texture = MagicMock()
        state.hydra_texture = old_texture
        state.applied_render_vars = {"LdrColor", "Depth"}
        state.render_product_path = "/Render/ProductA"
        state.rp_sub_stop = object()
        state.rp_sub_play = object()

        state.set_render_product("/Render/ProductB")

        old_texture.set_updates_enabled.assert_called_once_with(False)
        self.assertIsNone(state.hydra_texture)
        self.assertEqual(state.applied_render_vars, set())
        self.assertIsNone(state.rp_sub_stop)
        self.assertIsNone(state.rp_sub_play)
        self.assertEqual(state.render_product_path, "/Render/ProductB")

    async def test_same_render_product_preserves_cached_state(self) -> None:
        """Repeated evaluation of the same target should keep the existing texture."""
        state = State.__new__(State)
        texture = MagicMock()
        state.hydra_texture = texture
        state.applied_render_vars = {"LdrColor"}
        state.render_product_path = "/Render/ProductA"
        state.rp_sub_stop = object()
        state.rp_sub_play = object()

        state.set_render_product("/Render/ProductA")

        texture.set_updates_enabled.assert_not_called()
        self.assertIs(state.hydra_texture, texture)
        self.assertEqual(state.applied_render_vars, {"LdrColor"})
