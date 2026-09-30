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

"""Test the high-level Python viewport interface."""

from typing import Any
from unittest.mock import Mock

import pytest
from isaacsim.foundation.objects import Stage
from isaacsim.ovgl_viewport.debug import Camera, Viewport, author_viewport
from isaacsim.ovgl_viewport.debug.impl import api
from pxr import Gf, Sdf, Usd, UsdUtils


def test_camera_defaults() -> None:
    """Test camera defaults."""
    camera = Camera()

    assert camera.target == (0.0, 0.0, 0.0)
    assert camera.distance == 6.0


def test_author_viewport() -> None:
    """Test authoring the default OVGL render target."""
    default_stage = Stage("openusd").create_stage()
    stage = Stage("openusd").create_stage(make_default=False)
    try:
        author_viewport(stage, resolution=(640, 360))
        usd_stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(stage.get_stage_id()))
        default_usd_stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(default_stage.get_stage_id()))
        camera = usd_stage.GetPrimAtPath("/ViewportCamera")
        render_product = usd_stage.GetPrimAtPath("/Render/ViewportProduct")

        assert camera.GetTypeName() == "Camera"
        assert not default_usd_stage.GetPrimAtPath("/ViewportCamera")
        assert render_product.GetAttribute("resolution").Get() == Gf.Vec2i(640, 360)
        assert render_product.GetRelationship("camera").GetTargets() == [Sdf.Path("/ViewportCamera")]
        assert render_product.GetRelationship("orderedVars").GetTargets() == [Sdf.Path("/Render/Vars/LdrColor")]
    finally:
        stage.close_stage()
        default_stage.close_stage()


def test_viewport_context_manager(monkeypatch: pytest.MonkeyPatch) -> None:
    """Test configuration forwarding and deterministic closure.

    Args:
        monkeypatch: Pytest fixture used to replace the native viewport.
    """

    class FakeStage:
        """Minimal valid OVStage interface."""

        def is_valid(self) -> bool:
            """Return whether the fake stage is valid.

            Returns:
                Whether the stage is valid.
            """
            return True

        def get_backend(self) -> str:
            """Return the fake stage backend.

            Returns:
                Stage backend name.
            """
            return "ovstage"

        def get_stage_ptr(self) -> int:
            """Return a nonzero fake native pointer.

            Returns:
                Fake native pointer.
            """
            return 123

        def get_stage_id(self) -> int:
            """Return a fake stage identifier.

            Returns:
                Fake stage identifier.
            """
            return 1

    class FakeNativeViewport:
        """Record arguments forwarded by the public viewport.

        Args:
            stage: Fake native stage pointer.
            camera_pose_writer: Camera callback forwarded by the wrapper.
            config: Native viewport configuration.
        """

        last_instance: Any = None

        def __init__(self, stage: int, camera_pose_writer: Any, config: Any) -> None:
            self.stage = stage
            self.camera_pose_writer = camera_pose_writer
            self.config = config
            self.closed = False
            FakeNativeViewport.last_instance = self

        def close(self) -> None:
            """Record native resource closure."""
            self.closed = True

        def poll_events(self) -> bool:
            """Return a fixed open state.

            Returns:
                Always true.
            """
            return True

        def render(self) -> object:
            """Return a fake frame.

            Returns:
                New fake frame.
            """
            return object()

    monkeypatch.setattr(api, "_NativeViewport", FakeNativeViewport)
    camera = Camera(target=(1.0, 2.0, 3.0), distance=4.0)

    def writer(_: Any) -> None:
        """Accept camera poses for the fake viewport."""

    with Viewport(
        FakeStage(),
        title="Test viewport",
        visible=False,
        camera=camera,
        camera_pose_writer=writer,
    ) as viewport:
        native = FakeNativeViewport.last_instance
        assert native is not None
        assert native.stage == 123
        assert native.camera_pose_writer is writer
        assert native.config.title == "Test viewport"
        assert not native.config.visible
        assert tuple(native.config.camera.target) == camera.target
        assert viewport.poll_events()
        assert not viewport.closed

    assert viewport.closed
    assert native.closed
    with pytest.raises(RuntimeError, match="closed"):
        viewport.render()


def test_viewport_uses_supplied_nondefault_stage(monkeypatch: pytest.MonkeyPatch) -> None:
    """Test automatic camera publication targets the supplied stage.

    Args:
        monkeypatch: Pytest fixture used to replace the native viewport.
    """
    stage_text = '#usda 1.0\n\ndef Camera "ViewportCamera" {}\n'
    default_stage = Stage("ovstage").import_stage_from_string(stage_text)
    stage = Stage("ovstage").import_stage_from_string(stage_text, make_default=False)
    native_viewport = Mock()
    monkeypatch.setattr(api, "_NativeViewport", native_viewport)

    try:
        viewport = Viewport(stage)
        assert viewport._camera_xform is not None
        assert viewport._camera_xform.get_stage().get_stage_id() == stage.get_stage_id()
        viewport.close()
    finally:
        stage.close_stage()
        default_stage.close_stage()
