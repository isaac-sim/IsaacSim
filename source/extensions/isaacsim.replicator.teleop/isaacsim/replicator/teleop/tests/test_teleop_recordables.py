# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Self-contained tests for teleop controller/head recordables."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import omni.kit.app
import omni.kit.test
from isaacsim.replicator.teleop import TeleopControllerRecordable, TeleopHeadRecordable


@dataclass
class _Vec3:
    x: float
    y: float
    z: float


@dataclass
class _Quat:
    x: float
    y: float
    z: float
    w: float


@dataclass
class _Pose:
    position: _Vec3
    orientation: _Quat


@dataclass
class _TrackedPose:
    pose: _Pose | None = None
    is_valid: bool = True


@dataclass
class _Inputs:
    trigger_value: float = 0.0
    squeeze_value: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0
    primary_click: bool = False
    secondary_click: bool = False
    thumbstick_click: bool = False


@dataclass
class _ControllerSnapshot:
    inputs: _Inputs = field(default_factory=_Inputs)
    aim_pose: _TrackedPose = field(default_factory=_TrackedPose)


class _FakeTeleopManager:
    def __init__(self) -> None:
        self.controller_observers: list[Any] = []
        self.head_observers: list[Any] = []

    def add_controller_inputs_observer(self, observer) -> None:  # noqa: ANN001
        self.controller_observers.append(observer)

    def remove_controller_inputs_observer(self, observer) -> None:  # noqa: ANN001
        try:
            self.controller_observers.remove(observer)
        except ValueError:
            pass

    def add_head_observer(self, observer) -> None:  # noqa: ANN001
        self.head_observers.append(observer)

    def remove_head_observer(self, observer) -> None:  # noqa: ANN001
        try:
            self.head_observers.remove(observer)
        except ValueError:
            pass

    def emit_controllers(self, left, right) -> None:  # noqa: ANN001
        for observer in list(self.controller_observers):
            observer(left, right)

    def emit_head(self, head) -> None:  # noqa: ANN001
        for observer in list(self.head_observers):
            observer(head)


def _pose(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> _TrackedPose:
    return _TrackedPose(pose=_Pose(position=_Vec3(x, y, z), orientation=_Quat(qx, qy, qz, qw)), is_valid=True)


class TestTeleopRecordables(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()

    async def test_controller_recordable_samples_selected_side(self) -> None:
        tm = _FakeTeleopManager()
        rec = TeleopControllerRecordable(group="teleop/left", side="left", teleop_manager=tm)
        rec.on_session_open(None)

        left = _ControllerSnapshot(
            inputs=_Inputs(
                trigger_value=0.25,
                squeeze_value=0.5,
                thumbstick_x=-0.75,
                thumbstick_y=1.0,
                primary_click=True,
                secondary_click=False,
                thumbstick_click=True,
            ),
            aim_pose=_pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.4),
        )
        right = _ControllerSnapshot(
            inputs=_Inputs(trigger_value=0.9),
            aim_pose=_pose(9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0),
        )
        tm.emit_controllers(left, right)

        frame = rec.sample()
        self.assertAlmostEqual(frame["trigger"], 0.25)
        self.assertAlmostEqual(frame["squeeze"], 0.5)
        self.assertAlmostEqual(frame["thumbstick_x"], -0.75)
        self.assertAlmostEqual(frame["thumbstick_y"], 1.0)
        self.assertEqual(frame["primary_click"], 1)
        self.assertEqual(frame["secondary_click"], 0)
        self.assertEqual(frame["thumbstick_click"], 1)
        np.testing.assert_allclose(frame["aim_position"], np.array([1.0, 2.0, 3.0], dtype=np.float32))
        np.testing.assert_allclose(frame["aim_orientation"], np.array([0.4, 0.1, 0.2, 0.3], dtype=np.float32))

        rec.on_session_close()
        self.assertEqual(tm.controller_observers, [])

    async def test_controller_recordable_defaults_invalid_or_disabled_aim(self) -> None:
        tm = _FakeTeleopManager()
        rec = TeleopControllerRecordable(group="teleop/right", side="right", teleop_manager=tm)
        rec.on_session_open(None)

        frame = rec.sample()
        np.testing.assert_array_equal(frame["aim_position"], np.zeros(3, dtype=np.float32))
        np.testing.assert_array_equal(frame["aim_orientation"], np.zeros(4, dtype=np.float32))

        no_aim = TeleopControllerRecordable(
            group="teleop/right/no_aim",
            side="right",
            record_aim_pose=False,
            teleop_manager=tm,
        )
        channels = no_aim.describe_channels()
        self.assertNotIn("aim_position", channels)
        self.assertNotIn("aim_orientation", channels)
        self.assertNotIn("aim_position", no_aim.sample())

        restored = TeleopControllerRecordable.from_manifest(
            {"group": "teleop/right", "side": "right", "record_aim_pose": False}
        )
        self.assertFalse(restored.record_aim_pose)

    async def test_head_recordable_samples_and_detaches(self) -> None:
        tm = _FakeTeleopManager()
        rec = TeleopHeadRecordable(teleop_manager=tm)
        rec.on_session_open(None)

        frame = rec.sample()
        np.testing.assert_array_equal(frame["position"], np.zeros(3, dtype=np.float32))
        np.testing.assert_array_equal(frame["orientation"], np.zeros(4, dtype=np.float32))

        tm.emit_head(_pose(-1.0, 2.5, 4.0, 0.2, 0.3, 0.4, 0.5))
        frame = rec.sample()
        np.testing.assert_allclose(frame["position"], np.array([-1.0, 2.5, 4.0], dtype=np.float32))
        np.testing.assert_allclose(frame["orientation"], np.array([0.5, 0.2, 0.3, 0.4], dtype=np.float32))

        rec.on_session_close()
        self.assertEqual(tm.head_observers, [])
