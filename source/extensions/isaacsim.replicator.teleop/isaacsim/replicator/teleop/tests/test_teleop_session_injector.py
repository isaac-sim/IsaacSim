# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Self-contained tests for teleop session injector wiring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import carb.settings
import omni.kit.test
from isaacsim.replicator.episode_recorder import (
    apply_session_injectors,
    clear_session_injectors,
    register_session_injector,
    registered_session_injectors,
)
from isaacsim.replicator.teleop.teleop_session_injector import (
    _RECORD_AIM_KEY,
    _RECORD_HEAD_KEY,
    install_teleop_session_injector,
)


@dataclass
class _ExistingRecordable:
    group: str


class _FakeRecorder:
    def __init__(self, recordables: list[Any] | None = None) -> None:
        self._recordables = list(recordables or [])

    def add(self, recordable: Any) -> None:
        self._recordables.append(recordable)

    def recordables(self) -> list[Any]:
        return list(self._recordables)


class _FakeTeleopManager:
    def add_controller_inputs_observer(self, _observer: Any) -> None:
        return None

    def remove_controller_inputs_observer(self, _observer: Any) -> None:
        return None

    def add_head_observer(self, _observer: Any) -> None:
        return None

    def remove_head_observer(self, _observer: Any) -> None:
        return None


class TestTeleopSessionInjector(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        self._saved_injectors = registered_session_injectors()
        clear_session_injectors()
        settings = carb.settings.get_settings()
        self._saved_record_aim = settings.get(_RECORD_AIM_KEY)
        self._saved_record_head = settings.get(_RECORD_HEAD_KEY)
        settings.set_bool(_RECORD_AIM_KEY, True)
        settings.set_bool(_RECORD_HEAD_KEY, True)

    async def tearDown(self) -> None:
        clear_session_injectors()
        for injector in self._saved_injectors:
            register_session_injector(injector)
        settings = carb.settings.get_settings()
        if self._saved_record_aim is None:
            settings.destroy_item(_RECORD_AIM_KEY)
        else:
            settings.set_bool(_RECORD_AIM_KEY, bool(self._saved_record_aim))
        if self._saved_record_head is None:
            settings.destroy_item(_RECORD_HEAD_KEY)
        else:
            settings.set_bool(_RECORD_HEAD_KEY, bool(self._saved_record_head))

    async def test_injector_adds_controller_and_head_recordables(self) -> None:
        unregister = install_teleop_session_injector(_FakeTeleopManager())
        recorder = _FakeRecorder()

        apply_session_injectors(recorder)

        by_group = {recordable.group: recordable for recordable in recorder.recordables()}
        self.assertEqual(set(by_group), {"teleop/left", "teleop/right", "teleop/head"})
        self.assertTrue(by_group["teleop/left"].record_aim_pose)
        self.assertTrue(by_group["teleop/right"].record_aim_pose)

        unregister()
        recorder_after_unregister = _FakeRecorder()
        apply_session_injectors(recorder_after_unregister)
        self.assertEqual(recorder_after_unregister.recordables(), [])

    async def test_injector_respects_settings_and_existing_groups(self) -> None:
        settings = carb.settings.get_settings()
        settings.set_bool(_RECORD_AIM_KEY, False)
        settings.set_bool(_RECORD_HEAD_KEY, False)
        existing_left = _ExistingRecordable("teleop/left")
        recorder = _FakeRecorder([existing_left])

        install_teleop_session_injector(_FakeTeleopManager())
        apply_session_injectors(recorder)

        by_group = {recordable.group: recordable for recordable in recorder.recordables()}
        self.assertIs(by_group["teleop/left"], existing_left)
        self.assertIn("teleop/right", by_group)
        self.assertFalse(by_group["teleop/right"].record_aim_pose)
        self.assertNotIn("teleop/head", by_group)
