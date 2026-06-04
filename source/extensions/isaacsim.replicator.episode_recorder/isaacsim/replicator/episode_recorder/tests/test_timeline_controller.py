# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

"""Unit tests for :class:`TimelineDrivenEpisodeController`.

Exercises the PLAY/STOP gating logic (including the runtime-settable
``auto_start_on_play`` flag) by invoking the private ``_on_started`` /
``_on_stopped`` callbacks directly instead of going through the real
``SimulationManager`` event bus - the bus is tested elsewhere and the
behavior we care about here is the controller's internal gating.
"""

from __future__ import annotations

import tempfile

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.usd
from isaacsim.replicator.episode_recorder import (
    ChannelDescriptor,
    EpisodeRecorder,
    Recordable,
    ReplayPolicy,
    SamplingConfig,
    TimelineDrivenEpisodeController,
    register_recordable,
    registered_types,
    unregister_recordable,
)

_NOOP_TYPE_ID = "_test_timeline_controller_noop"


class _NoopRecordable(Recordable):
    """Minimal :class:`Recordable` that doesn't touch the stage or produce frames."""

    TYPE_ID = _NOOP_TYPE_ID

    def __init__(self, *, group: str = "state/noop") -> None:
        super().__init__(group=group)

    def describe_channels(self) -> dict[str, ChannelDescriptor]:
        return {"value": ChannelDescriptor(shape=(), dtype="i8")}

    def on_session_open(self, stage) -> None:
        pass

    def sample(self) -> dict[str, np.ndarray]:
        return {"value": np.int64(0)}

    def apply(self, frame, *, policy: ReplayPolicy) -> None:
        pass

    def to_manifest(self) -> dict[str, object]:
        return {"type": self.TYPE_ID, "group": self.group}

    @classmethod
    def from_manifest(cls, entry):
        return cls(group=str(entry["group"]))


def _make_open_recorder(output_dir: str) -> EpisodeRecorder:
    recorder = EpisodeRecorder(
        output_dir,
        file_prefix="timeline_ctrl",
        sampling=SamplingConfig(mode="app_update"),
        link_stage_snapshot=False,
        auto_attach_sim_time=False,
    )
    recorder.add(_NoopRecordable())
    recorder.open_session()
    return recorder


class TimelineControllerTests(omni.kit.test.AsyncTestCase):
    async def setUp(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        if _NOOP_TYPE_ID not in registered_types():
            register_recordable(_NoopRecordable)

    async def tearDown(self) -> None:
        if _NOOP_TYPE_ID in registered_types():
            unregister_recordable(_NOOP_TYPE_ID)
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_auto_start_on_play_true_triggers_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="timeline_ctrl_test_") as tmp_dir:
            rec = _make_open_recorder(tmp_dir)
            try:
                ctrl = TimelineDrivenEpisodeController(rec, auto_start_on_play=True)
                self.assertTrue(ctrl.auto_start_on_play)
                self.assertFalse(rec.is_recording)
                ctrl._on_started(None)
                self.assertTrue(rec.is_recording)
                ctrl._on_stopped(None)
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()

    async def test_auto_start_on_play_false_skips_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="timeline_ctrl_test_") as tmp_dir:
            rec = _make_open_recorder(tmp_dir)
            try:
                ctrl = TimelineDrivenEpisodeController(rec, auto_start_on_play=False)
                self.assertFalse(ctrl.auto_start_on_play)
                ctrl._on_started(None)
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()

    async def test_set_auto_start_on_play_toggles_gating_live(self) -> None:
        with tempfile.TemporaryDirectory(prefix="timeline_ctrl_test_") as tmp_dir:
            rec = _make_open_recorder(tmp_dir)
            try:
                ctrl = TimelineDrivenEpisodeController(rec, auto_start_on_play=False)
                ctrl._on_started(None)
                self.assertFalse(rec.is_recording)

                ctrl.set_auto_start_on_play(True)
                ctrl._on_started(None)
                self.assertTrue(rec.is_recording)

                ctrl.set_auto_start_on_play(False)
                ctrl._on_stopped(None)
                self.assertFalse(rec.is_recording)
                ctrl._on_started(None)
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()

    async def test_stop_always_ends_active_episode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="timeline_ctrl_test_") as tmp_dir:
            rec = _make_open_recorder(tmp_dir)
            try:
                ctrl = TimelineDrivenEpisodeController(rec, auto_start_on_play=False)
                rec.start_episode()
                self.assertTrue(rec.is_recording)
                ctrl._on_stopped(None)
                self.assertFalse(rec.is_recording)
            finally:
                rec.close_session()
