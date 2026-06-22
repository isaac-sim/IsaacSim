# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for RecordingSession: build a scenario, step it, and record to disk."""

import os
import shutil
import tempfile

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

# Import examples so CarterRobot / RandomAccelerationScenario register themselves.
import isaacsim.replicator.mobility_gen.examples  # noqa: F401
import numpy as np
import omni.kit.test
import omni.usd
from isaacsim.replicator.experimental.mobility_gen import (
    ROBOTS,
    SCENARIOS,
    MobilityGenReader,
    OccupancyMap,
    RecordingSession,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Usd


class TestRecordingSession(omni.kit.test.AsyncTestCase):
    """Drive a build-and-record run on a clean scene through RecordingSession."""

    async def setUp(self) -> None:
        """Create an empty stage and a temp working directory."""
        # Ensure the assets root resolves so the robot USD can be referenced.
        await get_assets_root_path_async()
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        self._tmp_dir = tempfile.mkdtemp(prefix="mobility_gen_session_test_")
        self._session = RecordingSession()

    async def tearDown(self) -> None:
        """Stop simulation, wait for stage cleanup, and remove temp files."""
        self._session.close()
        app_utils.stop()
        await app_utils.update_app_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()
        shutil.rmtree(self._tmp_dir, ignore_errors=True)

    def _make_freespace_occupancy_map(self) -> OccupancyMap:
        """Build a trivial 20 m x 20 m all-freespace occupancy map centered at the origin.

        Returns:
            All-freespace occupancy map centered at the origin.
        """
        height = width = 400
        freespace = np.ones((height, width), dtype=bool)
        occupied = np.zeros((height, width), dtype=bool)
        return OccupancyMap.from_masks(freespace, occupied, 0.05, (-10.0, -10.0, 0.0))

    def _make_cached_stage(self) -> str:
        """Write a minimal stage file the recording's copy_stage can copy.

        Returns:
            Path to the cached stage file.
        """
        cache_dir = os.path.join(self._tmp_dir, "cache")
        os.makedirs(cache_dir, exist_ok=True)
        cached_stage_path = os.path.join(cache_dir, "stage.usd")
        stage = Usd.Stage.CreateNew(cached_stage_path)
        stage.GetRootLayer().Save()
        del stage
        return cached_stage_path

    async def test_build_record_step_produces_recording(self) -> None:
        """Build via RecordingSession, record a few steps, and verify the recording is readable."""
        robot_type = ROBOTS.get("CarterRobot")
        scenario_type = SCENARIOS.get("RandomAccelerationScenario")
        occupancy_map = self._make_freespace_occupancy_map()
        cached_stage_path = self._make_cached_stage()
        recordings_dir = os.path.join(self._tmp_dir, "recordings")

        # Build (stage already open + app warmed once in setUp — the caller's job).
        scenario = self._session.build(
            robot_type,
            scenario_type,
            occupancy_map,
            scene_usd="",
            cached_stage_path=cached_stage_path,
            recordings_dir=recordings_dir,
            add_ground_plane=True,
            build_chase_camera=False,
        )
        self.assertIsNotNone(scenario)
        self.assertIs(self._session.scenario, scenario)
        self.assertIsNotNone(self._session.robot)

        app_utils.play()
        await app_utils.update_app_async()
        self._session.initialize()
        self._session.reset()

        self._session.enable_recording()
        self.assertTrue(self._session.recording_enabled)
        recording_path = self._session.recording_path
        self.assertIsNotNone(recording_path)

        # A short run keeps the robot in bounds so it records one continuous episode.
        num_steps = 20
        dt = robot_type.physics_dt
        for _ in range(num_steps):
            await app_utils.update_app_async()
            self._session.step(dt)

        self.assertGreater(self._session.step_count, 0)
        self._session.disable_recording()
        self.assertFalse(self._session.recording_enabled)

        # The recording directory is well-formed and readable.
        self.assertTrue(os.path.exists(os.path.join(recording_path, "config.json")))
        self.assertTrue(os.path.exists(os.path.join(recording_path, "stage.usd")))
        reader = MobilityGenReader(recording_path)
        self.assertGreater(len(reader), 0)

        state = reader.read_state_dict_common(0)
        self.assertIn("robot.position", state)
        self.assertIn("robot.orientation", state)
        self.assertEqual(np.asarray(state["robot.position"]).shape, (3,))
        self.assertEqual(np.asarray(state["robot.orientation"]).shape, (4,))

    async def test_enable_recording_without_recordings_dir_raises(self) -> None:
        """enable_recording must fail with a clear error if build() got no recordings_dir."""
        robot_type = ROBOTS.get("CarterRobot")
        scenario_type = SCENARIOS.get("RandomAccelerationScenario")
        occupancy_map = self._make_freespace_occupancy_map()

        # recordings_dir defaults to None.
        self._session.build(
            robot_type,
            scenario_type,
            occupancy_map,
            add_ground_plane=False,
            build_chase_camera=False,
        )
        with self.assertRaises(RuntimeError):
            self._session.enable_recording()

    async def test_clear_closes_writer_and_resets_recording_state(self) -> None:
        """clear() must close the writer and reset recording_enabled, not just the build state."""

        class _FakeWriter:
            def __init__(self) -> None:
                self.closed = False

            def close(self) -> None:
                self.closed = True

        writer = _FakeWriter()
        self._session.writer = writer
        self._session.recording_enabled = True
        self._session.scenario = object()
        self._session.robot = object()

        self._session.clear()

        self.assertTrue(writer.closed)
        self.assertIsNone(self._session.writer)
        self.assertFalse(self._session.recording_enabled)
        self.assertIsNone(self._session.scenario)
        self.assertIsNone(self._session.robot)

    async def test_build_without_ground_plane_skips_ground_plane(self) -> None:
        """build(add_ground_plane=False) must not create the ground plane prim."""
        robot_type = ROBOTS.get("CarterRobot")
        scenario_type = SCENARIOS.get("RandomAccelerationScenario")
        occupancy_map = self._make_freespace_occupancy_map()

        self._session.build(
            robot_type,
            scenario_type,
            occupancy_map,
            recordings_dir=os.path.join(self._tmp_dir, "recordings"),
            add_ground_plane=False,
            build_chase_camera=False,
        )

        stage = omni.usd.get_context().get_stage()
        self.assertFalse(stage.GetPrimAtPath("/World/ground_plane").IsValid())
        self.assertTrue(stage.GetPrimAtPath("/World/robot").IsValid())
