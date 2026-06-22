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

"""Headless control of a MobilityGen build-and-record session."""

from __future__ import annotations

import datetime
import os
import shutil
from typing import TYPE_CHECKING

from .config import Config
from .sensor_overrides import save_sensor_overrides
from .writer import MobilityGenWriter

if TYPE_CHECKING:
    from .occupancy_map import OccupancyMap
    from .robot import MobilityGenRobot
    from .scenario import MobilityGenScenario


class RecordingSession:
    """Build a scenario, drive it step by step, and record its state to disk.

    This holds the parts of a recording run that do not depend on the user
    interface: spawning the robot and scenario, advancing the simulation, and
    writing each step to a recording directory. A user interface or a script
    owns the stage and the application loop and calls into this object.

    Call order:
        1. Open the stage and advance the application once (caller's job).
        2. ``build(...)`` to set up the simulation, robot, and scenario.
        3. ``initialize()`` once the application is playing.
        4. ``reset()`` to place the robot at its start pose.
        5. ``step(dt)`` every physics step.
        6. ``enable_recording()`` / ``disable_recording()`` around the part to save.

    An empty session starts with no built scenario and recording disabled.
    """

    def __init__(self) -> None:
        # Build state
        self.scenario: MobilityGenScenario | None = None
        self.robot: MobilityGenRobot | None = None
        self.occupancy_map: OccupancyMap | None = None
        self.config: Config | None = None
        self.cached_stage_path: str | None = None
        self.recordings_dir: str | None = None
        self.chase_camera_path: str | None = None

        # Recording state
        self.writer: MobilityGenWriter | None = None
        self.step_count: int = 0
        self.recording_time: float = 0.0
        self.recording_enabled: bool = False
        self.recording_path: str | None = None

    def build(
        self,
        robot_type: type,
        scenario_type: type,
        occupancy_map: OccupancyMap,
        *,
        scene_usd: str = "",
        cached_stage_path: str | None = None,
        recordings_dir: str | None = None,
        add_ground_plane: bool = True,
        build_chase_camera: bool = False,
    ) -> MobilityGenScenario:
        """Set up the simulation, spawn the robot, and create the scenario.

        The stage must already be open and the application advanced once before
        this is called; the caller owns that step because it differs between a
        user interface (async) and a script (sync).

        Args:
            robot_type: The robot class to spawn.
            scenario_type: The scenario class to create.
            occupancy_map: The occupancy map the scenario uses.
            scene_usd: Path to the input scene, stored in the recording config.
            cached_stage_path: Path to the copied stage to save with a recording.
            recordings_dir: Directory new recordings are created under.
            add_ground_plane: Whether to add a hidden physics ground plane.
            build_chase_camera: Whether to attach the robot's chase camera.

        Returns:
            The created scenario.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        self.occupancy_map = occupancy_map
        self.cached_stage_path = cached_stage_path
        self.recordings_dir = recordings_dir
        self.config = Config(
            scenario_type=scenario_type.__name__,
            robot_type=robot_type.__name__,
            scene_usd=scene_usd,
        )

        SimulationManager.setup_simulation(dt=robot_type.physics_dt)
        if add_ground_plane:
            self._add_ground_plane()
        self.robot = robot_type.build("/World/robot")
        if build_chase_camera:
            self.chase_camera_path = self.robot.build_chase_camera()
        self.scenario = scenario_type.from_robot_occupancy_map(self.robot, occupancy_map)
        return self.scenario

    def initialize(self) -> None:
        """Initialize physics for the built scenario."""
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.initialize_physics()

    def reset(self) -> None:
        """Return the scenario to its start state.

        Closes any open recording, resets the scenario, and starts a fresh
        recording if recording is enabled.
        """
        if self.writer is not None:
            self.writer.close()
        self.writer = None
        self.scenario.reset()
        if self.recording_enabled:
            self._start_new_recording()

    def step(self, step_size: float) -> bool:
        """Advance the scenario by one step and record it if recording is on.

        Args:
            step_size: The physics step size in seconds.

        Returns:
            True while the scenario is still active, False once it has ended.
            On ending it resets itself.
        """
        is_alive = self.scenario.step(step_size)

        if not is_alive:
            self.reset()

        if self.writer is not None:
            state_dict = self.scenario.state_dict_common()
            self.writer.write_state_dict_common(state_dict, step=self.step_count)
            self.step_count += 1
            self.recording_time += step_size

        return is_alive

    def enable_recording(self) -> None:
        """Start saving steps to a new recording directory."""
        if not self.recording_enabled:
            self._start_new_recording()
            self.recording_enabled = True

    def disable_recording(self) -> None:
        """Stop saving steps and close the current recording."""
        self.recording_enabled = False
        self.clear_recording()

    def _start_new_recording(self) -> None:
        """Create a timestamped recording directory and write its setup files."""
        if self.recordings_dir is None:
            raise RuntimeError("Cannot start recording: `recordings_dir` was not set in `build()`.")
        if self.cached_stage_path is None:
            raise RuntimeError("Cannot start recording: `cached_stage_path` was not set in `build()`.")
        recording_name = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        recording_path = os.path.join(self.recordings_dir, recording_name)
        writer = MobilityGenWriter(recording_path)
        writer.write_config(self.config)
        writer.write_occupancy_map(self.scenario.occupancy_map)
        writer.copy_stage(self.cached_stage_path)
        save_sensor_overrides(self.scenario.robot.prim_path, recording_path)
        self.step_count = 0
        self.recording_time = 0.0
        self.writer = writer
        self.recording_path = recording_path

    def clear_recording(self) -> None:
        """Close the current recording writer."""
        if self.writer is not None:
            self.writer.close()
        self.writer = None

    def clear(self) -> None:
        """Drop the built scenario, stop recording, and delete the cached stage copy."""
        self.clear_recording()
        self.recording_enabled = False
        if self.cached_stage_path is not None:
            shutil.rmtree(os.path.dirname(self.cached_stage_path), ignore_errors=True)
        self.scenario = None
        self.robot = None
        self.cached_stage_path = None

    def close(self) -> None:
        """Close the recording writer if one is open."""
        if self.writer is not None:
            self.writer.close()
        self.writer = None

    def _add_ground_plane(self) -> None:
        """Add a physics-only ground plane, hidden to avoid z-fighting with the scene floor."""
        from isaacsim.core.experimental.objects import GroundPlane
        from isaacsim.core.experimental.utils.stage import get_current_stage
        from pxr import UsdGeom

        ground_plane = GroundPlane("/World/ground_plane", templates=None)
        stage = get_current_stage()
        for mesh_path in ground_plane.meshes.paths:
            UsdGeom.Imageable(stage.GetPrimAtPath(mesh_path)).MakeInvisible()
