# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Handle synthetic data recording with state management and asynchronous recording operations."""

from enum import Enum

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import Semantics
from pxr import UsdGeom, UsdSemantics, UsdSkel

# If both width and height are larger than this value, warn the user
MAX_RESOLUTION_WARN = 8000

# Large number to use as a default value when the recorder should run indefinitely
MAX_NUM_FRAMES = 100000000

# Annotators that require semantically labeled prims
SEMANTICS_ANNOTATORS = (
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
    "bounding_box_3d",
    "occlusion",
)


# Possible states of the recorder
class RecorderState(Enum):
    """Enumeration representing the possible states of the synthetic data recorder.

    This enum defines the three operational states that a SyntheticRecorder instance can be in
    during its lifecycle. The states control the recording process flow and determine what
    operations are available at any given time.
    """

    STOPPED = 0
    """Recorder is not active and not capturing data."""
    RUNNING = 1
    """Recorder is actively capturing and processing frames."""
    PAUSED = 2
    """Recorder is temporarily suspended but can be resumed."""


class SyntheticRecorder:
    """Synthetic Data Recorder class handling the recording process."""

    def __init__(self) -> None:
        # Recording configuration
        self.num_frames = 0
        self.rt_subframes = 0
        self.control_timeline = False
        self.verbose = False

        # Writer and render products configuration
        self.writer_name = "BasicWriter"
        self.writer_params = {}  # Parameters to pass to writer.initialize()
        self.rp_data = []  # Render product data: [(camera_path, width, height, name), ...]

        # Backend configuration
        self.backend_type = None  # Options: None, "DiskBackend", "S3Backend", etc.
        self.backend_params = {}  # Backend-specific parameters

        # Internal
        self._state = RecorderState.STOPPED
        self._state_subscribers = []
        self._writer = None
        self._current_frame = 0
        self._render_products = []
        self._original_capture_on_play = None

    def get_state(self) -> RecorderState:
        """Current state of the recorder.

        Returns:
            The current recorder state (STOPPED, RUNNING, or PAUSED).
        """
        return self._state

    def subscribe_state_changed(self, callback: object) -> None:
        """Subscribe to the recorder state changes.

        Args:
            callback: Function to call when the recorder state changes.
        """
        self._state_subscribers.append(callback)

    def _notify_state_subscribers(self) -> None:
        """Notify the subscribers about the state change."""
        for callback in self._state_subscribers:
            callback()

    def _set_state(self, state: RecorderState) -> None:
        """Set the state of the recorder and notify any subscribers about the change.

        Args:
            state: The new recorder state to set.
        """
        self._state = state
        self._notify_state_subscribers()

    def init_recorder(self) -> bool:
        """Initialize the recorder (create a new one) and attach the writer to the render products (newly created).

        Returns:
            True if initialization was successful, False otherwise.
        """
        if self._writer is None:
            try:
                self._writer = rep.WriterRegistry.get(self.writer_name)
            except Exception as e:
                print(f"[SDR] Could not create writer {self.writer_name}: {e}")
                return False

        # Store the original capture on play setting and disable it if necessary
        self._original_capture_on_play = carb.settings.get_settings().get("/omni/replicator/captureOnPlay")
        if self._original_capture_on_play:
            rep.orchestrator.set_capture_on_play(False)
            print("[SDR] Disabling replicator capture on play flag during recording.")

        # Create and initialize the backend
        if not self.backend_type:
            print("[SDR] Backend not configured. Set backend_type before recording.")
            return False

        try:
            backend = rep.backends.get(self.backend_type)
            backend.initialize(**self.backend_params)
            if self.verbose:
                print(f"[SDR][Backend] Using {self.backend_type} with params: {self.backend_params}")
        except Exception as e:
            print(f"[SDR] Could not initialize {self.backend_type}: {e}")
            return False

        # Add backend to writer_params
        writer_params = {"backend": backend}
        writer_params.update(self.writer_params)

        # For BasicWriter: perform stage validation and disable incompatible annotators
        if self.writer_name == "BasicWriter":
            # If the stage is not semantically labeled, disable any semantics related annotators
            stage_is_labeled = self._check_if_stage_is_semantically_labeled()
            if not stage_is_labeled:
                print(
                    "[SDR] Stage is not semantically labeled, semantics related annotators will not work, removing them."
                )
                self._disable_semantics_annotators(writer_params)

            # If the stage does not have any skeleton prims, disable the skeleton_data annotator
            if writer_params.get("skeleton_data", False) and not self._check_if_stage_has_skeleton_prims():
                print("[SDR] Stage does not have any skeleton prims, disabling skeleton annotator.")
                writer_params["skeleton_data"] = False

        try:
            self._writer.initialize(**writer_params)
        except Exception as e:
            print(f"[SDR] Could not initialize writer {self.writer_name}: {e}")
            return False

        for rp_entry in self.rp_data:
            if not self._check_if_valid_rp_entry(rp_entry):
                print(f"[SDR] Invalid render product entry {rp_entry}.")
                continue
            camera_path = rp_entry[0]
            resolution = (rp_entry[1], rp_entry[2])
            custom_name = rp_entry[3] if rp_entry[3] else None
            rp = rep.create.render_product(camera_path, resolution, name=custom_name, force_new=True)
            self._render_products.append(rp)

        if not self._render_products:
            print("[SDR] No valid render products found to initialize the writer.")
            return False

        try:
            self._writer.attach(self._render_products)
        except Exception as e:
            print(f"[SDR] Could not attach render products to writer: {e}")
            return False

        if self.verbose:
            print("[SDR][Recorder] Initialized.")

        return True

    def clear_recorder(self) -> None:
        """Clear the recorder state, detach the writer, and destroy the render products."""
        if self._state != RecorderState.STOPPED:
            self._set_state(RecorderState.STOPPED)
        self._current_frame = 0
        if self._writer:
            self._writer.detach()
            self._writer = None
        for rp in self._render_products:
            rp.destroy()
        self._render_products.clear()

        # Reset the capture on play flag to its original value
        if self._original_capture_on_play is not None:
            rep.orchestrator.set_capture_on_play(self._original_capture_on_play)
            if self._original_capture_on_play:
                print("[SDR] Re-enabled replicator capture on play flag after recording.")
            self._original_capture_on_play = None

    async def start_stop_async(self) -> None:
        """Start or stop the recording loop."""
        timeline = omni.timeline.get_timeline_interface()
        if self._state == RecorderState.STOPPED and self.init_recorder():
            # Start recording if the state is STOPPED and init_recorder() was successful
            if self.verbose:
                print(
                    f"[SDR][Recorder] Start;\tFrame: {self._current_frame};\tTime: {timeline.get_current_time():.4f}."
                )
            self._set_state(RecorderState.RUNNING)
            if self.control_timeline and not timeline.is_playing():
                if self.verbose:
                    print("[SDR][ControlTimeline] Start Recording; Timeline is not playing. Starting it.")
                timeline.play()
                timeline.commit()
            # Start the recording loop with the specified number of frames (or run indefinitely == MAX_NUM_FRAMES)
            num_frames = self.num_frames if self.num_frames > 0 else MAX_NUM_FRAMES
            await self._run_recording_loop_async(num_frames)
        else:
            # Stop the recording if the state is RUNNING or PAUSED
            if self.verbose:
                print(f"[SDR] Stop;\tFrame: {self._current_frame};\tTime: {timeline.get_current_time():.4f}.")
            if self.rt_subframes > 0:
                rep.orchestrator.stop()
            self._set_state(RecorderState.STOPPED)
            await self._finish_recording_async()

    async def pause_resume_async(self) -> None:
        """Pause or resume the recording loop."""
        timeline = omni.timeline.get_timeline_interface()
        if self._state == RecorderState.RUNNING:
            if self.verbose:
                print(
                    f"[SDR][Recorder] Pause;\tFrame: {self._current_frame};\tTime: {timeline.get_current_time():.4f}."
                )
            if self.rt_subframes > 0:
                rep.orchestrator.pause()
            self._set_state(RecorderState.PAUSED)
            if self.control_timeline and timeline.is_playing():
                if self.verbose:
                    print("[SDR][ControlTimeline] Pausing Recording; Timeline is playing. Pausing it.")
                timeline.pause()
                timeline.commit()
        elif self._state == RecorderState.PAUSED:
            if self.verbose:
                print(
                    f"[SDR][Recorder] Resume;\tFrame: {self._current_frame};\tTime: {timeline.get_current_time():.4f}."
                )
            self._set_state(RecorderState.RUNNING)
            if self.control_timeline and not timeline.is_playing():
                if self.verbose:
                    print("[SDR][ControlTimeline] Resuming Recording; Timeline is not playing. Starting it.")
                timeline.play()
                timeline.commit()
            # Resume the recording loop (internal frame counter will continue from the last frame)
            num_frames = self.num_frames if self.num_frames > 0 else MAX_NUM_FRAMES
            await self._run_recording_loop_async(num_frames)
        else:
            print(f"[SDR] Recorder is in an unexpected state ({self._state.name}), try again.")

    def _check_if_valid_camera(self, path: str) -> bool:
        """Check if the camera path is valid for the render product.

        Args:
            path: The USD path to the camera prim.

        Returns:
            True if the camera path is valid, False otherwise.
        """
        context = omni.usd.get_context()
        stage = context.get_stage()
        prim = stage.GetPrimAtPath(path)

        if not prim.IsValid():
            print(f"[SDR] {path} is not a valid prim path.")
            return False

        if UsdGeom.Camera(prim):
            return True
        else:
            print(f"[SDR] {prim.GetPath()} is not a valid 'Camera' type.")
            return False

    def _check_if_valid_resolution(self, width: int, height: int) -> bool:
        """Check if the resolution is valid for the render product.

        Args:
            width: Width of the resolution in pixels.
            height: Height of the resolution in pixels.

        Returns:
            True if the resolution is valid (both width and height > 0), False otherwise.
        """
        if width > 0 and height > 0:
            if width > MAX_RESOLUTION_WARN and height > MAX_RESOLUTION_WARN:
                print(f"[SDR] Using a large resolution {width}x{height} might lead to out of memory issues.")
            return True
        else:
            print(f"[SDR] Invalid resolution: {width}x{height}. Width and height must be larger than 0.")
        return False

    def _check_if_valid_rp_entry(self, entry: object) -> bool:
        """Check if the render product entry is valid.

        Args:
            entry: Render product entry tuple containing (camera_path, width, height, custom_name).

        Returns:
            True if the entry is valid, False otherwise.
        """
        return (
            len(entry) == 4  # (camera path, width, height, custom name="")
            and self._check_if_valid_camera(entry[0])
            and self._check_if_valid_resolution(entry[1], entry[2])
        )

    def _check_if_stage_is_semantically_labeled(self) -> bool:
        """Check if the stage has any semantically labeled prims.

        Returns:
            True if the stage has semantically labeled prims, False otherwise.
        """
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            # Check the new semantics API
            if prim.HasAPI(UsdSemantics.LabelsAPI):
                return True
            # Check the old semantics API
            if prim.HasAPI(Semantics.SemanticsAPI):
                return True
        return False

    def _check_if_stage_has_skeleton_prims(self) -> bool:
        """Check if the stage has any skeleton prims.

        Returns:
            True if the stage has skeleton prims, False otherwise.
        """
        stage = omni.usd.get_context().get_stage()
        return any(prim.IsA(UsdSkel.Skeleton) for prim in stage.Traverse())

    def _disable_semantics_annotators(self, writer_params: dict) -> None:
        """Disable semantics related annotators if the stage does not have semantically labeled prims.

        Args:
            writer_params: Dictionary of writer parameters to modify by disabling semantics annotators.
        """
        # Store the annotators that were disabled due to the stage not having semantically labeled prims
        disabled_annotators = []
        # Iterate over the semantics related annotators and disable them if they are enabled
        for annotator in SEMANTICS_ANNOTATORS:
            # Check if the annotator is in the writer parameters and if it is enabled
            if annotator in writer_params and writer_params[annotator]:
                writer_params[annotator] = False
                disabled_annotators.append(annotator)
        if disabled_annotators:
            print(f"[SDR] Disabled the following semantics related annotators: {disabled_annotators}.")

    async def _run_recording_loop_async(self, num_frames: int) -> None:
        """Run the recording loop for the specified number of frames.

        Args:
            num_frames: Number of frames to record.
        """
        timeline = omni.timeline.get_timeline_interface()
        while self._current_frame < num_frames:
            # Stop the recording loop if the state has been changed from RUNNING to PAUSED or STOPPED
            if self._state != RecorderState.RUNNING:
                break
            # Make sure the timeline is playing if Control Timeline is enabled
            if self.control_timeline and not timeline.is_playing():
                if self.verbose:
                    print("[SDR][ControlTimeline] Recording; Timeline is not playing. Starting it.")
                timeline.play()
                timeline.commit()
            if self.verbose:
                print(f"[SDR][Capture] Frame: {self._current_frame};\tTime: {timeline.get_current_time():.4f};")
            await rep.orchestrator.step_async(rt_subframes=self.rt_subframes, delta_time=None, pause_timeline=False)
            self._current_frame += 1

        # The recording loop has finished change the state to STOPPED
        if self._state == RecorderState.RUNNING:
            self._set_state(RecorderState.STOPPED)
            await self._finish_recording_async()

    async def _finish_recording_async(self) -> None:
        """Finish the recording and wait until the data is complete."""
        timeline = omni.timeline.get_timeline_interface()
        # If the timeline should be controlled by the recorder and it is running, stop it
        if self.control_timeline and timeline.is_playing():
            if self.verbose:
                print("[SDR][ControlTimeline] Finishing Recording; Timeline is playing. Stopping it.")
            timeline.stop()
            timeline.commit()
        await rep.orchestrator.wait_until_complete_async()
        if self.verbose:
            # Print completion message based on backend type
            if self.backend_type == "DiskBackend":
                output_dir = self.backend_params.get("output_dir", "unknown")
                print(f"[SDR][Recorder] Finished;\tData written to: {output_dir}.")
            elif self.backend_type == "S3Backend":
                bucket = self.backend_params.get("bucket", "unknown")
                key_prefix = self.backend_params.get("key_prefix", "")
                print(f"[SDR][Recorder] Finished;\tData written to S3 bucket: {bucket}/{key_prefix}.")
            elif self.backend_type:
                print(f"[SDR][Recorder] Finished;\tData written using {self.backend_type}.")
            else:
                print("[SDR][Recorder] Finished;\tData written.")
        self.clear_recorder()
