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

"""Writer for saving MobilityGen recorded data to disk."""

import io
import os
import queue
import shutil
import threading

import carb
import numpy as np
import PIL.Image

from .config import Config
from .occupancy_map import OccupancyMap

_SENTINEL = object()


def _is_url_asset_path(asset_path: str) -> bool:
    # A `://` substring distinguishes URL schemes from Windows drive letters (`C:/`).
    return "://" in asset_path


def _copy_url_asset(src_url: str, dest_abs_path: str) -> bool:
    """Try `omni.client.copy(src_url -> dest_abs_path)`. Any failure is non-fatal.

    Args:
        src_url: Source asset URL.
        dest_abs_path: Destination absolute filesystem path.

    Returns:
        True if the asset exists at the destination after the copy attempt.
    """
    # Lazy import: writer.py is also exercised under non-Kit Python contexts.
    try:
        import omni.client
    except ImportError:
        return False

    os.makedirs(os.path.dirname(dest_abs_path), exist_ok=True)
    if os.path.exists(dest_abs_path):
        return True
    try:
        dst_url = omni.client.make_file_url_if_possible(dest_abs_path)
        result = omni.client.copy(src_url, dst_url, omni.client.CopyBehavior.OVERWRITE)
    except Exception as exc:  # noqa: BLE001
        carb.log_warn(f"MobilityGen: exception copying remote asset `{src_url}`: {exc}")
        return False
    if result != omni.client.Result.OK:
        return False
    return os.path.exists(dest_abs_path)


def _copy_local_or_url(src: str, dest: str) -> bool:
    """Copy `src` -> `dest`, handling both local files and `omniverse://` URLs.

    Creates the destination directory as needed and skips the copy if `dest`
    already exists. Returns True on success.

    Args:
        src: Source local path or URL.
        dest: Destination local path.

    Returns:
        True if the destination exists or the copy succeeds.
    """
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    if os.path.exists(dest):
        return True
    if _is_url_asset_path(src):
        return _copy_url_asset(src, dest)
    if not os.path.exists(src):
        return False
    shutil.copy2(src, dest)
    return True


def _url_to_local_path(url: str) -> str:
    """Return a local filesystem path for ``url``.

    A plain path is returned unchanged; a URL (such as ``file://`` or
    ``omniverse://``) is converted to its local path.

    Args:
        url: Local path or URL to convert.

    Returns:
        Local filesystem path when it can be resolved, otherwise ``url``.
    """
    if "://" not in url:
        return url
    try:
        import omni.client

        return omni.client.break_url(url).path
    except Exception:  # noqa: BLE001
        return url


async def collect_input(input_path: str, dest_dir: str) -> str:
    """Copy a scene into ``dest_dir`` as a self-contained copy.

    The copy can be opened from ``dest_dir``, or moved elsewhere, and still find
    all of its files.

    A ``.usdz`` scene is copied whole. A ``.usd``, ``.usda``, or ``.usdc`` scene
    copies the files it references — provide a ``.usdz`` if the scene also needs
    files it does not reference.

    Args:
        input_path: Source USD/USDZ scene path or URL.
        dest_dir: Destination directory for the self-contained copy.

    Returns:
        The path to the copied scene in ``dest_dir``.
    """
    os.makedirs(dest_dir, exist_ok=True)

    if input_path.lower().endswith(".usdz"):
        dest_stage_path = os.path.join(dest_dir, "stage.usdz")
        if not _copy_local_or_url(input_path, dest_stage_path):
            raise RuntimeError(f"MobilityGen: failed to copy USDZ input `{input_path}`")
        return dest_stage_path

    try:
        from omni.kit.usd.collect import Collector
    except ImportError as exc:
        raise RuntimeError(
            "MobilityGen: `omni.kit.usd.collect` is unavailable; cannot cache a `.usd` scene. "
            "It is declared as a dependency of `isaacsim.replicator.experimental.mobility_gen`; "
            "ensure that extension is enabled."
        ) from exc

    # A flat layout would split each `.mdl` from its `./Textures` and break in-MDL paths.
    collector = Collector(input_path, dest_dir, flat_collection=False)
    success, root_usd_url = await collector.collect()
    if not success or not root_usd_url:
        raise RuntimeError(f"MobilityGen: `omni.kit.usd.collect` failed to collect `{input_path}`")

    # Collect names the root after the source file; MobilityGen opens `stage.usd`.
    collected_root = _url_to_local_path(root_usd_url)
    dest_stage_path = os.path.join(dest_dir, "stage.usd")
    if os.path.abspath(collected_root) != os.path.abspath(dest_stage_path):
        os.replace(collected_root, dest_stage_path)
    return dest_stage_path


class MobilityGenWriter:
    """Writer for saving MobilityGen recordings to a directory.

    Args:
        path: The output directory path for the recording.
        async_write: If True (default), common state dicts are serialized to an
            in-memory buffer on the hot path and flushed to disk by a background
            thread.  This removes ~0.7 ms of blocking disk I/O from the physics
            callback.  Call ``flush()`` or ``close()`` before shutting down to
            ensure all writes complete.
        max_pending: Maximum number of serialized buffers that may be queued
            before the hot path blocks (backpressure).  Default 8.
    """

    def __init__(self, path: str, async_write: bool = True, max_pending: int = 8) -> None:
        self.path = path
        self._async_write = async_write
        if async_write:
            self._write_queue: queue.Queue = queue.Queue(maxsize=max_pending)
            self._writer_thread = threading.Thread(target=self._writer_worker, daemon=True, name="MobilityGenWriter")
            self._writer_thread.start()

    def _writer_worker(self) -> None:
        while True:
            item = self._write_queue.get()
            if item is _SENTINEL:
                self._write_queue.task_done()
                return
            path, buf = item
            try:
                with open(path, "wb") as f:
                    f.write(buf.getvalue())
            finally:
                self._write_queue.task_done()

    def flush(self) -> None:
        """Block until all pending async writes have been flushed to disk."""
        if self._async_write:
            self._write_queue.join()

    def close(self) -> None:
        """Flush all pending writes and shut down the background writer thread."""
        if self._async_write:
            self._write_queue.put(_SENTINEL)
            self._writer_thread.join()
            self._async_write = False

    def __del__(self) -> None:
        """Close the writer when the object is collected."""
        try:
            self.close()
        except Exception:
            pass

    def write_state_dict_common(self, state_dict: dict, step: int) -> None:
        """Write the common (non-image) state dictionary to disk.

        Args:
            state_dict: The state dictionary to save.
            step: The current step index used as the filename.
        """
        dict_folder = os.path.join(self.path, "state", "common")
        if not os.path.exists(dict_folder):
            os.makedirs(dict_folder)
        state_dict_path = os.path.join(dict_folder, f"{step:08d}.npz")
        arrays = {k: v for k, v in state_dict.items() if v is not None}
        if self._async_write:
            buf = io.BytesIO()
            np.savez(buf, **arrays)
            self._write_queue.put((state_dict_path, buf))
        else:
            np.savez(state_dict_path, **arrays)

    def write_state_dict_rgb(self, state_rgb: dict, step: int) -> None:
        """Write RGB image frames to disk.

        Args:
            state_rgb: A dict mapping camera name to RGB numpy array.
            step: The current step index used as the filename.
        """
        for name, value in state_rgb.items():
            if value is not None:
                image_folder = os.path.join(self.path, "state", "rgb", name)
                if not os.path.exists(image_folder):
                    os.makedirs(image_folder)
                image_path = os.path.join(image_folder, f"{step:08d}.jpg")
                image = PIL.Image.fromarray(value)
                image.save(image_path)

    def write_state_dict_segmentation(self, state_segmentation: dict, step: int) -> None:
        """Write segmentation image frames to disk.

        Args:
            state_segmentation: A dict mapping camera name to segmentation numpy array.
            step: The current step index used as the filename.
        """
        for name, value in state_segmentation.items():
            if value is not None:
                image_folder = os.path.join(self.path, "state", "segmentation", name)
                if not os.path.exists(image_folder):
                    os.makedirs(image_folder)
                image_path = os.path.join(image_folder, f"{step:08d}.png")
                image = PIL.Image.fromarray(value)
                image.save(image_path)

    def write_state_dict_depth(self, state_np: dict, step: int) -> None:
        """Write depth images to disk as 16-bit inverse depth PNGs.

        Args:
            state_np: A dict mapping camera name to depth numpy array.
            step: The current step index used as the filename.
        """
        for name, value in state_np.items():
            if value is not None:
                output_folder = os.path.join(self.path, "state", "depth", name)
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)

                # Inverse depth 16bit
                inverse_depth = 1.0 / (1.0 + value)
                inverse_depth = (65535 * inverse_depth).astype(np.uint16)
                image = PIL.Image.fromarray(inverse_depth, "I;16")

                output_path = os.path.join(output_folder, f"{step:08d}.png")

                image.save(output_path)

    def write_state_dict_normals(self, state_np: dict, step: int) -> None:
        """Write surface normals frames to disk as .npy files.

        Args:
            state_np: A dict mapping camera name to normals numpy array.
            step: The current step index used as the filename.
        """
        for name, value in state_np.items():
            if value is not None:
                output_folder = os.path.join(self.path, "state", "normals", name)
                if not os.path.exists(output_folder):
                    os.makedirs(output_folder)
                output_path = os.path.join(output_folder, f"{step:08d}.npy")
                np.save(output_path, value)

    def copy_stage(self, input_path: str) -> None:
        """Copy the stage and the files it needs into the recording directory.

        `input_path` is a stage produced by `collect_input`. Everything in its
        folder is copied into the recording directory so the references still
        resolve.

        Args:
            input_path: Path to the cached stage file to copy from.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        cache_dir = os.path.dirname(input_path)
        shutil.copytree(cache_dir, self.path, dirs_exist_ok=True)

    def write_config(self, config: Config) -> None:
        """Write the scenario configuration to disk as JSON.

        Args:
            config: The Config object to serialize and save.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        with open(os.path.join(self.path, "config.json"), "w") as f:
            f.write(config.to_json())

    def write_occupancy_map(self, occupancy_map: OccupancyMap) -> None:
        """Write the occupancy map to disk in ROS format.

        Args:
            occupancy_map: The OccupancyMap to save.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        occupancy_map.save_ros(os.path.join(self.path, "occupancy_map"))

    def copy_init(self, other_path: str) -> None:
        """Copy the setup files from another recording into this one.

        Copies the stage and the files it needs, the config, and the occupancy
        map. The recorded per-step ``state`` folder is not copied.

        Args:
            other_path: Recording directory to copy setup files from.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        for entry in os.listdir(other_path):
            if entry == "state":
                continue  # per-step recorded outputs are re-rendered, not copied
            src = os.path.join(other_path, entry)
            dst = os.path.join(self.path, entry)
            if os.path.isdir(src):
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copyfile(src, dst)
