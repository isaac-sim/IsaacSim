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

import hashlib
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

_ASSETS_SUBDIR = "assets"


def _is_url_asset_path(asset_path: str) -> bool:
    # A `://` substring distinguishes URL schemes from Windows drive letters (`C:/`).
    return "://" in asset_path


def _norm_path_key(p: str) -> str:
    """Canonicalize an absolute filesystem path for dict-key use across OSes.

    `UsdUtils.ComputeAllDependencies` and `Sdf.Layer.ComputeAbsolutePath` can
    return paths that differ in separator (`/` vs `\\`) or case on Windows.
    Canonicalize to forward slashes and lowercase the path on Windows so dict
    lookups hit. No-op on POSIX, where filesystem paths are case-sensitive.
    """
    p = p.replace("\\", "/")
    if os.name == "nt":
        p = p.lower()
    return p


def _stable_asset_dest(src_abs_path: str, dest_root: str) -> str:
    """Return `<dest_root>/assets/<md5(parent_dir)>/<basename>`.

    Hashing the parent dir prevents collisions between two assets sharing a
    basename but coming from different source directories. The hash is taken
    on the canonical (forward-slash, case-folded on Windows) form so the same
    logical parent always maps to the same hash.
    """
    parent = _norm_path_key(os.path.dirname(os.path.abspath(src_abs_path)))
    digest = hashlib.md5(parent.encode("utf-8")).hexdigest()[:8]
    return os.path.join(dest_root, _ASSETS_SUBDIR, digest, os.path.basename(src_abs_path))


def _copy_url_asset(src_url: str, dest_abs_path: str) -> bool:
    """Try `omni.client.copy(src_url → dest_abs_path)`. Any failure is non-fatal."""
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


def _copy_stage_with_dependencies(input_path: str, dest_root: str) -> None:
    """Copy a flattened `.usd` stage and its asset dependencies into `dest_root`.

    Walks `UsdUtils.ComputeAllDependencies`, copies each resolved asset into
    `dest_root/assets/<hash>/<basename>`, then rewrites asset paths via
    `UsdUtils.ModifyAssetPaths` on an anonymous copy of the cached layer and
    saves the rewritten layer to `dest_root/stage.usd`.

    The rewrite runs on an anonymous copy because the layer returned by
    `ComputeAllDependencies` is shared via USD's global `Sdf.Layer` registry —
    mutating it in place would leak rewrites into other consumers (e.g. a live
    simulation stage opened from the same on-disk file).

    URL-scheme assets (e.g. `omniverse://...`) are pulled down via
    `omni.client.copy`; on failure the URL is kept and a warning is logged.
    UDIM-templated paths are kept as `<UDIM>` in the rewritten layer with each
    tile copied alongside. Unresolved paths are warned about and left as-is.
    MDL and other asset-typed attributes are walked by `ModifyAssetPaths`.
    """
    # Lazy import: writer.py must remain importable without USD bindings.
    from pxr import Sdf, UsdUtils

    layers, resolved_assets, unresolved_paths = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(input_path))

    # Normalized-resolved-asset-path -> local destination (or None when an URL copy failed).
    # Keys are canonicalized via `_norm_path_key` so lookups in `_rewrite` (which
    # resolves authored paths through `Sdf.Layer.ComputeAbsolutePath`) match the
    # entries built from `ComputeAllDependencies`.
    asset_dest_map: dict[str, str | None] = {}
    # Track originals separately so the copy step uses the un-normalized path.
    local_copies: list[tuple[str, str]] = []
    for asset in resolved_assets:
        dest = _stable_asset_dest(asset, dest_root)
        key = _norm_path_key(asset)
        if _is_url_asset_path(asset):
            if _copy_url_asset(asset, dest):
                asset_dest_map[key] = dest
            else:
                asset_dest_map[key] = None
                carb.log_warn(
                    f"MobilityGen: could not copy remote asset `{asset}` into recording; "
                    "keeping the URL in `stage.usd`."
                )
            continue
        asset_dest_map[key] = dest
        local_copies.append((asset, dest))

    for missing in unresolved_paths:
        carb.log_warn(f"MobilityGen: could not resolve asset dependency `{missing}`; keeping original reference.")

    for src, dest in local_copies:
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        if not os.path.exists(dest):
            shutil.copy2(src, dest)

    dest_stage_path = os.path.join(dest_root, "stage.usd")
    dest_stage_dir = os.path.dirname(dest_stage_path)

    if not layers:
        carb.log_warn(
            f"MobilityGen: `ComputeAllDependencies` returned no layers for `{input_path}`; "
            "falling back to a plain copy. External asset references will not be rewritten."
        )
        shutil.copyfile(input_path, dest_stage_path)
        return

    source_layer = layers[0]

    layer_text = source_layer.ExportToString()
    rewrite_layer = Sdf.Layer.CreateAnonymous(".usda")
    if not rewrite_layer.ImportFromString(layer_text):
        raise RuntimeError(f"MobilityGen: failed to import source layer into anonymous copy for `{input_path}`")

    # Normalized parent dirs (on the source side) of every concretely-copied
    # local tile. Used to gate UDIM template rewrites so we only emit a
    # templated local ref when `ComputeAllDependencies` actually expanded the
    # family. Without this, an unresolvable UDIM (no tiles on disk) would
    # silently get rewritten to a nonexistent local path, replacing the
    # authored reference that USD's resolver could otherwise still expand.
    copied_parent_dirs: set[str] = {_norm_path_key(os.path.dirname(os.path.abspath(src))) for src, _ in local_copies}

    def _rewrite(asset_path: str) -> str:
        if not asset_path:
            return asset_path
        if _is_url_asset_path(asset_path):
            # URL UDIM families (e.g. `omniverse://.../tile.<UDIM>.png`) are not
            # locally consolidated: the keys in asset_dest_map are concrete tile
            # URLs, so a templated authored URL misses here and the authored URL
            # is preserved. The remote tiles remain reachable while online.
            dest = asset_dest_map.get(_norm_path_key(asset_path))
        else:
            # Resolve against the source layer — the anonymous copy has no on-disk
            # location, so its `ComputeAbsolutePath` would be an identity.
            resolved = source_layer.ComputeAbsolutePath(asset_path)
            if "<UDIM>" in resolved:
                # ComputeAllDependencies expands the template into individual
                # tiles (already in asset_dest_map); the templated destination
                # shares the same hash dir because `_stable_asset_dest` keys on
                # parent dir, which is identical for every tile in a family.
                # Only rewrite if at least one tile from this family actually
                # landed locally — otherwise keep the authored path so the
                # original reference (and any future resolver expansion) is
                # preserved instead of pointing at an empty local hash dir.
                template_parent = _norm_path_key(os.path.dirname(os.path.abspath(resolved)))
                dest = _stable_asset_dest(resolved, dest_root) if template_parent in copied_parent_dirs else None
            else:
                dest = asset_dest_map.get(_norm_path_key(resolved))
        if dest is None:
            return asset_path
        return os.path.relpath(dest, dest_stage_dir).replace(os.sep, "/")

    UsdUtils.ModifyAssetPaths(rewrite_layer, _rewrite)

    if not rewrite_layer.Export(dest_stage_path):
        raise RuntimeError(f"MobilityGen: failed to export rewritten stage to `{dest_stage_path}`")


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
        """Copy the stage to the recording directory as a self-contained tree.

        For `.usd` inputs, walks `UsdUtils.ComputeAllDependencies`, copies every
        reachable asset into a sibling ``assets/`` subdir, and rewrites the
        layer's asset paths to point at the copies. `.usdz` archives are
        already self-contained and are copied byte-for-byte.
        """
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if input_path.endswith(".usdz"):
            shutil.copyfile(input_path, os.path.join(self.path, "stage.usdz"))
        else:
            _copy_stage_with_dependencies(input_path, self.path)

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
        """Copy stage, config, occupancy map, and the sibling assets/ tree from
        another recording. `.usdz` recordings are archive-self-contained and
        don't need an assets/ tree."""
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if os.path.exists(os.path.join(other_path, "stage.usdz")):
            shutil.copyfile(os.path.join(other_path, "stage.usdz"), os.path.join(self.path, "stage.usdz"))
        else:
            shutil.copyfile(os.path.join(other_path, "stage.usd"), os.path.join(self.path, "stage.usd"))
            src_assets = os.path.join(other_path, _ASSETS_SUBDIR)
            if os.path.isdir(src_assets):
                shutil.copytree(src_assets, os.path.join(self.path, _ASSETS_SUBDIR), dirs_exist_ok=True)
        shutil.copyfile(os.path.join(other_path, "config.json"), os.path.join(self.path, "config.json"))
        shutil.copytree(os.path.join(other_path, "occupancy_map"), os.path.join(self.path, "occupancy_map"))
