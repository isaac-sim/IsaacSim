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

"""USD stage, pose, timestamp, and image helpers for the nurec example.

Operates on an already-open stage handed in by the caller.
"""

from __future__ import annotations

import bisect
import os
from typing import Any

import numpy as np
from PIL import Image

USDZ_RIG_PARENT = "/World/rig_trajectories/sensor_rig_0"


def is_remote_path(path: str) -> bool:
    """Return True when `path` is a URL (e.g. `omniverse://`), not a local filesystem path.

    Args:
        path: The path to classify.

    Returns:
        True when `path` is a non-`file://` URL.
    """
    return "://" in path and not path.lower().startswith("file://")


def save_image(path: str, image: np.ndarray) -> None:
    """Write `image` to `path`, creating parent dirs as needed.

    Args:
        path: Destination image path.
        image: The image array to write.
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    Image.fromarray(image).save(path)


def open_stage(stage_path: str) -> Any:
    """Open `stage_path` in the USD context and return the Stage.

    Lazily imports `omni.usd` so the rest of this module stays importable before a
    `SimulationApp` exists; call this only after boot.

    Args:
        stage_path: Path to the USD stage to open.

    Returns:
        The opened `Usd.Stage`, or None on failure.
    """
    import carb
    import omni.usd

    carb.log_info(f"[nurec] loading stage: {stage_path}")
    ctx = omni.usd.get_context()
    if not ctx.open_stage(stage_path):
        carb.log_error(f"[nurec] failed to open stage: {stage_path}")
        return None
    return ctx.get_stage()


def stage_time_params(stage: Any) -> tuple[float, int]:
    """Return the stage's time mapping as `(time_codes_per_second, absolute_offset_us)`.

    `absoluteTimeOffsetMicroSec` (rosbag epoch us) is stored on the root layer's
    customLayerData; together with tcps it maps USD time codes to wall timestamps.

    Args:
        stage: The open USD stage.

    Returns:
        Tuple of (time_codes_per_second, absolute_time_offset_us).
    """
    tcps = float(stage.GetTimeCodesPerSecond())
    offset_us = int(stage.GetRootLayer().customLayerData.get("absoluteTimeOffsetMicroSec", 0))
    return tcps, offset_us


def frame_timestamp_us(tc: float, tcps: float, offset_us: int) -> int:
    """Return the absolute recording timestamp (us) of a USD time code.

    Computes `offset_us + (tc / tcps) * 1e6` (rounded), or `offset_us` when `tcps <= 0`.

    Args:
        tc: The USD time code.
        tcps: Time codes per second.
        offset_us: Absolute time offset in microseconds.

    Returns:
        The absolute timestamp in microseconds.
    """
    return int(round(offset_us + (float(tc) / tcps) * 1e6)) if tcps > 0 else int(offset_us)


def time_code_for_timestamp(ts_ns: int, tcps: float, offset_us: int) -> float:
    """Map an absolute nanosecond timestamp to a USD time code (inverse of `frame_timestamp_us`).

    Args:
        ts_ns: Absolute timestamp in nanoseconds.
        tcps: Time codes per second.
        offset_us: Absolute time offset in microseconds.

    Returns:
        The USD time code, or 0.0 when `tcps <= 0`.
    """
    return (ts_ns / 1000.0 - offset_us) * tcps / 1e6 if tcps > 0 else 0.0


def camera_world_pose(stage: Any, camera_path: str, time_code: float) -> list[float] | None:
    """Return the world pose of `camera_path` at `time_code` in TUM order.

    Lazily imports `pxr`. Evaluates the camera's time-sampled rig transform at `time_code`
    (the same pose `render_at_keyframe` renders) and decomposes it into translation +
    quaternion.

    Args:
        stage: The open USD stage.
        camera_path: Prim path of the camera.
        time_code: The USD time code to evaluate at.

    Returns:
        The pose as [tx, ty, tz, qx, qy, qz, qw], or None when the camera prim can't be
        resolved.
    """
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(camera_path)
    if not prim.IsValid():
        return None
    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode(time_code))
    t = xf.ExtractTranslation()
    q = xf.ExtractRotationQuat()  # Gf.Quatd: real (w) + imaginary (x, y, z)
    im = q.GetImaginary()
    return [float(t[0]), float(t[1]), float(t[2]), float(im[0]), float(im[1]), float(im[2]), float(q.GetReal())]


def camera_world_position(stage: Any, camera_path: str, time_code: float) -> list[float] | None:
    """Return the world-space translation (x, y, z) of `camera_path` at `time_code`.

    Args:
        stage: The open USD stage.
        camera_path: Prim path of the camera.
        time_code: The USD time code to evaluate at.

    Returns:
        The world position as [x, y, z] (the translation of `camera_world_pose`), or None
        when the camera prim can't be resolved.
    """
    pose = camera_world_pose(stage, camera_path, time_code)
    return pose[:3] if pose else None


def frame_poses(stage: Any, camera_path: str, time_codes: list[float], tcps: float, offset_us: int) -> list[dict]:
    """Return per-frame `{time_code, ts_us, ts_ns, position}` for `camera_path`.

    `position` is None when the camera prim can't be resolved.

    Args:
        stage: The open USD stage.
        camera_path: Prim path of the camera.
        time_codes: The USD time codes to evaluate.
        tcps: Time codes per second.
        offset_us: Absolute time offset in microseconds.

    Returns:
        One dict per time code with "time_code", "ts_us", "ts_ns", and "position".
    """
    out: list[dict] = []
    for tc in time_codes:
        ts_us = frame_timestamp_us(tc, tcps, offset_us)
        out.append(
            {
                "time_code": tc,
                "ts_us": ts_us,
                "ts_ns": ts_us * 1000,
                "position": camera_world_position(stage, camera_path, tc),
            }
        )
    return out


def rig_keyframe_time_codes(stage: Any) -> list[float]:
    """Return the sensor rig's authored keyframe time codes (one per training frame).

    Per-training-frame transform samples are written on the sensor rig Xform's
    `xformOp:transform.timeSamples` (see `USDZ_RIG_PARENT`). These are the exact time codes
    the model was trained against, so each one has a matching GT image.

    Args:
        stage: The open USD stage.

    Returns:
        Sorted keyframe time codes, or [] when no sensor rig is present.
    """
    rig_prim = stage.GetPrimAtPath(USDZ_RIG_PARENT)
    if not rig_prim.IsValid():
        return []
    xform_attr = rig_prim.GetAttribute("xformOp:transform")
    if not xform_attr:
        return []
    return [float(tc) for tc in xform_attr.GetTimeSamples()]


class KeyframeIndex:
    """Nearest-within-tolerance map from a timestamp (ns) to a sensor-rig keyframe time code.

    Built from the rig's authored keyframe time codes and the stage time mapping. `match`
    returns the time code of the keyframe closest to a requested timestamp, within a small
    tolerance window.

    Args:
        keyframe_ts_ns: Keyframe timestamps in nanoseconds.
        time_codes: The USD time code for each keyframe (parallel to `keyframe_ts_ns`).
    """

    def __init__(self, keyframe_ts_ns: list[int], time_codes: list[float]) -> None:
        pairs = sorted(zip(keyframe_ts_ns, time_codes))
        self._ts = [int(p[0]) for p in pairs]
        self._tc = [float(p[1]) for p in pairs]

    def __len__(self) -> int:
        """Number of keyframes (0 when the stage has no sensor rig).

        Returns:
            The keyframe count.
        """
        return len(self._ts)

    @classmethod
    def from_stage(cls, stage: Any, tcps: float, offset_us: int) -> KeyframeIndex:
        """Build a KeyframeIndex from a stage's sensor-rig keyframes.

        Args:
            stage: The open USD stage.
            tcps: Time codes per second.
            offset_us: Absolute time offset in microseconds.

        Returns:
            A KeyframeIndex over the rig keyframes (empty when no rig is present).
        """
        tcs = rig_keyframe_time_codes(stage)
        ts_ns = [frame_timestamp_us(tc, tcps, offset_us) * 1000 for tc in tcs]
        return cls(ts_ns, tcs)

    def match(self, ts_ns: int, tol_us: float) -> float | None:
        """Return the time code of the keyframe nearest `ts_ns`, within `tol_us`.

        Args:
            ts_ns: The requested timestamp in nanoseconds.
            tol_us: Match tolerance in microseconds.

        Returns:
            The matching keyframe's time code, or None when none is within tolerance.
        """
        if not self._ts:
            return None
        target = int(ts_ns)
        tol_ns = tol_us * 1000.0
        i = bisect.bisect_left(self._ts, target)
        candidates: list[int] = []
        if i > 0:
            candidates.append(i - 1)
        if i < len(self._ts):
            candidates.append(i)
        best = min(candidates, key=lambda k: abs(self._ts[k] - target))
        if abs(self._ts[best] - target) > tol_ns:
            return None
        return self._tc[best]


def pick_evenly_spaced_time_codes(stage: Any, n: int) -> list[float]:
    """Pick `n` evenly-spaced USD time codes between the stage start/end (inclusive).

    Args:
        stage: The open USD stage.
        n: Number of time codes to pick.

    Returns:
        The selected time codes (empty when `n <= 0`).
    """
    if n <= 0:
        return []
    start_tc = float(stage.GetStartTimeCode())
    end_tc = float(stage.GetEndTimeCode())
    if not (end_tc > start_tc):
        return [start_tc] * n
    if n == 1:
        return [0.5 * (start_tc + end_tc)]
    return list(np.linspace(start_tc, end_tc, n).tolist())


def subsample_evenly(seq: list[float], n: int) -> list[float]:
    """Pick `n` evenly-spaced indices out of `seq` (inclusive endpoints).

    Args:
        seq: The sequence to subsample.
        n: Number of items to pick.

    Returns:
        The subsampled items (all of `seq` when `n >= len(seq)`; empty when `n <= 0`).
    """
    if not seq or n <= 0:
        return []
    if n >= len(seq):
        return list(seq)
    if n == 1:
        return [seq[len(seq) // 2]]
    idxs = np.linspace(0, len(seq) - 1, n).round().astype(int)
    return [seq[i] for i in idxs]


def subsample_first(seq: list[float], n: int) -> list[float]:
    """Take the first `n` items of `seq` (the start of the trajectory).

    Args:
        seq: The sequence to subsample.
        n: Number of leading items to take.

    Returns:
        The first `n` items (empty when `n <= 0`).
    """
    if not seq or n <= 0:
        return []
    return list(seq[:n])


def subsample(seq: list[float], n: int | None, mode: str = "even") -> list[float]:
    """Select keyframes from `seq`: all of them when `n` is falsy, else `n` by `mode`.

    Args:
        seq: The sequence to subsample.
        n: Number of items to pick, or a falsy value to keep all.
        mode: "even" (evenly spaced, inclusive endpoints) or "first" (the first `n`).

    Returns:
        The selected items.

    Raises:
        ValueError: If `mode` is not "even" or "first".
    """
    if not n:
        return list(seq)
    if mode == "first":
        return subsample_first(seq, n)
    if mode == "even":
        return subsample_evenly(seq, n)
    raise ValueError(f"unknown sample mode {mode!r}; expected 'even' or 'first'")


def read_timestamps_file(path: str) -> list[int]:
    """Read a per-camera timestamp file (one integer-ns timestamp per non-blank line).

    Args:
        path: Path to the timestamp file.

    Returns:
        The parsed timestamps in nanoseconds, in file order.
    """
    with open(os.path.expanduser(path)) as f:
        return [int(line.strip()) for line in f if line.strip()]


def write_tum(path: str, entries: list[tuple[int, list[float]]]) -> None:
    """Write a TUM trajectory file (one `timestamp tx ty tz qx qy qz qw` line per entry).

    The TUM timestamp column is in seconds; the input timestamps are nanoseconds.

    Args:
        path: Destination file path (parent dirs are created).
        entries: List of (ts_ns, pose) where pose is [tx, ty, tz, qx, qy, qz, qw].
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        for ts_ns, pose in entries:
            f.write(f"{ts_ns / 1e9:.9f} " + " ".join(f"{v:.9f}" for v in pose) + "\n")


def read_tum(path: str) -> list[tuple[int, list[float]]]:
    """Read a TUM trajectory file into (ts_ns, pose) entries.

    Lines are `timestamp tx ty tz qx qy qz qw` (timestamp in seconds); blank and `#`
    comment lines are skipped. The seconds timestamp is converted back to nanoseconds.

    Args:
        path: Path to the TUM file.

    Returns:
        List of (ts_ns, [tx, ty, tz, qx, qy, qz, qw]).
    """
    out: list[tuple[int, list[float]]] = []
    with open(os.path.expanduser(path)) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            out.append((int(round(float(parts[0]) * 1e9)), [float(x) for x in parts[1:8]]))
    return out
