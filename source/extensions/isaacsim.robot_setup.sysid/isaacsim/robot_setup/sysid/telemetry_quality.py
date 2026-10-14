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

"""Telemetry quality diagnostics for SysID run readiness and reports."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from .trajectory_csv import TrajectoryDataset
from .trajectory_segments import (
    TELEMETRY_CHUNK_ROLE_TRAIN,
    TELEMETRY_CHUNK_ROLE_VALIDATION,
    TelemetryChunkRunSpec,
    TrajectorySegmentError,
    build_trajectory_chunks,
)

WEAK_EXCITATION_RANGE_THRESHOLD = 1e-4
WEAK_EXCITATION_VELOCITY_THRESHOLD = 1e-4

#: Minimum normalized cross-correlation peak for a per-joint lag estimate to count.
COMMAND_LAG_MIN_CORRELATION = 0.6
#: Common-mode lag magnitude (in samples) above which a correction is suggested.
COMMAND_LAG_WARN_SAMPLES = 1.5
#: Default search range for the command-to-response lag.
COMMAND_LAG_MAX_SECONDS = 1.0
_COMMAND_LAG_MIN_CORE_SAMPLES = 16
_COMMAND_LAG_MAX_WINDOW_SAMPLES = 65536


@dataclass
class CommandLagJointEstimate:
    """Command-to-response lag of one joint, from cross-correlation."""

    joint_index: int
    lag_samples: float | None
    lag_seconds: float | None
    correlation_peak: float | None
    #: Lag remaining after removing the common-mode (median) lag. This part is a
    #: per-joint actuator latency — an input to actuator-model identification,
    #: not to the common-mode command shift.
    residual_samples: float | None = None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this joint estimate.

        Returns:
            All lag-estimate fields as a plain dictionary.
        """
        return asdict(self)


@dataclass
class CommandLagReport:
    """Command-to-response delay decomposition for loaded telemetry.

    Real telemetry can carry delay from two distinct sources: a COMMON-MODE
    transport/generation delay (trajectory-generator smoothing/lookahead — a
    data artifact, removed by ``telemetry.command_alignment_seconds``) and small
    PER-JOINT actuator latencies (a robot property, identified later by an
    actuator model with the plant frozen). The median over joints estimates the
    common mode; the per-joint residuals after removing the median estimate the
    actuator part and are reported only.
    """

    dt_seconds: float
    max_lag_samples: int
    joints: list[CommandLagJointEstimate]
    common_mode_lag_samples: float | None
    common_mode_lag_seconds: float | None
    #: Largest per-joint |lag - median| among valid estimates, in samples.
    residual_spread_samples: float | None
    #: Value for ``telemetry.command_alignment_seconds`` that removes the common mode.
    suggested_command_alignment_seconds: float | None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this lag report.

        Returns:
            Lag decomposition and per-joint estimates as a plain dictionary.
        """
        return asdict(self)


def estimate_command_lag(
    trajectory: TrajectoryDataset,
    *,
    max_lag_seconds: float = COMMAND_LAG_MAX_SECONDS,
) -> CommandLagReport | None:
    """Estimate per-joint command-to-response lag and its common-mode component.

    Per joint, the mean-removed time-gradient of the command series is
    cross-correlated against the measured velocities over the excited window,
    with parabolic sub-sample refinement of the peak. Positive lag means the
    measured response lags the commands. Returns ``None`` when the telemetry is
    too short or has no usable command channel.

    Args:
        trajectory: Loaded command and measured-velocity telemetry.
        max_lag_seconds: Largest positive or negative lag to search.

    Returns:
        Lag decomposition, or ``None`` when the telemetry cannot support an estimate.
    """
    try:
        max_lag_seconds = float(max_lag_seconds)
    except (TypeError, ValueError) as exc:
        raise ValueError("max_lag_seconds must be a finite positive number.") from exc
    if not np.isfinite(max_lag_seconds) or max_lag_seconds <= 0.0:
        raise ValueError("max_lag_seconds must be a finite positive number.")

    times = np.asarray(trajectory.times, dtype=np.float64).reshape(-1)
    commands = np.asarray(trajectory.commands, dtype=np.float64)
    velocities = np.asarray(trajectory.velocities, dtype=np.float64)
    if (
        times.shape[0] < _COMMAND_LAG_MIN_CORE_SAMPLES + 2
        or commands.ndim != 2
        or commands.shape != velocities.shape
        or commands.shape[0] != times.shape[0]
        or not np.all(np.isfinite(times))
    ):
        return None
    dt_values = np.diff(times)
    dt = float(np.median(dt_values))
    if not np.isfinite(dt) or dt <= 0.0:
        return None
    max_lag = max(1, int(round(max_lag_seconds / dt)))

    joints = [
        _estimate_joint_lag(times, commands[:, idx], velocities[:, idx], dt, max_lag, idx)
        for idx in range(commands.shape[1])
    ]
    valid = [estimate for estimate in joints if estimate.lag_samples is not None]
    common_samples: float | None = None
    common_seconds: float | None = None
    spread: float | None = None
    suggested: float | None = None
    if valid:
        common_samples = float(np.median([estimate.lag_samples for estimate in valid]))
        common_seconds = common_samples * dt
        for estimate in valid:
            estimate.residual_samples = float(estimate.lag_samples - common_samples)
        spread = float(max(abs(estimate.residual_samples) for estimate in valid))
        suggested = common_seconds
    return CommandLagReport(
        dt_seconds=dt,
        max_lag_samples=max_lag,
        joints=joints,
        common_mode_lag_samples=common_samples,
        common_mode_lag_seconds=common_seconds,
        residual_spread_samples=spread,
        suggested_command_alignment_seconds=suggested,
    )


def _estimate_joint_lag(
    times: np.ndarray,
    command: np.ndarray,
    velocity: np.ndarray,
    dt: float,
    max_lag: int,
    joint_index: int,
) -> CommandLagJointEstimate:
    """Cross-correlate d(command)/dt against measured velocity over the excited window.

    Args:
        times: Sample timestamps for the joint signals.
        command: Command series for one joint.
        velocity: Measured velocity series for the same joint.
        dt: Nominal sampling interval in seconds.
        max_lag: Maximum lag to search, in samples.
        joint_index: Joint index recorded in the result.

    Returns:
        Per-joint lag estimate, with unavailable fields set to ``None``.
    """
    no_estimate = CommandLagJointEstimate(joint_index, None, None, None)
    if not (np.all(np.isfinite(command)) and np.all(np.isfinite(velocity))):
        return no_estimate
    peak_velocity = float(np.max(np.abs(velocity))) if velocity.size else 0.0
    active = np.flatnonzero(np.abs(velocity) > max(0.05 * peak_velocity, WEAK_EXCITATION_VELOCITY_THRESHOLD))
    if active.size == 0:
        return no_estimate
    start = int(active[0])
    stop = min(int(active[-1]) + 1, start + _COMMAND_LAG_MAX_WINDOW_SAMPLES)
    x = np.gradient(command[start:stop], times[start:stop])
    y = velocity[start:stop]
    n = int(x.shape[0])
    lag_cap = min(max_lag, (n - _COMMAND_LAG_MIN_CORE_SAMPLES) // 2)
    if lag_cap < 1:
        return no_estimate

    core = y[lag_cap : n - lag_cap]
    core = core - np.mean(core)
    core_norm = float(np.linalg.norm(core))
    if core_norm <= 0.0:
        return no_estimate
    lags = np.arange(-lag_cap, lag_cap + 1)
    corr = _normalized_lag_correlation(x, core, core_norm)
    peak = int(np.argmax(corr))
    peak_value = float(corr[peak])
    if peak_value < COMMAND_LAG_MIN_CORRELATION:
        return CommandLagJointEstimate(joint_index, None, None, peak_value)
    offset = 0.0
    if 0 < peak < corr.shape[0] - 1:
        denom = corr[peak - 1] - 2.0 * corr[peak] + corr[peak + 1]
        if denom < 0.0:
            offset = float(np.clip(0.5 * (corr[peak - 1] - corr[peak + 1]) / denom, -0.5, 0.5))
    lag_samples = float(lags[peak]) + offset
    return CommandLagJointEstimate(joint_index, lag_samples, lag_samples * dt, peak_value)


def _normalized_lag_correlation(x: np.ndarray, core: np.ndarray, core_norm: float) -> np.ndarray:
    """Correlate ``core`` against every window of ``x`` it fits in, in one pass.

    Scoring lags one at a time costs O(len(x) * lag count) and issues two small
    BLAS calls per lag, which dominates telemetry quality on high-rate recordings.
    Two properties collapse it to a single transform:

    * ``core`` is already mean-removed, so the per-window mean subtraction
      reduces to the window mean times the (rounding-level) sum of ``core``. The
      numerator is then a plain cross-correlation, which one FFT pair evaluates
      for every lag at once, minus that term.
    * Every window's mean and centered energy come from
      :func:`_window_means_and_energies` in O(n log m).

    ``x`` is shifted by its median first. Nothing here changes under a constant
    shift, but it keeps the FFT's rounding, which scales with the magnitude of
    ``x``, small when the command has a steady rate.

    Results are indexed by ascending lag, matching ``arange(-lag_cap, lag_cap + 1)``.

    Args:
        x: Command-derivative series covering the excited window.
        core: Mean-removed measured-velocity core the lags are scored against.
        core_norm: Euclidean norm of ``core``, which the caller has verified is positive.

    Returns:
        Normalized correlation for each candidate lag.
    """
    x = x - np.median(x)
    n = int(x.shape[0])
    m = int(core.shape[0])
    # Linear (not circular) correlation needs room for n + m - 1 points.
    size = 1 << (n + m - 2).bit_length()
    spectrum = np.fft.rfft(x, size) * np.conj(np.fft.rfft(core, size))
    numerator = np.fft.irfft(spectrum, size)[: n - m + 1]

    window_mean, window_energy = _window_means_and_energies(x, m)
    # A window far from zero would amplify the residual sum of ``core``.
    numerator -= window_mean * float(np.sum(core))
    window_norm = np.sqrt(window_energy)

    # Window index and lag run in opposite directions: lag L reads x at lag_cap - L.
    numerator = numerator[::-1]
    window_norm = window_norm[::-1]
    corr = np.zeros(numerator.shape[0], dtype=np.float64)
    usable = window_norm > 0.0
    corr[usable] = numerator[usable] / (core_norm * window_norm[usable])
    return corr


def _window_means_and_energies(values: np.ndarray, width: int) -> tuple[np.ndarray, np.ndarray]:
    """Mean and centered energy of every contiguous ``width``-sample window, in O(n log width).

    Sliding sums of ``x`` and ``x**2`` give the energy as the difference of two
    nearly equal totals whenever a window's mean is large next to its spread, and
    cancellation then destroys it. Instead, the statistics of every span of 1, 2,
    4, ... samples are built by merging adjacent halves, and each window is
    assembled from the spans matching the set bits of ``width``. A merge only adds
    non-negative terms, so no window loses its energy to cancellation, and only
    samples inside a window contribute to it.

    Args:
        values: Series to summarize.
        width: Window length, at least one and at most ``len(values)``.

    Returns:
        ``len(values) - width + 1`` window means and centered energies, ordered by window start.
    """
    count = int(values.shape[0]) - width + 1
    # span_mean[i] and span_energy[i] describe values[i : i + span].
    span_mean = np.asarray(values, dtype=np.float64)
    span_energy = np.zeros_like(span_mean)
    span = 1
    covered = 0
    window_mean = np.empty(0, dtype=np.float64)
    window_energy = np.empty(0, dtype=np.float64)
    while True:
        if width & span:
            piece_mean = span_mean[covered : covered + count]
            piece_energy = span_energy[covered : covered + count]
            if covered == 0:
                window_mean, window_energy = piece_mean.copy(), piece_energy.copy()
            else:
                window_mean, window_energy = _merge_adjacent(
                    window_mean, window_energy, covered, piece_mean, piece_energy, span
                )
            covered += span
        if 2 * span > width:
            return window_mean, window_energy
        span_mean, span_energy = _merge_adjacent(
            span_mean[:-span], span_energy[:-span], span, span_mean[span:], span_energy[span:], span
        )
        span *= 2


def _merge_adjacent(
    left_mean: np.ndarray,
    left_energy: np.ndarray,
    left_count: int,
    right_mean: np.ndarray,
    right_energy: np.ndarray,
    right_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Combine the statistics of adjacent sample runs with the pairwise mean/variance update.

    Args:
        left_mean: Mean of each left run.
        left_energy: Centered energy of each left run.
        left_count: Samples in every left run.
        right_mean: Mean of each right run, which directly follows its left run.
        right_energy: Centered energy of each right run.
        right_count: Samples in every right run.

    Returns:
        Mean and centered energy of each combined run.
    """
    total = left_count + right_count
    delta = right_mean - left_mean
    mean = left_mean + delta * (right_count / total)
    energy = left_energy + right_energy + delta * delta * (left_count * right_count / total)
    return mean, energy


@dataclass(frozen=True)
class TelemetryQualityIssue:
    """One telemetry quality diagnostic."""

    code: str
    message: str
    severity: str = "warning"

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this quality issue.

        Returns:
            Issue code, message, and severity as a plain dictionary.
        """
        return asdict(self)


@dataclass
class TelemetryQualityReport:
    """Telemetry diagnostics attached to exported SysID artifacts."""

    sample_count: int
    num_joints: int
    duration_seconds: float
    nominal_sample_rate_hz: float
    dt_min: float | None
    dt_median: float | None
    dt_max: float | None
    timestamp_jitter_ratio: float | None
    channels: dict[str, bool]
    command_matches_position: bool
    finite: dict[str, bool]
    excitation_range_by_joint: list[float]
    excitation_velocity_rms_by_joint: list[float]
    weak_excitation_joint_indices: list[int]
    chunk_coverage_fraction: float
    train_chunk_count: int
    validation_chunk_count: int
    command_lag: CommandLagReport | None = None
    issues: list[TelemetryQualityIssue] = field(default_factory=list)

    @property
    def warning_count(self) -> int:
        """Return the number of non-error quality issues."""
        return sum(1 for issue in self.issues if issue.severity != "error")

    @property
    def error_count(self) -> int:
        """Return the number of error quality issues."""
        return sum(1 for issue in self.issues if issue.severity == "error")

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready representation of this quality report.

        Returns:
            Report fields, serialized issues, and severity counts.
        """
        payload = asdict(self)
        payload["issues"] = [issue.to_dict() for issue in self.issues]
        payload["warning_count"] = self.warning_count
        payload["error_count"] = self.error_count
        return payload

    def compact_summary(self) -> str:
        """Return a concise human-readable quality summary.

        Returns:
            One-line sample, channel, chunk-coverage, and warning summary.
        """
        channel_names = [name for name, enabled in self.channels.items() if enabled]
        rate = f"{self.nominal_sample_rate_hz:.2f} Hz" if self.nominal_sample_rate_hz > 0.0 else "unknown rate"
        coverage = f"{100.0 * self.chunk_coverage_fraction:.0f}%"
        warning_text = f"{self.warning_count} warning(s)" if self.warning_count else "no warnings"
        return (
            f"{self.sample_count} samples, {self.num_joints} joints, {rate}; "
            f"channels={', '.join(channel_names)}; chunks cover {coverage}; {warning_text}"
        )


def build_telemetry_quality_report(
    trajectory: TrajectoryDataset,
    chunks: Iterable[TelemetryChunkRunSpec] | None = None,
) -> TelemetryQualityReport:
    """Build a warn-first quality report for loaded telemetry.

    Args:
        trajectory: Loaded trajectory to assess.
        chunks: Optional train and validation chunk specifications.

    Returns:
        Quality metrics and warn-first diagnostics for the trajectory.
    """
    issues: list[TelemetryQualityIssue] = []
    times = np.asarray(trajectory.times, dtype=np.float64).reshape(-1)
    positions = np.asarray(trajectory.positions, dtype=np.float64)
    velocities = np.asarray(trajectory.velocities, dtype=np.float64)
    commands = np.asarray(trajectory.commands, dtype=np.float64)
    sample_count = int(times.shape[0])
    num_joints = int(trajectory.num_joints)
    duration = float(times[-1] - times[0]) if sample_count >= 2 else 0.0
    dt = np.diff(times) if sample_count >= 2 else np.asarray([], dtype=np.float64)
    finite = {
        "time": bool(np.all(np.isfinite(times))),
        "position": bool(np.all(np.isfinite(positions))),
        "velocity": bool(np.all(np.isfinite(velocities))),
        "command": bool(np.all(np.isfinite(commands))),
        "torque": _optional_finite(trajectory.torques),
        "end_effector_pose": _optional_finite(trajectory.end_effector_poses),
        "contact_force": _optional_finite(trajectory.contact_forces),
    }
    for name, ok in finite.items():
        if not ok:
            issues.append(
                TelemetryQualityIssue(
                    f"nonfinite_{name}",
                    f"Telemetry {name} channel contains NaN or Inf.",
                    severity="error",
                )
            )

    dt_min = float(np.min(dt)) if dt.size else None
    dt_median = float(np.median(dt)) if dt.size else None
    dt_max = float(np.max(dt)) if dt.size else None
    jitter = _timestamp_jitter_ratio(dt)
    if dt.size and np.any(dt <= 0.0):
        issues.append(
            TelemetryQualityIssue(
                "timestamp_order",
                "Telemetry timestamps are not strictly increasing.",
                severity="error",
            )
        )
    if jitter is not None and jitter > 0.1:
        issues.append(
            TelemetryQualityIssue(
                "timestamp_jitter",
                f"Telemetry timestamp jitter is high ({jitter:.3f} of median dt).",
            )
        )

    if sample_count < 2:
        issues.append(TelemetryQualityIssue("sample_count", "Telemetry has fewer than two samples."))

    position_range = _joint_range(positions)
    velocity_rms = _joint_rms(velocities)
    weak = [
        idx
        for idx, (q_range, dq_rms) in enumerate(zip(position_range, velocity_rms))
        if q_range < WEAK_EXCITATION_RANGE_THRESHOLD and dq_rms < WEAK_EXCITATION_VELOCITY_THRESHOLD
    ]
    if weak:
        issues.append(
            TelemetryQualityIssue(
                "weak_excitation",
                "Low position/velocity excitation for joint(s): " + ", ".join(str(idx) for idx in weak),
            )
        )

    command_matches_position = _same_shape_allclose(commands, positions)
    if command_matches_position:
        issues.append(
            TelemetryQualityIssue(
                "command_matches_position",
                "Command channel matches measured positions; command telemetry may be missing or inferred.",
            )
        )

    command_lag = None if command_matches_position else estimate_command_lag(trajectory)
    if command_lag is not None and command_lag.common_mode_lag_samples is not None:
        common = command_lag.common_mode_lag_samples
        if abs(common) > COMMAND_LAG_WARN_SAMPLES:
            issues.append(
                TelemetryQualityIssue(
                    "command_lag_common_mode",
                    (
                        f"Measured response lags commands by a common-mode {common:.1f} samples "
                        f"(~{1000.0 * command_lag.common_mode_lag_seconds:.0f} ms) uniformly across joints — "
                        "typically trajectory-generator smoothing/lookahead in the recording, not a robot "
                        f"property. Set telemetry.command_alignment_seconds = "
                        f"{command_lag.suggested_command_alignment_seconds:.4f} to realign the command series at "
                        "load; uncorrected, the solve mis-attributes the delay to drive gains, damping, and "
                        "friction. The per-joint residual spread after removing the common mode "
                        f"({command_lag.residual_spread_samples:.2f} samples) is actuator latency for "
                        "actuator-model identification, not part of this correction."
                    ),
                )
            )

    train_count, validation_count, coverage = _chunk_stats(trajectory, chunks, issues)
    if validation_count < 1:
        issues.append(TelemetryQualityIssue("no_validation_holdout", "No validation holdout chunk is configured."))

    channels = {
        "position": True,
        "velocity": True,
        "command": True,
        "torque": trajectory.torques is not None,
        "end_effector_pose": trajectory.end_effector_poses is not None,
        "contact_force": trajectory.contact_forces is not None,
    }
    nominal_rate = float((sample_count - 1) / duration) if sample_count >= 2 and duration > 0.0 else 0.0
    return TelemetryQualityReport(
        sample_count=sample_count,
        num_joints=num_joints,
        duration_seconds=duration,
        nominal_sample_rate_hz=nominal_rate,
        dt_min=dt_min,
        dt_median=dt_median,
        dt_max=dt_max,
        timestamp_jitter_ratio=jitter,
        channels=channels,
        command_matches_position=command_matches_position,
        finite=finite,
        excitation_range_by_joint=position_range,
        excitation_velocity_rms_by_joint=velocity_rms,
        weak_excitation_joint_indices=weak,
        chunk_coverage_fraction=coverage,
        train_chunk_count=train_count,
        validation_chunk_count=validation_count,
        command_lag=command_lag,
        issues=issues,
    )


def _optional_finite(array: np.ndarray | None) -> bool:
    return True if array is None else bool(np.all(np.isfinite(np.asarray(array, dtype=np.float64))))


def _timestamp_jitter_ratio(dt: np.ndarray) -> float | None:
    if dt.size < 2 or not np.all(np.isfinite(dt)):
        return None
    median = float(np.median(dt))
    if median <= 0.0:
        return None
    return float(np.std(dt) / median)


def _joint_range(values: np.ndarray) -> list[float]:
    if values.ndim != 2 or values.shape[1] < 1:
        return []
    with np.errstate(invalid="ignore"):
        ranges = np.nanmax(values, axis=0) - np.nanmin(values, axis=0)
    return [float(value) if np.isfinite(value) else 0.0 for value in ranges]


def _joint_rms(values: np.ndarray) -> list[float]:
    if values.ndim != 2 or values.shape[1] < 1:
        return []
    with np.errstate(invalid="ignore"):
        rms = np.sqrt(np.nanmean(values * values, axis=0))
    return [float(value) if np.isfinite(value) else 0.0 for value in rms]


def _same_shape_allclose(left: np.ndarray, right: np.ndarray) -> bool:
    return left.shape == right.shape and bool(np.allclose(left, right, rtol=1e-6, atol=1e-9, equal_nan=False))


def _chunk_stats(
    trajectory: TrajectoryDataset,
    chunks: Iterable[TelemetryChunkRunSpec] | None,
    issues: list[TelemetryQualityIssue],
) -> tuple[int, int, float]:
    specs = list(chunks or [])
    if not specs:
        return 1, 0, 1.0
    train_count = sum(1 for chunk in specs if str(chunk.role).lower() == TELEMETRY_CHUNK_ROLE_TRAIN)
    validation_count = sum(1 for chunk in specs if str(chunk.role).lower() == TELEMETRY_CHUNK_ROLE_VALIDATION)
    try:
        built = build_trajectory_chunks(trajectory, specs, require_train=False, reject_overlaps=True)
    except TrajectorySegmentError as exc:
        issues.append(TelemetryQualityIssue("chunk_validation", str(exc)))
        return train_count, validation_count, 0.0
    duration = max(float(trajectory.duration), 1e-12)
    covered = sum(max(0.0, float(chunk.duration_seconds)) for chunk in built)
    return train_count, validation_count, float(max(0.0, min(1.0, covered / duration)))
