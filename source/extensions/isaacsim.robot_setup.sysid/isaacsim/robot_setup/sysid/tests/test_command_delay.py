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

# ruff: noqa: D102

"""Unit tests for common-mode command-lag estimation and load-time correction."""

from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import omni.kit.test
import torch
from isaacsim.robot_setup.sysid.ingest import load_trajectory
from isaacsim.robot_setup.sysid.ingest.trajectory_builder import apply_command_delay
from isaacsim.robot_setup.sysid.newton_sysid_bridge import (
    _commands_for_actuator_owners,
    _delayed_command_rows,
    actuator_command_delay_seconds_vector,
)
from isaacsim.robot_setup.sysid.parameter_types import (
    SysIdParameterEntry,
    SysIdParameterType,
)
from isaacsim.robot_setup.sysid.run_spec import SysIdRunSpec, TelemetryRunSpec
from isaacsim.robot_setup.sysid.schema_validation import validate_sysid_run_spec_payload
from isaacsim.robot_setup.sysid.telemetry_quality import (
    _COMMAND_LAG_MAX_WINDOW_SAMPLES,
    _COMMAND_LAG_MIN_CORE_SAMPLES,
    COMMAND_LAG_MIN_CORRELATION,
    WEAK_EXCITATION_VELOCITY_THRESHOLD,
    _window_means_and_energies,
    build_telemetry_quality_report,
    estimate_command_lag,
)
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset

_RATE_HZ = 30.0
_DT = 1.0 / _RATE_HZ
_NUM_JOINTS = 7
#: Common-mode lag (samples) matching the transport delay measured on real Franka
#: telemetry (~16 samples at 30 Hz), plus per-joint actuator offsets with median 0.
_COMMON_LAG_SAMPLES = 16.0
_JOINT_OFFSETS = (-0.6, -0.3, -0.1, 0.0, 0.2, 0.4, 0.7)


def _multisine(t: np.ndarray, joint: int) -> tuple[np.ndarray, np.ndarray]:
    """Analytic excitation position and velocity for one joint.

    Args:
        t: Value supplied for ``t``.
        joint: Value supplied for ``joint``.

    Returns:
        Result produced by the operation.
    """
    position = np.zeros_like(t)
    velocity = np.zeros_like(t)
    for k, (freq, amp) in enumerate(((0.31, 0.6), (0.83, 0.3), (1.57, 0.15))):
        w = 2.0 * np.pi * freq * (1.0 + 0.05 * joint)
        phase = 0.9 * joint + 0.7 * k
        position += amp * np.sin(w * t + phase)
        velocity += amp * w * np.cos(w * t + phase)
    return position, velocity


def _delayed_trajectory(
    lag_samples_by_joint: list[float],
    duration_seconds: float = 40.0,
    rate_hz: float = _RATE_HZ,
) -> TrajectoryDataset:
    """Commands are the multisine; measurements respond ``lag`` samples later.

    Args:
        lag_samples_by_joint: Value supplied for ``lag_samples_by_joint``.
        duration_seconds: Value supplied for ``duration_seconds``.
        rate_hz: Telemetry sample rate, which sets how many candidate lags the
            one-second search window spans.

    Returns:
        Result produced by the operation.
    """
    dt = 1.0 / rate_hz
    times = np.arange(int(round(duration_seconds * rate_hz))) * dt
    commands = np.zeros((times.shape[0], len(lag_samples_by_joint)))
    positions = np.zeros_like(commands)
    velocities = np.zeros_like(commands)
    for joint, lag in enumerate(lag_samples_by_joint):
        commands[:, joint], _ = _multisine(times, joint)
        positions[:, joint], velocities[:, joint] = _multisine(times - lag * dt, joint)
    return TrajectoryDataset(times=times, positions=positions, velocities=velocities, commands=commands)


def _reference_joint_lag(
    times: np.ndarray,
    command: np.ndarray,
    velocity: np.ndarray,
    max_lag: int,
) -> tuple[float | None, float | None]:
    """Score every candidate lag one at a time, the definition of the estimator.

    This is deliberately the slow, obvious formulation: for each lag, slide the
    command-derivative window, remove its mean, and take a normalized dot product
    against the fixed core. It exists only so the production implementation has
    something independent to be checked against.

    Args:
        times: Sample timestamps for the joint signals.
        command: Command series for one joint.
        velocity: Measured velocity series for the same joint.
        max_lag: Maximum lag to search, in samples.

    Returns:
        Sub-sample refined lag and correlation peak, with the lag ``None`` when
        the peak falls below the acceptance threshold.
    """
    peak_velocity = float(np.max(np.abs(velocity)))
    active = np.flatnonzero(np.abs(velocity) > max(0.05 * peak_velocity, WEAK_EXCITATION_VELOCITY_THRESHOLD))
    start = int(active[0])
    stop = min(int(active[-1]) + 1, start + _COMMAND_LAG_MAX_WINDOW_SAMPLES)
    x = np.gradient(command[start:stop], times[start:stop])
    y = velocity[start:stop]
    n = int(x.shape[0])
    lag_cap = min(max_lag, (n - _COMMAND_LAG_MIN_CORE_SAMPLES) // 2)

    core = y[lag_cap : n - lag_cap]
    core = core - np.mean(core)
    core_norm = float(np.linalg.norm(core))
    lags = np.arange(-lag_cap, lag_cap + 1)
    corr = np.zeros(lags.shape[0], dtype=np.float64)
    for index, lag in enumerate(lags):
        window = x[lag_cap - lag : n - lag_cap - lag]
        window = window - np.mean(window)
        window_norm = float(np.linalg.norm(window))
        if window_norm > 0.0:
            corr[index] = float(np.dot(core, window)) / (core_norm * window_norm)

    peak = int(np.argmax(corr))
    peak_value = float(corr[peak])
    if peak_value < COMMAND_LAG_MIN_CORRELATION:
        return None, peak_value
    offset = 0.0
    if 0 < peak < corr.shape[0] - 1:
        denom = corr[peak - 1] - 2.0 * corr[peak] + corr[peak + 1]
        if denom < 0.0:
            offset = float(np.clip(0.5 * (corr[peak - 1] - corr[peak + 1]) / denom, -0.5, 0.5))
    return float(lags[peak]) + offset, peak_value


class CommandDelayTests(omni.kit.test.AsyncTestCase):
    """Tests for command-lag estimation, the spec key, and load-time correction."""

    async def test_lag_report_recovers_common_mode_and_residuals(self) -> None:
        """Recover the injected common-mode shift (median) and per-joint residual spread."""
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        report = estimate_command_lag(_delayed_trajectory(lags))

        self.assertIsNotNone(report)
        self.assertIsNotNone(report.common_mode_lag_samples)
        self.assertLess(abs(report.common_mode_lag_samples - _COMMON_LAG_SAMPLES), 0.2)
        self.assertLess(abs(report.common_mode_lag_seconds - _COMMON_LAG_SAMPLES * _DT), 0.2 * _DT)
        self.assertAlmostEqual(report.suggested_command_alignment_seconds, report.common_mode_lag_seconds)
        self.assertAlmostEqual(report.suggested_command_alignment_seconds, report.common_mode_lag_seconds)
        for estimate, offset in zip(report.joints, _JOINT_OFFSETS):
            self.assertIsNotNone(estimate.lag_samples)
            self.assertGreater(estimate.correlation_peak, 0.95)
            self.assertLess(abs(estimate.residual_samples - offset), 0.25)
        self.assertGreater(report.residual_spread_samples, 0.4)
        self.assertLess(report.residual_spread_samples, 1.0)

    async def test_lag_estimate_matches_direct_cross_correlation(self) -> None:
        """The vectorized correlation must reproduce the per-lag reference exactly.

        `_reference_joint_lag` is the straightforward per-lag normalized
        cross-correlation this estimator is defined by. Any faster formulation has
        to agree with it to well inside the sub-sample refinement step.
        """
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        # 1 kHz telemetry, where the one-second window spans 2001 candidate lags
        # per joint — the regime that made the shipped per-lag loop expensive.
        trajectory = _delayed_trajectory(lags, duration_seconds=12.0, rate_hz=1000.0)
        dt = float(np.median(np.diff(trajectory.times)))
        max_lag = max(1, int(round(1.0 / dt)))

        report = estimate_command_lag(trajectory)

        self.assertIsNotNone(report)
        self.assertEqual(report.max_lag_samples, max_lag)
        self.assertEqual(max_lag, 1000)
        for joint, estimate in enumerate(report.joints):
            expected_lag, expected_peak = _reference_joint_lag(
                trajectory.times,
                trajectory.commands[:, joint],
                trajectory.velocities[:, joint],
                max_lag,
            )
            self.assertAlmostEqual(estimate.correlation_peak, expected_peak, places=9)
            self.assertIsNotNone(estimate.lag_samples)
            self.assertAlmostEqual(estimate.lag_samples, expected_lag, places=6)

    async def test_lag_estimate_is_exact_under_a_steady_command_rate(self) -> None:
        """A constant command rate must not cost the estimate any precision.

        A steady rate adds the same constant to every sample of d(command)/dt.
        The normalized correlation is unchanged by that shift, so the estimate has
        to keep matching the per-lag reference however large the rate is relative
        to the excitation.
        """
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        trajectory = _delayed_trajectory(lags, duration_seconds=12.0, rate_hz=1000.0)
        creep_rate = 1.0e4
        trajectory.commands += creep_rate * trajectory.times[:, None]
        trajectory.velocities += creep_rate
        max_lag = max(1, int(round(1.0 / float(np.median(np.diff(trajectory.times))))))

        report = estimate_command_lag(trajectory)

        self.assertIsNotNone(report)
        for joint, estimate in enumerate(report.joints):
            expected_lag, expected_peak = _reference_joint_lag(
                trajectory.times,
                trajectory.commands[:, joint],
                trajectory.velocities[:, joint],
                max_lag,
            )
            self.assertIsNotNone(expected_lag)
            self.assertAlmostEqual(estimate.correlation_peak, expected_peak, places=9)
            self.assertIsNotNone(estimate.lag_samples)
            self.assertAlmostEqual(estimate.lag_samples, expected_lag, places=6)

    async def test_lag_estimate_is_exact_after_an_isolated_command_glitch(self) -> None:
        """One extreme command sample must not corrupt the score of windows that exclude it.

        A single-sample glitch puts a derivative spike roughly 1e8 times the excitation
        into d(command)/dt. Windows that do not contain it must still score exactly as
        the per-lag reference does, so the recovered lag is unchanged.
        """
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        trajectory = _delayed_trajectory(lags, duration_seconds=12.0, rate_hz=1000.0)
        max_lag = max(1, int(round(1.0 / float(np.median(np.diff(trajectory.times))))))
        # Early enough that the windows around the true lag exclude it.
        glitch_sample = max_lag // 10
        trajectory.commands[glitch_sample, :] += 2.0e5

        report = estimate_command_lag(trajectory)

        self.assertIsNotNone(report)
        for joint, estimate in enumerate(report.joints):
            expected_lag, expected_peak = _reference_joint_lag(
                trajectory.times,
                trajectory.commands[:, joint],
                trajectory.velocities[:, joint],
                max_lag,
            )
            self.assertIsNotNone(expected_lag)
            self.assertAlmostEqual(estimate.correlation_peak, expected_peak, places=6)
            self.assertIsNotNone(estimate.lag_samples)
            self.assertAlmostEqual(estimate.lag_samples, expected_lag, places=6)

    async def test_lag_estimate_is_exact_when_the_command_rate_changes(self) -> None:
        """Short windows away from the median command rate must keep full precision.

        A 2.2 s recording leaves 200-sample windows for 2001 candidate lags. The
        command rate is 1e6 except for a 0.5 s segment at half that rate, and the
        windows around the true lag sit entirely inside that segment. Shifting by
        the median rate cannot center them, so their energy and correlation
        numerator are evaluated about 5e5 away from zero.
        """
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        trajectory = _delayed_trajectory(lags, duration_seconds=2.2, rate_hz=1000.0)
        dt = float(np.median(np.diff(trajectory.times)))
        max_lag = max(1, int(round(1.0 / dt)))
        base_rate = 1.0e6
        slow_start, slow_end = 0.8, 1.3

        def command_rate(t: np.ndarray) -> np.ndarray:
            return np.where((t >= slow_start) & (t < slow_end), 0.5 * base_rate, base_rate)

        times = trajectory.times
        trajectory.commands += (base_rate * times - 0.5 * base_rate * np.clip(times - slow_start, 0.0, 0.5))[:, None]
        for joint, lag in enumerate(lags):
            trajectory.velocities[:, joint] += command_rate(times - lag * dt)

        report = estimate_command_lag(trajectory)

        self.assertIsNotNone(report)
        for joint, estimate in enumerate(report.joints):
            expected_lag, expected_peak = _reference_joint_lag(
                trajectory.times,
                trajectory.commands[:, joint],
                trajectory.velocities[:, joint],
                max_lag,
            )
            self.assertIsNotNone(expected_lag)
            self.assertAlmostEqual(estimate.correlation_peak, expected_peak, places=9)
            self.assertIsNotNone(estimate.lag_samples)
            self.assertAlmostEqual(estimate.lag_samples, expected_lag, places=6)

    async def test_window_statistics_match_direct_computation(self) -> None:
        """Every window's mean and centered energy must match the direct two-pass result.

        Widths cover one sample, the whole series, powers of two, and widths with
        several set bits, on a series with a large offset and a rate step.
        """
        rng = np.random.default_rng(7)
        values = 1.0e6 + rng.normal(size=1000)
        values[400:] += 5.0e5
        for width in (1, 2, 3, 64, 255, 256, 257, 999, 1000):
            means, energies = _window_means_and_energies(values, width)
            windows = np.lib.stride_tricks.sliding_window_view(values, width)
            expected_means = windows.mean(axis=1)
            expected_energies = np.square(windows - expected_means[:, None]).sum(axis=1)
            self.assertEqual(means.shape, expected_means.shape)
            np.testing.assert_allclose(means, expected_means, rtol=1e-14, atol=0.0)
            np.testing.assert_allclose(energies, expected_energies, rtol=1e-9, atol=1e-6)

    async def test_quality_report_warns_with_suggested_correction(self) -> None:
        """Common-mode lag above 1.5 samples surfaces a warn-first issue with the fix."""
        lags = [_COMMON_LAG_SAMPLES + offset for offset in _JOINT_OFFSETS]
        report = build_telemetry_quality_report(_delayed_trajectory(lags))

        issues = {issue.code: issue for issue in report.issues}
        self.assertIn("command_lag_common_mode", issues)
        message = issues["command_lag_common_mode"].message
        self.assertIn("command_alignment_seconds", message)
        self.assertIn(f"{report.command_lag.suggested_command_alignment_seconds:.4f}", message)
        self.assertIn("actuator-model identification", message)
        payload = report.to_dict()
        self.assertEqual(len(payload["command_lag"]["joints"]), _NUM_JOINTS)

    async def test_aligned_trajectory_reports_no_common_mode_warning(self) -> None:
        """Near-zero-lag telemetry estimates ~0 common mode and emits no lag issue."""
        # Exactly 0.0 would make commands == positions and trip the
        # command_matches_position guard, which skips lag estimation entirely.
        report = build_telemetry_quality_report(_delayed_trajectory([0.05] * _NUM_JOINTS))

        self.assertNotIn("command_lag_common_mode", {issue.code for issue in report.issues})
        self.assertIsNotNone(report.command_lag)
        self.assertLess(abs(report.command_lag.common_mode_lag_samples), 0.5)

    async def test_apply_command_delay_realigns_commands(self) -> None:
        """Shifting by the injected integer lag makes commands match the measured response."""
        lag_samples = 16.0
        trajectory = _delayed_trajectory([lag_samples] * _NUM_JOINTS)
        misalignment = float(np.sqrt(np.mean((trajectory.commands - trajectory.positions) ** 2)))

        corrected = apply_command_delay(trajectory, lag_samples * _DT)

        interior = slice(int(lag_samples), None)
        np.testing.assert_allclose(corrected.commands[interior], corrected.positions[interior], rtol=0.0, atol=1e-9)
        self.assertGreater(misalignment, 0.1)

    async def test_alignment_then_actuator_delay_compose_once_each(self) -> None:
        times = np.arange(10, dtype=np.float64)
        trajectory = TrajectoryDataset(
            times=times,
            positions=np.zeros((10, 1), dtype=np.float64),
            velocities=np.zeros((10, 1), dtype=np.float64),
            commands=times[:, None].copy(),
        )

        aligned = apply_command_delay(trajectory, 2.0)
        actuated = _delayed_command_rows(
            torch.as_tensor(aligned.commands, dtype=torch.float32),
            np.asarray([[3.0]], dtype=np.float32),
            1.0,
        )

        np.testing.assert_allclose(
            actuated[0, :, 0].numpy(),
            np.maximum(times - 5.0, 0.0),
        )

    async def test_bridge_applies_delay_once_for_each_actuator_owner(self) -> None:
        commands = torch.arange(6, dtype=torch.float32).view(-1, 1).repeat(1, 2)

        staged = _commands_for_actuator_owners(
            commands,
            np.asarray([[2.0, 2.0]], dtype=np.float32),
            actuator_dt=1.0,
            explicit_columns=(1,),
        )

        np.testing.assert_allclose(staged[0, :, 0].numpy(), np.asarray([0, 0, 0, 1, 2, 3], dtype=np.float32))
        np.testing.assert_allclose(staged[0, :, 1].numpy(), commands[:, 1].numpy())

    async def test_nonfinite_selected_actuator_delay_is_rejected(self) -> None:
        entries = [
            SysIdParameterEntry(
                param_type=SysIdParameterType.ACTUATOR_COMMAND_DELAY_SECONDS,
                dof_index=0,
            )
        ]

        for value in (float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "must be finite"):
                actuator_command_delay_seconds_vector(2, entries, np.asarray([value], dtype=np.float32))

    async def test_delay_staging_rejects_invalid_timestep_and_nonfinite_rows(self) -> None:
        commands = torch.zeros((3, 1), dtype=torch.float32)
        with self.assertRaisesRegex(ValueError, "actuator_dt"):
            _delayed_command_rows(commands, np.zeros((1, 1), dtype=np.float32), float("nan"))
        with self.assertRaisesRegex(ValueError, "finite"):
            _delayed_command_rows(commands, np.asarray([[np.nan]], dtype=np.float32), 0.1)

    async def test_load_trajectory_applies_spec_command_alignment(self) -> None:
        """Telemetry alignment shifts commands at load and is recorded separately."""
        lag_samples = 16.0
        trajectory = _delayed_trajectory([lag_samples] * _NUM_JOINTS)
        header = (
            ["time"]
            + [f"q{i + 1}" for i in range(_NUM_JOINTS)]
            + [f"dq{i + 1}" for i in range(_NUM_JOINTS)]
            + [f"cmd{i + 1}" for i in range(_NUM_JOINTS)]
        )
        rows = np.hstack([trajectory.times[:, None], trajectory.positions, trajectory.velocities, trajectory.commands])
        with tempfile.TemporaryDirectory(prefix="sysid_command_delay_") as tmp_dir:
            csv_path = Path(tmp_dir) / "telemetry.csv"
            csv_path.write_text(
                ",".join(header) + "\n" + "\n".join(",".join(f"{v:.12g}" for v in row) for row in rows) + "\n",
                encoding="utf-8",
            )
            spec = TelemetryRunSpec(
                source_type="csv",
                source_path=str(csv_path),
                command_alignment_seconds=lag_samples * _DT,
            )

            loaded = load_trajectory(spec.to_load_config())

        interior = slice(int(lag_samples), None)
        np.testing.assert_allclose(loaded.commands[interior], loaded.positions[interior], rtol=0.0, atol=1e-6)
        self.assertIsNotNone(loaded.metadata)
        self.assertAlmostEqual(loaded.metadata.extra["command_alignment_seconds"], lag_samples * _DT)

    async def test_run_spec_round_trips_canonical_command_alignment(self) -> None:
        spec = SysIdRunSpec.from_dict({"telemetry": {"command_alignment_seconds": 0.125}})

        self.assertAlmostEqual(spec.telemetry.resolved_command_alignment_seconds(), 0.125)
        self.assertAlmostEqual(spec.telemetry.to_load_config().command_alignment_seconds, 0.125)
        self.assertAlmostEqual(spec.to_dict()["telemetry"]["command_alignment_seconds"], 0.125)

    async def test_schema_validation_knows_command_alignment_key(self) -> None:
        """The new spec key is allowlisted; non-numeric values are rejected."""
        canonical = validate_sysid_run_spec_payload(
            {"schema_version": 2, "telemetry": {"command_alignment_seconds": 0.53}}
        )
        self.assertFalse([issue for issue in canonical if "command_alignment_seconds" in issue.path])

        bad = validate_sysid_run_spec_payload({"schema_version": 2, "telemetry": {"command_alignment_seconds": "half"}})
        errors = [issue for issue in bad if issue.path == "$.telemetry.command_alignment_seconds"]
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0].severity, "error")
