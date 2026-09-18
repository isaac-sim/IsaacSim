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

"""Verify the ``get_data_multi`` contract for rigid-contact entities.

Some impls return several tensors at once rather than one. ``raw-contact-data``
(rigid-contact) is the example used here: one ``get_data_multi`` call yields the
whole bundle of per-contact tensors.

This module checks that multi-buffer get behaves like single-buffer get across
the same access shapes:

* full - read every entity; the engine allocates the result tensors.
* external-buffer - read into a list of caller-supplied buffers.
* indexed - read a K-entity subset. Counts and starts are K-sized; contact
  payload buffers retain the configured record capacity.
* indexed external - an indexed read into caller-supplied buffers.

Every returned tensor must land on the simulation's device (CPU sim -> host,
GPU sim -> CUDA), with no hidden host copy. The scene is one box per env above a
ground plane, with a contact view over the boxes. Scenario fixture/warmup and the
device assertion are reused from ``get_set_contract``.
"""

from __future__ import annotations

import os
import sys
from typing import Protocol

import numpy as np
import pytest
import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
)
from common.get_set_contract import _INDICES, _assert_device, _ContractMixin  # noqa: E402
from pxr import Gf  # noqa: E402

_IMPL = "raw-contact-data"


class _ContactViewFactory(Protocol):
    def create_rigid_contact_view(
        self,
        pattern: str,
        *,
        filter_patterns: list[str] | None = None,
        max_contact_data_count: int = 0,
    ) -> object: ...


class _ContactContractBase(_ContractMixin, GridTestBase):
    """Contact scenario with one box per environment above the ground.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)
        actor_path = self.env_template_path.AppendChild("box")
        self.create_rigid_box(actor_path, Transform((0.0, 0.0, 0.08)), Gf.Vec3f(0.2, 0.2, 0.2))
        # The engine-side test mixin reads this and attaches contact-report
        # schemas per env, so the contact view resolves the boxes at creation.
        self._contact_body_paths = [actor_path]
        self.is_gpu = bool(device_params.use_gpu_pipeline)
        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply backend-specific contact-report schemas when required."""

    def on_start(self, sim: _ContactViewFactory) -> None:
        """Create the rigid-contact view used by the contract.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.sim = sim
        self.view = sim.create_rigid_contact_view(
            "/envs/*/box", filter_patterns=["/groundPlane"], max_contact_data_count=self.num_envs * 6
        )


class _MultiGetBase(_ContactContractBase):
    """multi-buffer get of ``raw-contact-data`` (internal/external x full/indexed)."""

    INDEXED = False
    EXTERNAL = False
    EXPECT_EXTERNAL_ALIAS = False
    EXPECT_EXTERNAL_VALIDATION = False
    EXPECT_EMPTY_SELECTION = False
    EXPECT_ZERO_FORCE = False
    EXPECT_TRUNCATION_CLAMP = False

    def check_cell(self) -> None:
        tc = self.test_case
        assert list(self.view.paths) == ["/envs/*/box"]
        idx = wp.array(_INDICES, dtype=wp.int32, device=self.wp_device) if self.INDEXED else None
        full = self.view.get_data_multi(_IMPL)
        if self.EXTERNAL:
            ref = self.view.get_data_multi(_IMPL, idx)
            bufs = [wp.zeros(t.shape, dtype=t.dtype, device=self.wp_device) for t in ref]
            out = self.view.get_data_multi(_IMPL, idx, bufs)
        elif self.INDEXED:
            out = self.view.get_data_multi(_IMPL, idx)
        else:
            out = full
        assert isinstance(out, list), "raw-contact-data multi get: expected a list of tensors"
        assert len(out) == 7, "raw-contact-data multi get: expected seven tensors"
        for i, t in enumerate(out):
            _assert_device(self, t, f"raw-contact-data multi get[{i}]")
        if self.EXTERNAL and self.EXPECT_EXTERNAL_ALIAS:
            for i, (actual, supplied) in enumerate(zip(out, bufs)):
                assert actual.ptr == supplied.ptr, f"raw-contact-data output {i} did not alias caller buffer"
        if self.EXTERNAL and self.EXPECT_EXTERNAL_VALIDATION:
            with pytest.raises(ValueError, match="Expected 7 output buffers"):
                self.view.get_data_multi(_IMPL, idx, bufs[:-1])

            wrong_shape = list(bufs)
            wrong_shape[0] = wp.zeros((bufs[0].shape[0],), dtype=wp.float32, device=self.wp_device)
            with pytest.raises(ValueError, match="output buffer 0 has shape"):
                self.view.get_data_multi(_IMPL, idx, wrong_shape)

            wrong_dtype = list(bufs)
            wrong_dtype[0] = wp.zeros(bufs[0].shape, dtype=wp.float64, device=self.wp_device)
            with pytest.raises(TypeError, match="output buffer 0 has dtype"):
                self.view.get_data_multi(_IMPL, idx, wrong_dtype)

            # CPU-only hosts cannot allocate the opposite-device negative
            # fixture. GPU cells can always use CPU as the mismatched device.
            if self.wp_device != "cpu" or wp.get_cuda_device_count() > 0:
                other_device = "cuda:0" if self.wp_device == "cpu" else "cpu"
                wrong_device = list(bufs)
                wrong_device[0] = wp.zeros(bufs[0].shape, dtype=wp.float32, device=other_device)
                with pytest.raises(ValueError, match="output buffer 0 is on"):
                    self.view.get_data_multi(_IMPL, idx, wrong_device)

        record_capacity = self.num_envs * 6
        sensor_count = len(_INDICES) if self.INDEXED else self.num_envs
        expected_shapes = [
            (record_capacity, 1),
            (record_capacity, 3),
            (record_capacity, 3),
            (record_capacity, 1),
            (sensor_count,),
            (sensor_count,),
            (record_capacity,),
        ]
        assert [tuple(t.shape) for t in out] == expected_shapes
        assert [t.dtype for t in out[:4]] == [wp.float32] * 4
        assert out[4].dtype in (wp.int32, wp.uint32)
        assert out[5].dtype == out[4].dtype
        assert out[6].dtype in (wp.int64, wp.uint64)

        counts = out[4].numpy().astype(np.int64)
        starts = out[5].numpy().astype(np.int64)
        expected_starts = np.zeros(sensor_count, dtype=np.int64)
        if sensor_count > 1:
            expected_starts[1:] = np.cumsum(counts[:-1])
        np.testing.assert_array_equal(starts, expected_starts)

        if self.INDEXED:
            full_counts = full[4].numpy().astype(np.int64)
            full_starts = full[5].numpy().astype(np.int64)
            np.testing.assert_array_equal(counts, full_counts[_INDICES])
            for destination_sensor, source_sensor in enumerate(_INDICES):
                count = int(counts[destination_sensor])
                source_start = int(full_starts[source_sensor])
                destination_start = int(starts[destination_sensor])

                def canonical_records(buffers: list[object], start: int) -> np.ndarray:
                    """Sort one sensor's contact records into a stable order.

                    Args:
                        buffers: Raw contact tensors.
                        start: First record for the sensor.

                    Returns:
                        Contact records sorted lexicographically.

                    """
                    contact_slice = slice(start, start + count)
                    records = np.column_stack(
                        [
                            buffers[6].numpy()[contact_slice],
                            buffers[1].numpy()[contact_slice],
                            buffers[2].numpy()[contact_slice],
                            buffers[3].numpy()[contact_slice],
                            buffers[0].numpy()[contact_slice],
                        ]
                    )
                    if records.shape[0] == 0:
                        return records
                    keys = tuple(records[:, column] for column in range(records.shape[1] - 1, -1, -1))
                    return records[np.lexsort(keys)]

                # The raw kernels assign per-sensor record slots atomically;
                # compare record multisets rather than launch-order rows.
                np.testing.assert_allclose(
                    canonical_records(out, destination_start),
                    canonical_records(full, source_start),
                )
            if self.EXPECT_EMPTY_SELECTION:
                empty_indices = wp.empty(0, dtype=wp.int32, device=self.wp_device)
                empty = self.view.get_data_multi(_IMPL, empty_indices)
                assert tuple(empty[4].shape) == (0,)
                assert tuple(empty[5].shape) == (0,)
                for payload in empty[:4] + empty[6:]:
                    assert np.all(payload.numpy() == 0)
            if self.EXPECT_TRUNCATION_CLAMP:
                capacity = 1
                tiny = self.sim.create_rigid_contact_view(
                    "/envs/*/box",
                    filter_patterns=["/groundPlane"],
                    max_contact_data_count=capacity,
                )
                tiny_full = tiny.get_data_multi(_IMPL)
                source_sensor = _INDICES[-1]
                source_count = int(tiny_full[4].numpy()[source_sensor])
                source_start = int(tiny_full[5].numpy()[source_sensor])
                expected_available = max(0, min(source_count, capacity - source_start))
                selected = tiny.get_data_multi(
                    _IMPL,
                    wp.array([source_sensor], dtype=wp.int32, device=self.wp_device),
                )
                assert (
                    int(selected[4].numpy()[0]) == expected_available
                ), "indexed raw-contact metadata exceeds the source records retained by capacity"

                tiny_counts = tiny_full[4].numpy().astype(np.int64)
                tiny_starts = tiny_full[5].numpy().astype(np.int64)
                retained = np.maximum(0, np.minimum(tiny_counts, capacity - tiny_starts))
                duplicate_source = int(np.flatnonzero(retained > 0)[0])
                duplicate = tiny.get_data_multi(
                    _IMPL,
                    wp.array([duplicate_source, duplicate_source], dtype=wp.int32, device=self.wp_device),
                )
                duplicate_counts = duplicate[4].numpy().astype(np.int64)
                duplicate_starts = duplicate[5].numpy().astype(np.int64)
                assert (
                    int(np.sum(duplicate_counts)) <= capacity
                ), "duplicate indexed sensors advertised more records than the destination can hold"
                np.testing.assert_array_equal(duplicate_starts, [0, duplicate_counts[0]])
        else:
            assert np.all(counts > 0), "raw-contact-data fixture did not produce contacts"
            forces = out[0].numpy().reshape(-1)
            normals = out[2].numpy().reshape(-1, 3)
            separations = out[3].numpy().reshape(-1)
            for count, start in zip(counts, starts):
                contact_slice = slice(int(start), int(start + count))
                assert np.all(np.isfinite(forces[contact_slice]))
                assert np.all(np.isfinite(normals[contact_slice]))
                # Separation is the signed contact gap (negative = penetration); its exact
                # few-mm residual at rest is a solver detail we don't pin. Bounding the
                # magnitude to well under the 0.1 box half-height catches a gross sign/scale
                # regression (e.g. separations coming back as large negatives) without
                # depending on the residual. The A=-B sign convention is covered by
                # RawContactSignConvention.
                assert np.all(
                    np.isfinite(separations[contact_slice])
                ), f"raw-contact-data separations must be finite: {separations[contact_slice]}"
                assert np.all(np.abs(separations[contact_slice]) < 0.1), (
                    f"raw-contact-data separation implausibly large vs the 0.1 box half-height: "
                    f"{separations[contact_slice]}"
                )
                net_normal = np.sum(normals[contact_slice], axis=0)
                assert net_normal[2] > 0.0, "raw-contact-data normal sign disagrees with the native oracle"
            if self.EXPECT_ZERO_FORCE:
                assert np.all(forces == 0.0), "pinned Newton XPBD unexpectedly published raw contact forces"


class RawContactMultiGetCommon(_MultiGetBase):
    """Verify full raw-contact reads with engine-allocated output buffers."""

    INDEXED = False
    EXTERNAL = False

    def check_cell(self) -> None:
        """Verify raw-contact outputs and the empty-view metadata contract."""
        super().check_cell()
        empty = self.sim.create_rigid_contact_view("/does/not/match")
        assert empty.count == 0
        assert list(empty.paths) == ["/does/not/match"]
        # A zero-sensor contact view exposes no sensor names -- it must NOT fall
        # back to the unmatched query glob (sensor_count and sensor_names agree).
        assert empty.sensor_count == 0
        assert list(empty.sensor_names) == []


class RawContactIndexedMultiGetCommon(_MultiGetBase):
    """Verify indexed raw-contact reads with engine-allocated output buffers."""

    INDEXED = True
    EXTERNAL = False


class RawContactExternalMultiGetCommon(_MultiGetBase):
    """Verify full raw-contact reads into caller-provided output buffers."""

    INDEXED = False
    EXTERNAL = True


class RawContactIndexedExternalMultiGetCommon(_MultiGetBase):
    """Verify indexed raw-contact reads into caller-provided output buffers."""

    INDEXED = True
    EXTERNAL = True
