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

"""Exercise articulation body, shape, and tendon tensor properties.

The scenarios round-trip Humanoid body and shape data, fixed and spatial
tendon data, and indexed tendon updates across heterogeneous articulation
views.
"""

from __future__ import annotations

import os
import sys

import warp as wp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

import warp_utils as wp_utils  # noqa: E402
from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    SimulationEntities,
    Transform,
    get_asset_root,
)


class BodyPropertiesCommon(GridTestBase):
    """Round-trip Humanoid link mass, center-of-mass, and inertia data.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("humanoid"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and verify body properties for every Humanoid link.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count, device=self.wp_device)

        max_links = humanoids.get_metadata("num-links")

        masses = wp.full((humanoids.count, max_links), 100.0, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("masses", masses, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("masses"), masses)

        inv_masses = humanoids.get_data("inv-masses").numpy()
        assert inv_masses.shape == (self.num_envs, max_links)

        com = humanoids.get_data("coms").numpy().reshape(self.num_envs, max_links, 7).astype("float32")
        com[:, :, 0] += 0.1
        wp_coms = wp.from_numpy(com, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("coms", wp_coms, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("coms").numpy(), com)

        inertias = humanoids.get_data("inertias").numpy().reshape(self.num_envs, max_links, 9).astype("float32")
        inertias[:, :, [0, 4, 8]] += 0.1
        wp_inertias = wp.from_numpy(inertias, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("inertias", wp_inertias, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("inertias").numpy(), inertias)

        assert humanoids.get_data("inv-inertias").numpy().shape == (self.num_envs, max_links, 9)
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """


class ShapePropertiesCommon(GridTestBase):
    """Round-trip Humanoid material, contact-offset, and rest-offset data.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("humanoid"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and verify shape properties for every Humanoid shape.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        humanoids = sim.create_articulation_view("/envs/*/humanoid/torso")
        self.check_articulation_view(humanoids, self.num_envs, 16, 21, True)
        all_indices = wp_utils.arange(humanoids.count, device=self.wp_device)

        max_shapes = humanoids.get_metadata("num-shapes")

        mat_props = wp.zeros((humanoids.count, max_shapes, 3), dtype=wp.float32, device="cpu").numpy()
        mat_props[:, :, 0] = 0.8
        mat_props[:, :, 1] = 0.7
        mat_props[:, :, 2] = 0.6
        wp_mat = wp.from_numpy(mat_props, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("material-properties", wp_mat, all_indices)
        assert wp_utils.wp_allclose(
            humanoids.get_data("material-properties").numpy(), mat_props
        ), "material properties round-trip"

        contact_offsets = wp.full((humanoids.count, max_shapes), 0.1, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("contact-offsets", contact_offsets, all_indices)
        assert wp_utils.wp_allclose(
            humanoids.get_data("contact-offsets"), contact_offsets
        ), "contact offsets round-trip"

        rest_offsets = wp.full((humanoids.count, max_shapes), 0.05, dtype=wp.float32, device=self.wp_device)
        humanoids.set_data("rest-offsets", rest_offsets, all_indices)
        assert wp_utils.wp_allclose(humanoids.get_data("rest-offsets"), rest_offsets), "rest offsets round-trip"

        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """

        # ---------------------------------------------------------------------------


# FixedTendonPropertiesCommon — port of legacy
# `TestArticulationFixedTendonProperties`. Loads `ShadowHand.usda` (which
# defines 4 fixed tendons via `PhysxSchema.PhysxTendonAxisRootAPI`); writes
# six per-tendon properties through the `fixed-tendon-*` impls, reads
# them back, and verifies the round-trip. The PhysxSchema lives in the
# *asset*, not in the scenario — Common scenarios are allowed to load
# such assets; only authoring PhysxSchema in scenario code is engine-
# specific. Exercises ovphysx tensor types 80-85.
# ---------------------------------------------------------------------------


class FixedTendonPropertiesCommon(GridTestBase):
    """Round-trip all fixed-tendon properties on replicated Shadow Hands.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "ShadowHand.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("shadow_hand"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and verify fixed-tendon properties.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        hands = sim.create_articulation_view("/envs/*/shadow_hand")
        self.check_articulation_view(hands, self.num_envs, 26, 24, True)
        all_indices = wp_utils.arange(hands.count, device=self.wp_device)

        max_tendons = hands.get_metadata("num-fixed-tendons")
        assert max_tendons is not None, "num-fixed-tendons metadata is not registered"
        assert max_tendons == 4

        n = hands.count * max_tendons
        # Per-tendon properties; the legacy test multiplies per-row indices
        # to build distinct values, but on ovphysx GPU mode the tendon write
        # accepts uniform values across envs, so we use a stable sweep
        # across [0, max_tendons) + a per-env offset.
        stiff = (10.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(hands.count, max_tendons)
        damp = (100.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(hands.count, max_tendons)
        lim_stiff = (50.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(
            hands.count, max_tendons
        )
        rest = (0.5 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(hands.count, max_tendons)
        off = (0.1 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(hands.count, max_tendons)
        lim = wp.zeros((hands.count, max_tendons, 2), dtype=wp.float32, device="cpu").numpy()
        flat = wp_utils.arange(n * 2, device="cpu").numpy().astype("float32").reshape(hands.count, max_tendons, 2)
        lim[..., 0] = -2.0 * flat[..., 0]
        lim[..., 1] = 2.0 * flat[..., 1]

        wp_args = {
            "fixed-tendon-stiffnesses": stiff,
            "fixed-tendon-dampings": damp,
            "fixed-tendon-limit-stiffnesses": lim_stiff,
            "fixed-tendon-limits": lim,
            "fixed-tendon-rest-lengths": rest,
            "fixed-tendon-offsets": off,
        }
        for impl, np_data in wp_args.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            hands.set_data(impl, wp_data, all_indices)

        # Round-trip readback.
        for impl, np_data in wp_args.items():
            actual = hands.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"{impl} round-trip: max diff={abs(actual - np_data).max()}"
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """

        # ---------------------------------------------------------------------------


# SpatialTendonPropertiesCommon — port of legacy
# `TestArticulationSpatialTendonProperties`. Loads
# `MultipleSpatialTendonsTest.usda` (asset defines spatial tendons via
# `PhysxSchema.PhysxTendonAttachmentRootAPI`); writes four per-tendon
# properties through `spatial-tendon-*` impls and verifies round-trip.
# Exercises ovphysx tensor types 90-93.
# ---------------------------------------------------------------------------


class SpatialTendonPropertiesCommon(GridTestBase):
    """Round-trip all properties on replicated spatial-tendon articulations.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        asset_path = os.path.join(get_asset_root(), "MultipleSpatialTendonsTest.usda")
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("spatial_tendon"),
            Transform((0.0, 0.0, 1.5)),
            asset_path,
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Write and verify spatial-tendon properties.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        view = sim.create_articulation_view("/envs/*/spatial_tendon")
        all_indices = wp_utils.arange(view.count, device=self.wp_device)

        max_tendons = view.get_metadata("num-spatial-tendons")
        assert max_tendons is not None, "num-spatial-tendons metadata is not registered"
        assert max_tendons > 0, "asset exposes no spatial tendons"

        n = view.count * max_tendons
        stiff = (10.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(view.count, max_tendons)
        damp = (100.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(view.count, max_tendons)
        lim_stiff = (50.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(view.count, max_tendons)
        off = (0.1 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(view.count, max_tendons)

        wp_args = {
            "spatial-tendon-stiffnesses": stiff,
            "spatial-tendon-dampings": damp,
            "spatial-tendon-limit-stiffnesses": lim_stiff,
            "spatial-tendon-offsets": off,
        }
        for impl, np_data in wp_args.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            view.set_data(impl, wp_data, all_indices)

        for impl, np_data in wp_args.items():
            actual = view.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"{impl} round-trip: max diff={abs(actual - np_data).max()}"
        self.finish()

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Keep the scenario complete after startup validation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based physics step number.
            dt: Duration of the physics step.

        """

        # ---------------------------------------------------------------------------


# HeterogeneousFixedTendonPropertiesCommon — port of legacy
# `TestArticulationHeterogeneousFixedTendonProperties`. Each env loads two
# different articulations (`FixedTendonTest.usda` with 1 tendon +
# `ShadowHand.usda` with 4 tendons); two views share the scene, each
# operates on its own articulation type. Exercises:
#   * Per-articulation tendon counts (1 vs 4) — the "heterogeneous" bit.
#   * Indexed writes — only envs [0, 4, 5] of the hands view get touched;
#     the other envs keep their pre-write values, which is the read-back
#     check.
# ---------------------------------------------------------------------------


class HeterogeneousFixedTendonPropertiesCommon(GridTestBase):
    """Validate fixed-tendon properties across two articulation topologies.

    The scenario verifies independent one-tendon and four-tendon views and
    performs indexed writes on selected Shadow Hand environments.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("FixedTendonTest"),
            Transform((0.0, 0.0, 1.5)),
            os.path.join(get_asset_root(), "FixedTendonTest.usda"),
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("shadow_hand"),
            Transform((0.0, 0.0, -1.5)),
            os.path.join(get_asset_root(), "ShadowHand.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create both views and prepare their property buffers.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self._ftt = sim.create_articulation_view("/envs/*/FixedTendonTest")
        self._hands = sim.create_articulation_view("/envs/*/shadow_hand")

        ftt_max = self._ftt.get_metadata("num-fixed-tendons")
        hands_max = self._hands.get_metadata("num-fixed-tendons")
        assert ftt_max is not None, "FixedTendonTest metadata is not registered"
        assert hands_max is not None, "ShadowHand tendon metadata is not registered"
        assert ftt_max > 0
        assert hands_max > 0
        assert ftt_max == 1
        assert hands_max == 4
        self._ftt_max = ftt_max
        self._hands_max = hands_max

        # Pre-compute the write data (CPU-only, no engine calls yet).
        ftt, hands = self._ftt, self._hands
        n_ftt = ftt.count * ftt_max
        ftt_props = {
            "fixed-tendon-stiffnesses": (10.0 * wp_utils.arange(n_ftt, device="cpu").numpy().astype("float32")).reshape(
                ftt.count, ftt_max
            ),
            "fixed-tendon-dampings": (100.0 * wp_utils.arange(n_ftt, device="cpu").numpy().astype("float32")).reshape(
                ftt.count, ftt_max
            ),
            "fixed-tendon-limit-stiffnesses": (
                50.0 * wp_utils.arange(n_ftt, device="cpu").numpy().astype("float32")
            ).reshape(ftt.count, ftt_max),
            "fixed-tendon-rest-lengths": (0.5 * wp_utils.arange(n_ftt, device="cpu").numpy().astype("float32")).reshape(
                ftt.count, ftt_max
            ),
            "fixed-tendon-offsets": (0.1 * wp_utils.arange(n_ftt, device="cpu").numpy().astype("float32")).reshape(
                ftt.count, ftt_max
            ),
        }
        ftt_lim = wp.zeros((ftt.count, ftt_max, 2), dtype=wp.float32, device="cpu").numpy()
        flat = wp_utils.arange(n_ftt * 2, device="cpu").numpy().astype("float32").reshape(ftt.count, ftt_max, 2)
        ftt_lim[..., 0] = -2.0 * flat[..., 0]
        ftt_lim[..., 1] = 2.0 * flat[..., 1]
        ftt_props["fixed-tendon-limits"] = ftt_lim
        self._ftt_props = ftt_props
        self._ftt_all_indices = wp_utils.arange(ftt.count, device=self.wp_device)

        self._hands_indices_np = [0, 4, 5]
        self._hands_indices = wp.array(self._hands_indices_np, dtype=wp.int64, device=self.wp_device)

        # In GPU pipeline mode, GPU tensor reads/writes require at least one
        # simulation step (GpuSimulationData::checkApiReady). Defer all engine
        # data operations to on_physics_step(stepno=0).
        if not self.device_params.use_gpu_pipeline:
            self._do_tendon_write_and_verify()
            self.finish()

    def _do_tendon_write_and_verify(self) -> None:
        ftt, hands = self._ftt, self._hands
        ftt_max, hands_max = self._ftt_max, self._hands_max
        ftt_props = self._ftt_props
        all_indices = self._ftt_all_indices
        hands_indices_np = self._hands_indices_np
        hands_indices = self._hands_indices

        for impl, np_data in ftt_props.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            ftt.set_data(impl, wp_data, all_indices)

        # Indexed write on the hands view: only envs 0, 4, 5 get written.
        hands_indices_np = [0, 4, 5]
        hands_indices = wp.array(hands_indices_np, dtype=wp.int32, device=self.wp_device)

        hands_props = {}
        for impl, value in (
            ("fixed-tendon-stiffnesses", 10.0),
            ("fixed-tendon-dampings", 50.0),
            ("fixed-tendon-limit-stiffnesses", 100.0),
            ("fixed-tendon-rest-lengths", 0.5),
            ("fixed-tendon-offsets", 0.1),
        ):
            current = hands.get_data(impl).numpy().reshape(hands.count, hands_max).copy()
            current[hands_indices_np, :] = value
            hands_props[impl] = current
        hands_limits = hands.get_data("fixed-tendon-limits").numpy().reshape(hands.count, hands_max, 2).copy()
        hands_limits[hands_indices_np, :, 0] = -2.0
        hands_limits[hands_indices_np, :, 1] = 2.0
        hands_props["fixed-tendon-limits"] = hands_limits
        for impl, np_data in hands_props.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            hands.set_data(impl, wp_data, hands_indices)

        for impl, np_data in ftt_props.items():
            actual = ftt.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"FixedTendonTest {impl}: max diff={abs(actual - np_data).max()}"
        for impl, np_data in hands_props.items():
            actual = hands.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"hands {impl}: max diff={abs(actual - np_data).max()}"

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Run deferred GPU tendon writes after simulation becomes ready.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based callback step number used by this scenario.
            dt: Duration of the physics step.

        """
        if self.device_params.use_gpu_pipeline and stepno == 0:
            self._do_tendon_write_and_verify()
            self.finish()


# ---------------------------------------------------------------------------
# HeterogeneousSpatialTendonPropertiesCommon — same shape as fixed but with
# `SpatialTendonTest.usda` (1 tendon) + `MultipleSpatialTendonsTest.usda`
# (2 tendons). Spatial tendons have 4 properties (no rest-lengths /
# limits) per the engine API.
# ---------------------------------------------------------------------------


class HeterogeneousSpatialTendonPropertiesCommon(GridTestBase):
    """Validate spatial-tendon properties across two articulation topologies.

    The scenario verifies independent one-tendon and two-tendon views and
    performs indexed writes on selected multi-tendon environments.

    Args:
        test_case: Test instance that owns the scenario.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=16, env_spacing=2.0), SimParams(), device_params)
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("SpatialTendonTest"),
            Transform((0.0, 0.0, 1.5)),
            os.path.join(get_asset_root(), "SpatialTendonTest.usda"),
        )
        self.create_actor_from_asset(
            self.env_template_path.AppendChild("MultipleSpatialTendonsTest"),
            Transform((0.0, 0.0, -1.5)),
            os.path.join(get_asset_root(), "MultipleSpatialTendonsTest.usda"),
        )

    def on_start(self, sim: SimulationEntities) -> None:
        """Create both views and prepare their property buffers.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self._single = sim.create_articulation_view("/envs/*/SpatialTendonTest")
        self._multi = sim.create_articulation_view("/envs/*/MultipleSpatialTendonsTest")

        single_max = self._single.get_metadata("num-spatial-tendons")
        multi_max = self._multi.get_metadata("num-spatial-tendons")
        assert single_max is not None, "single spatial-tendon metadata is not registered"
        assert multi_max is not None, "multi spatial-tendon metadata is not registered"
        assert single_max > 0
        assert multi_max > 0
        assert single_max == 1
        assert multi_max == 2
        self._single_max = single_max
        self._multi_max = multi_max

        single = self._single
        n = single.count * single_max
        self._single_props = {
            "spatial-tendon-stiffnesses": (10.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(
                single.count, single_max
            ),
            "spatial-tendon-dampings": (100.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(
                single.count, single_max
            ),
            "spatial-tendon-limit-stiffnesses": (
                50.0 * wp_utils.arange(n, device="cpu").numpy().astype("float32")
            ).reshape(single.count, single_max),
            "spatial-tendon-offsets": (0.1 * wp_utils.arange(n, device="cpu").numpy().astype("float32")).reshape(
                single.count, single_max
            ),
        }
        self._single_all_indices = wp_utils.arange(single.count, device=self.wp_device)
        self._multi_indices_np = [0, 4, 5]
        self._multi_indices = wp.array(self._multi_indices_np, dtype=wp.int64, device=self.wp_device)

        if not self.device_params.use_gpu_pipeline:
            self._do_spatial_tendon_write_and_verify()
            self.finish()

    def _do_spatial_tendon_write_and_verify(self) -> None:
        single, multi = self._single, self._multi
        single_max, multi_max = self._single_max, self._multi_max
        single_props = self._single_props
        all_indices = self._single_all_indices
        multi_indices_np = self._multi_indices_np
        multi_indices = self._multi_indices

        for impl, np_data in single_props.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            single.set_data(impl, wp_data, all_indices)

        multi_indices_np = [0, 4, 5]
        multi_indices = wp.array(multi_indices_np, dtype=wp.int32, device=self.wp_device)

        multi_props = {}
        for impl, value in (
            ("spatial-tendon-stiffnesses", 10.0),
            ("spatial-tendon-dampings", 100.0),
            ("spatial-tendon-limit-stiffnesses", 50.0),
            ("spatial-tendon-offsets", 0.1),
        ):
            current = multi.get_data(impl).numpy().reshape(multi.count, multi_max).copy()
            current[multi_indices_np, :] = value
            multi_props[impl] = current
        for impl, np_data in multi_props.items():
            wp_data = wp.from_numpy(np_data, dtype=wp.float32, device=self.wp_device)
            multi.set_data(impl, wp_data, multi_indices)

        for impl, np_data in single_props.items():
            actual = single.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"SpatialTendonTest {impl}: max diff={abs(actual - np_data).max()}"
        for impl, np_data in multi_props.items():
            actual = multi.get_data(impl).numpy().reshape(np_data.shape)
            assert wp_utils.wp_allclose(
                actual, np_data, rtol=1e-3, atol=1e-3
            ), f"multi {impl}: max diff={abs(actual - np_data).max()}"

    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Run deferred GPU tendon writes after simulation becomes ready.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based callback step number used by this scenario.
            dt: Duration of the physics step.

        """
        if self.device_params.use_gpu_pipeline and stepno == 0:
            self._do_spatial_tendon_write_and_verify()
            self.finish()
