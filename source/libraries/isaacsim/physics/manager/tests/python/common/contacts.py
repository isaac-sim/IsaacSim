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

"""Provide engine-neutral rigid-contact scenarios.

The scenarios validate net forces, filtered force matrices, structured contact
and friction buffers, raw contact records, raw-record sign conventions, and
articulation-link sensor metadata. Resting-box fixtures compare the steady
ground reaction with the box weight. Fixed-overlap fixtures make raw contacts
available without depending on a solver-specific settling period.

Engine-specific subclasses may implement ``_apply_engine_specifics`` when an
engine requires schemas or settings on contact-reporting bodies. Each affected
fixture exposes those bodies through ``self._contact_body_paths``.
"""

from __future__ import annotations

import os
import sys

import isaacsim.physics.manager.impl.tensors as t
import numpy as np
import pytest
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
    Transform,
)
from pxr import Gf, UsdPhysics  # noqa: E402

# Consecutive in-tolerance steps required before accepting one sensor's resting-contact
# reaction as equal to the weight. A short streak rejects the position-solver
# contact ramp-up's transient overshoot without assuming a fixed settle step.
_CONTACT_FORCE_STREAK = 3


class _PerSensorSettlingTracker:
    """Track independently when every sensor sustains an acceptable measurement.

    Args:
        num_sensors: Number of sensors to track.

    """

    __slots__ = ("_consecutive_steps", "_longest_streaks", "_settled")

    def __init__(self, num_sensors: int) -> None:
        self._consecutive_steps = np.zeros(num_sensors, dtype=np.int32)
        self._longest_streaks = np.zeros(num_sensors, dtype=np.int32)
        self._settled = np.zeros(num_sensors, dtype=bool)

    @property
    def is_complete(self) -> bool:
        """Whether every sensor has sustained an acceptable measurement."""
        return bool(self._settled.all())

    def update(self, is_in_tolerance: np.ndarray) -> None:
        """Record one step of per-sensor tolerance results.

        Args:
            is_in_tolerance: Boolean result for each sensor in the batched view.

        """
        self._consecutive_steps = np.where(is_in_tolerance, self._consecutive_steps + 1, 0)
        self._longest_streaks = np.maximum(self._longest_streaks, self._consecutive_steps)
        self._settled |= self._consecutive_steps >= _CONTACT_FORCE_STREAK

    def format_unsettled(self, values: np.ndarray) -> str:
        """Format the final values and longest streaks for sensors that never settled.

        Args:
            values: Final scalar or vector measurement for each sensor.

        Returns:
            Diagnostic text containing only sensors that did not settle.

        """
        indices = np.flatnonzero(~self._settled)
        return (
            f"unsettled sensor indices={indices.tolist()}, "
            f"longest streaks={self._longest_streaks[indices].tolist()}, "
            f"final values={values[indices].tolist()}"
        )


class RigidContactsCommon(GridTestBase):
    """Verify that resting boxes report a net contact force equal to weight.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(32, 1.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        actor_path = self.env_template_path.AppendChild("box")
        transform = Transform((0.0, 0.0, 0.15))
        box = self.create_rigid_box(actor_path, transform, Gf.Vec3f(0.3, 0.3, 0.3))

        self.box_mass = 1.0
        UsdPhysics.MassAPI(box).GetMassAttr().Set(self.box_mass)

        # The list of bodies the engine-specific layer should attach
        # contact-report API + sleep-threshold = 0 to. We store the
        # template path; the per-engine driver iterates env instances.
        self._contact_body_paths = [actor_path]

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create the rigid-contact view and reset settling state.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        contacts = sim.create_rigid_contact_view("/envs/*/box")
        self.check_rigid_contact_view(contacts, self.num_envs, 0)
        self.contacts = contacts
        self._settling = _PerSensorSettlingTracker(contacts.get_metadata("num-sensors"))

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Wait for a stable ground reaction equal to each box's weight.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        num_sensors = self.contacts.get_metadata("num-sensors")
        net_forces = self.contacts.get_data("net-contact-forces").numpy().reshape(num_sensors, 3)
        expected = np.array([0.0, 0.0, 9.81])
        self._settling.update(np.all(np.isclose(net_forces, expected, rtol=1e-3, atol=1e-2), axis=1))
        # Each spatially isolated box can settle on different steps, so accept its
        # own stable window instead of requiring every window to coincide.
        if self._settling.is_complete:
            self.finish()
        elif stepno + 1 >= self.maxsteps:
            details = self._settling.format_unsettled(net_forces)
            assert False, f"net contact force never reached weight m*g: {details}"
            self.finish()


class RigidContactMatrixCommon(GridTestBase):
    """Verify filtered contact-force matrices for grounded boxes.

    A ball is held above each grounded box so that only the ground filter
    contributes the box's weight.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=1.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 10.0
        super().__init__(test_case, grid_params, sim_params, device_params)
        self.g = 10.0
        self.ball_mass = 1.0
        self.box_mass = 1.0

        ball_path = self.env_template_path.AppendChild("ball")
        ball = self.create_rigid_ball(ball_path, Transform((0.0, 0.0, 1.0)), 0.25)
        UsdPhysics.MassAPI(ball).GetMassAttr().Set(self.ball_mass)

        box_path = self.env_template_path.AppendChild("box")
        box = self.create_rigid_box(box_path, Transform((0.0, 0.0, 0.15)), Gf.Vec3f(0.3, 0.3, 0.3))
        UsdPhysics.MassAPI(box).GetMassAttr().Set(self.box_mass)

        self._contact_body_paths = [ball_path, box_path]

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create body and contact views and initialize the hover force.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        balls = sim.create_rigid_body_view("/envs/*/ball")
        self.balls = balls
        self.box_indices = wp_utils.arange(self.num_envs, device=self.wp_device)
        self.ball_indices = wp_utils.arange(balls.count, device=self.wp_device)

        # Hover the ball with an applied force ~ ball_mass * g.
        hover_force = self.ball_mass * self.g
        self.ball_forces = wp_utils.fill_vec3(balls.count, value=wp.vec3(0.0, 0.0, hover_force), device=self.wp_device)
        balls.set_data("apply-forces", self.ball_forces, self.ball_indices)

        self.box_contacts = sim.create_rigid_contact_view(
            "/envs/*/box",
            filter_patterns=["/groundPlane", "/envs/*/ball"],
            max_contact_data_count=self.num_envs * 6,
        )
        self.check_rigid_contact_view(self.box_contacts, self.num_envs, 2)
        self._settling = _PerSensorSettlingTracker(self.box_contacts.get_metadata("num-sensors"))

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Maintain the hover force and validate the filtered ground reaction.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        # Keep the ball held aloft every step (Newton clears applied forces per step)
        # so it never loads the box; the box-vs-ground reaction is then the box weight.
        self.balls.set_data("apply-forces", self.ball_forces, self.ball_indices)
        box_force_matrix = (
            self.box_contacts.get_data("contact-force-matrix")
            .numpy()
            .reshape(self.box_contacts.get_metadata("num-sensors"), self.box_contacts.get_metadata("num-filters"), 3)
        )
        ground_z_force = box_force_matrix[:, 0, 2]
        expected = self.box_mass * self.g
        self._settling.update(np.isclose(np.abs(ground_z_force), expected, rtol=0.5, atol=0.5))
        if self._settling.is_complete:
            self.finish()
        elif stepno + 1 >= self.maxsteps:
            assert False, (
                f"box-vs-ground Z reaction never reached {expected}: "
                f"{self._settling.format_unsettled(ground_z_force)}"
            )
            self.finish()


class ContactDataCommon(RigidContactMatrixCommon):
    """Verify multi-buffer contact and friction reads from a filtered view.

    The inherited fixture filters each box against the ground and a hovering
    ball. The checks cover tensor count, device placement, shape, finite values,
    record-table consistency, and non-stale populated contact data.
    """

    @staticmethod
    def _record_mask(flat_counts: np.ndarray, flat_starts: np.ndarray, capacity: int) -> np.ndarray:
        """Build a mask selecting populated rows in a contact-record buffer.

        Args:
            flat_counts: Flattened record counts for each sensor-filter pair.
            flat_starts: Flattened starting offsets for each pair.
            capacity: Number of rows in the record buffer.

        Returns:
            Boolean mask with one element per record-buffer row.

        """
        mask = np.zeros(capacity, dtype=bool)
        for c, s in zip(flat_counts.tolist(), flat_starts.tolist()):
            if c > 0:
                mask[s : s + c] = True
        return mask

    def _check_multi(
        self,
        impl: str,
        expected_len: int,
        vec3_indices: tuple[int, ...],
        normals_index: int | None = None,
        normal_force_index: int | None = None,
    ) -> None:
        """Validate the structure and populated records of a multi-buffer read.

        Args:
            impl: Registered multi-buffer implementation name.
            expected_len: Expected number of returned tensors.
            vec3_indices: Positions of vector-valued tensors in the result.
            normals_index: Position of the contact-normal tensor, if present.
            normal_force_index: Position of the normal-force tensor, if present.

        """
        tc = self.test_case
        out = self.box_contacts.get_data_multi(impl)
        assert isinstance(out, list), f"{impl}: expected a list of tensors"
        assert len(out) == expected_len, f"{impl}: expected {expected_len} tensors, got {len(out)}"
        n_sensors = self.box_contacts.get_metadata("num-sensors")
        n_filters = self.box_contacts.get_metadata("num-filters")
        want_cuda = "cuda" in str(self.wp_device)
        for i, tensor in enumerate(out):
            assert (
                bool(tensor.device.is_cuda) == want_cuda
            ), f"{impl}[{i}] on {tensor.device}, expected cuda={want_cuda}"
            assert np.isfinite(tensor.numpy()).all(), f"{impl}[{i}] contains non-finite values"
        for i in vec3_indices:
            assert out[i].shape[1] == 3, f"{impl}[{i}] should be a [C, 3] buffer"
        # The last two outputs are the per-(sensor, filter) count/start tables.
        counts = out[expected_len - 2].numpy().astype(np.int64)
        starts = out[expected_len - 1].numpy().astype(np.int64)
        assert counts.shape == (n_sensors, n_filters), f"{impl}: counts should be [num_sensors, num_filters]"
        assert starts.shape == (n_sensors, n_filters), f"{impl}: starts should be [num_sensors, num_filters]"
        # Non-zero / non-stale: a DirectGPU read that failed to populate leaves the
        # pre-allocated buffer zeroed, so require the fixture's contacts to appear.
        capacity = out[0].shape[0]
        assert int(counts.sum()) > 0, f"{impl}: no contacts recorded (stale/zero DirectGPU read?)"
        # count/start consistency: every populated (sensor, filter) block is a
        # valid, in-capacity slice of the record buffers.
        assert np.all(starts >= 0), f"{impl}: negative record start offset"
        flat_c = counts.reshape(-1)
        flat_s = starts.reshape(-1)
        for c, s in zip(flat_c.tolist(), flat_s.tolist()):
            if c > 0:
                assert s + c <= capacity, f"{impl}: record block [{s}:{s + c}] exceeds capacity {capacity}"
        populated = self._record_mask(flat_c, flat_s, capacity)
        # Contact normals are unit vectors wherever a record exists.
        if normals_index is not None and populated.any():
            norms = np.linalg.norm(out[normals_index].numpy()[populated], axis=1)
            assert np.allclose(norms, 1.0, atol=1e-2), f"{impl}: contact normals are not unit length"
        # The resting box presses on the ground: some record carries a real force.
        if normal_force_index is not None and populated.any():
            forces = out[normal_force_index].numpy()[populated]
            assert float(np.max(np.abs(forces))) > 0.0, f"{impl}: all contact forces zero at populated records"

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Read and validate contact buffers after initial contact generation.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        # Keep the ball aloft (matrix fixture) so the box-vs-ground contact persists.
        self.balls.set_data("apply-forces", self.ball_forces, self.ball_indices)
        if stepno < _CONTACT_FORCE_STREAK:
            return
        # contact-data: forces[C,1] points[C,3] normals[C,3] separations[C,1] counts[S,F] starts[S,F]
        self._check_multi("contact-data", 6, vec3_indices=(1, 2), normals_index=2, normal_force_index=0)
        # friction-data: forces[C,3] points[C,3] counts[S,F] starts[S,F]
        # (the box is static, so tangential friction is ~0 -- no non-zero-force assertion here)
        self._check_multi("friction-data", 4, vec3_indices=(0, 1))
        # A partial out list violates the all-or-nothing multi-get contract -- a
        # clear error, not silent host-staging of the missing slots.
        partial = [wp.zeros((1,), dtype=wp.float32, device=self.wp_device)]
        with pytest.raises(Exception) as ctx:
            self.box_contacts.get_data_multi("friction-data", None, partial)
        assert "all or none" in str(ctx.value)
        self.finish()


class RawContactDataCommon(GridTestBase):
    """Overlapping-box raw-contact scenario.

    Exercises the seven-buffer ``raw-contact-data`` EntityView operation. Two
    boxes per env are placed in a fixed penetrating overlap with gravity off
    and no ground, so every env reports a contact deterministically on the
    first frame without dropping or settling.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    box_size = 0.3
    box_mass = 1.0

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=2.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0  # nothing moves -> contact is purely the initial overlap
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Two boxes in a fixed penetrating overlap (center distance < box_size).
        overlap = self.box_size / 3.0
        bottom_path = self.env_template_path.AppendChild("bottom_box")
        bottom = self.create_rigid_box(
            bottom_path,
            Transform((0.0, 0.0, 0.5)),
            Gf.Vec3f(self.box_size),
        )
        UsdPhysics.MassAPI(bottom).GetMassAttr().Set(self.box_mass)

        top_path = self.env_template_path.AppendChild("top_box")
        top = self.create_rigid_box(
            top_path,
            Transform((0.0, 0.0, 0.5 + self.box_size - overlap)),
            Gf.Vec3f(self.box_size),
        )
        UsdPhysics.MassAPI(top).GetMassAttr().Set(self.box_mass)

        self._contact_body_paths = [bottom_path, top_path]
        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create contact views for both overlapping-box roles.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.top_contacts = sim.create_rigid_contact_view(
            "/envs/*/top_box",
            max_contact_data_count=self.num_envs * 10,
        )
        self.bottom_contacts = sim.create_rigid_contact_view(
            "/envs/*/bottom_box",
            max_contact_data_count=self.num_envs * 10,
        )

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Verify that every top-box sensor returns a raw contact record.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt
        if self.finished:
            return
        # The overlap is fixed and gravity is zero, so every env reports a
        # contact on the first collide. `raw-contact-data` returns a 7-tuple of
        # warp arrays; a missing multi-get registration or adapter error is a
        # test failure.
        top_data = self.top_contacts.get_data_multi("raw-contact-data")
        forces, points, normals, separations, counts, start_indices, other_actor_ids = top_data
        counts_np = counts.numpy().flatten()
        assert (counts_np > 0).all(), f"all top-box sensors should have contacts. Counts: {counts_np}"
        self.finish()


class RawContactSignConventionCommon(GridTestBase):
    """Pin the raw-contact A/B sign convention shared by two sensors.

    Two boxes are placed in a *fixed penetrating overlap* with gravity disabled
    and no ground, so collision detection reports exactly one box_a<->box_b
    contact deterministically on the first frame -- no dropping, no settling, no
    Newton-GPU nondeterminism. The contact is reported by BOTH sensors; the raw
    kernel writes one side as ``(-normal, -separation)`` and the other as
    ``(+normal, +separation)`` at the *same* world contact point. So the two
    sensors' normals and separations for that contact must be exact negations.
    The shared contact points are used to match records between sensors.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    box_size = 0.3
    box_mass = 1.0

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=1, env_spacing=2.0)
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0  # nothing moves -> the contact is purely the initial overlap
        sim_params.add_default_ground = False
        super().__init__(test_case, grid_params, sim_params, device_params)

        # Center-to-center distance < box_size gives a fixed penetrating overlap.
        overlap = self.box_size / 3.0
        box_a_path = self.env_template_path.AppendChild("box_a")
        box_a = self.create_rigid_box(
            box_a_path,
            Transform((0.0, 0.0, 0.5)),
            Gf.Vec3f(self.box_size),
        )
        UsdPhysics.MassAPI(box_a).GetMassAttr().Set(self.box_mass)

        box_b_path = self.env_template_path.AppendChild("box_b")
        box_b = self.create_rigid_box(
            box_b_path,
            Transform((0.0, 0.0, 0.5 + self.box_size - overlap)),
            Gf.Vec3f(self.box_size),
        )
        UsdPhysics.MassAPI(box_b).GetMassAttr().Set(self.box_mass)

        self._contact_body_paths = [box_a_path, box_b_path]
        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create one contact view for each side of the overlap.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.a_contacts = sim.create_rigid_contact_view("/envs/*/box_a", max_contact_data_count=64)
        self.b_contacts = sim.create_rigid_contact_view("/envs/*/box_b", max_contact_data_count=64)

    @staticmethod
    def _sensor0_records(view: object) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Read populated raw-contact records for the first sensor.

        Args:
            view: Rigid-contact view containing at least one sensor.

        Returns:
            Contact points, normals, and separations for the first sensor.

        """
        _forces, points, normals, separations, counts, starts, _actors = view.get_data_multi("raw-contact-data")
        n = int(counts.numpy().flatten()[0])
        s = int(starts.numpy().flatten()[0])
        return points.numpy()[s : s + n], normals.numpy()[s : s + n], separations.numpy()[s : s + n]

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Match shared records and verify opposite normals and separations.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, stepno, dt
        if self.finished:
            return

        # The overlap is fixed and gravity is zero, so the box_a<->box_b contact
        # is present on the first collide. Read once, validate, and stop.
        a_points, a_normals, a_separations = self._sensor0_records(self.a_contacts)
        b_points, b_normals, b_separations = self._sensor0_records(self.b_contacts)
        assert len(a_points) > 0, "box_a must report the overlap contact"
        assert len(b_points) > 0, "box_b must report the overlap contact"

        matched = 0
        for i in range(len(a_points)):
            dist = np.linalg.norm(b_points - a_points[i], axis=1)
            j = int(np.argmin(dist))
            if dist[j] > 1e-3:
                continue
            matched += 1
            np.testing.assert_allclose(
                b_normals[j],
                -a_normals[i],
                atol=1e-5,
                err_msg="raw-contact sensor A/B normals must be exact negations at the shared contact",
            )
            np.testing.assert_allclose(
                b_separations[j],
                -a_separations[i],
                atol=1e-5,
                err_msg="raw-contact sensor A/B separations must be exact negations at the shared contact",
            )
        assert matched > 0, "the box_a<->box_b contact must appear in both sensors"
        self.finish()


class ArticulationContactsFullCommon(GridTestBase):
    """Verify articulation-link contact sensor identity and ordering.

    The fixture registers every Ant link as a contact-reporting body, then
    creates a torso contact view and checks its resolved sensor paths and
    metadata.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=4.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        from _scenario import get_asset_root

        ant_asset = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), ant_asset)

        # Box on the ground (companion contact sensor).
        box_path = self.env_template_path.AppendChild("box")
        box = self.create_rigid_box(box_path, Transform((0.0, 0.0, 0.5)), Gf.Vec3f(0.3, 0.3, 0.3))
        UsdPhysics.MassAPI(box).GetMassAttr().Set(1.0)

        # Apply contact-report API to every ant link, not just the box.
        # The engine's rigid-contact-sensor resolution recognizes
        # articulation links as valid sensors but requires
        # PhysxContactReportAPI on the prim. The legacy upstream test
        # achieved this via a stage-wide traversal
        # (testContacts.py:482-484); the umbrella scenario needs to
        # opt-in by listing the ant link paths here.
        # Names match the Ant.usda prims: front links are
        # `front_<side>_*`, back links are `<side>_back_*`.
        ant_link_names = (
            "torso",
            "right_back_leg",
            "left_back_leg",
            "front_right_leg",
            "front_left_leg",
            "right_back_foot",
            "left_back_foot",
            "front_right_foot",
            "front_left_foot",
        )
        self._contact_body_paths = [box_path] + [actor_path.AppendChild(link_name) for link_name in ant_link_names]
        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create articulation-link and companion box contact views.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        # Articulation link contact view — exercises legacy
        # `sensor_names` / `link_indices` metadata. A missing accessor remains
        # a test failure.
        self.ant_link_contacts = sim.create_rigid_contact_view(
            "/envs/*/ant/torso",
        )
        self.cubes_contact = sim.create_rigid_contact_view("/envs/*/box")

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Validate torso sensor paths after the articulation settles.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno >= 60:
            # Exercise `sensor_names` on an articulation-link contact
            # view. The engine resolves articulation links as valid
            # contact sensors provided the link prim carries
            # PhysxContactReportAPI -- applied to every ant link via
            # _contact_body_paths above. The legacy variant also
            # asserted on `shared_metatype.link_indices`, but that is a
            # per-articulation metatype that the umbrella's rigid-
            # contact-view adapter does not expose today (it would need
            # to resolve the parent articulation off each sensor and
            # build the link-name->index dict). For now we verify the
            # piece that the engine binding directly supports.
            sensor_names = self.ant_link_contacts.get_metadata("sensor-names")
            # Exact identity/order/count: the "/envs/*/ant/torso" view resolves to
            # one torso sensor per env, in ascending env-clone order.
            expected = [f"/envs/env{i}/ant/torso" for i in range(self.num_envs)]
            assert list(sensor_names) == expected
            # The num-sensors metadata agrees, and the sensor_names attribute
            # mirrors the sensor-names metadata.
            assert self.ant_link_contacts.get_metadata("num-sensors") == len(expected)
            assert list(self.ant_link_contacts.sensor_names) == expected
            self.finish()


class ArticulationContactsCommon(GridTestBase):
    """Verify contact forces for boxes accompanying articulation scenarios.

    Each box starts on the ground, and its steady net contact force must equal
    its weight.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=4, env_spacing=4.0)
        super().__init__(test_case, grid_params, SimParams(), device_params)

        box_path = self.env_template_path.AppendChild("box")
        box = self.create_rigid_box(box_path, Transform((0.0, 0.0, 0.15)), Gf.Vec3f(0.3, 0.3, 0.3))
        UsdPhysics.MassAPI(box).GetMassAttr().Set(1.0)

        self._contact_body_paths = [box_path]

        self._apply_engine_specifics()

    def _apply_engine_specifics(self) -> None:
        """Apply engine-specific contact-reporting configuration."""

    def on_start(self, sim: object) -> None:
        """Create the box contact view and reset settling state.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.cubes_contact = sim.create_rigid_contact_view("/envs/*/box")
        self.check_rigid_contact_view(self.cubes_contact, self.num_envs, 0)
        self._settling = _PerSensorSettlingTracker(self.cubes_contact.get_metadata("num-sensors"))

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Wait for a stable ground reaction equal to each box's weight.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        num_sensors = self.cubes_contact.get_metadata("num-sensors")
        net_forces = self.cubes_contact.get_data("net-contact-forces").numpy().reshape(num_sensors, 3)
        expected = np.array([0.0, 0.0, 9.81])
        self._settling.update(np.all(np.isclose(net_forces, expected, rtol=1e-2, atol=1e-2), axis=1))
        if self._settling.is_complete:
            self.finish()
        elif stepno + 1 >= self.maxsteps:
            details = self._settling.format_unsettled(net_forces)
            assert False, f"box net contact force never reached weight m*g: {details}"
            self.finish()


class ContactCapacityResizeCommon(RawContactDataCommon):
    """Scenario that verifies a contact view adopts the extent the caller's buffers declare.

    Reuses the fixed-overlap scene so contacts are deterministic, then reads ``raw-contact-data``
    through buffers of a different first axis. The view is expected to rebuild its binding at that
    size -- larger or smaller -- and to keep its declared shape hints in step, because the frontend
    sizes its own allocations from those hints and a stale one would undo the caller's choice on the
    next read.

    Args:
        test_case: Test object that owns scenario assertions.
        device_params: Simulation and tensor device selection.

    """

    #: Capacity the view is constructed with.
    initial_capacity = 40
    #: Capacity requested by a larger set of caller buffers.
    grown_capacity = 200
    #: Capacity requested afterwards, to prove the extent shrinks as well as grows.
    shrunk_capacity = 80

    #: Outputs of ``raw-contact-data`` carrying one row per contact record. The remaining two are
    #: sized by sensor count and never move.
    _payload_slots = (0, 1, 2, 3, 6)

    def on_start(self, sim: object) -> None:
        """Create a single contact view at the initial capacity.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        # Filters are supplied so `contact-data` and `friction-data` register too: all three reads share one
        # capacity, and their declared shapes are rewritten together on a resize.
        self.contacts = sim.create_rigid_contact_view(
            "/envs/*/top_box",
            filter_patterns=["/envs/*/bottom_box"],
            max_contact_data_count=self.initial_capacity,
        )

    def _declared_capacity(self, impl: str = "raw-contact-data") -> int:
        """Read the capacity a read operation currently declares through its shape hints.

        Args:
            impl: Registered multi-buffer read operation to inspect.

        Returns:
            First axis of that operation's first payload output.

        """
        specs = self.contacts.get_impl_spec_multi(impl, t.ImplKind.Get)
        return int(specs[0].shape_hint[0])

    def _assert_all_reads_declare(self, capacity: int, label: str) -> None:
        """Check every capacity-dependent read declares the same extent.

        The three reads share one binding, so a resize that updated only the operation it was triggered
        from would leave the others describing buffers that no longer exist.

        Args:
            capacity: Extent every read is expected to declare.
            label: Phase name used in assertion messages.

        """
        for impl in ("raw-contact-data", "contact-data", "friction-data"):
            assert (
                self._declared_capacity(impl) == capacity
            ), f"{label}: {impl} declares {self._declared_capacity(impl)}, expected {capacity}"

    def _buffers_at(self, capacity: int, reference: list) -> list:
        """Allocate a full output set whose payload slots declare a given capacity.

        Args:
            capacity: Requested number of contact records.
            reference: Result of a prior read, used for dtypes and the metadata shapes.

        Returns:
            One buffer per registered output.

        """
        buffers = []
        for slot, tensor in enumerate(reference):
            shape = list(tensor.shape)
            if slot in self._payload_slots:
                shape[0] = capacity
            buffers.append(wp.zeros(tuple(shape), dtype=tensor.dtype, device=self.wp_device))
        return buffers

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Grow, then shrink, the contact extent through the caller's buffers.

        Each resize replaces the underlying binding, and a fresh binding holds no contact data until a
        step populates it. The checks are therefore spread across steps: resize on one, verify the data
        came back on the next. That gap is the engine's contract, not a defect.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if self.finished:
            return

        view = self.contacts
        if stepno == 0:
            assert self._declared_capacity() == self.initial_capacity
            self._reference = view.get_data_multi("raw-contact-data")
            assert int(self._reference[0].shape[0]) == self.initial_capacity
            # The overlap is fixed, so a read that reports nothing would mean the resize is being
            # exercised against an empty binding and proves little.
            assert (self._reference[4].numpy().flatten() > 0).all(), "expected every sensor to report contacts"
            view.get_data_multi("raw-contact-data", None, self._buffers_at(self.grown_capacity, self._reference))
            self._assert_all_reads_declare(self.grown_capacity, "grown")
            return

        if stepno == 1:
            # The extent sticks: a read with no `out` uses the adopted capacity rather than reverting
            # to the one the view was built with, and the rebuilt binding reports real data again.
            grown_read = view.get_data_multi("raw-contact-data")
            assert int(grown_read[0].shape[0]) == self.grown_capacity
            assert (grown_read[4].numpy().flatten() > 0).all(), "contacts lost after growing the binding"
            view.get_data_multi("raw-contact-data", None, self._buffers_at(self.shrunk_capacity, self._reference))
            self._assert_all_reads_declare(self.shrunk_capacity, "shrunk")
            return

        final_read = view.get_data_multi("raw-contact-data")
        assert int(final_read[0].shape[0]) == self.shrunk_capacity
        assert (final_read[4].numpy().flatten() > 0).all(), "contacts lost after shrinking the binding"
        self.finish()
