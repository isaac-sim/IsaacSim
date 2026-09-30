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

"""Verify the ``get_data`` and ``set_data`` tensor contracts.

What this file checks
---------------------

For each way ``get_data`` / ``set_data`` can be called, the data must come back
(or be written) with the right size and on the right device:

* full get - read every entity; the engine allocates the result buffer.
* external-buffer get - read every entity into a buffer the caller supplies (``out=``).
* indexed get - read a K-entity subset; the result is K-sized, not the full N.
* indexed external get - an indexed read into a caller-supplied ``out`` buffer.
* full set - write every entity.
* subset set - write a K-entity subset, passing a full-size (N) buffer.
* index-size set - write a K-entity subset, passing only a K-sized buffer.
* multi-buffer get / set - impls whose value is several tensors at once
  (e.g. raw contact data; apply force + torque + position).

Device: every read asserts the result is on the simulation's device (CPU sim ->
host array, GPU sim -> CUDA array, with no hidden host copy); every write feeds
data already on that device.

Each case runs for every entity (articulation here; rigid-body and rigid-contact
in the sibling modules) and for both runtime state (positions, velocities) and
parameters (stiffness, mass).

Three non-tensor parts of the contract are also checked: a new impl can be
registered from Python with no framework change (extensibility); ``get_metadata``
returns the right non-tensor values (counts, names, paths); and an impl rejects
the wrong direction (set on a read-only impl, get on a write-only impl).

This module holds the shared cell checks + the articulation fixture/scenarios;
``get_set_contract_rigid_body`` and ``get_set_contract_contact`` reuse the checks.
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

from _scenario import (  # noqa: E402
    DeviceParams,
    GridParams,
    GridTestBase,
    SimParams,
    Transform,
    get_asset_root,
)

_INDICES = [0, 2]  # entity rows used by the indexed shapes (subset size K = len(_INDICES))


# ---------------------------------------------------------------------------
# Cell checks - each takes the scenario instance plus (impl, per-entity dim).
# They read scn.view / scn.num_envs / scn.test_case / scn.is_gpu.
# ---------------------------------------------------------------------------


def _assert_device(scn: _ContractMixin, a: wp.array, what: str) -> None:
    """Check that a result resides on the scenario's tensor device.

    Args:
        scn: Contract scenario that selected the expected device.
        a: Warp result array.
        what: Operation description included in an assertion failure.

    """
    is_cuda = bool(getattr(a.device, "is_cuda", False))
    assert is_cuda == scn.is_gpu, f"{what}: result on wrong device (is_cuda={is_cuda}, expected GPU={scn.is_gpu})"


def check_full_get(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check a full read into engine-allocated storage.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per entity.

    """
    a = scn.view.get_data(impl)
    assert a.size == scn.num_envs * dim, f"{impl} full get: wrong element count"
    _assert_device(scn, a, f"{impl} full get")


def check_external_get(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check a full read into caller-provided storage.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per entity.

    """
    out = wp.zeros((scn.num_envs, dim), dtype=wp.float32, device=scn.wp_device)
    r = scn.view.get_data(impl, out=out)
    assert r is not None, f"{impl} external get: returned None"
    assert r.size == scn.num_envs * dim, f"{impl} external get: wrong element count"
    assert (
        hasattr(r, "ptr") and hasattr(out, "ptr") and r.ptr == out.ptr
    ), f"{impl} external get: result did not alias the caller-provided `out` buffer"
    _assert_device(scn, r, f"{impl} external get")


def check_indexed_get(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check an indexed read into engine-allocated storage.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per selected entity.

    """
    indices = wp.array(_INDICES, dtype=wp.int32, device=scn.wp_device)
    a = scn.view.get_data(impl, indices)
    assert a.size == len(_INDICES) * dim, f"{impl} indexed get: expected index-size (K*D), not full-size"
    _assert_device(scn, a, f"{impl} indexed get")
    # int64 indices are rejected, not silently coerced (the int32 index contract) --
    # uniform with the set path (see check_index_size_set).
    with pytest.raises(ValueError, match="int32"):
        scn.view.get_data(impl, wp.array(_INDICES, dtype=wp.int64, device=scn.wp_device))


def check_indexed_external_get(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check an indexed read into caller-provided storage.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per selected entity.

    """
    indices = wp.array(_INDICES, dtype=wp.int32, device=scn.wp_device)
    out = wp.zeros((len(_INDICES), dim), dtype=wp.float32, device=scn.wp_device)
    r = scn.view.get_data(impl, indices, out)
    assert r.size == len(_INDICES) * dim, f"{impl} indexed external get: wrong element count"
    assert (
        hasattr(r, "ptr") and r.ptr == out.ptr
    ), f"{impl} indexed external get: result did not alias the caller-provided `out` buffer"
    _assert_device(scn, r, f"{impl} indexed external get")


def check_full_set(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check a full write and readback.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per entity.

    """
    cur = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim).copy()
    target = (cur + 0.05).astype(np.float32)
    scn.view.set_data(impl, wp.from_numpy(target, dtype=wp.float32, device=scn.wp_device))
    rb = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim)
    assert np.allclose(
        rb, target, atol=1e-2
    ), f"{impl} full set did not round-trip (max diff {np.max(np.abs(rb - target)):.4f})"


def check_subset_set(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check an indexed write from a full-size source buffer.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per entity.

    """
    cur = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim).copy()
    full = cur.copy()
    full[_INDICES] += 0.1
    indices = wp.array(_INDICES, dtype=wp.int32, device=scn.wp_device)
    scn.view.set_data(impl, wp.from_numpy(full.astype(np.float32), dtype=wp.float32, device=scn.wp_device), indices)
    rb = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim)
    others = [r for r in range(scn.num_envs) if r not in _INDICES]
    assert np.all(np.abs(rb[_INDICES] - cur[_INDICES]) > 0.05), f"{impl} subset set: selected rows did not change"
    assert np.all(np.abs(rb[others] - cur[others]) < 1e-3), f"{impl} subset set: non-selected rows changed"


def check_index_size_set(scn: _ContractMixin, impl: str, dim: int) -> None:
    """Check an indexed write from a compact source buffer.

    Args:
        scn: Contract scenario and entity view.
        impl: Registered tensor implementation name.
        dim: Number of scalar values per selected entity.

    """
    cur = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim).copy()
    k_buffer = (cur[_INDICES] + 0.2).astype(np.float32)  # K x D only; no full-size buffer
    indices = wp.array(_INDICES, dtype=wp.int32, device=scn.wp_device)
    scn.view.set_data(impl, wp.from_numpy(k_buffer, dtype=wp.float32, device=scn.wp_device), indices)
    rb = scn.view.get_data(impl).numpy().reshape(scn.num_envs, dim)
    others = [r for r in range(scn.num_envs) if r not in _INDICES]
    assert np.allclose(rb[_INDICES], cur[_INDICES] + 0.2, atol=1e-2), f"{impl} index-size set: selected rows wrong"
    assert np.all(np.abs(rb[others] - cur[others]) < 1e-3), f"{impl} index-size set: non-selected rows changed"
    # int64 indices are rejected, not silently coerced (the int32 index contract).
    with pytest.raises(ValueError, match="int32"):
        scn.view.set_data(
            impl,
            wp.from_numpy(k_buffer, dtype=wp.float32, device=scn.wp_device),
            wp.array(_INDICES, dtype=wp.int64, device=scn.wp_device),
        )


# ---------------------------------------------------------------------------
# Scenario plumbing - a per-entity fixture builds the scene + view; each leaf
# scenario overrides check_cell to run one shared check_* helper after warmup.
# ---------------------------------------------------------------------------


class _ContractMixin:
    """Base class for every contract scenario: runs one access-shape check, once.

    It carries the per-scenario behavior (the warmup below) but not the scene.
    The per-entity fixtures subclass it together with ``GridTestBase`` to add the
    scene and ``self.view``: ``_ArticulationContractBase`` (this module),
    ``_RigidBodyContractBase`` and ``_ContactContractBase`` (the sibling modules).
    Each leaf scenario overrides ``check_cell`` to run one shape against the view,
    typically a call to one of the shared ``check_*`` helpers above.

    The check runs after a brief warmup, then ``finish()`` ends the run; the
    warmup is required because PhysX DirectGPU read buffers only become valid
    after at least one simulation step.
    """

    _WARMUP_STEPS = 2  # GPU DirectGPU buffers need a sim step before reads are valid

    def check_cell(self) -> None:
        """Run the access-shape assertion implemented by a leaf scenario."""
        raise NotImplementedError

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Run the contract cell after backend buffer warmup.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno < self._WARMUP_STEPS:
            return
        self.check_cell()
        self.finish()


class _ArticulationContractBase(_ContractMixin, GridTestBase):
    """Contract scene with one gravity-free Ant articulation per environment.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    NUM_DOFS = 8  # Ant
    NUM_LINKS = 9  # Ant: torso + 4 legs x 2 links

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        grid_params = GridParams(num_envs=8, env_spacing=2.5)
        sim_params = SimParams()
        sim_params.gravity_mag = 0.0  # keep state still so set/get round-trips are exact
        super().__init__(test_case, grid_params, sim_params, device_params)
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        actor_path = self.env_template_path.AppendChild("ant")
        self.create_actor_from_asset(actor_path, Transform((0.0, 0.0, 1.0)), asset_path)
        self.is_gpu = bool(device_params.use_gpu_pipeline)

    def on_start(self, sim: object) -> None:
        """Create the articulation view used by the contract.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.sim = sim
        self.view = sim.create_articulation_view("/envs/*/ant/torso")


class ExtensibilityCommon(_ArticulationContractBase):
    """Verify dynamic implementation registration and pattern-list behavior."""

    def check_cell(self) -> None:
        """Check custom reads, empty patterns, ordering, and de-duplication."""
        from isaacsim.physics.manager.impl.tensors import ImplKind

        assert list(self.view.paths) == ["/envs/*/ant/torso"]
        empty = self.sim.create_articulation_view("/does/not/match")
        assert empty.count == 0
        assert list(empty.paths) == ["/does/not/match"]
        # A list of patterns that all match nothing must yield a 0-entity view -- it
        # must NOT fall back to a match-all pattern and capture the whole scene.
        empty_multi = self.sim.create_articulation_view(["/does/not/match", "/also/nope"])
        assert empty_multi.count == 0, "all-non-matching pattern list must yield a 0-entity view"
        assert "" not in list(
            empty_multi.paths
        ), "all-non-matching pattern list must not degrade to an empty-string path"

        # An empty pattern list has nothing to resolve; it must be rejected loudly
        # rather than degrade to an ambiguous empty-string pattern.
        with pytest.raises(ValueError):
            self.sim.create_articulation_view([])
        with pytest.raises(ValueError):
            self.sim.create_rigid_body_view([])

        # An all-miss multi-pattern list must behave exactly like a single
        # non-matching pattern -- same registered impls, same get_data result --
        # for both articulation and rigid-body views. ovphysx previously diverged:
        # a single non-matching pattern created valid zero-sized bindings, while an
        # all-miss list sent an empty prim-path list that ovphysx 0.5.1 rejected,
        # leaving has_impl false and get_data throwing.
        def _behavior(view: object, impl: str) -> tuple[bool, bool, bool | None]:
            has_get = view.has_impl(impl, ImplKind.Get)
            has_set = view.has_impl(impl, ImplKind.Set)
            got_empty = (view.get_data(impl) is None) if has_get else None
            return (has_get, has_set, got_empty)

        single_rb = self.sim.create_rigid_body_view("/does/not/match")
        multi_rb = self.sim.create_rigid_body_view(["/does/not/match", "/also/nope"])
        assert multi_rb.count == 0, "all-non-matching rigid-body list must yield a 0-entity view"
        for one, many, impl in ((empty, empty_multi, "dof-positions"), (single_rb, multi_rb, "velocities")):
            assert _behavior(many, impl) == _behavior(
                one, impl
            ), f"all-miss list must match a single non-matching pattern for {impl} (no divergent bindings)"

        # A multi-pattern list resolves in caller order with first-wins de-duplication
        # -- the ordering/dedup contract a joined-string pattern cannot guarantee.
        all_paths = list(self.view.resolved_prim_paths)
        if len(all_paths) >= 3:
            reordered = [all_paths[2], all_paths[0], all_paths[1]]
            assert (
                list(self.sim.create_articulation_view(reordered).resolved_prim_paths) == reordered
            ), "multi-pattern list must resolve in caller order"
            # An explicit path followed by a superset glob keeps the explicit one first
            # and does not repeat it.
            dedup = list(self.sim.create_articulation_view([all_paths[0], "/envs/*/ant/torso"]).resolved_prim_paths)
            assert dedup[0] == all_paths[0], "explicit path listed first must stay first"
            assert len(dedup) == len(set(dedup)), "resolved paths must be first-wins de-duplicated"
            assert len(dedup) == len(all_paths), "dedup must keep every entity exactly once"

        # List support is uniform across view types, not just articulation/rigid-body:
        # the deformable views accept a pattern list too (an all-miss list yields a
        # 0-entity view rather than raising "no matching overload").
        for make in (
            self.sim.create_volume_deformable_body_view,
            self.sim.create_surface_deformable_body_view,
            self.sim.create_deformable_material_view,
        ):
            assert (
                make(["/does/not/match", "/also/nope"]).count == 0
            ), "deformable view must accept a pattern list and yield a 0-entity all-miss view"
        n = self.num_envs
        sentinel = np.arange(n * 3, dtype=np.float32).reshape(n, 3)
        payload = wp.from_numpy(sentinel, dtype=wp.float32, device=self.wp_device)

        def _probe_get(indices: object | None, out: object | None) -> wp.array:
            del indices, out
            return payload

        self.view._register_impl("contract-probe-impl", ImplKind.Get, _probe_get)
        r = self.view.get_data("contract-probe-impl")
        assert r is not None, "extensibility: custom impl returned None"
        assert np.allclose(r.numpy().reshape(n, 3), sentinel), "extensibility: custom impl data mismatch"


class MetadataCommon(_ArticulationContractBase):
    """get_metadata returns non-tensor data with the right type and value."""

    def check_cell(self) -> None:
        """Verify articulation count, name, path, topology, and root metadata."""
        tc = self.test_case
        num_dofs = self.view.get_metadata("num-dofs")
        assert isinstance(num_dofs, int), f"metadata num-dofs: expected int, got {type(num_dofs).__name__}"
        assert num_dofs == self.NUM_DOFS, "metadata num-dofs: wrong value"
        prim_paths = self.view.get_metadata("prim-paths")
        assert isinstance(prim_paths, list), f"metadata prim-paths: expected list, got {type(prim_paths).__name__}"
        assert len(prim_paths) == self.num_envs, "metadata prim-paths: wrong count"
        num_links = self.view.get_metadata("num-links")
        assert isinstance(num_links, int), f"metadata num-links: expected int, got {type(num_links).__name__}"
        assert num_links == self.NUM_LINKS, "metadata num-links: wrong value"
        num_joints = self.view.get_metadata("num-joints")
        assert isinstance(num_joints, int), f"metadata num-joints: expected int, got {type(num_joints).__name__}"
        assert num_joints > 0, "metadata num-joints: expected a positive joint count"
        dof_names = self.view.get_metadata("dof-names")
        assert isinstance(dof_names, list), f"metadata dof-names: expected list, got {type(dof_names).__name__}"
        assert len(dof_names) == num_dofs, "metadata dof-names: one name per DOF"
        assert all(isinstance(n, str) for n in dof_names), "metadata dof-names: entries must be str"
        is_fixed_base = self.view.get_metadata("is-fixed-base")
        assert is_fixed_base is not None, "metadata is-fixed-base: must be registered (not None)"
        assert is_fixed_base in (True, False), "metadata is-fixed-base: expected a boolean flag"


class ReadOnlyRejectsSetCommon(_ArticulationContractBase):
    """The backed read-only inverse-mass datum rejects set_data."""

    def check_cell(self) -> None:
        """Verify read support and rejection of writing inverse masses."""
        tc = self.test_case
        assert self.view.has_impl("inv-masses", t.ImplKind.Get)
        assert not (self.view.has_impl("inv-masses", t.ImplKind.Set))
        value = self.view.get_data("inv-masses")
        assert value is not None
        data = wp.zeros(value.shape, dtype=value.dtype, device=self.wp_device)
        with pytest.raises(Exception, match="set-implementation 'inv-masses' not registered"):
            self.view.set_data("inv-masses", data)


# --- articulation cell scenarios: dof-positions (runtime) + dof-stiffnesses
# (parameter), one class per access shape, each running the matching check. ---


class DofPositionsFullGetCommon(_ArticulationContractBase):
    """Verify a full engine-allocated degree-of-freedom position read."""

    def check_cell(self) -> None:
        """Run the full position-read contract."""
        check_full_get(self, "dof-positions", 8)


class DofPositionsExternalGetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom position read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer position-read contract."""
        check_external_get(self, "dof-positions", 8)


class DofPositionsIndexedGetCommon(_ArticulationContractBase):
    """Verify an indexed engine-allocated degree-of-freedom position read."""

    def check_cell(self) -> None:
        """Run the indexed position-read contract."""
        check_indexed_get(self, "dof-positions", 8)


class DofPositionsIndexedExternalGetCommon(_ArticulationContractBase):
    """Verify an indexed degree-of-freedom position read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer position-read contract."""
        check_indexed_external_get(self, "dof-positions", 8)


class DofPositionsFullSetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom position write."""

    def check_cell(self) -> None:
        """Run the full position-write contract."""
        check_full_set(self, "dof-positions", 8)


class DofPositionsSubsetSetCommon(_ArticulationContractBase):
    """Verify a full-size position write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset position-write contract."""
        check_subset_set(self, "dof-positions", 8)


class DofPositionsIndexSizeSetCommon(_ArticulationContractBase):
    """Verify a compact position write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed position-write contract."""
        check_index_size_set(self, "dof-positions", 8)


class DofStiffnessesFullGetCommon(_ArticulationContractBase):
    """Verify a full engine-allocated degree-of-freedom stiffness read."""

    def check_cell(self) -> None:
        """Run the full stiffness-read contract."""
        check_full_get(self, "dof-stiffnesses", 8)


class DofStiffnessesExternalGetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom stiffness read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer stiffness-read contract."""
        check_external_get(self, "dof-stiffnesses", 8)


class DofStiffnessesIndexedGetCommon(_ArticulationContractBase):
    """Verify an indexed engine-allocated degree-of-freedom stiffness read."""

    def check_cell(self) -> None:
        """Run the indexed stiffness-read contract."""
        check_indexed_get(self, "dof-stiffnesses", 8)


class DofStiffnessesIndexedExternalGetCommon(_ArticulationContractBase):
    """Verify an indexed degree-of-freedom stiffness read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer stiffness-read contract."""
        check_indexed_external_get(self, "dof-stiffnesses", 8)


class DofStiffnessesFullSetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom stiffness write."""

    def check_cell(self) -> None:
        """Run the full stiffness-write contract."""
        check_full_set(self, "dof-stiffnesses", 8)


class DofStiffnessesSubsetSetCommon(_ArticulationContractBase):
    """Verify a full-size stiffness write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset stiffness-write contract."""
        check_subset_set(self, "dof-stiffnesses", 8)


class DofStiffnessesIndexSizeSetCommon(_ArticulationContractBase):
    """Verify a compact stiffness write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed stiffness-write contract."""
        check_index_size_set(self, "dof-stiffnesses", 8)


class DofDampingsFullGetCommon(_ArticulationContractBase):
    """Verify a full engine-allocated degree-of-freedom damping read."""

    def check_cell(self) -> None:
        """Run the full damping-read contract."""
        check_full_get(self, "dof-dampings", 8)


class DofDampingsExternalGetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom damping read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer damping-read contract."""
        check_external_get(self, "dof-dampings", 8)


class DofDampingsIndexedGetCommon(_ArticulationContractBase):
    """Verify an indexed engine-allocated degree-of-freedom damping read."""

    def check_cell(self) -> None:
        """Run the indexed damping-read contract."""
        check_indexed_get(self, "dof-dampings", 8)


class DofDampingsIndexedExternalGetCommon(_ArticulationContractBase):
    """Verify an indexed degree-of-freedom damping read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer damping-read contract."""
        check_indexed_external_get(self, "dof-dampings", 8)


class DofDampingsFullSetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom damping write."""

    def check_cell(self) -> None:
        """Run the full damping-write contract."""
        check_full_set(self, "dof-dampings", 8)


class DofDampingsSubsetSetCommon(_ArticulationContractBase):
    """Verify a full-size damping write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset damping-write contract."""
        check_subset_set(self, "dof-dampings", 8)


class DofDampingsIndexSizeSetCommon(_ArticulationContractBase):
    """Verify a compact damping write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed damping-write contract."""
        check_index_size_set(self, "dof-dampings", 8)


class DofArmaturesFullGetCommon(_ArticulationContractBase):
    """Verify a full engine-allocated degree-of-freedom armature read."""

    def check_cell(self) -> None:
        """Run the full armature-read contract."""
        check_full_get(self, "dof-armatures", 8)


class DofArmaturesExternalGetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom armature read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer armature-read contract."""
        check_external_get(self, "dof-armatures", 8)


class DofArmaturesIndexedGetCommon(_ArticulationContractBase):
    """Verify an indexed engine-allocated degree-of-freedom armature read."""

    def check_cell(self) -> None:
        """Run the indexed armature-read contract."""
        check_indexed_get(self, "dof-armatures", 8)


class DofArmaturesIndexedExternalGetCommon(_ArticulationContractBase):
    """Verify an indexed degree-of-freedom armature read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer armature-read contract."""
        check_indexed_external_get(self, "dof-armatures", 8)


class DofArmaturesFullSetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom armature write."""

    def check_cell(self) -> None:
        """Run the full armature-write contract."""
        check_full_set(self, "dof-armatures", 8)


class DofArmaturesSubsetSetCommon(_ArticulationContractBase):
    """Verify a full-size armature write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset armature-write contract."""
        check_subset_set(self, "dof-armatures", 8)


class DofArmaturesIndexSizeSetCommon(_ArticulationContractBase):
    """Verify a compact armature write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed armature-write contract."""
        check_index_size_set(self, "dof-armatures", 8)


class DofMaxForcesFullGetCommon(_ArticulationContractBase):
    """Verify a full engine-allocated degree-of-freedom force-limit read."""

    def check_cell(self) -> None:
        """Run the full force-limit-read contract."""
        check_full_get(self, "dof-max-forces", 8)


class DofMaxForcesExternalGetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom force-limit read into caller storage."""

    def check_cell(self) -> None:
        """Run the external-buffer force-limit-read contract."""
        check_external_get(self, "dof-max-forces", 8)


class DofMaxForcesIndexedGetCommon(_ArticulationContractBase):
    """Verify an indexed engine-allocated degree-of-freedom force-limit read."""

    def check_cell(self) -> None:
        """Run the indexed force-limit-read contract."""
        check_indexed_get(self, "dof-max-forces", 8)


class DofMaxForcesIndexedExternalGetCommon(_ArticulationContractBase):
    """Verify an indexed degree-of-freedom force-limit read into caller storage."""

    def check_cell(self) -> None:
        """Run the indexed external-buffer force-limit-read contract."""
        check_indexed_external_get(self, "dof-max-forces", 8)


class DofMaxForcesFullSetCommon(_ArticulationContractBase):
    """Verify a full degree-of-freedom force-limit write."""

    def check_cell(self) -> None:
        """Run the full force-limit-write contract."""
        check_full_set(self, "dof-max-forces", 8)


class DofMaxForcesSubsetSetCommon(_ArticulationContractBase):
    """Verify a full-size force-limit write with selected indices."""

    def check_cell(self) -> None:
        """Run the subset force-limit-write contract."""
        check_subset_set(self, "dof-max-forces", 8)


class DofMaxForcesIndexSizeSetCommon(_ArticulationContractBase):
    """Verify a compact force-limit write for selected indices."""

    def check_cell(self) -> None:
        """Run the compact indexed force-limit-write contract."""
        check_index_size_set(self, "dof-max-forces", 8)
