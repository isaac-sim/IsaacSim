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

"""Provide shared test fixtures."""

from collections.abc import Iterator

import isaacsim.physics.manager.impl.tensors as tensors
import numpy as np
import pytest
import warp as wp

_L = 12  # links
_D = 7  # DOFs
_J = 9  # joints
_T = 3  # fixed / spatial tendons
_S = 4  # shapes per articulation
_R = (_L - 1) * 6  # Jacobian rows (fixed base)
_C = _D  # Jacobian cols (fixed base)
_M = _D  # generalized coordinates (fixed base)

_DOF_NAMES = [f"joint_{i}" for i in range(_D)]
_LINK_NAMES = [f"link_{i}" for i in range(_L)]
_JOINT_NAMES = [f"joint_{i}" for i in range(_J)]

_ARTICULATION_IMPLEMENTATIONS: list[dict] = [
    {"name": "dof-positions", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-velocities", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-position-targets", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-velocity-targets", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-actuation-forces", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-limits", "dtype": wp.float32, "shape": [_D, 2], "get": True, "set": True},
    {"name": "dof-stiffnesses", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-dampings", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-armatures", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-max-forces", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-max-velocities", "dtype": wp.float32, "shape": [_D], "get": True, "set": True},
    {"name": "dof-friction-properties", "dtype": wp.float32, "shape": [_D, 3], "get": True, "set": True},
    {"name": "root-transforms", "dtype": wp.float32, "shape": [7], "get": True, "set": True},
    {"name": "root-velocities", "dtype": wp.float32, "shape": [6], "get": True, "set": True},
    {"name": "link-transforms", "dtype": wp.float32, "shape": [_L, 7], "get": True, "set": False},
    {"name": "link-velocities", "dtype": wp.float32, "shape": [_L, 6], "get": True, "set": False},
    {"name": "link-accelerations", "dtype": wp.float32, "shape": [_L, 6], "get": True, "set": False},
    {"name": "masses", "dtype": wp.float32, "shape": [_L], "get": True, "set": True},
    {"name": "inv-masses", "dtype": wp.float32, "shape": [_L], "get": True, "set": False},
    {"name": "coms", "dtype": wp.float32, "shape": [_L, 7], "get": True, "set": True},
    {"name": "inertias", "dtype": wp.float32, "shape": [_L, 9], "get": True, "set": True},
    {"name": "inv-inertias", "dtype": wp.float32, "shape": [_L, 9], "get": True, "set": False},
    {"name": "disable-gravities", "dtype": wp.uint8, "shape": [_L], "get": True, "set": True},
    {"name": "jacobians", "dtype": wp.float32, "shape": [_R, _C], "get": True, "set": False},
    {"name": "generalized-mass-matrices", "dtype": wp.float32, "shape": [_M, _M], "get": True, "set": False},
    {
        "name": "coriolis-and-centrifugal-compensation-forces",
        "dtype": wp.float32,
        "shape": [_M],
        "get": True,
        "set": False,
    },
    {"name": "gravity-compensation-forces", "dtype": wp.float32, "shape": [_M], "get": True, "set": False},
    {"name": "dof-projected-joint-forces", "dtype": wp.float32, "shape": [_D], "get": True, "set": False},
    {"name": "link-incoming-joint-force", "dtype": wp.float32, "shape": [_L, 6], "get": True, "set": False},
    {"name": "material-properties", "dtype": wp.float32, "shape": [_S, 3], "get": True, "set": True},
    {"name": "contact-offsets", "dtype": wp.float32, "shape": [_S], "get": True, "set": True},
    {"name": "rest-offsets", "dtype": wp.float32, "shape": [_S], "get": True, "set": True},
    {
        "name": "apply-forces-and-torques-at-position",
        "dtype": wp.float32,
        "shape": [_L, 9],
        "get": False,
        "set": True,
    },
    {"name": "fixed-tendon-stiffnesses", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "fixed-tendon-dampings", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "fixed-tendon-limit-stiffnesses", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "fixed-tendon-limits", "dtype": wp.float32, "shape": [_T, 2], "get": True, "set": True},
    {"name": "fixed-tendon-rest-lengths", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "fixed-tendon-offsets", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "spatial-tendon-stiffnesses", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {"name": "spatial-tendon-dampings", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
    {
        "name": "spatial-tendon-limit-stiffnesses",
        "dtype": wp.float32,
        "shape": [_T],
        "get": True,
        "set": True,
    },
    {"name": "spatial-tendon-offsets", "dtype": wp.float32, "shape": [_T], "get": True, "set": True},
]

_RIGID_BODY_IMPLEMENTATIONS: list[dict] = [
    {"name": "transforms", "dtype": wp.float32, "shape": [7], "get": True, "set": True},
    {"name": "velocities", "dtype": wp.float32, "shape": [6], "get": True, "set": True},
    {"name": "accelerations", "dtype": wp.float32, "shape": [6], "get": True, "set": False},
    {"name": "masses", "dtype": wp.float32, "shape": [], "get": True, "set": True},
    {"name": "inv-masses", "dtype": wp.float32, "shape": [], "get": True, "set": False},
    {"name": "inertias", "dtype": wp.float32, "shape": [9], "get": True, "set": True},
    {"name": "inv-inertias", "dtype": wp.float32, "shape": [9], "get": True, "set": False},
    {"name": "coms", "dtype": wp.float32, "shape": [7], "get": True, "set": True},
    {"name": "forces", "dtype": wp.float32, "shape": [3], "get": False, "set": True},
    {"name": "apply-forces", "dtype": wp.float32, "shape": [3], "get": False, "set": True},
    {"name": "wrenches", "dtype": wp.float32, "shape": [9], "get": False, "set": True},
    {
        "name": "apply-forces-and-torques-at-position",
        "dtype": wp.float32,
        "shape": [9],
        "get": False,
        "set": True,
    },
    {"name": "disable-simulations", "dtype": wp.uint8, "shape": [], "get": True, "set": True},
    {"name": "disable-gravities", "dtype": wp.uint8, "shape": [], "get": True, "set": True},
    {"name": "material-properties", "dtype": wp.float32, "shape": [1, 3], "get": True, "set": True},
    {"name": "contact-offsets", "dtype": wp.float32, "shape": [1], "get": True, "set": True},
    {"name": "rest-offsets", "dtype": wp.float32, "shape": [1], "get": True, "set": True},
]

_TENSOR_DTYPE_TO_NUMPY_DTYPE: dict[object, type] = {
    wp.float32: np.float32,
    wp.uint8: np.uint8,
}


def _assert_set_data(data: object, expected_shape: list[int], expected_dtype: type) -> None:
    array = wp.from_dlpack(data).numpy()
    np.testing.assert_equal(array.dtype, expected_dtype)
    np.testing.assert_array_equal(array, np.ones(expected_shape, dtype=expected_dtype))


class _MockRigidBodyEntityView(tensors.EntityView):
    def __init__(self, paths: list[str]) -> None:
        super().__init__(paths)
        self.count = len(paths)
        self._register_metadata("num-shapes", lambda: 100)
        for impl in _RIGID_BODY_IMPLEMENTATIONS:
            full_shape = [self.count] + impl["shape"]
            numpy_dtype = _TENSOR_DTYPE_TO_NUMPY_DTYPE[impl["dtype"]]
            tensor_spec = tensors.ArrayTensorSpec(dtype=impl["dtype"], shape_hint=full_shape, device="cpu")
            # create a mock implementation for the tensor
            if impl["get"]:
                self._register_impl(
                    impl["name"],
                    tensors.ImplKind.Get,
                    lambda indices, out, shape=full_shape, dtype=numpy_dtype: np.zeros(shape, dtype=dtype),
                    tensor_spec,
                )
            if impl["set"]:
                self._register_impl(
                    impl["name"],
                    tensors.ImplKind.Set,
                    lambda data, indices, shape=full_shape, dtype=numpy_dtype: _assert_set_data(data, shape, dtype),
                    tensor_spec,
                )


class _MockArticulationEntityView(tensors.EntityView):
    def __init__(self, paths: list[str]) -> None:
        super().__init__(paths)
        self.count = len(paths)
        self._register_metadata("num-dofs", lambda: _D)
        self._register_metadata("num-links", lambda: _L)
        self._register_metadata("num-joints", lambda: _J)
        self._register_metadata("num-shapes", lambda: _S)
        self._register_metadata("num-fixed-tendons", lambda: _T)
        self._register_metadata("dof-names", lambda: _DOF_NAMES)
        self._register_metadata("link-names", lambda: _LINK_NAMES)
        self._register_metadata("joint-names", lambda: _JOINT_NAMES)
        for impl in _ARTICULATION_IMPLEMENTATIONS:
            full_shape = [self.count] + impl["shape"]
            numpy_dtype = _TENSOR_DTYPE_TO_NUMPY_DTYPE[impl["dtype"]]
            tensor_spec = tensors.ArrayTensorSpec(dtype=impl["dtype"], shape_hint=full_shape, device="cpu")
            if impl["get"]:
                self._register_impl(
                    impl["name"],
                    tensors.ImplKind.Get,
                    lambda indices, out, shape=full_shape, dtype=numpy_dtype: np.zeros(shape, dtype=dtype),
                    tensor_spec,
                )
            if impl["set"]:
                self._register_impl(
                    impl["name"],
                    tensors.ImplKind.Set,
                    lambda data, indices, shape=full_shape, dtype=numpy_dtype: _assert_set_data(data, shape, dtype),
                    tensor_spec,
                )


@pytest.fixture(scope="function")
def engine() -> Iterator[None]:
    """Fixture to register and unregister mock entity factories for engine 'engine'.

    Yields:
        Control while the mock engine is registered.

    """
    registry = tensors.get_registry()
    # Entity factories take (paths, options); these mocks read no options.
    registry.register_entity("engine", "rigid-body", lambda paths, options=None: _MockRigidBodyEntityView(paths))
    registry.register_entity("engine", "articulation", lambda paths, options=None: _MockArticulationEntityView(paths))
    yield
    registry.unregister_entity("engine", "rigid-body")
    registry.unregister_entity("engine", "articulation")
