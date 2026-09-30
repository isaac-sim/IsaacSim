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

"""Test that the deprecated wrappers keep the scalar-first quaternion contract."""

import inspect
from typing import Any

import numpy as np
import pytest
from isaacsim.core.experimental import objects as deprecated_objects
from isaacsim.core.experimental.prims import GeomPrim, XformPrim
from isaacsim.foundation import objects as foundation_objects
from isaacsim.foundation import prims as foundation_prims
from isaacsim.foundation.objects import Stage

# A unit quaternion whose four components are distinct and non-zero, so that a wrong component
# permutation cannot coincide with the expected result.
ORIENTATION_WXYZ = [[0.4, 0.2, -0.4, 0.8]]
ORIENTATION_XYZW = [[0.2, -0.4, 0.8, 0.4]]

# The public surface of the extension module the deprecated one stands in for.
EXTENSION_EXPORTS = {
    "Camera",
    "Capsule",
    "Cone",
    "Cube",
    "Cylinder",
    "CylinderLight",
    "DiskLight",
    "DistantLight",
    "DomeLight",
    "GroundPlane",
    "Light",
    "Mesh",
    "Plane",
    "RectLight",
    "Shape",
    "Sphere",
    "SphereLight",
    "Stage",
}

# Every deprecated wrapper, paired with the foundation class it must not leak the ordering of.
# ``Stage`` is excluded: it is not transformable, so it has no pose methods and stays a plain alias.
WRAPPERS = [
    (deprecated_objects.Camera, foundation_objects.Camera),
    (deprecated_objects.Capsule, foundation_objects.Capsule),
    (deprecated_objects.Cone, foundation_objects.Cone),
    (deprecated_objects.Cube, foundation_objects.Cube),
    (deprecated_objects.Cylinder, foundation_objects.Cylinder),
    (deprecated_objects.CylinderLight, foundation_objects.CylinderLight),
    (deprecated_objects.DiskLight, foundation_objects.DiskLight),
    (deprecated_objects.DistantLight, foundation_objects.DistantLight),
    (deprecated_objects.DomeLight, foundation_objects.DomeLight),
    (deprecated_objects.GroundPlane, foundation_prims.GroundPlane),
    (deprecated_objects.Light, foundation_objects.Light),
    (deprecated_objects.Mesh, foundation_objects.Mesh),
    (deprecated_objects.Plane, foundation_objects.Plane),
    (deprecated_objects.RectLight, foundation_objects.RectLight),
    (deprecated_objects.Shape, foundation_objects.Shape),
    (deprecated_objects.Sphere, foundation_objects.Sphere),
    (deprecated_objects.SphereLight, foundation_objects.SphereLight),
    (XformPrim, foundation_objects.Xform),
    (GeomPrim, foundation_objects.Xform),
]

POSE_METHODS = ["get_world_poses", "get_local_poses", "set_world_poses", "set_local_poses"]

# ``Shape`` and ``Light`` are abstract bases: the foundation binds no constructor for them, so only their
# concrete subclasses can be instantiated.
ABSTRACT_WRAPPERS = [deprecated_objects.Shape, deprecated_objects.Light]
CONSTRUCTIBLE_WRAPPERS = [wrapper for wrapper, _ in WRAPPERS if wrapper not in ABSTRACT_WRAPPERS]


def _construct(stage: Any, wrapper: Any, path: str, **kwargs: Any) -> Any:
    """Construct a wrapper, meeting the requirements it places on the prim it wraps.

    Args:
        stage: Stage used by the test.
        wrapper: Deprecated wrapper class to construct.
        path: Prim path to wrap.
        **kwargs: Arguments forwarded to the wrapper.

    Returns:
        The resulting value.
    """
    if wrapper is GeomPrim:
        # GeomPrim wraps an existing prim rather than creating one; the others create their own.
        stage.define_prim(path, "Cube")
    # XformPrim and GeomPrim leave the transform ops alone by default, and a freshly created prim has none,
    # so placement needs the canonical ops authored first.
    return wrapper(path, reset_xform_op_properties=True, **kwargs)


def test_the_deprecated_module_exports_the_extension_surface() -> None:
    """The deprecated module stands in for the extension, so it exports exactly the same names."""
    assert set(deprecated_objects.__all__) == EXTENSION_EXPORTS


def test_stage_is_aliased_unchanged() -> None:
    """``Stage`` is not transformable, so it is passed through rather than wrapped."""
    assert deprecated_objects.Stage is foundation_objects.Stage


def test_every_transformable_export_is_wrapped() -> None:
    """No transformable export may reach a caller as a bare foundation class."""
    wrapped = {wrapper.__name__ for wrapper, _ in WRAPPERS}
    for name in deprecated_objects.__all__:
        exported = getattr(deprecated_objects, name)
        if issubclass(exported, foundation_objects.Xform):
            assert name in wrapped, f"{name} is transformable but is not a scalar-first wrapper"


@pytest.mark.parametrize("wrapper, base", WRAPPERS, ids=lambda value: getattr(value, "__name__", value))
def test_the_wrappers_derive_from_their_foundation_counterpart(wrapper: Any, base: Any) -> None:
    """Each deprecated wrapper subclasses the foundation class it replaces.

    Args:
        wrapper: Deprecated wrapper class under test.
        base: Foundation class the wrapper replaces.
    """
    assert issubclass(wrapper, base)


@pytest.mark.parametrize("wrapper", [pair[0] for pair in WRAPPERS], ids=lambda value: value.__name__)
@pytest.mark.parametrize("method_name", POSE_METHODS)
def test_the_pose_methods_hide_the_rotation_format_argument(wrapper: Any, method_name: str) -> None:
    """The deprecated signatures match the experimental ones, which had no rotation format.

    Args:
        wrapper: Deprecated wrapper class under test.
        method_name: Pose method to inspect.
    """
    signature = inspect.signature(getattr(wrapper, method_name))

    assert "rotation_format" not in signature.parameters


@pytest.mark.parametrize("method_name", POSE_METHODS)
def test_the_overrides_are_applied_once_per_hierarchy(method_name: str) -> None:
    """A wrapper whose base is decorated would delegate to an override that rejects the format.

    Args:
        method_name: Pose method supplied by the parameterized test matrix.
    """
    # GeomPrim and XformPrim share no ancestry, and the objects wrappers derive straight from
    # foundation classes, so no decorated class is the base of another.
    decorated = {wrapper for wrapper, _ in WRAPPERS}
    for wrapper in decorated:
        bases = set(inspect.getmro(wrapper)) - {wrapper}

        assert not (bases & decorated)


@pytest.mark.parametrize("wrapper", ABSTRACT_WRAPPERS, ids=lambda value: value.__name__)
def test_the_abstract_wrappers_stay_uninstantiable(wrapper: Any) -> None:
    """``Shape`` and ``Light`` are bases; only their concrete subclasses can be constructed.

    Args:
        wrapper: Abstract deprecated wrapper class under test.
    """
    with pytest.raises(TypeError, match="no constructor defined"):
        wrapper("/World/Prim")


@pytest.mark.parametrize("wrapper", CONSTRUCTIBLE_WRAPPERS, ids=lambda value: value.__name__)
@pytest.mark.parametrize("argument", ["positions", "translations", "orientations", "scales"])
def test_the_constructors_accept_the_placement_arguments(wrapper: Any, argument: str) -> None:
    """The foundation classes dropped these constructor arguments; the wrappers put them back.

    Args:
        wrapper: Constructible deprecated wrapper class under test.
        argument: Placement argument to supply.
    """
    values = {
        "positions": [[0.0, 0.0, 1.0]],
        "translations": [[0.0, 0.0, 1.0]],
        "orientations": ORIENTATION_WXYZ,
        "scales": [[2.0, 2.0, 2.0]],
    }
    stage = Stage("openusd").create_stage()
    try:
        _construct(stage, wrapper, "/World/Prim", **{argument: values[argument]})
    finally:
        stage.close_stage()


def test_the_constructor_rejects_positions_and_translations_together() -> None:
    """The two placement frames stay mutually exclusive."""
    stage = Stage("openusd").create_stage()
    try:
        with pytest.raises(ValueError, match="Specify only one of them"):
            deprecated_objects.Cube("/World/Cube", positions=[[0.0, 0.0, 0.0]], translations=[[0.0, 0.0, 0.0]])
    finally:
        stage.close_stage()


def test_the_constructor_keeps_the_wrapped_class_arguments() -> None:
    """Arguments owned by the wrapped class are forwarded, not swallowed by the added ones."""
    stage = Stage("openusd").create_stage()
    try:
        prims = deprecated_objects.Cube("/World/Cube", sizes=2.0, positions=[[0.0, 0.0, 1.0]])

        assert np.allclose(prims.get_sizes().numpy(), [[2.0]])
        assert np.allclose(prims.get_world_poses()[0].numpy(), [[0.0, 0.0, 1.0]])
    finally:
        stage.close_stage()


def test_a_deprecated_wrapper_round_trips_scalar_first_quaternions() -> None:
    """A wxyz orientation written through the wrapper reads back unchanged."""
    stage = Stage("openusd").create_stage()
    try:
        prims = XformPrim("/World/Xform", reset_xform_op_properties=True)

        prims.set_world_poses(None, ORIENTATION_WXYZ)

        assert np.allclose(prims.get_world_poses()[1].numpy(), ORIENTATION_WXYZ)
        assert np.allclose(prims.get_local_poses()[1].numpy(), ORIENTATION_WXYZ)
    finally:
        stage.close_stage()


def test_the_wrapper_stores_the_foundation_ordering_on_the_stage() -> None:
    """The wxyz argument reaches the stage reordered, so the authored USD matches the foundation API."""
    stage = Stage("openusd").create_stage()
    try:
        prims = XformPrim("/World/Xform", reset_xform_op_properties=True)
        prims.set_world_poses(None, ORIENTATION_WXYZ)

        # Reading through the foundation base bypasses the override.
        _, orientations = foundation_objects.Xform.get_world_poses(prims)

        assert np.allclose(orientations.numpy(), ORIENTATION_XYZW)
    finally:
        stage.close_stage()


@pytest.mark.parametrize("wrapper", CONSTRUCTIBLE_WRAPPERS, ids=lambda value: value.__name__)
def test_the_constructor_applies_scalar_first_orientations(wrapper: Any) -> None:
    """The constructor forwards orientations through the overridden setters, so it is wxyz too.

    Args:
        wrapper: Constructible deprecated wrapper class under test.
    """
    stage = Stage("openusd").create_stage()
    try:
        prims = _construct(stage, wrapper, "/World/Prim", orientations=ORIENTATION_WXYZ)

        assert np.allclose(prims.get_world_poses()[1].numpy(), ORIENTATION_WXYZ)
    finally:
        stage.close_stage()


def test_an_objects_wrapper_round_trips_scalar_first_quaternions() -> None:
    """The objects wrappers carry the same contract as the prim wrappers."""
    stage = Stage("openusd").create_stage()
    try:
        prims = deprecated_objects.Cube("/World/Cube", sizes=1.0)

        prims.set_local_poses(None, ORIENTATION_WXYZ)

        assert np.allclose(prims.get_local_poses()[1].numpy(), ORIENTATION_WXYZ)
        assert np.allclose(foundation_objects.Cube.get_local_poses(prims)[1].numpy(), ORIENTATION_XYZW)
    finally:
        stage.close_stage()
