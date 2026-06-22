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

"""Test suite for rotation conversion utilities across different implementations including numpy, experimental, scipy, and USD to ensure consistency in quaternion and Euler angle calculations."""

import numpy as np
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
import warp as wp
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.utils.transform import (
    euler_angles_to_quaternion,
    quaternion_to_euler_angles,
)
from isaacsim.core.utils.numpy.rotations import (
    euler_angles_to_quats,
    quats_to_euler_angles,
)
from pxr import Gf
from scipy.spatial.transform import Rotation


def create_usd_quat_cube(base_name: str, index: int, quat_wxyz: np.ndarray | wp.array) -> Cube:
    """Create a USD cube with the specified quaternion orientation.

    Creates a cube primitive in the USD stage at /World/{base_name} with a
    translation along the Z-axis based on the index and orientation set by
    the provided quaternion.

    Args:
        base_name: Base name for the cube prim path.
        index: Index used to offset the cube along the Z-axis.
        quat_wxyz: Quaternion in WXYZ format (scalar-first) for orientation.

    Returns:
        The created Cube object.
    """
    if hasattr(quat_wxyz, "numpy"):
        quat_wxyz = quat_wxyz.numpy()
    quat_wxyz = np.array(quat_wxyz)
    cube_path = f"/World/{base_name}"
    return Cube(cube_path, positions=[[0.0, 0.0, float(index)]], orientations=[quat_wxyz])


def compute_numpy_rotation(euler_angle: tuple[float, float, float], extrinsic: bool) -> tuple[np.ndarray, np.ndarray]:
    """Compute quaternion and round-trip Euler angles using numpy-based rotation utilities.

    Converts Euler angles to quaternion using isaacsim.core.utils.numpy.rotations,
    then converts back to Euler angles to verify round-trip consistency.

    Args:
        euler_angle: Input Euler angles as (roll, pitch, yaw) in degrees.
        extrinsic: If True, use extrinsic rotation convention (ZYX order).

    Returns:
        Tuple of (quaternion, euler_angles) where quaternion is in WXYZ format
        and euler_angles are the round-trip converted values.
    """
    euler_np = np.array(euler_angle)
    numpy_quat = euler_angles_to_quats(euler_np, degrees=True, extrinsic=extrinsic, device="cpu")
    numpy_euler = quats_to_euler_angles(numpy_quat, degrees=True, extrinsic=extrinsic, device="cpu")
    return numpy_quat, numpy_euler


def compute_experimental_rotation(
    euler_angle: tuple[float, float, float], extrinsic: bool, device_value: str | None
) -> tuple[wp.array, wp.array]:
    """Compute quaternion and round-trip Euler angles using experimental transform utilities.

    Converts Euler angles to quaternion using isaacsim.core.experimental.utils.transform,
    then converts back to Euler angles to verify round-trip consistency.

    Args:
        euler_angle: Input Euler angles as (roll, pitch, yaw) in degrees.
        extrinsic: If True, use extrinsic rotation convention.
        device_value: Device to perform computation on (e.g., "cpu", "cuda", or None).

    Returns:
        Tuple of (quaternion, euler_angles) where quaternion is in WXYZ format
        and euler_angles are the round-trip converted values.
    """
    exp_quat = euler_angles_to_quaternion(
        euler_angle,
        degrees=True,
        extrinsic=extrinsic,
        device=device_value,
    )
    exp_euler = quaternion_to_euler_angles(
        exp_quat,
        degrees=True,
        extrinsic=extrinsic,
        device=device_value,
    )
    return exp_quat, exp_euler


def compute_scipy_rotation(euler_angle: tuple[float, float, float], extrinsic: bool) -> tuple[np.ndarray, np.ndarray]:
    """Compute quaternion and round-trip Euler angles using scipy's Rotation class.

    Converts Euler angles to quaternion using scipy.spatial.transform.Rotation,
    then converts back to Euler angles. Used as a reference implementation for
    comparison with other rotation utilities.

    Args:
        euler_angle: Input Euler angles as (roll, pitch, yaw) in degrees.
        extrinsic: If True, use extrinsic rotation convention (lowercase sequence).

    Returns:
        Tuple of (quaternion, euler_angles) where quaternion is converted to WXYZ
        format (from scipy's XYZW) and euler_angles are the round-trip values.
    """
    euler_np = np.array(euler_angle)
    scipy_seq = "xyz" if extrinsic else "XYZ"
    scipy_rot = Rotation.from_euler(scipy_seq, euler_np, degrees=True)
    scipy_quat_xyzw = scipy_rot.as_quat()
    scipy_quat = np.array([scipy_quat_xyzw[3], scipy_quat_xyzw[0], scipy_quat_xyzw[1], scipy_quat_xyzw[2]])
    scipy_euler = scipy_rot.as_euler(scipy_seq, degrees=True)
    return scipy_quat, scipy_euler


def compute_usd_rotation(euler_angle: tuple[float, float, float], extrinsic: bool) -> tuple[np.ndarray, np.ndarray]:
    """Compute quaternion and round-trip Euler angles using USD's Gf.Rotation.

    Converts Euler angles to quaternion using pxr.Gf.Rotation by composing
    individual axis rotations, then decomposes back to Euler angles. Used as
    a reference implementation for comparison with other rotation utilities.

    Args:
        euler_angle: Input Euler angles as (roll, pitch, yaw) in degrees.
        extrinsic: If True, use extrinsic rotation convention (XYZ application order).

    Returns:
        Tuple of (quaternion, euler_angles) where quaternion is in WXYZ format
        and euler_angles are the decomposed values.
    """
    euler_np = np.array(euler_angle)
    if extrinsic:
        usd_rot = (
            Gf.Rotation(Gf.Vec3d.XAxis(), float(euler_np[0]))
            * Gf.Rotation(Gf.Vec3d.YAxis(), float(euler_np[1]))
            * Gf.Rotation(Gf.Vec3d.ZAxis(), float(euler_np[2]))
        )
        usd_euler_vec = usd_rot.Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
        usd_euler = np.array([usd_euler_vec[2], usd_euler_vec[1], usd_euler_vec[0]])
    else:
        usd_rot = (
            Gf.Rotation(Gf.Vec3d.ZAxis(), float(euler_np[2]))
            * Gf.Rotation(Gf.Vec3d.YAxis(), float(euler_np[1]))
            * Gf.Rotation(Gf.Vec3d.XAxis(), float(euler_np[0]))
        )
        usd_euler_vec = usd_rot.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
        usd_euler = np.array([usd_euler_vec[0], usd_euler_vec[1], usd_euler_vec[2]])
    usd_quat_obj = usd_rot.GetQuat()
    usd_quat = np.array(
        [
            float(usd_quat_obj.GetReal()),
            float(usd_quat_obj.GetImaginary()[0]),
            float(usd_quat_obj.GetImaginary()[1]),
            float(usd_quat_obj.GetImaginary()[2]),
        ]
    )
    return usd_quat, usd_euler


def compute_usd_ui_rotation(euler_angle: tuple[float, float, float], extrinsic: bool) -> np.ndarray:
    """Compute Euler angles using USD UI rotation convention.

    Computes rotation using the convention typically used in USD UI interfaces,
    which may differ from the standard mathematical convention. This is useful
    for verifying consistency with USD viewport rotation displays.

    Args:
        euler_angle: Input Euler angles as (roll, pitch, yaw) in degrees.
        extrinsic: If True, use extrinsic rotation convention (ZYX composition order).

    Returns:
        Decomposed Euler angles in the USD UI convention.
    """
    euler_np = np.array(euler_angle)
    usd_rot = (
        Gf.Rotation(Gf.Vec3d.XAxis(), float(euler_np[0]))
        * Gf.Rotation(Gf.Vec3d.YAxis(), float(euler_np[1]))
        * Gf.Rotation(Gf.Vec3d.ZAxis(), float(euler_np[2]))
    )
    usd_ui_euler_vec = usd_rot.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    usd_ui_euler = np.array([usd_ui_euler_vec[0], usd_ui_euler_vec[1], usd_ui_euler_vec[2]])
    return usd_ui_euler


def quat_wxyz_to_matrix(quat_wxyz: np.ndarray | wp.array) -> np.ndarray:
    """Convert a quaternion in WXYZ format to a 3x3 rotation matrix.

    Uses the standard quaternion-to-rotation-matrix formula to convert
    a unit quaternion into its equivalent rotation matrix representation.

    Args:
        quat_wxyz: Quaternion in WXYZ format (scalar-first), can be numpy array
            or tensor with .numpy() method.

    Returns:
        3x3 numpy array representing the rotation matrix.
    """
    quat_np = quat_wxyz.numpy() if hasattr(quat_wxyz, "numpy") else np.array(quat_wxyz)
    w, x, y, z = quat_np.astype(float)
    ww, xx, yy, zz = w * w, x * x, y * y, z * z
    wx, wy, wz = w * x, w * y, w * z
    xy, xz, yz = x * y, x * z, y * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def compare_rotations(
    euler_angle: tuple[float, float, float],
    numpy_quat: np.ndarray,
    numpy_euler: np.ndarray,
    exp_quat: wp.array,
    exp_euler: wp.array,
    scipy_quat: np.ndarray,
    scipy_euler: np.ndarray,
    usd_quat: np.ndarray,
    usd_euler: np.ndarray,
) -> dict:
    """Compare rotation outputs from multiple implementations.

    Performs comprehensive comparison of quaternions and Euler angles computed
    by numpy, experimental, scipy, and USD rotation utilities. Checks for NaN
    values, round-trip consistency, quaternion equivalence (accounting for sign
    ambiguity), and rotation matrix equivalence.

    Args:
        euler_angle: Original input Euler angles.
        numpy_quat: Quaternion from numpy rotation utilities.
        numpy_euler: Round-trip Euler angles from numpy.
        exp_quat: Quaternion from experimental transform utilities.
        exp_euler: Round-trip Euler angles from experimental.
        scipy_quat: Quaternion from scipy Rotation.
        scipy_euler: Round-trip Euler angles from scipy.
        usd_quat: Quaternion from USD Gf.Rotation.
        usd_euler: Round-trip Euler angles from USD.

    Returns:
        Dictionary containing comparison results with keys for NaN detection,
        Euler angle matching, quaternion matching (with and without sign flip),
        rotation matrix matching, and warning messages.
    """
    euler_np = np.array(euler_angle)
    numpy_quat_np = np.array(numpy_quat)
    numpy_euler_np = np.array(numpy_euler)
    exp_quat_np = exp_quat.numpy() if hasattr(exp_quat, "numpy") else np.array(exp_quat)
    exp_euler_np = exp_euler.numpy() if hasattr(exp_euler, "numpy") else np.array(exp_euler)

    nan_numpy = np.isnan(numpy_quat_np).any() or np.isnan(numpy_euler_np).any()
    nan_exp = np.isnan(exp_quat_np).any() or np.isnan(exp_euler_np).any()
    nan_scipy = np.isnan(scipy_quat).any() or np.isnan(scipy_euler).any()
    nan_usd = np.isnan(usd_quat).any() or np.isnan(usd_euler).any()

    numpy_euler_match = np.allclose(numpy_euler_np, euler_np, atol=1e-5, rtol=1e-5)
    exp_euler_match = np.allclose(exp_euler_np, euler_np, atol=1e-5, rtol=1e-5)
    scipy_euler_match = np.allclose(scipy_euler, euler_np, atol=1e-5, rtol=1e-5)
    usd_euler_match = np.allclose(usd_euler, euler_np, atol=1e-5, rtol=1e-5)

    quat_match_raw = np.allclose(exp_quat_np, numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match = quat_match_raw or np.allclose(exp_quat_np, -numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_scipy_numpy_raw = np.allclose(scipy_quat, numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_scipy_numpy = quat_match_scipy_numpy_raw or np.allclose(scipy_quat, -numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_scipy_exp_raw = np.allclose(scipy_quat, exp_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_scipy_exp = quat_match_scipy_exp_raw or np.allclose(scipy_quat, -exp_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_usd_numpy_raw = np.allclose(usd_quat, numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_usd_numpy = quat_match_usd_numpy_raw or np.allclose(usd_quat, -numpy_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_usd_exp_raw = np.allclose(usd_quat, exp_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_usd_exp = quat_match_usd_exp_raw or np.allclose(usd_quat, -exp_quat_np, atol=1e-5, rtol=1e-5)
    quat_match_usd_scipy_raw = np.allclose(usd_quat, scipy_quat, atol=1e-5, rtol=1e-5)
    quat_match_usd_scipy = quat_match_usd_scipy_raw or np.allclose(usd_quat, -scipy_quat, atol=1e-5, rtol=1e-5)

    numpy_matrix = quat_wxyz_to_matrix(numpy_quat_np)
    exp_matrix = quat_wxyz_to_matrix(exp_quat_np)
    scipy_matrix = quat_wxyz_to_matrix(scipy_quat)
    usd_matrix = quat_wxyz_to_matrix(usd_quat)

    matrix_match_exp_numpy = np.allclose(exp_matrix, numpy_matrix, atol=1e-5, rtol=1e-5)
    matrix_match_scipy_numpy = np.allclose(scipy_matrix, numpy_matrix, atol=1e-5, rtol=1e-5)
    matrix_match_usd_numpy = np.allclose(usd_matrix, numpy_matrix, atol=1e-5, rtol=1e-5)
    matrix_match_usd_exp = np.allclose(usd_matrix, exp_matrix, atol=1e-5, rtol=1e-5)
    matrix_match_usd_scipy = np.allclose(usd_matrix, scipy_matrix, atol=1e-5, rtol=1e-5)

    messages = []
    if nan_numpy or nan_exp:
        messages.append("  WARNING: NaN detected in outputs.")
    if nan_scipy:
        messages.append("  WARNING: NaN detected in scipy outputs.")
    if nan_usd:
        messages.append("  WARNING: NaN detected in USD outputs.")
    if not numpy_euler_match:
        messages.append("  WARNING: numpy euler_from_quat differs from input euler.")
    if not exp_euler_match:
        messages.append("  WARNING: experimental euler_from_quat differs from input euler.")
    if not scipy_euler_match:
        messages.append("  WARNING: scipy euler_from_quat differs from input euler.")
    if not usd_euler_match:
        messages.append("  WARNING: USD euler_from_quat differs from input euler.")
    if not quat_match:
        messages.append("  WARNING: quaternion mismatch between numpy and experimental.")
    if not quat_match_scipy_numpy:
        messages.append("  WARNING: quaternion mismatch between scipy and numpy.")
    if not quat_match_scipy_exp:
        messages.append("  WARNING: quaternion mismatch between scipy and experimental.")
    if not quat_match_usd_numpy:
        messages.append("  WARNING: quaternion mismatch between USD and numpy.")
    if not quat_match_usd_exp:
        messages.append("  WARNING: quaternion mismatch between USD and experimental.")
    if not quat_match_usd_scipy:
        messages.append("  WARNING: quaternion mismatch between USD and scipy.")
    if not matrix_match_exp_numpy:
        messages.append("  WARNING: rotation matrix mismatch between experimental and numpy.")
    if not matrix_match_scipy_numpy:
        messages.append("  WARNING: rotation matrix mismatch between scipy and numpy.")
    if not matrix_match_usd_numpy:
        messages.append("  WARNING: rotation matrix mismatch between USD and numpy.")
    if not matrix_match_usd_exp:
        messages.append("  WARNING: rotation matrix mismatch between USD and experimental.")
    if not matrix_match_usd_scipy:
        messages.append("  WARNING: rotation matrix mismatch between USD and scipy.")

    return {
        "nan_numpy": nan_numpy,
        "nan_exp": nan_exp,
        "nan_scipy": nan_scipy,
        "nan_usd": nan_usd,
        "numpy_euler_match": numpy_euler_match,
        "exp_euler_match": exp_euler_match,
        "scipy_euler_match": scipy_euler_match,
        "usd_euler_match": usd_euler_match,
        "quat_match": quat_match,
        "quat_match_raw": quat_match_raw,
        "quat_match_scipy_numpy": quat_match_scipy_numpy,
        "quat_match_scipy_numpy_raw": quat_match_scipy_numpy_raw,
        "quat_match_scipy_exp": quat_match_scipy_exp,
        "quat_match_scipy_exp_raw": quat_match_scipy_exp_raw,
        "quat_match_usd_numpy": quat_match_usd_numpy,
        "quat_match_usd_numpy_raw": quat_match_usd_numpy_raw,
        "quat_match_usd_exp": quat_match_usd_exp,
        "quat_match_usd_exp_raw": quat_match_usd_exp_raw,
        "quat_match_usd_scipy": quat_match_usd_scipy,
        "quat_match_usd_scipy_raw": quat_match_usd_scipy_raw,
        "matrix_match_exp_numpy": matrix_match_exp_numpy,
        "matrix_match_scipy_numpy": matrix_match_scipy_numpy,
        "matrix_match_usd_numpy": matrix_match_usd_numpy,
        "matrix_match_usd_exp": matrix_match_usd_exp,
        "matrix_match_usd_scipy": matrix_match_usd_scipy,
        "messages": messages,
    }


async def test_rotation_utils_async(
    device_values: list[str | None],
    extrinsic_values: list[bool],
    euler_angles: list[tuple[float, float, float]],
    print_comparison_stats: bool,
    create_cubes: bool = False,
) -> list[dict]:
    """Run comprehensive rotation utility comparison tests asynchronously.

    Tests rotation conversions across multiple implementations (numpy, experimental,
    scipy, USD) for various device types, rotation conventions, and Euler angles.
    Optionally creates visual cubes in a USD stage for debugging.

    Args:
        device_values: List of device values to test (e.g., [None], ["cpu", "cuda"]).
        extrinsic_values: List of extrinsic flags to test (e.g., [True, False]).
        euler_angles: List of Euler angle tuples to test.
        print_comparison_stats: If True, print detailed comparison output.
        create_cubes: If True, create USD cubes for visual verification.

    Returns:
        List of comparison dictionaries, each containing device, extrinsic flag,
        euler_angle, and comparison results.
    """
    if create_cubes:
        await omni.usd.get_context().new_stage_async()
        rep.functional.create.dome_light(intensity=1000)

    comparisons = []
    for device_value in device_values:
        device_label = "default" if device_value is None else device_value
        for extrinsic in extrinsic_values:
            if print_comparison_stats:
                print("\n------------------------------------------------------------")
                print(f"device={device_label}, extrinsic={extrinsic}")
                print("----------------------------------")

            for index, euler_angle in enumerate(euler_angles):
                if print_comparison_stats:
                    print(f" device={device_label}, extrinsic={extrinsic}, euler_angle: {euler_angle}")

                numpy_quat, numpy_euler = compute_numpy_rotation(euler_angle, extrinsic)
                exp_quat, exp_euler = compute_experimental_rotation(euler_angle, extrinsic, device_value)
                scipy_quat, scipy_euler = compute_scipy_rotation(euler_angle, extrinsic)
                usd_quat, usd_euler = compute_usd_rotation(euler_angle, extrinsic)

                if create_cubes:
                    numpy_quat_cube_name = f"cube_numpy_quat_{index}_extrinsic_{extrinsic}_euler_{euler_angle[0]}_{euler_angle[1]}_{euler_angle[2]}"
                    exp_quat_cube_name = f"cube_exp_quat_{index}_extrinsic_{extrinsic}_euler_{euler_angle[0]}_{euler_angle[1]}_{euler_angle[2]}"
                    scipy_quat_cube_name = f"cube_scipy_quat_{index}_extrinsic_{extrinsic}_euler_{euler_angle[0]}_{euler_angle[1]}_{euler_angle[2]}"
                    usd_quat_cube_name = f"cube_usd_quat_{index}_extrinsic_{extrinsic}_euler_{euler_angle[0]}_{euler_angle[1]}_{euler_angle[2]}"
                    create_usd_quat_cube(numpy_quat_cube_name, index, numpy_quat)
                    create_usd_quat_cube(exp_quat_cube_name, index, exp_quat)
                    create_usd_quat_cube(scipy_quat_cube_name, index, scipy_quat)
                    create_usd_quat_cube(usd_quat_cube_name, index, usd_quat)

                if print_comparison_stats:
                    print("  numpy.rotations:")
                    print(f"    quat: {numpy_quat}, type: {type(numpy_quat)}")
                    print(f"    euler_from_quat: {numpy_euler}")
                    print("  experimental.transform:")
                    print(f"    quat: {exp_quat}")
                    print(f"    euler_from_quat: {exp_euler}")
                    print("  scipy.spatial.transform:")
                    print(f"    quat: {scipy_quat}")
                    print(f"    euler_from_quat: {scipy_euler}")
                    print("  pxr.Gf:")
                    print(f"    quat: {usd_quat}")
                    print(f"    euler_from_quat: {usd_euler}")
                    print("  rotation matrices (wxyz):")
                    print(f"    numpy:\n{quat_wxyz_to_matrix(numpy_quat)}")
                    print(f"    experimental:\n{quat_wxyz_to_matrix(exp_quat)}")
                    print(f"    scipy:\n{quat_wxyz_to_matrix(scipy_quat)}")
                    print(f"    usd:\n{quat_wxyz_to_matrix(usd_quat)}")

                comparison = compare_rotations(
                    euler_angle,
                    numpy_quat,
                    numpy_euler,
                    exp_quat,
                    exp_euler,
                    scipy_quat,
                    scipy_euler,
                    usd_quat,
                    usd_euler,
                )
                comparisons.append(
                    {
                        "device": device_label,
                        "extrinsic": extrinsic,
                        "euler_angle": euler_angle,
                        "comparison": comparison,
                    }
                )
                if print_comparison_stats:
                    for message in comparison["messages"]:
                        print(message)
                    print("----------------------------------")
    return comparisons


class TestRotations(omni.kit.test.AsyncTestCase):
    """Test suite for validating rotation conversion utilities across implementations.

    Compares rotation conversions between numpy, experimental, scipy, and USD
    implementations to ensure consistency in quaternion and Euler angle calculations.
    """

    async def test_rotation_no_nan_outputs(self) -> None:
        """Verify that rotation utilities do not produce NaN values.

        Tests that quaternion and Euler angle outputs from numpy, experimental,
        scipy, and USD rotation utilities contain no NaN values for a variety
        of input Euler angles and rotation conventions.
        """
        device_values = [None, "cpu"]
        extrinsic_values = [True, False]
        euler_angles = [(10.0, 20.0, 30.0), (-15.0, 25.0, -35.0), (-90.0, 60.0, 90.0)]

        comparisons = await test_rotation_utils_async(
            device_values, extrinsic_values, euler_angles, print_comparison_stats=False
        )
        for entry in comparisons:
            comparison = entry["comparison"]
            self.assertFalse(
                comparison["nan_numpy"] or comparison["nan_exp"],
                f"NaN found for numpy/experimental with {entry}",
            )
            self.assertFalse(comparison["nan_scipy"], f"NaN found for scipy with {entry}")
            self.assertFalse(comparison["nan_usd"], f"NaN found for USD with {entry}")
            self.assertFalse(
                comparison["nan_exp"],
                f"NaN found for experimental rotation outputs with {entry}",
            )

    async def test_rotation_quaternion_matches(self) -> None:
        """Verify quaternion consistency across rotation implementations.

        Tests that quaternions computed from the same Euler angles match across
        numpy, experimental, scipy, and USD implementations. Accounts for the
        quaternion double-cover property (q and -q represent the same rotation).
        """
        device_values = [None, "cpu"]
        extrinsic_values = [True, False]
        euler_angles = [(10.0, 20.0, 30.0), (-15.0, 25.0, -35.0), (-90.0, 60.0, 90.0)]

        comparisons = await test_rotation_utils_async(
            device_values, extrinsic_values, euler_angles, print_comparison_stats=False
        )
        for entry in comparisons:
            comparison = entry["comparison"]
            self.assertTrue(comparison["quat_match_scipy_numpy"], f"scipy vs numpy mismatch: {entry}")
            self.assertTrue(comparison["quat_match"], f"experimental vs numpy mismatch: {entry}")
            self.assertTrue(comparison["quat_match_usd_numpy"], f"USD vs numpy mismatch: {entry}")

    async def test_rotation_matrix_matches(self) -> None:
        """Verify rotation matrix consistency across implementations.

        Tests that rotation matrices derived from quaternions match across
        numpy, experimental, scipy, and USD implementations. This provides
        an unambiguous comparison that avoids quaternion sign differences.
        """
        device_values = [None, "cpu"]
        extrinsic_values = [True, False]
        euler_angles = [(10.0, 20.0, 30.0), (-15.0, 25.0, -35.0), (-90.0, 60.0, 90.0)]

        comparisons = await test_rotation_utils_async(
            device_values, extrinsic_values, euler_angles, print_comparison_stats=False
        )
        for entry in comparisons:
            comparison = entry["comparison"]
            self.assertTrue(comparison["matrix_match_exp_numpy"], f"experimental vs numpy matrices mismatch: {entry}")
            self.assertTrue(comparison["matrix_match_scipy_numpy"], f"scipy vs numpy matrices mismatch: {entry}")
            self.assertTrue(comparison["matrix_match_usd_numpy"], f"USD vs numpy matrices mismatch: {entry}")
