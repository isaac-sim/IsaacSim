# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for 3D-to-2D point projection and camera coordinate transformations in replicator writers."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
from pxr import Gf, Usd, UsdGeom


def project_point_to_screen(camera_point, camera_params):
    """Project a 3D point from camera coordinates to 2D screen coordinates.

    Dispatches to the appropriate projection method based on the camera model.
    Supported models: pinhole, pinholeOpenCV, fisheyePolynomial.
    Unsupported models fall back to pinhole projection.

    Args:
        camera_point: Numpy array (4,) representing [x, y, z, w] in camera coordinates.
        camera_params: Dict containing camera parameters from the camera_params annotator.
            Required keys vary by model but typically include:
            - "cameraModel": str identifying the projection model
            - "cameraProjection": 4x4 projection matrix (for pinhole)
            - "renderProductResolution": [width, height] of the output image

    Returns:
        Tuple (x, y) of rounded pixel coordinates.
    """
    camera_model = camera_params.get("cameraModel", "pinhole")

    projection_methods = {
        "pinhole": project_pinhole,
        "pinholeOpenCV": project_pinhole_opencv,
        "fisheyePolynomial": project_fisheye_polynomial,
    }

    method = projection_methods.get(camera_model, project_pinhole)
    return method(camera_point, camera_params)


def project_pinhole(camera_point, camera_params):
    """Project using standard pinhole model.

    Args:
        camera_point: Numpy array (4,) representing [x, y, z, w] in camera coordinates.
        camera_params: Dict containing:
            - "cameraProjection": flattened 4x4 projection matrix
            - "renderProductResolution": [width, height]

    Returns:
        Tuple (x, y) of rounded pixel coordinates. Points on the projection plane return the screen center.
    """
    projection_matrix = camera_params["cameraProjection"].reshape((4, 4))
    screen_size = camera_params["renderProductResolution"]

    point_screen = camera_point @ projection_matrix
    if abs(point_screen[3]) < 1e-10:
        return round(screen_size[0] / 2), round(screen_size[1] / 2)

    point_screen_normalized = point_screen / point_screen[3]

    x = (point_screen_normalized[0] + 1) * screen_size[0] / 2
    y = (1 - point_screen_normalized[1]) * screen_size[1] / 2

    return round(x), round(y)


def project_fisheye_polynomial(camera_point, camera_params):
    """Project using fisheye polynomial model (f-theta).

    The fisheye polynomial in Omniverse defines the INVERSE mapping (r -> theta):
        theta = a + b*r + c*r^2 + d*r^3 + e*r^4 + f*r^5
    For projection (theta -> r), we invert this numerically.

    Args:
        camera_point: Numpy array (4,) representing [x, y, z, w] in camera coordinates.
        camera_params: Dict containing:
            - "cameraFisheyePolynomial": [a, b, c, d, e, f] coefficients
            - "cameraFisheyeOpticalCentre": [cx, cy] in pixels
            - "cameraFisheyeNominalWidth": nominal image width
            - "cameraFisheyeNominalHeight": nominal image height
            - "renderProductResolution": [width, height]

    Returns:
        Tuple (x, y) of rounded pixel coordinates.
    """
    poly_coeffs = camera_params["cameraFisheyePolynomial"]
    optical_center = camera_params["cameraFisheyeOpticalCentre"]
    nominal_width = camera_params["cameraFisheyeNominalWidth"]
    nominal_height = camera_params["cameraFisheyeNominalHeight"]
    screen_size = camera_params["renderProductResolution"]

    x_cam, y_cam, z_cam = camera_point[0], camera_point[1], camera_point[2]
    r_cam = np.sqrt(x_cam**2 + y_cam**2)

    if r_cam < 1e-10:
        px = optical_center[0] * screen_size[0] / nominal_width
        py = optical_center[1] * screen_size[1] / nominal_height
        return round(px), round(py)

    theta = np.arctan2(r_cam, -z_cam)
    r_pixels = invert_fisheye_polynomial(theta, poly_coeffs)

    dir_x = x_cam / r_cam
    dir_y = y_cam / r_cam

    px = optical_center[0] + r_pixels * dir_x
    py = optical_center[1] - r_pixels * dir_y

    px = px * screen_size[0] / nominal_width
    py = py * screen_size[1] / nominal_height

    return round(px), round(py)


def project_pinhole_opencv(camera_point, camera_params):
    """Project using OpenCV pinhole model with fx, fy, cx, cy.

    Args:
        camera_point: Numpy array (4,) representing [x, y, z, w] in camera coordinates.
        camera_params: Dict from camera_params annotator containing:
            - "cameraOpenCVFx": focal length in x (pixels)
            - "cameraOpenCVFy": focal length in y (pixels)
            - "cameraFisheyeOpticalCentre": [cx, cy] principal point (annotator uses this key for all models)
            - "renderProductResolution": [width, height]

    Returns:
        Tuple (x, y) of rounded pixel coordinates.
    """
    fx = camera_params["cameraOpenCVFx"]
    fy = camera_params["cameraOpenCVFy"]
    optical_center = camera_params["cameraFisheyeOpticalCentre"]
    screen_size = camera_params["renderProductResolution"]

    nominal_width = camera_params.get("cameraFisheyeNominalWidth", 0)
    nominal_height = camera_params.get("cameraFisheyeNominalHeight", 0)
    if nominal_width == 0:
        nominal_width = screen_size[0]
    if nominal_height == 0:
        nominal_height = screen_size[1]

    cx = optical_center[0] if optical_center[0] != 0 else nominal_width / 2.0
    cy = optical_center[1] if optical_center[1] != 0 else nominal_height / 2.0

    x_cam, y_cam, z_cam = camera_point[0], camera_point[1], camera_point[2]

    if abs(z_cam) < 1e-10:
        return round(cx), round(cy)

    px = fx * (x_cam / -z_cam) + cx
    py = fy * (-y_cam / -z_cam) + cy

    px = px * screen_size[0] / nominal_width
    py = py * screen_size[1] / nominal_height

    return round(px), round(py)


def invert_fisheye_polynomial(theta, poly_coeffs, max_iterations=10, tolerance=1e-6):
    """Invert the fisheye polynomial to solve for r given theta.

    The polynomial is: theta = a + b*r + c*r^2 + d*r^3 + e*r^4 + f*r^5
    We solve for r using Newton-Raphson iteration.

    Args:
        theta: The angle from optical axis in radians.
        poly_coeffs: List/array of 6 coefficients [a, b, c, d, e, f].
        max_iterations: Maximum Newton-Raphson iterations.
        tolerance: Convergence tolerance.

    Returns:
        The radial distance r in pixels. If Newton-Raphson does not converge, logs a warning before returning the
        final iterate.
    """
    a, b, c, d, e, f = poly_coeffs

    if abs(b) > 1e-10 and abs(c) < 1e-10 and abs(d) < 1e-10 and abs(e) < 1e-10 and abs(f) < 1e-10:
        return (theta - a) / b

    if abs(b) > 1e-10:
        r = (theta - a) / b
    else:
        r = theta

    iterations_run = 0
    for iterations_run in range(1, max_iterations + 1):
        r2, r3, r4, r5 = r**2, r**3, r**4, r**5
        f_r = a + b * r + c * r2 + d * r3 + e * r4 + f * r5 - theta
        f_prime = b + 2 * c * r + 3 * d * r2 + 4 * e * r3 + 5 * f * r4

        if abs(f_prime) < 1e-10:
            break

        r_new = r - f_r / f_prime

        if abs(r_new - r) < tolerance:
            return r_new

        r = r_new

    final_residual = a + b * r + c * r**2 + d * r**3 + e * r**4 + f * r**5 - theta

    carb.log_warn(
        "invert_fisheye_polynomial: Newton-Raphson did not converge within "
        f"{max_iterations} iterations (completed={iterations_run}, final residual={abs(final_residual):.2e}). "
        "Result may be inaccurate. Increase max_iterations or verify poly_coeffs conditioning."
    )
    return r


def calculate_truncation_ratio_simple(corners, img_width, img_height):
    """Calculate the truncation ratio of a cuboid using a simplified bounding box method.

    Args:
        corners: (9, 2) numpy array containing the projected corners of the cuboid.
        img_width: Width of image.
        img_height: Height of image.

    Returns:
        The truncation ratio of the cuboid.
        1 means object is fully truncated and 0 means object is fully within screen.
    """

    # Calculate the bounding box of the cuboid
    x_min, y_min = np.min(corners, axis=0)
    x_max, y_max = np.max(corners, axis=0)

    # Original bounding box area
    original_area = (x_max - x_min) * (y_max - y_min)

    # Clip the bounding box to the screen
    clipped_x_min = min(max(x_min, 0), img_width)
    clipped_y_min = min(max(y_min, 0), img_height)
    clipped_x_max = max(min(x_max, img_width), 0)
    clipped_y_max = max(min(y_max, img_height), 0)

    # Clipped bounding box area
    clipped_area = (clipped_x_max - clipped_x_min) * (clipped_y_max - clipped_y_min)

    # Compute the truncation ratio
    truncation_ratio = 1 - clipped_area / original_area if original_area > 0 else 1

    return truncation_ratio


def get_image_space_points(points, view_proj_matrix):
    """Project world space points into image space using a view projection matrix.

    Args:
        points: Numpy array of N points (N, 3) in the world space. Points will be projected into the image space.
        view_proj_matrix: Desired view projection matrix, transforming points from world frame to image space of desired camera.

    Returns:
        Numpy array of shape (N, 3) of points projected into the image space.
    """

    homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(homo, view_proj_matrix)
    tf_points = tf_points / (tf_points[..., -1:])
    tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
    image_space_points = tf_points[..., :3]

    return image_space_points


def get_transform_with_normalized_rotation(transform: np.ndarray) -> np.ndarray:
    """Get the transform after normalizing rotation component.

    Args:
        transform: transformation matrix with shape (4, 4).

    Returns:
        transformation matrix with normalized rotation with shape (4, 4).
    """
    transform_without_scale = np.copy(transform.astype(float))
    rotation_matrix = transform[:3, :3]
    column_magnitudes = np.linalg.norm(rotation_matrix, axis=0)
    normalized_rotation = rotation_matrix / column_magnitudes
    transform_without_scale[:3, :3] = normalized_rotation
    return transform_without_scale


def tf_matrix_from_pose(translation: list[float], orientation: list[float]) -> np.ndarray:
    """Compute input pose to transformation matrix.

    Args:
        translation: The translation vector.
        orientation: The orientation quaternion.

    Returns:
        A 4x4 matrix.
    """
    translation = np.asarray(translation)
    orientation = np.asarray(orientation)
    mat = Gf.Transform()
    mat.SetRotation(Gf.Rotation(Gf.Quatd(*orientation.tolist())))
    mat.SetTranslation(Gf.Vec3d(*translation.tolist()))
    return np.transpose(mat.GetMatrix())


def pose_from_tf_matrix(transformation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Get pose corresponding to input transformation matrix.

    Args:
        transformation: Column-major transformation matrix. shape is (4, 4).

    Returns:
        first index is translation corresponding to transformation. shape is (3, ).
        second index is quaternion orientation corresponding to transformation.
        quaternion is scalar-first (w, x, y, z). shape is (4, ).
    """
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(transformation)))
    calculated_translation = np.array(mat.GetTranslation())
    orientation = mat.GetRotation().GetQuat()
    calculated_orientation = np.zeros(4)
    calculated_orientation[1:] = orientation.GetImaginary()
    calculated_orientation[0] = orientation.GetReal()
    return calculated_translation, calculated_orientation


def get_mesh_vertices_relative_to(mesh_prim: UsdGeom.Mesh, coord_prim: Usd.Prim) -> np.ndarray:
    """Get vertices of the mesh prim in the coordinate system of the given prim.

    Args:
        mesh_prim: Mesh prim to get the vertice points.
        coord_prim: Prim used as relative coordinate.

    Returns:
        Vertices of the mesh in the coordinate system of the given prim. Shape is (N, 3).
    """
    # Vertices of the mesh in the mesh's coordinate system
    vertices_vec3f = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
    vertices = np.array(vertices_vec3f)
    vertices_tf_row_major = np.pad(vertices, ((0, 0), (0, 1)), constant_values=1.0)
    # Transformation matrix from the coordinate system of the mesh to the coordinate system of the prim
    relative_tf_column_major = xform_utils.get_relative_transform(mesh_prim, coord_prim)
    relative_tf_row_major = np.transpose(relative_tf_column_major)
    # Transform points so they are in the coordinate system of the top-level ancestral xform prim
    points_in_relative_coord = vertices_tf_row_major @ relative_tf_row_major
    return points_in_relative_coord[:, :-1] * stage_utils.get_stage_units()[0]
