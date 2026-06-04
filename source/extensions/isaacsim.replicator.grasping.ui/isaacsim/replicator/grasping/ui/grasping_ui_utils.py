# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for the grasping UI including prim selection, visualization, and grasp phase management."""

import carb
import isaacsim.replicator.grasping.grasping_utils as grasping_utils
import isaacsim.replicator.grasping.sampler_utils as sampler_utils
import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.util.debug_draw import _debug_draw
from pxr import Gf, Sdf, Usd, UsdGeom


def get_selected_prim_path() -> str | None:
    """Get the path of a single selected prim in the stage.

    Returns:
        The path of the selected prim, or None if no prim is selected or multiple prims are selected.
    """
    selection = omni.usd.get_context().get_selection()
    selected_prim_paths = selection.get_selected_prim_paths()
    if not selected_prim_paths:
        carb.log_warn("No prim selected. Please select a prim in the stage.")
        return None
    if len(selected_prim_paths) > 1:
        carb.log_warn("Multiple prims selected. Please select only one prim.")
        return None
    return selected_prim_paths[0]


def select_prim_in_stage(path: str) -> None:
    """Select/highlight a prim in the stage by its path.

    Args:
        path: The USD path of the prim to select.
    """
    omni.kit.commands.execute(
        "SelectPrimsCommand", old_selected_paths=[], new_selected_paths=[path], expand_in_stage=False
    )


def get_valid_prim_from_path(prim_path: str) -> Usd.Prim | None:
    """Check if a prim path is valid and the prim exists in the stage.

    Args:
        prim_path: Path to the prim to check.

    Returns:
        The valid prim if it exists, or None if the path is invalid or prim does not exist.
    """
    if not prim_path:
        return None
    if not Sdf.Path.IsValidPathString(prim_path):
        carb.log_warn(f"Prim path '{prim_path}' is not a valid path format.")
        return None
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        carb.log_warn(f"Prim at path '{prim_path}' is not valid or does not exist in the stage.")
        return None
    return prim


def get_joint_display_name(joint_path: str, gripper_path: str) -> str:
    """Get the display name of a joint relative to the gripper.

    Args:
        joint_path: Absolute path to the joint.
        gripper_path: Path to the gripper prim.

    Returns:
        The display name of the joint.
    """
    if gripper_path and joint_path.startswith(gripper_path):
        return joint_path.replace(gripper_path, "", 1)
    return joint_path


def get_joint_absolute_path(display_name: str, gripper_path: str) -> str:
    """Get the absolute path of a joint relative to the gripper.

    Args:
        display_name: Display name of the joint.
        gripper_path: Path to the gripper prim.

    Returns:
        The absolute path of the joint.
    """
    if gripper_path and not display_name.startswith("/"):
        return gripper_path + display_name
    return display_name


def draw_grasp_samples_as_lines(
    grasp_poses: list[tuple["Gf.Vec3d", "Gf.Quatd"]],
    point_size: float = 5,
    line_thickness: float = 3,
    line_length: float = 0.02,
    line_opacity: float = 0.5,
    clear_existing: bool = True,
) -> None:
    """Draw grasp samples in the viewport.

    Args:
        grasp_poses: List of (Gf.Vec3d, Gf.Quatd) tuples representing position and orientation.
        point_size: Size of the points to draw.
        line_thickness: Thickness of the orientation lines.
        line_length: Length of the orientation lines.
        line_opacity: Opacity of the orientation lines.
        clear_existing: Whether to clear existing samples before drawing.
    """
    draw_iface = _debug_draw.acquire_debug_draw_interface()
    if clear_existing:
        draw_iface.clear_points()
        draw_iface.clear_lines()
    draw_points = []
    draw_colors = []
    draw_sizes = []
    draw_lines_length = line_length
    draw_lines_start_points = []
    draw_lines_end_points = []
    draw_lines_sizes = []
    draw_lines_colors = []
    num_poses = len(grasp_poses)

    def simple_colormap(t: float) -> tuple[float, float, float, float]:
        # Linear interpolation from blue (0,0,1) to red (1,0,0)
        r = t
        g = 0.0
        b = 1.0 - t
        a = 1.0
        return (r, g, b, a)

    for idx, (location, quat) in enumerate(grasp_poses):
        x, y, z = location[0], location[1], location[2]
        draw_points.append([x, y, z])
        t = idx / (num_poses - 1) if num_poses > 1 else 0
        r, g, b, a = simple_colormap(t)
        draw_colors.append([r, g, b, a])
        draw_sizes.append(point_size)
        ref_vector = Gf.Vec3d(0, 0, 1)
        rotated_vector = quat.Transform(ref_vector)
        scaled_vector = draw_lines_length * rotated_vector
        line_end_point = [x + scaled_vector[0], y + scaled_vector[1], z + scaled_vector[2]]
        draw_lines_start_points.append([x, y, z])
        draw_lines_end_points.append(line_end_point)
        draw_lines_sizes.append(line_thickness)
        draw_lines_colors.append([r, g, b, a * line_opacity])
    draw_iface.draw_points(draw_points, draw_colors, draw_sizes)
    draw_iface.draw_lines(draw_lines_start_points, draw_lines_end_points, draw_lines_colors, draw_lines_sizes)


def draw_grasp_samples_as_axes(
    grasp_poses: list[tuple["Gf.Vec3d", "Gf.Quatd"]],
    axis_length: float = 0.03,
    line_thickness: float = 2,
    line_opacity: float = 0.5,
    clear_existing: bool = True,
) -> None:
    """Draw grasp samples as oriented frames (axes) in the viewport.

    Args:
        grasp_poses: List of (Gf.Vec3d, Gf.Quatd) tuples representing position and orientation.
        axis_length: Length of each axis line.
        line_thickness: Thickness of the axis lines.
        line_opacity: Opacity of the axis lines.
        clear_existing: Whether to clear existing samples before drawing.
    """
    draw_iface = _debug_draw.acquire_debug_draw_interface()
    if clear_existing:
        draw_iface.clear_points()
        draw_iface.clear_lines()
    # Axis colors: X=Red, Y=Green, Z=Blue
    x_color = [1.0, 0.0, 0.0, line_opacity]
    y_color = [0.0, 1.0, 0.0, line_opacity]
    z_color = [0.0, 0.0, 1.0, line_opacity]
    start_points = []
    end_points = []
    colors = []
    thicknesses = []
    for location, quat in grasp_poses:
        origin = [location[0], location[1], location[2]]
        # X axis
        x_axis = quat.Transform(Gf.Vec3d(1, 0, 0)) * axis_length
        x_end = [origin[0] + x_axis[0], origin[1] + x_axis[1], origin[2] + x_axis[2]]
        start_points.append(origin)
        end_points.append(x_end)
        colors.append(x_color)
        thicknesses.append(line_thickness)
        # Y axis
        y_axis = quat.Transform(Gf.Vec3d(0, 1, 0)) * axis_length
        y_end = [origin[0] + y_axis[0], origin[1] + y_axis[1], origin[2] + y_axis[2]]
        start_points.append(origin)
        end_points.append(y_end)
        colors.append(y_color)
        thicknesses.append(line_thickness)
        # Z axis
        z_axis = quat.Transform(Gf.Vec3d(0, 0, 1)) * axis_length
        z_end = [origin[0] + z_axis[0], origin[1] + z_axis[1], origin[2] + z_axis[2]]
        start_points.append(origin)
        end_points.append(z_end)
        colors.append(z_color)
        thicknesses.append(line_thickness)
    draw_iface.draw_lines(start_points, end_points, colors, thicknesses)


def draw_trimesh(prim: Usd.Prim, world_frame: bool = False, clear_existing: bool = True, verbose: bool = False) -> None:
    """Draw the mesh vertices and edges for the given prim using the debug draw interface.

    Args:
        prim: The USD prim (should be a UsdGeom.Mesh or contain one).
        world_frame: If True, transform vertices to world coordinates.
        clear_existing: If True, clear previous drawings first.
        verbose: If True, enables verbose logging during mesh conversion.

    Returns:
        None.
    """
    mesh_schema = None
    if prim.IsA(UsdGeom.Mesh):
        mesh_schema = UsdGeom.Mesh(prim)
    else:
        mesh_schema = grasping_utils.find_first_mesh_in_hierarchy(prim)
    if not mesh_schema:
        carb.log_warn(f"No UsdGeom.Mesh found for prim '{prim.GetPath()}' or its descendants.")
        return
    try:
        trimesh_obj = sampler_utils.usd_mesh_to_trimesh(mesh_schema, apply_scale=True, verbose=verbose)
    except Exception as e:
        carb.log_warn(f"Failed to convert mesh to trimesh: {e}")
        return
    vertices = trimesh_obj.vertices
    if world_frame:
        xform_cache = UsdGeom.XformCache()
        transform_matrix = xform_cache.GetLocalToWorldTransform(mesh_schema.GetPrim()).RemoveScaleShear()
        vertices = [transform_matrix.Transform(Gf.Vec3d(*v)) for v in vertices]
        vertices = np.array(vertices)

    draw_iface = _debug_draw.acquire_debug_draw_interface()
    if clear_existing:
        draw_iface.clear_points()
        draw_iface.clear_lines()

    vertex_color = [1.0, 0.0, 0.0, 1.0]  # Red, RGBA
    edge_color = [0.0, 1.0, 0.0, 1.0]  # Green, RGBA
    draw_iface.draw_points(vertices.tolist(), [vertex_color] * len(vertices), [5] * len(vertices))
    edges = trimesh_obj.edges
    edge_lines = [(vertices[e[0]], vertices[e[1]]) for e in edges]
    start_points = [line[0].tolist() for line in edge_lines]
    end_points = [line[1].tolist() for line in edge_lines]
    draw_iface.draw_lines(start_points, end_points, [edge_color] * len(edges), [1] * len(edges))


def clear_debug_draw() -> None:
    """Clear all debug draw points and lines from the viewport."""
    draw_iface = _debug_draw.acquire_debug_draw_interface()
    draw_iface.clear_points()
    draw_iface.clear_lines()


def move_grasp_phase_up(grasping_manager: object, phase_identifier: str | int) -> bool:
    """Move a grasp phase up in the sequence (earlier execution).

    Args:
        grasping_manager: The grasping manager containing grasp phases.
        phase_identifier: Name or index of the phase to move.

    Returns:
        True if the phase was moved successfully, False otherwise.
    """
    phase_index = -1
    if isinstance(phase_identifier, int):
        if 0 <= phase_identifier < len(grasping_manager.grasp_phases):
            phase_index = phase_identifier
    else:
        for i, phase in enumerate(grasping_manager.grasp_phases):
            if phase.name == phase_identifier:
                phase_index = i
                break
    if phase_index <= 0:
        return False
    grasping_manager.grasp_phases[phase_index], grasping_manager.grasp_phases[phase_index - 1] = (
        grasping_manager.grasp_phases[phase_index - 1],
        grasping_manager.grasp_phases[phase_index],
    )
    return True


def move_grasp_phase_down(grasping_manager: object, phase_identifier: str | int) -> bool:
    """Move a grasp phase down in the sequence (later execution).

    Args:
        grasping_manager: The grasping manager containing grasp phases.
        phase_identifier: Name or index of the phase to move.

    Returns:
        True if the phase was moved successfully, False otherwise.
    """
    phase_index = -1
    if isinstance(phase_identifier, int):
        if 0 <= phase_identifier < len(grasping_manager.grasp_phases):
            phase_index = phase_identifier
    else:
        for i, phase in enumerate(grasping_manager.grasp_phases):
            if phase.name == phase_identifier:
                phase_index = i
                break
    if phase_index < 0 or phase_index >= len(grasping_manager.grasp_phases) - 1:
        return False
    grasping_manager.grasp_phases[phase_index], grasping_manager.grasp_phases[phase_index + 1] = (
        grasping_manager.grasp_phases[phase_index + 1],
        grasping_manager.grasp_phases[phase_index],
    )
    return True


def parse_vector_string(vector_str: str) -> tuple[float, float, float] | None:
    """Parse a vector string in the format '(x, y, z)' into a tuple of floats.

    Args:
        vector_str: String representation of a 3D vector in format "(x, y, z)"

    Returns:
        Tuple of (x, y, z) as floats, or None if parsing fails
    """
    try:
        # Remove parentheses and split by commas
        clean_str = vector_str.strip("()")
        components = [float(x.strip()) for x in clean_str.split(",")]

        # Ensure we have exactly 3 components
        if len(components) == 3:
            return tuple(components)

        carb.log_warn(
            f"Invalid vector format: '{vector_str}'. Expected format is '(x, y, z)' with exactly 3 components."
        )
        return None
    except (ValueError, IndexError) as e:
        carb.log_warn(f"Failed to parse vector string '{vector_str}': {str(e)}. Expected format is '(x, y, z)'.")
        return None
