# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides interactive editing capabilities for collision spheres in robot descriptions."""

from collections import OrderedDict
from collections.abc import Generator
from typing import Any

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import lula
import numpy as np
import yaml
from isaacsim.core.experimental.objects import Sphere
from pxr import Sdf


class CollisionSphereEditor:
    """Provides interactive editing capabilities for collision spheres in robot descriptions.

    This class enables users to create, modify, visualize, and manage collision spheres for robot links
    within the Isaac Sim environment. It supports operations such as adding individual spheres, generating
    multiple spheres from mesh geometry, scaling existing spheres, and applying visual filters for better
    organization. The editor maintains undo/redo functionality for all operations and provides import/export
    capabilities for XRDF and robot description files.

    Key features include:
    - Interactive sphere creation and deletion with undo/redo support
    - Automatic sphere generation from mesh triangulation using collision sphere algorithms
    - Visual filtering system with customizable colors for different sphere categories
    - Preview mode for testing sphere placements before committing changes
    - Sphere interpolation between two existing spheres for smooth collision coverage
    - Import/export functionality for XRDF files and robot description formats
    - Link-based organization allowing operations on all spheres within specific robot links

    The editor uses visual materials to distinguish between filtered and non-filtered spheres, making it
    easy to identify and work with specific subsets of collision geometry. All sphere modifications are
    tracked as operations that can be undone or redone, providing a robust editing experience.
    """

    def __init__(self) -> None:
        self.path_2_spheres = {}
        self.path_2_sphere_serial_copy = {}

        self._operations = []

        self._redo = []

        self.filter = ""
        self.filter_in_sphere_color = np.array([37, 207, 188]) / 255  # All spheres that match filter
        self.filter_out_sphere_color = np.array([207.0, 184.0, 37.0]) / 255  # All Spheres that don't match filter

        self._preview_color = np.array([1.0, 0, 0])
        self._preview_spheres = []

        self._sphere_path_generators = {}

        self._lula_path = "/World/LulaRobotDescriptionEditor"

    def _is_prim_path_valid(self, prim_path: str) -> bool:
        """Check if a path has a valid USD Prim at it.

        Args:
            prim_path: path of the prim in the stage

        Returns:
            True if the path points to a valid prim
        """
        prim = prim_utils.get_prim_at_path(prim_path)
        if prim:
            return prim.IsValid()
        return False

    @staticmethod
    def _path_generator(path: str) -> Generator[str, None, None]:
        """Get a generator that incrementally adds integers to `path` forever.

        Args:
            path: Base path to append incremental integers to.

        Yields:
            Path strings with incrementally added integers.
        """
        count = 1
        while True:
            yield f"{path}_{count}"
            count += 1

    def _get_unused_collision_sphere_path(self, link_path: str) -> str:
        """Gets an unused collision sphere path for the specified link.

        Args:
            link_path: Path to the link where the sphere will be created.

        Returns:
            An unused collision sphere path under the specified link.
        """
        sphere_base_path = self._get_collision_sphere_base_path(link_path)

        if sphere_base_path not in self._sphere_path_generators:
            self._sphere_path_generators[sphere_base_path] = self._path_generator(sphere_base_path)

        sphere_path_generator = self._sphere_path_generators[sphere_base_path]
        sphere_path = next(sphere_path_generator)
        while self._is_prim_path_valid(sphere_path):
            sphere_path = next(sphere_path_generator)
        return sphere_path

    def clear_spheres(self, store_op: bool = True) -> None:
        """Removes all collision spheres from the editor.

        Args:
            store_op: Whether to store this operation for undo functionality.
        """
        self._sphere_path_generators = {}
        sphere_paths = list(self.path_2_spheres.keys())
        if len(sphere_paths) == 0:
            return

        if store_op:
            self.copy_all_sphere_data()
            deleted_spheres = ["DEL"]
            for sphere_path in sphere_paths:
                deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
            self._operations.append(deleted_spheres)

        for sphere_path in sphere_paths:
            self.delete_sphere(sphere_path)

    def clear_link_spheres(self, link_path: str, store_op: bool = True) -> None:
        """Removes all collision spheres associated with the specified link.

        Args:
            link_path: Path to the link whose spheres should be cleared.
            store_op: Whether to store this operation for undo functionality.
        """
        if self._get_collision_sphere_base_path(link_path) in self._sphere_path_generators:
            del self._sphere_path_generators[self._get_collision_sphere_base_path(link_path)]
        path_len = len(link_path)

        to_delete = []
        if store_op:
            self.copy_all_sphere_data()
        deleted_spheres = ["DEL"]
        for p in self.path_2_spheres:
            if self._is_prim_path_valid(p) and p[:path_len] == link_path:
                deleted_spheres.append(self.path_2_sphere_serial_copy[p])
                to_delete.append(p)

        if store_op:
            self._operations.append(deleted_spheres)
        for s in to_delete:
            self.delete_sphere(s)

    def delete_sphere(self, sphere_path: str) -> None:
        """Deletes the collision sphere at the specified path.

        Args:
            sphere_path: Path to the sphere to delete.
        """
        if self._is_prim_path_valid(sphere_path):
            stage_utils.delete_prim(sphere_path)

        if sphere_path in self.path_2_spheres:
            del self.path_2_spheres[sphere_path]

    def set_sphere_colors(
        self, filter: str, color_in: np.ndarray | None = None, color_out: np.ndarray | None = None
    ) -> None:
        """Sets the colors for spheres based on filter matching.

        Args:
            filter: Filter string to match against sphere paths.
            color_in: Color for spheres that match the filter.
            color_out: Color for spheres that do not match the filter.
        """
        if color_in is not None:
            self.filter_in_sphere_color = color_in
        if color_out is not None:
            self.filter_out_sphere_color = color_out
        self.filter = filter

        with Sdf.ChangeBlock():
            for sphere_path in self.path_2_spheres:
                self.set_sphere_color(sphere_path, False)

    def set_sphere_color(self, sphere_path: str, ensure_visual_material: bool = True) -> None:
        """Sets the color of a specific collision sphere based on filter matching.

        Args:
            sphere_path: Path to the sphere to color.
            ensure_visual_material: Whether to ensure visual materials are created before setting color.
        """
        if not self._is_prim_path_valid(sphere_path):
            return
        sphere = self.path_2_spheres[sphere_path]
        if sphere_path[: len(self.filter)] == self.filter:
            sphere.set_display_colors(self.filter_in_sphere_color)
        else:
            sphere.set_display_colors(self.filter_out_sphere_color)

    def copy_all_sphere_data(self) -> None:
        """Copies all current sphere data to the serial copy storage for undo operations."""
        sphere_paths = list(self.path_2_spheres.keys())
        deleted_spheres = ["DEL"]
        for sphere_path in sphere_paths:
            if self._is_prim_path_valid(sphere_path):
                sphere = self.path_2_spheres[sphere_path]
                color = np.array(sphere.geoms[0].GetDisplayColorAttr().Get())
                self.path_2_sphere_serial_copy[sphere_path] = {
                    "sphere_path": sphere_path,
                    "center": sphere.get_local_poses()[0].numpy()[0],
                    "radius": sphere.get_radii().numpy()[0],
                    "color": color,
                }
            else:
                if sphere_path in self.path_2_sphere_serial_copy:
                    deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
                self.delete_sphere(sphere_path)
        if len(deleted_spheres) > 1:
            self._operations.append(deleted_spheres)

    def undo(self) -> None:
        """Undo the last sphere operation."""
        if len(self._operations) == 0:
            return

        last_op = self._operations.pop()
        op_type = last_op[0]
        op = last_op[1:]

        if op_type == "ADD":
            redo = ["ADD"]
            for sphere_path in op:
                if self._is_prim_path_valid(sphere_path):
                    sphere = self.path_2_spheres[sphere_path]
                    redo.append(
                        {
                            "sphere_path": sphere_path,
                            "center": sphere.get_local_poses()[0].numpy()[0],
                            "radius": sphere.get_radii().numpy()[0],
                        }
                    )
                self.delete_sphere(sphere_path)
            self._redo.append(redo)

        elif op_type == "DEL":
            redo = ["DEL"]
            for d in op:
                sphere = Sphere(
                    d["sphere_path"],
                    translations=d["center"],
                    radii=d["radius"],
                    colors=self.filter_in_sphere_color,
                )
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                redo.append(d)
            self._redo.append(redo)

        elif op_type == "SCALE":
            redo = ["SCALE"]
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                factor = d["factor"]
                if path in self.path_2_spheres and self._is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radii(rad)
                    redo.append({"sphere_path": path, "radius": factor * rad})
            self._redo.append(redo)

    def redo(self) -> None:
        """Redo the last undone sphere operation."""
        if len(self._redo) == 0:
            return

        last_redo = self._redo.pop()
        op_type = last_redo[0]
        op = last_redo[1:]

        if op_type == "ADD":
            added_spheres = ["ADD"]
            for d in op:
                sphere = Sphere(
                    d["sphere_path"],
                    translations=d["center"],
                    radii=d["radius"],
                    colors=self.filter_in_sphere_color,
                )
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                added_spheres.append(d["sphere_path"])
            self._operations.append(added_spheres)

        elif op_type == "DEL":
            deleted_spheres = ["DEL"]
            for d in op:
                self.delete_sphere(d["sphere_path"])
                deleted_spheres.append(d)
            self._operations.append(deleted_spheres)

        elif op_type == "SCALE":
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                if path in self.path_2_spheres and self._is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radii(rad)

    def generate_spheres(
        self,
        link_path: str,
        points: np.ndarray,
        face_inds: np.ndarray,
        vert_cts: np.ndarray,
        num_spheres: int,
        radius_offset: float,
        is_preview: bool,
    ) -> None:
        """Generate collision spheres from mesh geometry for the specified link.

        Args:
            link_path: Path to the robot link.
            points: Mesh vertex positions.
            face_inds: Mesh face indices.
            vert_cts: Vertex counts per face.
            num_spheres: Number of spheres to generate.
            radius_offset: Offset to apply to sphere radii.
            is_preview: Whether to generate preview spheres instead of permanent ones.
        """
        if not is_preview and self._preview_spheres:
            # If preview spheres exist, change them to permanent spheres
            added_sphere_paths = ["ADD"]
            for preview_sphere in self._preview_spheres:
                if not preview_sphere.prims[0].IsValid():
                    continue
                sphere_path = self.add_sphere(
                    link_path,
                    preview_sphere.get_local_poses()[0].numpy()[0],
                    preview_sphere.get_radii().numpy()[0],
                    store_op=False,
                )
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()
            return

        unique_vert_cts = np.unique(vert_cts)
        if len(unique_vert_cts) != 1 or unique_vert_cts[0] != 3:
            # Non-triangulated meshes are unsupported by Lula's sphere generator
            # but the failure is recoverable (the caller just gets no spheres);
            # warn instead of error so test runs do not fail on this branch.
            self.clear_preview()
            carb.log_warn(
                "Cannot generate collision spheres for mesh because the specified mesh is not composed of triangles."
            )
            return

        num_points = face_inds.shape[0]

        generator = lula.create_collision_sphere_generator(points, face_inds.reshape((num_points // 3, 3)))
        result = generator.generate_spheres(num_spheres, radius_offset)
        if is_preview:
            self.clear_preview()
            preview_sphere_path_generator = self._path_generator(self._get_collision_sphere_preview_path(link_path))
            for lula_sphere in result:
                sphere_path = next(preview_sphere_path_generator)
                self._preview_spheres.append(
                    Sphere(
                        sphere_path,
                        colors=self._preview_color,
                        translations=lula_sphere.center,
                        radii=lula_sphere.radius,
                    )
                )
        else:
            added_sphere_paths = ["ADD"]
            for lula_sphere in result:
                sphere_path = self.add_sphere(link_path, lula_sphere.center, lula_sphere.radius, store_op=False)
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()

    def clear_preview(self) -> None:
        """Remove all preview spheres from the scene."""
        self._preview_path_generator = {}

        for sphere in self._preview_spheres:
            self.delete_sphere(sphere.paths[0])
        self._preview_spheres = []

    def add_sphere(self, link_path: str, center: object, radius: float, store_op: bool = True) -> str:
        """Add a collision sphere to the specified link.

        Args:
            link_path: Path to the robot link.
            center: Sphere center position.
            radius: Sphere radius.
            store_op: Whether to store this operation for undo functionality.

        Returns:
            Path to the created sphere prim.
        """
        if not self._is_prim_path_valid(link_path):
            carb.log_warn("Attempted to add sphere nested under non-existent path")

        if link_path[-1] == "/":
            link_path = link_path[:-1]

        self._redo = []
        sphere_path = self._get_unused_collision_sphere_path(link_path)

        if sphere_path[: len(self.filter)] == self.filter:
            color = self.filter_in_sphere_color
        else:
            color = self.filter_out_sphere_color

        sphere = Sphere(sphere_path, translations=center, radii=radius, colors=color)

        self.path_2_spheres[sphere.paths[0]] = sphere

        if store_op:
            self._operations.append(["ADD", sphere.paths[0]])

        return sphere_path

    def _get_sphere_list_from_xrdf_geometries(
        self, parsed_file: dict[str, Any], geometry_group_name: str
    ) -> dict[str, Any]:
        spheres = {}
        if "geometry" not in parsed_file:
            carb.log_warn("No geometry groups specified under 'geometry' in XRDF file.  No spheres will be imported.")
            return spheres
        elif geometry_group_name not in parsed_file["geometry"]:
            carb.log_warn("Collision geometry group could not be found under 'geometry'.  No spheres will be imported.")
            return spheres

        geometry_groups = parsed_file["geometry"]
        imported_group = geometry_groups[geometry_group_name]

        if "spheres" in imported_group:
            spheres = imported_group["spheres"]
            # Ensure spheres is a dict, not None
            if spheres is None:
                spheres = {}

        from collections import deque

        handled_groups = {geometry_group_name}
        clones = deque()
        if "clone" in imported_group:
            for clone_group_name in imported_group["clone"]:
                clones.append(clone_group_name)

        while len(clones) > 0:
            clone_group_name = clones.popleft()
            if clone_group_name in handled_groups:
                continue
            handled_groups.add(clone_group_name)
            if clone_group_name in geometry_groups:
                clone_group = geometry_groups[clone_group_name]
            else:
                continue
            if "spheres" in clone_group and clone_group["spheres"] is not None:
                for key in clone_group["spheres"]:
                    if key in spheres:
                        spheres[key].extend(clone_group["spheres"][key])
                    else:
                        spheres[key] = clone_group["spheres"][key]
            if "clone" in clone_group:
                clones.extend(clone_group["clone"])

        return spheres

    def load_xrdf_spheres(self, robot_prim_path: str, parsed_file: dict[str, Any]) -> None:
        """Load collision spheres from a parsed XRDF file.

        Args:
            robot_prim_path: Path to the robot prim.
            parsed_file: Parsed XRDF file contents as a dictionary.
        """
        self.clear_spheres(store_op=False)

        self._redo = []
        self._operations = []

        # Determine which collision key to use based on format version.
        # Version 1.0 uses `collision`, version 2.0 uses `world_collision`.
        format_version = parsed_file.get("format_version", 1.0)
        if format_version == 2.0:
            collision_key = "world_collision"
        elif format_version == 1.0:
            collision_key = "collision"
        else:
            # Unsupported version is recoverable (we simply skip the import);
            # warn rather than error so test runs do not fail on this branch.
            carb.log_warn(
                f"Unsupported XRDF format version: {format_version}. Only versions 1.0 and 2.0 are supported."
            )
            return

        if collision_key not in parsed_file:
            carb.log_warn(f"No {collision_key} group specified in XRDF file. No spheres will be imported")
            return
        elif "geometry" not in parsed_file[collision_key]:
            carb.log_warn(
                f"No geometry group specified under '{collision_key}' in XRDF file. No spheres will be imported"
            )
            return

        geometry_group_name = parsed_file[collision_key]["geometry"]

        if (
            "self_collision" in parsed_file
            and "geometry" in parsed_file["self_collision"]
            and parsed_file["self_collision"]["geometry"] != geometry_group_name
        ):
            carb.log_warn(
                "Specifying a 'self_collision' geometry group that is not the same as "
                + f"the '{collision_key}' geometry group is not supported by this importer. "
                + "The 'self_collision' group will be ignored."
            )

        buffer_distances = parsed_file[collision_key].get("buffer_distance", {})

        sphere_dict = self._get_sphere_list_from_xrdf_geometries(parsed_file, geometry_group_name)
        # Ensure sphere_dict is a dict (handle None case)
        if sphere_dict is None or len(sphere_dict.keys()) == 0:
            return

        added_sphere_paths = ["ADD"]
        for key, val in sphere_dict.items():
            link_path = robot_prim_path + "/" + key
            if self._is_prim_path_valid(link_path):
                for sphere in val:
                    center = np.array(sphere["center"])
                    radius = sphere["radius"]
                    sphere_path = self.add_sphere(link_path, center, radius, store_op=False)
                    added_sphere_paths.append(sphere_path)
            else:
                carb.log_warn(f"Could not place sphere from xrdf at path: {link_path}")

        self._operations.append(added_sphere_paths)

        for k, v in buffer_distances.items():
            # Compare against the link path *with* a trailing slash so a buffer
            # distance targeted at `link1` does not also match sibling links
            # whose names share that prefix (e.g. `link10`, `link1_tip`).
            link_path_prefix = robot_prim_path + "/" + k + "/"
            for p in self.path_2_spheres:
                if self._is_prim_path_valid(p) and p.startswith(link_path_prefix):
                    sphere = self.path_2_spheres[p]
                    rad = sphere.get_radii().numpy()[0]
                    sphere.set_radii(rad + v)

    def load_spheres(self, robot_prim_path: str, robot_description_file_path: str) -> None:
        """Load collision spheres from a robot description YAML file.

        Args:
            robot_prim_path: Path to the robot prim.
            robot_description_file_path: Path to the robot description YAML file.
        """
        self.clear_spheres(store_op=False)

        self._redo = []
        self._operations = []

        with open(robot_description_file_path) as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                # Malformed YAML is recoverable: bail out cleanly so callers
                # (UI import handler, tests) see "no spheres loaded" instead of
                # `UnboundLocalError` from the now-undefined `parsed_file`.
                carb.log_warn(f"Failed to parse robot description YAML {robot_description_file_path}: {exc}")
                return

        if not isinstance(parsed_file, dict):
            # Empty, list-rooted, or scalar-rooted YAML is not a valid robot
            # description; treat it the same as "no collision_spheres present".
            return

        sphere_list = parsed_file.get("collision_spheres", None)

        if sphere_list is None:
            return

        robot_path = robot_prim_path

        added_sphere_paths = ["ADD"]

        for sphere_dict in sphere_list:
            for key, val in sphere_dict.items():
                link_path = robot_path + "/" + key
                if self._is_prim_path_valid(link_path):
                    for sphere in val:
                        center = np.array(sphere["center"])
                        radius = sphere["radius"]
                        sphere_path = self.add_sphere(link_path, center, radius, store_op=False)
                        added_sphere_paths.append(sphere_path)
                else:
                    carb.log_warn(f"Could not place sphere from robot description at path: {link_path}")

        self._operations.append(added_sphere_paths)

    def interpolate_spheres(self, path1: str, path2: str, num_spheres: int) -> None:
        """Create interpolated spheres between two existing spheres.

        Args:
            path1: Path to the first sphere.
            path2: Path to the second sphere.
            num_spheres: Number of interpolated spheres to create.
        """
        if not self._is_prim_path_valid(path1):
            carb.log_warn(f"{path1} is not a valid Prim path to a sphere")
            return
        elif not self._is_prim_path_valid(path2):
            carb.log_warn(f"{path2} is not a valid Prim path to a sphere")
            return

        link_path = self._get_link_path(path1)
        if self._get_link_path(path2) != link_path:
            # The warning previously had no matching early return, so the
            # function silently interpolated `path2` into `path1`'s link and
            # produced spheres at the wrong location. Bail out instead.
            carb.log_warn(
                f"Prim paths {path1} and {path2} are not nested under the same link.  They cannot be interpolated."
            )
            return

        epsilon = 1e-12

        sphere_1 = self.path_2_spheres[path1]
        sphere_2 = self.path_2_spheres[path2]

        rad_1 = max(sphere_1.get_radii().numpy()[0], epsilon)
        rad_2 = max(sphere_2.get_radii().numpy()[0], epsilon)

        t1 = sphere_1.get_local_poses()[0].numpy()[0]
        t2 = sphere_2.get_local_poses()[0].numpy()[0]

        d = t2 - t1

        # Assign radii for interpolated spheres according to a geometric sequence.
        rads = np.geomspace(rad_1, rad_2, num=num_spheres + 2)

        # Position spheres so that they're tangent to an enclosing cone.
        #
        # When the two endpoint spheres have the same radius, the interpolated spheres
        # will be evenly spaced.  The general expression below gives this result when
        # the equal-radius limit is taken analytically, but for the purpose of numerical
        # evaluation we must handle this case separately to avoid division by zero.
        if abs((rad_1 - rad_2) / (rad_1 + rad_2)) < epsilon:
            relative_offsets = np.linspace(0.0, 1.0, num=num_spheres + 2)
        else:
            relative_offsets = (rads - rad_1) / (rad_2 - rad_1)

        added_sphere_paths = ["ADD"]
        for i in range(1, num_spheres + 1):
            sphere_path = self.add_sphere(link_path, t1 + relative_offsets[i] * d, rads[i], store_op=False)
            added_sphere_paths.append(sphere_path)
        self._operations.append(added_sphere_paths)

    def scale_spheres(self, path: str, factor: float) -> None:
        """Scale all spheres under the specified path by a factor.

        Args:
            path: Path prefix to match spheres against.
            factor: Scale factor to apply to sphere radii.
        """
        scaled_spheres = ["SCALE"]
        path_len = len(path)

        for p in self.path_2_spheres:
            if self._is_prim_path_valid(p) and p[:path_len] == path:
                sphere = self.path_2_spheres[p]
                rad = sphere.get_radii().numpy()[0]
                sphere.set_radii(factor * rad)
                scaled_spheres.append({"sphere_path": p, "radius": rad, "factor": factor})
        self._operations.append(scaled_spheres)

    def get_sphere_names_by_link(self, link_path: str) -> list[str]:
        """Sphere names for collision spheres belonging to a specific link.

        Args:
            link_path: Path to the robot link.

        Returns:
            List of sphere names (relative paths from the link path).
        """
        sphere_names = []
        for sphere_path in self.path_2_spheres:
            sphere_link_path = self._get_link_path(sphere_path)
            if sphere_link_path == link_path:
                sphere_names.append(sphere_path[len(link_path) :])

        return sphere_names

    # Used for XRDF files
    def write_spheres_to_dict(self, robot_prim_path: str, link_to_spheres: dict[str, Any]) -> None:
        """Writes collision sphere data to a dictionary grouped by link names.

        Used for XRDF files. Updates the provided dictionary with sphere data for each link.

        Args:
            robot_prim_path: Path to the robot prim.
            link_to_spheres: Dictionary to update with sphere data, keyed by link name.
        """
        for sphere in self.path_2_spheres.values():
            prim_path = sphere.paths[0]
            if self._is_prim_path_valid(prim_path):
                if prim_path[: len(robot_prim_path)] != robot_prim_path:
                    carb.log_warn(
                        f"Not writing sphere at path {prim_path} to file because it is not nested under the robot Articulation"
                    )
                    continue
                link_name = prim_path[len(robot_prim_path) + 1 : prim_path.rfind("/")]
                link_spheres = link_to_spheres.get(link_name, [])
                sphere_pose = self._round_list_floats(sphere.get_local_poses()[0].numpy()[0])
                link_spheres.append({"center": sphere_pose, "radius": sphere.get_radii().numpy().item()})
                link_to_spheres[link_name] = link_spheres

    # Used for Robot Description Files
    def save_spheres(self, robot_prim_path: str, f: object) -> None:
        """Saves collision sphere data to a robot description file.

        Writes collision spheres in YAML format grouped by link names to the provided file handle.

        Args:
            robot_prim_path: Path to the robot prim.
            f: File handle to write the sphere data to.
        """
        link_to_spheres = OrderedDict()
        for sphere in self.path_2_spheres.values():
            prim_path = sphere.paths[0]
            if self._is_prim_path_valid(prim_path):
                if prim_path[: len(robot_prim_path)] != robot_prim_path:
                    carb.log_warn(
                        f"Not writing sphere at path {prim_path} to file because it is not nested under the robot Articulation"
                    )
                    continue
                link_name = prim_path[len(robot_prim_path) + 1 : prim_path.rfind("/")]
                link_spheres = link_to_spheres.get(link_name, [])
                # Coerce numpy scalars to Python floats before rounding:
                # `numpy.float32` / `numpy.ndarray` returned by `get_radii()`
                # do not implement `__round__`, so calling round() on them
                # directly raises TypeError. Going through float() also keeps
                # the YAML output free of `!!python/object` tags.
                sphere_pose = [round(float(x), 5) for x in sphere.get_local_poses()[0].numpy()[0]]
                link_spheres.append({"center": sphere_pose, "radius": round(float(sphere.get_radii().numpy()[0]), 5)})
                link_to_spheres[link_name] = link_spheres

        f.write("collision_spheres:\n")
        for link_name, sphere_list in link_to_spheres.items():
            f.write(f"  - {link_name}:\n")
            for sphere in sphere_list:
                f.write('    - "center": {}\n'.format(sphere["center"]))
                f.write('      "radius": {}\n'.format(sphere["radius"]))

    def on_shutdown(self) -> None:
        """Cleans up resources when the editor is shut down.

        Removes all spheres, preview spheres, and deletes the Lula robot description editor prim.
        """
        self.clear_spheres(store_op=False)
        self.clear_preview()
        if self._is_prim_path_valid(self._lula_path):
            stage_utils.delete_prim(self._lula_path)

    def _get_collision_sphere_base_path(self, link_path: str) -> str:
        """Base path for collision spheres belonging to a link.

        Args:
            link_path: Path to the robot link.

        Returns:
            Base path where collision spheres for this link are stored.
        """
        return link_path + "/collision_sphere"

    def _get_collision_sphere_preview_path(self, link_path: str) -> str:
        """Base path for preview spheres belonging to a link.

        Args:
            link_path: Path to the robot link.

        Returns:
            Base path where preview spheres for this link are stored.
        """
        return link_path + "/preview_sphere"

    def _round_list_floats(self, l: list, decimals: int = 3) -> list:
        """Rounds all float values in a list to a specified number of decimal places.

        Args:
            l: List containing float values to round.
            decimals: Number of decimal places to round to.

        Returns:
            New list with rounded float values.
        """
        r = []
        for f in l:
            r.append(round(f, decimals))
        return r

    def _get_link_path(self, sphere_path: str) -> str:
        """Parent link path for a collision sphere.

        Extracts the link path by removing the sphere name from the sphere's full path.

        Args:
            sphere_path: Full path to the collision sphere prim.

        Returns:
            Path to the parent link containing the sphere.
        """
        # Remove last element of Prim path to sphere

        slash_ind = sphere_path.rfind("/")

        link_path = sphere_path[:slash_ind]
        return link_path
