# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide utility functions for Infinigen synthetic data generation."""

import math
import os
import re
from itertools import chain

import numpy as np
import omni.client
import omni.kit.app
import omni.kit.commands
import omni.physics.core
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.semantics import add_labels, remove_labels
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


def add_colliders(root_prim: Usd.Prim, approximation_type: str = "convexHull") -> None:
    """Add collision attributes to mesh and geometry primitives under the root prim."""
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Gprim):
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)

        if desc_prim.IsA(UsdGeom.Mesh):
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set(approximation_type)


def add_rigid_body(prim: Usd.Prim, disable_gravity: bool = False, ensure_mass: bool = False) -> None:
    """Apply rigid body physics, optionally ensuring a valid mass property exists (defaults to 1.0 kg)."""
    rep.functional.physics.apply_rigid_body(prim, disableGravity=disable_gravity)
    if not ensure_mass:
        return
    if not prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI(prim)
    mass_attr = mass_api.GetMassAttr()
    if not mass_attr or mass_attr.Get() is None or mass_attr.Get() <= 0:
        mass_api.CreateMassAttr(1.0)


def get_random_pose_on_sphere(
    origin: tuple[float, float, float],
    radius_range: tuple[float, float],
    polar_angle_range: tuple[float, float],
    camera_forward_axis: tuple[float, float, float] = (0, 0, -1),
    rng: np.random.Generator = None,
) -> tuple[Gf.Vec3d, Gf.Quatf]:
    """Generate a random pose on a sphere looking at the origin, with specified radius and polar angle ranges."""
    if rng is None:
        rng = np.random.default_rng()

    # https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/reference_conventions.html
    # Convert degrees to radians for polar angles (theta)
    polar_angle_min_rad = math.radians(polar_angle_range[0])
    polar_angle_max_rad = math.radians(polar_angle_range[1])

    # Generate random spherical coordinates
    radius = rng.uniform(radius_range[0], radius_range[1])
    polar_angle = rng.uniform(polar_angle_min_rad, polar_angle_max_rad)
    azimuthal_angle = rng.uniform(0, 2 * math.pi)

    # Convert spherical coordinates to Cartesian coordinates
    x = radius * math.sin(polar_angle) * math.cos(azimuthal_angle)
    y = radius * math.sin(polar_angle) * math.sin(azimuthal_angle)
    z = radius * math.cos(polar_angle)

    # Calculate the location in 3D space
    location = Gf.Vec3d(origin[0] + x, origin[1] + y, origin[2] + z)

    # Calculate direction vector from camera to look_at point
    direction = Gf.Vec3d(origin) - location
    direction_normalized = direction.GetNormalized()

    # Calculate rotation from forward direction (rotateFrom) to direction vector (rotateTo)
    rotation = Gf.Rotation(Gf.Vec3d(camera_forward_axis), direction_normalized)
    orientation = Gf.Quatf(rotation.GetQuat())

    return location, orientation


def randomize_camera_poses(
    cameras: list[Usd.Prim],
    targets: list[Usd.Prim],
    distance_range: tuple[float, float],
    polar_angle_range: tuple[float, float] = (0, 180),
    look_at_offset: tuple[float, float] = (-0.1, 0.1),
    rng: np.random.Generator = None,
) -> None:
    """Randomize the poses of cameras to look at random targets with adjustable distance and offset."""
    for cam in cameras:
        # Get a random target asset to look at
        target_asset = targets[rng.integers(len(targets))]

        # Add a look_at offset so the target is not always in the center of the camera view
        target_loc = target_asset.GetAttribute("xformOp:translate").Get()
        target_loc = (
            target_loc[0] + rng.uniform(look_at_offset[0], look_at_offset[1]),
            target_loc[1] + rng.uniform(look_at_offset[0], look_at_offset[1]),
            target_loc[2] + rng.uniform(look_at_offset[0], look_at_offset[1]),
        )

        # Generate random camera pose
        loc, quat = get_random_pose_on_sphere(target_loc, distance_range, polar_angle_range, rng=rng)

        # Set the camera's transform attributes to the generated location and orientation
        rep.functional.modify.pose(cam, position_value=loc, rotation_value=quat)


def get_usd_paths_from_folder(
    folder_path: str, recursive: bool = True, usd_paths: list[str] = None, skip_keywords: list[str] = None
) -> list[str]:
    """Retrieve USD file paths from a folder, optionally searching recursively and filtering by keywords."""
    if usd_paths is None:
        usd_paths = []
    skip_keywords = skip_keywords or []

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    if not ext_manager.is_extension_enabled("omni.client"):
        ext_manager.set_extension_enabled_immediate("omni.client", True)

    result, entries = omni.client.list(folder_path)
    if result != omni.client.Result.OK:
        print(f"[SDG] Error: Could not list assets in path: {folder_path}")
        return usd_paths

    for entry in entries:
        if any(keyword.lower() in entry.relative_path.lower() for keyword in skip_keywords):
            continue
        _, ext = os.path.splitext(entry.relative_path)
        if ext in [".usd", ".usda", ".usdc"]:
            path_posix = os.path.join(folder_path, entry.relative_path).replace("\\", "/")
            usd_paths.append(path_posix)
        elif recursive and entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
            sub_folder = os.path.join(folder_path, entry.relative_path).replace("\\", "/")
            get_usd_paths_from_folder(sub_folder, recursive=recursive, usd_paths=usd_paths, skip_keywords=skip_keywords)

    return usd_paths


def get_usd_paths(
    files: list[str] = None, folders: list[str] = None, skip_folder_keywords: list[str] = None
) -> list[str]:
    """Retrieve USD paths from specified files and folders, optionally filtering out specific folder keywords."""

    def resolve_path(path: str, assets_root: str, is_folder: bool = False) -> str:
        """Resolve path to full URL: remote URLs and existing local paths used as-is, otherwise prefixed with assets_root."""
        # Remote URLs - use as-is
        if path.startswith(("omniverse://", "http://", "https://", "file://")):
            return path
        # Windows absolute path (e.g., C:\path or C:/path) - use as-is
        if len(path) > 2 and path[1] == ":" and path[2] in ("/", "\\"):
            return path
        # Local absolute path that exists - use as-is
        if path.startswith("/") and (os.path.isfile(path) or (is_folder and os.path.isdir(path))):
            return path
        # Nucleus relative path - prepend assets root
        return assets_root + path

    files = files or []
    folders = folders or []
    skip_folder_keywords = skip_folder_keywords or []

    assets_root_path = get_assets_root_path()
    env_paths = []

    for file_path in files:
        env_paths.append(resolve_path(file_path, assets_root_path, is_folder=False))

    for folder_path in folders:
        resolved_folder = resolve_path(folder_path, assets_root_path, is_folder=True)
        env_paths.extend(get_usd_paths_from_folder(resolved_folder, recursive=True, skip_keywords=skip_folder_keywords))

    return env_paths


def load_env(usd_path: str, prim_path: str, remove_existing: bool = True) -> Usd.Prim:
    """Load an environment from a USD file into the stage at the specified prim path, optionally removing any existing prim."""
    stage = omni.usd.get_context().get_stage()

    # Remove existing prim if specified
    if remove_existing and stage.GetPrimAtPath(prim_path):
        omni.kit.commands.execute("DeletePrimsCommand", paths=[prim_path])

    root_prim = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    return root_prim


def add_colliders_to_env(root_path: str | None = None, approximation_type: str = "none") -> None:
    """Add colliders to all mesh prims within the specified root path in the stage."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPseudoRoot() if root_path is None else stage.GetPrimAtPath(root_path)

    for prim in Usd.PrimRange(prim):
        if prim.IsA(UsdGeom.Mesh):
            add_colliders(prim, approximation_type)


def find_matching_prims(
    match_strings: list[str], root_path: str | None = None, prim_type: str | None = None, first_match_only: bool = False
) -> Usd.Prim | list[Usd.Prim] | None:
    """Find prims matching specified strings, with optional type filtering and single match return."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPseudoRoot() if root_path is None else stage.GetPrimAtPath(root_path)

    matching_prims = []
    for prim in Usd.PrimRange(root_prim):
        if any(match in str(prim.GetPath()) for match in match_strings):
            if prim_type is None or prim.GetTypeName() == prim_type:
                if first_match_only:
                    return prim
                matching_prims.append(prim)

    return matching_prims if not first_match_only else None


def hide_matching_prims(match_strings: list[str], root_path: str | None = None, prim_type: str | None = None) -> None:
    """Set visibility of prims matching specified strings to 'invisible' within the root path."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPseudoRoot() if root_path is None else stage.GetPrimAtPath(root_path)

    for prim in Usd.PrimRange(root_prim):
        if prim_type is None or prim.GetTypeName() == prim_type:
            if any(match in str(prim.GetPath()) for match in match_strings):
                prim.GetAttribute("visibility").Set("invisible")


def setup_env(root_path: str | None = None, approximation_type: str = "none", hide_top_walls: bool = False) -> None:
    """Set up the environment with colliders, ceiling light adjustments, and optional top wall hiding."""
    # Fix ceiling lights: meshes are blocking the light and need to be set to invisible
    ceiling_light_meshes = find_matching_prims(["001_SPLIT_GLA"], root_path, "Xform")
    for light_mesh in ceiling_light_meshes:
        light_mesh.GetAttribute("visibility").Set("invisible")

    # Hide ceiling light meshes for lighting fix
    hide_matching_prims(["001_SPLIT_GLA"], root_path, "Xform")

    # Hide top walls for better debug view, if specified
    if hide_top_walls:
        hide_matching_prims(["_exterior", "_ceiling"], root_path)

    # Add colliders to the environment
    add_colliders_to_env(root_path, approximation_type)

    # Fix dining table collision by setting it to a bounding cube approximation
    table_prim = find_matching_prims(
        match_strings=["TableDining"], root_path=root_path, prim_type="Xform", first_match_only=True
    )
    if table_prim is not None:
        add_colliders(table_prim, approximation_type="boundingCube")
    else:
        print("[SDG] Warning: Could not find dining table prim in the environment")


def create_shape_distractors(
    num_distractors: int,
    shape_types: list[str],
    root_path: str,
    gravity_disabled_chance: float,
    rng: np.random.Generator = None,
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Create shape distractors with optional gravity settings, returning lists of floating and falling shapes."""
    if rng is None:
        rng = np.random.default_rng()
    stage = omni.usd.get_context().get_stage()
    floating_shapes = []
    falling_shapes = []
    for _ in range(num_distractors):
        rand_shape = shape_types[rng.integers(len(shape_types))]
        disable_gravity = rng.random() < gravity_disabled_chance
        name_prefix = "floating_" if disable_gravity else "falling_"
        prim_path = omni.usd.get_stage_next_free_path(stage, f"{root_path}/{name_prefix}{rand_shape}", False)
        prim = stage.DefinePrim(prim_path, rand_shape.capitalize())
        add_colliders(prim)
        add_rigid_body(prim, disable_gravity=disable_gravity, ensure_mass=True)
        (floating_shapes if disable_gravity else falling_shapes).append(prim)
    return floating_shapes, falling_shapes


def load_shape_distractors(
    shape_distractors_config: dict, rng: np.random.Generator = None
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Load shape distractors based on configuration, returning lists of floating and falling shapes."""
    num_shapes = shape_distractors_config.get("num", 0)
    shape_types = shape_distractors_config.get("shape_types", ["capsule", "cone", "cylinder", "sphere", "cube"])
    shape_gravity_disabled_chance = shape_distractors_config.get("gravity_disabled_chance", 0.0)
    return create_shape_distractors(num_shapes, shape_types, "/Distractors", shape_gravity_disabled_chance, rng)


def create_mesh_distractors(
    num_distractors: int,
    mesh_urls: list[str],
    root_path: str,
    gravity_disabled_chance: float,
    rng: np.random.Generator = None,
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Create mesh distractors from specified URLs with optional gravity settings."""
    if rng is None:
        rng = np.random.default_rng()
    stage = omni.usd.get_context().get_stage()
    floating_meshes = []
    falling_meshes = []
    for _ in range(num_distractors):
        rand_mesh_url = mesh_urls[rng.integers(len(mesh_urls))]
        disable_gravity = rng.random() < gravity_disabled_chance
        name_prefix = "floating_" if disable_gravity else "falling_"
        prim_name = os.path.basename(rand_mesh_url).split(".")[0]
        prim_path = omni.usd.get_stage_next_free_path(stage, f"{root_path}/{name_prefix}{prim_name}", False)
        try:
            prim = add_reference_to_stage(usd_path=rand_mesh_url, prim_path=prim_path)
        except Exception as e:
            print(f"[SDG] Error: Failed to load mesh distractor '{rand_mesh_url}': {e}")
            continue
        add_colliders(prim)
        add_rigid_body(prim, disable_gravity=disable_gravity, ensure_mass=True)
        (floating_meshes if disable_gravity else falling_meshes).append(prim)
    return floating_meshes, falling_meshes


def load_mesh_distractors(
    mesh_distractors_config: dict, rng: np.random.Generator = None
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Load mesh distractors based on configuration, returning lists of floating and falling meshes."""
    num_meshes = mesh_distractors_config.get("num", 0)
    mesh_gravity_disabled_chance = mesh_distractors_config.get("gravity_disabled_chance", 0.0)
    mesh_folders = mesh_distractors_config.get("folders", [])
    mesh_files = mesh_distractors_config.get("files", [])
    mesh_urls = get_usd_paths(
        files=mesh_files, folders=mesh_folders, skip_folder_keywords=["material", "texture", ".thumbs"]
    )
    floating_meshes, falling_meshes = create_mesh_distractors(
        num_meshes, mesh_urls, "/Distractors", mesh_gravity_disabled_chance, rng
    )
    for prim in chain(floating_meshes, falling_meshes):
        remove_labels(prim, include_descendants=True)
    return floating_meshes, falling_meshes


def create_auto_labeled_assets(
    num_assets: int,
    asset_urls: list[str],
    root_path: str,
    regex_replace_pattern: str,
    regex_replace_repl: str,
    gravity_disabled_chance: float,
    rng: np.random.Generator = None,
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Create assets with automatic labels, applying optional gravity settings."""
    if rng is None:
        rng = np.random.default_rng()
    stage = omni.usd.get_context().get_stage()
    floating_assets = []
    falling_assets = []
    for _ in range(num_assets):
        asset_url = asset_urls[rng.integers(len(asset_urls))]
        disable_gravity = rng.random() < gravity_disabled_chance
        name_prefix = "floating_" if disable_gravity else "falling_"
        basename = os.path.basename(asset_url)
        name_without_ext = os.path.splitext(basename)[0]
        label = re.sub(regex_replace_pattern, regex_replace_repl, name_without_ext)
        prim_path = omni.usd.get_stage_next_free_path(stage, f"{root_path}/{name_prefix}{label}", False)
        try:
            prim = add_reference_to_stage(usd_path=asset_url, prim_path=prim_path)
        except Exception as e:
            print(f"[SDG] Error: Failed to load asset '{asset_url}': {e}")
            continue
        add_colliders(prim)
        add_rigid_body(prim, disable_gravity=disable_gravity, ensure_mass=True)
        remove_labels(prim, include_descendants=True)
        add_labels(prim, labels=[label], instance_name="class")
        (floating_assets if disable_gravity else falling_assets).append(prim)
    return floating_assets, falling_assets


def load_auto_labeled_assets(
    auto_label_config: dict, rng: np.random.Generator = None
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Load auto-labeled assets based on configuration, returning lists of floating and falling assets."""
    num_assets = auto_label_config.get("num", 0)
    gravity_disabled_chance = auto_label_config.get("gravity_disabled_chance", 0.0)
    assets_files = auto_label_config.get("files", [])
    assets_folders = auto_label_config.get("folders", [])
    assets_urls = get_usd_paths(
        files=assets_files, folders=assets_folders, skip_folder_keywords=["material", "texture", ".thumbs"]
    )
    regex_replace_pattern = auto_label_config.get("regex_replace_pattern", "")
    regex_replace_repl = auto_label_config.get("regex_replace_repl", "")
    return create_auto_labeled_assets(
        num_assets,
        assets_urls,
        "/Assets",
        regex_replace_pattern,
        regex_replace_repl,
        gravity_disabled_chance,
        rng,
    )


def create_labeled_assets(
    num_assets: int,
    asset_url: str,
    label: str,
    root_path: str,
    gravity_disabled_chance: float,
    rng: np.random.Generator = None,
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Create labeled assets with optional gravity settings, returning lists of floating and falling assets."""
    if rng is None:
        rng = np.random.default_rng()
    stage = omni.usd.get_context().get_stage()
    assets_root_path = get_assets_root_path()
    asset_url = (
        asset_url
        if asset_url.startswith(("omniverse://", "http://", "https://", "file://"))
        else assets_root_path + asset_url
    )
    floating_assets = []
    falling_assets = []
    for _ in range(num_assets):
        disable_gravity = rng.random() < gravity_disabled_chance
        name_prefix = "floating_" if disable_gravity else "falling_"
        prim_path = omni.usd.get_stage_next_free_path(stage, f"{root_path}/{name_prefix}{label}", False)

        prim = add_reference_to_stage(usd_path=asset_url, prim_path=prim_path)
        add_colliders(prim)
        add_rigid_body(prim, disable_gravity=disable_gravity, ensure_mass=True)
        remove_labels(prim, include_descendants=True)
        add_labels(prim, labels=[label], instance_name="class")
        (floating_assets if disable_gravity else falling_assets).append(prim)
    return floating_assets, falling_assets


def load_manual_labeled_assets(
    manual_labeled_assets_config: list[dict], rng: np.random.Generator = None
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Load manually labeled assets based on configuration, returning lists of floating and falling assets."""
    labeled_floating_assets = []
    labeled_falling_assets = []
    for labeled_asset_config in manual_labeled_assets_config:
        asset_url = labeled_asset_config.get("url", "")
        asset_label = labeled_asset_config.get("label", "")
        num_assets = labeled_asset_config.get("num", 0)
        gravity_disabled_chance = labeled_asset_config.get("gravity_disabled_chance", 0.0)
        floating_assets, falling_assets = create_labeled_assets(
            num_assets,
            asset_url,
            asset_label,
            "/Assets",
            gravity_disabled_chance,
            rng,
        )
        labeled_floating_assets.extend(floating_assets)
        labeled_falling_assets.extend(falling_assets)
    return labeled_floating_assets, labeled_falling_assets


def resolve_scale_issues_with_metrics_assembler() -> None:
    """Enable and execute metrics assembler to resolve scale issues in the stage."""
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    if not ext_manager.is_extension_enabled("omni.usd.metrics.assembler"):
        ext_manager.set_extension_enabled_immediate("omni.usd.metrics.assembler", True)
    from omni.metrics.assembler.core import get_metrics_assembler_interface

    stage_id = omni.usd.get_context().get_stage_id()
    get_metrics_assembler_interface().resolve_stage(stage_id)


def get_matching_prim_location(match_string: str, root_path: str | None = None) -> tuple[float, float, float]:
    """Return the translation of the first prim matching the given string."""
    prim = find_matching_prims(
        match_strings=[match_string], root_path=root_path, prim_type="Xform", first_match_only=True
    )
    if prim is None:
        print("[SDG] Warning: Could not find matching prim, returning (0, 0, 0)")
        return (0, 0, 0)
    if prim.HasAttribute("xformOp:translate"):
        return prim.GetAttribute("xformOp:translate").Get()
    elif prim.HasAttribute("xformOp:transform"):
        return prim.GetAttribute("xformOp:transform").Get().ExtractTranslation()
    else:
        print(f"[SDG] Warning: Could not find location attribute for '{prim.GetPath()}', returning (0, 0, 0)")
        return (0, 0, 0)


def offset_range(
    range_coords: tuple[float, float, float, float, float, float], offset: tuple[float, float, float]
) -> tuple[float, float, float, float, float, float]:
    """Offset the min and max coordinates of a range by the specified offset."""
    return (
        range_coords[0] + offset[0],  # min_x
        range_coords[1] + offset[1],  # min_y
        range_coords[2] + offset[2],  # min_z
        range_coords[3] + offset[0],  # max_x
        range_coords[4] + offset[1],  # max_y
        range_coords[5] + offset[2],  # max_z
    )


def randomize_poses(
    prims: list[Usd.Prim],
    location_range: tuple[float, float, float, float, float, float],
    rotation_range: tuple[float, float],
    scale_range: tuple[float, float],
    rng: np.random.Generator = None,
) -> None:
    """Randomize the location, rotation, and scale of a list of prims within specified ranges."""
    if rng is None:
        rng = np.random.default_rng()
    for prim in prims:
        rand_loc = (
            rng.uniform(location_range[0], location_range[3]),
            rng.uniform(location_range[1], location_range[4]),
            rng.uniform(location_range[2], location_range[5]),
        )
        rand_rot = (
            rng.uniform(rotation_range[0], rotation_range[1]),
            rng.uniform(rotation_range[0], rotation_range[1]),
            rng.uniform(rotation_range[0], rotation_range[1]),
        )
        rand_scale = rng.uniform(scale_range[0], scale_range[1])
        rep.functional.modify.pose(prim, position_value=rand_loc, rotation_value=rand_rot, scale_value=rand_scale)


def get_or_create_physx_scene(prim_path: str = "/PhysicsScene") -> PhysxSchema.PhysxSceneAPI:
    """Get the existing PhysX scene or create a new one at the given prim path.

    Searches the current stage for an existing UsdPhysics.Scene prim. If found, applies
    and returns the PhysxSceneAPI on it. If not found, creates a new physics scene at the
    given prim path and returns the PhysxSceneAPI.

    Args:
        prim_path: The prim path to create a new physics scene at if none exists.

    Returns:
        The PhysxSceneAPI applied to the physics scene prim.
    """
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return PhysxSchema.PhysxSceneAPI.Apply(prim)

    UsdPhysics.Scene.Define(stage, prim_path)
    return PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(prim_path))


# Default GPU collision stack size for the infinigen SDG pipeline (300 MB).
# The PhysX default (64 MB) is insufficient for scenes with many colliders (e.g. 30+ shape
# distractors, 10+ mesh distractors, labeled assets, and environment collision meshes).
# The error message from PhysX recommends at least ~272 MB for a typical infinigen scene,
# so 300 MB provides a comfortable margin.
INFINIGEN_DEFAULT_GPU_COLLISION_STACK_SIZE = 314572800


def configure_physics_scene(physics_config: dict | None = None) -> None:
    """Configure the PhysX scene GPU memory settings for the infinigen SDG pipeline.

    This must be called before running any physics simulation. It sets the GPU collision
    stack size and other GPU memory parameters on the PhysX scene to prevent buffer overflow
    errors when simulating complex scenes with many colliders.

    Args:
        physics_config: Optional dictionary with physics configuration overrides.
            Supported keys (all optional):
            - gpu_collision_stack_size (int): GPU collision stack size in bytes.
              Defaults to INFINIGEN_DEFAULT_GPU_COLLISION_STACK_SIZE (300 MB).
            - gpu_found_lost_pairs_capacity (int): GPU found/lost pairs capacity.
            - gpu_found_lost_aggregate_pairs_capacity (int): GPU found/lost aggregate pairs capacity.
            - gpu_total_aggregate_pairs_capacity (int): GPU total aggregate pairs capacity.
            - gpu_max_rigid_contact_count (int): Maximum rigid body contact count on GPU.
            - gpu_max_rigid_patch_count (int): Maximum rigid body contact patches on GPU.
            - gpu_heap_capacity (int): GPU heap capacity in bytes.
            - gpu_temp_buffer_capacity (int): GPU temporary buffer capacity in bytes.
    """
    physics_config = physics_config or {}
    physx_scene = get_or_create_physx_scene()

    gpu_collision_stack_size = physics_config.get(
        "gpu_collision_stack_size", INFINIGEN_DEFAULT_GPU_COLLISION_STACK_SIZE
    )
    physx_scene.GetGpuCollisionStackSizeAttr().Set(gpu_collision_stack_size)

    # Apply other GPU memory settings if provided
    gpu_settings_map = {
        "gpu_found_lost_pairs_capacity": physx_scene.GetGpuFoundLostPairsCapacityAttr,
        "gpu_found_lost_aggregate_pairs_capacity": physx_scene.GetGpuFoundLostAggregatePairsCapacityAttr,
        "gpu_total_aggregate_pairs_capacity": physx_scene.GetGpuTotalAggregatePairsCapacityAttr,
        "gpu_max_rigid_contact_count": physx_scene.GetGpuMaxRigidContactCountAttr,
        "gpu_max_rigid_patch_count": physx_scene.GetGpuMaxRigidPatchCountAttr,
        "gpu_heap_capacity": physx_scene.GetGpuHeapCapacityAttr,
        "gpu_temp_buffer_capacity": physx_scene.GetGpuTempBufferCapacityAttr,
    }
    for key, attr_getter in gpu_settings_map.items():
        if key in physics_config:
            attr_getter().Set(physics_config[key])

    print(f"[SDG] Configured PhysX scene GPU collision stack size: {gpu_collision_stack_size} bytes")


def run_simulation(num_frames: int, render: bool = True) -> None:
    """Run a simulation for a specified number of frames, optionally without rendering."""
    if render:
        # Start the timeline and advance the app, this will render the physics simulation results every frame
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0)
        timeline.set_end_time(1000000)
        timeline.set_looping(False)
        timeline.play()
        for _ in range(num_frames):
            omni.kit.app.get_app().update()
        timeline.pause()
    else:
        # Run the physics simulation steps without advancing the app
        physx_scene = get_or_create_physx_scene()

        # Get simulation parameters
        physx_dt = 1 / physx_scene.GetTimeStepsPerSecondAttr().Get()
        physics_sim_interface = omni.physics.core.get_physics_simulation_interface()

        # Run physics simulation for each frame
        for _ in range(num_frames):
            physics_sim_interface.simulate(physx_dt, 0)


def register_dome_light_randomizer() -> None:
    """Register a replicator graph randomizer for dome lights using various sky textures."""
    assets_root_path = get_assets_root_path()
    dome_textures = [
        assets_root_path + "/NVIDIA/Assets/Skies/Cloudy/champagne_castle_1_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Cloudy/kloofendal_48d_partly_cloudy_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Clear/evening_road_01_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Clear/mealie_road_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Clear/qwantani_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Evening/evening_road_01_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Night/kloppenheim_02_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Night/moonlit_golf_4k.hdr",
    ]
    with rep.trigger.on_custom_event(event_name="randomize_dome_lights"):
        rep.create.light(light_type="Dome", texture=rep.distribution.choice(dome_textures))


def register_shape_distractors_color_randomizer(shape_distractors: list[Usd.Prim]) -> None:
    """Register a replicator graph randomizer to change colors of shape distractors."""
    with rep.trigger.on_custom_event(event_name="randomize_shape_distractor_colors"):
        shape_distractors_paths = [prim.GetPath() for prim in shape_distractors]
        shape_distractors_group = rep.create.group(shape_distractors_paths)
        with shape_distractors_group:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))


def randomize_lights(
    lights: list[Usd.Prim],
    location_range: tuple[float, float, float, float, float, float] | None = None,
    color_range: tuple[float, float, float, float, float, float] | None = None,
    intensity_range: tuple[float, float] | None = None,
    rng: np.random.Generator = None,
) -> None:
    """Randomize location, color, and intensity of specified lights within given ranges."""
    if rng is None:
        rng = np.random.default_rng()
    for light in lights:
        # Randomize the location of the light
        if location_range is not None:
            rand_loc = (
                rng.uniform(location_range[0], location_range[3]),
                rng.uniform(location_range[1], location_range[4]),
                rng.uniform(location_range[2], location_range[5]),
            )
            rep.functional.modify.pose(light, position_value=rand_loc)

        # Randomize the color of the light
        if color_range is not None:
            rand_color = (
                rng.uniform(color_range[0], color_range[3]),
                rng.uniform(color_range[1], color_range[4]),
                rng.uniform(color_range[2], color_range[5]),
            )
            light.GetAttribute("inputs:color").Set(rand_color)

        # Randomize the intensity of the light
        if intensity_range is not None:
            rand_intensity = rng.uniform(intensity_range[0], intensity_range[1])
            light.GetAttribute("inputs:intensity").Set(rand_intensity)


def setup_writer(config: dict) -> None:
    """Setup a writer based on configuration settings, initializing with specified arguments."""
    writer_type = config.get("type", None)
    if writer_type is None:
        print("[SDG] Warning: No writer type specified, skipping writer setup")
        return None

    try:
        writer = rep.writers.get(writer_type)
    except Exception as e:
        print(f"[SDG] Error: Writer type '{writer_type}' not found: {e}")
        return None

    writer_kwargs = config.get("kwargs", {})
    if out_dir := writer_kwargs.get("output_dir"):
        # If not an absolute path, make path relative to the current working directory
        if not os.path.isabs(out_dir):
            out_dir = os.path.join(os.getcwd(), out_dir)
            writer_kwargs["output_dir"] = out_dir

    writer.initialize(**writer_kwargs)
    return writer
