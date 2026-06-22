# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide utility functions for scene-based synthetic data generation."""

import math
import os

import carb
import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.bounds import (
    compute_combined_aabb,
    compute_obb,
    create_bbox_cache,
    get_obb_corners,
)
from isaacsim.core.experimental.utils.semantics import add_labels
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, define_prim
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion, quaternion_to_euler_angles
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd, UsdGeom


def setup_writer(config: dict) -> rep.Writer | None:
    """Setup and initialize writer with optional backend support and error handling."""

    def normalize_output_dir(params: dict) -> None:
        """Convert relative output_dir to absolute path."""
        if "output_dir" in params and not os.path.isabs(params["output_dir"]):
            params["output_dir"] = os.path.join(os.getcwd(), params["output_dir"])

    # Get writer from registry
    writer_type = config.get("writer", "BasicWriter")
    if writer_type not in rep.WriterRegistry.get_writers():
        carb.log_error(f"[SDG] Writer type '{writer_type}' not found in registry.")
        return None

    writer = rep.WriterRegistry.get(writer_type)
    writer_kwargs = dict(config.get("writer_config", {}))
    normalize_output_dir(writer_kwargs)

    # Initialize backend if specified
    backend_type = config.get("backend_type")
    backend = None
    if backend_type:
        try:
            backend = rep.backends.get(backend_type)
        except Exception as e:
            carb.log_error(f"[SDG] Backend '{backend_type}' not found: {e}")
            return None

        backend_params = dict(config.get("backend_params", {}))
        normalize_output_dir(backend_params)

        try:
            print(f"[SDG] Backend: {backend_type} | Params: {backend_params}")
            backend.initialize(**backend_params)
        except TypeError as e:
            carb.log_error(f"[SDG] Invalid backend params: {e}")
            return None

    # Initialize writer
    if "output_dir" in writer_kwargs:
        print(f"[SDG] Output: {writer_kwargs['output_dir']}")

    backend_info = f" + {backend_type}" if backend else ""
    print(f"[SDG] Writer: {writer_type}{backend_info} | Config: {writer_kwargs}")

    try:
        if backend:
            writer.initialize(backend=backend, **writer_kwargs)
        else:
            writer.initialize(**writer_kwargs)
    except TypeError as e:
        carb.log_error(f"[SDG] Invalid writer params: {e}")
        return None

    return writer


def simulate_falling_objects(
    forklift_prim: Usd.Prim,
    assets_root_path: str,
    config: dict,
    max_sim_steps: int = 250,
    num_boxes: int = 8,
    rng: np.random.Generator | None = None,
) -> None:
    """Run physics simulation to drop boxes on pallet near forklift."""
    if rng is None:
        rng = np.random.default_rng()

    forklift_transform = omni.usd.get_world_transform_matrix(forklift_prim)
    sim_pallet_offset = Gf.Matrix4d().SetTranslate(Gf.Vec3d(rng.uniform(-1, 1), rng.uniform(-4, -3.6), 0))
    sim_pallet_position = (sim_pallet_offset * forklift_transform).ExtractTranslation()
    sim_pallet_rotation = euler_angles_to_quaternion([0, 0, rng.uniform(0, math.pi)]).numpy()

    sim_pallet_path = "/World/SimulatedPallet"
    sim_pallet = define_prim(sim_pallet_path)
    add_reference_to_stage(assets_root_path + config["pallet"]["url"], sim_pallet_path)
    add_labels(sim_pallet, labels=[config["pallet"]["class"]], taxonomy="class")
    XformPrim(
        sim_pallet_path,
        positions=tuple(sim_pallet_position),
        orientations=sim_pallet_rotation,
        reset_xform_op_properties=True,
    )
    sim_pallet_geom = GeomPrim(f"{str(sim_pallet.GetPrimPath())}/.*", apply_collision_apis=True)
    sim_pallet_geom.set_collision_approximations("boundingCube")

    bbox_cache = create_bbox_cache()
    current_height = bbox_cache.ComputeLocalBound(sim_pallet).GetRange().GetSize()[2] * 1.1

    sim_box_rigid_prims = []
    for box_index in range(num_boxes):
        box_xy_offset = Gf.Vec3d(rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), current_height)
        sim_box_path = f"/World/SimulatedCardbox_{box_index}"
        sim_box = define_prim(sim_box_path)
        add_reference_to_stage(assets_root_path + config["cardbox"]["url"], sim_box_path)
        add_labels(sim_box, labels=[config["cardbox"]["class"]], taxonomy="class")
        XformPrim(
            sim_box_path,
            positions=tuple(sim_pallet_position + box_xy_offset),
            orientations=sim_pallet_rotation,
            reset_xform_op_properties=True,
        )
        current_height += bbox_cache.ComputeLocalBound(sim_box).GetRange().GetSize()[2] * 1.1

        sim_box_geom = GeomPrim(f"{str(sim_box.GetPrimPath())}/.*", apply_collision_apis=True)
        sim_box_geom.set_collision_approximations("convexHull")
        sim_box_rigid_prims.append(RigidPrim(str(sim_box.GetPrimPath())))

    SimulationManager.set_physics_dt(1.0 / 90.0)
    SimulationManager.initialize_physics()

    velocity_threshold = 0.01
    for step in range(max_sim_steps):
        SimulationManager.step()
        if sim_box_rigid_prims:
            top_box_velocity = sim_box_rigid_prims[-1].get_velocities(indices=[0])[0].numpy()
            if np.linalg.norm(top_box_velocity) < velocity_threshold:
                print(f"[SDG] Simulation settled at step {step}")
                break


def setup_camera_bounds(
    pallet_prim: Usd.Prim, forklift_prim: Usd.Prim, pallet_tf: Gf.Matrix4d, forklift_tf: Gf.Matrix4d
) -> dict[str, dict[str, tuple[float, float, float]]]:
    """Calculate camera randomization bounds for pallet, top view, and driver cameras."""
    pallet_pos = pallet_tf.ExtractTranslation()
    pallet_cam_bounds = {
        "min": (pallet_pos[0] - 2, pallet_pos[1] - 2, 2),
        "max": (pallet_pos[0] + 2, pallet_pos[1] + 2, 4),
    }

    forklift_pos = forklift_tf.ExtractTranslation()
    top_cam_bounds = {
        "min": (forklift_pos[0], forklift_pos[1], 9),
        "max": (forklift_pos[0], forklift_pos[1], 11),
    }

    driver_cam_pos = forklift_pos + Gf.Vec3d(0.0, 0.0, 1.9)
    driver_cam_bounds = {
        "min": (driver_cam_pos[0], driver_cam_pos[1], driver_cam_pos[2] - 0.25),
        "max": (driver_cam_pos[0], driver_cam_pos[1], driver_cam_pos[2] + 0.25),
    }

    return {
        "pallet_cam": pallet_cam_bounds,
        "top_cam": top_cam_bounds,
        "driver_cam": driver_cam_bounds,
    }


def create_scatter_plane_for_prim(
    prim: Usd.Prim, prim_tf: Gf.Matrix4d, parent_path: str, scale_factor: float = 0.8, visible: bool = False
) -> Usd.Prim:
    """Create scatter plane sized and aligned to prim surface."""
    bb_cache = create_bbox_cache()
    prim_bbox = bb_cache.ComputeLocalBound(prim)
    prim_bbox.Transform(prim_tf)
    prim_size = prim_bbox.GetRange().GetSize()

    prim_quat = prim_tf.ExtractRotation().GetQuaternion()
    prim_quat_xyzw = (prim_quat.GetReal(), *prim_quat.GetImaginary())
    prim_rotation_deg = quaternion_to_euler_angles(np.array(prim_quat_xyzw), degrees=True).numpy()

    prim_pos = prim_tf.ExtractTranslation()
    scatter_plane_scale = (prim_size[0] * scale_factor, prim_size[1] * scale_factor, 1)
    scatter_plane_pos = prim_pos + Gf.Vec3d(0, 0, prim_size[2])

    scatter_plane = rep.functional.create.plane(
        scale=scatter_plane_scale,
        position=tuple(scatter_plane_pos),
        rotation=tuple(prim_rotation_deg),
        visible=visible,
        parent=parent_path,
    )

    return scatter_plane


def setup_cone_placement_corners(
    forklift_prim: Usd.Prim, bb_cache: UsdGeom.BBoxCache | None = None, scale_factor: float = 1.3
) -> tuple[list[list[float]], tuple[float, float, float]]:
    """Calculate forklift OBB corners for cone placement."""
    if bb_cache is None:
        bb_cache = create_bbox_cache()

    forklift_obb_center, forklift_obb_axes, forklift_obb_extent = compute_obb(forklift_prim, bbox_cache=bb_cache)
    enlarged_extent = (
        forklift_obb_extent[0] * scale_factor,
        forklift_obb_extent[1] * scale_factor,
        forklift_obb_extent[2],
    )
    forklift_obb_corners = get_obb_corners(forklift_obb_center, forklift_obb_axes, enlarged_extent)

    cone_placement_corners = [
        forklift_obb_corners[0].tolist(),
        forklift_obb_corners[2].tolist(),
        forklift_obb_corners[4].tolist(),
        forklift_obb_corners[6].tolist(),
    ]

    forklift_obb_quat = Gf.Matrix3d(forklift_obb_axes).ExtractRotation().GetQuaternion()
    forklift_obb_quat_xyzw = (forklift_obb_quat.GetReal(), *forklift_obb_quat.GetImaginary())
    forklift_rotation_deg = quaternion_to_euler_angles(np.array(forklift_obb_quat_xyzw), degrees=True).numpy()

    return cone_placement_corners, forklift_rotation_deg


def register_lights_graph_randomizer(forklift_prim: Usd.Prim, pallet_prim: Usd.Prim, event_name: str) -> None:
    """Register graph randomizer for sphere lights."""
    bb_cache = create_bbox_cache()
    combined_bounds = compute_combined_aabb([forklift_prim, pallet_prim], bbox_cache=bb_cache)
    light_pos_min = (combined_bounds[0], combined_bounds[1], 6)
    light_pos_max = (combined_bounds[3], combined_bounds[4], 7)

    with rep.trigger.on_custom_event(event_name):
        rep.create.light(
            light_type="Sphere",
            color=rep.distribution.uniform((0.2, 0.1, 0.1), (0.9, 0.8, 0.8)),
            intensity=rep.distribution.uniform(2000, 4000),
            position=rep.distribution.uniform(light_pos_min, light_pos_max),
            scale=rep.distribution.uniform(1, 4),
            count=3,
        )


def register_cardboxes_materials_graph_randomizer(
    cardboxes: list[Usd.Prim], cardbox_material_urls: list[str], event_name: str
) -> None:
    """Register graph randomizer for cardbox materials."""
    cardbox_mesh_paths = []
    for cardbox in cardboxes:
        meshes = [child for child in cardbox.GetChildren() if child.IsA(UsdGeom.Mesh)]
        cardbox_mesh_paths.extend([mesh.GetPrimPath() for mesh in meshes])

    with rep.trigger.on_custom_event(event_name):
        cardbox_mesh_group_node = rep.create.group(cardbox_mesh_paths)
        with cardbox_mesh_group_node:
            rep.randomizer.materials(cardbox_material_urls)
