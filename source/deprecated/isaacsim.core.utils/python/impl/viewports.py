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

# python

"""Deprecated viewport utility functions."""

from __future__ import annotations

from typing import Any

# omniverse
import carb
import numpy as np
import omni
import omni.kit.app
import omni.kit.commands
from isaacsim.core.utils.prims import is_prim_path_valid, set_prim_hide_in_stage_window, set_prim_no_delete

# isaacsim
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, Usd, UsdGeom


def set_camera_view(
    eye: np.array, target: np.array, camera_prim_path: str = "/OmniverseKit_Persp", viewport_api: object = None
) -> None:
    """Set the location and target for a camera prim in the stage given its path.

    Args:
        eye: Location of camera.
        target: Location of camera target.
        camera_prim_path: Path to camera prim being set.
        viewport_api: Viewport API instance. If None, uses the active viewport.
    """
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        if viewport_api is None:
            viewport_api = get_active_viewport()
    except ImportError:
        carb.log_warn("omni.kit.viewport.utility needs to be enabled before using this function")
        return

    if viewport_api is None:
        carb.log_warn("could not get active viewport, cannot set camera view")
        return

    # get all inputs
    camera_position = np.asarray(eye, dtype=np.double)
    camera_target = np.asarray(target, dtype=np.double)
    prim = viewport_api.stage.GetPrimAtPath(camera_prim_path)

    # check if center of interest property exists, create if not
    coi_prop = prim.GetProperty("omni:kit:centerOfInterest")
    if not coi_prop or not coi_prop.IsValid():
        prim.CreateAttribute(
            "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Vector3d, True, Sdf.VariabilityUniform
        ).Set(Gf.Vec3d(0, 0, -10))

    # set camera prim position
    camera_state = ViewportCameraState(camera_prim_path, viewport_api)
    camera_state.set_position_world(Gf.Vec3d(camera_position[0], camera_position[1], camera_position[2]), True)

    # if camera target is not directly above or below camera, set target using omni.kit.viewport.utility
    if (camera_target[0:2] != camera_position[0:2]).any():
        camera_state.set_target_world(Gf.Vec3d(camera_target[0], camera_target[1], camera_target[2]), True)
    else:
        # if camera has an orient property, set it to look at target
        if prim.GetAttribute("xformOp:orient"):
            rotate_prop = prim.GetAttribute("xformOp:orient")
            # set orientation quaternion based on if camera is looking up or down
            quat = [
                0 if camera_target[2] >= camera_position[2] else 1,
                0,
                1 if camera_target[2] >= camera_position[2] else 0,
                0,
            ]

            # save new rotate property as double or float quaternion based on original rotate property type
            if rotate_prop.GetTypeName() == Sdf.ValueTypeNames.Quatd:
                new_rotate = Gf.Quatd(*quat)
            elif rotate_prop.GetTypeName() == Sdf.ValueTypeNames.Quatf:
                new_rotate = Gf.Quatf(*quat)
            else:
                # if rotate property is not float or double quaternion, log warning and return
                carb.log_warn("unknown orient type")
                return
        else:
            # else, use rotate property to set camera orientation
            # use up_down to determine if camera is looking up or down
            up_down = 180 if camera_target[2] >= camera_position[2] else 0

            # set potential rotate properties based on which rotate property could be used
            xyz_zyx = [0, up_down, 0]
            xzy_zxy = [0, 0, up_down]
            yxz_yzx = [up_down, 0, 0]

            # check which rotate property is being used
            rot = (
                xyz_zyx
                if prim.GetAttribute("xformOp:rotateXYZ") or prim.GetAttribute("xformOp:rotateZYX")
                else (
                    xzy_zxy
                    if prim.GetAttribute("xformOp:rotateXZY") or prim.GetAttribute("xformOp:rotateZXY")
                    else (
                        yxz_yzx
                        if prim.GetAttribute("xformOp:rotateYXZ") or prim.GetAttribute("xformOp:rotateYZX")
                        else None
                    )
                )
            )

            # if no rotate property is found, log warning and return
            if rot is None:
                carb.log_warn("no orient or rotate attributes found")
                return

            # set new rotate property
            rotate_prop = (
                prim.GetAttribute("xformOp:rotateXYZ")
                or prim.GetAttribute("xformOp:rotateXZY")
                or prim.GetAttribute("xformOp:rotateZXY")
                or prim.GetAttribute("xformOp:rotateZYX")
                or prim.GetAttribute("xformOp:rotateYXZ")
                or prim.GetAttribute("xformOp:rotateYZX")
            )

            # save new rotate property as double or float vector based on original rotate property type
            if rotate_prop is None:
                # if no rotate property is found, log warning and return
                carb.log_warn("no orient or rotate attributes found")
                return
            elif rotate_prop.GetTypeName() == Sdf.ValueTypeNames.Double3:
                new_rotate = Gf.Vec3d(*rot)
            elif rotate_prop.GetTypeName() == Sdf.ValueTypeNames.Float3:
                new_rotate = Gf.Vec3f(*rot)
            else:
                # if rotate property is not float3 or double3, log warning and return
                carb.log_warn("unknown rotation type")
                return

        # set new rotate property
        omni.kit.commands.create(
            "ChangePropertyCommand",
            prop_path=rotate_prop.GetPath(),
            value=new_rotate,
            prev=rotate_prop.Get(),
            timecode=Usd.TimeCode.Default(),
            type_to_create_if_not_exist=(rotate_prop.GetTypeName()),
        ).do()

        # set scale property to (1, 1, 1) to prevent weird near-infinite scale (scale doesn't affect anything)
        if prim.GetAttribute("xformOp:scale"):
            omni.kit.commands.create(
                "ChangePropertyCommand",
                prop_path=prim.GetAttribute("xformOp:scale").GetPath(),
                value=Gf.Vec3d(1, 1, 1),
                prev=prim.GetAttribute("xformOp:scale").Get(),
                timecode=Usd.TimeCode.Default(),
                type_to_create_if_not_exist=(prim.GetAttribute("xformOp:scale").GetTypeName()),
            ).do()

    return


def get_viewport_names(usd_context_name: str = None) -> list[str]:
    """Get list of all viewport names.

    Args:
        usd_context_name:  usd context to use.

    Returns:
        List of viewport names
    """
    viewport_names = []
    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        for window in get_viewport_window_instances(usd_context_name):
            viewport_names.append(window.title)
        return viewport_names
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        for viewport_handle in vp_iface.get_instance_list():
            if usd_context_name and (
                usd_context_name != vp_iface.get_viewport_window(viewport_handle).get_usd_context_name()
            ):
                continue
            viewport_names.append(vp_iface.get_viewport_window_name(viewport_handle))
        return viewport_names
    except ImportError:
        pass

    return viewport_names


def get_id_from_index(index: int) -> int | None:
    """Get the viewport id for a given index.

    This function was added for backwards compatibility for VP2 as viewport IDs are not the same as the viewport index.

    Args:
        index: viewport index to retrieve ID for

    Returns:
        viewport id : Returns None if window index was not found
    """
    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        instances = []
        for window in get_viewport_window_instances(None):
            instances.append(window)

        if len(instances) > index:
            return instances[index].viewport_api.id
        else:
            return None
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        instances = vp_iface.get_instance_list()
        if len(instances) > index:
            return vp_iface.get_viewport_window(instances[index]).get_id()
        else:
            return None
    except ImportError:
        pass

    return None


def get_window_from_id(id: object, usd_context_name: str = None) -> object | None:
    """Find window that matches a given viewport id.

    Args:
        id: Viewport ID to get window for
        usd_context_name: usd context to use.

    Returns:
        Window : Returns None if window with matching ID was not found
    """
    if id is None:
        return None

    try:
        from omni.kit.viewport.window import get_viewport_window_instances

        for window in get_viewport_window_instances(usd_context_name):
            if window.viewport_api.id == id:
                return window
    except ImportError:
        pass

    try:
        import omni.kit.viewport_legacy as vp_legacy

        vp_iface = vp_legacy.get_viewport_interface()
        for viewport_handle in vp_iface.get_instance_list():
            if usd_context_name and (
                usd_context_name != vp_iface.get_viewport_window(viewport_handle).get_usd_context_name()
            ):
                continue
            if vp_iface.get_viewport_window(viewport_handle).get_id() == id:
                from omni.kit.viewport.utility.legacy_viewport_window import LegacyViewportWindow

                return LegacyViewportWindow(vp_iface.get_viewport_window_name(viewport_handle))
    except ImportError:
        pass

    return None


def destroy_all_viewports(usd_context_name: str = None, destroy_main_viewport: bool = True) -> None:
    """Destroys all viewport windows.

    Args:
        usd_context_name: usd context to use.
        destroy_main_viewport: set to False to not destroy the default viewport. Defaults to True (destroy all viewports including the default one).
    """
    from omni.kit.viewport.window import get_viewport_window_instances

    for window in get_viewport_window_instances(usd_context_name):
        if window:
            if window.title == "Viewport" and not destroy_main_viewport:
                continue
            window.destroy()


def add_aov_to_viewport(viewport_api: object, aov_name: str) -> bool:
    """Add an arbitrary output variable (AOV) to the specified viewport for rendering.

    Args:
        viewport_api: Handle to viewport API.
        aov_name: Name of the AOV to add to the viewport.

    Returns:
        True if successful.

    Raises:
        RuntimeError: If the render product is invalid or render variable cannot be created.
    """
    if hasattr(viewport_api, "legacy_window"):
        return viewport_api.legacy_window.add_aov(aov_name)

    from pxr import Usd

    stage = viewport_api.stage
    render_product_path = viewport_api.render_product_path
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        render_prod_prim = stage.GetPrimAtPath(render_product_path)
        if not render_prod_prim:
            raise RuntimeError(f'Invalid renderProduct "{render_product_path}"')
        render_var_prim_path = Sdf.Path(f"/Render/Vars/{aov_name}")
        render_var_prim = stage.GetPrimAtPath(render_var_prim_path)
        if not render_var_prim:
            render_var_prim = stage.DefinePrim(render_var_prim_path)
        if not render_var_prim:
            raise RuntimeError(f'Cannot create renderVar "{render_var_prim_path}"')
        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)
        render_prod_var_rel = render_prod_prim.GetRelationship("orderedVars")
        if not render_prod_var_rel:
            render_prod_prim.CreateRelationship("orderedVars")
        if not render_prod_var_rel:
            raise RuntimeError(f'cannot set orderedVars relationship for renderProduct "{render_product_path}"')
        render_prod_var_rel.AddTarget(render_var_prim_path)

        set_prim_hide_in_stage_window(render_var_prim, True)
        set_prim_no_delete(render_var_prim, True)

    return True


def get_intrinsics_matrix(viewport_api: Any) -> np.ndarray:
    """Get intrinsic matrix for the camera attached to a specific viewport.

    Args:
        viewport_api: Handle to viewport api

    Returns:
        the intrinsic matrix associated with the specified viewport
                The following image convention is assumed:
                    +x should point to the right in the image
                    +y should point down in the image
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(viewport_api.get_active_camera())
    focal_length = prim.GetAttribute("focalLength").Get()
    width, height = viewport_api.get_texture_resolution()
    horizontal_aperture = prim.GetAttribute("horizontalAperture").Get()
    vertical_aperture = horizontal_aperture * (float(height) / width)
    fx = width * focal_length / horizontal_aperture
    fy = height * focal_length / vertical_aperture
    cx = width * 0.5
    cy = height * 0.5
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def set_intrinsics_matrix(viewport_api: Any, intrinsics_matrix: np.ndarray, focal_length: float = 1.0) -> None:
    """Set intrinsic matrix for the camera attached to a specific viewport.

    Note:
        We assume cx and cy are centered in the camera
        horizontal_aperture_offset and vertical_aperture_offset are computed and set on the camera prim but are not used

    Args:
        viewport_api: Handle to viewport api
        intrinsics_matrix: A 3x3 intrinsic matrix
        focal_length: Default focal length to use when computing aperture values.

    Raises:
        ValueError: If intrinsic matrix is not a 3x3 matrix.
        ValueError: If camera prim is not valid
    """
    if intrinsics_matrix.shape != (3, 3):
        raise ValueError("intrinsics_matrix must be 3x3")

    fx = intrinsics_matrix[0, 0]
    fy = intrinsics_matrix[1, 1]
    cx = intrinsics_matrix[0, 2]
    cy = intrinsics_matrix[1, 2]

    stage = get_current_stage()
    camera_prim = stage.GetPrimAtPath(viewport_api.get_active_camera())
    if not camera_prim or not camera_prim.IsValid():
        raise ValueError("Viewport does not have a valid camera prim")
    prim = UsdGeom.Camera(camera_prim)

    width, height = viewport_api.get_texture_resolution()

    horizontal_aperture = width * focal_length / fx
    vertical_aperture = height * focal_length / fy
    # TODO: this should be set_attr_val
    # We have to do it this way because the camera might be on a different layer (default cameras are on session layer),
    # and this is the simplest way to set the property on the right layer.
    omni.usd.set_prop_val(prim.GetFocalLengthAttr(), focal_length)
    omni.usd.set_prop_val(prim.GetHorizontalApertureAttr(), horizontal_aperture)
    omni.usd.set_prop_val(prim.GetVerticalApertureAttr(), vertical_aperture)
    omni.usd.set_prop_val(prim.GetHorizontalApertureOffsetAttr(), (cx - width / 2) / fx)
    omni.usd.set_prop_val(prim.GetVerticalApertureOffsetAttr(), (cy - height / 2) / fy)


def backproject_depth(depth_image: np.array, viewport_api: Any, max_clip_depth: float) -> np.array:
    """Backproject depth image to image space.

    Args:
        depth_image: Depth image buffer
        viewport_api: Handle to viewport api
        max_clip_depth: Depth values larger than this will be clipped

    Returns:
        3D point cloud with shape (height * width, 3) in camera space.
    """
    intrinsics_matrix = get_intrinsics_matrix(viewport_api)
    fx = intrinsics_matrix[0][0]
    fy = intrinsics_matrix[1][1]
    cx = intrinsics_matrix[0][2]
    cy = intrinsics_matrix[1][2]
    height = depth_image.shape[0]
    width = depth_image.shape[1]
    input_x = np.arange(width)
    input_y = np.arange(height)
    input_x, input_y = np.meshgrid(input_x, input_y)
    input_x = input_x.flatten()
    input_y = input_y.flatten()
    input_z = depth_image.flatten()
    input_z[input_z > max_clip_depth] = 0
    output_x = (input_x * input_z - cx * input_z) / fx
    output_y = (input_y * input_z - cy * input_z) / fy
    raw_pc = np.stack([output_x, output_y, input_z], -1).reshape([height * width, 3])
    return raw_pc


def project_depth_to_worldspace(depth_image: np.array, viewport_api: Any, max_clip_depth: float) -> list[carb.Float3]:
    """Project depth image to world space.

    Args:
        depth_image: Depth image buffer
        viewport_api: Handle to viewport api
        max_clip_depth: Depth values larger than this will be clipped

    Returns:
        List of points from depth in world space
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(viewport_api.get_active_camera())
    prim_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode())
    units_per_meter = 1.0 / UsdGeom.GetStageMetersPerUnit(stage)

    depth_data = depth_image * units_per_meter
    depth_data = -np.clip(depth_data, 0, max_clip_depth)

    pc = backproject_depth(depth_data, viewport_api, max_clip_depth)
    points = []
    for pts in pc:
        p = prim_tf.Transform(Gf.Vec3d(-pts[0], pts[1], pts[2]))
        points.append(carb.Float3(p[0], p[1], p[2]))

    return points


def create_viewport_for_camera(
    viewport_name: str,
    camera_prim_path: str,
    width: int = 1280,
    height: int = 720,
    position_x: int = 0,
    position_y: int = 0,
) -> object:
    """Create a new viewport and peg it to a specific camera specified by camera_prim_path. If the viewport already exists with the specified viewport_name, that viewport will be replaced with the new camera view.

    Args:
        viewport_name: name of the viewport. If not provided, it will default to camera name.
        camera_prim_path: name of the prim path of the camera
        width: width of the viewport window, in pixels.
        height: height of the viewport window, in pixels.
        position_x: location x of the viewport window.
        position_y: location y of the viewport window.

    Returns:
        The created viewport window.

    Raises:
        ValueError: If the provided camera_prim_path is an invalid prim in the scene.
    """
    import omni.kit.viewport.utility as kit_viewport_utils

    if not is_prim_path_valid(camera_prim_path):
        err = f"Provided camera_prim_path '{camera_prim_path}' is an invalid prim in the scene."
        raise ValueError(err)

    # Create new viewport. If viewport already exists, this will overwrite that window's content.
    viewport_window = kit_viewport_utils.create_viewport_window(
        name=viewport_name, width=width, height=height, position_x=position_x, position_y=position_y
    )

    # Peg the viewport to the specified camera.
    omni.kit.commands.execute(
        "SetViewportCamera", camera_path=camera_prim_path, viewport_api=viewport_window.viewport_api
    )

    carb.log_info(
        f"[ui_utils/create_viewport_for_camera] Added new viewport '{ viewport_name }' for camera '{ camera_prim_path }'"
    )
    return viewport_window


def set_active_viewport_camera(camera_prim_path: str) -> None:
    """Sets the camera for the active viewport.

    This method sets the active viewport to display the camera at the specified prim path.

    Args:
        camera_prim_path: name of the prim path of the camera
    """
    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        viewport.camera_path = camera_prim_path
    except ImportError:
        carb.log_warn("omni.kit.viewport.utility needs to be enabled before using this function")
        return
