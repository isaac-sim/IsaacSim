# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List, Optional

import carb
import numpy as np
import omni
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from isaacsim.sensors.camera.camera import Camera
from pxr import Gf, Usd


class SingleViewDepthSensor(Camera):
    _render_product_prim = None

    def initialize(self, physics_sim_view=None, attach_rgb_annotator=False) -> None:
        """Initialize the depth camera.

        Calls the parent class's initialize method, then retrieves the render product prim
        and applies the appropriate schema to enable depth sensing functionality.

        Supports the following annotators:
        - DepthSensorDistance
        - DepthSensorPointCloudPosition
        - DepthSensorPointCloudColor
        - DepthSensorImager
        """
        super().initialize(physics_sim_view=physics_sim_view, attach_rgb_annotator=attach_rgb_annotator)
        # Retrieve the render product prim and apply the appropriate schema to it
        self._render_product_prim = get_prim_at_path(self._render_product_path)
        self._render_product_prim.ApplyAPI("OmniSensorDepthSensorSingleViewAPI")

        settings = carb.settings.get_settings()
        settings.set("/exts/omni.usd.schema.render_settings/rtx/renderSettings/apiSchemas/autoApply", None)
        settings.set("/exts/omni.usd.schema.render_settings/rtx/camera/apiSchemas/autoApply", None)
        settings.set("/exts/omni.usd.schema.render_settings/rtx/renderProduct/apiSchemas/autoApply", None)

        self.set_enabled(enabled=True)

    def set_baseline_mm(self, baseline_mm: float = 55) -> None:
        """Set the baseline distance in millimeters."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:baselineMM").Set(baseline_mm)

    def get_baseline_mm(self) -> float:
        """Get the baseline distance in millimeters."""
        if self._render_product_prim and self._render_product_prim.HasAttribute("omni:rtx:post:depthSensor:baselineMM"):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:baselineMM").Get()
        return None

    def set_confidence_threshold(self, confidence_threshold: float = 0.95) -> None:
        """Set the confidence threshold for the depth sensor."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:confidenceThreshold").Set(
                confidence_threshold
            )

    def get_confidence_threshold(self) -> float:
        """Get the confidence threshold for the depth sensor."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:confidenceThreshold"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:confidenceThreshold").Get()
        return None

    def set_enabled(self, enabled: bool = False) -> None:
        """Enable or disable the depth sensor."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:enabled").Set(enabled)

    def get_enabled(self) -> bool:
        """Get whether the depth sensor is enabled."""
        if self._render_product_prim and self._render_product_prim.HasAttribute("omni:rtx:post:depthSensor:enabled"):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:enabled").Get()
        return None

    def set_focal_length_pixel(self, focal_length_pixel: float = 897) -> None:
        """Set the focal length in pixels."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:focalLengthPixel").Set(focal_length_pixel)

    def get_focal_length_pixel(self) -> float:
        """Get the focal length in pixels."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:focalLengthPixel"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:focalLengthPixel").Get()
        return None

    def set_max_disparity_pixel(self, max_disparity_pixel: float = 110) -> None:
        """Set the maximum disparity in pixels."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:maxDisparityPixel").Set(
                max_disparity_pixel
            )

    def get_max_disparity_pixel(self) -> float:
        """Get the maximum disparity in pixels."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:maxDisparityPixel"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:maxDisparityPixel").Get()
        return None

    def set_max_distance(self, max_distance: float = 10000000) -> None:
        """Set the maximum distance for disparity generation."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:maxDistance").Set(max_distance)

    def get_max_distance(self) -> float:
        """Get the maximum distance for disparity generation."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:maxDistance"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:maxDistance").Get()
        return None

    def set_min_distance(self, min_distance: float = 0.5) -> None:
        """Set the minimum distance for disparity generation."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:minDistance").Set(min_distance)

    def get_min_distance(self) -> float:
        """Get the minimum distance for disparity generation."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:minDistance"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:minDistance").Get()
        return None

    def set_noise_downscale_factor_pixel(self, noise_downscale_factor_pixel: float = 1) -> None:
        """Set the noise downscale factor in pixels."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseDownscaleFactorPixel").Set(
                noise_downscale_factor_pixel
            )

    def get_noise_downscale_factor_pixel(self) -> float:
        """Get the noise downscale factor in pixels."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:noiseDownscaleFactorPixel"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseDownscaleFactorPixel").Get()
        return None

    def set_noise_mean(self, noise_mean: float = 0.25) -> None:
        """Set the noise mean."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseMean").Set(noise_mean)

    def get_noise_mean(self) -> float:
        """Get the noise mean."""
        if self._render_product_prim and self._render_product_prim.HasAttribute("omni:rtx:post:depthSensor:noiseMean"):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseMean").Get()
        return None

    def set_noise_sigma(self, noise_sigma: float = 0.25) -> None:
        """Set the noise sigma."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseSigma").Set(noise_sigma)

    def get_noise_sigma(self) -> float:
        """Get the noise sigma."""
        if self._render_product_prim and self._render_product_prim.HasAttribute("omni:rtx:post:depthSensor:noiseSigma"):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:noiseSigma").Get()
        return None

    def set_outlier_removal_enabled(self, outlier_removal_enabled: int = 3) -> None:
        """Set the outlier removal enabled attribute. Samples separated by this range (in pixels) will be removed."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:outlierRemovalEnabled").Set(
                outlier_removal_enabled
            )

    def get_outlier_removal_enabled(self) -> int:
        """Get the outlier removal enabled attribute. Samples separated by this range (in pixels) will be removed."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:outlierRemovalEnabled"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:outlierRemovalEnabled").Get()
        return None

    def set_rgb_depth_output_mode(self, rgb_depth_output_mode: int = 0) -> None:
        """Set the RGB depth output mode."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:rgbDepthOutputMode").Set(
                rgb_depth_output_mode
            )

    def get_rgb_depth_output_mode(self) -> int:
        """Get the RGB depth output mode."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:rgbDepthOutputMode"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:rgbDepthOutputMode").Get()
        return None

    def set_sensor_size_pixel(self, sensor_size_pixel: int = 1280) -> None:
        """Set the sensor size in pixels."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:sensorSizePixel").Set(sensor_size_pixel)

    def get_sensor_size_pixel(self) -> int:
        """Get the sensor size in pixels."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:sensorSizePixel"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:sensorSizePixel").Get()
        return None

    def set_show_distance(self, show_distance: bool = False) -> None:
        """Set whether to show the distance."""
        if self._render_product_prim:
            self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:showDistance").Set(show_distance)

    def get_show_distance(self) -> bool:
        """Get whether to show the distance."""
        if self._render_product_prim and self._render_product_prim.HasAttribute(
            "omni:rtx:post:depthSensor:showDistance"
        ):
            return self._render_product_prim.GetAttribute("omni:rtx:post:depthSensor:showDistance").Get()
        return None


class SingleViewDepthSensorAsset:
    """Asset-based wrapper for single view depth sensors.

    This class provides a high-level interface for working with single view depth sensor assets.
    It automatically discovers and manages depth sensor templates from USD assets and provides
    access to individual depth sensors through their camera prim paths.

    The class handles the initialization of depth sensor assets by:
    1. Loading the specified USD asset as a reference on the stage
    2. Discovering render products with depth sensor schemas
    3. Creating SingleViewDepthSensor instances for each discovered camera
    4. Managing the mapping between render products and depth sensors

    Args:
        prim_path: The USD prim path where the asset will be placed.
        asset_path: The relative path to the USD asset file within the assets root.
        position: World position of the asset as a numpy array [x, y, z]. Defaults to None.
        translation: Local translation of the asset as a numpy array [x, y, z]. Defaults to None.
        orientation: Quaternion orientation as a numpy array [w, x, y, z]. Defaults to [1.0, 0.0, 0.0, 0.0].

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.camera import SingleViewDepthSensorAsset
        >>> import numpy as np
        >>>
        >>> # Create a depth sensor asset
        >>> depth_sensor_asset = SingleViewDepthSensorAsset(
        ...     prim_path="/World/DepthSensor",
        ...     asset_path="/Isaac/Sensors/DepthSensor/depth_sensor.usd",
        ...     position=np.array([0.0, 0.0, 1.0]),
        ...     orientation=np.array([1.0, 0.0, 0.0, 0.0])
        ... )
        >>>
        >>> # Initialize all the depth sensors inthe asset
        >>> depth_sensor_asset.initialize()
        >>>
        >>> # Get a specific depth sensor by camera prim path
        >>> depth_sensor = depth_sensor_asset.get_child_depth_sensor("/World/DepthSensor/Camera")
    """

    def __init__(
        self,
        prim_path: str,
        asset_path: str,
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = np.array([1.0, 0.0, 0.0, 0.0]),
    ) -> None:

        # Map template render product paths to SingleViewDepthSensor objects created using the corresponding camera prim
        self._depth_sensor_templates = {}
        # Map camera prim paths to SingleViewDepthSensor objects for ease of lookup
        self._camera_prim_to_depth_sensor = {}

        # Add provided asset path as reference on stage
        self._prim = add_reference_to_stage(
            usd_path=asset_path,
            prim_path=prim_path,
            prim_type="Xform",
        )

        # Set translation and orientation of prim
        if position is None and translation is None:
            position = np.array([0.0, 0.0, 0.0])
        p = position if translation is None else translation
        translation = Gf.Vec3d(p[0], p[1], p[2])
        orientation = Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3])
        reset_and_set_xform_ops(self._prim.GetPrim(), translation, orientation)

        # Iterate over children of the prim, looking for render products with the appropriate schema
        for child in Usd.PrimRange(self._prim.GetPrim()):
            if (
                child.GetPrim().GetTypeName() == "RenderProduct"
                and child.HasAPI("OmniSensorDepthSensorSingleViewAPI")
                and child.HasRelationship("camera")
            ):
                targets = child.GetRelationship("camera").GetTargets()
                if len(targets) > 1:
                    carb.log_warn(
                        f"Multiple cameras found for render product {str(child.GetPath())}. Will not use any of them."
                    )
                    continue
                camera_prim_path = targets[0]
                self._depth_sensor_templates[str(child.GetPath())] = self._make_depth_sensor_from_render_product(
                    str(child.GetPath()), str(camera_prim_path)
                )
                self._camera_prim_to_depth_sensor[str(camera_prim_path)] = self._depth_sensor_templates[
                    str(child.GetPath())
                ]

    def _make_depth_sensor_from_render_product(
        self, render_product_path: str, camera_prim_path: str
    ) -> SingleViewDepthSensor:
        """Create a SingleViewDepthSensor instance from a render product.

        This method extracts the resolution from the template render product and creates
        a new SingleViewDepthSensor instance with the specified camera prim path.

        Args:
            render_product_path: The USD path to the template render product.
            camera_prim_path: The USD path to the camera prim.

        Returns:
            A configured SingleViewDepthSensor instance.
        """
        template_render_product_prim = get_prim_at_path(render_product_path)
        resolution_val = template_render_product_prim.GetAttribute("resolution").Get()
        resolution = (resolution_val[0], resolution_val[1])
        return SingleViewDepthSensor(
            prim_path=camera_prim_path,
            resolution=resolution,
        )

    def initialize(self, physics_sim_view=None, attach_rgb_annotator=False) -> None:
        """Initialize all child depth sensors in the asset.

        This method initializes each depth sensor template and copies depth sensor
        attributes from the template render products to the actual render products.
        This ensures that all depth sensors have the correct configuration.

        Each depth sensor will be enabled, even if the template render product is disabled.

        Args:
            physics_sim_view: Optional physics simulation view. Defaults to None.
            attach_rgb_annotator: Whether to attach RGB annotator. Defaults to False.

        Example:

        .. code-block:: python

            >>> depth_sensor_asset = SingleViewDepthSensorAsset(
            ...     prim_path="/World/DepthSensor",
            ...     asset_path="/Isaac/Sensors/DepthSensor/depth_sensor.usd"
            ... )
            >>>
            >>> # Initialize all depth sensors in the asset
            >>> depth_sensor_asset.initialize(attach_rgb_annotator=True)
        """
        # Initialize all child depth sensors
        for template_render_product_path, depth_sensor in self._depth_sensor_templates.items():
            depth_sensor.initialize(physics_sim_view=physics_sim_view, attach_rgb_annotator=attach_rgb_annotator)

            # Copy depth sensorattributes from template render product to actual render product
            template_render_product_prim = get_prim_at_path(template_render_product_path)
            render_product_prim = get_prim_at_path(depth_sensor.get_render_product_path())
            for attribute in template_render_product_prim.GetAttributes():
                if attribute.GetName().startswith("omni:rtx:post:depthSensor:"):
                    render_product_prim.GetAttribute(attribute.GetName()).Set(attribute.Get())

            # Explicitly enable the depth sensor, in case the template render product is disabled
            depth_sensor.set_enabled(enabled=True)

    def get_all_depth_sensors(self) -> List[SingleViewDepthSensor]:
        """Get all depth sensors in the asset."""
        return list(self._camera_prim_to_depth_sensor.values())

    def get_all_depth_sensor_paths(self) -> List[str]:
        """Get all depth sensor paths in the asset."""
        return list(self._camera_prim_to_depth_sensor.keys())

    def get_child_depth_sensor(self, camera_prim_path: str) -> SingleViewDepthSensor:
        """Get a specific depth sensor by its camera prim path.

        This method returns the SingleViewDepthSensor instance associated with the
        specified camera prim path. This allows access to individual depth sensors
        within the asset for configuration and data retrieval.

        Args:
            camera_prim_path: The USD path to the camera prim.

        Returns:
            The SingleViewDepthSensor instance for the specified camera.

        Raises:
            KeyError: If no depth sensor is found for the specified camera prim path.

        Example:

        .. code-block:: python

            >>> depth_sensor_asset = SingleViewDepthSensorAsset(
            ...     prim_path="/World/DepthSensor",
            ...     asset_path="/Isaac/Sensors/DepthSensor/depth_sensor.usd"
            ... )
            >>>
            >>> # Get a specific depth sensor
            >>> depth_sensor = depth_sensor_asset.get_child_depth_sensor("/World/DepthSensor/Camera")
            >>>
            >>> # Configure the depth sensor
            >>> depth_sensor.set_baseline_mm(60.0)
            >>> depth_sensor.set_confidence_threshold(0.95)
        """
        return self._camera_prim_to_depth_sensor[str(camera_prim_path)]

    @staticmethod
    def add_template_render_product(parent_prim_path: str, camera_prim_path: str, **kwargs) -> Usd.Prim:
        """Add a template render product for a depth sensor to the USD stage.

        This static method creates a new RenderProduct prim with the depth sensor API
        and establishes a relationship with the specified camera prim. The render product
        is created as a child of the specified parent prim and is named based on the
        camera prim's name with "_render_product" suffix.

        The method performs validation to ensure:
        - The parent prim path is valid
        - The camera prim exists and is of type "Camera"
        - The render product can be successfully created

        Args:
            parent_prim_path: The USD path to the parent prim where the render product will be created.
                If the path ends with "/", it will be automatically removed.
            camera_prim_path: The USD path to the camera prim that will be associated with the render product.
            **kwargs: Additional keyword arguments to pass to the RenderProduct prim as attributes.

        Raises:
            RuntimeError: If the USD stage cannot be accessed.
            ValueError: If the camera prim path is invalid or the prim is not a Camera.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.camera import SingleViewDepthSensorAsset
            >>>
            >>> # Add a template render product for a depth sensor
            >>> SingleViewDepthSensorAsset.add_template_render_product(
            ...     parent_prim_path="/World/DepthSensor",
            ...     camera_prim_path="/World/DepthSensor/Camera"
            ... )
            >>>
            >>> # This creates a render product at "/World/DepthSensor/Camera_render_product"
            >>> # with the OmniSensorDepthSensorSingleViewAPI applied and a relationship
            >>> # to the camera at "/World/DepthSensor/Camera"
        """
        stage = omni.usd.get_context().get_stage()
        if parent_prim_path.endswith("/"):
            parent_prim_path = parent_prim_path[:-1]
        render_product_prim_path = parent_prim_path + "/" + camera_prim_path.split("/")[-1] + "_render_product"
        camera_prim = stage.GetPrimAtPath(camera_prim_path)
        if not camera_prim.IsValid():
            carb.log_warn(f"Failed to get valid prim at path {camera_prim_path}.")
        elif camera_prim.GetPrim().GetTypeName() != "Camera":
            carb.log_warn(f"Prim at path {camera_prim_path} is not a Camera.")
        else:
            render_product_prim = stage.DefinePrim(render_product_prim_path, "RenderProduct")
            if render_product_prim.IsValid():
                render_product_prim.ApplyAPI("OmniSensorDepthSensorSingleViewAPI")
                render_product_prim.CreateRelationship("camera").SetTargets([camera_prim_path])
            else:
                carb.log_warn(f"Failed to create RenderProduct at path {render_product_prim_path}.")
            for key, value in kwargs.items():
                if render_product_prim.HasAttribute(key):
                    render_product_prim.GetAttribute(key).Set(value)
                else:
                    carb.log_warn(f"RenderProduct at path {render_product_prim_path} does not have attribute {key}.")
            return render_product_prim
        return Usd.Prim()
