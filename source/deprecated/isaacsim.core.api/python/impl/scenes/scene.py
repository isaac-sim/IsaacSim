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

"""Scene management system for Isaac Sim environments that provides methods to add, manage, and interact with objects in the USD stage."""

from __future__ import annotations

import builtins
import gc

import carb
import numpy as np
import omni.usd.commands
from isaacsim.core.api.materials.deformable_material import DeformableMaterial
from isaacsim.core.api.materials.deformable_material_view import DeformableMaterialView
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.api.materials.particle_material_view import ParticleMaterialView
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.robots.robot_view import RobotView
from isaacsim.core.api.scenes.scene_registry import SceneRegistry
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
from isaacsim.core.prims import (
    Articulation,
    ClothPrim,
    DeformablePrim,
    GeometryPrim,
    ParticleSystem,
    RigidPrim,
    SingleArticulation,
    SingleClothPrim,
    SingleDeformablePrim,
    SingleGeometryPrim,
    SingleParticleSystem,
    SingleRigidPrim,
    SingleXFormPrim,
    XFormPrim,
)
from isaacsim.core.utils.prims import (
    get_prim_children,
    get_prim_parent,
    get_prim_path,
    is_prim_ancestral,
    is_prim_path_valid,
    is_prim_root_path,
)
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage, update_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf, Usd, UsdGeom


class Scene(object):
    """Provide methods to add objects of interest in the stage to retrieve their information and set their.

    reset default state in an easy way.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.scenes import Scene
        >>>
        >>> scene = Scene()
        >>> scene
        <isaacsim.core.api.scenes.scene.Scene object at 0x...>

    """

    def __init__(self) -> None:
        self._scene_registry = SceneRegistry()
        self._enable_bounding_box_computations = False
        self._bbox_cache = None
        return

    def __del__(self) -> None:
        """Clean up the scene registry and trigger garbage collection."""
        self.clear(registry_only=True)
        gc.collect()
        return

    @property
    def stage(self) -> Usd.Stage:
        """Current USD stage.

        Returns:
            Current USD stage.

        Example:

        .. code-block:: python

            >>> scene.stage
            Usd.Stage.Open(rootLayer=Sdf.Find('anon:0x...usd'),
                           sessionLayer=Sdf.Find('anon:0x...usda'),
                           pathResolverContext=<invalid repr>)

        """
        return get_current_stage()

    def add(self, obj: SingleXFormPrim) -> SingleXFormPrim:
        """Add an object to the scene registry.

        Args:
            obj: Object to be added.

        Raises:
            Exception: The object type is not supported yet.

        Returns:
            Object.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.prims import XFormPrim
            >>>
            >>> prims = XFormPrim(prim_paths_expr="/World")
            >>> scene.add(prims)
            <isaacsim.core.prims.XFormPrim object at 0x...>

        """
        if self._scene_registry.name_exists(obj.name):
            raise Exception(f"Cannot add the object {obj.name} to the scene since its name is not unique")
        if isinstance(obj, SingleRigidPrim):
            self._scene_registry.add_rigid_object(name=obj.name, rigid_object=obj)
        elif isinstance(obj, RigidPrim):
            self._scene_registry.add_rigid_prim_view(name=obj.name, rigid_prim_view=obj)
        elif isinstance(obj, RigidContactView):
            self._scene_registry.add_rigid_contact_view(name=obj.name, rigid_contact_view=obj)
        elif isinstance(obj, SingleGeometryPrim):
            self._scene_registry.add_geometry_object(name=obj.name, geometry_object=obj)
        elif isinstance(obj, GroundPlane):
            self._scene_registry.add_geometry_object(name=obj.name, geometry_object=obj)
        elif isinstance(obj, GeometryPrim):
            self._scene_registry.add_geometry_prim_view(name=obj.name, geometry_prim_view=obj)
        elif isinstance(obj, Robot):
            self._scene_registry.add_robot(name=obj.name, robot=obj)
        elif isinstance(obj, RobotView):
            self._scene_registry.add_robot_view(name=obj.name, robot_view=obj)
        elif isinstance(obj, SingleArticulation):
            self._scene_registry.add_articulated_system(name=obj.name, articulated_system=obj)
        elif isinstance(obj, Articulation):
            self._scene_registry.add_articulated_view(name=obj.name, articulated_view=obj)
        elif isinstance(obj, BaseSensor):
            self._scene_registry.add_sensor(name=obj.name, sensor=obj)
        elif isinstance(obj, SingleClothPrim):
            self._scene_registry.add_cloth(name=obj.name, cloth=obj)
        elif isinstance(obj, ClothPrim):
            self._scene_registry.add_cloth_view(name=obj.name, cloth_prim_view=obj)
        elif isinstance(obj, SingleParticleSystem):
            self._scene_registry.add_particle_system(name=obj.name, particle_system=obj)
        elif isinstance(obj, ParticleSystem):
            self._scene_registry.add_particle_system_view(name=obj.name, particle_system_view=obj)
        elif isinstance(obj, ParticleMaterial):
            self._scene_registry.add_particle_material(name=obj.name, particle_material=obj)
        elif isinstance(obj, ParticleMaterialView):
            self._scene_registry.add_particle_material_view(name=obj.name, particle_material_view=obj)
        elif isinstance(obj, SingleDeformablePrim):
            self._scene_registry.add_deformable(name=obj.name, deformable=obj)
        elif isinstance(obj, DeformablePrim):
            self._scene_registry.add_deformable_view(name=obj.name, deformable_prim_view=obj)
        elif isinstance(obj, DeformableMaterial):
            self._scene_registry.add_deformable_material(name=obj.name, deformable_material=obj)
        elif isinstance(obj, DeformableMaterialView):
            self._scene_registry.add_deformable_material_view(name=obj.name, deformable_material_view=obj)
        elif isinstance(obj, SingleXFormPrim):
            self._scene_registry.add_xform(name=obj.name, xform=obj)
        elif isinstance(obj, XFormPrim):
            self._scene_registry.add_xform_view(name=obj.name, xform_prim_view=obj)
        else:
            raise TypeError(f"object type {type(obj).__name__} is not supported")
        return obj

    def add_ground_plane(
        self,
        size: float | None = None,
        z_position: float = 0,
        name: str = "ground_plane",
        prim_path: str = "/World/groundPlane",
        static_friction: float = 0.5,
        dynamic_friction: float = 0.5,
        restitution: float = 0.8,
        color: np.ndarray | None = None,
    ) -> GroundPlane:
        """Create a ground plane and add it to the scene registry.

        Args:
            size: Length of each edge.
            z_position: Ground plane position in the z-axis.
            name: Shortname to be used as a key by Scene class.
                Note: needs to be unique if the object is added to the Scene.
            prim_path: Prim path of the prim to create.
            static_friction: Static friction coefficient.
            dynamic_friction: Dynamic friction coefficient.
            restitution: Restitution coefficient.
            color: Color of the visual plane. None means 50% gray.

        Returns:
            Ground plane instance.

        Example:

        .. code-block:: python

            >>> scene.add_ground_plane()
            <isaacsim.core.api.objects.ground_plane.GroundPlane object at 0x...>

        """
        if Scene.object_exists(self, name=name):
            carb.log_info(f"ground floor already created with name {name}.")
            return Scene.get_object(self, name=name)
        physics_material_path = find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            restitution=restitution,
        )
        plane = GroundPlane(
            prim_path=prim_path,
            name=name,
            z_position=z_position,
            size=size,
            color=color,
            physics_material=physics_material,
        )
        Scene.add(self, plane)
        return plane

    def add_default_ground_plane(
        self,
        z_position: float = 0,
        name: str = "default_ground_plane",
        prim_path: str = "/World/defaultGroundPlane",
        static_friction: float = 0.5,
        dynamic_friction: float = 0.5,
        restitution: float = 0.8,
    ) -> GroundPlane:
        """Create a ground plane (using the default asset for Isaac Sim environments) and add it to the scene registry.

        Args:
            z_position: Ground plane position in the z-axis.
            name: Shortname to be used as a key by Scene class.
                Note: needs to be unique if the object is added to the Scene.
            prim_path: Prim path of the prim to create.
            static_friction: Static friction coefficient.
            dynamic_friction: Dynamic friction coefficient.
            restitution: Restitution coefficient.

        Returns:
            Ground plane instance.

        Example:

        .. code-block:: python

            >>> scene.add_default_ground_plane()
            server...
            <isaacsim.core.api.objects.ground_plane.GroundPlane object at 0x...>

        """
        if Scene.object_exists(self, name=name):
            carb.log_info(f"ground floor already created with name {name}.")
            return Scene.get_object(self, name=name)
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return None
        usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        physics_material_path = find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=static_friction,
            dynamic_friction=dynamic_friction,
            restitution=restitution,
        )
        plane = GroundPlane(prim_path=prim_path, name=name, z_position=z_position, physics_material=physics_material)
        Scene.add(self, plane)
        return plane

    def post_reset(self) -> None:
        """Call the ``post_reset`` method on all added objects to the scene registry.

        Example:

        .. code-block:: python

            >>> scene.post_reset()

        """
        prim_registries_available = [
            self._scene_registry._geometry_objects,
            self._scene_registry._rigid_objects,
            self._scene_registry.rigid_contact_views,
            self._scene_registry.geometry_prim_views,
            self._scene_registry._articulated_systems,
            self._scene_registry._articulated_views,
            self._scene_registry._robots,
            self._scene_registry._sensors,
            self._scene_registry.xforms,
            self._scene_registry._xform_prim_views,
            self._scene_registry._robot_views,
            self._scene_registry.rigid_prim_views,
            self._scene_registry._cloth_prims,
            self._scene_registry._particle_systems,
            self._scene_registry._particle_materials,
            self._scene_registry._cloth_prim_views,
            self._scene_registry._particle_system_views,
            self._scene_registry._particle_material_views,
            self._scene_registry._deformable_prims,
            self._scene_registry._deformable_prim_views,
            self._scene_registry._deformable_materials,
            self._scene_registry._deformable_material_views,
        ]

        for prim_registery in prim_registries_available:
            for prim_name in list(prim_registery):
                if not prim_registery[prim_name].is_valid():
                    prim_object = prim_registery[prim_name]
                    self._scene_registry.remove_object(name=prim_name)
                    del prim_object
                else:
                    prim_registery[prim_name].post_reset()
        if self._enable_bounding_box_computations:
            self._bbox_cache.Clear()
        gc.collect()
        return

    def _finalize(self, physics_sim_view: object) -> None:
        """Initialize all registered objects with the physics simulation view.

        Args:
            physics_sim_view: Physics simulation view to initialize objects with.

        """
        for xform_name, xform_object in self._scene_registry.xforms.items():
            xform_object.initialize(physics_sim_view)
        for deformable_name, deformable_object in self._scene_registry.deformable_prims.items():
            deformable_object.initialize(physics_sim_view)
        for deformable_name, deformable_object in self._scene_registry.deformable_prim_views.items():
            deformable_object.initialize(physics_sim_view)
        for deformable_material_name, deformable_material_object in self._scene_registry.deformable_materials.items():
            deformable_material_object.initialize(physics_sim_view)
        for (
            deformable_material_name,
            deformable_material_object,
        ) in self._scene_registry.deformable_material_views.items():
            deformable_material_object.initialize(physics_sim_view)
        for cloth_name, cloth_object in self._scene_registry.cloth_prims.items():
            cloth_object.initialize(physics_sim_view)
        for cloth_name, cloth_object in self._scene_registry.cloth_prim_views.items():
            cloth_object.initialize(physics_sim_view)
        for particle_system_name, particle_system_object in self._scene_registry.particle_systems.items():
            particle_system_object.initialize(physics_sim_view)
        for particle_system_name, particle_system_object in self._scene_registry.particle_system_views.items():
            particle_system_object.initialize(physics_sim_view)
        for particle_material_name, particle_material_object in self._scene_registry.particle_materials.items():
            particle_material_object.initialize(physics_sim_view)
        for particle_material_name, particle_material_object in self._scene_registry.particle_material_views.items():
            particle_material_object.initialize(physics_sim_view)
        for sensor_name, sensor_object in self._scene_registry.sensors.items():
            sensor_object.initialize(physics_sim_view)
        for geometry_prim_name, geometry_object in self._scene_registry._geometry_objects.items():
            geometry_object.initialize(physics_sim_view)
        for geometry_prim_name, geometry_view in self._scene_registry.geometry_prim_views.items():
            geometry_view.initialize(physics_sim_view)
        for articulation_name, articulated_system in self._scene_registry.articulated_systems.items():
            articulated_system.initialize(physics_sim_view)
        for robot_name, robot in self._scene_registry.robots.items():
            robot.initialize(physics_sim_view)
        for robots_name, robot_view in self._scene_registry.robot_views.items():
            robot_view.initialize(physics_sim_view)
        for rigid_object_name, rigid_object in self._scene_registry.rigid_objects.items():
            rigid_object.initialize(physics_sim_view)
        for rigid_contact_view_name, rigid_contact_view in self._scene_registry.rigid_contact_views.items():
            rigid_contact_view.initialize(physics_sim_view)
        return

    def remove_object(self, name: str, registry_only: bool = False) -> None:
        """Remove and object from the scene registry and the USD stage if specified (enable by default).

        Args:
            name: Name of the prim to be removed.
            registry_only: True to remove the object from the scene registry only and not the USD.

        Example:

        .. code-block:: python

            >>> # given a default ground plane named 'default_ground_plane'
            >>> scene.remove_object("default_ground_plane")

        """
        prim_object = self.get_object(name=name)
        # sometimes the prim path is under a reference
        if not registry_only:
            if hasattr(prim_object, "prim"):
                prims = [prim_object.prim]
            elif hasattr(prim_object, "prims"):
                prims = prim_object.prims
            else:
                carb.log_error(f"No attribute prim present under the key: {name}")
                return
            # iterate over prims under prim-object
            for current_prim in prims:
                prim_path = get_prim_path(current_prim)
                while not is_prim_root_path(prim_path):
                    if not is_prim_ancestral(prim_path):
                        break
                    else:  # its under a reference
                        parent_prim = get_prim_parent(current_prim)
                        children_number = len(get_prim_children(parent_prim))
                        if children_number > 1:
                            break
                        current_prim = parent_prim
                        prim_path = get_prim_path(current_prim)
                if is_prim_path_valid(prim_path) and not is_prim_ancestral(prim_path):
                    omni.usd.commands.DeletePrimsCommand([get_prim_path(current_prim)]).do()
            # update the stage
            if not getattr(builtins, "ISAAC_LAUNCHED_FROM_TERMINAL", True):
                update_stage()
        self._scene_registry.remove_object(name=name)
        del prim_object
        return

    def get_object(self, name: str) -> SingleXFormPrim:
        """Get a registered object by its name if exists otherwise None.

        .. note::

            Object can be registered via the ``add`` method

        Args:
            name: Object name.

        Returns:
            Object if it exists otherwise None.

        Example:

        .. code-block:: python

            >>> # given a default ground plane named 'default_ground_plane'
            >>> scene.get_object("default_ground_plane")
            <isaacsim.core.api.objects.ground_plane.GroundPlane object at 0x...>

        """
        return self._scene_registry.get_object(name=name)

    def object_exists(self, name: str) -> bool:
        """Check if an object exists in the scene registry.

        Args:
            name: Object name.

        Returns:
            Whether the object exists in the scene registry or not.

        Example:

        .. code-block:: python

            >>> # given a default ground plane named 'default_ground_plane'
            >>> scene.object_exists("default_ground_plane")
            True

        """
        if self._scene_registry.name_exists(name):
            return True
        else:
            return False

    def clear(self, registry_only: bool = False) -> None:
        """Clear the stage from all added objects to the scene registry.

        Args:
            registry_only: True to remove the object from the scene registry only and not the USD.

        Example:

        .. code-block:: python

            >>> scene.clear()

        """
        # Group all of the stage delete events together
        with Sdf.ChangeBlock():
            for geometry_object_name in list(self._scene_registry._geometry_objects):
                self.remove_object(geometry_object_name, registry_only=registry_only)
            for geometry_prim_view_name in list(self._scene_registry.geometry_prim_views):
                self.remove_object(geometry_prim_view_name, registry_only=registry_only)
            for rigid_object_name in list(self._scene_registry._rigid_objects):
                self.remove_object(rigid_object_name, registry_only=registry_only)
            for rigid_prim_view_name in list(self._scene_registry.rigid_prim_views):
                self.remove_object(rigid_prim_view_name, registry_only=registry_only)
            for rigid_contact_view_name in list(self._scene_registry._rigid_contact_views):
                self.remove_object(rigid_contact_view_name, registry_only=registry_only)
            for articulated_system_name in list(self._scene_registry._articulated_systems):
                self.remove_object(articulated_system_name, registry_only=registry_only)
            for articulated_view in list(self._scene_registry._articulated_views):
                self.remove_object(articulated_view, registry_only=registry_only)
            for robot_name in list(self._scene_registry._robots):
                self.remove_object(robot_name, registry_only=registry_only)
            for sensor_name in list(self._scene_registry.sensors):
                self.remove_object(sensor_name, registry_only=registry_only)
            for xform_name in list(self._scene_registry.xforms):
                self.remove_object(xform_name, registry_only=registry_only)
            for robot_name in list(self._scene_registry._robot_views):
                self.remove_object(robot_name, registry_only=registry_only)
            for xform_name in list(self._scene_registry._xform_prim_views):
                self.remove_object(xform_name, registry_only=registry_only)

            for cloth_name in list(self._scene_registry._cloth_prims):
                self.remove_object(cloth_name, registry_only=registry_only)
            for particle_system_name in list(self._scene_registry._particle_systems):
                self.remove_object(particle_system_name, registry_only=registry_only)
            for particle_material_name in list(self._scene_registry._particle_materials):
                self.remove_object(particle_material_name, registry_only=registry_only)
            for cloth_name in list(self._scene_registry._cloth_prim_views):
                self.remove_object(cloth_name, registry_only=registry_only)
            for particle_system_name in list(self._scene_registry._particle_system_views):
                self.remove_object(particle_system_name, registry_only=registry_only)
            for particle_material_name in list(self._scene_registry._particle_material_views):
                self.remove_object(particle_material_name, registry_only=registry_only)

            for deformable_name in list(self._scene_registry._deformable_prims):
                self.remove_object(deformable_name, registry_only=registry_only)
            for deformable_name in list(self._scene_registry._deformable_prim_views):
                self.remove_object(deformable_name, registry_only=registry_only)
            for deformable_material_name in list(self._scene_registry._deformable_materials):
                self.remove_object(deformable_material_name, registry_only=registry_only)
            for deformable_material_name in list(self._scene_registry._deformable_material_views):
                self.remove_object(deformable_material_name, registry_only=registry_only)
        return

    def compute_object_AABB(self, name: str) -> tuple[np.ndarray, np.ndarray]:  # noqa: N802
        """Compute the bounding box points (minimum and maximum) of a registered object given its name.

        .. warning::

            The bounding box computations should be enabled, via the ``enable_bounding_boxes_computations`` method,
            before querying the Axis-Aligned Bounding Box (AABB) of an object

        Args:
            name: object name

        Raises:
            Exception: If the bounding box computation is not enabled

        Returns:
            bounding box points (minimum and maximum)

        Example:

        .. code-block:: python

            >>> scene.enable_bounding_boxes_computations()
            >>>
            >>> bbox = scene.compute_object_AABB("ground_plane")
            >>> bbox[0]  # minimum
            array([-50., -50.,  0.])
            >>> bbox[1]  # maximum
            array([50., 50.,  0.])

        """
        if not self._enable_bounding_box_computations:
            raise Exception("bounding box computations should be enabled before querying AABB of an object")
        prim_object = self.get_object(name)
        if not hasattr(prim_object, "prim"):
            raise ValueError(f"Computing AABB bounds supported only for single classes, but '{name}' is a view class.")
        bounds = self._bbox_cache.ComputeWorldBound(prim_object.prim)
        prim_range = bounds.ComputeAlignedRange()
        return np.array([np.array(prim_range.GetMin()), np.array(prim_range.GetMax())])

    def enable_bounding_boxes_computations(self) -> None:
        """Enable the bounding boxes computations.

        Example:

        .. code-block:: python

            >>> scene.enable_bounding_boxes_computations()

        """
        self._bbox_cache = UsdGeom.BBoxCache(
            time=Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=False
        )
        self._enable_bounding_box_computations = True
        return

    def disable_bounding_boxes_computations(self) -> None:
        """Disable the bounding boxes computations.

        Example:

        .. code-block:: python

            >>> scene.disable_bounding_boxes_computations()

        """
        self._bbox_cache = None
        self._enable_bounding_box_computations = False
        return
