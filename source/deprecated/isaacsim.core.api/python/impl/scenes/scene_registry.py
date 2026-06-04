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

"""Provides a registry system for tracking and managing different types of objects added to Isaac Sim scenes."""

from isaacsim.core.api.materials.deformable_material import DeformableMaterial
from isaacsim.core.api.materials.deformable_material_view import DeformableMaterialView
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.api.materials.particle_material_view import ParticleMaterialView
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.robots.robot_view import RobotView
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


class SceneRegistry(object):
    """Class to keep track of the different types of objects added to the scene.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.api.scenes import SceneRegistry
        >>>
        >>> scene_registry = SceneRegistry()
        >>> scene_registry
        <isaacsim.core.api.scenes.scene_registry.SceneRegistry object at 0x...>

    """

    def __init__(self) -> None:
        self._rigid_objects = {}
        self._geometry_objects = {}
        self._articulated_systems = {}
        self._robots = {}
        self._xforms = {}
        self._sensors = {}
        self._xform_prim_views = {}
        self._deformable_prims = {}
        self._deformable_prim_views = {}
        self._deformable_materials = {}
        self._deformable_material_views = {}
        self._cloth_prims = {}
        self._cloth_prim_views = {}
        self._particle_systems = {}
        self._particle_system_views = {}
        self._particle_materials = {}
        self._particle_material_views = {}
        self._geometry_prim_views = {}
        self._rigid_prim_views = {}
        self._rigid_contact_views = {}
        self._articulated_views = {}
        self._robot_views = {}

        self._all_object_dicts = [
            self._rigid_objects,
            self._geometry_objects,
            self._articulated_systems,
            self._robots,
            self._xforms,
            self._sensors,
            self._xform_prim_views,
            self._deformable_prims,
            self._deformable_prim_views,
            self._deformable_materials,
            self._deformable_material_views,
            self._cloth_prims,
            self._cloth_prim_views,
            self._particle_systems,
            self._particle_system_views,
            self._particle_materials,
            self._particle_material_views,
            self._geometry_prim_views,
            self._rigid_prim_views,
            self._rigid_contact_views,
            self._articulated_views,
            self._robot_views,
        ]
        return

    @property
    def articulated_systems(self) -> dict:
        """Registered ``SingleArticulation`` objects.

        Returns:
            Dictionary containing the registered articulated systems.

        """
        return self._articulated_systems

    @property
    def rigid_objects(self) -> dict:
        """Registered ``SingleRigidPrim`` objects.

        Returns:
            Dictionary containing the registered rigid objects.

        """
        return self._rigid_objects

    @property
    def rigid_prim_views(self) -> dict:
        """Registered ``RigidPrim`` objects.

        Returns:
            Dictionary containing the registered rigid prim views.

        """
        return self._rigid_prim_views

    @property
    def rigid_contact_views(self) -> dict:
        """Registered ``RigidContactView`` objects.

        Returns:
            Dictionary containing the registered rigid contact views.

        """
        return self._rigid_contact_views

    @property
    def geometry_prim_views(self) -> dict:
        """Registered ``GeometryPrim`` objects.

        Returns:
            Dictionary containing the registered geometry prim views.

        """
        return self._geometry_prim_views

    @property
    def articulated_views(self) -> dict:
        """Registered ``Articulation`` objects.

        Returns:
            Dictionary containing the registered articulated views.

        """
        return self._articulated_views

    @property
    def robot_views(self) -> dict:
        """Registered ``RobotView`` objects.

        Returns:
            Dictionary containing the registered robot views.

        """
        return self._robot_views

    @property
    def robots(self) -> dict:
        """Registered ``Robot`` objects.

        Returns:
            Dictionary containing the registered robots.

        """
        return self._robots

    @property
    def xforms(self) -> dict:
        """Registered ``SingleXFormPrim`` objects.

        Returns:
            Dictionary containing the registered xforms.

        """
        return self._xforms

    @property
    def sensors(self) -> dict:
        """Registered ``BaseSensor`` (and derived) objects.

        Returns:
            Dictionary containing the registered sensors.

        """
        return self._sensors

    @property
    def xform_prim_views(self) -> dict:
        """Registered ``XFormPrim`` objects.

        Returns:
            Dictionary of registered ``XFormPrim`` objects.

        """
        return self._xform_prim_views

    @property
    def deformable_prims(self) -> dict:
        """Registered ``SingleDeformablePrim`` objects.

        Returns:
            Dictionary of registered ``SingleDeformablePrim`` objects.

        """
        return self._deformable_prims

    @property
    def deformable_prim_views(self) -> dict:
        """Registered ``DeformablePrim`` objects.

        Returns:
            Dictionary of registered ``DeformablePrim`` objects.

        """
        return self._deformable_prim_views

    @property
    def deformable_materials(self) -> dict:
        """Registered ``DeformableMaterial`` objects.

        Returns:
            Dictionary of registered ``DeformableMaterial`` objects.

        """
        return self._deformable_materials

    @property
    def deformable_material_views(self) -> dict:
        """Registered ``DeformableMaterialView`` objects.

        Returns:
            Dictionary of registered ``DeformableMaterialView`` objects.

        """
        return self._deformable_material_views

    @property
    def cloth_prims(self) -> dict:
        """Registered ``SingleClothPrim`` objects.

        Returns:
            Dictionary of registered ``SingleClothPrim`` objects.

        """
        return self._cloth_prims

    @property
    def cloth_prim_views(self) -> dict:
        """Registered ``ClothPrim`` objects.

        Returns:
            Dictionary of registered ``ClothPrim`` objects.

        """
        return self._cloth_prim_views

    @property
    def particle_systems(self) -> dict:
        """Registered ``SingleParticleSystem`` objects.

        Returns:
            Dictionary of registered ``SingleParticleSystem`` objects.

        """
        return self._particle_systems

    @property
    def particle_system_views(self) -> dict:
        """Registered ``ParticleSystem`` objects.

        Returns:
            Dictionary of registered ``ParticleSystem`` objects.

        """
        return self._particle_system_views

    @property
    def particle_materials(self) -> dict:
        """Registered ``ParticleMaterial`` objects.

        Returns:
            Dictionary of registered ``ParticleMaterial`` objects.

        """
        return self._particle_materials

    @property
    def particle_material_views(self) -> dict:
        """Registered ``ParticleMaterialView`` objects.

        Returns:
            Dictionary mapping names to registered particle material view objects.

        """
        return self._particle_material_views

    def _register_object(self, name: str, obj: object, object_dict: dict) -> None:
        """Register an object in the registry.

        Args:
            name: Object name.
            obj: Object.
            object_dict: Dictionary to register the object in.

        Raises:
            ValueError: If the object name is not unique.
        """
        if self.name_exists(name):
            raise ValueError(f"Cannot add the object {name} to the scene since its name is not unique")
        object_dict[name] = obj
        return

    def add_rigid_object(self, name: str, rigid_object: SingleRigidPrim) -> None:
        """Register a ``SingleRigidPrim`` (or subclass) object.

        Args:
            name: Object name.
            rigid_object: Object.

        """
        self._register_object(name, rigid_object, self._rigid_objects)
        return

    def add_rigid_prim_view(self, name: str, rigid_prim_view: RigidPrim) -> None:
        """Register a ``RigidPrim`` (or subclass) object.

        Args:
            name: Object name.
            rigid_prim_view: Object.

        """
        self._register_object(name, rigid_prim_view, self._rigid_prim_views)
        return

    def add_rigid_contact_view(self, name: str, rigid_contact_view: RigidContactView) -> None:
        """Register a ``RigidContactView`` (or subclass) object.

        Args:
            name: Object name.
            rigid_contact_view: Object.

        """
        self._register_object(name, rigid_contact_view, self._rigid_contact_views)
        return

    def add_articulated_system(self, name: str, articulated_system: SingleArticulation) -> None:
        """Register a ``SingleArticulation`` (or subclass) object.

        Args:
            name: Object name.
            articulated_system: Object.

        """
        self._register_object(name, articulated_system, self._articulated_systems)
        return

    def add_articulated_view(self, name: str, articulated_view: Articulation) -> None:
        """Register a ``Articulation`` (or subclass) object.

        Args:
            name: Object name.
            articulated_view: Object.

        """
        self._register_object(name, articulated_view, self._articulated_views)
        return

    def add_geometry_object(self, name: str, geometry_object: SingleGeometryPrim) -> None:
        """Register a ``SingleGeometryPrim`` (or subclass) object.

        Args:
            name: Object name.
            geometry_object: Object.

        """
        self._register_object(name, geometry_object, self._geometry_objects)
        return

    def add_geometry_prim_view(self, name: str, geometry_prim_view: GeometryPrim) -> None:
        """Register a ``GeometryPrim`` (or subclass) object.

        Args:
            name: Object name.
            geometry_prim_view: Object.

        """
        self._register_object(name, geometry_prim_view, self._geometry_prim_views)
        return

    def add_robot(self, name: str, robot: Robot) -> None:
        """Register a ``Robot`` (or subclass) object.

        Args:
            name: Object name.
            robot: Object.

        """
        self._register_object(name, robot, self._robots)
        return

    def add_robot_view(self, name: str, robot_view: RobotView) -> None:
        """Register a ``RobotView`` (or subclass) object.

        Args:
            name: Object name.
            robot_view: Object.

        """
        self._register_object(name, robot_view, self._robot_views)
        return

    def add_xform_view(self, name: str, xform_prim_view: XFormPrim) -> None:
        """Register a ``XFormPrim`` (or subclass) object.

        Args:
            name: Object name
            xform_prim_view: Object

        """
        self._register_object(name, xform_prim_view, self._xform_prim_views)
        return

    def add_deformable(self, name: str, deformable: SingleDeformablePrim) -> None:
        """Register a ``SingleDeformablePrim`` (or subclass) object.

        Args:
            name: Object name
            deformable: Object

        """
        self._register_object(name, deformable, self._deformable_prims)
        return

    def add_deformable_view(self, name: str, deformable_prim_view: DeformablePrim) -> None:
        """Register a ``DeformablePrim`` (or subclass) object.

        Args:
            name: Object name
            deformable_prim_view: Object

        """
        self._register_object(name, deformable_prim_view, self._deformable_prim_views)
        return

    def add_deformable_material(self, name: str, deformable_material: DeformableMaterial) -> None:
        """Register a ``DeformableMaterial`` (or subclass) object.

        Args:
            name: Object name
            deformable_material: Object

        """
        self._register_object(name, deformable_material, self._deformable_materials)
        return

    def add_deformable_material_view(self, name: str, deformable_material_view: DeformableMaterialView) -> None:
        """Register a ``DeformableMaterialView`` (or subclass) object.

        Args:
            name: Object name
            deformable_material_view: Object

        """
        self._register_object(name, deformable_material_view, self._deformable_material_views)
        return

    def add_cloth(self, name: str, cloth: SingleClothPrim) -> None:
        """Register a ``SingleClothPrim`` (or subclass) object.

        Args:
            name: Object name
            cloth: Object

        """
        self._register_object(name, cloth, self._cloth_prims)
        return

    def add_cloth_view(self, name: str, cloth_prim_view: ClothPrim) -> None:
        """Register a ``ClothPrim`` (or subclass) object.

        Args:
            name: Object name
            cloth_prim_view: Object

        """
        self._register_object(name, cloth_prim_view, self._cloth_prim_views)
        return

    def add_particle_system(self, name: str, particle_system: SingleParticleSystem) -> None:
        """Register a ``SingleParticleSystem`` (or subclass) object.

        Args:
            name: Object name
            particle_system: Object

        """
        self._register_object(name, particle_system, self._particle_systems)
        return

    def add_particle_system_view(self, name: str, particle_system_view: ParticleSystem) -> None:
        """Register a ``ParticleSystem`` (or subclass) object.

        Args:
            name: Object name
            particle_system_view: Object

        """
        self._register_object(name, particle_system_view, self._particle_system_views)
        return

    def add_particle_material(self, name: str, particle_material: ParticleMaterial) -> None:
        """Register a ``ParticleMaterial`` (or subclass) object.

        Args:
            name: Object name
            particle_material: Object

        """
        self._register_object(name, particle_material, self._particle_materials)
        return

    def add_particle_material_view(self, name: str, particle_material_view: ParticleMaterialView) -> None:
        """Register a ``ParticleMaterialView`` (or subclass) object.

        Args:
            name: Object name
            particle_material_view: Object

        """
        self._register_object(name, particle_material_view, self._particle_material_views)
        return

    def add_xform(self, name: str, xform: SingleXFormPrim) -> None:
        """Register a ``SingleXFormPrim`` (or subclass) object.

        Args:
            name: Object name
            xform: Object

        """
        self._register_object(name, xform, self._xforms)
        return

    def add_sensor(self, name: str, sensor: BaseSensor) -> None:
        """Register a ``BaseSensor`` (or subclass) object.

        Args:
            name: Object name
            sensor: Object

        """
        self._register_object(name, sensor, self._sensors)

        return

    def name_exists(self, name: str) -> bool:
        """Check if an object exists in the registry by its name.

        Args:
            name: Object name

        Returns:
            Whether the object is registered or not.

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.name_exists("default_ground_plane")
            True

        """
        return any(name in object_dict for object_dict in self._all_object_dicts)

    def remove_object(self, name: str) -> None:
        """Remove and object from the registry.

        .. note::

            This method will only remove the object from the internal registry.
            The wrapped object will not be removed from the USD stage

        Args:
            name: Object name

        Raises:
            Exception: If the name doesn't exist in the registry

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.remove_object("default_ground_plane")

        """
        for object_dict in self._all_object_dicts:
            if name in object_dict:
                del object_dict[name]
                return
        raise Exception(f"Cannot remove object {name} from the scene since it doesn't exist")

    def get_object(self, name: str) -> SingleXFormPrim:
        """Get a registered object by its name if exists otherwise None.

        Args:
            name: Object name

        Returns:
            The object if it exists otherwise None.

        Example:

        .. code-block:: python

            >>> # given a registered ground plane named 'default_ground_plane'
            >>> scene_registry.get_object("default_ground_plane")
            <isaacsim.core.api.objects.ground_plane.GroundPlane object at 0x...>

        """
        for object_dict in self._all_object_dicts:
            if name in object_dict:
                return object_dict[name]
        return None
