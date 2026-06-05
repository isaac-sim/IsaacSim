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

"""Implementation of PhysX-based proximity sensors for detecting object overlaps in Isaac Sim simulations."""

import time

import carb
import numpy as np
import omni.ext
import omni.usd
from omni.physics.core import get_physics_scene_query_interface
from omni.usd._impl.utils import get_prim_at_path, get_world_transform_matrix
from pxr import PhysicsSchemaTools, Sdf, Usd, UsdGeom


class ProximitySensor:
    """A physics-based proximity sensor that detects overlapping objects using PhysX collision queries.

    The sensor performs box overlap queries to detect when other physics objects enter, remain within,
    or exit its detection zone. It provides callback functionality for handling entry, ongoing overlap,
    and exit events, along with tracking overlap duration and distance measurements.

    The sensor uses the parent prim's scale property to define the detection box size and performs
    continuous overlap detection through the PhysX scene query interface. It maintains internal state
    to track zone transitions and provides detailed overlap metadata including duration and distance.

    Args:
        parent: The USD prim that defines the sensor's position, orientation, and scale.
            The prim's transform determines the sensor's world position and the scale property
            defines the detection box dimensions.
        callback_fns: List of three callback functions [on_enter, on_inside, on_exit].
            Each callback receives the sensor instance as a parameter. Functions can be None
            to disable specific callbacks.
        exclusions: List of prim paths to exclude from overlap detection.
            Objects at these paths will not trigger sensor events or appear in overlap data.
    """

    def __init__(self, parent: Usd.Prim, callback_fns: list | None = None, exclusions: list | None = None) -> None:
        if callback_fns is None:
            callback_fns = [None, None, None]
        if exclusions is None:
            exclusions = []
        self.parent = parent
        self._callback_fns = callback_fns
        self._exclusions = exclusions

        self._active_zones = []
        self._prev_active_zones = []
        self._entered_zones = []
        self._exited_zones = []

        self._data = {}
        self.name = str(self.parent.GetPath()).rpartition("/")[-1]

        self._is_inside = False
        self.overlapping = False
        self.distance = 0
        self.stage = omni.usd.get_context().get_stage()

    def update(self) -> None:
        """Updates the proximity sensor state by checking for overlaps and triggering callbacks.

        Checks for overlap with collision meshes, updates active zones, determines entered and exited zones,
        and calls appropriate callback functions for zone transitions and while inside zones.
        """
        # get any active zones
        self._prev_active_zones = self._active_zones
        self._active_zones = []  # clear the active zones
        num_hits = self.check_for_overlap()  # update active_zones

        # check if we have entered or exited a zone
        if len(self._active_zones) != len(self._prev_active_zones):
            entered_zones = list(set(self._active_zones) - set(self._prev_active_zones))
            if len(entered_zones) > 0:
                self._entered_zones = entered_zones
                # Pass to external on_enter callback_fn
                if self._callback_fns[0] is not None:
                    self._callback_fns[0](self)
            else:
                self._entered_zones = []
            exited_zones = list(set(self._prev_active_zones) - set(self._active_zones))
            if len(exited_zones) > 0:
                self._exited_zones = exited_zones

                # remove timers from dict on exit
                for zone in self._exited_zones:
                    self._data.pop(zone)

                # Pass to external on_exit callback_fn
                if self._callback_fns[2] is not None:
                    self._callback_fns[2](self)
            else:
                self._exited_zones = []

        is_inside = len(self._active_zones) != 0
        # Check for a state change
        if self._is_inside != is_inside:
            # Reset the Tracker if we've moved outside
            if not is_inside:
                self.reset()
            # Update _is_inside
            self._is_inside = is_inside
            self.overlapping = is_inside

        # Pass to external is_inside callback_fn
        if self._is_inside:

            # update the overlap data
            for key, val in self._data.items():
                # update timer
                val["duration"] = time.time() - val["start_time"]
                # update the distance
                pos_a = get_world_transform_matrix(self.parent).ExtractTranslation()
                prim = get_prim_at_path(Sdf.Path(key))
                pos_b = get_world_transform_matrix(prim).ExtractTranslation()
                a = np.array((pos_a[0], pos_a[1], pos_a[2]))
                b = np.array((pos_b[0], pos_b[1], pos_b[2]))
                val["distance"] = np.linalg.norm(a - b)
            if self._callback_fns[1] is not None:
                self._callback_fns[1](self)

        return

    def report_hit(self, hit: object) -> bool:
        """Reports a hit from the physics overlap query.

        Processes a collision hit by adding the collided prim to active zones and starting a timer
        for duration tracking.

        Args:
            hit: The physics hit result from the overlap query.

        Returns:
            True to continue the physics query.
        """
        prim_path = str(PhysicsSchemaTools.intToSdfPath(hit.rigid_body))
        path = str(UsdGeom.Mesh.Get(omni.usd.get_context().get_stage(), prim_path).GetPrim().GetPrimPath())
        # Add to the active zone list
        if path not in self._active_zones and path not in self._exclusions:
            self._active_zones.append(path)

        # Start the timer
        if path not in self._data and path not in self._exclusions:
            self._data[path] = {"start_time": time.time(), "duration": 0}

        return True  # return True to continue the query

    def check_for_overlap(self) -> int:
        """Performs a physics overlap box query to detect collisions.

        Uses the parent prim's transform and scale to create a box overlap query that detects
        collisions with other geometry in the physics scene.

        Returns:
            Number of hits from the overlap query.
        """
        self.parent_pose = omni.usd.get_world_transform_matrix(self.parent)
        extent = self.parent.GetPropertyAtPath(str(self.parent.GetPath()) + ".xformOp:scale").Get()
        origin = self.parent_pose.ExtractTranslation()
        rot = self.parent_pose.ExtractRotation().GetQuat()

        # 'overlap multiple' query
        return get_physics_scene_query_interface().overlap_box(
            carb.Float3(extent[0] * 0.5, extent[1] * 0.5, extent[2] * 0.5),
            carb.Float3(origin[0], origin[1], origin[2]),
            carb.Float4(rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2], rot.GetReal()),
            self.report_hit,
        )

    def status(self) -> tuple[bool, dict[str, dict[str, float]]]:
        """Current overlapping status and data.

        Returns:
            A tuple containing the overlapping boolean state and the overlap data dictionary.
        """
        return (self.overlapping, self._data)

    def reset(self) -> None:
        """Resets the proximity sensor to its initial state.

        Clears all active zones, entered zones, exited zones, and overlap data, and sets the
        internal overlapping state to false.
        """
        self._is_inside = False
        self._active_zones = []
        self._entered_zones = []
        self._exited_zones = []
        self._data.clear()

    def get_data(self) -> dict[str, dict[str, float]]:
        """Returns dictionary of overlapped geometry and respective metadata.

            key: prim_path of overlapped geometry
            val: dictionary of metadata:

                "duration": float of time since overlap
                "distance": distance from origin of tracker to origin of overlapped geometry

        Returns:
            Overlapped geometry and metadata.
        """
        return self._data

    def to_string(self) -> str:
        """String representation of the proximity sensor state.

        Returns:
            A formatted string containing the tracker path, name, and active zone information with
            duration and distance details.
        """
        msg = f"Tracker: {self.parent.GetPath()}, \tname: {self.name}, \tactive_zones: "
        for key, val in self._data.items():
            duration = f"{val['duration']:.4f}"
            distance = f"{val['distance']:.4f}"
            msg += f"({key}, duration: {duration}, distance: {distance}), "
        return msg

    def is_overlapping(self) -> bool:
        """Whether the proximity sensor is currently overlapping with any geometry.

        Returns:
            True if overlapping with any collision meshes.
        """
        return self.overlapping

    def get_active_zones(self) -> list[str]:
        """Returns a list of the prim paths of all the collision meshes the tracker is inside of.

        Returns:
            Prim paths as strings.
        """
        return self._active_zones

    def get_entered_zones(self) -> list[str]:
        """Returns a list of the prim paths of all the collision meshes the tracker just entered.

        Returns:
            Prim paths as strings.
        """
        return self._entered_zones

    def get_exited_zones(self) -> list[str]:
        """Prim paths of all the collision meshes the tracker just exited.

        Returns:
            Prim paths as strings.
        """
        return self._exited_zones


class ProximitySensorManager(object):
    """Singleton manager for proximity sensors in PhysX simulations.

    This manager provides centralized control for multiple proximity sensors, handling their
    registration, updates, and lifecycle management. It implements the singleton pattern to
    ensure a single instance manages all proximity sensors in the simulation.

    The manager maintains a list of registered sensors and provides methods to add new sensors,
    clear all sensors, and update all registered sensors in a single call. This is particularly
    useful for coordinating multiple proximity detection zones in complex simulation environments.
    """

    _instance = None
    """Singleton instance of the ProximitySensorManager class."""

    def __new__(cls: type) -> "ProximitySensorManager":
        """Creates or returns the singleton instance of ProximitySensorManager.

        Implements the singleton pattern to ensure only one instance of the manager exists.
        Initializes an empty sensors list on first instantiation.

        Returns:
            The singleton instance of ProximitySensorManager.
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls.sensors = []
        return cls._instance

    def register_sensor(self, sensor: ProximitySensor) -> None:
        """Registers a proximity sensor with the manager.

        Args:
            sensor: The proximity sensor to register for management.
        """
        self.sensors.append(sensor)

    def clear_sensors(self) -> None:
        """Removes all registered sensors from the manager.

        Clears the internal sensors list, effectively unregistering all proximity sensors.
        """
        self.sensors = []

    def update(self) -> None:
        """Updates all registered proximity sensors.

        Iterates through all registered sensors and calls their update method to check for
        overlaps and trigger callbacks.
        """
        for sensor in self.sensors:
            sensor.update()


# Public API:
def register_sensor(sensor: ProximitySensor) -> None:
    """Register a proximity sensor with the global sensor manager.

    Adds the sensor to the list of active sensors that will be updated during physics simulation.
    The sensor will begin monitoring for overlaps and triggering callbacks once registered.

    Args:
        sensor: The ProximitySensor instance to register.
    """
    ProximitySensorManager().register_sensor(sensor)


def clear_sensors() -> None:
    """Clear all registered proximity sensors from the global sensor manager.

    Removes all sensors from the active sensor list, stopping their overlap monitoring and callbacks.
    This does not destroy the sensor objects themselves, only unregisters them from the manager.
    """
    ProximitySensorManager().clear_sensors()


# Example usage, create a physics scene and a Cube first
# from isaacsim.sensors.physx import ProximitySensor, register_sensor, clear_sensors
# import carb
# import omni
# half_extent = carb.Float3(25,25,25)
# stage = omni.usd.get_context().get_stage()
# parent = stage.GetPrimAtPath("/World/Cube")
# s = ProximitySensor(half_extent, parent)
# register_sensor(s)
