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

"""Deprecated camera utility functions."""

import omni
from isaacsim.core.utils.rotations import lookat_to_quatf
from pxr import Gf, Usd, UsdGeom


class SpringDamperFollower:
    """A spring-damper system for smooth interpolation between current and target positions.

    This class implements a mass-spring-damper system that provides smooth, physics-based transitions
    between a current position and a target position. It is commonly used for camera movement,
    object tracking, and other scenarios requiring smooth motion interpolation.

    The system uses the equation: acceleration = (k * displacement - c * velocity) / m,
    where displacement is the difference between target and current positions.

    Args:
        mass: Mass of the system, affecting inertia and response time.
        stiffness: Spring stiffness coefficient, controlling how strongly the system pulls toward the target.
        damping: Damping coefficient, controlling oscillation reduction and settling behavior.
        current: Current position as a 3D vector.
        target: Target position as a 3D vector.
        vel: Current velocity as a 3D vector.
    """

    def __init__(
        self,
        mass: float,
        stiffness: float,
        damping: float,
        current: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        target: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        vel: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
    ) -> None:
        self.m = mass
        self.k = stiffness
        self.c = damping
        self.current = current
        self.target = target
        self.v = vel

    def update(self, step: float) -> None:
        """Updates the spring-damper system position and velocity for one simulation step.

        Args:
            step: Time step for the simulation update.
        """
        d = self.target - self.current
        if self.m == 0:
            return
        a = (self.k * d - self.c * self.v) / self.m
        self.v = self.v + a * step
        self.current = self.current + self.v * step


class DynamicCamera:
    """A dynamic camera system that provides smooth camera movements using spring-damper physics.

    This class creates a USD camera with physically-based motion control using spring-damper systems
    for position, look target, and focus distance. The camera movements are smooth and natural,
    with configurable mass, stiffness, and damping parameters for different motion characteristics.

    The camera is created as a proxy transform with the actual camera as a child prim, allowing
    for flexible positioning and orientation control. The system supports automatic focus adjustment
    based on the distance to the look target.

    Args:
        stage: The USD stage where the camera will be created.
        base_path: The USD path where the camera proxy will be created.
        camera_name: Name for the camera prim and proxy.
        focal_length: Camera focal length in millimeters.
        f_stop: Camera f-stop value for depth of field.
        focus_distance: Initial focus distance for the camera.
    """

    def __init__(
        self,
        stage: Usd.Stage,
        base_path: str,
        camera_name: str,
        focal_length: float = 24,
        f_stop: float = 5,
        focus_distance: float = 0,
    ) -> None:
        self._stage = stage
        self.target_follower = SpringDamperFollower(mass=5, stiffness=5, damping=10)
        self.position_follower = SpringDamperFollower(mass=20, stiffness=5, damping=20)
        self.focus_follower = SpringDamperFollower(mass=1, stiffness=10, damping=10, current=10000, target=10000, vel=0)
        self._base_path = base_path
        self._camera_path = base_path + "/" + camera_name
        self.thresh = 0.1

        self.proxy = self._stage.DefinePrim(self._base_path + "/" + camera_name + "_proxy", "Xform")
        xform = UsdGeom.Xformable(self.proxy)
        xform.ClearXformOpOrder()
        xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        self._timeline = omni.timeline.get_timeline_interface()

        self.prim = self._stage.DefinePrim(base_path + "/" + camera_name + "_proxy/" + camera_name, "Camera")
        self.prim.GetAttribute("focalLength").Set(focal_length)
        self.prim.GetAttribute("fStop").Set(float(f_stop))
        self.prim.GetAttribute("focusDistance").Set(float(focus_distance))
        self.focus = False

    def reset(self) -> None:
        """Resets the camera to its target positions immediately.

        Sets both position and look target followers to their current target values,
        effectively eliminating any spring motion and placing the camera at its intended position.
        """
        self.position_follower.current = self.position_follower.target
        self.target_follower.current = self.target_follower.target
        self.update(1.0 / 60.0)

    def update(self, step: float, timecode: Usd.TimeCode = Usd.TimeCode.Default()) -> None:
        """Updates the camera position, orientation, and focus based on spring-damper physics.

        Args:
            step: Time step for the physics simulation.
            timecode: USD time code for setting animated attributes.
        """
        self.target_follower.update(step)
        self.position_follower.update(step)

        pos = self.position_follower.current
        target = self.target_follower.current

        orient = lookat_to_quatf(target, pos, Gf.Vec3d(0, 0, 1))
        mat = Gf.Matrix4d().SetRotateOnly(orient).SetTranslateOnly(pos)
        # mat_1 = Gf.Matrix4d().SetLookAt(self.position_follower.current, self.target_follower.current, Gf.Vec3d(0, 0, 1))
        # trans = mat_1.ExtractTranslation()
        # mat_1.SetTranslateOnly(Gf.Vec3d(trans[2], trans[0], -trans[1]))
        self.proxy.GetAttribute("xformOp:transform").Set(mat, timecode)

        if self.focus:
            self.focus_follower.target = float((target - pos).GetLength())
        else:
            self.focus_follower.target = 10000
        self.focus_follower.update(step)
        # print("focal", self.focus_follower.current)
        self.prim.GetAttribute("focusDistance").Set(float(self.focus_follower.current), timecode)

    def set_look_target(self, pos: Gf.Vec3d) -> None:
        """Sets the target position for the camera to look at.

        Args:
            pos: Target position in 3D space for the camera to look towards.
        """
        self.target_follower.target = pos

    def set_pos_target(self, pos: Gf.Vec3d) -> None:
        """Sets the target position for the camera location.

        Args:
            pos: Target position in 3D space for the camera to move towards.
        """
        self.position_follower.target = pos

    def set_autofocus_target(self, focus: bool) -> None:
        """Enables or disables automatic focus based on distance to look target.

        Args:
            focus: Whether to automatically adjust focus distance based on the distance to the look target.
        """
        self.focus = focus

    def set_pos_settings(self, mass: float, stiffness: float, damping: float) -> None:
        """Configures the spring-damper parameters for camera position movement.

        Args:
            mass: Mass parameter for the position spring-damper system.
            stiffness: Stiffness parameter for the position spring-damper system.
            damping: Damping parameter for the position spring-damper system.
        """
        self.position_follower.m = mass
        self.position_follower.k = stiffness
        self.position_follower.c = damping

    def set_target_settings(self, mass: float, stiffness: float, damping: float) -> None:
        """Configures the spring-damper parameters for camera look target movement.

        Args:
            mass: Mass parameter for the look target spring-damper system.
            stiffness: Stiffness parameter for the look target spring-damper system.
            damping: Damping parameter for the look target spring-damper system.
        """
        self.target_follower.m = mass
        self.target_follower.k = stiffness
        self.target_follower.c = damping
